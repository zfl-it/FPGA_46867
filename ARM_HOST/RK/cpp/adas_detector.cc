// adas_detector.cc — 见 adas_detector.h
#include "adas_detector.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// ---------------------------------------------------------------- 工具
static const char *kClassNames[ADAS_NC] = {
    "left_red", "left_green", "right_red", "right_green",
    "straight_red", "straight_green", "dash", "solid",
    "pedestrian", "crosswalk", "disc_red", "disc_green"};

const char *adasClassName(int cls)
{
    if (cls < 0 || cls >= ADAS_NC)
        return "unknown";
    return kClassNames[cls];
}

// 模型输出的类别置信度被预先乘了 kScoreScale（见 work/scale_scores.py）。
//
// 原因：output0 = concat(框坐标 0~1280, 置信度 0~1)，RKNN 对它做 per-tensor int8，
// 步长 = 1280/255 ≈ 5，0~1 的置信度会被全部舍入成 0（已实测：10 个置信度通道恒为 0）。
// 导出 ONNX 前在 Sigmoid 后插入了 ×1280，使 concat 后各通道量程一致，
// 因此解码时要除回来。混合量化无法解决（rknn 会强制上游 Div_1 转 fp16 并报错）。
static constexpr float kScoreScale = 1280.0f;

static bool readFileBytes(const std::string &path, std::vector<unsigned char> &out)
{
    std::ifstream f(path, std::ios::binary);
    if (!f)
        return false;
    f.seekg(0, std::ios::end);
    std::streamoff n = f.tellg();
    if (n <= 0)
        return false;
    out.resize(static_cast<size_t>(n));
    f.seekg(0, std::ios::beg);
    f.read(reinterpret_cast<char *>(out.data()), n);
    return true;
}

// ---------------------------------------------------------------- AdasDetector
AdasDetector::~AdasDetector()
{
    if (ctx_ != 0)
    {
        if (in_mem_ != nullptr)
        {
            rknn_destroy_mem(ctx_, in_mem_);
            in_mem_ = nullptr;
        }
        rknn_destroy(ctx_);
        ctx_ = 0;
    }
    ready_ = false;
}

bool AdasDetector::init(const std::string &model_path)
{
    std::vector<unsigned char> data;
    if (!readFileBytes(model_path, data))
    {
        std::cerr << "[ADAS] 读取模型失败: " << model_path << std::endl;
        return false;
    }

    int ret = rknn_init(&ctx_, data.data(), static_cast<uint32_t>(data.size()), 0, nullptr);
    if (ret < 0)
    {
        std::cerr << "[ADAS] rknn_init 失败, ret=" << ret << std::endl;
        ctx_ = 0;
        return false;
    }

    rknn_sdk_version ver{};
    if (rknn_query(ctx_, RKNN_QUERY_SDK_VERSION, &ver, sizeof(ver)) == RKNN_SUCC)
        std::cout << "[ADAS] RKNN api=" << ver.api_version << " drv=" << ver.drv_version << std::endl;

    rknn_input_output_num io{};
    if (rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io, sizeof(io)) != RKNN_SUCC)
    {
        std::cerr << "[ADAS] 查询输入输出数量失败" << std::endl;
        return false;
    }

    rknn_tensor_attr in{};
    in.index = 0;
    if (rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &in, sizeof(in)) != RKNN_SUCC)
    {
        std::cerr << "[ADAS] 查询输入属性失败" << std::endl;
        return false;
    }

    if (in.fmt == RKNN_TENSOR_NCHW)
    {
        model_channel_ = in.dims[1];
        model_height_ = in.dims[2];
        model_width_ = in.dims[3];
    }
    else
    {
        model_height_ = in.dims[1];
        model_width_ = in.dims[2];
        model_channel_ = in.dims[3];
    }

    out_attrs_.resize(io.n_output);
    for (uint32_t i = 0; i < io.n_output; ++i)
    {
        std::memset(&out_attrs_[i], 0, sizeof(rknn_tensor_attr));
        out_attrs_[i].index = i;
        if (rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &out_attrs_[i], sizeof(rknn_tensor_attr)) != RKNN_SUCC)
        {
            std::cerr << "[ADAS] 查询输出属性失败 idx=" << i << std::endl;
            return false;
        }
    }

    std::cout << "[ADAS] 模型 " << model_path
              << " 输入 " << model_width_ << "x" << model_height_ << "x" << model_channel_
              << " 输出数 " << io.n_output << std::endl;

    // ---- NPU 零拷贝输入：把 RGB 预处理结果直接写进 NPU 可见缓冲 ----
    in_attr_ = in;
    in_attr_.index = 0;
    const uint32_t in_bytes = in.size_with_stride > 0
                                  ? in.size_with_stride
                                  : static_cast<uint32_t>(in.n_elems);
    in_mem_ = rknn_create_mem(ctx_, in_bytes);
    if (in_mem_ != nullptr && in_mem_->virt_addr != nullptr)
    {
        // pass_through=1：驱动不做任何转换/拷贝，直接把这块内存当输入
        rknn_tensor_attr io_attr = in_attr_;
        io_attr.pass_through = 1;
        io_attr.type = RKNN_TENSOR_UINT8;
        io_attr.fmt = RKNN_TENSOR_NHWC;
        if (rknn_set_io_mem(ctx_, in_mem_, &io_attr) == RKNN_SUCC)
        {
            zero_copy_ = true;
            std::cout << "[ADAS] 零拷贝输入已启用 (" << in_bytes << " B, w_stride="
                      << in_attr_.w_stride << ")" << std::endl;
        }
        else
        {
            rknn_destroy_mem(ctx_, in_mem_);
            in_mem_ = nullptr;
            std::cout << "[ADAS] 零拷贝不可用，回退 rknn_inputs_set 路径" << std::endl;
        }
    }

    ready_ = true;
    return true;
}

AdasDetector::Letterbox AdasDetector::computeLetterbox(int img_w, int img_h) const
{
    Letterbox lb;
    lb.scale = std::min(static_cast<float>(model_height_) / img_h,
                        static_cast<float>(model_width_) / img_w);
    lb.new_w = static_cast<int>(img_w * lb.scale);
    lb.new_h = static_cast<int>(img_h * lb.scale);
    lb.left = (model_width_ - lb.new_w) / 2;
    lb.top = (model_height_ - lb.new_h) / 2;
    return lb;
}

// 从 (1, C, N) 或 (1, N, C) 中推断 rows/cols
static bool inferShape(const rknn_tensor_attr &attr, int &rows, int &cols, bool &transposed)
{
    std::vector<int> dims;
    for (int i = 0; i < attr.n_dims; ++i)
        if (attr.dims[i] > 1)
            dims.push_back(attr.dims[i]);
    if (dims.size() < 2)
        return false;
    int a = dims[dims.size() - 2];
    int b = dims[dims.size() - 1];
    if (b >= 5 && b <= 128)
    {
        rows = a;
        cols = b;
        transposed = false;
        return true;
    }
    if (a >= 5 && a <= 128)
    {
        rows = b;
        cols = a;
        transposed = true;
        return true;
    }
    return false;
}

bool AdasDetector::detect(const cv::Mat &bgr,
                          std::vector<AdasDetection> &out,
                          float conf_thres,
                          float nms_thres)
{
    out.clear();
    if (!ready_ || bgr.empty())
        return false;

    Letterbox lb = computeLetterbox(bgr.cols, bgr.rows);

    // 复用 canvas / rgb 缓冲：尺寸不变时 create() 不会重新分配
    if (lb_canvas_.rows != model_height_ || lb_canvas_.cols != model_width_ ||
        lb_canvas_.type() != CV_8UC3)
        lb_canvas_.create(model_height_, model_width_, CV_8UC3);

    const bool no_pad = (lb.new_w == model_width_ && lb.new_h == model_height_ &&
                         lb.left == 0 && lb.top == 0);
    if (no_pad)
    {
        cv::resize(bgr, lb_canvas_, lb_canvas_.size(), 0, 0, cv::INTER_LINEAR);
    }
    else
    {
        lb_canvas_.setTo(cv::Scalar(114, 114, 114));
        // 直接缩放进 ROI，省掉一次 2.8MB 的 resized->canvas 整幅拷贝
        cv::Mat roi = lb_canvas_(cv::Rect(lb.left, lb.top, lb.new_w, lb.new_h));
        cv::resize(bgr, roi, roi.size(), 0, 0, cv::INTER_LINEAR);
    }

    if (zero_copy_ && in_mem_ != nullptr && in_mem_->virt_addr != nullptr)
    {
        // 让 lb_rgb_ 直接指向 NPU 输入缓冲（按 w_stride 行距），
        // cvtColor 一次写进 NPU 可见内存，后面 rknn_run 无需再拷一遍。
        const int w_stride = in_attr_.w_stride > 0 ? in_attr_.w_stride : model_width_;
        lb_rgb_ = cv::Mat(model_height_, model_width_, CV_8UC3,
                          in_mem_->virt_addr,
                          static_cast<size_t>(w_stride) * model_channel_);
    }
    else if (lb_rgb_.rows != model_height_ || lb_rgb_.cols != model_width_ ||
             lb_rgb_.type() != CV_8UC3)
        lb_rgb_.create(model_height_, model_width_, CV_8UC3);
    cv::cvtColor(lb_canvas_, lb_rgb_, cv::COLOR_BGR2RGB);
    const cv::Mat &rgb = lb_rgb_;

    if (!zero_copy_)
    {
        rknn_input input;
        std::memset(&input, 0, sizeof(input));
        input.index = 0;
        input.type = RKNN_TENSOR_UINT8;
        input.fmt = RKNN_TENSOR_NHWC;
        input.size = static_cast<uint32_t>(rgb.total() * rgb.elemSize());
        input.buf = rgb.data;

        if (rknn_inputs_set(ctx_, 1, &input) < 0)
            return false;
    }
    if (rknn_run(ctx_, nullptr) < 0)
        return false;

    size_t n_out = out_attrs_.size();
    std::vector<rknn_output> rk_out(n_out);
    std::memset(rk_out.data(), 0, n_out * sizeof(rknn_output));
    for (size_t i = 0; i < n_out; ++i)
    {
        rk_out[i].index = static_cast<uint32_t>(i);
        rk_out[i].want_float = 1;
    }
    if (rknn_outputs_get(ctx_, static_cast<uint32_t>(n_out), rk_out.data(), nullptr) < 0)
        return false;

    out_bufs_.assign(n_out, std::vector<float>());
    for (size_t i = 0; i < n_out; ++i)
    {
        int elems = out_attrs_[i].n_elems;
        out_bufs_[i].resize(elems);
        std::memcpy(out_bufs_[i].data(), rk_out[i].buf, elems * sizeof(float));
    }
    rknn_outputs_release(ctx_, static_cast<uint32_t>(n_out), rk_out.data());

    // ---- 解码：输出布局 (1, 4+nc, N) -> transposed=true, rows=N, cols=4+nc
    const std::vector<float> &pred = out_bufs_[0];
    int rows = 0, cols = 0;
    bool transposed = false;
    if (!inferShape(out_attrs_[0], rows, cols, transposed))
        return false;
    if (cols < 4 + ADAS_NC)
    {
        std::cerr << "[ADAS] 输出列数 " << cols << " 小于 4+" << ADAS_NC << std::endl;
        return false;
    }

    auto at = [&](int r, int c) -> float
    {
        size_t idx = transposed ? (static_cast<size_t>(c) * rows + r)
                                : (static_cast<size_t>(r) * cols + c);
        return idx < pred.size() ? pred[idx] : 0.f;
    };

    std::vector<AdasDetection> cand;
    for (int i = 0; i < rows; ++i)
    {
        float best = -1.f;
        int bestc = 0;
        for (int c = 4; c < 4 + ADAS_NC; ++c)
        {
            float v = at(i, c) / kScoreScale;   // 置信度已预乘 1280，除回来
            if (v > best)
            {
                best = v;
                bestc = c - 4;
            }
        }
        if (best < conf_thres)
            continue;

        float cx = at(i, 0), cy = at(i, 1), w = at(i, 2), h = at(i, 3);
        if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(w) || !std::isfinite(h))
            continue;

        // letterbox 空间 -> 原图坐标
        float x0 = (cx - 0.5f * w - lb.left) / lb.scale;
        float y0 = (cy - 0.5f * h - lb.top) / lb.scale;
        float x1 = (cx + 0.5f * w - lb.left) / lb.scale;
        float y1 = (cy + 0.5f * h - lb.top) / lb.scale;

        cv::Rect box(static_cast<int>(x0), static_cast<int>(y0),
                     static_cast<int>(x1 - x0), static_cast<int>(y1 - y0));
        box &= cv::Rect(0, 0, bgr.cols, bgr.rows);
        if (box.area() <= 0)
            continue;

        AdasDetection d;
        d.box = box;
        d.cls = bestc;
        d.score = best;
        cand.push_back(d);
    }

    // ---- 按类别 NMS
    for (int c = 0; c < ADAS_NC; ++c)
    {
        std::vector<AdasDetection> by_cls;
        for (const auto &d : cand)
            if (d.cls == c)
                by_cls.push_back(d);
        if (by_cls.empty())
            continue;

        std::sort(by_cls.begin(), by_cls.end(),
                  [](const AdasDetection &a, const AdasDetection &b) { return a.score > b.score; });

        std::vector<bool> suppressed(by_cls.size(), false);
        for (size_t i = 0; i < by_cls.size(); ++i)
        {
            if (suppressed[i])
                continue;
            out.push_back(by_cls[i]);
            for (size_t j = i + 1; j < by_cls.size(); ++j)
            {
                if (suppressed[j])
                    continue;
                float inter = static_cast<float>((by_cls[i].box & by_cls[j].box).area());
                float uni = static_cast<float>(by_cls[i].box.area() + by_cls[j].box.area()) - inter;
                if (uni > 0.f && inter / uni > nms_thres)
                    suppressed[j] = true;
            }
        }
    }

    return true;
}

// ---------------------------------------------------------------- 信号映射
static float maxScoreOf(const std::vector<AdasDetection> &dets, int cls)
{
    float m = 0.f;
    for (const auto &d : dets)
        if (d.cls == cls && d.score > m)
            m = d.score;
    return m;
}

std::string AdasSignals::bits() const
{
    char b[7];
    b[0] = left_line_solid ? '1' : '0';
    b[1] = right_line_solid ? '1' : '0';
    b[2] = straight_green ? '1' : '0';
    b[3] = left_green ? '1' : '0';
    b[4] = right_green ? '1' : '0';
    b[5] = crosswalk_ahead ? '1' : '0';
    b[6] = '\0';
    return std::string(b);
}

std::string AdasSignals::frame() const
{
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%s %d", bits().c_str(), ped_count);
    return std::string(buf);
}

AdasSignals mapAdasSignals(const std::vector<AdasDetection> &dets, int frame_w)
{
    AdasSignals s;

    // 车道线：实线框按中心 x 归属左右
    const int mid = frame_w / 2;
    for (const auto &d : dets)
    {
        if (d.cls != ADAS_SOLID)
            continue;
        int cx = d.box.x + d.box.width / 2;
        if (cx < mid)
            s.left_line_solid = true;
        else
            s.right_line_solid = true;
    }

    // 信号灯判定：每个方向独立处理三种情况
    //   ① 该方向红、绿都没识别到（画面里没有这组灯）→ 按绿灯放行(1)
    //   ② 只识别到其中一个 → 以识别到的为准
    //   ③ 红绿都识别到 → 取分数高者，避免同时为真
    // ①的默认策略很关键：路口往往只有部分方向有箭头灯，若把"没识别到"当成红灯，
    // 车会在有通行权时被误判禁止通行而卡死。
    auto lightGreen = [&](int green_cls, int red_cls) -> bool {
        const float g = maxScoreOf(dets, green_cls);
        const float r = maxScoreOf(dets, red_cls);
        if (g <= 0.f && r <= 0.f)
            return true;          // 未识别到该方向的灯 -> 默认绿灯
        return g >= r;            // 有识别 -> 分数高者说了算
    };
    s.left_green = lightGreen(ADAS_LEFT_GREEN, ADAS_LEFT_RED);
    s.right_green = lightGreen(ADAS_RIGHT_GREEN, ADAS_RIGHT_RED);
    s.straight_green = lightGreen(ADAS_STRAIGHT_GREEN, ADAS_STRAIGHT_RED);

    // 圆盘灯（BL-3 新增）优先于箭头灯：
    //   圆盘绿灯 -> 直行 / 左转 / 右转 三路全部放行
    //   圆盘红灯 -> 禁止直行与左转，右转放行
    // 只有识别到圆盘灯时才覆盖上面的箭头灯结论；红绿同时出现取分数高者。
    const float disc_g = maxScoreOf(dets, ADAS_DISC_GREEN);
    const float disc_r = maxScoreOf(dets, ADAS_DISC_RED);
    if (disc_g > 0.f || disc_r > 0.f)
    {
        if (disc_g >= disc_r)
        {
            s.straight_green = true;
            s.left_green = true;
            s.right_green = true;
        }
        else
        {
            s.straight_green = false;
            s.left_green = false;
            s.right_green = true;
        }
    }

    for (const auto &d : dets)
        if (d.cls == ADAS_CROSSWALK)
            s.crosswalk_ahead = true;

    int ped = 0;
    for (const auto &d : dets)
        if (d.cls == ADAS_PED)
            ++ped;
    // 与上位机 PED_COUNT_MAX=999 对齐，避免异常帧把数字撑爆
    s.ped_count = std::min(ped, 999);

    return s;
}

// ---------------------------------------------------------------- 画框
void drawAdasResults(cv::Mat &bgr,
                     const std::vector<AdasDetection> &dets,
                     const AdasSignals &sig)
{
    static const cv::Scalar kColors[ADAS_NC] = {
        cv::Scalar(60, 60, 220),   // 左转红
        cv::Scalar(60, 200, 60),   // 左转绿
        cv::Scalar(60, 140, 240),  // 右转红
        cv::Scalar(100, 220, 100), // 右转绿
        cv::Scalar(40, 40, 200),   // 直行红
        cv::Scalar(80, 230, 80),   // 直行绿
        cv::Scalar(220, 200, 60),  // 虚线
        cv::Scalar(40, 200, 230),  // 实线
        cv::Scalar(230, 90, 200),  // 行人
        cv::Scalar(180, 180, 180), // 斑马线
        cv::Scalar(50, 50, 230),   // 圆盘红
        cv::Scalar(90, 230, 90)    // 圆盘绿
    };

    for (const auto &d : dets)
    {
        cv::Scalar c = (d.cls >= 0 && d.cls < ADAS_NC) ? kColors[d.cls] : cv::Scalar(255, 255, 255);
        cv::rectangle(bgr, d.box, c, 2);
        char label[64];
        std::snprintf(label, sizeof(label), "%s %.2f", adasClassName(d.cls), d.score);
        cv::putText(bgr, label, cv::Point(d.box.x, std::max(d.box.y - 4, 12)),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, c, 1, cv::LINE_AA);
    }

    // 左上角状态条
    std::string status = "ADAS " + sig.frame();
    cv::rectangle(bgr, cv::Rect(8, 8, 190, 26), cv::Scalar(0, 0, 0), -1);
    cv::putText(bgr, status, cv::Point(14, 26), cv::FONT_HERSHEY_SIMPLEX,
                0.55, cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
}

// ---------------------------------------------------------------- 串口
AdasSerialSender::~AdasSerialSender()
{
    stop();
    if (fd_ >= 0)
    {
        ::close(fd_);
        fd_ = -1;
    }
}

bool AdasSerialSender::open(const std::string &dev, int baud)
{
    fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0)
    {
        std::cerr << "[ADAS] 打开串口失败: " << dev << std::endl;
        return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0)
    {
        std::cerr << "[ADAS] tcgetattr 失败" << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    speed_t sp = B115200;
    switch (baud)
    {
    case 9600: sp = B9600; break;
    case 19200: sp = B19200; break;
    case 38400: sp = B38400; break;
    case 57600: sp = B57600; break;
    case 115200: sp = B115200; break;
    case 230400: sp = B230400; break;
    case 460800: sp = B460800; break;
    default: sp = B115200; break;
    }

    cfsetospeed(&tty, sp);
    cfsetispeed(&tty, sp);
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;      // 8 数据位
    tty.c_cflag &= ~PARENB;  // 无校验
    tty.c_cflag &= ~CSTOPB;  // 1 停止位
    tty.c_cflag &= ~CRTSCTS; // 无流控
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
        std::cerr << "[ADAS] tcsetattr 失败" << std::endl;
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    tcflush(fd_, TCIOFLUSH);

    std::cout << "[ADAS] 串口已打开 " << dev << " @" << baud << " 8N1" << std::endl;
    return true;
}

void AdasSerialSender::publish(const AdasSignals &sig)
{
    std::lock_guard<std::mutex> lk(mtx_);
    sig_ = sig;
    dirty_ = true;
}

void AdasSerialSender::start(int period_ms)
{
    if (running_)
        return;
    running_ = true;
    th_ = std::thread(&AdasSerialSender::loop, this, period_ms);
}

void AdasSerialSender::stop()
{
    if (!running_)
        return;
    running_ = false;
    if (th_.joinable())
        th_.join();
}

void AdasSerialSender::loop(int period_ms)
{
    while (running_)
    {
        AdasSignals s;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            s = sig_;
        }
        std::string line = s.frame() + "\n";
        if (fd_ >= 0)
        {
            ssize_t n = ::write(fd_, line.c_str(), line.size());
            if (n < 0)
            {
                // 非阻塞写失败通常意味着对端未就绪，忽略即可
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
    }
}
