// rk_yolo_drm_fpga_hdmi_lpr_web.cpp  —  V5 latency-optimised build
// Continuous 25 FPS PCIe + independent inference snapshot build.
// 新功能链路：
//   PCIe RGB565_LE视频输入 -> yolov5n_i8.rknn (640x640 INT8) 检测车牌位置
//   -> RK裁剪车牌ROI -> lprnet_color_fp16.rknn (128x48 FP16, exp6) OCR识别字符+颜色
//   -> MPP H.264硬编码 -> ZLMediaKit输出检测框和识别结果
//
// 说明：
// 1) 默认 --fpga-mode sw-edge 使用OpenCV Sobel模拟FPGA输出，先验证软件流程。
// 2) 真正接双HDMI链路时使用 --fpga-mode hdmi。
// 3) 编译HDMI/DRM模式需要定义 HAVE_DRM_ROI 并链接 libdrm：-DHAVE_DRM_ROI -ldrm。
// 4) 两块采集卡必须分开：--video 是相机输入；--fpga-video 是FPGA返回边缘图输入。
//
// 保留原代码必要功能：
// 1) PCIe RGB565_LE输入
// 2) RKNN检测模型 + OCR识别模型
// 3) ZLMediaKit WebRTC / HTTP-FLV / RTSP 输出最终标注画面
// 4) /status 显示车牌、采集FPS、推理FPS、编码FPS、分辨率等状态
//
// V4 连续采集版核心改动：
// 1) PCIe DMA按25 FPS连续采集，不再与RKNN推理、颜色转换分时互斥。
// 2) RKNN只按 --infer-fps 限频运行，使用从已发布BGR帧深拷贝出的独立快照。
// 3) 检测框在推理线程中画到"该次推理的原始帧"，编码线程不再把旧结果画到新帧。
// 4) H.264按 --output-fps 固定输出；等待下一张推理帧时重复上一张已对齐帧。
// 5) 重复输出的帧复用NV12转换结果，避免每个输出节拍重复做OpenCV颜色转换。
// 6) 永远只处理最新帧，旧帧直接丢弃，避免队列堆积。
// 7) 网页视频输出层使用MPP H.264 + ZLMediaKit，不再提供MJPEG。
// 8) PCIe RGB565输入使用16 MiB四槽环形缓冲，每个槽位在转换完成前不会被覆盖。
//
// V5 低延迟优化 (vs V4):
// 1) 推理帧到达即处理：不再等待固定 --infer-fps 节拍，冷却期结束后立即推理。
// 2) 编码线程条件唤醒：推理完成立即通知编码器，不再等待固定输出节拍。
// 3) 消除 aligned_frame 深拷贝：推理帧直接绘制标注后移动所有权。
// 4) GOP 默认值改为 10（原为 output_fps），加快客户端首帧/恢复延迟。

#include <opencv2/opencv.hpp>

#include "adas_detector.h"

#ifdef HAVE_OPENCV_FREETYPE
#include <opencv2/freetype.hpp>
#endif

#ifdef HAVE_DIRECT_FREETYPE
#include <ft2build.h>
#include FT_FREETYPE_H
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <ctime>
#include <cctype>
#include <cstdint>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <arpa/inet.h>
#include <dlfcn.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <netinet/tcp.h>

#ifdef HAVE_DRM_ROI
#if __has_include(<drm.h>)
#include <drm.h>
#else
#include "/usr/include/libdrm/drm.h"
#endif
#if __has_include(<drm_mode.h>)
#include <drm_mode.h>
#else
#include "/usr/include/libdrm/drm_mode.h"
#endif
#if __has_include(<drm_fourcc.h>)
#include <drm_fourcc.h>
#else
#include "/usr/include/libdrm/drm_fourcc.h"
#endif
#if __has_include(<xf86drm.h>)
#include <xf86drm.h>
#else
#include "/usr/include/xf86drm.h"
#endif
#if __has_include(<xf86drmMode.h>)
#include <xf86drmMode.h>
#else
#include "/usr/include/xf86drmMode.h"
#endif
#endif

#include "rknn_api.h"
#include "pango_fun.h"
#include "mpp_h264_encoder.h"
#include "pixel_convert.h"
#include "zlm_streamer.h"

using namespace cv;
using namespace std;

static const int DEFAULT_HTTP_PORT = 8080;
static const int DEFAULT_INFER_FPS = 25;
static const int DEFAULT_OUTPUT_FPS = 25;
static atomic<bool> g_running(true);
static int g_server_fd = -1;

static mutex g_input_mutex;
static condition_variable g_input_cv;
static Mat g_input_frame;
static uint64_t g_input_frame_id = 0;

// The encoder consumes this immutable, already-annotated frame.  Its id is the
// exact g_input_frame_id that was passed to RKNN, so boxes cannot drift onto a
// newer PCIe frame.
static mutex g_aligned_output_mutex;
static condition_variable g_aligned_output_cv;
static Mat g_aligned_output_frame;
static uint64_t g_aligned_output_frame_id = 0;

// ---- V6：画面与推理解耦（ADAS 专用）------------------------------------------
// 开启时编码线程不再等「推理完成的对齐帧」，而是直接发「最新采集帧」，并把最近
// 一次的检测结果重绘上去。效果：
//   * 画面帧率 8.5fps -> 25fps（跑满编码器），端到端帧龄 ~250ms -> ~40ms
//   * 识别链路一行未改：推理线程取帧/模型/阈值/NMS/映射/串口全部原样，串口位串
//     逐帧一致（只额外保存了一份结果快照给编码线程重绘）
//   * 唯一代价：画面上的框相对画面滞后 0~一次推理耗时（约 120ms）
// 关闭方式：命令行 --latest-frame off（等价于旧的「框与画面严格同帧」行为）。
// 注意：仅在 ADAS 模式下生效；车牌模式的 aligned-frame 逻辑完全不受影响。
static atomic<bool> g_send_latest_frame(true);
static mutex g_last_adas_mutex;
static vector<AdasDetection> g_last_adas_dets;   // 最近一次推理的检测框（原图坐标）
static AdasSignals g_last_adas_sig;              // 与之配套的信号快照
static uint64_t g_last_adas_frame_id = 0;        // 结果对应的采集帧号（诊断用）
static bool g_last_adas_valid = false;

static atomic<uint64_t> g_capture_count(0);
static atomic<uint64_t> g_infer_count(0);
// ZLM mode: stream_count counts H.264 packets accepted by ZLMediaKit.
static atomic<uint64_t> g_stream_count(0);
// ZLM mode: encode_count counts output video frames whose H.264 packets were accepted by ZLMediaKit.
static atomic<uint64_t> g_encode_count(0);
static atomic<uint64_t> g_drop_count(0);

static atomic<int> g_input_width(0);
static atomic<int> g_input_height(0);
static atomic<int> g_output_width(0);
static atomic<int> g_output_height(0);
static atomic<int> g_target_infer_fps(DEFAULT_INFER_FPS);
static atomic<int> g_target_output_fps(DEFAULT_OUTPUT_FPS);
static atomic<int64_t> g_latency_ring_acquire_us(0);
static atomic<int64_t> g_latency_convert_us(0);
static atomic<int64_t> g_frame_gen_us(0);   // frame gen timestamp (system_clock epoch us)
static atomic<int64_t> g_frame_delay_us(0);  // gen-to-send delay in us
static atomic<int64_t> g_latency_infer_wait_us(0);
static atomic<int64_t> g_latency_infer_us(0);
static atomic<int64_t> g_latency_encode_wait_us(0);
static atomic<int64_t> g_latency_encode_us(0);
static atomic<int64_t> g_mpp_pts_delay_us(0);
// 帧号 -> 采集时刻（system_clock epoch us）的环形表，128 项足够覆盖 4 秒。
// 用来算「真正被推出去的那一帧」有多旧：g_frame_gen_us 记录的是「最新采集帧」的
// 时间，而推流发的是「上一次推理完的对齐帧」，两者在推理慢时相差 200ms 以上，
// 所以旧的 [DIAG] delay_us 严重偏小，不能用来衡量端到端延迟。
static atomic<int64_t> g_cap_ts_ring[128];
static atomic<int64_t> g_aligned_gen_us(0);   // 当前对齐帧的真实采集时刻
static atomic<bool> g_yolo_only(false);
static atomic<bool> g_profile_yolo(false);
static atomic<bool> g_quiet_pcie(false);

enum class OcrBackend
{
    LPRNET,
    PLATE_COLOR
};

static const vector<string> PLATE_COLOR_CHARS = {
    "#",
    "京", "沪", "津", "渝", "冀", "晋", "蒙", "辽", "吉", "黑",
    "苏", "浙", "皖", "闽", "赣", "鲁", "豫", "鄂", "湘", "粤",
    "桂", "琼", "川", "贵", "云", "藏", "陕", "甘", "青", "宁", "新",
    "学", "警", "港", "澳", "挂", "使", "领", "民", "航", "危",
    "0", "1", "2", "3", "4", "5", "6", "7", "8", "9",
    "A", "B", "C", "D", "E", "F", "G", "H", "J", "K", "L", "M",
    "N", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z",
    "险", "品"};

static const vector<string> PLATE_COLORS = {
    "黑色", "蓝色", "绿色", "白色", "黄色"};

// V2 颜色映射 (sigmoid 多标签, 与训练时 COLORS 一致)
// indices: 0=蓝(blue), 1=黄(yellow), 2=绿(green), 3=黑(black), 4=白(white)
static const vector<string> PLATE_COLORS_V2 = {"蓝色", "黄色", "绿色", "黑色", "白色"};

// Decode V2 color output: 5-class multi-label sigmoid
static vector<string> splitUtf8Chars(const string &text);

// Color head (LPRNetNPU_v2): 5-class single-label, argmax over logits.
// Mapping: 0=蓝色 1=黄色 2=绿色 3=黑色 4=白色
// NOTE: a plate-format prior was removed (2026-08-18). It restricted the
// allowed colors by the recognized text, but text recognition is far harder
// than color recognition, so OCR errors fed wrong format info into the color
// decision and hurt accuracy. Pure argmax over the color head wins.
// out_score = softmax probability over all 5 classes.
static string decodeColorV2(const vector<float> &logits, float &out_score)
{
    if (logits.empty())
    {
        out_score = 0.0f;
        return "";
    }

    const size_t N = std::min(logits.size(), PLATE_COLORS_V2.size());

    int best = 0;
    float max_logit = -1e30f;
    for (size_t i = 0; i < N; ++i)
    {
        if (logits[i] > max_logit)
        {
            max_logit = logits[i];
            best = static_cast<int>(i);
        }
    }

    float denom = 0.0f;
    for (size_t i = 0; i < N; ++i)
        denom += expf(logits[i] - max_logit);
    out_score = expf(max_logit - max_logit) / (denom > 0.0f ? denom : 1.0f);

    return PLATE_COLORS_V2[best];
}

static const char *ocrBackendName(OcrBackend backend)
{
    return backend == OcrBackend::LPRNET ? "lprnet" : "plate_color";
}

static bool parseOcrBackendName(const string &name, OcrBackend &backend)
{
    string normalized = name;
    transform(normalized.begin(), normalized.end(), normalized.begin(),
              [](unsigned char c)
              { return static_cast<char>(tolower(c)); });
    if (normalized == "lprnet")
    {
        backend = OcrBackend::LPRNET;
        return true;
    }
    if (normalized == "plate_color")
    {
        backend = OcrBackend::PLATE_COLOR;
        return true;
    }
    return false;
}

struct RknnProfile
{
    double total_ms = 0.0;
    double preprocess_ms = 0.0;
    double inputs_set_ms = 0.0;
    double rknn_run_ms = 0.0;
    double outputs_get_ms = 0.0;
    double output_copy_ms = 0.0;

    void reset()
    {
        total_ms = 0.0;
        preprocess_ms = 0.0;
        inputs_set_ms = 0.0;
        rknn_run_ms = 0.0;
        outputs_get_ms = 0.0;
        output_copy_ms = 0.0;
    }
};

struct YoloProfile
{
    double total_ms = 0.0;
    RknnProfile rknn;
    double postprocess_ms = 0.0;
    size_t detections = 0;
    size_t rows = 0;
    size_t candidates_above_threshold = 0;
    size_t invalid_values = 0;
    float max_confidence = 0.0f;

    void reset()
    {
        total_ms = 0.0;
        rknn.reset();
        postprocess_ms = 0.0;
        detections = 0;
        rows = 0;
        candidates_above_threshold = 0;
        invalid_values = 0;
        max_confidence = 0.0f;
    }
};

static double elapsedMs(chrono::steady_clock::time_point t0,
                        chrono::steady_clock::time_point t1)
{
    return chrono::duration<double, milli>(t1 - t0).count();
}

struct CachedPlate
{
    Rect box;     // YOLO在原始整帧上的检测框（OBB时为轴对齐包络）
    Rect roi_box; // 实际裁剪并送入FPGA处理的ROI区域
    Rect ocr_box; // bypass/debug时实际送入OCR前的裁剪区域
    float score = 0.0f;
    float ocr_score = 0.0f;
    string raw_plate;
    string plate; // 跨帧稳定后的车牌；为空时只显示raw_plate，不更新/status
    string color; // plate_color模型输出：黑色/蓝色/绿色/白色/黄色
    float color_score = 0.0f;
    int track_id = -1;
    bool fpga_ok = false;
    bool obb = false;        // OBB 检测: 绘制旋转框
    float angle = 0.0f;      // 旋转框长边角度 (rad)
    float rot_w = 0.0f;      // 旋转框长边
    float rot_h = 0.0f;      // 旋转框短边
    bool has_quad = false;   // 角点模型: 绘制四角点四边形 [TL,TR,BR,BL]
    float quad[4][2] = {};
};

struct PlateStatusItem
{
    string plate;
    string color;
};

struct OcrDebugMeta
{
    vector<int> output_shape;
    vector<float> output_probs;
    int class_count = 0;
    int time_steps = 0;
    bool channel_major = true;
    double elapsed_ms = 0.0;
    string input_color = "BGR";
    string input_layout = "NHWC";
    string input_type = "uint8";
    string rknn_input_type = "uint8";
    string backend = "lprnet";
    string color;
    float color_score = 0.0f;
};

static mutex g_status_mutex;
static string g_latest_plate_text = "";
static vector<PlateStatusItem> g_latest_plate_details;
static string g_webrtc_url;
static string g_http_flv_url;
static string g_rtsp_url;

static void handleSignal(int)
{
    g_running = false;
    g_input_cv.notify_all();
    g_aligned_output_cv.notify_all();
    if (g_server_fd >= 0)
    {
        close(g_server_fd);
        g_server_fd = -1;
    }
}

static bool sendAll(int fd, const void *data, size_t len)
{
    const char *p = static_cast<const char *>(data);
    while (len > 0)
    {
        ssize_t n = send(fd, p, len, MSG_NOSIGNAL);
        if (n <= 0)
            return false;
        p += n;
        len -= static_cast<size_t>(n);
    }
    return true;
}

static bool sendText(int fd, const string &s)
{
    return sendAll(fd, s.data(), s.size());
}

static void setSocketLowLatency(int fd)
{
    int yes = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

    timeval tv{};
    tv.tv_sec = 3;
    tv.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
}

static void sendHttpStatus(int fd, int code, const string &message)
{
    string body = message + "\n";
    string header =
        "HTTP/1.1 " + to_string(code) + " " + message + "\r\n"
                                                        "Connection: close\r\n"
                                                        "Content-Type: text/plain; charset=utf-8\r\n"
                                                        "Cache-Control: no-store\r\n"
                                                        "Content-Length: " +
        to_string(body.size()) + "\r\n\r\n";
    sendText(fd, header);
    sendText(fd, body);
}

static void sendHttpTextResponse(int fd,
                                 int code,
                                 const string &reason,
                                 const string &body)
{
    string response_body = body;
    if (response_body.empty() || response_body[response_body.size() - 1] != '\n')
    {
        response_body += "\n";
    }
    string header =
        "HTTP/1.1 " + to_string(code) + " " + reason + "\r\n"
                                                       "Connection: close\r\n"
                                                       "Content-Type: text/plain; charset=utf-8\r\n"
                                                       "Cache-Control: no-store\r\n"
                                                       "Content-Length: " +
        to_string(response_body.size()) + "\r\n\r\n";
    sendText(fd, header);
    sendText(fd, response_body);
}

static bool recvHttpHeader(int fd, string &header)
{
    string data;
    char buffer[2048];

    while (g_running)
    {
        ssize_t n = recv(fd, buffer, sizeof(buffer), 0);
        if (n <= 0)
            return false;

        data.append(buffer, static_cast<size_t>(n));
        size_t pos = data.find("\r\n\r\n");
        if (pos != string::npos)
        {
            header = data.substr(0, pos + 4);
            return true;
        }

        if (data.size() > 64 * 1024)
            return false;
    }

    return false;
}

static string jsonEscape(const string &s)
{
    string out;
    out.reserve(s.size() + 16);
    for (unsigned char ch : s)
    {
        switch (ch)
        {
        case '\\':
            out += "\\\\";
            break;
        case '"':
            out += "\\\"";
            break;
        case '\n':
            out += "\\n";
            break;
        case '\r':
            out += "\\r";
            break;
        case '\t':
            out += "\\t";
            break;
        default:
            if (ch < 0x20)
            {
                char buf[8];
                snprintf(buf, sizeof(buf), "\\u%04x", ch);
                out += buf;
            }
            else
            {
                out += static_cast<char>(ch);
            }
        }
    }
    return out;
}

static bool fileExists(const string &path)
{
    return access(path.c_str(), R_OK) == 0;
}

static int clampPort(int port)
{
    return max(1, min(65535, port));
}

static bool readFile(const string &path, vector<unsigned char> &data)
{
    ifstream ifs(path.c_str(), ios::binary);
    if (!ifs)
        return false;
    ifs.seekg(0, ios::end);
    streamoff len = ifs.tellg();
    if (len <= 0)
        return false;
    ifs.seekg(0, ios::beg);
    data.resize(static_cast<size_t>(len));
    ifs.read(reinterpret_cast<char *>(data.data()), len);
    return ifs.good();
}

static void updateInputFrame(const Mat &frame)
{
    if (frame.empty())
        return;

    {
        lock_guard<mutex> lock(g_input_mutex);
        // convertBufferToMat已经把V4L2底层缓存转换成了独立Mat。
        // 这里使用浅拷贝，避免每帧额外clone一次整幅图。
        g_input_frame = frame;
        g_input_width = g_input_frame.cols;
        g_input_height = g_input_frame.rows;
        g_input_frame_id++;
        g_capture_count++;
    }

    g_input_cv.notify_all();
    g_frame_gen_us = chrono::duration_cast<chrono::microseconds>(
        chrono::system_clock::now().time_since_epoch()).count();
    // 同一时刻记进环形表：推理线程发布对齐帧时按 frame_id 回查，得到真实采集时刻
    g_cap_ts_ring[g_input_frame_id & 127].store(g_frame_gen_us.load());
}

static string joinPlates(const vector<string> &plates)
{
    if (plates.empty())
        return "";
    string text;
    for (size_t i = 0; i < plates.size(); ++i)
    {
        if (i)
            text += " / ";
        text += plates[i];
    }
    return text;
}

static void updatePlateStatus(const vector<string> &plates,
                              const vector<CachedPlate> &results)
{
    lock_guard<mutex> lock(g_status_mutex);
    g_latest_plate_text = joinPlates(plates);
    g_latest_plate_details.clear();
    for (const CachedPlate &item : results)
    {
        // 与原/status行为一致：仅发布已经通过跨帧投票的稳定车牌。
        if (item.plate.empty())
            continue;
        PlateStatusItem status;
        status.plate = item.plate;
        status.color = item.color;
        g_latest_plate_details.push_back(status);
    }
}

class RknnModel
{
public:
    RknnModel() = default;
    ~RknnModel() { release(); }

    bool init(const string &model_path)
    {
        release();

        vector<unsigned char> model_data;
        if (!readFile(model_path, model_data))
        {
            cerr << "Failed to read RKNN model: " << model_path << endl;
            return false;
        }

        int ret = rknn_init(&ctx_, model_data.data(),
                            static_cast<uint32_t>(model_data.size()), 0, nullptr);
        if (ret < 0)
        {
            cerr << "rknn_init failed, ret=" << ret << ", model=" << model_path << endl;
            return false;
        }

        rknn_sdk_version sdk_version{};
        ret = rknn_query(ctx_, RKNN_QUERY_SDK_VERSION, &sdk_version, sizeof(sdk_version));
        if (ret == RKNN_SUCC)
        {
            cout << "RKNN runtime: api=" << sdk_version.api_version
                 << ", driver=" << sdk_version.drv_version << endl;
        }

        ret = rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
        if (ret != RKNN_SUCC)
        {
            cerr << "RKNN_QUERY_IN_OUT_NUM failed, ret=" << ret << endl;
            return false;
        }

        input_attrs_.resize(io_num_.n_input);
        output_attrs_.resize(io_num_.n_output);

        cout << "RKNN model: " << model_path << endl;
        cout << "  inputs=" << io_num_.n_input << ", outputs=" << io_num_.n_output << endl;

        for (uint32_t i = 0; i < io_num_.n_input; ++i)
        {
            memset(&input_attrs_[i], 0, sizeof(rknn_tensor_attr));
            input_attrs_[i].index = i;
            ret = rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &input_attrs_[i], sizeof(rknn_tensor_attr));
            if (ret != RKNN_SUCC)
            {
                cerr << "RKNN_QUERY_INPUT_ATTR failed, ret=" << ret << endl;
                return false;
            }
            dumpAttr("input", input_attrs_[i]);
        }

        for (uint32_t i = 0; i < io_num_.n_output; ++i)
        {
            memset(&output_attrs_[i], 0, sizeof(rknn_tensor_attr));
            output_attrs_[i].index = i;
            ret = rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &output_attrs_[i], sizeof(rknn_tensor_attr));
            if (ret != RKNN_SUCC)
            {
                cerr << "RKNN_QUERY_OUTPUT_ATTR failed, ret=" << ret << endl;
                return false;
            }
            dumpAttr("output", output_attrs_[i]);
        }

        const rknn_tensor_attr &in = input_attrs_[0];

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

        if (model_width_ <= 0 || model_height_ <= 0 || model_channel_ <= 0)
        {
            cerr << "Invalid RKNN input shape." << endl;
            return false;
        }

        return true;
    }

    void release()
    {
        if (ctx_ != 0)
        {
            rknn_destroy(ctx_);
            ctx_ = 0;
        }
        input_attrs_.clear();
        output_attrs_.clear();
        uint8_input_.clear();
        float_input_.clear();
        outputs_.clear();
        converted_bgr_.release();
        resized_.release();
        color_.release();
        model_width_ = 0;
        model_height_ = 0;
        model_channel_ = 0;
    }

    bool runImage(const Mat &bgr, bool swap_rb, vector<vector<float>> &output_data,
                  bool force_float_input = false, RknnProfile *profile = nullptr)
    {
        if (profile)
            profile->reset();

        if (ctx_ == 0 || bgr.empty() || input_attrs_.empty())
            return false;

        auto total_t0 = chrono::steady_clock::now();
        auto pre_t0 = total_t0;

        // 统一转成BGR三通道，兼容灰度/BGRA输入。
        const Mat *src_bgr = &bgr;
        if (bgr.channels() == 1)
        {
            cvtColor(bgr, converted_bgr_, COLOR_GRAY2BGR);
            src_bgr = &converted_bgr_;
        }
        else if (bgr.channels() == 4)
        {
            cvtColor(bgr, converted_bgr_, COLOR_BGRA2BGR);
            src_bgr = &converted_bgr_;
        }

        resize(*src_bgr, resized_, Size(model_width_, model_height_), 0, 0, INTER_LINEAR);

        const Mat *color = &resized_;
        if (swap_rb)
        {
            cvtColor(resized_, color_, COLOR_BGR2RGB);
            color = &color_;
        }

        vector<unsigned char> &uint8_input = uint8_input_;
        vector<float> &float_input = float_input_;

        rknn_input input;
        memset(&input, 0, sizeof(input));
        input.index = 0;

        const rknn_tensor_attr &attr = input_attrs_[0];
        bool float_model = force_float_input || (attr.type == RKNN_TENSOR_FLOAT32);

        if (float_model)
        {
            input.type = RKNN_TENSOR_FLOAT32;
            input.fmt = attr.fmt;
            input.size = model_width_ * model_height_ * model_channel_ * sizeof(float);
            float_input.resize(model_width_ * model_height_ * model_channel_);

            if (input.fmt == RKNN_TENSOR_NCHW)
            {
                for (int c = 0; c < model_channel_; ++c)
                {
                    for (int y = 0; y < model_height_; ++y)
                    {
                        const Vec3b *row = color->ptr<Vec3b>(y);
                        for (int x = 0; x < model_width_; ++x)
                        {
                            float_input[c * model_width_ * model_height_ + y * model_width_ + x] =
                                row[x][c] / 255.0f;
                        }
                    }
                }
            }
            else
            {
                size_t k = 0;
                for (int y = 0; y < model_height_; ++y)
                {
                    const Vec3b *row = color->ptr<Vec3b>(y);
                    for (int x = 0; x < model_width_; ++x)
                    {
                        for (int c = 0; c < model_channel_; ++c)
                        {
                            float_input[k++] = row[x][c] / 255.0f;
                        }
                    }
                }
            }

            input.buf = float_input.data();
        }
        else
        {
            input.type = RKNN_TENSOR_UINT8;
            input.fmt = RKNN_TENSOR_NHWC;
            input.size = model_width_ * model_height_ * model_channel_;
            uint8_input.resize(input.size);

            if (color->isContinuous())
            {
                memcpy(uint8_input.data(), color->data, input.size);
            }
            else
            {
                size_t row_bytes = static_cast<size_t>(model_width_) * model_channel_;
                for (int y = 0; y < model_height_; ++y)
                {
                    memcpy(uint8_input.data() + static_cast<size_t>(y) * row_bytes,
                           color->ptr(y), row_bytes);
                }
            }
            input.buf = uint8_input.data();
        }

        auto pre_t1 = chrono::steady_clock::now();
        if (profile)
            profile->preprocess_ms = elapsedMs(pre_t0, pre_t1);

        auto input_t0 = chrono::steady_clock::now();
        int ret = rknn_inputs_set(ctx_, 1, &input);
        auto input_t1 = chrono::steady_clock::now();
        if (profile)
            profile->inputs_set_ms = elapsedMs(input_t0, input_t1);
        if (ret < 0)
        {
            cerr << "rknn_inputs_set failed, ret=" << ret << endl;
            if (profile)
                profile->total_ms = elapsedMs(total_t0, input_t1);
            return false;
        }

        auto run_t0 = chrono::steady_clock::now();
        ret = rknn_run(ctx_, nullptr);
        auto run_t1 = chrono::steady_clock::now();
        if (profile)
            profile->rknn_run_ms = elapsedMs(run_t0, run_t1);
        if (ret < 0)
        {
            cerr << "rknn_run failed, ret=" << ret << endl;
            if (profile)
                profile->total_ms = elapsedMs(total_t0, run_t1);
            return false;
        }

        if (outputs_.size() != io_num_.n_output)
        {
            outputs_.resize(io_num_.n_output);
        }
        memset(outputs_.data(), 0, outputs_.size() * sizeof(rknn_output));
        for (uint32_t i = 0; i < io_num_.n_output; ++i)
        {
            outputs_[i].index = i;
            outputs_[i].want_float = 1;
        }

        auto get_t0 = chrono::steady_clock::now();
        ret = rknn_outputs_get(ctx_, io_num_.n_output, outputs_.data(), nullptr);
        auto get_t1 = chrono::steady_clock::now();
        if (profile)
            profile->outputs_get_ms = elapsedMs(get_t0, get_t1);
        if (ret < 0)
        {
            cerr << "rknn_outputs_get failed, ret=" << ret << endl;
            if (profile)
                profile->total_ms = elapsedMs(total_t0, get_t1);
            return false;
        }

        auto copy_t0 = chrono::steady_clock::now();
        if (output_data.size() != io_num_.n_output)
            output_data.resize(io_num_.n_output);

        for (uint32_t i = 0; i < io_num_.n_output; ++i)
        {
            int elems = output_attrs_[i].n_elems;
            output_data[i].resize(elems);
            memcpy(output_data[i].data(), outputs_[i].buf, elems * sizeof(float));
        }
        auto copy_t1 = chrono::steady_clock::now();
        if (profile)
            profile->output_copy_ms = elapsedMs(copy_t0, copy_t1);

        rknn_outputs_release(ctx_, io_num_.n_output, outputs_.data());
        if (profile)
            profile->total_ms = elapsedMs(total_t0, chrono::steady_clock::now());
        return true;
    }

    int width() const { return model_width_; }
    int height() const { return model_height_; }
    int channel() const { return model_channel_; }
    const vector<rknn_tensor_attr> &inputAttrs() const { return input_attrs_; }
    const vector<rknn_tensor_attr> &outputAttrs() const { return output_attrs_; }
    rknn_context ctx() const { return ctx_; }

private:
    rknn_context ctx_ = 0;
    rknn_input_output_num io_num_{};
    vector<rknn_tensor_attr> input_attrs_;
    vector<rknn_tensor_attr> output_attrs_;
    int model_width_ = 0;
    int model_height_ = 0;
    int model_channel_ = 0;
    vector<unsigned char> uint8_input_;
    vector<float> float_input_;
    vector<rknn_output> outputs_;
    Mat converted_bgr_;
    Mat resized_;
    Mat color_;

    static void dumpAttr(const char *tag, const rknn_tensor_attr &attr)
    {
        cout << "  " << tag << "[" << attr.index << "] dims=[";
        for (int i = 0; i < attr.n_dims; ++i)
        {
            if (i)
                cout << ",";
            cout << attr.dims[i];
        }
        cout << "] elems=" << attr.n_elems
             << " size=" << attr.size
             << " fmt=" << attr.fmt
             << " type=" << attr.type
             << " qnt=" << attr.qnt_type
             << " zp=" << attr.zp
             << " scale=" << attr.scale << endl;
    }
};

struct Detection
{
    Rect box;            // HBB: detection box; OBB: axis-aligned envelope (original frame coords)
    float score = 0.0f;
    bool obb = false;    // true: rotated box from YOLOv8-OBB
    float angle = 0.0f;  // OBB long-edge angle, radians
    float rot_w = 0.0f;  // OBB long side, original image coords
    float rot_h = 0.0f;  // OBB short side, original image coords
    // OBB+corner model (7 outputs): quadrilateral corners, fixed identity order
    // [TL, TR, BR, BL] pinned to the box reference corners (original frame coords).
    bool has_quad = false;
    float quad[4][2] = {};
};

// ========== YOLOv5n post-processing constants (from RK_anchors.txt) ==========
// These anchors are already stride-scaled. Do NOT multiply by stride again.
static const int YOLOV5_STRIDES[3] = {8, 16, 32};
static const float YOLOV5_ANCHORS[3][3][2] = {
    {{10.0f, 13.0f}, {16.0f, 30.0f}, {33.0f, 23.0f}},   // stride 8
    {{30.0f, 61.0f}, {62.0f, 45.0f}, {59.0f, 119.0f}},   // stride 16
    {{116.0f, 90.0f}, {156.0f, 198.0f}, {373.0f, 326.0f}} // stride 32
};
static const int YOLOV5_IMG_SIZE = 640;
static const int YOLOV5_NC = 1;  // single class: plate
static const int YOLOV5_NA = 3;  // anchors per stride

struct LetterboxParams
{
    float scale = 1.0f;
    int new_w = 0;
    int new_h = 0;
    int left = 0;
    int top = 0;
};

// Compute YOLOv5 letterbox parameters (preserve aspect ratio, pad with gray 114)
static LetterboxParams computeLetterbox(int img_w, int img_h, int target_size = YOLOV5_IMG_SIZE)
{
    LetterboxParams lb;
    lb.scale = std::min(static_cast<float>(target_size) / img_h,
                        static_cast<float>(target_size) / img_w);
    lb.new_w = static_cast<int>(img_w * lb.scale);
    lb.new_h = static_cast<int>(img_h * lb.scale);
    lb.left = (target_size - lb.new_w) / 2;
    lb.top  = (target_size - lb.new_h) / 2;
    return lb;
}

// Apply letterbox: resize + pad to target_size, returns RGB uint8 NHWC-ready image
static Mat applyLetterbox(const Mat &bgr, int target_size, const LetterboxParams &lb)
{
    Mat resized, canvas;
    resize(bgr, resized, Size(lb.new_w, lb.new_h), 0, 0, INTER_LINEAR);
    canvas = Mat(target_size, target_size, CV_8UC3, Scalar(114, 114, 114));
    resized.copyTo(canvas(Rect(lb.left, lb.top, lb.new_w, lb.new_h)));
    Mat rgb;
    cvtColor(canvas, rgb, COLOR_BGR2RGB);
    return rgb;  // NHWC uint8, ready for RKNN INT8 input
}

// YOLOv5 decode: 3 raw feature maps → decoded boxes in original image coords
// Returns vector of (Rect, score) pairs; caller applies NMS.
static vector<pair<Rect, float>> decodeYolov5Outputs(
    const vector<vector<float>> &outputs,
    const vector<rknn_tensor_attr> &output_attrs,
    int orig_w, int orig_h,
    float conf_thres,
    const LetterboxParams &lb)
{
    vector<pair<Rect, float>> dets;

    // Sort outputs by spatial dimension (descending H×W → stride 8, 16, 32)
    vector<pair<int, int>> sorted;  // (index, spatial_size)
    for (size_t i = 0; i < outputs.size(); ++i)
    {
        bool nchw = (output_attrs[i].fmt == RKNN_TENSOR_NCHW);
        int h = nchw ? output_attrs[i].dims[2] : output_attrs[i].dims[1];
        int w = nchw ? output_attrs[i].dims[3] : output_attrs[i].dims[2];
        sorted.push_back({static_cast<int>(i), h * w});
    }
    sort(sorted.begin(), sorted.end(),
         [](const pair<int, int> &a, const pair<int, int> &b) { return a.second > b.second; });

    for (size_t si = 0; si < sorted.size(); ++si)
    {
        int idx = sorted[si].first;
        int stride = YOLOV5_STRIDES[si];

        // Determine H,W from tensor attributes (handle both NCHW and NHWC)
        bool nchw = (output_attrs[idx].fmt == RKNN_TENSOR_NCHW);
        int H, W;
        if (nchw)
        {
            H = output_attrs[idx].dims[2];  // NCHW: (1, C, H, W)
            W = output_attrs[idx].dims[3];
        }
        else
        {
            H = output_attrs[idx].dims[1];  // NHWC: (1, H, W, C)
            W = output_attrs[idx].dims[2];
        }

        const vector<float> &feat = outputs[idx];
        // feat shape: NCHW (1, 18, H, W) → reshape to (3, 6, H, W)
        // Channel layout per anchor a∈{0,1,2}: tx,ty,tw,th,obj,cls

        for (int a = 0; a < YOLOV5_NA; ++a)
        {
            float anchor_w = YOLOV5_ANCHORS[si][a][0];
            float anchor_h = YOLOV5_ANCHORS[si][a][1];

            for (int gy = 0; gy < H; ++gy)
            {
                for (int gx = 0; gx < W; ++gx)
                {
                    int ch_base = a * 6;

                    // NCHW flat index: pos = c*H*W + y*W + x
                    auto val_nchw = [&](int c, int y, int x) -> float {
                        size_t pos = static_cast<size_t>(c) * H * W +
                                     static_cast<size_t>(y) * W +
                                     static_cast<size_t>(x);
                        return pos < feat.size() ? feat[pos] : 0.0f;
                    };

                    float tx  = val_nchw(ch_base + 0, gy, gx);
                    float ty  = val_nchw(ch_base + 1, gy, gx);
                    float tw  = val_nchw(ch_base + 2, gy, gx);
                    float th  = val_nchw(ch_base + 3, gy, gx);
                    float obj = val_nchw(ch_base + 4, gy, gx);
                    float cls = val_nchw(ch_base + 5, gy, gx);

                    float score = obj * cls;
                    if (score < conf_thres)
                        continue;

                    // YOLOv5 v7.0 decode with -0.5 grid offset
                    float cx = (tx * 2.0f - 0.5f + static_cast<float>(gx)) * stride;
                    float cy = (ty * 2.0f - 0.5f + static_cast<float>(gy)) * stride;
                    float w  = std::pow(tw * 2.0f, 2.0f) * anchor_w;
                    float h  = std::pow(th * 2.0f, 2.0f) * anchor_h;

                    // Map from letterbox space → original image coords
                    int left   = static_cast<int>((cx - 0.5f * w - lb.left) / lb.scale);
                    int top    = static_cast<int>((cy - 0.5f * h - lb.top) / lb.scale);
                    int width  = static_cast<int>(w / lb.scale);
                    int height = static_cast<int>(h / lb.scale);

                    Rect box(left, top, width, height);
                    box &= Rect(0, 0, orig_w, orig_h);
                    if (box.area() <= 0)
                        continue;

                    dets.push_back({box, score});
                }
            }
        }
    }

    return dets;
}

static Rect expandBox(const Rect &box, int img_w, int img_h, float scale_x = 0.08f, float scale_y = 0.18f)
{
    int dx = static_cast<int>(box.width * scale_x);
    int dy = static_cast<int>(box.height * scale_y);
    Rect r(box.x - dx, box.y - dy, box.width + 2 * dx, box.height + 2 * dy);
    r &= Rect(0, 0, img_w, img_h);
    return r;
}

static Rect fitBoxToAspect(const Rect &box, int img_w, int img_h, float target_aspect)
{
    Rect clipped = box & Rect(0, 0, img_w, img_h);
    if (clipped.area() <= 0 || target_aspect <= 0.0f)
        return clipped;

    float cx = clipped.x + clipped.width * 0.5f;
    float cy = clipped.y + clipped.height * 0.5f;
    float w = static_cast<float>(clipped.width);
    float h = static_cast<float>(clipped.height);
    float aspect = w / max(1.0f, h);

    if (aspect < target_aspect)
    {
        w = h * target_aspect;
    }
    else
    {
        h = w / target_aspect;
    }

    Rect r(static_cast<int>(std::round(cx - w * 0.5f)),
           static_cast<int>(std::round(cy - h * 0.5f)),
           max(1, static_cast<int>(std::round(w))),
           max(1, static_cast<int>(std::round(h))));
    r &= Rect(0, 0, img_w, img_h);
    return r.area() > 0 ? r : clipped;
}

static float iouValue(const Rect &a, const Rect &b)
{
    Rect inter = a & b;
    float inter_area = static_cast<float>(inter.area());
    float union_area = static_cast<float>(a.area() + b.area()) - inter_area;
    return union_area <= 0.0f ? 0.0f : inter_area / union_area;
}

static vector<int> nmsIndices(const vector<Rect> &boxes,
                              const vector<float> &scores,
                              float conf_thres,
                              float nms_thres)
{
    vector<int> order;
    for (size_t i = 0; i < scores.size(); ++i)
    {
        if (scores[i] >= conf_thres)
        {
            order.push_back(static_cast<int>(i));
        }
    }

    sort(order.begin(), order.end(), [&](int a, int b)
         { return scores[a] > scores[b]; });

    vector<int> keep;
    vector<bool> suppressed(order.size(), false);

    for (size_t i = 0; i < order.size(); ++i)
    {
        if (suppressed[i])
            continue;

        int idx = order[i];
        keep.push_back(idx);

        for (size_t j = i + 1; j < order.size(); ++j)
        {
            if (!suppressed[j] && iouValue(boxes[idx], boxes[order[j]]) > nms_thres)
            {
                suppressed[j] = true;
            }
        }
    }

    return keep;
}

// ========== YOLOv8-OBB post-processing (single class: plate) ==========
// Ported from board-verified /userdata/obb_test/bench_post.cpp (rknn_model_zoo math).
struct ObbCandidate
{
    float cx = 0.0f, cy = 0.0f;   // center
    float w = 0.0f, h = 0.0f;     // long / short side
    float ang = 0.0f;             // long-edge angle, radians
    float score = 0.0f;
    Rect env;                     // axis-aligned envelope
    int aidx = -1;                // global anchor index (P3→P4→P5) for corner lookup
};

static inline float sigmoidObb(float x) { return 1.0f / (1.0f + expf(-x)); }

// DFL: softmax over 16 bins → expectation (distribution focal loss decode)
static float dflSide(const float *v)
{
    float m = v[0];
    for (int i = 1; i < 16; ++i)
        m = max(m, v[i]);
    float s = 0.0f, e[16];
    for (int i = 0; i < 16; ++i)
    {
        e[i] = expf(v[i] - m);
        s += e[i];
    }
    float acc = 0.0f;
    for (int i = 0; i < 16; ++i)
        acc += static_cast<float>(i) * e[i];
    return acc / s;
}

// Decode YOLOv8-OBB raw outputs → rotated candidates in letterbox space.
// outputs[0..2]: (1, 65, H, W) stride-8/16/32 heads — ch 0-63 = DFL (l/t/r/b × 16),
//                ch 64 = class logit (single class). outputs[3]: angle (1,1,8400),
//                sigmoid already applied inside the model, concatenated in head order.
static vector<ObbCandidate> decodeYolov8Obb(const vector<vector<float>> &outputs,
                                            const vector<rknn_tensor_attr> &attrs,
                                            float conf_thres)
{
    static const int STRIDES[3] = {8, 16, 32};
    vector<ObbCandidate> boxes;
    if (outputs.size() < 4 || attrs.size() < 4)
        return boxes;

    const vector<float> &ang_out = outputs[3];
    size_t ang_offset = 0;

    for (int i = 0; i < 3; ++i)
    {
        bool nchw = (attrs[i].fmt == RKNN_TENSOR_NCHW);
        int H = nchw ? attrs[i].dims[2] : attrs[i].dims[1];
        int W = nchw ? attrs[i].dims[3] : attrs[i].dims[2];
        int C = nchw ? attrs[i].dims[1] : attrs[i].dims[3];
        int HW = H * W;
        const vector<float> &feat = outputs[i];
        if (feat.empty() || ang_offset + static_cast<size_t>(HW) > ang_out.size())
            continue;

        auto val = [&](int c, int y, int x) -> float {
            size_t pos = nchw
                             ? static_cast<size_t>(c) * HW + static_cast<size_t>(y) * W + x
                             : (static_cast<size_t>(y) * W + x) * C + c;
            return pos < feat.size() ? feat[pos] : 0.0f;
        };

        for (int y = 0; y < H; ++y)
        {
            for (int x = 0; x < W; ++x)
            {
                int idx = y * W + x;
                float cls = sigmoidObb(val(64, y, x));
                if (cls <= conf_thres)
                    continue;

                float ltrb[4];
                for (int s = 0; s < 4; ++s)
                {
                    float bins[16];
                    for (int b = 0; b < 16; ++b)
                        bins[b] = val(s * 16 + b, y, x);
                    ltrb[s] = dflSide(bins);
                }

                float a = (ang_out[ang_offset + idx] - 0.25f) * static_cast<float>(CV_PI);
                float sub_x = (ltrb[2] - ltrb[0]) / 2.0f;
                float sub_y = (ltrb[3] - ltrb[1]) / 2.0f;
                float ca = cosf(a), sa = sinf(a);
                float rot_x = sub_x * ca - sub_y * sa;
                float rot_y = sub_x * sa + sub_y * ca;

                ObbCandidate c;
                c.cx = (rot_x + static_cast<float>(x) + 0.5f) * STRIDES[i];
                c.cy = (rot_y + static_cast<float>(y) + 0.5f) * STRIDES[i];
                c.w = (ltrb[0] + ltrb[2]) * STRIDES[i];
                c.h = (ltrb[1] + ltrb[3]) * STRIDES[i];
                c.ang = a;
                c.score = cls;
                c.aidx = static_cast<int>(ang_offset) + idx;
                boxes.push_back(c);
            }
        }
        ang_offset += static_cast<size_t>(HW);
    }
    return boxes;
}

// Rotated box corners: c + R(θ)·(±w/2, ±h/2)
static void obbCorners(const ObbCandidate &b, float px[4], float py[4])
{
    float ca = cosf(b.ang), sa = sinf(b.ang);
    float hw = b.w * 0.5f, hh = b.h * 0.5f;
    float v1x = hw * ca - hh * sa, v1y = hw * sa + hh * ca;
    float v2x = -hw * ca - hh * sa, v2y = -hw * sa + hh * ca;
    px[0] = b.cx + v1x; py[0] = b.cy + v1y;
    px[1] = b.cx + v2x; py[1] = b.cy + v2y;
    px[2] = b.cx - v1x; py[2] = b.cy - v1y;
    px[3] = b.cx - v2x; py[3] = b.cy - v2y;
}

// ========== OBB+corner model (7 outputs): quadrilateral branch ==========
// Corner heads (outputs[4..6]): (1, 8, H, W) raw logits x0,y0,x1,y1,x2,y2,x3,y3.
// Decode at global anchor `aidx` (P3→P4→P5, same order as the angle slice):
//   corner_px = (sigmoid(logit) * 2 - 1) * 150 + (grid + 0.5) * stride
// in 640 letterbox space. Board-verified against the Python reference:
// /userdata/obb_test/ref_obb_corner.cpp (DEPLOYMENT_OBB.md appendix A6).
static void decodeQuadCorners(const vector<vector<float>> &outputs,
                              const vector<rknn_tensor_attr> &attrs,
                              int aidx, float pts[4][2])
{
    static const float CORNER_HALF_RANGE = 150.0f;
    if (outputs.size() < 7 || attrs.size() < 7 || aidx < 0)
        return;
    int idx = aidx;
    for (int i = 0; i < 3; ++i)
    {
        const rknn_tensor_attr &a = attrs[4 + i];
        bool nchw = (a.fmt == RKNN_TENSOR_NCHW);
        int H = nchw ? a.dims[2] : a.dims[1];
        int W = nchw ? a.dims[3] : a.dims[2];
        int HW = H * W;
        int stride = 8 << i;
        if (idx < HW)
        {
            const vector<float> &c = outputs[4 + i];
            int y = idx / W, x = idx % W;
            auto v = [&](int ch) -> float {
                size_t pos = nchw
                                 ? static_cast<size_t>(ch) * HW + static_cast<size_t>(idx)
                                 : static_cast<size_t>(idx) * 8 + ch;
                return pos < c.size() ? c[pos] : 0.0f;
            };
            for (int k = 0; k < 4; ++k)
            {
                float t = sigmoidObb(v(2 * k));
                pts[k][0] = (t * 2.0f - 1.0f) * CORNER_HALF_RANGE +
                            (static_cast<float>(x) + 0.5f) * stride;
                t = sigmoidObb(v(2 * k + 1));
                pts[k][1] = (t * 2.0f - 1.0f) * CORNER_HALF_RANGE +
                            (static_cast<float>(y) + 0.5f) * stride;
            }
            return;
        }
        idx -= HW;
    }
}

// Canonical corner order: start at the top-left-most corner (min y, tie min x),
// continue clockwise (positive shoelace in y-down image coords). Identical to
// loss.py _canonicalize_quad / ref_obb_corner.cpp canonicalize().
static void canonicalizeQuad(float pts[4][2])
{
    int i0 = 0;
    for (int i = 1; i < 4; ++i)
        if (pts[i][1] < pts[i0][1] || (pts[i][1] == pts[i0][1] && pts[i][0] < pts[i0][0]))
            i0 = i;
    int rest[3], nr = 0;
    for (int i = 0; i < 4; ++i)
        if (i != i0) rest[nr++] = i;
    float ang[3];
    for (int k = 0; k < 3; ++k)
        ang[k] = atan2f(pts[rest[k]][1] - pts[i0][1], pts[rest[k]][0] - pts[i0][0]);
    for (int a = 0; a < 3; ++a)
        for (int b = a + 1; b < 3; ++b)
            if (ang[b] < ang[a])
            {
                swap(ang[a], ang[b]);
                swap(rest[a], rest[b]);
            }
    float q[4][2] = {{pts[i0][0], pts[i0][1]},
                     {pts[rest[0]][0], pts[rest[0]][1]},
                     {pts[rest[1]][0], pts[rest[1]][1]},
                     {pts[rest[2]][0], pts[rest[2]][1]}};
    float area = 0.0f;
    for (int k = 0; k < 4; ++k)
    {
        int k2 = (k + 1) % 4;
        area += q[k][0] * q[k2][1] - q[k2][0] * q[k][1];
    }
    if (area < 0.0f)
    {
        swap(q[1][0], q[3][0]);
        swap(q[1][1], q[3][1]);
    }
    memcpy(pts, q, sizeof(q));
}

// Pin canonical quad corners to fixed identity [TL, TR, BR, BL] by nearest box
// reference corner (c ± v1/v2, same math as obbCorners). The box angle carries
// the text direction, so identity follows it — the canonical order alone cannot
// tell which end of the plate is up when the tilt flips sign. Returns false
// when a corner deviates too far from the box (corner-branch garbage); the
// caller then falls back to the rotated-box crop.
static bool assignQuadIdentity(const float quad[4][2],
                               float cx, float cy, float angle, float w, float h,
                               float out_q[4][2])
{
    float ca = cosf(angle), sa = sinf(angle);
    float hw = w * 0.5f, hh = h * 0.5f;
    float v1x = hw * ca - hh * sa, v1y = hw * sa + hh * ca;
    float v2x = -hw * ca - hh * sa, v2y = -hw * sa + hh * ca;
    const float ref[4][2] = {{cx - v1x, cy - v1y},   // TL
                             {cx - v2x, cy - v2y},   // TR
                             {cx + v1x, cy + v1y},   // BR
                             {cx + v2x, cy + v2y}};  // BL
    float thr = max(32.0f, 0.6f * min(w, h));
    bool used[4] = {false, false, false, false};
    for (int r = 0; r < 4; ++r)
    {
        int best = -1;
        float best_d = 1e30f;
        for (int k = 0; k < 4; ++k)
        {
            if (used[k])
                continue;
            float dx = quad[k][0] - ref[r][0], dy = quad[k][1] - ref[r][1];
            float d = dx * dx + dy * dy;
            if (d < best_d)
            {
                best_d = d;
                best = k;
            }
        }
        if (best < 0 || sqrtf(best_d) > thr)
            return false;
        used[best] = true;
        out_q[r][0] = quad[best][0];
        out_q[r][1] = quad[best][1];
    }
    return true;
}

// Sutherland-Hodgman clip of subject polygon by a single edge
static int clipEdge(const float *sx, const float *sy, int n,
                    float ax, float ay, float bx, float by,
                    float *outx, float *outy)
{
    int m = 0;
    for (int i = 0; i < n; ++i)
    {
        int j = (i + 1) % n;
        float cxi = sx[i], cyi = sy[i];
        float cxj = sx[j], cyj = sy[j];
        float e1 = (bx - ax) * (cyi - ay) - (by - ay) * (cxi - ax);
        float e2 = (bx - ax) * (cyj - ay) - (by - ay) * (cxj - ax);
        if (e1 >= 0)
        {
            outx[m] = cxi;
            outy[m] = cyi;
            m++;
        }
        if ((e1 >= 0) != (e2 >= 0))
        {
            float t = e1 / (e1 - e2);
            outx[m] = cxi + t * (cxj - cxi);
            outy[m] = cyi + t * (cyj - cyi);
            m++;
        }
    }
    return m;
}

static float polygonArea(const float *x, const float *y, int n)
{
    float a = 0.0f;
    for (int i = 0; i < n; ++i)
    {
        int j = (i + 1) % n;
        a += x[i] * y[j] - x[j] * y[i];
    }
    return fabsf(a) * 0.5f;
}

static float obbIou(const ObbCandidate &a, const ObbCandidate &b)
{
    float ax[4], ay[4], bx[4], by[4];
    obbCorners(a, ax, ay);
    obbCorners(b, bx, by);
    float sx[16], sy[16], tmp[16], tmy[16];
    int n = 4;
    for (int i = 0; i < 4; ++i)
    {
        sx[i] = ax[i];
        sy[i] = ay[i];
    }
    for (int e = 0; e < 4; ++e)
    {
        int j = (e + 1) % 4;
        n = clipEdge(sx, sy, n, bx[e], by[e], bx[j], by[j], tmp, tmy);
        for (int k = 0; k < n; ++k)
        {
            sx[k] = tmp[k];
            sy[k] = tmy[k];
        }
        if (n < 3)
            return 0.0f;
    }
    float inter = polygonArea(sx, sy, n);
    float area_a = a.w * a.h, area_b = b.w * b.h;
    return inter / (area_a + area_b - inter + 1e-9f);
}

// Axis-aligned envelope overlap — cheap prefilter before the polygon IoU
static inline float envelopeIou(const ObbCandidate &a, const ObbCandidate &b)
{
    Rect ia = a.env & b.env;
    if (ia.area() <= 0)
        return 0.0f;
    return static_cast<float>(ia.area()) /
           (static_cast<float>(a.env.area() + b.env.area()) - ia.area() + 1e-9f);
}

// Rotated NMS: envelope prefilter (IoU<0.3 skip) + Sutherland-Hodgman polygon IoU
static vector<ObbCandidate> rotatedNms(vector<ObbCandidate> boxes, float iou_thres)
{
    sort(boxes.begin(), boxes.end(),
         [](const ObbCandidate &a, const ObbCandidate &b) { return a.score > b.score; });
    vector<ObbCandidate> keep;
    vector<char> sup(boxes.size(), 0);
    for (size_t i = 0; i < boxes.size(); ++i)
    {
        if (sup[i])
            continue;
        keep.push_back(boxes[i]);
        for (size_t j = i + 1; j < boxes.size(); ++j)
        {
            if (sup[j])
                continue;
            if (envelopeIou(boxes[i], boxes[j]) < 0.3f)
                continue;                      // cheap reject
            if (obbIou(boxes[i], boxes[j]) > iou_thres)
                sup[j] = 1;
        }
    }
    return keep;
}

// Geometry sanity check on rotated dims (long/short side), original frame coords.
// Mirrors isValidPlateGeometry but on the rotated box — the envelope of a tilted
// plate would otherwise fail the HBB aspect check.
static bool isValidObbGeometry(float w, float h, const Rect &env,
                               int frame_width, int frame_height,
                               float confidence)
{
    if (frame_width <= 0 || frame_height <= 0 || w <= 0 || h <= 0)
        return false;

    const int min_width = max(32, frame_width / 50);
    const int min_height = max(10, frame_height / 72);
    if (w < min_width || h < min_height)
        return false;

    const float aspect = w / h;
    if (aspect < 1.60f || aspect > 6.80f)
        return false;

    const float area_ratio = (w * h) / static_cast<float>(frame_width * frame_height);
    if (area_ratio < 0.00020f || area_ratio > 0.12f)
        return false;

    Rect inter = env & Rect(0, 0, frame_width, frame_height);
    if (inter.area() <= 0)
        return false;

    if ((aspect < 1.90f || aspect > 5.80f) && confidence < 0.25f)
        return false;
    if ((w < frame_width * 0.04f || h < frame_height * 0.02f) && confidence < 0.20f)
        return false;

    return true;
}

// Draw rotated box (4 corners from center + rotated half-extents)
static void drawRotatedBox(Mat &img, float cx, float cy, float w, float h, float ang,
                           const Scalar &color, int thickness)
{
    if (img.empty() || w <= 0 || h <= 0)
        return;
    float ca = cosf(ang), sa = sinf(ang);
    const float lx[4] = {w, -w, -w, w};
    const float ly[4] = {h, h, -h, -h};
    Point pts[4];
    for (int i = 0; i < 4; ++i)
    {
        float px = lx[i] * 0.5f, py = ly[i] * 0.5f;
        pts[i] = Point(static_cast<int>(roundf(cx + px * ca - py * sa)),
                       static_cast<int>(roundf(cy + px * sa + py * ca)));
    }
    const Point *pp[1] = {pts};
    int n[1] = {4};
    polylines(img, pp, n, 1, true, color, thickness, LINE_AA);
}

// Rotate-correct an OBB plate region for LPRNet: crop envelope+margin patch,
// warpAffine by +angle about the box center, crop the upright w×h plate area
// (plus margins). Returns BGR crop in the plate's own orientation.
// Sign note: corners = c + R(θ)·(±w/2, ±h/2) puts the long edge at direction θ
// (image coords, y down); getRotationMatrix2D(φ) maps direction θ → θ − φ, so
// φ = +θ uprights the plate. A −θ here doubles the tilt and clips the corners.
static Mat rotateCropPlate(const Mat &frame, const Rect &env, float angle,
                           float w, float h, float margin_w, float margin_h)
{
    if (frame.empty() || env.area() <= 0 || w <= 0 || h <= 0)
        return Mat();

    int pad_x = max(2, static_cast<int>(roundf(w * margin_w)));
    int pad_y = max(2, static_cast<int>(roundf(h * margin_h)));

    Rect patch_rect(env.x - pad_x, env.y - pad_y,
                    env.width + 2 * pad_x, env.height + 2 * pad_y);
    patch_rect &= Rect(0, 0, frame.cols, frame.rows);
    if (patch_rect.area() <= 0)
        return Mat();

    Mat patch = frame(patch_rect);
    float lcx = (env.x + env.width * 0.5f) - patch_rect.x;
    float lcy = (env.y + env.height * 0.5f) - patch_rect.y;

    Mat M = getRotationMatrix2D(Point2f(lcx, lcy),
                                angle * 180.0f / static_cast<float>(CV_PI), 1.0);
    Mat rotated;
    warpAffine(patch, rotated, M, patch.size(), INTER_LINEAR,
               BORDER_CONSTANT, Scalar(114, 114, 114));

    float crop_w = w + 2.0f * pad_x;
    float crop_h = h + 2.0f * pad_y;
    int px = max(0, static_cast<int>(roundf(lcx - crop_w * 0.5f)));
    int py = max(0, static_cast<int>(roundf(lcy - crop_h * 0.5f)));
    int pw = min(rotated.cols - px, static_cast<int>(roundf(crop_w)));
    int ph = min(rotated.rows - py, static_cast<int>(roundf(crop_h)));
    if (pw <= 8 || ph <= 4)
        return Mat();
    return rotated(Rect(px, py, pw, ph)).clone();
}

// Perspective-correct a plate from its 4 corner points (corner-regression model).
// quad[4] = identity order [TL, TR, BR, BL] in original frame coords; it maps onto
// an upright rect of the smoothed box size (plus the same margins as the rotated
// crop), so LPRNet sees the same plate-width distribution as before.
static Mat quadCropPlate(const Mat &frame, const float quad[4][2],
                         float cx, float cy, float w, float h,
                         float margin_w, float margin_h)
{
    if (frame.empty() || w <= 0 || h <= 0)
        return Mat();

    int pad_x = max(2, static_cast<int>(roundf(w * margin_w)));
    int pad_y = max(2, static_cast<int>(roundf(h * margin_h)));
    int cw = static_cast<int>(roundf(w)) + 2 * pad_x;
    int ch = static_cast<int>(roundf(h)) + 2 * pad_y;
    if (cw < 8 || ch < 4)
        return Mat();

    // Patch = bounding rect of quad ∪ destination rect, clamped to the frame
    float min_x = cx - cw * 0.5f, max_x = cx + cw * 0.5f;
    float min_y = cy - ch * 0.5f, max_y = cy + ch * 0.5f;
    for (int k = 0; k < 4; ++k)
    {
        min_x = min(min_x, quad[k][0]);
        max_x = max(max_x, quad[k][0]);
        min_y = min(min_y, quad[k][1]);
        max_y = max(max_y, quad[k][1]);
    }
    Rect patch_rect(static_cast<int>(floorf(min_x)), static_cast<int>(floorf(min_y)),
                    static_cast<int>(ceilf(max_x)) - static_cast<int>(floorf(min_x)) + 1,
                    static_cast<int>(ceilf(max_y)) - static_cast<int>(floorf(min_y)) + 1);
    patch_rect &= Rect(0, 0, frame.cols, frame.rows);
    if (patch_rect.area() <= 0)
        return Mat();

    Mat patch = frame(patch_rect);
    Point2f src[4];
    for (int k = 0; k < 4; ++k)
        src[k] = Point2f(quad[k][0] - patch_rect.x, quad[k][1] - patch_rect.y);
    Point2f dst[4] = {Point2f(0, 0),
                      Point2f(static_cast<float>(cw), 0),
                      Point2f(static_cast<float>(cw), static_cast<float>(ch)),
                      Point2f(0, static_cast<float>(ch))};

    Mat M = getPerspectiveTransform(src, dst);
    Mat out;
    warpPerspective(patch, out, M, Size(cw, ch), INTER_LINEAR,
                    BORDER_CONSTANT, Scalar(114, 114, 114));
    return out;
}

static vector<string> defaultLprnetChars()
{
    // LPRNet V2: 75 classes (74 visible + blank at index 74 which is '-')
    const char *chars[] = {
        "京","沪","津","渝","冀","晋","蒙","辽","吉","黑",      // 0-9   provinces
        "苏","浙","皖","闽","赣","鲁","豫","鄂","湘","粤",       // 10-19
        "桂","琼","川","贵","云","藏","陕","甘","青","宁",       // 20-29
        "新","学","港","澳","警","使","领","应","急","挂",       // 30-39 special
        "0","1","2","3","4","5","6","7","8","9",                 // 40-49 digits
        "A","B","C","D","E","F","G","H","J","K",                 // 50-59 letters (no I/O)
        "L","M","N","P","Q","R","S","T","U","V",                 // 60-69
        "W","X","Y","Z",                                          // 70-73
        "-"};                                                      // 74 blank
    return vector<string>(chars, chars + sizeof(chars) / sizeof(chars[0]));
}

static bool loadLprnetChars(const string &path, vector<string> &chars)
{
    chars.clear();
    ifstream ifs(path.c_str());
    if (!ifs)
        return false;

    string line;
    while (getline(ifs, line))
    {
        if (!line.empty() && line[line.size() - 1] == '\r')
        {
            line.erase(line.size() - 1);
        }
        chars.push_back(line);
    }

    return !chars.empty();
}

static size_t utf8CharLen(unsigned char c)
{
    if ((c & 0x80) == 0)
        return 1;
    if ((c & 0xE0) == 0xC0)
        return 2;
    if ((c & 0xF0) == 0xE0)
        return 3;
    if ((c & 0xF8) == 0xF0)
        return 4;
    return 1;
}

static vector<string> splitUtf8Chars(const string &text)
{
    vector<string> chars;
    for (size_t i = 0; i < text.size();)
    {
        size_t n = utf8CharLen(static_cast<unsigned char>(text[i]));
        if (i + n > text.size())
            n = 1;
        chars.push_back(text.substr(i, n));
        i += n;
    }
    return chars;
}

#ifdef HAVE_DIRECT_FREETYPE
class Utf8FreeTypeRenderer
{
public:
    Utf8FreeTypeRenderer() = default;
    ~Utf8FreeTypeRenderer() { release(); }

    Utf8FreeTypeRenderer(const Utf8FreeTypeRenderer &) = delete;
    Utf8FreeTypeRenderer &operator=(const Utf8FreeTypeRenderer &) = delete;

    bool init(const string &font_path)
    {
        release();
        if (font_path.empty() || !fileExists(font_path))
        {
            cerr << "FreeType font not found: " << font_path << endl;
            return false;
        }

        handle_ = dlopen("libfreetype.so.6", RTLD_LAZY | RTLD_LOCAL);
        if (!handle_)
            handle_ = dlopen("libfreetype.so", RTLD_LAZY | RTLD_LOCAL);
        if (!handle_)
        {
            cerr << "dlopen libfreetype failed: " << dlerror() << endl;
            return false;
        }

        if (!loadSymbol(init_free_type_, "FT_Init_FreeType") ||
            !loadSymbol(new_face_, "FT_New_Face") ||
            !loadSymbol(set_pixel_sizes_, "FT_Set_Pixel_Sizes") ||
            !loadSymbol(load_char_, "FT_Load_Char") ||
            !loadSymbol(done_face_, "FT_Done_Face") ||
            !loadSymbol(done_free_type_, "FT_Done_FreeType"))
        {
            release();
            return false;
        }

        if (init_free_type_(&library_) != 0)
        {
            cerr << "FT_Init_FreeType failed." << endl;
            release();
            return false;
        }

        if (new_face_(library_, font_path.c_str(), 0, &face_) != 0)
        {
            cerr << "FT_New_Face failed: " << font_path << endl;
            release();
            return false;
        }

        cout << "Direct FreeType text renderer enabled: " << font_path << endl;
        return true;
    }

    bool isReady() const { return face_ != nullptr; }

    bool drawText(Mat &img, const string &utf8, Point baseline, int pixel_size, Scalar color)
    {
        if (!face_ || img.empty() || img.type() != CV_8UC3 || utf8.empty())
            return false;
        if (set_pixel_sizes_(face_, 0, static_cast<FT_UInt>(pixel_size)) != 0)
            return false;

        int pen_x = baseline.x;
        int base_y = baseline.y;
        vector<uint32_t> codepoints = decodeUtf8(utf8);

        for (uint32_t cp : codepoints)
        {
            if (cp == '\n')
            {
                pen_x = baseline.x;
                base_y += pixel_size + 4;
                continue;
            }

            if (load_char_(face_, static_cast<FT_ULong>(cp), FT_LOAD_RENDER) != 0)
            {
                pen_x += pixel_size / 2;
                continue;
            }

            FT_GlyphSlot glyph = face_->glyph;
            const FT_Bitmap &bitmap = glyph->bitmap;
            int left = pen_x + glyph->bitmap_left;
            int top = base_y - glyph->bitmap_top;

            blendBitmap(img, bitmap, left, top, color);
            pen_x += static_cast<int>(glyph->advance.x >> 6);
        }

        return true;
    }

private:
    void *handle_ = nullptr;
    FT_Library library_ = nullptr;
    FT_Face face_ = nullptr;

    FT_Error (*init_free_type_)(FT_Library *) = nullptr;
    FT_Error (*new_face_)(FT_Library, const char *, FT_Long, FT_Face *) = nullptr;
    FT_Error (*set_pixel_sizes_)(FT_Face, FT_UInt, FT_UInt) = nullptr;
    FT_Error (*load_char_)(FT_Face, FT_ULong, FT_Int32) = nullptr;
    FT_Error (*done_face_)(FT_Face) = nullptr;
    FT_Error (*done_free_type_)(FT_Library) = nullptr;

    template <typename T>
    bool loadSymbol(T &fn, const char *name)
    {
        fn = reinterpret_cast<T>(dlsym(handle_, name));
        if (!fn)
        {
            cerr << "dlsym failed: " << name << endl;
            return false;
        }
        return true;
    }

    void release()
    {
        if (face_ && done_face_)
        {
            done_face_(face_);
            face_ = nullptr;
        }
        if (library_ && done_free_type_)
        {
            done_free_type_(library_);
            library_ = nullptr;
        }
        if (handle_)
        {
            dlclose(handle_);
            handle_ = nullptr;
        }
        init_free_type_ = nullptr;
        new_face_ = nullptr;
        set_pixel_sizes_ = nullptr;
        load_char_ = nullptr;
        done_face_ = nullptr;
        done_free_type_ = nullptr;
    }

    static vector<uint32_t> decodeUtf8(const string &s)
    {
        vector<uint32_t> out;
        size_t i = 0;
        while (i < s.size())
        {
            unsigned char c = static_cast<unsigned char>(s[i]);
            if (c < 0x80)
            {
                out.push_back(c);
                i++;
            }
            else if ((c & 0xE0) == 0xC0 && i + 1 < s.size())
            {
                out.push_back(((c & 0x1F) << 6) |
                              (static_cast<unsigned char>(s[i + 1]) & 0x3F));
                i += 2;
            }
            else if ((c & 0xF0) == 0xE0 && i + 2 < s.size())
            {
                out.push_back(((c & 0x0F) << 12) |
                              ((static_cast<unsigned char>(s[i + 1]) & 0x3F) << 6) |
                              (static_cast<unsigned char>(s[i + 2]) & 0x3F));
                i += 3;
            }
            else if ((c & 0xF8) == 0xF0 && i + 3 < s.size())
            {
                out.push_back(((c & 0x07) << 18) |
                              ((static_cast<unsigned char>(s[i + 1]) & 0x3F) << 12) |
                              ((static_cast<unsigned char>(s[i + 2]) & 0x3F) << 6) |
                              (static_cast<unsigned char>(s[i + 3]) & 0x3F));
                i += 4;
            }
            else
            {
                i++;
            }
        }
        return out;
    }

    static void blendBitmap(Mat &img, const FT_Bitmap &bitmap, int left, int top, Scalar color)
    {
        for (int row = 0; row < static_cast<int>(bitmap.rows); ++row)
        {
            int y = top + row;
            if (y < 0 || y >= img.rows)
                continue;

            const unsigned char *src = bitmap.pitch >= 0
                                           ? bitmap.buffer + row * bitmap.pitch
                                           : bitmap.buffer + (static_cast<int>(bitmap.rows) - 1 - row) * (-bitmap.pitch);

            Vec3b *dst = img.ptr<Vec3b>(y);
            for (int col = 0; col < static_cast<int>(bitmap.width); ++col)
            {
                int x = left + col;
                if (x < 0 || x >= img.cols)
                    continue;

                float alpha = src[col] / 255.0f;
                if (alpha <= 0.0f)
                    continue;

                for (int ch = 0; ch < 3; ++ch)
                {
                    float v = dst[x][ch] * (1.0f - alpha) + static_cast<float>(color[ch]) * alpha;
                    dst[x][ch] = static_cast<unsigned char>(max(0.0f, min(255.0f, v)));
                }
            }
        }
    }
};
#endif

#ifdef HAVE_DRM_ROI
class LibDrmApi
{
public:
    ~LibDrmApi()
    {
        if (handle_)
            dlclose(handle_);
    }

    bool load()
    {
        if (handle_)
            return true;

        handle_ = dlopen("libdrm.so.2", RTLD_LAZY | RTLD_LOCAL);
        if (!handle_)
            handle_ = dlopen("libdrm.so", RTLD_LAZY | RTLD_LOCAL);
        if (!handle_)
        {
            cerr << "dlopen libdrm failed: " << dlerror() << endl;
            return false;
        }

        return loadSymbol(ModeGetResources, "drmModeGetResources") &&
               loadSymbol(ModeFreeResources, "drmModeFreeResources") &&
               loadSymbol(ModeGetConnector, "drmModeGetConnector") &&
               loadSymbol(ModeFreeConnector, "drmModeFreeConnector") &&
               loadSymbol(ModeGetEncoder, "drmModeGetEncoder") &&
               loadSymbol(ModeFreeEncoder, "drmModeFreeEncoder") &&
               loadSymbol(ModeGetCrtc, "drmModeGetCrtc") &&
               loadSymbol(ModeFreeCrtc, "drmModeFreeCrtc") &&
               loadSymbol(ModeSetCrtc, "drmModeSetCrtc") &&
               loadSymbol(ModePageFlip, "drmModePageFlip") &&
               loadSymbol(HandleEvent, "drmHandleEvent") &&
               loadSymbol(ModeAddFB2, "drmModeAddFB2") &&
               loadSymbol(ModeRmFB, "drmModeRmFB");
    }

    drmModeResPtr (*ModeGetResources)(int fd) = nullptr;
    void (*ModeFreeResources)(drmModeResPtr ptr) = nullptr;
    drmModeConnectorPtr (*ModeGetConnector)(int fd, uint32_t connectorId) = nullptr;
    void (*ModeFreeConnector)(drmModeConnectorPtr ptr) = nullptr;
    drmModeEncoderPtr (*ModeGetEncoder)(int fd, uint32_t encoder_id) = nullptr;
    void (*ModeFreeEncoder)(drmModeEncoderPtr ptr) = nullptr;
    drmModeCrtcPtr (*ModeGetCrtc)(int fd, uint32_t crtcId) = nullptr;
    void (*ModeFreeCrtc)(drmModeCrtcPtr ptr) = nullptr;
    int (*ModeSetCrtc)(int fd, uint32_t crtcId, uint32_t bufferId, uint32_t x, uint32_t y,
                       uint32_t *connectors, int count, drmModeModeInfoPtr mode) = nullptr;
    int (*ModePageFlip)(int fd, uint32_t crtcId, uint32_t fbId, uint32_t flags, void *userData) = nullptr;
    int (*HandleEvent)(int fd, drmEventContextPtr evctx) = nullptr;
    int (*ModeAddFB2)(int fd, uint32_t width, uint32_t height, uint32_t pixel_format,
                      const uint32_t bo_handles[4], const uint32_t pitches[4],
                      const uint32_t offsets[4], uint32_t *buf_id, uint32_t flags) = nullptr;
    int (*ModeRmFB)(int fd, uint32_t bufferId) = nullptr;

private:
    void *handle_ = nullptr;

    template <typename T>
    bool loadSymbol(T &fn, const char *name)
    {
        fn = reinterpret_cast<T>(dlsym(handle_, name));
        if (!fn)
        {
            cerr << "dlsym libdrm failed: " << name << endl;
            return false;
        }
        return true;
    }
};

static LibDrmApi g_drm;

class DrmRoiOutput
{
public:
    DrmRoiOutput() = default;
    ~DrmRoiOutput() { closeDevice(); }

    DrmRoiOutput(const DrmRoiOutput &) = delete;
    DrmRoiOutput &operator=(const DrmRoiOutput &) = delete;

    bool init(const string &card_path,
              uint32_t connector_id,
              int request_w,
              int request_h)
    {
        closeDevice();

        if (!g_drm.load())
        {
            cerr << "libdrm runtime load failed. Install libdrm on RK board." << endl;
            return false;
        }

        fd_ = open(card_path.c_str(), O_RDWR | O_CLOEXEC);
        if (fd_ < 0)
        {
            cerr << "DRM open failed: " << card_path << ", " << strerror(errno) << endl;
            return false;
        }

        res_ = g_drm.ModeGetResources(fd_);
        if (!res_)
        {
            cerr << "drmModeGetResources failed." << endl;
            closeDevice();
            return false;
        }

        conn_ = findConnector(connector_id);
        if (!conn_)
        {
            cerr << "No connected DRM HDMI connector found. Try modetest to check connector id." << endl;
            closeDevice();
            return false;
        }
        conn_id_ = conn_->connector_id;

        if (!chooseMode(request_w, request_h))
        {
            cerr << "No usable DRM mode found." << endl;
            closeDevice();
            return false;
        }
        mode_w_ = mode_.hdisplay;
        mode_h_ = mode_.vdisplay;

        enc_ = findEncoderForConnector(conn_);
        if (!enc_)
        {
            cerr << "No usable DRM encoder found for connector " << conn_id_ << endl;
            closeDevice();
            return false;
        }

        crtc_id_ = findCrtcForEncoder(enc_);
        if (crtc_id_ == 0)
        {
            cerr << "No usable DRM CRTC found." << endl;
            closeDevice();
            return false;
        }

        old_crtc_ = g_drm.ModeGetCrtc(fd_, crtc_id_);

        if (!createDumbBuffer(buf_[0], mode_w_, mode_h_) ||
            !createDumbBuffer(buf_[1], mode_w_, mode_h_))
        {
            closeDevice();
            return false;
        }

        memset(buf_[0].map, 0, buf_[0].size);
        memset(buf_[1].map, 0, buf_[1].size);
        front_ = 0;
        back_ = 1;

        int ret = g_drm.ModeSetCrtc(fd_, crtc_id_, buf_[front_].fb_id, 0, 0, &conn_id_, 1, &mode_);
        if (ret != 0)
        {
            cerr << "drmModeSetCrtc failed: " << strerror(errno) << endl;
            closeDevice();
            return false;
        }

        cout << "DRM ROI output enabled: card=" << card_path
             << ", connector=" << connectorName(conn_)
             << " id=" << conn_id_
             << ", crtc=" << crtc_id_
             << ", mode=" << mode_w_ << "x" << mode_h_
             << " @" << mode_.vrefresh << "Hz"
             << ", pitch=" << buf_[front_].pitch
             << ", output=double-buffer pageflip" << endl;
        return true;
    }

    bool present(const Mat &roi_bgr)
    {
        if (!isOpened() || roi_bgr.empty() || mode_w_ <= 0 || mode_h_ <= 0)
            return false;

        Mat bgr;
        if (roi_bgr.channels() == 1)
            cvtColor(roi_bgr, bgr, COLOR_GRAY2BGR);
        else if (roi_bgr.channels() == 4)
            cvtColor(roi_bgr, bgr, COLOR_BGRA2BGR);
        else
            bgr = roi_bgr;

        Mat canvas(mode_h_, mode_w_, CV_8UC3, Scalar(0, 0, 0));

        // 为了让FPGA锁定稳定输入，HDMI一直输出固定分辨率；ROI按比例放大居中。
        double sx = mode_w_ / static_cast<double>(bgr.cols);
        double sy = mode_h_ / static_cast<double>(bgr.rows);
        double scale = min(sx, sy);
        int rw = max(1, min(mode_w_, static_cast<int>(bgr.cols * scale)));
        int rh = max(1, min(mode_h_, static_cast<int>(bgr.rows * scale)));
        Mat resized;
        resize(bgr, resized, Size(rw, rh), 0, 0, INTER_LINEAR);
        Rect dst((mode_w_ - rw) / 2, (mode_h_ - rh) / 2, rw, rh);
        resized.copyTo(canvas(dst));

        return presentCanvas(canvas);
    }

    // 低延迟快路径：当输入帧尺寸与显示模式完全一致时，跳过每帧一次的
    //   new Mat(mode_h_, mode_w_) + Scalar(0) 全屏清零 + 整帧双线性 resize，
    // 直接把 BGR 写进 dumb buffer。1080p 下这一跳省掉约 6 MB 清零和
    // 200 万像素的缩放。尺寸不匹配时自动退回 present() 的等比缩放居中路径。
    bool presentDirect(const Mat &bgr)
    {
        if (!isOpened() || bgr.empty() || mode_w_ <= 0 || mode_h_ <= 0)
            return false;
        if (bgr.cols == mode_w_ && bgr.rows == mode_h_ &&
            bgr.channels() == 3 && bgr.isContinuous())
            return presentCanvas(bgr);
        return present(bgr);
    }

    int modeWidth() const { return mode_w_; }
    int modeHeight() const { return mode_h_; }

    bool presentTestPattern()
    {
        if (!isOpened() || mode_w_ <= 0 || mode_h_ <= 0)
            return false;

        Mat canvas(mode_h_, mode_w_, CV_8UC3, Scalar(20, 20, 20));
        Scalar bars[] = {
            Scalar(255, 255, 255), Scalar(0, 255, 255), Scalar(255, 255, 0),
            Scalar(0, 255, 0), Scalar(255, 0, 255), Scalar(0, 0, 255),
            Scalar(255, 0, 0), Scalar(0, 0, 0)};
        int bar_w = max(1, mode_w_ / 8);
        for (int i = 0; i < 8; ++i)
        {
            Rect r(i * bar_w, 0, (i == 7) ? mode_w_ - i * bar_w : bar_w, mode_h_);
            rectangle(canvas, r, bars[i], FILLED);
        }

        rectangle(canvas, Rect(20, 20, max(1, mode_w_ - 40), max(1, mode_h_ - 40)),
                  Scalar(0, 255, 0), max(2, mode_w_ / 480));
        putText(canvas, "DRM READY / WAITING ROI", Point(40, max(70, mode_h_ / 10)),
                FONT_HERSHEY_SIMPLEX, max(0.8, mode_w_ / 1400.0), Scalar(0, 0, 0), 4, LINE_AA);
        putText(canvas, "DRM READY / WAITING ROI", Point(40, max(70, mode_h_ / 10)),
                FONT_HERSHEY_SIMPLEX, max(0.8, mode_w_ / 1400.0), Scalar(0, 255, 0), 2, LINE_AA);
        putText(canvas, "RK -> HDMI -> FPGA input", Point(40, max(115, mode_h_ / 10 + 45)),
                FONT_HERSHEY_SIMPLEX, max(0.6, mode_w_ / 1800.0), Scalar(0, 0, 0), 3, LINE_AA);
        putText(canvas, "RK -> HDMI -> FPGA input", Point(40, max(115, mode_h_ / 10 + 45)),
                FONT_HERSHEY_SIMPLEX, max(0.6, mode_w_ / 1800.0), Scalar(255, 255, 255), 1, LINE_AA);

        return presentCanvas(canvas);
    }

    int width() const { return mode_w_; }
    int height() const { return mode_h_; }

private:
    struct Buffer
    {
        uint32_t fb_id = 0;
        uint32_t handle = 0;
        uint32_t pitch = 0;
        uint64_t size = 0;
        void *map = nullptr;
    };

    int fd_ = -1;
    drmModeRes *res_ = nullptr;
    drmModeConnector *conn_ = nullptr;
    drmModeEncoder *enc_ = nullptr;
    drmModeCrtc *old_crtc_ = nullptr;
    drmModeModeInfo mode_{};
    uint32_t conn_id_ = 0;
    uint32_t crtc_id_ = 0;
    Buffer buf_[2];
    int front_ = 0;
    int back_ = 1;
    bool page_flip_done_ = false;
    int mode_w_ = 0;
    int mode_h_ = 0;
    uint64_t stat_frames_ = 0;
    chrono::steady_clock::time_point stat_t0_ = chrono::steady_clock::now();

    bool isOpened() const
    {
        return fd_ >= 0 && buf_[0].map != nullptr && buf_[1].map != nullptr;
    }

    static bool isHdmiConnector(const drmModeConnector *c)
    {
        return c && (c->connector_type == DRM_MODE_CONNECTOR_HDMIA ||
                     c->connector_type == DRM_MODE_CONNECTOR_HDMIB);
    }

    static string connectorName(const drmModeConnector *c)
    {
        if (!c)
            return "unknown";
        string type;
        switch (c->connector_type)
        {
        case DRM_MODE_CONNECTOR_HDMIA:
            type = "HDMI-A";
            break;
        case DRM_MODE_CONNECTOR_HDMIB:
            type = "HDMI-B";
            break;
        case DRM_MODE_CONNECTOR_DVID:
            type = "DVI-D";
            break;
        case DRM_MODE_CONNECTOR_DVII:
            type = "DVI-I";
            break;
        case DRM_MODE_CONNECTOR_DisplayPort:
            type = "DP";
            break;
        case DRM_MODE_CONNECTOR_LVDS:
            type = "LVDS";
            break;
        case DRM_MODE_CONNECTOR_eDP:
            type = "eDP";
            break;
        default:
            type = "connector";
            break;
        }
        return type + "-" + to_string(c->connector_type_id);
    }

    drmModeConnector *findConnector(uint32_t requested_id)
    {
        if (!res_)
            return nullptr;

        if (requested_id != 0)
        {
            drmModeConnector *c = g_drm.ModeGetConnector(fd_, requested_id);
            if (c && c->connection == DRM_MODE_CONNECTED && c->count_modes > 0)
                return c;
            if (c)
                g_drm.ModeFreeConnector(c);
            return nullptr;
        }

        drmModeConnector *first_connected = nullptr;
        for (int i = 0; i < res_->count_connectors; ++i)
        {
            drmModeConnector *c = g_drm.ModeGetConnector(fd_, res_->connectors[i]);
            if (!c)
                continue;

            bool connected = (c->connection == DRM_MODE_CONNECTED && c->count_modes > 0);
            if (connected && isHdmiConnector(c))
            {
                if (first_connected)
                    g_drm.ModeFreeConnector(first_connected);
                return c;
            }

            if (connected && !first_connected)
            {
                first_connected = c;
            }
            else
            {
                g_drm.ModeFreeConnector(c);
            }
        }
        return first_connected;
    }

    bool chooseMode(int request_w, int request_h)
    {
        if (!conn_ || conn_->count_modes <= 0)
            return false;

        int best = 0;
        bool found = false;
        if (request_w > 0 && request_h > 0)
        {
            for (int i = 0; i < conn_->count_modes; ++i)
            {
                drmModeModeInfo m = conn_->modes[i];
                if (m.hdisplay == request_w && m.vdisplay == request_h &&
                    !(m.flags & DRM_MODE_FLAG_INTERLACE))
                {
                    best = i;
                    found = true;
                    break;
                }
            }
        }

        if (!found)
        {
            for (int i = 0; i < conn_->count_modes; ++i)
            {
                drmModeModeInfo m = conn_->modes[i];
                if (m.hdisplay == 1280 && m.vdisplay == 720 &&
                    m.vrefresh >= 59 && m.vrefresh <= 61 &&
                    !(m.flags & DRM_MODE_FLAG_INTERLACE))
                {
                    best = i;
                    found = true;
                    break;
                }
            }
        }

        if (!found)
        {
            for (int i = 0; i < conn_->count_modes; ++i)
            {
                drmModeModeInfo m = conn_->modes[i];
                if (m.hdisplay == 1920 && m.vdisplay == 1080 &&
                    m.vrefresh >= 59 && m.vrefresh <= 61 &&
                    !(m.flags & DRM_MODE_FLAG_INTERLACE))
                {
                    best = i;
                    found = true;
                    break;
                }
            }
        }

        if (!found)
        {
            for (int i = 0; i < conn_->count_modes; ++i)
            {
                if (conn_->modes[i].type & DRM_MODE_TYPE_PREFERRED)
                {
                    best = i;
                    break;
                }
            }
        }

        mode_ = conn_->modes[best];
        if (request_w > 0 && request_h > 0 &&
            (mode_.hdisplay != request_w || mode_.vdisplay != request_h))
        {
            cerr << "Requested DRM mode " << request_w << "x" << request_h
                 << " not found, using " << mode_.hdisplay << "x" << mode_.vdisplay << endl;
        }
        return true;
    }

    drmModeEncoder *findEncoderForConnector(drmModeConnector *c)
    {
        if (!c)
            return nullptr;
        if (c->encoder_id)
        {
            drmModeEncoder *e = g_drm.ModeGetEncoder(fd_, c->encoder_id);
            if (e)
                return e;
        }
        for (int i = 0; i < c->count_encoders; ++i)
        {
            drmModeEncoder *e = g_drm.ModeGetEncoder(fd_, c->encoders[i]);
            if (e)
                return e;
        }
        return nullptr;
    }

    uint32_t findCrtcForEncoder(drmModeEncoder *e)
    {
        if (!res_ || !e)
            return 0;
        if (e->crtc_id)
            return e->crtc_id;
        for (int i = 0; i < res_->count_crtcs; ++i)
        {
            if (e->possible_crtcs & (1 << i))
                return res_->crtcs[i];
        }
        return 0;
    }

    bool createDumbBuffer(Buffer &b, int w, int h)
    {
        drm_mode_create_dumb creq{};
        creq.width = static_cast<uint32_t>(w);
        creq.height = static_cast<uint32_t>(h);
        creq.bpp = 32;

        if (ioctl(fd_, DRM_IOCTL_MODE_CREATE_DUMB, &creq) != 0)
        {
            cerr << "DRM_IOCTL_MODE_CREATE_DUMB failed: " << strerror(errno) << endl;
            return false;
        }
        b.handle = creq.handle;
        b.pitch = creq.pitch;
        b.size = creq.size;

        uint32_t handles[4] = {b.handle, 0, 0, 0};
        uint32_t pitches[4] = {b.pitch, 0, 0, 0};
        uint32_t offsets[4] = {0, 0, 0, 0};
        int ret = g_drm.ModeAddFB2(fd_, w, h, DRM_FORMAT_XRGB8888,
                                   handles, pitches, offsets, &b.fb_id, 0);
        if (ret != 0)
        {
            cerr << "drmModeAddFB2 failed: " << strerror(errno) << endl;
            destroyDumbBuffer(b);
            return false;
        }

        drm_mode_map_dumb mreq{};
        mreq.handle = b.handle;
        if (ioctl(fd_, DRM_IOCTL_MODE_MAP_DUMB, &mreq) != 0)
        {
            cerr << "DRM_IOCTL_MODE_MAP_DUMB failed: " << strerror(errno) << endl;
            destroyDumbBuffer(b);
            return false;
        }

        b.map = mmap(nullptr, b.size, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, mreq.offset);
        if (b.map == MAP_FAILED)
        {
            cerr << "mmap DRM dumb buffer failed: " << strerror(errno) << endl;
            b.map = nullptr;
            destroyDumbBuffer(b);
            return false;
        }
        return true;
    }

    void destroyDumbBuffer(Buffer &b)
    {
        if (b.map)
        {
            munmap(b.map, b.size);
            b.map = nullptr;
        }
        if (fd_ >= 0 && b.fb_id)
        {
            g_drm.ModeRmFB(fd_, b.fb_id);
            b.fb_id = 0;
        }
        if (fd_ >= 0 && b.handle)
        {
            drm_mode_destroy_dumb dreq{};
            dreq.handle = b.handle;
            ioctl(fd_, DRM_IOCTL_MODE_DESTROY_DUMB, &dreq);
            b.handle = 0;
        }
        b.pitch = 0;
        b.size = 0;
    }

    bool presentCanvas(const Mat &canvas)
    {
        if (!isOpened() || canvas.empty())
            return false;

        Mat screen;
        if (canvas.cols == mode_w_ && canvas.rows == mode_h_ && canvas.type() == CV_8UC3)
        {
            screen = canvas;
        }
        else
        {
            resize(canvas, screen, Size(mode_w_, mode_h_), 0, 0, INTER_LINEAR);
        }

        Buffer &dst = buf_[back_];
        // DRM_FORMAT_XRGB8888 在小端内存中按 B,G,R,X 写入。
        for (int y = 0; y < mode_h_; ++y)
        {
            const Vec3b *src = screen.ptr<Vec3b>(y);
            uint8_t *dst_row = static_cast<uint8_t *>(dst.map) + static_cast<size_t>(y) * dst.pitch;
            for (int x = 0; x < mode_w_; ++x)
            {
                dst_row[x * 4 + 0] = src[x][0];
                dst_row[x * 4 + 1] = src[x][1];
                dst_row[x * 4 + 2] = src[x][2];
                dst_row[x * 4 + 3] = 0x00;
            }
        }

        return flipBackBuffer();
    }

    static void pageFlipHandler(int, unsigned int, unsigned int, unsigned int, void *data)
    {
        DrmRoiOutput *self = static_cast<DrmRoiOutput *>(data);
        if (self)
            self->page_flip_done_ = true;
    }

    bool waitPageFlip()
    {
        drmEventContext ev{};
        ev.version = DRM_EVENT_CONTEXT_VERSION;
        ev.page_flip_handler = pageFlipHandler;

        while (!page_flip_done_ && g_running)
        {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(fd_, &fds);

            timeval tv{};
            tv.tv_sec = 1;
            tv.tv_usec = 0;

            int ret = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
            if (ret < 0)
            {
                if (errno == EINTR)
                    continue;
                cerr << "select waiting DRM page flip failed: " << strerror(errno) << endl;
                return false;
            }
            if (ret == 0)
            {
                cerr << "DRM page flip wait timeout." << endl;
                return false;
            }
            if (FD_ISSET(fd_, &fds))
            {
                if (g_drm.HandleEvent(fd_, &ev) != 0)
                {
                    cerr << "drmHandleEvent failed: " << strerror(errno) << endl;
                    return false;
                }
            }
        }
        return page_flip_done_;
    }

    bool flipBackBuffer()
    {
        page_flip_done_ = false;
        int ret = g_drm.ModePageFlip(fd_, crtc_id_, buf_[back_].fb_id,
                                     DRM_MODE_PAGE_FLIP_EVENT, this);
        if (ret != 0)
        {
            cerr << "drmModePageFlip failed: " << strerror(errno)
                 << ", fallback to drmModeSetCrtc." << endl;
            ret = g_drm.ModeSetCrtc(fd_, crtc_id_, buf_[back_].fb_id,
                                    0, 0, &conn_id_, 1, &mode_);
            if (ret != 0)
            {
                cerr << "drmModeSetCrtc fallback failed: " << strerror(errno) << endl;
                return false;
            }
        }
        else if (!waitPageFlip())
        {
            return false;
        }

        std::swap(front_, back_);
        statOutput();
        return true;
    }

    void statOutput()
    {
        stat_frames_++;
        auto now = chrono::steady_clock::now();
        double sec = chrono::duration_cast<chrono::duration<double>>(now - stat_t0_).count();
        if (sec >= 2.0)
        {
            cout << "HDMI ROI output FPS: " << (stat_frames_ / sec) << endl;
            stat_frames_ = 0;
            stat_t0_ = now;
        }
    }

    void closeDevice()
    {
        if (fd_ >= 0 && old_crtc_)
        {
            g_drm.ModeSetCrtc(fd_, old_crtc_->crtc_id, old_crtc_->buffer_id,
                              old_crtc_->x, old_crtc_->y,
                              &conn_id_, 1, &old_crtc_->mode);
        }
        destroyDumbBuffer(buf_[0]);
        destroyDumbBuffer(buf_[1]);
        if (old_crtc_)
        {
            g_drm.ModeFreeCrtc(old_crtc_);
            old_crtc_ = nullptr;
        }
        if (enc_)
        {
            g_drm.ModeFreeEncoder(enc_);
            enc_ = nullptr;
        }
        if (conn_)
        {
            g_drm.ModeFreeConnector(conn_);
            conn_ = nullptr;
        }
        if (res_)
        {
            g_drm.ModeFreeResources(res_);
            res_ = nullptr;
        }
        if (fd_ >= 0)
        {
            close(fd_);
            fd_ = -1;
        }
    }
};
#endif

// ============================================================
// 第二块采集卡专用：纯 V4L2 读取 FPGA HDMI OUT 返回的边缘图
// 说明：不要使用 cv::VideoCapture，避免依赖 opencv_videoio。
// 支持 MJPG/JPEG/YUYV/UYVY/BGR3/RGB3。
// ============================================================
class FpgaReturnV4L2Capture
{
public:
    FpgaReturnV4L2Capture() = default;
    ~FpgaReturnV4L2Capture() { closeDevice(); }

    FpgaReturnV4L2Capture(const FpgaReturnV4L2Capture &) = delete;
    FpgaReturnV4L2Capture &operator=(const FpgaReturnV4L2Capture &) = delete;

    bool openDevice(const string &video_dev, int width, int height, int fps, const string &fourcc)
    {
        closeDevice();

        string dev = normalizeDevice(video_dev);
        fd_ = open(dev.c_str(), O_RDWR | O_NONBLOCK, 0);
        if (fd_ < 0)
        {
            cerr << "Failed to open FPGA return capture device: " << dev
                 << ", " << strerror(errno) << endl;
            return false;
        }

        v4l2_capability cap{};
        if (!xioctl(VIDIOC_QUERYCAP, &cap))
        {
            cerr << "FPGA return VIDIOC_QUERYCAP failed: " << strerror(errno) << endl;
            closeDevice();
            return false;
        }

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) || !(cap.capabilities & V4L2_CAP_STREAMING))
        {
            cerr << "FPGA return device does not support V4L2 capture streaming: " << dev << endl;
            closeDevice();
            return false;
        }

        memset(&fmt_, 0, sizeof(fmt_));
        fmt_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt_.fmt.pix.width = width > 0 ? static_cast<uint32_t>(width) : 1280;
        fmt_.fmt.pix.height = height > 0 ? static_cast<uint32_t>(height) : 720;
        fmt_.fmt.pix.pixelformat = fourccToPixelFormat(fourcc);
        fmt_.fmt.pix.field = V4L2_FIELD_ANY;

        if (!xioctl(VIDIOC_S_FMT, &fmt_))
        {
            cerr << "FPGA return VIDIOC_S_FMT failed for " << fourcc
                 << ": " << strerror(errno) << endl;
            closeDevice();
            return false;
        }

        if (fps > 0)
        {
            v4l2_streamparm parm{};
            parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (xioctl(VIDIOC_G_PARM, &parm))
            {
                parm.parm.capture.timeperframe.numerator = 1;
                parm.parm.capture.timeperframe.denominator = static_cast<uint32_t>(fps);
                xioctl(VIDIOC_S_PARM, &parm);
            }
        }

        if (!initMmap())
        {
            closeDevice();
            return false;
        }

        for (size_t i = 0; i < buffers_.size(); ++i)
        {
            v4l2_buffer buf{};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = static_cast<uint32_t>(i);
            if (!xioctl(VIDIOC_QBUF, &buf))
            {
                cerr << "FPGA return VIDIOC_QBUF failed: " << strerror(errno) << endl;
                closeDevice();
                return false;
            }
        }

        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (!xioctl(VIDIOC_STREAMON, &type))
        {
            cerr << "FPGA return VIDIOC_STREAMON failed: " << strerror(errno) << endl;
            closeDevice();
            return false;
        }
        streaming_ = true;

        cout << "FPGA return capture opened by pure V4L2: " << dev
             << ", actual=" << fmt_.fmt.pix.width << "x" << fmt_.fmt.pix.height
             << ", fourcc=" << pixelFormatToString(fmt_.fmt.pix.pixelformat) << endl;
        return true;
    }

    bool read(Mat &frame, int timeout_ms)
    {
        frame.release();
        if (fd_ < 0 || !streaming_)
            return false;

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_, &fds);

        timeval tv{};
        int t = max(1, timeout_ms);
        tv.tv_sec = t / 1000;
        tv.tv_usec = (t % 1000) * 1000;

        int ret = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
        if (ret <= 0)
            return false;

        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (!xioctl(VIDIOC_DQBUF, &buf))
        {
            if (errno != EAGAIN)
            {
                cerr << "FPGA return VIDIOC_DQBUF failed: " << strerror(errno) << endl;
            }
            return false;
        }

        if (buf.index >= buffers_.size())
        {
            cerr << "FPGA return invalid buffer index: " << buf.index << endl;
            return false;
        }

        size_t used = buf.bytesused;
        if (used == 0 || used > buffers_[buf.index].length)
        {
            used = buffers_[buf.index].length;
        }

        bool ok = used > 0 && convertBufferToMat(buffers_[buf.index].start, used, frame);

        if (!xioctl(VIDIOC_QBUF, &buf))
        {
            cerr << "FPGA return VIDIOC_QBUF requeue failed: " << strerror(errno) << endl;
            return false;
        }

        return ok && !frame.empty();
    }

private:
    struct Buffer
    {
        void *start = nullptr;
        size_t length = 0;
    };

    int fd_ = -1;
    bool streaming_ = false;
    v4l2_format fmt_{};
    vector<Buffer> buffers_;

    bool xioctl(unsigned long request, void *arg)
    {
        int ret;
        do
        {
            ret = ioctl(fd_, request, arg);
        } while (ret == -1 && errno == EINTR);
        return ret != -1;
    }

    bool initMmap()
    {
        v4l2_requestbuffers req{};
        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (!xioctl(VIDIOC_REQBUFS, &req))
        {
            cerr << "FPGA return VIDIOC_REQBUFS failed: " << strerror(errno) << endl;
            return false;
        }
        if (req.count < 2)
        {
            cerr << "FPGA return insufficient mmap buffers." << endl;
            return false;
        }

        buffers_.resize(req.count);
        for (uint32_t i = 0; i < req.count; ++i)
        {
            v4l2_buffer buf{};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if (!xioctl(VIDIOC_QUERYBUF, &buf))
            {
                cerr << "FPGA return VIDIOC_QUERYBUF failed: " << strerror(errno) << endl;
                return false;
            }

            buffers_[i].length = buf.length;
            buffers_[i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE,
                                     MAP_SHARED, fd_, buf.m.offset);
            if (buffers_[i].start == MAP_FAILED)
            {
                buffers_[i].start = nullptr;
                cerr << "FPGA return mmap failed: " << strerror(errno) << endl;
                return false;
            }
        }
        return true;
    }

    bool convertBufferToMat(void *data, size_t size, Mat &frame)
    {
        frame.release();
        if (!data || size == 0)
            return false;

        int width = static_cast<int>(fmt_.fmt.pix.width);
        int height = static_cast<int>(fmt_.fmt.pix.height);
        uint32_t pixfmt = fmt_.fmt.pix.pixelformat;

        if (pixfmt == V4L2_PIX_FMT_MJPEG || pixfmt == V4L2_PIX_FMT_JPEG)
        {
            try
            {
                Mat encoded(1, static_cast<int>(size), CV_8UC1, data);
                frame = imdecode(encoded, IMREAD_COLOR);
            }
            catch (const cv::Exception &e)
            {
                cerr << "FPGA return MJPEG imdecode failed, bytes=" << size
                     << ": " << e.what() << endl;
                frame.release();
                return false;
            }
            return !frame.empty();
        }

        if (pixfmt == V4L2_PIX_FMT_YUYV)
        {
            if (size < static_cast<size_t>(width * height * 2))
                return false;
            Mat yuv(height, width, CV_8UC2, data);
            cvtColor(yuv, frame, COLOR_YUV2BGR_YUYV);
            return !frame.empty();
        }

        if (pixfmt == V4L2_PIX_FMT_UYVY)
        {
            if (size < static_cast<size_t>(width * height * 2))
                return false;
            Mat yuv(height, width, CV_8UC2, data);
            cvtColor(yuv, frame, COLOR_YUV2BGR_UYVY);
            return !frame.empty();
        }

        if (pixfmt == V4L2_PIX_FMT_BGR24)
        {
            if (size < static_cast<size_t>(width * height * 3))
                return false;
            Mat bgr(height, width, CV_8UC3, data);
            frame = bgr.clone();
            return true;
        }

        if (pixfmt == V4L2_PIX_FMT_RGB24)
        {
            if (size < static_cast<size_t>(width * height * 3))
                return false;
            Mat rgb(height, width, CV_8UC3, data);
            cvtColor(rgb, frame, COLOR_RGB2BGR);
            return !frame.empty();
        }

        cerr << "Unsupported FPGA return V4L2 pixel format: "
             << pixelFormatToString(pixfmt) << endl;
        return false;
    }

    void closeDevice()
    {
        if (fd_ >= 0 && streaming_)
        {
            v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            xioctl(VIDIOC_STREAMOFF, &type);
            streaming_ = false;
        }

        for (Buffer &b : buffers_)
        {
            if (b.start)
            {
                munmap(b.start, b.length);
                b.start = nullptr;
                b.length = 0;
            }
        }
        buffers_.clear();

        if (fd_ >= 0)
        {
            close(fd_);
            fd_ = -1;
        }
    }

    static uint32_t fourccToPixelFormat(const string &fourcc)
    {
        string f = fourcc;
        transform(f.begin(), f.end(), f.begin(), [](unsigned char c)
                  { return static_cast<char>(toupper(c)); });

        if (f == "MJPG" || f == "MJPEG")
            return V4L2_PIX_FMT_MJPEG;
        if (f == "JPEG")
            return V4L2_PIX_FMT_JPEG;
        if (f == "YUYV")
            return V4L2_PIX_FMT_YUYV;
        if (f == "UYVY")
            return V4L2_PIX_FMT_UYVY;
        if (f == "BGR3")
            return V4L2_PIX_FMT_BGR24;
        if (f == "RGB3")
            return V4L2_PIX_FMT_RGB24;
        if (f.size() >= 4)
            return v4l2_fourcc(f[0], f[1], f[2], f[3]);
        return V4L2_PIX_FMT_MJPEG;
    }

    static string pixelFormatToString(uint32_t pixfmt)
    {
        char s[5] = {
            static_cast<char>(pixfmt & 0xFF),
            static_cast<char>((pixfmt >> 8) & 0xFF),
            static_cast<char>((pixfmt >> 16) & 0xFF),
            static_cast<char>((pixfmt >> 24) & 0xFF),
            0};
        return string(s);
    }

    static string normalizeDevice(const string &video_dev)
    {
        bool numeric = !video_dev.empty() &&
                       all_of(video_dev.begin(), video_dev.end(), [](unsigned char c)
                              { return isdigit(c); });
        if (numeric)
            return "/dev/video" + video_dev;
        return video_dev;
    }
};

class FpgaRoiProcessor
{
public:
    bool init(const string &mode,
              const string &exchange_dir,
              int timeout_ms,
              const string &fpga_video_dev,
              int fpga_width,
              int fpga_height,
              int fpga_fps,
              const string &fpga_fourcc,
              const string &drm_card,
              uint32_t drm_connector_id,
              int drm_width,
              int drm_height,
              int settle_ms)
    {
        mode_ = mode;
        exchange_dir_ = exchange_dir;
        timeout_ms_ = max(1, timeout_ms);
        fpga_video_dev_ = fpga_video_dev;
        fpga_width_ = fpga_width;
        fpga_height_ = fpga_height;
        fpga_fps_ = fpga_fps;
        fpga_fourcc_ = fpga_fourcc;
        settle_ms_ = max(0, settle_ms);

        if (mode_ != "sw-edge" && mode_ != "bypass" && mode_ != "file" && mode_ != "hdmi")
        {
            cerr << "Unsupported --fpga-mode: " << mode_
                 << ". Use sw-edge, bypass, file, or hdmi." << endl;
            return false;
        }

        if (mode_ == "hdmi")
        {
#ifndef HAVE_DRM_ROI
            cerr << "--fpga-mode hdmi needs HAVE_DRM_ROI. Rebuild with -DHAVE_DRM_ROI and link -ldrm." << endl;
            return false;
#else
            if (!drm_.init(drm_card, drm_connector_id, drm_width, drm_height))
                return false;
            if (!drm_.presentTestPattern())
            {
                cerr << "Failed to present initial DRM test pattern." << endl;
                return false;
            }
            if (!openFpgaCapture())
                return false;
            cout << "FPGA HDMI mode enabled." << endl;
            cout << "  RK DRM HDMI OUT -> FPGA HDMI IN: " << drm_card << endl;
            cout << "  FPGA HDMI OUT -> RK second capture card: " << fpga_video_dev_ << endl;
#endif
        }
        else if (mode_ == "file")
        {
            if (exchange_dir_.empty())
                exchange_dir_ = "/tmp/rk_fpga_roi";
            mkdir(exchange_dir_.c_str(), 0777);
            cout << "FPGA file exchange mode enabled. Directory: " << exchange_dir_ << endl;
            cout << "  RK writes: " << exchange_dir_ << "/roi_input.jpg" << endl;
            cout << "  FPGA should write: " << exchange_dir_ << "/edge_output.jpg" << endl;
        }
        else if (mode_ == "sw-edge")
        {
            cout << "FPGA mode: sw-edge. Using software Sobel/Canny to simulate FPGA edge output." << endl;
        }
        else
        {
            cout << "FPGA mode: bypass. ROI will be sent directly to OCR recognizer." << endl;
        }

        return true;
    }

    bool process(const Mat &roi_bgr, Mat &fpga_out)
    {
        fpga_out.release();
        if (roi_bgr.empty())
            return false;

        if (mode_ == "bypass")
        {
            fpga_out = roi_bgr.clone();
            return true;
        }

        if (mode_ == "hdmi")
        {
            if (processByHdmi(roi_bgr, fpga_out))
                return true;
            cerr << "FPGA HDMI processing failed or timeout, fallback to sw-edge." << endl;
            processBySoftwareEdge(roi_bgr, fpga_out);
            return !fpga_out.empty();
        }

        if (mode_ == "file")
        {
            if (processByFileExchange(roi_bgr, fpga_out))
                return true;
            cerr << "FPGA file exchange failed or timeout, fallback to sw-edge." << endl;
            processBySoftwareEdge(roi_bgr, fpga_out);
            return !fpga_out.empty();
        }

        processBySoftwareEdge(roi_bgr, fpga_out);
        return !fpga_out.empty();
    }

private:
    string mode_ = "sw-edge";
    string exchange_dir_ = "/tmp/rk_fpga_roi";
    int timeout_ms_ = 30;
    string fpga_video_dev_ = "/dev/video2";
    int fpga_width_ = 1280;
    int fpga_height_ = 720;
    int fpga_fps_ = 30;
    string fpga_fourcc_ = "MJPG";
    int settle_ms_ = 20;
    FpgaReturnV4L2Capture fpga_cap_;

#ifdef HAVE_DRM_ROI
    DrmRoiOutput drm_;
#endif

    bool openFpgaCapture()
    {
        bool ok = fpga_cap_.openDevice(fpga_video_dev_, fpga_width_, fpga_height_, fpga_fps_, fpga_fourcc_);
        if (!ok)
        {
            cerr << "Failed to open FPGA return capture device by pure V4L2: "
                 << fpga_video_dev_ << endl;
            return false;
        }

        cout << "FPGA return capture request: " << fpga_video_dev_
             << ", request=" << fpga_width_ << "x" << fpga_height_
             << " @" << fpga_fps_ << ", fourcc=" << fpga_fourcc_ << endl;
        return true;
    }

    static void processBySoftwareEdge(const Mat &roi_bgr, Mat &edge_bgr)
    {
        Mat gray;
        if (roi_bgr.channels() == 1)
            gray = roi_bgr;
        else
            cvtColor(roi_bgr, gray, COLOR_BGR2GRAY);

        Mat blur_img;
        GaussianBlur(gray, blur_img, Size(3, 3), 0, 0);

        Mat grad_x, grad_y, abs_x, abs_y, sobel;
        Sobel(blur_img, grad_x, CV_16S, 1, 0, 3);
        Sobel(blur_img, grad_y, CV_16S, 0, 1, 3);
        convertScaleAbs(grad_x, abs_x);
        convertScaleAbs(grad_y, abs_y);
        addWeighted(abs_x, 0.65, abs_y, 0.35, 0, sobel);

        Mat edge;
        threshold(sobel, edge, 0, 255, THRESH_BINARY | THRESH_OTSU);

        Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 1));
        morphologyEx(edge, edge, MORPH_CLOSE, kernel);

        cvtColor(edge, edge_bgr, COLOR_GRAY2BGR);
    }

    bool processByHdmi(const Mat &roi_bgr, Mat &fpga_out)
    {
#ifndef HAVE_DRM_ROI
        (void)roi_bgr;
        (void)fpga_out;
        return false;
#else
        if (!drm_.present(roi_bgr))
            return false;
        if (settle_ms_ > 0)
            this_thread::sleep_for(chrono::milliseconds(settle_ms_));

        Mat frame;
        auto t0 = chrono::steady_clock::now();
        bool got = false;

        while (g_running)
        {
            // 抓几帧，只保留最新帧，尽量避免读到上一块ROI对应的旧边缘图。
            for (int i = 0; i < 3; ++i)
            {
                Mat tmp;
                if (fpga_cap_.read(tmp, max(1, timeout_ms_ / 3)) && !tmp.empty())
                {
                    frame = tmp;
                    got = true;
                }
            }
            if (got)
                break;

            auto now = chrono::steady_clock::now();
            int ms = static_cast<int>(chrono::duration_cast<chrono::milliseconds>(now - t0).count());
            if (ms >= timeout_ms_)
                break;
            this_thread::sleep_for(chrono::milliseconds(1));
        }

        if (!got || frame.empty())
            return false;

        // FPGA通常会把边缘图铺满固定HDMI画面。这里不再强行反裁剪，直接交给OCR预处理。
        if (frame.channels() == 1)
            cvtColor(frame, fpga_out, COLOR_GRAY2BGR);
        else if (frame.channels() == 4)
            cvtColor(frame, fpga_out, COLOR_BGRA2BGR);
        else
            fpga_out = frame.clone();
        return true;
#endif
    }

    bool processByFileExchange(const Mat &roi_bgr, Mat &fpga_out)
    {
        string in_path = exchange_dir_ + "/roi_input.jpg";
        string out_path = exchange_dir_ + "/edge_output.jpg";
        string busy_path = exchange_dir_ + "/busy.flag";

        unlink(out_path.c_str());

        vector<int> params;
        params.push_back(IMWRITE_JPEG_QUALITY);
        params.push_back(95);
        if (!imwrite(in_path, roi_bgr, params))
        {
            cerr << "Failed to write ROI for FPGA: " << in_path << endl;
            return false;
        }

        FILE *f = fopen(busy_path.c_str(), "w");
        if (f)
        {
            fprintf(f, "1\n");
            fclose(f);
        }

        auto t0 = chrono::steady_clock::now();
        while (g_running)
        {
            if (fileExists(out_path))
            {
                Mat out = imread(out_path, IMREAD_COLOR);
                if (!out.empty())
                {
                    fpga_out = out;
                    return true;
                }
            }
            auto now = chrono::steady_clock::now();
            int ms = static_cast<int>(chrono::duration_cast<chrono::milliseconds>(now - t0).count());
            if (ms >= timeout_ms_)
                break;
            this_thread::sleep_for(chrono::milliseconds(1));
        }
        return false;
    }
};

class PlateRecognizer
{
    struct OcrResult
    {
        string text;
        string raw_text;
        vector<int> labels;
        float quality = -1000.0f;
        float avg_margin = 0.0f;
        float avg_confidence = 0.0f;
        string color;
        float color_score = 0.0f;
        Mat crop;
        Mat input;
        Rect box;
        string tag;
        OcrDebugMeta debug;
    };

    struct OcrTrack
    {
        int id = -1;
        Rect box;
        deque<string> history;
        string stable;
        deque<string> color_history;
        string stable_color;
        chrono::steady_clock::time_point last_seen;
    };

public:
    bool init(const string &detect_model,
              const string &recog_model,
              const string &font_path,
              float conf_thres,
              float nms_thres,
              float ocr_pad_x,
              float ocr_pad_y,
              int ocr_vote_window,
              int ocr_min_votes,
              const string &debug_ocr_dir,
              const string &obb_jitter_log,
              const string &ocr_chars_path,
              OcrBackend ocr_backend,
              bool debug_ocr,
              bool expand_ocr_roi,
              const string &fpga_mode,
              const string &fpga_dir,
              int fpga_timeout_ms,
              const string &fpga_video_dev,
              int fpga_width,
              int fpga_height,
              int fpga_fps,
              const string &fpga_fourcc,
              const string &drm_card,
              uint32_t drm_connector_id,
              int drm_width,
              int drm_height,
              int fpga_settle_ms,
              bool yolo_only,
              bool profile_yolo)
    {
        yolo_only_ = yolo_only;
        profile_yolo_ = profile_yolo;
        ocr_backend_ = ocr_backend;
        last_yolo_profile_.reset();
        // Built-in false-positive suppression: do not allow an excessively low
        // detector threshold even when an old launch script still passes --conf 0.15.
        // (YOLOv8-OBB single-class plate recommendation is 0.25, so floor at 0.20.)
        conf_thres_ = max(0.20f, conf_thres);
        nms_thres_ = nms_thres;
        ocr_pad_x_ = max(0.0f, ocr_pad_x);
        ocr_pad_y_ = max(0.0f, ocr_pad_y);
        ocr_vote_window_ = max(1, ocr_vote_window);
        ocr_min_votes_ = max(1, min(ocr_vote_window_, ocr_min_votes));
        debug_ocr_dir_ = debug_ocr_dir;
        debug_ocr_ = debug_ocr || !debug_ocr_dir_.empty();
        if (debug_ocr_ && debug_ocr_dir_.empty())
        {
            debug_ocr_dir_ = "/tmp/ocr_debug";
        }

        // Raw OBB geometry dump for jitter quantification (--obb-jitter-log).
        // Logs the *unfiltered* per-frame decode so the jitter amplitude can be
        // measured before choosing a temporal filter. Off by default.
        obb_jitter_log_ = obb_jitter_log;
        if (!obb_jitter_log_.empty())
        {
            obb_jitter_file_.open(obb_jitter_log_.c_str(), ios::out | ios::trunc);
            if (obb_jitter_file_.is_open())
                obb_jitter_file_ << "frame,track,cx,cy,ang,w,h,conf\n";
            else
                cerr << "OBB jitter log open failed: " << obb_jitter_log_ << endl;
        }
        expand_ocr_roi_ = expand_ocr_roi;
        fpga_mode_ = fpga_mode;

        cout << "Plate candidate filter: conf>=" << conf_thres_
             << ", aspect=1.60..6.80, OCR format=valid 7/8-char Chinese plate"
             << endl;
        cout << "OBB anti-jitter: OneEuro (min_cutoff=1.2Hz, beta=0.03, d_cutoff=1.0Hz)" << endl;

        if (!detector_.init(detect_model))
            return false;
        cout << "Detector preprocessing: uint8 RGB using model input layout; RKNN model handles /255 normalize" << endl;
        if (yolo_only_)
        {
            cout << "YOLO-only mode: OCR, FPGA ROI, and plate voting are disabled." << endl;
            return true;
        }

        if (!recognizer_.init(recog_model))
            return false;

        cout << "OCR backend: " << ocrBackendName(ocr_backend_) << endl;
        cout << "OCR model: " << recog_model << endl;

        if (ocr_backend_ == OcrBackend::LPRNET)
        {
            if (!loadLprnetChars(ocr_chars_path, ocr_chars_))
            {
                ocr_chars_ = defaultLprnetChars();
                cerr << "OCR chars file not found or empty: " << ocr_chars_path
                     << ", using Python fallback chars=" << ocr_chars_.size() << endl;
            }
            else
            {
                cout << "OCR chars loaded: " << ocr_chars_path
                     << ", chars=" << ocr_chars_.size() << endl;
            }
            cout << "OCR input: 128x48" << endl;
        }
        else
        {
            if (PLATE_COLOR_CHARS.size() != 78)
            {
                cerr << "Invalid PLATE_COLOR_CHARS size" << endl;
                return false;
            }
            ocr_chars_ = PLATE_COLOR_CHARS;

            bool has_char_output = false;
            bool has_color_output = false;
            const vector<rknn_tensor_attr> &attrs = recognizer_.outputAttrs();
            for (const rknn_tensor_attr &attr : attrs)
            {
                if (attr.n_elems == 21 * 78)
                    has_char_output = true;
                else if (attr.n_elems == 5)
                    has_color_output = true;
            }
            if (!has_char_output)
            {
                cerr << "Unsupported plate_color OCR output: expected one output with n_elems="
                     << (21 * 78) << endl;
                return false;
            }

            cout << "OCR input: 168x48" << endl;
            cout << "OCR input expected: 168x48" << endl;
            cout << "OCR char output: 21x78" << endl;
            if (has_color_output)
            {
                cout << "OCR color output: enabled" << endl;
            }
        }
        cout << "OCR pipeline: python-style bbox crop, BGR uint8 NHWC, backend=" << ocrBackendName(ocr_backend_)
             << ", resize=" << (ocr_backend_ == OcrBackend::PLATE_COLOR ? "168x48" : "128x48") << " INTER_LINEAR"
             << ", expand_roi=" << (expand_ocr_roi_ ? "on" : "off")
             << ", tight_pad_x=" << ocr_pad_x_
             << ", tight_pad_y=" << ocr_pad_y_
             << ", vote_window=" << ocr_vote_window_
             << ", min_votes=" << ocr_min_votes_;
        if (debug_ocr_)
        {
            mkdir(debug_ocr_dir_.c_str(), 0777);
            cout << ", debug_dir=" << debug_ocr_dir_;
        }
        cout << endl;
        if (!fpga_.init(fpga_mode, fpga_dir, fpga_timeout_ms,
                        fpga_video_dev, fpga_width, fpga_height, fpga_fps, fpga_fourcc,
                        drm_card, drm_connector_id, drm_width, drm_height, fpga_settle_ms))
            return false;

#ifdef HAVE_OPENCV_FREETYPE
        if (!font_path.empty() && fileExists(font_path))
        {
            try
            {
                ft2_ = cv::freetype::createFreeType2();
                ft2_->loadFontData(font_path, 0);
                cout << "OpenCV FreeType enabled: " << font_path << endl;
            }
            catch (const cv::Exception &e)
            {
                ft2_.release();
                cerr << "FreeType font load failed, fallback to cv::putText: " << font_path << endl;
            }
        }
        else
        {
            cerr << "Font file not found, fallback to cv::putText: " << font_path << endl;
        }
#else
        cerr << "HAVE_OPENCV_FREETYPE not defined. Trying direct FreeType renderer." << endl;
#endif
#ifdef HAVE_DIRECT_FREETYPE
        if (!font_path.empty() && fileExists(font_path))
        {
            if (!text_renderer_.init(font_path))
            {
                cerr << "Direct FreeType init failed, fallback to cv::putText: " << font_path << endl;
            }
        }
        else
        {
            cerr << "Chinese font file is not readable, fallback to cv::putText: " << font_path << endl;
        }
#else
        (void)font_path;
        cerr << "HAVE_DIRECT_FREETYPE not defined. Chinese plate text may not render correctly." << endl;
#endif

        return true;
    }

    bool isYoloOnly() const { return yolo_only_; }

    YoloProfile lastYoloProfile() const { return last_yolo_profile_; }

    bool processFrame(const Mat &frame, Mat &annotated, vector<string> &plates)
    {
        vector<CachedPlate> results;
        if (!inferFrame(frame, results, plates))
        {
            if (!frame.empty())
            {
                annotated = frame.clone();
                drawSmallStatus(annotated, "RKNN detect failed");
            }
            return false;
        }

        annotated = frame.clone();
        drawCachedResults(annotated, results, true);
        return true;
    }

    bool inferFrame(const Mat &frame, vector<CachedPlate> &results, vector<string> &plates)
    {
        results.clear();
        plates.clear();
        if (frame.empty())
            return false;

        vector<Detection> dets;
        if (!detect(frame, dets))
        {
            return false;
        }

        // Raw-geometry dump for jitter measurement (no filtering applied here).
        logObbJitter(dets);

        // OBB anti-jitter: OneEuro adaptive smoothing (see GeomSmoothTrack).
        // Near-zero lag on real motion, strong suppression of the INT8
        // quantization flip-flop at rest.
        smoothDetections(dets);

        if (yolo_only_)
        {
            for (const Detection &det : dets)
            {
                CachedPlate item;
                item.box = det.box;
                item.roi_box = det.box;
                item.ocr_box = det.box;
                item.score = det.score;
                item.raw_plate = "";
                item.plate = "";
                item.color = "";
                item.color_score = 0.0f;
                item.track_id = -1;
                item.fpga_ok = true;
                item.obb = det.obb;
                item.angle = det.angle;
                item.rot_w = det.rot_w;
                item.rot_h = det.rot_h;
                item.has_quad = det.has_quad;
                memcpy(item.quad, det.quad, sizeof(item.quad));
                results.push_back(item);
            }
            return true;
        }

        pruneOcrTracks();

        uint64_t frame_seq = ++ocr_frame_seq_;
        Rect frame_rect(0, 0, frame.cols, frame.rows);
        for (size_t det_i = 0; det_i < dets.size(); ++det_i)
        {
            const Detection &det = dets[det_i];
            Rect roi_box = det.box & frame_rect;
            Rect ocr_box = roi_box;
            if (roi_box.area() <= 0)
                continue;

            OcrResult ocr = det.obb
                                ? recognizePythonStyleObb(frame, det, det_i, frame_seq)
                                : recognizePythonStyle(frame, det.box, det_i, frame_seq, expand_ocr_roi_);
            if (ocr.box.area() > 0)
            {
                ocr_box = ocr.box;
                roi_box = ocr.box;
            }

            CachedPlate item;
            item.box = det.box;
            item.roi_box = roi_box;
            item.ocr_box = ocr_box;
            item.score = det.score;
            item.ocr_score = ocr.quality;
            item.raw_plate = ocr.text;
            item.color = ocr.color;
            item.color_score = ocr.color_score;
            item.fpga_ok = true;
            item.obb = det.obb;
            item.angle = det.angle;
            item.rot_w = det.rot_w;
            item.rot_h = det.rot_h;
            item.has_quad = det.has_quad;
            memcpy(item.quad, det.quad, sizeof(item.quad));

            // OCR as verifier: only stable, legally-formatted plates are
            // promoted to the plate list; boxes are always shown regardless.
            if (isUsableOcr(ocr))
            {
                int track_id = -1;
                string stable_color;
                string stable_plate = updateOcrTrack(det.box, ocr, track_id, stable_color);
                item.track_id = track_id;
                if (!stable_plate.empty())
                {
                    item.plate = stable_plate;
                    plates.push_back(stable_plate);
                }
                if (!stable_color.empty())
                {
                    item.color = stable_color;   // voted color beats per-frame color
                }
            }
            results.push_back(item);
        }

        return true;
    }

    void drawCachedResults(Mat &img, const vector<CachedPlate> &results, bool draw_text)
    {
        if (img.empty())
            return;

        if (results.empty())
        {
            drawSmallStatus(img, "No plate");
            return;
        }

        for (const CachedPlate &r : results)
        {
            const Scalar display_color = plateDisplayColor(r.color);
            if (r.has_quad)
            {
                // Corner-regression quad (identity order [TL,TR,BR,BL])
                Point pts[4];
                for (int k = 0; k < 4; ++k)
                    pts[k] = Point(static_cast<int>(roundf(r.quad[k][0])),
                                   static_cast<int>(roundf(r.quad[k][1])));
                const Point *pp[1] = {pts};
                int n[1] = {4};
                polylines(img, pp, n, 1, true, display_color, 2, LINE_AA);
            }
            else if (r.obb)
            {
                drawRotatedBox(img,
                               r.box.x + r.box.width * 0.5f,
                               r.box.y + r.box.height * 0.5f,
                               r.rot_w, r.rot_h, r.angle,
                               display_color, 2);
            }
            else
            {
                rectangle(img, r.box, display_color, 2, LINE_AA);
            }
            if (draw_text)
            {
                string label;
                if (!r.plate.empty())
                    label = r.plate;                    // confirmed stable plate
                else if (!r.raw_plate.empty())
                    label = r.raw_plate;                // OCR returned something unconfirmed
                else
                    label = "plate (" + to_string(static_cast<int>(r.score * 100)) + "%)";  // YOLO only
                if (!r.fpga_ok)
                    label += " [raw]";
                if (!r.color.empty())
                    label += " " + r.color;
                drawPlateLabel(img, label, r.box, display_color);
            }
        }
    }

private:
    RknnModel detector_;
    RknnModel recognizer_;
    FpgaRoiProcessor fpga_;
    float conf_thres_ = 0.5f;
    float nms_thres_ = 0.45f;
    string fpga_mode_ = "sw-edge";
    bool yolo_only_ = false;
    bool profile_yolo_ = false;
    OcrBackend ocr_backend_ = OcrBackend::PLATE_COLOR;
    YoloProfile last_yolo_profile_;
    vector<vector<float>> detector_outputs_;
    vector<vector<float>> recognizer_outputs_;
    float ocr_pad_x_ = 0.04f;
    float ocr_pad_y_ = 0.10f;
    int ocr_vote_window_ = 5;
    int ocr_min_votes_ = 2;
    float ocr_track_iou_ = 0.5f;
    int ocr_track_ttl_ms_ = 800;
    string debug_ocr_dir_;
    bool debug_ocr_ = false;
    bool expand_ocr_roi_ = false;
    uint64_t debug_ocr_seq_ = 0;
    uint64_t ocr_frame_seq_ = 0;
    int next_ocr_track_id_ = 1;
    vector<OcrTrack> ocr_tracks_;

    // ---- OBB anti-jitter: OneEuro adaptive low-pass ----
    // Board measurement (2026-08-18, 3436 frames): angle is INT8-quantized to
    // 0.561°/step and flip-flops between adjacent steps; center/size jitter
    // ≤3 px (p90) on a hand-held "static" plate; real motion 3-486 px/frame.
    // OneEuro passes fast motion through (lag ≈1 frame) while crushing the
    // quantization noise at rest — a fixed EMA would lag ~100 ms always.
    struct OneEuro
    {
        float min_cutoff = 1.2f;   // Hz
        float beta = 0.03f;        // speed coefficient (px/s)
        float d_cutoff = 1.0f;     // Hz, derivative low-pass
        float x_prev = 0.0f, dx_prev = 0.0f, t_prev = 0.0f;
        bool have = false;

        static float alphaFor(float cutoff, float dt)
        {
            float tau = 1.0f / (2.0f * 3.14159265f * max(cutoff, 0.01f));
            return 1.0f / (1.0f + tau / max(dt, 1e-4f));
        }

        float filter(float x, float t)
        {
            if (!have)
            {
                x_prev = x; dx_prev = 0.0f; t_prev = t; have = true;
                return x;
            }
            float dt = t - t_prev;
            if (dt <= 0.0f)
                return x_prev;
            float dx = (x - x_prev) / dt;
            dx_prev = alphaFor(d_cutoff, dt) * dx +
                      (1.0f - alphaFor(d_cutoff, dt)) * dx_prev;
            float cutoff = min_cutoff + beta * fabsf(dx_prev);
            x_prev += alphaFor(cutoff, dt) * (x - x_prev);
            t_prev = t;
            return x_prev;
        }

        void reset() { have = false; }
    };

    struct GeomSmoothTrack
    {
        Rect env;
        int age = 0;
        OneEuro cx, cy, ang_deg, w, h;
        // Quad corners (identity order [TL,TR,BR,BL]): 8 independent OneEuro
        // channels. Identity is pinned by assignQuadIdentity before smoothing,
        // so channel k always holds the same physical corner across frames.
        bool has_quad = false;
        OneEuro qx[4], qy[4];
    };
    vector<GeomSmoothTrack> geom_tracks_;
    uint64_t geom_frame_seq_ = 0;

    // --obb-jitter-log instrumentation: raw per-frame OBB geometry dump
    // (CSV: frame,track,cx,cy,ang,w,h,conf). Matching by envelope IoU,
    // independent of OcrTrack; no filtering applied to the logged values.
    string obb_jitter_log_;
    ofstream obb_jitter_file_;
    uint64_t jitter_frame_seq_ = 0;
    struct JitterTrack
    {
        Rect env;
        int age = 0;
    };
    vector<JitterTrack> jitter_tracks_;

    vector<string> ocr_chars_;
    chrono::steady_clock::time_point last_ocr_log_time_ = chrono::steady_clock::now();

#ifdef HAVE_OPENCV_FREETYPE
    Ptr<cv::freetype::FreeType2> ft2_;
#endif
#ifdef HAVE_DIRECT_FREETYPE
    Utf8FreeTypeRenderer text_renderer_;
#endif

    static bool isValidPlateGeometry(const Rect &box,
                                     int frame_width,
                                     int frame_height,
                                     float confidence)
    {
        if (frame_width <= 0 || frame_height <= 0 ||
            box.width <= 0 || box.height <= 0)
            return false;

        const int min_width = max(32, frame_width / 50);
        const int min_height = max(10, frame_height / 72);
        if (box.width < min_width || box.height < min_height)
            return false;

        const float aspect = static_cast<float>(box.width) /
                             static_cast<float>(box.height);
        if (aspect < 1.60f || aspect > 6.80f)
            return false;

        const float frame_area = static_cast<float>(frame_width) *
                                 static_cast<float>(frame_height);
        const float area_ratio = static_cast<float>(box.area()) / frame_area;
        if (area_ratio < 0.00020f || area_ratio > 0.12f)
            return false;

        // Marginal geometry and very small boxes need stronger detector evidence.
        if ((aspect < 1.90f || aspect > 5.80f) && confidence < 0.45f)
            return false;
        if ((box.width < frame_width * 0.04f ||
             box.height < frame_height * 0.02f) &&
            confidence < 0.40f)
            return false;

        return true;
    }

    void logObbJitter(const vector<Detection> &dets)
    {
        if (!obb_jitter_file_.is_open())
            return;
        uint64_t f = ++jitter_frame_seq_;

        for (auto &jt : jitter_tracks_)
            jt.age++;

        for (const auto &d : dets)
        {
            if (!d.obb)
                continue;
            int best = -1;
            float best_iou = 0.0f;
            for (size_t i = 0; i < jitter_tracks_.size(); ++i)
            {
                const JitterTrack &jt = jitter_tracks_[i];
                if (jt.age == 0)
                    continue;   // already claimed this frame
                float iou = iouValue(d.box, jt.env);
                if (iou > best_iou)
                {
                    best_iou = iou;
                    best = static_cast<int>(i);
                }
            }

            if (best < 0 || best_iou < 0.5f)
            {
                JitterTrack jt;
                jt.env = d.box;
                jt.age = 0;
                jitter_tracks_.push_back(jt);
                best = static_cast<int>(jitter_tracks_.size()) - 1;
            }
            else
            {
                jitter_tracks_[best].env = d.box;
                jitter_tracks_[best].age = 0;
            }

            float cx = d.box.x + d.box.width * 0.5f;
            float cy = d.box.y + d.box.height * 0.5f;
            obb_jitter_file_ << f << ',' << (best + 1) << ','
                             << fixed << setprecision(2)
                             << cx << ',' << cy << ','
                             << setprecision(4) << d.angle << ','
                             << setprecision(2) << d.rot_w << ',' << d.rot_h << ','
                             << setprecision(3) << d.score << '\n';
        }

        jitter_tracks_.erase(
            remove_if(jitter_tracks_.begin(), jitter_tracks_.end(),
                      [](const JitterTrack &jt) { return jt.age > 30; }),
            jitter_tracks_.end());
    }

    // Axis-aligned envelope of a rotated box (center, long-edge angle, sides).
    static Rect obbEnvelope(float cx, float cy, float angle, float w, float h)
    {
        float ca = fabsf(cosf(angle)), sa = fabsf(sinf(angle));
        float eaw = w * ca + h * sa;
        float eah = w * sa + h * ca;
        return Rect(static_cast<int>(roundf(cx - eaw * 0.5f)),
                    static_cast<int>(roundf(cy - eah * 0.5f)),
                    static_cast<int>(roundf(eaw)),
                    static_cast<int>(roundf(eah)));
    }

    // Wrap an angle delta into (−90°, +90°] (long edge is unoriented).
    static float wrapAngleHalf(float d)
    {
        while (d > 90.0f)  d -= 180.0f;
        while (d <= -90.0f) d += 180.0f;
        return d;
    }

    // OneEuro-smooth OBB geometry per track (matched by envelope IoU).
    // Writes smoothed geometry back into dets, shared by the draw path and
    // the rot-crop, so the displayed quad and the LPRNet input stay stable.
    void smoothDetections(vector<Detection> &dets)
    {
        const float kMatchIou = 0.3f;   // generous: real motion must be able to match
        const int kMaxAge = 60;         // prune tracks unseen for ~2.4s
        const float kFps = 25.0f;
        float t = static_cast<float>(++geom_frame_seq_) / kFps;

        for (auto &gt : geom_tracks_)
            gt.age++;

        for (auto &d : dets)
        {
            if (!d.obb)
                continue;
            float cx = d.box.x + d.box.width * 0.5f;
            float cy = d.box.y + d.box.height * 0.5f;
            float ang_deg = static_cast<float>(d.angle * 180.0 / CV_PI);

            int best = -1;
            float best_iou = 0.0f;
            for (size_t i = 0; i < geom_tracks_.size(); ++i)
            {
                const GeomSmoothTrack &gt = geom_tracks_[i];
                if (gt.age == 0)
                    continue;   // already claimed this frame
                float iou = iouValue(d.box, gt.env);
                if (iou > best_iou)
                {
                    best_iou = iou;
                    best = static_cast<int>(i);
                }
            }

            if (best < 0 || best_iou < kMatchIou)
            {
                GeomSmoothTrack gt;
                gt.env = d.box;
                gt.age = 0;
                gt.cx.filter(cx, t);
                gt.cy.filter(cy, t);
                gt.ang_deg.filter(ang_deg, t);
                gt.w.filter(d.rot_w, t);
                gt.h.filter(d.rot_h, t);
                gt.has_quad = d.has_quad;
                if (d.has_quad)
                    for (int k = 0; k < 4; ++k)
                    {
                        gt.qx[k].filter(d.quad[k][0], t);
                        gt.qy[k].filter(d.quad[k][1], t);
                    }
                geom_tracks_.push_back(gt);
                continue;   // new track: first value passes through
            }

            GeomSmoothTrack &gt = geom_tracks_[best];
            gt.age = 0;
            gt.env = d.box;
            float scx = gt.cx.filter(cx, t);
            float scy = gt.cy.filter(cy, t);
            // Unwrap the raw angle against the previous filtered angle so
            // θ and θ±180° (same long-edge direction) smooth continuously.
            float sang = gt.ang_deg.filter(
                gt.ang_deg.x_prev + wrapAngleHalf(ang_deg - gt.ang_deg.x_prev), t);
            float sw = gt.w.filter(d.rot_w, t);
            float sh = gt.h.filter(d.rot_h, t);

            if (d.has_quad)
            {
                if (!gt.has_quad)
                {
                    for (int k = 0; k < 4; ++k)
                    {
                        gt.qx[k].reset();
                        gt.qy[k].reset();
                    }
                    gt.has_quad = true;
                }
                for (int k = 0; k < 4; ++k)
                {
                    d.quad[k][0] = gt.qx[k].filter(d.quad[k][0], t);
                    d.quad[k][1] = gt.qy[k].filter(d.quad[k][1], t);
                }
            }

            d.angle = static_cast<float>(sang * CV_PI / 180.0);
            d.rot_w = sw;
            d.rot_h = sh;
            d.box = obbEnvelope(scx, scy, d.angle, sw, sh);
        }

        geom_tracks_.erase(
            remove_if(geom_tracks_.begin(), geom_tracks_.end(),
                      [](const GeomSmoothTrack &gt) { return gt.age > kMaxAge; }),
            geom_tracks_.end());
    }

    bool detect(const Mat &frame, vector<Detection> &dets)
    {
        RknnProfile rknn_profile;
        YoloProfile *profile = &last_yolo_profile_;
        profile->reset();

        auto yolo_t0 = chrono::steady_clock::now();

        // Detect model type: 7 outputs = YOLOv8-OBB + corner branch; 4 outputs =
        // YOLOv8-OBB raw heads + angle; 3 outputs = raw YOLOv5 feature maps;
        // 1 output = decoded boxes
        bool is_obb_corner = (detector_.outputAttrs().size() == 7);
        bool is_obb = is_obb_corner || (detector_.outputAttrs().size() == 4);
        bool is_raw_yolov5 = (detector_.outputAttrs().size() == 3);

        Mat input_frame;
        LetterboxParams lb_params;

        if (is_raw_yolov5 || is_obb)
        {
            // YOLOv5n / YOLOv8-OBB path: letterbox preprocessing → RGB uint8 NHWC
            int det_w = detector_.width();
            int det_h = detector_.height();
            lb_params = computeLetterbox(frame.cols, frame.rows,
                                         max(det_w, det_h));
            input_frame = applyLetterbox(frame, max(det_w, det_h), lb_params);

            // Send letterboxed RGB image directly as NHWC uint8 (model handles /255)
            // Bypass runImage's resize + color convert since we already did it
            // Use the internal RKNN API path for uint8 NHWC input
            rknn_input input;
            memset(&input, 0, sizeof(input));
            input.index = 0;
            input.type = RKNN_TENSOR_UINT8;
            input.fmt = RKNN_TENSOR_NHWC;
            input.size = static_cast<uint32_t>(input_frame.total() * input_frame.elemSize());
            input.buf = input_frame.data;

            int ret = rknn_inputs_set(detector_.ctx(), 1, &input);
            if (ret < 0)
            {
                cerr << "[YOLOv5] rknn_inputs_set failed, ret=" << ret << endl;
                profile->total_ms = elapsedMs(yolo_t0, chrono::steady_clock::now());
                return false;
            }

            ret = rknn_run(detector_.ctx(), nullptr);
            if (ret < 0)
            {
                cerr << "[YOLOv5] rknn_run failed, ret=" << ret << endl;
                profile->total_ms = elapsedMs(yolo_t0, chrono::steady_clock::now());
                return false;
            }

            // Get outputs
            size_t n_out = detector_.outputAttrs().size();
            vector<rknn_output> rknn_outs(n_out);
            memset(rknn_outs.data(), 0, n_out * sizeof(rknn_output));
            for (size_t i = 0; i < n_out; ++i)
            {
                rknn_outs[i].index = static_cast<uint32_t>(i);
                rknn_outs[i].want_float = 1;
            }
            ret = rknn_outputs_get(detector_.ctx(), static_cast<uint32_t>(n_out),
                                   rknn_outs.data(), nullptr);
            if (ret < 0)
            {
                cerr << "[YOLOv5] rknn_outputs_get failed, ret=" << ret << endl;
                profile->total_ms = elapsedMs(yolo_t0, chrono::steady_clock::now());
                return false;
            }

            detector_outputs_.resize(n_out);
            for (size_t i = 0; i < n_out; ++i)
            {
                int elems = detector_.outputAttrs()[i].n_elems;
                detector_outputs_[i].resize(elems);
                memcpy(detector_outputs_[i].data(), rknn_outs[i].buf,
                       elems * sizeof(float));
            }
            rknn_outputs_release(detector_.ctx(), static_cast<uint32_t>(n_out),
                                 rknn_outs.data());

            auto post_t0 = chrono::steady_clock::now();

            if (is_obb)
            {
                // YOLOv8-OBB decode: DFL+angle heads → rotated candidates
                vector<ObbCandidate> cands = decodeYolov8Obb(
                    detector_outputs_, detector_.outputAttrs(), conf_thres_);

                profile->rows = cands.size();
                for (const auto &c : cands)
                    profile->max_confidence = max(profile->max_confidence, c.score);

                // Map from letterbox space → original image coords (angle unchanged)
                vector<ObbCandidate> filtered;
                filtered.reserve(cands.size());
                for (const ObbCandidate &c : cands)
                {
                    ObbCandidate f = c;
                    f.cx = (c.cx - lb_params.left) / lb_params.scale;
                    f.cy = (c.cy - lb_params.top) / lb_params.scale;
                    f.w = c.w / lb_params.scale;
                    f.h = c.h / lb_params.scale;
                    float ca = cosf(f.ang), sa = sinf(f.ang);
                    float eaw = f.w * fabsf(ca) + f.h * fabsf(sa);
                    float eah = f.w * fabsf(sa) + f.h * fabsf(ca);
                    f.env = Rect(static_cast<int>(roundf(f.cx - eaw * 0.5f)),
                                 static_cast<int>(roundf(f.cy - eah * 0.5f)),
                                 static_cast<int>(roundf(eaw)),
                                 static_cast<int>(roundf(eah)));
                    if (!isValidObbGeometry(f.w, f.h, f.env, frame.cols, frame.rows, f.score))
                        continue;
                    filtered.push_back(f);
                }

                vector<ObbCandidate> keep = rotatedNms(filtered, nms_thres_);

                dets.clear();
                for (const ObbCandidate &k : keep)
                {
                    Detection d;
                    d.box = k.env;
                    d.score = k.score;
                    d.obb = true;
                    d.angle = k.ang;
                    d.rot_w = k.w;
                    d.rot_h = k.h;
                    if (is_obb_corner && k.aidx >= 0)
                    {
                        // Corner branch: decode 4 corners at the winning anchor,
                        // canonicalize, then pin to fixed [TL,TR,BR,BL] identity
                        // via the raw (pre-smoothing) box reference corners.
                        float pts[4][2] = {};
                        decodeQuadCorners(detector_outputs_, detector_.outputAttrs(),
                                          k.aidx, pts);
                        for (auto &pt : pts)
                        {
                            pt[0] = (pt[0] - lb_params.left) / lb_params.scale;
                            pt[1] = (pt[1] - lb_params.top) / lb_params.scale;
                        }
                        canonicalizeQuad(pts);
                        d.has_quad = assignQuadIdentity(pts, k.cx, k.cy, k.ang,
                                                        k.w, k.h, d.quad);
                    }
                    dets.push_back(d);
                }

                auto t1 = chrono::steady_clock::now();
                profile->postprocess_ms = elapsedMs(post_t0, t1);
                profile->total_ms = elapsedMs(yolo_t0, t1);
                profile->detections = dets.size();
                return true;
            }

            // YOLOv5 decode: raw feature maps → boxes in original image coords
            vector<pair<Rect, float>> decoded = decodeYolov5Outputs(
                detector_outputs_, detector_.outputAttrs(),
                frame.cols, frame.rows, conf_thres_, lb_params);

            profile->rows = decoded.size();
            for (const auto &d : decoded)
                profile->max_confidence = max(profile->max_confidence, d.second);

            // Separate boxes/scores for NMS
            vector<Rect> boxes;
            vector<float> scores;
            for (const auto &d : decoded)
            {
                boxes.push_back(d.first);
                scores.push_back(d.second);
            }

            // Apply geometry filter
            vector<Rect> filtered_boxes;
            vector<float> filtered_scores;
            for (size_t i = 0; i < boxes.size(); ++i)
            {
                if (isValidPlateGeometry(boxes[i], frame.cols, frame.rows, scores[i]))
                {
                    filtered_boxes.push_back(boxes[i]);
                    filtered_scores.push_back(scores[i]);
                }
            }

            vector<int> keep = nmsIndices(filtered_boxes, filtered_scores,
                                          conf_thres_, nms_thres_);

            dets.clear();
            for (int idx : keep)
            {
                Detection d;
                d.box = filtered_boxes[idx];
                d.score = filtered_scores[idx];
                dets.push_back(d);
            }

            auto t1 = chrono::steady_clock::now();
            profile->postprocess_ms = elapsedMs(post_t0, t1);
            profile->total_ms = elapsedMs(yolo_t0, t1);
            profile->detections = dets.size();
            return true;
        }

        // --- Legacy path: single-output decoded-boxes model (plate_detect_416 etc.) ---
        if (!detector_.runImage(frame, true, detector_outputs_, false, &rknn_profile) || detector_outputs_.empty())
        {
            profile->rknn = rknn_profile;
            profile->total_ms = elapsedMs(yolo_t0, chrono::steady_clock::now());
            return false;
        }

        auto post_t0 = chrono::steady_clock::now();
        if (detector_outputs_.size() != 1)
        {
            cerr << "Detector has " << detector_outputs_.size()
                 << " outputs. Expected 1 (decoded boxes), 3 (raw YOLOv5), "
                    "4/7 (YOLOv8-OBB, 7 = with corner branch)." << endl;
            auto t1 = chrono::steady_clock::now();
            profile->rknn = rknn_profile;
            profile->postprocess_ms = elapsedMs(post_t0, t1);
            profile->total_ms = elapsedMs(yolo_t0, t1);
            return false;
        }

        const rknn_tensor_attr &attr = detector_.outputAttrs()[0];
        const vector<float> &pred = detector_outputs_[0];

        int rows = 0;
        int cols = 0;
        bool transposed = false;

        if (!inferDetectMatrixShape(attr, rows, cols, transposed))
        {
            cerr << "Unsupported detector output shape." << endl;
            auto t1 = chrono::steady_clock::now();
            profile->rknn = rknn_profile;
            profile->postprocess_ms = elapsedMs(post_t0, t1);
            profile->total_ms = elapsedMs(yolo_t0, t1);
            return false;
        }
        profile->rows = static_cast<size_t>(rows);

        if (cols < 5)
        {
            auto t1 = chrono::steady_clock::now();
            profile->rknn = rknn_profile;
            profile->postprocess_ms = elapsedMs(post_t0, t1);
            profile->total_ms = elapsedMs(yolo_t0, t1);
            return false;
        }

        vector<Rect> boxes;
        vector<float> scores;

        float x_factor = frame.cols / static_cast<float>(detector_.width());
        float y_factor = frame.rows / static_cast<float>(detector_.height());

        for (int i = 0; i < rows; ++i)
        {
            float conf = detectValue(pred, rows, cols, transposed, i, 4);
            if (!std::isfinite(conf))
            {
                profile->invalid_values++;
                continue;
            }
            profile->max_confidence = max(profile->max_confidence, conf);
            if (conf < conf_thres_)
                continue;
            profile->candidates_above_threshold++;

            float cx = detectValue(pred, rows, cols, transposed, i, 0);
            float cy = detectValue(pred, rows, cols, transposed, i, 1);
            float w = detectValue(pred, rows, cols, transposed, i, 2);
            float h = detectValue(pred, rows, cols, transposed, i, 3);
            if (!std::isfinite(cx) || !std::isfinite(cy) ||
                !std::isfinite(w) || !std::isfinite(h))
            {
                profile->invalid_values++;
                continue;
            }

            int left = static_cast<int>((cx - 0.5f * w) * x_factor);
            int top = static_cast<int>((cy - 0.5f * h) * y_factor);
            int width = static_cast<int>(w * x_factor);
            int height = static_cast<int>(h * y_factor);

            Rect box(left, top, width, height);
            box &= Rect(0, 0, frame.cols, frame.rows);

            if (box.area() <= 0)
                continue;

            if (!isValidPlateGeometry(box, frame.cols, frame.rows, conf))
                continue;

            boxes.push_back(box);
            scores.push_back(conf);
        }

        vector<int> keep = nmsIndices(boxes, scores, conf_thres_, nms_thres_);

        dets.clear();
        for (int idx : keep)
        {
            Detection d;
            d.box = boxes[idx];
            d.score = scores[idx];
            dets.push_back(d);
        }

        auto t1 = chrono::steady_clock::now();
        profile->rknn = rknn_profile;
        profile->postprocess_ms = elapsedMs(post_t0, t1);
        profile->total_ms = elapsedMs(yolo_t0, t1);
        profile->detections = dets.size();

        return true;
    }

    static bool inferDetectMatrixShape(const rknn_tensor_attr &attr,
                                       int &rows,
                                       int &cols,
                                       bool &transposed)
    {
        vector<int> dims;
        for (int i = 0; i < attr.n_dims; ++i)
        {
            if (attr.dims[i] > 1)
            {
                dims.push_back(attr.dims[i]);
            }
        }

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

    static float detectValue(const vector<float> &data,
                             int rows,
                             int cols,
                             bool transposed,
                             int r,
                             int c)
    {
        if (!transposed)
        {
            size_t idx = static_cast<size_t>(r) * cols + c;
            if (idx >= data.size())
                return 0.0f;
            return data[idx];
        }

        size_t idx = static_cast<size_t>(c) * rows + r;
        if (idx >= data.size())
            return 0.0f;
        return data[idx];
    }

    Rect makeOcrBox(const Rect &det_box, int img_w, int img_h) const
    {
        return expandBox(det_box, img_w, img_h, ocr_pad_x_, ocr_pad_y_);
    }

    static string tensorTypeName(int type)
    {
        if (type == RKNN_TENSOR_UINT8)
            return "uint8";
        if (type == RKNN_TENSOR_FLOAT32)
            return "float32";
        ostringstream oss;
        oss << "type_" << type;
        return oss.str();
    }

    bool shouldSaveOcrDebug(uint64_t frame_seq) const
    {
        return debug_ocr_ && frame_seq > 0 && (frame_seq % 100) == 0;
    }

    OcrResult recognizePythonStyle(const Mat &frame,
                                   const Rect &det_box,
                                   size_t det_index,
                                   uint64_t frame_seq,
                                   bool expand_roi = false)
    {
        OcrResult result;
        if (frame.empty())
            return result;

        Rect frame_rect(0, 0, frame.cols, frame.rows);
        Rect crop_box = (expand_roi ? makeOcrBox(det_box, frame.cols, frame.rows) : det_box) & frame_rect;
        result.box = crop_box;
        result.tag = expand_roi ? "python_expand" : "python_bbox";
        if (crop_box.area() <= 0)
            return result;

        Mat crop = frame(crop_box);
        if (crop.total() * static_cast<size_t>(crop.channels()) <= 100)
            return result;

        return runRecognizerTail(crop, crop_box,
                                 expand_roi ? "python_expand" : "python_bbox",
                                 det_index, frame_seq);
    }

    // OBB variant: rotate-correct the plate crop before LPRNet (aligned plate
    // is the trained input distribution; tilted raw crops read worse).
    // Corner-model detections use the 4-corner quad for a true perspective
    // correction instead of the box-only rotation.
    OcrResult recognizePythonStyleObb(const Mat &frame,
                                      const Detection &det,
                                      size_t det_index,
                                      uint64_t frame_seq)
    {
        OcrResult result;
        if (frame.empty())
            return result;

        float cx = det.box.x + det.box.width * 0.5f;
        float cy = det.box.y + det.box.height * 0.5f;
        Mat crop = det.has_quad
                       ? quadCropPlate(frame, det.quad, cx, cy,
                                       det.rot_w, det.rot_h, 0.08f, 0.18f)
                       : rotateCropPlate(frame, det.box, det.angle,
                                         det.rot_w, det.rot_h, 0.08f, 0.18f);
        if (crop.empty())
            return result;

        return runRecognizerTail(crop, det.box,
                                 det.has_quad ? "obb_quad" : "obb_rotate",
                                 det_index, frame_seq);
    }

    // Shared tail: run recognizer on a prepared crop and decode. crop_box is the
    // frame-coords anchor reported back in result.box (envelope for OBB crops).
    OcrResult runRecognizerTail(const Mat &crop, const Rect &crop_box, const string &tag,
                                size_t det_index, uint64_t frame_seq)
    {
        OcrResult result;
        result.box = crop_box;
        result.tag = tag;
        if (crop.empty() || crop.total() * static_cast<size_t>(crop.channels()) <= 100)
            return result;

        bool save_debug = shouldSaveOcrDebug(frame_seq);
        chrono::steady_clock::time_point t0;
        if (save_debug)
        {
            t0 = chrono::steady_clock::now();
        }

        Mat input = makeOcrInput(crop);
        if (input.empty())
            return result;

        if (!recognizer_.runImage(input, false, recognizer_outputs_) || recognizer_outputs_.empty())
        {
            return result;
        }

        if (ocr_backend_ == OcrBackend::PLATE_COLOR)
        {
            result = decodePlateColor(recognizer_outputs_);
        }
        else
        {
            // LPRNet backend: auto-detect V1 (1 output) vs V2 (2 outputs)
            const rknn_tensor_attr &attr = recognizer_.outputAttrs()[0];
            result = decodeLpr(recognizer_outputs_[0], attr);

            // V2: decode color from second output (multi-label sigmoid)
            const auto &out_attrs = recognizer_.outputAttrs();
            if (out_attrs.size() >= 2 && recognizer_outputs_.size() >= 2)
            {
                int color_idx = findOutputBySize(recognizer_outputs_, 5);
                if (color_idx >= 0)
                {
                    result.color = decodeColorV2(recognizer_outputs_[color_idx], result.color_score);
                    result.debug.color = result.color;
                    result.debug.color_score = result.color_score;
                }
            }
        }
        result.box = crop_box;
        result.tag = tag;
        result.input = input;
        result.debug.backend = ocrBackendName(ocr_backend_);

        if (save_debug)
        {
            result.debug.elapsed_ms = static_cast<double>(
                                          chrono::duration_cast<chrono::microseconds>(chrono::steady_clock::now() - t0).count()) /
                                      1000.0;
            if (!recognizer_.inputAttrs().empty())
            {
                result.debug.rknn_input_type = tensorTypeName(recognizer_.inputAttrs()[0].type);
            }
            int debug_output_index = 0;
            if (ocr_backend_ == OcrBackend::PLATE_COLOR)
            {
                debug_output_index = findOutputBySize(recognizer_outputs_, 21 * 78);
                if (debug_output_index < 0)
                    debug_output_index = 0;
            }
            if (result.debug.output_shape.empty() &&
                debug_output_index >= 0 &&
                debug_output_index < static_cast<int>(recognizer_.outputAttrs().size()))
            {
                const rknn_tensor_attr &attr = recognizer_.outputAttrs()[debug_output_index];
                for (int i = 0; i < attr.n_dims; ++i)
                {
                    result.debug.output_shape.push_back(attr.dims[i]);
                }
            }
            result.crop = crop.clone();
            if (result.debug.output_probs.empty() &&
                debug_output_index >= 0 &&
                debug_output_index < static_cast<int>(recognizer_outputs_.size()))
            {
                result.debug.output_probs = recognizer_outputs_[debug_output_index];
            }
            saveDebugOcr(result, det_index, frame_seq);
        }

        return result;
    }

    Mat makeOcrInput(const Mat &crop) const
    {
        if (ocr_backend_ == OcrBackend::PLATE_COLOR)
            return makePlateColorInput(crop);
        return makeLprnetInput(crop);
    }

    static Mat makeLprnetInput(const Mat &crop)
    {
        Mat input;
        if (crop.empty())
            return input;

        Mat bgr;
        if (crop.channels() == 1)
        {
            cvtColor(crop, bgr, COLOR_GRAY2BGR);
        }
        else if (crop.channels() == 4)
        {
            cvtColor(crop, bgr, COLOR_BGRA2BGR);
        }
        else
        {
            bgr = crop;
        }

        resize(bgr, input, Size(128, 48), 0, 0, INTER_LINEAR);
        if (input.type() != CV_8UC3)
        {
            input.convertTo(input, CV_8UC3);
        }
        if (!input.isContinuous())
        {
            input = input.clone();
        }
        return input;
    }

    static Mat makePlateColorInput(const Mat &roi)
    {
        Mat input;
        if (roi.empty())
            return input;

        Mat bgr;
        if (roi.channels() == 1)
        {
            cvtColor(roi, bgr, COLOR_GRAY2BGR);
        }
        else if (roi.channels() == 4)
        {
            cvtColor(roi, bgr, COLOR_BGRA2BGR);
        }
        else
        {
            bgr = roi;
        }

        resize(bgr, input, Size(168, 48), 0, 0, INTER_LINEAR);
        if (input.type() != CV_8UC3)
        {
            input.convertTo(input, CV_8UC3);
        }
        if (!input.isContinuous())
        {
            input = input.clone();
        }
        return input;
    }


    OcrResult recognizeBest(const Mat &frame, const Rect &det_box, const Rect &legacy_roi_box, size_t det_index)
    {
        OcrResult best;
        if (frame.empty())
            return best;

        vector<pair<string, Rect>> candidates;
        Rect frame_rect(0, 0, frame.cols, frame.rows);
        auto addCandidate = [&](const string &tag, Rect box)
        {
            box &= frame_rect;
            if (box.area() <= 0)
                return;
            if (box.width < 8 || box.height < 4)
                return;
            for (const auto &existing : candidates)
            {
                if (iouValue(existing.second, box) > 0.97f)
                    return;
            }
            candidates.push_back(make_pair(tag, box));
        };

        Rect primary = legacy_roi_box & frame_rect;
        float ocr_aspect = (ocr_backend_ == OcrBackend::PLATE_COLOR) ? (168.0f / 48.0f) : (94.0f / 24.0f);
        addCandidate("v3_roi", primary);
        addCandidate("tight", makeOcrBox(det_box, frame.cols, frame.rows));
        addCandidate("aspect", fitBoxToAspect(primary.area() > 0 ? primary : det_box,
                                              frame.cols, frame.rows, ocr_aspect));
        addCandidate("wide", expandBox(det_box, frame.cols, frame.rows, 0.16f, 0.12f));
        addCandidate("tall", expandBox(det_box, frame.cols, frame.rows, 0.06f, 0.24f));

        for (const auto &candidate : candidates)
        {
            Mat crop = frame(candidate.second).clone();
            OcrResult result = recognizeSingle(crop);
            result.crop = crop;
            result.input = makeOcrInput(crop);
            result.box = candidate.second;
            result.tag = candidate.first;
            result.quality += roiCandidateBonus(result);
            saveDebugOcr(result, det_index);

            if (isBetterOcrResult(result, best))
                best = result;
        }

        return best;
    }

    OcrResult recognizeSingle(const Mat &roi)
    {
        OcrResult result;
        if (roi.empty())
            return result;
        if (roi.total() * static_cast<size_t>(roi.channels()) <= 100)
            return result;

        Mat input = makeOcrInput(roi);
        if (input.empty())
            return result;

        if (!recognizer_.runImage(input, false, recognizer_outputs_) || recognizer_outputs_.empty())
        {
            return result;
        }

        if (ocr_backend_ == OcrBackend::PLATE_COLOR)
        {
            result = decodePlateColor(recognizer_outputs_);
        }
        else
        {
            // LPRNet backend: auto-detect V1 (1 output) vs V2 (2 outputs)
            const rknn_tensor_attr &attr = recognizer_.outputAttrs()[0];
            result = decodeLpr(recognizer_outputs_[0], attr);

            // V2: decode color from second output (multi-label sigmoid)
            const auto &out_attrs = recognizer_.outputAttrs();
            if (out_attrs.size() >= 2 && recognizer_outputs_.size() >= 2)
            {
                int color_idx = findOutputBySize(recognizer_outputs_, 5);
                if (color_idx >= 0)
                {
                    result.color = decodeColorV2(recognizer_outputs_[color_idx], result.color_score);
                    result.debug.color = result.color;
                    result.debug.color_score = result.color_score;
                }
            }
        }
        result.input = input;
        result.debug.backend = ocrBackendName(ocr_backend_);
        return result;
    }

    OcrResult decodeLpr(const vector<float> &output, const rknn_tensor_attr &attr) const
    {
        OcrResult result;
        vector<int> dims;
        for (int i = 0; i < attr.n_dims; ++i)
        {
            if (attr.dims[i] > 1)
                dims.push_back(attr.dims[i]);
        }
        if (dims.size() < 2)
            return result;

        int a = dims[dims.size() - 2];
        int b = dims[dims.size() - 1];
        int C = 0;
        int T = 0;
        bool channel_major = true;

        if ((a == 66 || a == 68 || (a > b && a <= 128)) && b <= 128)
        {
            C = a;
            T = b;
            channel_major = true;
        }
        else
        {
            C = b;
            T = a;
            channel_major = false;
        }

        const vector<string> &chars = ocr_chars_;
        int blank = C - 1;
        int prev = -1;
        int kept = 0;
        float conf_sum = 0.0f;
        float margin_sum = 0.0f;

        result.debug.class_count = C;
        result.debug.time_steps = T;
        result.debug.channel_major = channel_major;

        for (int t = 0; t < T; ++t)
        {
            float max_val = -1e30f;
            float second_val = -1e30f;
            int cls = 0;
            for (int c = 0; c < C; ++c)
            {
                size_t idx = channel_major
                                 ? static_cast<size_t>(c) * T + t
                                 : static_cast<size_t>(t) * C + c;
                if (idx >= output.size())
                    continue;

                float v = output[idx];
                if (v > max_val)
                {
                    second_val = max_val;
                    max_val = v;
                    cls = c;
                }
                else if (v > second_val)
                {
                    second_val = v;
                }
            }

            if (cls != blank && cls != prev && cls >= 0 && cls < static_cast<int>(chars.size()))
            {
                result.text += chars[cls];
                result.labels.push_back(cls);
                if (std::isfinite(max_val))
                    conf_sum += max_val;
                if (std::isfinite(max_val) && std::isfinite(second_val))
                {
                    margin_sum += max_val - second_val;
                }
                kept++;
            }
            prev = cls;
        }

        result.raw_text = result.text;
        if (kept > 0)
        {
            result.avg_confidence = conf_sum / kept;
            result.avg_margin = margin_sum / kept;
        }
        if (!std::isfinite(result.avg_confidence))
            result.avg_confidence = 0.0f;
        if (!std::isfinite(result.avg_margin))
            result.avg_margin = 0.0f;
        result.quality = result.text.empty() ? -1000.0f : result.avg_margin;
        return result;
    }

    static int findOutputBySize(const vector<vector<float>> &outputs, size_t elems)
    {
        for (size_t i = 0; i < outputs.size(); ++i)
        {
            if (outputs[i].size() == elems)
                return static_cast<int>(i);
        }
        return -1;
    }

    void copyOutputShape(OcrResult &result, int output_index) const
    {
        if (output_index < 0)
            return;
        const vector<rknn_tensor_attr> &attrs = recognizer_.outputAttrs();
        if (output_index >= static_cast<int>(attrs.size()))
            return;

        result.debug.output_shape.clear();
        const rknn_tensor_attr &attr = attrs[output_index];
        for (int i = 0; i < attr.n_dims; ++i)
        {
            result.debug.output_shape.push_back(attr.dims[i]);
        }
    }

    bool inferPlateColorChannelMajor(int output_index) const
    {
        const vector<rknn_tensor_attr> &attrs = recognizer_.outputAttrs();
        if (output_index < 0 || output_index >= static_cast<int>(attrs.size()))
            return false;

        vector<int> dims;
        const rknn_tensor_attr &attr = attrs[output_index];
        for (int i = 0; i < attr.n_dims; ++i)
        {
            if (attr.dims[i] > 1)
                dims.push_back(attr.dims[i]);
        }
        if (dims.size() < 2)
            return false;

        int a = dims[dims.size() - 2];
        int b = dims[dims.size() - 1];
        if (a == 78 && b == 21)
            return true;
        if (a == 21 && b == 78)
            return false;

        return false;
    }

    static bool isPlateColorProvinceLabel(int label)
    {
        return label >= 1 && label <= 31;
    }

    static bool isPlateColorDigitLabel(int label)
    {
        return label >= 42 && label <= 51;
    }

    static bool isPlateColorLetterLabel(int label)
    {
        return label >= 52 && label <= 75;
    }

    static bool isPlateColorSpecialLabel(int label)
    {
        return (label >= 32 && label <= 41) || label == 76 || label == 77;
    }

    float scorePlateColorResult(const OcrResult &result) const
    {
        if (result.text.empty() || result.labels.empty())
            return -1000.0f;

        int len = static_cast<int>(result.labels.size());
        float score = 0.0f;
        if (len == 7)
            score += 2.2f;
        else if (len == 8)
            score += 2.5f;
        else if (len == 6 || len == 9)
            score += 0.2f;
        else
            score -= 2.0f;

        if (isPlateColorProvinceLabel(result.labels[0]))
            score += 1.0f;
        else
            score -= 1.0f;

        if (len >= 2 && isPlateColorLetterLabel(result.labels[1]))
            score += 0.8f;
        else
            score -= 0.6f;

        for (int i = 2; i < len; ++i)
        {
            int label = result.labels[i];
            if (isPlateColorDigitLabel(label) || isPlateColorLetterLabel(label))
                score += 0.18f;
            else if (isPlateColorSpecialLabel(label))
                score += 0.08f;
            else if (isPlateColorProvinceLabel(label))
                score -= 0.25f;
        }

        score += min(1.5f, max(0.0f, result.avg_margin) * 0.08f);
        return score;
    }

    OcrResult decodePlateColor(const vector<vector<float>> &outputs) const
    {
        OcrResult result;
        result.debug.backend = ocrBackendName(ocr_backend_);
        result.debug.class_count = 78;
        result.debug.time_steps = 21;

        int char_output_index = findOutputBySize(outputs, 21 * 78);
        int color_output_index = findOutputBySize(outputs, 5);
        if (char_output_index < 0)
            return result;

        const vector<float> &output = outputs[char_output_index];
        const int T = 21;
        const int C = 78;
        const int blank = 0;
        bool channel_major = inferPlateColorChannelMajor(char_output_index);
        result.debug.channel_major = channel_major;
        copyOutputShape(result, char_output_index);

        int prev = -1;
        int kept = 0;
        float conf_sum = 0.0f;
        float margin_sum = 0.0f;

        for (int t = 0; t < T; ++t)
        {
            float max_val = -1e30f;
            float second_val = -1e30f;
            int cls = 0;

            for (int c = 0; c < C; ++c)
            {
                size_t idx = channel_major
                                 ? static_cast<size_t>(c) * T + t
                                 : static_cast<size_t>(t) * C + c;
                if (idx >= output.size())
                    continue;

                float v = output[idx];
                if (v > max_val)
                {
                    second_val = max_val;
                    max_val = v;
                    cls = c;
                }
                else if (v > second_val)
                {
                    second_val = v;
                }
            }

            if (cls != blank && cls != prev && cls < static_cast<int>(PLATE_COLOR_CHARS.size()))
            {
                result.text += PLATE_COLOR_CHARS[cls];
                result.labels.push_back(cls);
                if (std::isfinite(max_val))
                    conf_sum += max_val;
                if (std::isfinite(max_val) && std::isfinite(second_val))
                    margin_sum += max_val - second_val;
                kept++;
            }
            prev = cls;
        }

        result.raw_text = result.text;
        if (kept > 0)
        {
            result.avg_confidence = conf_sum / kept;
            result.avg_margin = margin_sum / kept;
        }
        if (!std::isfinite(result.avg_confidence))
            result.avg_confidence = 0.0f;
        if (!std::isfinite(result.avg_margin))
            result.avg_margin = 0.0f;

        if (color_output_index >= 0)
        {
            const vector<float> &color_output = outputs[color_output_index];
            float best_score = -1e30f;
            int best_color = -1;
            for (int i = 0; i < static_cast<int>(color_output.size()) && i < static_cast<int>(PLATE_COLORS.size()); ++i)
            {
                if (color_output[i] > best_score)
                {
                    best_score = color_output[i];
                    best_color = i;
                }
            }
            if (best_color >= 0)
            {
                result.color = PLATE_COLORS[best_color];
                result.color_score = best_score;
                result.debug.color = result.color;
                result.debug.color_score = result.color_score;
            }
        }

        result.quality = scorePlateColorResult(result);
        return result;
    }

    static float scoreOcrResult(const OcrResult &result, int class_count)
    {
        if (result.text.empty() || result.labels.empty())
            return -1000.0f;

        int len = static_cast<int>(result.labels.size());
        float score = 0.0f;
        if (len == 7)
            score += 2.2f;
        else if (len == 8)
            score += 2.4f;
        else if (len == 6 || len == 9)
            score += 0.2f;
        else
            score -= 2.4f;

        if (isProvinceLabel(result.labels[0]))
            score += 1.2f;
        else
            score -= 1.4f;

        if (len >= 2 && isLetterLabel(result.labels[1], class_count))
            score += 1.0f;
        else
            score -= 1.0f;

        int alnum_ok = 0;
        int alnum_total = max(0, len - 2);
        for (int i = 2; i < len; ++i)
        {
            if (isAlphaNumLabel(result.labels[i], class_count))
                alnum_ok++;
            if (isProvinceLabel(result.labels[i]))
                score -= 0.5f;
        }
        if (alnum_total > 0)
        {
            score += 1.2f * static_cast<float>(alnum_ok) / alnum_total;
        }

        if (len == 8)
        {
            score += 0.4f;
            if (hasEnergyLetter(result.labels, class_count))
                score += 0.4f;
        }

        score += min(2.0f, max(0.0f, result.avg_margin) * 0.12f);
        return score;
    }

    static float roiCandidateBonus(const OcrResult &result)
    {
        if (result.text.empty())
            return 0.0f;
        if (result.tag == "v3_roi")
            return 0.18f;
        if (result.tag == "aspect")
            return 0.12f;
        if (result.tag == "tight")
            return 0.08f;
        return 0.0f;
    }

    static bool isBetterOcrResult(const OcrResult &candidate, const OcrResult &best)
    {
        if (candidate.text.empty())
            return false;
        if (best.text.empty())
            return true;

        if (candidate.quality > best.quality + 0.05f)
            return true;
        if (best.quality > candidate.quality + 0.05f)
            return false;

        int cand_len = static_cast<int>(candidate.labels.size());
        int best_len = static_cast<int>(best.labels.size());
        int cand_dist = min(abs(cand_len - 7), abs(cand_len - 8));
        int best_dist = min(abs(best_len - 7), abs(best_len - 8));
        if (cand_dist != best_dist)
            return cand_dist < best_dist;

        return candidate.avg_margin > best.avg_margin;
    }

    static bool isProvinceLabel(int label)
    {
        return label >= 0 && label <= 30;
    }

    static bool isLetterLabel(int label, int class_count)
    {
        int blank = class_count - 1;
        return label >= 41 && label < blank;
    }

    static bool isAlphaNumLabel(int label, int class_count)
    {
        int blank = class_count - 1;
        return label >= 31 && label < blank;
    }

    static bool isEnergyLetterLabel(int label, int class_count)
    {
        int blank = class_count - 1;
        return label >= 0 && label < blank && (label == 44 || label == 46);
    }

    static bool hasEnergyLetter(const vector<int> &labels, int class_count)
    {
        for (size_t i = 2; i < labels.size(); ++i)
        {
            if (isEnergyLetterLabel(labels[i], class_count))
                return true;
        }
        return false;
    }

    static bool isV3PreferredResult(const OcrResult &result)
    {
        int len = static_cast<int>(result.labels.size());
        return !result.text.empty() && len >= 6;
    }

    static bool isAsciiPlateLetter(const string &ch)
    {
        return ch.size() == 1 && ch[0] >= 'A' && ch[0] <= 'Z' &&
               ch[0] != 'I' && ch[0] != 'O';
    }

    static bool isAsciiPlateDigit(const string &ch)
    {
        return ch.size() == 1 && ch[0] >= '0' && ch[0] <= '9';
    }

    static bool isStandardProvinceText(const string &ch)
    {
        static const char *const provinces[] = {
            "京", "津", "沪", "渝", "冀", "豫", "云", "辽",
            "黑", "湘", "皖", "鲁", "新", "苏", "浙", "赣",
            "鄂", "桂", "甘", "晋", "蒙", "陕", "吉", "闽",
            "贵", "粤", "青", "藏", "川", "宁", "琼"};
        for (const char *province : provinces)
        {
            if (ch == province)
                return true;
        }
        return false;
    }

    static bool isAllowedPlateSuffix(const string &ch)
    {
        static const char *const suffixes[] = {
            "学", "警", "港", "澳", "挂", "使", "领",
            "民", "航", "危", "险", "品"};
        for (const char *suffix : suffixes)
        {
            if (ch == suffix)
                return true;
        }
        return false;
    }

    static bool isValidPlateTextFormat(const string &text)
    {
        const vector<string> chars = splitUtf8Chars(text);
        if (chars.size() != 7 && chars.size() != 8)
            return false;
        if (!isStandardProvinceText(chars[0]))
            return false;
        if (!isAsciiPlateLetter(chars[1]))
            return false;

        for (size_t i = 2; i < chars.size(); ++i)
        {
            if (isAsciiPlateDigit(chars[i]) || isAsciiPlateLetter(chars[i]))
                continue;
            if (i + 1 == chars.size() && isAllowedPlateSuffix(chars[i]))
                continue;
            return false;
        }
        return true;
    }

    static bool isStableCandidate(const OcrResult &result)
    {
        return !result.text.empty() && isValidPlateTextFormat(result.text);
    }

    static bool isUsableOcr(const OcrResult &result)
    {
        return isStableCandidate(result);
    }

    void pruneOcrTracks()
    {
        auto now = chrono::steady_clock::now();
        ocr_tracks_.erase(remove_if(ocr_tracks_.begin(), ocr_tracks_.end(),
                                    [&](const OcrTrack &track)
                                    {
                                        int age_ms = static_cast<int>(
                                            chrono::duration_cast<chrono::milliseconds>(now - track.last_seen).count());
                                        return age_ms > ocr_track_ttl_ms_;
                                    }),
                          ocr_tracks_.end());
    }

    string updateOcrTrack(const Rect &box, const OcrResult &result, int &track_id, string &out_stable_color)
    {
        auto now = chrono::steady_clock::now();
        int best_idx = -1;
        float best_iou = 0.0f;

        for (size_t i = 0; i < ocr_tracks_.size(); ++i)
        {
            float v = iouValue(box, ocr_tracks_[i].box);
            if (v > best_iou)
            {
                best_iou = v;
                best_idx = static_cast<int>(i);
            }
        }

        if (best_idx < 0 || best_iou < ocr_track_iou_)
        {
            OcrTrack track;
            track.id = next_ocr_track_id_++;
            track.box = box;
            track.last_seen = now;
            ocr_tracks_.push_back(track);
            best_idx = static_cast<int>(ocr_tracks_.size()) - 1;
        }

        OcrTrack &track = ocr_tracks_[best_idx];
        track.box = box;
        track.last_seen = now;
        track_id = track.id;

        if (isUsableOcr(result))
        {
            track.history.push_back(result.text);
            while (static_cast<int>(track.history.size()) > ocr_vote_window_)
            {
                track.history.pop_front();
            }

            map<string, int> counts;
            for (const string &text : track.history)
            {
                counts[text]++;
            }

            string best_text;
            int best_count = 0;
            for (const auto &kv : counts)
            {
                if (kv.second > best_count)
                {
                    best_text = kv.first;
                    best_count = kv.second;
                }
            }

            if (best_count >= ocr_min_votes_)
            {
                track.stable = best_text;
            }

            // Color temporal voting: plate color is physically constant, so
            // majority vote over the window filters occasional single-frame
            // misclassifications (e.g. blue flash-read as black/yellow).
            if (!result.color.empty())
            {
                track.color_history.push_back(result.color);
                while (static_cast<int>(track.color_history.size()) > ocr_vote_window_)
                {
                    track.color_history.pop_front();
                }

                map<string, int> color_counts;
                for (const string &c : track.color_history)
                {
                    color_counts[c]++;
                }

                string best_color;
                int best_color_count = 0;
                for (const auto &kv : color_counts)
                {
                    if (kv.second > best_color_count)
                    {
                        best_color = kv.first;
                        best_color_count = kv.second;
                    }
                }

                if (best_color_count >= ocr_min_votes_)
                {
                    track.stable_color = best_color;
                }
            }
        }

        out_stable_color = track.stable_color;
        logOcrResult(result, track.stable, track.id);
        return track.stable;
    }

    void logOcrResult(const OcrResult &result, const string &stable, int track_id)
    {
        auto now = chrono::steady_clock::now();
        int elapsed_ms = static_cast<int>(
            chrono::duration_cast<chrono::milliseconds>(now - last_ocr_log_time_).count());
        if (elapsed_ms < 500)
            return;
        if (result.text.empty() && stable.empty() && result.color.empty())
            return;

        cout << "OCR track=" << track_id
             << " raw=" << (result.text.empty() ? "-" : result.text)
             << " stable=" << (stable.empty() ? "-" : stable)
             << " score=" << result.quality
             << " margin=" << result.avg_margin;
        if (!result.color.empty())
        {
            cout << " color=" << result.color
                 << " color_score=" << result.color_score;
        }
        if (result.box.area() > 0)
        {
            cout << " box=" << result.box.x << "," << result.box.y
                 << "," << result.box.width << "x" << result.box.height;
        }
        cout << endl;
        last_ocr_log_time_ = now;
    }

    void saveDebugOcr(const OcrResult &result, size_t det_index, uint64_t frame_seq = 0)
    {
        if (!debug_ocr_ || frame_seq == 0 || (frame_seq % 100) != 0)
            return;
        mkdir(debug_ocr_dir_.c_str(), 0777);

        uint64_t seq = ++debug_ocr_seq_;
        ostringstream base;
        base << debug_ocr_dir_ << "/ocr_"
             << "f" << setw(6) << setfill('0') << frame_seq
             << "_s" << setw(4) << seq
             << "_d" << det_index << "_" << result.tag;

        if (!result.crop.empty())
        {
            imwrite(base.str() + "_crop.jpg", result.crop);
        }
        if (!result.input.empty())
        {
            imwrite(base.str() + "_input.jpg", result.input);
        }

        ofstream ofs((base.str() + ".txt").c_str());
        if (ofs)
        {
            ofs << "raw=" << result.text << "\n"
                << "quality=" << result.quality << "\n"
                << "avg_margin=" << result.avg_margin << "\n"
                << "avg_confidence=" << result.avg_confidence << "\n"
                << "backend=" << result.debug.backend << "\n"
                << "color=" << (result.color.empty() ? result.debug.color : result.color) << "\n"
                << "color_score=" << (result.color.empty() ? result.debug.color_score : result.color_score) << "\n"
                << "box=" << result.box.x << "," << result.box.y << ","
                << result.box.width << "," << result.box.height << "\n"
                << "roi_size=" << result.box.width << "x" << result.box.height << "\n"
                << "input_size=" << result.input.cols << "x" << result.input.rows << "\n"
                << "input_color=" << result.debug.input_color << "\n"
                << "input_layout=" << result.debug.input_layout << "\n"
                << "input_type=" << result.debug.input_type << "\n"
                << "rknn_input_type=" << result.debug.rknn_input_type << "\n"
                << "class_count=" << result.debug.class_count << "\n"
                << "time_steps=" << result.debug.time_steps << "\n"
                << "channel_major=" << (result.debug.channel_major ? "true" : "false") << "\n"
                << "elapsed_ms=" << fixed << setprecision(3) << result.debug.elapsed_ms << "\n"
                << "output_shape=";
            for (size_t i = 0; i < result.debug.output_shape.size(); ++i)
            {
                if (i)
                    ofs << ",";
                ofs << result.debug.output_shape[i];
            }
            ofs << "\n";
        }

        if (!result.debug.output_probs.empty())
        {
            ofstream probs((base.str() + "_prob.txt").c_str());
            if (probs)
            {
                probs << setprecision(9);
                for (size_t i = 0; i < result.debug.output_probs.size(); ++i)
                {
                    probs << result.debug.output_probs[i];
                    probs << (((i + 1) % 16 == 0) ? '\n' : ' ');
                }
                probs << "\n";
            }
        }

        cout << "OCR debug frame=" << frame_seq
             << " det=" << det_index
             << " roi=" << result.box.width << "x" << result.box.height
             << " input=" << result.input.cols << "x" << result.input.rows
             << " color=" << result.debug.input_color
             << " type=" << result.debug.input_type
             << " layout=" << result.debug.input_layout
             << " backend=" << result.debug.backend
             << " plate_color=" << (result.color.empty() ? "-" : result.color)
             << " rknn_input=" << result.debug.rknn_input_type
             << " output_shape=[";
        for (size_t i = 0; i < result.debug.output_shape.size(); ++i)
        {
            if (i)
                cout << ",";
            cout << result.debug.output_shape[i];
        }
        cout << "] C=" << result.debug.class_count
             << " T=" << result.debug.time_steps
             << " text=" << (result.text.empty() ? "-" : result.text)
             << " cost_ms=" << fixed << setprecision(3) << result.debug.elapsed_ms
             << endl;
    }

    static Scalar plateDisplayColor(const string &plate_color)
    {
        // 处理多标签颜色 (如 "黄色+绿色" → 取第一个颜色)
        string first_color = plate_color;
        auto plus_pos = plate_color.find('+');
        if (plus_pos != string::npos)
            first_color = plate_color.substr(0, plus_pos);

        // OpenCV使用BGR顺序。黑色采用中灰作为"可视化黑色"，避免在深色
        // 遮罩和夜间画面上完全不可见；颜色文字本身仍明确显示"黑色"。
        if (first_color == "\351\273\221\350\211\262")
            return Scalar(170, 170, 170);
        if (first_color == "\350\223\235\350\211\262")
            return Scalar(255, 145, 35);
        if (first_color == "\347\273\277\350\211\262")
            return Scalar(90, 225, 105);
        if (first_color == "\347\231\275\350\211\262")
            return Scalar(245, 245, 245);
        if (first_color == "\351\273\204\350\211\262")
            return Scalar(0, 225, 255);
        return Scalar(0, 255, 0);
    }

    Size measurePlateText(const string &text, int font_px, int &baseline)
    {
#ifdef HAVE_DIRECT_FREETYPE
        if (text_renderer_.isReady())
        {
            int width = 0;
            const vector<string> chars = splitUtf8Chars(text);
            for (const string &ch : chars)
            {
                if (ch.size() > 1)
                    width += font_px;
                else if (ch == " ")
                    width += max(4, font_px / 3);
                else
                    width += max(5, static_cast<int>(font_px * 0.62f));
            }
            baseline = max(4, font_px / 5);
            return Size(max(1, width), font_px);
        }
#endif
#ifdef HAVE_OPENCV_FREETYPE
        if (ft2_)
            return ft2_->getTextSize(text, font_px, -1, &baseline);
#endif
        return getTextSize(text, FONT_HERSHEY_SIMPLEX, 0.8, 2, &baseline);
    }

    void drawPlateText(Mat &img,
                       const string &text,
                       Point org,
                       const Scalar &color,
                       int font_px)
    {
#ifdef HAVE_DIRECT_FREETYPE
        if (text_renderer_.isReady() &&
            text_renderer_.drawText(img, text, org, font_px, color))
        {
            return;
        }
#endif
#ifdef HAVE_OPENCV_FREETYPE
        if (ft2_)
        {
            ft2_->putText(img, text, org, font_px, color, -1, LINE_AA, false);
            return;
        }
#endif
        putText(img, text, org, FONT_HERSHEY_SIMPLEX, 0.8, color, 2, LINE_AA);
    }

    void drawPlateLabel(Mat &img,
                        const string &label,
                        const Rect &plate_box,
                        const Scalar &display_color)
    {
        if (img.empty() || label.empty())
            return;

        const int font_px = 28;
        const int pad_x = 8;
        const int pad_y = 5;
        int baseline = 0;
        const Size text_size = measurePlateText(label, font_px, baseline);

        const int mask_w = min(img.cols, max(1, text_size.width + pad_x * 2));
        const int mask_h = min(img.rows, max(1, text_size.height + baseline + pad_y * 2));
        const int max_x = max(0, img.cols - mask_w);
        const int mask_x = max(0, min(plate_box.x, max_x));

        int mask_y = plate_box.y - mask_h - 3;
        if (mask_y < 0)
            mask_y = plate_box.y + plate_box.height + 3;
        mask_y = max(0, min(mask_y, max(0, img.rows - mask_h)));

        const Rect mask_rect(mask_x, mask_y, mask_w, mask_h);
        Mat mask_roi = img(mask_rect);
        Mat mask_layer(mask_roi.size(), mask_roi.type(), Scalar(5, 12, 17));

        // 70%不透明深色遮罩：70%遮罩色 + 30%原始画面。
        addWeighted(mask_layer, 0.70, mask_roi, 0.30, 0.0, mask_roi);
        rectangle(img, mask_rect, display_color, 1, LINE_AA);

        const Point text_org(mask_x + pad_x,
                             mask_y + pad_y + text_size.height);
        drawPlateText(img, label, text_org, display_color, font_px);
    }

    static void drawSmallStatus(Mat &img, const string &text)
    {
        putText(img, text, Point(16, 32), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0, 255, 255), 2, LINE_AA);
    }
};

class V4L2Capture
{
public:
    V4L2Capture() = default;
    ~V4L2Capture() { closeDevice(); }

    V4L2Capture(const V4L2Capture &) = delete;
    V4L2Capture &operator=(const V4L2Capture &) = delete;

    bool openDevice(const string &video_dev, int width, int height, int fps, const string &fourcc)
    {
        closeDevice();

        string dev = normalizeDevice(video_dev);
        fd_ = open(dev.c_str(), O_RDWR | O_NONBLOCK, 0);
        if (fd_ < 0)
        {
            cerr << "Failed to open capture device: " << dev << ", " << strerror(errno) << endl;
            return false;
        }

        v4l2_capability cap{};
        if (!xioctl(VIDIOC_QUERYCAP, &cap))
        {
            cerr << "VIDIOC_QUERYCAP failed: " << strerror(errno) << endl;
            closeDevice();
            return false;
        }
        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE) || !(cap.capabilities & V4L2_CAP_STREAMING))
        {
            cerr << "Device does not support V4L2 capture streaming: " << dev << endl;
            closeDevice();
            return false;
        }

        memset(&fmt_, 0, sizeof(fmt_));
        fmt_.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt_.fmt.pix.width = width > 0 ? static_cast<uint32_t>(width) : 1280;
        fmt_.fmt.pix.height = height > 0 ? static_cast<uint32_t>(height) : 720;
        fmt_.fmt.pix.pixelformat = fourccToPixelFormat(fourcc);
        fmt_.fmt.pix.field = V4L2_FIELD_ANY;

        if (!xioctl(VIDIOC_S_FMT, &fmt_))
        {
            cerr << "VIDIOC_S_FMT failed for " << fourcc << ": " << strerror(errno) << endl;
            closeDevice();
            return false;
        }

        if (fps > 0)
        {
            v4l2_streamparm parm{};
            parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            if (xioctl(VIDIOC_G_PARM, &parm))
            {
                parm.parm.capture.timeperframe.numerator = 1;
                parm.parm.capture.timeperframe.denominator = static_cast<uint32_t>(fps);
                xioctl(VIDIOC_S_PARM, &parm);
            }
        }

        if (!initMmap())
        {
            closeDevice();
            return false;
        }

        for (size_t i = 0; i < buffers_.size(); ++i)
        {
            v4l2_buffer buf{};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = static_cast<uint32_t>(i);
            if (!xioctl(VIDIOC_QBUF, &buf))
            {
                cerr << "VIDIOC_QBUF failed: " << strerror(errno) << endl;
                closeDevice();
                return false;
            }
        }

        v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (!xioctl(VIDIOC_STREAMON, &type))
        {
            cerr << "VIDIOC_STREAMON failed: " << strerror(errno) << endl;
            closeDevice();
            return false;
        }
        streaming_ = true;

        cout << "Capture opened: " << dev
             << ", actual=" << fmt_.fmt.pix.width << "x" << fmt_.fmt.pix.height
             << ", fourcc=" << pixelFormatToString(fmt_.fmt.pix.pixelformat) << endl;
        return true;
    }

    bool read(Mat &frame)
    {
        if (fd_ < 0 || !streaming_)
            return false;

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_, &fds);

        timeval tv{};
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        int ret = select(fd_ + 1, &fds, nullptr, nullptr, &tv);
        if (ret <= 0)
            return false;

        v4l2_buffer buf{};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (!xioctl(VIDIOC_DQBUF, &buf))
        {
            if (errno != EAGAIN)
            {
                cerr << "VIDIOC_DQBUF failed: " << strerror(errno) << endl;
            }
            return false;
        }

        if (buf.index >= buffers_.size())
        {
            cerr << "VIDIOC_DQBUF returned invalid buffer index: " << buf.index << endl;
            return false;
        }

        size_t used = buf.bytesused;
        if (used == 0 || used > buffers_[buf.index].length)
        {
            used = buffers_[buf.index].length;
        }

        bool ok = used > 0 && convertBufferToMat(buffers_[buf.index].start, used, frame);

        if (!xioctl(VIDIOC_QBUF, &buf))
        {
            cerr << "VIDIOC_QBUF requeue failed: " << strerror(errno) << endl;
            return false;
        }

        return ok && !frame.empty();
    }

private:
    struct Buffer
    {
        void *start = nullptr;
        size_t length = 0;
    };

    int fd_ = -1;
    bool streaming_ = false;
    v4l2_format fmt_{};
    vector<Buffer> buffers_;

    bool xioctl(unsigned long request, void *arg)
    {
        int ret;
        do
        {
            ret = ioctl(fd_, request, arg);
        } while (ret == -1 && errno == EINTR);
        return ret != -1;
    }

    bool initMmap()
    {
        v4l2_requestbuffers req{};
        req.count = 4;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (!xioctl(VIDIOC_REQBUFS, &req))
        {
            cerr << "VIDIOC_REQBUFS failed: " << strerror(errno) << endl;
            return false;
        }
        if (req.count < 2)
        {
            cerr << "Insufficient V4L2 mmap buffers." << endl;
            return false;
        }

        buffers_.resize(req.count);
        for (uint32_t i = 0; i < req.count; ++i)
        {
            v4l2_buffer buf{};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if (!xioctl(VIDIOC_QUERYBUF, &buf))
            {
                cerr << "VIDIOC_QUERYBUF failed: " << strerror(errno) << endl;
                return false;
            }

            buffers_[i].length = buf.length;
            buffers_[i].start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE,
                                     MAP_SHARED, fd_, buf.m.offset);
            if (buffers_[i].start == MAP_FAILED)
            {
                buffers_[i].start = nullptr;
                cerr << "mmap V4L2 buffer failed: " << strerror(errno) << endl;
                return false;
            }
        }

        return true;
    }

    bool convertBufferToMat(void *data, size_t size, Mat &frame)
    {
        frame.release();
        if (!data || size == 0)
            return false;

        int width = static_cast<int>(fmt_.fmt.pix.width);
        int height = static_cast<int>(fmt_.fmt.pix.height);
        uint32_t pixfmt = fmt_.fmt.pix.pixelformat;

        if (pixfmt == V4L2_PIX_FMT_MJPEG || pixfmt == V4L2_PIX_FMT_JPEG)
        {
            try
            {
                Mat encoded(1, static_cast<int>(size), CV_8UC1, data);
                frame = imdecode(encoded, IMREAD_COLOR);
            }
            catch (const cv::Exception &e)
            {
                cerr << "MJPEG imdecode failed, bytes=" << size << ": " << e.what() << endl;
                frame.release();
                return false;
            }
            return !frame.empty();
        }

        if (pixfmt == V4L2_PIX_FMT_YUYV)
        {
            if (size < static_cast<size_t>(width * height * 2))
                return false;
            Mat yuv(height, width, CV_8UC2, data);
            cvtColor(yuv, frame, COLOR_YUV2BGR_YUYV);
            return !frame.empty();
        }

        if (pixfmt == V4L2_PIX_FMT_UYVY)
        {
            if (size < static_cast<size_t>(width * height * 2))
                return false;
            Mat yuv(height, width, CV_8UC2, data);
            cvtColor(yuv, frame, COLOR_YUV2BGR_UYVY);
            return !frame.empty();
        }

        if (pixfmt == V4L2_PIX_FMT_BGR24)
        {
            if (size < static_cast<size_t>(width * height * 3))
                return false;
            Mat bgr(height, width, CV_8UC3, data);
            frame = bgr.clone();
            return true;
        }

        if (pixfmt == V4L2_PIX_FMT_RGB24)
        {
            if (size < static_cast<size_t>(width * height * 3))
                return false;
            Mat rgb(height, width, CV_8UC3, data);
            cvtColor(rgb, frame, COLOR_RGB2BGR);
            return !frame.empty();
        }

        cerr << "Unsupported V4L2 pixel format: " << pixelFormatToString(pixfmt) << endl;
        return false;
    }

    void closeDevice()
    {
        if (fd_ >= 0 && streaming_)
        {
            v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            xioctl(VIDIOC_STREAMOFF, &type);
            streaming_ = false;
        }

        for (Buffer &b : buffers_)
        {
            if (b.start)
            {
                munmap(b.start, b.length);
                b.start = nullptr;
                b.length = 0;
            }
        }
        buffers_.clear();

        if (fd_ >= 0)
        {
            close(fd_);
            fd_ = -1;
        }
    }

    static uint32_t fourccToPixelFormat(const string &fourcc)
    {
        string f = fourcc;
        transform(f.begin(), f.end(), f.begin(), [](unsigned char c)
                  { return static_cast<char>(toupper(c)); });

        if (f == "MJPG" || f == "MJPEG")
            return V4L2_PIX_FMT_MJPEG;
        if (f == "JPEG")
            return V4L2_PIX_FMT_JPEG;
        if (f == "YUYV")
            return V4L2_PIX_FMT_YUYV;
        if (f == "UYVY")
            return V4L2_PIX_FMT_UYVY;
        if (f == "BGR3")
            return V4L2_PIX_FMT_BGR24;
        if (f == "RGB3")
            return V4L2_PIX_FMT_RGB24;
        if (f.size() >= 4)
            return v4l2_fourcc(f[0], f[1], f[2], f[3]);
        return V4L2_PIX_FMT_MJPEG;
    }

    static string pixelFormatToString(uint32_t pixfmt)
    {
        char s[5] = {
            static_cast<char>(pixfmt & 0xFF),
            static_cast<char>((pixfmt >> 8) & 0xFF),
            static_cast<char>((pixfmt >> 16) & 0xFF),
            static_cast<char>((pixfmt >> 24) & 0xFF),
            0};
        return string(s);
    }

    static string normalizeDevice(const string &video_dev)
    {
        bool numeric = !video_dev.empty() &&
                       all_of(video_dev.begin(), video_dev.end(), [](unsigned char c)
                              { return isdigit(c); });
        if (numeric)
            return "/dev/video" + video_dev;
        return video_dev;
    }
};

class PcieRawFrameRing4
{
public:
    static constexpr int kSlotCount = 4;
    static constexpr size_t kSlotBytes = 4U * 1024U * 1024U;
    static constexpr size_t kRingBytes = kSlotCount * kSlotBytes;

    enum class SlotState : uint8_t
    {
        FREE = 0,
        FILLING,
        READY,
        READING
    };

    struct Slot
    {
        uint8_t *data = nullptr;
        atomic<SlotState> state{SlotState::FREE};
        atomic<uint64_t> sequence{0};
        ssize_t driver_status = 0;
    };

    PcieRawFrameRing4() = default;
    ~PcieRawFrameRing4() { release(); }

    PcieRawFrameRing4(const PcieRawFrameRing4 &) = delete;
    PcieRawFrameRing4 &operator=(const PcieRawFrameRing4 &) = delete;

    bool init(int width, int height)
    {
        release();

        if (width <= 0 || height <= 0)
            return false;

        frame_bytes_ = static_cast<size_t>(width) * static_cast<size_t>(height) * 2U;
        if (frame_bytes_ > kSlotBytes)
        {
            cerr << "[PCIe ring] RGB565 frame needs " << frame_bytes_
                 << " bytes, exceeding the fixed 4 MiB slot." << endl;
            frame_bytes_ = 0;
            return false;
        }

        int mmap_flags = MAP_PRIVATE | MAP_ANONYMOUS;
#ifdef MAP_POPULATE
        mmap_flags |= MAP_POPULATE;
#endif
        mapping_ = mmap(nullptr, kRingBytes, PROT_READ | PROT_WRITE,
                        mmap_flags, -1, 0);
        if (mapping_ == MAP_FAILED)
        {
            cerr << "[PCIe ring] mmap 16 MiB failed: " << strerror(errno) << endl;
            return false;
        }

        uint8_t *base = static_cast<uint8_t *>(mapping_);
        for (int i = 0; i < kSlotCount; ++i)
        {
            slots_[i].data = base + static_cast<size_t>(i) * kSlotBytes;
            slots_[i].state.store(SlotState::FREE, memory_order_relaxed);
            slots_[i].sequence.store(0, memory_order_relaxed);
            slots_[i].driver_status = 0;
        }
        next_write_ = 0;
        producer_sequence_ = 0;

        cout << "[PCIe ring] allocated 16 MiB as four 4 MiB RGB565 slots"
             << ", frame_bytes=" << frame_bytes_ << endl;
        for (int i = 0; i < kSlotCount; ++i)
        {
            cout << "[PCIe ring] slot" << i
                 << " virtual_address=" << static_cast<void *>(slots_[i].data)
                 << endl;
        }
        return true;
    }

    void release()
    {
        ready_cv_.notify_all();
        if (mapping_ != MAP_FAILED)
        {
            munmap(mapping_, kRingBytes);
            mapping_ = MAP_FAILED;
        }
        frame_bytes_ = 0;
        for (int i = 0; i < kSlotCount; ++i)
        {
            slots_[i].data = nullptr;
            slots_[i].state.store(SlotState::FREE, memory_order_relaxed);
            slots_[i].sequence.store(0, memory_order_relaxed);
            slots_[i].driver_status = 0;
        }
    }

    size_t frameBytes() const { return frame_bytes_; }

    uint8_t *writeSlotData(int index)
    {
        if (index < 0 || index >= kSlotCount)
            return nullptr;
        if (slots_[index].state.load(memory_order_acquire) != SlotState::FILLING)
            return nullptr;
        return slots_[index].data;
    }

    // OpenCV's external-buffer Mat constructor takes void*, although cvtColor
    // only reads this RGB565 buffer.  Access is allowed exclusively in READING.
    uint8_t *readSlotData(int index)
    {
        if (index < 0 || index >= kSlotCount)
            return nullptr;
        if (slots_[index].state.load(memory_order_acquire) != SlotState::READING)
            return nullptr;
        return slots_[index].data;
    }

    int acquireWriteSlot(bool &reclaimed_ready)
    {
        reclaimed_ready = false;

        for (int offset = 0; offset < kSlotCount; ++offset)
        {
            int index = (next_write_ + offset) % kSlotCount;
            SlotState expected = SlotState::FREE;
            if (slots_[index].state.compare_exchange_strong(
                    expected, SlotState::FILLING,
                    memory_order_acq_rel, memory_order_acquire))
            {
                next_write_ = (index + 1) % kSlotCount;
                return index;
            }
        }

        // The consumer is intentionally latest-frame based.  If the ring is
        // full, replace the oldest unread READY frame, never a READING frame.
        for (int attempt = 0; attempt < kSlotCount; ++attempt)
        {
            int oldest_index = -1;
            uint64_t oldest_sequence = UINT64_MAX;
            for (int i = 0; i < kSlotCount; ++i)
            {
                if (slots_[i].state.load(memory_order_acquire) != SlotState::READY)
                    continue;
                uint64_t sequence = slots_[i].sequence.load(memory_order_acquire);
                if (sequence < oldest_sequence)
                {
                    oldest_sequence = sequence;
                    oldest_index = i;
                }
            }

            if (oldest_index < 0)
                break;

            SlotState expected = SlotState::READY;
            if (slots_[oldest_index].state.compare_exchange_strong(
                    expected, SlotState::FILLING,
                    memory_order_acq_rel, memory_order_acquire))
            {
                reclaimed_ready = true;
                next_write_ = (oldest_index + 1) % kSlotCount;
                return oldest_index;
            }
        }

        return -1;
    }

    void cancelWriteSlot(int index)
    {
        if (index < 0 || index >= kSlotCount)
            return;
        slots_[index].driver_status = 0;
        SlotState expected = SlotState::FILLING;
        if (!slots_[index].state.compare_exchange_strong(
                expected, SlotState::FREE,
                memory_order_acq_rel, memory_order_acquire))
        {
            cerr << "[PCIe ring] cancel rejected: slot is not FILLING" << endl;
        }
    }

    void publishWriteSlot(int index, ssize_t driver_status)
    {
        if (index < 0 || index >= kSlotCount)
            return;

        slots_[index].driver_status = driver_status;
        slots_[index].sequence.store(++producer_sequence_, memory_order_relaxed);
        SlotState expected = SlotState::FILLING;
        if (!slots_[index].state.compare_exchange_strong(
                expected, SlotState::READY,
                memory_order_acq_rel, memory_order_acquire))
        {
            cerr << "[PCIe ring] publish rejected: slot is not FILLING" << endl;
            return;
        }
        ready_cv_.notify_one();
    }

    bool waitAcquireLatest(int &index, uint64_t &sequence,
                           ssize_t &driver_status, int &older_frames_dropped)
    {
        index = -1;
        sequence = 0;
        driver_status = 0;
        older_frames_dropped = 0;

        while (g_running)
        {
            if (tryAcquireLatest(index, sequence, driver_status,
                                 older_frames_dropped))
                return true;

            unique_lock<mutex> lock(wait_mutex_);
            ready_cv_.wait_for(lock, chrono::milliseconds(50), [&]
                               { return !g_running || hasReadyFrame(); });
        }
        return false;
    }

    void releaseReadSlot(int index)
    {
        if (index < 0 || index >= kSlotCount)
            return;
        SlotState expected = SlotState::READING;
        if (!slots_[index].state.compare_exchange_strong(
                expected, SlotState::FREE,
                memory_order_acq_rel, memory_order_acquire))
        {
            cerr << "[PCIe ring] release rejected: slot is not READING" << endl;
        }
    }

    void wakeAll() { ready_cv_.notify_all(); }

private:
    bool hasReadyFrame() const
    {
        for (int i = 0; i < kSlotCount; ++i)
        {
            if (slots_[i].state.load(memory_order_acquire) == SlotState::READY)
                return true;
        }
        return false;
    }

    bool tryAcquireLatest(int &index, uint64_t &sequence,
                          ssize_t &driver_status, int &older_frames_dropped)
    {
        for (int attempt = 0; attempt < kSlotCount; ++attempt)
        {
            int newest_index = -1;
            uint64_t newest_sequence = 0;
            for (int i = 0; i < kSlotCount; ++i)
            {
                if (slots_[i].state.load(memory_order_acquire) != SlotState::READY)
                    continue;
                uint64_t candidate = slots_[i].sequence.load(memory_order_acquire);
                if (newest_index < 0 || candidate > newest_sequence)
                {
                    newest_sequence = candidate;
                    newest_index = i;
                }
            }

            if (newest_index < 0)
                return false;

            SlotState expected = SlotState::READY;
            if (!slots_[newest_index].state.compare_exchange_strong(
                    expected, SlotState::READING,
                    memory_order_acq_rel, memory_order_acquire))
                continue;

            index = newest_index;
            sequence = slots_[newest_index].sequence.load(memory_order_acquire);
            driver_status = slots_[newest_index].driver_status;

            // Drop every older unread frame so inference/output never build a
            // growing FIFO delay behind the live PCIe stream.
            for (int i = 0; i < kSlotCount; ++i)
            {
                if (i == newest_index)
                    continue;
                SlotState old_expected = SlotState::READY;
                if (slots_[i].state.compare_exchange_strong(
                        old_expected, SlotState::FREE,
                        memory_order_acq_rel, memory_order_acquire))
                    older_frames_dropped++;
            }
            return true;
        }
        return false;
    }

    void *mapping_ = MAP_FAILED;
    size_t frame_bytes_ = 0;
    Slot slots_[kSlotCount];
    int next_write_ = 0;
    uint64_t producer_sequence_ = 0;
    mutex wait_mutex_;
    condition_variable ready_cv_;
};

class PcieRgb565Capture
{
public:
    PcieRgb565Capture() = default;
    ~PcieRgb565Capture() { closeDevice(); }

    PcieRgb565Capture(const PcieRgb565Capture &) = delete;
    PcieRgb565Capture &operator=(const PcieRgb565Capture &) = delete;

    bool openDevice(const string &pcie_dev, const string &mem_dev, int width, int height)
    {
        closeDevice();

        if (width <= 0 || height <= 0)
        {
            cerr << "Invalid PCIe frame size: " << width << "x" << height << endl;
            return false;
        }

        pcie_dev_ = pcie_dev;
        mem_dev_ = mem_dev;
        width_ = width;
        height_ = height;
        frame_size_ = static_cast<size_t>(width_) * static_cast<size_t>(height_) * 2U;

        // Follow the working Qt sequence exactly:
        // getDevice() -> current_len/offset -> dma_map(1) -> BAR0 start command.
        if (!pango_device_.getDevice(pcie_dev_, mem_dev_))
        {
            cerr << "[PCIe] getDevice failed: " << pango_device_.lastError() << endl;
            closeDevice();
            return false;
        }

        static const unsigned int kDmaChunkBytes = 3840U;
        if (!pango_device_.configureDma(kDmaChunkBytes, 0))
        {
            cerr << "[PCIe] DMA configuration failed: "
                 << pango_device_.lastError() << endl;
            closeDevice();
            return false;
        }

        if (!pango_device_.dmaMap(true))
        {
            cerr << "[PCIe] DMA mapping failed: "
                 << pango_device_.lastError() << endl;
            closeDevice();
            return false;
        }

        opened_ = true;
        read_count_ = 0;
        not_ready_count_ = 0;
        last_not_ready_log_ = chrono::steady_clock::now();

        cout << "PCIe capture prepared through pango_fun: " << pcie_dev_
             << ", frame=" << width_ << "x" << height_
             << ", wire_format=RGB565_LE, byte_swap=off"
             << ", row_bytes=" << static_cast<size_t>(width_) * 2U
             << ", frame_bytes=" << frame_size_
             << ", dma_chunk_bytes=" << kDmaChunkBytes << endl;
        return true;
    }

    bool startCapture()
    {
        if (!opened_)
        {
            cerr << "[PCIe] startCapture called before the device was prepared" << endl;
            return false;
        }

        if (!pango_device_.startCapture())
        {
            cerr << "[PCIe] capture start failed: "
                 << pango_device_.lastError() << endl;
            return false;
        }

        cout << "PCIe capture started through pango_fun: legacy driver read mode"
             << ", success=rd_status>0, not_ready=errno EPERM" << endl;
        return true;
    }

    bool readRawFrame(uint8_t *destination, size_t capacity,
                      ssize_t &driver_status)
    {
        driver_status = 0;
        if (!opened_ || !destination || capacity < frame_size_)
            return false;

        errno = 0;
        const ssize_t n = pango_device_.dmaRead(destination, frame_size_);
        int err = errno;

        if (n <= 0)
        {
            if (err == EINTR)
                return false;

            // The legacy pango driver returns a raw -1 when BAR0+0x140 is
            // zero.  The syscall layer exposes that as errno=EPERM, but it
            // means "frame not ready", not a device permission failure.
            if (n == 0 || err == EAGAIN || err == EWOULDBLOCK || err == EPERM)
            {
                not_ready_count_++;
                const auto now = chrono::steady_clock::now();
                const auto log_ms = chrono::duration_cast<chrono::milliseconds>(
                                        now - last_not_ready_log_)
                                        .count();
                if (!g_quiet_pcie.load() && log_ms >= 1000)
                {
                    cerr << "[PCIe] waiting for FPGA frame-ready status: attempts="
                         << not_ready_count_;
                    if (n < 0)
                    {
                        cerr << ", last_errno=" << err
                             << " (" << strerror(err) << ")";
                    }
                    cerr << endl;
                    last_not_ready_log_ = now;
                }
                usleep(1000);
                return false;
            }

            cerr << "[PCIe] read failed: errno=" << err
                 << " (" << strerror(err) << ")" << endl;
            usleep(1000);
            return false;
        }

        // This legacy driver returns a completion/status token rather than a
        // POSIX byte count.  pango_fun/driver must guarantee that frame_size_
        // bytes were copied before returning a positive status.
        driver_status = n;
        read_count_++;
        if (!g_quiet_pcie.load() && (read_count_ % 60) == 0)
        {
            cerr << "[PCIe] DMA frames: " << read_count_
                 << ", driver_status=" << n
                 << ", not_ready_attempts=" << not_ready_count_ << endl;
        }
        return true;
    }

    size_t frameSize() const { return frame_size_; }

    void closeDevice()
    {
        if (opened_ || pango_device_.isOpen())
            pango_device_.closeDevice();
        opened_ = false;
        frame_size_ = 0;
    }

private:
    string pcie_dev_ = PCIE_DRIVER_FILE_PATH;
    string mem_dev_ = MEM_FILE_PATH;
    int width_ = 1280;
    int height_ = 720;
    size_t frame_size_ = 0;
    PangoFun pango_device_;
    bool opened_ = false;
    uint64_t read_count_ = 0;
    uint64_t not_ready_count_ = 0;
    chrono::steady_clock::time_point last_not_ready_log_;
};

static void captureThread(const string &pcie_dev,
                          const string &mem_dev,
                          int width,
                          int height,
                          int fps,
                          PcieRawFrameRing4 *ring)
{
    // The read rate is deliberately bounded.  This avoids the old busy-drain
    // behaviour (35-40 reads/s even when the stream only needed 25 frames/s).
    if (!ring)
    {
        g_running = false;
        g_input_cv.notify_all();
        return;
    }

    PcieRgb565Capture cap;
    if (!cap.openDevice(pcie_dev, mem_dev, width, height) ||
        cap.frameSize() != ring->frameBytes())
    {
        g_running = false;
        g_input_cv.notify_all();
        ring->wakeAll();
        return;
    }

    if (!cap.startCapture())
    {
        g_running = false;
        g_input_cv.notify_all();
        ring->wakeAll();
        return;
    }

    uint64_t local_count = 0;
    auto t0 = chrono::steady_clock::now();
    const int read_fps = max(1, fps);
    const auto read_interval = chrono::microseconds(1000000 / read_fps);
    auto next_read_time = chrono::steady_clock::now();

    while (g_running)
    {
        // sleep_until does not consume a CPU core.  Keep DMA starts on the
        // requested cadence and never busy-loop to obtain more than read_fps.
        auto now = chrono::steady_clock::now();
        if (now < next_read_time)
            this_thread::sleep_until(next_read_time);
        else if (now > next_read_time + read_interval * 2)
            next_read_time = now;
        next_read_time += read_interval;

        if (!g_running)
            break;

        bool reclaimed_ready = false;
        int slot_index = ring->acquireWriteSlot(reclaimed_ready);
        if (slot_index < 0)
        {
            g_drop_count.fetch_add(1);
            usleep(500);
            continue;
        }
        if (reclaimed_ready)
            g_drop_count.fetch_add(1);

        ssize_t driver_status = 0;
        uint8_t *write_ptr = ring->writeSlotData(slot_index);
        if (!write_ptr)
        {
            cerr << "[PCIe ring] DMA rejected: slot is not FILLING" << endl;
            ring->cancelWriteSlot(slot_index);
            g_drop_count.fetch_add(1);
            continue;
        }

        const bool ok = cap.readRawFrame(write_ptr, ring->frameBytes(),
                                         driver_status);

        if (!ok)
        {
            ring->cancelWriteSlot(slot_index);
            continue;
        }

        ring->publishWriteSlot(slot_index, driver_status);
        local_count++;

        const auto log_now = chrono::steady_clock::now();
        auto ms = chrono::duration_cast<chrono::milliseconds>(log_now - t0).count();
        if (ms >= 1000)
        {
            if (!g_quiet_pcie.load())
            {
                cout << "PCIe DMA publish FPS: " << local_count
                     << ", infer_count=" << g_infer_count.load()
                     << ", encode_count=" << g_encode_count.load()
                     << ", stream_count=" << g_stream_count.load()
                     << ", drop_count=" << g_drop_count.load()
                     << " | LATENCY(us): ring=" << g_latency_ring_acquire_us.load()
                     << " conv=" << g_latency_convert_us.load()
                     << " iwait=" << g_latency_infer_wait_us.load()
                     << " infer=" << g_latency_infer_us.load()
                     << " ewait=" << g_latency_encode_wait_us.load()
                     << " enc=" << g_latency_encode_us.load()
                     << " mpp_pts=" << g_mpp_pts_delay_us.load()
                     << endl;
            }
            local_count = 0;
            t0 = log_now;
        }
    }

    ring->wakeAll();
}

static void rawFrameConvertThread(PcieRawFrameRing4 *ring,
                                  int width,
                                  int height)
{
    if (!ring)
        return;

    while (g_running)
    {
        auto t0 = chrono::steady_clock::now();
        int slot_index = -1;
        uint64_t sequence = 0;
        ssize_t driver_status = 0;
        int older_frames_dropped = 0;
        if (!ring->waitAcquireLatest(slot_index, sequence,
                                     driver_status, older_frames_dropped))
            break;

        auto t_acquire = chrono::steady_clock::now();
        if (older_frames_dropped > 0)
            g_drop_count.fetch_add(static_cast<uint64_t>(older_frames_dropped));

        Mat converted;
        try
        {
            uint8_t *read_ptr = ring->readSlotData(slot_index);
            if (!read_ptr)
            {
                cerr << "[PCIe ring] conversion rejected: slot is not READING" << endl;
            }
            else
            {
                Mat raw(height, width, CV_8UC2, read_ptr,
                        static_cast<size_t>(width) * 2U);
                cvtColor(raw, converted, COLOR_BGR5652BGR);

                static int dbg_d2 = 0;
                if (dbg_d2 == 0) {
                    dbg_d2++;
                    const unsigned char *dp = read_ptr;
                    cerr << "[D2] raw 32B: ";
                    for (int di = 0; di < 32; di++) cerr << hex << setw(2) << setfill('0') << (int)dp[di];
                    cerr << dec << endl;
                    imwrite("/userdata/debug_frame.png", converted);
                    cerr << "[D2] saved " << converted.cols << "x" << converted.rows << endl;
                }
            }
        }
        catch (const cv::Exception &e)
        {
            cerr << "[PCIe ring] RGB565 conversion failed: " << e.what() << endl;
        }

        auto t_convert = chrono::steady_clock::now();
        ring->releaseReadSlot(slot_index);

        if (!converted.empty())
            updateInputFrame(converted);

        g_latency_ring_acquire_us = chrono::duration_cast<chrono::microseconds>(t_acquire - t0).count();
        g_latency_convert_us = chrono::duration_cast<chrono::microseconds>(t_convert - t_acquire).count();

        (void)sequence;
        (void)driver_status;
    }
}

// ---- ADAS 全局状态：检测器、串口发送器、最新信号 ----
// 双上下文（--adas-workers 2）：同一个 .rknn 载入两份独立实例（各自 rknn_context
// 与预处理缓冲），两个 worker 线程轮流从「最新帧槽」取帧推理。RK3568 的 3 个 NPU
// core 共享算力，单帧算力不受影响，收益来自把 CPU 前处理/后处理与 NPU 计算重叠。
static const int kAdasMaxWorkers = 2;
static AdasDetector g_adas[kAdasMaxWorkers];
static int g_adas_workers = 1;              // 1 = 旧的单线程路径（默认），2 = 双上下文
static AdasSerialSender g_adas_serial;
static mutex g_adas_mutex;
static AdasSignals g_adas_signals;
static atomic<float> g_adas_conf_thres(0.25f);
static atomic<float> g_adas_nms_thres(0.45f);

// 派发状态：全部在 g_input_mutex 保护下读写，保证两个 worker 不会算同一帧
static uint64_t g_adas_dispatched_id = 0;                            // 已派发的最大帧号
static chrono::steady_clock::time_point g_adas_next_start =          // --infer-fps 节拍
    chrono::steady_clock::now() - chrono::seconds(1);
// 发布状态：帧号单调保护（并行时帧 N+1 可能先算完，旧结果不得覆盖新结果）
static mutex g_adas_result_mutex;
static uint64_t g_adas_result_id = 0;

// ---- AI 识别暂停闸门（四路拼接视频 / 倒车辅助线）----
// 上位机在「显示界面切到四路拼接」或「打开倒车辅助线」时，经 UDP 控制口
// （默认 9998）下发 "AI 0"（暂停识别）/ "AI 1"（恢复识别），也接受单字节
// 0x00 / 0x01。暂停后 worker 不取帧、不推理、不发布，并清空缓存框，画面回到
// 干净的原始视频；串口保持最后一帧状态（上位机会显示「识别已暂停」）。
// 恢复后自动接上最新采集帧，无需重启进程。
static atomic<bool> g_ai_paused(false);
static int g_ai_ctrl_port = 9998;
static thread g_ai_ctrl_thread;

static void aiSetPaused(bool paused, const char *from)
{
    if (g_ai_paused.exchange(paused) == paused)
        return;
    if (paused)
    {
        // 清掉缓存框：编码线程按 g_last_adas_valid 决定是否重绘，置 false 即无框
        lock_guard<mutex> lk(g_last_adas_mutex);
        g_last_adas_dets.clear();
        g_last_adas_valid = false;
    }
    cout << "[AI-CTRL] AI 识别" << (paused ? "暂停" : "恢复")
         << "（来自 " << from << "，画面" << (paused ? "无检测框" : "恢复检测框") << "）"
         << endl;
}

// UDP 控制线程：只做一件事——把 "AI 0/1" 翻译成 g_ai_paused
static void aiCtrlThread()
{
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        cerr << "[AI-CTRL] UDP socket 创建失败，AI 识别闸门不可用" << endl;
        return;
    }
    int on = 1;
    ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const char *)&on, sizeof(on));
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons((uint16_t)g_ai_ctrl_port);
    if (::bind(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0)
    {
        cerr << "[AI-CTRL] 绑定 UDP " << g_ai_ctrl_port << " 失败（端口被占用？），闸门不可用" << endl;
        ::close(fd);
        return;
    }
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 300000;                 // 300ms 超时，便于检查 g_running
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof(tv));
    cout << "[AI-CTRL] 监听 UDP " << g_ai_ctrl_port
         << "（\"AI 0\"=暂停识别 / \"AI 1\"=恢复识别）" << endl;

    char buf[64];
    while (g_running)
    {
        struct sockaddr_in peer;
        memset(&peer, 0, sizeof(peer));
        socklen_t plen = sizeof(peer);
        const ssize_t n = ::recvfrom(fd, buf, sizeof(buf) - 1, 0,
                                     (struct sockaddr *)&peer, &plen);
        if (n <= 0)
            continue;

        char ip[32] = "?";
        if (peer.sin_family == AF_INET)
            ::inet_ntop(AF_INET, &peer.sin_addr, ip, sizeof(ip));

        // 二进制单字节 0x00 / 0x01
        if (n == 1 && (buf[0] == 0 || buf[0] == 1))
        {
            aiSetPaused(buf[0] == 0, ip);
            continue;
        }

        string msg(buf, buf + n);
        while (!msg.empty() && (msg.back() == '\r' || msg.back() == '\n' || msg.back() == ' '))
            msg.pop_back();
        for (size_t k = 0; k < msg.size(); ++k)
            msg[k] = (char)toupper((unsigned char)msg[k]);

        if (msg == "AI 0" || msg == "AI0" || msg == "PAUSE")
            aiSetPaused(true, ip);
        else if (msg == "AI 1" || msg == "AI1" || msg == "RESUME")
            aiSetPaused(false, ip);
        // 其它内容一律忽略，不影响推流与串口
    }
    ::close(fd);
}

// ADAS 双上下文 worker：取「未被派发过的最新帧」-> 推理 -> 帧号单调发布。
// 语义与原推理线程一致：单槽覆写、不排队，两个 worker 都忙时新帧直接丢弃，
// 因此结果年龄上限恒为一次推理耗时，不会随运行时间累积。
static void adasDualWorker(int idx, bool publish_aligned_frame)
{
    // 该实例是否支持 NPU 零拷贝（模型加载成功即支持）
    while (g_running)
    {
        // AI 识别闸门：四路拼接 / 倒车辅助线时上位机下发暂停，此时不取帧不推理
        if (g_ai_paused.load())
        {
            this_thread::sleep_for(chrono::milliseconds(50));
            continue;
        }
        Mat frame;
        uint64_t fid = 0;
        {
            unique_lock<mutex> lock(g_input_mutex);
            // 等一个尚未派发的新帧
            g_input_cv.wait(lock, [&]
                { return !g_running || g_input_frame_id > g_adas_dispatched_id; });
            if (!g_running)
                break;

            // --infer-fps 是「启动到启动」的上限，两个 worker 共享同一节拍
            const int fps = max(1, g_target_infer_fps.load());
            auto now = chrono::steady_clock::now();
            if (now < g_adas_next_start)
            {
                g_input_cv.wait_until(lock, g_adas_next_start, [&]
                    { return !g_running; });
                if (!g_running)
                    break;
                now = chrono::steady_clock::now();
            }
            g_adas_next_start = now + chrono::microseconds(1000000 / fps);

            fid = g_input_frame_id;
            g_adas_dispatched_id = fid;      // 认领该帧，另一个 worker 不会重复取
            frame = g_input_frame;           // 只共享头，clone 放在锁外做
        }

        if (frame.empty())
            continue;

        Mat snapshot;
        try
        {
            snapshot = frame.clone();        // 推理用独立快照，采集侧可立即覆盖
        }
        catch (const cv::Exception &e)
        {
            cerr << "ADAS worker snapshot clone failed: " << e.what() << endl;
            continue;
        }
        if (snapshot.empty())
            continue;

        vector<AdasDetection> dets;
        const auto t0 = chrono::steady_clock::now();
        const bool ok = g_adas[idx].detect(snapshot, dets,
                                           g_adas_conf_thres.load(),
                                           g_adas_nms_thres.load());
        const auto t1 = chrono::steady_clock::now();
        if (!ok)
            continue;

        g_latency_infer_us = chrono::duration_cast<chrono::microseconds>(t1 - t0).count();
        g_infer_count++;

        {
            lock_guard<mutex> lk(g_adas_result_mutex);
            // 帧号单调保护：晚到的旧结果直接丢弃，绝不回退信号/画面
            if (fid <= g_adas_result_id)
                continue;
            // 推理途中被通知暂停：这一帧结果直接丢弃。否则 aiSetPaused 刚清空的
            // 检测框会被重新写回快照，编码线程就会一直把暂停前的框画在画面上。
            if (g_ai_paused.load())
                continue;
            g_adas_result_id = fid;

            const AdasSignals sig = mapAdasSignals(dets, snapshot.cols);
            {
                lock_guard<mutex> lk2(g_adas_mutex);
                g_adas_signals = sig;
            }
            g_adas_serial.publish(sig);

            {
                lock_guard<mutex> lk3(g_last_adas_mutex);
                g_last_adas_dets = dets;
                g_last_adas_sig = sig;
                g_last_adas_frame_id = fid;
                g_last_adas_valid = true;
            }

            // --latest-frame off 时才发布「框与画面同帧」的对齐帧
            if (publish_aligned_frame && !g_send_latest_frame.load())
            {
                drawAdasResults(snapshot, dets, sig);
                {
                    lock_guard<mutex> lk4(g_aligned_output_mutex);
                    g_aligned_output_frame = std::move(snapshot);
                    g_aligned_output_frame_id = fid;
                }
                g_aligned_output_cv.notify_one();
            }
        }
    }
}

static void inferenceThread(PlateRecognizer *recognizer,
                            bool enable_infer,
                            bool publish_aligned_frame,
                            bool draw_video_text)
{
    uint64_t last_id = 0;
    auto next_infer_time = chrono::steady_clock::now() - chrono::seconds(1);
    auto last_latency_log_time = chrono::steady_clock::now();
    double latency_sum_ms = 0.0;
    double latency_max_ms = 0.0;
    int latency_count = 0;

    while (g_running)
    {
        // AI 识别闸门（与 adasDualWorker 一致）：暂停时既不推理也不发布
        if (g_ai_paused.load() && g_adas[0].enabled())
        {
            this_thread::sleep_for(chrono::milliseconds(50));
            continue;
        }

        int target_fps = max(1, g_target_infer_fps.load());
        auto min_interval = chrono::microseconds(1000000 / target_fps);
        Mat published_frame;
        uint64_t current_id = 0;

        {
            unique_lock<mutex> lock(g_input_mutex);
            // V5 latency opt: wait for a new frame.  If cooldown is still active
            // use a timed wait so we don't miss the cooldown expiry; otherwise
            // block until the next frame arrives.  When a frame arrives *after*
            // cooldown has passed we process it immediately — no artificial
            // cadence delay.
            auto now = chrono::steady_clock::now();
            if (now < next_infer_time)
            {
                // Cooldown active: wait until it expires, but wake early on any
                // new frame so we can check whether the timer has elapsed.
                g_input_cv.wait_until(lock, next_infer_time, [&]
                    { return !g_running || g_input_frame_id != last_id; });
            }
            else
            {
                // Cooldown passed: block until the next captured frame.
                g_input_cv.wait(lock, [&]
                    { return !g_running || g_input_frame_id != last_id; });
            }

            if (!g_running)
                break;

            // If we were woken by a new frame but cooldown hasn't expired yet,
            // skip this frame and go back to waiting.
            if (chrono::steady_clock::now() < next_infer_time)
            {
                last_id = g_input_frame_id;
                continue;
            }

            // Only retain the published allocation while holding the mutex.
            // The full snapshot copy is deliberately performed after unlock,
            // so PCIe conversion can publish the next frame immediately.
            published_frame = g_input_frame;
            current_id = g_input_frame_id;
        }

        if (published_frame.empty())
        {
            last_id = current_id;
            continue;
        }

        Mat frame;
        try
        {
            // Deep-copy the selected frame.  RKNN and ROI/OCR now read a fully
            // independent snapshot that PCIe publication can never replace or
            // modify while inference is running.
            frame = published_frame.clone();
        }
        catch (const cv::Exception &e)
        {
            cerr << "Inference snapshot clone failed: " << e.what() << endl;
        }

        if (frame.empty())
        {
            last_id = current_id;
            continue;
        }

        vector<CachedPlate> results;
        vector<string> plates;
        Mat aligned_frame;
        auto cycle_start = chrono::steady_clock::now();

        bool ok = true;
        if (enable_infer && g_adas[0].enabled())
        {
            // ---- ADAS 模式：12 类感知 -> 6 路布尔 -> 串口下发 ----
            const auto infer_t0 = chrono::steady_clock::now();
            cycle_start = infer_t0;

            vector<AdasDetection> adas_dets;
            ok = g_adas[0].detect(frame, adas_dets,
                               g_adas_conf_thres.load(), g_adas_nms_thres.load());
            const auto infer_t1 = chrono::steady_clock::now();
            g_latency_infer_us = chrono::duration_cast<chrono::microseconds>(infer_t1 - infer_t0).count();

            // 推理途中收到暂停：丢弃这帧结果（不更新信号、不糊回检测框）
            if (g_ai_paused.load() && g_adas[0].enabled())
                continue;

            AdasSignals sig = mapAdasSignals(adas_dets, frame.cols);
            {
                lock_guard<mutex> lk(g_adas_mutex);
                g_adas_signals = sig;
            }
            g_adas_serial.publish(sig);

            // V6 解耦：把这一帧的框与信号留成一份共享快照，供编码线程在
            // 「最新采集帧」上重绘。不改变任何识别逻辑，也不触发额外推理。
            const bool decouple = g_send_latest_frame.load();
            if (ok)
            {
                lock_guard<mutex> lk(g_last_adas_mutex);
                g_last_adas_dets = adas_dets;
                g_last_adas_sig = sig;
                g_last_adas_frame_id = current_id;
                g_last_adas_valid = true;
            }

            if (ok && !decouple && publish_aligned_frame)
            {
                drawAdasResults(frame, adas_dets, sig);
                aligned_frame = std::move(frame);
            }

            // 解耦时故意不发布对齐帧：编码线程改为直接取采集侧新帧，
            // 因此不必再等这 120ms 推理，画面不会因为推理慢而被拖住。
            if (ok && !decouple && !aligned_frame.empty())
            {
                {
                    lock_guard<mutex> lock(g_aligned_output_mutex);
                    g_aligned_output_frame = std::move(aligned_frame);
                    g_aligned_output_frame_id = current_id;
                }
                g_aligned_output_cv.notify_one();
            }
        }
        else if (enable_infer && recognizer)
        {
            const auto infer_t0 = chrono::steady_clock::now();
            cycle_start = infer_t0;
            auto t_wait = chrono::duration_cast<chrono::microseconds>(infer_t0 - cycle_start).count();
            if (t_wait > g_latency_infer_wait_us.load())
                g_latency_infer_wait_us = t_wait;
            ok = recognizer->inferFrame(frame, results, plates);
            const auto infer_t1 = chrono::steady_clock::now();
            g_latency_infer_us = chrono::duration_cast<chrono::microseconds>(infer_t1 - infer_t0).count();

            if (ok && publish_aligned_frame)
            {
                // V5 latency opt: draw results directly on the snapshot frame
                // (saves one 2.7 MB clone), then move ownership to aligned_frame.
                recognizer->drawCachedResults(
                    frame, results, draw_video_text);
                aligned_frame = std::move(frame);
            }

            if (ok && !aligned_frame.empty())
            {
                {
                    lock_guard<mutex> lock(g_aligned_output_mutex);
                    g_aligned_output_frame = std::move(aligned_frame);
                    g_aligned_output_frame_id = current_id;
                }
                // V5 latency opt: wake the encoder immediately so it can
                // encode the fresh result without waiting for the next fixed
                // output cadence tick.
                g_aligned_output_cv.notify_one();
            }

            double latency_ms = chrono::duration<double, milli>(infer_t1 - infer_t0).count();
            bool yolo_only = g_yolo_only.load();
            bool profile_yolo = g_profile_yolo.load();
            YoloProfile yolo_profile = recognizer->lastYoloProfile();
            double report_latency_ms = latency_ms;
            if (yolo_only && yolo_profile.total_ms > 0.0)
                report_latency_ms = yolo_profile.total_ms;

            latency_sum_ms += report_latency_ms;
            latency_max_ms = max(latency_max_ms, report_latency_ms);
            latency_count++;

            if (chrono::duration_cast<chrono::milliseconds>(infer_t1 - last_latency_log_time).count() >= 1000)
            {
                double avg_ms = latency_count > 0 ? latency_sum_ms / latency_count : 0.0;
                size_t det_count = yolo_profile.detections;
                ostringstream oss;
                oss << fixed << setprecision(2);
                if (yolo_only)
                {
                    oss << "YOLO-only latency: last=" << report_latency_ms
                        << " ms, avg=" << avg_ms
                        << " ms, max=" << latency_max_ms
                        << " ms, det=" << det_count;
                }
                else
                {
                    oss << "Model latency: last=" << report_latency_ms
                        << " ms, avg=" << avg_ms
                        << " ms, max=" << latency_max_ms
                        << " ms, detections=" << det_count;
                }
                if (!ok)
                    oss << ", status=failed";
                oss << " | max_conf=" << yolo_profile.max_confidence
                    << " candidates=" << yolo_profile.candidates_above_threshold
                    << " rows=" << yolo_profile.rows
                    << " invalid=" << yolo_profile.invalid_values;
                if (profile_yolo)
                {
                    const RknnProfile &rp = yolo_profile.rknn;
                    oss << " | yolo_total=" << yolo_profile.total_ms
                        << " runImage=" << rp.total_ms
                        << " pre=" << rp.preprocess_ms
                        << " input=" << rp.inputs_set_ms
                        << " run=" << rp.rknn_run_ms
                        << " out_get=" << rp.outputs_get_ms
                        << " out_copy=" << rp.output_copy_ms
                        << " post=" << yolo_profile.postprocess_ms;
                }
                oss << " | frame_delay=" << (g_frame_delay_us.load() / 1000) << "ms";
                cout << oss.str() << endl;

                latency_sum_ms = 0.0;
                latency_max_ms = 0.0;
                latency_count = 0;
                last_latency_log_time = infer_t1;
            }
        }

        // --infer-fps is a true start-to-start upper bound.  PCIe capture and
        // conversion continue independently while this thread runs RKNN.
        next_infer_time = cycle_start + min_interval;

        if (ok)
        {
            updatePlateStatus(plates, results);
            g_infer_count++;

            if (!g_yolo_only.load() && !plates.empty())
            {
                cout << "Plate:";
                for (const string &plate : plates)
                {
                    cout << " " << plate;
                }
                cout << endl;
            }
        }

        last_id = current_id;
    }
}

// JPEG+chunked UDP sender — mimics rk_sender.py protocol for low-latency testing.
// No bind() — just socket() + sendto(), same as the proven V3 Python sender.
static atomic<int> g_jpeg_udp_fd(-1);
static mutex g_jpeg_udp_mutex;
static struct sockaddr_in g_jpeg_udp_dest = {};

static bool jpegUdpInit(const string &dest_addr) {
    if (dest_addr.empty()) return false;

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { cerr << "[JPEG-UDP] socket failed: " << strerror(errno) << endl; return false; }

    int sndbuf = 4 * 1024 * 1024;
    setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    auto colon = dest_addr.rfind(':');
    if (colon == string::npos) {
        cerr << "[JPEG-UDP] bad dest format, expected IP:PORT" << endl;
        close(fd); return false;
    }
    string ip = dest_addr.substr(0, colon);
    int dport = atoi(dest_addr.substr(colon + 1).c_str());
    if (dport <= 0 || dport > 65535) {
        cerr << "[JPEG-UDP] invalid port: " << dport << endl;
        close(fd); return false;
    }

    struct sockaddr_in dest = {};
    dest.sin_family = AF_INET;
    dest.sin_port = htons((uint16_t)dport);
    if (inet_pton(AF_INET, ip.c_str(), &dest.sin_addr) != 1) {
        cerr << "[JPEG-UDP] invalid IP: " << ip << endl;
        close(fd); return false;
    }

    cout << "[JPEG-UDP] dest=" << ip << ":" << dport << endl;

    lock_guard<mutex> lock(g_jpeg_udp_mutex);
    int old = g_jpeg_udp_fd.exchange(fd);
    if (old >= 0) close(old);
    g_jpeg_udp_dest = dest;
    return true;
}

// ---------------------------------------------------------------------------
// JPEG-UDP 异步发送
//   原来 imencode(1280x720, q80) 是在编码线程里同步做的：一次整幅 JPEG 编码
//   再加约 130 个 1400 字节分片的 sendto，全部排在 H.264 编码之后，直接叠进
//   画面帧龄（实测 20~30ms）。这里改成「最新帧槽 + 独立发送线程」：
//   编码线程只做一次 Mat 浅拷贝（引用计数，不复制像素），发送线程在后台编码
//   与分片，跟不上就覆盖旧帧，绝不排队堆积。
//   识别不受任何影响——检测用的是板上的 BGR 原帧，与 JPEG 质量无关。
// ---------------------------------------------------------------------------
static mutex g_jpeg_pub_mutex;
static condition_variable g_jpeg_pub_cv;
static Mat g_jpeg_pub_frame;
static uint64_t g_jpeg_pub_fid = 0;
static atomic<uint64_t> g_jpeg_pub_seq(0);

static void publishJpegFrame(const Mat &frame, uint64_t fid)
{
    if (frame.empty() || g_jpeg_udp_fd.load() < 0)
        return;
    {
        lock_guard<mutex> lock(g_jpeg_pub_mutex);
        g_jpeg_pub_frame = frame;   // 浅拷贝：只增引用计数
        g_jpeg_pub_fid = fid;
        g_jpeg_pub_seq.fetch_add(1);
    }
    g_jpeg_pub_cv.notify_one();
}

static void jpegUdpSenderThread()
{
    uint64_t seen = 0;
    while (g_running)
    {
        Mat frame;
        uint64_t fid = 0;
        {
            unique_lock<mutex> lock(g_jpeg_pub_mutex);
            g_jpeg_pub_cv.wait_for(lock, chrono::milliseconds(200), [&] {
                return !g_running || g_jpeg_pub_seq.load() != seen;
            });
            if (!g_running)
                break;
            if (g_jpeg_pub_seq.load() == seen)
                continue;
            seen = g_jpeg_pub_seq.load();
            frame = g_jpeg_pub_frame;
            fid = g_jpeg_pub_fid;
        }
        if (frame.empty())
            continue;

        vector<uint8_t> jpeg_buf;
        vector<int> jpeg_params = {IMWRITE_JPEG_QUALITY, 80};
        if (!cv::imencode(".jpg", frame, jpeg_buf, jpeg_params))
            continue;

        // payload: [meta_len:4B BE][meta_json][jpeg_bytes]
        char meta[128];
        snprintf(meta, sizeof(meta),
                 R"({"boxes":[],"text":"","fps":%.1f,"frame":%llu})",
                 (double)g_target_output_fps.load(),
                 (unsigned long long)fid);
        uint32_t meta_len = strlen(meta);
        uint32_t meta_len_be = htonl(meta_len);

        size_t payload_size = 4 + meta_len + jpeg_buf.size();
        vector<uint8_t> payload(payload_size);
        memcpy(payload.data(), &meta_len_be, 4);
        memcpy(payload.data() + 4, meta, meta_len);
        memcpy(payload.data() + 4 + meta_len, jpeg_buf.data(), jpeg_buf.size());

        lock_guard<mutex> lock(g_jpeg_udp_mutex);
        int fd = g_jpeg_udp_fd.load();
        if (fd < 0 || g_jpeg_udp_dest.sin_port == 0)
            continue;

        // 分片发送：[frame_id:4B BE][total:2B BE][idx:2B BE][data]
        const size_t CHUNK_SIZE = 1400;
        uint16_t total_chunks = static_cast<uint16_t>(
            (payload_size + CHUNK_SIZE - 1) / CHUNK_SIZE);
        uint8_t buf[65536];
        uint32_t fid_be = htonl((uint32_t)fid);
        uint16_t tc_be = htons(total_chunks);
        for (uint16_t i = 0; i < total_chunks; i++)
        {
            size_t off = i * CHUNK_SIZE;
            size_t chunk_len = min(CHUNK_SIZE, payload_size - off);
            uint16_t ci_be = htons(i);
            memcpy(buf, &fid_be, 4);
            memcpy(buf + 4, &tc_be, 2);
            memcpy(buf + 6, &ci_be, 2);
            memcpy(buf + 8, payload.data() + off, chunk_len);
            sendto(fd, buf, 8 + chunk_len, MSG_NOSIGNAL,
                   (struct sockaddr *)&g_jpeg_udp_dest, sizeof(g_jpeg_udp_dest));
        }

        // 真实帧龄：采集时刻 -> JPEG 发出。这是上位机能看到的最短延迟下限，
        // 不含 PC 端解码与贴图。
        auto now_us = chrono::duration_cast<chrono::microseconds>(
            chrono::system_clock::now().time_since_epoch()).count();
        int64_t gen_us = g_cap_ts_ring[fid & 127].load();
        if (gen_us > 0)
            g_frame_delay_us = now_us - gen_us;
        static int jdiag = 0;
        if (++jdiag % 50 == 0)
            cout << "[JPEG-UDP] frame=" << fid
                 << " bytes=" << jpeg_buf.size()
                 << " chunks=" << total_chunks
                 << " frame_age_ms=" << (g_frame_delay_us.load() / 1000)
                 << endl;
    }
}

// ---------------------------------------------------------------------------
// 本地 HDMI 直出（--hdmi-out on）
//   传统链路：BGR -> H.264 编码 -> ZLMediaKit RTSP -> 网络回环 -> RTSP 收流
//             -> 解码 -> 色彩转换 -> kmssink
//   本链路  ：BGR -> DRM dumb buffer -> HDMI
// 端到端延迟从「GOP + 编码 + 网络 + 解码 + 转换」降到 1~2 帧。
// ---------------------------------------------------------------------------
static DrmRoiOutput g_display;
static atomic<bool> g_display_on(false);
static mutex g_display_mutex;
static Mat g_display_frame;
static uint64_t g_display_id = 0;
static condition_variable g_display_cv;

// 只保留最新一帧：显示侧跟不上时直接丢旧帧，绝不让队列堆积成延迟。
// Mat 赋值是引用计数浅拷贝，不会复制像素数据。
static void publishDisplayFrame(const Mat &frame)
{
    if (!g_display_on.load() || frame.empty())
        return;
    {
        lock_guard<mutex> lock(g_display_mutex);
        g_display_frame = frame;
        g_display_id++;
    }
    g_display_cv.notify_one();
}

static void displayThread()
{
    uint64_t seen = 0;
    while (g_running)
    {
        Mat frame;
        {
            unique_lock<mutex> lock(g_display_mutex);
            // 只等新帧；超时只是给 g_running 一个退出检查点
            if (!g_display_cv.wait_for(lock, chrono::milliseconds(200),
                                       [&] { return g_display_id != seen; }))
                continue;
            seen = g_display_id;
            frame = g_display_frame;
        }
        if (!frame.empty())
            g_display.presentDirect(frame);
    }
}

static void encoderThread(bool use_aligned_infer_frame,
                          MppH264Encoder *encoder,
                          ZlmStreamer *streamer)
{
    auto pts_origin = chrono::steady_clock::now();
    BgrToNv12Converter converter;
    vector<uint8_t> nv12;
    vector<H264Packet> packets;
    uint64_t converted_frame_id = UINT64_MAX;
    bool converted_frame_was_aligned = false;
    uint64_t last_encoded_frame_id = UINT64_MAX;
    // V6 解耦：最近一次已 clone+画框 的采集帧号与缓存画面。同一帧被按输出节拍
    // 重复编码时直接复用，避免每 40ms 都 clone 一次 2.8MB 并重绘一遍。
    uint64_t last_sent_input_id = UINT64_MAX;
    Mat cached_sent_frame;

    while (g_running)
    {
        auto t_enc_wait_start = chrono::steady_clock::now();
        int output_fps = max(1, g_target_output_fps.load());
        auto min_interval = chrono::microseconds(1000000 / output_fps);

        Mat frame;
        uint64_t source_frame_id = 0;
        bool frame_is_aligned = false;

        // V6 解耦只对 ADAS 生效：车牌模式没有「最近结果重绘」机制，必须走原来的
        // 对齐帧路径，否则画面上会丢失车牌框与车牌号文字。
        const bool send_latest = use_aligned_infer_frame &&
                                 g_send_latest_frame.load() && g_adas[0].enabled();

        if (use_aligned_infer_frame && !send_latest)
        {
            t_enc_wait_start = chrono::steady_clock::now();
            unique_lock<mutex> lock(g_aligned_output_mutex);
            // V5 latency opt: when a new aligned frame is already available
            // (inference just finished), grab it immediately — no cadence
            // delay.  Otherwise wait up to one output interval for the next
            // result, then re-encode whatever we have to keep the stream
            // smooth at the target output FPS.
            // On first iteration last_encoded_frame_id is UINT64_MAX so
            // we also wait for the very first frame to arrive.
            if (g_aligned_output_frame.empty() ||
                g_aligned_output_frame_id == last_encoded_frame_id)
            {
                g_aligned_output_cv.wait_for(lock, min_interval);
            }
            // else: new frame ready — proceed without waiting
            g_latency_encode_wait_us = chrono::duration_cast<chrono::microseconds>(
                chrono::steady_clock::now() - t_enc_wait_start).count();

            if (!g_running)
                break;

            if (!g_aligned_output_frame.empty())
            {
                // Shallow-copy an immutable published frame.  Reassignment of
                // the global Mat cannot invalidate this local reference.
                frame = g_aligned_output_frame;
                source_frame_id = g_aligned_output_frame_id;
                frame_is_aligned = true;
            }
        }

        // V6 解耦：直接发最新采集帧，用最近一次检测结果重绘。
        // 推理线程仍在独立地「取最新帧 -> 推理 -> 更新信号」，两者只共享一份结果快照。
        if (send_latest)
        {
            Mat src;
            uint64_t src_id = 0;
            {
                lock_guard<mutex> lock(g_input_mutex);
                if (!g_input_frame.empty())
                {
                    src = g_input_frame;      // 浅拷贝：锁内取一份稳定引用
                    src_id = g_input_frame_id;
                }
            }
            if (!src.empty())
            {
                if (src_id != last_sent_input_id)
                {
                    // 新采集帧：必须 clone —— 框画在副本上，不能污染共享的采集帧
                    frame = src.clone();
                    vector<AdasDetection> dets;
                    AdasSignals sig;
                    bool valid = false;
                    {
                        lock_guard<mutex> lk(g_last_adas_mutex);
                        dets = g_last_adas_dets;
                        sig = g_last_adas_sig;
                        // 暂停识别期间一律不画框（双保险：worker 已丢弃途中结果）
                        valid = g_last_adas_valid && !g_ai_paused.load();
                    }
                    if (valid)
                        drawAdasResults(frame, dets, sig);
                    last_sent_input_id = src_id;
                    cached_sent_frame = frame;
                    source_frame_id = src_id;
                }
                else if (!cached_sent_frame.empty() && !g_ai_paused.load())
                {
                    // 同一采集帧按输出节拍重复编码：复用已画好的副本
                    // 暂停识别时绝不复用——那份副本可能还带着暂停前的框
                    frame = cached_sent_frame;
                    source_frame_id = src_id;
                }
            }
        }

        // In --no-infer mode, output the latest clean PCIe frame.  In inference
        // mode, wait briefly for the first aligned result instead of first
        // showing a newer raw frame and then jumping backwards in time.
        if (frame.empty() && !use_aligned_infer_frame)
        {
            lock_guard<mutex> lock(g_input_mutex);
            if (!g_input_frame.empty())
            {
                frame = g_input_frame;
                source_frame_id = g_input_frame_id;
            }
        }

        if (!frame.empty())
        {
            // 本地 HDMI 直出：在 BGR->NV12 转换与 H.264 编码之前发布。
            // 此时 frame 已带检测框与 OCR 文字，且尚未被编码链路改动。
            publishDisplayFrame(frame);

            g_output_width = frame.cols;
            g_output_height = frame.rows;

            if (!encoder || !encoder->isReady() || !streamer || !streamer->isReady())
            {
                cerr << "Output encoder thread failed: MPP encoder or ZLMediaKit streamer is not ready." << endl;
                g_running = false;
                break;
            }

            if (frame.cols != encoder->width() || frame.rows != encoder->height())
            {
                cerr << "Output encoder thread failed: frame size " << frame.cols << "x" << frame.rows
                     << " does not match MPP encoder size "
                     << encoder->width() << "x" << encoder->height() << endl;
                g_running = false;
                break;
            }

            const bool need_nv12_convert =
                nv12.empty() ||
                source_frame_id != converted_frame_id ||
                frame_is_aligned != converted_frame_was_aligned;

            bool nv12_convert_ok = true;
            if (need_nv12_convert)
            {
                // Convert only once per new source-frame id.  Repeated output
                // frames reuse NV12, while PCIe remains free to capture.
                nv12_convert_ok = converter.convert(frame,
                                                    encoder->width(),
                                                    encoder->height(),
                                                    encoder->horStride(),
                                                    encoder->verStride(),
                                                    nv12);
            }

            if (!nv12_convert_ok)
            {
                cerr << "Output encoder thread failed: BGR to NV12 conversion failed." << endl;
                g_running = false;
                break;
            }

            if (need_nv12_convert)
            {
                converted_frame_id = source_frame_id;
                converted_frame_was_aligned = frame_is_aligned;
            }

            uint64_t pts_ms = static_cast<uint64_t>(
                chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now() - pts_origin).count());

            if (!encoder->encodeNv12(nv12.data(), nv12.size(), pts_ms, packets))
            {
                cerr << "Output encoder thread failed: MPP H.264 encode failed." << endl;
                g_running = false;
                break;
            }
            g_latency_encode_us = chrono::duration_cast<chrono::microseconds>(
                chrono::steady_clock::now() - t_enc_wait_start).count();

            // MPP pipeline delay: compare packet PTS with wall clock
            for (const auto &pkt : packets) {
                int64_t now_ms = chrono::duration_cast<chrono::milliseconds>(
                    chrono::steady_clock::now() - pts_origin).count();
                g_mpp_pts_delay_us = (now_ms - static_cast<int64_t>(pkt.pts_ms)) * 1000;
            }

            bool frame_sent = true;
            for (const H264Packet &packet : packets)
            {
                if (!streamer->inputH264(packet.data.data(),
                                         static_cast<int>(packet.data.size()),
                                         packet.pts_ms,
                                         packet.dts_ms))
                {
                    frame_sent = false;
                    break;
                }
                g_stream_count++;
            }

            if (!frame_sent)
            {
                cerr << "Output encoder thread failed: ZLMediaKit rejected H.264 packet." << endl;
                g_running = false;
                break;
            }

            // JPEG-UDP：只把「最新帧」投递给后台发送线程，本线程不做 JPEG 编码
            // 也不做分片 sendto —— 原来这两步同步压在这里，会直接叠进画面帧龄。
            publishJpegFrame(frame, source_frame_id);

            // 真实帧龄：这一帧画面从「采集」到「编码完成」过了多久。
            // 必须按 source_frame_id 回查采集时刻——旧代码用的是 g_frame_gen_us
            // （每收到新采集帧就刷新），而推流发的可能是 200ms 前的对齐帧，
            // 所以那个 delay_us 只反映「离最近采集过了多久」，不是帧龄。
            int64_t enc_age_us = 0;
            {
                auto now_sys_us = chrono::duration_cast<chrono::microseconds>(
                    chrono::system_clock::now().time_since_epoch()).count();
                int64_t gen_us = g_cap_ts_ring[source_frame_id & 127].load();
                if (gen_us > 0)
                    enc_age_us = now_sys_us - gen_us;
            }

            g_encode_count++;
            // Diagnostic: log every 50th frame
            static int diag_cnt = 0;
            if (++diag_cnt % 50 == 0) {
                cout << "[DIAG] enc_frame=" << diag_cnt
                     << " src_id=" << source_frame_id
                     << " last_enc_id=" << last_encoded_frame_id
                     << " enc_age_ms=" << (enc_age_us / 1000)
                     << endl;
            }
            last_encoded_frame_id = source_frame_id;
        }
    }
}

static string htmlEscape(const string &s)
{
    string out;
    out.reserve(s.size() + 16);
    for (char ch : s)
    {
        switch (ch)
        {
        case '&':
            out += "&amp;";
            break;
        case '<':
            out += "&lt;";
            break;
        case '>':
            out += "&gt;";
            break;
        case '"':
            out += "&quot;";
            break;
        case '\'':
            out += "&#39;";
            break;
        default:
            out += ch;
            break;
        }
    }
    return out;
}

static string buildIndexHtml()
{
    string webrtc = htmlEscape(g_webrtc_url);
    string http_flv = htmlEscape(g_http_flv_url);
    string rtsp = htmlEscape(g_rtsp_url);

    return string(R"HTML(
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="color-scheme" content="dark">
<title>RK3568 多车牌识别系统</title>
<style>
:root{
  --bg:#071017;--panel:#0d1c27;--line:rgba(132,177,201,.16);
  --text:#edf7fb;--muted:#7992a2;--accent:#48dbc9;--blue:#46a7ff;
  --ok:#5be59f;--warn:#ffbd66;--danger:#ff6f7d;--radius:14px;
}
*{box-sizing:border-box}
html,body{width:100%;height:100%;margin:0;overflow:hidden}
body{
  color:var(--text);
  background:
    radial-gradient(circle at 8% 0%,rgba(31,157,176,.14),transparent 30%),
    radial-gradient(circle at 90% 100%,rgba(36,102,180,.12),transparent 34%),
    linear-gradient(145deg,#071017,#08131b 46%,#061018);
  font-family:"Segoe UI","Microsoft YaHei","PingFang SC",Arial,sans-serif;
  -webkit-font-smoothing:antialiased;
}
button{font:inherit;color:inherit}
header{
  height:56px;padding:0 18px;display:flex;align-items:center;
  justify-content:space-between;gap:20px;border-bottom:1px solid var(--line);
  background:rgba(5,14,21,.86);
}
.brand{min-width:0;display:flex;align-items:center;gap:12px}
.brandMark{
  position:relative;width:32px;height:32px;flex:0 0 32px;
  border:1px solid rgba(77,224,209,.52);border-radius:10px;
  background:linear-gradient(135deg,rgba(72,219,201,.24),rgba(70,167,255,.08));
}
.brandMark:before,.brandMark:after{
  content:"";position:absolute;inset:8px;border:1.5px solid var(--accent);border-radius:4px;
}
.brandMark:after{inset:12px;border-color:var(--blue);border-radius:2px}
.brandCopy{min-width:0;display:flex;align-items:baseline;gap:12px}
.brandCopy h1{
  margin:0;overflow:hidden;font-size:17px;font-weight:680;letter-spacing:.02em;
  text-overflow:ellipsis;white-space:nowrap;
}
.brandCopy p{
  margin:0;color:var(--muted);font:10px Consolas,monospace;
  letter-spacing:.08em;white-space:nowrap;
}
.headerRight{display:flex;align-items:center;gap:10px}
.systemChip,.clock{
  height:30px;padding:0 10px;display:inline-flex;align-items:center;gap:7px;
  border:1px solid var(--line);border-radius:9px;background:rgba(14,29,40,.78);
  color:#a8bbc5;font-size:11px;
}
.statusDot{
  width:7px;height:7px;border-radius:50%;background:var(--ok);
  box-shadow:0 0 0 4px rgba(91,229,159,.08),0 0 12px rgba(91,229,159,.5);
}
.statusDot.offline{background:var(--danger);box-shadow:0 0 0 4px rgba(255,111,125,.08),0 0 12px rgba(255,111,125,.45)}
.clock{min-width:78px;justify-content:center;font-family:Consolas,monospace}
.wrap{
  height:calc(100vh - 56px);height:calc(100dvh - 56px);min-height:0;
  padding:12px 14px 14px;display:grid;grid-template-rows:minmax(0,1fr) 126px;gap:12px;
}
.workspace{
  min-height:0;display:grid;
  grid-template-columns:minmax(0,1.78fr) minmax(360px,.82fr);gap:12px;
}
.panel{
  min-width:0;min-height:0;overflow:hidden;border:1px solid var(--line);
  border-radius:var(--radius);
  background:linear-gradient(150deg,rgba(15,31,43,.97),rgba(9,21,30,.96));
  box-shadow:0 18px 48px rgba(0,0,0,.22);
}
.panelHead{
  height:46px;padding:0 14px;display:flex;align-items:center;
  justify-content:space-between;gap:12px;border-bottom:1px solid var(--line);
}
.panelTitle{min-width:0;display:flex;align-items:center;gap:9px}
.panelTitle:before{
  content:"";width:3px;height:15px;border-radius:3px;
  background:linear-gradient(180deg,var(--accent),var(--blue));
}
.panelTitle h2{margin:0;font-size:13px;font-weight:650;letter-spacing:.04em;white-space:nowrap}
.panelSub{color:var(--muted);font-size:10px;white-space:nowrap}
.player{display:flex;flex-direction:column}
.videoActions{display:flex;align-items:center;gap:8px}
.connectionState{display:flex;align-items:center;gap:6px;color:var(--ok);font-size:10px}
.connectionState:before{
  content:"";width:6px;height:6px;border-radius:50%;background:currentColor;box-shadow:0 0 9px currentColor;
}
.reconnect{
  height:27px;padding:0 10px;border:1px solid rgba(92,176,211,.24);
  border-radius:8px;background:rgba(20,42,56,.82);color:#b8cad3;font-size:10px;cursor:pointer;
}
.reconnect:disabled{opacity:.48;cursor:default}
.videoBox{position:relative;min-height:0;flex:1;overflow:hidden;background:#02070b}
.videoBox:before{
  content:"等待 WebRTC 视频流";position:absolute;left:50%;top:50%;
  transform:translate(-50%,-50%);color:rgba(180,211,220,.18);
  font:13px Consolas,monospace;letter-spacing:.18em;white-space:nowrap;
}
.videoBox video{
  position:absolute;inset:0;z-index:2;display:block;width:100%;height:100%;
  background:transparent;object-fit:contain;
}
.videoHud{position:absolute;z-index:3;left:12px;display:flex;gap:7px;pointer-events:none}
.videoHud.top{top:11px}.videoHud.bottom{bottom:11px}
.hudTag{
  min-height:24px;padding:0 8px;display:inline-flex;align-items:center;gap:6px;
  border:1px solid rgba(183,220,229,.14);border-radius:7px;
  background:rgba(3,12,17,.72);color:#c8d8df;font:9px Consolas,monospace;
}
.liveTag{color:#ffd3d7;border-color:rgba(255,111,125,.24)}
.liveTag:before{
  content:"";width:6px;height:6px;border-radius:50%;background:var(--danger);
  box-shadow:0 0 8px rgba(255,111,125,.7);
}
.side{min-height:0;display:grid;grid-template-rows:minmax(0,1fr) auto;gap:12px}
.platePanel{display:flex;flex-direction:column}
.plateCount{
  height:26px;min-width:62px;padding:0 9px;display:flex;align-items:center;
  justify-content:center;gap:5px;border:1px solid rgba(72,219,201,.24);
  border-radius:8px;background:rgba(72,219,201,.08);color:#9bf3e8;font-size:10px;
}
.plateCount strong{color:#e6fffb;font:700 13px Consolas,monospace}
.plateBody{min-height:0;padding:11px;display:flex;flex:1;flex-direction:column;gap:9px}
.plateSummary{display:flex;justify-content:space-between;gap:10px;color:var(--muted);font-size:10px}
.plateSummary strong{color:#c8d9df;font-weight:540}
.plateList{
  min-height:0;flex:1;display:grid;grid-template-columns:repeat(2,minmax(0,1fr));
  align-content:start;gap:8px;overflow-y:auto;scrollbar-color:rgba(80,139,159,.5) transparent;
}
.plateCard{
  --plate-color:var(--accent);min-width:0;height:64px;padding:9px 10px;
  display:grid;grid-template-columns:27px minmax(0,1fr);align-items:center;gap:8px;
  border:1px solid rgba(97,147,166,.16);border-left:3px solid var(--plate-color);
  border-radius:10px;background:linear-gradient(135deg,rgba(18,42,54,.86),rgba(9,25,34,.92));
}
.plateIndex{
  width:27px;height:27px;display:grid;place-items:center;border:1px solid rgba(72,219,201,.25);
  border-color:color-mix(in srgb,var(--plate-color) 35%,transparent);
  border-radius:8px;background:rgba(72,219,201,.06);color:var(--plate-color);font:9px Consolas,monospace;
}
.plateInfo{min-width:0}
.plateNumber{
  overflow:hidden;color:#effffd;font-size:clamp(13px,1.22vw,18px);
  font-weight:760;letter-spacing:.05em;text-overflow:ellipsis;white-space:nowrap;
}
.plateMeta{margin-top:4px;display:flex;align-items:center;gap:5px;color:var(--muted);font-size:8px}
.colorDot{width:5px;height:5px;border-radius:50%;background:var(--plate-color);box-shadow:0 0 6px var(--plate-color)}
.plateEmpty{
  grid-column:1/-1;min-height:118px;display:grid;place-items:center;
  border:1px dashed rgba(111,159,177,.18);border-radius:10px;color:var(--muted);
  font-size:11px;line-height:1.8;text-align:center;
}
.streamPanel .panelHead{height:38px}.streamPanel .panelTitle h2{font-size:12px}
.streamList{padding:8px 10px 10px;display:grid;gap:6px}
.streamRow{
  min-width:0;height:29px;display:grid;grid-template-columns:56px minmax(0,1fr) 27px;
  align-items:center;gap:7px;padding:0 6px 0 9px;border:1px solid rgba(102,151,169,.12);
  border-radius:8px;background:rgba(4,15,22,.6);
}
.streamKind{color:#91a7b4;font:8px Consolas,monospace;letter-spacing:.08em}
.streamUrl{
  min-width:0;overflow:hidden;color:#b7c9d2;font:8px Consolas,monospace;
  text-decoration:none;text-overflow:ellipsis;white-space:nowrap;
}
.copyBtn{
  width:25px;height:22px;padding:0;border:0;border-radius:6px;
  background:transparent;color:#7895a3;font-size:12px;cursor:pointer;
}
.copyBtn:hover{background:rgba(72,219,201,.08);color:var(--accent)}
.systemPanel{display:flex;flex-direction:column}
.systemPanel .panelHead{height:35px;flex:0 0 35px}.systemPanel .panelTitle h2{font-size:11px}
.metricNote{color:var(--muted);font:9px Consolas,monospace}
.metrics{
  min-height:0;padding:8px 10px 10px;display:grid;flex:1;
  grid-template-columns:repeat(8,minmax(0,1fr));gap:7px;
}
.metric{
  min-width:0;padding:8px 9px;display:flex;flex-direction:column;
  justify-content:center;gap:5px;overflow:hidden;border:1px solid rgba(105,153,170,.12);
  border-radius:9px;background:linear-gradient(145deg,rgba(17,38,50,.78),rgba(8,22,31,.8));
}
.metricK{overflow:hidden;color:#708a99;font-size:9px;text-overflow:ellipsis;white-space:nowrap}
.metricV{
  overflow:hidden;color:#e9f6fa;font:680 clamp(14px,1.25vw,19px) Consolas,monospace;
  letter-spacing:-.025em;text-overflow:ellipsis;white-space:nowrap;
}
.metricV small{margin-left:2px;color:#78909d;font-size:8px;font-weight:500}
.metric.accent .metricV{color:#9ff5eb}.metric.warn .metricV{color:var(--warn)}
.toast{
  position:fixed;z-index:20;right:18px;bottom:18px;padding:9px 12px;
  border:1px solid rgba(72,219,201,.28);border-radius:9px;background:rgba(9,27,35,.95);
  color:#baf8ef;font-size:10px;opacity:0;transform:translateY(8px);
  pointer-events:none;transition:.16s;
}
.toast.show{opacity:1;transform:translateY(0)}
@media(max-width:1180px){
  html,body{overflow:auto}.brandCopy p{display:none}
  .wrap{height:auto;min-height:calc(100vh - 56px);grid-template-rows:minmax(520px,calc(100vh - 210px)) auto}
  .workspace{grid-template-columns:minmax(0,1.55fr) minmax(330px,.8fr)}
  .metrics{grid-template-columns:repeat(4,minmax(0,1fr))}.systemPanel{min-height:188px}
}
@media(max-width:860px){
  .clock,.systemChip span{display:none}.systemChip{width:30px;padding:0;justify-content:center}
  .wrap{display:block;padding:10px}.workspace{grid-template-columns:1fr}
  .player{min-height:54vw}.platePanel{min-height:290px}.systemPanel{margin-top:12px}
}
</style>
</head>
<body>
<header>
  <div class="brand">
    <div class="brandMark" aria-hidden="true"></div>
    <div class="brandCopy">
      <h1>RK3568 多车牌识别系统</h1>
      <p>FPGA · PCIE DMA · RKNN · ZLMEDIAKIT</p>
    </div>
  </div>
  <div class="headerRight">
    <div class="systemChip"><i class="statusDot" id="statusDot"></i><span id="deviceState">RK3568 在线</span></div>
    <time class="clock" id="clock">--:--:--</time>
  </div>
</header>

<main class="wrap">
  <section class="workspace">
    <article class="panel player">
      <div class="panelHead">
        <div class="panelTitle">
          <h2>实时识别画面</h2>
          <span class="panelSub" id="videoResolution">等待视频</span>
        </div>
        <div class="videoActions">
          <span class="connectionState" id="playerState">等待连接</span>
          <button class="reconnect" id="reconnectBtn" type="button">↻&nbsp; 重连</button>
        </div>
      </div>
      <div class="videoBox">
        <video id="webrtcVideo" autoplay playsinline muted></video>
        <div class="videoHud top"><span class="hudTag liveTag">LIVE</span><span class="hudTag">CAM 01</span></div>
        <div class="videoHud bottom"><span class="hudTag" id="videoFpsHud">0.0 FPS</span><span class="hudTag">H.264</span></div>
      </div>
    </article>

    <aside class="side">
      <article class="panel platePanel">
        <div class="panelHead">
          <div class="panelTitle"><h2>多车牌识别结果</h2><span class="panelSub">当前推理帧</span></div>
          <div class="plateCount"><strong id="plateCount">0</strong> 个目标</div>
        </div>
        <div class="plateBody">
          <div class="plateSummary">
            <span>已稳定识别 <strong id="stableCount">0 个车牌</strong></span>
            <span id="plateUpdatedAt">等待识别</span>
          </div>
          <div class="plateList" id="plateList" aria-live="polite">
            <div class="plateEmpty">当前画面未识别到稳定车牌<br>识别结果将在此处自动更新</div>
          </div>
        </div>
      </article>

      <article class="panel streamPanel">
        <div class="panelHead">
          <div class="panelTitle"><h2>播放地址</h2></div>
          <span class="panelSub">ZLMediaKit</span>
        </div>
        <div class="streamList">
          <div class="streamRow">
            <span class="streamKind">WebRTC</span>
            <a class="streamUrl" id="webrtcLink" data-url=")HTML") +
           webrtc + R"HTML(" href=")HTML" + webrtc + R"HTML(" target="_blank" rel="noreferrer">正在解析地址</a>
            <button class="copyBtn" type="button" data-copy-target="webrtcLink" title="复制WebRTC地址">⧉</button>
          </div>
          <div class="streamRow">
            <span class="streamKind">HTTP-FLV</span>
            <a class="streamUrl" id="httpFlvLink" data-url=")HTML" +
           http_flv + R"HTML(" href=")HTML" + http_flv + R"HTML(" target="_blank" rel="noreferrer">正在解析地址</a>
            <button class="copyBtn" type="button" data-copy-target="httpFlvLink" title="复制HTTP-FLV地址">⧉</button>
          </div>
          <div class="streamRow">
            <span class="streamKind">RTSP</span>
            <span class="streamUrl" id="rtspLink" data-url=")HTML" +
           rtsp + R"HTML(">正在解析地址</span>
            <button class="copyBtn" type="button" data-copy-target="rtspLink" title="复制RTSP地址">⧉</button>
          </div>
        </div>
      </article>
    </aside>
  </section>

  <section class="panel systemPanel">
    <div class="panelHead">
      <div class="panelTitle"><h2>系统运行状态</h2></div>
      <span class="metricNote">STATUS REFRESH · 500 ms</span>
    </div>
    <div class="metrics">
      <div class="metric"><span class="metricK">采集输入分辨率</span><span class="metricV" id="serverInRes">—</span></div>
      <div class="metric"><span class="metricK">H.264 输出分辨率</span><span class="metricV" id="serverOutRes">—</span></div>
      <div class="metric accent"><span class="metricK">PCIe 采集帧率</span><span class="metricV" id="captureFps">0.0<small>FPS</small></span></div>
      <div class="metric"><span class="metricK">RKNN 推理帧率</span><span class="metricV" id="inferFps">0.0<small>FPS</small></span></div>
      <div class="metric"><span class="metricK">H.264 输入帧率</span><span class="metricV" id="encodeFps">0.0<small>FPS</small></span></div>
      <div class="metric"><span class="metricK">ZLM 包输入速率</span><span class="metricV" id="streamFps">0.0<small>FPS</small></span></div>
      <div class="metric warn"><span class="metricK">采集丢帧 / 失败</span><span class="metricV" id="dropCount">0</span></div>
    </div>
  </section>
</main>

<div class="toast" id="toast" role="status">播放地址已复制</div>

<script>
const webrtcVideo=document.getElementById('webrtcVideo');
const playerState=document.getElementById('playerState');
const reconnectBtn=document.getElementById('reconnectBtn');
const webrtcLink=document.getElementById('webrtcLink');
const httpFlvLink=document.getElementById('httpFlvLink');
const rtspLink=document.getElementById('rtspLink');
const clock=document.getElementById('clock');
const statusDot=document.getElementById('statusDot');
const deviceState=document.getElementById('deviceState');
const videoResolution=document.getElementById('videoResolution');
const videoFpsHud=document.getElementById('videoFpsHud');
const plateList=document.getElementById('plateList');
const plateCount=document.getElementById('plateCount');
const stableCount=document.getElementById('stableCount');
const plateUpdatedAt=document.getElementById('plateUpdatedAt');
const serverInRes=document.getElementById('serverInRes');
const serverOutRes=document.getElementById('serverOutRes');

const captureFps=document.getElementById('captureFps');
const inferFps=document.getElementById('inferFps');
const encodeFps=document.getElementById('encodeFps');
const streamFps=document.getElementById('streamFps');
const dropCount=document.getElementById('dropCount');
const toast=document.getElementById('toast');

let lastStatus=null;
let lastStatusTs=0;
let lastPlateSignature='';
let peerConnection=null;
let reconnectTimer=null;
let toastTimer=null;

function updateClock(){
  clock.textContent=new Intl.DateTimeFormat('zh-CN',{
    hour:'2-digit',minute:'2-digit',second:'2-digit',hour12:false
  }).format(new Date());
}

function setDeviceOnline(online){
  statusDot.classList.toggle('offline',!online);
  deviceState.textContent=online?'RK3568 在线':'状态接口离线';
}

function zlmHost(){
  const host=window.location.hostname||'127.0.0.1';
  return host.indexOf(':')>=0&&host[0]!=='['?'['+host+']':host;
}

function resolveUrl(template){
  return (template||'').split('<RK3568-IP>').join(zlmHost());
}

function refreshPlaybackUrls(){
  const webrtcUrl=resolveUrl(webrtcLink.dataset.url);
  const httpFlvUrl=resolveUrl(httpFlvLink.dataset.url);
  const rtspUrl=resolveUrl(rtspLink.dataset.url);
  webrtcLink.href=webrtcUrl;
  webrtcLink.textContent=webrtcUrl;
  httpFlvLink.href=httpFlvUrl;
  httpFlvLink.textContent=httpFlvUrl;
  rtspLink.textContent=rtspUrl;
  webrtcLink.dataset.resolvedUrl=webrtcUrl;
  httpFlvLink.dataset.resolvedUrl=httpFlvUrl;
  rtspLink.dataset.resolvedUrl=rtspUrl;
  return webrtcUrl;
}

function splitLegacyPlates(value){
  return String(value||'').split(/\s*\/\s*/).map(v=>v.trim()).filter(Boolean);
}

function normalizePlateDetails(status){
  if(Array.isArray(status.plate_details)&&status.plate_details.length){
    return status.plate_details
      .map(item=>({
        plate:String(item&&item.plate||'').trim(),
        color:String(item&&item.color||'').trim()
      }))
      .filter(item=>item.plate.length);
  }
  return splitLegacyPlates(status.plates).map(plate=>({plate,color:''}));
}

function plateCssColor(color){
  if(color==='黑色') return '#aeb7be';
  if(color==='蓝色') return '#46a7ff';
  if(color==='绿色') return '#5be59f';
  if(color==='白色') return '#f2f7f9';
  if(color==='黄色') return '#ffd15c';
  return '#48dbc9';
}

function renderPlateCards(status){
  const items=normalizePlateDetails(status);
  const signature=JSON.stringify(items);
  plateCount.textContent=String(items.length);
  stableCount.textContent=items.length+' 个车牌';

  if(signature!==lastPlateSignature){
    lastPlateSignature=signature;
    plateUpdatedAt.textContent=items.length
      ? new Date().toLocaleTimeString('zh-CN',{hour12:false})
      : '等待识别';
  }

  if(!items.length){
    const empty=document.createElement('div');
    empty.className='plateEmpty';
    empty.innerHTML='当前画面未识别到稳定车牌<br>识别结果将在此处自动更新';
    plateList.replaceChildren(empty);
    return;
  }

  const cards=items.map((item,index)=>{
    const card=document.createElement('div');
    card.className='plateCard';
    card.style.setProperty('--plate-color',plateCssColor(item.color));

    const order=document.createElement('span');
    order.className='plateIndex';
    order.textContent=String(index+1).padStart(2,'0');

    const info=document.createElement('div');
    info.className='plateInfo';

    const number=document.createElement('div');
    number.className='plateNumber';
    number.textContent=item.plate;

    const meta=document.createElement('div');
    meta.className='plateMeta';
    const dot=document.createElement('i');
    dot.className='colorDot';
    const color=document.createElement('span');
    color.textContent=(item.color||'颜色识别中')+' · 稳定识别';
    meta.append(dot,color);
    info.append(number,meta);
    card.append(order,info);
    return card;
  });
  plateList.replaceChildren(...cards);
}

function metricHtml(value,unit,digits){
  const number=Number(value);
  const safe=Number.isFinite(number)?number.toFixed(digits):'0.0';
  return safe+'<small>'+unit+'</small>';
}

function closeWebRtc(){
  if(reconnectTimer){
    clearTimeout(reconnectTimer);
    reconnectTimer=null;
  }
  if(peerConnection){
    peerConnection.ontrack=null;
    peerConnection.onconnectionstatechange=null;
    peerConnection.close();
    peerConnection=null;
  }
  webrtcVideo.srcObject=null;
}

function waitIceGatheringComplete(pc){
  if(pc.iceGatheringState==='complete') return Promise.resolve();
  return new Promise(resolve=>{
    const timer=setTimeout(resolve,1500);
    pc.addEventListener('icegatheringstatechange',()=>{
      if(pc.iceGatheringState==='complete'){
        clearTimeout(timer);
        resolve();
      }
    });
  });
}

async function startWebRtc(){
  const webrtcUrl=refreshPlaybackUrls();
  closeWebRtc();
  if(!window.RTCPeerConnection){
    playerState.textContent='浏览器不支持WebRTC';
    return;
  }

  reconnectBtn.disabled=true;
  playerState.textContent='连接中';

  try{
    const pc=new RTCPeerConnection();
    peerConnection=pc;

    pc.addTransceiver('video',{direction:'recvonly'});
    pc.ontrack=(event)=>{
      if(event.streams&&event.streams[0]){
        webrtcVideo.srcObject=event.streams[0];
      }else{
        const stream=new MediaStream();
        stream.addTrack(event.track);
        webrtcVideo.srcObject=stream;
      }
      playerState.textContent='已连接';
      webrtcVideo.play().catch(()=>{});
    };
    pc.onconnectionstatechange=()=>{
      if(pc!==peerConnection) return;
      if(pc.connectionState==='connected'){
        playerState.textContent='已连接';
      }else if(pc.connectionState==='failed'||pc.connectionState==='disconnected'){
        playerState.textContent='连接断开';
        reconnectTimer=setTimeout(startWebRtc,2000);
      }else if(pc.connectionState){
        playerState.textContent=pc.connectionState;
      }
    };

    const offer=await pc.createOffer();
    await pc.setLocalDescription(offer);
    await waitIceGatheringComplete(pc);

    const r=await fetch(webrtcUrl,{
      method:'POST',
      headers:{'Content-Type':'text/plain;charset=utf-8'},
      body:pc.localDescription.sdp,
      cache:'no-store'
    });
    if(!r.ok) throw new Error('HTTP '+r.status);
    const answer=await r.json();
    if(answer.code&&answer.code!==0) throw new Error(answer.msg||('code '+answer.code));
    if(!answer.sdp) throw new Error('empty answer sdp');
    await pc.setRemoteDescription({type:answer.type||'answer',sdp:answer.sdp});
  }catch(e){
    closeWebRtc();
    playerState.textContent='连接失败';
  }finally{
    reconnectBtn.disabled=false;
  }
}

reconnectBtn.addEventListener('click',startWebRtc);

async function pollStatus(){
  try{
    const r=await fetch('/status',{cache:'no-store'});
    if(!r.ok) throw new Error('HTTP '+r.status);
    const s=await r.json();
    const now=performance.now();

    if(lastStatus){
      const dt=(now-lastStatusTs)/1000;
      if(dt>0){
        const captureRate=Math.max(0,(s.capture_count-lastStatus.capture_count)/dt);
        const inferRate=Math.max(0,(s.infer_count-lastStatus.infer_count)/dt);
        const encodeRate=Math.max(0,(s.encode_count-lastStatus.encode_count)/dt);
        const streamRate=Math.max(0,(s.stream_count-lastStatus.stream_count)/dt);
        captureFps.innerHTML=metricHtml(captureRate,'FPS',1);
        inferFps.innerHTML=metricHtml(inferRate,'FPS',1);
        encodeFps.innerHTML=metricHtml(encodeRate,'FPS',1);
        streamFps.innerHTML=metricHtml(streamRate,'FPS',1);
        videoFpsHud.textContent=captureRate.toFixed(1)+' FPS';
      }
    }

    lastStatus=s;
    lastStatusTs=now;

    const inputRes=(s.input_width&&s.input_height)?(s.input_width+'×'+s.input_height):'—';
    const outputRes=(s.output_width&&s.output_height)?(s.output_width+'×'+s.output_height):'—';
    renderPlateCards(s);
    serverInRes.textContent=inputRes;
    serverOutRes.textContent=outputRes;
    videoResolution.textContent=inputRes;

    dropCount.textContent=String(s.drop_count||0);
    setDeviceOnline(true);
  }catch(e){
    setDeviceOnline(false);
  }
}

function showToast(message){
  toast.textContent=message;
  toast.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer=setTimeout(()=>toast.classList.remove('show'),1400);
}

document.querySelectorAll('[data-copy-target]').forEach(button=>{
  button.addEventListener('click',async()=>{
    const target=document.getElementById(button.dataset.copyTarget);
    const value=target.dataset.resolvedUrl||target.textContent.trim();
    try{
      await navigator.clipboard.writeText(value);
      showToast('播放地址已复制');
    }catch(e){
      const input=document.createElement('textarea');
      input.value=value;
      document.body.appendChild(input);
      input.select();
      const copied=document.execCommand('copy');
      input.remove();
      showToast(copied?'播放地址已复制':'浏览器未允许复制');
    }
  });
});

updateClock();
setInterval(updateClock,1000);
setInterval(pollStatus,500);
pollStatus();
refreshPlaybackUrls();
startWebRtc();
window.addEventListener('beforeunload',closeWebRtc);
</script>
</body>
</html>
)HTML";
}

static void handleIndexClient(int client_fd)
{
    string html = buildIndexHtml();
    string header =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: text/html; charset=utf-8\r\n"
        "Cache-Control: no-cache, no-store, must-revalidate\r\n"
        "Content-Length: " +
        to_string(html.size()) + "\r\n\r\n";

    sendText(client_fd, header);
    sendText(client_fd, html);
}

static void handleStatusClient(int client_fd)
{
    string plates;
    vector<PlateStatusItem> plate_details;
    {
        lock_guard<mutex> lock(g_status_mutex);
        plates = g_latest_plate_text;
        plate_details = g_latest_plate_details;
    }

    ostringstream details_json;
    details_json << "[";
    for (size_t i = 0; i < plate_details.size(); ++i)
    {
        if (i)
            details_json << ",";
        details_json << "{\"plate\":\"" << jsonEscape(plate_details[i].plate)
                     << "\",\"color\":\"" << jsonEscape(plate_details[i].color)
                     << "\"}";
    }
    details_json << "]";

    ostringstream body_stream;
    body_stream << "{"
                << "\"capture_count\":" << g_capture_count.load() << ","
                << "\"infer_count\":" << g_infer_count.load() << ","
                << "\"stream_count\":" << g_stream_count.load() << ","
                << "\"encode_count\":" << g_encode_count.load() << ","
                << "\"drop_count\":" << g_drop_count.load() << ","
                << "\"target_infer_fps\":" << g_target_infer_fps.load() << ","
                << "\"input_width\":" << g_input_width.load() << ","
                << "\"input_height\":" << g_input_height.load() << ","
                << "\"output_width\":" << g_output_width.load() << ","
                << "\"output_height\":" << g_output_height.load() << ","
                << "\"plates\":\"" << jsonEscape(plates) << "\","
                << "\"plate_details\":" << details_json.str()
                << "}";
    const string body = body_stream.str();

    string header =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: application/json; charset=utf-8\r\n"
        "Cache-Control: no-store\r\n"
        "Content-Length: " +
        to_string(body.size()) + "\r\n\r\n";

    sendText(client_fd, header);
    sendText(client_fd, body);
}

static void handleStreamClient(int client_fd)
{
    sendHttpTextResponse(
        client_fd,
        410,
        "Gone",
        "MJPEG stream has been removed. Please use ZLMediaKit stream URL.");
}

static void handleHttpClient(int client_fd)
{
    setSocketLowLatency(client_fd);

    string header;
    if (!recvHttpHeader(client_fd, header))
    {
        close(client_fd);
        return;
    }

    if (header.rfind("GET / ", 0) == 0 || header.rfind("GET /index.html ", 0) == 0)
    {
        handleIndexClient(client_fd);
    }
    else if (header.rfind("GET /stream.mjpg ", 0) == 0)
    {
        handleStreamClient(client_fd);
    }
    else if (header.rfind("GET /status ", 0) == 0)
    {
        handleStatusClient(client_fd);
    }
    else
    {
        sendHttpStatus(client_fd, 404, "Not Found");
    }

    close(client_fd);
}

static void httpServerThread(int port)
{
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0)
    {
        cerr << "socket failed: " << strerror(errno) << endl;
        g_running = false;
        return;
    }

    g_server_fd = fd;

    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(static_cast<uint16_t>(port));

    if (bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0)
    {
        cerr << "bind failed: " << strerror(errno) << endl;
        close(fd);
        g_server_fd = -1;
        g_running = false;
        return;
    }

    if (listen(fd, 16) != 0)
    {
        cerr << "listen failed: " << strerror(errno) << endl;
        close(fd);
        g_server_fd = -1;
        g_running = false;
        return;
    }

    cout << "Web server started: http://<RK3568-IP>:" << port << "/" << endl;
    cout << "/stream.mjpg removed; use ZLMediaKit playback URLs on the index page." << endl;

    while (g_running)
    {
        sockaddr_in client_addr{};
        socklen_t len = sizeof(client_addr);

        int client_fd = accept(fd, reinterpret_cast<sockaddr *>(&client_addr), &len);
        if (client_fd < 0)
        {
            if (!g_running)
                break;
            if (errno == EINTR)
                continue;
            cerr << "accept failed: " << strerror(errno) << endl;
            continue;
        }

        thread(handleHttpClient, client_fd).detach();
    }

    if (fd >= 0)
    {
        close(fd);
    }

    g_server_fd = -1;
}

static void printHelp(const char *prog)
{
    cout << "Usage: " << prog << " [options]\n\n"
         << "Input chain:\n"
         << "  PCIe RGB565_LE frame -> RK YOLO plate detect -> bbox crop BGR -> OCR backend(lprnet 128x48 or plate_color 168x48) -> MPP H.264 -> ZLMediaKit\n\n"
         << "Model options:\n"
         << "  --detect-model PATH    YOLO plate detector RKNN, default ./model/yolov5n_i8.rknn\n"
         << "  --recog-model PATH     OCR RKNN model path; default depends on --ocr-backend\n"
         << "  --ocr-backend NAME     OCR backend: lprnet or plate_color, default lprnet\n"
         << "  --recog-chars PATH     LPRNet chars file, only used when --ocr-backend lprnet\n"
         << "  --font PATH            Chinese font, default ./font/SimHei.ttf\n\n"
         << "ADAS options (10-class perception, overrides plate/OCR pipeline):\n"
         << "  --adas-model PATH      ADAS 10-class RKNN model; non-empty enables ADAS mode\n"
         << "  --adas-conf FLOAT      ADAS confidence threshold, default 0.25\n"
         << "  --adas-nms FLOAT       ADAS NMS IoU threshold, default 0.45\n"
         << "  --adas-serial-dev PATH Serial device for rule-decision uplink, default /dev/ttyS3\n"
         << "  --adas-serial-baud N   Serial baudrate, default 115200 (8-N-1, no flow control)\n"
         << "  --adas-serial-period N Uplink period in ms, default 50 (20 Hz)\n"
         << "  --ai-ctrl-port N       UDP port of the AI pause gate, default 9998 (0=off).\n"
         << "                         Recv \"AI 0\" / \"AI 1\" (or byte 0x00/0x01) from the\n"
         << "                         PC: 0 pauses ADAS inference (no boxes, no uplink\n"
         << "                         update) for stitched-quad view / reverse guide\n"
         << "                         lines; 1 resumes. Same port in v8_start.py.\n"
         << "  --adas-workers N       ADAS inference workers, default 1 (single context).\n"
         << "                         2 = dual context: same model loaded twice, two\n"
         << "                         threads alternate frames (frame-id monotonic,\n"
         << "                         newest-frame-only, no queueing).\n"
         << "  --latest-frame on|off  ADAS decoupling, default on: stream the newest\n"
         << "                         captured frame at full FPS (~40ms age) instead of\n"
         << "                         waiting for inference (~250ms age); boxes lag ~1\n"
         << "                         inference (~120ms). off = old aligned-frame path.\n"
         << "  Protocol: \"<6 bits> <ped_count>\\n\", e.g. \"111110 3\"\n"
         << "    bit0 left_line_solid  bit1 right_line_solid  bit2 straight_green\n"
         << "    bit3 left_green       bit4 right_green       bit5 crosswalk_ahead\n\n"
         << "PCIe capture options:\n"
         << "  --pcie-device PATH     PCIe driver node, default /dev/pango_pci_driver\n"
         << "  --pcie-mem PATH        Memory device for BAR0 mmap, default /dev/mem\n"
         << "  --width N              PCIe frame width, default 1280\n"
         << "  --height N             PCIe frame height, default 720\n"
         << "  --fps N                Limit PCIe DMA read FPS, default 25\n"
         << "  --quiet-pcie           Suppress nonfatal PCIe read/FPS logs\n\n"
         << "Web / inference options:\n"
         << "  --port N               HTTP port, default 8080\n"
         << "  --infer-fps N          Limit RKNN inference FPS, default 10\n"
         << "  --output-fps N         Limit H.264 output FPS, default 25\n"
         << "  --conf F               Detection threshold, default 0.30 (values below 0.30 are clamped)\n"
         << "  --nms F                NMS threshold, default 0.45\n"
         << "  --yolo-only            Run YOLO detector only; skip OCR, FPGA ROI, and plate voting\n"
         << "  --profile-yolo         Print YOLO RKNN/preprocess/postprocess timing once per second\n"
         << "  --no-infer             Disable RKNN inference; stream captured PCIe video directly through ZLM\n"
         << "  --no-stream            Disable ZLM/MPP/HTTP output for detector-only performance testing\n"
         << "  --no-video-text        Do not draw plate text on video; show text only in web status\n"
         << "  --ocr-pad-x F          Optional expanded OCR horizontal padding ratio, default 0.04\n"
         << "  --ocr-pad-y F          Optional expanded OCR vertical padding ratio, default 0.10\n"
         << "  --expand-ocr-roi       Use expanded ROI for OCR test; default off, Python-style bbox crop\n"
         << "  --ocr-vote-window N    OCR temporal vote window, default 5\n"
         << "  --ocr-min-votes N      Min votes before /status stable update, default 2\n"
         << "  --debug-ocr            Save OCR debug every 100 inference frames; default off\n"
         << "  --debug-ocr-dir PATH   OCR debug dir, default /tmp/ocr_debug when --debug-ocr is on\n"
         << "  --obb-jitter-log PATH  dump raw per-frame OBB geometry for jitter measurement\n"
         << "\nZLMediaKit / MPP output options:\n"
         << "  --zlm-sdk-dir PATH     ZLMediaKit C API SDK dir, default ./ZLMediaKit\n"
         << "  --zlm-config PATH      ZLMediaKit config.ini, default ./ZLMediaKit/config.ini\n"
         << "  --zlm-vhost NAME       ZLM vhost, default __defaultVhost__\n"
         << "  --zlm-app NAME         ZLM app, default live\n"
         << "  --zlm-stream NAME      ZLM stream, default plate\n"
         << "  --zlm-http-port N      ZLM HTTP/WebRTC signaling port, default 8081\n"
         << "  --zlm-rtsp-port N      ZLM RTSP port, default 8554\n"
         << "  --zlm-rtmp-port N      ZLM RTMP port, default 1935\n"
         << "  --zlm-rtc-port N       ZLM RTC media port, default 8000\n"
         << "  --zlm-bitrate N        H.264 bitrate in bps, default 3000000\n"
         << "  --zlm-gop N            H.264 GOP, default 10 (use 1 for low latency)\n"
         << "\nLocal HDMI display options:\n"
         << "  --hdmi-out on|off      Direct DRM output of inference frames, default off\n"
         << "                         Bypasses H.264 encode + RTSP + decode entirely.\n"
         << "                         Conflicts with --fpga-mode hdmi (same CRTC).\n"
         << "  --drm-card PATH        DRM card, default /dev/dri/card0\n"
         << "  --drm-connector N      DRM connector id, 0 = first connected\n"
         << "  --drm-width N          display mode width, default 1280\n"
         << "  --drm-height N         display mode height, default 720\n"
         << "                         Match the frame size (1280x720) to skip scaling.\n"
         << "  --zlm-profile NAME     H.264 profile baseline/main/high, default baseline\n"
         << "  --mpp-format FORMAT    MPP input pixel format, default nv12\n"
         << "\nFPGA ROI options:\n"
         << "  --fpga-mode MODE       sw-edge(default), bypass, file, hdmi\n"
         << "                         sw-edge: software Sobel simulates FPGA output\n"
         << "                         bypass : send original ROI directly to OCR backend for debug\n"
         << "                         file   : write ROI image and wait for FPGA bridge output image\n"
         << "                         hdmi   : DRM HDMI outputs ROI to FPGA; second capture card receives FPGA edge output\n"
         << "  --fpga-dir PATH        file mode exchange dir, default /tmp/rk_fpga_roi\n"
         << "  --fpga-timeout N       FPGA return timeout in ms, default 30\n"
         << "  --fpga-video DEV       FPGA HDMI OUT return capture card, default /dev/video2\n"
         << "  --fpga-width N         FPGA return capture width, default 1280\n"
         << "  --fpga-height N        FPGA return capture height, default 720\n"
         << "  --fpga-fps N           FPGA return capture FPS, default 30\n"
         << "  --fpga-fourcc XXXX     FPGA return capture FOURCC, default MJPG\n"
         << "  --fpga-settle-ms N     delay after DRM ROI output before grabbing FPGA frame, default 20\n"
         << "  --drm-card PATH        DRM output card for ROI HDMI, default /dev/dri/card0\n"
         << "  --drm-connector ID     DRM connector id, default 0(auto). Check with modetest\n"
         << "  --drm-width N          DRM HDMI mode width, default 1280\n"
         << "  --drm-height N         DRM HDMI mode height, default 720\n"
         << "  --test-pattern         Generate test pattern internally (bypass FPGA/PCIe)\n"
         << "  --jpeg-udp-dest IP:PORT  Send JPEG frames via chunked UDP (e.g. 192.168.137.1:9999)\n"
         << "  -h, --help             Show this help\n\n"
         << "Example:\n"
         << "  " << prog << " --pcie-device /dev/pango_pci_driver --width 1280 --height 720 \\\n"
         << "      --detect-model ./model/yolov5n_i8.rknn \\\n"
         << "      --ocr-backend lprnet --recog-model ./model/lprnet_color_fp16.rknn\n";
}

static void testPatternThread(int width, int height, int fps) {
    auto t0 = chrono::steady_clock::now();
    const auto interval = chrono::microseconds(1000000 / max(1, fps));
    auto next_time = t0;
    uint64_t frame_count = 0;

    while (g_running) {
        auto now = chrono::steady_clock::now();
        if (now < next_time)
            this_thread::sleep_until(next_time);
        else if (now > next_time + interval * 2)
            next_time = now;
        next_time += interval;

        Mat frame(height, width, CV_8UC3);
        int bar_w = width / 4;
        for (int i = 0; i < 4; i++) {
            Scalar color = i == 0 ? Scalar(255,0,0) : i == 1 ? Scalar(0,255,0) :
                           i == 2 ? Scalar(0,0,255) : Scalar(255,255,255);
            rectangle(frame, Rect(i * bar_w, 0, bar_w, height), color, FILLED);
        }
        int x = (frame_count * 5) % (width - 100);
        rectangle(frame, Rect(x, height / 2 - 50, 100, 100), Scalar(0, 255, 255), FILLED);
        auto now_t = chrono::system_clock::now();
        auto now_s = chrono::duration_cast<chrono::seconds>(now_t.time_since_epoch()).count();
        auto now_ms = chrono::duration_cast<chrono::milliseconds>(now_t.time_since_epoch()).count() % 1000;
        time_t sec = (time_t)now_s;
        struct tm tm_buf;
        localtime_r(&sec, &tm_buf);
        char time_str[64];
        snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d.%03d",
                 tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec, (int)now_ms);
        string text = format("GEN_TIME=%s  frame=%d", time_str, (int)frame_count);
        // White outline + black fill for visibility
        putText(frame, text, Point(20, height - 30), FONT_HERSHEY_SIMPLEX,
                1.0, Scalar(255,255,255), 4);
        putText(frame, text, Point(20, height - 30), FONT_HERSHEY_SIMPLEX,
                1.0, Scalar(0,0,0), 2);

        updateInputFrame(frame);
        g_frame_gen_us = chrono::duration_cast<chrono::microseconds>(
            chrono::system_clock::now().time_since_epoch()).count();
        frame_count++;
    }
}

int main(int argc, char **argv)
{
    signal(SIGINT, handleSignal);
    signal(SIGTERM, handleSignal);
    signal(SIGPIPE, SIG_IGN);

    string video_dev = "/dev/video0";
    string pcie_device = "/dev/pango_pci_driver";
    string pcie_mem = "/dev/mem";
    int cap_width = 1280;
    int cap_height = 720;
    int cap_fps = 30;
    string fourcc = "MJPG";

    int port = DEFAULT_HTTP_PORT;
    string detect_model = "./model/yolov5n_i8.rknn";
    string recog_model = "./model/lprnet_color_fp16.rknn";
    bool recog_model_user_set = false;
    string ocr_backend_name = "lprnet";
    OcrBackend ocr_backend = OcrBackend::LPRNET;
    string ocr_chars_path = "./model/lprnet_chars.txt";
    string font_path = "./model/simhei.ttf";
    bool enable_infer = true;
    bool enable_stream = true;
    bool test_pattern = false;
    string jpeg_udp_dest;  // JPEG+chunked UDP destination IP:port, e.g. "192.168.137.1:9999"
    int infer_fps = DEFAULT_INFER_FPS;
    int output_fps = DEFAULT_OUTPUT_FPS;
    bool draw_video_text = true;
    ZlmConfig zlm_cfg;
    bool zlm_config_explicit = false;
    int zlm_gop = 0;
    string zlm_profile = "baseline";
    string mpp_format = "nv12";
    float conf_thres = 0.30f;
    float nms_thres = 0.45f;
    float ocr_pad_x = 0.04f;
    float ocr_pad_y = 0.10f;
    int ocr_vote_window = 5;
    int ocr_min_votes = 2;
    bool expand_ocr_roi = false;
    bool debug_ocr = false;
    string debug_ocr_dir;
    string obb_jitter_log;
    bool yolo_only = false;
    bool profile_yolo = false;
    bool quiet_pcie = false;
    string fpga_mode = "sw-edge";
    string fpga_dir = "/tmp/rk_fpga_roi";
    int fpga_timeout_ms = 30;
    string fpga_video_dev = "/dev/video2";
    int fpga_width = 1280;
    int fpga_height = 720;
    int fpga_fps = 30;
    string fpga_fourcc = "MJPG";
    int fpga_settle_ms = 20;
    string drm_card = "/dev/dri/card0";
    uint32_t drm_connector_id = 0;
    int drm_width = 1280;
    int drm_height = 720;
    bool hdmi_out = false;

    // ---- ADAS 12 类感知（车道线/箭头灯/圆盘灯/行人/斑马线）+ 串口下发 ----
    bool adas_mode = false;
    string adas_model;                 // 非空即启用 ADAS 模式
    string adas_serial_dev = "/dev/ttyS3";
    int adas_serial_baud = 115200;
    int adas_serial_period_ms = 50;    // 20 Hz 下发
    float adas_conf_thres = 0.25f;
    float adas_nms_thres = 0.45f;
    int adas_workers = 1;              // 双上下文 worker 数（默认 1 = 旧行为）
    int ai_ctrl_port = 9998;           // AI 识别闸门 UDP 控制口（0 = 关闭闸门）

    for (int i = 1; i < argc; ++i)
    {
        string arg = argv[i];

        if (arg == "--video" && i + 1 < argc)
        {
            video_dev = argv[++i];
        }
        else if (arg == "--pcie-device" && i + 1 < argc)
        {
            pcie_device = argv[++i];
        }
        else if (arg == "--pcie-mem" && i + 1 < argc)
        {
            pcie_mem = argv[++i];
        }
        else if (arg == "--quiet-pcie")
        {
            quiet_pcie = true;
        }
        else if (arg == "--latest-frame" && i + 1 < argc)
        {
            // on（默认）= 画面与推理解耦：发最新采集帧，框滞后约一次推理耗时
            // off        = 旧行为：只发「推理完成的对齐帧」，框与画面严格同帧
            string v = argv[++i];
            g_send_latest_frame = !(v == "off" || v == "0" || v == "false" || v == "no");
            cout << "[V6] 画面/推理解耦: " << (g_send_latest_frame.load() ? "on（发最新帧）" : "off（发对齐帧）") << endl;
        }
        else if (arg == "--width" && i + 1 < argc)
        {
            cap_width = atoi(argv[++i]);
        }
        else if (arg == "--height" && i + 1 < argc)
        {
            cap_height = atoi(argv[++i]);
        }
        else if (arg == "--fps" && i + 1 < argc)
        {
            cap_fps = max(1, atoi(argv[++i]));
        }
        else if (arg == "--fourcc" && i + 1 < argc)
        {
            fourcc = argv[++i];
        }
        else if (arg == "--port" && i + 1 < argc)
        {
            port = clampPort(atoi(argv[++i]));
        }
        else if (arg == "--detect-model" && i + 1 < argc)
        {
            detect_model = argv[++i];
        }
        else if (arg == "--adas-model" && i + 1 < argc)
        {
            adas_model = argv[++i];
        }
        else if (arg == "--adas-serial-dev" && i + 1 < argc)
        {
            adas_serial_dev = argv[++i];
        }
        else if (arg == "--adas-serial-baud" && i + 1 < argc)
        {
            adas_serial_baud = atoi(argv[++i]);
        }
        else if (arg == "--adas-serial-period" && i + 1 < argc)
        {
            adas_serial_period_ms = max(10, atoi(argv[++i]));
        }
        else if (arg == "--adas-conf" && i + 1 < argc)
        {
            adas_conf_thres = static_cast<float>(atof(argv[++i]));
        }
        else if (arg == "--adas-nms" && i + 1 < argc)
        {
            adas_nms_thres = static_cast<float>(atof(argv[++i]));
        }
        else if (arg == "--adas-workers" && i + 1 < argc)
        {
            // 1 = 单上下文（旧行为）；2 = 双上下文（同一模型两份实例并行）
            adas_workers = max(1, min(2, atoi(argv[++i])));
        }
        else if (arg == "--ai-ctrl-port" && i + 1 < argc)
        {
            // AI 识别闸门 UDP 端口：上位机据此下发 "AI 0/1"（0 = 关闭闸门）
            ai_ctrl_port = max(0, atoi(argv[++i]));
        }
        else if (arg == "--recog-model" && i + 1 < argc)
        {
            recog_model = argv[++i];
            recog_model_user_set = true;
        }
        else if (arg == "--ocr-backend" && i + 1 < argc)
        {
            ocr_backend_name = argv[++i];
        }
        else if (arg == "--recog-chars" && i + 1 < argc)
        {
            ocr_chars_path = argv[++i];
        }
        else if (arg == "--font" && i + 1 < argc)
        {
            font_path = argv[++i];
        }
        else if (arg == "--infer-fps" && i + 1 < argc)
        {
            infer_fps = max(1, atoi(argv[++i]));
        }
        else if (arg == "--output-fps" && i + 1 < argc)
        {
            output_fps = max(1, atoi(argv[++i]));
        }
        else if (arg == "--zlm-sdk-dir" && i + 1 < argc)
        {
            zlm_cfg.sdk_dir = argv[++i];
            if (!zlm_config_explicit)
                zlm_cfg.config_ini = zlm_cfg.sdk_dir + "/config.ini";
        }
        else if (arg == "--zlm-config" && i + 1 < argc)
        {
            zlm_cfg.config_ini = argv[++i];
            zlm_config_explicit = true;
        }
        else if (arg == "--zlm-vhost" && i + 1 < argc)
        {
            zlm_cfg.vhost = argv[++i];
        }
        else if (arg == "--zlm-app" && i + 1 < argc)
        {
            zlm_cfg.app = argv[++i];
        }
        else if (arg == "--zlm-stream" && i + 1 < argc)
        {
            zlm_cfg.stream = argv[++i];
        }
        else if (arg == "--zlm-http-port" && i + 1 < argc)
        {
            zlm_cfg.http_port = clampPort(atoi(argv[++i]));
        }
        else if (arg == "--zlm-rtsp-port" && i + 1 < argc)
        {
            zlm_cfg.rtsp_port = clampPort(atoi(argv[++i]));
        }
        else if (arg == "--zlm-rtmp-port" && i + 1 < argc)
        {
            zlm_cfg.rtmp_port = clampPort(atoi(argv[++i]));
        }
        else if (arg == "--zlm-rtc-port" && i + 1 < argc)
        {
            zlm_cfg.rtc_port = clampPort(atoi(argv[++i]));
        }
        else if (arg == "--zlm-bitrate" && i + 1 < argc)
        {
            zlm_cfg.bitrate = max(1, atoi(argv[++i]));
        }
        else if (arg == "--zlm-gop" && i + 1 < argc)
        {
            zlm_gop = max(1, atoi(argv[++i]));
        }
        else if (arg == "--zlm-profile" && i + 1 < argc)
        {
            zlm_profile = argv[++i];
        }
        else if (arg == "--mpp-format" && i + 1 < argc)
        {
            mpp_format = argv[++i];
        }
        else if (arg == "--conf" && i + 1 < argc)
        {
            conf_thres = static_cast<float>(atof(argv[++i]));
        }
        else if (arg == "--nms" && i + 1 < argc)
        {
            nms_thres = static_cast<float>(atof(argv[++i]));
        }
        else if (arg == "--yolo-only")
        {
            yolo_only = true;
        }
        else if (arg == "--profile-yolo")
        {
            profile_yolo = true;
        }
        else if (arg == "--ocr-pad-x" && i + 1 < argc)
        {
            ocr_pad_x = max(0.0f, static_cast<float>(atof(argv[++i])));
        }
        else if (arg == "--ocr-pad-y" && i + 1 < argc)
        {
            ocr_pad_y = max(0.0f, static_cast<float>(atof(argv[++i])));
        }
        else if (arg == "--expand-ocr-roi")
        {
            expand_ocr_roi = true;
        }
        else if (arg == "--ocr-vote-window" && i + 1 < argc)
        {
            ocr_vote_window = max(1, atoi(argv[++i]));
        }
        else if (arg == "--ocr-min-votes" && i + 1 < argc)
        {
            ocr_min_votes = max(1, atoi(argv[++i]));
        }
        else if (arg == "--debug-ocr")
        {
            debug_ocr = true;
            if (debug_ocr_dir.empty())
                debug_ocr_dir = "/tmp/ocr_debug";
        }
        else if (arg == "--debug-ocr-dir" && i + 1 < argc)
        {
            debug_ocr_dir = argv[++i];
            debug_ocr = true;
        }
        else if (arg == "--obb-jitter-log" && i + 1 < argc)
        {
            obb_jitter_log = argv[++i];
        }
        else if (arg == "--no-infer")
        {
            enable_infer = false;
        }
        else if (arg == "--no-stream")
        {
            enable_stream = false;
        }
        else if (arg == "--no-video-text")
        {
            draw_video_text = false;
        }
        else if (arg == "--fpga-mode" && i + 1 < argc)
        {
            fpga_mode = argv[++i];
        }
        else if (arg == "--fpga-dir" && i + 1 < argc)
        {
            fpga_dir = argv[++i];
        }
        else if (arg == "--fpga-timeout" && i + 1 < argc)
        {
            fpga_timeout_ms = max(1, atoi(argv[++i]));
        }
        else if (arg == "--fpga-video" && i + 1 < argc)
        {
            fpga_video_dev = argv[++i];
        }
        else if (arg == "--fpga-width" && i + 1 < argc)
        {
            fpga_width = atoi(argv[++i]);
        }
        else if (arg == "--fpga-height" && i + 1 < argc)
        {
            fpga_height = atoi(argv[++i]);
        }
        else if (arg == "--fpga-fps" && i + 1 < argc)
        {
            fpga_fps = max(1, atoi(argv[++i]));
        }
        else if (arg == "--fpga-fourcc" && i + 1 < argc)
        {
            fpga_fourcc = argv[++i];
        }
        else if (arg == "--fpga-settle-ms" && i + 1 < argc)
        {
            fpga_settle_ms = max(0, atoi(argv[++i]));
        }
        else if (arg == "--drm-card" && i + 1 < argc)
        {
            drm_card = argv[++i];
        }
        else if (arg == "--drm-connector" && i + 1 < argc)
        {
            drm_connector_id = static_cast<uint32_t>(strtoul(argv[++i], nullptr, 0));
        }
        else if (arg == "--drm-width" && i + 1 < argc)
        {
            drm_width = atoi(argv[++i]);
        }
        else if (arg == "--drm-height" && i + 1 < argc)
        {
            drm_height = atoi(argv[++i]);
        }
        else if (arg == "--hdmi-out" && i + 1 < argc)
        {
            string v = argv[++i];
            hdmi_out = (v == "on" || v == "1" || v == "true" || v == "yes");
        }
        else if (arg == "--test-pattern")
        {
            test_pattern = true;
        }
        else if (arg == "--jpeg-udp-dest" && i + 1 < argc)
        {
            jpeg_udp_dest = argv[++i];  // e.g. "192.168.137.1:9999"
        }
        else if (arg == "--help" || arg == "-h")
        {
            printHelp(argv[0]);
            return 0;
        }
        else
        {
            cerr << "Unknown argument: " << arg << endl;
            printHelp(argv[0]);
            return 1;
        }
    }

    g_target_infer_fps = infer_fps;
    g_target_output_fps = output_fps;
    g_yolo_only = yolo_only;
    g_profile_yolo = profile_yolo;
    g_quiet_pcie = quiet_pcie;
    // V5 latency opt: default GOP=10 instead of output_fps.
    // A shorter GOP means the client gets a key-frame sooner after connecting
    // or after packet loss, cutting hundreds of ms of start-up / recovery delay.
    // Baseline profile with GOP=10 adds negligible bitrate overhead on RK MPP.
    if (zlm_gop <= 0)
        zlm_gop = 10;

    if (!parseOcrBackendName(ocr_backend_name, ocr_backend))
    {
        cerr << "Unsupported --ocr-backend: " << ocr_backend_name
             << ". Expected lprnet or plate_color." << endl;
        return 1;
    }
    ocr_backend_name = ocrBackendName(ocr_backend);
    if (!recog_model_user_set)
    {
        recog_model = (ocr_backend == OcrBackend::LPRNET)
                          ? "./model/lprnet_color_fp16.rknn"
                          : "./model/plate_recognition_color.rknn";
    }

    transform(mpp_format.begin(), mpp_format.end(), mpp_format.begin(), ::tolower);
    if (mpp_format != "nv12")
    {
        cerr << "Unsupported --mpp-format: " << mpp_format << ". Only nv12 is supported." << endl;
        return 1;
    }

    zlm_cfg.width = cap_width;
    zlm_cfg.height = cap_height;
    zlm_cfg.fps = cap_fps;   // match capture rate

    MppH264Config mpp_cfg;
    mpp_cfg.width = cap_width;
    mpp_cfg.height = cap_height;
    mpp_cfg.fps = cap_fps;   // match capture rate
    mpp_cfg.bitrate = zlm_cfg.bitrate;
    mpp_cfg.gop = zlm_gop;
    mpp_cfg.profile = zlm_profile;

    cout << "OpenCV version: " << CV_VERSION << endl;
    cout << "PCIe capture: device=" << pcie_device
         << ", mem=" << pcie_mem
         << ", frame=" << cap_width << "x" << cap_height
         << ", wire_format=RGB565_LE, byte_swap=off"
         << ", row_bytes=" << static_cast<size_t>(cap_width) * 2U
         << ", frame_bytes="
         << static_cast<size_t>(cap_width)
                * static_cast<size_t>(cap_height) * 2U
         << ", read_fps_limit=" << cap_fps << endl;
    cout << "RKNN inference: " << (enable_infer ? "enabled" : "disabled")
         << ", target inference FPS=" << g_target_infer_fps.load()
         << ", target output FPS=" << g_target_output_fps.load() << endl;
    if (enable_infer && enable_stream)
    {
        cout << "Inference video mode: exact source-frame alignment; "
             << "the last aligned frame is repeated at output FPS while RKNN runs."
             << endl;
    }
    cout << "YOLO-only: " << (yolo_only ? "on" : "off") << endl;
    cout << "YOLO profiling: " << (profile_yolo ? "on" : "off") << endl;
    cout << "PCIe quiet logs: " << (quiet_pcie ? "on" : "off") << endl;
    if (enable_stream)
    {
        cout << "Output stream: backend=ZLMediaKit"
             << ", output=" << zlm_cfg.width << "x" << zlm_cfg.height
             << ", fps=" << zlm_cfg.fps
             << ", mpp_format=" << mpp_format << endl;
        cout << "H.264 bitrate: " << mpp_cfg.bitrate << endl;
        cout << "H.264 gop: " << mpp_cfg.gop << endl;
        cout << "H.264 profile: " << mpp_cfg.profile << endl;
    }
    else
    {
        cout << "Output stream: disabled by --no-stream" << endl;
    }
    if (enable_infer)
    {
        cout << "RKNN models: detect=" << detect_model;
        if (!yolo_only)
        {
            cout << ", ocr=" << recog_model
                 << ", ocr_backend=" << ocr_backend_name;
            if (ocr_backend == OcrBackend::LPRNET)
            {
                cout << ", chars=" << ocr_chars_path;
            }
        }
        cout << endl;
    }
    if (yolo_only)
    {
        cout << "OCR/FPGA ROI: disabled by --yolo-only" << endl;
    }
    else
    {
        cout << "FPGA ROI processor: mode=" << fpga_mode
             << ", dir=" << fpga_dir
             << ", timeout_ms=" << fpga_timeout_ms << endl;
        cout << "OCR stabilization: primary=Python bbox ROI"
             << ", expand_roi=" << (expand_ocr_roi ? "on" : "off")
             << ", tight_pad_x=" << ocr_pad_x
             << ", tight_pad_y=" << ocr_pad_y
             << ", vote_window=" << ocr_vote_window
             << ", min_votes=" << min(ocr_min_votes, ocr_vote_window);
        if (debug_ocr)
            cout << ", debug_dir=" << debug_ocr_dir;
        cout << endl;
    }

    PlateRecognizer recognizer;
    PlateRecognizer *recognizer_ptr = nullptr;

    if (enable_infer && !adas_model.empty())
    {
        // ---- ADAS 模式：12 类感知模型 + 串口下发，不加载车牌/OCR 模型 ----
        adas_mode = true;
        if (!fileExists(adas_model))
        {
            cerr << "[ADAS] 模型不存在: " << adas_model << endl;
            return 1;
        }
        if (!g_adas[0].init(adas_model))
        {
            cerr << "[ADAS] 检测器初始化失败" << endl;
            return 1;
        }
        g_adas_workers = adas_workers;
        for (int w = 1; w < g_adas_workers; ++w)
        {
            // 双上下文：同一份 .rknn 再载入一份独立实例（各自 context 与缓冲）
            if (!g_adas[w].init(adas_model))
            {
                cerr << "[ADAS] 第 " << (w + 1) << " 份模型实例初始化失败，回退单上下文" << endl;
                g_adas_workers = 1;
                break;
            }
        }
        g_adas_conf_thres = adas_conf_thres;
        g_adas_nms_thres = adas_nms_thres;
        cout << "[ADAS] 模式启用: model=" << adas_model
             << " conf=" << adas_conf_thres << " nms=" << adas_nms_thres
             << " workers=" << g_adas_workers << endl;

        if (g_adas_serial.open(adas_serial_dev, adas_serial_baud))
        {
            g_adas_serial.start(adas_serial_period_ms);
            cout << "[ADAS] 串口下发 " << adas_serial_dev
                 << " 周期=" << adas_serial_period_ms << "ms" << endl;
        }
        else
        {
            cerr << "[ADAS] 串口打开失败，仅本地推理与推流，不向下位机下发" << endl;
        }

        // AI 识别闸门：四路拼接 / 倒车辅助线时由上位机经 UDP 暂停推理
        g_ai_ctrl_port = ai_ctrl_port;
        if (g_ai_ctrl_port > 0)
            g_ai_ctrl_thread = thread(aiCtrlThread);
        else
            cout << "[AI-CTRL] 闸门已关闭（--ai-ctrl-port 0）" << endl;
    }
    else if (enable_infer)
    {
        if (!fileExists(detect_model))
        {
            cerr << "Detector model not found: " << detect_model << endl;
            cerr << "Copy yolov5n_i8.rknn to ./model or pass --detect-model PATH." << endl;
            return 1;
        }

        if (!yolo_only && !fileExists(recog_model))
        {
            cerr << "OCR model not found: " << recog_model << endl;
            if (ocr_backend == OcrBackend::PLATE_COLOR)
            {
                cerr << "Copy plate_recognition_color.rknn to ./model or pass --recog-model PATH." << endl;
            }
            else
            {
                cerr << "Copy lprnet_color_fp16.rknn to ./model or pass --recog-model PATH." << endl;
            }
            return 1;
        }

        if (!recognizer.init(detect_model, recog_model, font_path, conf_thres, nms_thres,
                             ocr_pad_x, ocr_pad_y, ocr_vote_window, ocr_min_votes, debug_ocr_dir,
                             obb_jitter_log, ocr_chars_path, ocr_backend, debug_ocr, expand_ocr_roi,
                             fpga_mode, fpga_dir, fpga_timeout_ms,
                             fpga_video_dev, fpga_width, fpga_height, fpga_fps, fpga_fourcc,
                             drm_card, drm_connector_id, drm_width, drm_height, fpga_settle_ms,
                             yolo_only, profile_yolo))
        {
            cerr << "RKNN recognizer init failed." << endl;
            return 1;
        }

        recognizer_ptr = &recognizer;
    }

    ZlmStreamer zlm_streamer;
    MppH264Encoder mpp_encoder;
    if (enable_stream)
    {
        if (!zlm_streamer.init(zlm_cfg))
        {
            cerr << "ZLMediaKit streamer init failed." << endl;
            return 1;
        }
        g_webrtc_url = zlm_streamer.webrtcUrl();
        g_http_flv_url = zlm_streamer.httpFlvUrl();
        g_rtsp_url = zlm_streamer.rtspUrl();

        if (!mpp_encoder.init(mpp_cfg))
        {
            cerr << "MPP encoder init failed." << endl;
            zlm_streamer.release();
            return 1;
        }
    }

    PcieRawFrameRing4 pcie_ring;
    thread convert_thread, cap_thread, infer_thread, enc_thread, server_thread;
    thread pattern_thread;
    thread jpeg_thread;

    if (!jpeg_udp_dest.empty()) {
        if (jpegUdpInit(jpeg_udp_dest))
            jpeg_thread = thread(jpegUdpSenderThread);   // 独立发送线程：编码线程不再等 JPEG
    }

    // 本地 HDMI 直出：单独占用一个 connector/CRTC，与 --fpga-mode hdmi 互斥。
    // 没接显示器时 init 会失败，这里只禁用本地显示，不影响推流与识别。
    if (hdmi_out)
    {
        if (!g_display.init(drm_card, drm_connector_id, drm_width, drm_height))
        {
            cerr << "[HDMI-OUT] init failed (no monitor / DRM busy?) - local display disabled." << endl;
        }
        else
        {
            g_display.presentTestPattern();
            g_display_on.store(true);
            if (!enable_stream)
                cerr << "[HDMI-OUT] warning: --no-stream means encoderThread never runs, "
                        "so the local display has no frame source (only the test pattern)." << endl;
            cout << "[HDMI-OUT] enabled " << drm_width << "x" << drm_height
                 << " on " << drm_card
                 << ((drm_width == cap_width && drm_height == cap_height)
                     ? " (matches frame size, scaling skipped)"
                     : " (scaling active)")
                 << endl;
        }
    }

    if (test_pattern) {
        cout << "TEST PATTERN mode: GPU-generated color bars, no FPGA/PCIe" << endl;
        pattern_thread = thread(testPatternThread, cap_width, cap_height, cap_fps);
    } else {
        if (!pcie_ring.init(cap_width, cap_height))
        {
            cerr << "Failed to initialize the 16 MiB four-slot PCIe frame ring." << endl;
            if (enable_stream)
            {
                mpp_encoder.release();
                zlm_streamer.release();
            }
            return 1;
        }
        convert_thread = thread(rawFrameConvertThread, &pcie_ring, cap_width, cap_height);
        cap_thread = thread(captureThread, pcie_device, pcie_mem,
                          cap_width, cap_height, cap_fps, &pcie_ring);
    }

    thread disp_thread;
    if (g_display_on.load())
        disp_thread = thread(displayThread);

    // ADAS 双上下文：两个 worker 轮流吃帧，取代旧的单线程推理路径
    // （旧路径此时不再启动，避免同一帧被两条通路重复处理）
    vector<thread> adas_worker_threads;
    const bool dual_adas = adas_mode && enable_infer && g_adas_workers > 1 &&
                           g_adas[0].enabled();
    if (dual_adas)
    {
        for (int w = 0; w < g_adas_workers; ++w)
            adas_worker_threads.emplace_back(adasDualWorker, w,
                                             enable_stream && enable_infer);
        cout << "[ADAS-dual] workers=" << g_adas_workers
             << "，两份模型实例并行推理（最新帧覆写、帧号单调发布）" << endl;
    }
    else
    {
        infer_thread = thread(inferenceThread, recognizer_ptr, enable_infer,
                            enable_stream && enable_infer, draw_video_text);
    }
    if (enable_stream)
    {
        enc_thread = thread(encoderThread, enable_infer,
                            &mpp_encoder, &zlm_streamer);
        server_thread = thread(httpServerThread, port);
    }

    while (g_running)
    {
        this_thread::sleep_for(chrono::milliseconds(500));
    }

    g_running = false;
    g_input_cv.notify_all();
    g_aligned_output_cv.notify_all();
    g_display_cv.notify_all();
    g_jpeg_pub_cv.notify_all();
    if (!test_pattern)
        pcie_ring.wakeAll();

    if (server_thread.joinable())
        server_thread.join();
    if (enc_thread.joinable())
        enc_thread.join();
    // 编码线程先停（不再投递新帧），再停 JPEG 发送线程，最后才关 socket
    if (jpeg_thread.joinable())
        jpeg_thread.join();
    {
        lock_guard<mutex> lock(g_jpeg_udp_mutex);
        int fd = g_jpeg_udp_fd.exchange(-1);
        if (fd >= 0)
            close(fd);
    }

    if (disp_thread.joinable())
        disp_thread.join();
    if (infer_thread.joinable())
        infer_thread.join();
    for (auto &t : adas_worker_threads)
        if (t.joinable())
            t.join();
    if (pattern_thread.joinable())
        pattern_thread.join();
    if (cap_thread.joinable())
        cap_thread.join();
    if (convert_thread.joinable())
        convert_thread.join();

    if (adas_mode)
        g_adas_serial.stop();

    // AI 识别闸门线程：g_running 已置 false，recvfrom 到超时就退出
    if (g_ai_ctrl_thread.joinable())
        g_ai_ctrl_thread.join();

    if (enable_stream)
    {
        mpp_encoder.release();
        zlm_streamer.release();
    }

    cout << "Exit." << endl;
    return 0;
}
