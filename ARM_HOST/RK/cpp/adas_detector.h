// adas_detector.h — ADAS 12 类感知（YOLOv8n 640/1280）+ 交规信号映射 + 串口发送
//
// 类别顺序与 best.pt 的 names 严格一致（BL-3 模型，12 类）：
//   0 左转红灯  1 左转绿灯  2 右转红灯  3 右转绿灯
//   4 直行红灯  5 直行绿灯  6 虚线      7 实线
//   8 行人      9 斑马线   10 圆盘红灯 11 圆盘绿灯
//
// 圆盘灯（BL-3 新增）语义：
//   圆盘绿灯 -> 直行 / 左转 / 右转 三路全部放行（位串 1111?1，三灯均 1）
//   圆盘红灯 -> 禁止直行与左转、右转放行（直行 0、左转 0、右转 1）
//   圆盘灯优先于箭头灯：只要识别到圆盘灯，就以圆盘灯为准。
//
// 串口协议（与上位机「交规决策模拟器」对齐）：
//   115200-8-N-1，ASCII，形如 "111110 3\n"
//   6 位位串顺序 = 上位机 SENSORS：
//     bit0 左道线实线 / bit1 右道线实线 / bit2 直行绿灯
//     bit3 左转绿灯   / bit4 右转绿灯   / bit5 前方斑马线
//   后跟空格 + 行人数量（上位机正则：([01]{6})[ \t]+(\d{1,6})(?![0-9])）
#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include "rknn_api.h"

// ---------------------------------------------------------------- 类别定义
enum AdasClass
{
    ADAS_LEFT_RED = 0,
    ADAS_LEFT_GREEN = 1,
    ADAS_RIGHT_RED = 2,
    ADAS_RIGHT_GREEN = 3,
    ADAS_STRAIGHT_RED = 4,
    ADAS_STRAIGHT_GREEN = 5,
    ADAS_DASH = 6,
    ADAS_SOLID = 7,
    ADAS_PED = 8,
    ADAS_CROSSWALK = 9,
    ADAS_DISC_RED = 10,     // BL-3 新增：圆盘红灯（禁直行/左转，右转放行）
    ADAS_DISC_GREEN = 11,   // BL-3 新增：圆盘绿灯（三向全线放行）
    ADAS_NC = 12
};

struct AdasDetection
{
    cv::Rect box;   // 原图（1280x720）坐标
    int cls = 0;    // AdasClass
    float score = 0.f;
};

// ---------------------------------------------------------------- 交规信号
struct AdasSignals
{
    bool left_line_solid = false;
    bool right_line_solid = false;
    bool straight_green = false;
    bool left_green = false;
    bool right_green = false;
    bool crosswalk_ahead = false;
    int ped_count = 0;

    // 6 位位串，顺序与上位机 SENSORS 一致
    std::string bits() const;
    // 完整串口帧："111110 3"
    std::string frame() const;
};

// ---------------------------------------------------------------- 检测器
class AdasDetector
{
public:
    AdasDetector() = default;
    ~AdasDetector();

    bool init(const std::string &model_path);
    bool enabled() const { return ready_; }

    int width() const { return model_width_; }
    int height() const { return model_height_; }

    // bgr: 原始 BGR 帧（1280x720）；out: 原图坐标的检测结果
    bool detect(const cv::Mat &bgr,
                std::vector<AdasDetection> &out,
                float conf_thres,
                float nms_thres);

private:
    rknn_context ctx_ = 0;
    bool ready_ = false;
    int model_width_ = 0;
    int model_height_ = 0;
    int model_channel_ = 0;
    std::vector<rknn_tensor_attr> out_attrs_;
    std::vector<std::vector<float>> out_bufs_;

    // letterbox 缓冲复用（detect 只由推理线程单线程调用，无需加锁）：
    // 原来每帧都要新建 1280x736x3 的 canvas + resized + rgb 三块（各 2.8MB），
    // 其中 resized 还要整幅 copyTo 到 canvas。改成 resize 直接写 canvas 的 ROI，
    // 数值完全等价（同样是 INTER_LINEAR、同样的目标尺寸），只是省掉一次整幅拷贝
    // 和每帧的 malloc/free。
    cv::Mat lb_canvas_;
    cv::Mat lb_rgb_;

    // NPU 零拷贝输入（rknn_create_mem + rknn_set_io_mem）：
    // lb_rgb_ 直接包在 NPU 输入缓冲上，cvtColor 一次写到底，
    // 省掉 rknn_inputs_set 内部那趟 2.8MB 拷贝。初始化失败时自动回退老路径。
    rknn_tensor_attr in_attr_{};
    rknn_tensor_mem *in_mem_ = nullptr;
    bool zero_copy_ = false;

    // letterbox 参数（保留长宽比 + 灰边 114），与训练预处理一致
    struct Letterbox
    {
        float scale = 1.f;
        int new_w = 0;
        int new_h = 0;
        int left = 0;
        int top = 0;
    };
    Letterbox computeLetterbox(int img_w, int img_h) const;
};

// 10 类检测结果 -> 6 路布尔 + 行人数量
AdasSignals mapAdasSignals(const std::vector<AdasDetection> &dets, int frame_w);

// 在帧上画检测框与类别名（仅 ASCII 标签，避免中文字体依赖）
void drawAdasResults(cv::Mat &bgr,
                     const std::vector<AdasDetection> &dets,
                     const AdasSignals &sig);

// ---------------------------------------------------------------- 串口发送
class AdasSerialSender
{
public:
    ~AdasSerialSender();

    bool open(const std::string &dev, int baud);
    bool isOpen() const { return fd_ >= 0; }

    void start(int period_ms);   // 启动发送线程
    void stop();

    void publish(const AdasSignals &sig);   // 更新待发送状态

private:
    void loop(int period_ms);

    int fd_ = -1;
    std::atomic<bool> running_{false};
    std::thread th_;
    std::mutex mtx_;
    AdasSignals sig_;
    bool dirty_ = false;
};

// 类别名（日志用）
const char *adasClassName(int cls);
