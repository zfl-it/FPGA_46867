// adas_offline_test.cc — 离线验证工具（不接摄像头）
//
// 用途：在没有 FPGA 视频输入的情况下，验证
//   ① mapAdasSignals 的交规映射逻辑（圆盘灯 / 箭头灯 / 车道线 / 行人 / 斑马线）
//   ② 真实 RKNN 模型对 1280x720 图片的推理与坐标还原
//   ③ 串口发送线程的帧格式与周期
//
// 编译（板端）：
//   g++ -O2 -std=c++17 adas_offline_test.cc adas_detector.cc -o adas_offline_test \
//       -I/usr/include/opencv4 \
//       -I/userdata/RKSDK/rknn-toolkit2/rknpu2/runtime/Linux/librknn_api/include \
//       -L/userdata/RKSDK/rknn-toolkit2/rknpu2/runtime/Linux/librknn_api/aarch64 -lrknnrt \
//       -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -lpthread
//
// 用法：
//   ./adas_offline_test map
//   ./adas_offline_test img <model.rknn> <image> [loops]
//   ./adas_offline_test serial <pty_slave> [ms]

#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "adas_detector.h"

static AdasDetection mk(int cls, float score, int cx, int cy, int w, int h)
{
    AdasDetection d;
    d.cls = cls;
    d.score = score;
    d.box = cv::Rect(cx - w / 2, cy - h / 2, w, h);
    return d;
}

// ---------------------------------------------------------------- ① 映射逻辑
static int g_fail = 0;

static void expect(const char *name, const std::vector<AdasDetection> &dets,
                   int frame_w, const char *want_bits, int want_ped)
{
    AdasSignals s = mapAdasSignals(dets, frame_w);
    bool ok = s.bits() == want_bits && s.ped_count == want_ped;
    if (!ok)
        ++g_fail;
    std::printf("  %s %-46s bits=%s ped=%d  期望 bits=%s ped=%d\n",
                ok ? "[PASS]" : "[FAIL]", name, s.bits().c_str(), s.ped_count,
                want_bits, want_ped);
}

static int run_map()
{
    std::printf("=== ① 交规映射逻辑（不依赖模型）===\n");
    std::printf("位串含义: [0]左实线 [1]右实线 [2]直行绿 [3]左转绿 [4]右转绿 [5]斑马线\n\n");

    // 圆盘绿灯：三向全放行
    expect("圆盘绿灯 -> 直行/左转/右转 全 1",
           {mk(ADAS_DISC_GREEN, 0.88f, 640, 200, 60, 120)}, 1280, "001110", 0);

    // 圆盘红灯：禁直行禁左转，右转放行
    expect("圆盘红灯 -> 直0 左0 右1",
           {mk(ADAS_DISC_RED, 0.86f, 640, 200, 60, 120)}, 1280, "000010", 0);

    // 圆盘灯优先于箭头灯
    expect("圆盘红灯 + 直行箭头绿 -> 仍按圆盘为准(0 0 1)",
           {mk(ADAS_STRAIGHT_GREEN, 0.92f, 400, 180, 40, 80),
            mk(ADAS_DISC_RED, 0.71f, 640, 200, 60, 120)},
           1280, "000010", 0);
    expect("圆盘绿灯 + 左转箭头红 -> 仍按圆盘为准(1 1 1)",
           {mk(ADAS_LEFT_RED, 0.93f, 400, 180, 40, 80),
            mk(ADAS_DISC_GREEN, 0.66f, 640, 200, 60, 120)},
           1280, "001110", 0);

    // 圆盘红绿同现：取分数高者
    expect("圆盘红绿同现 -> 绿分高者胜(全放行)",
           {mk(ADAS_DISC_RED, 0.41f, 600, 200, 50, 100),
            mk(ADAS_DISC_GREEN, 0.79f, 640, 200, 50, 100)},
           1280, "001110", 0);

    // 无圆盘灯时保持原箭头逻辑（未识别到该方向 -> 默认放行）
    expect("无灯 -> 三向默认放行（原机制不变）", {}, 1280, "001110", 0);
    expect("仅直行箭头红 -> 直0 左1 右1",
           {mk(ADAS_STRAIGHT_RED, 0.83f, 640, 180, 40, 80)}, 1280, "000110", 0);
    expect("仅左转箭头绿 -> 直1 左1 右1",
           {mk(ADAS_LEFT_GREEN, 0.81f, 400, 180, 40, 80)}, 1280, "001110", 0);
    expect("右转箭头红 -> 右0",
           {mk(ADAS_RIGHT_RED, 0.80f, 900, 180, 40, 80)}, 1280, "001100", 0);
    expect("左转红绿同现 -> 绿分高者胜",
           {mk(ADAS_LEFT_RED, 0.35f, 400, 180, 40, 80),
            mk(ADAS_LEFT_GREEN, 0.77f, 400, 180, 40, 80)},
           1280, "001110", 0);

    // 车道线：按框中心 x 归属左右
    expect("左侧实线（中心 x<640）",
           {mk(ADAS_SOLID, 0.70f, 300, 500, 40, 300)}, 1280, "101110", 0);
    expect("右侧实线（中心 x>=640）",
           {mk(ADAS_SOLID, 0.70f, 980, 500, 40, 300)}, 1280, "011110", 0);
    expect("左右各一条实线",
           {mk(ADAS_SOLID, 0.70f, 300, 500, 40, 300),
            mk(ADAS_SOLID, 0.70f, 980, 500, 40, 300)},
           1280, "111110", 0);
    expect("虚线不算实线（不应置位）",
           {mk(ADAS_DASH, 0.90f, 300, 500, 40, 300)}, 1280, "001110", 0);

    // 斑马线 + 行人计数
    expect("斑马线 -> 末位 1",
           {mk(ADAS_CROSSWALK, 0.75f, 640, 600, 600, 120)}, 1280, "001111", 0);
    expect("3 个行人 -> ped=3",
           {mk(ADAS_PED, 0.80f, 200, 500, 40, 100),
            mk(ADAS_PED, 0.70f, 500, 520, 40, 100),
            mk(ADAS_PED, 0.66f, 900, 510, 40, 100)},
           1280, "001110", 3);

    // 行人数上限
    std::vector<AdasDetection> many;
    for (int i = 0; i < 1200; ++i)
        many.push_back(mk(ADAS_PED, 0.5f, 100 + (i % 1000), 400, 20, 60));
    expect("行人数上限截断为 999", many, 1280, "001110", 999);

    std::printf("\n  映射逻辑: %s（失败 %d 项）\n\n", g_fail == 0 ? "全部通过" : "有失败", g_fail);
    return g_fail == 0 ? 0 : 1;
}

// ---------------------------------------------------------------- ② 真机推理
static int run_img(const char *model, const char *img_path, int loops)
{
    std::printf("=== ② 真实模型 + 720P 图片推理 ===\n");
    AdasDetector det;
    if (!det.init(model))
    {
        std::printf("  模型加载失败\n");
        return 2;
    }
    std::printf("  模型输入尺寸: %dx%d (宽x高)\n", det.width(), det.height());

    cv::Mat bgr = cv::imread(img_path, cv::IMREAD_COLOR);
    if (bgr.empty())
    {
        std::printf("  图片读取失败: %s\n", img_path);
        return 2;
    }
    std::printf("  图片: %s  尺寸 %dx%d 通道 %d\n", img_path, bgr.cols, bgr.rows, bgr.channels());

    // letterbox 计算说明
    float sc = std::min((float)det.height() / bgr.rows, (float)det.width() / bgr.cols);
    int nw = (int)(bgr.cols * sc), nh = (int)(bgr.rows * sc);
    std::printf("  letterbox: scale=%.4f -> %dx%d, 左右补边 %d, 上下补边 %d\n",
                sc, nw, nh, (det.width() - nw) / 2, (det.height() - nh) / 2);

    std::vector<AdasDetection> dets;
    double total_ms = 0;
    for (int i = 0; i < loops; ++i)
    {
        auto t0 = std::chrono::steady_clock::now();
        bool ok = det.detect(bgr, dets, 0.25f, 0.45f);
        auto t1 = std::chrono::steady_clock::now();
        double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        total_ms += ms;
        if (!ok)
        {
            std::printf("  推理失败\n");
            return 2;
        }
        if (i == 0)
        {
            std::printf("  检测到 %zu 个目标（conf>=0.25）:\n", dets.size());
            for (const auto &d : dets)
                std::printf("    %-14s score=%.3f box=(%d,%d,%d,%d)\n",
                            adasClassName(d.cls), d.score, d.box.x, d.box.y,
                            d.box.width, d.box.height);
        }
    }
    std::printf("  推理耗时: 平均 %.1f ms/帧 (%.1f fps), 共 %d 次\n",
                total_ms / loops, 1000.0 * loops / total_ms, loops);

    AdasSignals sig = mapAdasSignals(dets, bgr.cols);
    std::printf("  >>> 串口帧: \"%s\"\n", sig.frame().c_str());
    return 0;
}

// ---------------------------------------------------------------- ③ 串口线程
static int run_serial(const char *dev, int ms)
{
    std::printf("=== ③ 串口发送线程 ===\n");
    AdasSerialSender s;
    if (!s.open(dev, 115200))
        return 2;
    AdasSignals sig;
    sig.left_line_solid = true;
    sig.right_line_solid = true;
    sig.straight_green = false;
    sig.left_green = false;
    sig.right_green = true;
    sig.crosswalk_ahead = true;
    sig.ped_count = 7;
    std::printf("  待发送帧内容: \"%s\"\n", sig.frame().c_str());
    s.start(50);
    s.publish(sig);
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
    s.stop();
    std::printf("  已发送 %d ms（应约 %d 帧）\n", ms, ms / 50);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::printf("用法: %s map | img <model> <image> [loops] | serial <dev> [ms]\n", argv[0]);
        return 1;
    }
    std::string mode = argv[1];
    if (mode == "map")
        return run_map();
    if (mode == "img" && argc >= 4)
        return run_img(argv[2], argv[3], argc >= 5 ? std::atoi(argv[4]) : 1);
    if (mode == "serial" && argc >= 3)
        return run_serial(argv[2], argc >= 4 ? std::atoi(argv[3]) : 600);
    std::printf("参数错误\n");
    return 1;
}
