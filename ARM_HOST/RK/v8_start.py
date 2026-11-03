#!/usr/bin/env python3
"""V8_720P control: start / stop / status."""

import paramiko, sys, time, os

BOARD_IP = "192.168.137.100"
BOARD_USER = "linaro"
BOARD_PASS = "linaro"
BOARD_V8 = "/userdata/RK3568_V8_720P"
BUILD = f"{BOARD_V8}/cpp/build"
MODEL = f"{BOARD_V8}/model"

# ---- 低延迟相关 ----
# GOP=1 = 每帧都是 I 帧：RTSP 服务端不必攒够一个 GOP 才开始转发，
# 25fps 下能消掉约 400ms 的 GOP 缓冲 —— 这是本地 HDMI 显示延迟的最大单项。
# 代价是码率上升，所以同步把 bitrate 提到 8 Mbps。
# 带宽紧张或 PC 端卡顿就把 ZLM_GOP 改回 10、ZLM_BITRATE 改回 3000000。
ZLM_GOP = 1
ZLM_BITRATE = 8000000

# True = 主程序直接把推理帧写进 DRM 出 HDMI（方案 B，延迟降到 1~2 帧）。
# 前提：板上的二进制是用加了 --hdmi-out 的新 main.cc 编出来的。
# 启用后不要再跑 v8_hdmi_display.py —— 两者抢同一个 CRTC，会互相顶掉。
#
# 2026-10-29 实测：modetest 显示 HDMI-A-1(175) 为 disconnected、只有 DSI-1(191)
# 连接，说明当时没接 HDMI 显示器。此时开 True 会让 g_display.init() 失败并自动
# 降级（只打一行 [HDMI-OUT] init failed，推流与识别不受影响），所以默认保持 False。
# 接上 HDMI 显示器、确认 `cat /sys/class/drm/card0-HDMI-A-1/status` 是 connected
# 之后，把这里改成 True 即可，不用重新编译。
HDMI_OUT = False
DRM_CONNECTOR = 175   # modetest -c: 175 = HDMI-A-1, 191 = DSI-1
DRM_W = 1280      # 与 --width/--height 一致时 presentDirect 会跳过缩放
DRM_H = 720

# True = 启动时把 CPU governor 切到 performance。
# CPU 侧要跑 BGR565->BGR、BGR->NV12、JPEG 编码，ondemand 会让它们跑在中间档位。
# 收益约 5~15ms，代价是温度上升（实测 59°C 起步，切完注意观察）。
CPU_PERF = True

# ---- ADAS 12 类感知模式 ----
# True = 用 ADAS 模型（车道线/信号灯/圆盘灯/行人/斑马线）替换车牌+OCR 流水线，
#        并把 6 路布尔 + 行人数通过串口发给上位机「交规决策模拟器」。
# 传了 --adas-model 就会自动进入 ADAS 模式，车牌/OCR 模型不再加载。
ADAS_MODE = True
# 用「置信度预乘 1280」的 int8 版：避开 RKNN per-tensor 量化把置信度压成 0 的坑。
# 当前用 BLP-7（YOLOv8n，12 类，含圆盘红灯/圆盘绿灯），C++ 侧 ADAS_NC=12 必须与之一致。
# 旧版本 adas12_*(BL-3/v8s)、adas12n_*(BLP-3)、adas12n4_*(BLP-4)、adas12n6_*(BLP-6)
# 本地 model/ 里都保留，换回时改这一行 + python push_and_build.py 即可。
ADAS_MODEL = f"{MODEL}/adas12n7_736x1280_i8_scaled.rknn"
ADAS_SERIAL_DEV = "/dev/ttyS3"     # 2026-10-27 实测：40PIN pin8(TX)/pin9(GND)/pin10(RX) = ttyS3（PIN-TEST 广播实证）；
                                   # 顶边丝印 TTL 座才是 ttyS4，但那是细间距座插不了杜邦线
ADAS_SERIAL_BAUD = 115200          # 与上位机一致：115200-8-N-1，无流控
ADAS_SERIAL_PERIOD_MS = 50         # 20 Hz 下发
ADAS_CONF = 0.25
ADAS_NMS = 0.45
# 双上下文 worker 数：2 = 同一模型载两份实例并行推理（框/串口更新约 +28%），
# 1 = 旧的单线程路径。不想改文件时用环境变量：ADAS_WORKERS=1 v8_start.py start
ADAS_WORKERS = int(os.environ.get("ADAS_WORKERS", "2"))
# AI 识别闸门：上位机「四路拼接 / 倒车辅助线」时经该 UDP 端口下发 "AI 0/1"，
# 让板端暂停 / 恢复 ADAS 推理（0 = 关闭闸门）。与上位机 AI_CTRL_PORT 一致。
AI_CTRL_PORT = int(os.environ.get("AI_CTRL_PORT", "9998"))

# True  = 画面与推理解耦：编码线程发「最新采集帧」（25fps、帧龄约 40ms），
#         检测框用最近一次推理结果重绘，因此框相对画面滞后约一次推理耗时（~120ms）。
#         识别链路与串口输出完全不变。
# False = 旧行为：只发「推理完成的对齐帧」，框与画面严格同帧，但画面只有 8fps、
#         帧龄约 250ms。演示要求「框绝对不能拖影」时改这里。
SEND_LATEST_FRAME = True

# True = 用内部测试图源代替 FPGA/PCIe 输入。FPGA 还没烧比特流、或 PCIe 一直报
# "waiting for FPGA frame-ready status ... EPERM" 时，开这个可以先跑通
# 「推理 → 串口下发 → 上位机」整条链路；真机联调时务必改回 False。
TEST_PATTERN = False

def run(ssh, cmd, timeout=15):
    _, out, err = ssh.exec_command(cmd, timeout=timeout)
    return out.read().decode().strip(), err.read().decode().strip()

def do_status(ssh):
    o, _ = run(ssh, "ps aux | grep rknn_browser | grep -v grep")
    if o:
        print("[RUNNING]")
        print(o[:500])
    else:
        print("[STOPPED]")

def do_stop(ssh):
    run(ssh, "sudo pkill -9 -f rknn_browser 2>/dev/null; sleep 1; echo stopped")
    print("Stopped")

def do_start(ssh):
    # Stop old first
    run(ssh, "sudo pkill -9 -f rknn_browser 2>/dev/null; sleep 1")

    # 板子每次冷启动后 pango PCIe 驱动不会自动加载，缺它就没有视频输入
    # （run.log 会刷 "open /dev/pango_pci_driver failed: No such file or directory"）。
    # 这里自动检查并加载，等价于手动: sudo insmod /userdata/RK3568_V8_720P/pango_pci_driver.ko
    o, _ = run(ssh, "ls /dev/pango_pci_driver 2>/dev/null")
    if not o:
        print("PCIe driver missing, loading pango_pci_driver.ko ...")
        run(ssh, "sudo insmod /userdata/RK3568_V8_720P/pango_pci_driver.ko 2>&1; sleep 1")
        o2, _ = run(ssh, "ls /dev/pango_pci_driver 2>/dev/null")
        if o2:
            print("[OK] PCIe driver loaded, /dev/pango_pci_driver ready")
        else:
            print("[WARN] 驱动加载后设备节点仍不存在，请检查 FPGA 是否上电/插好（lspci 应有 Device 0755:0755）")

    # ADAS 模式与车牌模式互斥：传 --adas-model 即走 ADAS 分支
    if ADAS_MODE:
        model_args = (
            f"--adas-model {ADAS_MODEL} "
            f"--adas-conf {ADAS_CONF} --adas-nms {ADAS_NMS} "
            f"--adas-serial-dev {ADAS_SERIAL_DEV} "
            f"--adas-serial-baud {ADAS_SERIAL_BAUD} "
            f"--adas-serial-period {ADAS_SERIAL_PERIOD_MS} "
            f"--adas-workers {ADAS_WORKERS} "
            f"--ai-ctrl-port {AI_CTRL_PORT} "
            f"--latest-frame {'on' if SEND_LATEST_FRAME else 'off'} "
        )
    else:
        model_args = (
            f"--detect-model {MODEL}/plate_obb_i8.rknn "
            f"--ocr-backend lprnet "
            f"--recog-model {MODEL}/lprnet_color_fp16.rknn "
            f"--recog-chars {MODEL}/lprnet_chars.txt "
            f"--font {MODEL}/simhei.ttf "
        )

    # HDMI 直出独占 CRTC：方案 A 的 gst 管线会和它互相顶掉，先停掉
    if HDMI_OUT:
        run(ssh, "sudo pkill -f 'gst-launch-1.0' 2>/dev/null; sleep 1")

    # CPU 调频：ondemand 会让 BGR565->BGR / NV12 / JPEG 编码跑在中间档
    if CPU_PERF:
        run(ssh, "for f in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; "
                 "do echo performance | sudo tee $f >/dev/null 2>&1; done; echo ok")
        o, _ = run(ssh, "cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq; "
                        "cat /sys/class/thermal/thermal_zone0/temp")
        print("[CPU] governor=performance  cur_freq/temp:", o.replace("\n", " "))

    start_cmd = (
        f"cd {BUILD} && "
        f"export LD_LIBRARY_PATH=/userdata/RKSDK/rknn-toolkit2/rknpu2/runtime/Linux/librknn_api/aarch64:{BOARD_V8}/cpp/ZLMediaKit/release/linux/Release:$LD_LIBRARY_PATH && "
        f"sudo setsid ./rknn_browser_hdmi_lpr_demo "
        f"--conf 0.25 --nms 0.4 --jpeg-udp-dest 192.168.137.1:9999 "
        f"--width 1280 --height 720 --fps 25 "
        f"--infer-fps 1000 --output-fps 25 --zlm-gop {ZLM_GOP} --zlm-bitrate {ZLM_BITRATE} "
        + model_args +
        f"--zlm-sdk-dir {BOARD_V8}/cpp/ZLMediaKit "
        f"--zlm-http-port 8084 --zlm-rtsp-port 8558 "
        f"--zlm-rtmp-port 1939 --zlm-rtc-port 8004 "
        f"--port 8080 "
        + (f"--hdmi-out on --drm-connector {DRM_CONNECTOR} --drm-width {DRM_W} --drm-height {DRM_H} " if HDMI_OUT else "")
        + ("--test-pattern " if TEST_PATTERN else "")
        + f"</dev/null >run.log 2>&1 &"
    )
    print("Launching V8_720P (%s)..." % ("ADAS" if ADAS_MODE else "PLATE"))
    print("Launching V8_720P...")
    ssh.exec_command(start_cmd, timeout=8)
    time.sleep(6)

    o, _ = run(ssh, "ps aux | grep rknn_browser | grep -v grep")
    if o:
        print("[OK] Process running")
    else:
        print("[FAIL] Process not found")

    # Show key logs
    o, _ = run(ssh, f"tail -25 {BUILD}/run.log 2>/dev/null")
    if o:
        for line in o.split('\n'):
            if any(kw in line.lower() for kw in ['rtc', 'jpeg-udp', 'error', 'fail', 'test pattern', 'frame_delay', 'model latency', 'udp server', 'tcp server', 'rknn runtime']):
                print(f"  {line.strip()}")

def main():
    cmd = sys.argv[1] if len(sys.argv) > 1 else "status"
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(BOARD_IP, username=BOARD_USER, password=BOARD_PASS)

    if cmd == "start":
        do_start(ssh)
    elif cmd == "stop":
        do_stop(ssh)
    elif cmd == "restart":
        do_stop(ssh)
        time.sleep(1)
        do_start(ssh)
    else:
        do_status(ssh)

    ssh.close()

if __name__ == "__main__":
    main()
