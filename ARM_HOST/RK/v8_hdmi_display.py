#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
V8_720P · HDMI 本地显示（方案 A：零改 C++，板子自拉 RTSP → 解码 → DRM）

原理：板子自己拉自己的 RTSP 流，用 kmssink 直接写 DRM，把推理画面打到 HDMI。
     这条路要绕「H.264 编码 → RTSP → 解码 → 色彩转换」，延迟天生比
     方案 B（--hdmi-out 直出 DRM）大，所以本脚本的管道已按低延迟参数调优。

延迟构成（默认 --zlm-gop 10 时最严重）：
    GOP 缓冲 400ms  +  编码  +  RTSP 服务端攒帧  +  rtpjitterbuffer  +  解码  +  转换
本脚本能消掉的：jitterbuffer、队列堆积、同步等待。
必须配合改主程序启动参数才能消掉的：GOP（见 v8_start.py --zlm-gop）。

用法：
    python v8_hdmi_display.py check            # 上板自检（显示器/桌面/插件/流/GOP）
    python v8_hdmi_display.py start            # 低延迟启动 HDMI 显示（1280x720）
    python v8_hdmi_display.py start 1920 1080  # 指定显示分辨率（需显示器支持该模式）
    python v8_hdmi_display.py stop             # 停止
    python v8_hdmi_display.py status           # 看进程与日志
    python v8_hdmi_display.py test             # 不拉 RTSP，直接打彩条测 HDMI 通路
    python v8_hdmi_display.py gop 1            # 查看/提示如何把 GOP 改成 1

可选参数（追加在命令后面）：
    --url rtsp://127.0.0.1:8558/live/plate     # 自定义流地址
    --decoder mppvideodec|avdec_h264           # 手动指定解码器，默认自动挑选
    --fast                                     # 免色彩转换直送 kmssink（更快，兼容性略低）
    --kill-desktop                             # 自动杀掉占用 DRM 的 X/Wayland 桌面
    --debug                                    # 打开 gst 调试日志（日志会变很大）
    --connector-id N                           # 指定输出到哪个屏（默认自动，可能跑到 DSI 屏）
                                               #   查 id：modetest -c ；HDMI-A-1 常见 175，DSI-1 常见 191
"""

import paramiko
import sys
import time

BOARD_IP = "192.168.137.100"
BOARD_USER = "linaro"
BOARD_PASS = "linaro"
LOG = "/tmp/hdmi_display.log"
DEFAULT_URL = "rtsp://127.0.0.1:8558/live/plate"


# ----------------------------------------------------------------- SSH 基础
def connect():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(BOARD_IP, username=BOARD_USER, password=BOARD_PASS, timeout=15)
    return ssh


def run(ssh, cmd, timeout=20):
    _, out, err = ssh.exec_command(cmd, timeout=timeout)
    return out.read().decode(errors="replace").strip(), err.read().decode(errors="replace").strip()


def has(ssh, cmd):
    """命令返回码为 0 则 True"""
    _, out, _ = ssh.exec_command(cmd + " >/dev/null 2>&1; echo $?", timeout=20)
    return out.read().decode().strip() == "0"


# ----------------------------------------------------------------- 管道构造
def rtsp_props(ssh):
    """探测 rtspsrc 实际支持的属性名。

    老版本 gstreamer 没有 buffer-mode / do-retransmission 等属性，写进去会让
    整条管道直接协商失败。这里先探测再拼装，保证跨镜像可用。
    """
    o, _ = run(ssh, "gst-inspect-1.0 rtspsrc 2>/dev/null | "
                    "grep -oE '^  [a-z0-9-]+' | tr -d ' '")
    return set(x for x in o.split("\n") if x)


def build_pipeline(ssh, url, width, height, dec, fast=False, connector_id=None):
    """低延迟管道。

    关键参数的作用：
      latency=0                rtpjitterbuffer 目标延迟归零
      buffer-mode=0            关闭 jitterbuffer 缓冲模式（NONE），不再攒帧
      do-retransmission=false  丢包不请求重传，宁可花一帧也不等一个 RTT
      ntp-sync=false           不做 NTP 时间戳对齐，避免等待
      drop-on-latency=true     来不及就丢帧，而不是排队
      queue leaky=downstream   队列只留 1 帧，下游慢时丢最旧的帧，杜绝堆积
      kmssink sync=false       不对齐时钟，拿到就显示
    """
    props = rtsp_props(ssh)
    opts = ["protocols=tcp", "latency=0"]
    for name, val in (("drop-on-latency", "true"),
                      ("buffer-mode", "0"),
                      ("do-retransmission", "false"),
                      ("ntp-sync", "false")):
        if name in props:
            opts.append(f"{name}={val}")
    opts += ["retry=5", "timeout=5000000"]

    head = f"rtspsrc location={url} " + " ".join(opts)
    q = "queue leaky=downstream max-size-buffers=1 max-size-bytes=0 max-size-time=0"

    # 板子同时接着 HDMI 和 MIPI DSI 屏时，kmssink 默认挑第一个 connected 的
    # connector，画面可能跑到另一块屏上。用 --connector-id 显式指定。
    # 查 id：modetest -c   第一列即 connector-id
    sink = "kmssink sync=false async=false"
    if connector_id is not None:
        sink = f"kmssink connector-id={connector_id} sync=false async=false"

    if fast:
        # 解码器输出直接喂 kmssink，省掉 videoconvert/videoscale 的整帧 CPU 转换。
        # 需要 kmssink 支持解码器输出格式（Rockchip 上通常是 NV12）。
        tail = f"{dec} ! {q} ! {sink}"
    else:
        tail = (f"{dec} ! {q} ! videoconvert ! videoscale "
                f"! video/x-raw,width={width},height={height},format=BGRx "
                f"! {sink}")

    return f"gst-launch-1.0 -e {head} ! rtph264depay ! h264parse ! {tail}"


def pick_decoder(ssh, forced=None):
    if forced:
        return forced
    for cand in ("mppvideodec", "rkvdec", "avdec_h264"):
        if has(ssh, f"gst-inspect-1.0 {cand}"):
            return cand
    return "avdec_h264"


# ----------------------------------------------------------------- 自检
def current_gop(ssh):
    """读主进程命令行里的 --zlm-gop"""
    o, _ = run(ssh, "pid=$(pgrep -f rknn_browser | head -1); "
                    "if [ -n \"$pid\" ]; then tr '\\0' ' ' < /proc/$pid/cmdline; fi")
    if "--zlm-gop" in o:
        parts = o.split()
        i = parts.index("--zlm-gop")
        if i + 1 < len(parts):
            return parts[i + 1]
    return None


def do_check(ssh, quiet=False):
    info = {}

    # 1. DRM 连接器状态
    o, _ = run(ssh, "for f in /sys/class/drm/card0-*/status; do echo \"$(basename $(dirname $f)) $(cat $f)\"; done")
    info["connectors"] = [ln.split() for ln in o.split("\n") if ln.strip()]

    # 2. 是否有桌面占着 DRM master
    o, _ = run(ssh, "ps aux | grep -E 'Xorg|weston|lightdm|gdm|sddm' | grep -v grep")
    info["desktop"] = o

    # 3. GStreamer 插件
    info["kmssink"] = has(ssh, "gst-inspect-1.0 kmssink")
    info["mppdec"] = has(ssh, "gst-inspect-1.0 mppvideodec")
    info["softdec"] = has(ssh, "gst-inspect-1.0 avdec_h264")
    info["gst"] = has(ssh, "which gst-launch-1.0")

    # 4. 主进程与 RTSP 端口
    o, _ = run(ssh, "ps aux | grep rknn_browser | grep -v grep")
    info["app"] = bool(o)
    o, _ = run(ssh, "sudo ss -lntp 2>/dev/null | grep -E ':8558|:8084'")
    info["ports"] = o

    # 5. GOP —— 方案 A 延迟的最大单项来源
    info["gop"] = current_gop(ssh)

    if quiet:
        return info

    print("== HDMI 本地显示 · 上板自检 ==")
    print("\n[1] DRM 连接器（connected = 显示器已插好）")
    if info["connectors"]:
        for name, st in info["connectors"]:
            print(f"    {name:26s} {st}")
    else:
        print("    (读不到 card0-*，检查内核 DRM 驱动)")

    print("\n[2] 桌面服务（占着 DRM master 会让 kmssink 失败）")
    print("    " + (info["desktop"].replace("\n", "\n    ") if info["desktop"] else "无（很好）"))

    print("\n[3] GStreamer")
    print(f"    gst-launch-1.0   : {'OK' if info['gst'] else '缺失 -> sudo apt install gstreamer1.0-tools'}")
    print(f"    kmssink          : {'OK' if info['kmssink'] else '缺失 -> 装 gstreamer1.0-plugins-bad'}")
    print(f"    mppvideodec(硬解): {'OK' if info['mppdec'] else '无 -> 将退化为软解 avdec_h264'}")

    print("\n[4] 主程序 / 流地址")
    print(f"    rknn_browser 进程 : {'运行中' if info['app'] else '未运行 -> 先 python v8_ctrl.py 启动'}")
    print(f"    监听端口          :\n    {info['ports'] or '未监听 8558/8084'}")

    print("\n[5] 编码 GOP（影响延迟最大的单项）")
    g = info["gop"]
    if g is None:
        print("    未检测到 --zlm-gop（进程没跑，或用了旧参数）")
    elif g == "1":
        print(f"    --zlm-gop {g}  -> 全 I 帧，GOP 缓冲已消除（好）")
    else:
        print(f"    --zlm-gop {g}  -> 25fps 下约 {int(1000 * int(g) / 25)} ms 的 GOP 缓冲！")
        print("    建议改成 1：v8_start.py 里 --zlm-gop 10 改成 --zlm-gop 1 后重启主程序")

    # 结论
    print("\n== 结论 ==")
    problems = []
    if not any(st == "connected" for _, st in info["connectors"]):
        problems.append("没有检测到 connected 的显示器，插好 HDMI 再试")
    if not info["gst"] or not info["kmssink"]:
        problems.append("GStreamer / kmssink 不可用，需离线安装插件包")
    if not info["app"]:
        problems.append("识别主进程没跑，先把它拉起来（显示器上才有画面）")
    if info["desktop"]:
        problems.append("桌面服务占用 DRM，启动时加 --kill-desktop")
    if info["gop"] not in (None, "1"):
        problems.append(f"GOP={info['gop']} 偏大，是延迟主因，建议改 1")
    if problems:
        for p in problems:
            print(f"  [ ] {p}")
    else:
        print("  [x] 全部就绪，可以 python v8_hdmi_display.py start")
    return info


# ----------------------------------------------------------------- 起停
def kill_desktop(ssh):
    print("  · 释放 DRM：停桌面服务")
    run(ssh, "sudo systemctl stop lightdm gdm sddm 2>/dev/null; "
             "sudo pkill -9 -f Xorg 2>/dev/null; "
             "sudo pkill -9 -f weston 2>/dev/null; sleep 1; echo done", timeout=30)


def do_start(ssh, url, width, height, decoder=None, do_kill_desktop=False,
             debug=False, fast=False, connector_id=None):
    # 先收掉旧实例，避免抢同一个 CRTC
    run(ssh, "sudo pkill -9 -f gst-launch-1.0 2>/dev/null; sleep 1; echo done", timeout=20)

    if do_kill_desktop:
        kill_desktop(ssh)

    dec = pick_decoder(ssh, decoder)
    print(f"  解码器: {dec}{'（硬件解码）' if dec in ('mppvideodec', 'rkvdec') else '（软件解码，注意 CPU）'}")

    gop = current_gop(ssh)
    if gop and gop != "1":
        print(f"  提示: 主程序 --zlm-gop {gop}，25fps 下约 {int(1000 * int(gop) / 25)} ms GOP 缓冲")
        print("        这是当前延迟的最大来源，改 v8_start.py 的 --zlm-gop 1 并重启主程序")

    pipeline = build_pipeline(ssh, url, width, height, dec, fast, connector_id)

    if debug:
        pipeline = pipeline.replace("gst-launch-1.0 -e", "gst-launch-1.0 --gst-debug-level=3 -e", 1)

    start_cmd = f"sudo -E setsid {pipeline} </dev/null >{LOG} 2>&1 &"

    print("  低延迟管道:")
    print("    " + pipeline)
    ssh.exec_command(start_cmd, timeout=8)
    time.sleep(4)

    o, _ = run(ssh, "ps aux | grep gst-launch-1.0 | grep -v grep")
    if o:
        print("[OK] HDMI 显示管道已运行")
        first = o.split("\n")[0].split()
        print(f"     pid={first[1] if len(first) > 1 else '?'}")
    else:
        print("[FAIL] 管道没起来，看日志：")
        o, _ = run(ssh, f"tail -30 {LOG}")
        print(o)
        if fast:
            print("  （--fast 模式依赖 kmssink 直接接受解码器输出格式；"
                  "协商失败就去掉 --fast 重试）")
        return

    time.sleep(2)
    o, _ = run(ssh, f"grep -iE 'error|warn|fail' {LOG} | head -15")
    if o:
        print("\n  日志告警/错误：")
        for ln in o.split("\n"):
            print("    " + ln.strip())


def do_stop(ssh):
    run(ssh, "sudo pkill -9 -f gst-launch-1.0 2>/dev/null; sleep 1; echo done", timeout=20)
    print("已停止 HDMI 显示（主程序的推流不受影响）")


def do_status(ssh):
    o, _ = run(ssh, "ps aux | grep gst-launch-1.0 | grep -v grep")
    print("[RUNNING]\n" + o if o else "[STOPPED]")
    o, _ = run(ssh, f"tail -20 {LOG} 2>/dev/null")
    if o:
        print("\n最近日志：")
        for ln in o.split("\n"):
            print("  " + ln)


def do_test(ssh, width, height, connector_id=None):
    """不依赖主进程，直接给 HDMI 打彩条：用来判断是"显示器通路"坏还是"流"坏"""
    run(ssh, "sudo pkill -9 -f gst-launch-1.0 2>/dev/null; sleep 1; echo done", timeout=20)
    print("  打 10 秒彩条，看显示器有没有画面……")
    sink = "kmssink sync=false"
    if connector_id is not None:
        sink = f"kmssink connector-id={connector_id} sync=false"
    pipeline = (
        f"gst-launch-1.0 videotestsrc num-buffers=250 "
        f"! video/x-raw,width={width},height={height} "
        f"! {sink}"
    )
    ssh.exec_command(f"sudo -E {pipeline} </dev/null >{LOG} 2>&1", timeout=25)
    o, _ = run(ssh, f"tail -15 {LOG}")
    print(o if o else "(无输出)")


def do_gop(ssh, want=None):
    """检查 / 提示如何调整 GOP"""
    g = current_gop(ssh)
    print(f"  当前主程序 --zlm-gop = {g or '(未检测到，主进程可能没跑)'}")
    print("\n  GOP 对延迟的影响（输出 25fps）：")
    for v in (10, 4, 2, 1):
        mark = "  <== 当前" if g == str(v) else ""
        print(f"    --zlm-gop {v:<2d} -> 约 {int(1000 * v / 25):3d} ms GOP 缓冲{mark}")
    print("\n  改法：编辑 v8_start.py，把")
    print("        f\"--infer-fps 1000 --output-fps 25 --zlm-gop 10 \"")
    print("    改成")
    print("        f\"--infer-fps 1000 --output-fps 25 --zlm-gop 1 --zlm-bitrate 8000000 \"")
    print("    （GOP=1 是全 I 帧，码率会涨，所以同步把 bitrate 提到 8 Mbps）")
    print("    然后 python v8_start.py stop && python v8_start.py start 重启主程序。")
    if want:
        print(f"\n  你要求的目标值：{want}")


# ----------------------------------------------------------------- 入口
def parse_args(argv):
    a = {
        "cmd": argv[0] if argv else "check",
        "url": DEFAULT_URL,
        "w": 1280,
        "h": 720,
        "decoder": None,
        "kill_desktop": False,
        "debug": False,
        "fast": False,
        "connector_id": None,
        "want_gop": None,
    }
    rest = argv[1:]
    nums = []
    i = 0
    while i < len(rest):
        tok = rest[i]
        if tok == "--kill-desktop":
            a["kill_desktop"] = True
        elif tok == "--debug":
            a["debug"] = True
        elif tok == "--fast":
            a["fast"] = True
        elif tok == "--url" and i + 1 < len(rest):
            a["url"] = rest[i + 1]
            i += 1
        elif tok == "--decoder" and i + 1 < len(rest):
            a["decoder"] = rest[i + 1]
            i += 1
        elif tok == "--connector-id" and i + 1 < len(rest):
            a["connector_id"] = int(rest[i + 1])
            i += 1
        elif tok.isdigit():
            nums.append(int(tok))
        i += 1
    if len(nums) >= 2:
        a["w"], a["h"] = nums[0], nums[1]
    elif len(nums) == 1 and a["cmd"] == "gop":
        a["want_gop"] = nums[0]
    return a


def main():
    a = parse_args(sys.argv[1:])
    ssh = connect()
    try:
        {
            "check": lambda: do_check(ssh),
            "start": lambda: do_start(ssh, a["url"], a["w"], a["h"], a["decoder"],
                                      a["kill_desktop"], a["debug"], a["fast"],
                                      a["connector_id"]),
            "stop": lambda: do_stop(ssh),
            "status": lambda: do_status(ssh),
            "test": lambda: do_test(ssh, a["w"], a["h"], a["connector_id"]),
            "gop": lambda: do_gop(ssh, a["want_gop"]),
        }.get(a["cmd"], lambda: do_check(ssh))()
    finally:
        ssh.close()


if __name__ == "__main__":
    main()
