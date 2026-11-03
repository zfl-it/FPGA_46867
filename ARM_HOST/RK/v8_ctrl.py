#!/usr/bin/env python3
"""V8_720P program control: start / stop / restart / status.

Usage:
  python v8_ctrl.py start              # 6-step startup with FPGA
  python v8_ctrl.py stop               # Stop all processes
  python v8_ctrl.py restart            # Restart
  python v8_ctrl.py status             # Show running status
  python v8_ctrl.py log                # Tail run.log
  python v8_ctrl.py debug              # Driver/hardware debug
"""

import paramiko, time, sys, re

# ⚠️ 2026-10-27: 本脚本启动的是【旧车牌+OCR 模式】（plate_obb_corner + gop10），
# 会把 ADAS 模式进程顶掉（屏幕左上角变 "No plate"、串口停止下发）。
# ADAS 自动驾驶链路请用：v8_start.py start 或 start_auto_drive.bat
if len(sys.argv) > 1 and sys.argv[1] in ("start", "restart"):
    print("=" * 60)
    print("⚠️  警告: v8_ctrl.py 启动的是旧【车牌+OCR】模式，不是 ADAS！")
    print("    启动 ADAS 自动驾驶链路请改用:")
    print("      v8_start.py start   或   start_auto_drive.bat")
    print("=" * 60)
    ans = input("仍要继续启动车牌模式? [y/N] ").strip().lower()
    if ans != "y":
        print("已取消。")
        sys.exit(0)

BOARD_IP = "192.168.137.100"
BOARD_USER = "linaro"
BOARD_PASS = "linaro"

# V8_720P — single resolution, all paths self-contained
BOARD_V8 = "/userdata/RK3568_V8_720P"
CFG = {
    "board_dir": BOARD_V8,
    "driver_ko": f"{BOARD_V8}/pango_pci_driver.ko",
    "build_dir": f"{BOARD_V8}/cpp/build",
    "model_dir": f"{BOARD_V8}/model",
    "zlm_dir": f"{BOARD_V8}/cpp/ZLMediaKit",
    "width": 1280,
    "height": 720,
    # Raw per-frame OBB geometry dump for jitter measurement ("" = off).
    "obb_jitter_log": "/tmp/obb_jitter.log",
}

ERROR_PATTERNS = [
    (r"segmentation fault|SIGSEGV",        "程序崩溃 (Segfault) — 空指针或内存越界"),
    (r"cannot open|No such file|not found", "文件缺失 — 检查模型/驱动/字体路径"),
    (r"RKNN runtime: api=2\.3\.0\b",        "librknnrt 2.3.0 — OBB 模型必段错误! LD_LIBRARY_PATH 必须指向 RKSDK 2.3.2 (板端坑 #1)"),
    (r"RKNN.*fail|rknn_init.*error",        "NPU 模型加载失败 — 检查 .rknn 文件"),
    (r"MPP.*fail|mpp_create.*error",        "MPP 硬编码初始化失败"),
    (r"ZLMediaKit.*fail|mk_env_init",       "ZLMediaKit 初始化失败 — 检查 libmk_api.so"),
    (r"PCIe.*fail|BAR0.*fail|mmap.*fail",   "PCIe/FPGA 通信失败"),
    (r"Permission denied",                   "权限不足 — 需要用 sudo"),
    (r"Cannot allocate memory|OOM",         "内存不足 — 检查 swap 或关闭其他进程"),
]


def ssh_connect():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        ssh.connect(BOARD_IP, username=BOARD_USER, password=BOARD_PASS, timeout=15)
        return ssh
    except Exception as e:
        print(f"[FAIL] 无法连接到 {BOARD_IP}: {e}")
        sys.exit(1)


def run(ssh, cmd, timeout=30):
    _, out, err = ssh.exec_command(cmd, timeout=timeout)
    o = out.read().decode().strip()
    e = err.read().decode().strip()
    return o, e


def ok(msg):
    print(f"  [OK] {msg}")


def warn(msg):
    print(f"  [WARN] {msg}")


def fail(msg):
    print(f"  [FAIL] {msg}")


def scan_errors(log_text):
    found = []
    for pattern, desc in ERROR_PATTERNS:
        if re.search(pattern, log_text, re.IGNORECASE):
            found.append(f"    -> {desc}")
    return found


def do_stop(ssh):
    """Kill processes, release ports, unload driver."""
    print("[1/3] 停止 rknn_browser 进程...")
    run(ssh, "sudo pkill -9 -f rknn_browser 2>/dev/null; sleep 2; echo done")

    out, _ = run(ssh, "ps aux | grep rknn_browser | grep -v grep")
    if out:
        warn(f"仍有残留进程: {out[:120]}")
        run(ssh, "sudo pkill -9 -f rknn_browser 2>/dev/null; sleep 1")
    else:
        ok("无残留进程")

    print("[2/3] 释放端口...")
    run(ssh, "sudo fuser -k 8080/tcp 8084/tcp 8558/tcp 1939/tcp 8004/udp 2>/dev/null; sleep 1; echo done")

    print("[3/3] 卸载驱动...")
    run(ssh, "sudo rmmod pango_pci_driver 2>/dev/null; sleep 1; echo done")
    print("停止完成\n")


def do_start(ssh):
    """Standard 6-step startup. Returns True on success, False on failure."""
    B = CFG["board_dir"]
    D = CFG["driver_ko"]
    W = CFG["width"]
    H = CFG["height"]
    M = CFG["model_dir"]
    BUILD = CFG["build_dir"]
    ZLM = CFG["zlm_dir"]

    # ---- Step 1: Clean ----
    print("=" * 55)
    print("  STEP 1: 清理环境")
    print("=" * 55)
    run(ssh, "sudo pkill -9 -f rknn_browser 2>/dev/null; sleep 2; echo done")
    run(ssh, "sudo fuser -k 8080/tcp 8084/tcp 8558/tcp 1939/tcp 8004/udp 2>/dev/null; sleep 1; echo done")
    run(ssh, "sudo rmmod pango_pci_driver 2>/dev/null; sleep 1; echo done")
    ok("清理完成")

    # ---- Step 2: Load driver ----
    print("\n" + "=" * 55)
    print("  STEP 2: 加载驱动")
    print("=" * 55)

    # Check driver file
    d_exists, _ = run(ssh, f"ls -la {D} 2>/dev/null")
    if not d_exists:
        # Try to copy from V7_720P (known-good v2.0.0-ring4, same FPGA)
        # NOTE: V5_1080P has no driver file on the board — do not fall back there.
        print("  驱动文件不存在，从 V7_720P 复制...")
        run(ssh, f"cp /userdata/RK3568_V7_720P/pango_pci_driver.ko {D}")
        d_exists, _ = run(ssh, f"ls -la {D} 2>/dev/null")
        if not d_exists:
            # 换板后 V7 目录也不存在 —— 从 Windows 工程 driver/ 目录 SFTP 推过去。
            # 驱动源码已在工程内（driver/*.c），必要时可在板端用内核头文件重编。
            import os as _os
            _ko = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)),
                                "driver", "pango_pci_driver.ko")
            if _os.path.exists(_ko):
                print("  板端无备份，从 Windows 工程 driver/ 推送...")
                try:
                    sftp = ssh.open_sftp()
                    sftp.put(_ko, D)
                    sftp.close()
                except Exception as e:
                    print(f"  推送失败: {e}")
                d_exists, _ = run(ssh, f"ls -la {D} 2>/dev/null")
            if not d_exists:
                fail(f"无法获取驱动文件: {D}")
                print("  提示：驱动源码在 工程/driver/，可在板端 make 重新编译")
                return False

    out, err = run(ssh, f"sudo insmod {D} 2>&1")
    if err and "File exists" in err:
        run(ssh, "sudo rmmod pango_pci_driver; sleep 1")
        out, err = run(ssh, f"sudo insmod {D} 2>&1")
    if err and err.strip():
        fail(f"驱动加载失败: {err.strip()[:200]}")
        return False

    out, _ = run(ssh, "lsmod | grep pango_pci_driver")
    if "pango_pci_driver" in out:
        ok("驱动已加载")
    else:
        warn("驱动未在 lsmod 中找到")

    # ---- Step 3: Verify PCIe ----
    print("\n" + "=" * 55)
    print("  STEP 3: 验证 PCIe")
    print("=" * 55)
    enable, _ = run(ssh, "cat /sys/bus/pci/devices/0002:21:00.0/enable")
    print(f"  PCIe enable = {enable}")
    if enable != "1":
        fail("PCIe 设备未使能!")
        print("  尝试 PCI rescan...")
        run(ssh, "sudo rmmod pango_pci_driver 2>/dev/null; sleep 1")
        run(ssh, "echo 1 | sudo tee /sys/bus/pci/devices/0002:21:00.0/remove > /dev/null 2>&1; sleep 1")
        run(ssh, "echo 1 | sudo tee /sys/bus/pci/rescan > /dev/null 2>&1; sleep 2")
        run(ssh, f"sudo insmod {D} 2>&1")
        enable, _ = run(ssh, "cat /sys/bus/pci/devices/0002:21:00.0/enable")
        if enable != "1":
            fail("PCI rescan 也失败了，需断电重启")
            return False
        ok("PCI rescan 恢复成功")
    else:
        ok("PCIe 已使能")

    bar0, _ = run(ssh, "sudo busybox devmem 0xf0200140 32")
    print(f"  BAR0[0x140] = {bar0}")
    if bar0.upper() in ("0XFFFFFFFF", "FFFFFFFF"):
        fail("FPGA 无响应 (BAR0=0xFFFFFFFF)")
        print("  尝试 PCI rescan 恢复...")
        run(ssh, "sudo rmmod pango_pci_driver 2>/dev/null; sleep 1")
        run(ssh, "echo 1 | sudo tee /sys/bus/pci/devices/0002:21:00.0/remove > /dev/null 2>&1; sleep 1")
        run(ssh, "echo 1 | sudo tee /sys/bus/pci/rescan > /dev/null 2>&1; sleep 2")
        run(ssh, f"sudo insmod {D} 2>&1")
        bar0, _ = run(ssh, "sudo busybox devmem 0xf0200140 32")
        if bar0.upper() in ("0XFFFFFFFF", "FFFFFFFF"):
            fail("PCI rescan 未能恢复")
            return False
        ok("PCI rescan 成功")
    ok("FPGA 正常响应")

    # ---- Step 4: Performance ----
    print("\n" + "=" * 55)
    print("  STEP 4: 性能模式")
    print("=" * 55)
    for cpu in ["cpu0", "cpu1", "cpu2", "cpu3"]:
        run(ssh, f"sudo sh -c 'echo performance > /sys/devices/system/cpu/{cpu}/cpufreq/scaling_governor' 2>/dev/null || true")
    run(ssh, "sudo sh -c 'echo performance > /sys/class/devfreq/fde40000.npu/governor' 2>/dev/null || true")
    run(ssh, "sudo sh -c 'echo performance > /sys/class/devfreq/fde60000.gpu/governor' 2>/dev/null || true")
    run(ssh, "sudo sh -c 'echo performance > /sys/class/devfreq/dmc/governor' 2>/dev/null || true")

    cpu_gov, _ = run(ssh, "cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor")
    npu_gov, _ = run(ssh, "cat /sys/class/devfreq/fde40000.npu/governor")
    ok(f"CPU={cpu_gov}  NPU={npu_gov}")

    # ---- Step 5: Launch ----
    print("\n" + "=" * 55)
    print("  STEP 5: 启动程序 (V8_720P)")
    print("=" * 55)

    jlog = CFG.get("obb_jitter_log", "")
    jlog_arg = f"--obb-jitter-log {jlog} " if jlog else ""

    # --test-pattern: 用板端生成的彩条+移动方块代替 FPGA 采集，
    # 用于在没有 HDMI 源的情况下验证「编码 -> ZLM -> 网页播放」整条显示链路是否正常。
    tp_arg = "--test-pattern " if any(a == "--test-pattern" for a in sys.argv[2:]) else ""

    start_cmd = (
        f"cd {BUILD} && "
        f"export LD_LIBRARY_PATH=/userdata/RKSDK/rknn-toolkit2/rknpu2/runtime/Linux/librknn_api/aarch64:{ZLM}/release/linux/Release:$LD_LIBRARY_PATH && "
        f"sudo setsid ./rknn_browser_hdmi_lpr_demo "
        f"--pcie-device /dev/pango_pci_driver "
        f"--jpeg-udp-dest 192.168.137.1:9999 "
        f"--width {W} --height {H} --fps 25 "
        f"--infer-fps 1000 --output-fps 25 "
        f"--conf 0.25 --nms 0.4 "
        f"{tp_arg}"
        f"{jlog_arg}"
        f"--detect-model {M}/plate_obb_corner_i8.rknn "
        f"--ocr-backend lprnet "
        f"--recog-model {M}/lprnet_color_fp16.rknn "
        f"--recog-chars {M}/lprnet_chars.txt "
        f"--font {M}/simhei.ttf "
        f"--zlm-sdk-dir {ZLM} "
        f"--zlm-http-port 8084 --zlm-rtsp-port 8558 "
        f"--zlm-rtmp-port 1939 --zlm-rtc-port 8004 "
        f"--zlm-gop 10 --port 8080 "
        f"</dev/null >run.log 2>&1 &"
    )

    ssh.exec_command(start_cmd, timeout=10)
    time.sleep(2)

    # Check process
    pid = None
    out, _ = run(ssh, "ps aux | grep rknn_browser | grep -v grep")
    if out:
        try:
            pid = out.split()[1]
        except:
            pass
        ok(f"进程已启动 (PID={pid})")
    else:
        time.sleep(4)
        out, _ = run(ssh, "ps aux | grep rknn_browser | grep -v grep")
        if out:
            try:
                pid = out.split()[1]
            except:
                pass
            ok(f"进程已启动 (PID={pid}, 延迟)")
        else:
            fail("进程启动失败! 检查错误日志:")
            log_out, _ = run(ssh, f"tail -30 {BUILD}/run.log")
            print(f"\n  --- run.log (tail) ---")
            for line in log_out.split('\n'):
                print(f"  | {line}")
            print(f"  --- end ---\n")
            errors = scan_errors(log_out)
            if errors:
                print("  可能原因:")
                for e in errors:
                    print(e)
            else:
                print("  未识别到已知错误模式，请检查 run.log")
            return False

    # ---- Step 6: Crash detection ----
    print("\n" + "=" * 55)
    print("  STEP 6: 确认运行 + 崩溃检测")
    print("=" * 55)

    time.sleep(4)
    out, _ = run(ssh, f"ps -p {pid} -o pid= 2>/dev/null") if pid else ("", "")
    if not out:
        fail(f"进程启动后崩溃! (PID={pid} 已不存在)")
        log_out, _ = run(ssh, f"tail -40 {BUILD}/run.log")
        print(f"\n  --- 崩溃前日志 (tail) ---")
        for line in log_out.split('\n')[-30:]:
            print(f"  | {line}")
        print(f"  --- end ---\n")
        errors = scan_errors(log_out)
        if errors:
            print("  可能原因:")
            for e in errors:
                print(e)
        return False

    ok(f"进程稳定运行 (PID={pid})")

    # V8-specific: verify RKNN runtime is 2.3.2 (OBB segfaults on system 2.3.0)
    rt_out, _ = run(ssh, f"grep -m1 'RKNN runtime' {BUILD}/run.log 2>/dev/null")
    if "api=2.3.2" in rt_out:
        ok(f"librknnrt: {rt_out.strip()[:70]}")
    elif rt_out:
        warn(f"librknnrt 版本异常: {rt_out.strip()[:80]}")
        print("  OBB 模型在 2.3.0 上必段错误 — 确认 LD_LIBRARY_PATH 指向 RKSDK 2.3.2")
    else:
        warn("run.log 中未找到 'RKNN runtime' 行 — 模型可能未加载")

    # ---- Show key log ----
    print(f"\n  --- 最近日志 ---")
    log_out, _ = run(ssh, f"tail -30 {BUILD}/run.log")
    for line in log_out.split('\n'):
        if any(kw in line.lower() for kw in ['rtc', 'jpeg-udp', 'error', 'fail', 'bar0', 'capture', 'saved',
                                              'model latency', 'frame_delay', '8004', 'udp server', 'tcp server',
                                              'color=', 'webrtc', 'dma ring', 'rknn runtime']):
            print(f"  | {line.strip()[:130]}")

    print("\n" + "=" * 55)
    print("  [SUCCESS] V8_720P 启动成功!")
    print("=" * 55)
    print(f"  WebRTC:    http://{BOARD_IP}:8080/")
    print(f"  RTSP:      rtsp://{BOARD_IP}:8558/live/plate")
    print(f"  HTTP-FLV:  http://{BOARD_IP}:8084/live/plate.live.flv")
    print(f"  JPEG-UDP:  udp://{BOARD_IP}:9999 -> pc_receiver.py")
    print(f"  PID:       {pid}")
    return True


def do_status(ssh):
    """Show running status."""
    BUILD = CFG["build_dir"]
    W = CFG["width"]
    H = CFG["height"]

    print(f"板端: {BOARD_IP}  |  分辨率: {W}x{H}  |  V8_720P")

    out, _ = run(ssh, "ps aux | grep rknn_browser | grep -v grep")
    if not out:
        print("状态: STOPPED (未运行)")
        return

    parts = out.split()
    pid = parts[1] if len(parts) > 1 else "?"
    cpu = parts[2] if len(parts) > 2 else "?"
    mem = parts[3] if len(parts) > 3 else "?"
    runtime = parts[9] if len(parts) > 9 else "?"
    print(f"状态: RUNNING  PID={pid}  CPU={cpu}%  MEM={mem}%  运行时间={runtime}")

    out, _ = run(ssh, f"grep -oE 'Model latency [0-9.]+|frame_delay=[0-9]+|color=[A-Z]+|capture started|saved {W}' {BUILD}/run.log 2>/dev/null | tail -3")
    if out:
        print(f"指标: {out}")

    out, _ = run(ssh, "cat /sys/bus/pci/devices/0002:21:00.0/enable 2>/dev/null")
    print(f"PCIe: enable={out}")

    out, _ = run(ssh, "sudo busybox devmem 0xf0200140 32 2>/dev/null")
    print(f"FPGA: BAR0[0x140]={out}")


def do_log(ssh):
    BUILD = CFG["build_dir"]
    out, _ = run(ssh, f"tail -50 {BUILD}/run.log")
    print(out)


def do_debug(ssh):
    """Common driver/hardware debug operations."""
    D = CFG["driver_ko"]
    PCI_PATH = "/sys/bus/pci/devices/0002:21:00.0"

    print("=" * 55)
    print("  [1] PCIe 状态")
    print("=" * 55)
    enable, _ = run(ssh, f"cat {PCI_PATH}/enable")
    bar0, _ = run(ssh, "sudo busybox devmem 0xf0200140 32")
    bar0_ctl, _ = run(ssh, "sudo busybox devmem 0xf0200000 32")
    vendor, _ = run(ssh, f"cat {PCI_PATH}/vendor 2>/dev/null")
    device, _ = run(ssh, f"cat {PCI_PATH}/device 2>/dev/null")
    print(f"  enable = {enable}")
    print(f"  vendor:device = {vendor}:{device}")
    print(f"  BAR0[0x000] CONTROL = {bar0_ctl}")
    print(f"  BAR0[0x140] STATUS  = {bar0}")
    if bar0.upper() in ("0XFFFFFFFF", "FFFFFFFF"):
        fail("FPGA 无响应 — BAR0 全 F")
    else:
        ok("FPGA 正常响应")

    print("\n" + "=" * 55)
    print("  [2] 驱动状态")
    print("=" * 55)
    out, _ = run(ssh, "lsmod | grep pango_pci_driver")
    if out:
        ok(f"驱动已加载: {out}")
    else:
        warn("驱动未加载")
    out, _ = run(ssh, f"ls -la {D}")
    if "No such file" not in out:
        ok(f"驱动文件存在")
    else:
        warn(f"驱动文件不存在: {D}")

    print("\n" + "=" * 55)
    print("  [3] 内核日志")
    print("=" * 55)
    out, _ = run(ssh, "dmesg | grep -iE 'pango|pci|fpga|bar0|error|fail' | tail -15")
    if out:
        for line in out.split('\n'):
            print(f"  | {line}")
    else:
        print("  (无相关日志)")

    while True:
        print("\n" + "-" * 45)
        print("  调试操作:")
        print("    r  重载驱动 (rmmod + insmod)")
        print("    p  PCI rescan 恢复")
        print("    s  发送 FPGA 启动信号")
        print("    b  读取 BAR0 全部寄存器")
        print("    d  最近 dmesg (30行)")
        print("    q  退出")
        print("-" * 45)
        choice = input("  >>> ").strip().lower()

        if choice == 'r':
            print("\n  重载驱动...")
            run(ssh, "sudo rmmod pango_pci_driver 2>/dev/null; sleep 1")
            o, e = run(ssh, f"sudo insmod {D} 2>&1")
            if e and "File exists" in e:
                run(ssh, "sudo rmmod pango_pci_driver; sleep 1")
                o, e = run(ssh, f"sudo insmod {D} 2>&1")
            if e and e.strip():
                fail(f"insmod 失败: {e[:200]}")
            else:
                ok("驱动重载完成")
            en, _ = run(ssh, f"cat {PCI_PATH}/enable")
            b0, _ = run(ssh, "sudo busybox devmem 0xf0200140 32")
            print(f"  PCIe enable = {en}")
            print(f"  BAR0[0x140] = {b0}")

        elif choice == 'p':
            print("\n  PCI rescan 恢复...")
            confirm = input("  确认? (y/N): ").strip().lower()
            if confirm != 'y':
                print("  取消")
                continue
            run(ssh, "sudo rmmod pango_pci_driver 2>/dev/null; sleep 1")
            run(ssh, "echo 1 | sudo tee /sys/bus/pci/devices/0002:21:00.0/remove > /dev/null 2>&1; sleep 1")
            run(ssh, "echo 1 | sudo tee /sys/bus/pci/rescan > /dev/null 2>&1; sleep 2")
            run(ssh, f"sudo insmod {D} 2>&1")
            en, _ = run(ssh, f"cat {PCI_PATH}/enable")
            b0, _ = run(ssh, "sudo busybox devmem 0xf0200140 32")
            print(f"  PCIe enable = {en}")
            print(f"  BAR0[0x140] = {b0}")
            if b0.upper() not in ("0XFFFFFFFF", "FFFFFFFF"):
                ok("PCI rescan 成功")
            else:
                fail("PCI rescan 未能恢复")

        elif choice == 's':
            print("\n  发送 FPGA 启动信号...")
            run(ssh, "sudo busybox devmem 0xf0200000 32 0xffffffe5")
            time.sleep(0.5)
            b0, _ = run(ssh, "sudo busybox devmem 0xf0200140 32")
            print(f"  BAR0[0x140] STATUS = {b0}")
            ok("启动信号已发送")

        elif choice == 'b':
            print("\n  BAR0 寄存器扫描:")
            for offset in [0x000, 0x004, 0x008, 0x00C, 0x100, 0x104, 0x108, 0x10C, 0x140, 0x144, 0x148, 0x14C]:
                addr = 0xf0200000 + offset
                val, _ = run(ssh, f"sudo busybox devmem 0x{addr:08x} 32")
                print(f"    BAR0[0x{offset:03x}] = {val}")

        elif choice == 'd':
            out, _ = run(ssh, "dmesg | tail -30")
            print(f"\n{out}")

        elif choice == 'q':
            print("  退出调试")
            break
        else:
            print("  无效选项")


# ============================================================
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    cmd = sys.argv[1].lower()
    print(f"[V8_720P] >>> {cmd}\n")

    ssh = ssh_connect()

    if cmd == "start":
        do_stop(ssh)
        success = do_start(ssh)
        if not success:
            print("\n[ABORT] 启动失败，请根据上面的错误信息排查。")
    elif cmd == "stop":
        do_stop(ssh)
    elif cmd == "restart":
        do_stop(ssh)
        time.sleep(1)
        success = do_start(ssh)
        if not success:
            print("\n[ABORT] 重启失败，请根据上面的错误信息排查。")
    elif cmd == "status":
        do_status(ssh)
    elif cmd == "log":
        do_log(ssh)
    elif cmd == "debug":
        do_debug(ssh)
    else:
        print(f"未知命令: {cmd}")
        print("用法: python v8_ctrl.py start | stop | restart | status | log | debug")

    ssh.close()
