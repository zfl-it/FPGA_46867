#!/usr/bin/env python3
"""Push V8_720P source files to RK3568 board and build."""

import paramiko, sys, os, time

BOARD_IP = "192.168.137.100"
BOARD_USER = "linaro"
BOARD_PASS = "linaro"
BOARD_V8 = "/userdata/RK3568_V8_720P"
LOCAL_V7 = os.path.dirname(os.path.abspath(__file__))

def run(ssh, cmd, timeout=120):
    _, out, err = ssh.exec_command(cmd, timeout=timeout)
    return out.read().decode().strip(), err.read().decode().strip()


def run_long(ssh, cmd, log="/tmp/build_v8.log", limit=1800, poll=15):
    """长时间编译：后台执行 + 轮询进度，避免 exec_command 读超时。

    paramiko 的 exec_command timeout 是"读超时"，首次编译几分钟无输出必然触发
    TimeoutError。改为 nohup 后台跑、写日志、这边定期 tail。
    """
    run(ssh, "rm -f %s" % log)
    # setsid + </dev/null + disown：彻底脱离 SSH 会话，否则后台进程抓着
    # channel 不放，exec_command 的 read() 永远等不到 EOF
    wrapped = ("setsid bash -c '%s' </dev/null >/dev/null 2>&1 & disown; echo started"
               % cmd.replace("'", "'\\''"))
    try:
        run(ssh, wrapped, timeout=30)
    except Exception:
        pass   # 启动命令本身超时无所谓，靠轮询日志判断进度
    t0 = time.time()
    print("  编译中（后台，日志 %s）..." % log)
    while time.time() - t0 < limit:
        time.sleep(poll)
        tail, _ = run(ssh, "tail -2 %s 2>/dev/null" % log)
        done, _ = run(ssh, "grep -c BUILD_EXIT %s 2>/dev/null || echo 0" % log)
        if tail:
            print("    |", tail.splitlines()[-1][:150])
        if done.strip().isdigit() and int(done.strip()) > 0:
            break
    out, _ = run(ssh, "cat %s 2>/dev/null | tail -40" % log, timeout=120)
    return out, ""

def main():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(BOARD_IP, username=BOARD_USER, password=BOARD_PASS)
    print("Connected to board")

    # Create directory structure
    cmds = f"""
mkdir -p {BOARD_V8}/cpp/build {BOARD_V8}/cpp/ZLMediaKit/release/linux/Release {BOARD_V8}/cpp/ZLMediaKit/include {BOARD_V8}/model
echo DONE
"""
    run(ssh, cmds)
    print("Directories ready")

    # SFTP push all files
    sftp = ssh.open_sftp()

    # Core source files
    core_files = [
        'main.cc', 'main.c', 'CMakeLists.txt',
        'zlm_streamer.cc', 'zlm_streamer.h',
        'mpp_h264_encoder.cc', 'mpp_h264_encoder.h',
        'pixel_convert.cc', 'pixel_convert.h',
        'adas_detector.cc', 'adas_detector.h',
        'pango_fun.cpp', 'pango_fun.h',
        'pcie_reader_lib.c', 'pcie_reader_lib.h', 'pango_pci.h',
    ]
    for f in core_files:
        local = os.path.join(LOCAL_V7, 'cpp', f)
        remote = f'{BOARD_V8}/cpp/{f}'
        sftp.put(local, remote)
    print(f"Pushed {len(core_files)} core source files")

    # ZLM include files
    zlm_inc = os.path.join(LOCAL_V7, 'cpp', 'ZLMediaKit', 'include')
    for f in os.listdir(zlm_inc):
        local = os.path.join(zlm_inc, f)
        remote = f'{BOARD_V8}/cpp/ZLMediaKit/include/{f}'
        sftp.put(local, remote)
    print(f"Pushed ZLM include files")

    # ZLM lib (WebRTC version)
    lib_local = os.path.join(LOCAL_V7, 'cpp', 'ZLMediaKit', 'release', 'linux', 'Release', 'libmk_api.so')
    lib_remote = f'{BOARD_V8}/cpp/ZLMediaKit/release/linux/Release/libmk_api.so'
    sftp.put(lib_local, lib_remote)
    print(f"Pushed libmk_api.so (7.0MB WebRTC)")

    # ZLM config.ini
    cfg_local = os.path.join(LOCAL_V7, 'cpp', 'ZLMediaKit', 'config.ini')
    cfg_remote = f'{BOARD_V8}/cpp/ZLMediaKit/config.ini'
    sftp.put(cfg_local, cfg_remote)
    print(f"Pushed config.ini")

    # Model files：只推板端当前真正加载的那一个（从 v8_start.py 的 ADAS_MODEL 解析），
    # 板端 model/ 已按"只留当前模型"清理过，这里若整目录推会把旧版/车牌模型全推回去。
    # .onnx/.pt 是转换中间产物（adas12 的 onnx 就 45MB），一律不推。
    import re as _re
    model_dir = os.path.join(LOCAL_V7, 'model')
    adas_name = 'adas12n_736x1280_i8_scaled.rknn'
    try:
        _src = open(os.path.join(LOCAL_V7, 'v8_start.py'), encoding='utf-8').read()
        _m = _re.search(r'ADAS_MODEL\s*=\s*f"\{MODEL\}/([^"]+)"', _src)
        if _m:
            adas_name = _m.group(1)
    except Exception as _e:
        print(f"[WARN] 解析 ADAS_MODEL 失败，用默认 {adas_name}: {_e}")
    local_model = os.path.join(model_dir, adas_name)
    if os.path.exists(local_model):
        sftp.put(local_model, f'{BOARD_V8}/model/{adas_name}')
        print(f"Pushed model {adas_name}")
    else:
        print(f"[WARN] 本地模型不存在，跳过推送: {local_model}")

    sftp.close()

    # Build
    print("\n=== BUILDING V8_720P ===")
    build = (
        f"cd {BOARD_V8}/cpp/build && "
        f"cmake .. -DCMAKE_BUILD_TYPE=Release -DHAVE_DIRECT_FREETYPE=ON "
        f"-DHAVE_OPENCV_FREETYPE=OFF -DHAVE_DRM_ROI=ON "
        f"-DOpenCV_DIR=/usr/lib/aarch64-linux-gnu/cmake/opencv4 "
        f"-DLIBRKNNRT=/userdata/RKSDK/rknn-toolkit2/rknpu2/runtime/Linux/librknn_api/aarch64/librknnrt.so "
        f"-DLIBRKNNRT_INCLUDES=/userdata/RKSDK/rknn-toolkit2/rknpu2/runtime/Linux/librknn_api/include "
        f"-DZLM_SDK_DIR={BOARD_V8}/cpp/ZLMediaKit "
        f"> /tmp/build_v8.log 2>&1 && make -j2 >> /tmp/build_v8.log 2>&1; "
        f"echo BUILD_EXIT=$? >> /tmp/build_v8.log"
    )
    # 首次编译通常 5~15 分钟，前台 exec_command 会读超时，改后台 + 轮询
    out, err = run_long(ssh, build)
    if out:
        print(out[-3000:] if len(out) > 3000 else out)
    if err:
        print("STDERR:", err[-500:] if len(err) > 500 else err)

    # Verify
    out, _ = run(ssh, f"ls -la {BOARD_V8}/cpp/build/rknn_browser_hdmi_lpr_demo 2>/dev/null")
    if out:
        print(f"\n=== BUILD SUCCESS ===\n{out}")
    else:
        print("\n=== BUILD FAILED ===")
        ssh.close()
        return 1

    ssh.close()
    return 0

if __name__ == "__main__":
    sys.exit(main())
