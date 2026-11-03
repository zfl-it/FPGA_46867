#!/usr/bin/env python3
"""
RK3568 real-time clock display + V7 test-pattern launcher.
Shows the board's actual system time (ms precision) in a large window.
Use together with the test-pattern video to measure end-to-end latency:
  latency = video_GEN_TIME vs displayed_RK_time

Usage:
  python rk_clock.py              # Show RK clock only
  python rk_clock.py --launch     # Launch V7 test-pattern + show clock
  python rk_clock.py --stop       # Stop V7
"""

import tkinter as tk
import paramiko
import time
import threading
import sys

BOARD_IP = "192.168.137.100"
BOARD_USER = "linaro"
BOARD_PASS = "linaro"
BUILD = "/userdata/RK3568_V7_720P/cpp/build"
MODEL = "/userdata/RK3568_V7_720P/model"
V7 = "/userdata/RK3568_V7_720P"

class RkClock:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("RK3568 System Clock")
        self.root.geometry("500x200")
        self.root.configure(bg="#0a0a1a")
        self.root.resizable(False, False)

        self.rk_time = "--:--:--.---"
        self.latency_ms = 0
        self.running = True

        # Title
        tk.Label(self.root, text=f"RK3568 @ {BOARD_IP}",
                 font=("Consolas", 12, "bold"), fg="#888", bg="#0a0a1a").pack(pady=(20, 5))

        # Big clock
        self.clock_label = tk.Label(self.root, text="--:--:--.---",
                                     font=("Consolas", 56, "bold"),
                                     fg="#00ff88", bg="#0a0a1a")
        self.clock_label.pack(pady=5)

        # Info line
        self.info_label = tk.Label(self.root, text="connecting...",
                                    font=("Consolas", 9), fg="#555", bg="#0a0a1a")
        self.info_label.pack(pady=(0, 15))

        # Start background fetcher
        self.ssh = None
        self._connect()
        self._fetch_loop()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _connect(self):
        try:
            self.ssh = paramiko.SSHClient()
            self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            self.ssh.connect(BOARD_IP, username=BOARD_USER, password=BOARD_PASS, timeout=5)
            self.info_label.config(text="connected", fg="#00d2ff")
        except Exception as e:
            self.info_label.config(text=f"SSH error: {e}", fg="#ff4444")
            self.ssh = None

    def _fetch_loop(self):
        """Background thread: continuously fetch RK time."""
        def loop():
            while self.running:
                if self.ssh:
                    try:
                        t0 = time.perf_counter()
                        _, out, _ = self.ssh.exec_command(
                            "date '+%H:%M:%S.%3N'", timeout=2)
                        t = out.read().decode().strip()
                        self.latency_ms = (time.perf_counter() - t0) * 1000
                        if len(t) >= 12:
                            self.rk_time = t
                    except:
                        self.info_label.config(text="fetch error, reconnecting...", fg="#ff4444")
                        try:
                            self._connect()
                        except:
                            pass
                else:
                    self._connect()
                time.sleep(0.08)  # ~12 Hz refresh

        t = threading.Thread(target=loop, daemon=True)
        t.start()
        self.root.after(50, self._update_display)

    def _update_display(self):
        self.clock_label.config(text=self.rk_time)
        self.info_label.config(
            text=f"RK3568 system time  |  SSH RTT: {self.latency_ms:.0f}ms  |  Refresh: ~12Hz")
        if self.running:
            self.root.after(50, self._update_display)

    def _on_close(self):
        self.running = False
        if self.ssh:
            self.ssh.close()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def launch_v6():
    """Start V7 in test-pattern mode (no FPGA)."""
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        ssh.connect(BOARD_IP, username=BOARD_USER, password=BOARD_PASS, timeout=10)
    except Exception as e:
        print(f"[FAIL] SSH connect: {e}")
        return

    # Stop old
    ssh.exec_command("sudo pkill -9 -f rknn_browser 2>/dev/null; sleep 1", timeout=5)
    time.sleep(1)

    # Launch with test pattern
    cmd = (
        f"cd {BUILD} && "
        f"export LD_LIBRARY_PATH={V7}/cpp/ZLMediaKit/release/linux/Release:\$LD_LIBRARY_PATH && "
        f"sudo setsid ./rknn_browser_hdmi_lpr_demo "
        f"--test-pattern --jpeg-udp-dest 192.168.137.1:9999 "
        f"--width 1280 --height 720 --fps 25 "
        f"--infer-fps 1000 --output-fps 25 --zlm-gop 10 "
        f"--detect-model {MODEL}/yolov5n_i8.rknn "
        f"--ocr-backend lprnet "
        f"--recog-model {MODEL}/lprnet_color_fp16.rknn "
        f"--recog-chars {MODEL}/lprnet_chars.txt "
        f"--font {MODEL}/simhei.ttf "
        f"--zlm-sdk-dir {V7}/cpp/ZLMediaKit "
        f"--zlm-http-port 8084 --zlm-rtsp-port 8558 "
        f"--zlm-rtmp-port 1939 --zlm-rtc-port 8004 "
        f"--port 8080 "
        f"</dev/null >run.log 2>&1 &"
    )
    ssh.exec_command(cmd, timeout=8)
    time.sleep(5)

    _, out, _ = ssh.exec_command("ps aux | grep rknn_browser | grep -v grep")
    if out.read().decode().strip():
        print("[OK] V7 test-pattern started")
        print(f"  WebRTC:  http://{BOARD_IP}:8080/")
        print(f"  RTSP:    rtsp://{BOARD_IP}:8558/live/plate")
        print(f"  UDP:     udp://{BOARD_IP}:9999 -> pc_receiver.py")
    else:
        print("[FAIL] V7 not running, check log")
    ssh.close()


def stop_v6():
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    ssh.connect(BOARD_IP, username=BOARD_USER, password=BOARD_PASS, timeout=10)
    ssh.exec_command("sudo pkill -9 -f rknn_browser 2>/dev/null; sleep 1; echo stopped", timeout=5)
    print("[OK] V7 stopped")
    ssh.close()


if __name__ == "__main__":
    if "--stop" in sys.argv:
        stop_v6()
    elif "--launch" in sys.argv:
        launch_v6()
        RkClock().run()
    else:
        RkClock().run()
