#!/usr/bin/env python3
"""Dual stopwatch for end-to-end latency measurement.
Click START to begin both timers simultaneously (ms precision).
Press SPACE or click STOP to freeze. Press R to reset.
"""

import tkinter as tk
import time

class DualStopwatch:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Dual Stopwatch — Latency Measurement")
        self.root.geometry("1100x280")
        self.root.configure(bg="#1a1a2e")
        self.root.resizable(False, False)

        self.running = False
        self.start_time = 0.0
        self.elapsed = 0.0

        # Title
        tk.Label(self.root, text="End-to-End Latency Stopwatch",
                 font=("Consolas", 14, "bold"), fg="#e0e0e0", bg="#1a1a2e").pack(pady=(15, 2))
        tk.Label(self.root, text="Click START, compare both timers with video GEN_TIME overlay",
                 font=("Consolas", 9), fg="#888", bg="#1a1a2e").pack(pady=(0, 12))

        # Horizontal frame for both timers
        hf = tk.Frame(self.root, bg="#1a1a2e")
        hf.pack(pady=5)

        # Timer 1 (left)
        f1 = tk.Frame(hf, bg="#16213e", relief=tk.RIDGE, bd=2)
        f1.pack(side=tk.LEFT, padx=(10, 40))
        tk.Label(f1, text="STOPWATCH 1", font=("Consolas", 10, "bold"),
                 fg="#00d2ff", bg="#16213e").pack(pady=(10, 0))
        self.timer1 = tk.Label(f1, text="00:00:00.000", font=("Consolas", 36, "bold"),
                                fg="#00ff88", bg="#16213e", width=14)
        self.timer1.pack(padx=30, pady=(2, 10))

        # Timer 2 (right)
        f2 = tk.Frame(hf, bg="#16213e", relief=tk.RIDGE, bd=2)
        f2.pack(side=tk.LEFT, padx=(40, 10))
        tk.Label(f2, text="STOPWATCH 2", font=("Consolas", 10, "bold"),
                 fg="#ff6b6b", bg="#16213e").pack(pady=(10, 0))
        self.timer2 = tk.Label(f2, text="00:00:00.000", font=("Consolas", 36, "bold"),
                                fg="#ff6b6b", bg="#16213e", width=14)
        self.timer2.pack(padx=30, pady=(2, 10))

        # Buttons
        btn_frame = tk.Frame(self.root, bg="#1a1a2e")
        btn_frame.pack(pady=10)
        self.btn_start = tk.Button(btn_frame, text="START (Click)", command=self.start,
                                    font=("Consolas", 12, "bold"), bg="#00d2ff", fg="#000",
                                    width=14, height=1, activebackground="#00ff88")
        self.btn_start.pack(side=tk.LEFT, padx=5)
        self.btn_stop = tk.Button(btn_frame, text="STOP (Space)", command=self.stop,
                                   font=("Consolas", 12, "bold"), bg="#ff6b6b", fg="#000",
                                   width=14, height=1, activebackground="#ff4444", state=tk.DISABLED)
        self.btn_stop.pack(side=tk.LEFT, padx=5)

        tk.Label(self.root, text="START=Click  |  STOP=Space  |  RESET=R",
                 font=("Consolas", 8), fg="#555", bg="#1a1a2e").pack(pady=2)

        # Status
        self.status = tk.Label(self.root, text="READY", font=("Consolas", 10, "bold"),
                                fg="#888", bg="#1a1a2e")
        self.status.pack(pady=(0, 8))

        # Key bindings
        self.root.bind("<space>", lambda e: self.stop())
        self.root.bind("<r>", lambda e: self.reset())
        self.root.bind("<R>", lambda e: self.reset())

        self.update_display()

    def start(self):
        if not self.running:
            self.start_time = time.perf_counter()
            self.elapsed = 0.0
            self.running = True
            self.btn_start.config(state=tk.DISABLED)
            self.btn_stop.config(state=tk.NORMAL)
            self.status.config(text="RUNNING", fg="#00ff88")
            self.update_display()

    def stop(self):
        if self.running:
            self.elapsed = time.perf_counter() - self.start_time
            self.running = False
            self.btn_start.config(state=tk.NORMAL)
            self.btn_stop.config(state=tk.DISABLED)
            self.status.config(text="STOPPED", fg="#ff6b6b")
            self.update_display()

    def reset(self):
        self.running = False
        self.elapsed = 0.0
        self.start_time = 0.0
        self.btn_start.config(state=tk.NORMAL)
        self.btn_stop.config(state=tk.DISABLED)
        self.status.config(text="READY", fg="#888")
        self.update_display()

    def update_display(self):
        if self.running:
            self.elapsed = time.perf_counter() - self.start_time

        ms = int(self.elapsed * 1000)
        h = ms // 3600000
        m = (ms % 3600000) // 60000
        s = (ms % 60000) // 1000
        milli = ms % 1000
        text = f"{h:02d}:{m:02d}:{s:02d}.{milli:03d}"

        self.timer1.config(text=text)
        self.timer2.config(text=text)

        if self.running:
            self.root.after(20, self.update_display)

    def run(self):
        self.root.mainloop()


if __name__ == "__main__":
    DualStopwatch().run()
