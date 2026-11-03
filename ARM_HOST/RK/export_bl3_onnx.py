#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""第 1 步：BL-3 的 best.pt -> ONNX（必须在装了 ultralytics 的 Python 里跑）。

由 convert_bl3.py 自动调用，一般不需要单独执行：
    /root/lane_work/onnx_env/bin/python export_bl3_onnx.py

为什么要单独一个文件：量化（第 2/3 步）需要 rknn-toolkit2，它只装在系统
python3 里；而 ultralytics 装在 /root/lane_work/onnx_env 虚拟环境里。
两个环境混在一起会互相顶掉 numpy/onnx 版本，所以拆成两段、各自用自己的解释器。
"""

from __future__ import annotations

import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from convert_bl3 import IMGSZ, MODEL_DIR, NC, ONNX_RAW, PT  # noqa: E402


def main() -> None:
    if not os.path.isfile(PT):
        sys.exit("best.pt 不存在: " + PT)

    print("导出解释器:", sys.executable, flush=True)
    from ultralytics import YOLO

    m = YOLO(PT)
    names = list(m.names.values()) if isinstance(m.names, dict) else list(m.names)
    print("类别数:", len(names), names, flush=True)
    if len(names) != NC:
        sys.exit("类别数 %d != %d，确认这个 best.pt 是不是 BL-3" % (len(names), NC))

    # end2end=False 必须显式传：
    #   YOLO26（BLP-4）的 Detect 头默认走端到端分支，导出 (1, 300, 6) 的
    #   「已完成 NMS 的最终框」；板上 adas_detector.cc 解的是 (1, 4+nc, anchors)
    #   的 anchor 矩阵并自己做 NMS，两者不兼容。
    #   关掉后回到 one2many 分支 -> (1, 16, 19320)，与 YOLOv8 完全一致。
    #   YOLOv8（BLP-3/BL-3）没有 e2e 分支，传 False 无副作用。
    out = m.export(format="onnx", imgsz=IMGSZ, opset=12,
                   simplify=True, dynamic=False, nms=False, end2end=False)
    os.makedirs(MODEL_DIR, exist_ok=True)
    shutil.copy2(out, ONNX_RAW)
    print("ONNX ->", ONNX_RAW, flush=True)


if __name__ == "__main__":
    main()
