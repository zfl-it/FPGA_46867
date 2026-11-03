#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""BL-3（12 类）-> RK3568 int8 RKNN 一键转换。

用法（必须在 Linux x86_64 下跑，rknn-toolkit2 只有 x86 版）：
    wsl -d Ubuntu-22.04 -u root bash -lc "cd /mnt/c/Users/Lenovo/Desktop/RK && python3 convert_bl3.py"

三步流水线：
    1. best.pt  ->  ONNX (1280x736, opset12, simplify)
    2. ONNX     ->  ONNX(置信度分支 ×1280)
    3. ONNX     ->  RKNN int8（per-channel 量化，校准集 54 张 1280x736 实车帧）

为什么第 2 步必须做：output0 = concat(框坐标 0~1280, 置信度 0~1)，
RKNN 对它做 per-tensor int8 时步长 ≈ 5，0~1 的置信度会被全部舍入成 0。
在 Sigmoid 后乘 1280 让各通道量程一致，板端解码时除回来即可
（adas_detector.cc 的 kScoreScale = 1280）。

类别顺序（与 BL-3/weights/best.pt 的 names 一致，C++ 侧 ADAS_NC 必须等于 12）：
    0 左转红灯  1 左转绿灯  2 右转红灯  3 右转绿灯
    4 直行红灯  5 直行绿灯  6 虚线      7 实线
    8 行人      9 斑马线   10 圆盘红灯 11 圆盘绿灯
"""

from __future__ import annotations

import glob
import os
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.abspath(__file__))
NC = 12
SCORE_SCALE = 1280.0
IMGSZ = (736, 1280)          # (H, W)：720 向上补到 32 的倍数，横向保持 1280

NAMES = ["left_red", "left_green", "right_red", "right_green",
         "straight_red", "straight_green", "dash", "solid",
         "pedestrian", "crosswalk", "disc_red", "disc_green"]


def p(win_path: str) -> str:
    """Windows 路径 -> 当前平台路径（WSL 下转成 /mnt/c/...）。"""
    if os.name == "nt":
        return win_path
    drive = win_path[0].lower()
    return "/mnt/" + drive + "/" + win_path[2:].replace("\\", "/")


# 换模型只改这 5 行：PT 指向新的 best.pt，输出加后缀区分版本，
# 不覆盖旧的 adas12_*（BL-3/YOLOv8s）、adas12n_*（BLP-3）、adas12n4_*（BLP-4）、
# adas12n6_*（BLP-6），随时可回退。
PT = p(r"C:\Users\Lenovo\Desktop\BLP-7\weights\best.pt")
MODEL_DIR = os.path.join(ROOT, "model")
CALIB_DIR = p(r"C:\Users\Lenovo\WorkBuddy\2026-10-27-12-36-19\work\calib")
ONNX_RAW = os.path.join(MODEL_DIR, "adas12n7_736x1280.onnx")
ONNX_SCALED = os.path.join(MODEL_DIR, "adas12n7_736x1280_scaled.onnx")
RKNN_OUT = os.path.join(MODEL_DIR, "adas12n7_736x1280_i8_scaled.rknn")


# 导出用的解释器候选（按顺序找）：当前解释器若没装 ultralytics 就用这些
EXPORT_PYTHON_CANDIDATES = (
    "/root/lane_work/onnx_env/bin/python",          # 本机已验证可用的 ultralytics 环境
    os.path.join(ROOT, ".venv-export", "bin", "python"),
)


def _pick_export_python() -> list[str]:
    """选一个装了 ultralytics 的解释器：优先当前解释器，其次候选列表。"""
    try:
        import ultralytics  # noqa: F401
        return [sys.executable]
    except Exception:  # noqa: BLE001
        pass
    for cand in EXPORT_PYTHON_CANDIDATES:
        if os.path.isfile(cand):
            return [cand]
    sys.exit("✗ 找不到装了 ultralytics 的 Python。请在 WSL 里执行：\n"
             "    python3 -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple ultralytics onnxslim")


def step1_export() -> str:
    print("\n=== [1/3] best.pt -> ONNX ===")
    helper = os.path.join(ROOT, "export_bl3_onnx.py")
    if not os.path.isfile(helper):
        sys.exit("✗ 缺少导出脚本: " + helper)
    cmd = _pick_export_python() + [helper]
    print("  解释器:", cmd[0], flush=True)
    rc = subprocess.call(cmd)
    if rc != 0 or not os.path.isfile(ONNX_RAW):
        sys.exit("✗ 导出 ONNX 失败（返回码 %d，见上方日志）" % rc)
    print("ONNX ->", ONNX_RAW)
    return ONNX_RAW


def step2_scale(src: str) -> str:
    print("\n=== [2/3] 置信度分支 ×%d ===" % int(SCORE_SCALE))
    import numpy as np
    import onnx
    from onnx import helper, numpy_helper

    m = onnx.load(src)
    g = m.graph

    concat = None
    for n in g.node:
        if n.op_type == "Concat" and "output0" in n.output:
            concat = n
            break
    assert concat is not None, "找不到输出 output0 的 Concat 节点"

    # 校验输出通道数：4 框 + NC 置信度
    shp = [d.dim_value for d in g.output[0].type.tensor_type.shape.dim]
    print("output0 shape:", shp)
    if len(shp) == 3 and shp[1] not in (0, 4 + NC):
        sys.exit(f"✗ 输出通道 {shp[1]} != {4 + NC}，与 ADAS_NC={NC} 不一致")

    score_in = concat.input[1]
    cname = "/model/adas_score_scale_const"
    g.initializer.append(numpy_helper.from_array(
        np.array(SCORE_SCALE, dtype=np.float32), name=cname))

    scaled_out = "/model/Sigmoid_scaled_output_0"
    mul = helper.make_node("Mul", inputs=[score_in, cname],
                           outputs=[scaled_out], name="/model/adas_score_mul")
    g.node.insert(list(g.node).index(concat), mul)
    concat.input[1] = scaled_out

    onnx.checker.check_model(m)
    onnx.save(m, ONNX_SCALED)
    print("ONNX(scaled) ->", ONNX_SCALED)
    return ONNX_SCALED


def step3_build(src: str) -> str:
    print("\n=== [3/3] ONNX -> RKNN int8 ===")
    from rknn.api import RKNN

    imgs = sorted(glob.glob(os.path.join(CALIB_DIR, "*.jpg")))
    if not imgs:
        sys.exit("✗ 校准集为空: " + CALIB_DIR)
    ds = os.path.join(ROOT, "calib_dataset.txt")
    with open(ds, "w", encoding="utf-8") as f:
        f.write("\n".join(imgs) + "\n")
    print("校准图:", len(imgs), "张，清单:", ds)

    rknn = RKNN(verbose=False)
    # 板端直接送 uint8 RGB，归一化写进模型（mean=0 / std=255）
    rknn.config(mean_values=[[0, 0, 0]], std_values=[[255, 255, 255]],
                target_platform="rk3568", quantized_algorithm="normal",
                quantized_method="channel", optimization_level=3)
    assert rknn.load_onnx(model=src) == 0, "load_onnx 失败"
    assert rknn.build(do_quantization=True, dataset=ds) == 0, "build 失败"
    assert rknn.export_rknn(RKNN_OUT) == 0, "export_rknn 失败"
    print("RKNN ->", RKNN_OUT)
    return RKNN_OUT


def step4_verify(rknn_path: str) -> None:
    """模拟器上跑一张校准图，确认 12 个置信度通道没被量化压成 0。"""
    print("\n=== 验证（模拟器推理）===")
    try:
        import cv2
        import numpy as np
        from rknn.api import RKNN
    except Exception as e:  # noqa: BLE001
        print("跳过验证（缺依赖）:", e)
        return

    imgs = sorted(glob.glob(os.path.join(CALIB_DIR, "*.jpg")))
    bgr = cv2.imread(imgs[0])
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

    rknn = RKNN(verbose=False)
    if rknn.load_rknn(rknn_path) != 0:
        print("load_rknn 失败，跳过验证")
        return
    if rknn.init_runtime() != 0:
        print("init_runtime 不可用（当前机器无 NPU 模拟器），跳过验证")
        return
    out = rknn.inference(inputs=[np.expand_dims(rgb, 0)], data_format=["nhwc"])
    p0 = np.asarray(out[0], dtype=np.float32).reshape(4 + NC, -1)
    cls = p0[4:] / SCORE_SCALE
    per = cls.max(axis=1)
    for i, v in enumerate(per):
        print(f"  cls{i:2d} {NAMES[i]:14s} max={v:.4f}")
    alive = int((per > 0.01).sum())
    print(f"置信度通道非零个数: {alive}/{NC}"
          + ("  ✓" if alive > 0 else "  ✗ 疑似量化塌缩，检查 x1280 是否生效"))
    rknn.release()


def main() -> None:
    os.makedirs(MODEL_DIR, exist_ok=True)
    if not os.path.isfile(PT):
        sys.exit("✗ 找不到 best.pt: " + PT)
    raw = step1_export()
    scaled = step2_scale(raw)
    out = step3_build(scaled)
    step4_verify(out)
    size = os.path.getsize(out) / 1024 / 1024
    print(f"\n=== DONE ===\n{out}  ({size:.1f} MB)")
    print("下一步：python push_and_build.py  ->  python v8_start.py restart")


if __name__ == "__main__":
    main()
