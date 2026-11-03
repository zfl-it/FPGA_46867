#!/bin/bash
# BL-3 conversion environment diagnostics
echo "=== [1] basic info ==="
uname -a
python3 -V
which python3
echo
echo "=== [2] python packages ==="
python3 - <<'EOF'
mods = [("torch", "torch"), ("onnx", "onnx"), ("onnxsim", "onnxsim"),
        ("rknn_api", "rknn.api"), ("cv2", "cv2"), ("numpy", "numpy"),
        ("ultralytics", "ultralytics")]
for name, mod in mods:
    try:
        m = __import__(mod.split(".")[0])
        v = getattr(m, "__version__", "?")
        print("  OK   %-12s %s" % (name, v))
    except Exception as e:
        print("  MISS %-12s %s" % (name, type(e).__name__))
EOF
echo
echo "=== [3] pip config ==="
python3 -m pip config list 2>&1 | head -5
echo "--- installed pkgs ---"
python3 -m pip list 2>/dev/null | grep -i -E "rknn|onnx|torch|ultra" | head -10
echo
echo "=== [4] network ==="
for u in https://pypi.tuna.tsinghua.edu.cn/simple/ https://mirrors.aliyun.com/pypi/simple/ https://pypi.org/simple/; do
  code=$(timeout 15 curl -s -o /dev/null -w "%{http_code}" "$u" 2>/dev/null)
  echo "  $code  $u"
done
echo
echo "=== [5] input files ==="
for f in /mnt/c/Users/Lenovo/Desktop/BL-3/weights/best.pt /mnt/c/Users/Lenovo/WorkBuddy/2026-10-27-12-36-19/work/calib; do
  if [ -e "$f" ]; then echo "  OK   $f"; else echo "  MISS $f"; fi
done
ls /mnt/c/Users/Lenovo/WorkBuddy/2026-10-27-12-36-19/work/calib 2>/dev/null | wc -l | sed 's/^/  calib image count: /'
echo
echo "=== [6] convert script ==="
ls -la /mnt/c/Users/Lenovo/Desktop/RK3568_V8_720P/convert_bl3.py 2>&1
echo
echo "=== done ==="
