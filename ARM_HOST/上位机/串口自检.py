"""串口自检：找出哪个 COM 口在收 RK3568 的 ADAS 帧。

用法（上位机关闭串口时运行，否则端口被占用会 PermissionError）：
    串口自检.py            # 扫描全部端口，每个 5 秒
    串口自检.py COM4       # 只扫指定口

协议（与 traffic_rules_sim.py 一致）：115200-8-N-1 无流控，
帧 = "<6位位串> <行人数量>\n"，正则 ([01]{6})[ \t]+(\d{1,6})(?![0-9])
位序：bit0 左道线实线 / bit1 右道线实线 / bit2 直行绿灯
      bit3 左转绿灯 / bit4 右转绿灯 / bit5 前方斑马线
"""
import re
import sys
import time

import serial
import serial.tools.list_ports

PAT = re.compile(rb"([01]{6})[ \t]+(\d{1,6})(?![0-9])")
BIT_NAMES = ["左实线", "右实线", "直行绿", "左转绿", "右转绿", "斑马线"]


def scan(port, seconds=5):
    try:
        s = serial.Serial(port, 115200, bytesize=8, parity="N", stopbits=1,
                          timeout=1, rtscts=False, dsrdtr=False)
    except Exception as e:
        print(f"  {port}: 打不开 -> {e}")
        return 0
    buf, frames = b"", []
    t0 = time.time()
    while time.time() - t0 < seconds:
        data = s.read(s.in_waiting or 1)
        if data:
            buf += data
            frames += PAT.findall(buf)
            buf = buf[-64:]
    s.close()
    if frames:
        print(f"  {port}: 收到 {len(frames)} 帧  <== RK3568 在这个口")
        b, n = frames[-1]
        active = [BIT_NAMES[i] for i, c in enumerate(b.decode()) if c == "1"]
        print(f"       最新帧: {b.decode()} {int(n)}  置位={active or '无'}")
    else:
        print(f"  {port}: 5 秒内无匹配帧（可能没接 RK3568，或程序未启动）")
    return len(frames)


def main():
    args = sys.argv[1:]
    ports = args or [p.device for p in serial.tools.list_ports.comports()]
    if not ports:
        print("未发现任何串口设备")
        return
    print("扫描端口(115200-8-N-1)：" + ", ".join(ports))
    for p in ports:
        scan(p)
    print("\n提示：若两路都无数据 -> 确认 RK 端已 start，且 --adas-serial-dev 与实际接线一致")


if __name__ == "__main__":
    main()
