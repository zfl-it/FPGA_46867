#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""从板端 JPEG-UDP(9999) 收一帧并保存为 jpg，用于离线拿真实画面验证模型。

协议（main.cc 编码线程）：
    每个分片 = [frame_id:4B BE][total:2B BE][idx:2B BE][payload]
    payload = [meta_len:4B BE][meta_json][jpeg_bytes]
"""
import socket
import struct
import sys
import time

PORT = 9999
OUT = sys.argv[1] if len(sys.argv) > 1 else "grab.jpg"
WAIT = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
s.bind(("0.0.0.0", PORT))
s.settimeout(1.0)

frames = {}          # fid -> {idx: bytes}
total = {}           # fid -> total_chunks
newest = None
deadline = time.time() + WAIT

while time.time() < deadline:
    try:
        buf, _ = s.recvfrom(2048)
    except socket.timeout:
        continue
    if len(buf) < 8:
        continue
    fid, tc, idx = struct.unpack(">IHH", buf[:8])
    frames.setdefault(fid, {})[idx] = buf[8:]
    total[fid] = tc
    if newest is None or fid > newest:
        newest = fid
    # 收到一个完整帧就退出
    if len(frames.get(fid, {})) == tc:
        newest = fid
        break

s.close()
if newest is None or len(frames.get(newest, {})) != total.get(newest, -1):
    got = len(frames.get(newest, {})) if newest is not None else 0
    print("FAIL: 帧不完整 fid=%s got=%d/%s" % (newest, got, total.get(newest)))
    sys.exit(1)

payload = b"".join(frames[newest][i] for i in range(total[newest]))
meta_len = struct.unpack(">I", payload[:4])[0]
meta = payload[4:4 + meta_len].decode("utf-8", "replace")
jpeg = payload[4 + meta_len:]
with open(OUT, "wb") as f:
    f.write(jpeg)
print("OK fid=%d chunks=%d jpeg=%d bytes -> %s" % (newest, total[newest], len(jpeg), OUT))
print("meta:", meta)
