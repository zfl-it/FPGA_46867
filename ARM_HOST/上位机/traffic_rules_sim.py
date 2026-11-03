#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
智能驾驶 · 交规决策模拟器  (Traffic-Rule Decision Simulator)
=============================================================

功能
----
1. 控制小窗顶部「感知状态」：6 个布尔量（模拟车载传感器输入），**只读显示**，
      由串口接收数据驱动（v3.11 从调试界面左栏搬到小窗，2 列 x 3 行徽标）。
      L1  左道线是否为实线        left_line_solid
      L2  右道线是否为实线        right_line_solid
      L3  直行红绿灯是否为绿      straight_green
      L4  左转红绿灯是否为绿      left_green
      L5  右转红绿灯是否为绿      right_green
      L6  前方是否有斑马线            crosswalk_ahead
      同一块底部还有「位串 111110」与「行人 N 人」：读串口帧里的位串与人数，只读。
2. 两位小数浮点变量：车速 speed (km/h)，默认 0.00
      —— speed 是外部输入量：程序内部**不会**自动加速、自动制动或以任何方式改写它；
         本版本已关闭手动输入，后续将由串口协议扩展支持。
3. 七个动作按钮：直行 / 左转 / 右转 / 左变道 / 右变道 / 倒车 / 停车
   （v3.14 五键，v3.21 增加左/右变道）
      - **启动与复位后的默认状态就是「停车」**（无行驶意图，停车键红色高亮）
      - 合法 -> 绿色放行提示（给出建议巡航车速，但车速不因此变化），
        并播报按钮名称（v3.21，六语种）
      - 非法 -> 红色报警（**人声语音** + 闪烁 + 列出违反的具体交规条款），不播按钮名
      - 行驶途中若传感器状态变化使当前动作变为非法 -> 立即报警提示（不自动制动）；
        v3.18：其中**灯类理由（信号灯非绿色）只记日志不播语音** —— 红灯武装
        瞬间静默；标线类理由照常报警
      - **左转 / 右转 / 左变道 / 右变道是瞬时动作**：按下 1 秒后自动回到转向前的
        状态（直行或停车），连按另一个转向不会改变回退目标；
        手动按 直行 / 倒车 / 停车 立即取消回退；自动回退不播报按钮名（v3.21）
      - **报警不锁定操作**：所有按键始终可用，
        违规只做语音提醒 + 红色提示，你可以照常按下任何动作，程序不拦
      - 语音提示**只在条件成立的瞬间触发一次**，且同类提示有 10 秒冷却
      - （v3.20 起）不再播报「已经可直行 / 左转 / 右转」「全线放行」等
        放行提醒，绿灯放行完全静默；违规报警语音不受影响
      - 报警打断（v3.15）：禁止语音能打断「注意斑马线 / 注意行人」等提示音，
        报警之间也互相打断 —— 连续违规时只播最新的一条
        （防止人数在阈值附近抖动时反复播报、盖住违规报警）：
          遇到斑马线            -> 提示「注意斑马线」
          行人数量 >= 5 人      -> 提示「注意行人」
          任一动作违规          -> 提示「警告，……（具体违规内容）」
5. 人声提示支持联合国六种通用语言（阿拉伯文 / 中文 / 英文 / 法文 / 俄文 / 西班牙文）
      可随时切换；优先使用在线神经网络语音（合成结果缓存，同一句只联网一次），
      无网络时自动降级到系统离线语音，再降级到蜂鸣。
4. 内置串口助手：**两路独立串口**，链路参数均固定为 115200 / 8 数据位 /
      1 停止位 / 无校验位 / 无流控（即 115200-8-N-1），避免误设导致通信失败；
      支持打开关闭、ASCII/HEX 收发显示、收发计数、日志模式、自动滚屏、发送框等。
        - 串口 A「RK3568 感知」：接收 RK3568 发来的感知帧，格式为
          「6 位 1/0 位串 + 一个空格 + 行人数量」，例如 "111110 3"；
          6 位位串按界面从上到下的顺序解析为 6 路布尔量，空格后的数字解析为
          行人数量（只有 A 通道参与解析）。为兼容旧版本，仅收到 6 位位串时
          仍可正常更新布尔量，行人数量保持上一次的值；
        - 串口 B「盘古50K 下发」：向盘古 50K 输出画面控制指令与手动下发数据。
      两路串口可同时打开、互不干扰；画面控制指令与数据发送区各自可选下发目标。
      画面控制指令共 7 条，下发单字节 0~6：
        0 画面切换（纯动作）   1 单路亮度自适应   2 单路夜景修复   3 单路雾天模式
        4 倒车辅助线           5 拼接夜景模式     6 拼接雾天模式
      其中 1~6 是开关类：按一下打开功能、按钮变红；再按一下关闭、按钮变黑。
      两次下发同一个字节，由板端按"收到即翻转"解释，协议不新增字节。
6. 倒车轨迹角度盘（v3.6）：左栏下方的 ±30° 半圆仪表盘，鼠标拖动 / 滚轮微调 /
      双击回正；串口 B 一打开就按 100ms 周期持续下发角度帧
      「'A' + dir + 十位 + 个位 + '1' + '1'」（如 A01511 = 右 15°），
      与盘古 50K（PGL50H）reverse_aux_lines.v 中的 traj_uart_ctrl 对齐。
      仅在单路视图且用 '4' 打开倒车辅助线时，板端才会把辅助线画出来。
7. 双界面 + 3D 虚拟路况（v3.12）：程序打开时停在**显示界面**，控制界面默认不显示
      - 显示界面：顶部黑色按钮条（横排三枚圆形按钮：双画面 / 3D / 控制台，
        不遮挡画面）。拉取 RK3568 实时画面，v3.10 起优先走**板端 JPEG-UDP 直推**
        （板子把带检测框的帧直接 UDP 分片发过来，不经过 RTSP/ZLM/H.264，
        延迟低得多；3 秒收不到自动回退 RTSP）。
        可用环境变量 SIM_VIDEO_SRC=rtsp 强制走 RTSP、=udp 强制只走 UDP；
        F11 或双击画面 -> 全屏，Esc 退出全屏；
      - 3D 界面：按「3D」按钮在 实时画面 ⇄ 3D 虚拟路况 间切换。3D 页面就是
        3D/虚拟路况视图.html，经 WebView2 内核嵌入（three.js 已本地化，离线可用），
        首次加载 3~8 秒，之后常驻后台、切换即时；
      - 调试界面：上面 1~6 的其余内容；F2 或「双画面」按钮切换；
      - 多层叠放在同一窗口内，切换用 tkraise，拉流线程始终在后台跑，切换即时的。

运行
----
    python traffic_rules_sim.py            启动图形界面
    python traffic_rules_sim.py --selftest 运行交规判定内核的自测（无需界面）
"""

from __future__ import annotations

import ctypes
import math
import os
import queue
import re
import socket
import struct
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

# ---- 显示界面（网口视频）用到的可选依赖 ----
# 缺了不会导致程序起不来，只是显示界面退化成「未安装依赖」提示。
try:
    import cv2
except Exception:                                    # pragma: no cover
    cv2 = None
try:
    import numpy as np
except Exception:                                    # pragma: no cover
    np = None
try:
    from PIL import Image, ImageTk
except Exception:                                    # pragma: no cover
    Image = None
    ImageTk = None

# Windows 高分屏（如 133%/150% 缩放）下让 Tk 按物理像素布局，
# 避免 DPI 虚拟化把窗口放大导致控件超出屏幕 / 内容被裁剪
if sys.platform == "win32":
    try:
        import ctypes
        ctypes.windll.shcore.SetProcessDpiAwareness(2)   # PER_MONITOR_AWARE_V2
    except Exception:
        try:
            ctypes.windll.user32.SetProcessDPIAware()
        except Exception:
            pass

# ------------------------------------------------------------------ 语音依赖搜索路径
def _bootstrap_voice_libs() -> None:
    """让语音依赖（edge-tts / pygame）可被找到。

    推荐做法：在项目虚拟环境里执行「安装语音依赖.bat」安装 edge-tts 与 pygame。
    若尚未安装，则回退到创建本虚拟环境时所用基础解释器的 site-packages
    （本工程最初即由该解释器创建，依赖已装在那里的情况下无需重复安装）。
    """
    try:
        base = getattr(sys, "_base_executable", "") or sys.executable
        sp = os.path.join(os.path.dirname(base), "Lib", "site-packages")
        if os.path.isdir(sp) and sp not in sys.path:
            sys.path.append(sp)
    except Exception:
        pass


_bootstrap_voice_libs()

try:
    from voice_prompts import (  # type: ignore
        DEFAULT_LANG as VOICE_DEFAULT_LANG,
        LANG_CODES,
        LANG_NAMES,
        PRIO_ALARM,
        PRIO_TIP,
        Speaker,
        alarm_parts,
        alarm_text,
        phrase,
        selftest_phrases,
        selftest_parts,
    )

    HAS_VOICE = True
except Exception as _e:                      # voice_prompts.py 缺失时不阻断主程序
    HAS_VOICE = False
    VOICE_DEFAULT_LANG = "zh"
    LANG_CODES = ["zh", "en", "ar", "fr", "ru", "es"]
    LANG_NAMES = {c: c for c in LANG_CODES}
    Speaker = None                            # type: ignore
    alarm_text = None                         # type: ignore
    alarm_parts = None                        # type: ignore
    phrase = None                             # type: ignore
    selftest_phrases = None                   # type: ignore
    selftest_parts = None                     # type: ignore
    PRIO_ALARM, PRIO_TIP = 0, 1

# ------------------------------------------------------------------ 串口依赖
try:
    import serial
    import serial.tools.list_ports

    HAS_SERIAL = True
except Exception:  # pyserial 未安装时仍可打开界面，串口功能不可用
    serial = None  # type: ignore
    HAS_SERIAL = False

try:  # Windows 蜂鸣报警
    import winsound

    def _beep(freq: int = 1200, ms: int = 260, times: int = 2) -> None:
        try:
            for _ in range(times):
                winsound.Beep(freq, ms)
                time.sleep(0.05)
        except Exception:
            pass

except ImportError:  # 非 Windows 降级

    def _beep(*_a, **_k) -> None:
        pass


# ==================================================================== 交规内核
# 各动作的"建议"巡航车速 (km/h)
SUGGEST_SPEED = {"直行": 30.00, "左转": 15.00, "右转": 10.00,
                 "左变道": 25.00, "右变道": 20.00, "倒车": 5.00}

# ---- 行驶意图按键 ----
# v3.14：五键 = 直行 / 左转 / 右转 / 倒车 / 停车；启动与复位后的默认状态都是「停车」。
# v3.21：新增「左变道 / 右变道」——判定条件与左转/右转完全一致，只是标签与车速不同。
TURN_ACTIONS = ("左转", "右转", "左变道", "右变道")   # 瞬时动作：按下 1 秒后自动回退
TURN_RETURN_SEC = 1.0               # 转向/变道自动回到先前状态（直行或停车）的延时

# ---- 提示类语音冷却（秒）----
# 斑马线 / 行人提示触发一次后，在冷却时间内即使条件反复满足也不再播报，
# 防止人数在阈值附近抖动时"注意行人"一直响、盖住左右转与直行的违规报警。
TIP_COOLDOWN_SEC = 10.0

# 6 个布尔量（键 -> 中文名），顺序与界面从上到下一致，也对应串口 6 字节字符串顺序
SENSORS = (
    ("left_line_solid",  "左道线是否为实线"),
    ("right_line_solid", "右道线是否为实线"),
    ("straight_green",   "直行红绿灯是否为绿"),
    ("left_green",       "左转红绿灯是否为绿"),
    ("right_green",      "右转红绿灯是否为绿"),
    ("crosswalk_ahead",  "前方是否有斑马线"),
)

SENSOR_KEYS = [k for k, _ in SENSORS]

# 控制小窗里的紧凑命名（小窗宽度只有 360px，长名字放不下）
SENSOR_SHORT_NAMES = {
    "left_line_solid":  "左线实线",
    "right_line_solid": "右线实线",
    "straight_green":   "直行灯绿",
    "left_green":       "左转灯绿",
    "right_green":      "右转灯绿",
    "crosswalk_ahead":  "前方斑马线",
}

# 启动时六个布尔量的默认值：真真真真真假
DEFAULT_SENSOR_BITS = "111110"

# ---- 仅作提示、不参与通断行进 / 不触发报警的感知量 ----
# 这些人畜无害的信息只驱动语音播报（「注意斑马线」「注意行人」）与界面高亮，
# 不会出现在 judge() 的违规理由里，也不会导致禁止动作或报警锁定。
# （2026-10-27 按需求调整：原实现把斑马线当成最高优先级让行依据，会让车在
#   斑马线前一味被判禁止通行，与"只提示"的定位不符。）
TIP_ONLY_SENSORS = ("crosswalk_ahead",)

# ---- 行人数量：串口帧「6 位位串 + 空格 + 人数」中的人数部分 ----
DEFAULT_PED_COUNT = 0        # 启动/复位时的默认人数
PED_COUNT_MAX = 999          # 单帧允许的最大人数（防止异常帧把数字撑爆）
PED_ALERT_THRESHOLD = 5      # 人数 >= 该值 -> 语音提示「注意行人」

# ---- 报警提示时长（秒）----
# v3.11 起报警只做「视觉闪烁 + 人声播报」的提示，**不再锁定操作**：
# 直行 / 左转 / 右转 / 倒车 / 停车五键始终可用，用户随时可以按下自己的操作意图。
ALARM_FLASH_SEC = 3.0
ALARM_NOTICE_SEC = 4.0
# 兼容旧配置项（已不用于锁定，保留常量名以防外部脚本引用）
ALARM_LOCK_SEC = 0.0

# ---- 语音提示默认语言（界面可切换）----
DEFAULT_VOICE_LANG = VOICE_DEFAULT_LANG


def judge(action: str, st: dict) -> tuple[bool, list[str]]:
    """决策内核（纯函数，可单测）"""
    reasons: list[str] = []

    if action == "直行":
        if not st["straight_green"]:
            reasons.append(
                "直行信号灯非绿色 —— 违反《道交法》第38条：绿灯亮时准许通行，红灯/黄灯禁止越过停止线"
            )
        # 注：斑马线/行人已降级为"纯提示"，不再作为禁止通行的依据（见 TIP_ONLY_SENSORS）。

    elif action in ("左转", "左变道"):
        if st["left_line_solid"]:
            reasons.append(
                "左侧车道线为实线 —— 违反禁止标线指示（《道交法》第38条 / 第90条）：实线一侧禁止压线、越线转向"
            )
        if not st["left_green"]:
            reasons.append(
                "左转信号灯非绿色 —— 违反《道交法》第38条：左转箭头灯未放行不得左转"
            )

    elif action in ("右转", "右变道"):
        if st["right_line_solid"]:
            reasons.append(
                "右侧车道线为实线 —— 违反禁止标线指示（《道交法》第38条 / 第90条）：实线一侧禁止压线、越线转向"
            )
        if not st["right_green"]:
            reasons.append(
                "右转信号灯非绿色 —— 违反《道交法》第38条：设有右转箭头灯时应按灯通行"
            )

    elif action == "倒车":
        # 倒车不设信号灯 / 标线限制：始终允许（轨迹与速度由驾驶员控制，
        # 倒车辅助线由「倒车轨迹角度盘」经串口 B 下发）。
        pass

    else:
        raise ValueError(f"未知动作: {action}")

    return (len(reasons) == 0), reasons


# 判定理由 -> 语音词条键（用于拼装"警告，……"后面的具体内容）
REASON_KEY_RULES = (
    ("r_straight_light", "直行信号灯非绿色"),
    ("r_left_light",     "左转信号灯非绿色"),
    ("r_right_light",    "右转信号灯非绿色"),
    ("r_left_solid",     "左侧车道线为实线"),
    ("r_right_solid",    "右侧车道线为实线"),
    # r_crosswalk 已移除：斑马线不再构成违规理由，只做语音提示
)


def reason_keys(reasons: list[str]) -> list[str]:
    """把 judge() 的中文理由映射为语音词条键（保持顺序、去重）。"""
    keys: list[str] = []
    for r in reasons:
        for key, marker in REASON_KEY_RULES:
            if marker in r:
                if key not in keys:
                    keys.append(key)
                break
    return keys


# ------------------------------------------------------------------ 串口数据解析
def try_parse_sensor_bits(data: bytes) -> str | None:
    """从字节流中提取最近一个合法的 6 位 1/0 传感器字符串。

    串口协议：固定 6 字节 ASCII，'1'=True，'0'=False，顺序与 SENSORS 一致。
    若当前缓冲区包含多个完整帧，取最后一帧；若不存在合法帧，返回 None。
    """
    if len(data) < 6:
        return None
    # 从后往前找 6 个连续 '0'/'1' 字节
    for i in range(len(data) - 6, -1, -1):
        chunk = data[i:i + 6]
        if all(b in (0x30, 0x31) for b in chunk):
            return chunk.decode("ascii")
    return None


# 新协议：6 位位串 + 至少一个空格/制表符 + 行人数量，例如 b"111110 3"
#   (?![0-9]) 保证多位数收全（"12" 不会在只收到 "1" 时被当成完整帧）
FRAME_RE = re.compile(rb"([01]{6})[ \t]+(\d{1,6})(?![0-9])")
FRAME_HINT = "6位位串 + 空格 + 行人数量，例：111110 3"


def try_parse_sensor_frame(data: bytes) -> tuple[str, int] | None:
    """从字节流中提取最近一帧完整的「位串 + 行人数量」。

    返回 (bits, count)；不存在完整帧时返回 None（兼容旧协议：由调用方回退到
    只解析 6 位位串的做法，行人数量保持上一次的值）。
    """
    if not data:
        return None
    matches = list(FRAME_RE.finditer(data))
    if not matches:
        return None
    m = matches[-1]
    try:
        count = int(m.group(2))
    except ValueError:
        return None
    return m.group(1).decode("ascii"), max(0, min(PED_COUNT_MAX, count))


def consume_sensor_frame(buf: bytearray) -> tuple[str, int] | None:
    """在接收缓冲区里取最后一帧完整数据，并把该帧及其之前的字节全部消费掉。"""
    matches = list(FRAME_RE.finditer(bytes(buf)))
    if not matches:
        return None
    m = matches[-1]
    try:
        count = int(m.group(2))
    except ValueError:
        return None
    res = (m.group(1).decode("ascii"), max(0, min(PED_COUNT_MAX, count)))
    del buf[:m.end()]
    return res


# ==================================================================== 自测
def selftest() -> int:
    ok = fail = 0

    def check(desc: str, cond: bool) -> None:
        nonlocal ok, fail
        if cond:
            ok += 1
            print(f"  [PASS] {desc}")
        else:
            fail += 1
            print(f"  [FAIL] {desc}")

    base = {k: False for k in SENSOR_KEYS}

    print("== 1. 基准：虚线 + 三灯全绿 + 无斑马线 ==")
    st = dict(base, straight_green=True, left_green=True, right_green=True)
    check("直行放行", judge("直行", st)[0])
    check("左转放行", judge("左转", st)[0])
    check("右转放行", judge("右转", st)[0])

    print("== 2. 信号灯约束 ==")
    check("直行红灯 -> 禁直行", not judge("直行", dict(st, straight_green=False))[0])
    check("左转红灯 -> 禁左转", not judge("左转", dict(st, left_green=False))[0])
    check("右转红灯 -> 禁右转", not judge("右转", dict(st, right_green=False))[0])
    check("直行绿灯不受左转灯影响", judge("直行", dict(st, left_green=False))[0])

    print("== 3. 实线约束 ==")
    check("左实线 -> 禁左转", not judge("左转", dict(st, left_line_solid=True))[0])
    check("左实线 -> 不禁右转", judge("右转", dict(st, left_line_solid=True))[0])
    check("左实线 -> 不禁直行", judge("直行", dict(st, left_line_solid=True))[0])
    check("右实线 -> 禁右转", not judge("右转", dict(st, right_line_solid=True))[0])
    check("右实线 -> 不禁左转", judge("左转", dict(st, right_line_solid=True))[0])
    check("右实线 -> 不禁直行", judge("直行", dict(st, right_line_solid=True))[0])

    print("== 4. 斑马线/行人：只提示，不阻断通行 ==")
    cw = dict(st, crosswalk_ahead=True)
    for a in ("直行", "左转", "右转"):
        allowed, rs = judge(a, cw)
        check(f"有斑马线 -> {a} 仍放行", allowed)
        check(f"有斑马线 -> {a} 不产生第47条依据", not any("47条" in r for r in rs))
    check("crosswalk_ahead 属于仅提示感知量", "crosswalk_ahead" in TIP_ONLY_SENSORS)
    check("理由映射中已无斑马线键", not any(k == "r_crosswalk" for k, _ in REASON_KEY_RULES))

    print("== 5. 多条件叠加，条目齐全 ==")
    st2 = dict(base, left_line_solid=True, left_green=False, crosswalk_ahead=True)
    allowed, rs = judge("左转", st2)
    check("左实线+红灯+斑马线 -> 禁左转且只列出 2 条依据", (not allowed) and len(rs) == 2)
    check("禁用依据里不含斑马线", not any("斑马线" in r for r in rs))

    print("== 6. 串口数据解析 ==")
    check("111110 -> 传感器串", try_parse_sensor_bits(b"111110") == "111110")
    check("前置垃圾后接合法帧 -> 取合法帧", try_parse_sensor_bits(b"ABC011000111110") == "111110")
    check("长度不足 6 -> None", try_parse_sensor_bits(b"1111") is None)
    check("含非法字符 -> None", try_parse_sensor_bits(b"121110") is None)

    print("== 6b. 新协议：位串 + 空格 + 行人数量 ==")
    check('"111110 3" -> ("111110", 3)',
          try_parse_sensor_frame(b"111110 3") == ("111110", 3))
    check('"111110 0" -> ("111110", 0)',
          try_parse_sensor_frame(b"111110 0") == ("111110", 0))
    check('带换行结尾 -> 正常解析',
          try_parse_sensor_frame(b"000000 12\r\n") == ("000000", 12))
    check("多帧 -> 取最后一帧",
          try_parse_sensor_frame(b"111110 3 000000 7") == ("000000", 7))
    check("缺人数 -> None", try_parse_sensor_frame(b"111110") is None)
    check("人数超上限 -> 截断到 999",
          try_parse_sensor_frame(b"111110 12345") == ("111110", PED_COUNT_MAX))
    check("位串只有 5 位 -> None", try_parse_sensor_frame(b"11111 3") is None)
    buf = bytearray(b"xx111110 3yy000000 9")
    check("consume 取最后一帧", consume_sensor_frame(buf) == ("000000", 9))
    check("consume 后缓冲区清空", bytes(buf) == b"")

    print("== 6c. 语音词条与触发阈值 ==")
    check("理由 -> 词条键映射", reason_keys(
        ["左侧车道线为实线 —— xxx", "左转信号灯非绿色 —— xxx"]) ==
        ["r_left_solid", "r_left_light"])
    check("行人提示阈值为 5", PED_ALERT_THRESHOLD == 5)
    check("报警不锁定操作（锁定时长归零）", ALARM_LOCK_SEC <= 0)
    if HAS_VOICE and selftest_phrases is not None:
        miss = selftest_phrases()
        check(f"六种语言词条齐全（缺失 {len(miss)} 项）", not miss)
        check("六种语言齐全", LANG_CODES == ["zh", "en", "ar", "fr", "ru", "es"])
        check("报警文本以「警告」类前缀开头",
              len(alarm_text("左转", ["r_left_solid"], "zh")) > 4)
        # 低延迟关键：报警句必须能被预热的固定片段完整覆盖，否则播报时要现联网合成
        miss2 = selftest_parts()
        check(f"报警片段全部可预热（未覆盖 {len(miss2)} 项）", not miss2)
        for lg in LANG_CODES:
            for act in ("直行", "左转", "右转"):
                parts = alarm_parts(act, ["r_left_solid", "r_left_light"], lg)
                if not (parts and all(p.strip() for p in parts)):
                    check(f"{lg}/{act} 片段为空", False)
                    break
        check("六语言报警片段均非空", True)
        check("报警优先级高于提示", PRIO_ALARM < PRIO_TIP)
    else:
        print("  [SKIP] voice_prompts 不可用，跳过语音词条自检")

    print("== 7. 异常输入 ==")
    try:
        judge("掉头", st)
        check("未知动作应抛异常", False)
    except ValueError:
        check("未知动作应抛异常", True)
    legal, why = judge("倒车", st)       # v3.14：倒车始终合法（变量名避开计数器 ok）
    check("倒车始终合法", legal and not why)
    check("倒车有建议车速", SUGGEST_SPEED.get("倒车", 0) > 0)
    check("转向回退延时为 1 秒", TURN_RETURN_SEC == 1.0)
    check("瞬时动作含左转/右转/左变道/右变道",
          set(TURN_ACTIONS) == {"左转", "右转", "左变道", "右变道"})
    check("提示类语音冷却 >= 8 秒", TIP_COOLDOWN_SEC >= 8.0)

    print("== 7b. 变道动作（v3.21）：判定与左转/右转完全一致 ==")
    for a, light_key, solid_key in (("左变道", "left_green", "left_line_solid"),
                                     ("右变道", "right_green", "right_line_solid")):
        check(f"{a} 绿灯虚线 -> 放行", judge(a, st)[0])
        check(f"{a} 灯红 -> 禁行", not judge(a, dict(st, **{light_key: False}))[0])
        check(f"{a} 实线 -> 禁行", not judge(a, dict(st, **{solid_key: True}))[0])
        check(f"{a} 有建议车速", SUGGEST_SPEED.get(a, 0) > 0)
    check("左变道 与 左转 判定逐条相同",
          judge("左变道", st2) == judge("左转", st2))
    check("右变道 与 右转 判定逐条相同",
          judge("右变道", dict(st2, right_line_solid=True, right_green=False)) ==
          judge("右转", dict(st2, right_line_solid=True, right_green=False)))
    if HAS_VOICE and phrase is not None:
        for a in ("直行", "左转", "右转", "左变道", "右变道", "倒车", "停车"):
            check(f"按键播报词条 act_{a} 六语种齐全",
                  all(phrase(f"act_{a}", lg) not in ("", f"act_{a}")
                      for lg in LANG_CODES))

    print("== 8. 画面控制指令表 ==")
    check("指令共 6 条（已移除串口 2）", len(VIDEO_COMMANDS) == 6)
    check("指令数据为 0/1/3/4/5/6（夜景修复 2 已去除，其余下发值不变）",
          [c for c, _l, _t in VIDEO_COMMANDS] == ["0", "1", "3", "4", "5", "6"])
    check("按钮标签齐全且不带串口数字", [l for _c, l, _t in VIDEO_COMMANDS] ==
          ["画面切换", "单路亮度自适应", "单路去雾",
           "倒车辅助线", "拼接增亮", "拼接去雾"])
    check("所有标签均不含数字", all(not any(ch.isdigit() for ch in l)
                                    for _c, l, _t in VIDEO_COMMANDS))
    check("仅「画面切换」非开关",
          [t for _c, _l, t in VIDEO_COMMANDS] == [False] + [True] * 5)
    check("ASCII 编码 -> b'0'", VIDEO_COMMANDS[0][0].encode("ascii") == b"0")
    check("HEX 编码 -> b'\\x03'", bytes([int(VIDEO_COMMANDS[2][0], 16)]) == b"\x03")
    check("HEX 编码 -> b'\\x06'", bytes([int(VIDEO_COMMANDS[5][0], 16)]) == b"\x06")

    print("== 9. 双串口通道 ==")
    check("共两路串口 A / B", [t for t, _ in SERIAL_CHANNELS] == ["A", "B"])
    check("默认由 A 通道解析感知位串", SENSOR_CHANNEL == "A")
    check("默认指令下发到 B 通道", DEFAULT_TX_CHANNEL == "B")

    print("== 10. 倒车轨迹角度帧（盘古50K）==")
    check("帧长固定 6 字节", len(build_angle_frame(0)) == 6)
    check("直行 -> A00011", build_angle_frame(0) == b"A00011")
    check("右 15° -> A01511", build_angle_frame(15) == b"A01511")
    check("左 15° -> A11511", build_angle_frame(15, True) == b"A11511")
    check("左 30° -> A13011", build_angle_frame(30, True) == b"A13011")
    check("右 30° -> A03011", build_angle_frame(30, False) == b"A03011")
    check("个位 0 补零 -> A00511", build_angle_frame(5) == b"A00511")
    check("超限截断到 30", build_angle_frame(45, True) == b"A13011")
    check("负值截断到 0", build_angle_frame(-5) == b"A00011")
    check("0° 时方向位归一为 '0'", build_angle_frame(0, True) == b"A00011")
    check("末两位恒为 '11'", build_angle_frame(7)[4:] == b"11")
    check("角度上限与 FPGA 一致（val8 <= 30）", ANGLE_MAX_DEG == 30)

    print(f"\n结果: {ok} 通过 / {fail} 失败")
    return 0 if fail == 0 else 1


# ==================================================================== 界面
BG       = "#F5F7FA"
CARD     = "#FFFFFF"
INK      = "#1F2937"
SUBINK   = "#6B7280"
GREEN    = "#16A34A"
RED      = "#DC2626"
LINE     = "#E5E7EB"
BLUE     = "#2563EB"
HEAD     = "#111827"      # 顶栏深色
OK_SOFT  = "#DCFCE7"      # 布尔量为 True 的浅底
BAD_SOFT = "#F3F4F6"      # 布尔量为 False 的浅底
TIP_SOFT = "#FEF3C7"      # 仅提示类感知量为 True 的浅底（黄=提示，区别于"放行绿的绿"）
TIP_INK   = "#92400E"     # 提示类文字色
FIELD    = "#F9FAFB"      # 输入/只读区底色

FONT     = ("Microsoft YaHei UI", 10)
FONT_B   = ("Microsoft YaHei UI", 10, "bold")
FONT_S   = ("Microsoft YaHei UI", 9)
FONT_H   = ("Microsoft YaHei UI", 12, "bold")
FONT_ACT = ("Microsoft YaHei UI", 14, "bold")
FONT_MONO = ("Consolas", 9)

DISPLAY_BG = "#0B0F14"    # 显示界面底色（近黑，突出画面）
DISPLAY_FG = "#9CA3AF"    # 显示界面次要文字

PAD  = 14        # 卡片内边距
GAP  = 12        # 卡片间距
CM1  = 272       # 控制小窗里「感知状态」的参考宽度（仅文档意义，v3.11 起小窗自适应）
CM2  = 340       # 调试界面第 1 列最小宽度：决策状态 / 运行日志
CM3  = 440       # 调试界面第 2 列最小宽度：数据日志 / 数据发送

# ---- 右侧控制台（v3.7 / v3.21 固定停靠，不再悬浮拖动）----
PANEL_W = 360      # 控制台总宽（含外边距）
PANEL_MARGIN = 10  # 控制台与窗口边缘的间距
PANEL_MAX_H = 660  # 历史常量：早期浮动小窗的最大高度（v3.21 起固定占满整窗）

# ---- 串口链路参数：固定 115200 / 8 数据位 / 1 停止位 / 无校验 / 无流控 ----
SERIAL_BAUDRATE = 115200
SERIAL_BYTESIZE = 8
SERIAL_STOPBITS = 1
SERIAL_PARITY   = "N"
SERIAL_FIXED_DESC = "115200 · 8 数据位 · 1 停止位 · 无校验 · 无流控"

# ==================================================================== 显示界面（网口视频）
# 两条通路，延迟差一大截：
#   ① JPEG-UDP 直推（首选，v3.10）板端 main.cc --jpeg-udp-dest 直接把「带 OSD 的 BGR 帧」
#      压成 JPEG 分片走 UDP 发给 PC，不经过 ZLMediaKit / RTSP / H.264 解码 ——
#      这就是 V7 时代 pc_receiver.py 的做法，端到端只有「采集 + JPEG 编码 + 网络」。
#   ② RTSP（兜底）板端 ZLMediaKit 推流，链路多两跳（H.264 编码 → ZLM 转发 → PC 解码），
#      抖动/缓冲累积明显，实测比 ① 慢 100ms 以上。
# 默认 auto：先开 UDP，3 秒内收不到完整帧就自动退到 RTSP，无需人工干预。
VIDEO_SRC = (os.environ.get("SIM_VIDEO_SRC", "auto").strip().lower() or "auto")  # auto|udp|rtsp
UDP_JPEG_PORT = int(os.environ.get("SIM_UDP_PORT", "9999") or 9999)
UDP_STALE_S = 3.0              # 多久收不到完整帧就认为 UDP 通路不可用

# RK3568 上 ZLMediaKit 推出来的 RTSP 流。板端 IP / 端口变了改这里即可，
# 也可以用环境变量临时覆盖：set SIM_RTSP_URL=rtsp://192.168.1.50:8558/live/plate
DEFAULT_RTSP_URL = "rtsp://192.168.137.100:8558/live/plate"
RTSP_URL = os.environ.get("SIM_RTSP_URL", DEFAULT_RTSP_URL).strip() or DEFAULT_RTSP_URL

# ---- AI 识别闸门（v3.23）----
# 「四路拼接视频」或「倒车辅助线」打开时，画面已不是单路正视图，识别结果没有意义，
# 由上位机通知板端停掉 ADAS 推理（板端 --ai-ctrl-port，默认 9998）：
#     "AI 0" = 暂停识别（不取帧、不推理、不出框），"AI 1" = 恢复识别
# 通道走 UDP，与 JPEG-UDP 视频同一条网线，不占用串口、不依赖额外接线。
BOARD_IP_DEFAULT = (os.environ.get("SIM_BOARD_IP", "192.168.137.100").strip()
                    or "192.168.137.100")
AI_CTRL_PORT = int(os.environ.get("SIM_AI_CTRL_PORT", "9998") or 9998)
AI_BADGE_BG = "#B45309"          # 暂停提示条底色（琥珀色，压在画面上也看得清）

# ---- RTSP 拉流参数（v3.9：延迟优先）----
# RTSP 走 TCP：UDP 丢包会花屏 / 长时间卡死。
# 低延迟三件套（缺一不可）：
#   fflags;nobuffer   让 FFmpeg 不做输入缓冲，收到即解
#   flags;low_delay   解码器不做重排序等待（不等后续帧）
#   max_delay;0       关键项！旧值是 300000us = 300ms 的「抖动缓冲」，
#                     光是这一条就凭空多出 0.3 秒；置 0 后画面立刻跟上
# analyzeduration;0 + 小 probesize 让起播不再先啃一大段码流（影响首帧，不影响稳态）
RTSP_LOWLAT_OPT = ("rtsp_transport;tcp|fflags;nobuffer|flags;low_delay"
                   "|max_delay;0|analyzeduration;0|probesize;100000"
                   "|stimeout;5000000")
# 兜底：某些 FFmpeg 版本对上述选项组合会打不开流，这时退回保守参数（能播但不降延迟）
RTSP_SAFE_OPT = "rtsp_transport;tcp|stimeout;5000000"
RTSP_TRANSPORT_OPT = RTSP_LOWLAT_OPT
VIDEO_POLL_MS = 4              # 界面轮询新帧的间隔（有帧才贴图，不等同于播放帧率）
VIDEO_TARGET_FPS = 20          # 兼容旧配置项（已不用于绘制节流）
VIDEO_MARGIN_PX = 12           # 画面左右留白（v3.9：28 -> 12，画面更满）
VIDEO_PAD_TOP = 48             # 顶部标题条占位，画面不压在标题上
VIDEO_PAD_BOTTOM = 44          # 底部状态/快捷键占位
VIDEO_UPSCALE_MAX = 2.0        # 允许放大到源分辨率的多少倍（1.0 = 不放大，旧行为）
TOPBAR_H = 44                  # 显示界面顶部的黑色按钮条高度（按钮横排，不遮挡画面）


# ==================================================================== 视频 / 画面控制
# 画面控制指令：(下发数据, 按钮标签, 是否开关类)。按下按钮即通过串口把数据发出去。
#   · 数据值沿用板端定义的 0~6；「单路夜景修复」（2）已按需求移除，
#     保留的按钮下发数据不变（0/1/3/4/5/6），按钮上不再显示串口号；
#   · 开关类（第 3 项为 True）：按一下打开功能、按钮变红；再按一下关闭、按钮变黑。
#     两次下发的是同一个字节，由板端按"收到即翻转"解释，因此协议不需要新增字节。
#   · 「画面切换」是纯动作（False），不记忆状态、不变色。
VIDEO_COMMANDS = (
    ("0", "画面切换",        False),
    ("1", "单路亮度自适应",  True),
    ("3", "单路去雾",        True),
    ("4", "倒车辅助线",      True),
    ("5", "拼接增亮",        True),
    ("6", "拼接去雾",        True),
)

# 画面切换（指令 0）在 FPGA 内部是「收到即翻到下一个模式」的循环，不是单路/拼接两态：
#   上电默认 HDMI 单路 -> 按 1 下 -> 按 2 下 -> 按 3 下 -> 按 4 下 = 四路拼接 -> 再按回 HDMI。
# 也就是 5 个模式循环，四路拼接是第 5 个（下标 4）。上位机只能靠本地计数推断当前模式，
# 界面上显示「画面模式 n/5」并提供复位按钮，用于 FPGA 被硬件按键改过时的重新对齐。
VIEW_CYCLE = 5          # 画面模式总数（循环长度）
VIEW_QUAD_STEP = 4      # 下标：四路拼接视图（第 5 个模式）

def _bytesize(v: str):
    if not serial:
        return 8
    return {"5": serial.FIVEBITS, "6": serial.SIXBITS,
            "7": serial.SEVENBITS, "8": serial.EIGHTBITS}.get(v, serial.EIGHTBITS)


def _stopbits(v: str):
    if not serial:
        return 1
    return {"1": serial.STOPBITS_ONE, "1.5": serial.STOPBITS_ONE_POINT_FIVE,
            "2": serial.STOPBITS_TWO}.get(v, serial.STOPBITS_ONE)


def _parity(v: str):
    if not serial:
        return "N"
    return {"NONE": serial.PARITY_NONE, "EVEN": serial.PARITY_EVEN,
            "ODD": serial.PARITY_ODD, "MARK": serial.PARITY_MARK,
            "SPACE": serial.PARITY_SPACE}.get(v, serial.PARITY_NONE)


# ---- 两路独立串口：(标识, 用途) ----
#   A：接 RK3568，收 6 字节 1/0 感知位串
#   B：接盘古 50K，下发画面控制指令 / 手动数据
SERIAL_CHANNELS = (
    ("A", "RK3568 感知"),
    ("B", "盘古50K 下发"),
)

SENSOR_CHANNEL = "A"       # 只有这一路收到的字节会被解析为 6 路布尔量
DEFAULT_TX_CHANNEL = "B"   # 画面控制指令与手动发送的默认目标

# 各路"默认选中"的端口偏好（按次序匹配，命不中就退回列表第一个）。
# 2026-10-27 实测：RK3568 的 TTL 经 CH340 小板接在 COM10，板载 CP210x(COM4) 是
# 调试 console 口，收不到业务数据。写在这里只是为了启动后自动选中正确的一路，
# 用户在下拉框里随时可以改，不影响其它功能。
PREFERRED_PORTS = {
    "A": ("COM10", "CH340"),   # RK3568 感知：优先 CH340 小板
    "B": ("COM4", "CP210"),    # 盘古 50K 下发
}


# ==================================================================== 倒车轨迹角度帧
# 与盘古 50K（PGL50H）`traj_uart_ctrl` 模块对齐的角度帧，6 字节 ASCII / 115200-8-N-1：
#     'A' | dir | 十位 | 个位 | '1' | '1'
#   dir ：'0' = 向右、'1' = 向左；角度为 0 时方向位无意义，统一发 '0'
#   角度：00 ~ 30（度）。超过 30、末两位不是 '1'、或 5ms 内没发满 6 字节，FPGA 会
#         整帧丢弃并保持上一次角度（静默失败，不会有任何回执）。
#   例  ：A00011 直行 / A01511 右 15° / A11511 左 15° / A13011 左 30°
ANGLE_CHANNEL      = DEFAULT_TX_CHANNEL   # 角度帧只发给「盘古50K 下发」通道（B）
ANGLE_MAX_DEG      = 30                   # 左右最大转角，对应 FPGA 侧 val8 <= 30
ANGLE_KEEPALIVE_MS = 100                  # 串口打开后的持续发送周期（10 帧/秒）
ANGLE_MIN_TX_GAP   = 0.04                 # 鼠标拖动时即时帧的最小间隔（秒）


def build_angle_frame(deg: int, left: bool = False) -> bytes:
    """把角度打包成盘古50K 角度帧：'A' + dir + 十位 + 个位 + '1' + '1'。"""
    deg = max(0, min(ANGLE_MAX_DEG, int(deg)))
    d = b"1" if (left and deg > 0) else b"0"
    return b"A" + d + f"{deg:02d}".encode("ascii") + b"11"


class SerialChannel:
    """一路独立串口的运行时状态（链路参数固定 115200-8-N-1，无流控）。"""

    def __init__(self, tag: str, desc: str) -> None:
        self.tag = tag            # "A" / "B"
        self.desc = desc          # 用途说明，用于日志与界面
        self.ser: "serial.Serial | None" = None
        self.stop = threading.Event()
        self.queue: "queue.Queue[tuple[str, object]]" = queue.Queue()
        self.thread: "threading.Thread | None" = None
        self.rx_buffer = bytearray()   # 未消费的接收字节（仅解析通道会用到）
        self.last_rx_text = ""

    @property
    def is_open(self) -> bool:
        return bool(self.ser is not None and self.ser.is_open)


def _escape_ascii(s: str) -> bytes:
    """解析常用转义字符：\\r \\n \\t \\\\ \\xHH"""
    out = []
    i = 0
    while i < len(s):
        c = s[i]
        if c == "\\" and i + 1 < len(s):
            n = s[i + 1]
            if n == "r":
                out.append("\r")
                i += 2
                continue
            if n == "n":
                out.append("\n")
                i += 2
                continue
            if n == "t":
                out.append("\t")
                i += 2
                continue
            if n == "\\":
                out.append("\\")
                i += 2
                continue
            if n == "x" and i + 3 < len(s):
                hh = s[i + 2:i + 4]
                try:
                    out.append(chr(int(hh, 16)))
                    i += 4
                    continue
                except ValueError:
                    pass
        out.append(c)
        i += 1
    return "".join(out).encode("latin-1", errors="replace")


class AngleDial(tk.Canvas):
    """倒车轨迹角度盘：鼠标拖动 / 滚轮微调 / 双击回正。

    取值范围 **-30° ~ +30°**（负 = 向左，正 = 向右），按整数度保存，与盘古 50K
    角度帧一一对应（0 度即直行，方向位发 '0'）。
    视觉为 180° 半圆仪表盘：量程 ±30°，指针跟着鼠标转，1° 角度 = 3° 视觉角。
    """

    W, H = 240, 150                   # 画布尺寸
    CX, CY, R = 120, 104, 64          # 圆心与刻度半径
    SWEEP = 90.0 / ANGLE_MAX_DEG      # 每 1° 角度对应的视觉角度（3°/°）

    def __init__(self, parent, on_change=None, on_release=None, **kw) -> None:
        super().__init__(parent, width=self.W, height=self.H, bg=CARD,
                         bd=0, highlightthickness=0, cursor="hand2", **kw)
        self._value: int = 0
        self._drag: bool = False
        self.on_change = on_change
        self.on_release = on_release
        self._draw()
        self.bind("<Button-1>", self._press)
        self.bind("<B1-Motion>", self._move)
        self.bind("<ButtonRelease-1>", self._release)
        self.bind("<Double-Button-1>", lambda _e: self.set_value(0))
        self.bind("<MouseWheel>", self._wheel)

    # ---------------- 数值 ----------------
    @property
    def value(self) -> int:
        return self._value

    def set_value(self, v: float, notify: bool = True) -> None:
        v = max(-ANGLE_MAX_DEG, min(ANGLE_MAX_DEG, int(round(v))))
        if v == self._value:
            return
        self._value = v
        self._draw()
        if notify and self.on_change:
            self.on_change(v)

    # ---------------- 鼠标 ----------------
    def _angle_from_xy(self, x: float, y: float) -> float | None:
        """把鼠标位置换算成转角：向右为正、向左为负；圆心下方不响应。"""
        dx, dy = x - self.CX, self.CY - y     # dy 取反：屏幕 y 轴向下
        if dx * dx + dy * dy < 100:           # 太靠近圆心不响应，避免抖动
            return None
        if dy < -8:                           # 圆心以下不参与，避免跨下半圆跳变
            return None
        dev = 90.0 - math.degrees(math.atan2(dy, dx))   # 偏离正上方的视觉角
        return dev / self.SWEEP

    def _press(self, e) -> None:
        self._drag = True
        a = self._angle_from_xy(e.x, e.y)
        if a is not None:
            self.set_value(a)

    def _move(self, e) -> None:
        if not self._drag:
            return
        a = self._angle_from_xy(e.x, e.y)
        if a is not None:
            self.set_value(a)

    def _release(self, _e) -> None:
        was = self._drag
        self._drag = False
        if was and self.on_release:
            self.on_release()          # 松手时把最终角度立刻发出去（忽略限速）

    def _wheel(self, e) -> None:
        self.set_value(self._value + (1 if e.delta > 0 else -1))

    # ---------------- 绘制 ----------------
    def _draw(self) -> None:
        self.delete("all")
        cx, cy, r, k = self.CX, self.CY, self.R, self.SWEEP
        v = self._value
        accent = BLUE if v > 0 else "#8B5CF6"

        # 半圆底弧：Tk 角度 0°=3 点方向、逆时针为正，0~180 正好是上半圆
        self.create_arc(cx - r, cy - r, cx + r, cy + r, start=0, extent=180,
                        style="arc", outline=LINE, width=14)
        # 有效弧：从正上方（90°）转到当前角度（向右为顺时针，extent 取负）
        if v:
            self.create_arc(cx - r, cy - r, cx + r, cy + r,
                            start=90, extent=-v * k,
                            style="arc", outline=accent, width=14)

        # 刻度：每 5° 一根，整十加长；量程文字只标 -30 / 0 / +30
        for a in range(-ANGLE_MAX_DEG, ANGLE_MAX_DEG + 1, 5):
            rad = math.radians(90 - a * k)
            major = (a % 10 == 0)
            r0, r1 = (r - 20, r - 8) if major else (r - 17, r - 9)
            self.create_line(cx + r0 * math.cos(rad), cy - r0 * math.sin(rad),
                             cx + r1 * math.cos(rad), cy - r1 * math.sin(rad),
                             fill="#9CA3AF", width=1)
        for a in (-ANGLE_MAX_DEG, 0, ANGLE_MAX_DEG):
            rad = math.radians(90 - a * k)
            rr = r + 14
            self.create_text(cx + rr * math.cos(rad), cy - rr * math.sin(rad),
                             text=("0" if a == 0 else f"{a:+d}"),
                             fill="#9CA3AF", font=("Microsoft YaHei UI", 8))

        # 指针 + 中心毂
        rad = math.radians(90 - v * k)
        self.create_line(cx, cy,
                         cx + (r - 12) * math.cos(rad), cy - (r - 12) * math.sin(rad),
                         fill=INK, width=3, capstyle="round")
        self.create_oval(cx - 9, cy - 9, cx + 9, cy + 9,
                         fill="white", outline=INK, width=2)
        self.create_oval(cx - 3, cy - 3, cx + 3, cy + 3, fill=INK, outline="")

        # 角度文字
        txt = "直行" if v == 0 else f"{'左' if v < 0 else '右'} {abs(v)}°"
        self.create_text(cx, cy + 30, text=txt,
                         fill=INK if v == 0 else accent,
                         font=("Microsoft YaHei UI", 13, "bold"))


class JpegUdpViewer:
    """接收板端 JPEG 分片 UDP 直推 —— 低延迟通路（旧版 pc_receiver.py 的做法）。

    协议（板端 main.cc --jpeg-udp-dest 发送）：
        单个数据报 = [frame_id:4B BE][total:2B BE][chunk_idx:2B BE][分片数据 ≤1400B]
        拼起来的 payload = [meta_len:4B BE][meta json][JPEG 字节]
    JPEG 是把**已经画好检测框/文字的那一帧**（BGR）直接压出来的，所以画面内容与
    RTSP 完全一致，但少了 H.264 编码、ZLM 转发、PC 端 H.264 解码三跳，
    端到端只剩「采集 + JPEG 编码 + 网络 + JPEG 解码」。

    和 RtspViewer 保持同一套接口（available/start/stop/latest_img/set_target +
    status/frames/seq/fps/aspect/src_size），App 里可以无缝替换。
    """

    def __init__(self, port: int = UDP_JPEG_PORT) -> None:
        self.port = int(port)
        self._sock = None
        self._lock = threading.Lock()
        self._img = None            # 已缩放好的 PIL 图
        self._target = None
        self._th: threading.Thread | None = None
        self._dth: threading.Thread | None = None
        self._run = False
        # 「只留最新一帧」的交换槽：收包线程写、解码线程读，永远丢掉旧帧
        self._cv = threading.Condition()
        self._pending: bytes | None = None
        self._pending_fid = -1
        self.status = f"等待板端 UDP:{self.port} 数据…"
        self.frames = 0             # 成功解码上屏的帧数
        self.frames_rx = 0          # 收到的完整帧数（可能 > frames，说明解码跟不上被丢）
        self.packets = 0
        self.dropped = 0
        self.seq = 0
        self.fps = 0.0
        self.rx_fps = 0.0
        self.aspect = None
        self.src_size = (0, 0)
        self._last_t = 0.0
        self._last_rx_t = 0.0

    # ---------------- 对外（与 RtspViewer 同名同义） ----------------
    @property
    def available(self) -> bool:
        return cv2 is not None and np is not None and ImageTk is not None

    def start(self) -> None:
        if not self.available or self._run:
            return
        self._run = True
        self._th = threading.Thread(target=self._loop, name="jpeg-udp-rx", daemon=True)
        self._dth = threading.Thread(target=self._decode_loop, name="jpeg-udp-decode", daemon=True)
        self._th.start()
        self._dth.start()

    def stop(self) -> None:
        self._run = False
        with self._cv:
            self._cv.notify_all()
        for th in (self._th, self._dth):
            if th is not None and th.is_alive():
                th.join(timeout=1.5)
        self._th = None
        self._dth = None
        s = self._sock
        self._sock = None
        if s is not None:
            try:
                s.close()
            except Exception:
                pass

    def latest_img(self):
        with self._lock:
            return self.seq, self._img

    def set_target(self, width: int, height: int) -> None:
        size = (max(int(width), 2), max(int(height), 2))
        with self._lock:
            if size == self._target:
                return
            self._target = size
            self._img = None        # 尺寸变了，等下一帧重新缩放

    # ---------------- 收包线程：只管拼帧，拼好就扔进「最新帧」槽 ----------------
    def _loop(self) -> None:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # 板端一秒十几帧、每帧 200KB+：缓冲给小了在这几十毫秒里就会丢分片
            try:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
            except Exception:
                pass
            s.bind(("0.0.0.0", self.port))
            s.settimeout(0.3)
        except Exception as e:
            self.status = f"UDP {self.port} 端口打不开：{e}"
            self._run = False
            return
        self._sock = s
        self.status = "等待板端 UDP 数据…"

        buf: dict[int, dict] = {}
        last_ok = -1
        while self._run:
            try:
                data, _addr = s.recvfrom(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(data) < 9:
                continue
            try:
                fid, total, idx = struct.unpack("!IHH", data[:8])
            except Exception:
                continue
            if total <= 0 or idx >= total:
                continue
            self.packets += 1
            if fid <= last_ok:
                continue                        # 已经拼过/更旧的帧，直接丢（板端会把同一帧发两遍）
            ent = buf.get(fid)
            if ent is None:
                ent = {"total": total, "parts": {}}
                buf[fid] = ent
                if len(buf) > 3:                # 只留最近 3 帧，防止积压吃内存
                    for k in sorted(buf)[:-3]:
                        buf.pop(k, None)
                        self.dropped += 1
            ent["parts"][idx] = data[8:]
            if len(ent["parts"]) < ent["total"]:
                continue
            payload = b"".join(ent["parts"][i] for i in range(ent["total"]))
            buf.pop(fid, None)
            last_ok = fid

            # 关键：不在这里解码。直接覆盖「最新帧」槽，解码慢了就丢掉旧的，
            # 绝不排队 —— 排队 = 延迟越积越大（旧版卡顿/延迟的根源）。
            now = time.time()
            if self._last_rx_t:
                dt = now - self._last_rx_t
                if dt > 0:
                    inst = 1.0 / dt
                    self.rx_fps = inst if self.rx_fps <= 0 else self.rx_fps * 0.8 + inst * 0.2
            self._last_rx_t = now
            self.frames_rx += 1
            with self._cv:
                self._pending = payload
                self._pending_fid = fid
                self._cv.notify()

        try:
            s.close()
        except Exception:
            pass

    # ---------------- 解码线程：永远只取最新那一帧 ----------------
    def _decode_loop(self) -> None:
        while self._run:
            with self._cv:
                while self._run and self._pending is None:
                    self._cv.wait(0.3)
                payload = self._pending
                self._pending = None
            if payload is None:
                continue
            try:
                self._on_frame(payload)
            except Exception:
                self.dropped += 1

    def _on_frame(self, payload: bytes) -> None:
        if len(payload) < 5:
            self.dropped += 1
            return
        try:
            meta_len = struct.unpack("!I", payload[:4])[0]
            off = 4 + meta_len
            if off >= len(payload):
                self.dropped += 1
                return
            frame = cv2.imdecode(np.frombuffer(payload[off:], dtype=np.uint8),
                                 cv2.IMREAD_COLOR)
        except Exception:
            self.dropped += 1
            return
        if frame is None:
            self.dropped += 1
            return

        now = time.time()
        if self._last_t:
            dt = now - self._last_t
            if dt > 0:
                inst = 1.0 / dt
                self.fps = inst if self.fps <= 0 else self.fps * 0.8 + inst * 0.2
        self._last_t = now

        if self.aspect is None:
            h, w = frame.shape[:2]
            self.aspect = (w / h) if h else (16 / 9)
            self.src_size = (w, h)

        with self._lock:
            target = self._target
        if target is not None:
            h, w = frame.shape[:2]
            if (w, h) != target:
                frame = cv2.resize(frame, target, interpolation=cv2.INTER_LINEAR)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        pil = Image.fromarray(rgb)
        with self._lock:
            self._img = pil
            self.seq += 1
        self.frames += 1
        if self.status != "播放中（UDP 直推）":
            self.status = "播放中（UDP 直推）"


class RtspViewer:
    """后台线程拉 RTSP 流，主线程只取「最新一帧」。

    为什么必须在后台线程：cv2.VideoCapture.read() 在网络抖动时会阻塞好几秒，
    放在主线程会把整个界面冻住（串口收数、按钮响应全部停摆）。
    这里只保留最新帧，宁可丢帧也不堆积延迟——监控画面要的是「实时」不是「完整」。
    """

    def __init__(self, url: str) -> None:
        self.url = url
        self._cap = None
        self._lock = threading.Lock()
        self._frame = None
        self._rgb = None            # 已按 UI 尺寸缩放、且转成 RGB 的帧
        self._target = None         # (w, h) UI 需要的贴图尺寸，None = 不缩放
        self._th: threading.Thread | None = None
        self._run = False
        self.status = "未开始"
        self.frames = 0
        self.seq = 0                # 每解出一帧 +1，用来判断「有没有新帧」
        self.fps = 0.0              # 入流帧率（指数滑动平均）
        self.aspect = None          # 源画幅比例 w/h
        self.src_size = (0, 0)      # 源分辨率（限制上采样用）
        self._img = None            # 已缩放的 PIL 图（后台线程生成，主线程直接贴）
        self._last_t = 0.0

    # ---------------- 对外 ----------------
    @property
    def available(self) -> bool:
        """依赖是否齐全（opencv + pillow）"""
        return cv2 is not None and ImageTk is not None

    def start(self) -> None:
        if not self.available or self._run:
            return
        self._run = True
        self._th = threading.Thread(target=self._loop, name="rtsp-reader", daemon=True)
        self._th.start()

    def stop(self) -> None:
        self._run = False
        th = self._th
        if th is not None and th.is_alive():
            th.join(timeout=3.0)
        self._th = None
        self._release()

    def latest(self):
        """取最新一帧（BGR ndarray），没有就返回 None"""
        with self._lock:
            return self._frame

    def latest_rgb(self):
        """取最新一帧的 (序号, RGB ndarray)，供界面直接贴图"""
        with self._lock:
            return self.seq, self._rgb

    def latest_img(self):
        """取最新一帧的 (序号, PIL.Image)，主线程只需 ImageTk.PhotoImage"""
        with self._lock:
            return self.seq, self._img

    def set_target(self, width: int, height: int) -> None:
        """告诉拉流线程 UI 需要多大：缩放就在后台做完，主线程不再处理像素。"""
        size = (max(int(width), 2), max(int(height), 2))
        with self._lock:
            if size == self._target:
                return
            self._target = size
            self._rgb = None        # 尺寸变了，等下一帧重新缩放

    # ---------------- 内部 ----------------
    def _release(self) -> None:
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

    def _open(self):
        """先用低延迟参数开流；打不开再退回保守参数（宁可延迟也不能没有画面）。"""
        for opt in (RTSP_LOWLAT_OPT, RTSP_SAFE_OPT):
            try:
                # OpenCV 的 FFmpeg 后端在创建 VideoCapture 时才读这个环境变量
                os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = opt
            except Exception:
                pass
            cap = self._make_capture()
            if cap is not None:
                return cap
        return None

    def _make_capture(self):
        cap = None
        try:
            cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG)
        except Exception:
            cap = None
        if cap is None or not cap.isOpened():
            try:
                cap = cv2.VideoCapture(self.url)
            except Exception:
                cap = None
        if cap is None:
            return None
        if not cap.isOpened():
            try:
                cap.release()
            except Exception:
                pass
            return None
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)      # 缓冲压到最小，降低累积延迟
        except Exception:
            pass
        return cap

    def _sleep(self, sec: float) -> None:
        """可被 stop() 打断的 sleep"""
        end = time.time() + sec
        while self._run and time.time() < end:
            time.sleep(0.05)

    def _loop(self) -> None:
        backoff = 1.0
        while self._run:
            if self._cap is None:
                self.status = "连接中…"
                self._cap = self._open()
                if self._cap is None:
                    self.status = f"连接失败，{backoff:.0f}s 后重试"
                    self._sleep(backoff)
                    backoff = min(backoff * 2, 8.0)
                    continue
                backoff = 1.0
                self.status = "播放中"
            ok, frame = False, None
            try:
                ok, frame = self._cap.read()
            except Exception:
                ok, frame = False, None
            if not ok or frame is None:
                self._release()
                self.status = "流中断，重连中"
                self._sleep(1.0)
                continue
            now = time.time()
            if self._last_t:
                dt = now - self._last_t
                if dt > 0:
                    inst = 1.0 / dt
                    self.fps = inst if self.fps <= 0 else self.fps * 0.8 + inst * 0.2
            self._last_t = now
            if self.aspect is None:
                h, w = frame.shape[:2]
                self.aspect = (w / h) if h else (16 / 9)
                self.src_size = (w, h)
            # 缩放 + 转 RGB 都在本线程做：主线程只拿到「可以直接贴」的数组
            target = self._target
            if target is not None:
                h, w = frame.shape[:2]
                if (w, h) != target:
                    frame = cv2.resize(frame, target, interpolation=cv2.INTER_LINEAR)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pil = Image.fromarray(rgb)      # 转 PIL 也放后台，主线程只做 Tk 贴图
            with self._lock:
                self._frame = frame
                self._rgb = rgb
                self._img = pil
                self.seq += 1
            self.frames += 1
            # 不主动 sleep：read() 本身会被码流帧率阻塞，主动 sleep 只会增加延迟


# ==================================================================== 3D 虚拟路况（WebView2 嵌入）
class WebView3D:
    """把 3D/虚拟路况视图.html 用 WebView2 渲染出来，并作为子窗口嵌进 Tk 容器。

    为什么用 WebView2 而不是直接嵌 Edge 窗口：WebView2 是纯渲染内核，
    没有浏览器标题条 / 地址栏，也不存在「窗口改父后 Chromium 内部偏移」的问题，
    WebGL（three.js）照常走 GPU。DLL 直接用项目 3D/webview2/ 里的三件套
    （Microsoft.Web.WebView2.Core / WinForms / WebView2Loader），依赖只有 pythonnet。

    线程模型：WinForms 消息循环跑在一条后台 STA 线程上（WebView2 要求 STA），
    Tk 主线程只通过 user32 对窗口句柄做 SetParent / MoveWindow / ShowWindow。
    """

    def __init__(self, html_path: str, profile_dir: str) -> None:
        self.html_path = str(html_path)
        self.profile_dir = str(profile_dir)
        self.state = "idle"          # idle -> starting -> ready / failed
        self.error = ""
        self.hwnd = 0                # WinForms 窗口句柄（创建后由线程填好）
        self._shown = False
        self._th: threading.Thread | None = None
        self._form = None
        self._wv = None

    # ---- Win32 封装 ----
    @staticmethod
    def _u32():
        u32 = ctypes.windll.user32
        u32.SetParent.argtypes = (ctypes.c_void_p, ctypes.c_void_p)
        return u32

    def start(self) -> None:
        if self._th is not None and self._th.is_alive():
            return
        self.state = "starting"
        self._th = threading.Thread(target=self._run, name="webview-3d", daemon=True)
        self._th.start()

    def _run(self) -> None:
        try:
            import ctypes as _c
            # WebView2 要求 STA 线程；pythonnet 初始化过的线程可能是 MTA
            _c.windll.ole32.CoInitializeEx(None, 0x2)      # COINIT_APARTMENTTHREADED
            import clr
            base = Path(__file__).resolve().parent
            lib = base / "3D" / "webview2"
            if not lib.exists():                            # 兜底：pywebview 自带的一份
                lib = Path(sys.prefix) / "Lib" / "site-packages" / "webview" / "lib"
            os.add_dll_directory(str(lib))
            os.environ["PATH"] = str(lib) + os.pathsep + os.environ.get("PATH", "")
            clr.AddReference(str(lib / "Microsoft.Web.WebView2.WinForms.dll"))
            from System import Uri
            from System.Drawing import Point
            from System.Windows.Forms import (Application, Form, FormBorderStyle,
                                              FormStartPosition, DockStyle)
            from Microsoft.Web.WebView2.WinForms import (WebView2,
                                                         CoreWebView2CreationProperties)

            Application.EnableVisualStyles()
            form = Form()
            form.FormBorderStyle = getattr(FormBorderStyle, "None")
            form.StartPosition = FormStartPosition.Manual
            form.Location = Point(-32000, -32000)            # 嵌入前别闪现在屏幕上
            form.Width, form.Height = 898, 552
            wv = WebView2()
            props = CoreWebView2CreationProperties()
            props.UserDataFolder = self.profile_dir
            props.AdditionalBrowserArguments = ("--disable-sync --no-first-run "
                                                "--no-default-browser-check")
            wv.CreationProperties = props
            wv.Dock = DockStyle.Fill

            def on_init(_s, args) -> None:
                # 事件在 WinForms 线程回调，只改原子状态，别碰 Tk
                if args.IsSuccess:
                    self.state = "ready"
                else:
                    self.state = "failed"
                    self.error = str(args.InitializationException)[:300]
            wv.CoreWebView2InitializationCompleted += on_init

            def on_shown(_s, _e) -> None:
                try:
                    wv.EnsureCoreWebView2Async()
                    wv.Source = Uri(self.html_path)
                except Exception as e:                       # pragma: no cover
                    self.state = "failed"
                    self.error = f"启动 WebView2 失败：{e}"
            form.Shown += on_shown
            form.Controls.Add(wv)
            form.Show()
            self._form, self._wv = form, wv
            self.hwnd = int(form.Handle.ToInt64())
            Application.Run(form)                            # 阻塞在消息循环上
        except Exception as e:                               # pragma: no cover
            self.state = "failed"
            self.error = f"3D 渲染模块加载失败：{e}"

    # ---- 由 Tk 主线程调用的窗口操作 ----
    def embed(self, parent_hwnd: int) -> None:
        """把 WebView2 窗口变成 Tk 容器的子窗口（幂等）。"""
        if not self.hwnd:
            return
        u32 = self._u32()
        GetWindowLongPtrW = getattr(u32, "GetWindowLongPtrW", u32.GetWindowLongW)
        SetWindowLongPtrW = getattr(u32, "SetWindowLongPtrW", u32.SetWindowLongW)
        GetWindowLongPtrW.restype = ctypes.c_ssize_t
        SetWindowLongPtrW.restype = ctypes.c_ssize_t
        SetWindowLongPtrW.argtypes = (ctypes.c_void_p, ctypes.c_int, ctypes.c_ssize_t)
        WS_CHILD, WS_VISIBLE = 0x40000000, 0x10000000
        WS_CAPTION, WS_THICKFRAME, WS_POPUP = 0x00C00000, 0x00040000, 0x80000000
        u32.SetParent(self.hwnd, parent_hwnd)
        style = GetWindowLongPtrW(self.hwnd, -16)
        SetWindowLongPtrW(self.hwnd, -16,
                          (style & ~(WS_CAPTION | WS_THICKFRAME | WS_POPUP))
                          | WS_CHILD | WS_VISIBLE)
        u32.SetWindowPos(self.hwnd, 0, 0, 0, 0, 0, 0x0064)   # FRAMECHANGED|NOZORDER|SHOW

    def fit(self, w: int, h: int) -> None:
        if self.hwnd and w > 10 and h > 10:
            SWP_FRAMECHANGED, SWP_NOZORDER, SWP_SHOWWINDOW = 0x0020, 0x0004, 0x0040
            self._u32().SetWindowPos(self.hwnd, 0, 0, 0, int(w), int(h),
                                     SWP_FRAMECHANGED | SWP_NOZORDER | SWP_SHOWWINDOW)

    def show(self) -> None:
        if self.hwnd:
            self._u32().ShowWindow(self.hwnd, 5)             # SW_SHOW
            self._shown = True

    def hide(self) -> None:
        if self.hwnd:
            self._u32().ShowWindow(self.hwnd, 0)             # SW_HIDE
            self._shown = False

    def stop(self) -> None:
        if self.hwnd:
            try:
                self._u32().PostMessageW(self.hwnd, 0x0010, 0, 0)   # WM_CLOSE
            except Exception:
                pass


class App(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("智能驾驶 · 交规决策模拟器")
        self.configure(bg=BG)
        # v3.9：默认按屏幕大小铺开（旧版固定 1360x880，1080p 屏上视频只能占中间一小块）。
        # 想全屏按 F11，想还原按 F11 再按一次。
        self._init_geometry()
        self.minsize(1200, 740)
        self.resizable(True, True)

        # ---------------- 状态变量 ----------------
        # v3.14：启动时默认就是「停车」（action=None 即停车状态）
        self.state: dict[str, bool] = {
            k: (DEFAULT_SENSOR_BITS[i] == "1")
            for i, (k, _) in enumerate(SENSORS)
        }
        self.speed: float = 0.00
        self.action: str | None = None          # None = 停车（无行驶意图）
        self.action_legal: bool = False      # 当前意图是否合规（不合规只提示，不拦截）
        self.alarm_until: float = 0.0
        self._alarm_tick = 0
        self._notice_text: str = ""          # 按键下方那行黄色提示的文字

        # 左/右转自动回退：转向是瞬时动作，TURN_RETURN_SEC 秒后恢复到按下前的状态
        self._turn_return_state: str | None = None   # 回退目标：'直行' 或 None(停车)
        self._turn_return_after = None               # 待触发的 after 定时器 id
        self._turn_return_deadline: float = 0.0      # 回退截止时刻（状态行倒计时用）

        # 行人数量：串口帧「6 位位串 + 空格 + 人数」中的人数部分
        self.ped_count: int = DEFAULT_PED_COUNT
        # 报警提示条（控制小窗里那行黄字）到什么时候自动消失
        self._notice_until: float = 0.0
        # 上一次报警的特征串（动作 + 原因），用于避免同一违规反复播报
        self._alarm_sig: str | None = None
        # 提示类语音的闭锁：条件成立只播报一次，条件解除后复位
        self._voice_latch: dict[str, bool] = {"crosswalk": False, "pedestrian": False}
        # 提示类语音的上次播报时刻（冷却用，防止条件抖动时反复播报）
        self._tip_last: dict[str, float] = {"crosswalk": 0.0, "pedestrian": 0.0}
        # 五个动作按钮（报警不禁用，始终保持可点）
        self.action_btns: list[tk.Button] = []
        # 感知状态徽标（建在控制小窗里，建好之前 refresh_sensors 会跳过）
        self.sensor_labels: dict[str, tk.Label] = {}

        # ---------------- 人声语音 ----------------
        self.voice_lang: str = DEFAULT_VOICE_LANG
        self.speaker = None
        if HAS_VOICE and Speaker is not None:
            try:
                self.speaker = Speaker(
                    on_log=lambda m: self.write_log(m, "info"),
                    lang=self.voice_lang,
                    script_path=os.path.join(
                        os.path.dirname(os.path.abspath(__file__)), "sapi_speak.ps1"),
                )
            except Exception as _e:
                self.speaker = None
                self._voice_init_error = str(_e)
            else:
                self._voice_init_error = ""
                # Speaker 内部会在后台线程并发预热全部六种语言（当前语言优先），
                # 这里再显式触发一次，确保当前语言排在队首。
                self.speaker.prefetch(self.voice_lang)

        # ---------------- 串口变量（A=RK3568 感知 / B=盘古50K 下发）----------------
        self.chan: dict[str, SerialChannel] = {
            tag: SerialChannel(tag, desc) for tag, desc in SERIAL_CHANNELS
        }
        self.tx_count = 0
        self.rx_count = 0
        # 画面控制指令（开关类按钮）的当前状态与按钮引用：code -> bool / code -> tk.Button
        self._cmd_on: dict[str, bool] = {}
        self._cmd_btns: dict[str, "tk.Button"] = {}
        # AI 识别闸门（v3.23）：「画面切换」(0) 是板端收到即翻转的纯动作，这里按
        # 翻转奇偶记住当前是否停在四路拼接视图；「倒车辅助线」(4) 的状态在
        # _cmd_on["4"] 里。任意一项成立 -> 通知板端暂停 ADAS 识别。
        self._quad_view: bool = False
        # v3.25 画面模式计数：FPGA 是 5 态循环（HDMI 单路 -> … -> 四路拼接 -> 回 HDMI），
        # 只有下标 == VIEW_QUAD_STEP 这一个模式才是四路拼接，才暂停 AI 识别。
        self._view_step: int = 0
        self._view_var = None                # 界面上「画面模式 n/5」文字
        self._ai_manual_off: bool = False    # v3.24 手动关闭 AI 识别（按钮开关）
        self._ai_btn = None                  # 手动开关按钮引用（构建界面时赋值）
        self._ai_suspended: bool = False
        self._ai_sock = None                 # 懒创建的 UDP 套接字
        self._ai_last_sent = None            # 上一次成功下发的状态（None=从未）
        self._ai_err_logged = False          # 下发失败只记一次日志
        self._ai_gate_logged = ""            # 上一次写日志的原因，避免刷屏
        # 接收历史：(通道标识, 原始字节)，用于切换 ASCII/HEX 时整体重绘
        self._rx_history: list[tuple[str, bytes]] = []
        self._rx_history_max = 3000

        # ---------------- 倒车轨迹角度（底部角度盘 -> 串口 B）----------------
        self.angle_deg: int = 0          # 0 ~ 30，整数度
        self.angle_left: bool = False    # True = 向左，False = 向右
        self._angle_tx_t: float = 0.0    # 上一次发送时刻（拖动限速用）
        self._angle_last_frame: bytes | None = None
        self._angle_tx_frames: int = 0

        # ---------------- 大画面 / 右侧控制台（v3.8 / v3.21）----------------
        # 大画面有两页：display = 实时视频，debug = 感知/决策/数据日志。
        # v3.21：控制台固定停靠在窗口右侧（place），大画面只在左侧区域内居中，
        # 因此画面永远不会被控制台压住；原来的悬浮拖动已取消。
        self._page: str = "display"
        self._panel_on: bool = True
        self._panel_pos = None      # 历史字段，保留兼容（已不再用于拖动定位）
        self._panel_drag = None     # 历史字段，保留兼容（拖动功能已取消）
        # v3.10 视频源：优先 JPEG-UDP 直推（低延迟），收不到再自动退 RTSP
        self._udp_viewer = JpegUdpViewer(UDP_JPEG_PORT)
        self._rtsp_viewer = RtspViewer(RTSP_URL)
        self._src_is_udp = (VIDEO_SRC != "rtsp")
        self.viewer = self._udp_viewer if self._src_is_udp else self._rtsp_viewer
        self._src_t = time.time()   # 本次视频源启动时刻（用于 UDP 超时回退判定）
        self._photo = None          # 持有当前帧，防止被 GC 回收后画面变黑
        self._painted_seq = -1      # 已贴到屏幕上的帧号（判重，避免重复绘制）
        self._status_t = 0.0        # 状态栏节流
        self._fullscreen = False
        # Windows 默认定时器精度 15.6ms，会把 after(4) 拖成 ~16ms；提到 1ms 才能跟满帧
        self._timer_period = False
        try:
            import ctypes
            ctypes.windll.winmm.timeBeginPeriod(1)
            self._timer_period = True
        except Exception:
            pass

        self._build_style()
        self._build_ui()
        if self.speaker is not None:
            self.write_log(f"人声提示已就绪，可用后端：{self.speaker.describe()}", "ok")
        else:
            self.write_log("人声提示不可用（缺少 voice_prompts / 依赖），已回退蜂鸣", "err")
        self._tick()
        self._poll_serial()
        self._video_tick()

        self.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------- 样式 ----------------
    def _init_geometry(self) -> None:
        """按屏幕尺寸开一个「接近最大化」的窗口：画面尽量大，又留出任务栏/边框。

        取屏幕 96%（宽）/ 92%（高），并居中；小屏（<1280x800）时不小于最小尺寸。
        """
        try:
            sw = self.winfo_screenwidth()
            sh = self.winfo_screenheight()
        except Exception:
            self.geometry("1360x880")
            return
        w = max(int(sw * 0.96), 1200)
        h = max(int(sh * 0.92), 740)
        w = min(w, sw - 8)
        h = min(h, sh - 60)          # 留出任务栏
        x = max((sw - w) // 2, 0)
        y = max((sh - h) // 2 - 12, 0)
        self.geometry(f"{w}x{h}+{x}+{y}")

    def _build_style(self) -> None:
        s = ttk.Style()
        try:
            s.theme_use("clam")
        except tk.TclError:
            pass
        s.configure("TFrame", background=BG)
        s.configure("TCombobox", font=FONT, padding=(4, 3))
        s.configure("TButton", font=FONT, padding=(6, 4))
        s.configure("Vertical.TScrollbar", troughcolor=BG, background=LINE)

    # ---------------- 布局辅助控件 ----------------
    def _card(self, parent) -> tk.Frame:
        """统一白底描边卡片"""
        return tk.Frame(parent, bg=CARD, highlightthickness=1,
                        highlightbackground=LINE, bd=0)

    def _card_head(self, card: tk.Frame, title: str, hint: str = "",
                   accent: str = BLUE, wrap: int = 0) -> None:
        """卡片标题区：左侧竖色条 + 标题 + 次级说明

        wrap > 0 时给说明文字加换行宽度，避免长句把整列撑宽（列宽只由内容决定）。
        """
        head = tk.Frame(card, bg=CARD)
        head.pack(fill="x", padx=PAD, pady=(PAD, 8))
        tk.Frame(head, bg=accent, width=3, height=30).pack(side="left", padx=(0, 8))
        box = tk.Frame(head, bg=CARD)
        box.pack(side="left", fill="x", expand=True)
        tk.Label(box, text=title, bg=CARD, fg=INK, font=FONT_H, anchor="w").pack(fill="x")
        if hint:
            tk.Label(box, text=hint, bg=CARD, fg=SUBINK, font=FONT_S,
                     anchor="w", justify="left",
                     wraplength=wrap if wrap else 0).pack(fill="x")

    def _btn(self, parent, text: str, cmd, kind: str = "ghost", **kw) -> tk.Button:
        """统一按钮样式：primary / dark / danger / soft / ghost"""
        palette = {
            "primary": (BLUE,      "white",     "#1D4ED8", "white"),
            "dark":    (HEAD,      "white",     "#374151", "white"),
            "danger":  (RED,       "white",     "#B91C1C", "white"),
            "soft":    ("#EEF2FF", "#3730A3",   "#E0E7FF", "#3730A3"),
            "ghost":   (LINE,      INK,         "#D1D5DB", INK),
        }[kind]
        opts = dict(zip(("bg", "fg", "activebackground", "activeforeground"), palette))
        opts.update({"font": FONT_B, "padx": 10, "pady": 5})
        opts.update(kw)
        return tk.Button(parent, text=text, bd=0, relief="flat", cursor="hand2",
                         command=cmd, **opts)

    def _radios(self, parent, var, options, cmd=None) -> tk.Frame:
        """一行紧凑单选钮"""
        f = tk.Frame(parent, bg=CARD)
        for text, val in options:
            tk.Radiobutton(f, text=text, variable=var, value=val, bg=CARD, fg=INK,
                           font=FONT_S, selectcolor="white", bd=0,
                           highlightthickness=0, activebackground=CARD,
                           command=cmd).pack(side="left", padx=(0, 6))
        return f

    def _check(self, parent, text: str, var, cmd=None) -> tk.Checkbutton:
        return tk.Checkbutton(parent, text=text, variable=var, bg=CARD, fg=INK,
                              font=FONT_S, selectcolor="white", bd=0,
                              highlightthickness=0, activebackground=CARD,
                              command=cmd)

    def _field(self, parent, height: int, **kw) -> tuple:
        """带滚动条的多行文本区，返回 (容器, Text)"""
        outer = tk.Frame(parent, bg=CARD, highlightthickness=1,
                         highlightbackground=LINE)
        # width 只作最小保障；实际宽度由外层布局拉伸决定
        txt = tk.Text(outer, height=height, width=12, font=FONT_MONO, bd=0,
                      relief="flat", wrap="word", padx=8, pady=6, **kw)
        sb = tk.Scrollbar(outer, command=txt.yview, width=8)
        txt.config(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        txt.pack(side="left", fill="both", expand=True)
        return outer, txt

    # ---------------- 布局 ----------------
    def _build_ui(self) -> None:
        # 版面结构（v3.8）：
        #   大画面铺满整窗，两页叠在同一个网格里用 tkraise 切换：
        #       display_layer  实时画面（RK3568 网口视频），启动默认这一页
        #       debug_layer    调试界面（决策状态 / 运行日志 / 数据日志 / 数据发送）
        #   控制小窗是**浮在大画面上的浮窗**（place 定位，不占布局空间）：
        #       唤出/收起时大画面（尤其实时画面）尺寸完全不变，不会被挤小或重排。
        self.grid_rowconfigure(0, weight=1)
        self.grid_columnconfigure(0, weight=1)

        stack = tk.Frame(self, bg=BG)
        stack.grid(row=0, column=0, sticky="nsew")
        stack.grid_rowconfigure(0, weight=1)
        stack.grid_columnconfigure(0, weight=1)
        self._stack = stack

        # ---- 大画面 A：调试界面（先建，后建的显示层会在它上面）----
        self.debug_layer = tk.Frame(stack, bg=BG)
        self.debug_layer.grid(row=0, column=0, sticky="nsew")
        self.debug_layer.grid_columnconfigure(0, weight=1)
        self.debug_layer.grid_rowconfigure(0, weight=0)   # 顶栏
        self.debug_layer.grid_rowconfigure(1, weight=1)   # 主体
        self.debug_layer.grid_rowconfigure(2, weight=0)   # 状态栏
        self._build_topbar(self.debug_layer)
        self._build_body(self.debug_layer)
        self._build_statusbar(self.debug_layer)

        # ---- 大画面 B：实时画面 ----
        self.display_layer = tk.Frame(stack, bg=DISPLAY_BG)
        self.display_layer.grid(row=0, column=0, sticky="nsew")
        self._build_display_layer()

        # ---- 浮动控制小窗（盖在大画面上，不参与 grid）----
        self._build_panel()

        # 窗口尺寸变化时重新贴浮窗位置
        self.bind("<Configure>", self._layout_panel)
        # F1 收起/展开控制小窗，F2 大画面 实时画面 ⇄ 调试界面
        self.bind("<F1>", lambda _e: self._toggle_panel())
        self.bind("<F2>", lambda _e: self._toggle_page())
        self._set_panel(True)
        self._show_page("display")
        # AI 识别闸门心跳：启动先发一次「恢复」，之后每 2 秒对齐一次板端状态
        self.after(800, self._ai_gate_beat)

    # ================= 大画面 B：实时画面 / 3D 虚拟路况 =================
    def _build_display_layer(self) -> None:
        layer = self.display_layer

        # ---- 顶部黑色条状区域：三个圆形按钮横着并排（不遮挡画面）----
        #   [双画面=调试界面] [3D=虚拟路况] [≡=控制小窗]
        bar = tk.Frame(layer, bg=DISPLAY_BG)
        bar.place(x=0, y=0, relwidth=1.0, height=TOPBAR_H)
        self._top_bar = bar
        self._top_btns: dict[str, tk.Canvas] = {}
        for i, (kind, cmd, tip) in enumerate((
                ("debug", self._toggle_page, "切换调试界面（F2）"),
                ("3d", self._toggle_3d, "切换 3D 虚拟路况"),
                ("menu", self._toggle_panel, "控制小窗（F1）"))):
            b = self._icon_btn(bar, kind=kind, cmd=cmd, tip=tip)
            b.pack(side="left", padx=(14 if i == 0 else 8, 0), pady=4)
            self._top_btns[kind] = b
        self._top_title = tk.Label(bar, text="智能驾驶 · 实时画面", bg=DISPLAY_BG,
                                   fg="#E5E7EB", font=("Microsoft YaHei UI", 13, "bold"))
        self._top_title.pack(side="right", padx=20)

        # ---- 画面容器（黑条以下居中，随窗口缩放）----
        holder = tk.Frame(layer, bg=DISPLAY_BG)
        holder.place(relx=0.5, rely=0.5, y=TOPBAR_H // 2, anchor="center")
        self._video_holder = holder
        self._video_label = tk.Label(holder, bg=DISPLAY_BG)
        self._video_label.pack()

        # ---- 3D 虚拟路况容器（WebView2 嵌这里；先建，后建的顶栏压在它上面）----
        self._host3d = tk.Frame(layer, bg="#0B0F14")
        self._host3d.place(x=0, y=TOPBAR_H, relwidth=1.0, relheight=1.0,
                           height=-TOPBAR_H, anchor="nw")
        self._host3d.bind("<Configure>", self._fit_3d)
        self._load3d_var = tk.StringVar(value="")
        tk.Label(self._host3d, textvariable=self._load3d_var, bg="#0B0F14",
                 fg="#9CA3AF", font=FONT_B, justify="center").place(
            relx=0.5, rely=0.5, anchor="center")
        self.wv3d = WebView3D(
            str(Path(__file__).resolve().parent / "3D" / "虚拟路况视图.html"),
            str(Path(__file__).resolve().parent / "3D" / "_wv2profile"))
        self._embedded3d = False
        # 叠放层级（低 -> 高）：3D 容器 < 视频画面 < 顶部按钮条
        holder.lift()
        bar.lift()

        # v3.17：去掉左下角 RTSP/UDP 状态行与右下角快捷键提示行，
        # 画面下方只保留一个帧率徽章，且放到右下角（原提示行位置），
        # 面积小、位置贴边，不遮挡画面中央内容。
        self._fps_var = tk.StringVar(value="— FPS")
        self._fps_label = tk.Label(layer, textvariable=self._fps_var, bg=DISPLAY_BG,
                                   fg="#6B7280", font=("Consolas", 18, "bold"))
        self._fps_label.place(relx=1.0, rely=1.0, x=-20, y=-16, anchor="se")

        # AI 识别暂停提示条（v3.23）：四路拼接 / 倒车辅助线时贴在画面左上角，
        # 平时不占位（place_forget），需要时才出现。
        self._ai_badge = tk.Label(layer, text="", bg=AI_BADGE_BG, fg="#FFFFFF",
                                  font=("Microsoft YaHei UI", 11, "bold"),
                                  padx=10, pady=5)

        # 全屏：F11 或双击画面切换，Esc 退出
        self.bind("<F11>", lambda _e: self._toggle_fullscreen())
        self.bind("<Escape>", lambda _e: self._set_fullscreen(False))
        for w in (holder, self._video_label):
            w.bind("<Double-Button-1>", lambda _e: self._toggle_fullscreen())

        # 依赖缺失时给个明确提示，而不是默默黑屏
        if not self.viewer.available:
            miss = [n for n, m in (("opencv-python", cv2), ("pillow", ImageTk)) if m is None]
            tk.Label(layer, text="缺少依赖：" + "、".join(miss) + "\n请执行「安装依赖.bat」",
                     bg=DISPLAY_BG, fg="#FCA5A5", font=FONT_B,
                     justify="center").place(relx=0.5, rely=0.5, anchor="center")

    # ================= 固定右侧控制台 =================
    def _layout_panel(self, _e=None) -> None:
        """窗口尺寸变化时重新排布（控制台固定贴右，画面区在左侧）。"""
        self._apply_panel()
        self._layout_display_area()

    def _dock_reserve(self) -> int:
        """右侧固定控制台占用的宽度（收起 / 未建好时为 0）。"""
        if not getattr(self, "_panel_on", False) or not hasattr(self, "_panel"):
            return 0
        return PANEL_W + PANEL_MARGIN

    def _panel_geometry(self):
        """算出控制台的 (x, y, w, h)。

        v3.21：取消悬浮拖动，控制台**固定停靠在窗口右侧**、高度占满整窗；
        大画面（视频/3D）只在左侧区域内居中，永远不会被控制台压住。
        """
        try:
            W = self.winfo_width()
            H = self.winfo_height()
        except Exception:
            return None
        if W < 200 or H < 200:
            return None
        pw = min(PANEL_W, max(W - 80, 240))
        ph = max(H - 2 * PANEL_MARGIN, 200)
        x, y = W - pw - PANEL_MARGIN, PANEL_MARGIN
        return x, y, pw, ph

    def _layout_display_area(self) -> None:
        """把视频 / 3D 画面限制在控制台左侧的可视区内（保证不被挡住）。

        顶部按钮条同样收窄到左侧，否则右上角的标题会被控制台盖住。
        """
        try:
            W = self.winfo_width()
            H = self.winfo_height()
        except Exception:
            return
        if W < 200 or H < 200:
            return
        reserve = self._dock_reserve()
        left_w = max(W - reserve, 200)
        body_h = max(H - TOPBAR_H, 120)
        try:
            self._top_bar.place_configure(x=0, y=0, relx=0, rely=0, relwidth=0,
                                          width=left_w, height=TOPBAR_H)
        except Exception:
            pass
        try:
            self._video_holder.place_configure(
                relx=0, rely=0, anchor="center",
                x=left_w // 2, y=TOPBAR_H + body_h // 2)
        except Exception:
            pass
        try:
            self._host3d.place_configure(x=0, y=TOPBAR_H, relx=0, rely=0,
                                         relwidth=0, relheight=0,
                                         width=left_w, height=body_h)
        except Exception:
            pass
        self._place_fps_badge()

    def _apply_panel(self) -> None:
        """把算好的几何位置真正落到控件上（阴影层跟主体一起动）。

        v3.19 优化：几何没变化就直接跳过、lift 只在几何真正变化时做。
        v3.21：控制台固定停靠右侧，不再有拖动/自由位置。
        """
        if not self._panel_on or not hasattr(self, "_panel"):
            return
        geo = self._panel_geometry()
        if geo is None:
            return
        if geo == getattr(self, "_panel_last_geo", None):
            return
        self._panel_last_geo = geo
        x, y, pw, ph = geo
        try:
            self._panel_shadow.place(x=x + 3, y=y + 3, width=pw, height=ph)
            self._panel.place(x=x, y=y, width=pw, height=ph)
            self._panel.lift()
        except Exception:
            pass
        # 小窗动过就可能压到 / 让开右下角的帧率徽章，联动重摆
        self._place_fps_badge()

    def _place_fps_badge(self) -> None:
        """帧率徽章：贴在「左侧画面区」的右下角，控制台固定占右侧所以永不遮挡。"""
        if not hasattr(self, "_fps_label"):
            return
        x_off = -(self._dock_reserve() + 20)
        try:
            self._fps_label.place(relx=1.0, rely=1.0, x=x_off, y=-16, anchor="se")
            self._fps_label.lift()
        except Exception:
            pass

    # ---- v3.21：控制台已固定停靠，原「拖动移动 / 双击复位」整体取消 ----

    def _build_panel(self) -> None:
        """固定在右侧的控制台（v3.21 取消悬浮拖动）：行驶意图 / 语音 / 倒车角度 / 画面指令 / 串口。"""
        # 阴影层（浅色背景上给面板一点层次），先建再建主体，主体压在它上面
        self._panel_shadow = tk.Frame(self, bg="#000000", bd=0)
        self._panel = tk.Frame(self, bg=CARD, highlightthickness=1,
                               highlightbackground="#111827", bd=0)
        panel = self._panel

        # ---- 标题条（v3.21 起固定停靠，不再作为拖动条）----
        head = tk.Frame(panel, bg="#1F2937")
        head.pack(fill="x")
        # 装饰握把：知道这里原来是拖动条，现在只作视觉分隔（不再绑拖动事件）
        grip = tk.Canvas(head, width=12, height=24, bg="#1F2937",
                         highlightthickness=0, bd=0)
        for _r in range(3):
            for _c in range(2):
                grip.create_oval(1 + _c * 6, 6 + _r * 6, 4 + _c * 6, 9 + _r * 6,
                                 fill="#4B5563", outline="")
        grip.pack(side="left", padx=(10, 2))
        title = tk.Label(head, text="控 制 台", bg="#1F2937", fg="#E5E7EB",
                         font=("Microsoft YaHei UI", 11, "bold"),
                         padx=2, pady=7)
        title.pack(side="left")
        hint = tk.Label(head, text="固定停靠右侧", bg="#1F2937", fg="#6B7280",
                        font=FONT_S)
        hint.pack(side="left", padx=(8, 0))
        close = tk.Label(head, text="×", bg="#1F2937", fg="#9CA3AF", cursor="hand2",
                         font=("Microsoft YaHei UI", 14, "bold"), padx=12, pady=3)
        close.pack(side="right")
        close.bind("<Button-1>", lambda _e: self._set_panel(False))
        close.bind("<Enter>", lambda _e: close.configure(fg="white"))
        close.bind("<Leave>", lambda _e: close.configure(fg="#9CA3AF"))
        self._panel_head = head

        # ---- 可滚动内容区（窗口矮时也能看全）----
        holder = tk.Frame(panel, bg=CARD)
        holder.pack(fill="both", expand=True)
        canvas = tk.Canvas(holder, bg=CARD, bd=0, highlightthickness=0)
        vsb = tk.Scrollbar(holder, orient="vertical", command=canvas.yview, width=8)
        canvas.configure(yscrollcommand=vsb.set)
        vsb.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        inner = tk.Frame(canvas, bg=CARD)
        win = canvas.create_window((0, 0), window=inner, anchor="nw")
        inner.bind("<Configure>",
                   lambda _e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>",
                    lambda e: canvas.itemconfigure(win, width=e.width))

        def _wheel(e):
            canvas.yview_scroll(-1 if e.delta > 0 else 1, "units")
        canvas.bind("<Enter>", lambda _e: canvas.bind_all("<MouseWheel>", _wheel))
        canvas.bind("<Leave>", lambda _e: canvas.unbind_all("<MouseWheel>"))

        # ---- 六个功能小节 ----
        # 感知状态（v3.11 从调试界面左栏搬到这里：操作时直接看到感知条件）
        body = self._sec(inner, "感知状态", f"串口 {SENSOR_CHANNEL} 帧")
        self._build_sensor_section(body)

        body = self._sec(inner, "行驶意图", "违规只语音提示")
        self._build_action_panel(body)

        body = self._sec(inner, "语音播报", "六语种")
        self._build_voice_panel(body)

        body = self._sec(inner, "倒车轨迹角度", f"串口 {ANGLE_CHANNEL} 下行")
        self._build_angle_card(body)

        body = self._sec(inner, "画面控制指令")
        self._build_cmd_panel(body)

        body = self._sec(inner, "串口连接", "两路独立")
        self._build_link_panel(body)

        tk.Frame(inner, bg=CARD, height=PAD).pack(fill="x")

    def _sec(self, parent, title: str, hint: str = "") -> tk.Frame:
        """小窗内的小节：一条分隔线 + 标题（右可挂提示），返回正文容器。"""
        tk.Frame(parent, bg=LINE, height=1).pack(fill="x", padx=PAD, pady=(9, 0))
        head = tk.Frame(parent, bg=CARD)
        head.pack(fill="x", padx=PAD, pady=(7, 6))
        tk.Label(head, text=title, bg=CARD, fg=INK, font=FONT_B).pack(side="left")
        if hint:
            tk.Label(head, text=hint, bg=CARD, fg=SUBINK,
                     font=FONT_S).pack(side="right")
        body = tk.Frame(parent, bg=CARD)
        body.pack(fill="x", padx=PAD)
        return body

    def _build_action_panel(self, parent) -> None:
        """直行 / 左转 / 右转 / 左变道 / 右变道 / 倒车 / 停车 —— 七键三列。

        v3.21：新增「左变道 / 右变道」，排在右转之后、倒车之前；
        判定条件与左转/右转完全一致（同样的灯与标线约束），同样按瞬时动作处理。
        """
        grid = tk.Frame(parent, bg=CARD)
        grid.pack(fill="x")
        for c in range(3):
            grid.grid_columnconfigure(c, weight=1, uniform="act")
        self.action_btns = []
        acts = ("直行", "左转", "右转", "左变道", "右变道", "倒车", "停车")
        last_row = (len(acts) - 1) // 3
        for i, a in enumerate(acts):
            r, c = divmod(i, 3)
            # 3 列排布：末键独占一行时横跨补满（7 键 -> 最后一行 1 个，跨 3 列）
            span = 3 - c if i == len(acts) - 1 else 1
            kind = "danger" if a == "停车" else "dark"
            cmd = self.stop_car if a == "停车" else (lambda x=a: self.do_action(x))
            btn = self._btn(grid, a, cmd, kind=kind, font=FONT_ACT, pady=7)
            if span > 1:
                padx = (0, 0)
            else:
                padx = (0, 4) if c == 0 else ((4, 4) if c == 1 else (4, 0))
            btn.grid(row=r, column=c, columnspan=span, sticky="ew",
                     padx=padx, pady=(0, 5) if r < last_row else 0)
            self.action_btns.append(btn)

        # 行驶状态条：紧跟在七键下方，实时显示当前行驶状态（大字加粗、按状态着色）
        self._action_state_var = tk.StringVar(value="当前状态：停车")
        self._action_state_label = tk.Label(
            parent, textvariable=self._action_state_var,
            bg="#4B5563", fg="white",
            font=("Microsoft YaHei UI", 11, "bold"),
            anchor="center", justify="center",
            wraplength=PANEL_W - 2 * PAD - 24)
        self._action_state_label.pack(fill="x", pady=(6, 0), ipady=5)

        self._notice_var = tk.StringVar(value="")
        tk.Label(parent, textvariable=self._notice_var, bg=CARD, fg=TIP_INK,
                 font=FONT_S, anchor="w", justify="left",
                 wraplength=PANEL_W - 2 * PAD - 24).pack(fill="x", pady=(2, 0))

    def _build_voice_panel(self, parent) -> None:
        """语言切换 + 静音 + 试听（人声后端详情放在调试界面）。"""
        self._lang_name2code = {LANG_NAMES.get(c, c): c for c in LANG_CODES}
        self.voice_lang_var = tk.StringVar(
            value=LANG_NAMES.get(self.voice_lang, self.voice_lang))
        self.voice_combo = ttk.Combobox(
            parent, textvariable=self.voice_lang_var, state="readonly", width=11,
            values=[LANG_NAMES.get(c, c) for c in LANG_CODES])
        self.voice_combo.pack(side="left")
        self.voice_combo.bind("<<ComboboxSelected>>", self._on_voice_lang)
        self._voice_mute_var = tk.BooleanVar(value=False)
        self._check(parent, "静音", self._voice_mute_var,
                    self._on_voice_mute).pack(side="left", padx=(8, 0))
        self._btn(parent, "试听", self._voice_test, kind="ghost",
                  font=FONT_S, pady=2, padx=10).pack(side="right")
        self._voice_ready_t = 0.0

    def _icon_btn(self, parent, kind: str, cmd, tip: str) -> tk.Canvas:
        """圆形图标按钮（由父容器决定排列方向与间距）。

        用 Canvas 手绘而不是 Unicode 字符：⚙ / ⚑ 这类符号在部分字体下会显示成
        「豆腐块」，画出来则任何机器上都一致。
        """
        size, r = 34, 15
        c = tk.Canvas(parent, width=size, height=size, bg=DISPLAY_BG,
                      highlightthickness=0, cursor="hand2")
        oval = c.create_oval(size / 2 - r, size / 2 - r, size / 2 + r, size / 2 + r,
                             fill="#1F2937", outline="#374151", width=1)

        cx, cy = size / 2, size / 2
        if kind == "menu":                      # 三条横线 = 控制小窗
            for dy in (-5, 0, 5):
                c.create_line(cx - 6, cy + dy, cx + 6, cy + dy,
                              fill="#E5E7EB", width=2, tags="glyph")
        elif kind == "3d":                      # 圆圈中间一个 "3D"
            c.create_text(cx, cy, text="3D", fill="#E5E7EB",
                          font=("Segoe UI", 9, "bold"), tags="glyph")
        elif kind == "debug":                   # 两块叠起来的画面 = 切换大画面
            c.create_rectangle(cx - 7, cy - 6, cx + 4, cy + 2,
                               outline="#E5E7EB", width=2, tags="glyph")
            c.create_rectangle(cx - 3, cy - 2, cx + 7, cy + 6,
                               fill="#1F2937", outline="#E5E7EB", width=2,
                               tags="glyph")
        else:                                   # 左箭头 = 返回
            c.create_line(cx + 5, cy, cx - 6, cy, fill="#E5E7EB", width=2, tags="glyph")
            c.create_line(cx - 6, cy, cx - 1, cy - 5, fill="#E5E7EB", width=2, tags="glyph")
            c.create_line(cx - 6, cy, cx - 1, cy + 5, fill="#E5E7EB", width=2, tags="glyph")

        def enter(_e=None):
            if not getattr(c, "_active", False):
                c.itemconfigure(oval, fill="#374151")
        def leave(_e=None):
            if not getattr(c, "_active", False):
                c.itemconfigure(oval, fill="#1F2937")
        c.bind("<Enter>", enter)
        c.bind("<Leave>", leave)
        c.bind("<Button-1>", lambda _e: cmd())
        c._oval = oval                          # 外部可用 _set_btn_active 高亮当前模式
        return c

    def _set_btn_active(self, kind: str, on: bool) -> None:
        """顶栏按钮的「当前模式」高亮（例如 3D 模式下 3D 键点亮成蓝色）。"""
        c = self._top_btns.get(kind)
        if c is None:
            return
        c._active = bool(on)
        try:
            c.itemconfigure(c._oval,
                            fill="#2563EB" if on else "#1F2937",
                            outline="#93C5FD" if on else "#374151")
        except Exception:
            pass

    def _toggle_fullscreen(self) -> None:
        self._set_fullscreen(not self._fullscreen)

    def _set_fullscreen(self, on: bool) -> None:
        self._fullscreen = bool(on)
        try:
            self.attributes("-fullscreen", self._fullscreen)
        except Exception:
            pass

    # ---- 大画面切换 / 控制小窗开关 ----
    def _show_page(self, page: str) -> None:
        """把大画面切到 display（实时画面）/ 3d（虚拟路况）/ debug（调试界面）。"""
        if page not in ("display", "3d", "debug"):
            page = "display"
        if page == "debug":
            self._page = "debug"
            self.debug_layer.tkraise()
            self.title("智能驾驶 · 调试界面")
            self._set_btn_active("debug", True)
            self.wv3d.hide()                    # 看不见就别让 WebView2 白白渲染
            self._top_title.config(text="智能驾驶 · 调试界面")
            return

        self._page = page
        self.display_layer.tkraise()
        self._set_btn_active("debug", False)
        self._set_btn_active("3d", page == "3d")
        if page == "3d":
            self._top_title.config(text="智能驾驶 · 3D 虚拟路况")
            self.title("智能驾驶 · 3D 虚拟路况")
            self._show_3d()
        else:
            self._top_title.config(text="智能驾驶 · 实时画面")
            self.title("智能驾驶 · 实时画面")
            self._video_holder.lift()       # 从 3D 切回来：把画面抬回 3D 容器之上
            self._top_bar.lift()
            self._fps_label.lift()          # 帧率徽章跟着回到最上层（否则可能被画面盖住）
            self._paint_ai_badge(self._ai_gate_reason())   # AI 暂停提示条同理
            self.wv3d.hide()
            if self.viewer.available:
                self.viewer.start()

    def _show_display(self) -> None:
        self._show_page("display")

    def _show_debug(self) -> None:
        self._show_page("debug")

    def _toggle_page(self) -> None:
        """F2 / 双画面按钮：调试界面 ⇄ 显示界面/3D 界面（记住离开前的那一页）。"""
        if self._page == "debug":
            self._show_page(getattr(self, "_last_display_page", "display"))
        else:
            self._last_display_page = self._page
            self._show_page("debug")

    def _toggle_3d(self) -> None:
        """3D 按钮：显示界面 ⇄ 3D 虚拟路况。"""
        self._show_page("display" if self._page == "3d" else "3d")

    # ---- 3D 界面（WebView2 嵌入）----
    def _show_3d(self) -> None:
        self._host3d.tkraise()
        self._top_bar.lift()
        if self.wv3d.state == "idle":
            self._load3d_var.set("正在加载 3D 虚拟路况…\n（首次加载约需 3~8 秒）")
            self.wv3d.start()
            self.after(300, self._poll_3d)
        elif self.wv3d.state == "starting":
            self._load3d_var.set("正在加载 3D 虚拟路况…\n（首次加载约需 3~8 秒）")
            self.after(300, self._poll_3d)
        elif self.wv3d.state == "ready":
            if not self._embedded3d and self.wv3d.hwnd:
                try:
                    self.wv3d.embed(self._host3d.winfo_id())
                    self._embedded3d = True
                except Exception as e:
                    self.wv3d.state = "failed"
                    self.wv3d.error = f"嵌入失败：{e}"
            if self._embedded3d:
                self._load3d_var.set("")
                self.wv3d.show()
                self._fit_3d()
        if self.wv3d.state == "failed":
            self._load3d_var.set("3D 视图不可用：\n" + self.wv3d.error)

    def _poll_3d(self) -> None:
        """3D 加载期间的轮询：等 WebView2 就绪后把它嵌进容器。"""
        if self._page != "3d":
            return
        if self.wv3d.state == "starting":
            self.after(300, self._poll_3d)
            return
        self._show_3d()

    def _fit_3d(self, _e=None) -> None:
        if self._embedded3d:
            try:
                self.wv3d.fit(self._host3d.winfo_width(), self._host3d.winfo_height())
            except Exception:
                pass
    def _set_panel(self, on: bool) -> None:
        """收起 / 展开右侧固定控制台（F1 或标题条上的 × 切换）。

        v3.21：控制台固定停靠右侧，不再可拖动；收起后画面区自动铺满整窗。
        """
        self._panel_on = bool(on)
        if self._panel_on:
            self._panel_last_geo = None      # 收起再展开：强制重 place 一次
            self._apply_panel()
        else:
            self._panel_drag = None
            try:
                self._panel.place_forget()
                self._panel_shadow.place_forget()
            except Exception:
                pass
        self._layout_display_area()          # 画面区随控制台开合重排
        self._sync_debug_reserve()

    def _sync_debug_reserve(self) -> None:
        """调试界面给右侧控制台让出空间（否则日志会被挡住）。"""
        reserve = self._dock_reserve()
        try:
            self._debug_body.grid_configure(padx=(GAP, GAP + reserve))
            self._debug_topbar.grid_configure(padx=(0, reserve))
            self._debug_statusbar.grid_configure(padx=(0, reserve))
        except Exception:
            pass

    def _toggle_panel(self) -> None:
        self._set_panel(not self._panel_on)

    def _video_tick(self) -> None:
        """实时画面刷新：**有新帧才贴图**（不重复刷同一帧、也不漏掉新帧）。

        缩放 / 色彩转换都放在拉流线程里做（见 RtspViewer / JpegUdpViewer），主线程
        只剩 ImageTk 贴图这一下，单帧开销从 ~8ms 降到 ~2ms，画面跟着源码流走不再抖。
        """
        try:
            self._check_video_source()
            if self._page == "display":
                self._video_paint()
        except Exception as e:
            self.write_log(f"视频显示异常：{e}", "err")
        self.after(VIDEO_POLL_MS, self._video_tick)

    def _check_video_source(self) -> None:
        """UDP 直推开了却没收到帧（板端没加 --jpeg-udp-dest、或端口被占）→ 自动退 RTSP。

        只判断一次，退过去就不再回头：中途来回切会让画面反复黑一下。
        """
        if not self._src_is_udp or self.viewer is not self._udp_viewer:
            return
        if self._udp_viewer.frames > 0:
            return
        if time.time() - self._src_t < UDP_STALE_S:
            return
        udp_status = self._udp_viewer.status
        self._udp_viewer.stop()
        self._rtsp_viewer.start()
        self.viewer = self._rtsp_viewer
        self._src_is_udp = False
        self._src_t = time.time()
        self._painted_seq = -1
        self.write_log(f"UDP 直推不可用（{udp_status}），已回退 RTSP", "err")

    def _target_size(self) -> tuple[int, int]:
        """按源画幅比例算出贴图尺寸：**铺满可用区域**（v3.9 允许放大）。

        旧版「不超过源分辨率就不放大」会让 1280×720 的流在 1080p 屏上只占中间一小块，
        四周全是黑边；现在按窗口可用区域等比放大（上限 VIDEO_UPSCALE_MAX 倍），
        缩放依旧在后台线程做，主线程只贴图。
        """
        try:
            aw = max(self.display_layer.winfo_width() - self._dock_reserve()
                     - VIDEO_MARGIN_PX * 2, 320)
            # 顶部黑色按钮条(TOPBAR_H)以下才是画面区，底部给状态行留 VIDEO_PAD_BOTTOM
            ah = max(self.display_layer.winfo_height() - TOPBAR_H - VIDEO_PAD_BOTTOM, 180)
        except Exception:
            aw, ah = 960, 540
        ratio = self.viewer.aspect or (16 / 9)
        nw = aw
        nh = int(round(nw / ratio))
        if nh > ah:
            nh = ah
            nw = int(round(nh * ratio))
        # 放大有上限：再大也只是把像素拉糊，还白白多花缩放/贴图时间
        sw, sh = self.viewer.src_size
        if sw and sh:
            limit_w = sw * VIDEO_UPSCALE_MAX
            limit_h = sh * VIDEO_UPSCALE_MAX
            if nw > limit_w or nh > limit_h:
                scale = min(limit_w / nw, limit_h / nh)
                nw, nh = int(nw * scale), int(nh * scale)
        return max(nw - nw % 2, 2), max(nh - nh % 2, 2)

    def _video_paint(self) -> None:
        if not self.viewer.available:
            self.write_log("缺少视频依赖，无法拉流", "err")
            return

        nw, nh = self._target_size()
        self.viewer.set_target(nw, nh)          # 后台线程按此尺寸预缩放
        seq, img = self.viewer.latest_img()
        # v3.19：拖动小窗期间暂停贴图（v3.21 控制台固定后该分支恒不触发，
        # 保留判断仅作兼容）。帧继续在后台缓存。
        if (img is not None and seq != self._painted_seq
                and getattr(self, "_panel_drag", None) is None):
            # 实测新建 PhotoImage(≈5ms) 比 paste(6~8ms) 更快，直接重建
            photo = ImageTk.PhotoImage(img)
            self._video_label.configure(image=photo)
            self._photo = photo                 # 必须持有引用，否则被 GC 回收会黑屏
            self._painted_seq = seq

        now = time.time()
        if now - self._status_t > 0.5:
            self._status_t = now
            playing = str(self.viewer.status).startswith("播放中")
            self._paint_fps(playing)

    def _paint_fps(self, playing: bool) -> None:
        """刷新画面正下方的帧率徽章。

        数值取 viewer.fps（入流帧率的指数滑动平均），颜色按健康度分三档：
            ≥20 绿（跟得上 25fps 源） / 10~20 黄（开始掉帧） / <10 红（明显卡）
        未播放或无帧时显示「— FPS」并转灰，避免把 0.0 误读成"零帧率故障"。
        """
        try:
            fps = float(self.viewer.fps or 0.0)
        except Exception:
            fps = 0.0
        if not playing or fps <= 0.05:
            self._fps_var.set("— FPS")
            self._fps_label.configure(fg="#4B5563")
            return
        self._fps_var.set(f"{fps:.1f} FPS")
        if fps >= 20.0:
            color = "#34D399"
        elif fps >= 10.0:
            color = "#FBBF24"
        else:
            color = "#F87171"
        self._fps_label.configure(fg=color)

    # ---- 顶栏（调试界面）----
    def _build_topbar(self, parent=None) -> None:
        parent = parent or self
        bar = tk.Frame(parent, bg=HEAD)
        bar.grid(row=0, column=0, sticky="ew")
        self._debug_topbar = bar
        bar.grid_columnconfigure(0, weight=1)

        left = tk.Frame(bar, bg=HEAD)
        left.grid(row=0, column=0, sticky="w", padx=16, pady=11)
        tk.Label(left, text="智能驾驶 · 调试界面", bg=HEAD, fg="white",
                 font=("Microsoft YaHei UI", 15, "bold"), anchor="w").pack(side="left")
        tk.Label(left, text="v3.16", bg="#374151", fg="#E5E7EB", font=FONT_S,
                 padx=8, pady=2).pack(side="left", padx=(10, 0))

        self._link_var = tk.StringVar(value="● 串口未连接")
        self._link_label = tk.Label(bar, textvariable=self._link_var, bg="#374151",
                                    fg="#FCA5A5", font=FONT_B, padx=12, pady=4)
        self._link_label.grid(row=0, column=1, sticky="e", padx=(0, 10))

        self._btn(bar, "控制小窗", self._toggle_panel, kind="ghost",
                  padx=12).grid(row=0, column=2, sticky="e", padx=(0, 8))
        self._btn(bar, "实时画面  F2", self._show_display, kind="soft",
                  padx=12).grid(row=0, column=3, sticky="e", padx=(0, 16))

    # ---- 主体：两列（决策状态 + 运行日志 / 数据收发）----
    # 感知状态（v3.11）已搬到控制小窗，这里只剩决策与数据两块。
    def _build_body(self, parent=None) -> None:
        parent = parent or self
        body = tk.Frame(parent, bg=BG)
        body.grid(row=1, column=0, sticky="nsew", padx=GAP, pady=GAP)
        self._debug_body = body
        body.grid_columnconfigure(0, weight=3, minsize=CM2)
        body.grid_columnconfigure(1, weight=8, minsize=CM3)
        body.grid_rowconfigure(0, weight=1)

        # 每列各放一张满高卡片，互不挤占垂直空间
        col1 = tk.Frame(body, bg=BG)
        col1.grid(row=0, column=0, sticky="nsew", padx=(0, GAP))
        col1.grid_columnconfigure(0, weight=1)
        col1.grid_rowconfigure(0, weight=1)
        self._build_status_card(col1)

        col2 = tk.Frame(body, bg=BG)
        col2.grid(row=0, column=1, sticky="nsew")
        col2.grid_columnconfigure(0, weight=1)
        col2.grid_rowconfigure(0, weight=1)     # 数据日志：可伸缩
        col2.grid_rowconfigure(1, weight=0)     # 数据发送：固定高度
        self._build_rx_panel(col2)
        self._build_tx_panel(col2)

    # ---- 控制小窗：感知状态（v3.11 从调试界面左栏搬来，2 列 × 3 行紧凑徽标）----
    def _build_sensor_section(self, parent) -> None:
        grid = tk.Frame(parent, bg=CARD)
        grid.pack(fill="x")
        grid.grid_columnconfigure(0, weight=1, uniform="sen")
        grid.grid_columnconfigure(1, weight=1, uniform="sen")

        self.sensor_labels = {}
        last_row = (len(SENSORS) - 1) // 2
        for i, (key, name) in enumerate(SENSORS):
            r, c = divmod(i, 2)
            row = tk.Frame(grid, bg=FIELD, highlightthickness=1,
                           highlightbackground=LINE)
            row.grid(row=r, column=c, sticky="nsew",
                     padx=(0, 3) if c == 0 else (3, 0),
                     pady=(0, 3) if r < last_row else 0)
            tk.Label(row, text=SENSOR_SHORT_NAMES.get(key, name), bg=FIELD, fg=INK,
                     font=FONT_S, anchor="w").pack(side="left", padx=(7, 0), pady=2)
            lab = tk.Label(row, text="ON", bg=OK_SOFT, fg=GREEN,
                           font=("Microsoft YaHei UI", 9, "bold"), padx=4)
            lab.pack(side="right", padx=(0, 3), pady=2)
            self.sensor_labels[key] = lab

        # ---- 底部：位串摘要 + 行人数量（原来是调试界面左栏的两块内容）----
        foot = tk.Frame(grid, bg="#FFFBEB", highlightthickness=1,
                        highlightbackground="#FDE68A")
        foot.grid(row=last_row + 1, column=0, columnspan=2, sticky="ew",
                  pady=(5, 0))
        self._bits_var = tk.StringVar(value="位串 ------")
        tk.Label(foot, textvariable=self._bits_var, bg="#FFFBEB", fg=TIP_INK,
                 font=FONT_MONO).pack(side="left", padx=(7, 0), pady=4)

        self.ped_var = tk.StringVar(value=str(DEFAULT_PED_COUNT))
        self.ped_label = tk.Label(foot, textvariable=self.ped_var, bg="#FEF3C7",
                                  fg="#92400E", font=FONT_S,
                                  padx=6)
        self.ped_label.pack(side="right", padx=(0, 5), pady=3)
        tk.Label(foot, text="行人", bg="#FFFBEB", fg=SUBINK,
                 font=FONT_S).pack(side="right", padx=(0, 3))

        self.refresh_sensors()

    # ---- 中栏：决策状态 + 运行日志（动作按钮已移到右侧控制小窗）----
    def _build_status_card(self, parent) -> None:
        card = self._card(parent)
        card.grid(row=0, column=0, sticky="nsew")
        self._card_head(card, "决策状态", "感知状态在控制小窗 · 选方向后判定",
                        "#8B5CF6", wrap=CM2 - 2 * PAD - 12)

        self.status_var = tk.StringVar(value="已停车：请选择行驶方向")
        self.status_label = tk.Label(card, textvariable=self.status_var, bg=FIELD,
                                     fg=INK, font=FONT_B, anchor="nw", justify="left",
                                     wraplength=CM2 - 2 * PAD - 12, padx=10, pady=8,
                                     highlightthickness=1, highlightbackground=LINE)
        self.status_label.pack(fill="x", padx=PAD, pady=(0, 8))

        # 人声后端详情（语言/静音/试听都在右侧小窗，这里只做状态回显）
        self._voice_info_var = tk.StringVar(
            value=("人声后端：" + self.speaker.describe()) if self.speaker
            else "人声后端：不可用（蜂鸣兜底）")
        tk.Label(card, textvariable=self._voice_info_var, bg=CARD, fg=SUBINK,
                 font=FONT_S, anchor="w", justify="left",
                 wraplength=CM2 - 2 * PAD - 12).pack(fill="x", padx=PAD, pady=(0, 8))

        self._btn(card, "复位场景", self.reset, kind="ghost", pady=4).pack(
            fill="x", padx=PAD, pady=(0, 8))

        lg = tk.Frame(card, bg=CARD)
        lg.pack(fill="both", expand=True, padx=PAD, pady=(0, PAD))
        tk.Label(lg, text="运行日志", bg=CARD, fg=SUBINK, font=FONT_S,
                 anchor="w").pack(fill="x", pady=(0, 4))
        logbox, self.log = self._field(lg, 3, bg=FIELD, fg=INK)
        logbox.pack(fill="both", expand=True)
        self.log.tag_config("ok", foreground=GREEN)
        self.log.tag_config("err", foreground=RED)
        self.log.tag_config("warn", foreground="#B45309")
        self.log.tag_config("info", foreground=SUBINK)
        self.log.configure(state="disabled")

    # ---- 控制小窗：倒车轨迹角度盘（盘古50K 角度帧下行）----
    def _build_angle_card(self, parent) -> None:
        self.angle_dial = AngleDial(parent, on_change=self._on_angle_change,
                                    on_release=self._angle_flush)
        self.angle_dial.pack(pady=(0, 2))

        self.angle_frame_var = tk.StringVar(value="A00011")
        tk.Label(parent, textvariable=self.angle_frame_var, bg=CARD, fg="#8B5CF6",
                 font=("Consolas", 11, "bold")).pack()

        self.angle_state_var = tk.StringVar(value="串口 B 未打开")
        tk.Label(parent, textvariable=self.angle_state_var, bg=CARD, fg=SUBINK,
                 font=FONT_S).pack(pady=(2, 0))

        row = tk.Frame(parent, bg=CARD)
        row.pack(fill="x", pady=(8, 0))
        self._btn(row, "回正", self._center_angle, kind="soft",
                  font=FONT_S, pady=2, padx=10).pack(side="left")
        self._angle_auto_var = tk.BooleanVar(value=True)
        self._check(row, "持续发送", self._angle_auto_var,
                    self._refresh_angle_status).pack(side="left", padx=(8, 0))
        self.angle_tx_var = tk.StringVar(value="已发 0 帧")
        tk.Label(row, textvariable=self.angle_tx_var, bg=CARD, fg=SUBINK,
                 font=FONT_S).pack(side="right")

        self._refresh_angle_status()

    # ---- 角度盘：数值变化 -> 回显 + 即时下发 ----
    def _on_angle_change(self, v: int) -> None:
        self.angle_deg = abs(v)
        self.angle_left = v < 0
        self._refresh_angle_status()
        self._angle_send(immediate=True)

    def _center_angle(self) -> None:
        self.angle_dial.set_value(0)
        self._angle_flush()            # 回正立即下发，不等 100ms 持续帧

    def _angle_flush(self) -> None:
        """忽略限速把当前角度立刻发一帧（松手 / 回正）。"""
        self._angle_send(immediate=True, force=True)

    def _refresh_angle_status(self) -> None:
        """刷新帧回显与串口状态（卡片尚未建好时静默跳过）。"""
        if not hasattr(self, "angle_frame_var") or not hasattr(self, "_angle_auto_var"):
            return
        frame = build_angle_frame(self.angle_deg, self.angle_left)
        self.angle_frame_var.set(frame.decode("ascii"))

        ch = self.chan[ANGLE_CHANNEL]
        hz = 1000 // ANGLE_KEEPALIVE_MS
        if not ch.is_open:
            self.angle_state_var.set(f"串口 {ANGLE_CHANNEL} 未打开 · 打开后自动发送")
        elif self._angle_auto_var.get():
            self.angle_state_var.set(f"串口 {ANGLE_CHANNEL} 已连 · 持续 {hz} 帧/秒")
        else:
            self.angle_state_var.set(f"串口 {ANGLE_CHANNEL} 已连 · 仅变化时发送")
        self.angle_tx_var.set(f"已发 {self._angle_tx_frames} 帧")

    def _angle_send(self, immediate: bool = False, force: bool = False) -> None:
        """把当前角度按盘古50K 帧格式写出去。

        immediate=True 时受 40ms 限速（拖动过程中防刷爆串口）；
        force=True 时忽略限速（松手 / 点「回正」时把最终值立刻发出）。
        """
        ch = self.chan[ANGLE_CHANNEL]
        if not ch.is_open or ch.ser is None:
            return
        now = time.time()
        if immediate and not force and (now - self._angle_tx_t) < ANGLE_MIN_TX_GAP:
            return                      # 拖动中限速，剩余变化交给持续帧兜底
        frame = build_angle_frame(self.angle_deg, self.angle_left)
        try:
            ch.ser.write(frame)   # type: ignore[union-attr]
        except Exception as e:
            self.write_log(f"角度帧发送失败：{e}", "err")
            return
        self._angle_tx_t = now
        self._angle_tx_frames += 1
        self.tx_count += len(frame)
        self.tx_count_var.set(f"TX: {self.tx_count}")
        if hasattr(self, "angle_tx_var"):
            self.angle_tx_var.set(f"已发 {self._angle_tx_frames} 帧")
        # 只在角度真正变化时记日志，避免 10Hz 持续帧刷屏
        if frame != self._angle_last_frame:
            self._angle_last_frame = frame
            self.write_log(
                f"倒车角度帧 -> 串口{ANGLE_CHANNEL}：{frame.decode('ascii')}"
                f"（{'左' if self.angle_left else '右'} {self.angle_deg}°）", "ok")

    # ---- 小窗：串口连接（A / B 两路，参数相同）----
    def _build_link_panel(self, parent) -> None:
        head = tk.Frame(parent, bg=CARD)
        head.pack(fill="x", pady=(0, 4))
        self._btn(head, "刷新端口", self._refresh_ports, kind="ghost",
                  font=FONT_S, pady=1, padx=8).pack(side="right")

        self._port_var: dict[str, tk.StringVar] = {}
        self.port_combo: dict[str, ttk.Combobox] = {}
        self.open_btn: dict[str, tk.Button] = {}
        self.chan_status_var: dict[str, tk.StringVar] = {}

        for idx, (tag, desc) in enumerate(SERIAL_CHANNELS):
            self._build_port_block(parent, tag, desc, first=(idx == 0))

        self._sync_link()

    def _build_port_block(self, parent, tag: str, desc: str, first: bool) -> None:
        """单路串口：通道徽标 / 端口下拉+开关 / 状态条（窄窗三行布局）"""
        # 第 1 行：通道徽标 + 用途说明
        head = tk.Frame(parent, bg=CARD)
        head.pack(fill="x", pady=(0 if first else 9, 0))
        dot = BLUE if tag == SENSOR_CHANNEL else "#8B5CF6"
        tk.Label(head, text=f"串口 {tag}", bg=dot, fg="white", font=FONT_S,
                 padx=6, pady=1).pack(side="left")
        tk.Label(head, text=desc, bg=CARD, fg=SUBINK, font=FONT_S,
                 anchor="w").pack(side="left", padx=(6, 0))

        # 第 2 行：端口下拉 + 打开/关闭
        self._port_var[tag] = tk.StringVar()
        row = tk.Frame(parent, bg=CARD)
        row.pack(fill="x", pady=(4, 0))
        row.grid_columnconfigure(0, weight=1)
        self.port_combo[tag] = ttk.Combobox(row, textvariable=self._port_var[tag],
                                            state="readonly", width=8)
        self.port_combo[tag].grid(row=0, column=0, sticky="ew")
        self.open_btn[tag] = self._btn(
            row, "打开", lambda t=tag: self._toggle_serial(t),
            kind="soft", font=FONT_S, pady=3, padx=12)
        self.open_btn[tag].grid(row=0, column=1, sticky="e", padx=(6, 0))

        # 第 3 行：状态条
        self.chan_status_var[tag] = tk.StringVar(
            value="就绪：串口未打开" if HAS_SERIAL else "错误：未安装 pyserial")
        self._refresh_ports(tag)
        tk.Label(parent, textvariable=self.chan_status_var[tag], bg=FIELD, fg=SUBINK,
                 font=FONT_S, anchor="w", padx=8, pady=3, justify="left",
                 wraplength=PANEL_W - 2 * PAD - 26,
                 highlightthickness=1, highlightbackground=LINE).pack(
                     fill="x", pady=(4, 0))
        self.chan_status_var[tag].trace_add("write", lambda *_a: self._sync_link())

    def _sync_link(self) -> None:
        """顶栏连接徽标跟随两路串口状态"""
        parts = [f"{tag}:{'已连' if self.chan[tag].is_open else '未连'}"
                 for tag, _ in SERIAL_CHANNELS]
        ok = any(self.chan[t].is_open for t, _ in SERIAL_CHANNELS)
        self._link_var.set("● " + "  ".join(parts))
        self._link_label.configure(fg="#86EFAC" if ok else "#FCA5A5")
        self._refresh_angle_status()      # 角度盘的状态行跟随开关串口刷新

    # ---- 小窗：画面控制指令 ----
    def _build_cmd_panel(self, parent) -> None:
        head = tk.Frame(parent, bg=CARD)
        head.pack(fill="x", pady=(0, 5))
        tk.Label(head, text="下发目标", bg=CARD, fg=SUBINK, font=FONT_S).pack(side="left")
        self._cmd_target_var = tk.StringVar(value=DEFAULT_TX_CHANNEL)
        self._radios(head, self._cmd_target_var,
                     tuple((t, t) for t, _ in SERIAL_CHANNELS),
                     ).pack(side="right")

        grid = tk.Frame(parent, bg=CARD)
        grid.pack(fill="x")
        grid.grid_columnconfigure(0, weight=1)
        grid.grid_columnconfigure(1, weight=1)
        total = len(VIDEO_COMMANDS)
        rows = (total + 1) // 2
        for i, (code, label, toggle) in enumerate(VIDEO_COMMANDS):
            r, c = divmod(i, 2)
            # 奇数条时，最后一条独占一行 -> 横跨两列，避免出现空按钮位
            span = 2 if (i == total - 1 and total % 2 == 1) else 1
            if span == 2:
                padx = 0
            else:
                padx = (0, 4) if c == 0 else (4, 0)
            btn = self._btn(grid, f"{label}",
                            lambda cc=code, ll=label, tg=toggle:
                            self._send_video_cmd(cc, ll, tg),
                            kind="dark", font=FONT_S, pady=5)
            btn.grid(row=r, column=c, columnspan=span, sticky="ew",
                     padx=padx, pady=(0, 4) if r < rows - 1 else 0)
            if toggle:                 # 开关类：记住按钮与初始状态（黑=关）
                self._cmd_on[code] = False
                self._cmd_btns[code] = btn

        # 手动开关（v3.24）：与上面 6 条画面指令无关，直接控制板端 ADAS 识别
        ai_row = tk.Frame(parent, bg=CARD)
        ai_row.pack(fill="x", pady=(6, 0))
        self._ai_btn = self._btn(ai_row, "AI 识别：运行中（点击关闭）",
                                 self._toggle_ai_manual, kind="ghost",
                                 font=FONT_S, pady=5)
        self._ai_btn.pack(fill="x")
        self._paint_ai_btn()

        # 画面模式计数（FPGA 5 态循环，只有第 5 个是四路拼接）：
        # 显示当前推断到的模式，右侧「复位」把计数拉回 1/5（HDMI 单路）。
        view_row = tk.Frame(parent, bg=CARD)
        view_row.pack(fill="x", pady=(4, 0))
        self._view_var = tk.StringVar(value="")
        tk.Label(view_row, textvariable=self._view_var, bg=CARD, fg=SUBINK,
                 font=FONT_S, anchor="w").pack(side="left")
        self._btn(view_row, "复位", self._reset_view_step, kind="ghost",
                  font=FONT_S, pady=2, padx=8).pack(side="right")
        self._paint_view_state()

        opt = tk.Frame(parent, bg=CARD)
        opt.pack(fill="x", pady=(7, 0))
        self._cmd_hex_var = tk.BooleanVar(value=False)
        self._radios(opt, self._cmd_hex_var,
                     (("HEX", True), ("ASCII", False))).pack(side="left")
        self._cmd_newline_var = tk.BooleanVar(value=False)
        self._check(opt, r"追加 \r\n", self._cmd_newline_var).pack(side="right")

    # ---- 调试界面右列上：数据日志 ----
    def _build_rx_panel(self, parent) -> None:
        card = self._card(parent)
        card.grid(row=0, column=0, sticky="nsew", pady=(0, GAP))

        self.rx_count_var = tk.StringVar(value="RX: 0")
        head = tk.Frame(card, bg=CARD)
        head.pack(fill="x", padx=PAD, pady=(PAD, 6))
        tk.Frame(head, bg=BLUE, width=3, height=24).pack(side="left", padx=(0, 8))
        tk.Label(head, text="数据日志", bg=CARD, fg=INK, font=FONT_B).pack(side="left")
        tk.Label(head, text="收发原始字节", bg=CARD, fg=SUBINK,
                 font=FONT_S).pack(side="left", padx=(8, 0))
        tk.Label(head, textvariable=self.rx_count_var, bg=CARD, fg=SUBINK,
                 font=FONT_MONO).pack(side="right")

        # 接收设置：两行排布，避免在窄宽度下被挤出卡片
        cfg = tk.Frame(card, bg=CARD)
        cfg.pack(fill="x", padx=PAD)
        r0 = tk.Frame(cfg, bg=CARD)
        r0.pack(fill="x")
        r1 = tk.Frame(cfg, bg=CARD)
        r1.pack(fill="x", pady=(5, 0))
        self._rx_ascii_var = tk.BooleanVar(value=True)
        self._radios(r0, self._rx_ascii_var, (("ASCII", True), ("HEX", False)),
                     self._refresh_rx_display).pack(side="left", padx=(0, 10))
        self._rx_logmode_var = tk.BooleanVar(value=True)
        self._check(r0, "日志模式", self._rx_logmode_var).pack(side="left")
        self._rx_autoscroll_var = tk.BooleanVar(value=True)
        self._check(r1, "自动滚屏", self._rx_autoscroll_var).pack(side="left", padx=(0, 8))
        self._rx_wrap_var = tk.BooleanVar(value=False)
        self._check(r1, "自动换行", self._rx_wrap_var,
                    self._on_rx_wrap_toggle).pack(side="left", padx=(0, 8))
        self._btn(r1, "清除接收", self._clear_rx, kind="ghost",
                  pady=3).pack(side="left")

        rxbox, self.rx_text = self._field(card, 6, bg=CARD, fg=INK)
        rxbox.pack(fill="both", expand=True, padx=PAD, pady=(6, PAD))
        self.rx_text.tag_config("rx", foreground=BLUE)
        self.rx_text.tag_config("tx", foreground=GREEN)
        self.rx_text.tag_config("info", foreground=SUBINK)
        self.rx_text.tag_config("err", foreground=RED)

    # ---- 调试界面右列下：数据发送 ----
    def _build_tx_panel(self, parent) -> None:
        card = self._card(parent)
        card.grid(row=1, column=0, sticky="ew")

        self.tx_count_var = tk.StringVar(value="TX: 0")
        head = tk.Frame(card, bg=CARD)
        head.pack(fill="x", padx=PAD, pady=(PAD, 6))
        tk.Frame(head, bg=GREEN, width=3, height=24).pack(side="left", padx=(0, 8))
        tk.Label(head, text="数据发送", bg=CARD, fg=INK, font=FONT_B).pack(side="left")
        tk.Label(head, textvariable=self.tx_count_var, bg=CARD, fg=SUBINK,
                 font=FONT_MONO).pack(side="right")

        txbox, self.tx_text = self._field(card, 3, bg=CARD, fg=INK)
        txbox.pack(fill="x", padx=PAD, pady=(0, 6))

        cfg = tk.Frame(card, bg=CARD)
        cfg.pack(fill="x", padx=PAD, pady=(0, PAD))
        c0 = tk.Frame(cfg, bg=CARD)
        c0.pack(fill="x")
        c1 = tk.Frame(cfg, bg=CARD)
        c1.pack(fill="x", pady=(5, 0))
        c2 = tk.Frame(cfg, bg=CARD)
        c2.pack(fill="x", pady=(5, 0))
        self._tx_ascii_var = tk.BooleanVar(value=True)
        self._radios(c0, self._tx_ascii_var,
                     (("ASCII", True), ("HEX", False))).pack(side="left", padx=(0, 10))
        self._tx_newline_var = tk.BooleanVar(value=False)
        self._check(c0, "发送新行", self._tx_newline_var).pack(side="left")
        self._tx_escape_var = tk.BooleanVar(value=False)
        self._check(c1, "自动解析转义符", self._tx_escape_var).pack(side="left")
        self._send_target_var = tk.StringVar(value=DEFAULT_TX_CHANNEL)
        self._radios(c2, self._send_target_var,
                     tuple((f"发往{t}", t) for t, _ in SERIAL_CHANNELS)).pack(side="left")
        self._btn(c2, "发送", self._send_serial, kind="primary",
                  pady=3).pack(side="right")
        self._btn(c2, "清除发送", self._clear_tx, kind="ghost",
                  pady=3).pack(side="right", padx=(0, 6))

    # ---- 底部状态栏 ----
    def _build_statusbar(self, parent=None) -> None:
        parent = parent or self
        bar = tk.Frame(parent, bg=FIELD, highlightthickness=1, highlightbackground=LINE)
        bar.grid(row=2, column=0, sticky="ew")
        self._debug_statusbar = bar
        # 位串摘要已搬到控制小窗的「感知状态」里，这里只留操作提示与串口参数
        tk.Label(bar, text="F1 控制小窗 · F2 实时画面 ⇄ 调试界面 · F11 全屏",
                 bg=FIELD, fg=SUBINK, font=FONT_S,
                 anchor="w").pack(side="left", padx=16, pady=6)
        tk.Label(bar, text=SERIAL_FIXED_DESC, bg=FIELD, fg=SUBINK, font=FONT_S,
                 anchor="e").pack(side="right", padx=16, pady=6)

    # ---------------- 视频：画面控制指令下发 ----------------
    def _paint_cmd_btn(self, code: str) -> None:
        """开关类按钮上色：开=红，关=黑"""
        btn = self._cmd_btns.get(code)
        if btn is None:
            return
        on = bool(self._cmd_on.get(code))
        btn.configure(bg=RED if on else HEAD,
                      activebackground="#B91C1C" if on else "#374151",
                      fg="white", activeforeground="white")

    # ---------------- AI 识别闸门（v3.23：四路拼接 / 倒车辅助线） ----------------
    def _ai_gate_reason(self) -> str:
        """当前画面状态下是否该暂停板端 ADAS 识别；返回原因（空串 = 正常识别）。"""
        why = []
        if getattr(self, "_ai_manual_off", False):     # v3.24 手动关闭优先显示
            why.append("手动关闭")
        if getattr(self, "_quad_view", False):
            why.append("四路拼接视频")
        if self._cmd_on.get("4", False):
            why.append("倒车辅助线")
        return " + ".join(why)

    def _send_ai_gate(self, pause: bool) -> bool:
        """UDP 通知板端暂停/恢复 ADAS 识别（尽力而为：板子不在线只记一行日志）。"""
        try:
            if self._ai_sock is None:
                self._ai_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._ai_sock.sendto(b"AI 0" if pause else b"AI 1",
                                 (BOARD_IP_DEFAULT, AI_CTRL_PORT))
            self._ai_last_sent = pause
            self._ai_err_logged = False
            return True
        except Exception as e:
            if not self._ai_err_logged:          # 同一故障只记一次，避免 2 秒一条
                self._ai_err_logged = True
                self.write_log(f"AI 识别闸门下发失败：{e}（板端需在网内且带 --ai-ctrl-port）",
                               "err")
            return False

    def _paint_ai_badge(self, why: str) -> None:
        """画面左上角的「已暂停 AI 识别」提示条：有原因才显示。"""
        badge = getattr(self, "_ai_badge", None)
        if badge is None:
            return
        try:
            if not why:
                badge.place_forget()
                return
            badge.configure(text=f"已暂停 AI 识别 · {why}")
            badge.place(x=16, y=TOPBAR_H + 10, anchor="nw")
            badge.lift()
        except Exception:
            pass

    def _sync_ai_gate(self) -> None:
        """把「四路拼接 / 倒车辅助线」状态同步给板端；状态没变就不重复下发。"""
        why = self._ai_gate_reason()
        want = bool(why)
        self._paint_ai_badge(why)
        if want == self._ai_suspended and self._ai_last_sent == want:
            return
        self._ai_suspended = want
        if not self._send_ai_gate(want):
            return
        if why != self._ai_gate_logged:
            self._ai_gate_logged = why
            if want:
                self.write_log(f"AI 识别已暂停（{why}）：已通知板端 {BOARD_IP_DEFAULT}:"
                               f"{AI_CTRL_PORT} 停止推理，框与串口状态冻结", "warn")
            else:
                self.write_log("AI 识别已恢复：板端继续推理并恢复检测框", "ok")

    def _ai_gate_beat(self) -> None:
        """2 秒心跳：板端重启 / 丢包后自动收敛到当前状态（幂等，不写日志）。"""
        try:
            self._send_ai_gate(self._ai_suspended)
        except Exception:
            pass
        self.after(2000, self._ai_gate_beat)

    # ---------------- 手动 AI 识别开关（v3.24） ----------------
    def _paint_ai_btn(self) -> None:
        """手动开关按钮上色：运行中=绿，已关闭=红。"""
        btn = getattr(self, "_ai_btn", None)
        if btn is None:
            return
        if getattr(self, "_ai_manual_off", False):
            btn.configure(text="AI 识别：已关闭（点击开启）", bg=RED,
                          activebackground="#B91C1C",
                          fg="white", activeforeground="white")
        else:
            btn.configure(text="AI 识别：运行中（点击关闭）", bg=GREEN,
                          activebackground="#15803D",
                          fg="white", activeforeground="white")

    def _paint_view_state(self) -> None:
        """刷新「画面模式 n/5」文字（四路拼接那一档标出来）。"""
        var = getattr(self, "_view_var", None)
        if var is None:
            return
        n = getattr(self, "_view_step", 0) + 1
        tail = "四路拼接" if getattr(self, "_quad_view", False) else "非拼接"
        var.set(f"画面模式 {n}/{VIEW_CYCLE}（{tail}）")

    def _reset_view_step(self) -> None:
        """把画面模式计数拉回第 1 个（HDMI 单路）。FPGA 被硬件按键改过时用它重新对齐。"""
        self._view_step = 0
        self._quad_view = False
        self._paint_view_state()
        self._sync_ai_gate()
        self.write_log(f"画面模式计数已复位为 1/{VIEW_CYCLE}（HDMI 单路）", "info")

    def _toggle_ai_manual(self) -> None:
        """手动关闭/开启 AI 识别：与「四路拼接 / 倒车辅助线」取或关系。"""
        self._ai_manual_off = not getattr(self, "_ai_manual_off", False)
        self._paint_ai_btn()
        self._sync_ai_gate()
        if not self._ai_manual_off:
            # 手动已开，但画面本身还要求暂停时要说清楚，别让人以为是按钮失灵
            rest = []
            if getattr(self, "_quad_view", False):
                rest.append("四路拼接视频")
            if self._cmd_on.get("4", False):
                rest.append("倒车辅助线")
            if rest:
                self.write_log(f"已开启 AI 识别，但当前「{' + '.join(rest)}」"
                               f"仍暂停识别", "warn")

    def _send_video_cmd(self, code: str, label: str, toggle: bool = False) -> None:
        tag = self._cmd_target_var.get()
        ch = self.chan[tag]
        if not ch.is_open:
            messagebox.showwarning("串口未打开",
                                   f"请先打开「串口 {tag}（{ch.desc}）」，再发送「{label}」指令")
            return
        # 开关类：先算出"本次按下之后"的目标状态，发送成功才真正翻转按钮颜色
        next_on = (not self._cmd_on.get(code, False)) if toggle else False
        try:
            if self._cmd_hex_var.get():
                payload = bytes([int(code, 16)])     # 0x00 ~ 0x06
            else:
                payload = code.encode("ascii")       # b'0' ~ b'6'
            if self._cmd_newline_var.get():
                payload += b"\r\n"
        except Exception as e:
            self.write_log(f"指令「{label}」编码失败：{e}", "err")
            return

        try:
            ch.ser.write(payload)   # type: ignore[union-attr]
        except Exception as e:
            self.write_log(f"指令「{label}」发送失败：{e}", "err")
            return

        if toggle:                                    # 开 -> 按钮变红；关 -> 变回黑
            self._cmd_on[code] = next_on
            self._paint_cmd_btn(code)

        if code == "0":
            # 画面切换：FPGA 内部 5 态循环（HDMI 单路 -> … -> 四路拼接 -> 回 HDMI），
            # 不能按「翻转」处理，否则每按一下都会误判成四路拼接
            self._view_step = (self._view_step + 1) % VIEW_CYCLE
            self._quad_view = (self._view_step == VIEW_QUAD_STEP)
            self._paint_view_state()
            self.write_log(f"画面已切到第 {self._view_step + 1}/{VIEW_CYCLE} 个模式"
                           + ("：四路拼接视图" if self._quad_view else ""), "info")
        # 拼接视图 / 倒车辅助线 -> 通知板端停掉 AI 识别（两者都关掉才恢复）
        self._sync_ai_gate()

        self.tx_count += len(payload)
        self.tx_count_var.set(f"TX: {self.tx_count}")
        mode = "HEX" if self._cmd_hex_var.get() else "ASCII"
        state = ("（打开）" if next_on else "（关闭）") if toggle else ""
        self.write_log(f"画面指令「{label}」{state}-> 串口{tag} [{mode}]：{payload!r}", "ok")

    # ---------------- 传感器 ----------------
    def refresh_sensors(self) -> None:
        bits = []
        for key, _ in SENSORS:
            v = self.state[key]
            bits.append("1" if v else "0")
            lab = self.sensor_labels.get(key)
            if lab is None:      # 控制小窗还没建好（启动早期）就只算位串
                continue
            if v and key in TIP_ONLY_SENSORS:
                # 仅提示类：用提示黄，视觉上区分于"可以通行"的绿
                lab.configure(text="ON ", bg=TIP_SOFT, fg=TIP_INK)
            else:
                lab.configure(text="ON " if v else "OFF",
                              bg=OK_SOFT if v else BAD_SOFT,
                              fg=GREEN if v else SUBINK)
        if hasattr(self, "_bits_var"):
            self._bits_var.set("位串 " + "".join(bits))
        if hasattr(self, "ped_var"):
            self.ped_var.set(str(self.ped_count))
            hot = self.ped_count >= PED_ALERT_THRESHOLD
            # 行人只作提示：人数再多也保持"提示黄"，不染红（报警色留给真正违规的信号量）
            self.ped_label.configure(bg="#FDE68A" if hot else "#FEF3C7",
                                     fg="#92400E" if hot else "#92400E")

    def _apply_sensor_bits(self, bits: str) -> None:
        """兼容旧协议：只有 6 位位串时更新布尔量，行人数量保持上一次的值。"""
        self._apply_sensor_frame(bits, self.ped_count)

    def _apply_sensor_frame(self, bits: str, count: int) -> None:
        """将「6 位位串 + 行人数量」映射到界面，并触发语音提示与决策重检。"""
        for i, (key, _) in enumerate(SENSORS):
            self.state[key] = bits[i] == "1"
        self.ped_count = max(0, min(PED_COUNT_MAX, int(count)))
        self.refresh_sensors()
        name_list = "，".join(f"{n}={self.state[k]}" for k, n in SENSORS)
        self.write_log(f"串口更新感知帧：{bits} {self.ped_count} -> {name_list}；"
                       f"行人 {self.ped_count} 人", "info")
        self._evaluate_voice_triggers()
        self.recheck_while_driving()

    # ---------------- 语音提示 ----------------
    def _speak(self, key: str, fallback_cn: str, dedupe_sec: float = 2.0) -> None:
        """按当前语言播报一条提示（无语音模块时静默）。"""
        if not self.speaker:
            return
        text = phrase(key, self.voice_lang) if phrase else fallback_cn
        self.write_log(f"语音[{LANG_NAMES.get(self.voice_lang, self.voice_lang)}]：{text}", "ok")
        # 提示类词条本身就是缓存片段，命中缓存即可发声；优先级低于报警
        self.speaker.say(text, dedupe_sec=dedupe_sec, pieces=[text], prio=PRIO_TIP)

    def _speak_alarm(self, action: str, reasons: list[str]) -> None:
        """报警语音：同一违规只播报一次（条件变化后才会再播）。"""
        keys = reason_keys(reasons)
        sig = f"{action}|" + ",".join(keys)
        if sig == self._alarm_sig:
            return
        self._alarm_sig = sig
        if not self.speaker or alarm_text is None:
            _beep()
            return
        lg = self.voice_lang
        text = alarm_text(action, keys, lg)
        self.write_log(f"语音[{LANG_NAMES.get(lg, lg)}]：{text}", "err")
        # 传 alarm_parts 让播放器用已缓存片段本地拼接发声（不联网、零等待），
        # 并用 PRIO_ALARM 插队、可打断正在播放的提示音。
        parts = alarm_parts(action, keys, lg) if alarm_parts else None
        self.speaker.say(text, lang=lg, dedupe_sec=1.0,
                         pieces=parts, prio=PRIO_ALARM)

    def _evaluate_voice_triggers(self) -> None:
        """条件类提示：满足条件时播报一次；同一条提示有 TIP_COOLDOWN_SEC 冷却。

        冷却的意义：行人数量 / 位串在阈值附近抖动时（如 5↔4 反复横跳），
        闭锁会被反复解锁又触发，「注意行人」就会一直响，把左右转与直行的
        违规报警全部盖住 —— 现在两次提示之间至少间隔 TIP_COOLDOWN_SEC 秒。
        """
        now = time.time()
        # ① 遇到斑马线
        cw = bool(self.state.get("crosswalk_ahead"))
        if cw and not self._voice_latch["crosswalk"] \
                and now - self._tip_last["crosswalk"] >= TIP_COOLDOWN_SEC:
            self._voice_latch["crosswalk"] = True
            self._tip_last["crosswalk"] = now
            self._speak("tip_crosswalk", "注意斑马线")
        elif not cw:
            self._voice_latch["crosswalk"] = False
        # ② 行人数量达到阈值
        many = self.ped_count >= PED_ALERT_THRESHOLD
        if many and not self._voice_latch["pedestrian"] \
                and now - self._tip_last["pedestrian"] >= TIP_COOLDOWN_SEC:
            self._voice_latch["pedestrian"] = True
            self._tip_last["pedestrian"] = now
            self._speak("tip_pedestrian", "注意行人")
        elif not many:
            self._voice_latch["pedestrian"] = False

    def _on_voice_lang(self, _e=None) -> None:
        name = self.voice_lang_var.get()
        code = self._lang_name2code.get(name, DEFAULT_VOICE_LANG)
        self.voice_lang = code
        if self.speaker:
            self.speaker.set_lang(code)
            self.speaker.prefetch(code)
        self._refresh_voice_info(force=True)
        self.write_log(f"语音语言切换为：{name}", "info")

    def _on_voice_mute(self) -> None:
        if self.speaker:
            self.speaker.enabled = not self._voice_mute_var.get()
        self.write_log("语音已静音" if self._voice_mute_var.get() else "语音已开启", "info")

    def _refresh_voice_info(self, force: bool = False) -> None:
        """刷新「人声后端」一行：显示后端、预热状态与最近一次播报延迟。"""
        if not hasattr(self, "_voice_info_var") or self._voice_info_var is None:
            return
        now = time.time()
        if not force and now - getattr(self, "_voice_ready_t", 0.0) < 1.0:
            return
        self._voice_ready_t = now
        sp = self.speaker
        if sp is None:
            self._voice_info_var.set("人声后端：不可用（蜂鸣兜底）")
            return
        try:
            ready = sp.is_ready(self.voice_lang)
        except Exception:
            ready = False
        lat = getattr(sp, "last_latency", -1.0)
        lat_s = f"；上次播报延迟 {lat * 1000:.0f} ms" if lat >= 0 else ""
        self._voice_info_var.set(
            f"人声后端：{sp.describe()}；"
            f"{LANG_NAMES.get(self.voice_lang, self.voice_lang)}缓存"
            f"{'已就绪' if ready else '预热中…'}{lat_s}")

    def _voice_test(self) -> None:
        """试听：立即播报当前语言的「注意斑马线」。"""
        if not self.speaker:
            messagebox.showwarning("语音不可用",
                                   "未加载 voice_prompts 模块，当前只能使用蜂鸣报警。\n"
                                   "请运行「安装语音依赖.bat」后重启程序。")
            return
        text = phrase("tip_crosswalk", self.voice_lang)
        self.write_log(f"试听[{LANG_NAMES.get(self.voice_lang, self.voice_lang)}]：{text}", "info")
        self.speaker.say(text, dedupe_sec=0.0)

    # ---------------- 报警提示（v3.11：只语音提醒，不锁定操作）----------------
    def _is_locked(self) -> bool:
        """保留接口兼容旧调用：报警不再锁定操作，因此永远返回 False。"""
        return False

    def _notice(self, text: str) -> None:
        """在行驶意图按键下方显示一行黄色提示，几秒后自动消失（不影响操作）。"""
        self._notice_text = text
        self._notice_until = time.time() + ALARM_NOTICE_SEC
        self._sync_action_buttons()

    def _clear_notice(self) -> None:
        self._notice_text = ""
        self._notice_until = 0.0
        self._sync_action_buttons()

    def _sync_action_buttons(self) -> None:
        """五键**始终可用**（报警不锁操作），这里只刷新配色与提示条。
        「停车」在无行驶意图（默认/复位）时红色高亮，直观显示当前处于停车状态。"""
        stopped = (self.action is None)
        for b in self.action_btns:
            try:
                bg = RED if (b["text"] == "停车" and stopped) else HEAD
                b.configure(state="normal", cursor="hand2", bg=bg, fg="white")
            except Exception:
                pass
        if not hasattr(self, "_notice_var"):
            return
        if self._notice_until and time.time() < self._notice_until:
            left = max(self._notice_until - time.time(), 0.0)
            self._notice_var.set(f"{self._notice_text}（{left:.0f}s）· 按键不锁定，可自由操作")
        else:
            self._notice_until = 0.0
            self._notice_text = ""
            self._notice_var.set("")

    # ---------------- 动作 ----------------
    def _cancel_turn_return(self) -> None:
        """取消未触发的"转向后自动回退"定时器（手动选择新动作时用）。"""
        if getattr(self, "_turn_return_after", None):
            try:
                self.after_cancel(self._turn_return_after)
            except Exception:
                pass
            self._turn_return_after = None
        self._turn_return_deadline = 0.0

    def _action_state_text(self) -> str:
        """行驶意图五键下方那行状态字的文本。"""
        if self.action in TURN_ACTIONS:
            prev = "直行" if self._turn_return_state == "直行" else "停车"
            if self._turn_return_deadline:
                left = max(self._turn_return_deadline - time.time(), 0.0)
                return f"当前状态：{self.action}中（{left:.1f}s 后自动回{prev}）"
            return f"当前状态：{self.action}中（即将自动回{prev}）"
        if self.action:
            tag = "合规" if self.action_legal else "违规放行"
            return f"当前状态：{self.action}（{tag}）"
        return "当前状态：停车"

    def _sync_action_state(self) -> None:
        """刷新五键下方的行驶状态条（面板未建好时静默跳过）。

        着色规则：停车=灰底、合规行驶=绿底、违规放行=红底，白字大号加粗。"""
        var = getattr(self, "_action_state_var", None)
        if var is None:
            return
        text = self._action_state_text()
        var.set(text)
        lbl = getattr(self, "_action_state_label", None)
        if lbl is not None:
            if self.action is None:
                bg = "#4B5563"                        # 停车：中性灰
            elif self.action_legal:
                bg = GREEN                            # 合规：绿
            else:
                bg = RED                              # 违规放行：红
            try:
                lbl.configure(bg=bg, fg="white")
            except Exception:
                pass

    def _turn_return(self) -> None:
        """左/右转按下 TURN_RETURN_SEC 秒后自动回到先前状态（直行或停车）。"""
        self._turn_return_after = None
        self._turn_return_deadline = 0.0
        prev = self._turn_return_state
        self._turn_return_state = None
        if prev == "直行":
            self.write_log("转向结束：自动恢复「直行」", "info")
            self.do_action("直行", announce=False)
        else:
            self.write_log("转向结束：自动恢复「停车」", "info")
            self.stop_car(announce=False)

    def do_action(self, action: str, announce: bool = True) -> None:
        """执行行驶意图。违规也照常执行，只做语音提醒，绝不拦截操作。

        v3.14：左转 / 右转是「瞬时动作」—— 按下后把转向前的状态（直行或停车）
        记为回退目标，TURN_RETURN_SEC 秒后自动恢复；连按另一个转向不改变回退目标。
        手动按下 直行 / 倒车 / 停车 则取消未决的自动回退，以手动操作为准。
        v3.21：announce=False 用于「自动回退」这类非按键触发，不播报按钮名称。
        """
        if action in TURN_ACTIONS:
            if self.action not in TURN_ACTIONS:
                self._turn_return_state = self.action    # '直行' 或 None(停车)
            self._cancel_turn_return()
            self._turn_return_deadline = time.time() + TURN_RETURN_SEC
            self._turn_return_after = self.after(
                int(TURN_RETURN_SEC * 1000), self._turn_return)
        else:
            self._cancel_turn_return()
            self._turn_return_state = None

        allowed, reasons = judge(action, self.state)
        self.action = action            # 无论合法与否都按用户意图记录（自由操作）
        self.action_legal = bool(allowed)
        self._sync_action_state()       # 五键下方那行状态字
        if allowed:
            self._alarm_sig = None      # 合法放行后，允许同类违规再次播报
            self.status_label.configure(bg="#ECFDF5", fg=GREEN)
            self.status_var.set(
                f"允许{action} —— 符合通行条件；建议巡航车速 {SUGGEST_SPEED[action]:.2f} km/h")
            self.write_log(f"执行「{action}」：合法放行", "ok")
            self._clear_notice()
            # v3.21：不违规时播报按钮名称（六语种），让操作有明确语音反馈；
            # 违规走 alarm() 的警告语音，两者互斥不会叠着播。
            if announce:
                self._speak(f"act_{action}", action, dedupe_sec=0.6)
        else:
            self.alarm(action, reasons)  # 红色闪烁 + 人声播报，但不锁操作

    def stop_car(self, announce: bool = True) -> None:
        """停车：清除当前行驶意图。任何时刻都能点，也是启动 / 复位后的默认状态。"""
        self._cancel_turn_return()
        self._turn_return_state = None
        self.action = None
        self.action_legal = False
        self._alarm_sig = None
        self._sync_action_state()       # 五键下方那行状态字
        self.status_label.configure(bg=CARD, fg=INK)
        self.status_var.set("已停车：当前无行驶意图（倒车轨迹角度由角度盘控制）")
        self.write_log("点击「停车」：车辆静止，无行驶意图", "info")
        self._clear_notice()
        # v3.21：停车永远不违规，同样播报按钮名称（自动回退时不播）
        if announce:
            self._speak("act_停车", "停车", dedupe_sec=0.6)

    def alarm(self, action: str, reasons: list[str]) -> None:
        """违规报警：红色闪烁 + 人声播报 + 日志。**不锁定任何操作**。"""
        self.alarm_until = time.time() + ALARM_FLASH_SEC
        self.status_label.configure(bg="#FEF2F2", fg=RED)
        self.status_var.set(f"违规{action}！" + "；".join(reasons) +
                            "（已按你的操作放行，仅语音提示）")
        self.write_log(f"== 报警：违规{action}（仅提示，不锁定操作）==", "err")
        for r in reasons:
            self.write_log("   - " + r, "err")
        self._speak_alarm(action, reasons)
        self._notice(f"⚠ 违规{action}：已语音提醒")

    def recheck_while_driving(self) -> None:
        """行驶途中若原先合法的条件失效 -> 报警提示（不自动制动，保留车速不变）

        v3.18：红绿灯由绿转红的「武装瞬间」不再播报任何语音 —— 灯类理由
        （信号灯非绿色）只写日志、不闪烁不发声；标线类理由（实线等）
        仍照常报警。v3.20 起「已经可XX / 全线放行」放行语音整体移除。
        """
        if self.action is None:
            return
        allowed, reasons = judge(self.action, self.state)
        if allowed:
            return
        light = [r for r in reasons if "信号灯非绿色" in r]
        others = [r for r in reasons if "信号灯非绿色" not in r]
        if others:
            # 标线类违规照常报警（语音只报标线理由，灯类理由仅记录）
            if light:
                self.write_log("行驶条件已失效（" + self.action + "）：灯类理由仅记录不播报 —— "
                               + "；".join(light), "err")
            self.write_log(f"行驶条件已失效（{self.action}）—— 请自行降速停车", "err")
            self.alarm(self.action, others)
        elif light:
            # 纯灯类理由 = 红灯武装瞬间：只记日志，静默不播报
            self.write_log(f"行驶条件已失效（{self.action}）：{light[0].split(' ——')[0]}"
                           "（红灯武装，不播报语音，待绿灯放行提示）", "err")

    # ---------------- 复位 ----------------
    def reset(self) -> None:
        self.state = {
            k: (DEFAULT_SENSOR_BITS[i] == "1")
            for i, (k, _) in enumerate(SENSORS)
        }
        self.speed = 0.00
        self.action = None
        self.action_legal = False
        self.alarm_until = 0.0
        self._alarm_sig = None
        self._cancel_turn_return()
        self._turn_return_state = None
        self.ped_count = DEFAULT_PED_COUNT
        self._voice_latch = {"crosswalk": False, "pedestrian": False}
        self._tip_last = {"crosswalk": 0.0, "pedestrian": 0.0}
        self.refresh_sensors()
        self._sync_action_state()
        self.status_label.configure(bg=CARD, fg=INK)
        self.status_var.set("已停车：场景已复位，请选择行驶方向")
        self.write_log(f"场景已复位至默认值 {DEFAULT_SENSOR_BITS} {DEFAULT_PED_COUNT}；"
                       f"行人数量归零", "info")
        self._clear_notice()

    # ---------------- 日志 ----------------
    def write_log(self, msg: str, tag: str = "info") -> None:
        ts = time.strftime("%H:%M:%S")
        self.log.configure(state="normal")
        self.log.insert("end", f"[{ts}] {msg}\n", tag)
        self.log.see("end")
        self.log.configure(state="disabled")

    # ---------------- 定时循环 (100ms) ----------------
    def _tick(self) -> None:
        # 报警期间判定状态条红白闪烁
        if time.time() < self.alarm_until:
            self._alarm_tick = (self._alarm_tick + 1) % 6
            self.status_label.configure(
                bg=RED if self._alarm_tick < 3 else "#FEF2F2", fg="white")
        elif self.action and self.action_legal:
            self.status_label.configure(bg="#ECFDF5", fg=GREEN)
        elif self.action:
            # 违规但已按用户意图放行：保持浅红，提醒"这个操作不合规"
            self.status_label.configure(bg="#FEF2F2", fg=RED)
        else:
            self.status_label.configure(bg=CARD, fg=INK)

        # 五键保持可用 + 提示条倒计时（v3.11：不再有报警锁定）
        self._sync_action_buttons()
        self._sync_action_state()     # 五键下方行驶状态行（转向时带倒计时）
        # 语音状态行（后端 / 预热 / 延迟），每秒刷新一次
        self._refresh_voice_info()

        # 角度帧持续下发：串口 B 已打开且勾选「持续发送」时，每 100ms 一帧
        if hasattr(self, "_angle_auto_var") and self._angle_auto_var.get():
            self._angle_send()

        self.after(100, self._tick)

    # ---------------- 串口：端口列表 ----------------
    def _refresh_ports(self, tag: str | None = None) -> None:
        tags = [tag] if tag else [t for t, _ in SERIAL_CHANNELS]
        for t in tags:
            combo = self.port_combo[t]
            if not HAS_SERIAL:
                combo["values"] = []
                continue
            infos = list(serial.tools.list_ports.comports())
            ports = [p.device for p in infos]
            combo["values"] = ports
            if ports and not self._port_var[t].get():
                # 优先挑 PREFERRED_PORTS 里描述的那个口（按 COM 名，再按描述关键字）
                pick = ports[0]
                for hint in PREFERRED_PORTS.get(t, ()):
                    hit = next(
                        (p.device for p in infos
                         if p.device.upper() == hint.upper()
                         or (p.description and hint.upper() in p.description.upper())),
                        None)
                    if hit:
                        pick = hit
                        break
                self._port_var[t].set(pick)

    # ---------------- 串口：打开/关闭 ----------------
    def _toggle_serial(self, tag: str) -> None:
        if self.chan[tag].is_open:
            self._close_serial(tag)
        else:
            self._open_serial(tag)

    def _open_serial(self, tag: str) -> None:
        if not HAS_SERIAL:
            messagebox.showerror("缺少依赖", "未安装 pyserial，串口功能不可用。\n"
                                "请在项目目录运行：.venv\\Scripts\\python.exe -m pip install pyserial")
            return
        ch = self.chan[tag]
        port = self._port_var[tag].get().strip()
        if not port:
            messagebox.showwarning("未选择串口", f"请先为「串口 {tag}」选择一个串口号")
            return

        # 链路参数固定：115200 / 8 数据位 / 1 停止位 / 无校验 / 无流控
        baud = SERIAL_BAUDRATE
        try:
            ch.ser = serial.Serial(
                port=port,
                baudrate=SERIAL_BAUDRATE,
                bytesize=SERIAL_BYTESIZE,
                stopbits=SERIAL_STOPBITS,
                parity=SERIAL_PARITY,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
                timeout=0.05,
            )
        except Exception as e:
            self.chan_status_var[tag].set(f"错误：{e}")
            self.write_log(f"串口 {tag} 打开失败：{e}", "err")
            return

        ch.stop.clear()
        ch.thread = threading.Thread(target=self._serial_reader, args=(tag,), daemon=True)
        ch.thread.start()

        self.open_btn[tag].configure(text="关闭", bg="#FEF2F2", fg=RED)
        self.chan_status_var[tag].set(f"已打开 {port} @ {baud}")
        self.write_log(f"串口 {tag}（{ch.desc}）已打开：{port} @ {baud}（{SERIAL_FIXED_DESC}）", "ok")

    def _close_serial(self, tag: str | None = None) -> None:
        """关闭指定串口；不传 tag 则两路全关。"""
        tags = [tag] if tag else [t for t, _ in SERIAL_CHANNELS]
        for t in tags:
            ch = self.chan[t]
            if ch.ser:
                try:
                    ch.stop.set()
                    ch.ser.cancel_read()
                    ch.ser.close()
                except Exception as e:
                    self.write_log(f"串口 {t} 关闭异常：{e}", "err")
                finally:
                    ch.ser = None
            self.open_btn[t].configure(text="打开", bg="#E0E7FF", fg="#3730A3")
            self.chan_status_var[t].set("就绪：串口未打开")
            self.write_log(f"串口 {t} 已关闭", "info")

    def _serial_reader(self, tag: str) -> None:
        ch = self.chan[tag]
        ser = ch.ser
        if ser is None:
            return
        while not ch.stop.is_set():
            try:
                data = ser.read(ser.in_waiting or 1)
                if data:
                    ch.queue.put(("rx", data))
            except serial.SerialException as e:
                ch.queue.put(("err", f"串口 {tag} 异常：{e}"))
                break
            except Exception as e:
                ch.queue.put(("err", f"串口 {tag} 读取异常：{e}"))
                break
        try:
            if ser.is_open:
                ser.close()
        except Exception:
            pass

    # ---------------- 串口：主线程轮询 ----------------
    def _poll_serial(self) -> None:
        try:
            for tag, _desc in SERIAL_CHANNELS:
                ch = self.chan[tag]
                while not ch.queue.empty():
                    kind, payload = ch.queue.get()
                    if kind == "rx":
                        data: bytes = payload  # type: ignore
                        self.rx_count += len(data)
                        self.rx_count_var.set(f"RX: {self.rx_count}")
                        ch.last_rx_text += data.decode("latin-1", errors="replace")
                        self._rx_history.append((tag, data))
                        if len(self._rx_history) > self._rx_history_max:
                            del self._rx_history[:len(self._rx_history) - self._rx_history_max]
                        self._append_rx_log(tag, data)
                        # 只有感知通道（A）的字节参与感知帧解析
                        if tag == SENSOR_CHANNEL:
                            ch.rx_buffer.extend(data)
                            # ① 新协议：6 位位串 + 空格 + 行人数量（例："111110 3"）
                            frame = consume_sensor_frame(ch.rx_buffer)
                            if frame:
                                self._apply_sensor_frame(*frame)
                            else:
                                # ② 兼容旧协议：只有 6 位位串，人数保持上一次的值
                                bits = try_parse_sensor_bits(bytes(ch.rx_buffer))
                                if bits:
                                    self._apply_sensor_frame(bits, self.ped_count)
                                    idx = bytes(ch.rx_buffer).find(bits.encode())
                                    if idx >= 0:
                                        del ch.rx_buffer[:idx + 6]
                                elif len(ch.rx_buffer) > 4096:
                                    # 长时间没有合法帧：只保留尾部，防止缓冲区无限增长
                                    del ch.rx_buffer[:-512]
                    elif kind == "err":
                        self.write_log(str(payload), "err")
                        self.chan_status_var[tag].set(f"错误：{payload}")
                        self._close_serial(tag)
        except Exception as exc:
            # 任何异常都不能打断 after 轮询链，否则串口数据将永远不再被处理
            self.write_log(f"串口轮询异常：{exc}", "err")

        self.after(50, self._poll_serial)

    def _append_rx_log(self, tag: str, data: bytes) -> None:
        self.rx_text.configure(state="normal")
        if self._rx_ascii_var.get():
            text = data.decode("latin-1", errors="replace")
        else:
            text = " ".join(f"{b:02X}" for b in data)
        if self._rx_logmode_var.get():
            ts = time.strftime("%H:%M:%S")
            self.rx_text.insert("end", f"[{ts}][{tag}] ", "info")
            self.rx_text.insert("end", text + "\n", "rx")
        else:
            self.rx_text.insert("end", text, "rx")
            if self._rx_wrap_var.get():
                self.rx_text.insert("end", "\n", "rx")
        self.rx_text.configure(state="disabled")
        if self._rx_autoscroll_var.get():
            self.rx_text.see("end")

    def _refresh_rx_display(self) -> None:
        """切换 ASCII/HEX 时按接收历史整体重绘（保留 A/B 通道来源与先后顺序）"""
        self.rx_text.configure(state="normal")
        self.rx_text.delete("1.0", "end")
        for tag, data in self._rx_history:
            if self._rx_ascii_var.get():
                text = data.decode("latin-1", errors="replace")
            else:
                text = " ".join(f"{b:02X}" for b in data)
            self.rx_text.insert("end", f"[{tag}] ", "info")
            self.rx_text.insert("end", text + "\n", "rx")
        self.rx_text.configure(state="disabled")
        if self._rx_autoscroll_var.get():
            self.rx_text.see("end")

    def _on_rx_wrap_toggle(self) -> None:
        self.rx_text.configure(wrap="word" if self._rx_wrap_var.get() else "none")

    def _clear_rx(self) -> None:
        self.rx_text.configure(state="normal")
        self.rx_text.delete("1.0", "end")
        self.rx_text.configure(state="disabled")
        self._rx_history = []
        for ch in self.chan.values():
            ch.last_rx_text = ""
            ch.rx_buffer.clear()

    # ---------------- 串口：发送 ----------------
    def _send_serial(self) -> None:
        tag = self._send_target_var.get()
        ch = self.chan[tag]
        if not ch.is_open:
            messagebox.showwarning("串口未打开",
                                   f"请先打开「串口 {tag}（{ch.desc}）」再发送数据")
            return
        raw = self.tx_text.get("1.0", "end").strip()
        if not raw:
            return
        try:
            if self._tx_ascii_var.get():
                if self._tx_escape_var.get():
                    payload = _escape_ascii(raw)
                else:
                    payload = raw.encode("latin-1", errors="replace")
                if self._tx_newline_var.get():
                    payload += b"\r\n"
            else:
                # HEX 模式：去掉空白，按字节解析
                hex_str = re.sub(r"\s+", "", raw)
                if len(hex_str) % 2:
                    messagebox.showwarning("HEX 长度错误", "HEX 字符串长度应为偶数")
                    return
                payload = bytes.fromhex(hex_str)
                if self._tx_newline_var.get():
                    payload += b"\r\n"
        except ValueError as e:
            messagebox.showwarning("HEX 解析失败", f"无法解析为 HEX：{e}")
            return

        try:
            ch.ser.write(payload)   # type: ignore[union-attr]
            self.tx_count += len(payload)
            self.tx_count_var.set(f"TX: {self.tx_count}")
            mode = "ASCII" if self._tx_ascii_var.get() else "HEX"
            self.write_log(f"串口 {tag} 发送 [{mode}]：{payload!r}", "ok")
        except Exception as e:
            self.write_log(f"串口 {tag} 发送失败：{e}", "err")

    def _clear_tx(self) -> None:
        self.tx_text.delete("1.0", "end")

    # ---------------- 关闭窗口 ----------------
    def _on_close(self) -> None:
        if getattr(self, "_timer_period", False):
            try:
                import ctypes
                ctypes.windll.winmm.timeEndPeriod(1)
            except Exception:
                pass
        try:
            if self.speaker is not None:
                self.speaker.shutdown()
        except Exception:
            pass
        self._close_serial()
        try:
            self.viewer.stop()          # 必须先停拉流线程，否则 daemon 线程会拖住退出
        except Exception:
            pass
        try:
            self.wv3d.stop()            # 通知 WebView2 窗口关闭（后台消息循环随之退出）
        except Exception:
            pass
        self.destroy()


def main() -> int:
    if "--selftest" in sys.argv:
        return selftest()
    App().mainloop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
