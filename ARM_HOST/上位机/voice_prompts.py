#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""多语言人声提示模块（联合国六种通用语言）—— 低延迟版 v2
================================================================

延迟问题与对策（v1 -> v2）
--------------------------
v1 的延迟几乎全部来自"报警整句每次都要联网合成"：
    报警文本是动态拼出来的（如「警告，违规左转，左侧车道线为实线，禁止左转」），
    而预热只缓存了单个词条，整句缓存命中率 = 0，于是每次报警都要走一遍
    edge-tts 网络往返（0.5~3 s，失败还要退避重试 0.8/1.6/2.4 s）。

v2 的四项关键改动：

1. **分片合成 + 本地拼接**
   把报警句拆成固定的音频片段（「警告，违规左转」+ 各条原因），
   片段数量固定（每语言 20 段），启动时后台**并发**预热一次；
   报警时只需把已缓存的片段在本地做 MP3 字节拼接（毫秒级），
   **不联网、不等待**，做到"缓存即播"。

2. **后台补齐自然整句**
   拼接版先播出去（低延迟），随后在后台联网合成一版"整句自然语调"的音频
   覆盖缓存，之后同样的报警会用更自然的版本播放，兼得速度与音质。

3. **优先级队列 + 打断**
   报警(0) > 提示(1) > 预热(5)。报警可以插队，能打断正在播放的低优先级
   提示音，也不会被预热任务堵在队列里；报警之间也互相打断（v3.15），
   连续报警时只播最新的一条。

4. **预热与播放解耦、合成并发**
   紧急合成用独立线程池（不会被预热占满），预热用 asyncio 并发（20 段从
   串行 ~12 s 降到 ~2 s）；pygame / edge-tts 的导入与声卡初始化放在后台
   启动线程完成，避免首次播放时的导入卡顿。

对外接口
--------
    sp = Speaker(on_log=print, cache_dir=...)
    sp.set_lang("zh")
    sp.say("注意行人", dedupe_sec=1.5)                     # 普通提示
    sp.say(text, lang, pieces=[...], prio=0)               # 报警（分片、最高优先级）
    sp.prefetch("zh")        # 后台预热某语言（20 段 + 常见整句）
    sp.prefetch_all()        # 后台预热全部六种语言
    sp.describe()            # 当前后端说明
    sp.shutdown()
"""

from __future__ import annotations

import hashlib
import importlib.util
import io
import itertools
import os
import queue
import subprocess
import sys
import tempfile
import threading
import time
from collections import OrderedDict
from concurrent.futures import ThreadPoolExecutor

# ---------------------------------------------------------------- 语言表
# code: 内部标识 / name: 界面显示（母语写法）/ edge: edge-tts 语音 / sapi: 系统语音名
LANGS = (
    {"code": "zh", "name": "中文",     "edge": "zh-CN-XiaoxiaoNeural", "sapi": "Microsoft Huihui Desktop"},
    {"code": "en", "name": "English",  "edge": "en-US-AvaNeural",      "sapi": "Microsoft Zira Desktop"},
    {"code": "ar", "name": "العربية",  "edge": "ar-EG-SalmaNeural",    "sapi": ""},
    {"code": "fr", "name": "Français", "edge": "fr-FR-DeniseNeural",   "sapi": ""},
    {"code": "ru", "name": "Русский",  "edge": "ru-RU-SvetlanaNeural", "sapi": ""},
    {"code": "es", "name": "Español",  "edge": "es-ES-ElviraNeural",   "sapi": ""},
)

LANG_CODES = [d["code"] for d in LANGS]
LANG_NAMES = {d["code"]: d["name"] for d in LANGS}
LANG_BY_CODE = {d["code"]: d for d in LANGS}

DEFAULT_LANG = "zh"

# 动作与原因词条键（用于枚举需要预热的音频片段）
# v3.21：新增「左变道 / 右变道」，判定条件与左转/右转完全一致，只是标签不同。
ACTIONS = ("直行", "左转", "右转", "左变道", "右变道")
REASON_KEYS = (
    "r_straight_light", "r_left_light", "r_right_light",
    "r_left_solid", "r_right_solid", "r_crosswalk",
)
TIP_KEYS = ("tip_crosswalk", "tip_pedestrian")
# 按键确认播报（v3.21）：按下行驶意图键且不违规时，播报该按钮的名称
ACTION_SAY_KEYS = ("act_直行", "act_左转", "act_右转", "act_左变道",
                   "act_右变道", "act_倒车", "act_停车")

# ---------------------------------------------------------------- 提示语词条
# 结构：PHRASES[词条][语言代码] = 文本
# warn    : 报警前缀（"警告，"）
# viol_*  : 违规动作
# r_*     : 具体违法原因（与 judge() 的理由一一对应）
# tip_*   : 提示类（不带"警告"前缀）
PHRASES: dict[str, dict[str, str]] = {
    "warn": {
        "zh": "警告，",
        "en": "Warning, ",
        "ar": "تحذير، ",
        "fr": "Attention, ",
        "ru": "Внимание, ",
        "es": "Advertencia, ",
    },

    "viol_直行": {
        "zh": "违规直行",
        "en": "illegal straight driving",
        "ar": "القيادة المستقيمة مخالفة",
        "fr": "circulation tout droit illégale",
        "ru": "нарушение при движении прямо",
        "es": "conducción recta ilegal",
    },
    "viol_左转": {
        "zh": "违规左转",
        "en": "illegal left turn",
        "ar": "الانعطاف إلى اليسار مخالف",
        "fr": "virage à gauche illégal",
        "ru": "нарушение при повороте налево",
        "es": "giro a la izquierda ilegal",
    },
    "viol_右转": {
        "zh": "违规右转",
        "en": "illegal right turn",
        "ar": "الانعطاف إلى اليمين مخالف",
        "fr": "virage à droite illégal",
        "ru": "нарушение при повороте направо",
        "es": "giro a la derecha ilegal",
    },
    "viol_左变道": {
        "zh": "违规左变道",
        "en": "illegal lane change to the left",
        "ar": "تغيير المسار إلى اليسار مخالف",
        "fr": "changement de voie à gauche illégal",
        "ru": "нарушение при смене полосы влево",
        "es": "cambio al carril izquierdo ilegal",
    },
    "viol_右变道": {
        "zh": "违规右变道",
        "en": "illegal lane change to the right",
        "ar": "تغيير المسار إلى اليمين مخالف",
        "fr": "changement de voie à droite illégal",
        "ru": "нарушение при смене полосы вправо",
        "es": "cambio al carril derecho ilegal",
    },

    "r_straight_light": {
        "zh": "直行信号灯未放行",
        "en": "the straight signal is not green",
        "ar": "إشارة المرور للأمام ليست خضراء",
        "fr": "le feu tout droit n'est pas vert",
        "ru": "прямой сигнал не зелёный",
        "es": "el semáforo de frente no está en verde",
    },
    "r_left_light": {
        "zh": "左转信号灯未放行",
        "en": "the left-turn signal is not green",
        "ar": "إشارة الانعطاف إلى اليسار ليست خضراء",
        "fr": "le feu de virage à gauche n'est pas vert",
        "ru": "сигнал поворота налево не зелёный",
        "es": "el semáforo de giro a la izquierda no está en verde",
    },
    "r_right_light": {
        "zh": "右转信号灯未放行",
        "en": "the right-turn signal is not green",
        "ar": "إشارة الانعطاف إلى اليمين ليست خضراء",
        "fr": "le feu de virage à droite n'est pas vert",
        "ru": "сигнал поворота направо не зелёный",
        "es": "el semáforo de giro a la derecha no está en verde",
    },
    "r_left_solid": {
        "zh": "左侧车道线为实线，禁止左转",
        "en": "the left lane line is solid, left turn prohibited",
        "ar": "الخط الأيسر متصل، يُمنع الانعطاف إلى اليسار",
        "fr": "la ligne de gauche est continue, virage à gauche interdit",
        "ru": "слева сплошная линия, поворот налево запрещён",
        "es": "la línea izquierda es continua, giro a la izquierda prohibido",
    },
    "r_right_solid": {
        "zh": "右侧车道线为实线，禁止右转",
        "en": "the right lane line is solid, right turn prohibited",
        "ar": "الخط الأيمن متصل، يُمنع الانعطاف إلى اليمين",
        "fr": "la ligne de droite est continue, virage à droite interdit",
        "ru": "справа сплошная линия, поворот направо запрещён",
        "es": "la línea derecha es continua, giro a la derecha prohibido",
    },
    "r_crosswalk": {
        "zh": "前方有斑马线，须停车让行",
        "en": "pedestrian crossing ahead, you must stop and yield",
        "ar": "يوجد ممر مشاة أمامك، يجب التوقف وإفساح الطريق",
        "fr": "passage piéton devant, vous devez vous arrêter et céder le passage",
        "ru": "впереди пешеходный переход, необходимо остановиться и уступить",
        "es": "hay un paso de peatones adelante, debe detenerse y ceder el paso",
    },

    "tip_crosswalk": {
        "zh": "注意斑马线",
        "en": "Caution, pedestrian crossing",
        "ar": "انتبه إلى ممر المشاة",
        "fr": "Attention au passage piéton",
        "ru": "Внимание, пешеходный переход",
        "es": "Atención, paso de peatones",
    },
    "tip_pedestrian": {
        "zh": "注意行人",
        "en": "Caution, pedestrians ahead",
        "ar": "انتبه للمشاة",
        "fr": "Attention aux piétons",
        "ru": "Внимание, пешеходы",
        "es": "Atención a los peatones",
    },

    # ---- 按键确认播报（v3.21）：不违规时按下行驶意图键，播报按钮名称 ----
    "act_直行": {
        "zh": "直行",
        "en": "Go straight",
        "ar": "التقدّم مباشرة",
        "fr": "Tout droit",
        "ru": "Прямо",
        "es": "Siga recto",
    },
    "act_左转": {
        "zh": "左转",
        "en": "Turn left",
        "ar": "الانعطاف إلى اليسار",
        "fr": "Tourner à gauche",
        "ru": "Поворот налево",
        "es": "Gire a la izquierda",
    },
    "act_右转": {
        "zh": "右转",
        "en": "Turn right",
        "ar": "الانعطاف إلى اليمين",
        "fr": "Tourner à droite",
        "ru": "Поворот направо",
        "es": "Gire a la derecha",
    },
    "act_左变道": {
        "zh": "左变道",
        "en": "Change to the left lane",
        "ar": "تغيير المسار إلى اليسار",
        "fr": "Changer de voie à gauche",
        "ru": "Смена полосы влево",
        "es": "Cambie al carril izquierdo",
    },
    "act_右变道": {
        "zh": "右变道",
        "en": "Change to the right lane",
        "ar": "تغيير المسار إلى اليمين",
        "fr": "Changer de voie à droite",
        "ru": "Смена полосы вправо",
        "es": "Cambie al carril derecho",
    },
    "act_倒车": {
        "zh": "倒车",
        "en": "Reverse",
        "ar": "الرجوع إلى الخلف",
        "fr": "Marche arrière",
        "ru": "Задний ход",
        "es": "Marcha atrás",
    },
    "act_停车": {
        "zh": "停车",
        "en": "Stop",
        "ar": "التوقف",
        "fr": "Arrêt",
        "ru": "Остановка",
        "es": "Detenerse",
    },
}


def phrase(key: str, lang: str) -> str:
    """取词条的指定语言文本；缺失时回退中文，再回退键名。"""
    tbl = PHRASES.get(key)
    if not tbl:
        return key
    return tbl.get(lang) or tbl.get(DEFAULT_LANG) or key


def alarm_text(action: str, reason_keys: list[str], lang: str) -> str:
    """拼装报警语音文本（用于日志显示与整句缓存键）：「警告，违规XX，原因1；原因2」。"""
    head = phrase("warn", lang).strip()
    viol = phrase(f"viol_{action}", lang)
    if lang == "zh":
        detail = "；".join(phrase(k, lang) for k in reason_keys)
        return f"{head}{viol}，{detail}" if detail else f"{head}{viol}"
    sep = "؛ " if lang == "ar" else "; "
    detail = sep.join(phrase(k, lang) for k in reason_keys)
    return f"{head} {viol}: {detail}" if detail else f"{head} {viol}"


def alarm_parts(action: str, reason_keys: list[str], lang: str) -> list[str]:
    """把报警句拆成**可复用**的音频片段列表（低延迟的关键）。

    片段只有两类、数量固定，因此可以全部预先合成并缓存：
        [0] "警告，违规左转"        —— 前缀 + 动作（3 种动作）
        [1..] 每条原因              —— 6 种原因
    报警时把这些片段的 MP3 在本地拼接即可发声，无需联网。
    """
    head = phrase("warn", lang).strip()
    viol = phrase(f"viol_{action}", lang)
    first = f"{head}{viol}" if lang == "zh" else f"{head} {viol}"
    return [first] + [phrase(k, lang) for k in reason_keys]


def seg_texts(lang: str) -> list[str]:
    """该语言全部需要预热的音频片段文本。

    5 个动作头 + 6 条原因 + 2 条提示 + 7 条按键确认 = 20 段。
    """
    out = [alarm_parts(a, [], lang)[0] for a in ACTIONS]
    out += [phrase(k, lang) for k in REASON_KEYS]
    out += [phrase(k, lang) for k in TIP_KEYS]
    out += [phrase(k, lang) for k in ACTION_SAY_KEYS]
    return out


# 启动时额外预热的高频整句组合（本地拼接，不联网，几乎不耗时）
COMMON_ALARMS: tuple[tuple[str, tuple[str, ...]], ...] = (
    ("直行", ("r_straight_light",)),
    ("直行", ("r_crosswalk",)),
    ("直行", ("r_straight_light", "r_crosswalk")),
    ("左转", ("r_left_solid",)),
    ("左转", ("r_left_light",)),
    ("左转", ("r_left_solid", "r_left_light")),
    ("右转", ("r_right_solid",)),
    ("右转", ("r_right_light",)),
    ("右转", ("r_right_solid", "r_right_light")),
)


# ---------------------------------------------------------------- 后端探测
def _has(mod: str) -> bool:
    try:
        return importlib.util.find_spec(mod) is not None
    except Exception:
        return False


def _beep(freq: int = 1200, ms: int = 220, times: int = 2) -> None:
    try:
        import winsound
        for _ in range(times):
            winsound.Beep(freq, ms)
            time.sleep(0.05)
    except Exception:
        pass


def _sapi_speak(text: str, voice_hint: str, script_path: str = "") -> bool:
    """用 Windows 系统语音朗读（离线兜底）。文本经临时文件传入，避免命令行转义问题。"""
    if sys.platform != "win32":
        return False
    fd, tmp = tempfile.mkstemp(suffix=".txt", prefix="tts_")
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            f.write(text)
        if script_path and os.path.isfile(script_path):
            cmd = ["powershell", "-NoProfile", "-ExecutionPolicy", "Bypass",
                   "-File", script_path, "-TextFile", tmp]
            if voice_hint:
                cmd += ["-Voice", voice_hint]
        else:
            cmd = ["powershell", "-NoProfile", "-Command",
                   "Add-Type -AssemblyName System.Speech;"
                   "$s = New-Object System.Speech.Synthesis.SpeechSynthesizer;"
                   f"$t = Get-Content -LiteralPath '{tmp}' -Raw -Encoding UTF8;"
                   "$s.Speak($t)"]
        r = subprocess.run(cmd, capture_output=True, timeout=60)
        return r.returncode == 0
    except Exception:
        return False
    finally:
        try:
            os.remove(tmp)
        except Exception:
            pass


def _strip_id3(data: bytes) -> bytes:
    """拼接前剥掉 MP3 开头的 ID3v2 标签（标签夹在中间会让播放器报错）。"""
    if len(data) > 10 and data[:3] == b"ID3":
        size = data[6:10]
        if all(b < 0x80 for b in size):          # synchsafe integer
            n = (size[0] << 21) | (size[1] << 14) | (size[2] << 7) | size[3]
            return data[10 + n:]
    return data


# ---------------------------------------------------------------- 优先级
PRIO_ALARM = 0      # 违规报警：最高，可插队；能打断提示音，也能打断正在播的报警
PRIO_TIP = 1        # 条件提示（斑马线 / 行人）
PRIO_BULK = 5       # 后台预热

# 待播队列最大长度：串口高频触发时，只允许再积压 1 条，
# 否则语音会越播越滞后，用户听到的永远是"十几秒前的路况"。
MAX_PENDING = 2


class _PrioQueue:
    """极简优先级队列：支持按条件丢弃旧任务、限制待播长度。"""

    def __init__(self) -> None:
        self._items: list[tuple] = []
        self._cv = threading.Condition()

    def put(self, item: tuple) -> None:
        with self._cv:
            self._items.append(item)
            self._cv.notify()

    def get(self, timeout: float | None = None) -> tuple:
        with self._cv:
            if not self._items:
                self._cv.wait(timeout)
            if not self._items:
                raise queue.Empty
            self._items.sort(key=lambda it: (it[0], it[1]))
            return self._items.pop(0)

    def drop(self, pred) -> None:
        with self._cv:
            self._items = [it for it in self._items if not pred(it)]

    def trim(self, maxlen: int) -> None:
        with self._cv:
            if len(self._items) <= maxlen:
                return
            self._items.sort(key=lambda it: (it[0], it[1]))
            self._items = self._items[:maxlen]


# ---------------------------------------------------------------- 播放器
class Speaker:
    """低延迟、串行播放、可缓存的多语言人声播放器。"""

    def __init__(self, on_log=None, cache_dir: str | None = None,
                 lang: str = DEFAULT_LANG, script_path: str = "") -> None:
        self.on_log = on_log or (lambda *_a, **_k: None)
        self.lang = lang if lang in LANG_CODES else DEFAULT_LANG
        self.cache_dir = cache_dir or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "voice_cache")
        try:
            os.makedirs(self.cache_dir, exist_ok=True)
        except Exception:
            pass
        self.script_path = script_path
        self.enabled = True                 # 静音开关
        self.backend = "未检测"
        self.last_latency = -1.0            # 最近一次从 say() 到开始出声的耗时（秒）

        # ---- 音频缓存 / 声卡 ----
        self._mixer_ready = False
        self._mixer_failed = False
        self._snd_cache: "OrderedDict[str, object]" = OrderedDict()
        self._snd_lock = threading.Lock()

        # ---- 去重 ----
        self._last_spoken: dict[tuple[str, str], float] = {}
        self._synth_notified: set[tuple[str, str]] = set()   # 已提示过"首次联网合成"的语句

        # ---- 播放队列（优先级，可控长度）----
        self._play_q = _PrioQueue()
        self._seq = itertools.count()
        self._cur_prio = 99                 # 当前正在播放任务的优先级
        self._interrupt = threading.Event()
        self._stop = threading.Event()

        # ---- 线程池：紧急合成与后台预热分离，预热不会堵住报警 ----
        self._urgent_pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix="tts-urg")
        self._bulk_pool = ThreadPoolExecutor(max_workers=2, thread_name_prefix="tts-bulk")
        self._prefetched: set[str] = set()
        self._prefetch_lock = threading.Lock()

        # ---- 后台启动：导入 + 声卡初始化 + 六语言预热 ----
        self._warm_thread = threading.Thread(target=self._warmup, daemon=True)
        self._warm_thread.start()
        # ---- 播放线程（唯一，避免抢声卡）----
        self._play_thread = threading.Thread(target=self._play_loop, daemon=True)
        self._play_thread.start()

    # ---------------- 日志 ----------------
    def _log(self, msg: str) -> None:
        try:
            self.on_log(msg)
        except Exception:
            pass

    # ---------------- 对外 ----------------
    def set_lang(self, lang: str) -> None:
        if lang in LANG_CODES:
            self.lang = lang

    def say(self, text: str, lang: str | None = None, dedupe_sec: float = 1.5,
            pieces: list[str] | None = None, prio: int = PRIO_TIP) -> None:
        """请求朗读一句话。立即返回，不阻塞界面。

        pieces : 可选的音频片段列表（报警时传入 alarm_parts() 的结果）；
                 给出时优先用已缓存片段在本地拼接发声，避免联网等待。
        prio   : 优先级，见 PRIO_* 常量；报警用 PRIO_ALARM 可插队并打断提示音。
        """
        if not text or not self.enabled:
            return
        lg = lang or self.lang
        now = time.time()
        if now - self._last_spoken.get((lg, text), 0.0) < dedupe_sec:
            return
        self._last_spoken[(lg, text)] = now
        t0 = time.time()
        pool = self._urgent_pool if prio <= PRIO_TIP else self._bulk_pool
        pool.submit(self._prepare, text, lg, list(pieces) if pieces else None, prio, t0)

    def prefetch(self, lang: str | None = None) -> None:
        """后台预热某语言：11 个音频片段 + 高频整句组合（本地拼接）。"""
        lg = lang or self.lang
        with self._prefetch_lock:
            if lg in self._prefetched:
                return
            self._prefetched.add(lg)
        self._bulk_pool.submit(self._prefetch_lang, lg)

    def prefetch_all(self) -> None:
        """后台预热全部六种语言（当前语言优先）。"""
        order = [self.lang] + [c for c in LANG_CODES if c != self.lang]
        for lg in order:
            self.prefetch(lg)

    def is_ready(self, lang: str | None = None) -> bool:
        """该语言的音频片段是否已预热完成（可用于界面显示"语音就绪"）。"""
        lg = lang or self.lang
        return all(os.path.isfile(self._seg_path(t, lg)) for t in seg_texts(lg))

    def shutdown(self) -> None:
        """安全关闭：先停声卡，再等线程退出，避免退出瞬间的崩溃。"""
        self._stop.set()
        self._interrupt.set()
        # ① 停止正在播放的音频并释放 SDL 音频设备
        if self._mixer_ready:
            try:
                import pygame
                pygame.mixer.stop()
                pygame.mixer.quit()
            except Exception:
                pass
            self._mixer_ready = False
        # ② 唤醒播放线程并等它退出（正在播放时会先被打断）
        try:
            self._play_q.put((99, -1, "", "", "", 0.0))
        except Exception:
            pass
        for th in (getattr(self, "_play_thread", None),
                   getattr(self, "_warm_thread", None)):
            try:
                if th is not None and th.is_alive():
                    th.join(timeout=2.0)
            except Exception:
                pass
        # ③ 线程池
        for p in (self._urgent_pool, self._bulk_pool):
            try:
                p.shutdown(wait=False)
            except Exception:
                pass

    def describe(self) -> str:
        parts = []
        if _has("edge_tts"):
            parts.append("edge-tts(在线)")
        if sys.platform == "win32":
            parts.append("系统语音(离线)")
        parts.append("蜂鸣(兜底)")
        return " / ".join(parts)

    # ---------------- 后台启动 ----------------
    def _warmup(self) -> None:
        """后台完成重活：导入 pygame / edge-tts、初始化声卡、预热六语言。"""
        if _has("pygame"):
            try:
                import pygame
                try:
                    pygame.mixer.pre_init(24000, -16, 1, 1024)
                except Exception:
                    pass
                try:
                    pygame.mixer.init(24000, -16, 1, 1024)
                except Exception:
                    pygame.mixer.init()
                self._mixer_ready = True
            except Exception as e:
                self._mixer_failed = True
                self._log(f"声卡初始化失败（{e}），将使用系统语音/蜂鸣")
        if _has("edge_tts"):
            try:
                import edge_tts  # noqa: F401   # 预热导入，避免首次播放卡顿
            except Exception:
                pass
        self.prefetch_all()

    def _ensure_mixer(self, wait: bool = False) -> bool:
        if self._mixer_ready:
            return True
        if wait and _has("pygame") and not self._mixer_failed:
            for _ in range(150):                 # 最多等 3 s 后台初始化完成
                if self._mixer_ready or self._mixer_failed:
                    break
                time.sleep(0.02)
        if self._mixer_ready:
            return True
        if not _has("pygame"):
            return False
        try:
            import pygame
            pygame.mixer.init()
            self._mixer_ready = True
        except Exception:
            self._mixer_failed = True
            return False
        return True

    # ---------------- 缓存路径 ----------------
    def _path_for(self, text: str, lang: str) -> str:
        h = hashlib.md5(f"{lang}|{text}".encode("utf-8")).hexdigest()[:16]
        return os.path.join(self.cache_dir, f"{lang}_{h}.mp3")

    def _seg_path(self, text: str, lang: str) -> str:
        """片段缓存路径（与整句共用同一命名空间，文本不同则不会冲突）。"""
        return self._path_for(text, lang)

    # ---------------- 合成 ----------------
    def _synth_edge(self, text: str, lang: str, path: str, tries: int = 2) -> bool:
        """单条在线合成（带重试）。失败时清理临时文件。"""
        if not _has("edge_tts"):
            return False
        voice = LANG_BY_CODE.get(lang, LANG_BY_CODE[DEFAULT_LANG])["edge"]
        last_err = ""
        for attempt in range(tries):
            try:
                import asyncio

                import edge_tts

                async def _run() -> None:
                    comm = edge_tts.Communicate(text, voice)
                    await comm.save(path + ".part")

                asyncio.run(_run())
                if os.path.getsize(path + ".part") > 0:
                    os.replace(path + ".part", path)
                    return True
                try:
                    os.remove(path + ".part")
                except Exception:
                    pass
                last_err = "返回音频为空"
            except Exception as e:
                last_err = str(e)
            time.sleep(0.4 * (attempt + 1))      # 退避重试（比 v1 更短）
        self._log(f"在线合成失败（{last_err}），改用系统语音")
        try:
            if os.path.exists(path + ".part"):
                os.remove(path + ".part")
        except Exception:
            pass
        return False

    def _synth_many(self, items: list[tuple[str, str, str]], limit: int = 3) -> None:
        """并发合成多条（预热用，比串行快 3~5 倍）。items: (text, lang, path)。"""
        missing = [(t, lg, p) for (t, lg, p) in items if not os.path.isfile(p)]
        if not missing or not _has("edge_tts"):
            return
        try:
            import asyncio

            import edge_tts

            sem = asyncio.Semaphore(limit)

            async def _one(text: str, lang: str, path: str) -> None:
                voice = LANG_BY_CODE.get(lang, LANG_BY_CODE[DEFAULT_LANG])["edge"]
                for attempt in range(2):
                    try:
                        async with sem:
                            await edge_tts.Communicate(text, voice).save(path + ".part")
                        if os.path.getsize(path + ".part") > 0:
                            os.replace(path + ".part", path)
                            return
                        os.remove(path + ".part")
                    except Exception:
                        pass
                    await asyncio.sleep(0.4 * (attempt + 1))
                try:
                    if os.path.exists(path + ".part"):
                        os.remove(path + ".part")
                except Exception:
                    pass

            async def _main() -> None:
                await asyncio.gather(*(_one(t, lg, p) for (t, lg, p) in missing))

            asyncio.run(_main())
        except Exception as e:
            self._log(f"批量合成异常（{e}），改为逐条合成")
            for (t, lg, p) in missing:
                self._synth_edge(t, lg, p, tries=1)

    # ---------------- 拼接 ----------------
    def _join_mp3(self, segs: list[str], out: str) -> bool:
        """把若干 MP3 片段按字节拼成一个文件（同一语音、同参数，直接拼接即可）。"""
        try:
            buf = bytearray()
            for p in segs:
                with open(p, "rb") as f:
                    buf += _strip_id3(f.read())
            if not buf:
                return False
            with open(out + ".part", "wb") as f:
                f.write(bytes(buf))
            os.replace(out + ".part", out)
            return True
        except Exception as e:
            self._log(f"音频拼接失败：{e}")
            try:
                if os.path.exists(out + ".part"):
                    os.remove(out + ".part")
            except Exception:
                pass
            return False

    def _mark_joined(self, path: str) -> None:
        try:
            with open(path + ".joined", "wb"):
                pass
        except Exception:
            pass

    def _is_joined(self, path: str) -> bool:
        return os.path.isfile(path + ".joined")

    # ---------------- 准备音频（在工作线程执行）----------------
    def _ensure_segs(self, parts: list[str], lang: str) -> list[str]:
        """确保每个片段都有缓存；缺的现场合成（通常已在预热中就绪）。"""
        paths = []
        need = []
        for t in parts:
            p = self._seg_path(t, lang)
            paths.append(p)
            if not os.path.isfile(p):
                need.append((t, lang, p))
        if need:
            self._synth_many(need, limit=3)
        return paths

    def _prepare(self, text: str, lang: str, pieces: list[str] | None,
                 prio: int, t0: float) -> None:
        """把一句话变成可播放的音频文件，然后交给播放队列。"""
        path = self._path_for(text, lang)
        if not os.path.isfile(path):
            ok = False
            if pieces:
                # ① 分片拼接：全部命中缓存时零网络往返
                segs = self._ensure_segs(pieces, lang)
                if all(os.path.isfile(p) for p in segs) and self._join_mp3(segs, path):
                    self._mark_joined(path)
                    self.backend = "edge-tts（在线·拼接）"
                    ok = True
            if not ok:
                # ② 整句在线合成（首次 / 片段缺失时的兜底）
                key = (lang, text)
                if key not in self._synth_notified:
                    self._synth_notified.add(key)
                    self._log(f"首次播报该语句，正在联网合成（{lang}）：{text[:18]}…"
                              f"，之后将直接命中缓存（毫秒级）")
                if self._synth_edge(text, lang, path):
                    self.backend = "edge-tts（在线）"
                    ok = True
            if not ok:
                # ③ 离线兜底
                self._sapi_or_beep(text, lang)
                return
        else:
            self.backend = "edge-tts（缓存）"

        self._push_play(prio, path, text, lang, t0)
        # 报警可打断正在播放的**任何**语音：包括低优先级提示音，也包括
        # 正在播放的上一条报警（v3.15：禁止语音之间互相打断，只说最新的）。
        if prio < self._cur_prio or (prio <= PRIO_ALARM and self._cur_prio <= PRIO_ALARM):
            self._interrupt.set()
        # 拼接版播完之后，后台补一条自然整句覆盖缓存（下次更自然）
        if self._is_joined(path) and _has("edge_tts"):
            self._bulk_pool.submit(self._upgrade, text, lang, path, t0)

    def _push_play(self, prio: int, path: str, text: str, lang: str, t0: float) -> None:
        """入队播放任务；新报警作废旧报警，并把积压限制在 MAX_PENDING 条。"""
        if prio <= PRIO_ALARM:
            self._play_q.drop(lambda it: it[0] <= PRIO_ALARM)   # 旧报警作废，只说最新的
        self._play_q.put((prio, next(self._seq), path, text, lang, t0))
        self._play_q.trim(MAX_PENDING)

    def _upgrade(self, text: str, lang: str, path: str, t0: float) -> None:
        """后台合成自然整句，替换拼接版缓存。"""
        tmp = path + ".full"
        if self._synth_edge(text, lang, tmp, tries=2):
            try:
                os.replace(tmp, path)
                with self._snd_lock:
                    self._snd_cache.pop(path, None)      # 失效内存解码缓存
                try:
                    os.remove(path + ".joined")
                except Exception:
                    pass
                if abs(time.time() - t0) < 60:
                    self._log(f"已缓存自然整句（{lang}）：{text[:20]}…")
            except Exception:
                pass
        else:
            try:
                if os.path.exists(tmp):
                    os.remove(tmp)
            except Exception:
                pass

    def _sapi_or_beep(self, text: str, lang: str) -> None:
        hint = LANG_BY_CODE.get(lang, LANG_BY_CODE[DEFAULT_LANG])["sapi"]
        if _sapi_speak(text, hint, self.script_path):
            self.backend = "系统语音（离线）"
            return
        self.backend = "蜂鸣（兜底）"
        _beep()

    def _prefetch_lang(self, lang: str) -> None:
        """预热一个语言：片段并发合成 + 高频整句本地拼接。"""
        self._synth_many([(t, lang, self._seg_path(t, lang)) for t in seg_texts(lang)],
                         limit=3)
        for act, keys in COMMON_ALARMS:
            txt = alarm_text(act, list(keys), lang)
            p = self._path_for(txt, lang)
            if os.path.isfile(p):
                continue
            segs = self._ensure_segs(alarm_parts(act, list(keys), lang), lang)
            if all(os.path.isfile(s) for s in segs) and self._join_mp3(segs, p):
                self._mark_joined(p)
        self._log(f"语音预热完成：{LANG_NAMES.get(lang, lang)}")

    # ---------------- 播放 ----------------
    def _sound(self, path: str):
        """取已解码的 Sound 对象（内存缓存，避免每次重复解码 MP3）。"""
        with self._snd_lock:
            snd = self._snd_cache.get(path)
            if snd is not None:
                self._snd_cache.move_to_end(path)
                return snd
        try:
            import pygame
            with open(path, "rb") as f:
                data = f.read()
            snd = pygame.mixer.Sound(file=io.BytesIO(data))
            with self._snd_lock:
                self._snd_cache[path] = snd
                while len(self._snd_cache) > 40:
                    self._snd_cache.popitem(last=False)
            return snd
        except Exception as e:
            self._log(f"音频解码失败：{e}")
            return None

    def _play_loop(self) -> None:
        while not self._stop.is_set():
            try:
                prio, _s, path, text, lang, t0 = self._play_q.get(timeout=0.2)
            except queue.Empty:
                continue
            if not path:
                return
            self._cur_prio = prio
            self._interrupt.clear()
            try:
                self.last_latency = time.time() - t0     # say() -> 开始出声
                self._do_play(path, text, lang)
            except Exception as e:
                self._log(f"语音播放异常（{e}）")
            finally:
                self._cur_prio = 99

    def _do_play(self, path: str, text: str, lang: str) -> None:
        if not self._ensure_mixer(wait=True):
            self._sapi_or_beep(text, lang)
            return
        snd = self._sound(path)
        if snd is None:
            self._sapi_or_beep(text, lang)
            return
        try:
            import pygame
            ch = snd.play()
            if ch is None:                      # 声道耗尽
                pygame.mixer.stop()
                ch = snd.play()
            if ch is None:
                self._sapi_or_beep(text, lang)
                return
            deadline = time.time() + 30
            while ch.get_busy():
                if self._interrupt.is_set():     # 更高优先级的报警来了
                    ch.stop()
                    break
                if time.time() > deadline:
                    break
                time.sleep(0.02)
        except Exception as e:
            self._log(f"音频播放失败：{e}")
            self._sapi_or_beep(text, lang)


def selftest_phrases() -> list[str]:
    """词条完整性自检：每种语言的每个词条都必须非空。"""
    bad = []
    for key, tbl in PHRASES.items():
        for code in LANG_CODES:
            if not tbl.get(code):
                bad.append(f"{key}.{code}")
    return bad


def selftest_parts() -> list[str]:
    """分片自检：报警片段必须能被 seg_texts 全覆盖（否则拼接时会现合成、产生延迟）。"""
    bad = []
    combos = ([(a, ()) for a in ACTIONS]
              + [(a, (k,)) for a in ACTIONS for k in REASON_KEYS]
              + [(a, tuple(keys)) for (a, keys) in COMMON_ALARMS])
    for lang in LANG_CODES:
        valid = set(seg_texts(lang))
        for act, keys in combos:
            for p in alarm_parts(act, list(keys), lang):
                if p not in valid:
                    bad.append(f"{lang}/{act}/{p}")
    return bad


if __name__ == "__main__":
    miss = selftest_phrases()
    print("词条缺失:", miss or "无")
    miss2 = selftest_parts()
    print("片段未覆盖:", miss2 or "无")
    for code in LANG_CODES:
        print(f"[{code}] {alarm_text('左转', ['r_left_solid', 'r_left_light'], code)}")
        print(f"        parts = {alarm_parts('左转', ['r_left_solid', 'r_left_light'], code)}")
