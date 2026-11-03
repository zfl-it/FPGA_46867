# -*- coding: utf-8 -*-
"""GUI 冒烟测试：构造界面、驱动若干帧、模拟串口输入，验证无异常。

重点验证约束：
    · 程序内部不得自动修改 speed；
    · 六个布尔量默认初始化 111110；
    · 不再提供手动修改布尔量和车速的入口；
    · 串口 6 字节 1/0 字符串能正确映射到 6 路传感器。
"""
import importlib.util
import pathlib
import sys

spec = importlib.util.spec_from_file_location(
    "trs", pathlib.Path(__file__).with_name("traffic_rules_sim.py"))
trs = importlib.util.module_from_spec(spec)
spec.loader.exec_module(trs)
trs._beep = lambda *a, **k: None                     # 静音，避免测试时打扰
trs.messagebox.showwarning = lambda *a, **k: None    # 屏蔽模态弹窗，避免阻塞
trs.messagebox.showerror = lambda *a, **k: None


def pump(app, n=5):
    """手动推进 n 帧（不依赖真实时钟的 after 队列）"""
    for _ in range(n):
        app._tick()
        app.update_idletasks()
        app.update()


app = trs.App()
app.after = lambda ms, cb=None, *a: "0"     # 禁用 after 调度，改由 pump 手动驱动
pump(app, 3)

errors = []

try:
    # 0) 初始车速必须为 0.00
    assert app.speed == 0.00, f"初始车速应为 0.00, got {app.speed}"
    print(f"  [OK] 初始车速 = {app.speed:.2f}")

    # 1) 六个布尔量默认应为真真真真真假（111110）
    bits = "".join("1" if app.state[k] else "0" for k in trs.SENSOR_KEYS)
    assert bits == trs.DEFAULT_SENSOR_BITS, f"默认传感器应为 {trs.DEFAULT_SENSOR_BITS}, got {bits}"
    print(f"  [OK] 默认传感器状态 = {bits}")

    # 2) 不再存在手动修改入口
    assert not hasattr(app, "speed_entry"), "应删除手动车速输入框"
    assert not hasattr(app, "toggle_sensor"), "应删除传感器手动切换入口"
    assert not hasattr(app, "random_scene"), "应删除随机路况功能"
    print("  [OK] 手动修改车速/布尔量入口已移除，随机路况已移除")

    # 3) 当前默认状态（111110）下，左转/右转因实线为真应报警；
    #    第 6 位 crosswalk_ahead=False（前方无斑马线），不应额外触发第 47 条
    for a in ("左转", "右转"):
        app.do_action(a)
        pump(app, 3)
        txt = app.status_var.get()
        assert txt.startswith("违规"), f"{a} 应报警, got: {txt}"
    print("  [OK] 默认状态下左转/右转因实线报警")

    # 4) 串口传入合法帧 000000 后，三个动作都应报警
    app._apply_sensor_bits("000000")
    pump(app, 3)
    for a in ("直行", "左转", "右转"):
        app.do_action(a)
        pump(app, 3)
        txt = app.status_var.get()
        assert txt.startswith("违规"), f"{a} 在 000000 下应报警, got: {txt}"
    print("  [OK] 串口 000000 后所有动作均报警")

    # 5) 串口传入 111110（默认），左转/右转仍因实线报警
    app._apply_sensor_bits(trs.DEFAULT_SENSOR_BITS)
    pump(app, 3)
    app.do_action("左转")
    pump(app, 3)
    assert app.status_var.get().startswith("违规"), "默认 111110 下左转仍应因左实线报警"
    print("  [OK] 串口 111110 后左转因左实线报警")

    # 6) 放行直行（默认状态 111110 允许直行），行驶过程中车速不应被程序改动
    app.reset()                  # 复位：清掉上一步的违规提示状态
    pump(app, 3)
    app.do_action("直行")
    pump(app, 3)
    assert app.status_var.get().startswith("允许直行"), \
        f"复位后直行应放行: {app.status_var.get()}"
    pump(app, 40)
    assert app.speed == 0.00, f"直行后 speed 被程序改动: {app.speed}"
    print("  [OK] 直行约 4 秒后车速仍为 0.00（未被自动加速）")

    app.stop_car()
    pump(app, 60)
    assert app.speed == 0.00, f"点停车后 speed 被程序改动: {app.speed}"
    print("  [OK] 点击停车后车速仍为 0.00（未被自动制动）")

    # 7) 行驶中条件失效 -> 报警但不制动
    app._apply_sensor_bits("111110")
    app.do_action("直行")
    pump(app, 3)
    app._apply_sensor_bits("111010")   # 把直行绿灯熄灭
    pump(app, 20)
    assert app.speed == 0.00, f"报警时 speed 被改动: {app.speed}"
    print(f"  [OK] 行驶条件失效报警后车速仍为 0.00")

    # 8) 复位场景回到默认值
    app.reset()
    pump(app, 3)
    assert app.speed == 0.00 and app.action is None, f"复位失败: {app.speed} {app.action}"
    reset_bits = "".join("1" if app.state[k] else "0" for k in trs.SENSOR_KEYS)
    assert reset_bits == trs.DEFAULT_SENSOR_BITS, f"复位后传感器应为默认值, got {reset_bits}"
    print("  [OK] 复位后车速归零且传感器恢复默认值 111110")

    # 9) 串口数据解析函数测试
    assert trs.try_parse_sensor_bits(b"111110") == "111110"
    assert trs.try_parse_sensor_bits(b"000000") == "000000"
    assert trs.try_parse_sensor_bits(b"prefix111110") == "111110"
    assert trs.try_parse_sensor_bits(b"123456") is None
    assert trs.try_parse_sensor_bits(b"11111") is None
    print("  [OK] 串口数据解析函数工作正常")

    # 10) 真实串口接收链路：数据进队列 -> _poll_serial 处理 -> 布尔量更新
    def feed(raw: bytes, tag="A") -> str:
        app.chan[tag].queue.put(("rx", raw))
        app._poll_serial()
        pump(app, 3)
        return "".join("1" if app.state[k] else "0" for k in trs.SENSOR_KEYS)

    assert feed(b"100000") == "100000", "串口接收链路未更新布尔量为 100000"
    print("  [OK] 串口接收链路（队列 -> _poll_serial）可更新布尔量")

    # 11) 一秒一帧的连续发送 + 换行结尾，均应被正确解析
    assert feed(b"010000\r\n") == "010000", "带 \\r\\n 结尾的帧未被解析"
    assert feed(b"001000") == "001000", "连续第二帧未被解析"
    assert feed(b"111110") == "111110", "连续第三帧未被解析"
    print("  [OK] 带换行结尾与连续多帧均能正确解析")

    # 11b) 只有 A 通道解析位串：B 通道收到同样的字节不应改动布尔量
    before = "".join("1" if app.state[k] else "0" for k in trs.SENSOR_KEYS)
    assert feed(b"000000", tag="B") == before, "B 通道的数据不应被解析为感知位串"
    assert feed(b"111111", tag="A") == "111111", "A 通道仍应正常解析"
    print("  [OK] 仅 A 通道解析感知位串，B 通道数据不影响布尔量")

    # 11c) 两路串口通道均存在且初始为关闭
    assert set(app.chan) == {"A", "B"}, "应存在 A / B 两路串口通道"
    assert not app.chan["A"].is_open and not app.chan["B"].is_open, "初始两路均应未打开"
    print("  [OK] A / B 两路串口通道均已建立且初始未打开")

    # 12) 视频采集已移除：界面不应残留采集控件；画面控制指令为 6 条
    #    （v3.17：串口 2「单路夜景修复」已移除，其余下发值不变，按钮不显示数字）
    assert not hasattr(app, "video_label"), "视频预览标签未清理"
    assert not hasattr(app, "cam_btn"), "采集开关按钮未清理"
    assert not hasattr(app, "cam_combo"), "采集设备下拉框未清理"
    assert len(trs.VIDEO_COMMANDS) == 6, "画面控制指令应为 6 条"
    assert [c for c, _l, _t in trs.VIDEO_COMMANDS] == ["0", "1", "3", "4", "5", "6"], \
        "指令数据应为 0/1/3/4/5/6（2 已移除，其余不变）"
    assert [l for _c, l, _t in trs.VIDEO_COMMANDS] == [
        "画面切换", "单路亮度自适应", "单路去雾",
        "倒车辅助线", "拼接增亮", "拼接去雾"], "按钮标签不符"
    assert all(not any(ch.isdigit() for ch in l) for _c, l, _t in trs.VIDEO_COMMANDS), \
        "按钮标签不应显示串口数字"
    assert [t for _c, _l, t in trs.VIDEO_COMMANDS] == [False] + [True] * 5, \
        "只有「画面切换」是纯动作，其余均为开关"
    print("  [OK] 画面控制 6 条（0/1/3/4/5/6，夜景修复已移除，按钮不显示数字）")

    # 13) 指令下发：ASCII / HEX / 追加换行 三种形态（默认发往 B 通道，此处改指 A）
    class _FakeSer:
        is_open = True

        def __init__(self):
            self.buf = bytearray()

        def write(self, data):
            self.buf.extend(data)

        def close(self):
            self.is_open = False

    fake = _FakeSer()
    app.chan["A"].ser = fake
    app._cmd_target_var.set("A")     # 让指令走 A 通道，便于断言

    app._cmd_hex_var.set(False)
    app._cmd_newline_var.set(False)
    for code, label, toggle in trs.VIDEO_COMMANDS:
        fake.buf.clear()
        app._send_video_cmd(code, label, toggle)
        assert bytes(fake.buf) == code.encode("ascii"), \
            f"「{label}」ASCII 下发错误: {bytes(fake.buf)!r}"
    print("  [OK] ASCII 模式：6 条指令按各自串口值下发（0/1/3/4/5/6）")

    app._cmd_hex_var.set(True)
    for code, label, toggle in trs.VIDEO_COMMANDS:
        fake.buf.clear()
        app._send_video_cmd(code, label, toggle)
        assert bytes(fake.buf) == bytes([int(code, 16)]), \
            f"「{label}」HEX 下发错误: {bytes(fake.buf)!r}"
    print("  [OK] HEX 模式：6 条指令分别下发 0x00/0x01/0x03/0x04/0x05/0x06")

    # 13b) 开关语义：每按一次翻转状态，按钮红(开)/黑(关)；「画面切换」不变色
    for code, label, toggle in trs.VIDEO_COMMANDS:
        if toggle:
            app._cmd_on[code] = False
            app._send_video_cmd(code, label, True)
            assert app._cmd_on[code] is True, f"「{label}」第一次点击应置为开"
            btn = app._cmd_btns[code]
            assert str(btn.cget("bg")).upper() == trs.RED.upper(), \
                f"「{label}」打开后按钮应变红, 实际 {btn.cget('bg')}"
            app._send_video_cmd(code, label, True)
            assert app._cmd_on[code] is False, f"「{label}」第二次点击应置为关"
            assert str(btn.cget("bg")).upper() == trs.HEAD.upper(), \
                f"「{label}」关闭后按钮应变黑, 实际 {btn.cget('bg')}"
    assert "0" not in app._cmd_btns, "「画面切换」不应参与开关状态"
    print("  [OK] 五个开关按钮：按下变红、再按变黑，「画面切换」保持纯动作不变色")

    app._cmd_hex_var.set(False)
    app._cmd_newline_var.set(True)
    fake.buf.clear()
    app._send_video_cmd("3", "单路去雾", True)
    assert bytes(fake.buf) == b"3\r\n", f"追加换行失败: {bytes(fake.buf)!r}"
    print("  [OK] 勾选追加 \\r\\n 后下发 b'3\\r\\n'")
    app._cmd_newline_var.set(False)

    # 14) 指令默认目标是 B 通道；指向未打开的通道时安全提示、不崩溃
    app._cmd_target_var.set(trs.DEFAULT_TX_CHANNEL)
    assert trs.DEFAULT_TX_CHANNEL == "B", "默认下发目标应为盘古50K 所在的 B 通道"
    app._send_video_cmd("1", "单路亮度自适应", True)     # B 未打开 -> 仅弹窗提示
    print("  [OK] 默认下发到 B 通道；串口未打开时点击指令按钮安全提示、不崩溃")

    # 15) 关闭串口安全无异常（不传参 = 两路全关）
    app._close_serial()
    assert not app.chan["A"].is_open and not app.chan["B"].is_open, "两路均应已关闭"
    print("  [OK] 关闭两路串口安全无异常")

    # 16) 手动发送区也有独立目标选择，且默认同为 B
    assert app._send_target_var.get() == trs.DEFAULT_TX_CHANNEL, "手动发送默认目标应为 B"
    print("  [OK] 手动发送区默认同样发往 B 通道（盘古50K）")

    # 17) 新协议：串口帧「6 位位串 + 空格 + 行人数量」
    app.reset()
    pump(app, 3)
    assert feed(b"111110 7") == "111110", "新协议帧未更新布尔量"
    assert app.ped_count == 7, f"行人数量应为 7, got {app.ped_count}"
    assert app.ped_var.get() == "7", "行人数量显示框未同步"
    print("  [OK] 新协议帧解析：111110 7 -> 位串 111110 + 行人 7 人")

    assert feed(b"000000 0") == "000000", "连续第二帧未更新"
    assert app.ped_count == 0, f"行人数量应回落为 0, got {app.ped_count}"
    print("  [OK] 连续帧与人数归零正常；旧协议（仅 6 位位串）仍兼容")

    # 18) 语音提示：条件成立播报一次，持续成立不重复，解除后可再次播报
    calls = []
    if app.speaker is not None:
        app.speaker.say = lambda text, lang=None, dedupe_sec=1.5, **kw: calls.append(text)
    app.reset()
    pump(app, 3)
    feed(b"000001 0")            # 只有斑马线置位 -> 播报一次
    pump(app, 3)
    assert len(calls) == 1, f"斑马线提示应播报 1 次, got {len(calls)}"
    feed(b"000001 1")            # 斑马线仍在 -> 不重复播报
    pump(app, 3)
    assert len(calls) == 1, f"斑马线持续成立不应重复播报, got {len(calls)}"
    feed(b"000000 0")            # 斑马线消失 -> 闭锁复位
    pump(app, 3)
    app._tip_last["crosswalk"] = 0.0    # 模拟 10s 冷却已过（测试无法真实等待）
    feed(b"000001 0")            # 再次遇到 -> 再播报一次
    pump(app, 3)
    assert len(calls) == 2, f"条件解除后应能再次播报, got {len(calls)}"
    print("  [OK] 斑马线提示：满足即播报一次，持续满足不重复，冷却过后可再播报")

    # 19) 行人数量 >= 5 触发一次
    calls.clear()
    app.reset()
    pump(app, 3)
    feed(b"000000 5")
    pump(app, 3)
    assert len(calls) == 1, f"行人数 5 应播报 1 次, got {len(calls)}"
    feed(b"000000 9")
    pump(app, 3)
    assert len(calls) == 1, f"人数持续 >= 5 不应重复播报, got {len(calls)}"
    feed(b"000000 4")
    pump(app, 3)
    app._tip_last["pedestrian"] = 0.0   # 模拟冷却结束
    feed(b"000000 6")
    pump(app, 3)
    assert len(calls) == 2, f"冷却结束后再次达标应再播报, got {len(calls)}"
    print("  [OK] 行人 >= 5 提示：只播报一次，冷却结束后再次达标可再播报")

    # 19b) 提示冷却（v3.14）：人数在阈值附近抖动时，冷却期内绝不重复播报，
    #      避免「注意行人」一直响、盖住左右转与直行的违规报警
    calls.clear()
    app.reset()
    pump(app, 3)
    for n in (5, 4, 5, 6, 4, 5):         # 阈值附近反复横跳
        feed(b"000000 %d" % n)
        pump(app, 3)
    assert len(calls) == 1, f"冷却期内人数抖动只应播报 1 次, got {len(calls)}"
    print(f"  [OK] 行人提示冷却 {trs.TIP_COOLDOWN_SEC:.0f}s：人数抖动不再刷屏、不再盖过违规报警")

    # 19c) v3.20 放行语音整体移除：无论先红后绿还是任何灯序变化，
    #      都不再播「已经可直行/左转/右转」「全线放行」——绿灯完全静默。
    import time as _time
    calls.clear()
    app.reset()
    pump(app, 3)
    feed(b"000000 0")            # 全红基线
    pump(app, 3)
    feed(b"111110 0")            # 圆盘红灯 -> 圆盘绿灯（先红后绿齐全）
    pump(app, 3)
    feed(b"111110 0")            # 持续绿灯
    pump(app, 3)
    feed(b"111000 0")            # 左/右转变红
    pump(app, 3)
    feed(b"111110 0")            # 左/右转恢复绿
    pump(app, 3)
    feed(b"000000 0")            # 全灭
    pump(app, 3)
    feed(b"111110 0")            # 三路同时转绿（原「全线放行」特征）
    pump(app, 3)
    assert not [t for t in calls if "可" in t or "全线" in t], \
        f"绿灯不应再播任何放行语音: {calls}"
    print("  [OK] v3.20 放行语音已移除：先红后绿/圆盘绿灯均静默")

    # 19e) 复位后再收绿灯同样静默（无武装逻辑残留）
    calls.clear()
    app.reset()                  # state 回到默认（全绿）
    pump(app, 3)
    feed(b"000000 0")            # 全红
    pump(app, 3)
    feed(b"111110 0")            # 红灯 -> 绿灯
    pump(app, 3)
    assert not [t for t in calls if "可" in t or "全线" in t], \
        f"复位后绿灯不应播报: {calls}"
    print("  [OK] v3.20 复位后绿灯同样静默")

    # 19f) v3.18 武装瞬间静默：行驶途中红灯出现（灯类理由）只记日志，
    #      不播任何报警语音、不触发报警闪烁；v3.20 起转绿也完全静默。
    calls.clear()
    app.reset()
    pump(app, 3)
    app.do_action("直行")        # 默认全绿 -> 直行合法放行
    pump(app, 3)
    calls.clear()
    feed(b"000000 0")            # 行驶途中红灯出现（武装）-> 静默
    pump(app, 3)
    assert not calls, f"红灯武装瞬间不应播任何语音: {calls}"
    assert app.alarm_until <= _time.time(), "红灯武装瞬间不应触发报警闪烁"
    feed(b"001000 0")            # 直行灯转绿 -> 完全静默
    pump(app, 3)
    assert not [t for t in calls if "可" in t or "全线" in t], \
        f"绿灯放行不应再播语音: {calls}"
    print("  [OK] v3.20 武装瞬间静默：行驶中见红灯/转绿均不播语音")
    app.reset()
    pump(app, 3)

    # 19d) 报警打断策略（v3.15）：报警可打断提示音，报警之间也互相打断；
    #      提示音不能打断提示音（保持串行，避免刷屏）
    sp = app.speaker
    if sp is not None:
        import os as _os
        import tempfile as _tf
        _fd, _tmp = _tf.mkstemp(suffix=".mp3")
        _os.close(_fd)
        sp._push_play = lambda *a, **k: None     # 不真播，只验证打断判定
        sp._path_for = lambda text, lang: _tmp
        try:
            for cur, prio, expect in ((1, 0, True), (0, 0, True), (1, 1, False)):
                sp._interrupt.clear()
                sp._cur_prio = cur
                sp._prepare("interrupt-probe", "zh", None, prio, _time.time())
                assert sp._interrupt.is_set() == expect, \
                    f"打断判定错误：正在播优先级={cur} 新语音优先级={prio}"
        finally:
            try:
                _os.remove(_tmp)
            except OSError:
                pass
        print("  [OK] 语音打断：报警打断提示音、报警互相打断，提示不打断提示")

    # 20) 违规只提示、不锁操作（v3.11）：五键任何时候都能按，动作照常执行
    app.reset()
    pump(app, 3)
    feed(b"111110 0")            # 默认状态：左转因左实线违规 -> 报警（不锁）
    app.do_action("左转")
    pump(app, 3)
    assert not app._is_locked(), "违规不应再进入操作锁定"
    assert app.action == "左转", "违规动作也应按用户意图执行（自由操作）"
    assert app.action_legal is False, "违规动作应被标记为不合规"
    for b in app.action_btns:
        assert str(b["state"]) == "normal", f"违规后按钮不应置灰：{b['text']}"

    app.do_action("直行")         # 违规后立刻按别的键：照常执行
    pump(app, 3)
    assert app.action == "直行", "违规后应能立即切换到其它动作"
    assert app.action_legal is True, "默认场景下直行应合规"

    app.do_action("左转")
    pump(app, 3)
    app.stop_car()                # 停车同样随时可用，并取消未决的转向回退
    pump(app, 3)
    assert app.action is None, "停车应能立即取消行驶意图"
    assert app._turn_return_after is None, "停车应取消未决的转向回退定时器"
    assert not app._is_locked(), "五键始终不受限制"
    app.do_action("直行")
    pump(app, 3)
    assert app.status_var.get().startswith("允许直行"), "合规动作应放行"
    print("  [OK] 违规不锁操作：五键始终可点，违规动作照常执行、只做语音提醒")

    # 20a) 七键布局与默认停车（v3.14 / v3.21 增变道）
    texts = [b["text"] for b in app.action_btns]
    assert texts == ["直行", "左转", "右转", "左变道", "右变道", "倒车", "停车"], \
        f"按键不符: {texts}"
    app.reset()
    pump(app, 3)
    assert app.action is None, "启动 / 复位后的默认状态应为停车"
    btn_stop = next(b for b in app.action_btns if b["text"] == "停车")
    assert str(btn_stop.cget("bg")).upper() == trs.RED.upper(), \
        f"停车状态下车键应红色高亮, got {btn_stop.cget('bg')}"
    # 五键下方状态行：默认停车
    assert "停车" in app._action_state_var.get(), \
        f"状态行应显示停车, got {app._action_state_var.get()!r}"
    app.do_action("直行")
    pump(app, 3)
    assert "直行" in app._action_state_var.get(), \
        f"状态行应显示直行, got {app._action_state_var.get()!r}"
    app.do_action("左转")
    pump(app, 3)
    st_txt = app._action_state_var.get()
    assert "左转" in st_txt and "回直行" in st_txt, \
        f"转向中状态行应带回退提示, got {st_txt!r}"
    app.stop_car()
    pump(app, 3)
    assert "停车" in app._action_state_var.get(), \
        f"停车后状态行应显示停车, got {app._action_state_var.get()!r}"
    print("  [OK] 七键 = 直行/左转/右转/左变道/右变道/倒车/停车，默认停车（红色），"
          "键下方状态行随动作实时更新（转向/变道带倒计时）")

    # 20b) 左/右转瞬时动作：TURN_RETURN_SEC 秒后自动回到先前状态（直行或停车）
    app.reset()
    pump(app, 3)
    app.do_action("直行")
    pump(app, 3)
    app.do_action("左转")                 # 转向前是直行 -> 回退目标 = 直行
    assert app._turn_return_state == "直行", "转向前是直行，回退目标应为直行"
    assert app.action == "左转"
    app._turn_return()                    # 测试环境 after 被禁用，手动触发
    assert app.action == "直行", "转向结束应自动恢复直行"

    app.do_action("右转")                 # 再次转向，先前状态仍是直行
    assert app._turn_return_state == "直行"
    app.stop_car()
    assert app._turn_return_after is None, "手动停车应取消回退定时器"

    app.do_action("左转")                 # 从停车状态转向 -> 回退目标 = None(停车)
    assert app._turn_return_state is None, "转向前是停车，回退目标应为停车"
    app._turn_return()
    assert app.action is None, "转向结束应自动恢复停车"

    app.do_action("直行")
    app.do_action("左转")
    app.do_action("右转")                 # 连按另一个转向：回退目标不变
    assert app._turn_return_state == "直行", "连按转向不应改变回退目标"
    print("  [OK] 左/右转 1 秒后自动回到先前状态（直行或停车），手动操作可取消回退")

    # 20c) v3.21 变道按钮：判定与左转/右转一致；合规按键播报按钮名称（六语种见第 21 节）
    app.reset()
    pump(app, 3)
    app.do_action("左变道")               # 默认 111110：左线实线 -> 违规
    pump(app, 3)
    assert app.status_var.get().startswith("违规左变道"), \
        f"左线实线时左变道应违规, got {app.status_var.get()}"

    calls.clear()
    app._apply_sensor_bits("011110")      # 左线改虚线，其余全绿
    pump(app, 3)
    app.do_action("左变道")
    pump(app, 3)
    assert app.action_legal, "左线虚线+左转绿灯时左变道应合规"
    assert "左变道" in calls, f"合规左变道应播报「左变道」: {calls}"

    # 灯红时同样判违规，且只播报警、不播按钮名
    calls.clear()
    app._apply_sensor_bits("010010")      # 左线虚线，但左转灯灭
    pump(app, 3)
    app.do_action("左变道")
    pump(app, 3)
    assert app.status_var.get().startswith("违规左变道"), \
        f"左转灯红时左变道应违规, got {app.status_var.get()}"
    assert "左变道" not in calls, f"违规时不应播按钮名: {calls}"

    calls.clear()
    app._apply_sensor_bits("001110")      # 左右线均虚线，灯全绿
    pump(app, 3)
    app.do_action("右变道")
    pump(app, 3)
    assert app.action_legal and "右变道" in calls, \
        f"合规右变道应播报「右变道」: legal={app.action_legal}, calls={calls}"

    # 变道与转向一样是瞬时动作：1 秒后自动回退，且回退不播报
    app.do_action("直行")
    pump(app, 3)
    app.do_action("左变道")
    assert app._turn_return_state == "直行", "变道前是直行，回退目标应为直行"
    calls.clear()
    app._turn_return()
    assert app.action == "直行", "变道结束应自动恢复直行"
    assert not calls, f"自动回退不应播报按钮名: {calls}"

    # 停车按钮同样播报名称
    calls.clear()
    app.stop_car()
    pump(app, 3)
    assert "停车" in calls, f"停车应播报「停车」: {calls}"
    print("  [OK] v3.21 变道按钮：判定同左/右转，合规按键播报按钮名，"
          "自动回退静默，停车按键播报「停车」")

    # 21) 六种语言词条与报警文本
    assert trs.LANG_CODES == ["zh", "en", "ar", "fr", "ru", "es"], "应为联合国六种语言"
    for code in trs.LANG_CODES:
        txt = trs.alarm_text("左转", ["r_left_solid", "r_left_light"], code)
        assert txt and len(txt) > 6, f"{code} 报警文本异常"
        assert trs.phrase("tip_crosswalk", code), f"{code} 缺斑马线提示"
        assert trs.phrase("tip_pedestrian", code), f"{code} 缺行人提示"
        for a in ("直行", "左转", "右转", "左变道", "右变道", "倒车", "停车"):
            t = trs.phrase(f"act_{a}", code)
            assert t and t != f"act_{a}", f"{code} 缺按键播报词条 act_{a}"
    print("  [OK] 阿/中/英/法/俄/西 六种语言词条齐全（含 7 条按键播报），报警文本可生成")

    # 22) 低延迟契约：报警必须携带分片(pieces)且使用最高优先级
    kws = []
    if app.speaker is not None:
        app.speaker.say = lambda text, lang=None, dedupe_sec=1.5, **kw: kws.append((text, kw))
    app.reset()
    pump(app, 3)
    feed(b"111110 0")          # 左侧实线 -> 左转违规
    app.do_action("左转")
    pump(app, 3)
    assert app.action_legal is False, "左转违规应被标记为不合规（但不锁操作）"
    assert kws, "报警未触发语音播报"
    _text, kw = kws[-1]
    assert kw.get("prio") == 0, f"报警应为最高优先级 0, got {kw.get('prio')}"
    assert kw.get("pieces"), "报警未携带分片 -> 会被迫联网合成，延迟高"
    assert len(kw["pieces"]) >= 2, "报警分片应至少包含「前缀+动作」与「原因」"
    print("  [OK] 报警走分片拼接 + 最高优先级：命中缓存即可发声，无需联网等待")

    # 23) 高频触发不积压：待播队列长度受限
    if app.speaker is not None:
        q = app.speaker._play_q
        with q._cv:
            n = len(q._items)
        assert n <= 2, f"待播队列积压过多：{n}"
        print(f"  [OK] 待播队列长度受控（当前 {n} 条），高频触发不会越播越滞后")

    # 23b) v3.21 布局：控制台固定停靠右侧、拖动已取消、画面区在左侧不被遮挡
    app.update_idletasks()
    app._apply_panel()
    app._layout_display_area()
    app.update_idletasks()
    W = app.winfo_width()
    geo = app._panel_geometry()
    assert geo is not None, "控制台几何计算失败"
    px, py, pw, ph = geo
    assert px + pw == W - trs.PANEL_MARGIN, f"控制台应贴右停靠: x={px}, w={pw}, W={W}"
    assert trs.PANEL_MAX_H is not None        # 历史常量保留（不再参与布局）
    reserve = trs.PANEL_W + trs.PANEL_MARGIN
    assert app._dock_reserve() == reserve, "右侧预留宽度应等于控制台宽度"
    holder_info = app._video_holder.place_info()
    assert int(holder_info["x"]) == (W - reserve) // 2, \
        f"画面应居中于左侧区域: x={holder_info['x']}, 期望 {(W - reserve) // 2}"
    assert holder_info["anchor"] == "center", "画面应以中心点定位"
    assert int(app._top_bar.place_info()["width"]) == W - reserve, \
        "顶部按钮条应同步收窄，避免右上角标题被控制台盖住"
    assert not app._panel_head.bind("<B1-Motion>"), "控制台标题条不应再绑定拖动"
    app._set_panel(False)
    app.update_idletasks()
    assert app._dock_reserve() == 0, "收起控制台后应释放右侧预留"
    assert int(app._video_holder.place_info()["x"]) == W // 2, "收起后画面应回到整窗居中"
    app._set_panel(True)
    app.update_idletasks()
    assert app._dock_reserve() == reserve, "重新展开控制台应恢复右侧预留"
    print("  [OK] v3.21 控制台固定右侧（无拖动），画面/顶栏/帧率徽章自动让出左侧区域")

    # 24) 倒车轨迹角度盘：帧格式 / 鼠标映射 / 串口下发
    assert trs.build_angle_frame(0) == b"A00011"
    assert trs.build_angle_frame(15) == b"A01511"
    assert trs.build_angle_frame(15, True) == b"A11511"
    assert trs.build_angle_frame(30, True) == b"A13011"
    assert trs.build_angle_frame(99, True) == b"A13011", "超限应截断到 30"
    d = app.angle_dial
    assert abs(d._angle_from_xy(d.CX + d.R, d.CY) - 30) < 0.01, "正右应为 +30°"
    assert abs(d._angle_from_xy(d.CX - d.R, d.CY) + 30) < 0.01, "正左应为 -30°"
    assert abs(d._angle_from_xy(d.CX, d.CY - 60)) < 0.01, "正上方应为 0°"

    class _FakeSer:
        is_open = True

        def __init__(self):
            self.buf = bytearray()

        def write(self, b):
            self.buf.extend(b)

    fake = _FakeSer()
    app.chan["B"].ser = fake
    d.set_value(-12)
    fake.buf.clear()
    app._angle_flush()                 # 模拟松手：强制补发最终角度
    assert bytes(fake.buf) == b"A11211", f"左 12° 帧错误: {bytes(fake.buf)}"
    assert app.angle_frame_var.get() == "A11211", "帧回显未同步"
    d.set_value(0)
    app._angle_flush()
    app.chan["B"].ser = None
    app._refresh_angle_status()        # 真实路径由 _close_serial -> _sync_link 触发
    assert "未打开" in app.angle_state_var.get(), "关串口后状态行应提示未打开"
    print("  [OK] 角度盘 -> 盘古50K 角度帧（A + dir + 十位 + 个位 + 11）")

    # 25) v3.23 AI 识别闸门：四路拼接 / 倒车辅助线 -> 通知板端暂停 ADAS 推理
    sent = []

    def _fake_gate(pause):
        sent.append(bool(pause))
        app._ai_last_sent = bool(pause)
        return True

    app._send_ai_gate = _fake_gate            # 不真的发 UDP
    app._quad_view = False
    app._ai_suspended = False
    app._ai_last_sent = None
    app._ai_gate_logged = ""
    assert app._ai_gate_reason() == "", "初始应为正常识别"
    app._sync_ai_gate()
    assert sent == [False], f"启动对齐应下发「恢复」: {sent}"

    app._cmd_target_var.set("B")
    app.chan["B"].ser = fake

    # v3.25：FPGA 画面是 5 态循环（1=HDMI 单路 … 5=四路拼接 -> 再按回单路）。
    # 只有第 VIEW_QUAD_STEP 下标（第 5 个模式）才允许判成四路拼接。
    app._view_step = 0
    app._quad_view = False
    app._paint_view_state()
    for i in range(1, trs.VIEW_CYCLE):
        app._send_video_cmd("0", "画面切换", False)
        if i != trs.VIEW_QUAD_STEP:
            assert app._quad_view is False, f"第 {i + 1}/{trs.VIEW_CYCLE} 个模式不应判成四路拼接"
            assert app._ai_suspended is False, f"第 {i + 1} 个模式不应暂停识别"
            assert sent[-1] is False or sent == [False], "非拼接模式不该发暂停"

    assert app._view_step == trs.VIEW_QUAD_STEP, "第 4 下应停在第 5 个模式（四路拼接）"
    assert app._quad_view is True, "第 5 个模式应判成四路拼接"
    assert app._ai_suspended is True, "四路拼接时应暂停识别"
    assert sent[-1] is True, "应下发「暂停」"
    assert app._ai_badge.winfo_manager(), "暂停时画面左上角应出现提示条"
    assert "5/5" in app._view_var.get(), f"界面应显示 5/5：{app._view_var.get()}"

    app._send_video_cmd("4", "倒车辅助线", True)     # 打开倒车辅助线
    assert app._ai_suspended is True, "辅助线打开同样应保持暂停"
    app._send_video_cmd("4", "倒车辅助线", True)     # 关掉辅助线，但仍在拼接视图
    assert app._ai_suspended is True, "拼接视图下关掉辅助线仍应暂停"

    n_sent = len(sent)
    app._sync_ai_gate()                      # 状态没变不该重复下发
    assert len(sent) == n_sent, "状态未变化时不应重复下发"

    app._send_video_cmd("0", "画面切换", False)      # 第 5 下 -> 回 HDMI 单路
    assert app._view_step == 0, "第 5 下应回到第 1 个模式"
    assert app._quad_view is False and app._ai_suspended is False, "两项都关应恢复识别"
    assert sent[-1] is False, "应下发「恢复」"
    assert not app._ai_badge.winfo_manager(), "恢复后提示条应消失"

    # 再循环一整圈：只在第 5 个模式暂停一次，其余模式都不暂停
    for _ in range(trs.VIEW_CYCLE):
        app._send_video_cmd("0", "画面切换", False)
        expect = (app._view_step == trs.VIEW_QUAD_STEP)
        assert app._quad_view is expect, f"模式 {app._view_step + 1} 判定错误"
        assert app._ai_suspended is expect, f"模式 {app._view_step + 1} 暂停状态错误"

    # 复位按钮：计数归零、恢复识别
    app._reset_view_step()
    assert app._view_step == 0 and app._quad_view is False
    assert app._ai_suspended is False and sent[-1] is False
    app.chan["B"].ser = None
    print("  [OK] v3.23/v3.25 AI 识别闸门：5 态画面循环中只在四路拼接暂停识别")

    # 26) v3.24 手动 AI 开关：手动关闭即暂停，与画面状态取「或」关系
    sent.clear()
    app._quad_view = False
    app._ai_manual_off = False
    app._ai_suspended = False
    app._ai_last_sent = None
    app._ai_gate_logged = ""
    app._paint_ai_btn()
    app._sync_ai_gate()
    assert sent == [False], f"启动对齐应下发「恢复」: {sent}"

    app._toggle_ai_manual()                      # 手动关闭
    assert app._ai_manual_off is True, "按钮应切到手动关闭"
    assert app._ai_suspended is True, "手动关闭后应处于暂停"
    assert sent[-1] is True, "手动关闭应下发「暂停」"
    assert "手动关闭" in app._ai_gate_reason(), "暂停原因应包含手动关闭"
    assert app._ai_badge.winfo_manager(), "手动关闭时画面左上角应有提示条"
    assert "已关闭" in app._ai_btn.cget("text"), "按钮文字应提示已关闭"

    n_sent = len(sent)
    app._sync_ai_gate()
    assert len(sent) == n_sent, "状态未变化时不应重复下发"

    app._toggle_ai_manual()                      # 手动开启
    assert app._ai_manual_off is False and app._ai_suspended is False
    assert sent[-1] is False, "手动开启应下发「恢复」"
    assert not app._ai_badge.winfo_manager(), "恢复后提示条应消失"
    assert "运行中" in app._ai_btn.cget("text"), "按钮文字应提示运行中"

    # 或关系：手动已开 + 四路拼接 -> 仍然暂停
    app._view_step = trs.VIEW_QUAD_STEP
    app._quad_view = True
    app._paint_view_state()
    app._toggle_ai_manual()                      # 手动关
    app._toggle_ai_manual()                      # 手动开，但拼接还在
    assert app._ai_manual_off is False, "手动应已开启"
    assert app._ai_suspended is True, "拼接视图下即使手动开启也应保持暂停"
    app._view_step = 0
    app._quad_view = False
    app._paint_view_state()
    app._sync_ai_gate()
    assert app._ai_suspended is False, "拼接关闭且手动开启后应恢复识别"
    print("  [OK] v3.24 手动 AI 开关：手动关闭即暂停，与画面状态取「或」")

except AssertionError as e:
    errors.append(str(e))

print("\nSMOKE FAIL: " + "; ".join(errors) if errors else "\nSMOKE TEST PASSED")
app.destroy()
sys.exit(1 if errors else 0)
