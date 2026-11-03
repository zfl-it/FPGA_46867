`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 260911: single-view branch — 4 raw 720P streams -> 4x async FIFO -> 4:1 MUX
//         -> CH4 ping-pong frame buffer (DDR) -> 5th video_rect_read_data
//
// Why the frame buffer: the four sources live on independent crystals and can
// never be forced onto the color_bar (pix_clk_w) timing by FIFOs alone. CH4
// stores whole frames and the readout re-clocks them to 720P60 exactly.
//
// sel[2:0] (already 2-FF synchronized to pix_clk_w in the top):
//   sel[2]   = 1 -> stitch view (this whole module's output is ignored)
//   sel[1:0] = 00 HDMI / 01 CAM1 / 10 CAM2 / 11 Ethernet
//
// Frame-switch hygiene: on a sel change the current (torn) frame is not
// written; writing resumes at the next mux_vs rising edge = clean frame head.
//
// Frame-head guard (same disease as CH3, fixed 260910b): the Ethernet stream
// delivers pixels immediately after vsync, while the frame_fifo_write S_ACK
// handshake clears its FIFO for ~10+ cycles. The muxed {de,data} is therefore
// delayed 16 pix_clk_w cycles; mux_vs itself is NOT delayed (using a re-created
// vs was exactly the 260910b regression).
//
// 260912 shear fixes (board showed a diagonal tear on every single view):
//   A) FIFO output qualification. rd_data HOLDS its last entry while the FIFO
//      is empty, so during blanking / Ethernet gaps mux_de stayed 1 and the
//      last pixel was re-written for the whole gap — the frame in DDR grew
//      ~30% and the readout sheared. mux_vs/mux_de are now gated by "an entry
//      was actually popped this cycle".
//   B) CH4 is a real 4-slot ring like CH0~CH3. write_len counts 256-bit beats
//      (= 4 frames), so slots 2/3 must have their own addresses; aliasing
//      them onto slots 0/1 made 2 of every 4 frames unreadable and rewrote
//      readable slots mid-read.
//   C) stream-gap realign: if mux_de stays low longer than any legal blanking
//      the source dropped out; pixels are masked until the next mux_vs so a
//      resumed stream always starts a frame at a slot boundary.
//
// 260912b single-view freeze fix: fix A must qualify ONLY de, not vs —
// gating vs by the "entry popped" strobe flickered the camera/HDMI frame
// vsync (their FIFOs run near-empty), restarting the CH4 frame write on
// every flicker; the display then froze on the previous source's frame.
// See the mux_vs assignment below for the full analysis.
//
// 260912c HDMI-strip fix: the HDMI branch additionally passes through a
// frame-geometry normalizer (section 1b) that forces exactly one frame
// start and exactly 921600 de pulses per frame before the 4:1 mux —
// the raw HDMI-in vsync/de stream is strap-configured at the RX chip and
// its anomalies only corrupt this branch (stitch re-times via
// video_scale_process, CAM/ETH are self-consistent). HDMI-in pixel clock
// also gained its missing create_clock in the .fdc this round.
//
// 260912e stale-frame fix: the ring invariant is "exactly one full frame of
// pixels (real + black pad) flows between two n0_vs pulses" — cmos_write_req_gen
// advances write/read_addr_index on every n0_vs, so a vs pulse without a full
// frame behind it moves the read pointer onto a slot that still holds the
// PREVIOUS source's picture, and the display then cycles stale slots (the
// "stuck on the pre-switch frame + jitter" board symptom). The vsync-edge
// frame start violated this: it fired n0_vs before knowing whether any de
// would follow (admission-gate-rejected frames push their vsync entry before
// g_admit drops; garbled RX streams emit bare vsyncs). The vsync edge now
// only ARMS the pending start — exactly like the 260912d idle boundary — and
// the frame opens on the first real de entry. With the vsync seen, the open
// uses guard=0 (the first de after a vsync IS the frame head, zero loss);
// without any vsync (idle-armed), the 32-cycle guard still masks garbage.
// Every remaining n0_vs source (de-open, wrap fallback, pad completion) is
// preceded by real pixel flow, so the ring can never advance onto an
// unwritten slot again.
// The .fdc pix_clk generated-clock ratio was also corrected this round:
// the PLL delivers 74.21875 MHz (50 * 95 / 64) but STA was told 67.5 MHz,
// leaving the whole pix_clk_w domain ~10% under-constrained.
//
// 260912f board symptoms killed the "every pulse has pixels behind it"
// claim: selecting HDMI after an ETH session flickered between the ETH
// last frame and black at ~30 Hz. Three independent paths put a pulse
// whose slot receives NOTHING (or pure pad black) into the ring:
//   (1) pad-completion pulse + vsync-during-pad. A vsync arriving while
//       n0_pad is running arms n0_pend (c0_vs_edge && !n0_open). Pad
//       completion then fires its pulse AND the pending frame opens one
//       cycle later with its own pulse — back-to-back, so the slot the
//       pad-completion pulse reserved receives zero pixels and keeps the
//       PREVIOUS source's picture (ETH residue).
//   (2) starved frames (gate-rejected / skip / gap-masked) end in the pad
//       path, which black-fills the rest of the window and still pulses —
//       a pure-black slot per starved frame.
//   (3) in the back-to-back case the second pulse's S_ACK re-clears the
//       write FIFO while the new window's first pixels are in flight.
// cmos_write_req_gen then steps read_addr_index onto those dead slots and
// the display cycles residue/black. The fix is NOT gating n0_vs (the write
// FSM needs a per-window S_ACK to re-latch the slot base and reset its
// beat counter, or writes overrun the slot): the WRITE pointer keeps
// advancing on every pulse, but the READ pointer now steps only when the
// slot that was just written actually received one full frame of pixels
// (section 4b): a write-pixel counter counts mux_de passes between
// pulses; when it reaches NORM_FRAME_PX the slot is complete and — after
// a drain delay that lets the hdr shift / write_buf / burst pipeline
// land — read_addr_index steps onto it. Incomplete slots are never
// displayed; the monitor holds the last complete frame instead.
// Additionally the pad black-fill of a window whose real pixel count is
// below NORM_MIN_B (3/4 frame) is no longer written at all (n0_junk), so
// starved frames cannot displace a good frame with a black one, and the
// full-width FIFO water-level wiring in frame_read_write was fixed
// (undriven bits X-poisoned the burst-size compares in simulation).
//
// 260913 raw-vs fix (geometry normalizer RETIRED): board symptom -- select
// ETH, then HDMI -> the display holds the ETH last frame forever. Comparing
// what the four branches actually hand the MUX exposed the structural
// difference: CAM1/CAM2/ETH deliver RAW FIFO pops (vs = qX[17] held level,
// de = qX[16]&vX, data = qX[15:0]) and work on board; HDMI was the only
// stream whose vs/de had to be GRANTED by the §1b FSM, and the only one
// that dead-ends. Two hard defects:
//   (a) the normalizer edge-detected c0p_vs = q0[17] & v0 -- the exact
//       pattern the 260912b board lesson banned ("vs must NOT be qualified
//       by vX"). The HDMI write clock (74.25M) sits only 0.04% above the
//       read clock (74.21875M), so this FIFO runs near-empty and the vsync
//       entry run pops as 1-cycle bursts separated by empty cycles; &v0
//       re-fragmented the run that the FIFO hold property had glued into
//       one clean edge, while CAM/ETH get their edges from the UNqualified
//       held level and are immune by construction.
//   (b) every n0_vs / n0_de emission required the FSM to successfully
//       "open a frame" (pend arm -> vs_seen -> first de -> guard -> count).
//       Any refusal (admission-gate-rejected frame leaking 1-2 de entries,
//       non-standard vsync shape, dropout recovery) yields a window with
//       zero/full-short pixels: §4b never counts NORM_FRAME_PX, the read
//       pointer never steps, and the display holds the previous source's
//       frame. (The 260912e-era ETH/black flicker and the 260912f-era
//       freeze are the same disease observed by two pointer policies.)
// The stitch branch consumes the SAME physical HDMI tap through
// video_scale_process and has been correct since 260911, so the source
// geometry is standard 720p60; and §4b already provides every protection
// the normalizer used to offer: a bare-vsync window tallies 0 pixels, a
// short window tallies <921600, anomalous double edges produce tiny
// windows -- none of these are ever displayed, and a window delivering
// MORE than 921600 pixels still fills its slot completely. The FSM could
// therefore only ever subtract frames, never add one -- and on the board
// it subtracted all of them. Retired: HDMI now feeds the MUX in exactly
// the raw form of the three working paths. Unchanged: frame admission
// gate (§1a, now also blocks a rejected frame's vsync entries entirely
// since g_admit gates (vs|de) together), frame-head guard / switch
// hygiene (§3), write request (§4), completeness-bound read pointer
// (§4b), CH4 ring (§5), readout window (§6).
//
// 260913c HDMI DE-window pinner: the four MUX inputs must hand over the
// SAME frame geometry -- CAM/ETH are architecturally exact, but the HDMI
// DE stream is whatever the RX chip straps out. A line one pixel wide or
// narrow shears the DDR frame linearly; a short window never reaches
// 921600 px and §4b then never publishes (display sticks on the previous
// source). The HDMI-only pinner (hdmi_de_pinner, section 0) re-marks the
// c0 stream BEFORE the batching FIFO, all in c0_clk:
//   - every line emits exactly 1280 DE cycles: real pixels pass 1:1,
//     a short tail is black-padded, an overflow is truncated;
//   - at most 720 lines per vs-to-vs window;
//   - if the source stalls mid-window with >= 3/4 frame of real pixels
//     (same REAL_MIN policy as 260912f), black lines top the window up
//     to 720; garbage/unlock streams stay far below the threshold, are
//     left short and §4b never displays them.
// vs passes through untouched -- one level run per frame, the exact
// shape the MUX already consumes (no regenerated vsync, the 260913
// dead-end disease cannot return). The batching pop (§1) is unchanged;
// no new clock, no new CDC, nothing in the .fdc moves. For a standard
// 720p60 source the pinner is wire-transparent (identical FIFO entry
// shape), so it can only repair, never regress, the working case.
//////////////////////////////////////////////////////////////////////////////////
module single_video_select #(
    parameter CH4_BASE    = 25'd7372800,   // = FRAME_BYTES * 16  (byte address)
    parameter SLOT_STRIDE = 25'd1843200,   // = FRAME_BYTES * 4 (one 720P frame)
    // 260912 fix: read_len/write_len count MEM-side beats, i.e. 256-bit words.
    // frame_fifo_write/read increment their counters by BURST_SIZE (=16) per
    // 512-byte burst, so len = number of 256-bit beats, NOT 64-bit words:
    //   1 true 720P frame = 1280*720 px * 2 B = 1,843,200 B = 57,600 beats
    //   one write transaction transfers len = 230,400 beats = 4 frames and
    //   sweeps the 4 ring slots exactly like CH0~CH3 (len = 4 * frame beats).
    // The first cut reused FRAME_LEN_64 (57,600): only the top quarter of the
    // frame got real data, the rest was stale DDR content.
    parameter FRAME_LEN   = 25'd230400
)(
    // ---- control (pix_clk_w domain) ----
    input  wire        rst_n,             // rstn_out
    input  wire [2:0]  sel,
    // ---- 4 raw streams (source domains) ----
    input  wire        c0_clk,            // HDMI    pixclk_hdmi_in
    input  wire        c0_vs,
    input  wire        c0_de,
    input  wire [15:0] c0_data,
    input  wire        c1_clk,            // CAM1    cmos1_pclk_16bit
    input  wire        c1_vs,
    input  wire        c1_de,
    input  wire [15:0] c1_data,
    input  wire        c2_clk,            // CAM2    cmos2_pclk_16bit
    input  wire        c2_vs,
    input  wire        c2_de,
    input  wire [15:0] c2_data,
    input  wire        c3_clk,            // ETH     eth_write_clk
    input  wire        c3_vs,
    input  wire        c3_de,
    input  wire [15:0] c3_data,
    // ---- CH4 mem side (core_clk, to arbiter ch4 ports) ----
    input  wire        mem_rst,           // ~ddr_init_done
    input  wire        mem_clk,           // core_clk
    output wire        ch4_rd_burst_req,
    output wire [9:0]  ch4_rd_burst_len,
    output wire [24:0] ch4_rd_burst_addr,
    input  wire        ch4_rd_burst_data_valid,
    input  wire [255:0] ch4_rd_burst_data,
    input  wire        ch4_rd_burst_finish,
    output wire        ch4_wr_burst_req,
    output wire [9:0]  ch4_wr_burst_len,
    output wire [24:0] ch4_wr_burst_addr,
    input  wire        ch4_wr_burst_data_req,
    output wire [255:0] ch4_wr_burst_data,
    input  wire        ch4_wr_burst_finish,
    // ---- readout timing (pix_clk_w) ----
    input  wire        pix_clk_w,
    input  wire        v2_hs,             // 5th vrd must use the v2_* timing chain
    input  wire        v2_vs,             // (same delay as vrd_eth) so its output
    input  wire        v2_de,             // aligns with v3_* — see plan §5.7
    input  wire [15:0] v2_data,
    // ---- single-view output, v3_*-aligned ----
    output wire        o_hs,
    output wire        o_vs,
    output wire        o_de,
    output wire [15:0] o_data
);
    // ============================================================
    // 0) [260913c] HDMI DE 窗口钉扎器 —— HDMI 单路独占第一级。
    //    进攒批 FIFO 之前把 DE 窗口钉成精确 1280×720：行内恒 1280 拍
    //    （不足补黑、超出截断），每窗最多 720 行；窗内实像素 ≥3/4 帧且
    //    源停发超约 2 行期时补黑行凑满 720 行。vs 原样直通。CAM/ETH
    //    架构级精确，不走此级。攒批弹出原样保留，后于第一帧冻结修复中
    //    因 CDC 撤回（见 §1 注释）。
    // ============================================================
    wire        c0p_vs, c0p_de;
    wire [15:0] c0p_data;

    hdmi_de_pinner u_hdmi_de_pinner (
        .clk     (c0_clk),
        .rst_n   (rst_n),
        .in_vs   (c0_vs),
        .in_de   (c0_de),
        .in_data (c0_data),
        .out_vs  (c0p_vs),
        .out_de  (c0p_de),
        .out_data(c0p_data)
    );

    // ============================================================
    // 1a) [260913 RETIRED] c0 写使能帧准入门 —— 与 c1~c3 完全同形。
    //     门是 c0 独占的最后一处 RTL 差异，也是 HDMI 单路异常的头号嫌疑：
    //     过速/非标源下"放行帧必饱和溢出(wr_full 丢条目→§4b 判不完整)、
    //     拒帧(vs|de 一起挡)零像素"两难 → R 冻结 → 永远显示上一源最后帧。
    //     §4b 完整性验收已能兜住过载撕裂帧（不完整槽永不上屏），门的
    //     职责实质被取代。c0_wlvl 已无消费者（攒批弹出 260913c 因 CDC 撤回）。
    //     还原方法：恢复本块 + 写使能加回 & g_admit。
    // ============================================================
    wire [9:0] c0_wlvl;

    // ============================================================
    // 1) four async FIFOs into pix_clk_w
    //    bundle = {vs, de, data[15:0]}; write only when vs|de so the
    //    125MHz Ethernet idle gaps do not overflow the FIFO.
    // ============================================================
    wire [17:0] q0, q1, q2, q3;
    wire        e0, e1, e2, e3;

    // 260913c: 攒批弹出撤回（rd_en 与 c1~c3 恢复完全同形）。原 `~e0 & (c0_wlvl>=4)`
    // 用【写钟域】水位直接门控【读钟域】弹出 —— 多 bit 跨钟采样无同步，瞬态错值
    // 会空弹破坏 {vs,de,data} 条目流（水位常态悬在 3~5 门槛附近，逐线触发），
    // 是"HDMI 只显示第一帧 + 交错花屏"的机制链之一。滴灌顾虑由 FIFO 保持特性
    // 天然化解：q0[17] 在弹空间隙保持电平，vs 仍为每帧一个干净上升沿（260912b
    // 已验证形态）；de 有 v0 门控不外泄。§0 钉扎器保证条目形状与 CAM 同构。
    afifo_18i_18o_1024 u_afifo_hdmi (
        .wr_clk(c0_clk), .wr_rst(~rst_n), .wr_en(c0p_vs | c0p_de), .wr_data({c0p_vs, c0p_de, c0p_data}),
        .wr_full(), .wr_water_level(c0_wlvl), .almost_full(),
        .rd_clk(pix_clk_w), .rd_rst(~rst_n), .rd_en(~e0), .rd_data(q0), .rd_empty(e0),
        .rd_water_level(), .almost_empty()
    );
    afifo_18i_18o_1024 u_afifo_cam1 (
        .wr_clk(c1_clk), .wr_rst(~rst_n), .wr_en(c1_vs | c1_de), .wr_data({c1_vs, c1_de, c1_data}),
        .wr_full(), .wr_water_level(), .almost_full(),
        .rd_clk(pix_clk_w), .rd_rst(~rst_n), .rd_en(~e1), .rd_data(q1), .rd_empty(e1),
        .rd_water_level(), .almost_empty()
    );
    afifo_18i_18o_1024 u_afifo_cam2 (
        .wr_clk(c2_clk), .wr_rst(~rst_n), .wr_en(c2_vs | c2_de), .wr_data({c2_vs, c2_de, c2_data}),
        .wr_full(), .wr_water_level(), .almost_full(),
        .rd_clk(pix_clk_w), .rd_rst(~rst_n), .rd_en(~e2), .rd_data(q2), .rd_empty(e2),
        .rd_water_level(), .almost_empty()
    );
    afifo_18i_18o_1024 u_afifo_eth (
        .wr_clk(c3_clk), .wr_rst(~rst_n), .wr_en(c3_vs | c3_de), .wr_data({c3_vs, c3_de, c3_data}),
        .wr_full(), .wr_water_level(), .almost_full(),
        .rd_clk(pix_clk_w), .rd_rst(~rst_n), .rd_en(~e3), .rd_data(q3), .rd_empty(e3),
        .rd_water_level(), .almost_empty()
    );

    // ============================================================
    // 1b) [260913 RETIRED] HDMI frame-geometry normalizer. See the
    // header note: its v0-qualified vsync edge was the banned 260912b
    // pattern, and its frame-opening permission could only subtract
    // frames -- on the board it starved every HDMI window, so the
    // display held the previous source's last complete frame. §4b's
    // completeness counter (260912f) fully subsumes the protection.
    // ============================================================

    // ============================================================
    // 2) 4:1 mux on sel[1:0]  (combinational, pix_clk_w domain)
    // ============================================================
    // 260912 fix A: qualify the FIFO outputs. rd_data is non-show-ahead and
    // HOLDS its last entry while the FIFO is empty, so during blanking or
    // Ethernet idle gaps q*[16] (de) stayed 1 and ch4_write_en re-wrote the
    // last pixel for the entire gap (~370 beats per H blank, ~50k per V blank,
    // millions per Ethernet inter-frame gap). The frame in DDR grew ~30% and
    // the readout sheared diagonally. vX = "an entry was popped for this qX
    // cycle": qX updates at edge T iff the FIFO was not empty at T, and eX_d
    // samples eX at that same edge, so vX is aligned with qX by construction.
    reg e0_d, e1_d, e2_d, e3_d;
    always @(posedge pix_clk_w or negedge rst_n) begin
        if (!rst_n) begin
            e0_d <= 1'b1; e1_d <= 1'b1; e2_d <= 1'b1; e3_d <= 1'b1;
        end else begin
            e0_d <= e0; e1_d <= e1; e2_d <= e2; e3_d <= e3;
        end
    end
    wire v0 = ~e0_d, v1 = ~e1_d, v2 = ~e2_d, v3 = ~e3_d;

    // 260912b fix: vs must NOT be qualified by vX. vs is a long run of entries
    // (a LEVEL), not per-cycle data. CAM/HDMI write clocks are slower than the
    // 74.25M read clock, so their FIFOs sit near-empty and the vs run pops as
    // 1-cycle bursts separated by empty cycles; gating vs by vX turned one
    // clean frame vsync into thousands of flickering rising edges. Each edge
    // re-triggers cmos_write_req_gen -> frame_fifo_write re-enters S_ACK
    // (write_cnt reset + write_buf FIFO held in aclr) mid-frame, so CH4 never
    // completes a frame and the display freezes on the previous source's last
    // written frame (the ETH picture). ETH is immune only because its 125M
    // write clock keeps a backlog, making pops contiguous.
    // qX[17] holds its last value while the FIFO is empty, which stretches the
    // pulse across the trickle gaps — exactly the clean pre-260912 behaviour.
    // Only de needs the vX gate (fix A below): a held de=1 re-writes the last
    // pixel through blanking; a held vs=1 is harmless (only its rising edge is
    // consumed, and the last entry of a frame is always a de entry).
    // 260913: HDMI now obeys the same raw-form contract as the three
    // working paths (see header note): vs = held q0[17] level, de =
    // q0[16]&v0, data = q0[15:0]. The FIFO hold property glues HDMI's
    // near-empty vsync trickle into ONE clean rising edge per source
    // frame (the 260912b-proven pattern), so both edge consumers --
    // cmos_wr_gen_ch4 (write_req / W++) and the §4b win_cnt reset --
    // see exactly one frame boundary. §4b alone decides whether the
    // window between two edges is complete enough to display.
    wire        mux_vs   = (sel[1:0] == 2'b00) ? q0[17] :
                           (sel[1:0] == 2'b01) ? q1[17] :
                           (sel[1:0] == 2'b10) ? q2[17] : q3[17];
    wire        mux_de   = (sel[1:0] == 2'b00) ? (q0[16] & v0) :
                           (sel[1:0] == 2'b01) ? (q1[16] & v1) :
                           (sel[1:0] == 2'b10) ? (q2[16] & v2) : (q3[16] & v3);
    wire [15:0] mux_data = (sel[1:0] == 2'b00) ? q0[15:0] :
                           (sel[1:0] == 2'b01) ? q1[15:0] :
                           (sel[1:0] == 2'b10) ? q2[15:0] : q3[15:0];

    // ============================================================
    // 3) frame-switch hygiene + frame-head guard
    //    {de,data} delayed 16 cycles, vs not delayed.
    // ============================================================
    // 260911 fix: hdr_dat_sr must hold 16 pixels of 16 bit = 256 bit, exactly
    // like CH3's ch3_dat_sr. Declaring it 16 bit (as first written) shifted the
    // 16-bit word into a 16-bit register, so only the LSB survived and
    // hdr_dat_sr[15] was a 1-bit "pixel" — the whole single view came out
    // bit-blasted. Tap is [255:240].
    reg  [15:0]  hdr_de_sr;
    reg  [255:0] hdr_dat_sr;
    always @(posedge pix_clk_w or negedge rst_n) begin
        if (!rst_n) begin
            hdr_de_sr  <= 16'd0;
            hdr_dat_sr <= 256'd0;
        end else begin
            hdr_de_sr  <= {hdr_de_sr[14:0],   mux_de};
            hdr_dat_sr <= {hdr_dat_sr[239:0], mux_data};
        end
    end

    reg        skip_frame;
    reg        mux_vs_d0;
    reg  [1:0] sel_d0;
    wire       mux_vs_rise = mux_vs & ~mux_vs_d0;
    wire       sel_changed = (sel_d0 != sel[1:0]);

    always @(posedge pix_clk_w or negedge rst_n) begin
        if (!rst_n) begin
            mux_vs_d0  <= 1'b0;
            sel_d0     <= 2'b00;
            skip_frame <= 1'b1;                 // first frame after reset dropped
        end else begin
            mux_vs_d0 <= mux_vs;
            sel_d0    <= sel[1:0];
            if (sel_changed)
                skip_frame <= 1'b1;             // torn frame: no write
            else if (mux_vs_rise)
                skip_frame <= 1'b0;             // clean head of the new source
        end
    end

    // 260912 fix C: stream-gap realign. If mux_de stays low longer than any
    // legal blanking (camera/HDMI V blank is ~666 us = 49.4k cycles), the
    // source dropped out mid-frame (Ethernet pusher stopped, cable pulled).
    // frame_fifo_write would freeze its transaction and later resume it from
    // a half-frame position, permanently desyncing the ring slots. Masking
    // pixels until the next mux_vs forces every resumed stream to start a
    // frame at a slot boundary again.
    reg  [17:0] gap_cnt;
    reg         gap_sticky;
    localparam  GAP_THRESH = 18'd200000;     // 2.7 ms @ 74.21875 MHz
    always @(posedge pix_clk_w or negedge rst_n) begin
        if (!rst_n) begin
            gap_cnt    <= 18'd0;
            gap_sticky <= 1'b1;                 // mask until the first vs
        end else begin
            if (mux_de)
                gap_cnt <= 18'd0;
            else if (gap_cnt < GAP_THRESH)
                gap_cnt <= gap_cnt + 1'b1;
            if (mux_vs_rise)
                gap_sticky <= 1'b0;
            else if (gap_cnt == GAP_THRESH)
                gap_sticky <= 1'b1;
        end
    end

    wire        ch4_write_en  = hdr_de_sr[15] & ~skip_frame & ~gap_sticky;
    wire [15:0] ch4_write_data = hdr_dat_sr[255:240];

    // ============================================================
    // 4) CH4 write request (4-slot ring)
    //    260912f: cmos_write_req_gen now provides ONLY the write side:
    //    every mux_vs edge fires the per-window write_req (the FSM needs
    //    its S_ACK to re-latch the slot base and reset the beat counter)
    //    and advances write_addr_index. Its read_addr_index output —
    //    which stepped onto EVERY window including pad-only/starved/
    //    double-pulsed ones — is left unconnected; the read pointer is
    //    driven in section 4b.
    // ============================================================
    wire       ch4_write_req, ch4_write_req_ack;
    wire [1:0] ch4_write_addr_index;

    cmos_write_req_gen #(.DROP_FRAMES(2)) cmos_wr_gen_ch4 (
        .rst              (~rst_n),
        .pclk             (pix_clk_w),
        .cmos_vsync       (mux_vs),
        .write_req        (ch4_write_req),
        .write_addr_index (ch4_write_addr_index),
        .read_addr_index  (),
        .write_req_ack    (ch4_write_req_ack)
    );

    // ============================================================
    // 4b) 260912f — read pointer bound to COMPLETE frame writes.
    // A mux_vs pulse means "a new window started", not "the previous slot
    // got a full frame": pad-only/starved windows, the pad-completion +
    // pending-open double pulse (its reserved slot receives zero pixels)
    // and gap/skip-masked windows all advance the WRITE ring while their
    // slot stays empty or partial — the read pointer then lands on a slot
    // still holding the previous source's picture (the ETH-residue/black
    // flicker). Here a counter tallies the pixels the write path actually
    // passes between pulses; when it reaches exactly one frame the slot is
    // complete, and after a drain delay (hdr shift + write_buf + burst
    // pipeline landing in DDR) read_addr_index steps onto it. Incomplete
    // slots are never displayed: the monitor keeps showing the last
    // complete frame. Frame granularity loss replaces stale-slot playback.
    // ============================================================
    localparam [15:0] R_DRAIN = 16'd1024;      // ~14us @74.21875M: 16-cycle hdr delay +
                                               // write_buf drain + last burst to DDR
    localparam [19:0] FRAME_PX_720 = 20'd921600;  // 1280 * 720: one complete slot

    wire win_px = mux_de & ~skip_frame & ~gap_sticky;   // pixel passed to the write path

    reg  [19:0] win_cnt;                       // write pixels since the current slot opened
    reg  [1:0]  wr_slot;                       // slot the current window writes into
    reg  [2:0]  wr_slot_s;                     // delay: req_gen increments W 2 cycles
    reg  [1:0]  done_slot;                     //   after the vs edge; latch after it settles
    reg         r_step_pend;
    reg  [15:0] r_step_cnt;
    reg  [1:0]  ch4_read_addr_index;           // display reads the LAST COMPLETE frame

    // fires exactly once per slot, on the cycle its last counted pixel
    // passes. 260913: with the normalizer gone there is no pad tail -- a
    // window's tally is simply the pixels that passed between two raw vs
    // edges, and only a full tally is ever published to the reader.
    wire win_full = win_px & (win_cnt == FRAME_PX_720 - 20'd1);

    always @(posedge pix_clk_w or negedge rst_n) begin
        if (!rst_n) begin
            win_cnt   <= 20'd0;
            wr_slot   <= 2'd0;
            wr_slot_s <= 3'd0;
        end else begin
            if (mux_vs_rise)
                // 260913: with raw vs the rise lands on a vs-entry pop, so
                // for standard sources the window starts empty here (no
                // pixel can share the pop cycle). Count a same-cycle pixel
                // anyway (sources that overlap vsync with active video) so
                // every window always tallies exactly its own pixels.
                win_cnt <= win_px ? 20'd1 : 20'd0;
            else if (win_px && (win_cnt < FRAME_PX_720))
                win_cnt <= win_cnt + 20'd1;

            wr_slot_s <= {wr_slot_s[1:0], mux_vs_rise};
            if (wr_slot_s[2])
                wr_slot <= ch4_write_addr_index;
        end
    end

    always @(posedge pix_clk_w or negedge rst_n) begin
        if (!rst_n) begin
            done_slot           <= 2'd0;
            r_step_pend         <= 1'b0;
            r_step_cnt          <= 16'd0;
            ch4_read_addr_index <= 2'd0;
        end else begin
            if (win_full) begin
                done_slot   <= wr_slot;
                r_step_cnt  <= R_DRAIN;
                r_step_pend <= 1'b1;
            end else if (r_step_pend) begin
                if (r_step_cnt == 16'd0) begin
                    ch4_read_addr_index <= done_slot;
                    r_step_pend         <= 1'b0;
                end else begin
                    r_step_cnt <= r_step_cnt - 16'd1;
                end
            end
        end
    end

    // ============================================================
    // 5) CH4 whole-frame ring buffer (same recipe as CH0~CH3)
    //    write_len = 4 frame beats -> one write transaction sweeps slots
    //    0..3 while write_addr_index increments per vs; the read side
    //    restarts each display frame at slot[read_addr_index] (= freshest
    //    complete frame). 260912 fix B: slots 2/3 had been aliased onto
    //    slots 0/1, so 2 of every 4 frames landed at addresses nothing ever
    //    reads and the readable slots were rewritten mid-read (tearing).
    //    DDR footprint: CH4_BASE .. CH4_BASE + 4*1,843,200 B ~= 14 MB.
    // ============================================================
    // slot 0..3 base addresses, pre-truncated to ADDR_BITS=25 (max used
    // address 12,902,400 fits 25 bits = 33,554,432 with room to spare)
    // 260915b: ADDR0/ADDR1 也一并定宽 —— read_addr_0/1 与 write_addr_0/1
    // 直接接 CH4_BASE / CH4_BASE+SLOT_STRIDE，而 `+` 会把表达式撑宽，
    // 端口只有 25bit，工具会报 4 条 Width mismatch（取值不变）。
    localparam [24:0] CH4_ADDR0 = CH4_BASE;              // = 7,372,800
    localparam [24:0] CH4_ADDR1 = CH4_BASE + SLOT_STRIDE;// = 9,216,000
    localparam [24:0] CH4_ADDR2 = CH4_BASE + 2*SLOT_STRIDE;
    localparam [24:0] CH4_ADDR3 = CH4_BASE + 3*SLOT_STRIDE;
    frame_read_write #(
        .MEM_DATA_BITS(256), .READ_DATA_BITS(16), .WRITE_DATA_BITS(16),
        .ADDR_BITS(25), .BUSRT_BITS(10), .BURST_SIZE(16)
    ) frw_ch4 (
        .rst             (mem_rst),
        .mem_clk         (mem_clk),
        .rd_burst_req    (ch4_rd_burst_req),  .rd_burst_len(ch4_rd_burst_len),
        .rd_burst_addr   (ch4_rd_burst_addr), .rd_burst_data_valid(ch4_rd_burst_data_valid),
        .rd_burst_data   (ch4_rd_burst_data), .rd_burst_finish(ch4_rd_burst_finish),
        .read_clk        (pix_clk_w),
        .read_req        (vrd_read_req),      .read_req_ack(vrd_read_req_ack),
        .read_finish     (),
        .read_en         (vrd_read_en),       .read_data(vrd_read_data),
        .read_addr_0     (CH4_ADDR0),                 .read_addr_1(CH4_ADDR1),
        .read_addr_2     (CH4_ADDR2),                 .read_addr_3(CH4_ADDR3),
        .read_addr_index (ch4_read_addr_index),   .read_len(FRAME_LEN),
        .wr_burst_req    (ch4_wr_burst_req),  .wr_burst_len(ch4_wr_burst_len),
        .wr_burst_addr   (ch4_wr_burst_addr), .wr_burst_data_req(ch4_wr_burst_data_req),
        .wr_burst_data   (ch4_wr_burst_data), .wr_burst_finish(ch4_wr_burst_finish),
        .write_clk       (pix_clk_w),
        .write_req       (ch4_write_req),     .write_req_ack(ch4_write_req_ack),
        .write_finish    (),
        .write_addr_0    (CH4_ADDR0),                 .write_addr_1(CH4_ADDR1),
        .write_addr_2    (CH4_ADDR2),                 .write_addr_3(CH4_ADDR3),
        .write_addr_index(ch4_write_addr_index),  .write_len(FRAME_LEN),
        .write_en        (ch4_write_en),      .write_data(ch4_write_data)
    );

    // ============================================================
    // 6) 5th video_rect_read_data: full-frame window, v2_* timing chain
    //    -> output phase-aligned with v3_* (plan §5.7)
    // ============================================================
    wire       vrd_read_req, vrd_read_req_ack, vrd_read_en;
    wire [15:0] vrd_read_data;

    video_rect_read_data #(.DATA_WIDTH(16)) vrd_single (
        .video_clk        (pix_clk_w),
        .rst              (~rst_n),
        .video_left_offset(12'd0),
        .video_top_offset (12'd0),
        .video_width      (12'd1280),
        .video_height     (12'd720),
        .read_req         (vrd_read_req),
        .read_req_ack     (vrd_read_req_ack),
        .read_en          (vrd_read_en),
        .read_data        (vrd_read_data),
        .timing_hs        (v2_hs),
        .timing_vs        (v2_vs),
        .timing_de        (v2_de),
        .timing_data      (v2_data),
        .x_cnt            (),
        .y_cnt            (),
        .hs               (o_hs),
        .vs               (o_vs),
        .de               (o_de),
        .vout_data        (o_data)
    );

endmodule

//////////////////////////////////////////////////////////////////////////////////
// hdmi_de_pinner —— 260913c HDMI 单路独占：DE 窗口钉扎器（内联本文件，免加源文件）
//
// 目的：进 MUX 前把 HDMI 流的 DE 窗口钉成精确 1280×720，与 CAM1/CAM2/ETH
// （架构级精确）四路格式一致。源端行宽异常 → DDR 帧线性斜切；行数/像素
// 不足 → 窗口凑不满 921600 → §4b 永不上屏 → 画面停在上一源。
//
// 规则（全部 c0_clk 同域，直进 u_afifo_hdmi，不碰攒批弹出）：
//   1) vs 原样直通（一帧一个电平，与今日直通同形，不生成不门控——
//      260913 已证明"再造 vsync"是 HDMI 单路死帧的病根）；
//   2) 两枚 vs 沿之间为一窗；窗内每行恒发射 1280 拍 DE：行内实像素
//      1:1 直传，行尾不足 1280 补黑，超出 1280 截断；
//   3) 每窗最多 720 行，第 720 行之后的 DE 整行丢弃；
//   4) 源中途停发（无 DE 超 PAD_GAP ≈ 2 个 720p 行期）且窗内实像素
//      ≥3/4 帧时补黑行凑满 720 行（同 260912f NORM_MIN_B 门槛；
//      解锁/垃圾流实像素远低于门槛 → 不补 → 短窗 → §4b 不上屏）。
//
// 标准源下逐拍透明：写入 FIFO 的条目形状与直通完全一致，只会修坏帧，
// 不会弄坏好帧。无新时钟、无 CDC、不占额外存储。
//////////////////////////////////////////////////////////////////////////////////
module hdmi_de_pinner #(
    parameter LINE_W   = 11'd1280,     // 行 DE 窗钉死宽度
    parameter FRAME_L  = 11'd720,      // 每窗行数上限
    parameter PAD_GAP  = 13'd3300,     // 无 DE 补黑判停（≈2 个 720p 行期）
    parameter REAL_MIN = 20'd691200    // 补黑门槛 = 3/4 × 921600
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        in_vs,
    input  wire        in_de,
    input  wire [15:0] in_data,
    output wire        out_vs,
    output wire        out_de,
    output wire [15:0] out_data
);
    // 输入打两拍：一拍采样，一拍做沿检测
    reg         in_vs_r, in_vs_rr;
    reg         in_de_r,  in_de_rr;
    reg         in_de_a;
    reg  [15:0] in_data_r, in_data_a;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_vs_r   <= 1'b0;
            in_vs_rr  <= 1'b0;
            in_de_r   <= 1'b0;
            in_de_rr  <= 1'b0;
            in_de_a   <= 1'b0;
            in_data_r <= 16'd0;
            in_data_a <= 16'd0;
        end else begin
            in_vs_r   <= in_vs;
            in_vs_rr  <= in_vs_r;
            in_de_r   <= in_de;
            in_de_rr  <= in_de_r;
            in_de_a   <= in_de_r;
            in_data_r <= in_data;
            in_data_a <= in_data_r;
        end
    end

    wire vs_edge = in_vs_r & ~in_vs_rr;   // 帧界 = 原始 vs 沿（直通同形）
    wire de_rise = in_de_r & ~in_de_rr;

    // ---- 窗口 / 行状态 ----
    reg        win_open;     // 两枚 vs 沿之间为一窗
    reg [10:0] line_cnt;     // 本窗已发射完的行数
    reg [19:0] win_real;     // 本窗已通过的实像素（补黑门槛用）
    reg        line_run;     // 行发射中（恒 LINE_W 拍）
    reg [10:0] x_cnt;
    reg [12:0] pad_cnt;      // 无 DE 计时

    // 行起点：窗内、未满 720 行、空闲、实 DE 上升沿（vs 沿当拍不起始，
    // 保证每窗第一行都在 vs 之后才开）
    wire line_start = win_open && (line_cnt < FRAME_L) && !line_run
                      && !vs_edge && de_rise;
    // 补黑行起点：实像素够格 + 源停发超时
    wire pad_start  = win_open && (line_cnt < FRAME_L) && !line_run
                      && !vs_edge && (win_real >= REAL_MIN) && (pad_cnt == PAD_GAP);
    wire line_fire  = line_start || pad_start;
    wire line_x_end = line_run && (x_cnt == LINE_W - 11'd1);

    // 行发射：一开跑恒走满 LINE_W 拍，行内数据 = 实像素 or 黑
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            line_run <= 1'b0;
            x_cnt    <= 11'd0;
        end else if (line_fire) begin
            line_run <= 1'b1;
            x_cnt    <= 11'd0;
        end else if (line_run) begin
            if (line_x_end) begin
                line_run <= 1'b0;
                x_cnt    <= 11'd0;
            end else begin
                x_cnt <= x_cnt + 11'd1;
            end
        end
    end

    // 窗口 / 行计数
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            win_open <= 1'b0;
            line_cnt <= 11'd0;
            win_real <= 20'd0;
            pad_cnt  <= 13'd0;
        end else begin
            // 补黑计时：窗外 / 行发射中 / 有 DE 活动 / vs 沿 → 清零
            if (!win_open || line_run || in_de_r || vs_edge)
                pad_cnt <= 13'd0;
            else if (pad_cnt < PAD_GAP)
                pad_cnt <= pad_cnt + 13'd1;

            if (vs_edge) begin
                win_open <= 1'b1;      // 新窗开
                line_cnt <= 11'd0;
                win_real <= 20'd0;
            end else begin
                if (line_x_end && (line_cnt < FRAME_L))
                    line_cnt <= line_cnt + 11'd1;
                // 只统计真正通过的实像素（out_de 门控行数上限）
                if (out_de && in_de_a)
                    win_real <= win_real + 20'd1;
            end
        end
    end

    assign out_vs   = in_vs_r;                        // vs 直通
    assign out_de   = line_run && (x_cnt < LINE_W);   // 行内恒 LINE_W 拍
    assign out_data = in_de_a ? in_data_a : 16'h0000; // 行尾/补黑行填黑

endmodule
