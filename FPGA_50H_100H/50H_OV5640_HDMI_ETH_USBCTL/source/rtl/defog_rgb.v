`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// defog_rgb — dark-channel-prior image defogging for RGB565
//
// Ported from the Xilinx "defog" reference design (He et al. dark channel prior),
// re-implemented as pure hand-written Verilog with NO Xilinx IP, for Pango PDS.
//
// Input : RGB565 {R[4:0],G[5:0],B[4:0]}, per-pixel de
// Output: RGB565 (defogged), de aligned with data
//
// Algorithm:
//   1. RGB565 -> RGB888 (bit replication, linear full-range mapping)
//   2. dark channel d = min(r,g,b)
//   3. 5x5 minimum filter on d (horizontal 5-tap + 4 line buffers + vertical 5-tap)
//   4. atmospheric light A = per-frame max of min-filtered dark, clamped [MIN_A,A_MAX]
//   5. transmission t = 255 - w*D (w = OMEGA/256 haze retention), clamped >= T0
//   6. recover J = clamp((I - A) * rcp[t] >> 16 + A, 0, 255), rcp = 255*2^16/t LUT
//   7. RGB888 -> RGB565
//
// Division is replaced by a reciprocal LUT + multiply (no divider IP).
// Line buffers are circular RAMs (infer GTP_DRM block RAM).
//
//================================================================================
// 260915 — IN-PLACE (ZERO-LATENCY) RESTRUCTURE — fixes the right/down picture
// shift seen when the module is enabled (720p panel: ~2 lines / ~5 px; measured
// on a 1080p monitor that scales 720p by 1.5: ~3 lines / ~7 px).
//================================================================================
// Why it shifted (root cause is inside this module, and it is a *delay* problem,
// not an arithmetic problem):
//   The old version delayed the RGB by DLY_DEPTH = 2 + 2*H_ACT "valid" cycles to
//   line it up with a CENTRED dark-channel window, while its de_out was only 6
//   clocks late:
//       line buffer  : 2563 valid px  = 2 lines + 3 px   (delay = DEPTH+1)
//     + prod_r/data_out : 2 free-running clocks        = 2 px
//     ------------------------------------------------------------------
//       data offset vs de: 2 lines + 5 px   (at H_ACT = 1280)
//   In the 720p single-view display path the consumer (video_select -> HDMI)
//   used to take the splice branch timing (v3_*) as its raster reference and
//   mux the DATA only, so a pure data delay appeared 1:1 as a picture offset.
//   (The 360p write side was immune because frame_fifo_write self-counts its
//   own de.) V4 260915 note: the consumer now selects hs/vs/de/data together
//   and this module sits AFTER the selector, so timing travels with data —
//   the zero-latency property below is what keeps that chain hs/vs-aligned.
//
// How it is fixed — the module is now a zero-delay "process in place" stage:
//   * The 5x5 minimum window is purely causal (current pixel + the 4 previous
//     ones), i.e. it needs NO look-ahead — so no delay line on the RGB path is
//     needed at all, and the input pixel is used where it arrives.
//   * de_out = de_in and data_out = the merge result, both combinational
//     -> de/data are aligned exactly as before the module, offset = 0.
//   * Only the haze path (D -> t -> rcp) is pipelined. That is harmless: the
//     transmission map may lag a pixel/line (haze is low-frequency, a shifted
//     t-map is invisible), while a shifted *picture* is not.
//   * The dark-channel window is therefore anchored 2 lines + 2 px *behind* the
//     corrected pixel — the minimum possible for a causal min filter. A centred
//     window would need the next 2 lines, i.e. exactly the delay that caused
//     the bug.
//   * bypass is now a bit-exact in-place passthrough (data_out == data_in), and
//     the 2562 x 24b RGB delay line (~7.7 KB RAM) is gone.
//
//================================================================================
// 260915b — LEFT-EDGE CLAMP (fixes "the leftmost ~4 source columns of the single
// view show the ORIGINAL, un-defogged picture" -> ~6-7 px on a 1080p monitor)
//================================================================================
//   A causal 5-tap window has no pixels to its left at the start of a line, and
//   the old chain simply kept shifting, so those 4 taps held the END of the
//   previous line — i.e. the window of the leftmost columns reached into the
//   opposite edge of the image. On this project that edge is black (the HDMI DE
//   pinner black-pads a short line tail; a dark scene edge behaves the same),
//   so the min filter returned D = 0 there, t = 255, and those columns came out
//   with J = I — literally the raw picture.
//   Fix: replicate (clamp) the line's first pixel instead of wrapping
//   (BORDER_REPLICATE), so the left columns get a genuine, continuous dark
//   channel instead of a black-contaminated one. Nothing changes from column 4
//   onwards, and the geometry is untouched (still zero added latency).
//
// Resource/timing note: the merge (one 9x18 signed multiply + shift/clamp) is
// the only combinational logic added on the data path; the pre-register path
// (min trees -> OMEGA -> t -> rcp LUT) is strictly SHORTER than before, because
// the big multiply no longer sits in front of the prod_r register.
//
//================================================================================
// 260915c — TIMING CLOSURE (fixes "pix_clk setup not met", WNS -0.987 ns)
//================================================================================
//   Post-route STA reported one violated domain: pix_clk -> pix_clk, WNS
//   -0.987 ns, TNS -131.78 ns, 241 failing endpoints — and all 241 of them were
//   u_defog_sng/rcp_q[*], all launched from the same source register
//   (u_enhance_sng/data_out[10]). One single combinational chain:
//
//       data_in -> dark(5x5 min tree) -> hmin -> D
//               -> OMEGA*D -> t = 255-x, clamp -> 256-entry reciprocal table
//               -> rcp_q
//
//   Measured on the routed design: 13.68 ns of data path on a 13.47 ns period
//   (74.25 MHz), split as min3 1.86 + min5(h) 4.36 + min5(v) 3.70
//   + t 2.32 + LUT 1.1, with routing at 53% of the total.
//
//   Fix — pipelining, applied ONLY to the haze side:
//     * min5/min3 rewritten as a balanced binary comparator tree (3 stages
//       instead of 4). Arithmetically identical.
//     * the five vertical taps {hmin, lb0..lb3} are registered together, so the
//       5x5 window keeps its exact shape (spacing) and only shifts phase by 1 px.
//     * D is registered (D_q) before OMEGA*D / t / the reciprocal table.
//   Result: three stages of roughly 6.2 / 3.7 / 3.5 ns.
//
//   The RGB path is untouched: i_r/i_g/i_b still come straight from data_in and
//   the merge is still combinational, so de_out/data_out keep ZERO added
//   latency and the picture geometry cannot move. D/D_q feed the transmission
//   map only, and a 3-cycle lag of a low-frequency haze map is invisible.
//
//   Side effects checked: a_max sees exactly the same pixel set (D is now one
//   clock later, so its valid flag is de_D; the frame's last pixel is folded one
//   clock after it, still long before the next frame_start), and the line
//   buffers keep the same we/din/DEPTH so lb0..lb3 are bit-identical.
//
//================================================================================
// 260916 — LEFT-EDGE, ROUND 2 : "行首那几列还是没被增强"
//================================================================================
// 现场：单路 720p 经 defog_rgb 后，屏幕最左侧约 6~7 个像素点看上去仍是原始（有雾）画面。
// 260915b 的行首横向钳位已经修掉了"窗口绕到上一行行尾"那一层，但只把问题从 ~7 列
// 压到 ~3 列，左边那一条还在。逐拍推演（比特级模型）后确认，剩下的宽度由**另外两层**
// 机制叠加而成，三层都作用在同一处，所以视觉上仍是一条连续的左边条：
//
//   层 1  横向因果窗口绕行（260915b 已修，但触发条件不够健壮）
//         行首 4 个抽头还握着上一行的**行尾**；本工程行尾是补黑的
//         （hdmi_de_pinner 行尾填黑；暗场景边缘同理）⇒ D=0、t=255、rcp=1024、J==I。
//         钳位只在 DE 出现上升沿时生效；DE 背靠背（无行消隐间隔）的非标源下
//         line_first 永不置位，钳位**整段失效**，未增强宽度从 3 列回退到 7 列。
//         -> 修法：行首判据并联 px_cnt==0（§3a）。
//
//   层 2  雾侧流水自由运行，行首吃进消隐期像素
//         D_q 与 rcp_q 原来是**无条件**寄存的，行消隐/场消隐的像素照样进流水，
//         于是行首前几拍拿到的 t/rcp 是"消隐值"（t=255 ⇒ rcp=1024 ⇒ J==I）。
//         -> 修法：两级都按其像素有效标志推进，无效周期保持（§3d / §5）。
//
//   层 3  行首根本来不及算出自己的雾估计（本层是"最后那一列"的来源）
//         causal 最小窗锚在像素之后 + 雾侧滞后 2~3 拍 ⇒ 行首第 0 列无论如何
//         也拿不到本行的雾估计，只能沿用"上一行最后一个有效值"；而那个值来自行尾，
//         行尾是补黑的 ⇒ 仍然是 rcp=1024。层 2 打门之后这一点**依然成立**。
//         -> 修法：行首保护列（前 GUARD 列）改用"上一行行首区域实测到的 rcp"，
//            每行只在行首区域采样一次（§3e）。雾是低频量，跨一行平移不可见。
//
// 结论（比特级模型，行尾补黑 4 列、消隐送黑为前提）：
//     无钳位                      : 左侧 7 列 rcp=1024（完全没增强）
//     仅 260915b 钳位              : 3 列        ；DE 无消隐间隔时回退到 7 列
//     钳位 + 层2 门控              : 3 列（层 2 单独不能解决，因为保持的是"行尾污染值"）
//     钳位 + 层2 + 层3(§3e)        : 0 列，且与 DE 有无消隐间隔无关
//
// 代价：§3a 多一个比较器；§3d/§5 各多一个使能；§3e 一个 18bit 寄存器 + 一个比较器
//       + 一个 MUX。像素通路（i_r/i_g/i_b → prod → j → defog_565）依旧**全组合**，
//       de_out/data_out 依旧零延时，几何一动不动。
//
//================================================================================
// 261023 — 移植到 360P 写侧（拼接画面四象限各自去雾）
//================================================================================
//   本模块被复制到 CAM1/CAM2/HDMI/ETH 四路写侧（video_scale_process 之后、
//   DDR 帧缓存之前），每路 640x360。这样拼接画面的四个子画面各自独立去雾：
//   暗通道 5x5 窗口不会跨象限取像素，大气光 A 也按画面分别统计。
//
//   由此必须修掉 §3a 的一条隐含前提 —— **DE 是连续的**。360P 写侧的 DE 来自
//   video_scale_process 的 2:1 抽取，是稀疏的，line_first 会恒置位，横向窗口
//   被逐拍重置。详见 §3a 开头 261023 的说明。改后行首判据只看 px_cnt，
//   与 DE 形态无关。
//
//   零延迟特性默认保持不变（PIX_DLY = 0）：de_out/data_out 仍是组合直通。
//   261023b 追加 PIX_DLY 参数：写侧四路统一取 1（像素 / rcp / de 同拍晚一级），
//   因为 eth_write_clk = 125MHz 下原组合级实测 9.57ns（WNS -1.996，见 §5 末）。
//   写侧 frame_fifo_write 按 de 计数写入，同步延迟不改变像素序列；
//   读侧 720P 链仍必须用 0（hs/vs 不跟数据走，延迟即画面右移）。
//////////////////////////////////////////////////////////////////////////////////

module defog_rgb #(
    parameter MIN_A   = 8'd160,   // min atmospheric light (floor, prevent over-darkening)
    parameter A_MAX   = 8'd245,   // max atmospheric light (cap, skip pure-white sources)
    parameter T0      = 8'd50,    // min transmission (50/255 ~ 0.196, avoid div-by-tiny)
    parameter OMEGA   = 9'd240,   // defog strength w = OMEGA/256; 256 = full removal (v1), smaller = weaker/brighter
    parameter H_ACT   = 11'd640,  // active pixels per line (720p60 single-view: 1280; 10bit would truncate 1280!)
    parameter V_ACT   = 10'd360,  // active lines per frame (720p60 single-view: 720)
    // 260916: 行首保护列数（见 §3e）。行首 GUARD 列的倒数改用"上一行行首区域"
    //         实测值，必须 >= 左边缘未增强的列数（本工程实测：无钳位 7 列 / 有钳位 3 列）。
    parameter [11:0] GUARD = 12'd8,
    // 261023b: 像素通路流水级数（0 / 1 / 2）。
    //   0 = 原始行为：de_out == de_in、data_out 纯组合。
    //       注：261028 起读侧 720P 链（u_defog_sng）改用 1 —— 顶层把 hs/vs 与
    //       de/data 按同一深度打拍，"延迟 = 右移 1 px" 的前提已解除；
    //       两者必须同深（见顶层 DEFOG_DLY）。单独改这里而不改顶层就是右移。
    //   1 = 像素 / rcp / de 同拍多打一级（切掉 px_cnt→rcp_use MUX 那段）。
    //   2 = 再在乘法输出打一级（级B 只剩 进位+clamp+布线）。
    //   写侧 360P 用 2：frame_fifo_write 按 de 计数写入，de 与 data 同步延迟，
    //       收集到的像素序列逐个不变（只是晚几个时钟开始），四象限几何一致。
    //   实测依据见 §5 末：eth_write_clk = 125MHz（8ns），零延迟版 WNS -1.996、
    //       一级版 -1.169，故两级。
    parameter        PIX_DLY = 0
)(
    input              clk,       // clock domain per instantiation (360p write side: source clock; 720p single-view: pix_clk_w)
    input              rst_n,     // active-low (rstn_out)
    input              bypass,    // 1 = passthrough (defog disabled), bit-exact & zero delay
    input              de_in,
    input  [15:0]      data_in,   // RGB565 {R[4:0],G[5:0],B[4:0]}
    output             de_out,    // 260915: combinational, == de_in (zero latency)
    output [15:0]      data_out   // 260915: combinational (zero latency, aligned with de_out)
);

    //=================================================================
    // helper functions
    //=================================================================
    function [7:0] min2;
        input [7:0] a, b;
        begin
            min2 = (a < b) ? a : b;
        end
    endfunction

    function [7:0] min3;
        input [7:0] a, b, c;
        begin
            min3 = min2(min2(a, b), c);
        end
    endfunction

    // 260915c: balanced binary tree (3 comparator stages) instead of the old
    // min3(min3(a,b,c),d,e) (4 stages). Arithmetically identical (the min of
    // five values is unique), one carry chain shorter.
    function [7:0] min5;
        input [7:0] a, b, c, d, e;
        begin
            min5 = min2(min2(min2(a, b), min2(c, d)), e);
        end
    endfunction

    function [7:0] clamp_8;
        input signed [39:0] x;
        begin
            if (x < 0)         clamp_8 = 8'd0;
            else if (x > 255)  clamp_8 = 8'd255;
            else               clamp_8 = x[7:0];
        end
    endfunction

    //=================================================================
    // 1. RGB565 -> RGB888 (bit replication)
    //=================================================================
    wire [7:0] r8 = {data_in[15:11], data_in[15:13]};
    wire [7:0] g8 = {data_in[10:5],  data_in[10:9] };
    wire [7:0] b8 = {data_in[4:0],   data_in[4:2]  };

    //=================================================================
    // 2. dark channel d = min(r,g,b)
    //=================================================================
    wire [7:0] dark = min3(r8, g8, b8);

    //=================================================================
    // frame boundary detection (deterministic pixel/line counters)
    // 260916: 由文件后段上移至此（§3a 行首判据需要 px_cnt），逻辑未改
    //=================================================================
    // 260915b: 位宽与字面量统一到声明的位宽（原来声明 12/11 位、字面量却写
    // 10'd0/9'd0，值恒为 0 所以功能无碍，但容易误读成"只有 10 位"）
    reg [11:0] px_cnt;             // 12 位：需容纳 0..1279（720p 的 H_ACT-1）
    reg [10:0] ln_cnt;             // 11 位：需容纳 0..719（720p 的 V_ACT-1）
    wire       frame_start = de_in && (px_cnt == 12'd0) && (ln_cnt == 11'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            px_cnt <= 12'd0;
            ln_cnt <= 11'd0;
        end else if (de_in) begin
            if (px_cnt == (H_ACT - 1)) begin
                px_cnt <= 12'd0;
                ln_cnt <= (ln_cnt == (V_ACT - 1)) ? 11'd0 : (ln_cnt + 1'b1);
            end else begin
                px_cnt <= px_cnt + 1'b1;
            end
        end
    end

    //=================================================================
    // 3a. horizontal 5-tap minimum filter — CAUSAL window, LEFT-EDGE CLAMPED
    //      taps = current pixel + the 4 previous ones (no look-ahead, so no
    //      RGB delay line is needed — see the 260915 header note).
    //
    //      Shifting is gated by de_in, so blanking data and intra-line DE
    //      gaps can never enter the window.
    //
    //      260915b — LINE-START CLAMP (fixes "the leftmost columns come out
    //      un-defogged"): with a plain shift chain the 4 taps of the first
    //      columns of a line still hold the END of the previous line, i.e.
    //      the 5x5 window of the leftmost 4 columns reaches into the opposite
    //      edge of the image. That tail is black on this project (the HDMI DE
    //      pinner black-pads a short line tail; a dark scene edge does the
    //      same), so D collapsed to 0 (= t 255 = no defog) for those columns
    //      and they showed the raw picture. A causal min filter must NOT wrap
    //      across the line boundary: replicate the first pixel instead
    //      (standard BORDER_REPLICATE boundary handling).
    //      On the line's first pixel the whole clamped window is that single
    //      pixel; loading the chain with it makes columns 1..3 come out as
    //      min(col0..colN) automatically, and column 4 onward sees the
    //      normal full 5-tap window (nothing changes for the rest of the line).
    //=================================================================
    reg [7:0] hr0, hr1, hr2, hr3;
    reg       de_in_d;                      // de_in delayed 1 clock; also the
                                            // valid flag of D (see §3c, de_D)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) de_in_d <= 1'b0;
        else        de_in_d <= de_in;
    end

    // 260916 (§3d): 第二级延迟标志 —— D_q / t 比 D 再晚一拍，雾侧门控要用它
    reg de_in_d2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) de_in_d2 <= 1'b0;
        else        de_in_d2 <= de_in_d;
    end

    // 261023 — 行首判据统一到 px_cnt（移植到 360P 写侧时暴露的问题）：
    //   video_scale_process 输出 640x360 是 2:1 抽取，DE 是**稀疏**的（有效/无效
    //   交替）。稀疏 DE 下 de_in_d 在两个有效像素之间恒为 0，于是原来的
    //   line_first = de_in & ~de_in_d 会在**每一个有效像素**上都成立；而
    //   line_start 用的是"或"，结果每一拍都把 {hr3,hr2,hr1,hr0} 重置成当前 dark
    //   —— 横向 5 抽头退化成 1 抽头，暗通道变成逐像素 min(r,g,b)，画面出现
    //   明显光晕与噪点。260916 那次"并联 px_cnt==0"只补了 DE 背靠背的情形，
    //   没有覆盖 DE 稀疏的情形（两者方向相反）。
    //   px_cnt 由 de_in 自计数、每个行首恒为 0，与 DE 连续与否无关。对原来那条
    //   连续 DE 的 720P 单路链，line_first 与 px_cnt==0 本就同一拍成立，逐位不变。
    wire line_start = de_in & (px_cnt == 12'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)              {hr3, hr2, hr1, hr0} <= 32'd0;
        else if (de_in) begin
            if (line_start)      {hr3, hr2, hr1, hr0} <= {4{dark}};   // clamp: replicate col 0
            else                 {hr3, hr2, hr1, hr0} <= {hr2, hr1, hr0, dark};
        end
    end

    // clamped window: on the line's first pixel all 5 taps are the same pixel
    wire [7:0] hmin = line_start ? dark : min5(dark, hr0, hr1, hr2, hr3);

    //=================================================================
    // 3b. vertical: 4 cascaded line buffers, exactly 1 line each
    //      line_buffer delay = DEPTH+1 valid writes, so DEPTH = H_ACT-1
    //      gives an exact 1-line (H_ACT valid px) delay -> the 5 vertical
    //      taps sit exactly 1 line apart and D is a true 5x5 window.
    //=================================================================
    wire [7:0] lb0, lb1, lb2, lb3;
    line_buffer #(.DEPTH(H_ACT-11'd1), .DW(8)) u_lb0 (.clk(clk), .rst_n(rst_n), .we(de_in), .din(hmin), .dout(lb0));
    line_buffer #(.DEPTH(H_ACT-11'd1), .DW(8)) u_lb1 (.clk(clk), .rst_n(rst_n), .we(de_in), .din(lb0),  .dout(lb1));
    line_buffer #(.DEPTH(H_ACT-11'd1), .DW(8)) u_lb2 (.clk(clk), .rst_n(rst_n), .we(de_in), .din(lb1),  .dout(lb2));
    line_buffer #(.DEPTH(H_ACT-11'd1), .DW(8)) u_lb3 (.clk(clk), .rst_n(rst_n), .we(de_in), .din(lb2),  .dout(lb3));

    //-----------------------------------------------------------------
    // 3c. 260915c — TIMING FIX: register the five vertical taps TOGETHER
    //-----------------------------------------------------------------
    // All five taps move on the same clock edge, so their spacing (one line
    // each) is preserved exactly and the 5x5 window keeps its shape — only its
    // PHASE shifts by 1 px. That is invisible for a low-frequency haze map,
    // exactly like the rcp_q lag below. Nothing on the RGB data path moves.
    // The line buffers themselves are untouched (same we = de_in, same
    // din = hmin, same DEPTH), so lb0..lb3 are bit-identical to before.
    //
    // Before this stage the single combinational path was
    //     data_in -> dark -> hmin -> D -> OMEGA*D -> t -> rcp table -> rcp_q
    // = 13.7 ns on a 13.47 ns pix_clk period (WNS -0.987 ns, all 241 failing
    //   endpoints of the design). Splitting it here gives three stages of
    //   roughly 6.2 / 3.7 / 3.5 ns.
    reg [7:0] ht0, lt0, lt1, lt2, lt3;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) {ht0, lt0, lt1, lt2, lt3} <= 40'd0;
        else        {ht0, lt0, lt1, lt2, lt3} <= {hmin, lb0, lb1, lb2, lb3};
    end

    // valid flag for D: every tap below is a register whose input was valid
    // iff de_in was high one clock earlier — that is exactly de_in_d from §3a,
    // so the same delayed flag is reused instead of a duplicate register.
    wire de_D = de_in_d;

    // min-filtered dark channel D, valid when de_D is high
    // (window still anchored behind the pixel being corrected, now 1 px more)
    wire [7:0] D = min5(ht0, lt0, lt1, lt2, lt3);

    //-----------------------------------------------------------------
    // 3d. 260915c — second haze-side pipeline stage
    //-----------------------------------------------------------------
    // Everything downstream of D (OMEGA*D -> t -> reciprocal table) is
    // combinational up to rcp_q. Registering D splits that tail in two so
    // each half comfortably fits the clock period. D_q feeds ONLY the
    // transmission map; the RGB data path never sees it.
    //
    // 260916 — 层 2：D 原来是无条件寄存的，行/场消隐的像素照样进流水。消隐期的
    //   像素不属于画面，它的暗通道会给出完全错误的 t ⇒ 行首前几拍拿到"消隐值"
    //   （t=255 ⇒ rcp=1024 ⇒ J==I ⇒ 完全没增强）。改为只在 D 有效时推进，
    //   无效周期保持（跨过消隐）。active 期间 de_D 恒 1，行为与原来逐位相同。
    reg [7:0] D_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)       D_q <= 8'd0;
        else if (de_D)    D_q <= D;      // 无效周期保持（层 2）
    end

    // D_q / t / rcp_val 的有效标志（比 de_D 再晚一拍）
    wire de_Dq = de_in_d2;

    //=================================================================
    // frame boundary detection — 260916: 本块**整块上移**到 §2 之后。
    // 原因：§3a 的行首判据要用 px_cnt（DE 无消隐间隔时也能认出"行首"），
    // 而 Verilog 要求先声明后使用。逻辑一字未改，只是位置前移。
    //=================================================================

    //=================================================================
    // 4. atmospheric light A (per-frame max of D, latched at frame start)
    //    261024: 统计源由组合的 D 改为已寄存的 D_q（见下方注释），
    //    260915c 时"末像素晚一拍折叠"的结论顺延为晚两拍，余量不变。
    //=================================================================
    reg [7:0]  a_max;
    reg [7:0]  a_prev;

    // 261024: a_max 改用已经存在的 D_q / de_Dq（而不是组合的 D / de_D）。
    //   动机：ETH 那一路 125MHz 域原来最差路径是
    //     lt0[] -> min5(3 级) -> (D > a_max) -> a_max 的 CE      共 8 级
    //   换成 D_q 后，min5 只走到 D_q 寄存器（6 级），a_max 这边只剩
    //     D_q -> (D_q > a_max) -> CE                              共 2 级
    //   语义完全等价：D_q(t)=D(t-1)、de_Dq(t)=de_D(t-1)，a_max 看到的像素
    //   序列一模一样（含稀疏 DE 的 ETH 360P 写侧），只是整体晚一拍；
    //   末像素折叠仍远早于下一个 frame_start（中间隔着 ~30 行消隐）。
    //   D_q 原本只喂传输率 t / rcp，这里把它复用到 A 的统计，不新增寄存器。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_max  <= 8'd0;
            a_prev <= A_MAX;
        end else if (frame_start) begin
            a_prev <= (a_max > MIN_A) ? ((a_max < A_MAX) ? a_max : A_MAX) : MIN_A;
            a_max  <= 8'd0;
        end else if (de_Dq && (D_q > a_max)) begin
            a_max <= D_q;
        end
    end

    //=================================================================
    // alignment (260915): no RGB delay line any more — the input pixel is
    // used in place, so data_out/de_out carry ZERO added latency. The old
    // DLY_DEPTH = 2 + 2*H_ACT delay line was what shifted the picture.
    // 261023b: i_r/i_g/i_b 与 rcp_final 的定义移到 §3e 之后（PIX_DLY 延迟级要
    //          同时用到 rcp_use，而 rcp_use 在本处之后才声明）。
    //=================================================================

    //=================================================================
    // 5. transmission t = 255 - D, clamped >= T0 ; reciprocal LUT
    //=================================================================
    wire [16:0] omega_dark = OMEGA * D_q;           // 9x8 -> 17-bit (D_q = haze-side stage 2)
    wire [7:0]  t_raw = 8'd255 - omega_dark[15:8];  // t = 1 - (OMEGA/256)*D
    wire [7:0]  t     = (t_raw > T0) ? t_raw : T0;

    // reciprocal of NORMALIZED transmission, 16 fractional bits:
    //   t_norm = t/255, so 1/t_norm = 255/t
    //   rcp = floor(255 * 2^16 / t) = floor(16711680 / t)   (t clamped >= T0)
    //   260915b: 全表 229 项与 floor 逐项核对一致，与 round 有 104 项不符 —— 是向下取整
    // Combinational case LUT (no `initial` block, synthesizer-safe).
    // 260915: the table itself is unchanged (all tuned values preserved); it is
    // now consumed as rcp_val[23:6] by the registered rcp_q below, see §6.
    reg [23:0] rcp_val;
    always @(*) begin
        case (t)
            8'd0   : rcp_val = 24'd642756;
            8'd1   : rcp_val = 24'd642756;
            8'd2   : rcp_val = 24'd642756;
            8'd3   : rcp_val = 24'd642756;
            8'd4   : rcp_val = 24'd642756;
            8'd5   : rcp_val = 24'd642756;
            8'd6   : rcp_val = 24'd642756;
            8'd7   : rcp_val = 24'd642756;
            8'd8   : rcp_val = 24'd642756;
            8'd9   : rcp_val = 24'd642756;
            8'd10  : rcp_val = 24'd642756;
            8'd11  : rcp_val = 24'd642756;
            8'd12  : rcp_val = 24'd642756;
            8'd13  : rcp_val = 24'd642756;
            8'd14  : rcp_val = 24'd642756;
            8'd15  : rcp_val = 24'd642756;
            8'd16  : rcp_val = 24'd642756;
            8'd17  : rcp_val = 24'd642756;
            8'd18  : rcp_val = 24'd642756;
            8'd19  : rcp_val = 24'd642756;
            8'd20  : rcp_val = 24'd642756;
            8'd21  : rcp_val = 24'd642756;
            8'd22  : rcp_val = 24'd642756;
            8'd23  : rcp_val = 24'd642756;
            8'd24  : rcp_val = 24'd642756;
            8'd25  : rcp_val = 24'd642756;
            8'd26  : rcp_val = 24'd642756;
            8'd27  : rcp_val = 24'd618951;
            8'd28  : rcp_val = 24'd596845;
            8'd29  : rcp_val = 24'd576264;
            8'd30  : rcp_val = 24'd557056;
            8'd31  : rcp_val = 24'd539086;
            8'd32  : rcp_val = 24'd522240;
            8'd33  : rcp_val = 24'd506414;
            8'd34  : rcp_val = 24'd491520;
            8'd35  : rcp_val = 24'd477476;
            8'd36  : rcp_val = 24'd464213;
            8'd37  : rcp_val = 24'd451667;
            8'd38  : rcp_val = 24'd439781;
            8'd39  : rcp_val = 24'd428504;
            8'd40  : rcp_val = 24'd417792;
            8'd41  : rcp_val = 24'd407601;
            8'd42  : rcp_val = 24'd397897;
            8'd43  : rcp_val = 24'd388643;
            8'd44  : rcp_val = 24'd379810;
            8'd45  : rcp_val = 24'd371370;
            8'd46  : rcp_val = 24'd363297;
            8'd47  : rcp_val = 24'd355567;
            8'd48  : rcp_val = 24'd348160;
            8'd49  : rcp_val = 24'd341054;
            8'd50  : rcp_val = 24'd334233;
            8'd51  : rcp_val = 24'd327680;
            8'd52  : rcp_val = 24'd321378;
            8'd53  : rcp_val = 24'd315314;
            8'd54  : rcp_val = 24'd309475;
            8'd55  : rcp_val = 24'd303848;
            8'd56  : rcp_val = 24'd298422;
            8'd57  : rcp_val = 24'd293187;
            8'd58  : rcp_val = 24'd288132;
            8'd59  : rcp_val = 24'd283248;
            8'd60  : rcp_val = 24'd278528;
            8'd61  : rcp_val = 24'd273961;
            8'd62  : rcp_val = 24'd269543;
            8'd63  : rcp_val = 24'd265264;
            8'd64  : rcp_val = 24'd261120;
            8'd65  : rcp_val = 24'd257102;
            8'd66  : rcp_val = 24'd253207;
            8'd67  : rcp_val = 24'd249428;
            8'd68  : rcp_val = 24'd245760;
            8'd69  : rcp_val = 24'd242198;
            8'd70  : rcp_val = 24'd238738;
            8'd71  : rcp_val = 24'd235375;
            8'd72  : rcp_val = 24'd232106;
            8'd73  : rcp_val = 24'd228927;
            8'd74  : rcp_val = 24'd225833;
            8'd75  : rcp_val = 24'd222822;
            8'd76  : rcp_val = 24'd219890;
            8'd77  : rcp_val = 24'd217034;
            8'd78  : rcp_val = 24'd214252;
            8'd79  : rcp_val = 24'd211540;
            8'd80  : rcp_val = 24'd208896;
            8'd81  : rcp_val = 24'd206317;
            8'd82  : rcp_val = 24'd203800;
            8'd83  : rcp_val = 24'd201345;
            8'd84  : rcp_val = 24'd198948;
            8'd85  : rcp_val = 24'd196608;
            8'd86  : rcp_val = 24'd194321;
            8'd87  : rcp_val = 24'd192088;
            8'd88  : rcp_val = 24'd189905;
            8'd89  : rcp_val = 24'd187771;
            8'd90  : rcp_val = 24'd185685;
            8'd91  : rcp_val = 24'd183644;
            8'd92  : rcp_val = 24'd181648;
            8'd93  : rcp_val = 24'd179695;
            8'd94  : rcp_val = 24'd177783;
            8'd95  : rcp_val = 24'd175912;
            8'd96  : rcp_val = 24'd174080;
            8'd97  : rcp_val = 24'd172285;
            8'd98  : rcp_val = 24'd170527;
            8'd99  : rcp_val = 24'd168804;
            8'd100 : rcp_val = 24'd167116;
            8'd101 : rcp_val = 24'd165462;
            8'd102 : rcp_val = 24'd163840;
            8'd103 : rcp_val = 24'd162249;
            8'd104 : rcp_val = 24'd160689;
            8'd105 : rcp_val = 24'd159158;
            8'd106 : rcp_val = 24'd157657;
            8'd107 : rcp_val = 24'd156183;
            8'd108 : rcp_val = 24'd154737;
            8'd109 : rcp_val = 24'd153318;
            8'd110 : rcp_val = 24'd151924;
            8'd111 : rcp_val = 24'd150555;
            8'd112 : rcp_val = 24'd149211;
            8'd113 : rcp_val = 24'd147890;
            8'd114 : rcp_val = 24'd146593;
            8'd115 : rcp_val = 24'd145318;
            8'd116 : rcp_val = 24'd144066;
            8'd117 : rcp_val = 24'd142834;
            8'd118 : rcp_val = 24'd141624;
            8'd119 : rcp_val = 24'd140434;
            8'd120 : rcp_val = 24'd139264;
            8'd121 : rcp_val = 24'd138113;
            8'd122 : rcp_val = 24'd136980;
            8'd123 : rcp_val = 24'd135867;
            8'd124 : rcp_val = 24'd134771;
            8'd125 : rcp_val = 24'd133693;
            8'd126 : rcp_val = 24'd132632;
            8'd127 : rcp_val = 24'd131588;
            8'd128 : rcp_val = 24'd130560;
            8'd129 : rcp_val = 24'd129547;
            8'd130 : rcp_val = 24'd128551;
            8'd131 : rcp_val = 24'd127570;
            8'd132 : rcp_val = 24'd126603;
            8'd133 : rcp_val = 24'd125651;
            8'd134 : rcp_val = 24'd124714;
            8'd135 : rcp_val = 24'd123790;
            8'd136 : rcp_val = 24'd122880;
            8'd137 : rcp_val = 24'd121983;
            8'd138 : rcp_val = 24'd121099;
            8'd139 : rcp_val = 24'd120227;
            8'd140 : rcp_val = 24'd119369;
            8'd141 : rcp_val = 24'd118522;
            8'd142 : rcp_val = 24'd117687;
            8'd143 : rcp_val = 24'd116864;
            8'd144 : rcp_val = 24'd116053;
            8'd145 : rcp_val = 24'd115252;
            8'd146 : rcp_val = 24'd114463;
            8'd147 : rcp_val = 24'd113684;
            8'd148 : rcp_val = 24'd112916;
            8'd149 : rcp_val = 24'd112158;
            8'd150 : rcp_val = 24'd111411;
            8'd151 : rcp_val = 24'd110673;
            8'd152 : rcp_val = 24'd109945;
            8'd153 : rcp_val = 24'd109226;
            8'd154 : rcp_val = 24'd108517;
            8'd155 : rcp_val = 24'd107817;
            8'd156 : rcp_val = 24'd107126;
            8'd157 : rcp_val = 24'd106443;
            8'd158 : rcp_val = 24'd105770;
            8'd159 : rcp_val = 24'd105104;
            8'd160 : rcp_val = 24'd104448;
            8'd161 : rcp_val = 24'd103799;
            8'd162 : rcp_val = 24'd103158;
            8'd163 : rcp_val = 24'd102525;
            8'd164 : rcp_val = 24'd101900;
            8'd165 : rcp_val = 24'd101282;
            8'd166 : rcp_val = 24'd100672;
            8'd167 : rcp_val = 24'd100069;
            8'd168 : rcp_val = 24'd99474;
            8'd169 : rcp_val = 24'd98885;
            8'd170 : rcp_val = 24'd98304;
            8'd171 : rcp_val = 24'd97729;
            8'd172 : rcp_val = 24'd97160;
            8'd173 : rcp_val = 24'd96599;
            8'd174 : rcp_val = 24'd96044;
            8'd175 : rcp_val = 24'd95495;
            8'd176 : rcp_val = 24'd94952;
            8'd177 : rcp_val = 24'd94416;
            8'd178 : rcp_val = 24'd93885;
            8'd179 : rcp_val = 24'd93361;
            8'd180 : rcp_val = 24'd92842;
            8'd181 : rcp_val = 24'd92329;
            8'd182 : rcp_val = 24'd91822;
            8'd183 : rcp_val = 24'd91320;
            8'd184 : rcp_val = 24'd90824;
            8'd185 : rcp_val = 24'd90333;
            8'd186 : rcp_val = 24'd89847;
            8'd187 : rcp_val = 24'd89367;
            8'd188 : rcp_val = 24'd88891;
            8'd189 : rcp_val = 24'd88421;
            8'd190 : rcp_val = 24'd87956;
            8'd191 : rcp_val = 24'd87495;
            8'd192 : rcp_val = 24'd87040;
            8'd193 : rcp_val = 24'd86589;
            8'd194 : rcp_val = 24'd86142;
            8'd195 : rcp_val = 24'd85700;
            8'd196 : rcp_val = 24'd85263;
            8'd197 : rcp_val = 24'd84830;
            8'd198 : rcp_val = 24'd84402;
            8'd199 : rcp_val = 24'd83978;
            8'd200 : rcp_val = 24'd83558;
            8'd201 : rcp_val = 24'd83142;
            8'd202 : rcp_val = 24'd82731;
            8'd203 : rcp_val = 24'd82323;
            8'd204 : rcp_val = 24'd81920;
            8'd205 : rcp_val = 24'd81520;
            8'd206 : rcp_val = 24'd81124;
            8'd207 : rcp_val = 24'd80732;
            8'd208 : rcp_val = 24'd80344;
            8'd209 : rcp_val = 24'd79960;
            8'd210 : rcp_val = 24'd79579;
            8'd211 : rcp_val = 24'd79202;
            8'd212 : rcp_val = 24'd78828;
            8'd213 : rcp_val = 24'd78458;
            8'd214 : rcp_val = 24'd78091;
            8'd215 : rcp_val = 24'd77728;
            8'd216 : rcp_val = 24'd77368;
            8'd217 : rcp_val = 24'd77012;
            8'd218 : rcp_val = 24'd76659;
            8'd219 : rcp_val = 24'd76309;
            8'd220 : rcp_val = 24'd75962;
            8'd221 : rcp_val = 24'd75618;
            8'd222 : rcp_val = 24'd75277;
            8'd223 : rcp_val = 24'd74940;
            8'd224 : rcp_val = 24'd74605;
            8'd225 : rcp_val = 24'd74274;
            8'd226 : rcp_val = 24'd73945;
            8'd227 : rcp_val = 24'd73619;
            8'd228 : rcp_val = 24'd73296;
            8'd229 : rcp_val = 24'd72976;
            8'd230 : rcp_val = 24'd72659;
            8'd231 : rcp_val = 24'd72344;
            8'd232 : rcp_val = 24'd72033;
            8'd233 : rcp_val = 24'd71723;
            8'd234 : rcp_val = 24'd71417;
            8'd235 : rcp_val = 24'd71113;
            8'd236 : rcp_val = 24'd70812;
            8'd237 : rcp_val = 24'd70513;
            8'd238 : rcp_val = 24'd70217;
            8'd239 : rcp_val = 24'd69923;
            8'd240 : rcp_val = 24'd69632;
            8'd241 : rcp_val = 24'd69343;
            8'd242 : rcp_val = 24'd69056;
            8'd243 : rcp_val = 24'd68772;
            8'd244 : rcp_val = 24'd68490;
            8'd245 : rcp_val = 24'd68210;
            8'd246 : rcp_val = 24'd67933;
            8'd247 : rcp_val = 24'd67658;
            8'd248 : rcp_val = 24'd67385;
            8'd249 : rcp_val = 24'd67115;
            8'd250 : rcp_val = 24'd66846;
            8'd251 : rcp_val = 24'd66580;
            8'd252 : rcp_val = 24'd66316;
            8'd253 : rcp_val = 24'd66054;
            8'd254 : rcp_val = 24'd65794;
            8'd255 : rcp_val = 24'd65536;
            default: rcp_val = 24'd642756;
        endcase
    end

    // registered reciprocal — haze side only. rcp_q = rcp_val[23:6] is
    // floor(255*2^10/t): using it with a >>10 shift is equivalent to the old
    // (rcp_val * diff) >> 16 to within 1/4 LSB (the 6 discarded table LSBs
    // cost at most 255*63/65536 = 0.25 of an 8-bit code).
    // Registering here (instead of on the product) is what keeps the data path
    // combinational, and a 1-cycle lag of the t/rcp map cannot move a picture.
    // 260916 — 层 2：与 D_q 同理，只在 t/rcp_val 有效的周期推进，消隐期保持。
    reg [17:0] rcp_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)     rcp_q <= 18'd1024;      // == table entry for t = 255 (no defog)
        else if (de_Dq) rcp_q <= rcp_val[23:6];
    end

    //-----------------------------------------------------------------
    // 3e. 260916 — LINE-HEAD GUARD（层 3）：行首保护列的倒数
    //-----------------------------------------------------------------
    // 为什么光打门还不够：causal 5x5 的最小窗锚在像素之后（§3c），雾侧又天然滞后
    // 2~3 拍，所以行首第 0 列在这一拍**根本还没有**自己的雾估计；打门之后它沿用的
    // 是"上一行的最后一个有效值"，而那个值来自行尾 —— 本工程行尾是补黑
    // （hdmi_de_pinner 行尾填黑）或暗边缘 ⇒ D=0、t=255、rcp=1024（J==I）。
    // 于是左边缘仍然是一条没被增强的竖条，只是从 7 列收窄到 3 列。
    //
    // 解法：每行在行首区域（列 GUARD-1，此处流水早已进入真实值）采样一次 rcp_q
    //   存进 rcp_left；**下一行**的前 GUARD 列就用它。取到的正是"上一行行首那块
    //   真正有雾的区域"的雾估计 —— 与 §3c/§3d 的相位平移同理，雾是低频量，
    //   跨一行的平移在画面里不可见，而"没增强"是可见的。
    // 时序上天然成立：rcp_left 每行只写一次（行首区域），下一行的行首才读它，
    //   中间隔着 1200 多拍 + 一段消隐，不存在读写竞争。
    // 上电后第一行的前 GUARD 列 rcp_left 还是复位值 1024（保守 = 不增强），
    //   只影响复位后第一帧的左上角 8 个像素，第二行起即正常。
    // 代价：1 个 18bit 寄存器 + 1 个比较器 + 1 个 MUX；像素通路与几何均不受影响。
    //-----------------------------------------------------------------
    localparam [11:0] GUARD_LAST = GUARD - 12'd1;   // 采样点 = 行内第 GUARD 个像素
    reg [17:0] rcp_left;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                               rcp_left <= 18'd1024;
        else if (de_in && (px_cnt == GUARD_LAST)) rcp_left <= rcp_q;
    end

    // 真正送进合并式的倒数：行首保护列取上一行的采样值，其余取本拍 rcp_q
    wire [17:0] rcp_use = (px_cnt < GUARD) ? rcp_left : rcp_q;

    //-----------------------------------------------------------------
    // 261023b — 可选像素流水级（PIX_DLY），只给 125MHz 写侧用
    //-----------------------------------------------------------------
    // 违例实测（eth_write_clk = 8ns）：
    //   第 1 版 PIX_DLY=0
    //     px_cnt -> rcp_use MUX -> APM 9x18 -> 40bit 进位 -> clamp
    //     = 9.57ns，WNS -1.996 / TNS -25.48 / 16 端点
    //   第 2 版 PIX_DLY=1 + 24bit 加法器
    //     i_b_q -> 减 -> APM -> 24bit 进位 -> clamp -> FIFO DIA
    //     = 8.70ns，WNS -1.169 / TNS -13.60 / 16 端点
    //   只拆一级不够：乘法本身 2.53ns，前后再各带一段布线就吃掉半个周期。
    //   本版 PIX_DLY=2 在乘法输出再插一级：
    //     级A  i_*_q  -> 减 -> APM -> prod_q        ≈ 4.7ns
    //     级B  prod_q -> 进位 -> clamp -> FIFO DIA   ≈ 3.5ns
    //
    // 写侧无害：de_out 与 data_out 同步晚 PIX_DLY 拍，frame_fifo_write 按 de
    //   计数，收集到的像素序列逐个不变，帧长、帧相位、四象限对齐都不动。
    // 读侧（u_defog_sng，pix_clk_w 74.25MHz）必须保持 0：那里 hs/vs 不跟数据走，
    //   任何延迟都会变成画面右移。
    //-----------------------------------------------------------------
    reg [7:0]  i_r_q, i_g_q, i_b_q;
    reg        de_q;
    reg [17:0] rcp_use_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            i_r_q <= 8'd0; i_g_q <= 8'd0; i_b_q <= 8'd0;
            de_q  <= 1'b0;
            rcp_use_d <= 18'd1024;
        end else begin
            i_r_q <= r8;
            i_g_q <= g8;
            i_b_q <= b8;
            de_q  <= de_in;
            rcp_use_d <= rcp_use;
        end
    end

    // PIX_DLY = 0：本拍输入（原始零延迟行为；上面 5 个寄存器被综合器 DCE）
    // PIX_DLY >= 1：用延迟一级的副本
    wire [7:0]  i_r   = (PIX_DLY >= 1) ? i_r_q : r8;
    wire [7:0]  i_g   = (PIX_DLY >= 1) ? i_g_q : g8;
    wire [7:0]  i_b   = (PIX_DLY >= 1) ? i_b_q : b8;
    wire [17:0] rcp_f = (PIX_DLY >= 1) ? rcp_use_d : rcp_use;

    // bypass 直通要用"同一相位"的原始像素，否则 de 与 data 错位。
    // 重建逐位无损：r8[7:3]=data_in[15:11]、g8[7:2]=data_in[10:5]、b8[7:3]=data_in[4:0]。
    wire [15:0] dly1_565 = {i_r_q[7:3], i_g_q[7:2], i_b_q[7:3]};

    //=================================================================
    // 6. recover J = clamp((I - A) * rcp >> 10 + A, 0, 255)
    //    COMBINATIONAL on purpose (260915): I must be the pixel that arrives
    //    in THIS cycle — every register inserted between data_in and data_out
    //    becomes a 1-px right shift of the whole picture. Only rcp_q/a_prev
    //    are registered, and they feed the haze estimate, not the geometry.
    //    261023b: PIX_DLY>=1 时 i_* 来自上面那一级寄存器、PIX_DLY>=2 时 prod_*
    //    再晚一级，但「de_out 与 data_out 永远同拍」这一点不变（de_f 按同一档
    //    选相位），所以写侧帧内的像素排列一个都不动，只是整体推迟几个时钟。
    //=================================================================
    wire signed [8:0]  diff_r = {1'b0, i_r} - {1'b0, a_prev};
    wire signed [8:0]  diff_g = {1'b0, i_g} - {1'b0, a_prev};
    wire signed [8:0]  diff_b = {1'b0, i_b} - {1'b0, a_prev};

    // 260916: 乘数由 rcp_q 改成 rcp_use —— 只有行首前 GUARD 列不同（用上一行
    // 行首区域的实测值，见 §3e），其余列 rcp_use == rcp_q，逐位不变。
    // 位宽、移位量、clamp 全不动，仍是一级 DSP + 组合移位。
    wire signed [31:0] prod_r_w = diff_r * $signed({1'b0, rcp_f});  // 9 x 18, one DSP
    wire signed [31:0] prod_g_w = diff_g * $signed({1'b0, rcp_f});
    wire signed [31:0] prod_b_w = diff_b * $signed({1'b0, rcp_f});

    //-----------------------------------------------------------------
    // 261023b 级 B：乘法输出寄存（PIX_DLY >= 2）
    //   只保留 [31:10]（后面只用到这一截），22bit x 3 = 66 个寄存器，
    //   比整字寄存省 30 个。i_*_q2 只为 bypass 直通重建同相位的像素。
    //   级B 的组合段只剩 进位链 + clamp + 输出布线。
    //-----------------------------------------------------------------
    reg signed [21:0] prod_r_q, prod_g_q, prod_b_q;
    reg               de_q2;
    reg [7:0]         i_r_q2, i_g_q2, i_b_q2;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prod_r_q <= 22'sd0; prod_g_q <= 22'sd0; prod_b_q <= 22'sd0;
            de_q2    <= 1'b0;
            i_r_q2 <= 8'd0; i_g_q2 <= 8'd0; i_b_q2 <= 8'd0;
        end else begin
            prod_r_q <= prod_r_w[31:10];
            prod_g_q <= prod_g_w[31:10];
            prod_b_q <= prod_b_w[31:10];
            de_q2    <= de_q;
            i_r_q2 <= i_r;
            i_g_q2 <= i_g;
            i_b_q2 <= i_b;
        end
    end

    wire signed [21:0] prod_r_s = (PIX_DLY >= 2) ? prod_r_q : prod_r_w[31:10];
    wire signed [21:0] prod_g_s = (PIX_DLY >= 2) ? prod_g_q : prod_g_w[31:10];
    wire signed [21:0] prod_b_s = (PIX_DLY >= 2) ? prod_b_q : prod_b_w[31:10];

    // de_out 与 data_out 严格同相位：0 拍 / 1 拍 / 2 拍三档
    wire de_f = (PIX_DLY >= 2) ? de_q2 : ((PIX_DLY >= 1) ? de_q : de_in);

    // bypass 用的原始像素，同样按 0/1/2 拍选相位
    wire [15:0] dly2_565 = {i_r_q2[7:3], i_g_q2[7:2], i_b_q2[7:3]};
    wire [15:0] dly_565  = (PIX_DLY >= 2) ? dly2_565 :
                           ((PIX_DLY >= 1) ? dly1_565 : data_in);

    // 261023b — 加法器收窄：原来 clamp_8 直接吃 40bit 有符号数，(prod>>>10)+a_prev
    //   综合出一条 40 位进位链（实测 1.8ns）。实际取值范围：
    //     |diff| <= 255, rcp <= 5222 (t 被钳到 >= T0=50) -> |prod>>>10| <= 1300
    //     再加 a_prev(<=255) -> |sum| <= 1555，24 位绰绰有余。
    //   prod[31:10] 与 (prod >>> 10) 逐位等价（signed 算术右移取低位）。
    wire signed [23:0] sum_r = $signed(prod_r_s) + $signed({16'd0, a_prev});
    wire signed [23:0] sum_g = $signed(prod_g_s) + $signed({16'd0, a_prev});
    wire signed [23:0] sum_b = $signed(prod_b_s) + $signed({16'd0, a_prev});

    wire [7:0] j_r = clamp_8(sum_r);
    wire [7:0] j_g = clamp_8(sum_g);
    wire [7:0] j_b = clamp_8(sum_b);

    //=================================================================
    // 7. RGB888 -> RGB565, bypass mux, output
    //    PIX_DLY=0：bypass 逐位精确原地直通（data_out == data_in, de_out == de_in）
    //    PIX_DLY>=1：两者同步晚同样拍数，仍然严格对齐（dly_565 已按档选相位）
    //=================================================================
    wire [15:0] defog_565 = {j_r[7:3], j_g[7:2], j_b[7:3]};

    assign de_out   = de_f;
    assign data_out = bypass ? dly_565 : defog_565;

endmodule


//=====================================================================
// line_buffer : circular-RAM delay line
//   read-before-write (READ_FIRST) => dout is din delayed by exactly
//   DEPTH+1 "we"-pulses. Instantiate with DEPTH = (row length)-1 to get
//   an exact 1-line delay (e.g. DEPTH = H_ACT-1 for a full raster line).
//   dout is refreshed every clock, but the pointer only moves on we, so
//   the delay is counted in valid pixels, not in clock cycles.
//=====================================================================
module line_buffer #(
    parameter DEPTH = 640,
    parameter DW    = 8
)(
    input                clk,
    input                rst_n,
    input                we,
    input  [DW-1:0]      din,
    output reg [DW-1:0]  dout
);
    // ceil(log2) — constant-elaboration, no $clog2 dependency
    function integer clogb2;
        input integer value;
        integer i;
        begin
            clogb2 = 0;
            for (i = 0; i < 31; i = i + 1)
                if (value > (1 << i))
                    clogb2 = i + 1;
        end
    endfunction
    localparam AW = clogb2(DEPTH);
    reg [DW-1:0] mem [0:DEPTH-1];
    reg [AW-1:0] wp;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)   wp <= {AW{1'b0}};
        else if (we)  wp <= (wp == (DEPTH-1)) ? {AW{1'b0}} : (wp + 1'b1);
    end

    always @(posedge clk) begin
        if (we) mem[wp] <= din;
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)   dout <= {DW{1'b0}};
        else          dout <= mem[wp];   // read-before-write: value from DEPTH valid-cycles ago
    end
endmodule
