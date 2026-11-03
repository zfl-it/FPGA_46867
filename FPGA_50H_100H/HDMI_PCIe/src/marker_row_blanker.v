`timescale 1ns / 1ps
`default_nettype none

//////////////////////////////////////////////////////////////////////////////////
// marker_row_blanker — 行 0 统一置黑（V4 方案新增，2026-10-11）
//
// 需求：50H 发来的行 0 是模式标识行（拼接 = 0xFFFF 白 / 单路 = 0x0000 黑）。
// 100H 在 marker_row_detect 中识别出模式后，把发给 RK3568 的 DMA 流里的
// 行 0 **统一替换成黑色**，使标识行不再出现在屏幕画面上；模式信息改由
// BAR0 0x144 MARKER_STATUS 寄存器（marker_row_detect 输出）继续提供给 RK。
//
// 数据通路位置（hdmi_loop.v）：
//   fb_vout_data/de/vs ──┬─> marker_row_detect（纯观察，看**原始**流，
//                        │   若先置黑它就永远只识别出"黑"了）
//                        └─> marker_row_blanker（本模块）──> pcie_tx_fun
//
// 行为：
//   * blank_en = 1 时，行 0 的有效像素一律输出 BLANK_COLOR（默认 0x0000）；
//     blank_en = 0 时逐位直通。顶层接 marker_row_detect 的 o_valid
//     （"上电以来至少成功识别过一次"）：识别成功前的第一帧行 0 原样
//     透传，从第二帧起统一置黑——避免把"未注入标识的真实画面首行"
//     误删（例如 50H 端 bypass 的情形）。
//   * 单路模式行 0 本来就是黑，置黑前后无差别；只有拼接模式的白行
//     会被变黑，屏幕上不再出现白色横线。
//
// 时序安全性（本模块的唯一时序考量）：
//   * 全部逻辑位于 pix_clk_1080p（74.25 MHz，周期 13.468 ns）单一时钟域，
//     **不引入任何新时钟、不产生任何跨时钟域路径**；blank_en 来自同域的
//     marker_row_detect 寄存器输出，复位复用 ddr_init_done_pix。
//   * 唯一新增组合路径：in_data(寄存器输出) → 16bit 2:1 MUX → out_data
//     寄存器 D 端；row0 判定最深为 17bit 比较 + 3 输入与门。该域当前
//     WNS ≈ +9 ns，MUX 增量 ≈ 1~1.5 ns，余量仍 > 7 ns。
//   * out_data / out_de / out_vs **同拍寄存**（一起 +1 cycle），数据与
//     时序的对齐关系保持不变；pcie_tx_fun 在 vs 上升沿启动捕获、
//     行 0 前约 25 行，1 拍的整体延迟无任何影响。
//
// 帧首检测：与 50H marker_row_add / 100H marker_row_detect 完全同款——
//   刻意不用 vs（极性随源变化），改用 de 间隔：空档 > 一整行有效像素
//   （GAP_MIN = H_ACT+1）即判为场消隐 ⇒ 帧首。720p60 行消隐 ≈ 370 clk、
//   场消隐 ≈ 49500 clk，阈值 1281 clk，判别余量一个数量级以上。
//
// 回退：把顶层 blank_en 接 0（或移除本模块、恢复直连）即整体关闭置黑。
//////////////////////////////////////////////////////////////////////////////////

module marker_row_blanker #(
    parameter [10:0] H_ACT       = 11'd1280,   // 每行有效像素数（720p）
    parameter [9:0]  V_ACT       = 10'd720,    // 每帧有效行数（计数器回卷保护）
    parameter [15:0] BLANK_COLOR = 16'h0000    // 行 0 统一替换色（黑）
)(
    input  wire        clk,           // 像素时钟（读侧 pix_clk_1080p）
    input  wire        rst_n,
    input  wire        blank_en,      // 1 = 置黑行 0（顶层接 marker_row_detect.o_valid）
    input  wire [15:0] in_data,       // RGB565，来自 fb_vout_data
    input  wire        in_de,         // 来自 fb_vout_de
    input  wire        in_vs,         // 来自 rd_vs
    output reg  [15:0] out_data,      // → pcie_tx_fun.i_video_data
    output reg         out_de,        // → pcie_tx_fun.i_video_de
    output reg         out_vs         // → pcie_tx_fun.i_video_vs
);

    // 略大于"一整行有效像素"的 de 空档即判为场消隐
    localparam [16:0] GAP_MIN = {6'b0, H_ACT} + 17'd1;

    //-----------------------------------------------------------------
    // de 间隔 → 帧首检测；行 / 列计数（与 marker_row_detect 同款）
    //-----------------------------------------------------------------
    reg  [16:0] gap_cnt;     // 距上一次 de 有效的时钟数（饱和，不回卷）
    reg  [10:0] px_cnt;      // 行内像素计数 0..H_ACT-1
    reg  [9:0]  ln_cnt;      // 帧内行计数 0..V_ACT-1
    reg         de_d;

    wire new_frame = in_de & ~de_d & (gap_cnt > GAP_MIN);
    // 行 0：new_frame 当前拍即为本帧第一个有效像素（ln_cnt 还没被清零，
    // 必须用 new_frame 补上），之后整行 ln_cnt 恒为 0 直到列计数满。
    wire row0      = new_frame | (ln_cnt == 10'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de_d     <= 1'b0;
            gap_cnt  <= 17'd0;
            px_cnt   <= 11'd0;
            ln_cnt   <= 10'd0;
            out_data <= 16'd0;
            out_de   <= 1'b0;
            out_vs   <= 1'b0;
        end else begin
            de_d <= in_de;

            // de 空档计数：饱和，不回卷（避免长消隐期把计数绕回小值）
            if (in_de)
                gap_cnt <= 17'd0;
            else if (gap_cnt != {17{1'b1}})
                gap_cnt <= gap_cnt + 17'd1;

            // 行 / 列计数（px_cnt 满行回卷；到 V_ACT 停住等 new_frame 复位，
            // 与 marker_row_detect 同一套防呆，避免坏信号造出假"第 0 行"）
            if (new_frame) begin
                px_cnt <= 11'd0;
                ln_cnt <= 10'd0;
            end else if (in_de) begin
                if (px_cnt == (H_ACT - 11'd1)) begin
                    px_cnt <= 11'd0;
                    ln_cnt <= (ln_cnt == (V_ACT - 10'd1)) ? ln_cnt
                                                          : (ln_cnt + 10'd1);
                end else begin
                    px_cnt <= px_cnt + 11'd1;
                end
            end

            // 输出：data / de / vs 同拍寄存（对齐不变），行 0 就地替换
            out_de   <= in_de;
            out_vs   <= in_vs;
            out_data <= (blank_en && row0 && in_de) ? BLANK_COLOR : in_data;
        end
    end

endmodule

`default_nettype wire
