`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// marker_row_detect — 标识行识别（100H 接收侧）
//
// 与 50H 发送侧的 marker_row_add 配对使用：
//   50H 在显示链最末端把每帧第 0 行整行替换成纯色标识
//     mode = 1 → 整行 0xFFFF（白，拼接 / 四宫格帧）
//     mode = 0 → 整行 0x0000（黑，单路帧）
//   100H 这边收 HDMI → DDR 帧缓存 → 本模块在**读侧**识别该行
//   → 单比特电平 → PCIe 域两级同步 → pio_crtl 状态寄存器 → RK3568 读 BAR0
//
// RK3568 侧寄存器（pio_crtl.v，只读，与既有 WR_FRAME_DONE = 0x140 同一个 BAR0
//   PIO 窗口、地址按字节偏移）：
//     +0x144  MARKER_STATUS
//       [0]      marker_mode   1 = 白（拼接 / 四宫格帧）  0 = 黑（单路帧）
//       [1]      marker_ok     1 = 最近一帧标识行识别成功
//       [2]      marker_valid  1 = 上电以来至少成功识别过一次
//       [3]      marker_new    1 = 有过新判决；**读本寄存器即清零**
//       [15:4]   frame_cnt     成功判决次数低 12 位（判断新鲜度用）
//       [30:16]  保留 0
//       [31]     恒 1（驱动探测该寄存器是否存在）
//   采样诊断不用过寄存器：RK 直接看 DMA 帧的第 0 行即可。
//
// 为什么挂在读侧（fb_vout_*）而不是写侧（hdmi_rgb565）：
//   读侧就是 pcie_tx_fun 打包发走的那一路，识别结果与 RK3568 实际收到的像素
//   **必然一致**。若挂在写侧，一旦帧缓存的写/读通路吞掉首行，就会出现
//   "FPGA 报有标识、RK 却收不到"的假一致。现在若 marker_ok 长期为 0，就说明
//   RK 拿到的那一帧首行不是干净纯色，可直接把怀疑范围收到帧缓存链路上。
//
// 帧首检测：与发送侧同一套模型，**刻意不用 vs**（这个位置上 vs 的极性会随
//   选中的源而变，用 vs 沿需要先知道极性，容易埋雷）。改用 de 间隔检测：
//   行/列计数器由 de 驱动；某个 de 上升沿之前的空档只要长于"一整行有效像素"，
//   就只可能是场消隐 ⇒ 那就是新一帧的第 0 行。720p60 行消隐约 370 clk、
//   场消隐 ≈ 30 行（≈49500 clk），阈值取 1281 clk，判别余量一个数量级以上，
//   而且计数器最多一帧就能自对齐。
//
// 判决规则（抗噪 + 抗 HDMI 色彩范围差异）：
//   统计整行"够亮"与"够暗"的像素个数，任一超过 3/4 总像素即判定：
//     亮：R5 >= BRIGHT_R 且 G6 >= BRIGHT_G 且 B5 >= BRIGHT_B
//     暗：R5 <= DARK_R    且 G6 <= DARK_G    且 B5 <= DARK_B
//   阈值刻意取得宽松（亮 ≈ 75% 满量程、暗 ≈ 25%），因此 HDMI 收发器即使走
//   limited range（16~235）也能正确分类；3/4 多数则容忍边缘少数坏像素。
//   既不够亮也不够暗 ⇒ marker_ok = 0（无标识 / 被去雾等算法改写），此时
//   mode **保持不变**：沿用上一次成功判决，比让模式位跟着噪声乱跳安全。
//
// 输出全部是**单比特电平**，每次判决才变化一次：跨到 PCIe 时钟域每根只需要
//   一级两级同步器，不存在多比特一致性风险 —— 这正是本模块不吐 16bit 采样值
//   的原因。要看原始采样值，RK 侧直接读 DMA 帧的第 0 行即可，不必再过寄存器。
//
// 只读观察者：本模块不参与数据通路，不影响 de / vs / 数据的任何时序，
//   也不产生任何新的时钟或复位。
//////////////////////////////////////////////////////////////////////////////////

module marker_row_detect #(
    parameter [10:0] H_ACT    = 11'd1280,   // 每行有效像素数（720p）
    parameter [9:0]  V_ACT    = 10'd720,    // 每帧有效行数（仅用于行计数保护）
    parameter [4:0]  BRIGHT_R = 5'd24,      // 亮判据 ≈ 75% 满量程
    parameter [5:0]  BRIGHT_G = 6'd48,
    parameter [4:0]  BRIGHT_B = 5'd24,
    parameter [4:0]  DARK_R   = 5'd8,       // 暗判据 ≈ 25% 满量程
    parameter [5:0]  DARK_G   = 6'd16,
    parameter [4:0]  DARK_B   = 5'd8
)(
    input  wire        clk,             // 像素时钟（读侧 pix_clk_1080p）
    input  wire        rst_n,
    input  wire [15:0] data_in,         // RGB565，读侧 fb_vout_data
    input  wire        de_in,           // 读侧 fb_vout_de
    output reg         o_mode,          // 1 = 白（拼接）/ 0 = 黑（单路）
    output reg         o_marker_ok,     // 1 = 最近一帧标识行识别成功
    output reg         o_valid,         // 电平：上电以来至少成功识别过一次
    output reg         o_update_tgl     // 每次成功识别翻转一次（给对端计数）
);

    // 略大于"一整行有效像素"的 de 空档即判为场消隐
    localparam [16:0] GAP_MIN = {6'b0, H_ACT} + 17'd1;
    // 3/4 总像素（1280 → 960），多数判决门槛
    localparam [11:0] MAJOR   = (H_ACT >> 2) * 3;

    //-----------------------------------------------------------------
    // 输入打一拍：把 6 路比较的组合路径与后面的计数器隔开，同时保证
    // data/de 严格同拍（两路一起寄存，对齐关系不变）。
    //-----------------------------------------------------------------
    reg [15:0] data_d;
    reg        de_dd;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            data_d <= 16'd0;
            de_dd  <= 1'b0;
        end else begin
            data_d <= data_in;
            de_dd  <= de_in;
        end
    end

    wire bright = (data_d[15:11] >= BRIGHT_R) &&
                  (data_d[10: 5] >= BRIGHT_G) &&
                  (data_d[ 4: 0] >= BRIGHT_B);
    wire dark   = (data_d[15:11] <= DARK_R) &&
                  (data_d[10: 5] <= DARK_G) &&
                  (data_d[ 4: 0] <= DARK_B);

    //-----------------------------------------------------------------
    // de 间隔 → 帧首；行 / 列计数
    //-----------------------------------------------------------------
    reg  [16:0] gap_cnt;     // 距上一次 de 有效的时钟数（饱和，不回卷）
    reg  [10:0] px_cnt;      // 行内像素计数 0..H_ACT-1
    reg  [9:0]  ln_cnt;      // 帧内行计数 0..V_ACT-1
    reg         de_d;

    wire new_frame = de_dd & ~de_d & (gap_cnt > GAP_MIN);
    // 行 0：new_frame 当前拍即为本帧第一个有效像素（此时 ln_cnt 还没来得及被
    // 清零，所以必须用 new_frame 补上），之后整行 ln_cnt 恒为 0 直到列计数满。
    wire row0      = new_frame | (ln_cnt == 10'd0);
    wire line_end  = ~de_dd & de_d;   // 上一行的最后一个像素已经计入

    reg  [11:0] white_cnt;   // 本行"够亮"像素数
    reg  [11:0] black_cnt;   // 本行"够暗"像素数
    reg         was_row0;    // 刚结束/正在进行的是否为第 0 行

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de_d         <= 1'b0;
            gap_cnt      <= 17'd0;
            px_cnt       <= 11'd0;
            ln_cnt       <= 10'd0;
            white_cnt    <= 12'd0;
            black_cnt    <= 12'd0;
            was_row0     <= 1'b0;
            o_mode       <= 1'b0;
            o_marker_ok  <= 1'b0;
            o_valid      <= 1'b0;
            o_update_tgl <= 1'b0;
        end else begin
            de_d <= de_dd;

            // de 空档计数：饱和，不回卷（避免长消隐期把计数绕回小值）
            if (de_dd)
                gap_cnt <= 17'd0;
            else if (gap_cnt != {17{1'b1}})
                gap_cnt <= gap_cnt + 17'd1;

            // 行 / 列计数（px_cnt 必须在满 H_ACT 时回卷，否则 ln_cnt 永不动）
            if (new_frame) begin
                px_cnt <= 11'd0;
                ln_cnt <= 10'd0;
            end else if (de_dd) begin
                if (px_cnt == (H_ACT - 11'd1)) begin
                    px_cnt <= 11'd0;
                    // 到 V_ACT 就停在最后一行：若在这一拍回卷成 0，遇到行长异常
                    // 的坏信号会造出一个假"第 0 行"。真正的新帧由 new_frame 复位。
                    ln_cnt <= (ln_cnt == (V_ACT - 10'd1)) ? ln_cnt : (ln_cnt + 10'd1);
                end else begin
                    px_cnt <= px_cnt + 11'd1;
                end
            end

            if (line_end) begin
                // 刚结束的那一行若是第 0 行 → 出判决
                if (was_row0) begin
                    if (white_cnt > MAJOR) begin
                        o_mode       <= 1'b1;   // 白 → 拼接
                        o_marker_ok  <= 1'b1;
                        o_valid      <= 1'b1;
                        o_update_tgl <= ~o_update_tgl;
                    end else if (black_cnt > MAJOR) begin
                        o_mode       <= 1'b0;   // 黑 → 单路
                        o_marker_ok  <= 1'b1;
                        o_valid      <= 1'b1;
                        o_update_tgl <= ~o_update_tgl;
                    end else begin
                        // 既非纯白也非纯黑：无标识 / 被算法污染
                        // mode / valid / tgl 全部保持，不制造假更新
                        o_marker_ok  <= 1'b0;
                    end
                end
                white_cnt <= 12'd0;
                black_cnt <= 12'd0;
                was_row0  <= 1'b0;
            end else if (de_dd && row0) begin
                was_row0 <= 1'b1;
                if (bright) white_cnt <= white_cnt + 12'd1;
                if (dark)   black_cnt <= black_cnt + 12'd1;
            end
        end
    end

endmodule

`default_nettype wire
