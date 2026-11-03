`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// marker_row_add — 行 0 标识行注入（V4 方案第 3 步 / 升级方案 §11.3）
//
// 在显示链的最末端，把每一帧的**第 0 行整行替换**成纯色标识行：
//   u_vsel → image_enhance → defog_rgb → 【marker_row_add】→ RGB565→888 → HDMI
//
// 为什么必须放在这里（260916 定稿的硬约束）：
//   image_enhance 与 defog_rgb 都会改写接近纯白 / 纯黑的像素 —— 标识行若在它们
//   之前注入，会被算法"吃掉"，到 100H 那边就不再是干净的 0xFFFF / 0x0000。
//   放在 defog 之后，后面只剩补位的 RGB565→888（逐位复制，不改灰度关系）。
//
// 标识格式（自 260916 冻结，100H 的下发端与 flag_detect 依赖它，不要改）：
//   mode_ctrl = 1 → 整行 0xFFFF（白，拼接 / 四宫格帧）
//   mode_ctrl = 0 → 整行 0x0000（黑，单路帧）
// mode_ctrl **直连** video_select 的 o_ctrl（选择器内部在 color_vs 帧首沿锁存产出的
// 模式位）：不译码、不再加同步器 —— 多一级就会让它比画面晚，标识行颜色会错一帧
// （方案 §4.2.4 / §10-D7）。
//
// 几何：**零附加延迟**。o_de = de、o_vs = vs，o_data 是 data 上的纯组合 MUX，
//   像素通路上没有任何寄存器 ⇒ 画面不可能位移。行 0 是"就地替换"而不是"插一行"，
//   总行数、vs/de 的时序一律不变，所以下游的 de 计数 / flag_detect 都不受影响。
//
// 帧首检测：**刻意不用 vs**。到这个位置 vs 的极性取决于当前选中的是哪一股流
//   （摄像头 vs / HDMI vs / DDR 读出），用 vs 沿复位需要知道极性，容易埋雷。
//   改用 de 间隔检测，与极性无关：行/列计数器由 de 驱动；某个 de 上升沿之前的
//   空档只要长于"一整行有效像素"，就只可能是场消隐 ⇒ 那就是新一帧的第一行。
//   720p60 的行消隐约 370 clk、场消隐约 30 行（≈49500 clk），而阈值取 1281 clk，
//   两者相差一个数量级以上，判别很宽；并且计数器最多一帧就能自对齐。
//
// 回退：把 bypass 接 1 即整体关闭标识行（逐位直通，纯组合），不必改任何接线。
//////////////////////////////////////////////////////////////////////////////////

module marker_row_add #(
    parameter [10:0] H_ACT = 11'd1280,      // 每行有效像素数（720p）
    parameter [9:0]  V_ACT = 10'd720,       // 每帧有效行数（仅用于计数器回卷保护）
    parameter [15:0] WHITE = 16'hFFFF,      // 拼接帧标识色
    parameter [15:0] BLACK = 16'h0000       // 单路帧标识色
)(
    input                   clk,
    input                   rst_n,
    input                   mode_ctrl,      // 直连 video_select.o_ctrl：1=拼接 / 0=单路
    input                   bypass,         // 1 = 逐位直通（回退点）
    input                   vs,
    input                   de,
    input   [15:0]          data,
    output                  o_vs,           // == vs（本模块零延迟）
    output                  o_de,           // == de
    output  reg [15:0]      o_data
);

    //=================================================================
    // de 间隔 → 帧首检测
    //=================================================================
    localparam [16:0] GAP_MIN = {6'b0, H_ACT} + 17'd1;   // 略大于一整行有效像素

    reg  [16:0] gap_cnt;        // 距上一次 de 有效的时钟数（饱和，不回卷）
    reg  [10:0] px_cnt;         // 行内像素计数 0..H_ACT-1
    reg  [9:0]  ln_cnt;         // 帧内行计数 0..V_ACT-1
    reg         de_d;

    wire new_frame = de & ~de_d & (gap_cnt > GAP_MIN);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de_d    <= 1'b0;
            gap_cnt <= 17'd0;
            px_cnt  <= 11'd0;
            ln_cnt  <= 10'd0;
        end else begin
            de_d <= de;

            if (de)
                gap_cnt <= 17'd0;
            else if (gap_cnt != {17{1'b1}})
                gap_cnt <= gap_cnt + 17'd1;

            if (new_frame) begin
                px_cnt <= 11'd0;
                ln_cnt <= 10'd0;
            end else if (de) begin
                if (px_cnt == (H_ACT - 11'd1)) begin
                    px_cnt <= 11'd0;
                    ln_cnt <= (ln_cnt == (V_ACT - 10'd1)) ? 10'd0 : (ln_cnt + 10'd1);
                end else begin
                    px_cnt <= px_cnt + 11'd1;
                end
            end
        end
    end

    // 行 0：new_frame 当前拍即为本帧第一个有效像素（此时 ln_cnt 尚未更新，必须
    // 用 new_frame 补上），之后整行 ln_cnt 恒为 0 直到列计数满。
    wire row0 = new_frame | (ln_cnt == 10'd0);

    //=================================================================
    // 输出：时序全直通，只就地替换行 0 的像素值
    //=================================================================
    assign o_vs = vs;
    assign o_de = de;

    always @(*) begin
        if (!bypass && de && row0) o_data = mode_ctrl ? WHITE : BLACK;
        else                       o_data = data;
    end

endmodule
