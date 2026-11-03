`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// luma_adapt — 模块 A：单路亮度自适应链路（替代 image_enhance 的 u_enhance_sng）
//
// 节拍表（beat = de/data_in 之后的第几拍，LAT = 15）：
//   beat0  data_in
//   beat3  y_calc 输出（px3 / y3 / core3 / de3）   ← 全局亮度统计从这里抽头
//   beat4..12  9 拍对齐（像素 / Y 占位）
//   beat13 像素延迟到此处（dly_px），与旧版夜景级输出同拍
//   beat14  色调表地址寄存器
//   beat15  色调表输出（= 全局增益后的像素）  ← 输出
//
// ---- 261031：夜景修复（night_repair）整级删除 ---------------------------------
//   该级原来是 beat3 -> beat13 的 10 拍流水（遮光复原 7 级 + 曲线 3 级），
//   现在换成 dly_px 的 10 级纯延迟 —— LAT 保持 15 不变，下游（defog 的
//   DEFOG_DLY / hs_sr）一行都不用改，几何零漂移。
//   随之退役的还有只服务于夜景的三条支路：hl_stat / block_stat / frame_stat 的
//   ref、blob、bbox 接口；统计只留"全局亮度"一条。
//
// 硬规则：
//   1) 旁路 = 查 bank0（identity），不是数据直通 —— 任何状态下延迟恒为 15，
//      hs/vs 只需一套同深移位链，几何不可能因开关而漂移。
//   2) 统计通路常开（不受开关影响），否则重开后的头几帧会用陈旧数据。
//   3) 拼接模式 ctrl=1 整体旁路（写侧四路已各自处理）。
//////////////////////////////////////////////////////////////////////////////////
module luma_adapt #(
    parameter [10:0] H_ACT = 11'd1280,
    parameter [9:0]  V_ACT = 10'd720,
    parameter        LAT   = 15
)(
    input               clk,
    input               rst_n,
    input               hs_in,
    input               vs_in,
    input               de_in,
    input       [15:0]  data_in,
    input               ctrl,          // 1 = 拼接：整体旁路
    input               l_en,          // 亮度自适应开关（增亮+增暗，帧边界锁存）

    output              hs_out,
    output              vs_out,
    output              de_out,
    output      [15:0]  data_out,
    output      [1:0]   gear_o         // 调试用：当前档位
);

    //==================================================================
    // beat3：Y / 掩码
    //==================================================================
    wire [15:0] px3;
    wire [7:0]  y3;
    wire        core3, de3;

    y_calc u_y_calc (
        .clk     (clk),
        .rst_n   (rst_n),
        .de_in   (de_in),
        .data_in (data_in),
        .px_o    (px3),
        .y_o     (y3),
        .core_o  (core3),
        .de_o    (de3)
    );

    //==================================================================
    // de / hs / vs 同深移位链（LAT = 15，任何开关状态下恒定）
    //==================================================================
    reg [14:0] de_s, hs_s, vs_s;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            de_s <= 15'd0; hs_s <= 15'd0; vs_s <= 15'd0;
        end else begin
            de_s <= {de_s[13:0], de_in};
            hs_s <= {hs_s[13:0], hs_in};
            vs_s <= {vs_s[13:0], vs_in};
        end
    end

    //==================================================================
    // beat3 -> beat13：10 级纯像素延迟（原夜景级的流水深度，保持 LAT=15）
    //   用扁平移位寄存器而不是数组：Synplify 一定推成 160 个 FF，
    //   不会因为索引写法被误推成 RAM。
    //==================================================================
    reg [159:0] px_sr;          // 10 级 x 16bit

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) px_sr <= 160'd0;
        else        px_sr <= {px_sr[143:0], px3};
    end

    wire [15:0] rp_px = px_sr[159:144];

    //==================================================================
    // 全局亮度统计（beat3 抽头，只读；窗口 = 1/4 帧，4x4 抽取）
    //==================================================================
    wire [7:0]  mean_v;
    wire        mean_vld, frame_vld;
    wire [19:0] cnt_v, cnt_hi;

    frame_stat #(
        .H_ACT (H_ACT),
        .V_ACT (V_ACT)
    ) u_fstat (
        .clk       (clk),
        .rst_n     (rst_n),
        .y         (y3),
        .sat       (core3),
        .de        (de3),
        .vs        (vs_in),
        .mean_v    (mean_v),
        .mean_vld  (mean_vld),
        .frame_vld (frame_vld),
        .cnt_v_o   (cnt_v),
        .cnt_hi_o  (cnt_hi)
    );

    //==================================================================
    // 判定 → bank
    //==================================================================
    wire [2:0] bank_dec;
    wire [1:0] gear_mon;               // gear_ctrl 的档位输出，仅供调试

    gear_ctrl u_gear (
        .clk       (clk),
        .rst_n     (rst_n),
        .mean_vld  (mean_vld),
        .frame_vld (frame_vld),
        .mean_v    (mean_v),
        .cnt_v     (cnt_v),
        .cnt_hi    (cnt_hi),          // 压暗侧用它算 dark_score（顶满比例）
        .l_en      (l_en),
        .bank      (bank_dec),
        .gear      (gear_mon)          // 仅供调试，不引出
    );

    wire [2:0] bank = ctrl ? 3'd0 : bank_dec;

    // gear_o：支内序号（提亮 bank1..4 -> 0..3 / 压暗 bank5..7 -> 0..2），直通给 0
    assign gear_o = (bank == 3'd0) ? 2'd0 :
                    (bank <= 3'd4) ? (bank - 3'd1) :
                                     (bank - 3'd5);

    //==================================================================
    // beat13..15：色调表（全局亮度自适应）
    //==================================================================
    reg [4:0] ar;
    reg [5:0] ag;
    reg [4:0] ab;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ar <= 5'd0; ag <= 6'd0; ab <= 5'd0;
        end else begin
            ar <= rp_px[15:11];
            ag <= rp_px[10:5];
            ab <= rp_px[4:0];
        end
    end

    wire [4:0] lr;
    wire [5:0] lg;
    wire [4:0] lb;

    tone_lut u_tl (
        .bank  (bank),
        .r_in  (ar),
        .g_in  (ag),
        .b_in  (ab),
        .r_out (lr),
        .g_out (lg),
        .b_out (lb)
    );

    reg [15:0] px15;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) px15 <= 16'd0;
        else        px15 <= {lr, lg, lb};
    end

    assign data_out = px15;
    assign hs_out   = hs_s[14];
    assign vs_out   = vs_s[14];
    assign de_out   = de_s[14];

endmodule
