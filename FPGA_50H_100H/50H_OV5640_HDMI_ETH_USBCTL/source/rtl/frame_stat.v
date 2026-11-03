`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// frame_stat — 全局亮度统计（模块 A 的 S0 分析支路）        261031 提速版
//
// 只统计"非饱和像素"（sat=0）：远光灯核占 5% 就会把全帧均值抬高约 12 码，
//   足够让夜景被判成白天、增亮不启动，所以必须剔除。
//
// ---- 261031：统计推进单元 整帧 -> 1/4 帧，配合 4x4 抽取 ---------------------
//   ① 水平：每行只取 px_cnt[1:0]==0 的像素（1/4）
//   ② 垂直：每 4 行只取 ln_cnt[1:0]==0 的那一行（1/4）
//   ③ 窗口：QTR_LINES = V_ACT/4 = 180 行，一帧滚出 4 个均值
//
//   单窗口样本数 = (1280/4) x (180/4) = 320 x 45 = 14400（整帧 921600 的 1/64），
//   均值精度仍有 SE≈0.4 码，够用。
//
//   送出去判决的 mean_v 是**最近 4 个窗口的滚动和**（= 最近 1/4+3/4 帧，覆盖整帧），
//   不是"只用最后一个窗口"。原因：单个窗口只覆盖画面 180 行，若直接拿它判决，
//   上下亮度差大的场景（上亮天下暗地）会系统性偏亮/偏暗，而 gear_ctrl 的七段
//   阈值是按整帧均值标定的。滚动和既保住整帧语义，又让统计每 4.2 ms 推进一次。
//
//   时延的缩短来自"判决节拍"：旧版 mean_vld 每帧一次、gear_ctrl 每 4 次才判一次
//   → 8 帧 ≈ 133 ms；现在是每个窗口都有新均值，gear_ctrl 每帧判一次、连续两次
//   一致就换档 → 2 帧 ≈ 33 ms，正好 1/4。
//
//   窗口边界落在每帧第 180/360/540/720 行的行末（de 下降沿），第 4 个窗口结束时
//   正好是垂直消隐之前 —— 换档判决天然发生在帧边界，不会切出上下半帧不同曲线。
//
//   除法仍在窗口末的消隐里做：8 步移位减法，商 <= 255，不需要除法器 IP。
//
// 输出（都是 pix_clk 域）：
//   mean_v     最近 4 个窗口的非饱和像素 Y 均值（= 整帧覆盖），窗口末更新
//   mean_vld   单拍：mean_v 有效（每帧 4 次，每 4.2 ms 一次）
//   frame_vld  与 mean_vld 同拍，标记"这是帧内最后一个窗口"（gear_ctrl 用它换档）
//////////////////////////////////////////////////////////////////////////////////
module frame_stat #(
    parameter [10:0] H_ACT = 11'd1280,
    parameter [9:0]  V_ACT = 10'd720
)(
    input               clk,
    input               rst_n,
    input       [7:0]   y,
    input               sat,
    input               de,
    input               vs,

    output reg  [7:0]   mean_v,        // 非饱和像素的 Y 均值（滚动 4 窗口）
    output reg          mean_vld,      // 单拍：mean_v 有效
    output reg          frame_vld,     // 单拍：与 mean_vld 同拍，帧内最后一个窗口
    output reg  [19:0]  cnt_v_o,       // 滚动 4 窗口的有效（非饱和）样本数
    output reg  [19:0]  cnt_hi_o       // 滚动 4 窗口内 Y>=240 的样本数
);

    localparam integer QTR_LINES = V_ACT / 4;      // 720/4 = 180

    //------------------------------------------------------------------
    // 行内 / 窗口内的位置
    //------------------------------------------------------------------
    reg  [10:0] px_cnt;        // 行内像素序号 0..H_ACT-1
    reg  [9:0]  ln_cnt;        // 窗口内行号 0..QTR_LINES-1
    reg  [1:0]  win_idx;       // 窗口序号 0..3
    reg         de_d;

    wire        de_fall = de_d & ~de;
    wire        win_end = de_fall && (ln_cnt == (QTR_LINES - 1));
    wire        is_last = (win_idx == 2'd3);

    // 4x4 抽取：每 4 个像素取 1 个、每 4 行取 1 行
    wire        samp = de && (px_cnt[1:0] == 2'd0) && (ln_cnt[1:0] == 2'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            px_cnt <= 11'd0; ln_cnt <= 10'd0; win_idx <= 2'd0; de_d <= 1'b0;
        end else begin
            de_d <= de;
            if (vs) begin
                px_cnt  <= 11'd0;
                ln_cnt  <= 10'd0;
                win_idx <= 2'd0;
            end else begin
                if (de)
                    px_cnt <= (px_cnt == (H_ACT - 1)) ? 11'd0 : (px_cnt + 11'd1);

                if (de_fall) begin
                    px_cnt <= 11'd0;
                    if (win_end) begin
                        ln_cnt  <= 10'd0;
                        win_idx <= win_idx + 2'd1;      // 自动回绕，正好 0..3
                    end else begin
                        ln_cnt <= ln_cnt + 10'd1;
                    end
                end
            end
        end
    end

    //------------------------------------------------------------------
    // 累加（只算抽到的样本）
    //   sat=1 的像素不进 sumy（会被光斑抬高均值），但仍计入 cnt_hi ——
    //   压暗侧要的正是"顶满像素占比"，剔掉它就没法判过曝了。
    //------------------------------------------------------------------
    reg [27:0] sumy;
    reg [19:0] cntv, cnthi;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sumy <= 28'd0; cntv <= 20'd0; cnthi <= 20'd0;
        end else if (vs) begin
            sumy <= 28'd0; cntv <= 20'd0; cnthi <= 20'd0;
        end else if (win_end) begin
            sumy <= 28'd0; cntv <= 20'd0; cnthi <= 20'd0;
        end else if (samp) begin
            if (!sat) begin
                sumy <= sumy + {20'd0, y};
                cntv <= cntv + 20'd1;
            end
            if (y >= 8'd240)        cnthi <= cnthi + 20'd1;
        end
    end

    //------------------------------------------------------------------
    // 窗口末锁存：sumy_lat/cnt_lat 取"最近 4 个窗口"的滚动和
    //   h0/h1/h2 里放着前 3 个窗口的值；当前窗口还没进移位链，
    //   所以直接 sumy + h0 + h1 + h2 = 最近 4 个窗口（含刚结束的这个）。
    //   last_win 随快照一起锁存，8 拍后与 mean_vld 同拍输出成 frame_vld。
    //------------------------------------------------------------------
    reg [27:0] sumy_lat;
    reg [19:0] cnt_lat;
    reg        start_r, last_win;

    reg [23:0] sh0, sh1, sh2;      // 前 3 个窗口的有效 Y 累加和（<= 14400x255）
    reg [15:0] ch0, ch1, ch2;      // 前 3 个窗口的有效样本数
    reg [15:0] hi0, hi1, hi2;      // 前 3 个窗口的 Y>=240 样本数

    // 位宽：单窗口 sumy <= 14400x255 = 3.67e6（22 bit），4 窗口 <= 1.47e7（24 bit）
    wire [23:0] sum_all = {2'd0, sumy[21:0]} + sh0 + sh1 + sh2;
    wire [15:0] cnt_all = cntv[13:0] + ch0 + ch1 + ch2;
    wire [15:0] hi_all  = cnthi[13:0] + hi0 + hi1 + hi2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sumy_lat <= 28'd0;
            cnt_lat  <= 20'd0;
            start_r  <= 1'b0;
            last_win <= 1'b0;
            cnt_v_o  <= 20'd0;
            cnt_hi_o <= 20'd0;
            sh0 <= 24'd0; sh1 <= 24'd0; sh2 <= 24'd0;
            ch0 <= 16'd0; ch1 <= 16'd0; ch2 <= 16'd0;
            hi0 <= 16'd0; hi1 <= 16'd0; hi2 <= 16'd0;
        end else begin
            start_r <= win_end;
            if (win_end) begin
                sumy_lat <= {4'd0, sum_all};
                cnt_lat  <= {4'd0, cnt_all};
                cnt_v_o  <= {4'd0, cnt_all};
                cnt_hi_o <= {4'd0, hi_all};
                last_win <= is_last;
                // 移位链：把刚结束的窗口推进历史
                sh2 <= sh1; sh1 <= sh0; sh0 <= {2'd0, sumy[21:0]};
                ch2 <= ch1; ch1 <= ch0; ch0 <= {2'd0, cntv[13:0]};
                hi2 <= hi1; hi1 <= hi0; hi0 <= {2'd0, cnthi[13:0]};
            end
        end
    end

    //------------------------------------------------------------------
    // 8 步移位减法除法：mean = sumy_lat / cnt_lat（商 <= 255）
    //   bit 从 7 递减到 0，每拍试减一次 cnt_lat << bit
    //------------------------------------------------------------------
    wire start = start_r;

    reg [27:0] rem;
    reg [7:0]  quot;
    reg [3:0]  bit;
    reg        busy;
    reg        last_q;

    wire [27:0] den    = {8'd0, cnt_lat} << bit;
    wire        zero_d = (cnt_lat == 20'd0);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rem <= 28'd0; quot <= 8'd0; bit <= 4'd0; busy <= 1'b0;
            mean_v <= 8'd0; mean_vld <= 1'b0; frame_vld <= 1'b0; last_q <= 1'b0;
        end else begin
            mean_vld  <= 1'b0;
            frame_vld <= 1'b0;
            if (start) begin
                rem    <= sumy_lat;
                quot   <= 8'd0;
                bit    <= 4'd7;
                busy   <= 1'b1;
                last_q <= last_win;
            end else if (busy) begin
                if (bit == 4'd0) begin
                    // 最后一位要算进商里（否则 mean_v 少 1）
                    busy      <= 1'b0;
                    mean_v    <= zero_d ? 8'd0
                                        : (quot | ((rem >= den) ? 8'd1 : 8'd0));
                    mean_vld  <= 1'b1;
                    frame_vld <= last_q;
                end else begin
                    if (!zero_d && (rem >= den)) begin
                        rem       <= rem - den;
                        quot[bit] <= 1'b1;
                    end
                    bit <= bit - 4'd1;
                end
            end
        end
    end

endmodule
