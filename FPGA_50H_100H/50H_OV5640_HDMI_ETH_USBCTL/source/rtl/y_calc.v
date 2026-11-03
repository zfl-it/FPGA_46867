`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// y_calc — RGB565 -> Y8 + 高亮失真/光晕掩码（模块 A 的 S0 级）  261028d
//
// 三级流水（与 luma_adapt 的节拍表对齐：输出在 beat3）：
//   s1: 展开 888 + 三个乘法（77/150/29）
//   s2: 三积相加（<= 58905，16 bit 足够）
//   s3: >>8 得 Y；同时给出 core
//
// 系数与官方 RGB2YCbCr.v 一致：Y = (77R + 150G + 29B) >> 8
// 8 bit 展开用位复制：{R,R[4:2]} / {G,G[5:4]} / {B,B[2:0]}
//
// ---- 掩码定义（261028d 改）--------------------------------------------------
//   core = 任一通道顶满 且 Y>=200   —— "已经高亮失真"
//   （261028f 删掉 halo：光晕复原改成按"比背景亮多少"连续取值，不再需要硬门限）
//
//   旧定义是 core = Y>=250，只认纯白。实测暖色过曝（如 255,240,180）Y≈237，
//   判不成饱和 → 夜景修复对它完全不起作用。现在按"通道是否顶满"判，
//   红尾灯那类低亮度饱和色（Y≈90）仍然被排除，不会误判。
//-----------------------------------------------------------------------------
//////////////////////////////////////////////////////////////////////////////////
module y_calc (
    input               clk,
    input               rst_n,
    input               de_in,
    input       [15:0]  data_in,      // RGB565
    output reg  [15:0]  px_o,         // 与 y_o/core_o/de_o 同拍（beat3）
    output reg  [7:0]   y_o,
    output reg          core_o,       // 通道顶满 & Y>=200：真正的高光失真
    output reg          de_o
);

    // ---------------- s1 ----------------
    reg [15:0] px1;
    reg [15:0] pr1, pg1, pb1;
    reg        de1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            px1 <= 16'd0; pr1 <= 16'd0; pg1 <= 16'd0; pb1 <= 16'd0; de1 <= 1'b0;
        end else begin
            px1 <= data_in;
            pr1 <= 16'd77  * {data_in[15:11], data_in[13:11]};
            pg1 <= 16'd150 * {data_in[10:5],  data_in[5:4]};
            pb1 <= 16'd29  * {data_in[4:0],   data_in[2:0]};
            de1 <= de_in;
        end
    end

    // ---------------- s2 ----------------
    reg [15:0] px2;
    reg [15:0] ysum2;
    reg        de2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            px2 <= 16'd0; ysum2 <= 16'd0; de2 <= 1'b0;
        end else begin
            px2   <= px1;
            ysum2 <= pr1 + pg1 + pb1;
            de2   <= de1;
        end
    end

    // ---------------- s3 ----------------
    wire [7:0] y_w = ysum2[15:8];

    // 任一通道到达满量程（R=31 / G=63 / B=31）—— 真正的"顶死"
    wire clip_w = (px2[15:11] == 5'd31) ||
                  (px2[10:5]  == 6'd63) ||
                  (px2[4:0]   == 5'd31);

    wire core_w = clip_w && (y_w >= 8'd200);        // 顶满 + 够亮

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            px_o <= 16'd0; y_o <= 8'd0; core_o <= 1'b0; de_o <= 1'b0;
        end else begin
            px_o   <= px2;
            y_o    <= y_w;
            core_o <= core_w;
            de_o   <= de2;
        end
    end

endmodule
