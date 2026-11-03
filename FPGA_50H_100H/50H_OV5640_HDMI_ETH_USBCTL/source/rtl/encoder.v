`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 260911: video source select key decoder
// 260912: KEY1~KEY5 直映五个视图（OV5640_1/2/HDMI/ETH/stitch）
// 261011: 改为"循环切换"（50H_OV5640_HDMI_ETHERNET_USBCTL_0_mid 专用）——
//         K18 或串口 '0' 每来一个脉冲，视图按
//             HDMI → CAM1 → CAM2 → ETH → 拼接画面 → HDMI（循环）
//         前进一步。L15/J17/K16 与串口 '1'~'3' 已改配
//         增强/去雾/中值（见 uart_key_ctrl.v 与顶层）。
//
// Input : key_next_pulse — 单周期脉冲（K18 或串口 '0'）
// Output: sel[2:0] @ sys_clk，编码不变（video_select / single_video_select
//         两个消费方无需任何改动）：
//           3'b000 HDMI | 3'b001 CAM1 | 3'b010 CAM2 | 3'b011 ETH | 3'b100 stitch
//         循环顺序 000→001→010→011→100→000（拼接画面在 ETH 之后）。
//
// Power-on: btn_deb_fix 在按键 RELEASE 期间也计数，复位后 ~20ms 每个未按
// 的键都会发一个假脉冲；60ms 开机屏蔽吞掉它，上电画面即复位值 3'b000
// （HDMI 单路）。副作用：前 60ms 内的按键被忽略。
//////////////////////////////////////////////////////////////////////////////////
module encoder(
    input              clk,            // sys_clk 50MHz (same domain as btn_deb_fix)
    input              rst_n,
    input              key_next_pulse, // K18 / 串口 '0'：切到下一个视图
    output reg [2:0]   sel
);
    // 260912: swallow the power-on spurious pulses (60 ms @ 50 MHz) — 保留
    reg  [21:0] boot_cnt;
    wire        boot_done = (boot_cnt == 22'd3_000_000);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            boot_cnt <= 22'd0;
        else if (!boot_done)
            boot_cnt <= boot_cnt + 1'b1;
    end
    wire next_m = key_next_pulse & boot_done;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            sel <= 3'b000;                              // power-on default: HDMI single view
        else if (next_m) begin
            if (sel == 3'b100)
                sel <= 3'b000;                          // 拼接画面 -> HDMI（循环回起点）
            else
                sel <= sel + 1'b1;                      // HDMI->CAM1->CAM2->ETH->stitch
        end else
            sel <= sel;                                 // no pulse -> hold
    end
endmodule
