`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 260919  UART key control（'0'~'7' 直映 KEY1~KEY8）
// 261011  重排 / 261019 / 261023 / 261028 两次扩位
// 261028b 与物理键 1:1 对齐（用户定稿）：
//
//   '0' (0x30) / 0x00 -> key_next_pulse      画面切换 HDMI→CAM1→CAM2→ETH→拼接→HDMI
//   '1' (0x31) / 0x01 -> key_enh_pulse       单路亮度自适应 开/关（增亮+增暗合并）
//   '2' (0x32) / 0x02 -> 空（261031 夜景修复整级删除，此码保留但无动作）
//   '3' (0x33) / 0x03 -> key_defog_pulse     单路 720P 去雾 开/关
//   '4' (0x34) / 0x04 -> key_aux_pulse       倒车辅助线    开/关
//   '5' (0x35) / 0x05 -> key_enh360_pulse    拼接增亮      开/关（写侧四路 image_enhance）
//   '6' (0x36) / 0x06 -> key_defog360_pulse  拼接去雾      开/关（写侧四路 defog_rgb）
//   '7' 及其余字节一律丢弃（261028c 起 '7'/H17 空出）。
//   注意：码位保持 261028c 的编号不变（'2' 空着），不要重排，否则现有
//   串口脚本 / 上位机会整体错位。
//
// 配套物理按键（顶层相或，pin 见 .fdc）：
//   key_sel[0] K18 画面切换 | key_sel[1] L15 亮度自适应
//   key_sel[2] K16 单路去雾 | key_sel[3] J16 倒车辅助线
//   key_defog360 J19 拼接增亮 | key_defog H20 拼接去雾
//   261031：J17 上的夜景修复键已删除，.fdc 中不再约束该脚。
//
// 接收链与时序防呆不变：uart_rx(官方 04_uart_test) -> rx_en 上升沿作
// 1 拍选通 -> 字节延后 1 拍配合 vld 解码，避开同拍采样竞争。
//////////////////////////////////////////////////////////////////////////////////
module uart_key_ctrl #(
    parameter              BPS_NUM = 16'd434      // 115200bps @ 50MHz sys_clk
)(
    //input ports
    input                  clk,                   // sys_clk 50MHz，与 btn_deb_fix 同域
    input                  rst_n,
    input                  uart_rx_pin,           // 板上 R8（USB-UART 的 TX）

    //output ports —— 均为 1 个 clk 宽度的脉冲
    output                 key_next_pulse,        // '0'：画面切换（encoder 消费）
    output                 key_enh_pulse,         // '1'：单路亮度自适应 开/关
    output                 key_defog_pulse,       // '3'：单路 720P 去雾 开/关
    output                 key_aux_pulse,         // '4'：倒车辅助线 开/关
    output                 key_enh360_pulse,      // '5'：拼接增亮 开/关
    output                 key_defog360_pulse     // '6'：拼接去雾 开/关
);

//==========================================================================
// 官方 uart_rx 接收模块（source/rtl/uart_rx.v，原样整合）
//==========================================================================
    wire        [7:0]  rx_data;
    wire               rx_en;

    uart_rx #(
        .BPS_NUM            (BPS_NUM)
    ) u_uart_rx (
        .clk                (clk),
        .uart_rx            (uart_rx_pin),
        .rx_data            (rx_data),
        .rx_en              (rx_en)
    );

//==========================================================================
// 字节就绪脉冲（原样保留）
//   uart_rx 里 rx_en 在 RECEIV_STOP 拉高（同一拍 rx_data 也刷新），
//   并在 RECEIV_END 保持，所以不能直接当"单周期"用。
//   这里取 rx_en 的上升沿 -> rx_stb（严格 1 拍），
//   再把 rx_byte_vld 延后 1 拍，用"已稳定"的 rx_byte 去解码，
//   彻底避开同拍采样风险。
//==========================================================================
    reg                rx_en_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rx_en_d <= 1'b0;
        else        rx_en_d <= rx_en;
    end

    wire               rx_stb = rx_en & ~rx_en_d;     // 单周期接收完成标志

    reg         [7:0]  rx_byte;
    reg                rx_byte_vld;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_byte     <= 8'hFF;
            rx_byte_vld <= 1'b0;
        end else begin
            rx_byte_vld <= rx_stb;                    // 延后 1 拍的"数据有效"
            if (rx_stb) rx_byte <= rx_data;           // 该拍与 vld 同时装载
        end
    end

//==========================================================================
// 解码（261011 重排 / 261023 扩到 '5'）：'0'~'5'(0x30~0x35) 或 0x00~0x05 -> 0~5，
// 其它（'6'~'7'、0x06~0x07、任意其他字节）-> 0xFF 丢弃
//==========================================================================
    // 261028c: 有效码 '0'~'6'（0x06 同步），'7' 及其余字节丢弃
    wire        [7:0]  rx_code =
        (rx_byte >= 8'h30 && rx_byte <= 8'h36) ? (rx_byte - 8'h30) :
        (rx_byte <= 8'h06)                     ?  rx_byte          :
                                                  8'hFF;

    assign key_next_pulse     = rx_byte_vld & (rx_code == 8'd0);   // '0' 画面切换
    assign key_enh_pulse      = rx_byte_vld & (rx_code == 8'd1);   // '1' 亮度自适应
    // 261031：'2' 原来是夜景修复，功能已删除 —— 码位保留、不产生任何脉冲
    assign key_defog_pulse    = rx_byte_vld & (rx_code == 8'd3);   // '3' 单路去雾
    assign key_aux_pulse      = rx_byte_vld & (rx_code == 8'd4);   // '4' 倒车辅助线
    assign key_enh360_pulse   = rx_byte_vld & (rx_code == 8'd5);   // '5' 拼接增亮
    assign key_defog360_pulse = rx_byte_vld & (rx_code == 8'd6);   // '6' 拼接去雾

endmodule
