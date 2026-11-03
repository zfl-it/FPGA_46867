`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// reverse_aux_lines — 倒车辅助线叠加 v5（261024b：几何改为逐行直算，无累加器）
//
// 为什么不用累加器（v4 的问题）：
//   v4 用 x += d; d += dd 逐行递推，几何变成了"历史状态" —— 只要某一行的
//   de_fall 没按预期发生/多发生一次，后面的行全部带着偏差，表现为中心漂移、
//   线条不直。v5 把几何改成 y 的纯函数，每行独立重算，任何一行出错都不累积。
//
// 几何（1280x720，全部 localparam）：
//   整体宽度按 80% 缩放（绕两线中点 652.5 缩放）：线间距、斜率、曲率偏移
//   三者同时 ×0.8，所以上端/下端/任意角度的间距都严格是原来的 80%。
//   k   = y - 370                     窗口内 0..328，窗口外钳位
//   u   = 328 - k
//   dx  = (k*243) >> 8                0.8*19/16 = 0.95 px/行（243/256，误差 0.08%）
//   off = (Kp * u * u) >> 17          Kp = round(K*131072/107584*0.8)
//   xl  = 450 - dx + s*off            s = +1 右弯 / -1 左弯
//   xr  = 855 + dx + s*off
//   近端 k=328 处 off=0（支点固定），远端 k=0 处偏 s*0.8K（摆动）。
//
// 样式：
//   线宽 8 px（每行水平跨度 [c-3, c+4]），虚线节距 32 / 实 20、每段相位归零，
//   绿/黄段上端各一条横向虚线（8 行厚），红杆在近端；端帽已删除。
//   斜率上限 19/16 + 2*260/328 = 2.77 px/行 < 8 px 跨度 ⇒ 任意角度不断线。
//
// 时序（261028）：零延迟不变 —— o_vs/o_de 直通，o_data 纯组合替换。
//       几何链改 5 级流水（k_r / u_r,dx_r / u2_r / off26_r / xlc_r,xrc_r）：
//       y_cnt 整行恒定、几何只在 de_fall 采样，故插拍与原式逐位等价。
//       像素通路仍是 6 个 14 bit 有符号比较，未改动。
//
// angle_in[5:0] = {dir, mag[4:0]}：dir 0=右弯 1=左弯，mag 0..30。vs 沿锁存。
//////////////////////////////////////////////////////////////////////////////////
`default_nettype none

module reverse_aux_lines #(
    parameter [10:0] H_ACT = 11'd1280,
    parameter [9:0]  V_ACT = 10'd720
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        mode_in,       // u_vsel o_ctrl：1=拼接 / 0=单路（帧锁存）
    input  wire        aux_en,        // 用户开关（帧锁存，= key_aux_f）
    input  wire [5:0]  angle_in,      // {dir, mag[4:0]}，pix_clk_w 域
    input  wire        vs,
    input  wire        de,
    input  wire [15:0] data,
    output wire        o_vs,
    output wire        o_de,
    output wire [15:0] o_data,
    output wire        aux_ctrl       // = mode_in | aux_en：1=白行（拼接或带线帧）
);

    wire bypass   = mode_in | (~aux_en);
    assign aux_ctrl = mode_in | aux_en;

    //------------------------------------------------------------------
    // 行列计数器（与数据同拍，无流水）
    //------------------------------------------------------------------
    reg vs_r, de_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) {vs_r, de_r} <= 2'b00;
        else        {vs_r, de_r} <= {vs, de};
    end
    wire vs_rise = vs & ~vs_r;
    wire de_fall = de_r & ~de;

    reg [10:0] y_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)        y_cnt <= 11'd0;
        else if (vs_rise)  y_cnt <= 11'd0;
        else if (de_fall)  y_cnt <= y_cnt + 11'd1;
    end

    reg [11:0] x_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)      x_cnt <= 12'd0;
        else if (!de)    x_cnt <= 12'd0;
        else             x_cnt <= x_cnt + 12'd1;
    end

    //------------------------------------------------------------------
    // 颜色常数
    //------------------------------------------------------------------
    localparam [15:0] C_RED = 16'hF800;
    localparam [15:0] C_YLW = 16'hFFE0;
    localparam [15:0] C_GRN = 16'h07E0;

    //------------------------------------------------------------------
    // 几何常数（720p 像素坐标）
    //------------------------------------------------------------------
    localparam [10:0] SLANT_TOP = 11'd370;   // 远端（摆动端）
    localparam [10:0] SLANT_BOT = 11'd698;   // 近端（支点，固定）
    localparam [11:0] LINE_XL0  = 12'd450;   // y=370 处左线中心（mag=0，间距的80%）
    localparam [11:0] LINE_XR0  = 12'd855;   // y=370 处右线中心（mag=0，间距的80%）

    // 色段（段长 = 32n+20，段末齐口）
    localparam [10:0] GRN_Y0 = 11'd370, GRN_Y1 = 11'd453;   //  84 行 -> 3 段
    localparam [10:0] YLW_Y0 = 11'd486, YLW_Y1 = 11'd601;   // 116 行 -> 4 段
    localparam [10:0] RED_Y0 = 11'd615, RED_Y1 = 11'd698;   //  84 行 -> 3 段

    // 横向虚线（8 行厚，绿/黄在上端，红在近端）
    localparam [10:0] GBR_Y0 = 11'd370, GBR_Y1 = 11'd377;
    localparam [10:0] YBR_Y0 = 11'd486, YBR_Y1 = 11'd493;
    localparam [10:0] RBR_Y0 = 11'd658, RBR_Y1 = 11'd665;

    localparam [4:0]  DASH_ON = 5'd20;                  // 节距 32 / 实 20
    localparam signed [13:0] LW_LO = 14'sd3;            // 线宽 8：c-3 .. c+4
    localparam signed [13:0] LW_HI = 14'sd4;

    //------------------------------------------------------------------
    // 角度 -> 系数（vs 沿锁存）
    //   Kp = round( K * 131072/107584 * 0.8 )，K = round(450*tanθ)
    //   末尾的 0.8 就是"整体宽度缩到 80%"里的曲率部分
    //------------------------------------------------------------------
    wire [4:0] mag  = angle_in[4:0];
    wire       bend = angle_in[5];        // 0=右弯(+x) 1=左弯(-x)

    reg [8:0] kp_c;

    always @(*) begin
        case (mag)
            5'd0  : kp_c = 9'd0;
            5'd1  : kp_c = 9'd8;
            5'd2  : kp_c = 9'd16;
            5'd3  : kp_c = 9'd23;
            5'd4  : kp_c = 9'd30;
            5'd5  : kp_c = 9'd38;
            5'd6  : kp_c = 9'd46;
            5'd7  : kp_c = 9'd54;
            5'd8  : kp_c = 9'd61;
            5'd9  : kp_c = 9'd69;
            5'd10 : kp_c = 9'd77;
            5'd11 : kp_c = 9'd85;
            5'd12 : kp_c = 9'd94;
            5'd13 : kp_c = 9'd101;
            5'd14 : kp_c = 9'd109;
            5'd15 : kp_c = 9'd118;
            5'd16 : kp_c = 9'd126;
            5'd17 : kp_c = 9'd134;
            5'd18 : kp_c = 9'd142;
            5'd19 : kp_c = 9'd151;
            5'd20 : kp_c = 9'd160;
            5'd21 : kp_c = 9'd169;
            5'd22 : kp_c = 9'd177;
            5'd23 : kp_c = 9'd186;
            5'd24 : kp_c = 9'd195;
            5'd25 : kp_c = 9'd205;
            5'd26 : kp_c = 9'd213;
            5'd27 : kp_c = 9'd223;
            5'd28 : kp_c = 9'd233;
            5'd29 : kp_c = 9'd243;
            5'd30 : kp_c = 9'd253;
            default: kp_c = 9'd0;
        endcase
    end

    reg [8:0] kp_f;
    reg       bend_f;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            kp_f <= 9'd0; bend_f <= 1'b0;
        end else if (vs_rise) begin
            kp_f <= kp_c;  bend_f <= bend;
        end
    end

    //------------------------------------------------------------------
    // 逐行直算（无状态）：k 钳在 0..328
    // 261028: 五级流水。y_cnt 整行恒定（1650 clk），几何只在 de_fall 采样，
    //   中间插入任意级寄存器都与原式逐位等价 —— 只是把 20.2 ns 的
    //   组合链拆成 5 段。详见《时序收敛修正方案》。
    //   y_cnt -> k_r -> u_r/dx_r -> u2_r -> off26_r -> xlc_r/xrc_r -> (de_fall)
    //------------------------------------------------------------------
    wire [10:0] k_raw = y_cnt - SLANT_TOP;
    wire [10:0] k_w   = (y_cnt < SLANT_TOP) ? 11'd0 :
                        (y_cnt > SLANT_BOT) ? 11'd328 : k_raw;

    // stage 1: k 钳位
    reg [10:0] k_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) k_r <= 11'd0;
        else        k_r <= k_w;
    end

    // stage 2: u = 328-k，dx = (k*243)>>8（0.95 px/行，末行误差 < 0.6 px）
    reg [10:0] u_r;
    reg [12:0] dx_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            u_r  <= 11'd0;
            dx_r <= 13'd0;
        end else begin
            u_r  <= 11'd328 - k_r;
            dx_r <= ({2'b0, k_r, 8'b0} - {2'b0, k_r, 3'b0}
                     - {2'b0, k_r, 2'b0} - {2'b0, k_r}) >> 8;
        end
    end

    // stage 3: u^2  (0..107584)
    reg [16:0] u2_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) u2_r <= 17'd0;
        else        u2_r <= u_r * u_r;
    end

    // stage 4: Kp*u^2  (<= 253*107584 = 27.2M)
    reg [25:0] off26_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) off26_r <= 26'd0;
        else        off26_r <= kp_f * u2_r;
    end

    // stage 5: off = off26>>17 (/131072 -> 0..208)、符号、左右线中心
    wire [8:0] off = off26_r[25:17];
    wire signed [13:0] soff = bend_f ? -$signed({5'd0, off})
                                     :  $signed({5'd0, off});

    reg signed [13:0] xlc_r, xrc_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            xlc_r <= 14'sd0; xrc_r <= 14'sd0;
        end else begin
            xlc_r <= $signed({2'b0, LINE_XL0}) - $signed({1'b0, dx_r}) + soff;
            xrc_r <= $signed({2'b0, LINE_XR0}) + $signed({1'b0, dx_r}) + soff;
        end
    end

    // stage 6: de_fall 锁存（行消隐里更新，像素通路只用寄存器）
    reg signed [13:0] xl_lo, xl_hi, xr_lo, xr_hi;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            xl_lo <= 14'sd0; xl_hi <= 14'sd0;
            xr_lo <= 14'sd0; xr_hi <= 14'sd0;
        end else if (de_fall) begin
            xl_lo <= xlc_r - LW_LO;
            xl_hi <= xlc_r + LW_HI;
            xr_lo <= xrc_r - LW_LO;
            xr_hi <= xrc_r + LW_HI;
        end
    end

    //------------------------------------------------------------------
    // 像素比较（组合，与数据同拍）
    //------------------------------------------------------------------
    wire [10:0] y = y_cnt;
    wire [11:0] x = x_cnt;

    wire signed [13:0] xs = $signed({2'b0, x});

    wire in_slant_y = (y >= SLANT_TOP) && (y <= SLANT_BOT);
    wire hit_l = (xs >= xl_lo) && (xs <= xl_hi);
    wire hit_r = (xs >= xr_lo) && (xs <= xr_hi);
    wire hit_slant = in_slant_y & (hit_l | hit_r);

    // 色段 + 段内相位（每段归零 -> 段首必是完整实线）
    wire [10:0] dg_y = y - GRN_Y0;
    wire [10:0] dy_y = y - YLW_Y0;
    wire [10:0] dr_y = y - RED_Y0;

    wire in_grn = (y >= GRN_Y0) && (y <= GRN_Y1);
    wire in_ylw = (y >= YLW_Y0) && (y <= YLW_Y1);
    wire in_red = (y >= RED_Y0) && (y <= RED_Y1);

    wire dsh_g = (dg_y[4:0] < DASH_ON);
    wire dsh_y = (dy_y[4:0] < DASH_ON);
    wire dsh_r = (dr_y[4:0] < DASH_ON);

    // 横向虚线：8 行厚，x 从本行左线左沿到右线右沿
    wire in_gbar = (y >= GBR_Y0) && (y <= GBR_Y1);
    wire in_ybar = (y >= YBR_Y0) && (y <= YBR_Y1);
    wire in_rbar = (y >= RBR_Y0) && (y <= RBR_Y1);
    wire dsh_x   = (x[4:0] < DASH_ON);

    wire hit_bar_x = (xs >= xl_lo) && (xs <= xr_hi);
    wire hit_bar   = (in_gbar | in_ybar | in_rbar) & hit_bar_x & dsh_x;

    //------------------------------------------------------------------
    // 零延迟像素替换。优先级：横杆 > 斜线；红 > 黄 > 绿
    //------------------------------------------------------------------
    reg [15:0] aux_data;
    always @(*) begin
        if (hit_bar && in_rbar)                     aux_data = C_RED;
        else if (hit_bar && in_ybar)                aux_data = C_YLW;
        else if (hit_bar && in_gbar)                aux_data = C_GRN;
        else if (hit_slant && in_red && dsh_r)      aux_data = C_RED;
        else if (hit_slant && in_ylw && dsh_y)      aux_data = C_YLW;
        else if (hit_slant && in_grn && dsh_g)      aux_data = C_GRN;
        else                                        aux_data = data;
    end

    assign o_vs   = vs;
    assign o_de   = de;
    assign o_data = bypass ? data : aux_data;

endmodule


//////////////////////////////////////////////////////////////////////////////////
// traj_uart_ctrl — 倒车轨迹角度帧解析（261024 新增，未改动）
//
// 为什么要单独一个接收器：现有 uart_key_ctrl 是单字节直译（'0'~'5'），
// 角度帧里的 '0'/'3'/'1' 会被它当成 视图切换 / 辅助线开关 / 增强 执行一遍。
// 这里用第二个 uart_rx 专门吃角度帧，uart_key_ctrl 源码一个字不改；
// 解析期间拉高 busy，顶层把 6 条单字节脉冲与 ~busy 相与，物理按键不受影响。
//
// 帧格式（6 字节 ASCII，115200 8N1）：
//   'A' | dir('0'右 / '1'左) | 十位 | 个位 | '1' | '1'
//   例：A00011 直行 / A15011 右 15° / A13011 左 30°
//   角度 ≤ 30，末两位必须都是 '1'，否则整帧丢弃并保持上次角度。
//   收到 'A' 后 5 ms 内没收满 -> 丢弃并释放 busy。
//////////////////////////////////////////////////////////////////////////////////
module traj_uart_ctrl #(
    parameter [15:0] BPS_NUM = 16'd434,      // 115200bps @ 50MHz
    parameter [19:0] TMO_NUM = 20'd250000    // 5 ms @ 50MHz
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       uart_rx_pin,
    output reg  [5:0] angle,                 // {dir, mag[4:0]}
    output reg        angle_vld,             // 单拍
    output reg        busy                   // 帧解析中
);

    wire [7:0] rx_data;
    wire       rx_en;

    uart_rx #(
        .BPS_NUM            (BPS_NUM)
    ) u_rx_traj (
        .clk                (clk),
        .uart_rx            (uart_rx_pin),
        .rx_data            (rx_data),
        .rx_en              (rx_en),
        .rx_finish          ()
    );

    reg  rx_en_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rx_en_d <= 1'b0;
        else        rx_en_d <= rx_en;
    end
    wire rx_stb = rx_en & ~rx_en_d;

    reg [7:0] rx_byte;
    reg       rx_vld;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_byte <= 8'hFF;
            rx_vld  <= 1'b0;
        end else begin
            rx_vld <= rx_stb;
            if (rx_stb) rx_byte <= rx_data;
        end
    end

    localparam [2:0] ST_IDLE = 3'd0, ST_D1 = 3'd1, ST_D2 = 3'd2,
                     ST_D3   = 3'd3, ST_D4 = 3'd4, ST_D5 = 3'd5;

    reg [2:0]  st;
    reg [7:0]  b1, b2, b3;
    reg [19:0] tmo;

    wire is_digit = (rx_byte >= 8'h30) && (rx_byte <= 8'h39);
    wire [7:0] d2 = b2 - 8'h30;
    wire [7:0] d3 = b3 - 8'h30;
    wire [7:0] val8 = (d2 * 8'd10) + d3;      // ≤ 99

    wire tmo_hit = (tmo >= TMO_NUM);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st        <= ST_IDLE;
            busy      <= 1'b0;
            angle_vld <= 1'b0;
            angle     <= 6'd0;                // 上电默认直行
            tmo       <= 20'd0;
            b1        <= 8'h30;
            b2        <= 8'h30;
            b3        <= 8'h30;
        end else begin
            angle_vld <= 1'b0;

            if (rx_vld) begin
                tmo <= 20'd0;                 // 每收到一个字节就刷新超时
                case (st)
                    ST_IDLE : begin
                        if (rx_byte == 8'h41) begin          // 'A'
                            st   <= ST_D1;
                            busy <= 1'b1;
                        end
                    end
                    ST_D1 : begin
                        if (rx_byte == 8'h30 || rx_byte == 8'h31) begin
                            b1 <= rx_byte;                   // '0' 右 / '1' 左
                            st <= ST_D2;
                        end else begin
                            st   <= ST_IDLE;
                            busy <= 1'b0;
                        end
                    end
                    ST_D2 : begin
                        if (is_digit) begin
                            b2 <= rx_byte;
                            st <= ST_D3;
                        end else begin
                            st   <= ST_IDLE;
                            busy <= 1'b0;
                        end
                    end
                    ST_D3 : begin
                        if (is_digit) begin
                            b3 <= rx_byte;
                            st <= ST_D4;
                        end else begin
                            st   <= ST_IDLE;
                            busy <= 1'b0;
                        end
                    end
                    ST_D4 : begin
                        if (rx_byte == 8'h31) st <= ST_D5;   // '1'
                        else begin
                            st   <= ST_IDLE;
                            busy <= 1'b0;
                        end
                    end
                    ST_D5 : begin
                        if (rx_byte == 8'h31 && val8 <= 8'd30) begin
                            angle     <= {b1[0], val8[4:0]};
                            angle_vld <= 1'b1;
                        end
                        st   <= ST_IDLE;
                        busy <= 1'b0;
                    end
                    default : begin
                        st   <= ST_IDLE;
                        busy <= 1'b0;
                    end
                endcase
            end else if (busy && tmo_hit) begin
                st   <= ST_IDLE;
                busy <= 1'b0;
                tmo  <= 20'd0;
            end else if (busy) begin
                tmo <= tmo + 20'd1;
            end
        end
    end

endmodule

`default_nettype wire
