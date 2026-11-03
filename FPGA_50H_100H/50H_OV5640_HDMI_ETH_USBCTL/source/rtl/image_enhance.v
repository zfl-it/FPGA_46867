`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// image_enhance — sRGB transfer function for RGB565 (IEC 61966-2-1)
//   Replaces Gamma 1.5 with industry-standard sRGB curve.
//   Linear segment near black eliminates noise amplification of pure power-law.
//   Latency: 1 pixel clock cycle
//   Resources: ~3 small LUTs (synthesised as distributed RAM)
//
// Input:  {R[4:0], G[5:0], B[4:0]}  RGB565, resolution-independent per-pixel LUT
// Output: {R[4:0], G[5:0], B[4:0]}  RGB565, sRGB-corrected
//   260914: in use at 5 places in two roles — 360p write side (CH0~CH3:
//   u_enhance_cam1/cam2/hdmi/eth, one per source clock domain) and the
//   720p60 post-select unified chain (u_enhance_sng @ pix_clk_w; V4 260915:
//   that single instance sits after u_vsel and serves all five streams, and
//   its bypass is driven by the joint KEY x mode encoder).
//////////////////////////////////////////////////////////////////////////////////
module image_enhance (
    input               clk,
    input               rst_n,
    input               bypass,             // 1 = pass-through, 0 = sRGB on
    input               de_in,
    input       [15:0]  data_in,            // RGB565
    output reg          de_out,
    output reg  [15:0]  data_out            // RGB565
);

/////////////////////////////////////////////////////////////////////////////////////
// Decompose RGB565
/////////////////////////////////////////////////////////////////////////////////////
wire [4:0] r_in = data_in[15:11];
wire [5:0] g_in = data_in[10: 5];
wire [4:0] b_in = data_in[ 4: 0];

/////////////////////////////////////////////////////////////////////////////////////
// sRGB transfer function (IEC 61966-2-1)
//   if x <= 0.0031308:  y = 12.92 * x          (linear segment)
//   else:                y = 1.055*x^(1/2.4)-0.055  (power-law segment)
//
//   R/B (5-bit):  round(srgb(i/31) * 31)   for i in 0..31
//   G   (6-bit):  round(srgb(i/63) * 63)   for i in 0..63
//
//   Key difference vs Gamma 1.5: linear segment prevents noise amplification
//   in very dark regions (input < 0.00313, i.e. i < 0.1 in 5-bit = i < 1).
/////////////////////////////////////////////////////////////////////////////////////
reg [4:0] r_lut_val;
reg [5:0] g_lut_val;
reg [4:0] b_lut_val;

// R LUT (5-bit, 32 levels) — sRGB
always @(*) begin
    case (r_in)
        5'd 0: r_lut_val = 5'd 0;
        5'd 1: r_lut_val = 5'd 6;
        5'd 2: r_lut_val = 5'd 9;
        5'd 3: r_lut_val = 5'd11;
        5'd 4: r_lut_val = 5'd12;
        5'd 5: r_lut_val = 5'd14;
        5'd 6: r_lut_val = 5'd15;
        5'd 7: r_lut_val = 5'd16;
        5'd 8: r_lut_val = 5'd17;
        5'd 9: r_lut_val = 5'd18;
        5'd10: r_lut_val = 5'd19;
        5'd11: r_lut_val = 5'd20;
        5'd12: r_lut_val = 5'd20;
        5'd13: r_lut_val = 5'd21;
        5'd14: r_lut_val = 5'd22;
        5'd15: r_lut_val = 5'd22;
        5'd16: r_lut_val = 5'd23;
        5'd17: r_lut_val = 5'd24;
        5'd18: r_lut_val = 5'd24;
        5'd19: r_lut_val = 5'd25;
        5'd20: r_lut_val = 5'd26;
        5'd21: r_lut_val = 5'd26;
        5'd22: r_lut_val = 5'd27;
        5'd23: r_lut_val = 5'd27;
        5'd24: r_lut_val = 5'd28;
        5'd25: r_lut_val = 5'd28;
        5'd26: r_lut_val = 5'd29;
        5'd27: r_lut_val = 5'd29;
        5'd28: r_lut_val = 5'd30;
        5'd29: r_lut_val = 5'd30;
        5'd30: r_lut_val = 5'd31;
        5'd31: r_lut_val = 5'd31;
    endcase
end

// G LUT (6-bit, 64 levels) — sRGB
always @(*) begin
    case (g_in)
        6'd 0: g_lut_val = 6'd 0;
        6'd 1: g_lut_val = 6'd 8;
        6'd 2: g_lut_val = 6'd12;
        6'd 3: g_lut_val = 6'd15;
        6'd 4: g_lut_val = 6'd18;
        6'd 5: g_lut_val = 6'd20;
        6'd 6: g_lut_val = 6'd21;
        6'd 7: g_lut_val = 6'd23;
        6'd 8: g_lut_val = 6'd25;
        6'd 9: g_lut_val = 6'd26;
        6'd10: g_lut_val = 6'd27;
        6'd11: g_lut_val = 6'd29;
        6'd12: g_lut_val = 6'd30;
        6'd13: g_lut_val = 6'd31;
        6'd14: g_lut_val = 6'd32;
        6'd15: g_lut_val = 6'd33;
        6'd16: g_lut_val = 6'd34;
        6'd17: g_lut_val = 6'd35;
        6'd18: g_lut_val = 6'd36;
        6'd19: g_lut_val = 6'd37;
        6'd20: g_lut_val = 6'd38;
        6'd21: g_lut_val = 6'd39;
        6'd22: g_lut_val = 6'd39;
        6'd23: g_lut_val = 6'd40;
        6'd24: g_lut_val = 6'd41;
        6'd25: g_lut_val = 6'd42;
        6'd26: g_lut_val = 6'd43;
        6'd27: g_lut_val = 6'd43;
        6'd28: g_lut_val = 6'd44;
        6'd29: g_lut_val = 6'd45;
        6'd30: g_lut_val = 6'd45;
        6'd31: g_lut_val = 6'd46;
        6'd32: g_lut_val = 6'd47;
        6'd33: g_lut_val = 6'd47;
        6'd34: g_lut_val = 6'd48;
        6'd35: g_lut_val = 6'd49;
        6'd36: g_lut_val = 6'd49;
        6'd37: g_lut_val = 6'd50;
        6'd38: g_lut_val = 6'd50;
        6'd39: g_lut_val = 6'd51;
        6'd40: g_lut_val = 6'd52;
        6'd41: g_lut_val = 6'd52;
        6'd42: g_lut_val = 6'd53;
        6'd43: g_lut_val = 6'd53;
        6'd44: g_lut_val = 6'd54;
        6'd45: g_lut_val = 6'd54;
        6'd46: g_lut_val = 6'd55;
        6'd47: g_lut_val = 6'd55;
        6'd48: g_lut_val = 6'd56;
        6'd49: g_lut_val = 6'd56;
        6'd50: g_lut_val = 6'd57;
        6'd51: g_lut_val = 6'd57;
        6'd52: g_lut_val = 6'd58;
        6'd53: g_lut_val = 6'd58;
        6'd54: g_lut_val = 6'd59;
        6'd55: g_lut_val = 6'd59;
        6'd56: g_lut_val = 6'd60;
        6'd57: g_lut_val = 6'd60;
        6'd58: g_lut_val = 6'd61;
        6'd59: g_lut_val = 6'd61;
        6'd60: g_lut_val = 6'd62;
        6'd61: g_lut_val = 6'd62;
        6'd62: g_lut_val = 6'd63;
        6'd63: g_lut_val = 6'd63;
    endcase
end

// B LUT (5-bit, 32 levels) — same as R (both 5-bit sRGB)
always @(*) begin
    case (b_in)
        5'd 0: b_lut_val = 5'd 0;
        5'd 1: b_lut_val = 5'd 6;
        5'd 2: b_lut_val = 5'd 9;
        5'd 3: b_lut_val = 5'd11;
        5'd 4: b_lut_val = 5'd12;
        5'd 5: b_lut_val = 5'd14;
        5'd 6: b_lut_val = 5'd15;
        5'd 7: b_lut_val = 5'd16;
        5'd 8: b_lut_val = 5'd17;
        5'd 9: b_lut_val = 5'd18;
        5'd10: b_lut_val = 5'd19;
        5'd11: b_lut_val = 5'd20;
        5'd12: b_lut_val = 5'd20;
        5'd13: b_lut_val = 5'd21;
        5'd14: b_lut_val = 5'd22;
        5'd15: b_lut_val = 5'd22;
        5'd16: b_lut_val = 5'd23;
        5'd17: b_lut_val = 5'd24;
        5'd18: b_lut_val = 5'd24;
        5'd19: b_lut_val = 5'd25;
        5'd20: b_lut_val = 5'd26;
        5'd21: b_lut_val = 5'd26;
        5'd22: b_lut_val = 5'd27;
        5'd23: b_lut_val = 5'd27;
        5'd24: b_lut_val = 5'd28;
        5'd25: b_lut_val = 5'd28;
        5'd26: b_lut_val = 5'd29;
        5'd27: b_lut_val = 5'd29;
        5'd28: b_lut_val = 5'd30;
        5'd29: b_lut_val = 5'd30;
        5'd30: b_lut_val = 5'd31;
        5'd31: b_lut_val = 5'd31;
    endcase
end

/////////////////////////////////////////////////////////////////////////////////////
// Pipeline register — 1 cycle delay
//   bypass=1 : data_out = data_in (straight through, 1 cycle delayed)
//   bypass=0 : data_out = sRGB-corrected
/////////////////////////////////////////////////////////////////////////////////////
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        de_out   <= 1'b0;
        data_out <= 16'd0;
    end else begin
        de_out   <= de_in;
        if (bypass)
            data_out <= data_in;
        else
            data_out <= {r_lut_val, g_lut_val, b_lut_val};
    end
end

endmodule
