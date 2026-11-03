`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 260911: final 2:1 between the quad-stitch branch (v3_*) and the single-view
// branch (sng_*). Both branches share the color_bar time base and the same
// vrd delay chain depth.
//
// 260915 (V4 single-chain rework): hs/vs/de/data are now muxed TOGETHER by the
// frame-latched flag (s0_frame). Timing always follows the selected branch, so
// any pipeline depth in the downstream processing chain (image_enhance,
// defog_rgb, ...) delays the whole picture uniformly — a pipeline can no longer
// show up as a pixel/line offset. This retires the old "timing always passes
// the stitch branch" shortcut, which made every display-side insertion a
// geometry bug (see the 260915 defog right/down shift).
//
// Switch hygiene: the branch flag is latched only at color_vs frame start, so
// a switch replaces whole frames — no tearing. Total switch latency is at most
// one frame plus the CH4 ping-pong lag (~2 frames worst case).
//
// o_ctrl (V4 new): the stitch/single mode bit, taken from the SAME s0_frame
// latch that muxes the four output lines. It is the only legal "which mode is
// on screen" source for the downstream chain: it changes exactly when the
// picture changes — never earlier (sel[2] flips on the key press, 1~2 frames
// before the picture) and never later. Downstream must NOT add another
// synchronizer/latch on it (that would make it late — plan V4 §10-D7).
//
// sel[2] = 1 -> stitch, 0 -> single view (see encoder.v for the key map).
//////////////////////////////////////////////////////////////////////////////////
module video_select(
    input  wire        clk,          // pix_clk_w 74.21875MHz
    input  wire        rst_n,
    input  wire [2:0]  sel,          // already 2-FF synchronized to pix_clk_w
    input  wire        color_vs,     // frame boundary reference (color_bar vs)
    // stitch branch (existing v3_*)
    input  wire        splice_hs,
    input  wire        splice_vs,
    input  wire        splice_de,
    input  wire [15:0] splice_data,
    // single-view branch (single_video_select output)
    input  wire        single_hs,
    input  wire        single_vs,
    input  wire        single_de,
    input  wire [15:0] single_data,
    // output: four lines muxed by the frame-latched flag
    output reg         o_hs,
    output reg         o_vs,
    output reg         o_de,
    output reg  [15:0] o_data,
    // output: mode bit, same latch as o_data — 1 = stitch / 0 = single
    output reg         o_ctrl
);
    // one register on color_vs to build an edge strobe
    reg color_vs_d;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) color_vs_d <= 1'b0;
        else        color_vs_d <= color_vs;
    end

    // frame-boundary latch of the branch flag
    reg s0_frame;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            s0_frame <= 1'b0;                            // default: single view
        else if (color_vs && !color_vs_d)
            s0_frame <= sel[2];                          // sel[2]=1 -> stitch
    end

    // four-line 2:1 — data and timing always come from the SAME latch (V4 iron
    // rule: a selected stream carries its own hs/vs/de, geometry is exact)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            o_hs   <= 1'b0;
            o_vs   <= 1'b0;
            o_de   <= 1'b0;
            o_data <= 16'd0;
        end else begin
            o_hs   <= s0_frame ? splice_hs   : single_hs;
            o_vs   <= s0_frame ? splice_vs   : single_vs;
            o_de   <= s0_frame ? splice_de   : single_de;
            o_data <= s0_frame ? splice_data : single_data;
        end
    end

    // mode bit out — same s0_frame, flips on the same edge as o_data
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) o_ctrl <= 1'b0;
        else        o_ctrl <= s0_frame;
    end

endmodule
