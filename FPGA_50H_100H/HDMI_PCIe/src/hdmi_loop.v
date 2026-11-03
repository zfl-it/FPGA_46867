`timescale 1ns / 1ps
`default_nettype none

module hdmi_loop(
    input wire        sys_clk,     // input system clock 25MHz


    //pcie
    input                       button_rst_n    ,
    input                       perst_n         ,

    //ov5640

    //clk and rst
    input                       ref_clk_n       ,      //100 Mhz
    input                       ref_clk_p       ,      //100 Mhz
    //diff signals

    input           [1:0]       rxn             ,
    input           [1:0]       rxp             ,
    output  wire    [1:0]       txn             ,
    output  wire    [1:0]       txp             ,

    // 2026-10-11: the unused SFP receive chain (sfp_img_recv + hsst_core +
    // word_align + video_packet_rec + rgb565_downscale_2of3) and the unused
    // video_sel input have been removed from this build.  The design is and
    // always was fixed to HDMI IN (pixclk_in); see the acquisition-path
    // comment further below.

    // DDR3
    output                      mem_rst_n       ,
    output                      mem_ck          ,
    output                      mem_ck_n        ,
    output                      mem_cke         ,
    output                      mem_cs_n        ,
    output                      mem_ras_n       ,
    output                      mem_cas_n       ,
    output                      mem_we_n        ,
    output                      mem_odt         ,
    output  [14:0]              mem_a           ,
    output  [2:0]               mem_ba          ,
    inout   [1:0]               mem_dqs         ,
    inout   [1:0]               mem_dqs_n       ,
    inout   [15:0]              mem_dq          ,
    output  [1:0]               mem_dm          ,
//    output                      ddr_init_done    ,

    //LED signals
    output reg                  ref_led         ,
    output reg                  pclk_led        ,
    output wire                 smlh_link_up    ,
    output wire                 rdlh_link_up    ,

    //hdmi
    output                      rstn_out        ,
    output                      iic_scl         ,
    inout                       iic_sda         ,
    input                       pixclk_in       ,
    input                       vs_in           ,
    input                       hs_in           /* synthesis PAP_MARK_DEBUG="true" */,
    input                       de_in           ,
    input     [7:0]             r_in            ,
    input     [7:0]             g_in            ,
    input     [7:0]             b_in            ,

    output                      led_int           ,

    input                       ddr_ref_clk_n    ,
    input                       ddr_ref_clk_p
);

reg  [15:0] rstn_1ms;
wire        pix_clk_1080p;
wire        cfg_clk;
wire        locked;
wire        pll_rst_n;
wire        rstn_out;
wire        init_over;

wire ddr_ref_clk_125m;
wire ddr_pll_lock;

GTP_INBUFGDS #(
    .IOSTANDARD ("LVDS"),
    .TERM_DIFF  ("OFF")
) u_ddr_ref_clk_ibuf (
    .I  (ddr_ref_clk_p),
    .IB (ddr_ref_clk_n),
    .O  (ddr_ref_clk_125m)
);

(* ASYNC_REG = "TRUE" *) reg r_cfg_lock_meta;
(* ASYNC_REG = "TRUE" *) reg r_cfg_lock_sync;

// Configure this PLL IP for:
//     input clock  = 25.000 MHz
//     output clock = 74.250 MHz
// Together with the sync_vg parameters (H_TOTAL=1650, V_TOTAL=750) this
// produces 1280x720 @ 60 Hz (720P60).  NOTE: the net/port name
// "pix_clk_1080p" is a historical leftover from the 1080p reference design;
// the frequency and all sync_vg parameters in this build are 720p values.
// Kept unrenamed so the diff against the reference stays minimal.
hdmi_pll hdmi_pll_inst (
  .clkout0(pix_clk_1080p),
  .lock(pll_rst_n),          // output
  .clkin1(sys_clk),      // input
  .rst(1'b0)             // input
);


PLL_50mHZ  U_PLL(
  .clkout0(cfg_clk),    // output
  .lock(locked),          // output
  .clkin1(sys_clk)       // 25MHz input; keep the configured cfg_clk frequency
);


ms72xx_ctl ms72xx_ctl(
    .clk         (  cfg_clk    ), //input       clk,
    .rst_n       (  rstn_out   ), //input       rstn,

    .init_over   (  init_over  ), //output      init_over,
    .iic_tx_scl  (             ), //output      iic_scl,
    .iic_tx_sda  (             ), //inout       iic_sda
    .iic_scl     (  iic_scl    ), //output      iic_scl,
    .iic_sda     (  iic_sda    )  //inout       iic_sda
);

assign    led_int  =  init_over;

//----------------------------------- SFP image receive module --------------------------------
// 2026-10-11: REMOVED.  This build never used the SFP video input
// (sfp_img_recv + hsst_core + word_align + video_packet_rec), so the whole
// chain was deleted together with its sys_reset_n synchronizer, which had no
// other consumer.

/* PLL lock is asynchronous status.  Synchronize it before using it in the
   configuration clock domain. */
always @(posedge cfg_clk or negedge locked) begin
    if (!locked) begin
        r_cfg_lock_meta <= 1'b0;
        r_cfg_lock_sync <= 1'b0;
    end else begin
        r_cfg_lock_meta <= 1'b1;
        r_cfg_lock_sync <= r_cfg_lock_meta;
    end
end

always @(posedge cfg_clk) begin
    if (!r_cfg_lock_sync) begin
        rstn_1ms <= 16'd0;
    end else if (rstn_1ms != 16'h2710) begin
        rstn_1ms <= rstn_1ms + 16'd1;
    end
end

assign rstn_out = (rstn_1ms == 16'h2710);

/* 2026-10-11: the SFP-only acquisition path (sfp_rx_clk reset synchronizer
   and rgb565_downscale_2of3) was removed together with the unused SFP
   receive chain.  The frame-buffer write side is fixed to HDMI IN; see the
   comment below for why no runtime clock mux is allowed here. */

/* ------------------------------------------------------------------------- */
/* Temporary HDMI-IN acquisition path                                        */
/* ------------------------------------------------------------------------- */
/*
 * This test build fixes the frame-buffer write side to HDMI IN at compile
 * time.  Do not replace this with a LUT mux such as
 *
 *     video_sel ? sfp_rx_clk : pixclk_in
 *
 * because switching unrelated clocks in ordinary fabric is not CDC-safe.
 *
 * Test-source requirement:
 *     1280 x 720 @ 60 Hz progressive RGB888
 *
 * The RGB bus, DE and VS are all sampled by their associated pixclk_in, then
 * delayed by the same two cycles before RGB888-to-RGB565 conversion.  No
 * multi-bit video bus crosses into another clock domain through ordinary
 * flip-flops; the later transfer to ddr_core_clk remains inside wr_buf's
 * vendor true dual-clock line RAM.
 */
wire hdmi_async_reset_n;
wire hdmi_video_reset_n;
wire hdmi_video_reset_n_neg;

(* ASYNC_REG = "TRUE" *) reg r_hdmi_reset_meta;
(* ASYNC_REG = "TRUE" *) reg r_hdmi_reset_sync;
(* ASYNC_REG = "TRUE" *) reg r_hdmi_reset_meta_neg;
(* ASYNC_REG = "TRUE" *) reg r_hdmi_reset_sync_neg;

reg [23:0] r_hdmi_data_d0
    /* synthesis syn_preserve=1 */;
reg [23:0] r_hdmi_data_d1
    /* synthesis PAP_MARK_DEBUG="true" syn_preserve=1 */;
reg        r_hdmi_de_d0
    /* synthesis syn_preserve=1 */;
reg        r_hdmi_de_d1
    /* synthesis PAP_MARK_DEBUG="true" syn_preserve=1 */;
reg        r_hdmi_vs_d0
    /* synthesis syn_preserve=1 */;
reg        r_hdmi_vs_d1
    /* synthesis PAP_MARK_DEBUG="true" syn_preserve=1 */;
reg        r_hdmi_hs_d0
    /* synthesis syn_preserve=1 */;
reg        r_hdmi_hs_d1
    /* synthesis PAP_MARK_DEBUG="true" syn_preserve=1 */;

/*
 * Keep both source-synchronous capture edges available until hardware
 * qualification has observed at least 10,000 consecutive 1280-pixel lines.
 * The 720p build uses the falling edge.  The corresponding FDC
 * input delays are constrained with -clock_fall; never change one without
 * changing the other.
 */
localparam HDMI_CAPTURE_NEGEDGE = 1'b1;
localparam integer HDMI_ACTIVE_PIXELS = 1280;
localparam [13:0] HDMI_DIAG_GOOD_LIMIT = 14'd10000;

reg [23:0] r_hdmi_data_neg_d0
    /* synthesis PAP_MARK_DEBUG="true" syn_preserve=1 */;
reg [23:0] r_hdmi_data_neg_d1
    /* synthesis PAP_MARK_DEBUG="true" syn_preserve=1 */;
reg        r_hdmi_de_neg_d0
    /* synthesis syn_preserve=1 */;
reg        r_hdmi_de_neg_d1
    /* synthesis PAP_MARK_DEBUG="true" syn_preserve=1 */;
reg        r_hdmi_vs_neg_d0
    /* synthesis syn_preserve=1 */;
reg        r_hdmi_vs_neg_d1
    /* synthesis PAP_MARK_DEBUG="true" syn_preserve=1 */;
reg        r_hdmi_hs_neg_d0
    /* synthesis syn_preserve=1 */;
reg        r_hdmi_hs_neg_d1
    /* synthesis PAP_MARK_DEBUG="true" syn_preserve=1 */;

wire [23:0] w_hdmi_capture_data
    /* synthesis PAP_MARK_DEBUG="true" */;
wire        w_hdmi_capture_de;
wire        w_hdmi_capture_vs;
wire        w_hdmi_capture_hs;

/*
 * The selected falling-edge samples first cross into a low-fanout rising-edge
 * handoff stage.  This keeps the half-cycle path local and gives wr_buf a full
 * rising-edge period for RGB, DE, VS and HS together.
 */
reg [23:0] r_hdmi_selected_data
    /* synthesis PAP_MARK_DEBUG="true" syn_preserve=1 */;
reg        r_hdmi_selected_de
    /* synthesis PAP_MARK_DEBUG="true" syn_preserve=1 */;
reg        r_hdmi_selected_vs
    /* synthesis PAP_MARK_DEBUG="true" syn_preserve=1 */;
reg        r_hdmi_selected_hs
    /* synthesis PAP_MARK_DEBUG="true" syn_preserve=1 */;

wire [15:0] hdmi_rgb565 /* synthesis PAP_MARK_DEBUG="true" */;
wire        hdmi_de     /* synthesis PAP_MARK_DEBUG="true" */;
wire        hdmi_vs     /* synthesis PAP_MARK_DEBUG="true" */;
wire        hdmi_hs     /* synthesis PAP_MARK_DEBUG="true" */;

assign hdmi_async_reset_n = rstn_out && init_over;
assign hdmi_video_reset_n = r_hdmi_reset_sync;
assign hdmi_video_reset_n_neg = r_hdmi_reset_sync_neg;

always @(posedge pixclk_in or negedge hdmi_async_reset_n) begin
    if (!hdmi_async_reset_n) begin
        r_hdmi_reset_meta <= 1'b0;
        r_hdmi_reset_sync <= 1'b0;
    end else begin
        r_hdmi_reset_meta <= 1'b1;
        r_hdmi_reset_sync <= r_hdmi_reset_meta;
    end
end

always @(negedge pixclk_in or negedge hdmi_async_reset_n) begin
    if (!hdmi_async_reset_n) begin
        r_hdmi_reset_meta_neg <= 1'b0;
        r_hdmi_reset_sync_neg <= 1'b0;
    end else begin
        r_hdmi_reset_meta_neg <= 1'b1;
        r_hdmi_reset_sync_neg <= r_hdmi_reset_meta_neg;
    end
end

always @(posedge pixclk_in) begin
    if (!hdmi_video_reset_n) begin
        r_hdmi_data_d0 <= 24'd0;
        r_hdmi_data_d1 <= 24'd0;
        r_hdmi_de_d0   <= 1'b0;
        r_hdmi_de_d1   <= 1'b0;
        r_hdmi_vs_d0   <= 1'b0;
        r_hdmi_vs_d1   <= 1'b0;
        r_hdmi_hs_d0   <= 1'b0;
        r_hdmi_hs_d1   <= 1'b0;
    end else begin
        r_hdmi_data_d0 <= {r_in, g_in, b_in};
        r_hdmi_data_d1 <= r_hdmi_data_d0;
        r_hdmi_de_d0   <= de_in;
        r_hdmi_de_d1   <= r_hdmi_de_d0;
        r_hdmi_vs_d0   <= vs_in;
        r_hdmi_vs_d1   <= r_hdmi_vs_d0;
        r_hdmi_hs_d0   <= hs_in;
        r_hdmi_hs_d1   <= r_hdmi_hs_d0;
    end
end

always @(negedge pixclk_in) begin
    if (!hdmi_video_reset_n_neg) begin
        r_hdmi_data_neg_d0 <= 24'd0;
        r_hdmi_data_neg_d1 <= 24'd0;
        r_hdmi_de_neg_d0   <= 1'b0;
        r_hdmi_de_neg_d1   <= 1'b0;
        r_hdmi_vs_neg_d0   <= 1'b0;
        r_hdmi_vs_neg_d1   <= 1'b0;
        r_hdmi_hs_neg_d0   <= 1'b0;
        r_hdmi_hs_neg_d1   <= 1'b0;
    end else begin
        r_hdmi_data_neg_d0 <= {r_in, g_in, b_in};
        r_hdmi_data_neg_d1 <= r_hdmi_data_neg_d0;
        r_hdmi_de_neg_d0   <= de_in;
        r_hdmi_de_neg_d1   <= r_hdmi_de_neg_d0;
        r_hdmi_vs_neg_d0   <= vs_in;
        r_hdmi_vs_neg_d1   <= r_hdmi_vs_neg_d0;
        r_hdmi_hs_neg_d0   <= hs_in;
        r_hdmi_hs_neg_d1   <= r_hdmi_hs_neg_d0;
    end
end

/*
 * Independent per-edge timing monitors.  They never feed control back into
 * the video path, so the two clock edges cannot create a functional CDC.
 * first_bad_line is zero based and 12'hfff means "none in this frame".
 */
reg        r_diag_pos_de_d;
reg        r_diag_pos_vs_d;
reg [12:0] r_diag_pos_line_pixel_count
    /* synthesis PAP_MARK_DEBUG="true" */;
reg [11:0] r_diag_pos_frame_line_count
    /* synthesis PAP_MARK_DEBUG="true" */;
reg [11:0] r_diag_pos_last_frame_line_count
    /* synthesis PAP_MARK_DEBUG="true" */;
reg [15:0] r_diag_pos_short_line_count
    /* synthesis PAP_MARK_DEBUG="true" */;
reg [15:0] r_diag_pos_long_line_count
    /* synthesis PAP_MARK_DEBUG="true" */;
reg [11:0] r_diag_pos_first_bad_line
    /* synthesis PAP_MARK_DEBUG="true" */;
reg [13:0] r_diag_pos_consecutive_good_lines
    /* synthesis PAP_MARK_DEBUG="true" */;

wire w_diag_pos_line_start
    /* synthesis PAP_MARK_DEBUG="true" */;
wire w_diag_pos_line_end
    /* synthesis PAP_MARK_DEBUG="true" */;
wire w_diag_pos_frame_start;

assign w_diag_pos_line_start = r_hdmi_de_d1 && !r_diag_pos_de_d;
assign w_diag_pos_line_end   = !r_hdmi_de_d1 && r_diag_pos_de_d;
assign w_diag_pos_frame_start = !r_hdmi_vs_d1 && r_diag_pos_vs_d;

always @(posedge pixclk_in) begin
    if (!hdmi_video_reset_n) begin
        r_diag_pos_de_d                   <= 1'b0;
        r_diag_pos_vs_d                   <= 1'b0;
        r_diag_pos_line_pixel_count       <= 13'd0;
        r_diag_pos_frame_line_count       <= 12'd0;
        r_diag_pos_last_frame_line_count  <= 12'd0;
        r_diag_pos_short_line_count       <= 16'd0;
        r_diag_pos_long_line_count        <= 16'd0;
        r_diag_pos_first_bad_line         <= 12'hfff;
        r_diag_pos_consecutive_good_lines <= 14'd0;
    end else begin
        r_diag_pos_de_d <= r_hdmi_de_d1;
        r_diag_pos_vs_d <= r_hdmi_vs_d1;

        if (w_diag_pos_frame_start) begin
            r_diag_pos_last_frame_line_count
                <= r_diag_pos_frame_line_count;
            r_diag_pos_frame_line_count <= 12'd0;
            r_diag_pos_line_pixel_count <= 13'd0;
            r_diag_pos_first_bad_line   <= 12'hfff;
        end else if (w_diag_pos_line_start) begin
            r_diag_pos_line_pixel_count <= 13'd1;
        end else if (r_hdmi_de_d1) begin
            if (r_diag_pos_line_pixel_count
                < HDMI_ACTIVE_PIXELS + 1) begin
                r_diag_pos_line_pixel_count
                    <= r_diag_pos_line_pixel_count + 13'd1;
            end
        end else if (w_diag_pos_line_end) begin
            r_diag_pos_line_pixel_count <= 13'd0;
            if (r_diag_pos_frame_line_count != 12'hfff) begin
                r_diag_pos_frame_line_count
                    <= r_diag_pos_frame_line_count + 12'd1;
            end

            if (r_diag_pos_line_pixel_count
                == HDMI_ACTIVE_PIXELS) begin
                if (r_diag_pos_consecutive_good_lines
                    < HDMI_DIAG_GOOD_LIMIT) begin
                    r_diag_pos_consecutive_good_lines
                        <= r_diag_pos_consecutive_good_lines + 14'd1;
                end
            end else begin
                r_diag_pos_consecutive_good_lines <= 14'd0;
                if (r_diag_pos_first_bad_line == 12'hfff) begin
                    r_diag_pos_first_bad_line
                        <= r_diag_pos_frame_line_count;
                end

                if (r_diag_pos_line_pixel_count
                    < HDMI_ACTIVE_PIXELS) begin
                    if (r_diag_pos_short_line_count != 16'hffff) begin
                        r_diag_pos_short_line_count
                            <= r_diag_pos_short_line_count + 16'd1;
                    end
                end else if (r_diag_pos_long_line_count != 16'hffff) begin
                    r_diag_pos_long_line_count
                        <= r_diag_pos_long_line_count + 16'd1;
                end
            end
        end
    end
end

reg        r_diag_neg_de_d;
reg        r_diag_neg_vs_d;
reg [12:0] r_diag_neg_line_pixel_count
    /* synthesis PAP_MARK_DEBUG="true" */;
reg [11:0] r_diag_neg_frame_line_count
    /* synthesis PAP_MARK_DEBUG="true" */;
reg [11:0] r_diag_neg_last_frame_line_count
    /* synthesis PAP_MARK_DEBUG="true" */;
reg [15:0] r_diag_neg_short_line_count
    /* synthesis PAP_MARK_DEBUG="true" */;
reg [15:0] r_diag_neg_long_line_count
    /* synthesis PAP_MARK_DEBUG="true" */;
reg [11:0] r_diag_neg_first_bad_line
    /* synthesis PAP_MARK_DEBUG="true" */;
reg [13:0] r_diag_neg_consecutive_good_lines
    /* synthesis PAP_MARK_DEBUG="true" */;

wire w_diag_neg_line_start
    /* synthesis PAP_MARK_DEBUG="true" */;
wire w_diag_neg_line_end
    /* synthesis PAP_MARK_DEBUG="true" */;
wire w_diag_neg_frame_start;

assign w_diag_neg_line_start =
    r_hdmi_de_neg_d1 && !r_diag_neg_de_d;
assign w_diag_neg_line_end =
    !r_hdmi_de_neg_d1 && r_diag_neg_de_d;
assign w_diag_neg_frame_start =
    !r_hdmi_vs_neg_d1 && r_diag_neg_vs_d;

always @(negedge pixclk_in) begin
    if (!hdmi_video_reset_n_neg) begin
        r_diag_neg_de_d                   <= 1'b0;
        r_diag_neg_vs_d                   <= 1'b0;
        r_diag_neg_line_pixel_count       <= 13'd0;
        r_diag_neg_frame_line_count       <= 12'd0;
        r_diag_neg_last_frame_line_count  <= 12'd0;
        r_diag_neg_short_line_count       <= 16'd0;
        r_diag_neg_long_line_count        <= 16'd0;
        r_diag_neg_first_bad_line         <= 12'hfff;
        r_diag_neg_consecutive_good_lines <= 14'd0;
    end else begin
        r_diag_neg_de_d <= r_hdmi_de_neg_d1;
        r_diag_neg_vs_d <= r_hdmi_vs_neg_d1;

        if (w_diag_neg_frame_start) begin
            r_diag_neg_last_frame_line_count
                <= r_diag_neg_frame_line_count;
            r_diag_neg_frame_line_count <= 12'd0;
            r_diag_neg_line_pixel_count <= 13'd0;
            r_diag_neg_first_bad_line   <= 12'hfff;
        end else if (w_diag_neg_line_start) begin
            r_diag_neg_line_pixel_count <= 13'd1;
        end else if (r_hdmi_de_neg_d1) begin
            if (r_diag_neg_line_pixel_count
                < HDMI_ACTIVE_PIXELS + 1) begin
                r_diag_neg_line_pixel_count
                    <= r_diag_neg_line_pixel_count + 13'd1;
            end
        end else if (w_diag_neg_line_end) begin
            r_diag_neg_line_pixel_count <= 13'd0;
            if (r_diag_neg_frame_line_count != 12'hfff) begin
                r_diag_neg_frame_line_count
                    <= r_diag_neg_frame_line_count + 12'd1;
            end

            if (r_diag_neg_line_pixel_count
                == HDMI_ACTIVE_PIXELS) begin
                if (r_diag_neg_consecutive_good_lines
                    < HDMI_DIAG_GOOD_LIMIT) begin
                    r_diag_neg_consecutive_good_lines
                        <= r_diag_neg_consecutive_good_lines + 14'd1;
                end
            end else begin
                r_diag_neg_consecutive_good_lines <= 14'd0;
                if (r_diag_neg_first_bad_line == 12'hfff) begin
                    r_diag_neg_first_bad_line
                        <= r_diag_neg_frame_line_count;
                end

                if (r_diag_neg_line_pixel_count
                    < HDMI_ACTIVE_PIXELS) begin
                    if (r_diag_neg_short_line_count != 16'hffff) begin
                        r_diag_neg_short_line_count
                            <= r_diag_neg_short_line_count + 16'd1;
                    end
                end else if (r_diag_neg_long_line_count != 16'hffff) begin
                    r_diag_neg_long_line_count
                        <= r_diag_neg_long_line_count + 16'd1;
                end
            end
        end
    end
end

assign w_hdmi_capture_data = HDMI_CAPTURE_NEGEDGE
                           ? r_hdmi_data_neg_d1 : r_hdmi_data_d1;
assign w_hdmi_capture_de   = HDMI_CAPTURE_NEGEDGE
                           ? r_hdmi_de_neg_d1 : r_hdmi_de_d1;
assign w_hdmi_capture_vs   = HDMI_CAPTURE_NEGEDGE
                           ? r_hdmi_vs_neg_d1 : r_hdmi_vs_d1;
assign w_hdmi_capture_hs   = HDMI_CAPTURE_NEGEDGE
                           ? r_hdmi_hs_neg_d1 : r_hdmi_hs_d1;

always @(posedge pixclk_in) begin
    if (!hdmi_video_reset_n) begin
        r_hdmi_selected_data <= 24'd0;
        r_hdmi_selected_de   <= 1'b0;
        r_hdmi_selected_vs   <= 1'b0;
        r_hdmi_selected_hs   <= 1'b0;
    end else begin
        r_hdmi_selected_data <= w_hdmi_capture_data;
        r_hdmi_selected_de   <= w_hdmi_capture_de;
        r_hdmi_selected_vs   <= w_hdmi_capture_vs;
        r_hdmi_selected_hs   <= w_hdmi_capture_hs;
    end
end

assign hdmi_rgb565 = {
    r_hdmi_selected_data[23:19],
    r_hdmi_selected_data[15:10],
    r_hdmi_selected_data[7:3]
};
assign hdmi_de = r_hdmi_selected_de;
assign hdmi_vs = r_hdmi_selected_vs;
assign hdmi_hs = r_hdmi_selected_hs;



/* ------------------------------------------------------------------------- */
/* 720P60 read clock (pix_clk_1080p, name is a historical leftover) reset and
   DDR-init synchronization                                                  */
/* ------------------------------------------------------------------------- */
wire pix_async_reset_n;
wire pix_domain_reset_n;
wire ddr_init_done_pix;
(* ASYNC_REG = "TRUE" *) reg r_pix_reset_meta;
(* ASYNC_REG = "TRUE" *) reg r_pix_reset_sync;
(* ASYNC_REG = "TRUE" *) reg r_ddr_init_pix_meta;
(* ASYNC_REG = "TRUE" *) reg r_ddr_init_pix_sync;

/* 必须放在 pcie_tx_fun 例化之前 */
wire        pclk_div2;
wire        s_pclk_div2_rstn;
wire        ddr_init_done_w;

wire        fb_vout_de;     /* synthesis PAP_MARK_DEBUG="true" */
wire [15:0] fb_vout_data /* synthesis PAP_MARK_DEBUG="true" */;
wire        rd_vs;

/* 2026-10-11: DMA-side video stream after the marker row blanker (declared
   here, BEFORE the pcie_tx_fun instance, because default_nettype none
   forbids use-before-declaration). */
wire [15:0] w_dma_video_data;
wire        w_dma_video_de;
wire        w_dma_video_vs;
wire        rd_de;

assign pix_async_reset_n = rstn_out && pll_rst_n;
assign pix_domain_reset_n = r_pix_reset_sync;
assign ddr_init_done_pix = r_ddr_init_pix_sync;

always @(posedge pix_clk_1080p or negedge pix_async_reset_n) begin
    if (!pix_async_reset_n) begin
        r_pix_reset_meta <= 1'b0;
        r_pix_reset_sync <= 1'b0;
    end else begin
        r_pix_reset_meta <= 1'b1;
        r_pix_reset_sync <= r_pix_reset_meta;
    end
end

always @(posedge pix_clk_1080p) begin
    if (!pix_domain_reset_n) begin
        r_ddr_init_pix_meta <= 1'b0;
        r_ddr_init_pix_sync <= 1'b0;
    end else begin
        r_ddr_init_pix_meta <= ddr_init_done_w;
        r_ddr_init_pix_sync <= r_ddr_init_pix_meta;
    end
end

/* ------------------------------------------------------------------------- */
/* PCIe DMA video packetizer: 1280x720 RGB565, four RK DMA frame slots      */
/* ------------------------------------------------------------------------- */
localparam integer IMG_WIDTH    = 1280;
localparam integer IMG_HEIGHT   = 720;
localparam integer PPC          = 1;
localparam integer PIXCEL_BYTES = 2;
localparam integer DMA_LEN      = 3840;

wire          w_start_flag; /* synthesis PAP_MARK_DEBUG="true" */
wire          w_dma_rd_en;  /* synthesis PAP_MARK_DEBUG="true" */
wire [127:0]  w_dma_rd_data /* synthesis PAP_MARK_DEBUG="true" */;
wire          video_dma_req;        /* synthesis PAP_MARK_DEBUG="true" */
wire [63:0]   video_dma_addr;      /* synthesis PAP_MARK_DEBUG="true" */
wire [9:0]    video_dma_len;        /* synthesis PAP_MARK_DEBUG="true" */
wire          dma_cmd_rdy;          /* synthesis PAP_MARK_DEBUG="true" */
wire          dma_tx_done;          /* synthesis PAP_MARK_DEBUG="true" */
wire          set_dma_config_en;
wire [63:0]   ch0_dma_base_addr;
wire [63:0]   ch0_dma_base_addr2;
wire [63:0]   ch0_dma_base_addr3;
wire [63:0]   ch0_dma_base_addr4;
wire [1:0]    pcie_dma_wr_index;    /* synthesis PAP_MARK_DEBUG="true" */
wire          pcie_dma_wr_done;     /* synthesis PAP_MARK_DEBUG="true" */
wire [31:0]   pcie_check_data;

pcie_tx_fun #(
    .IMG_WIDTH        (IMG_WIDTH),
    .IMG_HEIGHT       (IMG_HEIGHT),
    .DMA_ADDR_WIDTH   (64),
    .VIDEO_DATA_WIDTH (16),
    .PCIE_DATA_WIDTH  (128),
    .PPC              (PPC),
    .PIXCEL_BYTES     (PIXCEL_BYTES),
    .DMA_LEN          (DMA_LEN)
) u_pcie_tx_fun (
    .i_pcie_clk        (pclk_div2),
    .i_pcie_rst_n      (s_pclk_div2_rstn),
    .i_dma_base_addr   (ch0_dma_base_addr),
    .i_dma_base_addr2  (ch0_dma_base_addr2),
    .i_dma_base_addr3  (ch0_dma_base_addr3),
    .i_dma_base_addr4  (ch0_dma_base_addr4),
    // LEGACY dead value kept from the 2560B-packet era: 640 DWORD = 2560 B.
    // The actual per-request length no longer comes from this port -- with
    // DMA_LEN = 3840 (480 packets per frame, 1.5 lines each) r_dma_len is
    // overwritten by the 4 KiB-boundary fragment logic on every request, so
    // this value is only a reset-time seed and is never used in steady state.
    .i_dma_len         (10'd640),
    .i_dma_set_en      (set_dma_config_en),
    .o_dma_wr_index    (pcie_dma_wr_index),
    .o_dma_wr_done     (pcie_dma_wr_done),
    .i_start_tx_flag   (w_start_flag),
    .o_check_data      (pcie_check_data),
    .i_dma_cmd_rdy     (dma_cmd_rdy),
    .o_dma_req         (video_dma_req),
    .o_dma_addr        (video_dma_addr),
    .o_dma_len         (video_dma_len),
    .i_dma_tx_done     (dma_tx_done),
    .i_video_clk       (pix_clk_1080p),
    .i_video_rst_n     (ddr_init_done_pix),
    .i_video_data      (w_dma_video_data),
    .i_video_vs        (w_dma_video_vs),
    .i_video_de        (w_dma_video_de),
    .o_dma_rd_data     (w_dma_rd_data),
    .i_dma_rd_en       (w_dma_rd_en)
);


/* ------------------------------------------------------------------------- */
/* Marker row detection (companion of the 50H side marker_row_add)           */
/* ------------------------------------------------------------------------- */
/*
 * The 50H board replaces line 0 of every displayed frame with a solid colour:
 * 0xFFFF = stitched quad view, 0x0000 = single channel.  Detection sits on the
 * READ side of the frame buffer -- i.e. on exactly the stream that pcie_tx_fun
 * packetizes for the DMA -- so the reported mode can never disagree with the
 * pixels RK3568 actually receives.  It is a pure observer: de / vs / data are
 * not touched and no video timing changes.
 *
 * All four outputs are single-bit levels that change at most once per frame,
 * so each one crosses into the PCIe clock domain through its own two-flop
 * synchronizer.  No multi-bit word ever crosses, so nothing can be sampled
 * half-updated.
 */
localparam [10:0] MK_H_ACT = 11'd1280;
localparam [9:0]  MK_V_ACT = 10'd720;

wire w_mk_mode;
wire w_mk_ok;
wire w_mk_valid;
wire w_mk_tgl;

marker_row_detect #(
    .H_ACT (MK_H_ACT),
    .V_ACT (MK_V_ACT)
) u_marker_row_detect (
    .clk          (pix_clk_1080p),
    .rst_n        (ddr_init_done_pix),
    .data_in      (fb_vout_data),
    .de_in        (fb_vout_de),
    .o_mode       (w_mk_mode),
    .o_marker_ok  (w_mk_ok),
    .o_valid      (w_mk_valid),
    .o_update_tgl (w_mk_tgl)
);

/* ------------------------------------------------------------------------- */
/* Marker row blanker (2026-10-11): unify row 0 to black on the DMA stream   */
/* ------------------------------------------------------------------------- */
/*
 * After the mode has been identified (w_mk_valid = at least one successful
 * detection), line 0 of the stream sent to RK3568 is replaced with black so
 * the marker row no longer shows up on screen.  marker_row_detect keeps
 * watching the ORIGINAL fb_vout stream (blanking it would make every frame
 * look like "single channel"), so the BAR0 0x144 MARKER_STATUS register
 * still reports the true quad/single mode.
 *
 * Consequences for RK side: the in-band row-0 sampler can no longer tell the
 * modes apart (it always sees black) -- the mode must come from 0x144.
 *
 * Timing: same pix_clk_1080p domain, no new clock, no CDC crossing; the only
 * new path is a 16-bit 2:1 mux into the output register (domain WNS is
 * +9 ns, the mux costs ~1 ns).  out_data/de/vs are registered together, so
 * data/timing alignment into pcie_tx_fun is preserved (vs edge shifts by one
 * clock, still ~25 lines before row 0).
 *
 * Fallback: tie blank_en to 0 (or bypass the instance) to restore the raw
 * marker row on the DMA stream.  (w_dma_video_* are declared up in the video
 * wire section, before the pcie_tx_fun instance.)
 */
marker_row_blanker #(
    .H_ACT       (MK_H_ACT),
    .V_ACT       (MK_V_ACT),
    .BLANK_COLOR (16'h0000)
) u_marker_row_blanker (
    .clk      (pix_clk_1080p),
    .rst_n    (ddr_init_done_pix),
    .blank_en (w_mk_valid),        // 置黑仅在成功识别过标识行之后生效
    .in_data  (fb_vout_data),
    .in_de    (fb_vout_de),
    .in_vs    (rd_vs),
    .out_data (w_dma_video_data),
    .out_de   (w_dma_video_de),
    .out_vs   (w_dma_video_vs)
);

/* One independent two-flop synchronizer per bit: every source is a level that
   changes at most once per frame, so a plain 2FF chain is sufficient and
   cannot produce an inconsistent multi-bit value. */
(* ASYNC_REG = "TRUE" *) reg r_mk_mode_meta;
(* ASYNC_REG = "TRUE" *) reg r_mk_mode_sync;
(* ASYNC_REG = "TRUE" *) reg r_mk_ok_meta;
(* ASYNC_REG = "TRUE" *) reg r_mk_ok_sync;
(* ASYNC_REG = "TRUE" *) reg r_mk_valid_meta;
(* ASYNC_REG = "TRUE" *) reg r_mk_valid_sync;
(* ASYNC_REG = "TRUE" *) reg r_mk_tgl_meta;
(* ASYNC_REG = "TRUE" *) reg r_mk_tgl_sync;

always @(posedge pclk_div2) begin
    if (!s_pclk_div2_rstn) begin
        r_mk_mode_meta  <= 1'b0;
        r_mk_mode_sync  <= 1'b0;
        r_mk_ok_meta    <= 1'b0;
        r_mk_ok_sync    <= 1'b0;
        r_mk_valid_meta <= 1'b0;
        r_mk_valid_sync <= 1'b0;
        r_mk_tgl_meta   <= 1'b0;
        r_mk_tgl_sync   <= 1'b0;
    end else begin
        r_mk_mode_meta  <= w_mk_mode;
        r_mk_mode_sync  <= r_mk_mode_meta;
        r_mk_ok_meta    <= w_mk_ok;
        r_mk_ok_sync    <= r_mk_ok_meta;
        r_mk_valid_meta <= w_mk_valid;
        r_mk_valid_sync <= r_mk_valid_meta;
        r_mk_tgl_meta   <= w_mk_tgl;
        r_mk_tgl_sync   <= r_mk_tgl_meta;
    end
end


//----------------------------------------------------------rst debounce ----------------------------------------------------------
localparam  DEVICE_TYPE   = 3'b000  ;//@IPC enum 3'b000,3'b001,3'b100
localparam  AXIS_SLAVE_NUM = 3      ;  //@IPC enum 1 2 3
//TEST UNIT MODE SIGNALS
wire            pcie_cfg_ctrl_en        ;
wire            axis_master_tready_cfg  ;

wire            cfg_axis_slave0_tvalid  ;
wire    [127:0] cfg_axis_slave0_tdata   ;
wire            cfg_axis_slave0_tlast   ;
wire            cfg_axis_slave0_tuser   ;

//for mux
wire            axis_master_tready_mem  ;
wire            axis_master_tvalid_mem  ;
wire    [127:0] axis_master_tdata_mem   ;
wire    [3:0]   axis_master_tkeep_mem   ;
wire            axis_master_tlast_mem   ;
wire    [7:0]   axis_master_tuser_mem   ;

wire            cross_4kb_boundary      ;

wire            dma_axis_slave0_tvalid  ;
wire    [127:0] dma_axis_slave0_tdata   ;
wire            dma_axis_slave0_tlast   ;
wire            dma_axis_slave0_tuser   ;

//RESET DEBOUNCE and SYNC
wire            sync_button_rst_n       ;
wire            sync_perst_n            ;
wire            ref_core_rst_n          ;
wire            s_pclk_rstn             ;

//********************** internal signal
//clk and rst
wire            pclk                    ;
wire            ref_clk                 ;
wire            core_rst_n              ;
//AXIS master interface
wire            axis_master_tvalid      ;
wire            axis_master_tready      ;
wire    [127:0] axis_master_tdata       ;
wire    [3:0]   axis_master_tkeep       ;
wire            axis_master_tlast       ;
wire    [7:0]   axis_master_tuser       ;

//axis slave 0 interface
wire            axis_slave0_tready      ;
wire            axis_slave0_tvalid      ;
wire    [127:0] axis_slave0_tdata       ;
wire            axis_slave0_tlast       ;
wire            axis_slave0_tuser       ;

//axis slave 1 interface
wire            axis_slave1_tready      ;
wire            axis_slave1_tvalid      ;
wire    [127:0] axis_slave1_tdata       ;
wire            axis_slave1_tlast       ;
wire            axis_slave1_tuser       ;

//axis slave 2 interface
wire            axis_slave2_tready      ;
wire            axis_slave2_tvalid      ;
wire    [127:0] axis_slave2_tdata       ;
wire            axis_slave2_tlast       ;
wire            axis_slave2_tuser       ;

wire    [7:0]   cfg_pbus_num            ;
wire    [4:0]   cfg_pbus_dev_num        ;
wire    [2:0]   cfg_max_rd_req_size     ;
wire    [2:0]   cfg_max_payload_size    ;
wire            cfg_rcb                 ;

wire            cfg_ido_req_en          ;
wire            cfg_ido_cpl_en          ;
wire    [7:0]   xadm_ph_cdts            ;
wire    [11:0]  xadm_pd_cdts            ;
wire    [7:0]   xadm_nph_cdts           ;
wire    [11:0]  xadm_npd_cdts           ;
wire    [7:0]   xadm_cplh_cdts          ;
wire    [11:0]  xadm_cpld_cdts          ;

//system signal
wire    [4:0]   smlh_ltssm_state        ;

// led lights up
reg     [22:0]  ref_led_cnt             ;
reg     [26:0]  pclk_led_cnt            ;

//uart2apb 32bits
wire            uart_p_sel              ;
wire    [3:0]   uart_p_strb             ;
wire    [15:0]  uart_p_addr             ;
wire    [31:0]  uart_p_wdata            ;
wire            uart_p_ce               ;
wire            uart_p_we               ;
wire            uart_p_rdy              ;
wire    [31:0]  uart_p_rdata            ;

//apb
wire    [3:0]   p_strb                  ;
wire    [15:0]  p_addr                  ;
wire    [31:0]  p_wdata                 ;
wire            p_ce                    ;
wire            p_we                    ;

//apb mux
wire            p_sel_pcie              ;       //0~5:hsstlp 6:Reserved 7:pcie
wire            p_sel_cfg               ;       //8: cfg
wire            p_sel_dma               ;       //9: dma

wire    [31:0]  p_rdata_pcie            ;       //0~5:hsstlp 6:Reserved 7:pcie
wire    [31:0]  p_rdata_cfg             ;       //8: cfg
wire    [31:0]  p_rdata_dma             ;       //9: dma

wire            p_rdy_pcie              ;       //0~5:hsstlp 6:Reserved 7:pcie
wire            p_rdy_cfg               ;       //8: cfg
wire            p_rdy_dma               ;       //9: dma

wire            start_flag              ;       //开始采集标志

assign cfg_ido_req_en   =   1'b0;
assign cfg_ido_cpl_en   =   1'b0;
assign xadm_ph_cdts     =   8'b0;
assign xadm_pd_cdts     =   12'b0;
assign xadm_nph_cdts    =   8'b0;
assign xadm_npd_cdts    =   12'b0;
assign xadm_cplh_cdts   =   8'b0;
assign xadm_cpld_cdts   =   12'b0;
//ASYNC RST  define IPS2L_PCIE_SPEEDUP_SIM when simulation
hsst_rst_cross_sync_v1_0 #(
    `ifdef IPS2L_PCIE_SPEEDUP_SIM
    .RST_CNTR_VALUE     (16'h10             )
    `else
    .RST_CNTR_VALUE     (16'hC000           )
    `endif
)
u_refclk_buttonrstn_debounce(
    .clk                (ref_clk            ),
    .rstn_in            (button_rst_n       ),
    .rstn_out           (sync_button_rst_n  )
);

hsst_rst_cross_sync_v1_0 #(
    `ifdef IPS2L_PCIE_SPEEDUP_SIM
    .RST_CNTR_VALUE     (16'h10             )
    `else
    .RST_CNTR_VALUE     (16'hC000           )
    `endif
)
u_refclk_perstn_debounce(
    .clk                (ref_clk            ),
    .rstn_in            (perst_n            ),
    .rstn_out           (sync_perst_n       )
);

hsst_rst_sync_v1_0  u_ref_core_rstn_sync    (
    .clk                (ref_clk            ),
    .rst_n              (core_rst_n         ),
    .sig_async          (1'b1               ),
    .sig_synced         (ref_core_rst_n     )
);

hsst_rst_sync_v1_0  u_pclk_core_rstn_sync   (
    .clk                (pclk               ),
    .rst_n              (core_rst_n         ),
    .sig_async          (1'b1               ),
    .sig_synced         (s_pclk_rstn        )
);

hsst_rst_sync_v1_0 u_pclk_div2_core_rstn_sync (
    .clk                (pclk_div2          ),
    .rst_n              (core_rst_n         ),
    .sig_async          (1'b1               ),
    .sig_synced         (s_pclk_div2_rstn   )
);


always @(posedge ref_clk or negedge sync_perst_n)
begin
    if (!sync_perst_n)
        ref_led_cnt    <= 23'd0;
    else
        ref_led_cnt    <= ref_led_cnt + 23'd1;
end

always @(posedge ref_clk or negedge sync_perst_n)
begin
    if (!sync_perst_n)
        ref_led        <= 1'b1;
    else if(&ref_led_cnt)
        ref_led        <= ~ref_led;
end

always @(posedge pclk or negedge s_pclk_rstn)
begin
    if (!s_pclk_rstn)
        pclk_led_cnt    <= 27'd0;
    else
        pclk_led_cnt    <= pclk_led_cnt + 27'd1;
end

always @(posedge pclk or negedge s_pclk_rstn)
begin
    if (!s_pclk_rstn)
        pclk_led        <= 1'b1;
    else if(&pclk_led_cnt)
        pclk_led        <= ~pclk_led;
end


//----------------------------------------------------------   pcie pio  ----------------------------------------------------------
wire            pio_wr_en      ;
wire    [9:0]   pio_wr_addr    ;
wire    [31:0]  pio_wr_data    ;

wire            pio_rd_en      ;        /* synthesis PAP_MARK_DEBUG="true" */
wire    [9:0]   pio_rd_addr    ;        /* synthesis PAP_MARK_DEBUG="true" */
wire    [31:0]  pio_rd_data    ;        /* synthesis PAP_MARK_DEBUG="true" */
pio_crtl u_pio_crtl(
    .pcie_clk              ( pclk_div2         ),
    .rst_n                 ( s_pclk_div2_rstn  ),
    .start_flag            ( w_start_flag      ),
    .set_dma_config_en     ( set_dma_config_en ),
    .o_ch0_base_addr       ( ch0_dma_base_addr ),
    .o_ch0_base_addr2      ( ch0_dma_base_addr2),
    .o_ch0_base_addr3      ( ch0_dma_base_addr3),
    .o_ch0_base_addr4      ( ch0_dma_base_addr4),
    .i_wr_frame_done       ( pcie_dma_wr_done  ),
    .i_wr_index            ( pcie_dma_wr_index ),
    .i_marker_mode         ( r_mk_mode_sync    ),
    .i_marker_ok           ( r_mk_ok_sync      ),
    .i_marker_valid        ( r_mk_valid_sync   ),
    .i_marker_tgl          ( r_mk_tgl_sync     ),
    .pio_wr_en             ( pio_wr_en         ),
    .pio_wr_addr           ( pio_wr_addr       ),
    .pio_wr_data           ( pio_wr_data       ),
    .pio_rd_en             ( pio_rd_en         ),
    .pio_rd_addr           ( pio_rd_addr       ),
    .pio_rd_data           ( pio_rd_data       )
);

//----------------------------------------------------------   pcie dma  ----------------------------------------------------------

// DMA CTRL      BASE ADDR = 0x8000
ips2l_pcie_dma #(
    .DEVICE_TYPE            (DEVICE_TYPE            ),
    .AXIS_SLAVE_NUM         (AXIS_SLAVE_NUM         )
)
u_ips2l_pcie_dma
(
    .clk                    (pclk_div2              ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                  (s_pclk_div2_rstn       ),
    //**********************************************************************
    .i_video_dma_req        (video_dma_req          ),
    .i_video_dma_addr       (video_dma_addr         ),
    .i_video_dma_len        (video_dma_len          ),
    .i_dma_32or64           (1'b1                   ),    // 1: 64-bit MWr
    .o_dma_cmd_rdy          (dma_cmd_rdy            ),
    .o_dma_tx_done          (dma_tx_done            ),
    .o_ch0_base_addr        (      ),
    //dma
    .o_dma_rd_en            (w_dma_rd_en            ),
    .i_dma_rd_data          (w_dma_rd_data          ),
    .o_dma_wr_en            (                       ),
    .o_dma_wr_data          (                       ),
    //pio
    .o_pio_wr_en            (pio_wr_en              ),
    .o_pio_wr_addr          (pio_wr_addr            ),
    .o_pio_wr_data          (pio_wr_data            ),
    .o_pio_rd_en            (pio_rd_en              ),
    .o_pio_rd_addr          (pio_rd_addr            ),
    .o_pio_rd_data          (pio_rd_data            ),
    //**********************************************************************
    //num
    .i_cfg_pbus_num         (cfg_pbus_num           ),  //input [7:0]
    .i_cfg_pbus_dev_num     (cfg_pbus_dev_num       ),  //input [4:0]
    .i_cfg_max_rd_req_size  (cfg_max_rd_req_size    ),  //input [2:0]
    .i_cfg_max_payload_size (cfg_max_payload_size   ),  //input [2:0]
    //**********************************************************************
    //axis master interface
    .i_axis_master_tvld     (axis_master_tvalid_mem ),
    .o_axis_master_trdy     (axis_master_tready_mem ),
    .i_axis_master_tdata    (axis_master_tdata_mem  ),
    .i_axis_master_tkeep    (axis_master_tkeep_mem  ),
    .i_axis_master_tlast    (axis_master_tlast_mem  ),
    .i_axis_master_tuser    (axis_master_tuser_mem  ),

    //**********************************************************************
    //axis_slave0 interface
    .i_axis_slave0_trdy     (axis_slave0_tready     ),
    .o_axis_slave0_tvld     (dma_axis_slave0_tvalid ),
    .o_axis_slave0_tdata    (dma_axis_slave0_tdata  ),
    .o_axis_slave0_tlast    (dma_axis_slave0_tlast  ),
    .o_axis_slave0_tuser    (dma_axis_slave0_tuser  ),
    //axis_slave1 interface
    .i_axis_slave1_trdy     (axis_slave1_tready     ),
    .o_axis_slave1_tvld     (axis_slave1_tvalid     ),
    .o_axis_slave1_tdata    (axis_slave1_tdata      ),
    .o_axis_slave1_tlast    (axis_slave1_tlast      ),
    .o_axis_slave1_tuser    (axis_slave1_tuser      ),
    //axis_slave2 interface
    .i_axis_slave2_trdy     (axis_slave2_tready     ),
    .o_axis_slave2_tvld     (axis_slave2_tvalid     ),
    .o_axis_slave2_tdata    (axis_slave2_tdata      ),
    .o_axis_slave2_tlast    (axis_slave2_tlast      ),
    .o_axis_slave2_tuser    (axis_slave2_tuser      ),
    //from pcie
    .i_cfg_ido_req_en       (cfg_ido_req_en         ),
    .i_cfg_ido_cpl_en       (cfg_ido_cpl_en         ),
    .i_xadm_ph_cdts         (xadm_ph_cdts           ),
    .i_xadm_pd_cdts         (xadm_pd_cdts           ),
    .i_xadm_nph_cdts        (xadm_nph_cdts          ),
    .i_xadm_npd_cdts        (xadm_npd_cdts          ),
    .i_xadm_cplh_cdts       (xadm_cplh_cdts         ),
    .i_xadm_cpld_cdts       (xadm_cpld_cdts         ),
    //**********************************************************************
    //apb interface
    .i_apb_psel             (p_sel_dma              ),
    .i_apb_paddr            (p_addr[8:0]            ),
    .i_apb_pwdata           (p_wdata                ),
    .i_apb_pstrb            (p_strb                 ),
    .i_apb_pwrite           (p_we                   ),
    .i_apb_penable          (p_ce                   ),
    .o_apb_prdy             (p_rdy_dma              ),
    .o_apb_prdata           (p_rdata_dma            ),
    .o_cross_4kb_boundary   (cross_4kb_boundary     )
);

generate
    if (DEVICE_TYPE == 3'd4)
    begin:rc
    //----------------------------------------------------------   cfg ctrl  ----------------------------------------------------------
    //CFG TLP TX RX     BASE ADDR = 0x9000
        pcie_cfg_ctrl   u_pcie_cfg_ctrl(
            //from APB
            .pclk_div2              (pclk_div2              ),
            .apb_rst_n              (s_pclk_div2_rstn       ),
            .p_sel                  (p_sel_cfg              ),
            .p_strb                 (p_strb                 ),
            .p_addr                 (p_addr[7:0]            ),
            .p_wdata                (p_wdata                ),
            .p_ce                   (p_ce                   ),
            .p_we                   (p_we                   ),
            .p_rdy                  (p_rdy_cfg              ),
            .p_rdata                (p_rdata_cfg            ),
            .pcie_cfg_ctrl_en       (pcie_cfg_ctrl_en       ),
            //to PCIE ctrl
            .axis_slave_tready      (axis_slave0_tready     ),
            .axis_slave_tvalid      (cfg_axis_slave0_tvalid ),
            .axis_slave_tlast       (cfg_axis_slave0_tlast  ),
            .axis_slave_tuser       (cfg_axis_slave0_tuser  ),
            .axis_slave_tdata       (cfg_axis_slave0_tdata  ),

            .axis_master_tready     (axis_master_tready_cfg ),
            .axis_master_tvalid     (axis_master_tvalid     ),
            .axis_master_tlast      (axis_master_tlast      ),
        //    .axis_master_tuser      (axis_master_tuser      ),
            .axis_master_tkeep      (axis_master_tkeep      ),
            .axis_master_tdata      (axis_master_tdata      )
        );

        //----------------------------------------------------------   logic mux  ----------------------------------------------------------
        assign axis_slave0_tvalid      = pcie_cfg_ctrl_en ? cfg_axis_slave0_tvalid  : dma_axis_slave0_tvalid;
        assign axis_slave0_tlast       = pcie_cfg_ctrl_en ? cfg_axis_slave0_tlast   : dma_axis_slave0_tlast;
        assign axis_slave0_tuser       = pcie_cfg_ctrl_en ? cfg_axis_slave0_tuser   : dma_axis_slave0_tuser;
        assign axis_slave0_tdata       = pcie_cfg_ctrl_en ? cfg_axis_slave0_tdata   : dma_axis_slave0_tdata;

        assign axis_master_tvalid_mem  = pcie_cfg_ctrl_en ? 1'b0                    : axis_master_tvalid;
        assign axis_master_tdata_mem   = pcie_cfg_ctrl_en ? 128'b0                  : axis_master_tdata;
        assign axis_master_tkeep_mem   = pcie_cfg_ctrl_en ? 4'b0                    : axis_master_tkeep;
        assign axis_master_tlast_mem   = pcie_cfg_ctrl_en ? 1'b0                    : axis_master_tlast;
        assign axis_master_tuser_mem   = pcie_cfg_ctrl_en ? 8'b0                    : axis_master_tuser;

        assign axis_master_tready      = pcie_cfg_ctrl_en ? axis_master_tready_cfg  : axis_master_tready_mem;
    end
    else
    begin:ep
        assign p_rdy_cfg               = 1'b0;
        assign p_rdata_cfg             = 32'b0;

        assign axis_slave0_tvalid      = dma_axis_slave0_tvalid;
        assign axis_slave0_tlast       = dma_axis_slave0_tlast;
        assign axis_slave0_tuser       = dma_axis_slave0_tuser;
        assign axis_slave0_tdata       = dma_axis_slave0_tdata;

        assign axis_master_tvalid_mem  = axis_master_tvalid;
        assign axis_master_tdata_mem   = axis_master_tdata;
        assign axis_master_tkeep_mem   = axis_master_tkeep;
        assign axis_master_tlast_mem   = axis_master_tlast;
        assign axis_master_tuser_mem   = axis_master_tuser;

        assign axis_master_tready      = axis_master_tready_mem;
    end
endgenerate

//----------------------------------------------------------   pcie wrap  ----------------------------------------------------------
//pcie wrap : HSSTLP : 0x0000~6000 PCIe BASE ADDR : 0x7000
pcie_test
u_ips2l_pcie_wrap
(
    .button_rst_n               (sync_button_rst_n      ),
    .power_up_rst_n             (sync_perst_n           ),
    .perst_n                    (sync_perst_n           ),
    //clk and rst
    .pclk                       (pclk                   ),      //output
    .pclk_div2                  (pclk_div2              ),      //output
    .ref_clk                    (ref_clk                ),      //output
    .ref_clk_n                  (ref_clk_n              ),      //input
    .ref_clk_p                  (ref_clk_p              ),      //input
    .core_rst_n                 (core_rst_n             ),      //output

    //APB interface to  DBI cfg
    //.p_clk                      (ref_clk                ),      //input
    .p_sel                      (p_sel_pcie             ),      //input
    .p_strb                     (uart_p_strb            ),      //input  [ 3:0]
    .p_addr                     (uart_p_addr            ),      //input  [15:0]
    .p_wdata                    (uart_p_wdata           ),      //input  [31:0]
    .p_ce                       (uart_p_ce              ),      //input
    .p_we                       (uart_p_we              ),      //input
    .p_rdy                      (p_rdy_pcie             ),      //output
    .p_rdata                    (p_rdata_pcie           ),      //output [31:0]

    //PHY diff signals
    .rxn                        (rxn                    ),      //input   [3:0]
    .rxp                        (rxp                    ),      //input   [3:0]
    .txn                        (txn                    ),      //output  [3:0]
    .txp                        (txp                    ),      //output  [3:0]

    .pcs_nearend_loop           ({2{1'b0}}              ),      //input
    .pma_nearend_ploop          ({2{1'b0}}              ),      //input
    .pma_nearend_sloop          ({2{1'b0}}              ),      //input

    //AXIS master interface
    .axis_master_tvalid         (axis_master_tvalid     ),      //output
    .axis_master_tready         (axis_master_tready     ),      //input
    .axis_master_tdata          (axis_master_tdata      ),      //output [127:0]
    .axis_master_tkeep          (axis_master_tkeep      ),      //output [3:0]
    .axis_master_tlast          (axis_master_tlast      ),      //output
    .axis_master_tuser          (axis_master_tuser      ),      //output [7:0]

    //axis slave 0 interface
    .axis_slave0_tready         (axis_slave0_tready     ),      //output
    .axis_slave0_tvalid         (axis_slave0_tvalid     ),      //input
    .axis_slave0_tdata          (axis_slave0_tdata      ),      //input  [127:0]
    .axis_slave0_tlast          (axis_slave0_tlast      ),      //input
    .axis_slave0_tuser          (axis_slave0_tuser      ),      //input

    //axis slave 1 interface
    .axis_slave1_tready         (axis_slave1_tready     ),      //output
    .axis_slave1_tvalid         (axis_slave1_tvalid     ),      //input
    .axis_slave1_tdata          (axis_slave1_tdata      ),      //input  [127:0]
    .axis_slave1_tlast          (axis_slave1_tlast      ),      //input
    .axis_slave1_tuser          (axis_slave1_tuser      ),      //input
    //axis slave 2 interface
    .axis_slave2_tready         (axis_slave2_tready     ),      //output
    .axis_slave2_tvalid         (axis_slave2_tvalid     ),      //input
    .axis_slave2_tdata          (axis_slave2_tdata      ),      //input  [127:0]
    .axis_slave2_tlast          (axis_slave2_tlast      ),      //input
    .axis_slave2_tuser          (axis_slave2_tuser      ),      //input

    .pm_xtlh_block_tlp          (                       ),      //output

    .cfg_send_cor_err_mux       (                       ),      //output
    .cfg_send_nf_err_mux        (                       ),      //output
    .cfg_send_f_err_mux         (                       ),      //output
    .cfg_sys_err_rc             (                       ),      //output
    .cfg_aer_rc_err_mux         (                       ),      //output
    //radm timeout
    .radm_cpl_timeout           (                       ),      //output

    //configuration signals
    .cfg_max_rd_req_size        (cfg_max_rd_req_size    ),      //output [2:0]
    .cfg_bus_master_en          (                       ),      //output
    .cfg_max_payload_size       (cfg_max_payload_size   ),      //output [2:0]
    .cfg_ext_tag_en             (                       ),      //output
    .cfg_rcb                    (cfg_rcb                ),      //output
    .cfg_mem_space_en           (                       ),      //output
    .cfg_pm_no_soft_rst         (                       ),      //output
    .cfg_crs_sw_vis_en          (                       ),      //output
    .cfg_no_snoop_en            (                       ),      //output
    .cfg_relax_order_en         (                       ),      //output
    .cfg_tph_req_en             (                       ),      //output [2-1:0]
    .cfg_pf_tph_st_mode         (                       ),      //output [3-1:0]
    .rbar_ctrl_update           (                       ),      //output
    .cfg_atomic_req_en          (                       ),      //output

    .cfg_pbus_num               (cfg_pbus_num           ),      //output [7:0]
    .cfg_pbus_dev_num           (cfg_pbus_dev_num       ),      //output [4:0]

    //debug signals
    .radm_idle                  (                       ),      //output
    .radm_q_not_empty           (                       ),      //output
    .radm_qoverflow             (                       ),      //output
    .diag_ctrl_bus              (2'b0                   ),      //input   [1:0]
    .cfg_link_auto_bw_mux       (                       ),      //output              merge cfg_link_auto_bw_msi and cfg_link_auto_bw_int
    .cfg_bw_mgt_mux             (                       ),      //output              merge cfg_bw_mgt_int and cfg_bw_mgt_msi
    .cfg_pme_mux                (                       ),      //output              merge cfg_pme_int and cfg_pme_msi
    .app_ras_des_sd_hold_ltssm  (1'b0                   ),      //input
    .app_ras_des_tba_ctrl       (2'b0                   ),      //input   [1:0]

    .dyn_debug_info_sel         (4'b0                   ),      //input   [3:0]
    .debug_info_mux             (                       ),      //output  [132:0]

    //system signal
    .smlh_link_up               (smlh_link_up           ),      //output
    .rdlh_link_up               (rdlh_link_up           ),      //output
    .smlh_ltssm_state           (smlh_ltssm_state       )       //output  [4:0]
);





/////////////////////////////////////////////////////////////////////////////////////
// DDR3 Frame Buffer (fram_buf + ddr3 from HDMI_IN_DDR3_bin reference)
/////////////////////////////////////////////////////////////////////////////////////
    localparam integer MEM_ROW_WIDTH  = 15;
    localparam integer MEM_COL_WIDTH  = 10;
    localparam integer MEM_BANK_WIDTH = 3;
    localparam integer MEM_DQ_WIDTH   = 16;
    localparam integer CTRL_ADDR_W    = 28;
    localparam integer H_NUM          = 1280;
    localparam integer V_NUM          = 720;
    localparam integer PIX_W          = 16;

    wire        ddr_core_clk;
    wire        fb_init_done_ddr;   /* synthesis PAP_MARK_DEBUG="true" */

    // AXI bus
    wire [CTRL_ADDR_W-1:0]   ddr_axi_awaddr, ddr_axi_araddr;
    wire [3:0]               ddr_axi_awid, ddr_axi_awlen;
    wire [3:0]               ddr_axi_arid, ddr_axi_arlen;
    wire                     ddr_axi_awready, ddr_axi_awvalid;
    wire                     ddr_axi_wready, ddr_axi_wusero_last;
    wire                     ddr_axi_arready, ddr_axi_arvalid;
    wire                     ddr_axi_rvalid, ddr_axi_rlast;
    wire [3:0]               ddr_axi_rid;
    wire [MEM_DQ_WIDTH*8-1:0] ddr_axi_wdata, ddr_axi_rdata;
    wire [MEM_DQ_WIDTH-1:0]   ddr_axi_wstrb;

    // fram_buf: HDMI-IN write at pixclk_in, 720p60 read at pix_clk_1080p
    wire [15:0] fb_wr_data;     /* synthesis PAP_MARK_DEBUG="true" */
    wire        fb_wr_fsync;    /* synthesis PAP_MARK_DEBUG="true" */
    wire        fb_wr_en;       /* synthesis PAP_MARK_DEBUG="true" */
    assign      fb_wr_data  = hdmi_rgb565;
    assign      fb_wr_en    = hdmi_de;
    assign      fb_wr_fsync = ~hdmi_vs;

    fram_buf #(
        .H_NUM(H_NUM), .V_NUM(V_NUM), .PIX_WIDTH(PIX_W),
        .MEM_ROW_WIDTH(MEM_ROW_WIDTH), .MEM_COLUMN_WIDTH(MEM_COL_WIDTH),
        .MEM_BANK_WIDTH(MEM_BANK_WIDTH), .MEM_DQ_WIDTH(MEM_DQ_WIDTH),
        .INTERLACED_INPUT(0)
    ) u_fram_buf (
        .vin_clk    (pixclk_in),
        .wr_fsync   (fb_wr_fsync),
        .wr_en      (fb_wr_en),
        .wr_data    (fb_wr_data),
        .ddr_clk    (ddr_core_clk),
        .ddr_rstn   (ddr_init_done_w),
        .vout_clk   (pix_clk_1080p),
        .rd_fsync   (rd_vs),
        .rd_en      (rd_de),
        .vout_de    (fb_vout_de),
        .vout_data  (fb_vout_data),
        .init_done  (fb_init_done_ddr),
        .axi_awaddr (ddr_axi_awaddr),
        .axi_awid   (ddr_axi_awid),
        .axi_awlen  (ddr_axi_awlen),
        .axi_awsize (),
        .axi_awburst(),
        .axi_awready(ddr_axi_awready),
        .axi_awvalid(ddr_axi_awvalid),
        .axi_wdata  (ddr_axi_wdata),
        .axi_wstrb  (ddr_axi_wstrb),
        .axi_wlast  (ddr_axi_wusero_last),
        .axi_wvalid (),
        .axi_wready (ddr_axi_wready),
        .axi_bid    (4'd0),
        .axi_araddr (ddr_axi_araddr),
        .axi_arid   (ddr_axi_arid),
        .axi_arlen  (ddr_axi_arlen),
        .axi_arsize (),
        .axi_arburst(),
        .axi_arvalid(ddr_axi_arvalid),
        .axi_arready(ddr_axi_arready),
        .axi_rready (),
        .axi_rdata  (ddr_axi_rdata),
        .axi_rvalid (ddr_axi_rvalid),
        .axi_rlast  (ddr_axi_rlast),
        .axi_rid    (ddr_axi_rid)
    );

    // sync_vg: generate read-side VS/DE timing
    sync_vg #(
        .H_TOTAL (1650),
        .H_FP    (110),
        .H_BP    (220),
        .H_SYNC  (40),
        .H_ACT   (1280),
        .V_TOTAL (750),
        .V_FP    (5),
        .V_BP    (20),
        .V_SYNC  (5),
        .V_ACT   (720)
    ) u_sync_vg (
        .clk    (pix_clk_1080p),
        .rstn   (ddr_init_done_pix),
        .vs_out (rd_vs),
        .hs_out (),
        .de_out (rd_de),
        .de_re  (),
        .x_act  (),
        .y_act  ()
    );

    /* rstn_out is generated in cfg_clk.  Assert reset asynchronously and
       release it through two ref_clk flip-flops before driving the DDR IP. */
    wire ddr_ref_reset_n;
    (* ASYNC_REG = "TRUE" *) reg r_ddr_ref_reset_meta;
    (* ASYNC_REG = "TRUE" *) reg r_ddr_ref_reset_sync;

    always @(posedge ref_clk or negedge rstn_out) begin
        if (!rstn_out) begin
            r_ddr_ref_reset_meta <= 1'b0;
            r_ddr_ref_reset_sync <= 1'b0;
        end else begin
            r_ddr_ref_reset_meta <= 1'b1;
            r_ddr_ref_reset_sync <= r_ddr_ref_reset_meta;
        end
    end

    assign ddr_ref_reset_n = r_ddr_ref_reset_sync;

   // ddr3: PG2L100H DDR3 Controller
   ddr3_demo #(
       .MEM_ROW_WIDTH(MEM_ROW_WIDTH), .MEM_COLUMN_WIDTH(MEM_COL_WIDTH),
       .MEM_BANK_WIDTH(MEM_BANK_WIDTH), .MEM_DQ_WIDTH(MEM_DQ_WIDTH)
   ) u_ddr3 (
       .ref_clk               (ddr_ref_clk_125m),
       .resetn                (ddr_ref_reset_n),
       .core_clk              (ddr_core_clk),
       .pll_lock              (ddr_pll_lock),
       .phy_pll_lock          (),
       .gpll_lock             (),
       .rst_gpll_lock         (),
       .ddrphy_cpd_lock       (),
       .ddr_init_done         (ddr_init_done_w),
       .axi_awaddr            (ddr_axi_awaddr),
       .axi_awuser_ap         (1'b0),
       .axi_awuser_id         (ddr_axi_awid),
       .axi_awlen             (ddr_axi_awlen),
       .axi_awready           (ddr_axi_awready),
       .axi_awvalid           (ddr_axi_awvalid),
       .axi_wdata             (ddr_axi_wdata),
       .axi_wstrb             (ddr_axi_wstrb),
       .axi_wready            (ddr_axi_wready),
       .axi_wusero_id         (),
       .axi_wusero_last       (ddr_axi_wusero_last),
       .axi_araddr            (ddr_axi_araddr),
       .axi_aruser_ap         (1'b0),
       .axi_aruser_id         (ddr_axi_arid),
       .axi_arlen             (ddr_axi_arlen),
       .axi_arready           (ddr_axi_arready),
       .axi_arvalid           (ddr_axi_arvalid),
       .axi_rdata             (ddr_axi_rdata),
       .axi_rid               (ddr_axi_rid),
       .axi_rlast             (ddr_axi_rlast),
       .axi_rvalid            (ddr_axi_rvalid),
       .apb_clk               (1'b0),
       .apb_rst_n             (1'b1),
       .apb_sel               (1'b0),
       .apb_enable            (1'b0),
       .apb_addr              (8'b0),
       .apb_write             (1'b0),
       .apb_ready             (),
       .apb_wdata             (16'b0),
       .apb_rdata             (),
       .mem_rst_n             (mem_rst_n),
       .mem_ck                (mem_ck),
       .mem_ck_n              (mem_ck_n),
       .mem_cke               (mem_cke),
       .mem_cs_n              (mem_cs_n),
       .mem_ras_n             (mem_ras_n),
       .mem_cas_n             (mem_cas_n),
       .mem_we_n              (mem_we_n),
       .mem_odt               (mem_odt),
       .mem_a                 (mem_a),
       .mem_ba                (mem_ba),
       .mem_dqs               (mem_dqs),
       .mem_dqs_n             (mem_dqs_n),
       .mem_dq                (mem_dq),
       .mem_dm                (mem_dm),
       .dbg_gate_start        (1'b0),
       .dbg_cpd_start         (1'b0),
       .dbg_ddrphy_rst_n      (1'b1),
       .dbg_gpll_scan_rst     (1'b0),
       .samp_position_dyn_adj (1'b0),
       .init_samp_position_even(32'd0),
       .init_samp_position_odd (32'd0),
       .wrcal_position_dyn_adj(1'b0),
       .init_wrcal_position   (32'd0),
       .force_read_clk_ctrl   (1'b0),
       .init_slip_step        (16'd0),
       .init_read_clk_ctrl    (12'd0),
       .debug_calib_ctrl      (),
       .dbg_slice_status      (),
       .dbg_slice_state       (),
       .debug_data            (),
       .dbg_dll_upd_state     (),
       .debug_gpll_dps_phase  (),
       .dbg_rst_dps_state     (),
       .dbg_tran_err_rst_cnt  (),
       .dbg_ddrphy_init_fail  (),
       .debug_cpd_offset_adj  (1'b0),
       .debug_cpd_offset_dir  (1'b0),
       .debug_cpd_offset      (10'd0),
       .debug_dps_cnt_dir0    (),
       .debug_dps_cnt_dir1    (),
       .ck_dly_en             (1'b0),
       .init_ck_dly_step      (8'h05),
       .ck_dly_set_bin        (),
       .align_error           (),
       .debug_rst_state       (),
       .debug_cpd_state       ()
   );

/////////////////////////////////////////////////////////////////////////////////////
endmodule

`default_nettype wire
