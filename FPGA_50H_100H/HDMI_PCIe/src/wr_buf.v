`timescale 1ns / 1ps
`default_nettype none

/*
 * Video-line writer for the DDR frame buffer.
 *
 * wr_clk and ddr_clk are asynchronous.  Only event toggles cross between
 * them, through two synchronizer flip-flops.  Pixel data crosses through the
 * vendor dual-clock RAM wr_fram_buf.
 */
module wr_buf #(
    parameter integer ADDR_WIDTH      = 27,
    parameter integer ADDR_OFFSET     = 0,
    parameter integer H_NUM           = 1280,
    parameter integer V_NUM           = 720,
    parameter integer DQ_WIDTH        = 16,
    parameter integer LEN_WIDTH       = 32,
    parameter integer PIX_WIDTH       = 16,
    parameter integer LINE_ADDR_WIDTH = 22,
    parameter integer FRAME_CNT_WIDTH = 6,
    parameter integer INTERLACED_INPUT = 0
)(
    input  wire                         ddr_clk,
    input  wire                         ddr_rstn,

    input  wire                         wr_clk,
    input  wire                         wr_fsync,
    input  wire                         wr_en,
    input  wire [PIX_WIDTH-1:0]         wr_data,

    input  wire                         rd_bac,
    input  wire                         active_read_bank,
    input  wire                         active_read_valid,
    output wire                         ddr_wreq,
    output wire [ADDR_WIDTH-1:0]        ddr_waddr,
    output wire [LEN_WIDTH-1:0]         ddr_wr_len,
    input  wire                         ddr_wrdy,
    input  wire                         ddr_wdone,
    output wire [8*DQ_WIDTH-1:0]        ddr_wdata,
    input  wire                         ddr_wdata_req,

    output wire [FRAME_CNT_WIDTH-1:0]   frame_wcnt,
    output wire                         frame_wirq
);

    localparam integer RAM_WIDTH        = 32;
    localparam integer DDR_DATA_WIDTH   = DQ_WIDTH * 8;
    localparam integer WR_LINE_NUM      = H_NUM * PIX_WIDTH / RAM_WIDTH;
    localparam integer RD_LINE_NUM      = WR_LINE_NUM * RAM_WIDTH
                                        / DDR_DATA_WIDTH;
    localparam integer DDR_ADDR_OFFSET  = RD_LINE_NUM * DDR_DATA_WIDTH
                                        / DQ_WIDTH;
    /*
     * INTERLACED_INPUT = 1:
     *   two V_NUM/2-line fields are interleaved into one V_NUM-line frame.
     *
     * INTERLACED_INPUT = 0:
     *   each input VSYNC already contains one complete V_NUM-line progressive
     *   frame and DDR addresses advance by one output-line stride.
     */
    localparam integer FIELD_LINE_NUM =
        (INTERLACED_INPUT != 0) ? (V_NUM / 2) : V_NUM;
    /*
     * DDR_ADDR_OFFSET is one output-line stride.  Progressive input advances
     * by one stride per line.  When INTERLACED_INPUT is enabled, field 0 is
     * written to even output lines and field 1 to odd output lines, so two
     * strides are used between consecutive lines in the same field.
     */
    localparam integer DDR_FIELD_LINE_STRIDE =
        (INTERLACED_INPUT != 0)
        ? (DDR_ADDR_OFFSET * 2) : DDR_ADDR_OFFSET;
    localparam integer RAM_RD_ADDR_WIDTH = (DDR_DATA_WIDTH == 128) ? 10 : 9;

    /* rd_bac and ddr_wrdy are retained for interface compatibility. */
    wire w_unused_inputs;
    assign w_unused_inputs = rd_bac | ddr_wrdy;

    /* --------------------------------------------------------------------- */
    /* Reset synchronization into wr_clk                                     */
    /* --------------------------------------------------------------------- */
    (* ASYNC_REG = "TRUE" *) reg r_wr_reset_meta;
    (* ASYNC_REG = "TRUE" *) reg r_wr_reset_sync;

    always @(posedge wr_clk or negedge ddr_rstn) begin
        if (!ddr_rstn) begin
            r_wr_reset_meta <= 1'b0;
            r_wr_reset_sync <= 1'b0;
        end else begin
            r_wr_reset_meta <= 1'b1;
            r_wr_reset_sync <= r_wr_reset_meta;
        end
    end

    /* --------------------------------------------------------------------- */
    /* wr_clk domain: pack valid pixels into 32-bit words                    */
    /* --------------------------------------------------------------------- */
    reg                         r_wr_fsync_d;
    reg                         r_wr_en_d;
    reg                         r_wr_enable;    /* synthesis PAP_MARK_DEBUG="true" */
    reg [11:0]                  r_x_cnt;        /* synthesis PAP_MARK_DEBUG="true" */
    reg [11:0]                  r_y_cnt;        /* synthesis PAP_MARK_DEBUG="true" */
    reg [PIX_WIDTH-1:0]         r_wr_data_d;
    reg [31:0]                  r_write_data;
    reg                         r_write_en;
    reg [11:0]                  r_wr_addr;
    reg [11:0]                  r_line_word_count
        /* synthesis PAP_MARK_DEBUG="true" */;
    reg                         r_line_event_toggle;
    reg                         r_line_event_pending;
    /*
     * r_frame_event_toggle is the only field-boundary state that crosses to
     * ddr_clk.  Its synchronized new level carries the phase as well:
     * level 1 starts field 0 and level 0 starts field 1.  Event and phase can
     * therefore never disagree because they come from the same synchronized
     * flip-flop rather than two independent CDC paths.
     */
    reg                         r_frame_event_toggle;
    reg                         r_half_frame_phase
        /* synthesis PAP_MARK_DEBUG="true" */;
    reg [11:0]                  r_field_line_count
        /* synthesis PAP_MARK_DEBUG="true" */;
    reg [11:0]                  r_last_field_line_count
        /* synthesis PAP_MARK_DEBUG="true" */;
    reg                         r_wr_field_line_error
        /* synthesis PAP_MARK_DEBUG="true" */;
    reg                         r_seen_frame_start;
    reg                         r_frame_invalid
        /* synthesis PAP_MARK_DEBUG="true" */;
    reg                         r_short_line_error
        /* synthesis PAP_MARK_DEBUG="true" */;
    reg                         r_long_line_error
        /* synthesis PAP_MARK_DEBUG="true" */;
    reg                         r_line_word_count_error
        /* synthesis PAP_MARK_DEBUG="true" */;
    reg [11:0]                  r_first_bad_line
        /* synthesis PAP_MARK_DEBUG="true" */;

    wire w_wr_frame_start;      /* synthesis PAP_MARK_DEBUG="true" */
    wire w_full_frame_start;    /* synthesis PAP_MARK_DEBUG="true" */
    wire line_start             /* synthesis PAP_MARK_DEBUG="true" */;
    wire line_end               /* synthesis PAP_MARK_DEBUG="true" */;
    wire [11:0] line_pixel_count
                                    /* synthesis PAP_MARK_DEBUG="true" */;
    wire [11:0] line_word_count
                                    /* synthesis PAP_MARK_DEBUG="true" */;
    wire line_valid             /* synthesis PAP_MARK_DEBUG="true" */;
    wire short_line_error       /* synthesis PAP_MARK_DEBUG="true" */;
    wire long_line_error        /* synthesis PAP_MARK_DEBUG="true" */;
    wire line_word_count_error
                                    /* synthesis PAP_MARK_DEBUG="true" */;
    wire frame_invalid          /* synthesis PAP_MARK_DEBUG="true" */;
    wire [11:0] first_bad_line
                                    /* synthesis PAP_MARK_DEBUG="true" */;
    wire w_valid_pixel;         /* synthesis PAP_MARK_DEBUG="true" */
    wire w_line_ram_wr_en;      /* synthesis PAP_MARK_DEBUG="true" */
    wire w_line_pixel_done;
    wire w_line_commit;
    wire w_line_shape_valid;
    wire w_line_error;
    wire w_too_many_lines;

    assign w_wr_frame_start = wr_fsync && !r_wr_fsync_d;
    assign w_full_frame_start =
        w_wr_frame_start
        && ((INTERLACED_INPUT == 0) || !r_half_frame_phase);
    assign line_start = wr_en && !r_wr_en_d;
    assign line_end   = !wr_en && r_wr_en_d;
    assign line_pixel_count = r_x_cnt;
    assign line_word_count  = r_line_word_count;
    assign short_line_error = r_short_line_error;
    assign long_line_error  = r_long_line_error;
    assign line_word_count_error = r_line_word_count_error;
    assign frame_invalid    = r_frame_invalid;
    assign first_bad_line   = r_first_bad_line;

    /*
     * Once H_NUM pixels have been accepted, stop the packer at the limit.
     * r_x_cnt itself may advance once more and then saturates, making a long
     * DE pulse distinguishable without allowing it to wrap into another line.
     */
    assign w_valid_pixel =
        wr_en
        && r_wr_enable
        && !r_frame_invalid
        && (r_x_cnt < H_NUM);
    assign w_line_ram_wr_en =
        r_write_en
        && !w_wr_frame_start
        && r_wr_enable
        && !r_frame_invalid;
    assign w_line_pixel_done =
        w_valid_pixel && (r_x_cnt == H_NUM - 1);
    assign w_line_shape_valid =
        (r_x_cnt == H_NUM)
        && r_line_event_pending
        && w_line_ram_wr_en
        && (r_line_word_count == WR_LINE_NUM - 1);
    assign w_too_many_lines =
        r_field_line_count >= FIELD_LINE_NUM;
    assign w_line_commit =
        line_end
        && r_wr_enable
        && !r_frame_invalid
        && !w_too_many_lines
        && w_line_shape_valid;
    assign line_valid = w_line_commit;
    assign w_line_error =
        line_end
        && r_wr_enable
        && !r_frame_invalid
        && !w_line_commit;

    always @(posedge wr_clk) begin
        if (!r_wr_reset_sync) begin
            r_wr_fsync_d          <= 1'b0;
            r_wr_en_d             <= 1'b0;
            r_wr_enable           <= 1'b0;
            r_frame_event_toggle  <= 1'b0;
            r_half_frame_phase     <= 1'b0;
        end else begin
            r_wr_fsync_d <= wr_fsync;
            r_wr_en_d    <= wr_en;
            if (w_wr_frame_start) begin
                r_wr_enable          <= 1'b1;
                r_frame_event_toggle <= ~r_frame_event_toggle;
                if (INTERLACED_INPUT == 0) begin
                    r_half_frame_phase <= 1'b0;
                end else if (!r_half_frame_phase) begin
                    r_half_frame_phase <= 1'b1;
                end else begin
                    r_half_frame_phase <= 1'b0;
                end
            end else if (w_line_error) begin
                r_wr_enable <= 1'b0;
            end
        end
    end

    /*
     * Count only legal physical lines.  All status in this block remains in
     * wr_clk; no multi-bit counter crosses the clock boundary.
     */
    always @(posedge wr_clk) begin
        if (!r_wr_reset_sync) begin
            r_field_line_count      <= 12'd0;
            r_last_field_line_count <= 12'd0;
            r_wr_field_line_error   <= 1'b0;
            r_seen_frame_start      <= 1'b0;
            r_frame_invalid         <= 1'b0;
            r_short_line_error      <= 1'b0;
            r_long_line_error       <= 1'b0;
            r_line_word_count_error <= 1'b0;
            r_first_bad_line        <= 12'hfff;
        end else if (w_wr_frame_start) begin
            r_last_field_line_count <= r_field_line_count;
            if (r_seen_frame_start
                && (r_frame_invalid
                    || (r_field_line_count != FIELD_LINE_NUM))) begin
                r_wr_field_line_error <= 1'b1;
            end
            r_seen_frame_start  <= 1'b1;
            r_frame_invalid     <= 1'b0;
            r_first_bad_line    <= 12'hfff;
            r_field_line_count  <= 12'd0;
        end else if (w_line_error) begin
            r_frame_invalid <= 1'b1;
            if (r_first_bad_line == 12'hfff) begin
                r_first_bad_line <= r_field_line_count;
            end

            if (r_x_cnt < H_NUM) begin
                r_short_line_error <= 1'b1;
            end else if (r_x_cnt > H_NUM) begin
                r_long_line_error <= 1'b1;
            end else if (!w_line_shape_valid) begin
                r_line_word_count_error <= 1'b1;
            end

            if (w_too_many_lines) begin
                r_wr_field_line_error <= 1'b1;
            end
        end else if (w_line_commit) begin
            if (r_field_line_count != 12'hfff) begin
                r_field_line_count <= r_field_line_count + 12'd1;
            end
        end
    end

    always @(posedge wr_clk) begin
        if (!r_wr_reset_sync) begin
            r_x_cnt <= 12'd0;
            r_y_cnt <= 12'd0;
        end else if (w_wr_frame_start) begin
            r_x_cnt <= 12'd0;
            if (w_full_frame_start) begin
                r_y_cnt <= 12'd0;
            end
        end else begin
            if (line_start && r_wr_enable && !r_frame_invalid) begin
                r_x_cnt <= 12'd1;
            end else if (wr_en && r_wr_enable && !r_frame_invalid) begin
                if (r_x_cnt < H_NUM + 1) begin
                    r_x_cnt <= r_x_cnt + 12'd1;
                end
            end else if (!wr_en) begin
                r_x_cnt <= 12'd0;
            end

            if (w_line_commit) begin
                if (r_y_cnt < V_NUM - 1) begin
                    r_y_cnt <= r_y_cnt + 12'd1;
                end else begin
                    r_y_cnt <= r_y_cnt;
                end
            end
        end
    end

    /*
     * Count actual 32-bit RAM writes within the current DE pulse.  On the
     * line_end cycle, w_line_ram_wr_en is the delayed write for the final
     * packed word, so validation above expects WR_LINE_NUM-1 prior writes.
     */
    always @(posedge wr_clk) begin
        if (!r_wr_reset_sync || w_wr_frame_start) begin
            r_line_word_count <= 12'd0;
        end else if (line_start || line_end || r_frame_invalid) begin
            r_line_word_count <= 12'd0;
        end else if (w_line_ram_wr_en) begin
            if (r_line_word_count < WR_LINE_NUM) begin
                r_line_word_count <= r_line_word_count + 12'd1;
            end else begin
                r_line_word_count <= r_line_word_count;
            end
        end
    end

    generate
        if (PIX_WIDTH == 24) begin : g_pack_rgb888
            always @(posedge wr_clk) begin
                if (!r_wr_reset_sync || w_wr_frame_start) begin
                    r_wr_data_d   <= {PIX_WIDTH{1'b0}};
                    r_write_data <= 32'd0;
                    r_write_en   <= 1'b0;
                end else if (w_valid_pixel) begin
                    r_wr_data_d <= wr_data;
                    r_write_en <= (r_x_cnt[1:0] != 2'd0);
                    if (r_x_cnt[1:0] == 2'd1) begin
                        r_write_data <= {wr_data[7:0], r_wr_data_d};
                    end else if (r_x_cnt[1:0] == 2'd2) begin
                        r_write_data <= {wr_data[15:0],
                                         r_wr_data_d[PIX_WIDTH-1:8]};
                    end else if (r_x_cnt[1:0] == 2'd3) begin
                        r_write_data <= {wr_data,
                                         r_wr_data_d[PIX_WIDTH-1:16]};
                    end
                end else begin
                    r_write_en <= 1'b0;
                end
            end
        end else if (PIX_WIDTH == 16) begin : g_pack_rgb565
            always @(posedge wr_clk) begin
                if (!r_wr_reset_sync || w_wr_frame_start) begin
                    r_wr_data_d   <= {PIX_WIDTH{1'b0}};
                    r_write_data <= 32'd0;
                    r_write_en   <= 1'b0;
                end else if (w_valid_pixel) begin
                    r_wr_data_d <= wr_data;
                    r_write_en <= r_x_cnt[0];
                    if (r_x_cnt[0]) begin
                        r_write_data <= {wr_data, r_wr_data_d};
                    end
                end else begin
                    r_write_en <= 1'b0;
                end
            end
        end else begin : g_pack_native
            always @(posedge wr_clk) begin
                if (!r_wr_reset_sync || w_wr_frame_start) begin
                    r_wr_data_d   <= {PIX_WIDTH{1'b0}};
                    r_write_data <= 32'd0;
                    r_write_en   <= 1'b0;
                end else begin
                    r_wr_data_d   <= wr_data;
                    r_write_data <= wr_data;
                    r_write_en   <= w_valid_pixel;
                end
            end
        end
    endgenerate

    always @(posedge wr_clk) begin
        if (!r_wr_reset_sync || w_wr_frame_start) begin
            r_wr_addr <= 12'd0;
        end else if (w_line_ram_wr_en) begin
            r_wr_addr <= r_wr_addr + 12'd1;
        end
    end

    /* Pixel data is transported by a true dual-clock RAM, not by ordinary
       synchronizer registers.  For DQ_WIDTH=16 this IP must be configured as
       32-bit write / 128-bit read, with 12-bit / 10-bit addresses. */
    reg  [RAM_RD_ADDR_WIDTH-1:0] r_ram_rd_addr;
    wire [DDR_DATA_WIDTH-1:0]    w_ram_rd_data;

    wr_fram_buf u_wr_fram_buf (
        .wr_data (r_write_data),
        .wr_addr (r_wr_addr),
        .wr_en   (w_line_ram_wr_en),
        .wr_clk  (wr_clk),
        .wr_rst  (~r_wr_reset_sync),

        .rd_addr (r_ram_rd_addr),
        .rd_data (w_ram_rd_data),
        .rd_clk  (ddr_clk),
        .rd_rst  (~ddr_rstn)
    );

    /* --------------------------------------------------------------------- */
    /* ddr_clk domain: synchronize event toggles and manage bank ownership   */
    /* --------------------------------------------------------------------- */
    (* ASYNC_REG = "TRUE" *) reg r_line_toggle_meta;
    (* ASYNC_REG = "TRUE" *) reg r_line_toggle_sync;
    reg                            r_line_toggle_d;
    (* ASYNC_REG = "TRUE" *) reg r_frame_toggle_meta;
    (* ASYNC_REG = "TRUE" *) reg r_frame_toggle_sync;
    reg                            r_frame_toggle_d;

    wire w_line_event;       /* synthesis PAP_MARK_DEBUG="true" */
    wire w_frame_event;      /* synthesis PAP_MARK_DEBUG="true" */
    wire w_full_frame_event; /* synthesis PAP_MARK_DEBUG="true" */
    wire w_second_field_event;

    assign w_line_event  = r_line_toggle_sync ^ r_line_toggle_d;
    assign w_frame_event = r_frame_toggle_sync ^ r_frame_toggle_d;
    assign w_full_frame_event =
        w_frame_event
        && ((INTERLACED_INPUT == 0) || r_frame_toggle_sync);
    assign w_second_field_event =
        (INTERLACED_INPUT != 0)
        && w_frame_event
        && !r_frame_toggle_sync;

    always @(posedge ddr_clk) begin
        if (!ddr_rstn) begin
            r_line_toggle_meta  <= 1'b0;
            r_line_toggle_sync  <= 1'b0;
            r_line_toggle_d     <= 1'b0;
            r_frame_toggle_meta <= 1'b0;
            r_frame_toggle_sync <= 1'b0;
            r_frame_toggle_d    <= 1'b0;
        end else begin
            r_line_toggle_meta  <= r_line_event_toggle;
            r_line_toggle_sync  <= r_line_toggle_meta;
            r_line_toggle_d     <= r_line_toggle_sync;
            r_frame_toggle_meta <= r_frame_event_toggle;
            r_frame_toggle_sync <= r_frame_toggle_meta;
            r_frame_toggle_d    <= r_frame_toggle_sync;
        end
    end

    /*
     * Fail-closed DDR field writer.
     *
     * A frame/field boundary is accepted only while no line transaction is
     * pending.  In interlaced mode, field 0 must have exactly FIELD_LINE_NUM
     * completed DDR lines before field 1 may start.  A line event received
     * while the previous line is still busy invalidates the current frame.
     * Invalid frames are never published.
     */
    localparam [2:0] ST_WAIT_FULL = 3'd0;
    localparam [2:0] ST_FIELD_0   = 3'd1;
    localparam [2:0] ST_FIELD_1   = 3'd2;
    localparam [2:0] ST_COMPLETE  = 3'd3;
    localparam [2:0] ST_DROP      = 3'd4;

    reg [2:0]                   r_ddr_state
        /* synthesis PAP_MARK_DEBUG="true" */;
    reg                         r_frame_accept
        /* synthesis PAP_MARK_DEBUG="true" */;
    reg                         r_current_write_bank;
    reg                         r_next_write_bank;
    reg                         r_ddr_wr_req;
    reg                         r_ddr_wr_req_d;
    reg                         r_ddr_wdata_req_d;
    reg                         r_ram_read_start_d;
    reg [DDR_DATA_WIDTH-1:0]    r_ram_rd_data_d;
    reg                         r_line_started;
    reg                         r_ddr_line_busy
        /* synthesis PAP_MARK_DEBUG="true" */;
    reg [11:0]                  r_ddr_line_cnt
        /* synthesis PAP_MARK_DEBUG="true" */;
    reg [11:0]                  r_ddr_field_line_cnt
        /* synthesis PAP_MARK_DEBUG="true" */;
    reg [LINE_ADDR_WIDTH-1:0]   r_line_addr
        /* synthesis PAP_MARK_DEBUG="true" */;
    reg                         r_ddr_field_phase
        /* synthesis PAP_MARK_DEBUG="true" */;
    reg [FRAME_CNT_WIDTH-1:0]   r_write_sequence;
    reg [FRAME_CNT_WIDTH-1:0]   r_completed_sequence;
    reg                         r_frame_done_pulse;
    reg [11:0]                  r_lines_at_full_frame_event
        /* synthesis PAP_MARK_DEBUG="true" */;
    reg                         r_incomplete_frame_seen
        /* synthesis PAP_MARK_DEBUG="true" */;
    reg                         r_field_boundary_error
        /* synthesis PAP_MARK_DEBUG="true" */;
    reg                         r_line_overrun_error
        /* synthesis PAP_MARK_DEBUG="true" */;
    reg                         r_unexpected_wdone_error
        /* synthesis PAP_MARK_DEBUG="true" */;
    reg                         r_bank_conflict_drop
        /* synthesis PAP_MARK_DEBUG="true" */;

    wire w_ddr_field_active;
    wire w_accept_line_event;
    wire w_safe_full_boundary;
    wire w_accept_second_field;

    assign w_ddr_field_active =
        (r_ddr_state == ST_FIELD_0)
        || (r_ddr_state == ST_FIELD_1);

    assign w_accept_line_event =
        w_line_event
        && !w_frame_event
        && w_ddr_field_active
        && r_frame_accept
        && !r_ddr_line_busy
        && !r_ddr_wr_req
        && (r_ddr_field_line_cnt < FIELD_LINE_NUM);

    assign w_safe_full_boundary =
        w_full_frame_event
        && !r_ddr_line_busy
        && !r_ddr_wr_req;

    assign w_accept_second_field =
        (INTERLACED_INPUT != 0)
        &&
        w_second_field_event
        && (r_ddr_state == ST_FIELD_0)
        && r_frame_accept
        && !r_ddr_line_busy
        && !r_ddr_wr_req
        && (r_ddr_field_line_cnt == FIELD_LINE_NUM)
        && (r_ddr_line_cnt == FIELD_LINE_NUM);

    /*
     * Request/data pipeline.  r_ddr_line_busy remains asserted from the
     * accepted line event through ddr_wdone, not merely through the command
     * request phase.
     */
    always @(posedge ddr_clk) begin
        if (!ddr_rstn) begin
            r_ddr_wr_req_d      <= 1'b0;
            r_ddr_wdata_req_d   <= 1'b0;
            r_ram_read_start_d  <= 1'b0;
            r_ram_rd_data_d     <= {DDR_DATA_WIDTH{1'b0}};
        end else begin
            r_ddr_wr_req_d     <= r_ddr_wr_req;
            r_ddr_wdata_req_d  <= ddr_wdata_req;
            r_ram_read_start_d <= !r_ddr_wr_req_d && r_ddr_wr_req;
            if (r_ddr_wdata_req_d || r_ram_read_start_d) begin
                r_ram_rd_data_d <= w_ram_rd_data;
            end
        end
    end

    /*
     * The final RGB565 pair is committed to the dual-clock RAM one cycle
     * after the final valid pixel.  Toggle the line event only on that RAM
     * write cycle, so the DDR domain can never observe "line complete" before
     * the last packed word is physically present.
     */
    always @(posedge wr_clk) begin
        if (!r_wr_reset_sync) begin
            r_line_event_toggle  <= 1'b0;
            r_line_event_pending <= 1'b0;
        end else if (w_wr_frame_start) begin
            r_line_event_pending <= 1'b0;
        end else begin
            if (w_line_commit) begin
                r_line_event_toggle  <= ~r_line_event_toggle;
                r_line_event_pending <= 1'b0;
            end else if (line_start || line_end || r_frame_invalid) begin
                r_line_event_pending <= 1'b0;
            end

            if (w_line_pixel_done) begin
                r_line_event_pending <= 1'b1;
            end
        end
    end

    /*
     * Line-RAM read addressing is reset only at an accepted, idle field
     * boundary.  A raw field marker can therefore no longer destroy an
     * outstanding final-line transaction.
     */
    always @(posedge ddr_clk) begin
        if (!ddr_rstn) begin
            r_line_started <= 1'b0;
            r_ram_rd_addr  <= {RAM_RD_ADDR_WIDTH{1'b0}};
        end else if (w_safe_full_boundary || w_accept_second_field) begin
            r_line_started <= 1'b0;
            r_ram_rd_addr  <= {RAM_RD_ADDR_WIDTH{1'b0}};
        end else begin
            if (w_accept_line_event) begin
                r_line_started <= 1'b1;
            end

            if (!r_ddr_wr_req_d && r_ddr_wr_req) begin
                r_ram_rd_addr <= r_ram_rd_addr + 1'b1;
            end else if (ddr_wdata_req) begin
                r_ram_rd_addr <= r_ram_rd_addr + 1'b1;
            end else if (w_accept_line_event && r_line_started) begin
                r_ram_rd_addr <= r_ram_rd_addr - 1'b1;
            end
        end
    end

    /*
     * Main DDR-domain control and frame-integrity checker.
     */
    always @(posedge ddr_clk) begin
        if (!ddr_rstn) begin
            r_ddr_state                 <= ST_WAIT_FULL;
            r_frame_accept              <= 1'b0;
            r_current_write_bank        <= 1'b0;
            r_next_write_bank           <= 1'b0;
            r_ddr_wr_req                <= 1'b0;
            r_ddr_line_busy             <= 1'b0;
            r_ddr_line_cnt              <= 12'd0;
            r_ddr_field_line_cnt        <= 12'd0;
            r_line_addr                 <= {LINE_ADDR_WIDTH{1'b0}};
            r_ddr_field_phase           <= 1'b0;
            r_write_sequence            <= {FRAME_CNT_WIDTH{1'b0}};
            r_completed_sequence        <= {FRAME_CNT_WIDTH{1'b0}};
            r_frame_done_pulse          <= 1'b0;
            r_lines_at_full_frame_event <= 12'd0;
            r_incomplete_frame_seen     <= 1'b0;
            r_field_boundary_error      <= 1'b0;
            r_line_overrun_error        <= 1'b0;
            r_unexpected_wdone_error    <= 1'b0;
            r_bank_conflict_drop        <= 1'b0;
        end else begin
            r_frame_done_pulse <= 1'b0;

            if (ddr_wdata_req) begin
                r_ddr_wr_req <= 1'b0;
            end

            /*
             * Complete one accepted line.  If the frame was invalidated
             * while this transaction was running, only clear busy; do not
             * advance any address or publish the corrupted frame.
             */
            if (ddr_wdone) begin
                if (r_ddr_line_busy) begin
                    r_ddr_line_busy <= 1'b0;

                    if (r_frame_accept
                        && (r_ddr_state == ST_FIELD_0)) begin
                        r_ddr_field_line_cnt
                            <= r_ddr_field_line_cnt + 12'd1;
                        r_ddr_line_cnt <= r_ddr_line_cnt + 12'd1;

                        if (r_ddr_field_line_cnt
                            < FIELD_LINE_NUM - 1) begin
                            r_line_addr
                                <= r_line_addr
                                   + DDR_FIELD_LINE_STRIDE;
                        end else if ((INTERLACED_INPUT == 0)
                                     && (r_ddr_line_cnt
                                         == V_NUM - 1)) begin
                            /*
                             * Progressive mode: one complete V_NUM-line
                             * input frame has reached DDR.
                             */
                            r_ddr_state          <= ST_COMPLETE;
                            r_completed_sequence <= r_write_sequence;
                            r_write_sequence
                                <= r_write_sequence + 1'b1;
                            r_frame_done_pulse
                                <= 1'b1;
                            r_next_write_bank
                                <= ~r_current_write_bank;
                        end
                    end else if (r_frame_accept
                                 && (r_ddr_state == ST_FIELD_1)) begin
                        r_ddr_field_line_cnt
                            <= r_ddr_field_line_cnt + 12'd1;
                        r_ddr_line_cnt <= r_ddr_line_cnt + 12'd1;

                        if (r_ddr_field_line_cnt
                            < FIELD_LINE_NUM - 1) begin
                            r_line_addr
                                <= r_line_addr
                                   + DDR_FIELD_LINE_STRIDE;
                        end else if (r_ddr_line_cnt
                                     == V_NUM - 1) begin
                            /*
                             * Exactly FIELD_LINE_NUM lines from each field
                             * have completed.  Only now is the bank visible
                             * to the reader.
                             */
                            r_ddr_state          <= ST_COMPLETE;
                            r_completed_sequence <= r_write_sequence;
                            r_write_sequence
                                <= r_write_sequence + 1'b1;
                            r_frame_done_pulse
                                <= 1'b1;
                            r_next_write_bank
                                <= ~r_current_write_bank;
                        end
                    end
                end else begin
                    r_unexpected_wdone_error <= 1'b1;
                end
            end

            /*
             * A full-frame marker starts field 0.  If an old line is still
             * active, this entire incoming pair is dropped; the active line
             * is allowed to finish so no transaction is aborted halfway.
             */
            if (w_frame_event) begin
                if (w_full_frame_event) begin
                    r_lines_at_full_frame_event <= r_ddr_line_cnt;

                    if (w_safe_full_boundary) begin
                        r_ddr_line_cnt       <= 12'd0;
                        r_ddr_field_line_cnt <= 12'd0;
                        r_line_addr
                            <= {LINE_ADDR_WIDTH{1'b0}};
                        r_ddr_field_phase <= 1'b0;

                        if (!active_read_valid
                            || (r_next_write_bank
                                != active_read_bank)) begin
                            r_ddr_state          <= ST_FIELD_0;
                            r_frame_accept       <= 1'b1;
                            r_current_write_bank
                                <= r_next_write_bank;
                        end else begin
                            r_ddr_state    <= ST_DROP;
                            r_frame_accept <= 1'b0;
                            r_bank_conflict_drop <= 1'b1;
                        end

                        if ((r_ddr_state == ST_FIELD_0)
                            || (r_ddr_state == ST_FIELD_1)) begin
                            r_incomplete_frame_seen <= 1'b1;
                        end
                    end else begin
                        r_ddr_state            <= ST_DROP;
                        r_frame_accept         <= 1'b0;
                        r_field_boundary_error <= 1'b1;
                        r_incomplete_frame_seen <= 1'b1;
                    end
                end else if (w_second_field_event) begin
                    if (w_accept_second_field) begin
                        r_ddr_state          <= ST_FIELD_1;
                        r_ddr_field_line_cnt <= 12'd0;
                        r_line_addr          <= DDR_ADDR_OFFSET;
                        r_ddr_field_phase    <= 1'b1;
                    end else begin
                        r_ddr_state            <= ST_DROP;
                        r_frame_accept         <= 1'b0;
                        r_field_boundary_error <= 1'b1;
                        r_incomplete_frame_seen <= 1'b1;
                    end
                end
            end else if (w_line_event) begin
                if (w_accept_line_event) begin
                    r_ddr_wr_req    <= 1'b1;
                    r_ddr_line_busy <= 1'b1;
                end else if (w_ddr_field_active) begin
                    /*
                     * A second line arrived before the previous line reached
                     * ddr_wdone, or more than FIELD_LINE_NUM lines arrived.
                     * Do not cancel the active transaction; invalidate the
                     * frame and recover at a later full-frame boundary.
                     */
                    r_ddr_state            <= ST_DROP;
                    r_frame_accept         <= 1'b0;
                    r_line_overrun_error   <= 1'b1;
                    r_incomplete_frame_seen <= 1'b1;
                end else if (r_ddr_state == ST_COMPLETE) begin
                    r_line_overrun_error <= 1'b1;
                end
            end
        end
    end

    assign ddr_wreq   = r_ddr_wr_req;
    assign ddr_waddr  = {r_current_write_bank, r_line_addr} + ADDR_OFFSET;
    assign ddr_wr_len = RD_LINE_NUM;
    assign ddr_wdata  = (!r_ddr_wdata_req_d && ddr_wdata_req)
                      ? r_ram_rd_data_d : w_ram_rd_data;
    assign frame_wcnt = r_completed_sequence;
    assign frame_wirq = r_frame_done_pulse;

endmodule

`default_nettype wire
