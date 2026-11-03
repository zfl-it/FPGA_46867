`timescale 1ns / 1ps
`default_nettype none

/*
 * DDR frame-buffer reader.
 *
 * vout_clk and ddr_clk are asynchronous.  Frame/line requests cross as event
 * toggles through two flip-flops.  Pixel data crosses through rd_fram_buf, a
 * vendor true dual-clock RAM.
 */
module rd_buf #(
    parameter integer ADDR_WIDTH      = 27,
    parameter integer ADDR_OFFSET     = 0,
    parameter integer H_NUM           = 1280,
    parameter integer V_NUM           = 720,
    parameter integer DQ_WIDTH        = 16,
    parameter integer LEN_WIDTH       = 32,
    parameter integer PIX_WIDTH       = 16,
    parameter integer LINE_ADDR_WIDTH = 22,
    parameter integer FRAME_CNT_WIDTH = 6
)(
    input  wire                         ddr_clk,
    input  wire                         ddr_rstn,

    input  wire                         vout_clk,
    input  wire                         rd_fsync,
    input  wire                         rd_en,
    output wire                         vout_de,
    output wire [PIX_WIDTH-1:0]         vout_data,

    input  wire                         init_done,
    input  wire [FRAME_CNT_WIDTH-1:0]   completed_frame_cnt,
    input  wire                         frame_done,
    output wire                         active_read_bank,
    output wire                         active_read_valid,

    output wire                         ddr_rreq,
    output wire [ADDR_WIDTH-1:0]        ddr_raddr,
    output wire [LEN_WIDTH-1:0]         ddr_rd_len,
    input  wire                         ddr_rrdy,
    input  wire                         ddr_rdone,

    input  wire [8*DQ_WIDTH-1:0]        ddr_rdata,
    input  wire                         ddr_rdata_en
);

    localparam integer RAM_WIDTH         = 32;
    localparam integer DDR_DATA_WIDTH    = DQ_WIDTH * 8;
    localparam integer WR_LINE_NUM       = H_NUM * PIX_WIDTH / RAM_WIDTH;
    localparam integer RD_LINE_NUM       = WR_LINE_NUM * RAM_WIDTH
                                         / DDR_DATA_WIDTH;
    localparam integer DDR_ADDR_OFFSET   = RD_LINE_NUM * DDR_DATA_WIDTH
                                         / DQ_WIDTH;
    localparam integer RAM_WR_ADDR_WIDTH = (DDR_DATA_WIDTH == 128) ? 10 : 9;

    /* --------------------------------------------------------------------- */
    /* Reset synchronization into vout_clk                                   */
    /* --------------------------------------------------------------------- */
    (* ASYNC_REG = "TRUE" *) reg r_vout_reset_meta;
    (* ASYNC_REG = "TRUE" *) reg r_vout_reset_sync;

    always @(posedge vout_clk or negedge ddr_rstn) begin
        if (!ddr_rstn) begin
            r_vout_reset_meta <= 1'b0;
            r_vout_reset_sync <= 1'b0;
        end else begin
            r_vout_reset_meta <= 1'b1;
            r_vout_reset_sync <= r_vout_reset_meta;
        end
    end

    /* --------------------------------------------------------------------- */
    /* vout_clk domain: turn VSYNC and DE line starts into event toggles      */
    /* --------------------------------------------------------------------- */
    reg r_rd_fsync_d;
    reg r_rd_en_d1;
    reg r_rd_en_d2;
    reg r_frame_event_toggle;
    reg r_line_event_toggle;

    wire w_vout_frame_start;
    wire w_vout_line_start;

    assign w_vout_frame_start = rd_fsync && !r_rd_fsync_d;
    assign w_vout_line_start  = rd_en && !r_rd_en_d1;

    always @(posedge vout_clk) begin
        if (!r_vout_reset_sync) begin
            r_rd_fsync_d        <= 1'b0;
            r_rd_en_d1          <= 1'b0;
            r_rd_en_d2          <= 1'b0;
            r_frame_event_toggle <= 1'b0;
            r_line_event_toggle  <= 1'b0;
        end else begin
            r_rd_fsync_d <= rd_fsync;
            r_rd_en_d1   <= rd_en;
            r_rd_en_d2   <= r_rd_en_d1;

            if (w_vout_frame_start) begin
                r_frame_event_toggle <= ~r_frame_event_toggle;
            end
            if (w_vout_line_start) begin
                r_line_event_toggle <= ~r_line_event_toggle;
            end
        end
    end

    /* --------------------------------------------------------------------- */
    /* ddr_clk domain: synchronize event toggles                              */
    /* --------------------------------------------------------------------- */
    (* ASYNC_REG = "TRUE" *) reg r_frame_toggle_meta;
    (* ASYNC_REG = "TRUE" *) reg r_frame_toggle_sync;
    reg                            r_frame_toggle_d;
    (* ASYNC_REG = "TRUE" *) reg r_line_toggle_meta;
    (* ASYNC_REG = "TRUE" *) reg r_line_toggle_sync;
    reg                            r_line_toggle_d;

    wire w_frame_event;
    wire w_line_event;

    assign w_frame_event = r_frame_toggle_sync ^ r_frame_toggle_d;
    assign w_line_event  = r_line_toggle_sync ^ r_line_toggle_d;

    always @(posedge ddr_clk) begin
        if (!ddr_rstn) begin
            r_frame_toggle_meta <= 1'b0;
            r_frame_toggle_sync <= 1'b0;
            r_frame_toggle_d    <= 1'b0;
            r_line_toggle_meta  <= 1'b0;
            r_line_toggle_sync  <= 1'b0;
            r_line_toggle_d     <= 1'b0;
        end else begin
            r_frame_toggle_meta <= r_frame_event_toggle;
            r_frame_toggle_sync <= r_frame_toggle_meta;
            r_frame_toggle_d    <= r_frame_toggle_sync;
            r_line_toggle_meta  <= r_line_event_toggle;
            r_line_toggle_sync  <= r_line_toggle_meta;
            r_line_toggle_d     <= r_line_toggle_sync;
        end
    end

    /* The latest completed writer bank is sampled only at an output frame
       boundary.  The selected bank then remains constant for the full frame. */
    reg [FRAME_CNT_WIDTH-1:0] r_latest_completed_frame;
    reg                       r_frame_available;        /* synthesis PAP_MARK_DEBUG="true" */
    reg                       r_active_read_bank;       /* synthesis PAP_MARK_DEBUG="true" */
    reg                       r_active_read_valid;      /* synthesis PAP_MARK_DEBUG="true" */

    always @(posedge ddr_clk) begin
        if (!ddr_rstn) begin
            r_latest_completed_frame <= {FRAME_CNT_WIDTH{1'b0}};
            r_frame_available        <= 1'b0;
            r_active_read_bank       <= 1'b0;
            r_active_read_valid      <= 1'b0;
        end else begin
            if (frame_done) begin
                r_latest_completed_frame <= completed_frame_cnt;
                r_frame_available        <= 1'b1;
            end

            if (w_frame_event) begin
                if (frame_done) begin
                    r_active_read_bank  <= completed_frame_cnt[0];
                    r_active_read_valid <= init_done;
                end else if (r_frame_available) begin
                    r_active_read_bank  <= r_latest_completed_frame[0];
                    r_active_read_valid <= init_done;
                end
            end
        end
    end

    /* Prefetch line 0 at VSYNC, then one next line at each DE rising edge. */
    reg [11:0]                  r_prefetch_line_count;
    reg [LINE_ADDR_WIDTH-1:0]   r_line_addr;
    reg                         r_ddr_read_req;

    always @(posedge ddr_clk) begin
        if (!ddr_rstn) begin
            r_prefetch_line_count <= 12'd0;
            r_line_addr           <= {LINE_ADDR_WIDTH{1'b0}};
            r_ddr_read_req        <= 1'b0;
        end else begin
            if (w_frame_event) begin
                r_line_addr <= {LINE_ADDR_WIDTH{1'b0}};
                if ((frame_done && init_done)
                    || (r_frame_available && init_done)) begin
                    r_prefetch_line_count <= 12'd1;
                    r_ddr_read_req <= 1'b1;
                end else begin
                    r_prefetch_line_count <= 12'd0;
                    r_ddr_read_req <= 1'b0;
                end
            end else begin
                if (w_line_event && r_active_read_valid
                    && (r_prefetch_line_count < V_NUM)) begin
                    r_prefetch_line_count <= r_prefetch_line_count + 12'd1;
                    r_ddr_read_req <= 1'b1;
                end else if (r_ddr_read_req && ddr_rrdy) begin
                    r_ddr_read_req <= 1'b0;
                end

                if (ddr_rdone && r_active_read_valid
                    && (r_prefetch_line_count <= V_NUM)) begin
                    r_line_addr <= r_line_addr + DDR_ADDR_OFFSET;
                end
            end
        end
    end

    assign ddr_rreq   = r_ddr_read_req;
    assign ddr_raddr  = {r_active_read_bank, r_line_addr} + ADDR_OFFSET;
    assign ddr_rd_len = RD_LINE_NUM;
    assign active_read_bank  = r_active_read_bank;
    assign active_read_valid = r_active_read_valid;

    /* --------------------------------------------------------------------- */
    /* Dual-clock line RAM                                                    */
    /* --------------------------------------------------------------------- */
    reg [RAM_WR_ADDR_WIDTH-1:0] r_ram_wr_addr;
    reg [11:0]                  r_ram_rd_addr;
    wire [RAM_WIDTH-1:0]        w_ram_rd_data;

    always @(posedge ddr_clk) begin
        if (!ddr_rstn || w_frame_event) begin
            r_ram_wr_addr <= {RAM_WR_ADDR_WIDTH{1'b0}};
        end else if (ddr_rdata_en) begin
            r_ram_wr_addr <= r_ram_wr_addr + 1'b1;
        end
    end

    rd_fram_buf u_rd_fram_buf (
        .wr_data (ddr_rdata),
        .wr_addr (r_ram_wr_addr),
        .wr_en   (ddr_rdata_en),
        .wr_clk  (ddr_clk),
        .wr_rst  (~ddr_rstn),

        .rd_addr (r_ram_rd_addr),
        .rd_data (w_ram_rd_data),
        .rd_clk  (vout_clk),
        .rd_rst  (~r_vout_reset_sync)
    );

    /* r_active_read_valid is a DDR-domain level and is therefore synchronized
       before it gates the output video domain. */
    (* ASYNC_REG = "TRUE" *) reg r_stream_active_meta;
    (* ASYNC_REG = "TRUE" *) reg r_stream_active_sync;

    always @(posedge vout_clk) begin
        if (!r_vout_reset_sync) begin
            r_stream_active_meta <= 1'b0;
            r_stream_active_sync <= 1'b0;
        end else begin
            r_stream_active_meta <= r_active_read_valid;
            r_stream_active_sync <= r_stream_active_meta;
        end
    end

    reg [1:0]                 r_pixel_phase;
    reg [PIX_WIDTH-1:0]       r_read_data;
    reg [RAM_WIDTH-1:0]       r_ram_rd_data_d;
    wire                      w_read_word_en;

    assign w_read_word_en = (PIX_WIDTH == 24)
                          ? (rd_en && r_stream_active_sync
                             && (r_pixel_phase != 2'd3))
                          : (PIX_WIDTH == 16)
                          ? (rd_en && r_stream_active_sync
                             && !r_pixel_phase[0])
                          : (rd_en && r_stream_active_sync);

    always @(posedge vout_clk) begin
        if (!r_vout_reset_sync || w_vout_frame_start) begin
            r_pixel_phase <= 2'd0;
            r_ram_rd_addr <= 12'd0;
            r_ram_rd_data_d <= {RAM_WIDTH{1'b0}};
        end else begin
            r_ram_rd_data_d <= w_ram_rd_data;

            if (rd_en && r_stream_active_sync) begin
                r_pixel_phase <= r_pixel_phase + 2'd1;
            end else begin
                r_pixel_phase <= 2'd0;
            end

            if (w_read_word_en) begin
                r_ram_rd_addr <= r_ram_rd_addr + 12'd1;
            end
        end
    end

    generate
        if (PIX_WIDTH == 24) begin : g_unpack_rgb888
            always @(posedge vout_clk) begin
                if (!r_vout_reset_sync || !r_stream_active_sync) begin
                    r_read_data <= {PIX_WIDTH{1'b0}};
                end else if (r_rd_en_d1) begin
                    if (r_pixel_phase == 2'd1) begin
                        r_read_data <= w_ram_rd_data[PIX_WIDTH-1:0];
                    end else if (r_pixel_phase == 2'd2) begin
                        r_read_data <= {w_ram_rd_data[15:0],
                                        r_ram_rd_data_d[31:PIX_WIDTH]};
                    end else if (r_pixel_phase == 2'd3) begin
                        r_read_data <= {w_ram_rd_data[7:0],
                                        r_ram_rd_data_d[31:16]};
                    end else begin
                        r_read_data <= r_ram_rd_data_d[31:8];
                    end
                end else begin
                    r_read_data <= {PIX_WIDTH{1'b0}};
                end
            end
        end else if (PIX_WIDTH == 16) begin : g_unpack_rgb565
            always @(posedge vout_clk) begin
                if (!r_vout_reset_sync || !r_stream_active_sync) begin
                    r_read_data <= {PIX_WIDTH{1'b0}};
                end else if (r_rd_en_d1) begin
                    if (r_pixel_phase[0]) begin
                        r_read_data <= w_ram_rd_data[15:0];
                    end else begin
                        r_read_data <= r_ram_rd_data_d[31:16];
                    end
                end else begin
                    r_read_data <= {PIX_WIDTH{1'b0}};
                end
            end
        end else begin : g_unpack_native
            always @(posedge vout_clk) begin
                if (!r_vout_reset_sync || !r_stream_active_sync) begin
                    r_read_data <= {PIX_WIDTH{1'b0}};
                end else begin
                    r_read_data <= w_ram_rd_data[PIX_WIDTH-1:0];
                end
            end
        end
    endgenerate

    assign vout_de   = r_rd_en_d2 && r_stream_active_sync;
    assign vout_data = r_read_data;

endmodule

`default_nettype wire
