`timescale 1ns / 1ps
`default_nettype none

/*
 * Video stream to PCIe DMA packetizer.
 *
 * Clock-domain rules:
 *   1. Video pixels cross from i_video_clk to i_pcie_clk only through the
 *      vendor asynchronous FIFO video_buffer.
 *   2. i_start_tx_flag is a PCIe-domain level.  It is synchronized into the
 *      video domain by two flip-flops.
 *   3. The number of complete DMA packets written to the FIFO crosses to the
 *      PCIe domain as a Gray-coded counter through two flip-flops.
 *
 * DMA_LEN is expressed in bytes.  i_dma_len/o_dma_len are expressed in
 * DWORDs, as required by ips2l_pcie_dma.
 */
module pcie_tx_fun #(
    parameter integer IMG_WIDTH        = 1280,
    parameter integer IMG_HEIGHT       = 720,
    parameter integer DMA_ADDR_WIDTH   = 64,
    parameter integer VIDEO_DATA_WIDTH = 16,
    parameter integer PCIE_DATA_WIDTH  = 128,
    parameter integer PPC              = 1,
    parameter integer PIXCEL_BYTES     = 2,
    parameter integer DMA_LEN          = 2560
)(
    input  wire                              i_pcie_clk,
    input  wire                              i_pcie_rst_n,

    input  wire [DMA_ADDR_WIDTH-1:0]         i_dma_base_addr,
    input  wire [DMA_ADDR_WIDTH-1:0]         i_dma_base_addr2,
    input  wire [DMA_ADDR_WIDTH-1:0]         i_dma_base_addr3,
    input  wire [DMA_ADDR_WIDTH-1:0]         i_dma_base_addr4,
    input  wire [9:0]                        i_dma_len,
    input  wire                              i_dma_set_en,
    output wire [1:0]                        o_dma_wr_index,
    output wire                              o_dma_wr_done,
    input  wire                              i_start_tx_flag,
    output wire [31:0]                       o_check_data,

    input  wire                              i_dma_cmd_rdy,
    output wire                              o_dma_req,
    output wire [DMA_ADDR_WIDTH-1:0]         o_dma_addr,
    output wire [9:0]                        o_dma_len,
    input  wire                              i_dma_tx_done,

    input  wire                              i_video_clk,
    input  wire                              i_video_rst_n,
    input  wire [VIDEO_DATA_WIDTH*PPC-1:0]   i_video_data,
    input  wire                              i_video_vs,
    input  wire                              i_video_de,

    output wire [PCIE_DATA_WIDTH-1:0]        o_dma_rd_data,
    input  wire                              i_dma_rd_en
);

    localparam integer IMG_SIZE_BYTES = IMG_WIDTH * IMG_HEIGHT * PIXCEL_BYTES;
    localparam integer SEND_TIMES     = IMG_SIZE_BYTES / DMA_LEN;
    localparam integer PACKET_PIXELS  = DMA_LEN / PIXCEL_BYTES / PPC;
    localparam integer DMA_LEN_DWORDS = DMA_LEN / 4;

    /*
     * Default 720p60 RGB565 configuration:
     *   one line       = 1280 * 2 = 2560 bytes;
     *   one DMA packet = 2560 bytes;
     *   one frame      = 720 DMA packets = 1,843,200 bytes.
     */

    /* --------------------------------------------------------------------- */
    /* Video clock domain                                                    */
    /* --------------------------------------------------------------------- */
    (* ASYNC_REG = "TRUE" *) reg r_start_video_meta;
    (* ASYNC_REG = "TRUE" *) reg r_start_video_sync;    /* synthesis PAP_MARK_DEBUG="true" */

    reg        r_video_vs_d;
    reg        r_capture_active;        /* synthesis PAP_MARK_DEBUG="true" */
    reg [15:0] r_packet_pixel_cnt;      /* synthesis PAP_MARK_DEBUG="true" */
    reg [15:0] r_packet_count_bin;      /* synthesis PAP_MARK_DEBUG="true" */
    reg [15:0] r_packet_count_gray;
    reg        r_fifo_overflow;

    wire       w_video_vs_rise;
    wire       w_fifo_full;     /* synthesis PAP_MARK_DEBUG="true" */
    wire       w_fifo_empty;
    wire       w_fifo_wr_en;    /* synthesis PAP_MARK_DEBUG="true" */
    wire       w_fifo_wr_accept;
    wire       w_fifo_wr_rst;
    wire       w_fifo_wr_ready;
    wire [15:0] w_packet_count_bin_next;
    wire [15:0] w_packet_count_gray_next;
    reg  [3:0] r_fifo_wr_reset_guard;

    assign w_video_vs_rise          = i_video_vs && !r_video_vs_d;
    assign w_fifo_wr_en             = r_capture_active && i_video_de;
    assign w_fifo_wr_accept         = w_fifo_wr_en && !w_fifo_full;
    assign w_fifo_wr_rst            = !i_video_rst_n
                                    || !r_start_video_sync;
    assign w_fifo_wr_ready          = r_fifo_wr_reset_guard[3];
    assign w_packet_count_bin_next  = r_packet_count_bin + 16'd1;
    assign w_packet_count_gray_next = (w_packet_count_bin_next >> 1)
                                    ^ w_packet_count_bin_next;

    always @(posedge i_video_clk) begin
        if (!i_video_rst_n) begin
            r_start_video_meta <= 1'b0;
            r_start_video_sync <= 1'b0;
            r_video_vs_d       <= 1'b0;
        end else begin
            r_start_video_meta <= i_start_tx_flag;
            r_start_video_sync <= r_start_video_meta;
            r_video_vs_d       <= i_video_vs;
        end
    end

    /*
     * Keep writes disabled for eight video-clock cycles after the FIFO reset
     * is released.  A start request arriving immediately before VSYNC will
     * therefore skip that frame and begin cleanly on the following VSYNC.
     */
    always @(posedge i_video_clk) begin
        if (!i_video_rst_n || !r_start_video_sync) begin
            r_fifo_wr_reset_guard <= 4'd0;
        end else if (!w_fifo_wr_ready) begin
            r_fifo_wr_reset_guard <= r_fifo_wr_reset_guard + 4'd1;
        end
    end

    /* Start capture only at a VSYNC boundary.  This prevents a DMA frame
       from beginning in the middle of a video frame. */
    always @(posedge i_video_clk) begin
        if (!i_video_rst_n) begin
            r_capture_active <= 1'b0;
        end else if (!r_start_video_sync) begin
            r_capture_active <= 1'b0;
        end else if (w_video_vs_rise && w_fifo_wr_ready) begin
            r_capture_active <= 1'b1;
        end
    end

    /* One Gray-counter increment means that one complete logical DMA packet
       has already been committed into the asynchronous FIFO. */
    always @(posedge i_video_clk) begin
        if (!i_video_rst_n) begin
            r_packet_pixel_cnt <= 16'd0;
            r_packet_count_bin <= 16'd0;
            r_packet_count_gray <= 16'd0;
            r_fifo_overflow <= 1'b0;
        end else if (!r_start_video_sync) begin
            r_packet_pixel_cnt <= 16'd0;
            r_packet_count_bin <= 16'd0;
            r_packet_count_gray <= 16'd0;
            r_fifo_overflow <= 1'b0;
        end else begin
            if (w_fifo_wr_en && w_fifo_full) begin
                r_fifo_overflow <= 1'b1;
            end

            if (w_fifo_wr_accept) begin
                if (r_packet_pixel_cnt == PACKET_PIXELS - 1) begin
                    r_packet_pixel_cnt  <= 16'd0;
                    r_packet_count_bin  <= w_packet_count_bin_next;
                    r_packet_count_gray <= w_packet_count_gray_next;
                end else begin
                    r_packet_pixel_cnt <= r_packet_pixel_cnt + 16'd1;
                end
            end
        end
    end

    /* --------------------------------------------------------------------- */
    /* PCIe clock domain                                                     */
    /* --------------------------------------------------------------------- */
    (* ASYNC_REG = "TRUE" *) reg [15:0] r_packet_gray_meta;
    (* ASYNC_REG = "TRUE" *) reg [15:0] r_packet_gray_sync;     /* synthesis PAP_MARK_DEBUG="true" */
    (* ASYNC_REG = "TRUE" *) reg        r_fifo_overflow_meta;
    (* ASYNC_REG = "TRUE" *) reg        r_fifo_overflow_sync;

    reg [DMA_ADDR_WIDTH-1:0] r_base_addr0;
    reg [DMA_ADDR_WIDTH-1:0] r_base_addr1;
    reg [DMA_ADDR_WIDTH-1:0] r_base_addr2;
    reg [DMA_ADDR_WIDTH-1:0] r_base_addr3;

    reg [15:0] r_consumed_packet_count;     /* synthesis PAP_MARK_DEBUG="true" */
    reg [15:0] r_frame_packet_count;
    reg [1:0]  r_dma_index;
    reg [1:0]  r_completed_index;
    reg        r_frame_done;

    reg [DMA_ADDR_WIDTH-1:0] r_dma_addr;
    reg [9:0]                r_dma_len;
    reg                      r_dma_req;
    reg                      r_dma_busy;
    reg [12:0]               r_bytes_remaining;
    reg [12:0]               r_fragment_bytes;
    reg                      r_fifo_underflow;

    wire [15:0] w_produced_packet_count;        /* synthesis PAP_MARK_DEBUG="true" */
    wire        w_complete_packet_available;
    wire        w_fragment_continuation;
    wire        w_dma_work_available;
    wire        w_fifo_rd_rst;
    wire        w_fifo_rd_ready;
    wire [1:0]  w_next_dma_index;
    wire [DMA_ADDR_WIDTH-1:0] w_next_frame_base;
    wire [12:0] w_bytes_to_4k_boundary;
    wire [12:0] w_next_fragment_bytes;
    wire [9:0]  w_next_fragment_dwords;
    reg  [3:0]  r_fifo_rd_reset_guard;

    function [15:0] gray_to_bin;
        input [15:0] gray_value;
        integer bit_index;
        begin
            gray_to_bin[15] = gray_value[15];
            for (bit_index = 14; bit_index >= 0; bit_index = bit_index - 1) begin
                gray_to_bin[bit_index] = gray_to_bin[bit_index + 1]
                                       ^ gray_value[bit_index];
            end
        end
    endfunction

    assign w_produced_packet_count     = gray_to_bin(r_packet_gray_sync);
    assign w_complete_packet_available = (w_produced_packet_count
                                        != r_consumed_packet_count);
    assign w_fragment_continuation     = (r_bytes_remaining != DMA_LEN);
    assign w_fifo_rd_rst               = !i_pcie_rst_n
                                       || !i_start_tx_flag;
    assign w_fifo_rd_ready             = r_fifo_rd_reset_guard[3];
    assign w_dma_work_available        = w_fifo_rd_ready
                                      && (w_fragment_continuation
                                          || w_complete_packet_available);
    assign w_next_dma_index            = r_dma_index + 2'd1;

    assign w_next_frame_base = (w_next_dma_index == 2'd0) ? r_base_addr0 :
                               (w_next_dma_index == 2'd1) ? r_base_addr1 :
                               (w_next_dma_index == 2'd2) ? r_base_addr2 :
                                                            r_base_addr3;

    /* A PCIe Memory Write request must not cross a 4 KiB boundary.  A
       3840-byte logical packet is therefore split into legal fragments when
       required.  The RK buffer layout is unchanged. */
    assign w_bytes_to_4k_boundary = 13'd4096 - {1'b0, r_dma_addr[11:0]};
    assign w_next_fragment_bytes  = (r_bytes_remaining
                                   <= w_bytes_to_4k_boundary)
                                  ? r_bytes_remaining
                                  : w_bytes_to_4k_boundary;
    assign w_next_fragment_dwords = w_next_fragment_bytes >> 2;

    /* Do not issue a DMA request until the read side has been out of reset
       for at least eight PCIe-clock cycles. */
    always @(posedge i_pcie_clk) begin
        if (!i_pcie_rst_n || !i_start_tx_flag) begin
            r_fifo_rd_reset_guard <= 4'd0;
        end else if (!w_fifo_rd_ready) begin
            r_fifo_rd_reset_guard <= r_fifo_rd_reset_guard + 4'd1;
        end
    end

    always @(posedge i_pcie_clk) begin
        if (!i_pcie_rst_n) begin
            r_packet_gray_meta    <= 16'd0;
            r_packet_gray_sync    <= 16'd0;
            r_fifo_overflow_meta  <= 1'b0;
            r_fifo_overflow_sync  <= 1'b0;
        end else if (!i_start_tx_flag) begin
            r_packet_gray_meta    <= 16'd0;
            r_packet_gray_sync    <= 16'd0;
            r_fifo_overflow_meta  <= 1'b0;
            r_fifo_overflow_sync  <= 1'b0;
        end else begin
            r_packet_gray_meta    <= r_packet_count_gray;
            r_packet_gray_sync    <= r_packet_gray_meta;
            r_fifo_overflow_meta  <= r_fifo_overflow;
            r_fifo_overflow_sync  <= r_fifo_overflow_meta;
        end
    end

    /* Base addresses are all in the PCIe clock domain.  The software writes
       all four addresses first and then pulses i_dma_set_en. */
    always @(posedge i_pcie_clk) begin
        if (!i_pcie_rst_n) begin
            r_base_addr0 <= {DMA_ADDR_WIDTH{1'b0}};
            r_base_addr1 <= {DMA_ADDR_WIDTH{1'b0}};
            r_base_addr2 <= {DMA_ADDR_WIDTH{1'b0}};
            r_base_addr3 <= {DMA_ADDR_WIDTH{1'b0}};
        end else if (i_dma_set_en) begin
            r_base_addr0 <= i_dma_base_addr;
            r_base_addr1 <= i_dma_base_addr2;
            r_base_addr2 <= i_dma_base_addr3;
            r_base_addr3 <= i_dma_base_addr4;
        end
    end

    always @(posedge i_pcie_clk) begin
        if (!i_pcie_rst_n) begin
            r_consumed_packet_count <= 16'd0;
            r_frame_packet_count    <= 16'd0;
            r_dma_index             <= 2'd0;
            r_completed_index       <= 2'd0;
            r_frame_done            <= 1'b0;
            r_dma_addr              <= {DMA_ADDR_WIDTH{1'b0}};
            r_dma_len               <= DMA_LEN_DWORDS;
            r_dma_req               <= 1'b0;
            r_dma_busy              <= 1'b0;
            r_bytes_remaining       <= DMA_LEN;
            r_fragment_bytes        <= 13'd0;
            r_fifo_underflow        <= 1'b0;
        end else begin
            r_frame_done <= 1'b0;

            if (!i_start_tx_flag) begin
                r_consumed_packet_count <= 16'd0;
                r_frame_packet_count    <= 16'd0;
                r_dma_index             <= 2'd0;
                r_completed_index       <= 2'd0;
                r_dma_addr              <= r_base_addr0;
                r_dma_len               <= (i_dma_len != 10'd0)
                                         ? i_dma_len : DMA_LEN_DWORDS;
                r_dma_req               <= 1'b0;
                r_dma_busy              <= 1'b0;
                r_bytes_remaining       <= DMA_LEN;
                r_fragment_bytes        <= 13'd0;
                r_fifo_underflow        <= 1'b0;
            end else if (i_dma_tx_done && r_dma_busy) begin
                r_dma_busy <= 1'b0;

                if (r_bytes_remaining > r_fragment_bytes) begin
                    r_dma_addr        <= r_dma_addr + r_fragment_bytes;
                    r_bytes_remaining <= r_bytes_remaining - r_fragment_bytes;
                end else begin
                    r_consumed_packet_count <= r_consumed_packet_count + 16'd1;
                    r_bytes_remaining       <= DMA_LEN;

                    if (r_frame_packet_count == SEND_TIMES - 1) begin
                        r_frame_packet_count <= 16'd0;
                        r_completed_index    <= r_dma_index;
                        r_frame_done         <= 1'b1;
                        r_dma_index          <= w_next_dma_index;
                        r_dma_addr           <= w_next_frame_base;
                    end else begin
                        r_frame_packet_count <= r_frame_packet_count + 16'd1;
                        r_dma_addr           <= r_dma_addr + r_fragment_bytes;
                    end
                end
            end else if (r_dma_req && i_dma_cmd_rdy) begin
                r_dma_req  <= 1'b0;
                r_dma_busy <= 1'b1;
            end else if (!r_dma_req && !r_dma_busy
                         && w_dma_work_available) begin
                r_dma_req        <= 1'b1;
                r_dma_len        <= w_next_fragment_dwords;
                r_fragment_bytes <= w_next_fragment_bytes;
            end

            if (i_dma_rd_en && w_fifo_empty) begin
                r_fifo_underflow <= 1'b1;
            end
        end
    end

    assign o_dma_wr_index = r_completed_index;
    assign o_dma_wr_done  = r_frame_done;
    assign o_check_data   = {29'd0, r_completed_index, r_frame_done};
    assign o_dma_req      = r_dma_req;
    assign o_dma_addr     = r_dma_addr;
    assign o_dma_len      = r_dma_len;

    /*
     * The vendor FIFO is the only multi-bit video-data CDC element.
     *
     * video_buffer must be regenerated with:
     *   FIFO type       : asynchronous / independent clocks
     *   write port      : 16 bits x 16384 words
     *   read port       : 128 bits x 2048 words
     *   total capacity  : 32768 bytes (32 KiB)
     *   read mode       : identical to the working 16 KiB IP
     *
     * Water-level and almost-full/empty ports are intentionally omitted.
     * Named-port omission is legal whether those optional IP ports are
     * enabled or disabled, so regenerating the IP cannot break this instance
     * merely because an optional status-port selection changed.
     */
    video_buffer u_video_buffer (
        .wr_clk   (i_video_clk),
        .wr_rst   (w_fifo_wr_rst),
        .wr_en    (w_fifo_wr_accept),
        .wr_data  (i_video_data),
        .wr_full  (w_fifo_full),

        .rd_clk   (i_pcie_clk),
        .rd_rst   (w_fifo_rd_rst),
        .rd_en    (i_dma_rd_en),
        .rd_data  (o_dma_rd_data),
        .rd_empty (w_fifo_empty)
    );

    /* Keep these status registers for on-chip debug probing. */
    wire w_fifo_error_unused;
    assign w_fifo_error_unused = r_fifo_overflow_sync | r_fifo_underflow;

endmodule

`default_nettype wire
