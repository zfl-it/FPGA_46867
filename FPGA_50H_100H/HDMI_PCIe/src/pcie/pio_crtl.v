`timescale 1ns / 1ps
`default_nettype none

/* PCIe PIO register block.  All ports in this module are in pcie_clk. */
module pio_crtl (
    input  wire        pcie_clk,
    input  wire        rst_n,

    output wire        start_flag,
    output wire        set_dma_config_en,
    output wire [63:0] o_ch0_base_addr,
    output wire [63:0] o_ch0_base_addr2,
    output wire [63:0] o_ch0_base_addr3,
    output wire [63:0] o_ch0_base_addr4,
    input  wire        i_wr_frame_done,
    input  wire [1:0]  i_wr_index,

    /* marker_row_detect status, already synchronized into pcie_clk by the top
       level.  Only single-bit levels are brought in on purpose: there is no
       multi-bit word here that would need a coherent handshake. */
    input  wire        i_marker_mode,
    input  wire        i_marker_ok,
    input  wire        i_marker_valid,
    input  wire        i_marker_tgl,

    input  wire        pio_wr_en,
    input  wire [9:0]  pio_wr_addr,
    input  wire [31:0] pio_wr_data,

    input  wire        pio_rd_en,
    input  wire [9:0]  pio_rd_addr,
    output wire [31:0] pio_rd_data
);

    localparam [9:0] WR_FRAME_DONE = 10'h140;
    localparam [9:0] MARKER_STATUS = 10'h144;   // read-only, read clears bit 3
    localparam [9:0] DMA_ADDR_L    = 10'h050;
    localparam [9:0] DMA_ADDR_H    = 10'h054;
    localparam [9:0] DMA_ADDR1_L   = 10'h020;
    localparam [9:0] DMA_ADDR1_H   = 10'h024;
    localparam [9:0] DMA_ADDR2_L   = 10'h040;
    localparam [9:0] DMA_ADDR2_H   = 10'h044;
    localparam [9:0] DMA_ADDR3_L   = 10'h030;
    localparam [9:0] DMA_ADDR3_H   = 10'h034;
    localparam [9:0] DMA_SET_EN    = 10'h060;

    reg        r_start_flag;
    reg        r_set_dma_config_en;
    reg [31:0] r_pio_rd_data;
    reg        r_wr_frame_done;
    reg [1:0]  r_wr_index_latched;
    reg [63:0] r_ch0_base_addr;
    reg [63:0] r_ch0_base_addr2;
    reg [63:0] r_ch0_base_addr3;
    reg [63:0] r_ch0_base_addr4;
    reg        r_marker_tgl_d;
    reg [11:0] r_marker_frame_cnt;
    reg        r_marker_new;

    always @(posedge pcie_clk) begin
        if (!rst_n) begin
            r_start_flag <= 1'b0;  // 复位值 0 = 不启动；RK 需向 0x000 写 0xffffffe5
                                   // 启动视频流、写 0xffffff00 停止（与 RK 驱动流程一致）
        end else if (pio_wr_en && (pio_wr_addr == 10'h000)
                     && (pio_wr_data == 32'hffff_ffe5)) begin
            r_start_flag <= 1'b1;
        end else if (pio_wr_en && (pio_wr_addr == 10'h000)
                     && (pio_wr_data == 32'hffff_ff00)) begin
            r_start_flag <= 1'b0;
        end
    end

    always @(posedge pcie_clk) begin
        if (!rst_n) begin
            r_set_dma_config_en <= 1'b0;
        end else begin
            r_set_dma_config_en <= pio_wr_en
                                && (pio_wr_addr == DMA_SET_EN)
                                && (pio_wr_data == 32'h0000_0001);
        end
    end

    always @(posedge pcie_clk) begin
        if (!rst_n) begin
            r_ch0_base_addr  <= 64'd0;
            r_ch0_base_addr2 <= 64'd0;
            r_ch0_base_addr3 <= 64'd0;
            r_ch0_base_addr4 <= 64'd0;
        end else if (pio_wr_en) begin
            case (pio_wr_addr)
                DMA_ADDR_L:  r_ch0_base_addr[31:0]   <= pio_wr_data;
                DMA_ADDR_H:  r_ch0_base_addr[63:32]  <= pio_wr_data;
                DMA_ADDR1_L: r_ch0_base_addr2[31:0]  <= pio_wr_data;
                DMA_ADDR1_H: r_ch0_base_addr2[63:32] <= pio_wr_data;
                DMA_ADDR2_L: r_ch0_base_addr3[31:0]  <= pio_wr_data;
                DMA_ADDR2_H: r_ch0_base_addr3[63:32] <= pio_wr_data;
                DMA_ADDR3_L: r_ch0_base_addr4[31:0]  <= pio_wr_data;
                DMA_ADDR3_H: r_ch0_base_addr4[63:32] <= pio_wr_data;
                default: begin
                    r_ch0_base_addr  <= r_ch0_base_addr;
                    r_ch0_base_addr2 <= r_ch0_base_addr2;
                    r_ch0_base_addr3 <= r_ch0_base_addr3;
                    r_ch0_base_addr4 <= r_ch0_base_addr4;
                end
            endcase
        end
    end

    /* The completion bit and its buffer index are latched atomically.  A new
       completion has priority over a simultaneous software read/clear. */
    always @(posedge pcie_clk) begin
        if (!rst_n) begin
            r_wr_frame_done    <= 1'b0;
            r_wr_index_latched <= 2'd0;
        end else if (i_wr_frame_done) begin
            r_wr_frame_done    <= 1'b1;
            r_wr_index_latched <= i_wr_index;
        end else if (pio_rd_en && (pio_rd_addr == WR_FRAME_DONE)) begin
            r_wr_frame_done <= 1'b0;
        end
    end

    /* Marker row bookkeeping.
       i_marker_tgl toggles once per successful detection (at most once per
       frame, ~60 Hz).  Its edge is detected in pcie_clk and used both to bump
       a frame counter and to raise the sticky "new" bit.  A new detection has
       priority over a simultaneous software read/clear, same rule as the
       frame-done bit above.  Both counters live in pcie_clk, so nothing
       multi-bit has to cross a clock domain. */
    always @(posedge pcie_clk) begin
        if (!rst_n) begin
            r_marker_tgl_d     <= 1'b0;
            r_marker_frame_cnt <= 12'd0;
            r_marker_new       <= 1'b0;
        end else begin
            r_marker_tgl_d <= i_marker_tgl;
            if (i_marker_tgl ^ r_marker_tgl_d) begin
                r_marker_frame_cnt <= r_marker_frame_cnt + 12'd1;
                r_marker_new       <= 1'b1;
            end else if (pio_rd_en && (pio_rd_addr == MARKER_STATUS)) begin
                r_marker_new       <= 1'b0;
            end
        end
    end

    always @(posedge pcie_clk) begin
        if (!rst_n) begin
            r_pio_rd_data <= 32'd0;
        end else if (pio_rd_en) begin
            case (pio_rd_addr)
                WR_FRAME_DONE:
                    r_pio_rd_data <= {29'd0, r_wr_index_latched,
                                      r_wr_frame_done};
                MARKER_STATUS:
                    r_pio_rd_data <= {1'b1,            // [31]    presence probe
                                      15'd0,           // [30:16] reserved
                                      r_marker_frame_cnt,  // [15:4]
                                      r_marker_new,    // [3]  sticky,读后清
                                      i_marker_valid,  // [2]  曾成功识别过
                                      i_marker_ok,     // [1]  最近一帧识别成功
                                      i_marker_mode};  // [0]  1=白/拼接 0=黑/单路
                default:
                    r_pio_rd_data <= 32'd0;
            endcase
        end else begin
            r_pio_rd_data <= 32'd0;
        end
    end

    assign start_flag         = r_start_flag;
    assign set_dma_config_en  = r_set_dma_config_en;
    assign pio_rd_data        = r_pio_rd_data;
    assign o_ch0_base_addr    = r_ch0_base_addr;
    assign o_ch0_base_addr2   = r_ch0_base_addr2;
    assign o_ch0_base_addr3   = r_ch0_base_addr3;
    assign o_ch0_base_addr4   = r_ch0_base_addr4;

endmodule

`default_nettype wire
