`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 260911: generic dual-clock asynchronous FIFO, 18-bit x 1024, pure RTL
// (no PDS IP generation needed). Gray-coded pointers with 2-FF synchronizers.
//
// Data width 18 bit = {vs, de, pixel[15:0]} so the frame boundary and valid
// travel with the pixels (a source can be switched mid-stream, the read side
// must never re-synthesize them).
//
// Write enable is qualified by the user (write only when vs|de is meaningful)
// so the average write rate stays below the read rate even for the 125MHz
// Ethernet domain. Read side pops whenever not empty (rd_en = ~rd_empty).
// Non-show-ahead: rd_data is valid one cycle after rd_en — the whole
// {vs,de,data} bundle keeps its internal ordering.
//
// 260911 fix: wr_full / rd_empty are now *registered* (classic Cummings
// style-1). The previous version fed rgray_next / wgray_next straight into the
// flags while the pointer increments were gated by those same flags
// (rbin_next = rbin + (rd_en & ~rd_empty), rd_en = ~rd_empty) — a real
// combinational loop that synthesis cannot resolve. Registering the flags
// breaks the loop and adds one cycle of pessimism (safe direction).
//////////////////////////////////////////////////////////////////////////////////
module afifo_18i_18o_1024(
    input  wire        wr_clk,
    input  wire        wr_rst,         // async clear, 1 active
    input  wire        wr_en,
    input  wire [17:0] wr_data,
    output wire        wr_full,
    output wire [9:0]  wr_water_level,
    output wire        almost_full,
    input  wire        rd_clk,
    input  wire        rd_rst,         // async clear, 1 active
    input  wire        rd_en,
    output reg  [17:0] rd_data,
    output wire        rd_empty,
    output wire [9:0]  rd_water_level,
    output wire        almost_empty
);
    localparam AW  = 10;              // 1024 entries
    localparam PTW = AW + 1;          // 11-bit wrap-around pointers

    // dual-port RAM, two clocks, no reset
    reg [17:0] mem [0:1023];

    // ---------------- registered status flags ----------------
    reg wr_full_r, rd_empty_r;

    // ---------------- write domain pointers ----------------
    // 260912e fix: the increment used {PTW{...}} (REPLICATION), so a true
    // condition added 2047 (= -1 mod 2048) instead of +1 — both pointers
    // walked BACKWARDS, the gray full/empty flags were garbage and the
    // {vs,de,data} bundle order through the FIFO was scrambled (the whole
    // single-view chain: shear / freeze / strips / stale-slot playback).
    // Replication -> zero-extended 1-bit add, +1 per accepted entry.
    reg [PTW-1:0] wbin, wptr_g;
    wire [PTW-1:0] wbin_next  = wbin + {{(PTW-1){1'b0}}, wr_en & ~wr_full_r};
    wire [PTW-1:0] wgray_next = (wbin_next >> 1) ^ wbin_next;

    always @(posedge wr_clk or posedge wr_rst) begin
        if (wr_rst) begin
            wbin   <= {PTW{1'b0}};
            wptr_g <= {PTW{1'b0}};
        end else begin
            wbin   <= wbin_next;
            wptr_g <= wgray_next;
        end
    end

    always @(posedge wr_clk) begin
        if (wr_en & ~wr_full_r)
            mem[wbin[AW-1:0]] <= wr_data;
    end

    // ---------------- read domain pointers ----------------
    reg [PTW-1:0] rbin, rptr_g;
    wire [PTW-1:0] rbin_next  = rbin + {{(PTW-1){1'b0}}, rd_en & ~rd_empty_r};
    wire [PTW-1:0] rgray_next = (rbin_next >> 1) ^ rbin_next;

    always @(posedge rd_clk or posedge rd_rst) begin
        if (rd_rst) begin
            rbin   <= {PTW{1'b0}};
            rptr_g <= {PTW{1'b0}};
        end else begin
            rbin   <= rbin_next;
            rptr_g <= rgray_next;
        end
    end

    // registered read port (BRAM-style sync read, 1 cycle latency)
    always @(posedge rd_clk) begin
        if (rd_en & ~rd_empty_r)
            rd_data <= mem[rbin[AW-1:0]];
    end

    // ---------------- pointer synchronizers (2-FF each) ----------------
    reg [PTW-1:0] rq2_wptr_g0, rq2_wptr_g1;   // wptr into read domain
    reg [PTW-1:0] wq2_rptr_g0, wq2_rptr_g1;   // rptr into write domain
    always @(posedge rd_clk or posedge rd_rst) begin
        if (rd_rst) begin
            rq2_wptr_g0 <= {PTW{1'b0}};
            rq2_wptr_g1 <= {PTW{1'b0}};
        end else begin
            rq2_wptr_g0 <= wptr_g;
            rq2_wptr_g1 <= rq2_wptr_g0;
        end
    end
    always @(posedge wr_clk or posedge wr_rst) begin
        if (wr_rst) begin
            wq2_rptr_g0 <= {PTW{1'b0}};
            wq2_rptr_g1 <= {PTW{1'b0}};
        end else begin
            wq2_rptr_g0 <= rptr_g;
            wq2_rptr_g1 <= wq2_rptr_g0;
        end
    end

    // ---------------- status flags ----------------
    // empty (read domain): next read would catch up with the synced write ptr
    always @(posedge rd_clk or posedge rd_rst) begin
        if (rd_rst) rd_empty_r <= 1'b1;                    // reset -> empty
        else        rd_empty_r <= (rgray_next == rq2_wptr_g1);
    end
    // full (write domain): next write would touch the synced read ptr's
    // inverted top bits
    // 260912e fix: rgray_hi was built from rq2_wptr_g1 (the synced WRITE
    // pointer) instead of wq2_rptr_g1 (the synced READ pointer) — with the
    // correct pointers this condition could never be met, so wr_full never
    // asserted and overload silently overwrote unread entries.
    wire [PTW-1:0] wgray_hi = {~wq2_rptr_g1[PTW-1:PTW-2], wq2_rptr_g1[PTW-3:0]};
    always @(posedge wr_clk or posedge wr_rst) begin
        if (wr_rst) wr_full_r <= 1'b0;                     // reset -> not full
        else        wr_full_r <= (wgray_next == wgray_hi);
    end
    assign rd_empty = rd_empty_r;
    assign wr_full  = wr_full_r;

    // gray -> binary helpers for water levels
    function [PTW-1:0] g2b;
        input [PTW-1:0] g;
        integer i;
        begin
            g2b[PTW-1] = g[PTW-1];
            for (i = PTW-2; i >= 0; i = i - 1)
                g2b[i] = g2b[i+1] ^ g[i];
        end
    endfunction

    wire [PTW-1:0] rbin_in_wdom = g2b(wq2_rptr_g1);   // read count seen by writer
    wire [PTW-1:0] wbin_in_rdom = g2b(rq2_wptr_g1);   // write count seen by reader

    assign wr_water_level = wbin[AW-1:0] - rbin_in_wdom[AW-1:0];
    assign rd_water_level = wbin_in_rdom[AW-1:0] - rbin[AW-1:0];

    assign almost_full  = (wr_water_level >= 10'd1016);
    assign almost_empty = (rd_water_level <= 10'd2);

endmodule
