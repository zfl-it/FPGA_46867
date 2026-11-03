`timescale 1ns / 1ps
`default_nettype none

/* 1280x720 timing generator.  All logic is synchronous to clk. */
module sync_vg #(
    parameter integer X_BITS    = 12,
    parameter integer Y_BITS    = 12,
    parameter integer V_TOTAL   = 750,
    parameter integer V_FP      = 5,
    parameter integer V_BP      = 20,
    parameter integer V_SYNC    = 5,
    parameter integer V_ACT     = 720,
    parameter integer H_TOTAL   = 1650,
    parameter integer H_FP      = 110,
    parameter integer H_BP      = 220,
    parameter integer H_SYNC    = 40,
    parameter integer H_ACT     = 1280,
    parameter integer HV_OFFSET = 0
)(
    input  wire                  clk,
    input  wire                  rstn,
    output reg                   vs_out,
    output reg                   hs_out,
    output reg                   de_out,
    output reg                   de_re,
    output reg [X_BITS-1:0]      x_act,
    output reg [Y_BITS-1:0]      y_act
);

    reg [X_BITS-1:0] r_h_count;
    reg [Y_BITS-1:0] r_v_count;

    always @(posedge clk) begin
        if (!rstn) begin
            r_h_count <= {X_BITS{1'b0}};
        end else if (r_h_count == H_TOTAL - 1) begin
            r_h_count <= {X_BITS{1'b0}};
        end else begin
            r_h_count <= r_h_count + 1'b1;
        end
    end

    always @(posedge clk) begin
        if (!rstn) begin
            r_v_count <= {Y_BITS{1'b0}};
        end else if (r_h_count == H_TOTAL - 1) begin
            if (r_v_count == V_TOTAL - 1) begin
                r_v_count <= {Y_BITS{1'b0}};
            end else begin
                r_v_count <= r_v_count + 1'b1;
            end
        end
    end

    always @(posedge clk) begin
        if (!rstn) begin
            hs_out <= 1'b0;
        end else begin
            hs_out <= (r_h_count < H_SYNC);
        end
    end

    always @(posedge clk) begin
        if (!rstn) begin
            vs_out <= 1'b0;
        end else if ((r_v_count == 0) && (r_h_count == HV_OFFSET)) begin
            vs_out <= 1'b1;
        end else if ((r_v_count == V_SYNC)
                     && (r_h_count == HV_OFFSET)) begin
            vs_out <= 1'b0;
        end
    end

    always @(posedge clk) begin
        if (!rstn) begin
            de_out <= 1'b0;
            de_re  <= 1'b0;
        end else begin
            de_out <= ((r_v_count >= V_SYNC + V_BP)
                       && (r_v_count <= V_TOTAL - V_FP - 1)
                       && (r_h_count >= H_SYNC + H_BP)
                       && (r_h_count <= H_TOTAL - H_FP - 1));
            de_re <= ((r_v_count >= V_SYNC + V_BP)
                      && (r_v_count <= V_TOTAL - V_FP - 1)
                      && (r_h_count >= H_SYNC + H_BP - 2)
                      && (r_h_count <= H_TOTAL - H_FP - 3));
        end
    end

    always @(posedge clk) begin
        if (!rstn) begin
            x_act <= {X_BITS{1'b0}};
            y_act <= {Y_BITS{1'b0}};
        end else begin
            if (r_h_count >= H_SYNC + H_BP) begin
                x_act <= r_h_count - (H_SYNC + H_BP);
            end else begin
                x_act <= {X_BITS{1'b0}};
            end

            if (r_v_count >= V_SYNC + V_BP) begin
                y_act <= r_v_count - (V_SYNC + V_BP);
            end else begin
                y_act <= {Y_BITS{1'b0}};
            end
        end
    end

    /* V_ACT and H_ACT document the intended active resolution and are kept
       as parameters for compatibility with existing project instances. */
    wire w_active_size_parameters_unused;
    assign w_active_size_parameters_unused = (V_ACT == 0) || (H_ACT == 0);

endmodule

`default_nettype wire
