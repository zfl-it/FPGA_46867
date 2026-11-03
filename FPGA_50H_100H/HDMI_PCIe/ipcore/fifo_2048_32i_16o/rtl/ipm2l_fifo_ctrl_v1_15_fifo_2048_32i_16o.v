
//////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2019 PANGO MICROSYSTEMS, INC
// ALL RIGHTS REVERVED.
//
// THE SOURCE CODE CONTAINED HEREIN IS PROPRIETARY TO PANGO MICROSYSTEMS, INC.
// IT SHALL NOT BE REPRODUCED OR DISCLOSED IN WHOLE OR IN PART OR USED BY
// PARTIES WITHOUT WRITTEN AUTHORIZATION FROM THE OWNER.
//
//////////////////////////////////////////////////////////////////////////////
//
// Library:
// Filename:ipm2l_fifo_ctrl.v
//
//////////////////////////////////////////////////////////////////////////////

module ipm2l_fifo_ctrl_v1_15_fifo_2048_32i_16o #(
    parameter  c_WR_DEPTH_WIDTH   = 9             ,
    parameter  c_RD_DEPTH_WIDTH   = 9             ,
    parameter  c_FIFO_TYPE        = "ASYN"        ,
    parameter  c_ALMOST_FULL_NUM  = 508           ,
    parameter  c_ALMOST_EMPTY_NUM = 4             ,
    parameter  c_PREFETCH_EN      = 0             ,
    parameter  c_DATA_VALID_EN    = 0             ,
    parameter  c_OUTPUT_REG       = 0             ,
    parameter  c_FAB_REG          = 0             ,
    parameter  c_RD_CLK_OR_POL_INV= 0
) (
    input  wire                           wclk            ,
    input  wire                           w_en            ,
    output wire [c_WR_DEPTH_WIDTH-1 : 0]  waddr           ,
    input  wire                           wrst            ,
    output wire                           wfull           ,
    output reg                            almost_full=1'b0,
    output wire [c_WR_DEPTH_WIDTH : 0]    wr_water_level  ,

    input  wire                           rclk            ,
    input  wire                           r_en            ,
    output wire [c_RD_DEPTH_WIDTH-1 : 0]  raddr           ,
    input  wire                           rrst            ,
    output wire                           rempty          ,
    output wire [c_RD_DEPTH_WIDTH : 0]    rd_water_level  ,
    output reg                            almost_empty=1'b1,
    output reg                            data_valid=1'b0 ,
    output wire                           rd_en_i         ,
    input  wire                           rd_oce          ,
    output wire                           rd_oce_i        

);

//**************************************************************************************************************
//declare inner variables
//write address operation variables
//write pointer
reg [c_WR_DEPTH_WIDTH : 0]  wptr = {(c_WR_DEPTH_WIDTH+1){1'b0}}                      /* synthesis syn_preserve=1 */ ; 
//1st read-domain to write-domain synchronizer
reg [c_RD_DEPTH_WIDTH : 0]  wrptr1 = {(c_RD_DEPTH_WIDTH+1){1'b0}}                    /* synthesis syn_preserve=1 */ ;
reg [c_RD_DEPTH_WIDTH : 0]  wrptr2 = {(c_RD_DEPTH_WIDTH+1){1'b0}}          ;          //2nd read-domain to write-domain synchronizer
reg [c_WR_DEPTH_WIDTH : 0]  wbin = {(c_WR_DEPTH_WIDTH+1){1'b0}}            ;          //write current binary  pointer
reg [c_WR_DEPTH_WIDTH : 0]  wbnext = {(c_WR_DEPTH_WIDTH+1){1'b0}}          ;          //write next binary  pointer
reg [c_WR_DEPTH_WIDTH : 0]  wgnext = {(c_WR_DEPTH_WIDTH+1){1'b0}}          ;          //wriet next gray pointer

//wire                        wgnext_2ndmsb   ;          //the second MSB of wgnext
//wire                        wrptr2_2ndmsb   ;          //the second MSB of wrptr2

//read address operation variables
//read pointer
reg [c_RD_DEPTH_WIDTH : 0]  rptr = {(c_RD_DEPTH_WIDTH+1){1'b0}}                      /* synthesis syn_preserve=1 */  ;
//1st  write-domain to read-domain synchronizer
reg [c_WR_DEPTH_WIDTH : 0]  rwptr1 = {(c_WR_DEPTH_WIDTH+1){1'b0}}                    /* synthesis syn_preserve=1 */  ;
reg [c_WR_DEPTH_WIDTH : 0]  rwptr2 = {(c_WR_DEPTH_WIDTH+1){1'b0}}          ;          //2nd  write-domain to read-domain synchronizer
reg [c_RD_DEPTH_WIDTH : 0]  rbin = {(c_RD_DEPTH_WIDTH+1){1'b0}}            ;          //read current binary  pointer
reg [c_RD_DEPTH_WIDTH : 0]  rbnext = {(c_RD_DEPTH_WIDTH+1){1'b0}}          ;          //read next binary  pointer
reg [c_RD_DEPTH_WIDTH : 0]  rgnext = {(c_RD_DEPTH_WIDTH+1){1'b0}}          ;          //read next gray pointer

reg [c_RD_DEPTH_WIDTH : 0]  wrptr2_b = {(c_RD_DEPTH_WIDTH+1){1'b0}}        ;          //wrptr2 into binary
reg [c_WR_DEPTH_WIDTH : 0]  rwptr2_b = {(c_WR_DEPTH_WIDTH+1){1'b0}}        ;          //rwptr2 into binary

reg                         asyn_wfull = 1'b0          ;
reg                         asyn_almost_full = 1'b0    ;
reg                         asyn_rempty = 1'b1         ;
reg                         asyn_almost_empty          ;
reg                         syn_wfull = 1'b0           ;
reg                         syn_almost_full = 1'b0     ;
reg                         syn_rempty = 1'b1          ;
reg                         syn_almost_empty = 1'b1    ;
wire                        rempty_i                   ;
reg [c_RD_DEPTH_WIDTH : 0]  rd_water_level_prefetch = {(c_RD_DEPTH_WIDTH+1){1'b0}};
reg [c_RD_DEPTH_WIDTH : 0]  rd_water_level_i = {(c_RD_DEPTH_WIDTH+1){1'b0}};
reg [c_WR_DEPTH_WIDTH : 0]  wr_water_level_prefetch = {(c_WR_DEPTH_WIDTH+1){1'b0}};
reg [c_WR_DEPTH_WIDTH : 0]  wr_water_level_i = {(c_WR_DEPTH_WIDTH+1){1'b0}};
wire    [c_WR_DEPTH_WIDTH:0]    wwptr;
wire    [c_WR_DEPTH_WIDTH:0]    wrptr;
wire    [c_RD_DEPTH_WIDTH:0]    rwptr;
wire    [c_RD_DEPTH_WIDTH:0]    rrptr;
wire    [c_RD_DEPTH_WIDTH:0]    rbnext_prefetch;
reg     [c_RD_DEPTH_WIDTH:0]    rgnext_prefetch= {(c_WR_DEPTH_WIDTH+1){1'b0}};
reg     [c_RD_DEPTH_WIDTH:0]    rptr_prefetch = {(c_RD_DEPTH_WIDTH+1){1'b0}}                      /* synthesis syn_preserve=1 */ ;
reg     [c_RD_DEPTH_WIDTH : 0]  wrptr1_prefetch = {(c_RD_DEPTH_WIDTH+1){1'b0}}                    /* synthesis syn_preserve=1 */ ;
reg     [c_RD_DEPTH_WIDTH : 0]  wrptr2_prefetch = {(c_RD_DEPTH_WIDTH+1){1'b0}}          ;
reg     [c_RD_DEPTH_WIDTH : 0]  wrptr2_b_prefetch = {(c_RD_DEPTH_WIDTH+1){1'b0}}        ;
wire    [c_WR_DEPTH_WIDTH:0]    wrptr_prefetch;
wire    [c_WR_DEPTH_WIDTH:0]    diff_ptr_wr;
wire    [c_RD_DEPTH_WIDTH:0]    diff_ptr_rd;
integer i;

//main code
//**************************************************************************************************************
//prefetch control
generate
if(c_PREFETCH_EN == 1) begin:gen_prefetch_ctrl
    reg                         empty_preout = 1'b1        ;
    wire                        rd_en_preout               ;
    wire                        rd_oce_preout              ;
    wire                        rd_done                    ;

    assign rd_done        = r_en & !rempty;
    
    if(c_OUTPUT_REG == 1 || c_FAB_REG == 1) begin:gen_or_prefetch_ctrl
        reg   [2:0]                 shift_vld = 3'b001         ;
        reg   [1:0]                 precnt    = 2'b00          ;
        reg                         rd_en_preout_d1            ;

        assign rd_en_preout   = (~shift_vld[2] | rd_done) & ~rempty_i;
        assign rd_oce_preout  = (shift_vld[1] & rd_en_preout_d1) || (shift_vld[2] & rd_done);

        always@(posedge rclk or posedge rrst) begin
            if (rrst)
                shift_vld <= 3'b001;
            else begin
                case ({rd_en_preout, rd_done})
                    2'b10  : shift_vld <= {shift_vld[1:0], shift_vld[2]};
                    2'b01  : shift_vld <= {shift_vld[0], shift_vld[2:1]};
                    default: shift_vld <= shift_vld;
                endcase
            end
        end
        
        always@(posedge rclk or posedge rrst) begin
            if (rrst)
                rd_en_preout_d1 <= 1'b0;
            else begin
                rd_en_preout_d1 <= rd_en_preout;
            end
        end

        always@(posedge rclk or posedge rrst) begin
            if (rrst) begin
                empty_preout     <=  1'b1;
            end
            else if (rd_oce_preout) begin                          //data pre read complete
                empty_preout     <=  1'b0;
            end
            else if (!rd_oce_preout && rd_done) begin                //last data read out
                empty_preout     <=  1'b1;
            end
        end

        always@(posedge rclk or posedge rrst) begin
            if (rrst) begin
                data_valid     <=  1'b0;
            end
            else if (rd_oce_preout) begin                          //data pre read complete
                data_valid     <=  1'b1;
            end
            else if (!rd_oce_preout && rd_done) begin                //last data read out
                data_valid     <=  1'b0;
            end
        end

        always@(posedge rclk or posedge rrst) begin
            if (rrst) begin
            precnt  <= 2'd0;
            end
            else if (rd_en_preout && !rd_done) begin
                precnt  <= precnt + 2'd1;
            end
            else if (!rd_en_preout && rd_done) begin
                precnt  <= precnt - 2'd1;
            end
        end
        if (c_FIFO_TYPE == "ASYN") begin:gen_asyn_prefetch_wl
            always@(posedge rclk or posedge rrst) begin
                if(rrst)
                    rd_water_level_prefetch <= 'b0;
                else if (precnt == 2'd0) begin
                    if (rd_en_preout && !rd_done)
                        rd_water_level_prefetch <= (rwptr - rbin - rd_en_preout) + 1'b1;
                    else
                        rd_water_level_prefetch <= (rwptr - rbin - rd_en_preout);
                end
                else if (precnt == 2'd1) begin
                    if (rd_en_preout && !rd_done)
                        rd_water_level_prefetch <= (rwptr - rbin - rd_en_preout) + 2'd2;
                    else if (!rd_en_preout && rd_done)
                        rd_water_level_prefetch <= (rwptr - rbin - rd_en_preout);
                    else
                        rd_water_level_prefetch <= (rwptr - rbin - rd_en_preout) + 1'b1;
                end
                else if (precnt == 2'd2) begin
                    if (!rd_en_preout && rd_done)
                        rd_water_level_prefetch <= (rwptr - rbin - rd_en_preout) + 1'b1;
                    else
                        rd_water_level_prefetch <= (rwptr - rbin - rd_en_preout) + 2'd2;
                end
            end
            assign rbnext_prefetch = rbin - precnt + rd_done; //rbnext - precnt + rd_done - rd_en_preout
            always@(*)
            begin
                rgnext_prefetch = (rbnext_prefetch >> 1) ^ rbnext_prefetch;          //binary to gray converter
            end

            always@( posedge rclk or posedge rrst )
            begin
                if(rrst)
                begin
                    rptr_prefetch <=0;
                end
                else
                begin
                    rptr_prefetch <= rgnext_prefetch;
                end
            end

            //read domain to write domain synchronizer
            always@(posedge wclk or posedge wrst)
            begin
                if(wrst)
                    {wrptr2_prefetch,wrptr1_prefetch} <= 0;
                else
                    {wrptr2_prefetch,wrptr1_prefetch} <= {wrptr1_prefetch,rptr_prefetch};
            end

            always@(*)
            begin
                for(i = 0;i <= c_RD_DEPTH_WIDTH;i = i+1 )  //gray to binary converter
                    wrptr2_b_prefetch[i] = ^(wrptr2_prefetch >> i);
            end

            if(c_WR_DEPTH_WIDTH > c_RD_DEPTH_WIDTH)
            begin
                assign wrptr_prefetch = {wrptr2_b_prefetch,{(c_WR_DEPTH_WIDTH-c_RD_DEPTH_WIDTH){1'b0}}};
            end
            else
            begin
                assign wrptr_prefetch = wrptr2_b_prefetch[c_RD_DEPTH_WIDTH:c_RD_DEPTH_WIDTH-c_WR_DEPTH_WIDTH];
            end
            always@(posedge wclk or posedge wrst) begin
                if(wrst)
                    wr_water_level_prefetch <= {(c_WR_DEPTH_WIDTH+1){1'b0}};
                else begin
                    wr_water_level_prefetch <= wwptr - wrptr_prefetch;
                end
            end
        end
        else begin:gen_sync_prefetch_wl
            always@(posedge rclk or posedge rrst) begin
                if(rrst)
                    rd_water_level_prefetch <= 'b0;
                else if (precnt == 2'd0) begin
                    if (rd_en_preout && !rd_done)
                        rd_water_level_prefetch <= (rwptr - rbin - rd_en_preout) + 1'b1;
                    else
                        rd_water_level_prefetch <= (rwptr - rbin - rd_en_preout);
                end
                else if (precnt == 2'd1) begin
                    if (rd_en_preout && !rd_done)
                        rd_water_level_prefetch <= (rwptr - rbin - rd_en_preout) + 2'd2;
                    else if (!rd_en_preout && rd_done)
                        rd_water_level_prefetch <= (rwptr - rbin - rd_en_preout);
                    else
                        rd_water_level_prefetch <= (rwptr - rbin - rd_en_preout) + 1'b1;
                end
                else if (precnt == 2'd2) begin
                    if (!rd_en_preout && rd_done)
                        rd_water_level_prefetch <= (rwptr - rbin - rd_en_preout) + 1'b1;
                    else
                        rd_water_level_prefetch <= (rwptr - rbin - rd_en_preout) + 2'd2;
                end
            end
            assign rbnext_prefetch = rbin - precnt + rd_done; //rbnext - precnt + rd_done - rd_en_preout;
            if(c_WR_DEPTH_WIDTH > c_RD_DEPTH_WIDTH)
            begin
                assign wrptr_prefetch = {rbnext_prefetch,{(c_WR_DEPTH_WIDTH-c_RD_DEPTH_WIDTH){1'b0}}};
            end
            else
            begin
                assign wrptr_prefetch = rbnext_prefetch[c_RD_DEPTH_WIDTH:c_RD_DEPTH_WIDTH-c_WR_DEPTH_WIDTH];
            end
            always@(posedge wclk or posedge wrst) begin
                if(wrst)
                    wr_water_level_prefetch <= {(c_WR_DEPTH_WIDTH+1){1'b0}};
                else begin
                    wr_water_level_prefetch <= wwptr - wrptr_prefetch;
                end
            end
        end
    end
    else begin:gen_nor_prefetch_ctrl
        reg   [1:0]                 shift_vld = 2'b01          ;
        reg                         precnt    = 1'b0           ;
 
        assign rd_en_preout   = (~shift_vld[1] | rd_done) & ~rempty_i;
        assign rd_oce_preout  = 1'b0;

        always@(posedge rclk or posedge rrst) begin
            if (rrst)
                shift_vld <= 2'b01;
            else begin
                case ({rd_en_preout, rd_done})
                    2'b10  : shift_vld <= {shift_vld[0], shift_vld[1]};
                    2'b01  : shift_vld <= {shift_vld[0], shift_vld[1]};
                    default: shift_vld <= shift_vld;
                endcase
            end
        end
        always@(posedge rclk or posedge rrst) begin
            if (rrst) begin
                empty_preout     <=  1'b1;
            end
            else if (rd_en_preout) begin                          //data pre read complete
                empty_preout     <=  1'b0;
            end
            else if (!rd_en_preout && rd_done) begin                //last data read out
                empty_preout     <=  1'b1;
            end
        end
        always@(posedge rclk or posedge rrst) begin
            if (rrst) begin
                data_valid     <=  1'b0;
            end
            else if (rd_en_preout) begin                          //data pre read complete
                data_valid     <=  1'b1;
            end
            else if (!rd_en_preout && rd_done) begin                //last data read out
                data_valid     <=  1'b0;
            end
        end
        always@(posedge rclk or posedge rrst) begin
            if (rrst) begin
                precnt  <= 1'd0;
            end
            else if (rd_en_preout && !rd_done) begin
                precnt  <= precnt + 1'd1;
            end
            else if (!rd_en_preout && rd_done) begin
                precnt  <= precnt - 1'd1;
            end
        end
        if (c_FIFO_TYPE == "ASYN") begin:gen_asyn_prefetch_wl
            always@(posedge rclk or posedge rrst) begin
                if(rrst)
                    rd_water_level_prefetch <= 'b0;
                else if (precnt == 2'd0) begin
                    if (rd_en_preout && !rd_done)
                        rd_water_level_prefetch <= (rwptr - rbin - rd_en_preout) + 1'b1;
                    else
                        rd_water_level_prefetch <= (rwptr - rbin - rd_en_preout);
                end
                else if (precnt == 2'd1) begin
                    if (!rd_en_preout && rd_done)
                        rd_water_level_prefetch <= (rwptr - rbin - rd_en_preout);
                    else
                        rd_water_level_prefetch <= (rwptr - rbin - rd_en_preout) + 1'b1;
                end
            end
            assign rbnext_prefetch = rbin - precnt + rd_done; //rbnext - precnt + rd_done - rd_en_preout
            always@(*)
            begin
                rgnext_prefetch = (rbnext_prefetch >> 1) ^ rbnext_prefetch;          //binary to gray converter
            end

            always@( posedge rclk or posedge rrst )
            begin
                if(rrst)
                begin
                    rptr_prefetch <=0;
                end
                else
                begin
                    rptr_prefetch <= rgnext_prefetch;
                end
            end

            //read domain to write domain synchronizer
            always@(posedge wclk or posedge wrst)
            begin
                if(wrst)
                    {wrptr2_prefetch,wrptr1_prefetch} <= 0;
                else
                    {wrptr2_prefetch,wrptr1_prefetch} <= {wrptr1_prefetch,rptr_prefetch};
            end

            always@(*)
            begin
                for(i = 0;i <= c_RD_DEPTH_WIDTH;i = i+1 )  //gray to binary converter
                    wrptr2_b_prefetch[i] = ^(wrptr2_prefetch >> i);
            end

            if(c_WR_DEPTH_WIDTH > c_RD_DEPTH_WIDTH)
            begin
                assign wrptr_prefetch = {wrptr2_b_prefetch,{(c_WR_DEPTH_WIDTH-c_RD_DEPTH_WIDTH){1'b0}}};
            end
            else
            begin
                assign wrptr_prefetch = wrptr2_b_prefetch[c_RD_DEPTH_WIDTH:c_RD_DEPTH_WIDTH-c_WR_DEPTH_WIDTH];
            end
            always@(posedge wclk or posedge wrst) begin
                if(wrst)
                    wr_water_level_prefetch <= {(c_WR_DEPTH_WIDTH+1){1'b0}};
                else begin
                    wr_water_level_prefetch <= wwptr - wrptr_prefetch;
                end
            end
        end
        else begin:gen_syn_prefetch_wl
            always@(posedge rclk or posedge rrst) begin
                if(rrst)
                    rd_water_level_prefetch <= 'b0;
                else if (precnt == 2'd0) begin
                    if (rd_en_preout && !rd_done)
                        rd_water_level_prefetch <= (rwptr - rbin - rd_en_preout) + 1'b1;
                    else
                        rd_water_level_prefetch <= (rwptr - rbin - rd_en_preout);
                end
                else if (precnt == 2'd1) begin
                    if (!rd_en_preout && rd_done)
                        rd_water_level_prefetch <= (rwptr - rbin - rd_en_preout);
                    else
                        rd_water_level_prefetch <= (rwptr - rbin - rd_en_preout) + 1'b1;
                end
            end
            assign rbnext_prefetch = rbin - precnt + rd_done; //rbnext - precnt + rd_done - rd_en_preout;
            if(c_WR_DEPTH_WIDTH > c_RD_DEPTH_WIDTH)
            begin
                assign wrptr_prefetch = {rbnext_prefetch,{(c_WR_DEPTH_WIDTH-c_RD_DEPTH_WIDTH){1'b0}}};
            end
            else
            begin
                assign wrptr_prefetch = rbnext_prefetch[c_RD_DEPTH_WIDTH:c_RD_DEPTH_WIDTH-c_WR_DEPTH_WIDTH];
            end
            always@(posedge wclk or posedge wrst) begin
                if(wrst)
                    wr_water_level_prefetch <= {(c_WR_DEPTH_WIDTH+1){1'b0}};
                else begin
                    wr_water_level_prefetch <= wwptr - wrptr_prefetch;
                end
            end
        end
    end



    assign rd_en_i = rd_en_preout;
    assign rd_oce_i = rd_oce_preout;
    assign rempty  = empty_preout;
    assign wr_water_level = wr_water_level_prefetch;
    assign rd_water_level = rd_water_level_prefetch;
end
else begin:gen_no_prefetch_ctrl
    if(c_OUTPUT_REG == 0 && c_FAB_REG == 0) begin:gen_std_fifo_vld_nor
        always@(posedge rclk or posedge rrst) begin
            if(rrst)
                data_valid <= 1'b0;
            else
                data_valid <= r_en;
        end
    end
    else if(c_OUTPUT_REG + c_FAB_REG == 1) begin:gen_std_fifo_vld_1or
        reg rd_en_d1;
        if(c_RD_CLK_OR_POL_INV == 0) begin:gen_orclk_noinv
            always@(posedge rclk or posedge rrst) begin
                if(rrst) begin
                    data_valid <= 1'b0;
                    rd_en_d1 <= 1'b0;    
                end    
                else begin
                    data_valid <= rd_en_d1;
                    rd_en_d1 <= r_en;
                end
            end
        end
        else begin:gen_orclk_inv
            always@(posedge rclk or posedge rrst) begin
                if(rrst) begin
                    rd_en_d1 <= 1'b0;    
                end    
                else begin
                    rd_en_d1 <= r_en;
                end
            end
            always@(negedge rclk or posedge rrst) begin
                if(rrst) begin
                    data_valid <= 1'b0;
                end    
                else begin
                    data_valid <= rd_en_d1;
                end
            end  
        end
    end
    else if(c_OUTPUT_REG + c_FAB_REG == 2) begin:gen_std_fifo_vld_2or
        reg rd_en_d1;
        reg rd_en_d2;
        if(c_RD_CLK_OR_POL_INV == 0) begin:gen_orclk_noinv
            always@(posedge rclk or posedge rrst) begin
                if(rrst) begin
                    data_valid <= 1'b0;
                    rd_en_d1 <= 1'b0; 
                    rd_en_d2 <= 1'b0;    
                end    
                else begin
                    data_valid <= rd_en_d2;
                    rd_en_d1 <= r_en;
                    rd_en_d2 <= rd_en_d1;
                end
            end
        end
        else begin:gen_orclk_inv
            always@(posedge rclk or posedge rrst) begin
                if(rrst) begin
                    rd_en_d1 <= 1'b0; 
                end    
                else begin
                    rd_en_d1 <= r_en;
                end
            end
            always@(negedge rclk or posedge rrst) begin
                if(rrst) begin
                    data_valid <= 1'b0;
                    rd_en_d2 <= 1'b0;    
                end    
                else begin
                    data_valid <= rd_en_d2;
                    rd_en_d2 <= rd_en_d1;
                end
            end    
        end
    end
    assign rd_en_i = r_en;
    assign rd_oce_i = rd_oce;
    assign rempty  = rempty_i;
    assign wr_water_level = wr_water_level_i;
    assign rd_water_level = rd_water_level_i;
end
endgenerate

generate
    if(c_FIFO_TYPE == "ASYN")
    begin:ASYN_CTRL
        //write gray pointer generate

        always@(*)
        begin
            if(!wfull)
                wbnext = wbin + w_en;
            else
                wbnext = wbin;
        end

        always@(*)
        begin
            wgnext = (wbnext >> 1) ^ wbnext;          //binary to gray converter
        end

        always@( posedge wclk or posedge wrst )
        begin
            if(wrst)
            begin
                wptr <=0;
                wbin <=0;
            end
            else
            begin
               wptr <= wgnext;
               wbin <= wbnext;
            end
        end

        //read domain to write domain synchronizer
        always@(posedge wclk or posedge wrst)
        begin
            if(wrst)
                {wrptr2,wrptr1} <= 0;
            else
                {wrptr2,wrptr1} <= {wrptr1,rptr};
        end

        always@(*)
        begin
            for(i = 0;i <= c_RD_DEPTH_WIDTH;i = i+1 )  //gray to binary converter
                wrptr2_b[i] = ^(wrptr2 >> i);
        end

//        //generate fifo write full flag
//        assign  wgnext_2ndmsb = wgnext[c_WR_DEPTH_WIDTH] ^ wgnext[c_WR_DEPTH_WIDTH-1];
//        assign  wrptr2_2ndmsb = wrptr2[c_WR_DEPTH_WIDTH] ^ wrptr2[c_WR_DEPTH_WIDTH-1];

        //**************************************************************************************************************
        //read gray pointer generate

        always@(*)
        begin
            if(!rempty_i)
                rbnext = rbin + rd_en_i;
            else
                rbnext = rbin;
            rgnext = (rbnext >> 1) ^ rbnext;          //binary to gray converter
        end

        always@( posedge rclk or posedge rrst )
        begin
            if(rrst)
            begin
                rptr <=0;
                rbin <=0;
            end
            else
            begin
                rptr <= rgnext;
                rbin <= rbnext;
            end
        end

        //read domain to write domain synchronizer
        always@(posedge rclk or posedge rrst)
        begin
            if(rrst)
                {rwptr2,rwptr1} <= 0;
            else
                {rwptr2,rwptr1} <= {rwptr1,wptr};
        end

        always@(*)
        begin
            for(i = 0;i <= c_WR_DEPTH_WIDTH;i = i+1 )  //gray to binary converter
                rwptr2_b[i] = ^(rwptr2 >> i);
        end

        if(c_WR_DEPTH_WIDTH > c_RD_DEPTH_WIDTH)
        begin
            assign wwptr = wbnext;
            assign wrptr = {wrptr2_b,{(c_WR_DEPTH_WIDTH-c_RD_DEPTH_WIDTH){1'b0}}};
            assign rwptr = rwptr2_b[c_WR_DEPTH_WIDTH:c_WR_DEPTH_WIDTH-c_RD_DEPTH_WIDTH];
            assign rrptr = rbnext;
        end
        else
        begin
            assign wwptr = wbnext;
            assign wrptr = wrptr2_b[c_RD_DEPTH_WIDTH:c_RD_DEPTH_WIDTH-c_WR_DEPTH_WIDTH];
            assign rwptr = {rwptr2_b,{(c_RD_DEPTH_WIDTH-c_WR_DEPTH_WIDTH){1'b0}}};
            assign rrptr = rbnext;
        end

        //generate async_fifo write full flag
        always@(posedge wclk or posedge wrst)
        begin
            if(wrst)
                asyn_wfull <= 1'b0;
            else
                asyn_wfull <= ((wwptr[c_WR_DEPTH_WIDTH] != wrptr[c_WR_DEPTH_WIDTH])
                            && (wwptr[c_WR_DEPTH_WIDTH-1:0] == wrptr[c_WR_DEPTH_WIDTH-1:0]));
        end

        //generate async_fifo read empty flag generate
        always@(posedge rclk or posedge rrst)
        begin
            if(rrst)
                asyn_rempty <= 1'b1;
            else
                asyn_rempty <= (rrptr == rwptr);
        end
    end
    else
    begin:SYN_CTRL
        //write operation
        always@(*)
        begin
            if(!wfull)
                wbnext = wptr + w_en;
            else
                wbnext = wptr;
        end

        always@(*)
        begin
            wgnext =  wbnext;    // syn fifo
        end

        always@( posedge wclk or posedge wrst )
        begin
            if(wrst)
            begin
                wptr <=0;
                wbin <=0;
            end
            else
            begin
                wptr <= wgnext;
                wbin <= wbnext;
            end
        end

        always@(*)
        begin
            wrptr2 = rptr;    // syn fifo
        end

        always@(*)
        begin
            wrptr2_b = rptr;    // syn fifo
        end

//        //generate fifo write full flag
//        assign  wgnext_2ndmsb = wgnext[c_WR_DEPTH_WIDTH-1];
//        assign  wrptr2_2ndmsb = wrptr2[c_WR_DEPTH_WIDTH-1];

        //**************************************************************************************************************
        //read operation
        always@(*)
        begin
            if(!rempty_i)
                rbnext = rptr + rd_en_i;
            else
                rbnext = rptr;
        end

        always@(*)
        begin
            rgnext =  rbnext;
        end

        always@( posedge rclk or posedge rrst )
        begin
            if(rrst)
            begin
                rptr <=0;
                rbin <=0;
            end
            else
            begin
                rptr <= rgnext;
                rbin <= rbnext;
            end
        end

        always@(*)
        begin
            rwptr2   =  wptr;    //syn fifo
        end

        always@(*)
        begin
            rwptr2_b =  wptr;    //syn fifo
        end
        //generate sync_fifo write full flag
        if(c_WR_DEPTH_WIDTH > c_RD_DEPTH_WIDTH)
        begin
            assign wwptr = wbnext;
            assign wrptr = {rbnext,{(c_WR_DEPTH_WIDTH-c_RD_DEPTH_WIDTH){1'b0}}};
            assign rwptr = wbnext[c_WR_DEPTH_WIDTH:c_WR_DEPTH_WIDTH-c_RD_DEPTH_WIDTH];
            assign rrptr = rbnext;
        end
        else
        begin
            assign wwptr = wbnext;
            assign wrptr = rbnext[c_RD_DEPTH_WIDTH:c_RD_DEPTH_WIDTH-c_WR_DEPTH_WIDTH];
            assign rwptr = {wbnext,{(c_RD_DEPTH_WIDTH-c_WR_DEPTH_WIDTH){1'b0}}};
            assign rrptr = rbnext;
        end
        always@(posedge wclk or posedge wrst)
        begin
            if(wrst)
                syn_wfull <= 1'b0;
            else
                syn_wfull <= ((wwptr[c_WR_DEPTH_WIDTH] != wrptr[c_WR_DEPTH_WIDTH])
                           && (wwptr[c_WR_DEPTH_WIDTH-1:0] == wrptr[c_WR_DEPTH_WIDTH-1:0]));
        end

        //generate sync_fifo read empty flag generate
        always@(posedge rclk or posedge rrst)
        begin
            if(rrst)
                syn_rempty <= 1'b1;
            else
                syn_rempty <= (rwptr == rrptr);
        end
    end
endgenerate

//write  flex memory address generate
assign waddr = wbin[c_WR_DEPTH_WIDTH-1:0];

//generate fifo write full flag
assign wfull = (c_FIFO_TYPE == "ASYN") ? asyn_wfull : syn_wfull;

//generate fifo write almost full flag
localparam WR_RD_RATIO = (c_WR_DEPTH_WIDTH > c_RD_DEPTH_WIDTH) ? (c_WR_DEPTH_WIDTH - c_RD_DEPTH_WIDTH) : 0;
localparam ALMOST_FULL_NUM_ADJ = c_PREFETCH_EN == 0 ? c_ALMOST_FULL_NUM : c_ALMOST_FULL_NUM-(2*(2**WR_RD_RATIO));
always@(posedge wclk or posedge wrst) begin
    if(wrst)
        almost_full <= 1'b0;
    else
        almost_full <= (diff_ptr_wr >= ALMOST_FULL_NUM_ADJ);
end
//generate write water level flag
assign diff_ptr_wr = wwptr - wrptr;
always@(posedge wclk or posedge wrst)
begin
    if(wrst)
        wr_water_level_i <= 'b0;
    else
        wr_water_level_i <= diff_ptr_wr ;
end

//read flex memory address generate
assign  raddr = rbin[c_RD_DEPTH_WIDTH-1:0];

//fifo read empty flag generate
assign rempty_i = (c_FIFO_TYPE == "ASYN") ? asyn_rempty : syn_rempty;

//generate fifo read almost empty flag
localparam ALMOST_EMPTY_NUM_ADJ = c_PREFETCH_EN == 0 ? c_ALMOST_EMPTY_NUM : c_ALMOST_EMPTY_NUM-2;
always@(posedge rclk or posedge rrst) begin
    if(rrst)
        almost_empty <= 1'b1;
    else
        almost_empty <= (diff_ptr_rd<=ALMOST_EMPTY_NUM_ADJ);
end

//generate read water level flag
assign diff_ptr_rd = rwptr - rrptr;
always@(posedge rclk or posedge rrst)
begin
    if(rrst)
        rd_water_level_i <= 'b0;
    else
        rd_water_level_i <= diff_ptr_rd;
end

endmodule
