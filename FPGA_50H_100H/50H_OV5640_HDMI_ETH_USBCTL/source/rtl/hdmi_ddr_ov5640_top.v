`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Dual OV5640 -> DDR3 -> Quad-Stitch -> HDMI 720p
// Based on reference working project, added stitching and 720p output
//
// Debug: comment out one of these to test single camera
//`define DISABLE_CAM1
//`define DISABLE_CAM2
//////////////////////////////////////////////////////////////////////////////////
`define UD #1

module hdmi_ddr_ov5640_top#(
    parameter MEM_ROW_ADDR_WIDTH   = 15,
    parameter MEM_COL_ADDR_WIDTH   = 10,
    parameter MEM_BADDR_WIDTH      = 3,
    parameter MEM_DQ_WIDTH         = 32,
    parameter MEM_DQS_WIDTH        = 32/8
)(
    input                                sys_clk              ,//50Mhz
    //coms1
    output  [1:0]                        cmos_init_done       ,//OV5640 init done
    inout                                cmos1_scl            ,
    inout                                cmos1_sda            ,
    input                                cmos1_vsync          ,
    input                                cmos1_href           ,
    input                                cmos1_pclk           ,
    input   [7:0]                        cmos1_data           ,
    output                               cmos1_reset          ,
    //coms2
    inout                                cmos2_scl            ,
    inout                                cmos2_sda            ,
    input                                cmos2_vsync          ,
    input                                cmos2_href           ,
    input                                cmos2_pclk           ,
    input   [7:0]                        cmos2_data           ,
    output                               cmos2_reset          ,
    //HDMI Input (HDMI_IN2)
    input                                pixclk_hdmi_in       ,
    input                                vs_hdmi_in           ,
    input                                hs_hdmi_in           ,
    input                                de_hdmi_in           ,
    input   [23:0]                       hdmi_data_in         ,
    output                               iic_hdmi_scl         ,
    inout                                iic_hdmi_sda         ,
    //DDR
    output                               mem_rst_n            ,
    output                               mem_ck               ,
    output                               mem_ck_n             ,
    output                               mem_cke              ,
    output                               mem_cs_n             ,
    output                               mem_ras_n            ,
    output                               mem_cas_n            ,
    output                               mem_we_n             ,
    output                               mem_odt              ,
    output  [MEM_ROW_ADDR_WIDTH-1:0]     mem_a                ,
    output  [MEM_BADDR_WIDTH-1:0]        mem_ba               ,
    inout   [MEM_DQ_WIDTH/8-1:0]         mem_dqs              ,
    inout   [MEM_DQ_WIDTH/8-1:0]         mem_dqs_n            ,
    inout   [MEM_DQ_WIDTH-1:0]           mem_dq               ,
    output  [MEM_DQ_WIDTH/8-1:0]         mem_dm               ,
    output reg                           heart_beat_led       ,
    output                               ddr_init_done         ,
    //MS72xx
    output                               rstn_out             ,
    output                               iic_tx_scl           ,
    inout                                iic_tx_sda           ,
    output                               hdmi_int_led         ,
    //HDMI_OUT
    output                               pix_clk              ,
    output     reg                       vs_out               ,
    output     reg                       hs_out               ,
    output     reg                       de_out               ,
    output     reg[7:0]                  r_out                ,
    output     reg[7:0]                  g_out                ,
    output     reg[7:0]                  b_out                ,
    // User key
    input                                key_gamma            ,// KEY8 H17：261028c 起空出（端口保留，.fdc 仍绑该脚）
    // 261028c 按键（用户定稿，与串口 '0'~'6' 一一对应）：
    //   261031：J17 上的夜景修复键删除 —— 只摘掉这一路，其他键的位号、
    //           串口码位一律不动（索引保持 261028c 原样，[2] 从此空置）。
    //   [0] K18 画面切换 | [1] L15 单路亮度自适应 | [2] 空（原 J17 夜景）
    //   [3] K16 单路去雾 | [4] J16 倒车辅助线
    //   key_defog360 脚=J19 拼接增亮 | key_defog 脚=H20 拼接去雾
    //   key_gamma 脚=H17 空出（端口名是历史遗留，功能以注释为准）
    // Power-on default = HDMI (encoder reset value; 60ms boot mask swallows
    // the spurious btn_deb_fix pulses ~20ms after reset)
    input        [4:0]                   key_sel              ,
    input                                key_defog360         ,// KEY6 J19：拼接增亮 开/关（名字是历史遗留，功能见下）
    input                                key_defog            ,// KEY7 H20：拼接去雾 开/关（名字是历史遗留，功能见下）
    // 串口（115200，ASCII '0'~'6' 或二进制 0x00~0x06），与物理键并联相或：
    //   '0' 画面切换 | '1' 亮度自适应 | '2' 空（夜景已删）| '3' 单路去雾
    //   '4' 倒车辅助线 | '5' 拼接增亮 | '6' 拼接去雾（'7' 丢弃）
    input                                uart_rx              ,// R8: USB-UART RX
    // 260909: Ethernet video (CH3, bottom-right quadrant)
    input                                rgmii_rxc            ,
    input                                rgmii_rx_ctl         ,
    input   [3:0]                        rgmii_rxd            ,
    output                               rgmii_txc            ,
    output                               rgmii_tx_ctl         ,
    output   [3:0]                       rgmii_txd            ,
    output                               phy_rstn             ,
    output                               eth_udp_led          ,// A5: UDP data activity
    output                               eth_hdr55_led        ,// F8: payload[0]==55 seen
    output                               eth_extract_led       // F7: parse reached EXTRACT
);
    parameter CTRL_ADDR_WIDTH = MEM_ROW_ADDR_WIDTH + MEM_BADDR_WIDTH + MEM_COL_ADDR_WIDTH; //28
    parameter TH_1S = 27'd33000000;

    // CAM2 IO delay (GTP_IODELAY for Logos): ~25ps/step, 7-bit 0~127
    // 40 steps ≈ 1.0ns delay — adjust based on observed green-noise level
    parameter CAM2_IO_DELAY = 7'd40;

    // 260906: defog_rgb removed from the HDMI input channel; DEFOG_EN parameter
    // deleted. The defog_rgb.v source file is kept for future re-use (its bypass
    // port remains available, untouched inside defog_rgb.v).

    // Clock & Reset
    reg  [15:0] rstn_1ms;
    wire        pix_clk_w, cfg_clk, clk_25M, locked, core_clk, pll_lock;

    // Camera signals
    wire        initial_en;
    wire [15:0] cmos1_d_16bit, cmos2_d_16bit;
    wire        cmos1_href_16bit, cmos2_href_16bit;
    wire        cmos1_pclk_16bit, cmos2_pclk_16bit;
    reg  [7:0]  cmos1_d_d0, cmos2_d_d0;
    reg         cmos1_href_d0, cmos1_vsync_d0;
    reg         cmos2_href_d0, cmos2_vsync_d0;

    // CAM2 IO-delayed signals (after GTP_IODELAY)
    wire [7:0] cmos2_data_dly;
    wire       cmos2_href_dly;

    // DDR / AXI
    wire [CTRL_ADDR_WIDTH-1:0] axi_awaddr, axi_araddr;
    wire [3:0]  axi_awuser_id, axi_awlen, axi_aruser_id, axi_arlen, axi_rid;
    wire        axi_awready, axi_awvalid, axi_wready, axi_wusero_last;
    wire        axi_arready, axi_arvalid, axi_rvalid, axi_rlast;
    wire [MEM_DQ_WIDTH*8-1:0]   axi_wdata, axi_rdata;
    wire [MEM_DQ_WIDTH*8/8-1:0] axi_wstrb;

    // Burst interface
    parameter MEM_DATA_BITS = 256;
    parameter ADDR_BITS     = 25;
    parameter BUSRT_BITS    = 10;
    wire wr_burst_req, wr_burst_finish, rd_burst_req, rd_burst_finish;
    wire wr_burst_data_req, rd_burst_data_valid;
    wire [BUSRT_BITS-1:0] rd_burst_len, wr_burst_len;
    wire [ADDR_BITS-1:0]  rd_burst_addr, wr_burst_addr;
    wire [MEM_DATA_BITS-1:0] wr_burst_data, rd_burst_data;

    // CH0: CAM1
    wire ch0_rd_burst_req, ch0_wr_burst_req, ch0_rd_burst_finish, ch0_wr_burst_finish;
    wire ch0_rd_burst_data_valid, ch0_wr_burst_data_req;
    wire [BUSRT_BITS-1:0] ch0_rd_burst_len, ch0_wr_burst_len;
    wire [ADDR_BITS-1:0]  ch0_rd_burst_addr, ch0_wr_burst_addr;
    wire [MEM_DATA_BITS-1:0] ch0_rd_burst_data, ch0_wr_burst_data;
    wire ch0_read_req, ch0_read_req_ack, ch0_read_en, ch0_write_en;
    wire ch0_write_req, ch0_write_req_ack;
    wire [15:0] ch0_read_data, ch0_write_data;
    wire [1:0]  ch0_write_addr_index, ch0_read_addr_index;

    // CH1: CAM2
    wire ch1_rd_burst_req, ch1_wr_burst_req, ch1_rd_burst_finish, ch1_wr_burst_finish;
    wire ch1_rd_burst_data_valid, ch1_wr_burst_data_req;
    wire [BUSRT_BITS-1:0] ch1_rd_burst_len, ch1_wr_burst_len;
    wire [ADDR_BITS-1:0]  ch1_rd_burst_addr, ch1_wr_burst_addr;
    wire [MEM_DATA_BITS-1:0] ch1_rd_burst_data, ch1_wr_burst_data;
    wire ch1_read_req, ch1_read_req_ack, ch1_read_en, ch1_write_en;
    wire ch1_write_req, ch1_write_req_ack;
    wire [15:0] ch1_read_data, ch1_write_data;
    wire [1:0]  ch1_write_addr_index, ch1_read_addr_index;

    // 261023: 拼接四路 360P 去雾 —— 开关状态与各写时钟域的帧界锁存
    // 声明提前：下方四个 defog_rgb 实例（CH0~CH3 各一）要读 d360_*_f，而
    // toggle / CDC 的 always 块放在文件后段（依赖 key_boot_done）。
    reg         defog360_on_sys;                              // sys_clk 域开关状态
    reg         d360_c1_s0, d360_c1_s1, d360_c1_f, c1_vs_d;   // CH0 cmos1_pclk_16bit
    reg         d360_c2_s0, d360_c2_s1, d360_c2_f, c2_vs_d;   // CH1 cmos2_pclk_16bit
    reg         d360_hd_s0, d360_hd_s1, d360_hd_f, hd_vs_d;   // CH2 pixclk_hdmi_in
    reg         d360_et_s0, d360_et_s1, d360_et_f, et_vs_d;   // CH3 eth_write_clk

    // AXI master signals
    wire [3:0]  s00_axi_awid, s00_axi_arid;
    wire [63:0] s00_axi_awaddr, s00_axi_araddr;
    wire [7:0]  s00_axi_awlen, s00_axi_arlen;
    wire [2:0]  s00_axi_awsize, s00_axi_arsize;
    wire [1:0]  s00_axi_awburst, s00_axi_arburst;
    wire        s00_axi_awlock, s00_axi_arlock;
    wire [3:0]  s00_axi_awcache, s00_axi_arcache;
    wire [2:0]  s00_axi_awprot, s00_axi_arprot;
    wire [3:0]  s00_axi_awqos, s00_axi_arqos;
    wire [0:0]  s00_axi_awuser, s00_axi_aruser;
    wire        s00_axi_awvalid, s00_axi_awready;
    wire [MEM_DATA_BITS-1:0] s00_axi_wdata;
    wire [MEM_DATA_BITS/8-1:0] s00_axi_wstrb;
    wire        s00_axi_wlast, s00_axi_wvalid, s00_axi_wready;
    wire [0:0]  s00_axi_wuser;
    wire [3:0]  s00_axi_bid;
    wire [1:0]  s00_axi_bresp;
    wire [0:0]  s00_axi_buser;
    wire        s00_axi_bvalid, s00_axi_bready;
    wire        s00_axi_arvalid, s00_axi_arready;
    wire [MEM_DATA_BITS-1:0] s00_axi_rdata;
    wire [1:0]  s00_axi_rresp;
    wire        s00_axi_rlast, s00_axi_rvalid, s00_axi_rready;
    wire [3:0]  s00_axi_rid_w;
    wire [0:0]  s00_axi_ruser;

    reg  [26:0] cnt;

    // Capture ready (no AF — camera starts as soon as I2C init done)
    wire cmos1_capture_rdy = cmos_init_done[0];
    wire cmos2_capture_rdy = cmos_init_done[1];

    // Frame buffer size: 640*360*16bit = 230400 16-bit words
    // = 230400*16/64 = 57600 64-bit words
    // = 230400*16/256 = 14400 256-bit words
    // = 230400*2 = 460800 bytes
    parameter FRAME_LEN_64  = 25'd57600;
    parameter FRAME_BYTES   = 25'd460800;
    // 260915b: 地址端口是 25bit，而 `FB*2`、`BASE+FB*3` 这类表达式会被撑宽，
    // 会报 32 条 "Width mismatch"。这里用 25bit 常量做中转（纯改名，取值不变）；
    // 写法与 single_video_select.v 里已有的 localparam [24:0] CH4_ADDR2 一致。
    localparam [24:0] FB1 = FRAME_BYTES;            // = 460800
    localparam [24:0] FB2 = FRAME_BYTES * 25'd2;    // = 921600
    localparam [24:0] FB3 = FRAME_BYTES * 25'd3;    // = 1382400

/////////////////////////////////////////////////////////////////////////////////////
// PLL
/////////////////////////////////////////////////////////////////////////////////////
    pll u_pll (
        .clkin1   (sys_clk),     // 50MHz
        .clkout0  (pix_clk_w),   // 74.25MHz for 720p60
        .clkout1  (cfg_clk),     // ~10MHz I2C
        .clkout2  (clk_25M),     // 25MHz camera I2C
        .pll_lock (locked)
    );
    assign pix_clk = pix_clk_w;

/////////////////////////////////////////////////////////////////////////////////////
// Reset & HDMI I2C
/////////////////////////////////////////////////////////////////////////////////////
    wire init_over_tx, init_over_rx;
    wire iic_scl_w, iic_sda_w;
    assign iic_hdmi_scl = iic_scl_w;
    assign iic_hdmi_sda = iic_sda_w;

    ms72xx_ctl ms72xx_ctl(
        .clk          (cfg_clk),
        .rst_n        (rstn_out),
        .init_over_tx (init_over_tx),
        .init_over_rx (init_over_rx),
        .iic_tx_scl   (iic_tx_scl),
        .iic_tx_sda   (iic_tx_sda),
        .iic_scl      (iic_scl_w),
        .iic_sda      (iic_sda_w)
    );
    assign hdmi_int_led = init_over_tx;

    always @(posedge cfg_clk) begin
        if(!locked)
            rstn_1ms <= 16'd0;
        else begin
            if(rstn_1ms == 16'h2710)
                rstn_1ms <= rstn_1ms;
            else
                rstn_1ms <= rstn_1ms + 1'b1;
        end
    end
    assign rstn_out = (rstn_1ms == 16'h2710);

/////////////////////////////////////////////////////////////////////////////////////
// OV5640 Camera Init
/////////////////////////////////////////////////////////////////////////////////////
    power_on_delay power_on_delay_inst(
        .clk_50M      (sys_clk),
        .reset_n      (1'b1),              // hardwired (matching working reference)
        .camera1_rstn (cmos1_reset),
        .camera2_rstn (cmos2_reset),
        .camera_pwnd  (),
        .initial_en   (initial_en)
    );

    reg_config coms1_reg_config(
        .clk_25M       (clk_25M),
        .camera_rstn   (cmos1_reset),
        .initial_en    (initial_en),
        .i2c_sclk      (cmos1_scl),
        .i2c_sdat      (cmos1_sda),
        .reg_conf_done (cmos_init_done[0]),
        .reg_index     (),
        .clock_20k     ()
    );

    reg_config coms2_reg_config(
        .clk_25M       (clk_25M),
        .camera_rstn   (cmos2_reset),
        .initial_en    (initial_en),
        .i2c_sclk      (cmos2_scl),
        .i2c_sdat      (cmos2_sda),
        .reg_conf_done (cmos_init_done[1]),
        .reg_index     (),
        .clock_20k     ()
    );

/////////////////////////////////////////////////////////////////////////////////////
// CAM1: 8-bit DVP -> 16-bit RGB565
/////////////////////////////////////////////////////////////////////////////////////
    always@(posedge cmos1_pclk) begin
        cmos1_d_d0     <= cmos1_data;
        cmos1_href_d0  <= cmos1_href;
        cmos1_vsync_d0 <= cmos1_vsync;
    end

    cmos_8_16bit cmos1_8_16bit(
        .pclk      (cmos1_pclk),
        .rst_n     (cmos1_capture_rdy),     // held until I2C init done
        .pdata_i   (cmos1_d_d0),
        .de_i      (cmos1_href_d0),
        .vs_i      (cmos1_vsync_d0),
        .pixel_clk (cmos1_pclk_16bit),
        .pdata_o   (cmos1_d_16bit),
        .de_o      (cmos1_href_16bit)
    );

/////////////////////////////////////////////////////////////////////////////////////
// CAM2: 8-bit DVP -> 16-bit RGB565  (with IO-delay to stabilize sampling)
/////////////////////////////////////////////////////////////////////////////////////
    // IO-delay on each CAM2 data line (GTP_IODELAY for Logos: ~25ps/step, 0~127)
    genvar ci;
    generate
        for (ci = 0; ci < 8; ci = ci + 1) begin : gen_cam2_data_dly
            GTP_IODELAY #(
                .DELAY_STEP  ( CAM2_IO_DELAY     ),
                .DELAY_DEPTH ( 7                  )
            ) u_cam2_data_dly (
                .DO           ( cmos2_data_dly[ci] ),
                .DELAY_OB     (                    ),
                .DI           ( cmos2_data[ci]     ),
                .LOAD_N       ( 1'b0               ),
                .MOVE         ( 1'b1               ),
                .DIRECTION    ( 1'b0               )
            );
        end
    endgenerate

    // IO-delay on CAM2 HREF (must stay aligned with data)
    GTP_IODELAY #(
        .DELAY_STEP  ( CAM2_IO_DELAY     ),
        .DELAY_DEPTH ( 7                  )
    ) u_cam2_href_dly (
        .DO           ( cmos2_href_dly   ),
        .DELAY_OB     (                  ),
        .DI           ( cmos2_href       ),
        .LOAD_N       ( 1'b0             ),
        .MOVE         ( 1'b1             ),
        .DIRECTION    ( 1'b0             )
    );

    // Register delayed CAM2 signals on posedge PCLK
    always@(posedge cmos2_pclk) begin
        cmos2_d_d0     <= cmos2_data_dly;
        cmos2_href_d0  <= cmos2_href_dly;
        cmos2_vsync_d0 <= cmos2_vsync;
    end

    cmos_8_16bit_neg cmos2_8_16bit(
        .pclk      (cmos2_pclk),
        .rst_n     (cmos2_capture_rdy),     // held until I2C + AF firmware done
        .pdata_i   (cmos2_d_d0),
        .de_i      (cmos2_href_d0),
        .vs_i      (cmos2_vsync_d0),
        .pixel_clk (cmos2_pclk_16bit),
        .pdata_o   (cmos2_d_16bit),
        .de_o      (cmos2_href_16bit)
    );

/////////////////////////////////////////////////////////////////////////////////////
// Video Scaling: 1280x720 -> 640x360
/////////////////////////////////////////////////////////////////////////////////////
    wire        cam1_scale_de, cam2_scale_de;
    wire [15:0] cam1_scale_data, cam2_scale_data;

    // CAM1 scale: 1280x720 -> 640x360
    video_scale_process #(.PIX_DATA_WIDTH(16)) u_scale_cam1 (
        .video_clk            (cmos1_pclk_16bit),
        .rst_n                (rstn_out),
        .frame_sync_n         (~cmos1_vsync_d0),
        .video_data_in        ({cmos1_d_16bit[4:0], cmos1_d_16bit[10:5], cmos1_d_16bit[15:11]}),
        .video_data_valid     (cmos1_href_16bit),
        .video_data_out       (cam1_scale_data),
        .video_data_out_valid (cam1_scale_de),
        .video_ready          (1'b1),
        // 260915b: 显式 16bit 定宽（原写法是不带位宽的十进制字面量 = 32bit，
        // 工具会报 "Port expects 16 bits, got 32"；数值本身都在 16 位内）
        .video_width_in       (16'd1280),
        .video_height_in      (16'd720),
        .video_width_out      (16'd640),
        .video_height_out     (16'd360)
    );

    // CAM2 scale: 1280x720 -> 640x360
    video_scale_process #(.PIX_DATA_WIDTH(16)) u_scale_cam2 (
        .video_clk            (cmos2_pclk_16bit),
        .rst_n                (rstn_out),
        .frame_sync_n         (~cmos2_vsync_d0),
        .video_data_in        ({cmos2_d_16bit[4:0], cmos2_d_16bit[10:5], cmos2_d_16bit[15:11]}),
        .video_data_valid     (cmos2_href_16bit),
        .video_data_out       (cam2_scale_data),
        .video_data_out_valid (cam2_scale_de),
        .video_ready          (1'b1),
        // 260915b: 显式 16bit 定宽（原写法是不带位宽的十进制字面量 = 32bit，
        // 工具会报 "Port expects 16 bits, got 32"；数值本身都在 16 位内）
        .video_width_in       (16'd1280),
        .video_height_in      (16'd720),
        .video_width_out      (16'd640),
        .video_height_out     (16'd360)
    );

/////////////////////////////////////////////////////////////////////////////////////
// HDMI Input pipeline: register -> RGB565(16bit) -> scale 1280x720 -> 640x360
// Convert 24-bit RGB888 to 16-bit RGB565 BEFORE scaling, matching camera format
// hdmi_data_in = {R[7:0], G[7:0], B[7:0]} (MS7200 24-bit output)
// hdmi_565_in  = {R[4:0], G[5:0], B[4:0]} (standard RGB565, same as camera after swap)
/////////////////////////////////////////////////////////////////////////////////////
    reg [23:0] hdmi_data_d0, hdmi_data_d1;
    reg        hdmi_de_d0, hdmi_de_d1;
    reg        hdmi_vs_d0, hdmi_vs_d1;
    always@(posedge pixclk_hdmi_in) begin
        hdmi_data_d0 <= hdmi_data_in;
        hdmi_data_d1 <= hdmi_data_d0;
        hdmi_de_d0   <= de_hdmi_in;
        hdmi_de_d1   <= hdmi_de_d0;
        hdmi_vs_d0   <= vs_hdmi_in;
        hdmi_vs_d1   <= hdmi_vs_d0;
    end

    // 24-bit RGB888 -> 16-bit {B,G,R} matching camera channel format
    wire [15:0] hdmi_565_in = {hdmi_data_d1[23:19], hdmi_data_d1[15:10], hdmi_data_d1[7:3]};

    wire        hdmi_scale_de;
    wire [15:0] hdmi_scale_data;
    video_scale_process #(.PIX_DATA_WIDTH(16)) u_scale_hdmi (
        .video_clk            (pixclk_hdmi_in),
        .rst_n                (rstn_out),
        .frame_sync_n         (~hdmi_vs_d1),
        .video_data_in        (hdmi_565_in),
        .video_data_valid     (hdmi_de_d1),
        .video_data_out       (hdmi_scale_data),
        .video_data_out_valid (hdmi_scale_de),
        .video_ready          (1'b1),
        .video_width_in       (16'd1280),    // HDMI source set to 720p
        .video_height_in      (16'd720),
        .video_width_out      (16'd640),
        .video_height_out     (16'd360)
    );

/////////////////////////////////////////////////////////////////////////////////////
// VSYNC detection & Write Request
/////////////////////////////////////////////////////////////////////////////////////
    // CAM1 vsync
    reg cmos1_vsync_d1;
    always@(posedge cmos1_pclk_16bit or negedge rstn_out) begin
        if(!rstn_out) cmos1_vsync_d1 <= 1'd0;
        else          cmos1_vsync_d1 <= cmos1_vsync_d0;
    end

    // CDC: sync gamma_bypass into cmos1_pclk_16bit domain
    reg gamma_bypass_c1_s0, gamma_bypass_c1_s1;
    always @(posedge cmos1_pclk_16bit or negedge rstn_out) begin
        if (!rstn_out) begin
            gamma_bypass_c1_s0 <= 1'b1;
            gamma_bypass_c1_s1 <= 1'b1;
        end else begin
            gamma_bypass_c1_s0 <= gamma_bypass;
            gamma_bypass_c1_s1 <= gamma_bypass_c1_s0;
        end
    end

    wire [15:0] cam1_enhanced_data;
    wire        cam1_enhanced_de;

    image_enhance u_enhance_cam1 (
        .clk      (cmos1_pclk_16bit),
        .rst_n    (rstn_out),
        .bypass   (gamma_bypass_c1_s1),
        .de_in    (cam1_scale_de),
        .data_in  (cam1_scale_data),
        .de_out   (cam1_enhanced_de),
        .data_out (cam1_enhanced_data)
    );

    // 261023: CH0 360P 去雾（拼接左上象限）。
    // 261023b: PIX_DLY=2 —— 像素/rcp/de 同步晚两级（输入一级 + 乘法输出一级）。
    // 写侧 frame_fifo_write 按 de 计数写入，收集到的像素序列逐个不变（帧长、
    // 帧相位、四象限对齐都不动），换来像素通路两级寄存器。四路统一开，
    // 保证象限之间不错位。
    wire [15:0] cam1_defog_data;
    wire        cam1_defog_de;

    defog_rgb #(
        .H_ACT  (11'd640),
        .V_ACT  (10'd360),
        .PIX_DLY(2)                       // 写侧：像素/rcp/de 同步晚 2 拍，帧内容不变
    ) u_defog_cam1 (
        .clk      (cmos1_pclk_16bit),
        .rst_n    (rstn_out),
        .bypass   (~d360_c1_f),           // 1 = 直通（去雾关）
        .de_in    (cam1_enhanced_de),
        .data_in  (cam1_enhanced_data),
        .de_out   (cam1_defog_de),
        .data_out (cam1_defog_data)
    );

    assign ch0_write_data = cam1_defog_data;
`ifdef DISABLE_CAM1
    assign ch0_write_en   = 1'b0;
`else
    assign ch0_write_en   = cam1_defog_de;
`endif

    cmos_write_req_gen #(.DROP_FRAMES(10)) cmos_wr_gen_cam1(
        .rst              (~cmos1_capture_rdy),
        .pclk             (cmos1_pclk_16bit),
        .cmos_vsync       (cmos1_vsync_d1),
        .write_req        (ch0_write_req),
        .write_addr_index (ch0_write_addr_index),
        .read_addr_index  (ch0_read_addr_index),
        .write_req_ack    (ch0_write_req_ack)
    );

    // CAM2 vsync
    reg cmos2_vsync_d1;
    always@(posedge cmos2_pclk_16bit or negedge rstn_out) begin
        if(!rstn_out) cmos2_vsync_d1 <= 1'd0;
        else          cmos2_vsync_d1 <= cmos2_vsync_d0;
    end

    // CDC: sync gamma_bypass into cmos2_pclk_16bit domain
    reg gamma_bypass_c2_s0, gamma_bypass_c2_s1;
    always @(posedge cmos2_pclk_16bit or negedge rstn_out) begin
        if (!rstn_out) begin
            gamma_bypass_c2_s0 <= 1'b1;
            gamma_bypass_c2_s1 <= 1'b1;
        end else begin
            gamma_bypass_c2_s0 <= gamma_bypass;
            gamma_bypass_c2_s1 <= gamma_bypass_c2_s0;
        end
    end

    wire [15:0] cam2_enhanced_data;
    wire        cam2_enhanced_de;

    image_enhance u_enhance_cam2 (
        .clk      (cmos2_pclk_16bit),
        .rst_n    (rstn_out),
        .bypass   (gamma_bypass_c2_s1),
        .de_in    (cam2_scale_de),
        .data_in  (cam2_scale_data),
        .de_out   (cam2_enhanced_de),
        .data_out (cam2_enhanced_data)
    );

    // 261023: CH1 360P 去雾（拼接右上象限）
    wire [15:0] cam2_defog_data;
    wire        cam2_defog_de;

    defog_rgb #(
        .H_ACT  (11'd640),
        .V_ACT  (10'd360),
        .PIX_DLY(2)                       // 写侧：像素/rcp/de 同步晚 2 拍，帧内容不变
    ) u_defog_cam2 (
        .clk      (cmos2_pclk_16bit),
        .rst_n    (rstn_out),
        .bypass   (~d360_c2_f),
        .de_in    (cam2_enhanced_de),
        .data_in  (cam2_enhanced_data),
        .de_out   (cam2_defog_de),
        .data_out (cam2_defog_data)
    );

    assign ch1_write_data = cam2_defog_data;
`ifdef DISABLE_CAM2
    assign ch1_write_en   = 1'b0;
`else
    assign ch1_write_en   = cam2_defog_de;
`endif

    cmos_write_req_gen #(.DROP_FRAMES(16)) cmos_wr_gen_cam2(
        .rst              (~cmos2_capture_rdy),
        .pclk             (cmos2_pclk_16bit),
        .cmos_vsync       (cmos2_vsync_d1),
        .write_req        (ch1_write_req),
        .write_addr_index (ch1_write_addr_index),
        .read_addr_index  (ch1_read_addr_index),
        .write_req_ack    (ch1_write_req_ack)
    );

    // CH2 (HDMI) vsync register
    reg hdmi_vs_d2;
    always@(posedge pixclk_hdmi_in or negedge rstn_out) begin
        if(!rstn_out) hdmi_vs_d2 <= 1'd0;
        else          hdmi_vs_d2 <= hdmi_vs_d1;
    end

    wire        ch2_write_req, ch2_write_req_ack;
    wire [1:0]  ch2_write_addr_index, ch2_read_addr_index;
    wire ch2_rd_burst_req, ch2_wr_burst_req, ch2_rd_burst_finish, ch2_wr_burst_finish;
    wire ch2_rd_burst_data_valid, ch2_wr_burst_data_req;
    wire [BUSRT_BITS-1:0] ch2_rd_burst_len, ch2_wr_burst_len;
    wire [ADDR_BITS-1:0]  ch2_rd_burst_addr, ch2_wr_burst_addr;
    wire [MEM_DATA_BITS-1:0] ch2_rd_burst_data, ch2_wr_burst_data;
    wire ch2_read_req, ch2_read_req_ack, ch2_read_en;
    wire [15:0] ch2_read_data, ch2_write_data;
    wire        ch2_write_en;

    // CDC: sync gamma_bypass into pixclk_hdmi_in domain
    reg gamma_bypass_hd_s0, gamma_bypass_hd_s1;
    always @(posedge pixclk_hdmi_in or negedge rstn_out) begin
        if (!rstn_out) begin
            gamma_bypass_hd_s0 <= 1'b1;
            gamma_bypass_hd_s1 <= 1'b1;
        end else begin
            gamma_bypass_hd_s0 <= gamma_bypass;
            gamma_bypass_hd_s1 <= gamma_bypass_hd_s0;
        end
    end

    wire [15:0] hdmi_enhanced_data;
    wire        hdmi_enhanced_de;

    image_enhance u_enhance_hdmi (
        .clk      (pixclk_hdmi_in),
        .rst_n    (rstn_out),
        .bypass   (gamma_bypass_hd_s1),
        .de_in    (hdmi_scale_de),
        .data_in  (hdmi_scale_data),
        .de_out   (hdmi_enhanced_de),
        .data_out (hdmi_enhanced_data)
    );

    // 261023: CH2 360P 去雾（拼接左下象限）—— 260906 删掉的那一级按当前
    // defog_rgb（零延迟、无 Xilinx IP）原样加回。
    // 各写通道仍由自己的 vsync 独立调度进各自 DDR 区域，DDR 帧缓存
    // （异步 FIFO + 4 槽乒乓）继续吸收像素相位差，无需跨通道补偿。
    wire [15:0] hdmi_defog_data;
    wire        hdmi_defog_de;

    defog_rgb #(
        .H_ACT  (11'd640),
        .V_ACT  (10'd360),
        .PIX_DLY(2)                       // 写侧：像素/rcp/de 同步晚 2 拍，帧内容不变
    ) u_defog_hdmi (
        .clk      (pixclk_hdmi_in),
        .rst_n    (rstn_out),
        .bypass   (~d360_hd_f),
        .de_in    (hdmi_enhanced_de),
        .data_in  (hdmi_enhanced_data),
        .de_out   (hdmi_defog_de),
        .data_out (hdmi_defog_data)
    );

    assign ch2_write_data = hdmi_defog_data;
    assign ch2_write_en   = hdmi_defog_de;

    cmos_write_req_gen #(.DROP_FRAMES(5)) cmos_wr_gen_hdmi(
        .rst              (~rstn_out),
        .pclk             (pixclk_hdmi_in),
        .cmos_vsync       (hdmi_vs_d2),
        .write_req        (ch2_write_req),
        .write_addr_index (ch2_write_addr_index),
        .read_addr_index  (ch2_read_addr_index),
        .write_req_ack    (ch2_write_req_ack)
    );

/////////////////////////////////////////////////////////////////////////////////////
// CH3: Ethernet video (260909) — eth_video_rx(RGMII+UDP stack, 55AA parse)
//      -> video_scale_process (1280x720 -> 640x360) -> image_enhance -> write req
/////////////////////////////////////////////////////////////////////////////////////
    wire        eth_write_clk;                       // rgmii_clk 125M
    wire        eth_video_vs, eth_video_de;
    wire [15:0] eth_video_data;

    wire        ch3_write_req, ch3_write_req_ack;
    wire [1:0]  ch3_write_addr_index, ch3_read_addr_index;
    wire ch3_rd_burst_req, ch3_wr_burst_req, ch3_rd_burst_finish, ch3_wr_burst_finish;
    wire ch3_rd_burst_data_valid, ch3_wr_burst_data_req;
    wire [BUSRT_BITS-1:0] ch3_rd_burst_len, ch3_wr_burst_len;
    wire [ADDR_BITS-1:0]  ch3_rd_burst_addr, ch3_wr_burst_addr;
    wire [MEM_DATA_BITS-1:0] ch3_rd_burst_data, ch3_wr_burst_data;
    wire ch3_read_req, ch3_read_req_ack, ch3_read_en;
    wire [15:0] ch3_read_data, ch3_write_data;
    wire        ch3_write_en;

    // 260911: CH4 single-view frame buffer (mem-side burst, core_clk domain)
    wire ch4_rd_burst_req, ch4_wr_burst_req, ch4_rd_burst_finish, ch4_wr_burst_finish;
    wire ch4_rd_burst_data_valid, ch4_wr_burst_data_req;
    wire [BUSRT_BITS-1:0] ch4_rd_burst_len, ch4_wr_burst_len;
    wire [ADDR_BITS-1:0]  ch4_rd_burst_addr, ch4_wr_burst_addr;
    wire [MEM_DATA_BITS-1:0] ch4_rd_burst_data, ch4_wr_burst_data;

    eth_video_rx #(
        .LOCAL_MAC(48'ha0_b1_c2_d3_e1_e1), .LOCAL_IP(32'hC0_A8_01_0B),
        .LOCL_PORT(16'h1F40), .DEST_IP(32'hC0_A8_01_69), .DEST_PORT(16'h1F40)
    ) eth_video_rx_u0 (
        .sys_clk          (sys_clk),
        .rgmii_rxc        (rgmii_rxc),
        .rgmii_rx_ctl     (rgmii_rx_ctl),
        .rgmii_rxd        (rgmii_rxd),
        .rgmii_txc        (rgmii_txc),
        .rgmii_tx_ctl     (rgmii_tx_ctl),
        .rgmii_txd        (rgmii_txd),
        .phy_rstn         (phy_rstn),
        .eth_clk          (eth_write_clk),
        .eth_video_vs     (eth_video_vs),
        .eth_video_de     (eth_video_de),
        .eth_video_data   (eth_video_data),
        .eth_udp_led      (eth_udp_led),
        .eth_hdr55_led    (eth_hdr55_led),
        .eth_extract_led  (eth_extract_led)
    );

    // CDC: sync gamma_bypass into rgmii_clk domain
    reg gamma_bypass_eth_s0, gamma_bypass_eth_s1;
    always @(posedge eth_write_clk or negedge rstn_out) begin
        if (!rstn_out) begin
            gamma_bypass_eth_s0 <= 1'b1;
            gamma_bypass_eth_s1 <= 1'b1;
        end else begin
            gamma_bypass_eth_s0 <= gamma_bypass;
            gamma_bypass_eth_s1 <= gamma_bypass_eth_s0;
        end
    end

    // vs pipeline (same pattern as CH2 hdmi_vs_d0/d1/d2)
    reg eth_vs_d0, eth_vs_d1, eth_vs_d2;
    always @(posedge eth_write_clk) begin
        eth_vs_d0 <= eth_video_vs;
        eth_vs_d1 <= eth_vs_d0;
    end
    always @(posedge eth_write_clk or negedge rstn_out) begin
        if(!rstn_out) eth_vs_d2 <= 1'b0;
        else          eth_vs_d2 <= eth_vs_d1;
    end

    // 260910b: CH3 像素流延迟 16 拍 —— 帧头保护。
    // 问题: 网口流 vsync 后当拍就有像素(相机通道 vsync 后有几百拍消隐, 无此问题),
    //       而写侧 frame_fifo_write 的 S_ACK 握手跨 rgmii/mem 两域需约 10+ 拍,
    //       期间 fifo_aclr 清一次内部 FIFO —— 帧头像素恰好撞进清零窗口被吃掉。
    //       每帧固定丢开头若干像素 → 显示内容整体右移 → 最右端轮转出帧头内容。
    // 方案: 像素流整体延迟 16 拍, 让帧头在清零窗口之后才进入写 FIFO。
    reg  [15:0] ch3_de_sr;
    reg [255:0] ch3_dat_sr;
    always @(posedge eth_write_clk) begin
        ch3_de_sr  <= {ch3_de_sr[14:0],  eth_video_de};
        ch3_dat_sr <= {ch3_dat_sr[239:0], eth_video_data};
    end
    wire        ch3_de_dly  = ch3_de_sr[15];
    wire [15:0] ch3_dat_dly = ch3_dat_sr[255:240];

    // scale 1280x720 -> 640x360 (nearest, same as other channels)
    // 260910b: frame_sync_n 改用 vsync 原始脉冲(同域 0 延迟) —— 原先延迟 2 拍的
    // eth_vs_d1 让帧头前 2 个像素落在计数器复位之前被吞掉, 每帧再偏 2 像素。
    wire        eth_scale_de;
    wire [15:0] eth_scale_data;
    video_scale_process #(.PIX_DATA_WIDTH(16)) u_scale_eth (
        .video_clk            (eth_write_clk),
        .rst_n                (rstn_out),
        .frame_sync_n         (~eth_video_vs),
        .video_data_in        (ch3_dat_dly),
        .video_data_valid     (ch3_de_dly),
        .video_data_out       (eth_scale_data),
        .video_data_out_valid (eth_scale_de),
        .video_ready          (1'b1),
        // 260915b: 显式 16bit 定宽（原写法是不带位宽的十进制字面量 = 32bit，
        // 工具会报 "Port expects 16 bits, got 32"；数值本身都在 16 位内）
        .video_width_in       (16'd1280),
        .video_height_in      (16'd720),
        .video_width_out      (16'd640),
        .video_height_out     (16'd360)
    );

    wire [15:0] eth_enhanced_data;
    wire        eth_enhanced_de;
    image_enhance u_enhance_eth (
        .clk      (eth_write_clk),
        .rst_n    (rstn_out),
        .bypass   (gamma_bypass_eth_s1),
        .de_in    (eth_scale_de),
        .data_in  (eth_scale_data),
        .de_out   (eth_enhanced_de),
        .data_out (eth_enhanced_data)
    );

    // 261023: CH3 360P 去雾（拼接右下象限）
    // 261023b: 本路 eth_write_clk = rgmii_clk = 125MHz（8ns），是四路里唯一
    // 明显快于 74.25MHz 的。两轮 STA 实测：
    //   零延迟    px_cnt -> MUX -> APM -> 40bit 进位 -> clamp = 9.57ns，WNS -1.996
    //   一级流水  i_b_q  -> 减 -> APM -> 24bit 进位 -> clamp = 8.70ns，WNS -1.169
    // 现在 PIX_DLY=2 在乘法输出再打一级：级A≈4.7ns / 级B≈3.5ns。
    // de 与 data 同步晚 2 拍，frame_fifo_write 按 de 计数，帧内容不变。
    wire [15:0] eth_defog_data;
    wire        eth_defog_de;

    defog_rgb #(
        .H_ACT  (11'd640),
        .V_ACT  (10'd360),
        .PIX_DLY(2)                       // 写侧：像素/rcp/de 同步晚 2 拍，帧内容不变
    ) u_defog_eth (
        .clk      (eth_write_clk),
        .rst_n    (rstn_out),
        .bypass   (~d360_et_f),
        .de_in    (eth_enhanced_de),
        .data_in  (eth_enhanced_data),
        .de_out   (eth_defog_de),
        .data_out (eth_defog_data)
    );

    assign ch3_write_data = eth_defog_data;
    assign ch3_write_en   = eth_defog_de;

    cmos_write_req_gen #(.DROP_FRAMES(5)) cmos_wr_gen_eth(
        .rst              (~rstn_out),
        .pclk             (eth_write_clk),
        .cmos_vsync       (eth_vs_d2),
        .write_req        (ch3_write_req),
        .write_addr_index (ch3_write_addr_index),
        .read_addr_index  (ch3_read_addr_index),
        .write_req_ack    (ch3_write_req_ack)
    );

/////////////////////////////////////////////////////////////////////////////////////
// Frame Buffer Controllers
/////////////////////////////////////////////////////////////////////////////////////
    // CH0 (CAM1)
    frame_read_write #(
        .MEM_DATA_BITS(256), .READ_DATA_BITS(16), .WRITE_DATA_BITS(16),
        .ADDR_BITS(25), .BUSRT_BITS(10), .BURST_SIZE(16)
    ) frw_cam1 (
        .rst             (~ddr_init_done),
        .mem_clk         (core_clk),
        .rd_burst_req    (ch0_rd_burst_req), .rd_burst_len(ch0_rd_burst_len),
        .rd_burst_addr   (ch0_rd_burst_addr), .rd_burst_data_valid(ch0_rd_burst_data_valid),
        .rd_burst_data   (ch0_rd_burst_data), .rd_burst_finish(ch0_rd_burst_finish),
        .read_clk        (pix_clk_w), .read_req(ch0_read_req), .read_req_ack(ch0_read_req_ack),
        .read_finish     (), .read_en(ch0_read_en), .read_data(ch0_read_data),
        .read_addr_0     (25'd0), .read_addr_1(FB1),
        .read_addr_2     (FB2), .read_addr_3(FB3),
        .read_addr_index (ch0_read_addr_index), .read_len(FRAME_LEN_64),
        .wr_burst_req    (ch0_wr_burst_req), .wr_burst_len(ch0_wr_burst_len),
        .wr_burst_addr   (ch0_wr_burst_addr), .wr_burst_data_req(ch0_wr_burst_data_req),
        .wr_burst_data   (ch0_wr_burst_data), .wr_burst_finish(ch0_wr_burst_finish),
        .write_clk       (cmos1_pclk_16bit), .write_req(ch0_write_req),
        .write_req_ack   (ch0_write_req_ack), .write_finish(),
        .write_addr_0    (25'd0), .write_addr_1(FB1),
        .write_addr_2    (FB2), .write_addr_3(FB3),
        .write_addr_index(ch0_write_addr_index), .write_len(FRAME_LEN_64),
        .write_en        (ch0_write_en), .write_data(ch0_write_data)
    );

    // CH1 (CAM2) — offset by 4 frames
    localparam [24:0] CH1_BASE = FB1 * 25'd4;   // 260915b: 显式 25bit，= 1843200
    frame_read_write #(
        .MEM_DATA_BITS(256), .READ_DATA_BITS(16), .WRITE_DATA_BITS(16),
        .ADDR_BITS(25), .BUSRT_BITS(10), .BURST_SIZE(16)
    ) frw_cam2 (
        .rst             (~ddr_init_done),
        .mem_clk         (core_clk),
        .rd_burst_req    (ch1_rd_burst_req), .rd_burst_len(ch1_rd_burst_len),
        .rd_burst_addr   (ch1_rd_burst_addr), .rd_burst_data_valid(ch1_rd_burst_data_valid),
        .rd_burst_data   (ch1_rd_burst_data), .rd_burst_finish(ch1_rd_burst_finish),
        .read_clk        (pix_clk_w), .read_req(ch1_read_req), .read_req_ack(ch1_read_req_ack),
        .read_finish     (), .read_en(ch1_read_en), .read_data(ch1_read_data),
        .read_addr_0     (CH1_BASE), .read_addr_1(CH1_BASE+FB1),
        .read_addr_2     (CH1_BASE+FB2), .read_addr_3(CH1_BASE+FB3),
        .read_addr_index (ch1_read_addr_index), .read_len(FRAME_LEN_64),
        .wr_burst_req    (ch1_wr_burst_req), .wr_burst_len(ch1_wr_burst_len),
        .wr_burst_addr   (ch1_wr_burst_addr), .wr_burst_data_req(ch1_wr_burst_data_req),
        .wr_burst_data   (ch1_wr_burst_data), .wr_burst_finish(ch1_wr_burst_finish),
        .write_clk       (cmos2_pclk_16bit), .write_req(ch1_write_req),
        .write_req_ack   (ch1_write_req_ack), .write_finish(),
        .write_addr_0    (CH1_BASE), .write_addr_1(CH1_BASE+FB1),
        .write_addr_2    (CH1_BASE+FB2), .write_addr_3(CH1_BASE+FB3),
        .write_addr_index(ch1_write_addr_index), .write_len(FRAME_LEN_64),
        .write_en        (ch1_write_en), .write_data(ch1_write_data)
    );

    // CH2 (HDMI) — offset by 8 frames
    localparam [24:0] CH2_BASE = FB1 * 25'd8;   // 260915b: 显式 25bit，= 3686400
    frame_read_write #(
        .MEM_DATA_BITS(256), .READ_DATA_BITS(16), .WRITE_DATA_BITS(16),
        .ADDR_BITS(25), .BUSRT_BITS(10), .BURST_SIZE(16)
    ) frw_hdmi (
        .rst             (~ddr_init_done),
        .mem_clk         (core_clk),
        .rd_burst_req    (ch2_rd_burst_req), .rd_burst_len(ch2_rd_burst_len),
        .rd_burst_addr   (ch2_rd_burst_addr), .rd_burst_data_valid(ch2_rd_burst_data_valid),
        .rd_burst_data   (ch2_rd_burst_data), .rd_burst_finish(ch2_rd_burst_finish),
        .read_clk        (pix_clk_w), .read_req(ch2_read_req), .read_req_ack(ch2_read_req_ack),
        .read_finish     (), .read_en(ch2_read_en), .read_data(ch2_read_data),
        .read_addr_0     (CH2_BASE), .read_addr_1(CH2_BASE+FB1),
        .read_addr_2     (CH2_BASE+FB2), .read_addr_3(CH2_BASE+FB3),
        .read_addr_index (ch2_read_addr_index), .read_len(FRAME_LEN_64),
        .wr_burst_req    (ch2_wr_burst_req), .wr_burst_len(ch2_wr_burst_len),
        .wr_burst_addr   (ch2_wr_burst_addr), .wr_burst_data_req(ch2_wr_burst_data_req),
        .wr_burst_data   (ch2_wr_burst_data), .wr_burst_finish(ch2_wr_burst_finish),
        .write_clk       (pixclk_hdmi_in), .write_req(ch2_write_req),
        .write_req_ack   (ch2_write_req_ack), .write_finish(),
        .write_addr_0    (CH2_BASE), .write_addr_1(CH2_BASE+FB1),
        .write_addr_2    (CH2_BASE+FB2), .write_addr_3(CH2_BASE+FB3),
        .write_addr_index(ch2_write_addr_index), .write_len(FRAME_LEN_64),
        .write_en        (ch2_write_en), .write_data(ch2_write_data)
    );

    // CH3 (Ethernet) — offset by 12 frames (260909)
    localparam [24:0] CH3_BASE = FB1 * 25'd12;   // 260915b: 显式 25bit，= 5529600
    frame_read_write #(
        .MEM_DATA_BITS(256), .READ_DATA_BITS(16), .WRITE_DATA_BITS(16),
        .ADDR_BITS(25), .BUSRT_BITS(10), .BURST_SIZE(16)
    ) frw_eth (
        .rst             (~ddr_init_done),
        .mem_clk         (core_clk),
        .rd_burst_req    (ch3_rd_burst_req), .rd_burst_len(ch3_rd_burst_len),
        .rd_burst_addr   (ch3_rd_burst_addr), .rd_burst_data_valid(ch3_rd_burst_data_valid),
        .rd_burst_data   (ch3_rd_burst_data), .rd_burst_finish(ch3_rd_burst_finish),
        .read_clk        (pix_clk_w), .read_req(ch3_read_req), .read_req_ack(ch3_read_req_ack),
        .read_finish     (), .read_en(ch3_read_en), .read_data(ch3_read_data),
        .read_addr_0     (CH3_BASE), .read_addr_1(CH3_BASE+FB1),
        .read_addr_2     (CH3_BASE+FB2), .read_addr_3(CH3_BASE+FB3),
        .read_addr_index (ch3_read_addr_index), .read_len(FRAME_LEN_64),
        .wr_burst_req    (ch3_wr_burst_req), .wr_burst_len(ch3_wr_burst_len),
        .wr_burst_addr   (ch3_wr_burst_addr), .wr_burst_data_req(ch3_wr_burst_data_req),
        .wr_burst_data   (ch3_wr_burst_data), .wr_burst_finish(ch3_wr_burst_finish),
        .write_clk       (eth_write_clk), .write_req(ch3_write_req),
        .write_req_ack   (ch3_write_req_ack), .write_finish(),
        .write_addr_0    (CH3_BASE), .write_addr_1(CH3_BASE+FB1),
        .write_addr_2    (CH3_BASE+FB2), .write_addr_3(CH3_BASE+FB3),
        .write_addr_index(ch3_write_addr_index), .write_len(FRAME_LEN_64),
        .write_en        (ch3_write_en), .write_data(ch3_write_data)
    );

/////////////////////////////////////////////////////////////////////////////////////
// DDR3_50H Controller
/////////////////////////////////////////////////////////////////////////////////////
    DDR3_50H u_DDR3_50H (
         .ref_clk                   (sys_clk),
         .resetn                    (rstn_out),
         .ddr_init_done             (ddr_init_done),
         .ddrphy_clkin              (core_clk),
         .pll_lock                  (pll_lock),
         .axi_awaddr                (s00_axi_awaddr),
         .axi_awuser_ap             (1'b0),
         .axi_awuser_id             (s00_axi_awid),
         .axi_awlen                 (s00_axi_awlen),
         .axi_awready               (s00_axi_awready),
         .axi_awvalid               (s00_axi_awvalid),
         .axi_wdata                 (s00_axi_wdata),
         .axi_wstrb                 (s00_axi_wstrb),
         .axi_wready                (s00_axi_wready),
         .axi_wusero_id             (),
         .axi_wusero_last           (axi_wusero_last),
         .axi_araddr                (s00_axi_araddr),
         .axi_aruser_ap             (1'b0),
         .axi_aruser_id             (s00_axi_arid),
         .axi_arlen                 (s00_axi_arlen),
         .axi_arready               (s00_axi_arready),
         .axi_arvalid               (s00_axi_arvalid),
         .axi_rdata                 (s00_axi_rdata),
         .axi_rid                   (s00_axi_rid_w),
         .axi_rlast                 (s00_axi_rlast),
         .axi_rvalid                (s00_axi_rvalid),
         .apb_clk                   (1'b0),
         .apb_rst_n                 (1'b1),
         .apb_sel                   (1'b0),
         .apb_enable                (1'b0),
         .apb_addr                  (8'b0),
         .apb_write                 (1'b0),
         .apb_ready                 (),
         .apb_wdata                 (16'b0),
         .apb_rdata                 (),
         .apb_int                   (),
         .mem_rst_n                 (mem_rst_n),
         .mem_ck                    (mem_ck),
         .mem_ck_n                  (mem_ck_n),
         .mem_cke                   (mem_cke),
         .mem_cs_n                  (mem_cs_n),
         .mem_ras_n                 (mem_ras_n),
         .mem_cas_n                 (mem_cas_n),
         .mem_we_n                  (mem_we_n),
         .mem_odt                   (mem_odt),
         .mem_a                     (mem_a),
         .mem_ba                    (mem_ba),
         .mem_dqs                   (mem_dqs),
         .mem_dqs_n                 (mem_dqs_n),
         .mem_dq                    (mem_dq),
         .mem_dm                    (mem_dm),
         .debug_data                (),
         .debug_slice_state         (),
         .debug_calib_ctrl          (),
         .ck_dly_set_bin            (),
         .force_ck_dly_en           (1'b0),
         .force_ck_dly_set_bin      (8'h05),
         .dll_step                  (),
         .dll_lock                  (),
         .init_read_clk_ctrl        (2'b0),
         .init_slip_step            (4'b0),
         .force_read_clk_ctrl       (1'b0),
         .ddrphy_gate_update_en     (1'b0),
         .update_com_val_err_flag   (),
         .rd_fake_stop              (1'b0)
    );

/////////////////////////////////////////////////////////////////////////////////////
// AXI Master
/////////////////////////////////////////////////////////////////////////////////////
    assign s00_axi_bvalid = 1'b1;

    aq_axi_master_256 u_aq_axi_master (
        .ARESETN      (ddr_init_done),
        .ACLK         (core_clk),
        .M_AXI_AWID   (s00_axi_awid),   .M_AXI_AWADDR(s00_axi_awaddr),
        .M_AXI_AWLEN  (s00_axi_awlen),  .M_AXI_AWSIZE(s00_axi_awsize),
        .M_AXI_AWBURST(s00_axi_awburst),.M_AXI_AWLOCK(s00_axi_awlock),
        .M_AXI_AWCACHE(s00_axi_awcache),.M_AXI_AWPROT(s00_axi_awprot),
        .M_AXI_AWQOS  (s00_axi_awqos),  .M_AXI_AWUSER(s00_axi_awuser),
        .M_AXI_AWVALID(s00_axi_awvalid),.M_AXI_AWREADY(s00_axi_awready),
        .M_AXI_WDATA  (s00_axi_wdata),  .M_AXI_WSTRB(s00_axi_wstrb),
        .M_AXI_WLAST  (s00_axi_wlast),  .M_AXI_WUSER(s00_axi_wuser),
        .M_AXI_WVALID (s00_axi_wvalid), .M_AXI_WREADY(s00_axi_wready),
        .M_AXI_BID    (s00_axi_bid),    .M_AXI_BRESP(s00_axi_bresp),
        .M_AXI_BUSER  (s00_axi_buser),  .M_AXI_BVALID(s00_axi_bvalid),
        .M_AXI_BREADY (s00_axi_bready),
        .M_AXI_ARID   (s00_axi_arid),   .M_AXI_ARADDR(s00_axi_araddr),
        .M_AXI_ARLEN  (s00_axi_arlen),  .M_AXI_ARSIZE(s00_axi_arsize),
        .M_AXI_ARBURST(s00_axi_arburst),.M_AXI_ARLOCK(s00_axi_arlock),
        .M_AXI_ARCACHE(s00_axi_arcache),.M_AXI_ARPROT(s00_axi_arprot),
        .M_AXI_ARQOS  (s00_axi_arqos),  .M_AXI_ARUSER(s00_axi_aruser),
        .M_AXI_ARVALID(s00_axi_arvalid),.M_AXI_ARREADY(s00_axi_arready),
        .M_AXI_RID    (s00_axi_rid_w),  .M_AXI_RDATA(s00_axi_rdata),
        .M_AXI_RRESP  (s00_axi_rresp),  .M_AXI_RLAST(s00_axi_rlast),
        .M_AXI_RUSER  (s00_axi_ruser),  .M_AXI_RVALID(s00_axi_rvalid),
        .M_AXI_RREADY (s00_axi_rready),
        .MASTER_RST   (1'b0),
        .WR_START     (wr_burst_req),
        .WR_ADRS      ({wr_burst_addr,5'd0}),
        .WR_LEN       ({wr_burst_len,5'd0}),
        .WR_READY     (), .WR_FIFO_RE(wr_burst_data_req),
        .WR_FIFO_EMPTY(1'b0), .WR_FIFO_AEMPTY(1'b0),
        .WR_FIFO_DATA (wr_burst_data), .WR_DONE(wr_burst_finish),
        .RD_START     (rd_burst_req),
        .RD_ADRS      ({rd_burst_addr,5'd0}),
        .RD_LEN       ({rd_burst_len,5'd0}),
        .RD_READY     (), .RD_FIFO_WE(rd_burst_data_valid),
        .RD_FIFO_FULL (1'b0), .RD_FIFO_AFULL(1'b0),
        .RD_FIFO_DATA (rd_burst_data), .RD_DONE(rd_burst_finish),
        .DEBUG        ()
    );

/////////////////////////////////////////////////////////////////////////////////////
// Read Arbiter
/////////////////////////////////////////////////////////////////////////////////////
    mem_read_arbi #(.MEM_DATA_BITS(256), .ADDR_BITS(25), .BUSRT_BITS(10)) u_rd_arbi (
        .rst_n      (ddr_init_done), .mem_clk(core_clk),
        .ch0_rd_burst_req(ch0_rd_burst_req), .ch0_rd_burst_len(ch0_rd_burst_len),
        .ch0_rd_burst_addr(ch0_rd_burst_addr), .ch0_rd_burst_data_valid(ch0_rd_burst_data_valid),
        .ch0_rd_burst_data(ch0_rd_burst_data), .ch0_rd_burst_finish(ch0_rd_burst_finish),
        .ch1_rd_burst_req(ch1_rd_burst_req), .ch1_rd_burst_len(ch1_rd_burst_len),
        .ch1_rd_burst_addr(ch1_rd_burst_addr), .ch1_rd_burst_data_valid(ch1_rd_burst_data_valid),
        .ch1_rd_burst_data(ch1_rd_burst_data), .ch1_rd_burst_finish(ch1_rd_burst_finish),
        .ch2_rd_burst_req(ch2_rd_burst_req), .ch2_rd_burst_len(ch2_rd_burst_len),
        .ch2_rd_burst_addr(ch2_rd_burst_addr), .ch2_rd_burst_data_valid(ch2_rd_burst_data_valid),
        .ch2_rd_burst_data(ch2_rd_burst_data), .ch2_rd_burst_finish(ch2_rd_burst_finish),
        .ch3_rd_burst_req(ch3_rd_burst_req), .ch3_rd_burst_len(ch3_rd_burst_len),
        .ch3_rd_burst_addr(ch3_rd_burst_addr), .ch3_rd_burst_data_valid(ch3_rd_burst_data_valid),
        .ch3_rd_burst_data(ch3_rd_burst_data), .ch3_rd_burst_finish(ch3_rd_burst_finish),
        .ch4_rd_burst_req(ch4_rd_burst_req), .ch4_rd_burst_len(ch4_rd_burst_len),
        .ch4_rd_burst_addr(ch4_rd_burst_addr), .ch4_rd_burst_data_valid(ch4_rd_burst_data_valid),
        .ch4_rd_burst_data(ch4_rd_burst_data), .ch4_rd_burst_finish(ch4_rd_burst_finish),
        .rd_burst_req(rd_burst_req), .rd_burst_len(rd_burst_len),
        .rd_burst_addr(rd_burst_addr), .rd_burst_data_valid(rd_burst_data_valid),
        .rd_burst_data(rd_burst_data), .rd_burst_finish(rd_burst_finish)
    );

/////////////////////////////////////////////////////////////////////////////////////
// Write Arbiter
/////////////////////////////////////////////////////////////////////////////////////
    mem_write_arbi #(.MEM_DATA_BITS(256), .ADDR_BITS(25), .BUSRT_BITS(10)) u_wr_arbi (
        .rst_n      (ddr_init_done), .mem_clk(core_clk),
        .ch0_wr_burst_req(ch0_wr_burst_req), .ch0_wr_burst_len(ch0_wr_burst_len),
        .ch0_wr_burst_addr(ch0_wr_burst_addr), .ch0_wr_burst_data_req(ch0_wr_burst_data_req),
        .ch0_wr_burst_data(ch0_wr_burst_data), .ch0_wr_burst_finish(ch0_wr_burst_finish),
        .ch1_wr_burst_req(ch1_wr_burst_req), .ch1_wr_burst_len(ch1_wr_burst_len),
        .ch1_wr_burst_addr(ch1_wr_burst_addr), .ch1_wr_burst_data_req(ch1_wr_burst_data_req),
        .ch1_wr_burst_data(ch1_wr_burst_data), .ch1_wr_burst_finish(ch1_wr_burst_finish),
        .ch2_wr_burst_req(ch2_wr_burst_req), .ch2_wr_burst_len(ch2_wr_burst_len),
        .ch2_wr_burst_addr(ch2_wr_burst_addr), .ch2_wr_burst_data_req(ch2_wr_burst_data_req),
        .ch2_wr_burst_data(ch2_wr_burst_data), .ch2_wr_burst_finish(ch2_wr_burst_finish),
        .ch3_wr_burst_req(ch3_wr_burst_req), .ch3_wr_burst_len(ch3_wr_burst_len),
        .ch3_wr_burst_addr(ch3_wr_burst_addr), .ch3_wr_burst_data_req(ch3_wr_burst_data_req),
        .ch3_wr_burst_data(ch3_wr_burst_data), .ch3_wr_burst_finish(ch3_wr_burst_finish),
        .ch4_wr_burst_req(ch4_wr_burst_req), .ch4_wr_burst_len(ch4_wr_burst_len),
        .ch4_wr_burst_addr(ch4_wr_burst_addr), .ch4_wr_burst_data_req(ch4_wr_burst_data_req),
        .ch4_wr_burst_data(ch4_wr_burst_data), .ch4_wr_burst_finish(ch4_wr_burst_finish),
        .wr_burst_req(wr_burst_req), .wr_burst_len(wr_burst_len),
        .wr_burst_addr(wr_burst_addr), .wr_burst_data_req(wr_burst_data_req),
        .wr_burst_data(wr_burst_data), .wr_burst_finish(wr_burst_finish)
    );

/////////////////////////////////////////////////////////////////////////////////////
// Output Stitching: color_bar(720p) + video_rect_read ×2
/////////////////////////////////////////////////////////////////////////////////////
    wire color_hs, color_vs, color_de;
    wire [7:0] color_r, color_g, color_b;
    wire [15:0] v0_data, v1_data;
    wire v0_hs, v0_vs, v0_de;
    wire v1_hs, v1_vs, v1_de;

    color_bar color_bar_m0 (
        .clk   (pix_clk_w),
        .rst   (~rstn_out),
        .hs    (color_hs),
        .vs    (color_vs),
        .de    (color_de),
        .rgb_r (color_r),
        .rgb_g (color_g),
        .rgb_b (color_b)
    );

    // Layer 0: CAM1 at top-left (0, 0, 640, 360)
    video_rect_read_data #(.DATA_WIDTH(16)) vrd_cam1 (
        .video_clk        (pix_clk_w),
        .rst              (~rstn_out),
        .video_left_offset(12'd0),
        .video_top_offset (12'd0),
        .video_width      (12'd640),
        .video_height     (12'd360),
        .read_req         (ch0_read_req),
        .read_req_ack     (ch0_read_req_ack),
        .read_en          (ch0_read_en),
        .read_data        (ch0_read_data),
        .timing_hs        (color_hs),
        .timing_vs        (color_vs),
        .timing_de        (color_de),
        .timing_data      (16'd0),            // black background
        .x_cnt            (), .y_cnt(),
        .hs               (v0_hs), .vs(v0_vs), .de(v0_de), .vout_data(v0_data)
    );

    // Layer 1: CAM2 at top-right (640, 0, 640, 360)
    video_rect_read_data #(.DATA_WIDTH(16)) vrd_cam2 (
        .video_clk        (pix_clk_w),
        .rst              (~rstn_out),
        .video_left_offset(12'd640),
        .video_top_offset (12'd0),
        .video_width      (12'd640),
        .video_height     (12'd360),
        .read_req         (ch1_read_req),
        .read_req_ack     (ch1_read_req_ack),
        .read_en          (ch1_read_en),
        .read_data        (ch1_read_data),
        .timing_hs        (v0_hs),
        .timing_vs        (v0_vs),
        .timing_de        (v0_de),
        .timing_data      (v0_data),
        .x_cnt            (), .y_cnt(),
        .hs               (v1_hs), .vs(v1_vs), .de(v1_de), .vout_data(v1_data)
    );

    // Layer 2: HDMI at bottom-left (0, 360, 640, 360)
    wire [15:0] v2_data;
    wire        v2_hs, v2_vs, v2_de;
    video_rect_read_data #(.DATA_WIDTH(16)) vrd_hdmi (
        .video_clk        (pix_clk_w),
        .rst              (~rstn_out),
        .video_left_offset(12'd0),
        .video_top_offset (12'd360),
        .video_width      (12'd640),
        .video_height     (12'd360),
        .read_req         (ch2_read_req),
        .read_req_ack     (ch2_read_req_ack),
        .read_en          (ch2_read_en),
        .read_data        (ch2_read_data),
        .timing_hs        (v1_hs),
        .timing_vs        (v1_vs),
        .timing_de        (v1_de),
        .timing_data      (v1_data),
        .x_cnt            (), .y_cnt(),
        .hs               (v2_hs), .vs(v2_vs), .de(v2_de), .vout_data(v2_data)
    );

    // Layer 3: Ethernet video at bottom-right (640, 360, 640, 360) — 260909 CH3
    wire [15:0] v3_data;
    wire        v3_hs, v3_vs, v3_de;
    video_rect_read_data #(.DATA_WIDTH(16)) vrd_eth (
        .video_clk        (pix_clk_w),
        .rst              (~rstn_out),
        .video_left_offset(12'd640),
        .video_top_offset (12'd360),
        .video_width      (12'd640),
        .video_height     (12'd360),
        .read_req         (ch3_read_req),
        .read_req_ack     (ch3_read_req_ack),
        .read_en          (ch3_read_en),
        .read_data        (ch3_read_data),
        .timing_hs        (v2_hs),
        .timing_vs        (v2_vs),
        .timing_de        (v2_de),
        .timing_data      (v2_data),
        .x_cnt            (), .y_cnt(),
        .hs               (v3_hs), .vs(v3_vs), .de(v3_de), .vout_data(v3_data)
    );

/////////////////////////////////////////////////////////////////////////////////////
// 260911: 5-choose-1 output select
//   branch A (existing): quad stitch via v0..v3 chain above
//   branch B (new)     : single 720P view via single_video_select (CH4)
//   KEY1~KEY5 select, switch applies at color_vs frame boundary (no tearing)
/////////////////////////////////////////////////////////////////////////////////////
    wire [15:0] sng_data;
    wire        sng_hs, sng_vs, sng_de;
    wire [15:0] sel_data;
    wire        sel_hs, sel_vs, sel_de;

    // ---- KEY1~KEY5 debounce (same recipe as key_gamma/KEY8) ----
    // 261011 重排：K18=视图循环 | L15=增强 | J17=去雾 | K16=中值 | J16=Gamma
    // 261031：夜景键（J17 / key_sel[2]）删除。位宽和位号保持原样，只有 [2]
    //         这一位空置 —— 其他键的索引/串口码一个都不动。
    wire [4:0] key_sel_pulse;      // single-cycle pulse per press+release
    btn_deb_fix #(
        .BTN_WIDTH      (5)
    ) u_btn_sel (
        .sys_clk        (sys_clk),
        .rst_n          (rstn_out),
        .btn_in         (key_sel),     // active low, pull-up in .fdc
        .btn_deb_fix    (key_sel_pulse)
    );

    // ---- 260919: UART key control —— 261011 重排 / 261023 扩 '6' ----------
    //   '0'=K18 视图循环(HDMI→CAM1→CAM2→ETH→拼接) | '1'=L15 亮度自适应
    //   '2'=空（261031 夜景修复已删，码位保留） | '3'=K16 单路去雾
    //   '4'=J16 倒车辅助线 | '5'=J19 拼接增亮 | '6'=H20 拼接去雾
    //   其余（'7' 及以上）丢弃。
    // 261024: 原始脉冲（未屏蔽）—— 供下面的 A 帧屏蔽门使用
    wire       raw_key_next_pulse;
    wire       raw_key_enh_pulse;
    wire       raw_key_defog_pulse, raw_key_aux_pulse;
    wire       raw_key_enh360_pulse, raw_key_defog360_pulse;

    wire       uart_key_next_pulse;
    wire       uart_key_enh_pulse;
    wire       uart_key_defog_pulse, uart_key_aux_pulse;
    wire       uart_key_enh360_pulse, uart_key_defog360_pulse;

    wire       traj_busy;               // sys_clk: A 帧解析中
    wire [5:0] traj_angle_sys;          // sys_clk: {dir, mag[4:0]}
    wire       traj_angle_vld;          // sys_clk: 单拍

    uart_key_ctrl #(
        .BPS_NUM            (16'd434)          // 115200 bps @ 50MHz sys_clk
    ) u_uart_key_ctrl (
        .clk                (sys_clk),
        .rst_n              (rstn_out),
        .uart_rx_pin        (uart_rx),         // R8, .fdc 中已加上拉
        .key_next_pulse     (raw_key_next_pulse),
        .key_enh_pulse      (raw_key_enh_pulse),
        .key_defog_pulse    (raw_key_defog_pulse),
        .key_aux_pulse      (raw_key_aux_pulse),
        .key_enh360_pulse   (raw_key_enh360_pulse),
        .key_defog360_pulse (raw_key_defog360_pulse)
    );

    // ---- 261024: 倒车轨迹角度帧（'A' 引导）------------------------------
    // 独立第二接收器 + 独立解析器，uart_key_ctrl.v 源码零改动。
    // A 帧期间把 6 条单字节脉冲屏蔽掉，避免帧内的 '0'/'3'/'1' 被单字节
    // 解码器当成 视图切换 / 辅助线开关 / 增强。物理按键不经过这个门。
    traj_uart_ctrl #(
        .BPS_NUM            (16'd434),
        .TMO_NUM            (20'd250000)       // 5 ms @ 50MHz
    ) u_traj_uart (
        .clk                (sys_clk),
        .rst_n              (rstn_out),
        .uart_rx_pin        (uart_rx),
        .angle              (traj_angle_sys),
        .angle_vld          (traj_angle_vld),
        .busy               (traj_busy)
    );

    wire uart_key_gate = ~traj_busy;
    // 全部 6 条都必须过 ~traj_busy —— 角度帧 A1?611 的个位可以是 '0'~'9'
    assign uart_key_next_pulse     = raw_key_next_pulse     & uart_key_gate;
    assign uart_key_enh_pulse      = raw_key_enh_pulse      & uart_key_gate;
    assign uart_key_defog_pulse    = raw_key_defog_pulse    & uart_key_gate;
    assign uart_key_aux_pulse      = raw_key_aux_pulse      & uart_key_gate;
    assign uart_key_enh360_pulse   = raw_key_enh360_pulse   & uart_key_gate;
    assign uart_key_defog360_pulse = raw_key_defog360_pulse & uart_key_gate;

    // ---- key -> sel[2:0] @sys_clk（261011：循环切换）----
    wire [2:0] sel_sys;
    encoder u_encoder (
        .clk            (sys_clk),
        .rst_n          (rstn_out),
        .key_next_pulse (key_sel_pulse[0] | uart_key_next_pulse), // K18 OR 串口'0'
        .sel            (sel_sys)
    );

    // ---- CDC: sel into pix_clk_w domain (3-bit, 2-FF each bit) ----
    reg [2:0] sel_pix_s0, sel_pix_s1;
    always @(posedge pix_clk_w or negedge rstn_out) begin
        if (!rstn_out) {sel_pix_s1, sel_pix_s0} <= 6'd0;
        else           {sel_pix_s1, sel_pix_s0} <= {sel_pix_s0, sel_sys};
    end

    // ---- single-view branch: 4xFIFO -> 4:1 -> CH4 4-slot ring -> vrd#5 ----
    localparam CH4_BASE    = FRAME_BYTES * 16;   // after CH3 (x12 + 4 slots)
    localparam CH4_STRIDE  = FRAME_BYTES * 4;    // one 720P frame = 4 quadrants
    // 260912 fix: read_len/write_len count MEM-side beats = 256-bit words
    // (frame_fifo_write/read bump their counters by BURST_SIZE=16 per 512B
    // burst). One 720P frame = 1,843,200 B = 57,600 beats, and one write
    // transaction transfers len = 230,400 beats = 4 frames, sweeping the 4
    // ring slots exactly like CH0~CH3 (whose len = 57,600 beats = 4 quadrant
    // frames). The "64-bit words" reading of 260911 was wrong about the unit
    // but numerically landed on the required 4x value.
    localparam CH4_LEN     = FRAME_BYTES * 4 / 8;   // = 25'd230400 = 4 frame beats

    single_video_select #(
        .CH4_BASE   (CH4_BASE),
        .SLOT_STRIDE(CH4_STRIDE),
        .FRAME_LEN  (CH4_LEN)
    ) u_svs (
        .rst_n            (rstn_out),
        .sel              (sel_pix_s1),
        // raw 720P streams (format-converted, BEFORE scale — keep 1:1 pixels)
        .c0_clk           (pixclk_hdmi_in),
        .c0_vs            (hdmi_vs_d1),
        .c0_de            (hdmi_de_d1),
        .c0_data          (hdmi_565_in),
        .c1_clk           (cmos1_pclk_16bit),
        .c1_vs            (cmos1_vsync_d1),
        .c1_de            (cmos1_href_16bit),
        .c1_data          ({cmos1_d_16bit[4:0], cmos1_d_16bit[10:5], cmos1_d_16bit[15:11]}),
        .c2_clk           (cmos2_pclk_16bit),
        .c2_vs            (cmos2_vsync_d1),
        .c2_de            (cmos2_href_16bit),
        .c2_data          ({cmos2_d_16bit[4:0], cmos2_d_16bit[10:5], cmos2_d_16bit[15:11]}),
        .c3_clk           (eth_write_clk),
        .c3_vs            (eth_vs_d2),          // vs as-is (260910b lesson)
        .c3_de            (ch3_de_dly),
        .c3_data          (ch3_dat_dly),
        // CH4 mem side
        .mem_rst          (~ddr_init_done),
        .mem_clk          (core_clk),
        .ch4_rd_burst_req       (ch4_rd_burst_req),
        .ch4_rd_burst_len       (ch4_rd_burst_len),
        .ch4_rd_burst_addr      (ch4_rd_burst_addr),
        .ch4_rd_burst_data_valid(ch4_rd_burst_data_valid),
        .ch4_rd_burst_data      (ch4_rd_burst_data),
        .ch4_rd_burst_finish    (ch4_rd_burst_finish),
        .ch4_wr_burst_req       (ch4_wr_burst_req),
        .ch4_wr_burst_len       (ch4_wr_burst_len),
        .ch4_wr_burst_addr      (ch4_wr_burst_addr),
        .ch4_wr_burst_data_req  (ch4_wr_burst_data_req),
        .ch4_wr_burst_data      (ch4_wr_burst_data),
        .ch4_wr_burst_finish    (ch4_wr_burst_finish),
        // readout timing: v2_* chain so sng_* aligns with v3_* (plan §5.7)
        .pix_clk_w        (pix_clk_w),
        .v2_hs            (v2_hs),
        .v2_vs            (v2_vs),
        .v2_de            (v2_de),
        .v2_data          (v2_data),
        .o_hs             (sng_hs),
        .o_vs             (sng_vs),
        .o_de             (sng_de),
        .o_data           (sng_data)
    );

    // ---- 260916: 60 ms boot mask for the standalone keys (J19/H20/H17) ---
    // btn_deb_fix keeps counting WHILE A KEY IS RELEASED, so ~20 ms after
    // rstn_out releases EVERY unpressed key emits one spurious pulse. KEY1~KEY5
    // are already protected by encoder.v's 60 ms boot mask; the standalone
    // instances (261023 起为 J19 拼接去雾 / H20 单路去雾 / H17 Gamma) have NO
    // mask, so that stray pulse used to toggle them ON right after
    // configuration -- i.e. the chain came up ENABLED even though every reset
    // value is 0 (= bypass = OFF). Masking the pulses here lets the reset value
    // finally survive boot.
    // Same recipe and same 60 ms window as encoder.v. Side effect (identical
    // to encoder.v as well): key presses in the first 60 ms are ignored.
    reg  [21:0] key_boot_cnt;
    wire        key_boot_done = (key_boot_cnt == 22'd3_000_000);   // 60 ms @ 50 MHz
    always @(posedge sys_clk or negedge rstn_out) begin
        if (!rstn_out)           key_boot_cnt <= 22'd0;
        else if (!key_boot_done) key_boot_cnt <= key_boot_cnt + 1'b1;
    end

    // ---- KEY2(L15) / 串口 '1'：模块 A 的"单路增亮开关"触发脉冲 ----------
    // 261028b: 语义已简化为开/关翻转（不再有 AUTO）。这个脉冲只做两件事：
    //   ① 翻转 l_on_sys（见下方模块 A 控制块）
    //   ② 没有别的地方再用它 —— 旧的 sRGB 旁路位已随 image_enhance 实例一起退役。
    // 261023: J19 上原来那个并联的 u_btn_enh 已删除（让给拼接四路去雾）。
    wire key_enh_pulse;       // single-cycle pulse (KEY2 OR uart '1')

    assign key_enh_pulse = uart_key_enh_pulse | key_sel_pulse[1];

    // 261028b: 原来的 enh_on_sys / enh_on_pix / key_enh_f / enc_enh 四段全删 ——
    //   KEY2 现在只驱动"亮度自适应开关" l_on_sys（见下方模块 A 控制块），
    //   旧的 sRGB 旁路位已经没有消费者，留着只会让人误以为有两个增强状态。

    // ---- KEY7 (H20): defog on/off toggle — same recipe as KEY6 ----
    // History: 260914 introduced the 720p defog stage; 260915 defog_rgb.v was
    // restructured to a ZERO-LATENCY in-place stage (de_out = de_in, data_out
    // combinational) because u_vsel used to pass the splice timing straight
    // through and mux only data, so every data delay showed 1:1 as a picture
    // offset (~2 lines / ~5 px at 720p). V4 removes that hazard structurally
    // (timing now travels with data), and the module stays zero-latency:
    // after it no hs/vs realignment is needed (ch2_* == ch1_*).
    // The separate defog_en_eff frame-boundary latch is retired in V4: the
    // joint encoder below latches BOTH key states at the frame boundary, so
    // the bypass can never change mid-frame — same hygiene, one place.
    // ---- KEY6 (J19) / 串口 '5' —— 拼接增亮开关 ------------------------
    // 作用域 = 写侧四路 image_enhance（CH0~CH3，各 640x360）的 bypass 位，
    // 也就是原代码里的 gamma_bypass（历史名字保留：0 = 增亮开，1 = 直通）。
    wire key_enh360_pulse;      // single-cycle pulse (J19 OR uart '5')
    wire key_enh360_btn_pulse;

    btn_deb_fix #(
        .BTN_WIDTH      (1)
    ) u_btn_enh360 (
        .sys_clk        (sys_clk),
        .rst_n          (rstn_out),
        .btn_in         (key_defog360),     // active low, KEY6 (J19), pullup in .fdc
        .btn_deb_fix    (key_enh360_btn_pulse)
    );

    assign key_enh360_pulse = key_enh360_btn_pulse | uart_key_enh360_pulse;

    // ---- KEY7 (H20) / 串口 '6' —— 拼接四路 360P 去雾开关 --------------
    // 作用域仅写侧四个 defog_rgb（CH0~CH3，各 640x360）；单路 720P 的
    // u_defog_sng 由 KEY4(K16) / 串口 '3' 控制，两条链互不影响。
    wire key_defog360_pulse;    // single-cycle pulse (H20 OR uart '6')
    wire key_defog360_btn_pulse;

    btn_deb_fix #(
        .BTN_WIDTH      (1)
    ) u_btn_defog360 (
        .sys_clk        (sys_clk),
        .rst_n          (rstn_out),
        .btn_in         (key_defog),        // active low, KEY7 (H20), pullup in .fdc
        .btn_deb_fix    (key_defog360_btn_pulse)
    );

    assign key_defog360_pulse = key_defog360_btn_pulse | uart_key_defog360_pulse;

    always @(posedge sys_clk or negedge rstn_out) begin
        if (!rstn_out) begin
            defog360_on_sys <= 1'b0;        // POWER-ON = OFF（与增强 / 单路去雾一致）
        end else if (key_defog360_pulse & key_boot_done) begin
            defog360_on_sys <= ~defog360_on_sys;
        end
    end

    // ---- CDC + 帧界锁存：sys_clk -> 四个写时钟域 -------------------------
    // 先 2FF 消亚稳，再在各自 vsync 上升沿锁存一次，保证 bypass 只可能在
    // 消隐期改变，不会切出"上半帧去雾 / 下半帧原图"的画面。
    // 四路的 vs 都是高有效同步脉冲（各自的 frame_sync_n = ~vs），上升沿即帧
    // 同步起点，此时距首行有效像素还有整段垂直消隐，锁存值必然在像素前稳定。
    // CH0 CAM1
    always @(posedge cmos1_pclk_16bit or negedge rstn_out) begin
        if (!rstn_out) begin
            {d360_c1_s1, d360_c1_s0} <= 2'b00;
            {d360_c1_f,  c1_vs_d}    <= 2'b00;
        end else begin
            {d360_c1_s1, d360_c1_s0} <= {d360_c1_s0, defog360_on_sys};
            c1_vs_d <= cmos1_vsync_d1;
            if (cmos1_vsync_d1 & ~c1_vs_d) d360_c1_f <= d360_c1_s1;
        end
    end

    // CH1 CAM2
    always @(posedge cmos2_pclk_16bit or negedge rstn_out) begin
        if (!rstn_out) begin
            {d360_c2_s1, d360_c2_s0} <= 2'b00;
            {d360_c2_f,  c2_vs_d}    <= 2'b00;
        end else begin
            {d360_c2_s1, d360_c2_s0} <= {d360_c2_s0, defog360_on_sys};
            c2_vs_d <= cmos2_vsync_d1;
            if (cmos2_vsync_d1 & ~c2_vs_d) d360_c2_f <= d360_c2_s1;
        end
    end

    // CH2 HDMI
    always @(posedge pixclk_hdmi_in or negedge rstn_out) begin
        if (!rstn_out) begin
            {d360_hd_s1, d360_hd_s0} <= 2'b00;
            {d360_hd_f,  hd_vs_d}    <= 2'b00;
        end else begin
            {d360_hd_s1, d360_hd_s0} <= {d360_hd_s0, defog360_on_sys};
            hd_vs_d <= hdmi_vs_d1;
            if (hdmi_vs_d1 & ~hd_vs_d) d360_hd_f <= d360_hd_s1;
        end
    end

    // CH3 ETH（125MHz 域，同配方）
    always @(posedge eth_write_clk or negedge rstn_out) begin
        if (!rstn_out) begin
            {d360_et_s1, d360_et_s0} <= 2'b00;
            {d360_et_f,  et_vs_d}    <= 2'b00;
        end else begin
            {d360_et_s1, d360_et_s0} <= {d360_et_s0, defog360_on_sys};
            et_vs_d <= eth_vs_d2;
            if (eth_vs_d2 & ~et_vs_d) d360_et_f <= d360_et_s1;
        end
    end

    // 261012c: 中值滤波级（rgb565_median）连同控制通路整体移除 —— 效果不达预期。
    // 261028c: K16（key_sel[3]）给"单路去雾"用。
    // 261031:  J17 上的夜景修复键（key_sel[2]）删除，索引保持不变，
    //          去雾仍在 key_sel[3]、辅助线仍在 key_sel[4]。

    // ---- KEY5 (J16) / 串口 '4' —— 倒车辅助线开关 ----------------------
    // 复用 KEY1~KEY5 那组 5 位去抖器（key_sel_pulse[4]），不再单独占一个键。
    wire key_aux_pulse = key_sel_pulse[4] | uart_key_aux_pulse;

    reg aux_on_sys;         // POWER-ON STATE = OFF（与增强/去雾一致，开机不出线）
    always @(posedge sys_clk or negedge rstn_out) begin
        if (!rstn_out)                          aux_on_sys <= 1'b0;
        else if (key_aux_pulse & key_boot_done) aux_on_sys <= ~aux_on_sys;
    end

    reg aux_on_s0, aux_on_pix;
    always @(posedge pix_clk_w or negedge rstn_out) begin
        if (!rstn_out) {aux_on_pix, aux_on_s0} <= 2'b00;
        else           {aux_on_pix, aux_on_s0} <= {aux_on_s0, aux_on_sys};
    end

    // ---- 260915 V4: unified post-select processing chain + joint encoding --
    // One chain serves ALL FIVE streams: u_vsel -> enhance -> defog -> 888.
    // ctrl (u_vsel o_ctrl) is the mode bit latched at the same color_vs
    // boundary as the muxed data; the defog / aux states are latched at the
    // output frame boundary (sel_vs edge). Both change ONLY in blanking:
    //   enc_* = ctrl | (~key_*_f)   KEY answers "user wants it on",
    //                               ctrl forces bypass on stitch frames
    //                               (the write side already enhanced the tiles).
    // This replaces the old per-module formulas (sel_pix_s1[2] | ~KEY) and the
    // separate defog_en_eff latch — same frame-boundary hygiene, one place for
    // both keys. Do NOT add any further synchronizer on ctrl (it is already
    // pix_clk_w-native and frame-latched; another stage would make it late).
    wire        ctrl;                       // 1 = stitch on screen / 0 = single (u_vsel o_ctrl)
    wire        ch1_hs, ch1_vs, ch1_de;     // enhance stage output
    wire [15:0] ch1_data;
    wire        ch2_hs, ch2_vs, ch2_de;     // defog stage output (== HDMI timing)
    wire [15:0] ch2_data;
    reg         sel_vs_r;                   // 1-beat vs pipeline（帧边界锁存仍要用它）
                                            // sel_hs_r 已退役：hs 改由 luma_adapt 内部同深延迟
    reg         key_defog_f;                // 单路去雾状态（帧边界锁存，源=KEY5/串口'4'）
    reg         key_aux_f;                  // 261019: aux-line state latched at frame boundary

    // joint encoder (plan V4 §4.2.3): one frame-boundary latch feeds both keys
    // 261012c: key_median_f 位随中值滤波级移除（恢复原 2 位锁存）
    always @(posedge pix_clk_w or negedge rstn_out) begin
        if (!rstn_out) begin
            {sel_vs_r, key_aux_f, key_defog_f} <= 3'b000;
        end else begin
            sel_vs_r <= sel_vs;                        // vs pipeline (edge detect + stage delay)
            if (sel_vs & ~sel_vs_r)                    // frame start of the ON-SCREEN stream
                {key_aux_f, key_defog_f} <= {aux_on_pix, defog_on_pix};
        end
    end
    // 261023: ctrl=1（拼接）时仍强制旁路 720P 去雾 —— 这个旁路的语义现在是
    // "拼接画面已由写侧四路各自去雾，读侧不再重复处理"，与新增的四个
    // defog_rgb 是互补关系，而不是原来的"拼接没有去雾"。
    wire enc_defog = ctrl | (~key_defog_f); // {ctrl, KEY4/串口'3'} -> 720P defog bypass

    // ---- KEY2 / KEY3 —— 模块 A 的亮度自适应开关（单键翻转，只有 开 / 关）----
    //   单路亮度自适应（增亮+增暗）：KEY2(L15) / 串口 '1' -> key_enh_pulse
    // 261031：夜景修复（KEY3 J17 / 串口 '2'）整级删除，模块 A 只剩这一路开关。
    // 打开后模块内部自己按 mean_v 决定做多少；关掉就是直通。上电全关。
    reg l_on_sys;
    always @(posedge sys_clk or negedge rstn_out) begin
        if (!rstn_out) begin
            l_on_sys <= 1'b0;
        end else if (key_boot_done) begin
            if (key_enh_pulse)   l_on_sys <= ~l_on_sys;
        end
    end

    // 2FF 同步 -> on-screen 帧起始锁存（与 key_*_f 同拍）
    reg l_s0, l_s1;
    always @(posedge pix_clk_w or negedge rstn_out) begin
        if (!rstn_out) begin
            l_s0<=1'b0; l_s1<=1'b0;
        end else begin
            l_s0<=l_on_sys; l_s1<=l_s0;
        end
    end

    reg l_en_f;
    always @(posedge pix_clk_w or negedge rstn_out) begin
        if (!rstn_out) begin
            l_en_f<=1'b0;
        end else if (sel_vs & ~sel_vs_r) begin
            l_en_f<=l_s1;
        end
    end

    wire [1:0] luma_gear;

    // 261028: u_enhance_sng 换成模块 A（luma_adapt）。
    // hs/vs 由模块内部按 LAT=15 同深延迟，顶层不再需要 sel_hs_r 那 1 拍。
    // 开关 l_en 在帧起始锁存；关掉 = 直通（bank0）。
    // ctrl（拼接模式）另外强制 bank0 —— 写侧四路已各自处理过。
    // 261031: 夜景修复端口 n_en 已随该级删除，模块 A 现在只有一路开关。
    luma_adapt #(
        .H_ACT (11'd1280),
        .V_ACT (10'd720)
    ) u_luma (
        .clk      (pix_clk_w),
        .rst_n    (rstn_out),
        .hs_in    (sel_hs),
        .vs_in    (sel_vs),
        .de_in    (sel_de),
        .data_in  (sel_data),
        .ctrl     (ctrl),
        .l_en     (l_en_f),
        .hs_out   (ch1_hs),
        .vs_out   (ch1_vs),
        .de_out   (ch1_de),
        .data_out (ch1_data),
        .gear_o   (luma_gear)
    );

    // ---- 261028: defog 像素流水 + hs/vs 同深对齐 ------------------------
    // u_defog_sng 由零延迟改为 PIX_DLY=1：把 px_cnt -> rcp_use MUX 整段从
    //   像素通路里切出去（它是 pix_clk 的第二薄路径，12.2 ns / 13.47 ns）。
    // 旧注释"读侧必须 PIX_DLY=0"的前提是"hs/vs 不跟数据走 ⇒ 延迟 = 右移 1 px"。
    //   现在 hs/vs 与 de/data 按同一深度打拍，该前提解除；但两者**必须同深**，
    //   差一拍就是画面右移 / 上一行。要改档位只改下面这一个 localparam。
    // 回退：DEFOG_DLY = 0 即恢复原来的零延迟（hs/vs 链路自动跟着退）。
    localparam DEFOG_DLY = 1;               // 0 / 1 / 2，必须与 .PIX_DLY 一致

    defog_rgb #(
        .H_ACT  (11'd1280),
        .V_ACT  (10'd720),
        .PIX_DLY(DEFOG_DLY)
    ) u_defog_sng (
        .clk      (pix_clk_w),
        .rst_n    (rstn_out),
        .bypass   (enc_defog),                // V4: joint encoding (was sel_pix_s1[2] | (~defog_on_pix))
        .de_in    (ch1_de),
        .data_in  (ch1_data),
        .de_out   (ch2_de),                   // = ch1_de 延迟 DEFOG_DLY 拍（与 data 同相）
        .data_out (ch2_data)
    );

    // hs/vs 与 defog 像素流水同深（两级移位链，按 DEFOG_DLY 取抽头）
    reg [1:0] hs_sr, vs_sr;
    always @(posedge pix_clk_w or negedge rstn_out) begin
        if (!rstn_out) begin
            {hs_sr, vs_sr} <= 4'b0000;
        end else begin
            hs_sr <= {hs_sr[0], ch1_hs};
            vs_sr <= {vs_sr[0], ch1_vs};
        end
    end
    assign ch2_hs = (DEFOG_DLY >= 2) ? hs_sr[1] :
                    (DEFOG_DLY == 1) ? hs_sr[0] : ch1_hs;
    assign ch2_vs = (DEFOG_DLY >= 2) ? vs_sr[1] :
                    (DEFOG_DLY == 1) ? vs_sr[0] : ch1_vs;

    // ---- 261019: 倒车辅助线叠加（单路，增强/去雾之后、标识行之前）--------
    // 位置硬约束：必须在 enhance/defog 之后（纯色会被算法改写），u_marker 之前。
    // 零延迟（o_vs/o_de 直通、o_data 组合替换），hs/vs/de/data 保持位对齐。
    // 261019b: 模式位收进 u_aux —— 输入 mode_in=ctrl / aux_en=key_aux_f（均帧锁存），
    // 模块内自决 bypass 与 aux_ctrl；aux_ctrl 只喂 u_marker，真实拼接判读仍取 o_ctrl。

    // 261024: 角度 sys_clk -> pix_clk_w（数据总线 + 翻转标志，2FF 同步后锁存）
    //   角度只在合法帧更新（最快 0.52ms 一次），翻转同步约 3 个 pix 时钟（40ns），
    //   总线在标志被采到之前早已稳定，不会出现"总线/标志错拍"。
    //   真正的帧边界由 u_aux 内部的 vs 上升沿再锁一次。
    reg        traj_req_sys;
    always @(posedge sys_clk or negedge rstn_out) begin
        if (!rstn_out)           traj_req_sys <= 1'b0;
        else if (traj_angle_vld) traj_req_sys <= ~traj_req_sys;
    end

    reg  [2:0] traj_req_p;
    reg  [5:0] angle_pix;
    always @(posedge pix_clk_w or negedge rstn_out) begin
        if (!rstn_out) begin
            traj_req_p <= 3'b000;
            angle_pix  <= 6'd0;              // 上电默认直行
        end else begin
            traj_req_p <= {traj_req_p[1:0], traj_req_sys};
            if (traj_req_p[1] ^ traj_req_p[2])
                angle_pix <= traj_angle_sys;
        end
    end

    wire        ax_vs, ax_de;
    wire [15:0] ax_data;
    wire        ax_ctrl;

    reverse_aux_lines #(
        .H_ACT  (11'd1280),
        .V_ACT  (10'd720)
    ) u_aux (
        .clk       (pix_clk_w),
        .rst_n     (rstn_out),
        .mode_in   (ctrl),
        .aux_en    (key_aux_f),
        .angle_in  (angle_pix),
        .vs        (ch2_vs),
        .de        (ch2_de),
        .data      (ch2_data),
        .o_vs      (ax_vs),
        .o_de      (ax_de),
        .o_data    (ax_data),
        .aux_ctrl  (ax_ctrl)
    );

    // ---- final 2:1 (stitch / single), frame-boundary latched ----
    // V4 260915: hs/vs/de/data are selected TOGETHER (geometry can no longer
    // drift when the downstream chain changes), and the same boundary latch
    // produces o_ctrl — the mode bit that drives the joint encoder above.
    video_select u_vsel (
        .clk        (pix_clk_w),
        .rst_n      (rstn_out),
        .sel        (sel_pix_s1),
        .color_vs   (color_vs),
        .splice_hs  (v3_hs),
        .splice_vs  (v3_vs),
        .splice_de  (v3_de),
        .splice_data(v3_data),
        .single_hs  (sng_hs),
        .single_vs  (sng_vs),
        .single_de  (sng_de),
        .single_data(sng_data),
        .o_hs       (sel_hs),
        .o_vs       (sel_vs),
        .o_de       (sel_de),
        .o_data     (sel_data),
        .o_ctrl     (ctrl)
    );


/////////////////////////////////////////////////////////////////////////////////////
// V4 第 3 步：链尾标识行注入（升级方案 §11.3）
// 位置硬约束：必须在本链最末端（defog 之后、888 之前）—— image_enhance 与
// defog_rgb 都会改写纯白 / 纯黑，标识行注早了会被算法吃掉。mode_ctrl 直连
// u_vsel 的 o_ctrl（帧锁存模式位），不译码、不再加同步器（多一级会晚一帧）。
// 本模块零延迟：o_de/o_vs 直通、o_data 纯组合替换 ⇒ 画面几何不受影响。
// 261019: mode_ctrl 扩展为 ctrl | key_aux_f —— 两者同为帧边界锁存位，
// 标识行颜色与辅助线画面严格同帧切换，无错位帧。
/////////////////////////////////////////////////////////////////////////////////////
    wire        mk_vs, mk_de;               // == ch2_vs / ch2_de（模块内直通）
    wire [15:0] mk_data;

    marker_row_add #(
        .H_ACT  (11'd1280),
        .V_ACT  (10'd720)
    ) u_marker (
        .clk       (pix_clk_w),
        .rst_n     (rstn_out),
        .mode_ctrl (ax_ctrl),               // 261019b: u_aux 输出（拼接 或 单路+辅助线开 = 白行）
        .bypass    (1'b0),                  // 置 1 = 整体关闭标识行（回退点）
        .vs        (ax_vs),
        .de        (ax_de),
        .data      (ax_data),
        .o_vs      (mk_vs),
        .o_de      (mk_de),
        .o_data    (mk_data)
    );

/////////////////////////////////////////////////////////////////////////////////////
// RGB565 → RGB888 conversion (before HDMI output)
// 260911: source switched from v3_data (stitch) to the video_select output.
// 260915 V4: source is now the END of the unified chain (ch2_* = defog out).
// The chain delays the whole picture uniformly (enhance +1 beat, defog 0), so
// hs/vs/de/data stay bit-aligned — geometry is unchanged by construction.
// 260915c V4 第 3 步: 末尾再插一级 marker_row_add（行 0 标识行），它零延迟，
// 所以这里只是把像素源由 ch2_data 换成 mk_data，几何依旧不变。
// 261019: ch2 -> reverse_aux_lines(零延迟) -> marker，源换成 ax_*，几何仍不变。
/////////////////////////////////////////////////////////////////////////////////////
    wire [7:0] sel_r, sel_g, sel_b;
    assign sel_r = {mk_data[15:11], 3'b0};
    assign sel_g = {mk_data[10:5],  2'b0};
    assign sel_b = {mk_data[4:0],   3'b0};

/////////////////////////////////////////////////////////////////////////////////////
// HDMI Output
/////////////////////////////////////////////////////////////////////////////////////
    always@(posedge pix_clk_w) begin
        r_out  <= sel_r;
        g_out  <= sel_g;
        b_out  <= sel_b;
        vs_out <= mk_vs;       // V4: timing rides the chain together with de/data
        hs_out <= ch2_hs;      // marker_row_add 不碰 hs（它只换行 0 的像素值）
        de_out <= mk_de;
    end

/////////////////////////////////////////////////////////////////////////////////////
// 261028b: 单路 720P 去雾（K16 / 串口 '3'）与 拼接增亮（J19 / 串口 '5'）
// 的历史注释：这里的 gamma_bypass 现在只当"写侧四路 image_enhance 的旁路"用。
/////////////////////////////////////////////////////////////////////////////////////
    // ---- KEY4 (K16) / 串口 '3' —— 单路 720P 去雾开关 -------------------
    wire key_defog_pulse;       // single-cycle pulse (K16 OR uart '3')

    assign key_defog_pulse = key_sel_pulse[3] | uart_key_defog_pulse;

    reg defog_on_sys;       // POWER-ON STATE = OFF: 0 -> enc_defog = ctrl | ~0 = 1 = bypass
    always @(posedge sys_clk or negedge rstn_out) begin
        if (!rstn_out) begin
            defog_on_sys <= 1'b0;
        end else if (key_defog_pulse & key_boot_done) begin
            defog_on_sys <= ~defog_on_sys;
        end
    end

    reg defog_on_s0, defog_on_pix;          // CDC: sys_clk -> pix_clk_w
    always @(posedge pix_clk_w or negedge rstn_out) begin
        if (!rstn_out) begin
            {defog_on_pix, defog_on_s0} <= 2'b00;
        end else begin
            {defog_on_pix, defog_on_s0} <= {defog_on_s0, defog_on_sys};
        end
    end

    // ---- 拼接增亮的状态位（变量名沿用 gamma_bypass）--------------------
    // 0 = 写侧四路 image_enhance 工作（增亮开），1 = 直通。
    // POWER-ON = 1（关）：上电全关，与其余七个开关一致。
    reg gamma_bypass;
    always @(posedge sys_clk or negedge rstn_out) begin
        if (!rstn_out) begin
            gamma_bypass <= 1'b1;
        end else if (key_enh360_pulse & key_boot_done) begin
            gamma_bypass <= ~gamma_bypass;
        end
    end

/////////////////////////////////////////////////////////////////////////////////////
// Heartbeat
/////////////////////////////////////////////////////////////////////////////////////
    always@(posedge core_clk) begin
        if (!ddr_init_done)          cnt <= 27'd0;
        else if (cnt >= TH_1S)       cnt <= 27'd0;
        else                         cnt <= cnt + 27'd1;
    end
    always @(posedge core_clk) begin
        if (!ddr_init_done)          heart_beat_led <= 1'd1;
        else if (cnt >= TH_1S)       heart_beat_led <= ~heart_beat_led;
    end

endmodule
