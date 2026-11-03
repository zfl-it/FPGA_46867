//
// Generated (version 2022.2-SP6.4<build 146967>) at Wed Sep 17 01:03:37 2025
//

module pcie_img_mwr_ctrl
(
    input [63:0] i_video_dma_addr,
    input [9:0] i_video_dma_len,
    input i_dma_32or64,
    input i_mwr32_req_ack,
    input i_mwr64_req_ack,
    input i_video_dma_req,
    input pcie_clk,
    input rst_n,
    output [63:0] o_mwr_req_addr,
    output [9:0] o_mwr_req_length,
    output o_dma_cmd_rdy,
    output o_mwr32_req,
    output o_mwr64_req
);
	// SDC constraint : (object pcie_clk) (id 1000) (clock pcie_img_mwr_ctrl|pcie_clk) (inferred)
    wire N1_0;
    wire N4;
    wire N14;
    wire N19;
    wire N23;
    wire N30;
    wire N33;
    wire N37;
    wire r_video_dma_req_d0;
    wire r_video_dma_req_d1;
    wire r_video_dma_req_d2;
    wire r_video_dma_req_d3;
    wire r_video_dma_req_d4;

    GTP_INV N1_0_vname (
            .Z (N1_0),
            .I (rst_n));
    // defparam N1_0_vname.orig_name = N1_0;

    GTP_LUT2 /* N4 */ #(
            .INIT(4'b0010))
        N4_vname (
            .Z (N4),
            .I0 (r_video_dma_req_d1),
            .I1 (r_video_dma_req_d2));
    // defparam N4_vname.orig_name = N4;
	// LUT = I0&~I1 ;
	// ../../src/pcie_img_mwr_crtl.v:60

    GTP_LUT6D /* N14 */ #(
            .INIT(64'b0000010000000100000001000000010000001000000010000000100000001000))
        N14_vname (
            .Z (N14),
            .Z5 (N19),
            .I0 (i_dma_32or64),
            .I1 (r_video_dma_req_d3),
            .I2 (r_video_dma_req_d4),
            .I3 (),
            .I4 (),
            .I5 (1'b1));
    // defparam N14_vname.orig_name = N14;
	// LUT = ~I0&I1&~I2 ;
	// Z5 = I0&I1&~I2 ;
	// ../../src/pcie_img_mwr_crtl.v:83

    GTP_LUT2 /* N23 */ #(
            .INIT(4'b1110))
        N23_vname (
            .Z (N23),
            .I0 (o_mwr32_req),
            .I1 (o_mwr64_req));
    // defparam N23_vname.orig_name = N23;
	// LUT = (I0)|(I1) ;
	// ../../src/pcie_img_mwr_crtl.v:115

    GTP_LUT2 /* N30 */ #(
            .INIT(4'b1011))
        N30_vname (
            .Z (N30),
            .I0 (i_mwr32_req_ack),
            .I1 (rst_n));
    // defparam N30_vname.orig_name = N30;
	// LUT = (~I1)|(I0) ;
	// ../../src/pcie_img_mwr_crtl.v:19

    GTP_LUT2 /* N33 */ #(
            .INIT(4'b1011))
        N33_vname (
            .Z (N33),
            .I0 (i_mwr64_req_ack),
            .I1 (rst_n));
    // defparam N33_vname.orig_name = N33;
	// LUT = (~I1)|(I0) ;
	// ../../src/pcie_img_mwr_crtl.v:21

    GTP_LUT5 /* N37 */ #(
            .INIT(32'b00000000000011101111111111111111))
        N37_vname (
            .Z (N37),
            .I0 (i_mwr32_req_ack),
            .I1 (i_mwr64_req_ack),
            .I2 (o_mwr32_req),
            .I3 (o_mwr64_req),
            .I4 (rst_n));
    // defparam N37_vname.orig_name = N37;
	// LUT = (~I4)|(I0&~I2&~I3)|(I1&~I2&~I3) ;
	// ../../src/pcie_img_mwr_crtl.v:15

    GTP_DFF_SE /* o_dma_cmd_rdy */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b1))
        o_dma_cmd_rdy_vname (
            .Q (o_dma_cmd_rdy),
            .CE (N23),
            .CLK (pcie_clk),
            .D (1'b0),
            .S (N37));
    // defparam o_dma_cmd_rdy_vname.orig_name = o_dma_cmd_rdy;
	// ../../src/pcie_img_mwr_crtl.v:112

    GTP_DFF_RE /* o_mwr32_req */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        o_mwr32_req_vname (
            .Q (o_mwr32_req),
            .CE (N14),
            .CLK (pcie_clk),
            .D (1'b1),
            .R (N30));
    // defparam o_mwr32_req_vname.orig_name = o_mwr32_req;
	// ../../src/pcie_img_mwr_crtl.v:78

    GTP_DFF_RE /* o_mwr64_req */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        o_mwr64_req_vname (
            .Q (o_mwr64_req),
            .CE (N19),
            .CLK (pcie_clk),
            .D (1'b1),
            .R (N33));
    // defparam o_mwr64_req_vname.orig_name = o_mwr64_req;
	// ../../src/pcie_img_mwr_crtl.v:91

    GTP_DFF_RE /* \o_mwr_req_addr[0]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[0]  (
            .Q (o_mwr_req_addr[0]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[0]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[1]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[1]  (
            .Q (o_mwr_req_addr[1]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[1]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[2]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[2]  (
            .Q (o_mwr_req_addr[2]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[2]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[3]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[3]  (
            .Q (o_mwr_req_addr[3]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[3]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[4]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[4]  (
            .Q (o_mwr_req_addr[4]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[4]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[5]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[5]  (
            .Q (o_mwr_req_addr[5]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[5]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[6]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[6]  (
            .Q (o_mwr_req_addr[6]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[6]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[7]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[7]  (
            .Q (o_mwr_req_addr[7]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[7]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[8]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[8]  (
            .Q (o_mwr_req_addr[8]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[8]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[9]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[9]  (
            .Q (o_mwr_req_addr[9]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[9]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[10]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[10]  (
            .Q (o_mwr_req_addr[10]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[10]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[11]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[11]  (
            .Q (o_mwr_req_addr[11]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[11]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[12]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[12]  (
            .Q (o_mwr_req_addr[12]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[12]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[13]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[13]  (
            .Q (o_mwr_req_addr[13]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[13]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[14]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[14]  (
            .Q (o_mwr_req_addr[14]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[14]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[15]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[15]  (
            .Q (o_mwr_req_addr[15]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[15]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[16]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[16]  (
            .Q (o_mwr_req_addr[16]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[16]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[17]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[17]  (
            .Q (o_mwr_req_addr[17]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[17]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[18]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[18]  (
            .Q (o_mwr_req_addr[18]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[18]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[19]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[19]  (
            .Q (o_mwr_req_addr[19]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[19]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[20]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[20]  (
            .Q (o_mwr_req_addr[20]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[20]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[21]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[21]  (
            .Q (o_mwr_req_addr[21]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[21]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[22]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[22]  (
            .Q (o_mwr_req_addr[22]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[22]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[23]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[23]  (
            .Q (o_mwr_req_addr[23]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[23]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[24]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[24]  (
            .Q (o_mwr_req_addr[24]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[24]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[25]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[25]  (
            .Q (o_mwr_req_addr[25]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[25]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[26]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[26]  (
            .Q (o_mwr_req_addr[26]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[26]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[27]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[27]  (
            .Q (o_mwr_req_addr[27]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[27]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[28]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[28]  (
            .Q (o_mwr_req_addr[28]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[28]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[29]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[29]  (
            .Q (o_mwr_req_addr[29]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[29]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[30]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[30]  (
            .Q (o_mwr_req_addr[30]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[30]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[31]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[31]  (
            .Q (o_mwr_req_addr[31]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[31]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[32]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[32]  (
            .Q (o_mwr_req_addr[32]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[32]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[33]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[33]  (
            .Q (o_mwr_req_addr[33]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[33]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[34]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[34]  (
            .Q (o_mwr_req_addr[34]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[34]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[35]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[35]  (
            .Q (o_mwr_req_addr[35]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[35]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[36]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[36]  (
            .Q (o_mwr_req_addr[36]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[36]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[37]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[37]  (
            .Q (o_mwr_req_addr[37]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[37]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[38]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[38]  (
            .Q (o_mwr_req_addr[38]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[38]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[39]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[39]  (
            .Q (o_mwr_req_addr[39]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[39]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[40]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[40]  (
            .Q (o_mwr_req_addr[40]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[40]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[41]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[41]  (
            .Q (o_mwr_req_addr[41]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[41]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[42]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[42]  (
            .Q (o_mwr_req_addr[42]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[42]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[43]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[43]  (
            .Q (o_mwr_req_addr[43]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[43]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[44]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[44]  (
            .Q (o_mwr_req_addr[44]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[44]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[45]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[45]  (
            .Q (o_mwr_req_addr[45]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[45]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[46]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[46]  (
            .Q (o_mwr_req_addr[46]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[46]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[47]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[47]  (
            .Q (o_mwr_req_addr[47]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[47]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[48]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[48]  (
            .Q (o_mwr_req_addr[48]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[48]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[49]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[49]  (
            .Q (o_mwr_req_addr[49]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[49]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[50]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[50]  (
            .Q (o_mwr_req_addr[50]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[50]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[51]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[51]  (
            .Q (o_mwr_req_addr[51]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[51]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[52]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[52]  (
            .Q (o_mwr_req_addr[52]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[52]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[53]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[53]  (
            .Q (o_mwr_req_addr[53]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[53]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[54]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[54]  (
            .Q (o_mwr_req_addr[54]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[54]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[55]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[55]  (
            .Q (o_mwr_req_addr[55]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[55]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[56]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[56]  (
            .Q (o_mwr_req_addr[56]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[56]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[57]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[57]  (
            .Q (o_mwr_req_addr[57]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[57]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[58]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[58]  (
            .Q (o_mwr_req_addr[58]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[58]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[59]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[59]  (
            .Q (o_mwr_req_addr[59]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[59]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[60]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[60]  (
            .Q (o_mwr_req_addr[60]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[60]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[61]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[61]  (
            .Q (o_mwr_req_addr[61]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[61]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[62]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[62]  (
            .Q (o_mwr_req_addr[62]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[62]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_addr[63]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_addr[63]  (
            .Q (o_mwr_req_addr[63]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_addr[63]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:67

    GTP_DFF_RE /* \o_mwr_req_length[0]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_length[0]  (
            .Q (o_mwr_req_length[0]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_len[0]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:57

    GTP_DFF_RE /* \o_mwr_req_length[1]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_length[1]  (
            .Q (o_mwr_req_length[1]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_len[1]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:57

    GTP_DFF_RE /* \o_mwr_req_length[2]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_length[2]  (
            .Q (o_mwr_req_length[2]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_len[2]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:57

    GTP_DFF_RE /* \o_mwr_req_length[3]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_length[3]  (
            .Q (o_mwr_req_length[3]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_len[3]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:57

    GTP_DFF_RE /* \o_mwr_req_length[4]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_length[4]  (
            .Q (o_mwr_req_length[4]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_len[4]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:57

    GTP_DFF_RE /* \o_mwr_req_length[5]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_length[5]  (
            .Q (o_mwr_req_length[5]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_len[5]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:57

    GTP_DFF_RE /* \o_mwr_req_length[6]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_length[6]  (
            .Q (o_mwr_req_length[6]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_len[6]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:57

    GTP_DFF_RE /* \o_mwr_req_length[7]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_length[7]  (
            .Q (o_mwr_req_length[7]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_len[7]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:57

    GTP_DFF_RE /* \o_mwr_req_length[8]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_length[8]  (
            .Q (o_mwr_req_length[8]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_len[8]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:57

    GTP_DFF_RE /* \o_mwr_req_length[9]  */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        \o_mwr_req_length[9]  (
            .Q (o_mwr_req_length[9]),
            .CE (N4),
            .CLK (pcie_clk),
            .D (i_video_dma_len[9]),
            .R (N1_0));
	// ../../src/pcie_img_mwr_crtl.v:57

    GTP_DFF /* r_video_dma_req_d0 */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        r_video_dma_req_d0_vname (
            .Q (r_video_dma_req_d0),
            .CLK (pcie_clk),
            .D (i_video_dma_req));
    // defparam r_video_dma_req_d0_vname.orig_name = r_video_dma_req_d0;
	// ../../src/pcie_img_mwr_crtl.v:48

    GTP_DFF /* r_video_dma_req_d1 */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        r_video_dma_req_d1_vname (
            .Q (r_video_dma_req_d1),
            .CLK (pcie_clk),
            .D (r_video_dma_req_d0));
    // defparam r_video_dma_req_d1_vname.orig_name = r_video_dma_req_d1;
	// ../../src/pcie_img_mwr_crtl.v:48

    GTP_DFF /* r_video_dma_req_d2 */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        r_video_dma_req_d2_vname (
            .Q (r_video_dma_req_d2),
            .CLK (pcie_clk),
            .D (r_video_dma_req_d1));
    // defparam r_video_dma_req_d2_vname.orig_name = r_video_dma_req_d2;
	// ../../src/pcie_img_mwr_crtl.v:48

    GTP_DFF /* r_video_dma_req_d3 */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        r_video_dma_req_d3_vname (
            .Q (r_video_dma_req_d3),
            .CLK (pcie_clk),
            .D (r_video_dma_req_d2));
    // defparam r_video_dma_req_d3_vname.orig_name = r_video_dma_req_d3;
	// ../../src/pcie_img_mwr_crtl.v:48

    GTP_DFF /* r_video_dma_req_d4 */ #(
            .GRS_EN("TRUE"), 
            .INIT(1'b0))
        r_video_dma_req_d4_vname (
            .Q (r_video_dma_req_d4),
            .CLK (pcie_clk),
            .D (r_video_dma_req_d3));
    // defparam r_video_dma_req_d4_vname.orig_name = r_video_dma_req_d4;
	// ../../src/pcie_img_mwr_crtl.v:48


endmodule

