`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 260909: 以太网收流前端 (从 ETHVIDEO_TRY 工程移植) —— 作为四宫格第 4 路 (CH3, 右下角)
// 数据流: 网口(Ethernet2) -> rgmii_interface -> mac/ip/udp -> 55AA重组 -> RGB565 输出
// 输出语义与本工程 CH2(HDMI输入) 写侧一致: vsync=帧起始脉冲(上升沿触发写请求),
//           de=像素有效, data[15:0]=RGB565 大端 {R[4:0],G[5:0],B[4:0]}, 时钟=rgmii_clk 125M
// 复位: sys_clk(50M) 计数 1ms -> rstn;  phy_rstn = rstn 后再保持 50ms 低 (RTL8211E 时序)
//////////////////////////////////////////////////////////////////////////////////
`timescale 1ns / 1ps
module eth_video_rx#(
    parameter       LOCAL_MAC = 48'ha0_b1_c2_d3_e1_e1,
    parameter       LOCAL_IP  = 32'hC0_A8_01_0B,//192.168.1.11
    parameter       LOCL_PORT = 16'h1F40,       //8000（260910 修正：原 1F90 实为 8080）
    parameter       DEST_IP   = 32'hC0_A8_01_69,//192.168.1.105
    parameter       DEST_PORT = 16'h1F40
)(
    input        sys_clk,               // 50MHz (复用本工程 sys_clk, 仅用于复位生成)
    input        rgmii_rxc,
    input        rgmii_rx_ctl,
    input [3:0]  rgmii_rxd,
    output       rgmii_txc,
    output       rgmii_tx_ctl,
    output [3:0] rgmii_txd,
    output       phy_rstn,

    // 视频输出 (rgmii_clk 125M 域)
    output       eth_clk,               // rgmii_clk 125M, 下游缩放/写侧时钟
    output       eth_video_vs,          // 帧起始单拍脉冲
    output       eth_video_de,          // 像素有效
    output [15:0] eth_video_data,       // RGB565 大端, 1280x720@60

    // 调试 LED (高有效): A5=UDP数据活动  F8=载荷首字节55出现过  F7=解析进过EXTRACT
    output       eth_udp_led,
    output       eth_hdr55_led,
    output       eth_extract_led
);

    // ------------------------------------------------------------ 复位
    reg [15:0] rst_cnt = 16'd0;
    reg        rst_done = 1'b0;
    always @(posedge sys_clk) begin
        if(rst_cnt == 16'd50_000) rst_done <= 1'b1;   // 1ms @ 50MHz
        else                      rst_cnt  <= rst_cnt + 1'b1;
    end
    wire rstn = rst_done;

    reg  [22:0] phy_rst_cnt = 23'd0;
    reg         phy_rst_done = 1'b0;
    always @(posedge sys_clk or negedge rstn) begin
        if(!rstn) begin
            phy_rst_cnt  <= 23'd0;
            phy_rst_done <= 1'b0;
        end else if(!phy_rst_done) begin
            if(phy_rst_cnt == 23'd2_500_000) phy_rst_done <= 1'b1; // 50ms @ 50MHz
            else                             phy_rst_cnt  <= phy_rst_cnt + 1'b1;
        end
    end
    assign phy_rstn = rstn & phy_rst_done;

    // ------------------------------------------------------------ 以太网收流
    wire        rgmii_clk;
    wire        mac_rx_data_valid;
    wire [7:0]  mac_rx_data;
    wire        mac_data_valid;
    wire [7:0]  mac_tx_data;
    wire        udp_rec_data_valid/*synthesis PAP_MARK_DEBUG="1"*/;

    eth_udp_test #(
        .LOCAL_MAC                (LOCAL_MAC        ),
        .LOCAL_IP                 (LOCAL_IP         ),
        .LOCL_PORT                (LOCL_PORT        ),
        .DEST_IP                  (DEST_IP          ),
        .DEST_PORT                (DEST_PORT        )
    ) eth_udp_test (
        .rgmii_clk                (rgmii_clk        ),
        .rstn                     (rstn             ),
        .gmii_rx_dv               (mac_rx_data_valid),
        .gmii_rxd                 (mac_rx_data      ),
        .gmii_tx_en               (mac_data_valid   ),
        .gmii_txd                 (mac_tx_data      ),

        .udp_rec_data_valid       (udp_rec_data_valid),
        .udp_rec_rdata            (                 ),
        .udp_rec_data_length      (                 ),

        .eth_video_vs             (eth_video_vs     ),
        .eth_video_de             (eth_video_de     ),
        .eth_video_data           (eth_video_data   ),

        .dbg_hdr55                (eth_hdr55_led    ),
        .dbg_extract              (eth_extract_led  )
    );

    wire pll_lock;
    wire rgmii_rxc_phase;
    wire rgmii_rxc_phase_d;

    pll_phase pll_phase (
        .clkin1   (rgmii_rxc        ),
        .pll_lock (pll_lock         ),
        .clkout0  (rgmii_rxc_phase  ),
        .clkout1  (rgmii_rxc_phase_d)
    );

    rgmii_interface rgmii_interface(
        .rst                      (~rstn             ),
        .rgmii_clk                (rgmii_clk         ),
        .rgmii_clk_90p            (                  ),

        .mac_tx_data_valid        (mac_data_valid    ),
        .mac_tx_data              (mac_tx_data       ),

        .mac_rx_error             (                  ),
        .mac_rx_data_valid        (mac_rx_data_valid ),
        .mac_rx_data              (mac_rx_data       ),

        .rgmii_rxc                (rgmii_rxc_phase   ),
        .rgmii_rx_ctl             (rgmii_rx_ctl      ),
        .rgmii_rxd                (rgmii_rxd         ),

        .rgmii_rxc_phase_d        (rgmii_rxc_phase_d ),
        .rgmii_txc                (rgmii_txc         ),
        .rgmii_tx_ctl             (rgmii_tx_ctl      ),
        .rgmii_txd                (rgmii_txd         )
    );

    // ------------------------------------------------------------ A5: 帧流心跳灯
    // 260910 语义升级: 每 16 个帧起始脉冲(vsync)翻转一次。
    // 30fps 推流时约 0.53Hz 慢闪 = 帧流活着; 常亮/常灭 = 没有完整帧到达
    // (此前为 UDP 数据活动保持灯, 只能证明有包, 不能证明帧完整, 已弃用)
    reg [3:0] vs_div;
    always @(posedge rgmii_clk or negedge rstn) begin
        if(!rstn)
            vs_div <= 4'd0;
        else if(eth_video_vs)
            vs_div <= vs_div + 4'd1;
    end
    assign eth_udp_led = vs_div[3];
    assign eth_clk     = rgmii_clk;

endmodule
