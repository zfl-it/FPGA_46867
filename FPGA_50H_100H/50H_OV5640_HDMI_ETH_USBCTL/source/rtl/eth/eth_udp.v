`timescale 1ns / 1ps
module eth_udp_test#(
    parameter       LOCAL_MAC = 48'h11_11_11_11_11_11,
    parameter       LOCAL_IP  = 32'hC0_A8_01_6E,//192.168.1.110
    parameter       LOCL_PORT = 16'h8080,

    parameter       DEST_IP   = 32'hC0_A8_01_69,//192.168.1.105 
    parameter       DEST_PORT = 16'h8080 
)(
    input                rgmii_clk,
    input                rstn,
    input                gmii_rx_dv/*synthesis PAP_MARK_DEBUG="1"*/,
    input  [7:0]         gmii_rxd/*synthesis PAP_MARK_DEBUG="1"*/,
    output reg           gmii_tx_en,
    output reg [7:0]     gmii_txd,

    output               udp_rec_data_valid/*synthesis PAP_MARK_DEBUG="1"*/,
    output [7:0]         udp_rec_rdata /*synthesis PAP_MARK_DEBUG="1"*/,
    output [15:0]        udp_rec_data_length,

    output               eth_video_vs/*synthesis PAP_MARK_DEBUG="1"*/,
    output               eth_video_de/*synthesis PAP_MARK_DEBUG="1"*/,
    output [15:0]        eth_video_data/*synthesis PAP_MARK_DEBUG="1"*/,

    // 260910 LED 语义升级(端口名保持顶层兼容, 引脚不变):
    //   dbg_hdr55(F8) -> 包长异常锁存: 某个 UDP 包载荷字节数既不是 1471(满包)
    //                    也不是 31(末包) —— 字节滑移/截断的直接证据
    //   dbg_extract(F7) -> 帧补黑激活锁存: 发生过"一帧像素不足、已自动补黑"
    output reg           dbg_hdr55,
    output reg           dbg_extract
);

// 字节计数器
reg [11:0] byte_cnt/*synthesis PAP_MARK_DEBUG="1"*/;
always @(posedge rgmii_clk or negedge rstn) begin
    if(!rstn) begin
        byte_cnt <= 12'd0;
    end
    else begin
        if(udp_rec_data_valid)
            byte_cnt <= byte_cnt + 1'b1;
        else
            byte_cnt <= 12'd0;
    end
end

// 包头帧头鉴别暂存
reg [7:0] byte_1/*synthesis PAP_MARK_DEBUG="1"*/, byte_2/*synthesis PAP_MARK_DEBUG="1"*/;
always @(posedge rgmii_clk or negedge rstn) begin
    if(!rstn) begin
        byte_1 <= 8'd0;
        byte_2 <= 8'd0;
    end
    else begin
        if(udp_rec_data_valid && byte_cnt == 12'd0)
            byte_1 <= udp_rec_rdata;
        else if(udp_rec_data_valid && byte_cnt == 12'd1)
            byte_2 <= udp_rec_rdata;
    end
end

// 状态机定义
localparam WAIT_HEADER  = 2'd0;
localparam PARSE_META   = 2'd1;
localparam EXTRACT_PIXELS= 2'd2;
localparam END_PACKET   = 2'd3;

reg [1:0] state/*synthesis PAP_MARK_DEBUG="1"*/, state_n;

always @(posedge rgmii_clk or negedge rstn) begin
    if (~rstn) state <= WAIT_HEADER;
    else       state <= state_n;
end

always @(*) begin
    case(state)
        WAIT_HEADER : begin
            if(byte_1 == 8'h55 && byte_2 == 8'hAA)
                state_n = PARSE_META;
            else
                state_n = WAIT_HEADER;
        end
        PARSE_META : begin
            if(byte_cnt == 12'd6)
                state_n = EXTRACT_PIXELS;
            else
                state_n = PARSE_META;
        end
        EXTRACT_PIXELS : begin
            if(!udp_rec_data_valid)
                state_n = END_PACKET;
            else
                state_n = EXTRACT_PIXELS;
        end
        END_PACKET : begin
            state_n = WAIT_HEADER;
        end
        default: state_n = WAIT_HEADER;
    endcase
end

// 解析包号 (稳固保留供调试，且不污染VSYNC脉冲)
reg [15:0] pkt_idx/*synthesis PAP_MARK_DEBUG="1"*/;
always @(posedge rgmii_clk or negedge rstn) begin
    if(!rstn)
        pkt_idx <= 16'hffff;
    else if(byte_cnt == 12'd5)
        pkt_idx[15:8] <= udp_rec_rdata;
    else if(byte_cnt == 12'd6)
        pkt_idx[7:0] <= udp_rec_rdata;
end

// 【核心修正】精确卡准 byte_cnt == 7 的这一拍产生一个周期的 VSYNC 脉冲
wire vs_raw = (byte_cnt == 12'd7) && (pkt_idx == 16'd0);
assign eth_video_vs = vs_raw;

// 【核心修正】利用 byte_cnt 的奇偶性，完美解决相差一拍的拼接错位问题
// 奇数(7,9,11)存高字节，偶数(8,10,12)输出完整16-bit
reg [7:0] high_byte;
always @(posedge rgmii_clk or negedge rstn) begin
    if(!rstn)
        high_byte <= 8'd0;
    else if(state == EXTRACT_PIXELS && byte_cnt[0] == 1'b1 && udp_rec_data_valid)
        high_byte <= udp_rec_rdata;
end

wire        de_raw;
wire [15:0] data_raw;
assign de_raw   = (state == EXTRACT_PIXELS) && (byte_cnt[0] == 1'b0) && udp_rec_data_valid;
assign data_raw = {high_byte, udp_rec_rdata};

/////////////////////////////////////////////////////////////////////////////////
// 260910 帧完整性兜底: 帧尾补黑
// 问题: 推流丢包(网络/缓冲)导致一帧内像素 < 921600 时, DDR 槽尾残留旧帧数据,
//       表现为"下半屏混着上一次推流的画面"。丢包量逐帧波动还会让缩放模块的
//       行边界漂移 -> 画面倾斜。
// 方案: 从帧起始脉冲(vsync)起统计像素数; 像素流空闲超过 1.6ms 判定本帧已结束,
//       若不足一帧则以黑色连续补齐到 921600 像素, 保证每个槽位每帧被完整重写。
/////////////////////////////////////////////////////////////////////////////////
localparam PIX_PER_FRAME = 20'd921600;   // 1280*720
localparam IDLE_PAD_TH   = 18'd37500;    // 0.3ms @125M: 大于帧内包间隔(<=12.3us),
                                         // 小于 60fps 帧间空隙(0.6ms) —— Sender 的
                                         // Sleep 按 16ms 整算, 实际空隙只有 0.6ms,
                                         // 阈值必须低于它, 否则 60fps 下补黑永不触发
reg  [19:0] pix_cnt;                     // 本帧已收像素数(饱和于 PIX_PER_FRAME)
reg  [17:0] idle_cnt;                    // 连续无像素拍数
wire        pad_active = (pix_cnt < PIX_PER_FRAME) && (idle_cnt >= IDLE_PAD_TH);

always @(posedge rgmii_clk or negedge rstn) begin
    if(!rstn) begin
        pix_cnt    <= 20'd0;
        idle_cnt   <= 18'd0;
        dbg_extract<= 1'b0;
    end
    else if(vs_raw) begin
        pix_cnt    <= 20'd0;                 // 新帧开始, 重新计数
        idle_cnt   <= 18'd0;
    end
    else begin
        if(pad_active)
            dbg_extract <= 1'b1;             // 发生过补黑 -> 锁存(F7)
        if(de_raw || pad_active) begin
            if(pix_cnt != PIX_PER_FRAME)
                pix_cnt <= pix_cnt + 20'd1;  // 像素到达或补黑中
            idle_cnt <= 18'd0;
        end
        else if(idle_cnt != 18'h3FFFF)
            idle_cnt <= idle_cnt + 18'd1;
    end
end

assign eth_video_de   = de_raw | pad_active;
assign eth_video_data = pad_active ? 16'h0000 : data_raw;

/////////////////////////////////////////////////////////////////////////////////
// 260910 包长校验(F8): 每个 UDP 包的载荷字节数应为 1471(满包)/31(末包)。
// 计数不符 = 字节滑移(物理层误码/包内气泡)或包被截断 —— 定位"红蓝紫色块"问题的直接证据。
/////////////////////////////////////////////////////////////////////////////////
reg [12:0] pkt_len_acc;
always @(posedge rgmii_clk or negedge rstn) begin
    if(!rstn) begin
        pkt_len_acc <= 13'd0;
        dbg_hdr55   <= 1'b0;
    end
    else if(udp_rec_data_valid) begin
        pkt_len_acc <= pkt_len_acc + 13'd1;
    end
    else begin
        if(pkt_len_acc != 13'd0 && pkt_len_acc != 13'd1471 && pkt_len_acc != 13'd31)
            dbg_hdr55 <= 1'b1;               // 包长异常 -> 锁存(F8)
        pkt_len_acc <= 13'd0;
    end
end

// 发送侧引脚强制拉低 (保证底层模块不主动发包)
wire ram_wr_en = 1'b0;
wire [7:0] ram_wr_data = 8'd0;
wire [15:0] udp_send_data_length = 16'd0;
wire udp_tx_req = 1'b0;
wire arp_request_req = 1'b0;

// 物理层延迟对齐
reg       gmii_rx_dv_1d;
reg [7:0] gmii_rxd_1d;
wire      gmii_tx_en_tmp;
wire [7:0]gmii_txd_tmp;

always@(posedge rgmii_clk or negedge rstn) begin
    if(rstn == 1'b0) begin
        gmii_rx_dv_1d <= 1'b0; 
        gmii_rxd_1d <= 8'd0;
    end else begin
        gmii_rx_dv_1d <= gmii_rx_dv; 
        gmii_rxd_1d <= gmii_rxd;
    end
end

always@(posedge rgmii_clk or negedge rstn) begin
    if(rstn == 1'b0) begin
        gmii_tx_en <= 1'b0; 
        gmii_txd <= 8'd0;
    end else begin
        gmii_tx_en <= gmii_tx_en_tmp; 
        gmii_txd <= gmii_txd_tmp;
    end
end

// 实例化底层以太网硬件协议栈
udp_ip_mac_top#(
    .LOCAL_MAC (LOCAL_MAC), .LOCAL_IP (LOCAL_IP), .LOCL_PORT (LOCL_PORT),
    .DEST_IP   (DEST_IP),   .DEST_PORT(DEST_PORT)
) udp_ip_mac_top (
    .rgmii_clk           (rgmii_clk),           .rstn                (rstn),
    .app_data_in_valid   (ram_wr_en),           .app_data_in         (ram_wr_data),
    .app_data_length     (udp_send_data_length),.app_data_request    (udp_tx_req),
    .udp_send_ack        (),                    .arp_req             (arp_request_req),
    .arp_found           (),                    .mac_not_exist       (),
    .mac_send_end        (),                    .udp_rec_rdata       (udp_rec_rdata),
    .udp_rec_data_length (udp_rec_data_length), .udp_rec_data_valid  (udp_rec_data_valid),
    .mac_data_valid      (gmii_tx_en_tmp),      .mac_tx_data         (gmii_txd_tmp),
    .rx_en               (gmii_rx_dv_1d),       .mac_rx_datain       (gmii_rxd_1d)
);

// (260909 锁存探针块已移除: dbg_hdr55/dbg_extract 现由上方 260910 新逻辑驱动)

endmodule