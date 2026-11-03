`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 2022/03/16 23:31:11
// Design Name: 
// Module Name: udp_layer
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module udp_layer #(
    parameter LOCAL_PORT_NUM = 16'hf000    //源端口号
) (
    input              udp_send_clk,      //时钟信号                                                                                                                                                                                                                                                                                                                                                                
    input              rstn,              //复位信号，低电平有效                                                                                                                                                                                                                                                                                                                                                    
                                                                                                                                                                                                                                                                                                                                                                                                                    
    //from software app                                                                                                                                                                                                                                                                                                                                                                                             
    input              app_data_in_valid, //本模块从外部所接收的数据输出有效信号，高电平有效                                                                                                                                                                                                                                                                                                                        
    input      [7:0]   app_data_in,       //本模块从外部所接收的数据输出                                                                                                                                                                                                                                                                                                                                            
    input      [15:0]  app_data_length,   //本模块从外部所接收的当前数据包的长度（不含udp、ip、mac 首部），单位：字节                                                                                                                                                                                                                                                                                               
    input      [15:0]  udp_dest_port,     //本模块从外部所接收的数据包的源端口号                                                                                                                                                                                                                                                                                                                                    
    input              app_data_request,  //用户接口数据发送请求，高电平有效                                                                                                                                                                                                                                                                                                                                        
                                                                                                                                                                                                                                                                                                                                                                                                                    
    output             udp_send_ready,    //握手是基于 ready、request、ack三个信号来实现的                                                                                                                                                                                                                                                                                                                          
    output             udp_send_ack,      //握手是基于 ready、request、ack三个信号来实现的                                                                                                                                                                                                                                                                                                                          
                                                                                                                                                                                                                                                                                                                                                                                                                    
    input              ip_send_ready,     //握手是基于 ready、request、ack三个信号来实现的                                                                                                                                                                                                                                                                                                                          
    input              ip_send_ack,       //握手是基于 ready、request、ack三个信号来实现的                                                                                                                                                                                                                                                                                                                          
    //to IP_send                                                                                                                                                                                                                                                                                                                                                                                                    
    output             udp_send_request,  //用户接口数据发送请求，高电平有效                                                                                                                                                                                                                                                                                                                                        
    output             udp_data_out_valid,//发送的数据输出有效信号，高电平有效                                                                                                                                                                                                                                                                                                                                      
    output     [7:0]   udp_data_out,      //发送的数据输出                                                                                                                                                                                                                                                                                                                                                          
    output     [15:0]  udp_packet_length, //当前数据包的长度（不含udp、ip、mac 首部），单位：字节  
    
    input      [7:0]   udp_rx_data,
    input              udp_rx_req,
    
    input              ip_checksum_error,
    input              ip_addr_check_error,
    
    output     [7:0]   udp_rec_rdata ,      //udp ram read data
    output     [15:0]  udp_rec_data_length, //udp data length
    output             udp_rec_data_valid   //udp data valid
);
    
    // 260909 精简: 板子从不开口发 UDP(app_data_request 在 eth_udp 中恒 0), 移除 udp_tx 模块
    // 发送侧握手输出全部封死: ready 常高(无请求时无影响), 其余拉低
    assign udp_send_ready    = 1'b1;
    assign udp_send_ack      = 1'b0;
    assign udp_send_request  = 1'b0;
    assign udp_data_out_valid= 1'b0;
    assign udp_data_out      = 8'h00;
    assign udp_packet_length = 16'd0;

    udp_rx #(
        .LOCAL_PORT            (  LOCAL_PORT_NUM      ) //parameter LOCAL_PORT = 16'hf000    //源端口号
    )udp_rx (
        .clk                   (  udp_send_clk        ),//input                  clk,   
        .rstn                  (  rstn                ),//input                  rstn,  
                                                      
        .udp_rx_data           (  udp_rx_data         ),//input      [7:0]       udp_rx_data,
        .udp_rx_req            (  udp_rx_req          ),//input                  udp_rx_req,
                                                      
        .ip_checksum_error     (  ip_checksum_error   ),//input                  ip_checksum_error,
        .ip_addr_check_error   (  ip_addr_check_error ),//input                  ip_addr_check_error,
                                                      
        .udp_rec_rdata         (  udp_rec_rdata       ),//output     [7:0]       udp_rec_rdata ,      //udp ram read data
        .udp_rec_data_length   (  udp_rec_data_length ),//output reg [15:0]      udp_rec_data_length,     //udp data length
        .udp_rec_data_valid    (  udp_rec_data_valid  ) //output reg             udp_rec_data_valid       //udp data valid
    );
    
endmodule
