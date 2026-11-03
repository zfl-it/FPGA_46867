`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:Meyesemi 
// Engineer: Will
// 
// Create Date: 2023-01-29 20:31  
// Design Name:  
// Module Name: 
// Project Name: 
// Target Devices: Pango
// Tool Versions: 
// Description: 
//      
// Dependencies: 
// 
// Revision:
// Revision 1.0 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

`define UD #1

module uart_rx # (
    parameter            BPS_NUM     =    16'd434
//  设置波特率为4800时，  bit位宽时钟周期个数:50MHz set 10417  40MHz set 8333
//  设置波特率为9600时，  bit位宽时钟周期个数:50MHz set 5208   40MHz set 4167
//  设置波特率为115200时，bit位宽时钟周期个数:50MHz set 434    40MHz set 347
)
(
      //input ports
      input             clk,
      input             uart_rx,
    
      //output ports
      output reg [7:0]  rx_data,
      output reg        rx_en,
      output            rx_finish
);

    // uart rx state machine's state
    localparam  IDLE         = 4'h0;    //空闲状态，等待开始信号到来.
    localparam  RECEIV_START = 4'h1;    //接收Uart开始信号，低电平一个波特周期.
    localparam  RECEIV_DATA  = 4'h2;    //接收Uart传输数据信号，此工程定义传输8bit，每个波特周期中间位置取值，8个周期后跳转到stop状态.
    localparam  RECEIV_STOP  = 4'h3;    //停止状态数据线是高电平，与空闲状态是一致的按照协议标准需要等待一个停止位周期再做状态跳转.
    localparam  RECEIV_END   = 4'h4;    //结束中转状态.

    // 261013 fix: 停止位提前退出点 = 3/4 位处（434 - 108 = 326）。
    // 原版 STOP 要计满整整一位（到 BPS_NUM）才进 END，状态机每字节固定
    // 消耗 10*435=4350 拍，而线上 115200bps 字节只有 10*434.03=4340 拍；
    // 背靠背连发时 RECEIV_END 直跳 RECEIV_START、从不回 IDLE 重同步，
    // 每字节滞后 ~9.7 拍持续累积，约 20 个连续字节后停止位中点采样
    // （帧起点+4146 拍）越过真实停止位边界（4340）落到下一字节起始位
    // （低电平）→ 帧被丢弃、后续数据位全面错位 —— 表现为"串口连发
    // 几十个字符后彻底失控"。提前到 3/4 位退出后，状态机每字节只需
    // ~4245 拍（< 4340），字节间必经 IDLE 重新同步，滞后永不累积。

    //==========================================================================
    //wire and reg in the module
    //==========================================================================
    reg    [2:0]        rx_state=0;       //current state of tx state machine. 当前状态
    reg    [2:0]        rx_state_n=0;     //next state of tx state machine.    下一个状态
    reg    [7:0]        rx_data_reg;      //                                   接收数据缓冲寄存器
    reg                 uart_rx_1d;       //save uart_rx one cycle.            保存uart_rx一个时钟周期
    reg                 uart_rx_2d;       //save uart_rx one cycle.保存uart_rx 前两个时钟周期
    wire                start;            //active when start a byte receive.  检测到start信号标志
    reg    [15:0]       clk_div_cnt;      //count for division the clock.      波特周期计数器

    // 261013 fix: 停止位 3/4 处退出计数点（对 BPS_NUM 参数化，434 -> 326）
    localparam [15:0]   STOP_EXIT_CNT = BPS_NUM - (BPS_NUM >> 2);

    // 261013c: 帧级看门狗。
    // 板级实测：串口失灵后按键仍正常、必须 FPGA 断电重上电才恢复 ——
    // 唯一自洽解释是 FSM 被"线持续为低"（CH340/驱动异常发出 break、
    // 线态异常）卡进 START->DATA->STOP->END->START 死循环，永远回不到
    // IDLE，此后所有真实字节全部错过。合法帧最长 ~11 个位周期
    // （起始 1 + 数据 8 + 停止 + 余量），这里取 32 个位周期（约 278us
    // @50MHz，3 倍裕量）为上限：任何非 IDLE 状态连续停留超时即强制回
    // IDLE，接收器最长 278us 自动脱困，之后照常接收，无需重新上电。
    // 正常收流（含背靠背）每字节必经 IDLE（261013 fix），计数器每字节
    // 清零，永不误触发。
    localparam [15:0]   WD_LIMIT = BPS_NUM * 16'd32;
    reg  [15:0]         wd_cnt;
    wire                wd_force = (wd_cnt == WD_LIMIT);

    always @ (posedge clk)
    begin
        if (rx_state == IDLE || wd_force)
            wd_cnt <= `UD 16'd0;
        else
            wd_cnt <= `UD wd_cnt + 16'd1;
    end

    //==========================================================================
    //logic
    //==========================================================================
    
    //some control single.
    always @ (posedge clk) 
    begin
         uart_rx_1d <= `UD uart_rx;
         uart_rx_2d <= `UD  uart_rx_1d;
    end

    // 261013b: 起始检测改用同步后的两拍样本。
    // 原式 (!uart_rx)&&(1d||2d) 把异步引脚直接引进状态转移组合逻辑：
    //   1) 线上任何 1 拍毛刺（本板 DDR3/HDMI/千兆网 EMI 环境）都会触发
    //      一个 ~87us 幻帧 —— 轻则占用接收器、紧随其后的真实字节因
    //      FSM 不在 IDLE/END 窗口而整帧丢失；重则幻帧采样出 0x00~0x04
    //      （0x00 = 合法切视图命令）造成"自己动"的假失控；
    //   2) 异步沿直入 rx_state_n，rx_state 存在采样亚稳态风险。
    // 改为要求连续两拍同步样本同为低：1 拍毛刺被天然滤除，真实起始沿
    // 仅晚 ~2 拍检测；261013 fix 已在 START 入口强制清零 cnt，位采样
    // 对齐不受影响。断流/背靠背行为不变。
    assign start = (!uart_rx_1d) && (!uart_rx_2d);
    assign rx_finish = (rx_state == RECEIV_END);


    //division the clock to satisfy baud rate.波特周期计数器
    // 261013 fix: RECEIV_END 直跳 RECEIV_START（背靠背/断流恢复）时强制
    // 清零 —— 原版此时 cnt 从 ~327 起步，START 只剩 ~107 拍，数据位
    // 采样完全错位。清零后 START 从真实起始沿重新计满一整位。
    always @ (posedge clk)
    begin
        if(rx_state == IDLE
           || (rx_state == RECEIV_END && rx_state_n == RECEIV_START)
           || clk_div_cnt == BPS_NUM)
            clk_div_cnt   <= `UD 16'h0;
        else
            clk_div_cnt   <= `UD clk_div_cnt + 16'h1;
    end
    
    // receive bit data numbers 
    //在接收数据状态中，接收的bit位计数，每一个波特周期计数加1
    reg    [2:0]      rx_bit_cnt=0;    //the bits number has transmited.
    always @ (posedge clk)
    begin
        if(rx_state == IDLE)
            rx_bit_cnt <= `UD 3'h0;
        else if((rx_bit_cnt == 3'h7) && (clk_div_cnt == BPS_NUM))
            rx_bit_cnt <= `UD 3'h0;
        else if((rx_state == RECEIV_DATA) && (clk_div_cnt == BPS_NUM))
            rx_bit_cnt <= `UD rx_bit_cnt + 3'h1;
        else 
            rx_bit_cnt <= `UD rx_bit_cnt;
    end

//==========================================================================
//receive state machine
//==========================================================================
    //状态机状态跳转
    always @(posedge clk)
    begin
        rx_state <= rx_state_n;
    end
    
    //状态机状态跳转条件及跳转规律
    always @ (*)
    begin
      case(rx_state)
          IDLE       :  
          begin
              if(start)                                     //监测到start信号到来，下一状态跳转到start状态
                  rx_state_n = RECEIV_START;
              else
                  rx_state_n = rx_state;
          end
          RECEIV_START    :  
          begin
              if(clk_div_cnt == BPS_NUM)                     //已完成接收start标志信号
                  rx_state_n = RECEIV_DATA;
              else
                  rx_state_n = rx_state;
          end
          RECEIV_DATA    :  
          begin
              if(rx_bit_cnt == 3'h7 && clk_div_cnt == BPS_NUM) //已完成8bit数据的传输
                  rx_state_n = RECEIV_STOP;
              else
                  rx_state_n = rx_state;
          end
          RECEIV_STOP    :
          begin
              if(clk_div_cnt == STOP_EXIT_CNT)                 // 261013 fix: 停止位 3/4 处即结束本帧，
                  rx_state_n = RECEIV_END;                     // 让字节间必经 IDLE 重同步（根治连发失步）
              else
                  rx_state_n = rx_state;
          end
          RECEIV_END    :  
          begin
              if(!uart_rx_1d)                                  //数据线重新被拉低，表示新数据传输又发送start标志信号，需要跳转到start状态
                  rx_state_n = RECEIV_START;
              else                                             //没有其他状况出现时，回到空闲状态，等待start信号的到来
                  rx_state_n = IDLE;
          end
          default    :  rx_state_n = IDLE;
      endcase
      // 261013c: 看门狗超时强制回 IDLE（覆盖上表的任何跳转决策）。
      // wd_cnt 在 IDLE 恒为 0，故 wd_force=1 时 rx_state 必然非 IDLE。
      if (wd_force)
          rx_state_n = IDLE;
    end
    
    // 状态机输出
    always @ (posedge clk)
    begin
        case(rx_state)
            IDLE         ,
            RECEIV_START :                               //在空闲和start状态时将接收数据缓冲寄存器和数据使能置位；
            begin
                rx_en <= `UD 1'b0;
                rx_data_reg <= `UD 8'h0;
            end
            RECEIV_DATA  :  
            begin
                if(clk_div_cnt == BPS_NUM[15:1])        //在一个波特周期的中间位置取数据线上传输的数据；
                    rx_data_reg  <= `UD {uart_rx , rx_data_reg[7:1]};  //以循环右移的方式将uart_rx数据填入缓冲寄存器的最高位（Uart传输低位在前，最后一个bit刚好是最高位）
            end
            RECEIV_STOP  : 
            begin
                // 261012c: 停止位中点校验 —— 停止位必须为高, 否则视为
                // 毛刺/break/帧错误, 不置 rx_en (解码器看不到该字节)。
                // 修复: R8 线被拉低(USB挂起抖动/驱动切换/EMI)时, 原版每 87us
                // 吐一个幻影 0x00 —— 0x00 恰是合法的视图切换命令, 导致长时
                // 连接后视图被狂切、控制"失灵"。
                if (clk_div_cnt == BPS_NUM[15:1])
                begin
                    if (uart_rx_1d)
                    begin
                        rx_en   <= `UD 1'b1;                // 输出使能信号，表示最新的数据输出有效
                        rx_data <= `UD rx_data_reg;         // 将缓冲寄存器的值赋值给输出寄存器
                    end
                end
            end
            RECEIV_END    :  
            begin
                rx_data_reg <= `UD 8'h0;
            end
            default:    rx_en <= `UD 1'b0;
        endcase
    end

endmodule




