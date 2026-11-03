//按键消抖 · 低电平按下稳定 20ms 后输出 1 个 clk 的脉冲（原中文注释已被编码事故损坏，以下按代码实际行为重写）
`define UD #1
module btn_deb_fix#(
    parameter                  BTN_WIDTH = 32'd8	//按键位宽
)
(
    input                      sys_clk	,  
    input  wire                rst_n    ,
    input      [BTN_WIDTH-1:0] btn_in	,
    
    output reg [BTN_WIDTH-1:0] btn_deb_fix
);

//----------------------------------parameter----------------------------------------
parameter	CNT_20MS_MAX	=	32'd1_000_000	;	//20MS 的计数值上限

//----------------------------------reg----------------------------------------------
reg [23:0]          cnt[BTN_WIDTH-1:0];    //每路按键各一个计数器
reg [BTN_WIDTH-1:0] btn_in_reg        ;    //输入电平同步打一拍
//打一拍
always @(posedge sys_clk)    begin
	btn_in_reg <=  btn_in;
end 
//----------------------------------消抖主逻辑----------------------------------------------
genvar i;
generate
begin
    for(i=0;i<BTN_WIDTH;i=i+1)
    begin
        always @(posedge sys_clk)    begin
            if(!rst_n)
                cnt[i] <=  24'd0;     
        	if (btn_in_reg[i] == 1'b0)       //按下（低电平）时计数器清零
        		cnt[i] <=  24'd0;
        	else if(cnt[i]==CNT_20MS_MAX)    //计满则保持，不再累加
        		cnt[i] <=  cnt[i];
        	else                      
        		cnt[i] <=  cnt[i] + 1'b1;
        end 

        always @(posedge sys_clk)    begin
            if(!rst_n)
                btn_deb_fix[i] <=  1'd0; 
        	else if(cnt[i]==CNT_20MS_MAX-1)   //计满前一个 clk 输出高电平（单周期脉冲）
        		btn_deb_fix[i] <=  1'b1;
        	else                        
        		btn_deb_fix[i] <=  1'b0;
        end 
    end
end
endgenerate

endmodule
