//生成vga需要显示的数据
module vga_disp
#(
    parameter   H_TOTAL     =   800    ,//显示区域宽度
    parameter   V_TOTAL     =   600     //显示区域高度
)
(
    input   wire                    vga_clk             ,//
    input   wire                    rst_n               ,//active low
    input   wire                    vga_hs              ,//horizontal sync
    input   wire                    vga_vs              ,//vertical sync
    input   wire                    vga_de              ,//data enable

    input   wire    [11:0]          x_pos               ,
    input   wire    [11:0]          y_pos               ,

    output  reg                     hdmi_vs             ,
    output  reg                     hdmi_hs             ,
    output  reg                     hdmi_de             ,
    output  reg     [23:0]          data_gen             
);
//parameter define
parameter RED = 24'HFF0000;//红
parameter ORANGE = 24'HFF7700;//橙
parameter YELLOW = 24'HFFFF00;//黄
parameter GREEN = 24'H00FF00;//绿
parameter CYAN = 24'H00FFFF;//青
parameter BLUE = 24'H0000FF;//蓝
parameter PURPLE = 24'HFF00FF;//紫
parameter BLACK = 24'H000000;//黑
parameter WHITE = 24'HFFFFFF;//白
parameter GRAY = 24'H666666;//灰

parameter NUM_W  = 15   ;//国际上使用的五子棋棋盘都是15*15

always@(posedge vga_clk or negedge rst_n) begin
    if(!rst_n) 
    begin
        hdmi_hs <= 1'b0;
        hdmi_vs <= 1'b0;
        hdmi_de <= 1'b0;
    end
    else 
    begin
        hdmi_hs <= vga_hs;
        hdmi_vs <= vga_vs;
        hdmi_de <= vga_de;
    end
    
end

wire    vs_pose ;
assign vs_pose = vga_vs && ~hdmi_vs;


reg [7:0]   vs_cnt;
//每一帧切换一次图像
always@(posedge vga_clk or negedge rst_n) begin
    if(!rst_n)
        vs_cnt <= 8'd0;
    else if(vs_pose && vs_cnt == 8'd59)
        vs_cnt <= 8'd0;
    else if(vs_pose)
        vs_cnt <= vs_cnt + 1'b1;
    else
        vs_cnt <= vs_cnt;
end

reg [3:0]  choose_image;
always@(posedge vga_clk or negedge rst_n) begin
    if(!rst_n)
        choose_image <= 3'd0;
    else if(vs_pose && vs_cnt == 8'd59 && choose_image == 3'd4) 
        choose_image <= 3'd0;
    else if(vs_pose && vs_cnt == 8'd59)
        choose_image <= choose_image + 1'b1;
    else
        choose_image <= choose_image;
end



//wire or reg define
reg     [23:0]          data_h               ;//横彩条
reg     [23:0]          data_v               ;//竖彩条
reg     [23:0]          data_h_j             ;//井字棋
reg     [23:0]          data_v_j             ;


//main code
//组合逻辑实现不同图像数据的选择
always @ (*) begin
    case(choose_image)
        3'b000:data_gen =   data_h  ;//横彩条
        3'b001:data_gen =   data_v  ;//竖彩条
        3'b010:data_gen =   (data_h^data_v);//横彩条和竖彩条按位异或产生一种棋盘格
        3'b011:data_gen =   (data_h^~data_v);//横彩条和竖彩条按位同或产生一种棋盘格
        3'b100:data_gen =   (data_h_j & data_v_j);//产生井字棋棋盘
        default:data_gen=   data_h  ;//默认显示横彩条
    endcase
end

//data_h:横彩条
always @ (posedge vga_clk or negedge rst_n) begin
    if(!rst_n)
        data_h<=24'd0;
    else if(y_pos<V_TOTAL/10)
        data_h<=RED;
    else if((y_pos>=V_TOTAL/10)&&(y_pos<V_TOTAL*2/10))
        data_h<=ORANGE;
    else if((y_pos>=V_TOTAL*2/10)&&(y_pos<V_TOTAL*3/10))
        data_h<=YELLOW;
    else if((y_pos>=V_TOTAL*3/10)&&(y_pos<V_TOTAL*4/10))
        data_h<=GREEN;
    else if((y_pos>=V_TOTAL*4/10)&&(y_pos<V_TOTAL*5/10))
        data_h<=CYAN;     
    else if((y_pos>=V_TOTAL*5/10)&&(y_pos<V_TOTAL*6/10))
        data_h<=BLUE;     
    else if((y_pos>=V_TOTAL*6/10)&&(y_pos<V_TOTAL*7/10))
        data_h<=PURPLE;     
    else if((y_pos>=V_TOTAL*7/10)&&(y_pos<V_TOTAL*8/10))
        data_h<=BLACK;
    else if((y_pos>=V_TOTAL*8/10)&&(y_pos<V_TOTAL*9/10))
        data_h<=WHITE;     
    else if((y_pos>=V_TOTAL*9/10)&&(y_pos<V_TOTAL))
        data_h<=GRAY;              
end

//data_v:竖彩条
always @ (posedge vga_clk or negedge rst_n) begin
    if(!rst_n)
        data_v<=24'd0;
    else if(x_pos<H_TOTAL/10)
        data_v<=RED;
    else if((x_pos>=H_TOTAL/10)&&(x_pos<H_TOTAL*2/10))
        data_v<=ORANGE;
    else if((x_pos>=H_TOTAL*2/10)&&(x_pos<H_TOTAL*3/10))
        data_v<=YELLOW;
    else if((x_pos>=H_TOTAL*3/10)&&(x_pos<H_TOTAL*4/10))
        data_v<=GREEN;
    else if((x_pos>=H_TOTAL*4/10)&&(x_pos<H_TOTAL*5/10))
        data_v<=CYAN;     
    else if((x_pos>=H_TOTAL*5/10)&&(x_pos<H_TOTAL*6/10))
        data_v<=BLUE;     
    else if((x_pos>=H_TOTAL*6/10)&&(x_pos<H_TOTAL*7/10))
        data_v<=PURPLE;     
    else if((x_pos>=H_TOTAL*7/10)&&(x_pos<H_TOTAL*8/10))
        data_v<=BLACK;
    else if((x_pos>=H_TOTAL*8/10)&&(x_pos<H_TOTAL*9/10))
        data_v<=WHITE;     
    else if((x_pos>=H_TOTAL*9/10)&&(x_pos<H_TOTAL))
        data_v<=GRAY;              
end

//data_h_j:产生井字棋棋盘白底黑边-横条
always @ (posedge vga_clk or negedge rst_n) begin
    if(!rst_n)
        data_h_j<=24'd0;
    else if(y_pos==V_TOTAL/3 || y_pos==V_TOTAL/3*2)//和竖的条纹是与逻辑的关系
        data_h_j<=BLACK;//黑色的横条纹
    else
        data_h_j<=WHITE;//白色的背景
end
//data_v_j:产生井字棋棋盘白底黑边-竖条
always @ (posedge vga_clk or negedge rst_n) begin
    if(!rst_n)
        data_v_j<=12'd0;
    else if(x_pos==H_TOTAL/3 || x_pos==H_TOTAL/3*2)//和竖的条纹是与逻辑的关系
        data_v_j<=BLACK;//黑色的横条纹
    else
        data_v_j<=WHITE;//白色的背景
end


endmodule