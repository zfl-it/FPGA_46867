/*
	v1_0:视频流控制，确保打开采集的时候，是完整的一帧
*/
module	video_crtl
#(
    parameter   VIDEO_CLK_FREQ  =   148500000    , 
	parameter	PPC			    =	4		     ,
	parameter	DATA_WIDTH	    =	16		     ,
	parameter	IMG_WIDTH	    =	1920	     ,
	parameter	IMG_HEIGHT	    =	1080	
)
(
	input						i_video_clk					,
	input						i_rst_n						,
			
	//crtl port		
	input						i_start_dma_tx_flag			/*synthesis PAP_MARK_DEBUG="1"*/,
			
			
	input	[DATA_WIDTH-1:0]	i_video_data				/*synthesis PAP_MARK_DEBUG="1"*/,
	input						i_video_vs					/*synthesis PAP_MARK_DEBUG="1"*/,
	input						i_video_de					/*synthesis PAP_MARK_DEBUG="1"*/,
		
		
	output	[DATA_WIDTH-1:0]	o_video_crtl_data			/*synthesis PAP_MARK_DEBUG="1"*/,
	output						o_video_crtl_vs				/*synthesis PAP_MARK_DEBUG="1"*/,
	output						o_video_crtl_de		        /*synthesis PAP_MARK_DEBUG="1"*/

);
//************************************************************************************
//状态定义
parameter					IDLE		=	3'd0	;
parameter					WAIT		=	3'd1	;	//等待新的一帧
parameter					TX_DATA		=	3'd2	;	//传输数据
parameter					SP_IDLE		=	3'd4	;	//传输数据
parameter                   TIME_OUT    =   VIDEO_CLK_FREQ;
parameter                   IMG_SIZE    =   IMG_WIDTH*IMG_HEIGHT;
//************************************************************************************
reg	[3:0]					state				/*synthesis PAP_MARK_DEBUG="1"*/;	//控制开始采集
reg	[15:0]					r_x_cnt				/*synthesis PAP_MARK_DEBUG="1"*/;	//
reg	[15:0]					r_y_cnt				/*synthesis PAP_MARK_DEBUG="1"*/;	//
reg							r_video_vs_d0		;
reg							r_video_vs_d1		;
reg	[DATA_WIDTH-1:0]		r_video_crtl_data	;	//
reg							r_video_crtl_vs		;
reg							r_video_crtl_de		;
reg [31:0]                  r_dly_cnt           /*synthesis PAP_MARK_DEBUG="1"*/;   //超时计数 每秒检测vs信号是否存在 不存在就回退
reg [31:0]                  r_img_total_cnt     /*synthesis PAP_MARK_DEBUG="1"*/;
wire                        r_clear             ;    //复位信号
always@(posedge i_video_clk)	begin
    if(!i_rst_n)
        r_dly_cnt    <=    'd0    ;
    else    if(i_video_vs && ~r_video_vs_d0 || i_video_de == 1)
        r_dly_cnt    <=    'd0    ;
    else    if(r_dly_cnt >= TIME_OUT)
        r_dly_cnt    <=    r_dly_cnt    ;
    else
        r_dly_cnt    <=    r_dly_cnt + 1'b1;
end

//异步复位
assign r_clear = (r_dly_cnt >= TIME_OUT) ? 1'b1 : 1'b0;

always@(posedge i_video_clk)	begin

	r_video_vs_d0	<=	i_video_vs		;
	r_video_vs_d1	<=	r_video_vs_d0	;

end


//横坐标计数
always@(posedge i_video_clk)	begin
	if(!i_rst_n)
		r_x_cnt		<=	'd0	;
	else	if(i_video_vs && ~r_video_vs_d0)
		r_x_cnt		<=	'd0	;
	else	if(i_video_de && r_x_cnt == IMG_WIDTH/PPC-1)
		r_x_cnt		<=	'd0	;
	else	if(i_video_de)
		r_x_cnt		<=	r_x_cnt + 1'b1;
	else
		r_x_cnt		<=	r_x_cnt;
end

//纵坐标计数
always@(posedge i_video_clk)	begin
	if(!i_rst_n)
		r_y_cnt		<=	'd0	;
	else	if(i_video_vs && ~r_video_vs_d0)
		r_y_cnt		<=	'd0	;
	else	if(i_video_de && r_x_cnt == IMG_WIDTH/PPC-1 && r_y_cnt == IMG_HEIGHT-1)
		r_y_cnt		<=	'd0	;
	else	if(i_video_de && r_x_cnt == IMG_WIDTH/PPC-1)
		r_y_cnt		<=	r_y_cnt + 1'b1;
	else
		r_y_cnt		<=	r_y_cnt;
end

         
//控制数据发送
always@(posedge i_video_clk)	begin
	if(!i_rst_n)    begin
		state	        <=	IDLE	;
        r_img_total_cnt <=  'd0     ;  
    end
	else	begin
		case(state)
			IDLE:		begin
							if(i_start_dma_tx_flag)
								state	<=	WAIT;
							else
								state	<=	IDLE;
						end
						
			WAIT:		begin
							if(i_video_vs && ~r_video_vs_d0)	//新的一帧到来
								state	<=	TX_DATA;
							else
								state	<=	WAIT	;
						end
					
			TX_DATA	:	begin
							if(!i_start_dma_tx_flag)    begin
								state	        <=	IDLE	;  
                            end
							else    begin
								state	        <=	TX_DATA	;
                            end   
						end

			
			default	:	state	<=	IDLE	;
		endcase
	end
end






//**************************输出***********************************
always@(posedge i_video_clk)	begin
	if(!i_rst_n)	begin
		r_video_crtl_data	<=	'd0	;
		r_video_crtl_vs		<=	'd0	;
		r_video_crtl_de		<=	'd0	;
	end
	else	if(state == TX_DATA)	begin
		r_video_crtl_data	<=	i_video_data	;				
	    r_video_crtl_vs		<=  i_video_vs		;				
	    r_video_crtl_de		<=  i_video_de		;				
	end
	else	begin
		r_video_crtl_data	<=	'd0	;
		r_video_crtl_vs		<=	'd0	;
		r_video_crtl_de		<=	'd0	;
	end	
end


assign	o_video_crtl_data	=	r_video_crtl_data	;	
assign  o_video_crtl_vs		=	r_video_crtl_vs		;	
assign  o_video_crtl_de		=	r_video_crtl_de		;

endmodule

