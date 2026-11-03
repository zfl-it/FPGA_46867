//彩色屏幕驱动v1.0
//支持各种分辨率
//需要去video_define.v中修改分辨率
//支持RGB565和RGB888
`include "video_define.v"
module screen_test 
#(
    `ifdef     RGB888
    parameter PIX_WIDTH = 24    ,
    `elsif     RGB565
    parameter PIX_WIDTH = 16    ,
    `endif
    parameter PPC       = 4     ,  
    parameter BPC       = 8     ,  
    parameter H_TOTAL   = 1920  ,
    parameter V_TOTAL   = 1080
)
(
    input   wire                    video_clk        ,       //像素时钟
    input   wire                    rst_n            ,       //复位信号  
     
    //输出端口               
    output  wire                    hsync            ,       //行同步信号
    output  wire                    vsync            ,       //场同步信号
    output  wire                    de               ,       //数据有效信号
    output  wire  [PIX_WIDTH-1:0]   pixel_data               //像素数据
);

//生成时序
wire    color_vs    ;       //场同步信号
wire    color_hs    ;       //行同步信号
wire    color_de    ;       //数据有效信号
color_bar 
#(
    .PPC    (PPC)
)u_color_bar(
    .clk   ( video_clk   ),
    .rst   ( ~rst_n      ) ,
    //user interface
    .hs    ( color_hs    ),
    .vs    ( color_vs    ),
    .de    ( color_de    ),
    .rgb_r (  ),
    .rgb_g (  ),
    .rgb_b (   )
);

wire            o_hs        ;       //行同步信号
wire            o_vs        ;       //场同步信号
wire            o_de        ;       //数据有效信号
wire    [11:0]  x_pos       ;       //行坐标
wire    [11:0]  y_pos       ;       //场坐标
//生成坐标
timing_gen_xy#(
    .DATA_WIDTH   ( PIX_WIDTH )
)u_timing_gen_xy(
    .rst_n        ( rst_n            ),
    .clk          ( video_clk        ),
    .i_hs         ( color_hs         ),
    .i_vs         ( color_vs         ),
    .i_de         ( color_de         ),     
    .i_data       (        ),
    .o_hs         ( o_hs             ),
    .o_vs         ( o_vs             ),
    .o_de         ( o_de             ),
    .o_data       (      ),
    .x            ( x_pos            ),
    .y            ( y_pos            )
);

//生成对应的数据
wire    hdmi_vs;
wire    hdmi_hs;
wire    hdmi_de;
wire    [23:0]hdmi_rgb;
vga_disp#(
    .H_TOTAL  ( H_TOTAL ),
    .V_TOTAL  ( V_TOTAL )
)u_vga_disp(
    .vga_clk  ( video_clk  ),
    .rst_n    ( rst_n    ),
    .vga_hs   ( o_hs   ),
    .vga_vs   ( o_vs   ),
    .vga_de   ( o_de   ),
    .x_pos    ( x_pos    ),
    .y_pos    ( y_pos    ),
    .hdmi_vs  ( hdmi_vs  ),
    .hdmi_hs  ( hdmi_hs  ),
    .hdmi_de  ( hdmi_de  ),
    .data_gen ( hdmi_rgb  )
);

assign  hsync           =    hdmi_hs;
assign  vsync           =    hdmi_vs;
assign  de              =    hdmi_de;



`ifdef RGB888
assign  pixel_data      =    hdmi_rgb;
`elsif RGB565
assign  pixel_data      =    {hdmi_rgb[23:19],hdmi_rgb[15:10],hdmi_rgb[7:3]};
`endif

endmodule
/*
screen_test#(
    .PIX_WIDTH  ( 24 ),
    .H_TOTAL    ( 1920 ),
    .V_TOTAL    ( 1080 )
)u_screen_test(
    .video_clk  ( video_clk  ),
    .rst_n      ( rst_n      ),
    .hsync      ( hsync      ),
    .vsync      ( vsync      ),
    .de         ( de         ),
    .pixel_data  ( pixel_data  )
);


*/