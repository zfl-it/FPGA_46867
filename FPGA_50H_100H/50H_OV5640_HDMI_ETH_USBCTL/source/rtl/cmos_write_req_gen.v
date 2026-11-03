module cmos_write_req_gen(
	input              rst,
	input              pclk,
	input              cmos_vsync,
	output reg         write_req,
	output reg[1:0]    write_addr_index,
	output reg[1:0]    read_addr_index,
	input              write_req_ack
);
parameter DROP_FRAMES = 120;

reg cmos_vsync_d0;
reg cmos_vsync_d1;
reg [7:0] frame_cnt;
reg       frame_valid;

always@(posedge pclk or posedge rst)
begin
	if(rst == 1'b1)
	begin
		cmos_vsync_d0 <= 1'b0;
		cmos_vsync_d1 <= 1'b0;
	end
	else
	begin
		cmos_vsync_d0 <= cmos_vsync;
		cmos_vsync_d1 <= cmos_vsync_d0;
	end
end

always@(posedge pclk or posedge rst)
begin
	if(rst == 1'b1) begin
		frame_cnt   <= 4'd0;
		frame_valid <= 1'b0;
	end else if(cmos_vsync_d0 == 1'b1 && cmos_vsync_d1 == 1'b0) begin
		if(frame_cnt < DROP_FRAMES)
			frame_cnt <= frame_cnt + 4'd1;
		else
			frame_valid <= 1'b1;
	end
end

always@(posedge pclk or posedge rst)
begin
	if(rst == 1'b1)
		write_req <= 1'b0;
	else if(cmos_vsync_d0 == 1'b1 && cmos_vsync_d1 == 1'b0 && frame_valid)
		write_req <= 1'b1;
	else if(write_req_ack == 1'b1)
		write_req <= 1'b0;
end

always@(posedge pclk or posedge rst)
begin
	if(rst == 1'b1)
		write_addr_index <= 2'b0;
	else if(cmos_vsync_d0 == 1'b1 && cmos_vsync_d1 == 1'b0 && frame_valid)
		write_addr_index <= write_addr_index + 2'd1;
end

always@(posedge pclk or posedge rst)
begin
	if(rst == 1'b1)
		read_addr_index <= 2'b0;
	else if(cmos_vsync_d0 == 1'b1 && cmos_vsync_d1 == 1'b0 && frame_valid)
		read_addr_index <= write_addr_index;
end
endmodule 