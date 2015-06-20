`timescale 1ns / 1ps

module delay #(parameter dw=8, lat=32)
	(
	input 				clk,
	input 				stall,
	input [dw-1:0]		din,
	output [dw-1:0] 	dout );
	generate
		if(lat==0) begin : du_0
			assign dout = din;
		end
		else begin : du_pipe
			reg [dw-1:0] D [0:lat-1];
			integer i;
			always @(posedge clk) begin
				if(!stall) begin
					D[0] <= din;
					for(i=1; i<lat; i=i+1) begin
						D[i] <= D[i-1];
					end
				end
			end
			assign dout = D[lat-1];
		end
	endgenerate
endmodule

module similarity_coefficients_pipe # (
	parameter WIDTH = 32,
				 OWIDTH = 32
)
(
    input clk,
    input vld_in,
    input [WIDTH-1:0] a,
    input [WIDTH-1:0] b,
    input [WIDTH-1:0] c,
    output vld_out,
    output [OWIDTH-1:0] Tanimoto_coefficient//,
//    output [OWIDTH-1:0] Dice,
 //   output [OWIDTH-1:0] Cosine,
 //   output [OWIDTH-1:0] Euclidean,
 //   output [OWIDTH-1:0] Manhattan_Euclidean
);

wire [WIDTH-1:0] a_t;
wire [WIDTH-1:0] b_t;
wire [WIDTH-1:0] c_t;

wire [WIDTH-1:0] a_plus_b;
wire [WIDTH-1:0] minus_c;

wire [WIDTH-1:0] a_plus_b_1;
wire [WIDTH-1:0] minus_c_1;

wire [WIDTH-1:0] a_plus_b_minus_c_1;
wire [WIDTH-1:0] a_plus_b_minus_c_2;
wire [WIDTH-1:0] a_plus_b_minus_c_2_fp;
wire [WIDTH-1:0] c_1;
wire [WIDTH-1:0] c_2;
wire [WIDTH-1:0] c_2_fp;

wire vld_in_t;
wire vld_in_1;
wire vld_in_2;
wire vld_in_3;

// Stage 0 Logic
assign a_t = a;
assign b_t = b;
assign c_t = c;
assign vld_in_t = vld_in;

assign a_plus_b = a_t + b_t;
assign minus_c[WIDTH-1:0] = -c_t[WIDTH-1:0];


// Stage 0 Registers
delay #(WIDTH, 1) du_0_a_plus_b (.clk(clk), .stall(1'b0), .din(a_plus_b), .dout(a_plus_b_1));
delay #(WIDTH, 1) du_0_minus_c_1c (.clk(clk), .stall(1'b0), .din(minus_c), .dout(minus_c_1));
delay #(WIDTH, 1) du_0_c_1c (.clk(clk), .stall(1'b0), .din(c_t), .dout(c_1));
delay #(1, 1) du_0_vld_in_1c (.clk(clk), .stall(1'b0), .din(vld_in_t), .dout(vld_in_1));

// Stage 1 Logic
assign a_plus_b_minus_c_1 = a_plus_b_1 + minus_c_1;

// Stage 1 Registers
delay #(WIDTH, 1) du_1_c_1_1c (.clk(clk), .stall(1'b0), .din(c_1), .dout(c_2));
delay #(WIDTH, 1) du_1_a_plus_b_minus_c_1_1c (.clk(clk), .stall(1'b0), .din(a_plus_b_minus_c_1), .dout(a_plus_b_minus_c_2));
delay #(1, 1) du_0_vld_in_1_1c (.clk(clk), .stall(1'b0), .din(vld_in_1), .dout(vld_in_2));

// Convert to FP
int32fp int32fp_A (.operation_nd(vld_in_2), .clk(clk), .rdy(vld_in_3), .a(a_plus_b_minus_c_2), .result(a_plus_b_minus_c_2_fp));
int32fp int32fp_B (.operation_nd(vld_in_2), .clk(clk), .rdy(), .a(c_2), .result(c_2_fp));

// Stage 3 Logic
fpdiv32 Tanimoto_div ( .operation_nd (vld_in_3), .clk(clk), .underflow(), .overflow(), .divide_by_zero(), .rdy(vld_out), .a(c_2_fp), .b(a_plus_b_minus_c_2_fp), .result(Tanimoto_coefficient));



endmodule
