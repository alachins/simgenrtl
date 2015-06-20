	
module du #(parameter dw=8, lat=32)	
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


	
module du_s #(parameter dw=8)	
	(	
	input 				clk,	
	input 				stall,	
	input [dw-1:0]		din,	
	output [dw-1:0] 	dout );	
	reg [dw-1:0] dout_t;	
	wire [dw-1:0] dout_t_w;	
	always @(posedge clk) begin	
		dout_t = din;	
	end	
	assign dout_t_w = dout_t;	
	assign dout = dout_t_w;	
endmodule	


module lut8_BRAM (
  input clk,
  input [7:0]lut8_BRAM_i_0,
  input [7:0]lut8_BRAM_i_1,
  output reg [3:0]lut8_BRAM_o_0,
  output reg [3:0]lut8_BRAM_o_1
);

 (* rom_style = "block" *)
 reg [3:0] lut8w[255:0];

 initial begin
  lut8w[0] = 4'b0000;
  lut8w[1] = 4'b0001;
  lut8w[2] = 4'b0001;
  lut8w[3] = 4'b0010;
  lut8w[4] = 4'b0001;
  lut8w[5] = 4'b0010;
  lut8w[6] = 4'b0010;
  lut8w[7] = 4'b0011;
  lut8w[8] = 4'b0001;
  lut8w[9] = 4'b0010;
  lut8w[10] = 4'b0010;
  lut8w[11] = 4'b0011;
  lut8w[12] = 4'b0010;
  lut8w[13] = 4'b0011;
  lut8w[14] = 4'b0011;
  lut8w[15] = 4'b0100;
  lut8w[16] = 4'b0001;
  lut8w[17] = 4'b0010;
  lut8w[18] = 4'b0010;
  lut8w[19] = 4'b0011;
  lut8w[20] = 4'b0010;
  lut8w[21] = 4'b0011;
  lut8w[22] = 4'b0011;
  lut8w[23] = 4'b0100;
  lut8w[24] = 4'b0010;
  lut8w[25] = 4'b0011;
  lut8w[26] = 4'b0011;
  lut8w[27] = 4'b0100;
  lut8w[28] = 4'b0011;
  lut8w[29] = 4'b0100;
  lut8w[30] = 4'b0100;
  lut8w[31] = 4'b0101;
  lut8w[32] = 4'b0001;
  lut8w[33] = 4'b0010;
  lut8w[34] = 4'b0010;
  lut8w[35] = 4'b0011;
  lut8w[36] = 4'b0010;
  lut8w[37] = 4'b0011;
  lut8w[38] = 4'b0011;
  lut8w[39] = 4'b0100;
  lut8w[40] = 4'b0010;
  lut8w[41] = 4'b0011;
  lut8w[42] = 4'b0011;
  lut8w[43] = 4'b0100;
  lut8w[44] = 4'b0011;
  lut8w[45] = 4'b0100;
  lut8w[46] = 4'b0100;
  lut8w[47] = 4'b0101;
  lut8w[48] = 4'b0010;
  lut8w[49] = 4'b0011;
  lut8w[50] = 4'b0011;
  lut8w[51] = 4'b0100;
  lut8w[52] = 4'b0011;
  lut8w[53] = 4'b0100;
  lut8w[54] = 4'b0100;
  lut8w[55] = 4'b0101;
  lut8w[56] = 4'b0011;
  lut8w[57] = 4'b0100;
  lut8w[58] = 4'b0100;
  lut8w[59] = 4'b0101;
  lut8w[60] = 4'b0100;
  lut8w[61] = 4'b0101;
  lut8w[62] = 4'b0101;
  lut8w[63] = 4'b0110;
  lut8w[64] = 4'b0001;
  lut8w[65] = 4'b0010;
  lut8w[66] = 4'b0010;
  lut8w[67] = 4'b0011;
  lut8w[68] = 4'b0010;
  lut8w[69] = 4'b0011;
  lut8w[70] = 4'b0011;
  lut8w[71] = 4'b0100;
  lut8w[72] = 4'b0010;
  lut8w[73] = 4'b0011;
  lut8w[74] = 4'b0011;
  lut8w[75] = 4'b0100;
  lut8w[76] = 4'b0011;
  lut8w[77] = 4'b0100;
  lut8w[78] = 4'b0100;
  lut8w[79] = 4'b0101;
  lut8w[80] = 4'b0010;
  lut8w[81] = 4'b0011;
  lut8w[82] = 4'b0011;
  lut8w[83] = 4'b0100;
  lut8w[84] = 4'b0011;
  lut8w[85] = 4'b0100;
  lut8w[86] = 4'b0100;
  lut8w[87] = 4'b0101;
  lut8w[88] = 4'b0011;
  lut8w[89] = 4'b0100;
  lut8w[90] = 4'b0100;
  lut8w[91] = 4'b0101;
  lut8w[92] = 4'b0100;
  lut8w[93] = 4'b0101;
  lut8w[94] = 4'b0101;
  lut8w[95] = 4'b0110;
  lut8w[96] = 4'b0010;
  lut8w[97] = 4'b0011;
  lut8w[98] = 4'b0011;
  lut8w[99] = 4'b0100;
  lut8w[100] = 4'b0011;
  lut8w[101] = 4'b0100;
  lut8w[102] = 4'b0100;
  lut8w[103] = 4'b0101;
  lut8w[104] = 4'b0011;
  lut8w[105] = 4'b0100;
  lut8w[106] = 4'b0100;
  lut8w[107] = 4'b0101;
  lut8w[108] = 4'b0100;
  lut8w[109] = 4'b0101;
  lut8w[110] = 4'b0101;
  lut8w[111] = 4'b0110;
  lut8w[112] = 4'b0011;
  lut8w[113] = 4'b0100;
  lut8w[114] = 4'b0100;
  lut8w[115] = 4'b0101;
  lut8w[116] = 4'b0100;
  lut8w[117] = 4'b0101;
  lut8w[118] = 4'b0101;
  lut8w[119] = 4'b0110;
  lut8w[120] = 4'b0100;
  lut8w[121] = 4'b0101;
  lut8w[122] = 4'b0101;
  lut8w[123] = 4'b0110;
  lut8w[124] = 4'b0101;
  lut8w[125] = 4'b0110;
  lut8w[126] = 4'b0110;
  lut8w[127] = 4'b0111;
  lut8w[128] = 4'b0001;
  lut8w[129] = 4'b0010;
  lut8w[130] = 4'b0010;
  lut8w[131] = 4'b0011;
  lut8w[132] = 4'b0010;
  lut8w[133] = 4'b0011;
  lut8w[134] = 4'b0011;
  lut8w[135] = 4'b0100;
  lut8w[136] = 4'b0010;
  lut8w[137] = 4'b0011;
  lut8w[138] = 4'b0011;
  lut8w[139] = 4'b0100;
  lut8w[140] = 4'b0011;
  lut8w[141] = 4'b0100;
  lut8w[142] = 4'b0100;
  lut8w[143] = 4'b0101;
  lut8w[144] = 4'b0010;
  lut8w[145] = 4'b0011;
  lut8w[146] = 4'b0011;
  lut8w[147] = 4'b0100;
  lut8w[148] = 4'b0011;
  lut8w[149] = 4'b0100;
  lut8w[150] = 4'b0100;
  lut8w[151] = 4'b0101;
  lut8w[152] = 4'b0011;
  lut8w[153] = 4'b0100;
  lut8w[154] = 4'b0100;
  lut8w[155] = 4'b0101;
  lut8w[156] = 4'b0100;
  lut8w[157] = 4'b0101;
  lut8w[158] = 4'b0101;
  lut8w[159] = 4'b0110;
  lut8w[160] = 4'b0010;
  lut8w[161] = 4'b0011;
  lut8w[162] = 4'b0011;
  lut8w[163] = 4'b0100;
  lut8w[164] = 4'b0011;
  lut8w[165] = 4'b0100;
  lut8w[166] = 4'b0100;
  lut8w[167] = 4'b0101;
  lut8w[168] = 4'b0011;
  lut8w[169] = 4'b0100;
  lut8w[170] = 4'b0100;
  lut8w[171] = 4'b0101;
  lut8w[172] = 4'b0100;
  lut8w[173] = 4'b0101;
  lut8w[174] = 4'b0101;
  lut8w[175] = 4'b0110;
  lut8w[176] = 4'b0011;
  lut8w[177] = 4'b0100;
  lut8w[178] = 4'b0100;
  lut8w[179] = 4'b0101;
  lut8w[180] = 4'b0100;
  lut8w[181] = 4'b0101;
  lut8w[182] = 4'b0101;
  lut8w[183] = 4'b0110;
  lut8w[184] = 4'b0100;
  lut8w[185] = 4'b0101;
  lut8w[186] = 4'b0101;
  lut8w[187] = 4'b0110;
  lut8w[188] = 4'b0101;
  lut8w[189] = 4'b0110;
  lut8w[190] = 4'b0110;
  lut8w[191] = 4'b0111;
  lut8w[192] = 4'b0010;
  lut8w[193] = 4'b0011;
  lut8w[194] = 4'b0011;
  lut8w[195] = 4'b0100;
  lut8w[196] = 4'b0011;
  lut8w[197] = 4'b0100;
  lut8w[198] = 4'b0100;
  lut8w[199] = 4'b0101;
  lut8w[200] = 4'b0011;
  lut8w[201] = 4'b0100;
  lut8w[202] = 4'b0100;
  lut8w[203] = 4'b0101;
  lut8w[204] = 4'b0100;
  lut8w[205] = 4'b0101;
  lut8w[206] = 4'b0101;
  lut8w[207] = 4'b0110;
  lut8w[208] = 4'b0011;
  lut8w[209] = 4'b0100;
  lut8w[210] = 4'b0100;
  lut8w[211] = 4'b0101;
  lut8w[212] = 4'b0100;
  lut8w[213] = 4'b0101;
  lut8w[214] = 4'b0101;
  lut8w[215] = 4'b0110;
  lut8w[216] = 4'b0100;
  lut8w[217] = 4'b0101;
  lut8w[218] = 4'b0101;
  lut8w[219] = 4'b0110;
  lut8w[220] = 4'b0101;
  lut8w[221] = 4'b0110;
  lut8w[222] = 4'b0110;
  lut8w[223] = 4'b0111;
  lut8w[224] = 4'b0011;
  lut8w[225] = 4'b0100;
  lut8w[226] = 4'b0100;
  lut8w[227] = 4'b0101;
  lut8w[228] = 4'b0100;
  lut8w[229] = 4'b0101;
  lut8w[230] = 4'b0101;
  lut8w[231] = 4'b0110;
  lut8w[232] = 4'b0100;
  lut8w[233] = 4'b0101;
  lut8w[234] = 4'b0101;
  lut8w[235] = 4'b0110;
  lut8w[236] = 4'b0101;
  lut8w[237] = 4'b0110;
  lut8w[238] = 4'b0110;
  lut8w[239] = 4'b0111;
  lut8w[240] = 4'b0100;
  lut8w[241] = 4'b0101;
  lut8w[242] = 4'b0101;
  lut8w[243] = 4'b0110;
  lut8w[244] = 4'b0101;
  lut8w[245] = 4'b0110;
  lut8w[246] = 4'b0110;
  lut8w[247] = 4'b0111;
  lut8w[248] = 4'b0101;
  lut8w[249] = 4'b0110;
  lut8w[250] = 4'b0110;
  lut8w[251] = 4'b0111;
  lut8w[252] = 4'b0110;
  lut8w[253] = 4'b0111;
  lut8w[254] = 4'b0111;
  lut8w[255] = 4'b1000;
 end

 always @(posedge clk) begin
  lut8_BRAM_o_0 = lut8w[lut8_BRAM_i_0];
  lut8_BRAM_o_1 = lut8w[lut8_BRAM_i_1];
 end

endmodule
module lut8_DIST (
  input clk,
  input [7:0]lut8_DIST_i_0,
  input [7:0]lut8_DIST_i_1,
  output reg [3:0]lut8_DIST_o_0,
  output reg [3:0]lut8_DIST_o_1
);

 (* rom_style = "distributed" *)
 reg [3:0] lut8w[255:0];

 initial begin
  lut8w[0] = 4'b0000;
  lut8w[1] = 4'b0001;
  lut8w[2] = 4'b0001;
  lut8w[3] = 4'b0010;
  lut8w[4] = 4'b0001;
  lut8w[5] = 4'b0010;
  lut8w[6] = 4'b0010;
  lut8w[7] = 4'b0011;
  lut8w[8] = 4'b0001;
  lut8w[9] = 4'b0010;
  lut8w[10] = 4'b0010;
  lut8w[11] = 4'b0011;
  lut8w[12] = 4'b0010;
  lut8w[13] = 4'b0011;
  lut8w[14] = 4'b0011;
  lut8w[15] = 4'b0100;
  lut8w[16] = 4'b0001;
  lut8w[17] = 4'b0010;
  lut8w[18] = 4'b0010;
  lut8w[19] = 4'b0011;
  lut8w[20] = 4'b0010;
  lut8w[21] = 4'b0011;
  lut8w[22] = 4'b0011;
  lut8w[23] = 4'b0100;
  lut8w[24] = 4'b0010;
  lut8w[25] = 4'b0011;
  lut8w[26] = 4'b0011;
  lut8w[27] = 4'b0100;
  lut8w[28] = 4'b0011;
  lut8w[29] = 4'b0100;
  lut8w[30] = 4'b0100;
  lut8w[31] = 4'b0101;
  lut8w[32] = 4'b0001;
  lut8w[33] = 4'b0010;
  lut8w[34] = 4'b0010;
  lut8w[35] = 4'b0011;
  lut8w[36] = 4'b0010;
  lut8w[37] = 4'b0011;
  lut8w[38] = 4'b0011;
  lut8w[39] = 4'b0100;
  lut8w[40] = 4'b0010;
  lut8w[41] = 4'b0011;
  lut8w[42] = 4'b0011;
  lut8w[43] = 4'b0100;
  lut8w[44] = 4'b0011;
  lut8w[45] = 4'b0100;
  lut8w[46] = 4'b0100;
  lut8w[47] = 4'b0101;
  lut8w[48] = 4'b0010;
  lut8w[49] = 4'b0011;
  lut8w[50] = 4'b0011;
  lut8w[51] = 4'b0100;
  lut8w[52] = 4'b0011;
  lut8w[53] = 4'b0100;
  lut8w[54] = 4'b0100;
  lut8w[55] = 4'b0101;
  lut8w[56] = 4'b0011;
  lut8w[57] = 4'b0100;
  lut8w[58] = 4'b0100;
  lut8w[59] = 4'b0101;
  lut8w[60] = 4'b0100;
  lut8w[61] = 4'b0101;
  lut8w[62] = 4'b0101;
  lut8w[63] = 4'b0110;
  lut8w[64] = 4'b0001;
  lut8w[65] = 4'b0010;
  lut8w[66] = 4'b0010;
  lut8w[67] = 4'b0011;
  lut8w[68] = 4'b0010;
  lut8w[69] = 4'b0011;
  lut8w[70] = 4'b0011;
  lut8w[71] = 4'b0100;
  lut8w[72] = 4'b0010;
  lut8w[73] = 4'b0011;
  lut8w[74] = 4'b0011;
  lut8w[75] = 4'b0100;
  lut8w[76] = 4'b0011;
  lut8w[77] = 4'b0100;
  lut8w[78] = 4'b0100;
  lut8w[79] = 4'b0101;
  lut8w[80] = 4'b0010;
  lut8w[81] = 4'b0011;
  lut8w[82] = 4'b0011;
  lut8w[83] = 4'b0100;
  lut8w[84] = 4'b0011;
  lut8w[85] = 4'b0100;
  lut8w[86] = 4'b0100;
  lut8w[87] = 4'b0101;
  lut8w[88] = 4'b0011;
  lut8w[89] = 4'b0100;
  lut8w[90] = 4'b0100;
  lut8w[91] = 4'b0101;
  lut8w[92] = 4'b0100;
  lut8w[93] = 4'b0101;
  lut8w[94] = 4'b0101;
  lut8w[95] = 4'b0110;
  lut8w[96] = 4'b0010;
  lut8w[97] = 4'b0011;
  lut8w[98] = 4'b0011;
  lut8w[99] = 4'b0100;
  lut8w[100] = 4'b0011;
  lut8w[101] = 4'b0100;
  lut8w[102] = 4'b0100;
  lut8w[103] = 4'b0101;
  lut8w[104] = 4'b0011;
  lut8w[105] = 4'b0100;
  lut8w[106] = 4'b0100;
  lut8w[107] = 4'b0101;
  lut8w[108] = 4'b0100;
  lut8w[109] = 4'b0101;
  lut8w[110] = 4'b0101;
  lut8w[111] = 4'b0110;
  lut8w[112] = 4'b0011;
  lut8w[113] = 4'b0100;
  lut8w[114] = 4'b0100;
  lut8w[115] = 4'b0101;
  lut8w[116] = 4'b0100;
  lut8w[117] = 4'b0101;
  lut8w[118] = 4'b0101;
  lut8w[119] = 4'b0110;
  lut8w[120] = 4'b0100;
  lut8w[121] = 4'b0101;
  lut8w[122] = 4'b0101;
  lut8w[123] = 4'b0110;
  lut8w[124] = 4'b0101;
  lut8w[125] = 4'b0110;
  lut8w[126] = 4'b0110;
  lut8w[127] = 4'b0111;
  lut8w[128] = 4'b0001;
  lut8w[129] = 4'b0010;
  lut8w[130] = 4'b0010;
  lut8w[131] = 4'b0011;
  lut8w[132] = 4'b0010;
  lut8w[133] = 4'b0011;
  lut8w[134] = 4'b0011;
  lut8w[135] = 4'b0100;
  lut8w[136] = 4'b0010;
  lut8w[137] = 4'b0011;
  lut8w[138] = 4'b0011;
  lut8w[139] = 4'b0100;
  lut8w[140] = 4'b0011;
  lut8w[141] = 4'b0100;
  lut8w[142] = 4'b0100;
  lut8w[143] = 4'b0101;
  lut8w[144] = 4'b0010;
  lut8w[145] = 4'b0011;
  lut8w[146] = 4'b0011;
  lut8w[147] = 4'b0100;
  lut8w[148] = 4'b0011;
  lut8w[149] = 4'b0100;
  lut8w[150] = 4'b0100;
  lut8w[151] = 4'b0101;
  lut8w[152] = 4'b0011;
  lut8w[153] = 4'b0100;
  lut8w[154] = 4'b0100;
  lut8w[155] = 4'b0101;
  lut8w[156] = 4'b0100;
  lut8w[157] = 4'b0101;
  lut8w[158] = 4'b0101;
  lut8w[159] = 4'b0110;
  lut8w[160] = 4'b0010;
  lut8w[161] = 4'b0011;
  lut8w[162] = 4'b0011;
  lut8w[163] = 4'b0100;
  lut8w[164] = 4'b0011;
  lut8w[165] = 4'b0100;
  lut8w[166] = 4'b0100;
  lut8w[167] = 4'b0101;
  lut8w[168] = 4'b0011;
  lut8w[169] = 4'b0100;
  lut8w[170] = 4'b0100;
  lut8w[171] = 4'b0101;
  lut8w[172] = 4'b0100;
  lut8w[173] = 4'b0101;
  lut8w[174] = 4'b0101;
  lut8w[175] = 4'b0110;
  lut8w[176] = 4'b0011;
  lut8w[177] = 4'b0100;
  lut8w[178] = 4'b0100;
  lut8w[179] = 4'b0101;
  lut8w[180] = 4'b0100;
  lut8w[181] = 4'b0101;
  lut8w[182] = 4'b0101;
  lut8w[183] = 4'b0110;
  lut8w[184] = 4'b0100;
  lut8w[185] = 4'b0101;
  lut8w[186] = 4'b0101;
  lut8w[187] = 4'b0110;
  lut8w[188] = 4'b0101;
  lut8w[189] = 4'b0110;
  lut8w[190] = 4'b0110;
  lut8w[191] = 4'b0111;
  lut8w[192] = 4'b0010;
  lut8w[193] = 4'b0011;
  lut8w[194] = 4'b0011;
  lut8w[195] = 4'b0100;
  lut8w[196] = 4'b0011;
  lut8w[197] = 4'b0100;
  lut8w[198] = 4'b0100;
  lut8w[199] = 4'b0101;
  lut8w[200] = 4'b0011;
  lut8w[201] = 4'b0100;
  lut8w[202] = 4'b0100;
  lut8w[203] = 4'b0101;
  lut8w[204] = 4'b0100;
  lut8w[205] = 4'b0101;
  lut8w[206] = 4'b0101;
  lut8w[207] = 4'b0110;
  lut8w[208] = 4'b0011;
  lut8w[209] = 4'b0100;
  lut8w[210] = 4'b0100;
  lut8w[211] = 4'b0101;
  lut8w[212] = 4'b0100;
  lut8w[213] = 4'b0101;
  lut8w[214] = 4'b0101;
  lut8w[215] = 4'b0110;
  lut8w[216] = 4'b0100;
  lut8w[217] = 4'b0101;
  lut8w[218] = 4'b0101;
  lut8w[219] = 4'b0110;
  lut8w[220] = 4'b0101;
  lut8w[221] = 4'b0110;
  lut8w[222] = 4'b0110;
  lut8w[223] = 4'b0111;
  lut8w[224] = 4'b0011;
  lut8w[225] = 4'b0100;
  lut8w[226] = 4'b0100;
  lut8w[227] = 4'b0101;
  lut8w[228] = 4'b0100;
  lut8w[229] = 4'b0101;
  lut8w[230] = 4'b0101;
  lut8w[231] = 4'b0110;
  lut8w[232] = 4'b0100;
  lut8w[233] = 4'b0101;
  lut8w[234] = 4'b0101;
  lut8w[235] = 4'b0110;
  lut8w[236] = 4'b0101;
  lut8w[237] = 4'b0110;
  lut8w[238] = 4'b0110;
  lut8w[239] = 4'b0111;
  lut8w[240] = 4'b0100;
  lut8w[241] = 4'b0101;
  lut8w[242] = 4'b0101;
  lut8w[243] = 4'b0110;
  lut8w[244] = 4'b0101;
  lut8w[245] = 4'b0110;
  lut8w[246] = 4'b0110;
  lut8w[247] = 4'b0111;
  lut8w[248] = 4'b0101;
  lut8w[249] = 4'b0110;
  lut8w[250] = 4'b0110;
  lut8w[251] = 4'b0111;
  lut8w[252] = 4'b0110;
  lut8w[253] = 4'b0111;
  lut8w[254] = 4'b0111;
  lut8w[255] = 4'b1000;
 end

 always @(posedge clk) begin
  lut8_DIST_o_0 = lut8w[lut8_DIST_i_0];
  lut8_DIST_o_1 = lut8w[lut8_DIST_i_1];
 end

endmodule


module popcnt_bram_dsp_16_9 (
input clk,
input	[15:0] input_v,
output	[4:0] output_v
);
wire [7:0] input_0;
wire [7:0] input_1;
wire [7:0] lut8_BRAM_i_0;
wire [7:0] lut8_BRAM_i_1;
wire [3:0] lut8_BRAM_o_0;
wire [3:0] lut8_BRAM_o_1;
assign input_0[7:0] = input_v[15:8];
assign input_1[7:0] = input_v[7:0];
assign lut8_BRAM_i_0[7:0] = input_0[7:0];
assign lut8_BRAM_i_1[7:0] = input_1[7:0];
lut8_BRAM lut8_BRAM_0 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_0),.lut8_BRAM_i_1(lut8_BRAM_i_1),.lut8_BRAM_o_0(lut8_BRAM_o_0),.lut8_BRAM_o_1(lut8_BRAM_o_1) );
wire [3:0] add_0_i_0_0;
wire [3:0] add_0_i_0_1;
wire [3:0] add_0_i_6_0;
wire [3:0] add_0_i_6_1;
wire [4:0] add_0_o_0_0;
wire [4:0] add_0_o_2_0;
assign add_0_i_0_0[3:0] = lut8_BRAM_o_0[3:0];
assign add_0_i_0_1[3:0] = lut8_BRAM_o_1[3:0];
du #(4, 6) du_0_0 (.clk(clk), .stall(1'b0), .dout(add_0_i_6_0),.din(add_0_i_0_0));
du #(4, 6) du_0_1 (.clk(clk), .stall(1'b0), .dout(add_0_i_6_1),.din(add_0_i_0_1));

assign add_0_o_0_0 = add_0_i_6_0 + add_0_i_6_1;

du #(5, 2) du_0_o_0 (.clk(clk), .stall(1'b0), .dout(add_0_o_2_0),.din(add_0_o_0_0));

assign output_v = add_0_o_2_0;

endmodule
module popcnt_bram_nodsp_16_9 (
input clk,
input	[15:0] input_v,
output	[4:0] output_v
);
wire [7:0] input_0;
wire [7:0] input_1;
wire [7:0] lut8_BRAM_i_0;
wire [7:0] lut8_BRAM_i_1;
wire [3:0] lut8_BRAM_o_0;
wire [3:0] lut8_BRAM_o_1;
assign input_0[7:0] = input_v[15:8];
assign input_1[7:0] = input_v[7:0];
assign lut8_BRAM_i_0[7:0] = input_0[7:0];
assign lut8_BRAM_i_1[7:0] = input_1[7:0];
lut8_BRAM lut8_BRAM_0 ( .clk(clk), .lut8_BRAM_i_0(lut8_BRAM_i_0),.lut8_BRAM_i_1(lut8_BRAM_i_1),.lut8_BRAM_o_0(lut8_BRAM_o_0),.lut8_BRAM_o_1(lut8_BRAM_o_1) );
wire [3:0] add_0_i_0_0;
wire [3:0] add_0_i_0_1;
wire [3:0] add_0_i_6_0;
wire [3:0] add_0_i_6_1;
wire [4:0] add_0_o_0_0;
wire [4:0] add_0_o_2_0;
assign add_0_i_0_0[3:0] = lut8_BRAM_o_0[3:0];
assign add_0_i_0_1[3:0] = lut8_BRAM_o_1[3:0];
du #(4, 6) du_0_0 (.clk(clk), .stall(1'b0), .dout(add_0_i_6_0),.din(add_0_i_0_0));
du #(4, 6) du_0_1 (.clk(clk), .stall(1'b0), .dout(add_0_i_6_1),.din(add_0_i_0_1));

assign add_0_o_0_0 = add_0_i_6_0 + add_0_i_6_1;

du #(5, 2) du_0_o_0 (.clk(clk), .stall(1'b0), .dout(add_0_o_2_0),.din(add_0_o_0_0));

assign output_v = add_0_o_2_0;

endmodule
module popcnt_dist_dsp_16_9 (
input clk,
input	[15:0] input_v,
output	[4:0] output_v
);
wire [7:0] input_0;
wire [7:0] input_1;
wire [7:0] lut8_DIST_i_0;
wire [7:0] lut8_DIST_i_1;
wire [3:0] lut8_DIST_o_0;
wire [3:0] lut8_DIST_o_1;
assign input_0[7:0] = input_v[15:8];
assign input_1[7:0] = input_v[7:0];
assign lut8_DIST_i_0[7:0] = input_0[7:0];
assign lut8_DIST_i_1[7:0] = input_1[7:0];
lut8_DIST lut8_DIST_0 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_0),.lut8_DIST_i_1(lut8_DIST_i_1),.lut8_DIST_o_0(lut8_DIST_o_0),.lut8_DIST_o_1(lut8_DIST_o_1) );
wire [3:0] add_0_i_0_0;
wire [3:0] add_0_i_0_1;
wire [3:0] add_0_i_6_0;
wire [3:0] add_0_i_6_1;
wire [4:0] add_0_o_0_0;
wire [4:0] add_0_o_2_0;
assign add_0_i_0_0[3:0] = lut8_DIST_o_0[3:0];
assign add_0_i_0_1[3:0] = lut8_DIST_o_1[3:0];
du #(4, 6) du_0_0 (.clk(clk), .stall(1'b0), .dout(add_0_i_6_0),.din(add_0_i_0_0));
du #(4, 6) du_0_1 (.clk(clk), .stall(1'b0), .dout(add_0_i_6_1),.din(add_0_i_0_1));

assign add_0_o_0_0 = add_0_i_6_0 + add_0_i_6_1;

du #(5, 2) du_0_o_0 (.clk(clk), .stall(1'b0), .dout(add_0_o_2_0),.din(add_0_o_0_0));

assign output_v = add_0_o_2_0;

endmodule
module popcnt_dist_nodsp_16_9 (
input clk,
input	[15:0] input_v,
output	[4:0] output_v
);
wire [7:0] input_0;
wire [7:0] input_1;
wire [7:0] lut8_DIST_i_0;
wire [7:0] lut8_DIST_i_1;
wire [3:0] lut8_DIST_o_0;
wire [3:0] lut8_DIST_o_1;
assign input_0[7:0] = input_v[15:8];
assign input_1[7:0] = input_v[7:0];
assign lut8_DIST_i_0[7:0] = input_0[7:0];
assign lut8_DIST_i_1[7:0] = input_1[7:0];
lut8_DIST lut8_DIST_0 ( .clk(clk), .lut8_DIST_i_0(lut8_DIST_i_0),.lut8_DIST_i_1(lut8_DIST_i_1),.lut8_DIST_o_0(lut8_DIST_o_0),.lut8_DIST_o_1(lut8_DIST_o_1) );
wire [3:0] add_0_i_0_0;
wire [3:0] add_0_i_0_1;
wire [3:0] add_0_i_6_0;
wire [3:0] add_0_i_6_1;
wire [4:0] add_0_o_0_0;
wire [4:0] add_0_o_2_0;
assign add_0_i_0_0[3:0] = lut8_DIST_o_0[3:0];
assign add_0_i_0_1[3:0] = lut8_DIST_o_1[3:0];
du #(4, 6) du_0_0 (.clk(clk), .stall(1'b0), .dout(add_0_i_6_0),.din(add_0_i_0_0));
du #(4, 6) du_0_1 (.clk(clk), .stall(1'b0), .dout(add_0_i_6_1),.din(add_0_i_0_1));

assign add_0_o_0_0 = add_0_i_6_0 + add_0_i_6_1;

du #(5, 2) du_0_o_0 (.clk(clk), .stall(1'b0), .dout(add_0_o_2_0),.din(add_0_o_0_0));

assign output_v = add_0_o_2_0;

endmodule
	
module accum #(	
	parameter IPORT_WIDTH=6,	
				 OPORT_WIDTH=32	
)	
(	
	input clk,	
	input rst,	
	input [IPORT_WIDTH-1:0] valin,	
	output reg[OPORT_WIDTH-1:0] valout	
);	
	
	wire [OPORT_WIDTH-1:0] valin_ext;	
	
	assign valin_ext[IPORT_WIDTH-1:0] = valin;	
	assign valin_ext[OPORT_WIDTH-1:IPORT_WIDTH] = {OPORT_WIDTH - IPORT_WIDTH{1'b0}};	
		
	always @(posedge clk)	
	begin	
		if(rst)		
			valout <= valin_ext;//{OPORT_WIDTH {1'b0}};	
		else	
			valout <= valout + valin_ext;	
	end	
		
endmodule	


module applogic_0	#(					
	parameter PORT_WIDTH = 6					
)					
(					
	input vld_in,					
	input [PORT_WIDTH-1:0] val_i,					
	output [PORT_WIDTH-1:0] val_o					
);				   	
	
	wire [PORT_WIDTH-1:0] vld_v; 
						
	assign vld_v = {PORT_WIDTH-1{vld_in}};						
	assign val_o = val_i & vld_v;						
		
endmodule

		module applogic_1		#(					
	parameter PORT_WIDTH = 6					
)					
(					
	input [PORT_WIDTH-1:0] val_i_0,					
	input [PORT_WIDTH-1:0] val_i_1,					
	output [PORT_WIDTH-1:0] val_o					
);				   	
						
	assign val_o = val_i_0 & val_i_1;						
		
endmodule

		module gblock_0			#(					
	parameter IPORT_WIDTH = 16,					
		  OPORT_WIDTH = 32					
)					
(					
	input clk,					
	input rst,					
	input vld_in,					
	input [IPORT_WIDTH-1:0] val_i,					
	output [IPORT_WIDTH-1:0] val_i_t,					
	output [OPORT_WIDTH-1:0] val_o					
);
				  	 
		
	wire [IPORT_WIDTH-1:0] t_l_val_o;   
	wire [4:0] t_p_val_o;
	wire [31:0] t_a_val_o;
					
	applogic_0 #(IPORT_WIDTH) l_inst (.vld_in(vld_in), .val_i(val_i), .val_o(t_l_val_o));					
	assign val_i_t = t_l_val_o;				   	

	(* use_dsp48 = "yes" *) popcnt_bram_dsp_16_9 p_inst (.clk(clk), .input_v(t_l_val_o), .output_v(t_p_val_o));

	(* use_dsp48 = "yes" *) accum #(5, 32) a_inst (.clk(clk), .rst(rst), .valin(t_p_val_o), .valout(t_a_val_o));

	assign val_o = t_a_val_o;

endmodule

module gblock_1			#(					
	parameter IPORT_WIDTH = 16,					
		  OPORT_WIDTH = 32					
)					
(					
	input clk,					
	input rst,					
	input [IPORT_WIDTH-1:0] val_i_0,					
	input [IPORT_WIDTH-1:0] val_i_1,					
	output [OPORT_WIDTH-1:0] val_o					
);
				   	
		
	wire [IPORT_WIDTH-1:0] t_l_val_o;   
	wire [4:0] t_p_val_o;
	wire [31:0] t_a_val_o;
					
	applogic_1 #(IPORT_WIDTH) l_inst (.val_i_0(val_i_0), .val_i_1(val_i_1), .val_o(t_l_val_o));				  	

	(* use_dsp48 = "yes" *) popcnt_bram_dsp_16_9 p_inst (.clk(clk), .input_v(t_l_val_o), .output_v(t_p_val_o));

	(* use_dsp48 = "yes" *) accum #(5, 32) a_inst (.clk(clk), .rst(rst), .valin(t_p_val_o), .valout(t_a_val_o));

	assign val_o = t_a_val_o;

endmodule

module gblock_1_special			#(					
	parameter IPORT_WIDTH = 16,					
		  OPORT_WIDTH = 32					
)					
(					
	input clk,					
	input rst,					
	input [IPORT_WIDTH-1:0] val_i_0,					
	input [IPORT_WIDTH-1:0] val_i_1,					
	output [OPORT_WIDTH-1:0] val_o					
);
				   	
		
	wire [IPORT_WIDTH-1:0] t_l_val_o;   
	wire [4:0] t_p_val_o;
	wire [31:0] t_a_val_o;
					
	applogic_1 #(IPORT_WIDTH) l_inst (.val_i_0(val_i_0), .val_i_1(val_i_1), .val_o(t_l_val_o));				  	

	(* use_dsp48 = "yes" *) popcnt_bram_dsp_16_9 p_inst (.clk(clk), .input_v(t_l_val_o), .output_v(t_p_val_o));

	(* use_dsp48 = "yes" *) accum #(5, 32) a_inst (.clk(clk), .rst(rst), .valin(t_p_val_o), .valout(t_a_val_o));

	assign val_o = t_a_val_o;

endmodule



module pw_sim_4_1_16_9 (
input clk,
input rst,
input vld_in,
input	[15:0] val_i_y_0,
output	[31:0] y_cnt_0,
input	[15:0] val_i_x_0,
output	[31:0] x_cnt_0,
input	[15:0] val_i_x_1,
output	[31:0] x_cnt_1,
input	[15:0] val_i_x_2,
output	[31:0] x_cnt_2,
input	[15:0] val_i_x_3,
output	[31:0] x_cnt_3,
output	[31:0] xy_cnt_0_0,
output	[31:0] xy_cnt_0_1,
output	[31:0] xy_cnt_0_2,
output	[31:0] xy_cnt_0_3
);

wire [15:0] val_i_y_t_0;

wire [15:0] val_i_x_t_0;
wire [15:0] val_i_x_t_1;
wire [15:0] val_i_x_t_2;
wire [15:0] val_i_x_t_3;

gblock_0 #(16, 32) gblock_0_x_0 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_0), .val_o(x_cnt_0), .val_i_t(val_i_x_t_0));
gblock_0 #(16, 32) gblock_0_x_1 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_1), .val_o(x_cnt_1), .val_i_t(val_i_x_t_1));
gblock_0 #(16, 32) gblock_0_x_2 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_2), .val_o(x_cnt_2), .val_i_t(val_i_x_t_2));
gblock_0 #(16, 32) gblock_0_x_3 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_3), .val_o(x_cnt_3), .val_i_t(val_i_x_t_3));

gblock_0 #(16, 32) gblock_0_y_0 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_y_0), .val_o(y_cnt_0), .val_i_t(val_i_y_t_0));

gblock_1 #(16, 32) gblock_1_0_0 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_0), .val_o(xy_cnt_0_0));
gblock_1 #(16, 32) gblock_1_0_1 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_1), .val_o(xy_cnt_0_1));
gblock_1 #(16, 32) gblock_1_0_2 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_2), .val_o(xy_cnt_0_2));
gblock_1 #(16, 32) gblock_1_0_3 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_3), .val_o(xy_cnt_0_3));


endmodule


module pw_sim_special_4_1_16_9 (
input clk,
input rst,
input vld_in,
input	[15:0] val_i_y_0,
output	[31:0] y_cnt_0,
input	[15:0] val_i_x_0,
output	[31:0] x_cnt_0,
input	[15:0] val_i_x_1,
output	[31:0] x_cnt_1,
input	[15:0] val_i_x_2,
output	[31:0] x_cnt_2,
input	[15:0] val_i_x_3,
output	[31:0] x_cnt_3,
output	[31:0] xy_cnt_0_0,
output	[31:0] xy_cnt_0_1,
output	[31:0] xy_cnt_0_2,
output	[31:0] xy_cnt_0_3
);

wire [15:0] val_i_y_t_0;

wire [15:0] val_i_x_t_0;
wire [15:0] val_i_x_t_1;
wire [15:0] val_i_x_t_2;
wire [15:0] val_i_x_t_3;

gblock_0 #(16, 32) gblock_0_x_0 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_0), .val_o(x_cnt_0), .val_i_t(val_i_x_t_0));
gblock_0 #(16, 32) gblock_0_x_1 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_1), .val_o(x_cnt_1), .val_i_t(val_i_x_t_1));
gblock_0 #(16, 32) gblock_0_x_2 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_2), .val_o(x_cnt_2), .val_i_t(val_i_x_t_2));
gblock_0 #(16, 32) gblock_0_x_3 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_x_3), .val_o(x_cnt_3), .val_i_t(val_i_x_t_3));

gblock_0 #(16, 32) gblock_0_y_0 (.clk(clk), .rst(rst), .vld_in(vld_in), .val_i(val_i_y_0), .val_o(y_cnt_0), .val_i_t(val_i_y_t_0));

gblock_1_special #(16, 32) gblock_1_special_0_0 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_0), .val_o(xy_cnt_0_0));
gblock_1_special #(16, 32) gblock_1_special_0_1 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_1), .val_o(xy_cnt_0_1));
gblock_1_special #(16, 32) gblock_1_special_0_2 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_2), .val_o(xy_cnt_0_2));
gblock_1_special #(16, 32) gblock_1_special_0_3 (.clk(clk), .rst(rst), .val_i_0(val_i_y_t_0), .val_i_1(val_i_x_t_3), .val_o(xy_cnt_0_3));


endmodule


module pw_coefficients_wrapper_4_1_16_9 (
input clk,
input vld_in,
input	[31:0] y_cnt_0,
input	[31:0] x_cnt_0,
input	[31:0] x_cnt_1,
input	[31:0] x_cnt_2,
input	[31:0] x_cnt_3,
input	[31:0] xy_cnt_0_0,
input	[31:0] xy_cnt_0_1,
input	[31:0] xy_cnt_0_2,
input	[31:0] xy_cnt_0_3,
output Tanimoto_wren_0_0,
output Tanimoto_wren_0_1,
output Tanimoto_wren_0_2,
output Tanimoto_wren_0_3,
output	[31:0] Tanimoto_0_0,
output	[31:0] Tanimoto_0_1,
output	[31:0] Tanimoto_0_2,
output	[31:0] Tanimoto_0_3
);

similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_0 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_0), .c(xy_cnt_0_0), .vld_out(Tanimoto_wren_0_0), .Tanimoto_coefficient(Tanimoto_0_0));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_1 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_1), .c(xy_cnt_0_1), .vld_out(Tanimoto_wren_0_1), .Tanimoto_coefficient(Tanimoto_0_1));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_2 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_2), .c(xy_cnt_0_2), .vld_out(Tanimoto_wren_0_2), .Tanimoto_coefficient(Tanimoto_0_2));
similarity_coefficients_pipe #(32, 32) similarity_coefficients_pipe_0_3 (.clk(clk), .vld_in(vld_in), .a(y_cnt_0), .b(x_cnt_3), .c(xy_cnt_0_3), .vld_out(Tanimoto_wren_0_3), .Tanimoto_coefficient(Tanimoto_0_3));


endmodule




module pru_array (
input clk,
input rst,
input	[31:0] element_length_t,
input vld_in,
input	[15:0] val_i_y_0,
input	[15:0] val_i_x_0,
input	[15:0] val_i_x_1,
input	[15:0] val_i_x_2,
input	[15:0] val_i_x_3,
output Tanimoto_wren_0_0,
output Tanimoto_wren_0_1,
output Tanimoto_wren_0_2,
output Tanimoto_wren_0_3,
output	[31:0] Tanimoto_0_0,
output	[31:0] Tanimoto_0_1,
output	[31:0] Tanimoto_0_2,
output	[31:0] Tanimoto_0_3
);

wire vld_in_t;
du #(1, 9) du_vld_in (.clk(clk), .stall(1'b0), .dout(vld_in_t),.din(vld_in));
reg [31:0] element_cnt;
reg rst_l;
wire rst_l_w;
reg vld_in_tt;
wire vld_in_tt_w;

	
	
	always @(posedge clk) begin	
		if(rst_l || rst) begin	
			element_cnt = 0;	
		end	
		else begin	
			if(vld_in_t) begin	
				element_cnt = element_cnt+1;	
			end	
		end	
	end	
	

	
	
	always @(*) begin	
		if(element_cnt==element_length_t) begin	
			rst_l = 1;	
		end	
		else begin	
			rst_l = 0;	
		end	
	end	
	

	
	assign rst_l_w = rst_l;	
	assign vld_in_tt_w = vld_in_tt;	
	always @(posedge clk) begin	
			vld_in_tt = rst_l_w;	
	end	
	

wire [31:0] t_y_cnt_0;

wire [31:0] t_x_cnt_0;
wire [31:0] t_x_cnt_1;
wire [31:0] t_x_cnt_2;
wire [31:0] t_x_cnt_3;

wire [31:0] t_xy_cnt_0_0;
wire [31:0] t_xy_cnt_0_1;
wire [31:0] t_xy_cnt_0_2;
wire [31:0] t_xy_cnt_0_3;
pw_sim_4_1_16_9 pw_sim_4_1_16_9_0(
 .clk(clk),
 .rst(rst | vld_in_tt_w),
 .vld_in(vld_in),
 .val_i_y_0(val_i_y_0),
 .y_cnt_0(t_y_cnt_0),
 .val_i_x_0(val_i_x_0),
 .x_cnt_0(t_x_cnt_0),
 .val_i_x_1(val_i_x_1),
 .x_cnt_1(t_x_cnt_1),
 .val_i_x_2(val_i_x_2),
 .x_cnt_2(t_x_cnt_2),
 .val_i_x_3(val_i_x_3),
 .x_cnt_3(t_x_cnt_3),
 .xy_cnt_0_0(t_xy_cnt_0_0),
 .xy_cnt_0_1(t_xy_cnt_0_1),
 .xy_cnt_0_2(t_xy_cnt_0_2),
 .xy_cnt_0_3(t_xy_cnt_0_3)
);
pw_coefficients_wrapper_4_1_16_9 pw_coefficients_wrapper_4_1_16_9_0(
 .clk(clk),
 .vld_in(vld_in_tt),
 .y_cnt_0(t_y_cnt_0),
 .x_cnt_0(t_x_cnt_0),
 .x_cnt_1(t_x_cnt_1),
 .x_cnt_2(t_x_cnt_2),
 .x_cnt_3(t_x_cnt_3),
 .xy_cnt_0_0(t_xy_cnt_0_0),
 .xy_cnt_0_1(t_xy_cnt_0_1),
 .xy_cnt_0_2(t_xy_cnt_0_2),
 .xy_cnt_0_3(t_xy_cnt_0_3),
 .Tanimoto_wren_0_0(Tanimoto_wren_0_0),
 .Tanimoto_wren_0_1(Tanimoto_wren_0_1),
 .Tanimoto_wren_0_2(Tanimoto_wren_0_2),
 .Tanimoto_wren_0_3(Tanimoto_wren_0_3),
 .Tanimoto_0_0(Tanimoto_0_0),
 .Tanimoto_0_1(Tanimoto_0_1),
 .Tanimoto_0_2(Tanimoto_0_2),
 .Tanimoto_0_3(Tanimoto_0_3)
);
endmodule


module pru_array_special (
input clk,
input rst,
input	[31:0] element_length_t,
input vld_in,
input	[15:0] val_i_y_0,
input	[15:0] val_i_x_0,
input	[15:0] val_i_x_1,
input	[15:0] val_i_x_2,
input	[15:0] val_i_x_3,
output Tanimoto_wren_0_0,
output Tanimoto_wren_0_1,
output Tanimoto_wren_0_2,
output Tanimoto_wren_0_3,
output	[31:0] Tanimoto_0_0,
output	[31:0] Tanimoto_0_1,
output	[31:0] Tanimoto_0_2,
output	[31:0] Tanimoto_0_3
);

wire vld_in_t;
du #(1, 9) du_vld_in (.clk(clk), .stall(1'b0), .dout(vld_in_t),.din(vld_in));
reg [31:0] element_cnt;
reg rst_l;
wire rst_l_w;
reg vld_in_tt;
wire vld_in_tt_w;

	
	
	always @(posedge clk) begin	
		if(rst_l || rst) begin	
			element_cnt = 0;	
		end	
		else begin	
			if(vld_in_t) begin	
				element_cnt = element_cnt+1;	
			end	
		end	
	end	
	

	
	
	always @(*) begin	
		if(element_cnt==element_length_t) begin	
			rst_l = 1;	
		end	
		else begin	
			rst_l = 0;	
		end	
	end	
	

	
	assign rst_l_w = rst_l;	
	assign vld_in_tt_w = vld_in_tt;	
	always @(posedge clk) begin	
			vld_in_tt = rst_l_w;	
	end	
	

wire [31:0] t_y_cnt_0;

wire [31:0] t_x_cnt_0;
wire [31:0] t_x_cnt_1;
wire [31:0] t_x_cnt_2;
wire [31:0] t_x_cnt_3;

wire [31:0] t_xy_cnt_0_0;
wire [31:0] t_xy_cnt_0_1;
wire [31:0] t_xy_cnt_0_2;
wire [31:0] t_xy_cnt_0_3;
pw_sim_special_4_1_16_9 pw_sim_special_4_1_16_9_0(
 .clk(clk),
 .rst(rst | vld_in_tt_w),
 .vld_in(vld_in),
 .val_i_y_0(val_i_y_0),
 .y_cnt_0(t_y_cnt_0),
 .val_i_x_0(val_i_x_0),
 .x_cnt_0(t_x_cnt_0),
 .val_i_x_1(val_i_x_1),
 .x_cnt_1(t_x_cnt_1),
 .val_i_x_2(val_i_x_2),
 .x_cnt_2(t_x_cnt_2),
 .val_i_x_3(val_i_x_3),
 .x_cnt_3(t_x_cnt_3),
 .xy_cnt_0_0(t_xy_cnt_0_0),
 .xy_cnt_0_1(t_xy_cnt_0_1),
 .xy_cnt_0_2(t_xy_cnt_0_2),
 .xy_cnt_0_3(t_xy_cnt_0_3)
);
pw_coefficients_wrapper_4_1_16_9 pw_coefficients_wrapper_4_1_16_9_0(
 .clk(clk),
 .vld_in(vld_in_tt),
 .y_cnt_0(t_y_cnt_0),
 .x_cnt_0(t_x_cnt_0),
 .x_cnt_1(t_x_cnt_1),
 .x_cnt_2(t_x_cnt_2),
 .x_cnt_3(t_x_cnt_3),
 .xy_cnt_0_0(t_xy_cnt_0_0),
 .xy_cnt_0_1(t_xy_cnt_0_1),
 .xy_cnt_0_2(t_xy_cnt_0_2),
 .xy_cnt_0_3(t_xy_cnt_0_3),
 .Tanimoto_wren_0_0(Tanimoto_wren_0_0),
 .Tanimoto_wren_0_1(Tanimoto_wren_0_1),
 .Tanimoto_wren_0_2(Tanimoto_wren_0_2),
 .Tanimoto_wren_0_3(Tanimoto_wren_0_3),
 .Tanimoto_0_0(Tanimoto_0_0),
 .Tanimoto_0_1(Tanimoto_0_1),
 .Tanimoto_0_2(Tanimoto_0_2),
 .Tanimoto_0_3(Tanimoto_0_3)
);
endmodule
	
module bram_tdp #(	
    parameter DATA = 18,	
    parameter ADDR = 9	
)	
 (	
    // Port A	
    input   wire                a_clk,	
    input   wire                a_wr,	
    input   wire    [ADDR-1:0]  a_addr,	
    input   wire    [DATA-1:0]  a_din,	
    output  reg     [DATA-1:0]  a_dout,	
     	
    // Port B	
    input   wire                b_clk,	
    input   wire                b_wr,	
    input   wire    [ADDR-1:0]  b_addr,	
    input   wire    [DATA-1:0]  b_din,	
    output  reg     [DATA-1:0]  b_dout	
);	
 	
// Shared memory	
(* ram_style = "block" *)	
reg [DATA-1:0] mem [(2**ADDR)-1:0];	
 	
// Port A	
always @(posedge a_clk) begin	
    a_dout      <= mem[a_addr];	
    if(a_wr) begin	
        a_dout      <= a_din;	
        mem[a_addr] <= a_din;	
    end	
end	
 	
// Port B	
always @(posedge b_clk) begin	
    b_dout      <= mem[b_addr];	
    if(b_wr) begin	
        b_dout      <= b_din;	
        mem[b_addr] <= b_din;	
    end	
end	
 	
endmodule	
	


	
module dist_tdp #(	
    parameter DATA = 18,	
    parameter ADDR = 9	
)	
 (	
    // Port A	
    input   wire                a_clk,	
    input   wire                a_wr,	
    input   wire    [ADDR-1:0]  a_addr,	
    input   wire    [DATA-1:0]  a_din,	
    output  reg     [DATA-1:0]  a_dout,	
     	
    // Port B	
    input   wire                b_clk,	
    input   wire                b_wr,	
    input   wire    [ADDR-1:0]  b_addr,	
    input   wire    [DATA-1:0]  b_din,	
    output  reg     [DATA-1:0]  b_dout	
);	
 	
// Shared memory	
(* ram_style = "distributed" *)	
reg [DATA-1:0] mem [(2**ADDR)-1:0];	
 	
// Port A	
always @(posedge a_clk) begin	
    a_dout      <= mem[a_addr];	
    if(a_wr) begin	
        a_dout      <= a_din;	
        mem[a_addr] <= a_din;	
    end	
end	
 	
// Port B	
always @(posedge b_clk) begin	
    b_dout      <= mem[b_addr];	
    if(b_wr) begin	
        b_dout      <= b_din;	
        mem[b_addr] <= b_din;	
    end	
end	
 	
endmodule	
	


module imem_bank (
  input clk,
  input [1:0] bank_id,
  input wren_q,
  input [6:0]wraddr_q,
  input [1:0]wrid_q,
  input [15:0]wrdin_q,
  input [6:0]rdaddr_a,
  output [15:0]rddout_a,
  input [6:0]rdaddr_b,
  output [15:0]rddout_b
);

wire wren_a;
assign wren_a = (wren_q==1) & (wrid_q==bank_id);
wire [6:0] addr_a;
assign addr_a = (wren_a==1)?wraddr_q:rdaddr_a;
bram_tdp #(16,7) umem ( .a_clk(clk), .a_wr(wren_a), .a_addr(addr_a), .a_din(wrdin_q), .a_dout(rddout_a), 
  .b_clk(clk), .b_wr(1'b0), .b_addr(rdaddr_b), .b_din(16'b0), .b_dout(rddout_b)); 
endmodule
module omem_bank (
  input clk,
  input [1:0] bank_id,
  input wren,
  input [6:0]wraddr,
  input [31:0]wrdin,
  input rden_q,
  input [6:0]rdaddr_q,
  input [1:0]rdid_q,
  output [31:0]rddout_q
);

reg rden_q_1c_r;
wire rden_q_1c_w;
reg [1:0]rdid_q_1c_r;
wire [1:0]rdid_q_1c_w;
wire [31:0]rddout_q_tmp;
wire rden_sel;
	
	always @(posedge clk) begin	
		rden_q_1c_r = rden_q;	
		rdid_q_1c_r = rdid_q;	
	end	
	
	assign rden_q_1c_w = rden_q_1c_r;	
	assign rdid_q_1c_w = rdid_q_1c_r;	
	assign rden_sel = (rden_q_1c_w==1) & (rdid_q_1c_w==bank_id);
assign rddout_q = (rden_sel==1)?rddout_q_tmp:0;
bram_tdp #(32,7) umem ( .a_clk(clk), .a_wr(wren), .a_addr(wraddr), .a_din(wrdin), .a_dout(), 
  .b_clk(clk), .b_wr(1'b0), .b_addr(rdaddr_q), .b_din(32'b0), .b_dout(rddout_q_tmp)); 
endmodule
module multi_b_imem (
input clk,
input wren_q,
input	[6:0] wraddr_q,
input	[1:0] wrid_q,
input	[15:0] wrdin_q,
input	[6:0] rdaddr_a,
input	[6:0] rdaddr_b,
output	[15:0] rddout_a_0_0,
output	[15:0] rddout_b_0_0,
output	[15:0] rddout_a_0_1,
output	[15:0] rddout_b_0_1,
output	[15:0] rddout_a_0_2,
output	[15:0] rddout_b_0_2,
output	[15:0] rddout_a_0_3,
output	[15:0] rddout_b_0_3
);

wire [1:0] bank_id_0_0;
assign bank_id_0_0 = 0;
imem_bank mem_bank_0_0 (
  .clk(clk),
  .bank_id(bank_id_0_0),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_0),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_0)
);

wire [1:0] bank_id_0_1;
assign bank_id_0_1 = 1;
imem_bank mem_bank_0_1 (
  .clk(clk),
  .bank_id(bank_id_0_1),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_1),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_1)
);

wire [1:0] bank_id_0_2;
assign bank_id_0_2 = 2;
imem_bank mem_bank_0_2 (
  .clk(clk),
  .bank_id(bank_id_0_2),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_2),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_2)
);

wire [1:0] bank_id_0_3;
assign bank_id_0_3 = 3;
imem_bank mem_bank_0_3 (
  .clk(clk),
  .bank_id(bank_id_0_3),
  .wren_q(wren_q),
  .wraddr_q(wraddr_q),
  .wrid_q(wrid_q),
  .wrdin_q(wrdin_q),
  .rdaddr_a(rdaddr_a),
  .rddout_a(rddout_a_0_3),
  .rdaddr_b(rdaddr_b),
  .rddout_b(rddout_b_0_3)
);

endmodule
module multi_b_omem (
input clk,
input	[6:0] wraddr,
input wren_a_0_0,
input wren_b_0_0,
input wren_a_0_1,
input wren_b_0_1,
input wren_a_0_2,
input wren_b_0_2,
input wren_a_0_3,
input wren_b_0_3,
input	[31:0] wrdin_a_0_0,
input	[31:0] wrdin_b_0_0,
input	[31:0] wrdin_a_0_1,
input	[31:0] wrdin_b_0_1,
input	[31:0] wrdin_a_0_2,
input	[31:0] wrdin_b_0_2,
input	[31:0] wrdin_a_0_3,
input	[31:0] wrdin_b_0_3,
input rden_q,
input	[6:0] rdaddr_q,
input	[1:0] rdid_q,
output	[31:0] rddout_q_a_0,
output	[31:0] rddout_q_b_0
);

wire [31:0] rddout_q_a_0_0;
wire [31:0] rddout_q_a_0_1;
wire [31:0] rddout_q_a_0_2;
wire [31:0] rddout_q_a_0_3;
wire [31:0] rddout_q_a_0_0_0;
wire [31:0] rddout_q_a_0_1_1;
wire [31:0] rddout_q_a_0_2_2;
wire [31:0] rddout_q_a_0_3_3;
assign rddout_q_a_0_0_0[31:0] = rddout_q_a_0_0;
assign rddout_q_a_0_1_1[31:0] = rddout_q_a_0_1;
assign rddout_q_a_0_2_2[31:0] = rddout_q_a_0_2;
assign rddout_q_a_0_3_3[31:0] = rddout_q_a_0_3;
wire [31:0] rddout_q_a_0_tmp_0_0;
wire [31:0] rddout_q_a_0_tmp_0_1;
wire [31:0] rddout_q_a_0_tmp_d_0_0;
wire [31:0] rddout_q_a_0_tmp_d_0_1;
wire [31:0] rddout_q_a_0_tmp_1_0;
wire [31:0] rddout_q_a_0_tmp_d_1_0;

 assign rddout_q_a_0_tmp_0_0 = rddout_q_a_0_0_0 | rddout_q_a_0_1_1 ;
 
 assign rddout_q_a_0_tmp_0_1 = rddout_q_a_0_2_2 | rddout_q_a_0_3_3 ;
 
 assign rddout_q_a_0_tmp_1_0 = rddout_q_a_0_tmp_d_0_0 | rddout_q_a_0_tmp_d_0_1 ;
 du_s #(32) du_0_0_0 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_0),.din(rddout_q_a_0_tmp_0_0));
du_s #(32) du_0_0_1 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_0_1),.din(rddout_q_a_0_tmp_0_1));
du_s #(32) du_0_1_0 (.clk(clk), .stall(1'b0), .dout(rddout_q_a_0_tmp_d_1_0),.din(rddout_q_a_0_tmp_1_0));
assign rddout_q_a_0 = rddout_q_a_0_tmp_d_1_0;
wire [31:0] rddout_q_b_0_0;
wire [31:0] rddout_q_b_0_1;
wire [31:0] rddout_q_b_0_2;
wire [31:0] rddout_q_b_0_3;
wire [31:0] rddout_q_b_0_0_0;
wire [31:0] rddout_q_b_0_1_1;
wire [31:0] rddout_q_b_0_2_2;
wire [31:0] rddout_q_b_0_3_3;
assign rddout_q_b_0_0_0[31:0] = rddout_q_b_0_0;
assign rddout_q_b_0_1_1[31:0] = rddout_q_b_0_1;
assign rddout_q_b_0_2_2[31:0] = rddout_q_b_0_2;
assign rddout_q_b_0_3_3[31:0] = rddout_q_b_0_3;
wire [31:0] rddout_q_b_0_tmp_0_0;
wire [31:0] rddout_q_b_0_tmp_0_1;
wire [31:0] rddout_q_b_0_tmp_d_0_0;
wire [31:0] rddout_q_b_0_tmp_d_0_1;
wire [31:0] rddout_q_b_0_tmp_1_0;
wire [31:0] rddout_q_b_0_tmp_d_1_0;

 assign rddout_q_b_0_tmp_0_0 = rddout_q_b_0_0_0 | rddout_q_b_0_1_1 ;
 
 assign rddout_q_b_0_tmp_0_1 = rddout_q_b_0_2_2 | rddout_q_b_0_3_3 ;
 
 assign rddout_q_b_0_tmp_1_0 = rddout_q_b_0_tmp_d_0_0 | rddout_q_b_0_tmp_d_0_1 ;
 du_s #(32) du_1_0_0 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_0),.din(rddout_q_b_0_tmp_0_0));
du_s #(32) du_1_0_1 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_0_1),.din(rddout_q_b_0_tmp_0_1));
du_s #(32) du_1_1_0 (.clk(clk), .stall(1'b0), .dout(rddout_q_b_0_tmp_d_1_0),.din(rddout_q_b_0_tmp_1_0));
assign rddout_q_b_0 = rddout_q_b_0_tmp_d_1_0;
omem_bank mem_bank_0_0_a (
  .clk(clk),
  .bank_id(2'b00),
  .wren(wren_a_0_0),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_0),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_0)
);

omem_bank mem_bank_0_0_b (
  .clk(clk),
  .bank_id(2'b00),
  .wren(wren_b_0_0),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_0),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_0)
);

omem_bank mem_bank_0_1_a (
  .clk(clk),
  .bank_id(2'b01),
  .wren(wren_a_0_1),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_1),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_1)
);

omem_bank mem_bank_0_1_b (
  .clk(clk),
  .bank_id(2'b01),
  .wren(wren_b_0_1),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_1),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_1)
);

omem_bank mem_bank_0_2_a (
  .clk(clk),
  .bank_id(2'b10),
  .wren(wren_a_0_2),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_2),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_2)
);

omem_bank mem_bank_0_2_b (
  .clk(clk),
  .bank_id(2'b10),
  .wren(wren_b_0_2),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_2),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_2)
);

omem_bank mem_bank_0_3_a (
  .clk(clk),
  .bank_id(2'b11),
  .wren(wren_a_0_3),
  .wraddr(wraddr),
  .wrdin(wrdin_a_0_3),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_a_0_3)
);

omem_bank mem_bank_0_3_b (
  .clk(clk),
  .bank_id(2'b11),
  .wren(wren_b_0_3),
  .wraddr(wraddr),
  .wrdin(wrdin_b_0_3),
  .rden_q(rden_q),
  .rdid_q(rdid_q),
  .rdaddr_q(rdaddr_q),
  .rddout_q(rddout_q_b_0_3)
);

endmodule
module top (
input clk,
input rst,
input	[31:0] element_length,
input wren_q,
input	[6:0] wraddr_q,
input	[1:0] wrid_q,
input	[15:0] wrdin_q,
input vld_in,
input	[6:0] rdaddr_a,
input	[6:0] rdaddr_b,
input	[15:0] val_i_y_0,
input rden_q,
input	[6:0] rdaddr_q,
input	[1:0] rdid_q,
output	[31:0] rddout_q_a_0,
output	[31:0] rddout_q_b_0
);

reg [31:0] element_length_t;
reg [31:0] cur_element_length;
reg vld_in_t, rst_l;
reg [6:0] wraddr;
wire [15:0] d1_val_i_y_0;
du_s #(16) du_99_0 (.clk(clk), .stall(1'b0), .dout(d1_val_i_y_0),.din(val_i_y_0));

	
	
	always @(posedge clk) begin	
		if(rst)	
			element_length_t <= element_length-1;	
	end	
	

	
	
	always @(posedge clk) begin	
		vld_in_t <= vld_in;	
	end	
	

	
	
	always @(posedge clk) begin	
		if(rst_l || rst) begin	
			cur_element_length = 0;	
		end	
		else begin	
			if(vld_in_t) begin	
				cur_element_length = cur_element_length+1;	
			end	
		end	
	end	
	

	
	
	always @(posedge clk) begin	
		if(cur_element_length==element_length_t) begin	
			rst_l = 1;	
		end	
		else begin	
			rst_l = 0;	
		end	
	end	
	

	
	
	always @(posedge clk) begin	
		if(rst) begin	
			wraddr = 0;	
		end	
		else begin	
			if(wren_a_0_0) begin	
				wraddr = wraddr+{7'b1};	
			end	
		end	
	end	
	


wire [15:0] rddout_a_0_0;
wire [15:0] rddout_a_0_1;
wire [15:0] rddout_a_0_2;
wire [15:0] rddout_a_0_3;


wire [15:0] rddout_b_0_0;
wire [15:0] rddout_b_0_1;
wire [15:0] rddout_b_0_2;
wire [15:0] rddout_b_0_3;




wire [15:0] val_i_x_0_a;
wire [15:0] val_i_x_1_a;
wire [15:0] val_i_x_2_a;
wire [15:0] val_i_x_3_a;
wire [31:0] Tanimoto_0_0_a;
wire [31:0] Tanimoto_0_1_a;
wire [31:0] Tanimoto_0_2_a;
wire [31:0] Tanimoto_0_3_a;
wire [0:0] Tanimoto_wren_0_0_a;
wire [0:0] Tanimoto_wren_0_1_a;
wire [0:0] Tanimoto_wren_0_2_a;
wire [0:0] Tanimoto_wren_0_3_a;


wire [15:0] val_i_x_0_b;
wire [15:0] val_i_x_1_b;
wire [15:0] val_i_x_2_b;
wire [15:0] val_i_x_3_b;
wire [31:0] Tanimoto_0_0_b;
wire [31:0] Tanimoto_0_1_b;
wire [31:0] Tanimoto_0_2_b;
wire [31:0] Tanimoto_0_3_b;
wire [0:0] Tanimoto_wren_0_0_b;
wire [0:0] Tanimoto_wren_0_1_b;
wire [0:0] Tanimoto_wren_0_2_b;
wire [0:0] Tanimoto_wren_0_3_b;


wire [0:0] wren_a_0_0;
wire [0:0] wren_a_0_1;
wire [0:0] wren_a_0_2;
wire [0:0] wren_a_0_3;
wire [31:0] wrdin_a_0_0;
wire [31:0] wrdin_a_0_1;
wire [31:0] wrdin_a_0_2;
wire [31:0] wrdin_a_0_3;


wire [0:0] wren_b_0_0;
wire [0:0] wren_b_0_1;
wire [0:0] wren_b_0_2;
wire [0:0] wren_b_0_3;
wire [31:0] wrdin_b_0_0;
wire [31:0] wrdin_b_0_1;
wire [31:0] wrdin_b_0_2;
wire [31:0] wrdin_b_0_3;


assign val_i_x_0_a[15:0] = rddout_a_0_0[15:0];
assign val_i_x_1_a[15:0] = rddout_a_0_1[15:0];
assign val_i_x_2_a[15:0] = rddout_a_0_2[15:0];
assign val_i_x_3_a[15:0] = rddout_a_0_3[15:0];


assign val_i_x_0_b[15:0] = rddout_b_0_0[15:0];
assign val_i_x_1_b[15:0] = rddout_b_0_1[15:0];
assign val_i_x_2_b[15:0] = rddout_b_0_2[15:0];
assign val_i_x_3_b[15:0] = rddout_b_0_3[15:0];


du_s #(1) du_2_0 (.clk(clk), .stall(1'b0), .dout(wren_a_0_0),.din(Tanimoto_wren_0_0_a));
du_s #(1) du_2_1 (.clk(clk), .stall(1'b0), .dout(wren_a_0_1),.din(Tanimoto_wren_0_1_a));
du_s #(1) du_2_2 (.clk(clk), .stall(1'b0), .dout(wren_a_0_2),.din(Tanimoto_wren_0_2_a));
du_s #(1) du_2_3 (.clk(clk), .stall(1'b0), .dout(wren_a_0_3),.din(Tanimoto_wren_0_3_a));
du_s #(1) du_3_0 (.clk(clk), .stall(1'b0), .dout(wren_b_0_0),.din(Tanimoto_wren_0_0_b));
du_s #(1) du_3_1 (.clk(clk), .stall(1'b0), .dout(wren_b_0_1),.din(Tanimoto_wren_0_1_b));
du_s #(1) du_3_2 (.clk(clk), .stall(1'b0), .dout(wren_b_0_2),.din(Tanimoto_wren_0_2_b));
du_s #(1) du_3_3 (.clk(clk), .stall(1'b0), .dout(wren_b_0_3),.din(Tanimoto_wren_0_3_b));
du_s #(32) du_4_0 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_0),.din(Tanimoto_0_0_a));
du_s #(32) du_4_1 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_1),.din(Tanimoto_0_1_a));
du_s #(32) du_4_2 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_2),.din(Tanimoto_0_2_a));
du_s #(32) du_4_3 (.clk(clk), .stall(1'b0), .dout(wrdin_a_0_3),.din(Tanimoto_0_3_a));
du_s #(32) du_5_0 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_0),.din(Tanimoto_0_0_b));
du_s #(32) du_5_1 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_1),.din(Tanimoto_0_1_b));
du_s #(32) du_5_2 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_2),.din(Tanimoto_0_2_b));
du_s #(32) du_5_3 (.clk(clk), .stall(1'b0), .dout(wrdin_b_0_3),.din(Tanimoto_0_3_b));


multi_b_imem multi_b_imem_0(
 .clk(clk),
 .wren_q(wren_q),
 .wraddr_q(wraddr_q),
 .wrid_q(wrid_q),
 .wrdin_q(wrdin_q),
 .rdaddr_a(rdaddr_a),
 .rdaddr_b(rdaddr_b),
 .rddout_a_0_0(rddout_a_0_0),
 .rddout_b_0_0(rddout_b_0_0),
 .rddout_a_0_1(rddout_a_0_1),
 .rddout_b_0_1(rddout_b_0_1),
 .rddout_a_0_2(rddout_a_0_2),
 .rddout_b_0_2(rddout_b_0_2),
 .rddout_a_0_3(rddout_a_0_3),
 .rddout_b_0_3(rddout_b_0_3)
);
pru_array pru_array_a(
 .clk(clk),
 .rst(rst),
 .element_length_t(element_length_t),
 .vld_in(vld_in_t),
 .val_i_y_0(d1_val_i_y_0),
 .val_i_x_0(val_i_x_0_a),
 .val_i_x_1(val_i_x_1_a),
 .val_i_x_2(val_i_x_2_a),
 .val_i_x_3(val_i_x_3_a),
 .Tanimoto_wren_0_0(Tanimoto_wren_0_0_a),
 .Tanimoto_wren_0_1(Tanimoto_wren_0_1_a),
 .Tanimoto_wren_0_2(Tanimoto_wren_0_2_a),
 .Tanimoto_wren_0_3(Tanimoto_wren_0_3_a),
 .Tanimoto_0_0(Tanimoto_0_0_a),
 .Tanimoto_0_1(Tanimoto_0_1_a),
 .Tanimoto_0_2(Tanimoto_0_2_a),
 .Tanimoto_0_3(Tanimoto_0_3_a)
);
pru_array_special pru_array_special_b(
 .clk(clk),
 .rst(rst),
 .element_length_t(element_length_t),
 .vld_in(vld_in_t),
 .val_i_y_0(d1_val_i_y_0),
 .val_i_x_0(val_i_x_0_b),
 .val_i_x_1(val_i_x_1_b),
 .val_i_x_2(val_i_x_2_b),
 .val_i_x_3(val_i_x_3_b),
 .Tanimoto_wren_0_0(Tanimoto_wren_0_0_b),
 .Tanimoto_wren_0_1(Tanimoto_wren_0_1_b),
 .Tanimoto_wren_0_2(Tanimoto_wren_0_2_b),
 .Tanimoto_wren_0_3(Tanimoto_wren_0_3_b),
 .Tanimoto_0_0(Tanimoto_0_0_b),
 .Tanimoto_0_1(Tanimoto_0_1_b),
 .Tanimoto_0_2(Tanimoto_0_2_b),
 .Tanimoto_0_3(Tanimoto_0_3_b)
);


multi_b_omem multi_b_omem_0(
 .clk(clk),
 .wraddr(wraddr),
 .wren_a_0_0(wren_a_0_0),
 .wren_b_0_0(wren_b_0_0),
 .wren_a_0_1(wren_a_0_1),
 .wren_b_0_1(wren_b_0_1),
 .wren_a_0_2(wren_a_0_2),
 .wren_b_0_2(wren_b_0_2),
 .wren_a_0_3(wren_a_0_3),
 .wren_b_0_3(wren_b_0_3),
 .wrdin_a_0_0(wrdin_a_0_0),
 .wrdin_b_0_0(wrdin_b_0_0),
 .wrdin_a_0_1(wrdin_a_0_1),
 .wrdin_b_0_1(wrdin_b_0_1),
 .wrdin_a_0_2(wrdin_a_0_2),
 .wrdin_b_0_2(wrdin_b_0_2),
 .wrdin_a_0_3(wrdin_a_0_3),
 .wrdin_b_0_3(wrdin_b_0_3),
 .rden_q(rden_q),
 .rdaddr_q(rdaddr_q),
 .rdid_q(rdid_q),
 .rddout_q_a_0(rddout_q_a_0),
 .rddout_q_b_0(rddout_q_b_0)
);


endmodule
