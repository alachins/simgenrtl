`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   18:33:16 06/09/2015
// Design Name:   top
// Module Name:   /home/alachins/Desktop/CHEM-SIM-RTL/chem_pipe2/testbench.v
// Project Name:  chem_pipe2
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: top
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module testbench;

	// Inputs
	reg clk;
	reg rst;
	reg [31:0] element_length;
	reg wren_q;
	reg [6:0] wraddr_q;
	reg [1:0] wrid_q;
	reg [15:0] wrdin_q;
	reg vld_in;
	reg [6:0] rdaddr_a;
	reg [6:0] rdaddr_b;
	reg [15:0] val_i_y_0;
	reg rden_q;
	reg [6:0] rdaddr_q;
	reg [1:0] rdid_q;

	// Outputs
	wire [31:0] rddout_q_a_0;
	wire [31:0] rddout_q_b_0;

	// Instantiate the Unit Under Test (UUT)
	top uut (
		.clk(clk), 
		.rst(rst), 
		.element_length(element_length), 
		.wren_q(wren_q), 
		.wraddr_q(wraddr_q), 
		.wrid_q(wrid_q), 
		.wrdin_q(wrdin_q), 
		.vld_in(vld_in), 
		.rdaddr_a(rdaddr_a), 
		.rdaddr_b(rdaddr_b), 
		.val_i_y_0(val_i_y_0), 
		.rden_q(rden_q), 
		.rdaddr_q(rdaddr_q), 
		.rdid_q(rdid_q), 
		.rddout_q_a_0(rddout_q_a_0), 
		.rddout_q_b_0(rddout_q_b_0)
	);
	
	always  #10 clk = ~clk ;

	initial begin
		// Initialize Inputs
		clk = 0;
		rst = 1;
		element_length = 4;
		wren_q = 1;
		wraddr_q = 0;
		wrid_q = 0;
		wrdin_q = 0;
		vld_in = 0;
		rdaddr_a = 0;
		rdaddr_b = 0;
		val_i_y_0 = 0;
		rden_q = 0;
		rdaddr_q = 0;
		rdid_q = 0;

		// Wait 100 ns for global reset to finish
		#100;
      @(posedge clk) ;
		wrid_q = 1;
		@(posedge clk) ;
				wrid_q = 2;
		@(posedge clk) ;
				wrid_q = 3;
		@(posedge clk) ;
		rst=0;
		wren_q = 0;
		@(posedge clk) ;
@(posedge clk) ;
@(posedge clk) ;
		@(posedge clk) ;
@(posedge clk) ;
@(posedge clk) ;
		@(posedge clk) ;
@(posedge clk) ;
@(posedge clk) ;
		@(posedge clk) ;
@(posedge clk) ;
@(posedge clk) ;
		@(posedge clk) ;
@(posedge clk) ;
@(posedge clk) ;
		@(posedge clk) ;
@(posedge clk) ;
@(posedge clk) ;
		@(posedge clk) ;
@(posedge clk) ;
@(posedge clk) ;
		@(posedge clk) ;
@(posedge clk) ;
@(posedge clk) ;
		@(posedge clk) ;
@(posedge clk) ;
@(posedge clk) ;
		@(posedge clk) ;
@(posedge clk) ;
@(posedge clk) ;
		@(posedge clk) ;
@(posedge clk) ;
@(posedge clk) ;
		@(posedge clk) ;
@(posedge clk) ;
@(posedge clk) ;
		@(posedge clk) ;
@(posedge clk) ;
@(posedge clk) ;
// Bank 0 Fingerprint 0		
		wren_q = 1;
		wraddr_q = 0;
		wrid_q = 0;
		wrdin_q = 1;
		@(posedge clk) ;
		wren_q = 1;
		wraddr_q = 1;
		wrid_q = 0;
		wrdin_q = 2;
		@(posedge clk) ;	
// Bank 1 Fingerprint 0		
		wren_q = 1;
		wraddr_q = 0;
		wrid_q = 1;
		wrdin_q = 3;
		@(posedge clk) ;
		wren_q = 1;
		wraddr_q = 1;
		wrid_q = 1;
		wrdin_q = 4;
		@(posedge clk) ;		
// Bank 1 Fingerprint 0		
		wren_q = 1;
		wraddr_q = 0;
		wrid_q = 2;
		wrdin_q = 5;
		@(posedge clk) ;
		wren_q = 1;
		wraddr_q = 1;
		wrid_q = 2;
		wrdin_q = 6;
		@(posedge clk) ;		
// Bank 1 Fingerprint 0		
		wren_q = 1;
		wraddr_q = 0;
		wrid_q = 3;
		wrdin_q = 7;
		@(posedge clk) ;
		wren_q = 1;
		wraddr_q = 1;
		wrid_q = 3;
		wrdin_q = 8;
		@(posedge clk) ;
		
		wren_q = 0;
		wraddr_q = 0;
		wrid_q = 0;
		wrdin_q = 0;
		rst =1;
		@(posedge clk) ;
		rst = 0;
		@(posedge clk) ;

// Reading fingerprint		
		vld_in = 1;
		rdaddr_a = 0;
		rdaddr_b = 0;
		val_i_y_0 = 7;
			@(posedge clk) ;	
		rdaddr_a = 1;
		rdaddr_b = 1;
			@(posedge clk) ;	
		rdaddr_a = 0;
		rdaddr_b = 0;
			@(posedge clk) ;	
		rdaddr_a = 1;
		rdaddr_b = 1;		
			@(posedge clk) ; // second match
	// one cycle gap
			vld_in = 0;
			val_i_y_0 = 0;
		rdaddr_a = 0;
		rdaddr_b = 0;
			@(posedge clk) ; // second match
						val_i_y_0 = 45;
			vld_in = 1;
		rdaddr_a = 0;
		rdaddr_b = 0;
			@(posedge clk) ; // second match
			vld_in = 1;
		rdaddr_a = 1;
		rdaddr_b = 1;
			@(posedge clk) ; // second match
			vld_in = 1;
		rdaddr_a = 0;
		rdaddr_b = 0;		
			@(posedge clk) ;

			vld_in = 1;
		rdaddr_a = 1;
		rdaddr_b = 1;
						@(posedge clk) ; // second match		
	rdaddr_a = 0;
		rdaddr_b = 0;
vld_in = 0;	
			@(posedge clk) ;
	val_i_y_0 = 0;	

#1000;
      @(posedge clk) ;
			rden_q=1;
		rdaddr_q = 0;
		rdid_q = 0;
		      @(posedge clk) ;	
		rdaddr_q = 0;
		rdid_q = 1;
		      @(posedge clk) ;	
		rdaddr_q = 0;
		rdid_q = 2;
		      @(posedge clk) ;	
		rdaddr_q = 0;
		rdid_q = 3;
		      @(posedge clk) ;	
		rdaddr_q = 1;
		rdid_q = 0;
		      @(posedge clk) ;	
		rdaddr_q = 1;
		rdid_q = 1;
		      @(posedge clk) ;	
		rdaddr_q = 1;
		rdid_q = 2;
		      @(posedge clk) ;	
		rdaddr_q = 1;
		rdid_q = 3;
		@(posedge clk) ;
		rden_q=0;	
		@(posedge clk) ;
		
		// Add stimulus here

	end
      
endmodule