`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   23:14:27 05/04/2025
// Design Name:   cia
// Module Name:   /mnt/batibat/Maciejdev/plus4/burstcart/ciasdr-hdl-72/ciasdr_tb.v
// Project Name:  ciasdr-hdl-72
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: cia
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module ciasdr_tb;

	// Inputs
	reg RESET_n;
	reg E_CLK;
	reg RW;
	reg MUX;
	reg [15:0] A;
	reg phi2;
	reg aec;
	reg ba;
	reg c1lo;
	reg c1hi;
	reg c2lo;
	reg c2hi;

	// Outputs
	wire rom_a15;
	wire rom_cs;

	// Bidirs
	wire [7:0] D;
	wire CNT;
	wire SP;

    // Dwukierunkowa szyna danych - sterowana ręcznie w testbenchu
    reg [7:0] D_in;
    wire [7:0] D_out;
    assign D = (RW) ? 8'bz : D_in;
    assign D_out = D;

	// Instantiate the Unit Under Test (UUT)
	cia uut (
		.RESET_n(RESET_n), 
		.E_CLK(E_CLK), 
		.RW(RW), 
		.MUX(MUX), 
		.A(A), 
		.D(D), 
		.phi2(phi2), 
		.aec(aec), 
		.ba(ba), 
		.CNT(CNT), 
		.SP(SP), 
		.c1lo(c1lo), 
		.c1hi(c1hi), 
		.c2lo(c2lo), 
		.c2hi(c2hi), 
		.rom_a15(rom_a15), 
		.rom_cs(rom_cs)
	);

    // Clock generation
    always #10 E_CLK = ~E_CLK;  // 50 MHz

	initial begin
		// Initialize Inputs
		RESET_n = 0;
		E_CLK = 0;
		RW = 0;
		MUX = 0;
		A = 0;
		phi2 = 0;
		aec = 0;
		ba = 0;
		c1lo = 0;
		c1hi = 0;
		c2lo = 0;
		c2hi = 0;

		// Wait 100 ns for global reset to finish
		#100;
 
		#50;
      RESET_n = 1;
		  
		// Add stimulus here

        // Example write to CRA
        @(negedge E_CLK);
        A = 16'hFD91;  // REG_CRA address
        RW = 0;        // Write
        D_in = 8'h40;

        @(negedge E_CLK);
        RW = 1;        // Back to read

        // Wait and observe behavior
        #200;

        // Example write to SDR
        @(negedge E_CLK);
        A = 16'hFD90;  // REG_SDR address
        RW = 0;        // Write
        D_in = 8'hA5;

        @(negedge E_CLK);
        RW = 1;        // Back to read

        // Wait and observe behavior
        #200;

	end
      
endmodule

