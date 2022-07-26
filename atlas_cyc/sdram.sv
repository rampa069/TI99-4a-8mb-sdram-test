//
// sdram.v
//
// sdram controller implementation for the MiST board
// https://github.com/mist-devel/mist-board
// 
// Copyright (c) 2013 Till Harbaum <till@harbaum.org> 
// Copyright (c) 2019 Gyorgy Szombathelyi
//
// This source file is free software: you can redistribute it and/or modify 
// it under the terms of the GNU General Public License as published 
// by the Free Software Foundation, either version 3 of the License, or 
// (at your option) any later version. 
// 
// This source file is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of 
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the 
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License 
// along with this program.  If not, see <http://www.gnu.org/licenses/>. 
//

module sdram (

	// interface to the MT48LC16M16 chip
	inout  reg [15:0] SDRAM_DQ,   // 16 bit bidirectional data bus
	output reg [11:0] SDRAM_A,    // 12 bit multiplexed address bus
	output reg        SDRAM_DQML, // two byte masks
	output reg        SDRAM_DQMH, // two byte masks
	output reg [1:0]  SDRAM_BA,   // two banks
	output            SDRAM_nCS,  // a single chip select
	output            SDRAM_nWE,  // write enable
	output            SDRAM_nRAS, // row address select
	output            SDRAM_nCAS, // columns address select
	output            SDRAM_CKE,

	// cpu/chipset interface
	input             init_n,     // init signal after FPGA config to initialize RAM
	input             clk,        // sdram clock
	input             port1_req,
	output reg        port1_ack,
	input             port1_we,
	input      [21:1] port1_a,
	input       [1:0] port1_ds,
	input      [15:0] port1_d,
	output reg [15:0] port1_q
);

parameter  MHZ = 16'd85; // 80 MHz default clock, set it to proper value to calculate refresh rate

localparam RASCAS_DELAY   = 3'd2;   // tRCD=20ns -> 2 cycles@<100MHz
localparam BURST_LENGTH   = 3'b000; // 000=1, 001=2, 010=4, 011=8
localparam ACCESS_TYPE    = 1'b0;   // 0=sequential, 1=interleaved
localparam CAS_LATENCY    = 3'd2;   // 2/3 allowed
localparam OP_MODE        = 2'b00;  // only 00 (standard operation) allowed
localparam NO_WRITE_BURST = 1'b1;   // 0= write burst enabled, 1=only single access write

localparam MODE = { 2'b00, NO_WRITE_BURST, OP_MODE, CAS_LATENCY, ACCESS_TYPE, BURST_LENGTH}; 
// 64ms/8192 rows = 7.8us
// 64ms/4096 rows = 15.6us

localparam RFRSH_CYCLES = 16'd156*MHZ/4'd10;
// ---------------------------------------------------------------------
// ------------------------ cycle state machine ------------------------
// ---------------------------------------------------------------------

/*
 Simple SDRAM state machine
 1 word burst, CL2
cmd issued  registered
 0 RAS0
 1          ras0
 2 CAS0
 3          cas0
 4
 5          data0 returned
*/

//localparam STATE_RAS0      = 3'd0;   // first state in cycle
//localparam STATE_CAS0      = STATE_RAS0 + RASCAS_DELAY; // CAS phase - 3
//localparam STATE_READ0     = STATE_CAS0 + CAS_LATENCY + 1'd1; // 5
//localparam STATE_LAST      = STATE_READ0;

localparam STATE_RAS0      = 3'd0;   // first state in cycle
localparam STATE_RAS1      = 3'd3;   // Second ACTIVE command after RAS0 + tRRD (15ns)
localparam STATE_CAS0      = STATE_RAS0 + RASCAS_DELAY; // CAS phase - 3
localparam STATE_READ0     = STATE_CAS0 + CAS_LATENCY + 2'd2; // 6
localparam STATE_LAST      = 3'd7;

reg [2:0] t;

always @(posedge clk) begin
	t <= t + 1'd1;
	if (t == STATE_LAST) t <= STATE_RAS0;
	if (t == STATE_RAS0 && !init && !port1_active && !need_refresh) t <= STATE_RAS0;
end

// ---------------------------------------------------------------------
// --------------------------- startup/reset ---------------------------
// ---------------------------------------------------------------------

// wait 1ms (32 8Mhz cycles) after FPGA config is done before going
// into normal operation. Initialize the ram in the last 16 reset cycles (cycles 15-0)
reg [4:0]  reset;
reg        init = 1'b1;
always @(posedge clk, negedge init_n) begin
	if(!init_n) begin
		reset <= 5'h1f;
		init <= 1'b1;
	end else begin
		if((t == STATE_LAST) && (reset != 0)) reset <= reset - 5'd1;
		init <= !(reset == 0);
	end
end

// ---------------------------------------------------------------------
// ------------------ generate ram control signals ---------------------
// ---------------------------------------------------------------------

// all possible commands
localparam CMD_INHIBIT         = 4'b1111;
localparam CMD_NOP             = 4'b0111;
localparam CMD_ACTIVE          = 4'b0011;
localparam CMD_READ            = 4'b0101;
localparam CMD_WRITE           = 4'b0100;
localparam CMD_BURST_TERMINATE = 4'b0110;
localparam CMD_PRECHARGE       = 4'b0010;
localparam CMD_AUTO_REFRESH    = 4'b0001;
localparam CMD_LOAD_MODE       = 4'b0000;

reg [3:0]  sd_cmd;   // current command sent to sd ram

// drive control signals according to current command
assign SDRAM_nCS  = sd_cmd[3];
assign SDRAM_nRAS = sd_cmd[2];
assign SDRAM_nCAS = sd_cmd[1];
assign SDRAM_nWE  = sd_cmd[0];

reg [21:1] addr_latch;
reg [15:0] din_latch;
reg        oe_latch;
reg        we_latch;
reg  [1:0] ds;

reg        port1_state;

reg [10:0] refresh_cnt;
wire       need_refresh = (refresh_cnt >= RFRSH_CYCLES);
wire       port1_active = port1_req ^ port1_ack /* synthesis keep */;

always @(posedge clk) begin

	SDRAM_DQ <= 16'bZZZZZZZZZZZZZZZZ;
	{ SDRAM_DQMH, SDRAM_DQML } <= 2'b11;
	sd_cmd <= CMD_NOP;  // default: idle
	refresh_cnt <= refresh_cnt + 1'd1;

	if(init) begin
	   SDRAM_CKE <=1'b1;
		// initialization takes place at the end of the reset phase
		refresh_cnt <= 0;
		if(t == STATE_RAS0) begin

			if(reset == 15) begin
				sd_cmd <= CMD_PRECHARGE;
				{ SDRAM_DQMH, SDRAM_DQML } <= 2'b11;
				//SDRAM_A[10] <= 1'b1;      // precharge all banks
				SDRAM_A <= 12'b1;
				SDRAM_BA <= 2'b00;
			end
			if(reset == 1) begin
				sd_cmd <= CMD_LOAD_MODE;
				SDRAM_A <= MODE;
				//SDRAM_BA <= 2'b00;
			end
			if( reset ==12 || reset ==6) begin
				sd_cmd <= CMD_AUTO_REFRESH;
				SDRAM_CKE <= 1'b0;
			end

		end
	end else begin
		// RAS phase
		// bank 0,1
		SDRAM_CKE <= 1'b1;
		if(t == STATE_RAS0) begin
			{ oe_latch, we_latch } <= 2'b00;

			if (port1_active) begin
				sd_cmd <= CMD_ACTIVE;
				SDRAM_A <= port1_a[19:9];
				SDRAM_BA <= port1_a[21:20];
				addr_latch <= port1_a;
				{ oe_latch, we_latch } <= { ~port1_we, port1_we };
				ds <= port1_ds;
				din_latch <= port1_d;
				port1_state <= port1_req;
			end else if (need_refresh) begin
				sd_cmd <= CMD_AUTO_REFRESH;
				SDRAM_CKE <= 1'b0;
				refresh_cnt <= 0;
			end
		end

		// CAS phase
		if(t == STATE_CAS0 && (we_latch || oe_latch)) begin
			sd_cmd <= we_latch?CMD_WRITE:CMD_READ;
			{ SDRAM_DQMH, SDRAM_DQML } <= ~ds;
			if (we_latch) begin
				SDRAM_DQ <= din_latch;
				port1_ack <= port1_req;
			end
			SDRAM_A <= { 4'b0100, addr_latch[8:1] };  // auto precharge
			
			SDRAM_BA <= addr_latch[21:20];
		end

		// Data returned
		if(t == STATE_READ0 && oe_latch) begin
			port1_q <= SDRAM_DQ;
			port1_ack <= port1_req;
		end

	end
end

endmodule
