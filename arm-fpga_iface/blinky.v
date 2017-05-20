`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date:    22:53:55 05/19/2017 
// Design Name: 
// Module Name:    blinky 
// Project Name: 
// Target Devices: 
// Tool versions: 
// Description: 
//
// Dependencies: 
//
// Revision: 
// Revision 0.01 - File Created
// Additional Comments: 
//
//////////////////////////////////////////////////////////////////////////////////
`define CHIP_SELECT_LOW_TO_HIGH			2'b01 // chip select pin switches from low to high

module blinky(
    inout wire [15:0] data_io,		// input-output data bus
    input wire [24:0] addr_i,			// input address bus
    input wire read_i,					// input read strobe
    input wire write_i,					// input write strobe
    input wire cs_i,						// input chip select strobe
    input wire clk_i,					// input clk signal
    input wire reset_i,					// external reset signal, if low - reset
	 output wire [4:0] leds_o			// leds out
);

	// latches to avoid metastability
	reg stage_1 = 0;
	reg stage_2 = 0;
	reg stage_3 = 0;
	
	//signal which controls tristate iobuf
	wire disable_io;
	assign disable_io = (read_i);
	
	// to deal with external io data bus
	wire [15:0] data_to_iface;
	reg [15:0] data_from_iface = 0;
	
	reg [15:0] stored_data = 16'hDEFA;
	
	// state registers
	reg	[32:0] counter = 0; // internal counter reg

	assign leds_o[0] = 1;
	assign leds_o[1] = counter[29];
	assign leds_o[2] = counter[30];
	assign leds_o[3] = counter[31];
	assign leds_o[4] = counter[32];

	// iobuf instance
	genvar y;
	generate
	for(y = 0; y < 16; y = y + 1 ) 
	begin : iobuf_generation
		IOBUF io_y (
			.I( data_from_iface[y] ),
			.O( data_to_iface[y] ),
			.IO( data_io[y] ),
			.T ( disable_io )
		);
	end
	endgenerate

	// becomes true if chip select was switched from low to high
	wire iface_accessed = {stage_2, stage_3} == `CHIP_SELECT_LOW_TO_HIGH;

	always @ (posedge clk_i) 
	begin
		// external is triggered, clear inner state
		if (!reset_i)
		begin
			counter <= 0;
			stage_3 <= 0;
			stage_2 <= 0;
			stage_1 <= 0;
			data_from_iface <= 0;
			stored_data <= 0;
		end
		else
		begin
			counter <= counter + 1;
			// store chipselect stuff into flip-flops
			stage_3 <= stage_2;
			stage_2 <= stage_1;
			stage_1 <= cs_i;
			if (iface_accessed)
			begin
				// put data from fpga
				if (!read_i)
				begin
					data_from_iface <= stored_data;
				end
				// get data to fpga
				if (!write_i)
				begin
					stored_data <= data_to_iface;
				end
			end
		end
	end
endmodule
