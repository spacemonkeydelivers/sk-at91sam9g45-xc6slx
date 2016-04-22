`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
//
//////////////////////////////////////////////////////////////////////////////////

// data sizes
`define BYTE 7
`define SIZE_BYTE `BYTE:0
`define HALF 15
`define SIZE_HALF `HALF:0
`define WORD 31
`define SIZE_WORD `WORD:0

// magic constants used in iface
`define DATA_MAGIC						16'hDA1A // magic constant to be read from sanity reg
`define DATA_ERROR						16'h0E12 // magic constant if read means something went wrong and you should check status reg for more info
`define CHIP_SELECT_LOW_TO_HIGH			2'b01 // chip select pin switches from low to high

// bunch of states iface could be in
`define STATUS_IDLE							0 // iface is in idle state, nothing bad happened
`define STATUS_BUSY 						1 // iface is busy reading or writing data
`define STATUS_DONE 						2 // iface is done doing previous stuff it supposed to do
`define STATUS_ERROR_WRONG_ADDR_RD 			3 // iface has encountered error by reading wrong addr
`define STATUS_ERROR_WRONG_ADDR_WR 			4 // iface has encountered error by writing to wrong addr
`define STATUS_ERROR_WRONG_MEM_ACTION_SET	5 // tried to interact with mem but wrong action type(rd/wr) is set
`define STATUS_ERROR_WRONG_MEM_TYPE_SET		6 // tried to interact with mem but wrong mem type(im, dm, regfile) is set
`define STATUS_ERROR_HIT_WHILE_BUSY			7 // tried to run interaction while busy

// irq pin states
`define IRQ_CLEAR						0 // raise interrupt flag
`define IRQ_RAISE						1 // clear interrupt flag
`define BUSY_CLEAR						0 // clear busy flag
`define BUSY_SET						1 // set busy flag

// actions to do with mem
`define MEM_ACTION_NONE					0 // do nothing with mem
`define MEM_ACTION_READ					1 // read mem by iface
`define MEM_ACTION_WRITE				2 // write mem by iface

// purpose of parts of regulatory register
`define REGULATORY_MEM_ACTION			1:0 // stores memory action: none, rd, wr
`define REGULATORY_MEM_TYPE				3:2 // stores memory type: none, main

// type of memory to work with
`define MEM_TYPE_NONE					0 // no memory selected to interact with iface
`define MEM_TYPE_MAIN					1 // main memory is selected to interact with iface
`define MEM_TYPE_DATA					2 // data memory is selected to interact with iface

// addresses of the iface to work with
`define ADDR_STATUS_REG					24'hffffff // reads status register which stores current iface state / RO / +
`define ADDR_SANITY_REG					24'hfffffe // by reading this you should read MAGIC_DATA / RO / +
`define ADDR_IFACE_RESET_REG			24'hfffffd // resets iface state and inner registers / WO / +
`define ADDR_REGULATORY_LO_REG			24'hfffffc // low part of regulatory register of iface / WRRD / +
`define ADDR_REGULATORY_HI_REG			24'hfffffb // high part of regulatory register of iface / WRRD / +
`define ADDR_ADDRESS_LO_REG				24'hfffffa // low part of address for iface to work with / WRRD / +
`define ADDR_ADDRESS_HI_REG				24'hfffff9 // high part of address for iface to work with / WRRD / +
`define ADDR_DATA_LO_REG				24'hfffff8 // low part of data for iface to work with / WRRD / +
`define ADDR_DATA_HI_REG				24'hfffff7 // high part of data for iface to work with / WRRD / +
`define ADDR_RESULT_LO_REG				24'hfffff6 // low part of result data from iface / RO / +
`define ADDR_RESULT_HI_REG				24'hfffff5 // high part of result data from iface / RO / +
`define ADDR_RUN_MEM_INTERACT_REG		24'hfffff4 // writing to this address starts iface interaction with memory / WO / +

module data_memory(
	input	wire	[`SIZE_WORD]		address_i,		// input address bus
	input	wire	[`SIZE_WORD]		data_i,			// input data bus
	input	wire						write_data_i,	// input write strobe
	input	wire						read_data_i,	// input read strobe
	input	wire						clock_i,		// input clk source
	output	wire	[`SIZE_WORD]		read_data_o		// output data bus
	);

	parameter ENTRY_NUM = 128;
	parameter ENTRY_NUM_LOG2 = 7;
	
	reg [31:0] inner_memory [0:ENTRY_NUM-1]; // inner mem to store values
	reg [31:0] reg_out = 0;
	assign read_data_o = reg_out;

	always @ ( posedge clock_i )
	begin
		// if write strobe is high and read is low
		if (write_data_i == 1 && read_data_i == 0)
		begin
			inner_memory[address_i[(ENTRY_NUM_LOG2-1+2):2]] <= data_i;
		end
		// if write strobe is low and read is high
		if (write_data_i == 0 && read_data_i == 1)
		begin
			reg_out <= inner_memory[address_i[(ENTRY_NUM_LOG2-1+2):2]];
		end
	end
endmodule


module top(
	inout wire [`SIZE_HALF] data_io,	// input-output data bus
	input wire [24:0] addr_i,			// input address bus
	input wire read_i,					// input read strobe
	input wire write_i,					// input write strobe
	input wire cs_i,					// input chip select strobe
	input wire clk_i,					// input clk signal
	input wire reset_i,					// external reset signal, if low - reset
	output wire irq_o					// output interrupt pin
	);
	
	// latches to avoid metastability
	reg stage_1 = 0;
	reg stage_2 = 0;
	reg stage_3 = 0;
	
	//signal which controls tristate iobuf
	wire disable_io;
	assign disable_io = (read_i);
	
	// to deal with external io data bus
	wire [`SIZE_BYTE] data_write;
	reg [`SIZE_BYTE] data_read = 0;	
	
	// state registers
	reg [`SIZE_HALF] status_reg = `STATUS_IDLE; // stores current iface status
	reg [`SIZE_HALF] regulatory_lo_reg = 0; // stores low part of control bits of iface
	reg [`SIZE_HALF] regulatory_hi_reg = 0; // stores high part of control bits of iface
	reg [`SIZE_HALF] address_lo_reg = 0; // stores low part of address for iface to work with
	reg [`SIZE_HALF] address_hi_reg = 0; // stores high part of address for iface to work with
	reg [`SIZE_HALF] data_lo_reg = 0; // stores low part of data for iface to work with
	reg [`SIZE_HALF] data_hi_reg = 0; // stores low part of data for iface to work with
	reg [`SIZE_HALF] result_lo_reg = 0; // stores low part of result data from iface 
	reg [`SIZE_HALF] result_hi_reg = 0; // stores high part of result data from iface 
	wire [`SIZE_WORD] data_full_reg = {data_hi_reg, data_lo_reg}; // full version of data for iface to work with some mem
	wire [`SIZE_WORD] addr_full_reg = {addr_hi_reg, data_lo_reg}; // full version of address for iface to work with some mem
	wire [`SIZE_WORD] result_full_reg = {result_hi_reg, result_lo_reg}; // full version of result of iface interacting with some mem
	wire [`SIZE_WORD] regulatory_full_reg = {regulatory_hi_reg, regulatory_lo_reg}; // full version of regulatory register of iface
	reg busy_flag_reg = 0; // shows if iface is busy at the moment
	reg sys_read_en = 0; // read enable reg for mem to work with, should be set when read address is set
	reg sys_write_en = 0; // write enable reg for mem to work with, should be set when write address and data are set
	reg	[`SIZE_BYTE] counter = 0; // internal counter reg
	
	reg irq_reg = `IRQ_CLEAR; // register to store irq status
	assign irq_o = irq_reg;
	
	// iobuf instance
	genvar y;
	generate
	for(y = 0; y < 16; y = y + 1 ) 
	begin : iobuf_generation
		IOBUF io_y (
			.I( data_read[y] ),
			.O( data_write[y] ),
			.IO( data_io[y] ),
			.T ( disable_io )
		);
	end
	endgenerate

	// wires to connect 2 memories and iface
	wire read_strobe_mem_main = (regulatory_full_reg[`REGULATORY_MEM_TYPE] == `MEM_TYPE_MAIN) ? sys_read_en : 0;
	wire read_strobe_mem_data = (regulatory_full_reg[`REGULATORY_MEM_TYPE] == `MEM_TYPE_DATA) ? sys_read_en : 0;
	wire write_strobe_mem_main = (regulatory_full_reg[`REGULATORY_MEM_TYPE] == `MEM_TYPE_MAIN) ? sys_write_en : 0;
	wire write_strobe_mem_data = (regulatory_full_reg[`REGULATORY_MEM_TYPE] == `MEM_TYPE_DATA) ? sys_write_en : 0;
	wire [`SIZE_WORD] result_data_mem_main;
	wire [`SIZE_WORD] result_data_mem_data;
	wire [`SIZE_WORD] result_data;
	assign result_data = (regulatory_full_reg[`REGULATORY_MEM_TYPE] == `MEM_TYPE_MAIN) ? 
							result_data_mem_main : (regulatory_full_reg[`REGULATORY_MEM_TYPE] == `MEM_TYPE_DATA) ? 
														result_data_mem_data : 0;

	// instance and connect 2 mems
	data_memory #() MAIN_MEM(
		.address_i(addr_full_reg),
		.data_i(data_full_reg),
		.write_data_i(write_strobe_mem_main),
		.read_data_i(read_strobe_mem_main),
		.clock_i(clk_i),
		.read_data_o(result_data_mem_main)
	);
	
	data_memory #() DATA_MEM(
		.address_i(addr_full_reg),
		.data_i(data_full_reg),
		.write_data_i(write_strobe_mem_data),
		.read_data_i(read_strobe_mem_data),
		.clock_i(clk_i),
		.read_data_o(result_data_mem_data)
	);

	// becomes true if chip select was switched from low to high
	wire iface_accessed = = {stage_2, stage_3} == `CHIP_SELECT_LOW_TO_HIGH;
	// becomes true if memory action is not none
	wire mem_action_not_none = regulatory_full_reg[`REGULATORY_MEM_ACTION] != `MEM_ACTION_NONE;
	// becomes true if memory type is not none
	wire mem_type_not_none = regulatory_full_reg[`REGULATORY_MEM_TYPE] != `MEM_TYPE_NONE;
	// becomes true if memory action and type are set and we're not busy
	wire run_mem_interact = mem_action_not_none && mem_type_not_none && !busy_flag_reg;
	// becomes true if memory action type is read
	wire mem_action_read = regulatory_full_reg[`REGULATORY_MEM_ACTION] == `MEM_ACTION_READ;
	// becomes true if memory action type is write
	wire mem_action_write = regulatory_full_reg[`REGULATORY_MEM_ACTION] == `MEM_ACTION_WRITE;
	// TODO: fix counter value
	// becomes true if mem read action is set and counter is reached limit
	wire stop_mem_read = mem_action_read && counter == 2;
	// becomes true if mem write action is set and counter is reached limit
	wire stop_mem_write = mem_action_write && counter == 2;

	// store chipselect stuff into latches
	always @ (negedge clk_i or negedge reset_i)
	begin
		if (!reset_i)
		begin
			status_reg <= `STATUS_IDLE;
			irq_reg <= `IRQ_CLEAR;						
			regulatory_lo_reg <= 0;
			regulatory_hi_reg <= 0;
			address_hi_reg <= 0;
			address_lo_reg <= 0;
			data_hi_reg <= 0;
			data_lo_reg <= 0;
			result_hi_reg <= 0;
			result_lo_reg <= 0;
			busy_flag_reg <= 0;
			sys_read_en <= 0;
			sys_write_en <= 0;
			counter <= 0;
			stage_1 <= 0;
			stage_2 <= 0;
			stage_3 <= 0;
		end
		else
		begin
			stage_2 <= stage_1;
		end
	end

	always @ (posedge clk_i or negedge reset_i) 
	begin
		// external is triggered, clear inner state
		if (!reset_i)
		begin
			status_reg <= `STATUS_IDLE;
			irq_reg <= `IRQ_CLEAR;						
			regulatory_lo_reg <= 0;
			regulatory_hi_reg <= 0;
			address_hi_reg <= 0;
			address_lo_reg <= 0;
			data_hi_reg <= 0;
			data_lo_reg <= 0;
			result_hi_reg <= 0;
			result_lo_reg <= 0;
			busy_flag_reg <= 0;
			sys_read_en <= 0;
			sys_write_en <= 0;
			counter <= 0;
			stage_1 <= 0;
			stage_2 <= 0;
			stage_3 <= 0;
		end
		else
		begin
			// store chipselect stuff into latches
			stage_3 <= stage_2;
			stage_1 <= cs_i;
			if (iface_accessed)
			begin
				// read from fpga routine
				if (!read_i)
				begin
					case ( addr_i[24:1] )
						// reads magic constant, just sanity check
						`ADDR_SANITY_REG:
						begin
							data_read <= `DATA_MAGIC;
						end
						// reads iface status register and clears interrupt flag
						`ADDR_STATUS_REG:
						begin
							data_read <= status_reg;
							irq_reg <= `IRQ_CLEAR;
						end
						// reads low part of regulatory register of iface
						`ADDR_REGULATORY_LO_REG:
						begin
							data_read <= regulatory_lo_reg;
						end
						// reads high part of regulatory register of iface
						`ADDR_REGULATORY_HI_REG:
						begin
							data_read <= regulatory_hi_reg;
						end
						// reads high part of address for iface to work with some mem
						`ADDR_ADDRESS_HI_REG:
						begin
							data_read <= address_hi_reg;
						end
						// reads low part of address for iface to work with some mem
						`ADDR_ADDRESS_LO_REG:
						begin
							data_read <= address_lo_reg;
						end
						// reads high part of data to be written to some mem with iface
						`ADDR_DATA_HI_REG:
						begin
							data_read <= data_hi_reg;
						end
						// reads low part of data to be written to some mem with iface
						`ADDR_DATA_LO_REG:
						begin
							data_read <= data_lo_reg;
						end
						// reads high part of result of reading some mem with iface
						`ADDR_RESULT_HI_REG:
						begin
								data_read <= result_hi_reg;
						end
						// reads low part of result of reading some mem with iface
						`ADDR_RESULT_LO_REG:
						begin
							data_read <= result_lo_reg;
						end
						// if read to wrong address happens, set status and raise interrupt
						default:
						begin
							data_read <= `DATA_ERROR;
							status_reg <= `STATUS_ERROR_WRONG_ADDR_RD;
							irq_reg <= `IRQ_RAISE;
						end
					endcase
				end
				// write to fpga routine
				if (!write_i)
				begin
					case ( addr_i[24:1] )
						// writing to this address starts iface interaction with memory
						`ADDR_RUN_MEM_INTERACT_REG:
						begin
							// if action(read/write) is not set to none and mode(type of mem: im, dm, regfile) is not set to none and we're not busy
							if (run_mem_interact)
							begin
								// set busy status
								status_reg <= `STATUS_BUSY;
								busy_flag_reg <= `BUSY_SET;
								counter <= 0;
								// set read enable for mem interacting with
								if (mem_action_read)
								begin
									sys_read_en <= 1;
								end
								// set write enable for mem interacting with
								else if (mem_action_write)
								begin
									sys_write_en <= 1;
								end
								else
								// report error: wrong mem action set
								begin
									status_reg <= `STATUS_ERROR_WRONG_MEM_ACTION_SET;
									irq_reg <= `IRQ_RAISE;
								end
							end
							// something is set wrong or we're busy: report error
							else
							begin
								irq_reg <= `IRQ_RAISE;
								if (busy_flag_reg)
								begin
									status_reg <= `STATUS_ERROR_HIT_WHILE_BUSY;
								end
								else if (!mem_action_not_none)
								begin
									status_reg <= `STATUS_ERROR_WRONG_MEM_ACTION_SET;
								end
								else if (!mem_type_not_none)
								begin
									status_reg <= `STATUS_ERROR_WRONG_MEM_TYPE_SET;
								end
								else
								begin
								end
							end
						end
						// writes high part of address for iface to work with some mem
						`ADDR_ADDRESS_HI_REG:
						begin
							address_hi_reg <= data_write;
						end
						// writes low part of address for iface to work with some mem
						`ADDR_ADDRESS_LO_REG:
						begin
							address_lo_reg <= data_write;
						end
						// writes high part of data to be written with iface into some mem
						`ADDR_DATA_HI_REG:
						begin
							data_hi_reg <= data_write;
						end
						// writes low part of data to be written with iface into some mem
						`ADDR_DATA_LO_REG:
						begin
							data_lo_reg <= data_write;
						end
						// writes low part of regulatory register of iface
						`ADDR_REGULATORY_LO_REG:
						begin
							regulatory_lo_reg <= data_write;
						end
						// writes high part of regulatory register of iface
						`ADDR_REGULATORY_HI_REG:
						begin
							regulatory_hi_reg <= data_write;
						end
						// reset iface inner state and regs
						`ADDR_IFACE_RESET_REG:
						begin
							status_reg <= `STATUS_IDLE;
							irq_reg <= `IRQ_CLEAR;						
							regulatory_lo_reg <= 0;
							regulatory_hi_reg <= 0;
							address_hi_reg <= 0;
							address_lo_reg <= 0;
							data_hi_reg <= 0;
							data_lo_reg <= 0;
							result_hi_reg <= 0;
							result_lo_reg <= 0;
							busy_flag_reg <= 0;
							sys_read_en <= 0;
							sys_write_en <= 0;
							counter <= 0;
						end
						// if write to wrong address happens, set status and raise interrupt
						default:
						begin
							status_reg <= `STATUS_ERROR_WRONG_ADDR_WR;
							irq_reg <= `IRQ_RAISE;
						end
					endcase
				end
			end
			// fpga action routine
			// while busy flag is set run actio routine
			// TODO: check if counter should be 1?
			if (busy_flag_reg == 1)
			begin
				counter <= counter + 1;
				// clear read enable strobe on second tick and read result data
				if (stop_mem_read)
				begin
					{result_hi_reg, result_lo_reg} <= result_data;
					sys_read_en <= 0;
				end			
				// clear write enable strobe on second tick
				else if (stop_mem_write)
				begin
					sys_write_en <= 0;
				end
				else
				begin
				end
				// set finish status
				if (counter == 2)
				begin
					status_reg <= `STATUS_DONE;
					busy_flag_reg <= `BUSY_CLEAR;
					irq_reg <= `IRQ_RAISE;
				end
			end
		end
	end
endmodule


