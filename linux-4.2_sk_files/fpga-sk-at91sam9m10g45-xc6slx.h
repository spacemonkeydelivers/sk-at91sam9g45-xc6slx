

#define STATUS 0x1fffffe
#define ACTION 0x1fffffc
#define ACTIVATE 0x1fffffa
#define RESET 0x1fffff8
#define MODE 0x1fffff6
#define ADDRESS_HI 0x1fffff4
#define ADDRESS_LO 0x1fffff2
#define DATA_HI 0x1fffff0
#define DATA_LO 0x1ffffee
#define RUN 0x1ffffec
#define CPU_STATUS 0x1ffffea
#define DATA_READ_HI 0x1ffffe8
#define DATA_READ_LO 0x1ffffe6


struct fpga
{
	unsigned char* device_name;
	struct platform_device *pdev;	
	unsigned int fpga_mem_phys_start;
	unsigned char* fpga_mem;
	unsigned int fpga_mem_size;
	unsigned char fpga_open;
	unsigned char fpga_irq_pin;
	unsigned char fpga_cclk;
	unsigned char fpga_din;
	unsigned char fpga_done;
	unsigned char fpga_prog;
};
