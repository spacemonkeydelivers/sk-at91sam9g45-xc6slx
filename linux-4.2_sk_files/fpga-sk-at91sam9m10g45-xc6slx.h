

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

#define SMC 0xFFFFE800
#define SMC_SETUP(num) (0x10 * num + 0x00)
#define SMC_PULSE(num) (0x10 * num + 0x04)
#define SMC_CYCLE(num) (0x10 * num + 0x08)
#define SMC_MODE(num) (0x10 * num + 0x0C)
#define SMC_DELAY1 0xC0
#define SMC_DELAY2 0xC4
#define SMC_DELAY3 0xC8
#define SMC_DELAY4 0xCC
#define SMC_DELAY5 0xD0
#define SMC_DELAY6 0xD4
#define SMC_DELAY7 0xD8
#define SMC_DELAY8 0xDC





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
	unsigned char init;
};


void prog_fpga(unsigned char* buff, unsigned int bufLen);
