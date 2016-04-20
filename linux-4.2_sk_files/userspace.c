#include <stdint.h>
#include <fcntl.h>      /* open */
#include <unistd.h>     /* exit */
#include <sys/ioctl.h>  /* ioctl */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

#define FPGA_REG_STATUS 0x1fffffe // fpga register status register address
#define FPGA_REG_ACTION 0x1fffffc // fpga action register
#define FPGA_REG_TEST 0x1fffffa // fpga test register, 0xbeaf should be read
#define FPGA_REG_RESET 0x1fffff8 // fpga interface reset registert addr
#define FPGA_REG_MODE 0x1fffff6
#define FPGA_REG_ADDRESS_HI 0x1fffff4 // high part of address to write in foga
#define FPGA_REG_ADDRESS_LO 0x1fffff2 // low part of address to write in foga
#define FPGA_REG_DATA_HI 0x1fffff0 // high part of data to be written to fpga
#define FPGA_REG_DATA_LO 0x1ffffee // low part of data to be written to fpga
#define FPGA_REG_RUN 0x1ffffec // start writing
#define FPGA_REG_CPU_STATUS 0x1ffffea // ??
#define FPGA_REG_DATA_READ_HI 0x1ffffe8 // high part of data being read from fpga
#define FPGA_REG_DATA_READ_LO 0x1ffffe6 // low part of data being read from fpga



enum
{
	FPGA_MEM_READ = 0x23,
	FPGA_MEM_WRITE = 0x21,
	FPGA_RESET = 0x3,
	FPGA_PROGRAMM = 0x4,
	FPGA_IFACE_READ_REG = 0x5,
	FPGA_IFACE_WRITE_REG = 0x6,
	FPGA_PROGRAMM_FIN = 0x7,
	FPGA_INIT_SMC = 0x8,
};

typedef struct
{
	uint16_t mode;
	uint16_t address_lo;
	uint16_t address_hi;
	uint16_t data_lo;
	uint16_t data_hi;
} sk_fpga_iface_write;

typedef struct
{
	uint16_t data_read_lo;
	uint16_t data_read_hi;
} sk_fpga_iface_read;

typedef struct
{
	uint32_t addr;
	uint16_t data;
} sk_fpga_single_reg;


void fpga_reset(int file_desc)
{
	int ret_val;
	ret_val = ioctl(file_desc, FPGA_RESET, NULL);
}

void fpga_prog(int file_desc)
{
	int ret_val;
	ret_val = ioctl(file_desc, FPGA_PROGRAMM, NULL);
}

void fpga_prog_fin(int file_desc)
{
	printf("CALLING FIN USERSPACE\n");
	int ret_val;
	ret_val = ioctl(file_desc, FPGA_PROGRAMM_FIN, NULL);
}

void fpga_prog_init_smc(int file_desc)
{
	printf("CALLING FIN INIT SMC\n");
	int ret_val;
	ret_val = ioctl(file_desc, FPGA_INIT_SMC, NULL);
}

void fpga_write_mem(int file_desc, uint32_t addr, uint32_t data, uint16_t mode)
{

	int ret_val;
	sk_fpga_iface_write wr_data;
	wr_data.mode = mode;
	wr_data.address_hi = addr >> 16;
	wr_data.address_lo = addr & 0xffff;
	wr_data.data_hi = data >> 16;
	wr_data.data_lo = data & 0xffff;
	ret_val = ioctl(file_desc, FPGA_MEM_WRITE, &wr_data);
	//ret_val = ioctl(file_desc, FPGA_MEM_WRITE, NULL);
	printf("Writing to mem %x\n", ret_val);
}

uint32_t fpga_read_mem(int file_desc, uint32_t addr, uint32_t data, uint16_t mode)
{
	int ret_val;
	sk_fpga_iface_write rd_data;
	rd_data.mode = mode;
	rd_data.address_hi = addr >> 16;
	rd_data.address_lo = addr & 0xffff;
	rd_data.data_hi = data >> 16;
	rd_data.data_lo = data & 0xffff;
	ret_val = ioctl(file_desc, FPGA_MEM_READ, &rd_data);
	sk_fpga_iface_read* answer = (sk_fpga_iface_read*)(&rd_data);
	return (uint32_t)(answer->data_read_lo | (answer->data_read_hi << 16));
}

void fpga_write_reg(int file_desc, uint32_t addr, uint16_t data)
{
	int ret_val;
	sk_fpga_single_reg data_wr;
	data_wr.addr = addr;
	data_wr.data = data;
	ret_val = ioctl(file_desc, FPGA_IFACE_WRITE_REG, &data_wr);
}

uint16_t fpga_read_reg(int file_desc, uint32_t addr)
{
	int ret_val;
	sk_fpga_single_reg data;
	data.addr = addr;
	ret_val = ioctl(file_desc, FPGA_IFACE_READ_REG, &data);
	return data.data;
}

uint16_t fpga_read_test(int file_desc)
{
	return fpga_read_reg(file_desc, FPGA_REG_TEST);
}

void fpga_prg(const char* fileName, int fpgaFd)
{


	//int fpgaFd = open("/dev/fpga", 0);
	fpga_prog(fpgaFd);

	FILE *f = fopen(fileName, "rb");
	fseek(f, 0, SEEK_END);
	long fsize = ftell(f);
	fseek(f, 0, SEEK_SET);
	char *string = malloc(fsize + 1);
	fread(string, fsize, 1, f);
	int i = 0;
	int size = 4096;
	for ( i = 0; i < fsize; i = i + 4096)
	{
		if ( (i + size) > fsize )
		{
			size = fsize - i;
		}
		write(fpgaFd, string + i, size);
		fsync(fpgaFd);
	}

	free(string);
	fclose(f);
	fpga_prog_fin(fpgaFd);
}

int main()
{



	int file_desc, ret_val;

	file_desc = open("/dev/fpga", O_WRONLY);
	if (file_desc < 0) {
		printf ("Can't open device file: %s\n", "/dev/fpga");
		exit(-1);
	}




	fpga_prg("./top.bit", file_desc);

fpga_write_mem(file_desc, 5, 0xdeadb00f, 2);
fpga_write_mem(file_desc, 0, 0xaaaaaaaa, 2);
printf("READ MEM US %x\n", fpga_read_mem(file_desc, 5, 0, 2));
printf("READ MEM US %x\n", fpga_read_mem(file_desc, 0, 0, 2));
//printf("READ REG US %x\n", fpga_read_reg(file_desc, FPGA_REG_DATA_READ_LO));


	//fpga_prog_init_smc(file_desc);
	//sleep(2);
	//fpga_reset(file_desc);


	//printf("MODE %x\n", fpga_read_reg(file_desc, FPGA_REG_MODE));



	close(file_desc);
	return 0;
}
