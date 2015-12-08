#ifndef SK_FPGA_DRIVER_HEADER
#define SK_FPGA_DRIVER_HEADER

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <generated/utsrelease.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/wait.h>

#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/ioctl.h>
#include <linux/platform_data/atmel.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <linux/suspend.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/of_gpio.h>

#define ACTION_NONE 0 // no action selected
#define ACTION_READ 1 // read data
#define ACTION_WRITE 2 // write data

#define FPGA_MODE_NONE 0 // no
#define FPGA_MODE_IM 1 // choose im
#define FPGA_MODE_DM 2 // choose dm
#define FPGA_MODE_REG 3 // choose reg file
#define FPGA_MODE_RAM 4 // choose ram




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

#define SK_FPGA_WRITE_REG(a) *((uint16_t*)(my_fpga.fpga_mem_virt_start_cs0 + a))

#define CALC_ADDR(addr) (my_fpga.fpga_mem_virt_start_cs0 + addr)
#define WRITE_REG(phys_reg, value) (iowrite16(value, (void*)CALC_ADDR(phys_reg)))
#define READ_REG(phys_reg) (ioread16((void*)CALC_ADDR(phys_reg)))





typedef enum
{
	SK_FPGA_NO_STATUS = 0x0,
	SK_FPGA_PROGRAMMING = 0x1,
	SK_FPGA_SMC0_SET = 0x2,
	SK_FPGA_CLOCK_STARTED = 0x4,
	SK_FPGA_IRQ_REGISTERED = 0x8,
} sk_fpga_status;

//
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

struct sk_fpga
{
	struct platform_device *pdev;
	unsigned int fpga_mem_phys_start_cs0; // phys mapped addr of fpga mem on cs0
	unsigned int fpga_mem_phys_start_cs1; // phys mapped addr of fpga mem on cs1
	unsigned char *fpga_mem_virt_start_cs0; // virt mapped addr of fpga mem on cs0
	unsigned char *fpga_mem_virt_start_cs1; // virt mapped addr of fpga mem on cs0
	unsigned int fpga_mem_size; // phys mem size on any cs pin
	unsigned char fpga_open_counter; // number of fpga openings
	unsigned char fpga_irq_pin; // ping to recieve irq from fpga on arm
	unsigned char fpga_cclk; // pin to run cclk on fpga
	unsigned char fpga_din; // pin to set data to fpga
	unsigned char fpga_done; // pin to read status done from fpga
	unsigned char fpga_prog; // pin to set mode to prog on fpga
	unsigned int fpga_clk_rate; // clock rate for fpga in mhz
	unsigned int fpga_data_ready; // turns 1 when data is ready
	wait_queue_head_t fpga_wait_queue;
	int fpga_irq_num;
	sk_fpga_status fpga_status; // status of fpga
	struct clk* fpga_clk; // clock for fpga
};

enum
{
	FPGA_MEM_READ = 0x1,
	FPGA_MEM_WRITE = 0x2,
	FPGA_RESET = 0x3,
	FPGA_PROGRAMM = 0x4,
	FPGA_IFACE_READ_REG = 0x5,
	FPGA_IFACE_WRITE_REG = 0x6,
};




static int sk_fpga_remove (struct platform_device *pdev);
static int sk_fpga_probe (struct platform_device *pdev);
static int sk_fpga_open(struct inode *inode, struct file *file);
static int sk_fpga_release(struct inode *inode, struct file *file);
static ssize_t sk_fpga_write(struct file *filp, const char *buff, size_t len, loff_t *off);
static ssize_t sk_fpga_read(struct file *filp, char *buffer, size_t length, loff_t *offset);
static long sk_fpga_ioctl(struct file *file, unsigned int ioctl_num, unsigned long ioctl_arg);

// write a portion of data to fpga
void sk_fpga_prog(const unsigned char* buff, unsigned int bufLen);
// init fpga structure
int sk_fpga_fill_structure(struct platform_device *pdev);


int sk_fpga_reg_interrupt(unsigned pin);
void sk_fpga_unreg_interrupt(void);
irqreturn_t sk_fpga_interrupt_handler(int irq, void *dev_id);

// setup smc bus for cs0
int sk_fpga_setup_smc0(void);

int sk_fpga_start_clk(struct platform_device *pdev);

#endif
