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

#define MODE_NONE 0 // no
#define MODE_IM 1 // choose im
#define MODE_DM 2 // choose dm
#define MODE_REG 3 // choose reg file
#define MODE_RAM 4 // choose ram




#define STATUS 0x1fffffe
#define ACTION 0x1fffffc
//#define ACTIVATE 0x1fffffa
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

//#define WRITE_REG(phys_reg, value) (iowrite32(value, (void*)phys_reg))
//#define READ_REG(phys_reg) (ioread32((void*)phys_reg))

//#define CALC_ADDR(addr) ((uint32_t*)(my_fpga.fpga_mem + addr))



typedef enum
{

} sk_fpga_ioctl_command;


int sk_fpga_fill_structure(struct platform_device *pdev);
static int sk_fpga_remove (struct platform_device *pdev);
static int sk_fpga_probe (struct platform_device *pdev);
static int sk_fpga_open(struct inode *inode, struct file *file);
static int sk_fpga_release(struct inode *inode, struct file *file);
static ssize_t sk_fpga_write(struct file *filp, const char *buff, size_t len, loff_t *off);
void sk_fpga_prog(const unsigned char* buff, unsigned int bufLen);
static ssize_t sk_fpga_read(struct file *filp, char *buffer, size_t length, loff_t *offset);
int sk_fpga_reg_interrupt(unsigned pin);
void sk_fpga_unreg_interrupt(void);
irqreturn_t sk_fpga_interrupt_handler(int irq, void *dev_id);
void sk_fpga_write_command(void);
void sk_fpga_read_command(void);

static long sk_fpga_ioctl(struct file *file, unsigned int ioctl_num, unsigned long ioctl_arg);
int sk_fpga_setup_smc0(void);

