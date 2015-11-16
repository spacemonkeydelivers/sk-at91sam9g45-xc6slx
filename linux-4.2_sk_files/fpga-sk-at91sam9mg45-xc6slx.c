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
#include <linux/clk.h>
//#include <linux/ioport.h>


#define WRITE_REG(phys_reg, value) (iowrite32(value, (void*)phys_reg))
#define READ_REG(phys_reg) (ioread32((void*)phys_reg))

#define AT91_MATRIX 0xFFFFEA00
#define CCFG_EBICSA (AT91_MATRIX + 0x0128)

#define EBI_CS1A 0x1
#define EBI_DRIVE_LOW 0x10
#define EBI_DRIVE_HIGH 0x11

#define CSS 0x0
#define PRES 0x2
#define SCLKMCK 0x8

#define PMC 0xFFFFFC00
#define PMC_SCER (PMC + 0x0000)
#define PMC_SCDR (PMC + 0x0004)
#define PMC_SCSR (PMC + 0x0008)
#define PMC_PCK0 (PMC + 0x0040)
#define PMC_SR (PMC + 0x0068)
#define PCK0 0x8

#define PIO_PER 0x0000
#define PIO_PDR 0x0004
#define PIO_PSR 0x0008
#define PIO_OER 0x0010
#define PIO_ODR 0x0014
#define PIO_OSR 0x0018
#define PIO_IFER 0x0020
#define PIO_IFDR 0x0024
#define PIO_IFSR 0x0028
#define PIO_SODR 0x0030
#define PIO_CODR 0x0034
#define PIO_ODSR 0x0038
#define PIO_PDSR 0x003C
#define PIO_IER 0x0040
#define PIO_IDR 0x0044
#define PIO_IMR 0x0048
#define PIO_ISR 0x004C
#define PIO_MDER 0x0050
#define PIO_MDDR 0x0054
#define PIO_MDSR 0x0058
#define PIO_PUDR 0x0060
#define PIO_PUER 0x0064
#define PIO_PUSR 0x0068
#define PIO_ASR 0x0070
#define PIO_BSR 0x0074
#define PIO_ABSR 0x0078
#define PIO_OWER 0x00A0
#define PIO_OWDR 0x00A4
#define PIO_OWSR 0x00A8
#define PIO_DELAY0R 0x00C0
#define PIO_DELAY1R 0x00C4
#define PIO_DELAY2R 0x00C8
#define PIO_DELAY3R 0x00CC
#define PIO_WPMR 0x00E4
#define PIO_WPSR 0x00E8

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


#define SET_BIT(var, pos) (var |= 1 << pos)
#define CLEAR_BIT(var, pos) (var &= ~(1 << pos))
#define TOGGLE_BIT(var, pos) (var ^= 1 << pos)
#define CHECK_BIT(var, pos) (var & (1 << pos))

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



/*
typedef struct _AT91_SMC_Regs
{
unsigned int  SETUP;
unsigned int  PULSE;
unsigned int  CYCLE;
unsigned int  MODE;
unsigned int  DELAY1;
unsigned int  DELAY2;
unsigned int  DELAY3;
unsigned int  DELAY4;
unsigned int  DELAY5;
unsigned int  DELAY6;
unsigned int  DELAY7;
unsigned int  DELAY8;
} AT91_SMC_Regs;

unsigned short *my_map(unsigned int piobase)
{
    int fd;
    void *base;

//    AT91S_PIO *pio;
    unsigned short *pio;

    off_t addr = piobase;

    if ((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) {
        fprintf(stderr, "Cannot open /dev/mem.\n");
        exit(EXIT_FAILURE);
    }

//    fprintf(stderr, "/dev/mem opened.\n");

    base =
        mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
             addr & ~MAP_MASK);

    if (base == (void *) -1) {
        fprintf(stderr, "Cannot open /dev/mem.\n");
        exit(EXIT_FAILURE);
    }

//    fprintf(stderr, "Memory mapped at address %p.\n", base);

    pio = (unsigned short *) ((uint32_t)base + (addr & MAP_MASK));
    return pio;
}

void main()
{
unsigned short *CS0_ptr, *cs0_tmp;
unsigned short *CS1_ptr;
unsigned char *AddrBitTest_ptr;
unsigned short d1,d2,d3,d4;
AT91_SMC_Regs *SMC0_Reg;
AT91_SMC_Regs *SMC1_Reg;
unsigned short csum,rcsum,i,tmp,error;
volatile unsigned short tmp2;
unsigned long bit,tmp1,tmp3;

	//SMC0_Reg = my_map(0xFFFFe800);
	//SMC0_Reg->SETUP = 0x05050505;
	//SMC0_Reg->PULSE = 0x05050505;
	//SMC0_Reg->CYCLE = 0x000A000A;
	//SMC0_Reg->MODE = 0x3 | 1<<12;
	
	SMC0_Reg = my_map(0xFFFFe800);
	SMC0_Reg->SETUP = 0x01010101;
	SMC0_Reg->PULSE = 0x0e0e0e0e;
	SMC0_Reg->CYCLE = 0x000f000f;
	SMC0_Reg->MODE = 0x3 | 1<<12;
*/



/*

#setup
NCS_RD_SETUP
NRD_SETUP
NCS_WR_SETUP
NWE_SETUP

#pulse
NCS_RD_PULSE
NRD_PULSE
NCS_WR_PULSE
NWE_PULSE

#cycle
NRD_CYCLE
NWE_CYCLE

#mode
PS
PMEN
TDF_MODE
TDF_CYCLES
DBW
BAT
EXNW_MODE
WRITE_MODE
READ_MODE

*/

#define CALC_ADDR(addr) ((uint32_t*)(my_fpga.fpga_mem + addr))



struct clk* mclk;

struct fpga
{
	unsigned char* device_name;
	struct platform_device *pdev;
	
	unsigned int fpga_mem_phys_start;
	unsigned char* fpga_mem;
	unsigned int fpga_mem_size;
	
	unsigned char fpga_open;
};

struct fpga my_fpga;

void write_half(uint32_t addr, uint16_t data)
{
	printk("Writing to %x: %x\n", (unsigned int)CALC_ADDR(addr), data);
	iowrite16(data, CALC_ADDR(addr));
}

uint16_t read_half(uint32_t addr)
{
	return ioread16(CALC_ADDR(addr));
}


static int fpga_open(struct inode *inode, struct file *file)
{
	if ( my_fpga.fpga_open )
	{
		return -EBUSY;
	}
	my_fpga.fpga_open = 1;		
}

static int fpga_release(struct inode *inode, struct file *file)
{
   my_fpga.fpga_open = 0;
   return 0;
}


/* Called when a process, which already opened the dev file, attempts to
   //read from it.
*/
//static ssize_t device_read(struct file *filp,
   //char *buffer,    /* The buffer to fill with data */
   //size_t length,   /* The length of the buffer     */
   //loff_t *offset)  /* Our offset in the file       */
//{
   /* Number of bytes actually written to the buffer */
   //int bytes_read = 0;

   /* If we're at the end of the message, return 0 signifying end of file */
   //if (*msg_Ptr == 0) return 0;

   /* Actually put the data into the buffer */
   //while (length && *msg_Ptr)  {

        /* The buffer is in the user data segment, not the kernel segment;
         //* assignment won't work.  We have to use put_user which copies data from
         //* the kernel data segment to the user data segment. */
         //put_user(*(msg_Ptr++), buffer++);

         //length--;
         //bytes_read++;
   //}

   /* Most read functions return the number of bytes put into the buffer */
   //return bytes_read;
//}


/*  Called when a process writes to dev file: echo "hi" > /dev/hello */
//static ssize_t device_write(struct file *filp,
   //const char *buff,
   //size_t len,
   //loff_t *off)
//{
   //printk ("<1>Sorry, this operation isn't supported.\n");
   //return -EINVAL;
//}

static const struct file_operations fpga_fops = {
        .owner                = THIS_MODULE,
        //.read                 = device_read,
        //.write                = device_write,
        .open                 = fpga_open,
        .release              = fpga_release,
};




void setup_smc0(void);

static struct miscdevice fpga_dev = {
        /*
         * We don't care what minor number we end up with, so tell the
         * kernel to just pick one.
         */
        MISC_DYNAMIC_MINOR,
        /*
         * Name ourselves /dev/hello.
         */
        "fpga",
        /*
         * What functions to call when a program performs file
         * operations on the device.
         */
        &fpga_fops
};

static int sk_fpga_probe (struct platform_device *pdev)
{
	int ret = -EIO;
	unsigned int mclk_rate;
	my_fpga.pdev = pdev;

	printk("Loading FPGA driver for SK-AT91SAM9M10G45EK-XC6SLX\n");

	ret = misc_register(&fpga_dev);
	if (ret)
		printk(KERN_ERR"Unable to register \"fpga\" misc device\n");
	

	mclk = devm_clk_get(&pdev->dev, "mclk");
	if (IS_ERR(mclk)) {
		dev_err(&pdev->dev, "Failed to get MCLK\n");
		return ret;
	}
		
	
	mclk_rate = clk_get_rate(mclk);
	if (!mclk_rate) {
		dev_err(&pdev->dev, "Invalid slow clock rate\n");
		return -EINVAL;
	}
	
	ret = clk_set_rate(mclk, 133000000L);	
	
	ret = clk_prepare_enable(mclk);
	if (ret) {
		dev_err(&pdev->dev, "Could not enable mcllk clock\n");
		return ret;
	}
	
	ret = of_property_read_u32(pdev->dev.of_node, "fpga-mem-size", &my_fpga.fpga_mem_size);
	if (ret != 0)
	{
		printk("Failed to obtain fpga mem size from dtb\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "fpga-mem-phys-start", &my_fpga.fpga_mem_phys_start);
	if (ret != 0)
	{
		printk("Failed to obtain start mem address from dtb\n");
		return -ENOMEM;
	}

	request_mem_region(my_fpga.fpga_mem_phys_start, my_fpga.fpga_mem_size, "sk-fpga");
	my_fpga.fpga_mem = ioremap(my_fpga.fpga_mem_phys_start, my_fpga.fpga_mem_size);
	if ( !my_fpga.fpga_mem )
	{
		printk("Failed to remap mem for fpga\n");
		return -ENOMEM;
	}

	setup_smc0();
	my_fpga.fpga_open = 0;

	return ret;
}

void setup_smc0(void)
{
	uint32_t *smc;
	printk("Setup SMC0\n");
	request_mem_region(SMC, 0x1ff, "sk-fpga");
	
	smc = ioremap(SMC, 0x1ff);
	*(smc + SMC_SETUP(0)) = 0x01010101;
	*(smc + SMC_PULSE(0)) = 0x0e0e0e0e;
	*(smc + SMC_CYCLE(0)) = 0x000f000f;
	*(smc + SMC_MODE(0)) = 0x3 | 1<<12;
	
	iounmap(smc);
	
	release_mem_region(SMC, 0x1ff);
	return;
}


static int sk_fpga_remove (struct platform_device *pdev)
{
	printk(KERN_ALERT"Removing FPGA driver for SK-AT91SAM9M10G45EK-XC6SLX\n");
	iounmap(my_fpga.fpga_mem);
	release_mem_region(my_fpga.fpga_mem_phys_start, my_fpga.fpga_mem_size);
	misc_deregister(&fpga_dev);
	return 0;
}

static const struct of_device_id sk_fpga_of_match_table[] = {
	{ .compatible = "sk,at91-xc6slx", },
	{ /* end of list */ }
};
MODULE_DEVICE_TABLE(of, sk_fpga_of_match_table);

static struct platform_driver sk_fpga_driver = {
	.probe		= sk_fpga_probe,
	.remove		= sk_fpga_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "fpga",
		.of_match_table = of_match_ptr(sk_fpga_of_match_table),
	},
};
module_platform_driver(sk_fpga_driver);
MODULE_AUTHOR("Alexey Baturo <smd@hellheim.net>");
MODULE_DESCRIPTION("StarterKit fpga driver for SK-AT91-XC6SLX");
MODULE_LICENSE("GPL v2");
