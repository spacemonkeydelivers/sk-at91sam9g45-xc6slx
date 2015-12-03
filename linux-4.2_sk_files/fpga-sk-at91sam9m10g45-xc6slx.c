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
#include "fpga-sk-at91sam9m10f45-xc6slx.c"
//#include <linux/asm-arm/arch-at91/gpio.h>


#define WRITE_REG(phys_reg, value) (iowrite32(value, (void*)phys_reg))
#define READ_REG(phys_reg) (ioread32((void*)phys_reg))

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


#define CALC_ADDR(addr) ((uint32_t*)(my_fpga.fpga_mem + addr))

void prog_fpga(unsigned char* buff, unsigned int bufLen);

struct clk* mclk;

//struct fpga
//{
	//unsigned char* device_name;
	//struct platform_device *pdev;
	
	//unsigned int fpga_mem_phys_start;
	//unsigned char* fpga_mem;
	//unsigned int fpga_mem_size;
	
	//unsigned char fpga_open;
	//unsigned char fpga_irq_pin;
	//unsigned char fpga_cclk;
	//unsigned char fpga_din;
	//unsigned char fpga_done;
	//unsigned char fpga_prog;
//};

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
	unsigned err;
	int i, j;
	unsigned char byte;
	unsigned char bit;
	int done = 0;
	int counter = 0;
	err = gpio_request(my_fpga.fpga_prog, "fpga-prog-prog");
	gpio_direction_output(my_fpga.fpga_prog, 1);
	err = gpio_request(my_fpga.fpga_cclk, "fpga-prog-cclk");
	gpio_direction_output(my_fpga.fpga_cclk, 1);
	err = gpio_request(my_fpga.fpga_din, "fpga-prog-din");
	gpio_direction_output(my_fpga.fpga_din, 1);
	err = gpio_request(my_fpga.fpga_done, "fpga-prog-done");
	gpio_direction_input(my_fpga.fpga_done);
	
	gpio_set_value(my_fpga.fpga_prog, 0);
	udelay(10);
	gpio_set_value(my_fpga.fpga_prog, 1);
	udelay(1000);
	printk("fpga opened\n");
}

static int fpga_release(struct inode *inode, struct file *file)
{
	int counter , i, done = 0;
   my_fpga.fpga_open = 0;
   gpio_set_value(my_fpga.fpga_din, 1);
	done = gpio_get_value(my_fpga.fpga_done);
	counter = 0;
	while ( !done ) 
	{
		gpio_set_value(my_fpga.fpga_cclk, 1);
		gpio_set_value(my_fpga.fpga_cclk, 0);
		done = gpio_get_value(my_fpga.fpga_done);
		counter++;
		if ( counter > 8*2048 )
		{
			printk("prog fpga counter return\n");
			return;
		}
	}
	
	printk("GPIO DONE %d\n", done);
	
	for ( i = 0; i < 10; i++ )
	{
		gpio_set_value(my_fpga.fpga_cclk, 1);
		gpio_set_value(my_fpga.fpga_cclk, 0);
	}
   printk("fpga closed\n");
   return 0;
}


/* Called when a process, which already opened the dev file, attempts to
   ////read from it.
//*/
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
static ssize_t device_write(struct file *filp,
							   const char *buff,
							   size_t len,
							   loff_t *off)
{
	printk("DEV WRITE: %x %x\n", buff, len);
	prog_fpga(buff, len);
	//printk ("<1>Sorry, this operation isn't supported.\n");
	return len;
}

static const struct file_operations fpga_fops = {
        .owner                = THIS_MODULE,
        //.read                 = device_read,
        .write                = device_write,
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

/** Request IRQ for pin */
irqreturn_t interrupt_handler_function(int irq, void *dev_id)
{
    /* Start tasklet */
    if (gpio_get_value(my_fpga.fpga_irq_pin))
    {
		printk("IRQ HAPPENED\n");
	}
    return IRQ_HANDLED;
}

void prog_fpga(unsigned char* buff, unsigned int bufLen)
{
	unsigned err;
	int i, j;
	unsigned char byte;
	unsigned char bit;
	int done = 0;
	int counter = 0;
		
	for (i = 0; i < bufLen; i++)
	{
		//printk("COUNTER I: %d\n", i);
		byte = buff[i];
		for (j = 7; j >= 0; j--)
		{
			bit = 1 << j;
			bit &= byte;
			if (bit)
			{
				gpio_set_value(my_fpga.fpga_din, 1);
			}
			else
			{
				gpio_set_value(my_fpga.fpga_din, 0);
			}
			gpio_set_value(my_fpga.fpga_cclk, 1);
			gpio_set_value(my_fpga.fpga_cclk, 0);
		}
	}
	
}

unsigned register_interrupt(unsigned pin)
{
	unsigned err = gpio_request(pin, "fpga-irq-pin");
	gpio_direction_input(pin);
	unsigned irq_num = gpio_to_irq(pin);
	if(request_irq(irq_num, interrupt_handler_function, IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, "my_interrupt", NULL))  {
	//if(request_irq(irq_num, interrupt_handler_function, IRQF_TRIGGER_HIGH, "my_interrupt", NULL))  {
		printk(KERN_DEBUG"Can't register IRQ %d\n", irq_num);
		return -EIO;
	}
	
}



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

	ret = of_property_read_u32(pdev->dev.of_node, "fpga-mem-phys-start-cs0", &my_fpga.fpga_mem_phys_start);
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
	
	//fpga-irq-gpio = <&pioC 28 GPIO_ACTIVE_LOW>;
	//fpga-program-done = <&pioC 29 GPIO_ACTIVE_LOW>;
	//fpga-program-cclk = <&pioC 10 GPIO_ACTIVE_LOW>;
	//fpga-program-din = <&pioC 11 GPIO_ACTIVE_LOW>;
	//fpga-program-prog = <&pioC 30 GPIO_ACTIVE_LOW>;
	my_fpga.fpga_irq_pin = of_get_named_gpio(pdev->dev.of_node, "fpga-irq-gpio", 0);
	my_fpga.fpga_done = of_get_named_gpio(pdev->dev.of_node, "fpga-program-done", 0);
	my_fpga.fpga_cclk = of_get_named_gpio(pdev->dev.of_node, "fpga-program-cclk", 0);
	my_fpga.fpga_din = of_get_named_gpio(pdev->dev.of_node, "fpga-program-din", 0);
	my_fpga.fpga_prog = of_get_named_gpio(pdev->dev.of_node, "fpga-program-prog", 0);
	
	ret = register_interrupt(my_fpga.fpga_irq_pin);

//	int val = gpio_get_value(AT91_PIN_PD19);
//	printk("PD19: %x\n", val);

	//int irq = platform_get_irq(pdev, 0);
	//printk("IRQ %d\n", irq);
	


	//request_mem_region(0xfffff800, 0x200, "piod");
	//unsigned char* piodStart = ioremap(0xfffff800, 0x200);
	//printk("ABSR: %x\n", *(unsigned int*)(piodStart + PIO_ABSR));
	//printk("OWSR: %x\n", *(unsigned int*)(piodStart + PIO_OWSR));
	//printk("PUSR: %x\n", *(unsigned int*)(piodStart + PIO_PUSR));
	//printk("MDSR: %x\n", *(unsigned int*)(piodStart + PIO_MDSR));
	//printk("PSR: %x\n", *(unsigned int*)(piodStart + PIO_PSR));
	//printk("PDSR: %x\n", *(unsigned int*)(piodStart + PIO_PDSR));
	//printk("IFSR: %x\n", *(unsigned int*)(piodStart + PIO_IFSR));
	//printk("ODSR: %x\n", *(unsigned int*)(piodStart + PIO_ODSR));
	//printk("OSR: %x\n", *(unsigned int*)(piodStart + PIO_OSR));
	//iounmap(piodStart);

	//if(request_irq(irq, interrupt_handler_function, IRQF_TRIGGER_HIGH, "my_interrupt", NULL))  {
		//printk(KERN_DEBUG"Can't register IRQ %d\n", irq);
		//return -EIO;
	//}
	//printk("PDSR: %x\n", *(unsigned int*)(piodStart + PIO_PDSR));

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
