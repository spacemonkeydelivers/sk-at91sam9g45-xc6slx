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

struct fpga
{
	int mem_start;
	int mem_finish;
	int mem_size;
	void* virt_mem;
};

struct fpga my_fpga;

static int sk_fpga_probe (struct platform_device *pdev)
{
	int ret = -EIO;
	struct resource *res; 
	unsigned int clock_divider = 100;
	
	printk("Loading FPGA driver for SK-AT91SAM9M10G45EK-XC6SLX\n");

	ret =  of_property_read_u32(pdev->dev.of_node, "clock_divider", &clock_divider);
	if (ret!=0)
	{
		printk("Failed to obtain clock divider for fpga\n");
		return -ENOMEM;
	}
	printk(KERN_ALERT"Clock divider %d\n", clock_divider);

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "fpga_mem");
	if (!res)
	{
		printk(KERN_ALERT"Failed to get regs\n");
		return -ENOMEM;
	}
	my_fpga.mem_start = res->start;
	my_fpga.mem_finish = res->end;
	my_fpga.mem_size = res->end - res->start + 1;
	my_fpga.virt_mem = devm_request_and_ioremap(&pdev->dev, res);
	if (!my_fpga.virt_mem)
	{
		printk("Failed to remap\n");
		return -ENOMEM;
	}
	printk(KERN_ALERT"%x %x %x %p", my_fpga.mem_start, my_fpga.mem_finish, my_fpga.mem_size, my_fpga.virt_mem);

	return 0;
}


static int sk_fpga_remove (struct platform_device *pdev)
{
	printk(KERN_ALERT"Remove\n");
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
