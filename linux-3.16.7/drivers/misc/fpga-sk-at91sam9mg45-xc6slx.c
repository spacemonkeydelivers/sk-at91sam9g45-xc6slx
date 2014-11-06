#include <linux/slab.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_mtd.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

#include <linux/delay.h>
#include <linux/dmaengine.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_data/atmel.h>


static int fpga_probe(struct platform_device *pdev)
{
	printk(KERN_ALERT "Module init\n");
	return 0;
}

static int fpga_remove(struct platform_device *pdev)
{
	printk(KERN_ALERT "Module exit\n");
	return 0;
}

static const struct of_device_id fpga_sk_dt_ids[] = {
	{ .compatible = "atmel,fpga_sk_at91sam9g45_xc6slx" },
	{ /* sentinel */ }
};

static struct platform_driver atmel_nand_driver = {
	.probe		= fpga_probe,
	.remove		= fpga_remove,
	.driver		= {
		.name	= "fpga_sk_at91sam9g45_xc6slx",
		.owner	= THIS_MODULE,
		.of_match_table	= of_match_ptr(fpga_sk_dt_ids),
	},
};

module_platform_driver(atmel_nand_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexey Baturo");
MODULE_DESCRIPTION("FPGA driver for SK-AT91SAM9G45-XC6SLX board.");
MODULE_ALIAS("platform:fpga_sk_at91sam9g45_xc6slx");
