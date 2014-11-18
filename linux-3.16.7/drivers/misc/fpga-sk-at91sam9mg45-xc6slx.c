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

#define WRITE_REG(phys_reg, value) (iowrite32(value, (void*)phys_reg);)
#define READ_REG(phys_reg) (ioread32((void*)phys_reg);)

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


#define SET_BIT(var, pos) (var |= 1 << pos;)
#define CLEAR_BIT(var, pos) (var &= ~(1 << pos);)
#define TOGGLE_BIT(var, pos) (var ^= 1 << pos;)
#define CHECK_BIT(var, pos) (var & (1 << pos);)


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


bool set_ebicsa(void)
{
	unsigned int ebicsa = READ_REG(CCFG_EBICSA);
	//EBI Chip Select 1 is assigned to the Static Memory Controller
	CLEAR_BIT(ebicsa, EBICS1A)
	//Optimized for 3.3V powered memories with High Drive, Maximum load capacitance < 55 pF
	SET_BIT(ebicsa, EBI_DRIVE_LOW)
	SET_BIT(ebicsa, EBI_DRIVE_HIGH)	
	WRITE_REG(CCFG_EBICSA, ebicsa);
	unsigned int ebicsa_new = READ_REG(CCFG_EBICSA);
	if (ebicsa == ebicsa_new)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool set_fpga_clocking(void)
{
	unsigned int pck0 = (my_fpga.fpga_clock_divider << PRES) | (my_fpga.slckmck << SLCKMCK) | (my_fpga.css << CSS);
	WRITE_REG(PMC_PCK0, pck0)
	unsigned int scer = READ_REG(PMC_SCSR)
	SET_BIT(scer, PCK0)
	WRITE_REG(PMC_SCER, scer)
	unsigned int status = CHECK_BIT(READ_REG(PMC_SR), PCK0)
	if (status)
	{	
		return true;
	}
	else
	{
		return false;
	}
}

void set_fpga_pin_out(void)
{
	unsigned int periph_reg = READ_REG(my_fpga.fpga_clock_pin_group + PIO_ABSR)
	unsigned int periph_status = CHECK_BIT(periph_reg, my_fpga.fpga_clock_pin)
	if (!periph_status)
	{
		WRITE_REG(my_fpga.fpga_clock_pin_group + PIO_BSR, (1 << my_fpga.fpga_clock_pin))
	}
	unsigned int pullup_reg = READ_REG(my_fpga.fpga_clock_pin_group + PIO_PUSR)
	unsigned int pullup_status = CHECK_BIT(pullup_reg, my_fpga.fpga_clock_pin)
	if (!pullup_status)
	{
		WRITE_REG(my_fpga.fpga_clock_pin_group + PIO_PUDR, (1 << my_fpga.fpga_clock_pin))
	}
	unsigned int output_reg = READ_REG(my_fpga.fpga_clock_pin_group + PIO_OSR)
	unsigned int output_status = CHECK_BIT(output_reg, my_fpga.fpga_clock_pin)
	if (!output_status)
	{
		WRITE_REG(my_fpga.fpga_clock_pin_group + PIO_OER, (1 << my_fpga.fpga_clock_pin))
	}
}

struct fpga
{
	unsigned char* device_name;
	unsigned int fpga_clock_divider;
	unsigned int fpga_css;
	unsigned int fpga_sclkmck;
	int mem_start;
	int mem_finish;
	int mem_size;
	void* virt_mem;
	struct platform_device* pdev;
	unsigned int fpga_clock_pin;
	unsigned int fpga_clock_pin_group;
	unsigned int fpga_cs_index[2];
};

struct fpga my_fpga;


static int sk_fpga_probe (struct platform_device *pdev)
{
	int ret = -EIO;
	struct resource *res; 
	my_fpga.pdev = pdev;

	printk("Loading FPGA driver for SK-AT91SAM9M10G45EK-XC6SLX\n");

	ret =  of_property_read_u32(pdev->dev.of_node, "fpga_clock_pin", &my_fpga.fpga_clock_pin_group);
	if (ret!=0)
	{
		printk("Failed to obtain fpga clock pin group\n");
		return -ENOMEM;
	}

	ret =  of_property_read_u32(pdev->dev.of_node, "fpga_clock_pin", &my_fpga.fpga_clock_pin);
	if (ret!=0)
	{
		printk("Failed to obtain fpga clock pin\n");
		return -ENOMEM;
	}

	ret =  of_property_read_u32(pdev->dev.of_node, "fpga_css", &my_fpga.fpga_css);
	if (ret!=0)
	{
		printk("Failed to obtain css for fpga\n");
		return -ENOMEM;
	}

	ret =  of_property_read_u32(pdev->dev.of_node, "fpga_slckmck", &my_fpga.fpga_slckmck);
	if (ret!=0)
	{
		printk("Failed to obtain slckmck for fpga\n");
		return -ENOMEM;
	}
	ret =  of_property_read_u32(pdev->dev.of_node, "fpga_clock_divider", &my_fpga.fpga_clock_divider);
	if (ret!=0)
	{
		printk("Failed to obtain clock divider for fpga\n");
		return -ENOMEM;
	}

	ret = of_property_read_string(pdev->dev.of_node, "device_name", my_fpga.device_name);
	if (ret!=0)
	{
		printk("Failed to obtain clock divider for fpga\n");
		return -ENOMEM;
	}

	res = of_property_read_u32_array(pdev->dev.of_node, "fpga_cs_index", &my_fpga.fpga_cs_index, 2);
	if (!res)
	{
		printk(KERN_ALERT"Failed to get fpga cs indexex\n")
		return -ENOMEM;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "fpga_mem");
	if (!res)
	{
		printk(KERN_ALERT"Failed to get fpga regs\n");
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
