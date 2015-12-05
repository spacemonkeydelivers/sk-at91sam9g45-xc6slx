#include <misc/fpga-sk-at91sam9m10g45-xc6slx.h>

typedef enum
{
	SK_FPGA_NO_STATUS = 0x0,
	SK_FPGA_PROGRAMMED = 0x1,
	SK_FPGA_SMC0_SET = 0x2,
	SK_FPGA_CLOCK_RATE_SET = 0x4,
	SK_FPGA_CLOCK_STARTED = 0x8,
} sk_fpga_status;

struct sk_fpga_action_set
{
	uint16_t action;
	uint16_t mode;
	uint16_t address_lo;
	uint16_t address_hi;
	uint16_t data_lo;
	uint16_t data_hi;
};

struct sk_fpga_action_get
{
	uint16_t data_read_lo;
	uint16_t data_read_hi;
};

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
	int fpga_irq_num;
	sk_fpga_status fpga_status; // status of fpga
	struct clk* fpga_clk; // clock for fpga
};

struct sk_fpga my_fpga;
struct sk_fpga_action_set my_fpga_action_set;
struct sk_fpga_action_get my_fpga_action_get;


irqreturn_t sk_fpga_interrupt_handler(int irq, void *dev_id)
{
	uint16_t status;
    if (gpio_get_value(my_fpga.fpga_irq_pin) == 1)
    {
		printk("IRQ HAPPENED\n");
		status = *((uint16_t*)(my_fpga.fpga_mem_virt_start_cs0 + STATUS));
		// read status reg to clear the interrupt
		// if act

	}
    return IRQ_HANDLED;
}

int sk_fpga_reg_interrupt(unsigned pin)
{
	unsigned irq_num;
	int err;
	err = gpio_request(pin, "fpga-irq-pin");
	gpio_direction_input(pin);
	irq_num = gpio_to_irq(pin);
	if(request_irq(irq_num, sk_fpga_interrupt_handler,IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING, "sk_fpga_interrupt", NULL))
	{
		printk("Can't register IRQ %d\n", irq_num);
		return -EIO;
	}
	my_fpga.fpga_irq_num = irq_num;
	return 0;
}

void sk_fpga_unreg_interrupt(void)
{
	free_irq(my_fpga.fpga_irq_num, NULL);
	gpio_free(my_fpga.fpga_irq_pin);
	my_fpga.fpga_irq_num = -1;
}

void sk_fpga_read_command(void)
{
	if (my_fpga_action_set.action == 1)
	{
		my_fpga_action_get.data_read_lo = *((uint16_t*)(my_fpga.fpga_mem_virt_start_cs0 + DATA_READ_LO));
		my_fpga_action_get.data_read_hi = *((uint16_t*)(my_fpga.fpga_mem_virt_start_cs0 + DATA_READ_HI));
	}
}

void sk_fpga_write_command(void)
{
	*((uint16_t*)(my_fpga.fpga_mem_virt_start_cs0 + MODE)) = my_fpga_action_set.mode;
	*((uint16_t*)(my_fpga.fpga_mem_virt_start_cs0 + DATA_LO)) = my_fpga_action_set.data_lo;
	*((uint16_t*)(my_fpga.fpga_mem_virt_start_cs0 + DATA_HI)) = my_fpga_action_set.data_hi;
	*((uint16_t*)(my_fpga.fpga_mem_virt_start_cs0 + ADDRESS_LO)) = my_fpga_action_set.address_lo;
	*((uint16_t*)(my_fpga.fpga_mem_virt_start_cs0 + ADDRESS_HI)) = my_fpga_action_set.address_hi;
	*((uint16_t*)(my_fpga.fpga_mem_virt_start_cs0 + ACTION)) = my_fpga_action_set.action;
	*((uint16_t*)(my_fpga.fpga_mem_virt_start_cs0 + RUN)) = 1;
}

int sk_fpga_setup_smc0(void)
{
	int ret = -EIO;
	uint32_t *smc;
	request_mem_region(SMC, 0x1ff, "sk-fpga-smc0");
	smc = ioremap(SMC, 0x1ff);
	if (!smc)
	{
		printk("Failed to remap mem for smc0\n");
		return ret;
	}
	*(smc + SMC_SETUP(0)) = 0x01010101;
	*(smc + SMC_PULSE(0)) = 0x0e0e0e0e;
	*(smc + SMC_CYCLE(0)) = 0x000f000f;
	*(smc + SMC_MODE(0)) = 0x3 | 1<<12;
	iounmap(smc);
	release_mem_region(SMC, 0x1ff);
	return 0;
}

static ssize_t sk_fpga_read(struct file *filp,
							char *buffer,    /* The buffer to fill with data */
							size_t length,   /* The length of the buffer     */
							loff_t *offset)  /* Our offset in the file       */
{
	printk("Reading of fpga is not implemented.\n");
	return -EIO;
}

static int sk_fpga_open(struct inode *inode, struct file *file)
{
	int err;
	if (my_fpga.fpga_open_counter != 0)
	{
		printk("Fpga is already opened: %d\n", my_fpga.fpga_open_counter);
		return -EBUSY;
	}
	if (my_fpga.fpga_irq_num > 0)
	{
		printk("UNREGISTERING INTERRUPT\n");
		sk_fpga_unreg_interrupt();
	}

	my_fpga.fpga_open_counter = 1;
	my_fpga.fpga_status &= ~(1 << SK_FPGA_PROGRAMMED);
	err = gpio_request(my_fpga.fpga_prog, "fpga-prog-prog");
	gpio_direction_output(my_fpga.fpga_prog, 1);
	err = gpio_request(my_fpga.fpga_cclk, "fpga-prog-cclk");
	gpio_direction_output(my_fpga.fpga_cclk, 1);
	err = gpio_request(my_fpga.fpga_din, "fpga-prog-din");
	gpio_direction_output(my_fpga.fpga_din, 1);
	err = gpio_request(my_fpga.fpga_done, "fpga-prog-done");
	gpio_direction_input(my_fpga.fpga_done);

	gpio_set_value(my_fpga.fpga_prog, 0);
//	udelay(10);
	gpio_set_value(my_fpga.fpga_prog, 1);
	//udelay(1000);
	return err;
}

static int sk_fpga_release(struct inode *inode, struct file *file)
{
	int counter, i, done = 0;
    my_fpga.fpga_open_counter = 0;
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
			return -EIO;
		}
	}
	for ( i = 0; i < 10; i++ )
	{
		gpio_set_value(my_fpga.fpga_cclk, 1);
		gpio_set_value(my_fpga.fpga_cclk, 0);
	}
	my_fpga.fpga_status |= SK_FPGA_PROGRAMMED;
	gpio_free(my_fpga.fpga_prog);
	gpio_free(my_fpga.fpga_cclk);
	gpio_free(my_fpga.fpga_din);
	gpio_free(my_fpga.fpga_done);
	if (sk_fpga_reg_interrupt(my_fpga.fpga_irq_pin) != 0)
	{
		printk("Failed to register fpga interrupt\n");
		return -EIO;
	}
    return 0;
}

void sk_fpga_prog(const unsigned char* buff, unsigned int bufLen)
{
	int i, j;
	unsigned char byte;
	unsigned char bit;
	for (i = 0; i < bufLen; i++)
	{
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

static ssize_t sk_fpga_write(struct file *filp,
							 const char *buff,
							 size_t len,
							 loff_t *off)
{
	sk_fpga_prog(buff, len);
	return len;
}

static long sk_fpga_ioctl(struct file *file, unsigned int ioctl_num, unsigned long ioctl_arg)
{
//    query_arg_t q;
//
//    switch (cmd)
//    {
//        case QUERY_GET_VARIABLES:
//            q.status = status;
//            q.dignity = dignity;
//            q.ego = ego;
//            if (copy_to_user((query_arg_t *)arg, &q, sizeof(query_arg_t)))
//            {
//                return -EACCES;
//            }
//            break;
//        case QUERY_CLR_VARIABLES:
//            status = 0;
//            dignity = 0;
//            ego = 0;
//            break;
//        case QUERY_SET_VARIABLES:
//            if (copy_from_user(&q, (query_arg_t *)arg, sizeof(query_arg_t)))
//            {
//                return -EACCES;
//            }
//            status = q.status;
//            dignity = q.dignity;
//            ego = q.ego;
//            break;
//        default:
//            return -EINVAL;
//    }

    return 0;
}

int sk_fpga_set_clk_rate(struct platform_device *pdev)
{
	int ret = -EIO;
	ret = clk_set_rate(my_fpga.fpga_clk, my_fpga.fpga_clk_rate * 1000000);
	if (ret) {
		dev_err(&pdev->dev, "Could not set fpga clk rate\n");
		return ret;
	}
	my_fpga.fpga_status |= SK_FPGA_CLOCK_RATE_SET;
	return 0;
}

int sk_fpga_start_clk(struct platform_device *pdev)
{
	int ret = -EIO;
	ret = clk_prepare_enable(my_fpga.fpga_clk);
	if (ret) {
		dev_err(&pdev->dev, "Could not enable fpga clock\n");
		return ret;
	}
	my_fpga.fpga_status |= SK_FPGA_CLOCK_STARTED;
	return 0;
}

int sk_fpga_fill_structure(struct platform_device *pdev)
{
	int ret = -EIO;
	my_fpga.fpga_irq_pin = of_get_named_gpio(pdev->dev.of_node, "fpga-irq-gpio", 0);
	if (!my_fpga.fpga_irq_pin) {
		dev_err(&pdev->dev, "Failed to fpga irq pin\n");
		return ret;
	}

	my_fpga.fpga_done = of_get_named_gpio(pdev->dev.of_node, "fpga-program-done", 0);
	if (!my_fpga.fpga_done) {
		dev_err(&pdev->dev, "Failed to fpga done pin\n");
		return ret;
	}

	my_fpga.fpga_cclk = of_get_named_gpio(pdev->dev.of_node, "fpga-program-cclk", 0);
	if (!my_fpga.fpga_cclk) {
		dev_err(&pdev->dev, "Failed to fpga cclk pin\n");
		return ret;
	}

	my_fpga.fpga_din = of_get_named_gpio(pdev->dev.of_node, "fpga-program-din", 0);
	if (!my_fpga.fpga_din) {
		dev_err(&pdev->dev, "Failed to fpga din pin\n");
		return ret;
	}

	my_fpga.fpga_prog = of_get_named_gpio(pdev->dev.of_node, "fpga-program-prog", 0);
	if (!my_fpga.fpga_prog) {
		dev_err(&pdev->dev, "Failed to fpga prog pin\n");
		return ret;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "fpga-mem-size", &my_fpga.fpga_mem_size);
	if (ret != 0)
	{
		printk("Failed to obtain fpga mem size from dtb\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "fpga-clk-rate", &my_fpga.fpga_clk_rate);
	if (ret != 0)
	{
		printk("Failed to obtain fpga clk rate from dtb\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "fpga-mem-phys-start-cs0", &my_fpga.fpga_mem_phys_start_cs0);
	if (ret != 0)
	{
		printk("Failed to obtain start phys mem start address for cs0 from dtb\n");
		return -ENOMEM;
	}

	ret = of_property_read_u32(pdev->dev.of_node, "fpga-mem-phys-start-cs1", &my_fpga.fpga_mem_phys_start_cs1);
	if (ret != 0)
	{
		printk("Failed to obtain start phys mem start address for cs1 from dtb\n");
		return -ENOMEM;
	}

	my_fpga.fpga_status = SK_FPGA_NO_STATUS;
	my_fpga.fpga_open_counter = 0;
	my_fpga.fpga_irq_num = -1;
	my_fpga.fpga_mem_virt_start_cs0 = NULL;
	my_fpga.fpga_mem_virt_start_cs1 = NULL;

	my_fpga.fpga_clk = devm_clk_get(&pdev->dev, "mclk");
	if (IS_ERR(my_fpga.fpga_clk)) {
		dev_err(&pdev->dev, "Failed to get clk source for fpga from dtb\n");
		return ret;
	}
	return 0;
}

static const struct file_operations fpga_fops = {
        .owner                = THIS_MODULE,
        //.read                 = sk_fpga_read,
        .write                = sk_fpga_write,
        .open                 = sk_fpga_open,
        .release              = sk_fpga_release,
		.unlocked_ioctl 	  = sk_fpga_ioctl,
};

static struct miscdevice fpga_dev = {
        MISC_DYNAMIC_MINOR,
        "fpga",
        &fpga_fops
};

static int sk_fpga_probe (struct platform_device *pdev)
{
	int ret = -EIO;
	my_fpga.pdev = pdev;

	printk("Loading FPGA driver for SK-AT91SAM9M10G45EK-XC6SLX\n");

	ret = misc_register(&fpga_dev);
	if (ret)
	{
		printk(KERN_ERR"Unable to register \"fpga\" misc device\n");
		return -ENOMEM;
	}

	ret = sk_fpga_fill_structure(my_fpga.pdev);
	if (ret)
	{
		printk(KERN_ERR"Unable to fill fpga structure out of dts\n");
		return -EINVAL;
	}

	request_mem_region(my_fpga.fpga_mem_phys_start_cs0, my_fpga.fpga_mem_size, "sk-fpga-mem-cs0");
	request_mem_region(my_fpga.fpga_mem_phys_start_cs1, my_fpga.fpga_mem_size, "sk-fpga-mem-cs1");

	my_fpga.fpga_mem_virt_start_cs0 = ioremap(my_fpga.fpga_mem_phys_start_cs0, my_fpga.fpga_mem_size);
	if (!my_fpga.fpga_mem_virt_start_cs0)
	{
		printk("Failed to remap mem for fpga cs0\n");
		return -ENOMEM;
	}
	my_fpga.fpga_mem_virt_start_cs1 = ioremap(my_fpga.fpga_mem_phys_start_cs0, my_fpga.fpga_mem_size);
	if (!my_fpga.fpga_mem_virt_start_cs1)
	{
		printk("Failed to remap mem for fpga cs1\n");
		return -ENOMEM;
	}

//	sk_fpga_set_clk_rate(pdev);

//	sk_fpga_start_clk(pdev);

	//sk_fpga_setup_smc0();

	my_fpga_action_set.action = 0;
	my_fpga_action_set.address_hi = 0;
	my_fpga_action_set.address_lo = 0;
	my_fpga_action_set.data_hi = 0;
	my_fpga_action_set.data_lo = 0;
	my_fpga_action_set.mode = 0;

	my_fpga_action_get.data_read_hi = 0;
	my_fpga_action_get.data_read_lo = 0;

	return ret;
}

static int sk_fpga_remove(struct platform_device *pdev)
{
	printk(KERN_ALERT"Removing FPGA driver for SK-AT91SAM9M10G45EK-XC6SLX\n");
	if (my_fpga.fpga_mem_virt_start_cs0)
	{
		iounmap(my_fpga.fpga_mem_virt_start_cs0);
	}
	if (my_fpga.fpga_mem_virt_start_cs1)
	{
		iounmap(my_fpga.fpga_mem_virt_start_cs1);
	}
	if (my_fpga.fpga_irq_num > 0)
	{
		printk("REMOVE UNREG IRQ\n");
		sk_fpga_unreg_interrupt();
	}
	release_mem_region(my_fpga.fpga_mem_phys_start_cs0, my_fpga.fpga_mem_size);
	release_mem_region(my_fpga.fpga_mem_phys_start_cs1, my_fpga.fpga_mem_size);
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
