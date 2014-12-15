/*
 * Driver for the Solomon SSD1306 OLED controller
 *
 * Copyright 2012 Free Electrons
 * Copyright 2014 Xsilon
 *
 * Licensed under the GPLv2 or later.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/fb.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pwm.h>
#include <linux/delay.h>

#define DRIVER_NAME "ssd1306fb_gpio"
#define SPI_DRIVER_NAME "ssd1306fb_gpio-spi"

#define SSD1307FB_DATA			0x40
#define SSD1307FB_COMMAND		0x80

#define SSD1307FB_SET_ADDRESS_MODE	0x20
#define SSD1307FB_SET_ADDRESS_MODE_HORIZONTAL	(0x00)
#define SSD1307FB_SET_ADDRESS_MODE_VERTICAL	(0x01)
#define SSD1307FB_SET_ADDRESS_MODE_PAGE		(0x02)
#define SSD1307FB_SET_COL_RANGE		0x21
#define SSD1307FB_SET_PAGE_RANGE	0x22
#define SSD1307FB_CONTRAST		0x81
#define	SSD1307FB_CHARGE_PUMP		0x8d
#define SSD1307FB_SEG_REMAP_ON		0xa1
#define SSD1307FB_DISPLAY_OFF		0xae
#define SSD1307FB_SET_MULTIPLEX_RATIO	0xa8
#define SSD1307FB_DISPLAY_ON		0xaf
#define SSD1307FB_START_PAGE_ADDRESS	0xb0
#define SSD1307FB_SET_DISPLAY_OFFSET	0xd3
#define	SSD1307FB_SET_CLOCK_FREQ	0xd5
#define	SSD1307FB_SET_PRECHARGE_PERIOD	0xd9
#define	SSD1307FB_SET_COM_PINS_CONFIG	0xda
#define	SSD1307FB_SET_VCOMH		0xdb

static unsigned int spi_drv_registered;
static unsigned int device_num;
struct ssd1307fb_par;

struct ssd1307fb_par {
	char *name;
	struct spi_device *client;
	u32 height;
	struct fb_info *info;
	u32 page_offset;
	u32 width;
	struct platform_device *pdev;
	/* Pin Assignment */
	unsigned long iVBAT;
	unsigned long iVDD;
	unsigned long iRES;
	unsigned long iDC;
	unsigned long iSCLK;
	unsigned long iSDIN;
	unsigned long iCS;
	/* SPI Info */
	uint32_t spi_id;
};

struct ssd1307fb_array {
	u8	type;
	u8	data[0];
};

static struct fb_fix_screeninfo ssd1307fb_fix = {
	.id		= "Solomon SSD1307",
	.type		= FB_TYPE_PACKED_PIXELS,
	.visual		= FB_VISUAL_MONO10,
	.xpanstep	= 0,
	.ypanstep	= 0,
	.ywrapstep	= 0,
	.accel		= FB_ACCEL_NONE,
};

static struct fb_var_screeninfo ssd1307fb_var = {
	.bits_per_pixel	= 1,
};

static struct ssd1307fb_array *ssd1307fb_alloc_array(u32 len, u8 type)
{
	struct ssd1307fb_array *array;

	array = kzalloc(sizeof(struct ssd1307fb_array) + len, GFP_KERNEL);
	if (!array)
		return NULL;

	array->type = type;

	return array;
}

static int ssd1307fb_write_array(struct spi_device *client,
				 struct ssd1307fb_array *array, u32 len)
{
	int ret;

	len += sizeof(struct ssd1307fb_array);

	ret = spi_write(client, (u8 *)array, len);
	if (ret < 0) {
		dev_err(&client->dev, "Couldn't send SPI command.\n");
		return ret;
	}

	return 0;
}

static inline int ssd1307fb_write_cmd(struct spi_device *client, u8 cmd)
{
	struct ssd1307fb_array *array;
	int ret;

	array = ssd1307fb_alloc_array(1, SSD1307FB_COMMAND);
	if (!array)
		return -ENOMEM;

	array->data[0] = cmd;

	ret = ssd1307fb_write_array(client, array, 1);
	kfree(array);

	return ret;
}

static inline int ssd1307fb_write_data(struct spi_device *client, u8 data)
{
	struct ssd1307fb_array *array;
	int ret;

	array = ssd1307fb_alloc_array(1, SSD1307FB_DATA);
	if (!array)
		return -ENOMEM;

	array->data[0] = data;

	ret = ssd1307fb_write_array(client, array, 1);
	kfree(array);

	return ret;
}

static void ssd1307fb_update_display(struct ssd1307fb_par *par)
{
	struct ssd1307fb_array *array;
	u8 *vmem = par->info->screen_base;
	int i, j, k;

	array = ssd1307fb_alloc_array(par->width * par->height / 8,
				      SSD1307FB_DATA);
	if (!array)
		return;

	/*
	 * The screen is divided in pages, each having a height of 8
	 * pixels, and the width of the screen. When sending a byte of
	 * data to the controller, it gives the 8 bits for the current
	 * column. I.e, the first byte are the 8 bits of the first
	 * column, then the 8 bits for the second column, etc.
	 *
	 *
	 * Representation of the screen, assuming it is 5 bits
	 * wide. Each letter-number combination is a bit that controls
	 * one pixel.
	 *
	 * A0 A1 A2 A3 A4
	 * B0 B1 B2 B3 B4
	 * C0 C1 C2 C3 C4
	 * D0 D1 D2 D3 D4
	 * E0 E1 E2 E3 E4
	 * F0 F1 F2 F3 F4
	 * G0 G1 G2 G3 G4
	 * H0 H1 H2 H3 H4
	 *
	 * If you want to update this screen, you need to send 5 bytes:
	 *  (1) A0 B0 C0 D0 E0 F0 G0 H0
	 *  (2) A1 B1 C1 D1 E1 F1 G1 H1
	 *  (3) A2 B2 C2 D2 E2 F2 G2 H2
	 *  (4) A3 B3 C3 D3 E3 F3 G3 H3
	 *  (5) A4 B4 C4 D4 E4 F4 G4 H4
	 */

	for (i = 0; i < (par->height / 8); i++) {
		for (j = 0; j < par->width; j++) {
			u32 array_idx = i * par->width + j;
			array->data[array_idx] = 0;
			for (k = 0; k < 8; k++) {
				u32 page_length = par->width * i;
				u32 index = page_length + (par->width * k + j) / 8;
				u8 byte = *(vmem + index);
				u8 bit = byte & (1 << (j % 8));
				bit = bit >> (j % 8);
				array->data[array_idx] |= bit << k;
			}
		}
	}

	ssd1307fb_write_array(par->client, array, par->width * par->height / 8);
	kfree(array);
}


static ssize_t ssd1307fb_write(struct fb_info *info, const char __user *buf,
		size_t count, loff_t *ppos)
{
	struct ssd1307fb_par *par = info->par;
	unsigned long total_size;
	unsigned long p = *ppos;
	u8 __iomem *dst;

	total_size = info->fix.smem_len;

	if (p > total_size)
		return -EINVAL;

	if (count + p > total_size)
		count = total_size - p;

	if (!count)
		return -EINVAL;

	dst = (void __force *) (info->screen_base + p);

	if (copy_from_user(dst, buf, count))
		return -EFAULT;

	ssd1307fb_update_display(par);

	*ppos += count;

	return count;
}

static void ssd1307fb_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
	struct ssd1307fb_par *par = info->par;
	sys_fillrect(info, rect);
	ssd1307fb_update_display(par);
}

static void ssd1307fb_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
	struct ssd1307fb_par *par = info->par;
	sys_copyarea(info, area);
	ssd1307fb_update_display(par);
}

static void ssd1307fb_imageblit(struct fb_info *info, const struct fb_image *image)
{
	struct ssd1307fb_par *par = info->par;
	sys_imageblit(info, image);
	ssd1307fb_update_display(par);
}

static struct fb_ops ssd1307fb_ops = {
	.owner		= THIS_MODULE,
	.fb_read	= fb_sys_read,
	.fb_write	= ssd1307fb_write,
	.fb_fillrect	= ssd1307fb_fillrect,
	.fb_copyarea	= ssd1307fb_copyarea,
	.fb_imageblit	= ssd1307fb_imageblit,
};

static void ssd1307fb_deferred_io(struct fb_info *info,
				struct list_head *pagelist)
{
	ssd1307fb_update_display(info->par);
}

static struct fb_deferred_io ssd1307fb_defio = {
	.delay		= HZ,
	.deferred_io	= ssd1307fb_deferred_io,
};

static int add_gpio_pmodoled_device_to_bus(
					      struct ssd1307fb_par *dev)
{
	struct spi_master *spi_master;
	struct spi_device *spi_device;
	int status = 0;

	spi_master = spi_busnum_to_master(dev->spi_id);
	if (!spi_master) {
		dev_err(&dev->pdev->dev, "spi_busnum_to_master(%d) returned NULL\n", dev->spi_id);
		return -ENOSYS;
	}

	spi_device = spi_alloc_device(spi_master);
	if (!spi_device) {
		put_device(&spi_master->dev);
		dev_err(&dev->pdev->dev, "spi_alloc_device() failed\n");
		return -ENOMEM;
	}

	spi_device->chip_select = 0;
	spi_device->max_speed_hz = 4000000;
	spi_device->mode = SPI_MODE_0;
	spi_device->bits_per_word = 8;
	spi_device->controller_data = (void *) dev->iCS;
	spi_device->dev.platform_data = dev;
	strlcpy(spi_device->modalias, SPI_DRIVER_NAME, sizeof(SPI_DRIVER_NAME));

	status = spi_add_device(spi_device);
	if (status < 0) {
		spi_dev_put(spi_device);
		dev_err(&dev->pdev->dev, "spi_add_device() failed %d\n", status);
		return status;
	}
	dev->client = spi_device;

	put_device(&spi_master->dev);

	return status;
}

static int gpio_pmodoled_init_gpio(struct ssd1307fb_par *dev)
{
	struct gpio gpio_pmodoled_ctrl[] = {
		{dev->iVBAT, GPIOF_OUT_INIT_HIGH, "OLED VBat"},
		{dev->iVDD, GPIOF_OUT_INIT_HIGH, "OLED VDD"},
		{dev->iRES, GPIOF_OUT_INIT_HIGH, "OLED_RESET"},
		{dev->iDC, GPIOF_OUT_INIT_HIGH, "OLED_D/C"},
	};
	int status;
	int i;

	for (i = 0; i < ARRAY_SIZE(gpio_pmodoled_ctrl); i++) {
		status = gpio_is_valid(gpio_pmodoled_ctrl[i].gpio);
		if (!status) {
			dev_err(&dev->client->dev, "!! gpio_is_valid for GPIO %d, %s FAILED!, status: %d\n",
					gpio_pmodoled_ctrl[i].gpio, gpio_pmodoled_ctrl[i].label, status);
			goto gpio_invalid;
		}
	}

	status = gpio_request_array(gpio_pmodoled_ctrl, ARRAY_SIZE(gpio_pmodoled_ctrl));
	if (status) {
		dev_err(&dev->client->dev, "!!  gpio_request_array FAILED!\n");
		dev_err(&dev->client->dev, "          status is: %d\n", status);
		gpio_free_array(gpio_pmodoled_ctrl, 4);
		goto gpio_invalid;
	}

gpio_invalid:
	return status;
}

static int ssd1307fb_ssd1306_init(struct ssd1307fb_par *par)
{
	int ret;

	/* Set initial contrast */
	ret = ssd1307fb_write_cmd(par->client, SSD1307FB_CONTRAST);
	ret = ret & ssd1307fb_write_cmd(par->client, 0x7f);
	if (ret < 0)
		return ret;

	/* Set COM direction */
	ret = ssd1307fb_write_cmd(par->client, 0xc8);
	if (ret < 0)
		return ret;

	/* Set segment re-map */
	ret = ssd1307fb_write_cmd(par->client, SSD1307FB_SEG_REMAP_ON);
	if (ret < 0)
		return ret;

	/* Set multiplex ratio value */
	ret = ssd1307fb_write_cmd(par->client, SSD1307FB_SET_MULTIPLEX_RATIO);
	ret = ret & ssd1307fb_write_cmd(par->client, par->height - 1);
	if (ret < 0)
		return ret;

	/* set display offset value */
	ret = ssd1307fb_write_cmd(par->client, SSD1307FB_SET_DISPLAY_OFFSET);
	ret = ssd1307fb_write_cmd(par->client, 0x20);
	if (ret < 0)
		return ret;

	/* Set clock frequency */
	ret = ssd1307fb_write_cmd(par->client, SSD1307FB_SET_CLOCK_FREQ);
	ret = ret & ssd1307fb_write_cmd(par->client, 0xf0);
	if (ret < 0)
		return ret;

	/* Set precharge period in number of ticks from the internal clock */
	ret = ssd1307fb_write_cmd(par->client, SSD1307FB_SET_PRECHARGE_PERIOD);
	ret = ret & ssd1307fb_write_cmd(par->client, 0x22);
	if (ret < 0)
		return ret;

	/* Set COM pins configuration */
	ret = ssd1307fb_write_cmd(par->client, SSD1307FB_SET_COM_PINS_CONFIG);
	ret = ret & ssd1307fb_write_cmd(par->client, 0x22);
	if (ret < 0)
		return ret;

	/* Set VCOMH */
	ret = ssd1307fb_write_cmd(par->client, SSD1307FB_SET_VCOMH);
	ret = ret & ssd1307fb_write_cmd(par->client, 0x49);
	if (ret < 0)
		return ret;

	/* Turn on the DC-DC Charge Pump */
	ret = ssd1307fb_write_cmd(par->client, SSD1307FB_CHARGE_PUMP);
	ret = ret & ssd1307fb_write_cmd(par->client, 0x14);
	if (ret < 0)
		return ret;

	/* Switch to horizontal addressing mode */
	ret = ssd1307fb_write_cmd(par->client, SSD1307FB_SET_ADDRESS_MODE);
	ret = ret & ssd1307fb_write_cmd(par->client,
					SSD1307FB_SET_ADDRESS_MODE_HORIZONTAL);
	if (ret < 0)
		return ret;

	ret = ssd1307fb_write_cmd(par->client, SSD1307FB_SET_COL_RANGE);
	ret = ret & ssd1307fb_write_cmd(par->client, 0x0);
	ret = ret & ssd1307fb_write_cmd(par->client, par->width - 1);
	if (ret < 0)
		return ret;

	ret = ssd1307fb_write_cmd(par->client, SSD1307FB_SET_PAGE_RANGE);
	ret = ret & ssd1307fb_write_cmd(par->client, 0x0);
	ret = ret & ssd1307fb_write_cmd(par->client,
					par->page_offset + (par->height / 8) - 1);
	if (ret < 0)
		return ret;

	/* Turn on the display */
	ret = ssd1307fb_write_cmd(par->client, SSD1307FB_DISPLAY_ON);
	if (ret < 0)
		return ret;

	return 0;
}

/**
 * SPI hardware probe. Sets correct SPI mode, attempts
 * to obtain memory needed by the driver, and performs
 * a simple initialization of the device.
 */
static int ssd1306_spi_probe(struct spi_device *spi)
{
	int status = 0;
	struct ssd1307fb_par *par;

	/* We rely on full duplex transfers, mostly to reduce
	 * per transfer overheads (by making few transfers).
	 */
	if (spi->master->flags & SPI_MASTER_HALF_DUPLEX) {
		status = -EINVAL;
		dev_err(&spi->dev, "SPI settings incorrect: %d\n", status);
		goto spi_err;
	}

	/* We must use SPI_MODE_0 */
	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;

	status = spi_setup(spi);
	if (status < 0) {
		dev_err(&spi->dev, "needs SPI mode %02x, %d KHz; %d\n",
				spi->mode, spi->max_speed_hz / 1000,
				status);
		goto spi_err;
	}

	/* Get ssd1307fb_par structure */
	par = (struct ssd1307fb_par *) spi->dev.platform_data;
	if (par == NULL) {
		dev_err(&spi->dev, "Cannot get ssd1307fb_par.\n");
		status = -EINVAL;
		goto spi_platform_data_err;
	}

	printk(KERN_INFO SPI_DRIVER_NAME " [%s] SPI Probing\n", par->name);

	/**
	 * It is important to the OLED's longevity that the lines that
	 * control it's power are carefully controlled. This is a good
	 * time to ensure that the device is ot turned on until it is
	 * instructed to do so.
	 */
#ifdef CONFIG_PMODS_DEBUG
	printk(KERN_INFO SPI_DRIVER_NAME " [%s] spi_probe: initialize device\n", par->name);
#endif

	status = gpio_pmodoled_init_gpio(par);
	if (status) {
		dev_err(&spi->dev, "spi_probe: Error initializing GPIO\n");
		goto oled_init_error;
	}

	return status;

oled_init_error:
spi_platform_data_err:
spi_err:
	return status;
}

static int ssd1306_spi_remove(struct spi_device *spi)
{
	int status;
	struct ssd1307fb_par *dev;

	dev = (struct ssd1307fb_par *) spi->dev.platform_data;

	if (dev == NULL) {
		dev_err(&spi->dev, "spi_remove: Error fetch ssd1307fb_par struct\n");
		return -EINVAL;
	}

#ifdef CONFIG_PMODS_DEBUG
	printk(KERN_INFO SPI_DRIVER_NAME " [%s] spi_remove: Free GPIOs\n", dev->name);
#endif

{
	struct gpio gpio_pmodoled_ctrl[] = {
		{dev->iVBAT, GPIOF_OUT_INIT_HIGH, "OLED VBat"},
		{dev->iVDD, GPIOF_OUT_INIT_HIGH, "OLED VDD"},
		{dev->iRES, GPIOF_OUT_INIT_HIGH, "OLED_RESET"},
		{dev->iDC, GPIOF_OUT_INIT_HIGH, "OLED_D/C"},
	};

	gpio_free_array(gpio_pmodoled_ctrl, 4);
}

	printk(KERN_INFO SPI_DRIVER_NAME " [%s] spi_remove: Device Removed\n", dev->name);

	return status;
}

static struct spi_driver ssd1306_spi_driver = {
	.driver = {
		.name = SPI_DRIVER_NAME,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
	},
	.probe = ssd1306_spi_probe,
	.remove = ssd1306_spi_remove,
};

static int ssd1306_gpio_of_probe(struct platform_device *pdev)
{
	struct fb_info *info;
	struct device_node *node = pdev->dev.of_node;
	struct spi_gpio_platform_data *spi_gpio_pdata;
	struct platform_device *gpio_pmodoled_pdev;
	u32 vmem_size;
	struct ssd1307fb_par *par;
	u8 *vmem;
	int ret;
	const u32 *tree_info;
	int status = 0;

	if (!node) {
		dev_err(&pdev->dev, "No device tree data found!\n");
		return -EINVAL;
	}

	info = framebuffer_alloc(sizeof(struct ssd1307fb_par), &pdev->dev);
	if (!info) {
		dev_err(&pdev->dev, "Couldn't allocate framebuffer.\n");
		return -ENOMEM;
	}

	par = info->par;
	par->info = info;

	/* Get the GPIO Pins */
	par->iVBAT = of_get_named_gpio(node, "vbat-gpio", 0);
	par->iVDD = of_get_named_gpio(node, "vdd-gpio", 0);
	par->iRES = of_get_named_gpio(node, "res-gpio", 0);
	par->iDC = of_get_named_gpio(node, "dc-gpio", 0);
	par->iSCLK = of_get_named_gpio(node, "spi-sclk-gpio", 0);
	par->iSDIN = of_get_named_gpio(node, "spi-sdin-gpio", 0);
	status = of_get_named_gpio(node, "spi-cs-gpio", 0);
	par->iCS = (status < 0) ? SPI_GPIO_NO_CHIPSELECT : status;

	printk(KERN_INFO DRIVER_NAME " %s: iVBAT: 0x%lx\n", node->name, par->iVBAT);
	printk(KERN_INFO DRIVER_NAME " %s: iVDD : 0x%lx\n", node->name, par->iVDD);
	printk(KERN_INFO DRIVER_NAME " %s: iRES : 0x%lx\n", node->name, par->iRES);
	printk(KERN_INFO DRIVER_NAME " %s: iDC  : 0x%lx\n", node->name, par->iDC);
	printk(KERN_INFO DRIVER_NAME " %s: iSCLK: 0x%lx\n", node->name, par->iSCLK);
	printk(KERN_INFO DRIVER_NAME " %s: iSDIN: 0x%lx\n", node->name, par->iSDIN);
	printk(KERN_INFO DRIVER_NAME " %s: iCS  : 0x%lx\n", node->name, par->iCS);

	/* Get SPI Related Params */
	tree_info = of_get_property(node, "spi-bus-num", NULL);
	if (tree_info) {
		par->spi_id = be32_to_cpup((tree_info));
	}

	/* Alloc Space for spi_gpio data structure */
	spi_gpio_pdata = (struct spi_gpio_platform_data *) kzalloc(
			sizeof(*spi_gpio_pdata), GFP_KERNEL);
	if (!spi_gpio_pdata) {
		status = -ENOMEM;
		goto pdata_alloc_err;
	}

	/* Fill up Platform Data Structure */
	spi_gpio_pdata->sck = par->iSCLK;
	spi_gpio_pdata->miso = SPI_GPIO_NO_MISO;
	spi_gpio_pdata->mosi = par->iSDIN;
	spi_gpio_pdata->num_chipselect = 1;

	/* Alloc Space for platform data structure */
	gpio_pmodoled_pdev = (struct platform_device *) kzalloc(
			sizeof(*gpio_pmodoled_pdev), GFP_KERNEL);
	if (!gpio_pmodoled_pdev) {
		status = -ENOMEM;
		goto pdev_alloc_err;
	}

	/* Fill up Platform Device Structure */
	gpio_pmodoled_pdev->name = "spi_gpio";
	gpio_pmodoled_pdev->id = par->spi_id;
	gpio_pmodoled_pdev->dev.platform_data = spi_gpio_pdata;
	par->pdev = gpio_pmodoled_pdev;

	/* Register spi_gpio master */
	status = platform_device_register(par->pdev);
	if (status < 0) {
		dev_err(&pdev->dev, "platform_device_register failed: %d\n", status);
		goto pdev_reg_err;
	}

	printk(KERN_INFO DRIVER_NAME " %s: spi_gpio platform device registered.\n", node->name);

	par->name = node->name;

	/* Fill up Board Info for SPI device */
	status = add_gpio_pmodoled_device_to_bus(par);
	if (status < 0) {
		dev_err(&pdev->dev, "add_gpio_pmodoled_device_to_bus failed: %d\n",
				status);
		goto spi_add_err;
	}

	printk(KERN_INFO DRIVER_NAME " %s: spi device registered.\n", node->name);

	/* Point device node data to ssd1307fb_par structure */
	if (node->data == NULL)
		node->data = par;

	if (spi_drv_registered == 0) {
		/* Register SPI Driver for Pmodoled Device */
		status = spi_register_driver(&ssd1306_spi_driver);
		if (status < 0) {
			dev_err(&pdev->dev, "ssd1306_spi_driver register failed: %d\n",
					status);
			goto err_spi_register;
		}
		spi_drv_registered = 1;
	}

	device_num++;

	if (!gpio_is_valid(par->iRES)) {
		ret = -EINVAL;
		goto fb_alloc_error;
	}

	if (of_property_read_u32(node, "solomon,width", &par->width))
		par->width = 128;

	if (of_property_read_u32(node, "solomon,height", &par->height))
		par->height = 16;

	if (of_property_read_u32(node, "solomon,page-offset", &par->page_offset))
		par->page_offset = 1;

	vmem_size = par->width * par->height / 8;

	vmem = devm_kzalloc(&pdev->dev, vmem_size, GFP_KERNEL);
	if (!vmem) {
		dev_err(&pdev->dev, "Couldn't allocate graphical memory.\n");
		ret = -ENOMEM;
		goto fb_alloc_error;
	}

	info->fbops = &ssd1307fb_ops;
	info->fix = ssd1307fb_fix;
	info->fix.line_length = par->width / 8;
	info->fbdefio = &ssd1307fb_defio;

	info->var = ssd1307fb_var;
	info->var.xres = par->width;
	info->var.xres_virtual = par->width;
	info->var.yres = par->height;
	info->var.yres_virtual = par->height;

	info->var.red.length = 1;
	info->var.red.offset = 0;
	info->var.green.length = 1;
	info->var.green.offset = 0;
	info->var.blue.length = 1;
	info->var.blue.offset = 0;

	info->screen_base = (u8 __force __iomem *)vmem;
	info->fix.smem_start = (unsigned long)vmem;
	info->fix.smem_len = vmem_size;

	fb_deferred_io_init(info);

	/* Reset the screen */
	gpio_set_value(par->iRES, 0);
	udelay(4);
	gpio_set_value(par->iRES, 1);
	udelay(4);

	ret = ssd1307fb_ssd1306_init(par);
	if (ret)
		goto reset_oled_error;

	ret = register_framebuffer(info);
	if (ret) {
		dev_err(&pdev->dev, "Couldn't register the framebuffer\n");
		goto panel_init_error;
	}

	dev_info(&pdev->dev, "fb%d: %s framebuffer device registered, using %d bytes of video memory\n", info->node, info->fix.id, vmem_size);

	return 0;


panel_init_error:
reset_oled_error:
	fb_deferred_io_cleanup(info);
err_spi_register:
	spi_unregister_device(par->client);
spi_add_err:
	platform_device_unregister(par->pdev);
pdev_reg_err:
	kfree(gpio_pmodoled_pdev);
pdev_alloc_err:
	kfree(spi_gpio_pdata);
pdata_alloc_err:
fb_alloc_error:
	framebuffer_release(info);
	return ret;
}

static int ssd1306_gpio_of_remove(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct ssd1307fb_par *par = node->data;
	struct fb_info *info = par->info;

	unregister_framebuffer(info);
	fb_deferred_io_cleanup(info);
	framebuffer_release(info);

	if (par->pdev != NULL)
			platform_device_unregister(par->pdev);

		node->data = NULL;
		device_num--;

		if (device_num == 0) {
			spi_unregister_driver(par->pdev->dev.platform_data);
					spi_drv_registered = 0;
		}
	return 0;
}

static const struct of_device_id ssd1306_gpio_of_match[] = {
	{
			.compatible = "ssd1306_gpio",
	},
	{},
};
MODULE_DEVICE_TABLE(of, ssd1306_gpio_of_match);

static struct platform_driver ssd1306_gpio_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = ssd1306_gpio_of_match,
	},
	.probe = ssd1306_gpio_of_probe,
	.remove = ssd1306_gpio_of_remove,
};

module_platform_driver(ssd1306_gpio_driver);

MODULE_DESCRIPTION("FB driver for the Solomon SSD1306 OLED controller");
MODULE_AUTHOR("Simon Vincent <simon.vincent@xsilon.com>");
MODULE_LICENSE("GPL");
