/*
 *	qnap_leds.c 
 *
 *	GPIO LED Control for QNAP TS-239 Pro (ICP/iEi board)
 *
 *	(c) Copyright 2010  Damien ALBERT <dalbert@loisix.org>
 *
 *	Based on it87.c			by Chris Gauthron, Jean Delvare,
 *		 leds-alix2.c   	by Constantin Baranov
 *		 leds-clevo-mail.c	by Márton Németh
 *
 *
 *	IT8718F GPIO Affectations on QNAP TS-239 Pro board :
 *		
 *		GP11 : spining HDD2
 *		GP14 : spining HDD1
 *		GP16 : USB Copy Button
 *		GP17 : Reset Button
 *		GP20-GP24 : Red HDD Error Led
 * 		GP24-GP25 : Redundant Power Failure
 *		GP34 : Red STATUS Led
 * 		GP35 : Green STATUS Led
 *		GP40 | GP43 : Power Button ?
 *		GP50 : Blue USB Led
 *				
 *	Data-sheets: Publicly available at the ITE website
 *		    http://www.ite.com.tw/
 *
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program; if not, write to the Free Software
 *	Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/dmi.h>

static unsigned int __initdata nodetect;
module_param_named(nodetect, nodetect, bool, 0);
MODULE_PARM_DESC(nodetect, "Skip DMI hardware detection");

static struct platform_device *pdev;

/* GPIO Base Address */
static unsigned int gpio_base_address = 0;

/* IO Configuration Ports */
#define SIO_ADDR_REG		0x2e /* IT87 Special Address Port */
#define SIO_DATA_REG		0x2f /* IT87 Special Data Port after entering PnP Mode */

/* Configuration Registers and Functions */
#define SIO_LDN_REG		0x07 /* Logical Device Number index Register */
#define SIO_CHIP_ID		0x20 /* Chip ID Byte 1 & 2 */
#define SIO_CHIP_REV 		0x22 /* Chip Version (01h for C version) */

/* IT8718F Chip Id numbers */
#define SIO_IT8718_ID		0x8718 /* IT8718F Identification */

/* Logical device Numbers LDN */
#define SIO_GPIO_LDN		0x07 /* GPIO Device Number */

/* GPIO Configuration Registers when LDN=0x07 */
#define GPIO_SET1_SEL_REG  	0x25 /* GPIO Set 1 Multi-Function Pin Selection Register */
#define GPIO_SET2_SEL_REG  	0x26 /* GPIO Set 2 Multi-Function Pin Selection Register */
#define GPIO_SET3_SEL_REG  	0x27 /* GPIO Set 3 Multi-Function Pin Selection Register */
#define GPIO_SET4_SEL_REG  	0x28 /* GPIO Set 4 Multi-Function Pin Selection Register */
#define GPIO_SET5_SEL_REG  	0x29 /* GPIO Set 5 Multi-Function Pin Selection Register */

#define GPIO_EXT1_SEL_REG  	0x2a /* Extended 1 Multi-Function Pin Selection Register */
#define GPIO_EXT2_SEL_REG  	0x2c /* Extended 2 Multi-Function Pin Selection Register */

#define GPIO_ADDR_MSB_REG  	0x62 /* Simple I/O Base Address MSB Register */
#define GPIO_ADDR_LSB_REG  	0x63 /* Simple I/O Base Address LSB Register */

#define GPIO_DBOUNCE_SEL_REG	0x70 /* Panel Button De-bounce Interrupt Level Select Register */

#define GPIO_SET1_POLARITY_REG	0xb0 /* GPIO Set 1 Pin Polarity Register */
#define GPIO_SET2_POLARITY_REG	0xb1 /* GPIO Set 2 Pin Polarity Register */
#define GPIO_SET3_POLARITY_REG	0xb2 /* GPIO Set 3 Pin Polarity Register */
#define GPIO_SET4_POLARITY_REG	0xb3 /* GPIO Set 4 Pin Polarity Register */
#define GPIO_SET5_POLARITY_REG	0xb4 /* GPIO Set 5 Pin Polarity Register */
#define GPIO_SET6_POLARITY_REG	0xb5 /* GPIO Set 6 Pin Polarity Register */

#define GPIO_SET1_PULLUP_REG	0xb8 /* GPIO Set 1 Pin Internal Pull-up Enable Register */
#define GPIO_SET2_PULLUP_REG	0xb9 /* GPIO Set 2 Pin Internal Pull-up Enable Register */
#define GPIO_SET3_PULLUP_REG	0xba /* GPIO Set 3 Pin Internal Pull-up Enable Register */
#define GPIO_SET4_PULLUP_REG	0xbb /* GPIO Set 4 Pin Internal Pull-up Enable Register */
#define GPIO_SET5_PULLUP_REG	0xbc /* GPIO Set 5 Pin Internal Pull-up Enable Register */
#define GPIO_SET6_PULLUP_REG	0xbd /* GPIO Set 6 Pin Internal Pull-up Enable Register */

#define GPIO_SET1_ENABLE_REG	0xc0 /* Simple I/O Set 1 Enable Register */
#define GPIO_SET2_ENABLE_REG	0xc1 /* Simple I/O Set 2 Enable Register */
#define GPIO_SET3_ENABLE_REG	0xc2 /* Simple I/O Set 3 Enable Register */
#define GPIO_SET4_ENABLE_REG	0xc3 /* Simple I/O Set 4 Enable Register */
#define GPIO_SET5_ENABLE_REG	0xc4 /* Simple I/O Set 5 Enable Register */

#define GPIO_SET1_OUTPUT_EN_REG	0xc8 /* Simple I/O Set 1 Output Enable Register */
#define GPIO_SET2_OUTPUT_EN_REG	0xc9 /* Simple I/O Set 2 Output Enable Register */
#define GPIO_SET3_OUTPUT_EN_REG	0xca /* Simple I/O Set 3 Output Enable Register */
#define GPIO_SET4_OUTPUT_EN_REG	0xcb /* Simple I/O Set 4 Output Enable Register */
#define GPIO_SET5_OUTPUT_EN_REG	0xcc /* Simple I/O Set 5 Output Enable Register */

#define GPIO_BTN0_MAPPING_REG	0xe0 /* Panel Button De-bounce 0 Input Pin Mapping Register */
#define GPIO_BTN1_MAPPING_REG   0xe1 /* Panel Button De-bounce 1 Input Pin Mapping Register */

#define GPIO_IRQ0_MAPPING_REG	0xe2 /* IRQ External Routing 0 Input Pin Mapping Register */
#define GPIO_IRQ1_MAPPING_REG	0xe3 /* IRQ External Routing 1 Input Pin Mapping Register */

#define GPIO_IRQ_INT_SEL_REG	0xe4 /* IRQ External Routing 1-0 Interrupt Level Selection Register */

#define GPIO_LED_BLINK1_MAP_REG	0xf8 /* GP LED Blinking 1 Pin Mapping Register */
#define GPIO_LED_BLINK1_CTL_REG	0xf9 /* GP LED Blinking 1 Control Register */
#define GPIO_LED_BLINK2_MAP_REG	0xfa /* GP LED Blinking 2 Pin Mapping Register */
#define GPIO_LED_BLINK2_CTL_REG	0xfb /* GP LED Blinking 2 Control Register */

/* GPIO Set Index */
#define GPIO_SET1_INDEX		0
#define GPIO_SET2_INDEX   	1
#define GPIO_SET3_INDEX       	2
#define GPIO_SET4_INDEX       	3
#define GPIO_SET5_INDEX       	4

/* GPIO LED Blinking Control */
#define GPIO_LED_BLINK_NORMAL_4HZ	0x00
#define GPIO_LED_BLINK_NORMAL_1HZ	0x02
#define GPIO_LED_BLINK_NORMAL_1_4HZ	0x04
#define GPIO_LED_BLINK_NORMAL_1_8HZ	0x06
#define GPIO_LED_BLINK_PULSE_4HZ	0x08
#define GPIO_LED_BLINK_PULSE_1HZ	0x0a
#define GPIO_LED_BLINK_PULSE_1_4HZ	0x0c
#define GPIO_LED_BLINK_PULSE_1_8HZ	0x0e

/* Spinlock Definition */
static	DEFINE_SPINLOCK(spinlock);

/* Superio Chip */
static inline void superio_enter(void)
{
	outb(0x87, SIO_ADDR_REG);
	outb(0x01, SIO_ADDR_REG);
	outb(0x55, SIO_ADDR_REG);
	outb(0x55, SIO_ADDR_REG);
}

static inline void superio_exit(void)
{
	outb(0x02, SIO_ADDR_REG);
	outb(0x02, SIO_DATA_REG);
}

static inline void superio_select(int ldn)
{
	outb(SIO_LDN_REG, SIO_ADDR_REG);
	outb(ldn, SIO_DATA_REG);
}

static inline int superio_inb(int reg)
{
	outb(reg, SIO_ADDR_REG);
	return inb(SIO_DATA_REG);
}

static inline void superio_outb(int reg, int val)
{
	outb(reg, SIO_ADDR_REG);
	outb(val, SIO_DATA_REG);
}

static inline int superio_inw(int reg)
{
	int val;
	outb(reg++, SIO_ADDR_REG);
	val = inb(SIO_DATA_REG) << 8;
	outb(reg, SIO_ADDR_REG);
	val |= inb(SIO_DATA_REG);
	return val;
}

static unsigned char gpio_inb(unsigned char index)
{
	return inb(gpio_base_address + index);
}

static void gpio_outb(unsigned char index, unsigned char data)
{
        outb(data, gpio_base_address + index);
}

/* Helper Look Up Tables */
unsigned char gpio_set_sel_reg[] = {
	GPIO_SET1_SEL_REG,
	GPIO_SET2_SEL_REG,
	GPIO_SET3_SEL_REG,
	GPIO_SET4_SEL_REG,
	GPIO_SET5_SEL_REG
};

unsigned char gpio_set_enable_reg[] = {
	GPIO_SET1_ENABLE_REG,
	GPIO_SET2_ENABLE_REG,
	GPIO_SET3_ENABLE_REG,
	GPIO_SET4_ENABLE_REG,
	GPIO_SET5_ENABLE_REG
};

unsigned char gpio_set_output_enable_reg[] = {
	GPIO_SET1_OUTPUT_EN_REG,
	GPIO_SET2_OUTPUT_EN_REG,
	GPIO_SET3_OUTPUT_EN_REG,
	GPIO_SET4_OUTPUT_EN_REG,
	GPIO_SET5_OUTPUT_EN_REG
};

unsigned char gpio_led_blink_map_reg[] = {
	GPIO_LED_BLINK1_MAP_REG,
	GPIO_LED_BLINK2_MAP_REG
};

unsigned char gpio_led_blink_ctl_reg[] = {
	GPIO_LED_BLINK1_CTL_REG,
	GPIO_LED_BLINK2_CTL_REG
};

unsigned char byte_bit_set[] = {0x01, 0x02, 0x04, 0x08,	0x10, 0x20, 0x40, 0x80};
unsigned char byte_bit_clr[] = {0xfe, 0xfd, 0xfb, 0xf7, 0xef, 0xdf, 0xbf, 0x7f};

/* DMI hardware detection */
static int __init qnap_dmi_callback(const struct dmi_system_id *id)
{
	pr_info(KBUILD_MODNAME ": '%s' found\n", id->ident);
	return 1;
};

static struct dmi_system_id __initdata qnap_whitelist[] = {
	{
		.callback = qnap_dmi_callback,
		.ident = "QNAP TS-239 Pro",
		.matches = {
			DMI_MATCH(DMI_SYS_VENDOR, "ICP / iEi"),
			DMI_MATCH(DMI_PRODUCT_NAME, "ATOM NAS"),
			DMI_MATCH(DMI_BOARD_NAME, "ATOM NAS (QE05)"),
			DMI_MATCH(DMI_BOARD_VERSION, "V1.0")
		}
	},
};

/* LED settings Definitions */
struct qnap_led {
	struct led_classdev cdev;
	unsigned char set_index;	/* GPIO_SET1_INDEX to GPIO_SET5_INDEX */
	unsigned char bit_index;	/* 0 to 7 */
	unsigned char blink_bank;	/* 0 or 1 */
	unsigned char blink_map;	/* see mapping in IT8718F spec. */
};

static void qnap_led_set(struct led_classdev *led_cdev, 
				enum led_brightness brightness)
{
	int tmp;
	unsigned long flags;
	
	struct qnap_led *led_dev = container_of(led_cdev, struct qnap_led, cdev);

	spin_lock_irqsave(&spinlock, flags);
	superio_enter();
	superio_select(SIO_GPIO_LDN);

        tmp = superio_inb(gpio_set_enable_reg[led_dev->set_index]);
        superio_outb(gpio_set_enable_reg[led_dev->set_index], tmp | byte_bit_set[led_dev->bit_index]);
        tmp = superio_inb(gpio_set_output_enable_reg[led_dev->set_index]);
        superio_outb(gpio_set_output_enable_reg[led_dev->set_index], tmp | byte_bit_set[led_dev->bit_index]);
        tmp = gpio_inb(led_dev->set_index);

        if (brightness)
        	gpio_outb(led_dev->set_index, tmp & byte_bit_clr[led_dev->bit_index]);
        else
                gpio_outb(led_dev->set_index, tmp | byte_bit_set[led_dev->bit_index]);

	superio_exit();
	spin_unlock_irqrestore(&spinlock, flags);
}

static int qnap_led_blink(struct led_classdev *led_cdev,
				unsigned long *delay_on,
				unsigned long *delay_off)
{
	int status = -EINVAL;
	int tmp;
	unsigned char blink_ctl;
	unsigned long flags;

	struct qnap_led *led_dev = container_of(led_cdev, struct qnap_led, cdev);

	if (*delay_on == 0 /* ms */ && *delay_off == 0 /* ms */) {
		/* Special case: the leds subsystem requested us to
		 * chose one user friendly blinking of the LED, and
		 * start it. Let's blink the led slowly (1Hz).
		 */
		*delay_on = 500; /* ms */
		*delay_off = 500; /* ms */
		blink_ctl = GPIO_LED_BLINK_NORMAL_1HZ;
		status = 0;

	} else if (*delay_on == 125 /* ms */ && *delay_off == 125 /* ms */) {
		/* blink the led with 4Hz */
		blink_ctl = GPIO_LED_BLINK_NORMAL_4HZ;
		status = 0;

	} else if (*delay_on == 500 /* ms */ && *delay_off == 500 /* ms */) {
		/* blink the led with 1Hz */
		blink_ctl = GPIO_LED_BLINK_NORMAL_1HZ;
		status = 0;

	} else if (*delay_on == 2000 /* ms */ && *delay_off == 2000 /* ms */) {
		/* blink the led with 1/4Hz */
		blink_ctl = GPIO_LED_BLINK_NORMAL_1_4HZ;
		status = 0;

	} else if (*delay_on == 4000 /* ms */ && *delay_off == 4000 /* ms */) {
		/* blink the led with 1/8Hz */
		blink_ctl = GPIO_LED_BLINK_NORMAL_1_8HZ;
		status = 0;

	} else if (*delay_on < 500 /* ms */ && *delay_off == 500 /* ms */) {
		/* blink the led with 1Hz */
		blink_ctl = GPIO_LED_BLINK_PULSE_1HZ;
		status = 0;

	} else if (*delay_on < 500 /* ms */ && *delay_off == 2000 /* ms */) {
		/* blink the led with 1/4Hz */
		blink_ctl = GPIO_LED_BLINK_PULSE_1_4HZ;
		status = 0;

	} else if (*delay_on < 500 /* ms */ && *delay_off == 4000 /* ms */) {
		/* blink the led with 1/8Hz */
		blink_ctl = GPIO_LED_BLINK_PULSE_1_8HZ;
		status = 0;

	} else {
		pr_debug(KBUILD_MODNAME
		       ": qnap_led_blink(..., %lu, %lu),"
		       " returning -EINVAL (unsupported)\n",
		       *delay_on, *delay_off);
	}

	if (status == 0) {
		spin_lock_irqsave(&spinlock, flags);
		superio_enter();
		superio_select(SIO_GPIO_LDN);

		tmp = superio_inb(gpio_set_enable_reg[led_dev->set_index]);
        	superio_outb(gpio_set_enable_reg[led_dev->set_index], 
						tmp & byte_bit_clr[led_dev->bit_index]);
        	superio_outb(gpio_led_blink_map_reg[led_dev->blink_bank], led_dev->blink_map); 
                superio_outb(gpio_led_blink_ctl_reg[led_dev->blink_bank], blink_ctl);

		superio_exit();
		spin_unlock_irqrestore(&spinlock, flags);
	}

	return status;
}

static struct qnap_led qnap_leds[] = {
	{
		/* RED STATUS LED is GP34 -> bit 4 of GPIO Set 3 */
		.cdev = {
                        .name = "qnap:red_status",
                        .brightness_set = qnap_led_set,
			.blink_set = qnap_led_blink,	
                },
                .set_index = GPIO_SET3_INDEX,
                .bit_index = 4,
		.blink_bank = 0,
		.blink_map = 0x1c, /* GP34 = 011100 */
	},
	{
		/* GREEN STATUS LED is GP35 -> bit 5 of GPIO Set 3 */
		.cdev = {
                        .name = "qnap:green_status",
                        .brightness_set = qnap_led_set,
			.blink_set = qnap_led_blink,	
                },
                .set_index = GPIO_SET3_INDEX,
                .bit_index = 5,
		.blink_bank = 0,
		.blink_map = 0x1d, /* GP35 = 011101 */
	},
	{
		/* BLUE USB LED is GP50 -> bit 0 of GPIO Set 5 */
		.cdev = {
                        .name = "qnap:blue_usb",
                        .brightness_set = qnap_led_set,
			.blink_set = qnap_led_blink,	
                },
                .set_index = GPIO_SET5_INDEX,
                .bit_index = 0,
		.blink_bank = 1,
		.blink_map = 0x28, /* GP50 = 101000 */
	},
	{
		/* RED HDD1 LED is GP20 -> bit 0 of GPIO Set 2 */
		.cdev = {
                        .name = "qnap:red_hdd1",
                        .brightness_set = qnap_led_set,
			.blink_set = qnap_led_blink,	
                },
                .set_index = GPIO_SET2_INDEX,
                .bit_index = 0,
		.blink_bank = 1,
		.blink_map = 0x10, /* GP20 = 010000 */
	},
	{
		/* RED HDD2 LED is GP21 -> bit 1 of GPIO Set 2 */
		.cdev = {
                        .name = "qnap:red_hdd2",
                        .brightness_set = qnap_led_set,
			.blink_set = qnap_led_blink,	
                },
                .set_index = GPIO_SET2_INDEX,
                .bit_index = 1,
		.blink_bank = 1,
		.blink_map = 0x11, /* GP21 = 010001 */
	},

};

static int __init qnap_led_probe(struct platform_device *pdev)
{
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(qnap_leds); i++) {
		qnap_leds[i].cdev.flags |= LED_CORE_SUSPENDRESUME;
		ret = led_classdev_register(&pdev->dev, &qnap_leds[i].cdev);
		if (ret < 0)
			goto fail;
	}
	return 0;

fail:
	while (--i >= 0)
		led_classdev_unregister(&qnap_leds[i].cdev);
	return ret;
}

static int qnap_led_remove(struct platform_device *pdev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(qnap_leds); i++)
		led_classdev_unregister(&qnap_leds[i].cdev);
	return 0;
}

static struct platform_driver qnap_led_driver = {
	.probe	= qnap_led_probe,
	.remove = qnap_led_remove,
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
	},
};

static int __init qnap_gpio_led_init(void)
{
	u16 chip_type;
	u8  chip_rev;
	unsigned long flags;
	int err = -ENODEV;
	int tmp;

	spin_lock_irqsave(&spinlock, flags);
	superio_enter();

	chip_type = superio_inw(SIO_CHIP_ID);
	chip_rev  = superio_inb(SIO_CHIP_REV) & 0x0f;

	if (chip_type != SIO_IT8718_ID)
		goto exit;

	superio_select(SIO_GPIO_LDN);
		
	gpio_base_address = superio_inw(GPIO_ADDR_MSB_REG);

	//GP34 GP35
	tmp = superio_inb(gpio_set_sel_reg[GPIO_SET3_INDEX]);
        superio_outb(gpio_set_sel_reg[GPIO_SET3_INDEX], tmp | 0x30);
  	tmp = superio_inb(gpio_set_enable_reg[GPIO_SET3_INDEX]);
        superio_outb(gpio_set_enable_reg[GPIO_SET3_INDEX], tmp | 0x30);
        tmp = superio_inb(gpio_set_output_enable_reg[GPIO_SET3_INDEX]);
        superio_outb(gpio_set_output_enable_reg[GPIO_SET3_INDEX], tmp | 0x30);

	//GP50
	tmp = superio_inb(gpio_set_sel_reg[GPIO_SET5_INDEX]);
        superio_outb(gpio_set_sel_reg[GPIO_SET5_INDEX], tmp | 0x09);
  	tmp = superio_inb(gpio_set_enable_reg[GPIO_SET5_INDEX]);
        superio_outb(gpio_set_enable_reg[GPIO_SET5_INDEX], tmp | 0x09);
        tmp = superio_inb(gpio_set_output_enable_reg[GPIO_SET5_INDEX]);
        superio_outb(gpio_set_output_enable_reg[GPIO_SET5_INDEX], tmp | 0x09);

	//GP20 GP21
	tmp = superio_inb(gpio_set_sel_reg[GPIO_SET2_INDEX]);
        superio_outb(gpio_set_sel_reg[GPIO_SET2_INDEX], tmp | 0x0f);
  	tmp = superio_inb(gpio_set_enable_reg[GPIO_SET2_INDEX]);
        superio_outb(gpio_set_enable_reg[GPIO_SET2_INDEX], tmp | 0x0f);
        tmp = superio_inb(gpio_set_output_enable_reg[GPIO_SET2_INDEX]);
        superio_outb(gpio_set_output_enable_reg[GPIO_SET2_INDEX], tmp | 0x0f);

	err = 0;
exit:
	superio_exit();
	spin_unlock_irqrestore(&spinlock, flags);
	return err;
}

static int __init qnap_led_init(void)
{
	int error = 0;
	int count = 0;

	/* Check with the help of DMI if we are running on supported hardware */
	if (!nodetect) {
		count = dmi_check_system(qnap_whitelist);
	} else {
		count = 1;
		pr_err(KBUILD_MODNAME ": Skipping DMI detection.\n");
	}

	if (!count)
		return -ENODEV;
	
	error = qnap_gpio_led_init();

	if (error < 0)
		return error;

	pdev = platform_device_register_simple(KBUILD_MODNAME, -1, NULL, 0);
	if (!IS_ERR(pdev)) {
		error = platform_driver_probe(&qnap_led_driver,
					      qnap_led_probe);
		if (error) {
			pr_err(KBUILD_MODNAME ": Can't probe platform driver\n");
			platform_device_unregister(pdev);
		}
	} else
		error = PTR_ERR(pdev);

	return error;
}

static void __exit qnap_led_exit(void)
{
        platform_device_unregister(pdev);
	platform_driver_unregister(&qnap_led_driver);
}


module_init(qnap_led_init);
module_exit(qnap_led_exit);

MODULE_AUTHOR("Damien ALBERT <dalbert@loisix.org>");    
MODULE_DESCRIPTION("QNAP x86/Atom board LED driver"); 
MODULE_LICENSE("GPL");           
