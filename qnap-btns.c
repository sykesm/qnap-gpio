/*
 *	qnap_btns.c 
 *	
 *	GPIO Button Control for QNAP TS-239 Pro (ICP/iEi board)
 *
 *	(c) Copyright 2010  Damien ALBERT <dalbert@loisix.org>
 *
 *	Based on it87.c			by Chris Gauthron, Jean Delvare,
 *		 pl061.c   		by Baruch Siach <baruch@tkos.co.il>
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


#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/dmi.h>

static unsigned int __initdata nodetect;
module_param_named(nodetect, nodetect, bool, 0);
MODULE_PARM_DESC(nodetect, "Skip DMI hardware detection");

static struct platform_device *pdev;

/* GPIO Base Address */
static unsigned int gpio_base_address = 0;

/* De-Bounde timer */
struct timer_list timer;
#define DBOUNCE_INTERVAL	150 /* ms */

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
#define GPIO_ADDR_MSB_REG  	0x62 /* Simple I/O Base Address MSB Register */
#define GPIO_ADDR_LSB_REG  	0x63 /* Simple I/O Base Address LSB Register */
#define GPIO_DBOUNCE_SEL_REG	0x70 /* Panel Button De-bounce Interrupt Level Select Register */
#define GPIO_SET1_ENABLE_REG	0xc0 /* Simple I/O Set 1 Enable Register */
#define GPIO_SET1_OUTPUT_EN_REG	0xc8 /* Simple I/O Set 1 Output Enable Register */
#define GPIO_BTN0_MAPPING_REG	0xe0 /* Panel Button De-bounce 0 Input Pin Mapping Register */
#define GPIO_BTN1_MAPPING_REG   0xe1 /* Panel Button De-bounce 1 Input Pin Mapping Register */
#define GPIO_SMI_STATUS_REG	0xf3 /* SMI# Status Register 2 */

/* GPIO Set Index */
#define GPIO_SET1_INDEX		0

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

static void qnap_timer(unsigned long data)
{
	int tmp;
	unsigned long flags;

        struct input_dev *qnap_input = (struct input_dev *) data;

	spin_lock_irqsave(&spinlock, flags);
	superio_enter();
	superio_select(SIO_GPIO_LDN);

	tmp = inb(gpio_base_address);

	input_report_key(qnap_input, KEY_ARCHIVE, (tmp & 0x40) >> 6 ); /* USB Copy Button */
	input_report_key(qnap_input, KEY_RESTART, (tmp & 0x80) >> 7 ); /* RESET Button */
	input_sync(qnap_input);

	superio_outb(GPIO_SMI_STATUS_REG, tmp | 0xc0);

	superio_exit();
	spin_unlock_irqrestore(&spinlock, flags);
}


irqreturn_t qnap_interrupt(int irq, void *_qnap_input)
{
	if (!timer_pending(&timer))
        	mod_timer(&timer, jiffies + msecs_to_jiffies(DBOUNCE_INTERVAL));
	
    	return IRQ_HANDLED;
}

static int __init qnap_gpio_button_init(void)
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

	// GP16 GP17
	tmp = superio_inb(GPIO_SET1_SEL_REG);
	superio_outb(GPIO_SET1_SEL_REG, tmp | 0xc0);	//select GPIO Set1 function
	
	tmp = superio_inb(GPIO_SET1_ENABLE_REG);
	superio_outb(GPIO_SET1_ENABLE_REG, tmp | 0xc0);	//enable simple IO function

	tmp = superio_inb(GPIO_SET1_OUTPUT_EN_REG);
	superio_outb(GPIO_SET1_OUTPUT_EN_REG, tmp & 0x3f); //enable input direction

	superio_outb(GPIO_DBOUNCE_SEL_REG, 0x03); // set De-Bounce

	tmp = superio_inb(GPIO_SMI_STATUS_REG);
	superio_outb(GPIO_SMI_STATUS_REG, tmp | 0xc0); /* reset panel button de-bounce status */

	superio_outb(GPIO_BTN0_MAPPING_REG, 0x4e); /* GP16 = 001110 */
	superio_outb(GPIO_BTN1_MAPPING_REG, 0x4f); /* GP17 = 001111 */

	err = 0;
exit:
	superio_exit();
	spin_unlock_irqrestore(&spinlock, flags);
	return err;
}

static int __init qnap_button_probe(struct platform_device *pdev)
{
        struct input_dev *qnap_input;
        int err;

        qnap_input = input_allocate_device();
        if (!qnap_input) {
                dev_dbg(&pdev->dev, "Can't allocate QNAP buttons\n");
                return -ENOMEM;
        }

        qnap_input->name = "qnap_btns";
        qnap_input->phys = "qnap_btns/input0";
        qnap_input->dev.parent = &pdev->dev;

	qnap_input->users = 3; /* IRQ3 value */

	input_set_capability(qnap_input, EV_KEY, KEY_ARCHIVE); /* USB Copy Button */
	input_set_capability(qnap_input, EV_KEY, KEY_RESTART); /* Reset Button */

        err = request_threaded_irq(qnap_input->users, NULL, qnap_interrupt,
                        IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
                        "it87_gpio", qnap_input);
        if (err < 0) {
                dev_dbg(&pdev->dev, "Can't get IRQ for QNAP buttons: %d\n", err);
                goto free_input_dev;
        }

        err = input_register_device(qnap_input);
        if (err) {
                dev_dbg(&pdev->dev, "Can't register QNAP buttons: %d\n", err);
                goto free_irq;
        }

	input_report_key(qnap_input, KEY_ARCHIVE, 1 );
	input_report_key(qnap_input, KEY_RESTART, 1 );
	input_sync(qnap_input);
	setup_timer(&timer, qnap_timer, (unsigned long) qnap_input);

        platform_set_drvdata(pdev, qnap_input);

        return 0;

free_irq:
        free_irq(qnap_input->users, NULL);
free_input_dev:
        input_free_device(qnap_input);
        return err;
}

static int qnap_button_remove(struct platform_device *pdev)
{
        struct input_dev *qnap_input = platform_get_drvdata(pdev);

        free_irq(qnap_input->users, qnap_input);
	del_timer_sync(&timer);  
        input_unregister_device(qnap_input);
        input_free_device(qnap_input);
        return 0;
}

struct platform_driver qnap_button_driver = {
        .probe          = qnap_button_probe,
        .remove         = qnap_button_remove,
        .driver         = {
                .name   = KBUILD_MODNAME,
                .owner  = THIS_MODULE,
        },
};

static int __init qnap_button_init(void)
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
	
	error = qnap_gpio_button_init();

	if (error < 0)
		return error;

	pdev = platform_device_register_simple(KBUILD_MODNAME, -1, NULL, 0);
	if (!IS_ERR(pdev)) {
		error = platform_driver_probe(&qnap_button_driver,
					      qnap_button_probe);
		if (error) {
			pr_err(KBUILD_MODNAME ": Can't probe platform driver\n");
			platform_device_unregister(pdev);
		}
	} else
		error = PTR_ERR(pdev);

	return error;
}

static void __exit qnap_button_exit(void)
{
        platform_device_unregister(pdev);
	platform_driver_unregister(&qnap_button_driver);
}

module_init(qnap_button_init);
module_exit(qnap_button_exit);

MODULE_AUTHOR("Damien ALBERT <dalbert@loisix.org>");  
MODULE_DESCRIPTION("QNAP x86/Atom board Button driver");   
MODULE_LICENSE("GPL");           
