/*
 *  GPIO interface for Vortex86 Series SoC
 *
 *  Copyright (c) 2017 EMAC Inc.
 *  Copyright (c) 2017 QWERTY Embedded Design, LLC
 *  Author: Michael Welling <mwelling@ieee.org>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License 2 as published
 *  by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqdomain.h>
#include <linux/gpio/driver.h>


static DEFINE_SPINLOCK(sio_lock);

#define DRV_NAME "vortex86_gpio"
#ifdef CONFIG_VORTEX86DX2
#define VORTEX86_NUM_GPIOS	40
#define VORTEX86_NUM_IRQS	8

#define GPIO_DAT_BASE 0x78
#define GPIO_DIR_BASE 0x98
#else
#define VORTEX86_NUM_GPIOS	8
#define VORTEX86_NUM_IRQS	1

#define GPIO_DAT_BASE 0x102
#define GPIO_DIR_BASE 0x94
#endif

#define GPIO_DAT_NAME "vortex86-gpio-dat"
#define GPIO_DIR_NAME "vortex86-gpio-dir"


#define GPIO_INT_IMR	0xdc
#define GPIO_INT_ILR	0xdd
#define GPIO_INT_ICR	0xde

#define GPIO_INT0_ACK	0x9f
#define GPIO_IRQ_VORTEX86	3
#define GPIO_IOSIZE		8

struct vortex86_gpio_chip_data {
	u16 gpio_dat;
	u16 gpio_dir;

	u8 irq_mask;

	struct pci_dev *pdev;
	struct gpio_chip chip;
};


static int vortex86_gpio_get(struct gpio_chip *gc, unsigned gpio_num)
{
	u16 base = GPIO_DAT_BASE + (gpio_num / 8);
	u16 bit = 1 << (gpio_num % 8);
	u8 val;

	spin_lock(&sio_lock);
	val = inb(base);
	spin_unlock(&sio_lock);

	return (val & bit) ? 1 : 0;
}

static int vortex86_gpio_direction_in(struct gpio_chip *gc, unsigned gpio_num)
{
	u16 base = GPIO_DIR_BASE + (gpio_num / 8);
	u8 bit = 1 << (gpio_num % 8);
	u8 cval;

	spin_lock(&sio_lock);
	cval = inb(base);
	cval &= ~bit;
	outb(cval, base);
	spin_unlock(&sio_lock);

	return 0;
}

static void vortex86_gpio_set(struct gpio_chip *gc,
				unsigned gpio_num, int val)
{
	u16 base = GPIO_DAT_BASE + (gpio_num / 8);
	u8 bit = 1 << (gpio_num % 8);
	u8 cval;

	spin_lock(&sio_lock);
	cval = inb(base);
	if (val)
		cval |= bit;
	else
		cval &= ~bit;
	outb(cval, base);
	spin_unlock(&sio_lock);
}

static int vortex86_gpio_direction_out(struct gpio_chip *gc,
					unsigned gpio_num, int val)
{
	u16 base = GPIO_DIR_BASE + (gpio_num / 8);
	u8 bit = 1 << (gpio_num % 8);
	u8 cval;

	spin_lock(&sio_lock);
	cval = inb(base);
	cval |= bit;
	outb(cval, base);
	spin_unlock(&sio_lock);

	vortex86_gpio_set(gc, gpio_num, val);

	return 0;
}

static void vortex86_irq_mask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct vortex86_gpio_chip_data *sd = container_of(gc, struct
						vortex86_gpio_chip_data,
						chip);
	const unsigned long offset = irqd_to_hwirq(data);
	u8 imrval;

	if ((sd->pdev) && (offset < VORTEX86_NUM_IRQS)) {
		pci_read_config_byte(sd->pdev, GPIO_INT_IMR, &imrval);
		outb(1 << offset, GPIO_INT0_ACK);
		imrval &= ~(1 << offset);
		sd->irq_mask &= ~(1 << offset);
		pci_write_config_byte(sd->pdev, GPIO_INT_IMR, imrval);
	}
}

static void vortex86_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct vortex86_gpio_chip_data *sd = container_of(gc, struct
						vortex86_gpio_chip_data,
						chip);
	const unsigned long offset = irqd_to_hwirq(data);
	u8 imrval;

	if ((sd->pdev) && (offset < VORTEX86_NUM_IRQS)) {
		pci_read_config_byte(sd->pdev, GPIO_INT_IMR, &imrval);
		outb(1 << offset, GPIO_INT0_ACK);
		imrval |= 1 << offset;
		sd->irq_mask |= 1 << offset;
		pci_write_config_byte(sd->pdev, GPIO_INT_IMR, imrval);
	}
}

static int vortex86_irq_set_type(struct irq_data *data, unsigned type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct vortex86_gpio_chip_data *sd = container_of(gc, struct
						vortex86_gpio_chip_data,
						chip);
	const unsigned long offset = irqd_to_hwirq(data);
	u8 ilrval;

	if ((offset >= VORTEX86_NUM_IRQS) || (!sd->pdev)) {
		return -EINVAL;
	}

	if (type == IRQ_TYPE_LEVEL_HIGH) {
		pci_read_config_byte(sd->pdev, GPIO_INT_ILR, &ilrval);
		ilrval &= ~(1 << offset);
		pci_write_config_byte(sd->pdev, GPIO_INT_ILR, ilrval);
		return 0;
	}

	if (type == IRQ_TYPE_LEVEL_LOW) {
		pci_read_config_byte(sd->pdev, GPIO_INT_ILR, &ilrval);
		ilrval |= 1 << offset;
		pci_write_config_byte(sd->pdev, GPIO_INT_ILR, ilrval);
		return 0;
	}

	return -EINVAL;
}

static int vortex86_gpio_set_irq_affinity(struct irq_data *data,
					  const struct cpumask *mask,
					  bool force)
{
	return 0;
}

static struct irq_chip vortex86_irq_chip = {
	.name			= DRV_NAME,
	.irq_mask		= vortex86_irq_mask,
	.irq_unmask		= vortex86_irq_unmask,
	.irq_set_type		= vortex86_irq_set_type,
	.irq_set_affinity       = vortex86_gpio_set_irq_affinity,
};

static void vortex86_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *gc = irq_desc_get_handler_data(desc);
	struct irq_data *data = irq_desc_get_irq_data(desc);
	struct irq_chip *chip = irq_data_get_irq_chip(data);
	struct vortex86_gpio_chip_data *sd = container_of(gc, struct
						vortex86_gpio_chip_data,
						chip);

	u8 reg;
	int bit;
	unsigned int irqc;

	chained_irq_enter(chip, desc);
	reg = inb(GPIO_INT0_ACK) & sd->irq_mask;

	for (bit = 0; bit < 8; bit++) {
		if (reg & (1 << bit)) {
			irqc = irq_find_mapping(gc->irqdomain, bit);

			generic_handle_irq(irqc);
		}
	}

	outb(0xff, GPIO_INT0_ACK);
	chained_irq_exit(chip, desc);
}

static int vortex86_gpio_probe(struct pci_dev *pdev,
					const struct pci_device_id *ent)
{
	struct vortex86_gpio_chip_data *sd;
	struct gpio_chip *gc;
	int err;


	sd = kzalloc(sizeof(struct vortex86_gpio_chip_data), GFP_KERNEL);
	if (!sd)
		return -ENOMEM;

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "Vortex86 pci_enable_device failed!\n");
		goto pci_enable_err;
	}

	sd->gpio_dat = GPIO_DAT_BASE;
	sd->gpio_dir = GPIO_DIR_BASE;
	sd->irq_mask = 0;
	sd->pdev = pdev;

	if (!request_region(sd->gpio_dat, GPIO_IOSIZE, GPIO_DAT_NAME)) {
		err = -EBUSY;
		goto request_dat_err;
	}
	if (!request_region(sd->gpio_dir, GPIO_IOSIZE, GPIO_DIR_NAME)) {
		err = -EBUSY;
		goto request_dir_err;
	}

	gc = &sd->chip;
	gc->label = DRV_NAME,
	gc->owner = THIS_MODULE,
	gc->get = vortex86_gpio_get,
	gc->set = vortex86_gpio_set,
	gc->direction_input = vortex86_gpio_direction_in,
	gc->direction_output = vortex86_gpio_direction_out,
	gc->base = -1;
	gc->ngpio = VORTEX86_NUM_GPIOS;
	gc->parent = &pdev->dev;


	err = gpiochip_add(gc);
	if (err < 0)
		goto gpiochip_add_err;

	pdev->irq = GPIO_IRQ_VORTEX86;

	pci_write_config_byte(pdev, GPIO_INT_IMR, 0x00);
	pci_write_config_byte(pdev, GPIO_INT_ICR, 0x82);

	err = gpiochip_irqchip_add(gc, &vortex86_irq_chip, 0,
				   handle_simple_irq, IRQ_TYPE_NONE);
	if (err < 0) {
		dev_err(&pdev->dev, "Vortex86 gpiochip_irqchip_add failed!\n");
		goto irqchip_add_err;
	}

	gpiochip_set_chained_irqchip(gc, &vortex86_irq_chip,
				 pdev->irq, vortex86_gpio_irq_handler);

	pci_set_drvdata(pdev, sd);
	dev_info(&pdev->dev, "Vortex86 GPIO driver registered.\n");
	return 0;

irqchip_add_err:
	gpiochip_remove(gc);
gpiochip_add_err:
	release_region(sd->gpio_dat, GPIO_IOSIZE);
request_dir_err:
	release_region(sd->gpio_dir, GPIO_IOSIZE);
request_dat_err:
	pci_disable_device(pdev);
pci_enable_err:
	kfree(sd);

	return err;
}

static void vortex86_gpio_remove(struct pci_dev *pdev)
{
	struct vortex86_gpio_chip_data *sd = pci_get_drvdata(pdev);

	gpiochip_remove(&sd->chip);
	release_region(sd->gpio_dat, GPIO_IOSIZE);
	release_region(sd->gpio_dir, GPIO_IOSIZE);
	pci_disable_device(pdev);
	kfree(sd);
}

static const struct pci_device_id vortex86_gpio_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_RDC, 0x6031) },
	{ PCI_DEVICE(PCI_VENDOR_ID_RDC, 0x6025) },
	{}
};
MODULE_DEVICE_TABLE(pci, vortex86_gpio_table);

static struct pci_driver vortex86_gpio_driver = {
	.name		= DRV_NAME,
	.id_table	= vortex86_gpio_table,
	.probe		= vortex86_gpio_probe,
	.remove		= vortex86_gpio_remove,
};

module_pci_driver(vortex86_gpio_driver);

MODULE_AUTHOR("Michael Welling <mwelling@ieee.org>");
MODULE_DESCRIPTION("GPIO interface for Vortex86 Series SoC");
MODULE_LICENSE("GPL");
