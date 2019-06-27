/**
 * drivers/misc/classes/rtdm/fpga_rtdm.c
 *
 * Copyright (C) 2013, EMAC, Inc. <support@emacinc.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#include <rtdm/rtdm_driver.h>
#include <native/heap.h>
#include <linux/mman.h>
#include <linux/ioport.h>

#define FPGA_DEVICE_NAME "fpga-rtdm"
#define FPGA_PHYS_OFFSET 0x10000000
#define FPGA_PHYS_LEN 32

#define CPLD_DEVICE_NAME "cpld-rtdm"
#define CPLD_PHYS_OFFSET 0x20000000
#define CPLD_PHYS_LEN 16

#define MMAP_FPGA 0x1
#define MMAP_CPLD 0x2

static int fpga_rtdm_open(struct rtdm_dev_context *cxt, rtdm_user_info_t * uinfo, int oflags)
{
	return 0;
}

static int fpga_rtdm_close(struct rtdm_dev_context *cxt, rtdm_user_info_t * uinfo)
{
	return 0;
}

static ssize_t fpga_rtdm_ioctl(struct rtdm_dev_context *cxt, rtdm_user_info_t * uinfo, unsigned int cmd, void *arg)
{
	int ret=0;
	
	switch(cmd)
	{
		case MMAP_FPGA:
			ret = rtdm_iomap_to_user(uinfo,
					FPGA_PHYS_OFFSET,
					FPGA_PHYS_LEN,
					PROT_READ | PROT_WRITE,
					(void **)arg, NULL, NULL);
			if(ret != 0)
				printk(KERN_ERR "rtdm_iomap_to_user: failed\n");
			break;
		case MMAP_CPLD:
			ret = rtdm_iomap_to_user(uinfo,
					CPLD_PHYS_OFFSET,
					CPLD_PHYS_LEN,
					PROT_READ | PROT_WRITE,
					(void **)arg, NULL, NULL);
			if(ret != 0)
				printk(KERN_ERR "rtdm_iomap_to_user: failed\n");
			break;

	}
	return ret;
}

static struct rtdm_device device = {
	.struct_version = RTDM_DEVICE_STRUCT_VER,
	.device_flags = RTDM_NAMED_DEVICE,
	.context_size = 0,
	.device_name = FPGA_DEVICE_NAME,

	.open_nrt = fpga_rtdm_open,

	.ops = {
		.close_nrt = fpga_rtdm_close,
		.ioctl_rt  = fpga_rtdm_ioctl,
		.ioctl_nrt = fpga_rtdm_ioctl,
	},

	.device_class = RTDM_CLASS_EXPERIMENTAL,
	.device_sub_class = RTDM_SUBCLASS_GENERIC,
	.profile_version = 1,
	.driver_name = "fpga_rtdm",
	.driver_version = RTDM_DRIVER_VER(1, 0, 0),
	.peripheral_name = "FPGA RTDM",
	.provider_name = "EMAC, Inc.",
	.proc_name = device.device_name,
};

/** 
 * Registers the rtdm device.
 */
static int __init fpga_init(void)
{
	int err;

	err = rtdm_dev_register(&device);

	if (request_mem_region(CPLD_PHYS_OFFSET, CPLD_PHYS_LEN, CPLD_DEVICE_NAME) == NULL) 
		printk(KERN_ERR "Error requesting memory region for CPLD.\n");

	if (request_mem_region(FPGA_PHYS_OFFSET, FPGA_PHYS_LEN, FPGA_DEVICE_NAME) == NULL) 
		printk(KERN_ERR "Error requesting memory region for FPGA.\n");

	if(err) 
		printk(KERN_ERR "Error registering RTDM device.\n");

	return err;
}

/**
 * Unregisters the rtdm device.
 */
void fpga_exit(void)
{
	int err;

	err = rtdm_dev_unregister(&device, 0);

	if(err)
		printk(KERN_ERR "Error unregistering RTDM device.\n");
}

module_init(fpga_init);
module_exit(fpga_exit);

MODULE_AUTHOR("EMAC, Inc. <support@emacinc.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("FPGA RTDM Driver");
