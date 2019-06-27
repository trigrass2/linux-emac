/**
 * A class to implement an interface to SPI controllers from a real-time
 * context under Xenomai using the RTDM interface.
 *
 * Copyright (C) 2010 EMAC, Inc. <support@emacinc.com>
 *
 */

#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/class/spi.h>
#include <linux/class/rtdm/spi_rtdm.h>
#include <rtdm/rtdm_driver.h>
#include <native/heap.h>

/**
 * Size of data to be allocated for each device upon creation.
 */
#define SPI_RT_HEAP_SIZE 1024

/**
 * structure defining a RTDM spi device
 * @rtd: RTDM device
 * @spi: pointer to the parent SPI class device
 * @heap: memory heap for performing real-time allocations
 */
struct rtspi_device_s {
	struct rtdm_device rtd;
	struct spi_s *spi;
	RT_HEAP heap;
};

/**
 * Function called when opening the device in rt or nrt context
 */
static int rt_spi_open(struct rtdm_dev_context *context,
			rtdm_user_info_t * user_info, int oflags)
{
	return 0;
}

/**
 * Function called when closing the device in rt or nrt context
 */
static int rt_spi_close(struct rtdm_dev_context *context,
			 rtdm_user_info_t * user_info)
{
	return 0;
}

static int rt_spi_ioctl(struct rtdm_dev_context *context,
			 rtdm_user_info_t *user_info, unsigned int request,
			 void __user *umem)
{
	struct rtspi_device_s *dev =
	    container_of(context->device, struct rtspi_device_s, rtd);
	struct spi_s *spi = dev->spi;

	spi_control kmem[1];

	/*
	 * XMIT and XMITMSG can only be used in real-time context. Other
	 * ioctls may be used in rt or nrt context.
	 */
	switch (request) {
	case XMIT:
	{
		struct spi_transfer_s transfer;
		spi_data *buf = NULL;
		int errval = 0;

		if (unlikely(!rtdm_in_rt_context()))
			return -EPERM;
		if (unlikely(!spi->xmit))
			return -EFAULT; /* method not available */
		if (unlikely(rtdm_safe_copy_from_user(user_info, &transfer, 
						umem, sizeof(transfer)) != 0))
			return -EFAULT;
		if (unlikely(transfer.size == 0))
			return 0; /* nothing to do */
		if (unlikely(transfer.size > SPI_RT_HEAP_SIZE))
			return -EINVAL; /* invalid parameter for this ioctl */

		/*
		 * If one of mosi or miso exists, a buffer is needed to copy to/from the
		 * user. If neither exists, all data will be discarded, passing NULL to
		 * xmit for both parameters.
		 */
		if (likely(transfer.mosi || transfer.miso)) {
			if (rt_heap_alloc(&dev->heap, transfer.size, 
						TM_INFINITE, (void **)&buf) != 0)
				return -ENOMEM;
		}
		/* get mosi from user only if valid */
		if (likely(transfer.mosi)) {
			if (rtdm_safe_copy_from_user(user_info, buf, 
						transfer.mosi, transfer.size) != 0) {
				errval = -EFAULT;
				goto cleanup;
			}
		}
		/* pass the data to the controller driver for action */
		errval = spi->xmit(spi, transfer.mosi ? buf : NULL,
				transfer.miso ? buf : NULL,
				transfer.size);
		if (unlikely(errval != 0)) {
			errval = -EFAULT;
			goto cleanup;
		}

		/* data has been transferred at this point */
		/* copy back miso if the user wants it */
		if (transfer.miso) {
			if (rtdm_safe_copy_to_user(user_info, transfer.miso, buf,
					transfer.size) != 0)
				errval = -EFAULT;
		}
cleanup:
		if (buf)
			rt_heap_free(&dev->heap, buf);
		return errval;
		break;
	} /* XMIT */

	case XMITMSG:
	{
		struct spi_message_s msg;
		int errval = 0;
		spi_data *buf1;
		spi_data *buf2;
		spi_data *tmp_miso1;
		spi_data *tmp_miso2;

		if (unlikely(!rtdm_in_rt_context()))
			return -EPERM;
		if (unlikely(!spi->xmitmsg))
			return -EFAULT; /* method not available */
		if (unlikely(rtdm_safe_copy_from_user(user_info, &msg, umem, sizeof(msg)) != 0))
			return -EFAULT;
		if (unlikely((msg.xfer_1.size + msg.xfer_2.size) > SPI_RT_HEAP_SIZE))
			return -EINVAL; /* invalid parameter for this ioctl */

		/*
		 * If one of mosi or miso exists, a buffer is needed to copy to/from the
		 * user. If neither exists, all data will be discarded, passing NULL to
		 * xmit for both parameters.
		 */
		if (likely(msg.xfer_1.mosi || msg.xfer_1.miso
					|| msg.xfer_2.mosi || msg.xfer_2.miso))
		{
			if (rt_heap_alloc(&dev->heap, msg.xfer_1.size + msg.xfer_2.size,
						TM_INFINITE, (void **)&buf1) != 0)
				return -ENOMEM;
		}
		/*
		 * pointing buf2 to a section of buf1 allows allocation with
		 * only one rt_head_alloc call, reducing latency
		 */
		buf2 = buf1 + msg.xfer_1.size;

		/* get mosi from user only if valid */
		if (likely(msg.xfer_1.mosi)) {
			if (rtdm_safe_copy_from_user(user_info, buf1, msg.xfer_1.mosi,
						msg.xfer_1.size) != 0) {
				errval = -EFAULT;
				goto cleanup2;
			}
		}
		if (likely(msg.xfer_2.mosi)) {
			if (rtdm_safe_copy_from_user(user_info, buf2, msg.xfer_2.mosi,
						msg.xfer_2.size) != 0) {
				errval = -EFAULT;
				goto cleanup2;
			}
		}
		/*
		 * replace any userspace pointers with kernel buffers, saving
		 * the userspace pointers for copying after transfer
		 */
		tmp_miso1 = msg.xfer_1.miso;
		tmp_miso2 = msg.xfer_2.miso;

		msg.xfer_1.miso = (msg.xfer_1.miso ? buf1 : NULL);
		msg.xfer_1.mosi = (msg.xfer_1.mosi ? buf1 : NULL);

		msg.xfer_2.miso = (msg.xfer_2.miso ? buf2 : NULL);
		msg.xfer_2.mosi = (msg.xfer_2.mosi ? buf2 : NULL);

		/* all pointers have been replaced, the transfer may be
		 * submitted to the SPI driver
		 */

		errval = spi->xmitmsg(spi, &msg);
		if (unlikely(errval != 0)) {
			errval = -EFAULT;
			goto cleanup2;
		}

		/* data has been transferred at this point */
		/* copy back miso if the user wants it */
		if (tmp_miso1) {
			if (rtdm_safe_copy_to_user(user_info, tmp_miso1, buf1,
					msg.xfer_1.size) != 0)
			{
				errval = -EFAULT;
			}
		}
		if (tmp_miso2) {
			if (rtdm_safe_copy_to_user(user_info, tmp_miso2, buf2,
					msg.xfer_2.size) != 0)
			{
				errval = -EFAULT;
			}
		}
cleanup2:
		if (buf1)
			rt_heap_free(&dev->heap, buf1);
		return errval;
		break;

	} /* XMITMSG */

	case CONFREAD:
		if (!spi->confread)
			return -EFAULT;
		kmem[0] = spi->confread(spi);

		if (rtdm_safe_copy_to_user(user_info, umem, kmem, sizeof(spi_control)) != 0)
			return -EFAULT;
		return 0;
		break;
	case CONFWRITE:
		if (!spi->confwrite)
			return -EFAULT;

		if (rtdm_safe_copy_from_user(user_info, kmem, umem, sizeof(spi_control)) != 0)
			return -EFAULT;
		return spi->confwrite(spi, kmem[0]);
		break;
	case SPEEDREAD:
		if (!spi->speedread)
			return -EFAULT;
		kmem[0] = spi->speedread(spi);

		if (rtdm_safe_copy_to_user(user_info, umem, kmem, sizeof(spi_control)) != 0)
			return -EFAULT;
		return 0;
		break;
	case SPEEDWRITE:
		if (!spi->speedwrite)
			return -EFAULT;

		if (rtdm_safe_copy_from_user(user_info, kmem, umem, sizeof(spi_control)) != 0)
			return -EFAULT;
		return spi->speedwrite(spi, kmem[0]);
		break;
	case TIPREAD:
		/*
		 * note that the atomic_spi_tipread/write functions are used for
		 * TIP control. These should be very fast operations. Also note
		 * that TIP is unimplemented in most drivers, with locking control
		 * at the controller level only.
		 */
		if (!spi->tip)
			return -EFAULT;
		kmem[0] = atomic_spi_tip_read(spi);

		if (rtdm_safe_copy_to_user(user_info, umem, kmem, sizeof(spi_control)) != 0)
			return -EFAULT;
		return 0;
		break;
	case TIPWRITE:
		if (!spi->tip)
			return -EFAULT;

		if (rtdm_safe_copy_from_user(user_info, kmem, umem, sizeof(spi_control)) != 0)
			return -EFAULT;
		return atomic_spi_tip_write(spi, kmem[0]);
		break;
	default:
		break;
	}

	return -ENOTTY;
}

static const struct rtdm_device device_tmpl = {
      struct_version:RTDM_DEVICE_STRUCT_VER,

      device_flags:RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
      device_name:"",
      open_nrt:rt_spi_open,
      ops:{
	      close_nrt:rt_spi_close,
	      ioctl_rt:rt_spi_ioctl,
	      ioctl_nrt:rt_spi_ioctl,
	 },
      device_class:RTDM_CLASS_SPI,
      driver_name:"spi_rtdm",
      driver_version:RTDM_DRIVER_VER(1, 0, 0),
      peripheral_name:"spi",
      provider_name:"EMAC.Inc",
};

int rt_spi_device_create(struct spi_s *spi)
{
	int ret;
	struct rtspi_device_s *dev = kmalloc(sizeof(struct rtspi_device_s), GFP_KERNEL);
	dev->spi = spi;

	memcpy(&dev->rtd, &device_tmpl, sizeof(struct rtdm_device));
	strncpy(dev->rtd.device_name, dev->spi->name, RTDM_MAX_DEVNAME_LEN);
	dev->rtd.device_sub_class = dev->spi->subclass;
	dev->rtd.proc_name = dev->spi->name;

	if (rtdm_dev_register(&dev->rtd)) {
		printk(KERN_ERR "couldn't register rtspi device %s\n",
		       dev->rtd.device_name);
		kfree(dev);
		return -1;
	}
	/* create memory heap */
	if ((ret = rt_heap_create(&dev->heap, dev->spi->name,
			SPI_RT_HEAP_SIZE, H_FIFO | H_DMA)) < 0)
	{
		printk(KERN_ERR "error allocating rtspi heap %s\n",
				dev->rtd.device_name);
		kfree(dev);
		return ret;
	}
	return 0;
}
