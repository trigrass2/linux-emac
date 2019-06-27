/**
 * A character device interface to spi devices. Part of the SPI class.
 * 
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/cdev.h>
#include <linux/class/spi.h>
#include <linux/class/char/spi_char.h>
#include <asm/uaccess.h>

static int spi_major =    SPI_MAJOR;
static int spi_minor =    0;
static int spi_num_devs = SPI_MAX_DEVS;

struct spi_char_dev {
    spi_t *spi;
    struct cdev cdev;
};

//#define DEBUG_SC

#ifdef DEBUG_SC
#define DPRINTK(string, args...) printk("spi_char: " string, ##args)
#else
#define DPRINTK(string, args...)
#endif

/* function prototypes */
static long spi_char_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg);
static int spi_char_open(struct inode *inode, struct file *file);
static int spi_char_release(struct inode *inode, struct file *file);
static int spi_char_setup_cdev(struct spi_char_dev *dev);

/* struct for fops declarations */
static const struct file_operations spi_char_fops =
{ 
    .unlocked_ioctl   = spi_char_ioctl,
    .open    = spi_char_open,
    .release = spi_char_release,
};

static int spi_char_open(struct inode *inode, struct file *file)
{
    struct spi_char_dev *dev; /* device information */
    DPRINTK("spi_char_open\n");
    dev = container_of(inode->i_cdev, struct spi_char_dev, cdev);
    file->private_data = dev;
    return 0;
}

static int spi_char_release(struct inode *inode, struct file *file)
{
    DPRINTK("spi_char_release\n");
    /* nothing to do here */
    return 0;
}

static long spi_char_ioctl(struct file *file,
        unsigned int cmd, unsigned long arg)
{
    struct spi_char_dev *dev = container_of(file->f_dentry->d_inode->i_cdev, struct spi_char_dev, cdev);
    spi_t *spi = dev->spi;
    spi_control kmem[1];
    
    /* make sure that this command is indeed one of spi_char's */
    if (_IOC_TYPE(cmd) != CHAR_CLASS_SPI)
        return -ENOTTY;
    
    switch(cmd)
    {
    case XMIT: {
    	spi_transfer_t *t = NULL;
    	spi_data *buf = NULL;
    	int errval = 0;

        DPRINTK("XMIT ioctl\n");

        if (!spi->xmit)
        	return -EFAULT; /* method not available */

        if (!(t = kzalloc(sizeof(spi_transfer_t), GFP_KERNEL)))
        	return -ENOMEM;

        if (copy_from_user(t, (void *)arg, sizeof(spi_transfer_t)) != 0) {
        	errval =- EFAULT;
        	goto cleanup;
        }
        if (t->size == 0) {
        	/* nothing to do */
        	goto cleanup;
        }
        /* If one of mosi or miso exists, we will need a buffer to
         * copy to/from the user. If neither exists, all data will be
         * discarded, passing NULL to xmit for both parameters.
         */
        if ((t->miso) || (t->mosi)) {
        	buf = kmalloc(t->size, GFP_KERNEL);
        }
        if(t->mosi) {
        	if (copy_from_user(buf, t->mosi, t->size) != 0) {
        		errval =- EFAULT;
        		goto cleanup;
        	}
        }
        /* call into the driver to transfer data to the device. If both mosi and
         * miso were defined from the user, they will each be passed as buf to
         * the low-level driver.
         */
        if (atomic_spi_xmit(spi, t->mosi ? buf : NULL, t->miso ? buf : NULL, t->size) != 0) {
        	errval =- EFAULT;
        	goto cleanup;
        }
        if (t->miso) {
        	if (copy_to_user(t->miso, buf, t->size) != 0) {
        		errval =- EFAULT;
        		goto cleanup;
        	}
        }
cleanup:
        if (buf)
        	kfree(buf);
        if (t)
        	kfree(t);
        return errval;
        break;
    } /* XMIT */
    case CONFREAD:
        DPRINTK("CONFREAD ioctl\n");

        if (spi->confread)
            kmem[0] = atomic_spi_conf_read(spi);
        else
            return -EFAULT;
        return (copy_to_user((void *)arg, kmem, sizeof(spi_control)) == 0 ) ? 0 : -EFAULT;
        break;
    case CONFWRITE:
        DPRINTK("CONFWRITE ioctl\n");
        if (copy_from_user(kmem, (void *)arg, sizeof(spi_control)) != 0)
            return -EFAULT;
        if (spi->confwrite)
            atomic_spi_conf_write(spi, kmem[0]);
        else 
            return -EFAULT;
        return 0;
        break;
    case SPEEDREAD:
        DPRINTK("SPEEDREAD ioctl\n");
        if (spi->speedread)
            kmem[0] = atomic_spi_speed_read(spi);
        else
            return -EFAULT;
        return (copy_to_user((void *)arg, kmem, sizeof(spi_control)) == 0 ) ? 0 : -EFAULT;
        break;
    case SPEEDWRITE:
        DPRINTK("SPEEDWRITE ioctl\n");
        if (copy_from_user(kmem, (void *)arg, sizeof(spi_control)) != 0)
            return -EFAULT;
        if (spi->speedwrite)
            atomic_spi_speed_write(spi, kmem[0]);
        else {
            DPRINTK("error: invalid speed write\n");
            return -EFAULT;
        }
        return 0;
        break;
    case TIPREAD:
        DPRINTK("TIPREAD ioctl\n");
        if (spi->tip)
            kmem[0] = atomic_spi_tip_read(spi);
        else
            return -EFAULT;
        return (copy_to_user((void *)arg, kmem, sizeof(spi_control)) == 0 ) ? 0 : -EFAULT;
        break;
    case TIPWRITE:
        DPRINTK("TIPWRITE ioctl\n");
        if (copy_from_user(kmem, (void *)arg, sizeof(spi_control)) != 0)
            return -EFAULT;
        if (spi->tip)
            atomic_spi_tip_write(spi, kmem[0]);
        else {
            DPRINTK("error: invalid tip write\n");
            return -EFAULT;
        }
        return 0;
        break;
    default :
        DPRINTK("ioctl: no such command\n");
        return -ENOTTY;
    } /* END switch(cmd) */
    DPRINTK("Invalid state, broke out of switch\n");
    return -EFAULT; /* Error, we should never reach here */    
}

/* initialize the character interface -- called by spi class on init */

int spi_char_init(void)
{
    int result;
    dev_t dev = 0;
    DPRINTK("spi_char_init\n");
    /* dynamic and static character device allocation */
    if (spi_major) {
        dev = MKDEV(spi_major, spi_minor);
        result = register_chrdev_region(dev, spi_num_devs, "spi_char");
    }
    else {
        result = alloc_chrdev_region(&dev, spi_minor, 
                spi_num_devs, "spi_char");
        spi_major = MAJOR(dev);
    }
    if (result < 0)
        printk(KERN_WARNING "spi_char: can't get major %d, err %d\n", spi_major, result);
    
    return result;
}

EXPORT_SYMBOL(spi_char_init);

/* registers the actual char device, called by create when invoked by spi class */

static int spi_char_setup_cdev(struct spi_char_dev *dev)
{
    static int index = 0;
    int err;
    int devno = MKDEV(spi_major, spi_minor + index);
    
    DPRINTK("spi_char_setup_cdev\n");
    
    cdev_init(&dev->cdev, &spi_char_fops);
    dev->cdev.owner = THIS_MODULE;
    dev->cdev.ops = &spi_char_fops;
    dev->spi->cdev = &dev->cdev;
    err = cdev_add(&dev->cdev, devno, 1);
    if (err)
        printk(KERN_NOTICE "spi_char: Error %d adding spi_char%d\n", err, index);
    
    index++;
    return devno;
}

/* create and register a new char device */

int spi_char_create(struct spi_s *spi)
{
    struct spi_char_dev *chardev = kmalloc(sizeof(struct spi_char_dev), GFP_KERNEL);
    DPRINTK("spi_char_create\n");
    if (!chardev) {
        printk(KERN_NOTICE "Error allocating memory\n");
        return -ENOMEM;
    }
    chardev->spi = spi;
    
    return spi_char_setup_cdev(chardev);
}

EXPORT_SYMBOL(spi_char_create);

void spi_char_destroy(struct spi_s *spi)
{
	struct spi_char_dev *dev =
			container_of(spi->cdev, struct spi_char_dev, cdev);
	cdev_del(spi->cdev);
	kfree(dev);
}

EXPORT_SYMBOL(spi_char_destroy);

void spi_char_remove(void)
{
	dev_t devno;
	devno = MKDEV(spi_major, spi_minor);

	unregister_chrdev_region(devno, spi_num_devs);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("EMAC. Inc");
