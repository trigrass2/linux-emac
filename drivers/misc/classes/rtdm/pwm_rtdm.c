/**
 * A rtdm interface for pwm classes
 */

#include <linux/kernel.h>
#include <linux/ctype.h>
#include <linux/class/pwm.h>
#include <linux/class/pwm.h>
#include <linux/class/rtdm/pwm_rtdm.h>
#include <rtdm/rtdm_driver.h>

typedef struct rtpwm_device_s {
	struct rtdm_device rtd;
	pwm_t *pwm;
} rtpwm_device_t;

static int rt_pwm_open(struct rtdm_dev_context *context,
                  rtdm_user_info_t *user_info, int oflags){
    return 0;
}

static int rt_pwm_close(struct rtdm_dev_context *context,
                   rtdm_user_info_t *user_info){
return 0;
}

static int rt_pwm_ioctl(struct rtdm_dev_context *context,
		rtdm_user_info_t *user_info, unsigned int request, void __user *umem)
{
	rtpwm_device_t *dev = container_of(context->device, rtpwm_device_t, rtd);
	pwm_t *pwm = dev->pwm;

	pwm_data kmem[1];

	switch (request)
	{
	case PERIODUSREAD:
		if (pwm->periodus_read)
			kmem[0] = atomic_pwm_periodus_read(pwm);
		else
			return -EINVAL;
		return rtdm_safe_copy_to_user(user_info, umem, kmem, sizeof(pwm_data));
		break;

	case PERIODUSWRITE:
		if (rtdm_safe_copy_from_user(user_info, kmem, umem, sizeof(pwm_data)) != 0)
			return -EFAULT;
		if (pwm->periodus_write)
			atomic_pwm_periodus_write(pwm, kmem[0]);
		else
			return -EINVAL;
		return 0;
		break;

	case WIDTHUSREAD:
		if (pwm->widthus_read)
			kmem[0] = atomic_pwm_widthus_read(pwm);
		else
			return -EINVAL;
		return rtdm_safe_copy_to_user(user_info, umem, kmem, sizeof(pwm_data));
		break;

	case WIDTHUSWRITE:
		if (rtdm_safe_copy_from_user(user_info, kmem, umem, sizeof(pwm_data)) != 0)
			return -EFAULT;
		if (pwm->widthus_write)
			atomic_pwm_widthus_write(pwm, kmem[0]);
		else
			return -EINVAL;
		return 0;
		break;

	case WIDTHCTREAD:
		if (pwm->widthcount_read)
			kmem[0] = atomic_pwm_widthcount_read(pwm);
		else
			return -EINVAL;
		return rtdm_safe_copy_to_user(user_info, umem, kmem, sizeof(pwm_data));
		break;

	case WIDTHCTWRITE:
		if (rtdm_safe_copy_from_user(user_info, kmem, umem, sizeof(pwm_data)) != 0)
			return -EFAULT;
		if (pwm->widthcount_write)
			atomic_pwm_widthcount_write(pwm, kmem[0]);
		else
			return -EINVAL;
		return 0;
		break;
	default:
		return -EINVAL;
		break;
	}             
	return -EFAULT;
}


static const struct rtdm_device device_tmpl = {
    struct_version:RTDM_DEVICE_STRUCT_VER,

    device_flags:       RTDM_NAMED_DEVICE | RTDM_EXCLUSIVE,
    device_name:        "",
    open_nrt:           rt_pwm_open,
    ops: {
	close_nrt:		rt_pwm_close,
        ioctl_rt:		rt_pwm_ioctl,
        ioctl_nrt:		rt_pwm_ioctl,
    },
    device_class:       RTDM_CLASS_PWM,
    driver_name:        "pwm_rtdm",
    driver_version:     RTDM_DRIVER_VER(1, 0, 0),
    peripheral_name:    "pwm",
    provider_name:      "EMAC.Inc",
};


int rt_pwm_device_create(struct pwm_s *pwm)
{
	int ret;
	rtpwm_device_t *dev = kmalloc(sizeof(rtpwm_device_t),GFP_KERNEL);
	dev->pwm = pwm;

	memcpy(&dev->rtd, &device_tmpl, sizeof(struct rtdm_device));
	strncpy(dev->rtd.device_name, dev->pwm->name, RTDM_MAX_DEVNAME_LEN);
	dev->rtd.device_sub_class = dev->pwm->subclass;
	dev->rtd.proc_name = dev->pwm->name;

	if((ret = rtdm_dev_register(&dev->rtd)) != 0) {
		printk("couldn't register rtpwm device %s: %d\n", dev->rtd.device_name, ret);
		kfree(dev);
		return ret;
	}
	return 	0;
}


