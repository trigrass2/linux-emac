#include <linux/module.h>	/* for modules */
#include <linux/io.h>
#include <linux/init.h>		/* module_init, module_exit */
#include <linux/slab.h>		/* kmalloc */
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/of.h>

static	u8 *mapped;

struct kobject *example_kobject;
static int CPLD_VERSION;

static ssize_t id_show(struct kobject *kobj, struct kobj_attribute *attr,
                      char *buf)
{
        return sprintf(buf, "%x\n", CPLD_VERSION);
}

static struct kobj_attribute id_attribute =__ATTR(cpld_id, S_IRUGO, id_show,  NULL);

static int emac_pld_probe(struct platform_device *pdev)
{
	int error;
	struct resource *res;
	struct device *dev = &pdev->dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "no memory resource\n");
		return -EINVAL;
	}

	res = request_mem_region(res->start, resource_size(res), pdev->name);
	if (!res) {
		dev_err(dev, "can't request mem region\n");
		return -EBUSY;
	}

	mapped = ioremap(res->start, resource_size(res));
	if (!mapped) {
		dev_err(dev, "can't ioremap mem resource\n");
		error = -ENOMEM;
		goto err_release_mem;
	}

	CPLD_VERSION = ioread8(mapped);
	pr_info("EMAC CPLD Version %x detected at %x\n", CPLD_VERSION,res->start);


	example_kobject = kobject_create_and_add("emac_cpld", kernel_kobj); 
	if(!example_kobject) 
		return -ENOMEM;

	error = sysfs_create_file(example_kobject, &id_attribute.attr); 
	if (error) { 
		pr_debug("failed to create the cpld_id file in /sys/kernel/emac_cpld \n");
	}

    return 0;
err_release_mem:
	release_mem_region(res->start, resource_size(res));
	return error;
}

static int emac_pld_remove(struct platform_device *pdev)
{
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	release_mem_region(res->start, resource_size(res));
	kobject_put(example_kobject);
	iounmap(mapped);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id emac_pld_of_match[] = {
	{ .compatible = "emac,emac-cpld-id" },
	{},
};
#endif

static struct platform_driver emac_pld_device_driver = {
	.probe		= emac_pld_probe,
	.remove		= emac_pld_remove,
	.driver		= {
		.name	= "emac-pld-id",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(emac_pld_of_match),
#endif
	}
};

module_platform_driver(emac_pld_device_driver);

MODULE_DESCRIPTION("EMAC CPLD Version Driver");
MODULE_AUTHOR("EMAC Inc.");
MODULE_LICENSE("GPL");
