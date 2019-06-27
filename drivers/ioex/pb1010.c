/***************************************************************************
                          		pb1010.c    
					pb1010 soft core mapping driver             
                             -------------------
	author				 : NZG
    rewrite              : Fri Jun 08 2007
    copyright          	 : (C) 2007 by EMAC.Inc
    email                : support@emacinc.com
 ***************************************************************************/
 
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/ioex/pb1010.h>
#include <linux/platform_device.h>
#include <linux/class/gpio.h>
#include <linux/class/pwm.h>
#include <linux/module.h>
#include <asm/io.h>
#include <linux/of.h>
#include <linux/of_device.h>

#define DRV_MODULE_NAME 	"pb1010"
#define DRV_MODULE_VERSION 	"1.0"

static int  gpio_data_write16(gpio_t *gpio, gpio_data data){iowrite16(data,gpio->data+gpio->index);gpio->shadow=data;return 0;}
static gpio_data gpio_data_read16(gpio_t *gpio){return ioread16(gpio->data+gpio->index);}


static inline struct device *gpio16_indexed_create(void *base,unsigned long size, const char *name){
	gpio_t *gpio = kmalloc(sizeof(gpio_t),GFP_KERNEL);
	memset(gpio,0,sizeof(gpio_t));
	gpio->name = name;
	gpio->subclass = GPIO_SUBCLASS;
	gpio->data = base;
	gpio->index = 0;
	gpio->range=size;
	gpio->data_write = gpio_data_write16;
    gpio->data_read = gpio_data_read16;
	gpio->index_write = gpio_index_write;
    gpio->index_read = gpio_index_read;
    printk("registering indexed gpio device: %s\n",name);
    return gpio_register_device(gpio);
}

static inline struct device *gpio_indexed_create(void *base,unsigned long size, const char *name){
	gpio_t *gpio = kmalloc(sizeof(gpio_t),GFP_KERNEL);
	memset(gpio,0,sizeof(gpio_t));
	gpio->name = name;
	gpio->subclass = GPIO_SUBCLASS;
	gpio->data = base;
	gpio->index = 0;
	gpio->range=size;
	gpio->data_write = gpio_data_write8;
    gpio->data_read = gpio_data_read8;
	gpio->index_write = gpio_index_write;
    gpio->index_read = gpio_index_read;
    printk("registering indexed gpio device: %s\n",name);
    return gpio_register_device(gpio);
}

/**
 * create a generic index and data register pair for accessing the FPGA internals
 */
static inline int generic_map(unsigned long phys_addr,u8 *virt_addr,unsigned long size,const char *name,int flags){
	
	if(request_mem_region(phys_addr,size,name)==NULL){
		printk("could not obtain physical memory at %lx for pb1010 core\n",phys_addr);
		iounmap(virt_addr);
		return -1;
		}			
	//add appropriate class devices for this version to the system
	printk("%s detected at %lx\n",name,phys_addr);
	
#ifdef CONFIG_GPIOCLASS
//gpio index from base to base+0x10000
	if((flags&IORESOURCE_MEM_TYPE_MASK)==IORESOURCE_MEM_16BIT)gpio16_indexed_create((void *)virt_addr,size,name);
	if((flags&IORESOURCE_MEM_TYPE_MASK)==IORESOURCE_MEM_8BIT)gpio_indexed_create((void *)virt_addr,size,name);
#endif
				
return 0;
}


/************************************************************
 * map fpga core into virtual memory and declare device classes
 */
static inline void map_core(unsigned long phys_addr, unsigned long size, int key_offset,const char *name,int flags)
{	
	u8 *virt_addr = ioremap_nocache(phys_addr,size);
	//int version;
	
	if(virt_addr==NULL)printk("could not remap physical memory at %lx for pb1010 core\n",phys_addr);
	else{
		//version =ioread8(&virt_addr[key_offset]);//key currently not implemented.
		//printk("EMAC core version %x detected at %lx\n",version,phys_addr+key_offset );
		generic_map(phys_addr,virt_addr,size,name,flags);
		}
}

static int pb1010_probe(struct platform_device *pdev){
	struct pb1010_data *data;
	struct resource *r;
	int i;

	printk("pb1010_probe\n");	
	if (pdev == NULL)return -ENODEV;
	data = pdev->dev.platform_data;
	
	printk("num resources = %d\n",pdev->num_resources);
		
	for(i=0;i<pdev->num_resources;i++){
	r = &pdev->resource[i];
	map_core(r->start,(r->end-r->start+1),data->key_offset,r->name,r->flags);
	//return 0;
	}
	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id pb1010_dt_ids[] = {
	{ .compatible = "pb1010" },
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, ecoreex_dt_ids);
#endif

//driver currently has no removal method.
static struct platform_driver pb1010_driver = {
	.probe		= pb1010_probe,
	.driver		= {
		.name	= "pb1010",
#if defined(CONFIG_OF)
		.of_match_table = of_match_ptr(pb1010_dt_ids),
#endif
	},
};

static int __init pb1010_init_module(void)
{
	printk(KERN_INFO DRV_MODULE_NAME " version " DRV_MODULE_VERSION " loading\n");
	return platform_driver_register(&pb1010_driver);
}

static void __exit pb1010_cleanup_module(void)
{
	platform_driver_unregister(&pb1010_driver);
}


module_init(pb1010_init_module);
module_exit(pb1010_cleanup_module);
