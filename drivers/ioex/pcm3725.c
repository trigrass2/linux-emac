/***************************************************************************
                          		pcm3725.c    
					PCM-3725 IO expansion driver             
                             -------------------
	author				 : NZG
    copyright          	 : (C) 2007 by EMAC.Inc
    email                : support@emacinc.com
 ***************************************************************************/
 
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/autoconf.h>
#include <linux/ioport.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/class/gpio.h>
#include <asm/io.h>

#define DRV_MODULE_NAME 	"pcm3725"
#define DRV_MODULE_VERSION 	"1.0"

#define NAMELEN	20

#define PDEBUG printk

static int base[4]={0,0,0,0};
module_param_array(base,int,NULL,0);

char *dname[]={"pcm3725_a","pcm3725_b","pcm3725_c","pcm3725_d"};
static int j=0;
struct device *pcmdev[8];

#define MAXBOARDS 4
#define CADDRESS 4

static inline int pcm3725_register(int i,int address,char *ext){
	char *name = kmalloc(NAMELEN,GFP_KERNEL);
	sprintf(name,"%s_%s",dname[i],ext);	
	pcmdev[j++]=gpioio_device_create((void *)address, NULL,name);
	return 0;
}

static int __init pcm3725_init_module(void)
{
	int i;
	struct resource *res;

	printk(KERN_INFO DRV_MODULE_NAME " version " DRV_MODULE_VERSION " loading\n");
	for(i=0;i<MAXBOARDS;i++){      
		if(base[i]==0){return (0);}
		res = request_region(base[i],CADDRESS,dname[i]);/*allocate kernel resource*/ 
		if (!res) {
			PDEBUG("IO %x not available\n", base[i]);
			continue;
		}
		PDEBUG("%s claims IO region %x - %x\n",dname[i],base[i],base[i]+CADDRESS);
		pcm3725_register(i,base[i],"out");
		pcm3725_register(i,base[i]+1,"in");
	}    	
	return 0;
}

static void __exit pcm3725_cleanup_module(void){
	int i;
	for(i=0;i<8;i++)if(pcmdev[i])gpio_erase(pcmdev[i]);
	for(i=0;i<4;i++)if(base[i])release_region(base[i],CADDRESS);			
}

module_init(pcm3725_init_module);
module_exit(pcm3725_cleanup_module);

MODULE_AUTHOR("EMAC Inc.");
MODULE_LICENSE("GPL");
