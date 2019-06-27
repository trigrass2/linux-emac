#
# Makefile for the misc classes
#

EXTRA_CFLAGS += -Iinclude/xenomai
obj-$(CONFIG_GPIOCLASS)		+= gpio_class.o
gpio_class-objs := gpio.o
ifeq ($(CONFIG_GPIOCLASS_CHAR),y)
gpio_class-objs += char/gpio_char.o
endif
obj-$(CONFIG_PWMCLASS)		+= pwm.o
obj-$(CONFIG_SPICLASS)		+= spi_class.o
spi_class-objs := spi.o
ifeq ($(CONFIG_SPICLASS_CHAR),y)
spi_class-objs += char/spi_char.o
endif
obj-$(CONFIG_LSI2ESC)		+= lsi2esc.o
lsi2esc-objs := spi_interface.o
obj-y += rtdm/
obj-y += lsi2esc_devices/
