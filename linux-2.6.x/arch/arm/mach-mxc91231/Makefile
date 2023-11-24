#
# Makefile for the linux kernel.
#

# Object file lists.

obj-y			:= devices.o iomux.o system.o cpu.o mm.o time.o clock.o serial.o
obj-$(CONFIG_MACH_MXC27530EVB)	+= mxc27530evb.o mxc27530evb_gpio.o
obj-$(CONFIG_MACH_SCMA11REF)	+= scma11phone.o scma11ref_gpio.o
obj-$(CONFIG_MACH_ASCENSION)	+= scma11phone.o ascension_gpio.o
obj-$(CONFIG_MACH_LIDO)	        += scma11phone.o lido_gpio.o
obj-$(CONFIG_MACH_SAIPAN)       += scma11phone.o saipan_gpio.o
obj-$(CONFIG_MACH_ELBA)         += scma11phone.o elba_gpio.o
obj-$(CONFIG_MACH_PICO)		+= scma11phone.o pico_gpio.o

obj-$(CONFIG_MOT_FEAT_GPIO_API) += mot-gpio/

obj-y				+= mxc_pm.o

# Power management
obj-$(CONFIG_PM) 		+= pm.o
obj-$(CONFIG_DPM)		+= dpm.o

obj-$(CONFIG_MOT_FEAT_DEBUG_WDOG) += wdog2_fiq.o
