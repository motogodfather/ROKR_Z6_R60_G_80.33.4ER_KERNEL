# ###########################################################################
# Makefile - generation of hardhat (kernel) component
# Copyright 2006-2007 Motorola, Inc.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
#
# Date         Author          Comment
# ===========  ==============  ==============================================
# 09-Nov-2006  Motorola        Initial revision.
# 22-Nov-2006  Motorola        TOOLPREFIX dump functionality.
# 06-Dec-2006  Motorola        Added FEAT_ETM check.
# 15-Dec-2006  Motorola        Added memory dump support (DBG_MEMDUMP)
# 01-Jan-2007  Motorola        Removed an inccorect export
# 01-Jan-2007  Motorola        Fixed API rebuild issues, added -j4 to kernel
# 24-Jan-2007  Motorola        Added FEAT_PTRACE
# 07-Feb-2007  Motorola        Export light_sensor.h.
# 02-Mar-2007  Motorola        Exported motfb.h and mxcfb.h
# 05-May-2007  Motorola        Added TURBO_INDICATOR
# ###########################################################################

include $(BOOTSTRAP)

# ###########################################################################
# Variable Setup
# ###########################################################################

KERNEL_VERSION = 2.6
LINUXROOT = $(COMPTOP)/linux-$(KERNEL_VERSION).x

PRODUCT_DIR := $(BUILDTOP)/$(ARCH)/$(PROC_TYPE)/$(PRODUCT)
PRODUCT_ROOTFS := $(PRODUCT_DIR)/fs

LINUXBUILD = $(PRODUCT_DIR)/linux_build

LINUXBUILD_KERNEL = $(PRODUCT_DIR)/kernel/linux_build

API_INCS = $(COMPTOP)/linux-2.6.x/include//linux/moto_accy.h \
	$(COMPTOP)/linux-2.6.x/include//linux/camera.h \
	$(COMPTOP)/linux-2.6.x/include//linux/videodev.h \
	$(COMPTOP)/linux-2.6.x/include//linux/lights_funlights.h \
	$(COMPTOP)/linux-2.6.x/include//linux/light_sensor.h \
	$(COMPTOP)/linux-2.6.x/include//linux/power_ic.h \
	$(COMPTOP)/linux-2.6.x/include//linux/lights_backlight.h \
	$(COMPTOP)/linux-2.6.x/include//linux/usr_blk_dev.h \
	$(COMPTOP)/linux-2.6.x/include//linux/keypad.h \
	$(COMPTOP)/linux-2.6.x/include//linux/sdhc_user.h \
	$(COMPTOP)/linux-2.6.x/include//linux/keyv.h \
	$(COMPTOP)/linux-2.6.x/include//linux/sahara/sahara.h \
	$(COMPTOP)/linux-2.6.x/include//linux/sahara/fsl_shw.h \
	$(COMPTOP)/linux-2.6.x/include//linux/sahara/fsl_platform.h \
	$(COMPTOP)/linux-2.6.x/include//linux/sahara/sah_kernel.h \
	$(COMPTOP)/linux-2.6.x/include//linux/sahara/sf_util.h \
	$(COMPTOP)/linux-2.6.x/include//linux/sahara/adaptor.h \
	$(COMPTOP)/linux-2.6.x/drivers/mxc/apal/apal_driver.h \
        $(COMPTOP)/linux-2.6.x/include/asm-arm/arch-mxc//linux/mxc_mu.h \
        $(COMPTOP)/linux-2.6.x/include/asm-arm/arch-mxc//linux/mxc_ipc.h \
        $(COMPTOP)/linux-2.6.x/include//asm-arm/setup.h \
        $(COMPTOP)/linux-2.6.x/include//linux/motfb.h \
        $(COMPTOP)/linux-2.6.x/include//linux/mxcfb.h

# don't build kernel for x86
ifneq($(HW_ARCH),i686)

API_DIRS = $(LINUXBUILD) $(COMPTOP)/kernel_include

DOTCONFIG = $(LINUXBUILD)/.config

# Parse the full path and prefix for the cross compiler prefix
TOOLPREFIX = $(patsubst %-gcc,%-,$(CC))
TOOLPREFIX_DUMP = $(LINUXBUILD)/toolprefix.mk

# ###########################################################################
# DYNAMIC GENERATION OF KERNEL CONFIGURATION FILE (defconfig)
#
# For more information, see:
#   /vobs/jem/hardhat/linux-2.6.x/arch/arm/configs/motorola_ljap/README.txt
# ###########################################################################

TARGET_DEFCONFIG    := ${PRODUCT_DIR}/${PRODUCT}_defconfig
DEFCONFIGSRC        := ${LINUXROOT}/arch/${ARCH}/configs
LJAPDEFCONFIGSRC    := ${DEFCONFIGSRC}/motorola_ljap

# build list of files to be concatenated together to make the linux defconfig
PRODUCT_SPECIFIC_DEFCONFIGS := \
    ${DEFCONFIGSRC}/motorola_ljap_defconfig \
    ${LJAPDEFCONFIGSRC}/product-family/${PRODUCT_FAMILY}.config \
    ${LJAPDEFCONFIGSRC}/product/${PRODUCT}.config \
    ${DEFCONFIGSRC}/${PRODUCT}_defconfig

# Build the small page NAND YAFFS module or the Large Page NAND YAFFS module.
PRODUCT_SPECIFIC_DEFCONFIGS += \
  ${LJAPDEFCONFIGSRC}/feature/nand_${MEM_MAP_PAGE_SIZE}.config

# compile USBOTG drivers statically to support NFS-over-USB
ifeq($(FEAT_USBOTG),static)
    PRODUCT_SPECIFIC_DEFCONFIGS += \
	${LJAPDEFCONFIGSRC}/feature/usbotgstatic.config \
	${LJAPDEFCONFIGSRC}/feature/usbotgstatic.${PRODUCT_FAMILY}_config \
	${LJAPDEFCONFIGSRC}/feature/usbotgstatic.${PRODUCT}_config
endif

# allow MXC internal UART3 to be used as a serial console
ifeq($(FEAT_CONSOLE),serial)
    PRODUCT_SPECIFIC_DEFCONFIGS += \
	${LJAPDEFCONFIGSRC}/feature/serialconsole.config \
	${LJAPDEFCONFIGSRC}/feature/serialconsole.${PRODUCT_FAMILY}_config \
	${LJAPDEFCONFIGSRC}/feature/serialconsole.${PRODUCT}_config
endif

# SiERRA: Linear Vibrator
ifneq($(HW_LINEARVIBRATOR),0)
    PRODUCT_SPECIFIC_DEFCONFIGS += \
        ${LJAPDEFCONFIGSRC}/feature/linearvibrator.config \
        ${LJAPDEFCONFIGSRC}/feature/linearvibrator.${PRODUCT_FAMILY}_config \
        ${LJAPDEFCONFIGSRC}/feature/linearvibrator.${PRODUCT}_config
endif

# SiERRA: Capacitive Touch
ifneq($(HW_CAPTOUCH),0)
    PRODUCT_SPECIFIC_DEFCONFIGS += \
        ${LJAPDEFCONFIGSRC}/feature/captouch.config \
        ${LJAPDEFCONFIGSRC}/feature/captouch.${PRODUCT_FAMILY}_config \
        ${LJAPDEFCONFIGSRC}/feature/captouch.${PRODUCT}_config
endif

# SiERRA: Ceramic Speaker
ifneq($(HW_CERAMICSPEAKER),0)
    PRODUCT_SPECIFIC_DEFCONFIGS += \
        ${LJAPDEFCONFIGSRC}/feature/ceramicspeaker.config \
        ${LJAPDEFCONFIGSRC}/feature/ceramicspeaker.${PRODUCT_FAMILY}_config \
        ${LJAPDEFCONFIGSRC}/feature/ceramicspeaker.${PRODUCT}_config
endif

ifeq($(DBG_OPROFILE),1)
    PRODUCT_SPECIFIC_DEFCONFIGS += \
        ${LJAPDEFCONFIGSRC}/feature/oprofile.config
endif

ifeq($(DBG_MEMDUMP),1)
    PRODUCT_SPECIFIC_DEFCONFIGS += \
	${LJAPDEFCONFIGSRC}/feature/memdump.config
endif

ifeq($(FEAT_PTRACE),1)
    PRODUCT_SPECIFIC_DEFCONFIGS += \
	${LJAPDEFCONFIGSRC}/feature/ptrace.config \
	${LJAPDEFCONFIGSRC}/feature/ptrace.${PRODUCT_FAMILY}_config \
	${LJAPDEFCONFIGSRC}/feature/ptrace.${PRODUCT}_config
endif

# SiERRA: Proximity Sensor
ifneq($(HW_PROXSENSOR),0)
    PRODUCT_SPECIFIC_DEFCONFIGS += \
	${LJAPDEFCONFIGSRC}/feature/proxsensor.config \
	${LJAPDEFCONFIGSRC}/feature/proxsensor.${PRODUCT_FAMILY}_config \
	${LJAPDEFCONFIGSRC}/feature/proxsensor.${PRODUCT}_config
endif

# Enable ETM IOMUX configuration
ifneq($(FEAT_ETM),0)
    PRODUCT_SPECIFIC_DEFCONFIGS += \
	${LJAPDEFCONFIGSRC}/feature/etm.config \
	${LJAPDEFCONFIGSRC}/feature/etm.${PRODUCT_FAMILY}_config \
	${LJAPDEFCONFIGSRC}/feature/etm.${PRODUCT}_config
endif

ifeq($(TEST_I2CDEV),1)
    PRODUCT_SPECIFIC_DEFCONFIGS += \
	${LJAPDEFCONFIGSRC}/feature/i2cdev.config \
	${LJAPDEFCONFIGSRC}/feature/i2cdev.${PRODUCT_FAMILY}_config \
	${LJAPDEFCONFIGSRC}/feature/i2cdev.${PRODUCT}_config
endif

ifeq($(DBG_TURBO_INDICATOR),1)
    PRODUCT_SPECIFIC_DEFCONFIGS += \
	${LJAPDEFCONFIGSRC}/feature/turbo.config \
	${LJAPDEFCONFIGSRC}/feature/turbo.${PRODUCT_FAMILY}_config \
	${LJAPDEFCONFIGSRC}/feature/turbo.${PRODUCT}_config
endif
		

# filter out any non-existent files
PRODUCT_SPECIFIC_DEFCONFIGS := \
    $(strip $(foreach FILE, $(PRODUCT_SPECIFIC_DEFCONFIGS), \
	$(wildcard $(FILE))))

# Allow user to specify arbitrary files to be concatenated from the command
# line. Do this after filtering for non-existent files so that an error
# will be returned if a specified file doesn't exist.
ifneq($(TEST_LNXOS_DEFCONFIGS),)
    PRODUCT_SPECIFIC_DEFCONFIGS += ${strip ${TEST_LNXOS_DEFCONFIGS}}
endif

# generate a defconfig file
${TARGET_DEFCONFIG}: ${PRODUCT_SPECIFIC_DEFCONFIGS} 
	# generate an error message if no config files are defined
ifeq($(PRODUCT_SPECIFIC_DEFCONFIGS),)
	@echo "No defconfig file defined for PRODUCT=$(PRODUCT))"
	false
endif
	mkdir -p $(PRODUCT_DIR)
	@( perl -le 'print "# This file was automatically generated from:\n#\t" . join("\n#\t", @ARGV) . "\n"' $(PRODUCT_SPECIFIC_DEFCONFIGS) && cat $(PRODUCT_SPECIFIC_DEFCONFIGS) ) > $@ || ( rm -f $@ && false )


# ###########################################################################
# Kernel Build Targets
# ###########################################################################

.PHONY: distclean kernel modules kernel_clean

# don't build anything in parallel, needed for kernel_clean and $(DOTCONFIG)
.NOTPARALLEL: 

# kernel clean must come first, but cannot be a dependency
api_build: $(DOTCONFIG)

impl: kernel modules

# avoid partial winkins of .config when kernel is not properly cleaned
ifneq(_$(NO_KERNEL_CLEAN),_yes)
.NO_CONFIG_REC: kernel modules
else
.NO_CONFIG_REC: kernel modules $(DOTCONFIG)
endif

$(DOTCONFIG): $(TARGET_DEFCONFIG)
ifneq(_$(NO_KERNEL_CLEAN),_yes)
	# clean out config area to prevent partial builds from winking in
	rm -rf $(LINUXBUILD)
endif
	mkdir -p $(LINUXBUILD)
	cd $(LINUXROOT) && env -u MAKECMDGOALS make MAKE=make ARCH=$(ARCH) CROSS_COMPILE=$(TOOLPREFIX) O=$(@D) MOT_KBUILD_DEFCONFIG=$(TARGET_DEFCONFIG) defconfig modules_prepare
	# Build Environment sends the regular, non-metric-collecting compiler,
	# so we can skim off the the toolprefix for kernel and module builds.
	# For other components building modules, we need to save off this value
	# and bring it into the generated Makefile
	@echo "TOOLPREFIX=$(TOOLPREFIX)" > $(TOOLPREFIX_DUMP)

kernel: $(DOTCONFIG)
	# Kernel must make a copy of the config area to prevent the api step from rebuilding unnecessarily
ifneq(_$(NO_KERNEL_CLEAN),_yes)
	# Clean out kernel build area for safety
	rm -rf $(LINUXBUILD_KERNEL)
endif
	mkdir -p $(LINUXBUILD_KERNEL)
	cp -a  $(LINUXBUILD) $(dir $(LINUXBUILD_KERNEL)) 
	# JOBS tells gnumake to use the j option.
	cd $(LINUXBUILD_KERNEL) && $(MAKE) JOBS=-j4 ARCH=$(ARCH) zImage

modules: kernel
	mkdir -p $(PRODUCT_ROOTFS)
	# Build the modules in the kernel area, so that the LINUXBUILD area is not contaminated.
	cd $(LINUXBUILD_KERNEL) && $(MAKE) ARCH=$(ARCH) INSTALL_MOD_PATH=${PRODUCT_ROOTFS} modules modules_install

distclean: clean

endif

