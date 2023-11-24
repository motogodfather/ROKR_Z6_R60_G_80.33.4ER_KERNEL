/*
 *  linux/drivers/mmc/mmc_sysfs.c
 *
 *  Copyright (C) 2003 Russell King, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  MMC sysfs/driver model support.
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/completion.h>
#include <linux/fs.h>
#include <linux/blkdev.h>
#include <asm/uaccess.h>

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>

#include "mmc.h"

#define dev_to_mmc_card(d)	container_of(d, struct mmc_card, dev)
#define to_mmc_driver(d)	container_of(d, struct mmc_driver, drv)
#define cls_dev_to_mmc_host(d)	container_of(d, struct mmc_host, class_dev)

extern void register_repairfat_callback(void (*callback_fun)(struct super_block *sb));
extern void unregister_repairfat_callback(void);

static void mmc_release_card(struct device *dev)
{
	struct mmc_card *card = dev_to_mmc_card(dev);

	kfree(card);
}

/*
 * This currently matches any MMC driver to any MMC card - drivers
 * themselves make the decision whether to drive this card in their
 * probe method.  However, we force "bad" cards to fail.
 */
static int mmc_bus_match(struct device *dev, struct device_driver *drv)
{
	struct mmc_card *card = dev_to_mmc_card(dev);
	return !mmc_card_bad(card);
}

static int
mmc_bus_hotplug(struct device *dev, char **envp, int num_envp, char *buf,
		int buf_size)
{
	struct mmc_card *card = dev_to_mmc_card(dev);
	struct platform_device *pdev = to_platform_device(card->dev.parent);

	char ccc[13];
	int i = 0;

#define add_env(fmt,val)						\
	({								\
		int len, ret = -ENOMEM;					\
		if (i < num_envp) {					\
			envp[i++] = buf;				\
			len = snprintf(buf, buf_size, fmt, val) + 1;	\
			buf_size -= len;				\
			buf += len;					\
			if (buf_size >= 0)				\
				ret = 0;				\
		}							\
		ret;							\
	})

	for (i = 0; i < 12; i++)
		ccc[i] = card->csd.cmdclass & (1 << i) ? '1' : '0';
	ccc[12] = '\0';

	i = 0;
	add_env("MMC_CCC=%s", ccc);
	add_env("MMC_MANFID=%06x", card->cid.manfid);
	add_env("MMC_NAME=%s", mmc_card_name(card));
	add_env("MMC_OEMID=%04x", card->cid.oemid);
	add_env("SLOT=%d", pdev->id);

	return 0;
}

static int mmc_bus_suspend(struct device *dev, u32 state)
{
	struct mmc_driver *drv = to_mmc_driver(dev->driver);
	struct mmc_card *card = dev_to_mmc_card(dev);
	int ret = 0;

	if (dev->driver && drv->suspend)
		ret = drv->suspend(card, state);
	return ret;
}

static int mmc_bus_resume(struct device *dev)
{
	struct mmc_driver *drv = to_mmc_driver(dev->driver);
	struct mmc_card *card = dev_to_mmc_card(dev);
	int ret = 0;

	if (dev->driver && drv->resume)
		ret = drv->resume(card);
	return ret;
}

static struct bus_type mmc_bus_type = {
	.name		= "mmc",
	.match		= mmc_bus_match,
	.hotplug	= mmc_bus_hotplug,
	.suspend	= mmc_bus_suspend,
	.resume		= mmc_bus_resume,
};


static int mmc_drv_probe(struct device *dev)
{
	struct mmc_driver *drv = to_mmc_driver(dev->driver);
	struct mmc_card *card = dev_to_mmc_card(dev);

	return drv->probe(card);
}

static int mmc_drv_remove(struct device *dev)
{
	struct mmc_driver *drv = to_mmc_driver(dev->driver);
	struct mmc_card *card = dev_to_mmc_card(dev);

	drv->remove(card);

	return 0;
}


/**
 *	mmc_register_driver - register a media driver
 *	@drv: MMC media driver
 */
int mmc_register_driver(struct mmc_driver *drv)
{
	drv->drv.bus = &mmc_bus_type;
	drv->drv.probe = mmc_drv_probe;
	drv->drv.remove = mmc_drv_remove;
	return driver_register(&drv->drv);
}

EXPORT_SYMBOL(mmc_register_driver);

/**
 *	mmc_unregister_driver - unregister a media driver
 *	@drv: MMC media driver
 */
void mmc_unregister_driver(struct mmc_driver *drv)
{
	drv->drv.bus = &mmc_bus_type;
	driver_unregister(&drv->drv);
}

EXPORT_SYMBOL(mmc_unregister_driver);


#define MMC_ATTR(name, fmt, args...)					\
static ssize_t mmc_dev_show_##name (struct device *dev, char *buf)	\
{									\
	struct mmc_card *card = dev_to_mmc_card(dev);			\
	return sprintf(buf, fmt, args);					\
}									\
static DEVICE_ATTR(name, S_IRUGO, mmc_dev_show_##name, NULL)

MMC_ATTR(cid, "%08x%08x%08x%08x\n", card->raw_cid[0], card->raw_cid[1],
	card->raw_cid[2], card->raw_cid[3]);
MMC_ATTR(csd, "%08x%08x%08x%08x\n", card->raw_csd[0], card->raw_csd[1],
	card->raw_csd[2], card->raw_csd[3]);
MMC_ATTR(scr, "%08x%08x\n", card->raw_scr[0], card->raw_scr[1]);
MMC_ATTR(date, "%02d/%04d\n", card->cid.month, card->cid.year);
MMC_ATTR(fwrev, "0x%x\n", card->cid.fwrev);
MMC_ATTR(hwrev, "0x%x\n", card->cid.hwrev);
MMC_ATTR(manfid, "0x%06x\n", card->cid.manfid);
MMC_ATTR(name, "%s\n", card->cid.prod_name);
MMC_ATTR(oemid, "0x%04x\n", card->cid.oemid);
MMC_ATTR(serial, "0x%08x\n", card->cid.serial);

static struct device_attribute *mmc_dev_attributes[] = {
	&dev_attr_cid,
	&dev_attr_csd,
	&dev_attr_scr,
	&dev_attr_date,
	&dev_attr_fwrev,
	&dev_attr_hwrev,
	&dev_attr_manfid,
	&dev_attr_name,
	&dev_attr_oemid,
	&dev_attr_serial,
};

/*
 * Internal function.  Initialise a MMC card structure.
 */
void mmc_init_card(struct mmc_card *card, struct mmc_host *host)
{
	memset(card, 0, sizeof(struct mmc_card));
	card->host = host;
	device_initialize(&card->dev);
	card->dev.parent = card->host->dev;
	card->dev.bus = &mmc_bus_type;
	card->dev.release = mmc_release_card;
}

/*
 * Internal function.  Register a new MMC card with the driver model.
 */
int mmc_register_card(struct mmc_card *card)
{
	int ret, i;

	snprintf(card->dev.bus_id, sizeof(card->dev.bus_id),
		 "%s:%04x", mmc_hostname(card->host), card->rca);

	ret = device_add(&card->dev);
	if (ret == 0)
		for (i = 0; i < ARRAY_SIZE(mmc_dev_attributes); i++)
			device_create_file(&card->dev, mmc_dev_attributes[i]);

	return ret;
}

/*
 * Internal function.  Unregister a new MMC card with the
 * driver model, and (eventually) free it.
 */
void mmc_remove_card(struct mmc_card *card)
{
	if (mmc_card_present(card))
		device_del(&card->dev);

	put_device(&card->dev);
}


static void mmc_host_classdev_release(struct class_device *dev)
{
	struct mmc_host *host = cls_dev_to_mmc_host(dev);
	kfree(host);
}

DECLARE_COMPLETION(fatpanic_completion);
static char device_name[BDEVNAME_SIZE];
static void wakeup_mmc_repairfat(struct super_block *sb)
{
	int part = 0;
	struct block_device *panic_bdev = sb->s_bdev;
	printk("mxclay:repairfat wakeup\n");

	if(panic_bdev && panic_bdev->bd_disk && 
			!(strncmp("mmc", panic_bdev->bd_disk->devfs_name, 3))) {
		printk("mxclay:repairfat %s\n", panic_bdev->bd_disk->devfs_name);
		part = MINOR(panic_bdev->bd_dev) - panic_bdev->bd_disk->first_minor;
		snprintf(device_name, BDEVNAME_SIZE, "/dev/%s/part%d", 
				panic_bdev->bd_disk->devfs_name, part);	
		printk("mxclay:repairfat device= %s\n", device_name);
		complete(&fatpanic_completion);
	}
}

static ssize_t mmc_repairfs_show(struct class_device *dev, char *buf)
{
	printk("mxclay: show\n");
	INIT_COMPLETION(fatpanic_completion);
	wait_for_completion(&fatpanic_completion);

	memcpy(buf, device_name, strlen(device_name));
	printk("mxclay: show end size %d buf %s\n", strlen(buf), buf);

	return strlen(device_name);
}

static ssize_t mmc_repairfs_store(struct class_device *dev, const char *buf, size_t size)
{
	/* for furture use */
	return size;
}

static struct class_device_attribute attr_repairfs[] = {
	__ATTR(repairfs, S_IRUGO | S_IWUGO, mmc_repairfs_show, mmc_repairfs_store),
	__ATTR_NULL
};

static struct class mmc_host_class = {
	.name		= "mmc_host",
	.class_dev_attrs = attr_repairfs,
	.release	= mmc_host_classdev_release,
};

/*
 * Internal function. Allocate a new MMC host.
 */
struct mmc_host *mmc_alloc_host_sysfs(int extra, struct device *dev)
{
	struct mmc_host *host;

	host = kmalloc(sizeof(struct mmc_host) + extra, GFP_KERNEL);
	if (host) {
		memset(host, 0, sizeof(struct mmc_host) + extra);

		host->dev = dev;
		host->class_dev.dev = host->dev;
		host->class_dev.class = &mmc_host_class;
		class_device_initialize(&host->class_dev);
	}

	return host;
}

/*
 * Internal function. Register a new MMC host with the MMC class.
 */
int mmc_add_host_sysfs(struct mmc_host *host)
{
	static unsigned int host_num;

	snprintf(host->host_name, sizeof(host->host_name),
		 "mmc%d", host_num++);

	strlcpy(host->class_dev.class_id, host->host_name, BUS_ID_SIZE);
	return class_device_add(&host->class_dev);
}

/*
 * Internal function. Unregister a MMC host with the MMC class.
 */
void mmc_remove_host_sysfs(struct mmc_host *host)
{
	class_device_del(&host->class_dev);
}

/*
 * Internal function. Free a MMC host.
 */
void mmc_free_host_sysfs(struct mmc_host *host)
{
	class_device_put(&host->class_dev);
}


static int __init mmc_init(void)
{
	int ret = bus_register(&mmc_bus_type);
	if (ret == 0) {
		ret = class_register(&mmc_host_class);
		if (ret)
			bus_unregister(&mmc_bus_type);
	}

	register_repairfat_callback(wakeup_mmc_repairfat);
	return ret;
}

static void __exit mmc_exit(void)
{
	unregister_repairfat_callback();
	class_unregister(&mmc_host_class);
	bus_unregister(&mmc_bus_type);
}

module_init(mmc_init);
module_exit(mmc_exit);
