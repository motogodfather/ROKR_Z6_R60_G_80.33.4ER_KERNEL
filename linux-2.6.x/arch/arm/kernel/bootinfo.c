/*
 * bootinfo.c: This file contains the bootinfo code.  This code
 *	  provides boot information via /proc/bootinfo.  The
 *	  information currently includes:
 *            the powerup reason
 *        This file also provides EZX compatible interfaces for
 *	  retrieving the powerup reason.  All new user-space consumers
 *	  of the powerup reason should use the /proc/bootinfo
 *	  interface and all kernel-space consumers of the powerup
 *	  reason should use the mot_powerup_reason interface.  The EZX
 *	  compatibility code is deprecated.
 *        
 *
 * Copyright 2006 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Revision History:
 *
 * Date         Author    Comment
 * ----------   --------  -----------
 * 10/06/2006   Motorola  MBM support 
 * 12/14/2006   Motorola  Added new ATAGs.
 *
 */

#include <linux/config.h>

#ifdef CONFIG_MOT_FEAT_BOOTINFO

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <asm/setup.h>
#include <asm/bootinfo.h>

/*
 * DEF_ATAG_ACCESSOR is a macro that creates the static variable and
 * accessor functions for information coming into the kernel via
 * ATAGs.  DEF_ATAG_ACCESSOR is useful only at file scope.
 *
 * type is the C type of the variable that stores the information.
 *      It also is used as the return type for the accessor function
 *      and the argument type for the setting function.
 * name is the string used to construct the variable and accessor
 *      function names.
 * initval is the initial value for the variable.
 *
 * Example:
 * DEF_ATAG_ACCESSOR(u32,coolvar,0) produces this code.
 *      static u32 coolvar = 0;
 *      u32 mot_coolvar(void)
 *      {
 *          return coolvar;
 *      }
 *      void mot_set_coolvar(u32 __coolvar)
 *      {
 *          coolvar = __coolvar;
 *      }
 *      EXPORT_SYMBOL(mot_set_coolvar);
 *      EXPORT_SYMBOL(mot_coolvar);
 */
#define DEF_ATAG_ACCESSOR(type,name,initval)         \
	    static type name = (initval);            \
	    type mot_##name(void)                    \
	    {                                        \
	        return name;                         \
	    }                                        \
	    void mot_set_##name(type __##name)       \
	    {                                        \
	        name = __##name;                     \
	    }                                        \
	    EXPORT_SYMBOL(mot_set_##name);           \
	    EXPORT_SYMBOL(mot_##name);

/*
 * EMIT_BOOTINFO and EMIT_BOOTINFO_STR are used to emit the bootinfo
 * information for data provided via ATAGs.
 *
 * The format for all bootinfo lines is "name : val" and these macros
 * enforce that format.
 *
 * strname is the name printed in the name/val pair.
 * name is the name of the function to call
 *
 * EMIT_BOOTINFO and EMIT_BOOTINFO_STR depend on buf and len to already
 * be defined.
 */
#define EMIT_BOOTINFO(strname,fmt,name)                                    \
	    do {                                                           \
	        len += sprintf(buf+len, strname " : " fmt "\n",            \
				        mot_##name());                     \
	    } while (0)

#define EMIT_BOOTINFO_STR(strname,name)                                    \
	    do {                                                           \
		unsigned char *ptr;                                        \
		ptr = (unsigned char *)mot_##name();                       \
	        if (strlen(ptr) == 0) {                                    \
		    len += sprintf(buf+len, strname " : UNKNOWN\n");       \
		} else {                                                   \
		    len += sprintf(buf+len, strname " : %s\n", ptr);       \
		}                                                          \
	    } while (0)

/*-------------------------------------------------------------------------*/

/*
 * powerup_reason contains the powerup reason provided by the ATAGs when
 * the machine boots.
 *
 * Exported symbols:
 * mot_powerup_reason()             -- returns the powerup reason
 * mot_set_powerup_reason()         -- sets the powerup reason
 */
DEF_ATAG_ACCESSOR(u32,powerup_reason,0)

#define EMIT_POWERUPREASON() \
	    EMIT_BOOTINFO("POWERUPREASON", "0x%08x", powerup_reason)


#ifdef CONFIG_MOT_FEAT_FX2LP_I2C
/*
 * usb_firmware_address contains a pointer to usb hs firmware in RAM.
 * usb_firmware_size contains the size of the firmware partition.
 * usb_firmware_address and usb_firmware_size contain 0 if they are
 * not available.
 *
 * Exported symbols:
 * mot_usb_firmware_address()       -- returns the USB firmware address
 * mot_set_usb_firmware_address()   -- sets the USB firmware address
 * mot_usb_firmware_size()          -- returns the USB firmware size
 * mot_set_usb_firmware_size()      -- sets the USB firmware size
 */
DEF_ATAG_ACCESSOR(u32,usb_firmware_address,0)
DEF_ATAG_ACCESSOR(u32,usb_firmware_size,0)

#define EMIT_USB_FW_ADDRESS() \
	    EMIT_BOOTINFO("USB_FW_ADDRESS", "0x%08x", usb_firmware_address)
#define EMIT_USB_FW_SIZE() \
	    EMIT_BOOTINFO("USB_FW_SIZE", "0x%08x", usb_firmware_size)

#else

#define EMIT_USB_FW_ADDRESS()
#define EMIT_USB_FW_SIZE()

#endif


/*
 * mbm_version contains the MBM version.
 * mbm_loader_version contains the MBM loader version.
 * mbm_version and mbm_loader_version default to 0 if they are
 * not set.
 *
 * Exported symbols:
 * mot_mbm_version()                -- returns the MBM version
 * mot_set_mbm_version()            -- sets the MBM version
 * mot_mbm_loader_version()         -- returns the MBM loader version
 * mot_set_mbm_loader_version()     -- sets the MBM loader version
 */
DEF_ATAG_ACCESSOR(u32,mbm_version,0)
DEF_ATAG_ACCESSOR(u32,mbm_loader_version,0)

#define EMIT_MBM_VERSION() \
	    EMIT_BOOTINFO("MBM_VERSION", "0x%08x", mbm_version)
#define EMIT_MBM_LOADER_VERSION() \
	    EMIT_BOOTINFO("MBM_LOADER_VERSION", "0x%08x", mbm_loader_version)


/*
 * boardid contains the Motorola boardid.  The boardid is a board
 * revision number based on information available on the board.
 * The boardid defaults to -1 (0xffffffff) if it is not set.
 *
 * Exported symbols:
 * mot_boardid()                    -- returns the boardid
 * mot_set_boardid()                -- sets the boardid
 */
DEF_ATAG_ACCESSOR(u32,boardid,-1)

#define EMIT_BOARDID() \
	    EMIT_BOOTINFO("BOARDID", "0x%08x", boardid)


/*
 * flat_dev_tree_address contains the Motorola flat dev tree address.
 * flat_dev_tree_address defaults to -1 (0xffffffff) if it is not set.
 *
 * Exported symbols:
 * mot_flat_dev_tree_address()      -- returns the flat dev tree address
 * mot_set_flat_dev_tree_address()  -- sets the flat dev tree address
 */
DEF_ATAG_ACCESSOR(u32,flat_dev_tree_address,-1)

#define EMIT_FLAT_DEV_TREE_ADDRESS() \
	    EMIT_BOOTINFO("FLAT_DEV_TREE_ADDRESS", "0x%08x", flat_dev_tree_address)


/*
 * ramdisk_start contains the physical starting address of the ramdisk.
 * ramdisk_size contains the physical size in bytes of the ramdisk.
 * ramdisk_start defaults to 0 if it is not set.
 * ramdisk_size defaults to -1 (0xffffffff) if it is not set.
 *
 * These are mirrors of phys_initrd_start and phys_initrd_size, except
 * they survive the boot process.  We only keep them around for
 * reporting via /proc/bootinfo.
 *
 * The CONFIG_MOT_FEAT_RAMDISK does *not* enable the ramdisk.  That is
 * already enabled by Linux.  This ifdef only enabled the reporting of
 * the ramdisk information via /proc/bootinfo.
 *
 * Exported symbols:
 * mot_ramdisk_start()              -- returns the ramdisk start address
 * mot_set_ramdisk_start()          -- sets the ramdisk start address
 * mot_ramdisk_size()               -- returns the ramdisk start address
 * mot_set_ramdisk_size()           -- sets the ramdisk start address
 */
DEF_ATAG_ACCESSOR(u32,ramdisk_start,-1)
DEF_ATAG_ACCESSOR(u32,ramdisk_size,0)

#define EMIT_RAM_DISK_START_ADDRESS() \
	    EMIT_BOOTINFO("RAM_DISK_START_ADDRESS", "0x%08x", ramdisk_start)
#define EMIT_RAM_DISK_SIZE()          \
	    EMIT_BOOTINFO("RAM_DISK_SIZE", "0x%08x", ramdisk_size)


/*
 * flashing_completed contains 1 if flashing has completed and
 * 0 if it has not.
 * flashing_completed defaults to 0 if it is not set.
 *
 * Exported symbols:
 * mot_flashing_completed()         -- returns the ramdisk start address
 * mot_set_flashing_completed()     -- sets the ramdisk start address
 */
DEF_ATAG_ACCESSOR(u32,flashing_completed,0)

#define EMIT_FLASHING_COMPLETED() \
	    EMIT_BOOTINFO("FLASHING_COMPLETED", "0x%08x", flashing_completed)


/*
 * logo_version contains the Motorola logo version string.
 * It is NULL terminated.
 * logo_version defaults to an empty string if it is not set.
 *
 * Exported symbols:
 * mot_logo_version()               -- returns a pointer to the logo version
 * mot_set_logo_version()           -- sets a pointer to the logo version
 */
static u8 logo_version[MOT_LOGO_VERSION_SIZE+1] = {0};

u8 *mot_logo_version(void)
{
	return logo_version;
}

void mot_set_logo_version(const u8 *logo_version_string)
{
	memcpy (logo_version, logo_version_string, MOT_LOGO_VERSION_SIZE);
	logo_version[MOT_LOGO_VERSION_SIZE] = 0;
}

EXPORT_SYMBOL(mot_set_logo_version);
EXPORT_SYMBOL(mot_logo_version);

#define EMIT_LOGO_VERSION() \
	    EMIT_BOOTINFO_STR("LOGO_VERSION", logo_version)


/*
 * memory_type contains the memory type of the memory.
 * See MEMORY_TYPE_* for definitions of what the memory
 * types can be.
 * memory_type defaults to -1 (0xffff) if it is not set.
 *
 * Exported symbols:
 * mot_memory_type()         -- returns the memory type
 * mot_set_memory_type()     -- sets the memory type
 */
DEF_ATAG_ACCESSOR(u16,memory_type,-1)

#define EMIT_MEMORY_TYPE() \
	    EMIT_BOOTINFO("MEMORY_TYPE", "0x%04x", memory_type)


/*
 * battery_status_at_boot indicates the battery status
 * when the machine started to boot.
 * battery_status_at_boot defaults to -1 (0xffff) if the battery
 * status can't be determined.
 *
 * Exported symbols:
 * mot_battery_status_at_boot()         -- returns the battery boot status
 * mot_set_battery_status_at_boot()     -- sets the battery boot status
 */
DEF_ATAG_ACCESSOR(u16,battery_status_at_boot,-1)

#define EMIT_BATTERY_STATUS_AT_BOOT() \
	    EMIT_BOOTINFO("BATTERY_STATUS_AT_BOOT", "0x%04x", \
					battery_status_at_boot)


/*
 * boot_frequency contains the frequency used during boot.
 * See BOOT_FREQUENCY_* for definitions of what the various
 * frequencies can be.
 * boot_frequency defaults to -1 (0xffffffff) if it is not set.
 *
 * Exported symbols:
 * mot_boot_frequency()         -- returns the boot frequency
 * mot_set_boot_frequency()     -- sets the boot frequency
 */
DEF_ATAG_ACCESSOR(u32,boot_frequency,-1)

#define EMIT_BOOT_FREQUENCY() \
	    EMIT_BOOTINFO("BOOT_FREQUENCY", "0x%08x", boot_frequency)


/*
 * get_bootinfo fills in the /proc/bootinfo information.
 * We currently only have the powerup reason.
 */
static int get_bootinfo(char *buf, char **start,
                        off_t offset, int count,
                        int *eof, void *data)
{
    int len = 0;

    EMIT_POWERUPREASON();
    EMIT_USB_FW_ADDRESS();
    EMIT_USB_FW_SIZE();
    EMIT_MBM_VERSION();
    EMIT_MBM_LOADER_VERSION();
    EMIT_BOARDID();
    EMIT_FLAT_DEV_TREE_ADDRESS();
    EMIT_RAM_DISK_START_ADDRESS();
    EMIT_RAM_DISK_SIZE();
    EMIT_FLASHING_COMPLETED();
    EMIT_LOGO_VERSION();
    EMIT_MEMORY_TYPE();
    EMIT_BATTERY_STATUS_AT_BOOT();
    EMIT_BOOT_FREQUENCY();

    return len;
}

static struct proc_dir_entry *proc_bootinfo = NULL;
extern struct proc_dir_entry proc_root;

int __init bootinfo_init_module(void)
{

    proc_bootinfo = &proc_root;
    proc_bootinfo->owner = THIS_MODULE;
    create_proc_read_entry("bootinfo", 0, NULL, get_bootinfo, NULL);
    return 0;
}

void __exit bootinfo_cleanup_module(void)
{
    if (proc_bootinfo) {
        remove_proc_entry("bootinfo", proc_bootinfo);
        proc_bootinfo = NULL;
    }
}

module_init(bootinfo_init_module);
module_exit(bootinfo_cleanup_module);

#ifdef CONFIG_MOT_FEAT_POWERUP_REASON_EZX_COMPAT

/*
 * get_powerup_info_ezx_compat fills in the /proc/powerup_info information.
 */
static int get_powerup_info_ezx_compat(char *buf, char **start,
                                       off_t offset, int count,
                                       int *eof, void *data)
{
    int len = 0;

    len += sprintf(buf, "%08x", mot_powerup_reason());
    return len;
}

static struct proc_dir_entry *proc_powerup_info_ezx_compat = NULL;
extern struct proc_dir_entry proc_root;

int __init powerup_info_ezx_compat_init_module(void)
{

    proc_powerup_info_ezx_compat = &proc_root;
    proc_powerup_info_ezx_compat->owner = THIS_MODULE;
    create_proc_read_entry("powerup_info", 0, NULL,
			   get_powerup_info_ezx_compat, NULL);
    return 0;
}

void __exit powerup_info_ezx_compat_cleanup_module(void)
{
    if (proc_powerup_info_ezx_compat) {
        remove_proc_entry("powerup_info", proc_powerup_info_ezx_compat);
        proc_powerup_info_ezx_compat = NULL;
    }
}

module_init(powerup_info_ezx_compat_init_module);
module_exit(powerup_info_ezx_compat_cleanup_module);

#endif /* CONFIG_MOT_FEAT_POWERUP_REASON_EZX_COMPAT */

MODULE_AUTHOR("MOTOROLA");
#endif /* CONFIG_MOT_FEAT_BOOTINFO */
