/*
 * Copyright (C) 2006 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 *
 * Motorola 2006-Oct-06 - Update File
 * Motorola 2006-Jul-31 - Update comments
 * Motorola 2006-Jul-26 - Add TransFlash support to BUTE
 * Motorola 2006-Jun-28 - Montavista header upmerge fixes
 * Motorola 2006-Jun-19 - Fix montavista upmerge conditionals
 * Motorola 2006-Jun-14 - Fix TransFlash detection on Ascension
 * Motorola 2006-May-12 - Add Zeus support for SD
 * Motorola 2006-Apr-04 - Confine all GPIO functions to this file
 */

#ifndef __GPIO_H__
#define __GPIO_H__

 /*!
 * @file gpio.h
 *
 * @ingroup poweric_core
 *
 * @brief The main interface to GPIOs.
 *
 * This file contains all the function declarations for GPIOs
 */

#include <linux/lights_funlights.h>

#include <asm/mot-gpio.h>
#ifdef CONFIG_ARCH_MXC91321
#include <asm/arch/mxc91321.h>
#include <asm/arch/mxc91321_pins.h>
#elif defined(CONFIG_ARCH_MXC91231)
#include <asm/arch/mxc91231.h>
#include <asm/arch/mxc91231_pins.h>
#endif
#include "sdhc_main.h"
/******************************************************************************
* Local constants
******************************************************************************/
/*!
 * @brief Used to prevent unused variable warnings during compilation
 */
#define POWER_IC_UNUSED __attribute((unused))

/*!
 * @brief define GPIO for stereo emu headset
 */
#define GPIO99_ST_HST  99

#if defined(CONFIG_ARCH_MXC91231)
#define PM_INT      INT_EXT_INT5
#define PM_EDIO     ED_INT5
#elif defined(CONFIG_MACH_ARGONLVREF)
#define PM_INT      INT_EXT_INT1
#define PM_EDIO     ED_INT1
#endif /* ArgonLV */

/*!@cond INTERNAL */
#ifdef CONFIG_ARCH_MXC91131
/*! 
 * @brief Pin to which the media card detect switch is attached.
 */
#define USR_BLK_DEV_DEV1_INT_GPIO 16

/*!
 * @brief Returns non zero if the media card is attached.
 *
 * The gpio pin is low when the card is attached and high when it is removed.
 */
#define GPIO_DEV1_ATTACHED \
    ((gpio_get_data(1, USR_BLK_DEV_DEV1_INT_GPIO)) == 0)

#else

/*! @brief Port to which the media card detect switch is attached on BUTE. */
#define GPIO_MCU_PIN_SD1_DET_B    17

#endif
/*!@endcond */

/******************************************************************************
* Macros
******************************************************************************/
/* Define GPIOs specific to SCMA11 */
#if defined(CONFIG_ARCH_MXC91231)
#define EMU_GET_D_PLUS() \
({\
    ((gpio_get_data(GPIO_AP_C_PORT, 16)) ? \
        EMU_BUS_SIGNAL_STATE_ENABLED : EMU_BUS_SIGNAL_STATE_DISABLED); \
})

#define EMU_GET_D_MINUS() \
({\
    ((gpio_get_data(GPIO_AP_C_PORT, 17)) ? \
      EMU_BUS_SIGNAL_STATE_ENABLED : EMU_BUS_SIGNAL_STATE_DISABLED); \
})

/* Define GPIOs specific to BUTE */
#elif defined(CONFIG_MACH_ARGONLVREF)
#define EMU_GET_D_PLUS() \
({\
    ((gpio_get_data(0, 4)) ? \
        EMU_BUS_SIGNAL_STATE_ENABLED : EMU_BUS_SIGNAL_STATE_DISABLED); \
})

#define EMU_GET_D_MINUS() \
({\
    ((gpio_get_data(0, 6)) ? \
        EMU_BUS_SIGNAL_STATE_ENABLED : EMU_BUS_SIGNAL_STATE_DISABLED); \
})
 
#endif

/* Used for SDHC Driver */
/*!@cond INTERNAL */
#ifdef CONFIG_ARCH_MXC91231
#ifdef CONFIG_MACH_ASCENSION

#define ENABLE_TFLASH_IRQ()   (set_irq_type(ED_INT4, IRQT_BOTHEDGE))
#define DISABLE_TFLASH_IRQ()  (set_irq_type(ED_INT4, IRQT_NOEDGE))

#else /* Not Ascension */

#define ENABLE_TFLASH_IRQ() /* NULL */
#define DISABLE_TFLASH_IRQ() /* NULL */

#endif

#elif defined(CONFIG_MACH_ARGONLVREF)

#define ENABLE_TFLASH_IRQ() \
    do {                                                                               \
           /*                                                                          \
            * GPIO can't trigger on both edges, so when the interrupt is enabled       \
            * the proper edge must be set based on the current reading.                \
            */                                                                         \
            gpio_config(GPIO_MCU_PORT, GPIO_MCU_PIN_SD1_DET_B, false, ((sdhc_poll(0))  \
                        ? GPIO_INT_RISE_EDGE : GPIO_INT_FALL_EDGE));                   \
            enable_irq(MXC_GPIO_TO_IRQ(GPIO_PORT_SIG_TO_MXC_GPIO(GPIO_MCU_PORT,        \
                       GPIO_MCU_PIN_SD1_DET_B)));                                      \
        } while (0);

#define DISABLE_TFLASH_IRQ()                                                 \
    (disable_irq(MXC_GPIO_TO_IRQ(GPIO_PORT_SIG_TO_MXC_GPIO(GPIO_MCU_PORT,    \
                                 GPIO_MCU_PIN_SD1_DET_B))))


#elif defined(CONFIG_ARCH_MXC91131)

#define ENABLE_TFLASH_IRQ() \
  do {                                                                       \
     /*                                                                      \
      * GPIO can't trigger on both edges, so when the interrupt is enabled   \
      * the proper edge must be set based on the current reading.            \
      */                                                                     \
     gpio_config(1, USR_BLK_DEV_DEV1_INT_GPIO, false, ((GPIO_DEV1_ATTACHED)  \
                 ? GPIO_INT_RISE_EDGE : GPIO_INT_FALL_EDGE));                \
     enable_irq(MXC_GPIO_TO_IRQ(GPIO_PORT_SIG_TO_MXC_GPIO(1,                 \
                     USR_BLK_DEV_DEV1_INT_GPIO)));                           \
  } while (0);

#define DISABLE_TFLASH_IRQ()                                                 \
             (disable_irq(MXC_GPIO_TO_IRQ(GPIO_PORT_SIG_TO_MXC_GPIO(1,       \
                                            USR_BLK_DEV_DEV1_INT_GPIO))))

#endif
/*!@endcond */

/******************************************************************************
* function prototypes
******************************************************************************/
void power_ic_gpio_config_event_int(void);
int power_ic_gpio_event_read_priint(unsigned int signo);
void power_ic_gpio_emu_config(void);
void power_ic_gpio_lights_config(void);
void power_ic_gpio_lights_set_main_display(LIGHTS_FL_COLOR_T nColor);
void power_ic_gpio_lights_set_keypad_base(LIGHTS_FL_COLOR_T nColor);
void power_ic_gpio_lights_set_keypad_slider(LIGHTS_FL_COLOR_T nColor);
void power_ic_gpio_lights_set_camera_flash(LIGHTS_FL_COLOR_T nColor);
void power_ic_gpio_sdhc_mmc_config(void * call_back);
void power_ic_gpio_sdhc_gpio_config(SDHC_MODULE_T module, bool enable);
int power_ic_gpio_sdhc_poll(SDHC_MODULE_T module);

#endif /* __GPIO_H__ */
