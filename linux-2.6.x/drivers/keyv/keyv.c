/*
 * Copyright (C) 2006-2007 Motorola, Inc.
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
 * Motorola 2007-Jun-21 - Fix possible interrupt line not resetting
 * Motorola 2007-Jun-13 - Remove KEYV shutdown
 * Motorola 2007-Apr-26 - Fix possible reset when keys are enabled
 * Motorola 2007-Jan-04 - Release pressed key if key is requested to be disabled
 * Motorola 2006-Dec-07 - Remove original interrupt fix & fix current drain issue
 * Motorola 2006-Oct-10 - New Key Support
 * Motorola 2006-Sep-06 - Keycode changes
 * Motorola 2006-Aug-11 - MVL Upmerges
 * Motorola 2006-May-12 - Initial Creation
 */
 
 #include <linux/keyv.h>
 #include <linux/devfs_fs_kernel.h>
 #include <linux/delay.h>
 #include <linux/errno.h>
 #include <linux/fs.h>
 #include <linux/keypad.h>
 #include <linux/module.h>
 #include <linux/power_ic_kernel.h>
 #include <linux/sched.h>
 
 #include <asm/mot-gpio.h>
 #include <asm/uaccess.h>
 #include <asm/arch/gpio.h>
 
 #include "keyv_registers.h"
 
/******************************************************************************
* Constants
******************************************************************************/
#define KEYV_INT              12

#define KEYV_MODE_NONE      0xFF
#define KEYV_MODE0          0x80 
#define KEYV_MODE1          0x40
#define KEYV_MODE2          0x05
#define KEYV_MODE3          0x02

#define KEYV_RESET          0x20

#ifdef CONFIG_PROXSENSOR
#define KEYV_KEY_INIT       0x31
#else
#define KEYV_KEY_INIT       0X30
#endif /* PROXSENSOR */

#define KEYV_ALL_KEY        0x3E

#define KEYV_DONE           0x04
#define KEYV_RELEASE        0x40
#define KEYV_MASK_0         0x41
#define KEYV_MASK_1         0x42
#define KEYV_MASK_2         0x44
#define KEYV_MASK_3         0x48

static const struct
{
    unsigned int keycode;
    unsigned int enable;
    unsigned int disable;
} key_states[KEYV_KEY_MAX] =
{
    /* KEYV_KEY_0 */   { KEYPAD_KEYV_0, 0x02, 0xFD },        
    /* KEYV_KEY_1 */   { KEYPAD_KEYV_1, 0x04, 0xFB },
    /* KEYV_KEY_2 */   { KEYPAD_KEYV_2, 0x08, 0xF7 },
    /* KEYV_KEY_3 */   { KEYPAD_KEYV_3, 0x01, 0xFE },
};

/******************************************************************************
* Local Variables
******************************************************************************/
static unsigned int key_value = KEYV_KEY_INIT;
static unsigned int previous_key = KEYV_RELEASE;

/*! Flag used to indicated the read in the interrupt handler failed */
static bool interrupt_fail = false;

/*! Declare a wait queue used to signal arriving interrupts to the interrupt handler thread */
static DECLARE_WAIT_QUEUE_HEAD(interrupt_thread_wait);

/*! Flag to indicate pending interrupt conditions */ 
static int interrupt_flag = 0;

/******************************************************************************
* Global variables
******************************************************************************/

/******************************************************************************
* Local function prototypes
******************************************************************************/
static void initialize_keyv_reg(void);
static int keyv_mode1_set(void);
static int keyv_mode2_set(void);
static int keyv_mode3_set(void);
static void key_press(unsigned int key);
static void key_release(unsigned int key);
static int interrupt_thread_loop (void *unused);
static irqreturn_t keyv_irq_handler (int irq, void *dev_id, struct pt_regs *regs);
static int keyv_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
/******************************************************************************
* Local Functions
******************************************************************************/
/*!
 * @brief initializes the KeyV
 *
 * The KeyV part only has one register that needs to be written in order
 * to initialize all the keys to be interrupt driven.
 *
 */
static void initialize_keyv_reg(void)
{
    keyv_reg_write(KEYV_KEY_INIT);
}

/*!
 * @brief KeyV mode1 function
 *
 * This function sets the KeyV IC into mode1.  
 *
 * @return 0 if successful
 */
static int keyv_mode1_set(void)
{
#ifdef CONFIG_PROXSENSOR
    return(keyv_mode2_set());
#else
    return(keyv_reg_write(KEYV_MODE1));
#endif    
}

/*!
 * @brief KeyV mode2 function
 *
 * This function sets the KeyV IC into mode2.
 *
 * @return 0 if successful
 */
static int keyv_mode2_set(void)
{
    return(keyv_reg_write(KEYV_MODE2));
}

/*!
 * @brief KeyV mode3 function
 *
 * This function sets the KeyV IC into mode3.
 *
 * @return 0 if successful
 */
static int keyv_mode3_set(void)
{
    return(keyv_reg_write(KEYV_MODE3));
}

/*!
 * @brief Implements the press of any KeyV key
 *
 * This function will send a key press to the keypad driver for any KeyV key event.
 *
 * @param key KeyV key that is to be pressed
 *
 */
static void key_press(unsigned int key)
{
    /* check to see if the previous key was a press or release */
#ifdef CONFIG_PROXSENSOR
    if((previous_key == KEYV_MASK_0) || (previous_key == KEYV_MASK_1) ||
       (previous_key == KEYV_MASK_2) || (previous_key == KEYV_MASK_3))
#else
    if((previous_key == KEYV_MASK_1) || (previous_key == KEYV_MASK_2) ||
       (previous_key == KEYV_MASK_3))
#endif
    {
         /* Check to see if the previous key pressed was the same key that is currently pressed.
         * If so, do nothing */
        if(previous_key != key)
        {
            /* send keypad driver a release */
            key_release(previous_key);
        }
        else
        {
            return;
        }
    }
    
    /* send keypad driver the key press */
    switch(key)
    {
#ifdef CONFIG_PROXSENSOR
        case KEYV_MASK_0:
            generate_key_event(key_states[KEYV_KEY_3].keycode, KEYDOWN);
            break;
#endif
        case KEYV_MASK_1:
            generate_key_event(key_states[KEYV_KEY_0].keycode, KEYDOWN);
            break;
                
        case KEYV_MASK_2:
            generate_key_event(key_states[KEYV_KEY_1].keycode, KEYDOWN);
            break;
                    
        case KEYV_MASK_3:
            generate_key_event(key_states[KEYV_KEY_2].keycode, KEYDOWN);
            break;

        default:
            break;
    }
}

/*!
 * @brief Implements the release of a KeyV key
 *
 * This function will send a release key event to the keypad driver when any
 * KeyV key is released
 *
 * @param key KeyV key that is to be released
 *
 */
static void key_release(unsigned int key)
{
    switch(key)
    {
#ifdef CONFIG_PROXSENSOR
        case KEYV_MASK_0:
            generate_key_event(key_states[KEYV_KEY_3].keycode, KEYUP);
            break;
#endif        
        case KEYV_MASK_1:
            generate_key_event(key_states[KEYV_KEY_0].keycode, KEYUP);
            break;
                
        case KEYV_MASK_2:
            generate_key_event(key_states[KEYV_KEY_1].keycode, KEYUP);
            break;
                    
        case KEYV_MASK_3:
            generate_key_event(key_states[KEYV_KEY_2].keycode, KEYUP);
            break;

        default:
            break;
    }
}

/*!
 * @brief Implements the kernel thread for KeyV "bottom half" interrupt handling.
 *
 * The function that implements the kernel thread for interrupt handling.  This
 * is used to "simulate" a bottom half because the I2C driver only works in task
 * context, not in interrupt or bottom half context.  The function simply executes
 * the same bottom half function that would normally run in the bottom half tasklet.
 *
 * @param unused An unused parameter
 *
 * @return 0, but the function will only return upon receiving an abort signal.
 */
static int interrupt_thread_loop (void *unused)
{
    struct sched_param thread_sched_params = {.sched_priority = (MAX_RT_PRIO - 1)};
    unsigned int value;
    int error = 0;
    static unsigned int init_start = 0;
    bool read_me_twice;
        
    /* Usual thread setup. */
    lock_kernel();
    daemonize("kkeyvd");
    strcpy(current->comm, "kkeyvd");

    reparent_to_init();
    /* Ignore all signals, but those which terminate the thread. */
    siginitsetinv(&current->blocked, sigmask(SIGKILL)|sigmask(SIGINT)|sigmask(SIGTERM));
    unlock_kernel();
    /* Sets a realtime thread's priority */
    sched_setscheduler(current, SCHED_FIFO, &thread_sched_params);
         
    /*
     * Loop unless an abort signal is received.  All signals, but the abort signals are
     * masked off in the common setup.  As a result only abort signals can be pending.
     */
    while(!signal_pending(current))
    {
        value = 0;
        error = 0;
        read_me_twice = true;

        /* Sleep if an interrupt isn't pending again */
        if (wait_event_interruptible (interrupt_thread_wait, interrupt_flag) < 0)
        {
            break;
        }
                
        /* Reset the interrupt flag */
        interrupt_flag = 0;
        
        do
        {
            /* Handle the interrupt */
            error = keyv_reg_read(&value);
            
            if(error != 0)
            {
                interrupt_fail = true;
                
                /* If a key is being pressed, send a release */
                if(previous_key != KEYV_RELEASE)
                {
                    key_release(previous_key);
                    previous_key = KEYV_RELEASE;
                }
                
                break;
            }
            
            /* If the interrupt is for an initialization start of the KeyV part, then set
             * the variable to monitor */
            if((value & KEYV_MODE0) == KEYV_MODE0)
            {
                init_start = 1;
                
                /* If a key is being pressed, send a release */
                if(previous_key != KEYV_RELEASE)
                {
                    key_release(previous_key);
                    previous_key = KEYV_RELEASE;
                }
            }
            else
            {
                /* If the KeyV part was previously initializing, see if it is done */
                if(init_start)
                {
                    /* If initialization is complete, clear variable and be done */
                    if((value >> 4) == KEYV_DONE)
                    {
                        init_start = 0;
                        
                        /* Wait at least 100ms for reset and calibration to really finish */
                        msleep(100);
                        
                        /* Always re-write the keys that were enabled incase they got lost during init */
                        error = keyv_reg_write(key_value);
                            
                        /* If all the keys are disabled, then place the IC into mode1, otherwise do nothing */
                        if(key_value == KEYV_KEY_INIT)
                        {
                            error |= keyv_mode1_set();
                        }
                    }
                    /* If initialization was previously happening but the upper 4 bits do not match
                     * 0x4, then the 4 KeyV keys are not ready to be used.  At this point
                     * there would be a mechanical issue so the part has to be put into mode1. */
                    else
                    {
                        /* Wait at least 100ms for reset and calibration to really finish */
                        msleep(100);
                        error |= keyv_mode1_set();
                    }
                }
                /* If KeyV part was not previously initializing, start to check key presses */
                else
                {
                    /* KeyV 1 key is being pressed */
                    if(value == KEYV_MASK_1)
                    {
                        key_press(KEYV_MASK_1);
                    }
                    /* KeyV 2 key is being pressed */
                    else if(value == KEYV_MASK_2)
                    {
                        key_press(KEYV_MASK_2);
                    }
                    /* KeyV 3 key is being pressed */
                    else if(value == KEYV_MASK_3)
                    {
                        key_press(KEYV_MASK_3);
                    }
#ifdef CONFIG_PROXSENSOR
                    /* KeyV 0 key is being pressed */
                    else if(value == KEYV_MASK_0)
                    {
                        key_press(KEYV_MASK_0);
                    }
                    /* Is it a key release? */
                    else if(value == KEYV_RELEASE)
                    {
                        if(previous_key != KEYV_RELEASE)
                        {
                            /* send keypad driver a key release */
                            key_release(previous_key);
                        }
                    }
#else
                    /* Is it a key release? */
                    else if((value == KEYV_RELEASE) || (value == KEYV_MASK_0))
                    {
                        if((previous_key != KEYV_RELEASE) && (previous_key != KEYV_MASK_0))
                        {
                            /* send keypad driver a key release */
                            key_release(previous_key);
                        }
                    }
#endif
                    /* No valid key is recognized */
                    else
                    {
                        if(read_me_twice)
                        {
                            read_me_twice = false;
                        }
                        else
                        {
                            /* send keypad driver a key release */
                            key_release(previous_key);
                        
                            read_me_twice = true;
                            
                            /* reset the KeyV part */
                            error |= keyv_reset();
                        }
                    }
                    if(read_me_twice)
                    {
                        previous_key = value;
                    }
                }
            }
        } while(!read_me_twice);
    }

    return 0;
}

/*!
 * @brief Interrupt handler for KeyV interrupts
 *
 * This is the interrupt handler for the KeyV interrupts.
 * The purpose of the interrupt handler is to schedule the bottom half
 * interrupt handler.  Due to problems with the I2C driver failing
 * to operate in interrupt (or bottom half) context, the bottom half of
 * the interrupt is implemented as a kernel thread.  The interrupt handler
 * wakes up the thread (which is waiting on a wait queue).
 *
 * @note This function runs with context switching and all other interrupts
 * disabled, so it needs to run as fast as possible.
 *
 * @param        irq        the irq number
 * @param        dev_id     the pointer on the device
 * @param        regs       the interrupt parameters
 *
 * @return       The function returns IRQ_RETVAL(1) when handled.
 */
static irqreturn_t keyv_irq_handler (int irq, void *dev_id, struct pt_regs *regs)
{
    /* Set the interrupt flag to prevent the thread from sleeping */
    interrupt_flag = 1;

    /* Wake up the "bottom half" interrupt handler thread */
    wake_up(&interrupt_thread_wait);

    return IRQ_RETVAL(1);
}

/*!
 * @brief the ioctl() handler for the KeyV device node
 *
 * This function implements the ioctl() system call for the KeyV device node.
 * The command is handled directly by calling the appropriate function
 *
 * @param        inode       inode pointer
 * @param        file        file pointer
 * @param        cmd         the ioctl() command
 * @param        arg         the ioctl() argument
 *
 * @return 0 if successful
 */
static int keyv_ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
{
    int error;
    int temp;
       
    KEYV_KEY_SET_T keyv_key_update;
     
    /* Handle the request. */
    switch(cmd)
    {
        case KEYV_IOCTL_RESET:
            return(keyv_reset());
            break;

        case KEYV_IOCTL_KEY_EN:
            if(copy_from_user((void*)&keyv_key_update,(void *)arg, sizeof(keyv_key_update)) != 0)
            {
                return -EFAULT;
            }
             
            /* Enable/Disable key */
            if((error = keyv_key_en(1, keyv_key_update.key, keyv_key_update.en)))
            {
                return error;
            }
            break;
            
        case KEYV_IOCTL_READ_REG:
            /* Read the entire register and pass the data back the the caller in user space. */
            if ((error = keyv_reg_read(&temp)) != 0)
            {
                return error;
            }
 
            /* Return the read value back to the caller. */
            if(put_user(temp, (int *)arg))
            {
                return -EFAULT;
            }
            break;
            
        case KEYV_IOCTL_WRITE_REG:
            if ((error = keyv_reg_write((unsigned int)arg)) != 0)
            {
                return error;
            }
            break;
            
        case KEYV_IOCTL_PROX_SENSOR_EN:
            if((error = keyv_prox_sensor_en((unsigned int)arg)) != 0)
            {
                return error;
            }
            break;
            
        default: /* This shouldn't be able to happen, but just in case... */
            return -ENOTTY;
            break;
    
    }
    return 0;
}

/*! This structure defines the file operations for the KeyV device */
static struct file_operations keyv_fops =
{
    .owner =    THIS_MODULE,
    .ioctl =    keyv_ioctl,
};

/******************************************************************************
* Global Functions
******************************************************************************/
/*!
 * @brief KeyV IC initialization function
 *
 * This function implements the initialization function of the KeyV
 * driver.  It currently initializes the GPIO signals that will be used to control
 * the KeyV after it is registered with the I2C.
 *
 * @return 0 if successful
 */
int __init keyv_init(void)
{
    int ret;
    
    /* Create a device */
    ret = register_chrdev(KEYV_MAJOR_NUM, KEYV_DEV_NAME, &keyv_fops);
    if (ret < 0)
    {
        return ret;
    }

    devfs_mk_cdev(MKDEV(KEYV_MAJOR_NUM,0), S_IFCHR | S_IRUGO | S_IWUSR, KEYV_DEV_NAME);
    
    /* Initialize the KeyV driver */
    keyv_initialize(initialize_keyv_reg);
    
    /* Start our kernel thread */
    kernel_thread(interrupt_thread_loop, NULL, 0);
    
    /* Configure the KeyV interrupt */
    iomux_config_mux(SP_UH2_RXDP, OUTPUTCONFIG_DEFAULT, INPUTCONFIG_NONE);
    gpio_config(GPIO_SP_A_PORT, KEYV_INT, false, GPIO_INT_FALL_EDGE);
    gpio_request_irq(GPIO_SP_A_PORT, KEYV_INT, GPIO_HIGH_PRIO, keyv_irq_handler, 
                     SA_INTERRUPT, "KeyV irq: SCM-A11", 0);
    
    /* Reset the touchpad controller thru I2C */
    return(keyv_reg_write(KEYV_RESET));
}

/*!
 * @brief KeyV cleanup function
 *
 * This function is called when the KeyV driver is destroyed
 */
void __exit keyv_exit(void)
{
    unregister_chrdev(KEYV_MAJOR_NUM, KEYV_DEV_NAME);
}

/*!
 * @brief KeyV IC reset function
 *
 * This function resets the KeyV IC.
 *
 * @return 0 if successful
 */
int keyv_reset(void)
{
    return(keyv_reg_write(KEYV_RESET));
}

/*!
 * @brief KeyV Key enable/disable function
 *
 * This function enables and disables the keys for KeyV
 *
 * @param  nKeys The number of key data pairs which follow.
 * @param  ...   Parameter pairs which include key followed by it being enabled/disabled.
 * 
 * @return 0 if successful
 */
int keyv_key_en
(
    unsigned char nKeys,
    ...
    /* KEYV_KEY_T key,  Optional Input - Key to enable/disable */
    /* bool en,              Optional Input - Enabled/Disabled */
    /* Repeat the two inputs above as needed. */
)
{
    va_list pArgs;
    KEYV_KEY_T key;
    bool en;
    int error = 0;
    int i = 0;
    unsigned int prev_key_state = key_value;
    unsigned int value;
    

    /* If an interrupt failed, read from the IC to clear the interrupt */
    if(interrupt_fail)
    {
        error = keyv_reg_read(&value);
        
        if(error == 0)
        {
            interrupt_fail = false;
        } 
    }
    
    va_start(pArgs, nKeys);
    
    for(i = 0; i < nKeys; i++)
    {
        /*
         * Store the current value in the queue.  Please note, when variable parameter lists are
         * used argument promotion occurs.  If the size of the type is less than an int it will
         * be promoted to an int.  For this reason some special handling must be done to guarantee
         * the parameters are handled correctly.
         */
        key = (KEYV_KEY_T)va_arg(pArgs, unsigned int);
        en = (bool)va_arg(pArgs, unsigned int);
        
        if(key >= KEYV_KEY_MAX)
        {
            printk("KEYV:  Invalid key requested\n");
            return -EINVAL;
        }
    
        /* If someone is messing around with the third key, which could be index 0
         * then ignore what they are doing and just warn them */
        if(key == KEYV_KEY_3)
        {
            return -EINVAL;
        }
        else
        {
            /* Turn on or disable the specified keys.  Keep track also of the last keys that were
             * enabled/disabled so that a key is not accidentally changed.
             */
            if(en == 0)
            {
                /* If the key that is trying to be disabled was previously pressed, disable the key */
                if(previous_key == (KEYV_RELEASE | key_states[key].enable))
                {
                    generate_key_event(key_states[key].keycode, KEYUP);
                    previous_key = KEYV_RELEASE;
                }
                
                key_value &= key_states[key].disable;
            }
            else
            {
                key_value |= key_states[key].enable;
            }
        }
    }
    
    va_end(pArgs);
    
    if(en > 0)
    {
        /* Enable all the keys */
        error |= keyv_reg_write(KEYV_ALL_KEY);
        error |= keyv_mode3_set();
    }
    else
    {
        /* Disable all keys */
        error |= keyv_reg_write(KEYV_KEY_INIT);
    }
    
    /* If all the keys are disabled, then place the IC in the appropriate mode */
    if(key_value == KEYV_KEY_INIT)
    {
        error |= keyv_mode1_set();
    }
    
    return(error);
}

/*!
 * @brief Proximity Sensor enable/disable function
 *
 * This function enables and disables the Proximity Sensor
 *
 * @param  en 1 for enabling, 0 for disabling
 * 
 * @return 0 if successful
 */
int keyv_prox_sensor_en(unsigned int en)
{
    int error = 0;
    
    if(en > 0)
    {
        error |= keyv_reg_write(key_value |= key_states[KEYV_KEY_3].enable);
    }
    else
    {
        error |= keyv_reg_write(key_value &= key_states[KEYV_KEY_3].disable);
    }
    
    return(error);
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS
EXPORT_SYMBOL(keyv_reset);
EXPORT_SYMBOL(keyv_key_en);
EXPORT_SYMBOL(keyv_prox_sensor_en);

/*
 * Module entry points
 */
module_init(keyv_init);
module_exit(keyv_exit);

MODULE_DESCRIPTION("KeyV device driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
#endif
