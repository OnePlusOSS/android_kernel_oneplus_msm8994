/*************************************************************
 ** Copyright (C), 2012-2016, OEM Mobile Comm Corp., Ltd 
 ** VENDOR_EDIT
 ** File        : synaptics_dsx_i2c.c
 ** Description : 
 ** Date        : 2014-10-27 11:37
 ** Author      : BSP.Sensor
 ** 
 ** ------------------ Revision History: ---------------------
 **      <author>        <date>          <desc>
 *************************************************************/
/*
 * Synaptics DSX touchscreen driver
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
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
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <linux/fb.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/project_info.h>
//#include <mach/device_info.h>
//#include <linux/pcb_version.h>

#include "synaptics_dsx.h"
#include "synaptics_dsx_i2c.h"

#include "synaptics_test_rawdata.h"
#include "../../../fs/proc/internal.h"
#ifdef KERNEL_ABOVE_2_6_38
#include <linux/input/mt.h>
#endif
#include <linux/smp.h>

#define DRIVER_NAME "synaptics-rmi-ts"
#define INPUT_PHYS_NAME "synaptics-rmi-ts/input0"

#ifdef KERNEL_ABOVE_2_6_38
#define TYPE_B_PROTOCOL
#endif
#define REPORT_2D_Z
#define NO_0D_WHILE_2D

#define REPORT_2D_W

//mingqiang.guo add for check  wrong TP befor make ,only for test work station 
//#define CHECK_WRONG_TP_BEFORCE_MAKE
#define RPT_TYPE (1 << 0)
#define RPT_X_LSB (1 << 1)
#define RPT_X_MSB (1 << 2)
#define RPT_Y_LSB (1 << 3)
#define RPT_Y_MSB (1 << 4)
#define RPT_Z (1 << 5)
#define RPT_WX (1 << 6)
#define RPT_WY (1 << 7)
#define RPT_DEFAULT (RPT_TYPE | RPT_X_LSB | RPT_X_MSB | RPT_Y_LSB | RPT_Y_MSB)

#define EXP_FN_WORK_DELAY_MS 10 /* ms */
#define SYN_I2C_RETRY_TIMES 1
#define MAX_F11_TOUCH_WIDTH 15

#define CHECK_STATUS_TIMEOUT_MS 100

#define F01_STD_QUERY_LEN 21
#define F01_BUID_ID_OFFSET 18
#define F11_STD_QUERY_LEN 9
#define F11_STD_CTRL_LEN 10
#define F11_STD_DATA_LEN 12

#define STATUS_NO_ERROR 0x00
#define STATUS_RESET_OCCURRED 0x01
#define STATUS_INVALID_CONFIG 0x02
#define STATUS_DEVICE_FAILURE 0x03
#define STATUS_CONFIG_CRC_FAILURE 0x04
#define STATUS_FIRMWARE_CRC_FAILURE 0x05
#define STATUS_CRC_IN_PROGRESS 0x06

#define NORMAL_OPERATION (0 << 0)
#define SENSOR_SLEEP (1 << 0)
#define NO_SLEEP_OFF (0 << 2)
#define NO_SLEEP_ON (1 << 2)
#define CONFIGURED (1 << 7)

#include <linux/wakelock.h>
static struct wake_lock tp_wake_lock;//有双击唤醒中断时，延时2S，放弃suspend流程，等待resume流程

static void speedup_synaptics_resume(struct work_struct *work); //mingqiang.guo add for LCD show later when push power button  and  two click  in gesture  
static DEFINE_SEMAPHORE(work_sem);  
static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
        unsigned short addr, unsigned char *data,
        unsigned short length);

static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
        unsigned short addr, unsigned char *data,
        unsigned short length);

static int synaptics_rmi4_f12_set_enables(struct synaptics_rmi4_data *rmi4_data,
        unsigned short ctrl28);

static int synaptics_rmi4_free_fingers(struct synaptics_rmi4_data *rmi4_data);
static int synaptics_rmi4_reinit_device(struct synaptics_rmi4_data *rmi4_data);
static int synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data, unsigned short f01_cmd_base_addr);
static inline void wait_test_cmd_finished(void);

#ifdef CONFIG_HAS_EARLYSUSPEND
static ssize_t synaptics_rmi4_full_pm_cycle_show(struct device *dev,
        struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_full_pm_cycle_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);

static void synaptics_rmi4_early_suspend(struct early_suspend *h);

static void synaptics_rmi4_late_resume(struct early_suspend *h);
#endif

static int synaptics_rmi4_suspend(struct device *dev);

static int synaptics_rmi4_resume(struct device *dev);

static ssize_t synaptics_rmi4_f01_reset_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_f01_productinfo_show(struct device *dev,
        struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_f01_buildid_show(struct device *dev,
        struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_f01_flashprog_show(struct device *dev,
        struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_0dbutton_show(struct device *dev,
        struct device_attribute *attr, char *buf);

static ssize_t synaptics_rmi4_0dbutton_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_suspend_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_open_or_close_holster_mode_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);

static ssize_t synaptics_rmi4_open_or_close_holster_mode_show(struct device *dev,
        struct device_attribute *attr, char *buf);

struct synaptics_rmi4_f01_device_status {
    union {
        struct {
            unsigned char status_code:4;
            unsigned char reserved:2;
            unsigned char flash_prog:1;
            unsigned char unconfigured:1;
        } __packed;
        unsigned char data[1];
    };
};

struct synaptics_rmi4_f12_query_5 {
    union {
        struct {
            unsigned char size_of_query6;
            struct {
                unsigned char ctrl0_is_present:1;
                unsigned char ctrl1_is_present:1;
                unsigned char ctrl2_is_present:1;
                unsigned char ctrl3_is_present:1;
                unsigned char ctrl4_is_present:1;
                unsigned char ctrl5_is_present:1;
                unsigned char ctrl6_is_present:1;
                unsigned char ctrl7_is_present:1;
            } __packed;
            struct {
                unsigned char ctrl8_is_present:1;
                unsigned char ctrl9_is_present:1;
                unsigned char ctrl10_is_present:1;
                unsigned char ctrl11_is_present:1;
                unsigned char ctrl12_is_present:1;
                unsigned char ctrl13_is_present:1;
                unsigned char ctrl14_is_present:1;
                unsigned char ctrl15_is_present:1;
            } __packed;
            struct {
                unsigned char ctrl16_is_present:1;
                unsigned char ctrl17_is_present:1;
                unsigned char ctrl18_is_present:1;
                unsigned char ctrl19_is_present:1;
                unsigned char ctrl20_is_present:1;
                unsigned char ctrl21_is_present:1;
                unsigned char ctrl22_is_present:1;
                unsigned char ctrl23_is_present:1;
            } __packed;
            struct {
                unsigned char ctrl24_is_present:1;
                unsigned char ctrl25_is_present:1;
                unsigned char ctrl26_is_present:1;
                unsigned char ctrl27_is_present:1;
                unsigned char ctrl28_is_present:1;
                unsigned char ctrl29_is_present:1;
                unsigned char ctrl30_is_present:1;
                unsigned char ctrl31_is_present:1;
            } __packed;
        };
        unsigned char data[5];
    };
};

struct synaptics_rmi4_f12_query_8 {
    union {
        struct {
            unsigned char size_of_query9;
            struct {
                unsigned char data0_is_present:1;
                unsigned char data1_is_present:1;
                unsigned char data2_is_present:1;
                unsigned char data3_is_present:1;
                unsigned char data4_is_present:1;
                unsigned char data5_is_present:1;
                unsigned char data6_is_present:1;
                unsigned char data7_is_present:1;
            } __packed;
            struct {
                unsigned char data8_is_present:1;
                unsigned char data9_is_present:1;
                unsigned char data10_is_present:1;
                unsigned char data11_is_present:1;
                unsigned char data12_is_present:1;
                unsigned char data13_is_present:1;
                unsigned char data14_is_present:1;
                unsigned char data15_is_present:1;
            } __packed;
        };
        unsigned char data[3];
    };
};

struct synaptics_rmi4_f12_ctrl_8 {
    union {
        struct {
            unsigned char max_x_coord_lsb;
            unsigned char max_x_coord_msb;
            unsigned char max_y_coord_lsb;
            unsigned char max_y_coord_msb;
            unsigned char rx_pitch_lsb;
            unsigned char rx_pitch_msb;
            unsigned char tx_pitch_lsb;
            unsigned char tx_pitch_msb;
            unsigned char low_rx_clip;
            unsigned char high_rx_clip;
            unsigned char low_tx_clip;
            unsigned char high_tx_clip;
            unsigned char num_of_rx;
            unsigned char num_of_tx;
        };
        unsigned char data[14];
    };
};

struct synaptics_rmi4_f12_ctrl_23 {
    union {
        struct {
            unsigned char obj_type_enable;
            unsigned char max_reported_objects;
        };
        unsigned char data[2];
    };
};

struct synaptics_rmi4_f12_finger_data {
    unsigned char object_type_and_status;
    unsigned char x_lsb;
    unsigned char x_msb;
    unsigned char y_lsb;
    unsigned char y_msb;
#ifdef REPORT_2D_Z
		unsigned char z;
#endif
#ifdef REPORT_2D_W
    unsigned char wx;
    unsigned char wy;
#endif
};

struct synaptics_rmi4_f1a_query {
    union {
        struct {
            unsigned char max_button_count:3;
            unsigned char reserved:5;
            unsigned char has_general_control:1;
            unsigned char has_interrupt_enable:1;
            unsigned char has_multibutton_select:1;
            unsigned char has_tx_rx_map:1;
            unsigned char has_perbutton_threshold:1;
            unsigned char has_release_threshold:1;
            unsigned char has_strongestbtn_hysteresis:1;
            unsigned char has_filter_strength:1;
        } __packed;
        unsigned char data[2];
    };
};

struct synaptics_rmi4_f1a_control_0 {
    union {
        struct {
            unsigned char multibutton_report:2;
            unsigned char filter_mode:2;
            unsigned char reserved:4;
        } __packed;
        unsigned char data[1];
    };
};

struct synaptics_rmi4_f1a_control {
    struct synaptics_rmi4_f1a_control_0 general_control;
    unsigned char button_int_enable;
    unsigned char multi_button;
    unsigned char *txrx_map;
    unsigned char *button_threshold;
    unsigned char button_release_threshold;
    unsigned char strongest_button_hysteresis;
    unsigned char filter_strength;
};

struct synaptics_rmi4_f1a_handle {
    int button_bitmask_size;
    unsigned char max_count;
    unsigned char valid_button_count;
    unsigned char *button_data_buffer;
    unsigned char *button_map;
    struct synaptics_rmi4_f1a_query button_query;
    struct synaptics_rmi4_f1a_control button_control;
};

struct synaptics_rmi4_exp_fn {
    enum exp_fn fn_type;
    bool inserted;
    int (*func_init)(struct synaptics_rmi4_data *rmi4_data);
    void (*func_remove)(struct synaptics_rmi4_data *rmi4_data);
    void (*func_attn)(struct synaptics_rmi4_data *rmi4_data,
            unsigned char intr_mask);
    struct list_head link;
};

struct synaptics_rmi4_exp_fn_data {
    bool initialized;
    bool queue_work;
    struct mutex mutex;
    struct list_head list;
    struct delayed_work work;
    struct workqueue_struct *workqueue;
    struct synaptics_rmi4_data *rmi4_data;
};

static struct synaptics_rmi4_exp_fn_data exp_data;
static void synaptics_set_int_mask(struct synaptics_rmi4_data *ts, int enable);
static ssize_t synaptics_rmi4_gesture_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);
static ssize_t synaptics_rmi4_gesture_show(struct device *dev,
        struct device_attribute *attr, char *buf);
static ssize_t synaptics_rmi4_delta_data_show(struct device *dev,
        struct device_attribute *attr, char *buf);
static ssize_t synaptics_rmi4_baseline_show(struct device *dev,
        struct device_attribute *attr, char *buf);
static ssize_t synaptics_rmi4_baseline_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);
static ssize_t synaptics_rmi4_read_baseline_data_open_cbc(struct device *dev,
        struct device_attribute *attr, char *buf);
static ssize_t synaptics_rmi4_read_baseline_data_close_cbc(struct device *dev,
        struct device_attribute *attr, char *buf);
static ssize_t synaptics_rmi4_vendor_show(struct device *dev,
        struct device_attribute *attr, char *buf);
static ssize_t synaptics_attr_loglevel_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);
static ssize_t synaptics_attr_loglevel_show(struct device *dev,
        struct device_attribute *attr, char *buf);
static ssize_t sense_frequency_selection_reg_show(struct device *dev,
        struct device_attribute *attr, char *buf);
static ssize_t sense_frequency_selection_reg_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count);

static struct device_attribute attrs[] = {
#ifdef CONFIG_HAS_EARLYSUSPEND
    __ATTR(full_pm_cycle, (S_IRUGO | S_IWUSR),
            synaptics_rmi4_full_pm_cycle_show,
            synaptics_rmi4_full_pm_cycle_store),
#endif
    __ATTR(reset, S_IWUSR,
            synaptics_rmi4_show_error,
            synaptics_rmi4_f01_reset_store),
    __ATTR(productinfo, S_IRUGO,
            synaptics_rmi4_f01_productinfo_show,
            synaptics_rmi4_store_error),
    __ATTR(buildid, S_IRUGO,
            synaptics_rmi4_f01_buildid_show,
            synaptics_rmi4_store_error),
    __ATTR(flashprog, S_IRUGO,
            synaptics_rmi4_f01_flashprog_show,
            synaptics_rmi4_store_error),
    __ATTR(0dbutton, (S_IRUGO | S_IWUSR),
            synaptics_rmi4_0dbutton_show,
            synaptics_rmi4_0dbutton_store),
    __ATTR(suspend, S_IWUSR,
            synaptics_rmi4_show_error,
            synaptics_rmi4_suspend_store),
    __ATTR(gesture, (S_IRUGO | S_IWUSR),
            synaptics_rmi4_gesture_show,
            synaptics_rmi4_gesture_store),
    __ATTR(delta_data, S_IRUGO,
            synaptics_rmi4_delta_data_show,
            synaptics_rmi4_store_error),			
    __ATTR(baseline_test, (S_IRUGO | S_IWUSR),
            synaptics_rmi4_baseline_show,
            synaptics_rmi4_baseline_store),
    __ATTR(read_baseline_data_open_cbc, S_IRUGO,
            synaptics_rmi4_read_baseline_data_open_cbc,
            synaptics_rmi4_store_error),			
    __ATTR(read_baseline_data_close_cbc, S_IRUGO,
            synaptics_rmi4_read_baseline_data_close_cbc,
            synaptics_rmi4_store_error),					
    __ATTR(vendor_id, (S_IRUGO),
            synaptics_rmi4_vendor_show,
            synaptics_rmi4_store_error),
    __ATTR(log_level, (S_IRUGO | S_IWUSR),
            synaptics_attr_loglevel_show,
            synaptics_attr_loglevel_store),
    __ATTR(holstere_open_or_close, (S_IRUGO | S_IWUSR),
            synaptics_rmi4_open_or_close_holster_mode_show,
            synaptics_rmi4_open_or_close_holster_mode_store),
    __ATTR(sense_frequency_selection_reg, (S_IRUGO | S_IWUSR),
            sense_frequency_selection_reg_show,
            sense_frequency_selection_reg_store),
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static ssize_t synaptics_rmi4_full_pm_cycle_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%u\n",
            rmi4_data->full_pm_cycle);
}

static ssize_t synaptics_rmi4_full_pm_cycle_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int input;
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

    if (sscanf(buf, "%u", &input) != 1)
        return -EINVAL;

    rmi4_data->full_pm_cycle = input > 0 ? 1 : 0;

    return count;
}
#endif

static ssize_t synaptics_rmi4_f01_reset_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    int retval;
    unsigned int reset;
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

    if (sscanf(buf, "%u", &reset) != 1)
        return -EINVAL;

    if (reset != 1)
        return -EINVAL;

    printk(KERN_ERR "[syna]: reset device[%d]\n",rmi4_data->reset_count) ;
    rmi4_data->reset_count = 0 ;
    retval = synaptics_rmi4_reset_device(rmi4_data, rmi4_data->f01_cmd_base_addr);
    if (retval < 0) {
        dev_err(dev, "%s: Failed to issue reset command, error = %d\n", __func__, retval);
        return retval;
    }

    return count;
}

static ssize_t synaptics_rmi4_f01_productinfo_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "0x%02x 0x%02x\n",
            (rmi4_data->rmi4_mod_info.product_info[0]),
            (rmi4_data->rmi4_mod_info.product_info[1]));
}

static ssize_t synaptics_rmi4_f01_buildid_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%u\n",
            rmi4_data->firmware_id);
}

static ssize_t synaptics_rmi4_f01_flashprog_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int retval;
    struct synaptics_rmi4_f01_device_status device_status;
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f01_data_base_addr,
            device_status.data,
            sizeof(device_status.data));
    if (retval < 0) {
        dev_err(dev,
                "%s: Failed to read device status, error = %d\n",
                __func__, retval);
        return retval;
    }

    return snprintf(buf, PAGE_SIZE, "%u\n",
            device_status.flash_prog);
}

static ssize_t synaptics_rmi4_0dbutton_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%u\n",
            rmi4_data->button_0d_enabled);
}

static ssize_t synaptics_rmi4_0dbutton_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    int retval;
    unsigned int input;
    unsigned char ii;
    unsigned char intr_enable;
    struct synaptics_rmi4_fn *fhandler;
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
    struct synaptics_rmi4_device_info *rmi;

    rmi = &(rmi4_data->rmi4_mod_info);

    if (sscanf(buf, "%u", &input) != 1)
        return -EINVAL;

    input = input > 0 ? 1 : 0;

    if (rmi4_data->button_0d_enabled == input)
        return count;

    if (list_empty(&rmi->support_fn_list))
        return -ENODEV;

    list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
        if(fhandler == NULL)
        {
            printk("[TP] %s:%d  fhandler is NULL, continue loop\n",__func__,__LINE__);
            continue;
        }	        
        if (fhandler->fn_number == SYNAPTICS_RMI4_F1A) {
            ii = fhandler->intr_reg_num;

            retval = synaptics_rmi4_i2c_read(rmi4_data,
                    rmi4_data->f01_ctrl_base_addr + 1 + ii,
                    &intr_enable,
                    sizeof(intr_enable));
            if (retval < 0)
                return retval;

            if (input == 1)
                intr_enable |= fhandler->intr_mask;
            else
                intr_enable &= ~fhandler->intr_mask;

            retval = synaptics_rmi4_i2c_write(rmi4_data,
                    rmi4_data->f01_ctrl_base_addr + 1 + ii,
                    &intr_enable,
                    sizeof(intr_enable));
            if (retval < 0)
                return retval;
        }
    }

    rmi4_data->button_0d_enabled = input;

    return count;
}

static ssize_t synaptics_rmi4_suspend_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int input;

    if (sscanf(buf, "%u", &input) != 1)
        return -EINVAL;

    if (input == 1)
        synaptics_rmi4_suspend(dev);
    else if (input == 0)
        synaptics_rmi4_resume(dev);
    else
        return -EINVAL;

    return count;
}

static ssize_t synaptics_rmi4_open_or_close_holster_mode_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int input;
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
    uint8_t  databuf;
    if (sscanf(buf, "%u", &input) != 1)
        return -EINVAL;

    if (input == 1)
    {
        rmi4_data->holstere_mode_open_or_close= 1;//open holstere mode 
        synaptics_rmi4_i2c_write(rmi4_data,rmi4_data->holstere_mode_control_addr,(unsigned char*)&(rmi4_data->holstere_mode_open_or_close),1);
    }
    else if (input == 0)
    {
        rmi4_data->holstere_mode_open_or_close = 0;//close holstere mode 
        synaptics_rmi4_i2c_write(rmi4_data,rmi4_data->holstere_mode_control_addr,(unsigned char*)&(rmi4_data->holstere_mode_open_or_close),1);
    }
    else
        return -EINVAL;

    synaptics_rmi4_i2c_read(rmi4_data, rmi4_data->holstere_mode_control_addr, &databuf,1) ;
    printk("%s holstere mode %s\n",input ? "open" : "close",
            (databuf == (rmi4_data->holstere_mode_open_or_close) ) ? "success" : "fail");

    return count;
}

static ssize_t synaptics_rmi4_open_or_close_holster_mode_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
    uint8_t  databuf;

    synaptics_rmi4_i2c_read(rmi4_data, rmi4_data->holstere_mode_control_addr, &databuf,1) ;
    return sprintf(buf, " holstere mode is %s \n",databuf ? "open" : "close" ); 
}
/**
 * synaptics_rmi4_set_page()
 *
 * Called by synaptics_rmi4_i2c_read() and synaptics_rmi4_i2c_write().
 *
 * This function writes to the page select register to switch to the
 * assigned page.
 */
static int synaptics_rmi4_set_page(struct synaptics_rmi4_data *rmi4_data,
        unsigned int address)
{
    int retval = 0;
    unsigned char retry;
    unsigned char buf[PAGE_SELECT_LEN];
    unsigned char page;
    struct i2c_client *i2c = rmi4_data->i2c_client;

    page = ((address >> 8) & MASK_8BIT);
    if (page != rmi4_data->current_page) {
        buf[0] = MASK_8BIT;
        buf[1] = page;
        for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
            retval = i2c_master_send(i2c, buf, PAGE_SELECT_LEN);
            if (retval != PAGE_SELECT_LEN) {
                msleep(20);
            } else {
                rmi4_data->current_page = page;
                break;
            }
        }
        if (retry == SYN_I2C_RETRY_TIMES) {
			//dump_stack();
            dev_err(&rmi4_data->i2c_client->dev, "%s: I2C read over retry limit\n", __func__);
            synaptics_rmi4_reset_device(rmi4_data, rmi4_data->f01_cmd_base_addr) ;
            retval = -EIO;
        } else {
            rmi4_data->reset_count = 0 ;
        }

    } else {
        retval = PAGE_SELECT_LEN;
    }

    return retval;
}

/**
 * synaptics_rmi4_i2c_read()
 *
 * Called by various functions in this driver, and also exported to
 * other expansion Function modules such as rmi_dev.
 *
 * This function reads data of an arbitrary length from the sensor,
 * starting from an assigned register address of the sensor, via I2C
 * with a retry mechanism.
 */
static int synaptics_rmi4_i2c_read(struct synaptics_rmi4_data *rmi4_data,
        unsigned short addr, unsigned char *data, unsigned short length)
{
    int retval;
    unsigned char retry;
    unsigned char buf;
    struct i2c_msg msg[] = {
        {
            .addr = rmi4_data->i2c_client->addr,
            .flags = 0,
            .len = 1,
            .buf = &buf,
        },
        {
            .addr = rmi4_data->i2c_client->addr,
            .flags = I2C_M_RD,
            .len = length,
            .buf = data,
        },
    };

    buf = addr & MASK_8BIT;

    mutex_lock(&(rmi4_data->rmi4_io_ctrl_mutex));

    retval = synaptics_rmi4_set_page(rmi4_data, addr);
    if (retval != PAGE_SELECT_LEN)
        goto exit;

    for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
        if (i2c_transfer(rmi4_data->i2c_client->adapter, msg, 2) == 2) {
            retval = length;
            break;
        }

        msleep(20);
    }

    if (retry == SYN_I2C_RETRY_TIMES) {
		//dump_stack();
        dev_err(&rmi4_data->i2c_client->dev, "%s: I2C read over retry limit\n", __func__);
        rmi4_data->current_page = MASK_8BIT;
        retval = -EIO;
    }

exit:
    mutex_unlock(&(rmi4_data->rmi4_io_ctrl_mutex));

    return retval;
}

/**
 * synaptics_rmi4_i2c_write()
 *
 * Called by various functions in this driver, and also exported to
 * other expansion Function modules such as rmi_dev.
 *
 * This function writes data of an arbitrary length to the sensor,
 * starting from an assigned register address of the sensor, via I2C with
 * a retry mechanism.
 */
static int synaptics_rmi4_i2c_write(struct synaptics_rmi4_data *rmi4_data,
        unsigned short addr, unsigned char *data, unsigned short length)
{
    int retval;
    unsigned char retry;
    unsigned char buf[length + 1];
    struct i2c_msg msg[] = {
        {
            .addr = rmi4_data->i2c_client->addr,
            .flags = 0,
            .len = length + 1,
            .buf = buf,
        }
    };

    mutex_lock(&(rmi4_data->rmi4_io_ctrl_mutex));

    retval = synaptics_rmi4_set_page(rmi4_data, addr);
    if (retval != PAGE_SELECT_LEN)
        goto exit;

    buf[0] = addr & MASK_8BIT;
    memcpy(&buf[1], &data[0], length);

    for (retry = 0; retry < SYN_I2C_RETRY_TIMES; retry++) {
        if (i2c_transfer(rmi4_data->i2c_client->adapter, msg, 1) == 1) {
            retval = length;
            break;
        }

        msleep(20);
    }

    if (retry == SYN_I2C_RETRY_TIMES) {
		//dump_stack();
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: I2C write over retry limit\n",
                __func__);
        rmi4_data->current_page = MASK_8BIT;
        retval = -EIO;
    }

exit:
    mutex_unlock(&(rmi4_data->rmi4_io_ctrl_mutex));

    return retval;
}

/**
 * synaptics_rmi4_f11_abs_report()
 *
 * Called by synaptics_rmi4_report_touch() when valid Function $11
 * finger data has been detected.
 *
 * This function reads the Function $11 data registers, determines the
 * status of each finger supported by the Function, processes any
 * necessary coordinate manipulation, reports the finger data to
 * the input subsystem, and returns the number of fingers detected.
 */
static int synaptics_rmi4_f11_abs_report(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler)
{
    int retval;
    unsigned char touch_count = 0; /* number of touch points */
    unsigned char reg_index;
    unsigned char finger;
    unsigned char fingers_supported;
    unsigned char num_of_finger_status_regs;
    unsigned char finger_shift;
    unsigned char finger_status;
    unsigned char data_reg_blk_size;
    unsigned char finger_status_reg[3];
    unsigned char data[F11_STD_DATA_LEN];
    unsigned short data_addr;
    unsigned short data_offset;
    int x;
    int y;
    int wx;
    int wy;
    int temp;

    /*
     * The number of finger status registers is determined by the
     * maximum number of fingers supported - 2 bits per finger. So
     * the number of finger status registers to read is:
     * register_count = ceil(max_num_of_fingers / 4)
     */
    fingers_supported = fhandler->num_of_data_points;
    num_of_finger_status_regs = (fingers_supported + 3) / 4;
    data_addr = fhandler->full_addr.data_base;
    data_reg_blk_size = fhandler->size_of_data_register_block;

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            data_addr,
            finger_status_reg,
            num_of_finger_status_regs);
    if (retval < 0)
        return 0;

    for (finger = 0; finger < fingers_supported; finger++) {
        reg_index = finger / 4;
        finger_shift = (finger % 4) * 2;
        finger_status = (finger_status_reg[reg_index] >> finger_shift)
            & MASK_2BIT;

        /*
         * Each 2-bit finger status field represents the following:
         * 00 = finger not present
         * 01 = finger present and data accurate
         * 10 = finger present but data may be inaccurate
         * 11 = reserved
         */
#ifdef TYPE_B_PROTOCOL
        input_mt_slot(rmi4_data->input_dev, finger);
        input_mt_report_slot_state(rmi4_data->input_dev,
                MT_TOOL_FINGER, finger_status);
#endif

        if (finger_status) {
            data_offset = data_addr +
                num_of_finger_status_regs +
                (finger * data_reg_blk_size);
            retval = synaptics_rmi4_i2c_read(rmi4_data,
                    data_offset,
                    data,
                    data_reg_blk_size);
            if (retval < 0)
                return 0;

            x = (data[0] << 4) | (data[2] & MASK_4BIT);
            y = (data[1] << 4) | ((data[2] >> 4) & MASK_4BIT);
            wx = (data[3] & MASK_4BIT);
            wy = (data[3] >> 4) & MASK_4BIT;

            if (rmi4_data->board->swap_axes) {
                temp = x;
                x = y;
                y = temp;
                temp = wx;
                wx = wy;
                wy = temp;
            }

            if (rmi4_data->board->x_flip)
                x = rmi4_data->sensor_max_x - x;
            if (rmi4_data->board->y_flip)
                y = rmi4_data->sensor_max_y - y;

            input_report_key(rmi4_data->input_dev,
                    BTN_TOUCH, 1);
            input_report_key(rmi4_data->input_dev,
                    BTN_TOOL_FINGER, 1);
            input_report_abs(rmi4_data->input_dev,
                    ABS_MT_POSITION_X, x);
            input_report_abs(rmi4_data->input_dev,
                    ABS_MT_POSITION_Y, y);
#ifdef REPORT_2D_W
            input_report_abs(rmi4_data->input_dev,
                    ABS_MT_TOUCH_MAJOR, max(wx, wy));
            input_report_abs(rmi4_data->input_dev,
                    ABS_MT_TOUCH_MINOR, min(wx, wy));
#endif
#ifndef TYPE_B_PROTOCOL
            input_mt_sync(rmi4_data->input_dev);
#endif

            dev_dbg(&rmi4_data->i2c_client->dev,
                    "%s: Finger %d:\n"
                    "status = 0x%02x\n"
                    "x = %d\n"
                    "y = %d\n"
                    "wx = %d\n"
                    "wy = %d\n",
                    __func__, finger,
                    finger_status,
                    x, y, wx, wy);

            touch_count++;
        }
    }

    if (touch_count == 0) {
        input_report_key(rmi4_data->input_dev,
                BTN_TOUCH, 0);
        input_report_key(rmi4_data->input_dev,
                BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
        input_mt_sync(rmi4_data->input_dev);
#endif
    }

    input_sync(rmi4_data->input_dev);

    return touch_count;
}


/* Analog voltage @3.0 V */
#define SYNA_VTG_MIN_UV		3000000
#define SYNA_VTG_MAX_UV		3000000
#define SYNA_ACTIVE_LOAD_UA	15000
#define SYNA_LPM_LOAD_UA		10
/* i2c voltage @1.8 V */
#define SYNA_I2C_VTG_MIN_UV	1800000
#define SYNA_I2C_VTG_MAX_UV	1800000
#define SYNA_I2C_LOAD_UA		10000
#define SYNA_I2C_LPM_LOAD_UA	10


#define SYNA_NO_GESTURE					0x00
#define SYNA_ONE_FINGER_DOUBLE_TAP		0x03
#define SYNA_TWO_FINGER_SWIPE			0x07
#define SYNA_ONE_FINGER_CIRCLE			0x08
#define SYNA_ONE_FINGER_DIRECTION		0x0a
#define SYNA_ONE_FINGER_W_OR_M		    0x0b


#define KEY_F3			61  // Double click, light the screen 
#define KEY_F4			62  // cycle , open the carmer
#define KEY_F5			63  // v, open the flashlight
#define KEY_F6			64  // ||, stop the music
#define KEY_F7			65  // <, last music
#define KEY_F8			66  // >, next music 
#define KEY_F9			67  // M or W

#define UnkownGestrue       0
#define DouTap              1   // double tap
#define UpVee               2   // V
#define DownVee             3   // ^
#define LeftVee             4   // >
#define RightVee            5   // <
#define Circle              6   // O
#define DouSwip             7   // ||
#define Left2RightSwip      8   // -->
#define Right2LeftSwip      9   // <--
#define Up2DownSwip         10  // |v
#define Down2UpSwip         11  // |^
#define Mgestrue            12  // M
#define Wgestrue            13  // W

#define BLANK		1
#define UNBLANK		0

#define SYNA_SMARTCOVER_MIN     0
#define SYNA_SMARTCOVER_MAN     750


#define SYNA_ADDR_REPORT_FLAG        0x19  //report mode register
#define SYNA_ADDR_GESTURE_FLAG       0x1e  //gesture enable register
#define SYNA_ADDR_GLOVE_FLAG       	 0x1f  //glove enable register
#define SYNA_ADDR_GESTURE_OFFSET_s3320     0x07  //gesture register addr=0x08
#define SYNA_ADDR_GESTURE_OFFSET_s3508     0x07  //gesture register addr=0x07
#define SYNA_ADDR_GESTURE_EXT_S3508  0x402 //gesture ext data, for 13077 and 14001
#define SYNA_ADDR_GESTURE_EXT_S3528  0x400 //gesture ext data, for 14021
#define SYNA_ADDR_SMARTCOVER_EXT     0x41f //smartcover mode
#define SYNA_ADDR_PDOZE_FLAG         0x07  //pdoze status register
#define SYNA_ADDR_TOUCH_FEATURE      0x1E  //ThreeD Touch Features
#define SYNA_ADDR_F12_2D_CTRL23      0x1D
#define SYNA_ADDR_F12_2D_CTRL10      0x16

#define SYNA_ADDR_F54_ANALOG_CTRL113 0x136

extern int rmi4_fw_module_init(bool insert);

#define F54_CTRL_BASE_ADDR		(syna_rmi4_data->f54_ctrl_base_addr)
#define F54_CMD_BASE_ADDR		(syna_rmi4_data->f54_cmd_base_addr)
#define F54_DATA_BASE_ADDR		(syna_rmi4_data->f54_data_base_addr)

/*************** log definition **********************************/
#define TS_ERROR   1
#define TS_WARNING 2
#define TS_INFO    3
#define TS_DEBUG   4
#define TS_TRACE   5
static int syna_log_level = TS_INFO;
#define print_ts(level, ...) \
    do { \
        if (syna_log_level >= (level)) \
        printk(__VA_ARGS__); \
    } while (0) 
/*****************************************************************/

struct synaptics_rmi4_data *syna_rmi4_data=0;
EXPORT_SYMBOL(syna_rmi4_data);
static struct regulator *vdd_regulator=0;
static struct regulator *vdd_regulator_i2c=0;
int syna_use_gesture = 0 ;
static uint8_t int_mask = 0;
static int syna_test_max_err_count = 10;
static char synaptics_vendor_str[32];  //vendor string
static char *synaptics_id_str;
static unsigned int syna_lcd_ratio1 ;
static unsigned int syna_lcd_ratio2 ;
EXPORT_SYMBOL(syna_use_gesture);

/***** For virtual key definition begin *******************/
enum tp_vkey_enum
{
    TP_VKEY_MENU,
    TP_VKEY_HOME,
    TP_VKEY_BACK,

    TP_VKEY_NONE,
    TP_VKEY_COUNT = TP_VKEY_NONE,
};
static struct tp_vkey_button
{
    int x;
    int y;
    int width;
    int height;
}vkey_buttons[TP_VKEY_COUNT];

#define LCD_SENSOR_X  (1080)
#define LCD_SENSOR_Y  (2040)
#define LCD_MAX_X  (1080)
#define LCD_MAX_Y  (1920)
#define LCD_MAX_X_FIND7S  (1440)
#define LCD_MAX_Y_FIND7S  (2560)
#define LCD_MULTI_RATIO(m)   (((syna_lcd_ratio1)*(m))/(syna_lcd_ratio2))
#define VK_LCD_WIDTH  LCD_MULTI_RATIO(LCD_MAX_X/TP_VKEY_COUNT)   // 3 keys

#define VK_VENTE_Y  1974

static void vk_calculate_area(void)  //added by liujun
{
    int i;
    struct synaptics_rmi4_data *syna_ts_data = syna_rmi4_data ;
    int tp_max_x = syna_ts_data->sensor_max_x;
    int tp_max_y = syna_ts_data->sensor_max_y;
    int vk_height = syna_ts_data->virtual_key_height;
    int vk_width = tp_max_x/TP_VKEY_COUNT;
    int margin_x = 85;	


        syna_ts_data->vk_prop_center_y = 2626;
        syna_ts_data->vk_prop_height = 152;
        syna_ts_data->vk_prop_width = 200;


    print_ts(TS_INFO, "%s, max_x = %d,max_y = %d,virtual_kye_height = %d\n",  __func__, tp_max_x, tp_max_y, vk_height);

    for (i = 0; i < TP_VKEY_COUNT; ++i)
    {
        vkey_buttons[i].width = vk_width - margin_x*2;
        vkey_buttons[i].height = vk_height - 10;
        vkey_buttons[i].x = vk_width*i + margin_x;
        vkey_buttons[i].y = tp_max_y - vkey_buttons[i].height;

        printk("[syna][%d]w = %d,h = %d,x = %d,y = %d\n",i,vkey_buttons[i].width,vkey_buttons[i].height,vkey_buttons[i].x,vkey_buttons[i].y);
    }
    //vkey_buttons[TP_VKEY_MENU].x -= 20;
    vkey_buttons[TP_VKEY_BACK].x += 20;
}

static ssize_t vk_syna_show(struct kobject *kobj,
        struct kobj_attribute *attr, char *buf)
{
    struct synaptics_rmi4_data *ts = syna_rmi4_data;
    int len ;

    print_ts(TS_INFO, "%s\n", __func__);

    len =  sprintf(buf,
            __stringify(EV_KEY) ":" __stringify(KEY_MENU)  ":%d:%d:%d:%d"
            ":" __stringify(EV_KEY) ":" __stringify(KEY_HOMEPAGE)  ":%d:%d:%d:%d"
            ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)  ":%d:%d:%d:%d" "\n",
            VK_LCD_WIDTH/2 + 0,   ts->vk_prop_center_y, ts->vk_prop_width, ts->vk_prop_height,
            VK_LCD_WIDTH*3/2, ts->vk_prop_center_y, ts->vk_prop_width, ts->vk_prop_height,
            VK_LCD_WIDTH*5/2+20 , ts->vk_prop_center_y, ts->vk_prop_width, ts->vk_prop_height);

    return len ;
}

static struct kobj_attribute vk_syna_attr = {
    .attr = {
        .name = "virtualkeys."DRIVER_NAME,
        .mode = S_IRUGO,
    },
    .show = &vk_syna_show,
};

static struct attribute *syna_properties_attrs[] = {
    &vk_syna_attr.attr,
    NULL
};

static struct attribute_group syna_properties_attr_group = {
    .attrs = syna_properties_attrs,
};

static void synaptics_ts_init_area(struct synaptics_rmi4_data *ts){
    ts->snap_top = 0;
    ts->snap_left = 0;
    ts->snap_right = 0;
    ts->snap_bottom = 0 ;
}

static int synaptics_ts_init_virtual_key(struct synaptics_rmi4_data *ts )
{
    int ret = 0;
    
    vk_calculate_area();
    /* virtual keys */
    if(ts->properties_kobj || ts->sensor_max_x<=0 || ts->sensor_max_y<=0)
        return 0 ;
    if (1) /* delete it by lauson. */
    {
    ts->properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (ts->properties_kobj)
        ret = sysfs_create_group(ts->properties_kobj, &syna_properties_attr_group);
    }

    if (!ts->properties_kobj || ret)
        printk("%s: failed to create board_properties\n", __func__);
    /* virtual keys */
    return ret;
}

static int get_virtual_key_button(int x, int y)
{
    int i;
    int lcdheight = LCD_MAX_Y_FIND7S ;

    if(y <= lcdheight)
        return 0 ;	

    for (i = 0; i < TP_VKEY_NONE; ++i)
    {
        struct tp_vkey_button* button = &vkey_buttons[i];
        if ((x >= button->x) && (x <= button->x + button->width))
        {
            // In this button area.
            break;
        }
    }
    return i;
}
/***** For virtual key definition end *********************/

static int synaptics_set_f12ctrl_data(struct synaptics_rmi4_data *rmi4_data, bool enable, unsigned char suppression) 
{
    int retval;
    unsigned char val[4];
    unsigned char reportbuf[3];
    unsigned short reportaddr = SYNA_ADDR_REPORT_FLAG;  //F12_2D_CTRL register changed in new firmware

    retval = synaptics_rmi4_i2c_read(rmi4_data, reportaddr, reportbuf, sizeof(reportbuf));
    if (retval < 0) 
    {
        dev_err(&(rmi4_data->input_dev->dev), "%s: Failed to get report buffer\n", __func__);
        return -1 ;
    } 

    if(suppression) {
        if(suppression >= 200)
            suppression = 0 ;
        reportbuf[0] = suppression ;
        reportbuf[1] = suppression ;

    } else {
        retval = synaptics_rmi4_i2c_read(syna_rmi4_data,SYNA_ADDR_GESTURE_FLAG,val,sizeof(val));
        val[0] = syna_rmi4_data->gesture_enable & 0xff ;
        retval = synaptics_rmi4_i2c_write(syna_rmi4_data,SYNA_ADDR_GESTURE_FLAG,val,sizeof(val));

        if(enable)
            reportbuf[2] |= 0x02 ;
        else {
            reportbuf[2] &= 0xfd ;
            syna_use_gesture = (rmi4_data->gesture_enable&0xff)?1:0 ;
        }
    }

    retval = synaptics_rmi4_i2c_write(rmi4_data, reportaddr, reportbuf, sizeof(reportbuf));
    if (retval < 0) {
        dev_err(&(rmi4_data->input_dev->dev), "%s: Failed to write report buffer\n", __func__);
        return -1 ;
    }

    return 0 ;

}

static int synaptics_enable_gesture(struct synaptics_rmi4_data *rmi4_data, bool enable) 
{
    if(!syna_use_gesture)
        return 0 ;

    if(rmi4_data->gesture == enable)
        return 0 ;

    if(synaptics_set_f12ctrl_data(rmi4_data,enable,0) < 0)
        return -1 ;

    rmi4_data->gesture = enable ;
    print_ts(TS_DEBUG, KERN_ERR "[syna]:gesture source = 0x%x\n", rmi4_data->gesture_enable);

    return 0 ;

}

//enable pdoze function
static int synaptics_enable_pdoze(struct synaptics_rmi4_data *rmi4_data, bool enable) {
    unsigned char val[3] ;
    int retval ;

    if(!syna_rmi4_data->pdoze_enable)
        return 0 ;

    retval = synaptics_rmi4_i2c_read(syna_rmi4_data,SYNA_ADDR_REPORT_FLAG,val,sizeof(val));

    val[2] &= 0x7f ;
    if(enable)
        val[2] |= 0x80 ;

    retval = synaptics_rmi4_i2c_write(syna_rmi4_data,SYNA_ADDR_REPORT_FLAG,val,sizeof(val));

    retval = synaptics_rmi4_i2c_read(syna_rmi4_data,SYNA_ADDR_TOUCH_FEATURE,val,1);

    val[0] |= 0x10 ;
    if(enable)
        val[0] &= 0xEF ;

    retval = synaptics_rmi4_i2c_write(syna_rmi4_data,SYNA_ADDR_TOUCH_FEATURE,val,1);

    return 0;
}

//enable irq wakeup
static int synaptics_enable_irqwake(struct synaptics_rmi4_data *rmi4_data, bool enable) {
    if(enable) {
        enable_irq_wake(rmi4_data->i2c_client->irq);
    }
    else {
        disable_irq_wake(rmi4_data->i2c_client->irq);
    }

    return 0 ;
}

static ssize_t synaptics_rmi4_gesture_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%u\n",syna_use_gesture);
}

static ssize_t synaptics_rmi4_gesture_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int input;

    if (sscanf(buf, "%u", &input) != 1)
        return -EINVAL;
    printk("synap input:%d\n",input);
    if(input == 1) {
        synaptics_enable_gesture(syna_rmi4_data,true);
        syna_use_gesture = 1 ;
    } else if(input == 0) {
        synaptics_enable_gesture(syna_rmi4_data,false);
        syna_use_gesture = 0 ;
    }
    else
        return -EINVAL;

    return count;
}

//support tp2.0 interface, app read it to get points

static int synaptics_rmi4_coordinate_show(struct seq_file *seq, void *offset) 
{
	int len;
	
    print_ts(TS_ERROR, "%s, gesturemode = %d! (%d:%d),(%d:%d),(%d:%d),(%d:%d),(%d:%d),(%d:%d),%d\n", 
            __func__, syna_rmi4_data->gesturemode,                   
            syna_rmi4_data->points[0], syna_rmi4_data->points[1], syna_rmi4_data->points[2], syna_rmi4_data->points[3],                   
            syna_rmi4_data->points[4], syna_rmi4_data->points[5], syna_rmi4_data->points[6], syna_rmi4_data->points[7],                   
            syna_rmi4_data->points[8], syna_rmi4_data->points[9], syna_rmi4_data->points[10], syna_rmi4_data->points[11],
            syna_rmi4_data->points[12]);	

    len = seq_printf(seq, "%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d:%d,%d\n", syna_rmi4_data->gesturemode,                   
            syna_rmi4_data->points[0], syna_rmi4_data->points[1], syna_rmi4_data->points[2], syna_rmi4_data->points[3],                   
            syna_rmi4_data->points[4], syna_rmi4_data->points[5], syna_rmi4_data->points[6], syna_rmi4_data->points[7],                   
            syna_rmi4_data->points[8], syna_rmi4_data->points[9], syna_rmi4_data->points[10], syna_rmi4_data->points[11],
            syna_rmi4_data->points[12]);

    return 0 ;
}
static int synaptics_coordinate_open(struct inode *inode, struct file *file)
{
	return single_open(file, synaptics_rmi4_coordinate_show, inode->i_private);
}

//support pdoze status
static int synaptics_rmi4_pdoze_show(struct seq_file *seq, void *offset) 
{
    int len = 0 ;

    len = seq_printf(seq, "%d\n", syna_rmi4_data->pdoze_status);	

    return 0 ;
}
static int synaptics_pdoze_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, synaptics_rmi4_pdoze_show, inode->i_private);
}

static int synaptics_rmi4_gesture_proc_show(struct seq_file *seq, void *offset) 
{
    seq_printf(seq, "%d\n", ((syna_rmi4_data->gesture_enable)?1:0));
    return 0 ;
}

static int synaptics_gesture_open(struct inode *inode, struct file *file)
{
	return single_open(file, synaptics_rmi4_gesture_proc_show, inode->i_private);
}

static ssize_t synaptics_rmi4_gesture_proc_write(struct file *filp, const char __user *buff, size_t len, loff_t *lo) 
{
    unsigned char bak;
    unsigned int enable ;
    if(len > 2)
        return 0 ;

    enable = (buff[0]=='0')?0:1 ;
    bak = syna_rmi4_data->gesture_enable ;
    syna_rmi4_data->gesture_enable &= 0x00 ;
    if(enable)
        syna_rmi4_data->gesture_enable |= 0x7b ;
    if(bak == syna_rmi4_data->gesture_enable)
        return len ;

    if(!(syna_use_gesture && syna_rmi4_data->gesture))
        syna_use_gesture = (syna_rmi4_data->gesture_enable&0xff)?1:0 ;

    print_ts(TS_DEBUG, KERN_ERR "%s, gesture_enable = 0x%x\n", __func__, syna_rmi4_data->gesture_enable);

    return len;
}

#ifdef VENDOR_EDIT
//zhanhua.li@BSP.TP,2014/05/07,add smartcover function
//smartcover proc write function
static int synaptics_rmi4_open_smartcover( void )
{
    int retval;
    unsigned char val[10];

    retval = synaptics_rmi4_i2c_read(syna_rmi4_data,SYNA_ADDR_SMARTCOVER_EXT,val,1);    
    val[0] = 1;//syna_rmi4_data->smartcover_enable & 0xff ;
    retval = synaptics_rmi4_i2c_write(syna_rmi4_data,SYNA_ADDR_SMARTCOVER_EXT,val,1);

    //set 
    retval = synaptics_rmi4_i2c_read(syna_rmi4_data,SYNA_ADDR_GLOVE_FLAG,val,1);    
    val[0] |= 0x01;
    retval = synaptics_rmi4_i2c_write(syna_rmi4_data,SYNA_ADDR_GLOVE_FLAG,val,1);

    retval = synaptics_rmi4_i2c_read(syna_rmi4_data,SYNA_ADDR_F12_2D_CTRL10,val,7);    
    val[6] &= ~(0x01) ;//0b1111 1110
    retval = synaptics_rmi4_i2c_write(syna_rmi4_data,SYNA_ADDR_F12_2D_CTRL10,val,7);

    retval = synaptics_rmi4_i2c_read(syna_rmi4_data,SYNA_ADDR_F54_ANALOG_CTRL113,val,1);    
    val[0] &= ~(0x01<< 1) ; //0b1111 1101
    retval = synaptics_rmi4_i2c_write(syna_rmi4_data,SYNA_ADDR_F54_ANALOG_CTRL113,val,1);

    val[0] = 0x04 ;
    synaptics_rmi4_i2c_write(syna_rmi4_data, F54_CMD_BASE_ADDR, val,1); // force update
    wait_test_cmd_finished();
    printk(KERN_ERR " open smartcover\n");
    return retval ;
}

static int synaptics_rmi4_close_smartcover( void )
{
    int retval;
    unsigned char val[10];

    retval = synaptics_rmi4_i2c_read(syna_rmi4_data,SYNA_ADDR_SMARTCOVER_EXT,val,1);    
    val[0] = 0;//syna_rmi4_data->smartcover_enable & 0xff ;
    retval = synaptics_rmi4_i2c_write(syna_rmi4_data,SYNA_ADDR_SMARTCOVER_EXT,val,1);

    //set 0x001D bit5
    retval = synaptics_rmi4_i2c_read(syna_rmi4_data,SYNA_ADDR_GLOVE_FLAG,val,1);    
    val[0] &= ~(0x01);
    retval = synaptics_rmi4_i2c_write(syna_rmi4_data,SYNA_ADDR_GLOVE_FLAG,val,1);

    retval = synaptics_rmi4_i2c_read(syna_rmi4_data,SYNA_ADDR_F12_2D_CTRL10,val,7);    
    val[6] |= 0x01 ;
    retval = synaptics_rmi4_i2c_write(syna_rmi4_data,SYNA_ADDR_F12_2D_CTRL10,val,7);

    retval = synaptics_rmi4_i2c_read(syna_rmi4_data,SYNA_ADDR_F54_ANALOG_CTRL113,val,1);    
    val[0] |= 0x01<< 1 ;
    retval = synaptics_rmi4_i2c_write(syna_rmi4_data,SYNA_ADDR_F54_ANALOG_CTRL113,val,1);

    val[0] = 0x04 ;
    synaptics_rmi4_i2c_write(syna_rmi4_data, F54_CMD_BASE_ADDR, val,1); // force update
    wait_test_cmd_finished();
    printk(KERN_ERR "close smartcover\n");
    return retval;
}
#endif/*VENDOR_EDIT*/

//smartcover proc read function
static int synaptics_rmi4_proc_smartcover_show(struct seq_file *seq, void *offset) {
    seq_printf(seq, "%d\n", ((syna_rmi4_data->smartcover_enable)?1:0));
    return 0 ;
}

static int synaptics_smartcover_open(struct inode *inode, struct file *file)
{
	return single_open(file, synaptics_rmi4_proc_smartcover_show, inode->i_private);
}

static ssize_t synaptics_rmi4_proc_smartcover_write(struct file *file, const char __user *page, size_t t, loff_t *lo) {
    int retval;
    //	unsigned char val[10];
    unsigned char bak;
    unsigned int enable ;
    //user space interface,zhanhua.li@BSP.TP modify
    if(t > 2)
        return 0 ;
    printk(KERN_ERR "buff[0]=%x len=%ld\n",page[0],t);	
    if( !(page[0] == 0x31 || page[0] == 0x30) )	
        return t;

    printk(KERN_ERR "smartcover setting=%x \n",page[0]);
    enable =(page[0]=='0')?0:1 ;
    bak = syna_rmi4_data->smartcover_enable ;
    syna_rmi4_data->smartcover_enable &= 0x00 ;
    if(enable)
        syna_rmi4_data->smartcover_enable |= 0x01 ;
    if(bak == syna_rmi4_data->smartcover_enable)
        return t ;

    print_ts(TS_DEBUG, KERN_ERR "smartcover enable=0x%x\n", syna_rmi4_data->smartcover_enable);

    if ( enable) {
        retval = synaptics_rmi4_open_smartcover();
    } else { 
        retval = synaptics_rmi4_close_smartcover();
    }

    return t;
}

//glove proc read function
static int synaptics_rmi4_proc_glove_show(struct seq_file *seq, void *offset) {
    seq_printf(seq, "%d\n", ((syna_rmi4_data->glove_enable)?1:0));
    return 0 ;
}
static int synaptics_glove_open(struct inode *inode, struct file *file)
{
	return single_open(file, synaptics_rmi4_proc_glove_show, inode->i_private);
}
//glove proc write function
static ssize_t synaptics_rmi4_proc_glove_write(struct file *file, const char __user *page, size_t t, loff_t *lo) {
    int retval;
    unsigned char val[1];
    unsigned char bak;
    unsigned int enable ;
    if(t > 2)
        return 0 ;

    enable =(page[0]==0x30)?0:1 ;
    bak = syna_rmi4_data->glove_enable ;
    syna_rmi4_data->glove_enable &= 0x00 ;
    if(enable)
        syna_rmi4_data->glove_enable |= 0x01 ;
    if(bak == syna_rmi4_data->glove_enable)
        return t ;

    print_ts(TS_DEBUG, KERN_ERR "glove enable=0x%x\n", syna_rmi4_data->glove_enable);


    retval = synaptics_rmi4_i2c_read(syna_rmi4_data,SYNA_ADDR_GLOVE_FLAG,val,sizeof(val));


    val[0] = syna_rmi4_data->glove_enable & 0xff ;
    retval = synaptics_rmi4_i2c_write(syna_rmi4_data,SYNA_ADDR_GLOVE_FLAG,val,sizeof(val));

    return (retval==sizeof(val))?t:0;
}

//pdoze proc read function
static int synaptics_rmi4_proc_pdoze_show(struct seq_file *seq, void *offset)
{
    seq_printf(seq, "%d\n", ((syna_rmi4_data->pdoze_enable)?1:0));
    return 0 ;
}
static int synaptics_pdoze_open(struct inode *inode, struct file *file)
{
	return single_open(file, synaptics_rmi4_proc_pdoze_show, inode->i_private);
}

//pdoze proc write function
static ssize_t synaptics_rmi4_proc_pdoze_write(struct file *file, const char __user *page, size_t t, loff_t *lo) {
    unsigned int enable ;

    if(t > 2)
        return 0 ;

    enable =(page[0]==0x30)?0:1 ;

    syna_rmi4_data->pdoze_enable = enable ;

    print_ts(TS_DEBUG, KERN_ERR "[syna]:pdoze enable=0x%x\n", syna_rmi4_data->pdoze_enable);

    return t ;
}
static ssize_t synaptics_rmi4_baseline_data(char *buf, bool savefile);
static int synaptics_rmi4_proc_baseline_show(struct seq_file *seq, void *offset)
{
    unsigned char *buffer = 0 ;
	int rev = 0;
	
    buffer = kzalloc(1024, GFP_KERNEL);
    if(buffer) {
        synaptics_rmi4_baseline_data(buffer,0);
		rev = seq_printf(seq, buffer);
        kfree(buffer);
    }
    return rev ;
}
static int synaptics_rmi4_baseline_open(struct inode *inode, struct file *file)
{
	return single_open(file, synaptics_rmi4_proc_baseline_show, inode->i_private);
}
static int synaptics_rmi4_proc_vendor_id_show(struct seq_file *seq, void *offset)
{
    seq_printf(seq, "%d\n", syna_rmi4_data->vendor_id);
    return 0 ;
}
static int synaptics_rmi4_vendor_open(struct inode *inode, struct file *file)
{
	return single_open(file, synaptics_rmi4_proc_vendor_id_show, inode->i_private);
}

const struct file_operations synap[] =
{
	{
		.owner		= THIS_MODULE,
		.open		= synaptics_glove_open,
		.read		= seq_read,
		.write		= synaptics_rmi4_proc_glove_write,
		.llseek 	= seq_lseek,
		.release	= single_release,
	},
	
	{
		.owner		= THIS_MODULE,
		.open		= synaptics_gesture_open,
		.read		= seq_read,
		.write		= synaptics_rmi4_gesture_proc_write,
		.llseek 	= seq_lseek,
		.release	= single_release,
	},

	
	{
		.owner		= THIS_MODULE,
		.open		= synaptics_pdoze_open,
		.read		= seq_read,
		.write		= synaptics_rmi4_proc_pdoze_write,
		.llseek 	= seq_lseek,
		.release	= single_release,

	},
	
	{
		.owner		= THIS_MODULE,
		.open		= synaptics_smartcover_open,
		.read		= seq_read,
		.write		= synaptics_rmi4_proc_smartcover_write,
		.llseek 	= seq_lseek,
		.release	= single_release,
	},
	
	{
		.owner		= THIS_MODULE,
		.open		= synaptics_pdoze_status_open,
		.read		= seq_read,
		.llseek 	= seq_lseek,
		.release	= single_release,
	},
	
	{
		.owner		= THIS_MODULE,
		.open		= synaptics_coordinate_open,
		.read		= seq_read,
		.llseek 	= seq_lseek,
		.release	= single_release,
	},
	
	{
		.owner		= THIS_MODULE,
		.open		= synaptics_rmi4_baseline_open,
		.read		= seq_read,
		.llseek 	= seq_lseek,
		.release	= single_release,
	},
	
	{
		.owner		= THIS_MODULE,
		.open		= synaptics_rmi4_vendor_open,
		.read		= seq_read,
		.llseek 	= seq_lseek,
		.release	= single_release,
	}
};

static int synaptics_rmi4_init_touchpanel_proc(void)
{
    struct proc_dir_entry *proc_entry=0;

    struct proc_dir_entry *procdir = proc_mkdir( "touchpanel", NULL );
	
    //glove mode inteface	
   	proc_entry = proc_create_data("glove_mode_enable", 0666, procdir,&synap[0],NULL);

    proc_entry = proc_create_data("double_tap_enable", 0666, procdir,&synap[1],NULL);

    //for pdoze enable/disable interface
    proc_entry = proc_create_data("pdoze_mode_enable", 0666, procdir,&synap[2],NULL);

    //for smartcover
    proc_entry = proc_create_data("smartcover_mode_enable", 0666, procdir,&synap[3],NULL);

    //for pdoze status
    proc_entry = proc_create_data("pdozedetect", 0444, procdir,&synap[4],NULL);

    //for support tp2.0 coordirate
    proc_entry = proc_create_data("coordinate", 0444, procdir,&synap[5],NULL);

    //for baseline test
    proc_entry = proc_create_data("baseline_test", 0666, procdir,&synap[6],NULL);

    //for vendor id
    proc_entry = proc_create_data("vendor_id", 0444, procdir,&synap[7],NULL);

    printk("%s over.\n", __func__);

    return 0;
}

static inline void wait_test_cmd_finished(void)
{
    uint8_t data = 0;
    int i = 0 ;
    do {
        
        synaptics_rmi4_i2c_read(syna_rmi4_data, syna_rmi4_data->f54_cmd_base_addr, &data,1);
		mdelay(1); //wait 1ms
        if(i <= 80) {
            i ++;
        } else {
			print_ts(TS_WARNING,"synap %s time out!",__func__);
            break ;
        }
    } while (data != 0x00);
}

static void synaptics_set_int_mask(struct synaptics_rmi4_data *ts, int enable)
{
    uint8_t buf = 0;
    synaptics_rmi4_i2c_read(ts, ts->f01_data_base_addr+1, &buf,1);  //clear the interrupt

    if(enable)
        buf= int_mask;
    else 
        buf = 0x00;

    synaptics_rmi4_i2c_write(ts, ts->f01_ctrl_base_addr+1, &buf,1);
}

static ssize_t synaptics_rmi4_read_delta_data(char *buf)
{
    uint8_t tmp_new = 0;
    uint tx_num = 0;
    uint rx_num = 0;
    int16_t read_data;    
    uint16_t word_value;
    int x,y;
    ssize_t num_read_chars = 0;
    int ret = 0;

    struct synaptics_rmi4_data *syna_ts_data = syna_rmi4_data ;
    struct i2c_client* client = syna_ts_data->i2c_client;

    syna_rmi4_data->touch_stopped = true;

    disable_irq_nosync(client->irq);

    if (syna_ts_data->vendor_id == TP_VENDOR_WINTEK)
    {
        tx_num = TX_NUM_WINTEK;
        rx_num = RX_NUM_WINTEK;
    }
    else if (syna_ts_data->vendor_id == TP_VENDOR_TPK)
    {
        tx_num = TX_NUM_WINTEK;
        rx_num = RX_NUM_WINTEK;
    }
    else if (syna_ts_data->vendor_id == TP_VENDOR_TRULY)
    {
        tx_num = TX_NUM_TRULY_N3;
        rx_num = RX_NUM_TRULY_N3;
    }
    else if (syna_ts_data->vendor_id == TP_VENDOR_TPK_GFF)
    {
        tx_num = TX_NUM_TPK_GFF_N3;
        rx_num = RX_NUM_TPK_GFF_N3;
    }
    else if (syna_ts_data->vendor_id == TP_VENDOR_JDI)
    {
        tx_num = TX_NUM_JDI;
        rx_num = RX_NUM_JDI;
    }
    
    //set report type
    tmp_new = 0x02 ;
    synaptics_rmi4_i2c_write(syna_ts_data,F54_DATA_BASE_ADDR, &tmp_new,1);    

    word_value = 0 ;//set fifo 00 , first address 
    synaptics_rmi4_i2c_write(syna_ts_data,F54_DATA_BASE_ADDR+1,	(unsigned char*)&word_value,2);    

    tmp_new = 0x01 ;//send get report command 
    synaptics_rmi4_i2c_write(syna_ts_data, F54_CMD_BASE_ADDR, &tmp_new,1);     

    wait_test_cmd_finished();    

    for(x = 0;x < tx_num; x++) 
    {
        print_ts(TS_INFO,"\n[%d]",x);
        num_read_chars += sprintf(&(buf[num_read_chars]), "\n[%d]",x);
        for(y = 0; y < rx_num; y++) 
        {
            ret = i2c_smbus_read_word_data(client,F54_DATA_BASE_ADDR+3);
            read_data = ret & 0xffff;

            print_ts(TS_INFO,"%3d,",read_data);

            num_read_chars += sprintf(&(buf[num_read_chars]), "%d ",read_data);
        }
    }    

    synaptics_rmi4_reset_device(syna_ts_data, syna_ts_data->f01_cmd_base_addr);
    synaptics_rmi4_reinit_device(syna_rmi4_data);

    msleep(150);

    syna_rmi4_data->touch_stopped = false ;
    //enable_irq(client->irq);

    return num_read_chars;    
}

static ssize_t synaptics_rmi4_baseline_data_open_cbc(char *buf)
{
    int ret = 0;
    int x, y;
    int16_t read_data;
    uint8_t tmp_old = 0;
    uint8_t tmp_new = 0;
    ssize_t num_read_chars = 0;
    uint tx_num = 0;
    uint rx_num = 0;

    const int16_t *raw_cap_data = NULL;
    int16_t *cdata;
    uint16_t word_value;
    struct synaptics_rmi4_data *syna_ts_data = syna_rmi4_data ;
    struct i2c_client* client = syna_ts_data->i2c_client;

    syna_rmi4_data->touch_stopped = true;

    disable_irq_nosync(client->irq);

    if (syna_ts_data->vendor_id == TP_VENDOR_WINTEK)
    {
        tx_num = TX_NUM_WINTEK;
        rx_num = RX_NUM_WINTEK;
        raw_cap_data = (const int16_t *)raw_cap_data_wintek_9093;
    }
    else if (syna_ts_data->vendor_id == TP_VENDOR_TPK)
    {
        tx_num = TX_NUM_WINTEK;
        rx_num = RX_NUM_WINTEK;
        raw_cap_data = (const int16_t *)raw_cap_data_tpk;
    }
    else if (syna_ts_data->vendor_id == TP_VENDOR_TRULY)
    {
        tx_num = TX_NUM_TRULY_N3;
        rx_num = RX_NUM_TRULY_N3;
        raw_cap_data = (const int16_t *)raw_cap_data_turly_N3;
    }
    else if (syna_ts_data->vendor_id == TP_VENDOR_TPK_GFF)
    {
        tx_num = TX_NUM_TPK_GFF_N3;
        rx_num = RX_NUM_TPK_GFF_N3;
        raw_cap_data = (const int16_t *)raw_cap_data_tpk_gff_N3;
    }
    else if (syna_ts_data->vendor_id == TP_VENDOR_JDI)
    {
        tx_num = TX_NUM_JDI;
        rx_num = RX_NUM_JDI;
        raw_cap_data = (const int16_t *)raw_cap_data_JDI;
    }

    cdata = (int16_t *)raw_cap_data ;

    //set report type
    tmp_new = 0x03 ;
    synaptics_rmi4_i2c_write(syna_ts_data,F54_DATA_BASE_ADDR,	&tmp_new,1);

    //enable cbc 
    synaptics_rmi4_i2c_read(syna_ts_data, F54_CTRL_BASE_ADDR+23, &tmp_old,1);
    tmp_new = tmp_old | 0x10;
    synaptics_rmi4_i2c_write(syna_ts_data, F54_CTRL_BASE_ADDR +23, &tmp_new,1);

    // Forbid NoiseMitigation
    tmp_new = 0x01 ;
    synaptics_rmi4_i2c_write(syna_ts_data, F54_CTRL_BASE_ADDR +7, &tmp_new,1);
    tmp_new = 0x04 ;
    synaptics_rmi4_i2c_write(syna_ts_data, F54_CMD_BASE_ADDR, &tmp_new,1); // force update
    wait_test_cmd_finished();
    print_ts(TS_DEBUG, "Forbid NoiseMitigation oK, %d\n", __LINE__);
    tmp_new = 0x02 ;
    synaptics_rmi4_i2c_write(syna_ts_data, F54_CMD_BASE_ADDR, &tmp_new,1); // force cal
    wait_test_cmd_finished();
    print_ts(TS_DEBUG, "Force Cal oK\n");

    word_value = 0 ;//set fifo 00 , first address 
    synaptics_rmi4_i2c_write(syna_ts_data,F54_DATA_BASE_ADDR+1,	(unsigned char*)&word_value,2);
    tmp_new = 0x01 ;//send get report command 
    synaptics_rmi4_i2c_write(syna_ts_data, F54_CMD_BASE_ADDR, &tmp_new,1); 
    wait_test_cmd_finished();

    num_read_chars += sprintf(&(buf[num_read_chars]), "\n enable cbc:\n");

    for(x = 0;x < tx_num; x++)
    {
        num_read_chars += sprintf(&(buf[num_read_chars]), "\n[%d]",x);
        for(y = 0; y < rx_num; y++)
        {
            ret = i2c_smbus_read_word_data(client,F54_DATA_BASE_ADDR+3);
            read_data = ret & 0xffff;

            num_read_chars += sprintf(&(buf[num_read_chars]), "%d ",read_data);

            print_ts(TS_DEBUG, "Raw[%2d][%2d] = %4d,  ", x, y, read_data);
            print_ts(TS_DEBUG, "range:[%4d ~ %4d]\n", cdata[0], cdata[1]);

            cdata += 2 ;
        }

    }

    synaptics_rmi4_reset_device(syna_ts_data, syna_ts_data->f01_cmd_base_addr);
    synaptics_rmi4_reinit_device(syna_rmi4_data);

    msleep(150);

    syna_rmi4_data->touch_stopped = false ;
    //enable_irq(client->irq);

    return num_read_chars;

}

static ssize_t synaptics_rmi4_baseline_data_close_cbc(char *buf)
{
    int ret = 0;
    int x, y;
    int16_t read_data;
    uint8_t tmp_old = 0;
    uint8_t tmp_old2 = 0;
    uint8_t tmp_new = 0;
    ssize_t num_read_chars = 0;
    uint tx_num = 0;
    uint rx_num = 0;

    const int16_t *raw_cap_data = NULL;
    int16_t *cdata;
    uint16_t word_value;
    struct synaptics_rmi4_data *syna_ts_data = syna_rmi4_data ;
    struct i2c_client* client = syna_ts_data->i2c_client;

    syna_rmi4_data->touch_stopped = true;

    disable_irq_nosync(client->irq);

    if (syna_ts_data->vendor_id == TP_VENDOR_WINTEK)
    {
        tx_num = TX_NUM_WINTEK;
        rx_num = RX_NUM_WINTEK;
        raw_cap_data = (const int16_t *)raw_cap_data_wintek_9093;
    }
    else if (syna_ts_data->vendor_id == TP_VENDOR_TPK)
    {
        tx_num = TX_NUM_WINTEK;
        rx_num = RX_NUM_WINTEK;
        raw_cap_data = (const int16_t *)raw_cap_data_tpk;
    }
    else if (syna_ts_data->vendor_id == TP_VENDOR_TRULY)
    {
        tx_num = TX_NUM_TRULY_N3;
        rx_num = RX_NUM_TRULY_N3;
        raw_cap_data = (const int16_t *)raw_cap_data_turly_N3;
    }
    else if (syna_ts_data->vendor_id == TP_VENDOR_TPK_GFF)
    {
        tx_num = TX_NUM_TPK_GFF_N3;
        rx_num = RX_NUM_TPK_GFF_N3;
        raw_cap_data = (const int16_t *)raw_cap_data_tpk_gff_N3;
    }
    else if (syna_ts_data->vendor_id == TP_VENDOR_JDI)
    {
        tx_num = TX_NUM_JDI;
        rx_num = RX_NUM_JDI;
        raw_cap_data = (const int16_t *)raw_cap_data_JDI;
    }
    cdata = (int16_t *)raw_cap_data + tx_num * rx_num * 2 ;

    print_ts(TS_INFO, "F54_DATA_BASE_ADDR = 0x%x, F54_CTRL_BASE_ADDR = 0x%x, F54_CMD_BASE_ADDR = 0x%x.\n", F54_DATA_BASE_ADDR, F54_CTRL_BASE_ADDR, F54_CMD_BASE_ADDR);

    //set report type
    tmp_new = 0x03 ;
    synaptics_rmi4_i2c_write(syna_ts_data,F54_DATA_BASE_ADDR,	&tmp_new,1);

    tmp_new = 1;//disable cdm 
    synaptics_rmi4_i2c_write(syna_ts_data, F54_CTRL_BASE_ADDR +20, &tmp_new,1);

    //disable cbc 
    synaptics_rmi4_i2c_read(syna_ts_data, F54_CTRL_BASE_ADDR+23, &tmp_old,1);
    tmp_new = tmp_old & 0xef;
    synaptics_rmi4_i2c_write(syna_ts_data, F54_CTRL_BASE_ADDR +23, &tmp_new,1);

    synaptics_rmi4_i2c_read(syna_ts_data, F54_CTRL_BASE_ADDR+25, &tmp_old2,1);
    tmp_new = tmp_old2 & 0xdf; 
    print_ts(TS_DEBUG, "tmp_old =0x%x ,tmp_new = 0x%x\n", tmp_old2, tmp_new);
    synaptics_rmi4_i2c_write(syna_ts_data, F54_CTRL_BASE_ADDR +25, &tmp_new,1);

    tmp_new = 0x04 ;
    synaptics_rmi4_i2c_write(syna_ts_data, F54_CMD_BASE_ADDR, &tmp_new,1); // force update
    wait_test_cmd_finished();

    // Forbid NoiseMitigation
    tmp_new = 0x01 ;
    synaptics_rmi4_i2c_write(syna_ts_data, F54_CTRL_BASE_ADDR +7, &tmp_new,1);
    tmp_new = 0x04 ;
    synaptics_rmi4_i2c_write(syna_ts_data, F54_CMD_BASE_ADDR, &tmp_new,1); // force update
    wait_test_cmd_finished();
    print_ts(TS_DEBUG, "Forbid NoiseMitigation oK, %d\n", __LINE__);
    tmp_new = 0x02 ;
    synaptics_rmi4_i2c_write(syna_ts_data, F54_CMD_BASE_ADDR, &tmp_new,1); // force cal
    wait_test_cmd_finished();
    print_ts(TS_DEBUG, "Force Cal oK\n");

    word_value = 0 ;//set fifo 00
    synaptics_rmi4_i2c_write(syna_ts_data,F54_DATA_BASE_ADDR+1,	(unsigned char*)&word_value,2);
    tmp_new = 0x01 ;
    synaptics_rmi4_i2c_write(syna_ts_data, F54_CMD_BASE_ADDR, &tmp_new,1); 
    wait_test_cmd_finished();

    num_read_chars += sprintf(&(buf[num_read_chars]), "\n disable cbc:\n");        

    for(x = 0;x < tx_num; x++)
    {
        num_read_chars += sprintf(&(buf[num_read_chars]), "\n[%d]",x);
        for(y = 0; y < rx_num; y++)
        {
            ret = i2c_smbus_read_word_data(client,F54_DATA_BASE_ADDR+3);
            read_data = ret & 0xffff;

            num_read_chars += sprintf(&(buf[num_read_chars]), "%d ",read_data);

            print_ts(TS_DEBUG, "Raw[%2d][%2d] = %4d,  ", x, y, read_data);
            print_ts(TS_DEBUG, "range:[%4d ~ %4d]\n", cdata[0], cdata[1]);

            cdata += 2 ;
        }

    }

    synaptics_rmi4_reset_device(syna_ts_data, syna_ts_data->f01_cmd_base_addr);

    synaptics_rmi4_reinit_device(syna_rmi4_data);

    msleep(150);

    syna_rmi4_data->touch_stopped = false ;
    //enable_irq(client->irq);

    return num_read_chars;

}

static ssize_t synaptics_rmi4_baseline_data(char *buf, bool savefile)
{
    int ret = 0;
    int x, y, i;
    int16_t read_data;
    uint8_t tmp_old = 0;
    uint8_t tmp_old2 = 0;
    uint8_t tmp_new = 0;
    ssize_t num_read_chars = 0;
    uint tx_num = 0;
    uint rx_num = 0;
    uint tx_judge_num = 0;
    uint rx_judge_num = 0;    
    uint rx2rx_lower_limit = 0;
    uint rx2rx_upper_limit = 0;
    const int16_t *raw_cap_data = NULL;
    int16_t *cdata;
    int16_t *Rxdata = NULL;
    int error_count = 0;
    static bool isbaseline = 0 ;
    uint16_t word_value;
    struct synaptics_rmi4_data *syna_ts_data = syna_rmi4_data ;
    struct i2c_client* client = syna_ts_data->i2c_client;

    int fd = -1;
    struct timespec   now_time;
    struct rtc_time   rtc_now_time;
    uint8_t  data_buf[64];
    mm_segment_t old_fs;

    int  iCbcDataGroupe=0;
    int  iCbcDataSize=0;

    if(isbaseline == true) {
        return sprintf(&(buf[num_read_chars]), "-1 please wait to finish .\n");
    }

    syna_rmi4_data->touch_stopped = true ;

    isbaseline = true ;

    disable_irq_nosync(client->irq);

    if (syna_ts_data->vendor_id == TP_VENDOR_TRULY)
    {
        tx_num = TX_NUM_TRULY;
        rx_num = RX_NUM_TRULY;
        rx2rx_lower_limit = DiagonalLowerLimit_TRULY;
        rx2rx_upper_limit = DiagonalUpperLimit_TRULY;
        raw_cap_data = (const int16_t *)raw_cap_data_truly_3035;
        iCbcDataSize = sizeof(raw_cap_data_truly_3035);
    }
    else if (syna_ts_data->vendor_id == TP_VENDOR_WINTEK)
    {
        tx_num = TX_NUM_WINTEK;
        rx_num = RX_NUM_WINTEK;
        rx2rx_lower_limit = DiagonalLowerLimit_WINTEK;
        rx2rx_upper_limit = DiagonalUpperLimit_WINTEK;
        raw_cap_data = (const int16_t *)raw_cap_data_wintek_9093;
        iCbcDataSize = sizeof(raw_cap_data_wintek_9093);
    }
    else if (syna_ts_data->vendor_id == TP_VENDOR_TPK)
    {
        tx_num = TX_NUM_TPK;
        rx_num = RX_NUM_TPK;
        rx2rx_lower_limit = DiagonalLowerLimit_TPK;
        rx2rx_upper_limit = DiagonalUpperLimit_TPK;
        raw_cap_data = (const int16_t *)raw_cap_data_tpk;
        iCbcDataSize = sizeof(raw_cap_data_tpk);
    }
    else if (syna_ts_data->vendor_id == TP_VENDOR_YOUNGFAST)
    {
        tx_num = TX_NUM_YOUNGFAST;
        rx_num = RX_NUM_YOUNGFAST;
        rx2rx_lower_limit = DiagonalLowerLimit_YOUNGFAST;
        rx2rx_upper_limit = DiagonalUpperLimit_YOUNGFAST;
        raw_cap_data = (const int16_t *)raw_cap_data_youngfast;
        iCbcDataSize = sizeof(raw_cap_data_youngfast);
    }
    else if (syna_ts_data->vendor_id == TP_VENDOR_TPK_GFF)
    {
        tx_num = TX_NUM_TPK_GFF_N3;
        rx_num = RX_NUM_TPK_GFF_N3;
        rx2rx_lower_limit = DiagonalLowerLimit_TPK;
        rx2rx_upper_limit = DiagonalUpperLimit_TPK;        
        raw_cap_data = (const int16_t *)raw_cap_data_tpk_gff_N3;
        iCbcDataSize = sizeof(raw_cap_data_tpk_gff_N3);        
    }
    else if (syna_ts_data->vendor_id == TP_VENDOR_JDI)
    {
        tx_num = TX_NUM_JDI;
        rx_num = RX_NUM_JDI;
        rx2rx_lower_limit = DiagonalLowerLimit_JDI;
        rx2rx_upper_limit = DiagonalUpperLimit_JDI;        
        raw_cap_data = (const int16_t *)raw_cap_data_JDI;
        iCbcDataSize = sizeof(raw_cap_data_JDI);        
    }
    
    if (tx_num == 0 || rx_num == 0 || raw_cap_data == NULL
            || rx2rx_lower_limit == 0 || rx2rx_upper_limit == 0 || tx_num < rx_num)
    {
        num_read_chars += sprintf(&(buf[num_read_chars]), "%2d++ NO appropriate data for current tp.\n", ++error_count);
        goto END_TP_TEST;
    }

    print_ts(TS_DEBUG, "alloc test memory.\n");
    Rxdata = kmalloc(sizeof(int16_t)*rx_num*rx_num, GFP_KERNEL);
    if (Rxdata == NULL)
    {
        num_read_chars += sprintf(&(buf[num_read_chars]), "%2d++ alloc memory failed.\n", ++error_count);
        goto END_TP_TEST;
    }
    //memset(Rxdata, 0, sizeof(Rxdata));

	synaptics_set_int_mask(syna_ts_data, 0);
    //step1:check baseline  capacitance
    print_ts(TS_DEBUG, "-------------------Step 1: check baseline capacitance --------------------- \n");
    if(savefile) {
        getnstimeofday(&now_time);
        rtc_time_to_tm(now_time.tv_sec, &rtc_now_time);
        sprintf(data_buf, "/sdcard/tp_testlimit_%02d%02d%02d-%02d%02d%02d.csv", 
                (rtc_now_time.tm_year+1900)%100, rtc_now_time.tm_mon+1, rtc_now_time.tm_mday,
                rtc_now_time.tm_hour, rtc_now_time.tm_min, rtc_now_time.tm_sec);

        old_fs = get_fs();
        set_fs(KERNEL_DS);

        fd = sys_open(data_buf, O_WRONLY | O_CREAT | O_TRUNC, 0);
        if (fd < 0) {
            print_ts(TS_DEBUG, "Open log file '%s' failed.\n", data_buf);
            set_fs(old_fs);
        }

    }

    for(iCbcDataGroupe=0;iCbcDataGroupe<2;iCbcDataGroupe++)
    {
        if(0 == iCbcDataGroupe)       // enable cbc default, for enable cbc test  
        {
            cdata = (int16_t *)raw_cap_data ;
            //set report type
            tmp_new = 0x03 ;//set report type 0x03, for enable cbc
            synaptics_rmi4_i2c_write(syna_ts_data,F54_DATA_BASE_ADDR,	&tmp_new,1);

            // Forbid NoiseMitigation
            tmp_new = 0x01 ;
            synaptics_rmi4_i2c_write(syna_ts_data, F54_CTRL_BASE_ADDR +7, &tmp_new,1);

            tmp_new = 0x02 ;
            synaptics_rmi4_i2c_write(syna_ts_data, F54_CMD_BASE_ADDR, &tmp_new,1); // force cal
            wait_test_cmd_finished();
            print_ts(TS_DEBUG, "Force Cal oK\n");

            word_value = 0 ;//set fifo 00 , first address 
            synaptics_rmi4_i2c_write(syna_ts_data,F54_DATA_BASE_ADDR+1,	(unsigned char*)&word_value,2);
            tmp_new = 0x01 ;//send get report command 
            synaptics_rmi4_i2c_write(syna_ts_data, F54_CMD_BASE_ADDR, &tmp_new,1); 
            wait_test_cmd_finished();
        }	
        else if(1 == iCbcDataGroupe)  // disable cbc,  for disable cbc test 
        {
            if( iCbcDataSize < ( tx_num * rx_num * 2)  )
            {
                print_ts(TS_DEBUG, " Only one group test data  \n" );
                break ; 
            }
            else
            {
                cdata = (int16_t *)raw_cap_data + tx_num * rx_num * 2 ;

                //set report type
                tmp_new = 0x03 ;//set report type 0x03, for enable cbc
                synaptics_rmi4_i2c_write(syna_ts_data,F54_DATA_BASE_ADDR,   &tmp_new,1);

                //disable cbc 
                synaptics_rmi4_i2c_read(syna_ts_data, F54_CTRL_BASE_ADDR+21, &tmp_old,1);
                tmp_new = tmp_old & 0xef;
                synaptics_rmi4_i2c_write(syna_ts_data, F54_CTRL_BASE_ADDR +21, &tmp_new,1);

                tmp_new = 0x04 ;
                synaptics_rmi4_i2c_write(syna_ts_data, F54_CMD_BASE_ADDR, &tmp_new,1); // force update
                wait_test_cmd_finished();

                // Forbid NoiseMitigation
                tmp_new = 0x01 ;
                synaptics_rmi4_i2c_write(syna_ts_data, F54_CTRL_BASE_ADDR +7, &tmp_new,1);

                print_ts(TS_DEBUG, "Forbid NoiseMitigation oK, %d\n", __LINE__);
                tmp_new = 0x02 ;
                synaptics_rmi4_i2c_write(syna_ts_data, F54_CMD_BASE_ADDR, &tmp_new,1); // force cal
                wait_test_cmd_finished();
                print_ts(TS_DEBUG, "Force Cal oK\n");

                word_value = 0 ;//set fifo 00
                synaptics_rmi4_i2c_write(syna_ts_data,F54_DATA_BASE_ADDR+1,	(unsigned char*)&word_value,2);
                tmp_new = 0x01 ;
                synaptics_rmi4_i2c_write(syna_ts_data, F54_CMD_BASE_ADDR, &tmp_new,1); 
                wait_test_cmd_finished();
            }	
        }

        tx_judge_num = tx_num;
        rx_judge_num = rx_num;

        for(x = 0;x < tx_num; x++)
        {
			for(y = 0; y < rx_num; y++)
			{
				ret = i2c_smbus_read_word_data(client,F54_DATA_BASE_ADDR+3);
				read_data = ret & 0xffff;

				print_ts(TS_DEBUG, "Raw[%d][%d] = %d,  ", x, y, read_data);
				print_ts(TS_DEBUG, "range:[%d ~ %d], ", cdata[0], cdata[1]);

				if (fd >= 0)
				{
					sprintf(data_buf, "%d,", read_data);
					sys_write(fd, data_buf, strlen(data_buf));
				}

				if (read_data >= cdata[0] && read_data <= cdata[1])
				{
					print_ts(TS_DEBUG, "pass.\n");
				}
				else
				{
					print_ts(TS_ERROR, "NOT in range!!\n");

					if (error_count >= syna_test_max_err_count)
					{
						if (!savefile) {
							num_read_chars += sprintf(&(buf[num_read_chars]), "  == Reach max error count (%d), stop test.\n", syna_test_max_err_count);
							goto END_TP_TEST;
						}
					} else {
						num_read_chars += sprintf(&(buf[num_read_chars]), "%2d++ raw_cap[%02d][%02d]=%4d is not in range[%04d~%04d].\n",
								++error_count,x,y,read_data,cdata[0],cdata[1]);
					}

				}
				cdata += 2;
			}

			if (fd >= 0)
			{
				sys_write(fd, "\n", 1);
			}

		}
    }


    // step2   for  high impedance test
    print_ts(TS_DEBUG, "-------------------Step 2 : high resistance test--------------------- \n");
    print_ts(TS_DEBUG, "n");

    tmp_new = 1;// software reset TP
    synaptics_rmi4_i2c_write(syna_ts_data,syna_ts_data->f01_cmd_base_addr,(unsigned char*)&tmp_new,1);
    msleep(150);

    tmp_new = 4 ;//set report type 0x4, for disable cbc
    synaptics_rmi4_i2c_write(syna_ts_data,F54_DATA_BASE_ADDR,	&tmp_new,1);

    tmp_new = 1;//disable cdm 
    synaptics_rmi4_i2c_write(syna_ts_data, F54_CTRL_BASE_ADDR +20, &tmp_new,1);

    synaptics_rmi4_i2c_read(syna_ts_data, F54_CTRL_BASE_ADDR+23, &tmp_old,1);//disable cbc 
    tmp_new = tmp_old & 0xef;
    synaptics_rmi4_i2c_write(syna_ts_data, F54_CTRL_BASE_ADDR +23, &tmp_new,1);

    synaptics_rmi4_i2c_read(syna_ts_data, F54_CTRL_BASE_ADDR+25, &tmp_old2,1);
    tmp_new = tmp_old2 & 0xdf; 
    print_ts(TS_DEBUG, "tmp_old =0x%x ,tmp_new = 0x%x\n", tmp_old2, tmp_new);
    synaptics_rmi4_i2c_write(syna_ts_data, F54_CTRL_BASE_ADDR +25, &tmp_new,1);

    tmp_new = 0x04 ;
    synaptics_rmi4_i2c_write(syna_ts_data, F54_CMD_BASE_ADDR, &tmp_new,1); // force update
    wait_test_cmd_finished();

    word_value = 0 ;//set fifo 00 , first address 
    synaptics_rmi4_i2c_write(syna_ts_data,F54_DATA_BASE_ADDR+1,	(unsigned char*)&word_value,2);
    tmp_new = 0x01 ;//send get report command 
    synaptics_rmi4_i2c_write(syna_ts_data, F54_CMD_BASE_ADDR, &tmp_new,1); 
    wait_test_cmd_finished();

    //高阻测试项，主要的测试数据由3个WORD组成，读回数据除以1000后， Limit 分别为：（-1,0.45），（-1,0.45），（-0.5,0.02）
    for(i = 0;i < 3; i++)
    {
        int iTemp[2];
        ret = i2c_smbus_read_word_data(client,F54_DATA_BASE_ADDR+3); //is F54_DATA_BASE_ADDR+3   not F54_DATA_BASE_ADDR+i
        read_data = ret & 0xffff;

        if (fd >= 0)
        {
            sprintf(data_buf, "%d,", read_data);
            sys_write(fd, data_buf, strlen(data_buf));
        }

        if( (i==0) || (i==1) ) //(-1000 ,450)  , (-400,20)
        {
            iTemp[0] = (-1) *1000;
            iTemp[1] = 0.45*1000;
        }
        else if(i==2)		
        {
            iTemp[0] = (-0.5) *1000;
            iTemp[1] = 0.02*1000;
        }
        if (read_data >= iTemp[0] && read_data <= iTemp[1])
        {
            print_ts(TS_DEBUG, "pass.\n");
        }
        else
        {
            print_ts(TS_ERROR,"========= High Resistance Raw NOT in range, Raw[%d] = %d \n ", i, read_data);
            num_read_chars += sprintf(&(buf[num_read_chars]), "=========== High Resistance Raw NOT in range, Raw[%d] = %d \n ", i, read_data);
            error_count ++; // in app , pass ---> error_count = 0  , fail ---> error_count = not zero
        }

    }

    //Step3 : Check trx-to-trx ,short test 
    print_ts(TS_DEBUG, "-------------------Step 3 : Check trx-to-trx--------------------- \n");

    tmp_new = 1;// software reset TP
    synaptics_rmi4_i2c_write(syna_ts_data,syna_ts_data->f01_cmd_base_addr,(unsigned char*)&tmp_new,1);
    msleep(150);

    tmp_new = 26 ;
    synaptics_rmi4_i2c_write(syna_ts_data,F54_DATA_BASE_ADDR,	(unsigned char*)&tmp_new,1); //select report type 0x26

    tmp_new = 0 ;
    data_buf[0] = 0 ;
    data_buf[1] = 0 ;
    synaptics_rmi4_i2c_write(syna_ts_data,F54_DATA_BASE_ADDR+1,	(unsigned char*)data_buf,2);
    tmp_new = 1 ;
    synaptics_rmi4_i2c_write(syna_ts_data,F54_CMD_BASE_ADDR,	(unsigned char*)&tmp_new,1); //get report
    wait_test_cmd_finished();

    y = synaptics_rmi4_i2c_read(syna_ts_data, F54_DATA_BASE_ADDR+3, data_buf,7) ;
    print_ts(TS_DEBUG," trx-to-trx raw readback: %x  %x  %x  %x  %x  %x \n",  data_buf[0],data_buf[1],data_buf[2],data_buf[3],data_buf[4],data_buf[5]);
    num_read_chars += sprintf(&(buf[num_read_chars]), " trx-to-trx raw readback: %x  %x  %x  %x  %x  %x \n", data_buf[0],data_buf[1],data_buf[2],data_buf[3],data_buf[4],data_buf[5]);
	if((data_buf[0]==0) && (data_buf[1]==0) && (data_buf[2]==0)\
	        &&( (data_buf[3]&0x07)==0)  && (data_buf[4]==0) && ((data_buf[5]&0x3f)==0))		
	{
	    print_ts(TS_DEBUG, " trx-to-trx test pass.\n");
	}
	else 
	{
		print_ts(TS_ERROR," trx-to-trx test error: data_buf[3]&0x07 =%x data_buf[5]&0x3f=%x \n",data_buf[3]&0x07,data_buf[5]&0x3f) ;
		num_read_chars += sprintf(&(buf[num_read_chars]), " trx-to-trx test error: data_buf[3]&0x07 =%x data_buf[5]&0x3f=%x \n",data_buf[3]&0x07,data_buf[5]&0x3f) ;
		error_count ++;
	}

    //Step4 : Check trx-to-ground 
    print_ts(TS_DEBUG, "-------------------step 4 : Check trx-to-ground--------------------- \n");

    tmp_new = 1;// software reset TP
    synaptics_rmi4_i2c_write(syna_ts_data,syna_ts_data->f01_cmd_base_addr,(unsigned char*)&tmp_new,1);
    msleep(150);

    tmp_new = 25 ;
    synaptics_rmi4_i2c_write(syna_ts_data,F54_DATA_BASE_ADDR,	(unsigned char*)&tmp_new,1); //select report type 0x25

    tmp_new = 0 ;
    data_buf[0] = 0 ;
    data_buf[1] = 0 ;
    synaptics_rmi4_i2c_write(syna_ts_data,F54_DATA_BASE_ADDR+1,	(unsigned char*)data_buf,2);
    tmp_new = 1 ;
    synaptics_rmi4_i2c_write(syna_ts_data,F54_CMD_BASE_ADDR,	(unsigned char*)&tmp_new,1); //get report
    wait_test_cmd_finished();

    y = synaptics_rmi4_i2c_read(syna_ts_data, F54_DATA_BASE_ADDR+3, data_buf,7) ;
    print_ts(TS_DEBUG,"trx-to-ground raw readback: %x  %x  %x  %x  %x  %x \n",  data_buf[0],data_buf[1],data_buf[2],data_buf[3],data_buf[4],data_buf[5]);

    print_ts(TS_DEBUG, "%d: syna_ts_data->vendor_id=%d,data_buf[4]=%d\n",__LINE__,syna_ts_data->vendor_id,data_buf[4]);

    if((data_buf[0]==0x00) && (data_buf[1]==0xa0) && (data_buf[2]==0xff)\
            &&( (data_buf[3]&0x07)==0x7)  && (data_buf[4]==0x00) && (data_buf[5]==0x00))		
    {
        print_ts(TS_DEBUG, "pass.\n");
    }
    else 
    {
        print_ts(TS_ERROR, "%d: syna_ts_data->vendor_id=%d,data_buf[4]=%d\n",__LINE__,syna_ts_data->vendor_id,data_buf[4]);
        print_ts(TS_ERROR, " trx-to-ground  Test Failed [%d]\n",__LINE__);
        num_read_chars += sprintf(&(buf[num_read_chars]), " trx-to-ground test Failed[%d]\n",__LINE__);
        error_count ++;
    }
    //step 5:reset touchpanel and reconfig the device
END_TP_TEST:
    if (fd >= 0) {
        sys_close(fd);
        set_fs(old_fs);
    }

    if (Rxdata)
    {
        print_ts(TS_DEBUG, "Release test memory.\n");
        kfree(Rxdata);
    }

    printk(error_count ? " **********  TP auto  test error  ********** " : " **********  TP All test passed ********** ");
    num_read_chars += sprintf(&(buf[num_read_chars]), "imageid=0x%x,deviceid=0x%x\n", syna_ts_data->image_cid,syna_ts_data->device_cid);

    num_read_chars += sprintf(&(buf[num_read_chars]), "%d error(s). %s\n", error_count, error_count?"":" **********   All test passed. ********** ");
    synaptics_rmi4_reset_device(syna_ts_data, syna_ts_data->f01_cmd_base_addr);

    msleep(150);
	synaptics_set_int_mask(syna_ts_data, 1);

    syna_rmi4_data->touch_stopped = false ;
    //enable_irq(client->irq);

    isbaseline = false ;

    return num_read_chars;
}

static ssize_t synaptics_rmi4_baseline_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int input;
    unsigned char *buffer = 0 ;

    if (sscanf(buf, "%u", &input) != 1)
        return -EINVAL;

    if(input == 21) {
        buffer = kzalloc(1024, GFP_KERNEL);
        if(buffer) {
            synaptics_rmi4_baseline_data(buffer,1);
            kfree(buffer);
        }
    }

    return count ;
}
static ssize_t synaptics_attr_loglevel_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    //add test code. for tp is invalid
    unsigned int value = gpio_get_value(syna_rmi4_data->irq_gpio);
    return sprintf(buf, "%d:0x%x\n", syna_log_level,value);
}
static ssize_t synaptics_attr_loglevel_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int val = 0;
    sscanf(buf, "%d", &val);
    syna_log_level = val;
    printk("[synaptics] set log level : %d\n", val);
    return count;
}

static ssize_t synaptics_rmi4_vendor_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return sprintf(buf, "%d\n", syna_rmi4_data->vendor_id); 
}

static ssize_t synaptics_rmi4_delta_data_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return synaptics_rmi4_read_delta_data(buf);
}

static ssize_t synaptics_rmi4_baseline_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return synaptics_rmi4_baseline_data(buf,0);
}

static ssize_t synaptics_rmi4_read_baseline_data_open_cbc(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return synaptics_rmi4_baseline_data_open_cbc(buf);
}

static ssize_t synaptics_rmi4_read_baseline_data_close_cbc(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return synaptics_rmi4_baseline_data_close_cbc(buf);
}

static ssize_t sense_frequency_selection_reg_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    unsigned char value ;

    synaptics_rmi4_i2c_read(syna_rmi4_data, 0x010B, &value,1);
    return sprintf(buf, "sense_frequency_selection_reg 0x10b = 0x%x\n", value);
}
static ssize_t sense_frequency_selection_reg_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    unsigned int val = 0;
    sscanf(buf, "%d", &val);
    return count;
}

static struct synaptics_dsx_cap_button_map button_map = {
    .nbuttons = 0,
    .map = "NULL",
};


static struct synaptics_dsx_platform_data dsx_platformdata = {	
    .irq_flags = IRQF_TRIGGER_FALLING|IRQF_ONESHOT,	
    .reset_delay_ms = 100,	
    .cap_button_map = &button_map,
    .regulator_en = 1,
};

static int synaptics_parse_dt(struct device *dev, struct synaptics_rmi4_data *ts)
{
    int ret = 0;
    if (dev->of_node)
    {
        struct device_node *np = dev->of_node;

        /* reset, irq gpio info */
        ts->irq_gpio    = of_get_named_gpio(np, "synaptics,irq-gpio", 0);   
	if( ts->irq_gpio < 0 ){
		printk("ts->irq_gpio not specified\n");	
	}        
        ts->reset_gpio  = of_get_named_gpio(np, "synaptics,reset-gpio", 0);    
	if( ts->reset_gpio < 0 ){
		printk("ts->reset_gpio not specified\n");	
	}  
	/*
        ts->wakeup_gpio = of_get_named_gpio(np, "synaptics,wakeup-gpio", 0);    
	if( ts->wakeup_gpio < 0 ){
		printk("ts->wakeup_gpio not specified\n");	
	}        
        ts->id_gpio     = of_get_named_gpio(np, "synaptics,id-gpio", 0);      
	if( ts->id_gpio < 0 ){
		printk("ts->id_gpio not specified\n");	
	}
	*/
       printk("%s irq_gpio:%d reset_gpio:%d  wakeup_gpio:%d id_gpio:%d  \n", __func__, ts->irq_gpio, ts->reset_gpio, ts->wakeup_gpio, ts->id_gpio);

	vdd_regulator = regulator_get(dev, "vdd_2v8");
	if( IS_ERR(vdd_regulator) ){
		ret = PTR_ERR(vdd_regulator);
		printk("%s Regulator get failed vdd rc=%d\n", __func__, ret);	
	}	
	
	vdd_regulator_i2c = regulator_get(dev, "vcc_i2c_1v8");
	if( IS_ERR(vdd_regulator_i2c) ){
		ret = PTR_ERR(vdd_regulator_i2c);
		printk("%s Regulator get failed vdd_i2c rc=%d\n", __func__, ret);	
	}

    }
    return ret;
}

int synaptics_regulator_configure(bool on)
{
    int rc = -1;

    if (on == false)
        goto hw_shutdown;

    if(!vdd_regulator) {
        //vdd_regulator = regulator_get(NULL, "8941_l22");
        if (IS_ERR(vdd_regulator)) {
            rc = PTR_ERR(vdd_regulator);
            printk("Regulator get failed vcc_ana rc=%d\n", rc);
            return rc;
        }
    }
    if (regulator_count_voltages(vdd_regulator) > 0) {
        rc = regulator_set_voltage(vdd_regulator, SYNA_VTG_MIN_UV, SYNA_VTG_MAX_UV);
        if (rc) {
            printk( "regulator set_vtg failed rc=%d\n", rc);
            goto error_set_vtg_vcc_ana;
        }
        rc = regulator_enable(vdd_regulator);
    }

    //vdd_regulator_i2c = regulator_get(NULL, "8941_l14");
    if (IS_ERR(vdd_regulator_i2c)) {
        rc = PTR_ERR(vdd_regulator_i2c);
        printk( "Regulator get failed rc=%d\n",	rc);
        goto error_get_vtg_i2c;
    }
    if (regulator_count_voltages(vdd_regulator_i2c) > 0) {
        rc = regulator_set_voltage(vdd_regulator_i2c, SYNA_I2C_VTG_MIN_UV, SYNA_I2C_VTG_MAX_UV);
        if (rc) {
            printk( "regulator set_vtg failed rc=%d\n", rc);
            goto error_set_vtg_i2c;
        }
         rc = regulator_enable(vdd_regulator_i2c);
    }

    return 0;

error_set_vtg_i2c:
    regulator_put(vdd_regulator_i2c);
error_get_vtg_i2c:
    if (regulator_count_voltages(vdd_regulator) > 0)
        regulator_set_voltage(vdd_regulator, 0, 0);

error_set_vtg_vcc_ana:
    regulator_put(vdd_regulator);
    return rc;

hw_shutdown:
    if (regulator_count_voltages(vdd_regulator) > 0)
        regulator_set_voltage(vdd_regulator, 0, 0);
    regulator_put(vdd_regulator);
    //vdd_regulator = 0 ;

    if (regulator_count_voltages(vdd_regulator_i2c) > 0)
        regulator_set_voltage(vdd_regulator_i2c, 0, 0);
    regulator_put(vdd_regulator_i2c);
    //vdd_regulator_i2c = 0 ;

    return 0;
} 

static unsigned char synaptics_rmi4_update_gesture(unsigned char *gesture,unsigned char *gestureext) {
    int i ;
    unsigned char keyvalue = 0 ;
    unsigned char gesturemode = UnkownGestrue ;
    unsigned short points[16];

    for(i = 0 ; i < 2*6; i++) {
        points[i] = ((unsigned short)gestureext[i*2]) | (((unsigned short)gestureext[i*2+1])<<8) ;
    }

    points[i] = 
        (gestureext[24] == 0x10) ? 1 :
        (gestureext[24] == 0x20) ? 0 :
        2 ;// 1--clockwise, 0--anticlockwise, not circle, report 2

    print_ts(TS_ERROR, KERN_ERR "%s, gesture[0] = 0x%x, points[12] = %d (%s)\n", __func__,gesture[0], points[12], 
                                (points[12] == 1) ? "clockwise" : 
                                (points[12] == 0) ? "anticlockwise" :
                                                    "not circle");
    switch(gesture[0]) {
        case SYNA_ONE_FINGER_CIRCLE:
            gesturemode = Circle ;
            keyvalue = KEY_F4;
            break ;

        case SYNA_TWO_FINGER_SWIPE:
            gesturemode = 
                (gestureext[24] == 0x41) ? Left2RightSwip   :
                (gestureext[24] == 0x42) ? Right2LeftSwip   :
                (gestureext[24] == 0x44) ? Up2DownSwip      :
                (gestureext[24] == 0x48) ? Down2UpSwip      :
                (gestureext[24] == 0x80) ? DouSwip          :
                UnkownGestrue;

            if( gesturemode==DouSwip ||
                    gesturemode==Down2UpSwip ||
                    gesturemode==Up2DownSwip) {
                if(abs(points[3] - points[1]) <= 800)
                    gesturemode=UnkownGestrue;
            }

            if(gesturemode!=UnkownGestrue)	{  
                keyvalue = KEY_F6;
            } 
            break ;

        case SYNA_ONE_FINGER_DOUBLE_TAP:
            gesturemode = DouTap ;
            keyvalue = KEY_F3;
            break ;

        case SYNA_ONE_FINGER_DIRECTION:
            switch(gesture[2]){
                case 0x01:  //UP
                    gesturemode = DownVee ;
                    keyvalue = KEY_F5;
                    break;
                case 0x02:  //DOWN
                    gesturemode = UpVee ;
                    keyvalue = KEY_F5;
                    break;
                case 0x04:  //LEFT
                    gesturemode = RightVee ;
                    keyvalue = KEY_F7;
                    break;
                case 0x08:  //RIGHT
                    gesturemode = LeftVee ;
                    keyvalue = KEY_F8;
                    break;
            }
            break;

        case SYNA_ONE_FINGER_W_OR_M:
            gesturemode = 
                (gesture[2] == 0x77 && gesture[3] == 0x00) ? Wgestrue :
                (gesture[2] == 0x6d && gesture[3] == 0x00) ? Mgestrue :
                UnkownGestrue;

            keyvalue = KEY_F9;
            break ;
    }

    if(gesturemode != UnkownGestrue) 
    {
        syna_rmi4_data->gesturemode = gesturemode ;
        memcpy(syna_rmi4_data->points,points,sizeof(syna_rmi4_data->points));
    } 
    else 
    {
        keyvalue = 0 ;
    }

    print_ts(TS_ERROR, "%s, gesture keyvalue = %d, gesturemode = %d (%s).\n", __func__, keyvalue, gesturemode, 
                        (gesturemode ==  0) ? "UnkownGestrue" :
                        (gesturemode ==  1) ? "Double Tap" :
                        (gesturemode ==  2) ? "V" :
                        (gesturemode ==  3) ? "^" :
                        (gesturemode ==  4) ? ">" :
                        (gesturemode ==  5) ? "<" :
                        (gesturemode ==  6) ? "O" :
                        (gesturemode ==  7) ? "||" :
                        (gesturemode ==  8) ? "-->" :
                        (gesturemode ==  9) ? "<--" :
                        (gesturemode == 10) ? "|V" :
                        (gesturemode == 11) ? "|^" :
                        (gesturemode == 12) ? "M" :
                                              "W" );

    return keyvalue ;
}

/**
 * synaptics_rmi4_f12_abs_report()
 *
 * Called by synaptics_rmi4_report_touch() when valid Function $12
 * finger data has been detected.
 *
 * This function reads the Function $12 data registers, determines the
 * status of each finger supported by the Function, processes any
 * necessary coordinate manipulation, reports the finger data to
 * the input subsystem, and returns the number of fingers detected.
 */
static int synaptics_rmi4_f12_abs_report(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler)
{
    int retval;
    unsigned char touch_count = 0; /* number of touch points */
    unsigned char finger;
    unsigned char fingers_to_process;
    unsigned char finger_status;
    unsigned char size_of_2d_data;
    unsigned short data_addr;
    int x;
    int y;
    int wx;
    int wy;
    int temp;
    struct synaptics_rmi4_f12_extra_data *extra_data;
    struct synaptics_rmi4_f12_finger_data *data;
    struct synaptics_rmi4_f12_finger_data *finger_data;
    unsigned char gesture[5];
    unsigned char gestureext[25];
    unsigned char keyvalue;
    unsigned int  finger_info = 0;

    fingers_to_process = fhandler->num_of_data_points;
    data_addr = fhandler->full_addr.data_base;
    extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
    size_of_2d_data = sizeof(struct synaptics_rmi4_f12_finger_data);

    if(rmi4_data->gesture ) 
    {
        retval = synaptics_rmi4_i2c_read(rmi4_data, SYNA_ADDR_GESTURE_OFFSET_s3320, gesture, sizeof(gesture));
        retval = synaptics_rmi4_i2c_read(rmi4_data, SYNA_ADDR_GESTURE_EXT_S3528, gestureext, sizeof(gestureext));

		if(gesture[0]==0x0c) //mingqiang.guo@phone.bsp 厂家新添加了三击手势代号为0x0c，OPPO未用到，这个会影响到双击亮屏成功率，
							 //当成双击亮屏处理,双击或者多击成功率提高，与以前量产固件保持一致
			gesture[0]=0x03;
		
		if(gesture[0] != 0) //如果为双击唤醒手势中断，持有该wake_lock,如果当前正在跑suspend流程，将会调用resume唤醒系统,放弃suspend
		   wake_lock_timeout(&tp_wake_lock, 5 * HZ);

		if(gesture[0]) 
		{
			keyvalue = synaptics_rmi4_update_gesture(gesture,gestureext);
            if(keyvalue)
            {
                keyvalue = KEY_F4 ;

                print_ts(TS_ERROR, KERN_ERR"report gesture keyvalue!\n");

                input_report_key(rmi4_data->input_dev, keyvalue, 1);
                input_sync(rmi4_data->input_dev);
                input_report_key(rmi4_data->input_dev, keyvalue, 0);
                input_sync(rmi4_data->input_dev);
            }
        }
    }

    //check pdoze status
    if(rmi4_data->pdoze_enable) 
    {
        retval = synaptics_rmi4_i2c_read(rmi4_data, SYNA_ADDR_PDOZE_FLAG, &keyvalue, 1);
        keyvalue = (keyvalue & 0x60)?1:0 ;
        rmi4_data->pdoze_status = keyvalue ;
        print_ts(TS_DEBUG, KERN_ERR "[syna]pdoze status: %d\n",rmi4_data->pdoze_status);
    }
    
    retval = synaptics_rmi4_i2c_read(rmi4_data, data_addr + extra_data->data1_offset, (unsigned char *)fhandler->data, fingers_to_process * size_of_2d_data);
    if (retval < 0)
	{	
		printk(KERN_ERR"[TP]%s:%d i2c communication error \n ",__func__,__LINE__);
        return 0;
	}
    data = (struct synaptics_rmi4_f12_finger_data *)fhandler->data;

    for (finger = 0; finger < fingers_to_process; finger++) 
    {
        finger_data = data + finger;
        finger_status = finger_data->object_type_and_status & MASK_2BIT;
        finger_info <<= 1;
        //print_ts(TS_DEBUG, KERN_ERR"%s finger:%d  finger_status:%d  object_type_and_status:%d \n", __func__, finger, finger_status, finger_data->object_type_and_status);
        if (finger_status) 
        {
            x = (finger_data->x_msb << 8) | (finger_data->x_lsb);
            y = (finger_data->y_msb << 8) | (finger_data->y_lsb);
#ifdef REPORT_2D_W
            wx = finger_data->wx;
            wy = finger_data->wy;
#endif

            if (rmi4_data->board->swap_axes) {
                temp = x;
                x = y;
                y = temp;
                temp = wx;
                wx = wy;
                wy = temp;
            }

            if (rmi4_data->board->x_flip)
                x = rmi4_data->sensor_max_x - x;
            if (rmi4_data->board->y_flip)
                y = rmi4_data->sensor_max_y - y;
            
            {
                int pressed_vkey = get_virtual_key_button(x, y);

                //print_ts(TS_DEBUG, KERN_ERR"%s: virtual_key = %d\n", __func__, pressed_vkey);

                if (pressed_vkey == TP_VKEY_NONE)
                {
                    continue;
                }
            }

            input_mt_slot(rmi4_data->input_dev, finger);
            input_mt_report_slot_state(rmi4_data->input_dev, MT_TOOL_FINGER, finger_status);
            input_report_key(rmi4_data->input_dev, BTN_TOUCH, 1);
            input_report_key(rmi4_data->input_dev, BTN_TOOL_FINGER, 1);
            input_report_abs(rmi4_data->input_dev, ABS_MT_POSITION_X, x);
            input_report_abs(rmi4_data->input_dev, ABS_MT_POSITION_Y, y);
#ifdef REPORT_2D_W
            input_report_abs(rmi4_data->input_dev, ABS_MT_TOUCH_MAJOR, max(wx, wy));
            input_report_abs(rmi4_data->input_dev, ABS_MT_TOUCH_MINOR, min(wx, wy));
#endif
#ifndef TYPE_B_PROTOCOL
            input_mt_sync(rmi4_data->input_dev);
#endif

			print_ts(TS_DEBUG, KERN_ERR 
					"%s: Finger %d: status = 0x%02x "
					"x = %4d, y = %4d, wx = %2d, wy = %2d\n",
					__func__, finger, finger_status, x, y, wx, wy);

            touch_count++;
            finger_info |= 1 ;
        }
    }

    for (finger = 0; finger < fingers_to_process; finger++) 
    {
        finger_status = (finger_info>>(fingers_to_process-finger-1)) & 1 ;
        if(!finger_status) 
        {
            input_mt_slot(rmi4_data->input_dev, finger);
            input_mt_report_slot_state(rmi4_data->input_dev, MT_TOOL_FINGER, finger_status);
        }
    }

    if (touch_count == 0) 
    {
        input_report_key(rmi4_data->input_dev,BTN_TOUCH, 0);
        print_ts(TS_DEBUG,"all finger up\n");
        input_report_key(rmi4_data->input_dev, BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
        input_mt_sync(rmi4_data->input_dev);
#endif
    }

    input_sync(rmi4_data->input_dev);

    return touch_count;
}

static void synaptics_rmi4_f1a_report(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler)
{
    int retval;
    unsigned char touch_count = 0;
    unsigned char button;
    unsigned char index;
    unsigned char shift;
    unsigned char status;
    unsigned char *data;
    unsigned short data_addr = fhandler->full_addr.data_base;
    struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;
    static unsigned char do_once = 1;
    static bool current_status[MAX_NUMBER_OF_BUTTONS];
#ifdef NO_0D_WHILE_2D
    static bool before_2d_status[MAX_NUMBER_OF_BUTTONS];
    static bool while_2d_status[MAX_NUMBER_OF_BUTTONS];
#endif

    if (do_once) {
        memset(current_status, 0, sizeof(current_status));
#ifdef NO_0D_WHILE_2D
        memset(before_2d_status, 0, sizeof(before_2d_status));
        memset(while_2d_status, 0, sizeof(while_2d_status));
#endif
        do_once = 0;
    }

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            data_addr,
            f1a->button_data_buffer,
            f1a->button_bitmask_size);
    if (retval < 0) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Failed to read button data registers\n",
                __func__);
        return;
    }

    data = f1a->button_data_buffer;

    for (button = 0; button < f1a->valid_button_count; button++) {
        index = button / 8;
        shift = button % 8;
        status = ((data[index] >> shift) & MASK_1BIT);

        if (current_status[button] == status)
            continue;
        else
            current_status[button] = status;

        dev_dbg(&rmi4_data->i2c_client->dev,
                "%s: Button %d (code %d) ->%d\n",
                __func__, button,
                f1a->button_map[button],
                status);
#ifdef NO_0D_WHILE_2D
        if (rmi4_data->fingers_on_2d == false) {
            if (status == 1) {
                before_2d_status[button] = 1;
            } else {
                if (while_2d_status[button] == 1) {
                    while_2d_status[button] = 0;
                    continue;
                } else {
                    before_2d_status[button] = 0;
                }
            }
            touch_count++;
            input_report_key(rmi4_data->input_dev,
                    f1a->button_map[button],
                    status);
        } else {
            if (before_2d_status[button] == 1) {
                before_2d_status[button] = 0;
                touch_count++;
                input_report_key(rmi4_data->input_dev,
                        f1a->button_map[button],
                        status);
            } else {
                if (status == 1)
                    while_2d_status[button] = 1;
                else
                    while_2d_status[button] = 0;
            }
        }
#else
        touch_count++;
        input_report_key(rmi4_data->input_dev,
                f1a->button_map[button],
                status);
#endif
    }

    if (touch_count)
        input_sync(rmi4_data->input_dev);

    return;
}

/**
 * synaptics_rmi4_report_touch()
 *
 * Called by synaptics_rmi4_sensor_report().
 *
 * This function calls the appropriate finger data reporting function
 * based on the function handler it receives and returns the number of
 * fingers detected.
 */
static void synaptics_rmi4_report_touch(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler)
{
    unsigned char touch_count_2d;

    //print_ts(TS_DEBUG, KERN_ERR"%s: Function %02x reporting\n",__func__, fhandler->fn_number);

    switch (fhandler->fn_number) {
        case SYNAPTICS_RMI4_F11:
            touch_count_2d = synaptics_rmi4_f11_abs_report(rmi4_data,fhandler);
            if (touch_count_2d)
                rmi4_data->fingers_on_2d = true;
            else
                rmi4_data->fingers_on_2d = false;
            break;
        case SYNAPTICS_RMI4_F12:
            touch_count_2d = synaptics_rmi4_f12_abs_report(rmi4_data,fhandler);
            if (touch_count_2d)
                rmi4_data->fingers_on_2d = true;
            else
                rmi4_data->fingers_on_2d = false;
            break;
        case SYNAPTICS_RMI4_F1A:
            synaptics_rmi4_f1a_report(rmi4_data, fhandler);
            break;
        default:
            break;
    }

    return;
}

static void tpd_down(struct synaptics_rmi4_data *rmi4_data,int raw_x, int raw_y, int x, int y, int p) 
{
    if(rmi4_data && rmi4_data->input_dev) 
    {
        input_mt_slot(rmi4_data->input_dev, 0);
        input_mt_report_slot_state(rmi4_data->input_dev, MT_TOOL_FINGER, 1);

        input_report_key(rmi4_data->input_dev, BTN_TOUCH, 1);
        input_report_key(rmi4_data->input_dev, BTN_TOOL_FINGER, 1);

        input_report_abs(rmi4_data->input_dev, ABS_MT_TOUCH_MAJOR, p);
        input_report_abs(rmi4_data->input_dev, ABS_MT_WIDTH_MAJOR, (raw_x+raw_y)/2);
        input_report_abs(rmi4_data->input_dev, ABS_MT_POSITION_X, x);
        input_report_abs(rmi4_data->input_dev, ABS_MT_POSITION_Y, y);

        //input_mt_sync(rmi4_data->input_dev);
    }  
}

static void tpd_up(struct synaptics_rmi4_data *rmi4_data,int raw_x, int raw_y, int x, int y, int p) 
{   
    if(rmi4_data && rmi4_data->input_dev) 
    {
        input_mt_slot(rmi4_data->input_dev, 0);
        input_mt_report_slot_state(rmi4_data->input_dev, MT_TOOL_FINGER, 0);

        input_report_key(rmi4_data->input_dev,	BTN_TOUCH, 0);
        input_report_key(rmi4_data->input_dev,	BTN_TOOL_FINGER, 0);

        //input_mt_sync(rmi4_data->input_dev);
    }  
}

static void int_key_report(struct synaptics_rmi4_data *rmi4_data)
{
    int ret= 0;
    int F1A_0D_DATA00=0x0;
    i2c_smbus_write_byte_data(rmi4_data->i2c_client, 0xff, 0x2);
    ret = i2c_smbus_read_byte_data(rmi4_data->i2c_client, F1A_0D_DATA00);

    if((ret&0x07)!=0)
    {
        print_ts(TS_INFO,"F1A_0D_DATA00 is 0x%x\n",ret);

        if(ret&0x01)//menu
        {          
            tpd_down(rmi4_data,40, 20, rmi4_data->sensor_max_x/6, VK_VENTE_Y,44);
        }

        if(ret&0x02)//home
        {	              
            tpd_down(rmi4_data,40, 20, rmi4_data->sensor_max_x/2, VK_VENTE_Y,44);
        }

        if(ret&0x04)//reback
        {				
            tpd_down(rmi4_data,40, 20, rmi4_data->sensor_max_x*5/6, VK_VENTE_Y,44);
        }
    }
    else
    {
        print_ts(TS_INFO,"virtual key_up\n");
        tpd_up(rmi4_data,0, 0, 0, 0,0);
    }

    input_sync(rmi4_data->input_dev);
    i2c_smbus_write_byte_data(rmi4_data->i2c_client, 0xff, 0x0);
}

static void  synaptics_rmi4_esd_process(struct synaptics_rmi4_data *rmi4_data)
{
    print_ts(TS_ERROR,"%s: esd process begin! \n", __func__);
    
    synaptics_rmi4_reset_device(rmi4_data, rmi4_data->f01_cmd_base_addr);
    synaptics_rmi4_reinit_device(rmi4_data);
    
    print_ts(TS_ERROR,"%s: esd process end! \n", __func__);
}

/**
 * synaptics_rmi4_sensor_report()
 *
 * Called by synaptics_rmi4_irq().
 *
 * This function determines the interrupt source(s) from the sensor
 * and calls synaptics_rmi4_report_touch() with the appropriate
 * function handler for each function with valid data inputs.
 */
static void synaptics_rmi4_sensor_report(struct synaptics_rmi4_data *rmi4_data)
{
    int retval;
    unsigned char data[MAX_INTR_REGISTERS + 1];
    unsigned char *intr = &data[1];
    struct synaptics_rmi4_f01_device_status status;
    struct synaptics_rmi4_fn *fhandler;
    struct synaptics_rmi4_exp_fn *exp_fhandler;
    struct synaptics_rmi4_device_info *rmi;
    //print_ts(TS_DEBUG, KERN_ERR"%s report data .... \n", __func__);
    rmi = &(rmi4_data->rmi4_mod_info);

    down(&work_sem);
    /*
     * Get interrupt status information from F01 Data1 register to
     * determine the source(s) that are flagging the interrupt.
     */
    retval = synaptics_rmi4_i2c_read(rmi4_data, rmi4_data->f01_data_base_addr, data, rmi4_data->num_of_intr_regs + 1);
    if (retval < 0) 
    {
        print_ts(TS_DEBUG, KERN_ERR"'%s line:%d i2c error... \n ", __func__, __LINE__);
        synaptics_rmi4_esd_process(rmi4_data);    // zhangqiang add for tp esd process
        
        dev_err(&rmi4_data->i2c_client->dev, "%s: Failed to read interrupt status\n", __func__);
        up(&work_sem);
        return;
    }

    status.data[0] = data[0];
    if (status.unconfigured && !status.flash_prog) {
        print_ts(TS_INFO,"%s: spontaneous reset detected\n", __func__);
        print_ts(TS_DEBUG, KERN_ERR"%s: data[0] = 0x%x data[1] = 0x%x\n",__func__, data[0],data[1]);
        retval = synaptics_rmi4_reinit_device(rmi4_data);
        if (retval < 0) {
            dev_err(&rmi4_data->i2c_client->dev, "%s: Failed to reinit device\n", __func__);
        }
        up(&work_sem);
        return;
    }


    if(data[1] & 0x04)    
    {    
        /*
         * Traverse the function handler list and service the source(s)
         * of the interrupt accordingly.
         */
        if (!list_empty(&rmi->support_fn_list)) {
            list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
        		if (list_empty(&rmi->support_fn_list))
                {                                                                             
					printk(KERN_ERR"[TP] %s:%d support_fn_list is empty, tp happen erro,will reset tp\n",__func__,__LINE__);
					up(&work_sem);
					return;
				}                
                if (fhandler->num_of_data_sources) {
                    if (fhandler->intr_mask &  intr[fhandler->intr_reg_num]) 
					{
                        synaptics_rmi4_report_touch(rmi4_data, fhandler);
                    }
                }
            }
        }
    }
    if(data[1] &0x10)
    {
        int_key_report(rmi4_data);
    }

    mutex_lock(&exp_data.mutex);
    if (!list_empty(&exp_data.list)) {
        list_for_each_entry(exp_fhandler, &exp_data.list, link) {
            if(exp_fhandler == NULL)
            {                                                                             
                printk("[TP] %s:%d  fhandler is NULL, continue loop\n",__func__,__LINE__);
                continue;
            }            
            if (exp_fhandler->inserted &&
                    (exp_fhandler->func_attn != NULL))
                exp_fhandler->func_attn(rmi4_data, intr[0]);
        }
    }
    mutex_unlock(&exp_data.mutex);

    up(&work_sem);
    return;
}

/**
 * synaptics_rmi4_irq()
 *
 * Called by the kernel when an interrupt occurs (when the sensor
 * asserts the attention irq).
 *
 * This function is the ISR thread and handles the acquisition
 * and the reporting of finger data when the presence of fingers
 * is detected.
 */
static irqreturn_t synaptics_rmi4_irq(int irq, void *data)
{
    struct synaptics_rmi4_data *rmi4_data = data;

    if (!rmi4_data->touch_stopped)
        synaptics_rmi4_sensor_report(rmi4_data);
    return IRQ_HANDLED;
}

/**
 * synaptics_rmi4_irq_enable()
 *
 * Called by synaptics_rmi4_probe() and the power management functions
 * in this driver and also exported to other expansion Function modules
 * such as rmi_dev.
 *
 * This function handles the enabling and disabling of the attention
 * irq including the setting up of the ISR thread.
 */
static int synaptics_rmi4_irq_enable(struct synaptics_rmi4_data *rmi4_data,
        bool enable)
{
    int retval = 0;
    unsigned char intr_status[MAX_INTR_REGISTERS];
    const struct synaptics_dsx_platform_data *platform_data = rmi4_data->i2c_client->dev.platform_data;

    if (enable) {
        if (rmi4_data->irq_enabled)
            return retval;

        /* Clear interrupts first */
        retval = synaptics_rmi4_i2c_read(rmi4_data, rmi4_data->f01_data_base_addr + 1, intr_status, rmi4_data->num_of_intr_regs);
        if (retval < 0)
            return retval;


        retval = request_threaded_irq(rmi4_data->irq, NULL, synaptics_rmi4_irq, platform_data->irq_flags, DRIVER_NAME, rmi4_data);
        if (retval < 0) {
            dev_err(&rmi4_data->i2c_client->dev, "%s: Failed to create irq(:%d) thread\n", __func__, rmi4_data->irq);
            return retval;
        }

        rmi4_data->irq_enabled = true;
    } else {
        if (rmi4_data->irq_enabled) {
            disable_irq(rmi4_data->irq);
            free_irq(rmi4_data->irq, rmi4_data);
            rmi4_data->irq_enabled = false;
        }
    }

    return retval;
}

/**
 * synaptics_rmi4_f11_init()
 *
 * Called by synaptics_rmi4_query_device().
 *
 * This funtion parses information from the Function 11 registers
 * and determines the number of fingers supported, x and y data ranges,
 * offset to the associated interrupt status register, interrupt bit
 * mask, and gathers finger data acquisition capabilities from the query
 * registers.
 */
static int synaptics_rmi4_f11_init(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler,
        struct synaptics_rmi4_fn_desc *fd,
        unsigned int intr_count)
{
    int retval;
    unsigned char ii;
    unsigned char intr_offset;
    unsigned char abs_data_size;
    unsigned char abs_data_blk_size;
    unsigned char query[F11_STD_QUERY_LEN];
    unsigned char control[F11_STD_CTRL_LEN];

    fhandler->fn_number = fd->fn_number;
    fhandler->num_of_data_sources = fd->intr_src_count;

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            fhandler->full_addr.query_base,
            query,
            sizeof(query));
    if (retval < 0)
        return retval;

    /* Maximum number of fingers supported */
    if ((query[1] & MASK_3BIT) <= 4)
        fhandler->num_of_data_points = (query[1] & MASK_3BIT) + 1;
    else if ((query[1] & MASK_3BIT) == 5)
        fhandler->num_of_data_points = 10;

    rmi4_data->num_of_fingers = fhandler->num_of_data_points;

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            fhandler->full_addr.ctrl_base,
            control,
            sizeof(control));
    if (retval < 0)
        return retval;

    /* Maximum x and y */
    rmi4_data->sensor_max_x = ((control[6] & MASK_8BIT) << 0) |
        ((control[7] & MASK_4BIT) << 8);
    rmi4_data->sensor_max_y = ((control[8] & MASK_8BIT) << 0) |
        ((control[9] & MASK_4BIT) << 8);
    dev_dbg(&rmi4_data->i2c_client->dev,
            "%s: Function %02x max x = %d max y = %d\n",
            __func__, fhandler->fn_number,
            rmi4_data->sensor_max_x,
            rmi4_data->sensor_max_y);

    rmi4_data->max_touch_width = MAX_F11_TOUCH_WIDTH;

    fhandler->intr_reg_num = (intr_count + 7) / 8;
    if (fhandler->intr_reg_num != 0)
        fhandler->intr_reg_num -= 1;

    /* Set an enable bit for each data source */
    intr_offset = intr_count % 8;
    fhandler->intr_mask = 0;
    for (ii = intr_offset;
            ii < ((fd->intr_src_count & MASK_3BIT) +
                intr_offset);
            ii++)
        fhandler->intr_mask |= 1 << ii;

    abs_data_size = query[5] & MASK_2BIT;
    abs_data_blk_size = 3 + (2 * (abs_data_size == 0 ? 1 : 0));
    fhandler->size_of_data_register_block = abs_data_blk_size;
    fhandler->data = NULL;
    fhandler->extra = NULL;

    return retval;
}

static int synaptics_rmi4_f12_set_enables(struct synaptics_rmi4_data *rmi4_data,
        unsigned short ctrl28)
{
    int retval;
    static unsigned short ctrl_28_address;

    if (ctrl28)
        ctrl_28_address = ctrl28;

    retval = synaptics_rmi4_i2c_write(rmi4_data,
            ctrl_28_address,
            &rmi4_data->report_enable,
            sizeof(rmi4_data->report_enable));
    if (retval < 0)
        return retval;

    return retval;
}

/**
 * synaptics_rmi4_f12_init()
 *
 * Called by synaptics_rmi4_query_device().
 *
 * This funtion parses information from the Function 12 registers and
 * determines the number of fingers supported, offset to the data1
 * register, x and y data ranges, offset to the associated interrupt
 * status register, interrupt bit mask, and allocates memory resources
 * for finger data acquisition.
 */
static int synaptics_rmi4_f12_init(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler,
        struct synaptics_rmi4_fn_desc *fd,
        unsigned int intr_count)
{
    int retval;
    unsigned char ii;
    unsigned char intr_offset;
    unsigned char size_of_2d_data;
    unsigned char size_of_query8;
    unsigned char ctrl_8_offset;
    unsigned char ctrl_23_offset;
    unsigned char ctrl_28_offset;
    unsigned char num_of_fingers;
    struct synaptics_rmi4_f12_extra_data *extra_data;
    struct synaptics_rmi4_f12_query_5 query_5;
    struct synaptics_rmi4_f12_query_8 query_8;
    struct synaptics_rmi4_f12_ctrl_8 ctrl_8;
    struct synaptics_rmi4_f12_ctrl_23 ctrl_23;

    fhandler->fn_number = fd->fn_number;
    fhandler->num_of_data_sources = fd->intr_src_count;
    fhandler->extra = kmalloc(sizeof(*extra_data), GFP_KERNEL);
    extra_data = (struct synaptics_rmi4_f12_extra_data *)fhandler->extra;
    size_of_2d_data = sizeof(struct synaptics_rmi4_f12_finger_data);

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            fhandler->full_addr.query_base + 5,
            query_5.data,
            sizeof(query_5.data));
    if (retval < 0)
        return retval;

    ctrl_8_offset = query_5.ctrl0_is_present +
        query_5.ctrl1_is_present +
        query_5.ctrl2_is_present +
        query_5.ctrl3_is_present +
        query_5.ctrl4_is_present +
        query_5.ctrl5_is_present +
        query_5.ctrl6_is_present +
        query_5.ctrl7_is_present;

    ctrl_23_offset = ctrl_8_offset +
        query_5.ctrl8_is_present +
        query_5.ctrl9_is_present +
        query_5.ctrl10_is_present +
        query_5.ctrl11_is_present +
        query_5.ctrl12_is_present +
        query_5.ctrl13_is_present +
        query_5.ctrl14_is_present +
        query_5.ctrl15_is_present +
        query_5.ctrl16_is_present +
        query_5.ctrl17_is_present +
        query_5.ctrl18_is_present +
        query_5.ctrl19_is_present +
        query_5.ctrl20_is_present +
        query_5.ctrl21_is_present +
        query_5.ctrl22_is_present;

    ctrl_28_offset = ctrl_23_offset +
        query_5.ctrl23_is_present +
        query_5.ctrl24_is_present +
        query_5.ctrl25_is_present +
        query_5.ctrl26_is_present +
        query_5.ctrl27_is_present;

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            fhandler->full_addr.ctrl_base + ctrl_23_offset,
            ctrl_23.data,
            sizeof(ctrl_23.data));
    if (retval < 0)
        return retval;

    /* Maximum number of fingers supported */
    fhandler->num_of_data_points = min(ctrl_23.max_reported_objects,
            (unsigned char)F12_FINGERS_TO_SUPPORT);

    num_of_fingers = fhandler->num_of_data_points;
    rmi4_data->num_of_fingers = num_of_fingers;

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            fhandler->full_addr.query_base + 7,
            &size_of_query8,
            sizeof(size_of_query8));
    if (retval < 0)
        return retval;

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            fhandler->full_addr.query_base + 8,
            query_8.data,
            size_of_query8);
    if (retval < 0)
        return retval;

    /* Determine the presence of the Data0 register */
    extra_data->data1_offset = query_8.data0_is_present;

    if ((size_of_query8 >= 3) && (query_8.data15_is_present)) {
        extra_data->data15_offset = query_8.data0_is_present +
            query_8.data1_is_present +
            query_8.data2_is_present +
            query_8.data3_is_present +
            query_8.data4_is_present +
            query_8.data5_is_present +
            query_8.data6_is_present +
            query_8.data7_is_present +
            query_8.data8_is_present +
            query_8.data9_is_present +
            query_8.data10_is_present +
            query_8.data11_is_present +
            query_8.data12_is_present +
            query_8.data13_is_present +
            query_8.data14_is_present;
        extra_data->data15_size = (num_of_fingers + 7) / 8;
    } else {
        extra_data->data15_size = 0;
    }

    rmi4_data->report_enable = RPT_DEFAULT;
#ifdef REPORT_2D_Z
		rmi4_data->report_enable |= RPT_Z;
#endif
#ifdef REPORT_2D_W
    rmi4_data->report_enable |= (RPT_WX | RPT_WY);
#endif

    retval = synaptics_rmi4_f12_set_enables(rmi4_data,
            fhandler->full_addr.ctrl_base + ctrl_28_offset);
    if (retval < 0)
        return retval;

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            fhandler->full_addr.ctrl_base + ctrl_8_offset,
            ctrl_8.data,
            sizeof(ctrl_8.data));
    if (retval < 0)
        return retval;

    /* Maximum x and y */
    rmi4_data->sensor_max_x =
        ((unsigned short)ctrl_8.max_x_coord_lsb << 0) |
        ((unsigned short)ctrl_8.max_x_coord_msb << 8);
    rmi4_data->sensor_max_y =
        ((unsigned short)ctrl_8.max_y_coord_lsb << 0) |
        ((unsigned short)ctrl_8.max_y_coord_msb << 8);
    dev_dbg(&rmi4_data->i2c_client->dev,
            "%s: Function %02x max x = %d max y = %d\n",
            __func__, fhandler->fn_number,
            rmi4_data->sensor_max_x,
            rmi4_data->sensor_max_y);

    rmi4_data->num_of_rx = ctrl_8.num_of_rx;
    rmi4_data->num_of_tx = ctrl_8.num_of_tx;
    rmi4_data->max_touch_width = max(rmi4_data->num_of_rx,
            rmi4_data->num_of_tx);

    fhandler->intr_reg_num = (intr_count + 7) / 8;
    if (fhandler->intr_reg_num != 0)
        fhandler->intr_reg_num -= 1;

    /* Set an enable bit for each data source */
    intr_offset = intr_count % 8;
    fhandler->intr_mask = 0;
    for (ii = intr_offset;
            ii < ((fd->intr_src_count & MASK_3BIT) +
                intr_offset);
            ii++)
        fhandler->intr_mask |= 1 << ii;

    /* Allocate memory for finger data storage space */
    fhandler->data_size = num_of_fingers * size_of_2d_data;
    fhandler->data = kmalloc(fhandler->data_size, GFP_KERNEL);

    return retval;
}

static int synaptics_rmi4_f1a_alloc_mem(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler)
{
    int retval;
    struct synaptics_rmi4_f1a_handle *f1a;

    f1a = kzalloc(sizeof(*f1a), GFP_KERNEL);
    if (!f1a) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Failed to alloc mem for function handle\n",
                __func__);
        return -ENOMEM;
    }

    fhandler->data = (void *)f1a;
    fhandler->extra = NULL;

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            fhandler->full_addr.query_base,
            f1a->button_query.data,
            sizeof(f1a->button_query.data));
    if (retval < 0) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Failed to read query registers\n",
                __func__);
        return retval;
    }

    f1a->max_count = f1a->button_query.max_button_count + 1;

    f1a->button_control.txrx_map = kzalloc(f1a->max_count * 2, GFP_KERNEL);
    if (!f1a->button_control.txrx_map) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Failed to alloc mem for tx rx mapping\n",
                __func__);
        return -ENOMEM;
    }

    f1a->button_bitmask_size = (f1a->max_count + 7) / 8;

    f1a->button_data_buffer = kcalloc(f1a->button_bitmask_size,
            sizeof(*(f1a->button_data_buffer)), GFP_KERNEL);
    if (!f1a->button_data_buffer) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Failed to alloc mem for data buffer\n",
                __func__);
        return -ENOMEM;
    }

    f1a->button_map = kcalloc(f1a->max_count,
            sizeof(*(f1a->button_map)), GFP_KERNEL);
    if (!f1a->button_map) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Failed to alloc mem for button map\n",
                __func__);
        return -ENOMEM;
    }

    return 0;
}

static int synaptics_rmi4_f1a_button_map(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler)
{
    int retval;
    unsigned char ii;
    unsigned char mapping_offset = 0;
    struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;
    const struct synaptics_dsx_platform_data *pdata = rmi4_data->board;

    mapping_offset = f1a->button_query.has_general_control +
        f1a->button_query.has_interrupt_enable +
        f1a->button_query.has_multibutton_select;

    if (f1a->button_query.has_tx_rx_map) {
        retval = synaptics_rmi4_i2c_read(rmi4_data,
                fhandler->full_addr.ctrl_base + mapping_offset,
                f1a->button_control.txrx_map,
                sizeof(f1a->button_control.txrx_map));
        if (retval < 0) {
            dev_err(&rmi4_data->i2c_client->dev,
                    "%s: Failed to read tx rx mapping\n",
                    __func__);
            return retval;
        }

        rmi4_data->button_txrx_mapping = f1a->button_control.txrx_map;
    }

    if (!pdata->cap_button_map) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: cap_button_map is NULL in board file\n",
                __func__);
        return -ENODEV;
    } else if (!pdata->cap_button_map->map) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Button map is missing in board file\n",
                __func__);
        return -ENODEV;
    } else {
        if (pdata->cap_button_map->nbuttons != f1a->max_count) {
            f1a->valid_button_count = min(f1a->max_count,
                    pdata->cap_button_map->nbuttons);
        } else {
            f1a->valid_button_count = f1a->max_count;
        }

        for (ii = 0; ii < f1a->valid_button_count; ii++)
            f1a->button_map[ii] = pdata->cap_button_map->map[ii];
    }

    return 0;
}

static void synaptics_rmi4_f1a_kfree(struct synaptics_rmi4_fn *fhandler)
{
    struct synaptics_rmi4_f1a_handle *f1a = fhandler->data;

    if (f1a) {
        kfree(f1a->button_control.txrx_map);
        kfree(f1a->button_data_buffer);
        kfree(f1a->button_map);
        kfree(f1a);
        fhandler->data = NULL;
    }

    return;
}

static int synaptics_rmi4_f1a_init(struct synaptics_rmi4_data *rmi4_data,
        struct synaptics_rmi4_fn *fhandler,
        struct synaptics_rmi4_fn_desc *fd,
        unsigned int intr_count)
{
    int retval;
    unsigned char ii;
    unsigned short intr_offset;

    fhandler->fn_number = fd->fn_number;
    fhandler->num_of_data_sources = fd->intr_src_count;

    fhandler->intr_reg_num = (intr_count + 7) / 8;
    if (fhandler->intr_reg_num != 0)
        fhandler->intr_reg_num -= 1;

    /* Set an enable bit for each data source */
    intr_offset = intr_count % 8;
    fhandler->intr_mask = 0;
    for (ii = intr_offset;
            ii < ((fd->intr_src_count & MASK_3BIT) +
                intr_offset);
            ii++)
        fhandler->intr_mask |= 1 << ii;

    retval = synaptics_rmi4_f1a_alloc_mem(rmi4_data, fhandler);
    if (retval < 0)
        goto error_exit;

    retval = synaptics_rmi4_f1a_button_map(rmi4_data, fhandler);
    if (retval < 0)
        goto error_exit;

    rmi4_data->button_0d_enabled = 1;

    return 0;

error_exit:
    synaptics_rmi4_f1a_kfree(fhandler);

    return retval;
}

static void synaptics_rmi4_empty_fn_list(struct synaptics_rmi4_data *rmi4_data)
{
    struct synaptics_rmi4_fn *fhandler;
    struct synaptics_rmi4_fn *fhandler_temp;
    struct synaptics_rmi4_device_info *rmi;

    rmi = &(rmi4_data->rmi4_mod_info);

    if (!list_empty(&rmi->support_fn_list)) {
        list_for_each_entry_safe(fhandler,fhandler_temp,&rmi->support_fn_list,link) 
        {
            if(fhandler == NULL)
            {                                                                             
                printk("[TP] %s:%d  fhandler is NULL, continue loop\n",__func__,__LINE__);
                continue;
            }
            if (fhandler->fn_number == SYNAPTICS_RMI4_F1A) {
                synaptics_rmi4_f1a_kfree(fhandler);
            } else {
                kfree(fhandler->extra);
                kfree(fhandler->data);
            }
            list_del(&fhandler->link);
            kfree(fhandler);
        }
    }
    INIT_LIST_HEAD(&rmi->support_fn_list);

    return;
}

static int synaptics_rmi4_check_status(struct synaptics_rmi4_data *rmi4_data,
        bool *was_in_bl_mode)
{
    int retval;
    int timeout = CHECK_STATUS_TIMEOUT_MS;
    unsigned char command = 0x01;
    unsigned char intr_status;
    struct synaptics_rmi4_f01_device_status status;

    /* Do a device reset first */
    retval = synaptics_rmi4_i2c_write(rmi4_data,
            rmi4_data->f01_cmd_base_addr,
            &command,
            sizeof(command));
    if (retval < 0)
        return retval;

    msleep(rmi4_data->board->reset_delay_ms);

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f01_data_base_addr,
            status.data,
            sizeof(status.data));
    if (retval < 0)
        return retval;

    while (status.status_code == STATUS_CRC_IN_PROGRESS) {
        if (timeout > 0)
            msleep(20);
        else
            return -1;

        retval = synaptics_rmi4_i2c_read(rmi4_data,
                rmi4_data->f01_data_base_addr,
                status.data,
                sizeof(status.data));
        if (retval < 0)
            return retval;

        timeout -= 20;
    }

    if (timeout != CHECK_STATUS_TIMEOUT_MS)
        *was_in_bl_mode = true;

    if (status.flash_prog == 1) {
        rmi4_data->flash_prog_mode = true;
        pr_notice("%s: In flash prog mode, status = 0x%02x\n",
                __func__,
                status.status_code);
    } else {
        rmi4_data->flash_prog_mode = false;
    }

    retval = synaptics_rmi4_i2c_read(rmi4_data,
            rmi4_data->f01_data_base_addr + 1,
            &intr_status,
            sizeof(intr_status));
    if (retval < 0) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Failed to read interrupt status\n",
                __func__);
        return retval;
    }

    return 0;
}

static void synaptics_rmi4_set_configured(struct synaptics_rmi4_data *rmi4_data)
{
    int retval;
    unsigned char device_ctrl;

    retval = synaptics_rmi4_i2c_read(rmi4_data, rmi4_data->f01_ctrl_base_addr, &device_ctrl, sizeof(device_ctrl));
    if (retval < 0) 
    {
        dev_err(&(rmi4_data->input_dev->dev), "%s: Failed to set configured\n", __func__);
        return;
    }

    rmi4_data->no_sleep_setting = device_ctrl & NO_SLEEP_ON;
    device_ctrl |= CONFIGURED;

    retval = synaptics_rmi4_i2c_write(rmi4_data, rmi4_data->f01_ctrl_base_addr, &device_ctrl, sizeof(device_ctrl));
    if (retval < 0)
    {
        dev_err(&(rmi4_data->input_dev->dev), "%s: Failed to set configured\n", __func__);
    }

    return;
}

static int synaptics_rmi4_alloc_fh(struct synaptics_rmi4_fn **fhandler,
        struct synaptics_rmi4_fn_desc *rmi_fd, int page_number)
{
    *fhandler = kmalloc(sizeof(**fhandler), GFP_KERNEL);
    if (!(*fhandler))
        return -ENOMEM;

    (*fhandler)->full_addr.data_base = (rmi_fd->data_base_addr | (page_number << 8));
    (*fhandler)->full_addr.ctrl_base = (rmi_fd->ctrl_base_addr | (page_number << 8));
    (*fhandler)->full_addr.cmd_base = (rmi_fd->cmd_base_addr | (page_number << 8));
    (*fhandler)->full_addr.query_base = (rmi_fd->query_base_addr | (page_number << 8));

    return 0;
}

/**
 * synaptics_rmi4_query_device()
 *
 * Called by synaptics_rmi4_probe().
 *
 * This funtion scans the page description table, records the offsets
 * to the register types of Function $01, sets up the function handlers
 * for Function $11 and Function $12, determines the number of interrupt
 * sources from the sensor, adds valid Functions with data inputs to the
 * Function linked list, parses information from the query registers of
 * Function $01, and enables the interrupt sources from the valid Functions
 * with data inputs.
 */
static int synaptics_rmi4_query_device(struct synaptics_rmi4_data *rmi4_data)
{
    int retval;
    unsigned char ii;
    unsigned char page_number;
    unsigned char intr_count;
    unsigned char f01_query[F01_STD_QUERY_LEN];
    unsigned short pdt_entry_addr;
    unsigned short intr_addr;
    bool was_in_bl_mode;
    struct synaptics_rmi4_fn_desc rmi_fd;
    struct synaptics_rmi4_fn *fhandler;
    struct synaptics_rmi4_device_info *rmi;

    rmi = &(rmi4_data->rmi4_mod_info);

rescan_pdt:
    was_in_bl_mode = false;
    intr_count = 0;
    INIT_LIST_HEAD(&rmi->support_fn_list);

    /* Scan the page description tables of the pages to service */
    for (page_number = 0; page_number < PAGES_TO_SERVICE; page_number++) {
        for (pdt_entry_addr = PDT_START; pdt_entry_addr > PDT_END; pdt_entry_addr -= PDT_ENTRY_SIZE) {
            pdt_entry_addr |= (page_number << 8);

            retval = synaptics_rmi4_i2c_read(rmi4_data, pdt_entry_addr, (unsigned char *)&rmi_fd, sizeof(rmi_fd));
            if (retval < 0)
                return retval;

            fhandler = NULL;

            if (rmi_fd.fn_number == 0) {
                dev_dbg(&rmi4_data->i2c_client->dev, "%s: Reached end of PDT\n", __func__);
                break;
            }

            print_ts(TS_DEBUG, "%s: F%02x found (page %d)\n", __func__, rmi_fd.fn_number, page_number);

            switch (rmi_fd.fn_number) {
                case SYNAPTICS_RMI4_F01:
                    rmi4_data->f01_query_base_addr = rmi_fd.query_base_addr;
                    rmi4_data->f01_ctrl_base_addr = rmi_fd.ctrl_base_addr;
                    rmi4_data->f01_data_base_addr = rmi_fd.data_base_addr;
                    rmi4_data->f01_cmd_base_addr = rmi_fd.cmd_base_addr;

                    print_ts(TS_DEBUG, "Tp f01_query_base_addr = 0x%x!\n", rmi4_data->f01_query_base_addr);  // N3 : 0x27
                    print_ts(TS_DEBUG, "Tp f01_ctrl_base_addr = 0x%x!\n", rmi4_data->f01_ctrl_base_addr);    // N3 : 0x0e
                    print_ts(TS_DEBUG, "Tp f01_data_base_addr = 0x%x!\n", rmi4_data->f01_data_base_addr);    // N3 : 0x04
                    print_ts(TS_DEBUG, "Tp f01_cmd_base_addr = 0x%x!\n", rmi4_data->f01_cmd_base_addr);      // N3 : 0x22

                    retval = synaptics_rmi4_check_status(rmi4_data, &was_in_bl_mode);
                    if (retval < 0) {
                        dev_err(&rmi4_data->i2c_client->dev, "%s: Failed to check status\n", __func__);
                        return retval;
                    }

                    if (was_in_bl_mode)
                        goto rescan_pdt;

                    if (rmi4_data->flash_prog_mode)
                        goto flash_prog_mode;

                    break;
                case SYNAPTICS_RMI4_F11:
                    if (rmi_fd.intr_src_count == 0)
                        break;

                    retval = synaptics_rmi4_alloc_fh(&fhandler, &rmi_fd, page_number);
                    if (retval < 0) {
                        dev_err(&rmi4_data->i2c_client->dev, "%s: Failed to alloc for F%d\n", __func__, rmi_fd.fn_number);
                        return retval;
                    }

                    retval = synaptics_rmi4_f11_init(rmi4_data, fhandler, &rmi_fd, intr_count);
                    if (retval < 0)
                        return retval;
                    break;
                case SYNAPTICS_RMI4_F12:
                    if (rmi_fd.intr_src_count == 0)
                        break;

                    retval = synaptics_rmi4_alloc_fh(&fhandler, &rmi_fd, page_number);
                    if (retval < 0) {
                        dev_err(&rmi4_data->i2c_client->dev, "%s: Failed to alloc for F%d\n", __func__, rmi_fd.fn_number);
                        return retval;
                    }

                    retval = synaptics_rmi4_f12_init(rmi4_data, fhandler, &rmi_fd, intr_count);
                    if (retval < 0)
                        return retval;
                    break;
                case SYNAPTICS_RMI4_F1A:
                    if (rmi_fd.intr_src_count == 0)
                        break;

                    retval = synaptics_rmi4_alloc_fh(&fhandler, &rmi_fd, page_number);
                    if (retval < 0) {
                        dev_err(&rmi4_data->i2c_client->dev, "%s: Failed to alloc for F%d\n", __func__, rmi_fd.fn_number);
                        return retval;
                    }

                    retval = synaptics_rmi4_f1a_init(rmi4_data, fhandler, &rmi_fd, intr_count);
                    if (retval < 0) {
                        return retval;
                    }
                    break;
                case SYNAPTICS_RMI4_F54:
                    rmi4_data->f54_data_base_addr = (rmi_fd.data_base_addr |(page_number << 8));
                    rmi4_data->f54_ctrl_base_addr = (rmi_fd.ctrl_base_addr |(page_number << 8));
                    rmi4_data->f54_cmd_base_addr = (rmi_fd.cmd_base_addr |(page_number << 8));
                    rmi4_data->f54_query_base_addr = (rmi_fd.query_base_addr |(page_number << 8));

                    print_ts(TS_DEBUG, "Tp f54_query_base_addr = 0x%x!\n", rmi4_data->f54_query_base_addr);  // N3 : 0x149
                    print_ts(TS_DEBUG, "Tp f54_ctrl_base_addr = 0x%x!\n", rmi4_data->f54_ctrl_base_addr);    // N3 : 0x10e
                    print_ts(TS_DEBUG, "Tp f54_data_base_addr = 0x%x!\n", rmi4_data->f54_data_base_addr);    // N3 : 0x100
                    print_ts(TS_DEBUG, "Tp f54_cmd_base_addr = 0x%x!\n", rmi4_data->f54_cmd_base_addr);      // N3 : 0x148

                    break ;
            }

            /* Accumulate the interrupt count */
            intr_count += (rmi_fd.intr_src_count & MASK_3BIT);

            if (fhandler && rmi_fd.intr_src_count) {
                list_add_tail(&fhandler->link, &rmi->support_fn_list);
            }
        }
    }

flash_prog_mode:
    rmi4_data->num_of_intr_regs = (intr_count + 7) / 8;
    dev_dbg(&rmi4_data->i2c_client->dev, "%s: Number of interrupt registers = %d\n", __func__, rmi4_data->num_of_intr_regs);

    retval = synaptics_rmi4_i2c_read(rmi4_data, rmi4_data->f01_query_base_addr, f01_query, sizeof(f01_query));
    if (retval < 0)
        return retval;

    /* RMI Version 4.0 currently supported */
    rmi->version_major = 4;
    rmi->version_minor = 0;

    rmi->manufacturer_id = f01_query[0];
    rmi->product_props = f01_query[1];
    rmi->product_info[0] = f01_query[2] & MASK_7BIT;
    rmi->product_info[1] = f01_query[3] & MASK_7BIT;
    rmi->date_code[0] = f01_query[4] & MASK_5BIT;
    rmi->date_code[1] = f01_query[5] & MASK_4BIT;
    rmi->date_code[2] = f01_query[6] & MASK_5BIT;
    rmi->tester_id = ((f01_query[7] & MASK_7BIT) << 8) | (f01_query[8] & MASK_7BIT);
    rmi->serial_number = ((f01_query[9] & MASK_7BIT) << 8) | (f01_query[10] & MASK_7BIT);
    memcpy(rmi->product_id_string, &f01_query[11], 10);

    retval = synaptics_rmi4_i2c_read(rmi4_data, rmi4_data->f01_query_base_addr + F01_BUID_ID_OFFSET, rmi->build_id, sizeof(rmi->build_id));
    if (retval < 0)
        return retval;

    rmi4_data->firmware_id = (unsigned int)rmi->build_id[0] + (unsigned int)rmi->build_id[1] * 0x100 + (unsigned int)rmi->build_id[2] * 0x10000;

    memset(rmi4_data->intr_mask, 0x00, sizeof(rmi4_data->intr_mask));

    /*
     * Map out the interrupt bit masks for the interrupt sources
     * from the registered function handlers.
     */
    if (!list_empty(&rmi->support_fn_list)) {
        list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
            if(fhandler == NULL)
            {                                                                                        
                printk("[TP] %s:%d  fhandler is NULL, continue loop\n",__func__,__LINE__);
                continue;
            }            
            if (fhandler->num_of_data_sources) {
                rmi4_data->intr_mask[fhandler->intr_reg_num] |= fhandler->intr_mask;
            }
        }
    }

    /* Enable the interrupt sources */
    for (ii = 0; ii < rmi4_data->num_of_intr_regs; ii++) {
        if (rmi4_data->intr_mask[ii] != 0x00) {
            dev_dbg(&rmi4_data->i2c_client->dev, "%s: Interrupt enable mask %d = 0x%02x\n", __func__, ii, rmi4_data->intr_mask[ii]);
            intr_addr = rmi4_data->f01_ctrl_base_addr + 1 + ii;
            retval = synaptics_rmi4_i2c_write(rmi4_data, intr_addr, &(rmi4_data->intr_mask[ii]), sizeof(rmi4_data->intr_mask[ii]));
            if (retval < 0)
                return retval;
        }
    }

    synaptics_rmi4_set_configured(rmi4_data);

    return 0;
}

static void synaptics_rmi4_set_params(struct synaptics_rmi4_data *rmi4_data)
{
    unsigned char ii;
    struct synaptics_rmi4_f1a_handle *f1a;
    struct synaptics_rmi4_fn *fhandler;
    struct synaptics_rmi4_device_info *rmi;

    rmi = &(rmi4_data->rmi4_mod_info);

    set_bit(KEY_BACK, rmi4_data->input_dev->keybit);
    set_bit(KEY_MENU, rmi4_data->input_dev->keybit);
    set_bit(KEY_HOMEPAGE, rmi4_data->input_dev->keybit);
    set_bit(KEY_F3, rmi4_data->input_dev->keybit);
    set_bit(KEY_F4, rmi4_data->input_dev->keybit);
    set_bit(KEY_F5, rmi4_data->input_dev->keybit);
    set_bit(KEY_F6, rmi4_data->input_dev->keybit);
    set_bit(KEY_F7, rmi4_data->input_dev->keybit);
    set_bit(KEY_F8, rmi4_data->input_dev->keybit);
    set_bit(KEY_POWER, rmi4_data->input_dev->keybit);

    input_set_abs_params(rmi4_data->input_dev, ABS_MT_POSITION_X, rmi4_data->snap_left, rmi4_data->sensor_max_x-rmi4_data->snap_right, 0, 0);
    input_set_abs_params(rmi4_data->input_dev, ABS_MT_POSITION_Y, rmi4_data->snap_top, rmi4_data->sensor_max_y-rmi4_data->virtual_key_height-rmi4_data->snap_bottom, 0, 0);
    print_ts(TS_DEBUG, KERN_ERR"%s x:%d  y:%d \n", __func__, rmi4_data->sensor_max_x-rmi4_data->snap_right, rmi4_data->sensor_max_y-rmi4_data->virtual_key_height-rmi4_data->snap_bottom);
#ifdef REPORT_2D_W
    input_set_abs_params(rmi4_data->input_dev, ABS_MT_TOUCH_MAJOR, 0, rmi4_data->max_touch_width, 0, 0);
    input_set_abs_params(rmi4_data->input_dev, ABS_MT_TOUCH_MINOR, 0, rmi4_data->max_touch_width, 0, 0);
#endif

#ifdef TYPE_B_PROTOCOL
    input_mt_init_slots(rmi4_data->input_dev, rmi4_data->num_of_fingers, 0);
#endif

    f1a = NULL;
    if (!list_empty(&rmi->support_fn_list)) {
        list_for_each_entry(fhandler, &rmi->support_fn_list, link) {
            if(fhandler == NULL)
            {                                                                                        
                printk("[TP] %s:%d  fhandler is NULL, continue loop\n",__func__,__LINE__);
                continue;
            }            
            if (fhandler->fn_number == SYNAPTICS_RMI4_F1A)
                f1a = fhandler->data;
        }
    }

    if (f1a) {
        for (ii = 0; ii < f1a->valid_button_count; ii++) {
            set_bit(f1a->button_map[ii], rmi4_data->input_dev->keybit);
            input_set_capability(rmi4_data->input_dev, EV_KEY, f1a->button_map[ii]);
        }
    }

    return;
}

static int synaptics_rmi4_set_input_dev(struct synaptics_rmi4_data *rmi4_data)
{
    int retval;
    int temp;

    rmi4_data->input_dev = input_allocate_device();
    if (rmi4_data->input_dev == NULL) {
        dev_err(&rmi4_data->i2c_client->dev, "%s: Failed to allocate input device\n", __func__);
        retval = -ENOMEM;
        goto err_input_device;
    }

    rmi4_data->input_dev->name = DRIVER_NAME;
    rmi4_data->input_dev->phys = INPUT_PHYS_NAME;
    rmi4_data->input_dev->id.product = SYNAPTICS_DSX_DRIVER_PRODUCT;
    rmi4_data->input_dev->id.version = SYNAPTICS_DSX_DRIVER_VERSION;
    rmi4_data->input_dev->id.bustype = BUS_I2C;
    rmi4_data->input_dev->dev.parent = &rmi4_data->i2c_client->dev;
    input_set_drvdata(rmi4_data->input_dev, rmi4_data);

    set_bit(EV_SYN, rmi4_data->input_dev->evbit);
    set_bit(EV_KEY, rmi4_data->input_dev->evbit);
    set_bit(EV_ABS, rmi4_data->input_dev->evbit);
    set_bit(BTN_TOUCH, rmi4_data->input_dev->keybit);
    set_bit(BTN_TOOL_FINGER, rmi4_data->input_dev->keybit);
#ifdef INPUT_PROP_DIRECT
    set_bit(INPUT_PROP_DIRECT, rmi4_data->input_dev->propbit);
#endif

    retval = synaptics_rmi4_query_device(rmi4_data);
    if (retval < 0) {
        dev_err(&rmi4_data->i2c_client->dev, "%s: Failed to query device\n", __func__);
        goto err_query_device;
    }
	/* oem lifeng 2015.3.9 bootloader mode update fw then err report point begin*/
	if(true == rmi4_data->flash_prog_mode){
	 rmi4_data->max_touch_width = 30;
	 rmi4_data->num_of_fingers = 10;
	}
	/* oem lifeng 2015.3.9 bootloader mode update fw then err report point end*/
    if (rmi4_data->board->swap_axes) {
        temp = rmi4_data->sensor_max_x;
        rmi4_data->sensor_max_x = rmi4_data->sensor_max_y;
        rmi4_data->sensor_max_y = temp;
    }

    synaptics_rmi4_set_params(rmi4_data);

    retval = input_register_device(rmi4_data->input_dev);
    if (retval) {
        dev_err(&rmi4_data->i2c_client->dev, "%s: Failed to register input device\n", __func__);
        goto err_register_input;
    }

    return 0;

err_register_input:
err_query_device:
    synaptics_rmi4_empty_fn_list(rmi4_data);
    input_free_device(rmi4_data->input_dev);

err_input_device:
    return retval;
}

static int synaptics_rmi4_free_fingers(struct synaptics_rmi4_data *rmi4_data)
{
    unsigned char ii;

#ifdef TYPE_B_PROTOCOL
    for (ii = 0; ii < rmi4_data->num_of_fingers; ii++) {
        input_mt_slot(rmi4_data->input_dev, ii);
        input_mt_report_slot_state(rmi4_data->input_dev,MT_TOOL_FINGER, 0);
    }
#endif
    input_report_key(rmi4_data->input_dev,BTN_TOUCH, 0);
    input_report_key(rmi4_data->input_dev,BTN_TOOL_FINGER, 0);
#ifndef TYPE_B_PROTOCOL
    input_mt_sync(rmi4_data->input_dev);
#endif
    input_sync(rmi4_data->input_dev);

    rmi4_data->fingers_on_2d = false;

    return 0;
}

static int synaptics_rmi4_reinit_device(struct synaptics_rmi4_data *rmi4_data)
{
    int retval;
    unsigned char tmp_new ;
    struct synaptics_rmi4_device_info *rmi;

    rmi = &(rmi4_data->rmi4_mod_info);

    mutex_lock(&(rmi4_data->rmi4_reset_mutex));

    synaptics_rmi4_free_fingers(rmi4_data);

    synaptics_rmi4_set_configured(rmi4_data);

    if(syna_use_gesture && rmi4_data->gesture) {
        synaptics_set_f12ctrl_data(rmi4_data,rmi4_data->gesture,0);
    }

    tmp_new = 4 ;
    retval = synaptics_rmi4_i2c_write(rmi4_data,F54_CMD_BASE_ADDR,(unsigned char*)&tmp_new,1);

    mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
    return retval;
}

static int synaptics_rmi4_reset_device(struct synaptics_rmi4_data *rmi4_data, unsigned short f01_cmd_base_addr)
{
    int retval;
    int temp;
    int rc;

    rmi4_data->reset_count ++ ;
    if(rmi4_data->reset_count >= 2)
        return 0 ;

    mutex_lock(&(rmi4_data->rmi4_reset_mutex));

    rmi4_data->touch_stopped = true;
   synaptics_rmi4_irq_enable(rmi4_data, false) ;
    //power off device
    regulator_disable(rmi4_data->regulator);    
    msleep(30);
    rmi4_data->current_page = MASK_8BIT;
    rmi4_data->touch_stopped = false;
    //synaptics_rmi4_irq_enable(rmi4_data, false) ;

    synaptics_rmi4_free_fingers(rmi4_data);

    synaptics_rmi4_empty_fn_list(rmi4_data);

    //power on device
    rc = regulator_enable(rmi4_data->regulator);
    msleep(180);

    retval = synaptics_rmi4_query_device(rmi4_data);
    if (retval < 0) {
        dev_err(&rmi4_data->i2c_client->dev, "%s: Failed to query device\n", __func__);
        mutex_unlock(&(rmi4_data->rmi4_reset_mutex));
        rmi4_data->touch_stopped = false;

        return retval;
    }

    if (rmi4_data->board->swap_axes) {
        temp = rmi4_data->sensor_max_x;
        rmi4_data->sensor_max_x = rmi4_data->sensor_max_y;
        rmi4_data->sensor_max_y = temp;
    }

    synaptics_rmi4_set_params(rmi4_data);

    mutex_unlock(&(rmi4_data->rmi4_reset_mutex));

    //reinit device
    msleep(10);
    synaptics_rmi4_irq_enable(rmi4_data, true) ;
    synaptics_rmi4_i2c_read(rmi4_data,rmi4_data->f01_data_base_addr + 1,(unsigned char*)&temp,1);

    return retval;
}

/**
 * synaptics_rmi4_exp_fn_work()
 *
 * Called by the kernel at the scheduled time.
 *
 * This function is a work thread that checks for the insertion and
 * removal of other expansion Function modules such as rmi_dev and calls
 * their initialization and removal callback functions accordingly.
 */
static void synaptics_rmi4_exp_fn_work(struct work_struct *work)
{
    struct synaptics_rmi4_exp_fn *exp_fhandler;
    struct synaptics_rmi4_exp_fn *exp_fhandler_temp;
    struct synaptics_rmi4_data *rmi4_data = exp_data.rmi4_data;

    mutex_lock(&exp_data.mutex);
    if (!list_empty(&exp_data.list)) {
        list_for_each_entry_safe(exp_fhandler,exp_fhandler_temp,&exp_data.list,	link) 
        {
            if(exp_fhandler == NULL)
            {                                                                                        
                printk("[TP] %s:%d  fhandler is NULL, continue loop\n",__func__,__LINE__);
                continue;
            }
            if ((exp_fhandler->func_init != NULL) && (exp_fhandler->inserted == false)) 
            {
                if(exp_fhandler->func_init(rmi4_data) < 0) 
                {
                    //retry init
                    queue_delayed_work(exp_data.workqueue, &exp_data.work, msecs_to_jiffies(500));
                    mutex_unlock(&exp_data.mutex);
                    return ;
                }
                exp_fhandler->inserted = true;
            } 
            else if ((exp_fhandler->func_init == NULL) && (exp_fhandler->inserted == true)) 
            {
                exp_fhandler->func_remove(rmi4_data);
                list_del(&exp_fhandler->link);
                kfree(exp_fhandler);
            }
        }
    }
    mutex_unlock(&exp_data.mutex);

    return;
}

/**
 * synaptics_rmi4_new_function()
 *
 * Called by other expansion Function modules in their module init and
 * module exit functions.
 *
 * This function is used by other expansion Function modules such as
 * rmi_dev to register themselves with the driver by providing their
 * initialization and removal callback function pointers so that they
 * can be inserted or removed dynamically at module init and exit times,
 * respectively.
 */
void synaptics_rmi4_new_function(enum exp_fn fn_type, bool insert,
        int (*func_init)(struct synaptics_rmi4_data *rmi4_data),
        void (*func_remove)(struct synaptics_rmi4_data *rmi4_data),
        void (*func_attn)(struct synaptics_rmi4_data *rmi4_data,
            unsigned char intr_mask))
{
    struct synaptics_rmi4_exp_fn *exp_fhandler;

    if (!exp_data.initialized) {
        mutex_init(&exp_data.mutex);
        INIT_LIST_HEAD(&exp_data.list);
        exp_data.initialized = true;
    }

    mutex_lock(&exp_data.mutex);
    if (insert) {
        exp_fhandler = kzalloc(sizeof(*exp_fhandler), GFP_KERNEL);
        if (!exp_fhandler) {
            pr_err("%s: Failed to alloc mem for expansion function\n", __func__);
            goto exit;
        }
        exp_fhandler->fn_type = fn_type;
        exp_fhandler->func_init = func_init;
        exp_fhandler->func_attn = func_attn;
        exp_fhandler->func_remove = func_remove;
        exp_fhandler->inserted = false;
        list_add_tail(&exp_fhandler->link, &exp_data.list);
    } 
    else if (!list_empty(&exp_data.list)) 
    {
        list_for_each_entry(exp_fhandler, &exp_data.list, link) 
        {
            if(exp_fhandler == NULL)
            {                                                                                        
                printk("[TP] %s:%d  fhandler is NULL, continue loop\n",__func__,__LINE__);
                continue;
            }            
            if (exp_fhandler->fn_type == fn_type) {
                exp_fhandler->func_init = NULL;
                exp_fhandler->func_attn = NULL;
                goto exit;
            }
        }
    }

exit:
    mutex_unlock(&exp_data.mutex);

    if (exp_data.queue_work) {
        queue_delayed_work(exp_data.workqueue, &exp_data.work, msecs_to_jiffies(EXP_FN_WORK_DELAY_MS));
    }

    return;
}
EXPORT_SYMBOL(synaptics_rmi4_new_function);

int synaptics_rmi4_get_vendorid1(int id1, int id2, int id3) {
    if(id1 == 0 && id2 == 0 && id3 == 0) 
        return TP_VENDOR_TPK ;
    else if(id1 == 0 && id2 == 1 && id3 == 0)
        return TP_VENDOR_WINTEK ;

    return 0 ;
}

int synaptics_rmi4_get_vendorid2(int id1, int id2, int id3) {
    if(id1 == 0 && id2 == 0 && id3 == 0) 
        return TP_VENDOR_YOUNGFAST ;
    else if(id1 == 0 && id2 == 1 && id3 == 0)
        return TP_VENDOR_TRULY ;
    else if(id1 == 1 && id2 == 0 && id3 == 0)
        return TP_VENDOR_TPK ;
    else if(id1 == 1 && id2 == 1 && id3 == 0)
        return TP_VENDOR_WINTEK ;

    return 0 ;	
}

int synaptics_rmi4_get_vendorid3(int id1, int id2, int id3) {
    if(id1 == 0 && id2 == 0 ) 
        return TP_VENDOR_TPK ;
    else if(id1 == 1 && id2 == 0 )
        return TP_VENDOR_WINTEK ;
    else if(id1 == 0 && id2 == 1 )
        return TP_VENDOR_TRULY ;
    else if(id1 == 1 && id2 == 1 )
        return TP_VENDOR_TPK_GFF ;

    return 0 ;	
}

//return firmware version and string
extern int synaptics_rmi4_get_firmware_version(int vendor_id);
char* synaptics_rmi4_get_vendorstring(int id) {
    char *pconst = "UNKNOWN";

    switch(id) {
		case TP_VENDOR_WINTEK:
			pconst= "WINTEK" ;
			break ;
		case TP_VENDOR_TPK:
			pconst= "TPK" ;
			break ;
		case TP_VENDOR_TRULY:
			pconst= "TRULY" ;
			break ;
		case TP_VENDOR_YOUNGFAST:
			pconst= "YOUNGFAST" ;
			break ;
		case TP_VENDOR_TPK_GFF:
			pconst= "TPK_GFF" ;
			break ;
		case TP_VENDOR_JDI:
			pconst= "JDI" ;
			break ;
    }

    print_ts(TS_INFO, "%s, pconst = %s, version = 0x%x\n", __func__, pconst, synaptics_rmi4_get_firmware_version(id));

    sprintf(synaptics_vendor_str,"%s(%x)",pconst,synaptics_rmi4_get_firmware_version(id));

    return synaptics_vendor_str;

}

static void synaptics_rmi4_get_vendorid(struct synaptics_rmi4_data *rmi4_data)
{
	rmi4_data->vendor_id = TP_VENDOR_JDI;
	synaptics_rmi4_get_vendorstring(rmi4_data->vendor_id);
	print_ts(TS_INFO, KERN_ERR "[syna] vendor id: %x\n", rmi4_data->vendor_id);
}

#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *p,
        unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int new_status ;
    
    mutex_lock(&syna_rmi4_data->ops_lock);

    switch (event) {
        case FB_EVENT_BLANK :
            new_status = (*(int *)evdata->data) ? BLANK : UNBLANK;
            if (new_status == syna_rmi4_data->old_status)
                break;

            if(new_status != UNBLANK) {
                print_ts(TS_DEBUG, KERN_ERR "[syna]:suspend tp\n");
                synaptics_rmi4_suspend(&(syna_rmi4_data->input_dev->dev));
            }
            else {
                print_ts(TS_DEBUG, KERN_ERR "[syna]:resume tp\n");
                synaptics_rmi4_resume(&(syna_rmi4_data->input_dev->dev));
            }
            syna_rmi4_data->old_status = new_status;
            break;
    }
    mutex_unlock(&syna_rmi4_data->ops_lock);

    return 0;
}
#endif

/**
 * synaptics_rmi4_probe()
 *
 * Called by the kernel when an association with an I2C device of the
 * same name is made (after doing i2c_add_driver).
 *
 * This funtion allocates and initializes the resources for the driver
 * as an input driver, turns on the power to the sensor, queries the
 * sensor for its supported Functions and characteristics, registers
 * the driver to the input subsystem, sets up the interrupt, handles
 * the registration of the early_suspend and late_resume functions,
 * and creates a work queue for detection of other expansion Function
 * modules.
 */
static int  synaptics_rmi4_probe(struct i2c_client *client,
        const struct i2c_device_id *dev_id)
{
    int retval = 0;
	char product_info;
    unsigned char attr_count;
    struct synaptics_rmi4_data *rmi4_data;
    const struct synaptics_dsx_platform_data *platform_data =&dsx_platformdata;

        printk("%s, start ....!\n", __func__);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
        dev_err(&client->dev, "%s: SMBus byte data not supported\n", __func__);
        return -EIO;
    }

    if (!platform_data) {
        dev_err(&client->dev, "%s: No platform data found\n", __func__);
        return -EINVAL;
    }

    rmi4_data = kzalloc(sizeof(*rmi4_data), GFP_KERNEL);
    if (!rmi4_data) {
        dev_err(&client->dev, "%s: Failed to alloc mem for rmi4_data\n", __func__);
        return -ENOMEM;
    }

    rmi4_data->i2c_client = client;
    rmi4_data->current_page = MASK_8BIT;
    rmi4_data->board = platform_data;
    rmi4_data->touch_stopped = false;
    rmi4_data->sensor_sleep = false;
    rmi4_data->irq_enabled = false;
    rmi4_data->fingers_on_2d = false;

    rmi4_data->i2c_read = synaptics_rmi4_i2c_read;
    rmi4_data->i2c_write = synaptics_rmi4_i2c_write;
    rmi4_data->irq_enable = synaptics_rmi4_irq_enable;
    rmi4_data->reset_device = synaptics_rmi4_reset_device;
	rmi4_data->product_info = 3320;

	retval = i2c_master_send(rmi4_data->i2c_client,&product_info,1);
	if (retval < 0){
        printk("synap i2c test error!!! ret=%d\n",retval);
		goto err_regulator;
    }

    rmi4_data->virtual_key_height = 0;
    rmi4_data->sensor_max_x = LCD_MAX_X;
    rmi4_data->sensor_max_y = LCD_MAX_Y;
    syna_lcd_ratio1 = 133 ;
    syna_lcd_ratio2 = 100 ;

    mutex_init(&(rmi4_data->rmi4_io_ctrl_mutex));
    mutex_init(&(rmi4_data->rmi4_reset_mutex));

    syna_rmi4_data = rmi4_data;
    client->dev.platform_data = &dsx_platformdata ;
    retval = synaptics_parse_dt(&client->dev, rmi4_data);
    if (retval)
    {
        printk(KERN_ERR "%s synaptics parse_dt error \n", __func__);
        goto err_regulator;
    }

	if(vdd_regulator) 
       {
              printk("%s, config the regulator \n", __func__);
		if(synaptics_regulator_configure(true))
			goto err_set_input_dev;
	}

	rmi4_data->regulator = vdd_regulator ;

	dsx_platformdata.irq_gpio = rmi4_data->irq_gpio;
	dsx_platformdata.reset_gpio = rmi4_data->reset_gpio ;
	rmi4_data->irq = rmi4_data->i2c_client->irq ;

	rmi4_data->irq = gpio_to_irq(rmi4_data->irq_gpio);
    if (rmi4_data->irq <= 0)
    {
        printk("%s, irq number is not specified, irq #= %d, int pin=%d\n\n", __func__, rmi4_data->irq, rmi4_data->irq_gpio);
    }
    
    retval = gpio_request(rmi4_data->irq_gpio,"synap-s3320-int");
    if(retval < 0)
    {
        printk(KERN_ERR "%s: gpio_request_int, err=%d", __func__, retval);
        goto err_set_input_dev;
    }

    retval = gpio_direction_input(rmi4_data->irq_gpio);
    if(retval < 0)
    {
        printk(KERN_ERR "%s: gpio_direction_input, err=%d", __func__, retval);
        goto err_set_input_dev;
    }

    retval = gpio_request(rmi4_data->reset_gpio,"synap-s3320-reset");
    if(retval < 0)
    {
        printk(KERN_ERR "%s: gpio_request_reset, err=%d", __func__, retval);
        goto err_set_input_dev;
    }

    retval = gpio_direction_output(rmi4_data->reset_gpio,1);
    if(retval < 0)
    {
        printk(KERN_ERR "%s: reset gpio_direction_output, err=%d", __func__, retval);
        goto err_set_input_dev;
    }

	synaptics_rmi4_get_vendorid(rmi4_data);
	
	synaptics_ts_init_area(rmi4_data);

    i2c_set_clientdata(client, rmi4_data);

    retval = synaptics_rmi4_set_input_dev(rmi4_data);
    if (retval < 0) {
        dev_err(&client->dev, "%s: Failed to set up input device\n", __func__);
        goto err_set_input_dev;
    }

#ifdef CONFIG_HAS_EARLYSUSPEND
    rmi4_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
    rmi4_data->early_suspend.suspend = synaptics_rmi4_early_suspend;
    rmi4_data->early_suspend.resume = synaptics_rmi4_late_resume;
    register_early_suspend(&rmi4_data->early_suspend);
#endif

#ifdef CONFIG_FB   //for tp suspend and resume
    mutex_init(&rmi4_data->ops_lock);

    rmi4_data->fb_notif.notifier_call = fb_notifier_callback;

    retval = fb_register_client(&rmi4_data->fb_notif);

    if (retval)
        goto err_enable_irq ;
#endif

    if(rmi4_data->rmi4_mod_info.product_id_string[0])
        synaptics_id_str = rmi4_data->rmi4_mod_info.product_id_string ;
    else
        synaptics_id_str = "UNKNOWN";
    push_component_info(TP, synaptics_id_str, synaptics_vendor_str);

    if (syna_use_gesture)
    	rmi4_data->gesture_enable = 0x7b;
    synaptics_ts_init_virtual_key(rmi4_data);
    synaptics_rmi4_init_touchpanel_proc();

    //mingqiang.guo add for LCD show later when push power button  and  two click  in gesture     
    rmi4_data->speedup_resume_wq = create_singlethread_workqueue("speedup_resume_wq");
    INIT_DELAYED_WORK(&rmi4_data->speed_up_work,speedup_synaptics_resume);

    wake_lock_init(&tp_wake_lock, WAKE_LOCK_SUSPEND, "tp_gesture");

    if (!exp_data.initialized) {
        mutex_init(&exp_data.mutex);
        INIT_LIST_HEAD(&exp_data.list);
        exp_data.initialized = true;
    }

    exp_data.workqueue = create_singlethread_workqueue("dsx_exp_workqueue");
    INIT_DELAYED_WORK(&exp_data.work, synaptics_rmi4_exp_fn_work);
    exp_data.rmi4_data = rmi4_data;
    exp_data.queue_work = true;
    queue_delayed_work(exp_data.workqueue, &exp_data.work, msecs_to_jiffies(EXP_FN_WORK_DELAY_MS));

    rmi4_fw_module_init(true);
    while(1)
    {
        msleep(50);
        if(rmi4_data->bcontinue)
        {
            rmi4_fw_module_init(false);
            rmi4_data->bremove = 1;
            break ;
        }
    }

    for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
        retval = sysfs_create_file(&rmi4_data->input_dev->dev.kobj, &attrs[attr_count].attr);
        if (retval < 0) {
            dev_err(&client->dev, "%s: Failed to create sysfs attributes\n", __func__);
            goto err_sysfs;
        }
    }

    retval = synaptics_rmi4_irq_enable(rmi4_data, true);
    if (retval < 0) {
        dev_err(&client->dev, "%s: Failed to enable attention interrupt\n", __func__);
    }   
    synaptics_rmi4_i2c_read(rmi4_data, rmi4_data->f01_ctrl_base_addr+1, &int_mask,1);
    print_ts(TS_INFO, "%s: Interrupt enable mask = 0x%x\n", __func__, int_mask);

    printk("%s probe ok !!!! \n", __func__);
    return retval;

err_sysfs:
    for (attr_count--; attr_count >= 0; attr_count--) {
        sysfs_remove_file(&rmi4_data->input_dev->dev.kobj, &attrs[attr_count].attr);
    }
    cancel_delayed_work_sync(&exp_data.work);
    flush_workqueue(exp_data.workqueue);
    destroy_workqueue(exp_data.workqueue);
    synaptics_rmi4_irq_enable(rmi4_data, false);

err_enable_irq:
    if (platform_data->gpio_config && (platform_data->reset_gpio >= 0))
        platform_data->gpio_config(platform_data->reset_gpio, false);

    if (platform_data->gpio_config)
        platform_data->gpio_config(platform_data->irq_gpio, false);

    synaptics_rmi4_empty_fn_list(rmi4_data);

#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&rmi4_data->early_suspend);
#endif
    input_unregister_device(rmi4_data->input_dev);
    rmi4_data->input_dev = NULL;

err_set_input_dev:
    if (platform_data->regulator_en) {
        regulator_disable(rmi4_data->regulator);
        regulator_put(rmi4_data->regulator);
    }

err_regulator:
    kfree(rmi4_data);

    return retval;
}
#if 1
struct synaptics_optimize_data{
	struct delayed_work work;
	struct workqueue_struct *workqueue;
	struct i2c_client *client;
	const struct i2c_device_id *dev_id;
};
static struct synaptics_optimize_data optimize_data_s3320;

static void synaptics_s3320_work(struct work_struct *work)
{
    int retval;
    struct i2c_client *client_optimize = optimize_data_s3320.client;
    const struct i2c_device_id *dev_id = optimize_data_s3320.dev_id;

    retval = synaptics_rmi4_probe(client_optimize, dev_id);
}


static int synaptics_rmi4_probe_s3320(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
    int i, match;

    optimize_data_s3320.workqueue = create_workqueue("optimize_workqueue_s3320");
    INIT_DELAYED_WORK(&(optimize_data_s3320.work), synaptics_s3320_work);
    optimize_data_s3320.client = client;
    optimize_data_s3320.dev_id = dev_id;
    printk("synaptics s3320 probe cpu %d\n", smp_processor_id());

    for (i = 0; i <= 3; i++)
    {
        if (cpu_is_offline(i) || i == smp_processor_id())
        {
            continue;
        }
        queue_delayed_work_on(i, optimize_data_s3320.workqueue,
        &(optimize_data_s3320.work),
        msecs_to_jiffies(10));
        match = 1;
        printk("synap s3320 work on cpu %d\n",i);
        break;
    }
    if (match == 0)
    {
        queue_delayed_work_on(0, optimize_data_s3320.workqueue,
        &(optimize_data_s3320.work),
        msecs_to_jiffies(10));
    }
    return 0;
}
#endif

/**
 * synaptics_rmi4_remove()
 *
 * Called by the kernel when the association with an I2C device of the
 * same name is broken (when the driver is unloaded).
 *
 * This funtion terminates the work queue, stops sensor data acquisition,
 * frees the interrupt, unregisters the driver from the input subsystem,
 * turns off the power to the sensor, and frees other allocated resources.
 */
static int  synaptics_rmi4_remove(struct i2c_client *client)
{
    unsigned char attr_count;
    struct synaptics_rmi4_data *rmi4_data = i2c_get_clientdata(client);
    const struct synaptics_dsx_platform_data *platform_data =
        rmi4_data->board;

    synaptics_rmi4_irq_enable(rmi4_data, false);

    if (platform_data->gpio_config) {
        platform_data->gpio_config(
                platform_data->irq_gpio,
                false);

        if (platform_data->reset_gpio >= 0) {
            platform_data->gpio_config(
                    platform_data->reset_gpio,
                    false);
        }
    }

    for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
        sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
                &attrs[attr_count].attr);
    }

    cancel_delayed_work_sync(&exp_data.work);
    flush_workqueue(exp_data.workqueue);
    destroy_workqueue(exp_data.workqueue);

    synaptics_rmi4_empty_fn_list(rmi4_data);

#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&rmi4_data->early_suspend);
#endif
    input_unregister_device(rmi4_data->input_dev);

    if (platform_data->regulator_en) {
        regulator_disable(rmi4_data->regulator);
        regulator_put(rmi4_data->regulator);
    }

    kfree(rmi4_data);

    return 0;
}

#ifdef CONFIG_PM
/**
 * synaptics_rmi4_sensor_sleep()
 *
 * Called by synaptics_rmi4_early_suspend() and synaptics_rmi4_suspend().
 *
 * This function stops finger data acquisition and puts the sensor to sleep.
 */
static void synaptics_rmi4_sensor_sleep(struct synaptics_rmi4_data *rmi4_data)
{
    int retval;
    unsigned char device_ctrl;

    retval = synaptics_rmi4_i2c_read(rmi4_data, rmi4_data->f01_ctrl_base_addr, &device_ctrl, sizeof(device_ctrl));
    if (retval < 0) 
    {
        dev_err(&(rmi4_data->input_dev->dev), "%s: Failed to enter sleep mode\n", __func__);
        rmi4_data->sensor_sleep = false;
        return;
    }

    device_ctrl = (device_ctrl & ~MASK_3BIT);
    device_ctrl = (device_ctrl | NO_SLEEP_OFF | SENSOR_SLEEP);

    retval = synaptics_rmi4_i2c_write(rmi4_data, rmi4_data->f01_ctrl_base_addr, &device_ctrl, sizeof(device_ctrl));
    if (retval < 0) 
    {
        dev_err(&(rmi4_data->input_dev->dev), "%s: Failed to enter sleep mode\n", __func__);
        rmi4_data->sensor_sleep = false;
        return;
    }
    else 
    {
        rmi4_data->sensor_sleep = true;
    }

    return;
}

/**
 * synaptics_rmi4_sensor_wake()
 *
 * Called by synaptics_rmi4_resume() and synaptics_rmi4_late_resume().
 *
 * This function wakes the sensor from sleep.
 */
static void synaptics_rmi4_sensor_wake(struct synaptics_rmi4_data *rmi4_data)
{
    int retval;
    unsigned char device_ctrl;
    unsigned char no_sleep_setting = rmi4_data->no_sleep_setting;

    retval = synaptics_rmi4_i2c_read(rmi4_data, rmi4_data->f01_ctrl_base_addr, &device_ctrl, sizeof(device_ctrl));
    if (retval < 0) 
    {
        dev_err(&(rmi4_data->input_dev->dev), "%s: Failed to wake from sleep mode\n", __func__);
        rmi4_data->sensor_sleep = true;
        return;
    }

    device_ctrl = (device_ctrl & ~MASK_3BIT);
    device_ctrl = (device_ctrl | no_sleep_setting | NORMAL_OPERATION);

    retval = synaptics_rmi4_i2c_write(rmi4_data, rmi4_data->f01_ctrl_base_addr, &device_ctrl, sizeof(device_ctrl));
    if (retval < 0) 
    {
        dev_err(&(rmi4_data->input_dev->dev), "%s: Failed to wake from sleep mode\n", __func__);
        rmi4_data->sensor_sleep = true;
        return;
    } 
    else 
    {
        rmi4_data->sensor_sleep = false;
    }

    return;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
/**
 * synaptics_rmi4_early_suspend()
 *
 * Called by the kernel during the early suspend phase when the system
 * enters suspend.
 *
 * This function calls synaptics_rmi4_sensor_sleep() to stop finger
 * data acquisition and put the sensor to sleep.
 */
static void synaptics_rmi4_early_suspend(struct early_suspend *h)
{
    struct synaptics_rmi4_data *rmi4_data =
        container_of(h, struct synaptics_rmi4_data,
                early_suspend);

    if (rmi4_data->stay_awake) {
        rmi4_data->staying_awake = true;
        return;
    } else {
        rmi4_data->staying_awake = false;
    }

    rmi4_data->touch_stopped = true;
    synaptics_rmi4_irq_enable(rmi4_data, false);
    synaptics_rmi4_sensor_sleep(rmi4_data);
    synaptics_rmi4_free_fingers(rmi4_data);

    if (rmi4_data->full_pm_cycle)
        synaptics_rmi4_suspend(&(rmi4_data->input_dev->dev));

    return;
}

/**
 * synaptics_rmi4_late_resume()
 *
 * Called by the kernel during the late resume phase when the system
 * wakes up from suspend.
 *
 * This function goes through the sensor wake process if the system wakes
 * up from early suspend (without going into suspend).
 */
static void synaptics_rmi4_late_resume(struct early_suspend *h)
{
    int retval;
    struct synaptics_rmi4_data *rmi4_data =
        container_of(h, struct synaptics_rmi4_data,
                early_suspend);

    if (rmi4_data->staying_awake)
        return;

    if (rmi4_data->full_pm_cycle)
        synaptics_rmi4_resume(&(rmi4_data->input_dev->dev));

    if (rmi4_data->sensor_sleep == true) {
        synaptics_rmi4_sensor_wake(rmi4_data);
        synaptics_rmi4_irq_enable(rmi4_data, true);
        rmi4_data->touch_stopped = false;
        retval = synaptics_rmi4_reinit_device(rmi4_data);
        if (retval < 0) {
            dev_err(&rmi4_data->i2c_client->dev,
                    "%s: Failed to reinit device\n",
                    __func__);
        }
    }

    return;
}
#endif

/**
 * synaptics_rmi4_suspend()
 *
 * Called by the kernel during the suspend phase when the system
 * enters suspend.
 *
 * This function stops finger data acquisition and puts the sensor to
 * sleep (if not already done so during the early suspend phase),
 * disables the interrupt, and turns off the power to the sensor.
 */
static int synaptics_rmi4_suspend(struct device *dev)
{
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);
    unsigned char val=0;
    const struct synaptics_dsx_platform_data *platform_data =
        rmi4_data->board;

    if(rmi4_data->pwrrunning)
    {
        return 0 ;
    }

    down(&work_sem);
    rmi4_data->pwrrunning = true ;

    if(rmi4_data->smartcover_enable)
        synaptics_rmi4_close_smartcover();
    //synaptics_rmi4_i2c_write(syna_rmi4_data,SYNA_ADDR_SMARTCOVER_EXT,&val,sizeof(val));

    if(rmi4_data->glove_enable)
        synaptics_rmi4_i2c_write(syna_rmi4_data,SYNA_ADDR_GLOVE_FLAG,&val,sizeof(val));

    synaptics_enable_gesture(rmi4_data,true);
    synaptics_enable_pdoze(rmi4_data,true);
    if(rmi4_data->gesture || rmi4_data->pdoze_enable) {
        synaptics_enable_irqwake(rmi4_data,true);
        gpio_set_value(rmi4_data->reset_gpio,1);
        goto GO_OUT;
    }

    if (rmi4_data->staying_awake) {
        goto GO_OUT;
    }

    if (!rmi4_data->sensor_sleep) {
        rmi4_data->touch_stopped = true;

        synaptics_rmi4_irq_enable(rmi4_data, false);
        synaptics_rmi4_sensor_sleep(rmi4_data);
        synaptics_rmi4_free_fingers(rmi4_data);

        //when suspend, close tp's vdd power
        if (platform_data->regulator_en) {
#ifdef CHECK_WRONG_TP_BEFORCE_MAKE 
            regulator_disable(rmi4_data->regulator);
#else
            //regulator_disable(rmi4_data->regulator);
#endif 
        }
    }

GO_OUT:
    rmi4_data->pwrrunning = false ;
    up(&work_sem);
    return 0;
}

#ifdef SYNC_RMI4_PWR
void synaptics_rmi4_sync_lcd_suspend(void) {
    if(!syna_rmi4_data)
        return ;
    synaptics_rmi4_suspend(&syna_rmi4_data->i2c_client->dev);
}
void synaptics_rmi4_sync_lcd_resume(void) {
    if(!syna_rmi4_data)
        return ;
    synaptics_rmi4_resume(&syna_rmi4_data->i2c_client->dev);
}
#endif

//mingqiang.guo add for LCD show later when push power button  and  two click  in gesture  
static void speedup_synaptics_resume(struct work_struct *work)            
{   
    int retval;
    unsigned char val=1;
    struct synaptics_rmi4_data *rmi4_data = syna_rmi4_data; 
    const struct synaptics_dsx_platform_data *platform_data =	rmi4_data->board;
    print_ts(TS_DEBUG,"[TP] %s:%d\n",__func__,__LINE__);
    
    if(rmi4_data->pwrrunning)
        return;

    print_ts(TS_DEBUG, KERN_ERR "gesture status[0x%x,0x%x]\n", syna_use_gesture,rmi4_data->gesture_enable);
    
	down(&work_sem);
	synaptics_set_int_mask(syna_rmi4_data, 0);//mingqiang.guo add for black gesture failure

    rmi4_data->pwrrunning = true ;

#if 1  //software reset it
    synaptics_rmi4_i2c_write(rmi4_data,rmi4_data->f01_cmd_base_addr,&val,sizeof(val));
    msleep(200);
#else //hardware reset it ，maybe lead to balcak gesture failure 
    //hardware reset it 
    retval = synaptics_rmi4_reset_device(rmi4_data, rmi4_data->f01_cmd_base_addr);
    if (retval < 0) {
        printk(	"%s: Failed to issue TP reset command, error = %d\n",__func__, retval);
        goto GO_RETURN;
    }
#endif 

    if(rmi4_data->smartcover_enable)
        synaptics_rmi4_open_smartcover();
    //synaptics_rmi4_i2c_write(syna_rmi4_data,SYNA_ADDR_SMARTCOVER_EXT,&val,sizeof(val));

    if(rmi4_data->glove_enable)
        synaptics_rmi4_i2c_write(syna_rmi4_data,SYNA_ADDR_GLOVE_FLAG,&val,sizeof(val));

    if(rmi4_data->gesture || rmi4_data->pdoze_enable) {
        synaptics_enable_gesture(rmi4_data,false);
        synaptics_enable_pdoze(rmi4_data,false);
        synaptics_enable_irqwake(rmi4_data,false);
        rmi4_data->pwrrunning = false ;
        goto GO_RETURN;
    }

    if (rmi4_data->staying_awake) {
        rmi4_data->pwrrunning = false ;
        goto GO_RETURN;
    }

    if (!rmi4_data->sensor_sleep) {
        rmi4_data->pwrrunning = false ;
        goto GO_RETURN;
    }

    //resume tp's vdd power
    if (platform_data->regulator_en) {
#ifdef CHECK_WRONG_TP_BEFORCE_MAKE 
        regulator_enable(rmi4_data->regulator);
        msleep(platform_data->reset_delay_ms);
        rmi4_data->current_page = MASK_8BIT;
#else
        //regulator_enable(rmi4_data->regulator);
        //msleep(platform_data->reset_delay_ms);
        //rmi4_data->current_page = MASK_8BIT;
#endif
    }

    synaptics_rmi4_sensor_wake(rmi4_data);
    synaptics_rmi4_irq_enable(rmi4_data, true);
    rmi4_data->touch_stopped = false;
    retval = synaptics_rmi4_reinit_device(rmi4_data);
    if (retval < 0) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Failed to reinit device\n",
                __func__);
        rmi4_data->pwrrunning = false ;
        goto GO_RETURN;
    }
    rmi4_data->pwrrunning = false ;

GO_RETURN:
    synaptics_set_int_mask(syna_rmi4_data, 1);//mingqiang.guo add for black gesture failure
    up(&work_sem);
    return ;
}
//mingqiang.guo add end 

/**
 * synaptics_rmi4_resume()
 *
 * Called by the kernel during the resume phase when the system
 * wakes up from suspend.
 *
 * This function turns on the power to the sensor, wakes the sensor
 * from sleep, enables the interrupt, and starts finger data
 * acquisition.
 */
static int synaptics_rmi4_resume(struct device *dev)
{
    //mingqiang.guo add for LCD show later when push power button  and  two click  in gesture   
    print_ts(TS_DEBUG,"%s is called\n",__func__);
    queue_delayed_work(syna_rmi4_data->speedup_resume_wq,&syna_rmi4_data->speed_up_work, msecs_to_jiffies(10));
	msleep(50);//LCD亮得太快,会闪烁
    return 0;
}

#ifndef CONFIG_FB
static const struct dev_pm_ops synaptics_rmi4_dev_pm_ops = {
    .suspend = synaptics_rmi4_suspend,
    .resume  = synaptics_rmi4_resume,
};
#endif
#endif

static const struct i2c_device_id synaptics_rmi4_id_table[] = {
    {DRIVER_NAME, 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, synaptics_rmi4_id_table);

static struct of_device_id synaptics_of_match_table[] = {
    { .compatible = "synaptics,rmi-ts",},
    { },
};

static struct i2c_driver synaptics_rmi4_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
#ifdef CONFIG_PM
#ifndef CONFIG_FB
        .pm = &synaptics_rmi4_dev_pm_ops,
#endif
#endif
        .of_match_table = synaptics_of_match_table,
    },
    .probe = synaptics_rmi4_probe_s3320,
    .remove = (synaptics_rmi4_remove),
    .id_table = synaptics_rmi4_id_table,
};

/**
 * synaptics_rmi4_init()
 *
 * Called by the kernel during do_initcalls (if built-in)
 * or when the driver is loaded (if a module).
 *
 * This function registers the driver to the I2C subsystem.
 *
 */
static int __init synaptics_rmi4_init(void)
{
    return i2c_add_driver(&synaptics_rmi4_driver);
}

/**
 * synaptics_rmi4_exit()
 *
 * Called by the kernel when the driver is unloaded.
 *
 * This funtion unregisters the driver from the I2C subsystem.
 *
 */
static void __exit synaptics_rmi4_exit(void)
{
    i2c_del_driver(&synaptics_rmi4_driver);

    return;
}

module_init(synaptics_rmi4_init);
module_exit(synaptics_rmi4_exit);

MODULE_DESCRIPTION("Synaptics DSX I2C Touch Driver");
MODULE_LICENSE("GPL");
