/*************************************************************
 ** Copyright (C), 2012-2016, OEM Mobile Comm Corp., Ltd 
 ** VENDOR_EDIT
 ** File        : synaptics_dsx_s1302.c
 ** Description : 
 ** Date        : 2015-3-19 15:37
 ** Author      : BSP
 ** 
 ** ------------------ Revision History: ---------------------
 **      <author>        <date>          <desc>
 *************************************************************/

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
#include <linux/workqueue.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/wakelock.h>
#include <linux/project_info.h>

#include "synaptics_dsx.h"
#include "synaptics_dsx_i2c.h"

#define DRIVER_NAME "synaptics-s1302"
#define INPUT_PHYS_NAME "synaptics-rmi-ts/input5"

#include <linux/wakelock.h>
#include <linux/smp.h>

#define SYN_I2C_RETRY_TIMES 10
#define S1302_WORK_DELAY_MS 200 /* ms */

struct synaptics_rmi4_exp_fn_s1302{
    enum exp_fn fn_type;
    bool inserted;
    int (*func_init)(struct synaptics_rmi4_data *rmi4_data);
    void (*func_remove)(struct synaptics_rmi4_data *rmi4_data);
    void (*func_attn)(struct synaptics_rmi4_data *rmi4_data,
            unsigned char intr_mask);
    struct list_head link;
};


struct synaptics_rmi4_exp_fn_data_s1302 {
    bool initialized;
    bool queue_work;
    struct mutex mutex;
    struct list_head list;
    struct delayed_work work;
    struct workqueue_struct *workqueue;
    struct synaptics_rmi4_data *rmi4_data;
};
struct synaptics_rmi4_exp_fn_data_s1302 exp_data_s1302;
static struct synaptics_rmi4_data *syna_s1302_data=0;
static int key_rep = 0;

static int synaptics_rmi4_s1302_reset(struct synaptics_rmi4_data *rmi4_data, unsigned short f01_cmd_base_addr);
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
            dev_err(&rmi4_data->i2c_client->dev, "%s: I2C read over retry limit\n", __func__);
            synaptics_rmi4_s1302_reset(rmi4_data, rmi4_data->f01_cmd_base_addr);
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
#define S1302_RESET_ADDR	0x006D
#define S1302_SLEEP_ADDR	0x0037
#define KEY_ADD_STATUS		0x0014
#define KEY_ADD				0x0200
#define S1302_ID_ADDR		0x0083
static int synaptics_rmi4_s1302_reset(struct synaptics_rmi4_data *rmi4_data,unsigned short f01_cmd_base_addr)
{
	int retval;
	char status = 1;

	retval = synaptics_rmi4_i2c_write(rmi4_data,S1302_RESET_ADDR,&status,1);
	printk("s1302 soft reset !!!!\n");
	return 0;
}
static void synaptics_rmi4_s1302_hard_reset(struct synaptics_rmi4_data *rmi4_data)
{
	gpio_set_value(rmi4_data->reset_gpio,0);
	msleep(10);
    printk("%s rmi4_data->reset_gpio:%d\n",__func__,gpio_get_value(rmi4_data->reset_gpio));
    gpio_set_value(rmi4_data->reset_gpio,1);
    msleep(100);
    printk("%s rmi4_data->reset_gpio:%d\n",__func__,gpio_get_value(rmi4_data->reset_gpio));
	printk("s1302 hard reset !!!!\n");
}
#define REP_KEY_MENU (key_rep?(KEY_BACK):(KEY_MENU))
#define REP_KEY_BACK (key_rep?(KEY_MENU):(KEY_BACK))
static void synaptics_s1302_report(struct synaptics_rmi4_data *rmi4_data)
{

    int retval;
	char status;
	char button_key;

    retval = synaptics_rmi4_i2c_read(rmi4_data,KEY_ADD_STATUS,&status,1);
	//printk("%s  status:%d\n",__func__,status);
    if (retval < 0) {
        dev_err(&rmi4_data->i2c_client->dev,
                "%s: Failed to read button data registers\n",
                __func__);
        return;
    }
    if (status && 0x10)
    {
        retval = synaptics_rmi4_i2c_read(rmi4_data,KEY_ADD,&button_key,1);
        printk("%s	button_key:%d   pre_btn_state:%d\n",__func__,button_key,rmi4_data->pre_btn_state);
        if((button_key & 0x01) && !(rmi4_data->pre_btn_state & 0x01))//back
        {
            input_report_key(rmi4_data->input_dev, REP_KEY_BACK, 1);
            input_sync(rmi4_data->input_dev);
        }else if(!(button_key & 0x01) && (rmi4_data->pre_btn_state & 0x01)){
            input_report_key(rmi4_data->input_dev, REP_KEY_BACK, 0);
            input_sync(rmi4_data->input_dev);
        }

        if((button_key & 0x02) && !(rmi4_data->pre_btn_state & 0x02))//menu
        {
            input_report_key(rmi4_data->input_dev, REP_KEY_MENU, 1);
            input_sync(rmi4_data->input_dev);
        }else if(!(button_key & 0x02) && (rmi4_data->pre_btn_state & 0x02)){
            input_report_key(rmi4_data->input_dev, REP_KEY_MENU, 0);
            input_sync(rmi4_data->input_dev);
        }

        if((button_key & 0x04) && !(rmi4_data->pre_btn_state & 0x04))//home
        {
            input_report_key(rmi4_data->input_dev, KEY_HOMEPAGE, 1);//KEY_HOMEPAGE
            input_sync(rmi4_data->input_dev);
        }else if(!(button_key & 0x04) && (rmi4_data->pre_btn_state & 0x04)){
            input_report_key(rmi4_data->input_dev, KEY_HOMEPAGE, 0);
            input_sync(rmi4_data->input_dev);
        }

        rmi4_data->pre_btn_state = button_key & 0x07;
		//input_sync(rmi4_data->input_dev);
	}
    return;
}
//irq work function
static void synaptics_s1302_report_work(struct work_struct *work) {
    struct synaptics_rmi4_data *rmi4_data = syna_s1302_data;

    synaptics_s1302_report(rmi4_data);
    enable_irq(rmi4_data->i2c_client->irq);
}
static int synaptics_parse_dt(struct device *dev, struct synaptics_rmi4_data *ts)
{
    int ret = 0;
    if (dev->of_node)
    {
        struct device_node *np = dev->of_node;

	    /* reset, irq gpio info */
	    ts->irq_gpio = of_get_named_gpio(np, "synaptics,irq-gpio", 0);
		if( ts->irq_gpio < 0 ){
			printk("ts->irq_gpio not specified\n");
		}
	    ts->reset_gpio = of_get_named_gpio(np, "synaptics,reset-gpio", 0);
		if( ts->reset_gpio < 0 ){
			printk("ts->reset_gpio not specified\n");
		}
	    ts->v3_gpio = of_get_named_gpio(np, "synaptics,en3v_gpio", 0);
		if( ts->v3_gpio < 0 ){
			printk("ts->v3_gpio not specified\n");
		}
	}
    return ret;
}

/**
 * synaptics_s1302_wq()
 *
 * Called by the kernel when an interrupt occurs (when the sensor
 * asserts the attention irq).
 *
 * This function is the ISR thread and handles the acquisition
 * and the reporting of finger data when the presence of fingers
 * is detected.
 */
static irqreturn_t synaptics_s1302_irq(int irq, void *data)
{
    struct synaptics_rmi4_data *rmi4_data = data;

    disable_irq_nosync(rmi4_data->i2c_client->irq);
    //use work to handle irq event
    queue_work(rmi4_data->reportqueue, &rmi4_data->reportwork);
    return IRQ_HANDLED;
}

/**
 * synaptics_s1302_irq_enable()
 *
 * Called by synaptics_rmi4_probe() and the power management functions
 * in this driver and also exported to other expansion Function modules
 * such as rmi_dev.
 *
 * This function handles the enabling and disabling of the attention
 * irq including the setting up of the ISR thread.
 */
static int synaptics_s1302_irq_enable(struct synaptics_rmi4_data *rmi4_data,
        bool enable)
{
    int retval = 0;
    unsigned char intr_status;

    if (enable) {
        if (rmi4_data->irq_enabled)
            return retval;

        /* Clear interrupts first */
        retval = synaptics_rmi4_i2c_read(rmi4_data, 0x0014, &intr_status, 1);
        if (retval < 0)
            return retval;
		//printk("synap read *******status:[0x%d],[0x%d],[0x%d],[0x%d],[0x%d],[0x%d],[0x%d],[0x%d]\n",intr_status[0],
		//	intr_status[1],intr_status[2],intr_status[3],intr_status[4],intr_status[5],intr_status[6],intr_status[7]);
		printk("synap s1302 irq=%d\n",rmi4_data->irq);
        retval = request_irq(rmi4_data->irq, synaptics_s1302_irq, IRQF_TRIGGER_FALLING, DRIVER_NAME, rmi4_data);
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

static int synaptics_rmi4_set_input(struct synaptics_rmi4_data *rmi4_data)
{
    int retval;

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
    set_bit(KEY_BACK, rmi4_data->input_dev->keybit);
    set_bit(KEY_MENU, rmi4_data->input_dev->keybit);
    set_bit(KEY_HOMEPAGE, rmi4_data->input_dev->keybit);
#ifdef INPUT_PROP_DIRECT
    set_bit(INPUT_PROP_DIRECT, rmi4_data->input_dev->propbit);
#endif

   // retval = synaptics_rmi4_query_device(rmi4_data);
    if (retval < 0) {
        dev_err(&rmi4_data->i2c_client->dev, "%s: Failed to query device\n", __func__);
        goto err_query_device;
    }

    retval = input_register_device(rmi4_data->input_dev);
    if (retval) {
        dev_err(&rmi4_data->i2c_client->dev, "%s: Failed to register input device\n", __func__);
        goto err_register_input;
    }

    return 0;

err_register_input:
err_query_device:
    input_free_device(rmi4_data->input_dev);

err_input_device:
    return retval;
}
extern int rmi4_fw_module_init_s1302(bool insert);

/**
 * synaptics_rmi4_exp_fn_work_s1302()
 *
 * Called by the kernel at the scheduled time.
 *
 * This function is a work thread that checks for the insertion and
 * removal of other expansion Function modules such as rmi_dev and calls
 * their initialization and removal callback functions accordingly.
 */
static void synaptics_rmi4_exp_fn_work_s1302(struct work_struct *work)
{
    struct synaptics_rmi4_exp_fn_s1302 *exp_fhandler;
    struct synaptics_rmi4_exp_fn_s1302 *exp_fhandler_temp;
    struct synaptics_rmi4_data *rmi4_data = exp_data_s1302.rmi4_data;

    mutex_lock(&exp_data_s1302.mutex);
    if (!list_empty(&exp_data_s1302.list)) {
        list_for_each_entry_safe(exp_fhandler,exp_fhandler_temp,&exp_data_s1302.list,	link) 
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
                    queue_delayed_work(exp_data_s1302.workqueue, &exp_data_s1302.work, msecs_to_jiffies(500));
                    mutex_unlock(&exp_data_s1302.mutex);
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
    mutex_unlock(&exp_data_s1302.mutex);

    return;
}
/**
 * synaptics_rmi4_new_function_s1302()
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
void synaptics_rmi4_new_function_s1302(enum exp_fn fn_type, bool insert,
        int (*func_init)(struct synaptics_rmi4_data *rmi4_data),
        void (*func_remove)(struct synaptics_rmi4_data *rmi4_data),
        void (*func_attn)(struct synaptics_rmi4_data *rmi4_data,
            unsigned char intr_mask))
{
    struct synaptics_rmi4_exp_fn_s1302 *exp_fhandler;

    if (!exp_data_s1302.initialized) {
        mutex_init(&exp_data_s1302.mutex);
        INIT_LIST_HEAD(&exp_data_s1302.list);
        exp_data_s1302.initialized = true;
    }

    mutex_lock(&exp_data_s1302.mutex);
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
        list_add_tail(&exp_fhandler->link, &exp_data_s1302.list);
    } 
    else if (!list_empty(&exp_data_s1302.list)) 
    {
        list_for_each_entry(exp_fhandler, &exp_data_s1302.list, link) 
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
    mutex_unlock(&exp_data_s1302.mutex);

    if (exp_data_s1302.queue_work) {
        queue_delayed_work(exp_data_s1302.workqueue, &exp_data_s1302.work, msecs_to_jiffies(S1302_WORK_DELAY_MS));
    }

    return;
}
#ifdef CONFIG_FB
#define NORMAL_OPERATION (0 << 0)
#define SENSOR_SLEEP (1 << 0)
#define NO_SLEEP_OFF (0 << 2)
#define NO_SLEEP_ON (1 << 2)
#define CONFIGURED (1 << 7)

#define MASK_3BIT 0x07
/**
 * synaptics_s1302_sleep()
 *
 * Called by synaptics_rmi4_early_suspend() and synaptics_rmi4_suspend().
 *
 * This function stops finger data acquisition and puts the sensor to sleep.
 */
static void synaptics_s1302_sleep(struct synaptics_rmi4_data *rmi4_data)
{
    int retval;
    unsigned char device_ctrl;

    msleep(20);
    retval = synaptics_rmi4_i2c_read(rmi4_data, S1302_SLEEP_ADDR, &device_ctrl, sizeof(device_ctrl));
    if (retval < 0) 
    {
        dev_err(&(rmi4_data->input_dev->dev), "%s: Failed to enter sleep mode\n", __func__);
        rmi4_data->sensor_sleep = false;
        return;
    }

    device_ctrl = (device_ctrl & ~MASK_3BIT);
    device_ctrl = (device_ctrl | NO_SLEEP_OFF | SENSOR_SLEEP);

    retval = synaptics_rmi4_i2c_write(rmi4_data, S1302_SLEEP_ADDR, &device_ctrl, sizeof(device_ctrl));
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
 * synaptics_s1302_wake()
 *
 * Called by synaptics_rmi4_resume() and synaptics_rmi4_late_resume().
 *
 * This function wakes the sensor from sleep.
 */
static void synaptics_s1302_wake(struct synaptics_rmi4_data *rmi4_data)
{
    int retval;
    unsigned char device_ctrl;
    unsigned char no_sleep_setting = rmi4_data->no_sleep_setting;

    retval = synaptics_rmi4_i2c_read(rmi4_data, S1302_SLEEP_ADDR, &device_ctrl, sizeof(device_ctrl));
    if (retval < 0) 
    {
        dev_err(&(rmi4_data->input_dev->dev), "%s: Failed to wake from sleep mode\n", __func__);
        rmi4_data->sensor_sleep = true;
        return;
    }

    device_ctrl = (device_ctrl & ~MASK_3BIT);
    device_ctrl = (device_ctrl | no_sleep_setting | NORMAL_OPERATION);

    retval = synaptics_rmi4_i2c_write(rmi4_data, S1302_SLEEP_ADDR, &device_ctrl, sizeof(device_ctrl));
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
static DEFINE_SEMAPHORE(work_sem);  
static int synaptics_s1302_suspend(struct device *dev)
{
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

    synaptics_s1302_sleep(rmi4_data);
    down(&work_sem);
    synaptics_s1302_irq_enable(rmi4_data, false);
    //regulator_disable(rmi4_data->regulator);
    up(&work_sem);
    return 0;
}

static int synaptics_s1302_resume(struct device *dev)            
{   
    struct synaptics_rmi4_data *rmi4_data = dev_get_drvdata(dev);

    synaptics_s1302_wake(rmi4_data);
	down(&work_sem);
#if 0  //software reset it
    unsigned char val=1;
    synaptics_rmi4_i2c_write(rmi4_data,rmi4_data->f01_cmd_base_addr,&val,sizeof(val));
#endif

	//regulator_enable(rmi4_data->regulator);
    synaptics_s1302_irq_enable(rmi4_data, true);
    up(&work_sem);
    return 0;
}
#define BLANK		1
#define UNBLANK		0
static int fb_notifier_callback(struct notifier_block *p,
        unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int new_status ;
    
    mutex_lock(&syna_s1302_data->ops_lock);

    switch (event) {
        case FB_EVENT_BLANK :
            new_status = (*(int *)evdata->data) ? BLANK : UNBLANK;
            if (new_status == syna_s1302_data->old_status)
                break;

            if(new_status != UNBLANK) {
                printk("synap s1302:suspend!\n");
                synaptics_s1302_suspend(&(syna_s1302_data->input_dev->dev));
            }
            else {
                printk("synap s1302:resume!\n");
                synaptics_s1302_resume(&(syna_s1302_data->input_dev->dev));
            }
            syna_s1302_data->old_status = new_status;
            break;
    }
    mutex_unlock(&syna_s1302_data->ops_lock);

    return 0;
}
#endif
static int synaptics_fw_version_show(struct seq_file *seq, void *offset)
{
    seq_printf(seq, "s1302 device_id:%d,image_id:%d\n",syna_s1302_data->device_cid,\
		syna_s1302_data->image_cid);
    return 0 ;
}
static int synaptics_s1302_open(struct inode *inode, struct file *file)
{
	return single_open(file, synaptics_fw_version_show, inode->i_private);
}
static int synaptics_rep_show(struct seq_file *seq, void *offset)
{
    seq_printf(seq, "s1302 menu key in %s\n",key_rep?("right"):("left"));
    return 0 ;
}
static ssize_t synaptics_rep_write(struct file *file, const char __user *page, size_t t, loff_t *lo)
{
	int ret = 0;
	char buf[10];

	if( t > 2)
		return t;
	if( copy_from_user(buf, page, t) ){
		printk(KERN_INFO "%s: read proc input error.\n", __func__);
		return t;
	}

	sscanf(buf, "%d", &ret);

	if( (ret == 0 )||(ret == 1) )
    {
        key_rep = ret;
    }
    return t;
}
static int synaptics_s1302_key_rep(struct inode *inode, struct file *file)
{
	return single_open(file, synaptics_rep_show, inode->i_private);
}
static int spage ,addr,blc;
static int synap_read_address(struct seq_file *seq, void *offset)
{
	int ret;
	char buffer[128];
    int i;

    printk("%s page=0x%x,address=0x%x,block=0x%x\n",__func__,spage,addr,blc);
    ret = synaptics_rmi4_i2c_read(syna_s1302_data,((spage&0xff << 8) | addr),buffer,blc);
    for (i=0;i < blc;i++)
    {
        printk("buffer[%d]=0x%x\n",i,buffer[i]);
    }
    seq_printf(seq, "spage:%x addr:0x%x blc:%d\n",spage,addr,blc);
	return 0;
}

static ssize_t synap_write_address(struct file *file, const char __user *buffer, size_t count, loff_t *ppos)
{
	int buf[128];
    int ret,i;
    int temp_block,wbyte;
    char reg[30];

    ret = sscanf(buffer,"%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",\
    &buf[0],&buf[1],&buf[2],&buf[3],&buf[4],&buf[5],&buf[6],&buf[7],&buf[8],&buf[9],\
    &buf[10],&buf[11],&buf[12],&buf[13],&buf[14],&buf[15],&buf[16],&buf[17]);
    for (i = 0;i < ret;i++)
    {
        printk("buf[i]=0x%x,",buf[i]);
    }
    printk("\n");
    spage= buf[0];
    addr = buf[1];
    temp_block = buf[2];
    wbyte = buf[3];
    if (0xFF == temp_block)//the  mark is to write register else read register
    {
        for (i=0;i < wbyte;i++)
        {
            reg[i] = (char)buf[4+i];
        }
        ret = synaptics_rmi4_i2c_write(syna_s1302_data,((spage&0xff << 8) | addr),reg,wbyte);
        printk("%s write page=0x%x,address=0x%x\n",__func__,spage,addr);
        for (i=0;i < wbyte;i++)
        {
            printk("reg=0x%x\n",reg[i]);
        }
    }
    else
        blc = temp_block;
	return count;
}
static int synaptics_addr_open(struct inode *inode, struct file *file)
{
	return single_open(file, synap_read_address, inode->i_private);
}

static ssize_t synaptics_s1302_reset_write (struct file *file, const char *buffer, size_t count, loff_t *ppos)
{
    int ret,write_flag;
    int retval;
    char status = 1;

    ret = sscanf(buffer,"%x",&write_flag);
    printk("%s write %d\n",__func__,write_flag);
    if (1 == write_flag)
    {
        retval = synaptics_rmi4_i2c_write(syna_s1302_data,S1302_RESET_ADDR,&status,1);
        printk("s1302 soft reset !!!!\n");
    }
    else if(2 == write_flag)
    {
        synaptics_rmi4_s1302_hard_reset(syna_s1302_data);
    }
    else if(3 == write_flag)
    {
        gpio_set_value(syna_s1302_data->v3_gpio,0);
        msleep(10);
        printk("%s rmi4_data->v3_gpio:%d\n",__func__,gpio_get_value(syna_s1302_data->v3_gpio));
        gpio_set_value(syna_s1302_data->v3_gpio,1);
        msleep(100);
        printk("%s rmi4_data->v3_gpio:%d\n",__func__,gpio_get_value(syna_s1302_data->v3_gpio));
        printk("s1302 power off -> on !!!!\n");
    }
	return count;
}
static int test_err = 0;
static int synaptics_reset_show(struct seq_file *seq, void *offset)
{
    seq_printf(seq, "%d\n",test_err);
    return 0 ;
}

static int synaptics_s1302_reset_open(struct inode *inode, struct file *file)
{
	return single_open(file, synaptics_reset_show, inode->i_private);
}

const struct file_operations synap_s1302[] =
{
	{
		.owner		= THIS_MODULE,
		.open		= synaptics_s1302_open,
		.read		= seq_read,
		.llseek 	= seq_lseek,
		.release	= single_release,
	},
	{
		.owner		= THIS_MODULE,
		.open		= synaptics_s1302_key_rep,
		.read		= seq_read,
		.write      = synaptics_rep_write,
		.llseek 	= seq_lseek,
		.release	= single_release,
	},
    {
        .owner      = THIS_MODULE,
        .open       = synaptics_addr_open,
        .read       = seq_read,
        .write      = synap_write_address,
        .llseek     = seq_lseek,
        .release    = single_release,
    },
    {
        .owner      = THIS_MODULE,
        .open       = synaptics_s1302_reset_open,
        .read       = seq_read,
        .write      = synaptics_s1302_reset_write,
        .llseek     = seq_lseek,
        .release    = single_release,
    }
};

static int synaptics_s1302_proc(void)
{
    struct proc_dir_entry *proc_entry=0;

    struct proc_dir_entry *procdir = proc_mkdir( "s1302", NULL );
    //for firmware version
    proc_entry = proc_create_data("fw_version", 0444, procdir,&synap_s1302[0],NULL);
    proc_entry = proc_create_data("key_rep", 0666, procdir,&synap_s1302[1],NULL);
    proc_entry = proc_create_data("radd", 0666, procdir,&synap_s1302[2],NULL);
    proc_entry = proc_create_data("reset", 0666, procdir,&synap_s1302[3],NULL);
    printk("%s over.\n", __func__);

    return 0;
}
EXPORT_SYMBOL(synaptics_rmi4_new_function_s1302);

extern char* synaptics_s1302_get_vender(void);

static int  synaptics_1302_probe(struct i2c_client *client,
        const struct i2c_device_id *dev_id)
{
    int retval;
    char buffer[6] = {0};
	
    struct synaptics_rmi4_data *rmi4_data;
	
    rmi4_data = kzalloc(sizeof(*rmi4_data), GFP_KERNEL);
    if (!rmi4_data) {
        dev_err(&client->dev, "%s: Failed to alloc mem for rmi4_data\n", __func__);
        return -ENOMEM;
    }

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
        dev_err(&client->dev, "%s: SMBus byte data not supported\n", __func__);
        goto ERR_FREE;
    }

    mutex_init(&(rmi4_data->rmi4_io_ctrl_mutex));
    mutex_init(&(rmi4_data->rmi4_reset_mutex));

    retval = synaptics_parse_dt(&client->dev, rmi4_data);
    if (retval)
    {
        printk(KERN_ERR "%s synaptics parse_dt error \n", __func__);
		goto ERR_FREE;
    }

	rmi4_data->irq = gpio_to_irq(rmi4_data->irq_gpio);
	if (rmi4_data->irq <= 0)
	{
		printk("%s, irq number is not specified, irq #= %d, int pin=%d\n\n", __func__, rmi4_data->irq, rmi4_data->irq_gpio);
	}
	retval = gpio_request(rmi4_data->irq_gpio,"s1302_int");
	if(retval < 0)
	{
		printk(KERN_ERR "%s: gpio_request, err=%d", __func__, retval);
	}

	retval = gpio_request(rmi4_data->reset_gpio,"s1302_reset");
	if(retval < 0)
	{
		printk(KERN_ERR "%s: gpio_request, err=%d", __func__, retval);
	}

	retval = gpio_request(rmi4_data->v3_gpio,"s1302_3v_en");
	if(retval < 0)
	{
		printk(KERN_ERR "%s: v3_gpio_request, err=%d", __func__, retval);
	}
	retval = gpio_direction_output(rmi4_data->reset_gpio,1);
	retval = gpio_direction_output(rmi4_data->v3_gpio,1);
	retval = gpio_direction_input(rmi4_data->irq_gpio);
	if(retval < 0)
	{
		printk(KERN_ERR "%s: gpio_direction_input, err=%d", __func__, retval);
	}
	gpio_set_value(rmi4_data->irq_gpio,1);

    rmi4_data->i2c_client = client;
    rmi4_data->current_page = MASK_8BIT;
    rmi4_data->irq_enabled = false;

    rmi4_data->i2c_read = synaptics_rmi4_i2c_read;
    rmi4_data->i2c_write = synaptics_rmi4_i2c_write;
    rmi4_data->irq_enable = synaptics_s1302_irq_enable;
    rmi4_data->reset_device = synaptics_rmi4_s1302_reset;
	syna_s1302_data = rmi4_data;
	rmi4_data->product_info = 1302;
    retval = synaptics_rmi4_i2c_read(rmi4_data,S1302_ID_ADDR,&buffer[0],5);
    printk("%s i2c test read id is %s\n",__func__,&buffer[0]);
	if (retval < 0){
        test_err = 1;
        synaptics_rmi4_s1302_reset(rmi4_data, rmi4_data->f01_cmd_base_addr);
        msleep(100);
        retval = synaptics_rmi4_i2c_read(rmi4_data,S1302_ID_ADDR,&buffer[0],5);
        if (retval < 0){
            printk("synap s1302 i2c test fail retval=%d buffer:%s\n",retval,&buffer[0]);
            //goto ERR_FREE;
        }
	}

#ifdef CONFIG_FB   //for tp suspend and resume
		mutex_init(&rmi4_data->ops_lock);
		rmi4_data->fb_notif.notifier_call = fb_notifier_callback;
		retval = fb_register_client(&rmi4_data->fb_notif);
		if (retval)
			goto ERR_FREE ;
#endif
    i2c_set_clientdata(client, rmi4_data);
    retval = synaptics_rmi4_set_input(rmi4_data);
    if (retval < 0) {
        dev_err(&client->dev, "%s: Failed to set up input device\n", __func__);
        goto ERR_INPUT;
    }
	rmi4_data->reportqueue = create_singlethread_workqueue("synaptics_s1302_wq");
	INIT_WORK(&rmi4_data->reportwork, synaptics_s1302_report_work);

#if 1
	if (!exp_data_s1302.initialized) {
		mutex_init(&exp_data_s1302.mutex);
		INIT_LIST_HEAD(&exp_data_s1302.list);
		exp_data_s1302.initialized = true;
	}
    exp_data_s1302.workqueue = create_singlethread_workqueue("dsx1302_exp_workqueue");
    INIT_DELAYED_WORK(&exp_data_s1302.work, synaptics_rmi4_exp_fn_work_s1302);
    exp_data_s1302.rmi4_data = rmi4_data;
    exp_data_s1302.queue_work = true;
    queue_delayed_work(exp_data_s1302.workqueue, &exp_data_s1302.work, msecs_to_jiffies(S1302_WORK_DELAY_MS));
    rmi4_fw_module_init_s1302(true);
    while(1) 
    {
        msleep(50);
        if(rmi4_data->bcontinue) 
        {
            rmi4_fw_module_init_s1302(false);
            break ;
        }
    }
#endif
	synaptics_s1302_proc();
    push_component_info(TOUCH_KEY, "S1302", synaptics_s1302_get_vender());
	retval = synaptics_s1302_irq_enable(rmi4_data, true);
    syna_s1302_data->bremove = 1;
	return 0;

ERR_INPUT:
    input_unregister_device(rmi4_data->input_dev);
    rmi4_data->input_dev = NULL;
ERR_FREE:
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
static struct synaptics_optimize_data optimize_data_s1302;
extern struct synaptics_rmi4_data *syna_rmi4_data;
static void synaptics_s1302_work(struct work_struct *work)
{
	int retval;
	struct i2c_client *client_optimize = optimize_data_s1302.client;
	const struct i2c_device_id *dev_id = optimize_data_s1302.dev_id;

    while(1)
    {
        msleep(100);
        if(NULL == syna_rmi4_data || syna_rmi4_data->bremove)
        {
            retval = synaptics_1302_probe(client_optimize, dev_id);
            break ;
        }
    }
	printk("synaptics s1302 return %d\n", retval);
}


static int synaptics_rmi4_probe_s1302(struct i2c_client *client,
		const struct i2c_device_id *dev_id)
{
    int i, match;
    optimize_data_s1302.workqueue = create_workqueue("optimize_workqueue_s1302");
    INIT_DELAYED_WORK(&(optimize_data_s1302.work), synaptics_s1302_work);
    optimize_data_s1302.client = client;
    optimize_data_s1302.dev_id = dev_id;

    printk("synaptics s1302 probe cpu %d\n", smp_processor_id());
    for (i = 0; i <= 3; i++)
    {
        if (cpu_is_offline(i) || i == smp_processor_id())
        {
            continue;
        }
        queue_delayed_work_on(i, optimize_data_s1302.workqueue,
        &(optimize_data_s1302.work),
        msecs_to_jiffies(1000));
        match = 1;
        printk("synap s1302 work on cpu %d\n",i);
        break;
    }
    if (match == 0)
    {
        queue_delayed_work_on(0, optimize_data_s1302.workqueue,
        &(optimize_data_s1302.work),
        msecs_to_jiffies(1000));
    }
    return 0;
}
#endif
static int  synaptics_1302_remove(struct i2c_client *client)
{
    struct synaptics_rmi4_data *rmi4_data = i2c_get_clientdata(client);
	
	cancel_delayed_work_sync(&exp_data_s1302.work);
	flush_workqueue(exp_data_s1302.workqueue);
	destroy_workqueue(exp_data_s1302.workqueue);

	input_unregister_device(rmi4_data->input_dev);
	kfree(rmi4_data);
	return 0;
}
#ifdef CONFIG_FB   //for tp suspend and resume
static const struct dev_pm_ops synaptics_s1302_pm_ops = {
    .suspend = synaptics_s1302_suspend,
    .resume  = synaptics_s1302_resume,
};
#endif
static const struct i2c_device_id synaptics_1302_id_table[] = {
    {DRIVER_NAME, 0},
    {},
};
MODULE_DEVICE_TABLE(i2c, synaptics_1302_id_table);

static struct of_device_id synaptics_1302_match_table[] = {
    { .compatible = "synaptics,s1302",},
    { },
};

static struct i2c_driver synaptics_rmi4_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
        
#ifdef CONFIG_FB   //for tp suspend and resume
		//.pm = &synaptics_s1302_pm_ops,
#endif
        .of_match_table = synaptics_1302_match_table,
    },
    .probe = synaptics_rmi4_probe_s1302,
    .remove = synaptics_1302_remove,
    .id_table = synaptics_1302_id_table,
};

static int __init synaptics_1302_init(void)
{
    return i2c_add_driver(&synaptics_rmi4_driver);
}

static void __exit synaptics_1302_exit(void)
{
    i2c_del_driver(&synaptics_rmi4_driver);
    return;
}

module_init(synaptics_1302_init);
module_exit(synaptics_1302_exit);

MODULE_DESCRIPTION("Synaptics DSX I2C Touch Driver");
MODULE_LICENSE("GPL");
