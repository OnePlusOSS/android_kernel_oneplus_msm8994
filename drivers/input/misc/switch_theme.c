/*
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>

#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <asm/uaccess.h>

#include <linux/qpnp/qpnp-adc.h>

#include <linux/regulator/consumer.h>

#include <linux/delay.h>


#include <linux/sort.h>
#include <linux/seq_file.h>
#include "../../../fs/proc/internal.h"

#include <linux/wakelock.h>

//#include <linux/irq.h>
#define DRV_NAME	"switch-theme"
/*
* HW SCHEMATIC
*         Cover |   Phone
*               |
*          -----|----R2(2K Ohm)------[3.3 V
*          |    |
*          R1   |
*          |    |
*          |____|___________ADC
*               |       |
*               |       |
*               |       R3(51K ohm)
*               |       |
*               |       = GND
*/

/*
*   cover type | cover resist(K 0hm) | ADC voltage (V)
*       1      |     0               |    3.18
*       2      |     5.6             |    2.87
*       3      |     12              |    2.59
*       4      |     20              |    2.31
*       5      |     30.9            |    2.01
*       6      |     51              |    1.62
*       7      |     82              |    1.25
*       8      |     121             |    0.97
*       9      |     200             |    0.67
*       10     |     470             |    0.32
*       ..     |     ..              |    ...
*
*/

/*procfs enable_switch_theme when change cover feature
 procfs cover_type for boot up to read,when enable,read the type to change theme
*/

//TODO change name to real cover type
typedef enum {
    COVER_TYPE_UNKNOWN,
	COVER_TYPE1,
	COVER_TYPE2,
	COVER_TYPE3,
	COVER_TYPE4,
	COVER_TYPE5,
	COVER_TYPE6,
	COVER_TYPE7,
	COVER_TYPE8,
	COVER_TYPE9,
	COVER_TYPE10,
	COVER_TYPE_MAX_NUM
} cover_t;

struct switch_theme_data {
	cover_t last_type;
	cover_t cover_type;
	int adc_value;//int ?
	int switch_enable;
	int irq;
	int irq_gpio;//gpio66
	int cover_switch_gpio;//pm8994 gpio01
	//ADC channel PM8994 MPP04
	struct qpnp_vadc_chip	*vadc_dev;

	struct regulator *vdd_io;
	bool power_enabled;

	struct work_struct work;
	struct switch_dev sdev;
	struct device *dev;
	//struct input_dev *input;
	struct wake_lock cover_wl;

	struct pinctrl * key_pinctrl;
	struct pinctrl_state * set_state;
};

static struct switch_theme_data *switch_data;
/*
#define SUPPLY_3V3		2950000UL
#define SUPPLY_IO_MIN		SUPPLY_3V3
#define SUPPLY_IO_MAX		SUPPLY_3V3
#define SUPPLY_IO_REQ_CURRENT	6000U

int cover_switch_regulator_release(void)
{



	if (switch_data->vdd_io != NULL) {
		regulator_put(switch_data->vdd_io);
		switch_data->vdd_io = NULL;
	}

	switch_data->power_enabled = false;

	return 0;
}

int cover_switch_regulator_set(bool enable)
{
	int error = 0;

	if (switch_data->vdd_io == NULL) {
		dev_err(switch_data->dev,
			"Regulators not set\n");
			return -EINVAL;
	}

	if (enable) {
		dev_dbg(switch_data->dev, "%s on\n", __func__);

		regulator_set_optimum_mode(switch_data->vdd_io,
					SUPPLY_IO_REQ_CURRENT);

		error = (regulator_is_enabled(switch_data->vdd_io) == 0) ?
					regulator_enable(switch_data->vdd_io) : 0;

		if (error) {
			dev_err(switch_data->dev,
				"Regulator vdd_io enable failed, error=%d\n",
				error);
			goto out_err;
		}

	} else {
		dev_dbg(switch_data->dev, "%s off\n", __func__);

		error = (switch_data->power_enabled &&
			regulator_is_enabled(switch_data->vdd_io) > 0) ?
				 regulator_disable(switch_data->vdd_io) : 0;

		if (error) {
			dev_err(switch_data->dev,
				"Regulator vdd_io disable failed, error=%d\n",
				 error);
			goto out_err;
		}

	}
    switch_data->power_enabled = enable;
	return 0;

out_err:
	cover_switch_regulator_release();
	return error;
}

static int cover_switch_supply_init(void)
{
	int error = 0;

	switch_data->vdd_io = regulator_get(switch_data->dev, "vdd_io");
	if (IS_ERR(switch_data->vdd_io)) {
		error = PTR_ERR(switch_data->vdd_io);
		dev_err(switch_data->dev,
			"Regulator get failed, vdd_io, error=%d\n", error);
		goto err;
	}

	if (regulator_count_voltages(switch_data->vdd_io) > 0) {
		error = regulator_set_voltage(switch_data->vdd_io,
						SUPPLY_IO_MIN, SUPPLY_IO_MAX);
		if (error) {
			dev_err(switch_data->dev,
				"regulator set(io) failed, error=%d\n", error);
			goto err;
		}
	}
	if (error) {
		dev_err(switch_data->dev,
			"regulator configuration failed.\n");
		goto err;
	}

	error = cover_switch_regulator_set(true);
	if (error) {
		dev_err(switch_data->dev,
			"regulator enable failed.\n");
		goto err;
	}

err:
	return error;
}
*/
static int cover_switch_enable_show(struct seq_file *seq, void *offset)
{

 	seq_printf(seq, "%d\n", switch_data->switch_enable);
	return 0;
}

static ssize_t cover_switch_enable_write(struct file *file, const char __user *buffer, size_t count, loff_t *pos)
{
	unsigned int cover_switch;

	if (!switch_data) {
		return -EFAULT;
	}

	if (copy_from_user(&cover_switch, buffer, sizeof(cover_switch)))
		return -EFAULT;

    printk("cover_switch_enable %d change to %d\n",switch_data->switch_enable,cover_switch);

	if(switch_data->switch_enable!= cover_switch) {
	    switch_data->switch_enable = cover_switch;
	}

	return count;
}

static int cover_switch_enable_open(struct inode *inode, struct file *file)
{
	return single_open(file, cover_switch_enable_show, switch_data);
}


static const struct file_operations cover_switch_enable_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= cover_switch_enable_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= cover_switch_enable_write,
};

//add for test
static int cover_type_show(struct seq_file *seq, void *offset)
{
 	seq_printf(seq, "%d\n", switch_data->cover_type);
	return 0;
}

static int cover_type_open(struct inode *inode, struct file *file)
{
	return single_open(file, cover_type_show, switch_data);
}

static ssize_t cover_type_write(struct file *file, const char __user *buffer, size_t count, loff_t *pos)
{
    int state;
	if (!switch_data) {
		return -EFAULT;
	}
    /*
	if (copy_from_user(&cover_switch, buffer, sizeof(cover_switch)))
		return -EFAULT;
    */
    sscanf(buffer, "%d", &state);
    printk("cover_type_write by procfs: (%d) change to (%d)\n",switch_data->cover_type,state);

	if(switch_data->cover_type!= state) {
	    switch_data->last_type = switch_data->cover_type;
	    switch_data->cover_type = state;
	    switch_set_state(&switch_data->sdev, switch_data->cover_type);
	}

	return count;
}

static const struct file_operations cover_type_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= cover_type_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= cover_type_write,
};

static int ADC_show(struct seq_file *seq, void *offset)
{
 	seq_printf(seq, "%d\n", switch_data->adc_value);
	return 0;
}

static int ADC_open(struct inode *inode, struct file *file)
{
	return single_open(file, ADC_show, switch_data);
}


static const struct file_operations ADC_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= ADC_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	//.write		= ADC_write,
};

//#define Test //use gpio34(fingerprint key' gpio) for test,cause now we have no hardware

#ifdef Test //changhua add for test uevent

static void cover_switch_work(struct work_struct *work)
{
	    printk("%s  gpio_get_value(%d)=%d",__func__,switch_data->irq_gpio,gpio_get_value(switch_data->irq_gpio));
	    switch_set_state(&switch_data->sdev, (!!gpio_get_value(switch_data->irq_gpio)?COVER_TYPE1:COVER_TYPE2));
}
irqreturn_t cover_switch_interrupt(int irq, void *_dev)
{
    schedule_work(&switch_data->work);

		return IRQ_HANDLED;
}
#else
//static int voltage_tbl [] = {2840,2570,2310,2060,1790,1450,1110,860,590,290};
static int voltage_tbl [] = {2769,2388,2003,1635,1360,983,626,383};
static DEFINE_MUTEX(mLock);
#define COVER_WAKELOCK_HOLD_TIME 1000
irqreturn_t cover_switch_interrupt(int irq, void *_dev)
{
    wake_lock_timeout(&switch_data->cover_wl, msecs_to_jiffies(COVER_WAKELOCK_HOLD_TIME));
	//if (!gpio_get_value(switch_data->irq_gpio)) {//triger_falling
	    schedule_work(&switch_data->work);
		//return IRQ_HANDLED;
	//}
	return IRQ_NONE;
}
static void cover_switch_work(struct work_struct *work)
{
	int adc;
	int i=0;
	int err=0;
	struct qpnp_vadc_result result;
	mutex_lock(&mLock);
	printk("%s\n",__func__);
	if (gpio_get_value(switch_data->irq_gpio))
	{
	    switch_data->cover_type = COVER_TYPE_UNKNOWN;
	    printk("%s,cover_type(%d) --->(%d)\n",__func__,switch_data->last_type,switch_data->cover_type);
	    switch_data->last_type = switch_data->cover_type;
    	switch_set_state(&switch_data->sdev, switch_data->cover_type);
    	mutex_unlock(&mLock);
    	return;
	}
    disable_irq_nosync(switch_data->irq);
    free_irq(switch_data->irq,switch_data);

    //TODO read ADC & report data
    if(gpio_is_valid(switch_data->cover_switch_gpio))
        gpio_direction_output(switch_data->cover_switch_gpio,1);//switch to read adc channel
    mdelay(100);//wait for ADC stable
    //read adc
    //Read the VADC channel using the QPNP-ADC-API.
    err = qpnp_vadc_read(switch_data->vadc_dev, P_MUX4_1_3, &result); //Read the MPP4 VADC channel with 1:3 scaling
    if (err) {
		printk("%s,Unable to read cover adc, rc=%d\n",__func__,err);

	}
	printk("%s,read cover adc =%lld uV\n",__func__,result.physical);
    adc = (int) result.physical;
    adc = adc / 1000; /* uV to mV */
    printk("%s,read cover adc =%d mV\n",__func__,adc);
    switch_data->adc_value = adc;
    for(i=1;i<(sizeof(voltage_tbl)/sizeof(voltage_tbl[0]));i++)
    {
        if(switch_data->adc_value >= voltage_tbl[i])
        {
            if((switch_data->adc_value - voltage_tbl[i]) < (voltage_tbl[i-1] - switch_data->adc_value))
                switch_data->cover_type = (cover_t)(i+1);
            else
                switch_data->cover_type = (cover_t)(i);
            break;
        }
    }
    if(switch_data->adc_value > voltage_tbl[0])
        switch_data->cover_type = COVER_TYPE1;
    if(i >= (sizeof(voltage_tbl)/sizeof(voltage_tbl[0])))
        switch_data->cover_type = (cover_t)(i);//COVER_TYPE10;
    if(switch_data->adc_value < 20)
        switch_data->cover_type = COVER_TYPE_UNKNOWN;
    printk("%s,cover_type(%d) --->(%d)\n",__func__,switch_data->last_type,switch_data->cover_type);
    if(switch_data->last_type != switch_data->cover_type)
    {
        switch_data->last_type = switch_data->cover_type;
    	switch_set_state(&switch_data->sdev, switch_data->cover_type);
    }

    //switch to INT channel after read adc to detect next changing of cover
    if(gpio_is_valid(switch_data->cover_switch_gpio))
        gpio_direction_output(switch_data->cover_switch_gpio,0);//switch to INT
    mdelay(1);
    err = request_irq(switch_data->irq, cover_switch_interrupt,
			IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "covor_switch", switch_data);
	//to check whether cover is leave when we check type done,cause we have disable irq,so when cover leave here will not have interrupt to tell us
	if (gpio_get_value(switch_data->irq_gpio))
	{
	    switch_data->cover_type = COVER_TYPE_UNKNOWN;
	    printk("%s,cover is leave when we check type,cover_type(%d) --->(%d)\n",__func__,switch_data->last_type,switch_data->cover_type);
	    switch_data->last_type = switch_data->cover_type;
    	switch_set_state(&switch_data->sdev, switch_data->cover_type);
	}
	printk("%s OK\n",__func__);
	mutex_unlock(&mLock);
}


#endif
/* //no need cause switch_class.c state_show()
static ssize_t cover_switch_print_state(struct switch_dev *sdev, char *buf)
{
	cover_t state;
		state = switch_data->cover_type;

	if (state)
		return sprintf(buf, "%d\n", state);
	return -1;
}
*/
#ifdef CONFIG_OF
static int switch_theme_get_devtree_pdata(struct device *dev)
{
	struct device_node *node;

	node = dev->of_node;
	if (!node)
		return -EINVAL;

printk("%s, node name:%s  , %s\n",__func__,node->name,node->full_name);
	switch_data->irq_gpio= of_get_named_gpio(node, "cover,gpio_irq", 0);
	if ((!gpio_is_valid(switch_data->irq_gpio)))
		return -EINVAL;

printk("%s, irq gpio:%d \n", __func__, switch_data->irq_gpio);

	switch_data->cover_switch_gpio= of_get_named_gpio(node, "cover,gpio_switch", 0);
	if ((!gpio_is_valid(switch_data->cover_switch_gpio)))
		return -EINVAL;
printk("%s, switch gpio:%d \n", __func__, switch_data->cover_switch_gpio);
	return 0;
}

static struct of_device_id switch_theme_of_match[] = {
	{ .compatible = "oneplus,switch-theme", },
	{ },
};
MODULE_DEVICE_TABLE(of, switch_theme_of_match);

#else

static inline int
switch_theme_get_devtree_pdata(struct device *dev)
{
	return 0;
}
#endif

static int switch_theme_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	int error=0;

printk("%s\n",__func__);

        switch_data = kzalloc(sizeof(struct switch_theme_data), GFP_KERNEL);
        switch_data->dev = dev;
        switch_data->last_type = COVER_TYPE_UNKNOWN;


        /* early for VADC get, defer probe if needed */
	    switch_data->vadc_dev = qpnp_get_vadc(dev, "cover");
    	if (IS_ERR(switch_data->vadc_dev)) {
    		error = PTR_ERR(switch_data->vadc_dev);
    		if (error == -EPROBE_DEFER)
			    printk("%s, vadc not found - defer rc=%d\n",__func__, error);
		    else
			    printk("%s, vadc property missing, rc=%d\n",__func__, error);
    		goto err_switch_dev_register;;
    	}
/*
    switch_data->dev = dev;

    switch_data->input = input_allocate_device();

	switch_data->input->name = pdev->name;
	switch_data->input->phys = DRV_NAME"/input0";
	switch_data->input->dev.parent = &pdev->dev;

	switch_data->input->id.bustype = BUS_HOST;
	switch_data->input->id.vendor = 0x0001;
	switch_data->input->id.product = 0x0001;
	switch_data->input->id.version = 0x0100;

    input_set_drvdata(switch_data->input, switch_data);

	__set_bit(EV_KEY, switch_data->input->evbit);
 */
 //changhua remove for L21 always-on
 /*
        error = cover_switch_supply_init();
        if (error) {
			dev_err(dev, "parse device tree fail!!!\n");
			goto err_regulator;
		}
*/

        //parse device tree node
		error = switch_theme_get_devtree_pdata(dev);
		if (error) {
			dev_err(dev, "parse device tree fail!!!\n");
			goto err_switch_dev_register;
		}

	    switch_data->key_pinctrl = devm_pinctrl_get(switch_data->dev);
         if (IS_ERR_OR_NULL(switch_data->key_pinctrl)) {
		        dev_err(switch_data->dev, "Failed to get pinctrl \n");
		        goto err_switch_dev_register;
	     }
         switch_data->set_state =pinctrl_lookup_state(switch_data->key_pinctrl,"cover_int_active");
         if (IS_ERR_OR_NULL(switch_data->set_state)) {
		        dev_err(switch_data->dev, "Failed to lookup_state \n");
		        goto err_switch_dev_register;
	     }
	     pinctrl_select_state(switch_data->key_pinctrl, switch_data->set_state);

        //config cover_switch gpio(pm8994 gpio01) to output low to switch to INT_R,not ADC(pm8994 mpp04)
            error = gpio_request(switch_data->cover_switch_gpio,"cover_switch-ADC-INT");
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_request, err=%d", __func__, error);
        		goto err_switch_dev_register;

        	}
        	error = gpio_direction_output(switch_data->cover_switch_gpio,0);
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_direction_output, err=%d", __func__, error);
        		return error;

            }

       //config irq gpio and request irq
	   switch_data->irq = gpio_to_irq(switch_data->irq_gpio);
       if (switch_data->irq <= 0)
       {
            printk("%s, irq number is not specified, irq #= %d, int pin=%d\n\n", __func__, switch_data->irq, switch_data->irq_gpio);
            goto err_detect_irq_num_failed;
       }
       else
       {
            error = gpio_request(switch_data->irq_gpio,"switch_theme-int");
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_request, err=%d", __func__, error);
        		//return retval;
        		goto err_request_gpio;
        	}
        	error = gpio_direction_input(switch_data->irq_gpio);
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_direction_input, err=%d", __func__, error);
        		//return retval;
        		goto err_set_gpio_input;
        	}
        	printk("%s  gpio_get_value(%d)=%d",__func__,switch_data->irq_gpio,gpio_get_value(switch_data->irq_gpio));
            #ifdef Test
        	error = request_irq(switch_data->irq, cover_switch_interrupt,
			IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "covor_switch", switch_data);
            #else
            error = request_irq(switch_data->irq, cover_switch_interrupt,
			IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "covor_switch", switch_data);
			#endif
        	if (error) {
        		dev_err(dev,
        			"request_irq %i failed.\n",
        			switch_data->irq);

        		switch_data->irq = -EINVAL;
        		goto err_request_irq;
            }
       }
       enable_irq_wake(switch_data->irq);
       wake_lock_init(&switch_data->cover_wl, WAKE_LOCK_SUSPEND, "cover_switch_wl");
       device_init_wakeup(switch_data->dev, 1);

       if (!proc_create_data("cover_type", 0660, NULL, &cover_type_proc_fops, switch_data)) {
    		error = -ENOMEM;
    		goto err_request_irq;
	    }

        INIT_WORK(&switch_data->work, cover_switch_work);
        switch_data->sdev.name = DRV_NAME;
        //switch_data->sdev.print_state = cover_switch_print_state; //no need cause switch_class.c state_show()
       	error = switch_dev_register(&switch_data->sdev);
	    if (error < 0)
		    goto err_request_gpio;
        //report the first switch
        cover_switch_work(&switch_data->work);
        return 0;


err_request_gpio:
	switch_dev_unregister(&switch_data->sdev);
err_request_irq:
err_detect_irq_num_failed:
err_set_gpio_input:
	gpio_free(switch_data->cover_switch_gpio);
	gpio_free(switch_data->irq_gpio);
 //changhua remove for L21 always-on
 /*
err_regulator:
	cover_switch_regulator_release();
*/
err_switch_dev_register:
	kfree(switch_data);

	return error;
}

static int switch_theme_remove(struct platform_device *pdev)
{
printk("%s\n",__func__);
	cancel_work_sync(&switch_data->work);
	gpio_free(switch_data->cover_switch_gpio);
	gpio_free(switch_data->irq_gpio);
	switch_dev_unregister(&switch_data->sdev);
	kfree(switch_data);

	return 0;
}

static struct platform_driver switch_theme_driver = {
	.probe	= switch_theme_probe,
	.remove	= switch_theme_remove,
	.driver	= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(switch_theme_of_match),
	},
};
module_platform_driver(switch_theme_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("switch theme by rare cover type driver");
