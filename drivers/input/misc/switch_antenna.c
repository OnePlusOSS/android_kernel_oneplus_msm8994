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

#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>

#include <linux/sort.h>
#include <linux/seq_file.h>
#include "../../../fs/proc/internal.h"

#define DRV_NAME	"switch_antenna"
#define CLASS_NAME	"sw_antenna"
#define DEV_NAME	"sw_antenna"
#define CDEV_NAME	"sw_antenna"

//#define BUILD_MODULE

struct antenna_dev_data {


	int antenna1_gpio;
	int antenna2_gpio;
	int antenna3_gpio;
	int antenna4_gpio;

	struct device *dev;
	int state;
	
	/*
	struct device          *device;
	
	dev_t                  devno;
	*/
	//struct input_dev *input;
};

static struct antenna_dev_data *antenna_data;
/*
static int antenna_device_count;
static struct class *antenna_class;
static ssize_t antenna_state_store(struct device *dev, struct device_attribute *attr, 
		const char *buf, size_t count)
{
	char *after;
    unsigned long val = simple_strtoul(buf, &after, 10);
    //int rc = -1;
printk("%s : enter val=(%d)\n",__func__,val);
	switch(val)
	{
        case 0:
            gpio_direction_output(antenna_data->antenna1_gpio,0);
            gpio_direction_output(antenna_data->antenna2_gpio,0);
            gpio_direction_output(antenna_data->antenna3_gpio,0);
            gpio_direction_output(antenna_data->antenna4_gpio,0);
            break;
        case 1:
            gpio_direction_output(antenna_data->antenna1_gpio,1);
            gpio_direction_output(antenna_data->antenna2_gpio,0);
            gpio_direction_output(antenna_data->antenna3_gpio,0);
            gpio_direction_output(antenna_data->antenna4_gpio,0);
            break;
        case 2:
            gpio_direction_output(antenna_data->antenna1_gpio,0);
            gpio_direction_output(antenna_data->antenna2_gpio,1);
            gpio_direction_output(antenna_data->antenna3_gpio,0);
            gpio_direction_output(antenna_data->antenna4_gpio,0);
            break;
        case 3:
            gpio_direction_output(antenna_data->antenna1_gpio,0);
            gpio_direction_output(antenna_data->antenna2_gpio,0);
            gpio_direction_output(antenna_data->antenna3_gpio,1);
            gpio_direction_output(antenna_data->antenna4_gpio,0);
            break;
        case 4:
            gpio_direction_output(antenna_data->antenna1_gpio,0);
            gpio_direction_output(antenna_data->antenna2_gpio,0);
            gpio_direction_output(antenna_data->antenna3_gpio,0);
            gpio_direction_output(antenna_data->antenna4_gpio,1);
            break; 
        default:
            gpio_direction_output(antenna_data->antenna1_gpio,0);
            gpio_direction_output(antenna_data->antenna2_gpio,0);
            gpio_direction_output(antenna_data->antenna3_gpio,0);
            gpio_direction_output(antenna_data->antenna4_gpio,0);
            break;
        
	}
	return count;
}
static ssize_t antenna_state_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	int val=0;
	if(gpio_direction_input(antenna_data->antenna1_gpio))
	{
	    val=1;
		return sprintf(buf, "%d\n", val);
    }
	if(gpio_direction_input(antenna_data->antenna2_gpio))
	{
	    val=2;
		return sprintf(buf, "%d\n", val);
    }
    if(gpio_direction_input(antenna_data->antenna3_gpio))
	{
	    val=3;
		return sprintf(buf, "%d\n", val);
    }
    if(gpio_direction_input(antenna_data->antenna4_gpio))
	{
	    val=4;
		return sprintf(buf, "%d\n", val);
    }

    val=0;
	return sprintf(buf, "%d\n", val);
    
}

static struct device_attribute antenna_state_attr =
    __ATTR(antenna_state, 0666, antenna_state_show, antenna_state_store);
*/
static DEFINE_MUTEX(sem);  
static int antenna_state_show(struct seq_file *seq, void *offset)
{
	
 	seq_printf(seq, "%d\n", antenna_data->state);
	return 0;
}

static ssize_t antenna_state_write(struct file *file, const char __user *buffer, size_t count, loff_t *pos)
{
	//char* after; 
    //char* buf;
    int state;
    int enable;
	if (!antenna_data) {
		return -EFAULT;
	}
	mutex_lock(&sem); 
/*
	if (copy_from_user(buf, buffer, count)) 
		return -EFAULT;
    state = simple_strtoul(buf, &after, 10);
*/    
    sscanf(buffer, "%d", &state);
    
    printk("antenna_state before(0x%x),enter(%d),count=%zu \n",antenna_data->state,state,count);

    enable = state%10;
    state = state/10;
    if(enable!=0)
    {
	//if(antenna_data->state!= state) {
    	if(state == 0)
    	    antenna_data->state = 0;
    	else
    	    antenna_data->state |= (1<< (state-1));
        switch(state)
    	{
            case 0:
                gpio_direction_output(antenna_data->antenna1_gpio,0);
                gpio_direction_output(antenna_data->antenna2_gpio,0);
                gpio_direction_output(antenna_data->antenna3_gpio,0);
                gpio_direction_output(antenna_data->antenna4_gpio,0);
                break;
            case 1:
                gpio_direction_output(antenna_data->antenna1_gpio,1);
                /*
                gpio_direction_output(antenna_data->antenna2_gpio,0);
                gpio_direction_output(antenna_data->antenna3_gpio,0);
                gpio_direction_output(antenna_data->antenna4_gpio,0);
                */
                break;
            case 2:
                //gpio_direction_output(antenna_data->antenna1_gpio,0);
                gpio_direction_output(antenna_data->antenna2_gpio,1);
                //gpio_direction_output(antenna_data->antenna3_gpio,0);
                //gpio_direction_output(antenna_data->antenna4_gpio,0);
                break;
            case 3:
                //gpio_direction_output(antenna_data->antenna1_gpio,0);
                //gpio_direction_output(antenna_data->antenna2_gpio,0);
                gpio_direction_output(antenna_data->antenna3_gpio,1);
                //gpio_direction_output(antenna_data->antenna4_gpio,0);
                break;
            case 4:
                /*
                gpio_direction_output(antenna_data->antenna1_gpio,0);
                gpio_direction_output(antenna_data->antenna2_gpio,0);
                gpio_direction_output(antenna_data->antenna3_gpio,0);
                */
                gpio_direction_output(antenna_data->antenna4_gpio,1);
                break; 
            default:
                gpio_direction_output(antenna_data->antenna1_gpio,0);
                gpio_direction_output(antenna_data->antenna2_gpio,0);
                gpio_direction_output(antenna_data->antenna3_gpio,0);
                gpio_direction_output(antenna_data->antenna4_gpio,0);
                break;
            
    	}

	    
	    
	//}
	}
	else
	{
	//if(antenna_data->state!= state) {
    	if(state == 0)
    	    antenna_data->state = 0;
    	else
    	    antenna_data->state &= ~(1<< (state-1));
        switch(state)
    	{
            case 0:
                gpio_direction_output(antenna_data->antenna1_gpio,0);
                gpio_direction_output(antenna_data->antenna2_gpio,0);
                gpio_direction_output(antenna_data->antenna3_gpio,0);
                gpio_direction_output(antenna_data->antenna4_gpio,0);
                break;
            case 1:
                gpio_direction_output(antenna_data->antenna1_gpio,0);
                /*
                gpio_direction_output(antenna_data->antenna2_gpio,0);
                gpio_direction_output(antenna_data->antenna3_gpio,0);
                gpio_direction_output(antenna_data->antenna4_gpio,0);
                */
                break;
            case 2:
                //gpio_direction_output(antenna_data->antenna1_gpio,0);
                gpio_direction_output(antenna_data->antenna2_gpio,0);
                //gpio_direction_output(antenna_data->antenna3_gpio,0);
                //gpio_direction_output(antenna_data->antenna4_gpio,0);
                break;
            case 3:
                //gpio_direction_output(antenna_data->antenna1_gpio,0);
                //gpio_direction_output(antenna_data->antenna2_gpio,0);
                gpio_direction_output(antenna_data->antenna3_gpio,0);
                //gpio_direction_output(antenna_data->antenna4_gpio,0);
                break;
            case 4:
                /*
                gpio_direction_output(antenna_data->antenna1_gpio,0);
                gpio_direction_output(antenna_data->antenna2_gpio,0);
                gpio_direction_output(antenna_data->antenna3_gpio,0);
                */
                gpio_direction_output(antenna_data->antenna4_gpio,0);
                break; 
            default:
                gpio_direction_output(antenna_data->antenna1_gpio,0);
                gpio_direction_output(antenna_data->antenna2_gpio,0);
                gpio_direction_output(antenna_data->antenna3_gpio,0);
                gpio_direction_output(antenna_data->antenna4_gpio,0);
                break;
            
    	}

	    
	    
	//}
	}
    printk("antenna_state change to (0x%x) \n",antenna_data->state);
    mutex_unlock(&sem);
	return count;
}

static int antenna_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, antenna_state_show, antenna_data);
}


static const struct file_operations antenna_state_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= antenna_state_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= antenna_state_write,
};

#ifndef BUILD_MODULE
#ifdef CONFIG_OF
static int antenna_dev_get_devtree_pdata(struct device *dev)
{
	struct device_node *node;

	node = dev->of_node;
	if (!node)
		return -EINVAL;
	
	antenna_data->antenna2_gpio= of_get_named_gpio(node, "antenna,antenna2_gpio", 0);
	if ((!gpio_is_valid(antenna_data->antenna2_gpio)))
		return -EINVAL;
		


	antenna_data->antenna1_gpio= of_get_named_gpio(node, "antenna,antenna1_gpio", 0);
	if ((!gpio_is_valid(antenna_data->antenna1_gpio)))
		return -EINVAL;
		
    antenna_data->antenna3_gpio= of_get_named_gpio(node, "antenna,antenna3_gpio", 0);
	if ((!gpio_is_valid(antenna_data->antenna2_gpio)))
		return -EINVAL;
		


	antenna_data->antenna4_gpio= of_get_named_gpio(node, "antenna,antenna4_gpio", 0);
	if ((!gpio_is_valid(antenna_data->antenna1_gpio)))
		return -EINVAL;
		
	return 0;	
}

static struct of_device_id antenna_of_match[] = {
	{ .compatible = "oneplus,switch-antenna", },
	{ },
};
MODULE_DEVICE_TABLE(of, antenna_of_match);

#else

static int
antenna_dev_get_devtree_pdata(struct device *dev)
{
	return 0;
}
#endif
#endif

static int antenna_dev_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
    
	int error=0;
	
printk("%s\n",__func__);

        antenna_data = kzalloc(sizeof(struct antenna_dev_data), GFP_KERNEL);
        antenna_data->dev = dev;

/*
        if (!antenna_class) {
    		antenna_class = class_create(THIS_MODULE, CLASS_NAME);
    		if (IS_ERR(antenna_class))
    			error =  PTR_ERR(antenna_class);
    			//dev_err(dev,"%s: class_create FAILED %d.\n", __func__, error);
    			//goto err_antenna_dev_register;
	    }
	    
	    error = alloc_chrdev_region(&antenna_data->devno,
					antenna_device_count++,
					1,
					DRV_NAME);

	if (error < 0) {
		dev_err(dev,
				"%s: alloc_chrdev_region FAILED %d.\n", __func__, error);
		goto err_class;

	} else {
		dev_info(dev, "%s: major=%d, minor=%d\n",
						__func__,
						MAJOR(antenna_data->devno),
						MINOR(antenna_data->devno));
	}
	antenna_data->device = device_create(antenna_class, NULL, antenna_data->devno,
						NULL, "%s", CDEV_NAME);

	if (IS_ERR(antenna_data->device)) {
		dev_err(dev, "%s: device_create failed.\n",__func__);
		error = PTR_ERR(antenna_data->device);
		goto err_class;
	}
	
	    error =	sysfs_create_file(&antenna_data->device->kobj, &antenna_state_attr.attr);
        //error =	sysfs_create_file(&antenna_data->dev->kobj, &antenna_state_attr.attr);
*/        
       #ifndef BUILD_MODULE
      //parse device tree node
		error = antenna_dev_get_devtree_pdata(dev);
	   #else //TODO  modify gpio number by hw
         antenna_data->antenna1_gpio = 998;//34-->912  66-->944
         antenna_data->antenna1_gpio = 999;
         antenna_data->antenna1_gpio = 1000;
         antenna_data->antenna1_gpio = 1002;
         //TODO maybe need to use pin-ctrl if that gpios are not in gpio mode,
         //then to do in .ko will use ioremap instead of pin-ctrl to write the register(0xfd511xxx),
         //register bit[0]=
         //0:mode
         //4:in out status 0 1
         //8:INTR status  9f 
	   #endif	
		if (error) {
			dev_err(dev, "parse device tree fail!!!\n");
			//goto err_device;
			goto err_antenna_dev_register;
		}
        if (!proc_create_data("antenna_switch", 0666, NULL, &antenna_state_proc_fops, antenna_data)) {
    		error = -ENOMEM;
    		goto err_antenna_dev_register;
	    }
		
        	error = gpio_request(antenna_data->antenna1_gpio,"antenna1_gpio");        
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_request, err=%d", __func__, error);
        		//return retval;
        		goto err_set_gpio;
        	}
        	error = gpio_direction_output(antenna_data->antenna1_gpio,0);
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_direction_output, err=%d", __func__, error);
        		//return retval;
        		goto err_set_gpio;
        	}

              
        	error = gpio_request(antenna_data->antenna2_gpio,"antenna2_gpio");        
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_request, err=%d", __func__, error);
        		//return retval;
        		goto err_set_gpio;
        	}
        	error = gpio_direction_output(antenna_data->antenna2_gpio,0);
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_direction_output, err=%d", __func__, error);
        		//return retval;
        		goto err_set_gpio;
        	}
        	
            error = gpio_request(antenna_data->antenna3_gpio,"antenna3_gpio");        
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_request, err=%d", __func__, error);
        		//return retval;
        		goto err_set_gpio;
        	}
        	error = gpio_direction_output(antenna_data->antenna3_gpio,0);
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_direction_output, err=%d", __func__, error);
        		//return retval;
        		goto err_set_gpio;
        	}

            error = gpio_request(antenna_data->antenna4_gpio,"antenna4_gpio");        
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_request, err=%d", __func__, error);
        		//return retval;
        		goto err_set_gpio;
        	}
        	error = gpio_direction_output(antenna_data->antenna4_gpio,0);
        	if(error < 0)
        	{
        		printk(KERN_ERR "%s: gpio_direction_output, err=%d", __func__, error);
        		//return retval;
        		goto err_set_gpio;
        	}
        printk("%s  ok!\n",__func__);	
        return 0;



err_set_gpio:
if (gpio_is_valid(antenna_data->antenna1_gpio))
	gpio_free(antenna_data->antenna1_gpio);
if (gpio_is_valid(antenna_data->antenna2_gpio))	
	gpio_free(antenna_data->antenna2_gpio);
if (gpio_is_valid(antenna_data->antenna3_gpio))	
	gpio_free(antenna_data->antenna3_gpio);
if (gpio_is_valid(antenna_data->antenna4_gpio))	
	gpio_free(antenna_data->antenna4_gpio);

/*
err_device:
 if (!IS_ERR_OR_NULL(antenna_data->device))
    		device_destroy(antenna_class, antenna_data->devno);

err_class:
class_destroy(antenna_class);
		antenna_class = NULL;	
*/		
err_antenna_dev_register:
	kfree(antenna_data);	

	return error;
}

static int antenna_dev_remove(struct platform_device *pdev)
{
printk("%s\n",__func__);

    if (gpio_is_valid(antenna_data->antenna1_gpio))
    	gpio_free(antenna_data->antenna1_gpio);
    if (gpio_is_valid(antenna_data->antenna2_gpio))	
    	gpio_free(antenna_data->antenna2_gpio);
    if (gpio_is_valid(antenna_data->antenna3_gpio))	
    	gpio_free(antenna_data->antenna3_gpio);
    if (gpio_is_valid(antenna_data->antenna4_gpio))	
    	gpio_free(antenna_data->antenna4_gpio);

    	/*
    if (!IS_ERR_OR_NULL(antenna_data->device))
    		device_destroy(antenna_class, antenna_data->devno);
    		
    class_destroy(antenna_class);
    antenna_class = NULL;
        */
	kfree(antenna_data);

	return 0;
}

#ifdef BUILD_MODULE
static struct platform_device antenna_device = {  
        .name = DRV_NAME,  
        .id = -1,  
};  

static struct platform_driver antenna_dev_driver = {
	.probe	= antenna_dev_probe,
	.remove	= antenna_dev_remove,
	.driver	= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init switch_antenna_init(void)
{
    
    platform_device_register(&antenna_device); 
    
    
    return = platform_driver_register(&antenna_dev_driver);
}



static void __exit switch_antenna_exit(void)
{
	printk(KERN_INFO "%s\n", __func__);
    platform_driver_unregister(&antenna_device);
    
	platform_driver_unregister(&antenna_dev_driver);
}
module_init(switch_antenna_init);
module_exit(switch_antenna_exit);

#else

static struct platform_driver antenna_dev_driver = {
	.probe	= antenna_dev_probe,
	.remove	= antenna_dev_remove,
	.driver	= {
		.name	= DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(antenna_of_match),
	},
};
module_platform_driver(antenna_dev_driver);
#endif
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("switch antenna by gpio's driver");

