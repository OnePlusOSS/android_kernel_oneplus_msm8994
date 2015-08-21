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
#include <linux/firmware.h>
#include "synaptics_dsx.h"
#include "synaptics_dsx_i2c.h"
#include "synaptics_firmware_jdi_14049.h"
#include "synaptics_firmware_s1302.h"

#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>

#define FW_IMAGE_NAME "synaptics/startup_fw_update.img"
static char s1302_vendor_str[32];  //vendor string

#define DO_STARTUP_FW_UPDATE
#define STARTUP_FW_UPDATE_DELAY_MS 200 /* ms */
#define FORCE_UPDATE false
#define DO_LOCKDOWN false

#define MAX_IMAGE_NAME_LEN 256
#define MAX_FIRMWARE_ID_LEN 10

#define LOCKDOWN_OFFSET 0xb0
#define FW_IMAGE_OFFSET 0x100

#define BOOTLOADER_ID_OFFSET 0
#define BLOCK_NUMBER_OFFSET 0

#define V5_PROPERTIES_OFFSET 2
#define V5_BLOCK_SIZE_OFFSET 3
#define V5_BLOCK_COUNT_OFFSET 5
#define V5_BLOCK_DATA_OFFSET 2

#define V6_PROPERTIES_OFFSET 1
#define V6_BLOCK_SIZE_OFFSET 2
#define V6_BLOCK_COUNT_OFFSET 3
#define V6_BLOCK_DATA_OFFSET 1
#define V6_FLASH_COMMAND_OFFSET 2
#define V6_FLASH_STATUS_OFFSET 3

#define LOCKDOWN_BLOCK_COUNT 5

#define REG_MAP (1 << 0)
#define UNLOCKED (1 << 1)
#define HAS_CONFIG_ID (1 << 2)
#define HAS_PERM_CONFIG (1 << 3)
#define HAS_BL_CONFIG (1 << 4)
#define HAS_DISP_CONFIG (1 << 5)
#define HAS_CTRL1 (1 << 6)

#define UI_CONFIG_AREA 0x00
#define PERM_CONFIG_AREA 0x01
#define BL_CONFIG_AREA 0x02
#define DISP_CONFIG_AREA 0x03

#define CMD_WRITE_FW_BLOCK 0x2
#define CMD_ERASE_ALL 0x3
#define CMD_WRITE_LOCKDOWN_BLOCK 0x4
#define CMD_READ_CONFIG_BLOCK 0x5
#define CMD_WRITE_CONFIG_BLOCK 0x6
#define CMD_ERASE_CONFIG 0x7
#define CMD_ERASE_BL_CONFIG 0x9
#define CMD_ERASE_DISP_CONFIG 0xa
#define CMD_ENABLE_FLASH_PROG 0xf

#define SLEEP_MODE_NORMAL (0x00)
#define SLEEP_MODE_SENSOR_SLEEP (0x01)
#define SLEEP_MODE_RESERVED0 (0x02)
#define SLEEP_MODE_RESERVED1 (0x03)

#define ENABLE_WAIT_MS (1 * 1000)
#define WRITE_WAIT_MS (1)
#define ERASE_WAIT_MS (5 * 1000)

#define MIN_SLEEP_TIME_US 50
#define MAX_SLEEP_TIME_US 100

static ssize_t fwu_sysfs_show_image(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static ssize_t fwu_sysfs_store_image(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static ssize_t fwu_sysfs_do_reflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_write_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_read_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_config_area_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_image_name_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_image_size_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_block_size_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_firmware_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_configuration_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_perm_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_bl_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_disp_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

enum bl_version {
	V5 = 5,
	V6 = 6,
};

enum flash_area {
	NONE,
	UI_FIRMWARE,
	CONFIG_AREA,
};

enum update_mode {
	NORMAL = 1,
	FORCE = 2,
	LOCKDOWN = 8,
};

struct image_header {
	/* 0x00 - 0x0f */
	unsigned char checksum[4];
	unsigned char reserved_04;
	unsigned char reserved_05;
	unsigned char options_firmware_id:1;
	unsigned char options_contain_bootloader:1;
	unsigned char options_reserved:6;
	unsigned char bootloader_version;
	unsigned char firmware_size[4];
	unsigned char config_size[4];
	/* 0x10 - 0x1f */
	unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE];
	unsigned char package_id[2];
	unsigned char package_id_revision[2];
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
	/* 0x20 - 0x2f */
	unsigned char reserved_20_2f[16];
	/* 0x30 - 0x3f */
	unsigned char ds_id[16];
	/* 0x40 - 0x4f */
	unsigned char ds_info[10];
	unsigned char reserved_4a_4f[6];
	/* 0x50 - 0x53 */
	unsigned char firmware_id[4];
};

struct image_header_data {
	bool contains_firmware_id;
	unsigned int firmware_id;
	unsigned int checksum;
	unsigned int firmware_size;
	unsigned int config_size;
	unsigned char bootloader_version;
	unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
};

struct pdt_properties {
	union {
		struct {
			unsigned char reserved_1:6;
			unsigned char has_bsr:1;
			unsigned char reserved_2:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f01_device_status {
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

struct f01_device_control {
	union {
		struct {
			unsigned char sleep_mode:2;
			unsigned char nosleep:1;
			unsigned char reserved:2;
			unsigned char charger_connected:1;
			unsigned char report_rate:1;
			unsigned char configured:1;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_fwu_handle {
	enum bl_version bl_version;
	bool initialized;
	bool program_enabled;
	bool has_perm_config;
	bool has_bl_config;
	bool has_disp_config;
	bool force_update;
	bool in_flash_prog_mode;
	bool do_lockdown;
	unsigned int data_pos;
	unsigned int image_size;
	unsigned char *image_name;
	unsigned char *ext_data_source;
	unsigned char *read_config_buf;
	unsigned char intr_mask;
	unsigned char command;
	unsigned char bootloader_id[2];
	unsigned char flash_properties;
	unsigned char flash_status;
	unsigned char productinfo1;
	unsigned char productinfo2;
	unsigned char properties_off;
	unsigned char blk_size_off;
	unsigned char blk_count_off;
	unsigned char blk_data_off;
	unsigned char flash_cmd_off;
	unsigned char flash_status_off;
	unsigned short block_size;
	unsigned short fw_block_count;
	unsigned short config_block_count;
	unsigned short lockdown_block_count;
	unsigned short perm_config_block_count;
	unsigned short bl_config_block_count;
	unsigned short disp_config_block_count;
	unsigned short config_size;
	unsigned short config_area;
	char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];
	const unsigned char *firmware_data;
	const unsigned char *config_data;
	const unsigned char *lockdown_data;
	struct workqueue_struct *fwu_workqueue;
	struct delayed_work fwu_work;
	struct synaptics_rmi4_fn_desc f01_fd;
	struct synaptics_rmi4_fn_desc f34_fd;
	struct synaptics_rmi4_exp_fn_ptr *fn_ptr;
	struct synaptics_rmi4_data *rmi4_data;
};

static struct bin_attribute dev_attr_data = {
	.attr = {
		.name = "data",
		.mode = (S_IWUSR | S_IRUSR),
	},
	.size = 0,
	.read = fwu_sysfs_show_image,
	.write = fwu_sysfs_store_image,
};

static struct device_attribute attrs[] = {
	__ATTR(doreflash, S_IWUSR,
			synaptics_rmi4_show_error,
			fwu_sysfs_do_reflash_store),
	__ATTR(writeconfig, S_IWUSR,
			synaptics_rmi4_show_error,
			fwu_sysfs_write_config_store),
	__ATTR(readconfig, S_IWUSR,
			synaptics_rmi4_show_error,
			fwu_sysfs_read_config_store),
	__ATTR(configarea, S_IWUSR,
			synaptics_rmi4_show_error,
			fwu_sysfs_config_area_store),
	__ATTR(imagename, S_IWUSR,
			synaptics_rmi4_show_error,
			fwu_sysfs_image_name_store),
	__ATTR(imagesize, S_IWUSR,
			synaptics_rmi4_show_error,
			fwu_sysfs_image_size_store),
	__ATTR(blocksize, S_IRUGO,
			fwu_sysfs_block_size_show,
			synaptics_rmi4_store_error),
	__ATTR(fwblockcount, S_IRUGO,
			fwu_sysfs_firmware_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(configblockcount, S_IRUGO,
			fwu_sysfs_configuration_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(permconfigblockcount, S_IRUGO,
			fwu_sysfs_perm_config_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(blconfigblockcount, S_IRUGO,
			fwu_sysfs_bl_config_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(dispconfigblockcount, S_IRUGO,
			fwu_sysfs_disp_config_block_count_show,
			synaptics_rmi4_store_error),
};

static struct synaptics_rmi4_fwu_handle *fwu;

DECLARE_COMPLETION(fwu_remove_complete);

static unsigned int extract_uint_le(const unsigned char *ptr)
{
	return (unsigned int)ptr[0] +
			(unsigned int)ptr[1] * 0x100 +
			(unsigned int)ptr[2] * 0x10000 +
			(unsigned int)ptr[3] * 0x1000000;
}

static unsigned int extract_uint_be(const unsigned char *ptr)
{
	return (unsigned int)ptr[3] +
			(unsigned int)ptr[2] * 0x100 +
			(unsigned int)ptr[1] * 0x10000 +
			(unsigned int)ptr[0] * 0x1000000;
}

static void parse_header(struct image_header_data *header,
		const unsigned char *fw_image)
{
	struct image_header *data = (struct image_header *)fw_image;

	header->checksum = extract_uint_le(data->checksum);

	header->bootloader_version = data->bootloader_version;

	header->firmware_size = extract_uint_le(data->firmware_size);

	header->config_size = extract_uint_le(data->config_size);

	memcpy(header->product_id, data->product_id, sizeof(data->product_id));
	header->product_id[sizeof(data->product_id)] = 0;

	memcpy(header->product_info, data->product_info,
			sizeof(data->product_info));

	header->contains_firmware_id = data->options_firmware_id;
	if (header->contains_firmware_id)
		header->firmware_id = extract_uint_le(data->firmware_id);

	return;
}

static int fwu_read_f01_device_status(struct f01_device_status *status)
{
	int retval;

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f01_fd.data_base_addr,
			status->data,
			sizeof(status->data));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Failed to read F01 device status\n",
				__func__);
		return retval;
	}

	return 0;
}

static int fwu_read_f34_queries(void)
{
	int retval;
	unsigned char count;
	unsigned char buf[10];

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f34_fd.query_base_addr + BOOTLOADER_ID_OFFSET,
			fwu->bootloader_id,
			sizeof(fwu->bootloader_id));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Failed to read bootloader ID\n",
				__func__);
		return retval;
	}

	if (fwu->bootloader_id[1] == '5') {
		fwu->bl_version = V5;
	} else if (fwu->bootloader_id[1] == '6') {
		fwu->bl_version = V6;
	} else {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Unrecognized bootloader version\n",
				__func__);
		return -EINVAL;
	}

	if (fwu->bl_version == V5) {
		fwu->properties_off = V5_PROPERTIES_OFFSET;
		fwu->blk_size_off = V5_BLOCK_SIZE_OFFSET;
		fwu->blk_count_off = V5_BLOCK_COUNT_OFFSET;
		fwu->blk_data_off = V5_BLOCK_DATA_OFFSET;
	} else if (fwu->bl_version == V6) {
		fwu->properties_off = V6_PROPERTIES_OFFSET;
		fwu->blk_size_off = V6_BLOCK_SIZE_OFFSET;
		fwu->blk_count_off = V6_BLOCK_COUNT_OFFSET;
		fwu->blk_data_off = V6_BLOCK_DATA_OFFSET;
	}

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f34_fd.query_base_addr + fwu->properties_off,
			&fwu->flash_properties,
			sizeof(fwu->flash_properties));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Failed to read flash properties\n",
				__func__);
		return retval;
	}

	count = 4;

	if (fwu->flash_properties & HAS_PERM_CONFIG) {
		fwu->has_perm_config = 1;
		count += 2;
	}

	if (fwu->flash_properties & HAS_BL_CONFIG) {
		fwu->has_bl_config = 1;
		count += 2;
	}

	if (fwu->flash_properties & HAS_DISP_CONFIG) {
		fwu->has_disp_config = 1;
		count += 2;
	}

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f34_fd.query_base_addr + fwu->blk_size_off,
			buf,
			2);
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Failed to read block size info\n",
				__func__);
		return retval;
	}

	batohs(&fwu->block_size, &(buf[0]));

	if (fwu->bl_version == V5) {
		fwu->flash_cmd_off = fwu->blk_data_off + fwu->block_size;
		fwu->flash_status_off = fwu->flash_cmd_off;
	} else if (fwu->bl_version == V6) {
		fwu->flash_cmd_off = V6_FLASH_COMMAND_OFFSET;
		fwu->flash_status_off = V6_FLASH_STATUS_OFFSET;
	}

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f34_fd.query_base_addr + fwu->blk_count_off,
			buf,
			count);
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Failed to read block count info\n",
				__func__);
		return retval;
	}

	batohs(&fwu->fw_block_count, &(buf[0]));
	batohs(&fwu->config_block_count, &(buf[2]));

	count = 4;

	if (fwu->has_perm_config) {
		batohs(&fwu->perm_config_block_count, &(buf[count]));
		count += 2;
	}

	if (fwu->has_bl_config) {
		batohs(&fwu->bl_config_block_count, &(buf[count]));
		count += 2;
	}

	if (fwu->has_disp_config)
		batohs(&fwu->disp_config_block_count, &(buf[count]));

	return 0;
}

static int fwu_read_f34_flash_status(void)
{
	int retval;
	unsigned char status;
	unsigned char command;

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f34_fd.data_base_addr + fwu->flash_status_off,
			&status,
			sizeof(status));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Failed to read flash status\n",
				__func__);
		return retval;
	}

	fwu->program_enabled = status >> 7;

	if (fwu->bl_version == V5)
		fwu->flash_status = (status >> 4) & MASK_3BIT;
	else if (fwu->bl_version == V6)
		fwu->flash_status = status & MASK_3BIT;

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f34_fd.data_base_addr + fwu->flash_cmd_off,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Failed to read flash command\n",
				__func__);
		return retval;
	}

	fwu->command = command & MASK_4BIT;

	return 0;
}

static int fwu_write_f34_command(unsigned char cmd)
{
	int retval;
	unsigned char command = cmd & MASK_4BIT;

	fwu->command = cmd;

	retval = fwu->fn_ptr->write(fwu->rmi4_data,
			fwu->f34_fd.data_base_addr + fwu->flash_cmd_off,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Failed to write command 0x%02x\n",
				__func__, command);
		return retval;
	}

	return 0;
}

static int fwu_wait_for_idle(int timeout_ms)
{
	int count = 0;
	int timeout_count = ((timeout_ms * 1000) / MAX_SLEEP_TIME_US) + 1;

	do {
		usleep_range(MIN_SLEEP_TIME_US, MAX_SLEEP_TIME_US);

		count++;
		if (count == timeout_count)
			fwu_read_f34_flash_status();

		if ((fwu->command == 0x00) && (fwu->flash_status == 0x00))
                {      
			return 0;
                 }
	} while (count < timeout_count);

	printk("synap %s: Timed out waiting for idle status\n",
			__func__);

	return -ETIMEDOUT;
}

static enum flash_area fwu_go_nogo(struct image_header_data *header)
{
	int retval;
	enum flash_area flash_area = NONE;
	unsigned char index = 0;
	unsigned char config_id[4];
	unsigned int device_config_id;
	unsigned int image_config_id;
	unsigned int device_fw_id;
	unsigned long image_fw_id;
	char *strptr;
	char *firmware_id;

	if (fwu->force_update) {
		flash_area = UI_FIRMWARE;
		goto exit;
	}

	/* Update both UI and config if device is in bootloader mode */
	if (fwu->in_flash_prog_mode) {
		flash_area = UI_FIRMWARE;
		goto exit;
	}

	/* Get device firmware ID */
	device_fw_id = fwu->rmi4_data->firmware_id;
//	dev_info(&fwu->rmi4_data->i2c_client->dev,
//			"synap %s: Device firmware ID = %d\n",
//			__func__, device_fw_id);

//	dev_info(&fwu->rmi4_data->i2c_client->dev,
//			"synap %s: image firmware ID = %d\n",
//			__func__, header->firmware_id);
    header->contains_firmware_id = 1 ;
    header->firmware_id = device_fw_id ;

	/* Get image firmware ID */
	if (header->contains_firmware_id) {
		image_fw_id = header->firmware_id;
	} else {
		strptr = strstr(fwu->image_name, "PR");
		if (!strptr) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"synap %s: No valid PR number (PRxxxxxxx) "
					"found in image file name (%s)\n",
					__func__, fwu->image_name);
			flash_area = NONE;
			goto exit;
		}

		strptr += 2;
		firmware_id = kzalloc(MAX_FIRMWARE_ID_LEN, GFP_KERNEL);
		while (strptr[index] >= '0' && strptr[index] <= '9') {
			firmware_id[index] = strptr[index];
			index++;
		}

		retval = sstrtoul(firmware_id, 10, &image_fw_id);
		kfree(firmware_id);
		if (retval) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"synap %s: Failed to obtain image firmware ID\n",
					__func__);
			flash_area = NONE;
			goto exit;
		}
	}
//	dev_info(&fwu->rmi4_data->i2c_client->dev,
//			"synap %s: Image firmware ID = %d\n",
//			__func__, (unsigned int)image_fw_id);

	if (image_fw_id > device_fw_id) {
		flash_area = UI_FIRMWARE;
		goto exit;
	} else if (image_fw_id < device_fw_id) {
		dev_info(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Image firmware ID older than device firmware ID\n",
				__func__);
		flash_area = NONE;
		goto exit;
	}

	/* Get device config ID */
	retval = fwu->fn_ptr->read(fwu->rmi4_data,
				fwu->f34_fd.ctrl_base_addr,
				config_id,
				sizeof(config_id));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Failed to read device config ID\n",
				__func__);
		flash_area = NONE;
		goto exit;
	}
	device_config_id = extract_uint_be(config_id);
//	dev_info(&fwu->rmi4_data->i2c_client->dev,
//			"synap %s: Device config ID = 0x%02x 0x%02x 0x%02x 0x%02x\n",
//			__func__,
//			config_id[0],
//			config_id[1],
//			config_id[2],
//			config_id[3]);

	/* Get image config ID */
	image_config_id = extract_uint_be(fwu->config_data);
//	dev_info(&fwu->rmi4_data->i2c_client->dev,
//			"synap %s: Image config ID = 0x%02x 0x%02x 0x%02x 0x%02x\n",
//			__func__,
//			fwu->config_data[0],
//			fwu->config_data[1],
//			fwu->config_data[2],
//			fwu->config_data[3]);

    fwu->rmi4_data->image_cid = image_config_id;
    fwu->rmi4_data->device_cid = device_config_id ;
	printk("[syna] config id: [image]=0x%x,[device]=0x%x\n",image_config_id,device_config_id);
    
	if (image_config_id != device_config_id || fwu->force_update) {
		printk("[syna] config id: [image]=0x%x,[device]=0x%x, update!\n",image_config_id,device_config_id);
		flash_area = UI_FIRMWARE;
		goto exit;
	}

	flash_area = NONE;

exit:
//	if (flash_area == NONE) {
//		dev_info(&fwu->rmi4_data->i2c_client->dev,
//				"synap %s: No need to do reflash\n",
//				__func__);
//	} else {
//		dev_info(&fwu->rmi4_data->i2c_client->dev,
//				"synap %s: Updating %s\n",
//				__func__,
//				flash_area == UI_FIRMWARE ?
//				"UI firmware" :
//				"config only");
//	}

	return flash_area;
}

static int fwu_scan_pdt(void)
{
	int retval;
	unsigned char ii;
	unsigned char intr_count = 0;
	unsigned char intr_off;
	unsigned char intr_src;
	unsigned short addr;
	bool f01found = false;
	bool f34found = false;
	struct synaptics_rmi4_fn_desc rmi_fd;

	for (addr = PDT_START; addr > PDT_END; addr -= PDT_ENTRY_SIZE) {
		retval = fwu->fn_ptr->read(fwu->rmi4_data,
				addr,
				(unsigned char *)&rmi_fd,
				sizeof(rmi_fd));
		if (retval < 0)
			return retval;

		if (rmi_fd.fn_number) {
			dev_dbg(&fwu->rmi4_data->i2c_client->dev,
					"synap %s: Found F%02x\n",
					__func__, rmi_fd.fn_number);
			switch (rmi_fd.fn_number) {
			case SYNAPTICS_RMI4_F01:
				f01found = true;
				fwu->f01_fd.query_base_addr =
						rmi_fd.query_base_addr;
				fwu->f01_fd.ctrl_base_addr =
						rmi_fd.ctrl_base_addr;
				fwu->f01_fd.data_base_addr =
						rmi_fd.data_base_addr;
				fwu->f01_fd.cmd_base_addr =
						rmi_fd.cmd_base_addr;
				break;
			case SYNAPTICS_RMI4_F34:
				f34found = true;
				fwu->f34_fd.query_base_addr =
						rmi_fd.query_base_addr;
				fwu->f34_fd.ctrl_base_addr =
						rmi_fd.ctrl_base_addr;
				fwu->f34_fd.data_base_addr =
						rmi_fd.data_base_addr;

				fwu->intr_mask = 0;
				intr_src = rmi_fd.intr_src_count;
				intr_off = intr_count % 8;
				for (ii = intr_off;
						ii < ((intr_src & MASK_3BIT) +
						intr_off);
						ii++) {
					fwu->intr_mask |= 1 << ii;
				}
				break;
			}
		} else {
			break;
		}

		intr_count += (rmi_fd.intr_src_count & MASK_3BIT);
	}

	if (!f01found || !f34found) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Failed to find both F01 and F34\n",
				__func__);
		return -EINVAL;
	}

	return 0;
}

static int fwu_write_blocks(unsigned char *block_ptr, unsigned short block_cnt,
		unsigned char command)
{
	int retval;
	unsigned char block_offset[] = {0, 0};
	unsigned short block_num;

	block_offset[1] |= (fwu->config_area << 5);

	retval = fwu->fn_ptr->write(fwu->rmi4_data,
			fwu->f34_fd.data_base_addr + BLOCK_NUMBER_OFFSET,
			block_offset,
			sizeof(block_offset));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Failed to write to block number registers\n",
				__func__);
		return retval;
	}

	for (block_num = 0; block_num < block_cnt; block_num++) {
		retval = fwu->fn_ptr->write(fwu->rmi4_data,
				fwu->f34_fd.data_base_addr + fwu->blk_data_off,
				block_ptr,
				fwu->block_size);
		if (retval < 0) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"synap %s: Failed to write block data (block %d)\n",
					__func__, block_num);
			return retval;
		}

		retval = fwu_write_f34_command(command);
		if (retval < 0) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"synap %s: Failed to write command for block %d\n",
					__func__, block_num);
			return retval;
		}

		retval = fwu_wait_for_idle(WRITE_WAIT_MS);
		if (retval < 0) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"synap %s: Failed to wait for idle status (block %d)\n",
					__func__, block_num);
			return retval;
		}

		block_ptr += fwu->block_size;
	}

	return 0;
}

static int fwu_write_firmware(void)
{
	return fwu_write_blocks((unsigned char *)fwu->firmware_data,
		fwu->fw_block_count, CMD_WRITE_FW_BLOCK);
}

static int fwu_write_configuration(void)
{
	return fwu_write_blocks((unsigned char *)fwu->config_data,
		fwu->config_block_count, CMD_WRITE_CONFIG_BLOCK);
}

static int fwu_write_lockdown(void)
{
	return fwu_write_blocks((unsigned char *)fwu->lockdown_data,
		fwu->lockdown_block_count, CMD_WRITE_LOCKDOWN_BLOCK);
}

static int fwu_write_bootloader_id(void)
{
	int retval;

	retval = fwu->fn_ptr->write(fwu->rmi4_data,
			fwu->f34_fd.data_base_addr + fwu->blk_data_off,
			fwu->bootloader_id,
			sizeof(fwu->bootloader_id));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Failed to write bootloader ID\n",
				__func__);
		return retval;
	}

	return 0;
}

static int fwu_enter_flash_prog(void)
{
	int retval;
	struct f01_device_status f01_device_status;
	struct f01_device_control f01_device_control;

	retval = fwu_write_bootloader_id();
	if (retval < 0)
		return retval;

	retval = fwu_write_f34_command(CMD_ENABLE_FLASH_PROG);
	if (retval < 0)
		return retval;

	retval = fwu_wait_for_idle(ENABLE_WAIT_MS);
	if (retval < 0)
		return retval;

	if (!fwu->program_enabled) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Program enabled bit not set\n",
				__func__);
		return -EINVAL;
	}

	retval = fwu_scan_pdt();
	if (retval < 0)
		return retval;

	retval = fwu_read_f01_device_status(&f01_device_status);
	if (retval < 0)
		return retval;

	if (!f01_device_status.flash_prog) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Not in flash prog mode\n",
				__func__);
		return -EINVAL;
	}

	retval = fwu_read_f34_queries();
	if (retval < 0)
		return retval;

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f01_fd.ctrl_base_addr,
			f01_device_control.data,
			sizeof(f01_device_control.data));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Failed to read F01 device control\n",
				__func__);
		return retval;
	}

	f01_device_control.nosleep = true;
	f01_device_control.sleep_mode = SLEEP_MODE_NORMAL;

	retval = fwu->fn_ptr->write(fwu->rmi4_data,
			fwu->f01_fd.ctrl_base_addr,
			f01_device_control.data,
			sizeof(f01_device_control.data));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Failed to write F01 device control\n",
				__func__);
		return retval;
	}

	return retval;
}

static int fwu_do_reflash(void)
{
	int retval;


	retval = fwu_enter_flash_prog();
	if (retval < 0)
		return retval;

	printk("synap %s: Entered flash prog mode\n",
			__func__);

	retval = fwu_write_bootloader_id();
	if (retval < 0)
		return retval;

	printk("synap %s: Bootloader ID written\n",
			__func__);

	retval = fwu_write_f34_command(CMD_ERASE_ALL);
	if (retval < 0)
		return retval;

	dev_dbg(&fwu->rmi4_data->i2c_client->dev,
			"synap %s: Erase all command written\n",
			__func__);

	retval = fwu_wait_for_idle(ERASE_WAIT_MS);
	if (retval < 0)
		return retval;

	printk("synap %s: Idle status detected\n",__func__);

	if (fwu->firmware_data) {
		retval = fwu_write_firmware();
		if (retval < 0)
			return retval;
		pr_notice("synap %s: Firmware programmed\n", __func__);
	}
	printk("synap %s: write_configuration\n",__func__);
	if (fwu->config_data) {
		retval = fwu_write_configuration();
		if (retval < 0)
			return retval;
		pr_notice("synap %s: Configuration programmed\n", __func__);
	}

	return retval;
}

static int fwu_do_write_config(void)
{
	int retval;

	retval = fwu_enter_flash_prog();
	if (retval < 0)
		return retval;

	dev_dbg(&fwu->rmi4_data->i2c_client->dev,
			"synap %s: Entered flash prog mode\n",
			__func__);

	if (fwu->config_area == PERM_CONFIG_AREA) {
		fwu->config_block_count = fwu->perm_config_block_count;
		goto write_config;
	}

	retval = fwu_write_bootloader_id();
	if (retval < 0)
		return retval;

	dev_dbg(&fwu->rmi4_data->i2c_client->dev,
			"synap %s: Bootloader ID written\n",
			__func__);

	switch (fwu->config_area) {
	case UI_CONFIG_AREA:
		retval = fwu_write_f34_command(CMD_ERASE_CONFIG);
		break;
	case BL_CONFIG_AREA:
		retval = fwu_write_f34_command(CMD_ERASE_BL_CONFIG);
		fwu->config_block_count = fwu->bl_config_block_count;
		break;
	case DISP_CONFIG_AREA:
		retval = fwu_write_f34_command(CMD_ERASE_DISP_CONFIG);
		fwu->config_block_count = fwu->disp_config_block_count;
		break;
	}
	if (retval < 0)
		return retval;

	dev_dbg(&fwu->rmi4_data->i2c_client->dev,
			"synap %s: Erase command written\n",
			__func__);

	retval = fwu_wait_for_idle(ERASE_WAIT_MS);
	if (retval < 0)
		return retval;

	dev_dbg(&fwu->rmi4_data->i2c_client->dev,
			"synap %s: Idle status detected\n",
			__func__);

write_config:
	retval = fwu_write_configuration();
	if (retval < 0)
		return retval;

	pr_notice("synap %s: Config written\n", __func__);

	return retval;
}

static int fwu_start_write_config(void)
{
	int retval;
	unsigned short block_count;
	unsigned short f01_cmd_base_addr;
	struct image_header_data header;

	f01_cmd_base_addr = fwu->f01_fd.cmd_base_addr;

	switch (fwu->config_area) {
	case UI_CONFIG_AREA:
		block_count = fwu->config_block_count;
		break;
	case PERM_CONFIG_AREA:
		if (!fwu->has_perm_config)
			return -EINVAL;
		block_count = fwu->perm_config_block_count;
		break;
	case BL_CONFIG_AREA:
		if (!fwu->has_bl_config)
			return -EINVAL;
		block_count = fwu->bl_config_block_count;
		break;
	case DISP_CONFIG_AREA:
		if (!fwu->has_disp_config)
			return -EINVAL;
		block_count = fwu->disp_config_block_count;
		break;
	default:
		return -EINVAL;
	}

	if (fwu->ext_data_source)
		fwu->config_data = fwu->ext_data_source;
	else
		return -EINVAL;

	fwu->config_size = fwu->block_size * block_count;

	/* Jump to the config area if given a packrat image */
	if ((fwu->config_area == UI_CONFIG_AREA) &&
			(fwu->config_size != fwu->image_size)) {
		parse_header(&header, fwu->ext_data_source);

		if (header.config_size) {
			fwu->config_data = fwu->ext_data_source +
					FW_IMAGE_OFFSET +
					header.firmware_size;
		} else {
			return -EINVAL;
		}
	}

	pr_notice("synap %s: Start of write config process\n", __func__);

	retval = fwu_do_write_config();
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Failed to write config\n",
				__func__);
	}

	fwu->rmi4_data->reset_device(fwu->rmi4_data, f01_cmd_base_addr);

	pr_notice("synap %s: End of write config process\n", __func__);

	return retval;
}

static int fwu_do_read_config(void)
{
	int retval;
	unsigned char block_offset[] = {0, 0};
	unsigned short block_num;
	unsigned short block_count;
	unsigned short index = 0;
	unsigned short f01_cmd_base_addr;

	f01_cmd_base_addr = fwu->f01_fd.cmd_base_addr;

	retval = fwu_enter_flash_prog();
	if (retval < 0)
		goto exit;

	dev_dbg(&fwu->rmi4_data->i2c_client->dev,
			"synap %s: Entered flash prog mode\n",
			__func__);

	switch (fwu->config_area) {
	case UI_CONFIG_AREA:
		block_count = fwu->config_block_count;
		break;
	case PERM_CONFIG_AREA:
		if (!fwu->has_perm_config) {
			retval = -EINVAL;
			goto exit;
		}
		block_count = fwu->perm_config_block_count;
		break;
	case BL_CONFIG_AREA:
		if (!fwu->has_bl_config) {
			retval = -EINVAL;
			goto exit;
		}
		block_count = fwu->bl_config_block_count;
		break;
	case DISP_CONFIG_AREA:
		if (!fwu->has_disp_config) {
			retval = -EINVAL;
			goto exit;
		}
		block_count = fwu->disp_config_block_count;
		break;
	default:
		retval = -EINVAL;
		goto exit;
	}

	fwu->config_size = fwu->block_size * block_count;

	kfree(fwu->read_config_buf);
	fwu->read_config_buf = kzalloc(fwu->config_size, GFP_KERNEL);

	block_offset[1] |= (fwu->config_area << 5);

	retval = fwu->fn_ptr->write(fwu->rmi4_data,
			fwu->f34_fd.data_base_addr + BLOCK_NUMBER_OFFSET,
			block_offset,
			sizeof(block_offset));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Failed to write to block number registers\n",
				__func__);
		goto exit;
	}

	for (block_num = 0; block_num < block_count; block_num++) {
		retval = fwu_write_f34_command(CMD_READ_CONFIG_BLOCK);
		if (retval < 0) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"synap %s: Failed to write read config command\n",
					__func__);
			goto exit;
		}

		retval = fwu_wait_for_idle(WRITE_WAIT_MS);
		if (retval < 0) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"synap %s: Failed to wait for idle status\n",
					__func__);
			goto exit;
		}

		retval = fwu->fn_ptr->read(fwu->rmi4_data,
				fwu->f34_fd.data_base_addr + fwu->blk_data_off,
				&fwu->read_config_buf[index],
				fwu->block_size);
		if (retval < 0) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"synap %s: Failed to read block data (block %d)\n",
					__func__, block_num);
			goto exit;
		}

		index += fwu->block_size;
	}

exit:
	fwu->rmi4_data->reset_device(fwu->rmi4_data, f01_cmd_base_addr);

	return retval;
}

static int fwu_do_lockdown(void)
{
	int retval;

	retval = fwu_enter_flash_prog();
	if (retval < 0)
		return retval;

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f34_fd.query_base_addr + fwu->properties_off,
			&fwu->flash_properties,
			sizeof(fwu->flash_properties));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Failed to read flash properties\n",
				__func__);
		return retval;
	}

	if ((fwu->flash_properties & UNLOCKED) == 0) {
		dev_info(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Device already locked down\n",
				__func__);
		return retval;
	}

	retval = fwu_write_lockdown();
	if (retval < 0)
		return retval;

	pr_notice("synap %s: Lockdown programmed\n", __func__);

	return retval;
}

static int fwu_start_reflash(void)
{
	int retval = 0;
	enum flash_area flash_area;
	unsigned short f01_cmd_base_addr;
	struct image_header_data header;
	struct f01_device_status f01_device_status;
	const unsigned char *fw_image;
	const struct firmware *fw_entry = NULL;
    unsigned char command = 0x01;

	f01_cmd_base_addr = fwu->f01_fd.cmd_base_addr;

	if (fwu->rmi4_data->sensor_sleep) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Sensor sleeping\n",
				__func__);
		return -ENODEV;
	}

	fwu->rmi4_data->stay_awake = true;

	//pr_notice("synap %s: Start of reflash process\n", __func__);

	if (fwu->ext_data_source) {
		fw_image = fwu->ext_data_source;
	} else {
		strncpy(fwu->image_name, FW_IMAGE_NAME, MAX_IMAGE_NAME_LEN);
		dev_dbg(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Requesting firmware image %s\n",
				__func__, fwu->image_name);

		retval = request_firmware(&fw_entry, fwu->image_name,
				&fwu->rmi4_data->i2c_client->dev);
		if (retval != 0) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"synap %s: Firmware image %s not available\n",
					__func__, fwu->image_name);
			retval = -EINVAL;
			goto exit;
		}

		dev_dbg(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Firmware image size = %d\n",
				__func__, (int)fw_entry->size);

		fw_image = fw_entry->data;
	}

	parse_header(&header, fw_image);

	if (fwu->bl_version != header.bootloader_version) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Bootloader version mismatch\n",
				__func__);
		retval = -EINVAL;
		goto exit;
	}

	retval = fwu_read_f01_device_status(&f01_device_status);
	if (retval < 0)
		goto exit;

	if (f01_device_status.flash_prog) {
		dev_info(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: In flash prog mode\n",
				__func__);
		fwu->in_flash_prog_mode = true;
	} else {
		fwu->in_flash_prog_mode = false;
	}

	if (fwu->do_lockdown) {
		switch (fwu->bl_version) {
		case V5:
		case V6:
			fwu->lockdown_data = fw_image + LOCKDOWN_OFFSET;
			fwu->lockdown_block_count = LOCKDOWN_BLOCK_COUNT;
			retval = fwu_do_lockdown();
			if (retval < 0) {
				dev_err(&fwu->rmi4_data->i2c_client->dev,
						"synap %s: Failed to do lockdown\n",
						__func__);
			}
		default:
			break;
		}
	}

	if (header.firmware_size)
		fwu->firmware_data = fw_image + FW_IMAGE_OFFSET;
	if (header.config_size) {
		fwu->config_data = fw_image + FW_IMAGE_OFFSET +
				header.firmware_size;
	}

	flash_area = fwu_go_nogo(&header);
     //flash_area = UI_FIRMWARE; // add by lauson , force update.
	switch (flash_area) {
	case UI_FIRMWARE:
		retval = fwu_do_reflash();
		break;
	case CONFIG_AREA:
		retval = fwu_do_write_config();
		break;
	case NONE:
	default:
		goto exit;
	}

	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"synap %s: Failed to do reflash\n",
				__func__);
	}

exit:
    if(flash_area)
    {
        // reset tp
        fwu->fn_ptr->write(fwu->rmi4_data,
        			fwu->f01_fd.cmd_base_addr,
        			&command,
        			sizeof(command));    
        
    	fwu->rmi4_data->reset_device(fwu->rmi4_data, f01_cmd_base_addr);
    }

	if (fw_entry)
		release_firmware(fw_entry);

	//pr_notice("synap %s: End of reflash process\n", __func__);

	fwu->rmi4_data->stay_awake = false;

	return retval;
}

static int synaptics_rmi4_fwu_init_func(struct synaptics_rmi4_data *rmi4_data) ;

//return current firmware version
int synaptics_rmi4_get_firmware_version(int vendor_id) 
{
    if(vendor_id == TP_VENDOR_JDI) 
    {
        return FIRMWARE_JDI_14049_VERSION;
    }  
    else 
    {
        return 0 ;
    }
}

int synaptics_fw_updater(unsigned char *fw_data)
{
	int retval;

	if (!fwu)
		return -ENODEV;

	if (!fwu->initialized)
		return -ENODEV;

	fwu->ext_data_source = fw_data;
	fwu->config_area = UI_CONFIG_AREA;

	retval = fwu_start_reflash();

	return retval;
}
EXPORT_SYMBOL(synaptics_fw_updater);

//read tp firmware from /sdcard/synaptics/startup_fw_update.img
#include <linux/syscalls.h>
static unsigned char* synaptics_get_fw_from_file(void)
{
	int fd = 0;
	int count = 0;
	unsigned char* filedata = NULL;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

    sprintf(fwu->image_name,"%s/%s",fwu->image_name,FW_IMAGE_NAME);
	fd = sys_open(fwu->image_name, O_RDONLY, 0);
	if (fd < 0) {
		set_fs(old_fs);
		printk(KERN_WARNING "synap %s: Can not open firmware:%d\n", __func__,fd);
		return 0;
	}
	count = sys_lseek(fd, (off_t)0, 2);
	if (count <= 0) {
		goto err_load_fw_close_file;
	}

	sys_lseek(fd, (off_t)0, 0);
	filedata = kmalloc(count, GFP_KERNEL);
	if (!filedata) {
		printk(KERN_WARNING "synap %s: Can not alloc data\n", __func__);
		goto err_load_fw_close_file;
	}
	if (sys_read(fd, (char *)filedata, count) != count) {
		goto err_load_fw_free_data;
	}

	if (strncmp(filedata+0x10, "S3508", 5)) {
		printk(KERN_WARNING "synap %s: Not correct fw file.\n", __func__);
		goto err_load_fw_free_data;
	}

    sys_close(fd);
	set_fs(old_fs);
	
	return filedata ;


err_load_fw_free_data:
	kfree(filedata);
err_load_fw_close_file:
	sys_close(fd);
	set_fs(old_fs);
	fwu->image_name[0] = 0;
	return 0;
}

static unsigned char* fwu_rmi4_get_firmware_data(void) {
    unsigned char* firmwaredata = 0;
    unsigned int vendor_id ;

    if(!fwu || !(fwu->rmi4_data))
    return 0 ;

    vendor_id = fwu->rmi4_data->vendor_id ;

    if(fwu->image_name && fwu->image_name[0] != 0)  //add manual update firmware
    {
        firmwaredata = synaptics_get_fw_from_file();
    }
	else if(1302 == fwu->rmi4_data->product_info)
	{
		firmwaredata = (unsigned char*)Syna_Firmware_Data;
		printk("synap firmware update s1302!\n");
	}
	else if(3320 == fwu->rmi4_data->product_info)
	{
		if(vendor_id == TP_VENDOR_JDI)
		{
			firmwaredata = (unsigned char*)Syna_Firmware_Data_JDI_14049;
			printk("synap firmware update s3320!\n");
		}
	}

    return firmwaredata ;
}

char* synaptics_s1302_get_vender(void)
{
    char *pconst = "fW_version";

    //printk("synap %s, pconst = %s, version = 0x%x\n", __func__, pconst, FIRMWARE_VERSION);
    sprintf(s1302_vendor_str,"%s(%x)",pconst,FIRMWARE_VERSION);

    return s1302_vendor_str;
}
EXPORT_SYMBOL(synaptics_s1302_get_vender);
static void fwu_startup_fw_update_work(struct work_struct *work)
{
	unsigned char* firmwaredata = 0;
    
	firmwaredata = fwu_rmi4_get_firmware_data() ;
	if(!firmwaredata) {
		printk("[syna]can't find firmware data\n");
		if(fwu && (fwu->rmi4_data))
			fwu->rmi4_data->bcontinue = 1 ;
        return ;
	}
		
	synaptics_fw_updater(firmwaredata);

	fwu->image_name[0] = 0;

	fwu->rmi4_data->bcontinue = 1 ;

	return;
}

static ssize_t fwu_sysfs_show_image(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	if (count < fwu->config_size) {
		dev_err(&rmi4_data->i2c_client->dev,
				"synap %s: Not enough space (%d bytes) in buffer\n",
				__func__, (int)count);
		return -EINVAL;
	}

	memcpy(buf, fwu->read_config_buf, fwu->config_size);

	return fwu->config_size;
}

static ssize_t fwu_sysfs_store_image(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	memcpy((void *)(&fwu->ext_data_source[fwu->data_pos]),
			(const void *)buf,
			count);

	fwu->data_pos += count;

	return count;
}

static ssize_t fwu_sysfs_do_reflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	if (sscanf(buf, "%u", &input) != 1) {
		retval = -EINVAL;
		goto exit;
	}

	if (input & LOCKDOWN) {
		fwu->do_lockdown = true;
		input &= ~LOCKDOWN;
	}

	if ((input != NORMAL) && (input != FORCE)) {
		retval = -EINVAL;
		goto exit;
	}

	if (input == FORCE)
		fwu->force_update = true;

	retval = synaptics_fw_updater(fwu->ext_data_source);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"synap %s: Failed to do reflash\n",
				__func__);
		goto exit;
	}

	retval = count;

exit:
	kfree(fwu->ext_data_source);
	fwu->ext_data_source = NULL;
	fwu->force_update = FORCE_UPDATE;
	fwu->do_lockdown = DO_LOCKDOWN;
	return retval;
}

static ssize_t fwu_sysfs_write_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	if (sscanf(buf, "%u", &input) != 1) {
		retval = -EINVAL;
		goto exit;
	}

	if (input != 1) {
		retval = -EINVAL;
		goto exit;
	}

	retval = fwu_start_write_config();
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"synap %s: Failed to write config\n",
				__func__);
		goto exit;
	}

	retval = count;

exit:
	kfree(fwu->ext_data_source);
	fwu->ext_data_source = NULL;
	return retval;
}

static ssize_t fwu_sysfs_read_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input != 1)
		return -EINVAL;

	retval = fwu_do_read_config();
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"synap %s: Failed to read config\n",
				__func__);
		return retval;
	}

	return count;
}

static ssize_t fwu_sysfs_config_area_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned long config_area;

	retval = sstrtoul(buf, 10, &config_area);
	if (retval)
		return retval;

	fwu->config_area = config_area;

	return count;
}

static ssize_t fwu_sysfs_image_name_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	memcpy(fwu->image_name, buf, count);

	return count;
}

static ssize_t fwu_sysfs_image_size_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned long size;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	retval = sstrtoul(buf, 10, &size);
	if (retval)
		return retval;

	fwu->image_size = size;
	fwu->data_pos = 0;

	kfree(fwu->ext_data_source);
	fwu->ext_data_source = kzalloc(fwu->image_size, GFP_KERNEL);
	if (!fwu->ext_data_source) {
		dev_err(&rmi4_data->i2c_client->dev,
				"synap %s: Failed to alloc mem for image data\n",
				__func__);
		return -ENOMEM;
	}

	return count;
}

static ssize_t fwu_sysfs_block_size_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->block_size);
}

static ssize_t fwu_sysfs_firmware_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->fw_block_count);
}

static ssize_t fwu_sysfs_configuration_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->config_block_count);
}

static ssize_t fwu_sysfs_perm_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->perm_config_block_count);
}

static ssize_t fwu_sysfs_bl_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->bl_config_block_count);
}

static ssize_t fwu_sysfs_disp_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->disp_config_block_count);
}

static void synaptics_rmi4_fwu_attn(struct synaptics_rmi4_data *rmi4_data,
		unsigned char intr_mask)
{
	if (!fwu)
		return;

	if (fwu->intr_mask & intr_mask)
		fwu_read_f34_flash_status();

	return;
}

//init device's function
static int synaptics_rmi4_fwu_init_func(struct synaptics_rmi4_data *rmi4_data) {
    int retval;
	struct pdt_properties pdt_props;

	retval = fwu->fn_ptr->read(rmi4_data,
			PDT_PROPS,
			pdt_props.data,
			sizeof(pdt_props.data));
	if (retval < 0) {
		dev_dbg(&rmi4_data->i2c_client->dev,
				"synap %s: Failed to read PDT properties, assuming 0x00\n",
				__func__);
	} else if (pdt_props.has_bsr) {
		dev_err(&rmi4_data->i2c_client->dev,
				"synap %s: Reflash for LTS not currently supported\n",
				__func__);
		retval = -ENODEV;
		goto exit_free_mem;
	}

	retval = fwu_scan_pdt();
	if (retval < 0)
		goto exit_free_mem;

	fwu->productinfo1 = rmi4_data->rmi4_mod_info.product_info[0];
	fwu->productinfo2 = rmi4_data->rmi4_mod_info.product_info[1];
	memcpy(fwu->product_id, rmi4_data->rmi4_mod_info.product_id_string,
			SYNAPTICS_RMI4_PRODUCT_ID_SIZE);
	fwu->product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE] = 0;

	dev_dbg(&rmi4_data->i2c_client->dev,
			"synap %s: F01 product info: 0x%04x 0x%04x\n",
			__func__, fwu->productinfo1, fwu->productinfo2);
	dev_dbg(&rmi4_data->i2c_client->dev,
			"synap %s: F01 product ID: %s\n",
			__func__, fwu->product_id);

	retval = fwu_read_f34_queries();
	if (retval < 0)
		goto exit_free_mem;

	fwu->force_update = FORCE_UPDATE;
	fwu->do_lockdown = DO_LOCKDOWN;
	fwu->initialized = true;

	return 0 ;
	
exit_free_mem:

	return retval;
	
}

static int synaptics_rmi4_fwu_init(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char attr_count;
	int i,match;

	fwu = kzalloc(sizeof(*fwu), GFP_KERNEL);
	if (!fwu) {
		dev_err(&rmi4_data->i2c_client->dev,
				"synap %s: Failed to alloc mem for fwu\n",
				__func__);
		retval = -ENOMEM;
		goto exit;
	}

	fwu->fn_ptr = kzalloc(sizeof(*(fwu->fn_ptr)), GFP_KERNEL);
	if (!fwu->fn_ptr) {
		dev_err(&rmi4_data->i2c_client->dev,
				"synap %s: Failed to alloc mem for fn_ptr\n",
				__func__);
		retval = -ENOMEM;
		goto exit_free_fwu;
	}

	fwu->image_name = kzalloc(MAX_IMAGE_NAME_LEN, GFP_KERNEL);
	if (!fwu->image_name) {
		dev_err(&rmi4_data->i2c_client->dev,
				"synap %s: Failed to alloc mem for image name\n",
				__func__);
		retval = -ENOMEM;
		goto exit_free_fn_ptr;
	}

	fwu->rmi4_data = rmi4_data;
	fwu->fn_ptr->read = rmi4_data->i2c_read;
	fwu->fn_ptr->write = rmi4_data->i2c_write;
	fwu->fn_ptr->enable = rmi4_data->irq_enable;

	if(synaptics_rmi4_fwu_init_func(rmi4_data))
		goto exit_free_mem ;



	retval = sysfs_create_bin_file(&rmi4_data->input_dev->dev.kobj,
			&dev_attr_data);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"synap %s: Failed to create sysfs bin file\n",
				__func__);
		goto exit_free_mem;
	}

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		retval = sysfs_create_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
					"synap %s: Failed to create sysfs attributes\n",
					__func__);
			retval = -ENODEV;
			goto exit_remove_attrs;
		}
	}

#ifdef DO_STARTUP_FW_UPDATE
if (1)
{
	fwu->fwu_workqueue = create_singlethread_workqueue("fwu_workqueue");
	INIT_DELAYED_WORK(&fwu->fwu_work, fwu_startup_fw_update_work);
	/*Call fwu thread to idle CPU*/
	match = 0;
	for(i = 0; i < 3; i++){
		if(cpu_is_offline(i)||(i==smp_processor_id()))
			continue;
          queue_delayed_work_on(i, fwu->fwu_workqueue,
                &fwu->fwu_work,
                msecs_to_jiffies(STARTUP_FW_UPDATE_DELAY_MS));
		  printk("synaptics fwu queue work %d\n", i);
		  match = 1;
		 break;
	}
	if(match == 0)
          queue_delayed_work_on(0, fwu->fwu_workqueue,
                &fwu->fwu_work,
                msecs_to_jiffies(STARTUP_FW_UPDATE_DELAY_MS));
}
#endif

	return 0;

exit_remove_attrs:
for (attr_count--; attr_count >= 0; attr_count--) {
	sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
			&attrs[attr_count].attr);
}

sysfs_remove_bin_file(&rmi4_data->input_dev->dev.kobj, &dev_attr_data);

exit_free_mem:
	kfree(fwu->image_name);

exit_free_fn_ptr:
	kfree(fwu->fn_ptr);

exit_free_fwu:
	kfree(fwu);
	fwu = NULL;

exit:
	return retval;
}

static void synaptics_rmi4_fwu_remove(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char attr_count;

	if (!fwu)
		goto exit;

	sysfs_remove_bin_file(&rmi4_data->input_dev->dev.kobj, &dev_attr_data);

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}

	kfree(fwu->read_config_buf);
	kfree(fwu->image_name);
	kfree(fwu->fn_ptr);
	kfree(fwu);
	fwu = NULL;

exit:
	complete(&fwu_remove_complete);

	return;
}
#if 1 /* delete it by lauson. */
static ssize_t synaptics_proc_write(struct file *file, const char __user *buff, size_t len, loff_t *lo)
{
	int copy_len = len;
	unsigned char temp[20] ;
	unsigned int val[4];
	int ret ;

	if (copy_len >= 20 || copy_from_user( temp, buff, copy_len )) {
		printk(KERN_INFO "synaptics read proc input error.\n");
		return -EFAULT;
	}
	temp[copy_len] = 0;

	if (temp[0] == '1') {
		fwu->force_update = 1;
		ret = synaptics_fw_updater(fwu_rmi4_get_firmware_data());
		printk("[syna]update return value=0x%x\n",ret);
	} else if(temp[0] == '2' && temp[1] == '2') {
	    //add interface for load tp firmware from file
		fwu->force_update = 1;
		if(!fwu->image_name[0]) {
			strcpy(fwu->image_name,"/sdcard");
			queue_delayed_work(fwu->fwu_workqueue,
					&fwu->fwu_work,
					msecs_to_jiffies(5));
		}
	} else if(temp[0]=='3' && temp[1]=='3') {
        //test codes
		sscanf(temp+3, "%x %x %x", val,val+1,val+2);
		temp[0]=val[0];temp[1]=val[1];temp[2]=val[2];
		if(temp[2]<=8) {
			ret = fwu->fn_ptr->read(fwu->rmi4_data,
				(((unsigned short)temp[0])<<8)|temp[1],temp+3,temp[2]);
			printk("[syna]:read[%x]:%x %x %x %x %x %x %x %x\n",ret,
				temp[3],temp[4],temp[5],temp[6],temp[7],temp[8],temp[9],temp[10]);
			
		}
	   
	} else if(temp[0]=='4' && temp[1]=='4') {
        //test codes
		sscanf(temp+3, "%x %x %x", val,val+1,val+2);
		temp[0]=val[0];temp[1]=val[1];temp[2]=val[2];
		if(temp[2]<=3) {
			sscanf(temp+12, "%x %x %x", val,val+1,val+2);
			temp[3]=val[0];temp[4]=val[1];temp[5]=val[2];
			ret = fwu->fn_ptr->write(fwu->rmi4_data,
				(((unsigned short)temp[0])<<8)|temp[1],temp+3,temp[2]);
			printk("[syna]:write[%x]\n",ret);
		}

	} else {
	printk("[syna]I can't get your cmd!!\n");
	}
	
	return len ;
}
const struct file_operations syna_write=
{
		//.owner		= THIS_MODULE,
		//.open		= seq_open,
		//.read		= seq_read,
		.write		= synaptics_proc_write,
		//.llseek 	= seq_lseek,
		//.release	= single_release,
};

static int init_synaptics_proc(void)
{
	int ret=0;

	struct proc_dir_entry *proc_entry;
	
   	proc_entry = proc_create_data("syna_write", 0222, NULL,&syna_write,NULL);

	if (proc_entry == NULL)
	{
		ret = -ENOMEM;
	  	printk(KERN_INFO"init_synaptics_proc: Couldn't create proc entry\n");
	}

	return ret;
}
#endif 

//init s3508 fw module
int rmi4_fw_module_init(bool insert) {
	synaptics_rmi4_new_function(RMI_FW_UPDATER, insert,
			synaptics_rmi4_fwu_init,
			synaptics_rmi4_fwu_remove,
			synaptics_rmi4_fwu_attn);
	return 0;
}

//init s1302 fw module
extern void synaptics_rmi4_new_function_s1302(enum exp_fn fn_type, bool insert,
        int (*func_init)(struct synaptics_rmi4_data *rmi4_data),
        void (*func_remove)(struct synaptics_rmi4_data *rmi4_data),
        void (*func_attn)(struct synaptics_rmi4_data *rmi4_data,
            unsigned char intr_mask));
int rmi4_fw_module_init_s1302(bool insert) {
	synaptics_rmi4_new_function_s1302(RMI_FW_UPDATER, insert,
			synaptics_rmi4_fwu_init,
			synaptics_rmi4_fwu_remove,
			synaptics_rmi4_fwu_attn);
	return 0;
}
static int __init rmi4_fw_update_module_init(void)
{
    /*
    synaptics_rmi4_new_function(RMI_FW_UPDATER, true,
    		synaptics_rmi4_fwu_init,
    		synaptics_rmi4_fwu_remove,
    		synaptics_rmi4_fwu_attn);
    */
    init_synaptics_proc();
    return 0;
}

static void __exit rmi4_fw_update_module_exit(void)
{
	synaptics_rmi4_new_function(RMI_FW_UPDATER, false,
			synaptics_rmi4_fwu_init,
			synaptics_rmi4_fwu_remove,
			synaptics_rmi4_fwu_attn);
	wait_for_completion(&fwu_remove_complete);
	return;
}

module_init(rmi4_fw_update_module_init);
module_exit(rmi4_fw_update_module_exit);

MODULE_DESCRIPTION("Synaptics DSX FW Update Module");
MODULE_LICENSE("GPL");
