/*
 *  stmvl6180.c - Linux kernel modules for STM VL6180 FlightSense Time-of-Flight sensor
 *
 *  Copyright (C) 2014 STMicroelectronics Imaging Division.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <asm/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/miscdevice.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

//API includes
#include "vl6180x_api.h"
#include "vl6180x_def.h"
#include "vl6180x_platform.h"
#include "stmvl6180.h"
#include "linux/workqueue.h"

#include <linux/param_rw.h>

//#define VL6180_DEBUG
#undef CDBG
#ifdef VL6180_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)

#else
#define CDBG(fmt, args...) do{}while(0)
#endif

stmvl6180x_dev vl6180x_dev;
//#define USE_INT
#define IRQ_NUM	   59
#define VL6180_I2C_ADDRESS  (0x52>>1)
static struct i2c_client *client;

/*
 * Global data
 */
//******************************** IOCTL definitions
#define VL6180_IOCTL_INIT 		_IO('p', 0x01)
#define VL6180_IOCTL_XTALKCALB		_IO('p', 0x02)
#define VL6180_IOCTL_OFFCALB		_IO('p', 0x03)
#define VL6180_IOCTL_STOP		_IO('p', 0x05)
#define VL6180_IOCTL_SETXTALK		_IOW('p', 0x06, unsigned int)
#define VL6180_IOCTL_SETOFFSET		_IOW('p', 0x07, int8_t)
#define VL6180_IOCTL_GETDATA 		_IOR('p', 0x0a, unsigned long)
#define VL6180_IOCTL_GETDATAS 		_IOR('p', 0x0b, VL6180x_RangeData_t)
struct mutex	  vl6180_mutex;
#define MULTI_READ	     1
#define CALIBRATION_FILE 1
#ifdef CALIBRATION_FILE
int8_t offset_calib=0;
int16_t xtalk_calib=0;
#endif
#ifdef MULTI_READ
static uint32_t get_unsigned_int_from_buffer(uint8_t *pdata, int8_t count);
static uint16_t get_unsigned_short_from_buffer(uint8_t *pdata, int8_t count);
static int stmvl6180_ps_read_result(struct i2c_client *client);
static void stmvl6180_ps_parse_result(struct i2c_client *client);
#endif
extern void i2c_setclient(struct i2c_client *client);
extern struct i2c_client* i2c_getclient(void);
int stmvl6180_power_enable(struct stmvl6180_data *vl6180_data, unsigned int enable);
static int stmvl6180_init_client(struct stmvl6180_data *vl6180_data);

struct stmvl6180_data *vl6180_data_g;

static int stmvl6180_set_enable(struct i2c_client *client, unsigned int enable)
{
	return 0;
}
#ifdef CALIBRATION_FILE

static void stmvl6180_read_calibration_file(void)
{
#if 1
	uint32 offset = 0;
	uint32 cross_talk = 0;

	if(get_param_camera_laser_sensor_offset(&offset) < 0){
		pr_err("%s:%d get_param_camera_laser_sensor_offset failed\n", __func__, __LINE__);
		return;
	}
	if(get_param_camera_laser_sensor_cross_talk(&cross_talk) < 0){
		pr_err("%s:%d get_param_camera_laser_sensor_cross_talk failed\n", __func__, __LINE__);
		return;
	}

	if(offset >= 128)
		offset_calib = offset - 256;
	else
		offset_calib = offset;

	xtalk_calib = cross_talk;

	CDBG("%s:%d, offset_calib as %d\n",  __func__, __LINE__, offset_calib);
	VL6180x_SetUserOffsetCalibration(vl6180x_dev, offset_calib);

	CDBG("%s:%d xtalk_calib as %d\n", __func__, __LINE__, xtalk_calib);
	VL6180x_SetUserXTalkCompensationRate(vl6180x_dev, xtalk_calib);

#else

	struct file *f;
	char buf[8];
	mm_segment_t fs;
	int i,is_sign=0;
#ifdef CALIBRATION_FILE
	int8_t offset_calib_t=0;
	int16_t xtalk_calib_t=0;
#endif

	f = filp_open("/persist/camera/LaserFocusOffset.txt", O_RDONLY, 0);
	if (f!= NULL && !IS_ERR(f) && f->f_dentry!=NULL)
	{
		fs = get_fs();
		set_fs(get_ds());
		//init the buffer with 0
		for (i=0;i<8;i++)
			buf[i]=0;
		f->f_op->read(f, buf, 8, &f->f_pos);
		set_fs(fs);
		CDBG("offset:%d, offset as:%s, buf[0]:%c\n", offset_calib_t, buf, buf[0]);
		for (i=0;i<8;i++)
		{
			if (i==0 && buf[0]=='-')
				is_sign=1;
			else if (buf[i]>='0' && buf[i]<='9')
				offset_calib_t = offset_calib_t*10 + (buf[i]-'0');
			else
				break;
		}
		CDBG("is_sign:%d, offset_calib:%d\n", is_sign, offset_calib_t);

		if (is_sign)
			offset_calib_t = -offset_calib_t;
		offset_calib = offset_calib_t;

		CDBG("offset_calib as %d\n", offset_calib);
		VL6180x_SetUserOffsetCalibration(vl6180x_dev, offset_calib);
		filp_close(f, NULL);
	}
	else
		pr_err("no offset calibration file exist!\n");

	is_sign=0;
	f = filp_open("/persist/camera/LaserFocusCrossTalk.txt", O_RDONLY, 0);
	if (f!= NULL && !IS_ERR(f) && f->f_dentry!=NULL)
	{
		fs = get_fs();
		set_fs(get_ds());
		//init the buffer with 0
		for (i=0;i<8;i++)
			buf[i]=0;
		f->f_op->read(f, buf, 8, &f->f_pos);
		set_fs(fs);
		CDBG("xtalk_calib:%d, xtalk as:%s, buf[0]:%c\n", xtalk_calib_t, buf, buf[0]);
		for (i=0;i<8;i++)
		{
			if (i==0 && buf[0]=='-')
				is_sign=1;
			else if (buf[i]>='0' && buf[i]<='9')
				xtalk_calib_t = xtalk_calib_t*10 + (buf[i]-'0');
			else
				break;
		}
		if (is_sign==1)
			xtalk_calib_t = -xtalk_calib_t;
		xtalk_calib = xtalk_calib_t;
		CDBG("xtalk_calib as %d\n", xtalk_calib);
		VL6180x_SetUserXTalkCompensationRate(vl6180x_dev, xtalk_calib);
		filp_close(f, NULL);
	}
	else
		pr_err("no xtalk calibration file exist!\n");

	return;
#endif
}
static void stmvl6180_write_offset_calibration_file(void)
{
#if 1
	uint offset = 0;
	if(offset_calib > 127 || offset_calib < -128){
		pr_err("%s:%d wrong offset value\n", __func__, __LINE__);
		return;
	}
//offset_calib is int8_t type, but the data is saved as unsigned int, so need to convert
	if(offset_calib < 0)
		offset = (uint)(256+offset_calib);
	else
		offset = offset_calib;
	if(set_param_camera_laser_sensor_offset(&offset) < 0)
		pr_err("%s:%d set_param_camera_laser_sensor_offset failed\n", __func__, __LINE__);

	return;

#else
	struct file *f;
	char buf[8]={0};
	mm_segment_t fs;

	f = filp_open("/persist/camera/LaserFocusOffset.txt", O_WRONLY|O_CREAT, 0644);
	if (f!= NULL)
	{
		fs = get_fs();
		set_fs(get_ds());
		sprintf(buf,"%d",offset_calib);
		CDBG("write offset as:%s, buf[0]:%c\n",buf, buf[0]);
		f->f_op->write(f, buf, 8, &f->f_pos);
		set_fs(fs);
		VL6180x_SetUserOffsetCalibration(vl6180x_dev, offset_calib);
	}else
		pr_err("%s:%d open /persist/camera/LaserFocusOffset.txt failed\n", __func__, __LINE__);

	if(f)
		filp_close(f, NULL);

	return;
#endif

}
static void stmvl6180_write_xtalk_calibration_file(void)
{
#if 1
	uint cross_talk = 0;
//cross talk is always positive
	if(xtalk_calib < 0){
		pr_err("%s:%d xtalk_calib = %d, convert it to positive\n", __func__, __LINE__, xtalk_calib);
		cross_talk = (uint)(-xtalk_calib);
	}else
		cross_talk = (uint)(xtalk_calib);
	if(set_param_camera_laser_sensor_cross_talk(&cross_talk) < 0)
		pr_err("%s:%d set_param_camera_laser_sensor_cross_talk failed\n", __func__, __LINE__);

	return;
#else

	struct file *f;
	char buf[8]={0};
	mm_segment_t fs;

	f = filp_open("/persist/camera/LaserFocusCrossTalk.txt", O_WRONLY|O_CREAT, 0644);
	if (f!= NULL)
	{
		fs = get_fs();
		set_fs(get_ds());
		sprintf(buf,"%d",xtalk_calib);
		CDBG("write xtalk as:%s, buf[0]:%c\n",buf, buf[0]);
		f->f_op->write(f, buf, 8, &f->f_pos);
		set_fs(fs);
		VL6180x_SetUserXTalkCompensationRate(vl6180x_dev, xtalk_calib);
	}else
		pr_err("%s:%d open /persist/camera/LaserFocusCrossTalk.txt failed\n", __func__, __LINE__);

	if(f)
		filp_close(f, NULL);

	return;
#endif
}

#endif
#ifdef MULTI_READ
static uint32_t get_unsigned_int_from_buffer(uint8_t *pdata, int8_t count)
{
	uint32_t value=0;
	while (count-- > 0)
	{
		value = (value << 8) | (uint32_t)*pdata++;
	}
	return value;
}
static uint16_t get_unsigned_short_from_buffer(uint8_t *pdata, int8_t count)
{
	uint16_t value=0;
	while (count-- > 0)
	{
		value = (value << 8) | (uint16_t)*pdata++;
	}
	return value;
}
static int stmvl6180_ps_read_result(struct i2c_client *client)
{
	struct stmvl6180_data *vl6180_data = vl6180_data_g;
	int status=0;

	status = VL6180x_RdBuffer(vl6180x_dev, RESULT_RANGE_STATUS , vl6180_data->ResultBuffer,RESULT_REG_COUNT);
        if (status) {
            pr_err("RESULT_RANGE_STATUS rd fail status is:%d\n",status);
            return status;
        }
	status = VL6180x_RdByte(vl6180x_dev, SYSRANGE_PART_TO_PART_RANGE_OFFSET, &(vl6180_data->rangeData.m_rangeOffset));
        if (status) {
            pr_err("SYSRANGE_PART_TO_PART_RANGE_OFFSET rd failstatus is:%d\n",status);
            return status;
        }
        status = VL6180x_RdWord(vl6180x_dev, SYSRANGE_CROSSTALK_COMPENSATION_RATE, &(vl6180_data->rangeData.m_crossTalk));
        if (status) {
            pr_err("SYSRANGE_CROSSTALK_COMPENSATION_RATE rd failstatus is:%d\n",status);
	   return status;
        }

	return status;
}
static void stmvl6180_ps_parse_result(struct i2c_client *client)
{
	struct stmvl6180_data *vl6180_data = vl6180_data_g;

	//RESULT_RANGE_STATUS:0x004D
	vl6180_data->rangeResult.Result_range_status = vl6180_data->ResultBuffer[0];
	//RESULT_INTERRUPT_STATUS:0x004F
	vl6180_data->rangeResult.Result_interrupt_status = vl6180_data->ResultBuffer[1];
	//RESULT_RANGE_VAL:0x0062
	vl6180_data->rangeResult.Result_range_val = vl6180_data->ResultBuffer[(0x62-0x4d)];
	//RESULT_RANGE_RAW:0x0064
	vl6180_data->rangeResult.Result_range_raw = vl6180_data->ResultBuffer[(0x64-0x4d)];
	//RESULT_RANGE_RETURN_RATE:0x0066
	vl6180_data->rangeResult.Result_range_return_rate = get_unsigned_short_from_buffer(vl6180_data->ResultBuffer+(0x66-0x4d),2);
	//RESULT_RANGE_REFERENCE_RATE:0x0068
	vl6180_data->rangeResult.Result_range_reference_rate = get_unsigned_short_from_buffer(vl6180_data->ResultBuffer+(0x68-0x4d),2);
	//RESULT_RANGE_RETURN_SIGNAL_COUNT:0x006c
	vl6180_data->rangeResult.Result_range_return_signal_count = get_unsigned_int_from_buffer(vl6180_data->ResultBuffer+(0x6c-0x4d),4);
	//RESULT_RANGE_REFERENCE_SIGNAL_COUNT:0x0070
	vl6180_data->rangeResult.Result_range_reference_signal_count = get_unsigned_int_from_buffer(vl6180_data->ResultBuffer+(0x70-0x4d),4);
	//RESULT_RANGE_RETURN_AMB_COUNT:0x0074
	vl6180_data->rangeResult.Result_range_return_amb_count = get_unsigned_int_from_buffer(vl6180_data->ResultBuffer+(0x74-0x4d),4);
	//RESULT_RANGE_REFERENCE_AMB_COUNT:0x0078
	vl6180_data->rangeResult.Result_range_reference_amb_count = get_unsigned_int_from_buffer(vl6180_data->ResultBuffer+(0x78-0x4d),4);
	//RESULT_RANGE_RETURN_CONV_TIME:0x007c
	vl6180_data->rangeResult.Result_range_return_conv_time = get_unsigned_int_from_buffer(vl6180_data->ResultBuffer+(0x7c-0x4d),4);
	//RESULT_RANGE_REFERENCE_CONV_TIME:0x0080
	vl6180_data->rangeResult.Result_range_reference_conv_time = get_unsigned_int_from_buffer(vl6180_data->ResultBuffer+(0x80-0x4d),4);


//	data->rangeData.m_refRate = data->rangeResult.Result_range_reference_rate;
//	data->rangeData.m_refRate = data->rangeResult.Result_range_reference_rate;
	vl6180_data->rangeData.m_rtnSignalCount = vl6180_data->rangeResult.Result_range_return_signal_count;
	vl6180_data->rangeData.m_refSignalCount = 	vl6180_data->rangeResult.Result_range_reference_signal_count;
	vl6180_data->rangeData.m_rtnAmbientCount = vl6180_data->rangeResult.Result_range_return_amb_count;
	vl6180_data->rangeData.m_refAmbientCount = vl6180_data->rangeResult.Result_range_reference_amb_count;
	vl6180_data->rangeData.m_rawRange_mm = vl6180_data->rangeResult.Result_range_raw;

	if(vl6180_data->rangeResult.Result_range_return_conv_time < vl6180_data->rangeResult.Result_range_reference_conv_time)
		vl6180_data->rangeData.m_convTime = 	vl6180_data->rangeResult.Result_range_reference_conv_time;
	else
		vl6180_data->rangeData.m_convTime = 	vl6180_data->rangeResult.Result_range_return_conv_time;

	return;
}
#endif
static void stmvl6180_ps_read_measurement(struct i2c_client *client)
{
	struct stmvl6180_data *vl6180_data = vl6180_data_g;
	struct timeval tv;

#ifdef MULTI_READ
	VL6180x_RangeGetMeasurement_ext(vl6180x_dev, &(vl6180_data->rangeResult), &(vl6180_data->rangeData));
#else
	VL6180x_RangeGetMeasurement(vl6180x_dev, &(vl6180_data->rangeData));
#endif
	do_gettimeofday(&tv);

	vl6180_data->ps_data = vl6180_data->rangeData.range_mm;

	input_report_abs(vl6180_data->input_dev_ps, ABS_DISTANCE, (int)(vl6180_data->ps_data+5)/10); //range in cm
//	input_report_abs(data->input_dev_ps, ABS_HAT0X, data->rangeData.rtnConvTime/1000000); //tv_sec
//	input_report_abs(data->input_dev_ps, ABS_HAT0Y, data->rangeData.rtnConvTime); //tv_usec
	input_report_abs(vl6180_data->input_dev_ps, ABS_HAT0X,tv.tv_sec);
	input_report_abs(vl6180_data->input_dev_ps, ABS_HAT0Y,tv.tv_usec);
	input_report_abs(vl6180_data->input_dev_ps, ABS_HAT1X,vl6180_data->rangeData.range_mm);
	input_report_abs(vl6180_data->input_dev_ps, ABS_HAT1Y,vl6180_data->rangeData.errorStatus);
#ifdef VL6180x_HAVE_RATE_DATA
	input_report_abs(vl6180_data->input_dev_ps, ABS_HAT2X,vl6180_data->rangeData.signalRate_mcps);
	input_report_abs(vl6180_data->input_dev_ps, ABS_HAT2Y,vl6180_data->rangeData.rtnAmbRate);
	input_report_abs(vl6180_data->input_dev_ps, ABS_HAT3X,vl6180_data->rangeData.rtnConvTime);
#endif
#if  VL6180x_HAVE_DMAX_RANGING
	input_report_abs(vl6180_data->input_dev_ps, ABS_HAT3Y,vl6180_data->rangeData.DMax);
#endif

	input_sync(vl6180_data->input_dev_ps);
	if (vl6180_data->enableDebug)
		CDBG("range:%d, signalrate_mcps:%d, error:0x%x,rtnsgnrate:%u, rtnambrate:%u,rtnconvtime:%u\n",
			vl6180_data->rangeData.range_mm,
			vl6180_data->rangeData.signalRate_mcps,
			vl6180_data->rangeData.errorStatus,
			vl6180_data->rangeData.rtnRate,
			vl6180_data->rangeData.rtnAmbRate,
			vl6180_data->rangeData.rtnConvTime);

}
/* interrupt work handler */
static void stmvl6180_work_handler(struct work_struct *work)
{
	struct stmvl6180_data *vl6180_data = vl6180_data_g;
	struct i2c_client *client=vl6180_data->client;
	int ret=0;
#ifndef MULTI_READ
	uint8_t gpio_status=0, range_start=0, range_status=0;
#endif
	uint8_t to_startPS=0;

	mutex_lock(&vl6180_data->work_mutex);

//likelong added
	if( !vl6180_data->enable_ps_sensor){
		mutex_unlock(&vl6180_data->work_mutex);
		pr_err("vl6180_data->enable_ps_sensor %d\n", vl6180_data->enable_ps_sensor);
		return;
	}

#ifdef MULTI_READ
	ret = stmvl6180_ps_read_result(client);
	if (ret == 0 && ((vl6180_data->ResultBuffer[0]&0x01) == 0x01)){
		if( vl6180_data->enable_ps_sensor){
			stmvl6180_ps_parse_result(client);
			CDBG("stmvl6180_ps_read_measurement\n");
			stmvl6180_ps_read_measurement(client);
			if (vl6180_data->ps_is_singleshot)
				to_startPS = 1;
		}else
			pr_err("enable_ps_sensor is %d\n", vl6180_data->enable_ps_sensor);
	}else
		pr_err("%s:%d ret %d\n", __func__, __LINE__, ret);
#else
	VL6180x_RangeGetInterruptStatus(vl6180x_dev, &gpio_status);
	VL6180x_RdByte(vl6180x_dev, RESULT_RANGE_STATUS, &range_status);
	VL6180x_RdByte(vl6180x_dev, SYSRANGE_START, &range_start);

	//if (gpio_status == RES_INT_STAT_GPIO_NEW_SAMPLE_READY)
	if (((range_status&0x01)==0x01) && (range_start== 0x00)){
		if( vl6180_data->enable_ps_sensor){
			CDBG("stmvl6180_ps_read_measurement\n");
			stmvl6180_ps_read_measurement(client);
			if (vl6180_data->ps_is_singleshot)
				to_startPS = 1;
		}else
			pr_err("enable_ps_sensor is %d\n", vl6180_data->enable_ps_sensor);
		VL6180x_RangeClearInterrupt(vl6180x_dev);

	}else
		pr_err("range_status %d, range_start %d\n", range_status, range_start);
#endif
	if (to_startPS){
		VL6180x_RangeSetSystemMode(vl6180x_dev, MODE_START_STOP | MODE_SINGLESHOT);
	}
	CDBG("schedule_delayed_work\n");
	schedule_delayed_work(&vl6180_data->dwork, msecs_to_jiffies((vl6180_data->delay_ms)));	// restart timer

   	mutex_unlock(&vl6180_data->work_mutex);

	return;
}

#ifdef USE_INT
static irqreturn_t stmvl6180_interrupt_handler(int vec, void *info)
{
	struct stmvl6180_data *vl6180_data = vl6180_data_g;

	if (vl6180_data->irq == vec){
		CDBG("==>interrupt_handler\n");
		CDBG("schedule_delayed_work\n");
		schedule_delayed_work(&vl6180_data->dwork, 0);
	}
	return IRQ_HANDLED;
}
#endif

/*
 * SysFS support
 */
static ssize_t stmvl6180_show_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct stmvl6180_data *vl6180_data = vl6180_data_g;

	return sprintf(buf, "%d\n", vl6180_data->enable_ps_sensor);
}

static ssize_t stmvl6180_store_enable_ps_sensor(struct device *dev,
				struct device_attribute *attr, const char *buf, size_t count)
{
	struct stmvl6180_data *vl6180_data = vl6180_data_g;
	unsigned long val = simple_strtoul(buf, NULL, 10);
 	unsigned long flags;
	int rc = 0;

	CDBG("enable %ld\n", val);
	if ((val != 0) && (val != 1)) {
		pr_err("%s:%d store unvalid value=%ld\n", __func__, __LINE__, val);
		return count;
	}
	mutex_lock(&vl6180_data->work_mutex);

	if(val == 1){
		//turn on p sensor
		if (vl6180_data->enable_ps_sensor==0) {
			stmvl6180_set_enable(client,0); /* Power Off */
			rc = stmvl6180_power_enable(vl6180_data, 1);
			if(rc){
				stmvl6180_power_enable(vl6180_data, 0);
				mutex_unlock(&vl6180_data->work_mutex);
				pr_err("%s:%d stmvl6180 power up failed rc %d\n", __func__, __LINE__, rc);
				return rc;
			}

			rc = stmvl6180_init_client(vl6180_data);
			if(rc){
				stmvl6180_power_enable(vl6180_data, 0);
				mutex_unlock(&vl6180_data->work_mutex);
				pr_err("%s:%d stmvl6180 init failed rc %d\n", __func__, __LINE__, rc);
				return rc;
			}
//			mutex_lock(&vl6180_data->work_mutex);

			//re-init
			VL6180x_Prepare(vl6180x_dev);
			VL6180x_UpscaleSetScaling(vl6180x_dev, 3);

			//set parameters
			//VL6180x_RangeSetInterMeasPeriod(vl6180x_dev, 10); //10ms
			//set interrupt mode
			//VL6180x_RangeSetupGPIO1(vl6180x_dev, GPIOx_SELECT_GPIO_INTERRUPT_OUTPUT, INTR_POL_HIGH);
			VL6180x_RangeConfigInterrupt(vl6180x_dev, CONFIG_GPIO_INTERRUPT_NEW_SAMPLE_READY);

			//start
			VL6180x_RangeSetSystemMode(vl6180x_dev, MODE_START_STOP|MODE_SINGLESHOT);
			vl6180_data->ps_is_singleshot = 1;
			vl6180_data->enable_ps_sensor= 1;

			/* we need this polling timer routine for house keeping*/
			spin_lock_irqsave(&vl6180_data->update_lock.wait_lock, flags);
			/*
			 * If work is already scheduled then subsequent schedules will not
			 * change the scheduled time that's why we have to cancel it first.
			 */
			CDBG("cancel_delayed_work\n");
			cancel_delayed_work(&vl6180_data->dwork);
			//schedule_delayed_work(&data->dwork, msecs_to_jiffies(INT_POLLING_DELAY));
			CDBG("schedule_delayed_work\n");
			schedule_delayed_work(&vl6180_data->dwork, msecs_to_jiffies(vl6180_data->delay_ms));
			spin_unlock_irqrestore(&vl6180_data->update_lock.wait_lock, flags);

			stmvl6180_set_enable(client, 1); /* Power On */
//		 	mutex_unlock(&vl6180_data->work_mutex);
		}
	}
	else {
		if (vl6180_data->enable_ps_sensor==1) { //#tt999
		//turn off p sensor
//	 	mutex_lock(&vl6180_data->work_mutex);
		vl6180_data->enable_ps_sensor = 0;
		if (vl6180_data->ps_is_singleshot == 0)
			VL6180x_RangeSetSystemMode(vl6180x_dev, MODE_START_STOP);
		VL6180x_RangeClearInterrupt(vl6180x_dev);

		stmvl6180_set_enable(client, 0);

		spin_lock_irqsave(&vl6180_data->update_lock.wait_lock, flags);
		/*
		 * If work is already scheduled then subsequent schedules will not
		 * change the scheduled time that's why we have to cancel it first.
		 */
		CDBG("cancel_delayed_work\n");
		cancel_delayed_work(&vl6180_data->dwork);
		spin_unlock_irqrestore(&vl6180_data->update_lock.wait_lock, flags);
// 		mutex_unlock(&vl6180_data->work_mutex);
//		kelong need to check
		rc = stmvl6180_power_enable(vl6180_data_g, 0);
		if(rc) {
			pr_err("failed rc %d\n", rc);
//			return rc;
		}
		}
	}
	mutex_unlock(&vl6180_data->work_mutex);

	return count;
}

static DEVICE_ATTR(enable_ps_sensor, S_IWUGO | S_IRUGO,
				   stmvl6180_show_enable_ps_sensor, stmvl6180_store_enable_ps_sensor);

static ssize_t stmvl6180_show_enable_debug(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct stmvl6180_data *vl6180_data = vl6180_data_g;

	return sprintf(buf, "%d\n", vl6180_data->enableDebug);
}

//for als integration time setup
static ssize_t stmvl6180_store_enable_debug(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	struct stmvl6180_data *vl6180_data = vl6180_data_g;
	long on = simple_strtol(buf, NULL, 10);
	if ((on !=0) &&  (on !=1)){
		pr_err("%s: set debug=%ld\n", __func__, on);
		return count;
	}
	vl6180_data->enableDebug=on;

	return count;
}

//DEVICE_ATTR(name,mode,show,store)
static DEVICE_ATTR(enable_debug, S_IWUSR | S_IRUGO,
				   stmvl6180_show_enable_debug, stmvl6180_store_enable_debug);


static ssize_t stmvl6180_show_set_delay_ms(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct stmvl6180_data *vl6180_data = vl6180_data_g;

	return sprintf(buf, "%d\n", vl6180_data->delay_ms);
}

//for als integration time setup
static ssize_t stmvl6180_store_set_delay_ms(struct device *dev,
					struct device_attribute *attr, const char *buf, size_t count)
{
	struct stmvl6180_data *vl6180_data = vl6180_data_g;
	long delay_ms = simple_strtol(buf, NULL, 10);

	CDBG("stmvl6180_store_set_delay_ms as %ld\n",delay_ms);
	if (delay_ms == 0){
		pr_err("%s: set delay_ms=%ld\n", __func__, delay_ms);
		return count;
	}
 	mutex_lock(&vl6180_data->work_mutex);
	vl6180_data->delay_ms=delay_ms;
	mutex_unlock(&vl6180_data->work_mutex);
	return count;
}

static DEVICE_ATTR(set_delay_ms, S_IWUSR | S_IRUGO,
				   stmvl6180_show_set_delay_ms, stmvl6180_store_set_delay_ms);

static struct attribute *stmvl6180_attributes[] = {
	&dev_attr_enable_ps_sensor.attr,
	&dev_attr_enable_debug.attr,
	&dev_attr_set_delay_ms.attr ,
	NULL
};

static const struct attribute_group stmvl6180_attr_group = {
	.attrs = stmvl6180_attributes,
};

/*
 * misc device file operation functions
 */
static int stmvl6180_ioctl_handler(struct file *file,
				unsigned int cmd, unsigned long arg, void __user *p)
{
	int rc=0;
 	unsigned long flags;
	unsigned long distance=0;
	struct i2c_client *client;

	switch (cmd) {
	case VL6180_IOCTL_INIT:	   /* init.  */
	{
		struct stmvl6180_data *vl6180_data = vl6180_data_g;
		client = i2c_getclient();

		CDBG("ioclt INIT\n");
		//turn on p sensor only if it's not enabled by other client
		if (vl6180_data->enable_ps_sensor==0) {
			stmvl6180_set_enable(client,0); /* Power Off */

			//re-init
			VL6180x_Prepare(vl6180x_dev);
			VL6180x_UpscaleSetScaling(vl6180x_dev, 3);
#if VL6180x_WRAP_AROUND_FILTER_SUPPORT
			VL6180x_FilterSetState(vl6180x_dev, 1); // turn on wrap around filter
#endif
			//set parameters
			//VL6180x_RangeSetInterMeasPeriod(vl6180x_dev, 10); //10ms
			//set interrupt mode
			//VL6180x_RangeSetupGPIO1(vl6180x_dev, GPIOx_SELECT_GPIO_INTERRUPT_OUTPUT, INTR_POL_HIGH);

			VL6180x_RangeConfigInterrupt(vl6180x_dev, CONFIG_GPIO_INTERRUPT_NEW_SAMPLE_READY);
			VL6180x_RangeClearInterrupt(vl6180x_dev);

			//start
			//range_set_systemMode(client->addr, RANGE_START_SINGLESHOT);
			//data->ps_is_singleshot = 1;
			VL6180x_RangeSetSystemMode(vl6180x_dev, MODE_START_STOP|MODE_SINGLESHOT);
			vl6180_data->ps_is_singleshot = 1;
			vl6180_data->enable_ps_sensor= 1;

			/* we need this polling timer routine for house keeping*/
			spin_lock_irqsave(&vl6180_data->update_lock.wait_lock, flags);
			/*
			 * If work is already scheduled then subsequent schedules will not
			 * change the scheduled time that's why we have to cancel it first.
			 */
			CDBG("cancel_delayed_work\n");

			cancel_delayed_work(&vl6180_data->dwork);
			//schedule_delayed_work(&data->dwork, msecs_to_jiffies(INT_POLLING_DELAY));
			CDBG("schedule_delayed_work\n");

			schedule_delayed_work(&vl6180_data->dwork, msecs_to_jiffies(vl6180_data->delay_ms));
			spin_unlock_irqrestore(&vl6180_data->update_lock.wait_lock, flags);

			stmvl6180_set_enable(client, 1); /* Power On */
		}

		return 0;
	}
	case VL6180_IOCTL_XTALKCALB: 	/*crosstalk calibration*/
	{
		struct stmvl6180_data *vl6180_data = vl6180_data_g;
		client = i2c_getclient();
		CDBG("ioclt VL6180_IOCTL_XTALKCALB\n");

		//turn on p sensor only if it's not enabled by other client
		if (vl6180_data->enable_ps_sensor==0) {
			stmvl6180_set_enable(client,0); /* Power Off */
			//re-init
			VL6180x_Prepare(vl6180x_dev);
			VL6180x_UpscaleSetScaling(vl6180x_dev, 3);
#if VL6180x_WRAP_AROUND_FILTER_SUPPORT
			VL6180x_FilterSetState(vl6180x_dev, 1); // turn off wrap around filter
#endif
			VL6180x_RangeConfigInterrupt(vl6180x_dev, CONFIG_GPIO_INTERRUPT_NEW_SAMPLE_READY);
			VL6180x_RangeClearInterrupt(vl6180x_dev);
			VL6180x_WrWord(vl6180x_dev, SYSRANGE_CROSSTALK_COMPENSATION_RATE, 0);

			//start
			VL6180x_RangeSetSystemMode(vl6180x_dev, MODE_START_STOP|MODE_SINGLESHOT);
			vl6180_data->ps_is_singleshot = 1;
			vl6180_data->enable_ps_sensor= 1;

			/* we need this polling timer routine for house keeping*/
			spin_lock_irqsave(&vl6180_data->update_lock.wait_lock, flags);
			/*
			 * If work is already scheduled then subsequent schedules will not
			 * change the scheduled time that's why we have to cancel it first.
			 */
			CDBG("cancel_delayed_work\n");

			cancel_delayed_work(&vl6180_data->dwork);
			//schedule_delayed_work(&data->dwork, msecs_to_jiffies(INT_POLLING_DELAY));
			CDBG("schedule_delayed_work\n");

			schedule_delayed_work(&vl6180_data->dwork, msecs_to_jiffies(vl6180_data->delay_ms));
			spin_unlock_irqrestore(&vl6180_data->update_lock.wait_lock, flags);

			stmvl6180_set_enable(client, 1); /* Power On */
		}

		return 0;
	}
	case VL6180_IOCTL_SETXTALK:
	{
		unsigned int xtalkint=0;
		CDBG("ioctl SETXTALK as 0x%x\n", xtalkint);

		if (copy_from_user(&xtalkint, (unsigned int *)p, sizeof(unsigned int))) {
			rc = -EFAULT;
		}

#ifdef CALIBRATION_FILE
		xtalk_calib = xtalkint;
		stmvl6180_write_xtalk_calibration_file();
#endif
		VL6180x_SetXTalkCompensationRate(vl6180x_dev, xtalkint);

		return 0;
	}
	case VL6180_IOCTL_OFFCALB: 	/*offset calibration*/
	{
		struct stmvl6180_data *vl6180_data = vl6180_data_g;
		client = i2c_getclient();

		CDBG("ioclt OFFCALB to enable PS sensor for offset calibration\n");

		//turn on p sensor only if it's not enabled by other client
		if (vl6180_data->enable_ps_sensor==0) {
			stmvl6180_set_enable(client,0); /* Power Off */
			//re-init
			VL6180x_Prepare(vl6180x_dev);
			VL6180x_UpscaleSetScaling(vl6180x_dev, 1);
#if VL6180x_WRAP_AROUND_FILTER_SUPPORT
			VL6180x_FilterSetState(vl6180x_dev, 0); // turn off wrap around filter
#endif
			VL6180x_RangeConfigInterrupt(vl6180x_dev, CONFIG_GPIO_INTERRUPT_NEW_SAMPLE_READY);
			VL6180x_RangeClearInterrupt(vl6180x_dev);
			VL6180x_WrWord(vl6180x_dev, SYSRANGE_PART_TO_PART_RANGE_OFFSET, 0);
			VL6180x_WrWord(vl6180x_dev, SYSRANGE_CROSSTALK_COMPENSATION_RATE, 0);

			//start
			VL6180x_RangeSetSystemMode(vl6180x_dev, MODE_START_STOP|MODE_SINGLESHOT);
			vl6180_data->ps_is_singleshot = 1;
			vl6180_data->enable_ps_sensor= 1;

			/* we need this polling timer routine for house keeping*/
			spin_lock_irqsave(&vl6180_data->update_lock.wait_lock, flags);
			/*
			 * If work is already scheduled then subsequent schedules will not
			 * change the scheduled time that's why we have to cancel it first.
			 */
			CDBG("cancel_delayed_work\n");

			cancel_delayed_work(&vl6180_data->dwork);
			//schedule_delayed_work(&data->dwork, msecs_to_jiffies(INT_POLLING_DELAY));
			CDBG("schedule_delayed_work\n");

			schedule_delayed_work(&vl6180_data->dwork, msecs_to_jiffies(vl6180_data->delay_ms));
			spin_unlock_irqrestore(&vl6180_data->update_lock.wait_lock, flags);

			stmvl6180_set_enable(client, 1); /* Power On */
		}

		return 0;
	}
	case VL6180_IOCTL_SETOFFSET:
	{
		int8_t offsetint=0;
		CDBG("ioctl SETOFFSET as %d\n", offsetint);

		if (copy_from_user(&offsetint, (int8_t *)p, sizeof(int8_t))) {
			rc = -EFAULT;
		}

#ifdef CALIBRATION_FILE
		offset_calib = offsetint;
		stmvl6180_write_offset_calibration_file();
#endif
		VL6180x_SetOffset(vl6180x_dev,offsetint);

		return 0;
	}
	case VL6180_IOCTL_STOP:
	{
		struct stmvl6180_data *vl6180_data = vl6180_data_g;
		client = i2c_getclient();
		CDBG("ioclt VL6180_IOCTL_STOP\n");

		//turn off p sensor only if it's enabled by other client
		if (vl6180_data->enable_ps_sensor==1) {
			//turn off p sensor
			vl6180_data->enable_ps_sensor = 0;
			if (vl6180_data->ps_is_singleshot == 0)
				VL6180x_RangeSetSystemMode(vl6180x_dev, MODE_START_STOP);
			VL6180x_RangeClearInterrupt(vl6180x_dev);

			stmvl6180_set_enable(client, 0);

			spin_lock_irqsave(&vl6180_data->update_lock.wait_lock, flags);
			/*
			* If work is already scheduled then subsequent schedules will not
			* change the scheduled time that's why we have to cancel it first.
			*/
			CDBG("cancel_delayed_work\n");
			cancel_delayed_work(&vl6180_data->dwork);

			spin_unlock_irqrestore(&vl6180_data->update_lock.wait_lock, flags);
		}

		return 0;
	}
	case VL6180_IOCTL_GETDATA:	  /* Get proximity value only */
	{
		struct stmvl6180_data *vl6180_data = vl6180_data_g;
		CDBG("vl6180_getDistance return %ld\n",distance);

		mutex_lock(&vl6180_data->work_mutex);
		distance = vl6180_data->rangeData.FilteredData.range_mm;
		mutex_unlock(&vl6180_data->work_mutex);
		rc = put_user(distance, (unsigned long *)p);

		return rc;
	}
	case VL6180_IOCTL_GETDATAS:	 /* Get all range data */
	{
		struct stmvl6180_data *vl6180_data = vl6180_data_g;
		CDBG("IOCTL_GETDATAS, m_range_mm:%d\n", vl6180_data->rangeData.range_mm);

		mutex_lock(&vl6180_data->work_mutex);
		rc = copy_to_user((VL6180x_RangeData_t *)p, &(vl6180_data->rangeData), sizeof(VL6180x_RangeData_t));
		mutex_unlock(&vl6180_data->work_mutex);
		return rc;
	}
	default:
		return -EINVAL;
	}

	return rc;
}

static int stmvl6180_open(struct inode *inode, struct file *file)
{
	int rc = 0;
	if(vl6180_data_g&&(!vl6180_data_g->enable)){
		rc = stmvl6180_power_enable(vl6180_data_g, 1);
		if(rc){
			pr_err("%s:%d %d vl6180 power up failed\n", __func__, __LINE__, rc);
		}

		return rc;
	}else if(vl6180_data_g->enable){
		pr_err("%s:%d vl6180 was already opened, return false\n", __func__, __LINE__);
		return -1;
	}

	return -1;
}

static int stmvl6180_flush(struct file *file, fl_owner_t id)
{
 	unsigned long flags;
	struct i2c_client *client;
	struct stmvl6180_data *vl6180_data = vl6180_data_g;
	client = i2c_getclient();

	if (vl6180_data->enable_ps_sensor==1){
//turn off p sensor if it's enabled
		vl6180_data->enable_ps_sensor = 0;
		VL6180x_RangeClearInterrupt(vl6180x_dev);

		stmvl6180_set_enable(client, 0);

		spin_lock_irqsave(&vl6180_data->update_lock.wait_lock, flags);
/*
* If work is already scheduled then subsequent schedules will not
* change the scheduled time that's why we have to cancel it first.
*/
		CDBG("cancel_delayed_work\n");
		cancel_delayed_work(&vl6180_data->dwork);
		spin_unlock_irqrestore(&vl6180_data->update_lock.wait_lock, flags);
	}

	return 0;
}
static long stmvl6180_ioctl(struct file *file,
				unsigned int cmd, unsigned long arg)
{
	int ret;
	mutex_lock(&vl6180_mutex);
	ret = stmvl6180_ioctl_handler(file, cmd, arg, (void __user *)arg);
	mutex_unlock(&vl6180_mutex);
	return ret;
}

static int stmvl6180_release(struct inode *inode, struct file *file)
{
	int rc = 0;

    	if(vl6180_data_g){
		rc = stmvl6180_power_enable(vl6180_data_g, 0);
		if(rc<0)
	   		pr_err("%s:%d stmvl6180 power down failed %d\n", __func__, __LINE__, rc);
	}

	return 0;
}

static const struct file_operations stmvl6180_ranging_fops = {
		.owner =			THIS_MODULE,
		.unlocked_ioctl =	stmvl6180_ioctl,
		.open =			stmvl6180_open,
		.flush = 			stmvl6180_flush,
		.release = 		stmvl6180_release,
};

static struct miscdevice stmvl6180_ranging_dev = {
		.minor =	MISC_DYNAMIC_MINOR,
		.name =		"stmvl6180_ranging",
		.fops =		&stmvl6180_ranging_fops
};

//cci driver related code
static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_seq = msm_camera_cci_i2c_write_seq,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay = msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_poll =  msm_camera_cci_i2c_poll,
};
#ifdef VENDOR_EDIT
static long msm_stmvl6180_subdev_ioctl(struct v4l2_subdev *sd,
			unsigned int cmd, void *arg)
{
	unsigned long flags;
	struct i2c_client *client;
	struct stmvl6180_data *vl6180_data = vl6180_data_g;

	CDBG("%s:%d Enter\n", __func__, __LINE__);
//	CDBG("%s:%d o_ctrl %p argp %p\n", __func__, __LINE__, o_ctrl, argp);
	switch (cmd) {
	case MSM_SD_SHUTDOWN:
		client = i2c_getclient();
		CDBG("ioclt VL6180_IOCTL_STOP\n");
		mutex_lock(&vl6180_data->work_mutex);

		//turn off p sensor only if it's enabled by other client
		if (vl6180_data->enable_ps_sensor==1) {
			//turn off p sensor
			vl6180_data->enable_ps_sensor = 0;
			if (vl6180_data->ps_is_singleshot == 0)
				VL6180x_RangeSetSystemMode(vl6180x_dev, MODE_START_STOP);
			VL6180x_RangeClearInterrupt(vl6180x_dev);

			stmvl6180_set_enable(client, 0);

			spin_lock_irqsave(&vl6180_data->update_lock.wait_lock, flags);
			CDBG("cancel_delayed_work\n");
			cancel_delayed_work(&vl6180_data->dwork);
			spin_unlock_irqrestore(&vl6180_data->update_lock.wait_lock, flags);
		}
		mutex_unlock(&vl6180_data->work_mutex);
	default:
		return -ENOIOCTLCMD;
	}
}
static int32_t msm_stmvl6180_power(struct v4l2_subdev *sd, int on)
{
	int rc = 0;
	if(vl6180_data_g){
		rc = stmvl6180_power_enable(vl6180_data_g, on);
		if(rc<0)
			pr_err("%s:%d stmvl6180 power down failed %d\n", __func__, __LINE__, rc);
	}
	return rc;
}
static struct v4l2_subdev_core_ops msm_stmvl6180_subdev_core_ops = {
	.ioctl = msm_stmvl6180_subdev_ioctl,
	.s_power = msm_stmvl6180_power,
};
static struct v4l2_subdev_ops msm_stmvl6180_subdev_ops = {
	.core = &msm_stmvl6180_subdev_core_ops,
};
#endif
/*
 * Initialization function
 */
static int stmvl6180_init_client(struct stmvl6180_data *vl6180_data)
{
	uint8_t id=0,module_major=0,module_minor=0;
	uint8_t model_major=0,model_minor=0;
	uint8_t i=0,val;

	while(vl6180_data->force_reset_cnt++ < 3){
		// Read Model ID
		VL6180x_RdByte(vl6180x_dev, VL6180_MODEL_ID_REG, &id);
		CDBG("read MODLE_ID: 0x%x, i2cAddr:0x%x\n", id, vl6180_data->i2c_client.cci_client->sid);
		if (id == 0xb4) {
			pr_err("STM VL6180 Found\n");
			break;
		}
		else{
			if((vl6180_data->act_device_type == MSM_CAMERA_I2C_DEVICE) && (vl6180_data->force_reset_cnt < 3)){// CCI device didn't need this, as the retries paramter of cci is 3.
				pr_err("Not found STM VL6180, will reset and try again\n");
				stmvl6180_power_enable(vl6180_data, 0);
				msleep(10);
				stmvl6180_power_enable(vl6180_data, 1);
			}
			else if(vl6180_data->act_device_type == MSM_CAMERA_PLATFORM_DEVICE)
				break;
		}
	}
	if (id != 0xb4){
		vl6180_data->force_reset_cnt = 0;
		pr_err("Not found STM VL6180\n");
		return -EIO;
	}

	// Read Model Version
	VL6180x_RdByte(vl6180x_dev, VL6180_MODEL_REV_MAJOR_REG, &model_major);
	model_major &= 0x07;
	VL6180x_RdByte(vl6180x_dev, VL6180_MODEL_REV_MINOR_REG, &model_minor);
	model_minor &= 0x07;
	CDBG("STM VL6180 Model Version : %d.%d\n", model_major,model_minor);

	// Read Module Version
	VL6180x_RdByte(vl6180x_dev, VL6180_MODULE_REV_MAJOR_REG, &module_major);
	VL6180x_RdByte(vl6180x_dev, VL6180_MODULE_REV_MINOR_REG, &module_minor);
	CDBG("STM VL6180 Module Version : %d.%d\n", module_major,module_minor);

	// Read Identification
	printk("STM VL6180 Serial Numbe: ");
	for (i=0; i<=(VL6180_FIRMWARE_REVISION_ID_REG-VL6180_REVISION_ID_REG);i++)
	{
		VL6180x_RdByte(vl6180x_dev, (VL6180_REVISION_ID_REG+i), &val);
		printk("0x%x-",val);
	}
	printk("\n");


	vl6180_data->delay_ms=20; //init to 20ms
	vl6180_data->ps_data=0;
	vl6180_data->enableDebug=0;
	vl6180_data->force_reset_cnt = 0;
#ifdef CALIBRATION_FILE
	stmvl6180_read_calibration_file();
#endif

	//VL6180 Initialization
	VL6180x_WaitDeviceBooted(vl6180x_dev);
	VL6180x_InitData(vl6180x_dev);
	//VL6180x_FilterSetState(vl6180x_dev, 1); /* activate wrap around filter */
	//VL6180x_DisableGPIOxOut(vl6180x_dev, 1); /* diable gpio 1 output, not needed when polling */

	return 0;
}

int stmvl6180_power_enable(struct stmvl6180_data *vl6180_data, unsigned int enable)
{
	int rc = 0;

	CDBG("enable %d\n", enable);

	if(enable) {
#ifdef CALIBRATION_FILE
		stmvl6180_read_calibration_file();// read calibration file while power up Laser sensor
#endif

		if(vl6180_data->enable == 0){
			rc = gpio_request(vl6180_data->ce_gpio, "vl_6180-ce");
			if (rc < 0){
				pr_err("%s:%d gpio request failed for vl6180 rc = %d\n", __func__, __LINE__, rc);
				return rc;
			}
			rc = gpio_request(vl6180_data->irq_gpio,"vl_6180-int");
			if(rc < 0){
				pr_err("%s:%d gpio_request failed, rc=%d\n", __func__, __LINE__, rc);
				gpio_free(vl6180_data->ce_gpio);
				return rc;
			}

			if (regulator_count_voltages(vl6180_data->vdd_regulator) > 0){
				rc = regulator_set_voltage(vl6180_data->vdd_regulator, 2850000, 2850000);
				if (rc) {
					pr_err( "%s:%d vdd regulator set failed rc=%d\n", __func__, __LINE__, rc);
					gpio_free(vl6180_data->ce_gpio);
					gpio_free(vl6180_data->irq_gpio);
					return rc;
				}
			}
			rc = regulator_enable(vl6180_data->vdd_regulator);
			if (rc) {
				pr_err("%s:%d stmvl6180 enable vdd_regulator failed rc=%d\n", __func__, __LINE__, rc);
				gpio_free(vl6180_data->ce_gpio);
				gpio_free(vl6180_data->irq_gpio);
				return rc;
			}

			if (regulator_count_voltages(vl6180_data->vdd_regulator_i2c) > 0) {
				rc = regulator_set_voltage(vl6180_data->vdd_regulator_i2c, 1800000, 1800000);
				if (rc) {
					pr_err( "%s:%d vdd_regulator_i2c set failed rc=%d\n", __func__, __LINE__, rc);
					gpio_free(vl6180_data->ce_gpio);
					gpio_free(vl6180_data->irq_gpio);
					regulator_disable(vl6180_data->vdd_regulator);
					return rc;
				}
			}
			rc = regulator_enable(vl6180_data->vdd_regulator_i2c);
			if(rc){
				pr_err("%s:%d stmvl6180 power on i2c regulator failed rc=%d \n", __func__, __LINE__, rc);
				gpio_free(vl6180_data->ce_gpio);
				gpio_free(vl6180_data->irq_gpio);
				regulator_disable(vl6180_data->vdd_regulator);
				return rc;
			}

		//just cci device need this
		if (vl6180_data->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
			rc = vl6180_data->i2c_client.i2c_func_tbl->i2c_util(&vl6180_data->i2c_client, MSM_CCI_INIT);
			if (rc < 0){
				pr_err("%s:%d cci_init failed rc=%d\n", __func__, __LINE__, rc);
				gpio_free(vl6180_data->ce_gpio);
				gpio_free(vl6180_data->irq_gpio);
				regulator_disable(vl6180_data->vdd_regulator_i2c);
				regulator_disable(vl6180_data->vdd_regulator);
				return rc;
			}
		}
		msleep(5);
#if 0
		rc = gpio_direction_output(vl6180_data->ce_gpio,1);
		if(rc){
			pr_err("%s:%d stmvl6180 gpip output failed rc=%d\n", __func__, __LINE__, rc);
					gpio_free(vl6180_data->ce_gpio);
					gpio_free(vl6180_data->irq_gpio);
			regulator_disable(vl6180_data->vdd_regulator_i2c);
			regulator_disable(vl6180_data->vdd_regulator);
			return rc;
		}
		msleep(1);
		rc = gpio_direction_output(vl6180_data->ce_gpio,0);
		if(rc){
			pr_err("%s:%d stmvl6180 gpip output failed rc=%d\n", __func__, __LINE__, rc);
					gpio_free(vl6180_data->ce_gpio);
					gpio_free(vl6180_data->irq_gpio);
			regulator_disable(vl6180_data->vdd_regulator_i2c);
			regulator_disable(vl6180_data->vdd_regulator);
			return rc;
		}
		msleep(1);
#endif
		rc = gpio_direction_output(vl6180_data->ce_gpio,1);
		if(rc){
			pr_err("%s:%d stmvl6180 gpip output failed rc=%d\n", __func__, __LINE__, rc);
			gpio_free(vl6180_data->ce_gpio);
			gpio_free(vl6180_data->irq_gpio);
			regulator_disable(vl6180_data->vdd_regulator_i2c);
			regulator_disable(vl6180_data->vdd_regulator);
			return rc;
		}
		msleep(1);
		vl6180_data->enable = 1;
		}
	} else {
		if(vl6180_data->enable){
			rc = gpio_direction_output(vl6180_data->ce_gpio,0);
			if(rc < 0){
				pr_err("%s:%d stmvl6180 ce_gpio output failed %d\n", __func__, __LINE__, rc);
//				return -1;
			}

//just cci device need this
			if (vl6180_data->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
//				mutex_lock(&vl6180_data->work_mutex);
				rc = vl6180_data->i2c_client.i2c_func_tbl->i2c_util(&vl6180_data->i2c_client, MSM_CCI_RELEASE);
//				mutex_unlock(&vl6180_data->work_mutex);
				if (rc < 0){
					pr_err("%s:%d cci_release failed\n", __func__, __LINE__);
//					return -1;
				}
			}

			if (vl6180_data->vdd_regulator_i2c) {
				rc = regulator_disable(vl6180_data->vdd_regulator_i2c);
				if(rc){
					pr_err("%s:%d stmvl6180 vdd_regulator_i2c disable failed %d\n", __func__, __LINE__, rc);
//					return -1;
				}
			}

			if (vl6180_data->vdd_regulator) {
				rc = regulator_disable(vl6180_data->vdd_regulator);
				if(rc){
					pr_err("%s:%d stmvl6180 vdd_regulator disable failed %d\n", __func__, __LINE__, rc);
			//		return -1;
				}
			}

			if(gpio_is_valid(vl6180_data->ce_gpio)){
					gpio_free(vl6180_data->ce_gpio);
			}
			if(gpio_is_valid(vl6180_data->irq_gpio)){
					gpio_free(vl6180_data->irq_gpio);
			}

			vl6180_data->enable = 0;
			vl6180_data->enable_ps_sensor = 0;
		}
	}
	return rc;
}

static int stmvl6180_parse_dt(struct device *dev, struct stmvl6180_data *vl6180_data)
{
	int ret = 0;

	if (dev->of_node) {
		struct device_node *np = dev->of_node;

		/* reset, irq gpio info */
		vl6180_data->irq_gpio    = of_get_named_gpio(np, "st,irq-gpio", 0);
		if( vl6180_data->irq_gpio < 0 ) {
			pr_err("%s:%d get vl6180 irq_gpio failed\n", __func__, __LINE__);
			return -1;
		}
		vl6180_data->ce_gpio  = of_get_named_gpio(np, "st,standby-gpio", 0);
		if( vl6180_data->ce_gpio < 0 ) {
			pr_err("%s:%d get vl6180 ce gpio fained\n", __func__, __LINE__);
			return -1;
		}
		CDBG("irq_gpio:%d ce_gpio:%d\n", vl6180_data->irq_gpio, vl6180_data->ce_gpio);

		vl6180_data->vdd_regulator = regulator_get(dev, "vdd_1v8");
		if(IS_ERR(vl6180_data->vdd_regulator)) {
			pr_err("%s:%d Regulator vdd_1v8 get failed\n", __func__, __LINE__);
			gpio_free(vl6180_data->ce_gpio);
			gpio_free(vl6180_data->irq_gpio);
			return -1;
		}

		vl6180_data->vdd_regulator_i2c = regulator_get(dev, "vcc_i2c_1v8");
		if(IS_ERR(vl6180_data->vdd_regulator_i2c) ) {
			pr_err("%s:%d Regulator vcc_i2c_1v8 get failed\n", __func__, __LINE__);
			regulator_put(vl6180_data->vdd_regulator);
			gpio_free(vl6180_data->ce_gpio);
			gpio_free(vl6180_data->irq_gpio);
			return -1;
		}
	}else{
		pr_err("%s:%d of_node is NULL\n", __func__, __LINE__);
		return -EFAULT;
	}
	return ret;
}

static int32_t stmvl6180_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	struct msm_camera_cci_client *cci_client = NULL;
	struct stmvl6180_data *vl6180_data = NULL;
	CDBG("Enter\n");

	if (!pdev->dev.of_node) {
		pr_err("%s:%d of_node NULL\n", __func__, __LINE__);
		return -EINVAL;
	}

	vl6180_data = kzalloc(sizeof(struct stmvl6180_data), GFP_KERNEL);
	if (!vl6180_data) {
		pr_err("%s:%d kzalloc failed\n", __func__, __LINE__);
		return -ENOMEM;
	}
	vl6180_data_g = vl6180_data;

//new driver start
	vl6180_data->enable = 0;		/* default mode is standard */
	CDBG("enable = %x\n", vl6180_data->enable);

	mutex_init(&vl6180_data->update_lock);
	mutex_init(&vl6180_data->work_mutex);
	mutex_init(&vl6180_mutex);

#ifdef USE_INT
	gpio_request(IRQ_NUM,"vl6180_gpio_int");
	gpio_direction_input(IRQ_NUM);
	irq = gpio_to_irq(IRQ_NUM);
	if (irq < 0)
	{
		pr_err("%s:%d filed to map GPIO :%d to interrupt:%d\n", __func__, __LINE__, IRQ_NUM, irq);
	}
	else
	{
		int result;
		vl6180_dbgmsg("%s:%d register_irq:%d\n",__func__, __LINE__, irq);
		if ((result = request_threaded_irq(irq, NULL, stmvl6180_interrupt_handler, IRQF_TRIGGER_RISING, //IRQF_TRIGGER_FALLING- poliarity:0 IRQF_TRIGGER_RISNG - poliarty:1
			"vl6180_lb_gpio_int", (void *)client)))
		{
			pr_err("%s:%d Could not allocate STMVL6180_INT ! result:%d\n", __func__, __LINE__, result);

			goto exit_kfree;
		}
	}
	//disable_irq(irq);
	vl6180_data->irq = irq;
	vl6180_dbgmsg("%s:%d interrupt is hooked\n", __func__, __LINE__);
#endif
//new driver end

	rc = of_property_read_u32(pdev->dev.of_node, "cell-index", &pdev->id);
	if (rc < 0) {
		pr_err("%s:%d failed rc %d\n", __func__, __LINE__, rc);
		goto exit_free_irq;
	}
	CDBG("cell-index %d, rc %d\n", pdev->id, rc);

	rc = of_property_read_u32(pdev->dev.of_node, "qcom,cci-master", &vl6180_data->cci_master);
	if (rc < 0) {
		pr_err("%s:%d failed rc %d\n", __func__, __LINE__, rc);
		goto exit_free_irq;
	}
	CDBG("qcom,cci-master %d, rc %d\n", vl6180_data->cci_master, rc);

	rc = stmvl6180_parse_dt(&pdev->dev, vl6180_data);
	if (rc < 0) {
		pr_err("%s:%d failed rc %d\n", __func__, __LINE__, rc);
		goto exit_free_irq;
	}

	/* Set platform device handle */
	vl6180_data->pdev = pdev;
	vl6180_data->subdev_id = pdev->id;

	/* Set device type as platform device */
	vl6180_data->act_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	vl6180_data->i2c_client.i2c_func_tbl = &msm_sensor_cci_func_tbl;
	vl6180_data->v4l2_subdev_ops = &msm_stmvl6180_subdev_ops;
	vl6180_data->i2c_client.cci_client = kzalloc(sizeof(struct msm_camera_cci_client), GFP_KERNEL);
	if (!vl6180_data->i2c_client.cci_client) {
		rc = -ENOMEM;
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		goto exit_free_irq;
	}

	cci_client = vl6180_data->i2c_client.cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->sid = VL6180_I2C_ADDRESS;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	cci_client->cci_i2c_master = vl6180_data->cci_master;
	vl6180_data->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	vl6180_data->force_reset_cnt = 0;

	rc = stmvl6180_power_enable(vl6180_data, 1);
	if(rc){
		pr_err("%s:%d failed rc %d\n", __func__, __LINE__, rc);
		goto exit_kfree_client;
	}

	/* Initialize the STM VL6180 chip */
	rc = stmvl6180_init_client(vl6180_data);
	if (rc){
		stmvl6180_power_enable(vl6180_data, 0);
		pr_err("%s:%d failed rc %d\n", __func__, __LINE__, rc);
		goto exit_kfree_client;
	}

	rc = stmvl6180_power_enable(vl6180_data, 0);
	if(rc){
		pr_err("%s:%d failed rc %d\n", __func__, __LINE__, rc);
		goto exit_kfree_client;
	}
#ifdef VENDOR_EDIT
	v4l2_subdev_init(&vl6180_data->msm_sd.sd,
		vl6180_data->v4l2_subdev_ops);
	vl6180_data->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(vl6180_data->msm_sd.sd.name,
		ARRAY_SIZE(vl6180_data->msm_sd.sd.name), "msm_laser");
	media_entity_init(&vl6180_data->msm_sd.sd.entity, 0, NULL, 0);
	vl6180_data->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	vl6180_data->msm_sd.close_seq = MSM_SD_CLOSE_1ST_CATEGORY | 0x3;
	msm_sd_register(&vl6180_data->msm_sd);
#endif

	//to register as a misc device
	rc = misc_register(&stmvl6180_ranging_dev);
	if (rc){
		pr_err(KERN_INFO "%s:%d Could not register misc. dev for stmvl6180 ranging\n", __func__, __LINE__);
		goto exit_kfree_client;
	}

//new driver start
/* Register to Input Device */
	vl6180_data->input_dev_ps = input_allocate_device();
	if (!vl6180_data->input_dev_ps) {
		rc = -ENOMEM;
		pr_err("%s:%d Failed to allocate input device ps\n", __func__, __LINE__);
		goto exit_kfree_client;
	}

	vl6180_data->input_dev_ps->name = "STM VL6180 proximity sensor";
	set_bit(EV_ABS, vl6180_data->input_dev_ps->evbit);

	input_set_abs_params(vl6180_data->input_dev_ps, ABS_DISTANCE, 0, 76, 0, 0); //range in cm
	input_set_abs_params(vl6180_data->input_dev_ps, ABS_HAT0X, 0, 0xffffffff, 0, 0); //timeval.tv_sec
	input_set_abs_params(vl6180_data->input_dev_ps, ABS_HAT0Y, 0, 0xffffffff, 0, 0); //timeval.tv_usec
	input_set_abs_params(vl6180_data->input_dev_ps, ABS_HAT1X, 0, 765, 0, 0); //range in mm
	input_set_abs_params(vl6180_data->input_dev_ps, ABS_HAT1Y, 0, 0xffffffff, 0, 0); //errorStatus
	input_set_abs_params(vl6180_data->input_dev_ps, ABS_HAT2X, 0, 0xffffffff, 0, 0); //signal rate (MCPS)
	input_set_abs_params(vl6180_data->input_dev_ps, ABS_HAT2Y, 0, 0xffffffff, 0, 0); //Return Ambient rate in KCPS
	input_set_abs_params(vl6180_data->input_dev_ps, ABS_HAT3X, 0, 0xffffffff, 0, 0); // Return Convergence time
	input_set_abs_params(vl6180_data->input_dev_ps, ABS_HAT3Y, 0, 0xffffffff, 0, 0); //DMax

	rc = input_register_device(vl6180_data->input_dev_ps);
	if (rc<0){
		rc = -ENOMEM;
		pr_err("%s:%d Unable to register input device ps: %s\n",__func__, __LINE__, vl6180_data->input_dev_ps->name);
		goto exit_free_dev_ps;
	}

/* Register sysfs hooks */
	rc = sysfs_create_group(&vl6180_data->pdev->dev.kobj, &stmvl6180_attr_group);
	if (rc<0){
		pr_err("%s%d Unable to create sysfs group\n",__func__, __LINE__);
		goto exit_unregister_dev_ps;
	}

	INIT_DELAYED_WORK(&vl6180_data->dwork, stmvl6180_work_handler);

	CDBG("support ver. %s enabled\n", DRIVER_VERSION);

//new driver end
	return 0;

exit_unregister_dev_ps:
	input_unregister_device(vl6180_data->input_dev_ps);
exit_free_dev_ps:
	input_free_device(vl6180_data->input_dev_ps);
exit_kfree_client:
	kfree(vl6180_data->i2c_client.cci_client);
exit_free_irq:
#ifdef USE_INT
	free_irq(irq, client);
#endif
//exit_kfree:
	kfree(vl6180_data);
	vl6180_data_g = NULL;
//exit:
	return rc;
}


static const struct of_device_id stmvl6180_dt_match[] = {
	{.compatible = "stmv,vl6180", .data=NULL},
	{}
};

static struct platform_driver stmvl6180_platform_driver = {
	.driver = {
		.name = "stmv,vl6180",
		.owner = THIS_MODULE,
		.of_match_table = stmvl6180_dt_match,
	},
};

static int stmvl6180_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct stmvl6180_data *vl6180_data;
	struct msm_camera_cci_client *cci_client = NULL;
	int err = 0;
	CDBG("start\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		pr_err("%s:%d check i2c functionality failed\n", __func__, __LINE__);
		goto exit;
	}

	vl6180_data = kzalloc(sizeof(struct stmvl6180_data), GFP_KERNEL);
	if (!vl6180_data) {
		err = -ENOMEM;
		pr_err("%s:%d kzalloc failed\n", __func__, __LINE__);
		goto exit;
	}

//parse dtsi file
	if (client->dev.of_node) {
		err = stmvl6180_parse_dt(&client->dev, vl6180_data);
		if (err < 0) {
			pr_err("%s:%d parse dt failed err %d\n", __func__, __LINE__, err);
			goto exit_kfree;
		}
	} else {
		pr_err("%s:%d of_node NULL\n", __func__, __LINE__);
		goto exit_kfree;
	}

	vl6180_data->client = client;
	i2c_set_clientdata(client, vl6180_data);
	vl6180_data->enable = 0;		/* default mode is standard */
	vl6180_data_g = vl6180_data;

	// setup platform i2c client
	i2c_setclient(client);
	//vl6180x_dev.I2cAddress = client->addr;

	mutex_init(&vl6180_data->update_lock);
	mutex_init(&vl6180_data->work_mutex);
	mutex_init(&vl6180_mutex);

	//interrupt set up
#ifdef USE_INT
	gpio_request(IRQ_NUM,"vl6180_gpio_int");
	gpio_direction_input(IRQ_NUM);
	irq = gpio_to_irq(IRQ_NUM);
	if (irq < 0)
	{
		pr_err("filed to map GPIO :%d to interrupt:%d\n",IRQ_NUM,irq);
	}
	else
	{
		int result;
		vl6180_dbgmsg("register_irq:%d\n",irq);
		if ((result = request_threaded_irq(irq, NULL, stmvl6180_interrupt_handler, IRQF_TRIGGER_RISING, //IRQF_TRIGGER_FALLING- poliarity:0 IRQF_TRIGGER_RISNG - poliarty:1
			"vl6180_lb_gpio_int", (void *)client)))
		{
			pr_err("%s Could not allocate STMVL6180_INT ! result:%d\n", __func__,result);
			goto exit_kfree;
		}
	}
	//disable_irq(irq);
	vl6180_data->irq = irq;
	vl6180_dbgmsg("%s interrupt is hooked\n", __func__);
#endif

#ifdef VENDOR_EDIT
	vl6180_data->act_device_type = MSM_CAMERA_I2C_DEVICE;//define I2C device
	vl6180_data->i2c_client.i2c_func_tbl = &msm_sensor_cci_func_tbl;
	vl6180_data->v4l2_subdev_ops = &msm_stmvl6180_subdev_ops;
	vl6180_data->i2c_client.cci_client = kzalloc(sizeof(struct msm_camera_cci_client), GFP_KERNEL);
	if (!vl6180_data->i2c_client.cci_client) {
		err = -ENOMEM;
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		goto exit_free_irq;
	}
	cci_client = vl6180_data->i2c_client.cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->sid = VL6180_I2C_ADDRESS;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	cci_client->cci_i2c_master = vl6180_data->cci_master;
	vl6180_data->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;
#endif

	err = stmvl6180_power_enable(vl6180_data, 1);
	if(err) {
		pr_err("%s:%d failed err %d\n", __func__, __LINE__, err);
		goto exit_kfree_client;
	}
	vl6180_data->force_reset_cnt = 0;

	/* Initialize the STM VL6180 chip */
	err = stmvl6180_init_client(vl6180_data);
	if (err) {
		stmvl6180_power_enable(vl6180_data, 0);
		pr_err("%s:%d failed err %d\n", __func__, __LINE__, err);
		goto exit_kfree_client;
	}

	err = stmvl6180_power_enable(vl6180_data, 0);
	if(err) {
		pr_err("%s:%d failed err %d\n", __func__, __LINE__, err);
		goto exit_kfree_client;
	}
	INIT_DELAYED_WORK(&vl6180_data->dwork, stmvl6180_work_handler);
#ifdef VENDOR_EDIT
	v4l2_subdev_init(&vl6180_data->msm_sd.sd,
		vl6180_data->v4l2_subdev_ops);
	vl6180_data->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(vl6180_data->msm_sd.sd.name,
		ARRAY_SIZE(vl6180_data->msm_sd.sd.name), "msm_laser");
	media_entity_init(&vl6180_data->msm_sd.sd.entity, 0, NULL, 0);
	vl6180_data->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	vl6180_data->msm_sd.close_seq = MSM_SD_CLOSE_1ST_CATEGORY | 0x3;
	msm_sd_register(&vl6180_data->msm_sd);
#endif

	//to register as a misc device
	err = misc_register(&stmvl6180_ranging_dev);
	if (err){
		pr_err("%s:%d Could not register misc. dev for stmvl6180 ranging\n", __func__, __LINE__);
		goto exit_kfree_client;
	}

	/* Register to Input Device */
	vl6180_data->input_dev_ps = input_allocate_device();
	if (!vl6180_data->input_dev_ps) {
		err = -ENOMEM;
		pr_err("%s:%d Failed to allocate input device ps\n",__func__, __LINE__);
		goto exit_kfree_client;
	}
	vl6180_data->input_dev_ps->name = "STM VL6180 proximity sensor";
	set_bit(EV_ABS, vl6180_data->input_dev_ps->evbit);

	input_set_abs_params(vl6180_data->input_dev_ps, ABS_DISTANCE, 0, 76, 0, 0); //range in cm
	input_set_abs_params(vl6180_data->input_dev_ps, ABS_HAT0X, 0, 0xffffffff, 0, 0); //timeval.tv_sec
	input_set_abs_params(vl6180_data->input_dev_ps, ABS_HAT0Y, 0, 0xffffffff, 0, 0); //timeval.tv_usec
	input_set_abs_params(vl6180_data->input_dev_ps, ABS_HAT1X, 0, 765, 0, 0); //range in mm
	input_set_abs_params(vl6180_data->input_dev_ps, ABS_HAT1Y, 0, 0xffffffff, 0, 0); //errorStatus
	input_set_abs_params(vl6180_data->input_dev_ps, ABS_HAT2X, 0, 0xffffffff, 0, 0); //signal rate (MCPS)
	input_set_abs_params(vl6180_data->input_dev_ps, ABS_HAT2Y, 0, 0xffffffff, 0, 0); //Return Ambient rate in KCPS
	input_set_abs_params(vl6180_data->input_dev_ps, ABS_HAT3X, 0, 0xffffffff, 0, 0); // Return Convergence time
	input_set_abs_params(vl6180_data->input_dev_ps, ABS_HAT3Y, 0, 0xffffffff, 0, 0); //DMax

	err = input_register_device(vl6180_data->input_dev_ps);
	if (err) {
		err = -ENOMEM;
		pr_err("%s:%d Unable to register input device ps: %s\n",__func__, __LINE__, vl6180_data->input_dev_ps->name);
		goto exit_free_dev_ps;
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &stmvl6180_attr_group);
	if (err){
		pr_err("%s%d Unable to create sysfs group\n",__func__, __LINE__);
		goto exit_unregister_dev_ps;
	}

	CDBG("%s successfully! support ver. %s enabled\n", __func__, DRIVER_VERSION);

	return 0;

exit_unregister_dev_ps:
	input_unregister_device(vl6180_data->input_dev_ps);
exit_free_dev_ps:
	input_free_device(vl6180_data->input_dev_ps);
exit_kfree_client:
	kfree(vl6180_data->i2c_client.cci_client);
exit_free_irq:
#ifdef USE_INT
	free_irq(irq, client);
#endif
exit_kfree:
	kfree(vl6180_data);
exit:
	return err;
}

static int stmvl6180_i2c_remove(struct i2c_client *client)
{
	struct stmvl6180_data *vl6180_data = vl6180_data_g;

	input_unregister_device(vl6180_data->input_dev_ps);
	input_free_device(vl6180_data->input_dev_ps);

#ifdef  USE_INT
	free_irq(vl6180_data->irq, client);
#endif

	sysfs_remove_group(&client->dev.kobj, &stmvl6180_attr_group);

	/* Power down the device */
	stmvl6180_set_enable(client, 0);

	kfree(vl6180_data);
	vl6180_data_g = NULL;

	return 0;
}

#ifdef CONFIG_PM

static int stmvl6180_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return stmvl6180_set_enable(client, 0);
}

static int stmvl6180_i2c_resume(struct i2c_client *client)
{
	return stmvl6180_set_enable(client, 0);
}

#else

#define stmvl6180_i2c_suspend	NULL
#define stmvl6180_i2c_resume	NULL

#endif /* CONFIG_PM */

static const struct i2c_device_id stmvl6180_i2c_id[] = {
	{ "vl6180", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, stmvl6180_id);

static struct i2c_driver stmvl6180_i2c_driver = {
	.driver = {
		.name	= "vl6180",
		.owner	= THIS_MODULE,
	},
	.suspend = stmvl6180_i2c_suspend,
	.resume	= stmvl6180_i2c_resume,
	.probe	= stmvl6180_i2c_probe,
	.remove	= stmvl6180_i2c_remove,
	.id_table = stmvl6180_i2c_id,
};

static int __init stmvl6180_init(void)
{
	int rc=0;
#if 0
	struct i2c_adapter *adapter;
	struct i2c_board_info info = {
		.type = "stmvl6180",
		.addr = VL6180_I2C_ADDRESS,
	};
#endif
	CDBG("start stmvl6180_init\n");
	rc = platform_driver_probe(&stmvl6180_platform_driver, stmvl6180_platform_probe);

	if(rc){
		pr_err("%s:%d platform device register failed, try to register as I2C device\n", __func__, __LINE__);
		rc = i2c_add_driver(&stmvl6180_i2c_driver);
	}

#if 0
	if(ret){
		ret = i2c_add_driver(&stmvl6180_driver);
		if (ret)
			return ret;
		adapter = i2c_get_adapter(4);
		if (!adapter)
			return -EINVAL;

		client = i2c_new_device(adapter, &info);
		if (!client)
			return -EINVAL;
	}
#endif
	return rc;
}

static void __exit stmvl6180_exit(void)
{
	struct stmvl6180_data *vl6180_data = vl6180_data_g;

	CDBG("exit\n");
	input_unregister_device(vl6180_data->input_dev_ps);
	input_free_device(vl6180_data->input_dev_ps);

#ifdef  USE_INT
	free_irq(vl6180_data->irq, client);
#endif

	sysfs_remove_group(&client->dev.kobj, &stmvl6180_attr_group);

	/* Power down the device */
	stmvl6180_set_enable(client, 0);

	if(vl6180_data_g->i2c_client.cci_client)
		kfree(vl6180_data_g->i2c_client.cci_client);
	if(vl6180_data_g){
		kfree(vl6180_data_g);
		vl6180_data_g = NULL;
	}

	platform_driver_unregister(&stmvl6180_platform_driver);
	i2c_del_driver(&stmvl6180_i2c_driver);
}

module_init(stmvl6180_init);
module_exit(stmvl6180_exit);
MODULE_AUTHOR("STMicroelectronics Imaging Division");
MODULE_DESCRIPTION("ST FlightSense Time-of-Flight sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

