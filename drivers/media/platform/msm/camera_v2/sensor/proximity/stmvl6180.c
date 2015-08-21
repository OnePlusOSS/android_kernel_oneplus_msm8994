/*
 *  stmvl6180.c - Linux kernel module for STM VL6180 FlightSense Time-of-Flight
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
#include "stmvl6180.h"

#include <linux/kernel.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <linux/fb.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include "msm_cci.h"

//#define DEBUG_I2C_LOG
//debug


#define DEFAULT_CROSSTALK	 0//4 // Already coded in 9.7 format

//distance filter
#define DISTANCE_FILTER

struct stmvl6180_data *vl6180_data_g;
struct mutex	  vl6180_mutex;

#ifdef DISTANCE_FILTER
void VL6180_InitDistanceFilter(struct stmvl6180_data *vl6180_data);
uint16_t VL6180_DistanceFilter(struct stmvl6180_data *vl6180_data, 
	uint16_t m_trueRange_mm, uint16_t m_rawRange_mm, 
	uint32_t m_rtnSignalRate, uint32_t m_rtnAmbientRate, uint16_t errorCode);
uint32_t VL6180_StdDevDamper(uint32_t AmbientRate, uint32_t SignalRate, 
	uint32_t StdDevLimitLowLight, uint32_t StdDevLimitLowLightSNR, 
	uint32_t StdDevLimitHighLight, uint32_t StdDevLimitHighLightSNR);
#endif

/*
 * Communication functions
 */
#if 0
static int stmvl6180_write(struct stmvl6180_data *vl6180_data, unsigned int reg_addr, 
							unsigned int reg_bytes, unsigned int reg_data) 
{
	int err=0;	
	unsigned char write_buffer[6] = { 0, 0, 0, 0, 0,0 };

	unsigned int i=0;
	struct i2c_msg msg[1];

	struct i2c_client *client = vl6180_data->i2c_client.client;

#ifdef DEBUG_I2C_LOG
	vl6180_dbgmsg("WRITE REG: 0x%x , VAL: 0x%x \n", reg_addr, reg_data);
#endif
 	//set register address
	write_buffer[i++]=reg_addr >> 8;
	write_buffer[i++]=reg_addr;

	switch (reg_bytes) {
	case 4:
		write_buffer[i++] = (unsigned char)(reg_data >> 24);
		write_buffer[i++] = (unsigned char)(reg_data >> 16);
		write_buffer[i++] = (unsigned char)(reg_data >> 8);
		write_buffer[i++] = (unsigned char)(reg_data);
		break;
	case 2:
		write_buffer[i++] = (unsigned char)(reg_data >> 8);
		write_buffer[i++] = (unsigned char)(reg_data);		  
		break;
	case 1:
		write_buffer[i++] = (unsigned char)(reg_data);		  
		break;
	default:
		return -1;
	}

	msg[0].addr = client->addr;
	msg[0].flags = I2C_M_WR;
	msg[0].buf= &write_buffer[0];
	msg[0].len=i;
	
	err = i2c_transfer(client->adapter,msg,1); //return the actual messages transfer
	if(err != 1) {
		pr_err("%s: i2c_transfer err:%d, addr:0x%x, reg:0x%x\n", __func__, err, client->addr, reg_addr);
		return -1;
	}
	return 0;
}
#endif

static int stmvl6180_read(struct stmvl6180_data *vl6180_data, unsigned int reg_addr, 
					unsigned int reg_bytes, unsigned int *reg_data, unsigned int bitMask)
{
	int err=0;	
	unsigned char read_buffer[4] = { 0, 0, 0, 0 };



	err = vl6180_data->i2c_client.i2c_func_tbl->i2c_read_seq(
		&vl6180_data->i2c_client, reg_addr, read_buffer, reg_bytes);
	
	if(err < 0) {
		pr_err("%s: i2c_transfer err:%d, addr:0x%x, reg:0x%x\n", __func__, err, reg_addr, reg_addr);
		return -1;
	}
	switch (reg_bytes) {
	case 4:
		*reg_data = (unsigned int)( (unsigned int)(read_buffer[0] <<24)
					 | (unsigned int)((read_buffer[1])<<16)
					 | (unsigned int)((read_buffer[2])<<8)
					 | (unsigned int)(read_buffer[3]) );
		break;
	case 2:
		*reg_data = (unsigned int)(  (unsigned int)(read_buffer[0]<<8)
					 | (unsigned int)(read_buffer[1]) );	  
		break;
	case 1:
		*reg_data = (unsigned int)(read_buffer[0]);
		break;
	default:
		return -1;
	}
#ifdef DEBUG_I2C_LOG
	vl6180_dbgmsg("READ REG: 0x%x , VAL: 0x%x BITMASK:0x%x\n", reg_addr, *reg_data, bitMask);
#endif
	*reg_data &= bitMask;	  
	return 0;
}
// 8 bits cci read
int vl6180_i2c_read_8bits(struct stmvl6180_data *vl6180_data, unsigned int addr,  uint16_t *pdata)
{
	uint16_t tmp=0;
	int rc = 0;

	rc = vl6180_data->i2c_client.i2c_func_tbl->i2c_read(
		&vl6180_data->i2c_client, addr,
		&tmp, MSM_CAMERA_I2C_BYTE_DATA);

	*pdata = (uint16_t)tmp;
	return rc;
}
// 8 bits cci write
int vl6180_i2c_write_8bits(struct stmvl6180_data *vl6180_data, uint32_t addr,  uint16_t data)
{	
	int rc = 0;
	rc = vl6180_data->i2c_client.i2c_func_tbl->i2c_write(
		&vl6180_data->i2c_client, addr, data, MSM_CAMERA_I2C_BYTE_DATA);

	return rc;
}
// 16 bits cci write
int vl6180_i2c_write_16bits(struct stmvl6180_data *vl6180_data, uint32_t addr,  uint16_t data)
{
	int rc = 0;
	uint8_t write_buffer[2] = { 0, 0};
	write_buffer[1] = (uint8_t)(data & 0xFF);
	write_buffer[0] = (uint8_t)(data >> 8);

	rc = vl6180_data->i2c_client.i2c_func_tbl->i2c_write_seq(
		&vl6180_data->i2c_client, addr, write_buffer, MSM_CAMERA_I2C_WORD_DATA);

	return rc;
}

// 32 bits cci read
int vl6180_i2c_read_32bits(struct stmvl6180_data *vl6180_data, unsigned int addr,  unsigned int *pdata)
{	
	unsigned char read_buffer[4] = { 0, 0, 0, 0 };
	int rc = 0;
	
	rc = vl6180_data->i2c_client.i2c_func_tbl->i2c_read_seq(
		&vl6180_data->i2c_client, addr,read_buffer, 4);

	*pdata = (unsigned int)( (unsigned int)(read_buffer[0] <<24)
				 | (unsigned int)((read_buffer[1])<<16)
				 | (unsigned int)((read_buffer[2])<<8)
				 | (unsigned int)(read_buffer[3]) );

	return rc;
} 

int stmvl6180_power_enable(struct stmvl6180_data *vl6180_data, unsigned int enable)
{
	int rc = 0;

	printk("%s %d\n",__func__, enable);
	
	if(enable) {
		if(vl6180_data->enable == 0) {
			if (regulator_count_voltages(vl6180_data->vdd_regulator) > 0) {
				rc = regulator_set_voltage(vl6180_data->vdd_regulator, 2850000, 2850000);
				if (rc) {
					printk( "regulator set_vtg failed rc=%d\n", rc);
					return -1;
				}
			}
			rc = regulator_enable(vl6180_data->vdd_regulator);
			printk("stmvl6180 power on vdd regulator %d\n", rc);
	
			if (regulator_count_voltages(vl6180_data->vdd_regulator_i2c) > 0) {
				rc = regulator_set_voltage(vl6180_data->vdd_regulator_i2c, 1800000, 1800000);
				if (rc) {
					printk( "regulator set_vtg failed rc=%d\n", rc);
					regulator_disable(vl6180_data->vdd_regulator);
					return -1;
				}
			}
			rc = regulator_enable(vl6180_data->vdd_regulator_i2c);
			printk("stmvl6180 power on i2c regulator %d\n", rc);

			vl6180_data->enable = 1;
		}
		
		msleep(20);
		
		gpio_direction_output(vl6180_data->ce_gpio,1);
		
		if (vl6180_data->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
			rc = vl6180_data->i2c_client.i2c_func_tbl->i2c_util(
				&vl6180_data->i2c_client, MSM_CCI_INIT);
			if (rc < 0)
				pr_err("cci_init failed\n");
		}

	} else {

		if (vl6180_data->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
			rc = vl6180_data->i2c_client.i2c_func_tbl->i2c_util(
				&vl6180_data->i2c_client, MSM_CCI_RELEASE);
			if (rc < 0)
				pr_err("cci_init failed\n");
		}

		gpio_direction_output(vl6180_data->ce_gpio,0);

		if(vl6180_data->enable) {
			if (vl6180_data->vdd_regulator) {
				regulator_disable(vl6180_data->vdd_regulator);
			}

			if (vl6180_data->vdd_regulator_i2c) {
				regulator_disable(vl6180_data->vdd_regulator_i2c);
			}
			vl6180_data->enable = 0;
		}
	}
	return 0;
}

int vl6180_init(struct stmvl6180_data *vl6180_data)
{
	int rc = 0;
	int i;
	int8_t offsetByte;
	uint16_t modelID = 0;
	uint16_t revID = 0;
	int8_t rangeTemp = 0;
	uint16_t chipidRange = 0;
	uint16_t CrosstalkHeight;
	uint16_t IgnoreThreshold;
	uint16_t IgnoreThresholdHeight;
	uint16_t dataByte;
	uint16_t ambpart2partCalib1 = 0;
	uint16_t ambpart2partCalib2 = 0;
#ifdef USE_INTERRUPTS
	uint16_t chipidgpio = 0;
#endif
	pr_err("vl6180_init ENTER!\n");

	stmvl6180_power_enable(vl6180_data, 1);

	vl6180_i2c_read_8bits(vl6180_data, IDENTIFICATION__MODEL_ID, &modelID);
	vl6180_i2c_read_8bits(vl6180_data, IDENTIFICATION__REVISION_ID, &revID);
	pr_err("Model ID : 0x%X, REVISION ID : 0x%X\n", modelID, revID);

	//waitForStandby
	for(i=0; i<100; i++) {
		vl6180_i2c_read_8bits(vl6180_data, FIRMWARE__BOOTUP, &modelID);
		if( (modelID & 0x01) == 1 ) {
			i=100;
		}
	}

	//range device ready
	for(i=0; i<100; i++) {
		vl6180_i2c_read_8bits(vl6180_data, RESULT__RANGE_STATUS, &modelID);
		if( (modelID & 0x01) == 1) {
			i = 100;
		}
	}

	vl6180_i2c_write_8bits(vl6180_data, 0x0207, 0x01);
	vl6180_i2c_write_8bits(vl6180_data, 0x0208, 0x01);
	vl6180_i2c_write_8bits(vl6180_data, 0x0133, 0x01);
	vl6180_i2c_write_8bits(vl6180_data, 0x0096, 0x00);
	vl6180_i2c_write_8bits(vl6180_data, 0x0097, 0x54);
	vl6180_i2c_write_8bits(vl6180_data, 0x00e3, 0x00);
	vl6180_i2c_write_8bits(vl6180_data, 0x00e4, 0x04);
	vl6180_i2c_write_8bits(vl6180_data, 0x00e5, 0x02);
	vl6180_i2c_write_8bits(vl6180_data, 0x00e6, 0x01);
	vl6180_i2c_write_8bits(vl6180_data, 0x00e7, 0x03);
	vl6180_i2c_write_8bits(vl6180_data, 0x00f5, 0x02);
	vl6180_i2c_write_8bits(vl6180_data, 0x00D9, 0x05);

	// AMB P2P calibration
	vl6180_i2c_read_8bits(vl6180_data, SYSTEM__FRESH_OUT_OF_RESET, &dataByte);
	if(dataByte==0x01) {
		vl6180_i2c_read_8bits(vl6180_data, 0x26, &dataByte);
		ambpart2partCalib1 = dataByte<<8;
		vl6180_i2c_read_8bits(vl6180_data, 0x27, &dataByte);
		ambpart2partCalib1 = ambpart2partCalib1 + dataByte;
		vl6180_i2c_read_8bits(vl6180_data, 0x28, &dataByte);
		ambpart2partCalib2 = dataByte<<8;
		vl6180_i2c_read_8bits(vl6180_data, 0x29, &dataByte);
		ambpart2partCalib2 = ambpart2partCalib2 + dataByte;
		if(ambpart2partCalib1!=0) {
			// p2p calibrated
			vl6180_i2c_write_8bits(vl6180_data, 0xDA, (ambpart2partCalib1>>8)&0xFF);
			vl6180_i2c_write_8bits(vl6180_data, 0xDB, ambpart2partCalib1&0xFF);
			vl6180_i2c_write_8bits(vl6180_data, 0xDC, (ambpart2partCalib2>>8)&0xFF);
			vl6180_i2c_write_8bits(vl6180_data, 0xDD, ambpart2partCalib2&0xFF);
		} else {
			// No p2p Calibration, use default settings
			vl6180_i2c_write_8bits(vl6180_data, 0xDB, 0xCE);
			vl6180_i2c_write_8bits(vl6180_data, 0xDC, 0x03);
			vl6180_i2c_write_8bits(vl6180_data, 0xDD, 0xF8);
		}
	}
	
	vl6180_i2c_write_8bits(vl6180_data, 0x009f, 0x00);
	vl6180_i2c_write_8bits(vl6180_data, 0x00a3, 0x28);
	vl6180_i2c_write_8bits(vl6180_data, 0x00b7, 0x00);
	vl6180_i2c_write_8bits(vl6180_data, 0x00bb, 0x28);
	vl6180_i2c_write_8bits(vl6180_data, 0x00b2, 0x09);
	vl6180_i2c_write_8bits(vl6180_data, 0x00ca, 0x09);
	vl6180_i2c_write_8bits(vl6180_data, 0x0198, 0x01);
	vl6180_i2c_write_8bits(vl6180_data, 0x01b0, 0x17);
	vl6180_i2c_write_8bits(vl6180_data, 0x01ad, 0x00);
	vl6180_i2c_write_8bits(vl6180_data, 0x00FF, 0x05);
	vl6180_i2c_write_8bits(vl6180_data, 0x0100, 0x05);
	vl6180_i2c_write_8bits(vl6180_data, 0x0199, 0x05);
	vl6180_i2c_write_8bits(vl6180_data, 0x0109, 0x07);
	vl6180_i2c_write_8bits(vl6180_data, 0x010a, 0x30);
	vl6180_i2c_write_8bits(vl6180_data, 0x003f, 0x46);
	vl6180_i2c_write_8bits(vl6180_data, 0x01a6, 0x1b);
	vl6180_i2c_write_8bits(vl6180_data, 0x01ac, 0x3e);
	vl6180_i2c_write_8bits(vl6180_data, 0x01a7, 0x1f);
	vl6180_i2c_write_8bits(vl6180_data, 0x0103, 0x01);
	vl6180_i2c_write_8bits(vl6180_data, 0x0030, 0x00);
	vl6180_i2c_write_8bits(vl6180_data, 0x001b, 0x0A);
	vl6180_i2c_write_8bits(vl6180_data, 0x003e, 0x0A);
	vl6180_i2c_write_8bits(vl6180_data, 0x0131, 0x04);
	vl6180_i2c_write_8bits(vl6180_data, 0x0011, 0x10);
	vl6180_i2c_write_8bits(vl6180_data, 0x0014, 0x24);
	vl6180_i2c_write_8bits(vl6180_data, 0x0031, 0xFF);
	vl6180_i2c_write_8bits(vl6180_data, 0x00d2, 0x01);
	vl6180_i2c_write_8bits(vl6180_data, 0x00f2, 0x01);

	// RangeSetMaxConvergenceTime
	vl6180_i2c_write_8bits(vl6180_data, SYSRANGE__MAX_CONVERGENCE_TIME, 0x3F);
	vl6180_i2c_write_8bits(vl6180_data, SYSRANGE__MAX_AMBIENT_LEVEL_MULT, 0xFF);//SNR
	
	vl6180_i2c_read_8bits(vl6180_data, SYSTEM__FRESH_OUT_OF_RESET, &dataByte);
	if(dataByte==0x01) {
		//readRangeOffset
		vl6180_i2c_read_8bits(vl6180_data, SYSRANGE__PART_TO_PART_RANGE_OFFSET, &dataByte);
		rangeTemp = (int8_t)dataByte;
		if(dataByte > 0x7F) {
			rangeTemp -= 0xFF;
		}
		rangeTemp /= 3;
		rangeTemp = rangeTemp +1; //roundg
		//Range_Set_Offset
		offsetByte = *((u8*)(&rangeTemp)); // round
		vl6180_i2c_write_8bits(vl6180_data, SYSRANGE__PART_TO_PART_RANGE_OFFSET,(u8)offsetByte);
	}
	
	// ClearSystemFreshOutofReset
	vl6180_i2c_write_8bits(vl6180_data, SYSTEM__FRESH_OUT_OF_RESET, 0x0);
	
	// VL6180 CrossTalk
	vl6180_i2c_write_8bits(vl6180_data, SYSRANGE__CROSSTALK_COMPENSATION_RATE,
										(DEFAULT_CROSSTALK>>8)&0xFF);
	vl6180_i2c_write_8bits(vl6180_data, SYSRANGE__CROSSTALK_COMPENSATION_RATE+1,
										DEFAULT_CROSSTALK&0xFF);
	
	CrosstalkHeight = 40;
	vl6180_i2c_write_8bits(vl6180_data, SYSRANGE__CROSSTALK_VALID_HEIGHT,CrosstalkHeight&0xFF);
	
	
	// Will ignore all low distances (<100mm) with a low return rate
	IgnoreThreshold = 64; // 64 = 0.5Mcps
	IgnoreThresholdHeight = 33; // 33 * scaler3 = 99mm
	vl6180_i2c_write_8bits(vl6180_data, SYSRANGE__RANGE_IGNORE_THRESHOLD, (IgnoreThreshold>>8)&0xFF);
	vl6180_i2c_write_8bits(vl6180_data, SYSRANGE__RANGE_IGNORE_THRESHOLD+1,IgnoreThreshold&0xFF);
	vl6180_i2c_write_8bits(vl6180_data, SYSRANGE__RANGE_IGNORE_VALID_HEIGHT,IgnoreThresholdHeight&0xFF);
	
	vl6180_i2c_read_8bits(vl6180_data, SYSRANGE__RANGE_CHECK_ENABLES, &dataByte);
	dataByte = dataByte & 0xFE; // off ECE
	dataByte = dataByte | 0x02; // on ignore thr
	vl6180_i2c_write_8bits(vl6180_data, SYSRANGE__RANGE_CHECK_ENABLES, dataByte);
	
	// Init of Averaging samples
	for(i=0; i<8;i++) {
		vl6180_data->LastMeasurements[i]=65535; // 65535 means no valid data
	}
	vl6180_data->CurrentIndex = 0;

#ifdef USE_INTERRUPTS
	// SetSystemInterruptConfigGPIORanging
	vl6180_i2c_read_8bits(vl6180_data, SYSTEM__INTERRUPT_CONFIG_GPIO, &chipidgpio);
	vl6180_i2c_write_8bits(vl6180_data, SYSTEM__INTERRUPT_CONFIG_GPIO, (chipidgpio | 0x04));
#endif

	//RangeSetSystemMode
	chipidRange = 0x01;
	vl6180_i2c_write_8bits(vl6180_data, SYSRANGE__START, chipidRange);

#ifdef DISTANCE_FILTER
	VL6180_InitDistanceFilter(vl6180_data);
#endif

	return rc;
}

int vl6180_release(struct stmvl6180_data *vl6180_data) {

	stmvl6180_power_enable(vl6180_data, 0);

	return 0;
}


uint16_t vl6180_getDistance(struct stmvl6180_data *vl6180_data)
{
	uint16_t dist = 0;
	uint16_t chipidcount = 0;
	uint32_t m_rawRange_mm=0;
	uint32_t m_rtnConvTime=0;
	uint32_t m_rtnSignalRate=0;
	uint32_t m_rtnAmbientRate=0;
	uint32_t m_rtnSignalCount = 0;
	uint32_t m_refSignalCount = 0;
	uint32_t m_rtnAmbientCount =0;
	uint32_t m_refAmbientCount =0;
	uint32_t m_refConvTime =0;
	uint32_t m_refSignalRate =0;
	uint32_t m_refAmbientRate =0;
	uint32_t cRtnSignalCountMax = 0x7FFFFFFF;
	uint32_t  cDllPeriods = 6;
	uint32_t rtnSignalCountUInt = 0;
	uint32_t  calcConvTime = 0;
	uint16_t chipidRangeStart = 0;
	uint16_t statusCode = 0;
	uint16_t errorCode = 0;
	uint16_t m_rangeOffset;
	unsigned int m_crossTalk;

	
	vl6180_i2c_read_8bits(vl6180_data, SYSRANGE__START, &chipidRangeStart);
	//Read Error Code
	vl6180_i2c_read_8bits(vl6180_data, RESULT__RANGE_STATUS, &statusCode);
	errorCode = statusCode>>4;

	printk("status code 0x%x, chipidRangeStart %x\n",statusCode,chipidRangeStart);

	if(((statusCode&0x01)==0x01)&&(chipidRangeStart==0x00)){
				
		vl6180_i2c_read_8bits(vl6180_data, RESULT__RANGE_VAL, &dist);

		dist *= 3;

		vl6180_i2c_read_8bits(vl6180_data, RESULT__RANGE_RAW, &chipidcount);
		m_rawRange_mm = (uint32_t)chipidcount;

		vl6180_i2c_read_32bits(vl6180_data, RESULT__RANGE_RETURN_SIGNAL_COUNT, &rtnSignalCountUInt);

		if(rtnSignalCountUInt > cRtnSignalCountMax){
			rtnSignalCountUInt = 0;
		}

		m_rtnSignalCount  = rtnSignalCountUInt;

		vl6180_i2c_read_32bits(vl6180_data, RESULT__RANGE_REFERENCE_SIGNAL_COUNT, &m_refSignalCount);
		vl6180_i2c_read_32bits(vl6180_data, RESULT__RANGE_RETURN_AMB_COUNT, &m_rtnAmbientCount);
		vl6180_i2c_read_32bits(vl6180_data, RESULT__RANGE_REFERENCE_AMB_COUNT, &m_refAmbientCount);
		vl6180_i2c_read_32bits(vl6180_data, RESULT__RANGE_RETURN_CONV_TIME, &m_rtnConvTime);
		vl6180_i2c_read_32bits(vl6180_data, RESULT__RANGE_REFERENCE_CONV_TIME, &m_refConvTime);
		vl6180_i2c_read_8bits(vl6180_data, SYSRANGE__PART_TO_PART_RANGE_OFFSET, &m_rangeOffset);
		stmvl6180_read(vl6180_data, SYSRANGE__CROSSTALK_COMPENSATION_RATE, 2, &m_crossTalk, 0xFFFF);
		stmvl6180_read(vl6180_data, RESULT__RANGE_RETURN_RATE, 2, &m_rtnSignalRate, 0xFFFF);
		stmvl6180_read(vl6180_data, RESULT__RANGE_REFERENCE_RATE, 2, &m_refSignalRate, 0xFFFF);

		vl6180_i2c_write_8bits(vl6180_data, SYSTEM__INTERRUPT_CLEAR, 0x07);

		calcConvTime = m_refConvTime;
		if (m_rtnConvTime > m_refConvTime){
			calcConvTime = m_rtnConvTime;
		}
		if(calcConvTime==0)
			calcConvTime=63000;
//		m_rtnSignalRate  = (m_rtnSignalCount*1000)/calcConvTime;
//		m_refSignalRate  = (m_refSignalCount*1000)/calcConvTime;
		m_rtnAmbientRate = (m_rtnAmbientCount * cDllPeriods*1000)/calcConvTime;
		m_refAmbientRate = (m_rtnAmbientCount * cDllPeriods*1000)/calcConvTime;
		//printk("m_rtnSignalRate is %d, m_rtnAmbientRate is %d\n",m_rtnSignalRate,m_rtnAmbientRate);
#ifdef DISTANCE_FILTER
		dist = VL6180_DistanceFilter(vl6180_data, dist, m_rawRange_mm*3, 
						m_rtnSignalRate, m_rtnAmbientRate, errorCode);
#endif

		// Start new measurement
		//vl6180_i2c_write_8bits( SYSRANGE__START, 0x03);
		vl6180_i2c_write_8bits(vl6180_data, SYSRANGE__START, 0x01);
		vl6180_data->m_chipid = dist;
		vl6180_data->rangeData.m_range = dist;
		vl6180_data->rangeData.m_rtnRate = m_rtnSignalRate;
		vl6180_data->rangeData.m_refRate = m_refSignalRate;
		vl6180_data->rangeData.m_rtnAmbRate = m_rtnAmbientRate;
		vl6180_data->rangeData.m_refAmbRate = m_refAmbientRate;
		vl6180_data->rangeData.m_rawRange_mm = m_rawRange_mm*3;
		vl6180_data->rangeData.m_convTime = calcConvTime;

		vl6180_data->rangeData.m_rtnSignalCount = m_rtnSignalCount;
		vl6180_data->rangeData.m_refSignalCount = m_refSignalCount;
		vl6180_data->rangeData.m_rtnAmbientCount = m_rtnAmbientCount;
		vl6180_data->rangeData.m_refAmbientCount = m_refAmbientCount;
		vl6180_data->rangeData.m_errorCode = statusCode;
		vl6180_data->rangeData.m_rtnConvTime = m_rtnConvTime;
		vl6180_data->rangeData.m_refConvTime = m_refConvTime;
		vl6180_data->rangeData.m_rangeOffset = m_rangeOffset;
		vl6180_data->rangeData.m_crossTalk = m_crossTalk;

		printk("dist %d, rate %d, ambi rate%d\n",dist, m_rtnSignalRate, m_rtnAmbientRate);

	}
	else{
		// Return immediately with previous value
		dist = vl6180_data->m_chipid;
	}
	return dist;

}

#ifdef DISTANCE_FILTER
void VL6180_InitDistanceFilter(struct stmvl6180_data *vl6180_data)
{
	int i;

	vl6180_data->MeasurementIndex = 0;

	vl6180_data->Default_ZeroVal = 0;
	vl6180_data->Default_VAVGVal = 0;
	vl6180_data->NoDelay_ZeroVal = 0;
	vl6180_data->NoDelay_VAVGVal = 0;
	vl6180_data->Previous_VAVGDiff = 0;

	vl6180_data->StdFilteredReads = 0;
	vl6180_data->PreviousRangeStdDev = 0;
	vl6180_data->PreviousReturnRateStdDev = 0;

	for (i = 0; i < FILTERNBOFSAMPLES; i++){
		vl6180_data->LastTrueRange[i] = FILTERINVALIDDISTANCE;
		vl6180_data->LastReturnRates[i] = 0;
	}
}

uint32_t VL6180_StdDevDamper(uint32_t AmbientRate, uint32_t SignalRate, 
			uint32_t StdDevLimitLowLight, uint32_t StdDevLimitLowLightSNR, 
			uint32_t StdDevLimitHighLight, uint32_t StdDevLimitHighLightSNR)
{
	uint32_t newStdDev;
	uint16_t SNR;

	if (AmbientRate > 0)
		SNR = (uint16_t)((100 * SignalRate) / AmbientRate);
	else
		SNR = 9999;

	if (SNR >= StdDevLimitLowLightSNR){
		newStdDev = StdDevLimitLowLight;
	}
	else{
		if (SNR <= StdDevLimitHighLightSNR)
			newStdDev = StdDevLimitHighLight;
		else{
			newStdDev = (uint32_t)(StdDevLimitHighLight + 
					(SNR - StdDevLimitHighLightSNR) * (int)(StdDevLimitLowLight - StdDevLimitHighLight) /
					(StdDevLimitLowLightSNR - StdDevLimitHighLightSNR));
		}
	}

	return newStdDev;
}
uint16_t VL6180_DistanceFilter(struct stmvl6180_data *vl6180_data, uint16_t m_trueRange_mm,
	uint16_t m_rawRange_mm, uint32_t m_rtnSignalRate, uint32_t m_rtnAmbientRate, uint16_t errorCode)
{
	uint16_t m_newTrueRange_mm = 0;

	uint16_t i;
	uint16_t bypassFilter = 0;

	uint16_t registerValue;
   // uint16_t dataByte;
	uint32_t register32BitsValue1;
	uint32_t register32BitsValue2;

	uint16_t ValidDistance = 0;
	uint16_t MaxOrInvalidDistance = 0;

	uint16_t WrapAroundFlag = 0;
	uint16_t NoWrapAroundFlag = 0;
	uint16_t NoWrapAroundHighConfidenceFlag = 0;

	uint16_t FlushFilter = 0;
	uint32_t RateChange = 0;

	uint16_t StdDevSamples = 0;
	uint32_t StdDevDistanceSum = 0;
	uint32_t StdDevDistanceMean = 0;
	uint32_t StdDevDistance = 0;
	uint32_t StdDevRateSum = 0;
	uint32_t StdDevRateMean = 0;
	uint32_t StdDevRate = 0;
	uint32_t StdDevLimitWithTargetMove = 0;

	uint32_t VAVGDiff;
	uint32_t IdealVAVGDiff;
	uint32_t MinVAVGDiff;
	uint32_t MaxVAVGDiff;

	// Filter Parameters
	uint16_t WrapAroundLowRawRangeLimit = 20;
	uint32_t WrapAroundLowReturnRateLimit = 800;
	uint16_t WrapAroundLowRawRangeLimit2 = 55;
	uint32_t WrapAroundLowReturnRateLimit2 = 300;

	uint32_t WrapAroundLowReturnRateFilterLimit = 600;
	uint16_t WrapAroundHighRawRangeFilterLimit = 350;
	uint32_t WrapAroundHighReturnRateFilterLimit = 900;

	uint32_t WrapAroundMaximumAmbientRateFilterLimit = 7500;

	// Temporal filter data and flush values
	uint32_t MinReturnRateFilterFlush = 75;
	uint32_t MaxReturnRateChangeFilterFlush = 50;

	// STDDEV values and damper values
	uint32_t StdDevLimit = 300;
	uint32_t StdDevLimitLowLight = 300;
	uint32_t StdDevLimitLowLightSNR = 30; // 0.3
	uint32_t StdDevLimitHighLight = 2500;
	uint32_t StdDevLimitHighLightSNR = 5; //0.05

	uint32_t StdDevHighConfidenceSNRLimit = 8;

	uint32_t StdDevMovingTargetStdDevLimit = 90000;
	uint32_t StdDevMovingTargetReturnRateLimit = 3500;
	uint32_t StdDevMovingTargetStdDevForReturnRateLimit = 5000;

	uint32_t MAX_VAVGDiff = 1800;

	// WrapAroundDetection variables
	uint16_t WrapAroundNoDelayCheckPeriod = 2;

	// Reads Filtering values
	uint16_t StdFilteredReadsIncrement = 2;
	uint16_t StdMaxFilteredReads = 4;

	// End Filter Parameters

	MaxOrInvalidDistance = (uint16_t)(255 * 3);

	// Check if distance is Valid or not
	switch (errorCode){
	case 0x0C:
		m_trueRange_mm = MaxOrInvalidDistance;
		ValidDistance = 0;
		break;
	case 0x0D:
		m_trueRange_mm = MaxOrInvalidDistance;
		ValidDistance = 1;
		break;
	default:
		if (m_rawRange_mm >= MaxOrInvalidDistance){
			ValidDistance = 0;
		}
		else{
			ValidDistance = 1;
		}
		break;
	}
	m_newTrueRange_mm = m_trueRange_mm;

	// Checks on low range data
	if ((m_rawRange_mm < WrapAroundLowRawRangeLimit) && 
		(m_rtnSignalRate < WrapAroundLowReturnRateLimit)){
		//Not Valid distance
		m_newTrueRange_mm = MaxOrInvalidDistance;
		bypassFilter = 1;
	}
	if ((m_rawRange_mm < WrapAroundLowRawRangeLimit2) && 
		(m_rtnSignalRate < WrapAroundLowReturnRateLimit2)){
		//Not Valid distance
		m_newTrueRange_mm = MaxOrInvalidDistance;
		bypassFilter = 1;
	}

	// Checks on Ambient rate level
	if (m_rtnAmbientRate > WrapAroundMaximumAmbientRateFilterLimit){
		// Too high ambient rate
		FlushFilter = 1;
		bypassFilter = 1;
	}
	// Checks on Filter flush
	if (m_rtnSignalRate < MinReturnRateFilterFlush){
		// Completely lost target, so flush the filter
		FlushFilter = 1;
		bypassFilter = 1;
	}
	if (vl6180_data->LastReturnRates[0] != 0){
		if (m_rtnSignalRate > vl6180_data->LastReturnRates[0])
			RateChange = (100 * (m_rtnSignalRate - vl6180_data->LastReturnRates[0])) / 
						vl6180_data->LastReturnRates[0];
		else
			RateChange = (100 * (vl6180_data->LastReturnRates[0] - m_rtnSignalRate)) / 
						vl6180_data->LastReturnRates[0];
	}
	else
		RateChange = 0;
	if (RateChange > MaxReturnRateChangeFilterFlush){
		FlushFilter = 1;
	}

	if (FlushFilter == 1){
		vl6180_data->MeasurementIndex = 0;
		for (i = 0; i < FILTERNBOFSAMPLES; i++){
			vl6180_data->LastTrueRange[i] = FILTERINVALIDDISTANCE;
			vl6180_data->LastReturnRates[i] = 0;
		}
	}
	else{
		for (i = (uint16_t)(FILTERNBOFSAMPLES - 1); i > 0; i--){
			vl6180_data->LastTrueRange[i] = vl6180_data->LastTrueRange[i - 1];
			vl6180_data->LastReturnRates[i] = vl6180_data->LastReturnRates[i - 1];
		}
	}
	if (ValidDistance == 1)
		vl6180_data->LastTrueRange[0] = m_trueRange_mm;
	else
		vl6180_data->LastTrueRange[0] = FILTERINVALIDDISTANCE;
	vl6180_data->LastReturnRates[0] = m_rtnSignalRate;

	// Check if we need to go through the filter or not
	if (!(((m_rawRange_mm < WrapAroundHighRawRangeFilterLimit) && 
		(m_rtnSignalRate < WrapAroundLowReturnRateFilterLimit)) ||
		((m_rawRange_mm >= WrapAroundHighRawRangeFilterLimit) && 
		(m_rtnSignalRate < WrapAroundHighReturnRateFilterLimit))))
		bypassFilter = 1;

	// Check which kind of measurement has been made
	vl6180_i2c_read_8bits(vl6180_data, 0x01AC, &registerValue);

	// Read data for filtering
	vl6180_i2c_read_32bits(vl6180_data, 0x010C, &register32BitsValue1);
	vl6180_i2c_read_32bits(vl6180_data, 0x0110, &register32BitsValue2);
	if (registerValue == 0x3E){
		vl6180_data->Default_ZeroVal = register32BitsValue1;
		vl6180_data->Default_VAVGVal = register32BitsValue2;
	}
	else{
		vl6180_data->NoDelay_ZeroVal = register32BitsValue1;
		vl6180_data->NoDelay_VAVGVal = register32BitsValue2;
	}

	if (bypassFilter == 1) {
		// Do not go through the filter
		if (registerValue != 0x3E)
		{
			vl6180_i2c_write_8bits(vl6180_data, 0x01AC, 0x3E);
		}
		// Set both Defaut and NoDelay To same value
		vl6180_data->Default_ZeroVal = register32BitsValue1;
		vl6180_data->Default_VAVGVal = register32BitsValue2;
		vl6180_data->NoDelay_ZeroVal = register32BitsValue1;
		vl6180_data->NoDelay_VAVGVal = register32BitsValue2;
		vl6180_data->MeasurementIndex = 0;

		// Return immediately
		return m_newTrueRange_mm;
	}

	if (vl6180_data->MeasurementIndex % WrapAroundNoDelayCheckPeriod == 0){
		vl6180_i2c_write_8bits(vl6180_data, 0x01AC, 0x3F);
	}
	else{
		vl6180_i2c_write_8bits(vl6180_data, 0x01AC, 0x3E);
	}

	vl6180_data->MeasurementIndex = (uint16_t)(vl6180_data->MeasurementIndex + 1);

	// Computes current VAVGDiff
	if (vl6180_data->Default_VAVGVal > vl6180_data->NoDelay_VAVGVal)
		VAVGDiff = vl6180_data->Default_VAVGVal - vl6180_data->NoDelay_VAVGVal;
	else
		VAVGDiff = 0;
	vl6180_data->Previous_VAVGDiff = VAVGDiff;

	// Check the VAVGDiff
	if(vl6180_data->Default_ZeroVal > vl6180_data->NoDelay_ZeroVal)
		IdealVAVGDiff = vl6180_data->Default_ZeroVal - vl6180_data->NoDelay_ZeroVal;
	else
		IdealVAVGDiff = vl6180_data->NoDelay_ZeroVal - vl6180_data->Default_ZeroVal;
	if (IdealVAVGDiff > MAX_VAVGDiff)
		MinVAVGDiff = IdealVAVGDiff - MAX_VAVGDiff;
	else
		MinVAVGDiff = 0;
	MaxVAVGDiff = IdealVAVGDiff + MAX_VAVGDiff;
	if (VAVGDiff < MinVAVGDiff || VAVGDiff > MaxVAVGDiff){
		WrapAroundFlag = 1;
	}
	else{
		// Go through filtering check

		// StdDevLimit Damper on SNR
		StdDevLimit = VL6180_StdDevDamper(m_rtnAmbientRate, m_rtnSignalRate, 
								StdDevLimitLowLight, StdDevLimitLowLightSNR,
								StdDevLimitHighLight, StdDevLimitHighLightSNR);

		// Standard deviations computations
		StdDevSamples = 0;
		StdDevDistanceSum = 0;
		StdDevDistanceMean = 0;
		StdDevDistance = 0;
		StdDevRateSum = 0;
		StdDevRateMean = 0;
		StdDevRate = 0;
		for (i = 0; (i < FILTERNBOFSAMPLES) && (StdDevSamples < FILTERSTDDEVSAMPLES); i++){
			if (vl6180_data->LastTrueRange[i] != FILTERINVALIDDISTANCE){
				StdDevSamples = (uint16_t)(StdDevSamples + 1);
				StdDevDistanceSum = (uint32_t)(StdDevDistanceSum + vl6180_data->LastTrueRange[i]);
				StdDevRateSum = (uint32_t)(StdDevRateSum + vl6180_data->LastReturnRates[i]);
			}
		}
		if (StdDevSamples > 0){
			StdDevDistanceMean = (uint32_t)(StdDevDistanceSum / StdDevSamples);
			StdDevRateMean = (uint32_t)(StdDevRateSum / StdDevSamples);
		}
		StdDevSamples = 0;
		StdDevDistanceSum = 0;
		StdDevRateSum = 0;
		for (i = 0; (i < FILTERNBOFSAMPLES) && (StdDevSamples < FILTERSTDDEVSAMPLES); i++){
			if (vl6180_data->LastTrueRange[i] != FILTERINVALIDDISTANCE){
				StdDevSamples = (uint16_t)(StdDevSamples + 1);
				StdDevDistanceSum = (uint32_t)(StdDevDistanceSum + 
					(int)(vl6180_data->LastTrueRange[i] - StdDevDistanceMean) * 
					(int)(vl6180_data->LastTrueRange[i] - StdDevDistanceMean));
				StdDevRateSum = (uint32_t)(StdDevRateSum + (int)(vl6180_data->LastReturnRates[i] - 
					StdDevRateMean) * (int)(vl6180_data->LastReturnRates[i] - StdDevRateMean));
			}
		}
		if (StdDevSamples >= MINFILTERSTDDEVSAMPLES){
			StdDevDistance = (uint16_t)(StdDevDistanceSum / StdDevSamples);
			StdDevRate = (uint16_t)(StdDevRateSum / StdDevSamples);
		}
		else{
			StdDevDistance = 0;
			StdDevRate = 0;
		}

		// Check Return rate standard deviation
		if (StdDevRate < StdDevMovingTargetStdDevLimit){
			if (StdDevSamples < MINFILTERVALIDSTDDEVSAMPLES){
				m_newTrueRange_mm = MaxOrInvalidDistance;
			}
			else{
				// Check distance standard deviation
				if (StdDevRate < StdDevMovingTargetReturnRateLimit)
					StdDevLimitWithTargetMove = StdDevLimit + 
						(((StdDevMovingTargetStdDevForReturnRateLimit - StdDevLimit) * StdDevRate) / 
						StdDevMovingTargetReturnRateLimit);
				else
					StdDevLimitWithTargetMove = StdDevMovingTargetStdDevForReturnRateLimit;

				if ((StdDevDistance * StdDevHighConfidenceSNRLimit) < StdDevLimitWithTargetMove){
					NoWrapAroundHighConfidenceFlag = 1;
				}
				else{
					if (StdDevDistance < StdDevLimitWithTargetMove){
						if (StdDevSamples >= MINFILTERVALIDSTDDEVSAMPLES){
							NoWrapAroundFlag = 1;
						}
						else{
							m_newTrueRange_mm = MaxOrInvalidDistance;
						}
					}
					else{
						WrapAroundFlag = 1;
					}
				}
			}
		}
		else{
			WrapAroundFlag = 1;
		}
	}

	if (m_newTrueRange_mm == MaxOrInvalidDistance){
		if (vl6180_data->StdFilteredReads > 0)
			vl6180_data->StdFilteredReads = (uint16_t)(vl6180_data->StdFilteredReads - 1);
	}
	else{
		if (WrapAroundFlag == 1){
			m_newTrueRange_mm = MaxOrInvalidDistance;
			vl6180_data->StdFilteredReads = (uint16_t)(vl6180_data->StdFilteredReads + 
												StdFilteredReadsIncrement);
			if (vl6180_data->StdFilteredReads > StdMaxFilteredReads)
				vl6180_data->StdFilteredReads = StdMaxFilteredReads;
		}
		else{
			if (NoWrapAroundFlag == 1){
				if (vl6180_data->StdFilteredReads > 0){
					m_newTrueRange_mm = MaxOrInvalidDistance;
					if (vl6180_data->StdFilteredReads > StdFilteredReadsIncrement)
						vl6180_data->StdFilteredReads = (uint16_t)(vl6180_data->StdFilteredReads - 
															StdFilteredReadsIncrement);
					else
						vl6180_data->StdFilteredReads = 0;
				}
			}
			else{
				if (NoWrapAroundHighConfidenceFlag == 1){
					vl6180_data->StdFilteredReads = 0;
				}
			}
		}
	}
	vl6180_data->PreviousRangeStdDev = StdDevDistance;
	vl6180_data->PreviousReturnRateStdDev = StdDevRate;
	vl6180_data->PreviousStdDevLimit = StdDevLimitWithTargetMove;

	return m_newTrueRange_mm;
}

#endif


static int stmvl6180_parse_dt(struct device *dev, struct stmvl6180_data *vl6180_data)
{
	int ret = 0;

	if (dev->of_node) {
		struct device_node *np = dev->of_node;

		/* reset, irq gpio info */
		vl6180_data->irq_gpio    = of_get_named_gpio(np, "st,irq-gpio", 0);
		if( vl6180_data->irq_gpio < 0 ) {
			printk(KERN_ERR"vl6180 irq_gpio not specified\n");
			return -1;
		}
		vl6180_data->ce_gpio  = of_get_named_gpio(np, "st,standby-gpio", 0);
		if( vl6180_data->ce_gpio < 0 ) {
			printk(KERN_ERR"vl6180 ce gpio not specified\n");	
			return -1;
		}
		printk("%s irq_gpio:%d ce_gpio:%d\n", __func__, vl6180_data->irq_gpio, vl6180_data->ce_gpio);

		ret = gpio_request(vl6180_data->ce_gpio, "vl_6180-ce");
		if (ret < 0) {
			printk(KERN_ERR"%s: gpio request failed for vl6180\n", __func__);
			return ret;
		}

		ret = gpio_request(vl6180_data->irq_gpio,"vl_6180-int"); 
		if(ret < 0) {
			printk(KERN_ERR "%s: gpio_request, err=%d", __func__, ret);
			gpio_free(vl6180_data->ce_gpio);
			return ret;
		}

		vl6180_data->vdd_regulator = regulator_get(dev, "vdd_1v8");
		if(IS_ERR(vl6180_data->vdd_regulator)) {
			printk(KERN_ERR"%s Regulator get failed vdd rc=%d\n", __func__, ret);
			gpio_free(vl6180_data->ce_gpio);
			gpio_free(vl6180_data->irq_gpio);
			return -1;
		}

		vl6180_data->vdd_regulator_i2c = regulator_get(dev, "vcc_i2c_1v8");
		if(IS_ERR(vl6180_data->vdd_regulator_i2c) ) {
			printk(KERN_ERR"%s Regulator get failed vdd_i2c rc=%d\n", __func__, ret);
			gpio_free(vl6180_data->ce_gpio);
			gpio_free(vl6180_data->irq_gpio);
			regulator_put(vl6180_data->vdd_regulator);
			return -1;
		}

	}
	return ret;
}

/*
 * misc device file operation functions
 */
static int stmvl6180_ioctl_handler(struct file *file, 
				unsigned int cmd, unsigned long arg, void __user *p)
{

	int rc=0;
	struct stmvl6180_data *vl6180_data = file->private_data;
    void __user *argp = (void __user *)p;
	unsigned long data=0;
	RegisterInfo register_data;
	switch (cmd) {
	case VL6180_IOCTL_INIT:	   /* init.  */
	{
        unsigned long *distance = (unsigned long *)argp;
		data = vl6180_getDistance(vl6180_data);
        *distance = data;
        printk("vl6180_getDistance init return %ld\n",*distance);
		return 0;
	}
	case VL6180_IOCTL_GETDATA:	  /* Get proximity value only */
	{
		data = vl6180_getDistance(vl6180_data);
		printk("vl6180_getDistance return %ld\n",data);
		//return put_user(data, (unsigned long *)p);
		return 0;
	}
	case VL6180_IOCTL_GETDATAS:	 /* Get all range data */
	{
		data = vl6180_getDistance(vl6180_data);

		if (copy_to_user((RangeData *)p, &(vl6180_data->rangeData), sizeof(RangeData))) {
			printk("error copy to user\n");
			rc = -EFAULT;
		}	 
		return rc;   
	}
	case VL6180_IOCTL_CONFIG:        /* Get all range data */
       {
               if (copy_from_user(&register_data, argp, sizeof(RegisterInfo))) {
                       printk("error copy from user\n");
                       rc = -EFAULT;
               }
               printk("%s:%d wirte register 0x%x:0x%x\n", __func__, __LINE__, register_data.addr, register_data.data);

		if(register_data.size == 1)
			rc = vl6180_i2c_write_8bits(vl6180_data, register_data.addr, register_data.data);
		else if(register_data.size == 2)
			rc = vl6180_i2c_write_16bits(vl6180_data, register_data.addr, register_data.data);

               if(rc)
                       printk("%s:%d failed to wirte register 0x%x:0x%x\n", __func__, __LINE__, register_data.addr, register_data.data);
               return rc;
       }

	default:
		return -EINVAL;
	}
	return rc;
}

static int stmvl6180_open(struct inode *inode, struct file *file)
{
	file->private_data = vl6180_data_g;
	//vl6180_init(i2c_get_clientdata(file->private_data));
    if(vl6180_data_g!=NULL){
	    vl6180_init(vl6180_data_g);
    }
	return 0;
}

static int stmvl6180_release(struct inode *inode, struct file *file)
{
	//vl6180_release(i2c_get_clientdata(file->private_data));
    if(vl6180_data_g!=NULL){
	    vl6180_release(vl6180_data_g);
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

static const struct file_operations stmvl6180_ranging_fops = {
	.owner =			THIS_MODULE,
	.unlocked_ioctl =	stmvl6180_ioctl,
	.open =			stmvl6180_open,
	.release = 	stmvl6180_release,
};

static struct miscdevice stmvl6180_ranging_dev = {
	.minor =	MISC_DYNAMIC_MINOR,
	.name =		"stmvl6180_ranging",
	.fops =		&stmvl6180_ranging_fops
};

static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_seq = msm_camera_cci_i2c_write_seq,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_poll =  msm_camera_cci_i2c_poll,
};

static struct msm_camera_i2c_fn_t msm_sensor_qup_func_tbl = {
	.i2c_read = msm_camera_qup_i2c_read,
	.i2c_read_seq = msm_camera_qup_i2c_read_seq,
	.i2c_write = msm_camera_qup_i2c_write,
	.i2c_write_table = msm_camera_qup_i2c_write_table,
	.i2c_write_seq_table = msm_camera_qup_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_qup_i2c_write_table_w_microdelay,
	.i2c_poll = msm_camera_qup_i2c_poll,
};

/*
 * Initialization function
 */

static int stmvl6180_init_client(struct stmvl6180_data *vl6180_data)
{
	int err;
	int id=0,module_major=0,module_minor=0;
	int model_major=0,model_minor=0;
	int i=0,val;

	vl6180_data->is_6180 = 1;
	err = stmvl6180_read(vl6180_data,
						 VL6180_MODEL_ID_REG,
						 1,
						 &id,
						 0xFF); 
	if (id == 0xb4) {
		printk("STM VL6180 Found\n");
	}
	else if (id==0) {
		printk("Not found STM VL6180\n");
		return -EIO;
	}

	// Read Model Version
	err = stmvl6180_read(vl6180_data,
						 VL6180_MODEL_REV_MAJOR_REG,
						 1,
						 &model_major,
						 0x07); 
	err = stmvl6180_read(vl6180_data,
						 VL6180_MODEL_REV_MINOR_REG,
						 1,
						 &model_minor,
						 0x07); 
	printk("STM VL6180 Model Version : %d.%d\n", model_major,model_minor);

	// Read Module Version
	err = stmvl6180_read(vl6180_data,
						 VL6180_MODULE_REV_MAJOR_REG,
						 1,
						 &module_major,
						 0xFF);
	err = stmvl6180_read(vl6180_data,
						 VL6180_MODULE_REV_MINOR_REG,
						 1,
						 &module_minor,
						 0xFF); 
	printk("STM VL6180 Module Version : %d.%d\n",module_major,module_minor);
	
	// Read Identification 
	printk("STM VL6180 Serial Number: ");
	for (i=0; i<=(VL6180_FIRMWARE_REVISION_ID_REG-VL6180_REVISION_ID_REG);i++) {
		err = stmvl6180_read(vl6180_data,
						 		(VL6180_REVISION_ID_REG+i),
						 		 1,
						 		 &val,
						 		 0xFF);
		printk("0x%x-",val);
	}
	printk("\n");

	//set up i2c client
	vl6180_data_g = vl6180_data;
	return 0;
}


/*
 * I2C init/probing/exit functions
 */

static int32_t stmvl6180_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	struct stmvl6180_data *vl6180_data = NULL;
	printk("%s Enter\n",__func__);

	if (client == NULL) {
		pr_err("msm_actuator_i2c_probe: client is null\n");
		return -EINVAL;
	}

	vl6180_data = kzalloc(sizeof(struct stmvl6180_data),
		GFP_KERNEL);
	if (!vl6180_data) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	rc = of_property_read_u32(client->dev.of_node, "cell-index",
				&vl6180_data->subdev_id);
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		goto probe_failure;
	}

	printk("%s cell-index %d, rc %d\n",__func__, vl6180_data->subdev_id, rc);

	rc = stmvl6180_parse_dt(&client->dev, vl6180_data);
	if (rc < 0) {
		kfree(vl6180_data);
		pr_err("failed rc %d\n", rc);
		return rc;
	}

	vl6180_data->i2c_client.client = client;
	/* Set device type as I2C */
	vl6180_data->act_device_type = MSM_CAMERA_I2C_DEVICE;
	vl6180_data->i2c_client.i2c_func_tbl = &msm_sensor_qup_func_tbl;

	vl6180_data->i2c_client.client->addr = 0x29;
	vl6180_data->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;

	mutex_init(&vl6180_mutex);
	vl6180_data->enable = 0;		/* default mode is standard */

	rc = stmvl6180_power_enable(vl6180_data, 1);
	if(rc) {
		printk(KERN_ERR "%s vl6180 power on error\n", __func__);
		goto probe_failure;
	}

	/* Initialize the STM VL6180 chip */
	rc = stmvl6180_init_client(vl6180_data);
	if (rc) {
		stmvl6180_power_enable(vl6180_data, 0);
		goto probe_failure;
	}

	stmvl6180_power_enable(vl6180_data, 0);

	//to register as a misc device
	if (misc_register(&stmvl6180_ranging_dev) != 0)
		printk(KERN_INFO "Could not register misc. dev for stmvl6180 ranging\n");

	pr_info("stmvl6180_i2c_probe: succeeded\n");
	printk("%s Exit\n",__func__);

	return 0;

probe_failure:
	kfree(vl6180_data);
	return rc;
}

static int32_t stmvl6180_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	struct msm_camera_cci_client *cci_client = NULL;
	struct stmvl6180_data *vl6180_data = NULL;
	printk("%s Enter\n",__func__);

	if (!pdev->dev.of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	vl6180_data = kzalloc(sizeof(struct stmvl6180_data),
		GFP_KERNEL);
	if (!vl6180_data) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}
	rc = of_property_read_u32((&pdev->dev)->of_node, "cell-index",
		&pdev->id);
	if (rc < 0) {
		kfree(vl6180_data);
		pr_err("failed rc %d\n", rc);
		return rc;
	}
	printk("%s cell-index %d, rc %d\n",__func__, pdev->id, rc);

	rc = of_property_read_u32((&pdev->dev)->of_node, "qcom,cci-master",
		&vl6180_data->cci_master);
	if (rc < 0) {
		kfree(vl6180_data);
		pr_err("failed rc %d\n", rc);
		return rc;
	}
	printk("%s qcom,cci-master %d, rc %d\n",__func__, vl6180_data->cci_master, rc);

	/* Set platform device handle */
	vl6180_data->pdev = pdev;

	rc = stmvl6180_parse_dt(&pdev->dev, vl6180_data);
	if (rc < 0) {
		kfree(vl6180_data);
		pr_err("failed rc %d\n", rc);
		return rc;
	}

	vl6180_data->subdev_id = pdev->id;

	/* Set device type as platform device */
	vl6180_data->act_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	vl6180_data->i2c_client.i2c_func_tbl = &msm_sensor_cci_func_tbl;
	vl6180_data->i2c_client.cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!vl6180_data->i2c_client.cci_client) {
		kfree(vl6180_data);
		pr_err("failed no memory\n");
		return -ENOMEM;
	}

	cci_client = vl6180_data->i2c_client.cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->sid = 0x29;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	cci_client->cci_i2c_master = vl6180_data->cci_master;
	vl6180_data->i2c_client.addr_type = MSM_CAMERA_I2C_WORD_ADDR;

	mutex_init(&vl6180_mutex);
	vl6180_data->enable = 0;		/* default mode is standard */

	rc = stmvl6180_power_enable(vl6180_data, 1);
	if(rc) {
		kfree(vl6180_data->i2c_client.cci_client);
		kfree(vl6180_data);
		pr_err("failed rc %d\n", rc);
		return rc;
	}

	/* Initialize the STM VL6180 chip */
	rc = stmvl6180_init_client(vl6180_data);
	if (rc) {
		stmvl6180_power_enable(vl6180_data, 0);
		kfree(vl6180_data->i2c_client.cci_client);
		kfree(vl6180_data);
		pr_err("failed rc %d\n", rc);
		return rc;
	}

	stmvl6180_power_enable(vl6180_data, 0);

	//to register as a misc device
	if (misc_register(&stmvl6180_ranging_dev) != 0)
		printk(KERN_INFO "Could not register misc. dev for stmvl6180 ranging\n");

	printk("%s Exit\n",__func__);
	return rc;
}

static int  stmvl6180_remove(struct i2c_client *client)
{
	struct stmvl6180_data *vl6180_data = i2c_get_clientdata(client);
	
	/* Power down the device */

	stmvl6180_power_enable(vl6180_data, 0);

	regulator_put(vl6180_data->vdd_regulator);
	regulator_put(vl6180_data->vdd_regulator_i2c);

	gpio_free(vl6180_data->ce_gpio);
	gpio_free(vl6180_data->irq_gpio);
	
	misc_deregister(&stmvl6180_ranging_dev);

	kfree(vl6180_data);

	return 0;
}

static const struct i2c_device_id stmvl6180_id[] = {
	{ STMVL6180_DRV_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, stmvl6180_id);



static const struct of_device_id stmvl6180_dt_match[] = {
	{.compatible = "stmv,vl6180", .data=NULL},
	{}
};

static struct i2c_driver stmvl6180_i2c_driver = {
	.driver = {
		.name	= STMVL6180_DRV_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = stmvl6180_dt_match,
	},
	.probe	= stmvl6180_i2c_probe,
	.remove	= stmvl6180_remove,
	.id_table = stmvl6180_id,
};

MODULE_DEVICE_TABLE(of, stmvl6180_dt_match);

static struct platform_driver stmvl6180_platform_driver = {
	.driver = {
		.name = "stmv,vl6180",
		.owner = THIS_MODULE,
		.of_match_table = stmvl6180_dt_match,
	},
};

static int __init stmvl6180_init(void)
{
	int32_t rc = 0;
	printk("Enter %s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&stmvl6180_platform_driver,
		stmvl6180_platform_probe);
	if (!rc)
		return rc;

	printk("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&stmvl6180_i2c_driver);
}

static void __exit stmvl6180_exit(void)
{
	i2c_del_driver(&stmvl6180_i2c_driver);
}

MODULE_AUTHOR("STM");
MODULE_DESCRIPTION("ST FlightSense Time-of-Flight sensor driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(stmvl6180_init);
module_exit(stmvl6180_exit);

