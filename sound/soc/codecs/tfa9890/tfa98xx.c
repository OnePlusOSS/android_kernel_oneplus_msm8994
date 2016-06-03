/*
 * Copyright (C) NXP Semiconductors (PLMA)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

//#define DEBUG
//#define pr_fmt(fmt) "%s(%s): " fmt, __func__, tfa98xx->fw.name

#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <sound/tfa98xx.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/sounddebug.h>

#include "tfa98xx-core.h"
#include "tfa98xx-regs.h"
#include "tfa_container.h"
#include "tfa_dsp.h"

int testLogOn = 0;
EXPORT_SYMBOL_GPL(testLogOn);
#ifdef VENDOR_EDIT
/*suzhiguang@MultiMedia.AudioDrv, 2015-08-11, repair when MTP 0 fail*/
extern int recoverMtp0;
#endif

static int test =0;

#define I2C_RETRY_DELAY		5 /* ms */
#define I2C_RETRIES		5
#define PLL_SYNC_RETRIES	10
#define MTPB_RETRIES		5


/* SNDRV_PCM_RATE_KNOT -> 12000, 24000 Hz, limit with constraint list */
#define TFA98XX_RATES (SNDRV_PCM_RATE_8000_48000 | SNDRV_PCM_RATE_KNOT)
#define TFA98XX_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE)

#define TFA98XX_STATUS_UP_MASK	(TFA98XX_STATUSREG_PLLS | \
				 TFA98XX_STATUSREG_CLKS | \
				 TFA98XX_STATUSREG_VDDS | \
				 TFA98XX_STATUSREG_AREFS)



struct tfa98xx *g_tfa98xx = NULL;
EXPORT_SYMBOL_GPL(g_tfa98xx);


#ifdef VENDOR_EDIT
/* zhiguang.su@MultiMedia.AudioDrv on 2015-05-29, add for smart pa calibtation*/
static ssize_t tfa98xx_state_store(struct device *dev, struct device_attribute *attr, 
		const char *buf, size_t count)
{
    pr_err("%s",__func__);

	return 0;
}
static ssize_t tfa98xx_state_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
    struct snd_soc_codec *codec;
    struct tfa98xx *tfa98xx;
    unsigned short mtp;
	unsigned short status;
    int done = 0;
    u32 re25 = 0;
    int try = 0;
    u16 mtp0;

    if(g_tfa98xx == NULL)
    {
        pr_err("%s g_tfa98xx = NULL\n",__func__);
        return 0;
    }

    tfa98xx = g_tfa98xx;
    codec = tfa98xx->codec;

    mutex_lock(&tfa98xx->dsp_init_lock);

#ifdef VENDOR_EDIT
/*suzhiguang@MultiMedia.AudioDrv, 2015-08-11, Modify for MTP recovery*/
        /*
	     * check the contents of  MTP register for non-zero,
	     * this indicates that the subsys is ready
	     */
	    mtp0 = (u16)snd_soc_read(codec, 0x84);

        /* NXP: if 0x84 is wrong, restore the correct mtp settings */
        if (!mtp0 || recoverMtp0)
        {
		    pr_err("%s mtp0 error,now recovery mtp.\n",__func__);
            if(recoverMtp0)
            {
                pr_err("%s mtp0 error detect in cold start up.\n",__func__);
                recoverMtp0 = false;
            }
            tfa98xx_restore_mtp(tfa98xx);
        }
#endif

	if (!tfa98xx_is_pwdn(tfa98xx)) {
		tfa98xx_dsp_stop(tfa98xx);
	}
    tfaRunColdStartup(tfa98xx);
    msleep(5);

    mtp = snd_soc_read(codec, TFA98XX_MTP);
	/* reset MTPEX bit if needed */
	if ( (mtp & TFA98XX_KEY2_PROTECTED_SPKR_CAL_MTP_MTPOTC) && (mtp & TFA98XX_KEY2_PROTECTED_SPKR_CAL_MTP_MTPEX))
	{
	    snd_soc_write(codec, 0x0B, 0x5A); /* unlock key2 */
        snd_soc_write(codec, TFA98XX_MTP, 1); /* MTPOTC=1, MTPEX=0 */
        snd_soc_write(codec, 0x62, 1<<11); /* CIMTP=1 */
    }

	do {
        try ++;
		msleep(10);
        status = snd_soc_read(codec, TFA98XX_STATUSREG);
	} while (((status & TFA98XX_STATUSREG_MTPB) == TFA98XX_STATUSREG_MTPB) && (try < 100));

    if(try == 100)
    {
        pr_err("%s try read TFA98XX_STATUSREG_MTPB time out\n",__func__);
        goto err;
    }

	if (!tfa98xx_is_pwdn(tfa98xx)) {
		tfa98xx_dsp_stop(tfa98xx);
	}
    msleep(10);
    tfa98xx->dsp_init = TFA98XX_DSP_INIT_RECOVER;
	/* start the DSP using the latest profile / vstep */
	if (!tfa98xx_dsp_start(tfa98xx, tfa98xx->profile, tfa98xx->vstep))
		tfa98xx->dsp_init = TFA98XX_DSP_INIT_DONE;

#ifdef VENDOR_EDIT
/*suzhiguang@MultiMedia.AudioDrv, 2015-08-11, Modify for MTP recovery*/
        /*
	     * check the contents of  MTP register for non-zero,
	     * this indicates that the subsys is ready
	     */
	    mtp0 = (u16)snd_soc_read(codec, 0x84);

        /* NXP: if 0x84 is wrong, restore the correct mtp settings */
        if (!mtp0 )
        {
		    pr_err("%s mtp0 error,now recovery mtp.\n",__func__);
            tfa98xx_restore_mtp(tfa98xx);
        }
#endif

    mtp = snd_soc_read(codec, TFA98XX_MTP);
	done = (mtp & TFA98XX_MTP_MTPEX);
    tfa98xx_dsp_get_calibration_impedance(tfa98xx, &re25);

    pr_debug("%s done =%d re=%d\n",__func__,done,re25);
    mutex_unlock(&tfa98xx->dsp_init_lock);
    return sprintf(buf,"%d:%d",done,re25);

err:
    pr_err("%s calibrate fail\n",__func__);
    mutex_unlock(&tfa98xx->dsp_init_lock);
    return sprintf(buf,"%d:%d",0,0);

}



static struct device_attribute tfa98xx_state_attr =
     __ATTR(calibra, 0444, tfa98xx_state_show, tfa98xx_state_store);

/*zhiguang.su@MultiMedia.AudioDrv, 2015-11-05, add for debug*/
static ssize_t tfa98xx_Log_state_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
    pr_err("%s",__func__);

    if (sysfs_streq(buf, "LogOn"))
    {
        testLogOn = 1;
    }
    else if(sysfs_streq(buf, "LogOff"))
    {
        testLogOn = 0;
    }
    else
    {
        testLogOn = 0;
        count = -EINVAL;
    }
    return count;
}

static ssize_t tfa98xx_Log_state_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
    return 0;
}

static struct device_attribute tfa98xx_Log_state_attr =
     __ATTR(Log, S_IWUSR|S_IRUGO, tfa98xx_Log_state_show, tfa98xx_Log_state_store);

#endif


/*
 * I2C Read/Write Functions
 */

int tfa98xx_i2c_read(struct i2c_client *tfa98xx_client,	u8 reg, u8 *value,
		     int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
			.addr = tfa98xx_client->addr,
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = tfa98xx_client->addr,
			.flags = I2C_M_RD,
			.len = len,
			.buf = value,
		},
	};

	do {
		err = i2c_transfer(tfa98xx_client->adapter, msgs,
							ARRAY_SIZE(msgs));
		if (err != ARRAY_SIZE(msgs))
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != ARRAY_SIZE(msgs)) && (++tries < I2C_RETRIES));

	if (err != ARRAY_SIZE(msgs)) {
		dev_err(&tfa98xx_client->dev, "read transfer error %d\n" , err);
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

int tfa98xx_bulk_write_raw(struct snd_soc_codec *codec, const u8 *data, u8 count)
{
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
	int ret;

	ret = i2c_master_send(tfa98xx->i2c, data, count);
	if (ret == count) {
		return 0;
	} else if (ret < 0) {
		pr_err("Error I2C send %d\n", ret);
		return ret;
	} else {
		pr_err("Error I2C send size mismatch %d\n", ret);
		return -EIO;
	}
}


#ifndef VENDOR_EDIT
/*suzhiguang@MultiMedia.AudioDrv, 2015-08-24,remove monitor,it will cause system crash because of mutex lock*/
static void tfa98xx_monitor(struct work_struct *work)
{
	
	struct tfa98xx *tfa98xx = container_of(work, struct tfa98xx, delay_work.work);
	u16 val;

	mutex_lock(&tfa98xx->dsp_init_lock);

	/*
	 * check IC status bits: cold start, amp switching, speaker error
	 * and DSP watch dog bit to re init
	 */
	val = snd_soc_read(tfa98xx->codec, TFA98XX_STATUSREG);
	pr_debug("monitor SYS_STATUS: 0x%04x\n", val);
#ifndef VENDOR_EDIT
		/* zhiguang.su@MultiMedia.AudioDrv on 2015-04-21, fix bug for sound break off*/
	if ((TFA98XX_STATUSREG_ACS & val) ||
		(TFA98XX_STATUSREG_WDS & val) ||
		(TFA98XX_STATUSREG_SPKS & val) ||
	   !(TFA98XX_STATUSREG_SWS & val))
#else
/* zhiguang.su@MultiMedia.AudioDrv on 2015-08-13, changed by NXP suggestion.*/
    /* NXP: There's no need to recover in case of SPKS and SWS */
    if (TFA98XX_STATUSREG_SPKS & val)
	    pr_err("ERROR: SPKS\n");

    if (!(TFA98XX_STATUSREG_SWS & val))
		pr_err("ERROR: AMP_SWS\n");

	if ((TFA98XX_STATUSREG_ACS & val) ||
		(TFA98XX_STATUSREG_WDS & val) )

#endif
	   {
		tfa98xx->dsp_init = TFA98XX_DSP_INIT_RECOVER;

		if (TFA98XX_STATUSREG_ACS & val)
			pr_err("ERROR: ACS\n");
		if (TFA98XX_STATUSREG_WDS & val)
			pr_err("ERROR: WDS\n");

#ifndef VENDOR_EDIT
/* zhiguang.su@MultiMedia.AudioDrv on 2015-08-13, changed by NXP suggestion.*/
		if (TFA98XX_STATUSREG_SPKS & val)
			pr_err("ERROR: SPKS\n");
		if (!(TFA98XX_STATUSREG_SWS & val))
			pr_err("ERROR: AMP_SWS\n");
#endif

		/* schedule init now if the clocks are up and stable */
		if ((val & TFA98XX_STATUS_UP_MASK) == TFA98XX_STATUS_UP_MASK)
			queue_work(tfa98xx->tfa98xx_wq, &tfa98xx->init_work);
	}

	/* else just reschedule */
	queue_delayed_work(tfa98xx->tfa98xx_wq, &tfa98xx->delay_work, 5*HZ);
	mutex_unlock(&tfa98xx->dsp_init_lock);

}
#endif

static void tfa98xx_dsp_init(struct work_struct *work)
{
	struct tfa98xx *tfa98xx = container_of(work, struct tfa98xx, init_work);
	
	mutex_lock(&tfa98xx->dsp_init_lock);
	/* start the DSP using the latest profile / vstep */
	if (!tfa98xx_dsp_start(tfa98xx, tfa98xx->profile, tfa98xx->vstep))
		tfa98xx->dsp_init = TFA98XX_DSP_INIT_DONE;
	mutex_unlock(&tfa98xx->dsp_init_lock);
}

/*
 * ASOC OPS
*/

/*
static u32 tfa98xx_asrc_rates[] = {
	8000, 11025, 12000, 16000, 22050, 24000, 32000, 44100, 48000
};

static struct snd_pcm_hw_constraint_list constraints_12_24 = {
	.list   = tfa98xx_asrc_rates,
	.count  = ARRAY_SIZE(tfa98xx_asrc_rates),
};
*/
static int tfa98xx_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct tfa98xx *tfa98xx =
				snd_soc_codec_get_drvdata(codec_dai->codec);

	tfa98xx->sysclk = freq;
	return 0;
}

static int tfa98xx_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	//struct tfa98xx *tfa98xx =
	//			snd_soc_codec_get_drvdata(codec_dai->codec);
	u16 val;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		/* default value */
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
	default:
		/* only supports Slave mode */
		pr_err("tfa98xx: invalid DAI master/slave interface\n");
		return -EINVAL;
	}
	val = snd_soc_read(codec, TFA98XX_AUDIOREG);
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		/* default value */
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		val &= ~(TFA98XX_FORMAT_MASK);
		val |= TFA98XX_FORMAT_LSB;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		val &= ~(TFA98XX_FORMAT_MASK);
		val |= TFA98XX_FORMAT_MSB;
		break;
	default:
		pr_err("tfa98xx: invalid DAI interface format\n");
		return -EINVAL;
	}

	snd_soc_write(codec, TFA98XX_AUDIOREG, val);

	return 0;
}

static int tfa98xx_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);

	/* Store rate for further use during DSP init */
	tfa98xx->rate = params_rate(params);

	return 0;
}

static int tfa98xx_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);

	pr_debug("state: %d\n", mute);

	mutex_lock(&tfa98xx->dsp_init_lock);

	if (mute) {
#ifndef VENDOR_EDIT
/*suzhiguang@MultiMedia.AudioDrv, 2015-08-24,remove monitor,it will cause system crash because of mutex lock*/
		cancel_delayed_work_sync(&tfa98xx->delay_work);
#endif

		/*
		 * need to wait for amp to stop switching, to minimize
		 * pop, else I2S clk is going away too soon interrupting
		 * the dsp from smothering the amp pop while turning it
		 * off, It shouldn't take more than 50 ms for the amp
		 * switching to stop.
		 */
		if (!tfa98xx_is_pwdn(tfa98xx)) {
			tfa98xx_dsp_stop(tfa98xx);
		}
	} else {
		/*
		 * start monitor thread to check IC status bit 5secs, and
		 * re-init IC to recover.
		 */
		tfa98xx_dsp_quickstart(tfa98xx, tfa98xx->profile, tfa98xx->vstep);		
		test = 1;
#ifndef VENDOR_EDIT
/*suzhiguang@MultiMedia.AudioDrv, 2015-08-24,remove monitor,it will cause system crash because of mutex lock*/
		queue_delayed_work(tfa98xx->tfa98xx_wq, &tfa98xx->delay_work, 5*HZ);
#endif
	}

	mutex_unlock(&tfa98xx->dsp_init_lock);

	return 0;
}

static int tfa98xx_startup(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{
	
#if 0
if(substream->runtime == NULL)
	pr_debug("%s error,substream->runtime = NULL\n",__func__);

	snd_pcm_hw_constraint_list(substream->runtime, 0,
				   SNDRV_PCM_HW_PARAM_RATE,
				   &constraints_12_24);
#endif

	return 0;
}

static void tfa98xx_shutdown(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{
	//struct snd_soc_codec *codec = dai->codec;
	//struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);

}

/* Trigger callback is atomic function, It gets called when pcm is started */

static int tfa98xx_trigger(struct snd_pcm_substream *substream, int cmd,
			   struct snd_soc_dai *dai)
{
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(dai->codec);
	int ret = 0;
	
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		/*
		 * To initialize dsp all the I2S clocks must be up and running.
		 * so that the DSP's internal PLL can sync up and memory becomes
		 * accessible. Trigger callback is called when pcm write starts,
		 * so this should be the place where DSP is initialized
		 */
		 if(test)
		 {
		   test = 0;
		 }
		 else
		 {
			 queue_work(tfa98xx->tfa98xx_wq, &tfa98xx->init_work);
		 }
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

/*
 * ASOC controls
 */

static const struct snd_soc_dapm_widget tfa98xx_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("I2S1"),
	SND_SOC_DAPM_MIXER("NXP Output Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
};

static const struct snd_soc_dapm_route tfa98xx_dapm_routes[] = {
	{"NXP Output Mixer", NULL, "Playback"},
};


/*
 * Helpers for profile selection controls
 */
int tfa98xx_get_profile_ctl(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);

#ifdef VENDOR_EDIT
/* zhiguang.su@MultiMedia.AudioDrv on 2015-07-06,add lock */
	mutex_lock(&tfa98xx->dsp_init_lock);
#endif
	ucontrol->value.integer.value[0] = tfa98xx->profile;
#ifdef VENDOR_EDIT
/* zhiguang.su@MultiMedia.AudioDrv on 2015-07-06,add lock */
	mutex_unlock(&tfa98xx->dsp_init_lock);
#endif
	return 0;
}

int tfa98xx_set_profile_ctl(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
	struct tfaprofile *profiles = (struct tfaprofile *)kcontrol->private_value;
	int index = tfa98xx->profile;
	struct tfaprofile *prof = &profiles[index];

    if (tfa98xx->profile == ucontrol->value.integer.value[0])
     return 0;

#ifdef VENDOR_EDIT
/* zhiguang.su@MultiMedia.AudioDrv on 2015-07-06,add lock */
	mutex_lock(&tfa98xx->dsp_init_lock);
#endif

#ifdef VENDOR_EDIT
/* zhiguang.su@MultiMedia.AudioDrv on 2015-07-11,change for wave profile */
    tfa98xx->profile = ucontrol->value.integer.value[0];
	if (tfa98xx_is_amp_running(tfa98xx)) {
		tfaContWriteProfile(tfa98xx, tfa98xx->profile, prof->vstep);
	}
    pr_debug("%s WaveEnable %d profile %d",__func__,tfa98xx->WaveEnable,tfa98xx->profile);
#endif
#ifdef VENDOR_EDIT
/* zhiguang.su@MultiMedia.AudioDrv on 2015-07-06,add lock */
	mutex_unlock(&tfa98xx->dsp_init_lock);
#endif
	return 0;
}

int tfa98xx_info_profile_ctl(struct snd_kcontrol *kcontrol,
			 	struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 5;// tfa98xx->profile_count;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max =5 ;// tfa98xx->profile_count -1;

	return 0;
}


/*
 * Helpers for volume through vstep controls
 */
int tfa98xx_get_vol_ctl(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct tfaprofile *profiles = (struct tfaprofile *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
	int index = tfa98xx->profile;
	struct tfaprofile *prof = &profiles[index];

#ifdef VENDOR_EDIT
/* zhiguang.su@MultiMedia.AudioDrv on 2015-07-06,add lock */
	mutex_lock(&tfa98xx->dsp_init_lock);
#endif
	ucontrol->value.integer.value[0] = prof->vsteps - prof->vstep - 1;

#ifdef VENDOR_EDIT
/* zhiguang.su@MultiMedia.AudioDrv on 2015-07-06,add lock */
	mutex_unlock(&tfa98xx->dsp_init_lock);
#endif
	return 0;
}

int tfa98xx_set_vol_ctl(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct tfaprofile *profiles = (struct tfaprofile *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
	int index = tfa98xx->profile;
	struct tfaprofile *prof = &profiles[index];

	if (prof->vstep == prof->vsteps - ucontrol->value.integer.value[0] - 1)
		return 0;

#ifdef VENDOR_EDIT
/* zhiguang.su@MultiMedia.AudioDrv on 2015-07-06,add lock */
	mutex_lock(&tfa98xx->dsp_init_lock);
#endif
	prof->vstep = prof->vsteps - ucontrol->value.integer.value[0] - 1;

	if (prof->vstep < 0)
		prof->vstep = 0;

	if (tfa98xx_is_amp_running(tfa98xx)) {
		tfaContWriteProfile(tfa98xx, tfa98xx->profile, prof->vstep);
	}

#ifdef VENDOR_EDIT
/* zhiguang.su@MultiMedia.AudioDrv on 2015-07-06,add lock */
	mutex_unlock(&tfa98xx->dsp_init_lock);
#endif
	return 1;
}

int tfa98xx_info_vol_ctl(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_info *uinfo)
{
	struct tfaprofile *profiles = (struct tfaprofile *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
	struct tfaprofile *prof = &profiles[tfa98xx->profile];

	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = prof->vsteps - 1;

	return 0;
}

int tfa98xx_get_stop_ctl(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = 0;
	return 0;
}

int tfa98xx_set_stop_ctl(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);

	if ((ucontrol->value.integer.value[0] != 0) && !tfa98xx_is_pwdn(tfa98xx)) {
#ifndef VENDOR_EDIT
/*suzhiguang@MultiMedia.AudioDrv, 2015-08-24,remove monitor,it will cause system crash because of mutex lock*/
		cancel_delayed_work_sync(&tfa98xx->delay_work);
#endif

		tfa98xx_dsp_stop(tfa98xx);
	}

	ucontrol->value.integer.value[0] = 0;
	return 1;
}

#define MAX_CONTROL_NAME	32

static char prof_name[MAX_CONTROL_NAME];
static char vol_name[MAX_CONTROL_NAME];
static char stop_name[MAX_CONTROL_NAME];


static int tfa98xx_put_control(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{

	return 0;

}


static int  tfa98xx_get_control(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{

	return 0;
}


static struct snd_kcontrol_new tfa98xx_controls[] = {
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = prof_name,
		.info = tfa98xx_info_profile_ctl,
		.get = tfa98xx_get_profile_ctl,
		.put = tfa98xx_set_profile_ctl,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = vol_name,
		.info = tfa98xx_info_vol_ctl,
		.get = tfa98xx_get_vol_ctl,
		.put = tfa98xx_set_vol_ctl,
	},
	{
		.iface = SNDRV_CTL_ELEM_IFACE_MIXER,
		.name = stop_name,
		.info = snd_soc_info_bool_ext,
		.get = tfa98xx_get_stop_ctl,
		.put = tfa98xx_set_stop_ctl,
	},
	SOC_SINGLE_EXT("TFA98xx SPKR Enable", 0, 0, 1, 0,
						 tfa98xx_get_control, tfa98xx_put_control),
	SOC_SINGLE_EXT("Wave Enable", 0, 0, 1, 0,
                         tfa98xx_get_control, tfa98xx_put_control),

};


static const struct snd_soc_dai_ops tfa98xx_ops = {
	.hw_params	= tfa98xx_hw_params,
	.digital_mute	= tfa98xx_digital_mute,
	.set_fmt	= tfa98xx_set_dai_fmt,
	.set_sysclk	= tfa98xx_set_dai_sysclk,
	.startup	= tfa98xx_startup,
	.shutdown	= tfa98xx_shutdown,
	.trigger	= tfa98xx_trigger,
};


static struct snd_soc_dai_driver tfa98xx_dai = {
	.name = "tfa98xx_codec",
	.playback = {
		     .stream_name = "Playback",
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = TFA98XX_RATES,
		     .formats = TFA98XX_FORMATS,},
	.ops = &tfa98xx_ops,
	.symmetric_rates = 1,
};

static int tfa98xx_probe(struct snd_soc_codec *codec)
{
	struct tfa98xx *tfa98xx = snd_soc_codec_get_drvdata(codec);
	int ret;
	u16 rev;

	codec->control_data = tfa98xx->regmap;
	tfa98xx->codec = codec;
	codec->cache_bypass = true;

	ret = snd_soc_codec_set_cache_io(codec, 8, 16, SND_SOC_REGMAP);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	rev = snd_soc_read(codec, TFA98XX_REVISIONNUMBER);
	dev_info(codec->dev, "ID revision 0x%04x\n", rev);
	tfa98xx->rev = rev & 0xff;
	tfa98xx->subrev = (rev >> 8) & 0xff;

	snd_soc_dapm_new_controls(&codec->dapm, tfa98xx_dapm_widgets,
				  ARRAY_SIZE(tfa98xx_dapm_widgets));

	snd_soc_dapm_add_routes(&codec->dapm, tfa98xx_dapm_routes,
				ARRAY_SIZE(tfa98xx_dapm_routes));

	snd_soc_dapm_new_widgets(&codec->dapm);
	snd_soc_dapm_sync(&codec->dapm);

	ret = tfa98xx_cnt_loadfile(tfa98xx, 0);
	if(ret)
		return ret;

	tfa98xx->profile_current = -1;
	/* Overwrite kcontrol values that need container information */
	tfa98xx_controls[0].private_value = (unsigned long)tfa98xx->profiles,
	tfa98xx_controls[1].private_value = (unsigned long)tfa98xx->profiles,
	scnprintf(prof_name, MAX_CONTROL_NAME, "%s Profile", tfa98xx->fw.name);
	scnprintf(vol_name, MAX_CONTROL_NAME, "%s Master Volume", tfa98xx->fw.name);
	scnprintf(stop_name, MAX_CONTROL_NAME, "%s Stop", tfa98xx->fw.name);
	//scnprintf(speaker_enable, MAX_CONTROL_NAME, "%s Stop", tfa98xx->fw.name);

	snd_soc_add_codec_controls(codec, tfa98xx_controls, ARRAY_SIZE(tfa98xx_controls));

	dev_info(codec->dev, "tfa98xx codec registered");
#ifdef VENDOR_EDIT
	//zhiguang.su add
	g_tfa98xx = tfa98xx;
	//zhiguang.su add end
#endif	
	return 0;
}

static int tfa98xx_remove(struct snd_soc_codec *codec)
{
	dev_info(codec->dev, "tfa98xx codec removed");
	return 0;
}

static struct snd_soc_codec_driver tfa98xx_soc_codec = {
	.probe = tfa98xx_probe,
	.remove = tfa98xx_remove,
};

static const struct regmap_config tfa98xx_regmap = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = TFA98XX_MAX_REGISTER,
	.cache_type = REGCACHE_RBTREE,
};

static int tfa98xx_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct tfa98xx *tfa98xx;
	int ret;
	struct device_node *np = i2c->dev.of_node;
    int error = 0;

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		dev_err(&i2c->dev, "check_functionality failed\n");
		return -EIO;
	}

	tfa98xx = devm_kzalloc(&i2c->dev, sizeof(struct tfa98xx),
			       GFP_KERNEL);
	if (tfa98xx == NULL)
		return -ENOMEM;

	tfa98xx->i2c = i2c;
	tfa98xx->dsp_init = TFA98XX_DSP_INIT_PENDING;

	tfa98xx->regmap = devm_regmap_init_i2c(i2c, &tfa98xx_regmap);
	if (IS_ERR(tfa98xx->regmap)) {
		ret = PTR_ERR(tfa98xx->regmap);
		dev_err(&i2c->dev, "Failed to allocate regmap: %d\n", ret);
		return ret;
	}

	i2c_set_clientdata(i2c, tfa98xx);
	mutex_init(&tfa98xx->dsp_init_lock);

	/* work queue will be used to load DSP fw on first audio playback */
	tfa98xx->tfa98xx_wq = create_singlethread_workqueue("tfa98xx");
	if (tfa98xx->tfa98xx_wq == NULL) {
		ret = -ENOMEM;
		goto wq_fail;
	}

#ifdef VENDOR_EDIT
	//zhiguang.su add 1218
	tfa98xx->rst_gpio = of_get_named_gpio(np, "reset_gpio",0);
	ret = gpio_request(tfa98xx->rst_gpio, "tfa reset gpio");
	if (ret < 0)
	{
		pr_err("%s: tfa reset gpio_request failed: %d\n",__func__, ret);
		goto gpio_fail;
	}
	gpio_direction_output(tfa98xx->rst_gpio, 1);
/*zhiguang.su@MultiMedia.AudioDrv on 2015-05-18,optimize for speed */
	udelay(100);
	gpio_direction_output(tfa98xx->rst_gpio, 0);
	//zhiguang.su add end 1218
#endif

	INIT_WORK(&tfa98xx->init_work, tfa98xx_dsp_init);

#ifndef VENDOR_EDIT
/*suzhiguang@MultiMedia.AudioDrv, 2015-08-24,remove monitor,it will cause system crash because of mutex lock*/
	INIT_DELAYED_WORK(&tfa98xx->delay_work, tfa98xx_monitor);
#endif

	/* register codec */
	ret = snd_soc_register_codec(&i2c->dev, &tfa98xx_soc_codec,
				     &tfa98xx_dai, 1);
	if (ret < 0) {
		pr_err("%s: Error registering tfa98xx codec", __func__);
		goto codec_fail;
	}

#ifdef VENDOR_EDIT
/*zhiguang.su@MultiMedia.AudioDrv on 2015-05-18,optimize for speed */
	pr_debug("tfa98xx probed successfully!");
#endif


#ifdef VENDOR_EDIT
/* zhiguang.su@MultiMedia.AudioDrv on 2015-05-29, add for smart pa calibtation*/
	error = sysfs_create_file(&i2c->dev.kobj, &tfa98xx_state_attr.attr);
    if(error < 0)
    {
        pr_err("%s sysfs_create_file tfa98xx_state_attr err.",__func__);
    }

	error = sysfs_create_file(&i2c->dev.kobj, &tfa98xx_Log_state_attr.attr);
    if(error < 0)
    {
        pr_err("%s sysfs_create_file tfa98xx_Log_state_attr err.",__func__);
    }

#endif
	return ret;

codec_fail:
	destroy_workqueue(tfa98xx->tfa98xx_wq);
gpio_fail:
wq_fail:
	snd_soc_unregister_codec(&i2c->dev);

	return ret;
}

static int tfa98xx_i2c_remove(struct i2c_client *client)
{
	struct tfa98xx *tfa98xx = i2c_get_clientdata(client);

	snd_soc_unregister_codec(&client->dev);
	destroy_workqueue(tfa98xx->tfa98xx_wq);
	return 0;
}

static const struct i2c_device_id tfa98xx_i2c_id[] = {
	{ "tfa98xx", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tfa98xx_i2c_id);

#ifdef CONFIG_OF
static struct of_device_id tfa98xx_match_tbl[] = {
	{ .compatible = "nxp,tfa9890" },
	{ },
};
MODULE_DEVICE_TABLE(of, tfa98xx_match_tbl);
#endif

static struct i2c_driver tfa98xx_i2c_driver = {
	.driver = {
		.name = "tfa98xx",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(tfa98xx_match_tbl),
	},
	.probe =    tfa98xx_i2c_probe,
	.remove =   tfa98xx_i2c_remove,
	.id_table = tfa98xx_i2c_id,
};

module_i2c_driver(tfa98xx_i2c_driver);
