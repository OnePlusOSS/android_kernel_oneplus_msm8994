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

#ifndef _TFA98XX_H
#define _TFA98XX_H

#include <sound/core.h>
#include <sound/soc.h>

/* Revision IDs for tfa98xx variants */
#define REV_TFA9887	0x12
#define REV_TFA9890	0x80
#define REV_TFA9895	0x12
#define REV_TFA9897	0x97


struct tfaprofile;
struct nxpTfaDevice;
struct nxpTfaProfile;
struct nxpTfaVolumeStep2File;

struct tfaprofile {
	struct nxpTfaProfile *profile;
	struct nxpTfaVolumeStep2File *vp;
	int vsteps;
	int vstep;
	int index;
	int state;
	char *name;
};

struct tfa98xx_firmware {
	void			*base;
	struct nxpTfaDevice 	*dev;
	char			*name;
};

struct tfa98xx {
	struct regmap *regmap;
	struct i2c_client *i2c;
	struct snd_soc_codec *codec;
	struct workqueue_struct *tfa98xx_wq;
	struct work_struct init_work;
	struct delayed_work delay_work;
	struct mutex dsp_init_lock;
	int dsp_init;
	int speaker_imp;
	int sysclk;
	int rst_gpio;
	int mode;
	int mode_switched;
	int curr_mode;
	int vol_idx;
	int curr_vol_idx;
	int ic_version;
	u8 rev;
	u8 subrev;
	int vstep;
	int profile;
	int profile_current;
	int profile_count;
	int has_drc;
	int rate;
#ifdef VENDOR_EDIT
/* zhiguang.su@MultiMedia.AudioDrv on 2015-07-11,change for wave profile */
    int WaveEnable;
    int profileChange;
    int i2sOn;
#endif
	struct tfaprofile *profiles;
	struct tfa98xx_firmware fw;

	int (*info_profile)(struct snd_kcontrol *kcontrol,
			    struct snd_ctl_elem_info *uinfo);
	int (*set_profile)(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol);
	int (*get_profile)(struct snd_kcontrol *kcontrol,
			   struct snd_ctl_elem_value *ucontrol);

	int (*info_vstep)(struct snd_kcontrol *kcontrol,
			  struct snd_ctl_elem_info *uinfo);
	int (*set_vstep)(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol);
	int (*get_vstep)(struct snd_kcontrol *kcontrol,
			 struct snd_ctl_elem_value *ucontrol);

	struct snd_kcontrol_new *(*build_profile_controls)(struct tfa98xx *tfa98xx, int *kcontrol_count);
};

#endif
