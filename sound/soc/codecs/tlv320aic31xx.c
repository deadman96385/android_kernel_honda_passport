/*
 * ALSA SoC TLV320AIC31XX codec driver
 *
 * Copyright (C) 2014 Texas Instruments, Inc.
 *
 * Author: Jyri Sarha <jsarha@ti.com>
 *
 * Based on ground work by: Ajit Kulkarni <x0175765@ti.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED AS IS AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * The TLV320AIC31xx series of audio codec is a low-power, highly integrated
 * high performance codec which provides a stereo DAC, a mono ADC,
 * and mono/stereo Class-D speaker driver.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <dt-bindings/sound/tlv320aic31xx-micbias.h>

#include "tlv320aic31xx.h"

static const struct reg_default aic31xx_reg_defaults[] = {
	{ AIC31XX_CLKMUX, 0x00 },
	{ AIC31XX_PLLPR, 0x11 },
	{ AIC31XX_PLLJ, 0x04 },
	{ AIC31XX_PLLDMSB, 0x00 },
	{ AIC31XX_PLLDLSB, 0x00 },
	{ AIC31XX_NDAC, 0x01 },
	{ AIC31XX_MDAC, 0x01 },
	{ AIC31XX_DOSRMSB, 0x00 },
	{ AIC31XX_DOSRLSB, 0x80 },
	{ AIC31XX_NADC, 0x01 },
	{ AIC31XX_MADC, 0x01 },
	{ AIC31XX_AOSR, 0x80 },
	{ AIC31XX_IFACE1, 0x00 },
	{ AIC31XX_DATA_OFFSET, 0x00 },
	{ AIC31XX_IFACE2, 0x00 },
	{ AIC31XX_BCLKN, 0x01 },
	{ AIC31XX_DACSETUP, 0x14 },
	{ AIC31XX_DACMUTE, 0x0c },
	{ AIC31XX_LDACVOL, 0x00 },
	{ AIC31XX_RDACVOL, 0x00 },
	{ AIC31XX_ADCSETUP, 0x00 },
	{ AIC31XX_ADCFGA, 0x80 },
	{ AIC31XX_ADCVOL, 0x00 },
	{ AIC31XX_HPDRIVER, 0x04 },
	{ AIC31XX_SPKAMP, 0x06 },
	{ AIC31XX_DACMIXERROUTE, 0x00 },
	{ AIC31XX_LANALOGHPL, 0x7f },
	{ AIC31XX_RANALOGHPR, 0x7f },
	{ AIC31XX_LANALOGSPL, 0x7f },
	{ AIC31XX_RANALOGSPR, 0x7f },
	{ AIC31XX_HPLGAIN, 0x02 },
	{ AIC31XX_HPRGAIN, 0x02 },
	{ AIC31XX_SPLGAIN, 0x00 },
	{ AIC31XX_SPRGAIN, 0x00 },
	{ AIC31XX_MICBIAS, 0x00 },
	{ AIC31XX_MICPGA, 0x80 },
	{ AIC31XX_MICPGAPI, 0x00 },
	{ AIC31XX_MICPGAMI, 0x00 },
};

static bool aic31xx_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AIC31XX_PAGECTL: /* regmap implementation requires this */
	case AIC31XX_RESET: /* always clears after write */
	case AIC31XX_OT_FLAG:
	case AIC31XX_ADCFLAG:
	case AIC31XX_DACFLAG1:
	case AIC31XX_DACFLAG2:
	case AIC31XX_OFFLAG: /* Sticky interrupt flags */
	case AIC31XX_INTRDACFLAG: /* Sticky interrupt flags */
	case AIC31XX_INTRADCFLAG: /* Sticky interrupt flags */
	case AIC31XX_INTRDACFLAG2:
	case AIC31XX_INTRADCFLAG2:
		return true;
	}
	return false;
}

static bool aic31xx_writeable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AIC31XX_OT_FLAG:
	case AIC31XX_ADCFLAG:
	case AIC31XX_DACFLAG1:
	case AIC31XX_DACFLAG2:
	case AIC31XX_OFFLAG: /* Sticky interrupt flags */
	case AIC31XX_INTRDACFLAG: /* Sticky interrupt flags */
	case AIC31XX_INTRADCFLAG: /* Sticky interrupt flags */
	case AIC31XX_INTRDACFLAG2:
	case AIC31XX_INTRADCFLAG2:
		return false;
	}
	return true;
}

static const struct regmap_range_cfg aic31xx_ranges[] = {
	{
		.range_min = 0,
		.range_max = 12 * 128,
		.selector_reg = AIC31XX_PAGECTL,
		.selector_mask = 0xff,
		.selector_shift = 0,
		.window_start = 0,
		.window_len = 128,
	},
};

struct regmap_config aic31xx_i2c_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = aic31xx_writeable,
	.volatile_reg = aic31xx_volatile,
	.reg_defaults = aic31xx_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(aic31xx_reg_defaults),
	.cache_type = REGCACHE_RBTREE,
	.ranges = aic31xx_ranges,
	.num_ranges = ARRAY_SIZE(aic31xx_ranges),
	.max_register = 12 * 128,
};

#define AIC31XX_NUM_SUPPLIES	6
static const char * const aic31xx_supply_names[AIC31XX_NUM_SUPPLIES] = {
	"HPVDD",
	"SPRVDD",
	"SPLVDD",
	"AVDD",
	"IOVDD",
	"DVDD",
};

struct aic31xx_disable_nb {
	struct notifier_block nb;
	struct aic31xx_priv *aic31xx;
};

struct aic31xx_priv {
	struct snd_soc_codec *codec;
	u8 i2c_regs_status;
	struct device *dev;
	struct regmap *regmap;
	struct aic31xx_pdata pdata;
	struct regulator_bulk_data supplies[AIC31XX_NUM_SUPPLIES];
	struct aic31xx_disable_nb disable_nb[AIC31XX_NUM_SUPPLIES];
	unsigned int sysclk;
	u8 p_div;
	int rate_div_line;
};

struct aic31xx_rate_divs {
	u32 mclk_p;
	u32 rate;
	u8 pll_j;
	u16 pll_d;
	u16 dosr;
	u8 ndac;
	u8 mdac;
	u8 aosr;
	u8 nadc;
	u8 madc;
};

/* ADC dividers can be disabled by cofiguring them to 0 */
static const struct aic31xx_rate_divs aic31xx_divs[] = {
	/* mclk/p    rate  pll: j     d        dosr ndac mdac  aors nadc madc */
	/* 8k rate */
	{12000000,   8000,	8, 1920,	128,  48,  2,	128,  48,  2},
	{12000000,   8000,	8, 1920,	128,  32,  3,	128,  32,  3},
	{12500000,   8000,	7, 8643,	128,  48,  2,	128,  48,  2},
	/* 11.025k rate */
	{12000000,  11025,	7, 5264,	128,  32,  2,	128,  32,  2},
	{12000000,  11025,	8, 4672,	128,  24,  3,	128,  24,  3},
	{12500000,  11025,	7, 2253,	128,  32,  2,	128,  32,  2},
	/* 16k rate */
	{12000000,  16000,	8, 1920,	128,  24,  2,	128,  24,  2},
	{12000000,  16000,	8, 1920,	128,  16,  3,	128,  16,  3},
	{12500000,  16000,	7, 8643,	128,  24,  2,	128,  24,  2},
	/* 22.05k rate */
	{12000000,  22050,	7, 5264,	128,  16,  2,	128,  16,  2},
	{12000000,  22050,	8, 4672,	128,  12,  3,	128,  12,  3},
	{12500000,  22050,	7, 2253,	128,  16,  2,	128,  16,  2},
	/* 32k rate */
	{12000000,  32000,	8, 1920,	128,  12,  2,	128,  12,  2},
	{12000000,  32000,	8, 1920,	128,   8,  3,	128,   8,  3},
	{12500000,  32000,	7, 8643,	128,  12,  2,	128,  12,  2},
	/* 44.1k rate */
	{12000000,  44100,	7, 5264,	128,   8,  2,	128,   8,  2},
	{12000000,  44100,	8, 4672,	128,   6,  3,	128,   6,  3},
	{12500000,  44100,	7, 2253,	128,   8,  2,	128,   8,  2},
	/* 48k rate */
	{12000000,  48000,	8, 1920,	128,   8,  2,	128,   8,  2},
	{12000000,  48000,	7, 6800,	 96,   5,  4,	 96,   5,  4},
	{12500000,  48000,	7, 8643,	128,   8,  2,	128,   8,  2},
	/* 88.2k rate */
	{12000000,  88200,	7, 5264,	 64,   8,  2,	 64,   8,  2},
	{12000000,  88200,	8, 4672,	 64,   6,  3,	 64,   6,  3},
	{12500000,  88200,	7, 2253,	 64,   8,  2,	 64,   8,  2},
	/* 96k rate */
	{12000000,  96000,	8, 1920,	 64,   8,  2,	 64,   8,  2},
	{12000000,  96000,	7, 6800,	 48,   5,  4,	 48,   5,  4},
	{12500000,  96000,	7, 8643,	 64,   8,  2,	 64,   8,  2},
	/* 176.4k rate */
	{12000000, 176400,	7, 5264,	 32,   8,  2,	 32,   8,  2},
	{12000000, 176400,	8, 4672,	 32,   6,  3,	 32,   6,  3},
	{12500000, 176400,	7, 2253,	 32,   8,  2,	 32,   8,  2},
	/* 192k rate */
	{12000000, 192000,	8, 1920,	 32,   8,  2,	 32,   8,  2},
	{12000000, 192000,	7, 6800,	 24,   5,  4,	 24,   5,  4},
	{12500000, 192000,	7, 8643,	 32,   8,  2,	 32,   8,  2},
};

static const char * const ldac_in_text[] = {
	"Off", "Left Data", "Right Data", "Mono"
};

static const char * const rdac_in_text[] = {
	"Off", "Right Data", "Left Data", "Mono"
};

static SOC_ENUM_SINGLE_DECL(ldac_in_enum, AIC31XX_DACSETUP, 4, ldac_in_text);

static SOC_ENUM_SINGLE_DECL(rdac_in_enum, AIC31XX_DACSETUP, 2, rdac_in_text);

static const char * const mic_select_text[] = {
	"Off", "FFR 10 Ohm", "FFR 20 Ohm", "FFR 40 Ohm"
};

static const
SOC_ENUM_SINGLE_DECL(mic1lp_p_enum, AIC31XX_MICPGAPI, 6, mic_select_text);
static const
SOC_ENUM_SINGLE_DECL(mic1rp_p_enum, AIC31XX_MICPGAPI, 4, mic_select_text);
static const
SOC_ENUM_SINGLE_DECL(mic1lm_p_enum, AIC31XX_MICPGAPI, 2, mic_select_text);

static const
SOC_ENUM_SINGLE_DECL(cm_m_enum, AIC31XX_MICPGAMI, 6, mic_select_text);
static const
SOC_ENUM_SINGLE_DECL(mic1lm_m_enum, AIC31XX_MICPGAMI, 4, mic_select_text);

static const DECLARE_TLV_DB_SCALE(dac_vol_tlv, -6350, 50, 0);
static const DECLARE_TLV_DB_SCALE(adc_fgain_tlv, 0, 10, 0);
static const DECLARE_TLV_DB_SCALE(adc_cgain_tlv, -2000, 50, 0);
static const DECLARE_TLV_DB_SCALE(mic_pga_tlv, 0, 50, 0);
static const DECLARE_TLV_DB_SCALE(hp_drv_tlv, 0, 100, 0);
static const DECLARE_TLV_DB_SCALE(class_D_drv_tlv, 600, 600, 0);
static const DECLARE_TLV_DB_SCALE(hp_vol_tlv, -6350, 50, 0);
static const DECLARE_TLV_DB_SCALE(sp_vol_tlv, -6350, 50, 0);

/*
 * controls to be exported to the user space
 */
static const struct snd_kcontrol_new aic31xx_snd_controls[] = {
	SOC_DOUBLE_R_S_TLV("DAC Playback Volume", AIC31XX_LDACVOL,
			   AIC31XX_RDACVOL, 0, -127, 48, 7, 0, dac_vol_tlv),

	SOC_SINGLE_TLV("ADC Fine Capture Volume", AIC31XX_ADCFGA, 4, 4, 1,
		       adc_fgain_tlv),

	SOC_SINGLE("ADC Capture Switch", AIC31XX_ADCFGA, 7, 1, 1),
	SOC_DOUBLE_R_S_TLV("ADC Capture Volume", AIC31XX_ADCVOL, AIC31XX_ADCVOL,
			   0, -24, 40, 6, 0, adc_cgain_tlv),

	SOC_SINGLE_TLV("Mic PGA Capture Volume", AIC31XX_MICPGA, 0,
		       119, 0, mic_pga_tlv),

	SOC_DOUBLE_R("HP Driver Playback Switch", AIC31XX_HPLGAIN,
		     AIC31XX_HPRGAIN, 2, 1, 0),
	SOC_DOUBLE_R_TLV("HP Driver Playback Volume", AIC31XX_HPLGAIN,
			 AIC31XX_HPRGAIN, 3, 0x09, 0, hp_drv_tlv),

	SOC_DOUBLE_R_TLV("HP Analog Playback Volume", AIC31XX_LANALOGHPL,
			 AIC31XX_RANALOGHPR, 0, 0x7F, 1, hp_vol_tlv),
};

static const struct snd_kcontrol_new aic311x_snd_controls[] = {
	SOC_DOUBLE_R("Speaker Driver Playback Switch", AIC31XX_SPLGAIN,
		     AIC31XX_SPRGAIN, 2, 1, 0),
	SOC_DOUBLE_R_TLV("Speaker Driver Playback Volume", AIC31XX_SPLGAIN,
			 AIC31XX_SPRGAIN, 3, 3, 0, class_D_drv_tlv),

	SOC_DOUBLE_R_TLV("Speaker Analog Playback Volume", AIC31XX_LANALOGSPL,
			 AIC31XX_RANALOGSPR, 0, 0x7F, 1, sp_vol_tlv),
};

static const struct snd_kcontrol_new aic310x_snd_controls[] = {
	SOC_SINGLE("Speaker Driver Playback Switch", AIC31XX_SPLGAIN,
		   2, 1, 0),
	SOC_SINGLE_TLV("Speaker Driver Playback Volume", AIC31XX_SPLGAIN,
		       3, 3, 0, class_D_drv_tlv),

	SOC_SINGLE_TLV("Speaker Analog Playback Volume", AIC31XX_LANALOGSPL,
		       0, 0x7F, 1, sp_vol_tlv),
};

static const struct snd_kcontrol_new ldac_in_control =
	SOC_DAPM_ENUM("DAC Left Input", ldac_in_enum);

static const struct snd_kcontrol_new rdac_in_control =
	SOC_DAPM_ENUM("DAC Right Input", rdac_in_enum);

int aic31xx_wait_bits(struct aic31xx_priv *aic31xx, unsigned int reg,
		      unsigned int mask, unsigned int wbits, int sleep,
		      int count)
{
	unsigned int bits;
	int counter = count;
	int ret = regmap_read(aic31xx->regmap, reg, &bits);
	while ((bits & mask) != wbits && counter && !ret) {
		usleep_range(sleep, sleep * 2);
		ret = regmap_read(aic31xx->regmap, reg, &bits);
		counter--;
	}
	if ((bits & mask) != wbits) {
		dev_err(aic31xx->dev,
			"%s: Failed! 0x%x was 0x%x expected 0x%x (%d, 0x%x, %d us)\n",
			__func__, reg, bits, wbits, ret, mask,
			(count - counter) * sleep);
		ret = -1;
	}
	return ret;
}

#define WIDGET_BIT(reg, shift) (((shift) << 8) | (reg))

static int aic31xx_dapm_power_event(struct snd_soc_dapm_widget *w,
				    struct snd_kcontrol *kcontrol, int event)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(w->codec);
	unsigned int reg = AIC31XX_DACFLAG1;
	unsigned int mask;

	switch (WIDGET_BIT(w->reg, w->shift)) {
	case WIDGET_BIT(AIC31XX_DACSETUP, 7):
		mask = AIC31XX_LDACPWRSTATUS_MASK;
		break;
	case WIDGET_BIT(AIC31XX_DACSETUP, 6):
		mask = AIC31XX_RDACPWRSTATUS_MASK;
		break;
	case WIDGET_BIT(AIC31XX_HPDRIVER, 7):
		mask = AIC31XX_HPLDRVPWRSTATUS_MASK;
		break;
	case WIDGET_BIT(AIC31XX_HPDRIVER, 6):
		mask = AIC31XX_HPRDRVPWRSTATUS_MASK;
		break;
	case WIDGET_BIT(AIC31XX_SPKAMP, 7):
		mask = AIC31XX_SPLDRVPWRSTATUS_MASK;
		break;
	case WIDGET_BIT(AIC31XX_SPKAMP, 6):
		mask = AIC31XX_SPRDRVPWRSTATUS_MASK;
		break;
	case WIDGET_BIT(AIC31XX_ADCSETUP, 7):
		mask = AIC31XX_ADCPWRSTATUS_MASK;
		reg = AIC31XX_ADCFLAG;
		break;
	default:
		dev_err(w->codec->dev, "Unknown widget '%s' calling %s/n",
			w->name, __func__);
		return -EINVAL;
	}

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		return aic31xx_wait_bits(aic31xx, reg, mask, mask, 5000, 100);
	case SND_SOC_DAPM_POST_PMD:
		return aic31xx_wait_bits(aic31xx, reg, mask, 0, 5000, 100);
	default:
		dev_dbg(w->codec->dev,
			"Unhandled dapm widget event %d from %s\n",
			event, w->name);
	}
	return 0;
}

static const struct snd_kcontrol_new left_output_switches[] = {
	SOC_DAPM_SINGLE("From Left DAC", AIC31XX_DACMIXERROUTE, 6, 1, 0),
	SOC_DAPM_SINGLE("From MIC1LP", AIC31XX_DACMIXERROUTE, 5, 1, 0),
	SOC_DAPM_SINGLE("From MIC1RP", AIC31XX_DACMIXERROUTE, 4, 1, 0),
};

static const struct snd_kcontrol_new right_output_switches[] = {
	SOC_DAPM_SINGLE("From Right DAC", AIC31XX_DACMIXERROUTE, 2, 1, 0),
	SOC_DAPM_SINGLE("From MIC1RP", AIC31XX_DACMIXERROUTE, 1, 1, 0),
};

static const struct snd_kcontrol_new p_term_mic1lp =
	SOC_DAPM_ENUM("MIC1LP P-Terminal", mic1lp_p_enum);

static const struct snd_kcontrol_new p_term_mic1rp =
	SOC_DAPM_ENUM("MIC1RP P-Terminal", mic1rp_p_enum);

static const struct snd_kcontrol_new p_term_mic1lm =
	SOC_DAPM_ENUM("MIC1LM P-Terminal", mic1lm_p_enum);

static const struct snd_kcontrol_new m_term_mic1lm =
	SOC_DAPM_ENUM("MIC1LM M-Terminal", mic1lm_m_enum);

static const struct snd_kcontrol_new aic31xx_dapm_hpl_switch =
	SOC_DAPM_SINGLE("Switch", AIC31XX_LANALOGHPL, 7, 1, 0);

static const struct snd_kcontrol_new aic31xx_dapm_hpr_switch =
	SOC_DAPM_SINGLE("Switch", AIC31XX_RANALOGHPR, 7, 1, 0);

static const struct snd_kcontrol_new aic31xx_dapm_spl_switch =
	SOC_DAPM_SINGLE("Switch", AIC31XX_LANALOGSPL, 7, 1, 0);

static const struct snd_kcontrol_new aic31xx_dapm_spr_switch =
	SOC_DAPM_SINGLE("Switch", AIC31XX_RANALOGSPR, 7, 1, 0);

static int mic_bias_event(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/* change mic bias voltage to user defined */
		snd_soc_update_bits(codec, AIC31XX_MICBIAS,
				    AIC31XX_MICBIAS_MASK,
				    aic31xx->pdata.micbias_vg <<
				    AIC31XX_MICBIAS_SHIFT);
		dev_dbg(codec->dev, "%s: turned on\n", __func__);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		/* turn mic bias off */
		snd_soc_update_bits(codec, AIC31XX_MICBIAS,
				    AIC31XX_MICBIAS_MASK, 0);
		dev_dbg(codec->dev, "%s: turned off\n", __func__);
		break;
	}
	return 0;
}

static const struct snd_soc_dapm_widget aic31xx_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("DAC IN", "DAC Playback", 0, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_MUX("DAC Left Input",
			 SND_SOC_NOPM, 0, 0, &ldac_in_control),
	SND_SOC_DAPM_MUX("DAC Right Input",
			 SND_SOC_NOPM, 0, 0, &rdac_in_control),
	/* DACs */
	SND_SOC_DAPM_DAC_E("DAC Left", "Left Playback",
			   AIC31XX_DACSETUP, 7, 0, aic31xx_dapm_power_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	SND_SOC_DAPM_DAC_E("DAC Right", "Right Playback",
			   AIC31XX_DACSETUP, 6, 0, aic31xx_dapm_power_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),

	/* Output Mixers */
	SND_SOC_DAPM_MIXER("Output Left", SND_SOC_NOPM, 0, 0,
			   left_output_switches,
			   ARRAY_SIZE(left_output_switches)),
	SND_SOC_DAPM_MIXER("Output Right", SND_SOC_NOPM, 0, 0,
			   right_output_switches,
			   ARRAY_SIZE(right_output_switches)),

	SND_SOC_DAPM_SWITCH("HP Left", SND_SOC_NOPM, 0, 0,
			    &aic31xx_dapm_hpl_switch),
	SND_SOC_DAPM_SWITCH("HP Right", SND_SOC_NOPM, 0, 0,
			    &aic31xx_dapm_hpr_switch),

	/* Output drivers */
	SND_SOC_DAPM_OUT_DRV_E("HPL Driver", AIC31XX_HPDRIVER, 7, 0,
			       NULL, 0, aic31xx_dapm_power_event,
			       SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_OUT_DRV_E("HPR Driver", AIC31XX_HPDRIVER, 6, 0,
			       NULL, 0, aic31xx_dapm_power_event,
			       SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_POST_PMU),

	/* ADC */
	SND_SOC_DAPM_ADC_E("ADC", "Capture", AIC31XX_ADCSETUP, 7, 0,
			   aic31xx_dapm_power_event, SND_SOC_DAPM_POST_PMU |
			   SND_SOC_DAPM_POST_PMD),

	/* Input Selection to MIC_PGA */
	SND_SOC_DAPM_MUX("MIC1LP P-Terminal", SND_SOC_NOPM, 0, 0,
			 &p_term_mic1lp),
	SND_SOC_DAPM_MUX("MIC1RP P-Terminal", SND_SOC_NOPM, 0, 0,
			 &p_term_mic1rp),
	SND_SOC_DAPM_MUX("MIC1LM P-Terminal", SND_SOC_NOPM, 0, 0,
			 &p_term_mic1lm),

	SND_SOC_DAPM_MUX("MIC1LM M-Terminal", SND_SOC_NOPM, 0, 0,
			 &m_term_mic1lm),
	/* Enabling & Disabling MIC Gain Ctl */
	SND_SOC_DAPM_PGA("MIC_GAIN_CTL", AIC31XX_MICPGA,
			 7, 1, NULL, 0),

	/* Mic Bias */
	SND_SOC_DAPM_SUPPLY("MICBIAS", SND_SOC_NOPM, 0, 0, mic_bias_event,
			    SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	/* Outputs */
	SND_SOC_DAPM_OUTPUT("HPL"),
	SND_SOC_DAPM_OUTPUT("HPR"),

	/* Inputs */
	SND_SOC_DAPM_INPUT("MIC1LP"),
	SND_SOC_DAPM_INPUT("MIC1RP"),
	SND_SOC_DAPM_INPUT("MIC1LM"),
};

static const struct snd_soc_dapm_widget aic311x_dapm_widgets[] = {
	/* AIC3111 and AIC3110 have stereo class-D amplifier */
	SND_SOC_DAPM_OUT_DRV_E("SPL ClassD", AIC31XX_SPKAMP, 7, 0, NULL, 0,
			       aic31xx_dapm_power_event, SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_OUT_DRV_E("SPR ClassD", AIC31XX_SPKAMP, 6, 0, NULL, 0,
			       aic31xx_dapm_power_event, SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SWITCH("Speaker Left", SND_SOC_NOPM, 0, 0,
			    &aic31xx_dapm_spl_switch),
	SND_SOC_DAPM_SWITCH("Speaker Right", SND_SOC_NOPM, 0, 0,
			    &aic31xx_dapm_spr_switch),
	SND_SOC_DAPM_OUTPUT("SPL"),
	SND_SOC_DAPM_OUTPUT("SPR"),
};

/* AIC3100 and AIC3120 have only mono class-D amplifier */
static const struct snd_soc_dapm_widget aic310x_dapm_widgets[] = {
	SND_SOC_DAPM_OUT_DRV_E("SPK ClassD", AIC31XX_SPKAMP, 7, 0, NULL, 0,
			       aic31xx_dapm_power_event, SND_SOC_DAPM_POST_PMU |
			       SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SWITCH("Speaker", SND_SOC_NOPM, 0, 0,
			    &aic31xx_dapm_spl_switch),
	SND_SOC_DAPM_OUTPUT("SPK"),
};

static const struct snd_soc_dapm_route
aic31xx_audio_map[] = {
	/* DAC Input Routing */
	{"DAC Left Input", "Left Data", "DAC IN"},
	{"DAC Left Input", "Right Data", "DAC IN"},
	{"DAC Left Input", "Mono", "DAC IN"},
	{"DAC Right Input", "Left Data", "DAC IN"},
	{"DAC Right Input", "Right Data", "DAC IN"},
	{"DAC Right Input", "Mono", "DAC IN"},
	{"DAC Left", NULL, "DAC Left Input"},
	{"DAC Right", NULL, "DAC Right Input"},

	/* Mic input */
	{"MIC1LP P-Terminal", "FFR 10 Ohm", "MIC1LP"},
	{"MIC1LP P-Terminal", "FFR 20 Ohm", "MIC1LP"},
	{"MIC1LP P-Terminal", "FFR 40 Ohm", "MIC1LP"},
	{"MIC1RP P-Terminal", "FFR 10 Ohm", "MIC1RP"},
	{"MIC1RP P-Terminal", "FFR 20 Ohm", "MIC1RP"},
	{"MIC1RP P-Terminal", "FFR 40 Ohm", "MIC1RP"},
	{"MIC1LM P-Terminal", "FFR 10 Ohm", "MIC1LM"},
	{"MIC1LM P-Terminal", "FFR 20 Ohm", "MIC1LM"},
	{"MIC1LM P-Terminal", "FFR 40 Ohm", "MIC1LM"},

	{"MIC1LM M-Terminal", "FFR 10 Ohm", "MIC1LM"},
	{"MIC1LM M-Terminal", "FFR 20 Ohm", "MIC1LM"},
	{"MIC1LM M-Terminal", "FFR 40 Ohm", "MIC1LM"},

	{"MIC_GAIN_CTL", NULL, "MIC1LP P-Terminal"},
	{"MIC_GAIN_CTL", NULL, "MIC1RP P-Terminal"},
	{"MIC_GAIN_CTL", NULL, "MIC1LM P-Terminal"},
	{"MIC_GAIN_CTL", NULL, "MIC1LM M-Terminal"},

	{"ADC", NULL, "MIC_GAIN_CTL"},

	/* Left Output */
	{"Output Left", "From Left DAC", "DAC Left"},
	{"Output Left", "From MIC1LP", "MIC1LP"},
	{"Output Left", "From MIC1RP", "MIC1RP"},

	/* Right Output */
	{"Output Right", "From Right DAC", "DAC Right"},
	{"Output Right", "From MIC1RP", "MIC1RP"},

	/* HPL path */
	{"HP Left", "Switch", "Output Left"},
	{"HPL Driver", NULL, "HP Left"},
	{"HPL", NULL, "HPL Driver"},

	/* HPR path */
	{"HP Right", "Switch", "Output Right"},
	{"HPR Driver", NULL, "HP Right"},
	{"HPR", NULL, "HPR Driver"},
};

static const struct snd_soc_dapm_route
aic311x_audio_map[] = {
	/* SP L path */
	{"Speaker Left", "Switch", "Output Left"},
	{"SPL ClassD", NULL, "Speaker Left"},
	{"SPL", NULL, "SPL ClassD"},

	/* SP R path */
	{"Speaker Right", "Switch", "Output Right"},
	{"SPR ClassD", NULL, "Speaker Right"},
	{"SPR", NULL, "SPR ClassD"},
};

static const struct snd_soc_dapm_route
aic310x_audio_map[] = {
	/* SP L path */
	{"Speaker", "Switch", "Output Left"},
	{"SPK ClassD", NULL, "Speaker"},
	{"SPK", NULL, "SPK ClassD"},
};

static int aic31xx_add_controls(struct snd_soc_codec *codec)
{
	int ret = 0;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);

	if (aic31xx->pdata.codec_type & AIC31XX_STEREO_CLASS_D_BIT)
		ret = snd_soc_add_codec_controls(
			codec, aic311x_snd_controls,
			ARRAY_SIZE(aic311x_snd_controls));
	else
		ret = snd_soc_add_codec_controls(
			codec, aic310x_snd_controls,
			ARRAY_SIZE(aic310x_snd_controls));

	return ret;
}

static int aic31xx_add_widgets(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	if (aic31xx->pdata.codec_type & AIC31XX_STEREO_CLASS_D_BIT) {
		ret = snd_soc_dapm_new_controls(
			dapm, aic311x_dapm_widgets,
			ARRAY_SIZE(aic311x_dapm_widgets));
		if (ret)
			return ret;

		ret = snd_soc_dapm_add_routes(dapm, aic311x_audio_map,
					      ARRAY_SIZE(aic311x_audio_map));
		if (ret)
			return ret;
	} else {
		ret = snd_soc_dapm_new_controls(
			dapm, aic310x_dapm_widgets,
			ARRAY_SIZE(aic310x_dapm_widgets));
		if (ret)
			return ret;

		ret = snd_soc_dapm_add_routes(dapm, aic310x_audio_map,
					      ARRAY_SIZE(aic310x_audio_map));
		if (ret)
			return ret;
	}

	return 0;
}

static int aic31xx_setup_pll(struct snd_soc_codec *codec,
			     struct snd_pcm_hw_params *params)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	int bclk_score = snd_soc_params_to_frame_size(params);
	int mclk_p = aic31xx->sysclk / aic31xx->p_div;
	int bclk_n = 0;
	int match = -1;
	int i;

	/* Use PLL as CODEC_CLKIN and DAC_CLK as BDIV_CLKIN */
	snd_soc_update_bits(codec, AIC31XX_CLKMUX,
			    AIC31XX_CODEC_CLKIN_MASK, AIC31XX_CODEC_CLKIN_PLL);
	snd_soc_update_bits(codec, AIC31XX_IFACE2,
			    AIC31XX_BDIVCLK_MASK, AIC31XX_DAC2BCLK);

	for (i = 0; i < ARRAY_SIZE(aic31xx_divs); i++) {
		if (aic31xx_divs[i].rate == params_rate(params) &&
		    aic31xx_divs[i].mclk_p == mclk_p) {
			int s =	(aic31xx_divs[i].dosr * aic31xx_divs[i].mdac) %
				snd_soc_params_to_frame_size(params);
			int bn = (aic31xx_divs[i].dosr * aic31xx_divs[i].mdac) /
				snd_soc_params_to_frame_size(params);
			if (s < bclk_score && bn > 0) {
				match = i;
				bclk_n = bn;
				bclk_score = s;
			}
		}
	}

	if (match == -1) {
		dev_err(codec->dev,
			"%s: Sample rate (%u) and format not supported\n",
			__func__, params_rate(params));
		/* See bellow for details how fix this. */
		return -EINVAL;
	}
	if (bclk_score != 0) {
		dev_warn(codec->dev, "Can not produce exact bitclock");
		/* This is fine if using dsp format, but if using i2s
		   there may be trouble. To fix the issue edit the
		   aic31xx_divs table for your mclk and sample
		   rate. Details can be found from:
		   http://www.ti.com/lit/ds/symlink/tlv320aic3100.pdf
		   Section: 5.6 CLOCK Generation and PLL
		*/
	}
	i = match;

	/* PLL configuration */
	snd_soc_update_bits(codec, AIC31XX_PLLPR, AIC31XX_PLL_MASK,
			    (aic31xx->p_div << 4) | 0x01);
	snd_soc_write(codec, AIC31XX_PLLJ, aic31xx_divs[i].pll_j);

	snd_soc_write(codec, AIC31XX_PLLDMSB,
		      aic31xx_divs[i].pll_d >> 8);
	snd_soc_write(codec, AIC31XX_PLLDLSB,
		      aic31xx_divs[i].pll_d & 0xff);

	/* DAC dividers configuration */
	snd_soc_update_bits(codec, AIC31XX_NDAC, AIC31XX_PLL_MASK,
			    aic31xx_divs[i].ndac);
	snd_soc_update_bits(codec, AIC31XX_MDAC, AIC31XX_PLL_MASK,
			    aic31xx_divs[i].mdac);

	snd_soc_write(codec, AIC31XX_DOSRMSB, aic31xx_divs[i].dosr >> 8);
	snd_soc_write(codec, AIC31XX_DOSRLSB, aic31xx_divs[i].dosr & 0xff);

	/* ADC dividers configuration. Write reset value 1 if not used. */
	snd_soc_update_bits(codec, AIC31XX_NADC, AIC31XX_PLL_MASK,
			    aic31xx_divs[i].nadc ? aic31xx_divs[i].nadc : 1);
	snd_soc_update_bits(codec, AIC31XX_MADC, AIC31XX_PLL_MASK,
			    aic31xx_divs[i].madc ? aic31xx_divs[i].madc : 1);

	snd_soc_write(codec, AIC31XX_AOSR, aic31xx_divs[i].aosr);

	/* Bit clock divider configuration. */
	snd_soc_update_bits(codec, AIC31XX_BCLKN,
			    AIC31XX_PLL_MASK, bclk_n);

	aic31xx->rate_div_line = i;

	dev_dbg(codec->dev,
		"pll %d.%04d/%d dosr %d n %d m %d aosr %d n %d m %d bclk_n %d\n",
		aic31xx_divs[i].pll_j, aic31xx_divs[i].pll_d,
		aic31xx->p_div, aic31xx_divs[i].dosr,
		aic31xx_divs[i].ndac, aic31xx_divs[i].mdac,
		aic31xx_divs[i].aosr, aic31xx_divs[i].nadc,
		aic31xx_divs[i].madc, bclk_n);

	return 0;
}

static int aic31xx_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *tmp)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	u8 data = 0;

	dev_dbg(codec->dev, "## %s: format %d width %d rate %d\n",
		__func__, params_format(params), params_width(params),
		params_rate(params));

	switch (params_width(params)) {
	case 16:
		break;
	case 20:
		data = (AIC31XX_WORD_LEN_20BITS <<
			AIC31XX_IFACE1_DATALEN_SHIFT);
		break;
	case 24:
		data = (AIC31XX_WORD_LEN_24BITS <<
			AIC31XX_IFACE1_DATALEN_SHIFT);
		break;
	case 32:
		data = (AIC31XX_WORD_LEN_32BITS <<
			AIC31XX_IFACE1_DATALEN_SHIFT);
		break;
	default:
		dev_err(codec->dev, "%s: Unsupported format %d\n",
			__func__, params_format(params));
		return -EINVAL;
	}

	snd_soc_update_bits(codec, AIC31XX_IFACE1,
			    AIC31XX_IFACE1_DATALEN_MASK,
			    data);

	return aic31xx_setup_pll(codec, params);
}

static int aic31xx_dac_mute(struct snd_soc_dai *codec_dai, int mute)
{
	struct snd_soc_codec *codec = codec_dai->codec;

	if (mute) {
		snd_soc_update_bits(codec, AIC31XX_DACMUTE,
				    AIC31XX_DACMUTE_MASK,
				    AIC31XX_DACMUTE_MASK);
	} else {
		snd_soc_update_bits(codec, AIC31XX_DACMUTE,
				    AIC31XX_DACMUTE_MASK, 0x0);
	}

	return 0;
}

static int aic31xx_set_dai_fmt(struct snd_soc_dai *codec_dai,
			       unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u8 iface_reg1 = 0;
	u8 iface_reg3 = 0;
	u8 dsp_a_val = 0;

	dev_dbg(codec->dev, "## %s: fmt = 0x%x\n", __func__, fmt);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface_reg1 |= AIC31XX_BCLK_MASTER | AIC31XX_WCLK_MASTER;
		break;
	default:
		dev_alert(codec->dev, "Invalid DAI master/slave interface\n");
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_DSP_A:
		dsp_a_val = 0x1;
	case SND_SOC_DAIFMT_DSP_B:
		/* NOTE: BCLKINV bit value 1 equas NB and 0 equals IB */
		switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
		case SND_SOC_DAIFMT_NB_NF:
			iface_reg3 |= AIC31XX_BCLKINV_MASK;
			break;
		case SND_SOC_DAIFMT_IB_NF:
			break;
		default:
			return -EINVAL;
		}
		iface_reg1 |= (AIC31XX_DSP_MODE <<
			       AIC31XX_IFACE1_DATATYPE_SHIFT);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		iface_reg1 |= (AIC31XX_RIGHT_JUSTIFIED_MODE <<
			       AIC31XX_IFACE1_DATATYPE_SHIFT);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface_reg1 |= (AIC31XX_LEFT_JUSTIFIED_MODE <<
			       AIC31XX_IFACE1_DATATYPE_SHIFT);
		break;
	default:
		dev_err(codec->dev, "Invalid DAI interface format\n");
		return -EINVAL;
	}

	snd_soc_update_bits(codec, AIC31XX_IFACE1,
			    AIC31XX_IFACE1_DATATYPE_MASK |
			    AIC31XX_IFACE1_MASTER_MASK,
			    iface_reg1);
	snd_soc_update_bits(codec, AIC31XX_DATA_OFFSET,
			    AIC31XX_DATA_OFFSET_MASK,
			    dsp_a_val);
	snd_soc_update_bits(codec, AIC31XX_IFACE2,
			    AIC31XX_BCLKINV_MASK,
			    iface_reg3);

	return 0;
}

static int aic31xx_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				  int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	int i;

	dev_dbg(codec->dev, "## %s: clk_id = %d, freq = %d, dir = %d\n",
		__func__, clk_id, freq, dir);

	for (i = 1; freq/i > 20000000 && i < 8; i++)
		;
	if (freq/i > 20000000) {
		dev_err(aic31xx->dev, "%s: Too high mclk frequency %u\n",
			__func__, freq);
			return -EINVAL;
	}
	aic31xx->p_div = i;

	for (i = 0; aic31xx_divs[i].mclk_p != freq/aic31xx->p_div; i++) {
		if (i == ARRAY_SIZE(aic31xx_divs)) {
			dev_err(aic31xx->dev, "%s: Unsupported frequency %d\n",
				__func__, freq);
			return -EINVAL;
		}
	}

	/* set clock on MCLK, BCLK, or GPIO1 as PLL input */
	snd_soc_update_bits(codec, AIC31XX_CLKMUX, AIC31XX_PLL_CLKIN_MASK,
			    clk_id << AIC31XX_PLL_CLKIN_SHIFT);

	aic31xx->sysclk = freq;
	return 0;
}

static int aic31xx_regulator_event(struct notifier_block *nb,
				   unsigned long event, void *data)
{
	struct aic31xx_disable_nb *disable_nb =
		container_of(nb, struct aic31xx_disable_nb, nb);
	struct aic31xx_priv *aic31xx = disable_nb->aic31xx;

	if (event & REGULATOR_EVENT_DISABLE) {
		/*
		 * Put codec to reset and as at least one of the
		 * supplies was disabled.
		 */
		if (gpio_is_valid(aic31xx->pdata.gpio_reset))
			gpio_set_value(aic31xx->pdata.gpio_reset, 0);

		regcache_mark_dirty(aic31xx->regmap);
		dev_dbg(aic31xx->dev, "## %s: DISABLE received\n", __func__);
	}

	return 0;
}

static void aic31xx_clk_on(struct snd_soc_codec *codec)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	u8 mask = AIC31XX_PM_MASK;
	u8 on = AIC31XX_PM_MASK;

	dev_dbg(codec->dev, "codec clock -> on (rate %d)\n",
		aic31xx_divs[aic31xx->rate_div_line].rate);
	snd_soc_update_bits(codec, AIC31XX_PLLPR, mask, on);
	mdelay(10);
	snd_soc_update_bits(codec, AIC31XX_NDAC, mask, on);
	snd_soc_update_bits(codec, AIC31XX_MDAC, mask, on);
	if (aic31xx_divs[aic31xx->rate_div_line].nadc)
		snd_soc_update_bits(codec, AIC31XX_NADC, mask, on);
	if (aic31xx_divs[aic31xx->rate_div_line].madc)
		snd_soc_update_bits(codec, AIC31XX_MADC, mask, on);
	snd_soc_update_bits(codec, AIC31XX_BCLKN, mask, on);
}

static void aic31xx_clk_off(struct snd_soc_codec *codec)
{
	u8 mask = AIC31XX_PM_MASK;
	u8 off = 0;

	dev_dbg(codec->dev, "codec clock -> off\n");
	snd_soc_update_bits(codec, AIC31XX_BCLKN, mask, off);
	snd_soc_update_bits(codec, AIC31XX_MADC, mask, off);
	snd_soc_update_bits(codec, AIC31XX_NADC, mask, off);
	snd_soc_update_bits(codec, AIC31XX_MDAC, mask, off);
	snd_soc_update_bits(codec, AIC31XX_NDAC, mask, off);
	snd_soc_update_bits(codec, AIC31XX_PLLPR, mask, off);
}

static int aic31xx_power_on(struct snd_soc_codec *codec)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	ret = regulator_bulk_enable(ARRAY_SIZE(aic31xx->supplies),
				    aic31xx->supplies);
	if (ret)
		return ret;

	if (gpio_is_valid(aic31xx->pdata.gpio_reset)) {
		gpio_set_value(aic31xx->pdata.gpio_reset, 1);
		udelay(100);
	}
	regcache_cache_only(aic31xx->regmap, false);
	ret = regcache_sync(aic31xx->regmap);
	if (ret != 0) {
		dev_err(codec->dev,
			"Failed to restore cache: %d\n", ret);
		regcache_cache_only(aic31xx->regmap, true);
		regulator_bulk_disable(ARRAY_SIZE(aic31xx->supplies),
				       aic31xx->supplies);
		return ret;
	}
	return 0;
}

static int aic31xx_power_off(struct snd_soc_codec *codec)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	regcache_cache_only(aic31xx->regmap, true);
	ret = regulator_bulk_disable(ARRAY_SIZE(aic31xx->supplies),
				     aic31xx->supplies);

	return ret;
}

static int aic31xx_set_bias_level(struct snd_soc_codec *codec,
				  enum snd_soc_bias_level level)
{
	dev_dbg(codec->dev, "## %s: %d -> %d\n", __func__,
		codec->dapm.bias_level, level);

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		if (codec->dapm.bias_level == SND_SOC_BIAS_STANDBY)
			aic31xx_clk_on(codec);
		break;
	case SND_SOC_BIAS_STANDBY:
		switch (codec->dapm.bias_level) {
		case SND_SOC_BIAS_OFF:
			aic31xx_power_on(codec);
			break;
		case SND_SOC_BIAS_PREPARE:
			aic31xx_clk_off(codec);
			break;
		default:
			BUG();
		}
		break;
	case SND_SOC_BIAS_OFF:
		if (codec->dapm.bias_level == SND_SOC_BIAS_STANDBY)
			aic31xx_power_off(codec);
		break;
	}
	codec->dapm.bias_level = level;

	return 0;
}

static int aic31xx_suspend(struct snd_soc_codec *codec)
{
	aic31xx_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int aic31xx_resume(struct snd_soc_codec *codec)
{
	aic31xx_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	return 0;
}

static int aic31xx_codec_probe(struct snd_soc_codec *codec)
{
	int ret = 0;
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	int i;

	dev_dbg(aic31xx->dev, "## %s\n", __func__);

	aic31xx = snd_soc_codec_get_drvdata(codec);

	aic31xx->codec = codec;

	ret = snd_soc_codec_set_cache_io(codec, dev_get_regmap(codec->dev->parent, NULL));
	if (ret != 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(aic31xx->supplies); i++) {
		aic31xx->disable_nb[i].nb.notifier_call =
			aic31xx_regulator_event;
		aic31xx->disable_nb[i].aic31xx = aic31xx;
		ret = regulator_register_notifier(aic31xx->supplies[i].consumer,
						  &aic31xx->disable_nb[i].nb);
		if (ret) {
			dev_err(codec->dev,
				"Failed to request regulator notifier: %d\n",
				ret);
			return ret;
		}
	}

	regcache_cache_only(aic31xx->regmap, true);
	regcache_mark_dirty(aic31xx->regmap);

	ret = aic31xx_add_controls(codec);
	if (ret)
		return ret;

	ret = aic31xx_add_widgets(codec);

	return ret;
}

static int aic31xx_codec_remove(struct snd_soc_codec *codec)
{
	struct aic31xx_priv *aic31xx = snd_soc_codec_get_drvdata(codec);
	int i;
	/* power down chip */
	aic31xx_set_bias_level(codec, SND_SOC_BIAS_OFF);

	for (i = 0; i < ARRAY_SIZE(aic31xx->supplies); i++)
		regulator_unregister_notifier(aic31xx->supplies[i].consumer,
					      &aic31xx->disable_nb[i].nb);

	return 0;
}

static struct snd_soc_codec_driver soc_codec_driver_aic31xx = {
	.probe			= aic31xx_codec_probe,
	.remove			= aic31xx_codec_remove,
	.suspend		= aic31xx_suspend,
	.resume			= aic31xx_resume,
	.set_bias_level		= aic31xx_set_bias_level,
	.controls		= aic31xx_snd_controls,
	.num_controls		= ARRAY_SIZE(aic31xx_snd_controls),
	.dapm_widgets		= aic31xx_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(aic31xx_dapm_widgets),
	.dapm_routes		= aic31xx_audio_map,
	.num_dapm_routes	= ARRAY_SIZE(aic31xx_audio_map),
};

static struct snd_soc_dai_ops aic31xx_dai_ops = {
	.hw_params	= aic31xx_hw_params,
	.set_sysclk	= aic31xx_set_dai_sysclk,
	.set_fmt	= aic31xx_set_dai_fmt,
	.digital_mute	= aic31xx_dac_mute,
};

static struct snd_soc_dai_driver aic31xx_dai_driver[] = {
	{
		.name = "tlv320aic31xx-hifi",
		.playback = {
			.stream_name	 = "Playback",
			.channels_min	 = 1,
			.channels_max	 = 2,
			.rates		 = AIC31XX_RATES,
			.formats	 = AIC31XX_FORMATS,
		},
		.capture = {
			.stream_name	 = "Capture",
			.channels_min	 = 1,
			.channels_max	 = 2,
			.rates		 = AIC31XX_RATES,
			.formats	 = AIC31XX_FORMATS,
		},
		.ops = &aic31xx_dai_ops,
		.symmetric_rates = 1,
	}
};

#if defined(CONFIG_OF)
static const struct of_device_id tlv320aic31xx_of_match[] = {
	{ .compatible = "ti,tlv320aic310x" },
	{ .compatible = "ti,tlv320aic311x" },
	{ .compatible = "ti,tlv320aic3100" },
	{ .compatible = "ti,tlv320aic3110" },
	{ .compatible = "ti,tlv320aic3120" },
	{ .compatible = "ti,tlv320aic3111" },
	{},
};
MODULE_DEVICE_TABLE(of, tlv320aic31xx_of_match);

static void aic31xx_pdata_from_of(struct aic31xx_priv *aic31xx)
{
	struct device_node *np = aic31xx->dev->of_node;
	unsigned int value = MICBIAS_2_0V;
	int ret;

	of_property_read_u32(np, "ai31xx-micbias-vg", &value);
	switch (value) {
	case MICBIAS_2_0V:
	case MICBIAS_2_5V:
	case MICBIAS_AVDDV:
		aic31xx->pdata.micbias_vg = value;
		break;
	default:
		dev_err(aic31xx->dev,
			"Bad ai31xx-micbias-vg value %d DT\n",
			value);
		aic31xx->pdata.micbias_vg = MICBIAS_2_0V;
	}

	ret = of_get_named_gpio(np, "gpio-reset", 0);
	if (ret > 0)
		aic31xx->pdata.gpio_reset = ret;
}
#else /* CONFIG_OF */
static void aic31xx_pdata_from_of(struct aic31xx_priv *aic31xx)
{
}
#endif /* CONFIG_OF */

static int aic31xx_device_init(struct aic31xx_priv *aic31xx)
{
	int ret, i;

	dev_set_drvdata(aic31xx->dev, aic31xx);

	if (dev_get_platdata(aic31xx->dev))
		memcpy(&aic31xx->pdata, dev_get_platdata(aic31xx->dev),
		       sizeof(aic31xx->pdata));
	else if (aic31xx->dev->of_node)
		aic31xx_pdata_from_of(aic31xx);

	if (aic31xx->pdata.gpio_reset) {
		ret = devm_gpio_request_one(aic31xx->dev,
					    aic31xx->pdata.gpio_reset,
					    GPIOF_OUT_INIT_HIGH,
					    "aic31xx-reset-pin");
		if (ret < 0) {
			dev_err(aic31xx->dev, "not able to acquire gpio\n");
			return ret;
		}
	}

	for (i = 0; i < ARRAY_SIZE(aic31xx->supplies); i++)
		aic31xx->supplies[i].supply = aic31xx_supply_names[i];

	ret = devm_regulator_bulk_get(aic31xx->dev,
				      ARRAY_SIZE(aic31xx->supplies),
				      aic31xx->supplies);
	if (ret != 0)
		dev_err(aic31xx->dev, "Failed to request supplies: %d\n", ret);

	return ret;
}

static int aic31xx_i2c_probe(struct i2c_client *i2c,
			     const struct i2c_device_id *id)
{
	struct aic31xx_priv *aic31xx;
	int ret;
	const struct regmap_config *regmap_config;

	dev_dbg(&i2c->dev, "## %s: %s codec_type = %d\n", __func__,
		id->name, (int) id->driver_data);

	regmap_config = &aic31xx_i2c_regmap;

	aic31xx = devm_kzalloc(&i2c->dev, sizeof(*aic31xx), GFP_KERNEL);
	if (aic31xx == NULL)
		return -ENOMEM;

	aic31xx->regmap = devm_regmap_init_i2c(i2c, regmap_config);

	if (IS_ERR(aic31xx->regmap)) {
		ret = PTR_ERR(aic31xx->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}
	aic31xx->dev = &i2c->dev;

	aic31xx->pdata.codec_type = id->driver_data;

	ret = aic31xx_device_init(aic31xx);
	if (ret)
		return ret;

	ret = snd_soc_register_codec(&i2c->dev, &soc_codec_driver_aic31xx,
				     aic31xx_dai_driver,
				     ARRAY_SIZE(aic31xx_dai_driver));

	return ret;
}

static int aic31xx_i2c_remove(struct i2c_client *i2c)
{
	struct aic31xx_priv *aic31xx = dev_get_drvdata(&i2c->dev);

	kfree(aic31xx);
	return 0;
}

static const struct i2c_device_id aic31xx_i2c_id[] = {
	{ "tlv320aic310x", AIC3100 },
	{ "tlv320aic311x", AIC3110 },
	{ "tlv320aic3100", AIC3100 },
	{ "tlv320aic3110", AIC3110 },
	{ "tlv320aic3120", AIC3120 },
	{ "tlv320aic3111", AIC3111 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aic31xx_i2c_id);

static struct i2c_driver aic31xx_i2c_driver = {
	.driver = {
		.name	= "tlv320aic31xx-codec",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(tlv320aic31xx_of_match),
	},
	.probe		= aic31xx_i2c_probe,
	.remove		= (aic31xx_i2c_remove),
	.id_table	= aic31xx_i2c_id,
};

module_i2c_driver(aic31xx_i2c_driver);

MODULE_DESCRIPTION("ASoC TLV320AIC3111 codec driver");
MODULE_AUTHOR("Jyri Sarha");
MODULE_LICENSE("GPL");
