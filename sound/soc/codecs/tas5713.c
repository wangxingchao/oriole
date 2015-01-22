/*
 * ASoC Driver for TAS5713
 *
 * Author:	Sebastian Eickhoff <basti.eickhoff@googlemail.com>
 *		Copyright 2014
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <sound/soc-of-simple.h>

#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#include "tas5713.h"

static struct i2c_client *i2c;

struct tas5713_priv {
	struct regmap *regmap;
	int mclk_div;
	struct snd_soc_codec codec;
};

static struct tas5713_priv *priv_data;
static struct snd_soc_codec *tas_codec;

/*
 *    _   _    ___   _      ___         _           _    
 *   /_\ | |  / __| /_\    / __|___ _ _| |_ _ _ ___| |___
 *  / _ \| |__\__ \/ _ \  | (__/ _ \ ' \  _| '_/ _ \ (_-<
 * /_/ \_\____|___/_/ \_\  \___\___/_||_\__|_| \___/_/__/
 *
 */                                                      

static const DECLARE_TLV_DB_SCALE(tas5713_vol_tlv, -10000, 50, 1);

static const struct snd_kcontrol_new tas5713_snd_controls[] = {
	SOC_SINGLE_TLV  ("Master"    , TAS5713_VOL_MASTER, 0, 248, 1, tas5713_vol_tlv),
	SOC_DOUBLE_R_TLV("Channels"  , TAS5713_VOL_CH1, TAS5713_VOL_CH2, 0, 248, 1, tas5713_vol_tlv)
};

/*
 *  __  __         _    _            ___      _             
 * |  \/  |__ _ __| |_ (_)_ _  ___  |   \ _ _(_)_ _____ _ _ 
 * | |\/| / _` / _| ' \| | ' \/ -_) | |) | '_| \ V / -_) '_|
 * |_|  |_\__,_\__|_||_|_|_||_\___| |___/|_| |_|\_/\___|_|  
 *                                                           
 */

static unsigned int tas5713_hw_read(struct snd_soc_codec *codec,
				     unsigned int reg)
{
	struct tas5713_priv *tas5713 = snd_soc_codec_get_drvdata(codec);
	struct i2c_client *client = codec->control_data;
	int i2c_ret;
	u16 value;
	u8 buf0[2], buf1[2];
	u16 addr = client->addr;
	u16 flags = client->flags;
	struct i2c_msg msg[2] = {
		{addr, flags, 2, buf0},
		{addr, flags | I2C_M_RD, 2, buf1},
	};

	//tas5713->need_clk_for_access = 1;
	//tas5713_clock_gating(codec, 1);
	buf0[0] = (reg & 0xff00) >> 8;
	buf0[1] = reg & 0xff;
	i2c_ret = i2c_transfer(client->adapter, msg, 2);
	//tas5713->need_clk_for_access = 0;
	//tas5713_clock_gating(codec, 0);
	if (i2c_ret < 0) {
		pr_err("%s: read reg error : Reg 0x%02x\n", __func__, reg);
		return 0;
	}

	value = buf1[0] << 8 | buf1[1];

	pr_debug("r r:%02x,v:%04x\n", reg, value);
	return value;
}

static unsigned int tas5713_read(struct snd_soc_codec *codec, unsigned int reg)
{
	return tas5713_hw_read(codec, reg);
}

static inline void tas5713_write_reg_cache(struct snd_soc_codec *codec,
					    u16 reg, unsigned int value)
{
#if 0
	u16 *cache = codec->reg_cache;
	unsigned int offset = reg >> 1;
	if (offset < ARRAY_SIZE(tas5713_regs))
		cache[offset] = value;
#endif
}

static int tas5713_write(struct snd_soc_codec *codec, unsigned int reg,
			  unsigned int value)
{
	struct tas5713_priv *tas5713 = snd_soc_codec_get_drvdata(codec);
	struct i2c_client *client = codec->control_data;
	u16 addr = client->addr;
	u16 flags = client->flags;
	u8 buf[4];
	int i2c_ret;
	struct i2c_msg msg = { addr, flags, 4, buf };

	//tas5713->need_clk_for_access = 1;
	//tas5713_clock_gating(codec, 1);
	tas5713_write_reg_cache(codec, reg, value);
	pr_debug("w r:%02x,v:%04x\n", reg, value);
	buf[0] = (reg & 0xff00) >> 8;
	buf[1] = reg & 0xff;
	buf[2] = (value & 0xff00) >> 8;
	buf[3] = value & 0xff;

	i2c_ret = i2c_transfer(client->adapter, &msg, 1);
	//tas5713->need_clk_for_access = 0;
	//tas5713_clock_gating(codec, 0);
	if (i2c_ret < 0) {
		pr_err("%s: write reg error : Reg 0x%02x = 0x%04x\n",
		       __func__, reg, value);
		return -EIO;
	}

	return i2c_ret;
}

static int tas5713_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	u16 blen = 0x00;
	
	struct snd_soc_codec *codec;
	codec = dai->codec;
	//priv_data->codec = dai->codec;
	
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		blen = 0x03;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		blen = 0x1;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		blen = 0x04;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		blen = 0x05;
		break;
	default:
		dev_err(dai->dev, "Unsupported word length: %u\n",
			params_format(params));
		return -EINVAL;
	}

	// set word length
	snd_soc_update_bits(codec, TAS5713_SERIAL_DATA_INTERFACE, 0x7, blen);

	return 0;
}

static int tas5713_mute_stream(struct snd_soc_dai *dai, int mute)
{
	unsigned int val = 0;
	
	struct tas5713_priv *tas5713;
	struct snd_soc_codec *codec = dai->codec;
	tas5713 = snd_soc_codec_get_drvdata(codec);
	
	if (mute) {
		val = TAS5713_SOFT_MUTE_ALL;
	}

	//TODO: add correct operations here
	//return regmap_write(tas5713->regmap, TAS5713_SOFT_MUTE, val);
	return 0;
}

static const struct snd_soc_dai_ops tas5713_dai_ops = {
	.hw_params 		= tas5713_hw_params,
	.digital_mute		= tas5713_mute_stream,
};

static struct snd_soc_dai tas5713_dai = {
	.name		= "tas5713",
	.playback 	= {
		.stream_name	= "Playback",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		    = SNDRV_PCM_RATE_8000_48000,
		.formats	    = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE ),
	},
	.ops        = &tas5713_dai_ops,
};

/*
 *   ___         _          ___      _             
 *  / __|___  __| |___ __  |   \ _ _(_)_ _____ _ _ 
 * | (__/ _ \/ _` / -_) _| | |) | '_| \ V / -_) '_|
 *  \___\___/\__,_\___\__| |___/|_| |_|\_/\___|_|  
 *                                                 
 */

static int tas5713_remove(struct platform_device *pdev)
{
	struct tas5713_priv *tas5713;
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);

	tas5713 = socdev->codec_data;

	return 0;
}

static int tas5713_probe(struct platform_device *pdev)
{
	struct tas5713_priv *tas5713;
	int i, ret;
	struct snd_soc_codec *codec;
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	dev_info(&pdev->dev, "Probing TAS5713 SoC CODEC driver\n");
	dev_dbg(&pdev->dev, "socdev=%p\n", socdev);
	dev_dbg(&pdev->dev, "codec_data=%p\n", socdev->codec_data);

	/* Fetch the relevant tas private data here (it's already been
	 * stored in the .codec pointer) */
	tas5713 = socdev->codec_data;
	if (tas5713 == NULL) {
		dev_err(&pdev->dev, "tas: missing codec pointer\n");
		return -ENODEV;
	}

	codec = &tas5713->codec;
	socdev->card->codec = codec;

	dev_dbg(&pdev->dev, "Registering PCMs, dev=%p, socdev->dev=%p\n",
		&pdev->dev, socdev->dev);
	
	i2c = container_of(codec->dev, struct i2c_client, dev);
	
	//codec->control_data = client;

	ret = snd_soc_codec_set_cache_io(codec, 8, 8, SND_SOC_I2C);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set cache i/o: %d\n", ret);
		return ret;
	}
	
	// Reset error
	ret = snd_soc_write(codec, TAS5713_ERROR_STATUS, 0x00);
	
	// Trim oscillator
    ret = snd_soc_write(codec, TAS5713_OSC_TRIM, 0x00);
	msleep(1000);
	
	// Reset error
	ret = snd_soc_write(codec, TAS5713_ERROR_STATUS, 0x00);
	
	// Clock mode: 44/48kHz, MCLK=64xfs
	ret = snd_soc_write(codec, TAS5713_CLOCK_CTRL, 0x60);
	
	// I2S 24bit
	ret = snd_soc_write(codec, TAS5713_SERIAL_DATA_INTERFACE, 0x05);
	
	// Unmute
	ret = snd_soc_write(codec, TAS5713_SYSTEM_CTRL2, 0x00);
	ret = snd_soc_write(codec, TAS5713_SOFT_MUTE, 0x00);
	
	// Set volume to 0db
	ret = snd_soc_write(codec, TAS5713_VOL_MASTER, 0x00);
	
	// Now start programming the default initialization sequence
	for (i = 0; i < ARRAY_SIZE(tas5713_init_sequence); ++i) {
		ret = i2c_master_send(i2c,
				     tas5713_init_sequence[i].data,
				     tas5713_init_sequence[i].size);

		if (ret < 0) {
			printk(KERN_INFO "TAS5713 CODEC PROBE: InitSeq returns: %d\n", ret);
		}
	}
	
	// Unmute
	ret = snd_soc_write(codec, TAS5713_SYSTEM_CTRL2, 0x00);

	snd_soc_add_controls(codec, tas5713_snd_controls,
			     ARRAY_SIZE(tas5713_snd_controls));

	return 0;
}

struct snd_soc_codec_device soc_codec_dev_tas5713 = {
	.probe = tas5713_probe,
	.remove = tas5713_remove,
//	.controls = tas5713_snd_controls,
//	.num_controls = ARRAY_SIZE(tas5713_snd_controls),
};
EXPORT_SYMBOL_GPL(soc_codec_dev_tas5713);

/*
 *   ___ ___ ___   ___      _             
 *  |_ _|_  ) __| |   \ _ _(_)_ _____ _ _ 
 *   | | / / (__  | |) | '_| \ V / -_) '_|
 *  |___/___\___| |___/|_| |_|\_/\___|_|  
 *                                        
 */

static const struct reg_default tas5713_reg_defaults[] = {
	{ 0x07 ,0x80 },     // R7  - VOL_MASTER    - -40dB
	{ 0x08 ,  30 },     // R8  - VOL_CH1	   -   0dB
	{ 0x09 ,  30 },     // R9  - VOL_CH2       -   0dB
	{ 0x0A ,0x80 },     // R10 - VOL_HEADPHONE - -40dB
};

static bool tas5713_reg_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
		case TAS5713_DEVICE_ID:
		case TAS5713_ERROR_STATUS:
			return true;
	default:
			return false;
	}
}

static const struct of_device_id tas5713_of_match[] = {
	{ .compatible = "ti,tas5713", },
	{ }
};
MODULE_DEVICE_TABLE(of, tas5713_of_match);

static struct regmap_config tas5713_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = TAS5713_MAX_REGISTER,
	.volatile_reg = tas5713_reg_volatile,

	.cache_type = REGCACHE_RBTREE,
	.reg_defaults = tas5713_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(tas5713_reg_defaults),
};

static int tas5713_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	int ret;
	int val;

	priv_data = devm_kzalloc(&i2c->dev, sizeof *priv_data, GFP_KERNEL);
	if (!priv_data)
		return -ENOMEM;

#if 0
	priv_data->regmap = devm_regmap_init_i2c(i2c, &tas5713_regmap_config);
	if (IS_ERR(priv_data->regmap)) {
		ret = PTR_ERR(priv_data->regmap);
		return ret;
	}
#endif

	snd_soc_codec_set_drvdata(&priv_data->codec, priv_data);

	mutex_init(&priv_data->codec.mutex);
	INIT_LIST_HEAD(&priv_data->codec.dapm_widgets);
	INIT_LIST_HEAD(&priv_data->codec.dapm_paths);

	i2c_set_clientdata(i2c, priv_data);

	//TODO: Do we need regulator for voltage setting?

	priv_data->codec.control_data = i2c;

	//TODO: check i2c chip id or else, ensure the i2c hardware works
	msleep(2);

	priv_data->codec.dev = &i2c->dev;
	priv_data->codec.name = "tas5713";
	priv_data->codec.owner = THIS_MODULE;
	priv_data->codec.dai = &tas5713_dai;
	priv_data->codec.num_dai = 1;
	priv_data->codec.read = tas5713_read;
	priv_data->codec.write = tas5713_write;

	//should add cache operations here
	//

	tas5713_dai.dev = priv_data->codec.dev;
	tas_codec = &priv_data->codec;

	val = tas5713_read(tas_codec, TAS5713_DEVICE_ID);
	printk("tas5713 i2c read codec id=%x\n",val);
	//check return device id, if fail, return
	//if (val != correct_chip_id)
	//	return;
	//
	val = tas5713_read(tas_codec, 0x07);
	printk("tas5713 i2c read codec volume=%x\n",val);

	val = tas5713_read(tas_codec, 0x08);
	printk("tas5713 i2c read code volume=%x\n",val);

	ret = snd_soc_register_codec(tas_codec);
	if (ret != 0) {
		dev_err(tas_codec->dev, "Failed to register codec: %d\n", ret);
		kfree(priv_data);
		return ret;
	}

	ret = snd_soc_register_dai(&tas5713_dai);
	if (ret != 0) {
		dev_err(&i2c->dev, "Failed to register DAI: %d\n", ret);
		snd_soc_unregister_codec(tas_codec);
		kfree(&tas5713_dai);
		return ret;
	}

	
	printk("tas5713 codec driver register finished\n");
	return ret;
}

static int tas5713_i2c_remove(struct i2c_client *i2c)
{
	snd_soc_unregister_codec(tas_codec);
	i2c_set_clientdata(i2c, NULL);

	kfree(priv_data);
	
	return 0;
}

static const struct i2c_device_id tas5713_i2c_id[] = {
	{ "tas5713", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, tas5713_i2c_id);

static struct i2c_driver tas5713_i2c_driver = {
	.driver = {
		.name = "tas5713",
		.owner = THIS_MODULE,
	},
	.probe = tas5713_i2c_probe,
	.remove = tas5713_i2c_remove,
	.id_table = tas5713_i2c_id
};

static int __init tas5713_modinit(void)
{
	int ret = 0;

	printk("Initialize tas5713 driver\n");
	ret = i2c_add_driver(&tas5713_i2c_driver);
	if (ret) {
		printk(KERN_ERR "Failed to register tas5713 I2C driver: %d\n",
		       ret);
	}
	
	return ret;
}
module_init(tas5713_modinit);

static void __exit tas5713_exit(void)
{
	i2c_del_driver(&tas5713_i2c_driver);
}
module_exit(tas5713_exit);

MODULE_AUTHOR("Sebastian Eickhoff <basti.eickhoff@googlemail.com>");
MODULE_DESCRIPTION("ASoC driver for TAS5713");
MODULE_LICENSE("GPL v2");
