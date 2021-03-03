/*
 * Copyright (C) 2020 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT adi_adau1x61

#include "adau1x61.h"
#include <audio/codec.h>
#include <device.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include <errno.h>
#include <sys/util.h>


#define LOG_LEVEL CONFIG_AUDIO_CODEC_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(adau1x61);

#define CODEC_OUTPUT_VOLUME_MAX_DB		6
#define CODEC_OUTPUT_VOLUME_MIN_DB		-57

struct adau1x61_driver_config {
	const struct device 	*i2c_device;
	const char		*i2c_dev_name;
	uint8_t			i2c_address;
	const char		*adau_model;
};

struct adau1x61_driver_data {
	bool			has_dsp;
};

static struct adau1x61_driver_config adau1x61_device_config = {
	.i2c_device	= NULL,
	.i2c_dev_name	= DT_INST_BUS_LABEL(0),
	.i2c_address	= DT_INST_REG_ADDR(0),
	.adau_model	= DT_PROP(DT_INST(0, adi_adau1x61), label),
};

static struct adau1x61_driver_data adau1x61_device_data;

#define DEV_CFG(dev) \
	((struct adau1x61_driver_config *const)(dev)->config)
#define DEV_DATA(dev) \
	((struct adau1x61_driver_data *const)(dev)->data)

static int adau1x61_write_reg(const struct device *dev, uint16_t reg_addr,
			       uint8_t reg_val);
static int adau1x61_read_reg(const struct device *dev, uint16_t reg_addr,
			      uint8_t *reg_val);
static int adau1x61_update_reg(const struct device *dev, uint16_t reg_addr,
			       uint8_t mask, uint8_t reg_val);
static int adau1x61_configure_dai(const struct device *dev,
				  audio_dai_cfg_t *cfg);
static int adau1x61_configure_clocks(const struct device *dev,
				  struct audio_codec_cfg *cfg);
static void adau1x61_configure_output(const struct device *dev);
static int adau1x61_set_output_volume(const struct device *dev,
				      audio_channel_t channel, int vol);
static int adau1x61_set_output_mute(const struct device *dev,
				     audio_channel_t channel, bool mute);

#if (LOG_LEVEL >= LOG_LEVEL_DEBUG)
static void adau1x61_read_all_regs(const struct device *dev);
#define ADAU1X61_DUMP_REGS(dev)	adau1x61_read_all_regs((dev))
#else
#define ADAU1X61_DUMP_REGS(dev)
#endif

static int adau1x61_initialize(const struct device *dev)
{
	struct adau1x61_driver_config *const dev_cfg = DEV_CFG(dev);
	struct adau1x61_driver_data *const dev_data = DEV_DATA(dev);

	/* bind I2C */
	dev_cfg->i2c_device = device_get_binding(dev_cfg->i2c_dev_name);

	if (dev_cfg->i2c_device == NULL) {
		LOG_ERR("I2C device binding error");
		return -ENXIO;
	}

	if (strcmp(dev_cfg->adau_model, "ADAU1361") == 0) {
		dev_data->has_dsp = false;
	} else if ((strcmp(dev_cfg->adau_model, "ADAU1761") == 0) ||
		   (strcmp(dev_cfg->adau_model, "ADAU1961") == 0)) {
		dev_data->has_dsp = true;
	} else {
		LOG_ERR("ADAU model: %s label not valid!", dev_cfg->adau_model);
		return -EINVAL;
	}
	LOG_DBG("Connected ADAU model: %s", dev_cfg->adau_model);

	return 0;
}

static int adau1x61_configure(const struct device *dev,
			   struct audio_codec_cfg *cfg)
{
	int ret;

	if (cfg->dai_type != AUDIO_DAI_TYPE_I2S) {
		LOG_ERR("dai_type must be AUDIO_DAI_TYPE_I2S");
		return -EINVAL;
	}

	ret = adau1x61_configure_dai(dev, &cfg->dai_cfg);
	if (ret)
		return -EIO;

	ret = adau1x61_configure_clocks(dev, cfg);
	if (ret)
		return -EIO;

	adau1x61_configure_output(dev);

	return 0;
}

static void adau1x61_start_output(const struct device *dev)
{
	/* powerup DAC channels */
	adau1x61_update_reg(dev, ADAU1X61_DAC_CONTROL0,
			    ADAU1X61_DAC_CONTROL0_DACEN_MASK,
			    ADAU1X61_DAC_CONTROL0_DACEN_LEFT_DAC |
			    ADAU1X61_DAC_CONTROL0_DACEN_RIGHT_DAC);

	/* unmute DAC mixers MX3 and MX4 */
	adau1x61_write_reg(dev, ADAU1X61_PLAY_MIXER_LEFT0,
			   ADAU1X61_PLAY_MIXER_LEFT0_MIXER_EN |
			   ADAU1X61_PLAY_MIXER_LEFT0_LEFT_MUTE);
	adau1x61_write_reg(dev, ADAU1X61_PLAY_MIXER_RIGHT0,
			   ADAU1X61_PLAY_MIXER_RIGHT0_MIXER_EN |
			   ADAU1X61_PLAY_MIXER_RIGHT0_RIGHT_MUTE);

	ADAU1X61_DUMP_REGS(dev);
}

static void adau1x61_stop_output(const struct device *dev)
{
#if 0
	/* mute DAC channels */
	adau1x61_write_reg(dev, VOL_CTRL_ADDR, VOL_CTRL_MUTE_DEFAULT);

	/* powerdown DAC channels */
	adau1x61_write_reg(dev, DATA_PATH_SETUP_ADDR, DAC_LR_POWERDN_DEFAULT);
#endif
}

static int adau1x61_set_property(const struct device *dev,
			      audio_property_t property,
			      audio_channel_t channel,
			      audio_property_value_t val)
{
	switch (property) {
	case AUDIO_PROPERTY_OUTPUT_VOLUME:
		return adau1x61_set_output_volume(dev, channel, val.vol);
		break;
	case AUDIO_PROPERTY_OUTPUT_MUTE:
		return adau1x61_set_output_mute(dev, channel, val.mute);
		break;
	default:
		break;
	}

	return -EINVAL;
}

static int adau1x61_apply_properties(const struct device *dev)
{
	/* nothing to do because there is nothing cached */
	return 0;
}

static int adau1x61_write_reg(const struct device *dev, uint16_t reg_addr,
			    uint8_t reg_val)
{
	struct adau1x61_driver_config *const dev_cfg = DEV_CFG(dev);
	uint8_t tx_buf[3] = {reg_addr >> 8,  reg_addr, reg_val};

	LOG_DBG("WR REG:0x%04x VAL:0x%02x", reg_addr, reg_val);

	return i2c_write(dev_cfg->i2c_device, tx_buf, 3, dev_cfg->i2c_address);
}

static int adau1x61_read_reg(const struct device *dev, uint16_t reg_addr,
			   uint8_t *reg_val)
{
	struct adau1x61_driver_config *const dev_cfg = DEV_CFG(dev);
	uint8_t tx_buf[2] = {reg_addr >> 8,  reg_addr};
	int ret;

	ret = i2c_write_read(dev_cfg->i2c_device, dev_cfg->i2c_address, &tx_buf,
			     sizeof(tx_buf), reg_val, sizeof(*reg_val));

	LOG_DBG("RD REG:0x%04x VAL:0x%02x", reg_addr, *reg_val);

	return ret;
}

static int adau1x61_update_reg(const struct device *dev, uint16_t reg_addr,
			       uint8_t mask, uint8_t reg_val)
{
	uint8_t old_val, new_val;
	int ret;

	ret = adau1x61_read_reg(dev, reg_addr, &old_val);
	if (ret)
		return ret;

	new_val = (old_val & ~mask) | (reg_val & mask);
	if (new_val == old_val) {
		return 0;
	}

	LOG_DBG("UPDATE REG:0x%04x VAL:0x%02x", reg_addr, new_val);

	return adau1x61_write_reg(dev, reg_addr, new_val);
}

static int adau1x61_configure_dai(const struct device *dev,
				  audio_dai_cfg_t *cfg)
{
	struct adau1x61_driver_data *const dev_data = DEV_DATA(dev);
	struct i2s_config i2s_cfg = cfg->i2s;
	bool lrclk_pol;
	uint8_t reg0, reg1;

	/* configure DAI format */
	switch (i2s_cfg.format & I2S_FMT_DATA_FORMAT_MASK) {
	case I2S_FMT_DATA_FORMAT_I2S:
		reg1 = ADAU1X61_SERIAL_PORT1_DELAY1;
		lrclk_pol = 0;
		break;
	case I2S_FMT_DATA_FORMAT_PCM_SHORT:
		reg0 |= ADAU1X61_SERIAL_PORT0_PULSE_MODE;
		reg1 = ADAU1X61_SERIAL_PORT1_DELAY1;
		lrclk_pol = 1;
		break;
	case I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED:
	case I2S_FMT_DATA_FORMAT_RIGHT_JUSTIFIED:
		reg1 = ADAU1X61_SERIAL_PORT1_DELAY0;
		lrclk_pol = 1;
		break;

	default:
		LOG_ERR("Unsupported I2S data format");
		return -EINVAL;
	}

	/* configure word size */
	switch (i2s_cfg.word_size) {
	case AUDIO_PCM_WIDTH_16_BITS:
		reg1 = ADAU1X61_SERIAL_PORT1_DELAY16;
		break;
	case AUDIO_PCM_WIDTH_24_BITS:
		reg1 = ADAU1X61_SERIAL_PORT1_DELAY8;
		break;
	case AUDIO_PCM_WIDTH_32_BITS:
		reg1 = ADAU1X61_SERIAL_PORT1_DELAY0;
		break;
	default:
		LOG_ERR("Unsupported PCM sample bit width %u",
				cfg->i2s.word_size);
		return -EINVAL;
	}

	/* configure CLK format */
	switch (i2s_cfg.format & I2S_FMT_CLK_FORMAT_MASK) {
	case I2S_FMT_CLK_NF_NB:
		break;
	case I2S_FMT_CLK_NF_IB:
		reg0 |= ADAU1X61_SERIAL_PORT0_BCLK_POL;
		break;
	case I2S_FMT_CLK_IF_NB:
		lrclk_pol = !lrclk_pol;
		break;
	case I2S_FMT_CLK_IF_IB:
		reg0 |= ADAU1X61_SERIAL_PORT0_BCLK_POL;
		lrclk_pol = !lrclk_pol;
		break;
	default:
		LOG_ERR("Unsupported I2S clock format");
		return -EINVAL;
	}

	if (lrclk_pol)
		reg0 |= ADAU1X61_SERIAL_PORT0_LRCLK_POL;

	adau1x61_write_reg(dev, ADAU1X61_SERIAL_PORT0, reg0);
	adau1x61_write_reg(dev, ADAU1X61_SERIAL_PORT1, reg1);

	if (dev_data->has_dsp) {
		adau1x61_write_reg(dev, ADAU1761_SERIAL_INPUT_ROUTE, 0x01);
		adau1x61_write_reg(dev, ADAU1761_SERIAL_OUTPUT_ROUTE, 0x01);
		adau1x61_write_reg(dev, ADAU1761_CLK_ENABLE0, 0x7F);
		adau1x61_write_reg(dev, ADAU1761_CLK_ENABLE1, 0x03);
	}

	return 0;
}

static int adau1x61_configure_clocks(const struct device *dev,
				     struct audio_codec_cfg *cfg)
{
	struct i2s_config *i2s_cfg;
	uint8_t reg, infreq;

	i2s_cfg = &cfg->dai_cfg.i2s;
	LOG_DBG("MCLK Rate: %u Hz - PCM Rate: %u Hz", cfg->mclk_freq,
		i2s_cfg->frame_clk_freq);

	/* configure CLK slave/master */
	if (i2s_cfg->options & I2S_OPT_BIT_CLK_SLAVE & I2S_OPT_FRAME_CLK_SLAVE)
		reg = 0;
	else
		reg = ADAU1X61_SERIAL_PORT0_MASTER;

	adau1x61_update_reg(dev, ADAU1X61_SERIAL_PORT0,
			    ADAU1X61_SERIAL_PORT0_MASTER, reg);

	if (cfg->mclk_freq % i2s_cfg->frame_clk_freq != 0) {
		LOG_ERR("Wrong MCLK or PCM rate");
		return -EINVAL;
	}

	switch (cfg->mclk_freq / i2s_cfg->frame_clk_freq) {
		case 256:
			infreq = 0;
			break;
		case 512:
			infreq = 1;
			break;
		case 768:
			infreq = 2;
			break;
		case 1024:
			infreq = 3;
			break;
		default:
			return -EINVAL;
	}

#ifdef CONFIG_AUDIO_ADAU1x61_CLK_SRC_PLL
	adau1x61_configure_pll(dev, )
	adau1x61_write_reg(dev, ADAU1X61_CLOCK_CONTROL,
			   ADAU1X61_CLOCK_CONTROL_CLKSRC |
				   ADAU1X61_CLOCK_CONTROL_COREN);
#else

	adau1x61_write_reg(dev, ADAU1X61_CLOCK_CONTROL,
			   ADAU1X61_CLOCK_CONTROL_COREN | (infreq << 1));
#endif
	return 0;
}

int adau1x61_configure_pll(unsigned int freq_in, unsigned int freq_out,
	uint8_t regs[5])
{
#if 0
	unsigned int r, n, m, i, j;
	unsigned int div;

	if (!freq_out) {
		r = 0;
		n = 0;
		m = 0;
		div = 0;
	} else {
		if (freq_out % freq_in != 0) {
			div = DIV_ROUND_UP(freq_in, 13500000);
			freq_in /= div;
			r = freq_out / freq_in;
			i = freq_out % freq_in;
			j = gcd(i, freq_in);
			n = i / j;
			m = freq_in / j;
			div--;
		} else {
			r = freq_out / freq_in;
			n = 0;
			m = 0;
			div = 0;
		}
		if (n > 0xffff || m > 0xffff || div > 3 || r > 8 || r < 2)
			return -EINVAL;
	}

	regs[0] = m >> 8;
	regs[1] = m & 0xff;
	regs[2] = n >> 8;
	regs[3] = n & 0xff;
	regs[4] = (r << 3) | (div << 1);
	if (m != 0)
		regs[4] |= 1; /* Fractional mode */
#endif
	return 0;
}

static void adau1x61_configure_output(const struct device *dev)
{
#if 0
	uint8_t val;

	/*
	 * set common mode voltage to 1.65V (half of AVDD)
	 * AVDD is typically 3.3V
	 */
	adau1x61_read_reg(dev, HEADPHONE_DRV_ADDR, &val);
	val &= ~HEADPHONE_DRV_CM_MASK;
	val |= HEADPHONE_DRV_CM(CM_VOLTAGE_1P65) | HEADPHONE_DRV_RESERVED;
	adau1x61_write_reg(dev, HEADPHONE_DRV_ADDR, val);

	/* enable pop removal on power down/up */
	adau1x61_read_reg(dev, HP_OUT_POP_RM_ADDR, &val);
	adau1x61_write_reg(dev, HP_OUT_POP_RM_ADDR, val | HP_OUT_POP_RM_ENABLE);

	/* route DAC output to Headphone */
	val = OUTPUT_ROUTING_HPL | OUTPUT_ROUTING_HPR;
	adau1x61_write_reg(dev, OUTPUT_ROUTING_ADDR, val);

	/* enable volume control on Headphone out */
	adau1x61_write_reg(dev, HPL_ANA_VOL_CTRL_ADDR,
			HPX_ANA_VOL(HPX_ANA_VOL_DEFAULT));
	adau1x61_write_reg(dev, HPR_ANA_VOL_CTRL_ADDR,
			HPX_ANA_VOL(HPX_ANA_VOL_DEFAULT));

	/* set headphone outputs as line-out */
	adau1x61_write_reg(dev, HEADPHONE_DRV_CTRL_ADDR, HEADPHONE_DRV_LINEOUT);

	/* unmute headphone drivers */
	adau1x61_write_reg(dev, HPL_DRV_GAIN_CTRL_ADDR, HPX_DRV_UNMUTE);
	adau1x61_write_reg(dev, HPR_DRV_GAIN_CTRL_ADDR, HPX_DRV_UNMUTE);

	/* power up headphone drivers */
	adau1x61_read_reg(dev, HEADPHONE_DRV_ADDR, &val);
	val |= HEADPHONE_DRV_POWERUP | HEADPHONE_DRV_RESERVED;
	adau1x61_write_reg(dev, HEADPHONE_DRV_ADDR, val);
#endif
}

static int adau1x61_set_output_volume(const struct device *dev,
				   audio_channel_t channel, int vol)
{
	int8_t vol_val;

	if ((vol > CODEC_OUTPUT_VOLUME_MAX_DB) ||
			(vol < CODEC_OUTPUT_VOLUME_MIN_DB)) {
		LOG_ERR("Invalid volume %d.%d dB",
				vol >> 1, ((uint32_t)vol & 1) ? 5 : 0);
		return -EINVAL;
	}

	vol_val = vol - CODEC_OUTPUT_VOLUME_MIN_DB;

	adau1x61_update_reg(dev, ADAU1X61_PLAY_HP_LEFT_VOL, ADAU1X61_PLAY_VOL_MASK, vol_val << 2);
	adau1x61_update_reg(dev, ADAU1X61_PLAY_HP_RIGHT_VOL, ADAU1X61_PLAY_VOL_MASK, vol_val << 2);
	adau1x61_update_reg(dev, ADAU1X61_PLAY_LINE_LEFT_VOL, ADAU1X61_PLAY_VOL_MASK, vol_val << 2);
	adau1x61_update_reg(dev, ADAU1X61_PLAY_LINE_RIGHT_VOL, ADAU1X61_PLAY_VOL_MASK, vol_val << 2);
	return 0;
}

static int adau1x61_set_output_mute(const struct device *dev,
				    audio_channel_t channel, bool mute)
{
	uint8_t reg;

	if (mute)
		reg = ADAU1X61_PLAY_MUTE_MASK;
	else
		reg = 0;

	adau1x61_update_reg(dev, ADAU1X61_PLAY_HP_LEFT_VOL,
			    ADAU1X61_PLAY_MUTE_MASK, reg);
	adau1x61_update_reg(dev, ADAU1X61_PLAY_HP_RIGHT_VOL,
			    ADAU1X61_PLAY_MUTE_MASK, reg);
	adau1x61_update_reg(dev, ADAU1X61_PLAY_LINE_LEFT_VOL,
			    ADAU1X61_PLAY_MUTE_MASK, reg);
	adau1x61_update_reg(dev, ADAU1X61_PLAY_LINE_RIGHT_VOL,
			    ADAU1X61_PLAY_MUTE_MASK, reg);
	return 0;
}

#if (LOG_LEVEL >= LOG_LEVEL_DEBUG)
static void adau1x61_read_all_regs(const struct device *dev)
{
	struct adau1x61_driver_data *const dev_data = DEV_DATA(dev);
	uint8_t val;

	adau1x61_read_reg(dev, ADAU1X61_CLOCK_CONTROL, &val);
	//adau1x61_read_reg(dev, ADAU1X61_PLL_CONTROL, &val);
	adau1x61_read_reg(dev, ADAU1X61_SERIAL_PORT0, &val);
	adau1x61_read_reg(dev, ADAU1X61_SERIAL_PORT1, &val);
	adau1x61_read_reg(dev, ADAU1X61_CONVERTER0, &val);
	adau1x61_read_reg(dev, ADAU1X61_PLAY_MIXER_LEFT0, &val);
	adau1x61_read_reg(dev, ADAU1X61_PLAY_MIXER_RIGHT0, &val);
	adau1x61_read_reg(dev, ADAU1X61_PLAY_HP_LEFT_VOL, &val);
	adau1x61_read_reg(dev, ADAU1X61_PLAY_HP_RIGHT_VOL, &val);
	adau1x61_read_reg(dev, ADAU1X61_PLAY_LINE_LEFT_VOL, &val);
	adau1x61_read_reg(dev, ADAU1X61_PLAY_LINE_RIGHT_VOL, &val);
	adau1x61_read_reg(dev, ADAU1X61_DAC_CONTROL0, &val);
	adau1x61_read_reg(dev, ADAU1X61_DAC_CONTROL1, &val);
	adau1x61_read_reg(dev, ADAU1X61_DAC_CONTROL2, &val);
	if (dev_data->has_dsp) {
		adau1x61_read_reg(dev, ADAU1761_SERIAL_INPUT_ROUTE, &val);
		adau1x61_read_reg(dev, ADAU1761_SERIAL_OUTPUT_ROUTE, &val);
		adau1x61_read_reg(dev, ADAU1761_SERIAL_SAMPLING_RATE, &val);
		adau1x61_read_reg(dev, ADAU1761_CLK_ENABLE0, &val);
		adau1x61_read_reg(dev, ADAU1761_CLK_ENABLE1, &val);
	}
}
#endif

static const struct audio_codec_api adau1x61_driver_api = {
	.configure		= adau1x61_configure,
	.start_output		= adau1x61_start_output,
	.stop_output		= adau1x61_stop_output,
	.set_property		= adau1x61_set_property,
	.apply_properties	= adau1x61_apply_properties,
};

DEVICE_AND_API_INIT(adau1x61, DT_INST_LABEL(0), adau1x61_initialize,
		&adau1x61_device_data, &adau1x61_device_config, POST_KERNEL,
		CONFIG_AUDIO_CODEC_INIT_PRIORITY, &adau1x61_driver_api);
