/*nrf5340dk_nrf5340/nrf5340dk_nrf5340_cpuapp.dts
 * Copyright (c) 2019 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <drivers/i2s.h>
#include <audio/codec.h>

#include <string.h>

#define LOG_LEVEL LOG_LEVEL_DBG
#include <logging/log.h>
LOG_MODULE_REGISTER(drivers_adio_sample);

#define AUDIO_SAMPLE_FREQ		(48000)
#define AUDIO_NUM_CHANNELS		(2)
#define AUDIO_SAMPLE_BIT_WIDTH		(32)

static const struct device *codec_device;

static void audio_init(void)
{
	int ret;
	struct i2s_config i2s_cfg;
	struct audio_codec_cfg codec_cfg;

	codec_device = device_get_binding(DT_LABEL(DT_INST(0, adi_adau1x61)));
	if (!codec_device) {
		LOG_ERR("unable to find " DT_LABEL(DT_INST(0, adi_adau1x61)) " device");
		return;
	}

	/* configure i2s DAI*/
	i2s_cfg.word_size = AUDIO_SAMPLE_BIT_WIDTH;
	i2s_cfg.channels = AUDIO_NUM_CHANNELS;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_I2S | I2S_FMT_CLK_NF_NB;
	i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER |
		I2S_OPT_BIT_CLK_MASTER;
	i2s_cfg.frame_clk_freq = AUDIO_SAMPLE_FREQ;

	/* configure codec */
	codec_cfg.dai_type = AUDIO_DAI_TYPE_I2S,
	codec_cfg.dai_cfg.i2s = i2s_cfg;
	codec_cfg.dai_cfg.i2s.mem_slab = NULL;
	codec_cfg.mclk_freq = 12288000U;

	ret = audio_codec_configure(codec_device, &codec_cfg);
	if (ret)
		LOG_ERR("Unsupported I2S data format");
}

static void audio_start(void)
{
	int ret;

	LOG_DBG("Starting audio playback...");
	/* start codec input */
	audio_codec_start_input(codec_device);

	/* start codec output */
	audio_codec_start_output(codec_device);

	/* Line out setup */
	ret = audio_codec_set_property(codec_device, AUDIO_PROPERTY_OUTPUT_VOLUME, AUDIO_CHANNEL_LINE_LEFT, (audio_property_value_t)255);
	ret = audio_codec_set_property(codec_device, AUDIO_PROPERTY_OUTPUT_VOLUME, AUDIO_CHANNEL_LINE_RIGHT, (audio_property_value_t)255);
	ret = audio_codec_set_property(codec_device, AUDIO_PROPERTY_OUTPUT_MUTE, AUDIO_CHANNEL_LINE_LEFT, (audio_property_value_t)false);
	ret = audio_codec_set_property(codec_device, AUDIO_PROPERTY_OUTPUT_MUTE, AUDIO_CHANNEL_LINE_RIGHT, (audio_property_value_t)false);

	/* MIC setup */
	ret = audio_codec_set_property(codec_device, AUDIO_PROPERTY_INPUT_VOLUME, AUDIO_CHANNEL_MIC_LEFT, (audio_property_value_t)16);
	ret = audio_codec_set_property(codec_device, AUDIO_PROPERTY_INPUT_VOLUME, AUDIO_CHANNEL_MIC_RIGHT, (audio_property_value_t)16);
	ret = audio_codec_set_property(codec_device, AUDIO_PROPERTY_INPUT_MUTE, AUDIO_CHANNEL_MIC_LEFT, (audio_property_value_t)false);
	ret = audio_codec_set_property(codec_device, AUDIO_PROPERTY_INPUT_MUTE, AUDIO_CHANNEL_MIC_RIGHT, (audio_property_value_t)false);

	/* HP setup */
	ret = audio_codec_set_property(codec_device, AUDIO_PROPERTY_OUTPUT_VOLUME, AUDIO_CHANNEL_HP_LEFT, (audio_property_value_t)57);
	ret = audio_codec_set_property(codec_device, AUDIO_PROPERTY_OUTPUT_VOLUME, AUDIO_CHANNEL_HP_RIGHT, (audio_property_value_t)57);
	ret = audio_codec_set_property(codec_device, AUDIO_PROPERTY_OUTPUT_MUTE, AUDIO_CHANNEL_HP_LEFT, (audio_property_value_t)false);
	ret = audio_codec_set_property(codec_device, AUDIO_PROPERTY_OUTPUT_MUTE, AUDIO_CHANNEL_HP_RIGHT, (audio_property_value_t)false);
}

static void audio_stop(void)
{
	int ret;

	LOG_DBG("Stopping audio playback...");

	audio_codec_stop_output(codec_device);

	audio_codec_stop_input(codec_device);
}

static void audio_sample_app(void *p1, void *p2, void *p3)
{
	LOG_INF("Starting audio application ...");

	audio_init();

	k_msleep(500);

	audio_start();

	k_msleep(100000);

	audio_stop();

	LOG_INF("Exiting audio sample app ...");
	k_thread_suspend(k_current_get());
}

K_THREAD_DEFINE(audio_sample, 1024, audio_sample_app, NULL, NULL, NULL,
		10, 0, 0);
