/*
 * Copyright (c) 2023 Golioth, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT pisemi_pi15592

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pisemi_pi15592, CONFIG_PI15592_LOG_LEVEL);


#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/adc.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#include <zephyr/drivers/dac.h>
#include <app/drivers/pi15592.h> 
#include <app/drivers/pi15592_regmap.h> 


struct pi15592_config {
	struct spi_dt_spec spi;
};

struct pi15592_data {
	struct adc_context ctx;
	const struct device* dev;
	uint8_t dac_chan_cfg;
	uint8_t adc_chan_cfg;
	uint8_t adc_channels;

	uint16_t* buffer;
	uint16_t* repeat_buffer;

	struct k_thread thread;
	struct k_sem sem;

	K_KERNEL_STACK_MEMBER(stack, CONFIG_PI15592_ACQUISITION_THREAD_STACK_SIZE);
	 
};

static int pi15592_write(const struct device *dev, uint8_t *val, size_t len)
{
	const struct pi15592_config *config = dev->config;
	uint16_t nop_msg = 0;

	struct spi_buf tx_buf[] = { {.buf = &nop_msg, .len = sizeof(nop_msg)} };

	const struct spi_buf_set tx = { .buffers = tx_buf, .count = 1 };

	struct spi_buf rx_buf[] = { {.buf = val, .len = len} };

	const struct spi_buf_set rx = { .buffers = rx_buf, .count = 1 };

	return spi_transceive_dt(&config->spi, &tx, &rx);
}
 
static int pi15592_read(const struct device *dev, uint8_t *val, size_t len)
{
	const struct pi15592_config *config = dev->config;

	struct spi_buf tx_buf[] = { {.buf = val, .len = len} };

	const struct spi_buf_set tx = { .buffers = tx_buf, .count = 1 };

	return spi_write_dt(&config->spi, &tx);
}

static int write_reg(const struct device *dev, uint8_t reg, uint16_t val)
{
	uint16_t msg;
	msg = sys_cpu_to_be16((reg << PI15592_REG_SHIFT_VAL) | val);

	return pi15592_write(dev, (uint8_t *)&msg, sizeof(msg)); 
}

static int read_reg(const struct device *dev, uint8_t reg, uint8_t reg_data, uint16_t *val)
{
	uint16_t msg;
	uint16_t data;
	int ret;
	switch (reg) {
	case PI15592_REG_GPI_CFG:
			msg = sys_cpu_to_be16(PI15592_GPI_READBACK_EN | (reg << PI15592_REG_SHIFT_VAL) | reg_data);
			break;
	default:
			msg = sys_cpu_to_be16(PI15592_LDAC_READBACK_EN | (PI15592_REG_RB_AND_LDAC << PI15592_REG_SHIFT_VAL) | (reg << PI15592_REG_READBACK_SHIFT_VAL));
			break;
	}

	ret = pi15592_write(dev, (uint8_t*)&msg, sizeof(msg));
	if (ret < 0) {
		return ret;
	} 

	ret = pi15592_read(dev, (uint8_t*)&data, sizeof(data));
	if (ret < 0) {
		return ret;
	}

	*val = sys_be16_to_cpu(data);

	return 0;
}

static int software_reset(const struct device *dev) {
	return write_reg(dev, PI15592_REG_SOFTWARE_RESET, PI15592_RESET_VAL);
}

/*DAC*/

static int pi_dac_channel_setup(const struct device *dev, const struct dac_channel_cfg *channel_cfg) {
	//const struct pi15592_config *config = dev->config;
	struct pi15592_data *data = dev->data;

	if (channel_cfg->channel_id > PI15592_PIN_MAX - 1) {
		LOG_ERR("Invalid channel number %d", channel_cfg->channel_id);
		return -EINVAL;
	}

	if (channel_cfg->resolution != PI15592_RESOLUTION) {
		LOG_ERR("Invalid channel number %d", channel_cfg->resolution);
		return -EINVAL;
	}
	data->dac_chan_cfg |= BIT(channel_cfg->channel_id);
	return write_reg(dev, PI15592_REG_DAC_CFG, data->dac_chan_cfg);
}

static int pi_dac_write(const struct device *dev, uint8_t channel, uint16_t val){
	
	uint16_t msg;

	if (channel > PI15592_PIN_MAX - 1) {
		LOG_ERR("Invalid channel number %d", channel);
		return -EINVAL;
	}

	if (val >= (1 << PI15592_RESOLUTION)) {
		LOG_ERR("Value %d out of range", val);
		return -EINVAL;
	}

	msg = sys_cpu_to_be16(PI15592_DAC_WRITE_EN | (channel << PI15592_DAC_WRITE_SHIFT_VAL) | val);
	
	return pi15592_write(dev, (uint8_t*)&msg, sizeof(msg));
}

/*ADC*/
static int pi_adc_channel_setup(const struct device *dev, const struct adc_channel_cfg* channel_cfg) {
	//const struct pi15592_config *config = dev->config;
	struct pi15592_data *data = dev->data;

	if (channel_cfg->channel_id > PI15592_PIN_MAX - 1) {
		LOG_ERR("Invalid channel number %d", channel_cfg->channel_id);
		return -EINVAL;
	}

	data->adc_chan_cfg |= BIT(channel_cfg->channel_id);

	return write_reg(dev, PI15592_REG_ADC_CFG, data->adc_chan_cfg);
}

static int pi15592_adc_validate_buffer_size(const struct device *dev, const struct adc_sequence *sequence) {
	uint8_t channels;
	size_t needed;

	channels = POPCOUNT(sequence -> channels);
	needed = channels * sizeof(uint16_t);

	if (sequence->buffer_size < needed) {
		return -ENOMEM;
	}

	return 0;
}

static int pi15592_adc_start_read(const struct device* dev, const struct adc_sequence* sequence) {
	struct pi15592_data* data = dev->data;
	int ret;

	if (find_msb_set(sequence -> channels) > PI15592_PIN_MAX) {
		LOG_ERR("Invalid channel in mask 0x%08x", sequence->channels);
		return -EINVAL;
	}

	if (sequence->resolution != PI15592_RESOLUTION) {
		LOG_ERR("invalid resolution %d", sequence->resolution);
		return -EINVAL;
	}

	ret = pi15592_adc_validate_buffer_size(dev, sequence);
	if (ret < 0) {
		LOG_ERR("insufficient buffer size");
		return ret;
	}

	data->buffer = sequence->buffer;
	adc_context_start_read(&data->ctx, sequence);

	return adc_context_wait_for_completion(&data->ctx);
}


static void adc_context_start_sampling(struct adc_context *ctx){
	struct pi15592_data *data = CONTAINER_OF(ctx, struct pi15592_data, ctx);

	data->adc_channels = ctx->sequence.channels;
	data->repeat_buffer = data->buffer;

	k_sem_give(&data->sem);
}

static void adc_context_update_buffer_pointer(struct adc_context* ctx, bool repeat_sampling){
	struct pi15592_data* data = CONTAINER_OF(ctx, struct pi15592_data, ctx);

	if (repeat_sampling){
		data->buffer = data->repeat_buffer;
	}
}


static int pi15592_adc_read_channel(const struct device* dev, uint8_t channel, uint16_t* result){
	
	uint16_t val;
	int ret;

	ret = write_reg(dev, PI15592_REG_ADC_SEQ, BIT(channel));
	if (ret < 0) {
		return ret;
	}

	(void)pi15592_read(dev, (uint8_t *)&val, sizeof(val));
	ret = pi15592_read(dev, (uint8_t *)&val, sizeof(val));
	if (ret < 0) {
		return ret;
	}
	val = sys_be16_to_cpu(val);

	*result = val;

	return 0;
}

static void pi15592_adc_acquisition_thread(struct pi15592_data* data)
{
	uint16_t result;
	uint8_t channel;
	int ret;

	while (true) {
		k_sem_take(&data->sem, K_FOREVER);

		while (data->adc_channels != 0) {
			channel = find_lsb_set(data->adc_channels) - 1;

			LOG_DBG("reading channel %d", channel);

			ret = pi15592_adc_read_channel(data->dev, channel, &result);
			if (ret < 0) {
				LOG_ERR("failed to read channel %d (ret %d)", channel, ret);
				adc_context_complete(&data->ctx, ret);
				break;
			}

			*data->buffer++ = result;
			WRITE_BIT(data->adc_channels, channel, 0);
		}

		adc_context_on_sampling_done(&data->ctx, data->dev);
	}
}

static int pi_adc_read(const struct device* dev, const struct adc_sequence* sequence) {
	struct pi15592_data* data = dev->data;
	int ret;
	bool sync = false;

	adc_context_lock(&data->ctx, sync, NULL);
	ret = pi15592_adc_start_read(dev, sequence);
	adc_context_release(&data->ctx, ret);
	
	return ret;
}


static int pi15592_init(const struct device* dev)
{
	const struct pi15592_config* config = dev->config;
	struct pi15592_data* data = dev->data;
	k_tid_t tid;
	int ret;
	data->dev = dev;

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI bus %s not ready", config->spi.bus->name);
		return -ENODEV;
	}

	ret = write_reg(dev, PI15592_REG_RF, PI15592_EN_REF);
	if (ret < 0) {
		return ret;
	}

	k_sem_init(&data->sem, 0, 1);
	adc_context_init(&data->ctx);

	tid = k_thread_create(&data->thread, data->stack,
		K_KERNEL_STACK_SIZEOF(data->stack),
		(k_thread_entry_t)pi15592_adc_acquisition_thread, data, NULL, NULL,
		CONFIG_PI15592_ACQUISITION_THREAD_PRIO, 0, K_NO_WAIT);

	if (IS_ENABLED(CONFIG_THREAD_NAME)) {
		ret = k_thread_name_set(tid, "adc_pi15592");
		if (ret < 0) {
			return ret;
		}
	}

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}



static const struct pi15592_driver_api pi15592_api = {
	.pi15592_write_reg = &write_reg,
	.pi15592_dac_write = &pi_dac_write,
	.pi15592_read_reg = &read_reg,
	.pi15592_software_reset = &software_reset,
	.pi15592_dac_channel_setup = &pi_dac_channel_setup,
	.pi15592_adc_channel_setup = &pi_adc_channel_setup,
	.pi15592_adc_read = &pi_adc_read,
};



#define PI15592_SPI_CFG \
		(SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_WORD_SET(8) | SPI_TRANSFER_MSB)

#define PI15592_DEFINE(inst)																		 \
	static const struct pi15592_config pi15592_config##inst = {									 \
		.spi = SPI_DT_SPEC_INST_GET(inst, PI15592_SPI_CFG, 0),										 \
	};                                                                                               \
	static struct pi15592_data pi15592_data##inst;													 \
																									 \
	DEVICE_DT_INST_DEFINE(inst, pi15592_init, NULL, &pi15592_data##inst, &pi15592_config##inst,    \
			      POST_KERNEL, CONFIG_PI15592_INIT_PRIORITY, &pi15592_api);

DT_INST_FOREACH_STATUS_OKAY(PI15592_DEFINE)




