    /*
 * Copyright (c) 2022-2023 Golioth, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pisemi_rd_template, LOG_LEVEL_DBG);

#include <app/drivers/pi15592.h> 
#include <app/drivers/pi15592_regmap.h> 

#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/dac.h>

#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

//static const struct device *dev = DEVICE_DT_GET_ANY(sgmicro_sgm41296);

#define AD559X_ADC_VREF_MV 2500U
#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

/*
#if !DT_NODE_EXISTS(ZEPHYR_USER_NODE) || \
	!DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, io_channels)
#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),
*/
// Data of ADC io-channels specified in devicetree.

/*
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(ZEPHYR_USER_NODE, io_channels,
				 DT_SPEC_AND_COMMA)
};
*/

/* ADC node from the devicetree. */
#define ADC_NODE DT_ALIAS(piadc)

/* Data of ADC device specified in devicetree. */
static const struct device* adc_dev = DEVICE_DT_GET(ADC_NODE);

/* Data array of ADC channels for the specified ADC. */
static const struct adc_channel_cfg channel_cfgs[] = {
	DT_FOREACH_CHILD_SEP(ADC_NODE, ADC_CHANNEL_CFG_DT, (,)) };

/* Get the number of channels defined on the DTS. */
#define CHANNEL_COUNT ARRAY_SIZE(channel_cfgs)

#if (DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac) && \
	DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac_channel_id) && \
	DT_NODE_HAS_PROP(ZEPHYR_USER_NODE, dac_resolution))
#define DAC_NODE DT_PHANDLE(ZEPHYR_USER_NODE, dac)
#define DAC_CHANNEL_ID DT_PROP(ZEPHYR_USER_NODE, dac_channel_id)
#define DAC_RESOLUTION DT_PROP(ZEPHYR_USER_NODE, dac_resolution)
#else
#error "Unsupported board: see README and check /zephyr,user node"
#define DAC_NODE DT_INVALID_NODE
#define DAC_CHANNEL_ID 0
#define DAC_RESOLUTION 0
#endif

static const struct device* const dac_dev = DEVICE_DT_GET(DAC_NODE);

static const struct dac_channel_cfg dac_ch_cfg = {
	.channel_id = DAC_CHANNEL_ID,
	.resolution = DAC_RESOLUTION,
	.buffered = true
};


int main(void) {
	int err;
	uint32_t count = 0;
	//uint16_t buf;
	uint16_t channel_reading[CONFIG_SEQUENCE_SAMPLES][CHANNEL_COUNT];

	if (!device_is_ready(dac_dev)) {
		printk("DAC Device %s is not ready\n", dac_dev->name);
		return 0;
	}

	int ret = pi15592_dac_channel_setup(dac_dev, &dac_ch_cfg);
	if (ret != 0) {
			printk("DAC channel cfg is not success\n");
			return 0;
	}

	/* Options for the sequence sampling. */
	const struct adc_sequence_options options = {
		.extra_samplings = CONFIG_SEQUENCE_SAMPLES - 1,
		.interval_us = 0,
	};

	/* Configure the sampling sequence to be made. */
	struct adc_sequence sequence = {
		.buffer = channel_reading,
		/* buffer size in bytes, not number of samples */
		.buffer_size = sizeof(channel_reading),
		.resolution = CONFIG_SEQUENCE_RESOLUTION,
		.options = &options,
	};

	if (!device_is_ready(adc_dev)) {
		printf("ADC controller device %s not ready\n", adc_dev->name);
		return 0;
	}

	/* Configure channels individually prior to sampling. */
	for (size_t i = 0U; i < CHANNEL_COUNT; i++) {
		sequence.channels |= BIT(channel_cfgs[i].channel_id);
		err = pi15592_adc_channel_setup(adc_dev, &channel_cfgs[i]);
		if (err < 0) {
			printf("Could not setup channel #%d (%d)\n", i, err);
			return 0;
		}
	}

	/*
	struct adc_sequence sequence = {
		.buffer = &buf,
		.buffer_size = sizeof(buf),
	};

	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		if (!adc_is_ready_dt(&adc_channels[i])) {
			printk("ADC controller device %s not ready\n", adc_channels[i].dev->name);
			return 0;
		}

		err = pi15592_adc_channel_setup(adc_dev, &adc_channels[i]);
		if (err < 0) { 
			printk("Could not setup channel #%d (%d)\n", i, err);
			return 0;
		}
	}
*/
	
	
#ifndef CONFIG_COVERAGE
	while (1) {
#else
	for (int k = 0; k < 10; k++) {
#endif
		/*ADC*/
		printf("ADC sequence reading [%u]:\n", count++);
		k_msleep(1000);

		err = pi15592_adc_read(adc_dev, &sequence);
		if (err < 0) {
			printf("Could not read (%d)\n", err);
			continue;
		}

		for (size_t channel_index = 0U; channel_index < CHANNEL_COUNT; channel_index++) {
			int32_t val_mv;

			printf("- %s, channel %" PRId32 ", %" PRId32 " sequence samples:\n",
				adc_dev->name, channel_cfgs[channel_index].channel_id,
				CONFIG_SEQUENCE_SAMPLES);
			for (size_t sample_index = 0U; sample_index < CONFIG_SEQUENCE_SAMPLES; sample_index++) {

				val_mv = channel_reading[sample_index][channel_index];

				printf("- - %" PRId32, val_mv);
				err = adc_raw_to_millivolts(channel_cfgs[channel_index].reference, channel_cfgs[channel_index].gain, CONFIG_SEQUENCE_RESOLUTION, &val_mv);

				/* conversion to mV may not be supported, skip if not */
				if ((err < 0) || channel_cfgs[channel_index].reference == 0) {
					printf(" (value in mV not available)\n");
				}
				else {
					printf(" = %" PRId32 "mV\n", val_mv);
				}
			}
		}
		/*DAC*/
		/* Number of valid DAC values, e.g. 4096 for 12-bit DAC */
		const int dac_values = 1U << DAC_RESOLUTION;
		/*
		 * 1 msec sleep leads to about 4 sec signal period for 12-bit
		 * DACs. For DACs with lower resolution, sleep time needs to
		 * be increased.
		 * Make sure to sleep at least 1 msec even for future 16-bit
		 * DACs (lowering signal frequency).
		 */
		const int sleep_time = 4096 / dac_values > 0 ? 4096 / dac_values : 1;
		for (int i = 0; i < dac_values; i++) {
			ret = pi15592_dac_write(dac_dev, DAC_CHANNEL_ID, i);
			if (ret != 0) {
				printk("dac_write_value() failed with code %d\n", ret);
				return 0;
			}
			k_sleep(K_MSEC(sleep_time));
		}

	}
	return 0;
}
