/*
 * Copyright (c) 2023 Golioth, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APP_DRIVERS_PI15592_H_
#define APP_DRIVERS_PI15592_H_

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/dac.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef int (*pi155592_setval)(const struct device* dev, uint8_t reg, uint16_t val);
typedef int (*pi15592_getval)(const struct device* dev, uint8_t reg, uint8_t reg_data, uint16_t* val);
typedef int (*pi15592_set_spereg)(const struct device* dev);
typedef int (*pi15592_dac_cfg)(const struct device* dev, const struct dac_channel_cfg* channel_cfg);
typedef int (*pi15592_adc_cfg)(const struct device* dev, const struct adc_channel_cfg* channel_cfg);
typedef int (*pi15592_adc_seq)(const struct device* dev, const struct adc_sequence* sequence);

__subsystem struct pi15592_driver_api {
	pi155592_setval pi15592_write_reg;
	pi155592_setval pi15592_dac_write;
	pi15592_getval pi15592_read_reg;
	pi15592_set_spereg pi15592_software_reset;
	pi15592_dac_cfg pi15592_dac_channel_setup;
	pi15592_adc_cfg pi15592_adc_channel_setup;
	pi15592_adc_seq pi15592_adc_read;
};

__syscall int pi15592_write_reg(const struct device* dev, uint8_t reg, uint16_t val);

static inline int z_impl_pi15592_write_reg(const struct device* dev, uint8_t reg, uint16_t val)
{
	const struct pi15592_driver_api *api = (const struct pi15592_driver_api *)dev->api;
	if (api->pi15592_write_reg == NULL) {
		return -ENOSYS;
	}
	return api->pi15592_write_reg(dev, reg, val);
}

__syscall int pi15592_dac_write(const struct device* dev, uint8_t reg, uint16_t val);

static inline int z_impl_pi15592_dac_write(const struct device* dev, uint8_t reg, uint16_t val)
{
	const struct pi15592_driver_api* api = (const struct pi15592_driver_api*)dev->api;
	if (api->pi15592_dac_write == NULL) {
		return -ENOSYS;
	}
	return api->pi15592_dac_write(dev, reg, val);
}

__syscall int pi15592_read_reg(const struct device* dev, uint8_t reg, uint8_t reg_data, uint16_t* val);

static inline int z_impl_pi15592_read_reg(const struct device* dev, uint8_t reg, uint8_t reg_data, uint16_t* val)
{
	const struct pi15592_driver_api* api = (const struct pi15592_driver_api*)dev->api;
	if (api->pi15592_read_reg == NULL) {
		return -ENOSYS;
	}
	return api->pi15592_read_reg(dev, reg, reg_data, val);
}

__syscall int pi15592_software_reset(const struct device* dev);

static inline int z_impl_pi15592_software_reset(const struct device* dev)
{
	const struct pi15592_driver_api* api = (const struct pi15592_driver_api*)dev->api;
	if (api->pi15592_software_reset == NULL) {
		return -ENOSYS;
	}
	return api->pi15592_software_reset(dev);
}

__syscall int pi15592_dac_channel_setup(const struct device* dev, const struct dac_channel_cfg* channel_cfg);

static inline int z_impl_pi15592_dac_channel_setup(const struct device* dev, const struct dac_channel_cfg* channel_cfg)
{
	const struct pi15592_driver_api* api = (const struct pi15592_driver_api*)dev->api;
	if (api->pi15592_dac_channel_setup == NULL) {
		return -ENOSYS;
	}
	return api->pi15592_dac_channel_setup(dev, channel_cfg);
}

__syscall int pi15592_adc_channel_setup(const struct device* dev, const struct adc_channel_cfg* channel_cfg);

static inline int z_impl_pi15592_adc_channel_setup(const struct device* dev, const struct adc_channel_cfg* channel_cfg)
{
	const struct pi15592_driver_api* api = (const struct pi15592_driver_api*)dev->api;
	if (api->pi15592_adc_channel_setup == NULL) {
		return -ENOSYS;
	}
	return api->pi15592_adc_channel_setup(dev, channel_cfg);
}

__syscall int pi15592_adc_read(const struct device* dev, const struct adc_sequence* sequence);

static inline int z_impl_pi15592_adc_read(const struct device* dev, const struct adc_sequence* sequence)
{
	const struct pi15592_driver_api* api = (const struct pi15592_driver_api*)dev->api;
	if (api->pi15592_adc_read == NULL) {
		return -ENOSYS;
	}
	return api->pi15592_adc_read(dev, sequence);
}


#include <syscalls/pi15592.h>
#endif
