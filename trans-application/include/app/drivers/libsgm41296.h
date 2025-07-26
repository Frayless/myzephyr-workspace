/*
 * Copyright (c) 2023 Golioth, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APP_DRIVERS_LIBSGM41296_H_
#define APP_DRIVERS_LIBSGM41296_H_

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

struct sgm41296_config {
	struct i2c_dt_spec i2c;
};

typedef int (*sgm41296_setval)(const struct device *dev, uint8_t val);
typedef int (*sgm41296_getval)(const struct device *dev, uint8_t *val);

__subsystem struct sgm41296_driver_api {
	sgm41296_setval sgm41296_set_config;
	sgm41296_getval sgm41296_read_status;
	sgm41296_getval sgm41296_read_addr;
};

__syscall int sgm41296_set_config(const struct device *dev, uint8_t val);

static inline int z_impl_sgm41296_set_config(const struct device *dev, uint8_t val)
{
	const struct sgm41296_driver_api *api = (const struct sgm41296_driver_api *)dev->api;
	if (api->sgm41296_set_config == NULL) {
		return -ENOSYS;
	}
	return api->sgm41296_set_config(dev, val);
}

__syscall int sgm41296_read_status(const struct device *dev, uint8_t *val);

static inline int z_impl_sgm41296_read_status(const struct device *dev, uint8_t *val)
{
	const struct sgm41296_driver_api *api = (const struct sgm41296_driver_api *)dev->api;
	if (api->sgm41296_read_status == NULL) {
		return -ENOSYS;
	}
	return api->sgm41296_read_status(dev, val);
}

__syscall int sgm41296_read_addr(const struct device *dev, uint8_t *val);

static inline int z_impl_sgm41296_read_addr(const struct device *dev, uint8_t *val)
{
	const struct sgm41296_driver_api *api = (const struct sgm41296_driver_api *)dev->api;
	if (api->sgm41296_read_addr == NULL) {
		return -ENOSYS;
	}
	return api->sgm41296_read_addr(dev, val);
}

#include <syscalls/libsgm41296.h>
#endif
