/*
 * Copyright (c) 2023 Golioth, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT sgmicro_sgm41296

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sgmicro_sgm41296, CONFIG_SGM41296_LOG_LEVEL);

#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <string.h>
#include <app/drivers/libsgm41296.h>
#include <app/drivers/libsgm41296_regmap.h>

static int sgm41296_write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct sgm41296_config *config = dev->config;
	return i2c_reg_write_byte_dt(&config->i2c, reg, val);
}

static int sgm41296_read_reg(const struct device *dev, uint8_t reg, uint8_t *val)
{
	const struct sgm41296_config *config = dev->config;
	return i2c_write_read_dt(&config->i2c, &reg, 1, val, 1);
}

static int set_config(const struct device *dev, uint8_t val)
{
	return sgm41296_write_reg(dev, SGM41296_REG_SYS_SET, val);
}

static int read_status(const struct device *dev, uint8_t *val)
{
	return sgm41296_read_reg(dev, SGM41296_REG_STATUS, val);
}

static int read_addr(const struct device *dev, uint8_t *val)
{
	return sgm41296_read_reg(dev, SGM41296_REG_ADDR, val);
}

static const struct sgm41296_driver_api sgm41296_api = {
	.sgm41296_set_config = &set_config,
	.sgm41296_read_status = &read_status,
	.sgm41296_read_addr = &read_addr,
};


static int sgm41296_init(const struct device *dev)
{
	const struct sgm41296_config *config = dev->config;

	if (!device_is_ready(config->i2c.bus)) {
		LOG_ERR("I2C bus device not ready");
		return -ENODEV;
	}
	return 0;
}

#define SGM41296_DEFINE(inst)                                                                      \
	static const struct sgm41296_config sgm41296_config_##inst = {                             \
		.i2c = I2C_DT_SPEC_INST_GET(inst),                                                 \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, sgm41296_init, NULL, NULL, &sgm41296_config_##inst,            \
			      POST_KERNEL, CONFIG_SGM41296_INIT_PRIORITY, &sgm41296_api);

DT_INST_FOREACH_STATUS_OKAY(SGM41296_DEFINE)




