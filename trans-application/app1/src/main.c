    /*
 * Copyright (c) 2022-2023 Golioth, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sgmicro_rd_template, LOG_LEVEL_DBG);

#include <app/drivers/libsgm41296.h>
#include <app/drivers/libsgm41296_regmap.h>

#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>

static const struct device *dev = DEVICE_DT_GET_ANY(sgmicro_sgm41296);


int main(void) {
	uint8_t val_addr = 0;
	uint8_t val_status = 0;

	if (!device_is_ready(dev)) {
		printk("Device %s is not ready\n", dev->name);
		return 0;
	}
	printk("device is %p, name is %s\n", dev, dev->name);
	
	k_msleep(30); /* sleep at least 20ms for detect ADDR state */
	sgm41296_set_config(dev, 0x81);

	sgm41296_read_addr(dev, &val_addr);
	printk("reg_addr is %u\n", val_addr);
	sgm41296_read_status(dev, &val_status);
	printk("reg_status is %u\n", val_status);
	//LOG_INF("reg_addr is: %s", val_addr);
	//sgm41296_set_config(dev, 0x80);
	while(1) {

		sgm41296_read_status(dev, &val_status);
		//printk("reg_status is %u", val_status);
		k_sleep(K_SECONDS(1));
	}
}
