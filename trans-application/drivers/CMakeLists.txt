# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

# Out-of-tree drivers for custom classes
add_subdirectory_ifdef(CONFIG_BLINK blink)
add_subdirectory_ifdef(CONFIG_LIB_SGM41296 tec)
add_subdirectory_ifdef(CONFIG_PI15592 pi15592)

# Out-of-tree drivers for existing driver classes
add_subdirectory_ifdef(CONFIG_SENSOR sensor)
add_subdirectory_ifdef(CONFIG_ADC adc)
