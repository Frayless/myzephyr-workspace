# Copyright (c) 2024 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
#zephyr_syscall_header(${ZEPHYR_LIBSGM41296_MODULE_DIR}/include/libsgm41296.h)
#zephyr_include_directories(include)
zephyr_library_sources(libsgm41296.c)

