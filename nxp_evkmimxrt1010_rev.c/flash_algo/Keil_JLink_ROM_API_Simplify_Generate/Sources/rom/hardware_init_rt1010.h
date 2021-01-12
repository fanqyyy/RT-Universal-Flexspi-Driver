/*
 * Copyright 2014-2015 Freescale Semiconductor, Inc.
 * Copyright 2016-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef __HARDWARE_INIT_RT1010_H__
#define __HARDWARE_INIT_RT1010_H__

#include "bl_flexspi.h"
#include "bl_common.h"

void flexspi_iomux_config_rt1010(uint32_t instance, flexspi_mem_config_t *config);

void flexspi_update_padsetting_rt1010(flexspi_mem_config_t *config, uint32_t driveStrength);

void flexspi_clock_config_rt1010(uint32_t instance, uint32_t freq, uint32_t sampleClkMode);

status_t flexspi_set_failsafe_setting_rt1010(flexspi_mem_config_t *config);

status_t flexspi_get_max_supported_freq_rt1010(uint32_t instance, uint32_t *freq, uint32_t clkMode);

uint32_t CLOCK_GetCPUFreq_RT1010(void);

status_t flexspi_get_clock_rt1010(uint32_t instance, flexspi_clock_type_t type, uint32_t *freq);

#endif // __HARDWARE_INIT_RT1010_H__
