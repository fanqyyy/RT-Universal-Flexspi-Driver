/*
 * Copyright 2014-2016 Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef __BL_API_H__
#define __BL_API_H__

#include "fsl_device_registers.h"
#include "flexspi_nor_flash.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define RT1010_ROM_API_TREE_ADDR (0x0020001c)

typedef struct
{
    uint32_t version;
    status_t (*init)(uint32_t instance, flexspi_nor_config_t *config);
    void (*clear_cache)(uint32_t instance);
    status_t (*xfer)(uint32_t instance, flexspi_xfer_t *xfer);
    status_t (*update_lut)(uint32_t instance, uint32_t seqIndex, const uint32_t *lutBase, uint32_t seqNumber);
} flexspi_nor_flash_driver_imxrt1010_t;


typedef struct
{
    const uint32_t version;                                 //!< Bootloader version number
    const char *copyright;                                  //!< Bootloader Copyright
    void (*runBootloader)(void *arg);                       //!< Function to start the bootloader executing
    const uint32_t *reserved0;                              //!< Reserved
    const flexspi_nor_flash_driver_imxrt1010_t *flexSpiNorDriver; //!< FlexSPI NOR Flash API
} bootloader_tree_imxrt1010_t;


#define g_bootloaderTree_imxrt1010 (*(bootloader_tree_imxrt1010_t **)RT1010_ROM_API_TREE_ADDR)

#endif //__BL_API_H__
