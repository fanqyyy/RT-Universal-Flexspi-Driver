/*
 * Copyright (c) 2013-2015 Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __BOOTLOADER_COMMON_H__
#define __BOOTLOADER_COMMON_H__

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "fsl_common.h"

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
//! @name Alignment macros
//@{
#ifndef ALIGN_DOWN
#define ALIGN_DOWN(x, a) ((x) & -(a))
#endif
#ifndef ALIGN_UP
#define ALIGN_UP(x, a) (-(-(x) & -(a)))
#endif
//@}

//! @brief Bootloader status group numbers.
//!
//! @ingroup bl_core
enum _bl_status_groups
{
    kStatusGroup_Bootloader = 100,      //!< Bootloader status group number (100).
    kStatusGroup_SBLoader = 101,        //!< SB loader status group number (101).
    kStatusGroup_MemoryInterface = 102, //!< Memory interface status group number (102).
    kStatusGroup_PropertyStore = 103,   //!< Property store status group number (103).
    kStatusGroup_AppCrcCheck = 104,     //!< Application crc check status group number (104).
    kStatusGroup_Packetizer = 105,      //!< Packetizer status group number (105).
    kStatusGroup_ReliableUpdate = 106,  //!< Reliable Update status groupt number (106).

    kStatusGroup_SerialNorEeprom = 107, //!< Serial NOR/EEPROM status group number
    kStatusGroup_FlexSPINAND = 200,     //!< FlexSPINAND status group number.
    kStatusGroup_FLEXSPINOR = 201,      //!< FlexSPINOR status group number.
    kStatusGroup_OCOTP = 202,           //!< OCOTP status group number.
    kStatusGroup_SemcNOR = 211,         //!< SEMC NOR status group number.
    kStatusGroup_SemcNAND = 212,        //!< SEMC NAND status group number.
};

enum
{
    kFlexSpiSerialClk_30MHz = 1,
    kFlexSpiSerialClk_50MHz = 2,
    kFlexSpiSerialClk_60MHz = 3,
    kFlexSpiSerialClk_75MHz = 4,
    kFlexSpiSerialClk_80MHz = 5,
    kFlexSpiSerialClk_100MHz = 6,
    kFlexSpiSerialClk_133MHz = 7,
    kFlexSpiSerialClk_166MHz = 8,
    kFlexSpiSerialClk_200MHz = 9,
};

////////////////////////////////////////////////////////////////////////////////
// Definitions
////////////////////////////////////////////////////////////////////////////////
#ifndef FREQ_1MHz
#define FREQ_1MHz (1UL * 1000 * 1000)
#endif
#ifndef FREQ_24MHz
#define FREQ_24MHz (24UL * 1000 * 1000)
#endif
#ifndef FREQ_396MHz
#define FREQ_396MHz (396UL * 1000 * 1000)
#endif
#ifndef FREQ_432MHz
#define FREQ_432MHz (432UL * 1000 * 1000)
#endif
#ifndef FREQ_480MHz
#define FREQ_480MHz (480UL * 1000 * 1000)
#endif
#ifndef FREQ_500MHz
#define FREQ_500MHz (500UL * 1000 * 1000)
#endif
#ifndef FREQ_508MHz
#define FREQ_508MHz (508UL * 1000 * 1000)
#endif
#ifndef FREQ_528MHz
#define FREQ_528MHz (528UL * 1000 * 1000)
#endif

#define SystemCoreClock 297000000UL

/*====================== FLEXSPI IOMUXC Definitions ===========================*/
#define IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(x)        (((uint32_t)(((uint32_t)(x)) << 0U)) & 0x7U)

#define FLEXSPIA_MUX_VAL               IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(1)
#define FLEXSPIB_MUX_VAL               IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(1)
#define FLEXSPIA_SS1_MUX_VAL           IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(6)
#define FLEXSPIB_SS1_MUX_VAL           IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(6)
#define FLEXSPIB_SS0_MUX_VAL           IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(6)
#define FLEXSPIB_DQS_MUX_VAL           IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(6)

#define IOMUXC_SW_PAD_CTL_PAD_SRE(x)             (((uint32_t)(((uint32_t)(x)) << 0U)) & 0x1U)
#define IOMUXC_SW_PAD_CTL_PAD_DSE(x)             (((uint32_t)(((uint32_t)(x)) << 3U)) & 0x38U)
#define IOMUXC_SW_PAD_CTL_PAD_SPEED(x)           (((uint32_t)(((uint32_t)(x)) << 6U)) & 0xC0U)
#define IOMUXC_SW_PAD_CTL_PAD_PKE(x)             (((uint32_t)(((uint32_t)(x)) << 12U)) & 0x1000U)
#define IOMUXC_SW_PAD_CTL_PAD_PUE(x)             (((uint32_t)(((uint32_t)(x)) << 13U)) & 0x2000U)
#define IOMUXC_SW_PAD_CTL_PAD_PUS(x)             (((uint32_t)(((uint32_t)(x)) << 14U)) & 0xC000U)

#define FLEXSPI_SW_PAD_CTL_VAL    (IOMUXC_SW_PAD_CTL_PAD_SRE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(6) |   \
                                     IOMUXC_SW_PAD_CTL_PAD_SPEED(3) | IOMUXC_SW_PAD_CTL_PAD_PKE(1) | \
                                     IOMUXC_SW_PAD_CTL_PAD_PUE(0) | IOMUXC_SW_PAD_CTL_PAD_PUS(0))

#define FLEXSPI_DQS_SW_PAD_CTL_VAL    (IOMUXC_SW_PAD_CTL_PAD_SRE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(6) |   \
                                     IOMUXC_SW_PAD_CTL_PAD_SPEED(3) | IOMUXC_SW_PAD_CTL_PAD_PKE(1) | \
                                     IOMUXC_SW_PAD_CTL_PAD_PUE(1) | IOMUXC_SW_PAD_CTL_PAD_PUS(0) |\
                                     IOMUXC_SW_PAD_CTL_PAD_HYS(1) )

/*====================== FLEXSPI Secondary IOMUXC Definitions ===========================*/
#define FLEXSPIA_SEC_MUX_VAL               IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(1)


#endif // __BOOTLOADER_COMMON_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
