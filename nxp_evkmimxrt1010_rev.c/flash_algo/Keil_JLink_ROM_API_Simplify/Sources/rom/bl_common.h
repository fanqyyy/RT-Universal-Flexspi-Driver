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
#include "bootloader_config.h"
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

#define SystemCoreClock 528000000UL

#if defined(__CC_ARM)
#pragma anon_unions
#endif


/*====================== FLEXSPI1 IOMUXC Definitions ===========================*/
#define SW_MUX_CTL_PAD_FLEXSPI1B_DQS_IDX    kIOMUXC_SW_MUX_CTL_PAD_GPIO_00
#define SW_MUX_CTL_PAD_FLEXSPI1B_DATA3_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_04
#define SW_MUX_CTL_PAD_FLEXSPI1B_DATA2_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_02
#define SW_MUX_CTL_PAD_FLEXSPI1B_DATA1_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_01
#define SW_MUX_CTL_PAD_FLEXSPI1B_DATA0_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_03
#define SW_MUX_CTL_PAD_FLEXSPI1B_SS0_B_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_00
#define SW_MUX_CTL_PAD_FLEXSPI1B_SCLK_IDX   kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_13

#define SW_MUX_CTL_PAD_FLEXSPI1A_SEC_DQS_IDX kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_14
#define SW_MUX_CTL_PAD_FLEXSPI1A_DQS_IDX    kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_12
#define SW_MUX_CTL_PAD_FLEXSPI1A_SS0_B_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_06
#define SW_MUX_CTL_PAD_FLEXSPI1A_SS1_B_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_05
#define SW_MUX_CTL_PAD_FLEXSPI1A_SCLK_IDX   kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_10
#define SW_MUX_CTL_PAD_FLEXSPI1A_DATA0_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_09
#define SW_MUX_CTL_PAD_FLEXSPI1A_DATA1_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_07
#define SW_MUX_CTL_PAD_FLEXSPI1A_DATA2_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_08
#define SW_MUX_CTL_PAD_FLEXSPI1A_DATA3_IDX  kIOMUXC_SW_MUX_CTL_PAD_GPIO_SD_11

#define SW_PAD_CTL_PAD_FLEXSPI1B_DQS_IDX    kIOMUXC_SW_PAD_CTL_PAD_GPIO_00
#define SW_PAD_CTL_PAD_FLEXSPI1B_DATA3_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_04
#define SW_PAD_CTL_PAD_FLEXSPI1B_DATA2_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_02
#define SW_PAD_CTL_PAD_FLEXSPI1B_DATA1_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_01
#define SW_PAD_CTL_PAD_FLEXSPI1B_DATA0_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_03
#define SW_PAD_CTL_PAD_FLEXSPI1B_SS0_B_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_00
#define SW_PAD_CTL_PAD_FLEXSPI1B_SCLK_IDX   kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_13

#define SW_PAD_CTL_PAD_FLEXSPI1A_SEC_DQS_IDX    kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_14
#define SW_PAD_CTL_PAD_FLEXSPI1A_DQS_IDX    kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_12
#define SW_PAD_CTL_PAD_FLEXSPI1A_SS0_B_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_06
#define SW_PAD_CTL_PAD_FLEXSPI1A_SS1_B_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_05
#define SW_PAD_CTL_PAD_FLEXSPI1A_SCLK_IDX   kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_10
#define SW_PAD_CTL_PAD_FLEXSPI1A_DATA0_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_09
#define SW_PAD_CTL_PAD_FLEXSPI1A_DATA1_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_07
#define SW_PAD_CTL_PAD_FLEXSPI1A_DATA2_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_08
#define SW_PAD_CTL_PAD_FLEXSPI1A_DATA3_IDX  kIOMUXC_SW_PAD_CTL_PAD_GPIO_SD_11

#define SW_SELECT_INPUT_FLEXSPI1A_DQS_IDX   kIOMUXC_FLEXSPI_DQS_FA_SELECT_INPUT
#define kFLEXSPI1A_DQS_SRC_GPIO_SD_14 (0u)
#define kFLEXSPI1A_DQS_SRC_GPIO_SD_12 (1u)

#define FLEXSPI1_MUX_VAL IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(0)

// Fast Slew Rate
// Driver Strength: R0=260Ohm @3.3V, 150Ohm @1.8V, 240 Ohm for DDR, Actual R = R0/6
// Max Speed : 200MHz
// Pull enabled
// Keeper
#define FLEXSPI_SW_PAD_CTL_VAL                                                                      \
    (IOMUXC_SW_PAD_CTL_PAD_SRE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(6) | IOMUXC_SW_PAD_CTL_PAD_SPEED(3) | \
     IOMUXC_SW_PAD_CTL_PAD_PKE(0) | IOMUXC_SW_PAD_CTL_PAD_PUE(0) | IOMUXC_SW_PAD_CTL_PAD_PUS(0))

// Fast Slew Rate
// Driver Strength: R0=260Ohm @3.3V, 150Ohm @1.8V, 240 Ohm for DDR, Acutal R = R0/6
// Max Speed : 200MHz
// Pull enabled
// Pull
// 100k ohm pull down resistor
#define FLEXSPI_DQS_SW_PAD_CTL_VAL                                                                  \
    (IOMUXC_SW_PAD_CTL_PAD_SRE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(6) | IOMUXC_SW_PAD_CTL_PAD_SPEED(3) | \
     IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_PUE(1) | IOMUXC_SW_PAD_CTL_PAD_PUS(0) |   \
     IOMUXC_SW_PAD_CTL_PAD_HYS(1))

#endif // __BOOTLOADER_COMMON_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
