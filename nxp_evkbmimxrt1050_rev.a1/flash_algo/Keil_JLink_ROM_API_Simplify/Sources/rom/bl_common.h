/*
 * Copyright (c) 2013-2015 Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __BL_COMMON_H__
#define __BL_COMMON_H__

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
		 
/*====================== FLEXSPI Secondary IOMUXC Definitions ===========================*/
#define SW_MUX_CTL_PAD_FLEXSPIA_SEC_SS0_B_IDX          kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_15
#define SW_MUX_CTL_PAD_FLEXSPIA_SEC_SCLK_IDX           kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_14
#define SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA0_IDX          kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_13
#define SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA1_IDX          kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_12
#define SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA2_IDX          kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_11
#define SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA3_IDX          kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_10
#define SW_MUX_CTL_PAD_FLEXSPIA_SEC_DQS_IDX            kIOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_09

#define SW_PAD_CTL_PAD_FLEXSPIA_SEC_SS0_B_IDX          kIOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_15
#define SW_PAD_CTL_PAD_FLEXSPIA_SEC_SCLK_IDX           kIOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_14
#define SELECT_INPUT_FLEXSPIA_SEC_SCLK_IDX             kIOMUXC_FLEXSPIA_SCK_SELECT_INPUT
#define SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA0_IDX          kIOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_13
#define SELECT_INPUT_FLEXSPIA_SEC_DATA0_IDX            kIOMUXC_FLEXSPIA_DATA0_SELECT_INPUT
#define SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA1_IDX          kIOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_12
#define SELECT_INPUT_FLEXSPIA_SEC_DATA1_IDX            kIOMUXC_FLEXSPIA_DATA1_SELECT_INPUT
#define SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA2_IDX          kIOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_11
#define SELECT_INPUT_FLEXSPIA_SEC_DATA2_IDX            kIOMUXC_FLEXSPIA_DATA2_SELECT_INPUT
#define SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA3_IDX          kIOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_10
#define SELECT_INPUT_FLEXSPIA_SEC_DATA3_IDX            kIOMUXC_FLEXSPIA_DATA3_SELECT_INPUT
#define SW_PAD_CTL_PAD_FLEXSPIA_SEC_DQS_IDX            kIOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_09
#define SELECT_INPUT_FLEXSPIA_SEC_DQS_IDX              kIOMUXC_FLEXSPIA_DQS_SELECT_INPUT		 
	
#define FLEXSPIA_SEC_MUX_VAL               IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(0)

/*====================== FLEXSPI IOMUXC Definitions ===========================*/
#define SW_MUX_CTL_PAD_FLEXSPIB_DQS_IDX            111
#define SW_MUX_CTL_PAD_FLEXSPIB_DATA3_IDX          112
#define SW_MUX_CTL_PAD_FLEXSPIB_DATA2_IDX          113
#define SW_MUX_CTL_PAD_FLEXSPIB_DATA1_IDX          114
#define SW_MUX_CTL_PAD_FLEXSPIB_DATA0_IDX          115
#define SW_MUX_CTL_PAD_FLEXSPIB_SS0_B_IDX          110
#define SW_MUX_CTL_PAD_FLEXSPIB_SS1_B_IDX          107
#define SW_MUX_CTL_PAD_FLEXSPIB_SCLK_IDX           116

#define SW_MUX_CTL_PAD_FLEXSPIA_DQS_IDX            117
#define SW_MUX_CTL_PAD_FLEXSPIA_SS0_B_IDX          118
#define SW_MUX_CTL_PAD_FLEXSPIA_SS1_B_IDX          106
#define SW_MUX_CTL_PAD_FLEXSPIA_SCLK_IDX           119
#define SW_MUX_CTL_PAD_FLEXSPIA_DATA0_IDX          120
#define SW_MUX_CTL_PAD_FLEXSPIA_DATA1_IDX          121
#define SW_MUX_CTL_PAD_FLEXSPIA_DATA2_IDX          122
#define SW_MUX_CTL_PAD_FLEXSPIA_DATA3_IDX          123
#define SW_MUX_CTL_PAD_FLEXSPIA_SCLK_B_IDX         116

#define SW_PAD_CTL_PAD_FLEXSPIB_DQS_IDX            111
#define SW_PAD_CTL_PAD_FLEXSPIB_DATA3_IDX          112
#define SW_PAD_CTL_PAD_FLEXSPIB_DATA2_IDX          113
#define SW_PAD_CTL_PAD_FLEXSPIB_DATA1_IDX          114
#define SW_PAD_CTL_PAD_FLEXSPIB_DATA0_IDX          115
#define SW_PAD_CTL_PAD_FLEXSPIB_SS0_B_IDX          110
#define SW_PAD_CTL_PAD_FLEXSPIB_SS1_B_IDX          107
#define SW_PAD_CTL_PAD_FLEXSPIB_SCLK_IDX           116

#define SW_PAD_CTL_PAD_FLEXSPIA_DQS_IDX            117
#define SW_PAD_CTL_PAD_FLEXSPIA_SS0_B_IDX          118
#define SW_PAD_CTL_PAD_FLEXSPIA_SS1_B_IDX          106
#define SW_PAD_CTL_PAD_FLEXSPIA_SCLK_IDX           119
#define SW_PAD_CTL_PAD_FLEXSPIA_DATA0_IDX          120
#define SW_PAD_CTL_PAD_FLEXSPIA_DATA1_IDX          121
#define SW_PAD_CTL_PAD_FLEXSPIA_DATA2_IDX          122
#define SW_PAD_CTL_PAD_FLEXSPIA_DATA3_IDX          123
#define SW_PAD_CTL_PAD_FLEXSPIA_SCLK_B_IDX         116

#define FLEXSPIA_MUX_VAL               IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(1)
#define FLEXSPIB_MUX_VAL               IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(1)
#define FLEXSPIA_SS1_MUX_VAL           IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(6)
#define FLEXSPIB_SS1_MUX_VAL           IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(6)
#define FLEXSPIB_SS0_MUX_VAL           IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(4)
#define FLEXSPIB_DQS_MUX_VAL           IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(4)

// Fast Slew Rate
// Driver Strength: R0=260Ohm @3.3V, 150Ohm @1.8V, 240 Ohm for DDR, Actual R = R0/6
// Max Speed : 200MHz
// Pull enabled
// Keeper		 
#define FLEXSPI_SW_PAD_CTL_VAL                                                                      \
    (IOMUXC_SW_PAD_CTL_PAD_SRE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(6) | IOMUXC_SW_PAD_CTL_PAD_SPEED(3) | \
     IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_PUE(0) | IOMUXC_SW_PAD_CTL_PAD_PUS(0))

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


//ROM FUSEMAP
#define FUSE_BANK0_OFFSET         0x400
#define HW_FUSE_REG_ADDR(n)       (OCOTP_BASE + FUSE_BANK0_OFFSET + ((n) * 0x10))
#define HW_OCOTP_REG_RD(n)        (*(volatile uint32_t *)HW_FUSE_REG_ADDR(n))
/* Flash Type */
#define ROM_OCOTP_FLASH_TYPE_MASK   0x00000700
#define ROM_OCOTP_FLASH_TYPE_SHIFT  ((uint8_t)8)
#define ROM_OCOTP_FLASH_TYPE_VALUE()    \
        ((SRC->SBMR1&ROM_OCOTP_FLASH_TYPE_MASK) >> ROM_OCOTP_FLASH_TYPE_SHIFT)
				
/* QSPI 2ND pinmux */
#define ROM_OCOTP_QSPI_SIP_2ND_BOOT_PIN_ENABLE_SHIFT 20U
#define ROM_OCOTP_QSPI_SIP_2ND_BOOT_PIN_ENABLE_MASK (1U << ROM_OCOTP_QSPI_SIP_2ND_BOOT_PIN_ENABLE_SHIFT)
#define ROM_OCOTP_QSPI_SIP_2ND_BOOT_PIN_ENABLE_VALUE() \
            ((HW_OCOTP_REG_RD(4) & ROM_OCOTP_QSPI_SIP_2ND_BOOT_PIN_ENABLE_MASK) >> ROM_OCOTP_QSPI_SIP_2ND_BOOT_PIN_ENABLE_SHIFT)				

#endif // __BL_COMMON_H__
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
