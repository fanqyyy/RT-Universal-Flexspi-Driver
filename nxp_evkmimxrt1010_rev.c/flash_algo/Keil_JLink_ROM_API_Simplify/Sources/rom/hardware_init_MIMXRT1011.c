/*
 * Copyright 2017 NXP
 *
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootloader_common.h"
#include "fsl_assert.h"
#include "fsl_device_registers.h"
#include "bl_flexspi.h"
#include "flexspi_nor_flash.h"
#include "fusemap.h"
////////////////////////////////////////////////////////////////////////////////
/*******************************************************************************
 * Definitions
 ******************************************************************************/

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

/*====================== FLEXSPI Reset IOMUXC Definitions ===========================*/
#define SW_MUX_CTL_PAD_FLEXSPI_RESET_IDX    kIOMUXC_SW_MUX_CTL_PAD_GPIO_13
#define SW_PAD_CTL_PAD_FLEXSPI_RESET_IDX    kIOMUXC_SW_PAD_CTL_PAD_GPIO_13
#define FLEXSPI_RESET_PIN_MUX_VAL           IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(5)

#define FLEXSPI_RESET_PIN_GPIO              GPIO1
#define FLEXSPI_RESET_PIN_GPIO_SEL          (0) // 0 for GPIO1, 1 for GPIO2
#define FLEXSPI_RESET_PIN_INDEX             (13)

#define FLEXSPI_RESET_PIN_SW_PAD_CTRL_VAL                                                         \
    (IOMUXC_SW_PAD_CTL_PAD_DSE(6) | IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_PUE(0) | \
     IOMUXC_SW_PAD_CTL_PAD_PUS(0))

// GPIO for reset signals
#define FLEXSPI1A_RESET_GPIO_MUX_VAL    IOMUXC_SW_MUX_CTL_PAD_MUX_MODE(5)
#define FLEXSPI1A_RESET_GPIO            GPIO2
#define FLEXSPI1A_RESET_GPIO_SS0_B_PIN  (6)
#define FLEXSPI1A_RESET_GPIO_SCLK_PIN   (10)
#define FLEXSPI1A_RESET_GPIO_DATA0_PIN  (9)

#define GPIO_SW_PAD_CTL_VAL                                                                        \
    IOMUXC_SW_PAD_CTL_PAD_SRE(1) | IOMUXC_SW_PAD_CTL_PAD_DSE(1) | IOMUXC_SW_PAD_CTL_PAD_SPEED(0) | \
        IOMUXC_SW_PAD_CTL_PAD_PKE(1) | IOMUXC_SW_PAD_CTL_PAD_PUS(1)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
// static bool is_flexspi_2nd_bootpin(void);

/*******************************************************************************
 * Codes
 ******************************************************************************/
#if BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SPINAND_MODULE
// Set failsafe settings
status_t flexspi_set_failsafe_setting(flexspi_mem_config_t *config)
{
    status_t status = kStatus_InvalidArgument;
    do
    {
        if (config == NULL)
        {
            break;
        }
// This is an example that shows how to override the default pad setting in ROM, for now, the pad setting in ROM is
// idential to below values
// So, below codes are not required.
#if 0
        // See IOMUXC pad setting definitions for more details.
        config->controllerMiscOption |= (1<<kFlexSpiMiscOffset_PadSettingOverrideEnable);
        config->dqsPadSettingOverride = 0x130f1;
        config->sclkPadSettingOverride = 0x10f1;
        config->csPadSettingOverride = 0x10f1;
        config->dataPadSettingOverride = 0x10f1;
#endif
        if (config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad)
        {
            if (config->controllerMiscOption & (1 << kFlexSpiMiscOffset_DdrModeEnable))
            {
                config->dataValidTime[0].time_100ps = 15; // 1.5 ns // 1/4 * cycle of 166MHz DDR
            }
            else
            {
                if (config->dataValidTime[0].delay_cells < 1)
                {
                    config->dataValidTime[0].time_100ps = 30; // 3 ns // 1/2 * cycle of 166MHz DDR
                }
            }
        }
        status = kStatus_Success;

    } while (0);

    return status;
}
#endif // #if BL_FEATURE_FLEXSPI_NOR_MODULE || BL_FEATURE_SPINAND_MODULE

void dummy_byte_callback(uint8_t byte)
{
    (void)byte;
}

void debug_init(void) {}

#if __ICCARM__

size_t __write(int handle, const unsigned char *buf, size_t size)
{
    return size;
}

#endif // __ICCARM__


//!@brief Write FlexSPI persistent content
status_t flexspi_nor_write_persistent(const uint32_t data)
{
    SRC->GPR[2] = data;

    return kStatus_Success;
}
//!@brief Read FlexSPI persistent content
status_t flexspi_nor_read_persistent(uint32_t *data)
{
    *data = SRC->GPR[2];

    return kStatus_Success;
}

//!@brief Configure IOMUX for FlexSPI Peripheral
void flexspi_iomux_config(uint32_t instance, flexspi_mem_config_t *config)
{
    uint32_t csPadCtlValue = config->csPadSettingOverride ? config->csPadSettingOverride : FLEXSPI_SW_PAD_CTL_VAL;
    uint32_t dqsPadCtlValue = config->dqsPadSettingOverride ? config->dqsPadSettingOverride : FLEXSPI_DQS_SW_PAD_CTL_VAL;
    uint32_t sclkPadCtlValue = config->sclkPadSettingOverride ? config->sclkPadSettingOverride : FLEXSPI_SW_PAD_CTL_VAL;
    uint32_t dataPadCtlValue = config->dataPadSettingOverride ? config->dataPadSettingOverride : FLEXSPI_SW_PAD_CTL_VAL;

    if (instance == 0)
    {
        // The primary FlexSPI pinmux, support octal Flash and up to 4 QuadSPI NOR Flash
        {
            // Pinmux configuration for FLEXSPI1 PortA
            if (config->sflashA1Size || config->sflashA2Size)
            {
                if (config->sflashA2Size)
                {
                    // FLEXSPI1A_SS1_B
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_SS1_B_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_SS1_B_IDX] = csPadCtlValue;
                }

                // Basic pinmux configuration for FLEXSPI1
                if (config->sflashA1Size)
                {
                    // FLEXSPI1A_SS0_B
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_SS0_B_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_SS0_B_IDX] = csPadCtlValue;
                }

                // FLEXSPI1A_SCLK
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_SCLK_IDX] = \
                    FLEXSPI1_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_SCLK_IDX] = sclkPadCtlValue;

                // FLEXSPI1A_DATA0
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_DATA0_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_DATA0_IDX] = dataPadCtlValue;

                // FLEXSPI1A_DATA1
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_DATA1_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_DATA1_IDX] = dataPadCtlValue;

                // FLEXSPI1A_DATA2
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_DATA2_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_DATA2_IDX] = dataPadCtlValue;

                // FLEXSPI1A_DATA3
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_DATA3_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_DATA3_IDX] = dataPadCtlValue;

                if (config->sflashPadType == kSerialFlash_8Pads)
                {
                    // FLEXSPI1A_DATA4 / FLEXSPI1B_DATA0
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA0_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA0_IDX] = dataPadCtlValue;

                    // FLEXSPI1A_DATA5 / FLEXSPI1B_DATA1
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA1_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA1_IDX] = dataPadCtlValue;

                    // FLEXSPI1A_DATA6 / FLEXSPI1B_DATA2
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA2_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA2_IDX] = dataPadCtlValue;

                    // FLEXSPI1A_DATA7 / FLEXSPI1B_DATA3
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA3_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA3_IDX] = dataPadCtlValue;
                }

                // Configure DQS pad
                if ((config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad) ||
                    (config->readSampleClkSrc == kFlexSPIReadSampleClk_LoopbackFromDqsPad))
                {
                    if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_SecondDqsPinMux))
                    {
                        // FLEXSPI1A_SEC_DQS
                        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_SEC_DQS_IDX] = \
                            FLEXSPI1_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_SEC_DQS_IDX] = dqsPadCtlValue;
                        IOMUXC->SELECT_INPUT[SW_SELECT_INPUT_FLEXSPI1A_DQS_IDX] = kFLEXSPI1A_DQS_SRC_GPIO_SD_14;
                    }
                    else
                    {
                        // FLEXSPI1A_DQS
                        IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_DQS_IDX] = \
                            FLEXSPI1_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                        IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_DQS_IDX] = dqsPadCtlValue;
                        IOMUXC->SELECT_INPUT[SW_SELECT_INPUT_FLEXSPI1A_DQS_IDX] = kFLEXSPI1A_DQS_SRC_GPIO_SD_12;
                    }
                }

                // Configure Differential Clock pin
                if (flexspi_is_differential_clock_enable(config))
                {
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1A_SCLK_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1A_SCLK_IDX] = sclkPadCtlValue;
                }
            }

            // Pinmux configuration for FLEXSPI1 PortB
            if (config->sflashB1Size || config->sflashB2Size)
            {
                // Basic pinmux configuration for FLEXSPI1
                if (config->sflashB1Size)
                {
                    // FLEXSPI1B_SS0_B
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_SS0_B_IDX] = FLEXSPI1_MUX_VAL;
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_SS0_B_IDX] = csPadCtlValue;
                }

                // FLEXSPI1B_SCLK
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_SCLK_IDX] = \
                    FLEXSPI1_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_SCLK_IDX] = sclkPadCtlValue;

                // FLEXSPI1B_DATA0
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA0_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA0_IDX] = dataPadCtlValue;

                // FLEXSPI1B_DATA1
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA1_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA1_IDX] = dataPadCtlValue;

                // FLEXSPI1B_DATA2
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA2_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA2_IDX] = dataPadCtlValue;

                // FLEXSPI1B_DATA3
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DATA3_IDX] = FLEXSPI1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DATA3_IDX] = dataPadCtlValue;

                // Configure DQS pad
                if ((config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad) ||
                    (config->readSampleClkSrc == kFlexSPIReadSampleClk_LoopbackFromDqsPad))
                {
                    // FLEXSPI1B_DQS
                    IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPI1B_DQS_IDX] =
                        FLEXSPI1_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                    IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPI1B_DQS_IDX] = dqsPadCtlValue;
                }
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
