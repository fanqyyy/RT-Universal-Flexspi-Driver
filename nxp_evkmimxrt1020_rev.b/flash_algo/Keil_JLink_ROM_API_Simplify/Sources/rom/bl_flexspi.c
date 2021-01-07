/*
 * Copyright 2014-2015 Freescale Semiconductor, Inc.
 * Copyright 2016-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
#include <assert.h>
#include <stdbool.h>

#include "bl_flexspi.h"
#include "MIMXRT1021.h"
#include "bl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define FLEXSPI_LUT_KEY_VAL (0x5AF05AF0ul)    //!< FlesSPI Unlock/Lock Key
#define FLEXSPI_WAIT_TIMEOUT_NS (500000000UL) //!< FlexSPI timeout value, 500ms
#define FLEXSPI_FREQ_1GHz (1000000000UL)

#define FLEXSPI_DLLCR_DEFAULT (0x100UL)

#define FLEXSPI0_CLK_GATE_OFFSET 13U
#define FLEXSPI1_CLK_GATE_OFFSET 15U

#define CMD_LUT_FOR_IP_CMD 1 //!< LUT sequence id for IP command

enum
{
    kFlexSpiDelayCellUnit_Min = 75,  // 75ps
    kFlexSpiDelayCellUnit_Max = 225, // 225ps
};

#define FLEXSPI_PINMUX_VAL 0x08

/*******************************************************************************
 * Local variables
 ******************************************************************************/
static FLEXSPI_Type *const g_flexSpiInstances[] = FLEXSPI_BASE_PTRS;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
//!@brief Reset internal logical of FlexSPI
static void flexspi_swreset(FLEXSPI_Type *base);
//!@brief Unlock LUT
static void flexspi_unlock_lut(FLEXSPI_Type *base);
//!@brief Lock LUT
static void flexspi_lock_lut(FLEXSPI_Type *base);
//!@brief Clear IP TXFIFO
static void flexspi_clear_ip_txfifo(FLEXSPI_Type *base);
//!@brief Clear IP RXFIO
static void flexspi_clear_ip_rxfifo(FLEXSPI_Type *base);
//!@brief Wait Until FlexSPI IP is idle
static void flexspi_wait_until_ip_idle(FLEXSPI_Type *base);
//!@brief Get FlexSPI instance
static FLEXSPI_Type *flexspi_get_module_base(uint32_t instance);
//!@brief Clear FlexSPI error status
static void flexspi_clear_error_status(FLEXSPI_Type *base);
//!@brief Get interval ticks based on provided interval in terms of nano-seconds, frequency and unit
status_t flexspi_get_ticks(uint32_t *ticks, uint32_t intervalNs, uint32_t freq, uint32_t unit);
#if FLEXSPI_FEATURE_HAS_PARALLEL_MODE
//!@brief Extract received data under parallel mode
static status_t flexspi_extract_parallel_data(uint32_t *dst0, uint32_t *dst1, uint32_t *src, uint32_t length);
#endif // FLEXSPI_FEATURE_HAS_PARALLEL_MODE

//!@brief Configure Device workmode via FlexSPI
static status_t flexspi_device_workmode_config(uint32_t instance, flexspi_mem_config_t *config, uint32_t baseAddr);
static status_t flexspi_device_workmode_config_all_chips(uint32_t instance, flexspi_mem_config_t *config);
//!@brief Configure Device registers via FlexSPI
static status_t flexspi_device_cmd_config(uint32_t instance, flexspi_mem_config_t *config, uint32_t baseAddr);
static status_t flexspi_device_cmd_config_all_chips(uint32_t instance, flexspi_mem_config_t *config);

/*******************************************************************************
 * Codes
 ******************************************************************************/
static void flexspi_swreset(FLEXSPI_Type *base)
{
    base->MCR0 |= FLEXSPI_MCR0_SWRESET_MASK;
    while (base->MCR0 & FLEXSPI_MCR0_SWRESET_MASK)
    {
    }
}

static void flexspi_unlock_lut(FLEXSPI_Type *base)
{
    base->LUTKEY = FLEXSPI_LUT_KEY_VAL;
    base->LUTCR = 0x02;
}

static void flexspi_lock_lut(FLEXSPI_Type *base)
{
    base->LUTKEY = FLEXSPI_LUT_KEY_VAL;
    base->LUTCR = 0x01;
}

bool flexspi_is_parallel_mode(flexspi_mem_config_t *config)
{
    if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_ParallelEnable))
    {
        return true;
    }
    else
    {
        return false;
    }
}

status_t flexspi_get_clock(uint32_t instance, flexspi_clock_type_t type, uint32_t *freq)
{
    uint32_t clockFrequency = 0;
    status_t status = kStatus_Success;

    uint32_t ahbBusDivider;
    uint32_t seralRootClkDivider;
    uint32_t arm_clock = SystemCoreClock;

    switch (type)
    {
        case kFlexSpiClock_CoreClock:
            clockFrequency = SystemCoreClock;
            break;
        case kFlexSpiClock_AhbClock:
        {
            // Note: In I.MXRT1020, actual AHB clock is IPG_CLOCK_ROOT
            ahbBusDivider = ((CCM->CBCDR & CCM_CBCDR_IPG_PODF_MASK) >> CCM_CBCDR_IPG_PODF_SHIFT) + 1;
            clockFrequency = arm_clock / ahbBusDivider;
        }
        break;
        case kFlexSpiClock_SerialRootClock:
        {
            uint32_t pfdFrac;
            uint32_t pfdClk;

            // FLEXPI CLK SEL
            uint32_t flexspi_clk_src =
                (CCM->CSCMR1 & CCM_CSCMR1_FLEXSPI_CLK_SEL_MASK) >> CCM_CSCMR1_FLEXSPI_CLK_SEL_SHIFT;

            // PLL_480_PFD0
            pfdFrac = (CCM_ANALOG->PFD_480 & CCM_ANALOG_PFD_480_PFD0_FRAC_MASK) >> CCM_ANALOG_PFD_480_PFD0_FRAC_SHIFT;
            pfdClk = FREQ_480MHz / pfdFrac * 18;

            seralRootClkDivider = ((CCM->CSCMR1 & CCM_CSCMR1_FLEXSPI_PODF_MASK) >> CCM_CSCMR1_FLEXSPI_PODF_SHIFT) + 1;

            clockFrequency = pfdClk / seralRootClkDivider;
        }
        break;
        default:
            status = kStatus_InvalidArgument;
            break;
    }
    *freq = clockFrequency;

    return status;
}

//!@brief Gate on the clock for the FlexSPI peripheral
void flexspi_clock_gate_enable(uint32_t instance)
{
    CCM->CCGR6 |= CCM_CCGR6_CG5_MASK;
    __ISB();
}

//!@brief Gate off the clock the FlexSPI peripheral
void flexspi_clock_gate_disable(uint32_t instance)
{
    CCM->CCGR6 &= (uint32_t)~CCM_CCGR6_CG5_MASK;
    __ISB();
}

//!@brief Configure IOMUX for FlexSPI Peripheral
void flexspi_iomux_config(uint32_t instance, flexspi_mem_config_t *config)
{
    uint32_t csPadCtlValue = FLEXSPI_SW_PAD_CTL_VAL;
    uint32_t dqsPadCtlValue = FLEXSPI_DQS_SW_PAD_CTL_VAL;
    uint32_t sclkPadCtlValue = FLEXSPI_SW_PAD_CTL_VAL;
    uint32_t dataPadCtlValue = FLEXSPI_SW_PAD_CTL_VAL;

    if (flexspi_is_padsetting_override_enable(config))
    {
        csPadCtlValue = config->csPadSettingOverride;
        dqsPadCtlValue = config->dqsPadSettingOverride;
        sclkPadCtlValue = config->sclkPadSettingOverride;
        dataPadCtlValue = config->dataPadSettingOverride;
    }

    if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_SecondPinMux))
    {
        // The secondary FlexSPI Pinmux, supports only 1 Flash
        if (config->sflashA1Size > 0)
        {
            // FLEXSPIA_SS0_B
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SEC_SS0_B_IDX] = FLEXSPIA_SEC_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SEC_SS0_B_IDX] = csPadCtlValue;
            // FLEXSPIA_SCLK
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SEC_SCLK_IDX] =
                FLEXSPIA_SEC_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SEC_SCLK_IDX] = sclkPadCtlValue;
            IOMUXC->SELECT_INPUT[SELECT_INPUT_FLEXSPIA_SEC_SCLK_IDX] = 0x01;

            // FLEXSPIA_DATA0
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA0_IDX] = FLEXSPIA_SEC_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA0_IDX] = dataPadCtlValue;
            IOMUXC->SELECT_INPUT[SELECT_INPUT_FLEXSPIA_SEC_DATA0_IDX] = 0x01;

            // FLEXSPIA_DATA1
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA1_IDX] = FLEXSPIA_SEC_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA1_IDX] = dataPadCtlValue;
            IOMUXC->SELECT_INPUT[SELECT_INPUT_FLEXSPIA_SEC_DATA1_IDX] = 0x01;

            // FLEXSPIA_DATA2
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA2_IDX] = FLEXSPIA_SEC_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA2_IDX] = dataPadCtlValue;
            IOMUXC->SELECT_INPUT[SELECT_INPUT_FLEXSPIA_SEC_DATA2_IDX] = 0x01;

            // FLEXSPIA_DATA3
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SEC_DATA3_IDX] = FLEXSPIA_SEC_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SEC_DATA3_IDX] = dataPadCtlValue;
            IOMUXC->SELECT_INPUT[SELECT_INPUT_FLEXSPIA_SEC_DATA3_IDX] = 0x01;
        }
    }
    else // The primary FlexSPI pinmux, support octal Flash and up to 4 QuadSPI NOR Flash
    {
        // Pinmux configuration for FLEXSPI PortA
        if (config->sflashA1Size || config->sflashA2Size)
        {
            if (config->sflashA2Size)
            {
                // FLEXSPIA_SS1_B
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SS1_B_IDX] = FLEXSPIA_SS1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SS1_B_IDX] = csPadCtlValue;
            }

            // Basic pinmux configuration for FLEXSPI
            if (config->sflashA1Size)
            {
                // FLEXSPIA_SS0_B
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SS0_B_IDX] = FLEXSPIA_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SS0_B_IDX] = csPadCtlValue;
            }

            // FLEXSPIA_SCLK
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SCLK_IDX] = FLEXSPIA_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SCLK_IDX] = sclkPadCtlValue;

            // FLEXSPIA_DATA0
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_DATA0_IDX] = FLEXSPIA_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_DATA0_IDX] = dataPadCtlValue;

            // FLEXSPIA_DATA1
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_DATA1_IDX] = FLEXSPIA_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_DATA1_IDX] = dataPadCtlValue;

            // FLEXSPIA_DATA2
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_DATA2_IDX] = FLEXSPIA_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_DATA2_IDX] = dataPadCtlValue;

            // FLEXSPIA_DATA3
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_DATA3_IDX] = FLEXSPIA_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_DATA3_IDX] = dataPadCtlValue;

            if (config->sflashPadType == kSerialFlash_8Pads)
            {
                // FLEXSPIA_DATA4 / FLEXSPIB_DATA0
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA0_IDX] = FLEXSPIA_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA0_IDX] = dataPadCtlValue;

                // FLEXSPIA_DATA5 / FLEXSPIB_DATA1
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA1_IDX] = FLEXSPIA_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA1_IDX] = dataPadCtlValue;

                // FLEXSPIA_DATA6 / FLEXSPIB_DATA2
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA2_IDX] = FLEXSPIA_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA2_IDX] = dataPadCtlValue;

                // FLEXSPIA_DATA7 / FLEXSPIB_DATA3
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA3_IDX] = FLEXSPIA_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA3_IDX] = dataPadCtlValue;
            }

            // Configure DQS pad
            if ((config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad) ||
                (config->readSampleClkSrc == kFlexSPIReadSampleClk_LoopbackFromDqsPad))
            {
                // FLEXSPIA_DQS
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_DQS_IDX] =
                    FLEXSPIA_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_DQS_IDX] = dqsPadCtlValue;
            }

            // Configure Differential Clock pin
            if (flexspi_is_differential_clock_enable(config))
            {
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIA_SCLK_B_IDX] = FLEXSPIA_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIA_SCLK_B_IDX] = sclkPadCtlValue;
            }
        }

        // Pinmux configuration for FLEXSPI PortB
        if (config->sflashB1Size || config->sflashB2Size)
        {
            if (config->sflashB2Size)
            {
                // FLEXSPIB_SS1_B
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_SS1_B_IDX] = FLEXSPIB_SS1_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_SS1_B_IDX] = csPadCtlValue;
            }

            // Basic pinmux configuration for FLEXSPI
            if (config->sflashB1Size)
            {
                // FLEXSPIB_SS0_B
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_SS0_B_IDX] = FLEXSPIB_SS0_MUX_VAL;
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_SS0_B_IDX] = csPadCtlValue;
            }

            // FLEXSPIB_SCLK
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_SCLK_IDX] = FLEXSPIB_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_SCLK_IDX] = sclkPadCtlValue;

            // FLEXSPIB_DATA0
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA0_IDX] = FLEXSPIB_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA0_IDX] = dataPadCtlValue;

            // FLEXSPIB_DATA1
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA1_IDX] = FLEXSPIB_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA1_IDX] = dataPadCtlValue;

            // FLEXSPIB_DATA2
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA2_IDX] = FLEXSPIB_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA2_IDX] = dataPadCtlValue;

            // FLEXSPIB_DATA3
            IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DATA3_IDX] = FLEXSPIB_MUX_VAL;
            IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DATA3_IDX] = dataPadCtlValue;

            // Configure DQS pad
            if ((config->readSampleClkSrc == kFlexSPIReadSampleClk_ExternalInputFromDqsPad) ||
                (config->readSampleClkSrc == kFlexSPIReadSampleClk_LoopbackFromDqsPad))
            {
                // FLEXSPIB_DQS
                IOMUXC->SW_MUX_CTL_PAD[SW_MUX_CTL_PAD_FLEXSPIB_DQS_IDX] =
                    FLEXSPIB_DQS_MUX_VAL | IOMUXC_SW_MUX_CTL_PAD_SION(1);
                IOMUXC->SW_PAD_CTL_PAD[SW_PAD_CTL_PAD_FLEXSPIB_DQS_IDX] = dqsPadCtlValue;
            }
        }
    }
}

void flexspi_clock_config(uint32_t instance, uint32_t freq, uint32_t sampleClkMode)
{
    uint32_t pfd480 = 0;
    uint32_t cscmr1 = 0;
    uint32_t frac = 0;
    uint32_t podf = 0;

    typedef struct _flexspi_clock_param
    {
        uint8_t frac;
        uint8_t podf;
    } flexspi_clock_param_t;

    const flexspi_clock_param_t k_sdr_clock_config[kFlexSpiSerialClk_200MHz + 1] = {
        // Reserved, 30MHz   50MHz      60MHz      75MHz       80MHz      100MHz     133MHz     166MHz     200MHz
        { 0, 0 }, { 34, 8 }, { 22, 8 }, { 24, 6 }, { 30, 4 }, { 18, 6 }, { 14, 6 }, { 13, 5 }, { 13, 4 }, { 22, 2 }
    };
    const flexspi_clock_param_t k_ddr_clock_config[kFlexSpiSerialClk_200MHz + 1] = {
        // Reserved, 30MHz,     50MHz,   60MHz,      75MHz,     80Mhz,    100MHz,    133MHz,     166MHz,  200MHz
        { 0, 0 }, { 24, 6 }, { 22, 4 }, { 12, 6 }, { 30, 2 }, { 18, 3 }, { 22, 2 }, { 33, 1 }, { 13, 2 }, { 22, 1 }
    };

    do
    {
        if ((sampleClkMode != kFlexSpiClk_SDR) && (sampleClkMode != kFlexSpiClk_DDR))
        {
            break;
        }

        pfd480 = CCM_ANALOG->PFD_480 & (~CCM_ANALOG_PFD_480_PFD0_FRAC_MASK);
        cscmr1 = CCM->CSCMR1 & (~CCM_CSCMR1_FLEXSPI_PODF_MASK);

        // Note: Per ANALOG IP Owner's recommendation, FRAC should be even number,
        //       PODF should be even nubmer as well if the divider is greater than 1

        const flexspi_clock_param_t *flexspi_config_array = NULL;
        if (sampleClkMode == kFlexSpiClk_SDR)
        {
            flexspi_config_array = &k_sdr_clock_config[0];
        }
        else
        {
            flexspi_config_array = &k_ddr_clock_config[0];
        }

        if (freq >= kFlexSpiSerialClk_30MHz)
        {
            if (freq > kFlexSpiSerialClk_200MHz)
            {
                freq = kFlexSpiSerialClk_30MHz;
            }

            frac = flexspi_config_array[freq].frac;
            podf = flexspi_config_array[freq].podf;

            pfd480 |= CCM_ANALOG_PFD_480_PFD0_FRAC(frac);
            cscmr1 |= CCM_CSCMR1_FLEXSPI_PODF(podf - 1);

            FLEXSPI->MCR0 |= FLEXSPI_MCR0_MDIS_MASK;
            flexspi_clock_gate_disable(instance);

            if (pfd480 != CCM_ANALOG->PFD_480)
            {
                CCM_ANALOG->PFD_480 = pfd480;
            }
            if (cscmr1 != CCM->CSCMR1)
            {
                CCM->CSCMR1 = cscmr1;
            }
            flexspi_clock_gate_enable(instance);
            FLEXSPI->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;
            __ISB();
        }
        else
        {
            // Do nothing
        }
    } while (0);
}

bool flexspi_is_padsetting_override_enable(flexspi_mem_config_t *config)
{
    if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_PadSettingOverrideEnable))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool flexspi_is_differential_clock_enable(flexspi_mem_config_t *config)
{
    if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_DiffClkEnable))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool flexspi_is_word_addressable(flexspi_mem_config_t *config)
{
    if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_WordAddressableEnable))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool flexspi_is_ck2_enabled(flexspi_mem_config_t *config)
{
    if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_Ck2Enable))
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool flexspi_is_ddr_mode_enable(flexspi_mem_config_t *config)
{
    if (config->controllerMiscOption & FLEXSPI_BITMASK(kFlexSpiMiscOffset_DdrModeEnable))
    {
        return true;
    }
    else
    {
        return false;
    }
}

//!@brief Configure DLL register
status_t flexspi_configure_dll(uint32_t instance, flexspi_mem_config_t *config)
{
    status_t status = kStatus_InvalidArgument;
    bool mdisConfigRequired;

    do
    {
        FLEXSPI_Type *base = flexspi_get_module_base(instance);
        bool isUnifiedConfig = true;
        uint32_t flexspiRootClk;
        uint32_t flexspiDll[2] = { FLEXSPI_DLLCR_DEFAULT, FLEXSPI_DLLCR_DEFAULT };
        uint32_t dllValue;
        uint32_t temp;

        if ((base == NULL) || (config == NULL) ||
            (config->readSampleClkSrc > kFlexSPIReadSampleClk_ExternalInputFromDqsPad))
        {
            break;
        }

        switch (config->readSampleClkSrc)
        {
            case kFlexSPIReadSampleClk_LoopbackInternally:
            case kFlexSPIReadSampleClk_LoopbackFromDqsPad:
            case kFlexSPIReadSampleClk_LoopbackFromSckPad:
                isUnifiedConfig = true;
                break;
            case kFlexSPIReadSampleClk_ExternalInputFromDqsPad:
                if (flexspi_is_ck2_enabled(config))
                {
                    isUnifiedConfig = true;
                }
                else
                {
                    isUnifiedConfig = false;
                }
                break;
            default: // Never reach here
                break;
        }

        if (isUnifiedConfig)
        {
            flexspiDll[0] = FLEXSPI_DLLCR_DEFAULT; // 1 fixed delay cells in DLL delay chain)
            flexspiDll[1] = FLEXSPI_DLLCR_DEFAULT; // 1 fixed delay cells in DLL delay chain)
        }
#if FLEXSPI_ENABLE_OCTAL_FLASH_SUPPORT
        else
        {
            flexspi_get_clock(instance, kFlexSpiClock_SerialRootClock, &flexspiRootClk);

            bool useDLL = false;

            // See FlexSPI Chapter for more details
            if ((flexspiRootClk >= 100 * FREQ_1MHz) &&
                (!(config->controllerMiscOption & (1U << kFlexSpiMiscOffset_UseValidTimeForAllFreq))))
            {
                useDLL = true;
            }
            if (useDLL)
            {
                flexspiDll[0] = FLEXSPI_DLLCR_DLLEN(1) | FLEXSPI_DLLCR_SLVDLYTARGET(0x0F);
                flexspiDll[1] = FLEXSPI_DLLCR_DLLEN(1) | FLEXSPI_DLLCR_SLVDLYTARGET(0x0F);
            }
            // If Serial root closk is lower than 100MHz, DLL is unable to lock on
            // half cycle of serial root clock because the dealy cell number is limited
            // in delay chain, Then DLL should be configured as following instead:
            // OVRDEN = 0x01
            // OVRDVAL=N; each dealy cell in DLL is about 75ps - 225ps.
            // The delay of DLL delay chain ( N * delay_cell_delay) should be larger
            // than device output data valid time (from SCK edge to data valid).
            // The other condition is that some devices may be incompatible with current
            // FlexSPI defintions, so, use a backup way to support it.
            else
            {
                for (uint32_t i = 0; i < 2; i++)
                {
                    uint32_t dataValidTimeH = config->dataValidTime[i].delay_cells;
                    uint32_t dataValidTimeL = config->dataValidTime[i].time_100ps;
                    if (dataValidTimeH < 1)
                    {
                        // Convert the data valid time to n ps.
                        temp = dataValidTimeL * 100ul;
                        if (temp < 1)
                        {
                            uint32_t maxFreq;
                            bool is_ddr_enabled = flexspi_is_ddr_mode_enable(config);
                            flexspi_get_max_supported_freq(instance, &maxFreq, is_ddr_enabled);
                            // For SDR mode, the delay cell configuration must ensure that the delay time is greater
                            // than
                            // Half cycle of max  supported frequency
                            if (!is_ddr_enabled)
                            {
                                dllValue = FLEXSPI_FREQ_1GHz / maxFreq / 2 * 1000 / kFlexSpiDelayCellUnit_Min + 1;
                            }
                            // For SDR mode, the delay cell configuration must ensure that the delay time is greater
                            // than
                            // 1/4 cycle of max supported frequency
                            else
                            {
                                dllValue = FLEXSPI_FREQ_1GHz / maxFreq / 4 * 1000 / kFlexSpiDelayCellUnit_Min + 1;
                            }
                        }
                        else
                        {
                            dllValue = temp / kFlexSpiDelayCellUnit_Min;
                            if (dllValue * kFlexSpiDelayCellUnit_Min < temp)
                            {
                                dllValue++;
                            }
                        }
                    }
                    else
                    {
                        dllValue = dataValidTimeH;
                    }
                    // Calculate maximum dll value;
                    temp = (FLEXSPI_DLLCR_OVRDVAL_MASK >> FLEXSPI_DLLCR_OVRDVAL_SHIFT);
                    if (dllValue > temp)
                    {
                        dllValue = temp;
                    }
                    flexspiDll[i] = FLEXSPI_DLLCR_OVRDEN(1) | FLEXSPI_DLLCR_OVRDVAL(dllValue);
                }
            }
        }
#endif // FLEXSPI_ENABLE_OCTAL_FLASH_SUPPORT

        if (base->MCR0 & FLEXSPI_MCR0_MDIS_MASK)
        {
            mdisConfigRequired = false;
        }
        else
        {
            mdisConfigRequired = true;
        }

        if (mdisConfigRequired)
        {
            base->MCR0 |= FLEXSPI_MCR0_MDIS_MASK;
        }

        if (config->sflashA1Size || config->sflashA2Size)
        {
            base->DLLCR[0] = flexspiDll[0];
        }
        if (config->sflashB1Size || config->sflashB2Size)
        {
            base->DLLCR[1] = flexspiDll[1];
        }

        if (mdisConfigRequired)
        {
            base->MCR0 &= (uint32_t)~FLEXSPI_MCR0_MDIS_MASK;
        }

        status = kStatus_Success;
    } while (0);

    return status;
}

// Calculate ticks for the timeout interms of specified unit.
status_t flexspi_get_ticks(uint32_t *ticks, uint32_t intervalNs, uint32_t freq, uint32_t unit)
{
    status_t status = kStatus_InvalidArgument;
    do
    {
        if ((ticks == NULL) || (freq < 1) || (unit < 1))
        {
            break;
        }

        // Get clock cycle in terms of ns
        int32_t calculatedTicks;
        uint32_t cycleNs = FLEXSPI_FREQ_1GHz / freq;

        calculatedTicks = intervalNs / (cycleNs * unit);
        while (calculatedTicks * cycleNs * unit < intervalNs)
        {
            calculatedTicks++;
        }

        *ticks = calculatedTicks;

        status = kStatus_Success;

    } while (0);

    return status;
}

status_t flexspi_config_mcr1(uint32_t instance, flexspi_mem_config_t *config)
{
    uint32_t seqWaitTicks = 0xFFFFu;
    uint32_t ahbBusWaitTicks = 0xFFFFu;
    uint32_t serialRootClockFreq;
    uint32_t ahbBusClockFreq;
    FLEXSPI_Type *base = flexspi_get_module_base(instance);

    if ((base == NULL) || (config == NULL))
    {
        return kStatus_InvalidArgument;
    }

    flexspi_get_clock(instance, kFlexSpiClock_SerialRootClock, &serialRootClockFreq);
    flexspi_get_clock(instance, kFlexSpiClock_AhbClock, &ahbBusClockFreq);
    flexspi_get_ticks(&seqWaitTicks, FLEXSPI_WAIT_TIMEOUT_NS, serialRootClockFreq, 1024);
    flexspi_get_ticks(&ahbBusWaitTicks, FLEXSPI_WAIT_TIMEOUT_NS, ahbBusClockFreq, 1024);

    if (seqWaitTicks > 0xFFFF)
    {
        seqWaitTicks = 0xFFFF;
    }
    if (ahbBusWaitTicks > 0xFFFF)
    {
        ahbBusWaitTicks = 0xFFFF;
    }

    // Configure MCR1
    base->MCR1 = FLEXSPI_MCR1_SEQWAIT(seqWaitTicks) | FLEXSPI_MCR1_AHBBUSWAIT(ahbBusWaitTicks);

    return kStatus_Success;
}

static void flexspi_clear_ip_txfifo(FLEXSPI_Type *base)
{
    base->IPTXFCR |= FLEXSPI_IPTXFCR_CLRIPTXF_MASK;
    while (base->IPTXFCR & FLEXSPI_IPTXFCR_CLRIPTXF_MASK)
    {
    }
}

static void flexspi_clear_ip_rxfifo(FLEXSPI_Type *base)
{
    base->IPRXFCR |= FLEXSPI_IPRXFCR_CLRIPRXF_MASK;
    while (base->IPRXFCR & FLEXSPI_IPRXFCR_CLRIPRXF_MASK)
    {
    }
}

static void flexspi_wait_until_ip_idle(FLEXSPI_Type *base)
{
    while ((base->STS0 & FLEXSPI_STS0_SEQIDLE_MASK) != FLEXSPI_STS0_SEQIDLE_MASK)
    {
    }
    while ((base->STS0 & FLEXSPI_STS0_ARBIDLE_MASK) != FLEXSPI_STS0_ARBIDLE_MASK)
    {
    }
}

static FLEXSPI_Type *flexspi_get_module_base(uint32_t instance)
{
    FLEXSPI_Type *baseAddr = NULL;
    if (instance < sizeof(g_flexSpiInstances) / sizeof(g_flexSpiInstances[0]))
    {
        baseAddr = g_flexSpiInstances[instance];
    }
    return baseAddr;
}

void flexspi_clear_error_status(FLEXSPI_Type *base)
{
    base->INTR |= FLEXSPI_INTR_AHBCMDERR_MASK | FLEXSPI_INTR_IPCMDERR_MASK | FLEXSPI_INTR_AHBCMDGE_MASK |
                  FLEXSPI_INTR_IPCMDGE_MASK;
}

status_t flexspi_config_flash_control_registers(uint32_t instance, flexspi_mem_config_t *config)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        uint32_t index;
        uint32_t flashSize;
        uint32_t temp;
        uint32_t serialClockFrequency;
        uint32_t csIntervalTicks = 0;

        FLEXSPI_Type *base = flexspi_get_module_base(instance);
        if ((base == NULL) || (config == NULL))
        {
            break;
        }

        uint32_t *flashSizeStart = (uint32_t *)&config->sflashA1Size;

        for (index = 0; index < FLEXSPI_FLSHCR0_COUNT; index++)
        {
            // Configure FLSHCR0
            flashSize = *flashSizeStart++;
            base->FLSHCR0[index] = flashSize / 1024;

            // Configure FLSHCR1
            temp = FLEXSPI_FLSHCR1_TCSS(config->csSetupTime) | FLEXSPI_FLSHCR1_TCSH(config->csHoldTime) |
                   FLEXSPI_FLSHCR1_CAS(config->columnAddressWidth);
            if (flexspi_is_word_addressable(config))
            {
                temp |= FLEXSPI_FLSHCR1_WA_MASK;
            }
            // Calculate CS interval
            if (config->commandInterval)
            {
                flexspi_get_clock(instance, kFlexSpiClock_SerialRootClock, &serialClockFrequency);
                flexspi_get_ticks(&csIntervalTicks, config->commandInterval, serialClockFrequency, 1);

                temp |= FLEXSPI_FLSHCR1_CSINTERVAL(csIntervalTicks);
            }
            base->FLSHCR1[index] = temp;

            // Configure FLSHCR2
            temp = FLEXSPI_FLSHCR2_ARDSEQID(CMD_LUT_SEQ_IDX_READ);

            if (config->lutCustomSeqEnable && FLEXSPI_FLSHCR2_ARDSEQNUM(config->lutCustomSeq[CMD_INDEX_WRITE].seqNum))
            {
                temp |= FLEXSPI_FLSHCR2_AWRSEQID(config->lutCustomSeq[CMD_INDEX_WRITE].seqId);
                temp |= FLEXSPI_FLSHCR2_AWRSEQNUM(config->lutCustomSeq[CMD_INDEX_WRITE].seqNum - 1);
            }
            else
            {
                temp |= FLEXSPI_FLSHCR2_AWRSEQID(CMD_LUT_SEQ_IDX_WRITE);
            }
            base->FLSHCR2[index] = temp;
        }

        status = kStatus_Success;

    } while (0);

    return status;
}

status_t flexspi_config_ahb_buffers(FLEXSPI_Type *base, flexspi_mem_config_t *config)
{
    uint32_t temp;
    uint32_t index;
    status_t status = kStatus_InvalidArgument;

    do
    {
        if ((base == NULL) || (config == NULL))
        {
            break;
        }

        if (config->deviceType == kFlexSpiDeviceType_SerialNOR)
        {
            // Configure AHBCR
            temp = base->AHBCR & (~FLEXSPI_AHBCR_APAREN_MASK);
            // Remove alignment limitation when Flash device works under DDR mode.
            temp |= FLEXSPI_AHBCR_READADDROPT_MASK;
#if FLEXSPI_FEATURE_HAS_PARALLEL_MODE
            if (flexspi_is_parallel_mode(config))
            {
                temp |= FLEXSPI_AHBCR_APAREN_MASK;
            }
#endif // FLEXSPI_FEATURE_HAS_PARALLEL_MODE
            base->AHBCR = temp;
        }
        // Enable prefetch feature
        base->AHBCR |= FLEXSPI_AHBCR_PREFETCHEN_MASK;

        // Configure AHB RX buffer
        for (index = 0; index < FLEXSPI_AHBRXBUFCR0_COUNT - 1; index++)
        {
            base->AHBRXBUFCR0[index] &=
                ~(FLEXSPI_AHBRXBUFCR0_BUFSZ_MASK | FLEXSPI_AHBRXBUFCR0_MSTRID_MASK | FLEXSPI_AHBRXBUFCR0_PRIORITY_MASK);
        }
        status = kStatus_Success;

    } while (0);

    return status;
}

status_t flexspi_device_write_enable(uint32_t instance,
                                     flexspi_mem_config_t *config,
                                     bool isParallelMode,
                                     uint32_t baseAddr)
{
    status_t status = kStatus_InvalidArgument;
    do
    {
        if (config == NULL)
        {
            break;
        }
        flexspi_xfer_t flashXfer;
        flashXfer.operation = kFlexSpiOperation_Command;
        flashXfer.seqNum = 1;
        flashXfer.seqId = CMD_LUT_SEQ_IDX_WRITEENABLE;
#if FLEXSPI_FEATURE_HAS_PARALLEL_MODE
        flashXfer.isParallelModeEnable = isParallelMode;
#else
        flashXfer.isParallelModeEnable = false;
#endif // FLEXSPI_FEATURE_HAS_PARALLEL_MODE
        flashXfer.baseAddress = baseAddr;

        if (config->lutCustomSeqEnable && config->lutCustomSeq[CMD_INDEX_WRITEENABLE].seqNum)
        {
            flashXfer.seqId = config->lutCustomSeq[CMD_INDEX_WRITEENABLE].seqId;
            flashXfer.seqNum = config->lutCustomSeq[CMD_INDEX_WRITEENABLE].seqNum;
        }
        flexspi_update_lut(instance, CMD_LUT_FOR_IP_CMD, &config->lookupTable[4 * flashXfer.seqId], flashXfer.seqNum);
        flashXfer.seqId = CMD_LUT_FOR_IP_CMD;
        status = flexspi_command_xfer(instance, &flashXfer);

    } while (0);

    return status;
}

status_t flexspi_device_workmode_config(uint32_t instance, flexspi_mem_config_t *config, uint32_t baseAddr)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        FLEXSPI_Type *base = flexspi_get_module_base(instance);
        if ((base == NULL) || (config == NULL))
        {
            break;
        }

        // If device is working under DPI/QPI/OPI mode, ignore SPI2XPI command
        uint32_t read_cmd_pads = (base->LUT[0] >> 8) & 0x03;
        if ((read_cmd_pads > FLEXSPI_1PAD) && (config->deviceModeType == kDeviceConfigCmdType_Spi2Xpi))
        {
            status = kStatus_Success;
            break;
        }

        flexspi_xfer_t flashXfer;
        flashXfer.operation = kFlexSpiOperation_Config;
        flashXfer.baseAddress = baseAddr;
        flashXfer.seqId = config->deviceModeSeq.seqId;
        flashXfer.seqNum = config->deviceModeSeq.seqNum;
        flashXfer.isParallelModeEnable = false;
        flashXfer.txBuffer = &config->deviceModeArg;
        flashXfer.txSize = 4;

        flexspi_device_write_enable(instance, config, false, baseAddr);
        // Update LUT 1 for device mode config command
        flexspi_update_lut(instance, CMD_LUT_FOR_IP_CMD, &config->lookupTable[4 * flashXfer.seqId], flashXfer.seqNum);
        flashXfer.seqId = CMD_LUT_FOR_IP_CMD;
        status = flexspi_command_xfer(instance, &flashXfer);
        if (status != kStatus_Success)
        {
            break;
        }
        if ((!config->waitTimeCfgCommands) && (config->deviceModeType != (uint8_t)kDeviceConfigCmdType_Spi2Xpi) &&
            (config->deviceModeType != (uint8_t)kDeviceConfigCmdType_Xpi2Spi))
        {
            status = flexspi_device_wait_busy(instance, config, false, baseAddr);
        }
        else
        {
            flexspi_sw_delay_us(config->waitTimeCfgCommands * 100UL);
        }
    } while (0);

    return status;
}
status_t flexspi_device_workmode_config_all_chips(uint32_t instance, flexspi_mem_config_t *config)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        if (config == NULL)
        {
            break;
        }
        uint32_t baseAddr = 0;
        uint32_t index;
        uint32_t currentFlashSize;
        uint32_t *flashSizeStart = &config->sflashA1Size;

        for (index = 0; index < 4; index++)
        {
            currentFlashSize = *flashSizeStart++;
            if (currentFlashSize > 0)
            {
                status = flexspi_device_workmode_config(instance, config, baseAddr);
                if (status != kStatus_Success)
                {
                    break;
                }
                baseAddr += currentFlashSize;
            }
        }

    } while (0);

    return status;
}

status_t flexspi_device_cmd_config(uint32_t instance, flexspi_mem_config_t *config, uint32_t baseAddr)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        FLEXSPI_Type *base = flexspi_get_module_base(instance);
        if ((base == NULL) || (config == NULL))
        {
            break;
        }

        flexspi_xfer_t flashXfer;
        uint32_t index;
        flashXfer.operation = kFlexSpiOperation_Config;
        flashXfer.baseAddress = baseAddr;
        flashXfer.isParallelModeEnable = false;
        flashXfer.txSize = 4;

        for (index = 0; index < 3; index++)
        {
            if (config->configCmdSeqs[index].seqId > 0)
            {
                // If device is working under DPI/QPI/OPI mode, ignore SPI2XPI command
                uint32_t read_cmd_pads = (base->LUT[0] >> 8) & 0x03;
                if ((read_cmd_pads > FLEXSPI_1PAD) && (config->configModeType[index] == kDeviceConfigCmdType_Spi2Xpi))
                {
                    continue;
                }

                flashXfer.seqId = config->configCmdSeqs[index].seqId;
                flashXfer.seqNum = config->configCmdSeqs[index].seqNum;
                flashXfer.txBuffer = &config->configCmdArgs[index];

                status = flexspi_device_write_enable(instance, config, false, baseAddr);
                if (status != kStatus_Success)
                {
                    return status;
                }

                flexspi_update_lut(instance, CMD_LUT_FOR_IP_CMD, &config->lookupTable[4 * flashXfer.seqId],
                                   flashXfer.seqNum);
                flashXfer.seqId = CMD_LUT_FOR_IP_CMD;
                status = flexspi_command_xfer(instance, &flashXfer);
                if (status != kStatus_Success)
                {
                    return status;
                }

                if ((!config->waitTimeCfgCommands) &&
                    (config->configModeType[index] != (uint8_t)kDeviceConfigCmdType_Spi2Xpi) &&
                    (config->configModeType[index] != (uint8_t)kDeviceConfigCmdType_Xpi2Spi))
                {
                    status = flexspi_device_wait_busy(instance, config, false, baseAddr);

                    if (status != kStatus_Success)
                    {
                        return status;
                    }
                }
                else
                {
                    flexspi_sw_delay_us(config->waitTimeCfgCommands * 100UL);
                }
            }
        }
    } while (0);

    return status;
}

status_t flexspi_device_cmd_config_all_chips(uint32_t instance, flexspi_mem_config_t *config)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        if (config == NULL)
        {
            break;
        }
        uint32_t baseAddr = 0;
        uint32_t index;
        uint32_t currentFlashSize;
        uint32_t *flashSizeStart = &config->sflashA1Size;

        for (index = 0; index < 4; index++)
        {
            currentFlashSize = *flashSizeStart++;
            if (currentFlashSize > 0)
            {
                status = flexspi_device_cmd_config(instance, config, baseAddr);
                if (status != kStatus_Success)
                {
                    break;
                }
                baseAddr += currentFlashSize;
            }
        }
    } while (0);

    return status;
}

status_t flexspi_init(uint32_t instance, flexspi_mem_config_t *config)
{
    uint32_t mcr0;
    status_t status = kStatus_InvalidArgument;

    do
    {
        FLEXSPI_Type *base = flexspi_get_module_base(instance);

        if ((base == NULL) || (config == NULL))
        {
            break;
        }

        /* Determine if the flexspi serial clock should be configured to safe frequency
         *
         *  ROM configures the FlexSPI Serial clock to 30MHz if one of below case is met:
         *  1. ROM is required to configure external device to certain modes at safe frequency
         */
        bool need_safe_freq = (config->deviceModeCfgEnable || config->configCmdEnable) &&
                              ((config->controllerMiscOption & (1 << kFlexSpiMiscOffset_SafeConfigFreqEnable)));

        if (config->tag != FLEXSPI_CFG_BLK_TAG)
        {
            break;
        }

        /*
         *  !!! Important !!!
         *  The module clock must be disabled during clock switch in order to avoid glitch
         */
        flexspi_clock_gate_disable(instance);
        flexspi_iomux_config(instance, config);
        if (need_safe_freq)
        {
            // Configure FlexSPI serial clock using safe frequency
            flexspi_clock_config(instance, kFlexSpiSerialClk_SafeFreq, kFlexSpiClk_SDR);
        }
        else
        {
            // Configure FlexSPI serial clock with specified frequency
            flexspi_clock_config(instance, config->serialClkFreq, flexspi_is_ddr_mode_enable(config));
        }
        // Enable FlexSPI Clock Gate
        flexspi_clock_gate_enable(instance);

        base->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;
        flexspi_swreset(base);

        // Disable FlexSPI module during configuring control registers.
        base->MCR0 |= FLEXSPI_MCR0_MDIS_MASK;

        // Set Clock divider and sample clock source
        mcr0 = base->MCR0 & (uint32_t) ~(FLEXSPI_MCR0_RXCLKSRC_MASK | FLEXSPI_MCR0_IPGRANTWAIT_MASK |
                                         FLEXSPI_MCR0_AHBGRANTWAIT_MASK | FLEXSPI_MCR0_COMBINATIONEN_MASK
#if !defined(MIMXRT685S_cm33_SERIES)
                                         | FLEXSPI_MCR0_ATDFEN_MASK | FLEXSPI_MCR0_ARDFEN_MASK
#endif
                            );

#if FLEXSPI_ENABLE_NO_CMD_MODE_SUPPORT
        // If this condition meets, it means FlexSPI PORT B exists, the the 8 bit is supported by combining PORTA[3:0]
        // with PORTB[3:0]
        if ((sizeof(base->FLSHCR1) / sizeof(base->FLSHCR1[0])) > 2)
        {
            // Enable Combined mode if Serial FLASH works using Octal pad instructions.
            if (config->sflashPadType == kSerialFlash_8Pads)
            {
                mcr0 |= FLEXSPI_MCR0_COMBINATIONEN_MASK;
            }
        }
#endif

        // Configure AHBGRANTWAIT and IPGRANTWAIT
        mcr0 |= FLEXSPI_MCR0_IPGRANTWAIT_MASK | FLEXSPI_MCR0_AHBGRANTWAIT_MASK;
        // Configure Read sample clock source
        mcr0 |= FLEXSPI_MCR0_RXCLKSRC(config->readSampleClkSrc);
        base->MCR0 = mcr0;

#if FLEXSPI_ENABLE_NO_CMD_MODE_SUPPORT
        // Configure MCR1
        flexspi_config_mcr1(instance, config);
#endif

        // Configure MCR2
        base->MCR2 &= ~FLEXSPI_MCR2_SAMEDEVICEEN_MASK;

#if FLEXSPI_ENABLE_NO_CMD_MODE_SUPPORT
        // If this condition meets, it means FlexSPI PORT B exists, SCKB pads used as PORTB SCK with be used a inverted
        // SCK for PORTA
        // Enable differential clock as needed.
        if ((sizeof(base->FLSHCR1) / sizeof(base->FLSHCR1[0])) > 2)
        {
            if (flexspi_is_differential_clock_enable(config))
            {
                base->MCR2 |= FLEXSPI_MCR2_SCKBDIFFOPT_MASK;
            }
        }
#endif

        // Configure AHB buffer
        flexspi_config_ahb_buffers(base, config);

        // Configure Flash related control registers
        flexspi_config_flash_control_registers(instance, config);

        // Configure DLLCR
        flexspi_configure_dll(instance, config);

        // Enable FlexSPI before updating LUT.
        base->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;

        // Reset all registers except control registers
        flexspi_swreset(base);

        if (config->deviceModeCfgEnable)
        {
            status = flexspi_device_workmode_config_all_chips(instance, config);
            if (status != kStatus_Success)
            {
                break;
            }
        }

        if (config->configCmdEnable)
        {
            status = flexspi_device_cmd_config_all_chips(instance, config);
            if (status != kStatus_Success)
            {
                break;
            }
        }

        // Restore clock
        if (need_safe_freq)
        {
            /*
             *  !!! Important !!!
             *  The module clock must be disabled during clock switch in order to avoid glitch
             */
            base->MCR0 |= FLEXSPI_MCR0_MDIS_MASK;
            // Re-configure FlexSPI Serial clock frequency in order to acheive high performance.
            flexspi_clock_config(instance, config->serialClkFreq, flexspi_is_ddr_mode_enable(config));

            // Re-Configure MCR1
            flexspi_config_mcr1(instance, config);
            // Re-configure DLLCR
            flexspi_configure_dll(instance, config);
            base->MCR0 &= ~FLEXSPI_MCR0_MDIS_MASK;
        }

        status = kStatus_Success;

    } while (0);

    return status;
}

#if (!defined(BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI)) || (BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI == 0)
status_t flexspi_update_lut(uint32_t instance, uint32_t seqIndex, const uint32_t *lutBase, uint32_t seqNumber)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        FLEXSPI_Type *base = flexspi_get_module_base(instance);

        if ((base == NULL) || (lutBase == NULL) || (((uint64_t)seqIndex + seqNumber) > 16))
        {
            break;
        }

        flexspi_wait_until_ip_idle(base);

        uint32_t start_index = 0;
        uint32_t end_index = 0;
        volatile uint32_t *flexspiLutPtr;

        flexspi_unlock_lut(base);

        start_index = 4 * seqIndex;
        end_index = 4 * (seqIndex + seqNumber);
        flexspiLutPtr = &base->LUT[start_index];
        while (start_index < end_index)
        {
            *flexspiLutPtr = *lutBase;
            start_index++;
            flexspiLutPtr++;
            lutBase++;
        }

        flexspi_lock_lut(base);

        status = kStatus_Success;

    } while (0);

    return status;
}
#endif // #if (!defined (BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI)) || (BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI == 0)

#if (!BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI) || (!ROM_API_HAS_FLEXSPI_XFER)
status_t flexspi_command_xfer(uint32_t instance, flexspi_xfer_t *xfer)
{
    status_t status = kStatus_InvalidArgument;
    FLEXSPI_Type *base = flexspi_get_module_base(instance);

    do
    {
        if ((base == NULL) || (xfer == NULL) || (xfer->operation > kFlexSpiOperation_End))
        {
            break;
        }

        uint32_t temp = 0;
#if !FLEXSPI_FEATURE_HAS_PARALLEL_MODE
        bool isParallelMode = false;
#else
        bool isParallelMode = xfer->isParallelModeEnable;
#endif // FLEXSPI_FEATURE_HAS_PARALLEL_MODE

        flexspi_wait_until_ip_idle(base);

        // Clear sequence pointer before sending data to external devices
        flexspi_clear_sequence_pointer(instance);

        // Clear former pending status before start this transfer.
        flexspi_clear_error_status(base);

        // Configure base address
        base->IPCR0 = xfer->baseAddress;

        if (xfer->operation == kFlexSpiOperation_Write)
        {
            temp = FLEXSPI_IPCR1_IDATSZ(xfer->txSize);
        }

        if (xfer->operation == kFlexSpiOperation_Read)
        {
            temp = FLEXSPI_IPCR1_IDATSZ(xfer->rxSize);
        }

        temp |= FLEXSPI_IPCR1_ISEQID(xfer->seqId) | FLEXSPI_IPCR1_ISEQNUM(xfer->seqNum - 1) |
                FLEXSPI_IPCR1_IPAREN((uint32_t)isParallelMode);

        base->IPCR1 = temp;

        if (xfer->operation == kFlexSpiOperation_Read)
        {
            register uint32_t xferRemainingSize = xfer->rxSize;
            register uint32_t *xferBufferPtr = xfer->rxBuffer;
            if ((size_t)xferBufferPtr & 0x03)
            {
                break;
            }
            // Set Half RX FIFO size as Watermark size
            uint32_t rx_fifo_size = sizeof(base->RFDR);
            uint32_t watermark = rx_fifo_size / 8 / 2;
            uint32_t burst_rx_size = rx_fifo_size / 2;
            base->IPRXFCR = FLEXSPI_IPRXFCR_RXWMRK(watermark - 1);

            // Clear FIFO before read
            flexspi_clear_ip_rxfifo(base);

            // Start Read
            base->IPCMD |= FLEXSPI_IPCMD_TRG_MASK;
            while (xferRemainingSize > 0)
            {
                register volatile uint32_t *rx_fifo_reg = (volatile uint32_t *)&base->RFDR[0];
                if (xferRemainingSize >= burst_rx_size)
                {
                    register uint32_t burst_rx_round = burst_rx_size / sizeof(uint32_t);
                    // Make sure the RX FIFO contains valid data before read
                    if (base->INTR & FLEXSPI_INTR_IPRXWA_MASK)
                    {
                        while (burst_rx_round--)
                        {
                            *xferBufferPtr++ = *rx_fifo_reg++;
                        }
                        xferRemainingSize -= burst_rx_size;
                        // Pop up data to RXFIFO for next read.
                        base->INTR |= FLEXSPI_INTR_IPRXWA_MASK;
                    }
                }
                else if ((8 * ((base->IPRXFSTS & FLEXSPI_IPRXFSTS_FILL_MASK) >> FLEXSPI_IPRXFSTS_FILL_SHIFT)) >=
                         xferRemainingSize)
                {
                    while (xferRemainingSize >= 4)
                    {
                        *xferBufferPtr++ = *rx_fifo_reg++;
                        xferRemainingSize -= 4;
                    }
                    if (xferRemainingSize)
                    {
                        uint32_t buf = *rx_fifo_reg;
                        uint8_t *src = (uint8_t *)&buf;
                        uint8_t *dst = (uint8_t *)xferBufferPtr;
                        for (uint32_t index = 0; index < xferRemainingSize; index++)
                        {
                            *dst++ = *src++;
                        }
                    }
                    xferRemainingSize = 0;
                }

                // Ensure no error occurs during read
                if ((base->INTR & FLEXSPI_INTR_IPCMDERR_MASK) != 0U)
                {
                    break;
                }
            }
        }

        if ((xfer->operation == kFlexSpiOperation_Write) || (xfer->operation == kFlexSpiOperation_Config))
        {
            register int32_t xferRemainingSize = (int32_t)xfer->txSize;
            register uint32_t *xferBufferPtr = xfer->txBuffer;
            // Enusre that the tx buffer is 4 words aligned for better performance
            if ((size_t)xferBufferPtr & 0x03)
            {
                break;
            }

            // Set Half TX FIFO size as Watermark size
            uint32_t tx_fifo_size = sizeof(base->TFDR);
            uint32_t watermark = tx_fifo_size / 8 / 2;
            uint32_t burst_tx_size = tx_fifo_size / 2;
            base->IPTXFCR = FLEXSPI_IPTXFCR_TXWMRK(watermark - 1);

            // Clear FIFO before write
            flexspi_clear_ip_txfifo(base);

            bool is_transfer_started = false;
            while (xferRemainingSize > 0)
            {
                register volatile uint32_t *tx_fifo_reg = &base->TFDR[0];
                // Make sure the TXFIFO is not full before write
                if (base->INTR & FLEXSPI_INTR_IPTXWE_MASK)
                {
                    register uint32_t burst_tx_round = burst_tx_size / sizeof(uint32_t);
                    if (xferRemainingSize >= burst_tx_size)
                    {
                        while (burst_tx_round--)
                        {
                            *tx_fifo_reg++ = *xferBufferPtr++;
                        }
                        xferRemainingSize -= burst_tx_size;
                    }
                    else
                    {
                        while (burst_tx_round--)
                        {
                            if (xferRemainingSize > 0)
                            {
                                xferRemainingSize -= 4;
                                *tx_fifo_reg++ = *xferBufferPtr++;
                            }
                            else
                            {
                                *tx_fifo_reg++ = 0x0;
                            }
                        }
                    }
                    if (!is_transfer_started)
                    {
                        base->IPCMD |= FLEXSPI_IPCMD_TRG_MASK;
                        is_transfer_started = true;
                    }

                    base->INTR |= FLEXSPI_INTR_IPTXWE_MASK;
                }

                // Ensure no error occurs during write
                if ((base->INTR & FLEXSPI_INTR_IPCMDERR_MASK) != 0U)
                {
                    break;
                }
            }
        }
        if (xfer->operation == kFlexSpiOperation_Command)
        {
            base->IPCMD |= FLEXSPI_IPCMD_TRG_MASK;
        }

        flexspi_wait_until_ip_idle(base);

        if (base->INTR & FLEXSPI_INTR_IPCMDERR_MASK)
        {
            switch ((base->STS1 & FLEXSPI_STS1_IPCMDERRCODE_MASK) >> FLEXSPI_STS1_IPCMDERRCODE_SHIFT)
            {
                default:
                    status = kStatus_FLEXSPI_InvalidSequence;
                    break;
                case kFlexSpiIpCmdError_SequenceExecutionTimeout:
                    status = kStatus_FLEXSPI_SequenceExecutionTimeout;
                    break;
            }
        }
        else
        {
            status = kStatus_Success;
        }

    } while (0);

    return status;
}
#endif // #if (!BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI) || (!ROM_API_HAS_FLEXSPI_XFER)

void flexspi_wait_idle(uint32_t instance)
{
    do
    {
        FLEXSPI_Type *base = flexspi_get_module_base(instance);
        if (base == NULL)
        {
            break;
        }

        // Wait until FlexSPI controller becomes idle
        while (!(base->STS0 & FLEXSPI_STS0_ARBIDLE_MASK))
        {
        }

    } while (0);
}

#if (!BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI) || (!ROM_API_HAS_FLEXSPI_CLEAR_CACHE)
void flexspi_clear_cache(uint32_t instance)
{
    do
    {
        FLEXSPI_Type *base = flexspi_get_module_base(instance);
        if (base == NULL)
        {
            break;
        }

        flexspi_swreset(base);

    } while (0);
}
#endif // #if (!BL_FEATURE_HAS_FLEXSPI_NOR_ROMAPI) || (!ROM_API_HAS_FLEXSPI_CLEAR_CACHE)

#if FLEXSPI_FEATURE_HAS_PARALLEL_MODE
static status_t flexspi_extract_parallel_data(uint32_t *dst0, uint32_t *dst1, uint32_t *src, uint32_t length)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        uint8_t *dst0Byte = (uint8_t *)dst0;
        uint8_t *dst1Byte = (uint8_t *)dst1;
        uint8_t *srcByte = (uint8_t *)src;
        if ((length < 1) || (length & 1) || (dst0 == NULL) || (dst1 == NULL))
        {
            break;
        }

        while (length > 0)
        {
            *dst0Byte++ = *srcByte++;
            *dst1Byte++ = *srcByte++;
            length -= 2;
        }
        status = kStatus_Success;

    } while (0);

    return status;
}
#endif // FLEXSPI_FEATURE_HAS_PARALLEL_MODE

status_t flexspi_device_wait_busy(uint32_t instance,
                                  flexspi_mem_config_t *config,
                                  bool isParallelMode,
                                  uint32_t baseAddr)
{
    status_t status = kStatus_InvalidArgument;

    do
    {
        if (config == NULL)
        {
            break;
        }
        uint32_t statusDataBuffer[2];
        uint32_t busyMask;
        uint32_t busyPolarity;
        bool isBusy = false;
#if FLEXSPI_FEATURE_HAS_PARALLEL_MODE
        uint32_t status0, status1;
#else
        isParallelMode = false;
#endif // FLEXSPI_FEATURE_HAS_PARALLEL_MODE
        flexspi_xfer_t flashXfer;

        busyMask = 1 << config->busyOffset;
        busyPolarity = config->busyBitPolarity;

        flashXfer.baseAddress = baseAddr;
        flashXfer.operation = kFlexSpiOperation_Read;
        flashXfer.seqNum = 1;
        flashXfer.seqId = CMD_LUT_SEQ_IDX_READSTATUS;
        flashXfer.rxBuffer = &statusDataBuffer[0];

        flashXfer.rxSize = isParallelMode ? sizeof(statusDataBuffer) : sizeof(statusDataBuffer[0]);
        flashXfer.isParallelModeEnable = isParallelMode;

        if (config->lutCustomSeqEnable && config->lutCustomSeq[CMD_INDEX_READSTATUS].seqNum)
        {
            flashXfer.seqId = config->lutCustomSeq[CMD_INDEX_READSTATUS].seqId;
            flashXfer.seqNum = config->lutCustomSeq[CMD_INDEX_READSTATUS].seqNum;
        }

        flexspi_update_lut(instance, flashXfer.seqId, &config->lookupTable[4 * flashXfer.seqId], flashXfer.seqNum);

        bool enableTimeoutCheck = config->timeoutInMs ? true : false;
        uint64_t remainingMs = config->timeoutInMs;

        do
        {
            status = flexspi_command_xfer(instance, &flashXfer);
            if (status != kStatus_Success)
            {
                break;
            }
            else
            {
#if FLEXSPI_FEATURE_HAS_PARALLEL_MODE
                if (isParallelMode)
                {
                    // Extract parallel data to serial data
                    flexspi_extract_parallel_data(&status0, &status1, statusDataBuffer, sizeof(statusDataBuffer));

                    if (busyPolarity)
                    {
                        isBusy = ((~status0) & busyMask) | ((~status1) & busyMask);
                    }
                    else
                    {
                        isBusy = (status0 & busyMask) | (status1 & busyMask);
                    }
                }
                else
#endif // FLEXSPI_FEATURE_HAS_PARALLEL_MODE
                {
                    // Busy bit is 0 if polarity is 1
                    if (busyPolarity)
                    {
                        isBusy = (~statusDataBuffer[0]) & busyMask;
                    }
                    else
                    {
                        isBusy = statusDataBuffer[0] & busyMask;
                    }
                }
            }

            if (isBusy && enableTimeoutCheck)
            {
                if (remainingMs > 0)
                {
                    flexspi_sw_delay_us(1000);
                    remainingMs--;
                }
                else
                {
                    status = kStatus_FLEXSPI_DeviceTimeout;
                    break;
                }
            }

        } while (isBusy);

    } while (0);

    return status;
}

void flexspi_clear_sequence_pointer(uint32_t instance)
{
    uint32_t index;

    do
    {
        FLEXSPI_Type *base = flexspi_get_module_base(instance);
        if (base == NULL)
        {
            break;
        }

        for (index = 0; index < FLEXSPI_FLSHCR2_COUNT; index++)
        {
            base->FLSHCR2[index] |= FLEXSPI_FLSHCR2_CLRINSTRPTR_MASK;
        }

    } while (0);
}

void flexspi_half_clock_control(uint32_t instance, uint32_t option)
{
    do
    {
        FLEXSPI_Type *base = flexspi_get_module_base(instance);

        if (base == NULL)
        {
            break;
        }

        flexspi_wait_until_ip_idle(base);

        if (option)
        {
            base->MCR0 |= FLEXSPI_MCR0_HSEN_MASK;
        }
        else
        {
            base->MCR0 &= (uint32_t)~FLEXSPI_MCR0_HSEN_MASK;
        }

    } while (0);
}

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
                config->dataValidTime[0].time_100ps = 19; // 1.9 ns // 1/4 * cycle of 133MHz DDR
            }
            else
            {
                if (config->dataValidTime[0].delay_cells < 1)
                {
                    config->dataValidTime[0].time_100ps = 38; // 3.8 ns // 1/2 * cycle of 133MHz DDR
                }
            }
        }
        status = kStatus_Success;

    } while (0);

    return status;
}

// Get max supported Frequency in this SoC
status_t flexspi_get_max_supported_freq(uint32_t instance, uint32_t *freq, uint32_t clkMode)
{
    status_t status = kStatus_InvalidArgument;
    do
    {
        if ((instance != 0) || (freq == NULL))
        {
            break;
        }

        *freq = (133UL * 1000 * 1000);
        status = kStatus_Success;

    } while (0);

    return status;
}

/*! @brief PLL PFD name */
typedef enum _clock_pfd
{
    kCLOCK_Pfd0 = 0U, /*!< PLL PFD0 */
    kCLOCK_Pfd1 = 1U, /*!< PLL PFD1 */
    kCLOCK_Pfd2 = 2U, /*!< PLL PFD2 */
    kCLOCK_Pfd3 = 3U, /*!< PLL PFD3 */
} clock_pfd_t;

#define CCM_ANALOG_TUPLE(reg, shift) ((((reg)&0xFFFU) << 16U) | (shift))
#define CCM_ANALOG_TUPLE_SHIFT(tuple) (((uint32_t)(tuple)) & 0x1FU)
#define CCM_ANALOG_TUPLE_REG_OFF(base, tuple, off) \
    (*((volatile uint32_t *)((uint32_t)(base) + (((uint32_t)(tuple) >> 16U) & 0xFFFU) + (off))))
#define CCM_ANALOG_TUPLE_REG(base, tuple) CCM_ANALOG_TUPLE_REG_OFF(base, tuple, 0U)
/*!
 * @brief CCM Analog registers offset.
 */
#define PLL_SYS_OFFSET 0x30
#define PLL_USB1_OFFSET 0x10
#define PLL_AUDIO_OFFSET 0x70
#define PLL_ENET_OFFSET 0xE0

#define CCM_ANALOG_PLL_BYPASS_SHIFT (16U)
#define CCM_ANALOG_PLL_BYPASS_CLK_SRC_MASK (0xC000U)
#define CCM_ANALOG_PLL_BYPASS_CLK_SRC_SHIFT (14U)		

#define CLKPN_FREQ 0U
		
/*!@brief PLL clock source, bypass cloco source also */
enum _clock_pll_clk_src
{
    kCLOCK_PllClkSrc24M = 0U, /*!< Pll clock source 24M */
    kCLOCK_PllSrcClkPN  = 1U, /*!< Pll clock source CLK1_P and CLK1_N */
};
		
/*! @brief PLL name */
typedef enum _clock_pll
{
    kCLOCK_PllSys   = CCM_ANALOG_TUPLE(PLL_SYS_OFFSET, CCM_ANALOG_PLL_SYS_ENABLE_SHIFT),     /*!< PLL SYS */
    kCLOCK_PllUsb1  = CCM_ANALOG_TUPLE(PLL_USB1_OFFSET, CCM_ANALOG_PLL_USB1_ENABLE_SHIFT),   /*!< PLL USB1 */
    kCLOCK_PllAudio = CCM_ANALOG_TUPLE(PLL_AUDIO_OFFSET, CCM_ANALOG_PLL_AUDIO_ENABLE_SHIFT), /*!< PLL Audio */

    kCLOCK_PllEnet = CCM_ANALOG_TUPLE(PLL_ENET_OFFSET, CCM_ANALOG_PLL_ENET_ENABLE_SHIFT), /*!< PLL Enet0 */

    kCLOCK_PllEnet500M = CCM_ANALOG_TUPLE(PLL_ENET_OFFSET, CCM_ANALOG_PLL_ENET_ENET_500M_REF_EN_SHIFT), /*!< PLL ENET */

    kCLOCK_PllEnet25M = CCM_ANALOG_TUPLE(PLL_ENET_OFFSET, CCM_ANALOG_PLL_ENET_ENET_25M_REF_EN_SHIFT), /*!< PLL Enet1 */

} clock_pll_t;

typedef uint64_t clock_64b_t;

static inline bool CLOCK_IsPllEnabled(CCM_ANALOG_Type *base, clock_pll_t pll)
{
    return (bool)(CCM_ANALOG_TUPLE_REG(base, pll) & (1UL << CCM_ANALOG_TUPLE_SHIFT(pll)));
}

static inline uint32_t CLOCK_GetOscFreq(void)
{
	  /* External XTAL (OSC) clock frequency. */
    volatile uint32_t g_xtalFreq = 24000000UL;
    return ((XTALOSC24M->LOWPWR_CTRL & (uint32_t)XTALOSC24M_LOWPWR_CTRL_OSC_SEL_MASK) != 0UL) ? 24000000UL : g_xtalFreq;
}

static inline uint32_t CLOCK_GetPllBypassRefClk(CCM_ANALOG_Type *base, clock_pll_t pll)
{
    return ((((uint32_t)(CCM_ANALOG_TUPLE_REG(base, pll) & CCM_ANALOG_PLL_BYPASS_CLK_SRC_MASK)) >>
             CCM_ANALOG_PLL_BYPASS_CLK_SRC_SHIFT) == (uint32_t)kCLOCK_PllClkSrc24M) ?
               CLOCK_GetOscFreq() :
               CLKPN_FREQ;
}

static inline bool CLOCK_IsPllBypassed(CCM_ANALOG_Type *base, clock_pll_t pll)
{
    return (bool)(CCM_ANALOG_TUPLE_REG(base, pll) & (1UL << CCM_ANALOG_PLL_BYPASS_SHIFT));
}

uint32_t CLOCK_GetPllFreq(clock_pll_t pll)
{
    uint32_t freq;
    uint32_t divSelect;
    clock_64b_t freqTmp;

    const uint32_t enetRefClkFreq[] = {
        25000000U,  /* 25M */
        50000000U,  /* 50M */
        100000000U, /* 100M */
        125000000U  /* 125M */
    };

    /* check if PLL is enabled */
    if (!CLOCK_IsPllEnabled(CCM_ANALOG, pll))
    {
        return 0U;
    }

    /* get pll reference clock */
    freq = CLOCK_GetPllBypassRefClk(CCM_ANALOG, pll);

    /* check if pll is bypassed */
    if (CLOCK_IsPllBypassed(CCM_ANALOG, pll))
    {
        return freq;
    }

    switch (pll)
    {
        case kCLOCK_PllSys:
            /* PLL output frequency = Fref * (DIV_SELECT + NUM/DENOM). */
            freqTmp = ((clock_64b_t)freq * ((clock_64b_t)(CCM_ANALOG->PLL_SYS_NUM)));
            freqTmp /= ((clock_64b_t)(CCM_ANALOG->PLL_SYS_DENOM));

            if ((CCM_ANALOG->PLL_SYS & CCM_ANALOG_PLL_SYS_DIV_SELECT_MASK) != 0UL)
            {
                freq *= 22U;
            }
            else
            {
                freq *= 20U;
            }

            freq += (uint32_t)freqTmp;
            break;

        case kCLOCK_PllUsb1:
            freq = (freq * (((CCM_ANALOG->PLL_USB1 & CCM_ANALOG_PLL_USB1_DIV_SELECT_MASK) != 0UL) ? 22U : 20U));
            break;

        case kCLOCK_PllAudio:
            /* PLL output frequency = Fref * (DIV_SELECT + NUM/DENOM). */
            divSelect =
                (CCM_ANALOG->PLL_AUDIO & CCM_ANALOG_PLL_AUDIO_DIV_SELECT_MASK) >> CCM_ANALOG_PLL_AUDIO_DIV_SELECT_SHIFT;

            freqTmp = ((clock_64b_t)freq * ((clock_64b_t)(CCM_ANALOG->PLL_AUDIO_NUM)));
            freqTmp /= ((clock_64b_t)(CCM_ANALOG->PLL_AUDIO_DENOM));

            freq = freq * divSelect + (uint32_t)freqTmp;

            /* AUDIO PLL output = PLL output frequency / POSTDIV. */

            /*
             * Post divider:
             *
             * PLL_AUDIO[POST_DIV_SELECT]:
             * 0x00: 4
             * 0x01: 2
             * 0x02: 1
             *
             * MISC2[AUDO_DIV]:
             * 0x00: 1
             * 0x01: 2
             * 0x02: 1
             * 0x03: 4
             */
            switch (CCM_ANALOG->PLL_AUDIO & CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT_MASK)
            {
                case CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT(0U):
                    freq = freq >> 2U;
                    break;

                case CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT(1U):
                    freq = freq >> 1U;
                    break;

                case CCM_ANALOG_PLL_AUDIO_POST_DIV_SELECT(2U):
                    freq = freq >> 0U;
                    break;

                default:
                    assert(false);
                    break;
            }

            switch (CCM_ANALOG->MISC2 & (CCM_ANALOG_MISC2_AUDIO_DIV_MSB_MASK | CCM_ANALOG_MISC2_AUDIO_DIV_LSB_MASK))
            {
                case CCM_ANALOG_MISC2_AUDIO_DIV_MSB(1) | CCM_ANALOG_MISC2_AUDIO_DIV_LSB(1):
                    freq >>= 2U;
                    break;

                case CCM_ANALOG_MISC2_AUDIO_DIV_MSB(0) | CCM_ANALOG_MISC2_AUDIO_DIV_LSB(1):
                    freq >>= 1U;
                    break;

                case CCM_ANALOG_MISC2_AUDIO_DIV_MSB(0) | CCM_ANALOG_MISC2_AUDIO_DIV_LSB(0):
                case CCM_ANALOG_MISC2_AUDIO_DIV_MSB(1) | CCM_ANALOG_MISC2_AUDIO_DIV_LSB(0):
                    freq >>= 0U;
                    break;

                default:
                    assert(false);
                    break;
            }
            break;

        case kCLOCK_PllEnet:
            divSelect =
                (CCM_ANALOG->PLL_ENET & CCM_ANALOG_PLL_ENET_DIV_SELECT_MASK) >> CCM_ANALOG_PLL_ENET_DIV_SELECT_SHIFT;
            freq = enetRefClkFreq[divSelect];
            break;

        case kCLOCK_PllEnet25M:
            /* ref_enetpll1 if fixed at 25MHz. */
            freq = 25000000UL;
            break;

        case kCLOCK_PllEnet500M:
            /* PLL6 is fixed at 25MHz. */
            freq = 500000000UL;
            break;

        default:
            freq = 0U;
            break;
    }

    return freq;
}


uint32_t CLOCK_GetUsb1PfdFreq(clock_pfd_t pfd)
{
    uint32_t freq = CLOCK_GetPllFreq(kCLOCK_PllUsb1);

    switch (pfd)
    {
        case kCLOCK_Pfd0:
            freq /= ((CCM_ANALOG->PFD_480 & CCM_ANALOG_PFD_480_PFD0_FRAC_MASK) >> CCM_ANALOG_PFD_480_PFD0_FRAC_SHIFT);
            break;

        case kCLOCK_Pfd1:
            freq /= ((CCM_ANALOG->PFD_480 & CCM_ANALOG_PFD_480_PFD1_FRAC_MASK) >> CCM_ANALOG_PFD_480_PFD1_FRAC_SHIFT);
            break;

        case kCLOCK_Pfd2:
            freq /= ((CCM_ANALOG->PFD_480 & CCM_ANALOG_PFD_480_PFD2_FRAC_MASK) >> CCM_ANALOG_PFD_480_PFD2_FRAC_SHIFT);
            break;

        case kCLOCK_Pfd3:
            freq /= ((CCM_ANALOG->PFD_480 & CCM_ANALOG_PFD_480_PFD3_FRAC_MASK) >> CCM_ANALOG_PFD_480_PFD3_FRAC_SHIFT);
            break;

        default:
            freq = 0U;
            break;
    }
    freq *= 18U;

    return freq;
}

uint32_t CLOCK_GetSysPfdFreq(clock_pfd_t pfd)
{
    uint32_t freq = CLOCK_GetPllFreq(kCLOCK_PllSys);

    switch (pfd)
    {
        case kCLOCK_Pfd0:
            freq /= ((CCM_ANALOG->PFD_528 & CCM_ANALOG_PFD_528_PFD0_FRAC_MASK) >> CCM_ANALOG_PFD_528_PFD0_FRAC_SHIFT);
            break;

        case kCLOCK_Pfd1:
            freq /= ((CCM_ANALOG->PFD_528 & CCM_ANALOG_PFD_528_PFD1_FRAC_MASK) >> CCM_ANALOG_PFD_528_PFD1_FRAC_SHIFT);
            break;

        case kCLOCK_Pfd2:
            freq /= ((CCM_ANALOG->PFD_528 & CCM_ANALOG_PFD_528_PFD2_FRAC_MASK) >> CCM_ANALOG_PFD_528_PFD2_FRAC_SHIFT);
            break;

        case kCLOCK_Pfd3:
            freq /= ((CCM_ANALOG->PFD_528 & CCM_ANALOG_PFD_528_PFD3_FRAC_MASK) >> CCM_ANALOG_PFD_528_PFD3_FRAC_SHIFT);
            break;

        default:
            freq = 0U;
            break;
    }
    freq *= 18U;

    return freq;
}

uint32_t CLOCK_GetCPUFreq_RT1020(void)
{
    uint32_t freq;

    /* Periph_clk2_clk ---> Periph_clk */
    if ((CCM->CBCDR & CCM_CBCDR_PERIPH_CLK_SEL_MASK) != 0UL)
    {
        switch (CCM->CBCMR & CCM_CBCMR_PERIPH_CLK2_SEL_MASK)
        {
            /* Pll3_sw_clk ---> Periph_clk2_clk ---> Periph_clk */
            case CCM_CBCMR_PERIPH_CLK2_SEL(0U):
                freq = CLOCK_GetPllFreq(kCLOCK_PllUsb1);
                break;

            /* Osc_clk ---> Periph_clk2_clk ---> Periph_clk */
            case CCM_CBCMR_PERIPH_CLK2_SEL(1U):
                freq = CLOCK_GetOscFreq();
                break;

            case CCM_CBCMR_PERIPH_CLK2_SEL(2U):
                freq = CLOCK_GetPllFreq(kCLOCK_PllSys);
                break;

            case CCM_CBCMR_PERIPH_CLK2_SEL(3U):
            default:
                freq = 0U;
                break;
        }

        freq /= (((CCM->CBCDR & CCM_CBCDR_PERIPH_CLK2_PODF_MASK) >> CCM_CBCDR_PERIPH_CLK2_PODF_SHIFT) + 1U);
    }
    /* Pre_Periph_clk ---> Periph_clk */
    else
    {
        switch (CCM->CBCMR & CCM_CBCMR_PRE_PERIPH_CLK_SEL_MASK)
        {
            /* PLL2 */
            case CCM_CBCMR_PRE_PERIPH_CLK_SEL(0U):
                freq = CLOCK_GetPllFreq(kCLOCK_PllSys);
                break;

            /* PLL3 PFD3 */
            case CCM_CBCMR_PRE_PERIPH_CLK_SEL(1U):
                freq = CLOCK_GetUsb1PfdFreq(kCLOCK_Pfd3);
                break;

            /* PLL2 PFD3 */
            case CCM_CBCMR_PRE_PERIPH_CLK_SEL(2U):
                freq = CLOCK_GetSysPfdFreq(kCLOCK_Pfd3);
                break;

            /* PLL6 divided(/1) */
            case CCM_CBCMR_PRE_PERIPH_CLK_SEL(3U):
                freq = 500000000U;
                break;

            default:
                freq = 0U;
                break;
        }
    }
	
		freq = freq / (((CCM->CBCDR & CCM_CBCDR_AHB_PODF_MASK) >> CCM_CBCDR_AHB_PODF_SHIFT) + 1U);;
		
    return freq;
}

void flexspi_sw_delay_us(uint64_t us)
{
    uint32_t ticks_per_us = CLOCK_GetCPUFreq_RT1020() / 1000000;
    while(us--)
    {
        volatile uint32_t ticks = ticks_per_us / 4;
        while(ticks--)
        {
            __NOP();
        }
    }
}

void flexspi_update_padsetting(flexspi_mem_config_t *config, uint32_t driveStrength)
{
#define IOMUXC_PAD_SETTING_DSE_SHIFT (3)
#define IOMUXC_PAD_SETTING_DSE_MASK (0x07 << IOMUXC_PAD_SETTING_DSE_SHIFT)
#define IOMUXC_PAD_SETTING_DSE(x) (((x) << IOMUXC_PAD_SETTING_DSE_SHIFT) & IOMUXC_PAD_SETTING_DSE_MASK)
    if (driveStrength)
    {
        config->controllerMiscOption |= FLEXSPI_BITMASK(kFlexSpiMiscOffset_PadSettingOverrideEnable);
        config->dqsPadSettingOverride =
            (FLEXSPI_DQS_SW_PAD_CTL_VAL & ~IOMUXC_PAD_SETTING_DSE_MASK) | IOMUXC_PAD_SETTING_DSE(driveStrength);
        config->sclkPadSettingOverride =
            (FLEXSPI_SW_PAD_CTL_VAL & ~IOMUXC_PAD_SETTING_DSE_MASK) | IOMUXC_PAD_SETTING_DSE(driveStrength);
        config->dataPadSettingOverride =
            (FLEXSPI_SW_PAD_CTL_VAL & ~IOMUXC_PAD_SETTING_DSE_MASK) | IOMUXC_PAD_SETTING_DSE(driveStrength);

        config->csPadSettingOverride =
            (FLEXSPI_DQS_SW_PAD_CTL_VAL & ~IOMUXC_PAD_SETTING_DSE_MASK) | IOMUXC_PAD_SETTING_DSE(driveStrength);
    }
}
