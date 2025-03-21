/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/
/*
 * How to setup clock using clock driver functions:
 *
 * 1. call CLOCK_SetSimSafeDivs() to set the system clock dividers in SIM to safe value.
 *
 * 2. If external oscillator is used Call CLOCK_SetXtal0Freq() to set XTAL0 frequency based on board settings and
 *    call CLOCK_InitOsc0() to init the OSC.
 *
 * 3. Call CLOCK_BootToXxxMode()/CLOCK_SetXxxMode() to set ICS run at the target mode.
 *
 * 4. If ICSIRCLK is needed, call CLOCK_SetInternalRefClkConfig() to enable the clock.
 *
 * 5. call CLOCK_SetSimConfig() to configure the divider in sim.
 */

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Clocks v15.0
processor: MKE02Z64xxx4
package_id: MKE02Z64VLD4
mcu_data: ksdk2_0
processor_version: 24.12.10
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 ************************ BOARD_InitBootClocks function ************************
 ******************************************************************************/
void BOARD_InitBootClocks(void)
{
    BOARD_BootClockRUN();
}

/*******************************************************************************
 ********************** Configuration BOARD_BootClockRUN ***********************
 ******************************************************************************/
/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!Configuration
name: BOARD_BootClockRUN
called_from_default_init: true
outputs:
- {id: Bus_clock.outFreq, value: 16 MHz}
- {id: Core_clock.outFreq, value: 16 MHz}
- {id: Flash_clock.outFreq, value: 16 MHz}
- {id: ICSFF_clock.outFreq, value: 31.25/2 kHz}
- {id: LPO_clock.outFreq, value: 1 kHz}
- {id: Plat_clock.outFreq, value: 16 MHz}
- {id: System_clock.outFreq, value: 16 MHz}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/*******************************************************************************
 * Variables for BOARD_BootClockRUN configuration
 ******************************************************************************/
const ics_config_t icsConfig_BOARD_BootClockRUN =
    {
        .icsMode = kICS_ModeFEI,                  /* FEI - FLL Engaged Internal */
        .irClkEnableMode = 0,                     /* ICSIRCLK disabled */
        .bDiv = 0x1U,                             /* Bus clock divider: divided by 2 */
        .rDiv = 0x0U,                             /* FLL external reference clock divider: divided by 1 */
    };
const sim_clock_config_t simConfig_BOARD_BootClockRUN =
    {
        .busDiv = 0x0U,                           /* BUSDIV clock divider: divided by 1 */
        .busClkPrescaler = 0x0U,                  /* bus clock optional prescaler */
    };
const osc_config_t oscConfig_BOARD_BootClockRUN =
    {
        .freq = 0U,                               /* Oscillator frequency: 0Hz */
        .workMode = 0,                            /* Use external clock */
        .enableMode = 0,                          /* Disable external reference clock */
    };

/*******************************************************************************
 * Code for BOARD_BootClockRUN configuration
 ******************************************************************************/
void BOARD_BootClockRUN(void)
{
    /* Set the system clock dividers in SIM to safe value. */
    CLOCK_SetSimSafeDivs();
    /* Set ICS to FEI mode. */
    CLOCK_BootToFeiMode(icsConfig_BOARD_BootClockRUN.bDiv);
    /* Configure the Internal Reference clock (ICSIRCLK). */
    CLOCK_SetInternalRefClkConfig(icsConfig_BOARD_BootClockRUN.irClkEnableMode);
    /* Set the clock configuration in SIM module. */
    CLOCK_SetSimConfig(&simConfig_BOARD_BootClockRUN);
    /* Set SystemCoreClock variable. */
    SystemCoreClock = BOARD_BOOTCLOCKRUN_CORE_CLOCK;
}

