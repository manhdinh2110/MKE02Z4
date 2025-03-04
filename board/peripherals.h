/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "fsl_common.h"
#include "fsl_kbi.h"
#include "fsl_clock.h"
#include "fsl_ftm.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/
/* Definitions for BOARD_InitPeripherals functional group */
/* BOARD_InitPeripherals defines for KBI0 */
/* Definition of peripheral ID */
#define KBI0_PERIPHERAL KBI0
/* KBI0 interrupt vector ID (number). */
#define KBI0_IRQN KBI0_IRQn
/* KBI0 interrupt handler identifier. */
#define KBI0_IRQHANDLER KBI0_IRQHandler
/* Definition of peripheral ID */
#define FTM2_PERIPHERAL FTM2
/* Definition of the clock source frequency */
#define FTM2_CLOCK_SOURCE CLOCK_GetFreq(kCLOCK_BusClk)
/* Definition of the clock source frequency */
#define FTM2_TIMER_MODULO_VALUE (((FTM2_CLOCK_SOURCE/ (1U << (FTM2_PERIPHERAL->SC & FTM_SC_PS_MASK))) / 10000) - 1)

/***********************************************************************************************************************
 * Global variables
 **********************************************************************************************************************/
extern kbi_config_t KBI0_Config;
extern const ftm_config_t FTM2_config;

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/

void BOARD_InitPeripherals(void);

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void);

#if defined(__cplusplus)
}
#endif

#endif /* _PERIPHERALS_H_ */
