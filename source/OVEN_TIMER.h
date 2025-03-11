/*
 * Copyright (c) 2025-2026, RitaVo Group, LUXHOVO
 * Copyright 2025-2026 RTV
 * All rights reserved.
 *
/-----------------------------------------------------------------------------------
 * Project OVEN LUXHOVO
 * File OVEN_TIMER.h
 * Created on: 	Feb 25, 2025	                                                *
 * Version 1.0
 */

#ifndef OVEN_TIMER_H_
#define OVEN_TIMER_H_

#include "stdint.h"
#include "math.h"
#include "stdlib.h"



/******************************************************************************
 *                                                                            *
 *  												DEFINE  	                                        *
 *  														                                              *
 ******************************************************************************/



/* define instance */
#define BOARD_TPM_0 TPM0
#define BOARD_TPM_1 TPM1

/* Interrupt number and interrupt handler for the TPM instance used */
#define BOARD_TPM_IRQ_NUM_0 TPM0_IRQn
#define BOARD_TPM_HANDLER_0 TPM0_IRQHandler

#define BOARD_TPM_IRQ_NUM_1 TPM1_IRQn
#define BOARD_TPM_HANDLER_1 TPM1_IRQHandler
/* Get source clock for TPM driver */
#define TPM_SOURCE_CLOCK (CLOCK_GetFreq(kCLOCK_BusClk) / 4)
#ifndef DEMO_TIMER_PERIOD_US
/* Set counter period to 1ms */
#define DEMO_TIMER_PERIOD_US (1000U)
#endif
#ifndef TPM_PRESCALER_0
/* Calculate the clock division based on the PWM frequency to be obtained */
#define TPM_PRESCALER_0 TPM_CalculateCounterClkDiv(BOARD_TPM_0, 1000000U / DEMO_TIMER_PERIOD_US, TPM_SOURCE_CLOCK);
#endif

#ifndef TPM_PRESCALER_1
/* Calculate the clock division based on the PWM frequency to be obtained */
#define TPM_PRESCALER_1 TPM_CalculateCounterClkDiv(BOARD_TPM_1, 100000U / DEMO_TIMER_PERIOD_US, TPM_SOURCE_CLOCK);
#endif




#define DEMO_PIT_BASEADDR PIT
#define DEMO_PIT_CHANNEL  kPIT_Chnl_0
#define PIT_LED_HANDLER   PIT_CH0_IRQHandler
#define PIT_IRQ_ID        PIT_CH0_IRQn


#define DEMO_PIT_CHANNEL_1  kPIT_Chnl_1
#define PIT_IRQ_ID_1        PIT_CH1_IRQn

/******************************************************************************
*                                                                            *
*  												FUNCTION	                                        *
*  														                                              *
******************************************************************************/


void Config_Timer(void);
void delay_ms(uint16_t ms);

#endif
