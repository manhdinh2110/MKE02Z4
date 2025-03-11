/*
 * Copyright 2016-2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    MKE02Z4_Prjadc.c
 * @brief   Application entry point.
 */

#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_uart.h"
#include "fsl_tpm.h"
#include "utils.h"
#include <string.h>
#include "OVEN_UART.h"
#include "fsl_adc.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "fsl_irq.h"
#include "fsl_kbi.h"
#include "fsl_common.h"
#include "fsl_ftm.h"
#include "Defines.h"
#include "OVEN_PWM.h"
#include "OVEN_TIMER.h"
#include "OVEN_UART.h"
#include "OVEN_ADC.h"
#include "fsl_pit.h"


uint8_t rxbuff[20] = {0};
volatile bool check=0;



/*******************************************************************************
 * Definitions//
 ******************************************************************************/


/* Get source clock for TPM driver */
#define TPM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
#ifndef TPM_LED_ON_LEVEL
#define TPM_LED_ON_LEVEL kTPM_HighTrue
#endif
#ifndef DEMO_PWM_FREQUENCY
#define DEMO_PWM_FREQUENCY (3000U)
#endif


#ifndef DEMO_PWM_FREQUENCY_1
#define DEMO_PWM_FREQUENCY_1 (50U)
#endif



/*******************************************************************************
 * Variables
 ******************************************************************************/



uint8_t g_tipString[] =
    "CHECK_INTERRUPT";

uint8_t Oper[]="Operating Mode";
uint8_t Preset[]="Preset-Program";

uart_handle_t g_uartHandle;
#define ECHO_BUFFER_LENGTH 8

uint8_t g_txBuffer[ECHO_BUFFER_LENGTH] = {0};
uint8_t g_rxBuffer[ECHO_BUFFER_LENGTH] = {0};
volatile bool rxBufferEmpty            = true;
volatile bool txBufferFull             = false;
volatile bool txOnGoing                = false;
volatile bool rxOnGoing                = false;

uint8_t demoRingBuffer[DEMO_RING_BUFFER_SIZE];
volatile uint16_t txIndex; /* Index of the data to send out. */

#define BUFFER_SIZE 100
volatile char rxBuffer[BUFFER_SIZE];  // Buffer lưu dữ liệu
volatile uint8_t rxIndex = 0;


// Biến lưu giá trị sau khi tách
int temper;
int val1;
int val2;



#define TIMEOUT_LIMIT 50000
#define BUFFER_SIZE   64
#define MAX_WAIT_TIME 25  //

const uint32_t g_Adc_12bitFullRange = 4096U;


uint32_t numBytesRead = 0;
uint8_t uart_buffer[50] = {0};
uint8_t buffer_index = 0;


int val;

uint8_t buffer[10];

extern bool g_AdcConversionDoneFlag;
extern uint16_t g_AdcConversionValue;
volatile uint32_t g_AdcInterruptCounter;
ftm_chnl_pwm_signal_param_t ftmParam;



extern uint8_t Check_active;
extern uint8_t checkLed;
extern uint8_t Check_mode;
void DEMO_UART_IRQHandler(void)
{


	uint8_t data;


	uint8_t Modeoper[5];
    char * p;
    uint8_t data1[20];
    uint8_t testdata[5];
    uint8_t first[10], second[10];
    uint32_t numBytes = sizeof(buffer);
    uint32_t numBytesRead = UART_ReadMultipleBytes(DEMO_UART,buffer, BUFFER_SIZE);

      //Function for Split data
    String_Split(buffer,',',0,testdata);
    String_Split(buffer,',',1,Modeoper);
    String_Split(buffer,',',2,first);

    int convertedStr = atoi(testdata);
    UART_WriteBlocking(DEMO_UART, convertedStr, strlen((char*)convertedStr));
 //Run
if(convertedStr==1)
{
    UART_WriteBlocking(DEMO_UART,"Running Operating",strlen("Running_Operating"));
    Check_active=1;
    NVIC_EnableIRQ(KBI0_IRQn);  // Vô hiệu hóa ngắt KBI0 trong NVIC
	FTM_StartTimer(BOARD_FTM_BASEADDR, kFTM_SystemClock);
    PIT_EnableInterrupts(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL_1, kPIT_TimerInterruptEnable);
}

//ON LED
if(convertedStr==3)
{

    UART_WriteBlocking(DEMO_UART,"ON-LED",strlen("ON-LED"));
    checkLed=1;
    NVIC_EnableIRQ(KBI0_IRQn);  // Vô hiệu hóa ngắt KBI0 trong NVIC
	FTM_StartTimer(BOARD_FTM_BASEADDR, kFTM_SystemClock);
  //  PIT_EnableInterrupts(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL_1, kPIT_TimerInterruptEnable);
    TPM_EnableInterrupts(BOARD_TPM_0, kTPM_TimeOverflowInterruptEnable);

}

//OFF LED
if(convertedStr==5)
{
    UART_WriteBlocking(DEMO_UART,"OFF-LED",strlen("OFF-LED"));
    checkLed=0;
    NVIC_EnableIRQ(KBI0_IRQn);  // Vô hiệu hóa ngắt KBI0 trong NVIC
	FTM_StartTimer(BOARD_FTM_BASEADDR, kFTM_SystemClock);
	TPM_DisableInterrupts(BOARD_TPM_0, kTPM_TimeOverflowInterruptEnable);


}

//POWER
if(convertedStr==2)
{
    UART_WriteBlocking(DEMO_UART,"POWER OFF",strlen("POWER OFF"));
    Check_active=0;
    checkLed=0;
   	NVIC_EnableIRQ(KBI0_IRQn);
	FTM_StartTimer(BOARD_FTM_BASEADDR, kFTM_SystemClock);
	PIT_DisableInterrupts(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL_1, kPIT_TimerInterruptEnable);
    GPIO_PinWrite(BACK_FAN_GPIO, BACK_FAN_PIN, 0U);
    GPIO_PinWrite(TOP_HEATER_OUTER_GPIO, TOP_HEATER_OUTER_PIN, 0U);
    GPIO_PinWrite(BOTTOM_HEATER_GPIO, BOTTOM_HEATER_PIN, 0U);
}

    SDK_ISR_EXIT_BARRIER;
}

uint8_t buffer_adc[100];


void ADC_IRQHandler(void)
{
    g_AdcConversionDoneFlag = true;
	//UART_WriteBlocking(DEMO_UART, "CHANNEL_0", strlen("CHANNEL_0"));
    g_AdcConversionValue = ADC_GetChannelConversionValue(DEMO_ADC_BASE);
    SDK_ISR_EXIT_BARRIER;
}


uint16_t countFreq1 = 9800;

static bool state=0;
extern float Temperature;
volatile bool isRisingEdge = true;

void KBI0_IRQHandler(void)
{
    if (KBI_IsInterruptRequestDetected(EXAMPLE_KBI))
    {

		if (GPIO_PinRead(DOOR_SENSOR_GPIO,DOOR_SENSOR_PIN) == 0||checkLed==1)
		{

			SDK_DelayAtLeastUs(countFreq1, CLOCK_GetFreq(kCLOCK_CoreSysClk));
			//delay_ms(countFreq1*1000);
			GPIO_PinWrite(TRIAC_LAMP_GPIO,TRIAC_LAMP_PIN,1);
			//delay_ms(10*1000);
			SDK_DelayAtLeastUs(10, CLOCK_GetFreq(kCLOCK_CoreSysClk));
			GPIO_PinWrite(TRIAC_LAMP_GPIO,TRIAC_LAMP_PIN,0);
			if (countFreq1 >150){

			    countFreq1=countFreq1-50;
			//	KBI_ClearInterruptFlag(EXAMPLE_KBI);
				KBI_ClearInterruptFlag(EXAMPLE_KBI);

			}
			else
			{
				countFreq1=150;
			}


		}
	else if(GPIO_PinRead(DOOR_SENSOR_GPIO,DOOR_SENSOR_PIN)==1 && checkLed==0)
		{

        	SDK_DelayAtLeastUs(countFreq1, CLOCK_GetFreq(kCLOCK_CoreSysClk)); // Delay 1s
			GPIO_PinWrite(TRIAC_LAMP_GPIO,TRIAC_LAMP_PIN,1);
			SDK_DelayAtLeastUs(10, CLOCK_GetFreq(kCLOCK_CoreSysClk)); // Delay 1s
			GPIO_PinWrite(TRIAC_LAMP_GPIO,TRIAC_LAMP_PIN,0);

			if (countFreq1 <8000)
			{
				countFreq1=countFreq1+30;
				KBI_ClearInterruptFlag(EXAMPLE_KBI);
			}
			else {
				//countFreq1=9000;
				check=1;
				FTM_StopTimer(BOARD_FTM_BASEADDR);
				NVIC_DisableIRQ(KBI0_IRQn);
				TPM_DisableInterrupts(BOARD_TPM_0, kTPM_TimeOverflowInterruptEnable);

			}
		}
    }
}

volatile bool mode =0;
volatile bool door_open = false; // Biến trạng thái cửa
uint8_t intPart;

void TPM0_IRQHandler(void)
{
    /* Clear interrupt flag.*/
    TPM_ClearStatusFlags(TPM0, kTPM_TimeOverflowFlag);


    __DSB();
}


int main(void) {
    init_GPIO();
    tpm_config_t tpmInfo;


    BOARD_InitBootPins();
    BOARD_InitBootClocks();

    BOARD_InitDebugConsole();
    BOARD_InitBootPeripherals();

   // FTM0_Init();


    FTM_GetDefaultConfig(&ftmInfo);
    /* Initialize FTM module */
    FTM_Init(FTM0, &ftmInfo);

    /* Setup dual-edge capture on a FTM channel pair */
    FTM_SetupInputCapture(FTM0, kFTM_Chnl_0, kFTM_FallingEdge, 0);

    /* Set the timer to be in free-running mode */
    FTM_SetTimerPeriod(FTM0, 0xFFFF);

    /* Enable channel interrupt when the second edge is detected */
    FTM_EnableInterrupts(FTM0, FTM_CHANNEL_INTERRUPT_ENABLE);

    /* Enable at the NVIC */
    EnableIRQ(FTM_INTERRUPT_NUMBER);

    FTM_StartTimer(BOARD_FTM_BASEADDR, kFTM_SystemClock);



//    kbi_config_t kbiConfig;
//    kbiConfig.mode        = kKBI_EdgesDetect;
//    kbiConfig.pinsEnabled = 1 << EXAMPLE_KBI_PINS;
//    kbiConfig.pinsEdge    = (0 << EXAMPLE_KBI_PINS);
//    NVIC_DisableIRQ(KBI0_IRQn);  // Vô hiệu hóa ngắt KBI0 trong NVIC
//    KBI_Init(EXAMPLE_KBI, &kbiConfig);

    Config_UART();
    Config_PWM_NEWTRAL_LINE();



//  TPM_GetDefaultConfig(&tpmInfo);
//  TPM_Init(TPM1, &tpmInfo);
//  TPM_SetupInputCapture(TPM1, kTPM_Chnl_1, kTPM_RiseAndFallEdge);
//  /* Enable channel interrupt when the second edge is detected */
//  TPM_EnableInterrupts(DEMO_TPM_BASEADDR, TPM_CHANNEL_INTERRUPT_ENABLE);
//
//  /* Enable at the NVIC */
//  EnableIRQ(TPM_INTERRUPT_NUMBER1);
//
//  TPM_StartTimer(DEMO_TPM_BASEADDR, kTPM_SystemClock);

    //NVIC_SetPriority(KBI0_IRQn,0);
    UART_WriteBlocking(DEMO_UART,g_tipString,sizeof(g_tipString));
    Config_ADC();
    Config_Timer();
    while(1) {
    	//delay_ms(1000);
 if(GPIO_PinRead(DOOR_SENSOR_GPIO,DOOR_SENSOR_PIN)==0)
 {
	 NVIC_EnableIRQ(KBI0_IRQn);
	 GPIO_PinWrite(BACK_FAN_GPIO, BACK_FAN_PIN, 0U);
	 GPIO_PinWrite(TOP_HEATER_OUTER_GPIO, TOP_HEATER_OUTER_PIN, 0U);
	 GPIO_PinWrite(BOTTOM_HEATER_GPIO, BOTTOM_HEATER_PIN, 0U);
	 PIT_DisableInterrupts(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL_1, kPIT_TimerInterruptEnable);
	 FTM_StartTimer(BOARD_FTM_BASEADDR, kFTM_SystemClock);
	 }
 else
 {
		//PIT_EnableInterrupts(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL_1, kPIT_TimerInterruptEnable);
		//PIT_EnableInterrupts(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL, kPIT_TimerInterruptEnable);
if(Check_active==1)
{
	PIT_EnableInterrupts(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL_1, kPIT_TimerInterruptEnable);
	//PIT_EnableInterrupts(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL, kPIT_TimerInterruptEnable);
    GPIO_PinWrite(TOP_FAN_GPIO, TOP_FAN_PIN, 1U);


}
else
{
	 PIT_DisableInterrupts(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL_1, kPIT_TimerInterruptEnable);
	// PIT_DisableInterrupts(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL, kPIT_TimerInterruptEnable);

 	if(intPart>100)
 	{
 	    GPIO_PinWrite(TOP_FAN_GPIO, TOP_FAN_PIN, 1U);

 	}
 	else
 	{
 	    GPIO_PinWrite(TOP_FAN_GPIO, TOP_FAN_PIN, 0U);

 	}

}
	FTM_StartTimer(BOARD_FTM_BASEADDR, kFTM_SystemClock);
	}
 if(g_AdcConversionDoneFlag)
 {
	    g_AdcConversionDoneFlag = false;
 }



//state=0;
 if(state==1)
 {

 }

__asm volatile ("nop");
    }
return 0 ;
}







