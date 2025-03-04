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
    "Uart functional API interrupt example\r\nDEMO OVEN LUX HOVO\r\nSTART....\r\n";

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
;


// Biến lưu giá trị sau khi tách
int temper;
int val1;
int val2;



uint8_t data;


void Operating_Mode()
{

}

#define TIMEOUT_LIMIT 50000
#define BUFFER_SIZE   64
#define MAX_WAIT_TIME 25  //

const uint32_t g_Adc_12bitFullRange = 4096U;



//Function for reading many bytes
//uint32_t UART_ReadMultipleBytes(uint8_t *buffer, uint32_t maxLength) {
//    uint32_t bytesRead = 0;
//    // Đọc từng byte cho đến khi đạt maxLength hoặc không còn dữ liệu
//    while (bytesRead < maxLength) {
//        if (kUART_RxDataRegFullFlag & UART_GetStatusFlags(DEMO_UART)) {
//            buffer[bytesRead] = UART_ReadByte(DEMO_UART); // Đọc 1 byte
//            bytesRead++;
//        } else {
//            break;
//        }
//    }
//
//    return bytesRead;
//
//}


/// @brief
/// @param

uint32_t numBytesRead = 0;
uint8_t uart_buffer[50] = {0};
uint8_t buffer_index = 0;


int val;

uint8_t buffer[10];

volatile bool g_AdcConversionDoneFlag;
extern uint16_t g_AdcConversionValue;
volatile uint32_t g_AdcInterruptCounter;
ftm_chnl_pwm_signal_param_t ftmParam;

void DEMO_UART_IRQHandler(void)
{

   // uint8_t data1[20];
	uint8_t Modeoper[5];  // Creat a array for result
    char * p;
    uint8_t data1[20];
    uint8_t testdata[5];
    uint8_t data;
    uint8_t first[10], second[10];
       uint32_t numBytes = sizeof(buffer);
      uint32_t numBytesRead = UART_ReadMultipleBytes(DEMO_UART,buffer, BUFFER_SIZE);

      //Function for Split data
    String_Split(buffer,',',0,testdata,sizeof(testdata));
    String_Split(buffer,',',1,Modeoper,sizeof(Modeoper));
    String_Split(buffer,',',2,first,sizeof(first));

    sprintf(data1, "Mode: %s, Temp: %s", testdata, Modeoper);
  //   UART_WriteBlocking( DEMO_UART,data1,sizeof(data1));
   PRINTF("%s", first);


    /* If new data arrived. */
//    if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(DEMO_UART))
//    {
//
//        /* If ring buffer is not full, add data to ring buffer. */
//        if (((rxIndex + 1) % DEMO_RING_BUFFER_SIZE) != txIndex)
//        {
//            demoRingBuffer[rxIndex] = data;
//            rxIndex++;
//            rxIndex %= DEMO_RING_BUFFER_SIZE;
//
//        }
//    }
    SDK_ISR_EXIT_BARRIER;
}

uint8_t buffer_adc[100];


void ADC_IRQHandler(void)
{
  //  g_AdcConversionDoneFlag = true;
    /* Read conversion result to clear the conversion completed flag. */
  g_AdcConversionValue = ADC_GetChannelConversionValue(DEMO_ADC_BASE);
  //  sprintf(buffer_adc, "	GIA TRI DIEN TRO %d\n",ADC_GetChannelConversionValue(DEMO_ADC_BASE));
    //UART_WriteBlocking(DEMO_UART, buffer_adc, sizeof(buffer_adc));

    SDK_ISR_EXIT_BARRIER;

}

volatile bool g_keypress = false;



void Config_PWM_NEWTRAL_LINE_1(void)
{

    ftm_config_t ftmInfo;
    ftm_chnl_pwm_signal_param_t ftmParam;

    ftm_pwm_level_select_t pwmLevel = FTM_PWM_ON_LEVEL;

    FTM_GetDefaultConfig(&ftmInfo);
    ftmInfo.prescale = FTM_CalculateCounterClkDiv(BOARD_FTM_BASEADDR_1, DEMO_PWM_FREQUENCY_1, FTM_SOURCE_CLOCK);
    FTM_Init(BOARD_FTM_BASEADDR, &ftmInfo);
         /* Configure ftm params with frequency 24kHZ */
    ftmParam.chnlNumber            = BOARD_FTM_CHANNEL_1;
    ftmParam.level                 = pwmLevel;
    ftmParam.dutyCyclePercent      = 90;
    ftmParam.firstEdgeDelayPercent = 0U;

    ftmParam.enableComplementary   = false;
    ftmParam.enableDeadtime        = false;
  //  if (kStatus_Success !=
    FTM_SetupPwm(BOARD_FTM_BASEADDR_1, &ftmParam, 1U, kFTM_CenterAlignedPwm, DEMO_PWM_FREQUENCY_1, FTM_SOURCE_CLOCK);
}

uint16_t countFreq1 = 8950;

void KBI0_IRQHandler(void)
{
    if (KBI_IsInterruptRequestDetected(EXAMPLE_KBI))
    {
		if (GPIO_PinRead(DOOR_SENSOR_GPIO,DOOR_SENSOR_PIN) == 0)
		{
			SDK_DelayAtLeastUs(countFreq1, CLOCK_GetFreq(kCLOCK_CoreSysClk));
			GPIO_PinWrite(TRIAC_LAMP_GPIO,TRIAC_LAMP_PIN,1);
			SDK_DelayAtLeastUs(10, CLOCK_GetFreq(kCLOCK_CoreSysClk));
			GPIO_PinWrite(TRIAC_LAMP_GPIO,TRIAC_LAMP_PIN,0);

			if (countFreq1 >100){
				KBI_ClearInterruptFlag(EXAMPLE_KBI);
				countFreq1-=100;
			}
			else if (countFreq1==100)
			{
				countFreq1=100;
			}
		}
		else
		{
        	SDK_DelayAtLeastUs(countFreq1, CLOCK_GetFreq(kCLOCK_CoreSysClk)); // Delay 1s
			GPIO_PinWrite(TRIAC_LAMP_GPIO,TRIAC_LAMP_PIN,1);
			SDK_DelayAtLeastUs(10, CLOCK_GetFreq(kCLOCK_CoreSysClk)); // Delay 1s
			GPIO_PinWrite(TRIAC_LAMP_GPIO,TRIAC_LAMP_PIN,0);
			if (countFreq1 <8950)
			{
				countFreq1=countFreq1+50;
				KBI_ClearInterruptFlag(EXAMPLE_KBI);
			}
			else {
				countFreq1 =9000;
				FTM_StopTimer(BOARD_FTM_BASEADDR);
			}
		}
    }
}



int main(void) {
//
//    uint8_t control;
//
//    uint8_t ch;
//


//Config interrupt
    kbi_config_t kbiConfig;
          //
//    gpio_pin_config_t gpioConfig = { kGPIO_DigitalInput, 0 };
//    SIM->SOPT &= ~SIM_SOPT_RSTPE_MASK;   // Vô hiệu hóa RESET trên PTA3
//
//    GPIO_PinInit(kGPIO_PORTA, 3, &gpioConfig);

    init_GPIO();


    BOARD_InitBootPins();
    BOARD_InitBootClocks();

    BOARD_InitDebugConsole();
    BOARD_InitBootPeripherals();
 //Config_PWM_NEWTRAL_LINE();

 //Fid github

    //Config_PWM_NEWTRAL_LINE_1();


   // ADC_Init_Config();
//    IRQ_EnableInterrupt(IRQ, true);
//    EnableIRQ(IRQ_IRQn);
//	UART_WriteBlocking(DEMO_UART,g_tipString,sizeof(g_tipString)-1);

  //  Config_ADC();
   // Config_Timer();
    kbiConfig.mode        = kKBI_EdgesDetect;
    kbiConfig.pinsEnabled = 1 << EXAMPLE_KBI_PINS;
    kbiConfig.pinsEdge    = 0 << EXAMPLE_KBI_PINS; /* Raising edge.*/
    KBI_Init(EXAMPLE_KBI, &kbiConfig);




       //C5
//       GPIO_PinWrite(TRIAC_LAMP_GPIO,TRIAC_LAMP_PIN,1U);
//       GPIO_PinWrite(TRIAC_LAMP_GPIO,TRIAC_LAMP_PIN,0U);
//       //E0
//       GPIO_PinWrite(TOP_FAN_GPIO, TOP_FAN_PIN, 1U);
//       GPIO_PinWrite(TOP_FAN_GPIO, TOP_FAN_PIN, 0U);
//       //E2
//       GPIO_PinWrite(UNKNOWN2_GPIO, UNKNOWN2_PIN, 1U);
//       GPIO_PinWrite(UNKNOWN2_GPIO, UNKNOWN2_PIN, 0U);
//       //D1
//       GPIO_PinWrite(TOP_HEATER_OUTER_GPIO, TOP_HEATER_OUTER_PIN, 1U);
//       GPIO_PinWrite(TOP_HEATER_OUTER_GPIO, TOP_HEATER_OUTER_PIN, 0U);
//       //D2
//       GPIO_PinWrite(TOP_HEATER_INNER_GPIO, TOP_HEATER_INNER_PIN, 1U);
//       GPIO_PinWrite(TOP_HEATER_INNER_GPIO, TOP_HEATER_INNER_PIN, 0U);
//       //D3
//       GPIO_PinWrite(BACK_HEATER_GPIO, BACK_HEATER_PIN, 1U);
//       GPIO_PinWrite(BACK_HEATER_GPIO, BACK_HEATER_PIN, 0U);
//       //D4
//       GPIO_PinWrite(BOTTOM_HEATER_GPIO, BOTTOM_HEATER_PIN, 1U);
//       GPIO_PinWrite(BOTTOM_HEATER_GPIO, BOTTOM_HEATER_PIN, 0U);
//       //D5
//       GPIO_PinWrite(BACK_FAN_GPIO, BACK_FAN_PIN, 1U);
//       GPIO_PinWrite(BACK_FAN_GPIO, BACK_FAN_PIN, 0U);

    //Config for SINGLE PWM
//    TPM_GetDefaultConfig(&tpmInfo);//Get value default for structure
//    /* Calculate the clock division based on the PWM frequency to be obtained */
//    tpmInfo.prescale = TPM_CalculateCounterClkDiv(FTM2, DEMO_PWM_FREQUENCY, TPM_SOURCE_CLOCK);
//    /* Initialize TPM module */
//   TPM_Init(BOARD_TPM_BASEADDR, &tpmInfo);
//    /* Configure tpm params with frequency 24kHZ */
//   tpmParam.chnlNumber = (tpm_chnl_t)BOARD_TPM_CHANNEL;

    //Config Interrupt Timer
    //TPM_EnableInterrupts(BOARD_TPM, kTPM_TimeOverflowInterruptEnable);

 //   EnableIRQ(BOARD_TPM_IRQ_NUM);

  //  TPM_StartTimer(BOARD_TPM, kTPM_SystemClock);


//    #if (defined(FSL_FEATURE_TPM_HAS_PAUSE_LEVEL_SELECT) && FSL_FEATURE_TPM_HAS_PAUSE_LEVEL_SELECT)
//    tpmParam.pauseLevel = kTPM_ClearOnPause;
//#endif
//   tpmParam.level            = TPM_LED_ON_LEVEL;
//   tpmParam.dutyCyclePercent = updatedDutycycle;
//  if (kStatus_Success !=
//        TPM_SetupPwm(BOARD_TPM_BASEADDR, &tpmParam, 1U, kTPM_CenterAlignedPwm, DEMO_PWM_FREQUENCY, TPM_SOURCE_CLOCK))
//    {
//
//        return -1;
//    }

  //  TPM_StartTimer(BOARD_TPM_BASEADDR, kTPM_SystemClock);

    /* Record channel PWM mode configure */
   // control = TPM_GetChannelContorlBits(BOARD_TPM_BASEADDR, (tpm_chnl_t)BOARD_TPM_CHANNEL);
    /* Enter an infinite loop, just incrementing a counter. */
//    /* Start to echo. */
//    sendXfer.data        = g_txBuffer;
//    sendXfer.dataSize    = ECHO_BUFFER_LENGTH;
//    receiveXfer.data     = g_rxBuffer;
//    receiveXfer.dataSize = ECHO_BUFFER_LENGTH;
  // TPM_UpdatePwmDutycycle(BOARD_TPM_BASEADDR, (tpm_chnl_t)BOARD_TPM_CHANNEL,
        	                                         //   kTPM_CenterAlignedPwm, 20U);
    	//TPM_EnableChannel(TPM2, (tpm_chnl_t)BOARD_TPM_CHANNEL, control);

	//SDK_DelayAtLeastUs(5000000, CLOCK_GetFreq(kCLOCK_CoreSysClk)); // Delay 1s
	//FTM_StopTimer(BOARD_FTM_BASEADDR);

    while(1) {
       //  GPIO_PinWrite(kGPIO_PORTB,0U,1);
//    	for (i=4;i<128;i++)
//
//    	{
//
//    	dimming=i;
//
//		SDK_DelayAtLeastUs(20000, CLOCK_GetFreq(kCLOCK_CoreSysClk)); // Delay 1s
//
//    	}
//    	//uint32_t value = (GPIOA->PDIR & (1U << 6)) ? 1 : 0;
//    	for (i=128;i>4;i--)
//
//    	{
//
//    	dimming=i;
//
//		SDK_DelayAtLeastUs(20000, CLOCK_GetFreq(kCLOCK_CoreSysClk)); // Delay 1s
//
//    	}

 if(GPIO_PinRead(DOOR_SENSOR_GPIO,DOOR_SENSOR_PIN)==0)
 {
	FTM_StartTimer(BOARD_FTM_BASEADDR, kFTM_SystemClock);
//		SDK_DelayAtLeastUs(4000000, CLOCK_GetFreq(kCLOCK_CoreSysClk)); // Delay 1s
//
}

//    	//GPIO_PinWrite(GPIOD,5U,1);
//    	if (state==0) // Kiểm tra nút nhấn
//    	{
//
//        	FTM_StartTimer(BOARD_FTM_BASEADDR, kFTM_SystemClock);
//   			SDK_DelayAtLeastUs(20000, CLOCK_GetFreq(kCLOCK_CoreSysClk)); // Delay 1s
//   			state = GPIO_PinRead(DOOR_SENSOR_GPIO,DOOR_SENSOR_PIN);
//    	} else {
//    		SDK_DelayAtLeastUs(20000, CLOCK_GetFreq(kCLOCK_CoreSysClk)); // Delay 1s
//    		state = GPIO_PinRead(DOOR_SENSOR_GPIO,DOOR_SENSOR_PIN);
//    	}

//    	if (countFreq1 <10)
//		{
//    		countFreq1 =1;
//		}
//
//    	if (countFreq > 10000)
//		{
//			countFreq = 10000;
//		}

//		if(state!=current_state)
//		{
//			countFreq1=10000;
//			countFreq=1;
//		}
		//current_state = state;
//    	}
//    	else
//    	{
//        	KBI_EnableInterrupts(EXAMPLE_KBI);
//
//    		//PRINTF("Open not door!\r\n");
//    	    //UART_WriteBlocking(DEMO_UART, "Open door  !\r\n", sizeof("Open door!\r\n") - 1);
//
//    	}

//        g_AdcConversionDoneFlag = false;
       // ADC_SetChannelConfig(DEMO_ADC_BASE, &adcChannelConfigStruct);
//           while (!g_AdcConversionDoneFlag)
//           {

          // GPIO_PortToggle(EXAMPLE_KBI_SIGNAL_INPUT_REF_GPIO, 1u << EXAMPLE_KBI_SIGNAL_INPUT_REF_GPIO_INDEX);

      //  IRQ_Deinit(IRQ);

       //    PRINTF("\r\n KBI Driver Example End.\r\n");

           //PRINTF("ADC Value: %d\r\n", g_AdcConversionValue);
    	 //GPIO_PinWrite(LED_GPIO,LED_PIN,1);
//    	/* Update PWM duty cycle */
//    	if (kStatus_Success ==
//    	{
//    	    PRINTF("The duty cycle was successfully updated!\r\n");
//    	}

    	/* Start channel output with updated dutycycle */

//        if ((!rxOnGoing) && rxBufferEmpty)
//        {
//            rxOnGoing = true;
//            UART_TransferReceiveNonBlocking(DEMO_UART, &g_uartHandle, &receiveXfer, NULL);
//        }
       //  UART_WriteBlocking(DEMO_UART, g_tipString, sizeof(g_tipString) / sizeof(g_tipString[0]));

        /* Send data only when UART TX register is empty and ring buffer has data to send out. */
//        while ((kUART_TxDataRegEmptyFlag & UART_GetStatusFlags(DEMO_UART)) && (rxIndex != txIndex))
//        {
//            UART_WriteByte(DEMO_UART, demoRingBuffer[txIndex]);
//            txIndex++;
//            txIndex %= DEMO_RING_BUFFER_SIZE;
//        }

        //        UART_ReadBlocking(DEMO_UART, &ch, 3);
       //        UART_WriteBlocking(DEMO_UART, &ch, 3);
      //  UART_WriteBlocking(DEMO_UART, txbuff, sizeof(txbuff) - 1);

//        while (!ADC_GetChannelStatusFlags(DEMO_ADC_BASE))
//        {
//        }
//        sprintf(buffer, "	GIA TRI DIEN TRO\n\r");
//        UART_WriteBlocking(DEMO_UART, (uint8_t *)buffer, strlen(buffer));

      //  PRINTF("ADC Value: %d\r\n", ADC_GetChannelConversionValue(DEMO_ADC_BASE));

//SDK_DelayAtLeastUs(500000, CLOCK_GetFreq(kCLOCK_CoreSysClk)); // Delay 1s

/* Disable channel output before updating the dutycycle */
//TPM_DisableChannel(BOARD_TPM_BASEADDR, (tpm_chnl_t)BOARD_TPM_CHANNEL);



__asm volatile ("nop");
    }
    return 0 ;
}


//void BOARD_TPM_HANDLER(void)
//{
//    /* Clear interrupt flag.*/
//    TPM_ClearStatusFlags(BOARD_TPM, kTPM_TimeOverflowFlag);
//    tpmIsrFlag = true;
//    __DSB();
//}






