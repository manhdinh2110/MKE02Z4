#include "OVEN_TIMER.h"
#include "OVEN_ADC.h"

#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_tpm.h"
#include "fsl_pit.h"

#include "fsl_common.h"

#include "Defines.h"
#include "fsl_gpio.h"
#include "fsl_adc.h"
#include "fsl_debug_console.h"
#include "OVEN_PWM.h"
#include "fsl_ftm.h"

uint8_t SP=100;


volatile bool tpmIsrFlag           = false;
volatile uint32_t milisecondCounts = 0U;
extern float Temperature;

#define DEMO_PIT_BASEADDR PIT
#define DEMO_PIT_CHANNEL  kPIT_Chnl_0
#define PIT_LED_HANDLER   PIT_CH0_IRQHandler
#define PIT_IRQ_ID        PIT_CH0_IRQn


#define DEMO_PIT_CHANNEL_1  kPIT_Chnl_1
#define PIT_IRQ_ID_1        PIT_CH1_IRQn

#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)

uint32_t cnt;
uint32_t loop       = 2;
uint32_t secondLoop = 1000U;
const char *signals = "-|";

uint16_t g_AdcConversionValue;

void Config_Timer(void)
{
//	tpm_config_t tpmInfo;
////tpm_config_t tpmInfo1;
////
//    TPM_GetDefaultConfig(&tpmInfo);
//  // TPM_GetDefaultConfig(&tpmInfo1);
////
//    tpmInfo.prescale = TPM_PRESCALER_0;
////   tpmInfo1.prescale = TPM_PRESCALER_1;
////
////    /* Initialize TPM module */
//    TPM_Init(BOARD_TPM_0, &tpmInfo);
//  // TPM_Init(BOARD_TPM_1, &tpmInfo1);
////
////    /* Set timer period */
//    TPM_SetTimerPeriod(BOARD_TPM_0, USEC_TO_COUNT(DEMO_TIMER_PERIOD_US, TPM_SOURCE_CLOCK / (1U << tpmInfo.prescale)));
//    //TPM_SetTimerPeriod(BOARD_TPM_1, USEC_TO_COUNT(DEMO_TIMER_PERIOD_US, TPM_SOURCE_CLOCK / (1U << tpmInfo1.prescale)));
////
//    TPM_EnableInterrupts(BOARD_TPM_0, kTPM_TimeOverflowInterruptEnable);
//  //  TPM_EnableInterrupts(BOARD_TPM_1, kTPM_TimeOverflowInterruptEnable);
////
//    EnableIRQ(BOARD_TPM_IRQ_NUM_0);
//    //EnableIRQ(BOARD_TPM_IRQ_NUM_1);
////
//    TPM_StartTimer(BOARD_TPM_0, kTPM_SystemClock);
   // TPM_StartTimer(BOARD_TPM_1, kTPM_SystemClock);

   pit_config_t pitConfig;
//
  PIT_GetDefaultConfig(&pitConfig);
//
//        /* Init pit module */
       PIT_Init(DEMO_PIT_BASEADDR, &pitConfig);
//
//        /* Set timer period for channel 0 */
      PIT_SetTimerPeriod(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL, USEC_TO_COUNT(700000U, PIT_SOURCE_CLOCK));
//
//        /* Enable timer interrupts for channel 0 */
        PIT_EnableInterrupts(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL, kPIT_TimerInterruptEnable);
//
//        /* Enable at the NVIC */
        EnableIRQ(PIT_IRQ_ID);
//
      PIT_StartTimer(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL);

//
//
       PIT_SetTimerPeriod(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL_1, USEC_TO_COUNT(500000, PIT_SOURCE_CLOCK));
////
////        /* Enable timer interrupts for channel 0 */
        PIT_EnableInterrupts(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL_1, kPIT_TimerInterruptEnable);
////
////        /* Enable at the NVIC */
       EnableIRQ(PIT_IRQ_ID_1);
      PIT_StartTimer(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL_1);


}

adc_channel_config_t adcChannelConfigStruct;





void Config_ADC(void)
{

    adc_config_t adcConfigStruct;
   // adc_config_t adcConfigStruct1;

    EnableIRQ(DEMO_ADC_IRQn);

    ADC_GetDefaultConfig(&adcConfigStruct);
    //ADC_GetDefaultConfig(&adcConfigStruct1);

     // adcConfigStruct.ResolutionMode=g_Adc_8bitFullRange;
     // ADC->SC3 |= ADC_SC3_MODE(1);  // Chọn độ phân giải 12-bit
      adcConfigStruct.ResolutionMode = kADC_Resolution12BitMode; // Đổi thành 12-bit
     // adcConfigStruct1.ResolutionMode = kADC_Resolution12BitMode; // Đổi thành 12-bit

      ADC_Init(DEMO_ADC_BASE, &adcConfigStruct);
     // ADC_Init(DEMO_ADC_BASE, &adcConfigStruct1);

         /* Enable the releated analog pins. */
         ADC_EnableHardwareTrigger(DEMO_ADC_BASE, false);

         /* Clear interrupt flag.*/

         /* Configure the user channel and interrupt. */
         adcChannelConfigStruct.channelNumber                        = DEMO_ADC_USER_CHANNEL;
         adcChannelConfigStruct.enableInterruptOnConversionCompleted = true;
         adcChannelConfigStruct.enableContinuousConversion   = false;
         //g_AdcInterruptCounter                                       = 0U; /* Clear the interrupt counter. */

         ADC_SetChannelConfig(DEMO_ADC_BASE, &adcChannelConfigStruct);
         ADC_EnableAnalogInput(DEMO_ADC_BASE, 1U << DEMO_ADC_USER_CHANNEL, true);
       //  ADC_SetChannelConfig(DEMO_ADC_BASE, &adcChannelConfigStruct);

        // ADC_EnableInterrupts(ADC, kUART_TransmissionCompleteInterruptEnable);
}
uint8_t g_tipString1[] =
    "Uart functional API interrupt example\r\nDEMO OVEN LUX HOVO111\r\nSTART....\r\n";
//void BOARD_TPM_HANDLER_0(void)
//{
//
//    TPM_ClearStatusFlags(BOARD_TPM_0, kTPM_TimeOverflowFlag);
//    tpmIsrFlag = true;
//    //GPIO_PortToggle(TOP_HEATER_OUTER_GPIO, TOP_HEATER_OUTER_PIN);
//   GPIO_PinWrite(TOP_HEATER_OUTER_GPIO, TOP_HEATER_OUTER_PIN, 1U);
//    Delay_ms(100);
//   // GPIO_PinWrite(TOP_HEATER_OUTER_GPIO, TOP_HEATER_OUTER_PIN, 0U);
//    //Delay_ms(100);
//
//    ADC_SetChannelConfig(DEMO_ADC_BASE, &adcChannelConfigStruct);
//
//    __DSB();
//}

uint8_t temp=0;
void PIT_CH0_IRQHandler(void)
{
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL, kPIT_TimerFlag);
    /* Added for, and affects, all PIT handlers. For CPU clock which is much larger than the IP bus clock,
     * CPU can run out of the interrupt handler before the interrupt flag being cleared, resulting in the
     * CPU's entering the handler again and again. Adding DSB can prevent the issue from happening.
     */

    ADC_SetChannelConfig(DEMO_ADC_BASE, &adcChannelConfigStruct);
  //  PIT_StopTimer(PIT, kPIT_Chnl_0);
  // UART_WriteBlocking(DEMO_UART, g_tipString1, sizeof(g_tipString1));
    __DSB();
}
uint8_t buffer_adc1[100];



//void updateHeaterControl(float temp) {
//    static bool lastStateWasCooling = false;
//    bool check=1;
//    int i=0;
//    	bool state;//
//    	if(state=0)
//    	{
//
//            heater1 = true;
//            heater2 = true;
//            state=1;
//            break;
//    	}
//
//    else if (temp = SP + 2 || temp > SP+2 &&state==1) {
//        heater1 = false;
//        heater2 = false;
//    }
//    else if (temp < SP-2) {
//        if (check==0 && state==1) {
//            heater1 = true;
//            heater2 = false;
//            check=1;
//    }
//        else if(check==1 &&state==1)
//            {
//                heater1 = false;
//                heater2 = true;
//                check==0;
//            }
//        }
//    }
//void Update_Relay(void)
//{
//	if(heater1==0)
//	{
//
//	}
////}
//
void PIT_CH1_IRQHandler(void)
{
//    /* Clear interrupt flag.*/
   PIT_ClearStatusFlags(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL_1, kPIT_TimerFlag);
//    /* Added for, and affects, all PIT handlers. For CPU clock which is much larger than the IP bus clock,
//     * CPU can run out of the interrupt handler before the interrupt flag being cleared, resulting in the
//     * CPU's entering the handler again and again. Adding DSB can prevent the issue from happening.
//     */
  // ADC_SetChannelConfig(DEMO_ADC_BASE, &adcChannelConfigStruct);
//   // UART_WriteBlocking(DEMO_UART, g_tipString1, sizeof(g_tipString1));
//  // Covert_ADC_to_Temper(g_AdcConversionValue);
//
   float Voltage = (g_AdcConversionValue * 3.3) / 4095;
  //   //float Voltage=0.771;
    Temperature=17.977*Voltage*Voltage+142.68*Voltage-18.874;
//
 if(Temperature < 100)
  {

      GPIO_PinWrite(TOP_HEATER_OUTER_GPIO, TOP_HEATER_OUTER_PIN, 1U);
    //  GPIO_PinWrite(TOP_HEATER_OUTER_GPIO, TOP_HEATER_OUTER_PIN, 0U);

     GPIO_PinWrite(BOTTOM_HEATER_GPIO, BOTTOM_HEATER_PIN, 1U);
    //  GPIO_PinWrite(BOTTOM_HEATER_GPIO, BOTTOM_HEATER_PIN, 0U);
 }
  else if(Temperature>100)
  {
	 //. FTM_StopTimer(BOARD_FTM_BASEADDR);
      GPIO_PinWrite(TOP_HEATER_OUTER_GPIO, TOP_HEATER_OUTER_PIN, 0U);
      GPIO_PinWrite(BOTTOM_HEATER_GPIO, BOTTOM_HEATER_PIN, 0U);
  }

 int intPart = (int)Temperature;
int fracPart = (int)((Temperature - intPart) * 100);

 sprintf(buffer_adc1, "Temperature: %2d.%02d\n",intPart,abs(fracPart));
 //UART_WriteBlocking(DEMO_UART, buffer_adc1, sizeof(buffer_adc1));


  // snprintf(buffer_adc1, sizeof(buffer_adc1), "Voltage: %f\n", Voltage);
  	//  PRINTF("OK");

// sprintf(buffer_adc1, "Temperature: %d\n",g_AdcConversionValue);

    __DSB();
}

//
//void BOARD_TPM_HANDLER_1(void)
//{
////	temp++;
////	if(temp%2==0)
////	{
////		    GPIO_PinWrite(TOP_HEATER_OUTER_GPIO, TOP_HEATER_OUTER_PIN, 1U);
////
////	}
////	else
////	{
////	    GPIO_PinWrite(TOP_HEATER_OUTER_GPIO, TOP_HEATER_OUTER_PIN, 0U);
////
////
////	}
//    TPM_ClearStatusFlags(BOARD_TPM_1, kTPM_TimeOverflowFlag);
//    tpmIsrFlag = true;
//   // GPIO_PinWrite(TOP_HEATER_OUTER_GPIO, TOP_HEATER_OUTER_PIN, 1U);
//  //  Covert_ADC_to_Temper(g_AdcConversionValue);
//    UART_WriteBlocking(DEMO_UART, g_tipString1, sizeof(g_tipString1));
//
//    //Delay_ms(100);
//    //Delay_ms(100);
//  //  ADC_SetChannelConfig(DEMO_ADC_BASE, &adcChannelConfigStruct);
//
//    __DSB();
//}


void Delay_ms(uint32_t ms) {
    volatile uint32_t i, j;
    for (i = 0; i < ms; i++) {
        for (j = 0; j < 4000; j++) {  // Số vòng lặp này cần điều chỉnh theo tốc độ CPU
            __asm("NOP"); // Lệnh NOP (No Operation) giúp tối ưu hóa vòng lặp
        }
    }
}

