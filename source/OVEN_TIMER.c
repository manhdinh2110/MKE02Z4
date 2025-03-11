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
#include "OVEN_OPERATING.h"

uint8_t SP=100;
bool g_AdcConversionDoneFlag;





volatile bool tpmIsrFlag           = false;
volatile uint32_t milisecondCounts = 0U;
extern float Temperature;

uint8_t Check_mode;

#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)

uint32_t cnt;
uint32_t loop       = 2;
uint32_t secondLoop = 1000U;
const char *signals = "-|";

uint16_t g_AdcConversionValue;
extern uint8_t intPart;
bool led_state;


uint8_t buffer_adc1[100];
void Config_Timer(void)
{





	tpm_config_t tpmInfo;
////tpm_config_t tpmInfo1;
////
   TPM_GetDefaultConfig(&tpmInfo);
//  // TPM_GetDefaultConfig(&tpmInfo1);
////
    tpmInfo.prescale = TPM_PRESCALER_0;
////   tpmInfo1.prescale = TPM_PRESCALER_1;
////
////    /* Initialize TPM module */
    TPM_Init(BOARD_TPM_0, &tpmInfo);
//  // TPM_Init(BOARD_TPM_1, &tpmInfo1);
////
////    /* Set timer period */
    TPM_SetTimerPeriod(BOARD_TPM_0, USEC_TO_COUNT(DEMO_TIMER_PERIOD_US, TPM_SOURCE_CLOCK / (1U << tpmInfo.prescale)));
//    //TPM_SetTimerPeriod(BOARD_TPM_1, USEC_TO_COUNT(DEMO_TIMER_PERIOD_US, TPM_SOURCE_CLOCK / (1U << tpmInfo1.prescale)));
////
//  //  TPM_EnableInterrupts(BOARD_TPM_1, kTPM_TimeOverflowInterruptEnable);
////
    EnableIRQ(BOARD_TPM_IRQ_NUM_0);
//    //EnableIRQ(BOARD_TPM_IRQ_NUM_1);
////
    //TPM_StartTimer(BOARD_TPM_0, kTPM_SystemClock);
   // TPM_StartTimer(BOARD_TPM_1, kTPM_SystemClock);

   pit_config_t pitConfig;
//
  PIT_GetDefaultConfig(&pitConfig);
//
//        /* Init pit module */
       PIT_Init(DEMO_PIT_BASEADDR, &pitConfig);
//
//        /* Set timer period for channel 0 */
      PIT_SetTimerPeriod(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL, USEC_TO_COUNT(200000U, PIT_SOURCE_CLOCK));
//
//        /* Enable timer interrupts for channel 0 */
//
//        /* Enable at the NVIC */
    //    EnableIRQ(PIT_IRQ_ID);
//
     // PIT_StartTimer(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL);

//
//
       PIT_SetTimerPeriod(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL_1, USEC_TO_COUNT(1000000, PIT_SOURCE_CLOCK));
////
////        /* Enable timer interrupts for channel 0 */


        //NVIC_SetPriority(PIT_IRQ_ID_1, 2);  //
        //NVIC_SetPriority(PIT_IRQ_ID, 2);  //

////        /* Enable at the NVIC */
      EnableIRQ(PIT_IRQ_ID_1);
      EnableIRQ(PIT_IRQ_ID);

     PIT_StartTimer(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL_1);
       PIT_StartTimer(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL);
       PIT_EnableInterrupts(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL, kPIT_TimerInterruptEnable);

   	// PIT_StartTimer(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL_1);
}

adc_channel_config_t adcChannelConfigStruct;

uint32_t count=0;
uint8_t checkLed;
volatile bool pitDone = false;

//void delay_ms(uint16_t ms)
//{
//    uint32_t start = count;
//while((count-start)<ms)
//{
//	__asm volatile ("nop");
//	}
//UART_WriteBlocking(DEMO_UART,"TEST",sizeof("TEST"));
//
//
//
//}
//void TPM0_IRQHandler(void)
//{
//    /* Clear interrupt flag.*/
//    TPM_ClearStatusFlags(TPM0, kTPM_TimeOverflowFlag);
//    count++;
// //   delay_ms(10);
//    if(count>80000)
//    {
//    	checkLed=0;
//        NVIC_EnableIRQ(KBI0_IRQn);  // Vô hiệu hóa ngắt KBI0 trong NVIC
//    	FTM_StartTimer(BOARD_FTM_BASEADDR, kFTM_SystemClock);
// 	  count=0;
//    }
//
//    __DSB();
//}




void Config_ADC(void)
{

    adc_config_t adcConfigStruct;
   // adc_config_t adcConfigStruct1;

    EnableIRQ(DEMO_ADC_IRQn);

    ADC_GetDefaultConfig(&adcConfigStruct);
    //ADC_GetDefaultConfig(&adcConfigStruct1);
  //  ADC->SC3 = ADC_SC3_ADLSTS(1);  // Chọn thời gian lấy mẫu ngắn nhất

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
       //  NVIC_SetPriority(DEMO_ADC_IRQn, 1);   // Ưu tiên thấp hơn
        // NVIC_EnableIRQ(DEMO_ADC_IRQn);  // Bật ngắt ADC



         ADC_SetChannelConfig(DEMO_ADC_BASE, &adcChannelConfigStruct);
         ADC_EnableAnalogInput(DEMO_ADC_BASE, 1U << DEMO_ADC_USER_CHANNEL, true);
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
float Voltage;
uint8_t checkLed;
void PIT_CH1_IRQHandler(void)
{
//    /* Clear interrupt flag.*/
   PIT_ClearStatusFlags(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL_1, kPIT_TimerFlag);
//    /* Added for, and affects, all PIT handlers. For CPU clock which is much larger than the IP bus clock,
//     * CPU can run out of the interrupt handler before the interrupt flag being cleared, resulting in the
//     * CPU's entering the handler again and again. Adding DSB can prevent the issue from happening.
//     */

  // ADC_SetChannelConfig(DEMO_ADC_BASE, &adcChannelConfigStruct);
 //
//  // Covert_ADC_to_Temper(g_AdcConversionValue);
//

//UART_WriteBlocking(DEMO_UART, intPart, sizeof(intPart));
UART_WriteBlocking(DEMO_UART, buffer_adc1, strlen(buffer_adc1));
Conventional(intPart);


  // snprintf(buffer_adc1, sizeof(buffer_adc1), "Voltage: %f\n", Voltage);
  	//  PRINTF("OK");

// sprintf(buffer_adc1, "Temperature: %d\n",g_AdcConversionValue);

    __DSB();
}

uint8_t buffer_adc1[100];

uint8_t temp=0;


void Control_Temperature(uint16_t g_AdcConversionValue)
{

	 float Voltage = (g_AdcConversionValue * 3.3) / 4095;
		  //   //float Voltage=0.771;
	    Temperature=18*Voltage*Voltage+142.68*Voltage-18.774;
		//

//	   if(Check_mode==1)
//	   {
//
//		   Conventional(Temperature);
//	   }


}

#define SAMPLE_COUNT 4

static float temperatureSamples[SAMPLE_COUNT];
static int sampleIndex = 0;
uint8_t Check_active;
uint8_t buffe_CH0[10];
uint8_t start1=0;

void PIT_CH0_IRQHandler(void)
{
    /* Clear interrupt flag.*/
    /* Added for, and affects, all PIT handlers. For CPU clock which is much larger than the IP bus clock,
     * CPU can run out of the interrupt handler before the interrupt flag being cleared, resulting in the
     * CPU's entering the handler again and again. Adding DSB can prevent the issue from happening.
     */
    PIT_ClearStatusFlags(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL, kPIT_TimerFlag);
    ADC_SetChannelConfig(DEMO_ADC_BASE, &adcChannelConfigStruct);
   /// start1 = check_timer;
   //if(check_timer-start1>100)    led_state =1;
    //delay_ms();

//    while (!g_AdcConversionDoneFlag)
//    {
//    }
    g_AdcConversionValue = ADC_GetChannelConversionValue(DEMO_ADC_BASE);

	 float Voltage = (g_AdcConversionValue * 3.3) / 4095;
		  //   //float Voltage=0.771;
		    Temperature=18*Voltage*Voltage+142.68*Voltage-18.774;
	  //  	Control_Temperature(g_AdcConversionValue);
		    temperatureSamples[sampleIndex] = Temperature;
		    sampleIndex++;
		    if (sampleIndex >= SAMPLE_COUNT)
		        {
		            float sum = 0;
		            for (int i = 0; i < SAMPLE_COUNT; i++)
		            {
		                sum += temperatureSamples[i];
		            }
		            float avgTemperature = sum / SAMPLE_COUNT;

		            sampleIndex = 0;

		            intPart = (int)avgTemperature;
		            sprintf(buffer_adc1, "%2d\n",intPart);

		        }
		    if(GPIO_PinRead(DOOR_SENSOR_GPIO,DOOR_SENSOR_PIN)==0)
		    {
		    	if(checkLed==1)
		    	{
		    	    GPIO_PinWrite(BACK_FAN_GPIO, BACK_FAN_PIN, 0U);
		    	    GPIO_PinWrite(TOP_HEATER_OUTER_GPIO, TOP_HEATER_OUTER_PIN, 0U);
		    	   GPIO_PinWrite(BOTTOM_HEATER_GPIO, BOTTOM_HEATER_PIN, 0U);
		    	 PIT_DisableInterrupts(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL_1, kPIT_TimerInterruptEnable);

		    	}
		    	}
		    else
		    {
		    	if(Check_active==0)
		    			    {
		    		    	    GPIO_PinWrite(BACK_FAN_GPIO, BACK_FAN_PIN, 0U);
		    		    	    GPIO_PinWrite(TOP_HEATER_OUTER_GPIO, TOP_HEATER_OUTER_PIN, 0U);
		    		    	   GPIO_PinWrite(BOTTOM_HEATER_GPIO, BOTTOM_HEATER_PIN, 0U);
		    			    	 PIT_DisableInterrupts(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL_1, kPIT_TimerInterruptEnable);
		    						//NVIC_DisableIRQ(KBI0_IRQn);

		    			    }
		    	else
		    	{
			    	PIT_EnableInterrupts(DEMO_PIT_BASEADDR, DEMO_PIT_CHANNEL_1, kPIT_TimerInterruptEnable);

		    	}

		    }

////sprintf(buffer_adc1, "	GIA TRI DIEN TRO %d\n",ADC_GetChannelConversionValue(DEMO_ADC_BASE));
////UART_WriteBlocking(DEMO_UART, buffer_adc1, sizeof(buffer_adc1));


   // ADC_SetChannelConfig(DEMO_ADC_BASE, &adcChannelConfigStruct);
  //  PIT_StopTimer(PIT, kPIT_Chnl_0);



    __DSB();
}



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

