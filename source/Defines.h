


/* Interrupt number and interrupt handler for the FTM instance used */
#define FTM_INTERRUPT_NUMBER FTM2_IRQn
#define FTM_LED_HANDLER      FTM2_IRQHandler

/* Interrupt to enable */
#define FTM_CHANNEL_INTERRUPT_ENABLE kFTM_TimeOverflowInterruptEnable

/* Get source clock for FTM driver */
#define FTM_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)
#ifndef FTM_PWM_ON_LEVEL
#define FTM_PWM_ON_LEVEL kFTM_HighTrue
#endif


/* define instance */
#define BOARD_TPM_0 TPM0
/* Interrupt number and interrupt handler for the TPM instance used */
#define BOARD_TPM_IRQ_NUM_0 TPM0_IRQn
#define BOARD_TPM_HANDLER_0 TPM0_IRQHandler
/* Get source clock for TPM driver */
#define TPM_SOURCE_CLOCK (CLOCK_GetFreq(kCLOCK_BusClk) / 4)
#ifndef DEMO_TIMER_PERIOD_US
/* Set counter period to 1ms */
#define DEMO_TIMER_PERIOD_US (90000000U)
#endif
#ifndef TPM_PRESCALER
/* Calculate the clock division based on the PWM frequency to be obtained */
#define TPM_PRESCALER TPM_CalculateCounterClkDiv(BOARD_TPM, 1000000U / DEMO_TIMER_PERIOD_US, TPM_SOURCE_CLOCK);
#endif



#define BOARD_FTM_BASEADDR FTM2
#define BOARD_FTM_CHANNEL  kFTM_Chnl_2
#define BOARD_SECOND_TPM_CHANNEL 0U

#define BOARD_FTM_BASEADDR_1 FTM1
#define BOARD_FTM_CHANNEL_1  kFTM_Chnl_1


/* Interrupt to enable and flag to read; depends on the TPM channel used */
#define TPM_CHANNEL_INTERRUPT_ENABLE kTPM_Chnl0InterruptEnable
#define TPM_CHANNEL_FLAG             kTPM_Chnl0Flag

/* Interrupt number and interrupt handler for the TPM instance used */
#define TPM_INTERRUPT_NUMBER TPM2_IRQn
#define TPM_LED_HANDLER      TPM2_IRQHandler


#define DEMO_UART            UART1
#define DEMO_UART_CLK_FREQ   CLOCK_GetFreq(kCLOCK_BusClk)
#define DEMO_UART_IRQn       UART1_IRQn
#define DEMO_UART_IRQHandler UART1_IRQHandler


#define EXAMPLE_KBI                             KBI0
#define EXAMPLE_KBI_SIGNAL_INPUT_REF_GPIO       kGPIO_PORTA
#define EXAMPLE_KBI_SIGNAL_INPUT_REF_GPIO_INDEX 0
#define EXAMPLE_KBI_PINS                        (0)



/*! @brief Ring buffer size (Unit: Byte). */
#define DEMO_RING_BUFFER_SIZE 100





#define DEMO_UART          		UART1
#define DEMO_UART_CLK_FREQ 		CLOCK_GetFreq(kCLOCK_BusClk)

#define DEMO_ADC_BASE 			ADC
#define DEMO_ADC_USER_CHANNEL 	9U  // PTA1 = ADC0_SE1




#define INTERRUPT_50HZ_GPIO 	kGPIO_PORTA
#define INTERRUPT_50HZ_PIN 		0U


#define DOOR_SENSOR_GPIO 		kGPIO_PORTA
#define DOOR_SENSOR_PIN 		6U



#define MEAT_PROBE_GPIO  		kGPIO_PORTC
#define MEAT_PROBE_PIN  		0U

#define TRIAC_LAMP_GPIO 		kGPIO_PORTC
#define TRIAC_LAMP_PIN 			5U

#define PWM_NEUTRAL_LINE_GPIO 	kGPIO_PORTD
#define PWM_NEUTRAL_LINE_PIN 	0U

#define TOP_HEATER_OUTER_GPIO 	kGPIO_PORTD
#define TOP_HEATER_OUTER_PIN 	1U

#define TOP_HEATER_INNER_GPIO 	kGPIO_PORTD
#define TOP_HEATER_INNER_PIN 	2U

#define BACK_HEATER_GPIO 		kGPIO_PORTD
#define BACK_HEATER_PIN 		3U

#define BOTTOM_HEATER_GPIO 		kGPIO_PORTD
#define BOTTOM_HEATER_PIN 		4U

#define BACK_FAN_GPIO 			kGPIO_PORTD
#define BACK_FAN_PIN 			5U

#define TOP_FAN_GPIO 			kGPIO_PORTE
#define TOP_FAN_PIN  			0U

#define UNKNOWN2_GPIO 			kGPIO_PORTE
#define UNKNOWN2_PIN  			2U


