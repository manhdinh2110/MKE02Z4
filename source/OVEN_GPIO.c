#include "OVEN_GPIO.h"
#include "fsl_gpio.h"
#include "Defines.h"

void init_GPIO(void)
{

    gpio_pin_config_t ouput_config = {
        kGPIO_DigitalOutput,
        0,
    };

    gpio_pin_config_t input_config = {
    		kGPIO_DigitalInput,
        0,
    };


    GPIO_PinInit(TRIAC_LAMP_GPIO, TRIAC_LAMP_PIN, &ouput_config);

    GPIO_PinInit(BACK_FAN_GPIO, BACK_FAN_PIN, &ouput_config);
    GPIO_PinInit(PWM_NEUTRAL_LINE_GPIO, PWM_NEUTRAL_LINE_PIN, &ouput_config);
    GPIO_PinInit(TRIAC_LAMP_GPIO, TRIAC_LAMP_PIN, &ouput_config);
    GPIO_PinInit(UNKNOWN2_GPIO, UNKNOWN2_PIN, &ouput_config);

    GPIO_PinInit(TOP_HEATER_OUTER_GPIO, TOP_HEATER_OUTER_PIN, &ouput_config);
    GPIO_PinInit(BOTTOM_HEATER_GPIO, BOTTOM_HEATER_PIN, &ouput_config);
    GPIO_PinInit(TOP_HEATER_INNER_GPIO, TOP_HEATER_INNER_PIN, &ouput_config);

    GPIO_PinInit(BACK_HEATER_GPIO, BACK_HEATER_PIN, &ouput_config);//D3
    GPIO_PinInit(DOOR_SENSOR_GPIO, DOOR_SENSOR_PIN, &input_config);//D3

    GPIO_PinInit(INTERRUPT_50HZ_GPIO, INTERRUPT_50HZ_PIN, &input_config);//D3

}
