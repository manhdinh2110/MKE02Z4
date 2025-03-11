#include "OVEN_OPERATING.h"
#include "Defines.h"
#include "fsl_gpio.h"
#include "fsl_port.h"

void Conventional(int Temperature)
	{



	 if(Temperature < 90)
	 	 	 {

		 	 	 GPIO_PinWrite(TOP_HEATER_OUTER_GPIO, TOP_HEATER_OUTER_PIN, 1U);
		 	 	 GPIO_PinWrite(BOTTOM_HEATER_GPIO, BOTTOM_HEATER_PIN, 1U);
	 	 	 }
	  else if(Temperature>95)
			 {
                  GPIO_PinWrite(TOP_HEATER_OUTER_GPIO, TOP_HEATER_OUTER_PIN, 0U);
                  GPIO_PinWrite(BOTTOM_HEATER_GPIO, BOTTOM_HEATER_PIN, 0U);
			 }
	  else
			 {
                  GPIO_PinWrite(TOP_HEATER_OUTER_GPIO, TOP_HEATER_OUTER_PIN, 0U);
                  GPIO_PinWrite(BOTTOM_HEATER_GPIO, BOTTOM_HEATER_PIN, 1U);
			 }

	}



