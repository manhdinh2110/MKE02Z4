#include "OVEN_PWM.h"
#include "fsl_ftm.h"
#include "Defines.h"


volatile bool brightnessUp        = true; /* Indicate LED is brighter or dimmer */
volatile uint8_t updatedDutycycle = 50U;
volatile uint8_t getCharValue     = 0U;

#ifndef DEMO_PWM_FREQUENCY
#define DEMO_PWM_FREQUENCY (3000U)
#endif

#define T_Sample 		0.01


#define duty_max 		8399  //8399
#define duty_min 		-8400  //-8399

//Function for
#define max 				5000
#define min 				0

float Temperature;

PID PID_Temper_1={1.0,0.3,0.2,0,0,0,0,0};
PID PID_Temper_2={1.0,0.3,0.2,0,0,0,0,0};
PID PID_Temper_3={1.0,0.3,0.2,0,0,0,0,0};
PID PID_Temper_4={1.0,0.3,0.2,0,0,0,0,0};

PID PID_Motor={1.0,0.3,0.2,0,0,0,0,0};



	TEMPER_HEATER temper_heater1;
	TEMPER_HEATER temper_heater2;
	TEMPER_HEATER temper_heater3;
	TEMPER_HEATER temper_heater4;




void Config_PWM_NEWTRAL_LINE(void)
{

    ftm_config_t ftmInfo;
    ftm_chnl_pwm_signal_param_t ftmParam;
    ftm_pwm_level_select_t pwmLevel = FTM_PWM_ON_LEVEL;

    FTM_GetDefaultConfig(&ftmInfo);
    ftmInfo.prescale = FTM_CalculateCounterClkDiv(BOARD_FTM_BASEADDR, DEMO_PWM_FREQUENCY, FTM_SOURCE_CLOCK);
    FTM_Init(BOARD_FTM_BASEADDR, &ftmInfo);
         /* Configure ftm params with frequency 24kHZ */
    ftmParam.chnlNumber            = BOARD_FTM_CHANNEL;
    ftmParam.level                 = pwmLevel;
    ftmParam.dutyCyclePercent      = updatedDutycycle;
    ftmParam.firstEdgeDelayPercent = 0U;

    ftmParam.enableComplementary   = false;
    ftmParam.enableDeadtime        = false;
    if (kStatus_Success !=
    FTM_SetupPwm(BOARD_FTM_BASEADDR, &ftmParam, 1U, kFTM_CenterAlignedPwm, DEMO_PWM_FREQUENCY, FTM_SOURCE_CLOCK))
    {
             // PRINTF("\r\nSetup PWM fail, please check the configuration parameters!\r\n");
     return -1;
    }
	//FTM_StartTimer(BOARD_FTM_BASEADDR, kFTM_SystemClock);

}

void Temper_Config(void)
{



}


int32_t Heater_Control_temper(PID* pid, double current,double setpoint)
	{
		pid->Error =setpoint-current;
		pid->P_part = pid->Kp*pid->Error;
		pid->I_part = 0.5*pid->Ki*T_Sample*(pid->Error + pid->pre_Error);
		pid->D_part = pid->Kd/T_Sample*( pid->Error - 2*pid->pre_Error+pid->pre_pre_Error);
		pid->Output = pid->pre_Output + pid->P_part + pid->I_part + pid->D_part ;
		pid->pre_pre_Error = pid->pre_Error;
		pid->pre_Error = pid->Error;
		if (pid->Output>duty_max)
			pid->Output=duty_max;
		if (pid->Output<duty_min)
		pid->Output=duty_min;
		pid->pre_Output = pid->Output;
		return pid->Output;

	}


void Temper_Control(void )
	{
	temper_heater1.Output=Heater_Control_temper(&PID_Temper_1,Temperature,60);
	temper_heater2.Output=Heater_Control_temper(&PID_Temper_2,Temperature,60);
	//temper_heater3.Output=Heater_Control_temper(&PID_Temper_2,50,60);
	//temper_heater4.Output=Heater_Control_temper(&PID_Temper_2,50,60);
			//Function Control for limit bottom and top
		if (PID_Temper_1.Output ==0)
		temper_heater1.Output = 0;
	    else if (temper_heater1.Output > max)
		temper_heater1.Output = max;
		else if (temper_heater1.Output < min)
		temper_heater1.Output = min;
	    //This is for Temper 2
		if (PID_Temper_2.Output == 0)
		temper_heater2.Output = 0;
		else if (temper_heater2.Output > max)
		temper_heater2.Output = max ;
		else if (temper_heater2.Output < min)
		temper_heater2.Output = min ;
	    //This is for Temper 3
//		if (PID_Temper_3.Output == 0)
//		temper_heater3.Output = 0;
//		else if (temper_heater3.Output > max)
//		temper_heater3.Output = max ;
//		else if (temper_heater3.Output < min)
//		temper_heater3.Output = min ;
//	    //This is for Temper 4
//		if (PID_Temper_4.Output == 0)
//		temper_heater4.Output = 0;
//		else if (temper_heater4.Output > max)
//		temper_heater4.Output = max ;
//		else if (temper_heater4.Output < min)
//		temper_heater4.Output = min ;


	}










