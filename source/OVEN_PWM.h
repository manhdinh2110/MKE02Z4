/*
 * Copyright (c) 2025-2026, RitaVo Group, LUXHOVO
 * Copyright 2025-2026 RTV
 * All rights reserved.
 *
/-----------------------------------------------------------------------------------
 * Project OVEN LUXHOVO
 * File OVEN_PWM.h
 * Created on: 	Feb 27, 2025	                                                *
 * Version 1.0
 */

#ifndef OVEN_PWM_H_
#define OVEN_PWM_H_
#include "stdint.h"
#include "math.h"
#include "stdlib.h"

typedef struct
	{
		double Kp;
		double Ki;
		double Kd;
		double P_part;
		double I_part;
		double D_part;
		double Error;
		double pre_Error;
		double pre_pre_Error;
		double Output;
		double pre_Output;
	} PID;

	typedef struct
		{
			int32_t pulse;
			int32_t pre_pulse;
			int32_t delta_pulse;
			double Temper;
			int32_t Output;
		}TEMPER_HEATER;



#define T_Sample 		0.01


#define duty_max 		8399  //8399
#define duty_min 		-8400  //-8399




void Config_PWM_NEWTRAL_LINE(void);
int32_t Heater_Control_temper(PID* pid, double current,double setpoint);
void Temper_Control(void);




#endif
