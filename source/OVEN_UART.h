/*
 * Copyright (c) 2025-2026, RitaVo Group, LUXHOVO
 * Copyright 2025-2026 RTV
 * All rights reserved.
 *
/-----------------------------------------------------------------------------------
 * Project OVEN LUXHOVO
 * File OVEN_UART.h
 * Created on: 	Feb 25, 2025	                                                *
 * Version 1.0
 */

#ifndef OVEN_UART_H_
#define OVEN_UART_H_

#include "stdint.h"
#include "math.h"
#include "stdlib.h"
#include "fsl_uart.h"
#include "board.h"

uint32_t UART_ReadMultipleBytes(UART_Type *base, uint8_t *buffer, uint32_t maxLength);
void Config_UART(void);
void UART_WriteMultipleBytes(uint8_t *data, uint32_t length);



#endif






