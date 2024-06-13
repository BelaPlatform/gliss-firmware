#pragma once
#include "main.h"

extern TIM_HandleTypeDef htim1;
#define gpioHtim (&htim1)
#define gpioHtimChannelIn 1
#define gpioHtimChannelOut 2
extern TIM_HandleTypeDef htim2;
#define neoPixelHtim htim2
#define neoPixelHtim_TIM_CHANNEL_x TIM_CHANNEL_4
#define neoPixelHtim_COUNTER_PERIOD TIM2_COUNTER_PERIOD
extern TIM_HandleTypeDef htim6;
#define dacAdcHtim htim6
extern UART_HandleTypeDef huart2;
#define dbgHuart huart2
extern I2C_HandleTypeDef hi2c1;
#define externalHi2c hi2c1
extern I2C_HandleTypeDef hi2c2;
#define trillHi2c hi2c2
extern DAC_HandleTypeDef hdac1;
#define dac0Handle hdac1
#define dac0Channel DAC_CHANNEL_1
#define dac1Handle hdac1
#define dac1Channel DAC_CHANNEL_2
extern ADC_HandleTypeDef hadc2;
#define adcHandle hadc2

#ifndef DEBUG2_Pin
#define DEBUG2_Pin 0 // backwards compatibility
#endif
