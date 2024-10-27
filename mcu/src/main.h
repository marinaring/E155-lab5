// main.h
// Josh Brake
// jbrake@hmc.edu
// 10/31/22 

#ifndef MAIN_H
#define MAIN_H

#include "../lib/STM32L432KC.h"
#include <stm32l432xx.h>
#include <stdio.h>

///////////////////////////////////////////////////////////////////////////////
// Custom defines
///////////////////////////////////////////////////////////////////////////////

#define ENCODER_A PB7
#define ENCODER_B PA9
#define DELAY_TIM TIM2
#define MEASURE_TIM1 TIM6
#define MEASURE_TIM2 TIM15

#endif // MAIN_H