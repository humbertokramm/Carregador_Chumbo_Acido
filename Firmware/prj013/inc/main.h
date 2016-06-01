#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "stm32f0xx_nucleo.h"


extern TIM_HandleTypeDef    TimHandle;

void  interrupcaoTimer();

#endif /* __MAIN_H */

