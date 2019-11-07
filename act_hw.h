#ifndef ACTHW_H_INCLUDED
#define ACTHW_H_INCLUDED

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"


extern void init_act_hw();

extern void set_act1(output_pin_t level);
extern void set_act2(output_pin_t level);

#endif // ACTHW_H_INCLUDED
