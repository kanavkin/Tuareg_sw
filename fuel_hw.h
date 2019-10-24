#ifndef FUELHW_H_INCLUDED
#define FUELHW_H_INCLUDED

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"


extern void init_fuel_hw();

extern void set_injector_ch1(output_pin_t level);
extern void set_injector_ch2(output_pin_t level);

#endif // FUELHW_H_INCLUDED
