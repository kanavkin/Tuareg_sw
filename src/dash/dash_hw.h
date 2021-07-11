#ifndef DASHHW_H_INCLUDED
#define DASHHW_H_INCLUDED

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"


void init_dash_hw();

void set_mil(output_pin_t level);
void set_tachometer(output_pin_t level);

#endif // DASHHW_H_INCLUDED
