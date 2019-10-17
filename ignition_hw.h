#ifndef IGNITIONHW_H_INCLUDED
#define IGNITIONHW_H_INCLUDED

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"


void init_ignition_hw();

void set_ignition_ch1(output_pin_t level);
void set_ignition_ch2(output_pin_t level);

#endif // IGNITIONHW_H_INCLUDED
