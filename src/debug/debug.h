#ifndef DEBUG_H_INCLUDED
#define DEBUG_H_INCLUDED

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"

#include "Tuareg_types.h"


void set_debug_led(output_pin_t level);
void set_debug_pin(output_pin_t level);
void init_debug_pins();
void enable_sysclk_check();


#endif // DEBUG_H_INCLUDED
