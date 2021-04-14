#ifndef FUELINGHW_H_INCLUDED
#define FUELINGHW_H_INCLUDED

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"

#include "Tuareg_types.h"

void init_fueling_hw();

void set_injector1(actor_control_t level);
void set_injector2(actor_control_t level);
void set_fuel_pump(actor_control_t level);

#endif // FUELINGHW_H_INCLUDED
