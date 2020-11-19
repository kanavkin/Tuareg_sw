#ifndef IGNITIONHW_H_INCLUDED
#define IGNITIONHW_H_INCLUDED

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

extern void init_ignition_hw();

extern void set_ignition_ch1(actor_control_t level);
extern void set_ignition_ch2(actor_control_t level);

//extern void trigger_ignition_irq();

#endif // IGNITIONHW_H_INCLUDED
