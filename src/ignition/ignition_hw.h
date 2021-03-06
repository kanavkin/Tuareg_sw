#ifndef IGNITIONHW_H_INCLUDED
#define IGNITIONHW_H_INCLUDED

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"


void init_ignition_hw();

void set_ignition_ch1(actor_control_t level);
void set_ignition_ch2(actor_control_t level);


//helper functions
void set_coil1_powered();
void set_coil1_unpowered();
void set_coil2_powered();
void set_coil2_unpowered();

void trigger_ignition_irq();



#endif // IGNITIONHW_H_INCLUDED
