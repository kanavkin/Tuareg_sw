#ifndef FUELHW_H_INCLUDED
#define FUELHW_H_INCLUDED

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"

#include "Tuareg_types.h"

void init_fuel_hw();

void set_injector1(actor_control_t level);
void set_injector2(actor_control_t level);
void set_fuel_pump(actor_control_t level);

//helper functions
void set_injector1_powered();
void set_injector1_unpowered();

void set_injector2_powered();
void set_injector2_unpowered();

void set_fuel_pump_powered();
void set_fuel_pump_unpowered();


#endif // FUELHW_H_INCLUDED
