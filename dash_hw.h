#ifndef DASHHW_H_INCLUDED
#define DASHHW_H_INCLUDED

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"


extern void init_dash_hw();

extern void set_user_lamp(output_pin_t level);
extern void set_tachometer(output_pin_t level);

#endif // DASHHW_H_INCLUDED
