#ifndef DASHLOGIC_H_INCLUDED
#define DASHLOGIC_H_INCLUDED

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"


typedef enum {

    TACHOCTRL_CRANK,
    TACHOCTRL_0RPM

} tachoctrl_t;


typedef enum {

    MIL_OFF,
    MIL_PERMANENT,
    MIL_BLINK_SLOW,
    MIL_BLINK_MIDDLE,
    MIL_BLINK_FAST,
    MIL_CODE_TWO,
    MIL_CODE_THREE,
    MIL_CODE_FOUR,
    MIL_CODE_FIVE,
    MIL_CODE_SIX,
    MIL_CODE_SEVEN,
    MIL_CODE_EIGHT

} mil_state_t;


typedef struct {

    volatile tachoctrl_t tachoctrl;
    volatile mil_state_t mil;

} dashctrl_t;



void init_dash();
void dash_set_tachometer(volatile tachoctrl_t State);
void dash_set_mil(volatile mil_state_t State);

#endif // DASHLOGIC_H_INCLUDED
