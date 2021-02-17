#ifndef DASHLOGIC_H_INCLUDED
#define DASHLOGIC_H_INCLUDED

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"

typedef enum {

    TACHOCTRL_CRANK,
    TACHOCTRL_0RPM,
    TACHOCTRL_500RPM,
    TACHOCTRL_1000RPM,
    TACHOCTRL_1500RPM,
    TACHOCTRL_2000RPM,
    TACHOCTRL_2500RPM,
    TACHOCTRL_3000RPM,
    TACHOCTRL_3500RPM,
    TACHOCTRL_4000RPM,
    TACHOCTRL_4500RPM,
    TACHOCTRL_5000RPM,
    TACHOCTRL_5500RPM,
    TACHOCTRL_6000RPM,
    TACHOCTRL_6500RPM,
    TACHOCTRL_7000RPM,
    TACHOCTRL_7500RPM,
    TACHOCTRL_8000RPM,
    TACHOCTRL_8500RPM,
    TACHOCTRL_9000RPM,
    TACHOCTRL_9500RPM

} tachoctrl_t;


typedef enum {

    USERLAMP_OFF,
    USERLAMP_PERMANENT,
    USERLAMP_BLINK_SLOW,
    USERLAMP_BLINK_MIDDLE,
    USERLAMP_BLINK_FAST,
    USERLAMP_CODE_TWO,
    USERLAMP_CODE_THREE,
    USERLAMP_CODE_FOUR,
    USERLAMP_CODE_FIVE,
    USERLAMP_CODE_SIX,
    USERLAMP_CODE_SEVEN,
    USERLAMP_CODE_EIGHT

} userlamp_t;


typedef struct {

    volatile tachoctrl_t tachoctrl;
    volatile userlamp_t userlamp;

} dashctrl_t;



void init_dash_logic();
void dash_set_tachometer(volatile tachoctrl_t State);
void dash_set_lamp(volatile userlamp_t State);

#endif // DASHLOGIC_H_INCLUDED
