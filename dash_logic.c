/**



*/
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"
#include "dash_hw.h"
#include "dash_logic.h"
#include "Tuareg.h"





/**


*/
void init_dash_logic()
{
    //turn the engine lamp on if a system failure was detected at CONFIG_LOAD
    if(Tuareg.Errors & TERROR_CONFIG)
    {
        dash_set_lamp(USERLAMP_PERMANENT);
    }

    dash_set_tachometer(TACHOCTRL_0RPM);




}


void dash_set_tachometer(volatile tachoctrl_t State)
{

}

void gen_tachometer_pulse(U32 Duration_us)
{

}


void dash_set_lamp(volatile userlamp_t State)
{

}
