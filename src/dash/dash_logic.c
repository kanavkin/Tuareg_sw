/**
Provides the access functions to MIL and tachometer
*/
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "dash_hw.h"
#include "dash_logic.h"
#include "Tuareg.h"


            //working!!
            //set_user_lamp(ON);
            //lowprio_scheduler_togglemode_channel(LOWPRIO_CH1, set_user_lamp, 1000, 5000);

            /*
            //working!!!
            set_debug_pin(ON);
            lowprio_scheduler_seqmode_channel(LOWPRIO_CH1, set_debug_pin, 1000, 5000, 10000, 3);
            */

            /*
            set_user_lamp(ON);
            lowprio_scheduler_seqmode_channel(LOWPRIO_CH1, set_user_lamp, 400000, 500000, 3000000, 4);
            */


/******************************************************************************************************************
Init function
******************************************************************************************************************/
void init_dash()
{
    init_dash_hw();

    //turn tachometer off
    dash_set_tachometer(TACHOCTRL_0RPM);

    //turn the engine lamp off
    dash_set_mil(MIL_OFF);

}



/******************************************************************************************************************
Access function for tachometer
******************************************************************************************************************/
void dash_set_tachometer(volatile tachoctrl_t State)
{

}

void gen_tachometer_pulse(U32 Duration_us)
{

}



/******************************************************************************************************************
Access function for MIL
******************************************************************************************************************/
void dash_set_mil(volatile mil_state_t State)
{
    switch(State)
    {
    case MIL_PERMANENT:
        set_mil(PIN_ON);
        break;


    case MIL_OFF:
        set_mil(PIN_OFF);
        break;


    default:
        set_mil(PIN_ON);
        break;

    }
}
