/**



*/
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"
#include "dash_hw.h"
#include "dash_logic.h"
#include "Tuareg.h"


 ///// TODO (oli#9#): debug action enabled
            //working!!
            //set_user_lamp(ON);
            //lowprio_scheduler_togglemode_channel(LOWPRIO_CH1, set_user_lamp, 1000, 5000);

            /*
            //working!!!
            /// TODO (oli#9#): debug action enabled
            set_debug_pin(ON);
            lowprio_scheduler_seqmode_channel(LOWPRIO_CH1, set_debug_pin, 1000, 5000, 10000, 3);
            */

            ///// TODO (oli#3#): the first sequence triggered seems to be 1 pulse longer!
           // /// TODO (oli#9#): debug action enabled
            /*
            set_user_lamp(ON);
            lowprio_scheduler_seqmode_channel(LOWPRIO_CH1, set_user_lamp, 400000, 500000, 3000000, 4);
            */


/**


*/
void init_dash_logic()
{
    //turn the engine lamp on if a system failure was detected at CONFIG_LOAD
    /*
    if(Tuareg.errors & TERROR_CONFIG)
    {
        dash_set_lamp(USERLAMP_PERMANENT);
    }
    */

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
