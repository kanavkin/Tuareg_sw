/**
Provides the access functions to MIL and tachometer
*/
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "dash_hw.h"
#include "dash_logic.h"
#include "Tuareg.h"


volatile dashctrl_t Dash;



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
    if(State == Dash.mil)
    {
        //nothing to do
        return;
    }

    if(State < MIL_COUNT)
    {
        //store new state
        Dash.mil= State;
    }


    if(State == MIL_PERMANENT)
    {
        set_mil(PIN_ON);
    }
    else
    {
        set_mil(PIN_OFF);

        //let dash update function enable blinking
        Dash.mil_cycle= 0;
    }

}


/******************************************************************************************************************
Dash periodic update function


called every 100 ms from systick timer in interrupt context


indication scheme:

1. prio:    fatal error / service mode (no engine operation possible)   --> permanent
2. prio:    run inhibit set due to overheat / crash / sidestand         --> fast blink
3. prio:    limp mode                                                   --> slow blink





******************************************************************************************************************/
void update_dash()
{

    /**
    calculate new mil state
    */
    if((Tuareg.errors.fatal_error == true) || (Tuareg.flags.service_mode == true))
    {
        dash_set_mil(MIL_PERMANENT);
        return;
    }
    else if( (Tuareg.flags.run_inhibit == true) && ((Tuareg.flags.overheat_detected == true) || (Tuareg.flags.crash_sensor_triggered == true) || (Tuareg.flags.sidestand_sensor_triggered == true)) )
    {
        dash_set_mil(MIL_BLINK_FAST);
    }
    else if(Tuareg.flags.limited_op == true)
    {
        dash_set_mil(MIL_BLINK_SLOW);
    }
    else
    {
        dash_set_mil(MIL_OFF);
        return;
    }




    //check if interval has expired
    if(Dash.mil_cycle > 0)
    {
        Dash.mil_cycle--;
    }
    else
    {
        //interval has possibly expired, select action
        switch(Dash.mil)
        {

        case MIL_BLINK_SLOW:

            if(Tuareg.flags.mil == true)
            {
                set_mil(PIN_OFF);
                Dash.mil_cycle= MIL_BLINK_SLOW_OFF_ITV;
            }
            else
            {
                set_mil(PIN_ON);
                Dash.mil_cycle= MIL_BLINK_SLOW_ON_ITV;
            }

            break;

        case MIL_BLINK_FAST:

            if(Tuareg.flags.mil == true)
            {
                set_mil(PIN_OFF);
                Dash.mil_cycle= MIL_BLINK_FAST_OFF_ITV;
            }
            else
            {
                set_mil(PIN_ON);
                Dash.mil_cycle= MIL_BLINK_FAST_ON_ITV;
            }

            break;


        default:
            break;
        }
    }
}
