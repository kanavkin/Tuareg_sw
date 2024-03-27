/**
Provides the access functions to MIL and tachometer
*/
#include <Tuareg_platform.h>
#include <Tuareg.h>

#include "dash_hw.h"

volatile dashctrl_t Dash;


/******************************************************************************************************************
TACH update function
******************************************************************************************************************/
void update_tachometer()
{
    VU32 compare;

    //tachometer control could be up to service mode
    if(Tuareg.flags.service_mode == true)
    {
        return;
    }

    /**
    do not steal the drivers attention in dash quiet mode -> show only'engine speed!
    */

    //look up the timer compare value to command
    compare= getValue_TachTable(Tuareg.Decoder.crank_rpm);

    //the tachometer output parameter is user input data
    if(compare > TACH_PWM_RESOLUTION) compare=0;

    //command dash hw layer
    set_tachometer_compare(compare);
}



/******************************************************************************************************************
Access function for MIL
******************************************************************************************************************/
void set_mil(volatile mil_state_t State)
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
        //activate the physical MIL lamp only if dash is not in quiet mode
        set_mil_hw( (Tuareg_Setup.flags.QuietDash == true)? ACTOR_UNPOWERED : ACTOR_POWERED );
    }
    else
    {
        set_mil_hw(ACTOR_UNPOWERED);

        //let dash update function enable blinking
        Dash.mil_cycle= 0;
    }

}


/******************************************************************************************************************
MIL periodic update function

indication scheme:

1. prio:    fatal error / service mode (no engine operation possible)   --> permanent
2. prio:    run inhibit set due to overheat / crash / sidestand         --> fast blink
3. prio:    limp mode                                                   --> slow blink


Quiet dash mode:
suppress activation of the physical MIL lamp. In Tunerstudio (and its logs) the real MIL state is indicated



******************************************************************************************************************/
void update_mil()
{
    /**
    calculate new mil state
    */
    if((Tuareg.errors.fatal_error == true) || (Tuareg.flags.service_mode == true))
    {
        set_mil(MIL_PERMANENT);
        return;
    }
    else if( (Tuareg.flags.run_allow == false) && (Tuareg.flags.run_switch_deactivated == false))
    {
        set_mil(MIL_BLINK_FAST);
    }
    else if(Tuareg.flags.limited_op == true)
    {
        set_mil(MIL_BLINK_SLOW);
    }
    else
    {
        set_mil(MIL_OFF);
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
                set_mil_hw(ACTOR_UNPOWERED);
                Dash.mil_cycle= MIL_BLINK_SLOW_OFF_ITV;
            }
            else
            {
                //activate the physical MIL lamp only if dash is not in quiet mode
                set_mil_hw( (Tuareg_Setup.flags.QuietDash == true)? ACTOR_UNPOWERED : ACTOR_POWERED );

                Dash.mil_cycle= MIL_BLINK_SLOW_ON_ITV;
            }

            break;

        case MIL_BLINK_FAST:

            if(Tuareg.flags.mil == true)
            {
                set_mil_hw(ACTOR_UNPOWERED);
                Dash.mil_cycle= MIL_BLINK_FAST_OFF_ITV;
            }
            else
            {
                //activate the physical MIL lamp only if dash is not in quiet mode
                set_mil_hw( (Tuareg_Setup.flags.QuietDash == true)? ACTOR_UNPOWERED : ACTOR_POWERED );

                Dash.mil_cycle= MIL_BLINK_FAST_ON_ITV;
            }

            break;


        default:
            break;
        }
    }
}

/******************************************************************************************************************
Init function
******************************************************************************************************************/
void init_dash()
{
    init_dash_hw();

    //turn the engine lamp off
    set_mil(MIL_OFF);
}

/******************************************************************************************************************
Update function

called every 100 ms from systick timer in interrupt context
******************************************************************************************************************/
void update_dash()
{
    update_mil();
    update_tachometer();
}

