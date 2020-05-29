/**

assumption:
- there shall be 1 ignition event every crank revolution
- this ignition event will light up cylinder #1 or #2
- ignition system shall work independently from fuel injection

*/
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"
#include "ignition_hw.h"
#include "ignition_logic.h"
#include "decoder_logic.h"
#include "scheduler.h"
#include "Tuareg.h"
#include "uart.h"
#include "table.h"
#include "rotation_calc.h"



/**
calculates the position from which the delay, the scheduler has to provide, will be minimal
e.g. for dwell and ignition timing
*/
void fit_position( U32 Period_us, U32 Crank_angle_deg, engine_position_t * pTarget_position, U32 * pTarget_delay_us)
{
    engine_position_t closest;
    U32 smallest_delta_deg = 0xFFFFFFFF;
    U32 position, margin_deg, corresp_angle_deg, corresp_delta_deg;

    //get the safety margin in deg
    margin_deg= calc_rot_angle_deg(MARGIN_US, Period_us);

    //processing delay makes the required trigger position "earlier"
    if(margin_deg > Crank_angle_deg)
    {
        //for small positions trigger is before TDC
        Crank_angle_deg= 360 - (margin_deg - Crank_angle_deg);
    }
    else
    {
        Crank_angle_deg -= margin_deg;
    }

    //for all possible positions
    for(position=0; position < POSITION_COUNT; position++)
    {
        corresp_angle_deg= Tuareg.decoder->crank_position_table_deg[position];

        //if position is "earlier"
        if(Crank_angle_deg > corresp_angle_deg)
        {
            //get the angular difference
            corresp_delta_deg= Crank_angle_deg - corresp_angle_deg;

            //check if this is the minimal angular difference seen
            if(corresp_delta_deg < smallest_delta_deg)
            {
                /**
                bingo!
                */
                smallest_delta_deg= corresp_delta_deg;
                closest= position;
            }
        }
    }

    //export data
    * pTarget_position= closest;
    * pTarget_delay_us= calc_rot_duration_us(smallest_delta_deg, Period_us);
}

/**
provides an ignition timing which will allow engine operation
e.g. while LIMP HOME
*/
void default_ignition_timing(volatile ignition_timing_t * pTarget)
{
    /**
    late ignition
    */
    pTarget->coil_dwell_pos= DEFAULT_DWELL_POSITION;
    pTarget->coil_ignition_pos= DEFAULT_IGNITION_POSITION;
    pTarget->coil_dwell_timing_us= 0;
    pTarget->coil_ignition_timing_us= 0;

    pTarget->dwell_deg= DEFAULT_DWELL_DEG;
    pTarget->ignition_advance_deg= DEFAULT_ADVANCE_DEG;

}

/**
calculates the ignition timing for the next engine cycle
at a given rpm (from ignition_timing_t) and writes it there

input:
- engine rpm
- crank rotational period
- error states
- MAP


uses the more accurate rotation period value crank_T_us for calculations

this function shall never be called in LIMP Mode

problem on low rpms:    if coil_on and coil_off trigger on the same engine_position AND
                        dwell < segment_duration no spark will be generated
                        due to scheduler allocation error
                        n_crit= d/(6*dwell)
                        (1666 rpm at 5ms dwell + 50° segment)
            solution:   do not use the scheduler for dwell timing,
                        turn the coil on right on engine position
                        (extending dwell)

problem on high rpms:   with a large dwell AND large ignition advance
                        coil_on event can leave the engine cycle (dwell advance > 360°)
                        *not feasible with 2 cylinders and 1 coil*
            solution:   clip dwell advance to 360°
                        (cutting dwell)

*/
void calc_ignition_timing(volatile ignition_timing_t * pTarget, U32 Period_us, U32 Rpm)
{
    #warning TODO (oli#1#):check ignition calculation

    U32 position_deg, timing, advance_deg;
    engine_position_t position;

    if(Rpm > DYNAMIC_MIN_RPM)
    {
        /**
        calculate advance (off timing)
        */
        advance_deg= table3D_getValue(&ignitionTable, Rpm, Tuareg_get_asensor(ASENSOR_MAP) );
        position_deg= 360 - advance_deg;

        pTarget->ignition_advance_deg= advance_deg;

        fit_position(Period_us, position_deg, &(pTarget->coil_ignition_pos), &(pTarget->coil_ignition_timing_us));


        /**
        calculate dwell (on timing)
        */
        advance_deg += calc_rot_angle_deg(DYNAMIC_DWELL_US, Period_us);

        /**
        clip dwell to crank cycle
        */
        if(advance_deg > 360)
        {
            advance_deg= 360;
        }


       // position_deg= target_timing->ignition_advance_deg +
       // target_timing->dwell_advance_deg=

        //fit_position(target_timing->crank_period_us, target_timing->dwell_advance_deg, &target_timing->coil_on_pos, &target_timing->coil_on_timing_us );

        /**
        account for scheduler allocation
        */
       // if(target_timing->coil_on_pos == target_timing->coil_off_pos)
      //  {
        //    target_timing->coil_on_timing_us=0;
       // }

    }
    else
    {
        /**
        use fixed ignition triggers while cranking
        late ignition
        */
        default_ignition_timing(pTarget);
    }
}




/**
    using
    -GPIOC6 for ignition coil 1

    -use EXTI IRQ 2 for ignition timing
     recalculation after spark has fired

*/
/*
void init_ignition_logic(volatile ignition_timing_t * initial_timing)
{
    //provide initial ignition timing
    initial_timing->rpm= 0;
    calc_ignition_timings(initial_timing);
}
*/



void trigger_coil_by_timer(U32 delay_us, U32 level)
{
    if(delay_us == 0)
    {
        // immediate trigger
        set_ignition_ch1(level);
    }
    else
    {
        scheduler_set_channel(IGN_CH1, level, delay_us);
    }
}

