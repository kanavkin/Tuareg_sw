/**

requirements:
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
#include "base_calc.h"
#include "config.h"


/**
calculates the position from which the delay, the scheduler has to provide, will be minimal
e.g. for dwell and ignition timing
*/
void fit_position( VU32 Period_us, VU32 Crank_angle_deg, volatile crank_position_t * pTarget_position, VU32 * pTarget_delay_us)
{
    crank_position_t closest;
    U32 smallest_delta_deg = 0xFFFFFFFF;
    U32 position, margin_deg, corresp_angle_deg, corresp_delta_deg;

    //get the safety margin in deg
    margin_deg= calc_rot_angle_deg(configPage13.safety_margin_us, Period_us);

    //processing delay makes the required trigger position "earlier"
    if(margin_deg > Crank_angle_deg)
    {
        //for small target crank angles: trigger is bTDC
        Crank_angle_deg= 360 - (margin_deg - Crank_angle_deg);
    }
    else
    {
        Crank_angle_deg -= margin_deg;
    }

    //for all possible positions
    for(position=0; position < CRK_POSITION_COUNT; position++)
    {
        corresp_angle_deg= Tuareg.process.crank_position_table.a_deg[position];

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
provides a ignition timing which will allow engine operation
e.g. while LIMP HOME
*/
void default_ignition_timing(volatile ignition_timing_t * pTarget)
{
    pTarget->coil_dwell_pos= DEFAULT_DWELL_POSITION;
    pTarget->coil_ignition_pos= DEFAULT_IGNITION_POSITION;
    pTarget->coil_dwell_timing_us= 0;
    pTarget->coil_ignition_timing_us= 0;
    pTarget->dwell_ms= 1;
    pTarget->dwell_deg= DEFAULT_DWELL_DEG;
    pTarget->ignition_advance_deg= DEFAULT_ADVANCE_DEG;
}

/**
provides a late ignition timing for cranking
*/
inline void cranking_ignition_timing(volatile ignition_timing_t * pTarget)
{
    pTarget->coil_ignition_pos= configPage13.idle_ignition_position;
    pTarget->coil_dwell_pos= configPage13.idle_dwell_position;
    pTarget->ignition_advance_deg= configPage13.idle_advance_deg;
    pTarget->dwell_deg= configPage13.idle_dwell_deg;
    pTarget->dwell_ms= 1;
    pTarget->coil_ignition_timing_us =0;
    pTarget->coil_dwell_timing_us =0;
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

            solution:   do not use the scheduler for dwell timing and turn the coil on right on engine position
                        (extending dwell)

problem on high rpms:   with a large dwell AND large ignition advance
                        coil_on event can leave the engine cycle (dwell advance > 360°)
                        *not feasible with 2 cylinders and 1 coil*
                        (that means that dwell for the next cylinder should already have begun)

            solution:   clip dwell advance to 360°
                        (cutting dwell)

*/
void update_ignition_timing(volatile process_data_t * pImage, volatile ignition_timing_t * pTarget)
{
    #warning TODO (oli#1#):test ignition calculation

    VU32 target_crank_angle_deg, advance_deg, dwell_deg, current_crank_angle_deg;

    if(pImage->engine_rpm > configPage13.dynamic_min_rpm)
    {
        /**
        calculate advance (off timing)
        */

        //use fixed ignition position in dynamic mode
        pTarget->coil_ignition_pos= configPage13.dynamic_ignition_position;

        //get the expected (ideal) crank angle at ignition trigger position
        current_crank_angle_deg= pImage->crank_position_table.a_deg[configPage13.dynamic_ignition_position];

        //get target ignition advance angle
        advance_deg= table3D_getValue(&ignitionTable_TPS, pImage->engine_rpm, pImage->TPS_deg);

        //store ignition advance to timing object
        pTarget->ignition_advance_deg= advance_deg;

        //calculate the corresponding crank ignition angle
        target_crank_angle_deg= 360 - advance_deg;

        //clip large advance angles
        if(target_crank_angle_deg < current_crank_angle_deg)
        {
            pTarget->coil_ignition_timing_us= 0;
        }
        else
        {
            pTarget->coil_ignition_timing_us= calc_rot_duration_us(target_crank_angle_deg-current_crank_angle_deg, pImage->crank_T_us);
        }

        //calculate dwell duration in crank deg (on timing)
        dwell_deg = calc_rot_angle_deg(configPage13.dynamic_dwell_us, pImage->crank_T_us);

        //export to timing
        pTarget->dwell_deg= dwell_deg;
        pTarget->dwell_ms= configPage13.dynamic_dwell_us / 1000;

        //clip dwell to crank cycle
        if(target_crank_angle_deg >= dwell_deg)
        {
            //normally, dwell + ignition advance should be max. around 100 deg
            target_crank_angle_deg -= dwell_deg;
        }
        else
        {
            #warning TODO (oli#6#):check if this case is relevant / should be tackled more accurate

            //clip dwell advance to cycle
            target_crank_angle_deg =0;
        }

        //calculate and store the optimal scheduler trigger position to timing object (dwell)
        //fit_position(pImage->crank_T_us, crank_angle_deg, &(pTarget->coil_dwell_pos), &(pTarget->coil_dwell_timing_us));
        pTarget->coil_dwell_pos= CRK_POSITION_D2;
       pTarget->coil_dwell_timing_us= 10;


        /**
        account for scheduler allocation

        we have only one scheduler per ignition channel -> use the scheduler for precise interval end (ignition),
        use decoder irq for interval begin (dwell)
        */
        if(pTarget->coil_ignition_pos == pTarget->coil_dwell_pos)
        {
            pTarget->coil_dwell_timing_us =0;
        }

    }
    else
    {
        /**
        use fixed ignition triggers while cranking
        */
        cranking_ignition_timing(pTarget);
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



void trigger_coil_by_timer(VU32 delay_us, VU32 level)
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

