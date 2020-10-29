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
#include "table.h"
#include "base_calc.h"
#include "config.h"


/**
calculates the corresponding delay from the dynamic ignition position to the commanded ignition advance angle
and
finds the best dwell position for the commanded dwell target duration
-> no scheduler allocation for dwell!

*** this function shall never be called in LIMP Mode ***

*/
void fit_position( VU32 Ign_advance_deg, VU32 Dwell_target_us, volatile process_data_t * pImage, volatile ignition_timing_t * pTarget)
{
    VU32 ignition_angle_deg, base_crank_angle_deg, position, dwell_angle, dwell_prolongation_us;
    volatile crank_position_table_t dwell_table;

    /**
    calculate ignition (off) timing
    */

    //use fixed ignition position in dynamic mode
    pTarget->coil_ignition_pos= configPage13.dynamic_ignition_base_position;

    //get the expected (ideal) crank angle at ignition trigger position
    base_crank_angle_deg= pImage->crank_position_table.a_deg[configPage13.dynamic_ignition_base_position];

    //store ignition advance to timing object
    pTarget->ignition_advance_deg= Ign_advance_deg;

    //calculate the corresponding crank ignition angle
    ignition_angle_deg= 360 - Ign_advance_deg;

    //the next ignition event will take place after ca. 360 deg
    pTarget->coil_ignition_timing_us= calc_rot_duration_us( subtract_VU32(ignition_angle_deg, base_crank_angle_deg), pImage->crank_T_us);


    /**
    dwell calculation
    */

    //because of the crank_position_t order its more convenient to calculate all segments
    for(position= 0; position < CRK_POSITION_COUNT; position++)
    {
        //calculate angle from position to ignition

        //check if position is bTDC or aTDC, CRK_POSITION_B2 can be affected
        if(pImage->crank_position_table.a_deg[position] > 300)
        {
            dwell_angle= ignition_angle_deg + (360 - pImage->crank_position_table.a_deg[position]);
        }
        else
        {
            dwell_angle= ignition_angle_deg - pImage->crank_position_table.a_deg[position];
        }

        //calculate rotational duration from position to ignition
        dwell_table.a_deg[position]= calc_rot_duration_us(dwell_angle, pImage->crank_T_us);
    }

    /**
    loop through the positions and find the first position, that will extent dwell
    searching from the shortest possible dwell position, the first match is important, all others will be longer
    */
    for(position= DYNAMIC_DWELL_LATEST_POSITION; position > DYNAMIC_DWELL_EARLIEST_POSITION; position--)
    {
        //calculate, how much the resulting dwell duration is longer then the target dwell duration
        dwell_prolongation_us= subtract_VU32(dwell_table.a_deg[position], Dwell_target_us);

        if(dwell_prolongation_us > 0)
        {
            //bingo!
            break;
        }

        /**
        WARNING hard to understand code
        the for loop will anyways decrement the position variable if it is not left by 'break'
        */
    }

    pTarget->coil_dwell_timing_us =0;
    pTarget->coil_dwell_pos= position;
    pTarget->dwell_ms= dwell_table.a_deg[position] / 1000;
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
    pTarget->dwell_ms= DEFAULT_REPORTED_DWELL_MS;
    pTarget->ignition_advance_deg= DEFAULT_IGNITION_ADVANCE_DEG;

    pTarget->state.advance_map= FALSE;
    pTarget->state.advance_tps= FALSE;
    pTarget->state.dynamic= FALSE;
    pTarget->state.cold_idle= FALSE;
    pTarget->state.rev_limiter= FALSE;
    pTarget->state.cranking_timing= FALSE;
    pTarget->state.default_timing= TRUE;

}

/**
provides a late ignition timing for cranking
*/
inline void cranking_ignition_timing(volatile ignition_timing_t * pTarget)
{
    pTarget->coil_ignition_pos= configPage13.cranking_ignition_position;
    pTarget->coil_dwell_pos= configPage13.cranking_dwell_position;
    pTarget->ignition_advance_deg= CRANKING_REPORTED_IGNITION_ADVANCE_DEG;
    pTarget->dwell_ms= CRANKING_REPORTED_DWELL_MS;
    pTarget->coil_ignition_timing_us =0;
    pTarget->coil_dwell_timing_us =0;

    pTarget->state.advance_map= FALSE;
    pTarget->state.advance_tps= FALSE;
    pTarget->state.dynamic= FALSE;
    pTarget->state.cold_idle= FALSE;
    pTarget->state.rev_limiter= FALSE;
    pTarget->state.default_timing= FALSE;
    pTarget->state.cranking_timing= TRUE;
}

/**
calculates the ignition timing for the next engine cycle
at a given rpm (from ignition_timing_t) and writes it there

*** this function shall never be called in LIMP Mode ***


The fact that the crank decoder will not provide any crank velocity information for the first 2..3 crank revolutions after getting sync will not upset
the ignition logic. Without a suitable engine_rpm figure, cranking_ignition_timing will be used

*/
void update_ignition_timing(volatile process_data_t * pImage, volatile ignition_timing_t * pTarget)
{
    /// TODO (oli#1#):test ignition calculation

    VU32 Ign_advance_deg, Dwell_target_us;

    if(pImage->crank_rpm > configPage13.max_rpm)
    {
        /**
        rev limiter function activated
        we have to take care that the update process data, update ignition timing function will be executed anyways!
        */

        //suspend ignition
        pTarget->coil_ignition_pos= DEFAULT_IGNITION_POSITION;
        pTarget->coil_ignition_timing_us= 0;
        pTarget->ignition_advance_deg= 0;
        pTarget->coil_dwell_pos= CRK_POSITION_UNDEFINED;
        pTarget->coil_dwell_timing_us= 0;
        pTarget->dwell_ms= 0;

        //set status bit
        pTarget->state.advance_map= FALSE;
        pTarget->state.advance_tps= FALSE;
        pTarget->state.dynamic= FALSE;
        pTarget->state.cold_idle= FALSE;
        pTarget->state.default_timing= FALSE;
        pTarget->state.cranking_timing= FALSE;
        pTarget->state.rev_limiter= TRUE;

    }
    else if(pImage->crank_rpm > configPage13.dynamic_min_rpm)
    {
        /**
        dynamic ignition function activated
        */

        //set status bit
        pTarget->state.dynamic= TRUE;
        pTarget->state.default_timing= FALSE;
        pTarget->state.cranking_timing= FALSE;
        pTarget->state.rev_limiter= FALSE;

        if( (pImage->crank_rpm < configPage13.cold_idle_cutoff_rpm) && (pImage->CLT_K < configPage13.cold_idle_cutoff_CLT_K) )
        {
            /**
            cold idle function activated
            */
            pTarget->state.cold_idle= TRUE;

            Ign_advance_deg= configPage13.cold_idle_ignition_advance_deg;
            Dwell_target_us= configPage13.cold_idle_dwell_target_us;

        }
        else
        {
            ///get ignition advance from table

            //set status bit
            pTarget->state.advance_map= FALSE;
            pTarget->state.advance_tps= TRUE;
            pTarget->state.cold_idle= FALSE;


            //get target ignition advance angle
            //Ign_advance_deg= table3D_getValue(&ignitionTable_TPS, pImage->crank_rpm, pImage->TPS_deg);

            /// TODO (oli#3#): tps readout not stable yet
            Ign_advance_deg= table3D_getValue(&ignitionTable_TPS, pImage->crank_rpm, 30);


            ///get dwell from table

            /// TODO (oli#1#): dwell logic hacked! shall be replaced by a proper target dwell calculation/table soon!

            //get target dwell duration
            if(pImage->crank_rpm < 2000)
            {
                Dwell_target_us = 10000;
            }
            else
            {
                Dwell_target_us = configPage13.dynamic_dwell_target_us;
            }

        }

        /**
        prepare the ignition timing object
        */
        fit_position(Ign_advance_deg, Dwell_target_us, pImage, pTarget);

    }
    else
    {
        /**
        cranking function activated
        */
        cranking_ignition_timing(pTarget);
    }
}


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

