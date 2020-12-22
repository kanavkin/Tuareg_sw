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
#include "ignition_config.h"
#include "config_pages.h"
#include "config_tables.h"


/**
calculates the corresponding delay from the dynamic ignition position to the commanded ignition advance angle
and
finds the best dwell position for the commanded dwell target duration
-> no scheduler allocation for dwell!

*** this function shall never be called in LIMP Mode ***

THIS CALCULATION HAS BEEN TAILORED FOR 180° PARALLEL TWIN ENGINE!

*/
exec_result_t calculate_ignition_alignment( VU32 Ignition_AD, VU32 Dwell_target_us, VU32 Crank_T_us, volatile ignition_control_t * pTarget)
{
    process_position_t ignition_POS, dwell_target_POS, spark_end_POS;
    process_advance_t spark_end_PA, dwell_target_PA;
    angle_deg_t spark_burn_PD;
    U32 result;


    /**
    calculate ignition trigger timing

    Ignition base position and timing is common for cylinder #1 and #2.
    In phased mode an ignition event for cylinder #1 shall occur at ignition_pos:PHASE_CYL1_COMP
    In phased mode an ignition event for cylinder #2 shall occur at ignition_pos:PHASE_CYL1_EX
    In unphased mode an ignition event for cylinder #1 and #2 shall occur at every ignition_pos regardless of engine phase
    */

    //prepare transfer object for process table lookup (model: fixed ignition position in dynamic mode)
    ignition_POS.crank_pos= Ignition_Setup.dynamic_ignition_base_position;
    ignition_POS.phase= PHASE_CYL1_COMP;

    //get the basic advance angle the ignition base position provides
    result= get_process_advance(&ignition_POS);

    ASSERT_EXEC_OK(result);

    //ignition_timing_us reflects the scheduler delay to set up
    pTarget->ignition_timing_us= calc_rot_duration_us( subtract_VU32(ignition_POS.base_PA, Ignition_AD), Crank_T_us);

    //prepare further export data
    pTarget->ignition_pos= ignition_POS.crank_pos;
    pTarget->ignition_advance_deg= Ignition_AD;


    /**
    calculate the position corresponding to spark burn end across the whole engine cycle
    */
    spark_burn_PD= calc_rot_angle_deg(Ignition_Setup.spark_duration_us, Crank_T_us);

    //spark end process advance := 720 + ignition advance - spark burn angle
    spark_end_PA= subtract_VU32(720 + Ignition_AD, spark_burn_PD);

    //calculate spark_end_POS
    result= find_process_position_after(spark_end_PA, &spark_end_POS);

    ASSERT_EXEC_OK(result);

    /**
    calculate dwell target begin position
    */
    dwell_target_PA= Ignition_AD + calc_rot_angle_deg(Dwell_target_us, Crank_T_us);

    //find the position that will provide minimum the target dwell duration
    result= find_process_position_before(dwell_target_PA, &dwell_target_POS);

    ASSERT_EXEC_OK(result);


    /**
    spark burn duration has priority over dwell

    In unphased mode:
    -> dwell begins right after spark end target duration dwell_pos_unphased
    -> this can extend dwell beyond target dwell duration

    In phased mode:
    Dwell begin for cylinder #1 shall occur at dwell_pos_phased:dwell_phase_cyl1.
    Dwell begin for cylinder #2 shall occur at dwell_pos_phased:dwell_phase_cyl2.
    (spark burn end can be in EX or in the far COMP phase)

    Extended dwell means, that the dwell begin is before TDC of exhaust stroke.
    */

    /**
    dwell setup in phased mode
    */
    if(dwell_target_POS.base_PA > spark_end_POS.base_PA)
    {
        /*
        it has been requested more dwell than available in one engine cycle
        -> extended dwell condition is present
        -> clip dwell to end of spark burn
        -> dwell setup for phased mode
        */
        pTarget->state.extended_dwell= true;

        pTarget->dwell_pos_phased= spark_end_POS.crank_pos;
        pTarget->dwell_phase_cyl1= spark_end_POS.phase;
        pTarget->dwell_phase_cyl2= opposite_phase(spark_end_POS.phase);

        // dwell lies between spark end and ignition
        pTarget->dwell_ms_phased= calc_rot_duration_us( subtract_VU32(spark_end_POS.base_PA, Ignition_AD), Crank_T_us) / 1000;

    }
    else
    {
        /*
        standard scenario: target dwell position fits to engine cycle
        */
        pTarget->dwell_pos_phased= dwell_target_POS.crank_pos;
        pTarget->dwell_phase_cyl1= dwell_target_POS.phase;
        pTarget->dwell_phase_cyl2= opposite_phase(dwell_target_POS.phase);

        //check for extended dwell condition
        pTarget->state.extended_dwell= (dwell_target_POS.base_PA > 360)? true : false;
    }

    /**
    dwell setup in unphased mode
    */
    pTarget->dwell_pos_unphased= spark_end_POS.crank_pos;

    //spark burn end comes 360° earlier as in phased mode
    sub_VU32(&spark_burn_PD, 360);

    // dwell lies between spark end and ignition
    pTarget->dwell_ms_unphased= calc_rot_duration_us( subtract_VU32(spark_burn_PD, Ignition_AD), Crank_T_us) / 1000;

    return EXEC_OK;
}

/**
provides a ignition timing which will allow engine operation
e.g. while LIMP HOME
*/
void default_ignition_controls(volatile ignition_control_t * pTarget)
{
    //ignition setup
    pTarget->ignition_advance_deg= DEFAULT_IGNITION_ADVANCE_DEG;
    pTarget->ignition_timing_us= 0;
    pTarget->ignition_pos= DEFAULT_IGNITION_POSITION;

    //dwell setup in phased mode
    pTarget->dwell_phase_cyl1= PHASE_CYL1_COMP;
    pTarget->dwell_phase_cyl2= PHASE_CYL1_EX;
    pTarget->dwell_pos_phased= DEFAULT_DWELL_POSITION;
    pTarget->dwell_ms_phased= DEFAULT_REPORTED_DWELL_MS;

    //dwell setup in unphased mode
    pTarget->dwell_pos_unphased= DEFAULT_DWELL_POSITION;
    pTarget->dwell_ms_unphased= DEFAULT_REPORTED_DWELL_MS;

    //status data
    pTarget->state.all_flags= 0;
    pTarget->state.default_timing= true;
}

/**
provides a late ignition timing for cranking
*/
void cranking_ignition_controls(volatile ignition_control_t * pTarget)
{
    //ignition setup
    pTarget->ignition_advance_deg= CRANKING_REPORTED_IGNITION_ADVANCE_DEG;
    pTarget->ignition_timing_us= 0;
    pTarget->ignition_pos= Ignition_Setup.cranking_ignition_position;

    //dwell setup in phased mode
    pTarget->dwell_phase_cyl1= PHASE_CYL1_COMP;
    pTarget->dwell_phase_cyl2= PHASE_CYL1_EX;
    pTarget->dwell_pos_phased= DEFAULT_DWELL_POSITION;
    pTarget->dwell_ms_phased= CRANKING_REPORTED_DWELL_MS;

    //dwell setup in unphased mode
    pTarget->dwell_pos_unphased= DEFAULT_DWELL_POSITION;
    pTarget->dwell_ms_unphased= CRANKING_REPORTED_DWELL_MS;

    //status data
    pTarget->state.all_flags= 0;
    pTarget->state.cranking_timing= true;
}


/**
rev limiter function activated
we have to take care that the update process data, update ignition timing function will be executed anyways!
*/
void revlimiter_ignition_controls(volatile ignition_control_t * pTarget)
{
    //suspend ignition
    pTarget->ignition_advance_deg= REVLIMITER_REPORTED_IGNITION_ADVANCE_DEG;
    pTarget->ignition_timing_us= 0;
    pTarget->ignition_pos= CRK_POSITION_UNDEFINED;

    //dwell setup in phased mode
    pTarget->dwell_phase_cyl1= PHASE_UNDEFINED;
    pTarget->dwell_phase_cyl2= PHASE_UNDEFINED;
    pTarget->dwell_pos_phased= CRK_POSITION_UNDEFINED;
    pTarget->dwell_ms_phased= REVLIMITER_REPORTED_DWELL_MS;

    //dwell setup in unphased mode
    pTarget->dwell_pos_unphased= CRK_POSITION_UNDEFINED;
    pTarget->dwell_ms_unphased= CRANKING_REPORTED_DWELL_MS;

    //status data
    pTarget->state.all_flags= 0;
    pTarget->state.rev_limiter= true;

}


/**
calculates the ignition timing for the next engine cycle at a given rpm (from pImage) and writes it to the ignition_control_t provided

dynamic ignition function activated

*** this function shall never be called in LIMP Mode ***
*/
exec_result_t calculate_dynamic_ignition_controls(volatile process_data_t * pImage, volatile ignition_control_t * pTarget)
{
    /// TODO (oli#1#):test ignition calculation

    VU32 Ign_advance_deg, Dwell_target_us;
    exec_result_t result;

    //status data
    pTarget->state.all_flags= 0;
    pTarget->state.dynamic= true;

    if( (pImage->crank_rpm < Ignition_Setup.cold_idle_cutoff_rpm) && (pImage->CLT_K < Ignition_Setup.cold_idle_cutoff_CLT_K) )
    {
        /**
        cold idle function activated
        */
        pTarget->state.cold_idle= true;

        Ign_advance_deg= Ignition_Setup.cold_idle_ignition_advance_deg;
        Dwell_target_us= Ignition_Setup.cold_idle_dwell_target_us;

    }
    else
    {
        ///get ignition advance from table

        //set status bit
        pTarget->state.advance_tps= true;

        //get target ignition advance angle
        //Ign_advance_deg= table3D_getValue(&ignitionTable_TPS, pImage->crank_rpm, pImage->TPS_deg);

        /// TODO (oli#3#): tps readout not stable yet
        Ign_advance_deg= getValue_ignAdvTable_TPS(pImage->crank_rpm, 30);


        ///get dwell from table

        /// TODO (oli#1#): dwell logic hacked! shall be replaced by a proper target dwell calculation/table soon!

        //get target dwell duration
        if(pImage->crank_rpm < 2000)
        {
            Dwell_target_us = 10000;
        }
        else
        {
            Dwell_target_us = Ignition_Setup.dynamic_dwell_target_us;
        }

    }

    /**
    prepare the ignition control object
    */
    result= calculate_ignition_alignment(Ign_advance_deg, Dwell_target_us, pImage->crank_T_us, pTarget);

    return result;

}




