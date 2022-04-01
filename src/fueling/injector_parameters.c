#include "Tuareg.h"
#include "base_calc.h"

#include "Tuareg_fueling.h"
#include "Tuareg_fueling_controls.h"
#include "Fueling_syslog_locations.h"
#include "fueling_hw.h"
#include "fueling_config.h"

#include "diagnostics.h"
#include "fueling_diag.h"




/*
built in defaults
*/
const U32 cDefault_injector_deadtime_us= 1000;
const U32 cMin_injector_deadtime_us= 100;

const U32 cMin_injection_end_target_advance_deg= 140;
const U32 cMax_injection_end_target_advance_deg= 400;



/****************************************************************************************************************************************
*   Fueling controls update - injector timing parameters
*
*   calculate the required fueling actor power on times to inject the given fuel mass for each cylinder
*   and checks the calculated intervals against the maximum injector duty cycle
****************************************************************************************************************************************/
void update_injector_deadtime(volatile fueling_control_t * pTarget)
{
    U32 injector_deadtime_us;

    //check preconditions
    if((Tuareg.flags.limited_op == true) || (Tuareg.errors.fueling_config_error == true))
    {
        pTarget->injector_deadtime_us= cDefault_injector_deadtime_us;
        return;
    }

    /*
    look up dead time
    if the battery voltage sensor fails its default value will be sufficient, too
    The values in the Injector timing table are in 24 us increments
    */
    injector_deadtime_us= getValue_InjectorTimingTable(Tuareg.process.VBAT_V);

    if(injector_deadtime_us < cMin_injector_deadtime_us)
    {
        Syslog_Error(TID_FUELING_CONTROLS, FUELING_LOC_INJ_DEADTIME_INVALID);
        injector_deadtime_us= cDefault_injector_deadtime_us;
    }

    pTarget->injector_deadtime_us= injector_deadtime_us;
}




/****************************************************************************************************************************************
*   Fueling controls update - injector intervals - sequential
*
*   calculate the required fueling actor power on times to inject the given fuel mass for each cylinder
*   and checks the calculated intervals against the maximum injector duty cycle
****************************************************************************************************************************************/
void update_injector_intervals_sequential(volatile fueling_control_t * pTarget)
{
    U32 inj1_target_interval_us, inj2_target_interval_us, max_powered_interval_us, target_duty_cycle;

    //start over with clean flags
    pTarget->flags.injector_dc_clip= false;


    /*
    injector 1
    */

    //injection interval [µs] := 1000 * fuel mass [µg] / injector rate [mg/s := µg/ms]
    inj1_target_interval_us= divide_U32(1000 * pTarget->cmd_fuel_mass_ug, Fueling_Setup.injector1_rate_mgps);

    //interval [µs] := injection interval [µs] + injector dead time [µs]
    inj1_target_interval_us +=  pTarget->injector_deadtime_us;

    /*
    injector 2
    */

    //injection interval [µs] := 1000 * fuel mass [µg] / injector rate [mg/s := µg/ms]
    inj2_target_interval_us= divide_U32(1000 * pTarget->cmd_fuel_mass_ug, Fueling_Setup.injector2_rate_mgps);

    //interval [µs] := injection interval [µs] + injector dead time [µs]
    inj2_target_interval_us +=  pTarget->injector_deadtime_us;


    /*
    dc threshold calculation relies on crank speed information
    */
    if(Tuareg.pDecoder->flags.period_valid == true)
    {
        //calculate the maximum powered interval based on 720° engine cycle
        max_powered_interval_us= (2 * Tuareg.pDecoder->crank_period_us * Fueling_Setup.max_injector_duty_cycle_pct) / 100;

        //calculate the injector duty cycle based on 720° engine cycle
        target_duty_cycle= divide_F32(100 * inj1_target_interval_us, 2 * Tuareg.pDecoder->crank_period_us);

        //check if the required fuel amount can be delivered in this mode
        if(inj1_target_interval_us > max_powered_interval_us)
        {
            pTarget->flags.injector_dc_clip= true;

            //clip to safe value
            inj1_target_interval_us= max_powered_interval_us;
        }

        //check if the required fuel amount can be delivered in this mode
        if(inj2_target_interval_us > max_powered_interval_us)
        {
            pTarget->flags.injector_dc_clip= true;

            //clip to safe value
            inj2_target_interval_us= max_powered_interval_us;
        }

    }

    //export
    pTarget->injector1_interval_us= inj1_target_interval_us;
    pTarget->injector2_interval_us= inj2_target_interval_us;
    pTarget->injector_target_dc= target_duty_cycle;

}


/****************************************************************************************************************************************
*   Fueling controls update - injector intervals - batch
*
*   calculate the required fueling actor power on times to inject the given fuel mass for each cylinder
*   and checks the calculated intervals against the maximum injector duty cycle
*   in batch mode the required fuel mass (cycle based) will be spread evenly accross 2 crank revolutions
****************************************************************************************************************************************/
void update_injector_intervals_batch(volatile fueling_control_t * pTarget)
{
    U32 inj1_target_interval_us, inj2_target_interval_us, max_powered_interval_us;

    U32 target_duty_cycle;

    //start over with clean flags
    pTarget->flags.injector_dc_clip= false;

    /*
    injector 1
    */

    //injection interval [µs] := (1/2) * 1000 * fuel mass [µg] / injector rate [mg/s := µg/ms]
    inj1_target_interval_us= divide_U32(500 * pTarget->cmd_fuel_mass_ug, Fueling_Setup.injector1_rate_mgps);

    //interval [µs] := injection interval [µs] + injector dead time [µs]
    inj1_target_interval_us +=  pTarget->injector_deadtime_us;

    /*
    injector 2
    */

    //injection interval [µs] := (1/2) * 1000 * fuel mass [µg] / injector rate [mg/s := µg/ms]
    inj2_target_interval_us= divide_U32(500 * pTarget->cmd_fuel_mass_ug, Fueling_Setup.injector2_rate_mgps);

    //interval [µs] := injection interval [µs] + injector dead time [µs]
    inj2_target_interval_us +=  pTarget->injector_deadtime_us;


    /*
    dc threshold calculation relies on crank speed information
    */
    if(Tuareg.pDecoder->flags.period_valid == true)
    {
        //calculate the maximum powered interval based on 360° crank cycle
        max_powered_interval_us= (Tuareg.pDecoder->crank_period_us * Fueling_Setup.max_injector_duty_cycle_pct) / 100;

        //calculate the injector duty cycle based on 360° engine cycle
        target_duty_cycle= divide_F32(100 * inj1_target_interval_us, Tuareg.pDecoder->crank_period_us);


        //check if the required fuel amount can be delivered in this mode
        if(inj1_target_interval_us > max_powered_interval_us)
        {
            pTarget->flags.injector_dc_clip= true;

            //clip to safe value
            inj1_target_interval_us= max_powered_interval_us;
        }

        //check if the required fuel amount can be delivered in this mode
        if(inj2_target_interval_us > max_powered_interval_us)
        {
            pTarget->flags.injector_dc_clip= true;

            //clip to safe value
            inj2_target_interval_us= max_powered_interval_us;
        }
    }

    //export
    pTarget->injector1_interval_us= inj1_target_interval_us;
    pTarget->injector2_interval_us= inj2_target_interval_us;
    pTarget->injector_target_dc= target_duty_cycle;

}



/****************************************************************************************************************************************
*   Fueling controls update - injection begin - batch
*
*   in batch mode injection (for injector 1 and 2) begins at the default injection begin position in any phase
****************************************************************************************************************************************/
void update_injection_begin_batch(volatile fueling_control_t * pTarget)
{
    //data update in progress
    pTarget->flags.injection_begin_valid= false;

    /*
    in batch mode injection (for injector 1 and 2) begins at the default injection begin position in any phase
    */
    pTarget->injection_begin_pos= Fueling_Setup.injection_reference_pos;
    pTarget->seq_injector1_begin_phase= PHASE_UNDEFINED;
    pTarget->seq_injector2_begin_phase= PHASE_UNDEFINED;

    pTarget->flags.injection_begin_valid= true;
}



/****************************************************************************************************************************************
*   Fueling controls update - injection begin - sequential
*
*   this calculation is valid for cylinder #1 and #2 as it does not affect the actual fuel amount to be injected
*   it is designed for a 180° parallel twin engine
****************************************************************************************************************************************/
void update_injection_begin_sequential(volatile fueling_control_t * pTarget)
{
    VU32 injection_end_target_advance_deg, avail_timing_us, avail_interval_deg;
    VU32 injector1_interval_deg, injector2_interval_deg;
    process_position_t injection_base_POS;
    exec_result_t result;

    //data update in progress
    pTarget->flags.injection_begin_valid= false;


    /******************************************
    precondition check
    ******************************************/

    //check preconditions - crank period known
    if((Tuareg.pDecoder->flags.period_valid == false) || (Tuareg.pDecoder->flags.rpm_valid == false))
    {
        return;
    }


    /******************************************
    injection end target angle
    ******************************************/

    //get injection end target advance angle
    injection_end_target_advance_deg= getValue_InjectorPhaseTable(Tuareg.pDecoder->crank_rpm);

    //check target advance
    if(injection_end_target_advance_deg < cMin_injection_end_target_advance_deg)
    {
        //log incident
        Syslog_Warning(TID_FUELING_CONTROLS, FUELING_LOC_UPDINJBEG_ENDTARGET_INVALID);

        //clip to min
        injection_end_target_advance_deg= cMin_injection_end_target_advance_deg;
    }

    if(injection_end_target_advance_deg > cMax_injection_end_target_advance_deg)
    {
        //log incident
        Syslog_Warning(TID_FUELING_CONTROLS, FUELING_LOC_UPDINJBEG_ENDTARGET_INVALID);

        //clip to max
        injection_end_target_advance_deg= cMax_injection_end_target_advance_deg;
    }


    /******************************************
    injection interval calculation
    ******************************************/

    //calculate the injector intervals in degree
    injector1_interval_deg= calc_rot_angle_deg(pTarget->injector1_interval_us, Tuareg.pDecoder->crank_period_us);
    injector2_interval_deg= calc_rot_angle_deg(pTarget->injector2_interval_us, Tuareg.pDecoder->crank_period_us);


    //check which injection interval is longer
    if(injector1_interval_deg > injector2_interval_deg)
    {
        result= find_process_position_before(injector1_interval_deg + injection_end_target_advance_deg, &injection_base_POS, 10);
    }
    else
    {
        result= find_process_position_before(injector2_interval_deg + injection_end_target_advance_deg, &injection_base_POS, 10);
    }

    ASSERT_EXEC_OK_VOID(result);


    /******************************************
    injection begin timing
    ******************************************/

    /**
    calculate the remaining interval between the injection base position and the injection end target position
    */
    avail_interval_deg= subtract_U32(injection_base_POS.base_PA, injection_end_target_advance_deg);

    //calculate the available timing for the injection reference position in far compression stroke
    avail_timing_us= calc_rot_duration_us(avail_interval_deg, Tuareg.pDecoder->crank_period_us);

    //injector #1 and #2 timings
    pTarget->injection_begin_pos= injection_base_POS.crank_pos;
    pTarget->injector1_timing_us= subtract_U32(avail_timing_us, pTarget->injector1_interval_us);
    pTarget->seq_injector1_begin_phase= injection_base_POS.phase;
    pTarget->injector2_timing_us= subtract_U32(avail_timing_us, pTarget->injector2_interval_us);
    pTarget->seq_injector2_begin_phase= opposite_phase(injection_base_POS.phase);


    //ready
    pTarget->flags.injection_begin_valid= true;

}
