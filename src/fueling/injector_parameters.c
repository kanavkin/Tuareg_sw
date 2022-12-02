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

    /**
    look up the injector dead time
    if the battery voltage sensor fails its default value will be used
    -> battery voltage sensor error shall lead to a limited engine speed
    -> battery voltage sensor default value shall lead to a rich mixture
    */
    injector_deadtime_us= getValue_InjectorTimingTable(Tuareg.process.VBAT_V);

    //validate dead time
    if(injector_deadtime_us < cMin_injector_deadtime_us)
    {
        Fatal(TID_FUELING_CONTROLS, FUELING_LOC_INJ_DEADTIME_INVALID);
        injector_deadtime_us= cMin_injector_deadtime_us;
    }

    //export
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
    U32 inj1_target_interval_us, inj2_target_interval_us, max_powered_interval_us, target_duty_cycle =0;

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

            //limit engine speed
            Limp(TID_FUELING_INJECTOR_PARAMS, FUELING_LOC_INJ1_DC_CLIP);
        }

        //check if the required fuel amount can be delivered in this mode
        if(inj2_target_interval_us > max_powered_interval_us)
        {
            pTarget->flags.injector_dc_clip= true;

            //clip to safe value
            inj2_target_interval_us= max_powered_interval_us;

            //limit engine speed
            Limp(TID_FUELING_INJECTOR_PARAMS, FUELING_LOC_INJ2_DC_CLIP);
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
    U32 inj1_target_interval_us, inj2_target_interval_us, max_powered_interval_us, target_duty_cycle =0;

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

            //enter Limp mode
            Limp(TID_FUELING_CONTROLS, FUELING_LOC_INJ1_DC_CLIP);
        }

        //check if the required fuel amount can be delivered in this mode
        if(inj2_target_interval_us > max_powered_interval_us)
        {
            pTarget->flags.injector_dc_clip= true;

            //clip to safe value
            inj2_target_interval_us= max_powered_interval_us;

            //enter Limp mode
            Limp(TID_FUELING_CONTROLS, FUELING_LOC_INJ2_DC_CLIP);
        }
    }

    //export
    pTarget->injector1_interval_us= inj1_target_interval_us;
    pTarget->injector2_interval_us= inj2_target_interval_us;
    pTarget->injector_target_dc= target_duty_cycle;

}



