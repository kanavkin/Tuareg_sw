#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_adc.h"
#include "stm32_libs/boctok_types.h"

#include "base_calc.h"

#include "Tuareg_fueling.h"
#include "Tuareg_fueling_controls.h"
#include "fueling_hw.h"
#include "fueling_config.h"

#include "diagnostics.h"
#include "Tuareg.h"





/**
this position is as close as possible to the intake valve opening

in batch mode injection begins at this position

in sequential mode injection for cylinder #1 begins with the indicated engine phase
the default injection begin phase is valid for cylinder #1
*/
crank_position_t default_injection_begin_pos= CRK_POSITION_B1;
engine_phase_t seq_cyl1_default_injection_begin_phase= PHASE_CYL1_EX;


/**
this is the earliest position at which injection in sequential mode may begin
the earliest injection begin phase is valid for cylinder #1
*/
crank_position_t seq_earliest_injection_begin_pos= CRK_POSITION_B1;
engine_phase_t seq_cyl1_earliest_injection_begin_phase= PHASE_CYL1_COMP;




/**
the angle between intake valve closing and compression TDC
*/
U16 intake_close_advance_deg= 130;

U8 default_VE_pct= 40;
F32 default_AFR_target= 14.7;


/****************************************************************************************************************************************
*   Fueling controls update
****************************************************************************************************************************************/


/// TODO (oli#4#): implement batch -> sequential / sequential -> batch mode transition (via init?)



/**
calculates the fueling parameters according to the current system state and run mode

fueling_config_error indicates, that fueling tables are not available

*/
void Tuareg_update_fueling_controls()
{
    volatile fueling_control_t * pTarget= &(Tuareg.fueling_controls);

    //delete old controls
    invalid_fueling_controls(pTarget);


    //check if all preconditions for calculating fueling controls are met
    if((Tuareg.pDecoder->outputs.rpm_valid == false) || (Tuareg.actors.rev_limiter == true))
    {
        return;
    }


    //check for sequential / batch mode capabilities
    pTarget->flags.sequential_mode= Tuareg.pDecoder->outputs.phase_valid;


    /**
    VE
    */

    //check if VE tables are available
    if(Tuareg.Errors.fueling_config_error == false)
    {
        update_volumetric_efficiency(pTarget);
    }

    if(pTarget->flags.VE_valid == false)
    {
        //use default value
        pTarget->VE_pct= default_VE_pct;
    }


    /**
    air density
    calculation based on MAP and IAT values from process data
    */
    update_air_density(pTarget);


    /**
    AFR
    */

    //check if AFR table is available
    if(Tuareg.Errors.fueling_config_error == false)
    {
        update_AFR_target(pTarget);
    }

    if(pTarget->flags.AFR_target_valid == false)
    {
        //use default value
        pTarget->AFR_target= default_AFR_target;
    }


/// TODO (oli#2#): implement warm up enrichment (via AFR target)

    /**
    fuel mass
    calculation based on the calculated VE and AFR values
    */
    update_fuel_mass_target(pTarget);


/// TODO (oli#2#): implement acceleration pump


/// TODO (oli#2#): implement injector dead time table


    if(pTarget->flags.sequential_mode == true)
    {
        /**
        injector intervals calculation based on the calculated target fuel mass
        */
        update_injector_intervals_sequential(pTarget);

        /**
        injection begin positions
        */
        update_injection_begin_sequential(pTarget);

        //check if the calculation succeeded
        if(pTarget->flags.injection_begin_valid == false)
        {
            //use a safe default
            earliest_sequential_injection_begin(pTarget);
        }
    }
    else
    {
        /**
        injector intervals calculation based on the calculated target fuel mass
        */
        update_injector_intervals_batch(pTarget);

        /**
        injection begin positions
        */
        update_injection_begin_batch(pTarget);
    }

    //now the fueling controls are ready
    pTarget->flags.valid= true;

}


/****************************************************************************************************************************************
*   predefined controls
****************************************************************************************************************************************/

/**
clears the given fueling controls
*/
void invalid_fueling_controls(volatile fueling_control_t * pTarget)
{
    pTarget->AFR_target= 0;
    pTarget->VE_pct= 0;
    pTarget->air_density= 0;
    pTarget->fuel_mass_target_ug= 0;

    pTarget->injector1_interval_us= 0;
    pTarget->injector2_interval_us= 0;
    pTarget->injector_target_dc= 0;

    pTarget->injection_begin_pos= CRK_POSITION_UNDEFINED;
    pTarget->seq_injector1_begin_phase= PHASE_UNDEFINED;
    pTarget->seq_injector2_begin_phase= PHASE_UNDEFINED;

    pTarget->flags.all_flags= 0;
}






/****************************************************************************************************************************************
*   fueling controls update helper functions
****************************************************************************************************************************************/



/**
decides from which table to look up the current volumetric efficiency

MAP based VE table shall be referenced only if the MAP sensor readout is valid.
If the engine rpm is within the GET_VE_FROM_MAP window, MAP VE table shall be referenced.
If the TPS sensor readout is invalid, MAP VE table shall be referenced.

If both MAP and TPS sensor fail, the TPS based VE table shall be referenced.
(In this case lookup will use the default sensor value).

VE is defined in 0 .. 150 % range
*/
void update_volumetric_efficiency(volatile fueling_control_t * pTarget)
{
    U32 rpm;
    VF32 VE_value;

    rpm= Tuareg.pDecoder->crank_rpm;

    //check from which table to look up
    if((Tuareg.Errors.sensor_MAP_error == false) && ((rpm > Fueling_Setup.ve_from_map_min_rpm) && (rpm < Fueling_Setup.ve_from_map_max_rpm) && (Tuareg.Errors.sensor_TPS_error == true)))
    {
        //use VE table MAP lookup
        pTarget->flags.VE_from_MAP= true;

        VE_value= getValue_VeTable_MAP(rpm, Tuareg.process.MAP_kPa);
    }
    else
    {
        //use VE table TPS lookup
        pTarget->flags.VE_from_MAP= false;

        VE_value= getValue_VeTable_TPS(rpm, Tuareg.process.TPS_deg);
    }

    //range check
    if((VE_value > 0.0) && (VE_value <= 150.0))
    {
        pTarget->VE_pct= VE_value;
        pTarget->flags.VE_valid= true;
    }


}



/**
air density is expected in micro gram per cm³

dens :=  (cM_air / cR_gas) * (MAP/"IAT")

coolant temperature and throttle shall affect the effective charge temperature calculation

calculation has to rely on the MAP and IAT values reported in process data
*/
void update_air_density(volatile fueling_control_t * pTarget)
{
    F32 charge_temp_K, density;


    density= (Tuareg.process.MAP_kPa * cM_air) / cR_gas;

    //by now the charge temperature is assumed not to depend on engine state
/// TODO (oli#3#): coolant temperature and throttle shall affect the effective charge temperature calculation
    charge_temp_K= Tuareg.process.IAT_K;


    pTarget->air_density= divide_VF32(density, charge_temp_K);
}



/**
decides from which table to look up the AFR target

AFR is expected in 7 .. 30 range
*/
void update_AFR_target(volatile fueling_control_t * pTarget)
{
    VF32 AFR_value;

    //look up
    AFR_value= getValue_AfrTable_TPS(Tuareg.pDecoder->crank_rpm, Tuareg.process.TPS_deg);

    //range check
    if((AFR_value > 5.0) && (AFR_value < 30.0))
    {
        pTarget->AFR_target= AFR_value;
        pTarget->flags.AFR_target_valid= true;
    }
}



/**
calculate the required fuel mass to be injected into each cylinder
*/
void update_fuel_mass_target(volatile fueling_control_t * pTarget)
{
    U32 fuel_mass_target_ug;
    F32 air_mass_ug;

    //air mass [µg] := air_density [µg/cm³] * cylinder volume [cm³] * VE [%] / 100
    air_mass_ug= (Tuareg.fueling_controls.air_density * Fueling_Setup.cylinder_volume_ccm * Tuareg.fueling_controls.VE_pct) / 100;

    //target fuel mass [µg] := air mass [ug] / AFR [1]
    fuel_mass_target_ug= (U32) divide_VF32(air_mass_ug, Tuareg.fueling_controls.AFR_target);

    pTarget->fuel_mass_target_ug= fuel_mass_target_ug;
}



/**
calculate the required fueling actor power on times to inject the given fuel mass for each cylinder
and checks the calculated intervals against the maximum injector duty cycle
*/
void update_injector_intervals_sequential(volatile fueling_control_t * pTarget)
{
    U32 inj1_target_interval_us, inj2_target_interval_us, max_powered_interval_us;

    U32 target_duty_cycle;

    /// injector 1

    //injection interval [µs] := 1000 * fuel mass [µg] / injector rate [mg/s := µg/ms]
    inj1_target_interval_us= divide_VU32(1000 * pTarget->fuel_mass_target_ug, Fueling_Setup.injector1_rate_mgps);

    //interval [µs] := injection interval [µs] + injector dead time [µs]
    inj1_target_interval_us +=  Fueling_Setup.injector_deadtime_us;

    /// injector 2

    //injection interval [µs] := 1000 * fuel mass [µg] / injector rate [mg/s := µg/ms]
    inj2_target_interval_us= divide_VU32(1000 * pTarget->fuel_mass_target_ug, Fueling_Setup.injector2_rate_mgps);

    //interval [µs] := injection interval [µs] + injector dead time [µs]
    inj2_target_interval_us +=  Fueling_Setup.injector_deadtime_us;

    /// dc threshold

    //calculate the maximum powered interval based on 720° engine cycle
    max_powered_interval_us= (2 * Tuareg.pDecoder->crank_period_us * Fueling_Setup.max_injector_duty_cycle_pct) / 100;

    //calculate the injector duty cycle based on 720° engine cycle
    target_duty_cycle= divide_VU32(100 * inj1_target_interval_us, 2 * Tuareg.pDecoder->crank_period_us);


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

    //export
    pTarget->injector1_interval_us= inj1_target_interval_us;
    pTarget->injector2_interval_us= inj2_target_interval_us;
    pTarget->injector_target_dc= target_duty_cycle;

}


/**
calculate the required fueling actor power on times to inject the given fuel mass for each cylinder
and checks the calculated intervals against the maximum injector duty cycle

in batch mode the required fuel mass (cycle based) will be injected in 2 crank revolutions
*/
void update_injector_intervals_batch(volatile fueling_control_t * pTarget)
{
    U32 inj1_target_interval_us, inj2_target_interval_us, max_powered_interval_us;

    U32 target_duty_cycle;

    /// injector 1

    //injection interval [µs] := (1/2) * 1000 * fuel mass [µg] / injector rate [mg/s := µg/ms]
    inj1_target_interval_us= divide_VU32(500 * pTarget->fuel_mass_target_ug, Fueling_Setup.injector1_rate_mgps);

    //interval [µs] := injection interval [µs] + injector dead time [µs]
    inj1_target_interval_us +=  Fueling_Setup.injector_deadtime_us;

    /// injector 2

    //injection interval [µs] := (1/2) * 1000 * fuel mass [µg] / injector rate [mg/s := µg/ms]
    inj2_target_interval_us= divide_VU32(500 * pTarget->fuel_mass_target_ug, Fueling_Setup.injector2_rate_mgps);

    //interval [µs] := injection interval [µs] + injector dead time [µs]
    inj2_target_interval_us +=  Fueling_Setup.injector_deadtime_us;

    /// dc threshold

    //calculate the maximum powered interval based on 360° crank cycle
    max_powered_interval_us= (Tuareg.pDecoder->crank_period_us * Fueling_Setup.max_injector_duty_cycle_pct) / 100;

    //calculate the injector duty cycle based on 360° engine cycle
    target_duty_cycle= divide_VU32(100 * inj1_target_interval_us, Tuareg.pDecoder->crank_period_us);


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

    //export
    pTarget->injector1_interval_us= inj1_target_interval_us;
    pTarget->injector2_interval_us= inj2_target_interval_us;
    pTarget->injector_target_dc= target_duty_cycle;

}





/**
exports the default injection begin positions for batch mode

sets the injection_begin_valid flag
*/
void update_injection_begin_batch(volatile fueling_control_t * pTarget)
{

    //check injector state - no injectors shall be powered at this point
    if((Tuareg.actors.fuel_injector_1 == true) || (Tuareg.actors.fuel_injector_1 == true))
    {
        ///ERROR!!!
        set_injector1_unpowered();
        set_injector2_unpowered();

        //syslog
    }


    /*
    in batch mode injection (for injector 1 and 2) begins at the default injection begin position in any phase
    */
    pTarget->injection_begin_pos= default_injection_begin_pos;
    pTarget->seq_injector1_begin_phase= PHASE_UNDEFINED;
    pTarget->seq_injector2_begin_phase= PHASE_UNDEFINED;

    pTarget->flags.injection_begin_valid= true;

}


/**
calculates the injection begin positions for sequential mode

sets the injection_begin_valid flag on success
*/
void update_injection_begin_sequential(volatile fueling_control_t * pTarget)
{
    U32 injection_interval_deg, injection_target_advance_deg;
    process_position_t injection_begin_POS, injection_default_begin_POS, injection_earliest_begin_POS;
    exec_result_t result;


    /**
    check if the fueling system is about to be overloaded
    maximum fuel amount requires the earliest possible injection begin
    */
    if(pTarget->flags.injector_dc_clip == true)
    {
        earliest_sequential_injection_begin(pTarget);
        pTarget->flags.injection_begin_valid= true;
        return;
    }


    /**
    this base position calculation is valid for cylinder #1 and #2 as it does not affect the actual fuel amount to be injected
    it is designed for 180° parallel twin engine
    */

    //calculate the injector interval for cylinder #1 as reference
    injection_interval_deg= calc_rot_angle_deg(pTarget->injector1_interval_us, Tuareg.pDecoder->crank_period_us);

    //calculate the target injection begin advance angle for cylinder #1 as reference
    injection_target_advance_deg= injection_interval_deg + intake_close_advance_deg;


    /**
    check if the injector interval is so short that it should be moved closer to the intake valve opening
    (at default injection begin position)
    */

    injection_default_begin_POS.crank_pos= default_injection_begin_pos;
    injection_default_begin_POS.phase= seq_cyl1_default_injection_begin_phase;

    //get the basic advance angle the ignition base position provides
    result= get_process_advance(&injection_default_begin_POS);

    ASSERT_EXEC_OK_VOID(result);

    //check if the default injection begin position provides enough time to finish injection until intake valve closes
    if(injection_default_begin_POS.base_PA >= injection_target_advance_deg)
    {
        pTarget->injection_begin_pos= default_injection_begin_pos;
        pTarget->seq_injector1_begin_phase= seq_cyl1_default_injection_begin_phase;
        pTarget->seq_injector2_begin_phase= opposite_phase(seq_cyl1_default_injection_begin_phase);

        pTarget->flags.injection_begin_valid= true;
        return;
    }


    /**
    check if the injector interval is so long that it would not fit the earliest allowed injection begin position
    */

    injection_earliest_begin_POS.crank_pos= seq_earliest_injection_begin_pos;
    injection_earliest_begin_POS.phase= seq_cyl1_earliest_injection_begin_phase;

    //get the basic advance angle the ignition base position provides
    result= get_process_advance(&injection_earliest_begin_POS);

    ASSERT_EXEC_OK_VOID(result);

    //check if the earliest injection begin position is required
    if(injection_target_advance_deg >= injection_earliest_begin_POS.base_PA)
    {
        //begin injection at the earliest possible injection begin position
        earliest_sequential_injection_begin(pTarget);

        pTarget->flags.injection_begin_valid= true;
        return;
    }

    /**
    then try to find a position in between
    */
    result= find_process_position_before(injection_target_advance_deg, &injection_begin_POS);

    ASSERT_EXEC_OK_VOID(result);

    //check if the result from process table is valid
    if(injection_begin_POS.base_PA <= injection_earliest_begin_POS.base_PA)
    {
        //use the calculated position
        pTarget->injection_begin_pos= injection_begin_POS.crank_pos;
        pTarget->seq_injector1_begin_phase= injection_begin_POS.phase;
        pTarget->seq_injector2_begin_phase= opposite_phase(injection_begin_POS.phase);

        pTarget->flags.injection_begin_valid= true;
        return;
    }


    /**
    why did this lookup fail?
    */
}


/**
this is also a safe configuration, allowing the injection to finish within fueling cycle
(provided that the commanded interval is valid)

*/
void earliest_sequential_injection_begin(volatile fueling_control_t * pTarget)
{
    //begin injection as early as possible
    pTarget->injection_begin_pos= seq_earliest_injection_begin_pos;
    pTarget->seq_injector1_begin_phase= seq_cyl1_earliest_injection_begin_phase;
    pTarget->seq_injector2_begin_phase= opposite_phase(seq_cyl1_earliest_injection_begin_phase);
}
