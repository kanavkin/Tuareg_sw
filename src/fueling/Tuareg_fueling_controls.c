#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_adc.h"
#include "stm32_libs/boctok_types.h"

#include "base_calc.h"

#include "Tuareg_fueling.h"
#include "Tuareg_fueling_controls.h"
#include "Fueling_syslog_locations.h"
#include "fueling_hw.h"
#include "fueling_config.h"

#include "diagnostics.h"
#include "Tuareg.h"




/*
built in defaults
*/
const U32 cDefault_cranking_fuel_mass_ug= 10000;
const U32 cDefault_injector_deadtime_us= 1000;
const U32 cDefault_VE_pct= 40;
const F32 cDefault_AFR_target= 14.7;




/****************************************************************************************************************************************
*   Fueling controls update
****************************************************************************************************************************************/

/**
calculates the fueling parameters according to the current system state and run mode

fueling_config_error indicates, that fueling tables are not available

*/
void Tuareg_update_fueling_controls()
{
    volatile fueling_control_t * pTarget= &(Tuareg.fueling_controls);


    //check operational preconditions
    if((Tuareg.flags.run_inhibit == true) || (Tuareg.flags.standby == true) || (Tuareg.flags.rev_limiter == true) || ((Tuareg.engine_runtime == 0) && (Tuareg.flags.cranking == false)))
    {
        //clean controls
        invalid_fueling_controls(pTarget);
        return;
    }


    //check if the engine has finished cranking
    if(Tuareg.flags.cranking == false)
    {
        //not cranking
        pTarget->flags.dry_cranking= false;

        //check for sequential / batch mode capabilities
        pTarget->flags.sequential_mode= (Tuareg.errors.fueling_config_error == false) && (Tuareg.flags.limited_op == false) && (Fueling_Setup.features.sequential_mode_enabled == true) && (Tuareg.pDecoder->outputs.phase_valid);


        /**
        VE
        calculation will fail in limp mode
        */
        update_volumetric_efficiency(pTarget);

        //check if update has been successful
        if(pTarget->flags.VE_valid == false)
        {
    /// TODO (oli#3#): implement a ve estimation calculation for LIMP mode based on crank rpm

            //use default value
            pTarget->VE_pct= cDefault_VE_pct;
        }


        /**
        air density
        calculation based on MAP and IAT values from process data
        */
        update_air_density(pTarget);


        /**
        AFR
        calculation will fail in limp mode
        */
        update_AFR_target(pTarget);

        //check if update has been successful
        if(pTarget->flags.AFR_target_valid == false)
        {
            //use default value
            pTarget->AFR_target= cDefault_AFR_target;
        }

        /**
        base fuel mass
        calculation based on the calculated VE and AFR values
        in case of fueling config error the default value for cylinder volume applies
        */
        update_base_fuel_mass(pTarget);

        /**
        after start compensation
        feature will not be activated in limp mode
        */
        update_fuel_mass_afterstart_correction(pTarget);

        /**
        warm up compensation
        feature will not be activated in limp mode
        */
        update_fuel_mass_warmup_correction(pTarget);

        /**
        load transient compensation
        feature will not be activated in limp mode
        */
        update_fuel_mass_accel_correction(pTarget);

    }
    else
    {
        //start over with clean controls
        invalid_fueling_controls(pTarget);

        //use base fuel mass from cranking table
        update_base_fuel_mass_cranking(pTarget);
    }


    /**
    apply all activated corrections
    in limp mode the target fuel mass will be simply the base fuel mass
    */
    update_target_fuel_mass(pTarget);


    /**
    injector dead time
    */
    update_injector_deadtime(pTarget);


    //check if sequential mode has been requested
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


        //check if the calculation has succeeded
        if(pTarget->flags.injection_begin_valid == false)
        {
            //downgrade to batch mode
            pTarget->flags.sequential_mode= false;

            Syslog_Error(TID_TUAREG_FUELING, FUELING_LOC_SEQUENTIAL_CALC_FAIL);
        }
    }


    if(pTarget->flags.sequential_mode == false)
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
    pTarget->AFR_target= cDefault_AFR_target;
    pTarget->VE_pct= 0;
    pTarget->air_density= 0;
    pTarget->target_fuel_mass_ug= 0;

    pTarget->injector1_interval_us= 0;
    pTarget->injector2_interval_us= 0;
    pTarget->injector_target_dc= 0;

    pTarget->injection_begin_pos= CRK_POSITION_UNDEFINED;
    pTarget->seq_injector1_begin_phase= PHASE_UNDEFINED;
    pTarget->seq_injector2_begin_phase= PHASE_UNDEFINED;

    pTarget->flags.all_flags= 0;
}



/****************************************************************************************************************************************
*   fueling controls update helper functions - VE
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

    //data update in progress
    pTarget->flags.VE_valid= false;

    //check preconditions - engine speed is known and VE tables are available
    if((Tuareg.pDecoder->outputs.rpm_valid == false) || (Tuareg.errors.fueling_config_error == true) || (Tuareg.flags.limited_op == true))
    {
        return;
    }

    rpm= Tuareg.pDecoder->crank_rpm;


    //check from which table to look up
    if((Tuareg.errors.sensor_TPS_error == true) || ((Tuareg.errors.sensor_MAP_error == false) && (rpm > Fueling_Setup.ve_from_map_min_rpm) && (rpm < Fueling_Setup.ve_from_map_max_rpm)))
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


/****************************************************************************************************************************************
*   fueling controls update helper functions - air density
****************************************************************************************************************************************/


/**
air density is expected in micro grams per cm³

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
/// TODO (oli#3#): altitude shall affect the effective charge temperature calculation
    charge_temp_K= Tuareg.process.IAT_K;


    pTarget->air_density= divide_VF32(density, charge_temp_K);
}



/****************************************************************************************************************************************
*   fueling controls update helper functions - AFR
****************************************************************************************************************************************/


/**
decides from which table to look up the AFR target

AFR is expected in 7 .. 30 range
*/
void update_AFR_target(volatile fueling_control_t * pTarget)
{
    VF32 AFR_value;

    //data update in progress
    pTarget->flags.AFR_target_valid= false;

    //check preconditions - engine speed is known and AFR tables are available and TPS figure is valid
    if((Tuareg.pDecoder->outputs.rpm_valid == false) || (Tuareg.errors.fueling_config_error == true) || (Tuareg.errors.sensor_TPS_error == true) || (Tuareg.flags.limited_op == true))
    {
        return;
    }

    //look up
    AFR_value= getValue_AfrTable_TPS(Tuareg.pDecoder->crank_rpm, Tuareg.process.TPS_deg);

    //range check
    if((AFR_value > 5.0) && (AFR_value < 30.0))
    {
        pTarget->AFR_target= AFR_value;
        pTarget->flags.AFR_target_valid= true;
    }
}




/****************************************************************************************************************************************
*   Fueling controls update - base fuel mass
****************************************************************************************************************************************/


/**
calculate the required fuel mass to be injected into each cylinder

this calculation relies on the configured cylinder volume or its built in default!
*/
void update_base_fuel_mass(volatile fueling_control_t * pTarget)
{
    F32 base_fuel_mass_ug, air_mass_ug;

    //air mass [µg] := air_density [µg/cm³] * cylinder volume [cm³] * VE [%] / 100
    air_mass_ug= (Tuareg.fueling_controls.air_density * Fueling_Setup.cylinder_volume_ccm * Tuareg.fueling_controls.VE_pct) / 100.0;

    //base fuel mass [µg] := air mass [ug] / AFR [1]
    base_fuel_mass_ug= divide_float(air_mass_ug, Tuareg.fueling_controls.AFR_target);

    pTarget->base_fuel_mass_ug= base_fuel_mass_ug;
}





/**
calculate the required fuel mass to be injected into each cylinder
*/
void update_base_fuel_mass_cranking(volatile fueling_control_t * pTarget)
{
    pTarget->flags.dry_cranking= false;

    //check preconditions - table available
    if((Tuareg.errors.fueling_config_error == true) || (Tuareg.flags.limited_op == true))
    {
        pTarget->base_fuel_mass_ug= cDefault_cranking_fuel_mass_ug;
        return;
    }

    //check if dry cranking conditions are present
    if((Tuareg.process.TPS_deg > Fueling_Setup.dry_cranking_TPS_thres) && (Fueling_Setup.features.dry_cranking_enabled == true))
    {
        pTarget->base_fuel_mass_ug= 0;
        pTarget->flags.dry_cranking= true;
        return;
    }

    /**
    look up the cranking mode base fuel mass from the table
    if the CLT sensor has failed its default value would be sufficient, too
    */
    pTarget->base_fuel_mass_ug= getValue_CrankingFuelTable(Tuareg.process.CLT_K);
}



/****************************************************************************************************************************************
*   fueling controls update helper functions - fuel mass correction
****************************************************************************************************************************************/



/**
calculate the required fuel mass compensation for the present throttle change rate

any new trigger condition will enlarge the fuel mass compensation interval
*/
void update_fuel_mass_accel_correction(volatile fueling_control_t * pTarget)
{
    //check preconditions
    if((Tuareg.flags.limited_op == true) || (Tuareg.errors.sensor_TPS_error == true) || (Tuareg.errors.fueling_config_error == true) ||
        (Fueling_Setup.features.load_transient_comp_enabled == false))
    {
        pTarget->flags.accel_comp_active= false;
        pTarget->fuel_mass_accel_corr_cycles_left= 0;
        pTarget->fuel_mass_accel_corr_pct= 0.0;

        return;
    }

    /**
    check if acceleration compensation has been (re)triggered
    */
    if(Tuareg.process.ddt_TPS >= Fueling_Setup.accel_comp_thres)
    {
        pTarget->flags.accel_comp_active= true;
        pTarget->fuel_mass_accel_corr_cycles_left= Fueling_Setup.accel_comp_cycles;

        //look up the correction factor and export
        pTarget->fuel_mass_accel_corr_pct= getValue_AccelCompTable(Tuareg.process.ddt_TPS);

    }
    else if(Tuareg.process.ddt_TPS <= Fueling_Setup.decel_comp_thres)
    {
        pTarget->flags.accel_comp_active= true;
        pTarget->fuel_mass_accel_corr_cycles_left= Fueling_Setup.accel_comp_cycles;

        //export the correction factor
        pTarget->fuel_mass_accel_corr_pct= -Fueling_Setup.decel_comp_pct;

    }

    /**
    check if acceleration compensation is (now/still) active
    */
    if(pTarget->flags.accel_comp_active == false)
    {
        pTarget->fuel_mass_accel_corr_cycles_left= 0;
        pTarget->fuel_mass_accel_corr_pct= 0.0;
        return;
    }

    /**
    check if the compensation interval has expired
    */
    if(pTarget->fuel_mass_accel_corr_cycles_left > 0)
    {
        //one compensation cycle has been consumed
        pTarget->fuel_mass_accel_corr_cycles_left -= 1;
    }
    else
    {
        pTarget->flags.accel_comp_active= false;
        pTarget->fuel_mass_accel_corr_cycles_left= 0;
        pTarget->fuel_mass_accel_corr_pct= 0.0;
    }

}




/**
calculate the fuel mass compensation factor for engine warmup
*/

const U32 cAfterstart_begin_thres= 3;
const U32 cAfterstart_thres= 50;

void update_fuel_mass_afterstart_correction(volatile fueling_control_t * pTarget)
{
    //check preconditions
    if((Tuareg.flags.limited_op == true) || (Tuareg.errors.fueling_config_error == true) || (Fueling_Setup.features.afterstart_corr_enabled == false))
    {
        pTarget->flags.afterstart_comp_active= false;
        pTarget->fuel_mass_afterstart_corr_pct= 0.0;
        pTarget->fuel_mass_afterstart_corr_cycles_left= 0;
        return;
    }

    //check if the after start compensation is already enabled
    if(pTarget->flags.afterstart_comp_active == true)
    {
        //check if the after start cycle counter has expired
        if(pTarget->fuel_mass_afterstart_corr_cycles_left == 0)
        {
            //after start enrichment counter expired - deactivate
            pTarget->flags.afterstart_comp_active= false;
            pTarget->fuel_mass_afterstart_corr_pct= 0.0;
        }
        else
        {
            //after start compensation is still active
            pTarget->flags.afterstart_comp_active= true;
            pTarget->fuel_mass_afterstart_corr_pct= Fueling_Setup.afterstart_comp_pct;
            pTarget->fuel_mass_afterstart_corr_cycles_left -= 1;
        }

    }
    else
    {
        //check if the engine has just started
        if((Tuareg.engine_runtime > cAfterstart_begin_thres) && (Tuareg.engine_runtime < cAfterstart_thres))
        {
            //ase activated
            pTarget->flags.afterstart_comp_active= true;
            pTarget->fuel_mass_afterstart_corr_pct= Fueling_Setup.afterstart_comp_pct;
            pTarget->fuel_mass_afterstart_corr_cycles_left= Fueling_Setup.afterstart_comp_cycles;
        }
        else
        {
            //ase disabled
            pTarget->flags.afterstart_comp_active= false;
            pTarget->fuel_mass_afterstart_corr_pct= 0.0;
            pTarget->fuel_mass_afterstart_corr_cycles_left= 0;
        }
    }

}



/**
calculate the fuel mass compensation factor for engine warmup
*/
void update_fuel_mass_warmup_correction(volatile fueling_control_t * pTarget)
{
    VF32 warmup_comp;

    //check preconditions
    if((Tuareg.flags.limited_op == true) || (Tuareg.errors.fueling_config_error == true) || (Tuareg.errors.sensor_CLT_error == true) ||
        (Fueling_Setup.features.warmup_comp_enabled == false))
    {
        pTarget->flags.warmup_comp_active= false;
        pTarget->fuel_mass_warmup_corr_pct= 0.0;
        return;
    }

    //look up the correction factor and export
    warmup_comp= getValue_WarmUpCompTable(Tuareg.process.CLT_K);
    pTarget->fuel_mass_warmup_corr_pct= getValue_WarmUpCompTable(Tuareg.process.CLT_K);

    pTarget->flags.warmup_comp_active= (warmup_comp > 3.0)? true : false;
}


/****************************************************************************************************************************************
*   fueling controls update helper functions - target fuel mass
****************************************************************************************************************************************/

const F32 cMax_fuel_mass_comp_pct= 800.0;

/**
calculate the required fuel mass to be injected into each cylinder
*/
void update_target_fuel_mass(volatile fueling_control_t * pTarget)
{
    VF32 compensation_pct= 0.0;
    VF32 base_fuel_mass_ug;

    //get current base fuel mass in float format
    base_fuel_mass_ug= pTarget->base_fuel_mass_ug;


    //check preconditions
    if((Tuareg.flags.limited_op == true) || (Tuareg.errors.fueling_config_error == true) || (Tuareg.flags.cranking == true))
    {
        //copy base fuel mass only
        pTarget->target_fuel_mass_ug= (VU32) base_fuel_mass_ug;
        return;
    }


    //check if after start compensation is active
    if(pTarget->flags.afterstart_comp_active == true)
    {
        compensation_pct += pTarget->fuel_mass_afterstart_corr_pct;
    }

    //check if warm up enrichment is active
    if(pTarget->flags.warmup_comp_active == true)
    {
        compensation_pct += pTarget->fuel_mass_warmup_corr_pct;
    }

    //check if load transient compensation is active
    if(pTarget->flags.accel_comp_active == true)
    {
        compensation_pct += pTarget->fuel_mass_accel_corr_pct;
    }

    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wfloat-equal"

    //check if compensation is needed
    if(compensation_pct == 0.0)
    {
        //copy base fuel mass only
        pTarget->target_fuel_mass_ug= (VU32) base_fuel_mass_ug;
        return;
    }

    #pragma GCC diagnostic pop

    //check if compensation is within interval
    if(compensation_pct < -99.9)
    {
        //reduce 100% fuel -> no fuel
        pTarget->target_fuel_mass_ug= 0;
        return;
    }
    else if(compensation_pct > cMax_fuel_mass_comp_pct)
    {
        compensation_pct= Fueling_Setup.max_fuel_mass_comp_pct;

        Syslog_Warning(TID_FUELING_CONTROLS, FUELING_LOC_COMPENSATION_CLIP);
    }

    //enrichment or lean out -99.9 % ... max_fuel_mass_comp %
    pTarget->target_fuel_mass_ug= ((100.0 + compensation_pct) * base_fuel_mass_ug) / 100.0;

}



/****************************************************************************************************************************************
*   Fueling controls update - injector timing parameters
****************************************************************************************************************************************/

const U32 cMin_injector_deadtime_us= 100;

/**
calculate the required fueling actor power on times to inject the given fuel mass for each cylinder
and checks the calculated intervals against the maximum injector duty cycle
*/
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


/**
calculate the required fueling actor power on times to inject the given fuel mass for each cylinder
and checks the calculated intervals against the maximum injector duty cycle
*/
void update_injector_intervals_sequential(volatile fueling_control_t * pTarget)
{
    U32 inj1_target_interval_us, inj2_target_interval_us, max_powered_interval_us;

    U32 target_duty_cycle;

    //start over with clean flags
    pTarget->flags.injector_dc_clip= false;


    /// injector 1

    //injection interval [µs] := 1000 * fuel mass [µg] / injector rate [mg/s := µg/ms]
    inj1_target_interval_us= divide_VU32(1000 * pTarget->target_fuel_mass_ug, Fueling_Setup.injector1_rate_mgps);

    //interval [µs] := injection interval [µs] + injector dead time [µs]
    inj1_target_interval_us +=  pTarget->injector_deadtime_us;

    /// injector 2

    //injection interval [µs] := 1000 * fuel mass [µg] / injector rate [mg/s := µg/ms]
    inj2_target_interval_us= divide_VU32(1000 * pTarget->target_fuel_mass_ug, Fueling_Setup.injector2_rate_mgps);

    //interval [µs] := injection interval [µs] + injector dead time [µs]
    inj2_target_interval_us +=  pTarget->injector_deadtime_us;


    /// dc threshold calculation relies on crank speed information
    if(Tuareg.pDecoder->outputs.period_valid == true)
    {
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

    //start over with clean flags
    pTarget->flags.injector_dc_clip= false;

    /// injector 1

    //injection interval [µs] := (1/2) * 1000 * fuel mass [µg] / injector rate [mg/s := µg/ms]
    inj1_target_interval_us= divide_VU32(500 * pTarget->target_fuel_mass_ug, Fueling_Setup.injector1_rate_mgps);

    //interval [µs] := injection interval [µs] + injector dead time [µs]
    inj1_target_interval_us +=  pTarget->injector_deadtime_us;

    /// injector 2

    //injection interval [µs] := (1/2) * 1000 * fuel mass [µg] / injector rate [mg/s := µg/ms]
    inj2_target_interval_us= divide_VU32(500 * pTarget->target_fuel_mass_ug, Fueling_Setup.injector2_rate_mgps);

    //interval [µs] := injection interval [µs] + injector dead time [µs]
    inj2_target_interval_us +=  pTarget->injector_deadtime_us;


    /// dc threshold calculation relies on crank speed information
    if(Tuareg.pDecoder->outputs.period_valid == true)
    {
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
    }

    //export
    pTarget->injector1_interval_us= inj1_target_interval_us;
    pTarget->injector2_interval_us= inj2_target_interval_us;
    pTarget->injector_target_dc= target_duty_cycle;

}



/****************************************************************************************************************************************
*   Fueling controls update - injection begin
****************************************************************************************************************************************/


/**
default_injection_begin_pos
seq_cyl1_default_injection_begin_phase

this position is as close as possible to the intake valve opening

in batch mode injection begins at this position

in sequential mode injection for cylinder #1 begins with the indicated engine phase
the default injection begin phase is valid for cylinder #1
*/

/**
seq_earliest_injection_begin_pos
seq_cyl1_earliest_injection_begin_phase

the earliest injection begin phase is valid for cylinder #1
this is the earliest position at which injection in sequential mode may begin
*/


/**
exports the default injection begin positions for batch mode

sets the injection_begin_valid flag
*/
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


/**
calculates the injection begin positions for sequential mode

sets the injection_begin_valid flag on success

this calculation is valid for cylinder #1 and #2 as it does not affect the actual fuel amount to be injected
    it is designed for 180° parallel twin engine
*/
void update_injection_begin_sequential(volatile fueling_control_t * pTarget)
{
    VU32 injection_end_target_advance_deg, avail_timing_us, avail_interval_deg;
    VU32 injector1_interval_deg, injector2_interval_deg;
    process_position_t injection_base_POS;
    exec_result_t result;

    //data update in progress
    pTarget->flags.injection_begin_valid= false;


    //check preconditions - crank period known
    if((Tuareg.pDecoder->outputs.period_valid == false) || (Tuareg.pDecoder->outputs.rpm_valid == false))
    {
        return;
    }

    //get injection end target advance angle
    injection_end_target_advance_deg= getValue_InjectorPhaseTable(Tuareg.pDecoder->crank_rpm);

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


    /**
    calculate the remaining interval between the injection base position and the injection end target position
    */
    avail_interval_deg= subtract_VU32(injection_base_POS.base_PA, injection_end_target_advance_deg);

    //calculate the available timing for the injection reference position in far compression stroke
    avail_timing_us= calc_rot_duration_us(avail_interval_deg, Tuareg.pDecoder->crank_period_us);

    //injector #1 and #2 timings
    pTarget->injection_begin_pos= injection_base_POS.crank_pos;
    pTarget->injector1_timing_us= subtract_VU32(avail_timing_us, pTarget->injector1_interval_us);
    pTarget->seq_injector1_begin_phase= injection_base_POS.phase;
    pTarget->injector2_timing_us= subtract_VU32(avail_timing_us, pTarget->injector2_interval_us);
    pTarget->seq_injector2_begin_phase= opposite_phase(injection_base_POS.phase);


    //ready
    pTarget->flags.injection_begin_valid= true;

}
