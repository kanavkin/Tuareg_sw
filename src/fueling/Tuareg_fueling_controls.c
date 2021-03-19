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


U32 cylinder_volume_ccm= 425;


// gas constant 8.31446261815324 (kg * m²) / (s² * K * mol)
F32 cR_gas= 8.31446;

//molar mass of air in mg per mol (default: 28970)
F32 cM_air= 28970.0;

U16 injector_deadtime_us= 500;

U16 injector1_rate_mgps= 4160;
U16 injector2_rate_mgps= 4160;

U16 min_interval_us= 1000;
U8 max_powered_interval_pct= 85;

#define GET_VE_FROM_MAP_MIN_RPM 2000
#define GET_VE_FROM_MAP_MAX_RPM 7000


U32 intake_open_PA= 12;
U32 intake_open_interval_deg= 242;



/****************************************************************************************************************************************
*   Fueling controls update
****************************************************************************************************************************************/


/**
calculates the fueling parameters according to the current system state and run mode

*/
void Tuareg_update_fueling_controls()
{
    volatile fueling_control_t * pTarget= &(Tuareg.fueling_controls);

    //check for sequential / batch mode capabilities
    Tuareg.fueling_controls.flags.sequential_mode= Tuareg.pDecoder->outputs.phase_valid;

    update_volumetric_efficiency(pTarget);
    update_air_density(pTarget);
    update_AFR_target(pTarget);
    update_fuel_mass(pTarget);

    update_injector_intervals(pTarget);


    update_injector_timings(pTarget);





}


/****************************************************************************************************************************************
*   fueling controls update helper functions
****************************************************************************************************************************************/


/**
decides from which table to look up the current volumetric efficiency

VE is expected in 0 .. 1 range
*/
void update_volumetric_efficiency(volatile fueling_control_t * pTarget)
{
    U32 rpm;
    VF32 VE_value;

    //preconditions check
    if((Tuareg.pDecoder->outputs.rpm_valid == false) || (Tuareg.Errors.fueling_config_error == true))
    {
        pTarget->flags.valid= false;
        pTarget->volumetric_efficency_pct= 0.0;
        return;
    }

    rpm= Tuareg.pDecoder->crank_rpm;

    //check which table to look up
    if((rpm > GET_VE_FROM_MAP_MIN_RPM) && (rpm < GET_VE_FROM_MAP_MAX_RPM))
    {
        //use VE table MAP lookup
        pTarget->flags.VEfromMAP= true;

        VE_value= getValue_VeTable_MAP(Tuareg.pDecoder->crank_rpm, Tuareg.process.MAP_kPa);
    }
    else
    {
        //use VE table TPS lookup
        pTarget->flags.VEfromMAP= false;

        VE_value= getValue_VeTable_TPS(Tuareg.pDecoder->crank_rpm, Tuareg.process.TPS_deg);
    }

    //range check
    if((VE_value > 0.0) && (VE_value <= 150.0))
    {
        pTarget->volumetric_efficency_pct= VE_value;
    }
    else
    {
        pTarget->flags.valid= false;
        pTarget->volumetric_efficency_pct= 0.0;
    }

}



/**
air density is expected in micro gram per cm³

dens :=  (cM_air / cR_gas) * (MAP/"IAT")

coolant temperature and throttle shall affect the effective charge temperature calculation
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

    //preconditions check
    if((Tuareg.pDecoder->outputs.rpm_valid == false) || (Tuareg.Errors.fueling_config_error == true))
    {
        pTarget->flags.valid= false;
        pTarget->AFR_target= 14.5;
        return;
    }

    //look up
    AFR_value= getValue_AfrTable_TPS(Tuareg.pDecoder->crank_rpm, Tuareg.process.TPS_deg);

    //range check
    if((AFR_value > 5.0) && (AFR_value < 30.0))
    {
        pTarget->AFR_target= AFR_value;
    }
    else
    {
        pTarget->flags.valid= false;
        pTarget->AFR_target= 14.5;
    }

}



/**
calculate the required fuel mass to be injected into each cylinder
*/
void update_fuel_mass(volatile fueling_control_t * pTarget)
{
    U32 fuel_mass_ug;
    F32 air_mass_ug;

    //air mass [µg] := air_density [µg/cm³] * cylinder volume [cm³] * VE [%] / 100
    air_mass_ug= (Tuareg.fueling_controls.air_density * cylinder_volume_ccm * Tuareg.fueling_controls.volumetric_efficency_pct) / 100;

    //fuel mass [µg] := air mass [ug] / AFR [1]
    fuel_mass_ug= (U32) divide_VF32(air_mass_ug, Tuareg.fueling_controls.AFR_target);

    pTarget->fuel_mass_ug= fuel_mass_ug;
}



/**
calculate the required fueling actor power on times to inject the given fuel mass for each cylinder
and checks the calculated intervals against the maximum injector duty cycle
*/
void update_injector_intervals(volatile fueling_control_t * pTarget)
{
    U32 target_interval_us, max_powered_interval_us;

    //preconditions check
    if(Tuareg.pDecoder->outputs.rpm_valid == false)
    {
        pTarget->flags.valid= false;
        pTarget->injector1_interval_us= 0;
        pTarget->injector2_interval_us= 0;

        return;
    }

    pTarget->flags.injector_overload= false;

    /*
    injector 1
    */

    //injection interval [µs] := 1000 * fuel mass [µg] / injector rate [mg/s := µg/ms]
    target_interval_us= divide_VU32(1000 * pTarget->fuel_mass_ug, injector1_rate_mgps);

    //interval [µs] := injection interval [µs] + injector dead time [µs]
    target_interval_us +=  injector_deadtime_us;

    //check if sequential mode is active
    if(pTarget->flags.sequential_mode == true)
    {
        //calculate the maximum powered interval
        max_powered_interval_us= (2 * Tuareg.pDecoder->crank_period_us * max_powered_interval_pct) / 100;
    }
    else
    {
        //calculate the maximum powered interval
        max_powered_interval_us= (Tuareg.pDecoder->crank_period_us * max_powered_interval_pct) / 100;
    }

    //check if the required fuel amount can be delivered in this mode
    if(target_interval_us > max_powered_interval_us)
    {
        pTarget->flags.injector_overload= true;

        //clip to safe value
        target_interval_us= max_powered_interval_us;
    }

    //export
    pTarget->injector1_interval_us= target_interval_us;


    /*
    injector 2
    */

    //injection interval [µs] := 1000 * fuel mass [µg] / injector rate [mg/s := µg/ms]
    target_interval_us= divide_VU32(1000 * pTarget->fuel_mass_ug, injector2_rate_mgps);

    //interval [µs] := injection interval [µs] + injector dead time [µs]
    target_interval_us +=  injector_deadtime_us;

    //check if sequential mode is active
    if(pTarget->flags.sequential_mode == true)
    {
        //calculate the maximum powered interval
        max_powered_interval_us= (2 * Tuareg.pDecoder->crank_period_us * max_powered_interval_pct) / 100;
    }
    else
    {
        //calculate the maximum powered interval
        max_powered_interval_us= (Tuareg.pDecoder->crank_period_us * max_powered_interval_pct) / 100;
    }

    //check if the required fuel amount can be delivered in this mode
    if(target_interval_us > max_powered_interval_us)
    {
        pTarget->flags.injector_overload= true;

        //clip to safe value
        target_interval_us= max_powered_interval_us;
    }

    //export
    pTarget->injector2_interval_us= target_interval_us;

}


/**

*/
void update_injector_timings(volatile fueling_control_t * pTarget)
{

    if(pTarget->flags.sequential_mode)
    {
        update_injector_timings_sequential(pTarget);
    }
    else
    {
        update_injector_timings_batch(pTarget);
    }
}


/**

*/
void update_injector_timings_batch(volatile fueling_control_t * pTarget)
{
    U32 max_powered_interval_deg, target_powered_interval_deg;


    /*
    injector 1
    */




    //injection begins 1/3 of the total injection time before Intake Opens





}


void update_injector_timings_sequential(volatile fueling_control_t * pTarget)
{


}
