#include "Tuareg.h"
#include "base_calc.h"

#include "fueling_hw.h"
#include "fueling_config.h"

#include "Tuareg_fueling_controls.h"
#include "accel_comp.h"
#include "fueling_corrections.h"
#include "injector_parameters.h"

#include "Fueling_syslog_locations.h"
#include "diagnostics.h"
#include "fueling_diag.h"


//built in defaults
const F32 cDefault_AFR_target= 14.7;
const F32 cMin_AFR_target= 3.0;
const F32 cMax_AFR_target= 30.0;
const F32 cMin_VE_val= 0.0;
const F32 cMax_VE_val= 125.0;
const F32 cMax_fuel_rel_comp_pct= 100.0;
const F32 cMin_fuel_rel_comp_pct= -50.0;
const F32 cMax_fuel_abs_comp_ug= 10000.0;



/****************************************************************************************************************************************
*   Fueling controls update
*
*   calculates the fueling parameters according to the current system state and run mode
*
*   fueling_config_error indicates, that fueling tables are not available
*
*   the data from decoder interface can be considered stable for the entire function call
*   the rpm and period from decoder interface can be considered valid for the entire function call, if not cranking
*
*
*
*
****************************************************************************************************************************************/
/****************************************************************************************************************************************
*
*   decides which control strategy is currently available
*
*   MAP based VE table shall be referenced only if the MAP sensor readout is valid.
*   If the engine rpm is within the GET_VE_FROM_MAP window, MAP VE table shall be referenced.
*   If the TPS sensor readout is invalid, MAP VE table shall be referenced.
*
*   If both MAP and TPS sensor fail, the TPS based VE table shall be referenced.
*   (In this case lookup will use the default sensor value).
*
*   /// TODO (oli#1#03/31/22): harden fallback options
*   TODO fix design flaw:
*   If the MAP sensor is failing, fueling has to rely solely on TPS, calculated air density will be wrong! VE TPS table may not have
*   information for the rpm range
*
*   operating strategy:
*
*   full featured operation:
*   Cranking, CLT ok                -> cranking base fuel table lookup (CLT based)
*   MAP ok, rpm in SPD range        -> MAP based Speed-Density Algorithm (Air density, AFR by MAP)
*   MAP  ok, high rpm               -> TPS based Speed-Density Algorithm (Air density by MAP, AFR by TPS)
*   CIS ok, not cranking            -> sequential mode
*   -
*   error scenarios:
*
*   Cranking, CLT error             -> cranking base fuel table lookup (based on built-in CLT default)
*                                   -> CLT built-in default will implicate a fully warmed up engine
*                                   -> cranking performance degraded
*
*   Cranking, TPS error             -> dry cranking not available
*
*   Cranking, MAP/IAT/BARO error    -> no reaction
*
*
*   TPS error, MAP ok               -> MAP based Speed-Density Algorithm (Air density, AFR by MAP)
*                                   -> rev limiter adjusted to safe speed < SPD max rpm (VE, AFR tables shall cover this rpm range)
*                                   -> limited operation!
*
*   MAP error, TPS ok               -> TPS based Speed-Density Algorithm (Air density by default MAP!, default AFR!)
*                                   -> rev limiter adjusted to safe speed (VE table shall cover this rpm range)
*                                   -> limited operation!
*
*   MAP error, TPS error            -> no engine operation possible
*                                   -> fatal mode
*
*   IAT error                       -> AFR target set to default AFR
*                                   -> rev limiter adjusted to safe speed (VE table shall cover this rpm range)
*                                   -> limited operation!
*
*   CLT error                       -> warm up correction not available
*
*   BARO error                      -> barometric pressure based correction not available
*
*
*
****************************************************************************************************************************************/
void Tuareg_update_fueling_controls()
{
    volatile fueling_control_t * pTarget= &(Tuareg.fueling_controls);

    //log diag data
    fueling_diag_log_event(FDIAG_UPD_CTRLS_CALLS);


/// TODO (oli#1#02/28/23): move config check to end of config load
    //vital precondition: fueling config data available
    VitalAssert(Tuareg.errors.fueling_config_error == false, TID_FUELING_CONTROLS, FUELING_LOC_UPDCTRL_VITAL_PRECOND);

    /**
    functional preconditions:
        - at least one sensor to determine engine load is available
        without a load estimation, fueling the engine shall not be possible
        -> no functional impact on ignition!
    */
    if( (Tuareg.errors.fatal_error == true) ||
        (Tuareg.flags.run_inhibit == true) || (Tuareg.flags.standby == true) ||
        ((Tuareg.flags.cranking == false) && ((Tuareg.pDecoder->flags.rpm_valid == false) || (Tuareg.pDecoder->flags.period_valid == false) || (Tuareg.process.engine_runtime == 0))) ||
        ((Tuareg.errors.sensor_MAP_error == true) && (Tuareg.errors.sensor_TPS_error == true)) ||
        (Tuareg.Tuareg_controls.Flags.valid == false)
       )
    {
        //reset controls
        invalid_fueling_controls(pTarget);

        //log diag data
        fueling_diag_log_event(FDIAG_UPD_CTRLS_OP_PRECOND_FAIL);

        //early exit
        return;
    }

    /**
    calculate the fuel mass to be commanded
    */

    //check if the engine is cranking
    if(Tuareg.flags.cranking == true)
    {
        //use fuel mass from cranking table for base, target, cmd fuel mass
        update_fuel_mass_cranking(pTarget);

        //log diag data
        fueling_diag_log_event(FDIAG_UPD_CTRLS_CRANKING);
    }
    else
    {
        //dry cranking disabled
        pTarget->flags.dry_cranking= false;

        //log diag data
        fueling_diag_log_event(FDIAG_UPD_CTRLS_RUNNING);

/// TODO (oli#1#09/01/23): adapt to control strategy
        /**

        determine on which sensor input VE and AFR calculation shall be based
        already checked

        if((Tuareg.errors.sensor_TPS_error == true) && (Tuareg.errors.sensor_MAP_error == true))
        {
            //no engine operation without a load figure
            Fatal(TID_FUELING_CONTROLS, FUELING_LOC_UPDCTRL_SENSORS);
            return;
        }

        //select MAP sensor for lower engine speeds if possible and for higher speeds if TPS has failed (but higher engine speed will be rejected by rev limiter); TPS for higher
        pTarget->flags.SPD= ((Tuareg.errors.sensor_MAP_error == false) && (Tuareg.pDecoder->crank_rpm < Fueling_Setup.spd_max_rpm)) || (Tuareg.errors.sensor_TPS_error == true);

        //air density calculation relies on IAT sensor
        pTarget->flags.AFR_fallback= (Tuareg.errors.sensor_MAP_error == true) || (Tuareg.errors.sensor_IAT_error == true);

        //collect diagnostic data
        fueling_diag_log_event((pTarget->flags.MAP_nTPS == true)? FDIAG_UPD_CTRLS_MAP : FDIAG_UPD_CTRLS_TPS);
        */

        /**
        select fueling mode
        */

        //sequential mode requires phase data from decoder
        pTarget->flags.sequential_mode= (Fueling_Setup.features.sequential_mode_enabled == true) && (Tuareg.pDecoder->flags.phase_valid);

        //collect diagnostic data
        fueling_diag_log_event((pTarget->flags.sequential_mode == true)? FDIAG_UPD_CTRLS_SEQ : FDIAG_UPD_CTRLS_BATCH);

        /**
        VE
        /// TODO (oli#1#09/01/23): adapt to map set

        pTarget->VE_pct= (pTarget->flags.MAP_nTPS == true)? getValue_VeTable_MAP(Tuareg.pDecoder->crank_rpm, Tuareg.process.MAP_kPa) : getValue_VeTable_TPS(Tuareg.pDecoder->crank_rpm, Tuareg.process.TPS_deg);
        */

        pTarget->VE_pct= Tuareg.Tuareg_controls.VE;

        //range check
        if((pTarget->VE_pct < cMin_VE_val) || (pTarget->VE_pct > cMax_VE_val))
        {
            //no way to proceed without a valid VE
            Fatal(TID_FUELING_CONTROLS, FUELING_LOC_UPDCTRL_VE_INVALID);

            invalid_fueling_controls(pTarget);
            return;
        }

        ///AFR
        update_AFR_target(pTarget);

        ///air flow
        update_air_flow(pTarget);

        /**
        base fuel mass
        calculation based on the calculated VE and AFR values
        */
        update_base_fuel_mass(pTarget);

        //run correction algoritms
        update_fuel_mass_corrections(pTarget);

        //apply all activated corrections
        update_target_fuel_mass(pTarget);

        /**
        run X-Tau load transient compensation
        in limp mode the commanded fuel mass will be simply the target fuel mass
        */
        update_load_transient_comp(pTarget);

    }

    /**
    injector dead time
    */
    update_injector_deadtime(pTarget);


    /**
    injector intervals calculation based on the calculated target fuel mass
    */
    if(pTarget->flags.sequential_mode == true)
    {
        update_injector_intervals_sequential(pTarget);
    }
    else
    {
        update_injector_intervals_batch(pTarget);
    }


    //success
    pTarget->flags.valid= true;
}


/****************************************************************************************************************************************
*   fueling controls update helper functions - clear controls
****************************************************************************************************************************************/
void invalid_fueling_controls(volatile fueling_control_t * pTarget)
{
    pTarget->VE_pct= 0.0;
    pTarget->air_density= 0.0;
    pTarget->air_flowrate_gps= 0.0;
    pTarget->AFR_target= cDefault_AFR_target;
    pTarget->base_fuel_mass_ug= 0.0;

    pTarget->WUE_pct= 0.0;
    pTarget->ASE_pct= 0.0;
    pTarget->ASE_cycles_left= 0;
    pTarget->BARO_pct= 0.0;

    pTarget->legacy_AE_ug= 0.0;
    pTarget->legacy_AE_cycles_left= 0;

    pTarget->target_fuel_mass_ug= 0.0;
    pTarget->cmd_fuel_mass_ug= 0.0;
    pTarget->wall_fuel_mass_ug= 0.0;

    pTarget->injector_deadtime_us= 0.0;
    pTarget->injector_target_dc= 0.0;

    pTarget->injector1_interval_us= 0;
    pTarget->injector2_interval_us= 0;

    pTarget->flags.all_flags= 0;
}





/****************************************************************************************************************************************
*   fueling controls update helper functions - air density
*
*
*   air density is expected in micro grams per cm³
*
*   dens :=  (cM_air / cR_gas) * (p/T)
*                               (p/T ~ MAP/IAT)
*
*   TODO: coolant temperature and throttle shall affect the effective charge temperature calculation
*
*   calculation has to rely on the MAP and IAT values reported in process data
*
****************************************************************************************************************************************/

const F32 cMinChargeTempK= 200.0;

void update_air_flow(volatile fueling_control_t * pTarget)
{
    F32 charge_temp_K, charge_density, air_mass_ug, air_dens, air_rate;
    exec_result_t result;

    //cR_gas is a constant -> DIV/0 not possible
    charge_density= (Tuareg.process.MAP_kPa * cM_air) / cR_gas;

    //by now the charge temperature is assumed not to depend on engine state
/// TODO (oli#3#): coolant temperature and throttle shall affect the effective charge temperature calculation
/// TODO (oli#3#): altitude shall affect the effective charge density calculation
    charge_temp_K= getValue_ChargeTempMap(Tuareg.process.IAT_K, Tuareg.process.CLT_K);

    //validate parameter (DIV/0 protection)
    if(charge_temp_K < cMinChargeTempK)
    {
        //limit engine speed
        Limp(TID_FUELING_CONTROLS, FUELING_LOC_UPDAIRFLOW_CHRGTEMP_INVALID);

        //use a safe value
        charge_temp_K= Tuareg.process.IAT_K;
    }

    //export to controls
    pTarget->charge_temp_K= charge_temp_K;

    //calculate air density
    result= divide_float(charge_density, charge_temp_K, &air_dens);

    if(result != EXEC_OK)
    {
        Fatal(TID_FUELING_CONTROLS, FUELING_LOC_UPDAIRFLOW_DENS_ERR);
        pTarget->air_density= 0.0;
    }

    //export air density
    pTarget->air_density= air_dens;

    //copy from base fuel mass
    //air mass [µg] := air_density [µg/cm³] * cylinder volume [cm³] * VE [%] / 100
    air_mass_ug= (pTarget->air_density * Fueling_Setup.cylinder_volume_ccm * Tuareg.fueling_controls.VE_pct) / 100.0;

    //update air mass flow rate
    //period_us has already been validated in precondition check
    result= divide_float(air_mass_ug, Tuareg.pDecoder->crank_period_us, &air_rate);

    if(result != EXEC_OK)
    {
        Fatal(TID_FUELING_CONTROLS, FUELING_LOC_UPDAIRFLOW_RATE_ERR);
        pTarget->air_flowrate_gps= 0.0;
    }

    //export air density
    pTarget->air_flowrate_gps= air_rate;
}





/****************************************************************************************************************************************
*   fueling controls update helper functions - AFR
*
*   decides from which table to look up the AFR target
*   use a safe (default) AFR target if fuel mass calculation relates on defaults
*
*   AFR is expected in 7 .. 20 range
****************************************************************************************************************************************/
void update_AFR_target(volatile fueling_control_t * pTarget)
{
    F32 AFR_value;

    if(pTarget->flags.AFR_fallback == true)
    {
        pTarget->AFR_target= cDefault_AFR_target;
        return;
    }

/// TODO (oli#1#09/01/23): adopt to mapset
/*
    //check from which table to look up
    if(pTarget->flags.MAP_nTPS == true)
    {
        AFR_value= getValue_AfrTable_MAP(Tuareg.pDecoder->crank_rpm, Tuareg.process.MAP_kPa);
    }
    else
    {
        AFR_value= getValue_AfrTable_TPS(Tuareg.pDecoder->crank_rpm, Tuareg.process.TPS_deg);
    }
*/
    AFR_value= Tuareg.Tuareg_controls.AFRtgt;

    //do range check
    if((AFR_value > cMin_AFR_target) && (AFR_value < cMax_AFR_target))
    {
        //use value from config
        pTarget->AFR_target= AFR_value;
    }
    else
    {
        //limit engine speed
        Limp(TID_FUELING_CONTROLS, FUELING_LOC_UPDCTRL_AFR_INVALID);

        //use fallback value
        pTarget->AFR_target= cDefault_AFR_target;
        pTarget->flags.AFR_fallback= true;
    }
}



/****************************************************************************************************************************************
*   Fueling controls update - base fuel mass - cranking
*
*   calculate the fuel mass for each cylinder while the engine is cranking
****************************************************************************************************************************************/
void update_fuel_mass_cranking(volatile fueling_control_t * pTarget)
{
    F32 fuel_mass_ug;

    /**
    begin with clean controls -> this will disable all corrections and compensations, too
    */
    invalid_fueling_controls(pTarget);

    //check if dry cranking conditions are present
    if((Tuareg.errors.sensor_TPS_error == false) && (Tuareg.process.TPS_deg > Fueling_Setup.dry_cranking_TPS_thres) && (Fueling_Setup.features.dry_cranking_enabled == true))
    {
        pTarget->flags.dry_cranking= true;

        //log diag data
        fueling_diag_log_event(FDIAG_UPD_BASE_FMAS_CRK_DRY);
    }
    else
    {
        //look up the cranking mode base fuel mass from the table
        //if the CLT sensor has failed its default value will be sufficient, too
        fuel_mass_ug= getValue_CrankingFuelTable(Tuareg.process.CLT_K);

        //set consistent output values
        pTarget->base_fuel_mass_ug= fuel_mass_ug;
        pTarget->target_fuel_mass_ug= fuel_mass_ug;
        pTarget->cmd_fuel_mass_ug= fuel_mass_ug;

        //dummy valuee to produce a nice TunerStudio image
        pTarget->charge_temp_K= Tuareg.process.IAT_K;
    }
}



/****************************************************************************************************************************************
*   Fueling controls update - base fuel mass
*
*   calculate the nominal required fuel mass for each cylinder
****************************************************************************************************************************************/
void update_base_fuel_mass(volatile fueling_control_t * pTarget)
{
    F32 base_fuel_mass_ug, air_mass_ug;
    exec_result_t result;

    //air mass [µg] := air_density [µg/cm³] * cylinder volume [cm³] * VE [%] / 100
    air_mass_ug= (Tuareg.fueling_controls.air_density * Fueling_Setup.cylinder_volume_ccm * Tuareg.fueling_controls.VE_pct) / 100.0;

    //base fuel mass [µg] := air mass [ug] / AFR [1]
    //AFR_target has already been validated in get_AFR_target()
    result= divide_float(air_mass_ug, Tuareg.fueling_controls.AFR_target, &base_fuel_mass_ug);

    if(result != EXEC_OK)
    {
        Fatal(TID_FUELING_CONTROLS, FUELING_LOC_UPDBASEFUEL_MASS_ERR);
        pTarget->base_fuel_mass_ug= 0.0;
    }

    //export base fuel mass
    pTarget->base_fuel_mass_ug= base_fuel_mass_ug;

}




/****************************************************************************************************************************************
*   fueling controls update helper functions - target fuel mass
*
*   calculate the required fuel mass to be injected into each cylinder
****************************************************************************************************************************************/
void update_target_fuel_mass(volatile fueling_control_t * pTarget)
{
    VF32 rel_comp_pct= 0.0;
    VF32 target_fuel_mass_ug= pTarget->base_fuel_mass_ug;


    /**
    step #1 - relative compensation: warm up, after start, barometric
    */

    //check if after start compensation is active
    if(pTarget->flags.ASE_active == true)
    {
        rel_comp_pct= pTarget->ASE_pct;

        //log diag data
        fueling_diag_log_event(FDIAG_UPD_TGTFMASS_ASE_ACT);
    }

    //check if warm up enrichment is active
    if(pTarget->flags.WUE_active == true)
    {
        rel_comp_pct += pTarget->WUE_pct;

        //log diag data
        fueling_diag_log_event(FDIAG_UPD_TGTFMASS_WUE_ACT);
    }

    //check if BARO correction is active
    if(pTarget->flags.BARO_corr_active == true)
    {
        rel_comp_pct += pTarget->BARO_pct;

        //log diag data
        fueling_diag_log_event(FDIAG_UPD_TGTFMASS_BARO_ACT);
    }


    //check if compensation is needed
    if( abs_F32(rel_comp_pct) > 0)
    {
        //check if compensation is within interval
        if(rel_comp_pct > cMax_fuel_rel_comp_pct)
        {
            //clip compensation
            rel_comp_pct= cMax_fuel_rel_comp_pct;

            Syslog_Warning(TID_FUELING_CONTROLS, FUELING_LOC_REL_COMP_CLIP);
        }
        else if(rel_comp_pct < cMin_fuel_rel_comp_pct)
        {
            //clip compensation
            rel_comp_pct= cMin_fuel_rel_comp_pct;

            Syslog_Warning(TID_FUELING_CONTROLS, FUELING_LOC_REL_COMP_CLIP);
        }

        //enrichment of cMin_fuel_mass_comp_pct ... cMax_fuel_mass_comp_pct
        target_fuel_mass_ug += rel_comp_pct * (target_fuel_mass_ug / 100.0);
    }

    /**
    step #2 - absolute compensation: acceleration, deceleration
    */

    //check if legacy load transient compensation is active
    if(pTarget->flags.legacy_AE_active == true)
    {
        //check if compensation is within interval
        if(pTarget->legacy_AE_ug > cMax_fuel_abs_comp_ug)
        {
            //clip compensation
            pTarget->legacy_AE_ug= cMax_fuel_abs_comp_ug;

            Syslog_Warning(TID_FUELING_CONTROLS, FUELING_LOC_ABS_COMP_CLIP);
        }

        //add acceleration enrichment fuel
        target_fuel_mass_ug += pTarget->legacy_AE_ug;

        //clip to positive fuel mass
        if(target_fuel_mass_ug < 0.0)
        {
            target_fuel_mass_ug= 0.0;
        }

        //log diag data
        fueling_diag_log_event(FDIAG_UPD_TGTFMASS_AE_ACT);
    }


    /**
    export target fuel
    */
    pTarget->target_fuel_mass_ug= target_fuel_mass_ug;
}
