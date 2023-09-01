#include "Tuareg.h"
#include "base_calc.h"

#include "Tuareg_fueling.h"
#include "Tuareg_fueling_controls.h"
#include "Fueling_syslog_locations.h"
#include "fueling_hw.h"
#include "fueling_config.h"

#include "diagnostics.h"
#include "fueling_diag.h"

#include "fueling_corrections.h"
#include "accel_comp.h"




/****************************************************************************************************************************************
*   fueling controls update wrapper function
****************************************************************************************************************************************/
void update_fuel_mass_corrections(volatile fueling_control_t * pTarget)
{
    //after start compensation
    update_fuel_mass_afterstart_correction(pTarget);

    //warm up compensation
    update_fuel_mass_warmup_correction(pTarget);

    //barometric pressure compensation
    update_fuel_mass_barometric_correction(pTarget);


    //legacy AE - load transient compensation
    update_legacy_AE(pTarget);
}



/*
built in defaults
*/
const U32 cAfterstart_begin_thres= 3;
const U32 cAfterstart_thres= 50;




/****************************************************************************************************************************************
*   fueling controls update helper functions - after start fuel mass correction
*
*   adds extra fuel for the first seconds of an cold engine that has just started
*
*   cannot be activated without valid CLT information
****************************************************************************************************************************************/
void update_fuel_mass_afterstart_correction(volatile fueling_control_t * pTarget)
{
    //check preconditions
    if((Fueling_Setup.features.ASE_enabled == false) || (Tuareg.errors.sensor_CLT_error == true))
    {
        disable_fuel_mass_afterstart_correction(pTarget);
        return;
    }

    //check if the after start compensation is already enabled
    if(pTarget->flags.ASE_active == true)
    {
        //check if the after start cycle counter has expired
        if(pTarget->ASE_cycles_left > 0)
        {
            pTarget->ASE_cycles_left -= 1;
        }
        else
        {
            //after start enrichment counter expired - deactivate
            disable_fuel_mass_afterstart_correction(pTarget);
        }
    }
    else
    {
        //check if the engine has just started and the coolant is still cold
        if((Tuareg.process.engine_runtime > cAfterstart_begin_thres) && (Tuareg.process.engine_runtime < cAfterstart_thres) && (Tuareg.process.CLT_K < Fueling_Setup.afterstart_thres_K))
        {
            //ase activated
            pTarget->flags.ASE_active= true;
            pTarget->ASE_pct= Fueling_Setup.afterstart_comp_pct;
            pTarget->ASE_cycles_left= Fueling_Setup.afterstart_comp_cycles;
        }
    }

}


void disable_fuel_mass_afterstart_correction(volatile fueling_control_t * pTarget)
{
    pTarget->flags.ASE_active= false;
    pTarget->ASE_pct= 0.0;
    pTarget->ASE_cycles_left= 0;
}





/****************************************************************************************************************************************
*   fueling controls update helper functions - warm up fuel mass correction
*
*   calculate the fuel mass compensation factor for engine warmup
*
*   cannot be activated without valid CLT information
****************************************************************************************************************************************/
void update_fuel_mass_warmup_correction(volatile fueling_control_t * pTarget)
{
    F32 warmup_comp;

    //check preconditions
    if((Tuareg.errors.sensor_CLT_error == true) || (Fueling_Setup.features.WUE_enabled == false))
    {
        pTarget->flags.WUE_active= false;
        pTarget->WUE_pct= 0.0;
        return;
    }

    //look up the correction factor and export
    warmup_comp= getValue_WarmUpCompTable(Tuareg.process.CLT_K);

    pTarget->WUE_pct= warmup_comp;
    pTarget->flags.WUE_active= (warmup_comp > 0.0) ? true : false;

}



/****************************************************************************************************************************************
*   fueling controls update helper functions - barometric fuel mass correction
*
*   calculate the fuel mass compensation factor for changing barometric pressure
*
****************************************************************************************************************************************/

const F32 cBaroMaxCompRich= 50.0;
const F32 cBaroMaxCompLean= -30.0;


void update_fuel_mass_barometric_correction(volatile fueling_control_t * pTarget)
{
    F32 baro_comp;

    //check preconditions
    if((Tuareg.errors.sensor_BARO_error == true) || (Fueling_Setup.features.BARO_correction_enabled == false))
    {
        pTarget->flags.BARO_corr_active= false;
        pTarget->BARO_pct= 0.0;
        return;
    }

    //look up the correction factor and export
    baro_comp= getValue_BAROtable(Tuareg.process.Baro_kPa);

    //check if the correction factor is valid
    if((baro_comp > cBaroMaxCompRich) || (baro_comp < cBaroMaxCompLean))
    {
        pTarget->flags.BARO_corr_active= false;
        pTarget->BARO_pct= 0.0;

        //log the error
        Syslog_Error(TID_FUELING_CORRECTIONS, FUELING_LOC_BAROCORR_INVALID);

        return;
    }

    //use the correction factor
    pTarget->BARO_pct= baro_comp;

    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wfloat-equal"

    pTarget->flags.BARO_corr_active= ( baro_comp != 0.0) ? true : false;

    #pragma GCC diagnostic pop

}

