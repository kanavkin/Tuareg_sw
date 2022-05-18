#include "Tuareg.h"
#include "base_calc.h"

#include "Tuareg_fueling.h"
#include "Tuareg_fueling_controls.h"
#include "Fueling_syslog_locations.h"
#include "fueling_hw.h"
#include "fueling_config.h"

#include "diagnostics.h"
#include "fueling_diag.h"

#include "accel_comp.h"

///do not enable wall fuel code by now
#define WALL_FUEL_WIP

/*
built in defaults
*/
static const F32 cMax_cold_accel_pct= 30.0;
static const F32 cAccelCompMinThres= 500.0;
static const U32 cAccelCompMinRpm= 1000;



/****************************************************************************************************************************************
*   fueling controls update helper functions - acceleration fuel mass compensation - wall fuel model
****************************************************************************************************************************************/

const F32 cK1= 0.1;
const F32 cK2= 0.2;
const F32 cK3= 0.3;
const F32 cK4= 0.4;
const F32 cK5= 0.5;

//K6
const F32 cImpact_Factor= 0.3;



F32 calc_tau()
{
    F32 tau= 0.5;

	return tau;
}


F32 calc_alpha(U32 Rpm, F32 Tau)
{
    F32 alpha= 1.0;


    alpha= calc_expf(-120 / (Rpm * Tau), -1, 4);

	return alpha;
}


F32 calc_beta(F32 Alpha)
{
    F32 beta= 1.0;

	if(Alpha < cImpact_Factor)
	{
        beta= Alpha;
	}
	else
	{
        beta= cImpact_Factor;
	}

	return beta;
}





/****************************************************************************************************************************************
*   fueling controls update helper functions - load transient fuel mass compensation - wall fuel model
*
*   preconditions:
*   - engine not cranking  -> by caller function
*   - decoder rpm data valid -> by caller function
****************************************************************************************************************************************/
void update_load_transient_comp(volatile fueling_control_t * pTarget)
{
    F32 alpha, beta, tau;
    F32 cmd_fuel_mass_ug;

#ifndef WALL_FUEL_WIP
    //check preconditions
    if( (Tuareg.flags.limited_op == true) || (Tuareg.errors.fueling_config_error == true) ||
        ((Tuareg.errors.sensor_TPS_error == true) && (Tuareg.errors.sensor_MAP_error == true)) ||
        (Fueling_Setup.features.load_transient_comp_enabled == false) || (Tuareg.pDecoder->crank_rpm < cAccelCompMinRpm) )
#else
    if(true)
#endif
    {
        disable_load_transient_comp(pTarget);
    }


    tau= calc_tau();
    alpha= calc_alpha(Tuareg.pDecoder->crank_rpm, tau);
    beta= calc_beta(alpha);

	cmd_fuel_mass_ug= divide_float( (pTarget->target_fuel_mass_ug - (1.0 - alpha) * pTarget->wall_fuel_mass_ug), (1.0 - beta));

	//clip fuel mass to positive values
	if(cmd_fuel_mass_ug < 0.0)
    {
        cmd_fuel_mass_ug= 0.0;
    }

    //export compensated fuel mass to be commanded to injector subsystem
    pTarget->cmd_fuel_mass_ug= cmd_fuel_mass_ug;

	// remainder on walls from last time + new from this time
	pTarget->wall_fuel_mass_ug= alpha * pTarget->wall_fuel_mass_ug + beta * cmd_fuel_mass_ug;

	pTarget->flags.load_transient_comp= true;

}


/****************************************************************************************************************************************
*   fueling controls update helper functions - load transient fuel mass compensation - disable
****************************************************************************************************************************************/
void disable_load_transient_comp(volatile fueling_control_t * pTarget)
{
    pTarget->flags.load_transient_comp= false;
    pTarget->wall_fuel_mass_ug= 0.0;
    pTarget->cmd_fuel_mass_ug= pTarget->target_fuel_mass_ug;
}




/****************************************************************************************************************************************
*   fueling controls update helper functions - acceleration fuel mass compensation - legacy
*
*   calculate the required fuel mass compensation for the present throttle change rate
*   any new trigger condition will enlarge the fuel mass compensation interval
*   accelerating shortly after decelerating will activate enrichment
****************************************************************************************************************************************/
void update_legacy_AE(volatile fueling_control_t * pTarget)
{
    F32 comp_MAP_ug, comp_TPS_ug, comp_ug, scaling;

    //check preconditions
    if((Fueling_Setup.features.legacy_AE_enabled == false) || (Tuareg.pDecoder->crank_rpm < cAccelCompMinRpm))
    {
        disable_legacy_AE(pTarget);
        return;
    }


    /**
    check if acceleration compensation shall be (re)triggered
    */
    if((Tuareg.process.ddt_TPS >= Fueling_Setup.accel_comp_thres_TPS) || (Tuareg.process.ddt_MAP >= Fueling_Setup.accel_comp_thres_MAP))
    {
        /**
        engine is accelerating - turn on AE
        */
        pTarget->flags.legacy_AE= true;
        pTarget->legacy_AE_cycles_left= Fueling_Setup.accel_comp_cycles;

        //look up the correction factors
        comp_TPS_ug= getValue_AccelCompTableTPS(Tuareg.process.ddt_TPS);
        comp_MAP_ug= getValue_AccelCompTableMAP(Tuareg.process.ddt_MAP);

        //select the greater one
        comp_ug= (comp_TPS_ug > comp_MAP_ug)? comp_TPS_ug : comp_MAP_ug;

        //add extra fuel if engine is cold
        if((pTarget->flags.WUE_active == true) && (Fueling_Setup.cold_accel_pct > 0.99))
        {
            //check if the fuel config provides valid data
            if(Fueling_Setup.cold_accel_pct > cMax_cold_accel_pct)
            {
                //set a valid default
                Fueling_Setup.cold_accel_pct= cMax_cold_accel_pct;

                //emit a warning
                Syslog_Warning(TID_FUELING_CONTROLS, FUELING_LOC_ACCELCOMP_CLIP_COLD_ACCEL_PCT);
            }

            comp_ug += Fueling_Setup.cold_accel_pct * (comp_ug / 100.0);
        }

        /**
        scale compensation value to engine speed

        -> rpm scaling is considered active when thres rpm is smaller then max rpm
        */
        if((Fueling_Setup.accel_comp_scaling_max_rpm > Fueling_Setup.accel_comp_scaling_thres_rpm) && (Tuareg.pDecoder->crank_rpm > Fueling_Setup.accel_comp_scaling_thres_rpm))
        {
            scaling= 1.0 - divide_F32(subtract_U32(Tuareg.pDecoder->crank_rpm, Fueling_Setup.accel_comp_scaling_thres_rpm), subtract_U32(Fueling_Setup.accel_comp_scaling_max_rpm, Fueling_Setup.accel_comp_scaling_thres_rpm));

            if(scaling > 1.0)
            {
                //use a valid default
                scaling= 0.95;

                //emit an error message
                Syslog_Error(TID_TUAREG_FUELING, FUELING_LOC_ACCELCOMP_RPMSCALE);
            }

            comp_ug *= scaling;

            //log diag data
            fueling_diag_log_event(FDIAG_UPD_ACCELCOMP_RPMSCALED);
        }

        //export
        pTarget->legacy_AE_ug= comp_ug;

        //log diag data
        fueling_diag_log_event(FDIAG_UPD_ACCELCOMP_ACCEL);

    }
    else if((Tuareg.process.ddt_TPS <= Fueling_Setup.decel_comp_thres_TPS) || (Tuareg.process.ddt_MAP <= Fueling_Setup.decel_comp_thres_MAP))
    {
        /**
        engine is decelerating

        -> lean out mixture if system condition allows this

        (this is a feature to save fuel, choose a rich mixture when limit operation strategy is active)
        */
        if(Tuareg.flags.limited_op == false)
        {
            pTarget->flags.legacy_AE= true;
            pTarget->legacy_AE_cycles_left= Fueling_Setup.decel_comp_cycles;

            //export the correction factor
            pTarget->legacy_AE_ug= -Fueling_Setup.decel_comp_ug;

            //log diag data
            fueling_diag_log_event(FDIAG_UPD_ACCELCOMP_DECEL);
        }
        else
        {
            disable_legacy_AE(pTarget);
        }
    }
    else if(pTarget->legacy_AE_cycles_left > 0)
    {
        /**
        transient compensation is already active
        */

        /**
        scale compensation value down according accel taper

        -> the accel taper feature is considered active when remaining cycle threshold value < accel comp cycles
        -> then the AE percentage will be scaled when less cycles left than accel_comp_taper_thres

        The acceleration correction taper is implemented as a geometric sequence: y_n :=  k * alpha^(n+1)
        (k is the initial value y_0) Its factor alpha can be projected via the following relation: y_n / k = 1 / 2
        -> solve (n+1) = (-ln 2) / (ln alpha) to alpha:

        -> Which factor alpha makes the correction to drop to a half of the initial value after n events?
        -> alpha := 1 / 2^(n+1)

        example: alpha := 0.9 and n := 9

        --

        accel taper does not apply to deceleration!

        */

        //check if acceleration shall be compensated
        if(pTarget->legacy_AE_ug > 0.0)
        {
            //check if compensation will have practical effect
            if(pTarget->legacy_AE_ug < cAccelCompMinThres)
            {
                //end compensation
                disable_legacy_AE(pTarget);
                return;
            }

            if((Fueling_Setup.accel_comp_cycles > Fueling_Setup.accel_comp_taper_thres) && (pTarget->legacy_AE_cycles_left < Fueling_Setup.accel_comp_taper_thres))
            {
                //scale the fuel correction
                pTarget->legacy_AE_ug *= Fueling_Setup.accel_comp_taper_factor;

                //log diag data
                fueling_diag_log_event(FDIAG_UPD_ACCELCOMP_TAPERED);
            }

        }


        //one compensation cycle has been consumed
        pTarget->legacy_AE_cycles_left -= 1;

    }
    else
    {
        /**
        transient compensation interval has expired or compensation currently not active - set consistent output data
        */
        disable_legacy_AE(pTarget);
    }

}



/****************************************************************************************************************************************
*   fueling controls update helper functions - legacy acceleration fuel mass compensation - disable
****************************************************************************************************************************************/
void disable_legacy_AE(volatile fueling_control_t * pTarget)
{
    pTarget->flags.legacy_AE= false;
    pTarget->legacy_AE_cycles_left= 0;
    pTarget->legacy_AE_ug= 0.0;
}


