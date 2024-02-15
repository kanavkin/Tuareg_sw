#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/boctok_types.h"


#include "Tuareg.h"
#include "Tuareg_decoder.h"

#include "Tuareg_fueling.h"
#include "Tuareg_fueling_controls.h"
#include "fueling_hw.h"
#include "fueling_config.h"
#include "Fueling_syslog_locations.h"
#include "fueling_diag.h"

#include "base_calc.h"
#include "scheduler.h"
#include "syslog.h"
#include "diagnostics.h"


//#define FUELING_DEBUGMSG

#ifdef FUELING_DEBUGMSG
#include "debug_port_messages.h"
#warning Fueling debug outputs enabled
#endif // FUELING_DEBUGMSG



/**
crank position when the fueling controls shall be updated
*/
const crank_position_t c_fueling_controls_update_pos= CRK_POSITION_A2;


/**
The fueling module relies on external triggers to start injection, the scheduler is allocated only to maintain the injector interval
*/

void init_Fueling()
{
    exec_result_t result;

    //setup shall be loaded first
    result= load_Fueling_Config();

    //check if config has been loaded
    if((result != EXEC_OK) || (Fueling_Setup.Version != FUELING_REQUIRED_CONFIG_VERSION))
    {
        /**
        failed to load Fueling Config
        */
        Tuareg.errors.fueling_config_error= true;

        //no engine operation possible
        Fatal(TID_TUAREG_FUELING, FUELING_LOC_CONFIG_ERROR);

        #ifdef FUELING_DEBUGMSG
        DebugMsg_Error("Failed to load Fueling config!");
        #endif // FUELING_DEBUGMSG
    }
    else
    {
        //loaded Fueling config with correct Version
        Tuareg.errors.fueling_config_error= false;

    }

    //init hw part
    init_fueling_hw();

    //bring up vital scheduler
    init_Vital_Scheduler();

    Syslog_Info(TID_TUAREG_FUELING, FUELING_LOC_READY);
}


/**
emits the control events to actor (scheduler / injector / pump) layer

precondition:
Tuareg.pDecoder->outputs.timeout == false
Tuareg.pDecoder->outputs.position_valid == true

test result: ~61 us delay from signal edge B2 to injection begin
*/
void Tuareg_fueling_update_crankpos_handler()
{
    scheduler_activation_parameters_t scheduler_parameters;
    bool trigger_injector1= false, trigger_injector2= false;

    //collect diagnostic information
    fueling_diag_log_event(FDIAG_CRKPOSH_CALLS);


    /**
    check if the crank is at the injection reference position -> injection begin
    */
    if(Tuareg.pDecoder->crank_position == Fueling_Setup.injection_reference_pos)
    {
        //collect diagnostic information
        fueling_diag_log_event(FDIAG_CRKPOSH_INJBEG_POS);

        //check vital preconditions
        if((Tuareg.flags.run_allow == true) && (Tuareg.Tuareg_controls.fueling_controls.flags.valid == true))
        {
            //check the commanded mode
            if(Tuareg.Tuareg_controls.fueling_controls.flags.sequential_mode == true)
            {
                //check if phase information is available
                if(Tuareg.pDecoder->flags.phase_valid == true)
                {
                    //trigger injector 1 when the first cylinder is at the exhaust stroke
                    trigger_injector1= (Tuareg.pDecoder->phase == PHASE_CYL1_EX);

                    //trigger injector 2 when the second cylinder is at the exhaust stroke
                    //in an 180° engine this happens when the first cylinder is at the compression stroke
                    trigger_injector1= !trigger_injector1;
                }
                else
                {
                    //no injection - collect diagnostic information only
                    fueling_diag_log_event(FDIAG_CRKPOSH_SEQ_ERROR);
                }

            }
            else
            {
                //batch mode - trigger all injectors
                trigger_injector1= true;
                trigger_injector2= true;
            }


            /*
            prepare scheduler activation parameters
            - first action is injection end -> turn off injector
            - second action is N/A
            - use 1 interval
            - no realloc completion
            - interval 1 -> injection interval
            - interval 2 -> N/A
            */
            scheduler_parameters.flags.action1_power= false;
            scheduler_parameters.flags.interval2_enabled= false;
            scheduler_parameters.interval2_us= 0;
            scheduler_parameters.flags.action2_power= false;
            scheduler_parameters.flags.complete_cycle_realloc= false;


            /**
            injector #1
            */
            if(trigger_injector1 == true)
            {
                //trigger scheduler
                scheduler_reset_channel(SCHEDULER_CH_FUEL1);
                scheduler_parameters.interval1_us= Tuareg.Tuareg_controls.fueling_controls.injector1_interval_us;
                scheduler_set_channel(SCHEDULER_CH_FUEL1, &scheduler_parameters);

                //trigger actor
                set_injector1(ACTOR_POWERED);

                //register fuel mass injected
                Tuareg.process.fuel_mass_integrator_1s_ug += Tuareg.Tuareg_controls.fueling_controls.target_fuel_mass_ug;

                //collect diagnostic information
                fueling_diag_log_event(FDIAG_CRKPOSH_INJBEG_1);
            }


            /**
            injector #2
            */
            if(trigger_injector2 == true)
            {
                //trigger scheduler
                scheduler_reset_channel(SCHEDULER_CH_FUEL2);
                scheduler_parameters.interval1_us= Tuareg.Tuareg_controls.fueling_controls.injector2_interval_us;
                scheduler_set_channel(SCHEDULER_CH_FUEL2, &scheduler_parameters);

                //trigger actor
                set_injector2(ACTOR_POWERED);

                //register fuel mass injected
                Tuareg.process.fuel_mass_integrator_1s_ug += Tuareg.Tuareg_controls.fueling_controls.target_fuel_mass_ug;

                //collect diagnostic information
                fueling_diag_log_event(FDIAG_CRKPOSH_INJBEG_2);
            }
        }
        else
        {
            //collect diagnostic information
            fueling_diag_log_event(FDIAG_CRKPOSH_VIT_PRECOND_FAIL);
        }
    }


    /**
    check if fueling controls shall be updated
    */
    if(Tuareg.pDecoder->crank_position == c_fueling_controls_update_pos)
    {
        //update fueling controls
        Tuareg_update_fueling_controls();
    }


}


