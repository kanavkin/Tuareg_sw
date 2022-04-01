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
const crank_position_t fueling_controls_update_pos= CRK_POSITION_B1;


/**
The fueling module relies on external triggers to start injection, the scheduler is allocated only to maintain the injector interval
*/

void init_Fueling()
{
    exec_result_t result;

    //setup shall be loaded first
    result= load_Fueling_Config();

    //check if config has been loaded
    if(result != EXEC_OK)
    {
        /**
        failed to load Fueling Config
        */
        Tuareg.errors.fueling_config_error= true;

        //enter limp mode
        Limp(TID_TUAREG_FUELING, FUELING_LOC_CONFIGLOAD_ERROR);

        //load built in defaults
        load_essential_Fueling_Config();

        #ifdef FUELING_DEBUGMSG
        DebugMsg_Error("Failed to load Fueling config!");
        DebugMsg_Warning("Fueling essential config has been loaded");
        #endif // FUELING_DEBUGMSG
    }
    else if(Fueling_Setup.Version != FUELING_REQUIRED_CONFIG_VERSION)
    {
        /**
        loaded wrong Fueling Config Version
        */
        Tuareg.errors.fueling_config_error= true;

        //enter limp mode
        Limp(TID_TUAREG_FUELING, FUELING_LOC_CONFIGVERSION_ERROR);

        //load built in defaults
        load_essential_Fueling_Config();

        #ifdef FUELING_DEBUGMSG
        DebugMsg_Error("Fueling config version does not match");
        DebugMsg_Warning("Fueling essential config has been loaded");
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
    volatile scheduler_activation_parameters_t scheduler_parameters;

    //collect diagnostic information
    fueling_diag_log_event(FDIAG_CRKPOSH_CALLS);

    /**
    check vital preconditions
    */
    if((Tuareg.flags.run_inhibit == true) || (Tuareg.fueling_controls.flags.valid == false))
    {
        //collect diagnostic information
        fueling_diag_log_event(FDIAG_CRKPOSH_VIT_PRECOND_FAIL);

        //turn off injectors
        set_injector1(ACTOR_UNPOWERED);
        set_injector2(ACTOR_UNPOWERED);
        scheduler_reset_channel(SCHEDULER_CH_FUEL1);
        scheduler_reset_channel(SCHEDULER_CH_FUEL2);

        //delete fueling controls
        //replace by invalid_fueling_controls?
        Tuareg_update_fueling_controls();

        //nothing to do
        return;
    }


    //check if fueling controls shall be updated
    if(Tuareg.pDecoder->crank_position == fueling_controls_update_pos)
    {
        //update fueling controls
        Tuareg_update_fueling_controls();
    }


    //check if the crank is at the injection begin position
    if(Tuareg.pDecoder->crank_position == Tuareg.fueling_controls.injection_begin_pos)
    {
        //collect diagnostic information
        fueling_diag_log_event(FDIAG_CRKPOSH_INJBEG_POS);

        //check if sequential mode has been requested
        if(Tuareg.fueling_controls.flags.sequential_mode == true)
        {
            //check if sufficient information for this mode is available
            if(Tuareg.pDecoder->flags.phase_valid == false)
            {
                //collect diagnostic information
                fueling_diag_log_event(FDIAG_CRKPOSH_SEQ_ERROR);

                //early exit
                return;
            }

            /**
            sequential mode
            */
            scheduler_parameters.flags.action1_power= true;
            scheduler_parameters.flags.action2_power= false;
            scheduler_parameters.flags.interval2_enabled= true;
            scheduler_parameters.flags.complete_cycle_realloc= false;

            //injector #1
            if(Tuareg.fueling_controls.seq_injector1_begin_phase == Tuareg.pDecoder->phase)
            {
                scheduler_parameters.interval1_us= Tuareg.fueling_controls.injector1_timing_us;
                scheduler_parameters.interval2_us= Tuareg.fueling_controls.injector1_interval_us;
                scheduler_set_channel(SCHEDULER_CH_FUEL1, &scheduler_parameters);

                //register fuel mass injected
                Tuareg.injected_mass_ug += Tuareg.fueling_controls.target_fuel_mass_ug;

                //collect diagnostic information
                fueling_diag_log_event(FDIAG_CRKPOSH_INJBEG1_SEQ);
            }

            //injector #2
            if(Tuareg.fueling_controls.seq_injector2_begin_phase == Tuareg.pDecoder->phase)
            {
                scheduler_parameters.interval1_us= Tuareg.fueling_controls.injector2_timing_us;
                scheduler_parameters.interval2_us= Tuareg.fueling_controls.injector2_interval_us;
                scheduler_set_channel(SCHEDULER_CH_FUEL2, &scheduler_parameters);

                //register fuel mass injected
                Tuareg.injected_mass_ug += Tuareg.fueling_controls.target_fuel_mass_ug;

                //collect diagnostic information
                fueling_diag_log_event(FDIAG_CRKPOSH_INJBEG2_SEQ);
            }

        }
        else
        {
            /**
            batch mode
            --> immediate injection begin, scheduler controls injection end
            */
            scheduler_parameters.flags.action1_power= false;
            scheduler_parameters.flags.action2_power= false;
            scheduler_parameters.flags.interval2_enabled= false;
            scheduler_parameters.flags.complete_cycle_realloc= false;
            scheduler_parameters.interval2_us= 0;

            //injector #1
            scheduler_parameters.interval1_us= Tuareg.fueling_controls.injector1_interval_us;
            set_injector1(ACTOR_POWERED);
            scheduler_set_channel(SCHEDULER_CH_FUEL1, &scheduler_parameters);

            //injector #2
            scheduler_parameters.interval1_us= Tuareg.fueling_controls.injector2_interval_us;
            set_injector2(ACTOR_POWERED);
            scheduler_set_channel(SCHEDULER_CH_FUEL2, &scheduler_parameters);

            //collect diagnostic information
            fueling_diag_log_event(FDIAG_CRKPOSH_INJBEG_BATCH);
        }

        /**
        register fuel mass injected
        in sequential mode either injector #1 or #2 injects the full target fuel mass
        in batch mode each injector injects half of the target fuel mass
        */
        Tuareg.injected_mass_ug += Tuareg.fueling_controls.target_fuel_mass_ug;
    }
}


