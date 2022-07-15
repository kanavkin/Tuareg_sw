#include <Tuareg_platform.h>
#include <Tuareg.h>

#define IGNITION_REQUIRED_CONFIG_VERSION 5


//#define IGNITION_DEBUGMSG

#ifdef IGNITION_DEBUGMSG
#warning debug outputs enabled
#endif // IGNITION_DEBUGMSG


/**
crank position when the ignition controls shall be updated
*/
const crank_position_t ignition_controls_update_pos= CRK_POSITION_B2;


/**
How the Tuareg Ignition system works:

- With default or cranking ignition controls dwell shall begin immediately when the decoder indicates that the crank position is at dwell_pos.
- With default or cranking ignition controls ignition shall occur immediately when the decoder indicates that the crank position is at ignition_pos.

- With dynamic ignition controls the scheduler shall start coil dwell after the delay indicated by dwell_timing_sequential_us or dwell_timing_batch_us right after the spark has fired
- With dynamic ignition controls the scheduler shall trigger the spark after the delay indicated by ignition_timing_us after the decoder indicates that the crank position is at ignition_pos.

IMPLICATIONS

-> having only one scheduler for each coil controlling dwell and ignition events means that this scheduler can be allocated for dwell OR ignition timing at a time
-> conflicts arise when allocating the scheduler for ignition timing and dwell has not yet begun
-> to address this an override option "SCHEDOPT_REALLOC_COMPLETE" has been implemented


*/


void init_Ignition()
{
    exec_result_t result;

    //setup shall be loaded first
    result= load_Ignition_Config();

    //check if config has been loaded
    if((result != EXEC_OK) || (Ignition_Setup.Version != IGNITION_REQUIRED_CONFIG_VERSION))
    {
        /**
        failed to load Ignition Config
        */
        Tuareg.errors.ignition_config_error= true;

        //no engine operation possible
        Fatal(TID_TUAREG_IGNITION, IGNITION_LOC_CONFIG_ERROR);

        #ifdef IGNITION_DEBUGMSG
        DebugMsg_Error("Failed to load Ignition config!");
        #endif // IGNITION_DEBUGMSG
    }
    else
    {
        //loaded Ignition config with correct Version
        Tuareg.errors.ignition_config_error= false;
    }

    //init hw part
    init_ignition_hw();

    //bring up vital scheduler
    init_Vital_Scheduler();

    //provide ignition controls for startup
    Tuareg_update_ignition_controls();

    Syslog_Info(TID_TUAREG_IGNITION, IGNITION_LOC_READY);
}


/**
emits the control events to actor (scheduler / coil) layer

precondition:
Tuareg.pDecoder->outputs.timeout == false
Tuareg.pDecoder->outputs.position_valid == true
*/
void Tuareg_ignition_update_crankpos_handler()
{
    volatile scheduler_activation_parameters_t scheduler_parameters;

    //collect diagnostic information
    ignition_diag_log_event(IGNDIAG_CRKPOSH_CALLS);

    /**
    check vital preconditions
    */
    if(Tuareg.flags.run_inhibit == true)
    {
        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_CRKPOSH_INHIBIT);

        //turn off all ignition actors
        set_ignition_ch1(ACTOR_UNPOWERED);
        set_ignition_ch2(ACTOR_UNPOWERED);

        //delete ignition controls
        Tuareg_update_ignition_controls();

        //nothing to do
        return;
    }

    //check if ignition controls shall be updated
    if(Tuareg.pDecoder->crank_position == ignition_controls_update_pos)
    {
        //update ignition controls
        Tuareg_update_ignition_controls();
    }


    //check if ignition controls are valid
    if(Tuareg.ignition_controls.flags.valid == false)
    {
        //collect diagnostic information
        ignition_diag_log_event(IGNDIAG_CRKPOSH_CTRLS_INVALID);

        //nothing to do
        return;
    }


    //check if dynamic mode is active - this is the main operation scenario
    if(Tuareg.ignition_controls.flags.dynamic_controls == true)
    {
        //check if the crank is at the ignition base position
        if(Tuareg.pDecoder->crank_position == Tuareg.ignition_controls.ignition_pos)
        {
            //collect diagnostic information
            ignition_diag_log_event(IGNDIAG_CRKPOSH_IGNPOS);

            /*
            prepare scheduler activation parameters
            - first action is ignition -> turn off coil
            - second action is dwell -> power coil
            - use 2 intervals
            - use realloc completion (when dwell is shorter than the ignition timing) -> start dwell at the ignition base position
            - interval 1 -> corrected ignition timing
            - interval 2 -> dwell timing
            */
            scheduler_parameters.flags.action1_power= false;
            scheduler_parameters.flags.action2_power= true;
            scheduler_parameters.flags.interval2_enabled= true;
            scheduler_parameters.flags.complete_cycle_realloc= true;

            scheduler_parameters.interval2_us= Tuareg.ignition_controls.dwell_timing_us;
            scheduler_parameters.interval1_us= subtract_U32(Tuareg.ignition_controls.ignition_timing_us, decoder_get_position_data_age_us());


            //check if sequential mode has been commanded
            if((Tuareg.ignition_controls.flags.sequential_mode == true) && (Tuareg.pDecoder->flags.phase_valid == false))
            {
                //collect diagnostic information
                ignition_diag_log_event(IGNITION_LOC_SEQUENTIAL_FAIL);

                /**
                downgrade to batch mode is not possible because of specific interval calculation
                earlyexit
                */
                return;
            }

            //coil #1
            if((Tuareg.pDecoder->phase == PHASE_CYL1_COMP) || (Tuareg.ignition_controls.flags.sequential_mode == false))
            {
                scheduler_set_channel(SCHEDULER_CH_IGN1, &scheduler_parameters);

                //collect diagnostic information
                ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN1SCHED);
            }

            //coil #2
            if( (Ignition_Setup.flags.second_coil_installed == true) && ((Tuareg.pDecoder->phase == PHASE_CYL1_EX) || (Tuareg.ignition_controls.flags.sequential_mode == false)))
            {
                scheduler_set_channel(SCHEDULER_CH_IGN2, &scheduler_parameters);

                //collect diagnostic information
                ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN2SCHED);
            }
       }

    }
    else
    {
        /**
        Everything other than dynamic mode is fallback -> batch style
        use cases: cranking, default ignition controls
        operating scheme: immediate spark/dwell triggering on position update
        no scheduler allocation
        */
        if(Tuareg.pDecoder->crank_position == Tuareg.ignition_controls.ignition_pos)
        {
            scheduler_reset_channel(SCHEDULER_CH_IGN1);
            scheduler_reset_channel(SCHEDULER_CH_IGN2);

            //coil #1 and #2
            set_ignition_ch1(ACTOR_UNPOWERED);
            set_ignition_ch2(ACTOR_UNPOWERED);

            //collect diagnostic information
            ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN1_UNPOWER);
            ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN2_UNPOWER);

        }
        else if(Tuareg.pDecoder->crank_position == Tuareg.ignition_controls.dwell_pos)
        {
            scheduler_reset_channel(SCHEDULER_CH_IGN1);
            scheduler_reset_channel(SCHEDULER_CH_IGN2);

            //coil #1
            set_ignition_ch1(ACTOR_POWERED);

            //collect diagnostic information
            ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN1_POWER);

            //coil #2
            if(Ignition_Setup.flags.second_coil_installed == true)
            {
                set_ignition_ch2(ACTOR_POWERED);

                //collect diagnostic information
                ignition_diag_log_event(IGNDIAG_CRKPOSH_IGN2_POWER);
            }
        }
    }
}
