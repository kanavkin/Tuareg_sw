#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_adc.h"
#include "stm32_libs/boctok_types.h"

#include "base_calc.h"
#include "bitfields.h"
#include "Tuareg_decoder.h"

#include "Tuareg_ignition.h"
#include "ignition_hw.h"
#include "ignition_config.h"

#include "Tuareg_fueling.h"

#include "Tuareg_sensors.h"

#include "scheduler.h"
#include "lowprio_scheduler.h"
#include "uart.h"
#include "conversion.h"
#include "systick_timer.h"
#include "Tuareg_console.h"
#include "Tuareg_config.h"
#include "table.h"
#include "eeprom.h"
#include "sensors.h"
#include "fueling_hw.h"
#include "Tuareg_fueling_controls.h"

#include "dash_hw.h"
#include "dash_logic.h"
#include "act_hw.h"
#include "act_logic.h"

#include "process_table.h"

#include "diagnostics.h"
#include "Tuareg.h"
#include "uart_printf.h"
#include "module_test.h"

#include "syslog.h"
#include "debug_port_messages.h"
#include "Tuareg_syslog_locations.h"

#include "highspeed_loggers.h"


#define TUAREG_DEBUG_OUTPUT

#ifdef TUAREG_DEBUG_OUTPUT
#warning debug outputs enabled
#endif // TUAREG_DEBUG_OUTPUT


//level at which the crash sensor reports a crash event
#define CRASH_SENSOR_ENGAGE_LEVEL (1<< DSENSOR_CRASH)

//level at which the run sensor reports a run permission
#define RUN_SENSOR_ENGAGE_LEVEL (1<< DSENSOR_RUN)

//transitional config items
static const bool cCrashSensorTrigger_on_high= true;
static const bool cRunSensorTrigger_on_high= true;
static const bool cSidestandSensorTrigger_on_high= true;



void Tuareg_print_init_message()
{
    #ifdef TUAREG_MODULE_TEST
    moduletest_initmsg_action();
    #else

        #ifdef TUAREG_DEBUG_OUTPUT
        print(DEBUG_PORT, "\r \n \r \n . \r \n . \r \n . \r \n \r \n *** This is Tuareg, lord of the Sahara *** \r \n");
        print(DEBUG_PORT, "V 0.2");
        #endif // TUAREG_DEBUG_OUTPUT

    #endif
}


/******************************************************************************************************************************
INIT
******************************************************************************************************************************/

void Tuareg_Init()
{
    exec_result_t result;

    /**
    system init
    */

    //collect diagnostic information
    tuareg_diag_log_event(TDIAG_ENTER_INIT);

    //use 16 preemption priority levels
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    /**
    initialize core components
    */

    //initialize systick timer to provide system timestamp
    Tuareg.pTimer= init_systick_timer();
    Tuareg.pSyslog= Syslog_init();

    Eeprom_init();

    //init_dash_hw();
    //init_act_hw();

    //DEBUG
    //init_debug_pins();
    //set_debug_pin(PIN_ON);
    //dwt_init();

    UART_DEBUG_PORT_Init();
    Tuareg_print_init_message();

    //allows Tuareg_set_Runmode() to switch to next Runmode
    Tuareg.Runmode= TMODE_INIT;

    Tuareg.actors.ignition_inhibit= true;
    Tuareg.actors.fueling_inhibit= true;


    /**
    initialize core components and register interface access pointers
    */
    Tuareg.pDecoder= init_Decoder();

    Tuareg.pSensors= init_Sensors();

    init_Ignition();
    init_Fueling();

    init_scheduler();

    Tuareg.pHighspeedlog= highspeedlog_init();

    init_lowprio_scheduler();

    init_fueling_hw();


    init_dash_logic();
    init_act_logic();

    Tuareg_init_console();





    //loading the config data is essential, failure forces "limp home mode"
    result= load_Tuareg_Setup();

    //check if config has been loaded
    if(result != EXEC_OK)
    {
        //failed to load config
        Tuareg.Errors.tuareg_config_error= true;
        load_essential_Tuareg_Setup();

        Syslog_Error(TID_TUAREG, TUAREG_LOC_LOAD_CONFIG_FAIL);
        Syslog_Warning(TID_TUAREG, TUAREG_LOC_ESSENTIALS_CONFIG_LOADED);

        #ifdef TUAREG_DEBUG_OUTPUT
        DebugMsg_Error("Failed to load Tuareg config");
        DebugMsg_Warning("Tuareg essential config has been loaded");
        #endif // TUAREG_DEBUG_OUTPUT
    }
    else if(Tuareg_Setup.Version != TUAREG_REQUIRED_CONFIG_VERSION)
    {
        //loaded wrong config version
        Tuareg.Errors.tuareg_config_error= true;
        load_essential_Tuareg_Setup();

        Syslog_Error(TID_TUAREG, TUAREG_LOC_LOAD_CONFIG_VERSION_FAIL);
        Syslog_Warning(TID_TUAREG, TUAREG_LOC_ESSENTIALS_CONFIG_LOADED);

        #ifdef TUAREG_DEBUG_OUTPUT
        DebugMsg_Error("Tuareg config loaded version does not match");
        DebugMsg_Warning("Tuareg essential config has been loaded");
        #endif // TUAREG_DEBUG_OUTPUT
    }
    else
    {
        //loaded config with correct version
        Tuareg.Errors.tuareg_config_error= false;

        Syslog_Info(TID_TUAREG, TUAREG_LOC_LOAD_CONFIG_SUCCESS);
    }
}





inline void Tuareg_stop_engine()
{
    //set inhibit state
    Tuareg.actors.ignition_inhibit= true;
    Tuareg.actors.fueling_inhibit= true;

    //turn off ignition system
    set_ignition_ch1(ACTOR_UNPOWERED);
    set_ignition_ch2(ACTOR_UNPOWERED);

    //turn off fueling system
    set_fuel_pump(ACTOR_UNPOWERED);
    set_injector1(ACTOR_UNPOWERED);
    set_injector2(ACTOR_UNPOWERED);

    //reset scheduler
    scheduler_reset_ign1();
    scheduler_reset_ign2();
    //fuel



    //reset MAP calculation
    reset_asensor_sync_integrator(ASENSOR_SYNC_MAP);
}



/**
checks if the RUN switch, SIDESTAND sensor or CRASH sensor indicate a HALT condition
*/
void Tuareg_update_halt_sources()
{
    //shut engine off if the RUN switch is DISENGAGED
    Tuareg.Halt_source.run_switch= (getBit_BF8(DSENSOR_RUN, Tuareg.pSensors->dsensors) != cRunSensorTrigger_on_high) ? true : false;

    //shut engine off if the CRASH sensor is engaged
    Tuareg.Halt_source.crash_sensor= (getBit_BF8(DSENSOR_CRASH, Tuareg.pSensors->dsensors) == cCrashSensorTrigger_on_high) ? true : false;

    //shut engine off if the SIDESTAND sensor is engaged AND a gear has been selected
    Tuareg.Halt_source.sidestand_sensor= ((getBit_BF8(DSENSOR_SIDESTAND, Tuareg.pSensors->dsensors) == cSidestandSensorTrigger_on_high)  && (Tuareg.process.Gear != GEAR_NEUTRAL)) ? true : false;
}




/******************************************************************************************************************************
triggers run mode transitions based on periodically read input signals
-> periodic (time triggered) update
******************************************************************************************************************************/
void Tuareg_update_Runmode()
{
    //collect diagnostic information
    tuareg_diag_log_event(TDIAG_UPDATE_RUNMODE_CALLS);

    //process the digital sensors readout for halt sources
    Tuareg_update_halt_sources();

    switch(Tuareg.Runmode)
    {

    case TMODE_CRANKING:

        //check if any of the halt sources has been triggered
        if(Tuareg.Halt_source.all_flags > 0)
        {
            Syslog_Info(TID_TUAREG, TUAREG_LOC_UPDATE_RUNMODE_HALTSRC_PRESENT);
            tuareg_diag_log_event(TDIAG_HALTSRC_PRESENT);

            #ifdef TUAREG_DEBUG_OUTPUT
            DebugMsg_Warning("Haltsrc present, going to HALT mode!");
            #endif // TUAREG_DEBUG_OUTPUT

            Tuareg_set_Runmode(TMODE_HALT);
        }
        else if((Tuareg.pDecoder->outputs.rpm_valid == true) && (Tuareg.pDecoder->crank_rpm > Ignition_Setup.dynamic_min_rpm))
        {
            /// engine has finished cranking
            Syslog_Info(TID_TUAREG, TUAREG_LOC_UPDATE_RUNMODE_CRANKING_END);

            Tuareg_set_Runmode(TMODE_RUNNING);

            Tuareg_notify_fueling_cranking_end();
        }

        break;


    case TMODE_RUNNING:
        case TMODE_STB:

        //check if any of the halt sources has been triggered
        if(Tuareg.Halt_source.all_flags > 0)
        {
            Syslog_Info(TID_TUAREG, TUAREG_LOC_UPDATE_RUNMODE_HALTSRC_PRESENT);
            tuareg_diag_log_event(TDIAG_HALTSRC_PRESENT);

            #ifdef TUAREG_DEBUG_OUTPUT
            DebugMsg_Warning("Haltsrc present, going to HALT mode!");
            #endif // TUAREG_DEBUG_OUTPUT

            Tuareg_set_Runmode(TMODE_HALT);
        }

        break;


    case TMODE_HALT:

        //allow turning engine on if the RUN switch is engaged AND the CRASH sensor disengaged
        if(Tuareg.Halt_source.all_flags == 0)
        {
            Syslog_Info(TID_TUAREG, TUAREG_LOC_UPDATE_RUNMODE_HALTSRC_CLEARED);
            tuareg_diag_log_event(TDIAG_HALTSRC_CLEAR);

            Tuareg_set_Runmode(TMODE_STB);
        }

        break;


    case TMODE_LIMP:

        /**
        LIMP mode will persist until reboot
        LIMP mode will ignore all HALT sources BUT run switch
        */
        if(Tuareg.Halt_source.run_switch == true)
        {
            Syslog_Info(TID_TUAREG, TUAREG_LOC_UPDATE_RUNMODE_LIMP_RUNSWITCH_RELEASED);
            tuareg_diag_log_event(TDIAG_HALTSRC_PRESENT);

            Tuareg_stop_engine();
        }

        break;


    default:

        ///not every run mode requires updates
        break;

    }
}



/******************************************************************************************************************************
set Runmode manually and trigger transition helper functions
******************************************************************************************************************************/
void Tuareg_set_Runmode(volatile tuareg_runmode_t Target_runmode)
{
    if(Tuareg.Runmode == Target_runmode)
    {
        return;
    }

    //LIMP mode protection
    if((Tuareg.Runmode == TMODE_LIMP) && (Target_runmode != TMODE_FATAL))
    {
        Syslog_Warning(TID_TUAREG, TUAREG_LOC_SET_RUNMODE_LIMP_EXIT);

        #ifdef TUAREG_DEBUG_OUTPUT
        DebugMsg_Warning("trapped in LIMP mode");
        #endif // TUAREG_DEBUG_OUTPUT

        return;
    }

    //FATAL mode protection
    if(Tuareg.Runmode == TMODE_FATAL)
    {
        Syslog_Warning(TID_TUAREG, TUAREG_LOC_SET_RUNMODE_FATAL_EXIT);

        #ifdef TUAREG_DEBUG_OUTPUT
        DebugMsg_Warning("trapped in FATAL mode");
        #endif // TUAREG_DEBUG_OUTPUT

        return;
    }

    /**
    run state transition actions
    */
    switch(Target_runmode)
    {

    case TMODE_LIMP:

        Tuareg_LIMP_transition();
        break;

    case TMODE_SERVICE:

        Tuareg_SERVICE_transition();
        break;

    case TMODE_HALT:

        Tuareg_HALT_transition();
        break;

    case TMODE_RUNNING:

        Tuareg_RUNNING_transition();
        break;

    case TMODE_STB:

        Tuareg_STB_transition();
        break;

    case TMODE_CRANKING:

        Tuareg_CRANKING_transition();
        break;


    case TMODE_FATAL:

        Tuareg_FATAL_transition();
        break;


    default:

        Fatal(TID_TUAREG, TUAREG_LOC_SET_RUNMODE_DEFAULT_BRANCH);
        break;

    }

    //finally set the new mode
    Tuareg.Runmode= Target_runmode;

}





/******************************************************************************************************************************
helper functions - Runmode transitions - HALT, RUNNING, CRANKING, STB
******************************************************************************************************************************/

void Tuareg_HALT_transition()
{
    //turn off vital actors
    Tuareg_stop_engine();

    //collect diagnostic information
    tuareg_diag_log_event(TDIAG_ENTER_HALT);
}


void Tuareg_RUNNING_transition()
{
    /**
    begin normal engine operation
    */

    Tuareg.actors.ignition_inhibit= false;
    Tuareg.actors.fueling_inhibit= false;

    //collect diagnostic information
    if(Tuareg.Runmode == TMODE_CRANKING)
    {
        tuareg_diag_log_event(TDIAG_CRANKING_RUNNING_TR);
    }

    tuareg_diag_log_event(TDIAG_ENTER_RUNNING);
}

void Tuareg_STB_transition()
{
    //engine stalled, turn off vital actors, set inhibit state
    Tuareg_stop_engine();

    //collect diagnostic information
    tuareg_diag_log_event(TDIAG_ENTER_STB);
}


void Tuareg_CRANKING_transition()
{
    Tuareg.actors.ignition_inhibit= false;
    Tuareg.actors.fueling_inhibit= false;

    /// TODO (oli#9#): check if we need fuel pump priming
    //turn fuel pump on to maintain fuel pressure
    set_fuel_pump_powered();
}

/******************************************************************************************************************************
helper functions - Runmode transitions - LIMP
******************************************************************************************************************************/

void Tuareg_LIMP_transition()
{

}

/******************************************************************************************************************************
helper functions - Runmode transitions - SERVICE
******************************************************************************************************************************/

void Tuareg_SERVICE_transition()
{
    Syslog_Info(TID_TUAREG, TUAREG_LOC_SET_RUNMODE_SERVICE);

    #ifdef TUAREG_DEBUG_OUTPUT
    DebugMsg_Warning("Entering Service Mode");
    #endif // TUAREG_DEBUG_OUTPUT

    //allow vital actor activation
    Tuareg.actors.ignition_inhibit= false;
    Tuareg.actors.fueling_inhibit= false;
}

/******************************************************************************************************************************
helper functions - Runmode transitions - FATAL
******************************************************************************************************************************/

void Tuareg_FATAL_transition()
{
    //engine stalled, turn off vital actors, set inhibit state
    Tuareg_stop_engine();

    Syslog_Error(TID_TUAREG, TUAREG_LOC_FATAL_ERROR);

    /// TODO (oli#3#): implement FATAL mode with only defensive debug printouts enabled

    //remove config mod / burn permissions


}
