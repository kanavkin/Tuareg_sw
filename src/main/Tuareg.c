#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_adc.h"
#include "stm32_libs/boctok_types.h"

#include "base_calc.h"

#include "Tuareg_decoder.h"

#include "Tuareg_ignition.h"
#include "ignition_hw.h"
#include "ignition_config.h"

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
#include "fuel_hw.h"
#include "fuel_logic.h"

#include "dash_hw.h"
#include "dash_logic.h"
#include "act_hw.h"
#include "act_logic.h"

#include "process_table.h"

//#include "debug.h"
#include "diagnostics.h"
#include "Tuareg.h"
#include "uart_printf.h"
#include "module_test.h"


void Tuareg_print_init_message()
{
    #ifdef TUAREG_MODULE_TEST
    moduletest_initmsg_action();
    #else

    print(DEBUG_PORT, "\r \n \r \n . \r \n . \r \n . \r \n \r \n *** This is Tuareg, lord of the Sahara *** \r \n");
    print(DEBUG_PORT, "V 0.2");
    #endif
}




void Tuareg_update_Runmode()
{
    /**
    calculate new system state
    */

    //collect diagnostic information
    tuareg_diag_log_event(TDIAG_MODECTRL);

    Tuareg_update_halt_sources();

    switch(Tuareg.Runmode)
    {

    case TMODE_DIAG:

        ///DIAG mode will persist until reboot
        break;


    case TMODE_CRANKING:

        // any halt source triggered?
        if(Tuareg.Halt_source.all_flags)
        {
            Tuareg_set_Runmode(TMODE_HALT);
        }
        else if((Tuareg.pDecoder->outputs.rpm_valid == true) && (Tuareg.pDecoder->crank_rpm > Ignition_Setup.dynamic_min_rpm))
        {
            /// engine has finished cranking
            Tuareg_set_Runmode(TMODE_RUNNING);
        }

        break;


    case TMODE_RUNNING:
        case TMODE_STB:

        // any halt source triggered?
        if(Tuareg.Halt_source.all_flags)
        {
            Tuareg_set_Runmode(TMODE_HALT);
        }

        break;


    case TMODE_HALT:

        //allow turning engine on if the RUN switch is engaged AND the CRASH sensor disengaged
        if(Tuareg.Halt_source.all_flags == 0)
        {
            Tuareg_set_Runmode(TMODE_STB);

            //collect diagnostic information
            tuareg_diag_log_event(TDIAG_HALT_STB_TR);
        }

        break;

    case TMODE_LIMP:

        /**
        LIMP mode will persist until reboot
        LIMP mode will ignore all HALT sources BUT run switch
        */
        if(Tuareg.Halt_source.run_switch)
        {
            Tuareg_stop_engine();
        }

        break;

    default:
        //very strange
        print(DEBUG_PORT, "ERROR! default branch in TMODE machine reached");
        Tuareg_stop_engine();
        break;

    }
}


void Tuareg_set_Runmode(volatile tuareg_runmode_t Target_runmode)
{
    exec_result_t result;

    if(Tuareg.Runmode != Target_runmode)
    {
        /**
        LIMP mode protection
        */
        if(Tuareg.Runmode == TMODE_LIMP)
        {
            print(DEBUG_PORT, "cannot exit LIMP mode!");
            return;
        }

        /**
        run state transition actions
        */
        switch(Target_runmode)
        {
            case TMODE_HWINIT:

                /**
                system init
                */

                //collect diagnostic information
                tuareg_diag_log_event(TDIAG_ENTER_INIT);

                //use 16 preemption priority levels
                NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

                //set all errors
                Tuareg.Errors.all_flags= 0xFFFFFFFF;

                /**
                initialize core components
                */

                //initialize systick timer to provide system timestamp
                Tuareg.pTimer= init_systick_timer();

                Eeprom_init();
                //init_fuel_hw();
                //init_dash_hw();
                //init_act_hw();


                UART_DEBUG_PORT_Init();
                Tuareg_print_init_message();
                break;

            case TMODE_CONFIGLOAD:

                Tuareg.Errors.tuareg_config_error= true;

                //loading the config data is essential, failure forces "limp home mode"
                result= load_Tuareg_Setup();

                //check if config has been loaded
                if(result != EXEC_OK)
                {
                    load_essential_Tuareg_Setup();

                    //failed to load config
                    print(DEBUG_PORT, "\r\nEE Failed to load Tuareg config!");
                    print(DEBUG_PORT, "\r\nWW Tuareg essential Config has been loaded");
                }
                else if(Tuareg_Setup.Version != TUAREG_REQUIRED_CONFIG_VERSION)
                {
                    load_essential_Tuareg_Setup();

                    //loaded wrong config version
                    print(DEBUG_PORT, "\r\nEE Tuareg config version does not match");
                    print(DEBUG_PORT, "\r\nWW Tuareg essential Config has been loaded");
                }
                else
                {
                    //loaded config with correct version
                    Tuareg.Errors.tuareg_config_error= false;
                    print(DEBUG_PORT, "\r\nII Tuareg config has been loaded");
                }

                break;

            case TMODE_MODULEINIT:

                /**
                initialize core components and register interface access pointers
                */
                Tuareg.pDecoder= init_Decoder();

                Tuareg.pSensors= init_Sensors();

                init_Ignition();

                init_scheduler();

                init_lowprio_scheduler();
                init_fuel_logic();
                init_dash_logic();
                init_act_logic();

                Tuareg_init_console();

                Tuareg.actors.ignition_inhibit= true;
                Tuareg.actors.fueling_inhibit= true;

                break;

            case TMODE_LIMP:

                //control engine with minimum sensor input available

                break;

            case TMODE_DIAG:

                //perform diagnostic functions triggered by user, no engine operation

                //turn off vital actors
                Tuareg_stop_engine();

                break;

            case TMODE_HALT:

                //turn off vital actors
                Tuareg_stop_engine();

                //collect diagnostic information
                tuareg_diag_log_event(TDIAG_ENTER_HALT);
                break;

            case TMODE_RUNNING:

                /**
                begin normal engine operation
                */

                Tuareg.actors.ignition_inhibit= false;
                Tuareg.actors.fueling_inhibit= false;

                //collect diagnostic information
                if(Tuareg.Runmode == TMODE_CRANKING)
                {
                    tuareg_diag_log_event(TDIAG_CRANKING_RUNNING_TR);

                    /// TODO (oli#9#): debug message enabled
                    print(DEBUG_PORT, "\r\nstate transition TMODE_CRANKING --> TMODE_RUNNING");
                }

                tuareg_diag_log_event(TDIAG_ENTER_RUNNING);
                break;


            case TMODE_STB:

                //engine stalled, turn off vital actors, set inhibit state
                Tuareg_stop_engine();

                //collect diagnostic information
                tuareg_diag_log_event(TDIAG_ENTER_STB);
                break;

            case TMODE_CRANKING:

                Tuareg.actors.ignition_inhibit= false;
                Tuareg.actors.fueling_inhibit= false;

                /// TODO (oli#9#): check if we need fuel pump priming
                //turn fuel pump on to maintain fuel pressure
                set_fuelpump(PIN_ON);

                /// TODO (oli#9#): debug message enabled
                print(DEBUG_PORT, "\r\nstate transition TMODE_CRANKING");

                break;


            default:

                //collect diagnostic information
                tuareg_diag_log_event(TDIAG_INVALID_RUNMODE);

                //very strange
                print(DEBUG_PORT, "ERROR! default branch in Tuareg_set_Runmode reached");
                Tuareg_set_Runmode(TMODE_LIMP);
                break;

        }

        //finally set the new mode
        Tuareg.Runmode= Target_runmode;

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
    set_fuelpump(ACTOR_UNPOWERED);
    set_injector_ch1(ACTOR_UNPOWERED);
    set_injector_ch2(ACTOR_UNPOWERED);

    //reset scheduler
    scheduler_reset_ign1();
    scheduler_reset_ign2();

    //reset MAP calculation
    reset_asensor_sync_integrator(ASENSOR_SYNC_MAP);
}



/**
checks if the RUN switch, SIDESTAND sensor or CRASH sensor indicate a HALT condition
*/
void Tuareg_update_halt_sources()
{
    //shut engine off if the RUN switch is disengaged
    if( (Tuareg.pSensors->dsensors & (1<< DSENSOR_RUN)) != RUN_SENSOR_ENGAGE_LEVEL )
    {
        Tuareg.Halt_source.run_switch= true;

        //collect diagnostic information
        tuareg_diag_log_event(TDIAG_KILL_RUNSWITCH);

        /// TODO (oli#9#): debug message enabled
        print(DEBUG_PORT, "\r\nHALT! reason: DSENSOR_RUN");
    }
    else
    {
        Tuareg.Halt_source.run_switch= false;
    }

    //shut engine off if the CRASH sensor is engaged
    if( (Tuareg.pSensors->dsensors & (1<< DSENSOR_CRASH)) == CRASH_SENSOR_ENGAGE_LEVEL)
    {
        Tuareg.Halt_source.crash_sensor= true;

        //collect diagnostic information
        tuareg_diag_log_event(TDIAG_KILL_CRASH);

        /// TODO (oli#9#): debug message enabled
        print(DEBUG_PORT, "\r\nHALT! reason: DSENSOR_CRASH");
    }
    else
    {
        Tuareg.Halt_source.crash_sensor= false;
    }

    //shut engine off if the SIDESTAND sensor is engaged AND a gear has been selected
    if(( (Tuareg.pSensors->dsensors & (1<< DSENSOR_CRASH)) == CRASH_SENSOR_ENGAGE_LEVEL) && (Tuareg.process.Gear != GEAR_NEUTRAL) )
    {
        Tuareg.Halt_source.sidestand_sensor= true;

        //collect diagnostic information
        tuareg_diag_log_event(TDIAG_KILL_SIDESTAND);

        /// TODO (oli#9#): debug message enabled
        print(DEBUG_PORT, "\r\nHALT! reason: DSENSOR_SIDESTAND");
    }
    else
    {
        Tuareg.Halt_source.sidestand_sensor= false;
    }

}




