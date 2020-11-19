#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_adc.h"
#include "stm32_libs/boctok_types.h"

#include "base_calc.h"
#include "decoder_hw.h"
#include "decoder_logic.h"
#include "ignition_logic.h"
#include "ignition_hw.h"
#include "scheduler.h"
#include "lowprio_scheduler.h"
#include "uart.h"
#include "conversion.h"
#include "lowspeed_timers.h"
#include "TunerStudio.h"
#include "config.h"
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

#include "debug.h"
#include "diagnostics.h"
#include "Tuareg.h"

#include "module_test.h"


void Tuareg_print_init_message()
{
    #ifdef TUAREG_MODULE_TEST
    moduletest_initmsg_action();
    #else

    UART_Send(DEBUG_PORT, "\r \n \r \n . \r \n . \r \n . \r \n \r \n *** This is Tuareg, lord of the Sahara *** \r \n");
    UART_Send(DEBUG_PORT, "V 0.2");
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
        else if(Tuareg.process.crank_rpm > configPage13.dynamic_min_rpm)
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
        UART_Send(DEBUG_PORT, "ERROR! default branch in TMODE machine reached");
        Tuareg_stop_engine();
        break;

    }
}


void Tuareg_register_scheduler_error()
{
    Tuareg.Errors.scheduler_error= true;

    UART_Send(DEBUG_PORT, "\r\n*** Registered Scheduler error ***");
}


void Tuareg_set_Runmode(volatile tuareg_runmode_t Target_runmode)
{

    if(Tuareg.Runmode != Target_runmode)
    {
        /**
        LIMP mode protection
        */
        if(Target_runmode == TMODE_LIMP)
        {
            UART_Send(DEBUG_PORT, "cannot exit LIMP mode!");
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

                //set basic config error states
                Tuareg.Errors.config_load_error= true;

                /**
                initialize core components
                */
                init_decoder_hw();
                init_ignition_hw();
                init_fuel_hw();
                init_dash_hw();
                init_act_hw();
                init_eeprom();

                UART_DEBUG_PORT_Init();
                UART_TS_PORT_Init();

                Tuareg_print_init_message();
                break;

            case TMODE_CONFIGLOAD:

                //loading the config data is important to us, failure in loading forces "limp home mode"
                Tuareg.Errors.config_load_error= config_load();

                /**
                /// TODO (oli#1#): DEBUG: limp home test
                config_load_status= RETURN_FAIL;
                */

                if(Tuareg.Errors.config_load_error)
                {
                    //as we can hardly save some error logs in this system condition, print some debug messages only
                    UART_Send(DEBUG_PORT, "\r \n *** FAILED to load config data !");

                    //provide default values to ensure limp home operation even without eeprom
                    config_load_essentials();
                }
                else
                {
                    /**
                    prepare config data
                    */

                    //set 2D table dimension and link table data to config pages
                    init_2Dtables();
                }

                break;

            case TMODE_MODULEINIT:

                #ifdef TUAREG_MODULE_TEST
                moduletest_moduleinit_action();
                #else

                //set basic sensor error states
                Tuareg.Errors.sensor_BARO_error= true;
                Tuareg.Errors.sensor_MAP_error= true;
                Tuareg.Errors.sensor_O2_error= true;
                Tuareg.Errors.sensor_TPS_error= true;
                Tuareg.Errors.sensor_IAT_error= true;
                Tuareg.Errors.sensor_CLT_error= true;
                Tuareg.Errors.sensor_VBAT_error= true;
                Tuareg.Errors.sensor_KNOCK_error= true;
                Tuareg.Errors.sensor_GEAR_error= true;
                Tuareg.Errors.sensor_CIS_error= true;

                /**
                initialize core components and register interface access pointers
                */
                Tuareg.sensors= init_sensors();
                Tuareg.decoder= init_decoder_logic();
                init_scheduler();
                init_lowspeed_timers();
                init_lowprio_scheduler();
                init_fuel_logic();
                init_dash_logic();
                init_act_logic();

                #endif
                break;

            case TMODE_LIMP:

                //control engine with minimum sensor input available

                break;

            case TMODE_DIAG:

                //perform diagnostic functions triggered by user, no engine operation
                break;

            case TMODE_HALT:

                //engine operation prohibited due to kill switch or crash sensor
                Tuareg_stop_engine();

                //collect diagnostic information
                if(Tuareg.Runmode == TMODE_RUNNING)
                {
                    tuareg_diag_log_event(TDIAG_RUNNING_HALT_TR);

                     /// TODO (oli#9#): debug message enabled
                    UART_Send(DEBUG_PORT, "\r\nstate transition TMODE_RUNNING --> TMODE_HALT");
                }
                else if(Tuareg.Runmode == TMODE_HWINIT)
                {
                    tuareg_diag_log_event(TDIAG_INIT_HALT_TR);

                     /// TODO (oli#9#): debug message enabled
                    UART_Send(DEBUG_PORT, "\r\nstate transition TMODE_HWINIT --> TMODE_HALT");
                }
                else if(Tuareg.Runmode == TMODE_STB)
                {
                    tuareg_diag_log_event(TDIAG_STB_HALT_TR);

                     /// TODO (oli#9#): debug message enabled
                    UART_Send(DEBUG_PORT, "\r\nstate transition TMODE_STB --> TMODE_HALT");
                }


                //collect diagnostic information
                tuareg_diag_log_event(TDIAG_ENTER_HALT);
                break;

            case TMODE_RUNNING:

                /**
                begin normal engine operation
                */

                //collect diagnostic information
                if(Tuareg.Runmode == TMODE_CRANKING)
                {
                    tuareg_diag_log_event(TDIAG_CRANKING_RUNNING_TR);

                    /// TODO (oli#9#): debug message enabled
                    UART_Send(DEBUG_PORT, "\r\nstate transition TMODE_CRANKING --> TMODE_RUNNING");
                }

                tuareg_diag_log_event(TDIAG_ENTER_RUNNING);
                break;

            case TMODE_STB:

                /// TODO (oli#9#): check if we need fuel pump priming

                //engine stalled, system ready for start
                Tuareg_stop_engine();

                //provide ignition timing for engine startup
                cranking_ignition_controls(&(Tuareg.ignition_controls));

                //collect diagnostic information
                if(Tuareg.Runmode == TMODE_RUNNING)
                {
                    tuareg_diag_log_event(TDIAG_RUNNING_STB_TR);

                    /// TODO (oli#9#): debug message enabled
                    UART_Send(DEBUG_PORT, "\r\nstate transition TMODE_RUNNING --> TMODE_STB");
                }
                else if(Tuareg.Runmode == TMODE_HALT)
                {
                    tuareg_diag_log_event(TDIAG_HALT_STB_TR);

                    /// TODO (oli#9#): debug message enabled
                    UART_Send(DEBUG_PORT, "\r\nstate transition TMODE_HALT --> TMODE_STB");
                }
                else if(Tuareg.Runmode == TMODE_CRANKING)
                {
                    tuareg_diag_log_event(TDIAG_CRANKING_STB_TR);

                    /// TODO (oli#9#): debug message enabled
                    UART_Send(DEBUG_PORT, "\r\nstate transition TMODE_CRANKING --> TMODE_STB");
                }

                tuareg_diag_log_event(TDIAG_ENTER_STB);
                break;

            case TMODE_CRANKING:

                /// TODO (oli#9#): check if we need fuel pump priming
                //turn fuel pump on to maintain fuel pressure
                set_fuelpump(PIN_ON);

                //provide initial ignition timing for cranking
                default_ignition_controls(&(Tuareg.ignition_controls));

                //collect diagnostic information
                tuareg_diag_log_event(TDIAG_ENTER_CRANKING);

                /// TODO (oli#9#): debug message enabled
                UART_Send(DEBUG_PORT, "\r\nstate transition TMODE_CRANKING");

                break;

            case TMODE_MODULE_TEST:

                Tuareg_stop_engine();

                //collect diagnostic information
                tuareg_diag_log_event(TDIAG_ENTER_MTEST);

                /// TODO (oli#9#): debug message enabled
                UART_Send(DEBUG_PORT, "\r\nstate transition TMODE_MODULE_TEST");

                break;


            default:

                //collect diagnostic information
                tuareg_diag_log_event(TDIAG_INVALID_RUNMODE);

                //very strange
                UART_Send(DEBUG_PORT, "ERROR! default branch in Tuareg_set_Runmode reached");
                Tuareg_set_Runmode(TMODE_LIMP);
                break;

        }

        //finally set the new mode
        Tuareg.Runmode= Target_runmode;

    }

}





void Tuareg_stop_engine()
{
    //turn off vital engine actors
    set_fuelpump(ACTOR_UNPOWERED);
    set_injector_ch1(ACTOR_UNPOWERED);
    set_injector_ch2(ACTOR_UNPOWERED);
    set_ignition_ch1(ACTOR_UNPOWERED);
    set_ignition_ch2(ACTOR_UNPOWERED);

    scheduler_reset_channel(SCHEDULER_CH_IGN1);
    scheduler_reset_channel(SCHEDULER_CH_IGN2);
    scheduler_reset_channel(SCHEDULER_CH_FUEL1);
    scheduler_reset_channel(SCHEDULER_CH_FUEL2);

    //reset MAP calculation
    reset_asensor_sync_integrator(ASENSOR_SYNC_MAP);
}


/**
update engine control strategy based on available sensors and
*/
void Tuareg_update_process_data(process_data_t * pImage)
{
    volatile crank_position_table_t crank_table;

    //collect diagnostic information
    tuareg_diag_log_event(TDIAG_PROCESSDATA_CALLS);

    /**
    decoder based process data
    */

    //crank_T_us
    pImage->crank_T_us= Tuareg.decoder->crank_period_us;

    //engine_rpm
    pImage->crank_rpm= calc_rpm(pImage->crank_T_us);
    pImage->ddt_crank_rpms= 0;

    //crank position table
    update_crank_position_table(&crank_table);

    //update process table with data supplied by decoder
    update_process_table(&crank_table);

    //analog sensors
    pImage->MAP_kPa= Tuareg_update_MAP_sensor();
    pImage->Baro_kPa= Tuareg_update_BARO_sensor();
    pImage->TPS_deg= Tuareg_update_TPS_sensor();
    pImage->IAT_K= Tuareg_update_IAT_sensor();
    pImage->CLT_K= Tuareg_update_CLT_sensor();
    pImage->VBAT_V= Tuareg_update_VBAT_sensor();
    pImage->ddt_TPS= Tuareg_update_ddt_TPS();
    pImage->O2_AFR= Tuareg_update_O2_sensor();
    pImage->Gear= Tuareg_update_GEAR_sensor();

    // strategy?

    //delay
    //pImage->data_age= decoder_get_data_age_us();


}








/**
checks the health state of the MAP sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_MAP_sensor()
{
    //if sensor has already been validated and is still healthy
    if( (Tuareg.Errors.sensor_MAP_error == false) && (Tuareg.sensors->asensors_health & (1<< ASENSOR_MAP)) )
    {
        //use live value
        return Tuareg.sensors->asensors[ASENSOR_MAP];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.sensors->asensors_health & (1<< ASENSOR_MAP)) && (Tuareg.sensors->asensors_valid_samples[ASENSOR_MAP] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_MAP_error= false;

            //use live value
            return Tuareg.sensors->asensors[ASENSOR_MAP];
        }
        else
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_MAP_error= true;

            //use default value
            return MAP_DEFAULT_KPA;
        }

    }
}

/**
checks the health state of the O2 sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_O2_sensor()
{
    //if sensor has already been validated and is still healthy
    if( (Tuareg.Errors.sensor_O2_error == false) && (Tuareg.sensors->asensors_health & (1<< ASENSOR_O2)) )
    {
        //use live value
        return Tuareg.sensors->asensors[ASENSOR_O2];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.sensors->asensors_health & (1<< ASENSOR_O2)) && (Tuareg.sensors->asensors_valid_samples[ASENSOR_O2] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_O2_error= false;

            //use live value
            return Tuareg.sensors->asensors[ASENSOR_O2];
        }
        else
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_O2_error= true;

            //use default value
            return O2_DEFAULT_AFR;
        }

    }
}

/**
checks the health state of the TPS sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_TPS_sensor()
{
    //if sensor has already been validated and is still healthy
    if( (Tuareg.Errors.sensor_TPS_error == false) && (Tuareg.sensors->asensors_health & (1<< ASENSOR_TPS)) )
    {
        //use live value
        return Tuareg.sensors->asensors[ASENSOR_TPS];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.sensors->asensors_health & (1<< ASENSOR_TPS)) && (Tuareg.sensors->asensors_valid_samples[ASENSOR_TPS] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_TPS_error= false;

            //use live value
            return Tuareg.sensors->asensors[ASENSOR_TPS];
        }
        else
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_TPS_error= true;

            //use default value
            return TPS_DEFAULT_DEG;
        }

    }
}

VF32 Tuareg_update_ddt_TPS()
{
    /// must be executed after TPS sensor update!
    if(Tuareg.Errors.sensor_TPS_error == false)
    {
        //use live value
        return Tuareg.sensors->ddt_TPS;
    }
    else
    {
        return 0;
    }

}

/**
checks the health state of the IAT sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_IAT_sensor()
{
    //if sensor has already been validated and is still healthy
    if( (Tuareg.Errors.sensor_IAT_error == false) && (Tuareg.sensors->asensors_health & (1<< ASENSOR_IAT)) )
    {
        //use live value
        return Tuareg.sensors->asensors[ASENSOR_IAT];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.sensors->asensors_health & (1<< ASENSOR_IAT)) && (Tuareg.sensors->asensors_valid_samples[ASENSOR_IAT] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_IAT_error= false;

            //use live value
            return Tuareg.sensors->asensors[ASENSOR_IAT];
        }
        else
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_IAT_error= true;

            //use default value
            return IAT_DEFAULT_C;
        }

    }
}

/**
checks the health state of the CLT sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_CLT_sensor()
{
    //if sensor has already been validated and is still healthy
    if( (Tuareg.Errors.sensor_CLT_error == false) && (Tuareg.sensors->asensors_health & (1<< ASENSOR_CLT)) )
    {
        //use live value
        return Tuareg.sensors->asensors[ASENSOR_CLT];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.sensors->asensors_health & (1<< ASENSOR_CLT)) && (Tuareg.sensors->asensors_valid_samples[ASENSOR_CLT] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_CLT_error= false;

            //use live value
            return Tuareg.sensors->asensors[ASENSOR_CLT];
        }
        else
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_CLT_error= true;

            //use default value
            return CLT_DEFAULT_C;
        }

    }
}

/**
checks the health state of the VBAT sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_VBAT_sensor()
{
    //if sensor has already been validated and is still healthy
    if( (Tuareg.Errors.sensor_VBAT_error == false) && (Tuareg.sensors->asensors_health & (1<< ASENSOR_VBAT)) )
    {
        //use live value
        return Tuareg.sensors->asensors[ASENSOR_VBAT];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.sensors->asensors_health & (1<< ASENSOR_VBAT)) && (Tuareg.sensors->asensors_valid_samples[ASENSOR_VBAT] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_VBAT_error= false;

            //use live value
            return Tuareg.sensors->asensors[ASENSOR_VBAT];
        }
        else
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_VBAT_error= true;

            //use default value
            return VBAT_DEFAULT_V;
        }

    }
}


/**
checks the health state of the KNOCK sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_KNOCK_sensor()
{
    //if sensor has already been validated and is still healthy
    if( (Tuareg.Errors.sensor_KNOCK_error == false) && (Tuareg.sensors->asensors_health & (1<< ASENSOR_KNOCK)) )
    {
        //use live value
        return Tuareg.sensors->asensors[ASENSOR_KNOCK];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.sensors->asensors_health & (1<< ASENSOR_KNOCK)) && (Tuareg.sensors->asensors_valid_samples[ASENSOR_KNOCK] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_KNOCK_error= false;

            //use live value
            return Tuareg.sensors->asensors[ASENSOR_KNOCK];
        }
        else
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_KNOCK_error= true;

            //use default value
            return KNOCK_DEFAULT;
        }

    }
}

/**
checks the health state of the BARO sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_BARO_sensor()
{
    //if sensor has already been validated and is still healthy
    if( (Tuareg.Errors.sensor_BARO_error == false) && (Tuareg.sensors->asensors_health & (1<< ASENSOR_BARO)) )
    {
        //use live value
        return Tuareg.sensors->asensors[ASENSOR_BARO];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.sensors->asensors_health & (1<< ASENSOR_BARO)) && (Tuareg.sensors->asensors_valid_samples[ASENSOR_BARO] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_BARO_error= false;

            //use live value
            return Tuareg.sensors->asensors[ASENSOR_BARO];
        }
        else
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_BARO_error= true;

            //use default value
            return BARO_DEFAULT_KPA;
        }

    }
}

/**
checks the health state of the GEAR sensor
if more than ASENSOR_VALIDITY_THRES consecutive, valid samples have been read from this sensor, it is considered valid
uses a generic default value for disturbed sensors
*/
VF32 Tuareg_update_GEAR_sensor()
{
    //if sensor has already been validated and is still healthy
    if( (Tuareg.Errors.sensor_GEAR_error == false) && (Tuareg.sensors->asensors_health & (1<< ASENSOR_GEAR)) )
    {
        //use live value
        return Tuareg.sensors->asensors[ASENSOR_GEAR];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.sensors->asensors_health & (1<< ASENSOR_GEAR)) && (Tuareg.sensors->asensors_valid_samples[ASENSOR_GEAR] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.Errors.sensor_GEAR_error= false;

            //use live value
            return Tuareg.sensors->asensors[ASENSOR_GEAR];
        }
        else
        {
            //sensor temporarily disturbed
            Tuareg.Errors.sensor_GEAR_error= true;

            //use default value
            return GEAR_DEFAULT;
        }

    }
}


/**
checks if the RUN switch, SIDESTAND sensor or CRASH sensor indicate a HALT condition
*/
void Tuareg_update_halt_sources()
{
    //shut engine off if the RUN switch is disengaged
    if( (Tuareg.sensors->dsensors & (1<< DSENSOR_RUN)) != RUN_SENSOR_ENGAGE_LEVEL )
    {
        Tuareg.Halt_source.run_switch= true;

        //collect diagnostic information
        tuareg_diag_log_event(TDIAG_KILL_RUNSWITCH);

        /// TODO (oli#9#): debug message enabled
        UART_Send(DEBUG_PORT, "\r\nHALT! reason: DSENSOR_RUN");
    }
    else
    {
        Tuareg.Halt_source.run_switch= false;
    }

    //shut engine off if the CRASH sensor is engaged
    if( (Tuareg.sensors->dsensors & (1<< DSENSOR_CRASH)) == CRASH_SENSOR_ENGAGE_LEVEL)
    {
        Tuareg.Halt_source.crash_sensor= true;

        //collect diagnostic information
        tuareg_diag_log_event(TDIAG_KILL_SIDESTAND);

        /// TODO (oli#9#): debug message enabled
        UART_Send(DEBUG_PORT, "\r\nHALT! reason: DSENSOR_CRASH");
    }
    else
    {
        Tuareg.Halt_source.crash_sensor= false;
    }

}



