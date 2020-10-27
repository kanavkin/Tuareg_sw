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

    switch(Tuareg.Runmode)
    {
    case TMODE_DIAG:

        //DIAG mode will persist until reboot
        break;


    case TMODE_CRANKING:

        //shut engine off if the RUN switch is disengaged
        if( (Tuareg.sensors->dsensors & (1<< DSENSOR_RUN)) != RUN_SENSOR_ENGAGE_LEVEL )
        {
            Tuareg_set_Runmode(TMODE_HALT);

            //collect diagnostic information
            tuareg_diag_log_event(TDIAG_KILL_RUNSWITCH);

            /// TODO (oli#9#): debug message enabled
            UART_Send(DEBUG_PORT, "\r\nHALT! reason: DSENSOR_RUN");
        }

        //shut engine off if the CRASH sensor is engaged
        if( (Tuareg.sensors->dsensors & (1<< DSENSOR_CRASH)) == CRASH_SENSOR_ENGAGE_LEVEL)
        {
            Tuareg_set_Runmode(TMODE_HALT);

            //collect diagnostic information
            tuareg_diag_log_event(TDIAG_KILL_SIDESTAND);

            /// TODO (oli#9#): debug message enabled
            UART_Send(DEBUG_PORT, "\r\nHALT! reason: DSENSOR_CRASH");
        }

        /// TODO (oli#3#): implement cranking handling
        if((Tuareg.process.engine_rpm > configPage13.dynamic_min_rpm) && (Tuareg.process.crank_position != CRK_POSITION_UNDEFINED))
        {
            /**
            engine has finished cranking
            */

            Tuareg_set_Runmode(TMODE_RUNNING);
        }


    case TMODE_RUNNING:
        case TMODE_STB:

        //shut engine off if the RUN switch is disengaged
        if( (Tuareg.sensors->dsensors & (1<< DSENSOR_RUN)) != RUN_SENSOR_ENGAGE_LEVEL )
        {
            Tuareg_set_Runmode(TMODE_HALT);

            //collect diagnostic information
            tuareg_diag_log_event(TDIAG_KILL_RUNSWITCH);

            /// TODO (oli#9#): debug message enabled
            UART_Send(DEBUG_PORT, "\r\nHALT! reason: DSENSOR_RUN");
        }

        //shut engine off if the CRASH sensor is engaged
        if( (Tuareg.sensors->dsensors & (1<< DSENSOR_CRASH)) == CRASH_SENSOR_ENGAGE_LEVEL)
        {
            Tuareg_set_Runmode(TMODE_HALT);

            //collect diagnostic information
            tuareg_diag_log_event(TDIAG_KILL_SIDESTAND);

            /// TODO (oli#9#): debug message enabled
            UART_Send(DEBUG_PORT, "\r\nHALT! reason: DSENSOR_CRASH");
        }

        break;


    case TMODE_HALT:

        //allow turning engine on if the RUN switch is engaged AND the CRASH sensor disengaged
        if( ((Tuareg.sensors->dsensors & (1<< DSENSOR_RUN)) == RUN_SENSOR_ENGAGE_LEVEL ) && ((Tuareg.sensors->dsensors & (1<< DSENSOR_CRASH)) != CRASH_SENSOR_ENGAGE_LEVEL) )
        {
            Tuareg_set_Runmode(TMODE_STB);

            //collect diagnostic information
            tuareg_diag_log_event(TDIAG_HALT_STB_TR);
        }

        break;

    case TMODE_LIMP:

        /**
        LIMP mode will ignore RUN sensor
        */

        /**
        LIMP mode will ignore CRASH sensor
        */

        break;

    default:
        //very strange
        UART_Send(DEBUG_PORT, "ERROR! default branch in TMODE machine reached");
        Tuareg_stop_engine();
        break;

    }
}


void Tuareg_register_error(tuareg_error_t Error)
{
    if(Error < TERROR_CNT)
    {
        Tuareg.Errors |= (1<< Error);
    }

    UART_Send(DEBUG_PORT, "\r\n*** Registered error ");
    UART_Print_U(DEBUG_PORT, Error, TYPE_U32, NO_PAD);
    UART_Send(DEBUG_PORT, "***");
}

void Tuareg_set_Runmode(volatile tuareg_runmode_t Target_runmode)
{
    U32 config_load_status;

    if(Tuareg.Runmode != Target_runmode)
    {
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
                config_load_status= config_load();

                /**
                /// TODO (oli#1#): DEBUG: limp home test
                config_load_status= RETURN_FAIL;
                */

                if(config_load_status != RETURN_OK)
                {
                    //save to error register
                    Tuareg_register_error(TERROR_CONFIGLOAD);

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

                    //actually sets table dimension, could be removed
                    init_3Dtables();
                }

                //set sensor defaults
                Tuareg_set_asensor_defaults();
                break;

            case TMODE_MODULEINIT:

                #ifdef TUAREG_MODULE_TEST
                moduletest_moduleinit_action();
                #else

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
                update_ignition_timing( &(Tuareg.process), &(Tuareg.ignition_timing));

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
                default_ignition_timing(&(Tuareg.ignition_timing));

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
    set_fuelpump(PIN_OFF);
    set_injector_ch1(PIN_OFF);
    set_injector_ch2(PIN_OFF);
    set_ignition_ch1(COIL_POWERDOWN);
    set_ignition_ch2(COIL_POWERDOWN);

    //reset MAP calculation
    reset_asensor_sync_integrator(ASENSOR_SYNC_MAP);
}

/**
update engine control strategy based on available sensors and
*/
void Tuareg_update_process_data(process_data_t * pImage)
{
    //collect diagnostic information
    tuareg_diag_log_event(TDIAG_PROCESSDATA_CALLS);

    //crank_T_us
    pImage->crank_T_us= Tuareg.decoder->crank_period_us;

    //engine_rpm
    pImage->engine_rpm= calc_rpm(pImage->crank_T_us);

    //crank_position
    pImage->crank_position= Tuareg.decoder->crank_position;

    //crank position table
    update_crank_position_table(&(pImage->crank_position_table));

    //analog sensors
    pImage->MAP_kPa= Tuareg_get_asensor(ASENSOR_MAP);
    pImage->Baro_kPa= Tuareg_get_asensor(ASENSOR_BARO);
    pImage->TPS_deg= Tuareg_get_asensor(ASENSOR_TPS);
    pImage->IAT_K= Tuareg_get_asensor(ASENSOR_IAT);
    pImage->CLT_K= Tuareg_get_asensor(ASENSOR_CLT);
    pImage->VBAT_V= Tuareg_get_asensor(ASENSOR_VBAT);
    pImage->ddt_TPS= Tuareg.sensors->ddt_TPS;

    // strategy?

    //delay
    //pImage->data_age= decoder_get_data_age_us();


}



/**
recalculate ignition timing if the run mode allows engine operation
*/
void Tuareg_update_ignition_timing()
{

    //collect diagnostic information
    tuareg_diag_log_event(TDIAG_IGNITIONCALC_CALLS);

    switch(Tuareg.Runmode)
     {

        case TMODE_CRANKING:
        case TMODE_RUNNING:
        case TMODE_STB:

            update_ignition_timing( &(Tuareg.process), &(Tuareg.ignition_timing));
            break;


        case TMODE_LIMP:

            default_ignition_timing(&(Tuareg.ignition_timing));
            break;


        default:

            /**
            possible scenario:
            -engine has been killed by kill switch and the crank shaft is still rotating
            */
            break;

     }
}


void Tuareg_trigger_ignition()
{
    VU32 age_us, corr_timing_us;

    //collect diagnostic information
    tuareg_diag_log_event(TDIAG_TRIG_IGN_CALLS);

    /**
    its a normal situation that ignition and dwell are triggered from the same position
    */
    if(Tuareg.decoder->crank_position == Tuareg.ignition_timing.coil_ignition_pos)
    {
        //collect diagnostic information
        tuareg_diag_log_event(TDIAG_TRIG_COIL_IGN);

        //correct timing
        age_us= decoder_get_data_age_us();
        corr_timing_us= subtract_VU32(Tuareg.ignition_timing.coil_ignition_timing_us, age_us);

        trigger_coil_by_timer(corr_timing_us, COIL_IGNITION);
    }

    if(Tuareg.decoder->crank_position == Tuareg.ignition_timing.coil_dwell_pos)
    {
        //collect diagnostic information
        tuareg_diag_log_event(TDIAG_TRIG_COIL_DWELL);

        //correct timing
        age_us= decoder_get_data_age_us();
        corr_timing_us= subtract_VU32(Tuareg.ignition_timing.coil_dwell_timing_us, age_us);

        trigger_coil_by_timer(corr_timing_us, COIL_DWELL);
    }
}




U32 Tuareg_get_asensor(asensors_t sensor)
{
    //if sensor has already been validated and is still healthy
    if( (Tuareg.asensor_validity & (1<< sensor)) && (Tuareg.sensors->asensors_health & (1<< sensor)) )
    {
        //use live value
        return Tuareg.sensors->asensors[sensor];
    }
    else
    {
        //maybe sensor can be validated in this cycle?
        if( (Tuareg.sensors->asensors_health & (1<< sensor)) && (Tuareg.sensors->asensors_valid[sensor] > ASENSOR_VALIDITY_THRES) )
        {
            //sensor successfully validated
            Tuareg.asensor_validity |= (1<< sensor);

            //use live value
            return Tuareg.sensors->asensors[sensor];
        }
        else
        {
            //sensor temporarily disturbed
            Tuareg.asensor_validity &= ~(1<< sensor);

            //use default value
            return Tuareg.asensor_defaults[sensor];
        }

    }
}


void Tuareg_set_asensor_defaults()
{
    Tuareg.asensor_defaults[ASENSOR_O2]= O2_DEFAULT_AFR;
    Tuareg.asensor_defaults[ASENSOR_TPS]= TPS_DEFAULT_DEG;
    Tuareg.asensor_defaults[ASENSOR_IAT]= IAT_DEFAULT_C;
    Tuareg.asensor_defaults[ASENSOR_CLT]= CLT_DEFAULT_C;
    Tuareg.asensor_defaults[ASENSOR_VBAT]= VBAT_DEFAULT_V;
    Tuareg.asensor_defaults[ASENSOR_KNOCK]= KNOCK_DEFAULT;
    Tuareg.asensor_defaults[ASENSOR_BARO]= BARO_DEFAULT_KPA;
    Tuareg.asensor_defaults[ASENSOR_SPARE]= SPARE_DEFAULT;
    Tuareg.asensor_defaults[ASENSOR_MAP]= MAP_DEFAULT_KPA;

}
