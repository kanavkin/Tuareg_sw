#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_adc.h"
#include "stm32_libs/boctok_types.h"

#include "decoder_hw.h"
#include "decoder_logic.h"
#include "ignition_logic.h"
#include "ignition_hw.h"
#include "scheduler.h"
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

#include "debug.h"
#include "Tuareg.h"


void Tuareg_update_Runmode()
{
    /**
    calculate new system state
    */

    //collect diagnostic information
    Tuareg.diag[TDIAG_MODECTRL] += 1;


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
            Tuareg.diag[TDIAG_KILL_RUNSWITCH] += 1;

            #warning TODO (oli#9#): debug message enabled
            UART_Send(DEBUG_PORT, "\r\nHALT! reason: DSENSOR_RUN");
        }

        //shut engine off if the CRASH sensor is engaged
        if( (Tuareg.sensors->dsensors & (1<< DSENSOR_CRASH)) == CRASH_SENSOR_ENGAGE_LEVEL)
        {
            Tuareg_set_Runmode(TMODE_HALT);

            //collect diagnostic information
            Tuareg.diag[TDIAG_KILL_SIDESTAND] += 1;

            #warning TODO (oli#9#): debug message enabled
            UART_Send(DEBUG_PORT, "\r\nHALT! reason: DSENSOR_CRASH");
        }

        #warning TODO (oli#3#): implement cranking handling
        if((Tuareg.decoder->engine_rpm > 800) && (Tuareg.decoder->crank_position != CRK_POSITION_UNDEFINED))
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
            Tuareg.diag[TDIAG_KILL_RUNSWITCH] += 1;

            #warning TODO (oli#9#): debug message enabled
            UART_Send(DEBUG_PORT, "\r\nHALT! reason: DSENSOR_RUN");
        }

        //shut engine off if the CRASH sensor is engaged
        if( (Tuareg.sensors->dsensors & (1<< DSENSOR_CRASH)) == CRASH_SENSOR_ENGAGE_LEVEL)
        {
            Tuareg_set_Runmode(TMODE_HALT);

            //collect diagnostic information
            Tuareg.diag[TDIAG_KILL_SIDESTAND] += 1;

            #warning TODO (oli#9#): debug message enabled
            UART_Send(DEBUG_PORT, "\r\nHALT! reason: DSENSOR_CRASH");
        }

        break;


    case TMODE_HALT:

        //allow turning engine on if the RUN switch is engaged AND the CRASH sensor disengaged
        if( ((Tuareg.sensors->dsensors & (1<< DSENSOR_RUN)) == RUN_SENSOR_ENGAGE_LEVEL ) && ((Tuareg.sensors->dsensors & (1<< DSENSOR_CRASH)) != CRASH_SENSOR_ENGAGE_LEVEL) )
        {
            Tuareg_set_Runmode(TMODE_STB);

            //collect diagnostic information
            Tuareg.diag[TDIAG_HALT_STB_TR] += 1;
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



void Tuareg_set_Runmode(volatile tuareg_runmode_t Target_runmode)
{
    if(Tuareg.Runmode != Target_runmode)
    {
        /**
        run state transition actions
        */
        switch(Target_runmode)
        {
            case TMODE_HWINIT:

                //system init

                //collect diagnostic information
                Tuareg.diag[TDIAG_ENTER_INIT] += 1;


                break;

            case TMODE_CONFIGLOAD:


                break;

            case TMODE_MODULEINIT:


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
                    Tuareg.diag[TDIAG_RUNNING_HALT_TR] += 1;

                     #warning TODO (oli#9#): debug message enabled
                    UART_Send(DEBUG_PORT, "\r\nstate transition TMODE_RUNNING --> TMODE_HALT");
                }
                else if(Tuareg.Runmode == TMODE_HWINIT)
                {
                    Tuareg.diag[TDIAG_INIT_HALT_TR] += 1;

                     #warning TODO (oli#9#): debug message enabled
                    UART_Send(DEBUG_PORT, "\r\nstate transition TMODE_HWINIT --> TMODE_HALT");
                }
                else if(Tuareg.Runmode == TMODE_STB)
                {
                    Tuareg.diag[TDIAG_STB_HALT_TR] += 1;

                     #warning TODO (oli#9#): debug message enabled
                    UART_Send(DEBUG_PORT, "\r\nstate transition TMODE_STB --> TMODE_HALT");
                }


                //collect diagnostic information
                Tuareg.diag[TDIAG_ENTER_HALT] += 1;

                break;

            case TMODE_RUNNING:

                //begin normal engine operation

                //turn fuel pump on to maintain fuel pressure
                set_fuelpump(ON);

                //begin ignition coil charge for first spark
                set_ignition_ch1(COIL_DWELL);
                //set_ignition_ch2(COIL_DWELL);

                //provide initial ignition timing for cranking
                default_ignition_timing(&Tuareg.ignition_timing);

                //collect diagnostic information

                //collect diagnostic information
                if(Tuareg.Runmode == TMODE_CRANKING)
                {
                    Tuareg.diag[TDIAG_CRANKING_RUNNING_TR] += 1;

                    #warning TODO (oli#9#): debug message enabled
                    UART_Send(DEBUG_PORT, "\r\nstate transition TMODE_CRANKING --> TMODE_RUNNING");
                }

                Tuareg.diag[TDIAG_ENTER_RUNNING] += 1;

                break;

            case TMODE_STB:

                #warning TODO (oli#9#): check if we need fuel pump priming

                //engine stalled, system ready for start
                Tuareg_stop_engine();

                //collect diagnostic information
                if(Tuareg.Runmode == TMODE_RUNNING)
                {
                    Tuareg.diag[TDIAG_RUNNING_STB_TR] += 1;

                    #warning TODO (oli#9#): debug message enabled
                    UART_Send(DEBUG_PORT, "\r\nstate transition TMODE_RUNNING --> TMODE_STB");
                }
                else if(Tuareg.Runmode == TMODE_HALT)
                {
                    Tuareg.diag[TDIAG_HALT_STB_TR] += 1;

                    #warning TODO (oli#9#): debug message enabled
                    UART_Send(DEBUG_PORT, "\r\nstate transition TMODE_HALT --> TMODE_STB");
                }
                else if(Tuareg.Runmode == TMODE_CRANKING)
                {
                    Tuareg.diag[TDIAG_CRANKING_STB_TR] += 1;

                    #warning TODO (oli#9#): debug message enabled
                    UART_Send(DEBUG_PORT, "\r\nstate transition TMODE_CRANKING --> TMODE_STB");
                }

                //provide ignition timing for engine startup
                default_ignition_timing(&Tuareg.ignition_timing);

                Tuareg.diag[TDIAG_ENTER_STB] += 1;
                break;

            case TMODE_CRANKING:

                #warning TODO (oli#9#): check if we need fuel pump priming

                //collect diagnostic information
                Tuareg.diag[TDIAG_ENTER_CRANKING] += 1;


                #warning TODO (oli#9#): debug message enabled
                UART_Send(DEBUG_PORT, "\r\nstate transition TMODE_CRANKING");

                break;


            default:

                //very strange
                UART_Send(DEBUG_PORT, "ERROR! default branch in Tuareg_set_Runmode reached");
                Tuareg_stop_engine();
                break;

        }

        //finally set the new mode
        Tuareg.Runmode= Target_runmode;

    }

}





void Tuareg_stop_engine()
{
    //turn off vital engine actors
    set_fuelpump(OFF);
    set_injector_ch1(OFF);
    set_injector_ch2(OFF);
    set_ignition_ch1(COIL_POWERDOWN);
    set_ignition_ch2(COIL_POWERDOWN);

    //reset MAP calculation
    reset_asensor_sync_integrator(ASENSOR_SYNC_MAP);
}


/**
recalculate ignition timing if the run mode allows engine operation
*/
void Tuareg_update_ignition_timing()
{

    //collect diagnostic information
    Tuareg.diag[TDIAG_IGNITIONCALC_CALLS] += 1;


    switch(Tuareg.Runmode)
     {

        case TMODE_CRANKING:
        case TMODE_RUNNING:
        case TMODE_STB:

            calculate_ignition_timing(&(Tuareg.ignition_timing), Tuareg.decoder->crank_T_us, Tuareg.decoder->engine_rpm);
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
    //collect diagnostic information
    Tuareg.diag[TDIAG_TRIG_IGN_CALLS] += 1;

    /**
    its a normal situation that ignition and dwell are triggered from the same position
    */
    if(Tuareg.decoder->crank_position == Tuareg.ignition_timing.coil_ignition_pos)
    {
        //collect diagnostic information
        Tuareg.diag[TDIAG_TRIG_COIL_IGN] += 1;

        trigger_coil_by_timer(Tuareg.ignition_timing.coil_ignition_timing_us, COIL_IGNITION);
    }

    if(Tuareg.decoder->crank_position == Tuareg.ignition_timing.coil_dwell_pos)
    {
        //collect diagnostic information
        Tuareg.diag[TDIAG_TRIG_COIL_DWELL] += 1;

        trigger_coil_by_timer(Tuareg.ignition_timing.coil_dwell_timing_us, COIL_DWELL);
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


/**
writes the collected diagnostic data to the memory a pTarget
*/
void Tuareg_export_diag(VU32 * pTarget)
{
    U32 count;

    for(count=0; count < TDIAG_COUNT; count++)
    {
        pTarget[count]= Tuareg.diag[count];
    }
}
