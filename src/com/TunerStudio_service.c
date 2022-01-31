#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"

#include "Tuareg_console.h"
#include "TunerStudio.h"
#include "TunerStudio_outChannel.h"
#include "TunerStudio_service.h"
#include "Tuareg_service_functions.h"

#include "table.h"

#include "decoder_config.h"
#include "decoder_debug.h"
#include "ignition_config.h"
#include "sensor_calibration.h"
#include "Tuareg.h"
#include "eeprom.h"
#include "eeprom_layout.h"
#include "analog_sensors.h"
#include "digital_sensors.h"
#include "debug_port_messages.h"
#include "base_calc.h"

#include "diagnostics.h"

#include "bitfields.h"

#include "process_table.h"

#include "fault_log.h"

#include "debug_port_messages.h"

//#define TS_SERVICE_DEBUG

#ifdef TS_SERVICE_DEBUG
#warning TS Service Debug messages enabled
#endif // TS_SERVICE_DEBUG


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmultichar"

void ts_debug_features(U32 FeatureID)
{
    U8 u8_data =0;
    U32 data_1 =0;
    U32 data_2 =0;

    switch (FeatureID)
    {

        case 'ep':

            /**
            dump eeprom content in binary format
            */

            print(TS_PORT, "EEprom data:\r\n");

            for(data_1=0; data_1 < EEPROM_STORAGE_END; data_1++)
            {
                data_2= eeprom_read_byte(data_1, &u8_data);

                if(data_2 != 0)
                {
                    //read error - terminate
                    print(TS_PORT, "ERROR!");
                    return;
                }

                UART_Tx(TS_PORT, u8_data);
            }

            print(TS_PORT, "Transmission complete.\r\n");

            break;


        case 'rs':

            NVIC_SystemReset();
            break;

        case 'fa':

            Fatal(TID_TUNERSTUDIO_SERVICE, 42);
            break;

        case 'li':

            Limp(TID_TUNERSTUDIO_SERVICE, 43);
            break;

        case 'fl':

            if(Tuareg_console.cli_permissions.faultlog_permission == true)
            {
                Erase_Fault_Log();
            }
            else
            {
                #ifdef TS_SERVICE_DEBUG
                DebugMsg_Error("Will not erase Fault Log - no permission!");
                #endif // TS_SERVICE_DEBUG
            }

            break;


    default:
      break;
  }
}


void ts_debug_info(U32 InfoID, USART_TypeDef * Port)
{
    switch (InfoID)
    {
        case 'AD':

            print_sensors_diag(Port);
            break;

        case 'DD':

            print_decoder_diag(Port);
            break;


        case 'DI':

            cli_show_decoder_interface(Tuareg.pDecoder);

            break;

        #ifdef DECODER_CIS_DEBUG
        case 'DC':

            print_decoder_cis_debug_data(TS_PORT);
            break;
        #endif // DECODER_CIS_DEBUG


        case 'TD':

            print_tuareg_diag(Port);
            break;


        case 'SD':

            print_scheduler_diag(Port);
            break;


        case 'ID':

            print_ignition_diag(Port);
            break;



        case 'IC':

            cli_show_ignition_controls(&(Tuareg.ignition_controls));
            break;


        case 'PR':

            /**
            print current process data
            */
            cli_show_process_data(&(Tuareg.process));
            break;


        case 'PT':

            /**
            print current process table fancy
            */
            print_process_table_fancy(Port);
            break;



        case 'se':

            //show analog and digital sensor values
            cli_print_sensor_data(Port);
            break;

        case 'FD':

            print_fueling_diag(Port);
            break;



        case 'HL':

            show_highspeedlog(Port);
            break;


    default:
      break;
  }
}

#pragma GCC diagnostic pop


void cli_show_process_data(volatile process_data_t * pImage)
{
    /**
    volatile crank_position_t crank_position;
    volatile crank_position_table_t crank_position_table;
    VU32 crank_T_us;
    VU32 engine_rpm;

    volatile ctrl_strategy_t ctrl_strategy;

    VF32 MAP_Pa;
    VF32 Baro_Pa;
    VF32 TPS_deg;
    VF32 ddt_TPS;
    VF32 IAT_C;
    VF32 CLT_C;
    VF32 VBAT_V;
    */

    print(TS_PORT, "\r\n\r\nprocess data image:\r\n");
/*
    print(TS_PORT, "rpm: ");
    printf_U(TS_PORT, pImage->crank_rpm, NO_PAD);
*/


    print(TS_PORT, "\r\nMAP (kPa), BARO (kPa), TPS (deg), ddt_TPS, IAT (C), CLT (C), VBAT (V), O2 (AFR), Gear:\r\n");

    printf_F32(TS_PORT, pImage->MAP_kPa);
    printf_F32(TS_PORT, pImage->Baro_kPa);
    printf_F32(TS_PORT, pImage->TPS_deg);
    printf_F32(TS_PORT, pImage->ddt_TPS);
    printf_F32(TS_PORT, pImage->IAT_K - cKelvin_offset);
    printf_F32(TS_PORT, pImage->CLT_K - cKelvin_offset);
    printf_F32(TS_PORT, pImage->VBAT_V);
    printf_F32(TS_PORT, pImage->O2_AFR);
    printf_U(TS_PORT, pImage->Gear, NO_PAD);

}

void cli_show_ignition_controls(volatile ignition_controls_t * pTiming)
{
    print(TS_PORT, "\r\n\r\nignition setup:");

    print(TS_PORT, "\r\nadvance (deg), base pos, timing (us): ");
    printf_U(TS_PORT, pTiming->ignition_advance_deg, NO_PAD);
    printf_crkpos(TS_PORT, pTiming->ignition_pos);
    UART_Tx(TS_PORT, ' ');
    printf_U(TS_PORT, pTiming->ignition_timing_us, NO_PAD);

    print(TS_PORT, "\r\ndwell base pos, timing, duration: ");
    printf_crkpos(TS_PORT, pTiming->dwell_pos);
    UART_Tx(TS_PORT, ' ');
    printf_U(TS_PORT, pTiming->dwell_timing_us, NO_PAD);
    UART_Tx(TS_PORT, ' ');
    printf_U(TS_PORT, pTiming->dwell_us, NO_PAD);

    print(TS_PORT, "\r\nflags: valid dyn seq_mode cold_idle a_map: ");

    UART_Tx(TS_PORT, (pTiming->flags.valid? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->flags.dynamic_controls? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->flags.sequential_mode? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->flags.cold_idle? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->flags.advance_map? '1' :'0'));

}


void cli_show_decoder_interface(volatile decoder_output_t * pInterface)
{
    print(TS_PORT, "\r\n\r\ndecoder interface:");

    print(TS_PORT, "\r\ncrank position: ");
    printf_crkpos(TS_PORT, pInterface->crank_position);

    print(TS_PORT, "\r\nphase: ");
    printf_phase(TS_PORT, pInterface->phase);

    print(TS_PORT, "\r\ncrank period (us): ");
    printf_U(TS_PORT, pInterface->crank_period_us, NO_PAD);

    print(TS_PORT, "\r\ncrank rpm: ");
    printf_U(TS_PORT, pInterface->crank_rpm, NO_PAD);

    print(TS_PORT, "\r\ncrank acceleration: ");
    printf_F32(TS_PORT, pInterface->crank_acceleration);

    print(TS_PORT, "\r\nstate: pos_valid phase_valid period_valid rpm_valid accel_valid standstill: ");
    UART_Tx(TS_PORT, (pInterface->flags.position_valid? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pInterface->flags.phase_valid? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pInterface->flags.period_valid? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pInterface->flags.rpm_valid? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pInterface->flags.accel_valid? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pInterface->flags.standstill? '1' :'0'));
}


void cli_print_sensor_data(USART_TypeDef * Port)
{
    //can be called cyclic by adding a lowspeed action
    U32 sensor;

    print(Port, "\r\nsensors:\r\n");

    print(Port, "\r\nO2: ");
    printf_F32(Port, Analog_Sensors[ASENSOR_O2].out);
    print(Port, " (");
    printf_U(Port, Analog_Sensors[ASENSOR_O2].raw, NO_PAD | NO_TRAIL);
    print(Port, ")");

    print(Port, "\r\nTPS: ");
    printf_F32(Port, Analog_Sensors[ASENSOR_TPS].out);
    print(Port, " (");
    printf_U(Port, Analog_Sensors[ASENSOR_TPS].raw, NO_PAD | NO_TRAIL);
    print(Port, ")");

    print(Port, "\r\nIAT: ");
    printf_F32(Port, Analog_Sensors[ASENSOR_IAT].out);
    print(Port, " (");
    printf_U(Port, Analog_Sensors[ASENSOR_IAT].raw, NO_PAD | NO_TRAIL);
    print(Port, ")");

    print(Port, "\r\nCLT: ");
    printf_F32(Port, Analog_Sensors[ASENSOR_CLT].out);
    print(Port, " (");
    printf_U(Port, Analog_Sensors[ASENSOR_CLT].raw, NO_PAD | NO_TRAIL);
    print(Port, ")");

    print(Port, "\r\nVBAT: ");
    printf_F32(Port, Analog_Sensors[ASENSOR_VBAT].out);
    print(Port, " (");
    printf_U(Port, Analog_Sensors[ASENSOR_VBAT].raw, NO_PAD | NO_TRAIL);
    print(Port, ")");

    print(Port, "\r\nKNOCK: ");
    printf_F32(Port, Analog_Sensors[ASENSOR_KNOCK].out);
    print(Port, " (");
    printf_U(Port, Analog_Sensors[ASENSOR_KNOCK].raw, NO_PAD | NO_TRAIL);
    print(Port, ")");

    print(Port, "\r\nBARO: ");
    printf_F32(Port, Analog_Sensors[ASENSOR_BARO].out);
    print(Port, " (");
    printf_U(Port, Analog_Sensors[ASENSOR_BARO].raw, NO_PAD | NO_TRAIL);
    print(Port, ")");

    print(Port, "\r\nGEAR: ");
    printf_F32(Port, Analog_Sensors[ASENSOR_GEAR].out);
    print(Port, " (");
    printf_U(Port, Analog_Sensors[ASENSOR_GEAR].raw, NO_PAD | NO_TRAIL);
    print(Port, ")");

    print(Port, "\r\nMAP: ");
    printf_F32(Port, Analog_Sensors[ASENSOR_MAP].out);
    print(Port, " (");
    printf_U(Port, Analog_Sensors[ASENSOR_MAP].raw, NO_PAD | NO_TRAIL);
    print(Port, ")");

    print(Port, "\r\n");
    print(Port, "\r\nDIGITAL: SPARE2-NEUTRAL-RUN-CRASH-DEBUG\r\n");

    for(sensor=0; sensor < DSENSOR_COUNT; sensor++)
    {
        if(Digital_Sensors.all_sensors & (1<< sensor))
        {
            UART_Tx(Port, '1');
        }
        else
        {
            UART_Tx(Port, '0');
        }

        UART_Tx(Port, '-');
    }


}



