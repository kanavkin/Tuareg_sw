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

        case 'FC':

            cli_show_fueling_controls(&(Tuareg.fueling_controls));
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
//            print_process_table_fancy(Port);
            print_process_table(Port);
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
        cli_show_debug_help();
      break;
  }
}

#pragma GCC diagnostic pop



void cli_show_debug_help()
{
/// TODO (oli#9#): keep command list up to date

    print(TS_PORT, "\r\n\r\n\r\n*** Tuareg Debug CLI Help ***\n\r");

    print(TS_PORT, "AD - show sensor diagnostics\n\r");

    print(TS_PORT, "DI - show decoder interface\n\r");
    print(TS_PORT, "DD - show decoder diagnostics\n\r");

    #ifdef DECODER_CIS_DEBUG
    print(TS_PORT, "DC - show decoder CIS debug data\n\r");
    #endif // DECODER_CIS_DEBUG

    print(TS_PORT, "FC - show fueling controls\n\r");
    print(TS_PORT, "FD - show fueling diagnostics\n\r");

    print(TS_PORT, "IC - show ignition controls\n\r");
    print(TS_PORT, "ID - show ignition diagnostics\n\r");

    print(TS_PORT, "TD - show Tuareg diagnostics\n\r");
    print(TS_PORT, "SD - show scheduler diagnostics\n\r");

    print(TS_PORT, "PR - show process data\n\r");
    print(TS_PORT, "PT - show process table\n\r");

    print(TS_PORT, "se - show sensor data\n\r");
    print(TS_PORT, "HL - show high speed log\n\r");
}




void cli_show_process_data(volatile process_data_t * pImage)
{
    print(TS_PORT, "\r\n\r\nProcess data:\r\n");

    print(TS_PORT, "\r\nMAP (kPa), change rate (kPa/s): ");
    printf_F32(TS_PORT, pImage->MAP_kPa);
    printf_F32(TS_PORT, pImage->ddt_MAP);

    print(TS_PORT, "\r\nTPS (deg), change rate (°/s): ");
    printf_F32(TS_PORT, pImage->TPS_deg);
    printf_F32(TS_PORT, pImage->ddt_TPS);

    print(TS_PORT, "\r\nIAT (°C): ");
    printf_F32(TS_PORT, pImage->IAT_K - cKelvin_offset);

    print(TS_PORT, "\r\nCLT (°C): ");
    printf_F32(TS_PORT, pImage->CLT_K - cKelvin_offset);

    print(TS_PORT, "\r\nBARO (kPa): ");
    printf_F32(TS_PORT, pImage->Baro_kPa);

    print(TS_PORT, "\r\nBAT (V): ");
    printf_F32(TS_PORT, pImage->VBAT_V);

    print(TS_PORT, "\r\nAFR: ");
    printf_F32(TS_PORT, pImage->O2_AFR);

    print(TS_PORT, "\r\nGear: ");
    printf_U(TS_PORT, pImage->Gear, NO_PAD);

    print(TS_PORT, "\r\nGround Speed (mm/s): ");
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


void cli_show_fueling_controls(volatile fueling_control_t * pControls)
{
    print(TS_PORT, "\r\n\r\ncurrent fueling controls:");

    //basic parameters
    print(TS_PORT, "\r\nVE (%): ");
    printf_F32(TS_PORT, pControls->VE_pct);

    print(TS_PORT, "\r\ncharge temperature (°C): ");
    printf_F32(TS_PORT, pControls->charge_temp_K - cKelvin_offset);

    print(TS_PORT, "\r\nair density (µg/cm³): ");
    printf_F32(TS_PORT, pControls->air_density);

    print(TS_PORT, "\r\nair flow rate (g/s): ");
    printf_F32(TS_PORT, pControls->air_flowrate_gps);

    print(TS_PORT, "\r\nAFR target: ");
    printf_F32(TS_PORT, pControls->AFR_target);


    //WUE
    print(TS_PORT, "\r\nWUE (%): ");
    printf_F32(TS_PORT, pControls->WUE_pct);

    //ASE
    print(TS_PORT, "\r\nASE (%), cycles left: ");
    printf_F32(TS_PORT, pControls->ASE_pct);
    printf_U(TS_PORT, pControls->ASE_cycles_left, NO_PAD | NO_TRAIL);

    //BARO
    print(TS_PORT, "\r\nBARO corr (%): ");
    printf_F32(TS_PORT, pControls->BARO_pct);

    //legacy AE
    print(TS_PORT, "\r\nlegacy AE (µg), cycles left: ");
    printf_F32(TS_PORT, pControls->legacy_AE_ug);
    printf_U(TS_PORT, pControls->legacy_AE_cycles_left, NO_PAD | NO_TRAIL);

    //fuel masses
    print(TS_PORT, "\r\nbase fuel mass (µg): ");
    printf_F32(TS_PORT, pControls->base_fuel_mass_ug);
    print(TS_PORT, "\r\ntarget fuel mass (µg): ");
    printf_F32(TS_PORT, pControls->target_fuel_mass_ug);

    // X-Tau load compensation
    print(TS_PORT, "\r\nwall fuel mass (µg): ");
    printf_F32(TS_PORT, pControls->wall_fuel_mass_ug);
    print(TS_PORT, "\r\ncommanded fuel mass (µg): ");
    printf_F32(TS_PORT, pControls->cmd_fuel_mass_ug);

    //injector parameters
    print(TS_PORT, "\r\ninjector target dc (%), intervals 1/2 (µs),  dead time (µs): ");
    printf_F32(TS_PORT, pControls->injector_target_dc);
    printf_F32(TS_PORT, pControls->injector1_interval_us);
    printf_F32(TS_PORT, pControls->injector2_interval_us);
    printf_F32(TS_PORT, pControls->injector_deadtime_us);

    print(TS_PORT, "\r\ninjector begin pos, timing 1/2, phase 1/2: ");
    printf_crkpos(TS_PORT, pControls->injection_begin_pos);
    printf_F32(TS_PORT, pControls->injector1_timing_us);
    printf_F32(TS_PORT, pControls->injector2_timing_us);
    printf_phase(TS_PORT, pControls->seq_injector1_begin_phase);
    printf_phase(TS_PORT, pControls->seq_injector2_begin_phase);

    //status data
    print(TS_PORT, "\r\nflags\r\nvalid MAP_nTPS AFR_fallback inj_beg_valid: ");
    UART_Tx(TS_PORT, (pControls->flags.valid? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pControls->flags.MAP_nTPS? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pControls->flags.AFR_fallback? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pControls->flags.injection_begin_valid? '1' :'0'));

    print(TS_PORT, "\r\ndry_cranking sequential_mode injector_dc_clip: ");
    UART_Tx(TS_PORT, (pControls->flags.dry_cranking? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pControls->flags.sequential_mode? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pControls->flags.injector_dc_clip? '1' :'0'));


    print(TS_PORT, "\r\nWUE ASE BARO_corr load_comp: ");
    UART_Tx(TS_PORT, (pControls->flags.WUE_active? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pControls->flags.ASE_active? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pControls->flags.BARO_corr_active? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pControls->flags.load_transient_comp? '1' :'0'));

    print(TS_PORT, "\r\nAE active, MAP_accel, TPS_accel, MAP_decel, TPS_decel: ");
    UART_Tx(TS_PORT, (pControls->flags.legacy_AE_active? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pControls->flags.legacy_AE_trig_MAP_accel? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pControls->flags.legacy_AE_trig_TPS_accel? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pControls->flags.legacy_AE_trig_MAP_decel? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pControls->flags.legacy_AE_trig_TPS_decel? '1' :'0'));
    UART_Tx(TS_PORT, '-');
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



