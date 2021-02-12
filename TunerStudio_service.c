#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"

#include "Tuareg_console.h"
#include "TunerStudio.h"
#include "TunerStudio_legacy.h"
#include "TunerStudio_outChannel.h"
#include "TunerStudio_service.h"

#include "table.h"

#include "config_pages.h"
#include "config_tables.h"
#include "decoder_config.h"
#include "ignition_config.h"
#include "sensor_calibration.h"
#include "legacy_config.h"
#include "Tuareg.h"
#include "eeprom.h"
#include "eeprom_layout.h"
#include "sensors.h"
#include "debug.h"
#include "base_calc.h"
#include "diagnostics.h"
#include "bitfields.h"

#include "process_table.h"

#define TS_SERVICE_DEBUG


void ts_service_features(U32 FeatureID)
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

            //show analog and digital sensor values
            NVIC_SystemReset();
            break;


    default:
      break;
  }
}



void ts_service_info(U32 InfoID)
{
    switch (InfoID)
    {
        case 'DD':

            print_decoder_diag(TS_PORT);
            break;

        case 'Dd':

            print_decoder_diag_legend(TS_PORT);
            break;

        case 'DI':

            cli_show_decoder_interface(Tuareg.decoder);

            break;

        case 'TD':

            print_tuareg_diag(TS_PORT);
            break;

        case 'Td':

            print_tuareg_diag_legend(TS_PORT);
            break;

        case 'SD':

            print_scheduler_diag(TS_PORT);
            break;

        case 'Sd':

            print_scheduler_diag_legend(TS_PORT);
            break;

        case 'ID':

            print_ignition_diag(TS_PORT);
            break;

        case 'Id':

            print_ignition_diag_legend(TS_PORT);
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
            print_process_table_fancy(TS_PORT);
            break;



        case 'se':

            //show analog and digital sensor values
            print_sensor_data(TS_PORT);
            break;


    default:
      break;
  }
}



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

    print(TS_PORT, "rpm: ");
    printf_U(TS_PORT, pImage->crank_rpm, NO_PAD);



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

void cli_show_ignition_controls(volatile ignition_control_t * pTiming)
{
    print(TS_PORT, "\r\n\r\nignition setup:");

    print(TS_PORT, "\r\nadvance (deg), base pos, timing (us): ");
    printf_U(TS_PORT, pTiming->ignition_advance_deg, NO_PAD);
    printf_crkpos(TS_PORT, pTiming->ignition_pos);
    UART_Tx(TS_PORT, ' ');
    printf_U(TS_PORT, pTiming->ignition_timing_us, NO_PAD);

    print(TS_PORT, "\r\ndwell base pos, timing seq, batch: ");
    printf_crkpos(TS_PORT, pTiming->dwell_pos);
    UART_Tx(TS_PORT, ' ');
    printf_U(TS_PORT, pTiming->dwell_timing_sequential_us, NO_PAD);
    printf_U(TS_PORT, pTiming->dwell_timing_batch_us, NO_PAD);

    print(TS_PORT, "\r\ndwell duration seq, batch: ");
    printf_U(TS_PORT, pTiming->dwell_sequential_us, NO_PAD);
    printf_U(TS_PORT, pTiming->dwell_batch_us, NO_PAD);

    print(TS_PORT, "\r\nstate: valid default cranking dyn rev_limit seq_mode cold_idle a_map a_tps: ");

    UART_Tx(TS_PORT, (pTiming->state.valid? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->state.default_controls? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->state.cranking_controls? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->state.dynamic_controls? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->state.rev_limiter? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->state.sequential_mode? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->state.cold_idle? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->state.advance_map? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->state.advance_tps? '1' :'0'));

}


void cli_show_decoder_interface(volatile Tuareg_decoder_t * pInterface)
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

    print(TS_PORT, "\r\nstate: pos_valid phase_valid period_valid rpm_valid accel_valid timeout: ");
    UART_Tx(TS_PORT, (pInterface->outputs.position_valid? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pInterface->outputs.phase_valid? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pInterface->outputs.period_valid? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pInterface->outputs.rpm_valid? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pInterface->outputs.accel_valid? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pInterface->outputs.timeout? '1' :'0'));


}


