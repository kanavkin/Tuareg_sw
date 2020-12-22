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

#include "utils.h"
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
        case 'dD':

            print_decoder_diag(TS_PORT);
            break;

        case 'dd':

            print_decoder_diag_legend(TS_PORT);
            break;

        case 'dT':

            print_tuareg_diag(TS_PORT);
            break;

        case 'dt':

            print_tuareg_diag_legend(TS_PORT);
            break;

        case 'dS':

            print_scheduler_diag(TS_PORT);
            break;

        case 'ds':

            print_scheduler_diag_legend(TS_PORT);
            break;

        case 'dI':

            print_ignhw_diag(TS_PORT);
            break;

        case 'di':

            print_ignhw_diag_legend(TS_PORT);
            break;



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

        case 'Pr':

            /**
            print current process data
            */
            cli_show_process_data(&(Tuareg.process));
            break;

        case 'Pt':

            /**
            print current process table
            */
            print_process_table(TS_PORT);
            break;

        case 'Ig':

            /**
            print current ignition setup
            */
            cli_show_ignition_timing(&(Tuareg.ignition_controls));

            break;

        case 'sn':

            //show analog and digital sensor values
            print_sensor_data(TS_PORT);
            break;

        case 'rs':

            //show analog and digital sensor values
            NVIC_SystemReset();
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
/*
    print(TS_PORT, "\r\ncrank position: ");
   printf_crkpos(TS_PORT, pImage->crank_position);


    print(TS_PORT, "\r\ncrank position table: ");

    for(i=0; i< CRK_POSITION_COUNT; i++)
    {
        printf_U(TS_PORT, pImage->crank_position_table.a_deg[i], TYPE_U16, PAD);
    }
*/
    print(TS_PORT, "\r\ncrank rotational period: ");
    printf_U(TS_PORT, pImage->crank_T_us, NO_PAD);

    print(TS_PORT, "rpm: ");
    printf_U(TS_PORT, pImage->crank_rpm, NO_PAD);

    print(TS_PORT, "\r\nstrategy: ");
    printf_U(TS_PORT, pImage->ctrl_strategy, NO_PAD);

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

void cli_show_ignition_timing(volatile ignition_control_t * pTiming)
{
    /**

    U16 ignition_advance_deg;
    U32 ignition_timing_us;
    crank_position_t ignition_pos;

    crank_position_t dwell_pos_phased;
    engine_phase_t dwell_phase_cyl1;
    engine_phase_t dwell_phase_cyl2;
    U8 dwell_ms_phased;

    crank_position_t dwell_pos_unphased;
    U8 dwell_ms_unphased;

    ignition_logic_state_t state;

    U8 default_timing :1;
        U8 cranking_timing :1;
        U8 rev_limiter :1;
        U8 dynamic :1;
        U8 cold_idle :1;
        U8 advance_map :1;
        U8 advance_tps :1;
        U8 extended_dwell :1;
    */

    print(TS_PORT, "\r\n\r\nignition setup:");

    print(TS_PORT, "\r\nadvance (deg), position, timing (us): ");
    printf_U(TS_PORT, pTiming->ignition_advance_deg, NO_PAD);
    printf_crkpos(TS_PORT, pTiming->ignition_pos);
    printf_U(TS_PORT, pTiming->ignition_timing_us, NO_PAD);

    print(TS_PORT, "\r\ndwell -phased- duration (ms), position, phase cyl #1, phase cyl #2: ");
    printf_U(TS_PORT, pTiming->dwell_ms_phased, NO_PAD);
    printf_crkpos(TS_PORT, pTiming->dwell_pos_phased);
    printf_phase(TS_PORT, pTiming->dwell_phase_cyl1);
    printf_phase(TS_PORT, pTiming->dwell_phase_cyl2);

    print(TS_PORT, "\r\ndefault-cranking-rev_limiter-dyn-cold_idle-a_map-a_tps-ext_dwell: ");

    UART_Tx(TS_PORT, (pTiming->state.default_timing? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->state.cranking_timing? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->state.rev_limiter? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->state.dynamic? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->state.cold_idle? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->state.advance_map? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->state.advance_tps? '1' :'0'));
    UART_Tx(TS_PORT, '-');
    UART_Tx(TS_PORT, (pTiming->state.extended_dwell? '1' :'0'));

    print(TS_PORT, "\r\ndwell -unphased- duration (ms), position: ");
    printf_U(TS_PORT, pTiming->dwell_ms_unphased, NO_PAD);
    printf_crkpos(TS_PORT, pTiming->dwell_pos_unphased);

}
