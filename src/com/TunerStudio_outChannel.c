#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"

#include "Tuareg_console.h"
#include "TunerStudio.h"
#include "TunerStudio_outChannel.h"
#include "TunerStudio_service.h"

#include "table.h"


#include "Tuareg.h"
#include "eeprom.h"
#include "eeprom_layout.h"
#include "sensors.h"
#include "base_calc.h"
#include "diagnostics.h"
#include "bitfields.h"

#include "process_table.h"

#define TS_DEBUG


Output_Channels_t Out;

volatile U8 * const pOut_data= (volatile U8 *) &Out;
const U32 cOut_size= sizeof(Out);


void ts_sendOutputChannels(USART_TypeDef * Port)
{
    /**
    the TS connection timer ts_secl shall be zero right after connection establishment
    */
    if(Tuareg_console.ts_connected == false)
    {
        Tuareg_console.ts_secl= 0;
        Tuareg_console.ts_connected= true;
    }

    //secl             = scalar, U08,  0, "sec",    1.000, 0.000
    Out.secl= Tuareg_console.ts_secl;

    //tuareg           = scalar, U32,  1, "bits",   1.000, 0.000
    ts_tuareg_bits(&(Out.tuareg_bits));

    //ignition         = scalar,  U08,    5, "bits",   1.000, 0.000
    Out.ignition_bits= Tuareg.ignition_controls.flags.all_flags;

    //fueling         = scalar,  U16,    6, "bits",   1.000, 0.000
    Out.fueling_bits= Tuareg.fueling_controls.flags.all_flags;

    //comm             = scalar, U08,  8, "bits",   1.000, 0.000
    Out.com_bits= Tuareg_console.cli_permissions.all_flags;

    //rpm              = scalar,   U16,    9, "rpm",    1.000, 0.000
    Out.rpm= (Tuareg.pDecoder->outputs.rpm_valid == true)? Tuareg.pDecoder->crank_rpm : 0;

    //rpmDOT           = scalar,   F32,    11, "rpm/s",  1.000, 0.000
    Out.ddt_rpm= (Tuareg.pDecoder->outputs.accel_valid == true)? 1 : 0;

    //advance         = scalar,   U16,    15, "deg",    1.000, 0.000
    Out.ignition_adv_deg= Tuareg.ignition_controls.ignition_advance_deg;

    //dwell	        = scalar,   U16,    17, "us",     0.100, 0.00
    Out.ignition_dwell_us= Tuareg.ignition_controls.dwell_us;

    //VE              = scalar,   F32,    19, "%",  1.000, 0.000
    Out.VE_pct= Tuareg.fueling_controls.VE_pct;

    //airDens         = scalar,   F32,    23, "ug/cm3",  1.000, 0.000
    Out.air_dens= Tuareg.fueling_controls.air_density;

    //BasefuelMass    = scalar,   U32,    27, "ug",  1.000, 0.000
    Out.base_fuel_mass_ug= Tuareg.fueling_controls.base_fuel_mass_ug;

    //TargetfuelMass  = scalar,   U32,    31, "ug",  1.000, 0.000
    Out.target_fuel_mass_ug= Tuareg.fueling_controls.target_fuel_mass_ug;

    //AFRtgt          = scalar,   F32,    35, "AFR",  1.000, 0.000
    Out.target_AFR= Tuareg.fueling_controls.AFR_target;

    //inj1Iv          = scalar,   U32,    39, "us",  1.000, 0.000
    Out.inj1_interval_us= Tuareg.fueling_controls.injector1_interval_us;

    //inj2Iv          = scalar,   U32,    43, "us",  1.000, 0.000
    Out.inj2_interval_us= Tuareg.fueling_controls.injector2_interval_us;

    //injDcTgt        = scalar,   U32,    47, "us",  1.000, 0.000
    Out.inj_dc_pct= Tuareg.fueling_controls.injector_target_dc;

    //MAP             = scalar,   F32,    51, "kpa",    1.000, 0.000
    Out.MAP_kPa= Tuareg.process.MAP_kPa;

    //MAPdot          = scalar,   F32,    55, "kpa/s",    1.000, 0.000
    Out.ddt_MAP= Tuareg.process.ddt_MAP;

    //baro            = scalar,   F32,    59, "kpa",      1.000, 0.000
    Out.BARO_kPa= Tuareg.process.Baro_kPa;

    //TPS             = scalar,   F32,    63, "deg",      1.000, 0.000
    Out.TPS_deg= Tuareg.process.TPS_deg;

    //TPSdot          = scalar,   F32,    67, "deg/s",    10.00, 0.000
    Out.ddt_TPS= Tuareg.process.ddt_TPS;

    //IAT             = scalar,   F32,    71, "K",    1.000, -273.15
    Out.IAT_K= Tuareg.process.IAT_K;

    //CLT             = scalar,   F32,    75, "K",    1.000, -273.15
    Out.CLT_K= Tuareg.process.CLT_K;

    //battery         = scalar,   F32,    79, "V",      1.000, 0.000
    Out.BAT_V= Tuareg.process.VBAT_V;

    //AFR             = scalar,   F32,    83, "O2",     1.000, 0.000
    Out.AFR= Tuareg.process.O2_AFR;

    //knock level     = scalar,   F32,    87, "Knock",     1.000, 0.000
    Out.Knock= Tuareg.process.Knock_level;

    //gear             = scalar,   U08,    91, "gear",    1.000, 0.000
    Out.Gear= Tuareg.process.Gear;

    //ground_speed     = scalar,   U16,    92, "mm/s",    1.000, 0.000
    Out.ground_speed_mmps= Tuareg.process.ground_speed_mmps;

    //consumpt_1s    = scalar,   U32,    94, "ug",  1.000, 0.000
    Out.conspumtion_ugps= Tuareg.fuel_consumpt_1s_ug;

    //EngineRunTime   = scalar,   U32,    98, "#",  1.000, 0.000
    Out.engine_runtime_ms= Tuareg.engine_runtime;

    //injDelay        = scalar,   U32,    103, "us",  1.000, 0.000
    Out.inj_delay_us= Tuareg.fueling_controls.injector_deadtime_us;

    //avgMAP          = scalar,   F32,    107, "kpa",    1.000, 0.000
    Out.avg_MAP_kPa= Tuareg.process.avg_MAP_kPa;


    /**
    send output channels
    */
    UART_send_data(Port, pOut_data, cOut_size);
}



/**
prepare OutputChannel "tuareg" field
*/
void ts_tuareg_bits(ts_tuareg_bits_t * pTarget)
{
    pTarget->run_inhibit= Tuareg.flags.run_inhibit;
    pTarget->crash_sensor_triggered= Tuareg.flags.crash_sensor_triggered;
    pTarget->run_switch_deactivated= Tuareg.flags.run_switch_deactivated;
    pTarget->sidestand_sensor_triggered= Tuareg.flags.sidestand_sensor_triggered;
    pTarget->overheat_detected= Tuareg.flags.overheat_detected;
    pTarget->service_mode= Tuareg.flags.service_mode;
    pTarget->limited_op= Tuareg.flags.limited_op;
    pTarget->rev_limiter= Tuareg.flags.rev_limiter;
    pTarget->standby= Tuareg.flags.standby;
    pTarget->cranking= Tuareg.flags.cranking;
    pTarget->fuel_pump= Tuareg.flags.fuel_pump;
    pTarget->mil= Tuareg.flags.mil;
    pTarget->syslog_update= Tuareg.flags.syslog_update;
    pTarget->datalog_update= Tuareg.flags.datalog_update;
    pTarget->highspeedlog_update= Tuareg.flags.highspeedlog_update;
    pTarget->fatal_error= Tuareg.errors.fatal_error;
    pTarget->decoder_config_error= Tuareg.errors.decoder_config_error;
    pTarget->ignition_config_error= Tuareg.errors.ignition_config_error;
    pTarget->tuareg_config_error= Tuareg.errors.tuareg_config_error;
    pTarget->fueling_config_error= Tuareg.errors.fueling_config_error;
    pTarget->sensor_calibration_error= Tuareg.errors.sensor_calibration_error;
    pTarget->sensor_O2_error= Tuareg.errors.sensor_O2_error;
    pTarget->sensor_TPS_error= Tuareg.errors.sensor_TPS_error;
    pTarget->sensor_IAT_error= Tuareg.errors.sensor_IAT_error;
    pTarget->sensor_CLT_error= Tuareg.errors.sensor_CLT_error;
    pTarget->sensor_VBAT_error= Tuareg.errors.sensor_VBAT_error;
    pTarget->sensor_KNOCK_error= Tuareg.errors.sensor_KNOCK_error;
    pTarget->sensor_BARO_error= Tuareg.errors.sensor_BARO_error;
    pTarget->sensor_GEAR_error= Tuareg.errors.sensor_GEAR_error;
    pTarget->sensor_MAP_error= Tuareg.errors.sensor_MAP_error;
    pTarget->sensor_CIS_error= Tuareg.errors.sensor_CIS_error;
}



