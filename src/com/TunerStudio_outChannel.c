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



#define TS_OCHBLOCKSIZE 102


void ts_sendOutputChannels(USART_TypeDef * Port)
{
    U8 output[TS_OCHBLOCKSIZE];
    ts_tuareg_bits_t Tuareg_bits;

    if(Tuareg_console.ts_getOutputChannels_count == 0)
    {
        Tuareg_console.secl = 0;
    }

    Tuareg_console.ts_getOutputChannels_count++;


    /*
    secl is simply a counter that increments each second. Used to track unexpected resets (Which will reset this count to 0)
    secl             = scalar, U08,  0, "sec",    1.000, 0.000
    */
    output[0] = Tuareg_console.secl;

    /*
    tuareg           = scalar, U32,  1, "bits",   1.000, 0.000
    */
    ts_tuareg_bits(&Tuareg_bits);
    serialize_U32_U8(Tuareg_bits.all_flags, &(output[1]));

    /*
    ignition         = scalar,  U08,    5, "bits",   1.000, 0.000
    */
    output[5]=Tuareg.ignition_controls.flags.all_flags;

    /*
    fueling         = scalar,  U16,    6, "bits",   1.000, 0.000
    */
    serialize_U16_U8(Tuareg.fueling_controls.flags.all_flags, &(output[6]));

    /*
    comm             = scalar, U08,  8, "bits",   1.000, 0.000
    */
    output[8] = Tuareg_console.cli_permissions.all_flags;

    //rpm              = scalar,   U16,    9, "rpm",    1.000, 0.000
    if(Tuareg.pDecoder->outputs.rpm_valid == true)
    {
        serialize_U16_U8(Tuareg.pDecoder->crank_rpm, &(output[9]));
    }
    else
    {
        output[9]= 0;
        output[10]= 0;
    }

    //rpmDOT           = scalar,   F32,    11, "rpm/s",  1.000, 0.000
    if(Tuareg.pDecoder->outputs.accel_valid == true)
    {
        serialize_float_U8(0.42, &(output[11]));
    }
    else
    {
        output[11]= 0;
    }


    //advance         = scalar,   U16,    15, "deg",    1.000, 0.000
    serialize_U16_U8(Tuareg.ignition_controls.ignition_advance_deg, &(output[15]));

    //dwell	        = scalar,   U16,    17, "us",     0.100, 0.00
    serialize_U16_U8(Tuareg.ignition_controls.dwell_us, &(output[17]));

    //VE              = scalar,   F32,    19, "%",  1.000, 0.000
    serialize_float_U8(Tuareg.fueling_controls.VE_pct, &(output[19]));

    //airDens         = scalar,   F32,    23, "ug/cm3",  1.000, 0.000
    serialize_float_U8(Tuareg.fueling_controls.air_density, &(output[23]));

    //BasefuelMass    = scalar,   U32,    27, "ug",  1.000, 0.000
    serialize_U32_U8(Tuareg.fueling_controls.base_fuel_mass_ug, &(output[27]));

    //TargetfuelMass  = scalar,   U32,    31, "ug",  1.000, 0.000
    serialize_U32_U8(Tuareg.fueling_controls.target_fuel_mass_ug, &(output[31]));

    //AFRtgt          = scalar,   F32,    35, "AFR",  1.000, 0.000
    serialize_float_U8(Tuareg.fueling_controls.AFR_target, &(output[35]));

    //inj1Iv          = scalar,   U32,    39, "us",  1.000, 0.000
    serialize_U32_U8(Tuareg.fueling_controls.injector1_interval_us, &(output[39]));

    //inj2Iv          = scalar,   U32,    43, "us",  1.000, 0.000
    serialize_U32_U8(Tuareg.fueling_controls.injector2_interval_us, &(output[43]));

    //injDcTgt        = scalar,   U32,    47, "us",  1.000, 0.000
    serialize_U32_U8(Tuareg.fueling_controls.injector_target_dc, &(output[47]));


    //MAP             = scalar,   F32,    51, "kpa",    1.000, 0.000
    serialize_float_U8(Tuareg.process.MAP_kPa, &(output[51]));

    //MAPdot          = scalar,   F32,    55, "kpa/s",    1.000, 0.000
    serialize_float_U8(Tuareg.process.ddt_MAP, &(output[55]));

    //baro            = scalar,   F32,    59, "kpa",      1.000, 0.000
    serialize_float_U8(Tuareg.process.Baro_kPa, &(output[59]));

    //TPS             = scalar,   F32,    63, "deg",      1.000, 0.000
    serialize_float_U8(Tuareg.process.TPS_deg, &(output[63]));

    //TPSdot          = scalar,   F32,    67, "deg/s",    10.00, 0.000
    serialize_float_U8(Tuareg.process.ddt_TPS, &(output[67]));

    //IAT             = scalar,   F32,    71, "K",    1.000, -273.15
    serialize_float_U8(Tuareg.process.IAT_K, &(output[71]));

    //CLT             = scalar,   F32,    75, "K",    1.000, -273.15
    serialize_float_U8(Tuareg.process.CLT_K, &(output[75]));

    //battery         = scalar,   F32,    79, "V",      1.000, 0.000
    serialize_float_U8(Tuareg.process.VBAT_V, &(output[79]));

    //AFR             = scalar,   F32,    83, "O2",     1.000, 0.000
    serialize_float_U8(Tuareg.process.O2_AFR, &(output[83]));

    //knock level     = scalar,   F32,    87, "Knock",     1.000, 0.000
    serialize_float_U8(Tuareg.process.Knock_level, &(output[87]));

    //gear             = scalar,   U08,    91, "gear",    1.000, 0.000
    output[91]= Tuareg.process.Gear;

    //ground_speed     = scalar,   U16,    92, "mm/s",    1.000, 0.000
    serialize_U16_U8(Tuareg.process.ground_speed_mmps, &(output[92]));

    //consumpt_1s    = scalar,   U32,    94, "ug",  1.000, 0.000
    serialize_U32_U8(Tuareg.fuel_consumpt_1s_ug, &(output[94]));

    //EngineRunTime   = scalar,   U32,    98, "#",  1.000, 0.000
    serialize_U32_U8(Tuareg.engine_runtime, &(output[98]));

    // .. 99, 100, 101

    // -> size counts from 0 to this last value !!! := 102

    /**
    print output channels
    */
    UART_send_data(Port, (volatile U8 * const) &output, TS_OCHBLOCKSIZE);

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



