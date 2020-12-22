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

#define TS_DEBUG


void ts_sendOutputChannels(USART_TypeDef * Port)
{
    U8 output[TS_OCHBLOCKSIZE];

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
    serialize_U32_char((U32) ts_tuareg_bits(), &(output[1]));

    /*
    ignition         = scalar,  U08,    5, "bits",   1.000, 0.000
    */
    output[5] = (U8) ts_ignition_bits();

    /*
    comm             = scalar, U08,  6, "bits",   1.000, 0.000
    */
    output[6] = (U8) ts_comm_bits();

    //rpm              = scalar,   U16,    7, "rpm",    1.000, 0.000
    serialize_U16_U8(Tuareg.process.crank_rpm, &(output[7]));

    //rpmDOT           = scalar,   F32,    9, "rpm/s",  1.000, 0.000
    serialize_float_U8(0.42, &(output[9]));

    //advance          = scalar,   U16,    13, "deg",    1.000, 0.000
    serialize_U16_U8(Tuareg.ignition_controls.ignition_advance_deg, &(output[13]));

    //dwell	        = scalar,   U16,    15, "ms",     0.100, 0.00
    serialize_U16_U8(Tuareg.ignition_controls.dwell_ms_phased, &(output[15]));

    //map              = scalar,   F32,    17, "kpa",    1.000, 0.000
    serialize_float_U8(Tuareg.process.MAP_kPa, &(output[17]));

    //baro             = scalar,   F32,    21, "kpa",      1.000, 0.000
    serialize_float_U8(Tuareg.process.Baro_kPa, &(output[21]));

    //tps              = scalar,   F32,    25, "deg",      1.000, 0.000
    serialize_float_U8(Tuareg.process.TPS_deg, &(output[25]));

    //TPSdot           = scalar,   F32,    29, "deg/s",    10.00, 0.000
    serialize_float_U8(0.42, &(output[29]));

    //iat             = scalar,   F32,    33, "K",    1.000, 0.000
    serialize_float_U8(Tuareg.process.IAT_K, &(output[33]));

    //clt       = scalar,   F32,    37, "K",    1.000, 0.000
    serialize_float_U8(Tuareg.process.CLT_K, &(output[37]));

    //batteryVoltage   = scalar,   F32,    41, "V",      0.100, 0.000
    serialize_float_U8(Tuareg.process.VBAT_V, &(output[41]));

    //afr              = scalar,   F32,    45, "O2",     0.100, 0.000
    serialize_float_U8(14.5, &(output[45]));

    //afr 45, 46, 47, 48;


    /**
    print output channels
    */
    UART_send_data(Port, (volatile U8 * const) &output, TS_OCHBLOCKSIZE);

}



/**
prepare OutputChannel "comm" field
*/
BF8 ts_comm_bits()
{
    BF8 commbits =0;

    if(Tuareg_console.cli_permissions.burn_permission)
    {
        setBit_BF8(COMMBIT_BURN_PERMISSION, &commbits);
    }

    if(Tuareg_console.cli_permissions.calib_mod_permission)
    {
        setBit_BF8(COMMBIT_CALMOD_PERMISSION, &commbits);
    }

    if(Tuareg_console.cli_permissions.ignition_mod_permission)
    {
        setBit_BF8(COMMBIT_IGNMOD_PERMISSION, &commbits);
    }

    if(Tuareg_console.cli_permissions.decoder_mod_permission)
    {
        setBit_BF8(COMMBIT_DECMOD_PERMISSION, &commbits);
    }

    return commbits;
}


/**
prepare OutputChannel "tuareg" field

*/
BF32 ts_tuareg_bits()
{
    BF32 tuaregbits =0;

    if(Tuareg.Errors.config_load_error)
    {
        setBit_BF32(TBIT_CONFIGLOAD_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.scheduler_error)
    {
        setBit_BF32(TBIT_SCHEDULER_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.sensor_O2_error)
    {
        setBit_BF32(TBIT_O2SENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.sensor_TPS_error)
    {
        setBit_BF32(TBIT_TPSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.sensor_IAT_error)
    {
        setBit_BF32(TBIT_IATSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.sensor_CLT_error)
    {
        setBit_BF32(TBIT_CLTSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.sensor_VBAT_error)
    {
        setBit_BF32(TBIT_VBATSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.sensor_KNOCK_error)
    {
        setBit_BF32(TBIT_KNOCKSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.sensor_BARO_error)
    {
        setBit_BF32(TBIT_BAROSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.sensor_GEAR_error)
    {
        setBit_BF32(TBIT_GEARSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.sensor_MAP_error)
    {
        setBit_BF32(TBIT_MAPSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.sensor_CIS_error)
    {
        setBit_BF32(TBIT_CISENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.Runmode == TMODE_CRANKING)
    {
        setBit_BF32(TBIT_CRANKING_MODE, &tuaregbits);
    }
    else if(Tuareg.Runmode == TMODE_LIMP)
    {
        setBit_BF32(TBIT_LIMP_MODE, &tuaregbits);
    }
    else if(Tuareg.Runmode == TMODE_DIAG)
    {
        setBit_BF32(TBIT_DIAG_MODE, &tuaregbits);
    }

    return tuaregbits;
}


/**
prepare OutputChannel "ignition" field

*/
VU8 ts_ignition_bits()
{
    BF32 ignitionbits =0;

    if(Tuareg.ignition_controls.state.default_timing)
    {
        setBit_BF32(IGNBIT_DEFAULT_TIMING, &ignitionbits);
    }

    if(Tuareg.ignition_controls.state.cranking_timing)
    {
        setBit_BF32(IGNBIT_CRANKING_TIMING, &ignitionbits);
    }

    if(Tuareg.ignition_controls.state.rev_limiter)
    {
        setBit_BF32(IGNBIT_REV_LIMITER, &ignitionbits);
    }

    if(Tuareg.ignition_controls.state.dynamic)
    {
        setBit_BF32(IGNBIT_DYNAMIC, &ignitionbits);
    }

    if(Tuareg.ignition_controls.state.cold_idle)
    {
        setBit_BF32(IGNBIT_COLD_IDLE, &ignitionbits);
    }

    if(Tuareg.ignition_controls.state.advance_map)
    {
        setBit_BF32(IGNBIT_ADVANCE_MAP, &ignitionbits);
    }

    if(Tuareg.ignition_controls.state.advance_tps)
    {
        setBit_BF32(IGNBIT_ADVANCE_TPS, &ignitionbits);
    }

    return ignitionbits;
}

