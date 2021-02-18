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
//#include "debug.h"
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
    ignition         = scalar,  U16,    5, "bits",   1.000, 0.000
    */
    serialize_U16_U8(ts_ignition_bits(), &(output[5]));

    /*
    comm             = scalar, U08,  7, "bits",   1.000, 0.000
    */
    output[7] = (U8) ts_comm_bits();

    //rpm              = scalar,   U16,    8, "rpm",    1.000, 0.000
    if(Tuareg.pDecoder->outputs.rpm_valid == true)
    {
        serialize_U16_U8(Tuareg.pDecoder->crank_rpm, &(output[8]));
    }
    else
    {
        output[8]= 0;
        output[9]= 0;
    }

    //rpmDOT           = scalar,   F32,    10, "rpm/s",  1.000, 0.000
    if(Tuareg.pDecoder->outputs.accel_valid == true)
    {
        serialize_float_U8(0.42, &(output[10]));
    }
    else
    {
        output[10]= 0;
    }

    if(Tuareg.ignition_controls.state.valid == true)
    {
        //advance          = scalar,   U16,    14, "deg",    1.000, 0.000
        serialize_U16_U8(Tuareg.ignition_controls.ignition_advance_deg, &(output[14]));

        //dwell	        = scalar,   U16,    16, "ms",     0.100, 0.00
        serialize_U16_U8(Tuareg.ignition_controls.dwell_batch_us, &(output[16]));
    }
    else
    {
        //advance          = scalar,   U16,    14, "deg",    1.000, 0.000
        output[14]= 0;
        output[15]= 0;

        //dwell	        = scalar,   U16,    16, "ms",     0.100, 0.00
        output[16]= 0;
        output[17]= 0;
    }


    //map              = scalar,   F32,    18, "kpa",    1.000, 0.000
    serialize_float_U8(Tuareg.process.MAP_kPa, &(output[18]));

    //baro             = scalar,   F32,    22, "kpa",      1.000, 0.000
    serialize_float_U8(Tuareg.process.Baro_kPa, &(output[22]));

    //tps              = scalar,   F32,    26, "deg",      1.000, 0.000
    serialize_float_U8(Tuareg.process.TPS_deg, &(output[26]));

    //TPSdot           = scalar,   F32,    30, "deg/s",    10.00, 0.000
    serialize_float_U8(0.42, &(output[30]));

    //iat             = scalar,   F32,    34, "K",    1.000, 0.000
    serialize_float_U8(Tuareg.process.IAT_K, &(output[34]));

    //clt       = scalar,   F32,    38, "K",    1.000, 0.000
    serialize_float_U8(Tuareg.process.CLT_K, &(output[38]));

    //batteryVoltage   = scalar,   F32,    42, "V",      0.100, 0.000
    serialize_float_U8(Tuareg.process.VBAT_V, &(output[42]));

    //afr              = scalar,   F32,    46, "O2",     0.100, 0.000
    serialize_float_U8(Tuareg.process.O2_AFR, &(output[46]));

    //gear             = scalar,   U08,    50, "gear",    1.000, 0.000
    output[50]= Tuareg.process.Gear;

    //ground_speed     = scalar,   U08,    51, "kmh",    1.000, 0.000
    output[50]= Tuareg.process.ground_speed_kmh;


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

    //errors
    if(Tuareg.Errors.decoder_config_error)
    {
        setBit_BF32(TBIT_DECODERCONFIG_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.ignition_config_error)
    {
        setBit_BF32(TBIT_IGNITIONCONFIG_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.sensor_calibration_error)
    {
        setBit_BF32(TBIT_SENSORCALIB_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.tuareg_config_error)
    {
        setBit_BF32(TBIT_TUAREGCONFIG_ERROR, &tuaregbits);
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

    //runmode

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

    //halt sources

    if(Tuareg.Halt_source.crash_sensor)
    {
        setBit_BF32(TBIT_HSRC_CRASH, &tuaregbits);
    }

    if(Tuareg.Halt_source.run_switch)
    {
        setBit_BF32(TBIT_HSRC_RUN, &tuaregbits);
    }

    if(Tuareg.Halt_source.sidestand_sensor)
    {
        setBit_BF32(TBIT_HSRC_SIDESTAND, &tuaregbits);
    }

    //actors

    if(Tuareg.actors.ignition_inhibit)
    {
        setBit_BF32(TBIT_ACT_IGN_INH, &tuaregbits);
    }

    if(Tuareg.actors.fueling_inhibit)
    {
        setBit_BF32(TBIT_ACT_FUEL_INH, &tuaregbits);
    }

    if(Tuareg.actors.fuel_pump)
    {
        setBit_BF32(TBIT_ACT_FUEL_PUMP, &tuaregbits);
    }

    return tuaregbits;
}


/**
prepare OutputChannel "ignition" field
*/
VU16 ts_ignition_bits()
{
    BF32 ignitionbits =0;

    if(Tuareg.ignition_controls.state.valid)
    {
        setBit_BF32(IGNBIT_VALID, &ignitionbits);
    }

    if(Tuareg.ignition_controls.state.default_controls)
    {
        setBit_BF32(IGNBIT_DEFAULT_CTRL, &ignitionbits);
    }

    if(Tuareg.ignition_controls.state.cranking_controls)
    {
        setBit_BF32(IGNBIT_CRANKING_CTRL, &ignitionbits);
    }

    if(Tuareg.ignition_controls.state.dynamic_controls)
    {
        setBit_BF32(IGNBIT_DYNAMIC, &ignitionbits);
    }

    if(Tuareg.ignition_controls.state.rev_limiter)
    {
        setBit_BF32(IGNBIT_REV_LIMITER, &ignitionbits);
    }

    if(Tuareg.ignition_controls.state.sequential_mode)
    {
        setBit_BF32(IGNBIT_SEQ_MODE, &ignitionbits);
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

