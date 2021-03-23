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
    U32 i;

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
    serialize_U32_U8((U32) ts_tuareg_bits(), &(output[1]));

    /*
    ignition         = scalar,  U16,    5, "bits",   1.000, 0.000
    */
    serialize_U16_U8(ts_ignition_bits(), &(output[5]));

    /*
    fueling         = scalar,  U16,    7, "bits",   1.000, 0.000
    */
    serialize_U16_U8(ts_fueling_bits(), &(output[7]));

    /*
    comm             = scalar, U08,  9, "bits",   1.000, 0.000
    */
    output[9] = (U8) ts_comm_bits();

    //rpm              = scalar,   U16,    10, "rpm",    1.000, 0.000
    if(Tuareg.pDecoder->outputs.rpm_valid == true)
    {
        serialize_U16_U8(Tuareg.pDecoder->crank_rpm, &(output[10]));
    }
    else
    {
        output[10]= 0;
        output[11]= 0;
    }

    //rpmDOT           = scalar,   F32,    12, "rpm/s",  1.000, 0.000
    if(Tuareg.pDecoder->outputs.accel_valid == true)
    {
        serialize_float_U8(0.42, &(output[12]));
    }
    else
    {
        output[12]= 0;
    }


    if(Tuareg.ignition_controls.state.valid == true)
    {
        //advance         = scalar,   U16,    16, "deg",    1.000, 0.000
        serialize_U16_U8(Tuareg.ignition_controls.ignition_advance_deg, &(output[16]));

        //dwell	        = scalar,   U16,    18, "ms",     0.100, 0.00
        serialize_U16_U8(Tuareg.ignition_controls.dwell_batch_us, &(output[18]));
    }
    else
    {
        for(i=16; i< 20; i++)
        {
            output[i]= 0;
        }
    }

    if(Tuareg.fueling_controls.flags.valid == true)
    {
        //VE              = scalar,   F32,    20, "%",  1.000, 0.000
        serialize_float_U8(Tuareg.fueling_controls.VE_pct, &(output[20]));

        //airDens         = scalar,   F32,    24, "ug/cm3",  1.000, 0.000
        serialize_float_U8(Tuareg.fueling_controls.air_density, &(output[24]));

        //fuelMass        = scalar,   U32,    28, "ug",  1.000, 0.000
        serialize_U32_U8(Tuareg.fueling_controls.fuel_mass_ug, &(output[28]));

        //AFRtgt          = scalar,   F32,    32, "AFR",  1.000, 0.000
        serialize_float_U8(Tuareg.fueling_controls.AFR_target, &(output[32]));

        //inj1Iv          = scalar,   U32,    36, "us",  1.000, 0.000
        serialize_U32_U8(Tuareg.fueling_controls.injector1_interval_us, &(output[36]));

        //inj2Iv          = scalar,   U32,    40, "us",  1.000, 0.000
        serialize_U32_U8(Tuareg.fueling_controls.injector2_interval_us, &(output[40]));
    }
    else
    {
        for(i=20; i< 44; i++)
        {
            output[i]= 0;
        }
    }

    //MAP             = scalar,   F32,    44, "kpa",    1.000, 0.000
    serialize_float_U8(Tuareg.process.MAP_kPa, &(output[44]));

    //baro            = scalar,   F32,    48, "kpa",      1.000, 0.000
    serialize_float_U8(Tuareg.process.Baro_kPa, &(output[48]));

    //TPS             = scalar,   F32,    52, "deg",      1.000, 0.000
    serialize_float_U8(Tuareg.process.TPS_deg, &(output[52]));

    //TPSdot          = scalar,   F32,    56, "deg/s",    10.00, 0.000
    serialize_float_U8(0.42, &(output[56]));

    //IAT             = scalar,   F32,    60, "K",    1.000, -273.15
    serialize_float_U8(Tuareg.process.IAT_K, &(output[60]));

    //CLT             = scalar,   F32,    64, "K",    1.000, -273.15
    serialize_float_U8(Tuareg.process.CLT_K, &(output[64]));

    //battery         = scalar,   F32,    68, "V",      1.000, 0.000
    serialize_float_U8(Tuareg.process.VBAT_V, &(output[68]));

    //AFR             = scalar,   F32,    72, "O2",     1.000, 0.000
    serialize_float_U8(Tuareg.process.O2_AFR, &(output[72]));


    //gear             = scalar,   U08,    76, "gear",    1.000, 0.000
    output[76]= Tuareg.process.Gear;

    //ground_speed     = scalar,   U08,    77, "kmh",    1.000, 0.000
    output[77]= Tuareg.process.ground_speed_kmh;


    /**
    print output channels
    */
    UART_send_data(Port, (volatile U8 * const) &output, TS_OCHBLOCKSIZE);

}



/**
prepare OutputChannel "comm" field
*/
volatile BF8 ts_comm_bits()
{
    volatile BF8 commbits =0;

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

    if(Tuareg_console.cli_permissions.fueling_mod_permission)
    {
        setBit_BF8(COMMBIT_FUELMOD_PERMISSION, &commbits);
    }

    if(Tuareg_console.cli_permissions.decoder_mod_permission)
    {
        setBit_BF8(COMMBIT_DECMOD_PERMISSION, &commbits);
    }

    if(Tuareg.pSyslog->syslog_new_entry)
    {
        setBit_BF8(COMMBIT_SYSLOG_UPDATE, &commbits);
    }

    if(Tuareg.pSyslog->datalog_new_entry)
    {
        setBit_BF8(COMMBIT_DATALOG_UPDATE, &commbits);
    }

    if(Tuareg.pHighspeedlog->log_full)
    {
        setBit_BF8(COMMBIT_HSPDLOG_FULL, &commbits);
    }

    return commbits;
}


/**
prepare OutputChannel "tuareg" field

*/
volatile BF32 ts_tuareg_bits()
{
    volatile BF32 tuaregbits =0;

    //errors
    if(Tuareg.Errors.decoder_config_error)
    {
        setBit_BF32(TBIT_DECODERCONFIG_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.ignition_config_error)
    {
        setBit_BF32(TBIT_IGNITIONCONFIG_ERROR, &tuaregbits);
    }

    if(Tuareg.Errors.fueling_config_error)
    {
        setBit_BF32(TBIT_FUELCONFIG_ERROR, &tuaregbits);
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
    else if(Tuareg.Runmode == TMODE_SERVICE)
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
    volatile BF32 ignitionbits =0;

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


/**
prepare OutputChannel "fueling" field
*/
VU16 ts_fueling_bits()
{
    volatile BF32 fuelingbits =0;

    if(Tuareg.fueling_controls.flags.valid)
    {
        setBit_BF32(FUELBIT_VALID, &fuelingbits);
    }

    if(Tuareg.fueling_controls.flags.sequential_mode)
    {
        setBit_BF32(FUELBIT_SEQ_MODE, &fuelingbits);
    }

    if(Tuareg.fueling_controls.flags.VE_valid)
    {
        setBit_BF32(FUELBIT_VE_VALID, &fuelingbits);
    }

    if(Tuareg.fueling_controls.flags.VE_from_MAP)
    {
        setBit_BF32(FUELBIT_VE_MAP, &fuelingbits);
    }

    if(Tuareg.fueling_controls.flags.AFR_target_valid)
    {
        setBit_BF32(FUELBIT_AFR_VALID, &fuelingbits);
    }

    if(Tuareg.fueling_controls.flags.injector_dc_clip)
    {
        setBit_BF32(FUELBIT_DC_CLIP, &fuelingbits);
    }

    if(Tuareg.fueling_controls.flags.injection_begin_valid)
    {
        setBit_BF32(FUELBIT_BEGIN_VALID, &fuelingbits);
    }

    return fuelingbits;
}

