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



#define TS_OCHBLOCKSIZE 86


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


    if(Tuareg.ignition_controls.flags.valid == true)
    {
        //advance         = scalar,   U16,    16, "deg",    1.000, 0.000
        serialize_U16_U8(Tuareg.ignition_controls.ignition_advance_deg, &(output[16]));

        //dwell	        = scalar,   U16,    18, "ms",     0.100, 0.00
        serialize_U16_U8(Tuareg.ignition_controls.dwell_us, &(output[18]));
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

        //BasefuelMass    = scalar,   U32,    28, "ug",  1.000, 0.000
        serialize_U32_U8(Tuareg.fueling_controls.base_fuel_mass_ug, &(output[28]));

        //TargetfuelMass  = scalar,   U32,    32, "ug",  1.000, 0.000
        serialize_U32_U8(Tuareg.fueling_controls.target_fuel_mass_ug, &(output[32]));

        //AFRtgt          = scalar,   F32,    36, "AFR",  1.000, 0.000
        serialize_float_U8(Tuareg.fueling_controls.AFR_target, &(output[36]));

        //inj1Iv          = scalar,   U32,    40, "us",  1.000, 0.000
        serialize_U32_U8(Tuareg.fueling_controls.injector1_interval_us, &(output[40]));

        //inj2Iv          = scalar,   U32,    44, "us",  1.000, 0.000
        serialize_U32_U8(Tuareg.fueling_controls.injector2_interval_us, &(output[44]));

        //injDcTgt        = scalar,   U32,    48, "us",  1.000, 0.000
        serialize_U32_U8(Tuareg.fueling_controls.injector_target_dc, &(output[48]));
    }
    else
    {
        for(i=20; i< 52; i++)
        {
            output[i]= 0;
        }
    }

    //MAP             = scalar,   F32,    52, "kpa",    1.000, 0.000
    serialize_float_U8(Tuareg.process.MAP_kPa, &(output[52]));

    //baro            = scalar,   F32,    56, "kpa",      1.000, 0.000
    serialize_float_U8(Tuareg.process.Baro_kPa, &(output[56]));

    //TPS             = scalar,   F32,    60, "deg",      1.000, 0.000
    serialize_float_U8(Tuareg.process.TPS_deg, &(output[60]));

    //TPSdot          = scalar,   F32,    64, "deg/s",    10.00, 0.000
    serialize_float_U8(Tuareg.process.ddt_TPS, &(output[64]));

    //IAT             = scalar,   F32,    68, "K",    1.000, -273.15
    serialize_float_U8(Tuareg.process.IAT_K, &(output[68]));

    //CLT             = scalar,   F32,    72, "K",    1.000, -273.15
    serialize_float_U8(Tuareg.process.CLT_K, &(output[72]));

    //battery         = scalar,   F32,    76, "V",      1.000, 0.000
    serialize_float_U8(Tuareg.process.VBAT_V, &(output[76]));

    //AFR             = scalar,   F32,    80, "O2",     1.000, 0.000
    serialize_float_U8(Tuareg.process.O2_AFR, &(output[80]));


    //gear             = scalar,   U08,    84, "gear",    1.000, 0.000
    output[84]= Tuareg.process.Gear;

    //ground_speed     = scalar,   U08,    85, "kmh",    1.000, 0.000
    output[85]= Tuareg.process.ground_speed_kmh;

    //size = 86

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

    //flags
    if(Tuareg.errors.decoder_config_error)
    {
        setBit_BF32(TBIT_DECODERCONFIG_ERROR, &tuaregbits);
    }

    if(Tuareg.errors.ignition_config_error)
    {
        setBit_BF32(TBIT_IGNITIONCONFIG_ERROR, &tuaregbits);
    }

    if(Tuareg.errors.fueling_config_error)
    {
        setBit_BF32(TBIT_FUELCONFIG_ERROR, &tuaregbits);
    }

    if(Tuareg.errors.sensor_calibration_error)
    {
        setBit_BF32(TBIT_SENSORCALIB_ERROR, &tuaregbits);
    }

    if(Tuareg.errors.tuareg_config_error)
    {
        setBit_BF32(TBIT_TUAREGCONFIG_ERROR, &tuaregbits);
    }


    if(Tuareg.errors.sensor_O2_error)
    {
        setBit_BF32(TBIT_O2SENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.errors.sensor_TPS_error)
    {
        setBit_BF32(TBIT_TPSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.errors.sensor_IAT_error)
    {
        setBit_BF32(TBIT_IATSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.errors.sensor_CLT_error)
    {
        setBit_BF32(TBIT_CLTSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.errors.sensor_VBAT_error)
    {
        setBit_BF32(TBIT_VBATSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.errors.sensor_KNOCK_error)
    {
        setBit_BF32(TBIT_KNOCKSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.errors.sensor_BARO_error)
    {
        setBit_BF32(TBIT_BAROSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.errors.sensor_GEAR_error)
    {
        setBit_BF32(TBIT_GEARSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.errors.sensor_MAP_error)
    {
        setBit_BF32(TBIT_MAPSENSOR_ERROR, &tuaregbits);
    }

    if(Tuareg.errors.sensor_CIS_error)
    {
        setBit_BF32(TBIT_CISENSOR_ERROR, &tuaregbits);
    }

    //runmode

    if(Tuareg.flags.cranking == true)
    {
        setBit_BF32(TBIT_CRANKING_MODE, &tuaregbits);
    }

    if(Tuareg.flags.limited_op == true)
    {
        setBit_BF32(TBIT_LIMP_MODE, &tuaregbits);
    }

    if(Tuareg.flags.service_mode == true)
    {
        setBit_BF32(TBIT_DIAG_MODE, &tuaregbits);
    }

    //halt sources

    if(Tuareg.flags.crash_sensor_triggered)
    {
        setBit_BF32(TBIT_HSRC_CRASH, &tuaregbits);
    }

    if(Tuareg.flags.run_switch_deactivated)
    {
        setBit_BF32(TBIT_HSRC_RUN, &tuaregbits);
    }

    if(Tuareg.flags.sidestand_sensor_triggered)
    {
        setBit_BF32(TBIT_HSRC_SIDESTAND, &tuaregbits);
    }

    //actors

    if(Tuareg.flags.ignition_inhibit)
    {
        setBit_BF32(TBIT_ACT_IGN_INH, &tuaregbits);
    }

    if(Tuareg.flags.fueling_inhibit)
    {
        setBit_BF32(TBIT_ACT_FUEL_INH, &tuaregbits);
    }

    if(Tuareg.flags.fuel_pump)
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

    if(Tuareg.ignition_controls.flags.valid)
    {
        setBit_BF32(IGNBIT_VALID, &ignitionbits);
    }

    if(Tuareg.ignition_controls.flags.dynamic_controls)
    {
        setBit_BF32(IGNBIT_DYNAMIC, &ignitionbits);
    }



    if(Tuareg.ignition_controls.flags.sequential_mode)
    {
        setBit_BF32(IGNBIT_SEQ_MODE, &ignitionbits);
    }

    if(Tuareg.ignition_controls.flags.cold_idle)
    {
        setBit_BF32(IGNBIT_COLD_IDLE, &ignitionbits);
    }

    if(Tuareg.ignition_controls.flags.advance_map)
    {
        setBit_BF32(IGNBIT_ADVANCE_MAP, &ignitionbits);
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

    if(Tuareg.fueling_controls.flags.accel_comp_active)
    {
        setBit_BF32(FUELBIT_ACCELCOMP_ACT, &fuelingbits);
    }

    if(Tuareg.fueling_controls.flags.warmup_comp_active)
    {
        setBit_BF32(FUELBIT_WARMUPCOMP_ACT, &fuelingbits);
    }

    if(Tuareg.fueling_controls.flags.afterstart_comp_active)
    {
        setBit_BF32(FUELBIT_AFTERSTARTCOMP_ACT, &fuelingbits);
    }

    return fuelingbits;
}

