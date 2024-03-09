#ifndef TUNERSTUDIO_OUTCHANNEL_H_INCLUDED
#define TUNERSTUDIO_OUTCHANNEL_H_INCLUDED

#include "uart.h"

/**
ts_tuareg_bits_t
*/
typedef union
{
     U32 all_flags;

    struct
    {
        U32 run_allow :1;
        U32 crash_sensor_triggered :1;
        U32 run_switch_deactivated :1;
        U32 sidestand_sensor_triggered :1;
        U32 overheat_detected :1;

        U32 service_mode :1;
        U32 limited_op :1;
        U32 rev_limiter :1;
        U32 standby :1;
        U32 cranking :1;

        U32 fuel_pump :1;
        U32 mil :1;

        U32 syslog_update :1;
        U32 datalog_update :1;
        U32 highspeedlog_update :1;

        U32 fatal_error :1;

        U32 decoder_config_error :1;
        U32 ignition_config_error :1;
        U32 tuareg_config_error :1;
        U32 fueling_config_error :1;
        U32 sensor_calibration_error :1;

        U32 sensor_O2_error :1;
        U32 sensor_TPS_error :1;
        U32 sensor_IAT_error :1;
        U32 sensor_CLT_error :1;
        U32 sensor_VBAT_error :1;
        U32 sensor_KNOCK_error :1;
        U32 sensor_BARO_error :1;
        U32 sensor_GEAR_error :1;
        U32 sensor_MAP_error :1;
        U32 sensor_CIS_error :1;
     };

} ts_tuareg_bits_t;


/**
ts_control_bits_t
*/
typedef union
{
     U32 all_flags;

    struct
    {
        //control strategy
        U32 ctrl_SPD :1;
        U32 ctrl_trans :1;
        U32 ctrl_AFR_fb :1;
        U32 ctrl_val :1;

        //ignition
        U32 ign_val :1;
        U32 ign_dyn :1;
        U32 ign_seq :1;
        U32 ign_cld_idl :1;

        //fueling
        U32 fue_val :1;
        U32 fue_dry_crk :1;
        U32 fue_seq :1;
        U32 fue_dc_clip :1;

        U32 fue_WUE :1;
        U32 fue_ASE :1;
        U32 fue_BARO :1;
        U32 fue_AE :1;
        U32 fue_AE_MAP_acc :1;
        U32 fue_AE_MAP_dec :1;
        U32 fue_AE_TPS_acc :1;
        U32 fue_AE_TPS_dec :1;
        U32 fue_load_trans :1;

        U32 ctrl_set :2;

     };

} ts_control_bits_t;




typedef struct __attribute__ ((__packed__)) _Output_Channels_t {

    U8 secl;

    ts_tuareg_bits_t tuareg_bits;
    cli_permission_flags_t com_bits;

    U16 rpm;
    F32 ddt_rpm;

    F32 MAP_kPa;
    F32 ddt_MAP;
    F32 BARO_kPa;
    F32 TPS_deg;
    F32 ddt_TPS;
    F32 IAT_K;
    F32 CLT_K;
    F32 BAT_V;
    F32 AFR;
    F32 Knock;
    F32 IVT_K;

    gears_t Gear;
    U8 speed_kmh;
    U32 engine_runtime_ms;

    ts_control_bits_t control_bits;

    //control set data
    F32 IgnAdv_deg;
    F32 VE_pct;
    F32 AFRtgt;

    //ignition
    F32 dwell_us;

    //fueling
    F32 charge_temp_K;
    F32 air_rate_gps;

    F32 base_fuel_mass_ug;
    F32 target_fuel_mass_ug;
    F32 cmd_fuel_mass_ug;
    F32 wall_fuel_mass_ug;

    U32 inj1_interval_us;
    U32 inj2_interval_us;
    U32 inj_delay_us;
    F32 inj_dc_pct;


    F32 fuel_rate_gps;
    F32 fuel_eff_mpg;

} Output_Channels_t;



void ts_sendOutputChannels(USART_TypeDef * Port);


#endif // TUNERSTUDIO_OUTCHANNEL_H_INCLUDED
