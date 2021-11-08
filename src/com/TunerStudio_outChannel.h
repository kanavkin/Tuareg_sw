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
        U32 run_inhibit :1;
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




typedef struct __attribute__ ((__packed__)) _Output_Channels_t {

    U8 secl;
    ts_tuareg_bits_t tuareg_bits;
    U8 ignition_bits;
    U16 fueling_bits;
    U8 com_bits;

    U16 rpm;
    F32 ddt_rpm;

    U16 ignition_adv_deg;
    U16 ignition_dwell_us;

    F32 VE_pct;
    F32 air_dens;
    F32 air_rate_gps;
    U32 base_fuel_mass_ug;
    U32 target_fuel_mass_ug;

    F32 target_AFR;

    U32 inj1_interval_us;
    U32 inj2_interval_us;
    U32 inj_dc_pct;

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

    gears_t Gear;
    U16 ground_speed_mmps;

    U32 conspumtion_ugps;
    U32 engine_runtime_ms;

    U32 inj_delay_us;
    F32 avg_MAP_kPa;


} Output_Channels_t;







void ts_sendOutputChannels(USART_TypeDef * Port);

void ts_sendOutputChannels_new(USART_TypeDef * Port);

void ts_tuareg_bits(ts_tuareg_bits_t * pTarget);


#endif // TUNERSTUDIO_OUTCHANNEL_H_INCLUDED
