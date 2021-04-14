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




void ts_sendOutputChannels(USART_TypeDef * Port);

void ts_tuareg_bits(ts_tuareg_bits_t * pTarget);


#endif // TUNERSTUDIO_OUTCHANNEL_H_INCLUDED
