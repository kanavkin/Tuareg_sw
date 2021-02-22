#ifndef TUAREG_ERRORS_H_INCLUDED
#define TUAREG_ERRORS_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"
#include "Tuareg_ID.h"

typedef union
{
     U32 all_flags;

     struct {
/// TODO (oli#9#): add fatal error flag?

        VU32 decoder_config_error :1;
        VU32 ignition_config_error :1;
        VU32 sensor_calibration_error :1;
        VU32 tuareg_config_error :1;

        VU32 sensor_O2_error :1;
        VU32 sensor_TPS_error :1;
        VU32 sensor_IAT_error :1;
        VU32 sensor_CLT_error :1;
        VU32 sensor_VBAT_error :1;
        VU32 sensor_KNOCK_error :1;
        VU32 sensor_BARO_error :1;
        VU32 sensor_GEAR_error :1;
        VU32 sensor_MAP_error :1;
        VU32 sensor_CIS_error :1;
     };

} tuareg_error_t;

extern void Assert(bool Condition, Tuareg_ID Id, U8 Location);
void Fatal(Tuareg_ID Id, U8 Location);


#endif // TUAREG_ERRORS_H_INCLUDED
