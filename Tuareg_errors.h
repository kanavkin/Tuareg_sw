#ifndef TUAREG_ERRORS_H_INCLUDED
#define TUAREG_ERRORS_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"
#include "Tuareg_ID.h"

typedef union
{
     U32 all_flags;

     struct {

        VU32 config_load_error :1;
        VU32 scheduler_error :1;
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

void Tuareg_Assert(bool Condition, Tuareg_ID Id, U32 Location);
void Tuareg_Fatal(Tuareg_ID Id, U32 Location);
void Tuareg_register_scheduler_error();


#endif // TUAREG_ERRORS_H_INCLUDED
