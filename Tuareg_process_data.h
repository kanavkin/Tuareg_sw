#ifndef TUAREGPDATA_H_INCLUDED
#define TUAREGPDATA_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"


typedef struct _process_data_t {

    U16 crank_rpm;

    VF32 MAP_kPa;
    VF32 Baro_kPa;
    VF32 TPS_deg;
    VF32 ddt_TPS;
    VF32 IAT_K;
    VF32 CLT_K;
    VF32 VBAT_V;
    VF32 O2_AFR;
    VU8 Gear;

} process_data_t;


extern void Tuareg_update_process_data();


VF32 Tuareg_update_MAP_sensor();
VF32 Tuareg_update_GEAR_sensor();
VF32 Tuareg_update_BARO_sensor();
VF32 Tuareg_update_KNOCK_sensor();
VF32 Tuareg_update_VBAT_sensor();
VF32 Tuareg_update_CLT_sensor();
VF32 Tuareg_update_IAT_sensor();
VF32 Tuareg_update_TPS_sensor();
VF32 Tuareg_update_ddt_TPS();
VF32 Tuareg_update_O2_sensor();
VF32 Tuareg_update_MAP_sensor();


#endif // TUAREGPDATA_H_INCLUDED
