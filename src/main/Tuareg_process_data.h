#ifndef TUAREGPDATA_H_INCLUDED
#define TUAREGPDATA_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"


typedef struct _process_data_t {

    VF32 MAP_kPa;
    VF32 Baro_kPa;
    VF32 TPS_deg;
    VF32 ddt_TPS;
    VF32 IAT_K;
    VF32 CLT_K;
    VF32 VBAT_V;
    VF32 O2_AFR;

    gears_t Gear;

    VU8 ground_speed_kmh;


} process_data_t;


extern void Tuareg_update_process_data();




#endif // TUAREGPDATA_H_INCLUDED
