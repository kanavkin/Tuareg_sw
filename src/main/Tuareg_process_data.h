#ifndef TUAREGPDATA_H_INCLUDED
#define TUAREGPDATA_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"


#define TUAREG_DEFAULT_LOAD_PCT 30



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

    VF32 load_pct;


} process_data_t;


void Tuareg_update_process_data();

void Tuareg_update_load(volatile process_data_t * pProcess);


#endif // TUAREGPDATA_H_INCLUDED
