#ifndef TUAREGPDATA_H_INCLUDED
#define TUAREGPDATA_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"


#define TUAREG_DEFAULT_LOAD_PCT 30



typedef struct _process_data_t {

    VF32 MAP_kPa;
    VF32 ddt_MAP;
    VF32 Baro_kPa;
    VF32 TPS_deg;
    VF32 ddt_TPS;
    VF32 IAT_K;
    VF32 CLT_K;
    VF32 VBAT_V;
    VF32 O2_AFR;
    VF32 Knock_level;

    gears_t Gear;

    VU32 ground_speed_mmps;

    VF32 load_pct;
    VF32 IVT_K;

} process_data_t;


typedef struct _process_data_memory_t {

    VF32 last_MAP_kPa;
    VF32 last_ddt_MAP;

    VF32 last_TPS_deg;
    VF32 last_ddt_TPS;

    VF32 last_load_pct;

} process_data_memory_t;



void Tuareg_update_process_data();

void Tuareg_update_load(volatile process_data_t * pProcess);

#endif // TUAREGPDATA_H_INCLUDED
