#ifndef TUAREGPDATA_H_INCLUDED
#define TUAREGPDATA_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"


#define TUAREG_DEFAULT_LOAD_PCT 30



typedef struct _process_data_t {

    F32 MAP_kPa;
    F32 ddt_MAP;
    F32 Baro_kPa;
    F32 TPS_deg;
    F32 ddt_TPS;
    F32 IAT_K;
    F32 CLT_K;
    F32 VBAT_V;
    F32 O2_AFR;
    F32 Knock_level;

    gears_t Gear;

    F32 speed_kmh;

    F32 load_pct;
    F32 IVT_K;

    U32 engine_runtime;

    /*
    fuel consumption data
    */
    U32 fuel_mass_integrator_1s_ug;
    F32 fuel_mass_integrator_1min_mg;
    U32 trip_integrator_1min_mm;
    U32 consumption_counter;

    F32 fuel_rate_gps;
    F32 fuel_eff_mpg;

} process_data_t;


typedef struct _process_data_memory_t {

    F32 last_MAP_kPa;
    F32 last_ddt_MAP;

    F32 last_TPS_deg;
    F32 last_ddt_TPS;

    F32 last_load_pct;

} process_data_memory_t;



void Tuareg_update_process_data();

//void Tuareg_update_load(volatile process_data_t * pProcess);

#endif // TUAREGPDATA_H_INCLUDED
