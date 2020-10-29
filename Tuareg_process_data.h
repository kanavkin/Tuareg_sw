#ifndef TUAREGPDATA_H_INCLUDED
#define TUAREGPDATA_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"
#include "trigger_wheel_layout.h"


typedef struct _process_data_t {

    /*
    crank data
    */
    volatile crank_position_t crank_position;
    volatile crank_position_table_t crank_position_table;
    VU32 crank_T_us;
    VU32 crank_rpm;
    VU32 ddt_crank_rpms;

    volatile ctrl_strategy_t ctrl_strategy;

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




#endif // TUAREGPDATA_H_INCLUDED
