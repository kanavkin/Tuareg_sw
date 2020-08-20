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
    VU32 engine_rpm;

    volatile ctrl_strategy_t ctrl_strategy;

    VU32 MAP_kPa;
    VU32 Baro_kPa;
    VU32 TPS_deg;
    VS32 ddt_TPS;
    VU32 IAT_C;
    VU32 CLT_C;
    VU32 VBAT_V;


} process_data_t;




#endif // TUAREGPDATA_H_INCLUDED
