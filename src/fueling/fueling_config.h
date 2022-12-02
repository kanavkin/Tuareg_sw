#ifndef FUELING_CONFIG_H_INCLUDED
#define FUELING_CONFIG_H_INCLUDED

#include "Tuareg_fueling.h"


/**
fueling_features_t
memory order : first entry on rightmost bit
*/
typedef union
{
     U8 all_flags;

     struct
     {
        U8 load_transient_comp_enabled :1;
        U8 legacy_AE_enabled :1;
        U8 sequential_mode_enabled :1;
        U8 WUE_enabled :1;
        U8 ASE_enabled :1;
        U8 dry_cranking_enabled :1;
        U8 BARO_correction_enabled :1;
     };

} fueling_features_t;


/***************************************************************************************************************************************************
*   Fueling Setup Page
***************************************************************************************************************************************************/

typedef struct __attribute__ ((__packed__)) _Fueling_Setup_t {

    U8 Version;

    /*
    engine parameters
    */
    U16 cylinder_volume_ccm;

    //injection alignment
    crank_position_t injection_reference_pos;

    //injector parameters
    U16 injector1_rate_mgps;
    U16 injector2_rate_mgps;
    U8 max_injector_duty_cycle_pct;

    //legacy AE
    F32 accel_comp_thres_TPS_low;
    F32 accel_comp_thres_TPS_high;
    F32 accel_comp_thres_MAP_low;
    F32 accel_comp_thres_MAP_high;
    U16 accel_comp_thres_rpm;
    F32 decel_comp_thres_TPS;
    F32 decel_comp_thres_MAP;

    F32 accel_comp_taper_factor;
    U16 accel_comp_scaling_thres_rpm;
    U16 accel_comp_scaling_max_rpm;

    U8 cold_accel_pct;

    U16 decel_comp_ug;
    U16 decel_min_rpm;

    U8 accel_comp_cycles;
    U8 decel_comp_cycles;

    U8 accel_comp_taper_thres;

    //after start compensation
    U8 afterstart_comp_pct;
    U16 afterstart_comp_cycles;
    U16 afterstart_thres_K;

    //VE base lookup parameters
    U16 spd_min_rpm;
    U16 spd_max_rpm;

    //dry cranking
    U8 dry_cranking_TPS_thres;

    fueling_features_t features;

} Fueling_Setup_t;


/***************************************************************************************************************************************************
*   API section
***************************************************************************************************************************************************/

extern volatile Fueling_Setup_t Fueling_Setup;

//General Fueling Config
exec_result_t load_Fueling_Config();
void load_essential_Fueling_Config();


//Fueling Setup
exec_result_t load_Fueling_Setup();
exec_result_t store_Fueling_Setup();
void show_Fueling_Setup();
exec_result_t modify_Fueling_Setup(U32 Offset, U32 Value);
void send_Fueling_Setup(USART_TypeDef * Port);


//Fueling VE Table (TPS) - VeTable_TPS
exec_result_t load_VeTable_TPS();
exec_result_t store_VeTable_TPS();
void show_VeTable_TPS(USART_TypeDef * Port);
exec_result_t modify_VeTable_TPS(U32 Offset, U32 Value);
void send_VeTable_TPS(USART_TypeDef * Port);
F32 getValue_VeTable_TPS(U32 Rpm, F32 Tps_deg);

//Fueling VE Table (MAP based) - VeTable_MAP
exec_result_t load_VeTable_MAP();
exec_result_t store_VeTable_MAP();
void show_VeTable_MAP(USART_TypeDef * Port);
exec_result_t modify_VeTable_MAP(U32 Offset, U32 Value);
void send_VeTable_MAP(USART_TypeDef * Port);
F32 getValue_VeTable_MAP(U32 Rpm, F32 Map_kPa);

//Fueling AFR target Table (TPS based) - AfrTable_TPS
exec_result_t load_AfrTable_TPS();
exec_result_t store_AfrTable_TPS();
void show_AfrTable_TPS(USART_TypeDef * Port);
exec_result_t modify_AfrTable_TPS(U32 Offset, U32 Value);
void send_AfrTable_TPS(USART_TypeDef * Port);
F32 getValue_AfrTable_TPS(U32 Rpm, F32 Tps_deg);

//Fueling AFR target Table (MAP based) - AfrTable_MAP
exec_result_t load_AfrTable_MAP();
exec_result_t store_AfrTable_MAP();
void show_AfrTable_MAP(USART_TypeDef * Port);
exec_result_t modify_AfrTable_MAP(U32 Offset, U32 Value);
void send_AfrTable_MAP(USART_TypeDef * Port);
F32 getValue_AfrTable_MAP(U32 Rpm, F32 Map_kPa);

//Fueling acceleration compensation table TPS based - AccelCompTableTPS
exec_result_t load_AccelCompTableTPS();
exec_result_t store_AccelCompTableTPS();
void show_AccelCompTableTPS(USART_TypeDef * Port);
exec_result_t modify_AccelCompTableTPS(U32 Offset, U32 Value);
void send_AccelCompTableTPS(USART_TypeDef * Port);
F32 getValue_AccelCompTableTPS(F32 Ddt_TPS);

//Fueling acceleration compensation table MAP based - AccelCompTableMAP
exec_result_t load_AccelCompTableMAP();
exec_result_t store_AccelCompTableMAP();
void show_AccelCompTableMAP(USART_TypeDef * Port);
exec_result_t modify_AccelCompTableMAP(U32 Offset, U32 Value);
void send_AccelCompTableMAP(USART_TypeDef * Port);
F32 getValue_AccelCompTableMAP(F32 Ddt_MAP);

//Fueling Warm up Enrichment compensation table - WarmUpCompTable
exec_result_t load_WarmUpCompTable();
exec_result_t store_WarmUpCompTable();
void show_WarmUpCompTable(USART_TypeDef * Port);
exec_result_t modify_WarmUpCompTable(U32 Offset, U32 Value);
void send_WarmUpCompTable(USART_TypeDef * Port);
F32 getValue_WarmUpCompTable(F32 CLT_K);

//Injector dead time table - InjectorTimingTable
exec_result_t load_InjectorTimingTable();
exec_result_t store_InjectorTimingTable();
void show_InjectorTimingTable(USART_TypeDef * Port);
exec_result_t modify_InjectorTimingTable(U32 Offset, U32 Value);
void send_InjectorTimingTable(USART_TypeDef * Port);
F32 getValue_InjectorTimingTable(F32 Bat_V);

//Cranking base fuel mass table - CrankingFuelTable
exec_result_t load_CrankingFuelTable();
exec_result_t store_CrankingFuelTable();
void show_CrankingFuelTable(USART_TypeDef * Port);
exec_result_t modify_CrankingFuelTable(U32 Offset, U32 Value);
void send_CrankingFuelTable(USART_TypeDef * Port);
F32 getValue_CrankingFuelTable(F32 CLT_K);


//Barometric pressure correction - BAROtable
exec_result_t load_BAROtable();
exec_result_t store_BAROtable();
void show_BAROtable(USART_TypeDef * Port);
exec_result_t modify_BAROtable(U32 Offset, U32 Value);
void send_BAROtable(USART_TypeDef * Port);
F32 getValue_BAROtable(F32 BARO_kPa);

//charge temperature table - ChargeTempTable
exec_result_t load_ChargeTempTable();
exec_result_t store_ChargeTempTable();
void show_ChargeTempTable(USART_TypeDef * Port);
exec_result_t modify_ChargeTempTable(U32 Offset, U32 Value);
void send_ChargeTempTable(USART_TypeDef * Port);
F32 getValue_ChargeTempTable(F32 IAT_K, F32 CLT_K);


/***************************************************************************************************************************************************
*   helper macros
***************************************************************************************************************************************************/
/*
#define ASSERT_CRANK_POS(pos) if((pos) >= CRK_POSITION_COUNT) return EXEC_ERROR
#define ASSERT_Fueling_SETUP(setup) if((setup) >= COILS_COUNT) return EXEC_ERROR
*/

#endif // FUELING_CONFIG_H_INCLUDED
