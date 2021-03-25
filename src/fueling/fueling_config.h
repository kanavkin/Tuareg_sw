#ifndef FUELING_CONFIG_H_INCLUDED
#define FUELING_CONFIG_H_INCLUDED

#include "Tuareg_fueling.h"

#define FUELING_SETUP_SIZE 14

/***************************************************************************************************************************************************
*   Fueling Setup Page
***************************************************************************************************************************************************/

typedef struct __attribute__ ((__packed__)) _Fueling_Setup_t {

    U8 Version;

    U16 ve_from_map_min_rpm;
    U16 ve_from_map_max_rpm;

    U16 cylinder_volume_ccm;

    U16 injector1_rate_mgps;
    U16 injector2_rate_mgps;

    U16 injector_deadtime_us;
    U8 max_injector_duty_cycle_pct;

} Fueling_Setup_t;


/***************************************************************************************************************************************************
*   API section
***************************************************************************************************************************************************/

extern volatile Fueling_Setup_t Fueling_Setup;

//General Fueling Config
exec_result_t load_Fueling_Config();
void load_essential_Fueling_Config();


//Fueling Setup
exec_result_t store_Fueling_Setup();
void show_Fueling_Setup();
exec_result_t modify_Fueling_Setup(U32 Offset, U32 Value);
void send_Fueling_Setup(USART_TypeDef * Port);


//Fueling VE Table (TPS) - VeTable_TPS
exec_result_t store_VeTable_TPS();
void show_VeTable_TPS(USART_TypeDef * Port);
exec_result_t modify_VeTable_TPS(U32 Offset, U32 Value);
void send_VeTable_TPS(USART_TypeDef * Port);
VF32 getValue_VeTable_TPS(VU32 Rpm, VF32 Tps_deg);

//Fueling VE Table (MAP based) - VeTable_MAP
exec_result_t store_VeTable_MAP();
void show_VeTable_MAP(USART_TypeDef * Port);
exec_result_t modify_VeTable_MAP(U32 Offset, U32 Value);
void send_VeTable_MAP(USART_TypeDef * Port);
VF32 getValue_VeTable_MAP(VU32 Rpm, VF32 Map_kPa);

//Fueling AFR target Table (TPS based) - AfrTable_TPS
exec_result_t store_AfrTable_TPS();
void show_AfrTable_TPS(USART_TypeDef * Port);
exec_result_t modify_AfrTable_TPS(U32 Offset, U32 Value);
void send_AfrTable_TPS(USART_TypeDef * Port);
VF32 getValue_AfrTable_TPS(VU32 Rpm, VF32 Tps_deg);







/***************************************************************************************************************************************************
*   helper macros
***************************************************************************************************************************************************/
/*
#define ASSERT_CRANK_POS(pos) if((pos) >= CRK_POSITION_COUNT) return EXEC_ERROR
#define ASSERT_Fueling_SETUP(setup) if((setup) >= COILS_COUNT) return EXEC_ERROR
*/

#endif // FUELING_CONFIG_H_INCLUDED
