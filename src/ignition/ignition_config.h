#ifndef IGNITION_CONFIG_H_INCLUDED
#define IGNITION_CONFIG_H_INCLUDED

#include "Tuareg_ignition.h"

#define IGNITION_SETUP_SIZE 18
#define IGNITION_ADVTPS_SIZE 288

/***************************************************************************************************************************************************
*   Ignition Setup Page
***************************************************************************************************************************************************/

typedef struct __attribute__ ((__packed__)) _Ignition_Setup_t_ {

    U8 Version;

    //dynamic ignition function
    U16 dynamic_min_rpm;
    crank_position_t dynamic_ignition_base_position;
    U16 dynamic_dwell_target_us;

    //cold idle ignition advance function
    U16 cold_idle_cutoff_rpm;
    U16 cold_idle_cutoff_CLT_K;
    U8 cold_idle_ignition_advance_deg;
    U16 cold_idle_dwell_target_us;

    //static ignition setup for cranking
    crank_position_t cranking_ignition_position;
    crank_position_t cranking_dwell_position;

    //how many coils are installed?
    coil_setup_t coil_setup;

    U16 spark_duration_us;

} Ignition_Setup_t;


/***************************************************************************************************************************************************
*   API section
***************************************************************************************************************************************************/

//General Ignition Config
exec_result_t load_Ignition_Config();
void load_essential_Ignition_Config();

//Ignition Setup
extern volatile Ignition_Setup_t Ignition_Setup;
exec_result_t store_Ignition_Setup();
void show_Ignition_Setup();
exec_result_t modify_Ignition_Setup(U32 Offset, U32 Value);
void send_Ignition_Setup(USART_TypeDef * Port);

// Ignition Advance Table (TPS)
exec_result_t store_ignAdvTable_TPS();
void show_ignAdvTable_TPS(USART_TypeDef * Port);
exec_result_t modify_ignAdvTable_TPS(U32 Offset, U32 Value);
void send_ignAdvTable_TPS(USART_TypeDef * Port);
VU32 getValue_ignAdvTable_TPS(VU32 Rpm, VF32 TPS);


//Ignition Dwell time table - ignDwellTable
exec_result_t store_ignDwellTable();
void show_ignDwellTable(USART_TypeDef * Port);
exec_result_t modify_ignDwellTable(U32 Offset, U32 Value);
void send_ignDwellTable(USART_TypeDef * Port);
VU32 getValue_ignDwellTable(VU32 Rpm);

/***************************************************************************************************************************************************
*   helper macros
***************************************************************************************************************************************************/
#define ASSERT_CRANK_POS(pos) if((pos) >= CRK_POSITION_COUNT) return EXEC_ERROR
#define ASSERT_IGNITION_SETUP(setup) if((setup) >= COILS_COUNT) return EXEC_ERROR


#endif // IGNITION_CONFIG_H_INCLUDED
