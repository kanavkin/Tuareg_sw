#ifndef FUELING_CONFIG_H_INCLUDED
#define FUELING_CONFIG_H_INCLUDED

#include "Tuareg_fueling.h"

#define FUELING_SETUP_SIZE 1

/***************************************************************************************************************************************************
*   Fueling Setup Page
***************************************************************************************************************************************************/

typedef struct __attribute__ ((__packed__)) _Fueling_Setup_t {

    U8 Version;


} Fueling_Setup_t;


/***************************************************************************************************************************************************
*   API section
***************************************************************************************************************************************************/

/*
//General Fueling Config
exec_result_t load_Fueling_Config();
void load_essential_Fueling_Config();

//Fueling Setup
extern volatile Fueling_Setup_t Fueling_Setup;
exec_result_t store_Fueling_Setup();
void show_Fueling_Setup();
exec_result_t modify_Fueling_Setup(U32 Offset, U32 Value);
void send_Fueling_Setup(USART_TypeDef * Port);

// Fueling Advance Table (TPS)
exec_result_t store_ignAdvTable_TPS();
void show_ignAdvTable_TPS(USART_TypeDef * Port);
exec_result_t modify_ignAdvTable_TPS(U32 Offset, U32 Value);
void send_ignAdvTable_TPS(USART_TypeDef * Port);
VU32 getValue_ignAdvTable_TPS(VU32 Rpm, VF32 TPS);
*/

/***************************************************************************************************************************************************
*   helper macros
***************************************************************************************************************************************************/
/*
#define ASSERT_CRANK_POS(pos) if((pos) >= CRK_POSITION_COUNT) return EXEC_ERROR
#define ASSERT_Fueling_SETUP(setup) if((setup) >= COILS_COUNT) return EXEC_ERROR
*/

#endif // FUELING_CONFIG_H_INCLUDED
