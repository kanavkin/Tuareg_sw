#ifndef TUAREG_CTRL_SETS_H_INCLUDED
#define TUAREG_CTRL_SETS_H_INCLUDED

#include "Tuareg.h"



/***************************************************************************************************************************************************
*   Control sets
***************************************************************************************************************************************************/

extern volatile ctrlset_t Control_MAP;
extern volatile ctrlset_t Control_TPS;
extern volatile ctrlset_t Control_TPS_Limp;


/***************************************************************************************************************************************************
*   API section
***************************************************************************************************************************************************/

exec_result_t load_Control_MAP();
exec_result_t store_Control_MAP();
exec_result_t modify_Control_MAP(U32 Offset, U32 Value);
void show_Control_MAP(USART_TypeDef * Port);
void send_Control_MAP(USART_TypeDef * Port);


#endif // TUAREG_CTRL_SETS_H_INCLUDED
