#ifndef FAULT_LOG_H_INCLUDED
#define FAULT_LOG_H_INCLUDED

#include "stm32_libs/boctok_types.h"

#include "Tuareg.h"
#include "uart.h"
#include "Tuareg_ID.h"

#define FAULTLOG_SIZE 3

#define FAULTLOG_LENGTH 3


typedef struct __attribute__ ((__packed__)) _faultlog_entry_t {

    U8 location;
    Tuareg_ID src;

} faultlog_entry_t;


/***************************************************************************************************************************************************
*   Tuareg main configuration page
***************************************************************************************************************************************************/
typedef struct __attribute__ ((__packed__)) _Fault_Log_t {

    U8 entry_count;
    faultlog_entry_t entries[FAULTLOG_LENGTH];

} Fault_Log_t;



/***************************************************************************************************************************************************
*   API section
***************************************************************************************************************************************************/

extern volatile Fault_Log_t Fault_Log;

void init_Fault_Log();
void log_Fault(Tuareg_ID Src, U8 Location);
void Erase_Fault_Log();

exec_result_t load_Fault_Log();
void load_void_Fault_Log();
exec_result_t store_Fault_Log();

void clear_Fault_Log();

void show_Fault_Log(USART_TypeDef * Port);

exec_result_t modify_Fault_Log(U32 Offset, U32 Value);

void send_Fault_Log(USART_TypeDef * Port);





#endif // FAULT_LOG_H_INCLUDED
