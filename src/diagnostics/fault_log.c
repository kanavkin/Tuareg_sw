

#include "fault_log.h"
#include "eeprom.h"
#include "eeprom_layout.h"

#include "Tuareg.h"

#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"

#include "syslog.h"
#include "diag_syslog_locations.h"

#include "debug_port_messages.h"


//#define FAULTLOG_DEBUG_OUTPUT

#ifdef FAULTLOG_DEBUG_OUTPUT
#warning Fault Log Debug messages enabled
#endif // FAULTLOG_DEBUG_OUTPUT


//the fault log itself
volatile Fault_Log_t Fault_Log;

volatile U8 * const pFault_Log_data= (volatile U8 *) &Fault_Log;
const U32 cFault_Log_size= sizeof(Fault_Log);

bool Init_done;

/******************************************************************************************************************************
init
******************************************************************************************************************************/

void init_Fault_Log()
{
    exec_result_t result;

    //reinit protection
    if(Init_done == true)
    {
        return;
    }

    //existing log data shall be loaded first
    result= load_Fault_Log();

    //check if config has been loaded
    if(result != EXEC_OK)
    {
        //failed to load Ignition Config
        Tuareg.errors.fault_log_error= true;
        load_void_Fault_Log();

        Syslog_Error(TID_FAULTLOG, DIAG_LOC_FAULTLOG_LOAD_FAIL);

        #ifdef FAULTLOG_DEBUG_OUTPUT
        DebugMsg_Error("Failed to load Fault Log!");
        DebugMsg_Warning("Void Fault Log has been loaded");
        #endif // FAULTLOG_DEBUG_OUTPUT
    }
    else
    {
        //loaded Fault Log data
        Tuareg.errors.fault_log_error= false;

        Syslog_Info(TID_FAULTLOG, DIAG_LOC_FAULTLOG_LOAD_SUCCESS);
    }


    Init_done= true;
}



/******************************************************************************************************************************
write to fault log
******************************************************************************************************************************/

void log_Fault(Tuareg_ID Src, U8 Location)
{
    exec_result_t result;
    volatile faultlog_entry_t * pEntry= &(Fault_Log.entries[0]);

    //check preconditions
    if(Init_done == false)
    {
        Syslog_Error(TID_FAULTLOG, DIAG_LOC_FAULTLOG_NOINIT);
        return;
    }

    //check if fault log has some space left
    if((Fault_Log.entry_count < FAULTLOG_LENGTH) && (Tuareg.errors.fault_log_error == false))
    {
        pEntry= &(Fault_Log.entries[Fault_Log.entry_count]);
    }

    //fill log entry
    pEntry->src= Src;
    pEntry->location= Location;

    //count entry
    Fault_Log.entry_count += 1;

    //write fault log to eeprom
    result= store_Fault_Log();

    if(result != EXEC_OK)
    {
        //failed to store fault log data
        Tuareg.errors.fault_log_error= true;

        Syslog_Error(TID_FAULTLOG, DIAG_LOC_FAULTLOG_WRITE_FAIL);

        #ifdef FAULTLOG_DEBUG_OUTPUT
        DebugMsg_Error("Failed to write Fault Log!");
        #endif // FAULTLOG_DEBUG_OUTPUT
    }

}


/******************************************************************************************************************************
erase fault log
******************************************************************************************************************************/

void Erase_Fault_Log()
{
    exec_result_t result;

    //check preconditions
    if(Init_done == false)
    {
        Syslog_Error(TID_FAULTLOG, DIAG_LOC_FAULTLOG_NOINIT);
        return;
    }

    #ifdef FAULTLOG_DEBUG_OUTPUT
    DebugMsg_Warning("Fault Log will be erased");
    #endif // FAULTLOG_DEBUG_OUTPUT

    clear_Fault_Log();

    //write fault log to eeprom
    result= store_Fault_Log();

    if(result != EXEC_OK)
    {
        //failed to store fault log data
        Tuareg.errors.fault_log_error= true;

        Syslog_Error(TID_FAULTLOG, DIAG_LOC_FAULTLOG_WRITE_FAIL);

        #ifdef FAULTLOG_DEBUG_OUTPUT
        DebugMsg_Error("Failed to write Fault Log!");
        #endif // FAULTLOG_DEBUG_OUTPUT
    }

}




/**
* reads fault log data from eeprom
*/
exec_result_t load_Fault_Log()
{
    //bring up eeprom
    Eeprom_init();

   return Eeprom_load_data(EEPROM_FAULT_LOG_BASE, pFault_Log_data, cFault_Log_size);
}


/**
* provides sane defaults if log data data cannot be loaded from eeprom
*/
void load_void_Fault_Log()
{
    U32 i;

    for(i=0; i < cFault_Log_size; i++)
    {
        *(pFault_Log_data + i)= 0x42;
    }
}


/**
* writes fault log to eeprom
*/
exec_result_t store_Fault_Log()
{
    return Eeprom_update_data(EEPROM_FAULT_LOG_BASE, pFault_Log_data, cFault_Log_size);
}


void show_Fault_Log(USART_TypeDef * Port)
{
    U32 msg;

    //header
    print(Port, "\r\n\r\n*** Fault Log ***\r\n");

    print(Port, "Entries: ");
    printf_U(Port, Fault_Log.entry_count, NO_PAD | NO_TRAIL);

    for(msg=0; msg < FAULTLOG_LENGTH; msg++)
    {
        //skip empty lines
        if(Fault_Log.entries[msg].src != 0)
        {
            //src
            print(Port, "\r\nMod: ");
            printf_U(Port, Fault_Log.entries[msg].src, NO_PAD);

            //location
            print(Port, "Loc: ");
            printf_U(Port, Fault_Log.entries[msg].location, NO_PAD | NO_TRAIL);
        }
    }
}



/**
clear the entire fault log
*/
void clear_Fault_Log()
{
    U32 i;

    for(i=0; i < cFault_Log_size; i++)
    {
        *(pFault_Log_data + i)= 0;
    }

}


/**
this function implements the TS interface binary config page read command for fault log
*/
void send_Fault_Log(USART_TypeDef * Port)
{
    UART_send_data(Port, pFault_Log_data, cFault_Log_size);
}


