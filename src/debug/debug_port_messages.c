/**
this module contains the helper functions to print debug messages to the DEBUG_PORT

hint: to receive these messages physical access to the USART Port is required
*/


#include "stm32_libs/boctok_types.h"



#include "uart.h"
#include "uart_printf.h"


void DebugMsg_Error(char messg[])
{
    //timestamp

    //header
    print(DEBUG_PORT, "\r\nEE ");

    //message
    print(DEBUG_PORT, messg);
}


void DebugMsg_Warning(char messg[])
{
    //timestamp

    //header
    print(DEBUG_PORT, "\r\nWW ");

    //message
    print(DEBUG_PORT, messg);
}






