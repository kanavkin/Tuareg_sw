/**
functions to print debug messages supporting the developers work

In a release none of this messages are allowed!

this module contains the helper functions to print debug messages to the DEBUG_PORT

hint: to receive these messages physical access to the USART Port is required
*/

#include "Tuareg_types.h"

#include "uart.h"
#include "uart_printf.h"


void DebugMsg_Error(char messg[])
{
    //header
    print(DEBUG_PORT, "\r\nEE ");

    //message
    print(DEBUG_PORT, messg);
}


void DebugMsg_Warning(char messg[])
{
    //header
    print(DEBUG_PORT, "\r\nWW ");

    //message
    print(DEBUG_PORT, messg);
}


void DebugMsg_Info(char messg[])
{
    //header
    print(DEBUG_PORT, "\r\nII ");

    //message
    print(DEBUG_PORT, messg);
}



