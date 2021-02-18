#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/boctok_types.h"

#include "Tuareg.h"
#include "syslog.h"
#include "conversion.h"
#include "uart.h"
#include "uart_printf.h"


syslog_mgr_t Syslog_Mgr;


volatile syslog_message_t Syslog[SYSLOG_LENGTH];

volatile syslog_datagram_t Datalog[DATALOG_LENGTH];







void Syslog_init()
{
    Syslog_Mgr.Msg_E_ptr= SYSLOG_LENGTH -1;
    Syslog_Mgr.Msg_ptr= 0;
    Syslog_Mgr.D_ptr= 0;
}



void Syslog_Error(Tuareg_ID Src, U8 Location)
{
    //check preconditions
    if(Syslog_Mgr.Msg_E_ptr >= SYSLOG_LENGTH) return;

    //get timestamp
    Syslog[Syslog_Mgr.Msg_E_ptr].timestamp= Tuareg.pTimer->system_time;

    //save log message
    Syslog[Syslog_Mgr.Msg_E_ptr].src= Src;
    Syslog[Syslog_Mgr.Msg_E_ptr].location= Location | (1<< SYSLOG_LOC_BIT_E);

    //calculate next E message location
    if(Syslog_Mgr.Msg_E_ptr == 0)
    {
        //log full of E messages, start over at the end
        Syslog_Mgr.Msg_E_ptr= SYSLOG_LENGTH -1;
    }
    else
    {
        Syslog_Mgr.Msg_E_ptr -= 1;
    }
}



void Syslog_Warning(Tuareg_ID Src, U8 Location)
{
    //check preconditions
    if(Location > 0x7F) return;
    if(Syslog_Mgr.Msg_ptr >= SYSLOG_LENGTH) return;

    //get timestamp
    Syslog[Syslog_Mgr.Msg_ptr].timestamp= Tuareg.pTimer->system_time;

    //save log message
    Syslog[Syslog_Mgr.Msg_ptr].src= Src;
    Syslog[Syslog_Mgr.Msg_ptr].location= Location | (1<< SYSLOG_LOC_BIT_W);

    //calculate next message location
    if(Syslog_Mgr.Msg_ptr == SYSLOG_LENGTH -1)
    {
        //log full of E messages, start over at the end
        Syslog_Mgr.Msg_ptr= 0;
    }
    else
    {
        Syslog_Mgr.Msg_ptr += 1;
    }
}



void Syslog_Info(Tuareg_ID Src, U8 Location)
{
    //check preconditions
    if(Location > 0x3F) return;
    if(Syslog_Mgr.Msg_ptr >= SYSLOG_LENGTH) return;

    //get timestamp
    Syslog[Syslog_Mgr.Msg_ptr].timestamp= Tuareg.pTimer->system_time;

    //save log message
    Syslog[Syslog_Mgr.Msg_ptr].src= Src;
    Syslog[Syslog_Mgr.Msg_ptr].location= Location;

    //calculate next message location
    if(Syslog_Mgr.Msg_ptr == SYSLOG_LENGTH -1)
    {
        //log full of E messages, start over at the end
        Syslog_Mgr.Msg_ptr= 0;
    }
    else
    {
        Syslog_Mgr.Msg_ptr += 1;
    }
}

void Syslog_Data(Tuareg_ID Src, U8 Location)
{
    //check preconditions
    if(Syslog_Mgr.D_ptr >= SYSLOG_LENGTH) return;

    //get timestamp
    Datalog[Syslog_Mgr.D_ptr].timestamp= Tuareg.pTimer->system_time;

    //save log message
    Datalog[Syslog_Mgr.D_ptr].src= Src;
    Datalog[Syslog_Mgr.D_ptr].location= Location;

    //calculate next message location
    if(Syslog_Mgr.Msg_ptr == SYSLOG_LENGTH -1)
    {
        //log full of E messages, start over at the end
        Syslog_Mgr.Msg_ptr= 0;
    }
    else
    {
        Syslog_Mgr.Msg_ptr += 1;
    }
}


