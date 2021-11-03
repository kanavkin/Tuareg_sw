#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/boctok_types.h"

#include "base_calc.h"
#include "Tuareg.h"
#include "syslog.h"
#include "bitfields.h"
#include "uart.h"
#include "uart_printf.h"


syslog_mgr_t Syslog_Mgr;


volatile syslog_message_t Syslog[SYSLOG_LENGTH];

volatile syslog_datagram_t Datalog[DATALOG_LENGTH];

volatile U8 * const pSyslog_data= (volatile U8 *) &Syslog;
const U32 cSyslog_size= sizeof(Syslog);


volatile syslog_mgr_flags_t * Syslog_init()
{
    clear_syslog();
    clear_datalog();

    return &(Syslog_Mgr.flags);
}


void clear_syslog()
{
    U32 i;

    Syslog_Mgr.Msg_E_ptr= SYSLOG_LENGTH -1;
    Syslog_Mgr.Msg_ptr= 0;

    for(i=0; i< SYSLOG_LENGTH; i++)
    {
        Syslog[i].src= 0;
    }


    Syslog_Mgr.flags.syslog_new_entry= false;
}


void clear_datalog()
{
    U32 i;

    Syslog_Mgr.D_ptr= 0;

    for(i=0; i< SYSLOG_LENGTH; i++)
    {
        Datalog[i].src= 0;
    }

    Syslog_Mgr.flags.datalog_new_entry= false;

}



/******************************************************************************************************************************
write to syslog

Errors will be written from Log end to front until the log is full of errors. Older error entries will be kept, only index 0 will be
overwritten, every time a new error is reported.
******************************************************************************************************************************/

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
    Syslog_Mgr.Msg_E_ptr= subtract_VU32(Syslog_Mgr.Msg_E_ptr, 1);

    Syslog_Mgr.flags.syslog_new_entry= true;
}


void Syslog_Warning(Tuareg_ID Src, U8 Location)
{
    //check preconditions
    if(Location > 0x7F) return;
    if(Syslog_Mgr.Msg_ptr >= SYSLOG_LENGTH) return;
    if(Syslog_Mgr.Msg_E_ptr == 0) return;

    //don't overwrite error log entries
    if(Syslog_Mgr.Msg_ptr >= Syslog_Mgr.Msg_E_ptr)
    {
        //log full of E messages, start over at the end
        Syslog_Mgr.Msg_ptr= 0;
    }

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

    Syslog_Mgr.flags.syslog_new_entry= true;
}


void Syslog_Info(Tuareg_ID Src, U8 Location)
{
    //check preconditions
    if(Location > 0x3F) return;
    if(Syslog_Mgr.Msg_ptr >= SYSLOG_LENGTH) return;
    if(Syslog_Mgr.Msg_E_ptr == 0) return;

    //don't overwrite error log entries
    if(Syslog_Mgr.Msg_ptr >= Syslog_Mgr.Msg_E_ptr)
    {
        //log full of E messages, start over at the end
        Syslog_Mgr.Msg_ptr= 0;
    }

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

    Syslog_Mgr.flags.syslog_new_entry= true;
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

    Syslog_Mgr.flags.datalog_new_entry= true;
}


/******************************************************************************************************************************
syslog output in human readable form
******************************************************************************************************************************/
void show_syslog(USART_TypeDef * Port)
{
    U32 msg;

    //header
    print(Port, "\r\n\r\n*** Syslog ***\r\n");

    for(msg=0; msg < SYSLOG_LENGTH; msg++)
    {
        //skip empty lines
        if(Syslog[msg].src != 0)
        {
            //timestamp
            print(Port, "\r\n[");
            printf_U(Port, Syslog[msg].timestamp, PAD_10 | NO_TRAIL);
            print(Port, "] ");

            //type
            if( getBit_BF8(SYSLOG_LOC_BIT_E, Syslog[msg].location))
            {
                print(Port, "EE");
            }
            else if( getBit_BF8(SYSLOG_LOC_BIT_W, Syslog[msg].location) )
            {
                print(Port, "WW");
            }
            else
            {
                print(Port, "II");
            }

            //src
            print(Port, " Mod: ");

            //printf_U(Port, Syslog[msg].src, PAD_3);
            print_Tuareg_ID_label(Port, Syslog[msg].src);
            print(Port, "   ");

            //location (mask EE/WW bits)
            print(Port, "Loc: ");
            printf_U(Port, Syslog[msg].location & 0x3F, PAD_3 | NO_TRAIL);
        }
    }
}

void show_datalog(USART_TypeDef * Port)
{
    U32 entry, byte;


    //header
    print(Port, "\r\n\r\n*** Datalog ***\r\n");


    for(entry=0; entry < DATALOG_LENGTH; entry++)
    {
        if(Datalog[entry].src != 0)
        {
            //timestamp
            print(Port, "\r\n[");
            printf_U(Port, Datalog[entry].timestamp, PAD_10 | NO_TRAIL);
            print(Port, "]");

            //src
            print(Port, "Mod: ");
            printf_U(Port, Datalog[entry].src, PAD_3);

            //location
            print(Port, "] Loc: ");
            printf_U(Port, Datalog[entry].location, PAD_3 | NO_TRAIL);

            //data
            for(byte=0; byte < 24; byte++)
            {
                printf_U8hex(Port, Datalog[entry].data.as_U8[byte], 0);
            }
        }
    }
}


/******************************************************************************************************************************
this function implements the TS interface binary config page read command for syslog
******************************************************************************************************************************/

void send_syslog(USART_TypeDef * Port)
{
    UART_send_data(Port, pSyslog_data, cSyslog_size);

    Syslog_Mgr.flags.syslog_new_entry= false;
}
