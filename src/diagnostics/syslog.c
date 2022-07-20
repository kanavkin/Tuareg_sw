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
volatile datalog_entry_t Datalog[DATALOG_LENGTH];

volatile U8 * const pSyslog_data= (volatile U8 *) &Syslog;
const U32 cSyslog_size= sizeof(Syslog);


volatile syslog_mgr_flags_t * Syslog_init()
{
    clear_syslog();

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

    //clear datalog
    Syslog_Mgr.D_ptr= 0;
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

    //no datalog entry
    Syslog[Syslog_Mgr.Msg_E_ptr].datalog= 0;

    //calculate next E message location
    Syslog_Mgr.Msg_E_ptr= subtract_U32(Syslog_Mgr.Msg_E_ptr, 1);

    Syslog_Mgr.flags.syslog_new_entry= true;
}

void Syslog_Error_Datalog(Tuareg_ID Src, U8 Location, volatile datalog_entry_t * pPayload)
{
    //check preconditions
    if(Syslog_Mgr.Msg_E_ptr >= SYSLOG_LENGTH) return;

    //get timestamp
    Syslog[Syslog_Mgr.Msg_E_ptr].timestamp= Tuareg.pTimer->system_time;

    //save log message
    Syslog[Syslog_Mgr.Msg_E_ptr].src= Src;
    Syslog[Syslog_Mgr.Msg_E_ptr].location= Location | (1<< SYSLOG_LOC_BIT_E);

    //datalog entry
    Syslog[Syslog_Mgr.Msg_E_ptr].datalog= append_datalog(pPayload);

    //calculate next E message location
    Syslog_Mgr.Msg_E_ptr= subtract_U32(Syslog_Mgr.Msg_E_ptr, 1);

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

    //no datalog entry
    Syslog[Syslog_Mgr.Msg_E_ptr].datalog= 0;

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

void Syslog_Warning_Datalog(Tuareg_ID Src, U8 Location, volatile datalog_entry_t * pPayload)
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

    //datalog entry
    Syslog[Syslog_Mgr.Msg_E_ptr].datalog= append_datalog(pPayload);

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

    //no datalog entry
    Syslog[Syslog_Mgr.Msg_E_ptr].datalog= 0;

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


void Syslog_Info_Datalog(Tuareg_ID Src, U8 Location, volatile datalog_entry_t * pPayload)
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

    //datalog entry
    Syslog[Syslog_Mgr.Msg_E_ptr].datalog= append_datalog(pPayload);

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


U32 append_datalog(volatile datalog_entry_t * pSource)
{
    U32 i, entry;

    //check preconditions
    if(Syslog_Mgr.D_ptr >= SYSLOG_LENGTH) return 0;

    //copy payload
    for(i=0; i< DATALOG_PAYLOAD_LEN_INT; i++)
    {
        Datalog[Syslog_Mgr.D_ptr].as_U32[i]= pSource->as_U32[i];
    }

    //generate the datalog entry field
    entry= Syslog_Mgr.D_ptr | (1 << SYSLOG_DATALOG_BIT_D);

    //calculate next message location
    if(Syslog_Mgr.D_ptr < DATALOG_LENGTH)
    {
        Syslog_Mgr.D_ptr += 1;
    }

    Syslog_Mgr.flags.datalog_new_entry= true;

    return entry;
}


/******************************************************************************************************************************
syslog output in human readable form
******************************************************************************************************************************/
void show_syslog(USART_TypeDef * Port)
{
    U32 msg, datalog_entry, byte;

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
            print_Tuareg_ID_label(Port, Syslog[msg].src);
            print(Port, "   ");

            //location (mask EE/WW bits)
            print(Port, "Loc: ");
            printf_U(Port, Syslog[msg].location & 0x3F, PAD_3);

            //datalog
            if(Syslog[msg].datalog & (1 << SYSLOG_DATALOG_BIT_D))
            {
                datalog_entry= Syslog[msg].datalog & 0x3F;

                if(datalog_entry < DATALOG_LENGTH)
                {
                    print(Port, "Data: ");

                    //payload
                    for(byte=0; byte < 4* DATALOG_PAYLOAD_LEN_INT; byte++)
                    {
                        printf_U8hex(Port, Datalog[datalog_entry].as_U8[byte], 0);
                    }
                }
                else
                {
                    print(Port, "EE Datalog invalid!");
                }

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
