#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/boctok_types.h"

#include "base_calc.h"
#include "Tuareg.h"
#include "highspeed_loggers.h"
#include "bitfields.h"
#include "uart.h"
#include "uart_printf.h"

#include "conversion.h"


#define HSPLOG_TRACK_CRKPOS

#define HSPLOG_TRACK_CIS

#define HSPLOG_TRACK_COIL1
#define HSPLOG_TRACK_COIL2
#define HSPLOG_TRACK_INJECTOR1
#define HSPLOG_TRACK_INJECTOR2



volatile highspeedlog_entry_t Highspeedlog[HIGSPEEDLOG_LENGTH];

highspeedlog_mgr_t Highspeedlog_Mgr;


/******************************************************************************************************************************


******************************************************************************************************************************/

volatile highspeedlog_flags_t * highspeedlog_init()
{
    clear_highspeedlog();

    Highspeedlog_Mgr.cam_lobe_begin_triggered= false;

    return &(Highspeedlog_Mgr.flags);
}


/**
executed on init and after every high speed log readout
*/
void clear_highspeedlog()
{
    Highspeedlog_Mgr.entry_ptr= 0;
    Highspeedlog_Mgr.flags.log_full= false;
}





void highspeedlog_register_error()
{

}





/******************************************************************************************************************************
register event
******************************************************************************************************************************/

void highspeedlog_write(highspeedlog_event_t Event)
{
    volatile highspeedlog_entry_t * pTarget;

    //check if there is some space left to write to
    if(Highspeedlog_Mgr.entry_ptr >= HIGSPEEDLOG_LENGTH)
    {
        Highspeedlog_Mgr.flags.log_full= true;
        return;
    }

    __disable_irq();

    //get reference to the next entry to write to
    pTarget= &(Highspeedlog[Highspeedlog_Mgr.entry_ptr]);

    //next highspeedlog write will address the next entry
    Highspeedlog_Mgr.entry_ptr++;


    //timestamps
    pTarget->system_ts= Tuareg.pTimer->system_time;
    pTarget->fraction_ts= get_timestamp_fraction_us();


    //event
    pTarget->event= Event;

    //actors
    pTarget->flags.coil1= Tuareg.flags.ignition_coil_1;
    pTarget->flags.coil2= Tuareg.flags.ignition_coil_2;
    pTarget->flags.injector1= Tuareg.flags.fuel_injector_1;
    pTarget->flags.injector2= Tuareg.flags.fuel_injector_2;

    //position
    pTarget->crank_position= Tuareg.pDecoder->crank_position;

    //phasing
    pTarget->flags.cam_lobe= Highspeedlog_Mgr.cam_lobe_begin_triggered;
    pTarget->flags.phase_comp= (Tuareg.pDecoder->phase == PHASE_CYL1_COMP) ? true : false;
    pTarget->flags.phase_valid= Tuareg.pDecoder->flags.phase_valid;

    __enable_irq();


    //check if the log is ready to be read
    if(Highspeedlog_Mgr.entry_ptr >= HIGSPEEDLOG_LENGTH)
    {
        Highspeedlog_Mgr.flags.log_full= true;
    }
}





/******************************************************************************************************************************
crank position tracker
******************************************************************************************************************************/

void highspeedlog_register_crankpos()
{
    #ifdef HSPLOG_TRACK_CRKPOS
    highspeedlog_write(HLOGA_CRKPOS_UPD);
    #endif
}


/******************************************************************************************************************************
cam tracker
******************************************************************************************************************************/

void highspeedlog_register_cis_lobe_begin()
{
    Highspeedlog_Mgr.cam_lobe_begin_triggered= true;

    #ifdef HSPLOG_TRACK_CIS
    highspeedlog_write(HLOGA_CAMLOBE_BEG);
    #endif // HSPLOG_TRACK_CIS
}

void highspeedlog_register_cis_lobe_end()
{
    Highspeedlog_Mgr.cam_lobe_begin_triggered= false;

    #ifdef HSPLOG_TRACK_CIS
    highspeedlog_write(HLOGA_CAMLOBE_END);
    #endif // HSPLOG_TRACK_CIS
}


/******************************************************************************************************************************
coil 1 tracker
******************************************************************************************************************************/

void highspeedlog_register_coil1_power()
{
    #ifdef HSPLOG_TRACK_COIL1
    highspeedlog_write(HLOGA_COIL1_POWER);
    #endif // HSPLOG_TRACK_COIL1
}


void highspeedlog_register_coil1_unpower()
{
    #ifdef HSPLOG_TRACK_COIL1
    highspeedlog_write(HLOGA_COIL1_UNPOWER);
    #endif // HSPLOG_TRACK_COIL1
}


/******************************************************************************************************************************
coil 2 tracker
******************************************************************************************************************************/

void highspeedlog_register_coil2_power()
{
    #ifdef HSPLOG_TRACK_COIL2
    highspeedlog_write(HLOGA_COIL2_POWER);
    #endif // HSPLOG_TRACK_COIL2
}


void highspeedlog_register_coil2_unpower()
{
    #ifdef HSPLOG_TRACK_COIL2
    highspeedlog_write(HLOGA_COIL2_UNPOWER);
    #endif // HSPLOG_TRACK_COIL2
}


/******************************************************************************************************************************
injector 1 tracker
******************************************************************************************************************************/

void highspeedlog_register_injector1_power()
{
    #ifdef HSPLOG_TRACK_INJECTOR1
    highspeedlog_write(HLOGA_INJECTOR1_POWER);
    #endif // HSPLOG_TRACK_INJECTOR1
}

void highspeedlog_register_injector1_unpower()
{
    #ifdef HSPLOG_TRACK_INJECTOR1
    highspeedlog_write(HLOGA_INJECTOR1_UNPOWER);
    #endif // HSPLOG_TRACK_INJECTOR1
}

/******************************************************************************************************************************
injector 2 tracker
******************************************************************************************************************************/

void highspeedlog_register_injector2_power()
{
    #ifdef HSPLOG_TRACK_INJECTOR2
    highspeedlog_write(HLOGA_INJECTOR2_POWER);
    #endif // HSPLOG_TRACK_INJECTOR2

}

void highspeedlog_register_injector2_unpower()
{
    #ifdef HSPLOG_TRACK_INJECTOR2
    highspeedlog_write(HLOGA_INJECTOR2_UNPOWER);
    #endif // HSPLOG_TRACK_INJECTOR2
}


/******************************************************************************************************************************
this function implements the TS interface binary config page read command for highspeedlog
from the second log entry an alias data entry will be send for each log data entry (to force steep edges in TS)
so 2(n-1)+1 entries will be send (n is the data log length)
******************************************************************************************************************************/

void send_highspeedlog(USART_TypeDef * Port)
{
    U32 entry, i;
    volatile highspeedlog_entry_t * pTarget;

    U8 data_out[9];
    U8 alias_data_out[9];


    if(Highspeedlog_Mgr.flags.log_full == false)
    {
        return;
    }


    for(entry =0; entry < HIGSPEEDLOG_LENGTH; entry++)
    {

        //get reference to the next entry to write to
        pTarget= &(Highspeedlog[entry]);

        //flags
        data_out[0]= pTarget->flags.all_flags;

        //crank position
        data_out[1]= pTarget->crank_position;

        //event
        data_out[2]= pTarget->event;

        //fraction timestamp
        data_out[3]= pTarget->fraction_ts >> 8;
        data_out[4]= pTarget->fraction_ts;

        //system timestamp
        data_out[5]= pTarget->system_ts >> 24;
        data_out[6]= pTarget->system_ts >> 16;
        data_out[7]= pTarget->system_ts >> 8;
        data_out[8]= pTarget->system_ts;


        if(entry > 0)
        {
            /*
            create alias data entry
            data_out holds the last state
            */
            for(i=0; i< 9; i++)
            {
                alias_data_out[i]= data_out[i];
            }

            //event := alias
            alias_data_out[2]= HLOGA_ALIAS;

            //alias fraction timestamp: 5 us earlier
            alias_data_out[3]= subtract_VU32(pTarget->fraction_ts, 5) >> 8;
            alias_data_out[4]= subtract_VU32(pTarget->fraction_ts, 5);
        }

        //send data
        UART_send_data(Port, &(data_out[0]), 9);
        UART_send_data(Port, &(alias_data_out[0]), 9);
    }

    clear_highspeedlog();
}


/******************************************************************************************************************************
prints the high speed log in human readable form
******************************************************************************************************************************/

void show_highspeedlog(USART_TypeDef * Port)
{
    U32 entry;
    volatile highspeedlog_entry_t * pTarget;

    if(Highspeedlog_Mgr.flags.log_full == false)
    {
        return;
    }

    print(Port, "\r\nhighspeed log:\r\n");


    for(entry =0; entry < HIGSPEEDLOG_LENGTH; entry++)
    {

        //get reference to the next entry to write to
        pTarget= &(Highspeedlog[entry]);

        //timestamp
        print(Port, "\r\ntimestamp: ");
        printf_U(Port, pTarget->system_ts, NO_PAD | NO_TRAIL);
        UART_Tx(Port, '.');
        printf_U(Port, pTarget->fraction_ts, NO_PAD | NO_TRAIL);

        //crank position
        print(Port, "\r\ncrk pos: ");
        printf_crkpos(Port, pTarget->crank_position);

        //event
        print(Port, "\r\nevent: ");
        printf_U(Port, pTarget->event, NO_PAD | NO_TRAIL);

        //flags
        print(Port, "\r\ncoil1-coil2-inj1-inj2-cam_lobe-phase_valid-phase_comp: ");

        UART_Tx(Port, (pTarget->flags.coil1 == true)? '1' : '0');
        UART_Tx(Port, '-');
        UART_Tx(Port, (pTarget->flags.coil2 == true)? '1' : '0');
        UART_Tx(Port, '-');
        UART_Tx(Port, (pTarget->flags.injector1 == true)? '1' : '0');
        UART_Tx(Port, '-');
        UART_Tx(Port, (pTarget->flags.injector2 == true)? '1' : '0');
        UART_Tx(Port, '-');
        UART_Tx(Port, (pTarget->flags.cam_lobe == true)? '1' : '0');
        UART_Tx(Port, '-');
        UART_Tx(Port, (pTarget->flags.phase_valid == true)? 'v' : 'i');
        UART_Tx(Port, '-');
        UART_Tx(Port, (pTarget->flags.phase_comp == true)? 'c' : 'e');
    }

    clear_highspeedlog();
}
