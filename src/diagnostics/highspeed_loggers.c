#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/boctok_types.h"

#include "base_calc.h"
#include "Tuareg.h"
#include "highspeed_loggers.h"
#include "bitfields.h"
#include "uart.h"
#include "uart_printf.h"



#define HSPLOG_TRACK_CRKPOS
/*
#define HSPLOG_TRACK_CIS

#define HSPLOG_TRACK_COIL1
#define HSPLOG_TRACK_COIL2
#define HSPLOG_TRACK_INJECTOR1
#define HSPLOG_TRACK_INJECTOR2
*/


volatile highspeedlog_entry_t Highspeedlog[HIGSPEEDLOG_LENGTH];

highspeedlog_mgr_t Highspeedlog_Mgr;


volatile highspeedlog_entry_t Log_dummy;


volatile U8 * const pHighspeedlog_data= (volatile U8 *) &Highspeedlog;

const U32 cHighspeedlog_size= sizeof(Highspeedlog);



/******************************************************************************************************************************


******************************************************************************************************************************/

volatile highspeedlog_flags_t * highspeedlog_init()
{
    clear_highspeedlog();

    return &(Highspeedlog_Mgr.flags);
}


void clear_highspeedlog()
{
    Highspeedlog_Mgr.entry_ptr= 0;
    Highspeedlog_Mgr.flags.log_full= false;
}


void highspeedlog_register_error()
{

}


volatile highspeedlog_entry_t * const highspeedlog_reserve_entry()
{
    volatile highspeedlog_entry_t * pTarget;

    if(Highspeedlog_Mgr.entry_ptr >= HIGSPEEDLOG_LENGTH)
    {
        Highspeedlog_Mgr.flags.log_full= true;
        return &Log_dummy;
    }

    pTarget= &(Highspeedlog[Highspeedlog_Mgr.entry_ptr]);

    Highspeedlog_Mgr.entry_ptr++;

    if(Highspeedlog_Mgr.entry_ptr >= HIGSPEEDLOG_LENGTH)
    {
        Highspeedlog_Mgr.flags.log_full= true;
    }

    //already fill in timestamps
    pTarget->system_ts= Tuareg.pTimer->system_time;
    pTarget->fraction_ts= get_timestamp_fraction_us();

    return pTarget;
}



/******************************************************************************************************************************
crank position tracker
******************************************************************************************************************************/

void highspeedlog_register_crankpos(volatile crank_position_t Position)
{
    #ifdef HSPLOG_TRACK_CRKPOS
    volatile highspeedlog_entry_t * pTarget;

    pTarget= highspeedlog_reserve_entry();

    pTarget->event= (Position < CRK_POSITION_COUNT)? Position : HLOGA_CRKPOS_UNDEFINED;
    #endif
}


/******************************************************************************************************************************
cam tracker
******************************************************************************************************************************/

void highspeedlog_register_cis_lobe_begin()
{
    #ifdef HSPLOG_TRACK_CIS
    volatile highspeedlog_entry_t * pTarget;

    pTarget= highspeedlog_reserve_entry();

    pTarget->event= HLOGA_CAMLOBE_BEG;
    #endif // HSPLOG_TRACK_CIS
}

void highspeedlog_register_cis_lobe_end()
{
    #ifdef HSPLOG_TRACK_CIS
    volatile highspeedlog_entry_t * pTarget;

    pTarget= highspeedlog_reserve_entry();

    pTarget->event= HLOGA_CAMLOBE_END;
    #endif // HSPLOG_TRACK_CIS
}


/******************************************************************************************************************************
coil 1 tracker
******************************************************************************************************************************/

void highspeedlog_register_coil1_power()
{
    #ifdef HSPLOG_TRACK_COIL1
    volatile highspeedlog_entry_t * pTarget;

    pTarget= highspeedlog_reserve_entry();

    pTarget->event= HLOGA_COIL1_POWER;
    #endif // HSPLOG_TRACK_COIL1
}


void highspeedlog_register_coil1_unpower()
{
    #ifdef HSPLOG_TRACK_COIL1
    volatile highspeedlog_entry_t * pTarget;

    pTarget= highspeedlog_reserve_entry();

    pTarget->event= HLOGA_COIL1_UNPOWER;
    #endif // HSPLOG_TRACK_COIL1
}


/******************************************************************************************************************************
coil 2 tracker
******************************************************************************************************************************/

void highspeedlog_register_coil2_power()
{
    #ifdef HSPLOG_TRACK_COIL2
    volatile highspeedlog_entry_t * pTarget;

    pTarget= highspeedlog_reserve_entry();

    pTarget->event= HLOGA_COIL2_POWER;
    #endif // HSPLOG_TRACK_COIL2
}


void highspeedlog_register_coil2_unpower()
{
    #ifdef HSPLOG_TRACK_COIL2
    volatile highspeedlog_entry_t * pTarget;

    pTarget= highspeedlog_reserve_entry();

    pTarget->event= HLOGA_COIL2_UNPOWER;
    #endif // HSPLOG_TRACK_COIL2
}


/******************************************************************************************************************************
injector 1 tracker
******************************************************************************************************************************/

void highspeedlog_register_injector1_power()
{


}

void highspeedlog_register_injector1_unpower()
{


}

/******************************************************************************************************************************
injector 2 tracker
******************************************************************************************************************************/

void highspeedlog_register_injector2_power()
{


}

void highspeedlog_register_injector2_unpower()
{


}





/******************************************************************************************************************************
this function implements the TS interface binary config page read command for highspeedlog
******************************************************************************************************************************/

void send_highspeedlog(USART_TypeDef * Port)
{
    UART_send_data(Port, pHighspeedlog_data, cHighspeedlog_size);

    Highspeedlog_Mgr.flags.log_full= false;
}
