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

//#define HSPLOG_TRACK_CIS

//#define HSPLOG_TRACK_COIL1
//#define HSPLOG_TRACK_COIL2
//#define HSPLOG_TRACK_INJECTOR1
//#define HSPLOG_TRACK_INJECTOR2



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

void highspeedlog_write()
{
    VU32 system_ts_ms;
    VU16 fraction_ts_us;
    volatile highspeedlog_entry_t * pTarget;

    //check if there is some space left to write to
    if(Highspeedlog_Mgr.entry_ptr >= HIGSPEEDLOG_LENGTH)
    {
        Highspeedlog_Mgr.flags.log_full= true;
        return;
    }

    //get reference to the next entry to write to
    pTarget= &(Highspeedlog[Highspeedlog_Mgr.entry_ptr]);

    //next highspeedlog write will address the next entry
    Highspeedlog_Mgr.entry_ptr++;

    //cache current timestamps
    system_ts_ms= Tuareg.pTimer->system_time;
    fraction_ts_us= get_timestamp_fraction_us();


    //0 .. 3,5 timestamp ms (28 bits), inits data 0..3
    serialize_U32_U8_reversed(system_ts_ms << 4, &(pTarget->data[0]));

    //clear shared high nibble
    pTarget->data[3] &= 0xF0;

    //clip fraction just to be sure
    fraction_ts_us &= 0x0FFF;

    //3,5 .. 4 fractional timestamp MSB, LSB
    pTarget->data[3] |= fraction_ts_us >> 8;
    pTarget->data[4]= fraction_ts_us;


    //5 crank pos, CIS_LOBE_BIT, PHASE_COMP_BIT, PHASE_VALID_BIT, inits data[5]
    pTarget->data[5]= Tuareg.pDecoder->crank_position & 0x0F;

    /*
    if(Highspeedlog_Mgr.cam_lobe_begin_triggered == true) setBit_BF8(HIGSPEEDLOG_BYTE5_CIS_LOBE_BIT, &(pTarget->data[5]));
    if(Tuareg.pDecoder->phase == PHASE_CYL1_COMP) setBit_BF8(HIGSPEEDLOG_BYTE5_PHASE_COMP_BIT, &(pTarget->data[5]));
    if(Tuareg.pDecoder->outputs.phase_valid == true) setBit_BF8(HIGSPEEDLOG_BYTE5_PHASE_VALID_BIT, &(pTarget->data[5]));
    */

    //init data[6]
    pTarget->data[6]=0;

    /*
    //6 crank pos, COIL1_POWERED_BIT, COIL2_POWERED_BIT, INJECTOR1_POWERED_BIT, INJECTOR2_POWERED_BIT
    if(Tuareg.actors.ignition_coil_1 == true) setBit_BF8(HIGSPEEDLOG_BYTE6_COIL1_POWERED_BIT, &(pTarget->data[6]));
    if(Tuareg.actors.ignition_coil_2 == true) setBit_BF8(HIGSPEEDLOG_BYTE6_COIL2_POWERED_BIT, &(pTarget->data[6]));
    if(Tuareg.actors.fuel_injector_1 == true) setBit_BF8(HIGSPEEDLOG_BYTE6_INJECTOR1_POWERED_BIT, &(pTarget->data[6]));
    if(Tuareg.actors.fuel_injector_2 == true) setBit_BF8(HIGSPEEDLOG_BYTE6_INJECTOR2_POWERED_BIT, &(pTarget->data[6]));
    */

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
    highspeedlog_write();
    #endif
}


/******************************************************************************************************************************
cam tracker
******************************************************************************************************************************/

void highspeedlog_register_cis_lobe_begin()
{
    #ifdef HSPLOG_TRACK_CIS
    highspeedlog_write();
    #endif // HSPLOG_TRACK_CIS
}

void highspeedlog_register_cis_lobe_end()
{
    #ifdef HSPLOG_TRACK_CIS
    highspeedlog_write();
    #endif // HSPLOG_TRACK_CIS
}


/******************************************************************************************************************************
coil 1 tracker
******************************************************************************************************************************/

void highspeedlog_register_coil1_power()
{
    #ifdef HSPLOG_TRACK_COIL1
    highspeedlog_write();
    #endif // HSPLOG_TRACK_COIL1
}


void highspeedlog_register_coil1_unpower()
{
    #ifdef HSPLOG_TRACK_COIL1
    highspeedlog_write();
    #endif // HSPLOG_TRACK_COIL1
}


/******************************************************************************************************************************
coil 2 tracker
******************************************************************************************************************************/

void highspeedlog_register_coil2_power()
{
    #ifdef HSPLOG_TRACK_COIL2
    highspeedlog_write();
    #endif // HSPLOG_TRACK_COIL2
}


void highspeedlog_register_coil2_unpower()
{
    #ifdef HSPLOG_TRACK_COIL2
    highspeedlog_write();
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
    U32 entry, i;

    if(Highspeedlog_Mgr.flags.log_full == false)
    {
        return;
    }

    for(entry =0; entry < HIGSPEEDLOG_LENGTH; entry++)
    {
        //send data
        //UART_send_data(Port, &(Highspeedlog[entry].data), sizeof(highspeedlog_entry_t.data));

        print(DEBUG_PORT, "\r\n HSL ");
        printf_U(DEBUG_PORT, entry, NO_PAD);

        for(i =0; i < 7; i++)
        {
            UART_Tx(Port, Highspeedlog[entry].data[i]);

            printf_U8hex(DEBUG_PORT, Highspeedlog[entry].data[i], 0);
        }
    }

    clear_highspeedlog();
}
