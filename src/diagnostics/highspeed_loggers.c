#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/boctok_types.h"

#include "base_calc.h"
#include "Tuareg.h"
#include "highspeed_loggers.h"
#include "bitfields.h"
#include "uart.h"
#include "uart_printf.h"


highspeedlog_mgr_t Highspeedlog_Mgr;






void highspeedlog_init()
{



}





/******************************************************************************************************************************


******************************************************************************************************************************/


void highspeedlog_register_error()
{

}




void highspeedlog_register_crankpos(volatile crank_position_t Position, VU32 timestamp)
{

}



void highspeedlog_register_cis_lobe_begin()
{

}

void highspeedlog_register_cis_lobe_end()
{

}



void highspeedlog_register_coil1_power()
{


}

void highspeedlog_register_coil1_unpower()
{


}

void highspeedlog_register_coil2_power()
{


}

void highspeedlog_register_coil2_unpower()
{


}


void highspeedlog_register_injector1_power()
{


}

void highspeedlog_register_injector1_unpower()
{


}

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


}
