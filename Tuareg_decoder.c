#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

#include "decoder_hw.h"
#include "decoder_logic.h"
#include "decoder_config.h"

#include "uart.h"
#include "uart_printf.h"




/******************************************************************************************************************************
Decoder initialization
 ******************************************************************************************************************************/
volatile decoder_interface_t * init_Decoder()
{
    exec_result_t result;
    volatile decoder_interface_t * pInterface;

    //decoder logic uses configuration values -> config shall be loaded first
    result= load_Decoder_Setup();

    if(result != EXEC_OK)
    {
        load_essential_Decoder_Setup();

        print(DEBUG_PORT, "\r\nWARNING Decoder essential Config has been loaded");
    }

    if(Decoder_Setup.Version != DECODER_REQUIRED_CONFIG_VERSION)
    {
        print(DEBUG_PORT, "\r\nWARNING Decoder Config version does not match");
    }

    print(DEBUG_PORT, "\r\nINFO Decoder Config has been loaded");

    init_decoder_hw();

    pInterface= init_decoder_logic();

    return pInterface;
}





