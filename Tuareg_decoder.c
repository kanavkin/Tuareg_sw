#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

#include "Tuareg.h"

#include "decoder_hw.h"
#include "decoder_logic.h"
#include "decoder_config.h"

#include "uart.h"
#include "uart_printf.h"




/******************************************************************************************************************************
Decoder initialization
 ******************************************************************************************************************************/
volatile Tuareg_decoder_t * init_Decoder()
{
    exec_result_t result;
    volatile Tuareg_decoder_t * pInterface;

    //setup shall be loaded first
    result= load_Decoder_Setup();

    if(result != EXEC_OK)
    {
        load_essential_Decoder_Setup();

        Tuareg.Errors.decoder_config_error= true;

        print(DEBUG_PORT, "\r\nWARNING Decoder essential Config has been loaded");
    }

    if(Decoder_Setup.Version != DECODER_REQUIRED_CONFIG_VERSION)
    {
        Tuareg.Errors.decoder_config_error= true;

        print(DEBUG_PORT, "\r\nWARNING Decoder Config version does not match");
    }
    else
    {
        Tuareg.Errors.decoder_config_error= false;
    }

    print(DEBUG_PORT, "\r\nINFO Decoder Config has been loaded");


    //init hw part
    init_decoder_hw();

    //init logic part
    pInterface= init_decoder_logic();

    return pInterface;
}





