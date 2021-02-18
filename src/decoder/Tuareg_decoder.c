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

    //start with error state
    Tuareg.Errors.decoder_config_error= true;

    //setup shall be loaded first
    result= load_Decoder_Setup();

    //check if config has been loaded
    if(result != EXEC_OK)
    {
        //failed to load Decoder Config
        load_essential_Decoder_Setup();

        print(DEBUG_PORT, "\r\nEE Failed to load Decoder Config!");
        print(DEBUG_PORT, "\r\nWW Decoder essential Config has been loaded");
    }
    else if(Decoder_Setup.Version != DECODER_REQUIRED_CONFIG_VERSION)
    {
        //loaded wrong Decoder Config Version
        load_essential_Decoder_Setup();

        print(DEBUG_PORT, "\r\nEE Decoder Config version does not match");
        print(DEBUG_PORT, "\r\nWW Decoder essential Config has been loaded");
    }
    else
    {
        //loaded Decoder Config with correct Version
        Tuareg.Errors.decoder_config_error= false;
        print(DEBUG_PORT, "\r\nII Decoder Config has been loaded");
    }


    //init hw part
    init_decoder_hw();

    //init logic part
    pInterface= init_decoder_logic();

    return pInterface;
}





