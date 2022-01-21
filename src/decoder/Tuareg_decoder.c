#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

#include "base_calc.h"

#include "Tuareg.h"

#include "decoder_hw.h"
#include "decoder_logic.h"
#include "decoder_config.h"

#include "uart.h"
#include "uart_printf.h"

#include "debug_port_messages.h"

//#define DECODER_DEBUG

#ifdef DECODER_DEBUG
#warning Decoder Debug messages enabled
#endif // DECODER_DEBUG

/// TODO (oli#8#): implement decoder syslog messages


/******************************************************************************************************************************
Decoder initialization
 ******************************************************************************************************************************/
volatile decoder_output_t * init_Decoder()
{
    exec_result_t result;

    //setup shall be loaded first
    result= load_Decoder_Setup();

    //check if config has been loaded
    if(result != EXEC_OK)
    {
        //failed to load Decoder Config
        Tuareg.errors.decoder_config_error= true;
        Tuareg.flags.limited_op= true;
        load_essential_Decoder_Setup();

        #ifdef DECODER_DEBUG
        DebugMsg_Error("Failed to load Decoder Config!");
        DebugMsg_Warning("Decoder essential Config has been loaded");
        #endif // DECODER_DEBUG
    }
    else if(Decoder_Setup.Version != DECODER_REQUIRED_CONFIG_VERSION)
    {
        //loaded wrong Decoder Config Version
        Tuareg.errors.decoder_config_error= true;
        Tuareg.flags.limited_op= true;
        load_essential_Decoder_Setup();

        #ifdef DECODER_DEBUG
        DebugMsg_Error("Decoder Config version does not match");
        DebugMsg_Warning("Decoder essential Config has been loaded");
        #endif // DECODER_DEBUG
    }
    else
    {
        //loaded Decoder Config with correct Version
        Tuareg.errors.decoder_config_error= false;

        #ifdef DECODER_DEBUG
        print(DEBUG_PORT, "\r\nDecoder Config has been loaded");
        #endif // DECODER_DEBUG
    }


    //init hw part
    init_decoder_hw();

    //init logic part
    init_decoder_logic();

    return &(Decoder.out);
}




/******************************************************************************************************************************
calculate position data age
******************************************************************************************************************************/

VU32 decoder_get_position_data_age_us()
{
    VU32 now_ts, update_ts, interval_us;

    now_ts= decoder_get_timestamp();
    update_ts= Decoder_hw.current_timer_value;

    //check counting mode
    if(Decoder_hw.state.timer_continuous_mode == true)
    {
        //timer continuously counting since last position update
        interval_us= Decoder_hw.timer_period_us * subtract_VU32(now_ts, update_ts);
    }
    else
    {
        //timer has been reset on last position update
        interval_us= Decoder_hw.timer_period_us * now_ts;
    }

    return interval_us;

}

