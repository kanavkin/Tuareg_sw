#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"

#include "base_calc.h"

#include "Tuareg.h"
#include "Tuareg_errors.h"

#include "decoder_hw.h"
#include "decoder_logic.h"
#include "decoder_config.h"
#include "decoder_syslog_locations.h"

#include "uart.h"
#include "uart_printf.h"



//#define DECODER_DEBUGMSG

#ifdef DECODER_DEBUGMSG
#warning Decoder Debug messages enabled
#include "debug_port_messages.h"
#endif // DECODER_DEBUGMSG



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
        /**
        failed to load Decoder Config
        */
        Tuareg.errors.decoder_config_error= true;

        //enter limp mode
        Limp(TID_TUAREG_DECODER, DECODER_LOC_CONFIGLOAD_ERROR);

        //load built in defaults
        load_essential_Decoder_Setup();

        #ifdef DECODER_DEBUGMSG
        DebugMsg_Error("Failed to load Decoder Config!");
        DebugMsg_Warning("Decoder essential Config has been loaded");
        #endif // DECODER_DEBUGMSG
    }
    else if(Decoder_Setup.Version != DECODER_REQUIRED_CONFIG_VERSION)
    {
        /**
        loaded wrong Decoder Config Version
        */
        Tuareg.errors.decoder_config_error= true;

        //enter limp mode
        Limp(TID_TUAREG_DECODER, DECODER_LOC_CONFIGVERSION_ERROR);

        //load built in defaults
        load_essential_Decoder_Setup();

        #ifdef DECODER_DEBUGMSG
        DebugMsg_Error("Decoder Config version does not match");
        DebugMsg_Warning("Decoder essential Config has been loaded");
        #endif // DECODER_DEBUGMSG
    }
    else
    {
        /**
        loaded Decoder Config with correct Version
        */
        Tuareg.errors.decoder_config_error= false;


        #ifdef DECODER_DEBUGMSG
        print(DEBUG_PORT, "\r\nDecoder Config has been loaded");
        #endif // DECODER_DEBUGMSG
    }


    //init hw part
    init_decoder_hw();

    //init logic part
    init_decoder_logic();

    //report to syslog
    Syslog_Info(TID_TUAREG_DECODER, DECODER_LOC_READY);

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

