#include <Tuareg_platform.h>
#include <Tuareg.h>

//#define DECODER_DEBUGMSG

#ifdef DECODER_DEBUGMSG
#warning Decoder Debug messages enabled
#endif // DECODER_DEBUGMSG



/******************************************************************************************************************************
Decoder initialization
 ******************************************************************************************************************************/
void Init_Decoder()
{
    exec_result_t result;

    //setup shall be loaded first
    result= load_Decoder_Setup();

    //check if config has been loaded
    if((result != EXEC_OK) || (Decoder_Setup.Version != DECODER_REQUIRED_CONFIG_VERSION))
    {
        /**
        failed to load Decoder Config
        */
        Tuareg.errors.decoder_config_error= true;

        //enter limp mode
        Fatal(TID_TUAREG_DECODER, DECODER_LOC_CONFIGLOAD_ERROR);

        #ifdef DECODER_DEBUGMSG
        DebugMsg_Error("Failed to load Decoder Config!");
        DebugMsg_Warning("Decoder essential Config has been loaded");
        #endif // DECODER_DEBUGMSG

        return;
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
}


/******************************************************************************************************************************
Decoder shutdown
 ******************************************************************************************************************************/
void disable_Decoder()
{
    //disable hw
    disable_decoder_hw();

    //disable logic
    disable_decoder_logic();

    //report to syslog
    Syslog_Info(TID_TUAREG_DECODER, DECODER_LOC_HALTED);


    #ifdef DECODER_DEBUGMSG
    DebugMsg_Warning("Decoder disabled");
    #endif // DECODER_DEBUGMSG
}


/******************************************************************************************************************************
calculate position data age
******************************************************************************************************************************/

U32 decoder_get_position_data_age_us()
{
    U32 now_ts, update_ts, interval_us;

    now_ts= decoder_get_timestamp();
    update_ts= Decoder_hw.current_timer_value;

    //check counting mode
    if(Decoder_hw.state.timer_continuous_mode == true)
    {
        //timer continuously counting since last position update
        interval_us= Decoder_hw.timer_period_us * subtract_U32(now_ts, update_ts);
    }
    else
    {
        //timer has been reset on last position update
        interval_us= Decoder_hw.timer_period_us * now_ts;
    }

    return interval_us;

}

