

#include "utils.h"
#include "table.h"
#include "storage.h"
#include "eeprom.h"
#include "eeprom_layout.h"
#include "config_pages.h"
#include "config_tables.h"
#include "eeprom_layout.h"

#include "Tuareg.h"

//DEBUG
#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"



///the decoder configuration page
volatile Decoder_Config_t Decoder_Config;


/**
*
* reads decoder config data from eeprom
*
*/
exec_result_t load_Decoder_Config()
{
    U32 data, pos;
    U8 eeprom_data;
    eeprom_result_t ee_result;


    /**
    Version
    */
    ee_result= eeprom_read_byte(EEPROM_DECODER_CONFIG_BEGIN_ADDRESS + DECODER_CONFIG_EE_VERSION, &eeprom_data);

    ASSERT_CONFIG_SUCESS(ee_result);

    Decoder_Config.Version= data;


    //U16 trigger_position_map[POSITION_COUNT]
    for(pos=0; pos < CRK_POSITION_COUNT; pos++)
    {
        //every config item is 2 bytes in width
        ee_result= eeprom_read_bytes(EEPROM_DECODER_CONFIG_BEGIN_ADDRESS + DECODER_CONFIG_EE_TRIGGERMAP + 2*pos, &data, 2);

        ASSERT_CONFIG_SUCESS(ee_result);

        Decoder_Config.trigger_position_map.crank_angle_deg[pos]= (U16) data;
    }


    //S16 decoder_offset_deg
    ee_result= eeprom_read_bytes(EEPROM_DECODER_CONFIG_BEGIN_ADDRESS + DECODER_CONFIG_EE_TRIGGER_OFFSET, &data, 2);

    ASSERT_CONFIG_SUCESS(ee_result);

    Decoder_Config.trigger_offset_deg= (S16) data;


    //U16 decoder_delay_us
    ee_result= eeprom_read_bytes(EEPROM_DECODER_CONFIG_BEGIN_ADDRESS + DECODER_CONFIG_EE_VR_DELAY, &data, 2);

    ASSERT_CONFIG_SUCESS(ee_result);

    Decoder_Config.vr_delay_us= (U16) data;


    //U8 crank_noise_filter
    ee_result= eeprom_read_byte(EEPROM_DECODER_CONFIG_BEGIN_ADDRESS + DECODER_CONFIG_EE_CRANK_NOISE_FILTER, &eeprom_data);

    ASSERT_CONFIG_SUCESS(ee_result);

    Decoder_Config.crank_noise_filter= eeprom_data;


    //U8 sync_ratio_min_pct
    ee_result= eeprom_read_byte(EEPROM_DECODER_CONFIG_BEGIN_ADDRESS + DECODER_CONFIG_EE_SYNC_RATIO_MIN, &eeprom_data);

    ASSERT_CONFIG_SUCESS(ee_result);

    Decoder_Config.sync_ratio_min_pct= eeprom_data;


    //U8 sync_ratio_max_pct
    ee_result= eeprom_read_byte(EEPROM_DECODER_CONFIG_BEGIN_ADDRESS + DECODER_CONFIG_EE_SYNC_RATIO_MAX, &eeprom_data);

    ASSERT_CONFIG_SUCESS(ee_result);

    Decoder_Config.sync_ratio_max_pct= eeprom_data;


    //U8 decoder_timeout_s
    ee_result= eeprom_read_byte(EEPROM_DECODER_CONFIG_BEGIN_ADDRESS + DECODER_CONFIG_EE_TIMEOUT, &eeprom_data);

    ASSERT_CONFIG_SUCESS(ee_result);

    Decoder_Config.timeout_s= eeprom_data;

    //all done
    return EXEC_OK;
}


/**
*
* provides sane defaults if config data from eeprom is not available (limp home mode)
*
*/
void load_essential_Decoder_Config()
{

    /**
    trigger position map initialisation shall be done in a way independent of CRK_POSITION_XX enumerator order
    */
    Decoder_Config.trigger_position_map.crank_angle_deg[CRK_POSITION_A1]= DECODER_CONFIG_DEFAULT_POSITION_A1_ANGLE;
    Decoder_Config.trigger_position_map.crank_angle_deg[CRK_POSITION_A2]= DECODER_CONFIG_DEFAULT_POSITION_A2_ANGLE;
    Decoder_Config.trigger_position_map.crank_angle_deg[CRK_POSITION_B1]= DECODER_CONFIG_DEFAULT_POSITION_B1_ANGLE;
    Decoder_Config.trigger_position_map.crank_angle_deg[CRK_POSITION_B2]= DECODER_CONFIG_DEFAULT_POSITION_B2_ANGLE;
    Decoder_Config.trigger_position_map.crank_angle_deg[CRK_POSITION_C1]= DECODER_CONFIG_DEFAULT_POSITION_C1_ANGLE;
    Decoder_Config.trigger_position_map.crank_angle_deg[CRK_POSITION_C2]= DECODER_CONFIG_DEFAULT_POSITION_C2_ANGLE;
    Decoder_Config.trigger_position_map.crank_angle_deg[CRK_POSITION_D1]= DECODER_CONFIG_DEFAULT_POSITION_D1_ANGLE;
    Decoder_Config.trigger_position_map.crank_angle_deg[CRK_POSITION_D2]= DECODER_CONFIG_DEFAULT_POSITION_D2_ANGLE;

    Decoder_Config.trigger_offset_deg= DECODER_CONFIG_DEFAULT_TRIGGER_OFFSET;
    Decoder_Config.vr_delay_us= DECODER_CONFIG_DEFAULT_VR_DELAY;
    Decoder_Config.crank_noise_filter= DECODER_CONFIG_DEFAULT_CRANK_NOISE_FILTER;
    Decoder_Config.sync_ratio_min_pct= DECODER_CONFIG_DEFAULT_SYNC_RATIO_MIN;
    Decoder_Config.sync_ratio_max_pct= DECODER_CONFIG_DEFAULT_SYNC_RATIO_MAX;
    Decoder_Config.timeout_s= DECODER_CONFIG_DEFAULT_TIMEOUT;

}

/**
*
* writes decoder config data to eeprom
*
*/
exec_result_t write_Decoder_Config()
{
    U32 pos;
    eeprom_result_t ee_result;


    /**
    Version
    */
    ee_result= eeprom_update_byte(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + DECODER_CONFIG_EE_VERSION, Decoder_Config.Version);

    ASSERT_CONFIG_SUCESS(ee_result);


    //U16 trigger_position_map[CRK_POSITION_COUNT]
    for(pos=0; pos < CRK_POSITION_COUNT; pos++)
    {
        //every config item is 2 bytes in width
        ee_result= eeprom_update_bytes(EEPROM_DECODER_CONFIG_BEGIN_ADDRESS + DECODER_CONFIG_EE_TRIGGERMAP + 2*pos, Decoder_Config.trigger_position_map.crank_angle_deg[pos], 2);

        ASSERT_CONFIG_SUCESS(ee_result);
    }


    //S16 decoder_offset_deg
    ee_result= eeprom_update_bytes(EEPROM_DECODER_CONFIG_BEGIN_ADDRESS + DECODER_CONFIG_EE_TRIGGER_OFFSET, Decoder_Config.trigger_offset_deg, 2);

    ASSERT_CONFIG_SUCESS(ee_result);


    //U16 decoder_delay_us
    ee_result= eeprom_update_bytes(EEPROM_DECODER_CONFIG_BEGIN_ADDRESS + DECODER_CONFIG_EE_VR_DELAY, Decoder_Config.vr_delay_us, 2);

    ASSERT_CONFIG_SUCESS(ee_result);


    //U8 crank_noise_filter
    ee_result= eeprom_update_byte(EEPROM_DECODER_CONFIG_BEGIN_ADDRESS + DECODER_CONFIG_EE_CRANK_NOISE_FILTER, Decoder_Config.crank_noise_filter);

    ASSERT_CONFIG_SUCESS(ee_result);


    //U8 sync_ratio_min_pct
    ee_result= eeprom_update_byte(EEPROM_DECODER_CONFIG_BEGIN_ADDRESS + DECODER_CONFIG_EE_SYNC_RATIO_MIN, Decoder_Config.sync_ratio_min_pct);

    ASSERT_CONFIG_SUCESS(ee_result);


    //U8 sync_ratio_max_pct
    ee_result= eeprom_update_byte(EEPROM_DECODER_CONFIG_BEGIN_ADDRESS + DECODER_CONFIG_EE_SYNC_RATIO_MAX, Decoder_Config.sync_ratio_max_pct);

    ASSERT_CONFIG_SUCESS(ee_result);


    //U8 decoder_timeout_s
    ee_result= eeprom_update_byte(EEPROM_DECODER_CONFIG_BEGIN_ADDRESS + DECODER_CONFIG_EE_TIMEOUT, Decoder_Config.timeout_s);

    ASSERT_CONFIG_SUCESS(ee_result);

    //all done
    return EXEC_OK;
}


void show_Decoder_Config(USART_TypeDef * Port)
{
    U32 pos;

    print(Port, "\r\n\r\nDecoder Config:");

    /*
    Version
    */
    print(Port, "\r\nVersion: ");
    printf_U(Port, Sensor_Calibration.Version, NO_PAD | NO_TRAIL);

    /*
    trigger_position_map[CRK_POSITION_COUNT]
    */
    print(Port, "\r\nTrigger position map (deg after A1) \r\n");

    for(pos=0; pos< CRK_POSITION_COUNT; pos++)
    {
        printf_crkpos(Port, pos);
        UART_Tx(Port, ':');
        printf_U(Port, Decoder_Config.trigger_position_map.crank_angle_deg[pos], NO_PAD);
    }

    print(Port, "\r\ntrigger offset (deg): ");
    printf_S(Port, Decoder_Config.trigger_offset_deg, NO_PAD);


    print(Port, "\r\nvr delay (us): ");
    printf_U(Port, Decoder_Config.vr_delay_us, NO_PAD);


    print(Port, "\r\ncrank noise filter: ");
    printf_U(Port, Decoder_Config.crank_noise_filter, NO_PAD);

    print(Port, "\r\nsync ratio min (pct): ");
    printf_U(Port, Decoder_Config.sync_ratio_min_pct, NO_PAD);

    print(Port, "\r\nsync ratio max (pct): ");
    printf_U(Port, Decoder_Config.sync_ratio_max_pct, NO_PAD);

    print(Port, "\r\ntimeout (s): ");
    printf_U(Port, Decoder_Config.timeout_s, NO_PAD);
}


/**
replace a decoder configuration value
*/
exec_result_t modify_Decoder_Config(U32 Offset, U32 Value)
{
    switch(Offset)
    {
    case DECODER_CONFIG_CLI_TRIGGERMAP_A1:

        ASSERT_TRIGGER_ANGLE(Value);
        Decoder_Config.trigger_position_map.crank_angle_deg[CRK_POSITION_A1]= (U16) Value;
        break;

    case DECODER_CONFIG_CLI_TRIGGERMAP_A2:

        ASSERT_TRIGGER_ANGLE(Value);
        Decoder_Config.trigger_position_map.crank_angle_deg[CRK_POSITION_A2]= (U16) Value;
        break;

    case DECODER_CONFIG_CLI_TRIGGERMAP_B1:

        ASSERT_TRIGGER_ANGLE(Value);
        Decoder_Config.trigger_position_map.crank_angle_deg[CRK_POSITION_B1]= (U16) Value;
        break;

    case DECODER_CONFIG_CLI_TRIGGERMAP_B2:

        ASSERT_TRIGGER_ANGLE(Value);
        Decoder_Config.trigger_position_map.crank_angle_deg[CRK_POSITION_B2]= (U16) Value;
        break;

    case DECODER_CONFIG_CLI_TRIGGERMAP_C1:

        ASSERT_TRIGGER_ANGLE(Value);
        Decoder_Config.trigger_position_map.crank_angle_deg[CRK_POSITION_C1]= (U16) Value;
        break;

    case DECODER_CONFIG_CLI_TRIGGERMAP_C2:

        ASSERT_TRIGGER_ANGLE(Value);
        Decoder_Config.trigger_position_map.crank_angle_deg[CRK_POSITION_C2]= (U16) Value;
        break;

    case DECODER_CONFIG_CLI_TRIGGERMAP_D1:

        ASSERT_TRIGGER_ANGLE(Value);
        Decoder_Config.trigger_position_map.crank_angle_deg[CRK_POSITION_D1]= (U16) Value;
        break;

    case DECODER_CONFIG_CLI_TRIGGERMAP_D2:

        ASSERT_TRIGGER_ANGLE(Value);
        Decoder_Config.trigger_position_map.crank_angle_deg[CRK_POSITION_D2]= (U16) Value;
        break;

    case DECODER_CONFIG_CLI_TRIGGER_OFFSET:

        Decoder_Config.trigger_offset_deg= (S16) Value;
        break;

    case DECODER_CONFIG_CLI_VR_DELAY:

        Decoder_Config.vr_delay_us= (U16) Value;
        break;

    case DECODER_CONFIG_CLI_CRANK_NOISE_FILTER:

        Decoder_Config.crank_noise_filter= (U8) Value;
        break;

    case DECODER_CONFIG_CLI_SYNC_RATIO_MIN:

        Decoder_Config.sync_ratio_min_pct= (U8) Value;
        break;

    case DECODER_CONFIG_CLI_SYNC_RATIO_MAX:

        Decoder_Config.sync_ratio_max_pct= (U8) Value;
        break;

    case DECODER_CONFIG_CLI_TIMEOUT:

        Decoder_Config.timeout_s= (U8) Value;
        break;

    default:
        return EXEC_ERROR;
        break;

    }

    return EXEC_OK;
}


