

#include "utils.h"
#include "table.h"
#include "storage.h"
#include "eeprom.h"
#include "eeprom_layout.h"
#include "config_pages.h"
#include "config_tables.h"
#include "eeprom_layout.h"

#include "Tuareg.h"

#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"


///the decoder configuration page
volatile Decoder_Setup_t Decoder_Setup;

volatile U8 * const pDecoder_Setup_data= (volatile U8 *) &Decoder_Setup;
const U32 cDecoder_Setup_size= sizeof(Decoder_Setup);


/**
*
* reads decoder config data from eeprom
*
*/
exec_result_t load_Decoder_Setup()
{
   return Eeprom_load_data(EEPROM_DECODER_CONFIG_BASE, pDecoder_Setup_data, cDecoder_Setup_size);
}


/**
*
* provides sane defaults if config data from eeprom is not available (limp home mode)
*
*/
void load_essential_Decoder_Setup()
{

    /**
    trigger position map initialisation shall be done in a way independent of CRK_POSITION_XX enumerator order
    */
    Decoder_Setup.trigger_position_map.crank_angle_deg[CRK_POSITION_A1]= DECODER_CONFIG_DEFAULT_POSITION_A1_ANGLE;
    Decoder_Setup.trigger_position_map.crank_angle_deg[CRK_POSITION_A2]= DECODER_CONFIG_DEFAULT_POSITION_A2_ANGLE;
    Decoder_Setup.trigger_position_map.crank_angle_deg[CRK_POSITION_B1]= DECODER_CONFIG_DEFAULT_POSITION_B1_ANGLE;
    Decoder_Setup.trigger_position_map.crank_angle_deg[CRK_POSITION_B2]= DECODER_CONFIG_DEFAULT_POSITION_B2_ANGLE;
    Decoder_Setup.trigger_position_map.crank_angle_deg[CRK_POSITION_C1]= DECODER_CONFIG_DEFAULT_POSITION_C1_ANGLE;
    Decoder_Setup.trigger_position_map.crank_angle_deg[CRK_POSITION_C2]= DECODER_CONFIG_DEFAULT_POSITION_C2_ANGLE;
    Decoder_Setup.trigger_position_map.crank_angle_deg[CRK_POSITION_D1]= DECODER_CONFIG_DEFAULT_POSITION_D1_ANGLE;
    Decoder_Setup.trigger_position_map.crank_angle_deg[CRK_POSITION_D2]= DECODER_CONFIG_DEFAULT_POSITION_D2_ANGLE;

    Decoder_Setup.trigger_offset_deg= DECODER_CONFIG_DEFAULT_TRIGGER_OFFSET;
    Decoder_Setup.vr_delay_us= DECODER_CONFIG_DEFAULT_VR_DELAY;
    Decoder_Setup.crank_noise_filter= DECODER_CONFIG_DEFAULT_CRANK_NOISE_FILTER;
    Decoder_Setup.sync_ratio_min_pct= DECODER_CONFIG_DEFAULT_SYNC_RATIO_MIN;
    Decoder_Setup.sync_ratio_max_pct= DECODER_CONFIG_DEFAULT_SYNC_RATIO_MAX;
    Decoder_Setup.timeout_s= DECODER_CONFIG_DEFAULT_TIMEOUT;

}

/**
*
* writes decoder config data to eeprom
*
*/
exec_result_t store_Decoder_Setup()
{
    return Eeprom_update_data(EEPROM_DECODER_CONFIG_BASE, pDecoder_Setup_data, cDecoder_Setup_size);
}


void show_Decoder_Setup(USART_TypeDef * Port)
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
        printf_U(Port, Decoder_Setup.trigger_position_map.crank_angle_deg[pos], NO_PAD);
    }

    print(Port, "\r\ntrigger offset (deg): ");
    printf_S(Port, Decoder_Setup.trigger_offset_deg, NO_PAD);


    print(Port, "\r\nvr delay (us): ");
    printf_U(Port, Decoder_Setup.vr_delay_us, NO_PAD);


    print(Port, "\r\ncrank noise filter: ");
    printf_U(Port, Decoder_Setup.crank_noise_filter, NO_PAD);

    print(Port, "\r\nsync ratio min (pct): ");
    printf_U(Port, Decoder_Setup.sync_ratio_min_pct, NO_PAD);

    print(Port, "\r\nsync ratio max (pct): ");
    printf_U(Port, Decoder_Setup.sync_ratio_max_pct, NO_PAD);

    print(Port, "\r\ntimeout (s): ");
    printf_U(Port, Decoder_Setup.timeout_s, NO_PAD);
}


/**
replace a decoder configuration value
*/
exec_result_t modify_Decoder_Setup(U32 Offset, U32 Value)
{
    if(Offset >= cDecoder_Setup_size)
    {
        return EXEC_ERROR;
    }

    *(pDecoder_Setup_data + Offset)= (U8) Value;

    return EXEC_OK;
}


/**
this function implements the TS interface binary config page read command for Decoder Config
*/
void send_Decoder_Setup(USART_TypeDef * Port)
{
    UART_send_data(Port, pDecoder_Setup_data, cDecoder_Setup_size);
}


