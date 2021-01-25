

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
    Decoder_Setup.crank_noise_filter= DECODER_CONFIG_DEFAULT_CRANK_NOISE_FILTER;
    Decoder_Setup.sync_check_begin_pos= DECODER_CONFIG_DEFAULT_SYNC_CHECK_BEGIN_POS;
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
    print(Port, "\r\n\r\nDecoder Config:");

    /*
    Version
    */
    print(Port, "\r\nVersion: ");
    printf_U(Port, Decoder_Setup.Version, NO_PAD | NO_TRAIL);

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


