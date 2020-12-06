

#include "utils.h"
#include "table.h"
#include "storage.h"
#include "eeprom.h"
#include "eeprom_layout.h"
#include "config_pages.h"
#include "config_tables.h"
#include "ignition_config.h"
#include "eeprom_layout.h"

#include "Tuareg.h"

//DEBUG
#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"


///the ignition configuration page
volatile Ignition_Config_t Ignition_Config;


/**
*
* reads ignition config data from eeprom
*
*/
exec_result_t load_Ignition_Config()
{
    U32 data;
    U8 eeprom_data;
    eeprom_result_t ee_result;


    /**
    Version
    */
    ee_result= eeprom_read_byte(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_VERSION, &eeprom_data);

    ASSERT_CONFIG_SUCESS(ee_result);

    Ignition_Config.Version= data;


    //U16 max_rpm
    ee_result= eeprom_read_bytes(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_MAX_RPM, &data, 2);

    ASSERT_CONFIG_SUCESS(ee_result);

    Ignition_Config.max_rpm= (U16) data;


    //U16 dynamic_min_rpm
    ee_result= eeprom_read_bytes(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_DYN_MIN_RPM, &data, 2);

    ASSERT_CONFIG_SUCESS(ee_result);

    Ignition_Config.dynamic_min_rpm= (U16) data;


    //crank_position_t dynamic_ignition_base_position
    ee_result= eeprom_read_byte(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_DYN_IGN_BASE_POS, &eeprom_data);

    ASSERT_CONFIG_SUCESS(ee_result);

    Ignition_Config.dynamic_ignition_base_position= eeprom_data;


    //U16 dynamic_dwell_target_us
    ee_result= eeprom_read_bytes(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_DYN_DWELL_TGT, &data, 2);

    ASSERT_CONFIG_SUCESS(ee_result);

    Ignition_Config.dynamic_dwell_target_us= (U16) data;


    //U16 cold_idle_cutoff_rpm
    ee_result= eeprom_read_bytes(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_COLD_IDLE_CUTOFF_RPM, &data, 2);

    ASSERT_CONFIG_SUCESS(ee_result);

    Ignition_Config.cold_idle_cutoff_rpm= (U16) data;


    //U16 cold_idle_cutoff_CLT_K
    ee_result= eeprom_read_bytes(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_COLD_IDLE_CUTOFF_CLT, &data, 2);

    ASSERT_CONFIG_SUCESS(ee_result);

    Ignition_Config.cold_idle_cutoff_CLT_K= (U16) data;


    //U8 cold_idle_ignition_advance_deg
    ee_result= eeprom_read_byte(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_COLD_IDLE_IGN_ADV, &eeprom_data);

    ASSERT_CONFIG_SUCESS(ee_result);

    Ignition_Config.cold_idle_ignition_advance_deg= eeprom_data;


    //U16 cold_idle_dwell_target_us
    ee_result= eeprom_read_bytes(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_COLD_IDLE_DWELL_TGT, &data, 2);

    ASSERT_CONFIG_SUCESS(ee_result);

    Ignition_Config.cold_idle_cutoff_CLT_K= (U16) data;


    //crank_position_t cranking_ignition_position
    ee_result= eeprom_read_byte(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_CRANKING_IGN_POS, &eeprom_data);

    ASSERT_CONFIG_SUCESS(ee_result);

    Ignition_Config.cranking_ignition_position= eeprom_data;


    //crank_position_t cranking_dwell_position
    ee_result= eeprom_read_byte(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_CRANKING_DWELL_POS, &eeprom_data);

    ASSERT_CONFIG_SUCESS(ee_result);

    Ignition_Config.cranking_dwell_position= eeprom_data;


    //coil_setup_t coil_setup
    ee_result= eeprom_read_byte(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_COIL_SETUP, &eeprom_data);

    ASSERT_CONFIG_SUCESS(ee_result);

    Ignition_Config.coil_setup= eeprom_data;


    //U16 spark_duration_us
    ee_result= eeprom_read_bytes(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_SPARK_DURATION, &data, 2);

    ASSERT_CONFIG_SUCESS(ee_result);

    Ignition_Config.spark_duration_us= (U16) data;


    //all done
    return EXEC_OK;
}


/**
*
* provides sane defaults if config data from eeprom is not available (limp home mode)
*
*/
void load_essential_Ignition_Config()
{



}


/**
*
* writes ignition config data to eeprom
*
*/
exec_result_t write_Ignition_Config()
{
    eeprom_result_t ee_result;


    /**
    Version
    */
    ee_result= eeprom_update_byte(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + IGNITION_CONFIG_EE_VERSION, Ignition_Config.Version);

    ASSERT_CONFIG_SUCESS(ee_result);

    //U16 max_rpm
    ee_result= eeprom_update_bytes(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_MAX_RPM, Ignition_Config.max_rpm, 2);

    ASSERT_CONFIG_SUCESS(ee_result);


    //U16 dynamic_min_rpm
    ee_result= eeprom_update_bytes(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_DYN_MIN_RPM, Ignition_Config.dynamic_min_rpm, 2);

    ASSERT_CONFIG_SUCESS(ee_result);


    //crank_position_t dynamic_ignition_base_position
    ee_result= eeprom_update_byte(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_DYN_IGN_BASE_POS, Ignition_Config.dynamic_ignition_base_position);

    ASSERT_CONFIG_SUCESS(ee_result);


    //U16 dynamic_dwell_target_us
    ee_result= eeprom_update_bytes(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_DYN_DWELL_TGT, Ignition_Config.dynamic_dwell_target_us, 2);

    ASSERT_CONFIG_SUCESS(ee_result);


    //U16 cold_idle_cutoff_rpm
    ee_result= eeprom_update_bytes(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_COLD_IDLE_CUTOFF_RPM, Ignition_Config.cold_idle_cutoff_rpm, 2);

    ASSERT_CONFIG_SUCESS(ee_result);


    //U16 cold_idle_cutoff_CLT_K
    ee_result= eeprom_update_bytes(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_COLD_IDLE_CUTOFF_CLT, Ignition_Config.cold_idle_cutoff_CLT_K, 2);

    ASSERT_CONFIG_SUCESS(ee_result);


    //U8 cold_idle_ignition_advance_deg
    ee_result= eeprom_update_byte(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_COLD_IDLE_IGN_ADV, Ignition_Config.cold_idle_ignition_advance_deg);

    ASSERT_CONFIG_SUCESS(ee_result);


    //U16 cold_idle_dwell_target_us
    ee_result= eeprom_update_bytes(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_COLD_IDLE_DWELL_TGT, Ignition_Config.cold_idle_dwell_target_us, 2);

    ASSERT_CONFIG_SUCESS(ee_result);


    //crank_position_t cranking_ignition_position
    ee_result= eeprom_update_byte(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_CRANKING_IGN_POS, Ignition_Config.cranking_ignition_position);

    ASSERT_CONFIG_SUCESS(ee_result);


    //crank_position_t cranking_dwell_position
    ee_result= eeprom_update_byte(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_CRANKING_DWELL_POS, Ignition_Config.cranking_dwell_position);

    ASSERT_CONFIG_SUCESS(ee_result);


    //coil_setup_t coil_setup
    ee_result= eeprom_update_byte(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_COIL_SETUP, Ignition_Config.coil_setup);

    ASSERT_CONFIG_SUCESS(ee_result);


    //U16 spark_duration_us
    ee_result= eeprom_update_bytes(EEPROM_IGNITION_CONFIG_BEGIN_ADDRESS + IGNITION_CONFIG_EE_SPARK_DURATION, Ignition_Config.spark_duration_us, 2);

    ASSERT_CONFIG_SUCESS(ee_result);


    //all done
    return EXEC_OK;
}


void show_Ignition_Config(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nIgnition Config:");

    //max_rpm
    print(Port, "\r\nrev limiter (rpm): ");
    printf_U(Port, Ignition_Config.max_rpm, NO_PAD);


    //dynamic_min_rpm
    print(Port, "\r\ndynamic ignition function minimum rpm: ");
    printf_U(Port, Ignition_Config.dynamic_min_rpm, NO_PAD);

    //dynamic_ignition_base_position
    print(Port, "\r\ndynamic ignition base position: ");
    printf_crkpos(Port, Ignition_Config.dynamic_ignition_base_position);

    //dynamic_dwell_target_us
    print(Port, "\r\ndynamic dwell target (us): ");
    printf_U(Port, Ignition_Config.dynamic_dwell_target_us, NO_PAD);


    //cold_idle_cutoff_rpm
    print(Port, "\r\ncold idle cutoff (rpm): ");
    printf_U(Port, Ignition_Config.cold_idle_cutoff_rpm, NO_PAD);

    //cold_idle_cutoff_CLT_K
    print(Port, "\r\ncold idle cutoff CLT (K): ");
    printf_U(Port, Ignition_Config.cold_idle_cutoff_CLT_K, NO_PAD);

    //cold_idle_ignition_advance_deg
    print(Port, "\r\ncold idle ignition advance (deg): ");
    printf_U(Port, Ignition_Config.cold_idle_ignition_advance_deg, NO_PAD);

    //cold_idle_dwell_target_us
    print(Port, "\r\ncold idle dwell target (us): ");
    printf_U(Port, Ignition_Config.cold_idle_dwell_target_us, NO_PAD);


    //cranking_ignition_position
    print(Port, "\r\ncranking ignition position: ");
    printf_crkpos(Port, Ignition_Config.cranking_ignition_position);

    //cranking_dwell_position
    print(Port, "\r\ncranking dwell position: ");
    printf_crkpos(Port, Ignition_Config.cranking_dwell_position);


    //coil_setup_t coil_setup
    print(Port, "\r\ncoil setup: ");
    printf_U(Port, Ignition_Config.coil_setup, NO_PAD);

    //U16 spark_duration_us
    print(Port, "\r\nspark duration (us): ");
    printf_U(Port, Ignition_Config.spark_duration_us, NO_PAD);
}


/**
replace an ignition configuration value
*/
exec_result_t modify_Ignition_Config(U32 Offset, U32 Value)
{
    crank_position_t input_pos;

    switch(Offset)
    {
    case IGNITION_CONFIG_CLI_MAX_RPM:

        Ignition_Config.max_rpm= (U16) Value;
        break;

    case IGNITION_CONFIG_CLI_DYN_MIN_RPM:

        Ignition_Config.dynamic_min_rpm= (U16) Value;
        break;

    case IGNITION_CONFIG_CLI_DYN_IGN_BASE_POS:

        input_pos= parse_position(Value);

        ASSERT_CRANK_POS(input_pos);

        Ignition_Config.dynamic_ignition_base_position= input_pos;
        break;

    case IGNITION_CONFIG_CLI_DYN_DWELL_TGT:

        Ignition_Config.dynamic_dwell_target_us= (U16) Value;
        break;

    case IGNITION_CONFIG_CLI_COLD_IDLE_CUTOFF_RPM:

        Ignition_Config.cold_idle_cutoff_rpm= (U16) Value;
        break;

    case IGNITION_CONFIG_CLI_COLD_IDLE_CUTOFF_CLT:

        Ignition_Config.cold_idle_cutoff_CLT_K= (U16) Value;
        break;

    case IGNITION_CONFIG_CLI_COLD_IDLE_IGN_ADV:

        Ignition_Config.cold_idle_ignition_advance_deg= (U8) Value;
        break;

    case IGNITION_CONFIG_CLI_COLD_IDLE_DWELL_TGT:

        Ignition_Config.cold_idle_dwell_target_us= (U16) Value;
        break;

    case IGNITION_CONFIG_CLI_CRANKING_IGN_POS:

        input_pos= parse_position(Value);

        ASSERT_CRANK_POS(input_pos);

        Ignition_Config.cranking_ignition_position= input_pos;
        break;

    case IGNITION_CONFIG_CLI_CRANKING_DWELL_POS:

        input_pos= parse_position(Value);

        ASSERT_CRANK_POS(input_pos);

        Ignition_Config.cranking_dwell_position= input_pos;
        break;


    case IGNITION_CONFIG_CLI_COIL_SETUP:

        ASSERT_IGNITION_SETUP(Value);

        Ignition_Config.coil_setup= Value;
        break;


    case IGNITION_CONFIG_CLI_SPARK_DURATION:

        Ignition_Config.spark_duration_us= Value;
        break;

    default:

        return EXEC_ERROR;
        break;

    }

    return EXEC_OK;

}




