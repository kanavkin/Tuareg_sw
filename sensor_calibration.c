

#include "utils.h"
#include "table.h"
#include "storage.h"
#include "eeprom.h"
#include "eeprom_layout.h"
#include "config_pages.h"
#include "config_tables.h"
#include "sensor_calibration.h"
#include "eeprom_layout.h"

#include "Tuareg.h"

//DEBUG
#include "uart.h"
#include "uart_printf.h"
#include "conversion.h"



///the sensor calibration page
volatile Sensor_Calibration_t Sensor_Calibration;



/**
*
* reads sensor calibration data from eeprom
*
*/
exec_result_t load_Sensor_Calibration()
{
    U8 eeprom_data;
    U32 data;
    eeprom_result_t ee_result;


    /**
    Version
    */
    ee_result= eeprom_read_byte(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_VERSION, &eeprom_data);

    ASSERT_CONFIG_SUCESS(ee_result);

    Sensor_Calibration.Version= data;

    /**
    CLT
    */
    ee_result= eeprom_read_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_CLT_M, &data, 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    Sensor_Calibration.CLT_calib_M= compose_float(data);

    ee_result= eeprom_read_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_CLT_N, &data, 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    Sensor_Calibration.CLT_calib_N= compose_float(data);


    /**
    IAT
    */
    ee_result= eeprom_read_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_IAT_M, &data, 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    Sensor_Calibration.IAT_calib_M= compose_float(data);

    ee_result= eeprom_read_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_IAT_N, &data, 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    Sensor_Calibration.IAT_calib_N= compose_float(data);


    /**
    TPS
    */
    ee_result= eeprom_read_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_TPS_M, &data, 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    Sensor_Calibration.TPS_calib_M= compose_float(data);

    ee_result= eeprom_read_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_TPS_N, &data, 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    Sensor_Calibration.TPS_calib_N= compose_float(data);


    /**
    MAP
    */
    ee_result= eeprom_read_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_MAP_M, &data, 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    Sensor_Calibration.MAP_calib_M= compose_float(data);

    ee_result= eeprom_read_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_MAP_N, &data, 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    Sensor_Calibration.MAP_calib_N= compose_float(data);


    /**
    BARO
    */
    ee_result= eeprom_read_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_BARO_M, &data, 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    Sensor_Calibration.BARO_calib_M= compose_float(data);

    ee_result= eeprom_read_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_BARO_N, &data, 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    Sensor_Calibration.BARO_calib_N= compose_float(data);


    /**
    O2
    */
    ee_result= eeprom_read_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_O2_M, &data, 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    Sensor_Calibration.O2_calib_M= compose_float(data);

    ee_result= eeprom_read_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_O2_N, &data, 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    Sensor_Calibration.O2_calib_N= compose_float(data);


    /**
    VBAT
    */
    ee_result= eeprom_read_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_VBAT_M, &data, 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    Sensor_Calibration.VBAT_calib_M= compose_float(data);

    ee_result= eeprom_read_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_VBAT_N, &data, 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    Sensor_Calibration.VBAT_calib_N= compose_float(data);


    //all done
    return EXEC_OK;
}


/**
*
* writes sensor calibration data to eeprom
*
*/
exec_result_t write_Sensor_Calibration()
{
    eeprom_result_t ee_result;

    /**
    Version
    */
    ee_result= eeprom_update_byte(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_VERSION, Sensor_Calibration.Version);

    ASSERT_CONFIG_SUCESS(ee_result);

    /**
    IAT
    */
    ee_result= eeprom_update_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_IAT_M, serialize_float_U32(Sensor_Calibration.IAT_calib_M), 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    ee_result= eeprom_update_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_IAT_N, serialize_float_U32(Sensor_Calibration.IAT_calib_N), 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    /**
    CLT
    */
    ee_result= eeprom_update_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_CLT_M, serialize_float_U32(Sensor_Calibration.CLT_calib_M), 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    ee_result= eeprom_update_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_CLT_N, serialize_float_U32(Sensor_Calibration.CLT_calib_N), 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    /**
    TPS
    */
    ee_result= eeprom_update_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_TPS_M, serialize_float_U32(Sensor_Calibration.TPS_calib_M), 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    ee_result= eeprom_update_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_TPS_N, serialize_float_U32(Sensor_Calibration.TPS_calib_N), 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    /**
    MAP
    */
    ee_result= eeprom_update_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_MAP_M, serialize_float_U32(Sensor_Calibration.MAP_calib_M), 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    ee_result= eeprom_update_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_MAP_N, serialize_float_U32(Sensor_Calibration.MAP_calib_N), 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    /**
    BARO
    */
    ee_result= eeprom_update_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_BARO_M, serialize_float_U32(Sensor_Calibration.BARO_calib_M), 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    ee_result= eeprom_update_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_BARO_N, serialize_float_U32(Sensor_Calibration.BARO_calib_N), 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    /**
    O2
    */
    ee_result= eeprom_update_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_O2_M, serialize_float_U32(Sensor_Calibration.O2_calib_M), 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    ee_result= eeprom_update_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_O2_N, serialize_float_U32(Sensor_Calibration.O2_calib_N), 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    /**
    VBAT
    */
    ee_result= eeprom_update_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_VBAT_M, serialize_float_U32(Sensor_Calibration.VBAT_calib_M), 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    ee_result= eeprom_update_bytes(EEPROM_SENSOR_CALIBRATION_BEGIN_ADDRESS + SENSOR_CALIB_EE_VBAT_N, serialize_float_U32(Sensor_Calibration.VBAT_calib_N), 4);

    ASSERT_CONFIG_SUCESS(ee_result);

    //all done
    return EXEC_OK;
}



void show_Sensor_Calibration(USART_TypeDef * Port)
{
    print(Port, "\r\n\r\nSensor Calibration:");

    /*
    Version
    */
    print(Port, "\r\nVersion: ");
    printf_U(Port, Sensor_Calibration.Version, NO_PAD | NO_TRAIL);

    /*
    IAT
    */
    print(Port, "\r\nIAT M N: ");
    printf_F32(Port, Sensor_Calibration.IAT_calib_M);
    printf_F32(Port, Sensor_Calibration.IAT_calib_N);

    /*
    CLT
    */
    print(Port, "\r\nCLT M N: ");
    printf_F32(Port, Sensor_Calibration.CLT_calib_M);
    printf_F32(Port, Sensor_Calibration.CLT_calib_N);

    /*
    TPS
    */
    print(Port, "\r\nTPS M N: ");
    printf_F32(Port, Sensor_Calibration.TPS_calib_M);
    printf_F32(Port, Sensor_Calibration.TPS_calib_N);

    /*
    MAP
    */
    print(Port, "\r\nMAP M N: ");
    printf_F32(Port, Sensor_Calibration.MAP_calib_M);
    printf_F32(Port, Sensor_Calibration.MAP_calib_N);

    /*
    BARO
    */
    print(Port, "\r\nBARO M N: ");
    printf_F32(Port, Sensor_Calibration.BARO_calib_M);
    printf_F32(Port, Sensor_Calibration.BARO_calib_N);

    /*
    O2
    */
    print(Port, "\r\nO2 M N: ");
    printf_F32(Port, Sensor_Calibration.O2_calib_M);
    printf_F32(Port, Sensor_Calibration.O2_calib_N);

    /*
    VBAT
    */
    print(Port, "\r\nVBAT M N: ");
    printf_F32(Port, Sensor_Calibration.VBAT_calib_M);
    printf_F32(Port, Sensor_Calibration.VBAT_calib_N);

    /*
    KNOCK
    */
    print(Port, "\r\nKNOCK M N: ");
    printf_F32(Port, Sensor_Calibration.KNOCK_calib_M);
    printf_F32(Port, Sensor_Calibration.KNOCK_calib_N);

}


/**
replace a calibration value
*/
exec_result_t modify_Sensor_Calibration(U32 Offset, U32 Value)
{
    /**
    calibration layout:

    float IAT_calib_M;
    float IAT_calib_N;

    float CLT_calib_M;
    float CLT_calib_N;

    float TPS_calib_M;
    float TPS_calib_N;

    float MAP_calib_M;
    float MAP_calib_N;

    float BARO_calib_M;
    float BARO_calib_N;

    float O2_calib_M;
    float O2_calib_N;

    float VBAT_calib_M;
    float VBAT_calib_N;

    float KNOCK_calib_M;
    float KNOCK_calib_N;
    */

    switch(Offset)
    {
    case SENSOR_CALIB_CLI_IAT_M:

        Sensor_Calibration.IAT_calib_M= compose_float(Value);
        break;

    case SENSOR_CALIB_CLI_IAT_N:

        Sensor_Calibration.IAT_calib_N= compose_float(Value);
        break;


    case SENSOR_CALIB_CLI_CLT_M:

        Sensor_Calibration.CLT_calib_M= compose_float(Value);
        break;

    case SENSOR_CALIB_CLI_CLT_N:

        Sensor_Calibration.CLT_calib_N= compose_float(Value);
        break;


    case SENSOR_CALIB_CLI_TPS_M:

        Sensor_Calibration.TPS_calib_M= compose_float(Value);
        break;

    case SENSOR_CALIB_CLI_TPS_N:

        Sensor_Calibration.TPS_calib_N= compose_float(Value);
        break;


    case SENSOR_CALIB_CLI_MAP_M:

        Sensor_Calibration.MAP_calib_M= compose_float(Value);
        break;

    case SENSOR_CALIB_CLI_MAP_N:

        Sensor_Calibration.MAP_calib_N= compose_float(Value);
        break;


    case SENSOR_CALIB_CLI_BARO_M:

        Sensor_Calibration.BARO_calib_M= compose_float(Value);
        break;

    case SENSOR_CALIB_CLI_BARO_N:

        Sensor_Calibration.BARO_calib_N= compose_float(Value);
        break;


    case SENSOR_CALIB_CLI_O2_M:

        Sensor_Calibration.O2_calib_M= compose_float(Value);
        break;

    case SENSOR_CALIB_CLI_O2_N:

        Sensor_Calibration.O2_calib_N= compose_float(Value);
        break;


    case SENSOR_CALIB_CLI_VBAT_M:

        Sensor_Calibration.VBAT_calib_M= compose_float(Value);
        break;

    case SENSOR_CALIB_CLI_VBAT_N:

        Sensor_Calibration.VBAT_calib_N= compose_float(Value);
        break;


    case SENSOR_CALIB_CLI_KNOCK_M:

        Sensor_Calibration.KNOCK_calib_M= compose_float(Value);
        break;

    case SENSOR_CALIB_CLI_KNOCK_N:

        Sensor_Calibration.KNOCK_calib_N= compose_float(Value);
        break;

    default:
        return EXEC_ERROR;
        break;

    }

    return EXEC_OK;
}


void send_Sensor_Calibration(USART_TypeDef * Port)
{
    /*
    Version
    */
   // printf_U(Port, Sensor_Calibration.Version, NO_PAD | NO_TRAIL);

    //IAT
    send_float(Port, Sensor_Calibration.IAT_calib_M);
    send_float(Port, Sensor_Calibration.IAT_calib_N);

    //CLT
    send_float(Port, Sensor_Calibration.CLT_calib_M);
    send_float(Port, Sensor_Calibration.CLT_calib_N);

    //TPS
    send_float(Port, Sensor_Calibration.TPS_calib_M);
    send_float(Port, Sensor_Calibration.TPS_calib_N);

    //MAP
    send_float(Port, Sensor_Calibration.MAP_calib_M);
    send_float(Port, Sensor_Calibration.MAP_calib_N);

    //BARO
    send_float(Port, Sensor_Calibration.BARO_calib_M);
    send_float(Port, Sensor_Calibration.BARO_calib_N);

    //O2
    send_float(Port, Sensor_Calibration.O2_calib_M);
    send_float(Port, Sensor_Calibration.O2_calib_N);

    //VBAT
    send_float(Port, Sensor_Calibration.VBAT_calib_M);
    send_float(Port, Sensor_Calibration.VBAT_calib_N);

    //KNOCK
    send_float(Port, Sensor_Calibration.KNOCK_calib_M);
    send_float(Port, Sensor_Calibration.KNOCK_calib_N);
}
