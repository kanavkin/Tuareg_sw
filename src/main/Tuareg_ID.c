#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/boctok_types.h"

#include "conversion.h"
#include "uart.h"
#include "uart_printf.h"

#include "Tuareg_errors.h"


/******************************************************************************************************
Tuareg ID labels
*******************************************************************************************************/


#define TUAREG_ID_LABELS_LEN 30


const char Tuareg_ID_labels [TID_COUNT] [TUAREG_ID_LABELS_LEN] __attribute__((__section__(".rodata"))) = {

    "INVALID",
    "ACTORS_HW",
    "ACTORS_LOGIC",
    "SERIAL_MONITOR",
    "TUAREG_CONSOLE",
    "TUNERSTUDIO",
    "TUNERSTUDIO_OUTCHANNEL",
    "TUNERSTUDIO_SERVICE",
    "UART",
    "UART_PRINTF",

    "DASH_HW",
    "TID_TUAREG_DASH",

    "DEBUG",
    "DWT",
    "DECODER_CONFIG",
    "DECODER_HW",
    "DECODER_LOGIC",
    "TUAREG_DECODER",
    "DIAGNOSTICS",
    "FAULTLOG",

    "TID_FUELING_ACCELCOMP",
    "FUELING_CONFIG",
    "TID_FUELING_CORRECTIONS",
    "TID_FUELING_DIAG",
    "TID_FUELING_HW",
    "TID_FUELING_INJECTOR_PARAMS",
    "TUAREG_FUELING",
    "FUELING_CONTROLS",

    "SYSLOG",
    "IGNITION_CONFIG",
    "IGNITION_HW",
    "TUAREG_IGNITION",
    "TUAREG_IGNITION_CONTROLS",
    "MAIN",
    "TUAREG",
    "TUAREG_CONFIG",
    "TUAREG_ERRORS",
    "TUAREG_POCESS_DATA",
    "TUAREG_TYPES",
    "PROCESS_TABLE",
    "TUAREG_SENSORS",
    "SENSOR_CALIBRATION",
    "SENSORS",
    "SERVICE",
    "EEPROM",
    "TABLE",
    "LOWPRIO_SCHEDULER",
    "SCHEDULER",
    "SYSTICK_TIMER",
    "BASE_CALC",
    "BITFIELDS",
    "CONVERSION",
    "DUMMY"

};



/******************************************************************************************************
print Tuareg ID labels
*******************************************************************************************************/

void print_Tuareg_ID_label(USART_TypeDef * Port, Tuareg_ID Id)
{
    VitalAssert(Id < TID_COUNT, 0, 0);

    print_flash(Port, Tuareg_ID_labels[Id]);
}
