#ifndef TUAREG_ID_H_INCLUDED
#define TUAREG_ID_H_INCLUDED


typedef enum {

    TID_INVALID,
    TID_ACTORS_HW,
    TID_ACTORS_LOGIC,
    TID_SERIAL_MONITOR,
    TID_TUAREG_CONSOLE,
    TID_TUNERSTUDIO,
    TID_TUNERSTUDIO_OUTCHANNEL,
    TID_TUNERSTUDIO_SERVICE,
    TID_UART,
    TID_UART_PRINTF,
    TID_DASH_HW,
    TID_DASH_LOGIC,
    TID_DEBUG,
    TID_DWT,
    TID_DECODER_CONFIG,
    TID_DECODER_HW,
    TID_DECODER_LOGIC,
    TID_TUAREG_DECODER,
    TID_DIAGNOSTICS,
    TID_FUELING_CONFIG,
    TID_FUELING_CONTROLS,
    TID_TUAREG_FUELING,
    TID_SYSLOG,
    TID_FUEL_HW,
    TID_IGNITION_CONFIG,
    TID_IGNITION_HW,
    TID_TUAREG_IGNITION,
    TID_TUAREG_IGNITION_CONTROLS,
    TID_MAIN,
    TID_TUAREG,
    TID_TUAREG_CONFIG,
    TID_TUAREG_ERRORS,
    TID_TUAREG_POCESS_DATA,
    TID_TUAREG_TYPES,
    TID_PROCESS_TABLE,
    TID_TUAREG_SENSORS,
    TID_SENSOR_CALIBRATION,
    TID_SENSORS,
    TID_SERVICE,
    TID_EEPROM,
    TID_TABLE,
    TID_LOWPRIO_SCHEDULER,
    TID_SCHEDULER,
    TID_SYSTICK_TIMER,
    TID_BASE_CALC,
    TID_BITFIELDS,
    TID_CONVERSION,

    TID_COUNT

} Tuareg_ID;


#endif // TUAREG_ID_H_INCLUDED
