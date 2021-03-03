#ifndef TUNERSTUDIO_OUTCHANNEL_H_INCLUDED
#define TUNERSTUDIO_OUTCHANNEL_H_INCLUDED

#include "uart.h"

/**
OutputChannel comm bits
*/
typedef enum {

    COMMBIT_CALMOD_PERMISSION,
    COMMBIT_IGNMOD_PERMISSION,
    COMMBIT_DECMOD_PERMISSION,
    COMMBIT_BURN_PERMISSION,
    COMMBIT_SYSLOG_UPDATE,
    COMMBIT_DATALOG_UPDATE,
    COMMBIT_HSPDLOG_FULL,
    COMMBIT_COUNT

} comm_bits_t;


/**
OutputChannel Tuareg bits
32 bits
*/
typedef enum {

    //error bits
    TBIT_DECODERCONFIG_ERROR,
    TBIT_IGNITIONCONFIG_ERROR,
    TBIT_SENSORCALIB_ERROR,
    TBIT_TUAREGCONFIG_ERROR,

    TBIT_O2SENSOR_ERROR,
    TBIT_TPSENSOR_ERROR,
    TBIT_IATSENSOR_ERROR,
    TBIT_CLTSENSOR_ERROR,
    TBIT_VBATSENSOR_ERROR,
    TBIT_KNOCKSENSOR_ERROR,
    TBIT_BAROSENSOR_ERROR,
    TBIT_GEARSENSOR_ERROR,
    TBIT_MAPSENSOR_ERROR,
    TBIT_CISENSOR_ERROR,
    TBIT_SPARE1_ERROR,

    // status bits
    TBIT_CRANKING_MODE,
    TBIT_LIMP_MODE,
    TBIT_DIAG_MODE,

    //halt sources
    TBIT_HSRC_CRASH,
    TBIT_HSRC_RUN,
    TBIT_HSRC_SIDESTAND,

    //actors
    TBIT_ACT_IGN_INH,
    TBIT_ACT_FUEL_INH,
    TBIT_ACT_FUEL_PUMP,

    TBIT_COUNT

} tuareg_bits_t;


/**
OutputChannel ignition bits
*/
typedef enum {

    IGNBIT_VALID,
    IGNBIT_DEFAULT_CTRL,
    IGNBIT_CRANKING_CTRL,
    IGNBIT_DYNAMIC,
    IGNBIT_REV_LIMITER,
    IGNBIT_SEQ_MODE,
    IGNBIT_COLD_IDLE,
    IGNBIT_ADVANCE_MAP,
    IGNBIT_ADVANCE_TPS,
    IGNBIT_COUNT

} ignition_bits_t;



#define TS_OCHBLOCKSIZE 52


void ts_sendOutputChannels(USART_TypeDef * Port);

VU8 ts_comm_bits();
VU32 ts_tuareg_bits();
VU16 ts_ignition_bits();


#endif // TUNERSTUDIO_OUTCHANNEL_H_INCLUDED
