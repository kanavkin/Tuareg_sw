#ifndef TUNERSTUDIO_H_INCLUDED
#define TUNERSTUDIO_H_INCLUDED

#include "TunerStudio_syslog_locations.h"

/*
TS pages
*/
typedef enum {

    TS_ZERO_PAGE, // 0

    CALIBPAGE,
    INVCLT_TABLE,

    DECODERPAGE,

    TSETUP_PAGE,
    CTRLSET_MAP_PAGE,
    CTRLSET_TPS_PAGE,
    CTRLSET_TPSLIMP_PAGE,

    IGNITIONPAGE,
    IGNITIONMAP_DWELL,

    FUELINGPAGE,
    ACCELCOMP_TPS,
    ACCELCOMP_MAP,
    WARMUPCOMP_TABLE,
    INJ_TIMING_TABLE,
    CRANKINGFUEL_TABLE,
    BAROCORR_TABLE,
    CHARGETEMP_TABLE,

    TACH_TABLE,

    SYSLOG_PAGE,
    FAULTLOG_PAGE,

    TSPAGE_COUNT

} TS_page_t;


void ts_readPage(U32 Page);
exec_result_t ts_valueWrite(U32 Page, U32 Offset, U32 Value);
exec_result_t ts_burnPage(U32 Page);


#endif // TUNERSTUDIO_H_INCLUDED
