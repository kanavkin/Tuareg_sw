#ifndef TUNERSTUDIO_H_INCLUDED
#define TUNERSTUDIO_H_INCLUDED

/*
TS pages
*/
typedef enum {

    TS_ZERO_PAGE, // 0

    CALIBPAGE,

    DECODERPAGE,

    IGNITIONPAGE,
    IGNITIONMAP_TPS,
    IGNITIONMAP_DWELL,

    FUELINGPAGE,
    VEMAP_TPS,
    VEMAP_MAP,
    AFRMAP_TPS,
    ACCELCOMP_TABLE,
    WARMUPCOMP_TABLE,
    INJ_TIMING_TABLE,
    INJ_PHASE_TABLE,
    CRANKINGFUEL_TABLE,

    TSETUP_PAGE,

    SYSLOG_PAGE,

    TSPAGE_COUNT

} TS_page_t;


void ts_readPage(U32 Page);
void ts_valueWrite(U32 Page, U32 Offset, U32 Value);
void ts_burnPage(U32 Page);


#endif // TUNERSTUDIO_H_INCLUDED
