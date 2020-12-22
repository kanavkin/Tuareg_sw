#ifndef TUNERSTUDIO_LEGACY_H_INCLUDED
#define TUNERSTUDIO_LEGACY_H_INCLUDED


/**
this must reflect the current configuration data layout!!!
*/
#define VEMAPPAGE_SIZE    288
#define VESETPAGE_SIZE    64
#define IGNMAPPAGE_SIZE   288
#define IGNSETPAGE_SIZE   64
#define AFRMAPPAGE_SIZE   288
#define AFRSETPAGE_SIZE   64
#define IACPAGE_SIZE      64
#define BOOSTVVCPAGE_SIZE 160
#define SEQFUELPAGE_SIZE  192
#define CANBUSPAGE_SIZE   128
#define WARMUPPAGE_SIZE   192
#define MAP_PAGE_SIZE 288


#define TS_IGNITION_ADVANCE_OFFSET 40

void ts_readPage_legacy(U32 Page);
void ts_showPage_legacy(U32 Page);
exec_result_t ts_valueWrite_legacy(U32 Page, U32 Offset, U32 Value);

#endif // TUNERSTUDIO_LEGACY_H_INCLUDED
