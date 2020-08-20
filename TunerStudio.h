#ifndef TUNERSTUDIO_H_INCLUDED
#define TUNERSTUDIO_H_INCLUDED

#include "Tuareg_process_data.h"
#include "ignition_logic.h"

/*
#define VEMAPPAGE_NR    1
#define VESETPAGE_NR    2 //Config Page 1
#define IGNMAPPAGE_NR   3
#define IGNSETPAGE_NR   4 //Config Page 2
#define AFRMAPPAGE_NR   5
#define AFRSETPAGE_NR   6 //Config Page 3
#define IACPAGE_NR      7 //Config Page 4
#define BOOSTVVCPAGE_NR 8
#define SEQFUELPAGE_NR  9
#define CANBUSPAGE_NR   10 //Config Page 10
#define WARMUPPAGE_NR   11
#define CALIBPAGE_NR    12 // echo -ne \\x50\\x3c > /dev/ttyACM0
*/

#define COOLANT_TABLE_NR 0
#define IAT_TABLE_NR 1
#define O2_TABLE_NR 2

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



/*
TS pages
*/
typedef enum {

    TS_ZERO_PAGE, // 0
    VEMAPPAGE, // 1
    VESETPAGE, // 2 -> Config Page 1
    IGNMAPPAGE, // 3
    IGNSETPAGE, // 4 -> Config Page 2
    AFRMAPPAGE, // 5
    AFRSETPAGE, // 6 -> Config Page 3
    IACPAGE,    // 7 -> Config Page 4
    BOOSTVVCPAGE, // 8
    SEQFUELPAGE, // 9
    CANBUSPAGE, // 10 -> Config Page 10
    WARMUPPAGE, // 11
    CALIBPAGE,  // 12 '<' 60

    DECODERPAGE, // 13 '=' 61
    IGNITIONPAGE, // 14 '>' 62
    IGNITIONMAP_TPS, // 15 '?' 63
    IGNITIONMAP_MAP, // 16 '' 64

    TSPAGE_COUNT

} TS_page_t;


typedef struct
{
    //further rx data is required
    U8 cmd_pending :1;

    U8 mod_permission :1;
    U8 burn_permission :1;

} TS_state_t ;

#define SENDVALUE_BUFFERSIZE 42
#define SENDVALUE_FAKE_PACKETSIZE 74

#define COMMAND_MAX_DURATION_S 3

typedef struct _tuners_cli_t
{
    TS_page_t currentPage;

    U8 currentCommand;

    TS_state_t State;

    U8 command_duration;
    U32 A_cmd_requests;

} tuners_cli_t ;

extern volatile tuners_cli_t TS_cli;

void ts_communication();
void ts_sendValues(U32 offset, U32 length);
void ts_sendPage();
void ts_diagPage();
void ts_debug_features(U32 feature);
void ts_replaceConfig(U32 valueOffset, U32 newValue);

void ts_diag_process_data(volatile process_data_t * pImage);
void ts_diag_ignition_timing(volatile ignition_timing_t * pTiming);
void ts_diagPage_ignition();
void ts_diagPage_decoder();
void ts_diagPage_calib();


#endif // TUNERSTUDIO_H_INCLUDED
