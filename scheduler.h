#ifndef SCHEDULER_H_INCLUDED
#define SCHEDULER_H_INCLUDED

#include "Tuareg.h"


#define SCHEDULER_PERIOD_US 1

//we are using a 32 bit timer now
#define SCHEDULER_MAX_PERIOD_US (U32) 0xFFFFFFFF * SCHEDULER_PERIOD_US -1



typedef enum {


    SCHEDIAG_DELAY_CLIPPED,

    SCHEDIAG_ICH1_SET,
    CHEDIAG_ICH1_CURRC_SET,
    CHEDIAG_ICH1_NEXTC_PRELOAD_SET,
    CHEDIAG_ICH1_NEXTC_UPDATE_SET,
    SCHEDIAG_ICH2_SET,
    SCHEDIAG_FCH1_SET,
    SCHEDIAG_FCH2_SET,

    SCHEDIAG_ICH1_TRIG,
    SCHEDIAG_ICH2_TRIG,
    SCHEDIAG_FCH1_TRIG,
    SCHEDIAG_FCH2_TRIG,

    SCHEDIAG_ICH1_RESET,
    SCHEDIAG_ICH2_RESET,
    SCHEDIAG_FCH1_RESET,
    SCHEDIAG_FCH2_RESET,

    SCHEDIAG_COUNT

} scheduler_diag_t;





typedef enum {

    IGN_CH1,
    IGN_CH2,

    FUEL_CH1,
    FUEL_CH2

} scheduler_channel_t;




typedef struct _scheduler_t {

    U32 ign_ch1_action;
    U32 ign_ch2_action;
    U32 fuel_ch1_action;
    U32 fuel_ch2_action;

    VU32 diag[SCHEDIAG_COUNT];

} scheduler_t;




void init_scheduler();
void scheduler_set_channel(scheduler_channel_t target_ch, U32 action, U32 delay_us);
void scheduler_reset_channel(scheduler_channel_t target_ch);

#endif // SCHEDULER_H_INCLUDED
