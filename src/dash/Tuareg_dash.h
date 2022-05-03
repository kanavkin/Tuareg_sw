#ifndef DASHLOGIC_H_INCLUDED
#define DASHLOGIC_H_INCLUDED

#include "Tuareg_platform.h"


typedef enum {

    MIL_OFF,
    MIL_PERMANENT,

    MIL_BLINK_SLOW,
    MIL_BLINK_FAST,
    /*
    MIL_CODE_TWO,
    MIL_CODE_THREE,
    MIL_CODE_FOUR,
    MIL_CODE_FIVE,
    MIL_CODE_SIX,
    MIL_CODE_SEVEN,
    MIL_CODE_EIGHT,
    */
    MIL_COUNT

} mil_state_t;



#define MIL_BLINK_SLOW_ON_ITV 10
#define MIL_BLINK_SLOW_OFF_ITV 15

#define MIL_BLINK_FAST_ON_ITV 5
#define MIL_BLINK_FAST_OFF_ITV 5

#define TACH_MAX_READING_RPM 10000




typedef struct {

    U32 tacho_reading_rpm;
    U32 tacho_on_interval_ms;
    U32 tacho_off_interval_ms;
    U32 tacho_counter;

    mil_state_t mil;
    U32 mil_cycle;

} dashctrl_t;



void init_dash();

void update_mil();
void update_tachometer();

void set_tachometer(U32 Reading_rpm);
void set_mil(mil_state_t State);

#endif // DASHLOGIC_H_INCLUDED
