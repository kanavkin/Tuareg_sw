#ifndef TUAREGTYPES_H_INCLUDED
#define TUAREGTYPES_H_INCLUDED

#include "stm32_libs/boctok_types.h"


/**
basic types to handle a return value safely on a 32 bit platform
*/
//typedef U32 exec_result_t;

typedef enum {

    EXEC_ERROR,
    EXEC_OK

//} exec_result_codes_t;
} exec_result_t;

#define ASSERT_EXEC_OK(result) if((result) != EXEC_OK) return (result)
#define ASSERT_EXEC_OK_VOID(result) if((result) != EXEC_OK) return


/**
basic types to handle an actors state
*/
typedef enum {

    ACTOR_UNPOWERED,
    ACTOR_POWERED

} actor_control_t;


/**
trigger positions

essential data type for the whole Tuareg sw

defines the possible crank positions in decending order,
the the first one is expected to be the closest position to TDC, hence provides the minimal advance angle
*/
typedef enum {

    CRK_POSITION_B2,
    CRK_POSITION_B1,
    CRK_POSITION_A2,
    CRK_POSITION_A1,
    CRK_POSITION_D2,
    CRK_POSITION_D1,
    CRK_POSITION_C2,
    CRK_POSITION_C1,

    CRK_POSITION_COUNT,
    CRK_POSITION_UNDEFINED

} crank_position_t;


typedef enum {

    PHASE_CYL1_COMP,
    PHASE_CYL1_EX,
    PHASE_UNDEFINED,
    PHASE_COUNT,

} engine_phase_t;


typedef enum {

    ALPHA_N,
    SPEED_DENS

} ctrl_strategy_t;


typedef enum {

    GEAR_NEUTRAL,
    GEAR_1,
    GEAR_2,
    GEAR_3,
    GEAR_4,
    GEAR_5,

    GEAR_COUNT

} gears_t;


/**
basic type to handle an absolute angular difference in degree

variables of this type have to be designated with xxxx_PD
*/
typedef uint32_t angle_deg_t;


/**
basic type to handle the system time stamp
increment is 1 ms
*/
typedef uint32_t timestamp_t;



extern const F32 cKelvin_offset;
extern const F32 cR_gas;
extern const F32 cM_air;
extern const F32 cPi;
extern const F32 cEuler;

extern const U32 cU32max;



#endif // TUAREGTYPES_H_INCLUDED
