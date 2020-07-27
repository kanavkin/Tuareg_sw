#ifndef TRIGGERWHEELLAYOUT_H_INCLUDED
#define TRIGGERWHEELLAYOUT_H_INCLUDED

/**
essential config section
*/


/**
measuring the effective segment duration revealed:
82 40 49 10 79 7 82 7  rpm: 4265

-> 0 40 89 99 178 185 267 274
*/


/**
trigger wheel geometry

contains the trigger wheel layout (angles corresp. to positions A1 .. D2)
defines the crank angle corresponding to the trigger wheel key position,
counting from the position closest to TDC against the normal rotation direction
(reflecting ignition advance)

config item:
configPage12.trigger_position_map[POSITION_COUNT]

default:
DEFAULT_CONFIG12_POSITION_A1_ANGLE
DEFAULT_CONFIG12_POSITION_A2_ANGLE
DEFAULT_CONFIG12_POSITION_B1_ANGLE
DEFAULT_CONFIG12_POSITION_B2_ANGLE
DEFAULT_CONFIG12_POSITION_C1_ANGLE
DEFAULT_CONFIG12_POSITION_C2_ANGLE
DEFAULT_CONFIG12_POSITION_D1_ANGLE
DEFAULT_CONFIG12_POSITION_D2_ANGLE
*/
#define DEFAULT_CONFIG12_POSITION_A1_ANGLE 0
#define DEFAULT_CONFIG12_POSITION_A2_ANGLE 40
#define DEFAULT_CONFIG12_POSITION_B1_ANGLE 89
#define DEFAULT_CONFIG12_POSITION_B2_ANGLE 99
#define DEFAULT_CONFIG12_POSITION_C1_ANGLE 178
#define DEFAULT_CONFIG12_POSITION_C2_ANGLE 185
#define DEFAULT_CONFIG12_POSITION_D1_ANGLE 267
#define DEFAULT_CONFIG12_POSITION_D2_ANGLE 274



/**
trigger positions

essential data type for the whole Tuareg sw

defines the possible crank positions
*/
typedef enum {

    CRK_POSITION_A1,
    CRK_POSITION_A2,
    CRK_POSITION_B1,
    CRK_POSITION_B2,
    CRK_POSITION_C1,
    CRK_POSITION_C2,
    CRK_POSITION_D1,
    CRK_POSITION_D2,

    CRK_POSITION_COUNT,

    CRK_POSITION_UNDEFINED

} crank_position_t;



#endif // TRIGGERWHEELLAYOUT_H_INCLUDED
