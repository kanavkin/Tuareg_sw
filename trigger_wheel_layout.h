#ifndef TRIGGERWHEELLAYOUT_H_INCLUDED
#define TRIGGERWHEELLAYOUT_H_INCLUDED


/**
trigger wheel geometry
defines the crank angle corresponding to the trigger wheel key position,
counting from the position closest to TDC against the normal rotation direction
(reflecting ignition advance)
*/
#define POSITION_A1_ANGLE 0         //-> 260    --> 100
#define POSITION_A2_ANGLE 40        //-> 300    --> 60
#define POSITION_B1_ANGLE 90        //-> 350    --> 10
#define POSITION_B2_ANGLE 100       //-> 0      --> 360
#define POSITION_C1_ANGLE 180       //-> 80     --> 280
#define POSITION_C2_ANGLE 185       //-> 85     --> 275
#define POSITION_D1_ANGLE 270       //-> 170    --> 190
#define POSITION_D2_ANGLE 275       //-> 175    --> 185


typedef enum {

    POSITION_A1,
    POSITION_A2,
    POSITION_B1,
    POSITION_B2,
    POSITION_C1,
    POSITION_C2,
    POSITION_D1,
    POSITION_D2,

    POSITION_COUNT,

    UNDEFINED_POSITION

} engine_position_t;



#endif // TRIGGERWHEELLAYOUT_H_INCLUDED
