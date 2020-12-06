/*
This file is used for everything related to maps/tables including their definition, functions etc
*/
#ifndef TABLE_H
#define TABLE_H

#include "stm32_libs/boctok_types.h"
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"

#include "Tuareg_types.h"

#define TABLE_RPM_MULTIPLIER  100
#define TABLE_LOAD_MULTIPLIER 2


#define FUEL_TABLE_DIMENSION 16
#define IGNITION_TABLE_DIMENSION 16
#define AFR_TABLE_DIMENSION 16
#define BOOST_TABLE_DIMENSION 8
#define VVT_TABLE_DIMENSION 8
#define TRIM1_TABLE_DIMENSION 6
#define TRIM2_TABLE_DIMENSION 6
#define TRIM3_TABLE_DIMENSION 6
#define TRIM4_TABLE_DIMENSION 6


/**
Calibration tables
use linear interpolation for sensor data conversion (we can afford the the table overhead with low conversion rates, and save a lot of RAM)
All temperature measurements are stored offset by 40 degrees. This is so we can use an unsigned byte (0-255) to represent temperature ranges from -40 to 215
The fuel trim tables are offset by 128 to allow for -128 to +128 values
Ignition values from the main spark table are offset 40 degrees downards to allow for negative spark timing
*/


//#define CALIBRATION_TABLE_SIZE 512

#define OFFSET_FUELTRIM 127
#define OFFSET_IGNITION 40


#define TABLE_3D_ARRAYSIZE 16


/**
common size for 3D tables
*/
#define TABLE3D_DIMENSION 16

/**
helper constants for 3D table data handling

data layout: < Z (0).. (DIMENSION^2 -1)> < X (DIMENSION^2) .. (DIMENSION^2 -1 + DIMENSION)> < Y  (DIMENSION^2 + DIMENSION) .. (DIMENSION^2 -1 + 2*DIMENSION)>
*/
#define TABLE3D_Z_END 16




/**
This 2D table handles 16-bit unsigned values

It actually does not store any configuration data (config data lives in the configpages),
but provide quick access to them
*/
typedef struct _table2D_t {

  //number of tupels
  U32 dimension;

  VU16 * axisX;
  VU16 * axisY;

  /**
  Store the X environment from the last
  request to make the next access faster
  */
  U32 last_Xmax_index;

  //not needed anymore, to be removed
  U32 last_Xmin_index;

} table2D ;



/**
This 3D table "contains" 8-bit unsigned values in Z,
16-bit signed values in X and Y

Z = f(X, Y)

for simplicity all 3D tables have the same dimension (TABLE3D_DIMENSION)
*/
typedef struct _table3D_t {

    /**
    Store the X and Y interval from the last
    request to make the next access faster
    */
    VU8 last_Xmax_index;
    VU8 last_Xmin_index;
    VU8 last_Ymax_index;
    VU8 last_Ymin_index;

    VU8 axisZ[TABLE3D_DIMENSION] [TABLE3D_DIMENSION];
    VU16 axisX[TABLE3D_DIMENSION];
    VU16 axisY[TABLE3D_DIMENSION];

} table3D_t;



VF32 table2D_getValue(volatile table2D *fromTable, VU32 X);


exec_result_t load_3D_table(volatile table3D_t * pTarget, VU32 BaseAddress, VU32 Scaling_X, VU32 Scaling_Y);
exec_result_t write_3D_table(volatile table3D_t * pTable, VU32 BaseAddress, VU32 Scaling_X, VU32 Scaling_Y);
exec_result_t modify_3D_table(volatile table3D_t * pTable, U32 Offset, U32 Value);
VF32 lookup_3D_table(volatile table3D_t * fromTable, VU32 X, VU32 Y);
void print_3D_table(USART_TypeDef * pPort, volatile table3D_t * pTable);

exec_result_t load_tables();
exec_result_t write_tables();








#endif // TABLE_H
