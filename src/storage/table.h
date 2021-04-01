
#ifndef TABLE_H
#define TABLE_H

#include "stm32_libs/boctok_types.h"
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"

#include "Tuareg_types.h"


/**
common dimensions for tables
(X, Y, Z axis)
*/
#define T2D_DATA_DIMENSION 16
#define T3D_DATA_DIMENSION 16

/**
table manager
*/
typedef struct _table_cache_t {

    /**
    Store the X and Y interval from the last request to make the next access faster
    */
    U32 last_Xmax_index;
    U32 last_Xmin_index;
    U32 last_Ymax_index;
    U32 last_Ymin_index;

} table_cache_t;


/**
This is the data container for one 2D table

It stores 16-bit unsigned values in X and Y.

This is the data Tuner Studio operates on.

Y = f(X)

for simplicity all 2D table axes have the same dimension (T2D_DATA_DIMENSION)
*/
typedef struct __attribute__ ((__packed__)) _t2D_data_t {

    U16 axisX[T2D_DATA_DIMENSION];
    U8 axisY[T2D_DATA_DIMENSION];

} t2D_data_t;


/**
This is the data container for one 3D table

It stores 8-bit unsigned values in Z, 16-bit unsigned values in X and Y.

This is the data Tuner Studio operates on.

Z = f(X, Y)

for simplicity all 3D tables axes have the same dimension (T3D_DATA_DIMENSION)
*/
typedef struct __attribute__ ((__packed__)) _t3D_data_t {

    U16 axisX[T3D_DATA_DIMENSION];
    U16 axisY[T3D_DATA_DIMENSION];
    U8 axisZ[T3D_DATA_DIMENSION] [T3D_DATA_DIMENSION];

} t3D_data_t;



/**
This is a 2D config table
*/
typedef struct _t2D_t {

    //table_cache_t cache;
    t2D_data_t data;

} t2D_t;


/**
This is a 3D config table
*/
typedef struct _t3D_t {

    table_cache_t cache;
    t3D_data_t data;

} t3D_t;





VF32 getValue_t2D(volatile t2D_t *fromTable, VU32 X);
VF32 getValue_t3D(volatile t3D_t * fromTable, VU32 X, VU32 Y);


exec_result_t load_t2D_data(volatile t2D_data_t * pTableData, U32 BaseAddress);
exec_result_t load_t3D_data(volatile t3D_data_t * pTableData, U32 BaseAddress);

exec_result_t store_t2D_data(volatile t2D_data_t * pTableData, U32 BaseAddress);
exec_result_t store_t3D_data(volatile t3D_data_t * pTableData, U32 BaseAddress);


exec_result_t modify_t2D_data(volatile t2D_data_t * pTableData, U32 Offset, U32 Value);
exec_result_t modify_t3D_data(volatile t3D_data_t * pTableData, U32 Offset, U32 Value);

void send_t2D_data(USART_TypeDef * pPort, volatile t2D_data_t * pTable);
void send_t3D_data(USART_TypeDef * pPort, volatile t3D_data_t * pTable);


void show_t3D_data(USART_TypeDef * pPort, volatile t3D_data_t * pTableData);
void show_t2D_data(USART_TypeDef * pPort, volatile t2D_data_t * pTableData);



#endif // TABLE_H
