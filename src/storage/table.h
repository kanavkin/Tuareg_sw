
#ifndef TABLE_H
#define TABLE_H

#include "stm32_libs/boctok_types.h"
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"

#include "Tuareg_types.h"

/**
common dimensions for all tables
(X, Y axis)

data size (X axis) in bytes
data size (Y axis) in bytes
*/
#define TABLE_DIM 16
#define TABLE_CELL_SIZE_B 2

#define TABLE_STORAGE_SIZE_B 2*TABLE_DIM*TABLE_CELL_SIZE_B



/**
This is the data container for one 2D table

Y = f(X)

all data is volatile!
*/
typedef struct __attribute__ ((__packed__)) _table_data_t {

    VU16 axisX[TABLE_DIM];
    VU16 axisY[TABLE_DIM];

} table_data_t;


/**
This is a 2D config table
*/
typedef struct _table_t {

    //axisX and axisY (packed)
    volatile table_data_t data;

    //Xaxis cache -> xMin_index
    VU8 last_index;


} table_t;



F32 getValue_table(volatile table_t *fromTable, F32 X);
exec_result_t load_table(volatile table_t * pTable, U32 BaseAddress);
exec_result_t store_table(volatile table_t * pTable, U32 BaseAddress);
exec_result_t modify_table(volatile table_t * pTable, U32 Offset, U32 Value);
void send_table(USART_TypeDef * pPort, volatile table_t * pTable);
void show_table(USART_TypeDef * pPort, volatile table_t * pTable);

#endif // TABLE_H
