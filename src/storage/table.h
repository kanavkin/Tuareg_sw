
#ifndef TABLE_H
#define TABLE_H

#include "stm32_libs/boctok_types.h"
#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"

#include "Tuareg_types.h"



#define T2D_DATA_DIMENSION 16


/**
This is the data container for one 2D table

Y = f(X)

all 2D table axes have the same dimension (T2D_DATA_DIMENSION)
all data is volatile!
*/
typedef struct __attribute__ ((__packed__)) _t2D_data_t {

    VU16 axisX[T2D_DATA_DIMENSION];
    VU16 axisY[T2D_DATA_DIMENSION];

} t2D_data_t;


/**
This is a 2D config table
*/
typedef struct _t2D_t {

    //axisX and axisY (packed)
    volatile t2D_data_t data;

    //Xaxis cache -> xMin_index
    VU8 last_index;


} t2D_t;



F32 getValue_t2D(volatile t2D_t *fromTable, F32 X);


exec_result_t load_t2D_data(volatile t2D_data_t * pTableData, U32 BaseAddress);
exec_result_t load_t2D(volatile t2D_t * pTable, U32 BaseAddress);

exec_result_t store_t2D_data(volatile t2D_data_t * pTableData, U32 BaseAddress);
exec_result_t store_t2D(volatile t2D_t * pTable, U32 BaseAddress);

exec_result_t modify_t2D_data(volatile t2D_data_t * pTableData, U32 Offset, U32 Value);

void send_t2D_data(USART_TypeDef * pPort, volatile t2D_data_t * pTable);

#endif // TABLE_H
