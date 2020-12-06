#ifndef PROCESS_TABLE_H
#define PROCESS_TABLE_H

#include "stm32_libs/boctok_types.h"
#include "uart.h"

#define PROCESS_TABLE_LENGTH (3* CRK_POSITION_COUNT)


/**
basic type to handle an absolute process advance angle in degree

variables of this type have to be designated with xxxx_PA
*/
typedef uint16_t process_advance_t;



typedef struct {

    crank_position_t crank_pos;
    engine_phase_t phase;
    process_advance_t base_PA;

} process_position_t;


void update_process_table(volatile crank_position_table_t * pCrankTable);

exec_result_t get_position_from_index(VU32 Index, volatile process_position_t * pTarget);
exec_result_t get_index_from_position(volatile process_position_t * pPosition, volatile U32 * pTargetIndex);

exec_result_t find_process_position_before(volatile process_advance_t Reference_PA, volatile process_position_t * pTarget);
exec_result_t find_process_position_after(VU32 Reference_PA, volatile process_position_t * pTarget);

exec_result_t get_process_advance(volatile process_position_t * pPosition);

void print_process_table(USART_TypeDef * Port);

#endif // PROCESS_TABLE_H
