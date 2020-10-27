#ifndef CONVERSION_H_INCLUDED
#define CONVERSION_H_INCLUDED

#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "trigger_wheel_layout.h"

#define PAD 0xFFFFFFF
#define NO_PAD 0

typedef enum {

    TYPE_U8,
    TYPE_U16,
    TYPE_U32,

    TYPE_S8,
    TYPE_S16,
    TYPE_S32

} conversion_int_t ;



void UART_Print_S(USART_TypeDef * Port, S32 value, conversion_int_t inttype, U32 padding);
void UART_Print_U(USART_TypeDef * Port, U32 value, conversion_int_t inttype, U32 padding);

void UART_Print_U8Hex(USART_TypeDef * Port, U8 value);

void UART_Print_F32(USART_TypeDef * Port, F32 Value);

void UART_Print_crank_position(USART_TypeDef * Port, crank_position_t Position);

U32 compose_U32(U8 Msb, U8 Mid_h, U8 Mid_l, U8 Lsb);
float compose_float(U32 Buffer);
U32 serialize_float_U32(float Value);
void serialize_float_U8(float Value, U8 * pTarget);
void serialize_U16_U8(U16 Value, U8 * pTarget);


#endif // CONVERSION_H_INCLUDED
