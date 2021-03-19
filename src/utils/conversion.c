

#include "stm32_libs/boctok_types.h"
#include "conversion.h"
#include "uart.h"
#include "bitfields.h"






/**
we use ascii format for positions in config items in Tunerstudio interface

*/
/*
crank_position_t parse_position(U32 Input)
{
    crank_position_t parsed_pos = CRK_POSITION_UNDEFINED;

    switch(Input)
        {
        case 'P.A1':
            parsed_pos= CRK_POSITION_A1;
            break;

        case 'P.A2':
            parsed_pos= CRK_POSITION_A2;
            break;

        case 'P.B1':
            parsed_pos= CRK_POSITION_B1;
            break;

        case 'P.B2':
            parsed_pos= CRK_POSITION_B2;
            break;

        case 'P.C1':
            parsed_pos= CRK_POSITION_C1;
            break;

        case 'P.C2':
            parsed_pos= CRK_POSITION_C2;
            break;

        case 'P.D1':
            parsed_pos= CRK_POSITION_D1;
            break;

        case 'P.D2':
            parsed_pos= CRK_POSITION_D2;
            break;

        default:
            //parsed position initialized with CRK_POSITION_UNDEFINED
            break;

        }

    return parsed_pos;

}
*/


U32 serialize_float_U32(float Value)
{
    union {

        float in;
        U32  out;

    } u;

    u.in = Value;

    return u.out;
}


/*
serializes input float to 4 bytes output
system (FPU) byte order is little endian
*/
void serialize_float_U8(float Value, U8 * pTarget)
{
    U32 i;

    union {

        float in;
        U8  out[4];

    } u;

    u.in = Value;

    for(i=0; i<4; i++)
    {
        pTarget[i]= u.out[i];
    }
}

//writes 4 bytes to pTarget, LSB first
void send_float(USART_TypeDef * Port, float Value)
{
    U32 i;

    union {

        float in;
        U8  out[4];

    } u;

    u.in = Value;

    for(i=0; i<4; i++)
    {
        UART_Tx(Port, u.out[i]);
    }
}


//writes 2 bytes to pTarget, LSB first
void serialize_U16_U8(VU16 Value, VU8 * pTarget)
{
      *pTarget= (VU8)(Value & 0x00FF);
      *(pTarget +1)= (VU8)(Value >> 8);
}


//writes 2 bytes to pTarget, MSB first
void serialize_U16_U8_reversed(VU16 Value, VU8 * pTarget)
{
      *pTarget= (VU8)(Value >> 8);
      *(pTarget +1)= (VU8)(Value & 0x00FF);
}


/*
composes 4 bytes input bytes to float
FPU byte order is little endian, so input data (Buffer) byte order shall be little-endian too!
*/
float compose_float(U32 Buffer)
{
    union {

        float float_out;
        U32  in;

    } u;

    u.in = Buffer;

    return u.float_out;
}




U32 compose_U32(U8 Msb, U8 Mid_h, U8 Mid_l, U8 Lsb)
/// TODO (oli#9#): broken, data order?
{
    union {

        U32 out;
        U8  in[4];

    } u;

    u.in[0] = Msb;
    u.in[1] = Mid_h;
    u.in[2] = Mid_l;
    u.in[3] = Lsb;

    return u.out;
}





/*
writes 4 bytes to pTarget from first to last memory address (LSB to MSB)
*/
void serialize_U32_U8(VU32 Value, VU8 * pTarget)
{
    U32 i;

    union {

        U32 in;
        U8  out[4];

    } u;

    u.in = Value;

    for(i=0; i<4; i++)
    {
        *pTarget=  u.out[i];
        pTarget++;
    }
}

/*
writes 4 bytes to pTarget with reversed byte order
-> begin wit MSB
*/
void serialize_U32_U8_reversed(VU32 Value, VU8 * pTarget)
{
    U32 i;

    union {

        U32 in;
        U8  out[4];

    } u;

    u.in = Value;

    for(i=0; i<4; i++)
    {
        *pTarget=  u.out[3 - i];
        pTarget++;
    }
}


