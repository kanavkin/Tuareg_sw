#ifndef SERIAL_BUFFER_H_INCLUDED
#define SERIAL_BUFFER_H_INCLUDED


#define COMRX_BUFFER_SIZE 64




typedef struct __attribute__ ((__packed__)) _ComRx_Buffer_t {

    U32 write;
    U32 read;

    U32 unread;

    U8 data[COMRX_BUFFER_SIZE];

} ComRx_Buffer_t ;


//serial buffer functions
void ComRx_Buffer_reset();
VU32 ComRx_Buffer_avail();
exec_result_t ComRx_Buffer_push(VU8 Input);
exec_result_t ComRx_Buffer_pull(VU32 * Output);
exec_result_t ComRx_Buffer_pull_U16(VU32 * Output);
exec_result_t ComRx_Buffer_pull_U32(VU32 * Output);




#endif // SERIAL_BUFFER_H_INCLUDED
