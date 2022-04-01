#include "Tuareg.h"
#include "serial_buffer.h"



volatile ComRx_Buffer_t ComRx_Buffer= {

    .write= 0,
    .read= 0,
    .unread= 0
};




/**


*/
void ComRx_Buffer_reset()
{
    ComRx_Buffer.write= 0;
    ComRx_Buffer.read= 0;
    ComRx_Buffer.unread= 0;
}


/**


*/
VU32 ComRx_Buffer_avail()
{
    return ComRx_Buffer.unread;
}


/**


*/
exec_result_t ComRx_Buffer_push(VU8 Input)
{
    //check write pointer for post fence
    if(ComRx_Buffer.write >= COMRX_BUFFER_SIZE)
    {
        //recover error
        ComRx_Buffer_reset();

        return EXEC_ERROR;
    }

    //check if buffer has some space left
    if(ComRx_Buffer.unread >= COMRX_BUFFER_SIZE)
    {
        //buffer full
        return EXEC_ERROR;
    }

    //store data
    ComRx_Buffer.data[ComRx_Buffer.write]= Input;

    //count new data
    ComRx_Buffer.unread++;

    //calculate next free cell
    ComRx_Buffer.write= (ComRx_Buffer.write >= COMRX_BUFFER_SIZE -1)? 0 : ComRx_Buffer.write +1;

    return EXEC_OK;
}



/**


*/
exec_result_t ComRx_Buffer_pull(VU32 * Output)
{
    //check if buffer holds data
    if(ComRx_Buffer.unread == 0)
    {
        //buffer empty
        return EXEC_ERROR;
    }

    //check read pointer for post fence
    if(ComRx_Buffer.read >= COMRX_BUFFER_SIZE)
    {
        //recover error
        ComRx_Buffer_reset();

        return EXEC_ERROR;
    }

    //read data
    *Output= ComRx_Buffer.data[ComRx_Buffer.read];

    //count new data
    ComRx_Buffer.unread--;

    //calculate next cell
    ComRx_Buffer.read= (ComRx_Buffer.read >= COMRX_BUFFER_SIZE -1)? 0 : ComRx_Buffer.read +1;

    return EXEC_OK;
}


/**


*/
exec_result_t ComRx_Buffer_pull_U16(VU32 * Output)
{
    VU32 byte1, byte2;
    exec_result_t result;

    result= ComRx_Buffer_pull(&byte1);

    ASSERT_EXEC_OK(result);

    result= ComRx_Buffer_pull(&byte2);

    ASSERT_EXEC_OK(result);

    //set output
    *Output= (byte1 << 8) | byte2;

    return EXEC_OK;
}

exec_result_t ComRx_Buffer_pull_U32(VU32 * Output)
{
    VU32 byte1, byte2, byte3, byte4;
    exec_result_t result;

    result= ComRx_Buffer_pull(&byte1);

    ASSERT_EXEC_OK(result);

    result= ComRx_Buffer_pull(&byte2);

    ASSERT_EXEC_OK(result);

    result= ComRx_Buffer_pull(&byte3);

    ASSERT_EXEC_OK(result);

    result= ComRx_Buffer_pull(&byte4);

    ASSERT_EXEC_OK(result);

    //set output
    *Output= (byte1 << 24) | (byte2 << 16) | (byte3 << 8) | byte4;

    return EXEC_OK;
}
