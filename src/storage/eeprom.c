#include "stm32_libs/stm32f4xx/cmsis/stm32f4xx.h"
#include "stm32_libs/stm32f4xx/boctok/stm32f4xx_gpio.h"
#include "stm32_libs/boctok_types.h"
#include "eeprom.h"

#include "Tuareg_ID.h"
#include "Tuareg_errors.h"


volatile bool Eeprom_Init_done= false;


void Eeprom_deinit()
{
    I2C_Cmd(I2C1, DISABLE);
    I2C_DeInit(I2C1);

    //clock
    RCC->APB1ENR &= ~RCC_APB1ENR_I2C1EN;

    //SCL and SDA
    GPIO_configure(GPIOB, 6, GPIO_MODE_IN, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_NONE);
    GPIO_configure(GPIOB, 7, GPIO_MODE_IN, GPIO_OUT_OD, GPIO_SPEED_LOW, GPIO_PULL_NONE);

    //Init can be executed again
    Eeprom_Init_done= false;
}



/**
using I2C_1 for main data storage eeprom
*/
void Eeprom_init()
{
    I2C_InitTypeDef  I2C_InitStructure;

    //reinit protection
    if(Eeprom_Init_done == true)
    {
        return;
    }

    Eeprom_Init_done= true;

    //clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    //SCL and SDA
    GPIO_configure(GPIOB, 6, GPIO_MODE_AF, GPIO_OUT_OD, GPIO_SPEED_HIGH, GPIO_PULL_NONE);
    GPIO_configure(GPIOB, 7, GPIO_MODE_AF, GPIO_OUT_OD, GPIO_SPEED_HIGH, GPIO_PULL_NONE);

    //enable port alternate function 4
    GPIO_SetAF(GPIOB, 6, 4);
    GPIO_SetAF(GPIOB, 7, 4);

    //I2C configuration
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = I2C_OWN_ADDRESS;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;

    I2C_Cmd(I2C1, ENABLE);
    I2C_Init(I2C1, &I2C_InitStructure);

}



eeprom_result_t eeprom_write_byte(U32 Address, U32 data)
{
    U32 sEETimeout;
    eeprom_result_t wait_result;

    // wait while the bus is busy
    sEETimeout = sEE_LONG_TIMEOUT;
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
    {
        if((sEETimeout--) == 0) return EERES_BUS_BUSY;
    }

    /**
    wait for the previous write cycle to complete
    */
    wait_result= eeprom_wait();

    if( wait_result != EERES_READY)
    {
        return wait_result;
    }


    I2C_GenerateSTART(I2C1, ENABLE);

    //wait until Master mode selected
    sEETimeout = sEE_FLAG_TIMEOUT;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
    {
        if((sEETimeout--) == 0) return EERES_MASTER_MODE;
    }

    //Send EEPROM address for write mode
    I2C_Send7bitAddress(I2C1, (U8) sEE_HW_ADDRESS, I2C_Direction_Transmitter);


    //wait until Master Write mode selected
    sEETimeout = sEE_FLAG_TIMEOUT;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
        if((sEETimeout--) == 0) return EERES_MASTER_WRITE;
    }


    //Send the EEPROM's internal address: MSB
    I2C_SendData(I2C1, (U8)((Address & 0xFF00) >> 8));

    //wait until byte was transmitted
    sEETimeout = sEE_FLAG_TIMEOUT;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if((sEETimeout--) == 0) return EERES_TRANSMIT_FAIL;
    }

    //Send the EEPROM's internal address: LSB
    I2C_SendData(I2C1, (U8)(Address & 0x00FF));

    //wait until byte was transmitted
    sEETimeout = sEE_FLAG_TIMEOUT;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if((sEETimeout--) == 0) return EERES_TRANSMIT_FAIL;
    }


    //Send data to write
    I2C_SendData(I2C1, data);

    //wait until byte was transmitted
    sEETimeout = sEE_FLAG_TIMEOUT;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if((sEETimeout--) == 0) return EERES_TRANSMIT_FAIL;
    }

    I2C_GenerateSTOP(I2C1, ENABLE);

    /**
    Perform a read on SR1 and SR2 register
    to clear pending flags
    */
    (void)I2C1->SR1;
    (void)I2C1->SR2;

    return EERES_OK;
}



eeprom_result_t eeprom_read_byte(U32 Address, U8 * Data_read)
{
    U32 sEETimeout;
    eeprom_result_t wait_result;

    // wait while the bus is busy
    sEETimeout = sEE_LONG_TIMEOUT;
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
    {
        if((sEETimeout--) == 0) return EERES_BUS_BUSY;
    }

    /**
    wait for the previous write cycle to complete
    */
    wait_result= eeprom_wait();

    if( wait_result != EERES_READY)
    {
        return wait_result;
    }

    I2C_GenerateSTART(I2C1, ENABLE);

    //wait until Master mode selected
    sEETimeout = sEE_FLAG_TIMEOUT;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
    {
        if((sEETimeout--) == 0) return EERES_MASTER_MODE;
    }

    /**
    Dummy write
    */

    //Send EEPROM address for write mode
    I2C_Send7bitAddress(I2C1, (U8) sEE_HW_ADDRESS, I2C_Direction_Transmitter);


    //wait until Master Write mode selected
    sEETimeout = sEE_FLAG_TIMEOUT;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {
        if((sEETimeout--) == 0) return EERES_MASTER_WRITE;
    }

    //Send the EEPROM's internal address: MSB
    I2C_SendData(I2C1, (U8)((Address & 0xFF00) >> 8));

    //wait until byte was transmitted
    sEETimeout = sEE_FLAG_TIMEOUT;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if((sEETimeout--) == 0) return EERES_TRANSMIT_FAIL;
    }

    //Send the EEPROM's internal address: LSB
    I2C_SendData(I2C1, (U8)(Address & 0x00FF));

    //wait until byte was transmitted
    sEETimeout = sEE_FLAG_TIMEOUT;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {
        if((sEETimeout--) == 0) return EERES_TRANSMIT_FAIL;
    }

    /**
    Read data
    */

    I2C_GenerateSTART(I2C1, ENABLE);

    //wait until Master mode selected
    sEETimeout = sEE_FLAG_TIMEOUT;
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
    {
        if((sEETimeout--) == 0) return EERES_MASTER_MODE;
    }

    //send EEPROM address for read
    I2C_Send7bitAddress(I2C1, (U8) (sEE_HW_ADDRESS | 0x01), I2C_Direction_Receiver);

    //wait until ADDR flag is set (ADDR is still not cleared)
    sEETimeout = sEE_FLAG_TIMEOUT;
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_ADDR) == RESET)
    {
      if((sEETimeout--) == 0) return EERES_ADDR_FAIL;
    }

    // Disable Acknowledgement
    I2C_AcknowledgeConfig(I2C1, DISABLE);


    __disable_irq();

    //Clear ADDR register by reading SR2 register
    (void)I2C1->SR2;

    I2C_GenerateSTOP(I2C1, ENABLE);

    __enable_irq();


    //wait for the byte to be received
    sEETimeout = sEE_FLAG_TIMEOUT;
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_RXNE) == RESET)
    {
      if((sEETimeout--) == 0) return EERES_RX_FAIL;
    }

    //read the byte received
    *Data_read= I2C_ReceiveData(I2C1);

    //wait to make sure that STOP control bit has been cleared
    sEETimeout = sEE_FLAG_TIMEOUT;
    while(I2C1->CR1 & I2C_CR1_STOP)
    {
      if((sEETimeout--) == 0) return EERES_STOP_FAIL;
    }

    //Re-Enable Acknowledgement to be ready for another reception
    I2C_AcknowledgeConfig(I2C1, ENABLE);

    return EERES_OK;
}



/**
  * Wait for EEPROM Standby state.
  *
  * returns EERES_READY when ready
  */
eeprom_result_t eeprom_wait(void)
{
    U32 sEETimeout;
    VU16 tmpSR1 = 0;
    VU32 sEETrials = 0;

    //while the bus is busy
    sEETimeout = sEE_LONG_TIMEOUT;
    while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY))
    {
        if((sEETimeout--) == 0) return EERES_BUS_BUSY;
    }

    /**
    Keep looping till the slave acknowledge his address or maximum number of trials is reached
    */
    while (sEETrials < sEE_MAX_TRIALS_NUMBER)
    {
        I2C_GenerateSTART(I2C1, ENABLE);

        //wait until Master mode selected
        sEETimeout = sEE_FLAG_TIMEOUT;
        while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
        {
            if((sEETimeout--) == 0) return EERES_MASTER_MODE;
        }

        //send EEPROM address for write */
        I2C_Send7bitAddress(I2C1, (U8) sEE_HW_ADDRESS, I2C_Direction_Transmitter);

        sEETimeout = sEE_LONG_TIMEOUT;

        do
        {
            tmpSR1 = I2C1->SR1;

            if((sEETimeout--) == 0) return EERES_SLAVE_ACK;
        }
        while((tmpSR1 & (I2C_SR1_ADDR | I2C_SR1_AF)) == 0);

        //check if the ADDR flag has been set
        if (tmpSR1 & I2C_SR1_ADDR)
        {
            (void)I2C1->SR2;
            I2C_GenerateSTOP(I2C1, ENABLE);

            return EERES_READY;
        }
        else
        {
            I2C_ClearFlag(I2C1, I2C_FLAG_AF);
        }


        sEETrials++;
    }

    return EERES_MAX_TRIALS;
}



/***************************************************************************************************************************
* eeprom interface
***************************************************************************************************************************/

eeprom_result_t eeprom_read_bytes(U32 Address, U32 * pTarget, U32 Length)
{
    U32 eeprom_data =0, i;
    eeprom_result_t ee_result;
    U8 data;

    Assert(Length > 0, TID_EEPROM, 0);
    Assert(Length < 5, TID_EEPROM, 1);

    /**
    Eeprom data order: Little Endian ->  <MSB> <LSB>
    */

    for(i=0; i< Length; i++)
    {
        ee_result= eeprom_read_byte(Address + i, &data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        //data read successfully
        eeprom_data |= (data << 8*(Length - (i+1)) );

    }

    *pTarget= eeprom_data;

    return EERES_OK;
}


eeprom_result_t eeprom_write_bytes(U32 Address, U32 Data, U32 Length)
{
    U32 i;
    eeprom_result_t ee_result;

    Assert(Length > 0, TID_EEPROM, 2);
    Assert(Length < 5, TID_EEPROM, 3);

    for(i=0; i< Length; i++)
    {
        //data order: Little Endian MSB LSB
        ee_result= eeprom_write_byte(Address + i, (U8) (Data >> 8*(Length - (i+1))) );

        ASSERT_EEPROM_RESULT_OK(ee_result);
    }

    return EERES_OK;
}



/**
* Write ONE BYTE to the EEPROM only where needed to save write cycles
*/
eeprom_result_t eeprom_update_byte(U32 Address, U32 Data)
{
    eeprom_result_t ee_result;
    U8 eeprom_data;

    //read current data
    ee_result= eeprom_read_byte(Address, &eeprom_data);

    ASSERT_EEPROM_RESULT_OK(ee_result);

    //check if eeprom data has to be modified
    if(eeprom_data == Data)
    {
        return EERES_OK;
    }

    //continue with writing
    ee_result= eeprom_write_byte(Address, Data);

    ASSERT_EEPROM_RESULT_OK(ee_result);

    //read back written data
    ee_result= eeprom_read_byte(Address, &eeprom_data);

    ASSERT_EEPROM_RESULT_OK(ee_result);

    //verify written data
    if(eeprom_data == Data)
    {
        //success
        return EERES_OK;
    }

    return EERES_VERIFICATION;
}




/**
update 1 to 4 eeprom bytes, beginning from Address, with the data contained in Data
*/
eeprom_result_t eeprom_update_bytes(U32 Address, U32 Data, U32 Length)
{
    U32 i;
    eeprom_result_t ee_result;

    Assert(Length > 0, TID_EEPROM, 4);
    Assert(Length < 5, TID_EEPROM, 5);

    for(i=0; i< Length; i++)
    {
        //data order: Little Endian MSB LSB
        ee_result= eeprom_update_byte(Address + i, (U8) (Data >> 8*(Length - (i+1))) );

        ASSERT_EEPROM_RESULT_OK(ee_result);
    }

    return ee_result;
}


/***************************************************************************************************************************
* new byte stream oriented Eeprom interface
*
* these functions read/write eeprom data "as is", so padding bytes will be written as well
* the source / target data area shall be packed __attribute__ ((__packed__))
***************************************************************************************************************************/

exec_result_t Eeprom_load_data(U32 BaseAddress, VU8 * const pTarget, U32 Length)
{
    U32 i;
    eeprom_result_t ee_result;
    U8 data;

    for(i=0; i< Length; i++)
    {
        ee_result= eeprom_read_byte(BaseAddress + i, &data);

        ASSERT_EEPROM_RESULT_OK(ee_result);

        //data read successfully
        *(pTarget + i)= data;
    }

    return EXEC_OK;
}


exec_result_t Eeprom_write_data(U32 BaseAddress, VU8 * const pSource, U32 Length)
{
    U32 i;
    eeprom_result_t ee_result;

    for(i=0; i< Length; i++)
    {
        ee_result= eeprom_write_byte(BaseAddress + i, *(pSource +i) );

        ASSERT_EEPROM_RESULT_OK(ee_result);
    }

    return EXEC_OK;
}


exec_result_t Eeprom_update_data(U32 BaseAddress, VU8 * const pSource, U32 Length)
{
    U32 i;
    eeprom_result_t ee_result;

    for(i=0; i< Length; i++)
    {
        ee_result= eeprom_update_byte(BaseAddress + i, *(pSource +i) );

        ASSERT_EEPROM_RESULT_OK(ee_result);
    }

    return EXEC_OK;
}








