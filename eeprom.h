#ifndef EEPROM_H_INCLUDED
#define EEPROM_H_INCLUDED

#include "stm32_libs/boctok_types.h"
#include "Tuareg_types.h"


// E0 = E1 = E2 = 0
#define sEE_HW_ADDRESS      0xA0
#define I2C_OWN_ADDRESS     0xA0

#define I2C_SPEED               300000




/* Maximum Timeout values for flags and events waiting loops. These timeouts are
   not based on accurate values, they just guarantee that the application will
   not remain stuck if the I2C communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define sEE_FLAG_TIMEOUT         ((uint32_t)0x1000)
#define sEE_LONG_TIMEOUT         ((uint32_t)(10 * sEE_FLAG_TIMEOUT))


/* Maximum number of trials for sEE_WaitEepromStandbyState() function */
#define sEE_MAX_TRIALS_NUMBER     150


/**
eeprom read/write operation result / i2c communication error codes for maximum debugging information
*/
typedef enum {

    EERES_OK =0,
    EERES_BUS_BUSY,
    EERES_MASTER_MODE,
    EERES_MASTER_WRITE,
    EERES_TRANSMIT_FAIL,
    EERES_ADDR_FAIL,
    EERES_RX_FAIL,
    EERES_STOP_FAIL,
    EERES_SLAVE_ACK,
    EERES_MAX_TRIALS,
    EERES_VERIFICATION,
    EERES_READY

} eeprom_result_t;

//map eeprom result to common exec result code
#define ASSERT_EEPROM_RESULT_OK(eeres_code) if((eeres_code) != EERES_OK) return EXEC_ERROR

void init_eeprom(void);
void eeprom_i2c_deinit(void);

eeprom_result_t eeprom_read_byte(U32 Address, U8 * Data_read);
eeprom_result_t eeprom_read_bytes(U32 Address, U32 * Data, U32 Length);

eeprom_result_t eeprom_write_byte(U32 Address, U32 data);
eeprom_result_t eeprom_write_bytes(U32 Address, U32 Data, U32 Length);

eeprom_result_t eeprom_update_byte(U32 Address, U32 Data);
eeprom_result_t eeprom_update_bytes(U32 Address, U32 Data, U32 Length);

eeprom_result_t eeprom_wait(void);


exec_result_t Eeprom_load_data(U32 BaseAddress, VU8 * const pTarget, U32 Length);
exec_result_t Eeprom_write_data(U32 BaseAddress, VU8 * const pSource, U32 Length);
exec_result_t Eeprom_update_data(U32 BaseAddress, VU8 * const pSource, U32 Length);


#endif // EEPROM_H_INCLUDED
