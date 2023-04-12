///////////////////////////////////////////////////////////////////////////////
//
// \brief
//  sw_slave_i2c.h
//
// \version
// v0.0.1: 2022.05.01, Initial version.
///////////////////////////////////////////////////////////////////////////////

#ifndef __SW_SLAVE_I2C0_H_
#define __SW_SLAVE_I2C0_H_

#ifdef __cplusplus
extern "C" {
#endif

#define I2C0_SLAVE_ADDR_7BIT            0x51
/**
  * @brief  Configures slave I2C devices
  * @param  None
  * @retval None
  */
void i2c0_sw_slave_init(void);

/**
  * @brief  Configures slave I2C devices
  * @param  None
  * @retval None
  */
void i2c0_reg_update_poll(void);

enum I2C0_REG_LIST {
    /* 0x00 */
    I2C0_REG0_VERSION = 0x00,
    /* 0x10 for temp read*/
    I2C0_REG10_TEMP_H  = 0x10,
    I2C0_REG11_TEMP_L,
    I2C0_REG_END
};

#ifdef __cplusplus
}
#endif


#endif /*__SW_SLAVE_I2C0_H_*/
