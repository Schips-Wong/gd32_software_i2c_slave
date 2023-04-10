
///////////////////////////////////////////////////////////////////////////////
//
// \brief
//  sw_slave_i2c.h
//
// \version
// v0.0.1: 2022.05.01, Initial version.
///////////////////////////////////////////////////////////////////////////////

#ifndef __SW_SLAVE_I2C_H_
#define __SW_SLAVE_I2C_H_

#ifdef __cplusplus
extern "C" {
#endif


#define SW_SLAVE_ADDR_7BIT          0x51


#if 0

本例程中，对于400kbps速率的I2C通信，在进行代码编译链接时，需要使用-Ofast的优化方式，以提高中断处理函数的执行速度，使程序能正确执行。

如果使用默认的无优化配置，会造成程序无法正确运行。

对于主频比较低的MCU，使用这里提供的软件模拟I2C从机进行I2C通信时，
建议使用100kpbs以下的通信速率，并且注意使用可以提高代码执行速度的代码优化配置。
#endif
void i2c_tick_inc(void);
void i2c_sw_slave_init(void);
void i2c_sw_gpio_exti_isr(void);
void i2c_sw_check_timeout(void);



#ifdef __cplusplus
}
#endif


#endif /*__SW_SLAVE_I2C_H_*/
