/*!
    \file    i2c_slave_0.c
    \brief   Software I2C-Slave Device0 instantiation

    \version 2023-04-10, V1.0.0, init version
    \version 2023-04-12, V1.0.1, Friendly API
*/
#include "i2c_slave_core.h"
#include "i2c_slave_0.h"

#include "gd32f4xx.h"
#include "systick.h"
#include <stdio.h>

#define i2c_debug(format,...)
//#define i2c_debug printf

#define SW_SLAVE_SCL_PIN                   GPIO_PIN_8
#define SW_SLAVE_SCL_PORT                  GPIOB
#define SW_SLAVE_SCL_CLK                   RCU_GPIOB
#define SW_SLAVE_SCL_EXTI_LINE             EXTI_8
#define SW_SLAVE_SCL_EXTI_PORT_SOURCE      EXTI_SOURCE_GPIOB
#define SW_SLAVE_SCL_EXTI_PIN_SOURCE       EXTI_SOURCE_PIN8
#define SW_SLAVE_SCL_EXTI_IRQn             EXTI5_9_IRQn


#define SW_SLAVE_SDA_PORT                  GPIOB
#define SW_SLAVE_SDA_PIN                   GPIO_PIN_9
#define SW_SLAVE_SDA_CLK                   RCU_GPIOB
#define SW_SLAVE_SDA_EXTI_LINE             EXTI_9
#define SW_SLAVE_SDA_EXTI_PORT_SOURCE      EXTI_SOURCE_GPIOB
#define SW_SLAVE_SDA_EXTI_PIN_SOURCE       EXTI_SOURCE_PIN9
#define SW_SLAVE_SDA_EXTI_IRQn             EXTI5_9_IRQn

#define CHECK_PORTING

#define I2C0_VERSION         0x01

#define MAX_I2C_BUFF_SIZE    256
static uint8_t info_when_main_send[MAX_I2C_BUFF_SIZE] = {0x00};
static uint8_t data_for_main_read[MAX_I2C_BUFF_SIZE] = {I2C0_VERSION, 0xb2, 0xc3, 0xd4, 0xe5, 0xf6, 0x77, 0x88};


static struct SwSlaveI2C SwSlaveI2C0;


/**
  * @brief  Configures slave I2C device0
  * @param  None
  * @retval None
  */
void i2c0_sw_slave_init(void)
{
    i2c_sw_slave_init(&SwSlaveI2C0);
#if 0
    for(i = 0; i < SwSlaveI2C.MaxBufSize; i++)
    {
        SwSlaveI2C.TxIdx[i] = 0;
    }
    i2c0_reg_update_poll();
#endif
}


/**
  * @brief  update I2C regs period 周期性地更新I2C寄存器，以方便主机读取
  * @param  None
  * @retval None
  */
void i2c0_reg_update_poll(void)
{
    uint16_t tmp;
    static int index = 0; // 每次执行时，只更新一个寄存器
    uint8_t* TxBuf = SwSlaveI2C0.TxBuf;

    switch(index)
    {
        /*  */ case REG_INDEX_0x00: // read only
            TxBuf[I2C0_REG0_VERSION] = I2C0_VERSION;

        break; case REG_INDEX_0x01:
            tmp = 0x02a5;
            TxBuf[I2C0_REG10_TEMP_H] = tmp >> 8;
            TxBuf[I2C0_REG11_TEMP_L] = tmp & 0xff;
            //printf("TEMP is %f\r\n",tmp * 0.0625);

        break; default:
            index = 0;
            delay_1ms(10);
            return;
    }
    index++;
}

/**
  * @brief  check new value of reg 当主机写入寄存器值时，通过这个函数来把控寄存器的值。
                                   因为在有些场景下，一些寄存器的值或者某几个位就是不允许更新的
    \param[in]  reg  : 即将更新的寄存器
    \param[int] value: 寄存器未来的值
  * @retval value for reg.
  */
uint8_t i2c0_reg_will_changed(uint8_t reg, uint8_t value)
{
    if(reg == I2C0_REG0_VERSION)
    {
        i2c_debug("THIS reg is READ ONLY\r\n");
        return I2C0_VERSION;
    }

    i2c_debug("REG [0x%02x] VALUE CHANGE to [0x%02x]\r\n", reg, value);
    return value;
}


/*!
    \brief      this function handles I2C exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void EXTI5_9_IRQHandler(void)
{
    i2c_sw_gpio_exti_isr(&SwSlaveI2C0);
}


CHECK_PORTING void i2c_slave_with_soc_init(void)
{
    /* enable the GPIO, Interrupt clock*/
    rcu_periph_clock_enable(SW_SLAVE_SDA_CLK);
    rcu_periph_clock_enable(RCU_SYSCFG);

    /* configure SCL/SDA GPIO pin with pull up */
    gpio_mode_set(SW_SLAVE_SCL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SW_SLAVE_SCL_PIN);
    gpio_mode_set(SW_SLAVE_SDA_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SW_SLAVE_SDA_PIN);

    /* enable and set key EXTI interrupt to the --highest-- priority */
    nvic_irq_enable(SW_SLAVE_SCL_EXTI_IRQn, 2U, 0U);
    //nvic_irq_enable(SW_SLAVE_SDA_EXTI_IRQn, 2U, 1U);

    /* connect key EXTI line to key GPIO pin */
    syscfg_exti_line_config(SW_SLAVE_SCL_EXTI_PORT_SOURCE, SW_SLAVE_SCL_EXTI_PIN_SOURCE);
    syscfg_exti_line_config(SW_SLAVE_SDA_EXTI_PORT_SOURCE, SW_SLAVE_SDA_EXTI_PIN_SOURCE);

    /* configure key EXTI line, as rising and falling edge trigger */
    exti_init(SW_SLAVE_SCL_EXTI_LINE, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
    exti_init(SW_SLAVE_SDA_EXTI_LINE, EXTI_INTERRUPT, EXTI_TRIG_BOTH);

    /* clear interrupt flag before real-usage */
    exti_interrupt_flag_clear(SW_SLAVE_SCL_EXTI_LINE);
    exti_interrupt_flag_clear(SW_SLAVE_SDA_EXTI_LINE);
}

// 处理SCL的上下沿中断
CHECK_PORTING int is_scl0_trig(void)
{
    //__HAL_GPIO_EXTI_GET_IT(SW_SLAVE_SCL_PIN) != RESET)
    return exti_interrupt_flag_get(SW_SLAVE_SCL_EXTI_LINE);
}

// 处理SDA的上下沿中断
CHECK_PORTING int is_sda0_trig(void)
{
    //__HAL_GPIO_EXTI_GET_IT(SW_SLAVE_SDA_PIN) != RESET)
    return exti_interrupt_flag_get(SW_SLAVE_SDA_EXTI_LINE);
}

// 清除SCL中断
CHECK_PORTING void clr_scl0_rtig_it(void)
{
    //__HAL_GPIO_EXTI_CLEAR_IT(SW_SLAVE_SCL_PIN);
    exti_interrupt_flag_clear(SW_SLAVE_SCL_EXTI_LINE);
}

// 清除SDA中断
CHECK_PORTING void clr_sda0_rtig_it(void)
{
    //__HAL_GPIO_EXTI_CLEAR_IT(SW_SLAVE_SDA_PIN);
    exti_interrupt_flag_clear(SW_SLAVE_SDA_EXTI_LINE);
}

// 拉低SDA
CHECK_PORTING void clr_sda0_pin()
{
    gpio_bit_reset(SW_SLAVE_SDA_PORT, SW_SLAVE_SDA_PIN);
}
// 拉高SDA
CHECK_PORTING void set_sda0_pin()
{
    gpio_bit_set(SW_SLAVE_SDA_PORT, SW_SLAVE_SDA_PIN);
}

CHECK_PORTING void set_sda0_dir(int dir)
{
    if(GPIO_DIR_IN == dir)
    {
        gpio_mode_set(SW_SLAVE_SDA_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SW_SLAVE_SDA_PIN);
    }else if(GPIO_DIR_OUT == dir)
    {
        gpio_mode_set(SW_SLAVE_SDA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SW_SLAVE_SDA_PIN);
    }
}

//CHECK_PORTING void set_sda_output_value(int value)
//{
//    gpio_mode_set(SW_SLAVE_SDA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SW_SLAVE_SDA_PIN);
//    if(value == PIN_HIGH)
//    {
//        gpio_bit_set(SW_SLAVE_SDA_PORT, SW_SLAVE_SDA_PIN);
//    }else
//    {
//        gpio_bit_reset(SW_SLAVE_SDA_PORT, SW_SLAVE_SDA_PIN);
//    }
//}

CHECK_PORTING int get_scl0_state(void)
{
    return gpio_input_bit_get(SW_SLAVE_SCL_PORT, SW_SLAVE_SCL_PIN);
}

CHECK_PORTING int get_sda0_state(void)
{
    return gpio_input_bit_get(SW_SLAVE_SDA_PORT, SW_SLAVE_SDA_PIN);
}

static struct SwSlaveI2C SwSlaveI2C0 =
{
    I2C0_SLAVE_ADDR_7BIT,       // I2C ADDR
    I2C_STA_IDLE,             // State
    I2C_WRITE,                // Rw
    0,                        // SclFallCnt
    0,                        // Flag
    0,                        // StartMs
    info_when_main_send,      // RxBuf（每次i2c通信时，所变化的内容）
    data_for_main_read,       // TxBuf（每次i2cget时，发送出去的内容）
    0,                        // RxIdx
    0,                        // TxIdx
    MAX_I2C_BUFF_SIZE,        // MaxBufSize for Rx/Tx Buff
    0,                        // TickCnt For Timeout
    i2c_slave_with_soc_init,  // init for Pin/Interrupt
    i2c0_reg_will_changed,    // Reg keepper （规范化REG值读写）
    set_sda0_dir,             // SDA pin GPIO handle
    is_scl0_trig,             // SCL pin GPIO handle
    clr_scl0_rtig_it,         // SCL pin GPIO handle
    is_sda0_trig,             // SDA pin GPIO handle
    clr_sda0_rtig_it,         // SDA pin GPIO handle
    get_scl0_state,           // SCL pin GPIO handle
    get_sda0_state,           // SDA pin GPIO handle
    set_sda0_pin,             // SDA pin GPIO handle
    clr_sda0_pin,             // SDA pin GPIO handle
};
