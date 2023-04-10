///////////////////////////////////////////////////////////////////////////////
// \license The MIT License (MIT)
//
// \brief
//	software simulation I2C slave device
//
// \version
// v1.0.0: 2023.04.10, Initial version.
///////////////////////////////////////////////////////////////////////////////

#if 0
## I2C通信的时序中关键点

1. START和ReSTART信号：用于标识I2C通信的开始，时序特点是SCL为高电平的时候，SDA从高电平变成低电平。
2. STOP信号：用于标识I2C通信的结束，时序特点是SCL为高电平的时候，SDA从低电平变成高电平。
3. 应答信号：I2C通信每传输完8个比特的数据位后，紧接着需要传输应答标志位，当该位为0时，是ACK应答信号，该位为1时，是NACK无应答信号。应答信号在SCL的第9个时钟周期的位置。
4. 数据采集时刻：I2C通信的数据在SCL的上升沿进行采集确认，所以在SCL的高电平期间，数据必须保持不变，防止数据采集出错。当然，START信号和STOP信号的时序在SCL高电平期间是特殊情况，具有专门的含义。
5. 数据更新时刻：I2C通信的数据更新需要在SCL为低电平的时候进行。

由于各个关键点基本都发生在SCL或SDA的上升沿或者下降沿的地方，所以可以将用于模拟I2C通信引脚的GPIO口配置成边沿中断，这样就可以通过中断实时抓取边沿信号，并在中断中进行及时的数据处理。使用GPIO的边沿中断来模拟I2C从机的好处是可以实时获取到START和STOP信号，I2C主机发过来的数据可以通过中断得到及时处理，而且程序主流程无需关心模拟I2C从机的相关处理，可以处理其他事务。

因为是I2C从机，所以SCL引脚直接固定成输入引脚即可，而SDA信号由于是双向的，所以需要根据I2C通信中的各个状态来设置输入或输出方向。另外，由于GPIO中断只在GPIO配置成输入时才会产生，所以默认情况下，SDA必须设置成输入引脚。

## 程序的具体设计思路

1. 将SCL和SDA引脚设置成GPIO的边沿中断模式，默认为输入引脚。I2C通信状态机设置成默认的IDLE状态。SCL的中断用于处理数据的收发，SDA的中断只用于START/ReSTART/STOP这些特殊信号的判断。
2. SDA引脚中断处理思路：发生下降沿中断，并且SCL为高电平，则收到START信号，状态机更新成START状态；发生上升沿中断，并且SCL为高电平，则收到STOP信号，紧接着I2C通信就应该处于空闲状态，所以这里直接将状态机设置成IDLE状态。

3. SCL引脚中断处理思路：

**A. 发生下降沿中断时**

A1. 如果状态机为START状态，则I2C通信正式开始，准备开始接收设备地址，状态机更新成DATA状态。

A2.  如果状态机为DATA状态，SCL下降沿计数小于8时，如果是主机读取数据，则更新SDA的位数据输出。SCL下降沿计数等于8时，进入应答阶段，状态机更新成ACK状态；如果是主机写入数据，并且是设备地址数据，则判断设备地址是否匹配，如果设备地址匹配，则将SDA设置成输出，并输出ACK信号，否则如果地址不匹配，则SDA保持为输入状态，不输出ACK信号；如果是主机读取数据，将SDA设置成输入，准备接收主机的应答信号。

A3. 如果状态机为ACK状态，这时应答信号已经传输完毕，状态机更新成DATA状态，准备继续接收或发送数据。如果是主机写入数据，将SDA设置成输入，继续接收后续数据；如果是主机读取数据，将SDA设置成输出，继续发送后续数据。

A4. 如果状态机为NACK状态，说明紧接着I2C通信将停止或重新启动，准备接收STOP或者ReSTART信号，所以需要将SDA设置成输入。此时状态机状态保持不变。

**B. 发生上升沿中断时**

B1. 如果状态机为DATA状态，I2C通信处于数据阶段，如果是主机写入数据，则采集主机通过SDA发送过来的位数据。

B2. 如果状态机为ACK状态，I2C通信处于应答阶段，如果是主机读取数据，则采集主机的应答信号，如果主机应答信号为1，说明主机发送了NACK的应答，状态机需要更新成NACK状态，准备接收停止或重新启动信号。
#endif

#include "i2c_slave.h"
#include "gd32f4xx.h"
#include "systick.h"
#include <stdio.h>


//#define i2c_debug(format,...)
#define i2c_debug printf

#define I2C_STA_IDLE			0
#define I2C_STA_START			1
#define I2C_STA_DATA			2
#define I2C_STA_ACK				3
#define I2C_STA_NACK			4
#define I2C_STA_STOP			5

#define I2C_READ				1
#define I2C_WRITE				0

#define GPIO_DIR_IN				0
#define GPIO_DIR_OUT			1
#define PIN_LOW  RESET
#define PIN_HIGH SET

#define SW_SLAVE_ADDR			(SW_SLAVE_ADDR_7BIT << 1)

struct SwSlaveI2C
{
	uint8_t State;					// I2C通信状态
	uint8_t Rw;						// I2C读写标志：0-写，1-读
	uint8_t SclFallCnt;				// SCL下降沿计数
	uint8_t Flag;					// I2C状态标志，BIT0：0-地址无效，1-地址匹配
	uint32_t StartMs;				// I2C通信起始时间，单位ms，用于判断通信是否超时
	uint8_t* RxBuf;					// 指向接收缓冲区的指针
	uint8_t* TxBuf;					// 指向发送缓冲区的指针
	uint8_t RxIdx;					// 接收缓冲区数据写入索引，最大值255
	uint8_t TxIdx;					// 发送缓冲区数据读取索引，最大值255
};


#define MAX_I2C_BUFF_SIZE 256
static uint8_t info_when_main_send[MAX_I2C_BUFF_SIZE] = {0x00};
static uint8_t data_for_main_read[MAX_I2C_BUFF_SIZE] = {0xa1, 0xb2, 0xc3, 0xd4, 0xe5, 0xf6, 0x77, 0x88};

static struct SwSlaveI2C SwSlaveI2C =
{
	I2C_STA_IDLE,		      // State
	I2C_WRITE,			      // Rw
	0, 					      // SclFallCnt
	0,					      // Flag
	0,					      // StartMs
	info_when_main_send,	  // RxBuf（每次i2c通信时，所变化的内容）
	data_for_main_read,		  // TxBuf（每次i2cget时，发送出去的内容）
	0,					      // RxIdx
	0					      // TxIdx
};

#define CHECK_PORTING



#define SW_SLAVE_SCL_PIN                   GPIO_PIN_8
#define SW_SLAVE_SCL_PORT                  GPIOB
#define SW_SLAVE_SCL_CLK                   RCU_GPIOB
#define SW_SLAVE_SCL_EXTI_LINE             EXTI_8
#define SW_SLAVE_SCL_EXTI_PORT_SOURCE      EXTI_SOURCE_GPIOB
#define SW_SLAVE_SCL_EXTI_PIN_SOURCE       EXTI_SOURCE_PIN8
#define SW_SLAVE_SCL_EXTI_IRQn             EXTI5_9_IRQn 
    

#define SW_SLAVE_SDA_PORT		           GPIOB
#define SW_SLAVE_SDA_PIN		           GPIO_PIN_9
#define SW_SLAVE_SDA_CLK                   RCU_GPIOB
#define SW_SLAVE_SDA_EXTI_LINE             EXTI_9
#define SW_SLAVE_SDA_EXTI_PORT_SOURCE      EXTI_SOURCE_GPIOB
#define SW_SLAVE_SDA_EXTI_PIN_SOURCE       EXTI_SOURCE_PIN9
#define SW_SLAVE_SDA_EXTI_IRQn             EXTI5_9_IRQn  
    


/*!
    \brief      this function handles GPIO exception
    \param[in]  none
    \param[out] none
    \retval     none
*/
void EXTI5_9_IRQHandler(void)
{
    i2c_sw_gpio_exti_isr();
}


/**
  * @brief  Configures slave I2C GPIO pin in interrupt mode
  * @param  None
  * @retval None
  */
CHECK_PORTING void i2c_sw_slave_init(void)
{
    /* enable the GPIOA, Interrupt clock*/
    rcu_periph_clock_enable(SW_SLAVE_SDA_CLK);
    rcu_periph_clock_enable(RCU_SYSCFG);

    /* configure SCL/SDA GPIO pin with pull up */
    gpio_mode_set(SW_SLAVE_SCL_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SW_SLAVE_SCL_PIN);
    gpio_mode_set(SW_SLAVE_SDA_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SW_SLAVE_SDA_PIN);

    /* enable and set key EXTI interrupt to the --highest-- priority */
    nvic_irq_enable(SW_SLAVE_SCL_EXTI_IRQn, 2, 1);
    //nvic_irq_enable(SW_SLAVE_SDA_EXTI_IRQn, 0, 1);

    /* connect key EXTI line to key GPIO pin */
    syscfg_exti_line_config(SW_SLAVE_SCL_EXTI_PORT_SOURCE, SW_SLAVE_SCL_EXTI_PIN_SOURCE);
    syscfg_exti_line_config(SW_SLAVE_SDA_EXTI_PORT_SOURCE, SW_SLAVE_SDA_EXTI_PIN_SOURCE);

    /* configure key EXTI line, as rising and falling edge trigger */
    exti_init(SW_SLAVE_SCL_EXTI_LINE, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
    exti_init(SW_SLAVE_SDA_EXTI_LINE, EXTI_INTERRUPT, EXTI_TRIG_BOTH);

    /* clear interrupt flag before real-usage */
    exti_interrupt_flag_clear(SW_SLAVE_SCL_EXTI_LINE);
    exti_interrupt_flag_clear(SW_SLAVE_SDA_EXTI_LINE);
#if 0
    for(i = 0; i < sizeof(data_for_main_read); i++)
    {
        data_for_main_read[i] = i;
    }
#endif
}

// 处理SCL的上下沿中断
CHECK_PORTING int is_scl_trig(void)
{
    //__HAL_GPIO_EXTI_GET_IT(SW_SLAVE_SCL_PIN) != RESET)
    return exti_interrupt_flag_get(SW_SLAVE_SCL_EXTI_LINE);
}

// 处理SDA的上下沿中断
CHECK_PORTING int is_sda_trig(void)
{
    //__HAL_GPIO_EXTI_GET_IT(SW_SLAVE_SDA_PIN) != RESET)
    return exti_interrupt_flag_get(SW_SLAVE_SDA_EXTI_LINE);
}

// 清除SCL中断
CHECK_PORTING void clr_scl_rtig_it(void)
{
    //__HAL_GPIO_EXTI_CLEAR_IT(SW_SLAVE_SCL_PIN);
    exti_interrupt_flag_clear(SW_SLAVE_SCL_EXTI_LINE);
}

// 清除SDA中断
CHECK_PORTING void clr_sda_rtig_it(void)
{
    //__HAL_GPIO_EXTI_CLEAR_IT(SW_SLAVE_SDA_PIN);
    exti_interrupt_flag_clear(SW_SLAVE_SDA_EXTI_LINE);
}

static uint32_t tick_cnt = 0;

CHECK_PORTING void i2c_tick_inc(void)
{
    tick_cnt++;
}

CHECK_PORTING static uint32_t get_i2c_tick(void)
{
    return tick_cnt;
}

#if 0
CHECK_PORTING static void reset_i2c_tick(void)
{
    tick_cnt = 0;
}
#endif

// 拉低SDA
CHECK_PORTING void clr_sda_pin()
{
    gpio_bit_reset(SW_SLAVE_SDA_PORT, SW_SLAVE_SDA_PIN);
}
// 拉高SDA
CHECK_PORTING void set_sda_pin()
{
    gpio_bit_set(SW_SLAVE_SDA_PORT, SW_SLAVE_SDA_PIN);
}

CHECK_PORTING void set_sda_dir(int dir)
{
    if(GPIO_DIR_IN == dir)
    {
        gpio_mode_set(SW_SLAVE_SDA_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SW_SLAVE_SDA_PIN);
    }else if(GPIO_DIR_OUT == dir)
    {
        gpio_mode_set(SW_SLAVE_SDA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SW_SLAVE_SDA_PIN);
    }
}

CHECK_PORTING void set_sda_output_value(int value)
{
    gpio_mode_set(SW_SLAVE_SDA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SW_SLAVE_SDA_PIN);
    if(value == PIN_HIGH)
    {
        gpio_bit_set(SW_SLAVE_SDA_PORT, SW_SLAVE_SDA_PIN);
    }else
    {
        gpio_bit_reset(SW_SLAVE_SDA_PORT, SW_SLAVE_SDA_PIN);
    }
}

CHECK_PORTING int get_scl_state(void)
{
    return gpio_input_bit_get(SW_SLAVE_SCL_PORT, SW_SLAVE_SCL_PIN);
}

CHECK_PORTING int get_sda_state(void)
{
    return gpio_input_bit_get(SW_SLAVE_SDA_PORT, SW_SLAVE_SDA_PIN);
}

// 为了确保模拟I2C从机通信的可靠性，额外设计了I2C通信超时处理函数。
// 在I2C通信进行的过程中，如果通信出现了中断，则通过超时判断来重置I2C从机状态，确保出现通信异常时可以从异常状态中自动恢复。
// 该函数需要在主流程中调用。
void i2c_sw_check_timeout(void)
{
	uint32_t TimeMs, TimeCurMs;

	if(SwSlaveI2C.State != I2C_STA_IDLE)
	{
		TimeCurMs = get_i2c_tick();
		if(TimeCurMs >= SwSlaveI2C.StartMs)
		{
			TimeMs = TimeCurMs - SwSlaveI2C.StartMs;
		}
		else
		{
			TimeMs = ~(SwSlaveI2C.StartMs - TimeCurMs) + 1;
		}
		if(500 >= TimeMs)
		{
			// I2C通信超时的话，重置状态机，并把SDA设置成输入
			SwSlaveI2C.State = I2C_STA_IDLE;
            set_sda_dir(GPIO_DIR_IN);
		}
	}
    delay_1ms(50); // 要保证主循环内，要有足够的延迟来防止I2C通信超时
}

void reg_changed(uint8_t reg, uint8_t value)
{
    if(reg == 0x00)
    {
        i2c_debug("REG 0x00 VALUE CHANGE to [0x%02x]\r\n", value);
    }
    if(reg == 0x01)
    {
        i2c_debug("REG 0x01 VALUE CHANGE to [0x%02x]\r\n", value);
    }
}

void i2c_sw_action_after_done(void)
{
    int write_len;
    int i, reg;

    // 写动作处理
    if(!SwSlaveI2C.RxIdx)
        return;
    write_len = SwSlaveI2C.RxIdx;
    // I2C设备地址为8bit，其中最低位为读写位，0为写，1为读。
    if( !((SwSlaveI2C.RxBuf[0]) == ((SW_SLAVE_ADDR_7BIT << 1) | I2C_WRITE) ))
    {
        // 仅支持写；而读动作不在这里做触发。
        i2c_debug("Only Support write action\r\n");
        return;
    }

    // 防止 类似 i2cset -f -y 0 0x51 0x00 这样子的命令发过来（虽然可以正确响应）
    if(write_len < 3)
    {
        i2c_debug("Require A Value for Write\r\n");
        return ;
    }
    if(write_len == 3)
    {
        i2c_debug("Single Value when Writing\r\n");
    }else if(write_len > 4)
    {
        i2c_debug("MULIT Value when Writing\r\n");
    }

    reg = SwSlaveI2C.RxBuf[1];
#if 0
    i2c_debug("Goint To Write REG [0x%02x] with len %d\r\n", reg, write_len  - 2);
    for(i = 2; i < write_len; i++)
    {
        i2c_debug("0x%02x ", SwSlaveI2C.RxBuf[i]);
    }
    i2c_debug("\r\n");
#endif
    for(i = 2; i < write_len; i++)
    {
        if(reg + i - 2 > MAX_I2C_BUFF_SIZE - 1)
        {
            i2c_debug("OVERFOLW\r\n");
            break;
        }
        SwSlaveI2C.TxBuf[reg + i - 2 ] =  SwSlaveI2C.RxBuf[i];
        reg_changed(reg + i - 2, SwSlaveI2C.RxBuf[i]);
    }

    SwSlaveI2C.RxIdx = 0;

    for(i = 0; i < write_len; i++)
    {
        SwSlaveI2C.RxBuf[i] = 0x0;
    }
}

/**
  * @brief EXTI line interruption service
  * @param None
  * @retval None
  */
// 由于SCL/SDA引脚被设置成中断引脚，需要实现GPIO的中断处理函数。
// 中断处理函数中已经包含了软件模拟I2C从机的所有功能。代码如下。
// 在中断入口函数中调用模拟I2C从机的GPIO口中断处理函数i2c_sw_gpio_exti_isr()。
void i2c_sw_gpio_exti_isr(void)
{
	// 处理SCL的上下沿中断
	if(is_scl_trig())
	{
		clr_scl_rtig_it();
		// 更新通信起始时间
		SwSlaveI2C.StartMs = get_i2c_tick();
		// SCL的下降沿事件处理，此时需要更新要传输的数据
		if(get_scl_state() == PIN_LOW)
		{
			switch(SwSlaveI2C.State)
			{
				case I2C_STA_START:		// 起始信号的下降沿，初始化相关参数并转到接收比特数据状态
					SwSlaveI2C.SclFallCnt = 0;
					SwSlaveI2C.TxIdx = SwSlaveI2C.RxBuf[1]; // For Read
					SwSlaveI2C.RxIdx = 0;
					SwSlaveI2C.Flag = 0;	// 默认地址不匹配
					SwSlaveI2C.RxBuf[SwSlaveI2C.RxIdx] = 0;
					SwSlaveI2C.Rw = I2C_WRITE;	// 第1字节为设备地址，一定是写入
					SwSlaveI2C.State = I2C_STA_DATA;
					break;
				case I2C_STA_DATA:
					SwSlaveI2C.SclFallCnt++;
					if(8 > SwSlaveI2C.SclFallCnt)
					{
						// 如果是主机读取数据，则在SCL低电平时更新比特数据
						if(SwSlaveI2C.Rw == I2C_READ)
						{
							if(SwSlaveI2C.TxBuf[SwSlaveI2C.TxIdx] & (1 << (7 - SwSlaveI2C.SclFallCnt)))
							{
								set_sda_pin();
							}
							else
							{
								clr_sda_pin();
							}
						}
					}
					else if(8 == SwSlaveI2C.SclFallCnt)
					{
						if(SwSlaveI2C.Rw == I2C_WRITE)
						{
							// 从第一个地址字节中获取读写标志位，并判断地址是否匹配
							if(SwSlaveI2C.RxIdx == 0)
							{
								if((SwSlaveI2C.RxBuf[0] & 0xFE) == (SW_SLAVE_ADDR_7BIT << 1))
								{
									SwSlaveI2C.Flag = 1;	// 地址匹配
									SwSlaveI2C.Rw = SwSlaveI2C.RxBuf[0] & 0x01;
								}
							}
							if(SwSlaveI2C.Flag)
							{
								// 如果是主机写入数据，且地址匹配，则接收完8比特数据后，需要发送ACK信号进行应答
                                set_sda_output_value(PIN_LOW);
							}
						}
						else
						{
							// 如果是主机读取数据，需要将SDA设置成输入以便判断应答标志位状态
                            set_sda_dir(GPIO_DIR_IN);
							// 如果是主机读取数据，准备发送下一个字节的数据
                            SwSlaveI2C.TxIdx++;
                            if(SwSlaveI2C.TxIdx >= MAX_I2C_BUFF_SIZE - 1)
                            {
                                SwSlaveI2C.TxIdx = 0;
                            }
						}
						// 接收或发送完8比特数据后，准备发送或接收应答信号
						SwSlaveI2C.State = I2C_STA_ACK;
					}
					break;
				case I2C_STA_ACK:
					SwSlaveI2C.SclFallCnt = 0;
					if(SwSlaveI2C.Rw == I2C_WRITE)
					{
						// 如果是主机写入数据，且ACK发送完毕，则SDA设置成输入，继续接收数据
                        set_sda_dir(GPIO_DIR_IN);
						SwSlaveI2C.RxIdx++;
                        if(SwSlaveI2C.RxIdx >= MAX_I2C_BUFF_SIZE - 1)
                        {
                            SwSlaveI2C.RxIdx = 0;
                        }
						SwSlaveI2C.RxBuf[SwSlaveI2C.RxIdx] = 0;
					}
					else
					{
						// 如果是主机读取数据，且ACK接收完毕，则SDA设置成输出，继续发送数据
                        set_sda_dir(GPIO_DIR_OUT);

						if(SwSlaveI2C.TxBuf[SwSlaveI2C.TxIdx] & 0x80)
						{
                            set_sda_pin();
						}
						else
						{
							clr_sda_pin();
						}
					}
					SwSlaveI2C.State = I2C_STA_DATA;
					break;
				case I2C_STA_NACK:		// 如果收到了NACK，则后面将是STOP或者ReSTART信号，需要将SDA设置成输入
					SwSlaveI2C.SclFallCnt = 0;
                    set_sda_dir(GPIO_DIR_IN);
					break;
			}
		}
		// SCL的上升沿事件处理，此时需要采集数据，而且在数据阶段，SCL高电平时数据必须保持不变
		else //if(get_scl_state() == PIN_HIGH)
		{
			switch(SwSlaveI2C.State)
			{
				case I2C_STA_DATA:	// 数据阶段，如果是主机写入数据，则采集比特数据
					if((I2C_WRITE == SwSlaveI2C.Rw) && (8 > SwSlaveI2C.SclFallCnt))
					{
						if(get_sda_state() != PIN_LOW)
						{
							SwSlaveI2C.RxBuf[SwSlaveI2C.RxIdx] |= (1 << (7 - SwSlaveI2C.SclFallCnt));
						}
					}
					break;
				case I2C_STA_ACK:	// 应答阶段，如果是主机读取数据，则判断ACK/NACK信号，默认状态是ACK
					if((SwSlaveI2C.Rw == I2C_READ) && (get_sda_state() != PIN_LOW))
					{
						SwSlaveI2C.State = I2C_STA_NACK;
					}
					break;
			}
		}
	}
	if(is_sda_trig())
	{
		clr_sda_rtig_it();
		if(get_sda_state() == PIN_LOW)
		{
			// SCL为高电平时，SDA从高变低，说明是起始信号
			if(get_scl_state() == PIN_HIGH)
			{
				SwSlaveI2C.State = I2C_STA_START;
			}
		}
		else
		{
			// SCL为高电平时，SDA从低变高，说明是停止信号，一次I2C通信结束，直接将状态设置成空闲
			if(get_scl_state() == PIN_HIGH)
			{
                i2c_sw_action_after_done();
				SwSlaveI2C.State = I2C_STA_IDLE;
			}
		}
	}
}
