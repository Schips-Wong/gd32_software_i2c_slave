#ifndef __COM_H__
#define __COM_H__

#include <stdint.h>
#include "gd32f4xx.h"

void com_init(void);


/* COM0 - USART 0*/
#define COM0                        USART0
#define COM0_BAUDRATE               115200U
#define COM0_CLK                    RCU_USART0

#define COM0_TX_PIN                 GPIO_PIN_9
#define COM0_RX_PIN                 GPIO_PIN_10

#define COM0_GPIO_PORT              GPIOA
#define COM0_GPIO_CLK               RCU_GPIOA
#define COM0_AF                     GPIO_AF_7
#define COM0_IRQ                    USART0_IRQn
#define COM0_IRQ_PRIORITY           0


/* COM1 - UART4*/
#define COM1                        UART4
#define COM1_BAUDRATE               115200U
#define COM1_CLK                    RCU_UART4

#define COM1_TX_PIN                 GPIO_PIN_12
#define COM1_RX_PIN                 GPIO_PIN_2
                                    
#define COM1_GPIO_PORT              GPIOD
#define COM1_GPIO_CLK               RCU_GPIOD
#define COM1_AF                     GPIO_AF_8

#define COM1_IRQ                    UART4_IRQn
#define COM1_IRQ_PRIORITY           1

/*!
    \brief      init Uart port
    \retval     none
*/
void com_init(void);


/*!
    \brief      get Uart data from buff
    \param[in]  com          : COM0, COM1
    \param[out]  len         : data len of recv uart port
    \retval     uart data buf; error or empty will return NULL
*/
unsigned char* com_try_recv(uint32_t com, int *len);

/*!
    \brief      send Uart data
    \param[in]  com          : COM0, COM1
    \param[in]  data         : send data
    \param[in]  len          : len of send data to uart port
    \retval     none
*/
void com_send(uint32_t com, unsigned char* data, int len);

#endif /*__COM_H__*/
