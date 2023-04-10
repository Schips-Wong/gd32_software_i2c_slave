/*!
    \file    main.c
    \brief   Software I2C-Slave Device

    \version 2023-04-10, V1.0.0, init version
*/

#include "com.h"
#include "delay.h"
#include "i2c_slave.h"
#include "gd32f4xx.h"
#include "systick.h"
#include <stdio.h>
#include "main.h"
#include "gd32f450i_eval.h"

int main(void)
{
    systick_config();

    com_init();

    i2c_sw_slave_init();

    printf("I2C SLAVE IS READY.\r\n");

    while(1) {
        //i2c_sw_check_timeout();
    }
}

/* retarget the C library printf function to the USART */
int fputc(int ch, FILE *f)
{
    usart_data_transmit(COM0, (uint8_t)ch);
    while(RESET == usart_flag_get(COM0, USART_FLAG_TBE));
    return ch;
}
