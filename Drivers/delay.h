#ifndef DELAY_H
#define DELAY_H
 
#include "gd32f4xx_timer.h"
#define DELAY_TIMER				TIMER4
#define RCU_DELAY_TIMER			RCU_TIMER4
#define DELAY_TIMER_IRQHandler	TIMER4_IRQHandler
#define DELAY_TIMER_IRQn		TIMER4_IRQn

extern uint16_t us_count;

/*!
    \brief      初始化延时函数定时器
    \retval     none
*/
void delay_timer_init(void);

/*!
    \brief      延时us
    \param[in]  count   : 微秒数
    \retval     none
*/
void delay_1us(uint16_t count);
 
#endif /* DELAY_H */
