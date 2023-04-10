#include "delay.h"
#include "gd32f4xx.h"
 
//定时器不分频
#define DELAY_TIMER_PRESCALER	0	//分频值0,频率和系统时钟一样
//1us的定时计数
#define DELAY_TIMER_PERIOD		(SystemCoreClock / 1000 /1000 ) - 1	//71	//SystemCoreClock / 1000 /1000
 
#define TMR_ENABLE()	TIMER_CTL0(DELAY_TIMER)|=(uint32_t)TIMER_CTL0_CEN	//开启定时器
#define TMR_DISABEL()	TIMER_CTL0(DELAY_TIMER) &= ~(uint32_t)TIMER_CTL0_CEN //关闭定时器
#define TMR_CLR_FLAG()	TIMER_INTF(DELAY_TIMER) = (~(uint32_t)TIMER_INT_FLAG_UP)	//清除中断标志位
#define TMR_SET_CNT(X)	TIMER_CNT(DELAY_TIMER) = (uint32_t)(X)	//配置计数器
 
uint16_t us_count;
 
/*!
    \brief      初始化延时函数定时器
    \retval     none
*/
void delay_timer_init(void)
{
	timer_parameter_struct tmr;
 
	rcu_periph_clock_enable(RCU_DELAY_TIMER);
    timer_deinit(DELAY_TIMER);
 
	 /* TIMERx configuration */
    tmr.prescaler         = DELAY_TIMER_PRESCALER;
    tmr.alignedmode       = TIMER_COUNTER_EDGE;
    tmr.counterdirection  = TIMER_COUNTER_UP;
    tmr.period            = DELAY_TIMER_PERIOD;
    tmr.clockdivision     = TIMER_CKDIV_DIV1;
    tmr.repetitioncounter = 0;
    timer_init(DELAY_TIMER,&tmr);
	
    timer_interrupt_enable(DELAY_TIMER,TIMER_INT_UP); //使能更新中断
	nvic_irq_enable(DELAY_TIMER_IRQn,2,0); //使能中断线
	TMR_DISABEL();
}
 
 
/*----------------------------------------------------------------------------------
Function   :DELAY_TIMER_IRQHandler
Description:定时器中断
Input      :无
Retrun     :无
----------------------------------------------------------------------------------*/
void DELAY_TIMER_IRQHandler(void)
{
    //timer_interrupt_flag_clear(DELAY_TIMER, TIMER_INT_FLAG_UP); //清除中断标志位
	TMR_CLR_FLAG();
    us_count++;
}
 
 
/*!
    \brief      延时us
    \param[in]  count   : 微秒数
    \retval     none
*/
void delay_1us(uint16_t count)
{
	TMR_SET_CNT(0);	//timer_counter_value_config(DELAY_TIMER,0);	
	TMR_ENABLE();	//timer_enable
	us_count = 0;		
	while (us_count < count);
	TMR_DISABEL();	//timer_disable
}

#if 0
void delay_1ms_by_timer(uint16_t count)
{
   while (count--)
   {
	   delay_1us(1000);	//为了避开原来的systick中断
   }          
}
#endif
