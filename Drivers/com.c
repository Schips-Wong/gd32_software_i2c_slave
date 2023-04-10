#include "com.h"
#include <string.h>
#include "gd32f4xx.h"
#include "gd32f450i_eval.h"
#include "systick.h"

volatile uint8_t USART0_Rxbuffer[200] = {0x0};
volatile uint32_t USART0_Rxcounter = 0;
volatile uint32_t USART0_ReceiveState = 0;

volatile uint8_t UART3_Rxbuffer[200] = {0x0};
volatile uint32_t UART3_Rxcounter = 0;
volatile uint32_t UART3_ReceiveState = 0;

volatile uint8_t UART4_Rxbuffer[200] = {0x0};
volatile uint32_t UART4_Rxcounter = 0;
volatile uint32_t UART4_ReceiveState = 0;

void USART0_SendArray(uint8_t *buf,uint8_t len);
void UART3_SendArray(uint8_t *buf,uint8_t len);

static rcu_periph_enum  COM_CLK[] = {
    COM0_CLK,
    COM1_CLK
};

static rcu_periph_enum  COM_GPIO_CLK[] = {
    COM0_GPIO_CLK,
    COM1_GPIO_CLK
};

static uint32_t  COM_AF[] = {
    COM0_AF,
    COM1_AF
};

static uint32_t         COM_TX_PIN[] = {
    COM0_TX_PIN,
    COM1_TX_PIN,
};

static uint32_t         COM_RX_PIN[] = {
    COM0_RX_PIN,
    COM1_RX_PIN
};

static uint32_t         COM_PORT[] = {
    COM0_GPIO_PORT,
    COM1_GPIO_PORT
};

static uint32_t          COM_BAUDRATE[] = {
    COM0_BAUDRATE,
    COM1_BAUDRATE,
};

static uint8_t       COM_IRQ[] = {
    COM0_IRQ,
    COM1_IRQ
};
static uint8_t      COM_IRQ_PRIORITY[] = 
{
    COM0_IRQ_PRIORITY,
    COM1_IRQ_PRIORITY
};
void _com_init(uint32_t com);

void _com1_init(void);

void com_init(void)
{
    _com_init(COM0);
    //_com_init(COM1);
    _com1_init();
}

/*!
    \brief      get Uart data from buff
    \param[in]  com          : COM0, COM1
    \param[out]  len         : data len of recv uart port
    \retval     uart data buf; error or empty will return NULL
*/
unsigned char* com_try_recv(uint32_t com, int *len)
{
    unsigned char *rxbuf = NULL;
    int recv_len;
    int delay = 10;
    int retry = 0;

    if(len == NULL)
    {
        return NULL;
    }

    if(com == COM0)
    {
        if(USART0_ReceiveState == 0)
        {
            return NULL;
        }
        recv_len = USART0_Rxcounter;
        while(1)
        {
            if(retry++ == 20)
            {
                break;
            }
            delay_1ms(delay);
            if(recv_len == USART0_Rxcounter)
            {
                break;
            }
        }
        USART0_ReceiveState = 0;

        rxbuf = (unsigned char *)USART0_Rxbuffer;
        *len = USART0_Rxcounter;
        USART0_Rxcounter = 0;
    }

    if(com == COM1)
    {
        if(UART4_ReceiveState == 0)
        {
            return NULL;
        }
        recv_len = UART4_Rxcounter;
        while(1)
        {
            if(retry++ == 20)
            {
                break;
            }
            delay_1ms(delay);
            if(recv_len == UART4_Rxcounter)
            {
                break;
            }
        }
        UART4_ReceiveState = 0;

        rxbuf = (unsigned char *)UART4_Rxbuffer;
        *len = UART4_Rxcounter;
        UART4_Rxcounter = 0;
    }
    return rxbuf;
}

void UART4_SendArray(uint8_t *buf,uint8_t len);

/*!
    \brief      send Uart data
    \param[in]  com          : COM0, COM1
    \param[in]  data         : send data
    \param[in]  len          : len of send data to uart port
    \retval     none
*/
void com_send(uint32_t com, unsigned char* data, int len)
{
    if(com == COM0)
    {
        USART0_SendArray(data, len);
    }

    if(com == COM1)
    {
        UART4_SendArray(data, len);
    }
}

/*!
  \brief    configure COM port
  \param[in]  COM: COM on the board
  \arg        COM0_USART0: COM on the board
  \param[out] none
  \retval     none
  */
void _com_init(uint32_t com)
{
    uint32_t index;

    if(com == COM0)
    {
        index = 0;
    }else if (com == COM1)
    {
        index = 1;
    }

    /* enable GPIO clock */
    rcu_periph_clock_enable(COM_GPIO_CLK[index]);
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);

    /* enable USART clock */
    rcu_periph_clock_enable(COM_CLK[index]);

    /* connect port to USARTx_Tx */
    gpio_af_set(COM_PORT[index], COM_AF[index], COM_TX_PIN[index]);

    /* connect port to USARTx_Rx */
    gpio_af_set(COM_PORT[index], COM_AF[index], COM_RX_PIN[index]);

    /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(COM_PORT[index], GPIO_MODE_AF, GPIO_PUPD_PULLUP,COM_TX_PIN[index]);
    gpio_output_options_set(COM_PORT[index], GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,COM_TX_PIN[index]);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(COM_PORT[index], GPIO_MODE_AF, GPIO_PUPD_PULLUP,COM_RX_PIN[index]);
    gpio_output_options_set(COM_PORT[index], GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,COM_RX_PIN[index]);

    /* USART configure */
    usart_deinit(com);
    usart_baudrate_set(com,    COM_BAUDRATE[index]);
    usart_receive_config(com,  USART_RECEIVE_ENABLE);
    usart_transmit_config(com, USART_TRANSMIT_ENABLE);
    usart_enable(com);

	/* USART IRQ set */
	nvic_irq_enable(COM_IRQ[index], COM_IRQ_PRIORITY[index], 0); // nvic_irq_enable(USART0_IRQn, 0, 0);
	usart_interrupt_enable(com, USART_INT_RBNE);  // usart_interrupt_enable(USART2, USART_INT_RBNE);
}


/*!
  \brief    configure COM port
  \param[in]  COM: COM on the board
  \arg        COM0_USART0: COM on the board
  \param[out] none
  \retval     none
  */
void _com1_init(void)
{
    uint32_t index;
    uint32_t com = COM1;

    if(com == COM0)
    {
        index = 0;
    }else if (com == COM1)
    {
        index = 1;
    }

    /* enable GPIO clock */
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOD);

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_UART4);

    /* connect port to USARTx_Tx */// PC12
    gpio_af_set(GPIOC, COM_AF[index], GPIO_PIN_12);

    /* connect port to USARTx_Rx */   //PD2
    gpio_af_set(GPIOD, COM_AF[index], GPIO_PIN_2);

    
    
    /* configure USART Tx as alternate function push-pull */
    gpio_mode_set(GPIOC, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_12);
    gpio_output_options_set(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_12);

    /* configure USART Rx as alternate function push-pull */
    gpio_mode_set(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_2);
    gpio_output_options_set(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);

    /* USART configure */
    usart_deinit(com);
    usart_baudrate_set(com,    COM_BAUDRATE[index]);
    usart_receive_config(com,  USART_RECEIVE_ENABLE);
    usart_transmit_config(com, USART_TRANSMIT_ENABLE);
    usart_enable(com);

	/* USART IRQ set */
	nvic_irq_enable(COM_IRQ[index], COM_IRQ_PRIORITY[index], 0); // nvic_irq_enable(USART0_IRQn, 0, 0);
	usart_interrupt_enable(com, USART_INT_RBNE);  // usart_interrupt_enable(USART2, USART_INT_RBNE);
}

void USART0_SendByte(uint8_t ch);
/************************* USART 0************************/
void USART0_IRQHandler(void)
{
    if(usart_flag_get(USART0, USART_FLAG_RBNE) != RESET)
    {
        USART0_Rxbuffer[USART0_Rxcounter++] = usart_data_receive(USART0);
        if(USART0_Rxcounter >= 200)
        {
            USART0_Rxcounter = 0;
        }

        usart_interrupt_flag_clear(USART0, USART_INT_FLAG_RBNE);

        USART0_ReceiveState = 1;
        //USART0_SendByte(USART0_Rxbuffer[USART0_Rxcounter -1]);
        //while(RESET == usart_flag_get(USART0, USART_FLAG_TBE));//发送完成判断
    }
}

void USART0_SendByte(uint8_t ch)
{
    usart_data_transmit(USART0, ch);
    while (RESET == usart_flag_get(USART0, USART_FLAG_TBE));
}

void USART0_SendArray(uint8_t *buf,uint8_t len)
{
    uint8_t t;
    for(t=0;t<len;t++)
    {
        usart_data_transmit(USART0,buf[t]);
        while (RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    }
    while(usart_flag_get(USART0, USART_FLAG_TC) == RESET);
}

void USART0_Send_String(char *send_pt)
{
    USART0_SendArray((uint8_t *)send_pt,strlen(send_pt));
}

/************************* UART 3************************/
void UART3_IRQHandler(void)
{
    if(usart_flag_get(UART3,USART_FLAG_RBNE) != RESET)
    {
        UART3_Rxbuffer[UART3_Rxcounter++] = usart_data_receive(UART3);
        if(UART3_Rxcounter>=200)
        {
            UART3_Rxcounter=0;
        }

        usart_interrupt_flag_clear(UART3,USART_INT_FLAG_RBNE);

        UART3_ReceiveState = 1;
    }
}

void UART3_SendByte(uint8_t ch)
{
    usart_data_transmit(UART3,ch);
    while (RESET == usart_flag_get(UART3, USART_FLAG_TBE));
}

void UART3_SendArray(uint8_t *buf,uint8_t len)
{
    uint8_t t;
    for(t=0;t<len;t++)
    {
        usart_data_transmit(UART3,buf[t]);
        while (RESET == usart_flag_get(UART3, USART_FLAG_TBE));
    }
    while(usart_flag_get(UART3, USART_FLAG_TC) == RESET);

}

void Uart3_Send_String(char *send_pt)
{
    UART3_SendArray((uint8_t *)send_pt,strlen(send_pt));
}

/************************* UART 4************************/
void UART4_IRQHandler(void)
{
    if(usart_flag_get(UART4,USART_FLAG_RBNE) != RESET)
    {
        
        UART4_Rxbuffer[UART4_Rxcounter++] = usart_data_receive(UART4);
        printf("Recv %x at %d\r\n", UART4_Rxbuffer[UART4_Rxcounter -1 ], UART4_Rxcounter -1); 
        if(UART4_Rxcounter>=200)
        {
            UART4_Rxcounter=0;
        }

        usart_interrupt_flag_clear(UART4, USART_INT_FLAG_RBNE);

        UART4_ReceiveState = 1;
    }
}

void UART4_SendByte(uint8_t ch)
{
    usart_data_transmit(UART4,ch);
    while (RESET == usart_flag_get(UART4, USART_FLAG_TBE));
}

void UART4_SendArray(uint8_t *buf,uint8_t len)
{
    uint8_t t;
    for(t=0;t<len;t++)
    {
        usart_data_transmit(UART4,buf[t]);
        while (RESET == usart_flag_get(UART4, USART_FLAG_TBE));
    }
    while(usart_flag_get(UART4, USART_FLAG_TC) == RESET);

}

void Uart4_Send_String(char *send_pt)
{
    UART4_SendArray((uint8_t *)send_pt,strlen(send_pt));
}