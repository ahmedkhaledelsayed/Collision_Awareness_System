#include "stm32f10x.h"

void USART3_Init(void);
void GPIOB_Init(void);
void delay_ms(uint32_t ms);



int main(void) {
    // Initialize USART
	USART3_Init();

    // Initialize GPIO for push button
	GPIOB_Init();

    while (1) {
        // Check if the push button is pressed
        if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0) {

            // Send the word through USART
            USART_SendData(USART3, 'S');
            while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
            USART_SendData(USART3, 'T');
            while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
            USART_SendData(USART3, 'O');
            while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);
            USART_SendData(USART3, 'P');
            while (USART_GetFlagStatus(USART3, USART_FLAG_TXE) == RESET);

            // Add a delay to avoid multiple transmissions due to button bouncing
            delay_ms(40);
        }
    }
}

void USART3_Init(void) {
    USART_InitTypeDef USART_InitStructure;

    // Enable USART3 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    // Configure USART3
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
    USART_Init(USART3, &USART_InitStructure);

    // Enable USART1
    USART_Cmd(USART3, ENABLE);
}

void GPIOB_Init(void) {

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    // Configure USART3 TX (PB10) as alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);


    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}


void delay_ms(uint32_t ms) {
    // Delay function using SysTick
    uint32_t ticks = ms * (SystemCoreClock / 1000);
    SysTick->LOAD = ticks - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_ENABLE_Msk;

    while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));

    SysTick->CTRL = 0; // Disable SysTick
}
