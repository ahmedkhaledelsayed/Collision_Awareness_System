#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

void UART_Init(void);
void GPIOB_Init(void);
void TIM4_PWM_Init(void);
void delay_ms(uint32_t ms);

void CAR_OFF(void);
void CAR_ON(void);
void CAR_Adjust_Right(void);
void CAR_Adjust_Left(void);

int main (void) {

	UART_Init();
	GPIOB_Init();
	TIM4_PWM_Init();

    uint16_t receivedData = -1;

    while (1)
    {
        // Check if USART1 received data
        if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
        {
            // Read the received data
            receivedData = USART_ReceiveData(USART1);

            if (receivedData == '0')
            {
                CAR_OFF(); // Car Off
            }
            else if (receivedData == '1')
            {
            	CAR_ON();   // Car ON and Forward
            }
            else if (receivedData == '2')
            {
            	CAR_Adjust_Right(); // Adjust Right
            }
            else if (receivedData == '3')
            {
            	CAR_Adjust_Left();   // Adjust Left
            }
        }
    }

    return 0;
}




void CAR_OFF(void){
    GPIO_ResetBits(GPIOA, GPIO_Pin_0);
    GPIO_ResetBits(GPIOA, GPIO_Pin_1);
    TIM4->CCR1 = 0;
    TIM4->CCR2 = 0;

}

void CAR_ON(void){
    TIM4->CCR1 = 900;
    TIM4->CCR2 = 550;
    GPIO_SetBits(GPIOA, GPIO_Pin_0);
    GPIO_SetBits(GPIOA, GPIO_Pin_1);
}

void CAR_Adjust_Right(void){
    TIM4->CCR2 = 0;
    TIM4->CCR1 = 1000;
    GPIO_SetBits(GPIOA, GPIO_Pin_0);
    GPIO_SetBits(GPIOA, GPIO_Pin_1);
}
void CAR_Adjust_Left(void){
    TIM4->CCR2 = 1000;
    TIM4->CCR1 = 0;
    GPIO_SetBits(GPIOA, GPIO_Pin_0);
    GPIO_SetBits(GPIOA, GPIO_Pin_1);
}


void UART_Init(void){
    // Enable the USART1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    USART_InitTypeDef USART_InitStructure;

    // USART1 configuration
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);

    // Enable USART1
    USART_Cmd(USART1, ENABLE);
}

void GPIOB_Init(void){

    // Enable the GPIOA and AFIO clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    // Configure the GPIO pin for the LED
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // LED pin
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; // LED pin
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure the USART1 RX pin (PA10) as input floating
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);



    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;
    // Configure GPIO pin for PWM output
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);


}

void TIM4_PWM_Init(void){
    // Enable CLK and Configure TIM4 for PWM
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    TIM_TimeBaseInitStruct.TIM_Prescaler = 72 - 1; // Assuming a 72 MHz clock, adjust as needed
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStruct.TIM_Period = 1000 - 1; // Set the PWM period for a 1 kHz frequency
    TIM_TimeBaseInitStruct.TIM_ClockDivision = 0;
    TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStruct);

    // Configure PWM channel 2
    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM4, &TIM_OCInitStruct);
    TIM_OC2Init(TIM4, &TIM_OCInitStruct);

    // Enable TIM4
    TIM_Cmd(TIM4, ENABLE);
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
