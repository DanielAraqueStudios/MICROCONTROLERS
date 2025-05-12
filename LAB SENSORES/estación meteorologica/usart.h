#include <stm32f767xx.h>
#include <string.h>
#include <stdio.h>

char uart_buffer[50];   // Buffer increased size for multiple variables

void UART3_Init(void) {
    // Enable clocks
    RCC->AHB1ENR |= (1<<3);  // GPIOD clock
    
    // Configure UART pins
    GPIOD->MODER &= ~((0b11<<18)|(0b11<<16)); 
    GPIOD->MODER |= (1<<19)|(1<<17); // PD9=RX y PD8=TX alternant function
    GPIOD->AFR[1] &= ~((0b1111<<4)|(0b1111<<0));
    GPIOD->AFR[1] |= (0b111<<4)|(0b111<<0); // AF7 for USART3

    // Configure UART
    RCC->APB1ENR |= (1<<18); // USART3 clock
    USART3->BRR = 0x683;     // 9600 baud rate at 16MHz
    USART3->CR1 |= ((1<<5)|(0b11<<2)|(1<<0)); // Enable RX interrupt, TX, RX and UART
}

void UART3_SendChar(char c) {
    while(((USART3->ISR & 0x80) >> 7) == 0){} 
    USART3->TDR = c;
}

void UART3_SendString(const char* str) {
    while(*str) UART3_SendChar(*str++);
}

void Send_Variables(float var1, float var2, float var3, float var4) {
    sprintf(uart_buffer, "%.2f,%.2f,%.2f,%.2f\r\n", var1, var2, var3, var4);
    UART3_SendString(uart_buffer);
}

int main(void) {
    UART3_Init();
    
    float temperatura = 25.5;
    float humedad = 60.2;
    float presion = 1013.25;
    float altitud = 2600.0;
    
    while(1) {
        Send_Variables(temperatura, humedad, presion, altitud);
        // Delay or other processing as needed
    }
}

extern "C" void USART3_IRQHandler(void) {
    if ((USART3->ISR & USART_ISR_RXNE) != 0) {
        char c = USART3->RDR;
        // Process received data if needed
    }
}