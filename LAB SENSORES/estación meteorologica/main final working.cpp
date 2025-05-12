//Ejemplo TIMER
//Fabián Barrera Prieto
//Universidad ECCI
//STM32F767ZIT6U
//operation 'or' (|) for set bit and operation 'and' (&) for clear bit

#include <stdio.h>
#include "stm32f7xx.h"
#include <string.h>

uint8_t flag = 0, i, cont = 0;
unsigned char d;
char text[30];  // Aumentado a 30 caracteres
uint16_t digital2;
uint16_t digital1;
float voltaje1;  // Añadida declaración
float voltaje2;
uint32_t a, b, c;

void SysTick_Wait(uint32_t n){
    SysTick->LOAD = n - 1;
    SysTick->VAL = 0; 
    while (((SysTick->CTRL & 0x00010000) >> 16) == 0); 
}

void SysTick_ms(uint32_t x){
    for (uint32_t i = 0; i < x; i++){
        SysTick_Wait(16000); 
    }
}

extern "C"{
    void EXTI15_10_IRQHandler(void){
        EXTI->PR |= 1; 
        if(((GPIOC->IDR & (1<<13)) >> 13) == 1){
            flag = 1;
        }
    }

    void TIM3_IRQHandler(void){ // Interruption Timer 
        TIM3->SR &= ~(1<<0); // Clear the flag on TIM3 
        a = (~(GPIOB->ODR&(1<<7)));
        b = (GPIOB->ODR|(1<<7));
        c = a&b;
        GPIOB->ODR = c;
        cont += 1;
        if(cont == 50){
            cont = 0;
            flag = 1;
        }
    }


    void USART3_IRQHandler(void){ 
        if(((USART3->ISR & 0x20) >> 5) == 1){
            d = USART3->RDR;
            if(d == 'a'){
                flag = 1;
            }
        }
    }
}

int main(){
    //GPIOs
    RCC->AHB1ENR |= ((1<<1)|(1<<2)); 

    GPIOB->MODER &= ~((0b11<<0)|(0b11<<14));
    GPIOB->MODER |= ((1<<0)|(1<<14)); 
    GPIOC->MODER &= ~(0b11<<26);

    GPIOB->OTYPER &= ~((1<<0)|(1<<7));
    GPIOB->OSPEEDR |= (((1<<1)|(1<<0)|(1<<15)|(1<<14)));
    GPIOC->OSPEEDR |= ((1<<27)|(1<<26));
    GPIOB->PUPDR &= ~((0b11<<0)|(0b11<<14));
    GPIOC->PUPDR &= ~(0b11<<26);
    GPIOC->PUPDR |= (1<<27);

    //Systick
    SysTick->LOAD = 0x00FFFFFF; 
    SysTick->CTRL |= (0b101);

    //Interrupt
    RCC->APB2ENR |= (1<<14); 
    SYSCFG->EXTICR[3] &= ~(0b1111<<4); 
    SYSCFG->EXTICR[3] |= (1<<5); 
    EXTI->IMR |= (1<<13); 
    EXTI->RTSR |= (1<<13);
    NVIC_EnableIRQ(EXTI15_10_IRQn); 
            
    //UART
    RCC->AHB1ENR |= (1<<3); 
    GPIOD->MODER &= ~((0b11<<18)|(0b11<<16)); 
    GPIOD->MODER |= (1<<19)|(1<<17); 
    GPIOD->AFR[1] &= ~((0b1111<<4)|(0b1111<<0));
    GPIOD->AFR[1] |= (0b111<<4)|(0b111<<0); 
    RCC->APB1ENR |= (1<<18); 
    USART3->BRR = 0x683; 
    USART3->CR1 |= ((1<<5)|(0b11<<2)); 
    NVIC_EnableIRQ(USART3_IRQn); 

    //ADC2


    GPIOC->MODER |= (0b11<<0); 
    RCC->APB2ENR |= (1<<9); 
    ADC2->CR2 |= ((1<<10)|(1<<0)); 
    ADC2->CR1 &= ~(0b11<<24); 
    
    ADC2->SMPR1 |= (0b111<<0); 
    ADC2->SQR3 &= ~(0b11111<<0); 
    ADC2->SQR3 |= (0b1010<<0); //ch10
    
    //ADC1
    RCC->AHB1ENR |= (1<<0);         // Habilita GPIOA
    GPIOA->MODER |= (0b11<<0);      // PA0 como analógico
    GPIOA->PUPDR|=(1<<1); //PA0 as pull down mode
    
    RCC->APB2ENR |= (1<<8);         // Habilita reloj ADC1
    ADC->CCR |= (0b11<<16);         // F=32MHz
    ADC1->CR2 |= (1<<0);            // Enciende ADC1
    ADC1->CR2 |= (1<<10);           // EOC después de cada conversión
    ADC1->CR1 &= ~(0b11<<24);       // Resolución 12 bits
    ADC1->SMPR2 |= (0b111<<0);      // Máximo tiempo muestreo canal 0
    ADC1->SQR3 &= ~(0b11111<<0); 
    ADC1->SQR3 |= (0b00000<<0); //ch10

    //TIMER
    RCC->APB1ENR |= (1<<1); //Enable the TIMER3 clock 
    TIM3->PSC = 24; // Prescale factor 25 for 100ms of time
    TIM3->ARR = 63999; // Maximum count value
    TIM3->DIER |= (1<<0); //Enable IRQ on update		 
    TIM3->CR1 |= (1<<0); // Enable Counting
    NVIC_EnableIRQ(TIM3_IRQn); // Enable IRQ for TIM3 in NVIC	

    //UART
    USART3->CR1 |= (1<<0);

    while(1){
        GPIOB->ODR |= 1<<0; 
        SysTick_ms(500);
        GPIOB->ODR &= ~(1<<0);
        SysTick_ms(500);

        // ADC2 (PC0)
        ADC2->CR2 |= (1<<30); 
        while(((ADC2->SR & (1<<1)) >> 1) == 0){} 
        ADC2->SR &= ~(1<<1);
        digital2 = ADC2->DR;
        voltaje2 = (float)digital2*(3.3/4096.0);  // Corregido a 4096
        sprintf(text,"ADC2: %.2fV ", voltaje2);
        for(i=0; i<strlen(text); i++){
            USART3->TDR = text[i]; 
            while(((USART3->ISR & 0x80) >> 7) == 0){}
        }

        // ADC1 (PA0)
        ADC1->CR2 |= (1<<30);
        while(((ADC1->SR & (1<<1)) >> 1) == 0){}
        ADC1->SR &= ~(1<<1);
        digital1 = ADC1->DR;
        voltaje1 = (float)digital1*(3.3/4096.0);
        sprintf(text,"ADC1: %.2fV\r\n", voltaje1);
        for(i=0; i<strlen(text); i++){
            USART3->TDR = text[i]; 
            while(((USART3->ISR & 0x80) >> 7) == 0){}
        }
        
        SysTick_ms(100);  // Delay entre lecturas
    }
}