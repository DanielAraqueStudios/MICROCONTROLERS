#include <stdio.h>
#include "stm32f7xx.h"
#include <string.h>




void PORTS() {
    RCC->AHB1ENR |= (0b1111111111<<0); //enable clock for port A-J


    */
    GPIOA->MODER|=(1<<1)|(1<<0); //PA0 as analog mode
    GPIOA->PUPDR|=(1<<1); //PA0 as pull down mode

    ADC->CCR|=(1<<16); //F= 4MHZ
    ADC1->CR1|=(1<<25);//8 bit resolution
    ADC1->CR2|=(1<<0)|(1<<10); // turn on ADC & set end of conversion
    ADC1->CR2 &= ~(1<<11); //right alignment
    ADC1->SMPR2|=(0b101<<0); //112 cycles
    ADC1->SQR3 &= ~(0b11111<<0); //clear channel
    ADC1->SQR3|=(0b00000<<0); //channel 0

    //D PORTS AS OUTPUT
		GPIOD->MODER &= ~((1<<14)|(1<<12)|(1<<10));
    GPIOD->MODER|=(1<<14)|(1<<12)|(1<<10); //PD7,PD6,PD5 as output mode
    GPIOD->OTYPER=(1<<0); //output push pull
    GPIOD->OSPEEDR|=(3<<14)|(3<<12)|(3<<10); //high speed
    GPIOD->PUPDR|=(1<<15)|(1<<13)|(1<<11); //no pull up no pull down
}

int main() {
    PORTS();

    while(1) {
        ADC1->CR2|=(1<<30); //start conversion
        while(((ADC1->SR & (1<<1)) >> 1) == 0); //wait for conversion to complete (EOC flag)
        
        voltage = ADC1->DR; //read value
        volts = (voltage * 3.3) / 255; //convert to volts

				if(voltage > 230) {
            GPIOD->ODR = 0; //turn on blue led
				}
        else if(voltage > 165 & voltage < 230) {
            GPIOD->ODR = (1<<7); //turn on blue led
        } else if(voltage > 80 & voltage < 165) {
            GPIOD->ODR = (1<<6); //turn on green led
        } else  if (voltage > 0 & voltage < 80){
            GPIOD->ODR = (1<<5); //turn on red led
        }
    }
}