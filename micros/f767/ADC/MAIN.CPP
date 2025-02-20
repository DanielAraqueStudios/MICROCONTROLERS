
//operation 'or' (|) for set bit and operation 'and' (&) for clear bit

#include <stdio.h>
#include "stm32f7xx.h"
#include <string.h>


float value=0;

void PORTS(){

    RCC->AHB1ENR |= (1<<0)|(1<<1)|(1<<2)|(1<<3); //enable clock for port A,B,C ,D
    RCC->APB2ENR|=(1<<8); //ENABLE ADC1 CLOCK

    // A PORTS AS ADC

    /*MODER 0 FOR 8 BITS ,4 MHZ, 112 CYCLES , ALLINGMENT RIGHT 

    0,3-  1V----->TURN ON BLUE LED
    1,3-  2V----->TURN ON GREEN LED
    2-    3V----->TURN ON RED LED

    */

    GPIOA->MODER|=(3<<0); //PA0 as analog mode
    GPIOA->PUPDR|=(1<<1); //PA0 as pull up mode

    ADC->CCR|=(1<<16); //F= 4MHZ
    ADC1->CR1|=(1<<25);//8 bit resolution
    ADC1->CR2|=(1<<0)|(1<<30)|(0<<11)|(1<<10);
    ADC1->SMPR2|=(0b101<<0); //112 cycles
    ADC1->SQR3|=(0<<0); //channel 0


    //D PORTS AS OUTPUT
    GPIOD->MODER|=(1<<14)|(1<<12)|(1<<10); //PD7,PD6,PD5 as output mode
    GPIOD->OTYPER=0; //output push pull
    GPIOD->OSPEEDR|=(3<<14)|(3<<12)|(3<<10); //high speed
    GPIOD->PUPDR=0; //no pull up no pull down

}




int main(){


    while(1){

        PORTS();

        ADC1->CR2|=(1<<0); //start conversion
        while((ADC1->SR & (1<<1))==0){
            ADC1->SR =0; //clear flag
            value=ADC1->DR; //read value

        if(value<85){
            GPIOD->ODR=(1<<7); //turn on blue led
        }
        else if(value>=85 && value<170){
            GPIOD->ODR=(1<<6); //turn on green led
        }
        else{
            GPIOD->ODR=(1<<5); //turn on red led
        }          


        } //wait for conversion
        
    }


}