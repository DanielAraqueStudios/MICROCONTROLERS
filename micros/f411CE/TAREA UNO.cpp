
#include <stdio.h>
#include <stm32f4xx.h>

int cont=0;

int main(void){
	
	RCC->AHB1ENR |= (1<<0);
	GPIOA->MODER |= (1<<0) | (1<<1); //are thos two as an output if moder don 1 on decimla base is as an output?
    GPIOA->OTYPER |= (1<<0); //output push-pull
    GPIOA->OSPEEDR |= (0<<0) | (0<<1); //low  speed
    GPIOA->PUPDR |= (2<<0) | (2<<1); //pull-up

    while(true){
        for (cont=0;cont<100000;cont++)
			{
             GPIOA->ODR |= //turn on the USER led;
        }
				
        for (cont=0;cont<100000;cont++){
             GPIOA->ODR |= (1<<0) | (1<<1); 
        }
       
    }


	
	
}