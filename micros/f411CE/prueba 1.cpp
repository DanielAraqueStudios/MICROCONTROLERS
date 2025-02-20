
#include <stdio.h>
#include <stm32f4xx.h>

int cont=0;

int main(void){
	
	RCC->AHB1ENR |= (1<<0);
	GPIOA->MODER |= (1<<0); 
    GPIOA->OTYPER |= (1<<0); //output push-pull
    GPIOA->OSPEEDR |= (0<<0) ;
    GPIOA->PUPDR |= (2<<0) ; 

    while(true){
        GPIOA->ODR |= (1<<0)
;    }




	
	
}