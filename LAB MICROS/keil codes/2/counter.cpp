

#include <stdio.h>
#include <stm32f4xx.h>



int main(void){
	
		RCC->AHB1ENR |= (1<<0) | (1<<2); 
		
		GPIOA->MODER |= (1<<0) | (1<<2) | (1<<4)  | ;
    GPIOA->OTYPER |= (0<<0)  ; //output push-pull
    GPIOA->OSPEEDR |= (1<<0) | (1<<2) | (1<<4) | (1<<6);
    GPIOA->PUPDR |= (2<<0) | (1<<2) | (1<<5) | ( ; 
	
		GPIOC->MODER |= (1<<0);
		GPIOC->OTYPER |= 

	
	while(true){
		
		for (int i=0 ; i<100000 ; i++){
            GPIOA->ODR = (1<<0);
        }

        for (int i=0 ; i<100000 ; i++){
            GPIOA->ODR = (1<<1);
        }

        for (int i=0 ; i<100000 ; i++){
            GPIOA->ODR = (1<<2);
        }
		
	    
	   	
	}
	







	
	
}




