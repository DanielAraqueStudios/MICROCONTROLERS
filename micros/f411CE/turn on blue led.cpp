

#include <stdio.h>
#include <stm32f4xx.h>



int main(void){
	
	RCC->AHB1ENR |= (1<<2); 


    // setting up pins C USING CH 15  

	GPIOC->MODER |= (1<<26)  ;
    GPIOC->OTYPER |= (0<<0)  ; //output push-pull
    GPIOC->OSPEEDR |= (1<<26);
    GPIOC->PUPDR |= (1<<26)  ;
	
	

	
	
	


	
	while(true){
		
	
        GPIOC->ODR = (1<<15);	
	}
	
}
	
