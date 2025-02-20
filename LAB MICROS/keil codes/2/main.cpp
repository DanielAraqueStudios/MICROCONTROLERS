

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
		
				for (int i=0; i<1000000; i++){

        GPIOC-> ODR | = (1<<15); 
        } //delay

        for (int i=0; i<1000000; i++){

          GPIOC->ORD & = ~ (1<<15);//GPIOC->ORD = GPIOC->ORD & ~(1<<15)

        }
        
				
	}
	
}
	
