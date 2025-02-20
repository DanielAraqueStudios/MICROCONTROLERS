

#include <stdio.h>
#include <stm32f4xx.h>


//fx to set up ch A output

void seta(void){

    RCC->AHB1ENR |= (1<<0);


    // setting up pins C USING CH 15  

		GPIOA->MODER |= (1<<0)  ;
    GPIOA->OTYPER |= (0<<0)  ; //output push-pull
    GPIOA->OSPEEDR |= (1<<0);
    GPIOA->PUPDR |= (1<<1)  ;

}



int main(void){

  while(1){

    seta();
    GPIOA->ODR = 0x50;

  }	
				
}
	

	

	
