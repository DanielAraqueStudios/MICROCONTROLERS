
//matix 8x8 test 1

#include <stdio.h>
#include <stm32f7xx.h>





int t =1000000;


void GPIO_MATRIZ(){

    RCC->AHB1ENR |= (1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6); // Habilito el reloj para los puertos A, B, C, D, E y F
    
    //---------Para configurar el pin 5 de la GPIOA como salida 
    GPIOA->MODER |=(1<<0); 
    GPIOA->OTYPER |=(1<<0); 
    GPIOA->OSPEEDR |= (1<<0); 
    GPIOA->PUPDR |= (1<<1); 

    //---------Para configurar el pin 10 de la GPIOB como salida
    GPIOB->MODER |= (1<<0)|(1<<4)|(1<<12)|(1<<22)|(1<<20);  
    GPIOB->OTYPER |= (1<<0);
    GPIOB->OSPEEDR |= (1<<0)|(1<<4)|(1<<12)|(1<<22);
    GPIOB->PUPDR |=  (1<<1)|(1<<5)|(1<<13)|(1<<23)|(1<<21);

    //---------Para configurar el pin 15 de la GPIOC como salida
    GPIOC->MODER |= (1<<4);
    GPIOC->OTYPER |= (1<<0);
    GPIOC->OSPEEDR |= (1<<4);
    GPIOC->PUPDR |= (1<<5);

    //---------Para configurar el pin 0 de la GPIOD como salida
    GPIOD->MODER |=(1<<26)|(1<<24)|(1<<22);
    GPIOD->OTYPER |= (1<<0);
    GPIOD->OSPEEDR |=(1<<26)|(1<<24)|(1<<22);
    GPIOD->PUPDR |= (1<<27)|(1<<25)|(1<<23);

    //---------Para configurar el pin 0 de la GPIOE como salida
    GPIOE->MODER |= (1<<0)|(1<<28)|(1<<30)|(1<<24)|(1<<4)|(1<<26); // Añadir configuración para E13
    GPIOE->OTYPER |= (1<<0);
    GPIOE->OSPEEDR |= (1<<0)|(1<<28)|(1<<30);
    GPIOE->PUPDR |= (1<<1)|(1<<29)|(1<<31)|(1<<5)|(1<<25);

    //---------Para configurar el pin 0 de la GPIOF como salida
    GPIOF->MODER |=(1<<8);
    GPIOF->OTYPER |= (1<<0);
    GPIOF->OSPEEDR |=(1<<8);
    GPIOF->PUPDR|= (1<<9);

    //---------Para configurar el pin 13 de la GPIOC como entrada
    GPIOC->MODER &= ~(1<<26); // Configurar C13 como entrada
    GPIOC->PUPDR |= (1<<27);  // Habilitar pull-up para C13




        //CONFIGURE LEDS OUTPUT

        // B10 B12 B13 B14 B15 A8 A9 A10 A11 A12 A15 B3 B4 B5 B6 B7 B9 C15 C14
    }



int main() {        
    
        while(1) {
            
            GPIO_MATRIZ();

            if (GPIOC->IDR & (1<<13)) {
			

					  GPIOE->ODR = (1<<2);
//            GPIOA->ODR = (1<<0);
//            GPIOB->ODR = (1<<0);
//            GPIOE->ODR = (1<<0);
//            GPIOE->ODR = (1<<14);
//            GPIOE->ODR = (1<<15);
//            GPIOB->ODR = (1<<10);
//						GPIOB->ODR = (1<<11);

//						GPIOE->ODR=(1<<9);
//					GPIOC->ODR=(0<<2);
//					GPIOE->ODR=(0<<11);
//					GPIOE->ODR=(0<<13);
//					GPIOB->ODR=(0<<2);
//					GPIOD->ODR=(0<<13);
//					GPIOD->ODR=(0<<12);
//					GPIOD->ODR=(0<<11);




        
        }


                
            
}
    
