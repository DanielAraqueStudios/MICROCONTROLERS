//matix 8x8 test 1

#include <stdio.h>
#include <stm32f7xx.h>

//GLOBAL VARIABLES

 int time=0;
volatile int pin13=0;

void GPIO_MATRIZ(){
    // del A AL G
    RCC->AHB1ENR |= (1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7); // habilitar reloj para los puertos A,B,C,D,E,F,G

    // Configurar el pin C8 como salida
    GPIOC->MODER &= ~(0b11 << 16); // Limpiar bits
    GPIOC->MODER |= (1 << 16);  // Configurar PC8 como salida (MODER8[1:0] = 01)
    GPIOC->OTYPER &= ~(1 << 8); // Configurar PC8 como push-pull
    GPIOC->OSPEEDR |= (1 << 16); // Configurar PC8 como alta velocidad
    GPIOC->PUPDR &= ~(0b11 << 16);  // Limpiar bits
    GPIOC->PUPDR |= (1 << 17); // Configurar PC8 como pull-DOWN

    // Configurar el pin C9 como salida
    GPIOC->MODER &= ~(0b11 << 18); // Limpiar bits
    GPIOC->MODER |= (1 << 18);  // Configurar PC9 como salida (MODER9[1:0] = 01)
    GPIOC->OTYPER &= ~(1 << 9); // Configurar PC9 como push-pull
    GPIOC->OSPEEDR |= (1 << 18); // Configurar PC9 como alta velocidad
    GPIOC->PUPDR &= ~(0b11 << 18);  // Limpiar bits
    GPIOC->PUPDR |= (1 << 19); // Configurar PC9 como pull-DOWN

    // Configurar el pin C11 como salida
    GPIOC->MODER &= ~(0b11 << 22); // Limpiar bits
    GPIOC->MODER |= (1 << 22);  // Configurar PC11 como salida (MODER11[1:0] = 01)
    GPIOC->OTYPER &= ~(1 << 11); // Configurar PC11 como push-pull
    GPIOC->OSPEEDR |= (1 << 22); // Configurar PC11 como alta velocidad
    GPIOC->PUPDR &= ~(0b11 << 22);  // Limpiar bits
    GPIOC->PUPDR |= (1 << 23); // Configurar PC11 como pull-DOWN

    // Configurar el pin C12 como salida
    GPIOC->MODER &= ~(0b11 << 24); // Limpiar bits
    GPIOC->MODER |= (1 << 24);  // Configurar PC12 como salida (MODER12[1:0] = 01)
    GPIOC->OTYPER &= ~(1 << 12); // Configurar PC12 como push-pull
    GPIOC->OSPEEDR |= (1 << 24); // Configurar PC12 como alta velocidad
    GPIOC->PUPDR &= ~(0b11 << 24);  // Limpiar bits
    GPIOC->PUPDR |= (1 << 25); // Configurar PC12 como pull-DOWN

    // Configurar el pin D2 como salida
    GPIOD->MODER &= ~(0b11 << 4); // Limpiar bits
    GPIOD->MODER |= (1 << 4);  // Configurar PD2 como salida (MODER2[1:0] = 01)
    GPIOD->OTYPER &= ~(1 << 2); // Configurar PD2 como push-pull
    GPIOD->OSPEEDR |= (1 << 4); // Configurar PD2 como alta velocidad
    GPIOD->PUPDR &= ~(0b11 << 4);  // Limpiar bits
    GPIOD->PUPDR |= (1 << 5); // Configurar PD2 como pull-DOWN

    // Configurar el pin G2 como salida
    GPIOG->MODER &= ~(0b11 << 4); // Limpiar bits
    GPIOG->MODER |= (1 << 4);  // Configurar PG2 como salida (MODER2[1:0] = 01)
    GPIOG->OTYPER &= ~(1 << 2); // Configurar PG2 como push-pull
    GPIOG->OSPEEDR |= (1 << 4); // Configurar PG2 como alta velocidad
    GPIOG->PUPDR &= ~(0b11 << 4);  // Limpiar bits
    GPIOG->PUPDR |= (1 << 5); // Configurar PG2 como pull-DOWN

    // Configurar el pin C10 como salida
    GPIOC->MODER &= ~(0b11 << 20); // Limpiar bits
    GPIOC->MODER |= (1 << 20);  // Configurar PC10 como salida (MODER10[1:0] = 01)
    GPIOC->OTYPER &= ~(1 << 10); // Configurar PC10 como push-pull
    GPIOC->OSPEEDR |= (1 << 20); // Configurar PC10 como alta velocidad
    GPIOC->PUPDR &= ~(0b11 << 20);  // Limpiar bits
    GPIOC->PUPDR |= (1 << 21); // Configurar PC10 como pull-DOWN

    // Configurar el pin G3 como salida
    GPIOG->MODER &= ~(0b11 << 6); // Limpiar bits
    GPIOG->MODER |= (1 << 6);  // Configurar PG3 como salida (MODER3[1:0] = 01)
    GPIOG->OTYPER &= ~(1 << 3); // Configurar PG3 como push-pull
    GPIOG->OSPEEDR |= (1 << 6); // Configurar PG3 como alta velocidad
    GPIOG->PUPDR &= ~(0b11 << 6);  // Limpiar bits
    GPIOG->PUPDR |= (1 << 7); // Configurar PG3 como pull-DOWN

    // Configurar el pin D7 como salida
    GPIOD->MODER &= ~(0b11 << 14); // Limpiar bits
    GPIOD->MODER |= (1 << 14);  // Configurar PD7 como salida (MODER7[1:0] = 01)
    GPIOD->OTYPER &= ~(1 << 7); // Configurar PD7 como push-pull
    GPIOD->OSPEEDR |= (1 << 14); // Configurar PD7 como alta velocidad
    GPIOD->PUPDR &= ~(0b11 << 14);  // Limpiar bits
    GPIOD->PUPDR |= (1 << 15); // Configurar PD7 como pull-DOWN

    // Configurar el pin D6 como salida
    GPIOD->MODER &= ~(0b11 << 12); // Limpiar bits
    GPIOD->MODER |= (1 << 12);  // Configurar PD6 como salida (MODER6[1:0] = 01)
    GPIOD->OTYPER &= ~(1 << 6); // Configurar PD6 como push-pull
    GPIOD->OSPEEDR |= (1 << 12); // Configurar PD6 como alta velocidad
    GPIOD->PUPDR &= ~(0b11 << 12);  // Limpiar bits
    GPIOD->PUPDR |= (1 << 13); // Configurar PD6 como pull-DOWN

    // Configurar el pin D5 como salida
    GPIOD->MODER &= ~(0b11 << 10); // Limpiar bits
    GPIOD->MODER |= (1 << 10);  // Configurar PD5 como salida (MODER5[1:0] = 01)
    GPIOD->OTYPER &= ~(1 << 5); // Configurar PD5 como push-pull
    GPIOD->OSPEEDR |= (1 << 10); // Configurar PD5 como alta velocidad
    GPIOD->PUPDR &= ~(0b11 << 10);  // Limpiar bits
    GPIOD->PUPDR |= (1 << 11); // Configurar PD5 como pull-DOWN

    // Configurar el pin D4 como salida
    GPIOD->MODER &= ~(0b11 << 8); // Limpiar bits
    GPIOD->MODER |= (1 << 8);  // Configurar PD4 como salida (MODER4[1:0] = 01)
    GPIOD->OTYPER &= ~(1 << 4); // Configurar PD4 como push-pull
    GPIOD->OSPEEDR |= (1 << 8); // Configurar PD4 como alta velocidad
    GPIOD->PUPDR &= ~(0b11 << 8);  // Limpiar bits
    GPIOD->PUPDR |= (1 << 9); // Configurar PD4 como pull-DOWN

    // Configurar el pin D3 como salida
    GPIOD->MODER &= ~(0b11 << 6); // Limpiar bits
    GPIOD->MODER |= (1 << 6);  // Configurar PD3 como salida (MODER3[1:0] = 01)
    GPIOD->OTYPER &= ~(1 << 3); // Configurar PD3 como push-pull
    GPIOD->OSPEEDR |= (1 << 6); // Configurar PD3 como alta velocidad
    GPIOD->PUPDR &= ~(0b11 << 6);  // Limpiar bits
    GPIOD->PUPDR |= (1 << 7); // Configurar PD3 como pull-DOWN

    // Configurar el pin E2 como salida
    GPIOE->MODER &= ~(0b11 << 4); // Limpiar bits
    GPIOE->MODER |= (1 << 4);  // Configurar PE2 como salida (MODER2[1:0] = 01)
    GPIOE->OTYPER &= ~(1 << 2); // Configurar PE2 como push-pull
    GPIOE->OSPEEDR |= (1 << 4); // Configurar PE2 como alta velocidad
    GPIOE->PUPDR &= ~(0b11 << 4);  // Limpiar bits
    GPIOE->PUPDR |= (1 << 5); // Configurar PE2 como pull-DOWN

    // Configurar el pin E4 como salida
    GPIOE->MODER &= ~(0b11 << 8); // Limpiar bits
    GPIOE->MODER |= (1 << 8);  // Configurar PE4 como salida (MODER4[1:0] = 01)
    GPIOE->OTYPER &= ~(1 << 4); // Configurar PE4 como push-pull
    GPIOE->OSPEEDR |= (1 << 8); // Configurar PE4 como alta velocidad
    GPIOE->PUPDR &= ~(0b11 << 8);  // Limpiar bits
    GPIOE->PUPDR |= (1 << 9); // Configurar PE4 como pull-DOWN

    // Configurar el pin E5 como salida
    GPIOE->MODER &= ~(0b11 << 10); // Limpiar bits
    GPIOE->MODER |= (1 << 10);  // Configurar PE5 como salida (MODER5[1:0] = 01)
    GPIOE->OTYPER &= ~(1 << 5); // Configurar PE5 como push-pull
    GPIOE->OSPEEDR |= (1 << 10); // Configurar PE5 como alta velocidad
    GPIOE->PUPDR &= ~(0b11 << 10);  // Limpiar bits
    GPIOE->PUPDR |= (1 << 11); // Configurar PE5 como pull-DOWN

    // Configurar el pin C13 como ENTRADA
    GPIOC->MODER &= ~(0b11 << 26); // Limpiar bits
    GPIOC->PUPDR &= ~(0b11 << 26);  // Limpiar bits
    GPIOC->PUPDR |= (1 << 27); // Configurar PC13 como pull-DOWN



    //display gpios
    // Configurar el pin F7 como salida
    GPIOF->MODER &= ~(0b11 << 14); // Limpiar bits
    GPIOF->MODER |= (1 << 14);  // Configurar PF7 como
    GPIOF->OTYPER &= ~(1 << 7); // Configurar PF7 como push-pull
    GPIOF->OSPEEDR |= (1 << 14); // Configurar PF7 como alta velocidad
    GPIOF->PUPDR &= ~(0b11 << 14);  // Limpiar bits
    GPIOF->PUPDR |= (1 << 15); // Configurar PF7 como pull-DOWN

    // Configurar el pin F9 como salida
    GPIOF->MODER &= ~(0b11 << 18); // Limpiar bits
    GPIOF->MODER |= (1 << 18);  // Configurar PF9 como salida
    GPIOF->OTYPER &= ~(1 << 9); // Configurar PF9 como push-pull
    GPIOF->OSPEEDR |= (1 << 18); // Configurar PF9 como alta velocidad
    GPIOF->PUPDR &= ~(0b11 << 18);  // Limpiar bits
    GPIOF->PUPDR |= (1 << 19); // Configurar PF9 como pull-DOWN

    // Configurar el pin G1 como salida
    GPIOG->MODER &= ~(0b11 << 2); // Limpiar bits
    GPIOG->MODER |= (1 << 2);  // Configurar PG1 como salida
    GPIOG->OTYPER &= ~(1 << 1); // Configurar PG1 como push-pull
    GPIOG->OSPEEDR |= (1 << 2); // Configurar PG1 como alta velocidad
    GPIOG->PUPDR &= ~(0b11 << 2);  // Limpiar bits
    GPIOG->PUPDR |= (1 << 3); // Configurar PG1 como pull-DOWN

    // Configurar el pin G0 como salida
    GPIOG->MODER &= ~(0b11 << 0); // Limpiar bits
    GPIOG->MODER |= (1 << 0);  // Configurar PG0 como salida
    GPIOG->OTYPER &= ~(1 << 0); // Configurar PG0 como push-pull
    GPIOG->OSPEEDR |= (1 << 0); // Configurar PG0 como alta velocidad
    GPIOG->PUPDR &= ~(0b11 << 0);  // Limpiar bits
    GPIOG->PUPDR |= (1 << 1); // Configurar PG0 como pull-DOWN

    // Configurar el pin D1 como salida

    GPIOD->MODER &= ~(0b11 << 2); // Limpiar bits
    GPIOD->MODER |= (1 << 2);  // Configurar PD1 como salida
    GPIOD->OTYPER &= ~(1 << 1); // Configurar PD1 como push-pull
    GPIOD->OSPEEDR |= (1 << 2); // Configurar PD1 como alta velocidad
    GPIOD->PUPDR &= ~(0b11 << 2);  // Limpiar bits
    GPIOD->PUPDR |= (1 << 3); // Configurar PD1 como pull-DOWN

    // Configurar el pin D1 como salida

    GPIOD->MODER &= ~(0b11 << 0); // Limpiar bits
    GPIOD->MODER |= (1 << 0);  // Configurar PD0 como
    GPIOD->OTYPER &= ~(1 << 0); // Configurar PD0 como push-pull
    GPIOD->OSPEEDR |= (1 << 0); // Configurar PD0 como alta velocidad
    GPIOD->PUPDR &= ~(0b11 << 0);  // Limpiar bits
    GPIOD->PUPDR |= (1 << 1); // Configurar PD0 como pull-DOWN

    // Configurar el pin E3 como salida
    GPIOE->MODER &= ~(0b11 << 6); // Limpiar bits
    GPIOE->MODER |= (1 << 6);  // Configurar PE3 como salida
    GPIOE->OTYPER &= ~(1 << 3); // Configurar PE3 como push-pull
    GPIOE->OSPEEDR |= (1 << 6); // Configurar PE3 como alta velocidad
    GPIOE->PUPDR &= ~(0b11 << 6);  // Limpiar bits
    GPIOE->PUPDR |= (1 << 7); // Configurar PE3 como pull-DOWN

    // Configurar el pin E6 como salida

    GPIOE->MODER &= ~(0b11 << 12); // Limpiar bits
    GPIOE->MODER |= (1 << 12);  // Configurar PE6 como salida
    GPIOE->OTYPER &= ~(1 << 6); // Configurar PE6 como push-pull
    GPIOE->OSPEEDR |= (1 << 12); // Configurar PE6 como
    GPIOE->PUPDR &= ~(0b11 << 12);  // Limpiar bits
    GPIOE->PUPDR |= (1 << 13); // Configurar PE6 como pull-DOWN



}

void turn_all(){

    GPIOC->ODR |= (1 << 8); // Encender PC8
    GPIOC->ODR |= (1 << 9); // Encender PC9
    GPIOC->ODR |= (1 << 10); // Encender PC10
    GPIOC->ODR |= (1 << 11); // Encender PC11
    GPIOC->ODR |= (1 << 12); // Encender PC12
    GPIOD->ODR |= (1 << 2); // Encender PD2
    GPIOG->ODR |= (1 << 2); // Encender PG2
    GPIOG->ODR |= (1 << 3); // Encender PG3
    GPIOD->ODR &= ~(1 << 7); // Apagar PD7
    GPIOD->ODR &= ~(1 << 6); // Apagar PD6
    GPIOD->ODR &= ~(1 << 5); // Apagar PD5
    GPIOD->ODR &= ~(1 << 4); // Apagar PD4
    GPIOD->ODR &= ~(1 << 3); // Apagar PD3
    GPIOE->ODR &= ~(1 << 2); // Apagar PE2
    GPIOE->ODR &= ~(1 << 4); // Apagar PE4
    GPIOE->ODR &= ~(1 << 5); // Apagar PE5
}

void mask(){

    for (int i=0;i<8;i++){

        if (i==0) {

            GPIOD->ODR |= (1 << 7)|(1<<6)|(1<<4)|(1<<3);    
            GPIOE->ODR |= (1 << 4)|(1<<5); // Apagar PE2
            GPIOC->ODR |= (1 << 8); // Encender PC8


					
			GPIOC->ODR &= ~(1 << 8); 
					
					

            GPIOD->ODR &= ~((1 << 7)|(1<<6)|(1<<4)|(1<<3));  
 
            GPIOE->ODR &= ~(1 << 4);// Apagar PE2
			GPIOE->ODR &= ~(1 << 5);
            
        
        }

        if (i==1) {

            GPIOD->ODR |= (1 << 7);
            GPIOE->ODR |= (1<<5); // Apagar PE2

            GPIOC->ODR |= (1 << 9);



            GPIOC->ODR &= ~(1 << 9);
            GPIOD->ODR &= ~(1 << 7);
            GPIOE->ODR &= ~(1 << 5); // Apagar PE2
        }
        if (i==2) {
            

            GPIOC->ODR|= (1 << 10);
            GPIOC->ODR &= ~(1 << 10);

        }
        if (i==3) {

            GPIOD->ODR|= (1 << 5);
            GPIOE->ODR|= (1 << 2);

            GPIOC->ODR|= (1 << 11);


            GPIOC->ODR&= ~(1 << 11);

            GPIOD->ODR&= ~(1 << 5);
            GPIOE->ODR&= ~(1 << 2); 
        }

        if (i==4) {
            GPIOC->ODR |= (1 << 12);

            GPIOE->ODR |= (1<<5); // Apagar PE2

            GPIOD->ODR |= (1 << 7);



            GPIOC->ODR &= ~(1 << 12);
            GPIOD->ODR &= ~(1 << 7);
            GPIOE->ODR &= ~(1 << 5); // Apagar PE2
        }

        if (i==5){

            GPIOD->ODR |= (1 << 7)|(1<<5)|(1<<4)|(1<<3);
            GPIOE->ODR |= (1 << 2)|(1<<5); 

            GPIOD->ODR |= (1<<2);


            GPIOD->ODR &= ~(1<<2);


            GPIOD->ODR &= ~((1 << 7)|(1<<5)|(1<<4)|(1<<3));
            GPIOE->ODR &= ~(1 << 2);// Apagar PE2
            GPIOE->ODR &= ~(1 << 5);

           
            
        }

        if (i==6) {

            GPIOD->ODR |= (1 << 6);
            GPIOE->ODR |= (1<<4); // Apagar PE2

            GPIOG->ODR |= (1 << 2);


            GPIOG->ODR &= ~(1 << 2);

            GPIOD->ODR &= ~(1 << 6);
            GPIOE->ODR &= ~(1<<4); 
        } 

        if(i==7){

            GPIOD->ODR |= (1<<5)|(1<<6)|(1<<4)|(1<<3);
            GPIOE->ODR |= (1<<2)|(1<<4);

            GPIOG->ODR |= (1 << 3);
            
            GPIOG->ODR &= ~(1 << 3);

            GPIOD->ODR &= ~((1<<5)|(1<<6)|(1<<4)|(1<<3));
            GPIOE->ODR &= ~((1<<2)|(1<<4));
        }

    } 
}

void face(){
    for (int i=0; i<8; i++){
        if (i==0) {
            GPIOD->ODR |= (1 << 6) | (1 << 7) ;
            GPIOE->ODR |= (1 << 4) | (1 << 5);
            GPIOC->ODR |= (1 << 8);
            
            GPIOC->ODR &= ~(1 << 8);
            GPIOD->ODR &= ~((1 << 6) |(1 << 7)) ;
            GPIOE->ODR &= ~((1 << 4) |(1 << 5));
        }

        if (i==1) {

            GPIOD->ODR |= (1 << 7)|(1<<5)|(1<<4)|(1<<3);
            GPIOE->ODR |= (1 << 2)|(1<<5);

            GPIOC->ODR |= (1 << 9);
            
            GPIOC->ODR &= ~(1 << 9);

            GPIOD->ODR &= ~((1 << 7)|(1<<5)|(1<<4)|(1<<3));
            GPIOE->ODR &= ~(1 << 2);
            GPIOE->ODR &= ~(1 << 5);
        }

        if (i==2) {
            GPIOD->ODR |= (1 << 6) | (1 << 4)|(1<<3);
            GPIOE->ODR |= (1 << 4);
            GPIOC->ODR |= (1 << 10);
            
            GPIOC->ODR &= ~(1 << 10);
            GPIOD->ODR &= ~((1 << 6) |(1 << 4)|(1 << 3));
            GPIOE->ODR &= ~(1 << 4);
        }

        if (i==3) {




            
            GPIOD->ODR |= (1 << 6) | (1 << 5) | (1 << 4)| (1 << 3);
            GPIOE->ODR |= (1 << 4) | (1 << 2);
            GPIOC->ODR |= (1 << 11);
            
            GPIOC->ODR &= ~(1 << 11);
            GPIOD->ODR &= ~((1 << 6) |(1 << 5) |(1 << 4)|(1 << 3));
            GPIOE->ODR &= ~(1 << 4) & ~(1 << 2);
            /*

            GPIOD->ODR |= (1 << 6)|(1<<4)|(1<<3);
            GPIOE->ODR |= (1 << 4);


            GPIOC->ODR |= (1 << 11);
            
            GPIOC->ODR &= ~(1 << 11);

            GPIOD->ODR &= ~(1 << 6)&(1<<4)&(1<<3);
            GPIOE->ODR &= ~(1 << 4);

            */
        }

        if (i==4) {

            GPIOD->ODR |= (1 << 6) ;
            GPIOE->ODR |= (1 << 4) ;
            GPIOC->ODR |= (1 << 12);
            
            GPIOC->ODR &= ~(1 << 12);
            GPIOD->ODR &= ~(1 << 6);
            GPIOE->ODR &= ~(1 << 4);
        }

        if (i==5) {
            GPIOD->ODR |= (1 << 6) |(1<<5);
            GPIOE->ODR |= (1 << 4) |(1<<2);
            GPIOD->ODR |= (1 << 2);
            
            GPIOD->ODR &= ~(1 << 2);
            GPIOD->ODR &= ~((1 << 6)|(1<<5));
            GPIOE->ODR &= ~((1 << 4)|(1<<2));
        }

        if (i==6) {
            GPIOD->ODR |= (1 << 7)|(1<<5)|(1<<4)|(1<<3);
            GPIOE->ODR |= (1 << 2)|(1<<5);

            GPIOG->ODR |= (1 << 2);
            
            GPIOG->ODR &= ~(1 << 2);

            GPIOD->ODR &= ~((1 << 7)|(1<<5)|(1<<4)|(1<<3));
            GPIOE->ODR &= ~(1 << 2);
            GPIOE->ODR &= ~(1 << 5);
        }

        if (i==7) {
            GPIOD->ODR |= (1 << 6) | (1 << 7) ;
            GPIOE->ODR |= (1 << 4) | (1 << 5);
            GPIOG->ODR |= (1 << 3);
            
            GPIOG->ODR &= ~(1 << 3);
            GPIOD->ODR &= ~((1 << 6) |(1 << 7)) ;
            GPIOE->ODR &= ~((1 << 4) |(1 << 5));
        }
    }
}


void letter(){
    for (int i=0; i<8; i++){
        if (i==0) { 



            GPIOC->ODR |= (1 << 8);  
            GPIOC->ODR &= ~(1 << 8);
           
        }
        
        if (i==1) {
            GPIOD->ODR |= (1 << 5) | (1 << 3)|(1<<4);
            GPIOE->ODR |= (1 << 2);

            GPIOC->ODR |= (1 << 9);
            GPIOC->ODR &= ~(1 << 9);

            GPIOD->ODR &= ~((1 << 5) |(1<<4));
            GPIOD->ODR &= ~(1 << 3);
            GPIOE->ODR &= ~(1 << 2);
        }
        
        if (i==2) {
            GPIOD->ODR &= ~(1 << 3);
            GPIOD->ODR |= (1 << 6) | (1 << 4)|(1<<3);
            GPIOE->ODR |= (1 << 4);
            GPIOC->ODR |= (1 << 10);
            GPIOC->ODR &= ~(1 << 10);
            GPIOE->ODR &= ~(1 << 4);
            GPIOD->ODR &= ~((1 << 6) |(1 << 4));
        }
        
        if (i==3) {
            GPIOD->ODR &= ~(1 << 3);
            GPIOD->ODR |= (1 << 5)|(1<<6);
            GPIOE->ODR |= (1 << 2)|(1<<4);
            GPIOC->ODR |= (1 << 11);
            GPIOC->ODR &= ~(1 << 11);
            GPIOD->ODR &= ~((1 << 5)|(1<<6));
            GPIOE->ODR &= ~((1 << 2)|(1<<4));
        }
        
        if (i==4) {
            GPIOD->ODR |= (1 << 5)|(1<<6);
            GPIOE->ODR |= (1 << 2)|(1<<4);
            GPIOC->ODR |= (1 << 12);
            GPIOC->ODR &= ~(1 << 12);
            GPIOD->ODR &= ~((1 << 5)|(1<<6));
            GPIOE->ODR &= ~((1 << 2)|(1<<4));
        }
        
        if (i==5) {
            GPIOD->ODR |= (1 << 5) | (1 << 3)|(1<<4)|(1<<6);
            GPIOE->ODR |= (1 << 2)|(1<<4);

            GPIOD->ODR |= (1 << 2);
            GPIOD->ODR &= ~(1 << 2);

            GPIOD->ODR &= ~((1 << 5) | (1 << 3)|(1<<4)|(1<<6));
            GPIOE->ODR &= ~((1 << 2)|(1<<4));
        }
        
        if (i==6) {
            GPIOD->ODR |= (1 << 5) | (1 << 3)|(1<<4)|(1<<6);
            GPIOE->ODR |= (1 << 2)|(1<<4);

            GPIOG->ODR |= (1 << 2);
            GPIOG->ODR &= ~(1 << 2);

            GPIOD->ODR &= ~((1 << 5) | (1 << 3)|(1<<4)|(1<<6));
            GPIOE->ODR &= ~((1 << 2)|(1<<4));
        }
        
        if (i==7) {
            GPIOE->ODR &= ~(1 <<4 );
            GPIOG->ODR |= (1 << 3);
            GPIOG->ODR &= ~(1 << 3);

        }
    }
}


void heart(){

    for (int i=0; i<8; i++){
        if (i==0) {
            GPIOD->ODR |= (1 << 4)|(1<<3) | (1 << 7) ;
            GPIOE->ODR |= (1 << 5);
            GPIOC->ODR |= (1 << 8);
            
            GPIOC->ODR &= ~(1 << 8);
            GPIOD->ODR &= ~((1 << 4)|(1<<3) | (1 << 7)) ;
            GPIOE->ODR &= ~(1 << 5);
        }

        if (i==1) {

            GPIOD->ODR |= (1 << 6)|(1<<4)|(1<<3);
            GPIOE->ODR |= (1<<4);

            GPIOC->ODR |= (1 << 9);
            
            GPIOC->ODR &= ~(1 << 9);

            GPIOD->ODR &= ~((1 << 6)|(1<<4)|(1<<3));
            GPIOE->ODR &= ~(1 << 2);
            GPIOE->ODR &= ~(1 << 4);
        }

        if (i==2) {
            GPIOD->ODR |= (1 << 6) ;
            GPIOE->ODR |= (1 << 4);
            GPIOC->ODR |= (1 << 10);
            
            GPIOC->ODR &= ~(1 << 10);
            GPIOD->ODR &= ~(1 << 6) ;
            GPIOE->ODR &= ~(1 << 4);
        }

        if (i==3) {
            GPIOD->ODR |= (1 << 6)|(1<<5); ;
            GPIOE->ODR |= (1 << 4)|(1<<2);
            GPIOC->ODR |= (1 << 11);
            
            GPIOC->ODR &= ~(1 << 11);
            GPIOD->ODR &= ~((1 << 6)|(1<<5)); ;
            GPIOE->ODR &= ~((1 << 4)|(1<<2));
            }
            if (i==4) {
                GPIOD->ODR |= (1 << 6)|(1<<5)|(1<<4)|(1<<3);
                GPIOE->ODR |= (1 << 4)|(1<<2) ;
                GPIOC->ODR |= (1 << 12);
                
                GPIOC->ODR &= ~(1 << 12);
                GPIOD->ODR &= ~((1 << 6)|(1<<5)|(1<<4)|(1<<3));
                GPIOE->ODR &= ~((1 << 4)|(1<<2));
            }
            if (i==5) {
                GPIOD->ODR |= (1 << 7)|(1<<5)|(1<<4)|(1<<3);
                GPIOE->ODR |= (1 << 5)|(1<<2) ;
                GPIOD->ODR |= (1 << 2);
                
                GPIOD->ODR &= ~(1 << 2);
                GPIOD->ODR &= ~((1 << 7)|(1<<5)|(1<<4)|(1<<3));
                GPIOE->ODR &= ~((1 << 5)|(1<<2));
            }
            if (i==6) {
                GPIOD->ODR |= (1 << 7)|(1<<6)|(1<<4)|(1<<3);
                GPIOE->ODR |= (1 << 2)|(1<<5)|(1<<4) ;
                GPIOE->ODR &= ~(1 << 2);
                
                GPIOG->ODR |= (1 << 2);
                
                GPIOG->ODR &= ~(1 << 2);
                
                GPIOD->ODR &= ~((1 << 7)|(1<<6)|(1<<4)|(1<<3));
                GPIOE->ODR &= ~(1 << 2);
                GPIOE->ODR &= ~((1 << 5)|(1<<4));
            }
            if (i==7) {


                GPIOD->ODR|=(1<<7)|(1<<6)|(1<<5);
                GPIOE->ODR|=(1<<2)|(1<<4)|(1<<5);

                GPIOG->ODR|= (1 << 3);
                GPIOG->ODR &= ~(1 << 3);

                GPIOD->ODR&= ~((1<<7)|(1<<6)|(1<<5));
                GPIOE->ODR&= ~((1<<2)|(1<<4)|(1<<5));

                }
             
    }


}

void delay_ms(int ms) {
    for (int i = 0; i < ms * 1000; i++) {
        __asm("nop"); // No operation (simple delay)
    }
}

int read_button_with_debounce() {
    int stable_state = 0;
    int current_state = ((GPIOC->IDR & (1 << 13)) >> 13);

    if (current_state != stable_state) {
        delay_ms(500); // Delay for debouncing (50 ms)
        current_state = ((GPIOC->IDR & (1 << 13)) >> 13);
        if (current_state == stable_state) {
            return 0; // Button not pressed
        } else {
            stable_state = current_state;
            return 1; // Button pressed
        }
    }
    return 0; // Button not pressed
}

void contando() {
    if (read_button_with_debounce()) {
        time += 1;
    }
}

void clear(){


    GPIOF->ODR |= ((1 << 7)|(1 << 9));
    GPIOG->ODR |= (1 << 1);
    GPIOG->ODR |= (1 << 0);
    GPIOD->ODR |= (1 << 1);
    GPIOE->ODR |= (1 << 3);
    GPIOE->ODR |= (1 << 6); 

}

void number0(){
    GPIOF->ODR &= ~(1 << 7);
    GPIOF->ODR &= ~(1 << 9);
    GPIOG->ODR &= ~(1 << 1);
    GPIOG->ODR &= ~(1 << 0);
    GPIOD->ODR &= ~(1 << 1);
    GPIOE->ODR &= ~(1 << 3);


}
void number1() {

    GPIOF->ODR &= ~(1 << 9);
    GPIOG->ODR &= ~(1 << 1);


    /*
    GPIOF->ODR &= ~(1 << 7);
    GPIOG->ODR &= ~(1 << 0);
    GPIOD->ODR &= ~(1 << 1);
    GPIOE->ODR &= ~(1 << 3) & ~(1 << 6);

    */
    }

void number2(){

    /*
    GPIOF->ODR &= ~(1 << 7);
    GPIOF->ODR &= ~(1 << 9);
    GPIOG->ODR &= ~(1 << 1);
    GPIOG->ODR &= ~(1 << 0);
    GPIOD->ODR &= ~(1 << 1);
    GPIOE->ODR &= ~(1 << 3);
    GPIOE->ODR &= ~(1 << 6);

    */


    GPIOF->ODR &= ~(1 << 7);
    GPIOF->ODR &= ~(1 << 9);
    GPIOE->ODR &= ~(1 << 6);
    GPIOD->ODR &= ~(1 << 1);
    GPIOG->ODR &= ~(1 << 0);

    

    
    
}

void number3(){

    GPIOF->ODR &= ~(1 << 7);
    GPIOF->ODR &= ~(1 << 9);
    GPIOG->ODR &= ~(1 << 1);
    GPIOG->ODR &= ~(1 << 0);

    
    GPIOE->ODR &= ~(1 << 6);
    GPIOE->ODR|=(1<<3);
    GPIOD->ODR|=(1<<1);




}

void number4(){

    
    GPIOF->ODR &= ~(1 << 9);
    GPIOG->ODR &= ~(1 << 1);
  
    
    GPIOE->ODR &= ~(1 << 3);
    GPIOE->ODR &= ~(1 << 6);

}

void number5(){



    GPIOF->ODR &= ~(1 << 7);
    GPIOE->ODR &= ~(1 << 3);
    GPIOE->ODR &= ~(1 << 6);
    GPIOG->ODR &= ~(1 << 1);
    GPIOG->ODR &= ~(1 << 0);  
   


}

void number6(){

    GPIOF->ODR &= ~(1 << 7);

    GPIOG->ODR &= ~(1 << 1);
    GPIOG->ODR &= ~(1 << 0);
    GPIOD->ODR &= ~(1 << 1);
    GPIOE->ODR &= ~(1 << 3);
    GPIOE->ODR &= ~(1 << 6);
    
   

}

void number7(){

    GPIOF->ODR &= ~(1 << 7);

    GPIOF->ODR &= ~(1 << 9);

    GPIOG->ODR &= ~(1 << 1);
    GPIOE->ODR &= ~(1 << 6);



}


void number8(){

    GPIOF->ODR &= ~(1 << 7);
    GPIOF->ODR &= ~(1 << 9);
    GPIOG->ODR &= ~(1 << 1);
    GPIOG->ODR &= ~(1 << 0);
    GPIOD->ODR &= ~(1 << 1);
    GPIOE->ODR &= ~(1 << 3);
    GPIOE->ODR &= ~(1 << 6);

}



void number9(){
    GPIOF->ODR &= ~(1 << 7);
    GPIOF->ODR &= ~(1 << 9);
    GPIOG->ODR &= ~(1 << 1);

  
    GPIOE->ODR &= ~(1 << 3);
    GPIOE->ODR &= ~(1 << 6);




}

void number10(){


    GPIOF->ODR &= ~(1 << 7);
    GPIOF->ODR &= ~(1 << 9);
    GPIOG->ODR &= ~(1 << 1);

    GPIOD->ODR &= ~(1 << 1);
    GPIOE->ODR &= ~(1 << 3);
    GPIOE->ODR &= ~(1 << 6);
    


   
}

void number11(){

   
    
    GPIOG->ODR &= ~(1 << 1);
    GPIOG->ODR &= ~(1 << 0);
    GPIOD->ODR &= ~(1 << 1);
    GPIOE->ODR &= ~(1 << 3);
    GPIOE->ODR &= ~(1 << 6);
}

void number12(){
    

    GPIOF->ODR &= ~(1 << 7);


    GPIOG->ODR &= ~(1 << 0);
    GPIOD->ODR &= ~(1 << 1);
    GPIOE->ODR &= ~(1 << 3);



    
}

void number13(){
  
    GPIOF->ODR &= ~(1 << 9);
    GPIOG->ODR &= ~(1 << 1);
    GPIOG->ODR &= ~(1 << 0);
    GPIOD->ODR &= ~(1 << 1);

    GPIOE->ODR &= ~(1 << 6);
 
    

}

void number14(){

    GPIOF->ODR &= ~(1 << 7);


    GPIOG->ODR &= ~(1 << 0);
    GPIOD->ODR &= ~(1 << 1);
    GPIOE->ODR &= ~(1 << 3);
    GPIOE->ODR &= ~(1 << 6);

}

void number15(){


    GPIOF->ODR &= ~(1 << 7);
    

    GPIOG->ODR &= ~(1 << 0);
    GPIOD->ODR &= ~(1 << 1);
    GPIOE->ODR &= ~(1 << 3);
    GPIOE->ODR &= ~(1 << 6);

}










int main() {
    GPIO_MATRIZ();

    while (1) {





        contando();
			

        if (time == 0) {
            clear();
            number0();
            // display 0
        } else if (time == 1) {
            clear();
            number1();
            // display 1
        } else if (time == 2) {
            // display 2
            clear();
            number2();
        } else if (time == 3) {
            // display 3
            for(int n=0;n<2;n++){
                if (n==0){
                    mask();
                }
                else if(n==1){
                    clear();
                    number3();
                }
            }
            GPIOE->ODR|=(1<<3);
            GPIOD->ODR|=(1<<1);
            

         
        } else if (time == 4) {
            // display 4
            clear();
            number4();
        } else if (time == 5) {
            // display 5
            clear();
            number5();
        } else if (time == 6) {
            // display 6
            clear();
            number6();
        } else if (time == 7) {
            // display 7
            face();
            clear();            
            number7();
           
        } else if (time == 8) {
            //just clear
            clear();
            number8();
            
            // display 8
        } else if (time == 9) {

            // display 9
            clear();
            number9();
        } else if (time == 10) {
            // display 10
            clear();
            number10();
        } else if (time == 11) {
            //just clear
            letter();

            clear();
            number11();
            // display 11
            
        } else if (time == 12) {
            // display 12
            clear();
            number12();
        } else if (time == 13) {
            // display 13
            clear();
            number13();
        } else if (time == 14) {
            // display 14
            clear();
            number14();
        } else if (time == 15) {
            // display 15
            heart();
            clear();
            number15();
            
        }
    }
}