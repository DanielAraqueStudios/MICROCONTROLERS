//matix 8x8 test 1

#include <stdio.h>
#include <stm32f7xx.h>

//VARIABLES

 volatile int userbtn=0;

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


    //c13 como input

    GPIOC->MODER &= ~(0b11<<26); // Limpiar bits

    GPIOC->OSPEEDR |=(1<<26)|(1<<27);
    GPIOC->PUPDR &= ~(0b11 << 26);  // Limpiar bits    
    GPIOC->PUPDR &= ~(0b11 << 26); // Configurar PC13 como pull-DOWN
    GPIOC->PUPDR |= (1 << 27); // Configurar PC13 como pull-DOWN


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

void mascara(){
    // Configurar los pines como salida y encender los LEDs según la matriz dada
GPIOA->ODR |= (1 << 2);  // A2
GPIOA->ODR |= (1 << 5);  // A5

GPIOB->ODR |= (1 << 1);  // B1
GPIOB->ODR |= (1 << 2);  // B2
GPIOB->ODR |= (1 << 3);  // B3
GPIOB->ODR |= (1 << 4);  // B4
GPIOB->ODR |= (1 << 5);  // B5
GPIOB->ODR |= (1 << 6);  // B6

GPIOC->ODR |= (1 << 0);  // C0
GPIOC->ODR |= (1 << 1);  // C1
GPIOC->ODR |= (1 << 2);  // C2
GPIOC->ODR |= (1 << 3);  // C3
GPIOC->ODR |= (1 << 4);  // C4
GPIOC->ODR |= (1 << 5);  // C5
GPIOC->ODR |= (1 << 6);  // C6
GPIOC->ODR |= (1 << 7);  // C7

GPIOD->ODR |= (1 << 0);  // D0
GPIOD->ODR |= (1 << 1);  // D1
GPIOD->ODR |= (1 << 3);  // D3
GPIOD->ODR |= (1 << 4);  // D4
GPIOD->ODR |= (1 << 6);  // D6
GPIOD->ODR |= (1 << 7);  // D7

GPIOE->ODR |= (1 << 1);  // E1
GPIOE->ODR |= (1 << 2);  // E2
GPIOE->ODR |= (1 << 3);  // E3
GPIOE->ODR |= (1 << 4);  // E4
GPIOE->ODR |= (1 << 5);  // E5
GPIOE->ODR |= (1 << 6);  // E6

GPIOF->ODR |= (1 << 1);  // F1

GPIOG->ODR |= (1 << 0);  // G0
GPIOG->ODR |= (1 << 2);  // G2
GPIOG->ODR |= (1 << 3);  // G3
GPIOG->ODR |= (1 << 4);  // G4
GPIOG->ODR |= (1 << 5);  // G5

GPIOH->ODR |= (1 << 0);  // H0
GPIOH->ODR |= (1 << 7);  // H7

}

void mascara2(){

    // Configurar los pines como salida y encender los LEDs según la matriz dada
GPIOA->ODR |= (1 << 2);  // A2
GPIOA->ODR |= (1 << 5);  // A5

GPIOB->ODR |= (1 << 1);  // B1
GPIOB->ODR |= (1 << 2);  // B2
GPIOB->ODR |= (1 << 3);  // B3
GPIOB->ODR |= (1 << 4);  // B4
GPIOB->ODR |= (1 << 5);  // B5
GPIOB->ODR |= (1 << 6);  // B6

GPIOC->ODR |= (1 << 0);  // C0
GPIOC->ODR |= (1 << 1);  // C1
GPIOC->ODR |= (1 << 2);  // C2
GPIOC->ODR |= (1 << 3);  // C3
GPIOC->ODR |= (1 << 4);  // C4
GPIOC->ODR |= (1 << 5);  // C5
GPIOC->ODR |= (1 << 6);  // C6
GPIOC->ODR |= (1 << 7);  // C7

GPIOD->ODR |= (1 << 0);  // D0
GPIOD->ODR |= (1 << 1);  // D1
GPIOD->ODR |= (1 << 3);  // D3
GPIOD->ODR |= (1 << 4);  // D4
GPIOD->ODR |= (1 << 6);  // D6
GPIOD->ODR |= (1 << 7);  // D7

GPIOE->ODR |= (1 << 1);  // E1
GPIOE->ODR |= (1 << 2);  // E2
GPIOE->ODR |= (1 << 3);  // E3
GPIOE->ODR |= (1 << 4);  // E4
GPIOE->ODR |= (1 << 5);  // E5
GPIOE->ODR |= (1 << 6);  // E6

GPIOF->ODR |= (1 << 1);  // F1

GPIOG->ODR |= (1 << 0);  // G0
GPIOG->ODR |= (1 << 2);  // G2
GPIOG->ODR |= (1 << 3);  // G3
GPIOG->ODR |= (1 << 4);  // G4
GPIOG->ODR |= (1 << 5);  // G5

GPIOH->ODR |= (1 << 0);  // H0
GPIOH->ODR |= (1 << 7);  // H7

}

int main() {        
    while(1) {


    userbtn=GPIOC->IDR|=(1<<13);

    if (userbtn==0){
        mascara();

    }
    else if(userbtn==1){
        turn_all();

    }







    

    }
}