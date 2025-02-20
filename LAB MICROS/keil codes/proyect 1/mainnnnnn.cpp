#include <stdio.h>
#include <stm32f4xx.h>

void configurarPuertoA(){

//---------Para configurar el reloj del GPIOA
    RCC->AHB1ENR |= (1<<0);

//---------Para configurar el pin 5 de la GPIOA como salida 
    
    // Configuro los pines 0 a 6 como salida
    GPIOA->MODER |= 0x1555; 
    // Configuro los valores en push-pull (Es el de defecto gg)
    GPIOA->OTYPER |= 0x0; 
    // Configuro los pines 0 a 6 como velocidad media (01)
    GPIOA->OSPEEDR |= 0x1555; 
    // Configuro los pines en pull up
    GPIOA->PUPDR |= 0x555; 
}


void letraJ() {
    GPIOA->ODR = 0x71; // Dejo los pines 0, 4, 5 y 6 en 1
}

void letraU() {
    GPIOA->ODR = 0x63; // Dejo pines 0,1,5,6 en 1
}

void letraL() {
    GPIOA->ODR = 0x47; // Dejo pines 0,1,2 y 6 en 1
}

void letraI() {
    GPIOA->ODR = 0x66; // Dejo pines 1,2,5 y 6 en 1
}

void letraA() {
    GPIOA->ODR = 0x4; //Dejo pine 3 en 1
}

void letraN() {
    GPIOA->ODR = 0xB; //Dejo pines 0,1,3 y 5 en 1
}

void letraD() {
    GPIOA->ODR = 0x3E;
}

void letraR() {
    GPIOA->ODR = 0x50;
}

void letraE() {
    GPIOA->ODR = 0x79;
}

void letraS() {
    GPIOA->ODR = 0x6D;
}

void letraO() {
    GPIOA->ODR = 0x3F;
}

void letraG() {
    GPIOA->ODR = 0x3D;
}

void letraC() {
    GPIOA->ODR = 0x39;
}

void letraQ() {
    GPIOA->ODR = 0x3F;
}

void letraT() {
    GPIOA->ODR = 0x78;
}

void letraK() {
    GPIOA->ODR = 0x75;
}

void letraV() {
    GPIOA->ODR = 0x3E;
}

void letraB() {
    GPIOA->ODR = 0x7C;
}


void delay(int milliseconds) {
    for (int i = 0; i < milliseconds * 1000; ++i) {
        // Simple delay loop
    }
}

int main() {
    //------Configuro puerto A------
    configurarPuertoA();

    while(1){
        letraJ();
        delay(1000); // 1 second delay

        letraU();
        delay(1000); // 1 second delay

        letraL();
        delay(1000); // 1 second delay

        letraI();
        delay(1000); // 1 second delay

        letraA();
        delay(1000); // 1 second delay

        letraN();
        delay(1000); // 1 second delay
    }
}