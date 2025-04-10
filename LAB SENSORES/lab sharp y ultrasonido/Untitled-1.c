#include <stm32f7xx.h>
#include <stdio.h>
#include <stdlib.h>

volatile uint32_t systemTicks = 0;  // Contador global de milisegundos

volatile uint16_t voltage = 0; // Variable para almacenar valor del ADC


// Configuración del Systick
void SysTick_Wait(uint32_t n) {
    SysTick->LOAD = n - 1;
    SysTick->VAL = 0;
    while (((SysTick->CTRL & (1 << 16)) >> 16) == 0);
}

void SysTick_ms(uint32_t x) {
    for (uint32_t i = 0; i < x; i++) {
        SysTick_Wait(16000);  // Para 16MHz, esto da aproximadamente 1ms
    }
}

void enviar(void) {
    GPIOD->ODR |= (1 << 9);  // Activar E
    SysTick_ms(1);           // Retardo para la activación
    GPIOD->ODR &= ~(1 << 9); // Desactivar E
    SysTick_ms(1);           // Retardo para el comando
}

void mostrarNumero(int numero) {
    // Mover a la segunda línea
    GPIOD->ODR &= ~(1 << 8); // RS = 0 (Modo Comando)
    GPIOD->ODR = 0xC0;       // Comando para mover el cursor a la segunda línea
    enviar();
    
    SysTick_ms(5);           // Añadir delay para estabilización
    
    // Convertir el número a caracteres y mostrarlos
    char numStr[10];
    sprintf(numStr, "%d", numero);
    
    GPIOD->ODR |= (1 << 8);  // RS = 1 (Modo Datos)
    
    // Mostrar los dígitos del número
    for (int i = 0; numStr[i] != '\0'; i++) {
        GPIOD->ODR = 0x100 | numStr[i];
        enviar();
        SysTick_ms(2);       // Pequeño delay entre caracteres
    }
    
    SysTick_ms(100);         // Delay final para mantener visible el número
}

void adcportC0(){

    RCC->AHB1ENR|=(1<<2); //ENABLE PORT C CLOCK
    RCC->APB2ENR|=(1<<8)|(1<<9)|(1<<10); //ENABLE ADC 1,2 AND 3 CLOCK
    GPIOC->MODER|=(1<<1)|(1<<0);    //PC0 as analog mode
    GPIOC->PUPDR|=(1<<1); //PC0 as pull down mode
    ADC->CCR|=(1<<17); //F= 2MHZ
    ADC2->CR1|=(1<<24)|(1<<25); //6 bit resolution
    ADC2->CR2|=(1<<0)|(1<<10); // turn on ADC & set end of conversion
    ADC2->CR2 &= ~(1<<11); //left alignment
    ADC2->SMPR2|=(0b111<<0); //480 cycles
    ADC2->SQR3 &= ~(0b11111<<0); //clear channel
    ADC2->SQR3|=(0b00000<<0); //channel 0
}

int main(void) {
    RCC->AHB1ENR |= (1 << 2) | (1 << 3);

    // Configurar ADC
    adcportC0();
    
    // Iniciar conversión ADC
    ADC2->CR2 |= (1<<30); // Start conversion

    // Leer ADC
    while(!(ADC2->SR & (1<<1))); // Wait for conversion to complete (EOC flag)
    voltage = ADC2->DR; // Read value from data register
    float voltageFloat = (voltage * 3.3f) / 4095.0f; // Convert to voltage

    // PARA LA LCD
    GPIOD->MODER |= 0x55555; // Salidas GPIOD 0-10
    GPIOD->OTYPER |= 0x0;
    GPIOD->OSPEEDR |= 0xFFFFF; // Velocidad Very high
    GPIOD->PUPDR |= 0xAAAAA; // Pull Down

    // Configuración SysTick para generar interrupción cada 1 ms
    SysTick->CTRL = 0;                  // Deshabilitar SysTick
    SysTick->LOAD = 16000 - 1;          // Valor para 1ms con 16MHz
    SysTick->VAL = 0;                   // Limpiar el contador
    SysTick->CTRL = 0x00000007;         // Habilitar, usar reloj del procesador, habilitar interrupción

    while (1) {
        // Configuración de LCD para Modo A
        GPIOD->ODR &= ~(1 << 8); // RS = 0 (Modo Comando)
        GPIOD->ODR = (1 << 5) | (1 << 4) | (1 << 3); // Function Set: 8-bit, 2-line, 5x8 font
        enviar();

        GPIOD->ODR = 0x0E; // Display On, Cursor On, Blinking
        enviar();

        GPIOD->ODR = 0x01; // Clear Display
        enviar();
        SysTick_ms(5);     // Dar tiempo para borrar la pantalla

        GPIOD->ODR = 0x06; // Entry Mode Set: Increment cursor, no shift
        enviar();

        GPIOD->ODR = (1 << 1); // Return a Home
        enviar();

        // Ejemplo de mostrar texto en la LCD
        GPIOD->ODR |= (1 << 8);  // RS = 1 (Modo Datos)
        // Aquí puedes agregar los caracteres que quieras mostrar usando el formato:}
        GPIOD->ODR = 0x154; enviar();

        // GPIOD->ODR = 0x1XX; enviar(); donde XX es el código ASCII del carácter
    }
}

// Manejador de interrupción del SysTick
extern "C" void SysTick_Handler(void) {
    systemTicks++;  // Incrementa el contador global cada 1ms
}
