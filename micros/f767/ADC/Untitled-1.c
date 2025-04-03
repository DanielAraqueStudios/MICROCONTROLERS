#include <stdio.h>
#include <stm32f7xx.h>

// Declaramos las variables ADC como volatile para que el debugger vea los cambios
volatile uint16_t adc_val1 = 0, adc_val2 = 0, adc_val3 = 0;

// Función para inicializar el ADC2 y configurar los pines PA5, PA6 y PA7 como entradas analógicas
void configurarADC2() {
    // 1. Habilitar el reloj para GPIOA y ADC2
    RCC->AHB1ENR |= (1 << 0);      // Habilita el reloj para GPIOA (bit 0)
    RCC->APB2ENR |= (1 << 9);      // Habilita el reloj para ADC2 (nota: en STM32F7, ADC1, ADC2 y ADC3 se habilitan en APB2ENR; usualmente ADC1=bit8, ADC2=bit9, ADC3=bit10)

    // 2. Configurar PA5, PA6 y PA7 como entradas analógicas:
    // Para que un pin esté en modo analógico se deben poner los 2 bits correspondientes en '11'.
    GPIOA->MODER |= (3 << (5 * 2))  // PA5 en modo analógico (canal ADC2_IN5)
                  | (3 << (6 * 2))  // PA6 en modo analógico (canal ADC2_IN6)
                  | (3 << (7 * 2)); // PA7 en modo analógico (canal ADC2_IN7)
    // Desactivar cualquier configuración de pull-up o pull-down para estos pines
    GPIOA->PUPDR &= ~((3 << (5 * 2)) | (3 << (6 * 2)) | (3 << (7 * 2)));

    // 3. Configurar el ADC2
    // Apagar ADC2 mientras se configura
    ADC2->CR2 &= ~(1 << 0);
    // Configurar el tiempo de muestreo para los canales (se usa SMPR2 para canales 0 a 9)
    // Por ejemplo, asignamos un tiempo de muestreo de 28 ciclos (valor '3') para cada canal
    ADC2->SMPR2 |= (3 << (5 * 3))  // Para canal 5
                  | (3 << (6 * 3))  // Para canal 6
                  | (3 << (7 * 3)); // Para canal 7
    // Configurar el número de conversiones en la secuencia a 1 (SQR1: bits [23:20] en 0 indica 1 conversión)
    ADC2->SQR1 &= ~(0xF << 20);
    // Encender ADC2
    ADC2->CR2 |= (1 << 0);
}

// Función para realizar una conversión ADC en un canal específico
// En STM32F7, para una conversión de un solo canal se utiliza el registro SQR3
uint16_t leerADC2(uint8_t canal) {
    // Configurar el canal deseado en SQR3 para la primera (y única) conversión
    ADC2->SQR3 = canal;
    
    // Iniciar la conversión en modo software: se establece el bit SWSTART (bit 30 de CR2)
    ADC2->CR2 |= (1 << 30);
    
    // Esperar a que se complete la conversión, verificando el flag EOC (End Of Conversion, bit 1 en SR)
    while (!(ADC2->SR & (1 << 1))) {
        // Espera activa
    }
    // Retornar el resultado de la conversión
    return ADC2->DR;
}

int main(void) {
    configurarADC2();  // Inicializar ADC y pines

    // Bucle principal: leer repetidamente los tres potenciómetros conectados a PA5, PA6 y PA7
    while (1) {
        adc_val1 = leerADC2(5); // Leer canal 5: PA5 (ADC2_IN5)
        adc_val2 = leerADC2(6); // Leer canal 6: PA6 (ADC2_IN6)
        adc_val3 = leerADC2(7); // Leer canal 7: PA7 (ADC2_IN7)

    }
}