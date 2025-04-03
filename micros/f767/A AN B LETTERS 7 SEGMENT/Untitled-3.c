#include <stdio.h>
#include <stm32f7xx.h>

volatile bool stop = false;


// Función de retardo simple (ajusta el factor según tu CPU)
void delay(volatile uint32_t count) {
    while(count--) { }
}

//--------------------- Configuración del Display de 7 Segmentos ---------------------
void configuraDisplay() {
    // Habilitar reloj para GPIOD
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    // Configurar los pines PD1 a PD7 como salidas:
    for (int pin = 1; pin <= 7; pin++) {
        GPIOD->MODER &= ~(0x3 << (pin * 2));  // Limpiar modo
        GPIOD->MODER |=  (0x1 << (pin * 2));    // Modo salida (01)
        GPIOD->OTYPER &= ~(0x1 << pin);         // Salida push-pull
        GPIOD->OSPEEDR |=  (0x1 << (pin * 2));   // Velocidad (ajústalo si es necesario)
        GPIOD->PUPDR &= ~(0x3 << (pin * 2));      // Sin pull-up ni pull-down
    }
}

// Función para mostrar un dígito en el display de 7 segmentos (cátodo común)
// Se asume que el bit 0 del patrón se dirige a PD1, el bit 1 a PD2, …, el bit 6 a PD7.
void displayDigit(uint8_t digit) {
    static const uint8_t digitPatterns[10] = {
        0x3F, // 0: 0b00111111 -> segmentos a, b, c, d, e, f
        0x06, // 1: 0b00000110 -> segmentos b, c
        0x5B, // 2: 0b01011011 -> a, b, d, e, g
        0x4F, // 3: 0b01001111 -> a, b, c, d, g
        0x66, // 4: 0b01100110 -> b, c, f, g
        0x6D, // 5: 0b01101101 -> a, c, d, f, g
        0x7D, // 6: 0b01111101 -> a, c, d, e, f, g
        0x07, // 7: 0b00000111 -> a, b, c
        0x7F, // 8: 0b01111111 -> todos los segmentos
        0x6F  // 9: 0b01101111 -> a, b, c, d, f, g
    };
    uint8_t pattern = (digit < 10) ? digitPatterns[digit] : 0;
    
    // Limpiar PD1 a PD7 (bits 1 a 7 del ODR de GPIOD)
    GPIOD->ODR &= ~0xFE;  // 0xFE = 0b11111110
    // Colocar el patrón desplazado 1 bit (bit0 -> PD1, bit1 -> PD2, …)
    GPIOD->ODR |= (pattern << 1);
}

//--------------------- Configuración del Motor ---------------------
// Se usan PC8 a PC11 para controlar el driver de un motor de 4 entradas
void configurarMotor(void) {
    // Habilitar reloj para GPIOC (ya habilitado en main, pero se puede reasegurar)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    
    // Configurar PC8 a PC11 como salidas
    for (int pin = 8; pin <= 11; pin++) {
        GPIOC->MODER &= ~(0x3 << (pin * 2));
        GPIOC->MODER |= (0x1 << (pin * 2)); // Modo salida
    }
}

void encenderMotor(void) {
    // Aquí se activa el driver: por ejemplo, encenderá el motor según la secuencia
    // Esta función se llama desde la secuencia de pasos
    // No se deja activo de forma fija; se activa por la secuencia
}

void apagarMotor(void) {
    // Apaga todas las salidas de motor
    GPIOC->ODR &= ~((1 << 8) | (1 << 9) | (1 << 10) | (1 << 11));
}



//--------------------- Código Principal ---------------------
int main(void)
{
    // Habilitar relojes para:
    // - GPIOC para el motor y botón de usuario (contador)
    // - GPIOG para el pulsador Start
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;  // Para PC (motor y contador)
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;  // Para PG (pulsador Start)

    // Configurar salidas para el motor (PC8..PC11)
    for (int pin = 8; pin <= 11; pin++) {
        GPIOC->MODER &= ~(0x3 << (pin * 2));
        GPIOC->MODER |= (0x1 << (pin * 2));
    }

    // Configurar botón de usuario (contador) en PC13
    GPIOC->MODER &= ~(0x3 << (13 * 2));  // Entrada
    GPIOC->PUPDR &= ~(0x3 << (13 * 2));
    GPIOC->PUPDR |=  (0x2 << (13 * 2));  // Pull-down

    // Configurar pulsador de Start en PG3
    GPIOG->MODER &= ~(0x3 << (3 * 2));   // Entrada
    GPIOG->PUPDR &= ~(0x3 << (3 * 2));
    GPIOG->PUPDR |=  (0x2 << (3 * 2));   // Pull-down

    configuraDisplay();  // Configurar el display en PD1-PD7
    configurarMotor();   // Configurar el motor en PC8..PC11

    int contar = 0; // Contador de vueltas que debe dar el motor

    while (1)
    {
        // *** Fase de conteo ***
        // Mientras el Start (PG3) no esté presionado, se cuentan las pulsaciones en PC13
        while (((GPIOG->IDR & (1 << 3)) >> 3) == 0) {
            if (((GPIOC->IDR & (1 << 13)) >> 13) == 1) {
                contar++; // Incrementa el contador
                displayDigit(contar % 10); // Actualiza el display con el dígito (módulo 10)
                // Esperar a que se suelte el botón (debounce)
                if(contar >9){
                    contar = 0;
                    displayDigit(0);
                }
                while (((GPIOC->IDR & (1 << 13)) >> 13) == 1) {
                    delay(10000);
                }
            }
        }

        // *** Fase de arranque ***
        // Cuando se presiona el pulsador Start (PG3 == 1)
        if (((GPIOG->IDR & (1 << 3)) >> 3) == 1) {
            delay(10000); // Pequeño retardo de debounce

        stop = !stop;  // Alterna el estado de stop: true = pausa, false = reanudar
        // Espera a que se suelte el pulsador para evitar múltiples toggles
        while (((GPIOG->IDR & (1 << 3)) >> 3) == 1) {
            delay(1000);
        }

            // Ejecuta la secuencia de pasos 'contar' veces
            for (int i = 0; i < contar; i++) {
                // Para cada "vuelta", ejecuta 530 pasos (ajusta este valor según la velocidad deseada)

                for (int paso = 0; paso < 525; paso++) {

                    if (((GPIOG->IDR & (1 << 3)) >> 3) == 1) {
                        delay(10000);
                        stop = !stop;
                        while (((GPIOG->IDR & (1 << 3)) >> 3) == 1) {
                            delay(1000);
                        }
                    }
                    while (stop) {
                        delay(10000);
                        if (((GPIOG->IDR & (1 << 3)) >> 3) == 1) {
                            delay(10000);
                            stop = !stop;
                            while (((GPIOG->IDR & (1 << 3)) >> 3) == 1) {
                                delay(1000);
                            }
                        }
                    }


                    // Secuencia de 4 fases:
                    GPIOC->ODR = (1 << 8);   // Fase 1: PC8 alta
                    delay(5000);
                    GPIOC->ODR = (1 << 9);   // Fase 2: PC9 alta
                    delay(5000);
                    GPIOC->ODR = (1 << 10);  // Fase 3: PC10 alta
                    delay(5000);
                    GPIOC->ODR = (1 << 11);  // Fase 4: PC11 alta
                    delay(5000);
                }
                // Apagar las salidas al terminar cada vuelta
                GPIOC->ODR &= ~((1 << 8) | (1 << 9) | (1 << 10) | (1 << 11));
            }

            // Reinicio el contador y actualizar el display a 0
            contar = 0;
            displayDigit(0);

            // Esperar a que se suelte el pulsador Start para evitar múltiples activaciones
            while (((GPIOG->IDR & (1 << 3)) >> 3) == 1) {
                delay(1000);
            }
        }
    }

    return 0;
}