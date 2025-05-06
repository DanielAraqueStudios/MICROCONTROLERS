#include <stm32f767xx.h>
#include <string.h> // Para usar strlen
#include <stdio.h>  // Para usar sprintf

// Variables globales para almacenar valores de ADC y voltaje
volatile double adc1_value = 0;
volatile double volt1 = 0;
volatile uint32_t count_TIM = 0, count_TIM5 = 0;

char voltajeStr[20];           // Para almacenar el valor formateado del voltaje
char distanciaStr[20];         // Para almacenar el valor formateado de la distancia

volatile double volt_1 = 0;
volatile double volt_1_mV = 0;

int Muestra1 = 10;  // Valor por defecto para evitar división por cero

volatile double VoltajeProm1 = 0;
volatile double SumaVoltaje1 = 0;
int Contador1 = 0;

int AlternadorUnidades = 0;

// Prototipos de función
int USART3_SendChar(int value);

// Buffer para recibir comandos vía USART
char rx_buffer[32];
int rx_index = 0;

int main() {
    // USART3 (PB10 TX, PB11 RX)
    RCC->AHB1ENR |= 0xFF;                     // Habilitar relojes AHB1 (GPIOs)
    RCC->APB1ENR |= (1 << 18);                // Habilitar reloj para USART3

    GPIOB->MODER |= (2 << 20) | (2 << 22);    // PB10 y PB11 en modo alterno
    GPIOB->AFR[1] |= (7 << 8) | (7 << 12);    // AF7 para USART3 en PB10 y PB11
    USART3->BRR |= 0x682;
    USART3->CR1 |= ((1 << 2) | (1 << 3) | (1 << 5) | (1 << 0));
    NVIC_EnableIRQ(USART3_IRQn);              // Habilitar interrupción USART3 en NVIC

    // ADC Configuración General 
    RCC->APB2ENR |= 0x100;   // Habilita reloj ADC1

    // ADC1 - Canal 0 (PA0) -> resolución 12 bits, muestreo normal
    RCC->AHB1ENR |= 0x1;     // Habilita reloj GPIOA
    GPIOA->MODER |= 0x3;     // PA0 en modo analógico
    ADC->CCR |= (0 << 16);   // Modo independiente
    ADC1->CR1 |= (0 << 24);  // Resolución 12 bits
    ADC1->SMPR2 |= (5 << 0); // Tiempo de muestreo: 84 ciclos
    ADC1->CR2 |= 0x3;        // ADC ON + modo continuo
    ADC1->SQR3 |= 0;         // Canal 0 (PA0)

    // TIM7 (dispara ADC1)
    RCC->APB1ENR |= (1 << 5); // Habilita reloj TIM7
    TIM7->PSC = 15999;        // Prescaler (PSC)
    TIM7->ARR = 99;           // Auto-reload (ARR) -> genera interrupción cada 0.1 s
    TIM7->DIER |= (1 << 0);   // Habilita interrupción por update
    TIM7->CR1 |= (1 << 0);    // Habilita contador
    NVIC_EnableIRQ(TIM7_IRQn);

    // TIM5 (para UART) 
    RCC->APB1ENR |= (1 << 3); // Habilita reloj TIM5
    TIM5->PSC = 15999;
    TIM5->ARR = 99;
    TIM5->DIER |= (1 << 0);   // Interrupción por update
    TIM5->CR1 |= (1 << 0);    // Activa contador
    NVIC_EnableIRQ(TIM5_IRQn);
    
    // CONFIGURACIÓN INTERRUPCIÓN PARA PB1
    RCC->APB2ENR |= (1 << 14);    // Habilitar reloj SYSCFG (CRITICO)
    RCC->AHB1ENR |= (1 << 1);     // Activar reloj para puerto B
    GPIOB->MODER &= ~(0x3 << 2);  // PB1 en modo entrada (resetear bits 2-3)
    GPIOB->PUPDR |= (0x2 << 2);   // Pull-down en PB1 (bits 2-3 = 0b10)

    SYSCFG->EXTICR[0] &= ~(0xF << 4);  // Limpiar EXTICR1 bits 4-7 (EXTI1)
    SYSCFG->EXTICR[0] |= (0x1 << 4);  // Seleccionar PB1 para EXTI1 (bits 4-7 = 0b0001)

    EXTI->IMR |= (1 << 1);        // Habilitar línea EXTI1
    EXTI->RTSR |= (1 << 1);       // Trigger en flanco de subida para EXTI1
    EXTI->FTSR &= ~(1 << 1);      // Deshabilitar trigger en flanco de bajada
    NVIC_EnableIRQ(EXTI1_IRQn);   // Habilitar interrupción EXTI1

    // CONFIGURACIÓN DE PINES MATRIZ 8X8
    RCC->AHB1ENR |= (1 << 3) | (1 << 4); // GPIOD y GPIOE activados
    GPIOD->MODER |= 0x55555555;
    GPIOE->MODER |= 0x5555;

    // ================================ LOOP PRINCIPAL ==============================
    while (1) {
        count_TIM = TIM7->CNT;   // Lectura del contador de TIM7 (debug)
        count_TIM5 = TIM5->CNT;  // Lectura del contador de TIM5 (debug)  
    }
}

// =============================== INTERRUPCIONES =================================
extern "C" {

// ---------- TIM7_IRQHandler: Leer ADC1 (PA0) cada 0.1 s ----------
void TIM7_IRQHandler(void) {
    TIM7->SR &= ~(1 << 0);             // Limpia bandera de interrupción    
        
    for(int i = 0; i < Muestra1; i++){
        ADC1->CR2 |= (1 << 30);            // Inicia conversión por software
        while (!(ADC1->SR & (1 << 1)));    // Espera fin de conversión (EOC)
        adc1_value = ADC1->DR;             // Lee valor
        volt1 = (adc1_value * 3.3) / 4095.0; // Conversión a voltaje (12 bits)
        ADC1->SR &= ~(1 << 1);             // Limpia bandera EOC
        
        // Cálculo del promedio para el sensor 1
        SumaVoltaje1 += volt1;
    }       
    
    VoltajeProm1 = SumaVoltaje1 / Muestra1;
    SumaVoltaje1 = 0;

    // Envía los valores por USART3 en formato legible
    char buffer[32];
    int len1 = sprintf(buffer, "V1=%.2f\n", VoltajeProm1);
    for (int i = 0; i < len1; i++) {
        USART3_SendChar(buffer[i]);
    }
}

// ---------- TIM5_IRQHandler: Enviar datos UART ----------
void TIM5_IRQHandler(void) {
    TIM5->SR &= ~(1 << 0);             // Limpia bandera
    // No hay lectura de ADC2 en esta versión
}

// ---------- USART3_IRQHandler: Recibe comandos para cambiar tiempos y muestras ----------
void USART3_IRQHandler(void) {
    while (USART3->ISR & USART_ISR_RXNE) {
        char rx = (char)(USART3->RDR & 0xFF);  // Lee carácter recibido

        if (rx == '\n') {
            rx_buffer[rx_index] = '\0';       // Termina cadena
            rx_index = 0;

            if (rx_buffer[0] == 'P') {        // Comando para TIM7
                int psc_val, arr_val;
                if (sscanf(&rx_buffer[1], "%d,%d", &psc_val, &arr_val) == 2) {
                    TIM7->CR1 &= ~(1 << 0);   // Pausa temporizador
                    TIM7->PSC = psc_val;
                    TIM7->ARR = arr_val;
                    TIM7->CNT = 0;
                    TIM7->CR1 |= (1 << 0);    // Reanuda
                }
            } else if (rx_buffer[0] == 'M') { // Comando para muestra1
                if (sscanf(&rx_buffer[1], "%d", &Muestra1) == 1) {
                    // Reiniciar los acumuladores cuando cambia el número de muestras
                    SumaVoltaje1 = 0;
                    Contador1 = 0;
                }
            }
        } else {
            // Agrega al buffer si no se ha alcanzado el límite
            if (rx_index < sizeof(rx_buffer) - 1) {
                rx_buffer[rx_index++] = rx;
            }
        }
    }
}

// ---------- Interrupcion para cambio de unidades ----------
void EXTI1_IRQHandler(void) {
    if (EXTI->PR & (1 << 1)) {
        EXTI->PR |= (1 << 1);
        AlternadorUnidades = !AlternadorUnidades;
    }
}
}

// =============================== FUNCIONES  =================================

// FUNCION PARA ENVIAR UN CARACTER POR EL USART3
int USART3_SendChar(int value) {
    USART3->TDR = value;                         // Carga valor en el registro de transmisión
    while (!(USART3->ISR & USART_ISR_TXE));     // Espera a que se complete la transmisión
    return 0;
}