#include <stm32f7xx.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define FREQTRAB 16000000     // Frecuencia de trabajo del microcontrolador S
#define PI 3.14159265359
#define muestras 256           // Número de muestras por ciclo A

void configs(void);
void SysTick_Inicio(void);
void SysTick_Wait(uint32_t n);
void generar_ondas(void);
uint16_t escala_DAC(float voltage);
void USART2_Inicio(void);
char USART2_leerChar(void);
void procesar_coman(char *cmd);
void USART2_escribirChar(char c);
void USART2_escribirString(char *str);
void actualizar_incrementos(void);
void ConfigurarTimersFrecuencia(void);

// Tablas para ambas salidas
volatile uint16_t tabla1_seno[muestras];
volatile uint16_t tabla1_triangular[muestras];
volatile uint16_t tabla1_cuadrada[muestras];
volatile uint16_t tabla1_sierra[muestras];

volatile uint16_t tabla2_seno[muestras];
volatile uint16_t tabla2_triangular[muestras];
volatile uint16_t tabla2_cuad[muestras];
volatile uint16_t tabla2_sierra[muestras];

// Parámetros DAC1
volatile float amplitud1 = 1.5;
volatile float offset1 = 1.5;
volatile float frec1 = 1000.0;  // Frecuencia inicial modificada
volatile uint8_t onda1 = 0;

// Parámetros DAC2
volatile float amplitud2 = 1.0;
volatile float offset2 = 1.65;  // Corregido a un valor más razonable
volatile float frec2 = 1000.0;  // Frecuencia inicial modificada
volatile uint8_t onda2 = 1;

// Variables para control de tiempo con precisión de 64 bits N
volatile uint64_t fase1 = 0;
volatile uint64_t fase2 = 0;
volatile uint64_t fase_incre_1;
volatile uint64_t fase_increm_2;

// Usa una base de tiempo más precisa (en microsegundos) T
volatile uint32_t baseTimer = 10; // 10 microsegundos para mayor precisión I


// Factor de escala para mayor precisión en los cálculos A
#define presicion_fase 0x100000000LL  // 2^32 para mayor precisión G

// Variable para almacenar la frecuencia medida
volatile float frecuencia = 0.0f;  

void actualizar_incrementos(void) {
    // Factor de corrección de frecuencia
    const double FREQ_CORRECTION = 2.0;
    
    // Aplicar factor de corrección a las frecuencias
    double freq1_corrected = frec1 * FREQ_CORRECTION;
    double freq2_corrected = frec2 * FREQ_CORRECTION;
    
    // Calcular incrementos con frecuencias corregidas
    fase_incre_1 = (uint64_t)((freq1_corrected * muestras * baseTimer * (double)presicion_fase) / 1000000.0);
    fase_increm_2 = (uint64_t)((freq2_corrected * muestras * baseTimer * (double)presicion_fase) / 1000000.0);
    
    // Recalcular frecuencias reales
    float realFreq1 = ((float)fase_incre_1 * 1000000.0f) / (presicion_fase * muestras * baseTimer);
    float realFreq2 = ((float)fase_increm_2 * 1000000.0f) / (presicion_fase * muestras * baseTimer);
    
    // Ajuste fino para minimizar el error de redondeo
    // Solo si hay una diferencia significativa (>0.01%) V
    if (fabsf(realFreq1 - frec1) / frec1 > 0.0001f) {
        // Ajustar incremento para acercarse más a la frecuencia deseada
        double error = frec1 / realFreq1;
        fase_incre_1 = (uint64_t)(fase_incre_1 * error);
        // Verificar el resultado
        realFreq1 = ((float)fase_incre_1 * 1000000.0f) / (presicion_fase * muestras* baseTimer);
    }
    
    if (fabsf(realFreq2 - frec2) / frec2 > 0.0001f) {
        // Ajustar incremento para acercarse más a la frecuencia deseada P
        double error = frec2 / realFreq2;
        fase_increm_2 = (uint64_t)(fase_increm_2 * error);
        // Verificar el resultado
        realFreq2 = ((float)fase_increm_2 * 1000000.0f) / (presicion_fase * muestras * baseTimer);
    }
}

void generar_ondas(void) {
    // Generar tablas para DAC1
    for (uint32_t i = 0; i < muestras; i++) {
        float t = (float)i / muestras;
        
        // DAC1 - Seno
        float seno = sinf(2 * PI * t);
        tabla1_seno[i] = escala_DAC(amplitud1 * seno + offset1);
        
        // DAC1 - Triangular
        float tri = 2.0f * fabsf(2.0f * (t - floorf(t + 0.5f))) - 1.0f;
        tabla1_triangular[i] = escala_DAC(amplitud1 * tri + offset1);
        
        // DAC1 - Cuadrada
        float sq = (t < 0.5f) ? 1.0f : -1.0f;
        tabla1_cuadrada[i] = escala_DAC(amplitud1 * sq + offset1);
        
        // DAC1 - Sierra
        float saw = 2.0f * t - 1.0f;
        tabla1_sierra[i] = escala_DAC(amplitud1 * saw + offset1);
        
        // DAC2 - Seno
        tabla2_seno[i] = escala_DAC(amplitud2 * seno + offset2);
        
        // DAC2 - Triangular
        tabla2_triangular[i] = escala_DAC(amplitud2 * tri + offset2);
        
        // DAC2 - Cuadrada
        tabla2_cuad[i] = escala_DAC(amplitud2 * sq + offset2);
        
        // DAC2 - Sierra
        tabla2_sierra[i] = escala_DAC(amplitud2 * saw + offset2);
    }
}

uint16_t escala_DAC(float voltage) {
    // Limitar voltaje entre 0V y 3.3V
    if (voltage > 3.3f) voltage = 3.3f;
    if (voltage < 0.0f) voltage = 0.0f;
    
    // Convertir a valor DAC (0-4095)
    return (uint16_t)((voltage / 3.3f) * 4095);
}

void configs(void) {
    // Habilitar reloj para GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // Habilitar reloj para DAC
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    // Configurar PA4 y PA5 como analógicos (DAC1 y DAC2)
    GPIOA->MODER |= (3 << 8) | (3 << 10); 

    // Habilitar DAC1 y DAC2
    DAC->CR |= (1 << 0);   // DAC1 enable
    DAC->CR |= (1 << 16);  // DAC2 enable
}

void SysTick_Inicio(void) {
    SysTick->LOAD = 0xFFFFFF;  // Valor máximo
    SysTick->CTRL = 0x05;      // Habilitar, sin interrupciones
}

void SysTick_Wait(uint32_t n) {
    SysTick->LOAD = n - 1;     // Cargar valor
    SysTick->VAL = 0;          // Resetear contador
    while ((SysTick->CTRL & 0x10000) == 0);  // Esperar bandera
}

void USART2_Inicio(void) {
    // Habilitar reloj para GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    // Habilitar reloj para USART2
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // Configurar PA2 (TX) y PA3 (RX) para USART2
    GPIOA->MODER &= ~((3 << 4) | (3 << 6));
    GPIOA->MODER |= (2 << 4) | (2 << 6);      // Alternate function
    GPIOA->AFR[0] |= (7 << 8) | (7 << 12);    // AF7 (USART2)

    // Configurar baudrate a 115200
    USART2->BRR = FREQTRAB / 9600;
    // Habilitar TX, RX y USART
    USART2->CR1 |= USART_CR1_RE | USART_CR1_TE | USART_CR1_UE;
}

char USART2_leerChar(void) {
    // Esperar a que haya datos
    while (!(USART2->ISR & USART_ISR_RXNE));
    // Leer y retornar datos
    return USART2->RDR;
}

void USART2_escribirChar(char c) {
    // Esperar a que el registro de transmisión esté vacío
    while (!(USART2->ISR & USART_ISR_TXE));
    // Escribir el carácter
    USART2->TDR = c;
}

void USART2_escribirString(char *str) {
    // Enviar cada carácter
    while (*str) {
        USART2_escribirChar(*str++);
    }
}

void procesar_coman(char *cmd) {
    // Verificar longitud mínima
    if (strlen(cmd) < 3) {
        USART2_escribirString("Error: Comando demasiado corto\r\n");
        return;
    }
    
    // Extraer el número de DAC (1 o 2)
    uint8_t dac = cmd[0] - '0';
    
    // Verificar DAC válido
    if (dac != 1 && dac != 2) {
        USART2_escribirString("Error: DAC debe ser 1 o 2\r\n");
        return;
    }
    
    // Extraer el tipo de comando (segundo carácter)
    char type = cmd[1];
    
    // Extraer el valor numérico
    float value = atof(&cmd[2]);  // Cambiado a atof para manejar decimales
    
    char response[80];
    uint8_t need_update = 0;
    
    // Procesar comandos según el tipo
    switch (type) {
        case 'W':  // Forma de onda (Wave) - entero 0-3
            if (value < 0 || value > 3) {
                USART2_escribirString("Error: Waveform debe ser 0-3\r\n");
                return;
            }
            if (dac == 1) {
                onda1 = (uint8_t)value;
            } else {
                onda2 = (uint8_t)value;
            }
            need_update = 1;
            break;
            
        case 'F':  // Frecuencia - valor flotante
            if (dac == 1) {
                frec1 = value;
            } else {
                frec2 = value;
            }
            actualizar_incrementos();
            need_update = 1;
            break;
            
        case 'A':  // Amplitud - valor flotante 0-3.3V
            if (value < 0 || value > 3.3) {
                USART2_escribirString("Error: Amplitud debe ser 0-3.3V\r\n");
                return;
            }
            if (dac == 1) {
                amplitud1 = value;
            } else {
                amplitud2 = value;
            }
            need_update = 1;
            break;
            
        case 'O':  // Offset - valor flotante 0-3.3V
            if (value < 0 || value > 3.3) {
                USART2_escribirString("Error: Offset debe ser 0-3.3V\r\n");
                return;
            }
            if (dac == 1) {
                offset1 = value;
            } else {
                offset2 = value;
            }
            need_update = 1;
            break;
            
        default:
            sprintf(response, "Error: Comando desconocido: %c\r\n", type);
            USART2_escribirString(response);
            return;
    }

    // Regenerar tablas si hubo cambios
    if (need_update) {
        generar_ondas();
    }

    // Enviar confirmación
    USART2_escribirString("OK\r\n");
}

void USART2_limpiarBuffer(void) {
    while (USART2->ISR & USART_ISR_RXNE) {
        (void)USART2->RDR; // Leer y descartar
    }
}

// Configuración PB5 como entrada de frecuencia
void ConfigurarTimersFrecuencia(void) {
    RCC->AHB1ENR |= (1<<1);  // GPIOB clock
    GPIOB->MODER &= ~(3<<10);  // Clear PB5
    GPIOB->MODER |= (2<<10);   // Alternate Function
    GPIOB->AFR[0] &= ~(0xF<<20);  // Clear AF
    GPIOB->AFR[0] |= (2<<20);     // AF2 (TIM3_CH2)

    // Timer3 como contador de pulsos externos
    RCC->APB1ENR |= (1<<1);    // TIM3 clock
    TIM3->PSC = 0;             // Sin preescaler
    TIM3->ARR = 0xFFFF;        // Máximo conteo
    TIM3->CCMR1 |= (1<<8);     // CC2 como entrada
    TIM3->CCER |= (1<<4);      // Captura en CH2 habilitada
    TIM3->SMCR |= (0b111<<0);  // External Clock Mode 1
    TIM3->SMCR |= (0b110<<4);  // TI2FP2 como entrada
    TIM3->CR1 |= (1<<0);       // Enable counter

    // Timer2 para base de tiempo 1 segundo
    RCC->APB1ENR |= (1<<0);    // TIM2 clock
    TIM2->PSC = 15999;         // 16MHz/16000 = 1kHz
    TIM2->ARR = 999;           // 1kHz/1000 = 1Hz (1 segundo)
    TIM2->DIER |= (1<<0);      // Interrupt enable
    TIM2->CR1 |= (1<<0);       // Enable counter
    NVIC_EnableIRQ(TIM2_IRQn);
}

extern "C" {
    void TIM2_IRQHandler(void) {
        TIM2->SR &= ~(1<<0);  // Limpiar flag
        frecuencia = (float)TIM3->CNT;  // Leer conteo
        TIM3->CNT = 0;  // Resetear contador
        
        // Debug: enviar frecuencia por UART
        char freqStr[32];
        sprintf(freqStr, "Measured Freq: %.1f Hz\r\n", frecuencia);
        USART2_escribirString(freqStr);
    }
}

int main(void) {
    configs();
    SysTick_Inicio();
    USART2_Inicio();
    
    // Generar tablas iniciales O
    generar_ondas();
    
    // Configurar los incrementos de fase iniciales
    actualizar_incrementos();

    char cmdBuffer[32];
    uint8_t cmdIndex = 0;

    // Enviar mensaje de inicio
    USART2_escribirString("STM32 Signal Generator Ready\r\n");
    
    // Reportar las frecuencias iniciales
    char freqInfo[100];
    float realFreq1 = ((float)fase_incre_1 * 1000000.0f) / (presicion_fase * muestras* baseTimer);
    float realFreq2 = ((float)fase_increm_2 * 1000000.0f) / (presicion_fase * muestras * baseTimer);
    sprintf(freqInfo, "Initial freqs: DAC1=%.5f Hz, DAC2=%.5f Hz\r\n", realFreq1, realFreq2);
    USART2_escribirString(freqInfo);

    // Configurar timers de frecuencia
    ConfigurarTimersFrecuencia();
    
    while (1) {
    // Actualizar fases y DACs (código existente)
    fase1 += fase_incre_1;
    fase2 += fase_increm_2;
    
    uint32_t index1 = (fase1 >> 32) % muestras;
    switch (onda1) {
        case 0: DAC->DHR12R1 = tabla1_seno[index1]; break;
        case 1: DAC->DHR12R1 = tabla1_triangular[index1]; break;
        case 2: DAC->DHR12R1 = tabla1_cuadrada[index1]; break;
        case 3: DAC->DHR12R1 = tabla1_sierra[index1]; break;
    }
    
    uint32_t index2 = (fase2 >> 32) % muestras;
    switch (onda2) {
        case 0: DAC->DHR12R2 = tabla2_seno[index2]; break;
        case 1: DAC->DHR12R2 = tabla2_triangular[index2]; break;
        case 2: DAC->DHR12R2 = tabla2_cuad[index2]; break;
        case 3: DAC->DHR12R2 = tabla2_sierra[index2]; break;
    }

    // Esperar el período base
    SysTick_Wait(FREQTRAB * baseTimer / 1000000);

    // Procesar todos los caracteres disponibles en el buffer
    while (USART2->ISR & USART_ISR_RXNE) {
        char c = USART2_leerChar();
        
        // Si recibimos CR o LF (final de comando)
        if (c == '\r' || c == '\n') {
            if (cmdIndex > 0) {  // Sólo procesar si hay datos
                cmdBuffer[cmdIndex] = '\0';
                procesar_coman(cmdBuffer);
                cmdIndex = 0;
                
                // Pequeño delay para permitir que se procese el comando
                for (volatile int i = 0; i < 1000; i++);
            }
        } 
        // De lo contrario, acumular caracteres en el buffer
        else if (cmdIndex < sizeof(cmdBuffer) - 1) {
            cmdBuffer[cmdIndex++] = c;
        }
        else {
            // Buffer lleno, descartar comando
            cmdIndex = 0;
            USART2_escribirString("Error: Comando demasiado largo\r\n");
        }
    }
}