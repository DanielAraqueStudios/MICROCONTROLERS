#include <stm32f7xx.h>

// Variables globales
volatile int modo = 0;          // 0 = Contador, 1 = Detección de color
volatile uint32_t contador_tiempo = 0;
volatile uint32_t tiempo_modo = 0;
volatile uint16_t adc_value = 0;
volatile float VOLT_FTR = 0;
volatile int mostrar_codigo_flag = 0;
volatile int codigo_actual = -1; // -1 = Ningún color detectado

// Estructura para códigos de colores (según tu tabla)
typedef struct {
    char char1;
    char char2;
    int digit1;
    int digit2;
    float min_volt;
    float max_volt;
} ColorCode;

// Tabla de colores actualizada con rangos ajustados
const ColorCode colorCodes[] = {
    {'A', 'C', 5, 7, 3.25f, 3.3f},   // NARANJA - AC57
    {'E', 'F', 4, 2, 3.01f, 3.24f},   // AZUL - EF42
    {'B', 'E', 3, 6, 2.5f, 2.7f},   // negro
    {'A', 'D', 1, 8, 2.7f, 3.0f}    // VERDE - AD18 (rango ampliado)
};

// Tabla de segmentos para números (ánodo común)
const uint16_t segmentTable[] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b01000011, // 7
    0b01111111, // 8
    0b01101111  // 9
};

// Pines de displays (ánodo común)
const uint16_t displayPins[] = {
    1 << 7,   // D1 - PA7 (miles)
    1 << 3,   // D2 - PA3 (centenas)
    1 << 4,   // D3 - PA4 (decenas)
    1 << 11   // D4 - PA11 (unidades)
};

// Pines de segmentos (A-G)
const uint16_t segmentPins[] = {
    1 << 1,   // A  - PA1
    1 << 5,   // B  - PA5
    1 << 9,   // C  - PA9
    1 << 8,   // D  - PB8
    1 << 6,   // E  - PA6
    1 << 2,   // F  - PA2
    1 << 10   // G  - PA10
};

// Patrones para letras (A-F)
const uint16_t charA = 0b01110111;  // A
const uint16_t charB = 0b01111100;  // B
const uint16_t charC = 0b00111001;  // C
const uint16_t charD = 0b01011110;  // D
const uint16_t charE = 0b01111001;  // E
const uint16_t charF = 0b01110001;  // F

// Variables del contador
volatile int unidades = 0;
volatile int decenas = 0;
volatile int centenas = 0;
volatile int miles = 0;
volatile int prev_unidades = 0;
volatile int prev_decenas = 0;
volatile int prev_centenas = 0;
volatile int prev_miles = 0;

// Prototipos de funciones
void delay(volatile uint32_t s);
void SysTick_Init(void);
void mostrarDigito(uint8_t digito, uint8_t posicion);
void mostrarCaracter(uint16_t patron, uint8_t posicion);
void leerADC(void);
void comprobarColorADC(void);

// Función de delay simple
void delay(volatile uint32_t s) {
    for(; s>0; s--);
}

// Mostrar un dígito en un display específico
void mostrarDigito(uint8_t digito, uint8_t posicion) {
    // Apagar todos los displays
    GPIOA->ODR |= (GPIO_ODR_ODR_7 | GPIO_ODR_ODR_3 | GPIO_ODR_ODR_4 | GPIO_ODR_ODR_11);
    
    // Apagar todos los segmentos
    GPIOA->ODR |= (GPIO_ODR_ODR_1 | GPIO_ODR_ODR_2 | GPIO_ODR_ODR_5 | GPIO_ODR_ODR_6 | GPIO_ODR_ODR_9 | GPIO_ODR_ODR_10);
    GPIOB->ODR |= (1 << 8);
    
    // Obtener patrón del dígito
    uint16_t patron = segmentTable[digito];
    
    // Encender segmentos según el patrón (negado para ánodo común)
    if (!(patron & 0x01)) GPIOA->ODR &= ~(1 << 1);   // A
    if (!(patron & 0x02)) GPIOA->ODR &= ~(1 << 5);   // B
    if (!(patron & 0x04)) GPIOA->ODR &= ~(1 << 9);   // C
    if (!(patron & 0x08)) GPIOB->ODR &= ~(1 << 8);   // D
    if (!(patron & 0x10)) GPIOA->ODR &= ~(1 << 6);   // E
    if (!(patron & 0x20)) GPIOA->ODR &= ~(1 << 2);   // F
    if (!(patron & 0x40)) GPIOA->ODR &= ~(1 << 10);  // G
    
    // Activar display seleccionado
    GPIOA->ODR &= ~displayPins[posicion];
    
    delay(1000); // Pequeño retardo para persistencia visual
}

// Mostrar un carácter (letra) en un display
void mostrarCaracter(uint16_t patron, uint8_t posicion) {
    // Apagar todos los displays
    GPIOA->ODR |= (GPIO_ODR_ODR_7 | GPIO_ODR_ODR_3 | GPIO_ODR_ODR_4 | GPIO_ODR_ODR_11);
    
    // Apagar todos los segmentos
    GPIOA->ODR |= (GPIO_ODR_ODR_1 | GPIO_ODR_ODR_2 | GPIO_ODR_ODR_5 | GPIO_ODR_ODR_6 | GPIO_ODR_ODR_9 | GPIO_ODR_ODR_10);
    GPIOB->ODR |= (1 << 8);
    
    // Encender segmentos según el patrón (negado para ánodo común)
    if (!(patron & 0x01)) GPIOA->ODR &= ~(1 << 1);   // A
    if (!(patron & 0x02)) GPIOA->ODR &= ~(1 << 5);   // B
    if (!(patron & 0x04)) GPIOA->ODR &= ~(1 << 9);   // C
    if (!(patron & 0x08)) GPIOB->ODR &= ~(1 << 8);   // D
    if (!(patron & 0x10)) GPIOA->ODR &= ~(1 << 6);   // E
    if (!(patron & 0x20)) GPIOA->ODR &= ~(1 << 2);   // F
    if (!(patron & 0x40)) GPIOA->ODR &= ~(1 << 10);  // G
    
    // Activar display seleccionado
    GPIOA->ODR &= ~displayPins[posicion];
    
    delay(1000); // Pequeño retardo para persistencia visual
}

// Inicializar SysTick
void SysTick_Init(void) {
    SysTick->CTRL = 0;                // Deshabilitar SysTick
    SysTick->LOAD = 16000 - 1;        // 1ms a 16 MHz
    SysTick->VAL = 0;                 // Limpiar valor actual
    SysTick->CTRL = 7;                // Habilitar, usar reloj del procesador, generar interrupciones 
}

// Leer valor del ADC (fotorresistencia)
void leerADC(void) {
    uint32_t suma = 0;
    const int muestras = 10; // Tomamos 10 muestras para suavizar la señal

    for (int i = 0; i < muestras; i++) {
        ADC1->CR2 |= ADC_CR2_SWSTART;  // Iniciar conversión
        while (!(ADC1->SR & ADC_SR_EOC));  // Esperar a que termine
        suma += ADC1->DR;  // Sumar valor
    }

    adc_value = suma / muestras;  // Promediar
    VOLT_FTR = (adc_value * 3.3) / 4095.0;  // Convertir a voltaje
}

// Comprobar el color detectado con mejor manejo de rangos
void comprobarColorADC(void) {
    leerADC();  
    int previous_codigo = codigo_actual;
    codigo_actual = -1;  // Reiniciar detección

    // Recorrer todos los colores en la tabla y comparar el voltaje
    for (int i = 0; i < 4; i++) {
        if (VOLT_FTR >= colorCodes[i].min_volt && VOLT_FTR <= colorCodes[i].max_volt) {
            codigo_actual = i;
            break; // Salimos al detectar el primer color válido
        }
    }

    // Si cambió el color detectado, activar la bandera de actualización
    if (previous_codigo != codigo_actual) {
        mostrar_codigo_flag = 1;
    }
}

void mostrarCodigoColor(void) {
    if (codigo_actual < 0 || codigo_actual > 3) return;

    ColorCode code = colorCodes[codigo_actual];

    // Mostrar en orden: char1, char2, digit1, digit2
    mostrarCaracter(
        (code.char1 == 'A') ? charA :
        (code.char1 == 'B') ? charB :
        (code.char1 == 'C') ? charC :
        (code.char1 == 'D') ? charD :
        (code.char1 == 'E') ? charE :
        (code.char1 == 'F') ? charF : 0,
        0 // posición miles
    );

    mostrarCaracter(
        (code.char2 == 'A') ? charA :
        (code.char2 == 'B') ? charB :
        (code.char2 == 'C') ? charC :
        (code.char2 == 'D') ? charD :
        (code.char2 == 'E') ? charE :
        (code.char2 == 'F') ? charF : 0,
        1 // posición centenas
    );

    mostrarDigito(code.digit1, 2); // decenas
    mostrarDigito(code.digit2, 3); // unidades
}

extern "C" {
    // Interrupción SysTick (cada 1ms)
    void SysTick_Handler(void) {
        contador_tiempo++;
        
        if (modo == 1) { // Modo detección de color
            tiempo_modo++;
            
            // Volver a modo contador después de 30 segundos
            if (tiempo_modo >= 30000) {
                modo = 0;
                tiempo_modo = 0;
                mostrar_codigo_flag = 0;
                
                // Restaurar valores del contador
                unidades = prev_unidades;
                decenas = prev_decenas;
                centenas = prev_centenas;
                miles = prev_miles;
            }
            
            // Leer ADC cada 100ms en modo detección (era 500ms)
            static uint32_t last_adc = 0;
            if (contador_tiempo - last_adc > 100) {
                comprobarColorADC();
                last_adc = contador_tiempo;
            }
        }
        // Modo contador normal (incrementar cada 500ms)
        else if (contador_tiempo >= 500) {
            unidades++;
            
            if (unidades > 9) {
                unidades = 0;
                decenas++;
                if (decenas > 9) {
                    decenas = 0;
                    centenas++;
                    if (centenas > 9) {
                        centenas = 0;
                        miles++;
                        if (miles > 9) miles = 0;
                    }
                }
            }
            contador_tiempo = 0;
        }
    }
    
    // Interrupción EXTI (botón PC13)
    void EXTI15_10_IRQHandler(void) {
        if (EXTI->PR & EXTI_PR_PR13) {
            EXTI->PR = EXTI_PR_PR13; // Limpiar bandera
            
            // Debounce mejorado
            for(volatile uint32_t i=0; i<50000; i++);
            
            if ((GPIOC->IDR & (1 << 13)) == 0) {
                if (modo == 0) { // Cambiar a modo detección
                    // Guardar estado actual del contador
                    prev_unidades = unidades;
                    prev_decenas = decenas;
                    prev_centenas = centenas;
                    prev_miles = miles;
                    
                    modo = 1;
                    tiempo_modo = 0;
                    codigo_actual = -1; // Reiniciar detección
                    mostrar_codigo_flag = 0;
                    comprobarColorADC(); // Leer color inmediatamente
                } else { // Volver a modo contador
                    modo = 0;
                    tiempo_modo = 0;
                }
            }
        }
    }
}

int main(void) {
    // 1. Configuración de relojes
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_SYSCFGEN;
    
    // 2. Configuración de GPIO (displays y segmentos)
    GPIOA->MODER |= 0x55555555; // PA0-PA11 como salidas
    GPIOB->MODER |= (1 << (8 * 2)); // PB8 como salida
    
    // Inicializar todos los displays apagados
    GPIOA->ODR |= (GPIO_ODR_ODR_7 | GPIO_ODR_ODR_3 | GPIO_ODR_ODR_4 | GPIO_ODR_ODR_11);
    
    // Inicializar todos los segmentos apagados
    GPIOA->ODR |= (GPIO_ODR_ODR_1 | GPIO_ODR_ODR_2 | GPIO_ODR_ODR_5 | GPIO_ODR_ODR_6 | GPIO_ODR_ODR_9 | GPIO_ODR_ODR_10);
    GPIOB->ODR |= (1 << 8);
    
    // 3. Configuración de ADC (PA12)
    GPIOA->MODER |= (3 << (12 * 2)); // PA12 como analógico
    ADC1->SQR3 = 12; // Canal 12 (PA12)
    ADC1->CR2 |= ADC_CR2_ADON; // Encender ADC
	
	// Habilitar el reloj para GPIOA y ADC1
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

// Configurar PA0 como entrada analógica
GPIOA->MODER |= (3 << 0); // PA0 modo analógico (MODER0[1:0] = 11)
GPIOA->PUPDR &= ~(3 << 0); // Sin pull-up ni pull-down

// Configurar el ADC1 (canal 0 = PA0)
ADC1->SQR3 = 0; // Canal 0
ADC1->SQR1 = 0; // Solo 1 conversión
ADC1->CR2 |= ADC_CR2_ADON; // Encender ADC1
    
    // 4. Configuración de interrupciones
    // Botón PC13 (EXTI13)
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI13_PC;
    EXTI->IMR |= EXTI_IMR_IM13;
    EXTI->FTSR |= EXTI_RTSR_TR13;
    NVIC_EnableIRQ(EXTI15_10_IRQn);
    
    // SysTick (1ms)
    SysTick_Init();
    
    // Bucle principal - modificado para mejor gestión de los displays
    while(1) {
        if (modo == 1) { // Modo detección de color
            if (codigo_actual != -1) {
                // Multiplexado de displays - rotar entre las posiciones
                static int display_pos = 0;
                
                // Mostrar el carácter/dígito correspondiente a la posición actual
                switch(display_pos) {
                    case 0: // Primera posición - primer carácter
                        switch(colorCodes[codigo_actual].char1) {
                            case 'A': mostrarCaracter(charA, 0); break;
                            case 'B': mostrarCaracter(charB, 0); break;
                            case 'C': mostrarCaracter(charC, 0); break;
                            case 'D': mostrarCaracter(charD, 0); break;
                            case 'E': mostrarCaracter(charE, 0); break;
                            case 'F': mostrarCaracter(charF, 0); break;
                            default: mostrarDigito(0, 0); break;
                        }
                        break;
                    case 1: // Segunda posición - segundo carácter
                        switch(colorCodes[codigo_actual].char2) {
                            case 'A': mostrarCaracter(charA, 1); break;
                            case 'B': mostrarCaracter(charB, 1); break;
                            case 'C': mostrarCaracter(charC, 1); break;
                            case 'D': mostrarCaracter(charD, 1); break;
                            case 'E': mostrarCaracter(charE, 1); break;
                            case 'F': mostrarCaracter(charF, 1); break;
                            default: mostrarDigito(0, 1); break;
                        }
                        break;
                    case 2: // Tercera posición - primer dígito
                        mostrarDigito(colorCodes[codigo_actual].digit1, 2);
                        break;
                    case 3: // Cuarta posición - segundo dígito
                        mostrarDigito(colorCodes[codigo_actual].digit2, 3);
                        break;
                }
                
                // Avanzar a la siguiente posición para la próxima iteración
                display_pos = (display_pos + 1) % 4;
                
                // Si acabamos de cambiar un código y terminamos de mostrarlo,
                // reiniciar la bandera
                if (display_pos == 0 && mostrar_codigo_flag) {
                    mostrar_codigo_flag = 0;
                }
            }
            else {
                // No se detectó color, mostrar ceros
                mostrarDigito(0, 0);
                mostrarDigito(0, 1);
                mostrarDigito(0, 2);
                mostrarDigito(0, 3);
            }
        }
        else { // Modo contador normal
            // Multiplexado estándar del contador
            mostrarDigito(miles, 0);
            mostrarDigito(centenas, 1);
            mostrarDigito(decenas, 2);
            mostrarDigito(unidades, 3);
        }
    }
}