#include <stm32f767xx.h>
#include <stdio.h>
#include <math.h>

volatile uint32_t tiempo = 0;
volatile double Distancia = 0;
volatile double DistanciaFiltrada = 0;  // Para aplicar un filtro
volatile double DistanciaFiltrada2 = 0;  // Para aplicar un filtro
volatile double Volumen = 0;  // Variable para almacenar el volumen
char C5[16] = {'V', 'O', 'L', ' ', ':', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '}; // Para volumen
char C6[16] = {'E', 'N', ' ', 'C', 'M', '3', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};

// Declaración de funciones
void SysTick_Init(void);
void SysTick_Wait(uint32_t n);
void SysTick_Wait1ms(uint32_t delay);
void Enable(void);
void floatToString(float value, char* str, int precision);
void LCD_Init(void);
void Command(uint8_t cmd);
void Escribir(char data);
void LCD_SendString(const char X[]);
void LCD_SetCursor(uint8_t row, uint8_t col);
void UpdateDistancia(float temp);
void UpdateVolumen(float volumen);
void TriggerSensor(void);
uint32_t MeasurePulseWidth(void);
void TIM2_Init(void);
void TIM2_Start(void);
void TIM2_Stop(void);

int main() {
    // Habilitar el FPU para operaciones en coma flotante
    SCB->CPACR |= (0xF << 20);  // Habilitar el acceso completo al coprocesador de coma flotante (FPU)

    // 1. Habilitar el reloj para GPIOA, GPIOC, GPIOD y GPIOF
    RCC->AHB1ENR |= (1<<0) | (1<<2) | (1<<3) | (1<<5);
    
    // 2. Configurar los pines para el sensor ultrasonido: Trigger en PA3 y Echo en PA0
    GPIOA->MODER |= (1<<6);  // PA3 como salida (Trigger)
    GPIOA->MODER &= ~(3<<0); // PA0 como entrada (Echo)
    
    // 3. Configuración de pines para la LCD
    GPIOD->MODER |= 0x00005555;  // Configurar PD0-PD7 como salida
    GPIOD->OTYPER &= ~(0xFF);    // Configurar PD0-PD7 como push-pull
    GPIOF->MODER |= 0x00000005;  // Configurar PF0 y PF1 como salida
    GPIOF->PUPDR &= ~(0x0000000F);  // Sin resistencias pull-up/pull-down

    // 4. Inicializar SysTick
    SysTick_Init();

    // 5. Inicializar la LCD y escribir el texto estático
    LCD_Init();
    LCD_SendString(C5);
    LCD_SetCursor(1, 0);  // Mover el cursor a la segunda línea
    LCD_SendString(C6);

    // 6. Inicializar el temporizador TIM2
    TIM2_Init();

    while (1) {
        // 7. Activar el sensor y medir el tiempo de pulso
        TriggerSensor();
        tiempo = MeasurePulseWidth();
        
        // Limitar el tiempo máximo para evitar mediciones excesivas
        if (tiempo > 38000) {  // Más de 38ms (~6 metros)
            tiempo = 38000;  // Limitar a un valor razonable
        }
        
        // 8. Calcular la distancia en base al tiempo (tiempo en µs, velocidad del sonido 0.0343 cm/µs)
        Distancia = 22.8 - ((tiempo * 0.00343) / 3.6);  // Dividir entre 3.2 porque es el tiempo de ida y vuelta
        
        // **Aplicar filtro de rango (ignorar valores fuera de rango razonable)**
        if (Distancia < 2.0 || Distancia > 400.0) {
            Distancia = 0.0;  // Si está fuera del rango de 2 cm a 400 cm, ignorar
        }

        // 9. Aplicar un filtro simple (95% del valor anterior)
        DistanciaFiltrada = (DistanciaFiltrada * 0.95) + (Distancia * 0.05);

        // 10. Redondear la distancia filtrada a 1 decimal
        DistanciaFiltrada = round(DistanciaFiltrada * 10) / 10.0;

        // 11. Calcular el volumen usando la fórmula: Volumen = 176.72 * Distancia
        Volumen = (44.179 * DistanciaFiltrada);


        // 12. Actualizar la distancia y el volumen en la pantalla LCD
        UpdateVolumen(Volumen);

        // 13. Esperar antes de la próxima lectura
        SysTick_Wait1ms(50);
    }
}

// -------------------------------------------
// Funciones auxiliares
// -------------------------------------------

void SysTick_Init(void) {
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL = 0x00000005;
}

void SysTick_Wait(uint32_t n) {
    SysTick->LOAD = n - 1;
    SysTick->VAL = 0;
    while ((SysTick->CTRL & 0x00010000) == 0);
}

void SysTick_Wait1ms(uint32_t delay) {
    for (uint32_t i = 0; i < delay; i++) {
        SysTick_Wait(16000);  // Ajustar según la frecuencia del sistema
    }
}

void Enable(void) {
    GPIOF->ODR |= 0x2;  // Set EN high
    SysTick_Wait1ms(20);
    GPIOF->ODR &= ~0x2;  // Set EN low
    SysTick_Wait1ms(1);
}

void Command(uint8_t cmd) {
    GPIOD->ODR = cmd;  // Enviar byte completo
    GPIOF->ODR &= ~0x1;  // RS = 0 para comando
    Enable();
}

void Escribir(char data) {
    GPIOD->ODR = data;  // Enviar byte completo
    GPIOF->ODR |= 0x1;  // RS = 1 para datos
    Enable();
}

void LCD_SendString(const char X[]) {
    for (int i = 0; i < 16; i++) {
        Escribir(X[i]);
    }
}

void floatToString(float value, char* str, int precision) {
    int intPart = (int)value;
    int decPart = (int)((value - intPart) * pow(10, precision));
    sprintf(str, "%d.%0*d", intPart, precision, decPart);
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t address;
    if (row == 0) {
        address = 0x80 + col;  // Primera fila
    } else {
        address = 0xC0 + col;  // Segunda fila
    }
    Command(address);
}

//void UpdateDistancia(float temp) {
//    char tempStr[5];
//    floatToString(temp, tempStr, 1);  // Ajustar la precisión a 1 decimal
//    
//    LCD_SetCursor(0, 7);  // Mover el cursor a la posición de la distancia
//    for (int i = 0; i < 5; i++) {
//        if (tempStr[i] != '\0') {
//            Escribir(tempStr[i]);
//        } else {
//            Escribir(' ');  // Rellenar con espacios si el número es más corto
//        }
//    }
//}

void UpdateVolumen(float volumen) {
    char volumenStr[10];
    floatToString(volumen, volumenStr, 1);  // Ajustar la precisión a 1 decimal
    
    LCD_SetCursor(0, 7);  // Mover el cursor a la posición del volumen
    for (int i = 0; i < 10; i++) {
        if (volumenStr[i] != '\0') {
            Escribir(volumenStr[i]);
        } else {
            Escribir(' ');  // Rellenar con espacios si el número es más corto
        }
    }
}

// Función para enviar el pulso de Trigger
void TriggerSensor(void) {
    GPIOA->ODR |= (1<<3);  // Trigger high en PA3
    SysTick_Wait1ms(1);    // Esperar al menos 10 µs
    GPIOA->ODR &= ~(1<<3); // Trigger low en PA3
}

// Función para medir el ancho del pulso de Echo
uint32_t MeasurePulseWidth(void) {
    uint32_t timeout = 38000;  // Tiempo de espera máximo (en µs)
    uint32_t startTime, pulseWidth;
    
    // Esperar a que Echo sea alto (inicio del pulso), con un timeout
    while (!(GPIOA->IDR & (1<<0))) {
        if (timeout-- == 0) {
            return 38000;  // Devolver el tiempo máximo si hay timeout
        }
    }
    
    // Iniciar el temporizador
    TIM2_Start();

    // Esperar a que Echo sea bajo (fin del pulso), con un timeout
    timeout = 38000;
    while (GPIOA->IDR & (1<<0)) {
        if (timeout-- == 0) {
            TIM2_Stop();
            return 38000;  // Devolver el tiempo máximo si hay timeout
        }
    }

    // Detener el temporizador
    TIM2_Stop();

    // Leer el tiempo transcurrido
    pulseWidth = TIM2->CNT;

    // Proteger contra valores erráticos
    if (pulseWidth > 38000 || pulseWidth < 58) {  // Tiempo mínimo ~2 cm
        return 0;  // Valor inválido, fuera de rango
    }

    return pulseWidth;
}

// Inicializar TIM2 para medir tiempo de pulso
void TIM2_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // Habilitar el reloj de TIM2
    TIM2->PSC = 15;  // Prescaler para 1 MHz (1 µs por conteo)
    TIM2->CR1 = 0;  // Resetear configuración del temporizador
}

// Iniciar el temporizador TIM2
void TIM2_Start(void) {
    TIM2->CNT = 0;  // Reiniciar el contador
    TIM2->CR1 |= TIM_CR1_CEN;  // Habilitar el contador
}

// Detener el temporizador TIM2
void TIM2_Stop(void) {
    TIM2->CR1 &= ~TIM_CR1_CEN;  // Detener el contador
}

void LCD_Init(void) {
    SysTick_Wait1ms(50);  // Esperar para inicialización de la LCD
    Command(0x38);  // Modo 8-bit, 2 líneas, fuente 5x7
    Command(0x0C);  // Display on, cursor off, sin parpadeo
    Command(0x06);  // Entrada automática, sin desplazamiento de display
    Command(0x01);  // Limpiar display
    SysTick_Wait1ms(10);
}
