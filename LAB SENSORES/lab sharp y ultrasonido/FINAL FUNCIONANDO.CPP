#include <stm32f767xx.h>
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Definiciones para Sharp y Tanque
#define FILTER_ALPHA 0.05
#define TANK_HEIGHT 22.1
#define TANK_RADIUS 4.79  // Radio = Diámetro/2 = 9.58/2 = 4.79

#define BUFFER_SIZE 10

// Definiciones para LCD
#define LCD_DATA_PORT     GPIOD
#define LCD_CONTROL_PORT  GPIOF
#define LCD_RS           0
#define LCD_EN           1

// Variables globales Sharp
volatile double codigo_adc = 0;
volatile double volt_sharp = 0;
volatile float filteredDistance_sharp = 0;

// Variables globales Ultrasonido
volatile uint32_t tiempo_ultra = 0;
volatile double Distancia_ultra = 0;
volatile double volumen_tanque = 0;

volatile double volumen1 =0;

// Mensajes LCD (modificados para 3 líneas)
char fila3[16] = {'U', 'L', 'T', ':', ' ', ' ', ' ', ' ', 'c', 'm', ' ', ' ', ' ', ' ', ' '};
char fila2[16] = {'D', 'I', 'S', ':', ' ', ' ', ' ', ' ', 'c', 'm', ' ', ' ', ' ', ' ', ' '};
char fila1[16] = {'V', 'O', 'L', ':', ' ', ' ', ' ', ' ', 'c', 'm', '3', ' ', ' ', ' ', ' '};

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
void UpdateDisplay(float value);
float calculateDistance(float voltage);
float getFilteredDistance(float newReading);
void TIM2_Init(void);
void TriggerSensor(void);
uint32_t MeasurePulseWidth(void);
void TIM2_Start(void);
void TIM2_Stop(void);

int main() {
    // Habilitar el FPU
    SCB->CPACR |= (0xF << 20);

    // Configuración de relojes
    RCC->AHB1ENR |= (1<<0)|(1<<2)|(1<<3)|(1<<5);

    // Configurar pines
    GPIOA->MODER |= 0x3;
    GPIOA->MODER |= (1<<6);
    GPIOA->MODER &= ~(3<<4);

    // Configuración LCD
    GPIOD->MODER |= 0x00005555;
    GPIOD->OTYPER &= ~0x000000;
    GPIOF->MODER |= 0x00000005;
    GPIOF->PUPDR &= ~(0x0000000F);

    // Configuración ADC
    RCC->APB2ENR |= 0x700;
    ADC1->CR1 |= (2<<24);
    ADC1->SMPR2 |= (5<<0);
    ADC1->CR2 |= 0x3;

    // Inicializaciones
    SysTick_Init();
    LCD_Init();
    TIM2_Init();

    // Mostrar mensajes iniciales modificados
    LCD_SendString(fila1);
    LCD_SetCursor(1, 0);
    LCD_SendString(fila2);
    LCD_SetCursor(2, 0);
    LCD_SendString(fila3);

    while(1) {
        // Leer Sharp
        ADC1->CR2 |= 1 << 30;
        while ((ADC1->SR & (1 << 1)) == 0);
        codigo_adc = ADC1->DR;
        volt_sharp = (codigo_adc) * (3.3) / (255.0);


float distance_sharp = calculateDistance(volt_sharp);
filteredDistance_sharp = getFilteredDistance(distance_sharp);

// Aplicando la regresión directamente
filteredDistance_sharp = -0.191 * filteredDistance_sharp + 34.7;

// Leer Ultrasonido y calcular volumen
TriggerSensor();
tiempo_ultra = MeasurePulseWidth();
Distancia_ultra = (tiempo_ultra * 0.0343) / 2.0;
// Aplicando la regresión directamente
Distancia_ultra = -0.066 * Distancia_ultra + 38.0;

// Calcular volumen del cilindro (π × r² × h)
float altura_liquido = TANK_HEIGHT - Distancia_ultra; // Altura del líquido = altura total - distancia al líquido
if (altura_liquido < 0) altura_liquido = 0; // Verificar que no sea negativa
if (altura_liquido > TANK_HEIGHT) altura_liquido = TANK_HEIGHT;

// Calcular volumen en cm³ con mayor precisión
// Validar la distancia antes de calcular el volumen
if (Distancia_ultra < 0) {
    Distancia_ultra = 0;
} else if (Distancia_ultra > TANK_HEIGHT) {
    Distancia_ultra = TANK_HEIGHT;
}

// Buffer circular para almacenar 10 lecturas del ultrasonido
static float distance_buffer[BUFFER_SIZE] = {0};
static int buffer_index = 0;

// Almacenar la lectura actual en el buffer circular
distance_buffer[buffer_index] = Distancia_ultra;
buffer_index = (buffer_index + 1) % BUFFER_SIZE;

// Encontrar el valor máximo en el buffer
float max_distance = 0;
for (int i = 0; i < BUFFER_SIZE; i++) {
    if (distance_buffer[i] > max_distance) {
        max_distance = distance_buffer[i];
    }
}

// Usar el valor máximo como la distancia final
Distancia_ultra = max_distance;

// Usar la altura del líquido para el cálculo del volumen
volumen_tanque = M_PI * pow(TANK_RADIUS, 2) * Distancia_ultra;

// Actualizar LCD con la distancia y el volumen en las posiciones correctas
LCD_SetCursor(0, 5);
UpdateDisplay(volumen_tanque); // Primera línea: distancia ultrasónico
LCD_SetCursor(1, 5);
UpdateDisplay(Distancia_ultra); // Segunda línea: distancia sharp
LCD_SetCursor(2, 5);
UpdateDisplay(volumen_tanque); // Tercera línea: volumen calculado

        SysTick_Wait1ms(100);
    }
}

float calculateDistance(float voltage) {
    return 15.08 * pow(voltage, -1.058);
}

float getFilteredDistance(float newReading) {
    static float lastReading = 0;
    
    if (lastReading == 0) {
        lastReading = newReading;
        return newReading;
    }
    
    lastReading = (FILTER_ALPHA * newReading) + ((1 - FILTER_ALPHA) * lastReading);
    return lastReading;
}

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
        SysTick_Wait(16000);
    }
}

void Enable(void) {
    GPIOF->ODR |= (1 << LCD_EN);
    SysTick_Wait1ms(20);
    GPIOF->ODR &= ~(1 << LCD_EN);
    SysTick_Wait1ms(1);
}

void Command(uint8_t cmd) {
    GPIOD->ODR = cmd;
    GPIOF->ODR &= ~(1 << LCD_RS);
    Enable();
}

void Escribir(char data) {
    GPIOD->ODR = data;
    GPIOF->ODR |= (1 << LCD_RS);
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
    uint8_t address = row == 0 ? 0x80 + col : (row == 1 ? 0xC0 + col : 0x94 + col);
    Command(address);
}

void UpdateDisplay(float value) {
    char str[8];
    floatToString(value, str, 1);
    
    for (int i = 0; i < 6; i++) {
        if (str[i] != '\0') {
            Escribir(str[i]);
        } else {
            Escribir(' ');
        }
    }
}

void LCD_Init(void) {
    SysTick_Wait1ms(20);
    Command(0x38);  // 8-bit, 3 líneas, 5x7
    Command(0x0C);  // Display on, cursor off
    Command(0x01);  // Clear display
    SysTick_Wait1ms(2);
    Command(0x06);  // Increment cursor
    Command(0x80);  // Primera línea
}

void TIM2_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 15;
    TIM2->ARR = 0xFFFFFFFF;
    TIM2->CR1 |= (1<<0);
}

void TriggerSensor(void) {
    GPIOA->ODR |= (1<<3);
    SysTick_Wait1ms(1);
    GPIOA->ODR &= ~(1<<3);
}

void TIM2_Start(void) {
    TIM2->CNT = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
}

void TIM2_Stop(void) {
    TIM2->CR1 &= ~TIM_CR1_CEN;
}

uint32_t MeasurePulseWidth(void) {
    uint32_t timeout = 38000;
    uint32_t start = 0, end = 0;
    volatile uint32_t timeout_counter = 0;

    while (!(GPIOA->IDR & (1<<2))) {
        timeout_counter++;
        if (timeout_counter > 100000) {
            return 0;
        }
    }

    start = TIM2->CNT;
    timeout_counter = 0;

    while (GPIOA->IDR & (1<<2)) {
        timeout_counter++;
        if (timeout_counter > 100000) {
            end = TIM2->CNT;
            break;
        }
    }

    if (timeout_counter <= 100000) {
        end = TIM2->CNT;
    }

    uint32_t pulseWidth = end - start;

    if (pulseWidth > timeout || pulseWidth < 58) {
        return 0;
    }

    return pulseWidth;
}