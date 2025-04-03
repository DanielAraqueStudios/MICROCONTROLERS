#include <stm32f767xx.h>
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define FILTER_ALPHA 0.05  // Factor de suavizado para el filtro exponencial
#define TANK_HEIGHT 22.1  // Altura total del tanque en cm
#define TANK_RADIUS 3.75  // Radio del tanque en cm (5.5 cm de diámetro)

// Variables globales
volatile double codigo_adc = 0;
volatile double volt_sharp = 0;
volatile float altura_sharp = 0;
volatile float volumen_sharp = 0;
volatile float filteredDistance_sharp = 0;

volatile uint32_t tiempo_ultra = 0;
volatile double Distancia_ultra = 0;
volatile float Volumen_ultra = 0;

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
void UpdateVolumen(float vol);
float calculateDistance(float voltage);
float getFilteredDistance(float newReading);
void TIM2_Init(void);
void TriggerSensor(void);
uint32_t MeasurePulseWidth(void);

// Modificar los mensajes LCD para mostrar distancia en lugar de volumen
char fila1[16] = {'U', 'L', 'T', ':', ' ', ' ', ' ', ' ', 'c', 'm', ' ', ' ', ' ', ' ', ' '};
char fila2[16] = {'S', 'H', 'P', ':', ' ', ' ', ' ', ' ', 'c', 'm', ' ', ' ', ' ', ' ', ' '};

int main() {
    // Configuración inicial común
    RCC->AHB1ENR |= (1<<0)|(1<<2)|(1<<3)|(1<<5);  // GPIOA, GPIOC, GPIOD, GPIOF

    // Configurar pines
    GPIOA->MODER |= 0x3;        // PA0 como analógico (Sharp)
    GPIOA->MODER |= (1<<6);     // PA3 como salida (Trigger)
    GPIOA->MODER &= ~(3<<4);    // PA2 como entrada (Echo)

    // Configuración LCD
    GPIOD->MODER |= 0x00005555;  // Configurar PD0-PD7 como salida 
    GPIOD->OTYPER &= ~0x000000;  // Configurar PD0-PD7 como push-pull
    GPIOF->MODER |= 0x00000005;  // Configurar PF0 y PF1 como salida
    GPIOF->PUPDR &= ~(0x0000000F);  // Sin resistencias pull-up/pull-down

    // Configuración ADC
    RCC->APB2ENR |= 0x700;      // Habilitar ADC1,2,3
    ADC1->CR1 |= (2<<24);       // Configuración ADC
    ADC1->SMPR2 |= (5<<0);      // Muestreo
    ADC1->CR2 |= 0x3;           // Habilitar ADC

    // Inicializaciones
    SysTick_Init();
    LCD_Init();
    TIM2_Init();

    // Mostrar mensajes iniciales
    LCD_SendString(fila1);
    LCD_SetCursor(1, 0);
    LCD_SendString(fila2);

    while(1) {
        // Leer Sharp
        ADC1->CR2 |= 1 << 30;
        while ((ADC1->SR & (1 << 1)) == 0);
        codigo_adc = ADC1->DR;
        volt_sharp = (codigo_adc) * (3.3) / (255.0);
        float distance_sharp = calculateDistance(volt_sharp);
        filteredDistance_sharp = getFilteredDistance(distance_sharp);

        // Validación de rango para el Sharp
        if (filteredDistance_sharp < 4.0 || filteredDistance_sharp > 30.0) {
            // Si está fuera de rango, mantener la última lectura válida
            filteredDistance_sharp = getFilteredDistance(filteredDistance_sharp);
        }

        altura_sharp = TANK_HEIGHT - filteredDistance_sharp;
        if (altura_sharp < 0) altura_sharp = 0;
        if (altura_sharp > TANK_HEIGHT) altura_sharp = TANK_HEIGHT;
        volumen_sharp = M_PI * pow(TANK_RADIUS, 2) * altura_sharp;

        // Leer Ultrasonido
        TriggerSensor();
        tiempo_ultra = MeasurePulseWidth();
        if (tiempo_ultra > 38000) tiempo_ultra = 38000;
        Distancia_ultra = (tiempo_ultra * 0.0343) / 2.0;
        Volumen_ultra = 7.0 * Distancia_ultra;
        if (Volumen_ultra < 0) Volumen_ultra = 0;

        // Actualizar LCD con distancias en lugar de volúmenes
        LCD_SetCursor(0, 5);
        UpdateVolumen(Distancia_ultra);  // Mostrar distancia del ultrasonido
        LCD_SetCursor(1, 5);
        UpdateVolumen(filteredDistance_sharp);  // Mostrar distancia del Sharp

        SysTick_Wait1ms(100);
    }
}

float calculateDistance(float voltage) {
    // Ecuación para el sensor Sharp GP2Y0A41SK0F ajustada
    // Requiere calibración, revisa el datasheet para la curva de voltaje/distancia
    return 15.08 * pow(voltage, -1.058); // Ajusta el +1.0 según tus pruebas
}

float getFilteredDistance(float newReading) {
    static float lastFiltered = 0;
    
    if (lastFiltered == 0) {
        lastFiltered = newReading;
    }
    
    // Aplicar filtro exponencial simple
    lastFiltered = (FILTER_ALPHA * newReading) + ((1 - FILTER_ALPHA) * lastFiltered);
    return lastFiltered;
}

void SysTick_Init(void) {
    SysTick->LOAD = 0x00FFFFFF;  // Cargar el valor máximo
    SysTick->CTRL = 0x00000005;  // Habilitar el contador
}

void SysTick_Wait(uint32_t n) {
    SysTick->LOAD = n - 1;  // Cargar el valor de espera
    SysTick->VAL = 0;  // Reiniciar el contador
    while ((SysTick->CTRL & 0x00010000) == 0);  // Esperar a que se complete la espera
}

void SysTick_Wait1ms(uint32_t delay) {
    for (uint32_t i = 0; i < delay; i++) {
        SysTick_Wait(16000);  // Esperar 1 ms
    }
}

void Enable(void) {
    GPIOF->ODR |= 0x2;  // Set EN high
    SysTick_Wait1ms(20);  // Esperar
    GPIOF->ODR &= ~0x2;  // Set EN low
    SysTick_Wait1ms(1);  // Esperar
}

void Command(uint8_t cmd) {
    GPIOD->ODR = cmd;  // Enviar el comando completo
    GPIOF->ODR &= ~0x1;  // RS = 0 para comando
    Enable();  // Habilitar
}

void Escribir(char data) {
    GPIOD->ODR = data;  // Enviar el dato completo
    GPIOF->ODR |= 0x1;  // RS = 1 para dato
    Enable();  // Habilitar
}

void LCD_SendString(const char X[]) {
    for (int i = 0; i < 16; i++) {
        Escribir(X[i]);  // Enviar cada carácter
    }
}

void floatToString(float value, char* str, int precision) {
    int intPart = (int)value;  // Parte entera
    int decPart = (int)((value - intPart) * pow(10, precision));  // Parte decimal
    sprintf(str, "%d.%0*d", intPart, precision, decPart);  // Formatear el número
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t address;
    if (row == 0) {
        address = 0x80 + col;  // Primera fila
    } else {
        address = 0xC0 + col;  // Segunda fila
    }
    Command(address);  // Enviar comando de posición
}

void UpdateVolumen(float vol) {
    char volStr[8];
    floatToString(vol, volStr, 1);
    
    for (int i = 0; i < 6; i++) {
        if (volStr[i] != '\0') {
            Escribir(volStr[i]);
        } else {
            Escribir(' ');
        }
    }
}

void LCD_Init(void) {
    SysTick_Wait1ms(20);  // Esperar 20 ms
    Command(0x38);  // Inicializar en modo de 8 bits, 2 líneas, 5x7
    Command(0x0C);  // Encender el display y el cursor
    Command(0x01);  // Limpiar el display
    SysTick_Wait1ms(2);  // Esperar 2 ms
    Command(0x06);  // Desplazar a la derecha al escribir
}

void TIM2_Init(void) {
    RCC->APB1ENR |= (1<<0);  // Habilitar reloj para TIM2
    TIM2->PSC = 16 - 1;      // Prescaler
    TIM2->ARR = 0xFFFFFFFF;  // Auto-reload máximo
    TIM2->CR1 |= (1<<0);     // Habilitar contador
}

void TriggerSensor(void) {
    GPIOA->ODR |= (1<<3);  // Activar Trigger
    SysTick_Wait1ms(1);    // Esperar 10 us
    GPIOA->ODR &= ~(1<<3); // Desactivar Trigger
}

uint32_t MeasurePulseWidth(void) {
    uint32_t start = 0, end = 0;
    while (!(GPIOA->IDR & (1<<2)));  // Esperar a que Echo sea alto
    start = TIM2->CNT;               // Capturar tiempo inicial
    while (GPIOA->IDR & (1<<2));     // Esperar a que Echo sea bajo
    end = TIM2->CNT;                 // Capturar tiempo final
    return end - start;              // Calcular ancho de pulso
}
