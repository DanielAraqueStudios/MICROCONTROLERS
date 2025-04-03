#include <stm32f767xx.h>
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Definiciones de pines LCD
#define LCD_DATA_PORT     GPIOD   // PD0-PD7: Bus de datos de 8 bits
#define LCD_CONTROL_PORT  GPIOF   // Puerto de control
#define LCD_RS           0        // PF0: Register Select (0=Comando, 1=Datos)
#define LCD_EN           1        // PF1: Enable

// Definiciones de pines ADC
#define ADC_PIN_PORT     GPIOA   // Puerto para el ADC
#define ADC_PIN          0       // PA0: Pin para el sensor Sharp
#define ADC_RESOLUTION   255.0   // Resolución de 8 bits
#define ADC_VREF         3.3     // Voltaje de referencia

#define FILTER_ALPHA 0.05  // Factor de suavizado para el filtro exponencial
#define TANK_HEIGHT 22.1  // Altura total del tanque en cm
#define TANK_RADIUS 3.75  // Radio del tanque en cm (5.5 cm de diámetro)

// Variables globales
volatile double codigo = 0;
volatile double volt = 0;
volatile float altura = 0;
volatile float volumen = 0;
volatile float filteredDistance = 0;  // Distancia filtrada

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
void UpdateDisplay(float vol);
float calculateDistance(float voltage);
float getFilteredDistance(float newReading);

char fila1[16] = {'V', '(', 'c', 'm', '3', ')', ':',' ',' ',' ',' ',' ',' ',' ',' '};

int main() {
    // Configuración del ADC
    RCC->AHB1ENR |= (1<<0);     // Habilitar reloj para GPIOA
    ADC_PIN_PORT->MODER |= (3<<(ADC_PIN*2));  // Configurar PA0 como analógico
    
    // Configuración ADC1
    RCC->APB2ENR |= (1<<8);     // Habilitar reloj para ADC1
    ADC->CCR &= ~(3<<16);       // ADCPR = 00: No prescaler
    ADC1->CR1 &= ~((3<<24)|(1<<8)); // Resolución 8 bits
    ADC1->SMPR2 |= (7<<0);      // 480 ciclos de muestreo para canal 0
    ADC1->SQR3 &= ~(0x1F);      // Canal 0 para conversión
    ADC1->CR2 |= (1<<0);        // Habilitar ADC

    // Configuración de registros para pines
    LCD_DATA_PORT->MODER |= 0x00005555;  // Configurar PD0-PD7 como salida 
    LCD_DATA_PORT->OTYPER &= ~0x000000;  // Configurar PD0-PD7 como push-pull
    LCD_CONTROL_PORT->MODER |= 0x00000005;  // Configurar PF0 y PF1 como salida
    LCD_CONTROL_PORT->PUPDR &= ~(0x0000000F);  // Sin resistencias pull-up/pull-down

    SysTick_Init();
    LCD_Init();
    LCD_SendString(fila1);

    while (1) {
        // Leer el ADC
        ADC1->CR2 |= 1 << 30;  // Iniciar conversión
        while ((ADC1->SR & (1 << 1)) == 0) {
            // Esperar a que la conversión termine
        }
        
        codigo = ADC1->DR;  // Leer el valor del ADC
        volt = (codigo) * (ADC_VREF) / (ADC_RESOLUTION);  // Convertir a voltaje
        
        float distance = calculateDistance(volt);  // Calcular distancia
        filteredDistance = getFilteredDistance(distance);  // Aplicar filtrado
        
        // Calcular la altura del líquido (sensor en la parte superior)
        altura = TANK_HEIGHT - filteredDistance;
        if (altura < 0) altura = 0;  // Asegurar que no sea negativa
        if (altura > TANK_HEIGHT) altura = TANK_HEIGHT;  // Asegurar que no exceda la altura del tanque

        // Calcular el volumen en cm³
        volumen = M_PI * pow(TANK_RADIUS, 2) * altura;  // Volumen en cm³

        // Actualizar la pantalla LCD solo con el volumen
        UpdateDisplay(volumen);

        SysTick_Wait1ms(400);  // Esperar 1 segundo entre lecturas
    }
}

float calculateDistance(float voltage) {
    // Ecuación para el sensor Sharp GP2Y0A41SK0F ajustada
    // Requiere calibración, revisa el datasheet para la curva de voltaje/distancia
    return 15.08 * pow(voltage, -1.058); // Ajusta el +1.0 según tus pruebas
}

float getFilteredDistance(float newReading) {
    // Aplicar un filtro exponencial
    if (filteredDistance == 0) {
        filteredDistance = newReading;  // Inicializar la primera lectura
    }
    filteredDistance = (FILTER_ALPHA * newReading) + ((1 - FILTER_ALPHA) * filteredDistance);  // Filtro exponencial
    return filteredDistance;
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
    LCD_CONTROL_PORT->ODR |= (1 << LCD_EN);  // Set EN high
    SysTick_Wait1ms(20);  // Esperar
    LCD_CONTROL_PORT->ODR &= ~(1 << LCD_EN);  // Set EN low
    SysTick_Wait1ms(1);  // Esperar
}

void Command(uint8_t cmd) {
    LCD_DATA_PORT->ODR = cmd;  // Enviar el comando completo
    LCD_CONTROL_PORT->ODR &= ~(1 << LCD_RS);  // RS = 0 para comando
    Enable();  // Habilitar
}

void Escribir(char data) {
    LCD_DATA_PORT->ODR = data;  // Enviar el dato completo
    LCD_CONTROL_PORT->ODR |= (1 << LCD_RS);  // RS = 1 para dato
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

void UpdateDisplay(float vol) {
    char volStr[8];

    floatToString(vol, volStr, 1);  // Formatear el volumen con 1 decimal
    
    // Actualizar la primera fila con el volumen
    LCD_SetCursor(0, 6);
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
