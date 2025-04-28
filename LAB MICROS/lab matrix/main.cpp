#include <stdio.h>
#include "stm32f7xx.h"
#include <string.h>

// LCD Definitions
#define LCD_DATA_PORT     GPIOD
#define LCD_CONTROL_PORT  GPIOF
#define LCD_RS           0    // PF0: Register Select
#define LCD_EN           1    // PF1: Enable

// LCD Pinout
// Data pins (8-bit mode):
// D0 - PD0
// D1 - PD1
// D2 - PD2
// D3 - PD3
// D4 - PD4
// D5 - PD5
// D6 - PD6
// D7 - PD7
// Control pins:
// RS - PF0 (0=Command, 1=Data)
// EN - PF1 (Enable signal)
// RW - GND (We're only writing to the LCD)

// LCD message buffers
static char fila3[16] = {'U', 'L', 'T', ':', ' ', ' ', ' ', ' ', 'c', 'm', ' ', ' ', ' ', ' ', ' '};
static char fila2[16] = {'D', 'I', 'S', ':', ' ', ' ', ' ', ' ', 'c', 'm', ' ', ' ', ' ', ' ', ' '};
static char fila1[16] = {'V', 'O', 'L', ':', ' ', ' ', ' ', ' ', 'c', 'm', '3', ' ', ' ', ' ', ' '};

// Variables globales para control de estados
static uint8_t i;
static uint8_t button_state = 0;  // Variable para seguimiento del estado del botón
static uint8_t prev_button_state = 0; // Estado anterior del botón para detectar cambios
static uint8_t debounce_count = 0; // Contador para anti-rebote
static unsigned char d;    // Variable para almacenar el carácter recibido por USART
static char text[20];      // Buffer para mensajes a enviar por USART

// Agregar después de las variables globales existentes
#define MAX_MSG_LENGTH 32
static char message_buffer[MAX_MSG_LENGTH];
static uint8_t message_index = 0;

// Matriz de caracteres del teclado (4x4)
const char keymap[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

/**
 * @brief Función de espera basada en SysTick con precisión de ciclos
 * @param n Número de ciclos a esperar
 */
static void SysTick_Wait(uint32_t n) {
    SysTick->LOAD = n - 1;
    SysTick->VAL = 0;
    while((SysTick->CTRL & 0x00010000) == 0);
}

static void SysTick_Wait1ms(uint32_t delay) {
    for(uint32_t i = 0; i < delay; i++) {
        SysTick_Wait(16000);  // 16MHz clock / 1000 = 16000 cycles per ms
    }
}

static void SysTick_ms(uint32_t delay) {
    SysTick_Wait1ms(delay);
}

static void Enable(void) {
    GPIOF->ODR |= (1 << LCD_EN);
    SysTick_Wait1ms(20);
    GPIOF->ODR &= ~(1 << LCD_EN);
    SysTick_Wait1ms(1);
}

static void Command(uint8_t cmd) {
    GPIOD->ODR = cmd;
    GPIOF->ODR &= ~(1 << LCD_RS);
    Enable();
}

static void Escribir(char data) {
    GPIOD->ODR = data;
    GPIOF->ODR |= (1 << LCD_RS);
    Enable();
}

static void LCD_SendString(const char X[]) {
    for (int i = 0; i < 16; i++) {
        Escribir(X[i]);
    }
}

static void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t address = row == 0 ? 0x80 + col : (row == 1 ? 0xC0 + col : 0x94 + col);
    Command(address);
}

static void LCD_Init(void) {
    SysTick_Wait1ms(20);
    Command(0x38);  // 8-bit, 3 líneas, 5x7
    Command(0x0C);  // Display on, cursor off
    Command(0x01);  // Clear display
    SysTick_Wait1ms(2);
    Command(0x06);  // Increment cursor
    Command(0x80);  // Primera línea
}

static char scan_keypad(void) {
    static uint8_t last_key_state = 0xFF;
    char key = 0;
    
    uint8_t row_pins[4] = {2, 4, 5, 6};
    uint8_t col_pins[4] = {8, 9, 10, 11};
    
    for (int row = 0; row < 4; row++) {
        for (int i = 0; i < 4; i++) {
            GPIOB->BSRR = (1UL << row_pins[i]);
        }
        GPIOB->BSRR = (1UL << (row_pins[row] + 16));
        
        for (volatile int i = 0; i < 500; i++);
        
        for (int col = 0; col < 4; col++) {
            if ((GPIOB->IDR & (1UL << col_pins[col])) == 0) {
                uint8_t current_key = (row << 4) | col;
                if (current_key != last_key_state) {
                    last_key_state = current_key;
                    key = keymap[row][col];
                    return key;
                }
            }
        }
    }
    last_key_state = 0xFF;
    return 0;
}

static void USART_SendChar(char c) {
    USART3->TDR = c;
    while(((USART3->ISR & 0x80) >> 7) == 0);
}

static void USART_SendString(const char* str) {  // Changed to const char*
    for(uint32_t j = 0; j < strlen(str); j++) {  // Changed to uint32_t
        USART_SendChar(str[j]);
    }
    USART_SendChar('\r');
}

static void send_message(void) {
    if (message_index > 0) {
        message_buffer[message_index] = '\0';
        USART_SendString((const char*)message_buffer);  // Added cast
    }
}

int main() {
    //-------------------------------------------------------------------------
    // Configuración de GPIOs
    //-------------------------------------------------------------------------
    RCC->AHB1ENR |= ((1<<1)|(1<<2));  // Habilitamos relojes para GPIOB y GPIOC
    
    // Configurar pines para teclado matricial
    // Filas: PB2, PB4, PB5, PB6 como salidas
    GPIOB->MODER &= ~((3UL << (2 * 2)) | (3UL << (2 * 4)) | 
                      (3UL << (2 * 5)) | (3UL << (2 * 6)));
    GPIOB->MODER |= ((1UL << (2 * 2)) | (1UL << (2 * 4)) | 
                     (1UL << (2 * 5)) | (1UL << (2 * 6)));

    // Columnas: PB8, PB9, PB10, PB11 como entradas con pull-up
    GPIOB->MODER &= ~((3UL << (2 * 8)) | (3UL << (2 * 9)) | 
                      (3UL << (2 * 10)) | (3UL << (2 * 11)));
    GPIOB->PUPDR &= ~((3UL << (2 * 8)) | (3UL << (2 * 9)) | 
                      (3UL << (2 * 10)) | (3UL << (2 * 11)));
    GPIOB->PUPDR |= ((1UL << (2 * 8)) | (1UL << (2 * 9)) | 
                     (1UL << (2 * 10)) | (1UL << (2 * 11)));

    // Add LCD GPIO configuration
    GPIOD->MODER |= 0x00005555;    // Configure GPIOD pins for LCD data
    GPIOD->OTYPER &= ~0x000000;    // Push-pull configuration
    GPIOF->MODER |= 0x00000005;    // Configure GPIOF pins for LCD control
    GPIOF->PUPDR &= ~(0x0000000F); // No pull-up/pull-down

    //-------------------------------------------------------------------------
    // Configuración del SysTick
    //-------------------------------------------------------------------------
    SysTick->CTRL = 0;                        // Disable SysTick
    SysTick->LOAD = 16000 - 1;               // Set reload value for 1ms
    SysTick->VAL = 0;                        // Clear current value
    SysTick->CTRL = 0x00000005;              // Enable SysTick, no interrupts, processor clock

    //-------------------------------------------------------------------------
    // Configuración USART3 
    //-------------------------------------------------------------------------
    RCC->AHB1ENR |= (1<<3);              // Habilitamos reloj para GPIOD
    GPIOD->MODER &= ~((0b11<<18)|(0b11<<16));  // Limpiamos los bits de modo para PD9 y PD8
    GPIOD->MODER |= (1<<19)|(1<<17);     // Configuramos PD9 y PD8 como función alternativa (10)
    GPIOD->AFR[1] &= ~((0b1111<<4)|(0b1111<<0));  // Limpiamos los bits de función alternativa
    GPIOD->AFR[1] |= (0b111<<4)|(0b111<<0);    // Establecemos USART3 (AF7) como función alternativa
    
    RCC->APB1ENR |= (1<<18);            // Habilitamos reloj para USART3
    USART3->BRR = 0x683;                // Configuramos baudrate a 9600 para reloj de 16 MHz (HSI)
    USART3->CR1 |= ((1<<5)|(0b11<<2));  // Habilitamos interrupción RXNE, transmisor y receptor
    USART3->CR1 |= (1<<0);              // Habilitamos USART
    NVIC_EnableIRQ(USART3_IRQn);        // Habilitamos la interrupción USART3 en el NVIC
    
    // Enviamos mensaje inicial
    USART_SendString((const char*)"STM32 Ready");  // Fixed string literal warning
    
    // Initialize LCD
    LCD_Init();

    // Show initial messages
    LCD_SendString(fila1);
    LCD_SetCursor(1, 0);
    LCD_SendString(fila2);
    LCD_SetCursor(2, 0);
    LCD_SendString(fila3);

    while(1){
        // Manejar entrada del teclado
        char pressed_key = scan_keypad();
        if (pressed_key != 0) {
            if (pressed_key == '#' && message_index > 0) {
                send_message();
                message_index = 0;
                for (int i = 0; i < MAX_MSG_LENGTH; i++) {
                    message_buffer[i] = '\0';
                }
            }
            else if (pressed_key != '#' && message_index < MAX_MSG_LENGTH - 1) {
                message_buffer[message_index++] = pressed_key;
            }
            for (volatile int i = 0; i < 300000; i++); // debouncing
        }

        // Update LCD with keyboard input
        if (pressed_key != 0) {
            LCD_SetCursor(0, message_index);
            Escribir(pressed_key);
        }
        
        SysTick_ms(10);
    }
}