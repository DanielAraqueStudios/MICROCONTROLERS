#include <stm32f767xx.h>

void USART2_Inicio(void);
void USART2_enviarChar(char c);
char USART2_recibirChar(void);
void LED_inicio(void);
void LED_Control(char c);

int main(void) {
    USART2_Inicio();
    LED_inicio();
    
    while (1) {
        
      USART2_enviarChar('J'); // Envía la letra 'D' al terminal
			USART2_enviarChar('U'); // Envía la letra 'a' al terminal
			USART2_enviarChar('A'); // Envía la letra 'n' al terminal
			USART2_enviarChar('N'); // Envía la letra 'i' al terminal
			USART2_enviarChar('N'); // Envía la letra 'e' al terminal
			USART2_enviarChar('I'); // Envía la letra 'l' al terminal
			USART2_enviarChar(' C'); // Envía la letra ' ' al terminal
			USART2_enviarChar('O'); // Envía la letra ' ' al terminal
       // for ( int i = 0; i < 1000000; i++); // Pequeña pausa
        
        char recibir = USART2_recibirChar(); // Leer un carácter del teclado
        LED_Control(recibir); // Controlar LED según la tecla recibida
    }
}

void USART2_Inicio(void) {
    // Habilitar el reloj para GPIOA y USART2
    RCC->AHB1ENR |= 0x1;
    RCC->APB1ENR |= 0x20000;
    
    // Configurar PA2 como salida alternativa (TX) y PA3 como entrada alternativa (RX)
    GPIOA->MODER |= 0xA0;  // Modo AF
    GPIOA->AFR[0] |= (7 << (4 * 2)) | (7 << (4 * 3)); // AF7 para USART2 TX y RX
    
    // Configurar UART2
    USART2->BRR = 16000000 / 9600; // Baudrate 9600 (APB1 a 16MHz)
    USART2->CR1 = 0xC; // Habilitar el transmisor y receptor
    USART2->CR1 |= 0x1;   // Habilitar UART2
}

void USART2_enviarChar(char c) {
    while (!(USART2->ISR & USART_ISR_TXE)); // Esperar hasta que el buffer de transmisión esté vacío
    USART2->TDR = c; // Escribir el dato en el registro de transmisión
}

char USART2_recibirChar(void) {
    while (!(USART2->ISR & USART_ISR_RXNE)); // Esperar hasta que haya datos recibidos
    return USART2->RDR; // Leer el dato recibido
}

void LED_inicio(void) {
    RCC->AHB1ENR |= 0x2; // Habilitar reloj para GPIOB
    GPIOB->MODER |= 0x1; // Configurar PB0 como salida
}

void LED_Control(char c) {
    if (c == '1') {
        GPIOB->ODR |= 0x1; // Encender LED en PB0
    } else if (c == '0') {
        GPIOB->ODR &= ~0x1; // Apagar LED en PB0
    }
}