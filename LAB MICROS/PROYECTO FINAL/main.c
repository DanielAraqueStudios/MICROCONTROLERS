#include <stm32f767xx.h>
#include <string.h>
#include <stdio.h>

volatile int ContDerecho;
volatile int ContIzquierdo;
int refDerecho = 0;
int refIzquierdo = 0;
const int tickR=18;
int velocidad=80;
bool bandera;

char uart_buffer[20];   // Buffer donde se guardan los caracteres recibidos por UART
volatile uint8_t uart_index = 0;  // Índice actual del buffer
volatile bool uart_complete = false; // Bandera que indica que ya se recibió el mensaje completo
volatile int x = 0, y = 0; // Variables finales donde se guardarán los números recibidos

uint8_t flag = 0, i, cont = 0;
unsigned char d;
char name[7] = "Cely", text[10];

void SysTick_Wait(uint32_t n){
    SysTick->LOAD = n - 1; //15999
    SysTick->VAL = 0; //Clean the value of Systick counter
    while (((SysTick->CTRL & 0x00010000) >> 16) == 0); //Check the count flag until it's 1 
}

void SysTick_ms(uint32_t x){
    for (uint32_t i = 0; i < x; i++){//x ms
        SysTick_Wait(16000); //1ms
    }
}

enum Estado {
    INICIAL,
    AVANZANDO_X,
    GIRANDO,
    AVANZANDO_Y,
    TERMINADO
};

// Variable que guarda el estado actual
volatile enum Estado estado_actual;

void RefDistancia(){
		if(refDerecho==0&&refIzquierdo==0){
				refDerecho = ContDerecho;
				refIzquierdo = ContIzquierdo;
	}
}
void Avanzar(int n){
	if(n<0) n=n*-1;
	if ((ContDerecho-refDerecho) >= n && (ContIzquierdo-refIzquierdo) >= n) {
                // Cuando lleguemos a la distancia, cambiamos de estado
               if(bandera) estado_actual = TERMINADO; else estado_actual = GIRANDO;
								refDerecho = 0;
								refIzquierdo = 0;
            } else {
                // Continuamos avanzando
                GPIOD->ODR = (1<<1) | (1<<6);  // Enciende los motores
            }
}
void Girar(int n){
		if(n>0){
			if ((ContDerecho-refDerecho) >= tickR) {
                // Cuando lleguemos a la rotación, cambiamos de estado
								bandera=false;
                estado_actual = AVANZANDO_Y;
								refDerecho = 0;
								refIzquierdo = 0;
								
            } else {
                // Continuamos girando
                GPIOD->ODR &= ~(1 << 1);  // Apaga el motor derecho
                GPIOD->ODR |= (1 << 6);   // Enciende el motor izquierdo
            }
					 }else{
						 
						 if ((ContIzquierdo-refIzquierdo) >= tickR) {
                // Cuando lleguemos a la rotación, cambiamos de estado
								bandera=false;
                estado_actual = AVANZANDO_Y;
								refDerecho = 0;
								refIzquierdo = 0;
								
            } else {
                // Continuamos girando
                GPIOD->ODR = (1 << 1);  // Apaga el motor derecho
                GPIOD->ODR &= ~(1 << 6);   // Enciende el motor izquierdo
            }
					 }
}
int main(void) {
    // Configuración sensor de herradura
    RCC->AHB1ENR |= (1<<2)|(1 << 3); // Activa reloj para GPIOD y GPIOC
    RCC->APB2ENR |= (1 << 14); // Activa SYSCFG
    
    
    // Configuración para PD4(sensor)
    SYSCFG->EXTICR[1] |= (3 << 0);    // Configura PD4 como fuente de EXTI4
    GPIOD->PUPDR |= (2 << 8);         // Configura PD4 con pull-down
    EXTI->IMR |= (1 << 4);            // Habilita interrupción en PD4
    EXTI->FTSR |= (1 << 4);           // Activa flanco de bajada
    EXTI->PR |= (1 << 4);             // Limpia cualquier interrupción previa
    NVIC_EnableIRQ(EXTI4_IRQn);       // Habilita interrupción EXTI4 en NVIC
    
    // Configuración para PD5(sensor)
    SYSCFG->EXTICR[1] |= (3 << 4);    // Configura PD5 como fuente de EXTI5
    GPIOD->PUPDR |= (2 << 10);        // Configura PD5 con pull-down
    EXTI->IMR |= (1 << 5);            // Habilita interrupción en PD5
    EXTI->FTSR |= (1 << 5);           // Activa flanco de bajada
    EXTI->PR |= (1 << 5);             // Limpia cualquier interrupción previa
    NVIC_EnableIRQ(EXTI9_5_IRQn);     // Habilita interrupción EXTI9_5 en NVIC
		
		//Configuración para motores
		GPIOD->MODER&=~((3<<12)|(3<<14)|(3<<0)|(3<<2));		//Limpiar
		GPIOD->MODER |=  ((1<<12)|(1<<14)|(1<<0)|(1<<2)); //Configura PD0 PD1 PD6 PD7 como salida
		
		//Systick
    SysTick->LOAD = 0x00FFFFFF; 
    SysTick->CTRL |= (0b101);
		
		//Configuración para  pwm (velocidad)
		RCC->AHB1ENR |= (1 << 0); // Activa reloj para GPIOA
		RCC->APB1ENR|=(1<<3);							//Prender Timer5
		TIM5->ARR=99;
		TIM5->PSC=15;
		TIM5->CCR1 = velocidad;      					
		TIM5->DIER|=(1<<0);								//Interrupcion por desbordamiento
		TIM5->CCMR1|=(0b110<<4);					//PWM modo1 en CH1
		TIM5->CCER|=(1<<0);								//Habilita CH1 
		TIM5->CR1 |= (1<<0);  						// Habilitar el timer
		
		GPIOA->MODER &= ~(3<<0);    			// Limpiar bits
		GPIOA->MODER |=  (2<<0);    			// Modo Alternate Function (10)
		GPIOA->AFR[0]|=(2<<0);						//Asigna AF2 a PA0 
		
		 //Interrupt
    RCC->APB2ENR |= (1<<14); 
    SYSCFG->EXTICR[3] &= ~(0b1111<<4); 
    SYSCFG->EXTICR[3] |= (1<<5); 
    EXTI->IMR |= (1<<13); 
    EXTI->RTSR |= (1<<13);
    NVIC_EnableIRQ(EXTI15_10_IRQn); 
		
		//UART
    GPIOD->MODER &= ~((0b11<<18)|(0b11<<16)); //Clear (00) pins PD9 (bits 19:18) and PD8 (bits 17:16)
    GPIOD->MODER |= (1<<19)|(1<<17); // PD9=RX y PD8=TX alternant function
    GPIOD->AFR[1] &= ~((0b1111<<4)|(0b1111<<0)); //limpia alternant functions para PD9 y PD8
    GPIOD->AFR[1] |= (0b111<<4)|(0b111<<0); //Coloca USART3 (AF7) alternant function para PD9=RX and PD8=TX
    RCC->APB1ENR |= (1<<18); //Enable the USART3 clock
    USART3->BRR = 0x683; //Set the baud rate on 9600 baud to 16 MHz (HSI)
    USART3->CR1 |= ((1<<5)|(0b11<<2)); //RXNE interrupt enable, transmitter enable and receiver enable
    USART3->CR1 |= (1<<0); //USART enable
    NVIC_EnableIRQ(USART3_IRQn); //Enable the interrupt function on the NVIC module
		
    
    // Inicializar contadores
    ContDerecho = 0;
    ContIzquierdo = 0;
		estado_actual = TERMINADO;
		
    
			int tickX;
			int tickY;
		while (1){
			if(flag == 1){
            flag = 0;
            cont++;
            sprintf(text,"%s %d\n",uart_buffer, cont);
            for(i=0; i<strlen(text); i++){
                USART3->TDR = text[i]; //Data transmitted
                while(((USART3->ISR & 0x80) >> 7) == 0){} //Wait until the data is transferred to the shift register (flag TXE=0)
            }
            USART3->TDR = 0x0A; //Send end line
            while((USART3->ISR & 0x80)==0){};
            USART3->TDR = 0x0D; //Send carry return
            while(((USART3->ISR & 0x80) >> 7) == 0){}
        }
	 if (uart_complete) {
    uart_complete = false;  // Se apaga la bandera para no repetir

    int tempX = 0, tempY = 0;
    if (sscanf(uart_buffer, "%d,%d", &tempX, &tempY) == 2) {
        x = tempX;
        y = tempY;

        tickX = x * 0.84;
        tickY = y * 0.84;

        estado_actual = INICIAL;
    }
}
			switch(estado_actual) {
        case INICIAL:
            ContDerecho = 0;
            ContIzquierdo = 0;
						refDerecho = 0;
						refIzquierdo = 0;
            estado_actual = AVANZANDO_X;
            break;
        
        case AVANZANDO_Y:
					RefDistancia();
					bandera=true;
          // Avanzar en el eje Y
           Avanzar(tickY);
            break;
        
        case GIRANDO:
					RefDistancia();
            // Rotar
					Girar(x);
            break;
        
        case AVANZANDO_X:
					RefDistancia();
            // Avanzar en el eje X
          Avanzar(tickX);
            break;
        
        case TERMINADO:
            // El robot ha llegado a la posición final
            GPIOD->ODR &= ~((1<<6) | (1<<1));  // Apaga los motores
            break;
    }
	}
}


extern "C" void EXTI4_IRQHandler(void) {
    if (EXTI->PR & (1 << 4)) {        // Verificar si la interrupción es de línea 4 (corregido)
        EXTI->PR |= (1 << 4);         // Limpiar el registro de estado de la interrupción (corregido)
        ContIzquierdo++;              // Incrementar el contador izquierdo
    }
}


extern "C" void EXTI9_5_IRQHandler(void) {       // Nombre corregido (era EXTI10_5_IRQHandler)
    if (EXTI->PR & (1 << 5)) {
        EXTI->PR |= (1 << 5);         // Limpia la bandera de interrupción
        ContDerecho++;
    }
} 

extern "C" void USART3_IRQHandler(void){
    if ((USART3->ISR & USART_ISR_RXNE) != 0) {
        char c = USART3->RDR;  // Leer el carácter recibido

        if (c == '\n' || c == '\r') { // Si llegó un salto de línea (fin del mensaje)
            uart_buffer[uart_index] = '\0'; // Termina el string con null character
            uart_index = 0;
            uart_complete = true;  // Bandera que indica que ya llegó todo el mensaje
        } else {
            if (uart_index < sizeof(uart_buffer) - 1) {
                uart_buffer[uart_index++] = c; // Agrega el carácter al buffer
            }
        }
    }
}
extern "C" void EXTI15_10_IRQHandler(void){
        EXTI->PR |= (1<<13); //Down flag
        if(((GPIOC->IDR & (1<<13)) >> 13) == 1){
            flag = 1;
        }
			}