#include "stm32f7xx.h"

volatile uint32_t msTicks1;
volatile uint8_t var1;  // Frequency mode counter
volatile uint32_t counter; // Counter value
volatile uint32_t last_tick; // For frequency timing
volatile uint8_t count_enabled; // Flag for counter enable

void LCD_Init();  // Function to initialize the LCD
void Pulse_Enable();  // Function to generate enable pulse for LCD
void Enviar_Dato(uint8_t dato, uint8_t modo);  // Function to send data/command to LCD

//INTERRUPT HANDLER

// Frequency definitions in milliseconds
#define FREQ_A 2000  // 0.5 Hz
#define FREQ_Y 500   // 2 Hz
#define FREQ_H 20    // 50 Hz
#define FREQ_O 8     // 125 Hz


void display_mode(void) {
    switch(var1) {
        case 0: mode_A(); break;
        case 1: mode_Y(); break;
        case 2: mode_H(); break;
        case 3: mode_O(); break;
    }
}

void display_counter(void) {
    Enviar_Dato(0xC0, 0); // Move to second line
    delay(5);
    
    char str[7];
    sprintf(str, "%06lu", counter);
    
    for(int i = 0; str[i] != '\0'; i++) {
        Enviar_Dato(str[i], 1);
    }
}

void update_counter(void) {
    uint32_t current_tick = msTicks1;
    uint32_t freq_delay;
    
    switch(var1) {
        case 0: freq_delay = FREQ_A; break;
        case 1: freq_delay = FREQ_Y; break;
        case 2: freq_delay = FREQ_H; break;
        case 3: freq_delay = FREQ_O; break;
        default: freq_delay = FREQ_A;
    }
    
    if((current_tick - last_tick) >= freq_delay) {
        last_tick = current_tick;
        
        // Check PC0 for up counting
        if((GPIOC->IDR & (1 << 0)) != 0) {
            counter++;
            display_counter();
        }
        
        // Check PC1 for down counting
        if((GPIOC->IDR & (1 << 1)) != 0) {
            if(counter > 0) counter--;
            display_counter();
        }
    }
}

extern "C" {
void SysTick_Handler() {
    msTicks1++;  // Increment millisecond tick count
}
}

void delay(uint32_t tiempo) {
    uint32_t curTicks = msTicks1;  // Store current tick count
    while ((msTicks1 - curTicks) < tiempo) {
        __NOP();  // Wait until the specified time has passed
    }
}

//INTTERUPT HANDLER
extern "C"{
void EXTI2_IRQHandler(void) {

    EXTI->PR |= (1 << 2);  // Clear the pending bit
    if(((GPIOC->IDR) & (1 << 2)) == 1) {
        var1 +=1;  // Set var1 to 1
    }



}
}

void PORTS() {

    RCC->AHB1ENR |= (1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4) |(1 << 5); 

    //A channel to buttons input, using A0(EXTI 0), A1(EXTI 1) AND A15 (EXTI 15) AS EXTI AND PHYSICAL PINS
    //interrutps

    RCC->APB2ENR |= (1<<14); //Enable the SYSCFG clock (EXTI)
	SYSCFG->EXTICR[2] &= ~(0b1111<<0); //Clear bits [7:0] (Px13)
    SYSCFG->EXTICR[2] |= (0x200); //Clear bits [7:0] (Px13)
	
	EXTI->IMR |= (1<<2); //Enable the interrupt on pin 13
	EXTI->RTSR |= (1<<2); //Select rising edge for interrupt
    EXTI->FTSR &= ~(1<<2); //Select falling edge for interrupt
	NVIC_EnableIRQ(EXTI2_IRQn); //Enable the interrupt function on the NVIC module



    //CONFIGURE PC0 AND 1 AS INPUTS
    GPIOC->MODER &= ~(0b11); //Clear bits [1:0] (PC0)
    GPIOC->MODER &= ~(0b11<<2); //Clear bits [3:2] (PC1)



    //CONFIGURE PC0 AND 1 AS PULL-UP INPUTS
    GPIOC->PUPDR &= ~(0b11); //Clear bits [1:0] (PC0)
    GPIOC->PUPDR |= (0b10); //Set bits [1:0] (PC0)

    GPIOC->PUPDR &= ~(0b11<<2); //Clear bits [3:2] (PC1)
    GPIOC->PUPDR |= (0b10<<2); //Set bits [3:2] (PC1)






    var1 = 0;  // Initialize var1

    // Configure PD0-PD7 as output (Data lines)
    GPIOD->MODER = 0x5555;  
    GPIOD->OSPEEDR = 0xAAAA; 

    // Configure PF0 and PF1 as output (RS and E)
    GPIOF->MODER |= (0x01 << (2 * 0)) | (0x01 << (2 * 1)); 
    GPIOF->OSPEEDR |= (0x02 << (2 * 0)) | (0x02 << (2 * 1));  

    // Configure SysTick for 1ms interrupts
    SysTick_Config(SystemCoreClock / 1000);  

    LCD_Init();  // Initialize the LCD
}

void reset_lcd() {
    // Clear screen and configure cursor
    Enviar_Dato(0x01, 0); // Clear screen
    delay(5);
    Enviar_Dato(0x02, 0); // Return home
    delay(5);
    Enviar_Dato(0x06, 0); // Set entry mode
    delay(5);
    Enviar_Dato(0x0C, 0); // Turn on display
    delay(5);
}

void mode_H() {
    Enviar_Dato(0x01, 0);  
    delay(5);
    Enviar_Dato(0x02, 0);
    delay(5);

    // Write "T:" to the LCD
    Enviar_Dato(0X4D, 1); // M
    Enviar_Dato(0x4F, 1); // O
    Enviar_Dato(0x44, 1); // D
    Enviar_Dato(0x4F, 1); // O
    Enviar_Dato(0x3A, 1); // : 
    Enviar_Dato(0x20, 1); // Space
    
    Enviar_Dato(0x48, 1); // H
    Enviar_Dato(0x28, 1);// ( 
    Enviar_Dato(0x35, 1);//5

    Enviar_Dato(0x30, 1);//0
    Enviar_Dato(0x48, 1); // H
    Enviar_Dato(0x7A, 1); // H
    Enviar_Dato(0x29, 1);//)
}

void mode_A() {
    Enviar_Dato(0x01, 0);  
    delay(5);
    Enviar_Dato(0x02, 0);
    delay(5);

    // Write "T:" to the LCD
    Enviar_Dato(0X4D, 1); // M
    Enviar_Dato(0x4F, 1); // O
    Enviar_Dato(0x44, 1); // D
    Enviar_Dato(0x4F, 1); // O
    Enviar_Dato(0x3A, 1); // : 
    Enviar_Dato(0x20, 1); // Space
    
    Enviar_Dato(0x48, 1); // H
    Enviar_Dato(0x28, 1);// ( 
        Enviar_Dato(0x30, 1);//0
        Enviar_Dato(0x2E, 1);//0
    Enviar_Dato(0x35, 1);//5


    Enviar_Dato(0x48, 1); // H
    Enviar_Dato(0x7A, 1); // H
    Enviar_Dato(0x29, 1);//)

}

void mode_Y() {
    Enviar_Dato(0x01, 0);  
    delay(5);
    Enviar_Dato(0x02, 0);
    delay(5);

    // Write "T:" to the LCD
    Enviar_Dato(0X4D, 1); // M
    Enviar_Dato(0x4F, 1); // O
    Enviar_Dato(0x44, 1); // D
    Enviar_Dato(0x4F, 1); // O
    Enviar_Dato(0x3A, 1); // : 
    Enviar_Dato(0x20, 1); // Space
    
    Enviar_Dato(0x48, 1); // H
    Enviar_Dato(0x28, 1);// ( 
    Enviar_Dato(0x32, 1);//2


    Enviar_Dato(0x48, 1); // H
    Enviar_Dato(0x7A, 1); // H
    Enviar_Dato(0x29, 1);//)
}


void mode_O() {
    Enviar_Dato(0x01, 0);  
    delay(5);
    Enviar_Dato(0x02, 0);
    delay(5);

    // Write "T:" to the LCD
    Enviar_Dato(0X4D, 1); // M
    Enviar_Dato(0x4F, 1); // O
    Enviar_Dato(0x44, 1); // D
    Enviar_Dato(0x4F, 1); // O
    Enviar_Dato(0x3A, 1); // : 
    Enviar_Dato(0x20, 1); // Space
    
    Enviar_Dato(0x48, 1); // H
    Enviar_Dato(0x28, 1);// ( 
    Enviar_Dato(0x31, 1);//1
    Enviar_Dato(0x32, 1);//1
    Enviar_Dato(0x35, 1);//1


    Enviar_Dato(0x48, 1); // H
    Enviar_Dato(0x7A, 1); // H
    Enviar_Dato(0x29, 1);//)
}


int main() {
    PORTS();
    reset_lcd();
    
    // Initialize random counter value
    counter = (SysTick->VAL % 500000);
    last_tick = 0;
    var1 = 0; // Start with mode A
    
    display_mode();
    display_counter();
    
    while(1) {
        update_counter();
        // Update mode display if changed
        static uint8_t last_mode = 0xFF;
        if(last_mode != var1) {
            display_mode();
            last_mode = var1;
        }
    }
}

void LCD_Init() {
    delay(30);  // Wait for LCD to power up

    // Configure LCD in 8-bit mode
    Enviar_Dato(0x38, 0); 
    delay(5);

    // Turn on display, cursor, and blinking
    Enviar_Dato(0x0F, 0); 
    delay(5);

    // Clear screen
    Enviar_Dato(0x01, 0);
    delay(5);

    // Set entry mode (increment address)
    Enviar_Dato(0x06, 0);
    delay(5);
}

void Pulse_Enable() {
    GPIOF->BSRR = (1 << 1);  // Set E = 1 (Enable)
    delay(2);
    GPIOF->BSRR = (1 << (1 + 16)); // Reset E = 0 (Disable)
    delay(2);
}

void Enviar_Dato(uint8_t dato, uint8_t modo) {
    if (modo)
        GPIOF->BSRR = (1 << 0);  // Set RS = 1 (Data mode)
    else
        GPIOF->BSRR = (1 << (0 + 16)); // Reset RS = 0 (Command mode)

    GPIOD->ODR = dato; // Send the byte to data lines

    Pulse_Enable();  // Generate enable pulse to latch data/command
    delay(2);
}
