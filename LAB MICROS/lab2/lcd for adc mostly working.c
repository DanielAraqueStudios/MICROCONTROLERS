#include "stm32f7xx.h"  // Device header

volatile uint32_t msTicks1;  // Variable to store millisecond ticks
volatile uint8_t var1;  // General purpose variable

volatile double volts = 0;
volatile uint16_t voltage = 0; // Corregir el tipo de dato a uint16_t

volatile float temp_init = 0.0;  // Nueva variable para el valor entero de la temperatura
volatile float temp_fracc = 0.0;  // Nueva variable para el valor fraccionario de la temperatura

void LCD_Init();  // Function to initialize the LCD
void Pulse_Enable();  // Function to generate enable pulse for LCD
void Enviar_Dato(uint8_t dato, uint8_t modo);  // Function to send data/command to LCD

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

void PORTS() {

    RCC->AHB1ENR |= (1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4) |(1 << 5); 
    RCC->APB2ENR|=(1<<8)|(1<<9)|(1<<10); //ENABLE ADC1 CLOCK

    // A PORTS AS ADC
    /*MODER 0 FOR 8 BITS ,4 MHZ, 112 CYCLES , ALLINGMENT RIGHT 
    0,3-  1V----->TURN ON BLUE LED
    1,3-  2V----->TURN ON GREEN LED
    2-    3V----->TURN ON RED LED
    */
    GPIOA->MODER|=(1<<1)|(1<<0); //PA0 as analog mode                   
    GPIOA->PUPDR|=(1<<1); //PA0 as pull down mode

    ADC->CCR &= ~(0b00<<16); //F= max
    ADC1->CR1&= ~(0b00<<24);//12 bit resolution
    ADC1->CR2|=(1<<0)|(1<<10); // turn on ADC & set end of conversion
    ADC1->CR2 &= ~(1<<11); //right alignment
    ADC1->SMPR2|=(0b111<<0); //480 cycles
    ADC1->SQR3 &= ~(0b11111<<0); //clear channel
    ADC1->SQR3|=(0b00000<<0); //channel 0

    var1 = 10;  // Initialize var1

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

int main() {

    PORTS();
    
    // Clear screen and configure cursor
    Enviar_Dato(0x01, 0);  
    delay(5);
    Enviar_Dato(0x02, 0);
    delay(5);

    // Write "TEMP:" to the LCD
    Enviar_Dato(0x54, 1); 
    Enviar_Dato(0x45, 1); 
    Enviar_Dato(0x4D, 1); 
    Enviar_Dato(0x50, 1); 
    Enviar_Dato(0x45, 1);
    Enviar_Dato(0x52, 1);
    Enviar_Dato(0x41, 1);
    Enviar_Dato(0x54, 1);
    Enviar_Dato(0x55, 1);
    Enviar_Dato(0x52, 1);
    Enviar_Dato(0x45, 1);

    Enviar_Dato(0x3A, 1);  

    static uint16_t adc_buffer[10] = {0}; // Buffer for moving average
    static uint8_t index = 0;
    static uint32_t sum = 0;

    while (1) {
        ADC1->CR2 |= (1 << 30); // Start conversion
        while (((ADC1->SR & (1 << 1)) >> 1) == 0); // Wait for conversion to complete (EOC flag)
        
        voltage = ADC1->DR; // Read the current ADC value
        volts = (double)voltage * (3.3 / 4096); // Convert ADC value to voltage
        
        double temperature = 0.0; // Ensure temperature is of type double
    
        // Calculate temperature based on voltage using linear regression
        if (volts >= 0.100 && volts <= 0.500) {
            // Linear regression coefficients (example values, adjust as needed)
            double slope = (60.0 - 0.0) / (0.500 - 0.100);
            double intercept = 0.0 - slope * 0.100;
            temperature = slope * volts + intercept;
        }
    
        // Move cursor to the position after "TEMP:"
        Enviar_Dato(0xC0, 0); // Move to second line
    
        // Display temperature value on LCD
        int temp_int = (int)temperature;
        int temp_frac = (int)((temperature - temp_int) * 10);
    
        temp_init = (float)temp_int;  // Asignar valor a la nueva variable
        temp_fracc = (float)temp_frac;  // Asignar valor a la nueva variable
    
        Enviar_Dato(0x30 + (temp_int / 10), 1); // Tens digit
        Enviar_Dato(0x30 + (temp_int % 10), 1); // Units digit
        Enviar_Dato(0x2E, 1); // Decimal point
        Enviar_Dato(0x30 + temp_frac, 1); // Fractional digit
    
        delay(1000);  // Delay for 1 second
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
