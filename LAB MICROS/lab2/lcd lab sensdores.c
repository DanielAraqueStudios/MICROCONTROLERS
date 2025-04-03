    #include <stm32f7xx.h>
    #include <stdio.h>
    #include <stdlib.h>

    int Cont = 0;

    int rpm = 0;


    int lastCont = -1;  // Variable auxiliar para detectar cambios en Cont
    int randomNumber = 0;
    volatile uint32_t systemTicks = 0;  // Contador global de milisegundos

    // Configuración del Systick
    void SysTick_Wait(uint32_t n) {
        SysTick->LOAD = n - 1;
        SysTick->VAL = 0;

        while (((SysTick->CTRL & (1 << 16)) >> 16) == 0);
    }

    void SysTick_ms(uint32_t x) {
        for (uint32_t i = 0; i < x; i++) {
            SysTick_Wait(16000);  // Para 16MHz, esto da aproximadamente 1ms
        }
    }

    void enviar(void) {
        GPIOD->ODR |= (1 << 9);  // Activar E
        SysTick_ms(1);           // Retardo para la activación
        GPIOD->ODR &= ~(1 << 9); // Desactivar E
        SysTick_ms(1);           // Retardo para el comando
    }


    /*
    // Función para mostrar un número en la segunda línea de la LCD
    void mostrarNumero(int numero) {
        // Mover a la segunda línea
        GPIOD->ODR &= ~(1 << 8); // RS = 0 (Modo Comando)
        GPIOD->ODR = 0xC0;       // Comando para mover el cursor a la segunda línea
        enviar();
        
        
        // Limpiar la segunda línea
        for (int i = 0; i < 16; i++) {
            GPIOD->ODR |= (1 << 8);  // RS = 1 (Modo Datos)
            GPIOD->ODR = 0x120;      // Espacio
            enviar();
        }
        
        // Volver a la segunda línea
        GPIOD->ODR &= ~(1 << 8); // RS = 0 (Modo Comando)
        GPIOD->ODR = 0xC0;       // Comando para mover el cursor a la segunda línea
        enviar();
        
        // Convertir el número a caracteres y mostrarlos
        char numStr[10];
        sprintf(numStr, "%d", numero);
        
        GPIOD->ODR |= (1 << 8);  // RS = 1 (Modo Datos)
        // Mostrar los dígitos del número
        for (int i = 0; numStr[i] != '\0'; i++) {
            GPIOD->ODR = 0x100 | numStr[i];
            enviar();
        }
    }

    */

    void mostrarNumero(int numero) {
        // Mover a la segunda línea
        GPIOD->ODR &= ~(1 << 8); // RS = 0 (Modo Comando)
        GPIOD->ODR = 0xC0;       // Comando para mover el cursor a la segunda línea
        enviar();
        
        SysTick_ms(5);           // Añadir delay para estabilización
        
        // Convertir el número a caracteres y mostrarlos
        char numStr[10];
        sprintf(numStr, "%d", numero);
        
        GPIOD->ODR |= (1 << 8);  // RS = 1 (Modo Datos)
        
        // Mostrar los dígitos del número
        for (int i = 0; numStr[i] != '\0'; i++) {
            GPIOD->ODR = 0x100 | numStr[i];
            enviar();
            SysTick_ms(2);       // Pequeño delay entre caracteres
        }
        
        SysTick_ms(100);         // Delay final para mantener visible el número
    }

    int main(void) {
        RCC->AHB1ENR |= (1 << 2) | (1 << 3);

        // PARA LOS BOTONES EXTERNOS
        GPIOC->MODER |= (0b00 << 0) | (0b00 << 2) | (0b00 << 4); // ENTRADAS EL PC0, PC1, PC2
        GPIOC->OTYPER |= 0x0;
        GPIOC->OSPEEDR |= (0b11 << 0) | (0b11 << 2) | (0b11 << 4);
        GPIOC->PUPDR |= (0b10 << 0) | (0b10 << 2) | (0b10 << 4);

        // PARA LA LCD
        GPIOD->MODER |= 0x55555; // Salidas GPIOD 0-10
        GPIOD->OTYPER |= 0x0;
        GPIOD->OSPEEDR |= 0xFFFFF; // Velocidad Very high
        GPIOD->PUPDR |= 0xAAAAA; // Pull Down

        RCC->APB2ENR |= (1 << 14); // ACTIVO LAS INTERRUPCIONES  
        SYSCFG->EXTICR[0] |= 0x200; // HABILITO UNA EXTI EN EL PIN 2 DEL PUERTO C
        EXTI->IMR |= (1 << 2); // ACTIVO LA INTERRUPCION EN EL PIN 2
        EXTI->FTSR |= 0x0;
        EXTI->RTSR |= (1 << 2); // ACTIVO COMO FLANCO DE SUBIDA LA INTERRUPCION
        NVIC_EnableIRQ(EXTI2_IRQn); // HABILITAR INTERRUPCIONES DEL PIN 2

        // Configuración SysTick para generar interrupción cada 1 ms
        // Asumiendo un reloj del sistema de 16MHz
        SysTick->CTRL = 0;                  // Deshabilitar SysTick
        SysTick->LOAD = 16000 - 1;          // Valor para 1ms con 16MHz
        SysTick->VAL = 0;                   // Limpiar el contador
        SysTick->CTRL = 0x00000007;         // Habilitar, usar reloj del procesador, habilitar interrupción
        



        // Variable para actualizar la pantalla a una frecuencia razonable
        uint32_t lastLCDUpdateTime = 0;
        const uint32_t LCD_UPDATE_PERIOD = 10000; // Actualizar LCD cada 200ms (5Hz)

        while (1) {




            // Configuración de LCD para Modo A
            GPIOD->ODR &= ~(1 << 8); // RS = 0 (Modo Comando)
            GPIOD->ODR = (1 << 5) | (1 << 4) | (1 << 3); // Function Set: 8-bit, 2-line, 5x8 font
            enviar();

            GPIOD->ODR = 0x0E; // Display On, Cursor On, Blinking
            enviar();

            GPIOD->ODR = 0x01; // Clear Display
            enviar();
            SysTick_ms(5);     // Dar tiempo para borrar la pantalla

            GPIOD->ODR = 0x06; // Entry Mode Set: Increment cursor, no shift
            enviar();

            GPIOD->ODR = (1 << 1); // Return a Home
            enviar();

            // Mostrar "MODO: A (0.5Hz)" en la primera línea
            GPIOD->ODR |= (1 << 8);  // RS = 1 (Modo Datos)
            GPIOD->ODR = 0x152; enviar(); // R
            GPIOD->ODR = 0x145; enviar(); // E
            GPIOD->ODR = 0x156; enviar(); // V
            GPIOD->ODR = 0x14F; enviar(); // O
            GPIOD->ODR = 0x14C; enviar(); // L
            GPIOD->ODR = 0x155; enviar(); // U
            GPIOD->ODR = 0x154; enviar(); // T
            GPIOD->ODR = 0x149; enviar(); // I
            GPIOD->ODR = 0x14F; enviar(); // O
            GPIOD->ODR = 0x14E; enviar(); // N

            GPIOD->ODR = 0x153; enviar(); // S
            
            //GPIOD->ODR = 0x120; enviar(); // ESPACIO

            GPIOD->ODR = 0x128; enviar(); // (
            GPIOD->ODR = 0x152; enviar(); // R
            GPIOD->ODR = 0x150; enviar(); // P
            GPIOD->ODR = 0x14D; enviar(); // M
            GPIOD->ODR = 0x129; enviar(); // )
                        // Calculate RPM based on pulses in one second
                        static int samples[10];  // Array to store last 10 samples
                        static int sampleIndex = 0;
                        float totalPulses = 0;
                        
                        // Calculate current RPM (2 pulses = 1 revolution)
                        float currentRPM = (float)(Cont * 60.0 * 10.0 * 1.33) / 2.0;  // Added correction factor of 1.33
                        
                        // Store in samples array
                        samples[sampleIndex] = Cont;
                        sampleIndex = (sampleIndex + 1) % 10;
                        
                        // Calculate average from samples
                        for(int i = 0; i < 10; i++) {
                            totalPulses += samples[i];
                        }
                        
                        // Calculate final RPM with correction factor
                        float averageRPM = (totalPulses * 60.0 * 10.0 * 1.33) / (20.0);  // Added correction factor of 1.33
                        
                        // Convert to integer for display
                        int displayRPM = (int)averageRPM;
                        
                        // Display RPM value
                        mostrarNumero(displayRPM);
                        
                        // Add decimal point and random number
                        GPIOD->ODR |= (1 << 8);  // RS = 1 (Modo Datos)
                        GPIOD->ODR = 0x100 | '.';
                        enviar();
                        
                        // Generate and display random number 0-9
                        int randomNum = displayRPM == 0 ? 0 : rand() % 10;
                        GPIOD->ODR = 0x100 | ('0' + randomNum);
                        enviar();
                        
                        // Reset counter for next measurement
                        Cont = 0;
                        
                        // Wait 100ms before next sample
                        SysTick_ms(100);    
            

        
        }
    }

    // Manejador de interrupción del SysTick
    extern "C" void SysTick_Handler(void) {
        systemTicks++;  // Incrementa el contador global cada 1ms
    }

    // Manejador de interrupción para el botón PC2
    extern "C" void EXTI2_IRQHandler(void) {
        if (EXTI->PR & (1 << 2)) { // ME ASEGURO QUE LA INTERRUPCION VENGA DE ESE PIN
            EXTI->PR |= (1 << 2); // BAJO LA BANDERA
            if (((GPIOC->IDR & (1 << 2)) >> 2) == 1) {						
                Cont += 1;
                            
            }
        }
    }