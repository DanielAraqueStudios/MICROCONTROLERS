# Estación Meteorológica STM32F7

## Pinout

### ADC Inputs
- **PA0** -> ADC1 (Channel 0) 
  - Entrada Analógica 1
  - Rango: 0-3.3V
- **PC0** -> ADC2 (Channel 10)
  - Entrada Analógica 2
  - Rango: 0-3.3V
- **PF3** -> ADC3 (Channel 9)
  - Entrada Analógica 3
  - Rango: 0-3.3V

### UART Communication
- **PD8** -> USART3_TX (Transmisión)
- **PD9** -> USART3_RX (Recepción)
- Baudrate: 9600

### Timer Input Configuration
- **PB5** -> TIM3_CH2 (Entrada de Frecuencia)
  - Configurado como AF2
  - Máximo conteo: 65000 Hz
  - Variable de frecuencia: `frec1` (para señales del DAC1)
  - Variable de frecuencia: `frec2` (para señales del DAC2)
  - Base timer: `baseTimer` = 10µs para precisión en mediciones
  - Frecuencia de muestreo: `sampleRate` = 100Hz

### GPIO Outputs
- **PB0** -> LED de Estado (Heartbeat)
- **PB7** -> LED de Timer3

### Interrupts
- **PC13** -> EXTI13 (External Interrupt)

## Ecuaciones de Caracterización

[Aquí se irán añadiendo las ecuaciones de caracterización a medida que se proporcionen]

## Frecuencias por defecto:
- DAC1: `frec1` = 100.0 Hz (inicial)
- DAC2: `frec2` = 50.0 Hz (inicial)
