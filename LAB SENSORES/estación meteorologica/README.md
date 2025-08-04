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

# Weather Station UART Communication Protocol

## Overview
The weather station uses UART communication at 9600 baud rate to transmit sensor data from the microcontroller to the GUI application.

## Data Format
Each sensor reading is sent as a separate line with the following format:

Where:
- `x` is the ADC channel number (1-4)
- `Y.YY` is the voltage reading with 2 decimal places
- `\n` is the newline character

## Sensor Mappings

1. **ADC1: Temperature Sensor**
   - Format: `ADC1: Y.YYV`
   - Conversion: Temperature(°C) = 55.60 * voltage + 0.367 + 13
   - Range: 0-100°C

2. **ADC2: Wind Speed Sensor**
   - Format: `ADC2: Y.YYV`
   - Conversion: Speed(km/h) = 58.943 * voltage + 2.037
   - Range: 0-200 km/h

3. **ADC3: Light Intensity Sensor**
   - Format: `ADC3: Y.YYV`
   - Conversion: Intensity(%) = -45.45 * voltage + 100
   - Range: 0-100%

4. **ADC4: Humidity Sensor**
   - Format: `ADC4: Y.YYV`
   - Conversion: Humidity(%) = (voltage / 3.3) * 100
   - Range: 0-100%




  ## Example Data Stream
  ADC1: 1.23V ADC2: 2.50V ADC3: 1.50V ADC4: 2.75V

## Error Handling
- Invalid data is ignored
- Out of range values are clamped to valid ranges
- Debug messages are printed to console for troubleshooting

## Connection Settings
- Baud Rate: 9600
- Data Bits: 8
- Parity: None
- Stop Bits: 1
- Flow Control: None