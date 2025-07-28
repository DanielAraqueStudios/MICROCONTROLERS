#include <stm32f7xx.h>
#include <math.h>
#include <stdio.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Definiciones para el MPU6050
#define FREQTRAB 16000000
#define MPU6050_ADDR         0x68
#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6B

int rx_data[1];
int tx_data[2];
volatile int setup_ok, raw_adc = 0;
unsigned int data[2];
volatile int W_OK, R_OK;
void INIT_I2C(void);
void SysTick_Init(void);
void SysTick_Wait(uint32_t n);
void SysTick_Wait1ms(uint32_t delay);
int I2C1_Lee(int direccion, int reg_dir, int *buffer, int nbytes);
int I2C1_Escribe(int direccion, int reg_dir, int *buffer, int nbytes);
void configMPU(void);

double raw_acc_x, raw_acc_y;
double anguloInclinacion;
char angleStr[16];

// Variables para filtro de media móvil
#define N_SAMPLES 10
double angulos[N_SAMPLES] = {0};
int sample_index = 0;

void Enable(void);
void LCD_Init(void);
void Command(uint8_t cmd);
void Escribir(char data);
void LCD_SendString(const char X[]);
void LCD_SetCursor(uint8_t row, uint8_t col);
void UpdateDisplay(float angle);

void agregarMediaMovil(double nuevo_valor) {
    angulos[sample_index] = nuevo_valor;
    sample_index = (sample_index + 1) % N_SAMPLES;
}

double calcularPromedio() {
    double suma = 0.0;
    for (int i = 0; i < N_SAMPLES; i++) {
        suma += angulos[i];
    }
    return suma / N_SAMPLES;
}

void GPIO_LCD_Init(void) {
    // Habilitar reloj de GPIOF
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;

    // Configuración de PF0 y PF1 (RS y E) como salidas
    // Y PF4-PF11 (D0-D7) como salidas
    GPIOF->MODER |= (1 << 0) | (1 << 2)  // PF0, PF1 como salida
                  | (1 << 8) | (1 << 10) | (1 << 12) | (1 << 14)  // PF4-PF7
                  | (1 << 16) | (1 << 18) | (1 << 20) | (1 << 22); // PF8-PF11
    
    // Configurar como push-pull
    GPIOF->OTYPER &= ~(0xFF << 4);  // PF4-PF11
    GPIOF->OTYPER &= ~(3 << 0);     // PF0-PF1
    
    // Alta velocidad
    GPIOF->OSPEEDR |= 0xFFFFFF;
    
    // Sin pull-up/pull-down
    GPIOF->PUPDR &= ~0xFFFFFF;
}


int main(void) {
	
    SysTick_Init();
    INIT_I2C();
    SysTick_Wait1ms(1000);
    configMPU();

	  GPIO_LCD_Init();  // Inicializar los pines de la LCD 
    LCD_Init();
    LCD_SendString("Angulo(D):");

    while (1) {
        int16_t accum_x = 0, accum_y = 0;

        // Leer ejes X e Y del MPU6050
        I2C1_Lee(MPU6050_ADDR, 0x3D, rx_data, 1);
        accum_x = rx_data[0] << 8;
        SysTick_Wait1ms(100);
        I2C1_Lee(MPU6050_ADDR, 0x3E, rx_data, 1);
        accum_x |= rx_data[0];
        SysTick_Wait1ms(100);

        I2C1_Lee(MPU6050_ADDR, 0x3F, rx_data, 1);
        accum_y = rx_data[0] << 8;
        SysTick_Wait1ms(100);
        I2C1_Lee(MPU6050_ADDR, 0x40, rx_data, 1);
        accum_y |= rx_data[0];
        SysTick_Wait1ms(100);

        // Convertir los valores crudos a una escala de -1 a 1
        raw_acc_x = accum_x / 16384.0;
        raw_acc_y = accum_y / 16384.0;

        // Calcular ángulo en 0-360 grados usando atan2
        anguloInclinacion = atan2(raw_acc_y, raw_acc_x) * (180.0 / M_PI);
        if (anguloInclinacion < 0){
					anguloInclinacion += 360;}

        // Agregar a media móvil
        agregarMediaMovil(anguloInclinacion);

        // Calcular promedio de los ángulos
        double anguloFiltrado = calcularPromedio();

        // Mostrar en la LCD
        UpdateDisplay(anguloInclinacion);
        SysTick_Wait1ms(1000);  // Espera entre lecturas
    }
}


void INIT_I2C(){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    GPIOB->MODER |= (2<<16) | (2<<18);
    GPIOB->OTYPER |= (1<<8) | (1<<9);
    GPIOB->PUPDR |= (1<<16) | (1<<18);
    GPIOB->OSPEEDR |= (3<<16) | (3<<18);
    GPIOB->AFR[1] |= (4<<0) | (4<<4);
    RCC->DCKCFGR2 |= (2<<16);

    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    I2C1->CR1 &= ~I2C_CR1_PE;
    I2C1->TIMINGR = 0x205A1819;
    I2C1->CR1 |= I2C_CR1_PE;
}


void configMPU(void) {
    tx_data[0] = 0x00;
    I2C1_Escribe(MPU6050_ADDR, MPU6050_SMPLRT_DIV, tx_data, 1);	
    I2C1_Escribe(MPU6050_ADDR, MPU6050_CONFIG, tx_data, 1);	
    tx_data[0] = 0x08;
    I2C1_Escribe(MPU6050_ADDR, MPU6050_GYRO_CONFIG, tx_data, 1);	
    I2C1_Escribe(MPU6050_ADDR, MPU6050_ACCEL_CONFIG, tx_data, 1);
    tx_data[0] = 0x01;
    I2C1_Escribe(MPU6050_ADDR, MPU6050_PWR_MGMT_1, tx_data, 1);	
}

int	I2C1_Lee(int direccion, int reg_dir, int *buffer, int nbytes ) {
	uint32_t 	t_espera;
	uint8_t		n;			// Contador para la lectura de los nbytes

	// Dirección del dispositivo
	I2C1->CR2 &= ~I2C_CR2_SADD_Msk;
	I2C1->CR2 |= ((direccion <<1U) <<I2C_CR2_SADD_Pos);

  // i2c Modo Escritura
	I2C1->CR2 &= ~I2C_CR2_RD_WRN;

	I2C1->CR2 &= ~I2C_CR2_NBYTES;
	I2C1->CR2 |= (1 <<16U);
	I2C1->CR2 &= ~I2C_CR2_AUTOEND;

	// Iniciar charla I2C 
	I2C1->CR2 |= I2C_CR2_START;

	// Espera a TXIS o se sale del while y devuelve un 1
	// SI devuelve 1 significa que no se inicio la charla
	t_espera = 2000;
	while (((I2C1->ISR) & I2C_ISR_TXIS) != I2C_ISR_TXIS)
	{
		t_espera--;
		if (t_espera == 0) return 1;
	}

	// Envia la dirección del registro que se va a leer
	I2C1->TXDR = reg_dir;

	// Espera a TC o se sale del while y devuelve un 2
	t_espera = 2000;
	while (((I2C1->ISR) & I2C_ISR_TC) != I2C_ISR_TC)
	{
		t_espera--;
		if (t_espera == 0) return 2;
	}

	// i2c en modo lectura
	I2C1->CR2 |= I2C_CR2_RD_WRN;
  
	I2C1->CR2 &= ~I2C_CR2_NBYTES;
	I2C1->CR2 |= (nbytes <<16U);
	I2C1->CR2 &= ~I2C_CR2_AUTOEND;

	// Se repite la condición de inicio para indicar que se leen
	// nbytes
	I2C1->CR2 |= I2C_CR2_START;

	n = nbytes;

	while (n>0)
	{
		// Espera a RXNE o se sale del while y devuelve un 3
		t_espera = 2000;
		while (((I2C1->ISR) & I2C_ISR_RXNE) != I2C_ISR_RXNE)
		{
			t_espera--;
			if (t_espera == 0) return 3;
		}

		// Se guardan los datos en buffer.
		// Buffer debe tener tantas posiciones como datos a leer
		*buffer = I2C1->RXDR;
		buffer++;
		n--;
	}

	// para i2c
	I2C1->CR2 |= I2C_CR2_STOP;

	// Espera a STOPF o se sale del while y devuelve un 4
	t_espera = 2000;
	while (((I2C1->ISR) & I2C_ISR_STOPF) != I2C_ISR_STOPF)
	{
		t_espera--;
		if (t_espera == 0) return 4;
	}

	// Todo OK, todo correcto y yo retorno 0.
	return 0;
}



int	I2C1_Escribe( int direccion, int reg_dir, int *buffer, int nbytes ){
	uint32_t 	t_espera;	// t_espera
	uint8_t		n;		// Contador para la lectura de datos

	// Dirección del esclavo
	I2C1->CR2 &= ~I2C_CR2_SADD_Msk;
	I2C1->CR2 |= ((direccion <<1) <<I2C_CR2_SADD_Pos);

	// i2c modo escritura
	I2C1->CR2 &= ~I2C_CR2_RD_WRN;
	I2C1->CR2 &= ~I2C_CR2_NBYTES;
	I2C1->CR2 |= ((nbytes+1) <<16);
	I2C1->CR2 |= I2C_CR2_AUTOEND;

	// Limpiar bandera de STOP, por si acaso
	I2C1->ICR |= I2C_ICR_STOPCF;

	// iniciar i2c
	I2C1->CR2 |= I2C_CR2_START;

	// Espera lla bandera TXIS o que se acabe el tiempo de espera
	t_espera = 2000;
	while (((I2C1->ISR) & I2C_ISR_TXIS) != I2C_ISR_TXIS)
	{
		t_espera--;
		if (t_espera == 0) return 1;
	}

	// Dirección del registro que se quiere modificar
	I2C1->TXDR = reg_dir;

	n = nbytes;

	while(n>0)
	{
		// Espera la bandera TXIS o que se acabe el tiempo de espera
		t_espera = 2000;
		while (((I2C1->ISR) & I2C_ISR_TXIS) != I2C_ISR_TXIS)
		{
			t_espera--;
			if (t_espera == 0) return 2;
		}

		// Se envian los datos en el array llamado buffer
		I2C1->TXDR = *buffer;
		buffer++;
		n--; // La n que cuenta el numero de datos
	}

	// Hasta que este STOPF o se cumpla el t_espera
	t_espera = 2000;
	while (((I2C1->ISR) & I2C_ISR_STOPF) != I2C_ISR_STOPF)
	{
		t_espera--;
		if (t_espera == 0) return 3;
	}

	// Si todo sale bien seria 0
	return 0;
}

// LCD Functions

void LCD_Init(void) {
    SysTick_Wait1ms(20);      // Esperar a que la LCD encienda
    Command(0x38);            // Modo de 8 bits, 2 líneas, 5x8 puntos
    Command(0x0C);            // Display ON, cursor OFF
    Command(0x01);            // Limpiar pantalla
    Command(0x06);            // Incrementar y sin desplazamiento de pantalla
    SysTick_Wait1ms(5);       // Esperar para estabilización
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
    GPIOF->ODR = (GPIOF->ODR & ~(0xFF << 4)) | ((cmd & 0xFF) << 4);  // Datos en PF4-PF11
    GPIOF->ODR &= ~0x1;  // RS = 0 (PF0)
    Enable();
}

void Escribir(char data) {
    GPIOF->ODR = (GPIOF->ODR & ~(0xFF << 4)) | ((data & 0xFF) << 4);  // Datos en PF4-PF11
    GPIOF->ODR |= 0x1;   // RS = 1 (PF0)
    Enable();
}


void LCD_SendString(const char X[]) {
    for (int i = 0; X[i] != '\0'; i++) {
        Escribir(X[i]);
    }
}


void floatToString(float value, char* str, int precision) {
    int intPart = (int)value;  // Parte entera
    int decPart = (int)((value - intPart) * pow(10, precision));  // Parte decimal
    sprintf(str, "%d.%0*d", intPart, precision, decPart);  // Formatear el número
}

void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t address = (row == 0) ? col : col + 0x40;
    Command(0x80 | address);  // 0x80 es la instrucción para posicionar el cursor
}


void UpdateDisplay(float angle) {
    // Convierte el ángulo a string
    snprintf(angleStr, sizeof(angleStr), "%4.2f", angle);
    
    LCD_SetCursor(0, 11);  // Coloca el cursor donde deseas empezar a mostrar el ángulo
    LCD_SendString(angleStr);  // Muestra el ángulo en la LCD
}