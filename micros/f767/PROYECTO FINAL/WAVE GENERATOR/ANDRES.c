#include <stm32f7xx.h>
#include <math.h>

#define FREQTRAB 216000000.0
#define NUM_PUNTOS 216

const double tabla_seno[NUM_PUNTOS] = {
    0.0000, 0.0291, 0.0582, 0.0872, 0.1160, 0.1448, 0.1734, 0.2018, 0.2299, 0.2578,
    0.2855, 0.3129, 0.3400, 0.3668, 0.3932, 0.4193, 0.4450, 0.4703, 0.4951, 0.5196,
    0.5435, 0.5670, 0.5899, 0.6123, 0.6342, 0.6556, 0.6763, 0.6965, 0.7160, 0.7350,
    0.7532, 0.7708, 0.7877, 0.8039, 0.8193, 0.8340, 0.8479, 0.8610, 0.8733, 0.8848,
    0.8954, 0.9052, 0.9142, 0.9222, 0.9294, 0.9356, 0.9409, 0.9453, 0.9488, 0.9513,
    0.9529, 0.9535, 0.9532, 0.9519, 0.9496, 0.9463, 0.9421, 0.9368, 0.9306, 0.9234,
    0.9151, 0.9059, 0.8957, 0.8844, 0.8722, 0.8590, 0.8447, 0.8295, 0.8132, 0.7960,
    0.7778, 0.7586, 0.7384, 0.7172, 0.6951, 0.6720, 0.6479, 0.6230, 0.5970, 0.5702,
    0.5424, 0.5137, 0.4841, 0.4537, 0.4223, 0.3901, 0.3571, 0.3233, 0.2886, 0.2531,
    0.2169, 0.1798, 0.1420, 0.1034, 0.0641, 0.0241, -0.0166, -0.0580, -0.0999, -0.1424,
    -0.1855, -0.2290, -0.2729, -0.3172, -0.3619, -0.4069, -0.4522, -0.4976, -0.5433, -0.5891,
    -0.6350, -0.6810, -0.7269, -0.7728, -0.8186, -0.8643, -0.9098, -0.9551, -1.0000, -1.0447,
    -1.0889, -1.1328, -1.1762, -1.2191, -1.2615, -1.3033, -1.3445, -1.3850, -1.4248, -1.4639,
    -1.5021, -1.5396, -1.5761, -1.6117, -1.6463, -1.6800, -1.7126, -1.7441, -1.7746, -1.8039,
    -1.8321, -1.8591, -1.8849, -1.9095, -1.9328, -1.9548, -1.9755, -1.9949, -2.0129, -2.0296,
    -2.0448, -2.0586, -2.0710, -2.0819, -2.0913, -2.0992, -2.1056, -2.1104, -2.1137, -2.1154,
    -2.1156, -2.1142, -2.1112, -2.1065, -2.1003, -2.0924, -2.0829, -2.0718, -2.0591, -2.0447,
    -2.0287, -2.0111, -1.9918, -1.9709, -1.9484, -1.9242, -1.8984, -1.8710, -1.8419, -1.8112,
    -1.7789, -1.7450, -1.7096, -1.6725, -1.6339, -1.5938, -1.5521, -1.5090, -1.4644, -1.4184,
    -1.3710, -1.3221, -1.2719, -1.2204, -1.1675, -1.1133, -1.0578, -1.0012, -0.9433, -0.8842,
    -0.8240, -0.7626, -0.7001, -0.6366, -0.5720, -0.5064, -0.4399, -0.3724, -0.3040, -0.2347,
    -0.1645, -0.0936, -0.0219, 0.0505, 0.1236, 0.1974
};



void configs(void);

//VARIABLES PARA DAC 1
volatile int amplitud=0;
volatile int offset=0;
volatile double amplitud_voltaje=0;
volatile double offset_voltaje=0;
volatile double frecuencia=0;
volatile uint32_t v_DAC=0;
volatile uint16_t arriba = 1;
volatile uint32_t contador = 0;
volatile uint8_t salida_pulso = 0;
volatile int magnitud = 1;  // Inicializar en 1
volatile int N_pasos = 400;
volatile uint32_t v_dac=0;
volatile uint32_t indice_seno = 0;
volatile uint32_t paso_frecuencia = 0;
volatile uint32_t contador_pulso = 0;  // Contador específico para pulso DAC1
static char onda = '2';

//VARIABLES PARA DAC 2
volatile int amplitud2=0;
volatile int offset2=0;
volatile double amplitud_voltaje2=0;
volatile double offset_voltaje2=0;
volatile double frecuencia2=0;
volatile uint32_t v_DAC2=0;
volatile uint16_t arriba2 = 1;
volatile uint32_t contador2 = 0;
volatile uint8_t salida_pulso2 = 0;
volatile int magnitud2 = 1;  // Inicializar en 1
volatile int N_pasos2 = 400;
volatile uint32_t v_dac2=0;
volatile uint32_t indice_seno2 = 0;
volatile uint32_t paso_frecuencia2 = 0;
volatile uint32_t contador_pulso2 = 0;  // Contador específico para pulso DAC1
static char onda2 = '2';

//FUNCIONES GLOBALES
void SysTick_Init(void);
void SysTick_Wait(uint32_t n);
void SysTick_Wait1ms(uint32_t delay);
void PLL_Config();

//FUNCIONES DAC 1
int seno(int offset, int amplitud);
int cuadrada(int offset, int amplitud);
int Sierra(int offset, int amplitud, int N_pasos);
int pulso(int offset, int amplitud);
void TIM2_Config();
int calcular_frecuencia(double frecuencia);

//FUNCIONES DAC 2
int seno2(int offset, int amplitud);
int cuadrada2(int offset, int amplitud);
int Sierra2(int offset, int amplitud, int N_pasos);
int pulso2(int offset, int amplitud);
void TIM5_Config();
int calcular_frecuencia2(double frecuencia);

int main (void){
    //GLOBAL
		PLL_Config();
    configs();
    SysTick_Init();
    
    //DAC 1
    offset_voltaje = 1.65;
    amplitud_voltaje = 2;
    amplitud = (int)((amplitud_voltaje / 3.3) * 4095);
    offset = (int)((offset_voltaje / 3.3) * 4095);
    onda = '4';
    frecuencia = 1;
	
		//DAC 2
    offset_voltaje2 = 1.65;
    amplitud_voltaje2 = 2;
    amplitud2 = (int)((amplitud_voltaje2 / 3.3) * 4095);
    offset2 = (int)((offset_voltaje2 / 3.3) * 4095);
    onda2 = '4';
    frecuencia2 = 200000;
    
    SysTick_Wait1ms(1000);
    
    //TIM DAC 1
    TIM2_Config();
    TIM2->ARR = calcular_frecuencia(frecuencia);
    TIM2->EGR |= TIM_EGR_UG;
		
		//TIM DAC 2
    TIM5_Config();
    TIM5->ARR = calcular_frecuencia2(frecuencia2);
    TIM5->EGR |= TIM_EGR_UG;
    
    while(1){
        // Loop principal vacío
    }
}

//FUNCIONES GLOBALES
void configs(void){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_DACEN;
	GPIOA->MODER |= (3 << (4*2)) | (3 << (5*2));
	DAC->CR |= (1<<0) | (1<<16);
}

void SysTick_Init(void){
    SysTick->LOAD = 0x00FFFFFF;
    SysTick->CTRL = 0x00000005;
}

void SysTick_Wait(uint32_t n){
    SysTick->LOAD = n-1;
    SysTick->VAL = 0;
    while((SysTick->CTRL&0x00010000)==0);
}

void SysTick_Wait1ms(uint32_t delay){
    for(uint32_t i=0; i<delay; i++){
        SysTick_Wait(FREQTRAB/1000);
    }
}

void PLL_Config(){
	
	// 1. Habilitar PWR (para escala de voltaje)
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;


    // 2. Escala de voltaje: VOS = 1 (máximo rendimiento)
    // PWR_CR1_VOS bits[15:14] = 0b11
    PWR->CR1 |= PWR_CR1_VOS;

    // 3. Encender HSI y esperar a que esté estable
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)) { /* espera activa */ }

    // 4. Configurar Flash:

    FLASH->ACR = FLASH_ACR_ARTEN | FLASH_ACR_PRFTEN  // Prefetch/art accelerator
               | FLASH_ACR_LATENCY_4WS;

    // 5. Configurar PLL:
    //   PLLCFGR bits:
    //   PLLSRC = HSI, PLLM = 16, PLLN = 320, PLLP = 4, PLLQ = 8
    RCC->PLLCFGR  = (8   << RCC_PLLCFGR_PLLM_Pos)   // M = 16
                  | (432  << RCC_PLLCFGR_PLLN_Pos)   // N = 320
                  | (1    << RCC_PLLCFGR_PLLP_Pos)   // PLLP bits = 0b01 ? P = 4
                  | (8    << RCC_PLLCFGR_PLLQ_Pos)   // Q = 8
                  | RCC_PLLCFGR_PLLSRC_HSI;         // Fuente = HSI

    // 6. Encender PLL y esperar a que esté listo
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)) { /* espera activa */ }

    // 7. Seleccionar el PLL como SYSCLK y esperar confirmación
    RCC->CFGR |= RCC_CFGR_SW_PLL;  // SW bits = 0b10
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
        /* espera activa */
    }
	
}

int cuadrada(int offset, int amplitud){
    if (((offset + (amplitud / 2)) > 4095)) {
        amplitud = 2 * (4095 - offset);
    } else if(((offset - (amplitud / 2)) < 0 )){
        amplitud = 2 * offset;
    }
    v_dac = offset + (arriba? (amplitud / 2) : -(amplitud / 2));		
    return v_dac;
}

int cuadrada2(int offset2, int amplitud2){
    if (((offset2 + (amplitud2 / 2)) > 4095)) {
        amplitud2 = 2 * (4095 - offset2);
    } else if(((offset2 - (amplitud2 / 2)) < 0 )){
        amplitud2 = 2 * offset2;
    }
    v_dac2 = offset2 + (arriba2? (amplitud2 / 2) : -(amplitud2 / 2));		
    return v_dac2;
}

int Sierra(int offset, int amplitud, int N_pasos) {
    static int contador_pasos = 0;
    
    // Limitar amplitud para evitar overflow
    if (((offset + (amplitud / 2)) > 4095)) {     
        amplitud = 2 * (4095 - offset);   
    } else if(((offset - (amplitud / 2)) < 0 )){    
        amplitud = 2 * offset;  
    }
    
    // Calcular el valor actual basado en el contador de pasos
    int valor_min = offset - (amplitud / 2);
    int incremento_por_paso = amplitud / N_pasos;
    
    v_dac = valor_min + (contador_pasos * incremento_por_paso);
    
    // Incrementar contador y resetear cuando llegue al final
    contador_pasos++;
    if (contador_pasos >= N_pasos) {
        contador_pasos = 0;
    }
    
    return v_dac;
}

int Sierra2(int offset2, int amplitud2, int N_pasos2) {
    static int contador_pasos2 = 0;
    
    // Limitar amplitud para evitar overflow
    if (((offset2 + (amplitud2 / 2)) > 4095)) {     
        amplitud2 = 2 * (4095 - offset2);   
    } else if(((offset2 - (amplitud2 / 2)) < 0 )){    
        amplitud2 = 2 * offset2;  
    }
    
    // Calcular el valor actual basado en el contador de pasos
    int valor_min2 = offset2 - (amplitud2 / 2);
    int incremento_por_paso2 = amplitud2 / N_pasos2;
    
    v_dac2 = valor_min2 + (contador_pasos2 * incremento_por_paso2);
    
    // Incrementar contador y resetear cuando llegue al final
    contador_pasos2++;
    if (contador_pasos2 >= N_pasos) {
        contador_pasos2 = 0;
    }
    
    return v_dac2;
}

int pulso(int offset, int amplitud){
    // Verificar límites de amplitud
    if (((offset + (amplitud / 2)) > 4095)) {
        amplitud = 2 * (4095 - offset);
    } else if(((offset - (amplitud / 2)) < 0 )){
        amplitud = 2 * offset;
    }
    
    contador_pulso++;
    
    // Ciclo de trabajo: 10% alto, 90% bajo
    if(contador_pulso <= 1){        
        salida_pulso = 1;  // Alto por 1 período
    } else {
        salida_pulso = 0;  // Bajo por 9 períodos
    }
    
    if(contador_pulso >= 10){        
        contador_pulso = 0; 
    }    
    
    v_dac = offset + (salida_pulso ? (amplitud / 2) : -(amplitud / 2));	
    return v_dac;
}

int pulso2(int offset2, int amplitud2){
    // Verificar límites de amplitud
    if (((offset2 + (amplitud2 / 2)) > 4095)) {
        amplitud2 = 2 * (4095 - offset2);
    } else if(((offset2 - (amplitud2 / 2)) < 0 )){
        amplitud2 = 2 * offset2;
    }
    
    contador_pulso2++;
    
    // Ciclo de trabajo: 10% alto, 90% bajo
    if(contador_pulso2 <= 1){        
        salida_pulso2 = 1;  // Alto por 1 período
    } else {
        salida_pulso2 = 0;  // Bajo por 9 períodos
    }
    
    if(contador_pulso2 >= 10){        
        contador_pulso2 = 0; 
    }    
    
    v_dac2 = offset2 + (salida_pulso2 ? (amplitud2 / 2) : -(amplitud2 / 2));	
    return v_dac2;
}

int seno(int offset, int amplitud) {    
    if ((offset + amplitud/2) > 4095) {        
        amplitud = 2 * (4095 - offset);    
    } else if ((offset - amplitud/2) < 0) {        
        amplitud = 2 * offset;    
    }    
    
    indice_seno += paso_frecuencia;   
    uint32_t tabla_indice = (indice_seno >> 16) % NUM_PUNTOS;   
    double valor_seno = tabla_seno[tabla_indice];    
    int32_t valor_dac = offset + (int32_t)(amplitud/2 * (valor_seno));    
    
    if (valor_dac > 4095) valor_dac = 4095;    
    if (valor_dac < 0) valor_dac = 0;
   
    v_dac = (int)valor_dac;	
    return v_dac;
}

int seno2(int offset2, int amplitud2) {    
    if ((offset2 + amplitud2/2) > 4095) {        
        amplitud2 = 2 * (4095 - offset2);    
    } else if ((offset2 - amplitud2/2) < 0) {        
        amplitud2 = 2 * offset2;    
    }    
    
    indice_seno2 += paso_frecuencia2;   
    uint32_t tabla_indice = (indice_seno2 >> 16) % NUM_PUNTOS;   
    double valor_seno2 = tabla_seno[tabla_indice];    
    int32_t valor_dac2 = offset2 + (int32_t)(amplitud2/2 * (valor_seno2));    
    
    if (valor_dac2 > 4095) valor_dac2 = 4095;    
    if (valor_dac2 < 0) valor_dac2 = 0;
   
    v_dac2 = (int)valor_dac2;	
    return v_dac2;
}

void TIM2_Config(){
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
		if(onda == '4'){
		TIM2->PSC = 15;
		TIM2->ARR = 99;
		}else{
    TIM2->PSC = 1;
    TIM2->ARR = 99;
		}
    TIM2->EGR |= TIM_EGR_UG;
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= (1 << 0);
}

void TIM5_Config(){
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
		if(onda2 == '4'){
		TIM5->PSC = 15;
		TIM5->ARR = 99;
		}else{
    TIM5->PSC = 1;
    TIM5->ARR = 99;
		}
    TIM5->EGR |= TIM_EGR_UG;
    TIM5->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM5_IRQn);
    TIM5->CR1 |= (1 << 0);
}

extern "C" void TIM2_IRQHandler(void){    
    if(TIM2->SR & (1 << 0)){       
        TIM2->SR &= ~(1 << 0);
        switch(onda){
            case '1':
                v_DAC = cuadrada(offset, amplitud);
                arriba = !arriba;
                break;
						case '2':
                v_DAC = Sierra(offset, amplitud, N_pasos);
                break;
						case '3':
                v_DAC = pulso(offset, amplitud);
                break;
						case '4':
                v_DAC = seno(offset, amplitud);
                break;
        }
        DAC->DHR12R1 = v_DAC;
    }
}

int calcular_frecuencia(double frecuencia_param){
    double arr = 1;  // Cambiar de static a local
    switch(onda){
        case '1':
            if (frecuencia_param > 0) {							
                arr = ((FREQTRAB / (2 * frecuencia_param)) - 1) / 2;							
            }
            break;
						case '2':					         
            if (frecuencia_param > 0) {							
                if(frecuencia_param <= 10000.0){
                    N_pasos = 100;
                }else if(frecuencia_param > 10000.0 && frecuencia_param <= 100000.0){
                    N_pasos = 3;
                }else if(frecuencia_param > 100000.0 && frecuencia_param <= 1000000.0){
                    N_pasos = 3;
                }else {        
                    N_pasos = 3;
                }				
                arr = (108000000.0 / (frecuencia_param * N_pasos));           
            }
            break;
						case '3':
            if (frecuencia_param > 0) {
						    arr = ((FREQTRAB / (frecuencia_param * 20)) - 1);
            }
						break;
						case '4':
            arr = 1;
            if(frecuencia_param > 0) {
                paso_frecuencia = (uint32_t)((frecuencia_param * NUM_PUNTOS * (65535.0)) / 1160000);
            }
            break;
    }
    if (arr < 0) arr = 0;
    else if (arr > 4294967295) arr = 4294967295;
    return (int)arr;
}

extern "C" void TIM5_IRQHandler(void){    
    if(TIM5->SR & (1 << 0)){       
        TIM5->SR &= ~(1 << 0);
        switch(onda2){
            case '1':
                v_DAC2 = cuadrada2(offset2, amplitud2);
                arriba2 = !arriba2;
                break;
						case '2':
                v_DAC2 = Sierra2(offset2, amplitud2, N_pasos2);
                break;
						case '3':
                v_DAC2 = pulso(offset2, amplitud2);
                break;
						case '4':
                v_DAC2 = seno2(offset2, amplitud2);
                break;
        }
        DAC->DHR12R2 = v_DAC2;
    }
}

int calcular_frecuencia2(double frecuencia_param2){
    double arr = 1;  // Cambiar de static a local
    switch(onda2){
        case '1':
            if (frecuencia_param2 > 0) {							
                arr = ((FREQTRAB / (2 * frecuencia_param2)) - 1) / 2;							
            }
            break;
						case '2':					         
            if (frecuencia_param2 > 0) {							
                if(frecuencia_param2 <= 10000.0){
                    N_pasos2 = 100;
                }else if(frecuencia_param2 > 10000.0 && frecuencia_param2 <= 100000.0){
                    N_pasos2 = 3;
                }else if(frecuencia_param2 > 100000.0 && frecuencia_param2 <= 1000000.0){
                    N_pasos2 = 3;
                }else {        
                    N_pasos2 = 3;
                }				
                arr = (108000000.0 / (frecuencia_param2 * N_pasos2));           
            }
            break;
						case '3':
            if (frecuencia_param2 > 0) {
						    arr = ((FREQTRAB / (frecuencia_param2 * 20)) - 1);
            }
						break;
						case '4':
            arr = 1;
            if(frecuencia_param2 > 0) {
                paso_frecuencia2 = (uint32_t)((frecuencia_param2 * NUM_PUNTOS * (65535.0)) / 1160000);
            }
            break;
    }
    if (arr < 0) arr = 0;
    else if (arr > 4294967295) arr = 4294967295;
    return (int)arr;
}