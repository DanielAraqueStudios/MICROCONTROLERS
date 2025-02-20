#include <stdio.h>
#include <stm32f4xx.h>





int t =10000000;


void configurarPuertoA(){

    //---------Para configurar el reloj del GPIOA
        RCC->AHB1ENR |= (1<<0);
    
    //---------Para configurar el pin 5 de la GPIOA como salida 
        
        // Configuro los pines 0 a 6 como salida
        GPIOA->MODER |= 0x1555; 
        // Configuro los valores en push-pull (Es el de defecto gg)
        GPIOA->OTYPER |= 0x0; 
        // Configuro los pines 0 a 6 como velocidad media (01)
        GPIOA->OSPEEDR |= 0x1555; 
        // Configuro los pines en pull up
        GPIOA->PUPDR |= 0xAAA; 
    }
    
    //SETTING GPIOS FOR LETTERS
    void letraA() {
        GPIOA->ODR = 0x8; //Dejo pine 3 en 1
    }

    void letraB() {
        GPIOA->ODR = 0x7C;
    }

    void letraC() {
        GPIOA->ODR = 0x39;
    }

    void letraD() {
        GPIOA->ODR = 0x3E;
    }

    void letraE() {
        GPIOA->ODR = 0x79;
    }

    void letraF() {
        GPIOA->ODR = (1<<1) | (1<<2) | (1<<3) ;
    }

    void letraG() {
        GPIOA->ODR = 0x3D;
    }

    void letraH() {
        GPIOA->ODR = (1<<0) | (1<<3) ;
    }

    void letraI() {
        GPIOA->ODR = 0x66; // Dejo pines 1,2,5 y 6 en 1
    }

    void letraJ() {
        GPIOA->ODR = 0x71; // Dejo los pines 0, 4, 5 y 6 en 1
    }

    void letraK() {
        GPIOA->ODR = 0x75;
    }

    void letraL() {
        GPIOA->ODR = 0x47; // Dejo pines 0,1,2 y 6 en 1
    }
    
    void letraM() {
        GPIOA->ODR = (1<<3) | (1<<6)  | (1<<5)  | (1<<1) // Dejo pines 0,2,4 y 6 en 1
    }

    void letraN() {
        GPIOA->ODR = 0xB; //Dejo pines 0,1,3 y 5 en 1
    }

    void letraO() {
        GPIOA->ODR = 0x3F;
    }

    void letraP() {
        GPIOA->ODR =   (1<<3) | | (1<<2) ;
    }

    void letraQ() {
        GPIOA->ODR = 0x3F;
    }

    void letraR() {
        GPIOA->ODR = 0x50;
    }

    void letraS() {
        GPIOA->ODR = 0x6D;
    }

    void letraT() {
        GPIOA->ODR = 0x78;
    }

    void letraU() {
        GPIOA->ODR = 0x63; // Dejo pines 0,1,5,6 en 1
    }

    void letraV() {
        GPIOA->ODR = 0x3E;
    }


    
    //SETTING TIME FOR LETTERS TO BE DISPLAYED
		
		void  timeA(){
			
			for (int i=0; i<t; i++){
				
				letraA();
				
				
				
			}				
		}
        
        void  timeB(){
			
			for (int i=0; i<t; i++){
				
				letraB();
				
				
				
			}				
		}
        
        void  timeC(){
            for (int i=0; i<t; i++){
				
				letraC();
				
				
				
			}	
        }

        void  timeD(){
            for (int i=0; i<t; i++){
                letraC();
            }
     }


     void  timeE (){
        for (int i=0; i<t; i++){
            
            letraC();
            
            
            
        }	
    }

    void  timeF(){
        for (int i=0; i<t; i++){
            letraF();
        }
    }

    void  timeG(){
        for (int i=0; i<t; i++){
            letraG();
        }
    }

    void  timeH(){
        for (int i=0; i<t; i++){
            letraH();
        }
    }

    void  timeI(){
        for (int i=0; i<t; i++){
            letraI();
        }
    }

    void  timeJ(){
        for (int i=0; i<t; i++){
            letraJ();
        }
    }

    void  timeK(){
        for (int i=0; i<t; i++){
            letraK();
        }
    }

    void  timeL(){
        for (int i=0; i<t; i++){
            letraL();
        }
    }

    void  timeM(){
        for (int i=0; i<t; i++){
            letraM();
        }
    }

    void  timeN(){
        for (int i=0; i<t; i++){
            letraN();
        }
    }

    void  timeO(){
        for (int i=0; i<t; i++){
            letraO();
        }
    }

    void  timeP(){
        for (int i=0; i<t; i++){
            letraP();
        }
    }

    void  timeQ(){
        for (int i=0; i<t; i++){
            letraQ();
        }
    }

    void  timeR(){
        for (int i=0; i<t; i++){
            letraR();
        }
    }

    void  timeS(){
        for (int i=0; i<t; i++){
            letraS();
        }
    }

    void  timeT(){
        for (int i=0; i<t; i++){
            letraT();
        }
    }

    void  timeU(){
        for (int i=0; i<t; i++){
            letraU();
        }
    }

    /*

    void  timeV(){
        for (int i=0; i<t; i++){
            letraV();
        }
    }

    void  timeW(){
        for (int i=0; i<t; i++){
            letraW();
        }
    }

    void  timeX(){
        for (int i=0; i<t; i++){
            letraX();
        }
    }   

    void  timeY(){
        for (int i=0; i<t; i++){
            letraY();
        }
    }

    void  timeZ(){
        for (int i=0; i<t; i++){
            letraZ();
        }
    }

    */





		

    int main() {
        //------Configuro puerto A------
        configurarPuertoA();

        while(1){
           
					timeA();
					
                    timeB();
                    timeC();
					timeD();
                    timeE();
                    timeF();
                    timeG();    
                    timeH();
                    timeI();
                    timeJ();
                    timeK();
                    timeL();
                    timeM();
                    timeN();
                    timeO();
                    timeP();
                    timeQ();
                    timeR();
                    timeS();
                    timeT();
                    timeU();
                    
					
        }
    }
