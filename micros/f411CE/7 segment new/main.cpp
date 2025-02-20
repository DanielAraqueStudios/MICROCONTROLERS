#include <stdio.h>
#include <stm32f4xx.h>





int t =1000000;


void configurarPuertoA(){

    //---------Para configurar el reloj del GPIOA,B y C
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


        //CONFIGURE LEDS OUTPUT

        // B10 B12 B13 B14 B15 A8 A9 A10 A11 A12 A15 B3 B4 B5 B6 B7 B9 C15 C14


        GPIOB->MODER |= (1<<20);


        GPIOB->OTYPER |= (0<<10);


        GPIOB->OSPEEDR |= (1<<20);
        


        GPIOB->PUPDR |= (1<<21);









    }

		
		
    void configurarPuertoB(){
        //---------Para configurar el reloj del GPIOB
        RCC->AHB1ENR |= (1<<1);
        
        //---------Para configurar los pines 0 al 2 de la GPIOB como entrada
        GPIOB->MODER |= 0x0; // Configuro los pines 0 al 2 como entrada (00)
        GPIOB->OTYPER |= 0x0; // Configuro los pines 0 al 2 como push-pull (default)
        GPIOB->OSPEEDR |= 0x0; // Configuro los pines 0 al 2 como velocidad baja (00) porque son entradas, lo que importa es la velocidad de los pines de salida
        GPIOB->PUPDR |= 0x2A; // Configuro los pines 0 al 2 como pull-down (10)
    }
    
    //SETTING GPIOS FOR LETTERS
		
		
    void letraA() {
        GPIOA->ODR = 0x8; //Dejo pine 3 en 1
    }

    void letraB() {
        GPIOA->ODR = (1<<0)| (1<<1);
    }

    void letraC() {
        GPIOA->ODR = (1<<1) | (1<<2) |(1<<6);
    }

    void letraD() {
        GPIOA->ODR = (1<<5)|(1<<0);
    }

    void letraE() {
        GPIOA->ODR = (1<<1)|(1<<2);
    }

    void letraF() {
        GPIOA->ODR = (1<<1) | (1<<2) | (1<<3) ;
    }

    void letraG() {
        GPIOA->ODR = (1<<4);
    }

    void letraH() {
        GPIOA->ODR = (1<<0) | (1<<3) ;
    }

    void letraI() {
        GPIOA->ODR = (1<<1)|(1<<6)|(1<<2)|(1<<3)|(1<<0); // Dejo pines 1,2,5 y 6 en 1
    }

    void letraJ() {
        GPIOA->ODR = 0x71; // Dejo los pines 0, 4, 5 y 6 en 1
    }

    void letraK() {
        GPIOA->ODR = (1<<0)|(1<<2);
    }

    void letraL() {
        GPIOA->ODR = 0x47; // Dejo pines 0,1,2 y 6 en 1
    }
    
    void letraM() {
        GPIOA->ODR = (1<<3) | (1<<6)  ;
    }

    void letraN() {
        GPIOA->ODR = (1<<5)|(1<<0)|(1<<1)|(1<<3); //Dejo pines 0,1,3 y 5 en 1
    }

    void letraO() {
        GPIOA->ODR = (1<<6);
    }

    void letraP() {
        GPIOA->ODR =   (1<<3) |  (1<<2) ;
    }

    void letraQ() {
        GPIOA->ODR = (1<<3)|(1<<4);
    }

    void letraR() {
        GPIOA->ODR = (1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<5);
    }

    void letraS() {
        GPIOA->ODR = (1<<4)|(1<<1);
    }

    void letraT() {
        GPIOA->ODR = (1<<0)|(1<<1)|(1<<2);
    }

    void letraU() {
        GPIOA->ODR = (1<<6)|(1<<0);
    }

    void letraV() {
        
			GPIOA->ODR = 0x63; // Dejo pines 0,1,5,6 en 1
    }
		
		
		void letraY(){
			GPIOA->ODR = (1<<4)|(1<<0);
		}
		
		
		void letraW(){
		GPIOA->ODR = (1<<0)|(1<<6)|(1<<2)|(1<<4);
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
                letraD();
            }
     }


     void  timeE (){
        for (int i=0; i<t; i++){
            
            letraE();
            
            
            
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
		
		
		void timeV(){
			for (int i=0;i<t;i++){
				letraV();
			}
		}
		
		
		void timeY(){
			for (int i=0;i<t;i++){
				letraY();
			}
		}
		
				void timeW(){
			for (int i=0;i<t;i++){
				letraW();
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



//names

void julian(){
	
	timeJ();
	timeU();
	timeL();
	timeI();
	timeA();
	timeN();
}


void daniel(){
	
	timeD();
	timeA();
	timeN();
	timeI();
	timeE();
	timeL();
}

void andres(){
	
	
	timeA();
	timeN();
	timeD();
	timeR();
	timeE();
	timeS();
	
}

void rosas(){
        timeR();
        timeO();
        timeS();
        timeA();
        timeS();
    }

    void garcia(){
        timeG();
        timeA();
        timeR();
        timeC();
        timeI();
        timeA();
    }

    void araque(){
        timeA();
        timeR();
        timeA();
        timeQ();
        timeU();
        timeE();
    }

void seq1(){
	for (int i=0; i<t; i++){
		GPIOA->ODR= (1<<0);
	}
	for (int i=0; i<t; i++){
		GPIOA->ODR= (1<<1);
	}
	for (int i=0; i<t; i++){
		GPIOA->ODR= (1<<2);
	}
	for (int i=0; i<t; i++){
		GPIOA->ODR= (1<<3);
	}
	for (int i=0; i<t; i++){
		GPIOA->ODR= (1<<4);
	}
	for (int i=0; i<t; i++){
		GPIOA->ODR= (1<<5);
	}
	for (int i=0; i<t; i++){
		GPIOA->ODR= (1<<0);
	}
	for (int i=0; i<t; i++){
		GPIOA->ODR= (1<<5);
	}
	for (int i=0; i<t; i++){
		GPIOA->ODR= (1<<4);
	}
	for (int i=0; i<t; i++){
		GPIOA->ODR= (1<<3);
	}
	for (int i=0; i<t; i++){
		GPIOA->ODR= (1<<2);
	}
	for (int i=0; i<t; i++){
		GPIOA->ODR= (1<<1);
	}
	for (int i=0; i<t; i++){
		GPIOA->ODR= (1<<0);



	}
}

void seq2(){
	for (int i=0; i<t; i++){
		GPIOA->ODR = (1<<5)|(1<<0)|(1<<1);
	}
	
		for (int i=0; i<t; i++){
		GPIOA->ODR = (1<<3)|(1<<2)|(1<<4);
}
			for (int i=0; i<t; i++){
		GPIOA->ODR = (1<<5)|(1<<0)|(1<<1);
	}
	
		for (int i=0; i<t; i++){
		GPIOA->ODR = (1<<3)|(1<<2)|(1<<4);
}
			for (int i=0; i<t; i++){
		GPIOA->ODR = (1<<5)|(1<<0)|(1<<1);
	}
	
		for (int i=0; i<t; i++){
		GPIOA->ODR = (1<<3)|(1<<2)|(1<<4);
}
			for (int i=0; i<t; i++){
		GPIOA->ODR = (1<<5)|(1<<0)|(1<<1);
	}
	
		for (int i=0; i<t; i++){
		GPIOA->ODR = (1<<3)|(1<<2)|(1<<4);
}
		
}


void seq3(){
	for (int i=0; i<t; i++){
		GPIOA->ODR = (1<<1)|(1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6);
	}
		for (int i=0; i<t; i++){
		GPIOA->ODR = (1<<0)|(1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6);
	}
		for (int i=0; i<t; i++){
		GPIOA->ODR = (1<<0)|(1<<1)|(1<<3)|(1<<4)|(1<<5)|(1<<6);
	}
				for (int i=0; i<t; i++){
		GPIOA->ODR = (1<<0)|(1<<1)|(1<<2)|(1<<4)|(1<<5)|(1<<6);
	}
					for (int i=0; i<t; i++){
		GPIOA->ODR = (1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<5)|(1<<6);
	}
						for (int i=0; i<t; i++){
		GPIOA->ODR = (1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)|(1<<6);
	}
						
	
	
}

void seq4(){
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





void ledbtnon(){

    // B10 B12 B13 B14 B15 A8 A9 A10 A11 A12 A15 B3 B4 B5 B6 B7 B9 C15 C14

    for (int i=0; i<t; i++){
    GPIOB->ODR = (1<<10);
    }


}



int main() {
        //------Configuro puerto A y B------
        configurarPuertoA();
        configurarPuertoB();
    
        while(1) {
            // Leer el estado de los pines 0 al 2 del puerto B
            int estado = GPIOB->IDR & 0x7;
    
            // Llamar a la función correspondiente según el estado de los pines
            switch (estado) {
                case 0b000:
                    
								timeK();
								timeR();
								timeA();
								timeV();
								timeE();
								timeN();
								
								 seq3();
								
								timeT();
								timeO();
								timeN();
								timeY();
								timeS();
								timeT();
								timeA();
								timeR();
								timeK();
								
								
								seq3();
								timeB();
								timeR();
								timeU();
								timeC();
								timeE();
								timeW();
								timeA();
								timeN();
								timeE();
								timeR();
								
								
                    break;
                case 0b001:
                    
                // B10 B12 B13 B14 B15 A8 A9 A10 A11 A12 A15 B3 B4 B5 B6 B7 B9 C15 C14
                ledbtnon();

                    break;
                case 0b010:
                    daniel();
                    break;
                case 0b011:
                    andres();
                    break;
                case 0b100:
                    rosas();
                    break;
                case 0b101:
                    garcia();
                    break;
                case 0b110:
                    araque();
                    break;
                case 0b111:
                    seq2();
                    break;
            }
    
            // Secuencias libres
            // seq1();
            // seq2();
            // seq3();
            // seq4();
        }
    }
