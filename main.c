/* ----------------------------------------------------------------------------
 UFABC - Disciplina Sistemas Microprocessados - SuP - 2017.3
 Programa:  Prat_04
 Autor:     Joao Ranhel
 Descricao: Interrupcao Externa (input port) e "debouncing"
 Funcionamento:
No projeto da placa Display UFABCrev1 ha 1 botao no pino B6 (ver projeto):
Quando precionado, gerar uma INTERRUPCAO externa (criar ISR p/ isso
(observar a necessidade de debouning).
Quando precionado, trocar o padrao de acionamento dos LEDs
 Usa:
 a) PC13 - pino 13 da GPIOC   (iniciar bus APB2_C + GPIO + com estrutura dados)
 b) PC14 - pino 14 da GPIOC   (led na placa DUr1, = acima)
 c) PB7  - pino 7 da GPIOB    (controle de intensidade via PWM)
 d) PB6  - pino 6 da GPIOB    (input - pedido de interrupcao)
 OBS.1 : na pasta /src/ modificamos "stm32f1xx_it.c"
 OBS.2 : na bib. /Libraries/STM32F10X_StdPeriph_Driver vamos modificar os arqs:
 stm32f10x_exti.h ; stm32f10x_exti.h  para atender interrupcoes.
/  --------------------------------------------------------------------------*/

#include "stm32f10x.h"
#include "stm32f1xx_it.h"      // prots serv interrupt (ISR, em stm32f1xx_it.c)

#define FREQ_TICK 1000
#define DT_Clock_SHCP 0.25
#define DT_Clock_STCP 0.5
#define DT_INPUT 0.5
#define DT_PC13 500


// tempos das unidades em milisegundos
#define time_dec_seg 100
#define time_uni_seg 1000
#define time_dez_seg 10000
#define time_uni_min 60000

uint32_t millis(void);         // prot fn millis (em stm32f1xx_it.c)
uint32_t Ler_Modo_Oper(void);  // prot fn ModoOper (em stm32f1xx_it.c)

GPIO_InitTypeDef GPIO_PtB;             // estrutura dados config GPIO_B
EXTI_InitTypeDef IntExt_PtB;           // declara estrut IntExt_PtB
NVIC_InitTypeDef NVIC_PtB;             // estrutura NVIC port B

/**
 * Configura a linha 6 para receber a interrupção
 */
void setup_INT_externa(void)
{
  // configurar dados estrutura interrupcao
  IntExt_PtB.EXTI_Line = EXTI_Line6;           // qual linha pede interrupcao
  IntExt_PtB.EXTI_Mode = EXTI_Mode_Interrupt;  // modo interrupcao
  IntExt_PtB.EXTI_Trigger = EXTI_Trigger_Rising;  // dispara no falling_edge
  IntExt_PtB.EXTI_LineCmd = ENABLE;           // habilita ext_int
  EXTI_Init(&IntExt_PtB);  // chama função que inicializa interrupcao

  // configurar o NVIC (estrutura e funcao no misc.h e misc.c)
  NVIC_PtB.NVIC_IRQChannel = EXTI9_5_IRQn;       // IRQ_ext linha EXTI9_5_IRQn
  NVIC_PtB.NVIC_IRQChannelPreemptionPriority = 1;// prioridade preempt
  NVIC_PtB.NVIC_IRQChannelSubPriority = 1;       // prioridade 1
  NVIC_PtB.NVIC_IRQChannelCmd = ENABLE;		     // habilitada
  NVIC_Init(&NVIC_PtB);    // chama fn que inicializa NVIC
}

/**
 *  fn que configura da GPIO B
 */
void setup_GPIOs(void)
{
  // habilitar o clock para GPIOB
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  // habilitar o clock do periferico de interrupcao
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);


  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

  GPIO_InitTypeDef GPIO_PtC;             // estrutura dados config GPIO_C
  // setup dos vals pars da estrutura de dados da GPIO_C e iniciar
  GPIO_PtC.GPIO_Pin =  GPIO_Pin_14 ;
  GPIO_PtC.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_PtC.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOC, &GPIO_PtC);


  GPIO_PtB.GPIO_Pin = GPIO_Pin_13 + GPIO_Pin_1 + GPIO_Pin_0 + GPIO_Pin_7;
  GPIO_PtB.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_PtB.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_PtB);

  // setup dos vals p/ estrutura de GPIO_B e iniciar pino 6 INPUT
  GPIO_PtB.GPIO_Pin = GPIO_Pin_6;
  GPIO_PtB.GPIO_Mode = GPIO_Mode_IN_FLOATING; // modo input pino B6
  GPIO_PtB.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOB, &GPIO_PtB);

  // config que GPIOB pino 6 sera usado para gerar EXT INT
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource6);
  EXTI_ClearITPendingBit(EXTI_Line6);
}

/**
 *  fn que configura o SysTick c/ 1ms  (freq_systema/frequency)
 */
void setup_systick(uint16_t f_tick)
{
  RCC_ClocksTypeDef RCC_Clocks;
  RCC_GetClocksFreq(&RCC_Clocks);
  (void) SysTick_Config(RCC_Clocks.HCLK_Frequency / f_tick);
}

/**
 * Decodifica um numero decimal para um display de 7-segmentos.
 * Paramentros vet[]: vetor com bits p/ habilitar o numero no display.
 * 			   n: numero que deseja codificar.
 */
void Decod(int vet[], int n){

    switch(n){

        case 0:
            vet[0] = 1;	// ponto
            vet[1] = 1;	// g
            vet[2] = 0;	// f
            vet[3] = 0;	// e
            vet[4] = 0;	// d
            vet[5] = 0;	// c
            vet[6] = 0;	// b
            vet[7] = 0;	// a
            break;
        case 1:
            vet[0] = 1;
            vet[1] = 1;
            vet[2] = 1;
            vet[3] = 1;
            vet[4] = 1;
            vet[5] = 0;
            vet[6] = 0;
            vet[7] = 1;
            break;
        case 2:
            vet[0] = 1;
            vet[1] = 0;
            vet[2] = 1;
            vet[3] = 0;
            vet[4] = 0;
            vet[5] = 1;
            vet[6] = 0;
            vet[7] = 0;
            break;
        case 3:
            vet[0] = 1;  // Led Apagado
            vet[1] = 0;	 // Led Acesso
            vet[2] = 1;
            vet[3] = 1;
            vet[4] = 0;
            vet[5] = 0;
            vet[6] = 0;
            vet[7] = 0;
            break;
        case 4:
            vet[0] = 1;
            vet[1] = 0;
            vet[2] = 0;
            vet[3] = 1;
            vet[4] = 1;
            vet[5] = 0;
            vet[6] = 0;
            vet[7] = 1;
            break;
        case 5:
            vet[0] = 1;		// Ponto
            vet[1] = 0;		// g
            vet[2] = 0;		// f
            vet[3] = 1;		// e
            vet[4] = 0;		// d
            vet[5] = 0;		// c
            vet[6] = 1;		// b
            vet[7] = 0;		// a
            break;
        case 6:
            vet[0] = 1;
            vet[1] = 0;
            vet[2] = 0;
            vet[3] = 0;
            vet[4] = 0;
            vet[5] = 0;
            vet[6] = 1;
            vet[7] = 0;
            break;
        case 7:
            vet[0] = 1;
            vet[1] = 1;
            vet[2] = 1;
            vet[3] = 1;
            vet[4] = 1;
            vet[5] = 0;
            vet[6] = 0;
            vet[7] = 0;
            break;
        case 8:
            vet[0] = 1;
            vet[1] = 0;
            vet[2] = 0;
            vet[3] = 0;
            vet[4] = 0;
            vet[5] = 0;
            vet[6] = 0;
            vet[7] = 0;
            break;
        case 9:
            vet[0] = 1;
            vet[1] = 0;
            vet[2] = 0;
            vet[3] = 1;
            vet[4] = 0;
            vet[5] = 0;
            vet[6] = 0;
            vet[7] = 0;
            break;
        default:
            vet[0] = 0;
            vet[1] = 1;
            vet[2] = 1;
            vet[3] = 1;
            vet[4] = 1;
            vet[5] = 1;
            vet[6] = 1;
            vet[7] = 1;
    }


}

/**
 *  Habilita um dos 4 displays.
 *  Paramentos: vet[]: vetor que contêm os bits necessários para habilitar o display.
 *  		    n: O display que deseja habilitar.
 */
void Display(int vet[], int n){

    switch(n){

        case 1:
            vet[8] = 0;
            vet[9] = 0;
            vet[10] = 0;
            vet[11] = 0;
            vet[12] = 0;
            vet[13] = 0;
            vet[14] = 0;
            vet[15] = 1;
            break;
        case 2:
            vet[8] = 0;
            vet[9] = 0;
            vet[10] = 0;
            vet[11] = 0;
            vet[12] = 0;
            vet[13] = 0;
            vet[14] = 1;
            vet[15] = 0;
            break;
        case 3:
            vet[8] = 0;
            vet[9] = 0;
            vet[10] = 0;
            vet[11] = 0;
            vet[12] = 0;
            vet[13] = 1;
            vet[14] = 0;
            vet[15] = 0;
            break;
        case 4:
            vet[8] = 0;
            vet[9] = 0;
            vet[10] = 0;
            vet[11] = 0;
            vet[12] = 1;
            vet[13] = 0;
            vet[14] = 0;
            vet[15] = 0;
            break;
    }
}

/**
 *  Serializa sequência de 8bits
 */
void Serializa(int Acender_display, int vector[], int  dec_seg, int unid_seg, int dez_seg, int unid_min){

    switch(Acender_display){

        case 4:
        	Decod(vector, unid_min);
        	vector[0]=0;

        	Display(vector, 4);
            break;
        case 3:
        	Decod(vector, dez_seg);

        	Display(vector, 3);
            break;
        case 2:
        	Decod(vector, unid_seg);
        	vector[0]=0;
        	Display(vector, 2);
            break;
        case 1:
        	Decod(vector, dec_seg);
        	Display(vector, 1);
            break;
    }

}


/**
 *  Obtêm os decimos, unidades e dezenas de segundos
 *  e as unidades de minutos.
 */
void Time(int *dec_seg, int *unid_seg, int *dez_seg, int *unid_min,  int *prox_time_dec_seg, int active_time, int zera){
	// Decimos de segundos
	if(millis()>= *prox_time_dec_seg && active_time==1){
		*prox_time_dec_seg = millis() + time_dec_seg;
		*dec_seg += 1;
		if(*dec_seg==10){
			 *unid_seg+=1;
			 *dec_seg=0;
			 if(*unid_seg==10){
				 *dez_seg+=1;
				 *dec_seg=0;
				 *unid_seg=0;
				 if(*dez_seg==6){
					 *unid_min+=1;
					 *dec_seg=0;
					 *unid_seg=0;
					 *dez_seg=0;
					 if(*unid_min==10){
						 *dec_seg=0; *unid_seg=0; *dez_seg=0; *unid_min=0;
					 }
				 }
			 }
		}
	}


	if(zera==1){
		*dec_seg=0; *unid_seg=0;*dez_seg=0; *unid_min=0;
	}
  }

/**
 * funcao principal do programa
 */
int main(void)
{
	  int Acender_display=1;
	  int active_time =0, active_Clock_SHCP=0, zera=0;
  setup_GPIOs();                  // setup GPIOs interface LEDs
  setup_systick(FREQ_TICK);            // set timers p/ 1 ms  (1000 Hz)
  setup_INT_externa();                 // setup Interrupcao externa

  //Vars e flags de controle do programa para 3 tarefas no superloop...
  int prx_Clock_SHCP = (millis() + DT_Clock_SHCP); // calc prox toggle de Clock_SHCP
  int prx_Clock_STCP = (millis() + DT_Clock_STCP);
  int prx_INPUT = 0;
  int16_t f1=0, f0=0, contador_PHCP=0, active_STCP =0;

  int vector[16] = {0};
  GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_RESET); // Clock PHCP
  GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_RESET); // Clock PTCP

  // estados para controlar qual display entra primeiro



  int dec_seg=0, unid_seg=0, dez_seg=0, unid_min=0;
  int prox_time_dec_seg = millis() + time_dec_seg;
  int prox_time_uni_seg = millis() + time_uni_seg;
  int prox_time_dez_seg = millis() + time_dez_seg;
  int prox_time_uni_min = millis() + time_uni_min;
  int 	prx_PC13 = (millis() + DT_PC13), fpisc13=0;



  // entra no loop infinito
  while (1)
  {

	  switch (Ler_Modo_Oper()) // dependendo do estado da var Modo_Oper
	  {
	  	  case 1:                // no modo 1
	  		  //led piscando

	  		if (millis()>=prx_PC13)
	  			{
	  				if(fpisc13==0) {
	  					fpisc13 = 1;
	  					GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_SET);  // apaga o LED PC13
	  				} else {
	  					fpisc13 = 0;
	  					GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_RESET); // acende o LED PC13
	  				}
	  				prx_PC13 = (millis() + DT_PC13); // prox tempo para PC13

	  		     }






	  		  //-----------------------------

	  		  active_Clock_SHCP =1;
	  		  active_time =1;
	  		  zera =0;
	  		  break;
	  	  case 2:                // no modo 2

	  		GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_RESET);
	  		GPIO_WriteBit(GPIOC, GPIO_Pin_14, Bit_SET);
	  		  active_time =0;


	  		  break;
	  	  default:               // no modo 0
	  		GPIO_WriteBit(GPIOC, GPIO_Pin_14, Bit_RESET);
	  		GPIO_WriteBit(GPIOB, GPIO_Pin_7, Bit_SET);
	  		  Time(&dec_seg, &unid_seg, &dez_seg, &unid_min, &prox_time_dec_seg, active_time, 1);
	  		  active_Clock_SHCP =1;
	  		  break;
	  }


	  if (millis()>=prx_Clock_SHCP && active_Clock_SHCP == 1){
		  // calc proximo tempo para clock...
		  prx_Clock_SHCP = (millis() + DT_Clock_SHCP);
		  if(f1==0 && contador_PHCP<=17){
			  f1 = 1;
			  GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_SET);
			  contador_PHCP++;
		  }else{
			  f1 = 0;
			  GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_RESET);
			  if(contador_PHCP==17){
				  active_STCP = 1;
			  }
		  }
	  }

	  if (millis()>=prx_Clock_STCP && active_STCP ){
		  // calc proximo tempo para clock...
		  prx_Clock_STCP = (millis() + DT_Clock_STCP);
		  if(f0==0){
			  f0 = 1;
	  		  GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_SET);
	  	  }else{
	  		  f0 = 0;
			  GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_RESET);
			  active_STCP = 0;
			  contador_PHCP = 0;

			  Time(&dec_seg, &unid_seg, &dez_seg, &unid_min, &prox_time_dec_seg, active_time, zera);
			  Serializa(Acender_display, vector, dec_seg, unid_seg, dez_seg, unid_min);

			  // Controla quais displays vão ser acessos! deu trabalho fazer isso.
			  if(unid_seg==0 && dez_seg==0 && unid_min==0){
				  Acender_display=1;
			  }else if(dez_seg==0 && unid_min==0){
				  if(Acender_display<2){
					  Acender_display+=1;
				  }else{
					  Acender_display =1;
				  }

			  }else if(unid_min==0){
				  if(Acender_display<3){
					  Acender_display+=1;
				  }else{
					  Acender_display =1;
				  }

			  }else if(unid_min!=0){
				  if(Acender_display<4){
					  Acender_display+=1;
				  }else{
					  Acender_display =1;
				  }

			  }


	  	  }
	  }
	  if (millis()>=prx_INPUT){
	  		  // calc proximo tempo para clock...
	  		  prx_INPUT = (millis() + DT_INPUT);
	  	   	  if(vector[contador_PHCP-1] == 1 ){
	  	  	  	  GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_SET);
	  	   	  }else{
	  	  	  	  GPIO_WriteBit(GPIOB, GPIO_Pin_13, Bit_RESET);
	  		  }
	  }






  }
}
