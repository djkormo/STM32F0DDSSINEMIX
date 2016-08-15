
#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "algorithm.h"
#include "config.h"
#include "resources.h"


#include <math.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_tim.h>
#include <stm32f0xx_dac.h>
#include <stm32f0xx_dma.h>
#include <stm32f0xx_adc.h>


// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"




//Global initialization structures
GPIO_InitTypeDef 			LEDs;
TIM_TimeBaseInitTypeDef 	TTB;
DAC_InitTypeDef         	DAC_InitStructure;
GPIO_InitTypeDef			GPIO_InitStructure;
NVIC_InitTypeDef         	DACNVIC_InitStructure;

ADC_InitTypeDef 			A;
GPIO_InitTypeDef 			G;
NVIC_InitTypeDef 			N;
DMA_InitTypeDef 			D;



/* global consts */

/* global variables*/

volatile uint16_t R =573;


volatile uint32_t accumulator1=0;
volatile uint16_t accumulator1angle=0;
volatile uint16_t accumulator1step=0;
volatile uint32_t accumulator1r=15737418;
// for DAC output data (12-bit) from 0 to 4095
volatile uint16_t DAC1OutputData ;

volatile uint32_t DACtimer;
void InitBoard();
void InitDACTimers();
void InitADCTimers();

int main(int argc, char* argv[])
{

// interrupts, clocks, general settings

InitBoard();

// configure DAC
InitDACTimers();

// configure ADC
InitADCTimers();

// infinite loop
  while (1)
    {

    }
  return 0;
}





void InitBoard()
{

};

void InitDACTimers()
{
	/* DAC  clock enable */

		  RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);

		  /* TIM3 for DAC */
		  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 ,ENABLE);

		  /* for output PINS */
		  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);


		  // Configure PA.04/05 (DAC) as output -------------------------
		  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
		  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
		  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		  GPIO_Init(GPIOA, &GPIO_InitStructure);


		  /* Fill DAC InitStructure */



		  	  DAC_InitStructure.DAC_Trigger = DAC_Trigger_T3_TRGO;
		 	  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;


		 	  DAC_Init(DAC_Channel_1, &DAC_InitStructure);

		 	  //(+) Enable the DAC channel using DAC_Cmd()
		 	  DAC_Cmd(DAC_Channel_1, ENABLE);


			  //TTB.TIM_ClockDivision = TIM_CKD_DIV1;
	          TTB.TIM_CounterMode = TIM_CounterMode_Up;
	          TTB.TIM_RepetitionCounter = 0;
	          TTB.TIM_Prescaler = 5; //  4800 kHz // was 300-1
	          TTB.TIM_Period = 5; //1Hz; // was 10-1

	          TIM_TimeBaseInit(TIM3, &TTB);
	          TIM_Cmd(TIM3, ENABLE);





	          /* http://visualgdb.com/tutorials/arm/stm32/timers/ */
	          TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	          TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
	          /* http://forbot.pl/blog/artykuly/programowanie/kurs-stm32-7-liczniki-timery-w-praktyce-pwm-id8459 */


	          NVIC_SetPriority(TIM3_IRQn, 0);
	          NVIC_EnableIRQ (TIM3_IRQn);
	          DACNVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	          DACNVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	          DACNVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	          NVIC_Init(&DACNVIC_InitStructure);
};

void InitADCTimers()
{

};


//  handling TIM3 interrupt for DAC conversion

void TIM3_IRQHandler()
{



    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
    {

    	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

    			DACtimer=TIM_GetCounter(TIM3);

    	 	 	  accumulator1+=accumulator1r;
    	    	  //  first 10 (32 -22) bits -> lut table index
    	    	  accumulator1angle=(uint16_t)(accumulator1>>22);
    	    	  accumulator1step = Sine1024_12bit[accumulator1angle];



    	    	  DAC1OutputData = (uint16_t)

    	    	  							0.99*(
    	    	  									accumulator1step

    	    	  							);


    	    	  							// sending 12-bits output signal
    	    	  			DAC_SetChannel1Data(DAC_Align_12b_R,DAC1OutputData);

    	    	  			accumulator1r+=R>>4;

    }

}

// handling DMA for ADC
void DMA1_Channel1_IRQHandler(void){
	if(DMA_GetITStatus(DMA1_IT_TC1)){
		DMA_ClearITPendingBit(DMA1_IT_TC1);

	}
}



#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
