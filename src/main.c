
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
#define LEDToggleValue ((uint16_t) 3000)

// size of ACD coversion channels
#define ACDSIZE 3

//boundaries for DAC - 12-bit resolution

#define MININPUTADC 0.0
#define MAXINPUTADC 4095.0

/* global variables*/

struct Accumulator {
	uint32_t  value;
	uint16_t  angle;
	uint16_t  step;
	uint32_t  rvalue;
};


volatile uint16_t R =573;

struct Accumulator a1;
struct Accumulator a2;
struct Accumulator a3;


// 1st sine
volatile uint32_t accumulator1=0;
volatile uint16_t accumulator1angle=0;
volatile uint16_t accumulator1step=0;
volatile uint32_t accumulator1r=15737418;

//2nd sine
volatile uint32_t accumulator2=0;
volatile uint16_t accumulator2angle=0;
volatile uint16_t accumulator2step=0;
volatile uint32_t accumulator2r=25737418;

//3rd sine
volatile uint32_t accumulator3=0;
volatile uint16_t accumulator3angle=0;
volatile uint16_t accumulator3step=0;
volatile uint32_t accumulator3r=35737418;

// for DAC output data (12-bit) from 0 to 4095
volatile uint16_t DAC1OutputData ;

volatile uint32_t DACtimer;

volatile uint16_t CurrentTimerVal = 0;

// ADC conversions table
volatile uint16_t ACDconversions [ACDSIZE];


void InitBoard();
void InitDACTimers();
void InitADCTimers();
// here the main function begins
int main(int argc, char* argv[])
{

// interrupts, clocks, general settings

InitBoard();

// configure DAC
InitDACTimers();

// configure ADC
//InitADCTimers();

// infinite loop
  while (1)
    {
	  //Store current timer value in variable
	                      CurrentTimerVal = TIM_GetCounter(TIM3);
	                      	 // blinking leds
	                      //See if current timer value is more than LED toggle value
	                      if(CurrentTimerVal>LEDToggleValue){
	                                GPIO_SetBits(LEDGPIO, GreenLED);
	                                GPIO_ResetBits(LEDGPIO, BlueLED);

	                      }
	                      else{
	                                GPIO_ResetBits(LEDGPIO, GreenLED);
	                                GPIO_SetBits(LEDGPIO, BlueLED);
	                      }


	            }

  return 0;
}





void InitBoard()
{
	//Initialize system and ensure core clock is up to date
	          SystemInit();
	          SystemCoreClockUpdate();

	          InitDACTimers();
	          //InitADCTimers();
	          //Enable GPIO Clock
	          RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	          RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	          //Initialize LEDs
	          LEDs.GPIO_Pin = GreenLED | BlueLED;
	          LEDs.GPIO_Mode = GPIO_Mode_OUT;
	          LEDs.GPIO_OType = GPIO_OType_PP;
	          LEDs.GPIO_PuPd = GPIO_PuPd_NOPULL;
	          LEDs.GPIO_Speed = GPIO_Speed_Level_3;
	          GPIO_Init(LEDGPIO, &LEDs);

	          //Initialize and enable timer to have a maximum period for 16bits
	//running at a frequency of 48MHz/(65535*(1000 – 1)) = 0.732Hz
	          TTB.TIM_ClockDivision = TIM_CKD_DIV1;
	          TTB.TIM_CounterMode = TIM_CounterMode_Up;
	          TTB.TIM_Period = 48000; // 65535
	          TTB.TIM_Prescaler = 100;
	          TTB.TIM_RepetitionCounter = 0;
	          TIM_TimeBaseInit(TIM1, &TTB);
	          TIM_Cmd(TIM1, ENABLE);
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

	          // use as interrupt
	          NVIC_SetPriority(TIM3_IRQn, 0);
	          NVIC_EnableIRQ (TIM3_IRQn);
	          DACNVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	          DACNVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	          DACNVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	          NVIC_Init(&DACNVIC_InitStructure);
};

void InitADCTimers()
{
		// enable clock for DMA1
	    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1 , ENABLE);
	    // enable clock for ADC1
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	    ADC_DeInit(ADC1);




	    /* ADC Calibration */
	      ADC_GetCalibrationFactor(ADC1);

	      /* Convert the  ADC_Channnel_0  with  sampling time  - pot 1*/
	      ADC_ChannelConfig(ADC1, ADC_Channel_0 , ADC_SampleTime_28_5Cycles);

	      /* Convert the  ADC_Channnel_1  with  sampling time - pot 2*/
	      ADC_ChannelConfig(ADC1, ADC_Channel_1 , ADC_SampleTime_28_5Cycles);


	      /* Convert the  ADC_Channnel_2  with  sampling time - pot 3*/
	      ADC_ChannelConfig(ADC1, ADC_Channel_2 , ADC_SampleTime_28_5Cycles);


	      /* Enable ADC1 */
	      ADC_Cmd(ADC1, ENABLE);


	      // Wait until ADC enabled
	       while(ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN) == RESET);

	       /* ADC1 regular Software Start Conv */
	       ADC_StartOfConversion(ADC1);

	       // Configure GPIO as analog input (for pots)
	       GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	       GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	       GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	       GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	       GPIO_Init(GPIOA, &GPIO_InitStructure);


	       // configure interrupt for DMA1 channel
	      	NVIC_InitTypeDef NVIC_InitStructure;

	        /* Enable and set DMA1_Channel1 Interrupt */
	        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	        NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
	        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	        NVIC_Init(&NVIC_InitStructure);


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

    	    	  accumulator2+=accumulator2r;
    	    	  //  first 10 (32 -22) bits -> lut table index
    	    	  accumulator2angle=(uint16_t)(accumulator2>>22);
    	    	  accumulator2step = Sine1024_12bit[accumulator2angle];

    	    	  accumulator3+=accumulator3r;
    	    	  //  first 10 (32 -22) bits -> lut table index
    	    	  accumulator3angle=(uint16_t)(accumulator3>>22);
    	    	  accumulator3step = Sine1024_12bit[accumulator3angle];

    	    	  DAC1OutputData = (uint16_t)
    	    			  (accumulator1step+accumulator2step+accumulator3step)/3.0;

    	    	  			// sending 12-bits output signal
    	    	  			DAC_SetChannel1Data(DAC_Align_12b_R,DAC1OutputData);

    	    	  			accumulator1r+=R>>4;
    	    	  			accumulator2r-=R>>4;
    	    	  			accumulator3r+=R>>4;

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
