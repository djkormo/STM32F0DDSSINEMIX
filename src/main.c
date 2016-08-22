
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "diag/Trace.h"
#include "algorithm.h"
#include "config.h"
#include "resources.h"
#include "GFX.h"
#include "PCD8544F0.h"

#include <math.h>
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_tim.h>
#include <stm32f0xx_dac.h>
#include <stm32f0xx_dma.h>
#include <stm32f0xx_adc.h>
#include <stm32f0xx_spi.h>

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

ADC_InitTypeDef 			ADC_InitStructure;
DMA_InitTypeDef         	DMA_InitStructure;
NVIC_InitTypeDef			NVIC_InitStructure;
TIM_TimeBaseInitTypeDef 	TIM_TimeBaseStructure;
TIM_OCInitTypeDef 			TIM_OCInitStructure;
DMA_InitTypeDef 			DMA_InitStructure;

/* global consts */
#define LEDToggleValue ((uint16_t) 3000)

// size of ACD coversion channels
#define ACDSIZE 3

//boundaries for DAC - 12-bit resolution

#define MININPUTADC 0.0
#define MAXINPUTADC 4095.0
// adres od ADC1 DR register
#define ADC1_DR_Address    0x40012440

// for SPI
#define LCD_DC			GPIO_Pin_1
#define LCD_CE			GPIO_Pin_2
#define LCD_RST			GPIO_Pin_3
const int usingADC=1;
const int usingLEDS=0;

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

//Variables to store current states
volatile uint8_t Converted = 0, CChan = 0;



void InitBoard();
void InitDACTimers();

void InitACDTimers();
void InitSPI();

// here the main function begins
int main()
{

// interrupts, clocks, general settings

// INTPUT PA0, PA1, PA2 -> Pots for ADC
// OUTPUT PA4 -> for DAC (audio)

InitBoard();
//*TODO
// init SPI for LCD
//InitSPI();

if (usingADC)
{
// configure ADC

	InitACDTimers();

}

// configure DAC
InitDACTimers();

// infinite loop
  while (1)
    {


    }
  return 0;
}





void InitBoard()
{
			//Initialize system and ensure core clock is up to date
			//* TODO  setting master clock to 48Mhz
	        SystemInit();
	        SystemCoreClockUpdate();

	          //Enable GPIO Clock
	        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	        //Initialize LEDs
	        LEDs.GPIO_Pin = GreenLED | BlueLED;
	        LEDs.GPIO_Mode = GPIO_Mode_OUT;
	        LEDs.GPIO_OType = GPIO_OType_PP;
	        LEDs.GPIO_PuPd = GPIO_PuPd_NOPULL;
	        LEDs.GPIO_Speed = GPIO_Speed_Level_3;
	        GPIO_Init(GPIOC, &LEDs);

	        //Initialize and enable timer to have a maximum period for 16bits
	        //running at a frequency of 48MHz/(65535*(1000 – 1)) = 0.732Hz
	        TTB.TIM_ClockDivision = TIM_CKD_DIV1;
	        TTB.TIM_CounterMode = TIM_CounterMode_Up;
	        TTB.TIM_Period = 48000; // 65535
	        TTB.TIM_Prescaler = 2;
	        TTB.TIM_RepetitionCounter = 0;
	        TIM_TimeBaseInit(TIM1, &TTB);
	        TIM_Cmd(TIM1, ENABLE);


	        GPIO_SetBits(LEDGPIO, BlueLED);
	        GPIO_SetBits(LEDGPIO, GreenLED);
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
		  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;// | GPIO_Pin_5;
		  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
		  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
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



void InitACDTimers(void)
{

 //https://my.st.com/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/Flat.aspx?RootFolder=/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/STM32F0%20ADC-DMA%20touble&FolderCTID=0x01200200770978C69A1141439FE559EB459D7580009C4E14902C3CDE46A77F0FFD06506F5B&currentviews=35

  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
  ADC_InitTypeDef ADC_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;


  	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	//Configure PA0, PA1 and PA2 as analog inputs
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;


  /* ADC1 DeInit */
  ADC_DeInit(ADC1);

  /* ADC1 Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_TIM1, ENABLE);

  /* DMA1 clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* TIM2 Configuration */
  TIM_DeInit(TIM1);

  /* Time base configuration */
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 1000000) - 1; // 1 MHz, from 48 MHz
  TIM_TimeBaseStructure.TIM_Period = 2000 - 1; // 500 Hz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* Output Compare PWM Mode configuration */
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; /* low edge by default */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0x01;
  TIM_OC4Init(TIM1, &TIM_OCInitStructure);

  /* TIM1 enable counter */
  TIM_Cmd(TIM1, ENABLE);

  /* Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);

  /* DMA1 Channel1 Config */
  DMA_DeInit(DMA1_Channel1);

  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ACDconversions[0];
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 3;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);

  /* DMA1 Channel1 enable */
  DMA_Cmd(DMA1_Channel1, ENABLE);

  /* ADC DMA request in circular mode */
  ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);

  /* Enable DMA1 Channel1 Half Transfer and Transfer Complete interrupt */
  DMA_ITConfig(DMA1_Channel1, DMA_IT_HT, ENABLE);
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

  /* Enable ADC_DMA */
  ADC_DMACmd(ADC1, ENABLE);

  /* Initialize ADC structure */
  ADC_StructInit(&ADC_InitStructure);

  /* Configure the ADC1 in continous mode withe a resolutuion equal to 12 bits  */
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
  ADC_InitStructure.ADC_ExternalTrigConv =  ADC_ExternalTrigConv_T1_CC4;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
  ADC_Init(ADC1, &ADC_InitStructure);


	//Configure channels to be converted
	ADC_ChannelConfig(ADC1, ADC_Channel_0, ADC_SampleTime_239_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_1, ADC_SampleTime_239_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_2, ADC_SampleTime_239_5Cycles);


  /* ADC Calibration */
  ADC_GetCalibrationFactor(ADC1);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Wait the ADCEN flag */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN));

  /* ADC1 regular Software Start Conv */
  ADC_StartOfConversion(ADC1);


  NVIC_InitTypeDef NVIC_InitStructure;

  /* Enable and set DMA1_Channel1 Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}



void InitSPI()
{

	 /*
	  * SETUP NOKIA 5110
	  * 1-> RST -> PC3
	  * 2-> CE -> PC2
	  * 3 -> RST -> PC1
	  * 4-> CLK (SPI_SCK) -> PA5
	  * 5-> DIN (SPI_MOSI) -> PA7
	  * 6 -> VCC -> 3.3V
	  * 7 -> BL -> NONE
	  * 8 -> GND - 0 V (GROUND)
	  */

	/*
	GPIO_InitStructure.GPIO_Pin = LCD_DC|LCD_CE|LCD_RST;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_SetBits(GPIOC, LCD_CE|LCD_RST);
	*/
	/*
	lcd_reset();

	lcd_cmd(0x21);
	lcd_cmd(0x14);
	lcd_cmd(0x80 | 0x2f); //Ustawienie kontrastu
	lcd_cmd(0x20);
	lcd_cmd(0x0c);
	*/
	PCD8544_InitSetup();
}


//  handling TIM3 interrupt for DAC conversion

void TIM3_IRQHandler()
{



    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
    {

    	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
    			DACtimer=TIM_GetCounter(TIM3);
    			/*	based on
    			 http://amarkham.com/?p=49
    			 */
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
    	    	  if (!usingADC)
    	    	  {
    	    	  DAC1OutputData = (uint16_t)
    	    			  (accumulator1step+accumulator2step+accumulator3step)/3.0;
    	    	  }
    	    	  else
    	    	  {
    	    	  DAC1OutputData = (uint16_t)
						(accumulator1step+accumulator2step+accumulator3step)/3.0;
    	    	  }
    	    	  // sending 12-bits output signal
    	    	  DAC_SetChannel1Data(DAC_Align_12b_R,DAC1OutputData);
    	    	  // changing accumulator register in time ....
    	    	  if (!usingADC)
    	    	  {
    	    	  accumulator1r+=R>>4;
    	    	  accumulator2r-=R>>4;
    	    	  accumulator3r+=R>>4;
    	    	  }

    	    	  else
    	    	  {
    	    	  	  	accumulator1r=(uint32_t)257374*
    	    	  	  		rangeScaleLinear(ACDconversions[0],0,4095,10,5000);


    	    	  		accumulator2r=(uint32_t)257374*
    	    	  	    	rangeScaleLinear(ACDconversions[1],0,4095,10,5000);
    	    	  	  //accumulator2r-=R>>4;
    	    	  		accumulator3r=(uint32_t)257374*
    	    	  	    	rangeScaleLinear(ACDconversions[2],0,4095,10,2000);
    	    	  	  //accumulator3r+=R>>4;
    	    	  }
    	    	  	  	//accumulator1r+=R>>4;
    	    	  		//accumulator2r=157374*ACDconversions[1];
    	    	  		//accumulator3r=157374*ACDconversions[2];

    }

}


//Interrupt for DMA1 Channel 1. Used to notify the end of
//conversions.
void DMA1_Channel1_IRQHandler(void){
	if(DMA_GetITStatus(DMA1_IT_TC1)){
		DMA_ClearITPendingBit(DMA1_IT_TC1);
		//Converted = 1;
	}
}



void ADC1_COMP_IRQHandler(void){
	//Check for end of conversion
	if(ADC_GetITStatus(ADC1, ADC_IT_EOC)){
		//Clear interrupt bit
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);
		//Switch statement dependent on current channel. First
		//channel is initialized as zero.
		switch(CChan){
		case 0:
			ACDconversions[0] = ADC_GetConversionValue(ADC1);
			if (usingLEDS)
				{
				if (ACDconversions[0]>2000)
				{
					GPIO_ResetBits(LEDGPIO, BlueLED);
				}
				else
				{
					GPIO_SetBits(LEDGPIO, BlueLED);
				}
			}
			CChan = 1;
			break;
		case 1:
			ACDconversions[1] = ADC_GetConversionValue(ADC1);
			if (usingLEDS)
			{
				if (ACDconversions[1]>2000)
						{
							GPIO_ResetBits(LEDGPIO, GreenLED);
						}
						else
						{
							GPIO_SetBits(LEDGPIO, GreenLED);
						}
			}
			CChan = 2;
			break;
		case 2:
			ACDconversions[2] = ADC_GetConversionValue(ADC1);
			if (usingLEDS)
			{
					if (ACDconversions[2]>2000)
								{
									GPIO_ResetBits(LEDGPIO, BlueLED);
									GPIO_ResetBits(LEDGPIO, GreenLED);
								}
								else
								{
									GPIO_SetBits(LEDGPIO, BlueLED);
									GPIO_SetBits(LEDGPIO, GreenLED);
								}
			}
			CChan = 0;
			Converted = 1;
			break;
		}
	}
}




#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
