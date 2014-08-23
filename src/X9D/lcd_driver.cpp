/**
  ******************************************************************************
  * @file    Project/lcd/lcd.c 
  * @author  X9D Application Team
  * @Hardware version V0.2
  * @date    11-July-2012
  * @brief   This file provides LCD Init and botom drivers.
  * *
  ******************************************************************************
*/

#include "../ersky9x.h"

#include "x9d\stm32f2xx.h"
#include "x9d\stm32f2xx_gpio.h"
#include "x9d\stm32f2xx_rcc.h"
#include "x9d\hal.h"
#include "x9d\aspi.h"

#include "timers.h"
#include "lcd.h"
#include "logicio.h"
#include "myeeprom.h"
#ifndef SIMU
#include "CoOS.h"
#endif

//#define	WriteData(x)	 AspiData(x)
//#define	WriteCommand(x)	 AspiCmd(x)
#ifdef REVPLUS
#define CONTRAST_OFS 160
#else
#define CONTRAST_OFS 12
#endif

#define __no_operation     __NOP

void setupSPIdma( void ) ;

extern uint8_t DisplayBuf[] ;
extern uint8_t Main_running ;

void Set_Address(uint8_t x, uint8_t y)
{
  AspiCmd(x&0x0F);	//Set Column Address LSB CA[3:0]
  AspiCmd((x>>4)|0x10);	//Set Column Address MSB CA[7:4]
  
  AspiCmd((y&0x0F)|0x60);	//Set Row Address LSB RA [3:0]
  AspiCmd(((y>>4)&0x0F)|0x70);    //Set Row Address MSB RA [7:4]

}

#ifdef REVPLUS

OS_FlagID LcdFlag ;
volatile uint8_t DmaDone = 0 ;
uint8_t DmaDebugDone ;
uint16_t DmaDebugCount ;
uint16_t DmaDebugNDTR ;

// New hardware SPI driver for LCD
void initLcdSpi()
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_LCD, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_LCD_RST, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_LCD_NCS, ENABLE);
	
  RCC->APB1ENR |= RCC_APB1ENR_SPI3EN ;    // Enable clock
	// APB1 clock / 2 = 133nS per clock
	SPI3->CR1 = 0 ;		// Clear any mode error
	SPI3->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_CPOL | SPI_CR1_CPHA ;
	SPI3->CR2 = 0 ;
	SPI3->CR1 |= SPI_CR1_MSTR ;	// Make sure in case SSM/SSI needed to be set first
	SPI3->CR1 |= SPI_CR1_SPE ;

	configure_pins( PIN_LCD_NCS, PIN_OUTPUT | PIN_PORTA | PIN_PUSHPULL | PIN_OS25 | PIN_NO_PULLUP ) ;
	configure_pins( PIN_LCD_RST, PIN_OUTPUT | PIN_PORTD | PIN_PUSHPULL | PIN_OS25 | PIN_NO_PULLUP ) ;
	configure_pins( PIN_LCD_A0,  PIN_OUTPUT | PIN_PORTC | PIN_PUSHPULL | PIN_OS25 | PIN_NO_PULLUP ) ;
	configure_pins( PIN_LCD_MOSI|PIN_LCD_CLK, PIN_PORTC | PIN_PUSHPULL | PIN_OS50 | PIN_NO_PULLUP | PIN_PER_6 | PIN_PERIPHERAL ) ;

	setupSPIdma() ;
	LcdFlag = CoCreateFlag( TRUE, 0 ) ;
}


uint8_t GreyDisplayBuf[DISPLAY_W*DISPLAY_H/8*4] ;

void setupSPIdma()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN ;			// Enable DMA1 clock
 	// Chan 0, 8-bit wide, Medium priority, memory increments
	DMA1_Stream7->CR &= ~DMA_SxCR_EN ;		// Disable DMA
	DMA1->HIFCR = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7 ; // Write ones to clear bits
	DMA1_Stream7->CR =  DMA_SxCR_PL_0 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 ;
	DMA1_Stream7->PAR = (uint32_t) &SPI3->DR ;
//	DMA1_Stream7->M0AR = (uint32_t) GreyDisplayBuf ;
	DMA1_Stream7->FCR = 0x05 ; //DMA_SxFCR_DMDIS | DMA_SxFCR_FTH_0 ;
//	DMA1_Stream7->NDTR = DISPLAY_W*DISPLAY_H/8*4 ;

	NVIC_SetPriority( DMA1_Stream7_IRQn, 2 ) ; // Lower priority interrupt
	NVIC_EnableIRQ(DMA1_Stream7_IRQn) ;
}

void startSpiDma()
{
	DmaDone = 0 ;
	DMA1_Stream7->M0AR = (uint32_t) GreyDisplayBuf ;
	DMA1_Stream7->NDTR = DISPLAY_W*DISPLAY_H/8*4 ;
	DMA1->HIFCR = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7 ; // Write ones to clear bits
	DMA1_Stream7->CR |= DMA_SxCR_EN ;		// Enable DMA
	SPI3->CR2 |= SPI_CR2_TXDMAEN ;
	DMA1_Stream7->CR |= DMA_SxCR_TCIE ;				// Enable interrupt
}

extern "C" void DMA1_Stream7_IRQHandler()
{
//	DMA1_Stream7->CR &= ~DMA_SxCR_EN ;		// Disable DMA
	DMA1_Stream7->CR &= ~DMA_SxCR_TCIE ;		// Stop interrupt
	DMA1->HIFCR = DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7 ; // Write ones to clear flags
	SPI3->CR2 &= ~SPI_CR2_TXDMAEN ;
	DmaDebugNDTR = DMA1_Stream7->NDTR ;
	DmaDone = 1 ;
	DmaDebugCount += 1 ;

	if ( Main_running )
	{
		CoEnterISR() ; // Enter the interrupt
		isr_SetFlag( LcdFlag ) ;		// Tell task transfer finished
		CoExitISR() ; // Exit the interrupt
	}
}

void convertDisplay()
{
	uint8_t *d = GreyDisplayBuf ;
	for (uint32_t y=0; y<DISPLAY_H; y += 2)
	{
    uint8_t *p = &DisplayBuf[(y>>3)*DISPLAY_W];
    uint8_t mask = (1 << (y%8));
		for (uint32_t x=0; x<DISPLAY_W; x += 1 )
		{
      uint32_t data ;
			data = 0 ;
			if ( p[x] & mask ) data = 0x0F ;
			if (p[x] & (mask<<1) ) data += 0xF0 ;	
			*d++ = data ;
		}			 
	}
}
#endif

#ifdef REVPLUS
void refreshDisplay()
{
	
	Set_Address( 0, 0 ) ;
	
  LCD_NCS_LOW() ;  
	GPIOC->BSRRL = PIN_LCD_A0 ;			// A0 high

	convertDisplay() ;

//	setupSPIdma() ;
	DmaDebugDone = 0 ;
	startSpiDma() ;

//	DMA1_Stream7->CR |= DMA_SxCR_EN ;		// Enable DMA
//	SPI3->CR2 |= SPI_CR2_TXDMAEN ;
//	uint32_t x ;
	if ( Main_running )
	{
		CoWaitForSingleFlag( LcdFlag, 10 ) ;
		DmaDebugDone = 0x80 ;
	}

//	if ( x == E_OK )
//	{
//	}

//	while ( ( DMA1->HISR & DMA_HISR_TCIF7 ) == 0 )
//	{
//		// wait
//		if ( DmaDone )
//		{
//			DmaDebugNDTR = DMA1_Stream7->NDTR ;
//			DmaDebugDone = 1 ;
//		}
//	}
//	DmaDebugDone |= 0x20 ;

	while ( DmaDone == 0 )
	{
		// wait
//		if ( DMA1->HISR & DMA_HISR_TCIF7 )
//		{
//			SPI3->CR2 &= ~SPI_CR2_TXDMAEN ;
//			DMA1_Stream7->CR &= ~DMA_SxCR_EN ;		// Disable DMA
//			break ;
//		}
	}
	DmaDebugDone |= 1 ;
	if ( DMA1_Stream7->CR & DMA_SxCR_EN )
	{
		DmaDebugDone |= 0x10 ;
	}

//	if ( DMA1_Stream7->NDTR )
//	{
//		while ( ( DMA1->HISR & DMA_HISR_TCIF7 ) == 0 )
//		{
//			// wait
//		}
//		DmaDebugDone |= 0x20 ;
//	}

//	SPI3->CR2 &= ~SPI_CR2_TXDMAEN ;
//	DMA1_Stream7->CR &= ~DMA_SxCR_EN ;		// Disable DMA

	while ( ( SPI3->SR & SPI_SR_TXE ) == 0 )
	{
			
	} // Last byte being sent
	while ( SPI3->SR & SPI_SR_BSY )
	{
		// wait
	}
  GPIOA->BSRRL = PIN_LCD_NCS ;		// CS high
}
#endif

#ifndef REVPLUS
void refreshDisplay()
{  
  for (uint32_t y=0; y<DISPLAY_H; y++)
	{
    uint8_t *p = &DisplayBuf[(y>>3)*DISPLAY_W];
    uint8_t mask = (1 << (y%8));
		
		GPIO_TypeDef *gpiod = GPIOD ;
		uint32_t *bsrr = (uint32_t *)&gpiod->BSRRL ;

		Set_Address(0, y);
    AspiCmd(0xAF);
    
		gpiod->BSRRL = PIN_LCD_CLK ;		// Clock high
		gpiod->BSRRL = PIN_LCD_A0 ;			// A0 high
    gpiod->BSRRH = PIN_LCD_NCS ;		// CS low

		for (uint32_t x=0; x<DISPLAY_W; x+=2)
		{
      uint32_t data ;
			data = 0 ;
			if ( p[x] & mask )
			{
				data = 0xF0 ;
			}
			if (p[x+1] & mask )
			{
				data += 0x0F ;
			}	
			
        if(data&0x80)
        {
					gpiod->BSRRL = PIN_LCD_MOSI ;
        }
				else
				{
					gpiod->BSRRH = PIN_LCD_MOSI ;
				}
				gpiod->BSRRH = PIN_LCD_CLK ;		// Clock low

        if(data&0x40)
        {
					gpiod->BSRRL = PIN_LCD_MOSI | PIN_LCD_CLK ;
        }
				else
				{
					*bsrr = (PIN_LCD_MOSI<<16) | PIN_LCD_CLK ;
				}
				__no_operation() ;
				gpiod->BSRRH = PIN_LCD_CLK ;		// Clock low

        if(data&0x20)
        {
					gpiod->BSRRL = PIN_LCD_MOSI | PIN_LCD_CLK ;
        }
				else
				{
					*bsrr = (PIN_LCD_MOSI<<16) | PIN_LCD_CLK ;
				}
				__no_operation() ;
				gpiod->BSRRH = PIN_LCD_CLK ;		// Clock low

        if(data&0x10)
        {
					gpiod->BSRRL = PIN_LCD_MOSI | PIN_LCD_CLK ;
        }
				else
				{
					*bsrr = (PIN_LCD_MOSI<<16) | PIN_LCD_CLK ;
				}
				__no_operation() ;
				gpiod->BSRRH = PIN_LCD_CLK ;		// Clock low

        if(data&0x08)
        {
					gpiod->BSRRL = PIN_LCD_MOSI | PIN_LCD_CLK ;
        }
				else
				{
					*bsrr = (PIN_LCD_MOSI<<16) | PIN_LCD_CLK ;
				}
				__no_operation() ;
				gpiod->BSRRH = PIN_LCD_CLK ;		// Clock low

        if(data&0x04)
        {
					gpiod->BSRRL = PIN_LCD_MOSI | PIN_LCD_CLK ;
        }
				else
				{
					*bsrr = (PIN_LCD_MOSI<<16) | PIN_LCD_CLK ;
				}
				__no_operation() ;
				gpiod->BSRRH = PIN_LCD_CLK ;		// Clock low

        if(data&0x02)
        {
					gpiod->BSRRL = PIN_LCD_MOSI | PIN_LCD_CLK ;
        }
				else
				{
					*bsrr = (PIN_LCD_MOSI<<16) | PIN_LCD_CLK ;
				}
				__no_operation() ;
				gpiod->BSRRH = PIN_LCD_CLK ;		// Clock low

        if(data&0x01)
        {
					gpiod->BSRRL = PIN_LCD_MOSI | PIN_LCD_CLK ;
        }
				else
				{
					*bsrr = (PIN_LCD_MOSI<<16) | PIN_LCD_CLK ;
				}
				__no_operation() ;
				gpiod->BSRRH = PIN_LCD_CLK ;		// Clock low
				__no_operation() ;
				gpiod->BSRRL = PIN_LCD_CLK ;		// Clock high

		}
    gpiod->BSRRL = PIN_LCD_NCS ;		// CS high
		gpiod->BSRRL = PIN_LCD_A0 ;
    AspiData(0);
  }
}
#endif

uint16_t BacklightBrightness ;

#ifdef REVPLUS
uint16_t BacklightBrightness_white ;

void backlight_w_on()
{
	TIM4->CCR4 = 100 - BacklightBrightness_white ;
}

void backlight_w_off()
{
	TIM4->CCR4 = 0 ;
}

void backlight_on()
{
	TIM4->CCR2 = 100 - BacklightBrightness ;
}

void backlight_off()
{
	TIM4->CCR2 = 0 ;
}

void backlight_set( uint16_t brightness, uint16_t w_or_b )
{
	if ( w_or_b )
	{ // blue
		BacklightBrightness = brightness ;
		TIM4->CCR2 = 100 - BacklightBrightness ;
	}
	else
	{ // white
		BacklightBrightness_white = brightness ;
		TIM4->CCR4 = 100 - BacklightBrightness_white ;
	}
}

/**Init the Backlight GPIO */
static void LCD_BL_Config()
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOBL, ENABLE);
  
	GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_BL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOBL, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOBL, GPIO_PinSource_BL ,GPIO_AF_TIM4);

  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_BLW;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOBLW, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOBLW, GPIO_PinSource_BLW ,GPIO_AF_TIM4);

  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN ;    // Enable clock
	TIM4->ARR = 100 ;
	TIM4->PSC = (Peri1_frequency*Timer_mult2) / 10000 - 1 ;		// 100uS from 30MHz
	TIM4->CCMR1 = TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 ;	// PWM
	TIM4->CCMR2 = TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 ;	// PWM
	TIM4->CCER = TIM_CCER_CC4E | TIM_CCER_CC2E ;
	
	BacklightBrightness = 40 ;
	TIM4->CCR2 = BacklightBrightness ;
	TIM4->CCR4 = BacklightBrightness_white ;
	TIM4->EGR = 0 ;
	TIM4->CR1 = TIM_CR1_CEN ;				// Counter enable
}

// Init the Haptic
void initHaptic()
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOHAPTIC, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_HAPTIC;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOHAPTIC, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOHAPTIC, GPIO_PinSource_HAPTIC ,GPIO_AF_TIM10);

	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN ;		// Enable clock
	TIM10->ARR = 100 ;
	TIM10->PSC = (Peri2_frequency*Timer_mult2) / 10000 - 1 ;		// 100uS from 30MHz
	TIM10->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 ;	// PWM
	TIM10->CCER = TIM_CCER_CC1E ;
	
	TIM10->CCR1 = 0 ;
	TIM10->EGR = 0 ;
	TIM10->CR1 = TIM_CR1_CEN ;				// Counter enable
}

void hapticOff()
{
	TIM10->CCR1 = 0 ;
}

// pwmPercent 0-100
void hapticOn( uint32_t pwmPercent )
{
	if ( pwmPercent > 100 )
	{
		pwmPercent = 100 ;		
	}
	TIM10->CCR1 = pwmPercent ;
}

#else

void backlight_on()
{
	TIM10->CCR1 = 100 - BacklightBrightness ;
}

void backlight_off()
{
	TIM10->CCR1 = 0 ;
}

void backlight_set( uint16_t brightness )
{
	BacklightBrightness = brightness ;
	TIM10->CCR1 = 100 - BacklightBrightness ;
}


/**Init the Backlight GPIO */
static void LCD_BL_Config()
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOBL, ENABLE);
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_BL;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOBL, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOBL, GPIO_PinSource_BL ,GPIO_AF_TIM10);

	RCC->APB2ENR |= RCC_APB2ENR_TIM10EN ;		// Enable clock
	TIM10->ARR = 100 ;
	TIM10->PSC = (Peri2_frequency*Timer_mult2) / 10000 - 1 ;		// 100uS from 30MHz
	TIM10->CCMR1 = 0x60 ;	// PWM
	TIM10->CCER = 1 ;	
	
	BacklightBrightness = 80 ;
	TIM10->CCR1 = BacklightBrightness ;
	TIM10->EGR = 0 ;
	TIM10->CR1 = 1 ;
}
#endif

/** Init the anolog spi gpio
*/
static void LCD_Hardware_Init()
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_LCD, ENABLE);
#ifdef REVPLUS
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_LCD_RST, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_LCD_NCS, ENABLE);
#endif  
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /*!< Configure lcd CLK\ MOSI\ A0pin in output pushpull mode *************/
  GPIO_InitStructure.GPIO_Pin =PIN_LCD_MOSI | PIN_LCD_CLK | PIN_LCD_A0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIO_LCD, &GPIO_InitStructure);
  
  /*!< Configure lcd NCS pin in output pushpull mode ,PULLUP *************/
#ifdef REVPLUS
	GPIO_InitStructure.GPIO_Pin = PIN_LCD_NCS ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_Init(GPIO_LCD_NCS, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = PIN_LCD_RST ;
  GPIO_Init(GPIO_LCD_RST, &GPIO_InitStructure);

#else  
	GPIO_InitStructure.GPIO_Pin = PIN_LCD_NCS | PIN_LCD_RST ;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_Init(GPIO_LCD, &GPIO_InitStructure);
#endif  
}

#ifdef REVPLUS
static void LCD_Init()
{
//  LCD_BL_Config() ;
  /*Hardware Reset need delay*/
  /*LCD_RST_LOW();
    Delay(50);    
    LCD_RST_HIGH();*/

// From LCD Manufacturer
//  WriteCommand(0x2F);   //Internal pump control
//  Delay(20);
//  WriteCommand(0x24);   //Temperature compensation
//  WriteCommand(0xE9);    //set bias=1/10
//  WriteCommand(0x81);    //Set Vop
//  WriteCommand(0xBF);   //13.9v//(g_eeGeneral.contrast+CONTRAST_OFS);        //0--255
//  WriteCommand(0xA2);    //set line rate:28KLPS
//  WriteCommand(0x28);    //set pannel loading
//  WriteCommand(0x40);    //scroll line LSB
//  WriteCommand(0x50);    //SCROLL LINE MSB
//  WriteCommand(0x89);    //ram address control
//  WriteCommand(0xc0);    //LCD mapping control
//  WriteCommand(0x04);    //MX=0,MY=1
//  WriteCommand(0xd0);    //DISPLAY PATTERN = 16-SCALE GRAY
//  WriteCommand(0xF1);    //SET COM end
//  WriteCommand(0x3F);    //64

//  WriteCommand(0xF8);    //Set Window Program dIsable.
 
//  WriteCommand(0xF5);   //starting row address of RAM program window.PAGE1
//  WriteCommand(0x00);
//  WriteCommand(0xF7);   //end row address of RAM program window.PAGE32
//  WriteCommand(0x1f);
//  WriteCommand(0xF4);   //start column address of RAM program window.
//  WriteCommand(0x00);
//  WriteCommand(0xF6);   //end column address of RAM program window.SEG212
//  WriteCommand(0xd3);
	    
	AspiCmd(0x24);   //(5) Temperature compensation curve definition: 0x25 = -0.05%/oC
	AspiCmd(0x2b);   //(6) Panel loading set ,Internal VLCD.
  AspiCmd(0xEA);	 //(27) set bias=1/10
  AspiCmd(0x81);	 //(11) Set Vop + next byte
  AspiCmd(25+CONTRAST_OFS);		//0--255
	AspiCmd(0xA6);	//inverse display off
	AspiCmd(0xA2);	//line rates,28 Klps
  AspiCmd(0x84);	//Disable Partial Display
  AspiCmd(0xC8);	//SET N-LINE INVERSION
  AspiCmd(0x00);	//Disable NIV
  AspiCmd(0xF1);	//Set CEN
  AspiCmd(0x3F);	// 1/64DUTY
  AspiCmd(0xC0);	//(21) Set mapping
  AspiCmd(0x04);	// MY=1, MX=0, MSF=0
  AspiCmd(0x89);	//(15) WA=1,column (CA) increment (+1) first until CA reaches CA boundary, then RA will increment by (+1).
  AspiCmd(0xF8);	//Set Window Program Enable  ,inside modle
  AspiCmd(0xD0);	 //(23) SET 4 bits/pixel, pattern 0
	AspiCmd(0xF4);   //starting column address of RAM program window.
  AspiCmd(0x00);
  AspiCmd(0xF5);   //starting row address of RAM program window.
  AspiCmd(0x00);
  AspiCmd(0xF6);   //ending column address of RAM program window.
  AspiCmd(0xD3);
  AspiCmd(0xF7);   //ending row address of RAM program window.
  AspiCmd(0x3F);
	AspiCmd(0xAF);	// Active and 16-grey scale
	
	
	
//	AspiCmd(0x28);    //set panel loading
//	AspiCmd(0x40);    //scroll line LSB
//	AspiCmd(0x50);    //SCROLL LINE MSB
}
#else
static void LCD_Init()
{
  LCD_BL_Config() ;
  /*Hardware Reset need delay*/
  /*LCD_RST_LOW();
    Delay(50);    
    LCD_RST_HIGH();*/
    
  AspiCmd(0x25);   //Temperature compensation curve definition: 0x25 = -0.05%/oC
  AspiCmd(0x2b);   //Panel loading set ,Internal VLCD.
  AspiCmd(0xEA);	//set bias=1/10 :Command table NO.27
  AspiCmd(0x81);	//Set Vop
  AspiCmd(25+CONTRAST_OFS);		//0--255
  AspiCmd(0xA6);	//inverse display off
  AspiCmd(0xD1);	//SET RGB:Command table NO.21 .SET RGB or BGR.  D1=RGB
  AspiCmd(0xD5);	//set color mode 4K and 12bits  :Command table NO.22
  AspiCmd(0xA0);	//line rates,25.2 Klps
  AspiCmd(0xC8);	//SET N-LINE INVERSION
  AspiCmd(0x1D);	//Disable NIV
  AspiCmd(0xF1);	//Set CEN
  AspiCmd(0x3F);	// 1/64DUTY
  AspiCmd(0x84);	//Disable Partial Display
  AspiCmd(0xC4);	//MY=1,MX=0
  AspiCmd(0x89);	//WA=1,column (CA) increment (+1) first until CA reaches CA boundary, then RA will increment by (+1).

  AspiCmd(0xF8);	//Set Window Program Enable  ,inside modle
  AspiCmd(0xF4);   //starting column address of RAM program window.
  AspiCmd(0x00);
  AspiCmd(0xF5);   //starting row address of RAM program window.
  AspiCmd(0x60);
  AspiCmd(0xF6);   //ending column address of RAM program window.
  AspiCmd(0x47);
  AspiCmd(0xF7);   //ending row address of RAM program window.
  AspiCmd(0x9F);

  AspiCmd(0xAF);	//dc2=1,IC into exit SLEEP MODE,	 dc3=1  gray=ON 开灰阶	,dc4=1  Green Enhanc mode disabled	  绿色增强模式关

}
#endif

static void Delay(volatile unsigned int ms)
{
  volatile u8 i;
  while(ms != 0)
  {
    for(i=0;i<250;i++) {}
    for(i=0;i<75;i++) {}
    ms--;
  }
}

void lcd_init()
{
	GPIO_TypeDef *gpiod = GPIOD ;

  LCD_BL_Config();
  LCD_Hardware_Init();
#ifdef REVPLUS
	initLcdSpi() ;
#endif
  
	gpiod->BSRRL = PIN_LCD_RST ;		// RST high
  Delay(5);

  gpiod->BSRRH = PIN_LCD_RST ;		// RST low
  Delay(120); //11ms

	gpiod->BSRRL = PIN_LCD_RST ;		// RST high
  Delay(2500);
 
  AspiCmd(0xE2);			// System Reset

  Delay(2500);

  LCD_Init();
  Delay(120);
  AspiCmd(0xAF);	//dc2=1, IC into exit SLEEP MODE, dc3=1 gray=ON, dc4=1 Green Enhanc mode disabled

}

void lcdSetRefVolt(uint8_t val)
{
	AspiCmd(0x81);	//Set Vop
  AspiCmd(val+CONTRAST_OFS);		//0--255

}

