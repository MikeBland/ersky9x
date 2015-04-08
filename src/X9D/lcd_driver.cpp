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
#include <stdlib.h>

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
#ifdef REV9E
	TIM9->CCR2 = 100 - BacklightBrightness_white ;
#else
	TIM4->CCR4 = 100 - BacklightBrightness_white ;
#endif
}

void backlight_w_off()
{
#ifdef REV9E
	TIM9->CCR2 = 0 ;
#else
	TIM4->CCR4 = 0 ;
#endif
}

void backlight_on()
{
#ifdef REV9E
	TIM9->CCR1 = 100 - BacklightBrightness ;
#else
	TIM4->CCR2 = 100 - BacklightBrightness ;
#endif
}

void backlight_off()
{
#ifdef REV9E
	TIM9->CCR1 = 0 ;
#else
	TIM4->CCR2 = 0 ;
#endif
}

void backlight_set( uint16_t brightness, uint16_t w_or_b )
{
	if ( w_or_b )
	{ // blue
		BacklightBrightness = brightness ;
#ifdef REV9E
		TIM9->CCR1 = 100 - BacklightBrightness ;
#else
		TIM4->CCR2 = 100 - BacklightBrightness ;
#endif
	}
	else
	{ // white
		BacklightBrightness_white = brightness ;
#ifdef REV9E
		TIM9->CCR2 = 100 - BacklightBrightness_white ;
#else
		TIM4->CCR4 = 100 - BacklightBrightness_white ;
#endif
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

#ifdef REV9E
  GPIO_PinAFConfig(GPIOBL, GPIO_PinSource_BL ,GPIO_AF_TIM9);
#else
  GPIO_PinAFConfig(GPIOBL, GPIO_PinSource_BL ,GPIO_AF_TIM4);
#endif
  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_BLW;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOBLW, &GPIO_InitStructure);

#ifdef REV9E
  GPIO_PinAFConfig(GPIOBLW, GPIO_PinSource_BLW ,GPIO_AF_TIM9);
#else
  GPIO_PinAFConfig(GPIOBLW, GPIO_PinSource_BLW ,GPIO_AF_TIM4);
#endif

	BacklightBrightness = 40 ;
#ifdef REV9E
  RCC->APB2ENR |= RCC_APB2ENR_TIM9EN ;    // Enable clock
	TIM9->ARR = 100 ;
	TIM9->PSC = (PeripheralSpeeds.Peri2_frequency*PeripheralSpeeds.Timer_mult2) / 10000 - 1 ;		// 100uS from 30MHz
	TIM9->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 ;	// PWM
	TIM9->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E ;
	
	TIM9->CCR1 = BacklightBrightness ;
	TIM9->CCR2 = BacklightBrightness_white ;
	TIM9->EGR = 0 ;
	TIM9->CR1 = TIM_CR1_CEN ;				// Counter enable
#else
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN ;    // Enable clock
	TIM4->ARR = 100 ;
	TIM4->PSC = (PeripheralSpeeds.Peri1_frequency*PeripheralSpeeds.Timer_mult2) / 10000 - 1 ;		// 100uS from 30MHz
	TIM4->CCMR1 = TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 ;	// PWM
	TIM4->CCMR2 = TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 ;	// PWM
	TIM4->CCER = TIM_CCER_CC4E | TIM_CCER_CC2E ;
	
	TIM4->CCR2 = BacklightBrightness ;
	TIM4->CCR4 = BacklightBrightness_white ;
	TIM4->EGR = 0 ;
	TIM4->CR1 = TIM_CR1_CEN ;				// Counter enable
#endif
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
	TIM10->PSC = (PeripheralSpeeds.Peri2_frequency*PeripheralSpeeds.Timer_mult2) / 10000 - 1 ;		// 100uS from 30MHz
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
	TIM10->PSC = (PeripheralSpeeds.Peri2_frequency*PeripheralSpeeds.Timer_mult2) / 10000 - 1 ;		// 100uS from 30MHz
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

  AspiCmd(0xAF);	//dc2=1,IC into exit SLEEP MODE,	 dc3=1  gray=ON ���ҽ�	,dc4=1  Green Enhanc mode disabled	  ��ɫ��ǿģʽ��

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

#ifdef REV9E
// Top LCD

void delay1_7us()
{
	hw_delay( 17 ) ; // units of 0.1uS
}

#define CS1_LOW		(GPIO_TOPLCD->BSRRH = PIN_TOPLCD_CS1)
#define CS1_HIGH	(GPIO_TOPLCD->BSRRL = PIN_TOPLCD_CS1)
#define CS2_LOW		(GPIO_TOPLCD->BSRRH = PIN_TOPLCD_CS2)
#define CS2_HIGH	(GPIO_TOPLCD->BSRRL = PIN_TOPLCD_CS2)
#define DATA_LOW	(GPIO_TOPLCD->BSRRH = PIN_TOPLCD_DATA)
#define DATA_HIGH	(GPIO_TOPLCD->BSRRL = PIN_TOPLCD_DATA)
#define WR_LOW		(GPIO_TOPLCD->BSRRH = PIN_TOPLCD_WR)
#define WR_HIGH		(GPIO_TOPLCD->BSRRL = PIN_TOPLCD_WR)

#define VA_BL_ON 	(GPIO_TOPLCD->BSRRL = PIN_TOPLCD_LED)
#define VA_BL_OFF	(GPIO_TOPLCD->BSRRH = PIN_TOPLCD_LED)

// States:
#define TOP_LCD_IDLE		0
#define TOP_LCD_CKLOW		1
#define TOP_LCD_CKHI		2
#define TOP_LCD_END			3
#define TOP_LCD_1CKLOW	4
#define TOP_LCD_1CKHI		5

struct t_top_lcd_control
{
	uint8_t *data ;
	uint8_t *data2 ;
	uint16_t chip ;
	uint16_t count ;
	uint16_t count2 ;
	uint16_t state ;
	uint16_t bitCount ;
} TopLcdControl ;

void sendToplcdCommand( uint8_t command, uint8_t chip )
{
  uint32_t i ;
    
	WR_HIGH;
	if ( chip	) CS2_LOW ; else CS1_LOW ;
	delay1_7us() ;
	WR_LOW ;      //PRESENT 100 COMMAND CODE 
	DATA_HIGH ;
	delay1_7us() ;
	WR_HIGH ;
	delay1_7us() ;
	WR_LOW ;
	DATA_LOW ;
	delay1_7us() ;
	WR_HIGH ;
	delay1_7us() ;
	WR_LOW ;
	delay1_7us() ;
	WR_HIGH ;
	delay1_7us() ;
	for (i =0;i<=7;i++) 
	{
		WR_LOW ;
		if ((command & 0x80) !=0)  
		{
	    DATA_HIGH ;
		}
		else 
		{
			DATA_LOW ;
		}
		delay1_7us() ;
		WR_HIGH ;
		delay1_7us() ;
		command = command << 1 ;
	}
	WR_LOW ;
	delay1_7us() ;
	WR_HIGH ;
	delay1_7us() ;
	if ( chip	) CS2_HIGH ; else CS1_HIGH ;
	delay1_7us() ;
} 

// Send up to 1 byte of data, CS already valid
//void ht1621WrData( uint8_t data, uint8_t count )
//{
//	while ( count )
//	{
//		WR_LOW ;
//		if ( data & 0x80 )
//		{
//			DATA_HIGH ;
//		}
//		else  
//		{
//			DATA_LOW ;
//		}
//		delay1_7us() ;
//		WR_HIGH ;
//		delay1_7us() ;
//	  data <<= 1 ;
//		count -= 1 ;
//	}
//}

//void ht1621WrAllData( uint8_t *pData, uint8_t chip )
//{ 
//	uint32_t i ; 
//	uint32_t j ; 
//	if ( chip	) CS2_LOW ; else CS1_LOW ;
//	delay1_7us() ;
//  ht1621WrData( 0xa0, 3 ) ;
//  ht1621WrData( 0, 6 ) ; // HT1621 6 bit,left 2 bit;
//	j = chip ? 7 : 11 ;
//	for ( i = 0 ; i < j ; i += 1 ) 
//	{ 
//		ht1621WrData( *pData++, 8 ) ;
//	} 
//	if ( chip	) CS2_HIGH ; else CS1_HIGH ;
//	delay1_7us() ;
//} 

void updateTopLCD( uint32_t time, uint32_t batteryState ) ;

const uint8_t TimeLCDsegs[] = {	0xAF, 0x06, 0x6D, 0x4F, 0xC6, 0xCB, 0xEB, 0x0E, 0xEF, 0xCF } ;
const uint8_t RssiLCDsegs[] = {	0xFA, 0x60, 0xBC, 0xF4, 0x66, 0xD6, 0xDE, 0x70, 0xFE, 0xF6 } ;
const uint8_t OpTimeLCDsegs[] = {	0x5F, 0x06, 0x6B, 0x2F, 0x36, 0x3D, 0x7D, 0x07, 0x7F, 0x3F } ;
static uint8_t Ht1621Data1[12] ;
static uint8_t Ht1621Data2[8] ;
static uint8_t RssiValue ;

void initTimerTopLcd()
{
	// Timer12
	RCC->APB1ENR |= RCC_APB1ENR_TIM12EN ;		// Enable clock
	TIM12->ARR = 17 ;	// 1.7uS
	TIM12->PSC = (PeripheralSpeeds.Peri1_frequency*PeripheralSpeeds.Timer_mult1) / 10000000 - 1 ;		// 0.1uS from 30MHz
	TIM12->CCER = 0 ;	
	TIM12->CCMR1 = 0 ;
	TIM12->EGR = 0 ;
	TIM12->CR1 = 5 ;
  NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 2 ) ;
	NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn) ;
}

void initTopLcd()
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_TOPLCD, ENABLE);
	GPIO_TOPLCD->BSRRL = PIN_TOPLCD_CS1 | PIN_TOPLCD_CS2 ;
	configure_pins( PIN_TOPLCD_LED | PIN_TOPLCD_CS1 | PIN_TOPLCD_CS2 | PIN_TOPLCD_WR | PIN_TOPLCD_DATA,
									 PIN_OUTPUT | PIN_PORTG| PIN_PUSHPULL | PIN_OS25 | PIN_NO_PULLUP ) ;

  sendToplcdCommand(0x03, 0) ;
  sendToplcdCommand(0x01, 0) ;
  sendToplcdCommand(0x29, 0) ;
  sendToplcdCommand(0x03, 1) ;
  sendToplcdCommand(0x01, 1) ;
  sendToplcdCommand(0x29, 1) ;

	initTimerTopLcd() ;

	VA_BL_ON ;
	updateTopLCD( 0, 0 ) ;
	TIM12->DIER |= 1 ;		// First write to blank segments
}


extern "C" void TIM8_BRK_TIM12_IRQHandler()
{
	struct t_top_lcd_control *pc ;
	TIM12->SR &= ~TIM_SR_UIF ;
	pc = &TopLcdControl ;

	if ( pc->state == TOP_LCD_CKLOW )
	{
		WR_LOW ;      //PRESENT 100 COMMAND CODE 
		if ((*pc->data & 0x80) !=0)  
		{
			DATA_HIGH ;
		}
		else 
		{
			DATA_LOW ;
		}
		*pc->data <<= 1 ;
		pc->state = TOP_LCD_CKHI ;
	}
	else if ( pc->state == TOP_LCD_CKHI )
	{
		WR_HIGH ;
		if ( --pc->count == 0 )
		{
			pc->state = TOP_LCD_END ;
		}
		else
		{
			pc->state = TOP_LCD_CKLOW ;
			if ( ( ++pc->bitCount & 7) == 0 )
			{
				pc->data += 1 ;	// next byte
			}
		}
	}
	else if ( pc->state == TOP_LCD_END )
	{
		if ( pc->chip	)
		{
			CS2_HIGH ;
			pc->state = TOP_LCD_IDLE ;
			TIM12->DIER &= ~1 ;
		}
		else
		{
			CS1_HIGH ;
			pc->chip = 1 ;
			pc->data = pc->data2 ;
			pc->count = pc->count2 ;
			pc->bitCount = 0 ;
			pc->state = TOP_LCD_IDLE ;
		}
	}
	else if ( pc->state == TOP_LCD_IDLE )
	{
		WR_HIGH;
		if ( pc->chip	) CS2_LOW ; else CS1_LOW ;
		pc->bitCount = 0 ;
		pc->state = TOP_LCD_1CKLOW ;
	}
	else if ( pc->state == TOP_LCD_1CKLOW )
	{
		WR_LOW ;
		DATA_HIGH ;		// First 1 bit
		pc->state = TOP_LCD_1CKHI ;
	}
	else if ( pc->state == TOP_LCD_1CKHI )
	{
		WR_HIGH ;
		pc->state = TOP_LCD_CKLOW ;
	}
}


void setTopTime( uint32_t time )
{
	div_t qr ;
	uint32_t r ;
	qr = div( time, 60 ) ;
	r = qr.rem ;

	qr = div( qr.quot, 10 ) ;

	Ht1621Data1[1] = TimeLCDsegs[qr.quot] ;
	Ht1621Data1[2] = TimeLCDsegs[qr.rem] ;
	
	qr = div( r, 10 ) ;
	Ht1621Data1[3] = TimeLCDsegs[qr.quot] | 0x10 ; // ":"
	Ht1621Data1[4] = TimeLCDsegs[qr.rem] | 0x10 ; // "Operation Time"
}

void setTopRssi( uint32_t rssi )
{
	div_t qr ;
	uint32_t r = 0 ;
	RssiValue = rssi ;
	qr = div( rssi, 10 ) ;
	if ( qr.quot > 9 )
	{
		r = 1 ;
		qr.quot -= 10 ;
	}
	Ht1621Data2[3] = r ? RssiLCDsegs[1] : 0 ;
	Ht1621Data2[2] = ( r || qr.quot ) ? RssiLCDsegs[qr.quot] : 0 ;
	Ht1621Data2[1] = RssiLCDsegs[qr.rem] ;
	Ht1621Data1[5] |= 0x10 ;	// "RSSI"

}

void setTopVoltage( uint32_t volts )
{
	div_t qr ;
	uint32_t r = 0 ;
	uint32_t segs ;
	qr = div( volts, 10 ) ;
	if ( qr.quot > 9 )
	{
		r = 1 ;
		qr.quot -= 10 ;
	}
	segs = r ? RssiLCDsegs[1] : 0 ;
	Ht1621Data2[4] = ((segs & 7) << 1 ) | 1 ;	// "DB"
	Ht1621Data2[5] = segs & 0xF0 ;
	segs = ( r || qr.quot ) ? RssiLCDsegs[qr.quot] : 0 ;
	Ht1621Data2[5] |= (segs & 0x0E) | 1 ;	// Horizontal line
	Ht1621Data2[6] = segs & 0xF0 ;
	segs = RssiLCDsegs[qr.rem] ;
	Ht1621Data2[6] |= (segs & 0x0E) | 1 ;	// Decimal point
	Ht1621Data2[7] = segs & 0xF0 ;
}

void setTopOpTime( uint32_t hours, uint32_t mins, uint32_t secs )
{
	div_t qr ;
	uint32_t segs ;
	uint32_t segs2 ;
	
	qr = div( secs, 10 ) ;
	segs = OpTimeLCDsegs[qr.rem] ;
	Ht1621Data1[5] = (segs & 0x0F) | 0x70 ;
	segs &= 0x70 ;
	segs2 = OpTimeLCDsegs[qr.quot] ;
	Ht1621Data1[6] = segs | (segs2 & 0x0F) ;

	qr = div( mins, 10 ) ;
	segs = OpTimeLCDsegs[qr.rem] ;
	segs2 &= 0x70 ;
	Ht1621Data1[7] = segs2 | (segs & 0x0F ) | 0x80 ; // ":"
	segs &= 0x70 ;
	segs2 = OpTimeLCDsegs[qr.quot] ;
	Ht1621Data1[8] = segs | (segs2 & 0x0F) ;
	
	qr = div( hours, 10 ) ;
	segs = OpTimeLCDsegs[qr.rem] ;
	segs2 &= 0x70 ;
	Ht1621Data1[9] = segs2 | (segs & 0x0F ) | 0x80 ; // ":"
	segs &= 0x70 ;
	segs2 = OpTimeLCDsegs[qr.quot] ;
	Ht1621Data1[10] = segs | (segs2 & 0x0F) ;
	Ht1621Data1[11] = segs2 & 0xF0 ;
}

void updateTopLCD( uint32_t time, uint32_t batteryState )
{
	uint32_t x ;
	setTopTime( time ) ;
	if ( batteryState )
	{
		Ht1621Data1[8] |= 0x80 ;
	}
	if ( batteryState > 1 )
	{
		Ht1621Data1[10] |= 0x80 ;
	}
	if ( batteryState > 2 )
	{
		Ht1621Data1[6] |= 0x80 ;
	}
	if ( batteryState > 3 )
	{
		Ht1621Data1[11] |= 0x80 ;
	}
	if ( batteryState > 4 )
	{
		Ht1621Data1[5] |= 0x80 ;
	}

	x = RssiValue ;
	if ( x > 42 )
	{
		Ht1621Data2[3] |= 1 ;
	}
	if ( x > 45 )
	{
		Ht1621Data2[2] |= 1 ;
	}
	if ( x > 50 )
	{
		Ht1621Data2[1] |= 1 ;
	}
	if ( x > 60 )
	{
		Ht1621Data2[4] |= 0x10 ;
	}
	if ( x > 70 )
	{
		Ht1621Data2[4] |= 0x20 ;
	}
	if ( x > 80 )
	{
		Ht1621Data2[4] |= 0x40 ;
	}
	if ( x > 90 )
	{
		Ht1621Data2[4] |= 0x80 ;
	}

	Ht1621Data1[0] = 0x40 ;	// last 2 bits command and 6 bits 0 address
	Ht1621Data2[0] = 0x40 ;	// last 2 bits command and 6 bits 0 address
	
	TopLcdControl.data = Ht1621Data1 ;
	TopLcdControl.data2 = Ht1621Data2 ;
	TopLcdControl.chip = 0 ;
	TopLcdControl.count = 12*8-4 ;
	TopLcdControl.count2 = 8*8-4 ;
	TopLcdControl.state = TOP_LCD_IDLE ;
	TIM12->DIER |= 1 ;
}


#endif

