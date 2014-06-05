/**
  ******************************************************************************
  * @file    Project/spi/spi.c 
  * @author  X9D Application Team
  * @Hardware version V0.2
  * @date    11-July-2012
  * @brief   This file provides spi Init(Analog spi),read and write.
             driver for LCD.
  * *
  ******************************************************************************
*/

#include "../ersky9x.h"
#include "x9d\stm32f2xx.h"
#include "x9d\stm32f2xx_gpio.h"
#include "x9d\stm32f2xx_rcc.h"
#include "x9d\hal.h"
#include "x9d\aspi.h"
#include "logicio.h"

#define __no_operation     __NOP

/**
**********send command to lcd**************
*/
//hardware spi
/*u16 spiCmd(u8 addr)
{
	u8 readValue;
	
	SPI_RS_LOW();
    SPI_NCS_LOW();
	
	SPI_WRITE_BYTE(addr);
	SPI_WAIT_DONE();

	readValue=SPI_READ_BYTE();
	SPI_NCS_HIGH();
    return	readValue;
}*/

//Anolog spi
void AspiCmd(uint8_t Command_Byte)
{
    int i=8; 
    LCD_A0_LOW();

    LCD_CLK_HIGH();
    LCD_NCS_LOW();  
 
    while (i--) 
    { 
	    LCD_CLK_LOW(); 
	    if(Command_Byte&0x80)
	{
        LCD_MOSI_HIGH();
    }
	else LCD_MOSI_LOW();

	Command_Byte=Command_Byte<<1; 
	LCD_CLK_HIGH();  
    } 
    LCD_NCS_HIGH();  
    LCD_A0_HIGH();
}

/**

********send data to lcd**************

*/

//hardware spi
/*
u16 spiData(u8 addr)
{
	u16 readValue;
      	OLED_RS_HIGH();
	SPI_NCS_LOW();
	
	SPI_WRITE_BYTE(addr);
	SPI_WAIT_DONE();

	readValue=SPI_READ_BYTE();
	SPI_NCS_HIGH();
      	return	readValue;
}
*/

//Anolog spi
void AspiData(uint8_t Para_data)
{
    int i=8;
    LCD_CLK_HIGH();
    LCD_A0_HIGH();
    LCD_NCS_LOW();
    while (i--) 
    {
        if(Para_data&0x80)
        {
            LCD_MOSI_HIGH();
        }else LCD_MOSI_LOW();
        Para_data<<=1;
        LCD_CLK_LOW();
        __no_operation();
        LCD_CLK_HIGH();
        __no_operation();
    }
    LCD_NCS_HIGH();
    LCD_A0_HIGH();  
}

/***
**********read one byte in a register*******
*/

/*
u16 spiRegAccess(u8 addrByte, u8 writeValue)
{
	u8 readValue;

	SPI_NCS_LOW();
	
	SPI_WRITE_BYTE(addrByte);
	SPI_WAIT_DONE();

	SPI_WRITE_BYTE(writeValue);
	SPI_WAIT_DONE();

	readValue=SPI_READ_BYTE();
	SPI_NCS_HIGH();
	return	readValue;
}

*/

#ifdef REVPLUS
// New hardware SPI driver for LCD
void initLcdSpi()
{
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_LCD, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_LCD_RST, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_LCD_NCS, ENABLE);
	
  RCC->APB1ENR |= RCC_APB1ENR_SPI3EN ;    // Enable clock
	// APB1 clock / 2 = 133nS per clock
	SPI3->CR1 = SPI_CR1_BIDIOE | SPI_CR1_SPE | SPI_CR1_MSTR | SPI_CR1_CPOL ;
	SPI3->CR2 = 0 ;

	configure_pins( PIN_LCD_NCS, PIN_OUTPUT | PIN_PORTC | PIN_PUSHPULL | PIN_OS25 | PIN_NO_PULLUP ) ;
	configure_pins( PIN_LCD_RST, PIN_OUTPUT | PIN_PORTD | PIN_PUSHPULL | PIN_OS25 | PIN_NO_PULLUP ) ;
	configure_pins( PIN_LCD_A0,  PIN_OUTPUT | PIN_PORTC | PIN_PUSHPULL | PIN_OS25 | PIN_NO_PULLUP ) ;
	configure_pins( PIN_LCD_MOSI|PIN_LCD_CLK, PIN_OUTPUT | PIN_PORTC | PIN_PUSHPULL | PIN_OS50 | PIN_NO_PULLUP | PIN_PER_6 | PIN_PERIPHERAL ) ;
}

void xAspiCmd(uint8_t Command_Byte)
{
	while ( SPI3->SR & SPI_SR_TXE )
	{
		// wait
	}
  LCD_A0_LOW() ;
  LCD_NCS_LOW() ;  
	SPI3->DR = Command_Byte ;
	while ( SPI3->SR & SPI_SR_TXE )
	{
		// wait
	}
  LCD_NCS_HIGH() ;
}

void xAspiData(uint8_t Para_data)
{
	while ( SPI3->SR & SPI_SR_TXE )
	{
		// wait
	}
  LCD_A0_HIGH() ;
  LCD_NCS_LOW() ;  
	SPI3->DR = Para_data ;
	while ( SPI3->SR & SPI_SR_TXE )
	{
		// wait
	}
  LCD_NCS_HIGH() ;
}


#endif


