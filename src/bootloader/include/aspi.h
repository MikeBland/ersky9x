/**
  ******************************************************************************
  * @file    Project/spi/spi.h 
  * @author  X9D Application Team
  * @Hardware version V0.2
  * @date    11-July-2012
  * @brief   spi.c' Header file.
  * *
  ******************************************************************************
*/
#ifndef _ASPI_H_
#define _ASPI_H_

#include "stm32f2xx.h"

#ifdef REVPLUS
#define	LCD_NCS_HIGH()		    (GPIO_LCD_NCS->BSRRL = PIN_LCD_NCS)
#define	LCD_NCS_LOW()		    	(GPIO_LCD_NCS->BSRRH = PIN_LCD_NCS)
#else
#define	LCD_NCS_HIGH()		    (GPIO_LCD->BSRRL = PIN_LCD_NCS)
#define	LCD_NCS_LOW()		    	(GPIO_LCD->BSRRH = PIN_LCD_NCS)
#endif

#define LCD_A0_HIGH()         (GPIO_LCD->BSRRL = PIN_LCD_A0)
#define LCD_A0_LOW()          (GPIO_LCD->BSRRH = PIN_LCD_A0)

#ifdef REVPLUS
#define LCD_RST_HIGH()		    (GPIO_LCD_RST->BSRRL = PIN_LCD_RST)
#define LCD_RST_LOW()		    	(GPIO_LCD_RST->BSRRH = PIN_LCD_RST)
#else
#define LCD_RST_HIGH()		    (GPIO_LCD->BSRRL = PIN_LCD_RST)
#define LCD_RST_LOW()		    	(GPIO_LCD->BSRRH = PIN_LCD_RST)
#endif

#define LCD_CLK_HIGH()		    (GPIO_LCD->BSRRL = PIN_LCD_CLK)
#define LCD_CLK_LOW()		    	(GPIO_LCD->BSRRH = PIN_LCD_CLK)

#define LCD_MOSI_HIGH()		    (GPIO_LCD->BSRRL = PIN_LCD_MOSI)
#define LCD_MOSI_LOW()		    (GPIO_LCD->BSRRH = PIN_LCD_MOSI)


void AspiCmd(u8 Command_Byte);
void AspiData(u8 Para_data);
//u16 spiRegAccess(u8 addrByte, u8 writeValue);

#endif

