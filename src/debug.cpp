/****************************************************************************
*  Copyright (c) 2011 by Michael Blandford. All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*  3. Neither the name of the author nor the names of its contributors may
*     be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
*  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
*  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
*  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
*  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
*  SUCH DAMAGE.
*
****************************************************************************
*  History:
*
****************************************************************************/


#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#include "AT91SAM3S2.h"
#include "core_cm3.h"


#include "ersky9x.h"
#include "sound.h"
#include "lcd.h"
#include "myeeprom.h"
#include "drivers.h"
#include "debug.h"
#include "file.h"

#include "s9xsplash.lbm"

//extern uint32_t Per10ms_action ;
//extern uint32_t Permenu_action ;
static void disp_256( uint32_t address, uint32_t lines ) ;
//extern uint8_t eeprom[] ;



#ifdef	DEBUG
uint32_t Mem_address ;
uint32_t Memaddmode ;

void handle_serial()
{
	uint16_t rxchar ;

	if ( ( rxchar = rxuart() ) == 0xFFFF )
	{
		return ;
	}
	// Got a char, what to do with it?

	if ( Memaddmode )
	{
		rxchar = toupper( rxchar ) ;
		if ( ( ( rxchar >= '0' ) && ( rxchar <= '9' ) ) || ( ( rxchar >= 'A' ) && ( rxchar <= 'F' ) ) )
		{
			txmit( rxchar ) ;
			rxchar -= '0' ;
			if ( rxchar > 9 )
			{
				rxchar -= 7 ;				
			}
			Mem_address <<= 4 ;
			Mem_address |= rxchar ;			
		}
		else if ( rxchar == 13 )
		{
			crlf() ;
			disp_256( Mem_address, 4 ) ;
			Memaddmode = 0 ;				
		}
		else if ( rxchar == 8 )
		{
			txmit( rxchar ) ;
			txmit( rxchar ) ;
			txmit( rxchar ) ;
			Mem_address >>= 4 ;			
		}
		else if ( rxchar == 27 )
		{
			crlf() ;
			Memaddmode = 0 ;				
		}		

	}
	 
	if ( rxchar == '?' )
	{
		Memaddmode = 1 ;
		Mem_address = 0 ;
		txmit( '>' ) ;
	}

	if ( rxchar == 'V' )
	{
		uputs( (char *)VERSION ) ;
		crlf() ;
	}

//	if ( rxchar == 'A' )
//	{
//		if ( Permenu_action )
//		{
//			Permenu_action = 0 ;			
//		}
//		else
//		{
//			Permenu_action = 1 ;
//		}
		
//	}

	if ( rxchar == 'B' )
	{
		register Adc *padc ;

		padc = ADC ;
		p8hex( padc->ADC_CDR4 ) ;
		crlf() ;
		read_9_adc() ;
		DACC->DACC_CDR = padc->ADC_CDR4 ;		// Battery 
	}
	
	if ( rxchar == 'R' )
	{
		register const volatile uint32_t *pword ;
		register uint32_t i ;

		pword = &ADC->ADC_CDR0 ;
		txmit( 'R' ) ;
		crlf() ;
		for ( i = 0 ; i < 16 ; i += 1 )
		{
			p8hex( *pword++ ) ;
			crlf() ;
		}
	}
	
	if ( rxchar == 'K' )
	{
		txmit( 'K' ) ;
		p8hex( read_keys() ) ;
		crlf() ;
	}
	
	if ( rxchar == 'T' )
	{
		txmit( 'T' ) ;
		p4hex( read_trims() ) ;
		crlf() ;
	}

	if ( rxchar == 'S' )
	{
		txmit( 'E' ) ;
		p2hex( keyState( SW_ElevDR ) ) ;
		crlf() ;
		txmit( 'A' ) ;
		p2hex( keyState( SW_AileDR ) ) ;
		crlf() ;
		txmit( 'R' ) ;
		p2hex( keyState( SW_RuddDR ) ) ;
		crlf() ;
		txmit( 'G' ) ;
		p2hex( keyState( SW_Gear ) ) ;
		crlf() ;
		txmit( 'C' ) ;
		p2hex( keyState( SW_ThrCt ) ) ;
		crlf() ;
		txmit( 'T' ) ;
		p2hex( keyState( SW_Trainer ) ) ;
		crlf() ;
		txmit( '0' ) ;
		txmit( ' ' ) ;
		p2hex( keyState( SW_ID0 ) ) ;
		crlf() ;
		txmit( '1' ) ;
		txmit( ' ' ) ;
		p2hex( keyState( SW_ID1 ) ) ;
		crlf() ;
		txmit( '2' ) ;
		txmit( ' ' ) ;
		p2hex( keyState( SW_ID2 ) ) ;
		crlf() ;
	}

	if ( rxchar == 'Z' )
	{
		txmit( 'A' ) ;
		txmit( ' ' ) ;
		p8hex( PIOA->PIO_PDSR ) ;
		crlf() ;
		txmit( 'B' ) ;
		txmit( ' ' ) ;
		p8hex( PIOB->PIO_PDSR ) ;
		crlf() ;
		txmit( 'C' ) ;
		txmit( ' ' ) ;
		p8hex( PIOC->PIO_PDSR ) ;
		crlf() ;
	}
		 
	if ( rxchar == 'L' )
	{
		lcd_clear() ;
		lcd_init() ;		
		lcd_putsn_P( 7*FW, 0, "ERSKY9X", 7 ) ;
		refreshDisplay() ;
	}


//	if ( rxchar == 'Y' )
//	{
//		lcd_clear();
//  	lcd_img(0, 0, s9xsplash,0,0);
//  	lcd_putsnAtt( 0*FW, 7*FH, g_eeGeneral.ownerName ,sizeof(g_eeGeneral.ownerName),0);
//  	lcd_putsnAtt( 4*FW, 3*FH, "SKY" , 3, DBLSIZE ) ;
//		refreshDisplay();
//	}

//	if ( rxchar == 'W' )
//	{
//		if ( Per10ms_action )
//		{
//			Per10ms_action = 0 ;			
//		}
//		else
//		{
//			Per10ms_action = 1 ;
//		}
//	}

//	if ( rxchar == 'Q' )
//	{
//		register uint8_t *p ;
//		register uint32_t x ;

//		txmit( 'Q' ) ;
//		p = Spi_rx_buf ;
//		*(p+4) = 0 ;
//		*(p+5) = 0 ;
//		*(p+19) = 0 ;

//		p = Spi_tx_buf ;
//		*p = 3 ;		// Read command
//		*(p+1) = 0 ;
//		*(p+2) = 0 ;
//		*(p+3) = 0 ;		// 3 bytes address
//		x = spi_operation( p, Spi_rx_buf, 20 ) ;

//		p8hex( x ) ;
//		txmit( ' ' ) ;
//		p = Spi_rx_buf ;
//		p2hex( *(p+4) ) ;
//		p2hex( *(p+5) ) ;
//		p2hex( *(p+18) ) ;
//		p2hex( *(p+19) ) ;
//		crlf() ;
//	}

//	if ( rxchar == 'E' )
//	{
//		register uint8_t *p ;
//		register uint32_t x ;

//		txmit( 'E' ) ;
//		p = Spi_rx_buf ;
//		*(p+1) = 0 ;
//		*(p+2) = 0 ;

//		p = Spi_tx_buf ;
//		*p = 5 ;		// Read status command
//		*(p+1) = 0 ;
//		x = spi_operation( p, Spi_rx_buf, 2 ) ;

//		p8hex( x ) ;
//		txmit( ' ' ) ;
//		p = Spi_rx_buf ;
//		p2hex( *p ) ;
//		p2hex( *(p+1) ) ;
//		p2hex( *(p+2) ) ;
//		p2hex( *(p+3) ) ;
//		crlf() ;
//	}
	
//	if ( rxchar == 'F' )
//	{
//		register uint8_t *p ;
//		register uint32_t x ;

//		txmit( 'F' ) ;

//		eeprom_write_enable() ;
		
//		p = Spi_tx_buf ;
//		*p = 1 ;		// Write status register command
//		*(p+1) = 0 ;

//		x = spi_operation( p, Spi_rx_buf, 2 ) ;
//		p8hex( x ) ;
//		crlf() ;
//	}
	
//	if ( rxchar == 'X' )
//	{
//		register uint8_t *p ;
//		register uint32_t x ;
		
//		txmit( 'X' ) ;
//		eeprom_write_enable() ;
		
//		p = Spi_tx_buf ;
//		*p = 0x20 ;		// Block Erase command
//		*(p+1) = 0 ;
//		*(p+2) = 0 << 4 ;		// Erase block 0
//		*(p+3) = 0 ;		// 3 bytes address
		
//		x = spi_operation( p, Spi_rx_buf, 4 ) ;
//		p8hex( x ) ;
//		crlf() ;
//	}
	
//	if ( rxchar == 'C' )
//	{
//		register uint8_t *p ;
//		register uint32_t x ;

//		txmit( 'C' ) ;

//		eeprom_write_enable() ;
		
//		p = Spi_tx_buf ;
//		*p = 0x39 ;		// Unprotect sector command
//		*(p+1) = 0 ;
//		*(p+2) = 0 ;
//		*(p+3) = 0 ;		// 3 bytes address

//		x = spi_operation( p, Spi_rx_buf, 4 ) ;

//		p8hex( x ) ;
//		crlf() ;
//	}
	 
//	if ( rxchar == 'U' )
//	{
//		register uint8_t *p ;
//		register uint32_t x ;

//		txmit( 'U' ) ;

//		eeprom_write_enable() ;
//		p = Spi_tx_buf ;
		
//		*p = 2 ;		// Write command
//		*(p+1) = 0 ;
//		*(p+2) = 0 ;
//		*(p+3) = 0 ;		// 3 bytes address
//		*(p+4) = 1 ;
//		*(p+5) = 2 ;
//		*(p+18) = 0x0F ;
//		*(p+19) = 0x10 ;

//		x = spi_operation( p, Spi_rx_buf, 20 ) ;
//		p8hex( x ) ;
//		crlf() ;
//	}

	if ( rxchar == '-' )
	{
		register uint32_t x ;
		
		x = PWM->PWM_CH_NUM[0].PWM_CDTY ;				// Duty (current)
		if ( x < 100 )
		{
			x += 1 ;
			PWM->PWM_CH_NUM[0].PWM_CDTYUPD = x ;	// Duty update
			g_eeGeneral.bright = x ;
		}
	}

	if ( rxchar == '+' )
	{
		register uint32_t x ;
		
		x = PWM->PWM_CH_NUM[0].PWM_CDTY ;				// Duty (current)
		if ( x > 0 )
		{
			x -= 1 ;
			PWM->PWM_CH_NUM[0].PWM_CDTYUPD = x ;	// Duty update
			g_eeGeneral.bright = x ;
		}
	}

	if ( rxchar == '/' )
	{
		if ( g_eeGeneral.volume > 0 )
		{
			set_volume( --g_eeGeneral.volume ) ;
		}
	}

	if ( rxchar == '*' )
	{
		if ( g_eeGeneral.volume < NUM_VOL_LEVELS )
		{
			set_volume( ++ g_eeGeneral.volume ) ;
		}
	}

	if ( rxchar == '(' )
	{
		set_frequency( 500 ) ;
	}
	
	if ( rxchar == '=' )
	{
		set_frequency( 1000 ) ;
	}
	
	if ( rxchar == ')' )
	{
		set_frequency( 3000 ) ;
	}
	
	if ( rxchar == ' ' )
	{
		tone_start( 50 ) ;
	}

	if ( rxchar == '!' )
	{
		tone_start( 0 ) ;
	}

	if ( rxchar == '.' )
	{
		tone_stop() ;
	}

	// Display Ram version of EEPROM
//	if ( ( rxchar >= '0' ) && ( rxchar <= '7' ) )
//	{
//		disp_256( ( uint32_t)eeprom + (rxchar - '0') * 256, 16 ) ;
//	}

//	if ( rxchar == '@' )
//	{
//  	strncpy_P(g_eeGeneral.ownerName,PSTR("MIKE      "), 10) ;
//		STORE_GENERALVARS ;
//	}


}


//void disp_mem( register uint32_t address )
//{
//	p8hex( address ) ;
//	txmit('=') ;
//	p8hex( *( (uint32_t *)address ) ) ;
//	crlf() ;
//}

static void disp_256( register uint32_t address, register uint32_t lines )
{
	register uint32_t i ;
	register uint32_t j ;
	for ( i = 0 ; i < lines ; i += 1 )
	{
		p8hex( address ) ;
		for ( j = 0 ; j < 16 ; j += 1 )
		{
			txmit(' ') ;
			p2hex( *( (uint8_t *)address++ ) ) ;
		}
		crlf() ;
	}
}

#endif



