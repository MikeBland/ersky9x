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

#include "AT91SAM3S4.h"
#include "core_cm3.h"


#include "ersky9x.h"
#include "sound.h"
#include "lcd.h"
#include "myeeprom.h"
#include "drivers.h"
#include "debug.h"
#include "file.h"

//#include "s9xsplash.lbm"

//extern uint32_t Per10ms_action ;
//extern uint32_t Permenu_action ;
static void disp_256( uint32_t address, uint32_t lines ) ;
//extern uint8_t eeprom[] ;



#ifdef	DEBUG
uint32_t Mem_address ;
uint32_t Next_mem_address ;

uint32_t Memaddmode ;
uint32_t SoundCheck ;

uint32_t Sdcard_data[128] ;

void handle_serial()
{
	uint16_t rxchar ;

	if ( SoundCheck )
	{
		if ( queueTone( 610, 200, 30 ) )
		{
			SoundCheck = 0 ;
		}			
	}

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
			if ( Mem_address == 0 )
			{
				Mem_address = Next_mem_address ;
			}
			disp_256( Mem_address, 4 ) ;
			Next_mem_address = Mem_address + 64 ;
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


	if ( rxchar == 'Y' )
	{
		SoundCheck = 1 ;
	}

	if ( rxchar == 'H' )
	{
		txmit( 'H' ) ;
		hapticOn( 40 ) ;
	}

	if ( rxchar == 'I' )
	{
		txmit( 'I' ) ;
		hapticOff() ;
	}

extern void read_volume( void ) ;
extern uint8_t Volume_read ;
extern void read_coprocessor( void ) ;
extern uint8_t Coproc_read ;
extern uint8_t Coproc_valid ;
	
	if ( rxchar == 'W' )
	{
		read_volume() ;
		txmit( 'W' ) ;
		txmit( '-' ) ;
		p2hex( Volume_read ) ;
		crlf() ;
	}

	if ( rxchar == 'Q' )
	{
		read_coprocessor() ;
		txmit( 'Q' ) ;
		txmit( '-' ) ;
		p2hex( Coproc_read ) ;
		txmit( ' ' ) ;
		p2hex( Coproc_valid ) ;
		crlf() ;
	}


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

#define SD_ST_EMPTY		0
#define SD_ST_IDLE		1
#define SD_ST_READY		2
#define SD_ST_IDENT		3
#define SD_ST_STBY		4
#define SD_ST_TRAN		5
#define SD_ST_DATA		6

	if ( rxchar == 'E' )
	{
		uint32_t i ;
//		static uint32_t card_state = 0 ;

//		txmit( 'E' ) ;
//		txmit( '-' ) ;
//		switch ( card_state )
//		{
//			case SD_ST_EMPTY :
//				i = ( PIOB->PIO_PDSR & PIO_PB7 ) ? 0 : 1 ;
//				txmit( 'e' ) ;
//				txmit( ' ' ) ;
//  			p2hex( i ) ;
//				if ( i )
//				{
//					card_state = SD_ST_IDLE ;
//				}
//				crlf() ;
//			break ;

//			case SD_ST_IDLE :
//				i = sd_acmd41() ;
//				txmit( 'L' ) ;
//				txmit( ' ' ) ;
//				p8hex( i ) ;
//				crlf() ;
//				if ( i & 0x80000000 )
//				{
//					card_state = SD_ST_READY ;
//				}
//			break ;

//			case SD_ST_READY :
//				i = sd_cmd2() ;
//				card_state = SD_ST_IDENT ;
//				txmit( 'R' ) ;
//				txmit( ' ' ) ;
//				p8hex( Sd_128_resp[0] ) ;
//				txmit( ' ' ) ;
//				p8hex( Sd_128_resp[1] ) ;
//				txmit( ' ' ) ;
//				p8hex( Sd_128_resp[2] ) ;
//				txmit( ' ' ) ;
//				p8hex( Sd_128_resp[3] ) ;
//				crlf() ;
//			break ;

//			case SD_ST_IDENT :
//				i = sd_cmd3() ;
//				card_state = SD_ST_STBY ;
//				Sd_rca = i ;
//				txmit( 'T' ) ;
//				txmit( ' ' ) ;
//				p8hex( i ) ;
//				crlf() ;
//			break ;

//			case SD_ST_STBY :
//				i = sd_cmd9() ;
//				txmit( 'S' ) ;
//				txmit( ' ' ) ;
//				p8hex( Sd_128_resp[0] ) ;
//				txmit( ' ' ) ;
//				p8hex( Sd_128_resp[1] ) ;
//				txmit( ' ' ) ;
//				p8hex( Sd_128_resp[2] ) ;
//				txmit( ' ' ) ;
//				p8hex( Sd_128_resp[3] ) ;
//				txmit( '-' ) ;
//				p8hex(HSMCI->HSMCI_SR) ;
//				txmit( ' ' ) ;
//				p8hex(HSMCI->HSMCI_SR) ;
//				i = sd_cmd7() ;
//				crlf() ;
//				card_state = SD_ST_TRAN ;
//			break ;

static uint32_t SdAddress = 0 ;

//			case SD_ST_TRAN :
//				txmit( 't' ) ;
//				txmit( ' ' ) ;
//				Sd_128_resp[0] = 0 ;
//				Sd_128_resp[1] = 0 ;
//				i = sd_acmd51( Sd_128_resp ) ;
//				p8hex( Sd_128_resp[0] ) ;
//				txmit( ' ' ) ;
//				p8hex( Sd_128_resp[1] ) ;
//				card_state = SD_ST_DATA ;
//				i = sd_acmd6() ;
//				txmit( '+' ) ;
//				p8hex( i ) ;
//				crlf() ;
//				SdAddress = 0 ;
//			break ;
			
//			case SD_ST_DATA :
//				txmit( 'D' ) ;
//				txmit( ' ' ) ;
				i = sd_read_block( SdAddress, Sdcard_data ) ;
				p8hex( SdAddress ) ;
				txmit( ':' ) ;
				p8hex( i ) ;
				crlf() ;
				SdAddress += 1 ;
//			break ;
//		}
	}


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

	if ( rxchar == 'F' )
	{
		txmit( 'F' ) ;
		txmit( '-' ) ;
  	p2hex( ( PIOB->PIO_PDSR & PIO_PB7 ) ? 0 : 1 ) ;
		crlf() ;
	}	
	
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
		playTone( 1000, 50 ) ;
//		tone_start( 50 ) ;
	}

	if ( rxchar == '!' )
	{
		playTone( 1000, 20000 ) ;
//		tone_start( 0 ) ;
	}

	if ( rxchar == '.' )
	{
		playTone( 0, 0 ) ;
//		tone_stop() ;
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



