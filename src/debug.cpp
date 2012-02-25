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

#include "s9xsplash.lbm"

extern uint32_t Lcd_analog_display ;
extern uint32_t Per10ms_action ;
extern uint32_t Permenu_action ;
extern EEGeneral  g_eeGeneral;
extern ModelData  g_model;
extern PROGMEM s9xsplash[] ;

extern void disp_256( uint32_t address, uint32_t lines ) ;
extern uint8_t eeprom[] ;


extern void screen0( void ) ;

//const unsigned char Hello[] = {"Hello ERSKY9X"} ;
//const unsigned char Ersky9x[] = {"ERSKY9X"} ;



uint8_t Spi_tx_buf[24] ;
uint8_t Spi_rx_buf[24] ;






uint32_t Eeprom_image_updated ;		// Ram image changed
uint32_t Eeprom_sequence_no ;			// Ram image changed
uint8_t Current_eeprom_block ;		// 0 or 1 is active block
uint8_t Other_eeprom_block_blank ;
uint8_t Eeprom_process_state ;
uint8_t Eeprom_process_sub_no ;		// Used to manage writes
uint8_t	Eeprom_write_pending ;
uint8_t	Eeprom_writing_block_no ;


// States in Eeprom_process_state
#define E_IDLE							1
#define E_ERASESENDING			2
#define E_ERASEWAITING			3
#define E_WRITESENDING			4
#define E_WRITEWAITING			5


void handle_serial( void ) ;
//void hello( void ) ;
//void dbl9x( void ) ;
//uint32_t read_switch( enum EnumKeys enuk ) ;
uint32_t read_eeprom_block( uint32_t block_no, uint32_t immediate ) ;
uint32_t write_eeprom_block( uint32_t block_no, uint32_t sub_no, uint32_t size, uint32_t immediate ) ;
uint32_t eeprom_image_blank( uint32_t image_index ) ;

void eeprom_process( void ) ;

// New file system

// Start with 16 model memories
// Blocks 0 and 1, general
// Blocks 2 and 3, model 1
// Blocks 4 and 5, model 2 etc
// Blocks 32 and 33, model 16

uint8_t Current_general_block ;		// 0 or 1 is active block
uint8_t Other_general_block_blank ;

struct t_eeprom_header
{
	uint32_t sequence_no ;		// sequence # to decide which block is most recent
	uint16_t data_size ;			// # bytes in data area
	uint16_t flags ;					// Unused, for expansion
} ;

// Structure of data in a block
struct t_eeprom_block
{
	struct t_eeprom_header header ;
	union
	{
		uint8_t bytes[4088] ;
		uint32_t words[1022] ;
	} data ;
} ;

//struct t_eeprom_block E_images[2] ;

// Check all 4096 bytes of an image to see if they are blank
//uint32_t eeprom_image_blank( uint32_t image_index )
//{
//	register uint32_t x ;
//	register uint32_t *p ;

//	p = &E_images[image_index].sequence_no ;

//	for ( x = 0 ; x < 1024 ; x += 1 )
//	{
//		if ( *p++ != 0xFFFFFFFF )
//		{
//			return 0 ;
//		}		
//	}
//	return 1 ;
//}


// Read eeprom data starting at a 4096 byte block boundary
//uint32_t read_eeprom_block( uint32_t block_no, register uint8_t *buffer, uint32_t size, uint32_t immediate )
//{
//	register uint8_t *p ;
//	register uint32_t x ;

////	p = E_images[block_no].spi_command ;
//	*p = 3 ;		// Read command
//	*(p+1) = 0 ;
//	*(p+2) = block_no << 4 ;
//	*(p+3) = 0 ;		// 3 bytes address
//	spi_PDC_action( p, 0, buffer, 4, size + 4 ) ;

//	if ( immediate )
//	{
//		return 0 ;		
//	}
//	for ( x = 0 ; x < 100000 ; x += 1  )
//	{
//		if ( Spi_complete )
//		{
//			break ;				
//		}        			
//	}
//	return x ; 
//}

// Pass in an even block number, this and the next block will be checked
// to see which is the most recent, the block_no of the most recent
// is returned, with the corresponding data size if required
//uint32_t get_current_block_number( uint32_t block_no, uint16_t *p_size )
//{
//	struct t_eeprom_header b0 ;
//	struct t_eeprom_header b1 ;
//  uint16_t size ;
//	read_eeprom_block( block_no, ( uint8_t *)&b0, sizeof(b0), 0 ) ;		// Sequence # 0
//	read_eeprom_block( block_no+1, ( uint8_t *)&b1, sizeof(b1), 0 ) ;	// Sequence # 1

//  size = b0.data_size ;
//	if ( b0.sequence_no == 0xFFFFFFFF )
//	{
//		if ( b1.sequence_no != 0xFFFFFFFF )
//		{
//  		size = b1.data_size ;
//			block_no += 1 ;
//		}
//	}
//	else
//	{
//		if ( b1.sequence_no != 0xFFFFFFFF )
//		{
//			if ( b0.sequence_no > b1.sequence_no )
//			{
//	  		size = b1.data_size ;
//				block_no += 1 ;
//			}
//		}
//	}
//  if ( size == 0xFFFF )
//	{
//		size = 0 ;
//	}
//  if ( p_size )
//	{
//		*p_size = size ;
//	}
//  return block_no ;
//}

// bool eeLoadGeneral()
//{
//	register uint8_t *p ;
//	register uint8_t *q ;
//	uint32_t block_no ;
//	uint16_t size ;
//	uint32_t count ;
//	block_no = get_current_block_number( 0, &size ) ;

//	read_eeprom_block( block_no, ( uint8_t *)&E_images[0], size + sizeof(struct t_eeprom_header), 0 ) ;
  
//  memset(&g_eeGeneral, 0, sizeof(EEGeneral));

//	if ( size > sizeof(EEGeneral) )
//	{
//		size = sizeof(EEGeneral) ;
//	}
	
	// Copy data to general
//	p = E_images[0].data.bytes ;
//	q = (uint8_t*)&g_eeGeneral ;
//	for ( count = 0 ; count < size ; count += 1 )
//	{
//		*q++ = *p++ ;			
//	}
  
//  for(uint8_t i=0; i<sizeof(g_eeGeneral.ownerName);i++) // makes sure name is valid
//  {
//      uint8_t idx = char2idx(g_eeGeneral.ownerName[i]);
//      g_eeGeneral.ownerName[i] = idx2char(idx);
//  }

//  if(g_eeGeneral.myVers<MDVERS)
//      sysFlags |= sysFLAG_OLD_EEPROM; // if old EEPROM - Raise flag

//  g_eeGeneral.myVers   =  MDVERS; // update myvers

//  uint16_t sum=0;
//  if(sz>(sizeof(EEGeneral)-20)) for(uint8_t i=0; i<12;i++) sum+=g_eeGeneral.calibMid[i];
//  return g_eeGeneral.chkSum == sum;
//}


//void eeLoadModel(uint8_t id)
//{
//	register uint8_t *p ;
//	register uint8_t *q ;
//	uint32_t block_no ;
//	uint16_t size ;
//	uint32_t count ;

//    if(id<MAX_MODELS)
//    {
//			block_no = get_current_block_number( id * 2, &size ) ;

//			read_eeprom_block( block_no, ( uint8_t *)&E_images[1], size + sizeof(struct t_eeprom_header), 0 ) ;
				
//				memset(&g_model, 0, sizeof(g_model));
        
//			if ( size > sizeof(g_model) )
//			{
//				size = sizeof(g_model) ;
//			}
			
	// Copy data to model
//			p = E_images[1].data.bytes ;
//			q = (uint8_t*)&g_eeGeneral ;
//			for ( count = 0 ; count < size ; count += 1 )
//			{
//				*q++ = *p++ ;			
//			}
				
//      if(size<256) // if not loaded a fair amount
//      {
//          modelDefault(id);
//      }

//      for(uint8_t i=0; i<sizeof(g_model.name);i++) // makes sure name is valid
//      {
//          uint8_t idx = char2idx(g_model.name[i]);
//          g_model.name[i] = idx2char(idx);
//      }

//      g_model.mdVers = MDVERS; //update mdvers

//      resetTimer2();

//#ifdef FRSKY
//  FrskyAlarmSendState |= 0x40 ;		// Get RSSI Alarms
//        FRSKY_setModelAlarms();
//#endif
//    }
//}

//bool eeModelExists(uint8_t id)
//{
//	uint32_t block_no ;
//	uint16_t size ;
//	block_no = get_current_block_number( id * 2, &size ) ;
//	return ( size > 0 ) ;
//}







// Temporary eeprom handling - hence it's here

// storage defination
struct t_eeprom_image
{
	uint8_t spi_command[4] ;
	union
	{
		uint8_t bytes[2048] ;
		uint32_t words[512] ;
	} image ;
	uint32_t sequence_no ;	
	uint32_t filler[3] ;	
	union
	{
		uint8_t bytes[2016] ;
		uint32_t words[504] ;
	} imagex ;
} ;

// storage declaration
struct t_eeprom_image E_images[2] ;

static uint32_t Update_timer ;

void eeprom_process()
{
	register uint8_t *p ;
	register uint8_t *q ;
	register uint32_t block_no ;
	register uint32_t x ;
	
	if ( Eeprom_image_updated )
	{
		Eeprom_image_updated = 0 ;
		Update_timer = 100 ;		// 1 second
	}
	if ( Update_timer )
	{
		if ( --Update_timer == 0 )
		{
			// Changed, but no changes for 2 seconds
			// Time to write changes to eeprom
			Eeprom_write_pending = 1 ;
		}
	}

	if ( Eeprom_process_state == E_IDLE )
	{
		if ( Other_eeprom_block_blank == 0 )
		{
			// Need to erase it
			Eeprom_process_state = E_ERASESENDING ;
			block_no = 0 ;
			if ( Current_eeprom_block == 0 )
			{
				block_no = 1 ;
			}
			eeprom_write_enable() ;
			p = E_images[block_no].spi_command ;
			*p = 0x20 ;		// Block Erase command
			*(p+1) = 0 ;
			*(p+2) = block_no << 4 ;
			*(p+3) = 0 ;		// 3 bytes address
			spi_PDC_action( p, 0, 0, 4, 0 ) ;
		}
		else if ( Eeprom_write_pending ) 
		{
			Eeprom_write_pending = 0 ;
			block_no = 0 ;
			if ( Current_eeprom_block == 0 )
			{
				block_no = 1 ;
			}
			// Copy data from RAM based eeprom image

			p = E_images[block_no].image.bytes ;
			q = eeprom ;
			for ( x = 0 ; x < 2048 ; x += 1 )
			{
				*p++ = *q++ ;			
			}
			E_images[block_no].sequence_no = Eeprom_sequence_no + 1 ;
			Eeprom_process_sub_no = 0 ;
			write_eeprom_block( block_no, Eeprom_process_sub_no, 256, 1 ) ;
			Eeprom_process_state = E_WRITESENDING ;
			Eeprom_writing_block_no = block_no ;
		}
	}
	
	if ( Eeprom_process_state == E_ERASESENDING )
	{
		if ( Spi_complete )
		{
			Eeprom_process_state = E_ERASEWAITING ;
		}			
	}	
		
	if ( Eeprom_process_state == E_ERASEWAITING )
	{
		x = eeprom_read_status() ;
		if ( ( x & 2 ) == 0 )
		{ // Command finished
			Eeprom_process_state = E_IDLE ;
			Other_eeprom_block_blank = 1 ;
		}			
	}

	if ( Eeprom_process_state == E_WRITESENDING )
	{
		if ( Spi_complete )
		{
			Eeprom_process_state = E_WRITEWAITING ;
		}			
	}		
	
	if ( Eeprom_process_state == E_WRITEWAITING )
	{
		x = eeprom_read_status() ;
		if ( ( x & 2 ) == 0 )
		{
			register uint32_t size ;
			Eeprom_process_sub_no += 1 ;
			size = 256 ;
			if ( Eeprom_process_sub_no == 8)
			{
				size = 4 ;
			}
			if ( Eeprom_process_sub_no > 8)
			{
				size = 0 ;
			}
			if ( size > 0 )
			{
				write_eeprom_block( Eeprom_writing_block_no, Eeprom_process_sub_no, size, 1 ) ;
				Eeprom_process_state = E_WRITESENDING ;
			}
			else
			{
				Eeprom_process_state = E_IDLE ;
				Current_eeprom_block = Eeprom_writing_block_no ;
				Other_eeprom_block_blank = 0 ;
			}
		}
	}	
}


void init_eeprom()
{
	register uint32_t x ;
	register uint32_t y ;
	register uint8_t *p ;
	register uint8_t *q ;
//	register uint32_t valid ;
	
	read_eeprom_block( 0, 0 ) ;
	read_eeprom_block( 1, 0 ) ;

	// Here we should find which block is the most recent
	x = eeprom_image_blank( 0 ) ;
	y = eeprom_image_blank( 1 ) ;

	Eeprom_process_state = E_IDLE ;
	Eeprom_process_sub_no = 0 ;
	Eeprom_write_pending = 0 ;
	if ( x )
	{
		if ( y )
		{ // Both blank
			Other_eeprom_block_blank = 1 ;
			Current_eeprom_block = 1 ;
			Eeprom_sequence_no = 0 ;
		}
		else
		{ // Block 1 is active, 0 is blank
			Other_eeprom_block_blank = 1 ;
			Current_eeprom_block = 1 ;
			Eeprom_sequence_no = E_images[1].sequence_no ;
		}
	}
	else
	{
		if ( y )
		{ // Block 0 is active, 1 is blank
			Other_eeprom_block_blank = 1 ;
			Current_eeprom_block = 0 ;
			Eeprom_sequence_no = E_images[0].sequence_no ;
			
		}
		else
		{ // Check sequence number and erase other block
			Other_eeprom_block_blank = 0 ;
			x = E_images[0].sequence_no ;
			y = E_images[1].sequence_no ;
			if ( y == 0xFFFFFFFF )
			{
				y = 0 ;				
			}
			if ( x == 0xFFFFFFFF )
			{
				x = 0 ;				
			}
			if ( x > y )			// Simple test, assumes no 32 bit overflow
			{
				Current_eeprom_block = 0 ;
				Eeprom_sequence_no = x ;
			}
			else
			{
				Current_eeprom_block = 1 ;
				Eeprom_sequence_no = y ;
			}
		}
	}

	// Copy valid block to RAM eeprom image
	p = E_images[Current_eeprom_block].image.bytes ;
	q = eeprom ;
	for ( x = 0 ; x < 2048 ; x += 1 )
	{
		*q++ = *p++ ;			
	}
//	disp_256( (uint32_t)eeprom, 6 ) ;
	Eeprom_process_state = E_IDLE ;
}


uint32_t eeprom_image_blank( uint32_t image_index )
{
	register uint32_t x ;
	register uint32_t *p ;
	register uint32_t result ;

	result = 1 ;
	p = E_images[image_index].image.words ;

	for ( x = 0 ; x < 512 ; x += 1 )
	{
		if ( *p++ != 0xFFFFFFFF )
		{
			result = 0 ;			
		}		
	}
	if ( result )
	{
		x = E_images[image_index].sequence_no ;
		if ( x != 0xFFFFFFFF )
		{
			result = 0 ;			
		}
	}
	return result ;
}


uint32_t read_eeprom_block( uint32_t block_no, uint32_t immediate )
{
	register uint8_t *p ;
	register uint32_t x ;

	p = E_images[block_no].spi_command ;
	*p = 3 ;		// Read command
	*(p+1) = 0 ;
	*(p+2) = block_no << 4 ;
	*(p+3) = 0 ;		// 3 bytes address
	spi_PDC_action( p, 0, E_images[block_no].image.bytes, 4, 2048 + 4 ) ;

	if ( immediate )
	{
		return 0 ;		
	}
	for ( x = 0 ; x < 100000 ; x += 1  )
	{
		if ( Spi_complete )
		{
			break ;				
		}        			
	}
	return x ; 

//	return spi_action( p, E_images[block_no].image, E_images[block_no].image, 4, 2048 + 4 ) ;
}

uint32_t write_eeprom_block( uint32_t block_no, uint32_t sub_no, uint32_t size, uint32_t immediate )
{
	register uint8_t *p ;
	register uint32_t x ;

	eeprom_write_enable() ;
	
	p = E_images[block_no].spi_command ;
	*p = 2 ;		// Write command
	*(p+1) = 0 ;
	*(p+2) = (block_no << 4) + sub_no ;
	*(p+3) = 0 ;		// 3 bytes address
	spi_PDC_action( p, &E_images[block_no].image.bytes[sub_no << 8], 0, 4, size ) ;

	if ( immediate )
	{
		return 0 ;		
	}
	for ( x = 0 ; x < 100000 ; x += 1  )
	{
		if ( Spi_complete )
		{
			break ;				
		}        			
	}
	return x ; 
//	return spi_action( p, E_images[block_no].image[sub_no << 8], 0, 4, size ) ;
}



uint32_t write_eeprom_status( uint8_t value )
{
	register uint8_t *p ;

	eeprom_write_enable() ;
		
	p = Spi_tx_buf ;
	*p = 1 ;		// Write status register command
	*(p+1) = value ;

	return spi_operation( p, Spi_rx_buf, 2 ) ;
}

uint32_t unprotect_eeprom()
{
 	register uint8_t *p ;

	eeprom_write_enable() ;
		
	p = Spi_tx_buf ;
	*p = 0x39 ;		// Unprotect sector command
	*(p+1) = 0 ;
	*(p+2) = 0 ;
	*(p+3) = 0 ;		// 3 bytes address

	return spi_operation( p, Spi_rx_buf, 4 ) ;
}


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

	if ( rxchar == 'A' )
	{
		if ( Permenu_action )
		{
			Permenu_action = 0 ;			
		}
		else
		{
			Permenu_action = 1 ;
		}
		
	}

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


	if ( rxchar == 'D' )
	{
		if ( Lcd_analog_display )
		{
			Lcd_analog_display = 0 ;			
		}
		else
		{
			Lcd_analog_display = 1 ;
		}
	}
        
				
	if ( rxchar == 'Y' )
	{
		lcd_clear();
  	lcd_img(0, 0, s9xsplash,0,0);
  	lcd_putsnAtt( 0*FW, 7*FH, g_eeGeneral.ownerName ,sizeof(g_eeGeneral.ownerName),0);
  	lcd_putsnAtt( 4*FW, 3*FH, "SKY" , 3, DBLSIZE ) ;
		refreshDisplay();
	}

	if ( rxchar == 'W' )
	{
		if ( Per10ms_action )
		{
			Per10ms_action = 0 ;			
		}
		else
		{
			Per10ms_action = 1 ;
		}
	}

	if ( rxchar == 'J' )
	{
		register uint32_t x ;
		x = read_eeprom_block( 0, 0 ) ;
		txmit( 'J' ) ;
		p8hex( x ) ;
		crlf() ;
		disp_256( (uint32_t)E_images[0].image.bytes, 16 ) ;
		crlf() ;
	}

	if ( rxchar == 'P' )
	{
		register uint8_t *p ;
		register uint32_t x ;

		p = Spi_tx_buf ;
		*p = 3 ;		// Read command
		*(p+1) = 0 ;
		*(p+2) = 0 ; // block_no << 4 ;
		*(p+3) = 0 ;		// 3 bytes address
		x = spi_PDC_action( p, 0, E_images[0/*block_no*/].image.bytes, 4, 2048 + 4 ) ;

		p8hex( x ) ;
		txmit( ' ' ) ;

		for ( x = 0 ; x < 100000 ; x += 1  )
		{
			if ( Spi_complete )
			{
				break ;				
			}        			
		}
		p8hex( Spi_complete ) ;
		crlf() ;
		disp_256( (uint32_t)E_images[0].image.bytes, 5 ) ;
		disp_256( (uint32_t)&E_images[0].image.bytes[2048-16], 2 ) ;
		crlf() ;
		p = Spi_tx_buf ;
		*p = 3 ;		// Read command
		*(p+1) = 0 ;
		*(p+2) = 1 << 4 ; // block_no << 4 ;
		*(p+3) = 0 ;		// 3 bytes address
		x = spi_PDC_action( p, 0, E_images[1/*block_no*/].image.bytes, 4, 2048 + 4 ) ;

		p8hex( x ) ;
		txmit( ' ' ) ;

		for ( x = 0 ; x < 100000 ; x += 1  )
		{
			if ( Spi_complete )
			{
				break ;				
			}        			
		}
		p8hex( Spi_complete ) ;
		crlf() ;
		disp_256( (uint32_t)E_images[1].image.bytes, 5 ) ;
		disp_256( (uint32_t)&E_images[1].image.bytes[2048-16], 2 ) ;
		crlf() ;
		p2hex( Eeprom_process_state ) ;
		p2hex( Eeprom_process_sub_no ) ;
		p2hex( Eeprom_writing_block_no ) ;
		p2hex( Eeprom_write_pending ) ;
		txmit( ' ' ) ;
		p8hex( Eeprom_image_updated ) ;
		crlf() ;
	}


	if ( rxchar == 'N' )
	{
		register uint32_t x ;
		register uint8_t *p ;
		register uint8_t *q ;

		p = E_images[0].image.bytes ;
		q = eeprom ;
		for ( x = 0 ; x < 2048 ; x += 1 )
		{
			*p++ = *q++ ;			
		}

		txmit( 'N' ) ;
		
		x = write_eeprom_block( 0, 0, 256, 0 ) ;
		p8hex( x ) ;
		crlf() ;
		disp_256( (uint32_t)E_images[0].image.bytes, 16 ) ;
		crlf() ;
		disp_256( (uint32_t)&SPI->SPI_RPR, 4 ) ;
	}

	if ( rxchar == 'O' )
	{
		register uint8_t *p ;
		register uint32_t x ;

		txmit( 'O' ) ;

		p = Spi_rx_buf ;
		*p = 0x55 ;
		*(p+1) = 0x55 ;
		*(p+2) = 0x55 ;
		*(p+3) = 0x55 ;

		p = Spi_tx_buf ;
		
		*p = 0x9F ;		// Get Manuf. Code
		*(p+1) = 0 ;

		x = spi_PDC_action( p, 0, Spi_rx_buf, 1, 4 ) ;

		p8hex( x ) ;
		txmit( ' ' ) ;

		for ( x = 0 ; x < 100000 ; x += 1  )
		{
			if ( Spi_complete )
			{
				break ;				
			}        			
		}
		p8hex( Spi_complete ) ;
		txmit( ' ' ) ;

		p = Spi_rx_buf ;
		p2hex( *p ) ;
		p2hex( *(p+1) ) ;
		p2hex( *(p+2) ) ;
		p2hex( *(p+3) ) ;
		crlf() ;
//		disp_256( (uint32_t)&SPI->SPI_RPR, 4 ) ;

	}

	
	if ( rxchar == 'Q' )
	{
		register uint8_t *p ;
		register uint32_t x ;

		txmit( 'Q' ) ;
		p = Spi_rx_buf ;
		*(p+4) = 0 ;
		*(p+5) = 0 ;
		*(p+19) = 0 ;

		p = Spi_tx_buf ;
		*p = 3 ;		// Read command
		*(p+1) = 0 ;
		*(p+2) = 0 ;
		*(p+3) = 0 ;		// 3 bytes address
		x = spi_operation( p, Spi_rx_buf, 20 ) ;

		p8hex( x ) ;
		txmit( ' ' ) ;
		p = Spi_rx_buf ;
		p2hex( *(p+4) ) ;
		p2hex( *(p+5) ) ;
		p2hex( *(p+18) ) ;
		p2hex( *(p+19) ) ;
		crlf() ;
	}

	if ( rxchar == 'E' )
	{
		register uint8_t *p ;
		register uint32_t x ;

		txmit( 'E' ) ;
		p = Spi_rx_buf ;
		*(p+1) = 0 ;
		*(p+2) = 0 ;

		p = Spi_tx_buf ;
		*p = 5 ;		// Read status command
		*(p+1) = 0 ;
		x = spi_operation( p, Spi_rx_buf, 2 ) ;

		p8hex( x ) ;
		txmit( ' ' ) ;
		p = Spi_rx_buf ;
		p2hex( *p ) ;
		p2hex( *(p+1) ) ;
		p2hex( *(p+2) ) ;
		p2hex( *(p+3) ) ;
		crlf() ;
	}
	
	if ( rxchar == 'F' )
	{
		register uint8_t *p ;
		register uint32_t x ;

		txmit( 'F' ) ;

		eeprom_write_enable() ;
		
		p = Spi_tx_buf ;
		*p = 1 ;		// Write status register command
		*(p+1) = 0 ;

		x = spi_operation( p, Spi_rx_buf, 2 ) ;
		p8hex( x ) ;
		crlf() ;
	}
	
	if ( rxchar == 'X' )
	{
		register uint8_t *p ;
		register uint32_t x ;
		
		txmit( 'X' ) ;
		eeprom_write_enable() ;
		
		p = Spi_tx_buf ;
		*p = 0x20 ;		// Block Erase command
		*(p+1) = 0 ;
		*(p+2) = 0 << 4 ;		// Erase block 0
		*(p+3) = 0 ;		// 3 bytes address
		
		x = spi_operation( p, Spi_rx_buf, 4 ) ;
		p8hex( x ) ;
		crlf() ;
	}
	
	if ( rxchar == 'G' )
	{
		// Write complete eeprom image
		register uint32_t x ;
		register uint32_t y ;
		register uint8_t *p ;
		register uint8_t *q ;

		p = E_images[0].image.bytes ;
		q = eeprom ;
		for ( x = 0 ; x < 2048 ; x += 1 )
		{
			*p++ = *q++ ;			
		}
		E_images[0].sequence_no = 1 ;	

		for ( y = 0 ; y < 8 ; y += 1 )
		{
			x = write_eeprom_block( 0, y, 256, 0 ) ;
			p8hex(x) ;
			crlf() ;
		}
		x = write_eeprom_block( 0, 8, 4, 0 ) ;
		p8hex(x) ;
		crlf() ;
	}

	if ( rxchar == 'C' )
	{
		register uint8_t *p ;
		register uint32_t x ;

		txmit( 'C' ) ;

		eeprom_write_enable() ;
		
		p = Spi_tx_buf ;
		*p = 0x39 ;		// Unprotect sector command
		*(p+1) = 0 ;
		*(p+2) = 0 ;
		*(p+3) = 0 ;		// 3 bytes address

		x = spi_operation( p, Spi_rx_buf, 4 ) ;

		p8hex( x ) ;
		crlf() ;
	}
	 
	if ( rxchar == 'U' )
	{
		register uint8_t *p ;
		register uint32_t x ;

		txmit( 'U' ) ;

		eeprom_write_enable() ;
		p = Spi_tx_buf ;
		
		*p = 2 ;		// Write command
		*(p+1) = 0 ;
		*(p+2) = 0 ;
		*(p+3) = 0 ;		// 3 bytes address
		*(p+4) = 1 ;
		*(p+5) = 2 ;
		*(p+18) = 0x0F ;
		*(p+19) = 0x10 ;

		x = spi_operation( p, Spi_rx_buf, 20 ) ;
		p8hex( x ) ;
		crlf() ;
	}

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
		set_frequency( 100 ) ;
	}
	
	if ( rxchar == '=' )
	{
		set_frequency( 1000 ) ;
	}
	
	if ( rxchar == ')' )
	{
		set_frequency( 5500 ) ;
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
	if ( ( rxchar >= '0' ) && ( rxchar <= '7' ) )
	{
		disp_256( ( uint32_t)eeprom + (rxchar - '0') * 256, 16 ) ;
	}

	if ( rxchar == '@' )
	{
  	strncpy_P(g_eeGeneral.ownerName,PSTR("MIKE      "), 10) ;
		STORE_GENERALVARS ;
	}


}


void disp_mem( register uint32_t address )
{
	p8hex( address ) ;
	txmit('=') ;
	p8hex( *( (uint32_t *)address ) ) ;
	crlf() ;
}

void disp_256( register uint32_t address, register uint32_t lines )
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



