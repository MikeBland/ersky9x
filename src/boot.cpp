/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2011, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */


/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "AT91SAM3S4.h"
#include "..\src\core_cm3.h"
#include "..\src\lcd.h"
#include "..\src\ersky9x.h"
#include "..\src\lcd.h"
#include "..\src\ff.h"
#include "..\src\diskio.h"
#include "..\src\drivers.h"
#include "..\src\myeeprom.h"

extern void usbMassStorage( void ) ;

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

EEGeneral g_eeGeneral ;

uint32_t Block_buffer[1024] ;
uint8_t Eblock_buffer[4096] ;
UINT BlockCount ;

uint32_t FirmwareSize ;

uint32_t Master_frequency ;
volatile uint8_t  Tenms ;

//extern uint32_t Card_state ;
extern uint32_t sd_card_ready( void ) ;
//extern uint32_t Cmd_A41_resp ;

TCHAR FlashFilename[60] ;
FATFS g_FATFS ;
FIL FlashFile ;
DIR Dj ;
FILINFO Finfo ;

TCHAR Filenames[20][50] ;
uint32_t FileSize[20] ;
uint32_t FnStartIndex ;
uint32_t Valid ;

uint32_t FlashSize ;

uint8_t Spi_tx_buf[24] ;
uint8_t Spi_rx_buf[24] ;
uint32_t Spi_init_done = 0 ;

uint32_t EepromBlocked = 1 ;
uint32_t FlashBlocked = 1 ;
uint32_t LockBits ;

uint32_t  eeprom_write_one( uint8_t byte, uint8_t count ) ;
uint32_t eeprom_read_status( void ) ;
void eeprom_write_enable( void ) ;
void eeprom_wait_busy( void ) ;
uint32_t spi_operation( register uint8_t *tx, register uint8_t *rx, register uint32_t count ) ;
void init_spi( void ) ;
uint32_t unprotect_eeprom( void ) ;
uint32_t spi_PDC_action( register uint8_t *command, register uint8_t *tx, register uint8_t *rx, register uint32_t comlen, register uint32_t count ) ;
void AT25D_Read( uint8_t *BufferAddr, uint32_t size, uint32_t memoryOffset) ;
void AT25D_Write( uint8_t *BufferAddr, uint32_t size, uint32_t memoryOffset ) ;
uint32_t AT25D_EraseBlock( uint32_t memoryOffset ) ;


/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

// Starts TIMER0 at full speed (MCK/2) for delay timing
// @ 36MHz this is 18MHz
// This was 6 MHz, we may need to slow it to TIMER_CLOCK2 (MCK/8=4.5 MHz)
void start_timer0()
{
  register Tc *ptc ;

  PMC->PMC_PCER0 |= 0x00800000L ;		// Enable peripheral clock to TC0

  ptc = TC0 ;		// Tc block 0 (TC0-2)
	ptc->TC_BCR = 0 ;			// No sync
	ptc->TC_BMR = 2 ;
	ptc->TC_CHANNEL[0].TC_CMR = 0x00008001 ;	// Waveform mode MCK/8 for 36MHz osc.(Upset be write below)
	ptc->TC_CHANNEL[0].TC_RC = 0xFFF0 ;
	ptc->TC_CHANNEL[0].TC_RA = 0 ;
	ptc->TC_CHANNEL[0].TC_CMR = 0x00008040 ;	// 0000 0000 0000 0000 1000 0000 0100 0000, stop at regC, 18MHz
	ptc->TC_CHANNEL[0].TC_CCR = 5 ;		// Enable clock and trigger it (may only need trigger)
}

void stop_timer0( void )
{
	TC0->TC_CHANNEL[0].TC_CCR = TC_CCR0_CLKDIS ;		// Disable clock
}

void delay2ms()
{
	TC0->TC_CHANNEL[0].TC_CCR = 5 ;	// Enable clock and trigger it (may only need trigger)
	while ( TC0->TC_CHANNEL[0].TC_CV < 36000 )		// 2mS, Value depends on MCK/2 (used 18MHz)
	{
		// Wait
	}
}


#define _TBLDEF 1
static
const WCHAR Tbl[] = {	/*  CP437(0x80-0xFF) to Unicode conversion table */
	0x00C7, 0x00FC, 0x00E9, 0x00E2, 0x00E4, 0x00E0, 0x00E5, 0x00E7,
	0x00EA, 0x00EB, 0x00E8, 0x00EF, 0x00EE, 0x00EC, 0x00C4, 0x00C5,
	0x00C9, 0x00E6, 0x00C6, 0x00F4, 0x00F6, 0x00F2, 0x00FB, 0x00F9,
	0x00FF, 0x00D6, 0x00DC, 0x00A2, 0x00A3, 0x00A5, 0x20A7, 0x0192,
	0x00E1, 0x00ED, 0x00F3, 0x00FA, 0x00F1, 0x00D1, 0x00AA, 0x00BA,
	0x00BF, 0x2310, 0x00AC, 0x00BD, 0x00BC, 0x00A1, 0x00AB, 0x00BB,
	0x2591, 0x2592, 0x2593, 0x2502, 0x2524, 0x2561, 0x2562, 0x2556,
	0x2555, 0x2563, 0x2551, 0x2557, 0x255D, 0x255C, 0x255B, 0x2510,
	0x2514, 0x2534, 0x252C, 0x251C, 0x2500, 0x253C, 0x255E, 0x255F,
	0x255A, 0x2554, 0x2569, 0x2566, 0x2560, 0x2550, 0x256C, 0x2567,
	0x2568, 0x2564, 0x2565, 0x2559, 0x2558, 0x2552, 0x2553, 0x256B,
	0x256A, 0x2518, 0x250C, 0x2588, 0x2584, 0x258C, 0x2590, 0x2580,
	0x03B1, 0x00DF, 0x0393, 0x03C0, 0x03A3, 0x03C3, 0x00B5, 0x03C4,
	0x03A6, 0x0398, 0x03A9, 0x03B4, 0x221E, 0x03C6, 0x03B5, 0x2229,
	0x2261, 0x00B1, 0x2265, 0x2264, 0x2320, 0x2321, 0x00F7, 0x2248,
	0x00B0, 0x2219, 0x00B7, 0x221A, 0x207F, 0x00B2, 0x25A0, 0x00A0
};


WCHAR ff_convert (	/* Converted character, Returns zero on error */
	WCHAR	chr,	/* Character code to be converted */
	UINT	dir		/* 0: Unicode to OEMCP, 1: OEMCP to Unicode */
)
{
	WCHAR c;


	if (chr < 0x80) {	/* ASCII */
		c = chr;

	} else {
		if (dir) {		/* OEMCP to Unicode */
			c = (chr >= 0x100) ? 0 : Tbl[chr - 0x80];

		} else {		/* Unicode to OEMCP */
			for (c = 0; c < 0x80; c++) {
				if (chr == Tbl[c]) break;
			}
			c = (c + 0x80) & 0xFF;
		}
	}

	return c;
}


WCHAR ff_wtoupper (	/* Upper converted character */
	WCHAR chr		/* Input character */
)
{
	static const WCHAR tbl_lower[] = { 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6A, 0x6B, 0x6C, 0x6D, 0x6E, 0x6F, 0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0xA1, 0x00A2, 0x00A3, 0x00A5, 0x00AC, 0x00AF, 0xE0, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7, 0xE8, 0xE9, 0xEA, 0xEB, 0xEC, 0xED, 0xEE, 0xEF, 0xF0, 0xF1, 0xF2, 0xF3, 0xF4, 0xF5, 0xF6, 0xF8, 0xF9, 0xFA, 0xFB, 0xFC, 0xFD, 0xFE, 0x0FF, 0x101, 0x103, 0x105, 0x107, 0x109, 0x10B, 0x10D, 0x10F, 0x111, 0x113, 0x115, 0x117, 0x119, 0x11B, 0x11D, 0x11F, 0x121, 0x123, 0x125, 0x127, 0x129, 0x12B, 0x12D, 0x12F, 0x131, 0x133, 0x135, 0x137, 0x13A, 0x13C, 0x13E, 0x140, 0x142, 0x144, 0x146, 0x148, 0x14B, 0x14D, 0x14F, 0x151, 0x153, 0x155, 0x157, 0x159, 0x15B, 0x15D, 0x15F, 0x161, 0x163, 0x165, 0x167, 0x169, 0x16B, 0x16D, 0x16F, 0x171, 0x173, 0x175, 0x177, 0x17A, 0x17C, 0x17E, 0x192, 0x3B1, 0x3B2, 0x3B3, 0x3B4, 0x3B5, 0x3B6, 0x3B7, 0x3B8, 0x3B9, 0x3BA, 0x3BB, 0x3BC, 0x3BD, 0x3BE, 0x3BF, 0x3C0, 0x3C1, 0x3C3, 0x3C4, 0x3C5, 0x3C6, 0x3C7, 0x3C8, 0x3C9, 0x3CA, 0x430, 0x431, 0x432, 0x433, 0x434, 0x435, 0x436, 0x437, 0x438, 0x439, 0x43A, 0x43B, 0x43C, 0x43D, 0x43E, 0x43F, 0x440, 0x441, 0x442, 0x443, 0x444, 0x445, 0x446, 0x447, 0x448, 0x449, 0x44A, 0x44B, 0x44C, 0x44D, 0x44E, 0x44F, 0x451, 0x452, 0x453, 0x454, 0x455, 0x456, 0x457, 0x458, 0x459, 0x45A, 0x45B, 0x45C, 0x45E, 0x45F, 0x2170, 0x2171, 0x2172, 0x2173, 0x2174, 0x2175, 0x2176, 0x2177, 0x2178, 0x2179, 0x217A, 0x217B, 0x217C, 0x217D, 0x217E, 0x217F, 0xFF41, 0xFF42, 0xFF43, 0xFF44, 0xFF45, 0xFF46, 0xFF47, 0xFF48, 0xFF49, 0xFF4A, 0xFF4B, 0xFF4C, 0xFF4D, 0xFF4E, 0xFF4F, 0xFF50, 0xFF51, 0xFF52, 0xFF53, 0xFF54, 0xFF55, 0xFF56, 0xFF57, 0xFF58, 0xFF59, 0xFF5A, 0 };
	static const WCHAR tbl_upper[] = { 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A, 0x4B, 0x4C, 0x4D, 0x4E, 0x4F, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59, 0x5A, 0x21, 0xFFE0, 0xFFE1, 0xFFE5, 0xFFE2, 0xFFE3, 0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xCB, 0xCC, 0xCD, 0xCE, 0xCF, 0xD0, 0xD1, 0xD2, 0xD3, 0xD4, 0xD5, 0xD6, 0xD8, 0xD9, 0xDA, 0xDB, 0xDC, 0xDD, 0xDE, 0x178, 0x100, 0x102, 0x104, 0x106, 0x108, 0x10A, 0x10C, 0x10E, 0x110, 0x112, 0x114, 0x116, 0x118, 0x11A, 0x11C, 0x11E, 0x120, 0x122, 0x124, 0x126, 0x128, 0x12A, 0x12C, 0x12E, 0x130, 0x132, 0x134, 0x136, 0x139, 0x13B, 0x13D, 0x13F, 0x141, 0x143, 0x145, 0x147, 0x14A, 0x14C, 0x14E, 0x150, 0x152, 0x154, 0x156, 0x158, 0x15A, 0x15C, 0x15E, 0x160, 0x162, 0x164, 0x166, 0x168, 0x16A, 0x16C, 0x16E, 0x170, 0x172, 0x174, 0x176, 0x179, 0x17B, 0x17D, 0x191, 0x391, 0x392, 0x393, 0x394, 0x395, 0x396, 0x397, 0x398, 0x399, 0x39A, 0x39B, 0x39C, 0x39D, 0x39E, 0x39F, 0x3A0, 0x3A1, 0x3A3, 0x3A4, 0x3A5, 0x3A6, 0x3A7, 0x3A8, 0x3A9, 0x3AA, 0x410, 0x411, 0x412, 0x413, 0x414, 0x415, 0x416, 0x417, 0x418, 0x419, 0x41A, 0x41B, 0x41C, 0x41D, 0x41E, 0x41F, 0x420, 0x421, 0x422, 0x423, 0x424, 0x425, 0x426, 0x427, 0x428, 0x429, 0x42A, 0x42B, 0x42C, 0x42D, 0x42E, 0x42F, 0x401, 0x402, 0x403, 0x404, 0x405, 0x406, 0x407, 0x408, 0x409, 0x40A, 0x40B, 0x40C, 0x40E, 0x40F, 0x2160, 0x2161, 0x2162, 0x2163, 0x2164, 0x2165, 0x2166, 0x2167, 0x2168, 0x2169, 0x216A, 0x216B, 0x216C, 0x216D, 0x216E, 0x216F, 0xFF21, 0xFF22, 0xFF23, 0xFF24, 0xFF25, 0xFF26, 0xFF27, 0xFF28, 0xFF29, 0xFF2A, 0xFF2B, 0xFF2C, 0xFF2D, 0xFF2E, 0xFF2F, 0xFF30, 0xFF31, 0xFF32, 0xFF33, 0xFF34, 0xFF35, 0xFF36, 0xFF37, 0xFF38, 0xFF39, 0xFF3A, 0 };
	int i;


	for (i = 0; tbl_lower[i] && chr != tbl_lower[i]; i++) ;

	return tbl_lower[i] ? tbl_upper[i] : chr;
}


// Test for EEPROM file
uint32_t isEepromStart( uint8_t *p )
{
	uint32_t csum ;
	uint32_t size = 7 ;

	csum = 0 ;
	while( size )
	{
		csum += *p++ ;
		size -= 1 ;
	}
	if ( csum == *p )
	{
		return 1 ;
	}
	return 0 ;	
}


uint32_t isFirmwareStart( uint32_t *block )
{
	if ( ( block[0] & 0xFFFE3000 ) != 0x20000000 )
	{
		return 0 ;
	}
	if ( ( block[1] & 0xFFF80000 ) != 0x00400000 )
	{
		return 0 ;
	}
	if ( ( block[2] & 0xFFF80000 ) != 0x00400000 )
	{
		return 0 ;
	}
	return 1 ;	
}



uint32_t (*IAP_Function)(uint32_t, uint32_t) ;


uint32_t program( uint32_t *address, uint32_t *buffer )	// size is 256 bytes
{
	uint32_t FlashSectorNum ;
	uint32_t flash_cmd = 0 ;
	uint32_t i ;
//	uint32_t flash_status = 0;
	//	uint32_t EFCIndex = 0; // 0:EEFC0, 1: EEFC1
	/* Initialize the function pointer (retrieve function address from NMI vector) */

	if ( (uint32_t) address == 0x00408000 )
	{
		if ( isFirmwareStart( buffer) )
		{
			FlashBlocked = 0 ;
		}
		else
		{
			FlashBlocked = 1 ;
		}
	}

	if ( FlashBlocked )
	{
		return 1 ;
	}
	// Always initialise this here, setting a default doesn't seem to work
	IAP_Function = (uint32_t (*)(uint32_t, uint32_t))  *(( uint32_t *)0x00800008) ;
	FlashSectorNum = (uint32_t) address ;
	FlashSectorNum >>= 8 ;		// page size is 256 bytes
	FlashSectorNum &= 2047 ;	// max page number
	
	/* Send data to the sector here */
	for ( i = 0 ; i < 64 ; i += 1 )
	{
		*address++ = *buffer++ ;		
	}

	/* build the command to send to EEFC */
	flash_cmd = (0x5A << 24) | (FlashSectorNum << 8) | 0x03 ; //AT91C_MC_FCMD_EWP ;
	
	__disable_irq() ;
	/* Call the IAP function with appropriate command */
	i = IAP_Function( 0, flash_cmd ) ;
	__enable_irq() ;
	return i ;
}


uint32_t readLockBits()
{
	// Always initialise this here, setting a default doesn't seem to work
	IAP_Function = (uint32_t (*)(uint32_t, uint32_t))  *(( uint32_t *)0x00800008) ;
	
	uint32_t flash_cmd = (0x5A << 24) | 0x0A ; //AT91C_MC_FCMD_GLB ;
	__disable_irq() ;
	(void) IAP_Function( 0, flash_cmd ) ;
	__enable_irq() ;
	return EFC->EEFC_FRR ;
}


void clearLockBits()
{
	uint32_t i ;
	uint32_t flash_cmd = 0 ;

	// Always initialise this here, setting a default doesn't seem to work
	IAP_Function = (uint32_t (*)(uint32_t, uint32_t))  *(( uint32_t *)0x00800008) ;
	for ( i = 0 ; i < 16 ; i += 1 )
	{
		flash_cmd = (0x5A << 24) | ((128*i) << 8) | 0x09 ; //AT91C_MC_FCMD_CLB ;
		__disable_irq() ;
		/* Call the IAP function with appropriate command */
		(void) IAP_Function( 0, flash_cmd ) ;
		__enable_irq() ;
	} 
}


void interrupt10ms()
{
	Tenms |= 1 ;			// 10 mS has passed
 	per10ms() ;
}

void init10msTimer()
{
  register Tc *ptc ;
	register uint32_t timer ;

  PMC->PMC_PCER0 |= 0x02000000L ;		// Enable peripheral clock to TC2

	timer = Master_frequency / 12800  ;		// MCK/128 and 100 Hz

  ptc = TC0 ;		// Tc block 0 (TC0-2)
	ptc->TC_BCR = 0 ;			// No sync
	ptc->TC_BMR = 0 ;
	ptc->TC_CHANNEL[2].TC_CMR = 0x00008000 ;	// Waveform mode
	ptc->TC_CHANNEL[2].TC_RC = timer ;			// 10 Hz
	ptc->TC_CHANNEL[2].TC_RA = timer >> 1 ;
	ptc->TC_CHANNEL[2].TC_CMR = 0x0009C003 ;	// 0000 0000 0000 1001 1100 0000 0000 0011
																						// MCK/128, set @ RA, Clear @ RC waveform
	ptc->TC_CHANNEL[2].TC_CCR = 5 ;		// Enable clock and trigger it (may only need trigger)
	
	NVIC_EnableIRQ(TC2_IRQn) ;
	TC0->TC_CHANNEL[2].TC_IER = TC_IER0_CPCS ;
}

extern "C" void TC2_IRQHandler()
{
  register uint32_t dummy;

  /* Clear status bit to acknowledge interrupt */
  dummy = TC0->TC_CHANNEL[2].TC_SR;
	(void) dummy ;		// Discard value - prevents compiler warning

	interrupt10ms() ;
	
}

uint8_t *cpystr( uint8_t *dest, uint8_t *source )
{
  while ( (*dest++ = *source++) )
    ;
  return dest - 1 ;
}

		
FRESULT readBinDir( DIR *dj, FILINFO *fno )
{
	FRESULT fr ;
	uint32_t loop ;
	do
	{
		loop = 0 ;
		fr = f_readdir ( dj, fno ) ;		// First entry

		if ( fr != FR_OK || fno->fname[0] == 0 )
		{
			break ;
		}
		if ( *fno->lfname == 0 )
		{
			cpystr( (uint8_t *)fno->lfname, (uint8_t *)fno->fname ) ;		// Copy 8.3 name
		}
		int32_t len = strlen(fno->lfname) - 4 ;
		if ( len < 0 )
		{
			loop = 1 ;
		}
		if ( fno->lfname[len] != '.' )
		{
			loop = 1 ;
		}
		if ( ( fno->lfname[len+1] != 'b' ) && ( fno->lfname[len+1] != 'B' ) )
		{
			loop = 1 ;
		}
		if ( ( fno->lfname[len+2] != 'i' ) && ( fno->lfname[len+2] != 'I' ) )
		{
			loop = 1 ;
		}
		if ( ( fno->lfname[len+3] != 'n' ) && ( fno->lfname[len+3] != 'N' ) )
		{
			loop = 1 ;
		}

	} while ( loop ) ;
	return fr ;
}


uint32_t fillNames( uint32_t index )
{
	uint32_t i ;
	FRESULT fr ;
	Finfo.lfname = Filenames[0] ;
	Finfo.lfsize = 48 ;
	fr = f_readdir ( &Dj, 0 ) ;					// rewind
	fr = f_readdir ( &Dj, &Finfo ) ;		// Skip .
	fr = f_readdir ( &Dj, &Finfo ) ;		// Skip ..
	i = 0 ;
	while ( i <= index )
	{
		fr = readBinDir( &Dj, &Finfo ) ;		// First entry
		FileSize[0] = Finfo.fsize ;
		i += 1 ;
		if ( fr == FR_NO_FILE)
		{
			return 0 ;
		}
	}
	for ( i = 1 ; i < 7 ; i += 1 )
	{
		Finfo.lfname = Filenames[i] ;
		fr = readBinDir( &Dj, &Finfo ) ;		// First entry
//		if ( *Finfo.lfname == 0 )
//		{
//			cpystr( (uint8_t *)Finfo.lfname, (uint8_t *)Finfo.fname ) ;		// Copy 8.3 name
//		}
		FileSize[i] = Finfo.fsize ;
		if ( fr != FR_OK || Finfo.fname[0] == 0 )
		{
			break ;
		}
	}
	return i ;
}

FRESULT openFirmwareFile( uint32_t index )
{
	cpystr( cpystr( (uint8_t *)FlashFilename, (uint8_t *)"\\firmware\\" ), (uint8_t *)Filenames[index] ) ;
	f_open( &FlashFile, FlashFilename, FA_READ ) ;
	f_lseek ( &FlashFile, 32768 ) ;
	return f_read( &FlashFile, (BYTE *)Block_buffer, 4096, &BlockCount ) ;
}



uint8_t flashFile( uint32_t index )
{
	FRESULT fr ;
	
	lcd_clear() ;
	lcd_puts_Pleft( 0, "\005Flash File" ) ;
	if ( Valid == 0 )
	{
		// Validate file here
		// return 3 if invalid
		fr = openFirmwareFile( index ) ;
		fr = f_close( &FlashFile ) ;
		
		Valid = 1 ;
		if ( isFirmwareStart( Block_buffer ) == 0 )
		{
			Valid = 2 ;
		}
	}
	if ( Valid == 2 )
	{
	  lcd_puts_Pleft( 3*FH,"NOT A VALID FIRMWARE") ;
	  lcd_puts_Pleft( 6*FH,"\007[EXIT]") ;
		uint8_t event = getEvent() ;
		if ( event == EVT_KEY_LONG(KEY_EXIT) )
		{
			return 3 ;
		}
		return 4 ;		// 
	}
	lcd_putsn_P( 0, 2*FH, Filenames[index], 21 ) ;

  lcd_puts_Pleft( 5*FH,"\003YES\013NO") ;
  lcd_puts_Pleft( 6*FH,"\003[MENU]\013[EXIT]") ;

	
//lcd_outhex4( 20, 7*FH, Block_buffer[0] ) ;
//lcd_outhex4( 0, 7*FH, Block_buffer[0] >> 16 ) ;
//lcd_outhex4( 60, 7*FH, Block_buffer[1] ) ;
//lcd_outhex4( 40, 7*FH, Block_buffer[1] >> 16 ) ;

//lcd_outhex4( 80, 7*FH, FirmwareSize ) ;
//lcd_outhex4( 60, 7*FH, FirmwareSize >> 16 ) ;
				 
//	refreshDisplay() ;
	
	uint8_t event = getEvent() ;

	if ( event == EVT_KEY_LONG(KEY_MENU) )
	{
		fr = openFirmwareFile( index ) ;
		FirmwareSize = FileSize[index] ; 
		if ( fr != FR_OK )
		{
			return 4 ;		// File open error
		}
		return 2 ;
	}
	if ( event == EVT_KEY_LONG(KEY_EXIT) )
	{
		return 1 ;
	}
	return 0 ;
}


extern Key keys[] ;

static uint32_t PowerUpDelay ;

int main()
{
	uint32_t i ;
  uint8_t index = 0 ;
  uint8_t maxhsize = 21 ;
	FRESULT fr ;
	uint32_t state = 0 ;
	uint32_t nameCount = 0 ;
	uint32_t vpos = 0 ;
	uint32_t hpos = 0 ;
	uint32_t firmwareAddress = 0x00400000 ;
	uint32_t firmwareWritten = 0 ;

	MATRIX->CCFG_SYSIO |= 0x000000F0L ;		// Disable syspins, enable B4,5,6,7

	init_SDcard() ;
	PIOC->PIO_PER = PIO_PC25 ;		// Enable bit C25 (USB-detect)

	start_timer0() ;
	
	__enable_irq() ;
	init10msTimer() ;
	init_spi() ;

	uint32_t chip_id = CHIPID->CHIPID_CIDR ;

	FlashSize = ( (chip_id >> 8 ) & 0x000F ) == 9 ? 256 : 512 ; 

	LockBits = readLockBits() ;
	if ( LockBits )
	{
		clearLockBits() ;
	}

	for(;;)
	{
    usbMassStorage() ;

		if ( Tenms )
		{
	    wdt_reset() ;  // Retrigger hardware watchdog
			Tenms = 0 ;
			lcd_clear() ;
			lcd_puts_Pleft( 0, "Boot Loader" ) ;
			
			if ( sd_card_ready() )
			{
				lcd_puts_Pleft( 0, "\014Ready" ) ;
//				lcd_putc( 120, 0, clearCount + '0' ) ;
//				lcd_putc( 120, 8, cleared + '0' ) ;
//				lcd_outhex4( 0, 8, LockBits ) ;

				if ( PIOC->PIO_PDSR & 0x02000000 )
				{
					state = 10 ;
				}
		 
				if ( state == 10 )
				{
					lcd_puts_Pleft( 3*FH, "\010BUSY" ) ;
					if ( (PIOC->PIO_PDSR & 0x02000000 ) == 0 )
					{
						state = 0 ;
					}
					lcd_putc( 0, 6*FH, 'F' ) ;
					lcd_putc( 6, 6*FH, '0' + FlashBlocked ) ;
					lcd_putc( 0, 7*FH, 'E' ) ;
					lcd_putc( 6, 7*FH, '0' + EepromBlocked ) ;
				}


				if ( state == 0 )
				{
  				fr = f_mount(0, &g_FATFS) ;
				}
				else
				{
					fr = FR_OK ;
				}

				if ( fr == FR_OK)
				{
					if ( state == 0 )
					{
						state = 1 ;
					}
				}
				if ( state == 1 )
				{
					fr = f_chdir( (TCHAR *)"\\firmware" ) ;
					if ( fr == FR_OK )
					{
						state = 2 ;
						index = 0 ;
					}
				}
				if ( state == 1 )
				{
					lcd_puts_Pleft( 16, "\005No Firmware" ) ;
				}
				if ( state == 2 )
				{
					fr = f_opendir( &Dj, (TCHAR *) "." ) ;
					if ( fr == FR_OK )
					{
						state = 3 ;
						index = 0 ;
						nameCount = fillNames( 0 ) ;
						hpos = 0 ;
						vpos = 0 ;
					}
				}
				if ( state == 3 )
				{
					uint32_t limit = 6 ;
					if ( nameCount < limit )
					{
						limit = nameCount ;						
					}
					maxhsize = 0 ;
					for ( i = 0 ; i < limit ; i += 1 )
					{
						uint32_t x ;
						x = strlen( Filenames[i] ) ;
						if ( x > maxhsize )
						{
							maxhsize = x ;							
						}
						if ( x > 21 )
						{
							if ( ( hpos + 21 ) > x )
							{
								x = x - 21 ;
							}
							else
							{
								x = hpos ;
							}
						}
						else
						{
							x = 0 ;
						}
						lcd_putsn_P( 0, 16+FH*i, &Filenames[i][x], 21 ) ;
					}
					{
						uint8_t event = getEvent() ;

						if ( ( event == EVT_KEY_REPT(KEY_DOWN) ) || event == EVT_KEY_FIRST(KEY_DOWN) )
						{
							if ( vpos < limit-1 )
							{
								vpos += 1 ;
							}
							else
							{
								if ( nameCount > limit )
								{
									index += 1 ;
									nameCount = fillNames( index ) ;
								}
							}
						}
						if ( ( event == EVT_KEY_REPT(KEY_UP)) || ( event == EVT_KEY_FIRST(KEY_UP) ) )
						{
							if ( vpos > 0 )
							{
								vpos -= 1 ;
							}
							else
							{
								if ( index )
								{
									index -= 1 ;
									nameCount = fillNames( index ) ;
								}
							}
						}
						if ( ( event == EVT_KEY_REPT(KEY_RIGHT)) || ( event == EVT_KEY_FIRST(KEY_RIGHT) ) )
						{
							if ( hpos + 21 < maxhsize )
							{
								hpos += 1 ;								
							}
						}
						if ( ( event == EVT_KEY_REPT(KEY_LEFT)) || ( event == EVT_KEY_FIRST(KEY_LEFT) ) )
						{
							if ( hpos )
							{
								hpos -= 1 ;								
							}
						}
						if ( event == EVT_KEY_LONG(KEY_MENU) )
						{
							// Select file to flash
							state = 4 ;
							Valid = 0 ;
						}
					}
					lcd_char_inverse( 0, 2*FH+FH*vpos, 21*FW, 0 ) ;
				}
				if ( state == 4 )
				{
					i = flashFile( vpos ) ;
					FirmwareSize = FileSize[vpos] - 32768 ;
					if ( i == 1 )
					{
						state = 3 ;		// Canceled						
					}
					if ( i == 2 )
					{
						firmwareAddress = 0x00408000 ;
						firmwareWritten = 0 ;
						state = 5 ;		 // confirmed
					}
					if ( i == 3 )
					{
						// Invalid file
						state = 3 ;		// Canceled						
					}
				}
				if ( state == 5 )
				{
					// Commit to flashing
					uint32_t blockOffset = 0 ;
					lcd_puts_Pleft( 3*FH, "Flashing" ) ;
					while ( BlockCount )
					{
						program( (uint32_t *)firmwareAddress, &Block_buffer[blockOffset] ) ;	// size is 256 bytes
						blockOffset += 64 ;		// 32-bit words (256 bytes)
						firmwareAddress += 256 ;
						if ( BlockCount > 256 )
						{
							BlockCount -= 256 ;							
						}
						else
						{
							BlockCount = 0 ;
						}
					}
					firmwareWritten += 1 ;
					uint32_t width = FirmwareSize / 4096 ;
					lcd_hline( 0, 5*FH-1, width+1 ) ;
					lcd_hline( 0, 6*FH, width+1 ) ;
					lcd_vline( width, 5*FH, 8 ) ;
					for ( i = 0 ; i < firmwareWritten ; i += 1 )
					{
						lcd_vline( i, 5*FH, 8 ) ;
					}
					fr = f_read( &FlashFile, (BYTE *)Block_buffer, 4096, &BlockCount ) ;
					if ( BlockCount == 0 )
					{
						state = 6 ;
					}
					if ( firmwareWritten > FlashSize/4 - 9 )	// (127-8, or 63-8) 4K blocks
					{
						state = 6 ;				// Backstop
					}
				}
				if ( state == 6 )
				{
					lcd_puts_Pleft( 3*FH, "Flashing Complete" ) ;
				}
			}
//			lcd_putc( 120, 0, '0' + state ) ;

			refreshDisplay() ;
			if ( PowerUpDelay < 20 )	// 200 mS
			{
				PowerUpDelay += 1 ;
			}
			else
			{
				sd_poll_10mS() ;
			}
		}
	}
//	stop_timer0() ;

  return 0;
}



uint32_t eeprom_write_one( uint8_t byte, uint8_t count )
{
	register Spi *spiptr ;
	register uint32_t result ;
	
	spiptr = SPI ;
	spiptr->SPI_CR = 1 ;								// Enable
	(void) spiptr->SPI_RDR ;		// Dump any rx data
	
	spiptr->SPI_TDR = byte ;

	result = 0 ; 
	while( ( spiptr->SPI_SR & SPI_SR_RDRF ) == 0 )
	{
		// wait for received
		if ( ++result > 10000 )
		{
			break ;				
		}
	}
	if ( count == 0 )
	{
		spiptr->SPI_CR = 2 ;								// Disable
		return spiptr->SPI_RDR ;
	}
	(void) spiptr->SPI_RDR ;		// Dump the rx data
	spiptr->SPI_TDR = 0 ;
	result = 0 ; 
	while( ( spiptr->SPI_SR & SPI_SR_RDRF ) == 0 )
	{
		// wait for received
		if ( ++result > 10000 )
		{
			break ;				
		}
	}
	spiptr->SPI_CR = 2 ;								// Disable
	return spiptr->SPI_RDR ;
}


uint32_t eeprom_read_status()
{
	return eeprom_write_one( 5, 1 ) ;
}


void eeprom_write_enable()
{
	eeprom_write_one( 6, 0 ) ;
}

void eeprom_wait_busy()
{
	register uint32_t x ;
	register uint32_t y ;
	
	y = 0 ;
	do
	{
		y += 1 ;
		if ( y > 1000000 )
		{
			break ;			
		}
		x = eeprom_read_status() ;
	} while ( x & 1 ) ;
  
}

uint32_t spi_operation( register uint8_t *tx, register uint8_t *rx, register uint32_t count )
{
	register Spi *spiptr ;
	register uint32_t result ;

//  PMC->PMC_PCER0 |= 0x00200000L ;		// Enable peripheral clock to SPI

	result = 0 ; 
	spiptr = SPI ;
	spiptr->SPI_CR = 1 ;								// Enable
	(void) spiptr->SPI_RDR ;		// Dump any rx data
	while( count )
	{
		result = 0 ;
		while( ( spiptr->SPI_SR & SPI_SR_TXEMPTY ) == 0 )
		{
			// wait
			if ( ++result > 10000 )
			{
				result = 0xFFFF ;
				break ;				
			}
		}
		if ( result > 10000 )
		{
			break ;
		}
//		if ( count == 1 )
//		{
//			spiptr->SPI_CR = SPI_CR_LASTXFER ;		// LastXfer bit
//		}
		spiptr->SPI_TDR = *tx++ ;
		result = 0 ;
		while( ( spiptr->SPI_SR & SPI_SR_RDRF ) == 0 )
		{
			// wait for received
			if ( ++result > 10000 )
			{
				result = 0x2FFFF ;
				break ;				
			}
		}
		if ( result > 10000 )
		{
			break ;
		}
		*rx++ = spiptr->SPI_RDR ;
		count -= 1 ;
	}
	if ( result <= 10000 )
	{
		result = 0 ;
	}
	spiptr->SPI_CR = 2 ;								// Disable

// Power save
//  PMC->PMC_PCER0 &= ~0x00200000L ;		// Disable peripheral clock to SPI

	return result ;
}

uint32_t spi_PDC_action( register uint8_t *command, register uint8_t *tx, register uint8_t *rx, register uint32_t comlen, register uint32_t count )
{
	register Spi *spiptr ;
//	register uint32_t result ;
	register uint32_t condition ;
	static uint8_t discard_rx_command[4] ;

//  PMC->PMC_PCER0 |= 0x00200000L ;		// Enable peripheral clock to SPI

//	Spi_complete = 0 ;
	if ( comlen > 4 )
	{
//		Spi_complete = 1 ;
		return 0x4FFFF ;		
	}
	condition = SPI_SR_TXEMPTY ;
	spiptr = SPI ;
	spiptr->SPI_CR = 1 ;				// Enable
	(void) spiptr->SPI_RDR ;		// Dump any rx data
	(void) spiptr->SPI_SR ;			// Clear error flags
	spiptr->SPI_RPR = (uint32_t)discard_rx_command ;
	spiptr->SPI_RCR = comlen ;
	if ( rx )
	{
		spiptr->SPI_RNPR = (uint32_t)rx ;
		spiptr->SPI_RNCR = count ;
		condition = SPI_SR_RXBUFF ;
	}
	spiptr->SPI_TPR = (uint32_t)command ;
	spiptr->SPI_TCR = comlen ;
	if ( tx )
	{
		spiptr->SPI_TNPR = (uint32_t)tx ;
	}
	else
	{
		spiptr->SPI_TNPR = (uint32_t)rx ;
	}
	spiptr->SPI_TNCR = count ;

	spiptr->SPI_PTCR = SPI_PTCR_RXTEN | SPI_PTCR_TXTEN ;	// Start transfers

	// Wait for things to get started, avoids early interrupt
	for ( count = 0 ; count < 1000 ; count += 1 )
	{
		if ( ( spiptr->SPI_SR & SPI_SR_TXEMPTY ) == 0 )
		{
			break ;			
		}
	}
	
	count = 0 ;
	while( ( spiptr->SPI_SR & condition ) == 0 )
	{
		if ( ++count > 1000000 )
		{
			break ;			
		}
	}
	
	spiptr->SPI_CR = 2 ;				// Disable
	(void) spiptr->SPI_RDR ;		// Dump any rx data
	(void) spiptr->SPI_SR ;			// Clear error flags
	spiptr->SPI_PTCR = SPI_PTCR_RXTDIS | SPI_PTCR_TXTDIS ;	// Stop tramsfers

	if ( count > 1000000 )
	{
		return 1 ;
	}
	return 0 ;
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






void init_spi()
{
	register Pio *pioptr ;
	register Spi *spiptr ;
	register uint32_t timer ;
	register uint8_t *p ;
	uint8_t spi_buf[4] ;

	if ( !Spi_init_done)
	{
  	PMC->PMC_PCER0 |= 0x00200000L ;		// Enable peripheral clock to SPI
  	/* Configure PIO */
		pioptr = PIOA ;
  	pioptr->PIO_ABCDSR[0] &= ~0x00007800 ;	// Peripheral A bits 14,13,12,11
  	pioptr->PIO_ABCDSR[1] &= ~0x00007800 ;	// Peripheral A
  	pioptr->PIO_PDR = 0x00007800 ;					// Assign to peripheral
	
		spiptr = SPI ;
		timer = ( 64000000 / 3000000 ) << 8 ;
		spiptr->SPI_MR = 0x14000011 ;				// 0001 0100 0000 0000 0000 0000 0001 0001 Master
		spiptr->SPI_CSR[0] = 0x01180009 | timer ;		// 0000 0001 0001 1000 xxxx xxxx 0000 1001
	//	NVIC_EnableIRQ(SPI_IRQn) ;

		p = spi_buf ;
		
	//	*p = 0x39 ;		// Unprotect sector command
	//	*(p+1) = 0 ;
	//	*(p+2) = 0 ;
	//	*(p+3) = 0 ;		// 3 bytes address

	//	spi_operation( p, spi_buf, 4 ) ;
	
		eeprom_write_enable() ;

		*p = 1 ;		// Write status register command
		*(p+1) = 0 ;
		spi_operation( p, spi_buf, 2 ) ;
		Spi_init_done = 1 ;
	}

}


void AT25D_Read( uint8_t *BufferAddr, uint32_t size, uint32_t memoryOffset)
{
	register uint8_t *p ;
	
	p = Spi_tx_buf ;
	*p = 3 ;		// Read command
	*(p+1) = memoryOffset >> 16 ;
	*(p+2) = memoryOffset >> 8 ;
	*(p+3) = memoryOffset ;		// 3 bytes address
	
	spi_PDC_action( p, 0, BufferAddr, 4, size ) ;
}


uint32_t ee32_read_512( uint32_t sector, uint8_t *buffer )
{
	AT25D_Read( buffer, 512, sector * 512 ) ;
	return 1 ;
}

void AT25D_Write( uint8_t *BufferAddr, uint32_t size, uint32_t memoryOffset )
{
	register uint8_t *p ;
	
	eeprom_write_enable() ;
	
	p = Spi_tx_buf ;
	*p = 2 ;		// Write command
	*(p+1) = memoryOffset >> 16 ;
	*(p+2) = memoryOffset >> 8 ;
	*(p+3) = memoryOffset ;		// 3 bytes address
		 
	spi_PDC_action( p, BufferAddr, 0, 4, size ) ;

	eeprom_wait_busy() ;

}

uint32_t eeprom_block_erased( register uint8_t *p)
{
	register uint32_t x ;
	register uint32_t result ;

	result = 1 ;

	for ( x = 0 ; x < 4096 ; x += 1 )
	{
		if ( *p++ != 0xFF )
		{
			result = 0 ;			
			break ;
		}		
	}
	return result ;
}

uint32_t eeprom_page_erased( register uint8_t *p)
{
	register uint32_t x ;
	register uint32_t result ;

	result = 1 ;

	for ( x = 0 ; x < 256 ; x += 1 )
	{
		if ( *p++ != 0xFF )
		{
			result = 0 ;			
			break ;
		}		
	}
	return result ;
}

			 
uint32_t ee32_write( const uint8_t *buffer, uint32_t sector, uint32_t count )
{
	
	// EEPROM write
	uint32_t startMemoryOffset ;
	uint32_t memoryOffset ;
	uint32_t bytesToWrite ;
  uint8_t *pBuffer ;
	uint32_t block_address ;
	uint32_t x ;

	if ( sector == 0 )
	{
		if ( isEepromStart( (uint8_t *) buffer ) )
		{
			EepromBlocked = 0 ;
		}
		else
		{
			EepromBlocked = 1 ;
		}
	}

	if ( EepromBlocked )
	{
		return 1 ;
	}

	startMemoryOffset = sector ;
	startMemoryOffset *= 512 ;		// Byte address into EEPROM
  memoryOffset      = startMemoryOffset;

	bytesToWrite = count * 512 ;
				
	unprotect_eeprom() ;

  pBuffer = (uint8_t *) buffer ;

	block_address = memoryOffset &0xFFFFF000 ;		// 4k boundary
  AT25D_Read( Eblock_buffer, 4096, block_address ) ;	// read block to write to
	// Check to see if it is blank
	x = eeprom_block_erased( Eblock_buffer ) ;
	if ( x == 0 )
	{
		AT25D_EraseBlock( block_address ) ;
	}

	while (bytesToWrite)
	{
		uint32_t bytes_to_copy ;
		uint32_t i ;
		uint32_t j ;
		uint8_t *s ;
		uint8_t *dest ;

		dest = Eblock_buffer + (memoryOffset & 0x0FFF ) ;
		s = pBuffer ;
		bytes_to_copy = 4096 - ( memoryOffset - block_address ) ;
		if ( bytes_to_copy > bytesToWrite )
		{
			bytes_to_copy = bytesToWrite ;
		}
		for ( i = 0 ; i < bytes_to_copy ; i += 1 )
		{
			*dest++ = *s++ ;						
		}
		memoryOffset += bytes_to_copy ;
		bytesToWrite -= bytes_to_copy ;
		s = Eblock_buffer ;
		j = 0 ;
		for ( i = 0 ; i < 16 ; i += 1 )		// pages in block
		{
			j <<= 1 ;
			x = eeprom_page_erased( s ) ;
			if ( x == 0 )				// Not blank
			{
        AT25D_Write( s, 256, block_address ) ;
				j |= 1 ;
			}						
			s += 256 ;
			block_address += 256 ;
		}
	}
	
	return 1 ;
}

uint32_t AT25D_EraseBlock( uint32_t memoryOffset )
{
	register uint8_t *p ;
	register uint32_t x ;
	
	eeprom_write_enable() ;
	p = Spi_tx_buf ;
	*p = 0x20 ;		// Block Erase command
	*(p+1) = memoryOffset >> 16 ;
	*(p+2) = memoryOffset >> 8 ;
	*(p+3) = memoryOffset ;		// 3 bytes address
	x = spi_operation( p, Spi_rx_buf, 4 ) ;

	eeprom_wait_busy() ;
	return x ;
}


