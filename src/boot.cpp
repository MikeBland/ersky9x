/****************************************************************************
*  Copyright (c) 2014 by Michael Blandford. All rights reserved.
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
* Other Authors:
 * - Andre Bernet
 * - Bertrand Songis
 * - Bryan J. Rentoul (Gruvin)
 * - Cameron Weeks
 * - Erez Raviv
 * - Jean-Pierre Parisy
 * - Karl Szmutny
 * - Michal Hlavinka
 * - Pat Mackenzie
 * - Philip Moss
 * - Rob Thomson
 * - Romolo Manfredini
 * - Thomas Husterer
*
****************************************************************************/


/*----------------------------------------------------------------------------
 *        Headers
 *----------------------------------------------------------------------------*/
 
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#ifdef PCBSKY
#include "AT91SAM3S4.h"
#include "core_cm3.h"
#endif

#ifdef PCBX9D
#include "x9d\stm32f2xx.h"
//#include "x9d\stm32f2xx_gpio.h"
//#include "x9d\stm32f2xx_rcc.h"
//#include "x9d\stm32f2xx_usart.h"
#include "x9d\stm32f2xx_flash.h"
#include "X9D\i2c_ee.h"
#include "x9d\hal.h"

extern "C" {
#include "x9d\usb_dcd_int.h"
#include "x9d\usb_bsp.h"
#include "x9d\usbd_desc.h"
#include "x9d\usbd_msc_core.h"
#include "x9d\usbd_usr.h"
//#include "stm32f2xx_dbgmcu.h"
}

#endif

#include "lcd.h"
#include "ersky9x.h"
#include "lcd.h"
#include "ff.h"
#include "diskio.h"
#include "drivers.h"
#include "myeeprom.h"
#include "timers.h"
#include "logicio.h"

#ifdef PCBSKY
extern void usbMassStorage( void ) ;
#endif

#ifdef PCBSKY
#define BOOT_KEY_UP			KEY_UP
#define BOOT_KEY_DOWN		KEY_DOWN
#define BOOT_KEY_LEFT		KEY_LEFT
#define BOOT_KEY_RIGHT	KEY_RIGHT
#define BOOT_KEY_MENU		KEY_MENU
#define BOOT_KEY_EXIT		KEY_EXIT
#endif

#ifdef PCBX9D
#define BOOT_KEY_UP			KEY_MENU
#define BOOT_KEY_DOWN		KEY_RIGHT
#define BOOT_KEY_LEFT		KEY_LEFT
#define BOOT_KEY_RIGHT	KEY_EXIT
#define BOOT_KEY_MENU		KEY_UP
#define BOOT_KEY_EXIT		KEY_DOWN
#endif

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

#ifdef PCBSKY
EEGeneral g_eeGeneral ;
#endif

uint32_t FirmwareSize ;

uint32_t Master_frequency ;
volatile uint8_t  Tenms ;
uint8_t EE_timer ;

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


uint32_t FlashBlocked = 1 ;
uint32_t LockBits ;

//uint32_t EeWriteCount ;
//uint32_t EeEraseCount ;

uint32_t Block_buffer[1024] ;
UINT BlockCount ;

#ifdef PCBSKY
extern int32_t EblockAddress ;
#endif
extern uint32_t EepromBlocked ;

extern void init_spi( void ) ;
extern void writeBlock( void ) ;


/*----------------------------------------------------------------------------
 *         Global functions
 *----------------------------------------------------------------------------*/

#ifdef PCBSKY
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
#endif

static bool usbPlugged(void)
{
#ifdef PCBSKY
	return PIOC->PIO_PDSR & 0x02000000 ;
#endif
	
#ifdef PCBX9D
	return GPIOA->IDR & 0x0200 ;
#endif
}

#ifdef PCBX9D

extern "C" {
USB_OTG_CORE_HANDLE USB_OTG_dev;

void OTG_FS_IRQHandler(void)
{
  USBD_OTG_ISR_Handler (&USB_OTG_dev);
}
}

static void usbInit()
{
  USB_OTG_BSP_Init(&USB_OTG_dev);
}

static void usbStart()
{
  USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_desc, &USBD_MSC_cb, &USR_cb);
}

uint32_t isFirmwareStart( uint32_t *block )
{
	if ( ( block[0] & 0xFFFC0000 ) != 0x20000000 )
	{
		return 0 ;
	}
	if ( ( block[1] & 0xFFF00000 ) != 0x08000000 )
	{
		return 0 ;
	}
	if ( ( block[2] & 0xFFF00000 ) != 0x08000000 )
	{
		return 0 ;
	}
	return 1 ;	
}
#endif

#ifdef PCBSKY
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
#endif


#ifdef PCBSKY

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
#endif

void interrupt10ms()
{
	Tenms |= 1 ;			// 10 mS has passed
 	per10ms() ;
}

#ifdef PCBSKY
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
#endif

#ifdef PCBX9D
void init10msTimer()
{
	// Timer14
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN ;		// Enable clock
	TIM14->ARR = 9999 ;	// 10mS
	TIM14->PSC = (Peri1_frequency*Timer_mult1) / 1000000 - 1 ;		// 1uS from 12MHz
	TIM14->CCER = 0 ;	
	TIM14->CCMR1 = 0 ;
	TIM14->EGR = 0 ;
	TIM14->CR1 = 5 ;
	TIM14->DIER |= 1 ;
	NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn) ;
}

extern "C" void TIM8_TRG_COM_TIM14_IRQHandler()
{
	TIM14->SR &= ~TIM_SR_UIF ;
	interrupt10ms() ;
}

void init_hw_timer()
{
	// Timer13
	RCC->APB1ENR |= RCC_APB1ENR_TIM13EN ;		// Enable clock
	TIM13->ARR = 65535 ;
	TIM13->PSC = (Peri1_frequency*Timer_mult1) / 10000000 - 1 ;		// 0.1uS from 12MHz
	TIM13->CCER = 0 ;	
	TIM13->CCMR1 = 0 ;
	TIM13->EGR = 0 ;
	TIM13->CR1 = 1 ;
}


// delay in units of 0.1 uS up to 6.5535 mS
void hw_delay( uint16_t time )
{
	TIM13->CNT = 0 ;
	TIM13->EGR = 1 ;		// Re-start counter
	while ( TIM13->CNT < time )
	{
		// wait
	}
}

//After reset, write is not allowed in the Flash control register (FLASH_CR) to protect the
//Flash memory against possible unwanted operations due, for example, to electric
//disturbances. The following sequence is used to unlock this register:
//1. Write KEY1 = 0x45670123 in the Flash key register (FLASH_KEYR)
//2. Write KEY2 = 0xCDEF89AB in the Flash key register (FLASH_KEYR)
//Any wrong sequence will return a bus error and lock up the FLASH_CR register until the
//next reset.
//The FLASH_CR register can be locked again by software by setting the LOCK bit in the
//FLASH_CR register.
void unlockFlash()
{
	FLASH->KEYR = 0x45670123 ;
	FLASH->KEYR = 0xCDEF89AB ;
}

void waitFlashIdle()
{
	while (FLASH->SR & FLASH_FLAG_BSY)
	{
	 	wdt_reset() ;
	}
}

#define SECTOR_MASK               ((uint32_t)0xFFFFFF07)

void eraseSector( uint32_t sector )
{
	waitFlashIdle() ;

  FLASH->CR &= CR_PSIZE_MASK;
  FLASH->CR |= FLASH_PSIZE_WORD ;
  FLASH->CR &= SECTOR_MASK;
  FLASH->CR |= FLASH_CR_SER | (sector<<3) ;
  FLASH->CR |= FLASH_CR_STRT;
    
  /* Wait for operation to be completed */
	waitFlashIdle() ;
    
  /* if the erase operation is completed, disable the SER Bit */
  FLASH->CR &= (~FLASH_CR_SER);
  FLASH->CR &= SECTOR_MASK; 
}

uint32_t program( uint32_t *address, uint32_t *buffer )	// size is 256 bytes
{
	uint32_t i ;
//	uint32_t flash_status = 0;
	//	uint32_t EFCIndex = 0; // 0:EEFC0, 1: EEFC1
	/* Initialize the function pointer (retrieve function address from NMI vector) */

	if ( (uint32_t) address == 0x08008000 )
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

	if ( (uint32_t) address == 0x08008000 )
	{
		eraseSector( 2 ) ;
	}
	if ( (uint32_t) address == 0x0800C000 )
	{
		eraseSector( 3 ) ;
	}
	if ( (uint32_t) address == 0x08010000 )
	{
		eraseSector( 4 ) ;
	}
	if ( (uint32_t) address == 0x08020000 )
	{
		eraseSector( 5 ) ;
	}
	if ( (uint32_t) address == 0x08040000 )
	{
		eraseSector( 6 ) ;
	}
	if ( (uint32_t) address == 0x08060000 )
	{
		eraseSector( 7 ) ;
	}

	// Now program the 256 bytes
	 
//	i = IAP_Function( 0, flash_cmd ) ;

  for (i = 0 ; i < 64 ; i += 1 )
  {
    /* Device voltage range supposed to be [2.7V to 3.6V], the operation will
       be done by word */ 
    
	  // Wait for last operation to be completed
		waitFlashIdle() ;
  
    FLASH->CR &= CR_PSIZE_MASK;
    FLASH->CR |= FLASH_PSIZE_WORD;
    FLASH->CR |= FLASH_CR_PG;
  
    *address = *buffer ;
        
    /* Wait for operation to be completed */
		waitFlashIdle() ;
    FLASH->CR &= (~FLASH_CR_PG);
		 
		 /* Check the written value */
    if ( *address != *buffer )
    {
      /* Flash content doesn't match SRAM content */
      return 2 ;
    }
    /* Increment FLASH destination address */
    address += 1 ;
		buffer += 1 ;
  }
  return 0 ;
}

#endif


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
		if ( event == EVT_KEY_LONG(BOOT_KEY_EXIT) )
		{
			return 3 ;
		}
		return 4 ;		// 
	}
	lcd_putsn_P( 0, 2*FH, Filenames[index], 21 ) ;

  lcd_puts_Pleft( 5*FH,"\003YES\013NO") ;
  lcd_puts_Pleft( 6*FH,"\003[MENU]\013[EXIT]") ;

	
	uint8_t event = getEvent() ;

	if ( event == EVT_KEY_LONG(BOOT_KEY_MENU) )
	{
		fr = openFirmwareFile( index ) ;
		FirmwareSize = FileSize[index] ; 
		if ( fr != FR_OK )
		{
			return 4 ;		// File open error
		}
		return 2 ;
	}
	if ( event == EVT_KEY_LONG(BOOT_KEY_EXIT) )
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
#ifdef PCBX9D
  uint8_t TenCount = 2 ;
#endif			
  uint8_t maxhsize = 21 ;
	FRESULT fr ;
	uint32_t state = 0 ;
	uint32_t nameCount = 0 ;
	uint32_t vpos = 0 ;
	uint32_t hpos = 0 ;
	uint32_t firmwareAddress = 0x00400000 ;
	uint32_t firmwareWritten = 0 ;

#ifdef PCBX9D
	lcd_init() ;
	wdt_reset() ;
	lcd_clear() ;
	lcd_puts_Pleft( 0, "Boot Loader" ) ;
	refreshDisplay() ;
	wdt_reset() ;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN ; 		// Enable portA clock
#endif

#ifdef PCBSKY
	MATRIX->CCFG_SYSIO |= 0x000000F0L ;		// Disable syspins, enable B4,5,6,7
#endif

#ifdef PCBSKY
	init_SDcard() ;
	PIOC->PIO_PER = PIO_PC25 ;		// Enable bit C25 (USB-detect)
	start_timer0() ;
#endif

#ifdef PCBX9D	 
	init_keys() ;
//	init_trims() ;
	I2C_EE_Init() ;
	init_hw_timer()	;
#endif

	__enable_irq() ;
	init10msTimer() ;

#ifdef PCBSKY
	EblockAddress = -1 ;
	init_spi() ;
#endif

#ifdef PCBSKY
	uint32_t chip_id = CHIPID->CHIPID_CIDR ;

	FlashSize = ( (chip_id >> 8 ) & 0x000F ) == 9 ? 256 : 512 ; 
#endif

#ifdef PCBX9D
	FlashSize = 512 ;
#endif

#ifdef PCBSKY
	LockBits = readLockBits() ;
	if ( LockBits )
	{
		clearLockBits() ;
	}
#endif

#ifdef PCBX9D
	// SD card detect pin
	configure_pins( GPIO_Pin_CP, PIN_PORTD | PIN_INPUT | PIN_PULLUP ) ;
//	lcd_puts_Pleft( 24, "0" ) ;
//	refreshDisplay() ;
//	wdt_reset() ;
	disk_initialize( 0 ) ;
//	lcd_puts_Pleft( 32, "1" ) ;
//	refreshDisplay() ;
//	wdt_reset() ;
	sdInit() ;
//	lcd_puts_Pleft( 40, "2" ) ;
//	refreshDisplay() ;
//	wdt_reset() ;
	unlockFlash() ;

  usbInit() ;
  usbStart() ;
#endif

	for(;;)
	{
#ifdef PCBSKY
    usbMassStorage() ;
#endif
		
		wdt_reset() ;

		if ( Tenms )
		{
	    wdt_reset() ;  // Retrigger hardware watchdog

			if ( EE_timer )
			{
				if ( --EE_timer  == 0)
				{
#ifdef PCBSKY
					writeBlock() ;
#endif
				}
			}

			Tenms = 0 ;
			lcd_clear() ;
			lcd_puts_Pleft( 0, "Boot Loader" ) ;

//static uint32_t xc = 0 ;
//			lcd_outhex4( 0, 8, ++xc ) ;
//			lcd_outhex4( 0, 16, GPIOA->IDR ) ;
			
			if ( sd_card_ready() )
			{
				lcd_puts_Pleft( 0, "\014Ready" ) ;

//	lcd_outdez( 123, 0, EeWriteCount ) ;
//	lcd_outdez( 123, 8, EeEraseCount ) ;

//				lcd_putc( 120, 0, clearCount + '0' ) ;
//				lcd_putc( 120, 8, cleared + '0' ) ;
//				lcd_outhex4( 0, 8, LockBits ) ;

#define PIN_FS_VBUS                     GPIO_Pin_9  //PA.09

				if ( usbPlugged() )
				{
					state = 10 ;
				}
				
				if ( state == 10 )
				{
					lcd_puts_Pleft( 3*FH, "\010BUSY" ) ;
					if ( usbPlugged() == 0 )
					{
						state = 0 ;
					}
#ifdef PCBSKY
					lcd_putc( 0, 6*FH, 'F' ) ;
					lcd_putc( 6, 6*FH, '0' + FlashBlocked ) ;
					lcd_putc( 0, 7*FH, 'E' ) ;
					lcd_putc( 6, 7*FH, '0' + EepromBlocked ) ;
#endif
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

						if ( ( event == EVT_KEY_REPT(BOOT_KEY_DOWN) ) || event == EVT_KEY_FIRST(BOOT_KEY_DOWN) )
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
						if ( ( event == EVT_KEY_REPT(BOOT_KEY_UP)) || ( event == EVT_KEY_FIRST(BOOT_KEY_UP) ) )
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
						if ( ( event == EVT_KEY_REPT(BOOT_KEY_RIGHT)) || ( event == EVT_KEY_FIRST(BOOT_KEY_RIGHT) ) )
						{
							if ( hpos + 21 < maxhsize )
							{
								hpos += 1 ;								
							}
						}
						if ( ( event == EVT_KEY_REPT(BOOT_KEY_LEFT)) || ( event == EVT_KEY_FIRST(BOOT_KEY_LEFT) ) )
						{
							if ( hpos )
							{
								hpos -= 1 ;								
							}
						}
						if ( event == EVT_KEY_LONG(BOOT_KEY_MENU) )
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
#ifdef PCBSKY
						firmwareAddress = 0x00408000 ;
#endif
#ifdef PCBX9D
						firmwareAddress = 0x08008000 ;
#endif
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
					uint8_t event = getEvent() ;
					lcd_puts_Pleft( 3*FH, "Flashing Complete" ) ;
					if ( event == EVT_KEY_LONG(BOOT_KEY_EXIT) )
					{
						state = 3 ;
					}
				}
			}
//			lcd_putc( 120, 0, '0' + state ) ;

#ifdef PCBX9D
			if ( --TenCount == 0 )
			{
				TenCount = 2 ;
#endif			
			refreshDisplay() ;
#ifdef PCBX9D
			}
#endif			
			if ( PowerUpDelay < 20 )	// 200 mS
			{
				PowerUpDelay += 1 ;
			}
			else
			{
#ifdef PCBSKY
				sd_poll_10mS() ;
#endif			
#ifdef PCBX9D
				sdPoll10ms() ;
#endif			
			}
		}
	}
//	stop_timer0() ;

  return 0;
}

