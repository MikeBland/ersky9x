

#include <stdint.h>
//#include "lcd.h"
#include "AT91SAM3S4.h"

__attribute__ ((section(".bootrodata"), used))

const uint8_t BootCode[] = {
#ifdef REVX
#include "bootflash8.lbm"
#else
#include "bootflash4.lbm"
#endif
} ;

__attribute__ ((section(".text"), used))

extern void start_timer0( void ) ;
//extern void stop_timer0( void ) ;
//extern void init_SDcard( void ) ;

//extern "C" void loadAndRunBoot( void ) ;

//void loadAndRunBoot()
//{
//	const uint8_t *src ;
//	uint8_t *dest ;
//	uint32_t size ;

//	size = sizeof(BootCode) ;
//	src = BootCode ;
//	dest = (uint8_t *)0x20000000 ;

//	MATRIX->CCFG_SYSIO |= 0x000000F0L ;		// Disable syspins, enable B4,5,6,7

////	init_SDcard() ;

//	for ( ; size ; size -= 1 )
//	{
//		*dest++ = *src++ ;		
//	}	

//	size = sizeof(BootCode) ;
//	src = BootCode ;
//	dest = (uint8_t *)0x20000000 ;
	 
//	for ( ; size ; size -= 1 )
//	{
//		if ( *dest++ != *src++ )
//		{
//			for(;;)
//			{
//				start_timer0() ;
//				lcd_clear() ;
//				lcd_outhex4( 0, 0, (uint32_t)dest-1 ) ;
//				lcd_puts_Pleft( 24, "Boot Fail" ) ;
//				refreshDisplay() ;
//				stop_timer0() ;
//			}
//		}
//	}	

//	uint32_t address = *(uint32_t *)0x20000004 ;
	
//	((void (*)(void)) (address))() ;		// Go execute the loaded application
		 
//}


