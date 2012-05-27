/*
 * Author - Erez Raviv <erezraviv@gmail.com>
 *
 * Based on th9x -> http://code.google.com/p/th9x/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */



#include <stdint.h>
#include <stdlib.h>
#include "AT91SAM3S4.h"
#include "core_cm3.h"

#include "ersky9x.h"
#include "myeeprom.h"
#include "drivers.h"
#include "debug.h"

// Timer usage
// TIMER3 for input capture
// Timer4 to provide 0.5uS clock for input capture
// TIMER0 at full speed (MCK/2) for delay timing
// TIMER2 at 200Hz, provides 5mS for sound and 10mS tick on interrupt
// Timer1 used for DAC output timing
// Timer5 is currently UNUSED

uint16_t Analog_values[NUMBER_ANALOG] ;
uint16_t Temperature ;				// Raw temp reading
uint16_t Max_temperature ;		// Max raw temp reading

//struct t_fifo32
//{
//	uint8_t fifo[32] ;
//	uint32_t in ;
//	uint32_t out ;
//	volatile uint32_t count ;
//} ;


#define RX_UART_BUFFER_SIZE	32

struct t_rxUartBuffer
{
	uint8_t fifo[RX_UART_BUFFER_SIZE] ;
	uint8_t *outPtr ;
} ;

struct t_rxUartBuffer TelemetryInBuffer[2] ;
uint32_t TelemetryActiveBuffer ;



volatile uint32_t Spi_complete ;

void putEvent( register uint8_t evt) ;
uint32_t read_keys( void ) ;
uint32_t read_trims( void ) ;
void UART_Configure( uint32_t baudrate, uint32_t masterClock) ;
void txmit( uint8_t c ) ;
void uputs( register char *string ) ;
uint16_t rxuart( void ) ;
void UART3_Configure( uint32_t baudrate, uint32_t masterClock) ;
void txmitBt( uint8_t c ) ;
uint16_t rxBtuart( void ) ;

uint32_t keyState( enum EnumKeys enuk) ;
void per10ms( void ) ;
uint8_t getEvent( void ) ;
void pauseEvents(uint8_t event) ;
void killEvents(uint8_t event) ;
void init_spi( void ) ;
void end_spi( void ) ;
uint32_t eeprom_read_status( void ) ;
uint32_t  eeprom_write_one( uint8_t byte, uint8_t count ) ;
void eeprom_write_enable( void ) ;
uint32_t spi_operation( uint8_t *tx, uint8_t *rx, uint32_t count ) ;
//uint32_t spi_action( uint8_t *command, uint8_t *tx, uint8_t *rx, uint32_t comlen, uint32_t count ) ;
uint32_t spi_PDC_action( uint8_t *command, uint8_t *tx, uint8_t *rx, uint32_t comlen, uint32_t count ) ;
void crlf( void ) ;
void p8hex( uint32_t value ) ;
void p4hex( uint16_t value ) ;
void p2hex( unsigned char c ) ;
void hex_digit_send( unsigned char c ) ;
void read_9_adc(void ) ;
void init_adc( void ) ;
void init_ssc( void ) ;
void disable_ssc( void ) ;

/** Console baudrate 9600. */
#define CONSOLE_BAUDRATE    9600
/** Usart Hw interface used by the console (UART0). */
#define CONSOLE_USART       UART0
/** Usart Hw ID used by the console (UART0). */
#define CONSOLE_ID          ID_UART0
/** Pins description corresponding to Rxd,Txd, (UART pins) */
#define CONSOLE_PINS        {PINS_UART}

/** Second serial baudrate 9600. */
#define SECOND_BAUDRATE    9600
/** Usart Hw interface used by the console (UART0). */
#define SECOND_USART       USART0
/** Usart Hw ID used by the console (UART0). */
#define SECOND_ID          ID_USART0
/** Pins description corresponding to Rxd,Txd, (UART pins) */
#define SECOND_PINS        {PINS_USART0}

#define BT_USART       UART1
#define BT_ID          ID_UART1


static uint8_t s_evt;
void putEvent( register uint8_t evt)
{
  s_evt = evt;
}


uint8_t getEvent()
{
  register uint8_t evt = s_evt;
  s_evt=0;
  return evt;
}


class Key
{
#define FILTERBITS      4
#define FFVAL          ((1<<FILTERBITS)-1)
#define KSTATE_OFF      0
#define KSTATE_RPTDELAY 95 // gruvin: longer dely before key repeating starts
  //#define KSTATE_SHORT   96
#define KSTATE_START   97
#define KSTATE_PAUSE   98
#define KSTATE_KILLED  99
  uint8_t m_vals:FILTERBITS;   // key debounce?  4 = 40ms
  uint8_t m_dblcnt:2;
  uint8_t m_cnt;
  uint8_t m_state;
public:
  void input(bool val, EnumKeys enuk);
  bool state()       { return m_vals==FFVAL;                }
  void pauseEvents() { m_state = KSTATE_PAUSE;  m_cnt   = 0;}
  void killEvents()  { m_state = KSTATE_KILLED; m_dblcnt=0; }
  uint8_t getDbl()   { return m_dblcnt;                     }
};

Key keys[NUM_KEYS] ;

void Key::input(bool val, EnumKeys enuk)
{
  //  uint8_t old=m_vals;
  m_vals <<= 1;  if(val) m_vals |= 1; //portbit einschieben
  m_cnt++;

  if(m_state && m_vals==0){  //gerade eben sprung auf 0
    if(m_state!=KSTATE_KILLED) {
      putEvent(EVT_KEY_BREAK(enuk));
      if(!( m_state == 16 && m_cnt<16)){
        m_dblcnt=0;
      }
        //      }
    }
    m_cnt   = 0;
    m_state = KSTATE_OFF;
  }
  switch(m_state){
    case KSTATE_OFF:
      if(m_vals==FFVAL){ //gerade eben sprung auf ff
        m_state = KSTATE_START;
        if(m_cnt>16) m_dblcnt=0; //pause zu lang fuer double
        m_cnt   = 0;
      }
      break;
      //fallthrough
    case KSTATE_START:
      putEvent(EVT_KEY_FIRST(enuk));
      m_dblcnt++;
#ifdef KSTATE_RPTDELAY
      m_state   = KSTATE_RPTDELAY;
#else
      m_state   = 16;
#endif
      m_cnt     = 0;
      break;
#ifdef KSTATE_RPTDELAY
    case KSTATE_RPTDELAY: // gruvin: longer delay before first key repeat
      if(m_cnt == 32) putEvent(EVT_KEY_LONG(enuk)); // need to catch this inside RPTDELAY time
      if (m_cnt == 40) {
        m_state = 16;
        m_cnt = 0;
      }
      break;
#endif
    case 16:
#ifndef KSTATE_RPTDELAY
      if(m_cnt == 32) putEvent(EVT_KEY_LONG(enuk));
      //fallthrough
#endif
    case 8:
    case 4:
    case 2:
      if(m_cnt >= 48)  { //3 6 12 24 48 pulses in every 480ms
        m_state >>= 1;
        m_cnt     = 0;
      }
      //fallthrough
    case 1:
      if( (m_cnt & (m_state-1)) == 0)  putEvent(EVT_KEY_REPT(enuk));
      break;

    case KSTATE_PAUSE: //pause
      if(m_cnt >= 64)      {
        m_state = 8;
        m_cnt   = 0;
      }
      break;

    case KSTATE_KILLED: //killed
      break;
  }
}

extern uint32_t keyState(EnumKeys enuk)
{
	register uint32_t a ;
	register uint32_t c ;

  CPU_UINT xxx = 0 ;
  if(enuk < (int)DIM(keys))  return keys[enuk].state() ? 1 : 0;

	a = PIOA->PIO_PDSR ;
	c = PIOC->PIO_PDSR ;
	switch((uint8_t)enuk)
	{
#ifdef REVB
    case SW_ElevDR : xxx = c & 0x80000000 ;	// ELE_DR   PC31
#else 
    case SW_ElevDR : xxx = a & 0x00000100 ;	// ELE_DR   PA8
#endif 
    break ;
    
    case SW_AileDR : xxx = a & 0x00000004 ;	// AIL-DR  PA2
    break ;

    case SW_RuddDR : xxx = a & 0x00008000 ;	// RUN_DR   PA15
    break ;
      //     INP_G_ID1 INP_E_ID2
      // id0    0        1
      // id1    1        1
      // id2    1        0
    case SW_ID0    : xxx = ~c & 0x00004000 ;	// SW_IDL1     PC14
    break ;
    case SW_ID1    : xxx = (c & 0x00004000) ; if ( xxx ) xxx = (PIOC->PIO_PDSR & 0x00000800);
    break ;
    case SW_ID2    : xxx = ~c & 0x00000800 ;	// SW_IDL2     PC11
    break ;

    
		case SW_Gear   : xxx = c & 0x00010000 ;	// SW_GEAR     PC16
    break ;

#ifdef REVB
    case SW_ThrCt  : xxx = c & 0x00100000 ;	// SW_TCUT     PC20
#else 
    case SW_ThrCt  : xxx = a & 0x10000000 ;	// SW_TCUT     PA28
#endif 
    break ;

    case SW_Trainer: xxx = c & 0x00000100 ;	// SW-TRAIN    PC8
    break ;
    default:;
  }

  if ( xxx )
  {
    return 1 ;
  }
  return 0;
}

void pauseEvents(uint8_t event)
{
  event=event & EVT_KEY_MASK;
  if(event < (int)DIM(keys))  keys[event].pauseEvents();
}

void killEvents(uint8_t event)
{
  event=event & EVT_KEY_MASK;
  if(event < (int)DIM(keys))  keys[event].killEvents();
}


// keys:
// KEY_EXIT    PA31 (PC24)
// KEY_MENU    PB6 (PB5)
// KEY_DOWN  LCD5  PC3 (PC5)
// KEY_UP    LCD6  PC2 (PC1)
// KEY_RIGHT LCD4  PC4 (PC4)
// KEY_LEFT  LCD3  PC5 (PC3)
// Reqd. bit 6 LEFT, 5 RIGHT, 4 UP, 3 DOWN 2 EXIT 1 MENU
// LCD pins 5 DOWN, 4 RIGHT, 3 LEFT, 1 UP
uint32_t read_keys()
{
	register uint32_t x ;
	register uint32_t y ;
	
	x = PIOC->PIO_PDSR << 1 ; // 6 LEFT, 5 RIGHT, 4 DOWN, 3 UP ()
#ifdef REVB
	y = x & 0x00000020 ;		// RIGHT
	if ( x & 0x00000004 )
	{
		y |= 0x00000010 ;			// UP
	}
	if ( x & 0x00000010 )
	{
		y |= 0x00000040 ;			// LEFT
	}
	if ( x & 0x00000040 )
	{
		y |= 0x00000008 ;			// DOWN
	}
#else	
	y = x & 0x00000060 ;
	if ( x & 0x00000008 )
	{
		y |= 0x00000010 ;
	}
	if ( x & 0x00000010 )
	{
		y |= 0x00000008 ;
	}
#endif
#ifdef REVB
	if ( PIOC->PIO_PDSR & 0x01000000 )
#else 
	if ( PIOA->PIO_PDSR & 0x80000000 )
#endif
	{
		y |= 4 ;		// EXIT
	}
#ifdef REVB
	if ( PIOB->PIO_PDSR & 0x000000020 )
#else 
	if ( PIOB->PIO_PDSR & 0x000000040 )
#endif
	{
		y |= 2 ;		// MENU
	}
	return y ;
}



uint32_t read_trims()
{
	uint32_t trims ;
	uint32_t trima ;

	trims = 0 ;

	trima = PIOA->PIO_PDSR ;
// TRIM_LH_DOWN    PA7 (PA23)
#ifdef REVB
	if ( ( trima & 0x00800000 ) == 0 )
#else
	if ( ( trima & 0x0080 ) == 0 )
#endif
	{
		trims |= 1 ;
	}
    
// TRIM_LV_DOWN  PA27 (PA24)
#ifdef REVB
	if ( ( trima & 0x01000000 ) == 0 )
#else
	if ( ( trima & 0x08000000 ) == 0 )
#endif
	{
		trims |= 4 ;
	}

// TRIM_RV_UP    PA30 (PA1)
#ifdef REVB
	if ( ( trima & 0x00000002 ) == 0 )
#else
	if ( ( trima & 0x40000000 ) == 0 )
#endif
	{
		trims |= 0x20 ;
	}

// TRIM_RH_DOWN    PA29 (PA0)
#ifdef REVB
	if ( ( trima & 0x00000001 ) == 0 )
#else 
	if ( ( trima & 0x20000000 ) == 0 )
#endif 
	{
		trims |= 0x40 ;
	}

// TRIM_LH_UP PB4
	if ( ( PIOB->PIO_PDSR & 0x10 ) == 0 )
	{
		trims |= 2 ;
	}

	trima = PIOC->PIO_PDSR ;
// TRIM_LV_UP   PC28
	if ( ( trima & 0x10000000 ) == 0 )
	{
		trims |= 8 ;
	}

// TRIM_RV_DOWN   PC10
	if ( ( trima & 0x00000400 ) == 0 )
	{
		trims |= 0x10 ;
	}

// TRIM_RH_UP   PC9
	if ( ( trima & 0x00000200 ) == 0 )
	{
		trims |= 0x80 ;
	}

	return trims ;
}


volatile uint16_t g_tmr10ms;
volatile uint8_t  g_blinkTmr10ms;


void per10ms()
{
	register uint32_t i ;

  g_tmr10ms++;
  g_blinkTmr10ms++;

//	if ( PIOC->PIO_ODSR & 0x00080000 )
//	{
//		PIOC->PIO_CODR = 0x00200000L ;	// Set bit C19 OFF
//	}
//	else
//	{
//		PIOC->PIO_SODR = 0x00200000L ;	// Set bit C19 ON
//	}

  uint8_t enuk = KEY_MENU;
  uint8_t    in = ~read_keys() ;
  for( i=1; i<7; i++)
  {
    //INP_B_KEY_MEN 1  .. INP_B_KEY_LFT 6
    keys[enuk].input(in & (1<<i),(EnumKeys)enuk);
    ++enuk;
  }
//  static const uint8_t crossTrim[]={
//    1<<INP_D_TRM_LH_DWN,
//    1<<INP_D_TRM_LH_UP,
//    1<<INP_D_TRM_LV_DWN,
//    1<<INP_D_TRM_LV_UP,
//    1<<INP_D_TRM_RV_DWN,
//    1<<INP_D_TRM_RV_UP,
//    1<<INP_D_TRM_RH_DWN,
//    1<<INP_D_TRM_RH_UP
//  };
  in = read_trims() ;

	for( i=1; i<256; i<<=1)
  {
    // INP_D_TRM_RH_UP   0 .. INP_D_TRM_LH_UP   7
    keys[enuk].input(in & i,(EnumKeys)enuk);
    ++enuk;
  }
}


//void put_frsky_fifo( uint8_t c )
//{
//	FrskyFifo.fifo[FrskyFifo.in] = c ;
//	FrskyFifo.count++ ;
//	FrskyFifo.in = (FrskyFifo.in + 1) & 0x1F ;
//}

//int32_t get_frsky_fifo()
//{
//	uint32_t rxchar ;

//	if (FrskyFifo.count )						// Look for char available
//	{
//		rxchar = FrskyFifo.fifo[FrskyFifo.out] ;
//		__disable_irq() ;
//		FrskyFifo.count-- ;						// Protect from interrupts
//		__enable_irq() ;
//		FrskyFifo.out = ( FrskyFifo.out + 1 ) & 0x1F ;
//		return rxchar ;
//	}
//	return -1 ;
//}



// SPI i/f to EEPROM (4Mb)
// Peripheral ID 21 (0x00200000)
// Connections:
// SS   PA11 (peripheral A)
// MISO PA12 (peripheral A)
// MOSI PA13 (peripheral A)
// SCK  PA14 (peripheral A)
// Set clock to 3 MHz, AT25 device is rated to 70MHz, 18MHz would be better
void init_spi()
{
//	register Pio *pioptr ;
	register Spi *spiptr ;
	register uint32_t timer ;
	register uint8_t *p ;
	uint8_t spi_buf[4] ;

  PMC->PMC_PCER0 |= 0x00200000L ;		// Enable peripheral clock to SPI
  /* Configure PIO */
	configure_pins( 0x00007800, PIN_PERIPHERAL | PIN_INPUT | PIN_PER_A | PIN_PORTA | PIN_NO_PULLUP ) ;
//	pioptr = PIOA ;
//  pioptr->PIO_ABCDSR[0] &= ~0x00007800 ;	// Peripheral A bits 14,13,12,11
//  pioptr->PIO_ABCDSR[1] &= ~0x00007800 ;	// Peripheral A
//  pioptr->PIO_PDR = 0x00007800 ;					// Assign to peripheral
	
	spiptr = SPI ;
	timer = ( Master_frequency / 3000000 ) << 8 ;		// Baud rate 3Mb/s
	spiptr->SPI_MR = 0x14000011 ;				// 0001 0100 0000 0000 0000 0000 0001 0001 Master
	spiptr->SPI_CSR[0] = 0x01180009 | timer ;		// 0000 0001 0001 1000 xxxx xxxx 0000 1001
	NVIC_EnableIRQ(SPI_IRQn) ;

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

}


void end_spi()
{
	SPI->SPI_CR = 2 ;								// Disable
	SPI->SPI_IDR = 0x07FF ;					// All interrupts off
	NVIC_DisableIRQ(SPI_IRQn) ;
}

extern "C" void SPI_IRQHandler()
{
	register Spi *spiptr ;

	spiptr = SPI ;
	SPI->SPI_IDR = 0x07FF ;			// All interrupts off
	spiptr->SPI_CR = 2 ;				// Disable
	(void) spiptr->SPI_RDR ;		// Dump any rx data
	(void) spiptr->SPI_SR ;			// Clear error flags
	spiptr->SPI_PTCR = SPI_PTCR_RXTDIS | SPI_PTCR_TXTDIS ;	// Stop tramsfers
	Spi_complete = 1 ;					// Indicate completion

// Power save
//  PMC->PMC_PCER0 &= ~0x00200000L ;		// Disable peripheral clock to SPI
	
}


void eeprom_write_enable()
{
	eeprom_write_one( 6, 0 ) ;
}

uint32_t eeprom_read_status()
{
	return eeprom_write_one( 5, 1 ) ;
}

uint32_t  eeprom_write_one( uint8_t byte, uint8_t count )
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

// The following superceded by the PDC version after
//uint32_t spi_action( register uint8_t *command, register uint8_t *tx, register uint8_t *rx, register uint32_t comlen, register uint32_t count )
//{
//	register Spi *spiptr ;
//	register uint32_t result ;

////  PMC->PMC_PCER0 |= 0x00200000L ;		// Enable peripheral clock to SPI

//	result = 0 ; 
//	spiptr = SPI ;
//	spiptr->SPI_CR = 1 ;								// Enable
//	(void) spiptr->SPI_RDR ;		// Dump any rx data
//	while( comlen || count )
//	{
//		result = 0 ;
//		while( ( spiptr->SPI_SR & SPI_SR_TXEMPTY ) == 0 )
//		{
//			// wait
//			if ( ++result > 10000 )
//			{
//				result = 0xFFFF ;
//				break ;				
//			}
//		}
//		if ( result > 10000 )
//		{
//			break ;
//		}
////		if ( count == 1 )
////		{
////			spiptr->SPI_CR = SPI_CR_LASTXFER ;		// LastXfer bit
////		}
//		spiptr->SPI_TDR = comlen ? *command++ : tx ? *tx++ : 0 ;
//		result = 0 ;
//		while( ( spiptr->SPI_SR & SPI_SR_RDRF ) == 0 )
//		{
//			// wait for received
//			if ( ++result > 10000 )
//			{
//				result = 0x2FFFF ;
//				break ;				
//			}
//		}
//		if ( result > 10000 )
//		{
//			break ;
//		}
//		if ( !comlen )
//		{
//			if ( rx )
//		{
//				*rx++ = spiptr->SPI_RDR ;
//			}
//			else
//			{
//				(void) spiptr->SPI_RDR ;
//			}
//		}
//		else
//		{
//			(void) spiptr->SPI_RDR ;
//		}
//		if ( comlen )
//		{
//			comlen -= 1 ;			
//		}
//		else
//		{
//			count -= 1 ;
//		}
//	}
//	if ( result <= 10000 )
//	{
//		result = 0 ;
//	}
//	spiptr->SPI_CR = 2 ;								// Disable
//	(void) spiptr->SPI_SR ;							// Clear error flags

//// Power save
////  PMC->PMC_PCER0 &= ~0x00200000L ;		// Disable peripheral clock to SPI

//	return result ;
//}

uint32_t spi_PDC_action( register uint8_t *command, register uint8_t *tx, register uint8_t *rx, register uint32_t comlen, register uint32_t count )
{
	register Spi *spiptr ;
//	register uint32_t result ;
	register uint32_t condition ;
	static uint8_t discard_rx_command[4] ;

//  PMC->PMC_PCER0 |= 0x00200000L ;		// Enable peripheral clock to SPI

	Spi_complete = 0 ;
	if ( comlen > 4 )
	{
		Spi_complete = 1 ;
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
	spiptr->SPI_IER = condition ; 

	return 0 ;
}




/**
 * Configures a UART peripheral with the specified parameters.
 *
 * baudrate  Baudrate at which the UART should operate (in Hz).
 * masterClock  Frequency of the system master clock (in Hz).
 * uses PA9 and PA10, RXD2 and TXD2
 */
void UART_Configure( uint32_t baudrate, uint32_t masterClock)
{
//    const Pin pPins[] = CONSOLE_PINS;
  register Uart *pUart = CONSOLE_USART;
//	register Pio *pioptr ;

  /* Configure PIO */
	configure_pins( (PIO_PA9 | PIO_PA10), PIN_PERIPHERAL | PIN_INPUT | PIN_PER_A | PIN_PORTA | PIN_NO_PULLUP ) ;
//	pioptr = PIOA ;
//  pioptr->PIO_ABCDSR[0] &= ~(PIO_PA9 | PIO_PA10) ;	// Peripheral A
//  pioptr->PIO_ABCDSR[1] &= ~(PIO_PA9 | PIO_PA10) ;	// Peripheral A
//  pioptr->PIO_PDR = (PIO_PA9 | PIO_PA10) ;					// Assign to peripheral

  /* Configure PMC */
  PMC->PMC_PCER0 = 1 << CONSOLE_ID;

  /* Reset and disable receiver & transmitter */
  pUart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX
                 | UART_CR_RXDIS | UART_CR_TXDIS;

  /* Configure mode */
  pUart->UART_MR =  0x800 ;  // NORMAL, No Parity

  /* Configure baudrate */
  /* Asynchronous, no oversampling */
  pUart->UART_BRGR = (masterClock / baudrate) / 16;

  /* Disable PDC channel */
  pUart->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

  /* Enable receiver and transmitter */
  pUart->UART_CR = UART_CR_RXEN | UART_CR_TXEN;

}

void UART3_Configure( uint32_t baudrate, uint32_t masterClock)
{
//    const Pin pPins[] = CONSOLE_PINS;
  register Uart *pUart = BT_USART;
//	register Pio *pioptr ;

  /* Configure PIO */
	configure_pins( (PIO_PB2 | PIO_PB3), PIN_PERIPHERAL | PIN_INPUT | PIN_PER_A | PIN_PORTB | PIN_NO_PULLUP ) ;
//	pioptr = PIOB ;
//  pioptr->PIO_ABCDSR[0] &= ~(PIO_PB2 | PIO_PB3) ;	// Peripheral A
//  pioptr->PIO_ABCDSR[1] &= ~(PIO_PB2 | PIO_PB3) ;	// Peripheral A
//  pioptr->PIO_PDR = (PIO_PB2 | PIO_PB3) ;					// Assign to peripheral

  /* Configure PMC */
  PMC->PMC_PCER0 = 1 << BT_ID;

  /* Reset and disable receiver & transmitter */
  pUart->UART_CR = UART_CR_RSTRX | UART_CR_RSTTX
                 | UART_CR_RXDIS | UART_CR_TXDIS;

  /* Configure mode */
  pUart->UART_MR =  0x800 ;  // NORMAL, No Parity

  /* Configure baudrate */
  /* Asynchronous, no oversampling */
//  pUart->UART_BRGR = (masterClock / baudrate) / 16;
  
	baudrate = (masterClock * 8 / baudrate) / 16 ;
  pUart->UART_BRGR = ( baudrate / 8 ) | ( ( baudrate & 7 ) << 16 ) ;	// Fractional part to allow 152000 baud

  /* Disable PDC channel */
  pUart->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

  /* Enable receiver and transmitter */
  pUart->UART_CR = UART_CR_RXEN | UART_CR_TXEN;

}

// USART0 configuration, we will use this for FrSky etc
// Work in Progress, UNTESTED
// Uses PA5 and PA6 (RXD and TXD)
void UART2_Configure( uint32_t baudrate, uint32_t masterClock)
{
////    const Pin pPins[] = CONSOLE_PINS;
  register Usart *pUsart = SECOND_USART;
//	register Pio *pioptr ;

  /* Configure PIO */
	configure_pins( (PIO_PA5 | PIO_PA6), PIN_PERIPHERAL | PIN_INPUT | PIN_PER_A | PIN_PORTA | PIN_NO_PULLUP ) ;
//	pioptr = PIOA ;
//  pioptr->PIO_ABCDSR[0] &= ~(PIO_PA5 | PIO_PA6) ;	// Peripheral A
//  pioptr->PIO_ABCDSR[1] &= ~(PIO_PA5 | PIO_PA6) ;	// Peripheral A
//  pioptr->PIO_PDR = (PIO_PA5 | PIO_PA6) ;					// Assign to peripheral

//  /* Configure PMC */
  PMC->PMC_PCER0 = 1 << SECOND_ID;

//  /* Reset and disable receiver & transmitter */
  pUsart->US_CR = US_CR_RSTRX | US_CR_RSTTX
	                 | US_CR_RXDIS | US_CR_TXDIS;

//  /* Configure mode */
  pUsart->US_MR =  0x000008C0 ;  // NORMAL, No Parity, 8 bit

//  /* Configure baudrate */
//  /* Asynchronous, no oversampling */
  pUsart->US_BRGR = (masterClock / baudrate) / 16;

//  /* Disable PDC channel */
  pUsart->US_PTCR = US_PTCR_RXTDIS | US_PTCR_TXTDIS;

//  /* Enable receiver and transmitter */
  pUsart->US_CR = US_CR_RXEN | US_CR_TXEN;

}

// set outPtr start of buffer
// give 1st buffer to Uart as RPR/RCR
// set outPtr start of buffer
// give 2nd buffer to Uart as RNPR/RNCR

// read RPR
// if RPRcopy in TelemetryInBuffer[TelemetryActiveBuffer]
// process chars up to RPRcopy
// else process remaining chars in buffer, give buffer to Uart as RNPR/RNCR
//      TelemetryActiveBuffer becomes other buffer


//uint8_t OutputBuffer[128] ;
//uint32_t OutIndex ;

//void charProcess( uint8_t byte )
//{
//	OutputBuffer[OutIndex++] = byte ;
//	OutIndex &= 0x007F ;	
//}

//void poll2ndUsart10mS()
//{
//	rxPdcUsart( charProcess ) ;	
//}



void startPdcUsartReceive()
{
  register Usart *pUsart = SECOND_USART;
	
	TelemetryInBuffer[0].outPtr = TelemetryInBuffer[0].fifo ;
	TelemetryInBuffer[1].outPtr = TelemetryInBuffer[1].fifo ;
	pUsart->US_RPR = (uint32_t)TelemetryInBuffer[0].fifo ;
	pUsart->US_RNPR = (uint32_t)TelemetryInBuffer[1].fifo ;
	pUsart->US_RCR = RX_UART_BUFFER_SIZE ;
	pUsart->US_RNCR = RX_UART_BUFFER_SIZE ;
	pUsart->US_PTCR = US_PTCR_RXTEN ;
	TelemetryActiveBuffer = 0 ;
}

void endPdcUsartReceive()
{
  register Usart *pUsart = SECOND_USART;
	
	pUsart->US_PTCR = US_PTCR_RXTDIS ;
}

void rxPdcUsart( void (*pChProcess)(uint8_t x) )
{
  register Usart *pUsart = SECOND_USART;
	uint8_t *ptr ;
	uint8_t *endPtr ;
//	uint32_t bufIndex ;
//	uint32_t i ;
	uint32_t j ;

 //Find out where the DMA has got to
	__disable_irq() ;
	pUsart->US_PTCR = US_PTCR_RXTDIS ;		// Freeze DMA
	ptr = (uint8_t *)pUsart->US_RPR ;
	j = pUsart->US_RNCR ;
	pUsart->US_PTCR = US_PTCR_RXTEN ;			// DMA active again
	__enable_irq() ;

	endPtr = ptr - 1 ;
	ptr = TelemetryInBuffer[TelemetryActiveBuffer].outPtr ;
	if ( j == 0 )		// First buf is full
	{
		endPtr = &TelemetryInBuffer[TelemetryActiveBuffer].fifo[RX_UART_BUFFER_SIZE-1] ;		// last byte
	}
	while ( ptr <= endPtr )
	{
		(*pChProcess)(*ptr++) ;
	}
	TelemetryInBuffer[TelemetryActiveBuffer].outPtr = ptr ;
	if ( j == 0 )		// First buf is full
	{
		TelemetryInBuffer[TelemetryActiveBuffer].outPtr = TelemetryInBuffer[TelemetryActiveBuffer].fifo ;
		pUsart->US_RNPR = (uint32_t)TelemetryInBuffer[TelemetryActiveBuffer].fifo ;
		pUsart->US_RNCR = RX_UART_BUFFER_SIZE ;
		TelemetryActiveBuffer ^= 1 ;		// Other buffer is active
		rxPdcUsart( pChProcess ) ;			// Get any chars from second buffer
	}
}

uint32_t txPdcUsart( uint8_t *buffer, uint32_t size )
{
  register Usart *pUsart = SECOND_USART;

	if ( pUsart->US_TNCR == 0 )
	{
		pUsart->US_TNPR = (uint32_t)buffer ;
		pUsart->US_TNCR = size ;
		pUsart->US_PTCR = US_PTCR_TXTEN ;
		return 1 ;
	}
	return 0 ;
}

uint32_t txPdcPending()
{
  register Usart *pUsart = SECOND_USART;
	uint32_t x ;

	__disable_irq() ;
	pUsart->US_PTCR = US_PTCR_TXTDIS ;		// Freeze DMA
	x = pUsart->US_TNCR ;				// Total
	x += pUsart->US_TCR ;				// Still to send
	pUsart->US_PTCR = US_PTCR_TXTEN ;			// DMA active again
	__enable_irq() ;

	return x ;
}



/**
 * Outputs a character on the UART line.
 *
 * This function is synchronous (i.e. uses polling).
 * c  Character to send.
 */
void txmit( uint8_t c )
{
  Uart *pUart=CONSOLE_USART ;

	/* Wait for the transmitter to be ready */
  while ( (pUart->UART_SR & UART_SR_TXEMPTY) == 0 ) ;

  /* Send character */
  pUart->UART_THR=c ;
}

// Outputs a string to the UART

void uputs( register char *string )
{
	while ( *string )
	{
		txmit( *string++ ) ;		
	}	
}

uint16_t rxuart()
{
  Uart *pUart=CONSOLE_USART ;

  if (pUart->UART_SR & UART_SR_RXRDY)
	{
		return pUart->UART_RHR ;
	}
	return 0xFFFF ;
}

void txmit2nd( uint8_t c )
{
  register Usart *pUsart = SECOND_USART;

	/* Wait for the transmitter to be ready */
  while ( (pUsart->US_CSR & US_CSR_TXEMPTY) == 0 ) ;

  /* Send character */
  pUsart->US_THR=c ;
}

uint16_t rx2nduart()
{
  register Usart *pUsart = SECOND_USART;

  if (pUsart->US_CSR & US_CSR_RXRDY)
	{
		return pUsart->US_RHR ;
	}
	return 0xFFFF ;
}

void txmitBt( uint8_t c )
{
  Uart *pUart=BT_USART ;
	uint32_t x ;

	/* Wait for the transmitter to be ready */
	x = 10000 ;
  while ( (pUart->UART_SR & UART_SR_TXEMPTY) == 0 )
	{
		if ( --x == 0 )
		{
			break ;			// Timeout so we don't hang
		}
	}
  /* Send character */
  pUart->UART_THR=c ;
}

uint16_t rxBtuart()
{
  Uart *pUart=BT_USART ;

  if (pUart->UART_SR & UART_SR_RXRDY)
	{
		return pUart->UART_RHR ;
	}
	return 0xFFFF ;
}

// Send a <cr><lf> combination to the serial port
void crlf()
{
	txmit( 13 ) ;
	txmit( 10 ) ;
}

// Send the 32 bit value to the RS232 port as 8 hex digits
void p8hex( uint32_t value )
{
	p4hex( value >> 16 ) ;
	p4hex( value ) ;
}

// Send the 16 bit value to the RS232 port as 4 hex digits
void p4hex( uint16_t value )
{
	p2hex( value >> 8 ) ;
	p2hex( value ) ;
}

// Send the 8 bit value to the RS232 port as 2 hex digits
void p2hex( unsigned char c )
{
//	asm("swap %c") ;
	hex_digit_send( c >> 4 ) ;
//	asm("swap %c") ;
	hex_digit_send( c ) ;
}

// Send a single 4 bit value to the RS232 port as a hex digit
void hex_digit_send( unsigned char c )
{
	c &= 0x0F ;
	if ( c > 9 )
	{
		c += 7 ;
	}
	c += '0' ;
	txmit( c ) ;
}


// Read 8 (9 for REVB) ADC channels
// Documented bug, must do them 1 by 1
void read_9_adc()
{
	register Adc *padc ;
	register uint32_t y ;
	register uint32_t x ;

//	PMC->PMC_PCER0 |= 0x20000000L ;		// Enable peripheral clock to ADC

	padc = ADC ;
	y = padc->ADC_ISR ;		// Clear EOC flags
	for ( y = NUMBER_ANALOG+1 ; --y > 0 ; )		// Include temp sensor
	{
		padc->ADC_CR = 2 ;		// Start conversion
		x = 0 ;
		while ( ( padc->ADC_ISR & 0x01000000 ) == 0 )
		{
			// wait for DRDY flag
			if ( ++x > 1000000 )
			{
				break ;		// Software timeout				
			}
		}
		x = padc->ADC_LCDR ;		// Clear DRSY flag
	}
	// Next bit may be done using the PDC
	Analog_values[0] = ADC->ADC_CDR1 ;
	Analog_values[1] = ADC->ADC_CDR2 ;
	Analog_values[2] = ADC->ADC_CDR3 ;
	Analog_values[3] = ADC->ADC_CDR4 ;
	Analog_values[4] = ADC->ADC_CDR5 ;
	Analog_values[5] = ADC->ADC_CDR9 ;
	Analog_values[6] = ADC->ADC_CDR13 ;
	Analog_values[7] = ADC->ADC_CDR14 ;
#ifdef REVB
	Analog_values[8] = ADC->ADC_CDR8 ;
#endif
	Temperature = ( Temperature * 7 + ADC->ADC_CDR15 ) >> 3 ;	// Filter it
	if ( Temperature > Max_temperature )
	{
		Max_temperature = Temperature ;		
	}

// Power save
//  PMC->PMC_PCER0 &= ~0x20000000L ;		// Disable peripheral clock to ADC

}


// Settings for mode register ADC_MR
// USEQ off - silicon problem, doesn't work
// TRANSFER = 1
// TRACKTIM = 4 (5 clock periods)
// ANACH = 0
// SETTLING = 1 (not used if ANACH = 0)
// STARTUP = 1 (8 clock periods)
// PRESCAL = 3.6 MHz clock (between 1 and 20MHz)
// FREERUN = 0
// FWUP = 0
// SLEEP = 0
// LOWRES = 0
// TRGSEL = 0
// TRGEN = 0 (software trigger only)
void init_adc()
{
	register Adc *padc ;
	register uint32_t timer ;

	timer = ( Master_frequency / (3600000*2) ) << 8 ;
	// Enable peripheral clock ADC = bit 29
  PMC->PMC_PCER0 |= 0x20000000L ;		// Enable peripheral clock to ADC
	padc = ADC ;
	padc->ADC_MR = 0x14110000 | timer ;  // 0001 0100 0001 0001 xxxx xxxx 0000 0000
	padc->ADC_ACR = ADC_ACR_TSON ;			// Turn on temp sensor
#ifdef REVB
	padc->ADC_CHER = 0x0000E33E ;  // channels 1,2,3,4,5,8,9,13,14,15
#else
	padc->ADC_CHER = 0x0000E23E ;  // channels 1,2,3,4,5,9,13,14,15
#endif
	padc->ADC_CGR = 0 ;  // Gain = 1, all channels
	padc->ADC_COR = 0 ;  // Single ended, 0 offset, all channels
}


//void eeprom_write_byte_cmp (uint8_t dat, uint16_t pointer_eeprom)
//{
//	eeprom[pointer_eeprom] = dat ;
//	Eeprom_image_updated = 1 ;
//}

//void eeWriteBlockCmp(const void *i_pointer_ram, void *i_pointer_eeprom, size_t size)
//{
//  const char* pointer_ram = (const char*)i_pointer_ram;
//  uint32_t    pointer_eeprom = (uint32_t)i_pointer_eeprom;
//  while(size){
//    eeprom_write_byte_cmp(*pointer_ram++,pointer_eeprom++);
//    size--;
//  }
//}

//void eeprom_read_block( void *i_pointer_ram, const void *i_pointer_eeprom, register uint32_t size )
//{
//  char *pointer_ram = (char*)i_pointer_ram;
//  uint32_t    pointer_eeprom = (uint32_t)i_pointer_eeprom;
//	while ( size )
//	{
//		*pointer_ram++ = eeprom[pointer_eeprom++] ;
//		size -= 1 ;		
//	}
//}


// Start TIMER3 for input capture
void start_timer3()
{
  register Tc *ptc ;
//	register Pio *pioptr ;

	// Enable peripheral clock TC0 = bit 23 thru TC5 = bit 28
  PMC->PMC_PCER0 |= 0x04000000L ;		// Enable peripheral clock to TC3

  ptc = TC1 ;		// Tc block 1 (TC3-5)
	ptc->TC_BCR = 0 ;			// No sync
	ptc->TC_BMR = 2 ;
	ptc->TC_CHANNEL[0].TC_CMR = 0x00000000 ;	// Capture mode
	ptc->TC_CHANNEL[0].TC_CMR = 0x00090005 ;	// 0000 0000 0000 1001 0000 0000 0000 0101, XC0, A rise, b fall
	ptc->TC_CHANNEL[0].TC_CCR = 5 ;		// Enable clock and trigger it (may only need trigger)

	configure_pins( PIO_PC23, PIN_PERIPHERAL | PIN_INPUT | PIN_PER_B | PIN_PORTC | PIN_PULLUP ) ;
//	pioptr = PIOC ;
//  pioptr->PIO_ABCDSR[0] |= 0x00800000 ;		// Peripheral B = TIOA3
//  pioptr->PIO_ABCDSR[1] &= ~0x00800000 ;	// Peripheral B
//	pioptr->PIO_PDR = 0x00800000L ;		// Disable bit C23 (TIOA3) Assign to peripheral
	NVIC_SetPriority( TC3_IRQn, 15 ) ; // Low ppiority interrupt
	NVIC_EnableIRQ(TC3_IRQn) ;
	ptc->TC_CHANNEL[0].TC_IER = TC_IER0_LDRAS ;
}

// Start Timer4 to provide 0.5uS clock for input capture
void start_timer4()
{
  register Tc *ptc ;
	register uint32_t timer ;

	timer = Master_frequency / (2*2000000) ;		// MCK/2 and 2MHz

	// Enable peripheral clock TC0 = bit 23 thru TC5 = bit 28
  PMC->PMC_PCER0 |= 0x08000000L ;		// Enable peripheral clock to TC4

  ptc = TC1 ;		// Tc block 1 (TC3-5)
	ptc->TC_BCR = 0 ;			// No sync
	ptc->TC_BMR = 0 ;
	ptc->TC_CHANNEL[1].TC_CMR = 0x00008000 ;	// Waveform mode
	ptc->TC_CHANNEL[1].TC_RC = timer ;
	ptc->TC_CHANNEL[1].TC_RA = timer >> 1 ;
	ptc->TC_CHANNEL[1].TC_CMR = 0x0009C000 ;	// 0000 0000 0000 1001 1100 0000 0100 0000
																						// MCK/2, set @ RA, Clear @ RC waveform
	ptc->TC_CHANNEL[1].TC_CCR = 5 ;		// Enable clock and trigger it (may only need trigger)
}

void start_ppm_capture()
{
	start_timer4() ;
	start_timer3() ;
}

void end_ppm_capture()
{
	TC1->TC_CHANNEL[0].TC_IDR = TC_IDR0_LDRAS ;
	NVIC_DisableIRQ(TC3_IRQn) ;
}


// Timer3 used for PPM_IN pulse width capture. Counter running at 16MHz / 8 = 2MHz
// equating to one count every half microisecond. (2 counts = 1us). Control channel
// count delta values thus can range from about 1600 to 4400 counts (800us to 2200us),
// corresponding to a PPM signal in the range 0.8ms to 2.2ms (1.5ms at center).
// (The timer is free-running and is thus not reset to zero at each capture interval.)
// Timer 4 generates the 2MHz clock to clock Timer 3

uint16_t Temp_captures[8] ;

extern "C" void TC3_IRQHandler() //capture ppm in at 2MHz
{
  uint16_t capture ;
  static uint16_t lastCapt ;
  uint16_t val ;
	
	capture = TC1->TC_CHANNEL[0].TC_RA ;
	(void) TC1->TC_CHANNEL[0].TC_SR ;		// Acknowledgethe interrupt
  
//	cli();
//  ETIMSK &= ~(1<<TICIE3); //stop reentrance
//  sei();

  val = (capture - lastCapt) / 2 ;
  lastCapt = capture;

  // We prcoess g_ppmInsright here to make servo movement as smooth as possible
  //    while under trainee control
  if(ppmInState && ppmInState<=8){
    if(val>800 && val<2200)
		{
			Temp_captures[ppmInState - 1] = capture ;
      g_ppmIns[ppmInState++ - 1] =
        (int16_t)(val - 1500)*(g_eeGeneral.PPM_Multiplier+10)/10; //+-500 != 512, but close enough.

    }else{
      ppmInState=0; // not triggered
    }
  }else{
    if(val>4000 && val < 16000)
    {
      ppmInState=1; // triggered
    }
  }

//  cli();
//  ETIMSK |= (1<<TICIE3);
//  sei();
}

// Initialise the SSC to allow PXX output.
// TD is on PA17, peripheral A
void init_ssc()
{
//	register Pio *pioptr ;
	register Ssc *sscptr ;

  PMC->PMC_PCER0 |= 0x00400000L ;		// Enable peripheral clock to SSC
	
	configure_pins( PIO_PA17, PIN_PERIPHERAL | PIN_INPUT | PIN_PER_A | PIN_PORTA | PIN_NO_PULLUP ) ;
//	pioptr = PIOA ;
//  pioptr->PIO_ABCDSR[0] &= ~0x00020000 ;	// Peripheral A bit 17
//  pioptr->PIO_ABCDSR[1] &= ~0x00020000 ;	// Peripheral A
//  pioptr->PIO_PDR = 0x00020000 ;					// Assign to peripheral
	
	sscptr = SSC ;
	sscptr->SSC_CMR = Master_frequency / (125000*2) ;		// 8uS per bit
	sscptr->SSC_TCMR = 0 ;  	//  0000 0000 0000 0000 0000 0000 0000 0000
	sscptr->SSC_TFMR = 0x00000027 ; 	//  0000 0000 0000 0000 0000 0000 1010 0111 (8 bit data, lsb)
	sscptr->SSC_CR = SSC_CR_TXEN ;

}

void disable_ssc()
{
	register Pio *pioptr ;
	register Ssc *sscptr ;

	// Revert back to pwm output
	pioptr = PIOA ;
	pioptr->PIO_PER = PIO_PA17 ;						// Assign A17 to PIO
	
	sscptr = SSC ;
	sscptr->SSC_CR = SSC_CR_TXDIS ;
}

#ifndef SIMU
void configure_pins( uint32_t pins, uint16_t config )
{
	register Pio *pioptr ;
	
	pioptr = PIOA + ( ( config & PIN_PORT_MASK ) >> 6) ;
	if ( config & PIN_PULLUP )
	{
		pioptr->PIO_PPDDR = pins ;
		pioptr->PIO_PUER = pins ;
	}
	else
	{
		pioptr->PIO_PUDR = pins ;
	}

	if ( config & PIN_PULLDOWN )
	{
		pioptr->PIO_PUDR = pins ;
		pioptr->PIO_PPDER = pins ;
	}
	else
	{
		pioptr->PIO_PPDDR = pins ;
	}

	if ( config & PIN_HIGH )
	{
		pioptr->PIO_SODR = pins ;		
	}
	else
	{
		pioptr->PIO_CODR = pins ;		
	}

	if ( config & PIN_INPUT )
	{
		pioptr->PIO_ODR = pins ;
	}
	else
	{
		pioptr->PIO_OER = pins ;
	}

	if ( config & PIN_PERI_MASK_L )
	{
		pioptr->PIO_ABCDSR[0] |= pins ;
	}
	else
	{
		pioptr->PIO_ABCDSR[0] &= ~pins ;
	}
	if ( config & PIN_PERI_MASK_H )
	{
		pioptr->PIO_ABCDSR[1] |= pins ;
	}
	else
	{
		pioptr->PIO_ABCDSR[1] &= ~pins ;
	}

	if ( config & PIN_ENABLE )
	{
		pioptr->PIO_PER = pins ;		
	}
	else
	{
		pioptr->PIO_PDR = pins ;		
	}
}
#endif

// SD Card routines

#define MCI_INITIAL_SPEED   400000

// States for initialising card
#define SD_ST_EMPTY		0
#define SD_ST_INIT1		1
#define SD_ST_INIT2		2
#define SD_ST_IDLE		3
#define SD_ST_READY		4
#define SD_ST_IDENT		5
#define SD_ST_STBY		6
#define SD_ST_TRAN		7
#define SD_ST_DATA		8


uint32_t Card_ID[4] ;
uint32_t Card_SCR[2] ;
uint32_t Card_CSD[4] ;
uint32_t Card_state = SD_ST_EMPTY ;
uint32_t Sd_128_resp[4] ;
uint32_t Sd_rca ;
//uint32_t Cmd_55_resp ;
uint32_t Cmd_8_resp ;
uint32_t Cmd_A41_resp ;
/**
 * Initializes a MCI peripheral.
 */
void SD_Init()
{
  unsigned short clkDiv;
  Hsmci *pMciHw = HSMCI ;

  /* Enable the MCI peripheral */
  PMC->PMC_PCER0 |= 1 << ID_HSMCI ;		// Enable peripheral clock to HSMCI
  pMciHw->HSMCI_CR = HSMCI_CR_SWRST;  /* Reset the MCI */
  pMciHw->HSMCI_CR = HSMCI_CR_MCIDIS | HSMCI_CR_PWSDIS;  /* Disable the MCI */
  pMciHw->HSMCI_IDR = 0xFFFFFFFF;  /* Disable all the interrupts */
  pMciHw->HSMCI_DTOR = HSMCI_DTOR_DTOCYC | HSMCI_DTOR_DTOMUL ;  /* Set the Data Timeout Register */
  pMciHw->HSMCI_CSTOR = HSMCI_CSTOR_CSTOCYC | HSMCI_CSTOR_CSTOMUL ;  /* CSTOR ? */
  /* Set the Mode Register: 400KHz for MCK = 48MHz (CLKDIV = 58) */
  clkDiv = (Master_frequency / (MCI_INITIAL_SPEED * 2)) - 1;
  pMciHw->HSMCI_MR = clkDiv | (7 << 8) ;

  /* Set the SDCard Register 1-bit, slot A */
  pMciHw->HSMCI_SDCR = HSMCI_SDCR_SDCSEL_SLOTA | HSMCI_SDCR_SDCBUS_1 ;
  /* Enable the MCI and the Power Saving */
  pMciHw->HSMCI_CR = HSMCI_CR_MCIEN | HSMCI_CR_PWSEN ;
  /* Configure MCI */
  pMciHw->HSMCI_CFG = HSMCI_CFG_FIFOMODE | ((1 << 4) & HSMCI_CFG_FERRCTRL);
}


/**
 * Configure the  MCI SDCBUS in the MCI_SDCR register. Only two modes available
 *
 * \param busWidth  MCI bus width mode. 00: 1-bit, 10: 4-bit.
 */
uint32_t SD_SetBusWidth( uint32_t busWidth)
{
  uint32_t mciSdcr;

  if( (busWidth != HSMCI_SDCR_SDCBUS_1) && (busWidth != HSMCI_SDCR_SDCBUS_4) )
  {
  	return (uint32_t)-1;
	}

  busWidth &= HSMCI_SDCR_SDCBUS ;

	mciSdcr = (HSMCI->HSMCI_SDCR & ~(uint32_t)(HSMCI_SDCR_SDCBUS));
	HSMCI->HSMCI_SDCR = mciSdcr | busWidth;

	return 0;
}

/**
 * Configure the MCI_CFG to enable the HS mode
 * \param hsEnable 1 to enable, 0 to disable HS mode.
 */
void SD_EnableHsMode( uint8_t hsEnable)
{
	uint32_t cfgr;

	cfgr = HSMCI->HSMCI_CFG;
	if (hsEnable)
	{
	  cfgr |=  HSMCI_CFG_HSMODE;
	}
	else
	{
	  cfgr &= ~(uint32_t)HSMCI_CFG_HSMODE;
	}

	HSMCI->HSMCI_CFG = cfgr;
}


/**
 * Configure the  MCI CLKDIV in the MCI_MR register. The max. for MCI clock is
 * MCK/2 and corresponds to CLKDIV = 0
 * \param mciSpeed  MCI clock speed in Hz, 0 will not change current speed.
 * \return The actual speed used, 0 for fail.
 */
uint32_t SD_SetSpeed( uint32_t mciSpeed )
{
	uint32_t mciMr;
	uint32_t clkdiv;

	mciMr = HSMCI->HSMCI_MR & (~(uint32_t)HSMCI_MR_CLKDIV);
	/* Multimedia Card Interface clock (MCCK or MCI_CK) is Master Clock (MCK)
	 * divided by (2*(CLKDIV+1))
	 * mciSpeed = MCK / (2*(CLKDIV+1)) */
	if (mciSpeed > 0)
	{
	  clkdiv = (Master_frequency / 2 / mciSpeed);
	  /* Speed should not bigger than expired one */
	  if (mciSpeed < Master_frequency/2/clkdiv)
	  {
	      clkdiv++;
	  }
		
	  if ( clkdiv > 0 )
	  {
	      clkdiv -= 1;
	  }
	}
	else
	{
		clkdiv = 0 ;
	}

	/* Actual MCI speed */
	mciSpeed = Master_frequency / 2 / (clkdiv + 1);
	/* Modify MR */
	HSMCI->HSMCI_MR = mciMr | clkdiv;

	return (mciSpeed);
}

//void SD_SetBlklen( uint32_t blklen )
//{
//	uint32_t mciMr;
//	uint32_t clkdiv;

//	mciMr = HSMCI->HSMCI_MR & (~(uint32_t)HSMCI_MR_BLKLEN);
//	/* Modify MR */
//	HSMCI->HSMCI_MR = mciMr | blklen << 16 ;
//}


/**
 * Reset MCI HW interface and disable it.
 * \param keepSettings Keep old register settings, including
 *                     _MR, _SDCR, _DTOR, _CSTOR, _DMA and _CFG.
 */
void SD_Reset( uint8_t keepSettings)
{
  Hsmci *pMciHw = HSMCI ;
	
  if (keepSettings)
  {
    uint32_t mr, sdcr, dtor, cstor;
    uint32_t cfg;
    mr    = pMciHw->HSMCI_MR;
    sdcr  = pMciHw->HSMCI_SDCR;
    dtor  = pMciHw->HSMCI_DTOR;
    cstor = pMciHw->HSMCI_CSTOR;
    cfg   = pMciHw->HSMCI_CFG;
		pMciHw->HSMCI_CR = HSMCI_CR_SWRST ;		// Reset
		pMciHw->HSMCI_CR = HSMCI_CR_MCIDIS;		// Disable
    pMciHw->HSMCI_MR    = mr;
    pMciHw->HSMCI_SDCR  = sdcr;
    pMciHw->HSMCI_DTOR  = dtor;
    pMciHw->HSMCI_CSTOR = cstor;
    pMciHw->HSMCI_CFG   = cfg;
  }
  else
  {
		pMciHw->HSMCI_CR = HSMCI_CR_SWRST ;		// Reset
	  pMciHw->HSMCI_CR = HSMCI_CR_MCIDIS;		// Disable
  }
}

void init_SDcard()
{
	configure_pins( 0xFC000000, PIN_PERIPHERAL | PIN_INPUT | PIN_PER_C | PIN_PORTA | PIN_NO_PULLUP ) ;
	configure_pins( PIO_PB7, PIN_INPUT | PIN_PORTB | PIN_NO_PULLUP | PIN_NO_PULLDOWN ) ;
	SD_Init() ;
}



void sd_cmd55()
{
	uint32_t i ;
  Hsmci *phsmci = HSMCI ;
		
	if ( CardIsConnected() )
	{
		phsmci->HSMCI_ARGR = Sd_rca ;
		phsmci->HSMCI_CMDR = 0x00001077 ;

		for ( i = 0 ; i < 30000 ; i += 1 )
		{
			if ( phsmci->HSMCI_SR & HSMCI_SR_CMDRDY )
			{
				break ;				
			}			
		}
//		Cmd_55_resp = phsmci->HSMCI_RSPR[0] ;
	}
}


uint32_t sd_cmd0()
{
	uint32_t i ;
  Hsmci *phsmci = HSMCI ;

	if ( CardIsConnected() )
	{
		phsmci->HSMCI_ARGR = 0 ;
		phsmci->HSMCI_CMDR = 0x00001000 ;

		for ( i = 0 ; i < 30000 ; i += 1 )
		{
			if ( phsmci->HSMCI_SR & HSMCI_SR_CMDRDY )
			{
				break ;				
			}			
		}
		return phsmci->HSMCI_RSPR[0] ;
	}
	else
	{
		return 0 ;
	}
}

uint32_t sd_cmd8()
{
	uint32_t i ;
  Hsmci *phsmci = HSMCI ;

	if ( CardIsConnected() )
	{
		phsmci->HSMCI_ARGR = 0X000001AA ;
		phsmci->HSMCI_CMDR = 0x00001048 ;

		for ( i = 0 ; i < 30000 ; i += 1 )
		{
			if ( phsmci->HSMCI_SR & HSMCI_SR_CMDRDY )
			{
				break ;				
			}			
		}
		return Cmd_8_resp = phsmci->HSMCI_RSPR[0] ;
	}
	else
	{
		return 0 ;
	}
}


uint32_t sd_acmd41()
{
	uint32_t i ;
  Hsmci *phsmci = HSMCI ;

	if ( CardIsConnected() )
	{
		sd_cmd55() ;
		phsmci->HSMCI_ARGR = 0X403F8000 ;
//		phsmci->HSMCI_ARGR = 0X40000000 ;
		phsmci->HSMCI_CMDR = 0x00001069 ;

		for ( i = 0 ; i < 30000 ; i += 1 )
		{
			if ( phsmci->HSMCI_SR & HSMCI_SR_CMDRDY )
			{
				break ;				
			}			
		}
		return Cmd_A41_resp = phsmci->HSMCI_RSPR[0] ;
	}
	else
	{
		return 0 ;
	}
}

// Get Card ID
uint32_t sd_cmd2()
{
	uint32_t i ;
  Hsmci *phsmci = HSMCI ;

	if ( CardIsConnected() )
	{
		phsmci->HSMCI_ARGR = 0 ;
		phsmci->HSMCI_CMDR = 0x00001082 ;

		for ( i = 0 ; i < 30000 ; i += 1 )
		{
			if ( phsmci->HSMCI_SR & HSMCI_SR_CMDRDY )
			{
				break ;				
			}			
		}
		Sd_128_resp[0] = phsmci->HSMCI_RSPR[0] ;
		Sd_128_resp[1] = phsmci->HSMCI_RSPR[1] ;
		Sd_128_resp[2] = phsmci->HSMCI_RSPR[2] ;
		Sd_128_resp[3] = phsmci->HSMCI_RSPR[3] ;
		return 1 ;
	}
	else
	{
		return 0 ;
	}
}

// Get new RCA
uint32_t sd_cmd3()
{
	uint32_t i ;
  Hsmci *phsmci = HSMCI ;

	if ( CardIsConnected() )
	{
		phsmci->HSMCI_ARGR = 0 ;
		phsmci->HSMCI_CMDR = 0x00001043 ;

		for ( i = 0 ; i < 30000 ; i += 1 )
		{
			if ( phsmci->HSMCI_SR & HSMCI_SR_CMDRDY )
			{
				break ;				
			}			
		}
		return phsmci->HSMCI_RSPR[0] ;
	}
	else
	{
		return 0 ;
	}
}

// Get CSD
uint32_t sd_cmd9()
{
	uint32_t i ;
  Hsmci *phsmci = HSMCI ;

	if ( CardIsConnected() )
	{
		phsmci->HSMCI_ARGR = Sd_rca ;
		phsmci->HSMCI_CMDR = 0x00001089 ;

		for ( i = 0 ; i < 30000 ; i += 1 )
		{
			if ( phsmci->HSMCI_SR & HSMCI_SR_CMDRDY )
			{
				break ;				
			}			
		}
		Sd_128_resp[0] = phsmci->HSMCI_RSPR[0] ;
		Sd_128_resp[1] = phsmci->HSMCI_RSPR[1] ;
		Sd_128_resp[2] = phsmci->HSMCI_RSPR[2] ;
		Sd_128_resp[3] = phsmci->HSMCI_RSPR[3] ;
		return 1 ;
	}
	else
	{
		return 0 ;
	}
}

// Select Card
uint32_t sd_cmd7()
{
	uint32_t i ;
  Hsmci *phsmci = HSMCI ;

	if ( CardIsConnected() )
	{
		phsmci->HSMCI_ARGR = Sd_rca ;
		phsmci->HSMCI_CMDR = 0x00001047 ;

		for ( i = 0 ; i < 30000 ; i += 1 )
		{
			if ( phsmci->HSMCI_SR & HSMCI_SR_CMDRDY )
			{
				break ;				
			}			
		}
		return 1 ;
	}
	else
	{
		return 0 ;
	}
}

#define HSMCI_CMDR_SPCMD_STD						( 0 << 8 )
#define HSMCI_CMDR_RSPTYP_48_BIT				( 1 << 6 )
#define HSMCI_CMDR_TRCMD_START_DATA     ( 1 << 16 )
#define HSMCI_CMDR_TRDIR_READ						( 1 << 18 )
#define HSMCI_CMDR_TRTYP_SINGLE					( 0 << 19 )

#define SD_SEND_SCR  (51 | HSMCI_CMDR_SPCMD_STD | HSMCI_CMDR_RSPTYP_48_BIT \
                         | HSMCI_CMDR_TRCMD_START_DATA | HSMCI_CMDR_TRDIR_READ \
                         | HSMCI_CMDR_TRTYP_SINGLE | HSMCI_CMDR_MAXLAT)

#define SD_READ_SINGLE_BLOCK     (17 | HSMCI_CMDR_SPCMD_STD | HSMCI_CMDR_RSPTYP_48_BIT \
                                     | HSMCI_CMDR_TRCMD_START_DATA | HSMCI_CMDR_TRDIR_READ \
																		 | HSMCI_CMDR_TRTYP_SINGLE | HSMCI_CMDR_MAXLAT)

// Get SCR
uint32_t sd_acmd51( uint32_t *presult )
{
	uint32_t i ;
	uint32_t j = 0 ;
  Hsmci *phsmci = HSMCI ;

	if ( CardIsConnected() )
	{
		sd_cmd55() ;
		// Block size = 64/ 8, nblocks = 1
		phsmci->HSMCI_BLKR = ( ( 64 / 8 ) << 16 ) | 1 ;
		phsmci->HSMCI_ARGR = 0 ;
		phsmci->HSMCI_CMDR = SD_SEND_SCR ;

		for ( i = 0 ; i < 50000 ; i += 1 )
		{
			if ( phsmci->HSMCI_SR & HSMCI_SR_RXRDY )
			{
				*presult++ = __REV(phsmci->HSMCI_RDR) ;
				j += 0x10000000 ;
			}
			if ( ( phsmci->HSMCI_SR & ( HSMCI_SR_CMDRDY | HSMCI_SR_XFRDONE ) ) == ( HSMCI_SR_CMDRDY | HSMCI_SR_XFRDONE ) )
			{
				break ;				
			}
						 
		}
		if ( i >= 50000 )
		{
			*presult = phsmci->HSMCI_SR ;
		}
		return i | j ; //phsmci->HSMCI_RSPR[0] ;
	}
	else
	{
		return 0 ;
	}
}

#define SD_SET_BUS_WIDTH            (6 | HSMCI_CMDR_SPCMD_STD | HSMCI_CMDR_RSPTYP_48_BIT \
                                       | HSMCI_CMDR_MAXLAT)

// Set bus width to 4 bits, set speed to 9 MHz
uint32_t sd_acmd6()
{
	uint32_t i ;
  Hsmci *phsmci = HSMCI ;

	if ( CardIsConnected() )
	{
		sd_cmd55() ;
		phsmci->HSMCI_ARGR = 2 ;		// Bus width 4 bits
		phsmci->HSMCI_CMDR = SD_SET_BUS_WIDTH ;

		for ( i = 0 ; i < 300000 ; i += 1 )
		{
			if ( phsmci->HSMCI_SR & HSMCI_SR_CMDRDY )
			{
				break ;				
			}
		}
		if ( i >= 300000 )
		{
			return 0 ;
		}
		SD_SetBusWidth( HSMCI_SDCR_SDCBUS_4 ) ;
		SD_SetSpeed( 9000000 ) ;
		return i ; //phsmci->HSMCI_RSPR[0] ;
	}
	else
	{
		return 0 ;
	}
}


// Read SD card from (byte) address, for 512 bytes
uint32_t sd_cmd17( uint32_t address, uint32_t *presult )
{
	uint32_t i ;
	uint32_t j = 0 ;
  Hsmci *phsmci = HSMCI ;

	if ( CardIsConnected() )
	{
		// Block size = 512, nblocks = 1
		phsmci->HSMCI_BLKR = ( ( 512 ) << 16 ) | 1 ;
		phsmci->HSMCI_ARGR = address ;
		phsmci->HSMCI_CMDR = SD_READ_SINGLE_BLOCK ;

		for ( i = 0 ; i < 50000 ; i += 1 )
		{
			if ( phsmci->HSMCI_SR & HSMCI_SR_RXRDY )
			{
				*presult++ = phsmci->HSMCI_RDR ;
				j += 1 ;
			}
			if ( ( phsmci->HSMCI_SR & ( HSMCI_SR_CMDRDY | HSMCI_SR_XFRDONE ) ) == ( HSMCI_SR_CMDRDY | HSMCI_SR_XFRDONE ) )
			{
				break ;				
			}
			if ( j >= 128 )
 			{
				break ;				
			}
		}
		return i | (j << 16) ; //phsmci->HSMCI_RSPR[0] ;
	}
	else
	{
		return 0 ;
	}
}

// This routine is called every 10 mS, and checks for card
// insertion and removal.
// When a card is inserted, it initialises it ready for read/write access.
void sd_poll_10mS()
{
	uint32_t i ;

	if ( !CardIsConnected() )
	{
		Card_state = SD_ST_EMPTY ;
		Sd_rca = 0 ;
		Card_ID[0] = 0 ;
		Card_ID[1] = 0 ;
		Card_ID[2] = 0 ;
		Card_ID[3] = 0 ;
		Card_CSD[0] = 0 ;
		Card_CSD[1] = 0 ;
		Card_CSD[2] = 0 ;
		Card_CSD[3] = 0 ;
		Card_SCR[0] = 0 ;
		Card_SCR[1] = 0 ;
	}

	switch ( Card_state )
	{
		case SD_ST_EMPTY :
			if ( CardIsConnected() )
			{
				Card_state = SD_ST_INIT1 ;
			}
		break ;

		case SD_ST_INIT1 :
			
			i = sd_cmd0() ;
			
			Card_state = SD_ST_INIT2 ;
			
		break ;
		
		case SD_ST_INIT2 :
			i = sd_cmd8() ;
			
			Card_state = SD_ST_IDLE ;
			
		break ;

		case SD_ST_IDLE :
			i = sd_acmd41() ;
			if ( i & 0x80000000 )
			{
				Card_state = SD_ST_READY ;
			}
		break ;

		case SD_ST_READY :
			i = sd_cmd2() ;
			if ( i )
			{
				Card_ID[0] = Sd_128_resp[0] ;
				Card_ID[1] = Sd_128_resp[1] ;
				Card_ID[2] = Sd_128_resp[2] ;
				Card_ID[3] = Sd_128_resp[3] ;
				Card_state = SD_ST_IDENT ;
			}
		break ;

		case SD_ST_IDENT :
			i = sd_cmd3() ;		// Get new RCA
			Sd_rca = i ;
			Card_state = SD_ST_STBY ;
		break ;

		case SD_ST_STBY :
			i = sd_cmd9() ;
			Card_CSD[0] = Sd_128_resp[0] ;
			Card_CSD[1] = Sd_128_resp[1] ;
			Card_CSD[2] = Sd_128_resp[2] ;
			Card_CSD[3] = Sd_128_resp[3] ;
			i = sd_cmd7() ;		// Select Card
//			txmit( '+' ) ;
//			p8hex( i ) ;
//			crlf() ;
			Card_state = SD_ST_TRAN ;
		break ;

		case SD_ST_TRAN :
			i = sd_acmd51( Sd_128_resp ) ;
			Card_SCR[0] = Sd_128_resp[0] ;
			Card_SCR[1] = Sd_128_resp[1] ;
			Card_state = SD_ST_DATA ;
			i = sd_acmd6() ;		// Set bus width to 4 bits, and speed to 9 MHz
//			txmit( '-' ) ;
//			p8hex( i ) ;
//			crlf() ;
			// Should check the card can do this ****
		break ;
	}		
}

// Checks for card ready for read/write
// returns 1 for YES, 0 for NO
uint32_t sd_card_ready( void )
{
	if ( CardIsConnected() )
	{
		if ( Card_state == SD_ST_DATA )
		{
			return 1 ;
		}
	}
	return 0 ;
}

uint32_t sd_read_block( uint32_t block_no, uint32_t *data )
{
	if ( Card_state == SD_ST_DATA )
	{
		return sd_cmd17( block_no << 9, data ) ;
	}
	else
	{
		return 0 ;
	}	 
}


/*
 Notes on SD card:

1) CMD8 fails and CMD58 fails: must be MMC, thus initialize using CMD1
2) CMD8 fails and CMD58 passes: must be Ver1.x Standard Capacity SD Memory Card
3) CMD8 passes and CMD58 passes (CCS = 0): must be Ver2.00 or later Standard Capacity SD Memory Card
4) CMD8 passes and CMD58 passes (CCS = 1): must be Ver2.00 or later High Capacity SD Memory Card
5) CMD8 passes and CMD58 passes but indicates non compatible voltage range: unusable card 

On card present
1. Send CMD 55 (resp 48bit)
		followed by ACMD41 (resp48bit)
		until bit 31 of response is '1'

2. Send CMD 2 (resp 136bit) - card ID

3. Send CMD 3 (resp 48 bit) - new RCA returned

4. Send CMD 9 (resp 136) - CSD

5. Send CMD 7 (resp 48) - 

6. Send CMD 55 (resp 48bit)
		followed by ACMD51 (resp48bit) - SCR stored as 2 32 bit values

Now decide what the card can do!

7. Set block length

8. Set bus width

9. Read block 0

*/


#include "ff.h"
#include "diskio.h"

/* FAT sub-type boundaries */
/* Note that the FAT spec by Microsoft says 4085 but Windows works with 4087! */
#define MIN_FAT16	4086	/* Minimum number of clusters for FAT16 */
#define	MIN_FAT32	65526	/* Minimum number of clusters for FAT32 */


/* FatFs refers the members in the FAT structures as byte array instead of
/ structure member because there are incompatibility of the packing option
/ between compilers. */

#define BS_jmpBoot			0
#define BS_OEMName			3
#define BPB_BytsPerSec		11
#define BPB_SecPerClus		13
#define BPB_RsvdSecCnt		14
#define BPB_NumFATs			16
#define BPB_RootEntCnt		17
#define BPB_TotSec16		19
#define BPB_Media			21
#define BPB_FATSz16			22
#define BPB_SecPerTrk		24
#define BPB_NumHeads		26
#define BPB_HiddSec			28
#define BPB_TotSec32		32
#define BS_DrvNum			36
#define BS_BootSig			38
#define BS_VolID			39
#define BS_VolLab			43
#define BS_FilSysType		54
#define BPB_FATSz32			36
#define BPB_ExtFlags		40
#define BPB_FSVer			42
#define BPB_RootClus		44
#define BPB_FSInfo			48
#define BPB_BkBootSec		50
#define BS_DrvNum32			64
#define BS_BootSig32		66
#define BS_VolID32			67
#define BS_VolLab32			71
#define BS_FilSysType32		82
#define	FSI_LeadSig			0
#define	FSI_StrucSig		484
#define	FSI_Free_Count		488
#define	FSI_Nxt_Free		492
#define MBR_Table			446
#define BS_55AA				510

#define	DIR_Name			0
#define	DIR_Attr			11
#define	DIR_NTres			12
#define	DIR_CrtTime			14
#define	DIR_CrtDate			16
#define	DIR_FstClusHI		20
#define	DIR_WrtTime			22
#define	DIR_WrtDate			24
#define	DIR_FstClusLO		26
#define	DIR_FileSize		28
#define	LDIR_Ord			0
#define	LDIR_Attr			11
#define	LDIR_Type			12
#define	LDIR_Chksum			13
#define	LDIR_FstClusLO		26



// Low level for fat processing

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
				   BYTE drv,			/* Physical drive nmuber (0) */
				   BYTE *buff,			/* Pointer to the data buffer to store read data */
				   DWORD sector,		/* Start sector number (LBA) */
				   BYTE count			/* Sector count (1..255) */
				   )
{
	uint32_t result ;
	if (drv || !count) return RES_PARERR;

	if ( sd_card_ready() == 0 ) return RES_NOTRDY;
	
	if (count == 1)
	{	/* Single block read */
		result = sd_read_block( sector, ( uint32_t *)buff ) ;
		if ( result )
		{
			count = 0 ;
		}
//			if ((send_cmd(CMD17, sector) == 0)	/* READ_SINGLE_BLOCK */
//			&& rcvr_datablock(buff, 512))
//			count = 0;
	}
	else
	{				/* Multiple block read */
		do
		{
			result = sd_read_block( sector << 9, ( uint32_t *)buff ) ;
			if ( result )
			{
				sector += 1 ;
				buff += 512 ;
				count -= 1 ;
			}
			else
			{
				count = 1 ;		// Flag error
				break ;
			}
		}	while ( count ) ;
	}
	return count ? RES_ERROR : RES_OK;
}


// FAT processing - move to it's own file later

/*-----------------------------------------------------------------------*/
/* Load boot record and check if it is an FAT boot record                */
/*-----------------------------------------------------------------------*/

//static
BYTE check_fs (	/* 0:The FAT BR, 1:Valid BR but not an FAT, 2:Not a BR, 3:Disk error */
	FATFS *fs,	/* File system object */
	DWORD sect	/* Sector# (lba) to check if it is an FAT boot record or not */
)
{
	if (disk_read(fs->drv, fs->win, sect, 1) != RES_OK)	/* Load boot record */
		return 3;
	if (LD_WORD(&fs->win[BS_55AA]) != 0xAA55)		/* Check record signature (always placed at offset 510 even if the sector size is >512) */
		return 2;

	if ((LD_DWORD(&fs->win[BS_FilSysType]) & 0xFFFFFF) == 0x544146)	/* Check "FAT" string */
		return 0;
	if ((LD_DWORD(&fs->win[BS_FilSysType32]) & 0xFFFFFF) == 0x544146)
		return 0;

	return 1;
}








