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
#ifndef SIMU
#include "core_cm3.h"
#endif

#include "ersky9x.h"
#include "myeeprom.h"
#include "drivers.h"
#include "logicio.h"
#include "lcd.h"
#include "debug.h"
#ifndef SIMU
#include "CoOS.h"
#endif

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


#define RX_UART_BUFFER_SIZE	32

struct t_rxUartBuffer
{
	uint8_t fifo[RX_UART_BUFFER_SIZE] ;
	uint8_t *outPtr ;
} ;

struct t_rxUartBuffer TelemetryInBuffer[2] ;
uint32_t TelemetryActiveBuffer ;

struct t_fifo32 Console_fifo ;
struct t_fifo32 BtRx_fifo ;


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



volatile uint16_t g_tmr10ms;
volatile uint8_t  g_blinkTmr10ms;


void per10ms()
{
	register uint32_t i ;

  g_tmr10ms++;
  g_blinkTmr10ms++;

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

#ifdef PCBSKY
#if !defined(SIMU)
	keys[enuk].input( ~PIOB->PIO_PDSR & 0x40,(EnumKeys)enuk); // Rotary Enc. Switch
#endif

#endif
}


void put_fifo32( struct t_fifo32 *pfifo, uint8_t byte )
{
	pfifo->fifo[pfifo->in] = byte ;
#ifndef SIMU
	__disable_irq() ;
#endif
	pfifo->count += 1 ;
#ifndef SIMU
		__enable_irq() ;
#endif
	pfifo->in = ( pfifo->in + 1) & 0x1F ;
}

int32_t get_fifo32( struct t_fifo32 *pfifo )
{
	int32_t rxbyte ;
	if ( pfifo->count )						// Look for char available
	{
		rxbyte = pfifo->fifo[pfifo->out] ;
#ifndef SIMU
	__disable_irq() ;
#endif
		pfifo->count -= 1 ;
#ifndef SIMU
		__enable_irq() ;
#endif
		pfifo->out = ( pfifo->out + 1 ) & 0x1F ;
		return rxbyte ;
	}
	return -1 ;

	
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



#ifdef PCBSKY
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


uint32_t spi_PDC_action( register uint8_t *command, register uint8_t *tx, register uint8_t *rx, register uint32_t comlen, register uint32_t count )
{
#ifndef SIMU
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
#endif
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
  pUart->UART_IER = UART_IER_RXRDY ;

	NVIC_EnableIRQ(UART0_IRQn) ;

}

void UART_Stop()
{
  CONSOLE_USART->UART_IDR = UART_IDR_RXRDY ;
	NVIC_DisableIRQ(UART0_IRQn) ;
}


extern "C" void UART0_IRQHandler()
{
	put_fifo32( &Console_fifo, CONSOLE_USART->UART_RHR ) ;	
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
  pUart->UART_BRGR = ( (masterClock / baudrate) + 8 ) / 16;
  
// Following only available for USARTS
//	baudrate = (masterClock * 8 / baudrate) / 16 ;
//  pUart->UART_BRGR = ( baudrate / 8 ) | ( ( baudrate & 7 ) << 16 ) ;	// Fractional part to allow 115200 baud

  /* Disable PDC channel */
  pUart->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;

  /* Enable receiver and transmitter */
  pUart->UART_CR = UART_CR_RXEN | UART_CR_TXEN;
  pUart->UART_IER = UART_IER_RXRDY ;
	NVIC_EnableIRQ(UART1_IRQn) ;

}

void Bt_UART_Stop()
{
  BT_USART->UART_IDR = UART_IDR_RXRDY ;
	NVIC_DisableIRQ(UART1_IRQn) ;
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
#ifndef SIMU
  // TODO because of the 64bits cast ...
	pUsart->US_RPR = (uint32_t)TelemetryInBuffer[0].fifo ;
	pUsart->US_RNPR = (uint32_t)TelemetryInBuffer[1].fifo ;
#endif
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
#if !defined(SIMU)
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
#endif
}

uint32_t txPdcUsart( uint8_t *buffer, uint32_t size )
{
  register Usart *pUsart = SECOND_USART;

	if ( pUsart->US_TNCR == 0 )
	{
#ifndef SIMU
	  pUsart->US_TNPR = (uint32_t)buffer ;
#endif
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


struct t_serial_tx *Current_bt ;

uint32_t txPdcBt( struct t_serial_tx *data )
{
  Uart *pUart=BT_USART ;
		
	if ( pUart->UART_TNCR == 0 )
	{
		Current_bt = data ;
		data->ready = 1 ;
#ifndef SIMU
	  pUart->UART_TPR = (uint32_t)data->buffer ;
#endif
		pUart->UART_TCR = data->size ;
		pUart->UART_PTCR = US_PTCR_TXTEN ;
		pUart->UART_IER = UART_IER_TXBUFE ;
		NVIC_EnableIRQ(UART1_IRQn) ;
		return 1 ;			// Sent OK
	}
	return 0 ;				// Busy
}

void end_bt_tx_interrupt()
{
  Uart *pUart=BT_USART ;
	pUart->UART_IDR = UART_IDR_TXBUFE ;
	NVIC_DisableIRQ(UART1_IRQn) ;
}

extern "C" void UART1_IRQHandler()
{
  Uart *pUart=BT_USART ;
	if ( pUart->UART_SR & UART_SR_TXBUFE )
	{
		pUart->UART_IDR = UART_IDR_TXBUFE ;
		pUart->UART_PTCR = US_PTCR_TXTDIS ;
		Current_bt->ready = 0 ;	
	}
	if ( pUart->UART_SR & UART_SR_RXRDY )
	{
		put_fifo32( &BtRx_fifo, pUart->UART_RHR ) ;	
	}
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
	return get_fifo32( &Console_fifo ) ;
  
//	Uart *pUart=CONSOLE_USART ;

//  if (pUart->UART_SR & UART_SR_RXRDY)
//	{
//		return pUart->UART_RHR ;
//	}
//	return 0xFFFF ;
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
	return get_fifo32( &BtRx_fifo ) ;
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
// TRANSFER = 3
// TRACKTIM = 15 (16 clock periods)
// ANACH = 1
// SETTLING = 6 (not used if ANACH = 0)
// STARTUP = 6 (96 clock periods)
// PRESCAL = 9.0 MHz clock (between 1 and 20MHz)
// FREERUN = 0
// FWUP = 0
// SLEEP = 0
// LOWRES = 0
// TRGSEL = 0
// TRGEN = 0 (software trigger only)
// Gain/offset channels:
// Stick LV AD9  Gain:0x00080000 Offset:0x00000200
// Stick LH AD2  Gain:0x00000020 Offset:0x00000004
// Stick RV AD14 Gain:0x20000000 Offset:0x00004000
// Stick RH AD1  Gain:0x00000008 Offset:0x00000002
// Gains in ADC_CGR, offsets in ADC_COR

#define GAIN_LV			0x00080000
#define GAIN_LH	    0x00000020
#define GAIN_RV	    0x20000000
#define GAIN_RH	    0x00000008

#define OFF_LV			0x00000200
#define OFF_LH      0x00000004
#define OFF_RV      0x00004000
#define OFF_RH      0x00000002

void init_adc()
{
	register Adc *padc ;
	register uint32_t timer ;

	timer = ( Master_frequency / (9000000*2) - 1 ) << 8 ;
	// Enable peripheral clock ADC = bit 29
  PMC->PMC_PCER0 |= 0x20000000L ;		// Enable peripheral clock to ADC
	padc = ADC ;
	padc->ADC_MR = 0x3FB60000 | timer ;  // 0011 1111 1011 0110 xxxx xxxx 0000 0000
	padc->ADC_ACR = ADC_ACR_TSON ;			// Turn on temp sensor
#ifdef REVB
	padc->ADC_CHER = 0x0000E33E ;  // channels 1,2,3,4,5,8,9,13,14,15
#else
	padc->ADC_CHER = 0x0000E23E ;  // channels 1,2,3,4,5,9,13,14,15
#endif
	padc->ADC_CGR = 0 ;  // Gain = 1, all channels
	padc->ADC_COR = 0 ;  // Single ended, 0 offset, all channels

//	padc->ADC_CGR = GAIN_LV | GAIN_LH | GAIN_RV | GAIN_RH ;
//	padc->ADC_COR = OFF_LV | OFF_LH | OFF_RV | OFF_RH ;
}

void set_stick_gain( uint32_t gains )
{
	register Adc *padc ;
	uint32_t gain ;
	uint32_t offset ;

	gain = 0 ;
	offset = 0 ;
	padc = ADC ;

	if ( gains & STICK_LV_GAIN )
	{
		gain |= GAIN_LV ;
		offset |= OFF_LV ;
	}
	if ( gains & STICK_LH_GAIN )
	{
		gain |= GAIN_LH ;
		offset |= OFF_LH ;
	}
	if ( gains & STICK_RV_GAIN )
	{
		gain |= GAIN_RV ;
		offset |= OFF_RV ;
	}
	if ( gains & STICK_RH_GAIN )
	{
		gain |= GAIN_RH ;
		offset |= OFF_RH ;
	}

	padc->ADC_CGR = gain ;
	padc->ADC_COR = offset ;

}



// Start TIMER3 for input capture
void start_timer3()
{
#ifndef SIMU
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
	NVIC_SetPriority( TC3_IRQn, 14 ) ; // Low priority interrupt
	NVIC_EnableIRQ(TC3_IRQn) ;
	ptc->TC_CHANNEL[0].TC_IER = TC_IER0_LDRAS ;
#endif
}

// Start Timer4 to provide 0.5uS clock for input capture
void start_timer4()
{
#ifndef SIMU
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
#endif
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


extern "C" void TC3_IRQHandler() //capture ppm in at 2MHz
{
  uint16_t capture ;
  static uint16_t lastCapt ;
  uint16_t val ;
	
	capture = TC1->TC_CHANNEL[0].TC_RA ;
	(void) TC1->TC_CHANNEL[0].TC_SR ;		// Acknowledgethe interrupt
  

  val = (uint16_t)(capture - lastCapt) / 2 ;
  lastCapt = capture;

  // We prcoess g_ppmInsright here to make servo movement as smooth as possible
  //    while under trainee control
  if (val>4000 && val < 19000) // G: Prioritize reset pulse. (Needed when less than 8 incoming pulses)
    ppmInState = 1; // triggered
  else
  {
  	if(ppmInState && ppmInState<=8)
		{
    	if(val>800 && val<2200)
			{
  	    g_ppmIns[ppmInState++ - 1] =
    	    (int16_t)(val - 1500)*(g_eeGeneral.PPM_Multiplier+10)/10; //+-500 != 512, but close enough.

	    }else{
  	    ppmInState=0; // not triggered
    	}
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

#endif

