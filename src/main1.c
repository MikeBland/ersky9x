/****************************************************************************
*  Copyright (c) 2011 by Michael Fischer. All rights reserved.
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
*  09.04.2011  mifi  First Version
****************************************************************************/
#define __MAIN_C__

#include <stdint.h>
#include "AT91SAM3S2.h"

extern uint32_t Master_frequency ;

/*=========================================================================*/
/*  DEFINE: All Structures and Common Constants                            */
/*=========================================================================*/

/*=========================================================================*/
/*  DEFINE: Prototypes                                                     */
/*=========================================================================*/

void UART_Configure( uint32_t baudrate, uint32_t masterClock) ;
void UART_PutChar( uint8_t c ) ;
void start_timer( void ) ;
void sam_boot( void ) ;

/*=========================================================================*/
/*  DEFINE: Definition of all local Data                                   */
/*=========================================================================*/

/*=========================================================================*/
/*  DEFINE: All code exported                                              */
/*=========================================================================*/

/***************************************************************************/
/*  main                                                                   */
/***************************************************************************/
int main (void)
{
	register uint32_t i ;
	register Pio *pioptr ;

	WDT->WDT_MR = 0x3FFFAFFF ;			// Disable watchdog
	
  PMC->PMC_PCER0 = 0x3900 ;				// Enable clocks to PIOB and PIOA and PIOC and UART0
	pioptr = PIOA ;
	pioptr->PIO_PER = 0x00200000L ;		// Enable bit A21 (EXT3)
	pioptr->PIO_OER = 0x00200000L ;		// Set bit A21 as output
	pioptr->PIO_SODR = 0x00200000L ;	// Set bit A21  ON
	
	pioptr->PIO_PUER = 0x80000000 ;		// Enable pullup on bit A31 (EXIT)
	pioptr->PIO_PER = 0x80000000 ;		// Enable bit A31

	pioptr = PIOC ;
	pioptr->PIO_PER = 0x82040000L ;		// Enable bit C31 (EXT1), C25 (USB-detect), C18 (Backlight)
	pioptr->PIO_OER = 0x80040000L ;		// Set bits C18 and C31 as output
	pioptr->PIO_SODR = 0x80000000L ;	// Set bit C31

	pioptr = PIOB ;
	pioptr->PIO_PUER = 0x40 ;					// Enable pullup on bit B6 (MENU)
	pioptr->PIO_PER = 0x40 ;					// Enable bit B6

  // Enable PCK2 on PB3
	pioptr->PIO_ABCDSR[0] |=  0x00000008 ;	// Peripheral B
  pioptr->PIO_ABCDSR[1] &= ~0x00000008 ;	// Peripheral B
  pioptr->PIO_PDR = 0x00000008 ;					// Assign to peripheral
	PMC->PMC_SCER |= 0x0400 ;								// PCK2 enabled
	PMC->PMC_PCK[2] = 2 ;										// PCK2 is PLLA

	UART_Configure( 9600, Master_frequency ) ;

	pioptr = PIOC ;
//	start_timer() ;

	i = 0 ;    
  while (1)
  {
//	  PMC->PMC_PCER0 = 0x1800 ;				// Enable clocks to PIOB and PIOA
//		PIOA->PIO_PER = 0x04200000L ;		// Enable bit A21 (EXT3), A26 (EXT2)
//		PIOA->PIO_OER = 0x04200000L ;		// Set bits A21, A26 as output

//		PIOB->PIO_PUER = 0x40 ;					// Enable pullup on bit B6
//		PIOB->PIO_PER = 0x40 ;					// Enable bit B6
//		PIOA->PIO_PUER = 0x80000000 ;		// Enable pullup on bit A31 (EXIT)
//		PIOA->PIO_PER = 0x80000000 ;		// Enable bit A31
		
		if ( PIOB->PIO_PDSR & 0x40 )
		{
			PIOA->PIO_SODR = 0x00200000L ;	// Set bit A21 ON
		}
		else
		{
//			PIOA->PIO_SODR = 0x00200000L ;	// Set bit A21 ON
			PIOA->PIO_CODR = 0x00200000L ;	// Clear bit A21 OFF
		}
		i += 1 ;
		// Toggle a bit
		if ( i & 1 )
		{
			pioptr->PIO_CODR = 0x80040000L ;	// Set bits C18 and C31 OFF
		}
		else
		{
			pioptr->PIO_SODR = 0x80040000L ;	// Set bits C18 and C31 ON
		}
		if ( ( i & 0x07FF ) == 0x07FF )
		{
			UART_PutChar( 'x' ) ;
		}

		if ( PIOC->PIO_PDSR & 0x02000000 )
		{
			// Detected USB
			break ;
		}

		
//    a++;
//    b++;
//    c = a + b;
    
  }
	pioptr->PIO_CODR = 0x00200000L ;	// Clear bit A21 OFF

	// This might be replaced by a software reset
	sam_boot() ;			

  /*
   * Prevent compiler warnings
   */
//  (void)c;   
  
  /*
   * This return here make no sense.
   * But to prevent the compiler warning:
   * "return type of 'main' is not 'int'
   * we use an int as return :-)
   */ 
  return(0);
}


/** Console baudrate 9600. */
#define CONSOLE_BAUDRATE    9600
/** Usart Hw interface used by the console (UART0). */
#define CONSOLE_USART       UART0
/** Usart Hw ID used by the console (UART0). */
#define CONSOLE_ID          ID_UART0
/** Pins description corresponding to Rxd,Txd, (UART pins) */
#define CONSOLE_PINS        {PINS_UART}

/*----------------------------------------------------------------------------
 *        Variables
 *----------------------------------------------------------------------------*/

/**
 * \brief Configures an USART peripheral with the specified parameters.
 *
 * \param baudrate  Baudrate at which the USART should operate (in Hz).
 * \param masterClock  Frequency of the system master clock (in Hz).
 */
void UART_Configure( uint32_t baudrate, uint32_t masterClock)
{
//    const Pin pPins[] = CONSOLE_PINS;
  register Uart *pUart = CONSOLE_USART;
	register Pio *pioptr ;

  /* Configure PIO */
//    PIO_Configure(pPins, PIO_LISTSIZE(pPins));
	pioptr = PIOA ;
  pioptr->PIO_ABCDSR[0] &= ~0x00000600 ;	// Peripheral A
  pioptr->PIO_ABCDSR[1] &= ~0x00000600 ;	// Peripheral A
  pioptr->PIO_PDR = 0x00000600 ;					// Assign to peripheral

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

/**
 * \brief Outputs a character on the UART line.
 *
 * \note This function is synchronous (i.e. uses polling).
 * \param c  Character to send.
 */
void UART_PutChar( uint8_t c )
{
    Uart *pUart=CONSOLE_USART ;

    /* Wait for the transmitter to be ready */
    while ( (pUart->UART_SR & UART_SR_TXEMPTY) == 0 ) ;

    /* Send character */
    pUart->UART_THR=c ;
}


void start_timer()
{
	register Pio *pioptr ;
  register Tc *ptc ;
	
	// Enable peripheral clock TC0 = bit 23 thru TC5 = bit 28
  PMC->PMC_PCER0 = 0x02000000L ;		// Enable clock to TC2
	
	pioptr = PIOA ;

  ptc = TC0 ;		// Tc block 0 (TC0-2)
	ptc->TC_BCR = 0 ;			// No sync
	ptc->TC_BMR = 0 ;			// No sync
	ptc->TC_CHANNEL[2].TC_CMR = 0x00008000 ;	// Waveform mode
	ptc->TC_CHANNEL[2].TC_RC = 9999 ;
	ptc->TC_CHANNEL[2].TC_RA = 4999 ;
	ptc->TC_CHANNEL[2].TC_CMR = 0x0009C000 ;	// 0000 0000 0000 1001 1100 0000 0000 0000

  pioptr->PIO_ABCDSR[0] &= ~0x04000000 ;	// Peripheral B = TIOA2
  pioptr->PIO_ABCDSR[1] |= 0x04000000 ;		// Peripheral B
	pioptr->PIO_PDR = 0x04000000L ;		// Disable bit A26 (EXT2) Assign to peripheral


//typedef struct {
//  RwReg      TC_CCR;        /**< \brief (TcChannel Offset: 0x0) Channel Control Register */
//  RwReg      TC_CMR;        /**< \brief (TcChannel Offset: 0x4) Channel Mode Register */
//  RwReg      Reserved1[2]; 
//  RwReg      TC_CV;         /**< \brief (TcChannel Offset: 0x10) Counter Value */
//  RwReg      TC_RA;         /**< \brief (TcChannel Offset: 0x14) Register A */
//  RwReg      TC_RB;         /**< \brief (TcChannel Offset: 0x18) Register B */
//  RwReg      TC_RC;         /**< \brief (TcChannel Offset: 0x1C) Register C */
//  RwReg      TC_SR;         /**< \brief (TcChannel Offset: 0x20) Status Register */
//  RwReg      TC_IER;        /**< \brief (TcChannel Offset: 0x24) Interrupt Enable Register */
//  RwReg      TC_IDR;        /**< \brief (TcChannel Offset: 0x28) Interrupt Disable Register */
//  RwReg      TC_IMR;        /**< \brief (TcChannel Offset: 0x2C) Interrupt Mask Register */
//  RwReg      Reserved2[4]; 
//} TcChannel;
///** \brief Tc hardware registers */
//typedef struct {
//  TcChannel  TC_CHANNEL[3]; /**< \brief (Tc Offset: 0x0) channel = 0 .. 2 */
//  WoReg      TC_BCR;        /**< \brief (Tc Offset: 0xC0) Block Control Register */
//  RwReg      TC_BMR;        /**< \brief (Tc Offset: 0xC4) Block Mode Register */
//  WoReg      TC_QIER;       /**< \brief (Tc Offset: 0xC8) QDEC Interrupt Enable Register */
//  WoReg      TC_QIDR;       /**< \brief (Tc Offset: 0xCC) QDEC Interrupt Disable Register */
//  RoReg      TC_QIMR;       /**< \brief (Tc Offset: 0xD0) QDEC Interrupt Mask Register */
//  RoReg      TC_QISR;       /**< \brief (Tc Offset: 0xD4) QDEC Interrupt Status Register */
//  RwReg      TC_FMR;        /**< \brief (Tc Offset: 0xD8) Fault Mode Register */
//  RwReg      Reserved1[2]; 
//  RwReg      TC_WPMR;       /**< \brief (Tc Offset: 0xE4) Write Protect Mode Register */
//  RwReg      Reserved2[5]; 
//  RoReg      TC_VER;        /**< \brief (Tc Offset: 0xFC) Version Register */
//} Tc;

	
}


/*** EOF ***/
