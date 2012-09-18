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

#include "AT91SAM3S4.h"
#include "board.h"
#ifndef SIMU
 	#include "core_cm3.h"
#endif
#include "sound.h"
#include "ersky9x.h"
#include "myeeprom.h"
#include "drivers.h"
#include "audio.h"


void start_sound( void ) ;
void buzzer_on( void ) ;
void buzzer_off( void ) ;
void buzzer_sound( uint8_t time ) ;
void start_timer1( void ) ;
void init_dac( void ) ;
extern "C" void DAC_IRQHandler( void ) ;
void disp_mem( register uint32_t address ) ;
void end_sound( void ) ;
void tone_start( register uint32_t time ) ;
void tone_stop( void ) ;
void init_twi( void ) ;
void set_volume( register uint8_t volume ) ;
extern "C" void TWI0_IRQHandler (void) ;
void audioDefevent( uint8_t e ) ;


extern uint32_t Master_frequency ;

volatile uint8_t Buzzer_count ;


struct t_sound_globals Sound_g ;

struct t_VoiceBuffer VoiceBuffer[3] ;

#define SOUND_NONE	0
#define SOUND_TONE	1
#define SOUND_VOICE	2
#define SOUND_STOP	3

struct t_VoiceBuffer *PtrVoiceBuffer[3] ;
uint8_t VoiceCount ;
uint8_t SoundType ;

	 
// Must NOT be in flash, PDC needs a RAM source.
//uint16_t Sine_values[] =
//{
//2048,2173,2298,2422,2545,2666,2784,2899,3011,3119,
//3223,3322,3417,3505,3589,3666,3736,3800,3857,3907,
//3950,3985,4012,4032,4044,4048,4044,4032,4012,3985,
//3950,3907,3857,3800,3736,3666,3589,3505,3417,3322,
//3223,3119,3011,2899,2784,2666,2545,2422,2298,2173,
//2048,1922,1797,1673,1550,1429,1311,1196,1084, 976,
// 872, 773, 678, 590, 506, 429, 359, 295, 238, 188,
// 145, 110,  83,  63,  51,  48,  51,  63,  83, 110,
// 145, 188, 238, 295, 359, 429, 506, 590, 678, 773,
// 872, 976,1084,1196,1311,1429,1550,1673,1797,1922
//} ;

// Amplitude reduced to 30% to allow for voice volume
uint16_t Sine_values[] =
{
2048,2085,2123,2160,2197,2233,2268,2303,2336,2369,
2400,2430,2458,2485,2510,2533,2554,2573,2590,2605,
2618,2629,2637,2643,2646,2648,2646,2643,2637,2629,
2618,2605,2590,2573,2554,2533,2510,2485,2458,2430,
2400,2369,2336,2303,2268,2233,2197,2160,2123,2085,
2048,2010,1972,1935,1898,1862,1826,1792,1758,1726,
1695,1665,1637,1610,1585,1562,1541,1522,1505,1490,
1477,1466,1458,1452,1448,1448,1448,1452,1458,1466,
1477,1490,1505,1522,1541,1562,1585,1610,1637,1665,
1695,1726,1758,1792,1826,1862,1898,1935,1972,2010
} ;


// Must NOT be in flash, PDC needs a RAM source.
// We'll use these for higher frequencies
//uint16_t Sine_values64[] =
//{
//2048,2244,2438,2628,2813,2990,3159,3316,
//3462,3594,3710,3811,3895,3961,4009,4038,
//4048,4038,4009,3961,3895,3811,3710,3594,
//3462,3316,3159,2990,2813,2628,2438,2244,
//2048,1851,1657,1467,1282,1105, 936, 779,
// 633, 501, 385, 284, 200, 134,  86,  57,
//  48,  57,  86, 134, 200, 284, 385, 501,
// 633, 779, 936,1105,1282,1467,1657,1851
//} ;

const uint16_t PianoTones[] =
{
  28,   29,   31,   33,   35,   37,   39,   41,   44,   46,
  49,   52,   55,   58,   62,   65,   69,   73,   78,   82,
  87,   92,   98,  104,  110,  117,  123,  131,  139,  147,
 156,  165,  175,  185,  196,  208,  220,  233,  247,  262, // d#, E, F, f#, G, g#, A, a#, B, C(middle)
 277,  294,  311,  330,  349,  370,  392,  415,  440,  466, // c#, D, d#, E, F, f#, G, g#, A, a#
 494,  523,  554,  587,  622,  659,  698,  740,  784,  831, // B, C, c#, D, d#, E, F, f#, G, g#
 880,  932,  988, 1047, 1109, 1175, 1245, 1319, 1397, 1480,
1568, 1661, 1760, 1865, 1976, 2093, 2217, 2349, 2489, 2637,
2794, 2960, 3136, 3322, 3520 ,3729, 3951, 4186
} ;


// Sound routines

void start_sound()
{
	register Pio *pioptr ;
	
	start_timer1() ;
	init_dac() ;
	init_twi() ;

	pioptr = PIOA ;
#ifdef REVB
	pioptr->PIO_CODR = 0x02000000L ;	// Set bit A25 OFF
	pioptr->PIO_PER = 0x02000000L ;		// Enable bit A25 (Stock buzzer)
	pioptr->PIO_OER = 0x02000000L ;		// Set bit A25 as output
#else
	pioptr->PIO_CODR = 0x00010000L ;	// Set bit A16 OFF
	pioptr->PIO_PER = 0x00010000L ;		// Enable bit A16 (Stock buzzer)
	pioptr->PIO_OER = 0x00010000L ;		// Set bit A16 as output
#endif
}

#ifdef REVB
void buzzer_on()
{
	PIOA->PIO_SODR = 0x02000000L ;	// Set bit A25 ON
}

void buzzer_off()
{
	PIOA->PIO_CODR = 0x02000000L ;	// Set bit A25 ON
}
#else
void buzzer_on()
{
	PIOA->PIO_SODR = 0x00010000L ;	// Set bit A16 ON
}

void buzzer_off()
{
	PIOA->PIO_CODR = 0x00010000L ;	// Set bit A16 ON
}
#endif

void buzzer_sound( uint8_t time )
{
	buzzer_on() ;
	Buzzer_count = time ;
}


void set_frequency( uint32_t frequency )
{
  register Tc *ptc ;
	register uint32_t timer ;

	timer = Master_frequency / (8 * frequency) ;		// MCK/8 and 100 000 Hz
	if ( timer > 65535 )
	{
		timer = 65535 ;		
	}
	if ( timer < 2 )
	{
		timer = 2 ;		
	}
	ptc = TC0 ;		// Tc block 0 (TC0-2)
	ptc->TC_CHANNEL[1].TC_CCR = TC_CCR0_CLKDIS ;		// Stop clock
	ptc->TC_CHANNEL[1].TC_RC = timer ;			// 100 000 Hz
	ptc->TC_CHANNEL[1].TC_RA = timer >> 1 ;
	ptc->TC_CHANNEL[1].TC_CCR = 5 ;		// Enable clock and trigger it (may only need trigger)
}


// Start TIMER1 at 100000Hz, used for DACC trigger
void start_timer1()
{
  register Tc *ptc ;
	register uint32_t timer ;

	// Enable peripheral clock TC0 = bit 23 thru TC5 = bit 28
  PMC->PMC_PCER0 |= 0x01000000L ;		// Enable peripheral clock to TC1
  
	timer = Master_frequency / 800000 ;		// MCK/8 and 100 000 Hz
	ptc = TC0 ;		// Tc block 0 (TC0-2)
	ptc->TC_BCR = 0 ;			// No sync
	ptc->TC_BMR = 0 ;
	ptc->TC_CHANNEL[1].TC_CMR = 0x00008000 ;	// Waveform mode
	ptc->TC_CHANNEL[1].TC_RC = timer ;			// 100 000 Hz
	ptc->TC_CHANNEL[1].TC_RA = timer >> 1 ;
	ptc->TC_CHANNEL[1].TC_CMR = 0x0009C001 ;	// 0000 0000 0000 1001 1100 0000 0000 0001
																						// MCK/8, set @ RA, Clear @ RC waveform
	ptc->TC_CHANNEL[1].TC_CCR = 5 ;		// Enable clock and trigger it (may only need trigger)
	Sound_g.Frequency = 1000 ;
}





// Configure DAC1 (or DAC0 for REVB)
// Not sure why PB14 has not be allocated to the DAC, although it is an EXTRA function
// So maybe it is automatically done
void init_dac()
{
	register Dacc *dacptr ;

	SoundType = SOUND_NONE ;

  PMC->PMC_PCER0 |= 0x40000000L ;		// Enable peripheral clock to DAC
	dacptr = DACC ;
#ifdef REVB
	dacptr->DACC_MR = 0x0B000215L ;			// 0000 1011 0000 0001 0000 0010 0001 0101
#else
	dacptr->DACC_MR = 0x0B010215L ;			// 0000 1011 0000 0001 0000 0010 0001 0101
#endif
#ifdef REVB
	dacptr->DACC_CHER	= 1 ;							// Enable channel 0
#else
	dacptr->DACC_CHER	= 2 ;							// Enable channel 1
#endif
	dacptr->DACC_CDR = 2048 ;						// Half amplitude
// Data for PDC must NOT be in flash, PDC needs a RAM source.
#ifndef SIMU
	dacptr->DACC_TPR = (uint32_t) Sine_values ;
	dacptr->DACC_TNPR = (uint32_t) Sine_values ;
#endif
	dacptr->DACC_TCR = 50 ;		// words, 100 16 bit values
	dacptr->DACC_TNCR = 50 ;	// words, 100 16 bit values
	dacptr->DACC_PTCR = DACC_PTCR_TXTEN ;
	NVIC_EnableIRQ(DACC_IRQn) ;
}

//#ifndef SIMU
//extern "C" void DAC_IRQHandler()
//{
//	register Dacc *dacptr ;

//	dacptr = DACC ;
//// Data for PDC must NOT be in flash, PDC needs a RAM source.
//	if ( SoundType == SOUND_STOP )
//	{
//		if ( dacptr->DACC_ISR & DACC_ISR_TXBUFE ;	// All sent
//		{
//			dacptr->DACC_IDR = DACC_IDR_TXBUFE ;	// Kill interrupt
//			SoundType = SOUND_NONE ;		// Tell everyone else
//		}
//		else
//		{
//			dacptr->DACC_IDR = DACC_IDR_ENDTX ;		// Stop sending
//			dacptr->DACC_IER = DACC_IER_TXBUFE ;	// Wait for finished
//		}
//	}
//	else if ( SoundType == SOUND_TONE )
//	{
//		if ( Sound_g.Tone_timer )
//		{
//			if ( --Sound_g.Tone_timer == 0 )
//			{
//				dacptr->DACC_IDR = DACC_IDR_ENDTX ;
//				SoundType = SOUND_STOP ;
//			}
//			else
//			{
//				dacptr->DACC_TNPR = (uint32_t) Sine_values ;
//				dacptr->DACC_TNCR = 50 ;	// words, 100 16 bit values
//			}
//		}
//	}							 
//	else if ( SoundType == SOUND_VOICE )
//	{
//		VoiceBuffer[VoiceIndex[0]].flags |= VF_SENT ;		// Tell caller

//		VoiceIndex[0] = VoiceIndex[1] ;
//		VoiceIndex[1] = VoiceIndex[2] ;
//		VoiceCount -= 1 ;




//	}
//	else if ( SoundType == SOUND_NONE )
//	{
//		dacptr->DACC_IDR = DACC_IDR_ENDTX ;		// Kill interrupt
//		dacptr->DACC_IDR = DACC_IDR_TXBUFE ;	// Kill interrupt
//	}
//}
//#endif

#ifndef SIMU
extern "C" void DAC_IRQHandler()
{
// Data for PDC must NOT be in flash, PDC needs a RAM source.
	if ( Sound_g.VoiceActive )
	{
		PtrVoiceBuffer[0]->flags |= VF_SENT ;		// Flag sent
		PtrVoiceBuffer[0] = PtrVoiceBuffer[1] ;
		PtrVoiceBuffer[1] = PtrVoiceBuffer[2] ;

		if ( DACC->DACC_ISR & DACC_ISR_TXBUFE )
		{
			DACC->DACC_IDR = DACC_IDR_TXBUFE ;
			Sound_g.VoiceActive = 0 ;
		}

		VoiceCount -= 1 ;
		if ( VoiceCount == 0 )		// Run out of buffers
		{
			DACC->DACC_IDR = DACC_IDR_ENDTX ;
			DACC->DACC_IER = DACC_IER_TXBUFE ;
		}
		else
		{
			DACC->DACC_TNCR = PtrVoiceBuffer[1]->count / 2 ;		// words, 100 16 bit values
			DACC->DACC_TNPR = (uint32_t) PtrVoiceBuffer[1]->data ;
		}
	}
	else
	{
		DACC->DACC_TNPR = (uint32_t) Sine_values ;
		DACC->DACC_TNCR = 50 ;	// words, 100 16 bit values
		if ( Sound_g.Tone_timer )
		{
			if ( --Sound_g.Tone_timer == 0 )
			{
				DACC->DACC_IDR = DACC_IDR_ENDTX ;
			}
		}
	}
}
#endif


void end_sound()
{
	DACC->DACC_IDR = DACC_IDR_ENDTX ;		// Kill interrupt
	DACC->DACC_IDR = DACC_IDR_TXBUFE ;	// Kill interrupt
	NVIC_DisableIRQ(DACC_IRQn) ;
	TWI0->TWI_IDR = TWI_IDR_TXCOMP ;
	NVIC_DisableIRQ(TWI0_IRQn) ;
	PMC->PMC_PCER0 &= ~0x00080000L ;		// Disable peripheral clock to TWI0
  PMC->PMC_PCER0 &= ~0x40000000L ;		// Disable peripheral clock to DAC
}

// Called every 5mS from interrupt routine
void sound_5ms()
{
	register Dacc *dacptr ;

	dacptr = DACC ;
	if ( Sound_g.Tone_ms_timer > 0 )
	{
		Sound_g.Tone_ms_timer -= 1 ;
	}
		
	if ( Sound_g.Tone_ms_timer == 0 )
	{
		if ( Sound_g.VoiceRequest )
		{
			dacptr->DACC_IDR = DACC_IDR_ENDTX ;	// Disable interrupt
			Sound_g.Sound_time = 0 ;						// Remove any pending tone requests
			if ( dacptr->DACC_ISR & DACC_ISR_TXBUFE )	// All sent
			{
				// Now we can send the voice file
				Sound_g.VoiceRequest = 0 ;
				Sound_g.VoiceActive = 1 ;

				set_frequency( VoiceBuffer[0].frequency ? VoiceBuffer[0].frequency : 16000 ) ;
#ifndef SIMU
				dacptr->DACC_TPR = (uint32_t) VoiceBuffer[0].data ;
				dacptr->DACC_TCR = VoiceBuffer[0].count / 2 ;		// words, 100 16 bit values
			
				dacptr->DACC_TNPR = (uint32_t) VoiceBuffer[1].data ;
				dacptr->DACC_TNCR = VoiceBuffer[1].count / 2 ;		// words, 100 16 bit values
				dacptr->DACC_PTCR = DACC_PTCR_TXTEN ;
#endif
				dacptr->DACC_IER = DACC_IER_ENDTX ;
			}
			return ;
		}
		
		if ( ( Sound_g.VoiceActive ) || ( ( Voice.VoiceQueueCount ) && sd_card_ready() ) )
		{
			Sound_g.Sound_time = 0 ;						// Remove any pending tone requests
			return ;
		}
				
		if ( Sound_g.Sound_time )
		{
			Sound_g.Tone_ms_timer = ( Sound_g.Sound_time + 4 ) / 5 ;
			if ( Sound_g.Next_freq )		// 0 => silence for time
			{
				Sound_g.Frequency = Sound_g.Next_freq ;
				Sound_g.Frequency_increment = Sound_g.Next_frequency_increment ;
				set_frequency( Sound_g.Frequency * 100 ) ;
#ifndef SIMU
				dacptr->DACC_TPR = (uint32_t) Sine_values ;
				dacptr->DACC_TNPR = (uint32_t) Sine_values ;
#endif
				dacptr->DACC_TCR = 50 ;		// words, 100 16 bit values
				dacptr->DACC_TNCR = 50 ;	// words, 100 16 bit values
				tone_start( 0 ) ;
			}
			else
			{
				dacptr->DACC_IDR = DACC_IDR_ENDTX ;		// Silence
			}
			Sound_g.Sound_time = 0 ;
		}
		else
		{
			dacptr->DACC_IDR = DACC_IDR_ENDTX ;	// Disable interrupt
			Sound_g.Tone_timer = 0 ;	
		}
	}
	else if ( ( Sound_g.Tone_ms_timer & 1 ) == 0 )		// Every 10 mS
	{
		if ( Sound_g.Frequency )
		{
			if ( Sound_g.Frequency_increment )
			{
				Sound_g.Frequency += Sound_g.Frequency_increment ;
				set_frequency( Sound_g.Frequency * 100 ) ;
			}
		}
	}
}


void wavU8Convert( uint8_t *src, uint16_t *dest , uint32_t count )
{
	while( count-- )
	{
		*dest++ = *src++ << 4 ;
	}
}

void wavU16Convert( uint16_t *src, uint16_t *dest , uint32_t count )
{
	while( count-- )
	{
		*dest++ = (uint16_t)( (int16_t )*src++ + 32768) >> 4 ;
	}
}


void startVoice( uint32_t count )		// count of filled in buffers
{
	VoiceBuffer[0].flags &= ~VF_SENT ;
	PtrVoiceBuffer[0] = &VoiceBuffer[0] ;
	if ( count > 1 )
	{
		VoiceBuffer[1].flags &= ~VF_SENT ;
		PtrVoiceBuffer[1] = &VoiceBuffer[1] ;
	}
	if ( count > 2 )
	{
		VoiceBuffer[2].flags &= ~VF_SENT ;
		PtrVoiceBuffer[2] = &VoiceBuffer[2] ;
	}
	VoiceCount = count ;
	Sound_g.VoiceRequest = 1 ;
}


void appendVoice( uint32_t index )		// index of next buffer
{
	VoiceBuffer[index].flags &= ~VF_SENT ;
	__disable_irq() ;
	PtrVoiceBuffer[VoiceCount++] = &VoiceBuffer[index] ;
	if ( VoiceCount == 2 )
	{
		DACC->DACC_TNPR = CONVERT_PTR(VoiceBuffer[index].data);
		DACC->DACC_TNCR = VoiceBuffer[index].count / 2 ;		// words, 100 16 bit values
	}
	__enable_irq() ;
}

// frequency in Hz, time in mS
void playTone( uint32_t frequency, uint32_t time )
{
	Sound_g.Next_frequency_increment = 0 ;
	Sound_g.Next_freq = frequency ;
	Sound_g.Sound_time = time ;
//	set_frequency( frequency ) ;
//	Tone_ms_timer = ( time + 4 ) / 5 ;
//	tone_start( 0 ) ;
}

uint32_t queueTone( uint32_t frequency, uint32_t time, uint32_t frequency_increment )
{
	if ( Sound_g.Sound_time == 0 )
	{
		Sound_g.Next_freq = frequency ;
		Sound_g.Next_frequency_increment = frequency_increment ;
		Sound_g.Sound_time = time ;
		return 1 ;
	}
	return 0 ;	
}

// Time is in milliseconds
void tone_start( register uint32_t time )
{
  PMC->PMC_PCER0 |= 0x40000000L ;		// Enable peripheral clock to DAC
	Sound_g.Tone_timer = Sound_g.Frequency * time / 1000 ;
	DACC->DACC_IER = DACC_IER_ENDTX ;
}

void tone_stop()
{
	DACC->DACC_IDR = DACC_IDR_ENDTX ;	// Disable interrupt
	Sound_g.Tone_timer = 0 ;	
}


// Set up for volume control (TWI0)
// Need PA3 and PA4 set to peripheral A
void init_twi()
{
	register Pio *pioptr ;
	register uint32_t timing ;
  
	PMC->PMC_PCER0 |= 0x00080000L ;		// Enable peripheral clock to TWI0
	
	/* Configure PIO */
	pioptr = PIOA ;
  pioptr->PIO_ABCDSR[0] &= ~0x00000018 ;	// Peripheral A
  pioptr->PIO_ABCDSR[1] &= ~0x00000018 ;	// Peripheral A
  pioptr->PIO_PDR = 0x00000018 ;					// Assign to peripheral
	
	timing = Master_frequency * 5 / 2000000 ;		// 2.5uS high and low (200 kHz)
	timing += 15 - 4 ;
	timing /= 16 ;
	timing |= timing << 8 ;

	TWI0->TWI_CWGR = 0x00040000 | timing ;			// TWI clock set
	TWI0->TWI_CR = TWI_CR_MSEN | TWI_CR_SVDIS ;		// Master mode enable
	TWI0->TWI_MMR = 0x002F0000 ;		// Device 5E (>>1) and master is writing
	NVIC_EnableIRQ(TWI0_IRQn) ;
	set_volume( 2 ) ;
}

static int8_t Volume_required ;
static uint8_t Volume_read_pending ;
static uint8_t CoProc_read_pending ;
static uint8_t CoProc_write_pending ;
static uint8_t CoProc_appgo_pending ;
uint8_t Volume_read ;
uint8_t Coproc_read ;
int8_t Coproc_valid ;
//static uint8_t Twi_mode ;
static uint8_t *Twi_read_address ;
//static uint8_t TwiDevice ;
static uint8_t TwiOperation ;
//const uint8_t *CoProgBufPtr ;
//uint8_t Program_coprocessor ;
//uint8_t Program_blocks_written ;

//#define TWI_MODE_WRITE	0
//#define TWI_MODE_READ		1

//#define TWI_VOLUME	1
//#define TWI_COPROC	2

#define TWI_NONE		    	0
#define TWI_READ_VOL    	1
#define TWI_WRITE_VOL     2
#define TWI_READ_COPROC   3
#define TWI_COPROC_APPGO  4
#define TWI_WAIT_STOP		  5
#define TWI_WRITE_COPROC	6

// Commands to the coprocessor bootloader/application
#define TWI_CMD_PAGEUPDATE        	0x01	// TWI Command to program a flash page
#define TWI_CMD_EXECUTEAPP        	0x02	// TWI Command to jump to the application program
#define TWI_CMD_SETREAD_ADDRESS			0x03	// TWI Command to set address to read from
#define TWI_CMD_WRITE_DATA         	0x04	// TWI Command send data to the application


static const uint8_t Volume_scale[NUM_VOL_LEVELS] = 
{
	 0,  2,  4,   6,   8,  10,  13,  17,  22,  27,  33,  40,
	64, 82, 96, 105, 112, 117, 120, 122, 124, 125, 126, 127 	
} ;

#define COPROC_RX_BUXSIZE		22
uint8_t Co_proc_status[COPROC_RX_BUXSIZE] ;
uint8_t *Co_proc_write_ptr ;
uint32_t Co_proc_write_count ;


// This is called from an interrupt routine, or
// interrupts must be disabled while it is called
// from elsewhere.
void i2c_check_for_request()
{
	if ( TWI0->TWI_IMR & TWI_IMR_TXCOMP )
	{
		return ;		// Busy
	}
	
	if ( Volume_required >= 0 )				// Set volume to this value
	{
		TWI0->TWI_MMR = 0x002F0000 ;		// Device 5E (>>1) and master is writing
		TwiOperation = TWI_WRITE_VOL ;
		TWI0->TWI_THR = Volume_required ;		// Send data
		Volume_required = -1 ;
		TWI0->TWI_IER = TWI_IER_TXCOMP ;
		TWI0->TWI_CR = TWI_CR_STOP ;		// Stop Tx
	}
	else if ( CoProc_read_pending )
	{
		Coproc_valid = 0 ;
		CoProc_read_pending = 0 ;
		TWI0->TWI_MMR = 0x00351000 ;		// Device 35 and master is reading
		TwiOperation = TWI_READ_COPROC ;
#ifndef SIMU
		TWI0->TWI_RPR = (uint32_t)&Co_proc_status[0] ;
#endif
		TWI0->TWI_RCR = COPROC_RX_BUXSIZE - 1 ;
		if ( TWI0->TWI_SR & TWI_SR_RXRDY )
		{
			(void) TWI0->TWI_RHR ;
		}
		TWI0->TWI_PTCR = TWI_PTCR_RXTEN ;	// Start transfers
		TWI0->TWI_CR = TWI_CR_START ;		// Start Rx
		TWI0->TWI_IER = TWI_IER_RXBUFF | TWI_IER_TXCOMP ;
	}
	else if ( Volume_read_pending )
	{
		Volume_read_pending = 0 ;
		TWI0->TWI_MMR = 0x002F1000 ;		// Device 5E (>>1) and master is reading
		TwiOperation = TWI_READ_VOL ;
		Twi_read_address = &Volume_read ;
		TWI0->TWI_CR = TWI_CR_START | TWI_CR_STOP ;		// Start and stop Tx
		TWI0->TWI_IER = TWI_IER_TXCOMP ;
	}
	else if ( CoProc_appgo_pending )
	{
		CoProc_appgo_pending = 0 ;
		TWI0->TWI_MMR = 0x00350000 ;		// Device 35 and master is writing
		TwiOperation = TWI_COPROC_APPGO ;
		TWI0->TWI_THR = TWI_CMD_EXECUTEAPP ;	// Send appgo command
		TWI0->TWI_IER = TWI_IER_TXCOMP ;
		TWI0->TWI_CR = TWI_CR_STOP ;		// Stop Tx
	}
	else if ( CoProc_write_pending )
	{
		CoProc_write_pending = 0 ;
		TWI0->TWI_MMR = 0x00350000 ;		// Device 35 and master is writing
		TwiOperation = TWI_WRITE_COPROC ;
#ifndef SIMU
		TWI0->TWI_TPR = (uint32_t)Co_proc_write_ptr ;
#endif
		TWI0->TWI_RCR = Co_proc_write_count ;
		TWI0->TWI_THR = TWI_CMD_WRITE_DATA ;	// Send write command
		TWI0->TWI_PTCR = TWI_PTCR_TXTEN ;	// Start data transfer
		TWI0->TWI_IER = TWI_IER_TXBUFE | TWI_IER_TXCOMP ;
	}
}

void set_volume( register uint8_t volume )
{
//	PMC->PMC_PCER0 |= 0x00080000L ;		// Enable peripheral clock to TWI0
	
	if ( volume >= NUM_VOL_LEVELS )
	{
		volume = NUM_VOL_LEVELS - 1 ;		
	}
	volume = Volume_scale[volume] ;
	Volume_required = volume ;
	__disable_irq() ;
	i2c_check_for_request() ;
	__enable_irq() ;
}

void read_volume()
{
	Volume_read_pending = 1 ;
	__disable_irq() ;
	i2c_check_for_request() ;
	__enable_irq() ;
}

void read_coprocessor()
{
	CoProc_read_pending = 1 ;
	__disable_irq() ;
	i2c_check_for_request() ;
	__enable_irq() ;
}	

void write_coprocessor( uint8_t *ptr, uint32_t count )
{
	Co_proc_write_ptr = ptr ;
	Co_proc_write_count = count ;
	CoProc_write_pending = 1 ;
	__disable_irq() ;
	i2c_check_for_request() ;
	__enable_irq() ;
}	

void appgo_coprocessor()
{
	CoProc_appgo_pending = 1 ;
	__disable_irq() ;
	i2c_check_for_request() ;
	__enable_irq() ;
}	

//
//
//void set_volume( register uint8_t volume )
//{
////	PMC->PMC_PCER0 |= 0x00080000L ;		// Enable peripheral clock to TWI0
	
//	if ( volume >= NUM_VOL_LEVELS )
//	{
//		volume = NUM_VOL_LEVELS - 1 ;		
//	}
//	volume = Volume_scale[volume] ;

//	__disable_irq() ;
//	if ( TWI0->TWI_IMR & TWI_IMR_TXCOMP )
//	{
//		Volume_required = volume ;
//	}
//	else
//	{
//		TWI0->TWI_MMR = 0x002F0000 ;		// Device 5E (>>1) and master is writing
//		TwiDevice = TWI_VOLUME ;
//		Twi_mode = TWI_MODE_WRITE ;
//		TWI0->TWI_THR = volume ;		// Send data
//		TWI0->TWI_CR = TWI_CR_STOP ;		// Stop Tx
//		TWI0->TWI_IER = TWI_IER_TXCOMP ;
//	}
//	__enable_irq() ;
//}

#ifndef SIMU
extern "C" void TWI0_IRQHandler()
{
	if ( TwiOperation == TWI_READ_VOL )
	{
		if ( TWI0->TWI_SR & TWI_SR_RXRDY )
		{
			*Twi_read_address = TWI0->TWI_RHR ;		// Read data
		}
	}

	if ( TwiOperation == TWI_READ_COPROC )
	{
		if ( TWI0->TWI_SR & TWI_SR_RXBUFF )
		{
			TWI0->TWI_IDR = TWI_IDR_RXBUFF ;
			TwiOperation = TWI_WAIT_STOP ;
			TWI0->TWI_CR = TWI_CR_STOP ;	// Stop Rx
			TWI0->TWI_RCR = 1 ;						// Last byte
			return ;
		}
		else
		{
			Coproc_valid = -1 ;			
		}
	}		
			
	if ( TwiOperation == TWI_WAIT_STOP )
	{
		Coproc_valid = 1 ;
		Coproc_read = Co_proc_status[0] ;
		if ( Coproc_read & 0x80 )			// Bootloader
		{
			CoProc_appgo_pending = 1 ;	// Action application
		}
		TWI0->TWI_PTCR = TWI_PTCR_RXTDIS ;	// Stop transfers
		if ( TWI0->TWI_SR & TWI_SR_RXRDY )
		{
			(void) TWI0->TWI_RHR ;			// Discard any rubbish data
		}
	}

//	if ( TwiOperation == TWI_WRITE_VOL )
//	{
		
//	}

	if ( TwiOperation == TWI_WRITE_COPROC )
	{
		if ( TWI0->TWI_SR & TWI_SR_TXBUFE )
		{
			TWI0->TWI_IDR = TWI_IDR_TXBUFE ;
			TWI0->TWI_CR = TWI_CR_STOP ;		// Stop Tx
			TWI0->TWI_PTCR = TWI_PTCR_TXTDIS ;	// Stop transfers
			TwiOperation = TWI_NONE ;
			return ;
		}
	}
	 
	TWI0->TWI_IDR = TWI_IDR_TXCOMP | TWI_IDR_TXBUFE | TWI_PTCR_TXTDIS ;
	if ( TWI0->TWI_SR & TWI_SR_NACK )
	{
	}
	TwiOperation = TWI_NONE ;	
	i2c_check_for_request() ;
	
}
#endif


void hapticOff()
{
	PWM->PWM_DIS = PWM_DIS_CHID2 ;						// Disable channel 2
	PWM->PWM_OOV &= ~0x00040000 ;	// Force low
	PWM->PWM_OSS |= 0x00040000 ;	// Force low
}

// pwmPercent 0-100
void hapticOn( uint32_t pwmPercent )
{
	register Pwm *pwmptr ;

	pwmptr = PWM ;

	if ( pwmPercent > 100 )
	{
		pwmPercent = 100 ;		
	}
	pwmptr->PWM_CH_NUM[2].PWM_CDTYUPD = pwmPercent ;		// Duty
	pwmptr->PWM_ENA = PWM_ENA_CHID2 ;						// Enable channel 2
	pwmptr->PWM_OSC = 0x00040000 ;	// Enable output
}


