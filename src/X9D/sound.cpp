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

#include "stm32f2xx.h"
#ifndef SIMU
 	#include "core_cm3.h"
#endif
#include "..\sound.h"
#include "..\ersky9x.h"
#include "..\myeeprom.h"
#include "..\drivers.h"
#include "..\audio.h"
#include "..\logicio.h"
#include "..\timers.h"
#include "i2c_ee.h"
#include "hal.h"


void start_sound( void ) ;
//void buzzer_on( void ) ;
//void buzzer_off( void ) ;
//void buzzer_sound( uint8_t time ) ;
void start_dactimer( void ) ;
void init_dac( void ) ;
extern "C" void DAC_IRQHandler( void ) ;

//void disp_mem( register uint32_t address ) ;
void end_sound( void ) ;
void tone_start( register uint32_t time ) ;
void tone_stop( void ) ;
//void init_twi( void ) ;
void setVolume( register uint8_t volume ) ;
//extern "C" void TWI0_IRQHandler (void) ;
//void audioDefevent( uint8_t e ) ;


extern uint32_t Master_frequency ;
extern uint8_t CurrentVolume ;

struct t_sound_globals Sound_g ;

struct t_VoiceBuffer VoiceBuffer[3] ;

#define SOUND_NONE	0
#define SOUND_TONE	1
#define SOUND_VOICE	2
#define SOUND_STOP	3

struct t_VoiceBuffer *PtrVoiceBuffer[3] ;
uint8_t VoiceCount ;
uint8_t SoundType ;
uint8_t DacIdle ;

	 
// Must NOT be in flash, PDC needs a RAM source.
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

//const uint16_t PianoTones[] =
//{
//  28,   29,   31,   33,   35,   37,   39,   41,   44,   46,
//  49,   52,   55,   58,   62,   65,   69,   73,   78,   82,
//  87,   92,   98,  104,  110,  117,  123,  131,  139,  147,
// 156,  165,  175,  185,  196,  208,  220,  233,  247,  262, // d#, E, F, f#, G, g#, A, a#, B, C(middle)
// 277,  294,  311,  330,  349,  370,  392,  415,  440,  466, // c#, D, d#, E, F, f#, G, g#, A, a#
// 494,  523,  554,  587,  622,  659,  698,  740,  784,  831, // B, C, c#, D, d#, E, F, f#, G, g#
// 880,  932,  988, 1047, 1109, 1175, 1245, 1319, 1397, 1480,
//1568, 1661, 1760, 1865, 1976, 2093, 2217, 2349, 2489, 2637,
//2794, 2960, 3136, 3322, 3520 ,3729, 3951, 4186
//} ;


// Sound routines

void start_sound()
{
	start_dactimer() ;
	init_dac() ;

	// TODO - for volume, shared with EEPROM?
	//init_twi() ;
}


void set_frequency( uint32_t frequency )
{
	register uint32_t timer ;

	timer = (Peri1_frequency*Timer_mult1) / frequency - 1 ;		// MCK/8 and 100 000 Hz
	if ( timer > 65535 )
	{
		timer = 65535 ;		
	}
	if ( timer < 2 )
	{
		timer = 2 ;		
	}
	TIM6->CR1 &= ~TIM_CR1_CEN ;
	TIM6->CNT = 0 ;
	TIM6->ARR = timer ;
	TIM6->CR1 |= TIM_CR1_CEN ;
}

uint32_t test_sound()
{
//	uint32_t i ;
//	uint32_t j ;
	start_sound() ;
	return 0 ;
//	i = 0 ;
//	while ( ( DMA1->HISR & DMA_HISR_TCIF5 ) == 0 )
//	{
//		if ( ++i > 10000000 )
//		{
//			end_sound() ;
//			return 0xFFFF ;
//		}
//	}
//	j = DMA1_Stream5->FCR ;
//	TIM13->CNT = 0 ;
//	TIM13->EGR = 1 ;		// Re-start counter
//	i = 0 ;
//	while ( ( DAC->SR & DAC_SR_DMAUDR1 ) == 0 )
//	{
//		if ( ++i > 10000000 )
//		{
//			end_sound() ;
//			return 0xFFFE ;
//		}
//	}
//	return (j<<16) | TIM13->CNT ;
}

extern void stop_sound()
{
	DMA1_Stream5->CR &= ~DMA_SxCR_CIRC ;
}


// Start TIMER6 at 100000Hz, used for DAC trigger
void start_dactimer()
{
	// Now for timer 6
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN ;		// Enable clock
	
	TIM6->PSC = 0 ;													// Max speed
	TIM6->ARR = (Peri1_frequency*Timer_mult1) / 100000 - 1 ;	// 10 uS, 100 kHz
	TIM6->CR2 = 0 ;
	TIM6->CR2 = 0x20 ;
	TIM6->CR1 = TIM_CR1_CEN ;
}



// Configure STM32 DAC1 (not DAC2) on PA4
// Use TIMER 6 for trigger/timebase
// DMA1, Stream 5, channel 7
void init_dac()
{
	DacIdle = 1 ;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN ; 		// Enable portA clock
	configure_pins( 0x0010, PIN_ANALOG | PIN_PORTA ) ;
	RCC->APB1ENR |= RCC_APB1ENR_DACEN ;				// Enable clock
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN ;			// Enable DMA1 clock
	
 	// Chan 7, 16-bit wide, Medium priority, memory increments
	DMA1_Stream5->CR &= ~DMA_SxCR_EN ;		// Disable DMA
	DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5 ; // Write ones to clear bits
	DMA1_Stream5->CR = DMA_SxCR_CHSEL_0 | DMA_SxCR_CHSEL_1 | DMA_SxCR_CHSEL_2 | DMA_SxCR_PL_0 | 
										 DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0 | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_CIRC ;
	DMA1_Stream5->PAR = (uint32_t) &DAC->DHR12R1 ;
	DMA1_Stream5->M0AR = (uint32_t) Sine_values ;
	DMA1_Stream5->FCR = 0x05 ; //DMA_SxFCR_DMDIS | DMA_SxFCR_FTH_0 ;
	DMA1_Stream5->NDTR = 100 ;

	DAC->DHR12R1 = 2010 ;
	DAC->SR = DAC_SR_DMAUDR1 ;		// Write 1 to clear flag
	DAC->CR = DAC_CR_TEN1 | DAC_CR_EN1 ;			// Enable DAC
	NVIC_SetPriority( DMA1_Stream5_IRQn, 2 ) ; // Lower priority interrupt
	NVIC_EnableIRQ(TIM6_DAC_IRQn) ;
	NVIC_EnableIRQ(DMA1_Stream5_IRQn) ;
}

#ifndef SIMU
extern "C" void TIM6_DAC_IRQHandler()
{
	DAC->CR &= ~DAC_CR_DMAEN1 ;			// Stop DMA requests
	DAC->CR &= ~DAC_CR_DMAUDRIE1 ;	// Stop underrun interrupt
	DacIdle = 1 ;
	DAC->SR = DAC_SR_DMAUDR1 ;			// Write 1 to clear flag
}

uint8_t AudioVoiceUnderrun ;

extern "C" void DMA1_Stream5_IRQHandler()
{
	DMA1_Stream5->CR &= ~DMA_SxCR_TCIE ;		// Stop interrupt
	DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5 ; // Write ones to clear flags
	if ( Sound_g.VoiceActive )
	{
		PtrVoiceBuffer[0]->flags |= VF_SENT ;		// Flag sent
		PtrVoiceBuffer[0] = PtrVoiceBuffer[1] ;
		PtrVoiceBuffer[1] = PtrVoiceBuffer[2] ;

		VoiceCount -= 1 ;
		if ( VoiceCount == 0 )		// Run out of buffers
		{
			AudioVoiceUnderrun = 1 ;		// For debug
			Sound_g.VoiceActive = 0 ;
//			DMA1_Stream5->CR &= ~DMA_SxCR_TCIE ;			// Disable DMA interrupt
			DMA1_Stream5->CR &= ~DMA_SxCR_EN ;				// Disable DMA channel
			DacIdle = 1 ;
		}
		else
		{
			DMA1_Stream5->CR &= ~DMA_SxCR_EN ;				// Disable DMA channel
			DMA1_Stream5->M0AR = (uint32_t) PtrVoiceBuffer[0]->data ;
			DMA1_Stream5->NDTR =  PtrVoiceBuffer[0]->count ;
			DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5 ; // Write ones to clear bits
			DMA1_Stream5->CR |= DMA_SxCR_EN | DMA_SxCR_TCIE ;	// Enable DMA channel
			DAC->SR = DAC_SR_DMAUDR1 ;			// Write 1 to clear flag
		}
	}
//	else
//	{
//		if ( Sound_g.Tone_timer )
//		{
//			if ( --Sound_g.Tone_timer == 0 )
//			{
//				DMA1_Stream5->CR &= ~DMA_SxCR_EN & ~DMA_SxCR_TCIE ;				// Disable DMA channel
//				DacIdle = 1 ;
//			}
//			else
//			{
//				DMA1_Stream5->CR |= DMA_SxCR_EN | DMA_SxCR_TCIE ;	// Enable DMA channel
//			}
//		}
//	}
}
#endif


void end_sound()
{
	DAC->CR = 0 ;
	TIM6->CR1 = 0 ;
	// Also need to turn off any possible interrupts
	NVIC_DisableIRQ(TIM6_DAC_IRQn) ;
	NVIC_DisableIRQ(DMA1_Stream5_IRQn) ;

}

// Called every 5mS from interrupt routine
void sound_5ms()
{
	if ( Sound_g.Tone_ms_timer > 0 )
	{
		
		if ( --Sound_g.Tone_ms_timer == 0 )
		{
			DMA1_Stream5->CR &= ~DMA_SxCR_CIRC ;		// Stops DMA at end of cycle
//			Sound_g.Tone_timer = 0 ;	
		}
	}
		
	if ( Sound_g.Tone_ms_timer == 0 )
	{
		if ( Sound_g.VoiceRequest )
		{
			Sound_g.Sound_time = 0 ;						// Remove any pending tone requests
			
			if ( DacIdle )	// All sent
			{
				DacIdle = 0 ;
				// Now we can send the voice file
				Sound_g.VoiceRequest = 0 ;
				Sound_g.VoiceActive = 1 ;
				set_frequency( VoiceBuffer[0].frequency ? VoiceBuffer[0].frequency : 16000 ) ;
			
#ifndef SIMU
				DMA1_Stream5->CR &= ~DMA_SxCR_EN ;				// Disable DMA channel
				DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5 ; // Write ones to clear bits
				DMA1_Stream5->M0AR = (uint32_t) VoiceBuffer[0].data ;
				DMA1_Stream5->NDTR =  VoiceBuffer[0].count ;
				DMA1_Stream5->CR |= DMA_SxCR_EN | DMA_SxCR_TCIE ;		// Enable DMA channel and interrupt
				DAC->SR = DAC_SR_DMAUDR1 ;			// Write 1 to clear flag
				DAC->CR |= DAC_CR_EN1 | DAC_CR_DMAEN1 ;			// Enable DAC
				
#endif
			}
			return ;
		}
		
		if ( ( Sound_g.VoiceActive ) || ( ( Voice.VoiceQueueCount ) && sd_card_ready() ) )
//		if ( Sound_g.VoiceActive )
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
				DMA1_Stream5->CR &= ~DMA_SxCR_EN ;				// Disable DMA channel
				DMA1_Stream5->M0AR = (uint32_t) Sine_values ;
				DMA1_Stream5->NDTR = 100 ;
#endif
				DacIdle = 0 ;
				tone_start( 0 ) ;
			}
			else
			{
				DMA1_Stream5->CR &= ~DMA_SxCR_CIRC ;		// Stops DMA at end of cycle
				DMA1_Stream5->CR |= DMA_SxCR_EN | DMA_SxCR_TCIE ;	// Enable DMA channel
			}
			Sound_g.Sound_time = 0 ;
		}
		else
		{
			DMA1_Stream5->CR &= ~DMA_SxCR_CIRC ;		// Stops DMA at end of cycle
//			Sound_g.Tone_timer = 0 ;	
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


// frequency in Hz, time in mS
void playTone( uint32_t frequency, uint32_t time )
{
	Sound_g.Next_frequency_increment = 0 ;
	Sound_g.Next_freq = frequency ;
	Sound_g.Sound_time = time ;
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
//	Sound_g.Tone_timer = Sound_g.Frequency * time / 1000 ;

	DMA1->HIFCR = DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5 ; // Write ones to clear bits
	DMA1_Stream5->CR |= DMA_SxCR_CIRC | DMA_SxCR_EN ;				// Enable DMA channel
	DAC->SR = DAC_SR_DMAUDR1 ;			// Write 1 to clear flag
	DAC->CR |= DAC_CR_EN1 | DAC_CR_DMAEN1 | DAC_CR_DMAUDRIE1 ;			// Enable DAC
	DMA1_Stream5->CR |= DMA_SxCR_TCIE ;	// Enable DMA interrupt
}

void tone_stop()
{
	DMA1_Stream5->CR &= ~DMA_SxCR_CIRC ;
//	Sound_g.Tone_timer = 0 ;	
}


void startVoice( uint32_t count )		// count of filled in buffers
{
	AudioVoiceUnderrun = 0 ;
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
	__enable_irq() ;
}

static const uint8_t Volume_scale[NUM_VOL_LEVELS] = 
{
	 0,  2,  4,   6,   8,  10,  13,  17,  22,  27,  33,  40,
	64, 82, 96, 105, 112, 117, 120, 122, 124, 125, 126, 127 	
} ;


void setVolume( register uint8_t volume )
{
#if !defined(SIMU)
	if ( volume >= NUM_VOL_LEVELS )
	{
		volume = NUM_VOL_LEVELS - 1 ;		
	}
	CurrentVolume = volume ;
	volume = Volume_scale[volume] ;
	I2C_set_volume( volume ) ;
#endif
}

#ifndef REVPLUS
void initHaptic()
{
	configure_pins( GPIO_Pin_HAPTIC, PIN_OUTPUT | PIN_PUSHPULL | PIN_OS25 | PIN_PORTC ) ;
	GPIOHAPTIC->BSRRH = GPIO_Pin_HAPTIC ;
}

void hapticOff()
{
	GPIOHAPTIC->BSRRH = GPIO_Pin_HAPTIC ;
}

// pwmPercent 0-100
void hapticOn( uint32_t pwmPercent )
{
	GPIOHAPTIC->BSRRL = GPIO_Pin_HAPTIC ;
}
#endif

