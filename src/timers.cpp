/****************************************************************************
*  Copyright (c) 2012 by Michael Blandford. All rights reserved.
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
*
****************************************************************************/

#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>


#ifdef PCBSKY
#include "AT91SAM3S4.h"
#endif

#ifdef PCBX9D
#include "x9d\stm32f2xx.h"
#endif


#ifndef SIMU
#include "core_cm3.h"
#endif

#include "ersky9x.h"
#include "timers.h"
#include "logicio.h"
#include "myeeprom.h"


// Starts TIMER at 200Hz, 5mS period
#ifdef PCBSKY
void init5msTimer()
{
  register Tc *ptc ;
	register uint32_t timer ;

  PMC->PMC_PCER0 |= 0x02000000L ;		// Enable peripheral clock to TC2

	timer = Master_frequency / 12800 / 2 ;		// MCK/128 and 200 Hz

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

	interrupt5ms() ;
	
}


void stop5msTimer( void )
{
	TC0->TC_CHANNEL[2].TC_CCR = TC_CCR0_CLKDIS ;
	NVIC_DisableIRQ(TC2_IRQn) ;
  PMC->PMC_PCDR0 |= 0x02000000L ;		// Disable peripheral clock to TC2
}



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

#endif

// Starts TIMER at 200Hz, 5mS period
#ifdef PCBX9D
void init5msTimer()
{
	// Timer14
	RCC->APB1ENR |= RCC_APB1ENR_TIM14EN ;		// Enable clock
	TIM14->ARR = 4999 ;	// 5mS
	TIM14->PSC = (Peri1_frequency*Timer_mult1) / 1000000 - 1 ;		// 1uS from 30MHz
	TIM14->CCER = 0 ;	
	TIM14->CCMR1 = 0 ;
	TIM14->EGR = 0 ;
	TIM14->CR1 = 5 ;
	TIM14->DIER |= 1 ;
	NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn) ;
}

void stop5msTimer( void )
{
	TIM14->CR1 = 0 ;	// stop timer
	NVIC_DisableIRQ(TIM8_TRG_COM_TIM14_IRQn) ;
	RCC->APB1ENR &= ~RCC_APB1ENR_TIM14EN ;		// Disable clock
}

extern "C" void TIM8_TRG_COM_TIM14_IRQHandler()
{
	TIM14->SR &= ~TIM_SR_UIF ;
	interrupt5ms() ;
}


uint16_t PpmStream[20] =
{
	2000,
	2200,
	2400,
	2600,
	2800,
	3000,
	3200,
	3400,
	45000-21600,
	0,0,0,0,0,0,0,0,0,0,0
} ;

uint16_t TrainerPpmStream[20] =
{
	2000,
	2200,
	2400,
	2600,
	2800,
	3000,
	3200,
	3400,
	45000-21600,
	0,0,0,0,0,0,0,0,0,0,0
} ;


extern volatile uint16_t Analog[] ;

uint16_t *PulsePtr ;
uint16_t *TrainerPulsePtr ;

#define PPM_CENTER 1500*2

void setupPulses()
{
  uint32_t i ;
	uint32_t total ;
	uint32_t pulse ;
	uint16_t *ptr ;
  uint32_t p=8+g_model.ppmNCH*2; //Channels *2

  ptr = PpmStream ;

	total = 22500u*2; //Minimum Framelen=22.5 ms
  total += (int16_t(g_model.ppmFrameLength))*1000;

	for ( i = 0 ; i < p ; i += 1 )
	{
//  	pulse = max( (int)min(g_chans512[i],PPM_range),-PPM_range) + PPM_CENTER;
		
		pulse = Analog[i] >> 1 ;
		pulse += 2000 ;
		
		total -= pulse ;
		*ptr++ = pulse ;
	}
	*ptr++ = total ;
	*ptr = 0 ;
	TIM1->CCR2 = total - 1000 ;		// Update time
	TIM1->CCR1 = (g_model.ppmDelay*50+300)*2 ;
}


// PPM output
// Timer 1, channel 1 on PA8 for prototype
// Pin is AF1 function for timer 1
void init_ppm()
{
	// Timer1
	setupPulses() ;
	PulsePtr = PpmStream ;
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN ; 		// Enable portA clock
	configure_pins( 0x0100, PIN_PERIPHERAL | PIN_PORTA | PIN_PER_1 | PIN_OS25 | PIN_PUSHPULL ) ;
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN ;		// Enable clock
	
	TIM1->ARR = *PulsePtr++ ;
	TIM1->PSC = (Peri2_frequency*Timer_mult2) / 2000000 - 1 ;		// 0.5uS from 30MHz
	TIM1->CCER = TIM_CCER_CC1E ;	
	TIM1->CCMR1 = TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2PE ;			// PWM mode 1
	TIM1->CCR1 = 600 ;		// 300 uS pulse
	TIM1->BDTR = TIM_BDTR_MOE ;
 	TIM1->EGR = 1 ;
	TIM1->DIER = TIM_DIER_UDE ;

	TIM1->SR &= ~TIM_SR_UIF ;				// Clear flag
	TIM1->SR &= ~TIM_SR_CC2IF ;				// Clear flag
	TIM1->DIER |= TIM_DIER_CC2IE ;
	TIM1->DIER |= TIM_DIER_UIE ;

	TIM1->CR1 = TIM_CR1_CEN ;
	NVIC_EnableIRQ(TIM1_CC_IRQn) ;
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn) ;

}

extern "C" void TIM1_CC_IRQHandler()
{
	TIM1->DIER &= ~TIM_DIER_CC2IE ;		// stop this interrupt
	TIM1->SR &= ~TIM_SR_CC2IF ;				// Clear flag

	setupPulses() ;

	PulsePtr = PpmStream ;
	
	TIM1->DIER |= TIM_DIER_UDE ;


	TIM1->SR &= ~TIM_SR_UIF ;					// Clear this flag
	TIM1->DIER |= TIM_DIER_UIE ;				// Enable this interrupt
}

extern "C" void TIM1_UP_TIM10_IRQHandler()
{
	TIM1->SR &= ~TIM_SR_UIF ;				// Clear flag

	TIM1->ARR = *PulsePtr++ ;
	if ( *PulsePtr == 0 )
	{
		TIM1->SR &= ~TIM_SR_CC2IF ;			// Clear this flag
		TIM1->DIER |= TIM_DIER_CC2IE ;	// Enable this interrupt
	}
}


void init_hw_timer()
{
	// Timer13
	RCC->APB1ENR |= RCC_APB1ENR_TIM13EN ;		// Enable clock
	TIM13->ARR = 65535 ;
	TIM13->PSC = (Peri1_frequency*Timer_mult1) / 10000000 - 1 ;		// 0.1uS from 30MHz
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

void setupTrainerPulses()
{
  uint32_t i ;
	uint32_t total ;
	uint32_t pulse ;
	uint16_t *ptr ;
  uint32_t p=8+g_model.ppmNCH*2; //Channels *2

  ptr = TrainerPpmStream ;

	total = 22500u*2; //Minimum Framelen=22.5 ms
  total += (int16_t(g_model.ppmFrameLength))*1000;

	for ( i = 0 ; i < p ; i += 1 )
	{
//  	pulse = max( (int)min(g_chans512[i],PPM_range),-PPM_range) + PPM_CENTER;
		
		pulse = Analog[i] >> 1 ;
		pulse += 2000 ;
		
		total -= pulse ;
		*ptr++ = pulse ;
	}
	*ptr++ = total ;
	*ptr = 0 ;
	TIM8->CCR2 = total - 1000 ;		// Update time
	TIM8->CCR4 = (g_model.ppmDelay*50+300)*2 ;
}



// Trainer PPM oputput PC9, Timer 8 channel 4
void init_trainer_ppm()
{
	setupTrainerPulses() ;
	TrainerPulsePtr = TrainerPpmStream ;
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN ; 		// Enable portC clock
	configure_pins( 0x0200, PIN_PERIPHERAL | PIN_PORTC | PIN_PER_3 | PIN_OS25 | PIN_PUSHPULL ) ;
	RCC->APB2ENR |= RCC_APB2ENR_TIM8EN ;		// Enable clock
	
	TIM8->ARR = *TrainerPulsePtr++ ;
	TIM8->PSC = (Peri2_frequency*Timer_mult2) / 2000000 - 1 ;		// 0.5uS from 30MHz
	TIM8->CCER = TIM_CCER_CC4E ;	
	TIM8->CCMR2 = TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE ;			// PWM mode 1
	TIM8->CCR4 = 600 ;		// 300 uS pulse
	TIM8->BDTR = TIM_BDTR_MOE ;
 	TIM8->EGR = 1 ;
	TIM8->DIER = TIM_DIER_UDE ;

	TIM8->SR &= ~TIM_SR_UIF ;				// Clear flag
	TIM8->SR &= ~TIM_SR_CC2IF ;				// Clear flag
	TIM8->DIER |= TIM_DIER_CC2IE ;
	TIM8->DIER |= TIM_DIER_UIE ;

	TIM8->CR1 = TIM_CR1_CEN ;
	NVIC_EnableIRQ(TIM8_CC_IRQn) ;
	NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn) ;
}

extern "C" void TIM8_CC_IRQHandler()
{
	TIM8->DIER &= ~TIM_DIER_CC2IE ;		// stop this interrupt
	TIM8->SR &= ~TIM_SR_CC2IF ;				// Clear flag

	setupTrainerPulses() ;

	TrainerPulsePtr = TrainerPpmStream ;
	TIM8->DIER |= TIM_DIER_UDE ;

	TIM8->SR &= ~TIM_SR_UIF ;					// Clear this flag
	TIM8->DIER |= TIM_DIER_UIE ;				// Enable this interrupt
}

extern "C" void TIM8_UP_TIM13_IRQHandler()
{
	TIM8->SR &= ~TIM_SR_UIF ;				// Clear flag

	TIM8->ARR = *TrainerPulsePtr++ ;
	if ( *TrainerPulsePtr == 0 )
	{
		TIM8->SR &= ~TIM_SR_CC2IF ;			// Clear this flag
		TIM8->DIER |= TIM_DIER_CC2IE ;	// Enable this interrupt
	}
}

// Trainer capture, PC8, Timer 3 channel 3
void init_trainer_capture()
{
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN ; 		// Enable portC clock
	configure_pins( 0x0100, PIN_PERIPHERAL | PIN_PORTC | PIN_PER_2 ) ;
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN ;		// Enable clock
	
	TIM3->ARR = 0xFFFF ;
	TIM3->PSC = (Peri1_frequency*Timer_mult1) / 2000000 - 1 ;		// 0.5uS
	TIM3->CR2 = 0 ;
	TIM3->CCMR2 = TIM_CCMR2_IC3F_0 | TIM_CCMR2_IC3F_1 | TIM_CCMR2_CC3S_0 ;
	TIM3->CCER = TIM_CCER_CC3E ;	 
	TIM3->SR &= ~TIM_SR_CC3IF ;				// Clear flag
	TIM3->DIER |= TIM_DIER_CC3IE ;
	TIM3->CR1 = TIM_CR1_CEN ;
	NVIC_EnableIRQ(TIM3_IRQn) ;
	
}


extern "C" void TIM3_IRQHandler()
{
  
	uint16_t capture ;
  static uint16_t lastCapt ;
  uint16_t val ;
	
	capture = TIM3->CCR3 ;

  val = (uint16_t)(capture - lastCapt) / 2 ;
  lastCapt = capture;
	

  // We prcoess g_ppmInsright here to make servo movement as smooth as possible
  //    while under trainee control
  if ((val>4000) && (val < 19000)) // G: Prioritize reset pulse. (Needed when less than 8 incoming pulses)
	{
    ppmInState = 1; // triggered
	}
  else
  {
  	if(ppmInState && (ppmInState<=8))
		{
    	if((val>800) && (val<2200))
			{
  	    g_ppmIns[ppmInState++ - 1] = (int16_t)(val - 1500)*(g_eeGeneral.PPM_Multiplier+10)/10; //+-500 != 512, but close enough.

	    }else{
  	    ppmInState=0; // not triggered
    	}
    }
  }
}



#endif







