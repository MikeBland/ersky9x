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




void init5msTimer( void ) ;
void stop5msTimer( void ) ;

#ifdef PCBSKY
void start_timer0( void ) ;
void stop_timer0( void ) ;
#endif

#ifdef PCBX9D
extern uint32_t Peri1_frequency ;
extern uint32_t Peri2_frequency ;
extern uint32_t Timer_mult1 ;
extern uint32_t Timer_mult2 ;
extern void init_hw_timer( void ) ;
extern void hw_delay( uint16_t time ) ;
extern void init_main_ppm( void ) ;
extern void disable_main_ppm( void ) ;
extern void init_trainer_ppm( void ) ;
extern void init_trainer_capture( void ) ;
extern void init_pxx( void ) ;
extern void setupPulsesPXX( void ) ;
extern void setupPulses( void ) ;
extern void setupPulsesPpm( void ) ;
extern void disable_pxx( void ) ;

#endif


