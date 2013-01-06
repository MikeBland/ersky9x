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
extern void init_hw_timer( void ) ;
extern void hw_delay( uint16_t time ) ;

#endif


