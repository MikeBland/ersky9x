/*
 * Authors (alphabetical order)
 * - Andre Bernet <bernet.andre@gmail.com>
 * - Bertrand Songis <bsongis@gmail.com>
 * - Bryan J. Rentoul (Gruvin) <gruvin@gmail.com>
 * - Cameron Weeks <th9xer@gmail.com>
 * - Erez Raviv
 * - Jean-Pierre Parisy
 * - Karl Szmutny <shadow@privy.de>
 * - Michael Blandford
 * - Michal Hlavinka
 * - Pat Mackenzie
 * - Philip Moss
 * - Rob Thomson
 * - Romolo Manfredini <romolo.manfredini@gmail.com>
 * - Thomas Husterer
 *
 * open9x is based on code named
 * gruvin9x by Bryan J. Rentoul: http://code.google.com/p/gruvin9x/,
 * er9x by Erez Raviv: http://code.google.com/p/er9x/,
 * and the original (and ongoing) project by
 * Thomas Husterer, th9x: http://code.google.com/p/th9x/
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

#include "ersky9x.h"
#include "logicio.h"
#include "AT91SAM3S4.h"

uint32_t check_soft_power()
{
#ifdef SIMU
  return POWER_ON;
#endif

#ifdef REVB	
  if ( PIOC->PIO_PDSR & PIO_PC17 )  // Power on
  {
	  return POWER_ON;
  }

  if ( PIOA->PIO_PDSR & PIO_PA8 )   // Trainer plugged in
  {
		return POWER_TRAINER ;
  }
#endif

  return POWER_OFF;
}

// turn off soft power
void soft_power_off()
{
#ifdef REVB	
  configure_pins( PIO_PA8, PIN_ENABLE | PIN_OUTPUT | PIN_LOW | PIN_PORTA | PIN_NO_PULLUP | PIN_PULLDOWN ) ;
#endif
}

#ifdef REVB	
void init_soft_power()
{
  // Configure RF_power (PC17)
  configure_pins( PIO_PC17, PIN_ENABLE | PIN_INPUT | PIN_PORTC | PIN_NO_PULLUP | PIN_PULLDOWN ) ;
  configure_pins( PIO_PA8, PIN_ENABLE | PIN_INPUT | PIN_PORTA | PIN_PULLUP ) ; // Enable bit A8 (Soft Power)
}
#endif
