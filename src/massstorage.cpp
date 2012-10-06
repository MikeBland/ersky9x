/*
 * Authors (alphabetical order)
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
#include "board.h"
#include "debug.h"
#include "Audio.h"

#ifndef SIMU
#include "CoOS.h"
#endif

#ifndef SIMU

extern "C" {
#include "usb/device/massstorage/MSDDriver.h"
extern unsigned char MEDSdcard_Initialize(Media *media, unsigned char mciID);
}

/** Size of one block in bytes. */
#define BLOCK_SIZE          512

/** Size of the MSD IO buffer in bytes (6K, more the better). */
#define MSD_BUFFER_SIZE     (12*BLOCK_SIZE)

/** LUN read/write buffer. */
unsigned char msdBuffer[MSD_BUFFER_SIZE];

unsigned int msdReadTotal=0, msdWriteTotal=0;

static void ConfigureUsbClock(void)
{
    /* Enable PLLB for USB */
    PMC->CKGR_PLLBR = CKGR_PLLBR_DIVB(1)
                    | CKGR_PLLBR_MULB(7)
                    | CKGR_PLLBR_PLLBCOUNT;
    while((PMC->PMC_SR & PMC_SR_LOCKB) == 0); // TODO  && (timeout++ < CLOCK_TIMEOUT));
    /* USB Clock uses PLLB */
    PMC->PMC_USB = PMC_USB_USBDIV(1)    /* /2   */
                 | PMC_USB_USBS;        /* PLLB */
}

/** Maximum number of LUNs which can be defined. */
#define MAX_LUNS            1

/** Media index for different disks */
#define DRV_SDMMC           0    /** SD card */

Media medias[MAX_LUNS];

/*----------------------------------------------------------------------------
 *        Local variables
 *----------------------------------------------------------------------------*/

/** Device LUNs. */
MSDLun luns[MAX_LUNS];

static void MSDCallbacks_Data( unsigned char flowDirection, unsigned int dataLength,
                               unsigned int fifoNullCount, unsigned int fifoFullCount )
{
#if 0
  if (flowDirection)
    msdReadTotal += dataLength;
  else
    msdWriteTotal += dataLength;
#endif
}

void usbMassStorage()
{
  static bool initialized = false ;
  static bool active = false ;

  if ( PIOC->PIO_PDSR & 0x02000000 )
	{
    // TRACE_DEBUG("usbMassStorage\n\r");

    if (!initialized)
		{
      ConfigureUsbClock();
      /* Initialize LUN */
      MEDSdcard_Initialize(&(medias[DRV_SDMMC]), 0);
      LUN_Init(&(luns[DRV_SDMMC]), &(medias[DRV_SDMMC]),
          msdBuffer, MSD_BUFFER_SIZE,
          0, 0, 0, 0,
          MSDCallbacks_Data);
      /* BOT driver initialization */
      MSDDriver_Initialize(luns, 1);
      // VBus_Configure();
      USBD_Connect();
      initialized = true;
    }

  	if ( active == false )
		{
			CoSchedLock() ;
			if ( Voice.VoiceLock == 0 )
			{
				Voice.VoiceLock = 1 ;
				active = true ;
			}
  		CoSchedUnlock() ;
		}

  	if ( active )
		{
    	/* Mass storage state machine */
    	for (uint8_t i=0; i<50; i++)
			{
    	  MSDDriver_StateMachine() ;
			}
		}
  }
  else
	{
  	if ( active )
		{
  		active = false ;
			Voice.VoiceLock = 0 ;
		}
		
    msdReadTotal = 0;
    msdWriteTotal = 0;
  }
}

#endif
