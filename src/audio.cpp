/*
 * Author - Rob Thomson & Bertrand Songis
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
#if !defined(SIMU)
// Mike I think this include is not needed (already present in diskio.h)
#include "core_cm3.h"
#endif
#include "ersky9x.h"
#include "audio.h"
#include "sound.h"
#include "myeeprom.h"
#include "diskio.h"
#include "ff.h"

#ifndef SIMU
#include "CoOS.h"
#endif

//#define SPEAKER_OFF  PORTE &= ~(1 << OUT_E_BUZZER) // speaker output 'low'

struct t_voice Voice ;

audioQueue::audioQueue()
{
  aqinit();
}

// TODO should not be needed
void audioQueue::aqinit()
{
  //make sure haptic off by default
//  HAPTIC_OFF;

  toneTimeLeft = 0;
  tonePause = 0;

  t_queueRidx = 0;
  t_queueWidx = 0;

  toneHaptic = 0;
  hapticTick = 0;
  hapticMinRun = 0;

}

bool audioQueue::busy()
{
  return (toneTimeLeft > 0);
}


bool audioQueue::freeslots()
{
	uint8_t temp ;
	temp = t_queueWidx ;
	temp += AUDIO_QUEUE_LENGTH ;
	temp -= t_queueRidx ;
	temp %= AUDIO_QUEUE_LENGTH ;
	temp = AUDIO_QUEUE_LENGTH - temp ;
	return temp >= AUDIO_QUEUE_FREESLOTS ;
//  return AUDIO_QUEUE_LENGTH - ((t_queueWidx + AUDIO_QUEUE_LENGTH - t_queueRidx) % AUDIO_QUEUE_LENGTH) >= AUDIO_QUEUE_FREESLOTS;
}


// heartbeat is responsibile for issueing the audio tones and general square waves
// it is essentially the life of the class.
// it is called every 10ms
void audioQueue::heartbeat()
{
  //haptic switch off happens in separate loop
  if(hapticMinRun == 0){
	hapticOff();	
  } else {
	hapticMinRun--;	
  }	
	
  if (toneTimeLeft )
	{
		
		if ( queueTone( toneFreq * 61 / 2, toneTimeLeft * 10, toneFreqIncr * 61 / 2 ))
		{
    			toneTimeLeft = 0 ; //time gets counted down
		}

		//this works - but really needs a delay added in.
		// reason is because it takes time for the motor to spin up
		// need to take this into account when the tone sent is really short!
		// initial thoughts are a seconds queue to process haptic that gets
		// fired from here.  end result is haptic events run for mix of 2 seconds?
		
		if (toneHaptic){
	    		hapticOn((g_eeGeneral.hapticStrength *  2 ) * 10); 
	    		hapticMinRun = HAPTIC_SPINUP;
		}    
  }
  else
	{
	
		
    if ( tonePause )
		{
			if ( queueTone( 0, tonePause * 10, 0 ) )
			{
    		tonePause = 0 ; //time gets counted down
			}
 		}
		else
		{  
      if (t_queueRidx != t_queueWidx)
			{
        toneFreq = queueToneFreq[t_queueRidx];
        toneTimeLeft = queueToneLength[t_queueRidx];
        toneFreqIncr = queueToneFreqIncr[t_queueRidx];
        tonePause = queueTonePause[t_queueRidx];
        toneHaptic = queueToneHaptic[t_queueRidx];
        hapticTick = 0;
        if (!queueToneRepeat[t_queueRidx]--)
				{
          t_queueRidx = (t_queueRidx + 1) % AUDIO_QUEUE_LENGTH;
        }
      }
    }
  }
}

inline uint8_t audioQueue::getToneLength(uint8_t tLen)
{
  uint8_t result = tLen; // default
  if (g_eeGeneral.beeperVal == 2) {
    result /= 3;
  }
  else if (g_eeGeneral.beeperVal == 3) {
    result /= 2;
  }
  else if (g_eeGeneral.beeperVal == 5) {
    //long
    result *= 2;
  }
  else if (g_eeGeneral.beeperVal == 6) {
    //xlong
    result *= 3;
  }
  return result;
}

void audioQueue::playNow(uint8_t tFreq, uint8_t tLen, uint8_t tPause,
    uint8_t tRepeat, uint8_t tHaptic, int8_t tFreqIncr)
{
	
	if(!freeslots()){
			return;
	}
	
  if (g_eeGeneral.beeperVal) {
    toneFreq = (tFreq ? tFreq + g_eeGeneral.speakerPitch + BEEP_OFFSET : 0); // add pitch compensator
    toneTimeLeft = getToneLength(tLen);
    tonePause = tPause;
    toneHaptic = tHaptic;
    hapticTick = 0;
    toneFreqIncr = tFreqIncr;
    t_queueWidx = t_queueRidx;

    if (tRepeat) {
      playASAP(tFreq, tLen, tPause, tRepeat-1, tHaptic, tFreqIncr);
    }
  }
}

void audioQueue::playASAP(uint8_t tFreq, uint8_t tLen, uint8_t tPause,
    uint8_t tRepeat, uint8_t tHaptic, int8_t tFreqIncr)
{
	if(!freeslots()){
			return;
	}	
	
  if (g_eeGeneral.beeperVal) {
    uint8_t next_queueWidx = (t_queueWidx + 1) % AUDIO_QUEUE_LENGTH;
    if (next_queueWidx != t_queueRidx) {
      queueToneFreq[t_queueWidx] = (tFreq ? tFreq + g_eeGeneral.speakerPitch + BEEP_OFFSET : 0); // add pitch compensator
      queueToneLength[t_queueWidx] = getToneLength(tLen);
      queueTonePause[t_queueWidx] = tPause;
      queueToneHaptic[t_queueWidx] = tHaptic;
      queueToneRepeat[t_queueWidx] = tRepeat;
      queueToneFreqIncr[t_queueWidx] = tFreqIncr;
      t_queueWidx = next_queueWidx;
    }
  }
}

void audioQueue::event(uint8_t e, uint8_t f) {

  uint8_t beepVal = g_eeGeneral.beeperVal;
	if (t_queueRidx == t_queueWidx) {		
	  switch (e) {
		    case AU_WARNING1:
		      playNow(BEEP_DEFAULT_FREQ, 10, 1, 0, 1);
		      break;
		    case AU_WARNING2:
		      playNow(BEEP_DEFAULT_FREQ, 20, 1, 0, 1);
		      break;
		    case AU_WARNING3:
		      playNow(BEEP_DEFAULT_FREQ, 30, 1, 0, 1);
		      break;
	      case AU_CHEEP:
	        playASAP(BEEP_DEFAULT_FREQ+30,10,2,2,1,2);
	        break;
	      case AU_RING:
	        playASAP(BEEP_DEFAULT_FREQ+25,5,2,10,1);
	        playASAP(BEEP_DEFAULT_FREQ+25,5,10,1,1);
	        playASAP(BEEP_DEFAULT_FREQ+25,5,2,10,1);
	        break;
	      case AU_SCIFI:
	        playASAP(80,10,3,2,0,-1);
	        playASAP(60,10,3,2,0,1);
	        playASAP(70,10,1,0,2);
	        break;
	      case AU_ROBOT:
	        playASAP(70,5,1,1,1);
	        playASAP(50,15,2,1,1);
	        playASAP(80,15,2,1,1);
	        break;
	      case AU_CHIRP:
	        playASAP(BEEP_DEFAULT_FREQ+40,5,1,2,1);
	        playASAP(BEEP_DEFAULT_FREQ+54,5,1,3,1);
	        break;
	      case AU_TADA:
	        playASAP(50,5,5);
	        playASAP(90,5,5);
	        playASAP(110,3,4,2);
	        break;
	      case AU_CRICKET:
	        playASAP(80,5,10,3,1);
	        playASAP(80,5,20,1,1);
	        playASAP(80,5,10,3,1);
	        break;
	      case AU_SIREN:
	        playASAP(10,20,5,2,1,1);
	        break;
	      case AU_ALARMC:
	        playASAP(50,4,10,2,1);
	        playASAP(70,8,20,1,1);
	        playASAP(50,8,10,2,1);
	        playASAP(70,4,20,1,1);
	        break;
	      case AU_RATATA:
	        playASAP(BEEP_DEFAULT_FREQ+50,5,10,10,1);
	        break;
	      case AU_TICK:
	        playASAP(BEEP_DEFAULT_FREQ+50,5,50,2,1);
	        break;
	      case AU_HAPTIC1:
	        playASAP(0,20,10,1,1);
	        break;
	      case AU_HAPTIC2:
	        playASAP(0,20,10,2,1);
	        break;
	      case AU_HAPTIC3:
	        playASAP(0,20,10,3,1);
	        break;
		    case AU_ERROR:
		      playNow(BEEP_DEFAULT_FREQ, 40, 1, 0, 1);
		      break;
		    case AU_KEYPAD_UP:
		      if (beepVal != BEEP_NOKEYS) {
		        playNow(BEEP_KEY_UP_FREQ, 10, 1);
		      }
		      break;
		    case AU_KEYPAD_DOWN:
		      if (beepVal != BEEP_NOKEYS) {
		        playNow(BEEP_KEY_DOWN_FREQ, 10, 1);
		      }
		      break;
		    case AU_TRIM_MOVE:
		      playNow(f, 6, 1);
		      break;
		    case AU_TRIM_MIDDLE:
		      playNow(BEEP_DEFAULT_FREQ, 20, 2, 0, 1);
		      break;
		    case AU_MENUS:
		      if (beepVal != BEEP_NOKEYS) {
		        playNow(BEEP_DEFAULT_FREQ, 10, 2, 0, 0);
		      }
		      break;
		    case AU_POT_STICK_MIDDLE:
		      playNow(BEEP_DEFAULT_FREQ + 50, 10, 1, 0, 0);
		      break;
		    case AU_MIX_WARNING_1:
		      playNow(BEEP_DEFAULT_FREQ + 50, 10, 1, 1, 1);
		      break;
		    case AU_MIX_WARNING_2:
		      playNow(BEEP_DEFAULT_FREQ + 52, 10, 1, 2, 1);
		      break;
		    case AU_MIX_WARNING_3:
		      playNow(BEEP_DEFAULT_FREQ + 54, 10, 1, 3, 1);
		      break;
		    case AU_TIMER_30:
		      playNow(BEEP_DEFAULT_FREQ + 50, 15, 3, 3, 1);
		      break;
		    case AU_TIMER_20:
		      playNow(BEEP_DEFAULT_FREQ + 50, 15, 3, 2, 1);
		      break;
		    case AU_TIMER_10:
		      playNow(BEEP_DEFAULT_FREQ + 50, 15, 3, 1, 1);
		      break;
		    case AU_TIMER_LT3:
		      playNow(BEEP_DEFAULT_FREQ, 20, 25, 1, 1);
		      break;
		    case AU_INACTIVITY:
		      playNow(70, 10, 2,2);
		      break;
		    case AU_TX_BATTERY_LOW:
		        playASAP(60, 20, 3, 2, 0, 1);
		        playASAP(80, 20, 3, 2, 1, -1);
		      break;
		    default:
		      break;
	  }
	}  
}

void audioDefevent(uint8_t e)
{
	if ( (g_eeGeneral.speakerMode & 1) == 0 )
	{
		buzzer_sound( 4 ) ;
	}
	else if ( ( g_eeGeneral.speakerMode & 1 )== 1 )
	{
		audio.event(e, BEEP_DEFAULT_FREQ);
//		playTone( 2000, 60 ) ;		// 2KHz, 60mS
	}
}

void audioVoiceDefevent( uint8_t e, uint8_t v)
{
	if ( ( g_eeGeneral.speakerMode & 2 ) && Voice.VoiceLock == 0 )
	{
		putVoiceQueue( v ) ;
	}
	else
	{
    audioDefevent( e ) ;
	}
}



// Announce a value using voice
void voice_numeric( uint16_t value, uint8_t num_decimals, uint8_t units_index )
{
	uint8_t decimals = 0 ;
	div_t qr ;

	if ( num_decimals )
	{
		qr = div( value, num_decimals == 2 ? 100 : 10 ) ;
		decimals = qr.rem ;
		value = qr.quot ;
	}

	qr = div( value, 100 ) ;
	if ( qr.quot )
	{
		if ( qr.quot > 9 )		// Thousands
		{
			qr = div( qr.quot, 10 ) ;
			putVoiceQueue( qr.quot + 110 ) ;
			qr.quot = qr.rem ;			
		}
		putVoiceQueue( qr.quot + 100 ) ;
		putVoiceQueue( qr.rem + 400 ) ;
	}
	else
	{
		putVoiceQueue( qr.rem + 400 ) ;
	}

	if ( num_decimals )
	{
		if ( num_decimals == 2 )
		{
			qr = div( decimals, 10 ) ;
			putVoiceQueue( qr.quot + 6 ) ;		// Point x
			putVoiceQueue( qr.rem + 400 ) ;
		}
		else
		{
			putVoiceQueue( decimals + 6 ) ;		// Point x
		}
	}
		 
	if ( units_index )
	{
		putVoiceQueue( units_index ) ;
	}
}

void putVoiceQueue( uint16_t value )
{
	struct t_voice *vptr ;
	vptr = &Voice ;
	
	if ( vptr->VoiceQueueCount < VOICE_Q_LENGTH )
	{
		vptr->VoiceQueue[vptr->VoiceQueueInIndex++] = value ;
		vptr->VoiceQueueInIndex &= ( VOICE_Q_LENGTH - 1 ) ;
		__disable_irq() ;
		vptr->VoiceQueueCount += 1 ;
		__enable_irq() ;
	}
}

TCHAR VoiceFilename[48] ;
uint8_t FileData[1024] ;
FATFS g_FATFS ;
FIL Vfile ;
uint32_t SDlastError ;

void voice_task(void* pdata)
{
	uint32_t v_index ;
	FRESULT fr ;
	UINT nread ;
	uint32_t x ;
	uint32_t w8or16 ;
	uint32_t mounted = 0 ;
	uint32_t size ;

	for(;;)
	{
		while ( !sd_card_ready() )
		{
			CoTickDelay(5) ;					// 10mS for now
		}
		if ( mounted == 0 )
		{
  		fr = f_mount(0, &g_FATFS) ;
		}
		else
		{
			fr = FR_OK ;
		}

		if ( fr == FR_OK)
		{
			mounted = 1 ;
	
			while ( Voice.VoiceQueueCount == 0 )
			{
				CoTickDelay(3) ;					// 6mS for now
			}

			v_index = Voice.VoiceQueue[Voice.VoiceQueueOutIndex++] ;
			
			CoSchedLock() ;
			if ( Voice.VoiceLock == 0 )
			{
				Voice.VoiceLock = 1 ;
  			CoSchedUnlock() ;

				{	// Create filename
					TCHAR *ptr ;
					ptr = (TCHAR *)cpystr( ( uint8_t*)VoiceFilename, ( uint8_t*)"\\voice\\" ) ;
					*ptr++ = '0' ;
					*(ptr + 2) = '0' + v_index % 10 ;
					x = v_index / 10 ;
					*(ptr + 1) = '0' + x % 10 ;
					x /= 10 ;
					*ptr = '0' + x % 10 ;
					cpystr( ( uint8_t*)(ptr+3), ( uint8_t*)".wav" ) ;
				}
					
				fr = f_open( &Vfile, VoiceFilename, FA_READ ) ;
				if ( fr == FR_OK )
				{
					fr = f_read( &Vfile, FileData, 1024, &nread ) ;
					x = FileData[34] + ( FileData[35] << 8 ) ;		// sample size
					w8or16 = x ;
					x = FileData[24] + ( FileData[25] << 8 ) ;		// sample rate
					size = FileData[40] + ( FileData[41] << 8 ) + ( FileData[42] << 16 ) ;		// data size

					size -= 512-44 ;
					if ( w8or16 == 8 )
					{
						wavU8Convert( &FileData[44], VoiceBuffer[0].data, 512-44 ) ;
						VoiceBuffer[0].count = 512-44 ;
					}
					else if ( w8or16 == 16 )
					{
						wavU16Convert( (uint16_t*)&FileData[44], VoiceBuffer[0].data, 512-44/2 ) ;
						VoiceBuffer[0].count = 512-44/2 ;
						size -= 512 ;
					}
					else
					{
						w8or16 = 0 ;		// can't convert
					}
				
					if ( w8or16 )
					{
						uint32_t amount ;
						VoiceBuffer[0].frequency = x ;		// sample rate

						if ( w8or16 == 8 )
						{
							wavU8Convert( &FileData[512], VoiceBuffer[1].data, 512 ) ;
							size -= 512 ;
						}
						else
						{
							fr = f_read( &Vfile, (uint8_t *)FileData, 1024, &nread ) ;
							wavU16Convert( (uint16_t*)&FileData[0], VoiceBuffer[1].data, 512 ) ;
							size -= nread ;
						}
						VoiceBuffer[1].count = 512 ;
						VoiceBuffer[1].frequency = 0 ;
					
						amount = (w8or16 == 8) ? 512 : 1024 ;

						fr = f_read( &Vfile, (uint8_t *)FileData, amount, &nread ) ;		// Read next buffer
						if ( w8or16 == 8 )
						{
							wavU8Convert( &FileData[0], VoiceBuffer[2].data, 512 ) ;
						}
						else
						{
							wavU16Convert( (uint16_t*)&FileData[0], VoiceBuffer[2].data, 512 ) ;
						}
						size -= nread ;
						VoiceBuffer[2].count = 512 ;
						VoiceBuffer[2].frequency = 0 ;
						startVoice( 3 ) ;
						for(x = 0;;)
						{
							if ( size < amount )
							{
								amount = size ;								
							}
							fr = f_read( &Vfile, (uint8_t *)FileData, amount, &nread ) ;		// Read next buffer
							size -= nread ;
							if ( nread == 0 )
							{
								break ;
							}
	  					while ( ( VoiceBuffer[x].flags & VF_SENT ) == 0 )
							{
								CoTickDelay(1) ;					// 2mS for now
							}
							if ( w8or16 == 8 )
							{
								wavU8Convert( &FileData[0], VoiceBuffer[x].data, nread ) ;
							}
							else
							{
								nread /= 2 ;
								wavU16Convert( (uint16_t*)&FileData[0], VoiceBuffer[x].data, nread ) ;
							}
							VoiceBuffer[x].count = nread ;
							VoiceBuffer[x].frequency = 0 ;
							appendVoice( x ) ;					// index of next buffer
							v_index = x ;		// Last buffer sent
							x += 1 ;
							if ( x > 2 )
							{
								x = 0 ;							
							}
							if ( (int32_t)size <= 0 )
							{
								break ;								
							}
						}
					}
					fr = f_close( &Vfile ) ;
					// Now wait for last buffer to have been sent
					x = 100 ;
 					while ( ( VoiceBuffer[v_index].flags & VF_SENT ) == 0 )
					{
						CoTickDelay(1) ;					// 2mS for now
						if ( --x == 0 )
						{
							break ;		// Timeout, 200 mS
						}
					}
				}
				else
				{
					SDlastError = fr ;
					mounted = 0 ;
				}
				Voice.VoiceLock = 0 ;
			}
			else
			{
  			CoSchedUnlock() ;
			}
			Voice.VoiceQueueOutIndex &= ( VOICE_Q_LENGTH - 1 ) ;
			__disable_irq() ;
			Voice.VoiceQueueCount -= 1 ;
			__enable_irq() ;
		}
		else
		{
			SDlastError = fr ;
		}
		CoTickDelay(1) ;					// 2mS for now
	} // for(;;)
}

