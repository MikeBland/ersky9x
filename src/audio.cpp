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
#include "ersky9x.h"
#include "audio.h"
#include "sound.h"
#include "myeeprom.h"

//#define SPEAKER_OFF  PORTE &= ~(1 << OUT_E_BUZZER) // speaker output 'low'


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
		}    
  }
  else
	{
	
	hapticOff();	
		
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
	if ( g_eeGeneral.speakerMode == 0 )
	{
		buzzer_sound( 4 ) ;
	}
	else if ( g_eeGeneral.speakerMode == 1 )
	{
		audio.event(e, BEEP_DEFAULT_FREQ);
//		playTone( 2000, 60 ) ;		// 2KHz, 60mS
	}
}
