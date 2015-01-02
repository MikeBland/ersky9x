/*
 * Author - Mike Blandford
 *
 * Based on er9x by Erez Raviv <erezraviv@gmail.com>
 *
 * Based on th9x -> http://code.google.com/p/th9x/
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
#include <string.h>

#ifdef PCBSKY
#include "AT91SAM3S2.h"
#endif
#include "ersky9x.h"
#include "myeeprom.h"
#include "audio.h"
#include "sound.h"
#include "lcd.h"
#include "debug.h"
#include "menus.h"
#include "drivers.h"
#include "logicio.h"
#include "file.h"
#include "stringidx.h"
#include "templates.h"
#include "pulses.h"
#ifdef FRSKY
#include "frsky.h"
#endif
#ifndef SIMU
#include "CoOS.h"
#endif

#ifdef PCBX9D
#include "X9D/eeprom_rlc.h"
#include "timers.h"
#endif
#include "ff.h"
#include "maintenance.h"

#ifdef REVX
 #include "jeti.h"
#endif

union t_xmem Xmem ;


extern int32_t Rotary_diff ;
extern int16_t AltOffset ;

static uint8_t s_currIdx;
static uint8_t RestoreIndex ;
static uint8_t SubMenuFromIndex = 0 ;

#ifdef REVX
uint16_t jeti_keys = JETI_KEY_NOCHANGE;
uint8_t JetiBuffer[36] ; // 32 characters
uint8_t JetiIndex ;
uint8_t JetiBufferReady ;
#endif

struct t_timer s_timer[2] ;

uint8_t RotaryState ;		// Defaults to ROTARY_MENU_LR
uint8_t CalcScaleNest = 0 ;

uint8_t ThrottleStickyOn = 0 ;

//uint8_t MainPopup ;
//uint8_t MixPopup ;
//uint8_t ModelPopup ;

//#define MAIN_POPUP		1
//#define MIX_POPUP			3
//#define MODEL_POPUP		4

struct t_popupData PopupData ;

static uint8_t SubmenuIndex = 0 ;
static uint8_t IlinesCount ;
static uint8_t LastSubmenuIndex = 0 ;
static uint8_t UseLastSubmenuIndex = 0 ;

uint8_t FileSelectResult ;
char SelectedVoiceFileName[16] ;
void menuProcSelectMvoiceFile(uint8_t event) ;
void menuProcSelectUvoiceFile(uint8_t event) ;

//const char *Str_Switch_warn = PSTR(STR_SWITCH_WARN) ;

//const char *Str_ALTeq =  PSTR(STR_ALTEQ) ;
//const char *Str_TXeq =  PSTR(STR_TXEQ) ;
//const char *Str_RXeq =  PSTR(STR_RXEQ) ;
//const char *Str_TRE012AG =  PSTR(STR_TRE012AG) ;
//const char *Str_YelOrgRed = PSTR(STR_YELORGRED) ;
//const char *Str_A_eq =  PSTR(STR_A_EQ) ;
//const char *Str_Timer =  PSTR(STR_TIMER) ;
//const char *Str_Sounds = PSTR(STR_SOUNDS) ;

//const char *Str_PpmChannels = PSTR(STR_PPMCHANNELS) ;

int8_t phyStick[4] ;

const uint8_t UnitsVoice[] = {V_FEET,V_VOLTS,V_DEGREES,V_DEGREES,0,V_AMPS,V_METRES,V_WATTS,V_PERCENT } ;
const uint8_t UnitsText[] = { 'F','V','C','F','m','A','m','W','%' } ;
const uint8_t UnitsString[] = "\005Feet VoltsDeg_CDeg_FmAh  Amps MetreWattsPcent" ;

// TSSI set to zero on no telemetry data
//const char *Str_telemItems = PSTR(STR_TELEM_ITEMS) ;
const int8_t TelemIndex[] = { FR_A1_COPY, FR_A2_COPY,
															FR_RXRSI_COPY, FR_TXRSI_COPY,
															TIMER1, TIMER2,
															FR_ALT_BARO, FR_GPS_ALT,
															FR_GPS_SPEED, FR_TEMP1, FR_TEMP2, FR_RPM,
														  FR_FUEL, FR_A1_MAH, FR_A2_MAH, FR_CELL_MIN,
															BATTERY, FR_CURRENT, FR_AMP_MAH, FR_CELLS_TOT, FR_VOLTS,
															FR_ACCX, FR_ACCY,	FR_ACCZ, FR_VSPD, V_GVAR1, V_GVAR2,
															V_GVAR3, V_GVAR4, V_GVAR5, V_GVAR6, V_GVAR7, FR_WATT, FR_RXV, FR_COURSE,
															FR_A3, FR_A4, V_SC1, V_SC2, V_SC3, V_SC4, V_SC5, V_SC6, V_SC7, V_SC8, V_RTC, TMOK } ;

// TXRSSI is always considered valid as it is forced to zero on loss of telemetry
// Values are 0 - always valid, 1 - need telemetry, 2 - need hub
//const uint8_t TelemValid[] = { 1, 1, 1, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 0, 0, 2, 0, 2, 0, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 2, 1, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 0  } ;

// Make sclaers always OK!
const uint8_t TelemValid[] = { 1, 1, 1, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 0, 0, 2, 0, 2, 0, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 2, 1, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0  } ;

int8_t edit_dr_switch( uint8_t x, uint8_t y, int8_t drswitch, uint8_t attr, uint8_t flags, uint8_t event ) ;

void putsAttIdxTelemItems( uint8_t x, uint8_t y, uint8_t index, uint8_t attr )
{
#ifdef PCBSKY
	if ( index == 4 )
	{
		if ( g_model.protocol == PROTO_PXX )
		{
			lcd_putsAtt( x, y, XPSTR("SWR "), attr ) ;
			return ;
		}
	}
#endif
	lcd_putsAttIdx( x, y, PSTR(STR_TELEM_ITEMS), index, attr ) ;
}

int16_t m_to_ft( int16_t metres )
{
  // m to ft *105/32
  return metres * 3 + ( metres >> 2 ) + (metres >> 5) ;
}

int16_t c_to_f( int16_t degrees )
{
  degrees += 18 ;
  degrees *= 115 ;
  degrees >>= 6 ;
  return degrees ;
}

int16_t calc_scaler( uint8_t index, uint8_t *unit, uint8_t *num_decimals)
{
	int32_t value ;
	ScaleData *pscaler ;
	
	if ( CalcScaleNest > 5 )
	{
		return 0 ;
	}
	CalcScaleNest += 1 ;
	// process
	pscaler = &g_model.Scalers[index] ;
	if ( pscaler->source )
	{
		value = getValue( pscaler->source - 1 ) ;
		if ( ( pscaler->source == NUM_SKYXCHNRAW+1 ) || ( pscaler->source == NUM_SKYXCHNRAW+2 ) )
		{
			value = scale_telem_value( value, pscaler->source - NUM_SKYXCHNRAW-1, NULL ) ;
		}
	}
	else
	{
		value = 0 ;
	}
	if ( pscaler->offsetLast == 0 )
	{
		value += pscaler->offset ;
	}
	value *= pscaler->mult+1 ;
	value /= pscaler->div+1 ;
	if ( pscaler->offsetLast )
	{
		value += pscaler->offset ;
	}
	if ( pscaler->neg )
	{
		value = -value ;
	}
	if ( unit )
	{
		*unit = pscaler->unit ;
	}
	if ( num_decimals )
	{
		*num_decimals = pscaler->precision ;
	}

	CalcScaleNest -= 1 ;
	return value ;
}
									 
uint8_t telemItemValid( uint8_t index )
{
#ifdef FRSKY
	uint8_t x ;

	x = pgm_read_byte( &TelemValid[index] ) ;
	if ( x == 3 )
	{
		// A scaler
		uint8_t i = g_model.Scalers[index-TEL_ITEM_SC1].source - 1 ;
  	if(i<CHOUT_BASE+NUM_SKYCHNOUT)
		{
			x = 0 ;
		}
		else
		{
			i -= CHOUT_BASE-NUM_SKYCHNOUT ;
			x = pgm_read_byte( &TelemValid[i] ) ;
			if ( x == 3 )
			{
				x = 0 ;
			}
		}
	}
	if ( x == 0 )
	{
		return 1 ;
	}
	if ( x == 1 )
	{
		if ( frskyStreaming )
		{
			return 1 ;
		}
	}
	if ( frskyUsrStreaming )
	{
		return 1 ;
	}
	return 0 ;	
#else
	return 1 ;
#endif
}

void voice_telem_item( int8_t index )
{
	int16_t value ;
	uint8_t spoken = 0 ;
	uint8_t unit = 0 ;
	uint8_t num_decimals = 0 ;
#ifdef FRSKY
//	uint8_t att = 0 ;
#endif

	value = get_telemetry_value( index ) ;
	if (telemItemValid( index ) == 0 )
	{
//#ifdef TELEMETRY_LOST
//		if ( frskyStreaming )
//		{
//#endif
		putSystemVoice( SV_NO_TELEM, V_NOTELEM ) ;
//#ifdef TELEMETRY_LOST
//		}
//#endif
		spoken = 1 ;
	}
	index = pgm_read_byte( &TelemIndex[index] ) ;

  switch (index)
	{
		case V_SC1 :
		case V_SC2 :
		case V_SC3 :
		case V_SC4 :
		case V_SC5 :
		case V_SC6 :
		case V_SC7 :
		case V_SC8 :
			value = calc_scaler( index-V_SC1, &unit, &num_decimals ) ;
			unit = UnitsVoice[unit] ;
		break ;
		
		case BATTERY:
		case FR_VOLTS :
		case FR_CELLS_TOT :
			unit = V_VOLTS ;			
			num_decimals = 1 ;
		break ;

		case FR_CELL_MIN:
			unit = V_VOLTS ;			
			num_decimals = 2 ;
		break ;
			
		case TIMER1 :
		case TIMER2 :
		{	
			div_t qr ;
			qr = div( value, 60 ) ;
			voice_numeric( qr.quot, 0, V_MINUTES ) ;
			value = qr.rem ;
			unit = V_SECONDS ;			
		}
		break ;
		
		case V_GVAR1 :
		case V_GVAR2 :
		case V_GVAR3 :
		case V_GVAR4 :
		case V_GVAR5 :
		case V_GVAR6 :
		case V_GVAR7 :
			value = g_model.gvars[index-V_GVAR1].gvar ;
		break ;
		
#ifdef FRSKY
		case FR_A1_COPY:
    case FR_A2_COPY:
		{	
			uint8_t channel = index-FR_A1_COPY ;
			
			uint8_t ltype = g_model.frsky.channels[channel].type ;
			value = A1A2toScaledValue( channel, &num_decimals ) ;
			unit = V_VOLTS ;			
			if (ltype == 3)
			{
				unit = V_AMPS ;
			}
			else if (ltype == 1)
			{
				unit = 0 ;
				num_decimals = 0 ;
			}
		}
		break ;

		case FR_ALT_BARO:
      unit = V_METRES ;
			if (g_model.FrSkyUsrProto == 1)  // WS How High
			{
      	if ( g_model.FrSkyImperial )
        	unit = V_FEET ;
			}
      else
			{
				if ( g_model.FrSkyImperial )
      	{
	        // m to ft *105/32
  	      value = m_to_ft( value ) ;
    	    unit = V_FEET ;
      	}
			}
			if ( value < 1000 )
			{
				num_decimals = 1 ;
			}
			else
			{
				value /= 10 ;
			}
		break ;
		 
		case FR_CURRENT :
			num_decimals = 1 ;
      unit = V_AMPS ;
		break ;
			
		case FR_TEMP1:
		case FR_TEMP2:
			unit = V_DEGREES ;			
  		if ( g_model.FrSkyImperial )
  		{
				value = c_to_f(value) ;
			}
		break ;

		case FR_WATT :
			unit = V_WATTS ;
		break ;

		case FR_FUEL :
			unit = V_PERCENT ;
		break ;

		case FR_VSPD :
			num_decimals = 1 ;
			value /= 10 ;
		break ;

#endif

	}

	if ( spoken == 0 )
	{
		voice_numeric( value, num_decimals, unit ) ;
	}
}


// This routine converts an 8 bit value for custom switch use
int16_t convertTelemConstant( int8_t channel, int8_t value)
{
  int16_t result;

	channel = TelemIndex[channel] ;
	result = value + 125 ;
	if ( ( channel <= V_GVAR7 ) && ( channel >= V_GVAR1 ) )
	{
		return value ;
	}

  switch (channel)
	{
    case V_RTC :
      result *= 12 ;
    break;
    case TIMER1 :
    case TIMER2 :
      result *= 10 ;
    break;
    case FR_ALT_BARO:
    case FR_GPS_ALT:
			if ( result > 63 )
			{
      	result *= 2 ;
      	result -= 64 ;
			}
			if ( result > 192 )
			{
      	result *= 2 ;
      	result -= 192 ;
			}
			if ( result > 448 )
			{
      	result *= 2 ;
      	result -= 488 ;
			}
			result *= 10 ;		// Allow for decimal place
      if ( g_model.FrSkyImperial )
      {
        // m to ft *105/32
        value = m_to_ft( result ) ;
      }
    break;
    case FR_RPM:
      result *= 100;
    break;
    case FR_TEMP1:
    case FR_TEMP2:
      result -= 30;
    break;
    case FR_A1_MAH:
    case FR_A2_MAH:
		case FR_AMP_MAH :
      result *= 50;
    break;

		case FR_CELL_MIN:
      result *= 2;
		break ;
		case FR_CELLS_TOT :
		case FR_VOLTS :
      result *= 2;
		break ;
    case FR_WATT:
      result *= 8 ;
    break;
  }
  return result;
}


int16_t get_telemetry_value( int8_t channel )
{
#ifndef TELEMETRY_LOST
	if (telemItemValid( channel ) == 0 )
	{
		return 0 ;
	}
#endif	
	channel = pgm_read_byte( &TelemIndex[channel] ) ;
  if ( channel == TMOK )
	{
		return TmOK ;
	}
  if ( channel == V_RTC )
	{
		return Time.hour * 60 + Time.minute ;
	}
  if ( channel == FR_WATT )
	{
		return (uint32_t)FrskyHubData[FR_VOLTS] * (uint32_t)FrskyHubData[FR_CURRENT] / (uint32_t)100 ;
	}	 
	if ( channel < -10 )	// A Scaler
	{
		return calc_scaler(channel-V_SC1, 0, 0 ) ;
	}
	if ( channel < -3 )	// A GVAR
	{
		return g_model.gvars[channel-V_GVAR1].gvar ;
	}
  switch (channel)
	{
    case TIMER1 :
    case TIMER2 :
    return s_timer[channel+2].s_timerVal ;
    
    case BATTERY :
    return g_vbat100mV ;

#ifdef FRSKY
    case FR_ALT_BARO :
		return FrskyHubData[channel] + AltOffset ;

    case FR_CELL_MIN :
		return FrskyHubData[channel] * 2 ;
		 
		default :
		return FrskyHubData[channel] ;
#endif
  }
}

void displayTimer( uint8_t x, uint8_t y, uint8_t timer, uint8_t att )
{
	struct t_timer *tptr = &s_timer[timer] ;
//	FORCE_INDIRECT(tptr) ;
  att |= (tptr->s_timerState==TMR_BEEPING ? BLINK : 0);
  putsTime( x, y, tptr->s_timerVal, att, att ) ;
}

// Styles
#define TELEM_LABEL				0x01
#define TELEM_UNIT    		0x02
#define TELEM_UNIT_LEFT		0x04
#define TELEM_VALUE_RIGHT	0x08
#define TELEM_NOTIME_UNIT	0x10
#define TELEM_CONSTANT		0x80

uint8_t putsTelemetryChannel(uint8_t x, uint8_t y, int8_t channel, int16_t val, uint8_t att, uint8_t style)
{
	uint8_t unit = ' ' ;
	uint8_t xbase = x ;
	uint8_t fieldW = FW ;
	uint8_t displayed = 0 ;

	if ( style & TELEM_LABEL )
	{
		uint8_t displayed = 0 ;
		int8_t index = TelemIndex[channel] ;

		if ( (index >= V_SC1) && (index < V_SC1 + NUM_SCALERS) )
		{
			uint8_t text[6] ;
			index -= V_SC1 ;
			uint8_t *p = &g_model.Scalers[index].name[0] ;
			text[0] = *p++ ;

			if ( text[0] && (text[0] != ' ') )
			{
				text[1] = *p++ ;
				text[2] = *p++ ;
				text[3] = *p ;
				text[4] = 0 ;
				lcd_putsAtt( x, y, (const char *)text, 0 ) ;
				displayed = 1 ;
			}
		}
		if ( displayed == 0 )
		{
  		putsAttIdxTelemItems( x, y, channel+1, 0 ) ;
		}
		x += 4*FW ;
		if ( att & DBLSIZE )
		{
			x += 4 ;
			y -= FH ;												
			fieldW += FW ;
		}
	}

	if (style & TELEM_VALUE_RIGHT)
	{
		att &= ~LEFT ;
	}
	channel = TelemIndex[channel] ;
  switch (channel)
	{
		case V_SC1 :
		case V_SC2 :
		case V_SC3 :
		case V_SC4 :
		case V_SC5 :
		case V_SC6 :
		case V_SC7 :
		case V_SC8 :
		{
			int16_t cvalue ;
			uint8_t precision ;
			cvalue = calc_scaler( channel-V_SC1, &unit, &precision ) ;
			if ( precision == 1 )
			{
				att |= PREC1 ;
			}
			else if ( precision == 2 )
			{
				att |= PREC2 ;
			}
			// Sort units here
			unit = UnitsText[unit] ;
			if ( (style & TELEM_CONSTANT) == 0)
			{
				val = cvalue ;
			}
		}	
		break ;

    case V_RTC :
			style |= TELEM_NOTIME_UNIT ;
		case TIMER1 :
    case TIMER2 :
			if ( (att & DBLSIZE) == 0 )
			{
				x -= 4 ;
			}
			if ( style & TELEM_LABEL )
			{
				x += FW+4 ;
			}
//			att &= DBLSIZE ;
      putsTime(x-FW-2+5, y, val, att, att) ;
			displayed = 1 ;
			if ( !(style & TELEM_NOTIME_UNIT) )
			{
    		unit = channel + 2 + '1';
			}
			xbase -= FW ;
    break ;
    
		case FR_A1_COPY:
    case FR_A2_COPY:
      channel -= FR_A1_COPY ;
      // no break
      // A1 and A2
			unit = putsTelemValue( (style & TELEM_VALUE_RIGHT) ? xbase+61 : x-fieldW, y, val, channel, att|NO_UNIT/*|blink*/ ) ;
			displayed = 1 ;
    break ;

		case FR_RXV:
  		unit = 'v' ;
			att |= PREC1 ;
			val *= g_model.rxVratio ;
			val /= 255 ;
    break ;

    case FR_TEMP1:
    case FR_TEMP2:
			unit = 'C' ;
  		if ( g_model.FrSkyImperial )
  		{
				val = c_to_f(val) ;
  		  unit = 'F' ;
				x -= fieldW ;
  		}
    break;
    
		case FR_ALT_BARO:
    case FR_GPS_ALT:
      unit = 'm' ;
			if ( g_model.FrSkyImperial )
			{
				if (g_model.FrSkyUsrProto == 0)  // Not WS How High
				{
        	// m to ft *105/32
        	val = m_to_ft( val ) ;
				}
        unit = 'f' ;
			}
			if ( val < 1000 )
			{
				att |= PREC1 ;
			}
			else
			{
				val /= 10 ;
			}
    break;

		case FR_CURRENT :
			att |= PREC1 ;
      unit = 'A' ;
		break ;

		case FR_CELL_MIN:
			att |= PREC2 ;
      unit = 'v' ;
		break ;
		case FR_CELLS_TOT :
		case FR_VOLTS :
			att |= PREC1 ;
      unit = 'v' ;
		break ;
		case BATTERY:
			att |= PREC1 ;
      unit = 'v' ;
		break ;
		case FR_WATT :
      unit = 'w' ;
		break ;

		case FR_RPM :
  		lcd_outdezNAtt( (style & TELEM_VALUE_RIGHT) ? xbase+61 : x, y, (uint16_t)val, att, 5 ) ;
			displayed = 1 ;
		break ;

		case FR_VSPD :
			att |= PREC1 ;
			val /= 10 ;
		break ;

		case FR_FUEL :
      unit = '%' ;
		break ;

		default:
    break;
  }
	if ( !displayed )
	{
  	lcd_outdezAtt( (style & TELEM_VALUE_RIGHT) ? xbase+61 : x, y, val, att ) ;
	}
	if ( style & ( TELEM_UNIT | TELEM_UNIT_LEFT ) )
	{
		if ( style & TELEM_UNIT_LEFT )
		{
			x = xbase + FW + 4 ;			
			att &= ~DBLSIZE ;			 
		}
		else
		{
			x = Lcd_lastPos ;
		}
  	lcd_putcAtt( x, y, unit, att);
	}
	return unit ;
}


// This is for the customisable telemetry display
//void display_custom_telemetry_value( uint8_t x, uint8_t y, int8_t index )
//{
//  lcd_putsAttIdx( x, y, PSTR(STR_TELEM_ITEMS), index+1, 0 ) ;
//	index = TelemIndex[index] ;
//	if ( index < 0 )
//	{ // A timer
//		putsTime( x+4*FW, y, s_timer[index+2].s_timerVal, 0, 0);
//	}
//	else
//	{ // A telemetry value



//	}
//}

//int16_t get_telem_value( uint8_t index )
//{
//	index = TelemIndex[index] ;
//	if ( index < 0 )
//	{ // A timer
//		return s_timer[index+2].s_timerVal ;
//	}
//	return FrskyHubData[index] ;	
//}

#if GVARS

void dispGvar( uint8_t x, uint8_t y, uint8_t gvar, uint8_t attr )
{
	lcd_putsAtt( x, y, PSTR(STR_GV), attr ) ;
	lcd_putcAtt( x+2*FW, y, gvar+'0', attr ) ;
}

int8_t gvarMenuItem(uint8_t x, uint8_t y, int8_t value, int8_t min, int8_t max, uint8_t attr, uint8_t event )
{
  bool invers = attr&(INVERS|BLINK);
  
	if (value >= 126 || value <= -126)
	{
		dispGvar( x-3*FW, y, (uint8_t)value - 125, attr ) ;
    if (invers) value = checkIncDec16( event, (uint8_t)value, 126, 130, EE_MODEL);
  }
  else
	{
    lcd_outdezAtt(x, y, value, attr ) ;
    if (invers) CHECK_INCDEC_H_MODELVAR(event, value, min, max);
  }
  
	if (invers)
	{
		if ( event == EVT_TOGGLE_GVAR )
		{
    	value = ((value >= 126 || value <= -126) ? g_model.gvars[(uint8_t)value-126].gvar : 126);
	    eeDirty(EE_MODEL) ;
		}
	}
	return value;
}

//void displayGVar(uint8_t x, uint8_t y, int8_t value)
//{
//  if (value >= 126 || value <= -126)
//	{
//		lcd_puts_P( x-3*FW, y, PSTR("GV") ) ;
//		lcd_putc( x-FW, y, (uint8_t)value - 125+'0') ;
//  }
//  else
//	{
//    lcd_outdez(x, y, value ) ;
//  }
//}
#endif

/*
#define GET_DR_STATE(x) (!getSwitch(g_model.expoData[x].drSw1,0) ?   \
    DR_HIGH :                                  \
    !getSwitch(g_model.expoData[x].drSw2,0)?   \
    DR_MID : DR_LOW);
*/

uint8_t get_dr_state(uint8_t x)
{
	ExpoData *ped ;

	ped = &g_model.expoData[x] ;
	
 	return (!getSwitch( ped->drSw1,0) ? DR_HIGH :
    !getSwitch( ped->drSw2,0) ? DR_MID : DR_LOW) ;
}

//#define DO_SQUARE(xx,yy,ww)
//    lcd_vline(xx-ww/2,yy-ww/2,ww);
//    lcd_hline(xx-ww/2,yy+ww/2,ww);
//    lcd_vline(xx+ww/2,yy-ww/2,ww);
//    lcd_hline(xx-ww/2,yy-ww/2,ww);

#define DO_SQUARE(xx,yy,ww)         \
{uint8_t x,y,w ; x = xx; y = yy; w = ww ; \
    lcd_vline(x-w/2,y-w/2,w);  \
    lcd_hline(x-w/2,y+w/2,w);  \
    lcd_vline(x+w/2,y-w/2,w);  \
    lcd_hline(x-w/2,y-w/2,w);}

#define DO_CROSS(xx,yy,ww)          \
    lcd_vline(xx,yy-ww/2,ww);  \
    lcd_hline(xx-ww/2,yy,ww);  \

#define V_BAR(xx,yy,ll)       \
    lcd_vline(xx-1,yy-ll,ll); \
    lcd_vline(xx  ,yy-ll,ll); \
    lcd_vline(xx+1,yy-ll,ll);

#define NO_HI_LEN 25

#define WCHART 32
#define X0     (128-WCHART-2 - 6 )
#define Y0     32
#define WCHARTl 32l
#define X0l     (128l-WCHARTl-2)
#define Y0l     32l
#define RESX    (1<<10) // 1024
#define RESXu   1024u
#define RESXul  1024ul
#define RESXl   1024l
#define RESKul  100ul
#define RESX_PLUS_TRIM (RESX+128)


void menuProcSetup(uint8_t event) ;
void menuProcSetup1(uint8_t event) ;
void menuProcSetup2(uint8_t event) ;
void menuProcDiagAna(uint8_t event) ;
void menuProcDiagKeys(uint8_t event) ;
void menuProcDiagCalib(uint8_t event) ;
void menuProcSDstat(uint8_t event) ;
void menuProcBoot(uint8_t event) ;
void menuProcTrainer(uint8_t event) ;
void menuProcDiagVers(uint8_t event) ;
void menuProcModel(uint8_t event) ;
void menuModelPhases(uint8_t event) ;
void menuProcHeli(uint8_t event) ;
void menuProcModelSelect(uint8_t event) ;
void menuProcMix(uint8_t event) ;
void menuProcExpoAll(uint8_t event);
void menuProcLimits(uint8_t event);
void menuProcCurve(uint8_t event);
void menuProcSwitches(uint8_t event);
void menuProcSafetySwitches(uint8_t event);
void menuProcTemplates(uint8_t event);
void menuProcTelemetry(uint8_t event) ;
void menuProcTelemetry2(uint8_t event) ;
void menuProcDate(uint8_t event) ;
void menuProcRestore(uint8_t event) ;
void menuProcIndex(uint8_t event) ;
void menuProcModelIndex(uint8_t event) ;

void menuProcBattery(uint8_t event) ;
void menuProcStatistic(uint8_t event) ;
void menuProcStatistic2(uint8_t event) ;
void menuProcDsmDdiag(uint8_t event) ;
void menuProcTrainDdiag(uint8_t event) ;

void menuProcVoiceAlarm(uint8_t event) ;

uint8_t getMixerCount( void ) ;
bool reachMixerCountLimit( void ) ;


enum MainViews
{
  e_outputValues,
  e_outputBars,
  e_inputs1,
//  e_inputs2,
//  e_inputs3,
  e_timer2,
#ifdef FRSKY
  e_telemetry,
#endif
  MAX_VIEWS
};

int16_t calibratedStick[NUMBER_ANALOG+NUM_EXTRA_POTS];

int16_t ex_chans[NUM_SKYCHNOUT];          // Outputs + intermidiates
uint8_t s_pgOfs;
uint8_t s_editMode;
uint8_t s_editing;
uint8_t s_noHi;
uint8_t scroll_disabled;
int8_t scrollLR;
int8_t scrollUD;
uint8_t InverseBlink ;

int16_t g_chans512[NUM_SKYCHNOUT];

//extern int16_t p1valdiff;

#include "sticks.lbm"

enum EnumTabModel {
//    e_ModelSelect,
    e_Model,
#ifndef NO_HELI
    e_Heli,
#endif
		e_Phases,
    e_ExpoAll,
    e_Mix,
    e_Limits,
    e_Curve,
    e_Switches,
    e_SafetySwitches//,
#ifdef FRSKY
    ,e_Telemetry
    ,e_Telemetry2
#endif
//#ifndef NO_TEMPLATES
//    ,e_Templates
//#endif
#if GVARS
		,e_Globals
#endif
//		,e_mIndex
};

MenuFuncP menuTabModel[] = {
//    menuProcModelSelect,
    menuProcModel,
    #ifndef NO_HELI
    menuProcHeli,
    #endif
		menuModelPhases,
    menuProcExpoAll,
    menuProcMix,
    menuProcLimits,
    menuProcCurve,
    menuProcSwitches,
    menuProcSafetySwitches//,
    #ifdef FRSKY
    ,menuProcTelemetry
    ,menuProcTelemetry2
    #endif
//    #ifndef NO_TEMPLATES
//    ,menuProcTemplates
//    #endif
#if GVARS
    ,menuProcGlobals
#endif
//		,menuProcModelIndex
};

enum EnumTabStat
{
	e_battery,
	e_stat1,
	e_stat2,
	e_dsm,
	e_traindiag,
  e_Setup2,
  e_Setup3,
  e_Boot
} ;

MenuFuncP menuTabStat[] =
{
	menuProcBattery,
	menuProcStatistic,
	menuProcStatistic2,
	menuProcDsmDdiag,
	menuProcTrainDdiag,
  menuProcSetup2,
	menuProcSDstat,
	menuProcBoot
} ;

//enum EnumTabDiag
//{
//  e_Index//,
//  e_Setup2,
//  e_Setup//,
//  e_Setup1//,
//  e_Trainer,
//  e_Vers,
//	e_Date,
//  e_Keys,
//  e_Ana,
//  e_Calib
//  ,e_Setup3
//  ,e_Boot
//};

//MenuFuncP menuTabDiag[] =
//{
//	menuProcIndex//,
//  menuProcSetup2,
//  menuProcSetup//,
//  menuProcSetup1//,
//  menuProcTrainer,
//  menuProcDiagVers,
//	menuProcDate,
//  menuProcDiagKeys,
//  menuProcDiagAna,
//  menuProcDiagCalib//,
//	menuProcSDstat//,
//	menuProcBoot
//};

// Rotary Encoder states
#define RE_IDLE				0
#define RE_MENU_LR		1
#define RE_MENU_UD		2
#define RE_ITEMLR			3
#define RE_ITEM_EDIT	4

uint8_t Re_state ;

void displayNext()
{
	lcd_puts_P( 15*FW, 7*FH, XPSTR("[More]") ) ;
}

const char *get_curve_string()
{
    return PSTR(CURV_STR)	;
}	

void menu_lcd_onoff( uint8_t x,uint8_t y, uint8_t value, uint8_t mode )
{
    lcd_putsnAtt( x, y, PSTR(STR_OFF_ON)+3*value,3,mode ? InverseBlink:0) ;
}

void menu_lcd_HYPHINV( uint8_t x,uint8_t y, uint8_t value, uint8_t mode )
{
    lcd_putsAttIdx( x, y, PSTR(STR_HYPH_INV),value,mode ? InverseBlink:0) ;
}

void MState2::check_simple(uint8_t event, uint8_t curr, MenuFuncP *menuTab, uint8_t menuTabSize, uint8_t maxrow)
{
    check(event, curr, menuTab, menuTabSize, 0, 0, maxrow);
}

void MState2::check_submenu_simple(uint8_t event, uint8_t maxrow)
{
    check_simple(event, 0, 0, 0, maxrow);
}

#define EDIT_DR_SWITCH_EDIT		0x01
#define EDIT_DR_SWITCH_MOMENT	0x02

int8_t edit_dr_switch( uint8_t x, uint8_t y, int8_t drswitch, uint8_t attr, uint8_t flags, uint8_t event )
{
	if ( flags & EDIT_DR_SWITCH_MOMENT )
	{
		putsMomentDrSwitches( x, y, drswitch, attr ) ;
	}
	else
	{
		putsDrSwitches( x,  y, drswitch, attr ) ;
	}
	if(flags & EDIT_DR_SWITCH_EDIT)
	{
		if ( flags & EDIT_DR_SWITCH_MOMENT )
		{
#if PCBSKY
			int16_t value = drswitch ;
			if ( value < -HSW_MAX )
			{
				value += 256 ;
			}
			if ( value > HSW_MAX )
			{
				value = switchUnMap( value - HSW_MAX ) + MaxSwitchIndex - 1 ;
			}
			else
			{
				value = switchUnMap( drswitch ) ;
			}
      CHECK_INCDEC_H_MODELVAR( event, value ,(1-MaxSwitchIndex),(-2+2*MaxSwitchIndex));
			if ( value >= MaxSwitchIndex )
			{
				drswitch = switchMap( value - MaxSwitchIndex + 1 ) + HSW_MAX ;
			}
			else
			{
				drswitch = switchMap( value ) ;
			}
#endif
#if PCBX9D
			int16_t value = drswitch ;
			if ( value < -HSW_MAX )
			{
				value += 256 ;
			}
			if ( value > HSW_MAX )
			{
				value = switchUnMap( value - HSW_MAX ) + MaxSwitchIndex - 1 ;
			}
			else
			{
				value = switchUnMap( drswitch ) ;
			}
      CHECK_INCDEC_H_MODELVAR( event, value ,(1-MaxSwitchIndex),(-2+2*MaxSwitchIndex));
			if ( value >= MaxSwitchIndex )
			{
				drswitch = switchMap( value - MaxSwitchIndex + 1 ) + HSW_MAX ;
			}
			else
			{
				drswitch = switchMap( value ) ;
			}
//      CHECK_INCDEC_MODELSWITCH( event, drswitch ,(1-MAX_SKYDRSWITCH),(-2+2*MAX_SKYDRSWITCH));
#endif
		}
		else
		{
			CHECK_INCDEC_MODELSWITCH( event, drswitch, -MaxSwitchIndex, MaxSwitchIndex) ;
		}
	}
	return drswitch ;
}

void editName( uint8_t xpos, uint8_t x, uint8_t y, uint8_t *name, uint8_t len, uint8_t type, uint8_t event )
{
	lcd_puts_Pleft( y, PSTR(STR_NAME)) ;
	lcd_putsnAtt( xpos, y, (const char *)name, len, 0 ) ;
	if( type )
	{
		lcd_char_inverse( xpos+x*FW, y, 1*FW, s_editMode ) ;
		lcd_rect( xpos-2, y-1, len*FW+4, 9 ) ;
	  if(s_editMode)
		{
     	char v = name[x] ;
			if ( v )
			{
	  	  v = char2idx(v) ;
			}
			v = checkIncDec( event, v, 0, NUMCHARS-1, type ) ;
//	  	CHECK_INCDEC_H_MODELVAR_0( v ,NUMCHARS-1);
  	  v = idx2char(v);
			if ( name[x] != v )
			{
				name[x] = v ;
    		eeDirty( type ) ;				// Do here or the last change is not stored in name[]
			}
		}
	}
	asm("") ;
}


void DisplayScreenIndex(uint8_t index, uint8_t count, uint8_t attr)
{
	uint8_t x ;
	if ( RotaryState == ROTARY_MENU_LR )
	{
		attr = BLINK ;
	}
	lcd_outdezAtt(127,0,count,attr);
	x = 1+128-FW*(count>9 ? 3 : 2) ;
  lcd_putcAtt(x,0,'/',attr);
  lcd_outdezAtt(x-1,0,index+1,attr);
//		lcd_putc( x-12, 0, RotaryState + '0' ) ;
}

uint8_t M_longMenuTimer ;
uint8_t M_lastVerticalPosition ;

//#define MAXCOL(row) (horTab ? pgm_read_byte(horTab+min(row, horTabMax)) : (const uint8_t)0)

uint8_t MAXCOL( uint8_t row, const uint8_t *horTab, uint8_t horTabMax)
{
	return (horTab ? pgm_read_byte(horTab+min(row, horTabMax)) : (const uint8_t)0) ;
}

#define INC(val,max) if(val<max) {val++;} else {val=0;}
#define DEC(val,max) if(val>0  ) {val--;} else {val=max;}
uint8_t MState2::check(uint8_t event, uint8_t curr, MenuFuncP *menuTab, uint8_t menuTabSize, prog_uint8_t *horTab, uint8_t horTabMax, uint8_t maxrow)
{
    //    scrollLR = 0;
    //    scrollUD = 0;
	int16_t c4, c5 ;
	struct t_p1 *ptrp1 ;

    //check pot 2 - if changed -> scroll menu
    //check pot 3 if changed -> cursor down/up
    //we do this in these brackets to prevent it from happening in the main screen
	c4 = calibratedStick[4] ;		// Read only once
	c5 = calibratedStick[5] ;		// Read only once
    
	ptrp1 = &P1values ;
//		FORCE_INDIRECT(ptrp1) ;
		
  scrollLR = ( ptrp1->p2valprev-c4)/SCROLL_TH;
  scrollUD = ( ptrp1->p3valprev-c5)/SCROLL_TH;

  if(scrollLR) ptrp1->p2valprev = c4;
  if(scrollUD) ptrp1->p3valprev = c5;

  if(scroll_disabled || g_eeGeneral.disablePotScroll)
  {
    scrollLR = 0;
    scrollUD = 0;
    scroll_disabled = 0;
  }

	M_lastVerticalPosition = m_posVert ;
  
	if(scrollLR || scrollUD || ptrp1->p1valdiff) g_LightOffCounter = g_eeGeneral.lightAutoOff*500; // on keypress turn the light on 5*100

	if (menuTab)
	{
    uint8_t attr = m_posVert==0 ? INVERS : 0;


    if(m_posVert==0)
    {
			if ( RotaryState == ROTARY_MENU_LR )
			{
				if ( Rotary_diff > 0 )
				{
   				scrollLR = -1;
				}
				else if ( Rotary_diff < 0 )
				{
   				scrollLR = 1;
				}
				Rotary_diff = 0 ;
        if(event==EVT_KEY_BREAK(BTN_RE))
				{
					RotaryState = ROTARY_MENU_UD ;
		      event = 0 ;
				}
			}
			else if ( RotaryState == ROTARY_MENU_UD )
			{
        if(event==EVT_KEY_BREAK(BTN_RE))
				{
					RotaryState = ROTARY_MENU_LR ;
		      event = 0 ;
				}
			}
      if( scrollLR && !s_editMode)
      {
        int8_t cc = curr - scrollLR ;
        if(cc<1) cc = menuTabSize-1 ;
        if(cc>(menuTabSize-1)) cc = 0 ;

        if(cc!=curr)
        {
            //                    if(((MenuFuncP)pgm_read_adr(&menuTab[cc])) == menuProcDiagCalib)
            //                        chainMenu(menuProcDiagAna);
            //                    else
          chainMenu((MenuFuncP)pgm_read_adr(&menuTab[cc]));
					return event ;
        }

        scrollLR = 0;
      }

      if(event==EVT_KEY_FIRST(KEY_LEFT))
      {
        uint8_t index ;
        if(curr>0)
          index = curr ;
          //                    chainMenu((MenuFuncP)pgm_read_adr(&menuTab[curr-1]));
        else
          index = menuTabSize ;

       	chainMenu((MenuFuncP)pgm_read_adr(&menuTab[index-1]));
				return event ;
      }

      if(event==EVT_KEY_FIRST(KEY_RIGHT))
      {
        uint8_t index ;
        if(curr < (menuTabSize-1))
          index = curr +1 ;
         //                    chainMenu((MenuFuncP)pgm_read_adr(&menuTab[curr+1]));
        else
          index = 0 ;
        chainMenu((MenuFuncP)pgm_read_adr(&menuTab[index]));
				return event ;
      }
    }
		else
		{
			if ( s_editMode == 0 ) RotaryState = ROTARY_MENU_UD ;
		}

    DisplayScreenIndex(curr, menuTabSize, attr);
  }
	else if ( RotaryState == ROTARY_MENU_LR )
	{
		RotaryState = ROTARY_MENU_UD ;
	}
  uint8_t maxcol = MAXCOL(m_posVert, horTab, horTabMax);
		
 if ( maxrow != 0xFF )
 {
	if ( RotaryState == ROTARY_MENU_UD )
	{
		static uint8_t lateUp = 0 ;
		if ( lateUp )
		{
			lateUp = 0 ;
			m_posHorz = MAXCOL(m_posVert, horTab, horTabMax) ;
		}
		if ( Rotary_diff > 0 )
		{
    	INC(m_posHorz,maxcol) ;
			if ( m_posHorz == 0 )
			{
				INC(m_posVert,maxrow);
			}
		}
		else if ( Rotary_diff < 0 )
		{
			if ( m_posHorz == 0 )
			{
      	DEC(m_posVert,maxrow);
				lateUp = 1 ;
				m_posHorz = 0 ;
			}
			else
			{
      	DEC(m_posHorz,maxcol) ;
			}
		}
		Rotary_diff = 0 ;
    if(event==EVT_KEY_BREAK(BTN_RE))
		{
			RotaryState = ROTARY_VALUE ;
		}
	}
	else if ( RotaryState == ROTARY_VALUE )
	{
		if ( (event==EVT_KEY_BREAK(BTN_RE)) || ( s_editMode == 0 ) )
		{
			RotaryState = ROTARY_MENU_UD ;
		}
	}

	{
		uint8_t timer = M_longMenuTimer ;
		if ( menuPressed() )
		{
			if ( timer < 255 )
			{
				timer += 1 ;
			}
		}
		else
		{
			timer = 0 ;
		}
		if ( timer > 60 )
		{
			s_editMode = 1 ;
			RotaryState = ROTARY_VALUE ;
		}
		M_longMenuTimer = timer ;
	}

 } 

    //        scrollLR = 0;

  maxcol = MAXCOL(m_posVert, horTab, horTabMax) ;
  m_posHorz = min(m_posHorz, maxcol ) ;

  if(!s_editMode)
  {
    if(scrollUD)
    {
      int8_t cc = m_posVert - scrollUD;
      if(cc<1) cc = 0;
      if(cc>=maxrow) cc = maxrow;
      m_posVert = cc;

      m_posHorz = min(m_posHorz, MAXCOL(m_posVert, horTab, horTabMax));
      BLINK_SYNC;

      scrollUD = 0;
    }

    if(m_posVert>0 && scrollLR)
    {
      int8_t cc = m_posHorz - scrollLR;
      if(cc<1) cc = 0;
      if(cc>=MAXCOL(m_posVert, horTab, horTabMax)) cc = MAXCOL(m_posVert, horTab, horTabMax);
      m_posHorz = cc;

      BLINK_SYNC;
      //            scrollLR = 0;
    }
  }

  switch(event)
  {
    case EVT_ENTRY:
        //if(m_posVert>maxrow)
        init() ;
				M_lastVerticalPosition = m_posVert ;
        s_editMode = false;
        //init();BLINK_SYNC;
    break;
    case EVT_KEY_BREAK(BTN_RE):
    case EVT_KEY_FIRST(KEY_MENU):
        if ( (m_posVert > 0) || (!menuTab) )
				{
	 				if ( maxrow != 0xFF )
					{
						s_editMode = !s_editMode;
						if ( s_editMode )
						{
							RotaryState = ROTARY_VALUE ;
						}
					}
				}	
    break;
    case EVT_KEY_LONG(KEY_EXIT):
        s_editMode = false;
        //popMenu(true); //return to uppermost, beeps itself
        popMenu(false);
    break;
        //fallthrough
    case EVT_KEY_LONG(BTN_RE):
        killEvents(event);
    case EVT_KEY_BREAK(KEY_EXIT):
        if(s_editMode) {
            s_editMode = false;
            break;
        }
        if(m_posVert==0 || !menuTab) {
						RotaryState = ROTARY_MENU_LR ;
            popMenu();  //beeps itself
        } else {
            audioDefevent(AU_MENUS);
            init();BLINK_SYNC;
        }
    break;

    case EVT_KEY_REPT(KEY_RIGHT):  //inc
        if(m_posHorz==maxcol) break;
    case EVT_KEY_FIRST(KEY_RIGHT)://inc
        if(!horTab || s_editMode)break;
        INC(m_posHorz,maxcol);
        BLINK_SYNC;
				if ( maxcol )
				{
					event = 0 ;
				}
    break;

    case EVT_KEY_REPT(KEY_LEFT):  //dec
        if(m_posHorz==0) break;
    case EVT_KEY_FIRST(KEY_LEFT)://dec
        if(!horTab || s_editMode)break;
        DEC(m_posHorz,maxcol);
        BLINK_SYNC;
				if ( maxcol )
				{
					event = 0 ;
				}
    break;

    case EVT_KEY_REPT(KEY_DOWN):  //inc
        if(m_posVert==maxrow) break;
    case EVT_KEY_FIRST(KEY_DOWN): //inc
        if(s_editMode)break;
        INC(m_posVert,maxrow);
        m_posHorz = min(m_posHorz, MAXCOL(m_posVert, horTab, horTabMax));
        BLINK_SYNC;
    break;

    case EVT_KEY_REPT(KEY_UP):  //dec
        if(m_posVert==0) break;
    case EVT_KEY_FIRST(KEY_UP): //dec
        if(s_editMode)break;
        DEC(m_posVert,maxrow);
        m_posHorz = min(m_posHorz, MAXCOL(m_posVert, horTab, horTabMax));
        BLINK_SYNC;
    break;
  }
	s_editing = s_editMode || P1values.p1valdiff ;
//	InverseBlink = (!maxcol || s_editMode) ? BLINK : INVERS ;
	InverseBlink = (s_editMode) ? BLINK : INVERS ;
	
	return event ;
}

#define BOX_WIDTH     23
#define BAR_HEIGHT    (BOX_WIDTH-1l)
#define MARKER_WIDTH  5
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define BOX_LIMIT     (BOX_WIDTH-MARKER_WIDTH)
#define LBOX_CENTERX  (  SCREEN_WIDTH/4 + 10)
#define BOX_CENTERY  (SCREEN_HEIGHT-9-BOX_WIDTH/2)
#define RBOX_CENTERX  (3*SCREEN_WIDTH/4 - 10)
//#define RBOX_CENTERY  (SCREEN_HEIGHT-9-BOX_WIDTH/2)


void telltale( uint8_t centrex, int8_t xval, int8_t yval )
{
  DO_SQUARE( centrex, BOX_CENTERY, BOX_WIDTH ) ;
  DO_CROSS( centrex, BOX_CENTERY,3 ) ;
	DO_SQUARE( centrex +( xval/((2*RESX/16)/BOX_LIMIT)), BOX_CENTERY-( yval/((2*RESX/16)/BOX_LIMIT)), MARKER_WIDTH ) ;
}


void doMainScreenGrphics()
{
	int8_t *cs = phyStick ;
//	FORCE_INDIRECT(cs) ;
	
	telltale( LBOX_CENTERX, cs[0], cs[1] ) ;
	telltale( RBOX_CENTERX, cs[3], cs[2] ) ;
    
    //    V_BAR(SCREEN_WIDTH/2-5,SCREEN_HEIGHT-10,((calibratedStick[4]+RESX)*BAR_HEIGHT/(RESX*2))+1l) //P1
    //    V_BAR(SCREEN_WIDTH/2  ,SCREEN_HEIGHT-10,((calibratedStick[5]+RESX)*BAR_HEIGHT/(RESX*2))+1l) //P2
    //    V_BAR(SCREEN_WIDTH/2+5,SCREEN_HEIGHT-10,((calibratedStick[6]+RESX)*BAR_HEIGHT/(RESX*2))+1l) //P3

    // Optimization by Mike Blandford
    {
        uint8_t x, y, len ;			// declare temporary variables
#ifdef PCBSKY
        for( x = -5, y = 4 ; y < 7 ; x += 5, y += 1 )
        {
            len = ((calibratedStick[y]+RESX)/((RESX*2)/BAR_HEIGHT))+1 ;  // calculate once per loop
            V_BAR(SCREEN_WIDTH/2+x,SCREEN_HEIGHT-8, len )
        }
#endif
#ifdef PCBX9D
        for( x = -8, y = 4 ; y < 8 ; x += 5, y += 1 )
        {
        	uint8_t z = y - 1 ;
					if ( z == 6 )
					{
						z = 7 ;
					}
					else if ( z == 3 )
					{
						z = 6 ;
					}
          len = ((calibratedStick[z]+RESX)/((RESX*2)/BAR_HEIGHT))+1 ;  // calculate once per loop
          V_BAR(SCREEN_WIDTH/2+x,SCREEN_HEIGHT-8, len )
        }
#endif
    }
}

extern int16_t calcExpo( uint8_t channel, int16_t value ) ;

static uint8_t s_curveChan ;
static uint8_t s_expoChan ;
static uint8_t SingleExpoChan ;

#define XD (X0-2)

#define GRAPH_FUNCTION_CURVE		0
#define GRAPH_FUNCTION_EXPO			1

void drawFunction( uint8_t xpos, uint8_t function )
{
  int8_t yv ;
  int8_t prev_yv = 127 ;
	for ( int8_t xv = -WCHART ; xv <= WCHART ; xv++ )
	{
		if ( function == GRAPH_FUNCTION_CURVE )
		{
    	yv = intpol(xv * RESX / WCHART, s_curveChan) * WCHART / RESX ;
		}
		else
		{
    	yv = calcExpo( s_expoChan, xv * RESX / WCHART) * WCHART / RESX ;
		}
    if (prev_yv == 127)
		{
			prev_yv = yv ;
		}
		uint8_t len = abs(yv-prev_yv) ;
    if (len <= 1)
		{
    	lcd_plot(xpos + xv, Y0 - yv) ;
		}
		else
		{
      uint8_t tmp = (prev_yv < yv ? 0 : len-1 ) ;
      lcd_vline(xpos+xv, Y0 - yv - tmp, len ) ;
		}	
		if ( yv )
		{
     	lcd_plot(xpos + xv, Y0 ) ;
		}
		prev_yv = yv ;
	}
	
}


void drawCurve( uint8_t offset )
{
  uint8_t cv9 = s_curveChan >= MAX_CURVE5 ;
	int8_t *crv = cv9 ? g_model.curves9[s_curveChan-MAX_CURVE5] : g_model.curves5[s_curveChan];

	lcd_vline(XD, Y0 - WCHART, WCHART * 2);
  
	plotType = PLOT_BLACK ;
	for(uint8_t i=0; i<(cv9 ? 9 : 5); i++)
  {
    uint8_t xx = XD-1-WCHART+i*WCHART/(cv9 ? 4 : 2);
    uint8_t yy = Y0-crv[i]*WCHART/100;

    if(offset==i)
    {
			lcd_rect( xx-1, yy-2, 5, 5 ) ;
    }
    else
    {
			lcd_rect( xx, yy-1, 3, 3 ) ;
    }
  }

	drawFunction( XD, GRAPH_FUNCTION_CURVE ) ;
	
	plotType = PLOT_XOR ;
}

void menuProcCurveOne(uint8_t event)
{
    bool    cv9 = s_curveChan >= MAX_CURVE5 ;
		static int8_t dfltCrv;

    SUBMENU(PSTR(STR_CURVE), 1+(cv9 ? 9 : 5), { 0/*repeated...*/});
		
		if ( event == EVT_ENTRY )
		{
			dfltCrv = 0 ;
		}
		lcd_outdezAtt(7*FW, 0, s_curveChan+1, INVERS);

	int8_t *crv = cv9 ? g_model.curves9[s_curveChan-MAX_CURVE5] : g_model.curves5[s_curveChan];

	uint8_t  sub    = mstate2.m_posVert ;
	uint8_t  preset = cv9 ? 9 : 5 ;

	for (uint8_t i = 0; i < 5; i++)
	{
    uint8_t y = i * FH + 16;
    uint8_t attr = sub == i ? INVERS : 0;
    lcd_outdezAtt(4 * FW, y, crv[i], attr);
		if( cv9 )
		{
			if ( i < 4 )
			{
		    attr = sub == i + 5 ? INVERS : 0;
    		lcd_outdezAtt(8 * FW, y, crv[i + 5], attr);
			}
		}
	}
	lcd_putsAtt( 2*FW, 7*FH,PSTR(STR_PRESET), (sub == preset) ? InverseBlink : 0);


if( sub==preset) 
{
	if ( s_editMode )
	{
		
    dfltCrv = checkIncDec( event, dfltCrv, -4, 4, 0);
    if (checkIncDec_Ret)
		{
			uint8_t offset = cv9 ? 4 : 2 ;
		    
			for (int8_t i = -offset; i <= offset; i++) crv[i+offset] = i*dfltCrv* 25 / offset ;
      STORE_MODELVARS;        
    }
	}
} 
else  /*if(sub>0)*/
{
 CHECK_INCDEC_H_MODELVAR( event, crv[sub], -100,100);
}

// Draw the curve
	drawCurve( sub ) ;
}



void menuProcCurve(uint8_t event)
{
	TITLE( PSTR(STR_CURVES) ) ;
	static MState2 mstate2 ;

	if (SubMenuFromIndex)
	{
		mstate2.check_simple(event,0,NULL,0,1+MAX_CURVE5+MAX_CURVE9-1) ;
	}
	else
	{
		mstate2.check_simple(event,e_Curve,menuTabModel,DIM(menuTabModel),1+MAX_CURVE5+MAX_CURVE9-1) ;
//	SIMPLE_MENU(PSTR(STR_CURVES), menuTabModel, e_Curve, 1+MAX_CURVE5+MAX_CURVE9);
	}

  int8_t  sub    = mstate2.m_posVert - 1;

  evalOffset(sub, 6);

  switch (event)
	{
    case EVT_KEY_FIRST(KEY_RIGHT):
    case EVT_KEY_FIRST(KEY_MENU):
    case EVT_KEY_BREAK(BTN_RE):
      if (sub >= 0)
			{
        s_curveChan = sub;
        pushMenu(menuProcCurveOne);
      }
    break;
  }

    uint8_t y    = 1*FH;
//    uint8_t yd   = 1;
//    uint8_t m    = 0;
    for (uint8_t i = 0; i < 7; i++)
		{
      uint8_t k = i + s_pgOfs;
      uint8_t attr = sub == k ? INVERS : 0;
//      bool    cv9 = k >= MAX_CURVE5;

//        if(cv9 && (y>6*FH)) break;
      if(y>7*FH) break;
//        if(!m) m = attr;
      lcd_putsAtt(   FW*0, y,PSTR(STR_CV),attr);
      lcd_outdezAtt( (k<9) ? FW*3-1 : FW*4-2, y,k+1 ,attr);

//        int8_t *crv = cv9 ? g_model.curves9[k-MAX_CURVE5] : g_model.curves5[k];
//        for (uint8_t j = 0; j < (5); j++)
//				{
//            lcd_outdezAtt( j*(3*FW+3) + 7*FW, y, crv[j], 0);
//        }
        y += FH ;
//        if(cv9){
//            for (uint8_t j = 0; j < 4; j++) {
//                lcd_outdezAtt( j*(3*FW+3) + 7*FW, y, crv[j+5], 0);
//            }
//            y += FH ;
//        }
    }

//    if(!m) s_pgOfs++;

		if ( sub >= 0 )
		{
  		s_curveChan = sub ;
			drawCurve( 100 ) ;
		}

}

void setStickCenter() // copy state of 3 primary to subtrim
{
	uint8_t thisPhase ;
  int16_t zero_chans512_before[NUM_SKYCHNOUT];
  int16_t zero_chans512_after[NUM_SKYCHNOUT];

	thisPhase = getFlightPhase() ;
	CurrentPhase = thisPhase ;

    perOut(zero_chans512_before,NO_TRAINER | NO_INPUT | FADE_FIRST | FADE_LAST); // do output loop - zero input channels
    perOut(zero_chans512_after,NO_TRAINER | FADE_FIRST | FADE_LAST); // do output loop - zero input channels

    for(uint8_t i=0; i<NUM_SKYCHNOUT; i++)
    {
        int16_t v = g_model.limitData[i].offset;
        v += g_model.limitData[i].revert ?
                    (zero_chans512_before[i] - zero_chans512_after[i]) :
                    (zero_chans512_after[i] - zero_chans512_before[i]);
        g_model.limitData[i].offset = max(min(v,1000),-1000); // make sure the offset doesn't go haywire
    }

  for(uint8_t i=0; i<4; i++)
	{
		if(!IS_THROTTLE(i))
		{
			int16_t original_trim = getTrimValue(thisPhase, i);
      for (uint8_t phase=0; phase<MAX_MODES; phase +=  1)
			{
        int16_t trim = getRawTrimValue(phase, i);
        if (trim <= TRIM_EXTENDED_MAX)
				{
          setTrimValue(phase, i, trim - original_trim);
				}
			}
		}
	}
  STORE_MODELVARS_TRIM;
  audioDefevent(AU_WARNING2);
}

void menuProcLimits(uint8_t event)
{
	TITLE(PSTR(STR_LIMITS));
	static MState2 mstate2;
	static const uint8_t mstate_tab[] = {0, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 0} ;
	if (SubMenuFromIndex)
	{
		event = mstate2.check(event,0,NULL,0,mstate_tab,DIM(mstate_tab)-1,NUM_SKYCHNOUT+2-1) ;
	}
	else
	{
		event = mstate2.check(event,e_Limits,menuTabModel,DIM(menuTabModel),mstate_tab,DIM(mstate_tab)-1,NUM_SKYCHNOUT+2-1) ;
//    MENU( PSTR(STR_LIMITS), menuTabModel, e_Limits, NUM_SKYCHNOUT+2, {0, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 0});
	}

static bool swVal[NUM_SKYCHNOUT];

uint8_t y = 0;
uint8_t k = 0;
uint8_t  sub   = mstate2.m_posVert - 1;
uint8_t subSub = mstate2.m_posHorz;
    uint8_t t_pgOfs ;

	t_pgOfs = evalOffset(sub, 6);

	if ( sub < NUM_SKYCHNOUT )
	{
    lcd_outdez( 13*FW, 0, g_chans512[sub]/2 + 1500 ) ;
	}

	switch(event)
	{
    case EVT_KEY_LONG(KEY_MENU):
        if(sub>=0 && sub<NUM_SKYCHNOUT) {
            int16_t v = g_chans512[sub - t_pgOfs];
            LimitData *ld = &g_model.limitData[sub];
            switch (subSub) {
            case 0:
                ld->offset = (ld->revert) ? -v : v;
                STORE_MODELVARS;
//                eeWaitComplete() ;
                break;
            }
        }
				if(k==NUM_SKYCHNOUT)
				{
  			  //last line available - add the "copy trim menu" line
  			  s_noHi = NO_HI_LEN;
  			  killEvents(event);
  				s_editMode = 0 ;
  			  setStickCenter(); //if highlighted and menu pressed - copy trims
				}
        break;
	}
//  lcd_puts_P( 4*FW, 1*FH,PSTR("subT min  max inv"));
    int8_t limit = (g_model.extendedLimits ? 125 : 100);
for(uint8_t i=0; i<7; i++){
    y=(i+1)*FH;
    k=i+t_pgOfs;
    if(k==NUM_SKYCHNOUT) break;
    LimitData *ld = &g_model.limitData[k];
    int16_t v = (ld->revert) ? -ld->offset : ld->offset;
    if((g_chans512[k] - v) >  50) swVal[k] = (true==ld->revert);// Switch to raw inputs?  - remove trim!
    if((g_chans512[k] - v) < -50) swVal[k] = (false==ld->revert);
    putsChn(0,y,k+1,0);
    lcd_putcAtt(12*FW+FW/2, y, (swVal[k] ? 127 : 126),0); //'<' : '>'
    for(uint8_t j=0; j<4;j++)
		{
        uint8_t attr = ((sub==k && subSub==j) ? InverseBlink : 0);
				uint8_t active = (attr && (s_editMode || P1values.p1valdiff)) ;
				if ( active )
				{
					StickScrollAllowed = 3 ;		// Block L/R while editing
				}
				int16_t value ;
				int16_t t = 0 ;
				if ( g_model.sub_trim_limit )
				{
					if ( ( t = ld->offset ) )
					{
						if ( t > g_model.sub_trim_limit )
						{
							t = g_model.sub_trim_limit ;
						}
						else if ( t < -g_model.sub_trim_limit )
						{
							t = -g_model.sub_trim_limit ;
						}
					}
				}
				value = t / 10 ;
        switch(j)
        {
        case 0:
            lcd_outdezAtt(  7*FW+3, y,  ld->offset, attr|PREC1);
            if(active) {
                ld->offset = checkIncDec16(event, ld->offset, -1000, 1000, EE_MODEL);
            }
            break;
        case 1:
					value += (int8_t)(ld->min-100) ;
					if ( value < -125 )
					{
						value = -125 ;						
					}
          lcd_outdezAtt(  12*FW, y, value,   attr);
            if(active) {
                ld->min -=  100;
		            CHECK_INCDEC_H_MODELVAR( event, ld->min, -limit,25);
                ld->min +=  100;
            }
            break;
        case 2:
					value += (int8_t)(ld->max+100) ;
					if ( value > 125 )
					{
						value = 125 ;						
					}
          lcd_outdezAtt( 17*FW, y, value,    attr);
					if ( t )
					{
						lcd_rect( 9*FW-4, y-1, 56, 9 ) ;
					}
            if(active) {
                ld->max +=  100;
                CHECK_INCDEC_H_MODELVAR( event, ld->max, -25,limit);
                ld->max -=  100;
            }
            break;
        case 3:
						menu_lcd_HYPHINV( 18*FW, y, ld->revert, attr ) ;
//            lcd_putsnAtt(   18*FW, y, PSTR("---INV")+ld->revert*3,3,attr);
            if(active) {
                CHECK_INCDEC_H_MODELVAR_0(event, ld->revert, 1);
            }
            break;
        }
    }
}
	if(k==NUM_SKYCHNOUT)
	{
    //last line available - add the "copy trim menu" line
    uint8_t attr = (sub==NUM_SKYCHNOUT) ? INVERS : 0;
    lcd_putsAtt(  3*FW,y,PSTR(STR_COPY_TRIM),s_noHi ? 0 : attr);
//    if(attr && event==EVT_KEY_LONG(KEY_MENU))
//		{
//      s_noHi = NO_HI_LEN;
//      killEvents(event);
//  	  s_editMode = 0 ;
//      setStickCenter(); //if highlighted and menu pressed - copy trims
//    }
	}
}


#define PARAM_OFS   17*FW

static uint8_t onoffItem( uint8_t event, uint8_t value, uint8_t y, uint8_t condition, uint8_t flags )
{
	menu_lcd_onoff( PARAM_OFS, y, value, condition ) ;
  if(condition) value = checkIncDec( event, value, 0, 1, flags ) ;
  return value ;
}

static uint8_t onoffItem_m( uint8_t event, uint8_t value, uint8_t y, uint8_t condition )
{
	return onoffItem( event, value, y, condition, EE_MODEL ) ;
}

static uint8_t offonItem_m( uint8_t event, uint8_t value, uint8_t y, uint8_t condition )
{
	return 1-onoffItem_m( event, 1-value, y, condition ) ;
}

static uint8_t onoffMenuItem( uint8_t event, uint8_t value, uint8_t y, const char *s, uint8_t condition, uint8_t flags )
{
    lcd_puts_Pleft(y, s);
		return onoffItem( event, value, y, condition, EE_MODEL ) ;
}

static uint8_t onoffMenuItem_g( uint8_t event, uint8_t value, uint8_t y, const char *s, uint8_t condition )
{
	return onoffMenuItem( event, value, y, s, condition, EE_GENERAL ) ;
}

static uint8_t offonMenuItem_g( uint8_t event, uint8_t value, uint8_t y, const char *s, uint8_t condition )
{
	return 1-onoffMenuItem_g( event, 1-value, y, s, condition ) ;
}


static uint8_t onoffMenuItem_m( uint8_t event, uint8_t value, uint8_t y, const char *s, uint8_t condition )
{
    if(condition) CHECK_INCDEC_H_MODELVAR_0( event, value, 1 ) ;
    menu_lcd_onoff( PARAM_OFS, y, value, condition ) ;
    lcd_puts_Pleft(y, s);
    return value ;
}

uint8_t hyphinvMenuItem_m( uint8_t event, uint8_t value, uint8_t y, const char *s, uint8_t condition )
{
	value = onoffMenuItem_m( event, value, y, s, condition ) ;
	menu_lcd_HYPHINV( PARAM_OFS, y, value, condition ) ;		// Overtype the ON/OFF
  return value ;
}

uint8_t onoffMenuItem( uint8_t value, uint8_t y, const char *s, uint8_t sub, int8_t subN, uint8_t event )
{
  lcd_puts_Pleft(y, s);
  menu_lcd_onoff( PARAM_OFS, y, value, sub==subN ) ;
  if(sub==subN) CHECK_INCDEC_H_GENVAR_0(event, value, 1);
  return value ;
}



#ifdef PCBSKY
 #ifdef REVX
  #define MAX_TEL_OPTIONS		6
  const uint8_t TelOptions[] = {1,2,3,4,5,6} ;
 #else
  #define MAX_TEL_OPTIONS			4
  const uint8_t TelOptions[] = {1,2,3,6} ;
 #endif
#endif

#ifdef PCBX9D
 #define MAX_TEL_OPTIONS		2
 const uint8_t TelOptions[] = {1,2} ;
#endif

#ifdef FRSKY
void menuProcTelemetry(uint8_t event)
{
	static uint8_t columns = 0 ;

#ifdef PCBSKY
 #ifdef REVX
  #define TDATAITEMS	42
 #else
  #define TDATAITEMS	41
 #endif
#endif

#ifdef PCBX9D
  #define TDATAITEMS	40
#endif

		 
	TITLE(PSTR(STR_TELEMETRY));
	static MState2 mstate2;
//#ifdef PCBSKY
//	static const uint8_t mstate_tab[] = {0, 1, 1, 2, 2, 1, 2,2,1,1,0,1,1,0} ;
//#else
//	static const uint8_t mstate_tab[] = {0, 1, 1, 2, 2, 1, 2,2,1,1,0} ;
//#endif
	if (SubMenuFromIndex)
	{
#ifdef PCBSKY
		event = mstate2.check(event,0,NULL,0,&columns,0, TDATAITEMS ) ;
#else
		event = mstate2.check(event,0,NULL,0,&columns,0, TDATAITEMS ) ;
#endif
	}
	else
	{
#ifdef REVX
		event = mstate2.check(event,e_Telemetry,menuTabModel,DIM(menuTabModel),&columns,0,TDATAITEMS) ;
#else
		event = mstate2.check(event,e_Telemetry,menuTabModel,DIM(menuTabModel),&columns,0,TDATAITEMS) ;
#endif
//    MENU(PSTR(STR_CUST_SWITCH), menuTabModel, e_Switches, NUM_SKYCSW+1, {0, 3/*repeated...*/});
	}
//#ifdef REVX
//    MENU(PSTR(STR_TELEMETRY), menuTabModel, e_Telemetry, 13, {0, 1, 1, 2, 2, 1, 2,2,1,1,0,1});
//#else
//    MENU(PSTR(STR_TELEMETRY), menuTabModel, e_Telemetry, 11, {0, 1, 1, 2, 2, 1, 2,2,1,1,0});
//#endif

uint8_t  sub   = mstate2.m_posVert;
uint8_t subSub = mstate2.m_posHorz;
uint8_t blink;
uint8_t y = 2*FH;

	switch(event)
	{
    case EVT_KEY_BREAK(KEY_DOWN):
    case EVT_KEY_BREAK(KEY_UP):
    case EVT_KEY_BREAK(KEY_LEFT):
    case EVT_KEY_BREAK(KEY_RIGHT):
      if(s_editMode)
			{
         FrskyAlarmSendState |= 0x30 ;	 // update Fr-Sky module when edit mode exited
         FRSKY_setModelAlarms(); // update Fr-Sky module when edit mode exited
			}
    break ;
		case EVT_ENTRY :
  		FrskyAlarmSendState |= 0x40 ;		// Get RSSI/TSSI alarms
		break ;
	}

	blink = InverseBlink ;
	uint8_t subN = 1;
	lcd_puts_P( 20*FW-4, 7*FH, XPSTR("->") ) ;
	columns = 0 ;

 if ( sub < 8 )
 {
	lcd_puts_Pleft(FH, PSTR(STR_USR_PROTO));
	{
		uint8_t b ;
		uint8_t attr = 0 ;

		b = g_model.telemetryProtocol ;
//		lcd_outhex4(64,0, (g_model.DsmTelemetry<<8) | b ) ;
		for ( uint32_t c = 0 ; c < MAX_TEL_OPTIONS ; c += 1 )
		{
			if ( TelOptions[c] == b )
			{
				b = c ;
				break ;
			}
		}
		if(sub==subN)
		{
			columns = 1 ;

			if (subSub==0)
			{
				attr = blink ;
				CHECK_INCDEC_H_MODELVAR(event,b,0,MAX_TEL_OPTIONS-1) ;
			}
		}
		b = TelOptions[b] ;
		g_model.telemetryProtocol = b ;
		switch ( b )
		{
			case TELEMETRY_WSHHI :
				g_model.FrSkyUsrProto = 1 ;
				g_model.DsmTelemetry = 0 ;
			break ;

			case TELEMETRY_DSM :
				g_model.FrSkyUsrProto = 0 ;
				g_model.DsmTelemetry = 1 ;
			break ;

			case TELEMETRY_FRSKY :
			default :
				g_model.FrSkyUsrProto = 0 ;
				g_model.DsmTelemetry = 0 ;
			break ;
		}
		b -= 1 ;
		lcd_putsAttIdx( 10*FW, FH, PSTR(STR_FRHUB_WSHHI), b, attr ) ;

		attr = 0 ;
		b = g_model.FrSkyImperial ;
		if(sub==subN && subSub==1)
		{
			attr = INVERS ;
			CHECK_INCDEC_H_MODELVAR_0(event,b,1); g_model.FrSkyImperial = b ;
		}
		lcd_putsAttIdx( 16*FW, FH, PSTR(STR_MET_IMP), b, attr ) ;
	}
	subN++;

	for (int i=0; i<2; i++)
	{
    lcd_puts_Pleft(y, PSTR(STR_A_CHANNEL)) ;
    lcd_putc(FW, y, '1'+i);
    putsTelemValue(16*FW, y, 255, i, (sub==subN && subSub==0 ? blink:0)|NO_UNIT ) ;
    putsTelemValue( 21*FW, y, frskyTelemetry[i].value, i,  NO_UNIT ) ;
    //    lcd_putsnAtt(16*FW, y, PSTR("v-")+g_model.frsky.channels[i].type, 1, (sub==subN && subSub==1 ? blink:0));
    lcd_putsAttIdx(16*FW, y, XPSTR("\001v-VA"), g_model.frsky.channels[i].type, (sub==subN && subSub==1 ? blink:0));

    if (sub==subN)
		{
			columns = 1 ;
			if ( (s_editMode || P1values.p1valdiff))
			{
        switch (subSub)
				{
        	case 0:
            g_model.frsky.channels[i].ratio = checkIncDec16(event, g_model.frsky.channels[i].ratio, 0, 255, EE_MODEL);
          break;
	        case 1:
            //            CHECK_INCDEC_H_MODELVAR(event, g_model.frsky.channels[i].type, 0, 1);
            CHECK_INCDEC_H_MODELVAR_0(event, g_model.frsky.channels[i].type, 3);
          break;
        }
			}
    }
    subN++; y+=FH;

    for (int j=0; j<2; j++)
		{
        uint8_t ag;
        uint8_t al;
        al = ALARM_LEVEL(i, j);
        ag = ALARM_GREATER(i, j);
        lcd_putsAtt(4, y, PSTR(STR_ALRM), 0);
        lcd_putsAttIdx(6*FW, y, PSTR(STR_YELORGRED),al,(sub==subN && subSub==0 ? blink:0));
        lcd_putsnAtt(11*FW, y, XPSTR("<>")+ag,1,(sub==subN && subSub==1 ? blink:0));
        putsTelemValue(16*FW, y, g_model.frsky.channels[i].alarms_value[j], i, (sub==subN && subSub==2 ? blink:0)|NO_UNIT ) ;

        if(sub==subN)
				{
					columns = 2 ;
					if ((s_editMode || P1values.p1valdiff))
					{
            uint8_t original ;
            uint8_t value ;
            switch (subSub)
						{
            	case 0:
                value = checkIncDec(event, al, 0, 3, EE_MODEL) ;
                original = g_model.frsky.channels[i].alarms_level ;
                g_model.frsky.channels[i].alarms_level = j ? ( (original & 0xF3) | value << 2 ) : ( (original & 0xFC) | value ) ;
              break;
            	case 1:
                value = checkIncDec(event, ag, 0, 3, EE_MODEL) ;
                original = g_model.frsky.channels[i].alarms_greater ;
                g_model.frsky.channels[i].alarms_greater = j ? ( (original & 0xFD) | value << 1 ) : ( (original & 0xFE) | value ) ;
                if(checkIncDec_Ret)
                    FRSKY_setModelAlarms();
              break;
            	case 2:
                g_model.frsky.channels[i].alarms_value[j] = checkIncDec16(event, g_model.frsky.channels[i].alarms_value[j], 0, 255, EE_MODEL);
              break;
            }
					}
        }
        subN++; y+=FH;
    }
	}
 }
#ifdef PCBSKY
 else if ( sub < 13) // sub>=8
#else
 else if ( sub < 11) // sub>=8
#endif
 {
	uint8_t subN = 8 ;
	uint8_t b ;

	if (sub==subN)
	{
		columns = 1 ;
	}
 	lcd_puts_Pleft( y, XPSTR( "RSSI Warn") ) ;
	uint8_t attr = ( ( (sub==subN) && (subSub==0) ) ? InverseBlink : 0) ;
	lcd_outdezAtt( 15*FW, y, g_model.rssiOrange + 45, attr ) ;
  if( attr) CHECK_INCDEC_H_MODELVAR( event, g_model.rssiOrange, -30, 30 ) ;
	attr = ( ( (sub==subN) && (subSub==1) ) ? InverseBlink : 0) ;
	b = 1-g_model.enRssiOrange ;
	menu_lcd_onoff( PARAM_OFS+1, y, b, attr ) ;
  if( attr) { CHECK_INCDEC_H_MODELVAR_0( event, b, 1 ) ; g_model.enRssiOrange = 1-b ; }
	subN++; y+=FH;
		
	if (sub==subN)
	{
		columns = 1 ;
	}
 	lcd_puts_Pleft( y, XPSTR( "RSSI Critical") ) ;
	attr = ( ( (sub==subN) && (subSub==0) ) ? InverseBlink : 0) ;
  if( attr) CHECK_INCDEC_H_MODELVAR( event, g_model.rssiRed, -30, 30 ) ;
	lcd_outdezAtt( 15*FW, y, g_model.rssiRed + 42, attr ) ;
	attr = ( ( (sub==subN) && (subSub==1) ) ? InverseBlink : 0) ;
	b = 1-g_model.enRssiRed ;
	menu_lcd_onoff( PARAM_OFS+1, y, b, attr ) ;
  if( attr) { CHECK_INCDEC_H_MODELVAR_0( event, b, 1 ) ; g_model.enRssiRed = 1-b ; }

	subN++; y+=FH;
 
	attr = (sub==subN) ? InverseBlink : 0 ;
  lcd_puts_Pleft(y, XPSTR( "Rx Voltage") ) ;
	lcd_outdezAtt( 16*FW, y, g_model.rxVratio, attr|PREC1 ) ;
  lcd_putc(Lcd_lastPos, y, 'v' ) ;
	lcd_outdezAtt( 21*FW, y, FrskyHubData[FR_RXV] * g_model.rxVratio / 255, PREC1 ) ;
	if( attr) { g_model.rxVratio = checkIncDec16(event, g_model.rxVratio, 0, 255, EE_MODEL ) ; }
	subN++; y+=FH;

#ifdef PCBSKY
	if ( FrskyTelemetryType == 2 )
	{
		if (sub==subN)
		{
			columns = 1 ;
		}
 		lcd_puts_Pleft( y, XPSTR( "DSM Warning") ) ;
		attr = ( ( (sub==subN) && (subSub==0) ) ? InverseBlink : 0) ;
  	lcd_putsAttIdx(13*FW, y, XPSTR("\006fades lossesholds "),g_model.dsmLinkData.sourceWarn,attr);
  	if( attr) { CHECK_INCDEC_H_MODELVAR_0( event, g_model.dsmLinkData.sourceWarn, 2 ) ; }
		attr = ( ( (sub==subN) && (subSub==1) ) ? InverseBlink : 0) ;
		lcd_outdezAtt( 21*FW, y, g_model.dsmLinkData.levelWarn, attr ) ;
  	if( attr) { CHECK_INCDEC_H_MODELVAR_0( event, g_model.dsmLinkData.levelWarn, 40 ) ; }
		subN++; y+=FH;

		if (sub==subN)
		{
			columns = 1 ;
		}
 		lcd_puts_Pleft( y, XPSTR( "DSM Critical") ) ;
		attr = ( ( (sub==subN) && (subSub==0) ) ? InverseBlink : 0) ;
  	lcd_putsAttIdx(13*FW, y, XPSTR("\006fades lossesholds "),g_model.dsmLinkData.sourceCritical,attr);
  	if( attr) { CHECK_INCDEC_H_MODELVAR_0( event, g_model.dsmLinkData.sourceCritical, 2 ) ; }
		attr = ( ( (sub==subN) && (subSub==1) ) ? InverseBlink : 0) ;
		lcd_outdezAtt( 21*FW, y, g_model.dsmLinkData.levelCritical, attr ) ;
  	if( attr) { CHECK_INCDEC_H_MODELVAR_0( event, g_model.dsmLinkData.levelCritical, 40 ) ; }
		subN++; y+=FH;
	}
	else
	{
		if ( (sub==subN) || (sub==subN+1))
		{
			if ( M_lastVerticalPosition	< mstate2.m_posVert )
			{
				mstate2.m_posVert = 13 ;
			}
			else if ( M_lastVerticalPosition > mstate2.m_posVert )
			{
				mstate2.m_posVert = 10 ;
			}
		}
	}
#endif

 }
#ifdef PCBSKY
 else if ( sub < 18) // sub>=8
#else
 else if ( sub < 16) // sub>=8
#endif
	{
#ifdef PCBSKY
		uint8_t subN = 13 ;
#else
		uint8_t subN = 11 ;
#endif
		y = FH ;
		lcd_puts_Pleft(y, PSTR(STR_NUM_BLADES));
 	  lcd_outdezAtt(14*FW, y, g_model.numBlades, (sub==subN) ? INVERS : 0) ;
  	if(sub==subN) CHECK_INCDEC_H_MODELVAR(event, g_model.numBlades, 1, 127 ) ;
  	y += FH ;
		subN++;
  
		lcd_puts_Pleft( y, PSTR(STR_ALT_ALARM));
  	lcd_putsAttIdx(11*FW, y, PSTR(STR_OFF122400),g_model.FrSkyAltAlarm,(sub==subN ? blink:0));
  	if(sub==subN) {
  		g_model.FrSkyAltAlarm = checkIncDec16(event, g_model.FrSkyAltAlarm, 0, 2, EE_MODEL);
		}
  	y += FH ;
  	subN++;
  
		lcd_puts_Pleft( y, PSTR(STR_VOLT_THRES));
  	lcd_outdezNAtt(  14*FW, y, g_model.frSkyVoltThreshold * 2 ,((sub==subN) ? blink:0) | PREC2, 4);
  	if(sub==subN)
		{
  	  g_model.frSkyVoltThreshold=checkIncDec16(event, g_model.frSkyVoltThreshold, 0, 210, EE_MODEL);
  	}
  	y += FH ;
		subN++;
	
  	lcd_puts_Pleft( y, PSTR(STR_GPS_ALTMAIN) ) ;
  	menu_lcd_onoff( PARAM_OFS, y, g_model.FrSkyGpsAlt, sub==subN ) ;
  	if(sub==subN) CHECK_INCDEC_H_MODELVAR_0(event, g_model.FrSkyGpsAlt, 1);
  	y += FH ;
		subN++;
		
		if (sub==subN)
		{
			columns = 1 ;
		}
		uint16_t value = g_model.frskyAlarms.alarmData[0].frskyAlarmLimit << 6 ;
		lcd_puts_Pleft(y, PSTR(STR_MAH_ALARM));
  	uint8_t attr = ((sub==subN && subSub==0) ? InverseBlink : 0);
		uint8_t active = (attr && s_editMode) ;
  	lcd_outdezAtt( 14*FW, y, value, attr ) ;
		if ( active )
		{
  		g_model.frskyAlarms.alarmData[0].frskyAlarmLimit = checkIncDec16(event, g_model.frskyAlarms.alarmData[0].frskyAlarmLimit, 0, 200, EE_MODEL);
		}
  	attr = ((sub==subN && subSub==1) ? InverseBlink : 0);
		active = (attr && s_editMode) ;
		lcd_putsAttIdx(15*FW, y, PSTR(STR_SOUNDS), g_model.frskyAlarms.alarmData[0].frskyAlarmSound,attr);
		if ( active )
		{
  		CHECK_INCDEC_H_MODELVAR_0( event, g_model.frskyAlarms.alarmData[0].frskyAlarmSound, 15 ) ;
		}
  	subN++;


	}
#ifdef PCBSKY
 else if ( sub < 24+6) // sub>=8
#else
 else if ( sub < 22+6) // sub>=8
#endif
	{
		uint8_t *pindex ;
		uint8_t chr = '1' ;
  	lcd_puts_Pleft( FH, PSTR(STR_CUSTOM_DISP) );
		pindex = g_model.customDisplayIndex ;
#ifdef PCBSKY
		uint8_t subN = 18 ;
		if ( sub >= 24 )
		{
			subN = 24 ;
			pindex = g_model.customDisplay2Index ;
			chr = '2' ;
		}
#else
		uint8_t subN = 16 ;
		if ( sub >= 22 )
		{
			subN = 22 ;
			pindex = g_model.customDisplay2Index ;
			chr = '2' ;
		}
#endif
		lcd_putc( 15*FW, FH, chr ) ;

		for (uint8_t j=0; j<6; j++)
		{
			uint8_t x = ( j & 1 ) ? 64 : 0 ;
			uint8_t y = ( ( j & 6 ) + 2 ) * FH ;
	  	uint8_t attr = ((sub==subN) ? InverseBlink : 0);
			if ( pindex[j] )
			{
				putsAttIdxTelemItems( x, y, pindex[j], attr ) ;
			}
			else
			{
    		lcd_putsAtt(  x, y, XPSTR("----"), attr ) ;
			}
  		if(sub==subN) CHECK_INCDEC_H_MODELVAR_0(event, pindex[j], NUM_TELEM_ITEMS) ;
			subN++;
		}
	}
	else if ( sub <= TDATAITEMS - 7 )
	{
#ifdef PCBSKY
		uint8_t subN = 24+6 ;
#else
		uint8_t subN = 22+6 ;
#endif
		y = FH ;
  	lcd_puts_Pleft( y, PSTR(STR_BT_TELEMETRY) );
  	menu_lcd_onoff( PARAM_OFS, y, g_model.bt_telemetry, sub==subN ) ;
  	if(sub==subN) CHECK_INCDEC_H_MODELVAR_0(event, g_model.bt_telemetry, 1);
  	y += FH ;
		subN += 1 ;
  	
		lcd_puts_Pleft( y, PSTR(STR_FRSKY_COM_PORT) );
    uint8_t attr = (sub == subN) ? INVERS : 0 ;
  	lcd_putcAtt( 16*FW, y, g_model.frskyComPort + '1', attr ) ;
		if (attr) CHECK_INCDEC_H_MODELVAR_0( event, g_model.frskyComPort, 1 ) ;
	  y += FH ;
		subN += 1 ;
  	
#ifdef REVX
		uint8_t previous = g_model.telemetryRxInvert ;
		lcd_puts_Pleft( y, PSTR(STR_INVERT_COM1) );
  	menu_lcd_onoff( PARAM_OFS, y, g_model.telemetryRxInvert, sub==subN ) ;
  	if(sub==subN) CHECK_INCDEC_H_MODELVAR_0(event, g_model.telemetryRxInvert, 1);
		if ( g_model.telemetryRxInvert != previous )
		{
			if ( g_model.telemetryRxInvert )
			{
				setMFP() ;
			}
			else
			{
				clearMFP() ;
			}
		}
	  y += FH ;
		subN += 1 ;
#endif
		 
		lcd_puts_Pleft( y, PSTR(STR_FAS_OFFSET) );
    attr = PREC1 ;
		if ( (sub == subN) )
		{
			attr = INVERS | PREC1 ;
      CHECK_INCDEC_H_MODELVAR_0( event, g_model.FASoffset, 15 ) ;
		}
  	lcd_outdezAtt( 15*FW, y, g_model.FASoffset, attr ) ;
	  y += FH ;
		subN += 1 ;

		lcd_puts_Pleft( y, XPSTR("COM2 Func.") );
		uint8_t b = g_model.com2Function ;
    attr = 0 ;
		if ( (sub == subN) )
		{
			attr = INVERS ;
#ifdef PCBSKY
	  	CHECK_INCDEC_H_MODELVAR_0(event, g_model.com2Function, 3 ) ;
#endif
#ifdef PCBX9D
	  	CHECK_INCDEC_H_MODELVAR_0(event, g_model.com2Function, 3 ) ;
#endif
		}
#ifdef PCBSKY
		lcd_putsAttIdx(12*FW, y, XPSTR("\011TelemetrySbusTrainSbus57600BTdirect "), g_model.com2Function, attr ) ;
#endif
#ifdef PCBX9D
		lcd_putsAttIdx(12*FW, y, XPSTR("\011TelemetrySbusTrainSbus57600CppmTrain"), g_model.com2Function, attr ) ;
#endif
		if ( g_model.com2Function != b )
		{
#ifdef PCBX9D
			if ( b == 3 )
			{
				stop_serial_trainer_capture() ;
			}
#endif
			if ( g_model.com2Function == COM2_FUNC_SBUSTRAIN )
			{
				UART_Sbus_configure( Master_frequency ) ;
			}
			else if ( g_model.com2Function == COM2_FUNC_SBUS57600 )
			{
				UART_Sbus57600_configure( Master_frequency ) ;
			}
#ifdef PCBSKY
			else if( g_model.com2Function == COM2_FUNC_BTDIRECT )
			{
				UART_Configure( g_model.com2Baudrate, Master_frequency ) ;
			}
#endif
#ifdef PCBX9D
			else if( g_model.com2Function == COM2_FUNC_CPPMTRAIN )
			{
				init_serial_trainer_capture() ;
			}
#endif
			else
			{
				if ( g_model.frskyComPort == 1 )
				{
					telemetry_init( TEL_FRSKY_HUB ) ;
				}
				else
				{
					// Set for debug use
#ifdef PCBSKY
					UART_Configure( 9600, Master_frequency ) ;
#endif
#ifdef PCBX9D
					x9dConsoleInit() ;
#endif
				}
			}
		}
	  y += FH ;
		subN += 1 ;
		
		lcd_puts_Pleft( y, XPSTR("COM2 Baudrate") );
    attr = 0 ;
		b = g_model.com2Baudrate ;
		if ( (sub == subN) )
		{
			attr = INVERS ;
	  	CHECK_INCDEC_H_MODELVAR_0(event, g_model.com2Baudrate, 4 ) ;
		}
		lcd_putsAttIdx(15*FW, y, XPSTR("\006  9600 19200 38400 57600115200"), g_model.com2Baudrate, attr ) ;
#ifdef PCBSKY
		if ( b != g_model.com2Baudrate )
		{
			if( g_model.com2Function == COM2_FUNC_BTDIRECT )
			{
				UART_Configure( g_model.com2Baudrate, Master_frequency ) ;
			}
		}
#endif	// PCBSKY
	  y += FH ;
		subN += 1 ;

//#ifdef PCBSKY
//		lcd_puts_Pleft( y, XPSTR("BT as Trainer") );
//		if ( (sub == subN) )
//		{
//			CHECK_INCDEC_H_MODELVAR_0(event, g_model.BTfunction, 1 ) ;
//		}
//  	menu_lcd_onoff( PARAM_OFS, y, g_model.BTfunction, sub==subN ) ;
//#endif
		
	}
	else
	{
		uint8_t subN = TDATAITEMS - 6 ;
		// Vario
   	for( uint8_t j=0 ; j<7 ; j += 1 )
		{
			uint8_t b ;
      uint8_t attr = (sub==subN) ? INVERS : 0 ;
			uint8_t y = (1+j)*FH ;

			switch ( j )
			{
				case 0 :
					lcd_puts_Pleft( y, PSTR(STR_VARIO_SRC) ) ;
					lcd_putsAttIdx( 15*FW, y, PSTR(STR_VSPD_A2), g_model.varioData.varioSource, attr ) ;
   		  	if(attr)
					{
						CHECK_INCDEC_H_MODELVAR_0(event, g_model.varioData.varioSource, 2+NUM_SCALERS ) ;
   		  	}
				break ;
				
				case 1 :
					lcd_puts_Pleft( y, PSTR(STR_2SWITCH) ) ;
					g_model.varioData.swtch = edit_dr_switch( 15*FW, y, g_model.varioData.swtch, attr, attr ? EDIT_DR_SWITCH_EDIT : 0, event ) ;
				break ;

				case 2 :
					lcd_puts_Pleft( y, PSTR(STR_2SENSITIVITY) ) ;
 					lcd_outdezAtt( 17*FW, y, g_model.varioData.param, attr) ;
   			  if(attr)
					{
						CHECK_INCDEC_H_MODELVAR_0(event, g_model.varioData.param, 50 ) ;
	   		  }
				break ;

				case 3 :
	        b = 1-g_model.varioData.sinkTones ;
					g_model.varioData.sinkTones = 1-onoffMenuItem_m( event, b, y, PSTR(STR_SINK_TONES), attr ) ;
				break ;

				case 4 :
					lcd_puts_Pleft( y, PSTR(STR_LOG_SWITCH) ) ;
					g_model.logSwitch = edit_dr_switch( 15*FW, y, g_model.logSwitch, attr, attr ? EDIT_DR_SWITCH_EDIT : 0, event ) ;
				break ;

				case 5 :
					lcd_puts_Pleft( y, PSTR(STR_LOG_RATE) ) ;
					lcd_putsAttIdx( 15*FW, y, XPSTR("\0041.0s2.0s"), g_model.logRate, attr ) ;
   			  if(attr)
					{
						CHECK_INCDEC_H_MODELVAR_0(event, g_model.logRate, 1 ) ;
	   		  }
				break ;

				case 6 :
					lcd_puts_Pleft( y, XPSTR("Current Source" ) ) ;
					lcd_putsAttIdx( 15*FW, y, XPSTR("\004----A1  A2  FASVSC1 SC2 SC3 SC4 SC5 SC6 SC7 SC8 "), g_model.currentSource, attr ) ;
   			  if(attr)
					{
						CHECK_INCDEC_H_MODELVAR_0(event, g_model.currentSource, 3 + NUM_SCALERS ) ;
	   		  }
				break ;

			}

			subN += 1 ;
		}
	}

}

extern uint8_t frskyRSSIlevel[2] ;
extern uint8_t frskyRSSItype[2] ;

//VarioData VarioSetup ;
#ifdef REVX
#define T2COUNT_ITEMS	28
#else
 #ifdef PCBSKY
 #define T2COUNT_ITEMS	27
 #else
 #define T2COUNT_ITEMS	26
 #endif
#endif
void menuProcTelemetry2(uint8_t event)
{
  MENU(PSTR(STR_TELEMETRY2), menuTabModel, e_Telemetry2, T2COUNT_ITEMS, {0, 1, 1, 1, 0});

	uint8_t  sub    = mstate2.m_posVert;
	uint8_t subSub = mstate2.m_posHorz;
	uint8_t blink;
	uint8_t y = 1*FH;
	int16_t value ;

	switch(event)
	{
    case EVT_KEY_BREAK(KEY_DOWN):
    case EVT_KEY_BREAK(KEY_UP):
    case EVT_KEY_BREAK(KEY_LEFT):
    case EVT_KEY_BREAK(KEY_RIGHT):
      if(s_editMode)
         FrskyAlarmSendState |= 0x30 ;	 // update Fr-Sky module when edit mode exited
    break ;
		case EVT_ENTRY :
  		FrskyAlarmSendState |= 0x40 ;		// Get RSSI/TSSI alarms
		break ;
	}
	blink = InverseBlink ;
	
	if ( sub < 8 )
	{
		uint8_t subN = 1;

		for (uint8_t j=0; j<2; j++)
		{
  	  lcd_puts_Pleft( y, PSTR(STR_TX_RSSIALRM) );
  	  if ( j == 1 )
  	  {
  	      lcd_putcAtt( 0, y, 'R', 0 ) ;
  	  }
  	  lcd_putsAttIdx(11*FW, y, PSTR(STR_YELORGRED),frskyRSSItype[j],(sub==subN && subSub==0 ? blink:0));
  	  lcd_outdezNAtt(17*FW, y, frskyRSSIlevel[j], (sub==subN && subSub==1 ? blink:0), 3);

  	  if(sub==subN && (s_editMode || P1values.p1valdiff)) {
  	    	switch (subSub) {
  	    	case 0:
  	    	    frskyRSSItype[j] = checkIncDec(event, frskyRSSItype[j], 0, 3, EE_MODEL) ;
  	    	    break;
  	    	case 1:
  	    	    frskyRSSIlevel[j] = checkIncDec16(event, frskyRSSIlevel[j], 0, 120, EE_MODEL);
  	    	    break;
  	    	}
  	  }
  	  subN++; y+=FH;
		}

		value = g_model.frskyAlarms.alarmData[0].frskyAlarmLimit << 6 ;
		lcd_puts_Pleft(3*FH, PSTR(STR_MAH_ALARM));
  	uint8_t attr = ((sub==subN && subSub==0) ? InverseBlink : 0);
		uint8_t active = (attr && s_editMode) ;
  	lcd_outdezAtt( 14*FW, 3*FH, value, attr ) ;
		if ( active )
		{
  		g_model.frskyAlarms.alarmData[0].frskyAlarmLimit = checkIncDec16(event, g_model.frskyAlarms.alarmData[0].frskyAlarmLimit, 0, 200, EE_MODEL);
		}
  	attr = ((sub==subN && subSub==1) ? InverseBlink : 0);
		active = (attr && s_editMode) ;
		lcd_putsAttIdx(15*FW, 3*FH, PSTR(STR_SOUNDS), g_model.frskyAlarms.alarmData[0].frskyAlarmSound,attr);
		if ( active )
		{
  		CHECK_INCDEC_H_MODELVAR_0( event, g_model.frskyAlarms.alarmData[0].frskyAlarmSound, 15 ) ;
		}
  	subN++;

		lcd_puts_Pleft(4*FH, PSTR(STR_NUM_BLADES));
 	  lcd_outdezAtt(14*FW, 4*FH, g_model.numBlades, (sub==subN) ? INVERS : 0) ;
  	if(sub==subN) CHECK_INCDEC_H_MODELVAR(event, g_model.numBlades, 1, 127 ) ;
  	subN++;
  
		lcd_puts_Pleft(5*FH, PSTR(STR_ALT_ALARM));
  	lcd_putsAttIdx(11*FW, 5*FH, PSTR(STR_OFF122400),g_model.FrSkyAltAlarm,(sub==subN ? blink:0));
  	if(sub==subN) {
  		g_model.FrSkyAltAlarm = checkIncDec16(event, g_model.FrSkyAltAlarm, 0, 2, EE_MODEL);
		}
  	subN++;
  
		lcd_puts_Pleft(6*FH, PSTR(STR_VOLT_THRES));
  	lcd_outdezNAtt(  14*FW, 6*FH, g_model.frSkyVoltThreshold * 2 ,((sub==subN) ? blink:0) | PREC2, 4);
  	if(sub==subN)
		{
  	  g_model.frSkyVoltThreshold=checkIncDec16(event, g_model.frSkyVoltThreshold, 0, 210, EE_MODEL);
  	}
		subN++;
	
  	lcd_puts_Pleft( 7*FH, PSTR(STR_GPS_ALTMAIN) ) ;
  	menu_lcd_onoff( PARAM_OFS, 7*FH, g_model.FrSkyGpsAlt, sub==subN ) ;
  	if(sub==subN) CHECK_INCDEC_H_MODELVAR_0(event, g_model.FrSkyGpsAlt, 1);
	}
	else if ( sub < 14 )
	{
		uint8_t subN = 8 ;

  	lcd_puts_Pleft( FH, PSTR(STR_CUSTOM_DISP) );
		for (uint8_t j=0; j<6; j++)
		{
	  	uint8_t attr = ((sub==subN) ? InverseBlink : 0);
			if ( g_model.customDisplayIndex[j] )
			{
				putsAttIdxTelemItems( 0, j*FH + 2*FH, g_model.customDisplayIndex[j], attr ) ;
			}
			else
			{
    		lcd_putsAtt(  0, j*FH + 2*FH, XPSTR("----"), attr ) ;
			}
	  	if(sub==subN) g_model.customDisplayIndex[j] = checkIncDec( event, g_model.customDisplayIndex[j], 0, NUM_TELEM_ITEMS, EE_MODEL ) ;
			subN++;
		}
	}
	else if ( sub < T2COUNT_ITEMS - 7 )
	{
		uint8_t subN = 14 ;
  	lcd_puts_Pleft( FH, PSTR(STR_BT_TELEMETRY) );
  	menu_lcd_onoff( PARAM_OFS, FH, g_model.bt_telemetry, sub==subN ) ;
  	if(sub==subN) CHECK_INCDEC_H_MODELVAR_0(event, g_model.bt_telemetry, 1);
		subN += 1 ;
  	
		lcd_puts_Pleft( 2*FH, PSTR(STR_FRSKY_COM_PORT) );
    uint8_t attr = (sub == subN) ? INVERS : 0 ;
  	lcd_putcAtt( 16*FW, 2*FH, g_model.frskyComPort + '1', attr ) ;
		if (attr) CHECK_INCDEC_H_MODELVAR_0( event, g_model.frskyComPort, 1 ) ;
		subN += 1 ;
  	
#ifdef REVX
		uint8_t previous = g_model.telemetryRxInvert ;
		lcd_puts_Pleft( 3*FH, PSTR(STR_INVERT_COM1) );
  	menu_lcd_onoff( PARAM_OFS, 3*FH, g_model.telemetryRxInvert, sub==subN ) ;
  	if(sub==subN) CHECK_INCDEC_H_MODELVAR_0(event, g_model.telemetryRxInvert, 1);
		if ( g_model.telemetryRxInvert != previous )
		{
			if ( g_model.telemetryRxInvert )
			{
				setMFP() ;
			}
			else
			{
				clearMFP() ;
			}
		}
		subN += 1 ;

#endif
		 
		lcd_puts_Pleft( 4*FH, PSTR(STR_FAS_OFFSET) );
    attr = PREC1 ;
		if ( (sub == subN) )
		{
			attr = INVERS | PREC1 ;
      CHECK_INCDEC_H_MODELVAR_0( event, g_model.FASoffset, 15 ) ;
		}
  	lcd_outdezAtt( 15*FW, 4*FH, g_model.FASoffset, attr ) ;
		subN += 1 ;

		lcd_puts_Pleft( 5*FH, XPSTR("COM2 Func.") );
		uint8_t b = g_model.com2Function ;
    attr = 0 ;
		if ( (sub == subN) )
		{
			attr = INVERS ;
#ifdef PCBSKY
	  	CHECK_INCDEC_H_MODELVAR_0(event, g_model.com2Function, 3 ) ;
#endif
#ifdef PCBX9D
	  	CHECK_INCDEC_H_MODELVAR_0(event, g_model.com2Function, 3 ) ;
#endif
		}
#ifdef PCBSKY
		lcd_putsAttIdx(12*FW, 5*FH, XPSTR("\011TelemetrySbusTrainSbus57600BTdirect "), g_model.com2Function, attr ) ;
#endif
#ifdef PCBX9D
		lcd_putsAttIdx(12*FW, 5*FH, XPSTR("\011TelemetrySbusTrainSbus57600CppmTrain"), g_model.com2Function, attr ) ;
#endif
		if ( g_model.com2Function != b )
		{
#ifdef PCBX9D
			if ( b == 3 )
			{
				stop_serial_trainer_capture() ;
			}
#endif
			if ( g_model.com2Function == COM2_FUNC_SBUSTRAIN )
			{
				UART_Sbus_configure( Master_frequency ) ;
			}
			else if ( g_model.com2Function == COM2_FUNC_SBUS57600 )
			{
				UART_Sbus57600_configure( Master_frequency ) ;
			}
#ifdef PCBSKY
			else if( g_model.com2Function == COM2_FUNC_BTDIRECT )
			{
				UART_Configure( g_model.com2Baudrate, Master_frequency ) ;
			}
#endif
#ifdef PCBX9D
			else if( g_model.com2Function == COM2_FUNC_CPPMTRAIN )
			{
				init_serial_trainer_capture() ;
			}
#endif
			else
			{
				if ( g_model.frskyComPort == 1 )
				{
					telemetry_init( TEL_FRSKY_HUB ) ;
				}
				else
				{
					// Set for debug use
#ifdef PCBSKY
					UART_Configure( 9600, Master_frequency ) ;
#endif
#ifdef PCBX9D
					x9dConsoleInit() ;
#endif
				}
			}
		}
		subN += 1 ;
		
		lcd_puts_Pleft( 6*FH, XPSTR("COM2 Baudrate") );
    attr = 0 ;
		b = g_model.com2Baudrate ;
		if ( (sub == subN) )
		{
			attr = INVERS ;
	  	CHECK_INCDEC_H_MODELVAR_0(event, g_model.com2Baudrate, 4 ) ;
		}
		lcd_putsAttIdx(15*FW, 6*FH, XPSTR("\006  9600 19200 38400 57600115200"), g_model.com2Baudrate, attr ) ;
#ifdef PCBSKY
		if ( b != g_model.com2Baudrate )
		{
			if( g_model.com2Function == COM2_FUNC_BTDIRECT )
			{
				UART_Configure( g_model.com2Baudrate, Master_frequency ) ;
			}
		}
#endif	// PCBSKY
		subN += 1 ;

#ifdef PCBSKY
		lcd_puts_Pleft( 7*FH, XPSTR("BT as Trainer") );
		if ( (sub == subN) )
		{
			CHECK_INCDEC_H_MODELVAR_0(event, g_model.BTfunction, 1 ) ;
		}
  	menu_lcd_onoff( PARAM_OFS, 7*FH, g_model.BTfunction, sub==subN ) ;
#endif
	}
	else
	{
		uint8_t subN = T2COUNT_ITEMS - 7 ;
		// Vario
   	for( uint8_t j=0 ; j<7 ; j += 1 )
		{
			uint8_t b ;
      uint8_t attr = (sub==subN) ? INVERS : 0 ;
			uint8_t y = (1+j)*FH ;

			switch ( j )
			{
				case 0 :
					lcd_puts_Pleft( y, PSTR(STR_VARIO_SRC) ) ;
					lcd_putsAttIdx( 15*FW, y, PSTR(STR_VSPD_A2), g_model.varioData.varioSource, attr ) ;
   		  	if(attr)
					{
						CHECK_INCDEC_H_MODELVAR_0(event, g_model.varioData.varioSource, 2+NUM_SCALERS ) ;
   		  	}
				break ;
				
				case 1 :
					lcd_puts_Pleft( y, PSTR(STR_2SWITCH) ) ;
					g_model.varioData.swtch = edit_dr_switch( 15*FW, y, g_model.varioData.swtch, attr, attr ? EDIT_DR_SWITCH_EDIT : 0, event ) ;
				break ;

				case 2 :
					lcd_puts_Pleft( y, PSTR(STR_2SENSITIVITY) ) ;
 					lcd_outdezAtt( 17*FW, y, g_model.varioData.param, attr) ;
   			  if(attr)
					{
						CHECK_INCDEC_H_MODELVAR_0(event, g_model.varioData.param, 50 ) ;
	   		  }
				break ;

				case 3 :
	        b = 1-g_model.varioData.sinkTones ;
					g_model.varioData.sinkTones = 1-onoffMenuItem_m( event, b, y, PSTR(STR_SINK_TONES), attr ) ;
				break ;

				case 4 :
					lcd_puts_Pleft( y, PSTR(STR_LOG_SWITCH) ) ;
					g_model.logSwitch = edit_dr_switch( 15*FW, y, g_model.logSwitch, attr, attr ? EDIT_DR_SWITCH_EDIT : 0, event ) ;
				break ;

				case 5 :
					lcd_puts_Pleft( y, PSTR(STR_LOG_RATE) ) ;
					lcd_putsAttIdx( 15*FW, y, XPSTR("\0041.0s2.0s"), g_model.logRate, attr ) ;
   			  if(attr)
					{
						CHECK_INCDEC_H_MODELVAR_0(event, g_model.logRate, 1 ) ;
	   		  }
				break ;

				case 6 :
					lcd_puts_Pleft( y, XPSTR("Current Source" ) ) ;
					lcd_putsAttIdx( 15*FW, y, XPSTR("\004----A1  A2  FASVSC1 SC2 SC3 SC4 SC5 SC6 SC7 SC8 "), g_model.currentSource, attr ) ;
   			  if(attr)
					{
						CHECK_INCDEC_H_MODELVAR_0(event, g_model.currentSource, 3 + NUM_SCALERS ) ;
	   		  }
				break ;

			}

			subN += 1 ;
		}
	}
}

#endif

#if GVARS

void menuScaleOne(uint8_t event)
{
  SUBMENU_NOTITLE( 9, { 0, 3, 0 /*, 0*/} ) ;
  lcd_puts_Pleft( 0, XPSTR("SC  =") ) ;
	uint8_t index = s_currIdx ;
  int8_t sub = mstate2.m_posVert;
  lcd_putc( 2*FW, 0, index+'1' ) ;
	evalOffset(sub, 6);
	putsTelemetryChannel( 8*FW, 0, index+TEL_ITEM_SC1, 0, 0, TELEM_UNIT ) ;

	if ( event == EVT_ENTRY )
	{
		RotaryState = ROTARY_MENU_UD ;
	}
  
	for (uint8_t k = 0 ; k < 7 ; k += 1 )
	{
    uint8_t y = (k+1) * FH ;
    uint8_t i = k + s_pgOfs;
		uint8_t attr = (sub==i ? InverseBlink : 0);
		ScaleData *pscaler ;
		pscaler = &g_model.Scalers[index] ;
		switch(i)
		{
      case 0 :	// Source
				lcd_puts_Pleft( y, XPSTR("Source") ) ;
				putsChnRaw( 11*FW, y, pscaler->source, attr ) ;
  			if( attr ) CHECK_INCDEC_H_MODELVAR_0( event, pscaler->source, NUM_SKYXCHNRAW+NUM_TELEM_ITEMS ) ;
			break ;
			case 1 :	// name
				editName( 11*FW-2, mstate2.m_posHorz, y, (uint8_t *)pscaler->name, sizeof(pscaler->name), attr ? EE_MODEL : 0, event ) ;
//		    lcd_puts_Pleft( y, PSTR(STR_NAME)) ;
//    		lcd_putsnAtt( 11*FW, y, (const char *)pscaler->name, 4, 0 ) ;
//  			if( attr )
//				{
//					uint8_t subSub = mstate2.m_posHorz;
//					lcd_rect( 11*FW-2, y-1, 4*FW+4, 9 ) ;
//					lcd_char_inverse( (11+subSub)*FW, y, 1*FW, s_editMode ) ;
//	    		if(s_editMode)
//					{
//	        	char v = pscaler->name[subSub] ;
//						if ( v )
//						{
//	  	      	v = char2idx(v) ;
//						}
//	  	      CHECK_INCDEC_H_MODELVAR_0( event,v ,NUMCHARS-1);
//  	  	    v = idx2char(v);
//						if ( pscaler->name[subSub] != v )
//						{
//							pscaler->name[subSub] = v ;
////    					eeDirty( EE_MODEL ) ;				// Do here or the last change is not stored in ModelNames[]
//						}
//					}
//				} 
			break ;
      case 2 :	// offset
				lcd_puts_Pleft( y, PSTR(STR_OFFSET) ) ;
				lcd_outdezAtt( 13*FW, y, pscaler->offset, attr) ;
				if ( attr )
				{
//					if ( s_editMode )
//					{
//						int32_t v ;
//						int32_t w ;
//						v = w = pscaler->offset/100 ;
//						v = checkIncDec16(event, v, -320, 320, EE_MODEL ) ;
//						if ( v != w )
//						{
//							v -= w ;
//							v *= 100 ;
//							pscaler->offset += v ;
//						}
//					}
//					else
//					{
						pscaler->offset = checkIncDec16( event, pscaler->offset, -32000, 32000, EE_MODEL ) ;
//					}
				}

			break ;
      case 3 :	// mult
				lcd_puts_Pleft( y, XPSTR("Multiplier") ) ;
				lcd_outdezAtt( 13*FW, y, pscaler->mult+1, attr) ;
		    if (attr) pscaler->mult = checkIncDec16( event, pscaler->mult, 0, 255, EE_MODEL);
			break ;
      case 4 :	// div
				lcd_puts_Pleft( y, XPSTR("Divisor") ) ;
				lcd_outdezAtt( 13*FW, y, pscaler->div+1, attr) ;
		    if (attr) pscaler->div = checkIncDec16( event, pscaler->div, 0, 255, EE_MODEL);
			break ;
      case 5 :	// unit
				lcd_puts_Pleft( y, XPSTR("Unit") ) ;
				lcd_putsAttIdx( 11*FW, y, XPSTR(UnitsString), pscaler->unit, attr ) ;
  			if( attr ) CHECK_INCDEC_H_MODELVAR_0( event, pscaler->unit, 8 ) ;
			break ;
      case 6 :	// sign
				lcd_puts_Pleft( y, XPSTR("Sign") ) ;
  			lcd_putcAtt( 11*FW, y, pscaler->neg ? '-' : '+', attr ) ;
  			if( attr ) CHECK_INCDEC_H_MODELVAR_0( event, pscaler->neg, 1 ) ;
			break ;
      case 7 :	// precision
				lcd_puts_Pleft( y, XPSTR("Decimals") ) ;
				lcd_outdezAtt( 13*FW, y, pscaler->precision, attr) ;
  			if( attr ) CHECK_INCDEC_H_MODELVAR_0( event, pscaler->precision, 2 ) ;
			break ;
      case 8 :	// offsetLast
				lcd_puts_Pleft( y, XPSTR("Offset At") ) ;
				lcd_putsAttIdx( 11*FW, y, XPSTR("\005FirstLast "), pscaler->offsetLast, attr ) ;
  			if( attr ) CHECK_INCDEC_H_MODELVAR_0( event, pscaler->offsetLast, 1 ) ;
			break ;
		}
	}

}


void menuProcGlobals(uint8_t event)
{
	TITLE(PSTR(STR_GLOBAL_VARS));
	static MState2 mstate2;
	static const uint8_t mstate_tab[] = {0,2,2,2,2,2,2,2,0} ;
	
	if (SubMenuFromIndex)
	{
		event = mstate2.check(event,0,NULL,0,mstate_tab,DIM(mstate_tab)-1, MAX_GVARS + 1 + NUM_SCALERS-1 ) ;
	}
	else
	{
		event = mstate2.check(event,e_Globals,menuTabModel,DIM(menuTabModel),mstate_tab,DIM(mstate_tab)-1, MAX_GVARS + 1 + NUM_SCALERS-1 ) ;
//	  MENU(PSTR(STR_GLOBAL_VARS), menuTabModel, e_Globals, MAX_GVARS + 1 + NUM_SCALERS, {0,2,2,2,2,2,2,2,0} ) ;
	}

	uint8_t subN = mstate2.m_posVert ;
	uint8_t subSub = mstate2.m_posHorz;
	
//	lcd_outhex4( 0, 7*FH, ( (convert_mode_helper(g_model.gvars[1].gvsource) - 1) << 8 )+ (g_model.gvars[1].gvsource - 1) ) ;
  switch (event)
	{
    case EVT_KEY_FIRST(KEY_MENU) :
    case EVT_KEY_BREAK(BTN_RE) :
			if ( subN >= 8 ) //&& sub <= MAX_MODES )
			{
        s_currIdx = subN - 8 ;
//				RotaryState = ROTARY_MENU_UD ;
        killEvents(event);
        pushMenu(menuScaleOne) ;
    	}
		break;
  }

 if ( subN < 8 )
 {
	for (uint8_t i=0; i<MAX_GVARS; i++ )
	{
		uint8_t y = (i+1)*FH ;
    lcd_puts_Pleft(y, PSTR(STR_GV));
		lcd_putc( 2*FW, y, i+'1') ;
		for(uint8_t j=0; j<3;j++)
		{
      uint8_t attr = ((subN==i+1 && subSub==j) ? InverseBlink : 0);
			uint8_t active = (attr && (s_editMode) ) ;
			GvarData *pgvar ;
			pgvar = &g_model.gvars[i] ;
      switch(j)
			{
				case 0 :
        	putsDrSwitches( 6*FW, y, g_model.gvswitch[i] ,attr );
	  			if(active) CHECK_INCDEC_MODELSWITCH( event, g_model.gvswitch[i], -MaxSwitchIndex, MaxSwitchIndex) ;
				break ;

				case 1 :
					lcd_putsAttIdx( 12*FW, y, PSTR(STR_GV_SOURCE), pgvar->gvsource, attr ) ;
					// STR_GV_SOURCE
#ifdef PCBSKY
	  			if(active) CHECK_INCDEC_H_MODELVAR_0( event, pgvar->gvsource, 68 ) ;
#endif
#ifdef PCBX9D
	  			if(active) CHECK_INCDEC_H_MODELVAR_0( event, pgvar->gvsource, 69 ) ;
#endif
				break ;

				case 2 :
					lcd_outdezAtt( 19*FW, y, pgvar->gvar, attr) ;
	  			if(active) CHECK_INCDEC_H_MODELVAR( event, pgvar->gvar, -125, 125 ) ;
				break ;
			}
		}
	}
 }
 else
 {
	uint8_t sub = subN - 8 ;
  uint8_t t_pgOfs = (sub == 7 ) ? 1 : 0 ;
  uint8_t y ;
  uint8_t k ;
	
	for (uint8_t i=0; i<7; i++ )
	{
    y=(i+1)*FH;
    k=i+t_pgOfs;
  	lcd_puts_Pleft( y, XPSTR("SC\011+\015*\22/") ) ;
  	lcd_putc( 2*FW, y, k+'1' ) ;
		putsChnRaw( 4*FW, y, g_model.Scalers[k].source, 0 ) ;
		lcd_outdezAtt( 12*FW+3, y, g_model.Scalers[k].offset, 0) ;
		lcd_outdezAtt( 16*FW, y, g_model.Scalers[k].mult+1, 0) ;
		lcd_outdezAtt( 21*FW, y, g_model.Scalers[k].div+1, 0) ;
	}
	lcd_char_inverse( 0, (sub-t_pgOfs+1)*FH, 126, 0 ) ;
 }
}

#endif

#ifndef NO_TEMPLATES
void menuProcTemplates(uint8_t event)  //Issue 73
{
    SIMPLE_SUBMENU( PSTR(STR_TEMPLATES), NUM_TEMPLATES+1) ;
//    SIMPLE_MENU(PSTR(STR_TEMPLATES), menuTabModel, e_Templates, NUM_TEMPLATES+2);

    uint8_t t_pgOfs ;
    uint8_t y = 0;
    uint8_t k = 0;
    int8_t  sub    = mstate2.m_posVert - 1;

    t_pgOfs = evalOffset(sub, 6);

    switch(event)
    {
    case EVT_KEY_LONG(KEY_MENU):
        killEvents(event);
        //apply mixes or delete
        s_noHi = NO_HI_LEN;
        if(sub==NUM_TEMPLATES)
            clearMixes();
        else if((sub>=0) && (sub<(int8_t)NUM_TEMPLATES))
            applyTemplate(sub);
        audioDefevent(AU_WARNING2);
        break;
    }

    y=1*FH;
    for(uint8_t i=0; i<7; i++){
        k=i+t_pgOfs;
        if(k==NUM_TEMPLATES) break;

        //write mix names here
        lcd_outdezNAtt(3*FW, y, k+1, (sub==k ? INVERS : 0) + LEADING0,2);
        lcd_putsAtt(  4*FW, y, PSTR(n_Templates[k]), (s_noHi ? 0 : (sub==k ? INVERS  : 0)));
        y+=FH;
    }

    if(y>7*FH) return;
    uint8_t attr = s_noHi ? 0 : ((sub==NUM_TEMPLATES) ? INVERS : 0);
    lcd_putsAtt(  1*FW,y,PSTR(STR_CLEAR_MIXES),attr);
    y+=FH;

}
#endif

//FunctionData Function[1] ;


void menuProcSafetySwitches(uint8_t event)
{
	
	TITLE(PSTR(STR_SAFETY_SW));
	static MState2 mstate2;
	static const uint8_t mstate_tab[] = {0, 0, 2/*repeated*/} ;
	if (SubMenuFromIndex)
	{
		event = mstate2.check(event,0,NULL,0,mstate_tab,DIM(mstate_tab)-1,NUM_SKYCHNOUT+1+1+NUM_VOICE-1) ;
	}
	else
	{
		event = mstate2.check(event,e_SafetySwitches,menuTabModel,DIM(menuTabModel),mstate_tab,DIM(mstate_tab)-1,NUM_SKYCHNOUT+1+1+NUM_VOICE-1) ;
//	MENU(PSTR(STR_SAFETY_SW), menuTabModel, e_SafetySwitches, NUM_SKYCHNOUT+1+1+NUM_VOICE, {0, 0, 2/*repeated*/});
	}

	uint8_t y = 0;
	uint8_t k = 0;
	int8_t  sub    = mstate2.m_posVert - 1;
	uint8_t subSub = mstate2.m_posHorz;
  uint8_t t_pgOfs ;

	t_pgOfs = evalOffset(sub, 6);

//  lcd_puts_P( 0*FW, 1*FH,PSTR("ch    sw     val"));
	for(uint8_t i=0; i<7; i++)
	{
    y=(i+1)*FH;
    k=i+t_pgOfs;
		if ( k == 0 )
		{
  	  lcd_puts_Pleft( y, PSTR(STR_NUM_VOICE_SW) ) ;
  	  uint8_t attr = ((sub==k) ? InverseBlink : 0);
			lcd_outdezAtt(  18*FW, y,g_model.numVoice+NUM_VOICE, attr);
 		  if(sub==k)
			{
  	    CHECK_INCDEC_H_MODELVAR_0( event, g_model.numVoice, NUM_SKYCHNOUT ) ;
  	  }
		}
  	else // if(k<NUM_SKYCHNOUT+1+NUM_VOICE)
		{
			uint8_t numSafety = NUM_SKYCHNOUT - g_model.numVoice ;
    	SKYSafetySwData *sd = &g_model.safetySw[k-1];
    	if ( k-1 >= NUM_CHNOUT )
			{
				sd = (SKYSafetySwData*)&g_model.voiceSwitches[k-1-NUM_SKYCHNOUT];
			}
    	putsChn(0,y,k,0);
			if ( k <= numSafety )
			{
  	  	for(uint8_t j=0; j<3;j++)
				{
    		  uint8_t attr = ((sub==k && subSub==j) ? InverseBlink : 0);
					uint8_t active = (attr && (s_editMode || P1values.p1valdiff)) ;
  	  	  if (j == 0)
					{
						lcd_putsAttIdx( 5*FW, y, XPSTR("\001SAVX"), sd->opt.ss.mode, attr ) ;
	      	  if(active)
						{
	  	        CHECK_INCDEC_H_MODELVAR_0( event, sd->opt.ss.mode, 3 ) ;
  	  	    }
					}
	    	  else if (j == 1)
  	  	  {
						int8_t max = MaxSwitchIndex ;
						if ( sd->opt.ss.mode == 2 )
						{
							max = MaxSwitchIndex+3 ;
						}	 
						if ( ( sd->opt.ss.swtch > MAX_SKYDRSWITCH ) && ( sd->opt.ss.swtch <= MAX_SKYDRSWITCH + 3 ) )
						{
							lcd_putsAttIdx( 7*FW, y, PSTR(STR_V_OPT1), sd->opt.ss.swtch-MAX_SKYDRSWITCH-1, attr ) ;
						}
						else
						{
         	  	putsDrSwitches(7*FW, y, sd->opt.ss.swtch, attr);
						}
	    	    if(active)
						{
              CHECK_INCDEC_MODELSWITCH( event, sd->opt.ss.swtch, -MaxSwitchIndex, max ) ;
    		    }
					}
					else
					{
						int8_t min, max ;
						if ( sd->opt.ss.mode == 1 )
						{
							min = 0 ;
							max = 15 ;
							sd->opt.ss.val = limit( min, sd->opt.ss.val, max) ;
							lcd_putsAttIdx(15*FW, y, PSTR(STR_SOUNDS), sd->opt.ss.val,attr);
						}
						else if ( sd->opt.ss.mode == 2 )
						{
							if ( sd->opt.ss.swtch > MAX_SKYDRSWITCH )
							{
								min = 0 ;
								max = NUM_TELEM_ITEMS-1 ;
								sd->opt.ss.val = limit( min, sd->opt.ss.val, max) ;
  							putsAttIdxTelemItems( 16*FW, y, sd->opt.ss.val+1, attr ) ;
							}
							else
							{
								min = -128 ;
								max = 111 ;
								sd->opt.ss.val = limit( min, sd->opt.ss.val, max) ;
        				lcd_outdezAtt( 16*FW, y, sd->opt.ss.val+128, attr);
							}
						}
						else
						{
							min = -125 ;
							max = 125 ;
        			lcd_outdezAtt(  16*FW, y, sd->opt.ss.val, attr);
						}
  	  	    if(active)
						{
		          CHECK_INCDEC_H_MODELVAR( event, sd->opt.ss.val, min,max);
    	  	  }
					}
    	  }
    	}
			else
			{
	  	  lcd_puts_Pleft( y, PSTR(STR_VS) ) ;
    		for(uint8_t j=0; j<3;j++)
				{
    	    uint8_t attr = ((sub==k && subSub==j) ? InverseBlink : 0);
					uint8_t active = (attr && (s_editMode || P1values.p1valdiff)) ;
    		  if (j == 0)
					{
    		    if(active)
						{
    			    CHECK_INCDEC_MODELSWITCH( event, sd->opt.vs.vswtch, -MaxSwitchIndex+1, MaxSwitchIndex-1 ) ;
    		    }
  			    putsDrSwitches(5*FW, y, sd->opt.vs.vswtch, attr);
					}
    		  else if (j == 1)
    		  {
    		    if(active)
						{
    			    CHECK_INCDEC_H_MODELVAR_0( event, sd->opt.vs.vmode, 6 ) ;
    		    }
						lcd_putsAttIdx( 10*FW, y, PSTR(STR_VOICE_OPT), sd->opt.vs.vmode, attr ) ;
					}
					else
					{
						uint8_t max ;
						if ( sd->opt.vs.vmode > 5 )
						{
							max = NUM_TELEM_ITEMS-1 ;
							sd->opt.vs.vval = limit( (uint8_t)0, sd->opt.vs.vval, max) ;
							putsAttIdxTelemItems( 16*FW, y, sd->opt.vs.vval+1, attr ) ;
						}
						else
						{
							// Allow 251-255 to represent GVAR3-GVAR7
							max = 255 ;
							if ( sd->opt.vs.vval <= 250 )
							{
	  						lcd_outdezAtt( 19*FW, y, sd->opt.vs.vval, attr) ;
							}	
							else
							{
								dispGvar( 16*FW, y, sd->opt.vs.vval-248, attr ) ;
							}
						}
    			  if(active)
						{
    		      sd->opt.vs.vval = checkIncDec16(event, sd->opt.vs.vval, 0, max, EE_MODEL);
							if ( event==EVT_KEY_LONG(KEY_MENU))
							{
								if ( sd->opt.vs.vval <= 250 )
								{
									putVoiceQueue( sd->opt.vs.vval ) ;
								}
      					killEvents(event);
								s_editMode = 0 ;
							}
    			  }
					}	 
			  }
			}
		}
//		else
//		{
//			// Function(s)
//			lcd_puts_Pleft( y, PSTR("F1") ) ;
//  	 	for( uint8_t j=0 ; j<3 ; j += 1 )
//			{
//  	    uint8_t attr = ((sub==k && subSub==j) ? (s_editMode ? BLINK : INVERS) : 0) ;
//				uint8_t active = (attr && s_editMode) ;
//  	 		if (j == 0)
//				{
//					putsDrSwitches( 5*FW, y, Function[0].swtch , attr ) ;
//  	 		  if(active)
//					{
//						Function[0].swtch = checkIncDec( event, Function[0].swtch, -MAX_DRSWITCH+1, MAX_DRSWITCH, 0 ) ;
//  	 		  }
//				}
//  	 		else if (j == 1)
//  	 		{
//					lcd_putsAttIdx( 10*FW, y, PSTR("\005-----variovarA2"), Function[0].func, attr ) ;
//  	 		  if(active)
//					{
//						Function[0].func = checkIncDec( event, Function[0].func, 0, 2, 0 ) ;
//  	 		  }
//				}
//				else
//				{
// 					lcd_outdezAtt( 17*FW, y, Function[0].param[0], attr) ;
//  	 		  if(active)
//					{
//						Function[0].param[0] = (uint8_t)checkIncDec( event, (int8_t)Function[0].param[0], 0, 50, 0 ) ;
//  	 		  }
//				}	
//			}
//		}
	}
}

void menuProcSwitches(uint8_t event)  //Issue 78
{
	TITLE(PSTR(STR_CUST_SWITCH));
	static MState2 mstate2;
	static const uint8_t mstate_tab[] = {0, 3} ;
	if (SubMenuFromIndex)
	{
		event = mstate2.check(event,0,NULL,0,mstate_tab,DIM(mstate_tab)-1,NUM_SKYCSW+1-1) ;
	}
	else
	{
		event = mstate2.check(event,e_Switches,menuTabModel,DIM(menuTabModel),mstate_tab,DIM(mstate_tab)-1,NUM_SKYCSW+1-1) ;
//    MENU(PSTR(STR_CUST_SWITCH), menuTabModel, e_Switches, NUM_SKYCSW+1, {0, 3/*repeated...*/});
	}
	

uint8_t y = 0;
uint8_t k = 0;
int8_t  sub    = mstate2.m_posVert - 1;
uint8_t subSub = mstate2.m_posHorz;
    uint8_t t_pgOfs ;

	t_pgOfs = evalOffset(sub, 6);

//  lcd_puts_P( 4*FW, 1*FH,PSTR("Function V1  V2"));
for(uint8_t i=0; i<7; i++){
    y=(i+1)*FH;
    k=i+t_pgOfs;
//    if(k==NUM_SKYCSW) break;
    uint8_t attr ;
    SKYCSwData &cs = g_model.customSw[k];

		
    attr = (getSwitch(CSW_INDEX+k+1, 0) ) ? INVERS : 0 ;
    //write SW names here
//    lcd_putsnAtt( 0*FW , y, PSTR("SW"),2,attr) ;
    lcd_puts_Pleft( y, XPSTR("L") ) ;
//    lcd_putcAtt(  FW-1 , y, 'W', attr) ;
    lcd_putcAtt(  1*FW-1 , y, k + (k>8 ? 'A'-9: '1'), attr) ;
    
		attr = (sub==k ? InverseBlink  : 0);
		lcd_putsAttIdx( 2*FW+1, y, PSTR(CSWITCH_STR),cs.func,subSub==0 ? attr : 0);

    uint8_t cstate = CS_STATE(cs.func);

    if(cstate == CS_VOFS)
    {
			putsChnRaw(    10*FW-6, y, cs.v1  ,subSub==1 ? attr : 0);
//#ifdef FRSKY
      if (cs.v1 > CHOUT_BASE+NUM_SKYCHNOUT)
 			{
				int16_t value = convertTelemConstant( cs.v1-CHOUT_BASE-NUM_SKYCHNOUT-1, cs.v2 ) ;
				putsTelemetryChannel( 18*FW-8, y, cs.v1-CHOUT_BASE-NUM_SKYCHNOUT-1, value, subSub==2 ? attr : 0, TELEM_NOTIME_UNIT | TELEM_UNIT| TELEM_CONSTANT);
			}
      else
//#endif
        lcd_outdezAtt( 18*FW-9, y, cs.v2  ,subSub==2 ? attr : 0);
    }
    else if(cstate == CS_VBOOL)
    {
        putsDrSwitches(10*FW-6, y, cs.v1  ,subSub==1 ? attr : 0);
        putsDrSwitches(14*FW-7, y, cs.v2  ,subSub==2 ? attr : 0);
    }
    else if(cstate == CS_VCOMP)
    {
        putsChnRaw(    10*FW-6, y, cs.v1  ,subSub==1 ? attr : 0);
        putsChnRaw(    14*FW-4, y, cs.v2  ,subSub==2 ? attr : 0);
    }
		else if(cstate == CS_TIMER)
		{
	    lcd_puts_Pleft( y, PSTR(STR_15_ON) ) ;
			int8_t x ;
			uint8_t att = 0 ;
			x = cs.v1 ;
			if ( x < 0 )
			{
				x = -x-1 ;
				att = PREC1 ;
			}
      lcd_outdezAtt( 13*FW-5, y, x+1  ,att | (subSub==1 ? attr : 0) ) ;
			att = 0 ;
			x = cs.v2 ;
			if ( x < 0 )
			{
				x = -x-1 ;
				att = PREC1 ;
			}
      lcd_outdezAtt( 18*FW-3, y, x+1 , att | (subSub==2 ? attr : 0 ) ) ;
		}
		else// cstate == CS_TMONO
		{
      putsDrSwitches(10*FW-6, y, cs.v1  ,subSub==1 ? attr : 0);
			uint8_t att = 0 ;
			int8_t x ;
			x = cs.v2 ;
			if ( x < 0 )
			{
				x = -x-1 ;
				att = PREC1 ;
			}
      lcd_outdezAtt( 17*FW-2, y, x+1 , att | (subSub==2 ? attr : 0 ) ) ;
		}
//    lcd_putc( 19*FW+3, y, cs.andsw ? 'S' : '-') ;
		{
			int8_t as ;
			as = cs.andsw ;
			if ( as < 0 )
			{
				as = -as ;
		  	lcd_putcAtt( 18*FW-3, y, '!',(subSub==3 ? attr : 0)) ;
			}
#ifdef PCBSKY
			if ( ( as > 8 ) && ( as <= 9+NUM_SKYCSW+1 ) )
			{
				as += 1 ;				
			}
			if ( as == 9+NUM_SKYCSW+1 )
			{
				as = 9 ;			// Tag TRN on the end, keep EEPROM values
			}
#endif
			putsDrSwitches( 18*FW-3, y, as,(subSub==3 ? attr : 0)) ;
		}


    if((s_editMode || P1values.p1valdiff) && attr)
        switch (subSub) {
        case 0:
            CHECK_INCDEC_H_MODELVAR_0( event, cs.func, CS_MAXF);
            if(cstate != CS_STATE(cs.func))
            {
                cs.v1  = 0;
                cs.v2 = 0;
            }
            break;
        case 1:
            switch (cstate) {
            case (CS_VCOMP):
            case (CS_VOFS):
#ifdef FRSKY
                CHECK_INCDEC_H_MODELVAR_0( event, cs.v1, NUM_SKYXCHNRAW+NUM_TELEM_ITEMS);
#else
                CHECK_INCDEC_H_MODELVAR_0( event, cs.v1, NUM_SKYXCHNRAW);
#endif
                break;
            case (CS_VBOOL):
						case CS_TMONO :
                CHECK_INCDEC_MODELSWITCH( event, cs.v1, -MaxSwitchIndex,MaxSwitchIndex);
                break;
//            case (CS_VCOMP):
//#ifdef FRSKY
//                CHECK_INCDEC_H_MODELVAR_0( event, cs.v1, NUM_SKYXCHNRAW+NUM_TELEM_ITEMS);
//#else
//                CHECK_INCDEC_H_MODELVAR_0( event, cs.v1, NUM_SKYXCHNRAW);
//#endif
								break;
            case (CS_TIMER):
                CHECK_INCDEC_H_MODELVAR( event, cs.v1, -50,99);
                break;
						
						default:
                break;
            }
            break;
        case 2:
            switch (cstate) {
            case (CS_VOFS):
						case CS_TMONO :
              CHECK_INCDEC_H_MODELVAR( event, cs.v2, -125,125) ;
            break ;
            case (CS_VBOOL):
                CHECK_INCDEC_MODELSWITCH( event, cs.v2, -MaxSwitchIndex,MaxSwitchIndex);
                break;
            case (CS_VCOMP):
                CHECK_INCDEC_H_MODELVAR_0( event, cs.v2, NUM_SKYXCHNRAW+NUM_TELEM_ITEMS);
                break;
            case (CS_TIMER):
                CHECK_INCDEC_H_MODELVAR( event, cs.v2, -50,99);
                break;

            default:
                break;
            }
            break;
        case 3:
          CHECK_INCDEC_MODELSWITCH( event, cs.andsw, -MaxSwitchIndex+1,MaxSwitchIndex-1);
				break;
        }
}
}
//#endif

static int8_t s_currMixIdx;
static uint8_t s_moveMixIdx;
static int8_t s_currDestCh;
static bool   s_currMixInsMode;


void deleteMix(uint8_t idx)
{
    memmove(&g_model.mixData[idx],&g_model.mixData[idx+1],
            (MAX_SKYMIXERS-(idx+1))*sizeof(SKYMixData));
    memset(&g_model.mixData[MAX_SKYMIXERS-1],0,sizeof(SKYMixData));
    STORE_MODELVARS;
//    eeWaitComplete() ;
}

void insertMix(uint8_t idx, uint8_t copy)
{
    SKYMixData *md = &g_model.mixData[idx] ;

    memmove(md+1,md, (MAX_SKYMIXERS-(idx+1))*sizeof(SKYMixData) );
		if ( copy )
		{
	    memmove( md, md-1, sizeof(SKYMixData) ) ;
		}
		else
		{
    	memset(md,0,sizeof(SKYMixData));
    	md->destCh      = s_currDestCh; //-s_mixTab[sub];
    	md->srcRaw      = s_currDestCh; //1;   //
	    md->weight      = 100;
			md->lateOffset  = 1 ;
		}
		s_currMixIdx = idx ;
//    eeWaitComplete() ;
}

void put_curve( uint8_t x, uint8_t y, int8_t idx, uint8_t attr )
{
	if ( idx < 0 )
	{
    lcd_putcAtt( x-FW, y, '!', attr ) ;
		idx = -idx + 6 ;
	}
	lcd_putsAttIdx( x, y,get_curve_string(),idx,attr);
}

//PACK(typedef struct t_voiceAlarm
//{
//  uint8_t source ;
//	uint8_t func;
//  int8_t  swtch ;
//	uint8_t rate ;
//	uint8_t fnameType:2 ;
//	uint8_t spare1:6 ;
//  int16_t offset; 		//offset
//	union
//	{
//		int16_t vfile ;
//		uint8_t name[6] ;
//	} file ;
//}) VoiceAlarmData ;

void copyFileName( char *dest, char *source )
{
	uint32_t i ;
	for ( i = 0 ; i < 8 ; i += 1 )
	{
		char c ;
		c = source[i] ;
		if ( c == '.' )
		{
			break ;
		}
		dest[i] = c ;
	}
	while ( i < 8 )
	{
		dest[i] = '\0' ;
		i += 1 ;
	}
}


//VoiceAlarmData vad[2] ;

// FUnctions need to include ON, OFF and BOTH possibly
void menuProcVoiceOne(uint8_t event)
{
	static uint8_t columns = 0 ;
	TITLE(XPSTR("Voice Alarm"));
	lcd_outdezAtt( 13*FW, 0, s_currIdx+1, 0 ) ;
	static MState2 mstate2;
	VoiceAlarmData *pvad = &g_model.vad[s_currIdx] ;
	uint32_t rows = pvad->fnameType ? 11-1: 10-1 ;
	event = mstate2.check(event,0,NULL,0,&columns,0, rows ) ;
	columns = 0 ;
	if ( event == EVT_ENTRY_UP )
	{ // From menuProcSelectUvoiceFile
		if ( FileSelectResult == 1 )
		{
			copyFileName( (char *)pvad->file.name, SelectedVoiceFileName ) ;
	    eeDirty(EE_MODEL) ;		// Save it
		}
	}
	
	int8_t sub = mstate2.m_posVert ;

	if ( event == EVT_ENTRY )
	{
		RotaryState = ROTARY_MENU_UD ;
	}
	
//  t_pgOfs = evalOffset(sub, 6) ;
  for(uint8_t i=0; i<=rows; i++)
  {
		uint8_t y = (i+1)*FH;
    uint8_t attr = sub==i ? InverseBlink : 0 ;
//  	uint8_t b ;

		if ( sub < 6 )
		{
			displayNext() ;

	    switch(i)
			{
    	  case 0 :	// source
  	  		lcd_puts_Pleft( FH, XPSTR("Source") ) ;
					putsChnRaw( 16*FW, y, pvad->source, attr ) ;
					if ( attr )
					{
	      		pvad->source = checkIncDec16(event, pvad->source, 0, NUM_SKYXCHNRAW+NUM_TELEM_ITEMS, EE_MODEL);
					}
					if ( pvad->source )
					{
						int16_t value ;
						value = getValue( pvad->source - 1 ) ;
  	  			lcd_puts_Pleft( FH, XPSTR("\007(\015)") ) ;
      			if (pvad->source > CHOUT_BASE+NUM_SKYCHNOUT)
 						{
							putsTelemetryChannel( 12*FW, FH, pvad->source-CHOUT_BASE-NUM_SKYCHNOUT-1, value, 0, TELEM_NOTIME_UNIT | TELEM_UNIT ) ;
						}
						else
						{
							lcd_outdezAtt( 12*FW, FH, value, 0 ) ;
						}
					}
				break ;

				case 1 :	// func;
  	  		lcd_puts_Pleft( y, XPSTR("Function") ) ;
					lcd_putsAttIdx( 13*FW, y, XPSTR("\007-------v>val  v<val  |v|>val|v|<valv\140=valv=val  "), pvad->func, attr ) ;	// v1>v2  v1<v2  
	    		if(attr)
					{
      	    CHECK_INCDEC_H_MODELVAR_0( event, pvad->func, 6 ) ;
					}	
				break ;

				case 2 :
				{	
  	  		lcd_puts_Pleft( y, XPSTR("Value") ) ;
		      if (pvad->source > CHOUT_BASE+NUM_SKYCHNOUT)
		 			{
						putsTelemetryChannel( 20*FW, y, pvad->source-CHOUT_BASE-NUM_SKYCHNOUT-1, pvad->offset, attr, TELEM_NOTIME_UNIT | TELEM_UNIT | TELEM_CONSTANT ) ;
					}
					else
					{
						lcd_outdezAtt( FW*20, y, pvad->offset, attr ) ;
					}
					if ( attr )
					{
//						if ( s_editMode )
//						{
//							int32_t v ;
//							int32_t w ;
//							v = w = pvad->offset/100 ;
//							v = checkIncDec16(event, v, -320, 320, EE_MODEL ) ;
//							if ( v != w )
//							{
//								v -= w ;
//								v *= 100 ;
//								pvad->offset += v ;
//							}
//						}
//						else
//						{
							pvad->offset = checkIncDec16(event, pvad->offset, -32000, 32000, EE_MODEL ) ;
//						}
					}
				}
				break ;
			
				case 3 :	 // swtch ;
  	  		lcd_puts_Pleft( y, XPSTR("Switch") ) ;
   	  		putsDrSwitches(16*FW, y, pvad->swtch, attr);
	    		if(attr)
					{
      	    CHECK_INCDEC_MODELSWITCH( event, pvad->swtch, -MaxSwitchIndex, MaxSwitchIndex ) ;
    			}
				break ;

				case 4 :	 // rate ;
  	  		lcd_puts_Pleft( y, XPSTR("Rate") ) ;
					if ( pvad->rate < 3 )
					{
						lcd_putsAttIdx( 16*FW, y, XPSTR("\004  ON OFFBOTH"),pvad->rate,attr);
					}
					else
					{
      	    lcd_outdezAtt(FW*18,y,pvad->rate-2,attr ) ;
					}
	    		if(attr)
					{
      	    CHECK_INCDEC_H_MODELVAR_0( event, pvad->rate, 32 ) ;
					}	
				break ;

				case 5 :	 // haptic ;
  	  		lcd_puts_Pleft( y, XPSTR("Haptic") ) ;
					lcd_putsAttIdx( 13*FW, y, XPSTR("\007-------Haptic1Haptic2Haptic3"), pvad->haptic, attr ) ;
	    		if(attr)
					{
      	    CHECK_INCDEC_H_MODELVAR_0( event, pvad->haptic, 3 ) ;
					}
				break ;
			}

		}
		else
		{
			y = (i-5)*FH ;
	    switch(i)
			{
				case 6 :	 // vsource:2
  	  		lcd_puts_Pleft( y, XPSTR("Play Source") ) ;
					lcd_putsAttIdx( 14*FW, y, XPSTR("\006No    BeforeAfter "), pvad->vsource, attr ) ;
	    		if(attr)
					{
      	    CHECK_INCDEC_H_MODELVAR_0( event, pvad->vsource, 2 ) ;
					}
				break ;

				case 7 :
  	  		lcd_puts_Pleft( y, XPSTR("On no Telemetry") ) ;
					lcd_putsAttIdx( 17*FW, y, XPSTR("\004PlayMute"), pvad->mute, attr ) ;
	    		if(attr)
					{
      	    CHECK_INCDEC_H_MODELVAR_0( event, pvad->mute, 1 ) ;
					}
				break ;

				case 8 :	 // fnameType:3 ;
				{	
  	  		lcd_puts_Pleft( y, XPSTR("FileType") ) ;
					lcd_putsAttIdx( 14*FW, y, XPSTR("\006------  NameNumber Audio"),pvad->fnameType,attr ) ;
					uint8_t previous = pvad->fnameType ;
	    		if(attr)
					{
      	    CHECK_INCDEC_H_MODELVAR_0( event, pvad->fnameType, 3 ) ;
					}
					if ( pvad->fnameType != previous )
					{
						if ( pvad->fnameType )
						{
							pvad->file.name[0] = 0 ;
							pvad->file.name[1] = 0 ;
						}
						else
						{
							pvad->file.vfile = 0 ;
						}
					}
				}
				break ;
			
				case 9 :	 // filename ;
				if ( pvad->fnameType )
				{	
  	  		lcd_puts_Pleft( y, XPSTR("Voice File") ) ;
					if ( pvad->fnameType == 1 )	// Name
					{
						// Need to edit name here
						if ( pvad->file.name[0] )
						{
	    				lcd_putsnAtt( 12*FW, y, (const char *)pvad->file.name, 8, 0 ) ;
						}
	  				if( attr )
						{
							if (event == EVT_KEY_LONG(KEY_MENU) )
							{
      				 	pushMenu( menuProcSelectUvoiceFile ) ;
  	  					s_editMode = 0 ;
      				  killEvents(event);
							}
						
							uint8_t subSub = mstate2.m_posHorz;
							columns = 7 ;
							lcd_rect( 12*FW-2, y-1, 8*FW+4, 9 ) ;
							lcd_char_inverse( (12+subSub)*FW, y, 1*FW, s_editMode ) ;
	  		  		if(s_editMode)
							{
	    	  	  	char v = pvad->file.name[subSub] ;
								if ( v )
								{
		  		      	v = char2idx(v) ;
								}
	  			      CHECK_INCDEC_H_MODELVAR_0( event,v ,NUMCHARS-1);
  	  			    v = idx2char(v);
								if ( pvad->file.name[subSub] != v )
								{
									pvad->file.name[subSub] = v ;
	//    						eeDirty( EE_MODEL ) ;
								}
							}
						} 
					}
					else if ( pvad->fnameType == 2 )	// Number
					{
      	    lcd_outdezAtt(FW*20,y,pvad->file.vfile,attr ) ;
		  	  	if(attr)
						{
		  	    	pvad->file.vfile = checkIncDec16(event, pvad->file.vfile, 0, 500, EE_MODEL);
						}	
					}
					else if ( pvad->fnameType == 3 )	// Audio
					{
						lcd_putsAttIdx(15*FW, y, PSTR(STR_SOUNDS), pvad->file.vfile, attr ) ;
		  	  	if(attr)
						{
							CHECK_INCDEC_H_MODELVAR( event, pvad->file.vfile, 0, 15 ) ;
						}
					}
				break ;
				}
				
				case 10 :	 // Blank ;
  	  		lcd_puts_Pleft( y, XPSTR("Delete") ) ;
					lcd_putsAtt( 12*FW, y, XPSTR("MENU LONG"), attr ) ;
  				if( attr )
					{
						if (event == EVT_KEY_LONG(KEY_MENU) )
						{
							pvad->source = 0 ;
							pvad->func = 0 ;
							pvad->swtch = 0 ;
							pvad->rate = 0 ;
							pvad->fnameType  = 0 ;
							pvad->haptic = 0 ;
							pvad->vsource = 0  ;
							pvad->offset = 0 ;
							pvad->file.vfile = 0 ;
 	  					s_editMode = 0 ;
							if ( i == 10 )
							{
								mstate2.m_posVert = 8 ;
							}
						}
					}
				break ;
			}
		}
	}
}

void menuProcVoiceAlarm(uint8_t event)
{
  SUBMENU(XPSTR("Voice Alarms"), NUM_VOICE_ALARMS, {0} ) ;

  int8_t sub = mstate2.m_posVert ;

  evalOffset(sub, 6) ;

  switch (event)
	{
    case EVT_KEY_FIRST(KEY_MENU) :
    case EVT_KEY_BREAK(BTN_RE) :
      s_currIdx = sub ;
      killEvents(event);
      pushMenu(menuProcVoiceOne) ;
		break;
  }

  uint8_t y = 1*FH ;
  for (uint8_t i = 0 ; i < NUM_VOICE_ALARMS ; i += 1 )
	{
    uint8_t k = i + s_pgOfs ;
    uint8_t attr = sub == k ? INVERS : 0 ;
		VoiceAlarmData *pvad = &g_model.vad[k] ;
      
		if(y>7*FH) break ;
    
		lcd_puts_Pleft( y, XPSTR("VA") ) ;
    lcd_outdezAtt( (k<9) ? FW*3-1 : FW*4-2, y, k+1, 0 ) ;
		putsChnRaw( 5*FW, y, pvad->source, 0 ) ;
		putsDrSwitches( 9*FW, y, pvad->swtch, 0 ) ;
		if ( pvad->rate < 3 )
		{
			lcd_putsAttIdx( 13*FW, y, XPSTR("\004  ON OFFBOTH"),pvad->rate,0 ) ;
		}
		else
		{
      lcd_outdezAtt(FW*16,y,pvad->rate-2, 0 ) ;
		}

    switch ( pvad->fnameType )
		{
			case 1 :
				lcd_putc( 19*FW, y, 'N' ) ;
			break ;
			case 2 :
				lcd_putc( 19*FW, y, '#' ) ;
			break ;
			case 3 :
				lcd_putc( 19*FW, y, 'A' ) ;
			break ;
		}

		
		if (pvad->haptic)
		{
			lcd_putc( 20*FW, y, 'H' ) ;
		}
		if ( attr )
		{
			lcd_char_inverse( 0, y, 20*FW, 0 ) ;
		}
		
		y += FH ;
  }

}



void menuProcMixOne(uint8_t event)
{
	static uint8_t edit3Switch ;
	uint8_t numItems = 16 ;
	SKYMixData *md2 = &g_model.mixData[s_currMixIdx] ;
//#ifdef PCBX9D
//	if ( md2->srcRaw == MIX_3POS )
//	{
//		numItems = 17 ;
//	}
//#endif
//#ifdef PCBSKY
//	if ( md2->srcRaw == MIX_3POS )
//	{
//		numItems = 17 ;
//	}
//#endif
  SUBMENU_NOTITLE( numItems, { 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 0 } ) ;
    uint8_t x = TITLEP( PSTR(STR_EDIT_MIX));
//    uint8_t x = TITLEP(s_currMixInsMode ? PSTR("INSERT MIX ") : PSTR("EDIT MIX "));

		if ( event == EVT_ENTRY )
		{
			RotaryState = ROTARY_MENU_UD ;
			edit3Switch = 0 ;
		}
		if ( event == EVT_ENTRY_UP )
		{
			SingleExpoChan = 0 ;
		}

		putsChn(x+1*FW,0,md2->destCh,0);
    int8_t  sub    = mstate2.m_posVert;

    evalOffset(sub, 6);

    for(uint8_t k=0; k<7; k++)
    {
        uint8_t y = (k+1) * FH;
        uint8_t i = k + s_pgOfs;
        uint8_t attr = sub==i ? InverseBlink : 0;
    		uint8_t b ;
        switch(i){
        case 0:
            lcd_puts_P(  2*FW,y,PSTR(STR_2SOURCE));
						if ( md2->srcRaw == MIX_3POS )
						{
							if (attr)
							{
								if ( ( event == EVT_KEY_LONG(KEY_MENU) ) || ( event == EVT_KEY_LONG(BTN_RE) ) )
								{
									if ( edit3Switch == 0 )
									{
										edit3Switch = 1 ;
									}
									else
									{
										edit3Switch = 0 ;
									}
								}
							}
#ifdef PCBSKY
            	lcd_putsAttIdx(18*FW, y,XPSTR("\003IDxTHRRUDELEAILGEATRN"),md2->switchSource,edit3Switch ? INVERS : 0 ) ;
#endif
#ifdef PCBX9D
	            lcd_putsAttIdx(18*FW, y,PSTR(HW_SWITCHES_STR),md2->switchSource,edit3Switch ? INVERS : 0);
#endif
						}
						else
						{
							edit3Switch = 0 ;
						}
            putsChnRaw(   FW*13,y,md2->srcRaw, MIX_SOURCE | ( edit3Switch ? 0 : attr ) ) ;
								
						if ( edit3Switch )
						{
#ifdef PCBSKY
							if(attr) CHECK_INCDEC_H_MODELVAR_0( event, md2->switchSource, 6) ;
#endif
#ifdef PCBX9D
#ifdef REV9E
							if(attr) CHECK_INCDEC_H_MODELVAR_0( event, md2->switchSource, 17) ;
#else
							if(attr) CHECK_INCDEC_H_MODELVAR_0( event, md2->switchSource, 7) ;
#endif	// REV9E
#endif
						}
						else
						{
#if GVARS
            	if(attr)
							{
#if NUM_EXTRA_POTS
								uint8_t x = md2->srcRaw ;
								if ( x >= EXTRA_POTS_POSITION )
								{
									if ( x >= EXTRA_POTS_START )
									{
										x -= ( EXTRA_POTS_START - EXTRA_POTS_POSITION ) ;
									}
									else
									{
										x += NUM_EXTRA_POTS ;
									}
								}
								CHECK_INCDEC_H_MODELVAR( event, x, 1,NUM_SKYXCHNRAW+1+MAX_GVARS+1+NUM_SCALERS+8+NUM_EXTRA_POTS );
								if ( x >= EXTRA_POTS_POSITION )
								{
									if ( x < EXTRA_POTS_POSITION + NUM_EXTRA_POTS )
									{
										x += EXTRA_POTS_START - EXTRA_POTS_POSITION ;
									}
									else
									{
										x -= NUM_EXTRA_POTS ;
									}
								}
								md2->srcRaw = x ;
#else
								CHECK_INCDEC_H_MODELVAR( event, md2->srcRaw, 1,NUM_SKYXCHNRAW+1+MAX_GVARS+1+NUM_SCALERS+8 );
#endif	// NUM_EXTRA_POTS
							}
#else
	            if(attr) CHECK_INCDEC_H_MODELVAR( event, md2->srcRaw, 1,NUM_SKYXCHNRAW+1+4);
#endif
						}

  					if ( (attr) && ( event == EVT_KEY_LONG(KEY_MENU) ) )
						{
							if ( ( md2->srcRaw) && ( md2->srcRaw <= 4 ) )
							{
								SingleExpoChan = 1 ;
								s_expoChan = md2->srcRaw-1 ;
        				pushMenu(menuProcExpoAll);
							}
						}
            break;
        case 1:
            lcd_puts_P(  2*FW,y,PSTR(STR_2WEIGHT));
#if GVARS
						md2->weight = gvarMenuItem( FW*16, y, md2->weight, -125, 125, attr, event ) ;
#else
            lcd_outdezAtt(FW*13,y,md2->weight,attr|LEFT);
            if(attr) CHECK_INCDEC_H_MODELVAR( event, md2->weight, -125,125);
#endif
            break;
        case 2:
//            lcd_puts_P(  2*FW,y,md2->enableFmTrim ? PSTR(STR_FMTRIMVAL) : PSTR(STR_OFFSET));
            lcd_puts_P( FW, y, PSTR(STR_OFFSET) ) ;
#if GVARS
						md2->sOffset = gvarMenuItem( FW*16, y, md2->sOffset, -125, 125, attr, event ) ;
#else
						lcd_outdezAtt(FW*13,y,md2->sOffset,attr|LEFT);
            if(attr) CHECK_INCDEC_H_MODELVAR( event, md2->sOffset, -125,125);
#endif
            break;
        case 3:
						md2->lateOffset = onoffMenuItem_m( event, md2->lateOffset, y, PSTR(STR_2FIX_OFFSET), attr ) ;
            break;
        case 4:
						if ( ( md2->srcRaw <=4 ) )
						{
							md2->disableExpoDr = 1 - onoffMenuItem_m( event, 1-md2->disableExpoDr, y, PSTR(STR_ENABLEEXPO), attr ) ;
						}
						else
						{
							md2->disableExpoDr = onoffMenuItem_m( event, md2->disableExpoDr, y, XPSTR("\001Incl.Safety/Lims"), attr ) ;
						}
//						b = md2->enableFmTrim ;
//            lcd_puts_P(  2*FW,y,PSTR(STR_FLMODETRIM));
	//					lcd_putsnAtt(FW*13,y, PSTR("OFFON ")+3*b,3,attr);  //default is 0=OFF
            //            lcd_putsnAtt( x, y, PSTR("OFFON ")+3*value,3,mode ? INVERS:0) ;
            //            menu_lcd_onoff( FW*13, y, md2->enableFmTrim, sub==i ) ;
  //          if(attr) { CHECK_INCDEC_H_MODELVAR( event, b, 0,1); md2->enableFmTrim = b ; }
            break;
        case 5:
						md2->carryTrim = 1-onoffMenuItem_m( event, 1-md2->carryTrim, y, PSTR(STR_2TRIM), attr ) ;
//            lcd_puts_P(  2*FW,y,PSTR(STR_2TRIM));
//            lcd_putsnAtt(FW*13,y, PSTR("ON OFF")+3*md2->carryTrim,3,attr);  //default is 0=ON
//            if(attr) CHECK_INCDEC_H_MODELVAR( event, md2->carryTrim, 0,1);
            break;
        
				case 6:
					{	
					 	uint8_t value = md2->differential ;
						if ( value == 0 )
						{
							if ( md2->curve <= -28 )
							{
								value = 2 ;		// Expo
							}
						}
					 	uint8_t value2 = value ;
	          lcd_putsAtt(  1*FW, y, value ? ( value == 2 ) ? XPSTR("\021Expo") : PSTR(STR_15DIFF) : PSTR(STR_Curve), attr ) ;
    		    if(attr) CHECK_INCDEC_H_MODELVAR_0( event, value2, 2) ;
					 	if ( value != value2 )
						{
							if ( value2 == 2 )
							{
								md2->curve = -128 ;
							}
							else
							{
								md2->curve = 0 ;
							}
							md2->differential = value2 & 1 ;	// 0 and 2 turn it off
						}
					}
        break ;

        case 7:
						if ( md2->differential )		// Non zero for curve
						{	
		          md2->curve = gvarMenuItem( 12*FW, y, md2->curve, -100, 100, attr /*( m_posHorz==1 ? attr : 0 )*/, event ) ;
						}
						else
						{
							if ( md2->curve <= -28 )
							{
								int8_t value = md2->curve + 128 ;	// 0 to 100
            		lcd_outdezAtt(FW*17,y,value,attr|LEFT);
            		if(attr) CHECK_INCDEC_H_MODELVAR_0( event, value, 100 ) ;
								md2->curve = value - 128 ;
							}
							else
							{
								put_curve( 2*FW, y, md2->curve, attr ) ;
          	  	if(attr)
								{
        		  		CHECK_INCDEC_H_MODELVAR( event, md2->curve, -MAX_CURVE5-MAX_CURVE9, MAX_CURVE5+MAX_CURVE9+7-1);
									if ( event==EVT_KEY_FIRST(KEY_MENU) )
									{
										if ( md2->curve>=CURVE_BASE )
										{
          	  	    	s_curveChan = md2->curve-CURVE_BASE;
	          		      pushMenu(menuProcCurveOne);
										}
										if ( md2->curve < 0 )
										{
          	  	    	s_curveChan = -md2->curve-1 ;
	          		      pushMenu(menuProcCurveOne);
										}
									}
          	  	}
        		  }
						}
        break ;

        case 8:
            lcd_puts_P(  2*FW,y,PSTR(STR_2SWITCH));
            putsDrSwitches(13*FW,  y,md2->swtch,attr);
            if(attr) CHECK_INCDEC_MODELSWITCH( event, md2->swtch, -MaxSwitchIndex, MaxSwitchIndex);
            break;

        case 9:
					{	
						uint8_t b = 1 ;
            lcd_puts_Pleft( y,XPSTR("\001MODES"));
  					for ( uint8_t p = 0 ; p<MAX_MODES+1 ; p++ )
						{
							uint8_t z = md2->modeControl ;
    					lcd_putcAtt( (9+p)*(FW+1), y, '0'+p, ( z & b ) ? 0 : INVERS ) ;
							if( attr && ( mstate2.m_posHorz == p ) )
							{
								lcd_rect( (9+p)*(FW+1)-1, y-1, FW+2, 9 ) ;
								if ( event==EVT_KEY_BREAK(KEY_MENU) || event==EVT_KEY_BREAK(BTN_RE) ) 
								{
									md2->modeControl ^= b ;
      						eeDirty(EE_MODEL) ;
    							s_editMode = false ;
								}
							}
							b <<= 1 ;
						}
					}
        break ;

        case 10:
            lcd_puts_P(  2*FW,y,PSTR(STR_2WARNING));
						b = md2->mixWarn ;
            if(b)
                lcd_outdezAtt(FW*13,y,b,attr|LEFT);
            else
                lcd_putsAtt(  FW*13,y,PSTR(STR_OFF),attr);
            if(attr) { CHECK_INCDEC_H_MODELVAR_0( event, b, 3); md2->mixWarn = b ; }
            break;
        case 11:
            lcd_puts_P(  2*FW,y,PSTR(STR_2MULTIPLEX));
            lcd_putsAttIdx(13*FW, y,PSTR(STR_ADD_MULT_REP),md2->mltpx,attr);
            if(attr) CHECK_INCDEC_H_MODELVAR_0( event, md2->mltpx, 2); //!! bitfield
            break;
        case 12:
						b = md2->delayDown ;
            lcd_puts_P(  2*FW,y,PSTR(STR_2DELAY_DOWN));
  					lcd_putc( FW*16, y, '.' ) ;
            lcd_outdezAtt(FW*16,y,b/10,attr);
            lcd_outdezAtt(FW*18-3,y,b%10,attr);
            if(attr) { md2->delayDown = checkIncDec16( event, b, 0, 250, EE_MODEL); }
            break;
        case 13:
						b = md2->delayUp ;
            lcd_puts_P(  2*FW,y,PSTR(STR_2DELAY_UP));
  					lcd_putc( FW*16, y, '.' ) ;
            lcd_outdezAtt(FW*16,y,b/10,attr);
            lcd_outdezAtt(FW*18-3,y,b%10,attr);
            if(attr) { md2->delayUp = checkIncDec16( event, b, 0, 250, EE_MODEL); }
            break;
        case 14:
						b = md2->speedDown ;
            lcd_puts_P(  2*FW,y,PSTR(STR_2SLOW_DOWN));
  					lcd_putc( FW*16, y, '.' ) ;
            lcd_outdezAtt(FW*16,y,b/10,attr);
            lcd_outdezAtt(FW*18-3,y,b%10,attr);
            if(attr) { md2->speedDown = checkIncDec16( event, b, 0, 250, EE_MODEL); }
            break;
        case 15:
						b = md2->speedUp ;
            lcd_puts_P(  2*FW,y,PSTR(STR_2SLOW_UP));
  					lcd_putc( FW*16, y, '.' ) ;
            lcd_outdezAtt(FW*16,y,b/10,attr);
            lcd_outdezAtt(FW*18-3,y,b%10,attr);
            if(attr) { md2->speedUp = checkIncDec16( event, b, 0, 250, EE_MODEL); }
            break;
//#ifdef PCBX9D
//				case 16:
//            lcd_puts_P( 2*FW,y,XPSTR("3POS Switch"));
//            lcd_putsAttIdx(15*FW, y,XPSTR("\002SASBSCSDSESFSGSH"),md2->switchSource,attr);
//            if(attr) CHECK_INCDEC_H_MODELVAR_0( event, md2->switchSource, 7) ;
//				break ;
//#endif
//#ifdef PCBSKY
//				case 16:
//            lcd_puts_P( 2*FW,y,XPSTR("3POS Switch"));
//						{
//            	lcd_putsAttIdx(15*FW, y,XPSTR("\003IDxTHRRUDELEAILGEATRN"),md2->switchSource,attr);
//							if(attr) CHECK_INCDEC_H_MODELVAR_0( event, md2->switchSource, 6) ;
							
//							uint32_t number = 0 ;
//							uint8_t sm = g_eeGeneral.switchMapping ;
//							uint8_t text[20] ;
//							uint8_t *p ;
//							p = cpystr( text, (uint8_t *)"\003IDx" ) ;
//							if ( sm & USE_ELE_3POS )
//							{
//								p = cpystr( p, (uint8_t *)"ELE" ) ;
//								number += 1 ;
//							}
//							if (sm & USE_THR_3POS )
//							{
//								p = cpystr( p, (uint8_t *)"THR" ) ;
//								number += 1 ;
//							}
//							if (sm & USE_RUD_3POS )
//							{
//								p = cpystr( p, (uint8_t *)"RUD" ) ;
//								number += 1 ;
//							}
//							if ( sm & USE_GEA_3POS )
//							{
//								p = cpystr( p, (uint8_t *)"GEA" ) ;
//								number += 1 ;
//							}
//							if ( sm & USE_AIL_3POS )
//							{
//								p = cpystr( p, (uint8_t *)"AIL" ) ;
//								number += 1 ;
//							}
//          	  lcd_putsAttIdx(15*FW, y, (char*)text, md2->switchSource, attr ) ;
//        	    if(attr) CHECK_INCDEC_H_MODELVAR_0( event, md2->switchSource, number) ;
//						}	
//				break ;
//#endif
        }
    }
}

//struct MixTab{
//    uint8_t chId:6;    //:4  1..NUM_SKYXCHNOUT  dst chn id
//    uint8_t   showCh:1;// show the dest chn
//    uint8_t   hasDat:1;// show the data info
//    int8_t selCh;   //:5  1..MAX_SKYMIXERS+NUM_XCHNOUT sel sequence
//    int8_t selDat;  //:5  1..MAX_SKYMIXERS+NUM_XCHNOUT sel sequence
//    int8_t insIdx;  //:5  0..MAX_SKYMIXERS-1        insert index into mix data tab
//    int8_t editIdx; //:5  0..MAX_SKYMIXERS-1        edit   index into mix data tab
//} s_mixTab[MAX_SKYMIXERS+NUM_SKYXCHNOUT+1];
int8_t s_mixMaxSel;

//void genMixTab()
//{
//    uint8_t maxDst  = 0;
//    uint8_t mtIdx   = 0;
//    uint8_t sel     = 1;
//    memset(s_mixTab,0,sizeof(s_mixTab));

//    SKYMixData *md=g_model.mixData;

//    for(uint8_t i=0; i<MAX_SKYMIXERS; i++)
//    {
//        uint8_t destCh = md[i].destCh;
//        if(destCh==0) destCh=NUM_SKYXCHNOUT;
//        if(destCh > maxDst){
//            while(destCh > maxDst){ //ch-loop, hole alle channels auf
//                maxDst++;
//                s_mixTab[mtIdx].chId  = maxDst; //mark channel header
//                s_mixTab[mtIdx].showCh = true;
//                s_mixTab[mtIdx].selCh = sel++; //vorab vergeben, falls keine dat
//                s_mixTab[mtIdx].insIdx= i;     //
//                mtIdx++;
//            }
//            mtIdx--; //folding: letztes ch bekommt zusaetzlich dat
//            s_mixMaxSel =sel;
//            sel--; //letzte zeile hat dat, falls nicht ist selCh schon belegt
//        }
//        if(md[i].destCh==0) break; //letzter eintrag in mix data tab
//        s_mixTab[mtIdx].chId    = destCh; //mark channel header
//        s_mixTab[mtIdx].editIdx = i;
//        s_mixTab[mtIdx].hasDat  = true;
//        s_mixTab[mtIdx].selDat  = sel++;
//        if(md[i].destCh == md[i+1].destCh){
//            s_mixTab[mtIdx].selCh  = 0; //ueberschreibt letzte Zeile von ch-loop
//            s_mixTab[mtIdx].insIdx = 0; //
//        }
//        else{
//            s_mixTab[mtIdx].selCh  = sel++;
//            s_mixTab[mtIdx].insIdx = i+1; //
//        }
//        s_mixMaxSel =sel;
//        mtIdx++;
//    }
//}

static void memswap( void *a, void *b, uint8_t size )
{
    uint8_t *x ;
    uint8_t *y ;
    uint8_t temp ;

    x = (unsigned char *) a ;
    y = (unsigned char *) b ;
    while ( size-- )
    {
        temp = *x ;
        *x++ = *y ;
        *y++ = temp ;
    }
}

void moveMix(uint8_t idx, uint8_t dir) //true=inc=down false=dec=up - Issue 49
{
    SKYMixData *src= &g_model.mixData[idx];
    if(idx==0 && !dir)
		{
      if (src->destCh>0)
			{
			  src->destCh--;
			}
			STORE_MODELVARS;
			return ;
		}

    if(idx>MAX_SKYMIXERS || (idx==MAX_SKYMIXERS && dir)) return;
    uint8_t tdx = dir ? idx+1 : idx-1;
    SKYMixData *tgt= &g_model.mixData[tdx];

    if((src->destCh==0) || (src->destCh>NUM_SKYCHNOUT) || (tgt->destCh>NUM_SKYCHNOUT)) return;

    if(tgt->destCh!=src->destCh)
		{
        if ((dir)  && (src->destCh<NUM_SKYCHNOUT)) src->destCh++;
        if ((!dir) && (src->destCh>0))          src->destCh--;
				STORE_MODELVARS;
        return;
    }

    //flip between idx and tgt
    memswap( tgt, src, sizeof(SKYMixData) ) ;
		s_moveMixIdx = tdx ;
    
		STORE_MODELVARS;
//    eeWaitComplete() ;
}

void menuMixersLimit(uint8_t event)
{
    switch(event)
    {
    case  EVT_KEY_FIRST(KEY_EXIT):
        killEvents(event);
        popMenu(true);
        pushMenu(menuProcMix);
        break;
    }
    lcd_puts_Pleft(2*FH, PSTR(STR_MAX_MIXERS));
    lcd_outdezAtt(20*FW, 2*FH, getMixerCount(),0);

    lcd_puts_Pleft(4*FH, PSTR(STR_PRESS_EXIT_AB));
}

uint8_t getMixerCount()
{
    uint8_t mixerCount = 0;
    uint8_t dch ;

    for(uint8_t i=0;i<MAX_SKYMIXERS;i++)
    {
        dch = g_model.mixData[i].destCh ;
        if ((dch!=0) && (dch<=NUM_SKYCHNOUT))
        {
            mixerCount++;
        }
    }
    return mixerCount;
}

bool reachMixerCountLimit()
{
    // check mixers count limit
    if (getMixerCount() >= MAX_SKYMIXERS)
    {
        pushMenu(menuMixersLimit);
        return true;
    }
    else
    {
        return false;
    }
}

uint8_t mixToDelete;

void yesNoMenuExit( const char * s )
{
  
	lcd_puts_Pleft(1*FH, s ) ;	
  lcd_puts_Pleft( 5*FH,PSTR(STR_YES_NO));
  lcd_puts_Pleft( 6*FH,PSTR(STR_MENU_EXIT));
}


void menuDeleteMix(uint8_t event)
//void menuDeleteModel(uint8_t event)
{
  switch(event)
	{
    case EVT_ENTRY:
        audioDefevent(AU_WARNING1);
    break;
    
		case EVT_KEY_FIRST(KEY_MENU):
        deleteMix(mixToDelete);
        //fallthrough
    case EVT_KEY_FIRST(KEY_EXIT):
        killEvents(event);
        popMenu(true);
        pushMenu(menuProcMix);
    break;
  }
//  lcd_puts_Pleft(1*FH, PSTR("DELETE MIX?"));
	yesNoMenuExit( PSTR(STR_DELETE_MIX) ) ;

}

uint8_t s_moveMode;

int8_t qRotary()
{
	int8_t diff = 0 ;

	if ( Rotary_diff > 0)
	{
		diff = 1 ;
	}
	else if ( Rotary_diff < 0)
	{
		diff = -1 ;
	}
	Rotary_diff = 0 ;
	return diff ;
}

#define POPUP_NONE			0
#define POPUP_SELECT		1
#define POPUP_EXIT			2

static uint8_t popupProcess( uint8_t event, uint8_t max )
{
	int8_t popidxud = qRotary() ;
	uint8_t popidx = PopupData.PopupIdx ;

	if ( PopupData.PopupTimer )
	{
		if ( BLINK_ON_PHASE )
		{
			if ( --PopupData.PopupTimer == 0 )
			{
				// Timeout popup
				PopupData.PopupActive = 0 ;
				return POPUP_EXIT ;
			}
		}
	}
	else
	{
		PopupData.PopupTimer = 255 ;
  }
  
	switch(event)
	{
    case EVT_KEY_BREAK(KEY_MENU) :
    case EVT_KEY_BREAK(BTN_RE):
			PopupData.PopupActive = 0 ;
			PopupData.PopupTimer = 0 ;
		return POPUP_SELECT ;
    
		case EVT_KEY_FIRST(KEY_UP) :
			popidxud = -1 ;
		break ;
    
		case EVT_KEY_FIRST(KEY_DOWN) :
			popidxud = 1 ;
		break ;
    
		case EVT_KEY_FIRST(KEY_EXIT) :
		case EVT_KEY_LONG(BTN_RE) :
			killEvents( event ) ;
			PopupData.PopupActive = 0 ;
			PopupData.PopupTimer = 0 ;
		return POPUP_EXIT ;
	}

	if (popidxud > 0)
	{
		if ( popidx < max )
		{
			popidx += 1 ;
		}
	}
	else if (popidxud < 0)
	{		
		if ( popidx )
		{
			popidx -= 1 ;
		}
	}

	if (popidxud )
	{
		if ( PopupData.PopupTimer )
		{
			PopupData.PopupTimer = 255 ;
		}
	}	

	PopupData.PopupIdx = popidx ;
	return POPUP_NONE ;
}

//const char *MixPopList = PSTR(STR_MIX_POPUP) ;

static uint32_t popupDisplay( const char *list, uint16_t mask, uint8_t width )
{
	uint32_t entries = 0 ;
	uint8_t y = FH ;

	while ( mask )
	{
		if ( mask & 1 )
		{
			lcd_putsn_P( 3*FW, y, "              ", width ) ;
			lcd_puts_P( 4*FW, y, (const char *)(list) ) ;
			entries += 1 ;
			y += FH ;
		}
		mask >>= 1 ;
		while ( *list )
		{
			list += 1 ;			
		}		
		list += 1 ;			
	}
	plotType = PLOT_BLACK ;
	lcd_rect( 3*FW, 1*FH-1, width*FW, y+2-FH ) ;
	plotType = PLOT_XOR ;
	lcd_char_inverse( 4*FW, (PopupData.PopupIdx+1)*FH, (width-2)*FW, 0 ) ;

	return entries ;
}

static uint8_t popTranslate( uint8_t popidx, uint16_t mask )
{
	uint8_t position ;
	popidx += 1 ;
	for ( position = 0 ; position < 16 ; position += 1 )
	{
		if ( mask & 1 )
		{
			if ( --popidx == 0)
			{
				break ;
			}
		}
		mask >>= 1 ;
	}
	return position ;
}


uint32_t doPopup( const char *list, uint16_t mask, uint8_t width, uint8_t event )
{
	uint32_t count = popupDisplay( list, mask, width ) ;
	uint8_t popaction = popupProcess( event, count - 1 ) ;
	uint8_t popidx = PopupData.PopupIdx ;
	PopupData.PopupSel = popTranslate( popidx, mask ) ;
	return popaction ;
}

void mixpopup( uint8_t event )
{
	uint8_t popaction = doPopup( PSTR(STR_MIX_POPUP), 0x1F, 8, event ) ;
//	popupDisplay( PSTR(STR_MIX_POPUP), 0x1F, 8 ) ;
	
//	uint8_t popaction = popupProcess( event, 4 ) ;
//	uint8_t popidx = PopupIdx ;
//	lcd_char_inverse( 4*FW, (popidx+1)*FH, 6*FW, 0 ) ;

  if ( popaction == POPUP_SELECT )
	{
		uint8_t popidx = PopupData.PopupSel ;
		if ( popidx == 1 )
		{
      if ( !reachMixerCountLimit())
      {
				s_currMixInsMode = 1 ;
      	insertMix(++s_currMixIdx, 0 ) ;
  	    s_moveMode=false;
			}
		}
		if ( popidx < 2 )
		{
	    pushMenu(menuProcMixOne) ;
		}
		else if ( popidx == 4 )		// Delete
		{
			mixToDelete = s_currMixIdx;
			killEvents(event);
			pushMenu(menuDeleteMix);
		}
		else
		{
			if( popidx == 2 )	// copy
			{
     		insertMix(++s_currMixIdx, 1 ) ;
			}
			// PopupIdx == 2 or 3, copy or move
			s_moveMode = 1 ;
		}
		PopupData.PopupActive = 0 ;
	}
//	else if ( popaction == POPUP_EXIT )
//	{
//		PopupData.PopupActive = 0 ;
//		killEvents( event ) ;
//	}
	s_moveMixIdx = s_currMixIdx ;

//	if ( Tevent )
//	{
//  	killEvents(Tevent);
//	}

}



void menuProcMix(uint8_t event)
{
	TITLE(PSTR(STR_MIXER));
	static MState2 mstate2;
	
	if ( s_moveMode )
	{
		int8_t moveByRotary ;
		moveByRotary = qRotary() ;		// Do this now, check_simple destroys rotary data
		if ( moveByRotary )
		{
			if ( moveByRotary > 0 )
			{
				event = EVT_KEY_FIRST(KEY_DOWN) ;
			}
			else
			{
				event = EVT_KEY_FIRST(KEY_UP) ;
			}
		}
	}
	
	if ( !PopupData.PopupActive )
	{
		if (SubMenuFromIndex)
		{
			mstate2.check_simple(event,0,NULL,0,s_mixMaxSel+1) ;
		}
		else
		{
			mstate2.check_simple(event,e_Mix,menuTabModel,DIM(menuTabModel),s_mixMaxSel+1) ;
		}
	}
	uint8_t save_event = event ;

  int8_t  sub    = mstate2.m_posVert;
	int8_t	menulong = 0 ;

    switch(event)
    {
	    case EVT_ENTRY:
        s_moveMode=false;
  	  break;
    
			case EVT_KEY_FIRST(KEY_MENU):
	    case EVT_KEY_BREAK(BTN_RE):
				if ( s_moveMode )
				{
	  	  	s_moveMode = false ;
					RotaryState = ROTARY_MENU_UD ;
  	  		break;
				}
				// Else fall through    
				if ( !PopupData.PopupActive )
				{
		  		killEvents(event);
					save_event = 0 ;			// Prevent changing weight to/from Gvar
    		  if(sub<1) break;
					menulong = 1 ;
				}
  	  break;
    }

  if(sub==0) s_moveMode = false;
	uint8_t t_pgOfs = evalOffset( sub, 7 ) ;
    
	if ( PopupData.PopupActive )
	{
		event = 0 ;
	}
		
  uint8_t mix_index = 0 ;
  uint8_t current = 1 ;

	if ( s_moveMode )
	{
		int8_t dir ;
		
		if ( ( dir = (event == EVT_KEY_FIRST(KEY_DOWN) ) ) || event == EVT_KEY_FIRST(KEY_UP) )
		{
			moveMix( s_currMixIdx, dir ) ; //true=inc=down false=dec=up - Issue 49
		}
	}

  for ( uint8_t chan=1 ; chan <= NUM_SKYCHNOUT ; chan += 1 )
	{
    SKYMixData *pmd = &g_model.mixData[mix_index];
    
    if ( t_pgOfs < current && current-t_pgOfs < 8)
		{
      putsChn(0, (current-t_pgOfs)*FH, chan, 0) ; // show CHx
    }

		uint8_t firstMix = mix_index ;
		if (mix_index<MAX_SKYMIXERS && /* pmd->srcRaw && */ pmd->destCh == chan)
		{
    	do
			{
				if (t_pgOfs < current )
				{
					if ( current-t_pgOfs < 8 )
					{
    	  	  uint8_t y = (current-t_pgOfs)*FH ;
    				uint8_t attr = 0 ;

						if ( !s_moveMode && (sub == current) )
						{
							s_currMixIdx = mix_index ;
							s_currDestCh = chan ;		// For insert
							if ( menulong )
							{
								PopupData.PopupIdx = 0 ;
								PopupData.PopupActive = 1 ;
							}
							if ( PopupData.PopupActive == 0 )
							{
    						attr = INVERS ;
							}
						}
        	  if(firstMix != mix_index) //show prefix only if not first mix
        	 		lcd_putsAttIdx( 4*FW-5, y, XPSTR("\001+*R"),pmd->mltpx,0 ) ;

						if ( pmd->srcRaw == MIX_3POS )
						{
#ifdef PCBSKY
            	lcd_putsAttIdx(8*FW, y,XPSTR("\004sIDxsTHRsRUDsELEsAILsGEAsTRN"),pmd->switchSource, 0 ) ;
#endif
#ifdef PCBX9D
	            lcd_putsAttIdx(8*FW, y,PSTR(HW_SWITCHES_STR),pmd->switchSource, 0 ) ;
#endif
						}
						else
						{
							putsChnRaw(     8*FW, y, pmd->srcRaw, /*attr | */ MIX_SOURCE ) ;
						}
	#if GVARS
						pmd->weight = gvarMenuItem( 7*FW+FW/2, y, pmd->weight, -125, 125, attr, event ) ;
	#else
						lcd_outdezAtt(  7*FW+FW/2, y, pmd->weight, attr ) ; //attr);
	#endif
//						if ( attr )
//						{
//							lcd_char_inverse( 0, y, 12*FW, 0 ) ;
//						}

//    	  	  lcd_putcAtt(    7*FW+FW/2, y, '%', 0 ) ; //tattr);
    	  	  if( pmd->swtch) putsDrSwitches( 12*FW, y, pmd->swtch, 0 ) ; //tattr);
						if(pmd->curve)
						{
							if ( pmd->differential ) lcd_putcAtt(    16*FW, y, *PSTR(CHR_d), 0 ) ;
							else
							{
								if ( ( pmd->curve > -28 ) && ( pmd->curve <= 27 ) )
								{
	    	  	  		put_curve( 16*FW, y, pmd->curve, 0 ) ;
								}
								else
								{
									lcd_putcAtt( 16*FW, y, 'E', 0 ) ;
								}
							}
						}
						char cs = ' ';
        	  if (pmd->speedDown || pmd->speedUp)
        	    cs = *PSTR(CHR_S);
        	  if (pmd->delayUp || pmd->delayDown)
        	    cs = (cs == *PSTR(CHR_S) ? '*' : *PSTR(CHR_D));
        	  lcd_putc(20*FW+1, y, cs ) ;

						if ( s_moveMode )
						{
							if ( s_moveMixIdx == mix_index )
							{
								lcd_char_inverse( 4*FW, y, 17*FW, 0 ) ;
								s_currMixIdx = mix_index ;
								sub = mstate2.m_posVert = current ;
							}
						}
					}
					else
					{
						if ( current-t_pgOfs == 8 )
						{
							if ( s_moveMode )
							{
								if ( s_moveMixIdx == mix_index )
								{
									mstate2.m_posVert += 1 ;								
								}
							}
						}
					}
				}
				current += 1 ; mix_index += 1; pmd += 1 ;  // mixCnt += 1 ; 
    	} while ( (mix_index<MAX_SKYMIXERS && /* pmd->srcRaw && */ pmd->destCh == chan)) ;
		}
		else
		{
			if (sub == current)
			{
				s_currDestCh = chan ;		// For insert
				s_currMixIdx = mix_index ;
				lcd_rect( 0, (current-t_pgOfs)*FH-1, 25, 9 ) ;
//				s_moveMode = 0 ;		// Can't move this
				if ( menulong )		// Must need to insert here
				{
      		if ( !reachMixerCountLimit())
      		{
//						s_currMixInsMode = 1 ;
      			insertMix(s_currMixIdx, 0 ) ;
  	    		s_moveMode=false;
	      		pushMenu(menuProcMixOne) ;
						return ;
      		}
				}
			}
			current += 1 ;
		}
	}
  if ( t_pgOfs < current && current-t_pgOfs < 8)
	{
   	lcd_puts_Pleft( 7*FH,XPSTR("Templates\020MENU") ) ;
		if (sub == s_mixMaxSel + 1 )
		{
			lcd_char_inverse( 0, 7*FH, 21*FW, 0 ) ;
			if ( event == EVT_KEY_FIRST(KEY_MENU) || event == EVT_KEY_BREAK(BTN_RE) )
			{
				pushMenu( menuProcTemplates ) ;
			}
		}
	}
	
	if ( PopupData.PopupActive )
	{
		mixpopup( save_event ) ;
    s_editMode = false;
	}
	s_mixMaxSel = current - 1 ;

}


uint16_t expou(uint16_t x, uint16_t k)
{
    // k*x*x*x + (1-k)*x
    return ((unsigned long)x*x*x/0x10000*k/(RESXul*RESXul/0x10000) + (RESKul-k)*x+RESKul/2)/RESKul;
}
// expo-funktion:
// ---------------
// kmplot
// f(x,k)=exp(ln(x)*k/10) ;P[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20]
// f(x,k)=x*x*x*k/10 + x*(1-k/10) ;P[0,1,2,3,4,5,6,7,8,9,10]
// f(x,k)=x*x*k/10 + x*(1-k/10) ;P[0,1,2,3,4,5,6,7,8,9,10]
// f(x,k)=1+(x-1)*(x-1)*(x-1)*k/10 + (x-1)*(1-k/10) ;P[0,1,2,3,4,5,6,7,8,9,10]

int16_t expo(int16_t x, int16_t k)
{
    if(k == 0) return x;
    int32_t   y;
    bool    neg =  x < 0;
    if(neg)   x = -x;
    if(k<0){
        y = RESXu-expou(RESXu-x,-k);
    }else{
        y = expou(x,k);
    }
    return neg? -y:y;
}


#ifdef EXTENDED_EXPO
/// expo with y-offset
class Expo
{
    uint16_t   c;
    int16_t    d,drx;
public:
    void     init(uint8_t k, int8_t yo);
    static int16_t  expou(uint16_t x,uint16_t c, int16_t d);
    int16_t  expo(int16_t x);
};
void    Expo::init(uint8_t k, int8_t yo)
{
    c = (uint16_t) k  * 256 / 100;
    d = (int16_t)  yo * 256 / 100;
    drx = d * ((uint16_t)RESXu/256);
}
int16_t Expo::expou(uint16_t x,uint16_t c, int16_t d)
{
    uint16_t a = 256 - c - d;
    if( (int16_t)a < 0 ) a = 0;
    // a x^3 + c x + d
    //                         9  18  27        11  20   18
    uint32_t res =  ((uint32_t)x * x * x / 0x10000 * a / (RESXul*RESXul/0x10000) +
                     (uint32_t)x                   * c
                     ) / 256;
    return (int16_t)res;
}
int16_t  Expo::expo(int16_t x)
{
    if(c==256 && d==0) return x;
    if(x>=0) return expou(x,c,d) + drx;
    return -expou(-x,c,-d) + drx;
}
#endif


void editExpoVals(uint8_t event, uint8_t edit, uint8_t x, uint8_t y, uint8_t which, uint8_t exWt, uint8_t stkRL)
{
    uint8_t  invBlk = (edit) ? InverseBlink : 0 ;
		uint8_t doedit ;
		int8_t *ptr ;			// volatile forces compiler to produce 'better' code
		ExpoData *eptr ;

//		doedit = (edit || (P1values.p1valdiff ) ) ;
		doedit = edit ? EDIT_DR_SWITCH_EDIT : 0 ;

		eptr = &g_model.expoData[s_expoChan] ;
    
		if(which==DR_DRSW1) {
				eptr->drSw1 = edit_dr_switch( x, y, eptr->drSw1, invBlk, doedit, event ) ;
    }
    else if(which==DR_DRSW2) {
				eptr->drSw2 = edit_dr_switch( x, y, eptr->drSw2, invBlk, doedit, event ) ;
    }
    else
		{
				ptr = &eptr->expo[which][exWt][stkRL] ;
        if(exWt==DR_EXPO)
				{
            
#if GVARS
					*ptr = gvarMenuItem( x, y, *ptr, -100, 100, invBlk, event ) ;
#else
					lcd_outdezAtt(x, y, *ptr, invBlk);
          if(doedit) CHECK_INCDEC_H_MODELVAR(event,*ptr,-100, 100);
#endif
        }
        else
				{
#if GVARS
					*ptr = gvarMenuItem( x, y, *ptr+100, 0, 100, invBlk, event ) - 100 ;
#else
          lcd_outdezAtt(x, y, *ptr+100, invBlk);
          if(doedit) CHECK_INCDEC_H_MODELVAR(event,*ptr,-100, 0);
#endif
        }
		}
}

void menuProcExpoAll(uint8_t event)
{
	TITLE(PSTR(STR_EXPO_DR));
	static MState2 mstate2;
	static const uint8_t mstate_tab[] = {0} ;
	
	if (SubMenuFromIndex)
	{
		event = mstate2.check(event,0,NULL,0,mstate_tab,DIM(mstate_tab)-1, 6-1 ) ;
	}
	else
	{
		event = mstate2.check(event,e_ExpoAll,menuTabModel,DIM(menuTabModel),mstate_tab,DIM(mstate_tab)-1, 6-1 ) ;
//  	MENU(PSTR(STR_EXPO_DR), menuTabModel, e_ExpoAll, 6, {0} ) ;
	}
	
	uint8_t stkVal ;
	int8_t  sub    = mstate2.m_posVert;
	if( sub )
	{
		StickScrollAllowed = 0 ;
	}

	uint8_t l_expoChan = s_expoChan ;
	{
    uint8_t attr = 0 ;
		if ( sub == 1 )
		{
			if ( SingleExpoChan == 0 )
			{
				s_expoChan = l_expoChan = checkIncDec( event, s_expoChan, 0, 3, 0 ) ;
				attr = InverseBlink ;
			}
		}		 
		putsChnRaw(0,FH,l_expoChan+1,attr) ;
	}

	uint8_t expoDrOn = get_dr_state(l_expoChan);
	switch (expoDrOn)
	{
    case DR_MID:
      lcd_puts_Pleft( FH,PSTR(STR_4DR_MID));
    break;
    case DR_LOW:
      lcd_puts_Pleft( FH,PSTR(STR_4DR_LOW));
    break;
    default: // DR_HIGH:
      lcd_puts_Pleft( FH,PSTR(STR_4DR_HI));
    break;
	}

	stkVal = DR_BOTH ;
	if(calibratedStick[l_expoChan]> 100) stkVal = DR_RIGHT;
	if(calibratedStick[l_expoChan]<-100) stkVal = DR_LEFT;
	if(IS_EXPO_THROTTLE(l_expoChan)) stkVal = DR_RIGHT;

	lcd_puts_Pleft(2*FH,PSTR(STR_2EXPO));
	editExpoVals( event, (stkVal != DR_RIGHT) && (sub==2), 4*FW, 3*FH, expoDrOn ,DR_EXPO, DR_LEFT ) ;
	editExpoVals( event, (stkVal != DR_LEFT) && (sub==2), 8*FW, 3*FH, expoDrOn ,DR_EXPO, DR_RIGHT ) ;

	lcd_puts_Pleft(4*FH,PSTR(STR_2WEIGHT));
	editExpoVals( event, (stkVal != DR_RIGHT) && (sub==3), 4*FW, 5*FH, expoDrOn ,DR_WEIGHT, DR_LEFT ) ;
	editExpoVals( event, (stkVal != DR_LEFT) && (sub==3), 8*FW, 5*FH, expoDrOn ,DR_WEIGHT, DR_RIGHT ) ;

	lcd_puts_Pleft(6*FH,PSTR(STR_DR_SW1));
	editExpoVals( event, sub==4,5*FW, 6*FH, DR_DRSW1 , 0,0);
	lcd_puts_Pleft(7*FH,PSTR(STR_DR_SW2));
	editExpoVals( event, sub==5,5*FW, 7*FH, DR_DRSW2 , 0,0);

//int8_t   kViewR  = REG(g_model.expoData[s_expoChan].expo[expoDrOn][DR_EXPO][DR_RIGHT], -100, 100);  //NormR;
//int8_t   kViewL  = REG(g_model.expoData[s_expoChan].expo[expoDrOn][DR_EXPO][DR_LEFT], -100, 100);  //NormL;
//int8_t   wViewR  = REG(g_model.expoData[s_expoChan].expo[expoDrOn][DR_WEIGHT][DR_RIGHT]+100, 0, 100);  //NormWeightR+100;
//int8_t   wViewL  = REG(g_model.expoData[s_expoChan].expo[expoDrOn][DR_WEIGHT][DR_LEFT]+100, 0, 100);  //NormWeightL+100;

//#define WE_CHART	(WCHART-1)
//#define WE_CHARTl	(WCHARTl-1)

	lcd_vline(XD - (IS_EXPO_THROTTLE(s_expoChan) ? WCHART : 0), Y0 - WCHART, WCHART * 2);

	plotType = PLOT_BLACK ;

	drawFunction( XD, GRAPH_FUNCTION_EXPO ) ;

//	if (IS_EXPO_THROTTLE(s_expoChan))
//	{
//		for(uint8_t xv=0;xv<WE_CHARTl*2;xv++)
//		{
//	    uint16_t yv=2*expo(xv*(RESXu/WE_CHARTl)/2,kViewR) / (RESXu/WE_CHARTl);
//  	  yv = (yv * wViewR)/100;
//	    lcd_plot(X0l+xv-WE_CHARTl, 2*Y0l-yv);
//	    if((xv&3) == 0)
//			{
//        lcd_plot(X0l+xv-WE_CHARTl, 2*Y0l-1);
//        lcd_plot(X0l-WE_CHARTl   , Y0l+xv/2);
//    	}
//		}
//	}
//	else
//	{
//		for(uint8_t xv=0;xv<WE_CHARTl;xv++)
//		{
//		  uint16_t yv=expo(xv*(RESXu/WE_CHARTl),kViewR) / (RESXu/WE_CHARTl);
//		  yv = (yv * wViewR)/100;
//    	lcd_plot(X0l+xv, Y0l-yv);
//		  if((xv&3) == 0)
//			{
//    	  lcd_plot(X0l+xv, Y0l+0);
//      	lcd_plot(X0l  , Y0l+xv);
//		  }

//		  yv=expo(xv*(RESXu/WE_CHARTl),kViewL) / (RESXu/WE_CHARTl);
//		  yv = (yv * wViewL)/100;
//		  lcd_plot(X0l-xv, Y0l+yv);
//		  if((xv&3) == 0)
//			{
//	      lcd_plot(X0l-xv, Y0l+0);
//  	    lcd_plot(X0l  , Y0l-xv);
//    	}
//		}
//	}

	int16_t x512  = calibratedStick[s_expoChan];
	int16_t y512 = calcExpo( l_expoChan, x512 ) ;
  
	lcd_outdezAtt( 19*FW, 6*FH,x512*25/((signed) RESXu/4), 0 );
	lcd_outdezAtt( 14*FW, 1*FH,y512*25/((signed) RESXu/4), 0 );
	
	int8_t xv = (x512 * WCHART + RESX/2) / RESX + XD ;
  int8_t yv = Y0 - (y512 * WCHART + RESX/2) / RESX ;

	lcd_vline( xv, yv-3, 7 ) ;
	lcd_hline( xv-3, yv, 7 ) ;
	
	plotType = PLOT_XOR ;

//	int32_t y512 = 0;
//	if (IS_EXPO_THROTTLE(s_expoChan))
//	{
//    y512  = 2*expo((x512+RESX)/2,kViewR);
//    y512 = y512 * (wViewR / 4)/(100 / 4);
//    lcd_hline(X0l-WE_CHARTl, 2*Y0l-y512/(RESXu/WE_CHARTl),WE_CHARTl*2);
//    y512 /= 2;
//	}
//	else
//	{
//		y512  = expo(x512,(x512>0 ? kViewR : kViewL));
//		y512 = y512 * ((x512>0 ? wViewR : wViewL) / 4)/(100 / 4);

//		lcd_hline(X0l-WE_CHARTl, Y0l-y512/(int32_t)(RESXu/WE_CHARTl),WE_CHARTl*2);
//	}

}


uint8_t char2idx(char c)
{
	uint8_t ret ;
    for(ret=0;;ret++)
    {
        char cc= s_charTab[ret] ;
        if(cc==c) return ret;
        if(cc==0) return 0;
    }
}
char idx2char(uint8_t idx)
{
    if(idx < NUMCHARS) return s_charTab[idx] ;
    return ' ';
}

uint8_t DupIfNonzero = 0 ;
int8_t DupSub ;

void menuDeleteDupModel(uint8_t event)
{
    lcd_putsnAtt(1,2*FH, (char *)ModelNames[DupSub],sizeof(g_model.name),/*BSS*/0);
    lcd_putc(sizeof(g_model.name)*FW+FW,2*FH,'?');
		yesNoMenuExit( DupIfNonzero ? PSTR(STR_DUP_MODEL) : PSTR(STR_DELETE_MODEL) ) ;

//    uint8_t i;
    switch(event){
    case EVT_ENTRY:
        audioDefevent(AU_WARNING1);
        break;
    case EVT_KEY_FIRST(KEY_MENU):
        if ( DupIfNonzero )
        {
            message(PSTR(STR_DUPLICATING));
            if(eeDuplicateModel(DupSub))
            {
                audioDefevent(AU_MENUS);
                DupIfNonzero = 2 ;		// sel_editMode = false;
            }
            else audioDefevent(AU_WARNING1);
        }
        else
        {
						ee32_delete_model( DupSub-1 ) ;
            
//						i = g_eeGeneral.currModel;//loop to find next available model
//            while ( ! eeModelExists( i ) )
//						{
//							i--;
//              if(i>MAX_MODELS) i=MAX_MODELS-1;
//              if(i==g_eeGeneral.currModel)
//							{
//              	i=0;
//                break;
//              }
//            }
//            g_eeGeneral.currModel = i;
//            STORE_GENERALVARS;
//						ee32WaitLoadModel(g_eeGeneral.currModel) ;
//						putVoiceQueue( g_model.modelVoice + 260 ) ;
////            eeWaitComplete() ;
////            eeLoadModel(g_eeGeneral.currModel); //load default values
//            resetTimer();
        }
        killEvents(event);
        popMenu(true);
        pushMenu(menuProcModelSelect);
        break;
    case EVT_KEY_FIRST(KEY_EXIT):
        killEvents(event);
        popMenu(true);
        pushMenu(menuProcModelSelect);
        break;
    }
}

void dsmDisplayABLRFH()
{
	for ( uint32_t i = 0 ; i < 6 ; i += 1 )
	{
		uint16_t value = DsmABLRFH[i] ;
		lcd_putc( i*3*(FW+1), 7*FH, "ABLRFH"[i] ) ;
		if ( value == 0 )		// 0xFFFF + 1
		{
		  lcd_puts_P( (i+1)*3*(FW+1)-2-(2*FW), 7*FH, XPSTR("--") ) ;
		}
		else
		{
			if ( value > 1000 )
			{
				value = 1000 ;
			}
		  lcd_outdezAtt( (i+1)*3*(FW+1)-2, 7*FH, value - 1, 0 ) ;
		}
	}
}

void menuRangeBind(uint8_t event)
{
	static uint8_t timer ;
	uint8_t *ptrFlag ;

#ifdef PCBX9D
	ptrFlag = pxxFlag ? &pxxFlag : &pxxFlag_x ;
#else
	ptrFlag = &pxxFlag ;
#endif
	uint8_t flag = *ptrFlag & PXX_BIND ;
	lcd_puts_Pleft( 2*FH, (flag) ? PSTR(STR_6_BINDING) : PSTR(STR_RANGE_RSSI) ) ;
  if ( event == EVT_KEY_FIRST(KEY_EXIT) )
	{
    killEvents(event);
		*ptrFlag = 0 ;
		popMenu(false) ;
	}
//#ifdef PCBX9D
//	lcd_outhex4( 0, 7*FH, pxxFlag ) ;
//	lcd_outhex4( 25, 7*FH, pxxFlag_x ) ;
//#endif

#ifdef FRSKY
	if ( flag == 0 )
	{
		lcd_outdezAtt( 12 * FW, 4*FH, FrskyHubData[FR_RXRSI_COPY], DBLSIZE);
		if ( FrskyTelemetryType == 2 )		// DSM telemetry
		{
			dsmDisplayABLRFH() ;
		}
	}
#endif
	if ( --timer == 0 )
	{
  	audioDefevent(AU_WARNING2) ;
	}
}

//#ifndef DISABLE_PXX_SPORT
////#ifndef REVX
//void menuFailsafe(uint8_t event)
//{
//  SUBMENU( PSTR(STR_FAILSAFE), 16, { 0/*repeated...*/}) ;
	
//	uint8_t sub = mstate2.m_posVert ;
//	uint8_t t_pgOfs = evalOffset( sub, 6 ) ;
////  uint8_t current = 1 ;

//	uint8_t y ;
//	uint8_t k ;
//	for (uint8_t j=0 ; j<7 ; j += 1 )
//	{
//    y = (j+1) * FH ;
//    k = j + t_pgOfs ;
//		uint8_t attr = 0 ;
//	 	if(sub==k)
//		{
//			attr = INVERS ;
//			if ( event == EVT_KEY_BREAK(KEY_MENU) )
//			{
//				int16_t value = g_chans512[k] ;
//				value /= 10 ;
//				if ( value > 100 )
//				{
//					value = 100 ;					
//				}
//				if ( value < -100 )
//				{
//					value = -100 ;					
//				}
//				s_editMode = false ;
//				g_model.pxxFailsafe[k] = value ;
//			}
//			else
//			{
//        CHECK_INCDEC_H_MODELVAR( event, g_model.pxxFailsafe[k], -100, 100 ) ;
//			}
//		}
////		uint8_t y = j+1 ;
//		lcd_outdezAtt( 8*FW, y, g_model.pxxFailsafe[k], attr ) ;
//    putsChn( 0, y, k+1, 0 ) ; // show CHx
//	}
//}
//#endif



static void editTimer( uint8_t sub, uint8_t event )
{
	uint8_t subN ;
	uint8_t timer ;
	if ( sub < 7 )
	{
		subN = 0 ;
		timer = 0 ;
	}
	else
	{
		subN = 7 ;
		timer = 1 ;
	}
	t_TimerMode *ptm ;
	
	ptm = &g_model.timer[timer] ;

	uint8_t y = FH ;
	uint8_t blink = InverseBlink ;

  lcd_puts_Pleft( y, PSTR(STR_TIMER) ) ;
  lcd_putc( 5*FW, y, '1'+ timer ) ;
 	putsTime(12*FW-1, y, ptm->tmrVal,(sub==subN ? blink:0),(sub==subN ? blink:0) ) ;

#ifndef NOPOTSCROLL
//	    if(sub==subN && (s_editing	) )	// Use s_editing???
	    if(sub==subN)	// Use s_editing???
#else
//	    if(sub==subN && s_editMode )
	    if(sub==subN)
#endif
			{
				int16_t temp = 0 ;
//				StepSize = 60 ;
				CHECK_INCDEC_H_MODELVAR( event, temp, -60 ,60 ) ;
				ptm->tmrVal += temp ;
        if((int16_t)ptm->tmrVal < 0) ptm->tmrVal=0;
				
//				div_t qr ;

//				qr = div( ptConfig->tmrVal, 60 ) ;
//   		  int8_t sec=qr.rem ;
//	      int8_t min=qr.quot ;
//   	    switch (subSub)
//				{
//	 	      case 0:
//  	      {
//    	       CHECK_INCDEC_H_MODELVAR_0( min ,59);
//      	     ptConfig->tmrVal = sec + min*60;
//        	   break;
//	   	    }
// 		      case 1:
//    	    {
//      	     sec -= checkIncDec_hm( sec+2 ,1,62)-2;
//        	   ptConfig->tmrVal -= sec ;
//          	 if((int16_t)ptConfig->tmrVal < 0) ptConfig->tmrVal=0;
//	           break;
//  	     	}
//    		}
			}
			y += FH ;
//   		if((y+=FH)>7*FH) return y ;
//		}
		subN++ ;

//		if(t_pgOfs<subN) { //timer trigger source -> off, abs, stk, stk%, sw/!sw, !m_sw/!m_sw, chx(value > or < than tmrChVal), ch%
    	lcd_puts_Pleft(    y, PSTR(STR_TRIGGERA));
			uint8_t attr = 0 ;
    	if(sub==subN)
			{
   			attr = blink ;
    	  CHECK_INCDEC_MODELSWITCH( event, ptm->tmrModeA, 0, 1+2+24);
			}
    	putsTmrMode(10*FW,y,attr, timer, 1 ) ;
			y += FH ;
//   		if((y+=FH)>7*FH) return y ;
//		}
		subN++ ;
  	
//		if(t_pgOfs<subN) { //timer trigger source -> none, sw/!sw
  	  lcd_puts_Pleft(    y, PSTR(STR_TRIGGERB));
  	  attr = 0 ;
  	  if(sub==subN)
			{
	   		attr = blink ;
			}
			uint8_t doedit = attr ? EDIT_DR_SWITCH_MOMENT | EDIT_DR_SWITCH_EDIT : EDIT_DR_SWITCH_MOMENT ;
			ptm->tmrModeB = edit_dr_switch( 15*FW, y, ptm->tmrModeB, attr, doedit, event ) ;
			y += FH ;
//   		if((y+=FH)>7*FH) return y ;
//  	}
		subN++ ;
	 
//		if(t_pgOfs<subN)
//		{
  	  lcd_puts_Pleft( y, PSTR(STR_TIMER) );
  	  attr = 0 ;
  	  if(sub==subN)
			{
				attr = blink ; CHECK_INCDEC_H_MODELVAR_0( event, ptm->tmrDir,1) ;
			}
  	  lcd_putsAttIdx(  10*FW, y, PSTR(STR_COUNT_DOWN_UP), ptm->tmrDir, attr ) ;
			y += FH ;
//   		if((y+=FH)>7*FH) return y ;
//		}
		subN++ ;

//#ifndef GLOBAL_COUNTDOWN
////  	if(t_pgOfs<subN)
////		{
			uint8_t b = timer ? g_model.timer2Mbeep : g_model.timer1Mbeep ;
  	  (timer ? g_model.timer2Mbeep : g_model.timer1Mbeep) = onoffMenuItem_m( event, b, y, PSTR(STR_MINUTE_BEEP), sub ==subN ) ;
			y += FH ;
////   		if((y+=FH)>7*FH) return y ;
////			}
		subN++;

////  	if(t_pgOfs<subN)
////		{
			b = timer ? g_model.timer2Cdown : g_model.timer1Cdown ;
  	  (timer ? g_model.timer2Cdown : g_model.timer1Cdown) = onoffMenuItem_m( event, b, y, PSTR(STR_BEEP_COUNTDOWN), sub==subN  ) ;
			y += FH ;
////   		if((y+=FH)>7*FH) return y ;
////		}
		subN++;
//#endif

//		if(t_pgOfs<subN)
//		{
  	  lcd_puts_Pleft( y, XPSTR("Reset Switch"));
  	  attr = 0 ;
			if(sub==subN)
			{
				attr = blink ;
			}

			int16_t sw = (timer==0) ? g_model.timer1RstSw : g_model.timer2RstSw ;
			doedit = attr ? EDIT_DR_SWITCH_MOMENT | EDIT_DR_SWITCH_EDIT : EDIT_DR_SWITCH_MOMENT ;
			sw = edit_dr_switch( 15*FW, y, sw, attr, doedit, event ) ;
			if ( timer == 0 )
			{
				g_model.timer1RstSw = sw ;
			}
			else
			{
				g_model.timer2RstSw = sw ;
			}
			y += FH ;
//   		if((y+=FH)>7*FH) return y ;
//		}
//		subN++ ;

//	return y ;
}



  

void menuProcModel(uint8_t event)
{
	uint8_t need_bind_range = 0 ;

#ifdef PCBX9D
	uint8_t dataItems = 11 ;
#else
	uint8_t dataItems = 7 ;
#endif

#ifdef REVX
	if (g_model.protocol == PROTO_PPM)
	{
		dataItems += 1 ;
	}
#endif

	if (g_model.protocol == PROTO_PXX)
	{
#ifdef DISABLE_PXX_SPORT
//#ifdef REVX	
		dataItems -= 1 ;
#else	
		dataItems += 2 ;
#endif
		need_bind_range |= 1 ;
	}
	if (g_model.protocol == PROTO_DSM2)
	{
		if (g_model.sub_protocol == 3)
		{
			dataItems += 1 ;
			need_bind_range = 1 ;
		}
		else
		{
//			dataItems -= 1 ;
			need_bind_range = 4 ;
		}
	}
#ifdef PCBX9D
	if (g_model.xprotocol == PROTO_PXX)
	{
		dataItems += 3 ;
		need_bind_range |= 2 ;
	}
	if (g_model.xprotocol == PROTO_DSM2)
	{
		if (g_model.xsub_protocol == 3)
		{
			dataItems += 1 ;
			need_bind_range |= 2 ;
		}
		else
		{
			need_bind_range |= 8 ;
		}
	}
	if ( g_model.protocol == PROTO_OFF )
	{
		dataItems -= 4 ;
	}
	if ( g_model.xprotocol == PROTO_OFF )
	{
		dataItems -= 4 ;
	}
#endif

uint8_t blink = InverseBlink ;
static uint8_t columns = 0 ;

	TITLE( XPSTR("Protocol") ) ;
	static MState2 mstate2 ;


	if (SubMenuFromIndex)
	{
		event = mstate2.check(event,0,NULL,0,&columns,0,dataItems-1) ;
	}
	else
	{
		event = mstate2.check(event,e_Model,menuTabModel,DIM(menuTabModel),&columns,0,dataItems-1) ;
//		VARMENU(PSTR(STR_SETUP), menuTabModel, e_Model, dataItems, &columns ) ;
	}

#ifdef REVX	
//	VARMENU(PSTR(STR_SETUP), menuTabModel, e_Model, dataItems, &columns ) ;
//	MENU(PSTR(STR_SETUP), menuTabModel, e_Model, dataItems, {0,sizeof(g_model.name)-1,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,6,2,0/*repeated...*/});
#else	
//	VARMENU(PSTR(STR_SETUP), menuTabModel, e_Model, dataItems, &columns ) ;
//  MENU(PSTR(STR_SETUP), menuTabModel, e_Model, dataItems, {0,sizeof(g_model.name)-1,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,6,2,0/*repeated...*/});
#endif
//	if ( event == EVT_ENTRY_UP )
//	{ // From menuProcSelectMvoiceFile
//		if ( FileSelectResult == 1 )
//		{
//			copyFileName( g_model.modelVname, SelectedVoiceFileName ) ;
//			g_model.modelVoice = -1 ;
//	    eeDirty(EE_MODEL) ;		// Save it
//		}
//	}

	int8_t  sub    = mstate2.m_posVert;
	uint8_t subSub = mstate2.m_posHorz;
  uint8_t t_pgOfs ;
	columns = 0 ;

	t_pgOfs = evalOffset(sub, 7);

	uint8_t y = 1*FH;

//	lcd_outdezNAtt(13*FW,0,g_eeGeneral.currModel+1,INVERS+LEADING0,2);

	uint8_t subN = 1 ;

#ifndef PCBX9D
	if(t_pgOfs<subN)
	{
	  lcd_puts_Pleft( y, PSTR(STR_PPM2_START));
		if ( g_model.startPPM2channel == 0 )
		{
			lcd_putsAtt( 15*FW, y, PSTR(STR_FOLLOW), (sub==subN) ? INVERS : 0 ) ;
		}
		else
		{
			lcd_outdezAtt(  17*FW, y, g_model.startPPM2channel, (sub==subN) ? INVERS : 0 ) ;
		}
	  if(sub==subN) CHECK_INCDEC_H_MODELVAR_0( event, g_model.startPPM2channel, 17 ) ;
	  if((y+=FH)>7*FH) return ;
	} subN += 1 ;



	if(t_pgOfs<subN)
	{
		uint8_t attr = (sub==subN) ? INVERS : 0 ;
		lcd_puts_Pleft( y, PSTR(STR_PPM2_CHANNELS));
		uint8_t chans = g_model.ppm2NCH + 2 ;
		chans *= 2 ;
		if ( chans > 12 )
		{
			chans -= 13 ;
		}
		lcd_outdezAtt(  16*FW, y, (chans+4), attr ) ;
  	lcd_putsAtt( Lcd_lastPos, y, PSTR( STR_PPMCHANNELS ), attr ) ;
  	if(sub==subN)
		{
			CHECK_INCDEC_H_MODELVAR_0( event, chans, 12 ) ;
			if ( chans & 1 )	// odd
			{
				chans /= 2 ;
				chans += 7 ;
			}
			else
			{
				chans /= 2 ;
			}
			g_model.ppm2NCH = chans - 2 ;
		}
	  if((y+=FH)>7*FH) return ;
 	} subN += 1 ;
#endif

//	if(t_pgOfs<subN)
//	{
//		if(sub==subN)
//		{
//			columns = sizeof(g_model.name)-1 ;
//		}
//		editName( 11*FW-2, mstate2.m_posHorz, y, (uint8_t *)g_model.name, sizeof(g_model.name), sub==subN ? EE_MODEL : 0, event ) ;
//    if((y+=FH)>7*FH) return;
//	}subN++;
	
//	if(t_pgOfs<subN)
//	{
//		// modelVname
//		uint8_t attr = 0 ;
//    if(sub==subN)
//		{
//			if (event == EVT_KEY_LONG(KEY_MENU) )
//			{
////				if ( g_model.modelVoice == -1 )
////				{
////					putNamedVoiceQueue( g_model.modelVname, 0xC000 ) ;
////				}
////				else
////				{
////					putVoiceQueue( g_model.modelVoice + 260 ) ;
////				}
////  	  	s_editMode = 0 ;
////        killEvents(event);
////			}
////			if (event == EVT_KEY_BREAK(KEY_MENU) )
////			{
//       	pushMenu( menuProcSelectMvoiceFile ) ;				
//  	  	s_editMode = 0 ;
//        killEvents(event);
//			}
//			if ( mstate2.m_posHorz == 0 )
//			{
//				attr = InverseBlink ;
//  	    CHECK_INCDEC_H_MODELVAR( event, g_model.modelVoice, -1, 49 ) ;
//			}
//		}
//		if ( g_model.modelVoice == -1 )
//		{
//			if(sub==subN)
//			{
//				columns = VOICE_NAME_SIZE ;
//			}
//			uint8_t type = ( (sub==subN) && (mstate2.m_posHorz) ) ? EE_MODEL : 0 ;
//			editName( 9*FW-2, mstate2.m_posHorz-1, y, (uint8_t *)g_model.modelVname, sizeof(g_model.modelVname), type, event ) ;
//		}
//		else
//		{
//			if (event == EVT_KEY_BREAK(KEY_MENU) )
//			{
//				if(sub==subN)
//				{
//					putVoiceQueue( g_model.modelVoice + 260 ) ;
//					s_editMode = 0 ;
//				}
//			}
//		}
//    lcd_puts_Pleft( y, PSTR(STR_VOICE_INDEX) ) ;
//		if ( g_model.modelVoice == -1 )
//		{
//    	lcd_putcAtt( 7*FW+2, y, '>', attr) ;
//		}
//		else
//		{
//    	lcd_outdezAtt(  10*FW+2, y, (int16_t)g_model.modelVoice + 260 ,attr);
//		}
//    if((y+=FH)>7*FH) return;
//	}subN++;

//	if(t_pgOfs<subN)
//	{
//		g_model.thrTrim = onoffMenuItem_m( event, g_model.thrTrim, y, PSTR(STR_T_TRIM), sub==subN) ;
//    if((y+=FH)>7*FH) return;
//	}subN++;

//	if(t_pgOfs<subN) {
//		g_model.thrExpo = onoffMenuItem_m( event, g_model.thrExpo, y, PSTR(STR_T_EXPO), sub==subN) ;
//    if((y+=FH)>7*FH) return;
//	}subN++;

//	if(t_pgOfs<subN)
//	{
//    lcd_puts_Pleft(    y, PSTR(STR_TRIM_INC));
//    uint8_t attr = 0 ;
//		if(sub==subN)
//		{
//   		attr = InverseBlink ;
//    	CHECK_INCDEC_H_MODELVAR_0(event,g_model.trimInc,4) ;
//		}
//    lcd_putsAttIdx(  10*FW, y, PSTR(STR_TRIM_OPTIONS),g_model.trimInc, attr);
//    if((y+=FH)>7*FH) return;
//	}subN++;

//	if(t_pgOfs<subN)
//	{
//    lcd_puts_Pleft(    y, PSTR(STR_TRIM_SWITCH));
//    uint8_t attr = 0 ;
//    if(sub==subN) { attr = InverseBlink ; }
//		g_model.trimSw = edit_dr_switch( 9*FW, y, g_model.trimSw, attr, attr ? EDIT_DR_SWITCH_EDIT : 0, event ) ;
//    if((y+=FH)>7*FH) return;
//	} subN++ ;
	
//	if(t_pgOfs<subN)
//	{
//		g_model.extendedLimits = onoffMenuItem_m( event, g_model.extendedLimits, y, PSTR(STR_E_LIMITS), sub==subN) ;
//    if((y+=FH)>7*FH) return;
//	}subN++;

//	if(t_pgOfs<subN)
//	{
//		g_model.traineron = onoffMenuItem_m( event, g_model.traineron, y, PSTR(STR_Trainer), sub==subN) ;
//    if((y+=FH)>7*FH) return;
//	}subN++;

//	if(t_pgOfs<subN)
//	{
//  	uint8_t attr = PREC1 ;
//    lcd_puts_Pleft(    y, PSTR(STR_AUTO_LIMITS));
//    if(sub==subN) { attr = InverseBlink | PREC1 ; CHECK_INCDEC_H_MODELVAR_0( event, g_model.sub_trim_limit, 100 ) ; }
//    lcd_outdezAtt(  20*FW, y, g_model.sub_trim_limit, attr ) ;
//		if((y+=FH)>7*FH) return;
//	}subN++;

//  if(t_pgOfs<subN)
//	{
//    uint16_t states = g_model.modelswitchWarningStates ;
//    uint8_t b = 1 - (states & 1) ;
//		b = 1 - onoffMenuItem_m( event, b, y, PSTR(STR_SWITCH_WARN), sub==subN ) ;
//    g_model.modelswitchWarningStates = (states & ~1) | b ;
//    if((y+=FH)>7*FH) return;
//  }subN++;

//  if(t_pgOfs<subN)
//	{
//    lcd_puts_Pleft(    y, PSTR(STR_DEAFULT_SW));
// 		uint16_t states = g_model.modelswitchWarningStates >> 1 ;
        
//#ifdef PCBSKY
//		uint16_t index ;
//		uint8_t attr ;
		
//		index = oneSwitchText( HSW_ThrCt, states ) ;
//		attr = index >> 8 ;
//		index &= 0x00FF ;
//		lcd_putsAttIdx(12*FW, y, PSTR(SWITCHES_STR), index, attr ) ;
		
//		index = oneSwitchText( HSW_RuddDR, states ) ;
//		attr = index >> 8 ;
//		index &= 0x00FF ;
//		lcd_putsAttIdx(15*FW, y, PSTR(SWITCHES_STR), index, attr ) ;
		
//		index = oneSwitchText( HSW_ElevDR, states ) ;
//		attr = index >> 8 ;
//		index &= 0x00FF ;
//		lcd_putsAttIdx(18*FW, y, PSTR(SWITCHES_STR), index, attr ) ;

////    for(uint8_t i=0, q=1;i<8;q<<=1,i++)
////		{
////			lcd_putsnAtt((13+i)*FW, y, PSTR(STR_TRE012AG)+i,1,  (((uint8_t)states & q) ? INVERS : 0 ) );
////		}

//    if(sub==subN)
//		{
//			lcd_rect( 12*FW-1, y-1, 9*FW+2, 9 ) ;
//#endif
//#ifdef PCBX9D
//    for (uint8_t i=0 ; i<7 ; i += 1 )
//		{
//      lcd_putc( 11*FW+i*(2*FW), y, 'A'+i ) ;
//			lcd_putc( 12*FW+i*(2*FW), y, PSTR(HW_SWITCHARROW_STR)[states & 0x03] ) ;
//      states >>= 2 ;
//    }
//    if(sub==subN)
//		{
//			lcd_rect( 11*FW-1, y-1, 14*FW+2, 9 ) ;
//#endif
//      if (event==EVT_KEY_FIRST(KEY_MENU) || event==EVT_KEY_FIRST(BTN_RE))
//			{
//        killEvents(event);
////    		uint16_t states = g_model.modelswitchWarningStates & 1 ;
//#ifdef PCBSKY
//	      g_model.modelswitchWarningStates = (getCurrentSwitchStates() << 1 ) ;// states ;
//#endif
//#ifdef PCBX9D
//  		getMovedSwitch() ;	// loads switches_states
//extern uint16_t switches_states ;
//	    g_model.modelswitchWarningStates = ((switches_states & 0x3FFF) << 1 ) ;// states ;
//#endif
//        s_editMode = false ;
//        STORE_MODELVARS ;
//			}
//		}
//    if((y+=FH)>7*FH) return;
//  }subN++;

//#ifdef PCBSKY
//  if(t_pgOfs<subN)
//	{
//    lcd_puts_Pleft( y, PSTR(STR_DEAFULT_SW));
// 		uint16_t states = g_model.modelswitchWarningStates >> 1 ;
//		uint8_t attr ;
//		uint16_t index ;

//		index = oneSwitchText( HSW_ID0, states ) & 0x00FF ;
//		lcd_putsAttIdx(12*FW, y, PSTR(SWITCHES_STR), index, 0 ) ;

//		index = oneSwitchText( HSW_AileDR, states ) ;
//		attr = index >> 8 ;
//		index &= 0x00FF ;
//		lcd_putsAttIdx(15*FW, y, PSTR(SWITCHES_STR), index, attr ) ;
		
//		index = oneSwitchText( HSW_Gear, states ) ;
//		attr = index >> 8 ;
//		index &= 0x00FF ;
//		lcd_putsAttIdx(18*FW, y, PSTR(SWITCHES_STR), index, attr ) ;
//    if((y+=FH)>7*FH) return;
//  }subN++;
//#endif

//	if(t_pgOfs<subN)
//	{
//  	uint8_t attr = 0 ;
//    lcd_puts_Pleft( y, XPSTR("Throttle Off") ) ;
//    if(sub==subN) { attr = INVERS ; CHECK_INCDEC_H_MODELVAR_0( event, g_model.throttleIdle, 1 ) ; }
//    lcd_putsAttIdx( 16*FW, y, XPSTR("\005-100%   0%"),g_model.throttleIdle, attr ) ;
//		if((y+=FH)>7*FH) return;
//  }subN++;

//	if(t_pgOfs<subN)
//	{
//		g_model.useCustomStickNames = onoffMenuItem_m( event, g_model.useCustomStickNames, y, XPSTR("CustomStkNames"), sub==subN) ;
//    if((y+=FH)>7*FH) return;
//	}subN++;

//	if(t_pgOfs<subN) {
//  	uint8_t attr = 0 ;
//    lcd_puts_Pleft(    y, PSTR(STR_VOLUME_CTRL));
//    if(sub==subN) { attr = INVERS ; CHECK_INCDEC_H_MODELVAR_0( event, g_model.anaVolume, 7 ) ; }
//#ifdef PCBSKY
//    lcd_putsAttIdx( 17*FW, y, XPSTR("\003---P1 P2 P3 GV4GV5GV6GV7"),g_model.anaVolume, attr ) ;
//#endif
//#ifdef PCBX9D
//    lcd_putsAttIdx( 17*FW, y, XPSTR("\003---P1 P2 SL SR GV5GV6GV7"),g_model.anaVolume, attr ) ;
//#endif
//		if((y+=FH)>7*FH) return;
//	}subN++;

//	if(t_pgOfs<subN)
//	{
//    lcd_puts_Pleft(    y, PSTR(STR_BEEP_CENTRE));
//    for(uint8_t i=0;i<7;i++) lcd_putsnAtt((10+i)*FW, y, PSTR(STR_RETA123)+i,1, (((subSub)==i) && (sub==subN)) ? BLINK : ((g_model.beepANACenter & (1<<i)) ? INVERS : 0 ) );
//    if(sub==subN)
//		{
//			columns = 6 ;
//        if((event==EVT_KEY_FIRST(KEY_MENU)) || P1values.p1valdiff) {
//            killEvents(event);
//            s_editMode = false;
//            g_model.beepANACenter ^= (1<<(subSub));
//            STORE_MODELVARS;
////            eeWaitComplete() ;
//        }
//    }
//    if((y+=FH)>7*FH) return;
//	}subN++;

	uint8_t protocol = g_model.protocol ;
	
#ifdef PCBX9D
	if(t_pgOfs<subN)
	{
		uint8_t value ;
		uint8_t newvalue ;
		value = (protocol == PROTO_OFF) ? 0 : 1 ;
		newvalue = onoffMenuItem_m( event, value, y, XPSTR("Internal"), sub==subN ) ;
		if ( newvalue != value )
		{
			value = newvalue ? 0 : PROTO_OFF ;
		}
		else
		{
			value = protocol ;
		}
		if ( protocol != value )
		{
			 g_model.protocol = protocol = value ;
		}
    if((y+=FH)>7*FH) return;
	} subN++;

	if ( protocol != PROTO_OFF )
	{
#endif

		if(t_pgOfs<subN)
		{
  	  lcd_puts_Pleft(    y, PSTR(STR_PROTO));//sub==2 ? INVERS:0);
	//    lcd_putsnAtt(  6*FW, y, PSTR(PROT_STR)+PROT_STR_LEN*g_model.protocol,PROT_STR_LEN,(sub==subN && subSub==0 ? (s_editMode ? BLINK : INVERS):0));
			lcd_putsAttIdx(  6*FW, y, PSTR(STR_PROT_OPT), protocol, (sub==subN && subSub==0 ? blink:0) );
  	  if ( protocol == PROTO_PPM )
			{
				uint8_t x ;
			  lcd_puts_Pleft( y, PSTR(STR_21_USEC) );
				x = 10*FW ;
				if ( g_model.ppmNCH > 12 )
				{
					g_model.ppmNCH = 0 ;		// Correct if wrong from DSM
				}
//				uint8_t channels = g_model.ppmNCH * 2 + 8 ;
				uint8_t channels = (g_model.ppmNCH + 4) * 2 ;
				if ( channels > 16 )
				{
					channels -= 13 ;
				}
	//			if ( (protocol == PROTO_DSM2) && ( g_model.sub_protocol == DSM_9XR ) )
	//			{
	//				channels = g_model.ppmNCH ;
	//			}
				uint8_t attr = LEFT ;
				if ( (sub==subN) && ( subSub==1 ) )
				{
					attr |= blink ;
				}
				lcd_outdezAtt(  x, y, channels, attr ) ;
  			lcd_putsAtt( Lcd_lastPos, y, PSTR( STR_PPMCHANNELS ), attr ) ;
				lcd_outdezAtt(  x+7*FW-1, y,  (g_model.ppmDelay*50)+300, (sub==subN && subSub==2 ? blink:0));
  	  }
  	  else if ( ( protocol == PROTO_PXX) || ( ( protocol == PROTO_DSM2) && (g_model.sub_protocol != DSM_9XR) ) )
  	  {
			    lcd_puts_Pleft( y, PSTR(STR_13_RXNUM) );
  	      lcd_outdezAtt(  21*FW, y,  g_model.pxxRxNum, (sub==subN && subSub==1 ? blink:0));
  	  }
  	  else if ( ( protocol == PROTO_DSM2) && ( g_model.sub_protocol == DSM_9XR ) )
  	  {
			    lcd_puts_Pleft( y, XPSTR("\013Chans") );
  	      lcd_outdezAtt(  21*FW, y,  g_model.ppmNCH, (sub==subN && subSub==1 ? blink:0));
  	  }

		  if(sub==subN)
			{
			 columns = (protocol == PROTO_PPM) ? 2 : 1 ;
			 if (s_editing )
			 {
			
#ifdef PCBX9D
			  uint8_t prot_max = PROT_MAX - 1 ;		// No DSM on internal
#else
			  uint8_t prot_max = PROT_MAX ;
#endif
  	    switch (subSub)
				{
  	     	case 0:
  	        CHECK_INCDEC_H_MODELVAR_0(event, g_model.protocol, prot_max ) ;
  	      break;
  	      case 1:
  	        if (protocol == PROTO_PPM)
						{
							uint8_t chans = g_model.ppmNCH + 2 ;
							chans *= 2 ;
							if ( chans > 12 )
							{
								chans -= 13 ;
							}
 	          	CHECK_INCDEC_H_MODELVAR_0(event, chans, 12 ) ;
							if ( chans & 1 )	// odd
							{
								chans /= 2 ;
								chans += 7 ;
							}
							else
							{
								chans /= 2 ;
							}
							g_model.ppmNCH = chans - 2 ;
						}
  	  			else if ( protocol == PROTO_PXX)
						{
  	        	CHECK_INCDEC_H_MODELVAR_0(event,g_model.pxxRxNum,124);
						}
						else// must be DSM
						{
							if ( g_model.sub_protocol == DSM_9XR )
							{
  	          	CHECK_INCDEC_H_MODELVAR(event,g_model.ppmNCH,6,14) ;
							}
							else
							{
  	        		CHECK_INCDEC_H_MODELVAR_0(event,g_model.pxxRxNum,124);
							}
						}
  	      break;
  	      case 2:
  	        if (g_model.protocol == PROTO_PPM)
  	          CHECK_INCDEC_H_MODELVAR(event,g_model.ppmDelay,-4,10);
  	       break;
  	    }
			 }
  	  }
  	  if((y+=FH)>7*FH) return;
	}subN++;

  	if(g_model.protocol == PROTO_PPM)
		{
			if(t_pgOfs<subN)
  	  {
  	      lcd_puts_Pleft(    y, PSTR(STR_PPMFRAME_MSEC));
					uint8_t attr = PREC1 ;
  	      if(sub==subN) { attr = INVERS | PREC1 ; CHECK_INCDEC_H_MODELVAR(event,g_model.ppmFrameLength,-20,20) ; }
  	      lcd_outdezAtt(  14*FW-1, y, (int16_t)g_model.ppmFrameLength*5 + 225, attr ) ;
  	  	if((y+=FH)>7*FH) return;
  	  }
			subN += 1 ;

#ifdef REVX
			if(t_pgOfs<subN)
  	  {
  	    lcd_puts_Pleft(    y, XPSTR(" PPM Drive"));
				uint8_t attr = 0 ;
  		  uint8_t x ;
  	  	x = g_model.ppmOpenDrain ;
  	    if(sub==subN) { attr = INVERS ; CHECK_INCDEC_H_MODELVAR_0(event,g_model.ppmOpenDrain,1) ; }
  		  lcd_putsAttIdx( 12*FW, y, XPSTR("\011OpenDrainFullDrive"), g_model.ppmOpenDrain, attr );
				if ( x != g_model.ppmOpenDrain )
				{
					module_output_active() ;		// Changes drive as well
				}
				if((y+=FH)>7*FH) return;
  	  }
			subN += 1 ;
#endif
		}

#ifndef DISABLE_PXX_SPORT
//#ifndef REVX
		if (protocol == PROTO_PXX)
		{
			if(t_pgOfs<subN)
			{
		  	uint8_t attr = 0 ;
				lcd_puts_Pleft( y, PSTR(STR_TYPE) ) ;
				uint8_t ltype = g_model.sub_protocol ;
		    if(sub==subN) { attr = INVERS ; CHECK_INCDEC_H_MODELVAR_0( event, ltype, 2 ) ; }
				g_model.sub_protocol = ltype ;
  		  lcd_putsAttIdx( 10*FW, y, XPSTR("\003D16D8 LRP"), ltype, attr );
		    if((y+=FH)>7*FH) return ;
			}subN++;
		}
#endif
	
		if (protocol == PROTO_DSM2)
		{
			if(t_pgOfs<subN)
			{
		    lcd_puts_Pleft(    y, PSTR(STR_DSM_TYPE));
  		  int8_t x ;
  	  	x = g_model.sub_protocol ;
#ifndef REVX
//		    if ( x > 2 ) x = 2 ;
#endif
  		  g_model.sub_protocol = x ;
	//        lcd_putsnAtt(10*FW,y, PSTR(DSM2_STR)+DSM2_STR_LEN*(x),DSM2_STR_LEN, (sub==subN ? (s_editMode ? BLINK : INVERS):0));
		    lcd_putsAttIdx( 10*FW, y, XPSTR(DSM2_STR), x, (sub==subN ? blink:0) ) ;
#ifdef PCBSKY
  		  if(sub==subN) CHECK_INCDEC_H_MODELVAR_0(event,g_model.sub_protocol,3);
#else        
				if(sub==subN) CHECK_INCDEC_H_MODELVAR_0(event,g_model.sub_protocol,3);
				if ( g_model.sub_protocol != x )
				{
					g_model.ppmNCH = ( g_model.sub_protocol == 3 ) ? 12 : 6 ;
				}
#endif
  	  	if((y+=FH)>7*FH) return ;
			} subN += 1 ;
		}
			 
#ifndef DISABLE_PXX_SPORT
//#ifndef REVX
		if (protocol == PROTO_PXX)
		{
			if(t_pgOfs<subN)
			{
		  	uint8_t attr = 0 ;
				lcd_puts_Pleft( y, PSTR(STR_COUNTRY) ) ;
				uint8_t lcountry = g_model.country ;
		    if(sub==subN) { attr = INVERS ; CHECK_INCDEC_H_MODELVAR_0( event, lcountry, 2 ) ;}
				g_model.country = lcountry ;
  		  lcd_putsAttIdx( 10*FW, y, XPSTR("\003AmeJapEur"), lcountry, attr );
		    if((y+=FH)>7*FH) return ;
			}subN++;
			
//			if(t_pgOfs<subN)
//			{
//				lcd_puts_Pleft( y, PSTR(STR_SP_FAILSAFE) ) ;
//  	    if(sub==subN)
//				{
//					lcd_char_inverse( 1*FW, y, 8*FW, 0 ) ;
//					if ( event==EVT_KEY_BREAK(KEY_MENU))
//					{
//						pushMenu(menuFailsafe) ;
//					}
//				}
//		    if((y+=FH)>7*FH) return ;
//			}subN++;
		}
#endif
	
#ifdef PCBX9D
	}
#endif

#ifdef PCBX9D
	if ( g_model.protocol != PROTO_OFF )
	{
#endif
		if(t_pgOfs<subN)
		{
			uint8_t attr = 0 ;
  		lcd_puts_Pleft( y, PSTR(STR_PPM_1ST_CHAN));
  		if(sub==subN) { attr = INVERS ; CHECK_INCDEC_H_MODELVAR_0( event, g_model.startChannel, 16 ) ; }
			lcd_outdezAtt(  14*FW, y, g_model.startChannel + 1, attr ) ;
		  if((y+=FH)>7*FH) return ;
		} subN += 1 ;
#ifdef PCBX9D
	}
#endif

  if(g_model.protocol == PROTO_PPM)
	{
		if(t_pgOfs<subN) {
		  	uint8_t attr = 0 ;
		    lcd_puts_Pleft(    y, PSTR(STR_SHIFT_SEL));
		    if(sub==subN) { attr = INVERS ; CHECK_INCDEC_H_MODELVAR_0(event,g_model.pulsePol,1);}
		    lcd_putsAttIdx( 11*FW, y, PSTR(STR_POS_NEG),g_model.pulsePol,attr );
		    if((y+=FH)>7*FH) return;
		}subN++;
	}
	
	if ( need_bind_range & 5 )
	{
#ifdef DISABLE_PXX_SPORT
//#ifdef REVX
    if(g_model.protocol == PROTO_PXX)
		{
			if(t_pgOfs<subN)
			{			
    		lcd_putsAtt(0,    y, PSTR(STR_SEND_RX_NUM), (sub==subN ? INVERS:0));

      	if(sub==subN )
      	{
					if ( event==EVT_KEY_LONG(KEY_MENU))
					{
	    	    //send reset code
  	  	    pxxFlag = PXX_BIND ;
					}
					s_editMode = 0 ;
      	}
		  	if((y+=FH)>7*FH) return ;
	 		} subN += 1 ;
		}
		else
    {
#endif
			if ( need_bind_range & 1 )
			{
				if(t_pgOfs<subN)
				{
					lcd_puts_Pleft( y, PSTR(STR_BIND) ) ;
      		if(sub==subN)
					{
						lcd_char_inverse( FW, y, 4*FW, 0 ) ;
						if ( event==EVT_KEY_LONG(KEY_MENU))
						{
    		    	pxxFlag = PXX_BIND ;		    	//send bind code or range check code
							pushMenu(menuRangeBind) ;
						}
						s_editMode = 0 ;
					}
				  if((y+=FH)>7*FH) return ;
		 		} subN += 1 ;
			}
			
			if(t_pgOfs<subN)
			{
				lcd_puts_Pleft( y, PSTR(STR_RANGE) ) ;
      	if(sub==subN)
				{
					lcd_char_inverse( FW, y, 11*FW, 0 ) ;
					if ( event==EVT_KEY_LONG(KEY_MENU))
					{
    	    	pxxFlag = PXX_RANGE_CHECK ;		    	//send bind code or range check code
						pushMenu(menuRangeBind) ;
					}
					s_editMode = 0 ;
				}
			  if((y+=FH)>7*FH) return ;
		 	} subN += 1 ;
#ifdef DISABLE_PXX_SPORT
//#ifdef REVX
    }
#endif
	}

#ifdef PCBX9D

	protocol = g_model.xprotocol ;
	
	if(t_pgOfs<subN)
	{
		uint8_t value ;
		uint8_t newvalue ;
		value = (protocol == PROTO_OFF) ? 0 : 1 ;
		newvalue = onoffMenuItem_m( event, value, y, XPSTR("External"), sub==subN ) ;
		if ( newvalue != value )
		{
			value = newvalue ? 0 : PROTO_OFF ;
		}
		else
		{
			value = protocol ;
		}
		if ( protocol != value )
		{
			 g_model.xprotocol = protocol = value ;
		}
    if((y+=FH)>7*FH) return;
	} subN++;

	if ( protocol != PROTO_OFF )
	{
	
	
	
		if(t_pgOfs<subN)
		{
  	  lcd_puts_Pleft(    y, PSTR(STR_PROTO));//sub==2 ? INVERS:0);
	//    lcd_putsnAtt(  6*FW, y, PSTR(PROT_STR)+PROT_STR_LEN*g_model.protocol,PROT_STR_LEN,(sub==subN && subSub==0 ? (s_editMode ? BLINK : INVERS):0));
			lcd_putsAttIdx(  6*FW, y, PSTR(STR_PROT_OPT), protocol, (sub==subN && subSub==0 ? blink:0) );
  	  if ( protocol == PROTO_PPM )
			{
				uint8_t x ;
			  lcd_puts_Pleft( y, PSTR(STR_21_USEC) );
				x = 10*FW ;
				if ( g_model.xppmNCH > 12 )
				{
					g_model.xppmNCH = 0 ;		// Correct if wrong from DSM
				}
//				uint8_t channels = g_model.xppmNCH * 2 + 8 ;
				uint8_t channels = (g_model.xppmNCH + 4) * 2 ;
				if ( channels > 16 )
				{
					channels -= 13 ;
				}
	//			if ( (protocol == PROTO_DSM2) && ( g_model.sub_protocol == DSM_9XR ) )
	//			{
	//				channels = g_model.ppmNCH ;
	//			}
				uint8_t attr = LEFT ;
				if ( (sub==subN) && ( subSub==1 ) )
				{
					attr |= blink ;
				}
				lcd_outdezAtt(  x, y, channels, attr ) ;
  			lcd_putsAtt( Lcd_lastPos, y, PSTR( STR_PPMCHANNELS ), attr ) ;
				lcd_outdezAtt(  x+7*FW-1, y,  (g_model.xppmDelay*50)+300, (sub==subN && subSub==2 ? blink:0));
  	  }
  	  else if ( ( protocol == PROTO_PXX) || ( protocol == PROTO_DSM2) )
  	  {
			    lcd_puts_Pleft( y, PSTR(STR_13_RXNUM) );
  	      lcd_outdezAtt(  21*FW, y,  g_model.xPxxRxNum, (sub==subN && subSub==1 ? blink:0));
  	  }

		  if(sub==subN)
			{
			 columns = (protocol == PROTO_PPM) ? 2 : 1 ;
			 if (s_editing )
			 {
			  uint8_t prot_max = PROT_MAX ;
  	    switch (subSub)
				{
  	     	case 0:
  	        CHECK_INCDEC_H_MODELVAR_0(event, g_model.xprotocol, prot_max ) ;
  	      break;
  	      case 1:
  	        if (g_model.xprotocol == PROTO_PPM)
						{
							uint8_t chans = g_model.xppmNCH + 2 ;
							chans *= 2 ;
							if ( chans > 12 )
							{
								chans -= 13 ;
							}
  	          CHECK_INCDEC_H_MODELVAR_0(event, chans, 12 ) ;
							if ( chans & 1 )	// odd
							{
								chans /= 2 ;
								chans += 7 ;
							}
							else
							{
								chans /= 2 ;
							}
							g_model.xppmNCH = chans - 2 ;
						}
  	  			else if ( ( protocol == PROTO_PXX) || ( protocol == PROTO_DSM2) )
						{
  	          	CHECK_INCDEC_H_MODELVAR_0(event,g_model.xPxxRxNum,124);
						}
  	      break;
  	      case 2:
  	        if (g_model.xprotocol == PROTO_PPM)
  	          CHECK_INCDEC_H_MODELVAR(event,g_model.xppmDelay,-4,10);
  	       break;
  	    }
			 }
  	  }
  	  if((y+=FH)>7*FH) return;
	}subN++;

  	if(g_model.xprotocol == PROTO_PPM)
		{
			if(t_pgOfs<subN)
  	  {
  	      lcd_puts_Pleft(    y, PSTR(STR_PPMFRAME_MSEC));
					uint8_t attr = PREC1 ;
  	      if(sub==subN) { attr = INVERS | PREC1 ; CHECK_INCDEC_H_MODELVAR(event,g_model.xppmFrameLength,-20,20) ; }
  	      lcd_outdezAtt(  14*FW-1, y, (int16_t)g_model.xppmFrameLength*5 + 225, attr ) ;
  	  	if((y+=FH)>7*FH) return;
  	  }
			subN += 1 ;
		}

		if (protocol == PROTO_PXX)
		{
			if(t_pgOfs<subN)
			{
		  	uint8_t attr = 0 ;
				lcd_puts_Pleft( y, PSTR(STR_TYPE) ) ;
				uint8_t ltype = g_model.xsub_protocol ;
		    if(sub==subN) { attr = INVERS ; CHECK_INCDEC_H_MODELVAR_0( event, ltype, 2 ) ; }
				g_model.xsub_protocol = ltype ;
  		  lcd_putsAttIdx( 10*FW, y, XPSTR("\003D16D8 LRP"), ltype, attr );
		    if((y+=FH)>7*FH) return ;
			}subN++;
		}
	
		if (protocol == PROTO_DSM2)
		{
			if(t_pgOfs<subN)
			{
		    lcd_puts_Pleft(    y, PSTR(STR_DSM_TYPE));
  		  int8_t x ;
  	  	x = g_model.xsub_protocol ;
		    if ( x > 2 ) x = 2 ;
  		  g_model.xsub_protocol = x ;
	//        lcd_putsnAtt(10*FW,y, PSTR(DSM2_STR)+DSM2_STR_LEN*(x),DSM2_STR_LEN, (sub==subN ? (s_editMode ? BLINK : INVERS):0));
		    lcd_putsAttIdx( 10*FW, y, XPSTR(DSM2_STR), x, (sub==subN ? blink:0) ) ;
  		  if(sub==subN) CHECK_INCDEC_H_MODELVAR_0(event,g_model.xsub_protocol,2);
  	  	if((y+=FH)>7*FH) return ;
			} subN += 1 ;
		}
			 
		if (protocol == PROTO_PXX)
		{
			if(t_pgOfs<subN)
			{
		  	uint8_t attr = 0 ;
				lcd_puts_Pleft( y, PSTR(STR_COUNTRY) ) ;
				uint8_t lcountry = g_model.xcountry ;
		    if(sub==subN) { attr = INVERS ; CHECK_INCDEC_H_MODELVAR_0( event, lcountry, 2 ) ;}
				g_model.xcountry = lcountry ;
  		  lcd_putsAttIdx( 10*FW, y, XPSTR("\003AmeJapEur"), lcountry, attr );
		    if((y+=FH)>7*FH) return ;
			}subN++;
			
		}
	}

	if ( g_model.xprotocol != PROTO_OFF )
	{
		if(t_pgOfs<subN)
		{
			uint8_t attr = 0 ;
  		lcd_puts_Pleft( y, PSTR(STR_PPM_1ST_CHAN));
  		if(sub==subN) { attr = INVERS ; CHECK_INCDEC_H_MODELVAR_0( event, g_model.xstartChannel, 16 ) ; }
			lcd_outdezAtt(  14*FW, y, g_model.xstartChannel + 1, attr ) ;
		  if((y+=FH)>7*FH) return ;
		} subN += 1 ;
	}

	if ( protocol != PROTO_OFF )
	{
  	if(g_model.xprotocol == PROTO_PPM)
		{
			if(t_pgOfs<subN) {
			  	uint8_t attr = 0 ;
			    lcd_puts_Pleft(    y, PSTR(STR_SHIFT_SEL));
			    if(sub==subN) { attr = INVERS ; CHECK_INCDEC_H_MODELVAR_0(event,g_model.xpulsePol,1);}
			    lcd_putsAttIdx( 11*FW, y, PSTR(STR_POS_NEG),g_model.xpulsePol,attr );
			    if((y+=FH)>7*FH) return;
			}subN++;
		}
	}
#endif

#ifdef PCBX9D
	if ( need_bind_range & 2 )
	{
		if(t_pgOfs<subN)
		{
			lcd_puts_Pleft( y, PSTR(STR_BIND) ) ;
      if(sub==subN)
			{
				lcd_char_inverse( FW, y, 4*FW, 0 ) ;
				if ( event==EVT_KEY_LONG(KEY_MENU))
				{
    	    pxxFlag_x = PXX_BIND ;		    	//send bind code or range check code
					pushMenu(menuRangeBind) ;
				}
				s_editMode = 0 ;
			}
			if((y+=FH)>7*FH) return ;
		} subN += 1 ;
	}		
	if ( need_bind_range & 0x0A )
	{
		if(t_pgOfs<subN)
		{
			lcd_puts_Pleft( y, PSTR(STR_RANGE) ) ;
      if(sub==subN)
			{
				lcd_char_inverse( FW, y, 11*FW, 0 ) ;
				if ( event==EVT_KEY_LONG(KEY_MENU))
				{
    	    pxxFlag_x = PXX_RANGE_CHECK ;		    	//send bind code or range check code
					pushMenu(menuRangeBind) ;
				}
				s_editMode = 0 ;
			}
    }		
	}
#endif
}


void putsTrimMode( uint8_t x, uint8_t y, uint8_t phase, uint8_t idx, uint8_t att )
{
  int16_t v = getRawTrimValue(phase, idx);

  if (v > TRIM_EXTENDED_MAX)
	{
    uint8_t p = v - TRIM_EXTENDED_MAX - 1;
    if (p >= phase) p += 1 ;
    lcd_putcAtt(x, y, '0'+p, att);
  }
  else
	{
#ifdef FIX_MODE
  	lcd_putsAttIdx( x, y, PSTR(STR_1_RETA), idx, att ) ;
#else
  	lcd_putsAttIdx( x, y, PSTR(STR_1_RETA), *(modn12x3+g_eeGeneral.stickMode*4+idx)-1, att ) ;
#endif
//  	lcd_putsAttIdx( x, y, "\001RETA", idx, att ) ;
  }
}


void menuPhaseOne(uint8_t event)
{
  PhaseData *phase = &g_model.phaseData[s_currIdx] ;
  SUBMENU( PSTR(STR_FL_MODE), 5, { 0, 3, 0, 0, 5 /*, 0*/} ) ;
  lcd_putc( 8*FW, 0, '1'+s_currIdx ) ;

  int8_t sub = mstate2.m_posVert;
  int8_t editMode = s_editMode;

//#ifdef FIX_MODE
//	lcd_puts_P( 10*FW, 3*FH, &(PSTR(STR_1_RETA)[1]) ) ;
//#endif
  
	for (uint8_t i = 0 ; i < 5 ; i += 1 )
	{
    uint8_t y = (i+2) * FH;
		uint8_t attr = (sub==i ? InverseBlink : 0);
    
		switch(i)
		{
      case 0 : // switch
				lcd_puts_Pleft( y, PSTR(STR_SWITCH) ) ;
				phase->swtch = edit_dr_switch( 10*FW, y, phase->swtch, attr, attr ? EDIT_DR_SWITCH_EDIT : 0, event ) ;
			break;

      case 1 : // trims
				lcd_puts_Pleft( y, PSTR(STR_TRIMS) ) ;
        for ( uint8_t t = 0 ; t<NUM_STICKS ; t += 1 )
				{
          putsTrimMode( (10+t)*FW, y, s_currIdx+1, t, (mstate2.m_posHorz==t) ? attr : 0 ) ;
          if (attr && mstate2.m_posHorz==t && ((editMode>0) || P1values.p1valdiff))
					{
            int16_t v = phase->trim[t] ;
            if (v < TRIM_EXTENDED_MAX)
						{
							v = TRIM_EXTENDED_MAX;
						}
            v = checkIncDec16( event, v, TRIM_EXTENDED_MAX, TRIM_EXTENDED_MAX+MAX_MODES, EE_MODEL ) ;
            if (checkIncDec_Ret)
						{
              if (v == TRIM_EXTENDED_MAX) v = 0 ;
  						phase->trim[t] = v ;
            }
          }
        }
      break;
      
			case 2 : // fadeIn
				lcd_puts_Pleft( y, XPSTR("Fade In") ) ;
    		lcd_outdezAtt(14*FW, y, phase->fadeIn * 5, attr | PREC1 ) ;
  			if( attr ) CHECK_INCDEC_H_MODELVAR_0( event, phase->fadeIn, 15 ) ;
			break ;
      
			case 3 : // fadeOut
				lcd_puts_Pleft( y, XPSTR("Fade Out") ) ;
		    lcd_outdezAtt(14*FW, y, phase->fadeOut * 5, attr | PREC1 ) ;
  			if( attr ) CHECK_INCDEC_H_MODELVAR_0( event, phase->fadeOut, 15 ) ;
			break ;
			
			case 4 : // Name
				editName( 11*FW-2, mstate2.m_posHorz, y, (uint8_t *)phase->name, sizeof(phase->name), attr ? EE_MODEL : 0, event ) ;
//				lcd_puts_Pleft( y, XPSTR("Name") ) ;
//		    lcd_putsnAtt( 10*FW, y, phase->name, 6, 0 ) ;
//				if ( attr )
//				{
//					lcd_rect( 10*FW-2, y-1, 6*FW+4, 9 ) ;
//					lcd_char_inverse( (10+mstate2.m_posHorz)*FW, y, 1*FW, s_editMode ) ;
//	    		if(s_editMode)
//					{
//						uint8_t pos = mstate2.m_posHorz ;
//      		  uint8_t v = phase->name[pos] ;
//						if ( v )
//						{
//      		  	v = char2idx( v ) ;
//						}
//      		  CHECK_INCDEC_H_MODELVAR_0( event,v ,NUMCHARS-1);
//      		  v = idx2char(v) ;
//						if ( phase->name[pos] != v )
//						{
//      		  	phase->name[pos] = v ;
////    					eeDirty( EE_MODEL ) ;				// Do here or the last change is not stored in ModelNames[]
//						}
//					}
//				}
			break ;
	  }
	}
}





void menuModelPhases(uint8_t event)
{
	uint32_t i ;
  uint8_t attr ;
	
	TITLE(PSTR(STR_MODES));
	static MState2 mstate2;
	static const uint8_t mstate_tab[] = { 0 } ;
  
	if (SubMenuFromIndex)
	{
		event = mstate2.check(event,0,NULL,0,mstate_tab,DIM(mstate_tab)-1,7-1) ;
	}
	else
	{
		event = mstate2.check(event,e_Phases,menuTabModel,DIM(menuTabModel),mstate_tab,DIM(mstate_tab)-1,7-1) ;
//	SIMPLE_MENU( PSTR(STR_MODES), menuTabModel, e_Phases, 7 ) ;
	}
	
	uint8_t  sub    = mstate2.m_posVert ;
//	evalOffset(sub, 6) ;

  switch (event)
	{
    case EVT_KEY_FIRST(KEY_MENU) :
    case EVT_KEY_FIRST(BTN_RE) :
			if ( sub > 0 ) //&& sub <= MAX_MODES )
			{
        s_currIdx = sub - 1 ;
        killEvents(event);
        pushMenu(menuPhaseOne) ;
    	}
		break;
  }
    
	lcd_puts_Pleft( 1*FH, PSTR(STR_SP_FM0) ) ;
  {
    for ( i = 1 ; i <= 4 ; i += 1 )
    {
#ifdef FIX_MODE
	  	lcd_putsAttIdx( (15+i)*FW-2, 1*FH, PSTR(STR_1_RETA), i-1, 0 ) ;
#else
  		lcd_putsAttIdx( (15+i)*FW-2, 1*FH, PSTR(STR_1_RETA), *(modn12x3+g_eeGeneral.stickMode*4+(i-1))-1, 0 ) ;
#endif
    }
  }

  for ( i=1 ; i<=MAX_MODES ; i += 1 )
	{
    uint8_t y=(i+1)*FH ;
		PhaseData *p = &g_model.phaseData[i-1] ;
    attr = (i == sub) ? INVERS : 0 ;
    lcd_puts_Pleft( y, PSTR(STR_SP_FM) ) ;
    lcd_putc( 3*FW, y, '0'+i ) ;
		lcd_putsnAtt( 4*FW+3, y, g_model.phaseData[i-1].name, 6, /*BSS*/ attr ) ;
    putsDrSwitches( 11*FW, y, p->swtch, attr ) ;
    for ( uint8_t t = 0 ; t < NUM_STICKS ; t += 1 )
		{
			putsTrimMode( (16+t)*FW-2, y, i, t, attr ) ;
		}
		if ( p->fadeIn || p->fadeOut )
		{
	    lcd_putcAtt( 20*FW+1, y, '*', attr ) ;
		}
//    lcd_outdezAtt(18*FW+4, y, p->fadeIn * 5, attr | PREC1 ) ;
//    lcd_outdezAtt(21*FW+1, y, p->fadeOut * 5, attr | PREC1 ) ;
	}	 
	i = getFlightPhase() + 1 ;
	lcd_rect( 0, i*FH-1, 4*FW+1, 9 ) ;
	// Cosmetic tidy up due to inverse of top line
	if ( i == 1 )
	{
    lcd_hline( 0, FH-1, 4*FW+2 );
	}
}





#ifndef NO_HELI
void menuProcHeli(uint8_t event)
{
	TITLE(PSTR(STR_HELI_SETUP));
	static MState2 mstate2;
	static const uint8_t mstate_tab[] = { 0 } ;
    
	if (SubMenuFromIndex)
	{
		event = mstate2.check(event,0,NULL,0,mstate_tab,DIM(mstate_tab)-1,7-1) ;
	}
	else
	{
		event = mstate2.check(event,e_Heli,menuTabModel,DIM(menuTabModel),mstate_tab,DIM(mstate_tab)-1,7-1) ;
//		MENU(PSTR(STR_HELI_SETUP), menuTabModel, e_Heli, 7, {0 /*repeated*/});
	}

int8_t  sub    = mstate2.m_posVert;

evalOffset(sub, 7);

uint8_t y = 1*FH;
  uint8_t b ;
  uint8_t attr ;

uint8_t subN = 1;
    b = g_model.swashType ;
    lcd_puts_Pleft(    y, PSTR(STR_SWASH_TYPE));
		attr = 0 ;
    if(sub==subN) {attr = InverseBlink ; CHECK_INCDEC_H_MODELVAR_0(event,b,SWASH_TYPE_NUM); g_model.swashType = b ; }
    lcd_putsAttIdx( 17*FW, y, PSTR(SWASH_TYPE_STR),b,attr );
    if((y+=FH)>7*FH) return;
subN++;

    lcd_puts_Pleft(    y, PSTR(STR_COLLECTIVE));
    putsChnRaw(14*FW, y, g_model.swashCollectiveSource,  sub==subN ? INVERS : 0);
    if(sub==subN) CHECK_INCDEC_H_MODELVAR_0(event, g_model.swashCollectiveSource, NUM_SKYXCHNRAW);
    if((y+=FH)>7*FH) return;
subN++;

    lcd_puts_Pleft(    y, PSTR(STR_SWASH_RING));
    lcd_outdezAtt(14*FW, y, g_model.swashRingValue,  LEFT|(sub==subN ? INVERS : 0));
    if(sub==subN) CHECK_INCDEC_H_MODELVAR_0(event, g_model.swashRingValue, 100);
    if((y+=FH)>7*FH) return;
subN++;

		g_model.swashInvertELE = hyphinvMenuItem_m( event, g_model.swashInvertELE, y, PSTR(STR_ELE_DIRECTION), sub==subN ) ;
    if((y+=FH)>7*FH) return;
subN++;

		g_model.swashInvertAIL = hyphinvMenuItem_m( event, g_model.swashInvertAIL, y, PSTR(STR_AIL_DIRECTION), sub==subN ) ;
    if((y+=FH)>7*FH) return;
subN++;

		g_model.swashInvertCOL = hyphinvMenuItem_m( event, g_model.swashInvertCOL, y, PSTR(STR_COL_DIRECTION), sub==subN ) ;
//    if((y+=FH)>7*FH) return;

}
#endif

//const char *ModelPopList = PSTR(STR_MODEL_POPUP) ;

//#define XSTR_MODEL_POPUP    "EDIT\0SELECT\0COPY\0MOVE\0DELETE\0BACKUP\0RESTORE"

const char *BackResult ;

void menuProcModelSelect(uint8_t event)
{
  static MState2 mstate2;
  TITLE(PSTR(STR_MODELSEL));

  int8_t subOld  = mstate2.m_posVert;
	
	if ( !PopupData.PopupActive )
	{
		RotaryState = ROTARY_MENU_UD ;
  	mstate2.check_submenu_simple(event,MAX_MODELS-1) ;
	}

#ifdef PCBX9D
    lcd_puts_Pleft(  0, PSTR(STR_11_FREE));
    lcd_outdez(  18*FW, 0, EeFsGetFree());
#endif

//  DisplayScreenIndex(e_ModelSelect, DIM(menuTabModel), INVERS);

  int8_t  sub    = mstate2.m_posVert;
  static uint8_t sel_editMode;
  if ( DupIfNonzero == 2 )
  {
      sel_editMode = false ;
      DupIfNonzero = 0 ;
  }
  
	if(sub-s_pgOfs < 1)        s_pgOfs = max(0,sub-1);
  else if(sub-s_pgOfs >4 )  s_pgOfs = min(MAX_MODELS-6,sub-4);
  for(uint8_t i=0; i<6; i++)
	{
    uint8_t y=(i+2)*FH;
    uint8_t k=i+s_pgOfs;
    lcd_outdezNAtt(  3*FW, y, k+1, ((sub==k) ? INVERS : 0) + LEADING0,2);
    if(k==g_eeGeneral.currModel) lcd_putc(1,  y,'*');
    lcd_putsn_P(  4*FW, y, (char *)ModelNames[k+1],sizeof(g_model.name) ) ;
#ifdef PCBX9D
		if (ModelNames[k+1][0])
 		{
    	lcd_outdez( 20*FW, y, eeFileSize(k+1) ) ;
		}
#endif
		
		if ( (sub==k) && (sel_editMode ) )
		{
			lcd_rect( 0, y-1, 125, 9 ) ;
		}
//    lcd_putsnAtt(  4*FW, y, (char *)buf,sizeof(buf),/*BSS|*/((sub==k) ? (sel_editMode ? INVERS : 0 ) : 0));
  }

	if ( PopupData.PopupActive )
	{
//		uint8_t count = (g_eeGeneral.currModel == mstate2.m_posVert) ? 0x07 : 0x0F ; 
//		uint8_t count ;
		
		uint8_t mask ;
		if ( g_eeGeneral.currModel == mstate2.m_posVert )
		{
			mask = 0x2F ;
		}
		else
		{
			mask = ( eeModelExists( mstate2.m_posVert ) == 0 ) ?  0x42 :  0x3E ;
		}

		uint8_t popaction = doPopup( PSTR(STR_MODEL_POPUP), mask, 8, event ) ;
//		count = popupDisplay( PSTR(STR_MODEL_POPUP), mask, 8 ) ;
		
//		uint8_t popaction = popupProcess( event, count - 1 ) ;
//		uint8_t popidx = PopupIdx ;
//		lcd_char_inverse( 4*FW, (popidx+1)*FH, 6*FW, 0 ) ;
//		popidx = popTranslate( popidx, mask ) ;
		
  	if ( popaction == POPUP_SELECT )
		{
			uint8_t popidx = PopupData.PopupSel ;
			if ( popidx == 0 )	// edit
			{
				RotaryState = ROTARY_MENU_LR ;
				chainMenu(menuProcModel) ;
			}
			if ( popidx == 1 )	// select
			{
        g_eeGeneral.currModel = mstate2.m_posVert;
				WatchdogTimeout = 200 ;		// 2 seconds
        ee32WaitLoadModel(g_eeGeneral.currModel); //load default values
				checkSwitches() ;
  			checkTHR();
				SportStreamingStarted = 0 ;
				if ( g_model.modelVoice == -1 )
				{
					putNamedVoiceQueue( g_model.modelVname, 0xC000 ) ;
				}
				else
				{
					putVoiceQueue( g_model.modelVoice + 260 ) ;
				}
        resetTimer();
				VoiceCheckFlag |= 2 ;// Set switch current states
        STORE_GENERALVARS;
				if ( PopupData.PopupActive == 2 ) chainMenu(menuProcModel) ;
			}
			else if ( popidx == 4 )		// Delete
			{
//        s_editMode = false;
//        s_noHi = NO_HI_LEN;
//  	    if(g_eeGeneral.currModel != mstate2.m_posVert)
//				{ // Don't allow current model to be deleted, now not selectable!
       	killEvents(event);
       	DupIfNonzero = 0 ;
				DupSub = sub + 1 ;
       	pushMenu(menuDeleteDupModel);
//				}
			}
			else if( popidx == 2 )	// copy
			{
				{
 	        DupIfNonzero = 1 ;
 	        DupSub = sub + 1 ;
 	        pushMenu(menuDeleteDupModel);//menuProcExpoAll);
				}
			}
			else if( popidx == 5 )	// backup
			{
				WatchdogTimeout = 200 ;		// 2 seconds
				BackResult = ee32BackupModel( mstate2.m_posVert+1 ) ;
				AlertType = MESS_TYPE ;
				AlertMessage = BackResult ;
			}
			else if( popidx == 6 )	// restore
			{
				RestoreIndex = mstate2.m_posVert+1 ;
       	pushMenu( menuProcRestore ) ;				
			}
			else // Move
			{
 	    	sel_editMode = true ;
			}
		}
//		else if ( popaction == POPUP_EXIT )
//		{
//			killEvents( event ) ;
//			PopupData.PopupActive = 0 ;
//		}
//		s_moveMixIdx = s_currMixIdx ;
	}
	else
	{
  	switch(event)
  	{
  	//case  EVT_KEY_FIRST(KEY_MENU):
	  	case  EVT_KEY_FIRST(KEY_EXIT):
  	    if(sel_editMode)
				{
 	        sel_editMode = false;
//  	        audioDefevent(AU_MENUS);
//  	        killEvents(event);
//						ee32WaitLoadModel(g_eeGeneral.currModel = mstate2.m_posVert) ;
//						putVoiceQueue( g_model.modelVoice + 260 ) ;
	//          eeWaitComplete();    // Wait to load model if writing something
	//          eeLoadModel(g_eeGeneral.currModel = mstate2.m_posVert);
//  	        resetTimer();
//  	        STORE_GENERALVARS;
	//          eeWaitComplete();
	//          STORE_MODELVARS;
	//          eeWaitComplete();
//  	        break;
  	    }
      break ;

	  	case  EVT_KEY_FIRST(KEY_LEFT):
  		case  EVT_KEY_FIRST(KEY_RIGHT):
  	    if(g_eeGeneral.currModel != mstate2.m_posVert)
  	    {
          killEvents(event);
					PopupData.PopupIdx = 0 ;
					PopupData.PopupActive = 2 ;
//  	        killEvents(event);
	//          g_eeGeneral.currModel = mstate2.m_posVert;
	//          ee32WaitLoadModel(g_eeGeneral.currModel); //load default values
	//					putVoiceQueue( g_model.modelVoice + 260 ) ;
	////          eeWaitComplete();    // Wait to load model if writing something
	////          eeLoadModel(g_eeGeneral.currModel);
	//          resetTimer();
	//          STORE_GENERALVARS;
	////          eeWaitComplete();
//  	        audioDefevent(AU_WARNING2);
  	    }
				else
				{
					RotaryState = ROTARY_MENU_LR ;
		      if(event==EVT_KEY_FIRST(KEY_LEFT))  chainMenu(menuProcModelIndex);//{killEvents(event);popMenu(true);}

//	#ifndef NO_TEMPLATES
//	#if GVARS
//		      if(event==EVT_KEY_FIRST(KEY_LEFT))  chainMenu(menuProcGlobals);//{killEvents(event);popMenu(true);}
//	#else
//  		    if(event==EVT_KEY_FIRST(KEY_LEFT))  chainMenu(menuProcTemplates);//{killEvents(event);popMenu(true);}
//	#endif
//	//#elif defined(FRSKY)
//	//      if(event==EVT_KEY_FIRST(KEY_LEFT))  chainMenu(menuProcTelemetry);//{killEvents(event);popMenu(true);}
//	#else
//	#if GVARS
//  	  	  if(event==EVT_KEY_FIRST(KEY_LEFT))  chainMenu(menuProcGlobals);//{killEvents(event);popMenu(true);}
//	#else
//  	    	if(event==EVT_KEY_FIRST(KEY_LEFT))  chainMenu(menuProcSafetySwitches);//{killEvents(event);popMenu(true);}
//	#endif
//	#endif
		      if(event==EVT_KEY_FIRST(KEY_RIGHT)) chainMenu(menuProcModel);
  	    //      if(event==EVT_KEY_FIRST(KEY_EXIT))  chainMenu(menuProcModelSelect);
				}
 	    break;
  		
			case  EVT_KEY_FIRST(KEY_MENU) :
			case  EVT_KEY_BREAK(BTN_RE) :
				if(sel_editMode)
				{
  	    	sel_editMode = false ;
				}
				else
				{
          killEvents(event);
					PopupData.PopupIdx = 0 ;
					PopupData.PopupActive = 1 ;
				}	 
  	    s_editMode = 0 ;
//  	    audioDefevent(AU_MENUS);
  	  break;
  	
			case  EVT_KEY_LONG(BTN_RE) :
			case  EVT_KEY_LONG(KEY_EXIT):  // make sure exit long exits to main
  	    popMenu(true);
      break;
//  	case  EVT_KEY_LONG(KEY_MENU):
//  	    if(sel_editMode){

//  	        DupIfNonzero = 1 ;
//  	        DupSub = sub + 1 ;
//  	        pushMenu(menuDeleteDupModel);//menuProcExpoAll);

//  	        //        message(PSTR("Duplicating model"));
//  	        //        if(eeDuplicateModel(sub)) {
//  	        //          audioDefevent(AU_MENUS);
//  	        //          sel_editMode = false;
//  	        //        }
//  	        //        else audioDefevent(AU_WARNING1);
//  	    }
//  	    break;

  		case EVT_ENTRY:
  	    sel_editMode = false;
				PopupData.PopupActive = 0 ;
  	    mstate2.m_posVert = g_eeGeneral.currModel ;
    	break;
  	}
	}

  if(sel_editMode && subOld!=sub)
	{
		ee32SwapModels( subOld+1, sub+1 ) ;

		if ( sub == g_eeGeneral.currModel )
		{
			g_eeGeneral.currModel = subOld ;
  	  STORE_GENERALVARS ;     //eeWriteGeneral();
		}
		else if ( subOld == g_eeGeneral.currModel )
		{
			g_eeGeneral.currModel = sub ;
  	  STORE_GENERALVARS ;     //eeWriteGeneral();
		}
  }
}


//const char *menuWhenDone = PSTR(STR_MENU_DONE) ;


void menuProcDiagCalib(uint8_t event)
{
  //    scroll_disabled = 1; // make sure we don't flick out of the screen
//  SIMPLE_MENU(PSTR(STR_CALIBRATION), menuTabDiag, e_Calib, 2);
	TITLE(PSTR(STR_CALIBRATION));
	static MState2 mstate2;
//	if (SubMenuFromIndex)
//	{
		mstate2.check_simple(event,0,NULL,0,2-1) ;
//	}
//	else
//	{
//		mstate2.check_simple(event,e_Calib,menuTabDiag,DIM(menuTabDiag),2-1) ;
//	}

  //    int8_t  sub    = mstate2.m_posVert ;
  mstate2.m_posVert = 0; // make sure we don't scroll or move cursor here

#ifdef PCBSKY
  for(uint8_t i=0; i<7; i++)
#endif
#ifdef PCBX9D
  for(uint8_t i=0; i<8; i++)
#endif
	{ //get low and high vals for sticks and trims
    int16_t vt = anaIn(i);
    Xmem.Cal_data.loVals[i] = min(vt,Xmem.Cal_data.loVals[i]);
    Xmem.Cal_data.hiVals[i] = max(vt,Xmem.Cal_data.hiVals[i]);
    if(i>=4)
		{
#ifdef PCBX9D
	    if(i < 6)			
#endif
				Xmem.Cal_data.midVals[i] = (Xmem.Cal_data.loVals[i] + Xmem.Cal_data.hiVals[i])/2;
		}
    //if(i>=4) midVals[i] = (loVals[i] + hiVals[i])/2;
  }

  //    if(sub==0)
  //        idxState = 0;

  scroll_disabled = Xmem.Cal_data.idxState; // make sure we don't scroll while calibrating

  switch(event)
  {
	  case EVT_ENTRY:
      Xmem.Cal_data.idxState = 0;
    break;

  	case EVT_KEY_BREAK(KEY_MENU):
      Xmem.Cal_data.idxState++;
      if(Xmem.Cal_data.idxState==3)
      {
        audioDefevent(AU_MENUS);
        STORE_GENERALVARS;     //eeWriteGeneral();
        Xmem.Cal_data.idxState = 0;
      }
    break;
  }

    switch(Xmem.Cal_data.idxState)
  {
  	case 0:
      //START CALIBRATION
      //[MENU]
      lcd_puts_Pleft( 3*FH, PSTR(STR_MENU_TO_START) ) ;//, 15, sub>0 ? INVERS : 0);
    break;

	  case 1: //get mid
      //SET MIDPOINT
      //[MENU]
      
#ifdef PCBSKY
  for(uint8_t i=0; i<7; i++)
#endif
#ifdef PCBX9D
  for(uint8_t i=0; i<8; i++)
#endif
      {
          Xmem.Cal_data.loVals[i] =  15000;
          Xmem.Cal_data.hiVals[i] = -15000;
          Xmem.Cal_data.midVals[i] = anaIn(i);
      }
      lcd_puts_Pleft( 2*FH, PSTR(STR_SET_MIDPOINT) ) ;//, 12, sub>0 ? INVERS : 0);
      lcd_puts_P(3*FW, 3*FH, PSTR(STR_MENU_DONE) ) ;//, 16, sub>0 ? BLINK : 0);
    break;

	  case 2:
      //MOVE STICKS/POTS
      //[MENU]
			StickScrollAllowed = 0 ;

        for(uint8_t i=0; i<7; i++)
            if(abs(Xmem.Cal_data.loVals[i]-Xmem.Cal_data.hiVals[i])>50) {
                g_eeGeneral.calibMid[i]  = Xmem.Cal_data.midVals[i];
                int16_t v = Xmem.Cal_data.midVals[i] - Xmem.Cal_data.loVals[i];
                g_eeGeneral.calibSpanNeg[i] = v - v/64 ;
                v = Xmem.Cal_data.hiVals[i] - Xmem.Cal_data.midVals[i];
                g_eeGeneral.calibSpanPos[i] = v - v/64;
            }
  		g_eeGeneral.chkSum = evalChkSum() ;
#ifdef PCBX9D
			{
  			int16_t v = Xmem.Cal_data.midVals[7] - Xmem.Cal_data.loVals[7] ;
  			g_eeGeneral.x9dcalibMid = Xmem.Cal_data.midVals[7] ;
  			g_eeGeneral.x9dcalibSpanNeg = v - v/64 ;
        v = Xmem.Cal_data.hiVals[7] - Xmem.Cal_data.midVals[7] ;
  			g_eeGeneral.x9dcalibSpanPos = v - v/64 ;
			}
#endif
      lcd_puts_Pleft( 2*FH, PSTR(STR_MOVE_STICKS) ) ; //, 16, sub>0 ? INVERS : 0);
      lcd_puts_P(3*FW, 3*FH, PSTR(STR_MENU_DONE) ) ; //, 16, sub>0 ? BLINK : 0);
    break;
  }

  doMainScreenGrphics();
}

uint16_t Current ;
uint32_t Current_sum ;
uint8_t Current_count ;

#ifdef PCBSKY
#define NUM_ANA_ITEMS		3
#endif
#ifdef PCBX9D
#define NUM_ANA_ITEMS		2
#endif


void menuProcDiagAna(uint8_t event)
{
	register uint32_t i ;
//  SIMPLE_MENU(PSTR(STR_ANA), menuTabDiag, e_Ana, NUM_ANA_ITEMS ) ;
	TITLE(PSTR(STR_ANA));
	static MState2 mstate2;
//	if (SubMenuFromIndex)
//	{
		mstate2.check_simple(event,0,NULL,0,NUM_ANA_ITEMS-1) ;
//	}
//	else
//	{
//		mstate2.check_simple(event,e_Ana,menuTabDiag,DIM(menuTabDiag),NUM_ANA_ITEMS-1) ;
//	}

#ifdef PCBSKY
 #ifdef REVB    
	Current_sum += Current_current ;
	if ( ++Current_count > 49 )
	{
		Current = Current_sum / 50 ;
		Current_sum = 0 ;
		Current_count = 0 ;
	}
 #endif
#endif

  int8_t  sub    = mstate2.m_posVert ;
	StickScrollAllowed = 0 ;
  for(i=0; i<8; i++)
  {
    uint8_t y=i*FH;
    lcd_putc( 4*FW-3, y, 'A' ) ;
    lcd_putc( 5*FW-3, y, '1'+i ) ;
    lcd_outhex4( 6*FW, y,anaIn(i));
		uint8_t index = i ;
		if ( i < 4 )
		{
 			index = stickScramble[g_eeGeneral.stickMode*4+i] ;
 		}
    if(i<7)  lcd_outdezAtt(15*FW, y, (int32_t)calibratedStick[index]*1000/1024, PREC1);
    if(i==7)
		{
			 putsVBat(14*FW,y,(sub==1 ? InverseBlink : 0)|PREC1);
#ifdef PCBX9D
    	lcd_outhex4( 6*FW, y,anaIn(8));
#endif
		}
  }

#ifdef PCBSKY
 #ifdef REVB    
  lcd_putc( 18*FW, 2*FH, 'A' ) ;
  lcd_putc( 19*FW, 2*FH, '9' ) ;
  lcd_outhex4( 17*FW, 3*FH,Analog_values[9]);
 #endif
#endif

#ifdef PCBX9D
#ifdef REVPLUS
  lcd_putc( 18*FW, 2*FH, 'A' ) ;
  lcd_putc( 19*FW, 2*FH, '9' ) ;
  lcd_outhex4( 17*FW, 3*FH,anaIn(7));
  lcd_outdezAtt(20*FW, 4*FH, (int32_t)calibratedStick[7]*1000/1024, PREC1);
  lcd_putc( 18*FW, 5*FH, 'A' ) ;
  lcd_putc( 19*FW, 5*FH, '1' ) ;
  lcd_putc( 20*FW, 5*FH, '0' ) ;
  lcd_outhex4( 17*FW, 6*FH,anaIn(9));
  lcd_outdezAtt(20*FW, 7*FH, (int32_t)calibratedStick[8]*1000/1024, PREC1);
#else
  lcd_putc( 18*FW, 5*FH, 'A' ) ;
  lcd_putc( 19*FW, 5*FH, '9' ) ;
  lcd_outhex4( 17*FW, 6*FH,anaIn(7));
  lcd_outdezAtt(20*FW, 7*FH, (int32_t)calibratedStick[7]*1000/1024, PREC1);
#endif
#endif
  if(sub==1)
  {
    scroll_disabled = 1;
		if ( s_editMode )
		{
	    CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.vBatCalib, -127, 127);
		}
  }
#ifdef PCBSKY
  if(sub==2)
	{
		if ( s_editMode )
		{
	    CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.current_calib, -49, 49 ) ;
		}
	}
  scroll_disabled = 1;
	lcd_puts_Pleft( 5*FH, XPSTR("\022mA"));
  lcd_outdezAtt( 21*FW, 6*FH, Current/10 , (sub==2 ? InverseBlink : 0) ) ;
#endif
}


void menuProcDiagKeys(uint8_t event)
{
//  SIMPLE_MENU(PSTR(STR_DIAG), menuTabDiag, e_Keys, 1);
	TITLE(PSTR(STR_DIAG));
	static MState2 mstate2;
//	if (SubMenuFromIndex)
//	{
		mstate2.check_simple(event,0,NULL,0,1-1) ;
//	}
//	else
//	{
//		mstate2.check_simple(event,e_Keys,menuTabDiag,DIM(menuTabDiag),1-1) ;
//	}

  uint8_t x ;

#ifdef PCBX9D

#ifdef REV9E
	x = 7*FW;
#else  
	x = 8*FW;
#endif	// REV9E
  for(uint8_t i=0; i<8; i++)
	{
    uint8_t y=i*FH; //+FH;
		lcd_putsAttIdx( x, y, PSTR(HW_SWITCHES_STR), i, 0 ) ;
		uint32_t pos = switchPosition( i ) ;
		lcd_putc(x+2*FW, y, PSTR(HW_SWITCHARROW_STR)[pos] ) ;
	}

#ifdef REV9E
	x = 11*FW - 2;
  for(uint8_t i=0; i<8; i++)
	{
    uint8_t y=i*FH; //+FH;
		lcd_putsAttIdx( x, y, PSTR(HW_SWITCHES_STR), i+8, 0 ) ;
		uint32_t pos = switchPosition( i+8 ) ;
		lcd_putc(x+2*FW, y, PSTR(HW_SWITCHARROW_STR)[pos] ) ;
	}
  x=14*FW;
  for(uint8_t i=0; i<2; i++)
	{
    uint8_t y=i*FH; //+FH;
		lcd_putsAttIdx( x, y, PSTR(HW_SWITCHES_STR), i+16, 0 ) ;
		uint32_t pos = switchPosition( i+16 ) ;
		lcd_putc(x+2*FW, y, PSTR(HW_SWITCHARROW_STR)[pos] ) ;
	}
#endif	// REV9E

#endif

#ifdef PCBSKY
  x = 7*FW;
  for(uint8_t i=0; i<9; i++)
  {
    uint8_t y=i*FH; //+FH;
		uint8_t displayType = 0 ;
    if(i>(SW_ID0-SW_BASE_DIAG)) y-=FH; //overwrite ID0
    uint8_t t=keyState((EnumKeys)(SW_BASE_DIAG+i));
		switch ( i )
		{
			case 0 :	// THR
				if ( g_eeGeneral.switchMapping & USE_THR_3POS )
				{
					t = switchPosition( HSW_Thr3pos0 ) ;
					displayType = 1 ;
				}
			break ;
			case 1 :	// RUD
				if ( g_eeGeneral.switchMapping & USE_RUD_3POS )
				{
					t = switchPosition( HSW_Rud3pos0 ) ;
					displayType = 1 ;
				}
			break ;
			case 2 :	// ELE
				if ( g_eeGeneral.switchMapping & USE_ELE_3POS )
				{
					t = switchPosition( HSW_Ele3pos0 ) ;
					displayType = 1 ;
				}
			break ;
			case 6 :	// AIL
				if ( g_eeGeneral.switchMapping & USE_AIL_3POS )
				{
					t = switchPosition( HSW_Ail3pos0 ) ;
					displayType = 1 ;
				}
			break ;
			case 7 :	// GEA
				if ( g_eeGeneral.switchMapping & USE_GEA_3POS )
				{
					t = switchPosition( HSW_Gear3pos0 ) ;
					displayType = 1 ;
				}
			break ;
		}
    putsDrSwitches(x,y,i+1,0); //ohne off,on
		if ( displayType == 0 )
		{
    	lcd_putcAtt(x+FW*4+2,  y,t+'0',t ? INVERS : 0);
		}
		else
		{
			lcd_putc(x+FW*4+2, y, PSTR(HW_SWITCHARROW_STR)[t] ) ;
		}
  }
#endif

  x=0;
  for(uint8_t i=0; i<6; i++)
  {
    uint8_t y=(5-i)*FH+2*FH;
    bool t=keyState((EnumKeys)(KEY_MENU+i));
   	lcd_putsAttIdx(  x, y, PSTR(STR_KEYNAMES),i,0);
    lcd_putcAtt(x+FW*5+2,  y,t+'0',t);
  }

  x=14*FW;
  lcd_putsn_P(x, 3*FH,PSTR(STR_TRIM_M_P),7);
  for(uint8_t i=0; i<4; i++)
  {
    uint8_t y=i*FH+FH*4;
    lcd_img(    x,       y, sticks,i,0);
    bool tm=keyState((EnumKeys)(TRM_BASE+2*i));
    bool tp=keyState((EnumKeys)(TRM_BASE+2*i+1));
    lcd_putcAtt(x+FW*4,  y, tm+'0',tm ? INVERS : 0);
    lcd_putcAtt(x+FW*6,  y, tp+'0',tp ? INVERS : 0);
  }
}

void menuProcDiagVers(uint8_t event)
{
	TITLE(PSTR(STR_VERSION));
	static MState2 mstate2;
//	if (SubMenuFromIndex)
//	{
		mstate2.check_simple(event,0,NULL,0,1-1) ;
//	}
//	else
//	{
//		mstate2.check_simple(event,e_Vers,menuTabDiag,DIM(menuTabDiag),1-1) ;
//	}

    lcd_puts_Pleft( 2*FH,stamp4 );
    lcd_puts_Pleft( 3*FH,stamp1 );
    lcd_puts_Pleft( 4*FH,stamp2 );
    lcd_puts_Pleft( 5*FH,stamp3 );
    lcd_puts_Pleft( 6*FH,stamp5 );
		
#ifdef PCBSKY
#ifndef REVX
		lcd_puts_Pleft( 7*FH, XPSTR("Co Proc"));
    lcd_outhex4( 10*FW-3, 7*FH, (Coproc_valid << 8 ) + Coproc_read ) ;
#endif
 #endif
}


#ifdef PCBSKY
#ifdef REVX
#define NUM_SETUP1_ITEMS	13
#define NUM_SETUP1_PG1		7
#else
#define NUM_SETUP1_ITEMS	14
#define NUM_SETUP1_PG1		8
#endif
#endif
#ifdef PCBX9D
#ifdef REVPLUS
#define NUM_SETUP1_ITEMS	7
#define NUM_SETUP1_PG1		6
#else
#define NUM_SETUP1_ITEMS	6
#define NUM_SETUP1_PG1		5
#endif
#endif

//void menuProcSetup1(uint8_t event)
//{
//#ifdef PCBSKY
//	uint8_t x = g_eeGeneral.switchMapping ;
//#endif
//#ifdef PCBSKY
//	uint8_t num = NUM_SETUP1_ITEMS + 2 ;
//	if ( x & ( USE_ELE_3POS | USE_ELE_6POS ) )
//	{
//		num += 2 ;
//		if ( x & ( USE_THR_3POS | USE_RUD_3POS ) )
//		{
//			num -= 1 ;
//		}
//	}	 
//	if ( x & ( USE_AIL_3POS | USE_GEA_3POS ) )
//	{
//		num -= 1 ;
//	}
//#else
//	uint8_t num = NUM_SETUP1_ITEMS ;
//#endif

//  MENU(PSTR(STR_RADIO_SETUP2), menuTabDiag, e_Setup1, num, {0/*, 0*/});
	
//	int8_t  sub    = mstate2.m_posVert;
//	if ( sub >= num )
//	{
//		mstate2.m_posVert = sub = num-1 ;
//	}
////	uint8_t subSub = mstate2.m_posHorz;
////	bool    edit;
////	uint8_t blink ;

////	evalOffset(sub, 5);
//	uint8_t y = 1*FH;
//	uint8_t current_volume ;
//	uint8_t subN ;

//	if ( sub < NUM_SETUP1_PG1 )
//	{
//		subN = 1;
//		current_volume = g_eeGeneral.volume ;
//  	lcd_puts_Pleft(    y, PSTR(STR_VOLUME));
//		lcd_outdezAtt( PARAM_OFS+2*FW, y, current_volume, (sub==subN) ? INVERS : 0 ) ;
//  	if(sub==subN)
//		{
//			CHECK_INCDEC_H_GENVAR_0(event,current_volume,NUM_VOL_LEVELS-1);
//			if ( current_volume != g_eeGeneral.volume )
//			{
//				setVolume( g_eeGeneral.volume = current_volume ) ;
//			}
//		}
//  	if((y+=FH)>7*FH) return;
//		subN++;
	
//	// audio start by rob
//  	  if(s_pgOfs<subN) {
//  	      lcd_puts_P(0, y,PSTR(STR_SPEAKER_PITCH));
//  	      lcd_outdezAtt(PARAM_OFS+2*FW,y,g_eeGeneral.speakerPitch,(sub==subN ? INVERS : 0) );
//  	      if(sub==subN) {
//  	          CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.speakerPitch, 1, 100);
//  	      }
//  	      if((y+=FH)>7*FH) return;
//  	  }subN++;
	
//  	  if(s_pgOfs<subN) {
//  	      lcd_puts_P(0, y,PSTR(STR_HAPTICSTRENGTH));
//  	      lcd_outdezAtt(PARAM_OFS+2*FW,y,g_eeGeneral.hapticStrength,(sub==subN ? INVERS : 0) );
//  	      if(sub==subN) {
//  	          CHECK_INCDEC_H_GENVAR_0(event, g_eeGeneral.hapticStrength, 5);
//  	      }
//  	      if((y+=FH)>7*FH) return;
//  	  }subN++;	
// 	//audio end by rob   

//  	lcd_puts_Pleft(    y, PSTR(STR_BRIGHTNESS));
//#ifdef PCBX9D
//#ifdef REVPLUS
//    lcd_puts_P(12*FW, y, XPSTR( "BLUE"));
//#endif
//#endif
//		lcd_outdezAtt( PARAM_OFS+2*FW, y, 100-g_eeGeneral.bright, (sub==subN) ? INVERS : 0 ) ;
//  	if(sub==subN)
//		{
//			uint8_t b ;
//			b = 100 - g_eeGeneral.bright ;
//			CHECK_INCDEC_H_GENVAR_0( event, b, 100 ) ;
//			g_eeGeneral.bright = 100 - b ;
//#ifdef PCBSKY
//			PWM->PWM_CH_NUM[0].PWM_CDTYUPD = g_eeGeneral.bright ;
//#endif
//#ifdef PCBX9D
//#ifdef REVPLUS
//			backlight_set( g_eeGeneral.bright, 0 ) ;
//#else
//			backlight_set( g_eeGeneral.bright ) ;
//#endif
//#endif
//		}
//  	if((y+=FH)>7*FH) return;
//		subN++;

//#ifdef REVPLUS
//  	lcd_puts_Pleft(    y, PSTR(STR_BRIGHTNESS));
//    lcd_puts_P(12*FW, y, XPSTR( "WHITE"));
//		lcd_outdezAtt( PARAM_OFS+2*FW, y, 100-g_eeGeneral.bright_white, (sub==subN) ? INVERS : 0 ) ;
//  	if(sub==subN)
//		{
//			uint8_t b ;
//			b = 100 - g_eeGeneral.bright_white ;
//			CHECK_INCDEC_H_GENVAR_0( event, b, 100 ) ;
//			g_eeGeneral.bright_white = 100 - b ;
//			backlight_set( g_eeGeneral.bright_white, 1 ) ;
//		}
//  	if((y+=FH)>7*FH) return;
//		subN++;
//#endif

//#ifdef PCBSKY
//#ifndef REVX
//  	g_eeGeneral.optrexDisplay = onoffMenuItem( g_eeGeneral.optrexDisplay, y, XPSTR("Optrex Display"), sub, sub, event ) ;
//  	if((y+=FH)>7*FH) return;
//		subN++;
//#endif

//  	lcd_puts_Pleft( y, PSTR(STR_CAPACITY_ALARM));
//		lcd_outdezAtt( PARAM_OFS+2*FW, y, g_eeGeneral.mAh_alarm*50, (sub==subN) ? INVERS : 0 ) ;
//  	if(sub==subN)
//		{
//			CHECK_INCDEC_H_GENVAR_0(event,g_eeGeneral.mAh_alarm,100);
//		}
//  	if((y+=FH)>7*FH) return;
//		subN++;
//  	lcd_puts_Pleft( y, PSTR(STR_BT_BAUDRATE));
////  	lcd_putsAttIdx(  PARAM_OFS-4*FW, y, XPSTR("\006115200  9600 19200"),g_eeGeneral.bt_baudrate,(sub==subN ? BLINK:0));
//  	lcd_putsAttIdx(  PARAM_OFS-4*FW, y, XPSTR("\006115200  9600 1920057600"),g_eeGeneral.bt_baudrate,(sub==subN ? BLINK:0));
//  	if(sub==subN)
//		{
//			uint8_t b ;
//			b = g_eeGeneral.bt_baudrate ;
//			CHECK_INCDEC_H_GENVAR_0(event,g_eeGeneral.bt_baudrate,3);
//			if ( b != g_eeGeneral.bt_baudrate )
//			{
//				uint32_t baudrate = 115200 ;
//				if ( b )
//				{
////					baudrate = ( b == 1 ) ? 9600 : 19200 ;
//					baudrate = ( b == 1 ) ? 9600 : ( ( b == 2 ) ? 19200 : 57600 ) ;
//				}
//				UART3_Configure( baudrate, Master_frequency ) ;
//			}
//		}
//  	if((y+=FH)>7*FH) return;
//		subN++;
//#endif
//	}
//	else if ( sub < NUM_SETUP1_PG1 + 5 )
//	{
//#ifdef PCBSKY
//		uint8_t b ;
//		uint8_t lastValue ;
//		lastValue = g_eeGeneral.stickGain ;
//#endif
//		subN = NUM_SETUP1_PG1 ;
//  	lcd_puts_Pleft( FH, PSTR(STR_ROTARY_DIVISOR));
//  	lcd_putsAttIdx( 15*FW, y, XPSTR("\001142"),g_eeGeneral.rotaryDivisor,(sub==subN ? BLINK:0));
//  	if(sub==subN)
//		{
//			CHECK_INCDEC_H_GENVAR_0(event,g_eeGeneral.rotaryDivisor,2);
//		}
//  	if((y+=FH)>7*FH) return;
		
//#ifdef PCBSKY
//		subN++;
//  	lcd_puts_Pleft( FH*2, PSTR(STR_STICK_LV_GAIN));
//		b = g_eeGeneral.stickGain & STICK_LV_GAIN ? 1 : 0 ;
//    lcd_putcAtt( 15*FW, FH*2, b ? '2' : '1', sub==subN ? BLINK:0 );
//  	if(sub==subN)
//		{
//			CHECK_INCDEC_H_GENVAR_0(event, b,1);
//			g_eeGeneral.stickGain = (lastValue & ~STICK_LV_GAIN) | (b ? STICK_LV_GAIN : 0 ) ;
//		}
//		subN++;
//  	lcd_puts_Pleft( FH*3, PSTR(STR_STICK_LH_GAIN));
//		b = g_eeGeneral.stickGain & STICK_LH_GAIN ? 1 : 0 ;
//    lcd_putcAtt( 15*FW, FH*3, b ? '2' : '1', sub==subN ? BLINK:0 );
//  	if(sub==subN)
//		{
//			CHECK_INCDEC_H_GENVAR_0(event, b,1);
//			g_eeGeneral.stickGain = (lastValue & ~STICK_LH_GAIN) | (b ? STICK_LH_GAIN : 0 ) ;
//		}
//		subN++;
//  	lcd_puts_Pleft( FH*4, PSTR(STR_STICK_RV_GAIN));
//		b = g_eeGeneral.stickGain & STICK_RV_GAIN ? 1 : 0 ;
//    lcd_putcAtt( 15*FW, FH*4, b ? '2' : '1', sub==subN ? BLINK:0 );
//  	if(sub==subN)
//		{
//			CHECK_INCDEC_H_GENVAR_0(event, b,1);
//			g_eeGeneral.stickGain = (lastValue & ~STICK_RV_GAIN) | (b ? STICK_RV_GAIN : 0 ) ;
//		}
//		subN++;
//  	lcd_puts_Pleft( FH*5, PSTR(STR_STICK_RH_GAIN));
//		b = g_eeGeneral.stickGain & STICK_RH_GAIN ? 1 : 0 ;
//    lcd_putcAtt( 15*FW, FH*5, b ? '2' : '1', sub==subN ? BLINK:0 );
//  	if(sub==subN)
//		{
//			CHECK_INCDEC_H_GENVAR_0(event, b,1);
//			g_eeGeneral.stickGain = (lastValue & ~STICK_RH_GAIN) | (b ? STICK_RH_GAIN : 0 ) ;
//		}
//		if ( lastValue != g_eeGeneral.stickGain )
//		{
//			set_stick_gain( g_eeGeneral.stickGain ) ;
//		}
//		subN++;
//	}
//	else
//	{
//		subN = NUM_SETUP1_PG1 + 5 ;
		
//#ifdef PCBSKY
////		lcd_outhex4( 18*FW, 4*FH, ADC->ADC_CHSR ) ;	// Debug
////		lcd_outhex4( 18*FW, 5*FH, Analog_values[9] ) ;	// Debug
//		uint8_t sm = g_eeGeneral.switchMapping ;
//  	lcd_puts_Pleft( y, XPSTR("ELE switch"));
//		uint8_t type = 0 ;
//		if ( sm & USE_ELE_3POS )
//		{
//			type = 1 ;
//		}
//		if ( sm & USE_ELE_6POS )
//		{
//			type = 2 ;
//		}
//  	lcd_putsAttIdx(  12*FW, y, XPSTR("\0042POS3POS6POS"), type, (sub==subN ? BLINK:0));
//  	if(sub==subN)
//		{
//			CHECK_INCDEC_H_GENVAR_0(event, type, 2 ) ;
//			g_eeGeneral.switchMapping = g_eeGeneral.switchMapping & ~(USE_ELE_3POS | USE_ELE_6POS) ;
//			if ( type == 0 )
//			{
//				g_eeGeneral.switchMapping = g_eeGeneral.switchMapping & ~(USE_THR_3POS | USE_RUD_3POS) ;
				
//			}
//			if ( type == 1 )
//			{
//				g_eeGeneral.switchMapping |= USE_ELE_3POS ;
//			}
//			if ( type == 2 )
//			{
//				g_eeGeneral.switchMapping |= USE_ELE_6POS ;
//			}
//			if ( sm != g_eeGeneral.switchMapping )
//			{
//				createSwitchMapping() ;
//			}
//		}
//  	if((y+=FH)>7*FH) return;
//		subN++;

//		if ( sm & ( USE_ELE_3POS | USE_ELE_6POS ) )
//		{
//			if ( ( sm & USE_RUD_3POS ) == 0 )
//			{
//				type = (sm & USE_THR_3POS )!=0 ;
				
//		  	lcd_puts_Pleft( y, XPSTR("THR switch"));
//		  	lcd_putsAttIdx(  12*FW, y, XPSTR("\0042POS3POS"), type, (sub==subN ? BLINK:0));
//		  	if(sub==subN)
//				{
//					CHECK_INCDEC_H_GENVAR_0(event, type, 1 ) ;
//					g_eeGeneral.switchMapping = g_eeGeneral.switchMapping & ~(USE_THR_3POS) ;
//					if ( type )
//					{
//						g_eeGeneral.switchMapping |= USE_THR_3POS ;
//					}
//				}
//				if ( sm != g_eeGeneral.switchMapping )
//				{
//					createSwitchMapping() ;
//				}
//	  		if((y+=FH)>7*FH) return;
//				subN++;
//			}
//			if ( ( sm & USE_THR_3POS ) == 0 )
//			{
//				type = (sm & USE_RUD_3POS )!=0 ;
				
//		  	lcd_puts_Pleft( y, XPSTR("RUD switch"));
//		  	lcd_putsAttIdx(  12*FW, y, XPSTR("\0042POS3POS"), type, (sub==subN ? BLINK:0));
//		  	if(sub==subN)
//				{
//					CHECK_INCDEC_H_GENVAR_0(event, type, 1 ) ;
//					g_eeGeneral.switchMapping = g_eeGeneral.switchMapping & ~(USE_RUD_3POS) ;
//					if ( type )
//					{
//						g_eeGeneral.switchMapping |= USE_RUD_3POS ;
//					}
//				}
//				if ( sm != g_eeGeneral.switchMapping )
//				{
//					createSwitchMapping() ;
//				}
//  			if((y+=FH)>7*FH) return;
//				subN++;
//			}
//		}

//		if ( ( sm & USE_GEA_3POS ) == 0 )
//		{
//			type = (sm & USE_AIL_3POS )!=0 ;
				
//		  lcd_puts_Pleft( y, XPSTR("AIL switch"));
//		  lcd_putsAttIdx(  12*FW, y, XPSTR("\0042POS3POS"), type, (sub==subN ? BLINK:0));
//		  if(sub==subN)
//			{
//				CHECK_INCDEC_H_GENVAR_0(event, type, 1 ) ;
//				g_eeGeneral.switchMapping = g_eeGeneral.switchMapping & ~(USE_AIL_3POS) ;
//				if ( type )
//				{
//					g_eeGeneral.switchMapping |= USE_AIL_3POS ;
//				}
//			}
//			if ( sm != g_eeGeneral.switchMapping )
//			{
//				createSwitchMapping() ;
//			}
//	  	if((y+=FH)>7*FH) return;
//			subN++;
//		}
//		if ( ( sm & USE_AIL_3POS ) == 0 )
//		{
//			type = (sm & USE_GEA_3POS )!=0 ;
				
//		  lcd_puts_Pleft( y, XPSTR("GEAR switch"));
//		  lcd_putsAttIdx(  12*FW, y, XPSTR("\0042POS3POS"), type, (sub==subN ? BLINK:0));
//		  if(sub==subN)
//			{
//				CHECK_INCDEC_H_GENVAR_0(event, type, 1 ) ;
//				g_eeGeneral.switchMapping = g_eeGeneral.switchMapping & ~(USE_GEA_3POS) ;
//				if ( type )
//				{
//					g_eeGeneral.switchMapping |= USE_GEA_3POS ;
//				}
//			}
//			if ( sm != g_eeGeneral.switchMapping )
//			{
//				createSwitchMapping() ;
//			}
//  		if((y+=FH)>7*FH) return;
//			subN++;
//		}

////  	lcd_puts_Pleft( FH*6, XPSTR("Switches"));
////		lcd_outdezAtt( PARAM_OFS+2*FW, y, g_eeGeneral.switchMapping, (sub==subN) ? INVERS : 0 ) ;
////		uint8_t x = g_eeGeneral.switchMapping ;
////  	if(sub==subN)
////		{
////			CHECK_INCDEC_H_GENVAR_0(event, g_eeGeneral.switchMapping, 63);
////			if ( x != g_eeGeneral.switchMapping )
////			{
////				createSwitchMapping() ;
////			}
////		}
////  	if((y+=FH)>7*FH) return;
////		subN++;
////  	lcd_puts_Pleft( FH*7, XPSTR("Switch test"));
////		putsDrSwitches( 12*FW, y, g_eeGeneral.switchTest, (sub==subN) ? INVERS : 0 ) ;
////		lcd_outdezAtt( 21*FW, y, g_eeGeneral.switchTest, 0 ) ;
////  	if(sub==subN)
////		{
////			int8_t x = g_eeGeneral.switchTest ;
////			int8_t y = x ;
////			CHECK_INCDEC_GENERALSWITCH(event, x, -MaxSwitchIndex, MaxSwitchIndex ) ;
////			if ( x != y )
////			{
////				g_eeGeneral.switchTest = x ;
////			}
////		}
//#endif // PCBSKY
//#endif
//	}
//}


// This is debug for a new file system
void menuProcSetup2(uint8_t event)
{
#ifdef PCBSKY
	uint32_t i ;
#endif
	uint32_t j ;
//  MENU(PSTR(STR_MEMORY_STAT), menuTabDiag, e_Setup2, 15, {0/*, 0*/});
	MENU(PSTR(STR_MEMORY_STAT), menuTabStat, e_Setup2, 27, {0} ) ;
	
	int8_t  sub    = mstate2.m_posVert;
//	uint8_t subSub = mstate2.m_posHorz;
//	bool    edit;
//	uint8_t blink ;

	evalOffset(sub, 1);

	j = sub + 1 ;
	if ( j > 27 )
	{
		j = 27 ;
	}
	lcd_puts_Pleft( 1*FH, PSTR(STR_GENERAL));
#ifdef PCBSKY
  lcd_outhex4( 8*FW+3, 1*FH, File_system[0].block_no ) ;
  lcd_outhex4( 12*FW+3, 1*FH, File_system[0].sequence_no ) ;
  lcd_outhex4( 16*FW+3, 1*FH, File_system[0].size ) ;
	for ( i = 1 ; i < 7 ; i += 1 )
	{
		lcd_puts_Pleft( (i+1)*FH, PSTR(STR_Model));
		lcd_putc( 5*FW, (i+1)*FH, j/10 + '0' ) ;
		lcd_putc( 6*FW, (i+1)*FH, j%10 + '0' ) ;
  	lcd_outhex4( 8*FW+3, (i+1)*FH, File_system[j].block_no ) ;
  	lcd_outhex4( 12*FW+3, (i+1)*FH, File_system[j].sequence_no ) ;
  	lcd_outhex4( 16*FW+3, (i+1)*FH, File_system[j].size ) ;
		j += 1 ;
	}
#endif

//#ifdef REVX
//	// RTC debug
//	extern uint8_t RtcConfig[] ;
//	extern uint8_t Rtc_status[] ;
//	for ( i = 0 ; i < 8 ; i += 1 )
//	{
//  	lcd_outhex2( i*2*FW, 6*FH, RtcConfig[i] ) ;
//  	lcd_outhex2( i*2*FW, 7*FH, Rtc_status[i] ) ;
//	}
//#endif
}

void menuProcRSSI(uint8_t event)
{
	TITLE( XPSTR( "Module RSSI" ) ) ;
	static MState2 mstate2 ;
	static const uint8_t mstate_tab[] = {1} ;

	event = mstate2.check(event,0,NULL,0,mstate_tab,DIM(mstate_tab)-1,2-1) ;
	
	uint8_t  sub = mstate2.m_posVert ;
	uint8_t subSub = mstate2.m_posHorz ;
	uint8_t blink;
	uint8_t y = 1*FH;

	switch(event)
	{
    case EVT_KEY_BREAK(KEY_DOWN):
    case EVT_KEY_BREAK(KEY_UP):
    case EVT_KEY_BREAK(KEY_LEFT):
    case EVT_KEY_BREAK(KEY_RIGHT):
      if(s_editMode)
         FrskyAlarmSendState |= 0x30 ;	 // update Fr-Sky module when edit mode exited
    break ;
		case EVT_ENTRY :
  		FrskyAlarmSendState |= 0x40 ;		// Get RSSI/TSSI alarms
		break ;
	}
	blink = InverseBlink ;
	
	uint8_t subN = 0 ;

	for (uint8_t j=0; j<2; j++)
	{
  	lcd_puts_Pleft( y, PSTR(STR_TX_RSSIALRM) );
  	if ( j == 1 )
  	{
  	    lcd_putcAtt( 0, y, 'R', 0 ) ;
  	}
  	lcd_putsAttIdx(11*FW, y, PSTR(STR_YELORGRED),frskyRSSItype[j],(sub==subN && subSub==0 ? blink:0));
  	lcd_outdezNAtt(17*FW, y, frskyRSSIlevel[j], (sub==subN && subSub==1 ? blink:0), 3);

  	if(sub==subN && (s_editMode || P1values.p1valdiff))
		{
 	    switch (subSub)
			{
  	    case 0:
  	   	  frskyRSSItype[j] = checkIncDec(event, frskyRSSItype[j], 0, 3, 0 ) ;
  	    break;
  	    case 1:
  	   	  frskyRSSIlevel[j] = checkIncDec16(event, frskyRSSIlevel[j], 0, 120, 0 ) ;
  	    break;
  	  }
  	}
  	subN++; y+=FH;
	}
}

// From Bertrand, allow trainer inputs without using mixers.
// Raw trianer inputs replace raw sticks.
// Only first 4 PPMin may be calibrated.
void menuProcTrainer(uint8_t event)
{
	
//  SUBMENU(PSTR(STR_TRAINER), 7, {0, 3, 3, 3, 3, 0/*, 0*/});
//  MENU(PSTR(STR_TRAINER), menuTabDiag, e_Trainer, 7, {0, 3, 3, 3, 3, 0/*, 0*/});

	TITLE(PSTR(STR_TRAINER));
	static MState2 mstate2;
	static const uint8_t mstate_tab[] = {0, 3, 3, 3, 3, 0/*, 0*/} ;
	
//	if (SubMenuFromIndex)
//	{
		event = mstate2.check(event,0,NULL,0,mstate_tab,DIM(mstate_tab)-1,7-1) ;
//	}
//	else
//	{
//		event = mstate2.check(event,e_Trainer,menuTabDiag,DIM(menuTabDiag),mstate_tab,DIM(mstate_tab)-1,7-1) ;
//	}

	int8_t  sub    = mstate2.m_posVert;
	uint8_t subSub = mstate2.m_posHorz;
	uint8_t y;
	bool    edit;
	uint8_t blink ;

#ifdef PCBSKY
	if ( check_soft_power() == 2 )		// On trainer power
	{ // i am the slave
	    lcd_puts_P(7*FW, 3*FH, PSTR(STR_SLAVE));
	    return;
	}
#endif

#ifdef PCBX9D
	uint8_t attr = 0 ;
	lcd_puts_Pleft( 0, XPSTR("\010Source:"));
	if ( sub == 0 )
	{
		attr = INVERS ;
		CHECK_INCDEC_H_GENVAR_0(event, g_eeGeneral.trainerSource, 3 ) ;
	}
  lcd_putsAttIdx(15*FW, 0, XPSTR("\005Jack Sbus Cppm Slave"), g_eeGeneral.trainerSource, attr ) ;
	if ( g_eeGeneral.trainerSource == 3 )
	{ // i am the slave
	    lcd_puts_P(7*FW, 3*FH, PSTR(STR_SLAVE));
	    return;
	}
#endif

	lcd_puts_P(3*FW, 1*FH, XPSTR("mode   % src  sw"));
	sub -= 1 ;
	y = 2*FH;
	blink =	InverseBlink ;

	for (uint8_t i=0; i<4; i++)
	{
	  volatile TrainerMix *td = &g_eeGeneral.trainer.mix[i];
	  volatile exTrainerMix *xtd = &g_eeGeneral.exTrainer[i] ;
	  putsChnRaw(0, y, i+1, 0);

	  edit = (sub==i && subSub==0);
    lcd_putsAttIdx(4*FW, y, PSTR(STR_OFF_PLUS_EQ),td->mode, edit ? blink : 0);
	  if (edit && s_editMode)
	    CHECK_INCDEC_H_GENVAR_0(event, td->mode, 2); //!! bitfield

	  edit = (sub==i && subSub==1);
	  lcd_outdezAtt(11*FW, y, xtd->studWeight, edit ? blink : 0);
	  if (edit && s_editMode)
	    CHECK_INCDEC_H_GENVAR(event, xtd->studWeight, -100, 100); //!! bitfield

	  edit = (sub==i && subSub==2);
    lcd_putsAttIdx(12*FW, y, PSTR(STR_CH1_4),td->srcChn, edit ? blink : 0);
	  if (edit && s_editMode)
	    CHECK_INCDEC_H_GENVAR_0(event, td->srcChn, 3); //!! bitfield

	  edit = (sub==i && subSub==3);
	  putsDrSwitches(15*FW, y, xtd->swtch, edit ? blink : 0);
	  if (edit && s_editMode)
	    CHECK_INCDEC_GENERALSWITCH(event, xtd->swtch, -MaxSwitchIndex, MaxSwitchIndex) ;

	  y += FH;
	}

	lcd_puts_P(0*FW, y, PSTR(STR_MULTIPLIER));
	lcd_outdezAtt(13*FW, y, g_eeGeneral.PPM_Multiplier+10, (sub==4 ? blink : 0)|PREC1);
	if(sub==4) CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.PPM_Multiplier, -10, 40);
	y += FH;

	edit = (sub==5);
	lcd_putsAtt(0*FW, y, PSTR(STR_CAL), edit ? blink : 0);
	for (uint8_t i=0; i<4; i++)
	{
	    uint8_t x = (i*9+15)*FW/2;
	    lcd_outdezAtt(x , y, (g_ppmIns[i]-g_eeGeneral.trainer.calib[i])*2, PREC1);
	}
	if (edit)
	{
	  if (event==EVT_KEY_FIRST(KEY_MENU))
		{
	  	memcpy(g_eeGeneral.trainer.calib, g_ppmIns, sizeof(g_eeGeneral.trainer.calib));
	  	STORE_GENERALVARS;     //eeWriteGeneral();
	  	//        eeDirty(EE_GENERAL);
	  	audioDefevent(AU_MENUS);
	  }
	}
}



//void menuProcSetup(uint8_t event)
//{


/////*
////#ifdef BEEPSPKR
////#define COUNT_ITEMS 22
////#else
////#define COUNT_ITEMS 20
////#endif
////*/

//#ifdef FRSKY
//	uint8_t vCountItems = 23 ; //21 is default
////  int8_t sw_offset = -6 ;
////	switch (g_eeGeneral.speakerMode & 1)
////	{
////	//beeper
////	case 0 :
////		vCountItems = 23 ;
////	break ;
////	//speaker
////	 	case 1 :
//			vCountItems += 1 ;
////		break ;
////			 	//pcmwav
////			  case 2:
////						vCountItems = 24;
////						break;	  	
////	}		
//	if ( /*( (g_eeGeneral.speakerMode & 1) == 1 ) || g_eeGeneral.speakerMode == 2) && */( g_eeGeneral.frskyinternalalarm == 1) )
//	{ // add in alert red/org/yel
//			vCountItems += 3;
////			sw_offset -= 3 ;
//	}		
		
//#else 
//	uint8_t vCountItems = 22 ; //21 is default
////  int8_t sw_offset = -5 ;
////		switch (g_eeGeneral.speakerMode){
////				//beeper
////				case 0:
////						vCountItems = 23;
////						break;
////				//piezo speaker
////			 	case 1:
////			 			vCountItems = 25;
////			 			break;
////			 	//pcmwav
////			  case 2:
////						vCountItems = 24;
////						break;	  	
////		}
//#endif

////    sw_offset += vCountItems ;

////	SIMPLE_MENU("RADIO SETUP", menuTabDiag, e_Setup, 20 ) ;
////  //  SIMPLE_MENU("RADIO SETUP", menuTabDiag, e_Setup, COUNT_ITEMS+1);
//		MENU(PSTR(STR_RADIO_SETUP), menuTabDiag, e_Setup, vCountItems+1,{0,sizeof(g_eeGeneral.ownerName)-1,0/*repeated...*/}) ;
//    uint8_t sub    = mstate2.m_posVert;
//    uint8_t subSub = mstate2.m_posHorz;
    
//    uint8_t t_pgOfs ;
//    t_pgOfs = evalOffset(sub, 7);

////    //if(t_pgOfs==COUNT_ITEMS-7) t_pgOfs= sub<(COUNT_ITEMS-4) ? COUNT_ITEMS-8 : COUNT_ITEMS-6;
////    if(t_pgOfs==vCountItems-7) t_pgOfs= sub<(vCountItems-4) ? vCountItems-8 : vCountItems-6;
////    if(t_pgOfs==19-7) t_pgOfs= sub<(19-4) ? 20-8 : 20-6;
//    uint8_t y = 1*FH;

////  switch(event)
////	{
//////		case EVT_KEY_FIRST(KEY_MENU):
//////  	  if(sub>0) s_editMode = !s_editMode;
//////  	break;

////  	case EVT_KEY_FIRST(KEY_EXIT):
////  	  if(s_editMode)
////			{
////  	    s_editMode = false;
////  	    killEvents(event);
////  	  }
////  	break;

////    case EVT_KEY_REPT(KEY_LEFT):
////    case EVT_KEY_FIRST(KEY_LEFT):
////      if(sub==1 && subSub>0 && s_editMode) mstate2.m_posHorz--;
//////      if(sub==sw_offset && subSub>0) mstate2.m_posHorz--;   //for Sw Position
////    break;

////    case EVT_KEY_REPT(KEY_RIGHT):
////    case EVT_KEY_FIRST(KEY_RIGHT):
////      if(sub==1 && subSub<sizeof(g_model.name)-1 && s_editMode) mstate2.m_posHorz++;
//////      if(sub==sw_offset && subSub<7) mstate2.m_posHorz++;
////    break;

////    case EVT_KEY_REPT(KEY_UP):
////    case EVT_KEY_FIRST(KEY_UP):
////    case EVT_KEY_REPT(KEY_DOWN):
////    case EVT_KEY_FIRST(KEY_DOWN):
////			if (!s_editMode) mstate2.m_posHorz = 0;
////    break;
////  }

//  uint8_t subN = 1;

//  if(t_pgOfs<subN)
//	{
//    lcd_puts_Pleft(    y, PSTR(STR_OWNER_NAME));
//    lcd_putsnAtt(11*FW,   y, g_eeGeneral.ownerName ,sizeof(g_eeGeneral.ownerName), 0 );
    
//		if(sub==subN)
//		{
//			lcd_rect(11*FW-2, y-1, 10*FW+4, 9 ) ;
//			lcd_char_inverse( (11+subSub)*FW, y, 1*FW, s_editMode ) ;

//	    if(s_editMode)
//			{
//        char v = char2idx(g_eeGeneral.ownerName[subSub]);
//        CHECK_INCDEC_H_GENVAR_0( event,v ,NUMCHARS-1);
//        v = idx2char(v);
//				if ( g_eeGeneral.ownerName[subSub] != v )
//				{
//        	g_eeGeneral.ownerName[subSub] = v ;
//    			eeDirty( EE_GENERAL ) ;				// Do here or the last change is not stored in ModelNames[]
//				}
//			}
//		} 
//    if((y+=FH)>7*FH) return;
//  }subN++;

//  if(t_pgOfs<subN)
//	{
//    uint8_t b ;
//    b = g_eeGeneral.beeperVal ;
//        lcd_puts_Pleft( y,PSTR(STR_BEEPER));
//        lcd_putsAttIdx(PARAM_OFS - FW - 4, y, PSTR(STR_BEEP_MODES),b,(sub==subN ? INVERS:0));
//    if(sub==subN) { CHECK_INCDEC_H_GENVAR_0(event, b, 6); g_eeGeneral.beeperVal = b ; }

//    if((y+=FH)>7*FH) return;
//  }subN++;

////  if(t_pgOfs<subN)
////	{
////    uint8_t b ;
////    b = g_eeGeneral.speakerMode ;
////    lcd_puts_P(0, y,PSTR(STR_SOUND_MODE));
////    lcd_putsAttIdx( 11*FW, y, PSTR(STR_SPEAKER_OPTS),b,(sub==subN ? INVERS:0));
////    if(sub==subN) { CHECK_INCDEC_H_GENVAR(event, b, 0, 3); g_eeGeneral.speakerMode = b ; }	// 0 to 2 later
////    if((y+=FH)>7*FH) return;
////  }subN++;




//    if(t_pgOfs<subN) {
//        lcd_puts_Pleft( y,PSTR(STR_CONTRAST));
//        lcd_outdezAtt(PARAM_OFS,y,g_eeGeneral.contrast,(sub==subN ? INVERS : 0)|LEFT);
//        if(sub==subN) {
//#ifdef REVPLUS
//						CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.contrast, 10, 60);
//#else            
//						CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.contrast, 10, 45);
//#endif            
//						lcdSetRefVolt(g_eeGeneral.contrast);
//        }
//        if((y+=FH)>7*FH) return;
//    }subN++;

//    if(t_pgOfs<subN) {
//  			uint8_t attr = LEFT ;
//        lcd_puts_Pleft( y,PSTR(STR_BATT_WARN));
//        if(sub==subN) { attr = INVERS | LEFT ; CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.vBatWarn, 40, 120); } //5-10V
//        putsVolts(PARAM_OFS, y, g_eeGeneral.vBatWarn, attr);
//        if((y+=FH)>7*FH) return;
//    }subN++;

//    if(t_pgOfs<subN) {
//  			uint8_t attr = 0 ;//LEFT ;
//        lcd_puts_Pleft( y,PSTR(STR_INACT_ALARM));
//        if(sub==subN) { attr = INVERS ; CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.inactivityTimer, -10, 110); } //0..120minutes
//        lcd_outdezAtt(PARAM_OFS+2*FW-2, y, g_eeGeneral.inactivityTimer+10, attr);
//        if((y+=FH)>7*FH) return;
//    }subN++;

//    if(t_pgOfs<subN) {
//        lcd_puts_Pleft( y,PSTR(STR_FILTER_ADC));
//        lcd_putsAttIdx(PARAM_OFS, y, XPSTR("\004SINGOSMPFILT"),g_eeGeneral.filterInput,(sub==subN ? INVERS:0));
//        if(sub==subN) CHECK_INCDEC_H_GENVAR_0(event, g_eeGeneral.filterInput, 2);
//        if((y+=FH)>7*FH) return;
//    }subN++;

//    if(t_pgOfs<subN) {
//        g_eeGeneral.throttleReversed = onoffMenuItem( g_eeGeneral.throttleReversed, y, PSTR(STR_THR_REVERSE), sub, sub, event ) ;
//        if((y+=FH)>7*FH) return;
//    }subN++;

//    if(t_pgOfs<subN) {
//        g_eeGeneral.minuteBeep = onoffMenuItem( g_eeGeneral.minuteBeep, y, PSTR(STR_MINUTE_BEEP), sub, sub, event ) ;
//        if((y+=FH)>7*FH) return;
//    }subN++;

//    if(t_pgOfs<subN) {
//        g_eeGeneral.preBeep = onoffMenuItem( g_eeGeneral.preBeep, y, PSTR(STR_BEEP_COUNTDOWN), sub, sub, event ) ;
//        if((y+=FH)>7*FH) return;
//    }subN++;

//    if(t_pgOfs<subN) {
//        g_eeGeneral.flashBeep = onoffMenuItem( g_eeGeneral.flashBeep, y, PSTR(STR_FLASH_ON_BEEP), sub, sub, event ) ;
//        if((y+=FH)>7*FH) return;
//    }subN++;

//    if(t_pgOfs<subN) {
//        lcd_puts_Pleft( y,PSTR(STR_LIGHT_SWITCH));
//        putsDrSwitches(PARAM_OFS-FW,y,g_eeGeneral.lightSw,sub==subN ? INVERS : 0);
//        if(sub==subN) CHECK_INCDEC_GENERALSWITCH(event, g_eeGeneral.lightSw, -MaxSwitchIndex, MaxSwitchIndex);
//        if((y+=FH)>7*FH) return;
//    }subN++;

//		for ( uint8_t i = 0 ; i < 2 ; i += 1 )
//		{
//    	if(t_pgOfs<subN) {
//					uint8_t b ;
//    	    lcd_puts_Pleft( y,( i == 0) ? PSTR(STR_LIGHT_AFTER) : PSTR(STR_LIGHT_STICK) );
//					b = ( i == 0 ) ? g_eeGeneral.lightAutoOff : g_eeGeneral.lightOnStickMove ;

//    	    if(b) {
//    	        lcd_outdezAtt(PARAM_OFS, y, b*5,LEFT|(sub==subN ? INVERS : 0));
//    	        lcd_putc(Lcd_lastPos, y, 's');
//    	    }
//    	    else
//    	        lcd_putsnAtt(PARAM_OFS, y, PSTR(STR_OFF),3,(sub==subN ? INVERS:0));
//    	    if(sub==subN) CHECK_INCDEC_H_GENVAR_0(event, b, 600/5);
//					if ( i == 0 )
//					{
//						g_eeGeneral.lightAutoOff = b ;
//					}
//					else
//					{
//						g_eeGeneral.lightOnStickMove = b ;
//					}
//    	    if((y+=FH)>7*FH) return;
//    	}subN++;
			
//		}

//    if(t_pgOfs<subN) {
//        uint8_t b = 1-g_eeGeneral.disableSplashScreen;
//        g_eeGeneral.disableSplashScreen = 1-onoffMenuItem( b, y, PSTR(STR_SPLASH_SCREEN), sub, sub, event ) ;
//        if((y+=FH)>7*FH) return;
//    }subN++;

//    if(t_pgOfs<subN) {
//        uint8_t b = 1-g_eeGeneral.hideNameOnSplash;
//        g_eeGeneral.hideNameOnSplash = 1-onoffMenuItem( b, y, PSTR(STR_SPLASH_NAME), sub, sub, event ) ;
//        if((y+=FH)>7*FH) return;
//    }subN++;
    
//		if(t_pgOfs<subN) {
//        uint8_t b = 1-g_eeGeneral.disableThrottleWarning;
//        g_eeGeneral.disableThrottleWarning = 1-onoffMenuItem( b, y, PSTR(STR_THR_WARNING), sub, sub, event ) ;
//        if((y+=FH)>7*FH) return;
//    }subN++;

////    if(t_pgOfs<subN) {
////        uint8_t b = 1-g_eeGeneral.disableSwitchWarning;
////        g_eeGeneral.disableSwitchWarning = 1-onoffMenuItem( b, y, PSTR(STR_SWITCH_WARN), sub, sub, event ) ;
////        if((y+=FH)>7*FH) return;
////    }subN++;


////    if(t_pgOfs<subN) {
////        lcd_puts_Pleft(    y, PSTR(STR_DEAFULT_SW));
        
////        for(uint8_t i=0, q=1;i<8;q<<=1,i++)
////				{
////					lcd_putsnAtt((11+i)*FW, y, Str_TRE012AG+i,1,  (((uint8_t)g_eeGeneral.switchWarningStates & q) ? INVERS : 0 ) );
////				}

////        if(sub==subN)
////				{
////					lcd_rect( 11*FW-1, y-1, 8*FW+2, 9 ) ;
////          if (event==EVT_KEY_FIRST(KEY_MENU) || event==EVT_KEY_FIRST(BTN_RE))
////					{
////            killEvents(event);
////	          g_eeGeneral.switchWarningStates = getCurrentSwitchStates() ;
////        		s_editMode = false ;
////            STORE_GENERALVARS;
////					}
////				}
        
////        if((y+=FH)>7*FH) return;
////    }subN++;

////    if(t_pgOfs<subN) {
////        uint8_t b = 1-g_eeGeneral.disableMemoryWarning;
////        g_eeGeneral.disableMemoryWarning = 1-onoffMenuItem( b, y, PSTR(STR_MEM_WARN), sub, sub, event ) ;
////        //						;
////        //        }
////        if((y+=FH)>7*FH) return;
////    }subN++;

//    if(t_pgOfs<subN) {
//        uint8_t b = 1-g_eeGeneral.disableAlarmWarning;
//        g_eeGeneral.disableAlarmWarning = 1-onoffMenuItem( b, y, PSTR(STR_ALARM_WARN), sub, sub, event ) ;
//        if((y+=FH)>7*FH) return;
//    }subN++;

    
//    if(t_pgOfs<subN)
//    {
//        uint8_t b ;
//        b = 1-g_eeGeneral.disablePotScroll ;
//				b |= g_eeGeneral.stickScroll << 1 ;
//        lcd_puts_Pleft( y,PSTR(STR_SCROLLING));
//        lcd_putsAttIdx(PARAM_OFS-3, y, XPSTR("\005NONE POT  STICKBOTH "),b,(sub==subN ? INVERS:0));
//        if(sub==subN) CHECK_INCDEC_H_GENVAR_0(event, b, 3 ) ;
//				g_eeGeneral.stickScroll = b >> 1 ;
//				g_eeGeneral.disablePotScroll = 1 - ( b & 1 ) ;
//				if((y+=FH)>7*FH) return;
//    }subN++;

////frsky alert mappings
//#ifdef FRSKY

////		if((g_eeGeneral.speakerMode & 1) == 1 /*|| g_eeGeneral.speakerMode == 2*/){
////		if(g_eeGeneral.speakerMode == 1 || g_eeGeneral.speakerMode == 2){
//						if(t_pgOfs<subN) {
//				        g_eeGeneral.frskyinternalalarm = onoffMenuItem( g_eeGeneral.frskyinternalalarm, y, PSTR(STR_INT_FRSKY_ALRM), sub, sub, event ) ;
//				        if((y+=FH)>7*FH) return;
//				    }subN++;
////		}		    
				    
////    if(((g_eeGeneral.speakerMode & 1) == 1 /*|| g_eeGeneral.speakerMode == 2*/) && g_eeGeneral.frskyinternalalarm == 1){ 
////    if((g_eeGeneral.speakerMode == 1 || g_eeGeneral.speakerMode == 2) && g_eeGeneral.frskyinternalalarm == 1){ 
    

					  
//    if( g_eeGeneral.frskyinternalalarm == 1)
//		{
//						for ( uint8_t i = 0 ; i < 3 ; i += 1 )
//					  {
//					    uint8_t b ;
					
//					    b = g_eeGeneral.FRSkyYellow ;    // Done here to stop a compiler warning
//					    if(t_pgOfs<subN)
//							{
								
//								if ( i == 0 )
//								{
//					        lcd_puts_P(0, y,PSTR(STR_ALERT_YEL));
//								}
//								else if ( i == 1 )
//								{
//					        b = g_eeGeneral.FRSkyOrange ;
//					        lcd_puts_P(0, y,PSTR(STR_ALERT_ORG));
//								}
//								else if ( i == 2 )
//								{
//					        b = g_eeGeneral.FRSkyRed ;
//					        lcd_puts_P(0, y,PSTR(STR_ALERT_RED));
//								}
//					      //lcd_putsnAtt(PARAM_OFS - FW - 4, y, PSTR("Tone1 ""Tone2 ""Tone3 ""Tone4 ""Tone5 ""hTone1""hTone2""hTone3""hTone4""hTone5")+6*b,6,(sub==subN ? INVERS:0));
//					      if((g_eeGeneral.speakerMode & 1) == 1){
//					      			lcd_putsAttIdx(PARAM_OFS - FW - 4, y, PSTR(STR_SOUNDS),b,(sub==subN ? INVERS:0));
//								}
////					      if(g_eeGeneral.speakerMode == 2){
////					      			lcd_putsnAtt(PARAM_OFS - FW - 4, y, PSTR("Trck1 ""Trck2 ""Trck3 ""Trck4 ""Trck4 ""Trck5 ""Trck6 ""Trck7 ""Trck8 ""Trck9 ""Trck10""Trck11""Trck12""Haptc1""Haptc2""Haptc3")+6*b,6,(sub==subN ? INVERS:0));
////								}								
//					      if(sub==subN)
//								{
//									//CHECK_INCDEC_H_GENVAR_0(event, b, 9);
//									CHECK_INCDEC_H_GENVAR_0(event, b, 15);
//									if ( i == 0 )
//									{
//							      g_eeGeneral.FRSkyYellow = b ;
//									}
//									else if ( i == 1 )
//									{
//							      g_eeGeneral.FRSkyOrange = b ;
//									}
//									else if ( i == 2 )
//									{
//							      g_eeGeneral.FRSkyRed = b ;
//									}
//#ifdef PCBSKY
//								  if (checkIncDec_Ret)
//									{
//										audio.event(b);
//									}
//#endif
//								}
//								if((y+=FH)>7*FH) return;
//					    }subN++;
//					  }
//		}			  
//#endif

//	if(t_pgOfs<subN)
//	{
//		g_eeGeneral.crosstrim = onoffMenuItem( g_eeGeneral.crosstrim, y, PSTR(STR_CROSSTRIM), sub, sub, event ) ;
////    uint8_t b ;
////    b = g_eeGeneral.crosstrim ;
////    lcd_puts_Pleft(    y, PSTR(STR_CROSSTRIM));
////    menu_lcd_onoff( PARAM_OFS, y, b, sub==subN ) ;
////    if(sub==subN) { CHECK_INCDEC_H_GENVAR_0(event,b,1); g_eeGeneral.crosstrim = b ; }
//    if((y+=FH)>7*FH) return;
//	}subN++;

//	if(t_pgOfs<subN)
//	{
//    lcd_puts_Pleft( y,PSTR(STR_LANGUAGE));
//    lcd_putsAttIdx( 11*FW, y, XPSTR("\012   ENGLISH  FRANCAIS   DEUTSCH NORWEGIAN   SWEDISH"),g_eeGeneral.language,(sub==subN ? INVERS:0));
//    if(sub==subN) CHECK_INCDEC_H_GENVAR_0(event, g_eeGeneral.language, 4 ) ;
//		setLanguage() ;
//    if((y+=FH)>7*FH) return;
//	}subN++;

//  if(t_pgOfs<subN)
//	{
//		uint8_t attr = sub==subN ? INVERS : 0 ;
//	  lcd_puts_Pleft( y, PSTR(STR_CHAN_ORDER) ) ;//   RAET->AETR
//		uint8_t bch = bchout_ar[g_eeGeneral.templateSetup] ;
//    for ( uint8_t i = 4 ; i > 0 ; i -= 1 )
//		{
//			uint8_t letter ;
//			letter = *(PSTR(STR_SP_RETA) +(bch & 3) + 1 ) ;
//  		lcd_putcAtt( (16+i)*FW, y, letter, attr ) ;
//			bch >>= 2 ;
//		}
//	  if(attr) CHECK_INCDEC_H_GENVAR_0( event, g_eeGeneral.templateSetup, 23 ) ;
//		if((y+=FH)>7*FH) return ;
//  }
//	subN++;

//	if(t_pgOfs<subN)
//	{
//    lcd_puts_Pleft( y, PSTR(STR_MODE) );
//    for ( uint8_t i = 0 ; i < 4 ; i += 1 )
//		{
//			lcd_img((6+4*i)*FW, y, sticks, i, 0 ) ;
//			if (g_eeGeneral.stickReverse & (1<<i)) lcd_char_inverse( (6+4*i)*FW, y, 3*FW, 0 ) ;
//		}
//    if(sub==subN)
//		{
//			CHECK_INCDEC_H_GENVAR_0( event, g_eeGeneral.stickReverse, 15 ) ;
//			plotType = PLOT_BLACK ;
//			lcd_rect( 6*FW-1, y-1, 15*FW+2, 9 ) ;
//			plotType = PLOT_XOR ;
//		}
//		if((y+=FH)>7*FH) return ;
//  }
//	subN++;
    
//		if(t_pgOfs<subN)
//		{
//  			uint8_t attr = 0 ;
//				uint8_t mode = g_eeGeneral.stickMode ;
//        if(sub==subN)
//				{
//					attr = INVERS ;
//					if ( s_editMode )
//					{
//				 		attr = BLINK ;
						
//						CHECK_INCDEC_H_GENVAR_0(event, mode,3);
//						if ( mode != g_eeGeneral.stickMode )
//						{
//							g_eeGeneral.stickScroll = 0 ;
//							g_eeGeneral.stickMode = mode ;							
//						}
//					}
//				}
//        lcd_putcAtt( 3*FW, y, '1'+g_eeGeneral.stickMode,attr);
//#ifdef FIX_MODE
//        for(uint8_t i=0; i<4; i++) putsChnRaw( (6+4*i)*FW, y, modeFixValue( i ), 0 ) ;//sub==3?INVERS:0);
//#else
//        for(uint8_t i=0; i<4; i++) putsChnRaw( (6+4*i)*FW, y,i+1,0);//sub==3?INVERS:0);
//#endif
        
//				if((y+=FH)>7*FH) return;
//    }subN++;




//}

uint16_t s_timeCumTot;		// Total tx on time (secs)
uint16_t s_timeCumAbs;  //laufzeit in 1/16 sec
static uint16_t s_time;
static uint8_t s_cnt;

// Timer triggers:
// OFF - disabled
// ABS AND no switch - always running
// THs AND no switch - throttle stick
// TH% AND no switch - throttle %
// cx% AND no switch - channel %
// ABS AND switch/switchm - controlled by switch
// THs AND switch/switchm - both throttle stick AND switch
// TH% AND switch/switchm - both throttle % AND switch
// cx% AND switch/switchm - both channel % AND switch

// 1. Test modeA, if OFF, timer OFF
// 2. Test modeB, if no switch, or switch is ON, timer may run, else OFF
// 3. Test modeA, if ABS timer is ON
// 4. Test modeA, if THs, timer ON if throttle not 0
// 4. Test modeA, if Th% or cx%, timer on as %

//int16_t  s_timerVal;
void timer(int16_t throttle_val)
{
	
	int16_t val ;
	uint8_t timer ;
	int8_t tma ;
  int16_t tmb ;
  uint16_t tv ;
  
  s_cnt++;			// Number of times val added in
	for( timer = 0 ; timer < 2 ; timer += 1 )
	{
		struct t_timer *ptimer = &s_timer[timer] ;
		uint8_t resetting = 0 ;
		if ( timer == 0 )
		{
			tmb = g_model.timer1RstSw ;
		}
		else
		{
			tmb = g_model.timer2RstSw ;
		}
		if ( tmb )
		{
#ifdef PCBSKY
			if ( tmb < -HSW_MAX )
			{
				tmb += 256 ;
			}
#endif
#ifdef PCBSKY
    	if(tmb>=(HSW_MAX))	 // toggeled switch
#endif
#ifdef PCBX9D
//    	if(tmb>=(MAX_SKYDRSWITCH-1))	 // toggeled switch
    	if(tmb>=(HSW_MAX))	 // toggeled switch
#endif
			{
#ifdef PCBSKY
    	  uint8_t swPos = getSwitch( tmb-(HSW_MAX), 0 ) ;
#endif
#ifdef PCBX9D
//    	  uint8_t swPos = getSwitch( tmb-(MAX_SKYDRSWITCH-1), 0 ) ;
    	  uint8_t swPos = getSwitch( tmb-(HSW_MAX), 0 ) ;
#endif
				if ( swPos != ptimer->lastResetSwPos )
				{
					ptimer->lastResetSwPos = swPos ;
					if ( swPos )	// Now on
					{
						resetting = 1 ;
					}
				}
			}
			else
			{
				if ( getSwitch( tmb, 0, 0) )
				{
					resetting = 1 ;
				}
			}
		}
		if ( resetting )
		{
			if ( timer == 0 )
			{
				resetTimer1() ;
			}
			else
			{
				resetTimer2() ;
			}
		}
		
		tma = g_model.timer[timer].tmrModeA ;
    tmb = g_model.timer[timer].tmrModeB ;
#ifdef PCBSKY
		if ( tmb < -HSW_MAX )
		{
			tmb += 256 ;
		}
#endif
#ifdef PCBX9D
		if ( tmb < -HSW_MAX )
		{
			tmb += 256 ;
		}
#endif

// code for cx%
		val = throttle_val ;
   	if(tma>=TMR_VAROFS) // Cxx%
		{
 	    val = g_chans512[tma-TMR_VAROFS] ;
		}		

		val = ( val + RESX ) / (RESX/16) ;

		if ( tma != TMRMODE_NONE )		// Timer is not off
		{ // We have a triggerA so timer is running 
#ifdef PCBSKY
    	if(tmb>=(HSW_MAX))	 // toggeled switch
#endif
#ifdef PCBX9D
//    	if(tmb>=(MAX_SKYDRSWITCH-1))	 // toggeled switch
    	if(tmb>=(HSW_MAX))	 // toggeled switch
#endif
			{
    	  if(!(ptimer->sw_toggled | ptimer->s_sum | s_cnt | s_time | ptimer->lastSwPos)) ptimer->lastSwPos = 0 ;  // if initializing then init the lastSwPos
#ifdef PCBSKY
    	  uint8_t swPos = getSwitch( tmb-(HSW_MAX), 0 ) ;
#endif
#ifdef PCBX9D
//    	  uint8_t swPos = getSwitch( tmb-(MAX_SKYDRSWITCH-1), 0 ) ;
    	  uint8_t swPos = getSwitch( tmb-(HSW_MAX), 0 ) ;
#endif
    	  if(swPos && !ptimer->lastSwPos)  ptimer->sw_toggled = !ptimer->sw_toggled;  //if switch is flipped first time -> change counter state
    	  ptimer->lastSwPos = swPos;
    	}
    	else
			{
				if ( tmb )
				{
    	  	ptimer->sw_toggled = getSwitch( tmb ,0); //normal switch
				}
				else
				{
					ptimer->sw_toggled = 1 ;	// No trigger B so use as active
				}
			}
		}

		if ( ( ptimer->sw_toggled == 0 ) || resetting )
		{
			val = 0 ;
		}

    ptimer->s_sum += val ;   // Add val in
    if( ( (uint16_t)( get_tmr10ms()-s_time) ) < 100 )		// BEWARE of 32 bit processor extending 16 bit values
		{
			if ( timer == 0 )
			{
				continue ; //1 sec
			}
			else
			{
				return ;
			}
		}
    val     = ptimer->s_sum/s_cnt;   // Average of val over last 100mS
    ptimer->s_sum  -= val*s_cnt;     //rest (remainder not added in)

		if ( timer == 0 )
		{
    	s_timeCumTot += 1;
	    s_timeCumAbs += 1;
			g_eeGeneral.totalElapsedTime += 1 ;
		}
		else
		{
	    s_cnt   = 0;    // ready for next 100mS
			s_time += 100;  // 100*10mS passed
		}
    if(val) ptimer->s_timeCumThr       += 1;
		if ( !resetting )
		{
    	if(ptimer->sw_toggled) ptimer->s_timeCumSw += 1;
		}
    ptimer->s_timeCum16ThrP            += val>>1;	// val/2

    tv = ptimer->s_timerVal = g_model.timer[timer].tmrVal ;
    if(tma == TMRMODE_NONE)
		{
			ptimer->s_timerState = TMR_OFF;
		}
    else
		{
			if ( tma==TMRMODE_ABS )
			{
				if ( tmb == 0 ) ptimer->s_timerVal -= s_timeCumAbs ;
	    	else ptimer->s_timerVal -= ptimer->s_timeCumSw ; //switch
			}
	    else if(tma<TMR_VAROFS-1) ptimer->s_timerVal -= ptimer->s_timeCumThr;	// stick
		  else ptimer->s_timerVal -= ptimer->s_timeCum16ThrP/16 ; // stick% or Cx%
		}   
		 
    switch(ptimer->s_timerState)
    {
    case TMR_OFF:
        if(tma != TMRMODE_NONE) ptimer->s_timerState=TMR_RUNNING;
        break;
    case TMR_RUNNING:
        if(ptimer->s_timerVal<0 && tv) ptimer->s_timerState=TMR_BEEPING;
        break;
    case TMR_BEEPING:
        if(ptimer->s_timerVal <= -MAX_ALERT_TIME)   ptimer->s_timerState=TMR_STOPPED;
        if(tv == 0)       ptimer->s_timerState=TMR_RUNNING;
        break;
    case TMR_STOPPED:
        break;
    }

//		if ( timer == 0 )
//		{

  	  if(ptimer->last_tmr != ptimer->s_timerVal)  //beep only if seconds advance
    	{
    		ptimer->last_tmr = ptimer->s_timerVal;
        if(ptimer->s_timerState==TMR_RUNNING)
        {
					uint8_t audioControl ;
					if ( timer == 0 )
					{
						audioControl = g_eeGeneral.preBeep | g_model.timer1Cdown ;
					}
					else
					{
						audioControl = g_model.timer2Cdown ;
					}
					
            if(audioControl && g_model.timer[timer].tmrVal) // beep when 30, 15, 10, 5,4,3,2,1 seconds remaining
            {
              	if(ptimer->s_timerVal==30) {audioVoiceDefevent(AU_TIMER_30, V_30SECS);}
              	if(ptimer->s_timerVal==20) {audioVoiceDefevent(AU_TIMER_20, V_20SECS);}
                if(ptimer->s_timerVal==10) {audioVoiceDefevent(AU_TIMER_10, V_10SECS);}
                if(ptimer->s_timerVal<= 5)
								{
									if(ptimer->s_timerVal>= 0)
									{
										audioVoiceDefevent(AU_TIMER_LT3, ptimer->s_timerVal) ;
									}
									else
									{
										if ( ( timer == 0 ) && g_eeGeneral.preBeep )
										{
											audioDefevent(AU_TIMER_LT3);
										}
									}
								}
								if(g_eeGeneral.flashBeep && (ptimer->s_timerVal==30 || ptimer->s_timerVal==20 || ptimer->s_timerVal==10 || ptimer->s_timerVal<=3))
                    g_LightOffCounter = FLASH_DURATION;
            }
						div_t mins ;
						mins = div( g_model.timer[timer].tmrDir ? g_model.timer[timer].tmrVal- ptimer->s_timerVal : ptimer->s_timerVal, 60 ) ;
					if ( timer == 0 )
					{
						audioControl = g_eeGeneral.minuteBeep | g_model.timer1Mbeep ;
					}
					else
					{
						audioControl = g_model.timer2Mbeep ;
					}
            if( audioControl && ((mins.rem)==0)) //short beep every minute
            {
//								if ( g_eeGeneral.speakerMode & 2 )
//								{
									if ( mins.quot ) {voice_numeric( mins.quot, 0, V_MINUTES ) ;}
//								}
//								else
//								{
//                	audioDefevent(AU_WARNING1);
//								}
                if(g_eeGeneral.flashBeep) g_LightOffCounter = FLASH_DURATION;
            }
        }
        else if(ptimer->s_timerState==TMR_BEEPING)
        {
					if ( ( timer == 0 ) && g_eeGeneral.preBeep )
					{
            audioDefevent(AU_TIMER_LT3);
            if(g_eeGeneral.flashBeep) g_LightOffCounter = FLASH_DURATION;
					}
        }
    	}
//		}
    if( g_model.timer[timer].tmrDir) ptimer->s_timerVal = tv-ptimer->s_timerVal; //if counting backwards - display backwards
//    if( tv==0) s_timer[timer].s_timerVal = tv-s_timer[timer].s_timerVal; //if counting backwards - display backwards
	}
}


#define MAXTRACE 120
uint8_t s_traceBuf[MAXTRACE];
uint8_t s_traceWr;
uint16_t s_traceCnt;
void trace()   // called in perOut - once every 0.01sec
{
    //value for time described in g_model.tmrMode
    //OFFABSRUsRU%ELsEL%THsTH%ALsAL%P1P1%P2P2%P3P3%
    //now OFFABSTHsTH%
  int16_t val ;
#ifdef FIX_MODE
  val = calibratedStick[3-1];
#else
  val = calibratedStick[CONVERT_MODE(3)-1]; //Get throttle channel value
#endif
//    uint16_t v = 0;
//    if((g_model.timer[0].tmrModeA>1) && (g_model.timer[0].tmrModeA<TMR_VAROFS))
//		{
//      v = ( val + RESX ) / (RESX/16) ;
//    }

// ***** Handle sticky throttle here
		if ( ThrottleStickyOn )
		{
			val = -RESX ;
		}
    timer(val);

//    uint16_t val = calibratedStick[CONVERT_MODE(3)-1]; //Get throttle channel value
  val = (val+RESX) / (RESX/16); //calibrate it
  
	static uint16_t s_time;
  static uint16_t s_cnt;
  static uint16_t s_sum;
//    static uint8_t test = 1;
  s_cnt++;
  s_sum+=val;
  if((uint16_t)( get_tmr10ms()-s_time)<1000) //10 sec
      //		uint16_t time10ms ;
      //		time10ms = get_tmr10ms() ;
      //    if(( time10ms-s_time)<1000) //10 sec
      return;
  s_time= get_tmr10ms() ;
 
  if ((g_model.protocol==PROTO_DSM2)&&getSwitch(MAX_SKYDRSWITCH-1,0,0) ) audioDefevent(AU_TADA);   //DSM2 bind mode warning
  //    s_time= time10ms ;
  val   = s_sum/s_cnt;
  s_sum = 0;
  s_cnt = 0;

  s_traceCnt++;
  s_traceBuf[s_traceWr++] = val;
  if(s_traceWr>=MAXTRACE) s_traceWr=0;
}

uint16_t g_timeMain;
uint16_t g_timeRfsh ;
uint16_t g_timeMixer ;
uint16_t g_timePXX;

void menuProcStatistic2(uint8_t event)
{
//  TITLE(PSTR(STR_STAT2));
	MENU(PSTR(STR_STAT2), menuTabStat, e_stat2, 1, {0} ) ;

  switch(event)
  {
    case EVT_KEY_FIRST(KEY_MENU):
      g_timeMain = 0;
      audioDefevent(AU_MENUS) ;
    break;
    case EVT_KEY_LONG(KEY_MENU):
			g_eeGeneral.totalElapsedTime = 0 ;
    break;

//    case EVT_KEY_FIRST(KEY_DOWN):
//        chainMenu(menuProcStatistic);
//        break;
//    case EVT_KEY_FIRST(KEY_UP):
//    case EVT_KEY_FIRST(KEY_EXIT):
//      chainMenu(menuProc0);
//    	killEvents(event) ;
  //  break;
  }

  lcd_puts_Pleft( 1*FH, XPSTR("On Time")) ;
  lcd_putcAtt( 11*FW+3, 1*FH, ':', 0 ) ;
	div_t qr ;
	qr = div( g_eeGeneral.totalElapsedTime, 60 ) ;
  putsTime( 9*FW, FH*1, qr.quot, 0, 0 ) ;
  lcd_outdezNAtt( 14*FW, 1*FH, qr.rem, LEADING0, 2 ) ;

  lcd_puts_Pleft( 2*FH, XPSTR("tmain          ms"));
  lcd_outdezAtt(14*FW , 2*FH, (g_timeMain)/20 ,PREC2);
#ifdef PCBX9D
  lcd_puts_Pleft( 3*FH, XPSTR("trefresh       ms"));
  lcd_outdezAtt(14*FW , 3*FH, (g_timeRfsh)/20 ,PREC2);
  lcd_puts_Pleft( 4*FH, XPSTR("tmixer         ms"));
  lcd_outdezAtt(14*FW , 4*FH, (g_timeMixer)/20 ,PREC2);

extern uint32_t MixerRate ;
//	lcd_puts_Pleft( 5*FH, PSTR("Mixer Rate"));
  lcd_outdezAtt(20*FW , 4*FH, MixerRate, 0 ) ;

//	lcd_puts_Pleft( 5*FH, PSTR("Usart Errors"));
//extern uint32_t USART_ERRORS ;
//extern uint32_t USART_ORE ;
//extern uint32_t USART_NE ;
//extern uint32_t USART_FE ;
//extern uint32_t USART_PE ;
//  lcd_outdezAtt(14*FW , 5*FH, USART_ERRORS, 0 ) ;
//  lcd_outdezAtt( 5*FW , 6*FH, USART_ORE, 0 ) ;
//  lcd_outdezAtt( 10*FW , 6*FH, USART_NE, 0 ) ;
//  lcd_outdezAtt( 15*FW , 6*FH, USART_FE, 0 ) ;
//  lcd_outdezAtt( 20*FW , 6*FH, USART_PE, 0 ) ;
  
	lcd_puts_Pleft( 1*FH, XPSTR("ttimer1        us"));
  lcd_outdezAtt(14*FW , 1*FH, (g_timePXX)/2 ,0);
#endif

  
extern uint8_t AudioVoiceCountUnderruns ;
	lcd_puts_Pleft( 5*FH, XPSTR("Voice underruns"));
  lcd_outdezAtt( 20*FW, 5*FH, AudioVoiceCountUnderruns, 0 ) ;

#ifdef PCBSKY
// Debug code
  lcd_puts_Pleft( 7*FH, XPSTR("BT Reply(debug)"));
#ifndef SIMU
extern uint32_t Bt_ok ;
  lcd_outdezAtt( 17*FW , 7*FH, Bt_ok ,0 ) ;

//extern int16_t BtChannels[8] ;

//  lcd_outdezAtt(5*FW , 4*FH, BtChannels[0], 0 ) ;
//  lcd_outdezAtt(10*FW , 4*FH, BtChannels[1], 0 ) ;
//  lcd_outdezAtt(15*FW , 4*FH, BtChannels[2], 0 ) ;
//  lcd_outdezAtt(20*FW , 4*FH, BtChannels[3], 0 ) ;
//  lcd_outdezAtt(5*FW , 5*FH, BtChannels[4], 0 ) ;
//  lcd_outdezAtt(10*FW , 5*FH, BtChannels[5], 0 ) ;
//  lcd_outdezAtt(15*FW , 5*FH, BtChannels[6], 0 ) ;
//  lcd_outdezAtt(20*FW , 5*FH, BtChannels[7], 0 ) ;

//extern uint8_t BtReceived[16] ;
//  lcd_outhex4( 0, 4*FH, (BtReceived[0]<< 8) | BtReceived[1] ) ;
//  lcd_outhex4( 24, 4*FH, (BtReceived[2]<< 8) | BtReceived[3] ) ;
//  lcd_outhex4( 48, 4*FH, (BtReceived[4]<< 8) | BtReceived[5] ) ;
//  lcd_outhex4( 72, 4*FH, (BtReceived[6]<< 8) | BtReceived[7] ) ;
//  lcd_outhex4( 0, 5*FH, (BtReceived[8]<< 8) | BtReceived[9] ) ;
//  lcd_outhex4( 24, 5*FH, (BtReceived[10]<< 8) | BtReceived[11] ) ;
//  lcd_outhex4( 48, 5*FH, (BtReceived[12]<< 8) | BtReceived[13] ) ;
//  lcd_outhex4( 72, 5*FH, (BtReceived[14]<< 8) | BtReceived[15] ) ;

#endif
#endif

  lcd_puts_P( 3*FW,  6*FH, PSTR(STR_MENU_REFRESH));

#ifdef PCBSKY
//extern uint8_t DsmDebug[18] ;
//  lcd_outhex4( 0, 3*FH,  (DsmDebug[17] << 8) | DsmDebug[0] ) ;
//  lcd_outhex4( 24, 3*FH, (DsmDebug[1]<< 8) | DsmDebug[2] ) ;
//  lcd_outhex4( 48, 3*FH, (DsmDebug[3]<< 8) | DsmDebug[4] ) ;
//  lcd_outhex4( 72, 3*FH, (DsmDebug[5]<< 8) | DsmDebug[6] ) ;
//  lcd_outhex4( 96, 3*FH, (DsmDebug[7]<< 8) | DsmDebug[8] ) ;
//  lcd_outhex4( 0, 4*FH, (DsmDebug[9]<< 8) | DsmDebug[10] ) ;
//  lcd_outhex4( 24, 4*FH, (DsmDebug[11]<< 8) | DsmDebug[12] ) ;
//  lcd_outhex4( 48, 4*FH, (DsmDebug[13]<< 8) | DsmDebug[14] ) ;
//  lcd_outhex4( 72, 4*FH, (DsmDebug[15]<< 8) | DsmDebug[16] ) ;
//	lcd_outhex4( 96, 4*FH, (g_model.ppmNCH<< 8) | (g_model.dsmMode& 0x00FF) ) ;

//extern uint8_t DsmPass1[10] ;
//  lcd_outhex4( 0, 5*FH,  (DsmPass1[0]<< 8) | DsmPass1[1] ) ;
//  lcd_outhex4( 24, 5*FH, (DsmPass1[2]<< 8) | DsmPass1[3] ) ;
//  lcd_outhex4( 48, 5*FH, (DsmPass1[4]<< 8) | DsmPass1[5] ) ;
//  lcd_outhex4( 72, 5*FH, (DsmPass1[6]<< 8) | DsmPass1[7] ) ;
//  lcd_outhex4( 96, 5*FH, (DsmPass1[8]<< 8) | DsmPass1[9] ) ;
#endif

	 
//	lcd_outhex4( 0, 4*FH, (frskyStreaming<< 8) | frskyUsrStreaming ) ;
}

uint8_t TrainerMode ;
uint8_t TrainerPos ;
uint8_t TrainerDebugData[16] ;
uint16_t TdebugCounter ;

void setCaptureMode(uint32_t mode) ;
extern struct t_fifo64 CaptureRx_fifo ;

void menuProcTrainDdiag(uint8_t event)
{
	MENU(XPSTR("Train diag"), menuTabStat, e_traindiag, 2, {0} ) ;
	
	int8_t sub = mstate2.m_posVert ;

  lcd_puts_Pleft( 1*FH, XPSTR("Trainer Mode") ) ;
	lcd_putsAttIdx(16*FW, FH, XPSTR("\004NormSer Com1"), TrainerMode, (sub == 1) ? INVERS : 0 ) ;
	if(sub==1)
  {
#ifdef PCBSKY
		uint8_t b = TrainerMode ;
#endif	// PCBSKY
		TrainerMode = checkIncDec16( event, TrainerMode, 0, 2, 0 ) ;
#ifdef PCBSKY
		if ( TrainerMode != b )
		{
			if ( TrainerMode == 2 )
			{
				setCaptureMode( 0 ) ;			
				init_software_com1( 9600, 0 ) ;
			}
			else
			{
				configure_pins( (PIO_PA5 | PIO_PA6), PIN_PERIPHERAL | PIN_INPUT | PIN_PER_A | PIN_PORTA | PIN_NO_PULLUP ) ;
				setCaptureMode( TrainerMode ) ;			
			}
		}
#endif	// PCBSKY
  }
	
	uint32_t i ;
	for ( i = 0 ; i < 16 ; i += 1 )
	{
		if ( TrainerDebugData[i] )
		{
			lcd_putc( i*FW, 3*FH, TrainerDebugData[i] ) ;
		}
		else
		{
			lcd_putc( i*FW, 3*FH, ' ' ) ;
		}
	}

	uint16_t rxchar ;
	while ( ( rxchar = get_fifo64( &CaptureRx_fifo ) ) != 0xFFFF )
	{
		TdebugCounter += 1 ;
		TrainerDebugData[TrainerPos++] = rxchar ;
		if ( TrainerPos > 15 )
		{
			TrainerPos = 0 ;
		}
	}
  
#ifdef PCBSKY

extern uint8_t LineState ;
extern uint16_t BitTime ;
extern uint16_t HtoLtime ;
extern uint16_t LtoHtime ;

extern uint8_t BitState ;
extern uint8_t BitCount ;
extern uint8_t Byte ;
extern uint8_t Tc5Count ;

extern uint16_t SerTimes[] ;
extern uint8_t SerValues[] ;

	lcd_outhex4( 0, 6*FH, TdebugCounter ) ;
	lcd_outhex4( 25, 6*FH, LineState ) ;
	lcd_outhex4( 50, 6*FH, BitTime ) ;
	lcd_outhex4( 75, 6*FH, HtoLtime ) ;
	lcd_outhex4( 100, 6*FH, LtoHtime ) ;

	lcd_outhex4( 0, 7*FH, BitState ) ;
	lcd_outhex4( 25, 7*FH, BitCount ) ;
	lcd_outhex4( 50, 7*FH, Byte ) ;
	
	lcd_outhex4( 100, 7*FH, Tc5Count ) ;

	lcd_outhex4( 0, 4*FH, SerTimes[0] ) ;
	lcd_outhex4( 25, 4*FH, SerTimes[1] ) ;
	lcd_outhex4( 50, 4*FH, SerTimes[2] ) ;
	lcd_outhex4( 75, 4*FH, SerTimes[3] ) ;
	lcd_outhex4( 100, 4*FH, SerTimes[4] ) ;

	lcd_outhex4( 0, 5*FH, SerValues[0] ) ;
	lcd_outhex4( 25, 5*FH, SerValues[1] ) ;
	lcd_outhex4( 50, 5*FH, SerValues[2] ) ;
	lcd_outhex4( 75, 5*FH, SerValues[3] ) ;
	lcd_outhex4( 100, 5*FH, SerValues[4] ) ;

	lcd_outhex4( 25, 2*FH, SerTimes[14] ) ;
	lcd_outhex4( 50, 2*FH, SerValues[14] ) ;
	lcd_outhex4( 75, 2*FH, SerTimes[15] ) ;
	lcd_outhex4( 100, 2*FH, SerValues[15] ) ;

#endif	// PCBSKY


}

uint8_t DsmFrameRequired ;

void menuProcDsmDdiag(uint8_t event)
{
	
	MENU(XPSTR("DSM diag"), menuTabStat, e_dsm, 2, {0} ) ;
  
	int8_t sub = mstate2.m_posVert ;

  lcd_outhex4( 0, 2*FH, DsmFrameRequired ) ;
  if(sub==1)
  {
		lcd_char_inverse( 0, 2*FH, 20, 0 ) ;
		DsmFrameRequired = checkIncDec16( event, DsmFrameRequired, 0, 255, 0 ) ;
  }

//extern uint16_t DsmDbgCounters[] ;
//  lcd_outhex4( 0, 3*FH,  DsmDbgCounters[0] ) ;
//  lcd_outhex4( 24, 3*FH, DsmDbgCounters[1] ) ;
//  lcd_outhex4( 48, 3*FH, DsmDbgCounters[2] ) ;
//  lcd_outhex4( 72, 3*FH, DsmDbgCounters[3] ) ;
//  lcd_outhex4( 96, 3*FH, DsmDbgCounters[4] ) ;
//  lcd_outhex4( 0, 4*FH,  DsmDbgCounters[5] ) ;
//  lcd_outhex4( 24, 4*FH, DsmDbgCounters[6] ) ;
//  lcd_outhex4( 48, 4*FH, DsmDbgCounters[7] ) ;

  lcd_outhex4( 0, 1*FH, g_model.ppmNCH ) ;
  lcd_outhex4( 24, 1*FH, g_model.dsmMode ) ;

#ifdef PCBSKY
extern uint8_t DsmDebug[18] ;
extern uint8_t DsmControlDebug[18] ;
extern uint16_t DsmControlCounter ;

  lcd_outhex4( 0, 3*FH,  (DsmDebug[17] << 8) | DsmDebug[0] ) ;
  lcd_outhex4( 24, 3*FH, (DsmDebug[1]<< 8) | DsmDebug[2] ) ;
  lcd_outhex4( 48, 3*FH, (DsmDebug[3]<< 8) | DsmDebug[4] ) ;
  lcd_outhex4( 72, 3*FH, (DsmDebug[5]<< 8) | DsmDebug[6] ) ;
  lcd_outhex4( 96, 3*FH, (DsmDebug[7]<< 8) | DsmDebug[8] ) ;
  lcd_outhex4( 0, 4*FH, (DsmDebug[9]<< 8) | DsmDebug[10] ) ;
  lcd_outhex4( 24, 4*FH, (DsmDebug[11]<< 8) | DsmDebug[12] ) ;
  lcd_outhex4( 48, 4*FH, (DsmDebug[13]<< 8) | DsmDebug[14] ) ;
  lcd_outhex4( 72, 4*FH, (DsmDebug[15]<< 8) | DsmDebug[16] ) ;

  lcd_outhex4( 0, 6*FH,  (DsmControlDebug[17] << 8) | DsmControlDebug[0] ) ;
  lcd_outhex4( 24, 6*FH, (DsmControlDebug[1]<< 8) | DsmControlDebug[2] ) ;
  lcd_outhex4( 48, 6*FH, (DsmControlDebug[3]<< 8) | DsmControlDebug[4] ) ;
  lcd_outhex4( 72, 6*FH, (DsmControlDebug[5]<< 8) | DsmControlDebug[6] ) ;
  lcd_outhex4( 96, 6*FH, (DsmControlDebug[7]<< 8) | DsmControlDebug[8] ) ;
  lcd_outhex4( 0, 7*FH, (DsmControlDebug[9]<< 8) | DsmControlDebug[10] ) ;
  lcd_outhex4( 24, 7*FH, (DsmControlDebug[11]<< 8) | DsmControlDebug[12] ) ;
  lcd_outhex4( 48, 7*FH, (DsmControlDebug[13]<< 8) | DsmControlDebug[14] ) ;
  lcd_outhex4( 72, 7*FH, (DsmControlDebug[15]<< 8) | DsmControlDebug[16] ) ;
  
	lcd_outhex4( 96, 7*FH, DsmControlCounter ) ;

extern uint16_t TelemetryDebug ;
extern uint16_t TelemetryDebug1 ;
extern uint16_t TelemetryDebug2 ;
	lcd_outhex4( 48, FH, TelemetryDebug2 ) ;
	lcd_outhex4( 72, FH, TelemetryDebug1 ) ;
	lcd_outhex4( 96, FH, TelemetryDebug ) ;

extern uint8_t DebugDsmPass ;
	lcd_outhex4( 96, 2*FH, DebugDsmPass ) ;

#endif
}


#ifdef PCBSKY
struct t_i2cTime
{
	uint8_t setCode ;
	uint8_t Time[7] ;
} TimeToSet ;
#endif

//#ifdef PCBX9D
//t_time TimeToSet ;
//#endif


void dispMonth( uint8_t x, uint8_t y, uint32_t month, uint8_t attr)
{
	if ( month > 12 )
	{
		month = 0 ;		
	}
  lcd_putsAttIdx( x, y, PSTR(STR_MONTHS),month, attr ) ;
}

void disp_datetime( uint8_t y )
{
	lcd_puts_Pleft( y, XPSTR("  -   -     :  :"));
	lcd_outdezNAtt( 18*FW-2, y, Time.second, LEADING0, 2 ) ;
	lcd_outdezNAtt( 15*FW-1, y, Time.minute, LEADING0, 2 ) ;
	lcd_outdezNAtt( 12*FW,   y, Time.hour, LEADING0, 2 ) ;
	lcd_outdezNAtt( 2*FW,    y, Time.date, LEADING0, 2 ) ;
	lcd_outdezNAtt( 10*FW,   y, Time.year, LEADING0, 4 ) ;
	dispMonth( 3*FW, y, Time.month, 0 ) ;
}

t_time EntryTime ;

#ifdef REVX
#define DATE_COUNT_ITEMS	8
#else
#define DATE_COUNT_ITEMS	7
#endif

void menuProcDate(uint8_t event)
{
	TITLE(PSTR(STR_DATE_TIME));
	static MState2 mstate2;
	static const uint8_t mstate_tab[] = {0} ;
	
//	if (SubMenuFromIndex)
//	{
		event = mstate2.check(event,0,NULL,0,mstate_tab,DIM(mstate_tab)-1,DATE_COUNT_ITEMS-1) ;
//	}
//	else
//	{
//		event = mstate2.check(event,e_Date,menuTabDiag,DIM(menuTabDiag),mstate_tab,DIM(mstate_tab)-1,DATE_COUNT_ITEMS-1) ;
//	}

  switch(event)
  {
    case EVT_ENTRY :
			{	
				t_time *p = &EntryTime ;

				if ( Time.date == 0 )
				{
					Time.date = 1 ;
				}
				if ( Time.month == 0 )
				{
					Time.month = 1 ;
				}
				p->second = Time.second ;
				p->minute = Time.minute ;
				p->hour   = Time.hour ;
				p->date   = Time.date ;
				p->month  = Time.month ;
				p->year   = Time.year ;
				if ( p->year < 2000 )
				{
					p->year = 2000 + ( p->year % 100 ) ;
					
				}
			}
    break;
		
    case EVT_KEY_LONG(KEY_MENU) :
#ifdef PCBSKY
			TimeToSet.setCode = 0x74 ;		// Tiny SET TIME CODE command
			TimeToSet.Time[0] = EntryTime.second ;
			TimeToSet.Time[1] = EntryTime.minute ;
			TimeToSet.Time[2] = EntryTime.hour ;
			TimeToSet.Time[3] = EntryTime.date ;
			TimeToSet.Time[4] = EntryTime.month ;
			TimeToSet.Time[5] = (uint8_t) EntryTime.year ;
			TimeToSet.Time[6] = EntryTime.year >> 8 ;
#endif
#ifdef REVX
			writeRTC( (uint8_t *) &TimeToSet.Time[0] ) ;
#else
#ifdef PCBSKY
			write_coprocessor( (uint8_t *) &TimeToSet, 8 ) ;
#endif
#ifdef PCBX9D
			rtcSetTime( &EntryTime ) ;
#endif
#endif
      killEvents(event);
			s_editMode = 0 ;
    break;
	}		 
#ifdef PCBX9D
		disp_datetime( 1*FH ) ;
#endif

#if defined(REVB) || defined(PCBX9D)
		disp_datetime( 1*FH ) ;

//    lcd_outhex4( 17*FW+4, 6*FH, (Coproc_valid << 8 ) + Coproc_read ) ;
#ifndef REVX
#ifdef PCBSKY
extern uint8_t Co_proc_status[] ;
		lcd_outdezAtt( 21*FW, 7*FH, (uint8_t)Co_proc_status[8], 0 ) ;		// Co-Proc temperature

		uint8_t temp = (uint8_t)Co_proc_status[9] ; 
    lcd_putc( 18*FW, 6*FH, temp & 0x40 ? '1' : '0' ) ;
    lcd_putc( 19*FW, 6*FH, temp & 0x08 ? '1' : '0' ) ;
    lcd_putc( 20*FW, 6*FH, temp & 0x02 ? '1' : '0' ) ;
#endif
#endif
    int8_t  sub    = mstate2.m_posVert;

		for (uint8_t subN=1; subN<8; subN++)
		{
	  	uint8_t attr = ((sub==subN) ? InverseBlink : 0);
			switch ( subN )
			{
				case 1 :
			  	lcd_puts_Pleft( 2*FH, PSTR(STR_SEC) );
					lcd_outdezNAtt( 7*FW, 2*FH, EntryTime.second, LEADING0|attr, 2 ) ;
			  	if(sub==subN)  EntryTime.second = checkIncDec( event,  EntryTime.second, 0, 59, 0 ) ;
				break ;
				case 2 :
			  	lcd_puts_Pleft( 3*FH, PSTR(STR_MIN_SET) );
					lcd_outdezNAtt( 7*FW, 3*FH, EntryTime.minute, LEADING0|attr, 2 ) ;
			  	if(sub==subN)  EntryTime.minute = checkIncDec( event,  EntryTime.minute, 0, 59, 0 ) ;
				break ;
				case 3 :
			  	lcd_puts_Pleft( 4*FH, PSTR(STR_HOUR_MENU_LONG) );
					lcd_outdezNAtt( 7*FW, 4*FH, EntryTime.hour, LEADING0|attr, 2 ) ;
			  	if(sub==subN)  EntryTime.hour = checkIncDec( event,  EntryTime.hour, 0, 23, 0 ) ;
				break ;
				case 4 :
			  	lcd_puts_Pleft( 5*FH, PSTR(STR_DATE) );
					lcd_outdezNAtt( 7*FW, 5*FH, EntryTime.date, LEADING0|attr, 2 ) ;
			  	if(sub==subN)  EntryTime.date = checkIncDec( event,  EntryTime.date, 1, 31, 0 ) ;
				break ;
				case 5 :
			  	lcd_puts_Pleft( 6*FH, PSTR(STR_MONTH) );
					dispMonth( 5*FW+3, 6*FH, EntryTime.month, attr ) ;
			  	if(sub==subN)  EntryTime.month = checkIncDec( event,  EntryTime.month, 1, 12, 0 ) ;
				break ;
				case 6 :
#ifndef REVX
			  	lcd_puts_Pleft( 7*FH, PSTR(STR_YEAR_TEMP) );
#else			  	
					lcd_puts_Pleft( 7*FH, PSTR(STR_YEAR) );
#endif
					lcd_outdezNAtt( 9*FW-2, 7*FH, EntryTime.year, LEADING0|attr, 4 ) ;
			  	if(sub==subN)  EntryTime.year = checkIncDec16( event,  EntryTime.year, 0, 2999, 0 ) ;
				break ;
#ifdef REVX
				case 7 :
					uint8_t previous = g_eeGeneral.rtcCal ;
			  	lcd_puts_P( 12*FW, 5*FH, PSTR(STR_CAL) );
					lcd_outdezAtt( 20*FW, 5*FH, g_eeGeneral.rtcCal, attr ) ;
			  	if(sub==subN) CHECK_INCDEC_H_GENVAR( event, g_eeGeneral.rtcCal, -127, 127 ) ;
					if ( g_eeGeneral.rtcCal != previous )
					{
						previous = g_eeGeneral.rtcCal ;
						if ( g_eeGeneral.rtcCal < 0 )
						{
							previous = -g_eeGeneral.rtcCal | 0x80 ;
						}
						setRtcCAL( previous ) ;						
					}
				break ;
#endif
			}
		}
#endif
}

static struct fileControl FileControl = {	0,0,0,0, {0,0,0,0} } ;
extern DIR Dj ;

void menuProcRestore(uint8_t event)
{
	FRESULT fr ;
	struct fileControl *fc = &FileControl ;
	uint32_t i ;

  TITLE( "RESTORE MODEL" ) ;

  switch(event)
	{
    case EVT_ENTRY:
			fr = f_chdir( (TCHAR *)"/MODELS" ) ;
			if ( fr == FR_OK )
			{
				fc->index = 0 ;
				fr = f_opendir( &Dj, (TCHAR *) "." ) ;
				if ( fr == FR_OK )
				{
					fc->ext[0] = 'E' ;	// 'E'
					fc->ext[1] = 'E' ;	// 'E'
					fc->ext[2] = 'P' ;	// 'P'
					fc->ext[3] = 'M' ;	// 'M'
					fc->index = 0 ;
					fc->nameCount = fillNames( 0, fc ) ;
					fc->hpos = 0 ;
					fc->vpos = 0 ;
				}
			}
    break ;
	}
	
	i = fileList( event, &FileControl ) ;
	if ( i == 1 )	// Select
	{
		// Restore the model
		TCHAR RestoreFilename[60] ;
		cpystr( cpystr( (uint8_t *)RestoreFilename, (uint8_t *)"/MODELS/" ), (uint8_t *)Filenames[fc->vpos] ) ;
		WatchdogTimeout = 200 ;		// 2 seconds
		BackResult = ee32RestoreModel( RestoreIndex, RestoreFilename ) ;
		AlertType = MESS_TYPE ;
		AlertMessage = BackResult ;
    killEvents(event) ;
    popMenu() ;
	}
	else if ( i == 2 )	// EXIT
	{
    killEvents(event) ;
    popMenu() ;
	}
}

void menuProcSelectMvoiceFile(uint8_t event)
{
	FRESULT fr ;
	struct fileControl *fc = &FileControl ;
	uint32_t i ;

  TITLE( "SELECT FILE" ) ;

  switch(event)
	{
    case EVT_ENTRY:
			fr = f_chdir( (TCHAR *)"\\voice\\modelNames" ) ;
			if ( fr == FR_OK )
			{
				fc->index = 0 ;
				fr = f_opendir( &Dj, (TCHAR *) "." ) ;
				if ( fr == FR_OK )
				{
					fc->ext[0] = 'W' ;
					fc->ext[1] = 'A' ;
					fc->ext[2] = 'V' ;
					fc->ext[3] = '\0';
					fc->index = 0 ;
					fc->nameCount = fillNames( 0, fc ) ;
					fc->hpos = 0 ;
					fc->vpos = 0 ;
				}
			}
    break ;
	}
	
	i = fileList( event, &FileControl ) ;
	if ( i == 1 )	// Select
	{
		// Restore the model
		cpystr( (uint8_t *)SelectedVoiceFileName, (uint8_t *)Filenames[fc->vpos] ) ;
		killEvents(event) ;
    popMenu() ;
	}
	else if ( i == 2 )	// EXIT
	{
    killEvents(event) ;
    popMenu() ;
	}
	else if ( i == 3 )	// Tag
	{
		char name[12] ;
    killEvents(event) ;
		copyFileName( name, Filenames[fc->vpos] ) ;
		putNamedVoiceQueue( name, 0xC000 ) ;
	}
	FileSelectResult = i ;
}

void menuProcSelectUvoiceFile(uint8_t event)
{
	FRESULT fr ;
	struct fileControl *fc = &FileControl ;
	uint32_t i ;

  TITLE( "SELECT FILE" ) ;

  switch(event)
	{
    case EVT_ENTRY:
			fr = f_chdir( (TCHAR *)"\\voice\\user" ) ;
			if ( fr == FR_OK )
			{
				fc->index = 0 ;
				fr = f_opendir( &Dj, (TCHAR *) "." ) ;
				if ( fr == FR_OK )
				{
					fc->ext[0] = 'W' ;
					fc->ext[1] = 'A' ;
					fc->ext[2] = 'V' ;
					fc->ext[3] = '\0';
					fc->index = 0 ;
					fc->nameCount = fillNames( 0, fc ) ;
					fc->hpos = 0 ;
					fc->vpos = 0 ;
				}
			}
    break ;
	}
	
	i = fileList( event, &FileControl ) ;
	if ( i == 1 )	// Select
	{
		// Restore the model
		cpystr( (uint8_t *)SelectedVoiceFileName, (uint8_t *)Filenames[fc->vpos] ) ;
		killEvents(event) ;
    popMenu() ;
	}
	else if ( i == 2 )	// EXIT
	{
    killEvents(event) ;
    popMenu() ;
	}
	else if ( i == 3 )	// Tag
	{
		char name[12] ;
    killEvents(event) ;
		copyFileName( name, Filenames[fc->vpos] ) ;
		putNamedVoiceQueue( name, 0xD000 ) ;
	}
	FileSelectResult = i ;
}



void menuProcBattery(uint8_t event)
{
//#ifdef PCBSKY
//	uint32_t current_scale ;
//#endif

//  TITLE(PSTR(STR_BATTERY));
	MENU(PSTR(STR_BATTERY), menuTabStat, e_battery, 1, {0} ) ;

  switch(event)
  {
  	case EVT_KEY_BREAK(KEY_MENU):
      g_timeMain = 0;
#ifdef PCBSKY
			Current_max = 0 ;
#endif
    break;
    case EVT_KEY_LONG(KEY_MENU):
#ifdef PCBSKY
			MAh_used = 0 ;
			Current_used = 0 ;
#endif
      audioDefevent(AU_MENUS) ;
    	killEvents(event) ;
    break;

//    case EVT_KEY_FIRST(KEY_DOWN):
//			Program_coprocessor = 1 ;
//    	killEvents(event) ;
//    break;
//    case EVT_KEY_FIRST(KEY_EXIT):
//      chainMenu(menuProc0);
//    break;
//    case EVT_KEY_FIRST(KEY_UP):
//      chainMenu(menuProcStatistic);
//    	killEvents(event) ;
//    break;
  }

		lcd_puts_Pleft( 2*FH, PSTR(STR_Battery));
		putsVolts( 13*FW, 2*FH, g_vbat100mV, 0 ) ;

#ifdef PCBSKY
#ifdef REVB
		Current_sum += Current_current ;
		if ( ++Current_count > 49 )
		{
			Current = Current_sum / 500 ;
			Current_sum = 0 ;
			Current_count = 0 ;
		}
		lcd_puts_Pleft( 3*FH, PSTR(STR_CURRENT_MAX));
	  lcd_outdezAtt( 13*FW, 3*FH, Current, 0 ) ;
	  lcd_outdezAtt( 20*FW, 3*FH, Current_max/10, 0 ) ;
		lcd_puts_Pleft( 4*FH, XPSTR("mAh\017[MENU]"));
	  lcd_outdezAtt( 12*FW, 4*FH, MAh_used + Current_used/3600 ,PREC1 ) ;
		lcd_puts_Pleft( 6*FH, PSTR(STR_CPU_TEMP_MAX));
	  lcd_outdezAtt( 12*FW-2, 6*FH, (((((int32_t)Temperature - 838 ) * 621 ) >> 11 ) - 20) ,0 ) ;
	  lcd_outdezAtt( 20*FW-2, 6*FH, (((((int32_t)Max_temperature - 838 ) * 621 ) >> 11 ) - 20) ,0 ) ;

#ifdef PCBSKY
static uint8_t count = 0 ;
//static uint16_t value ;
static uint32_t sum ;
	if ( count == 0 )
	{
//		value = sum >> 6 ;
		sum = Analog_values[8] ;
	}
	else
	{
		sum += Analog_values[8] ;
	}
	count += 1 ;
	if ( count > 63 )
	{
		count = 0 ;
	}
//	lcd_outhex4( 100, 7*FH, value ) ;
#endif

// Temp test code for co-processor
//		lcd_puts_Pleft( 1*FH, PSTR("(test) Co Proc"));
//    lcd_outhex4( 15*FW-3, 1*FH, (Coproc_valid << 8 ) + Coproc_read ) ;
//extern uint8_t Co_proc_status[] ;
// Rotary encoder test/debug
//extern volatile int32_t Rotary_position ;
//extern volatile int32_t Rotary_count ;
//    lcd_outhex4( 15*FW-3, 0*FH, ( Rotary_count << 8 ) | Rotary_position ) ;
#endif

		disp_datetime( 5*FH ) ;
#endif

//#ifdef PCBSKY
//extern uint16_t Current_adjust ;
//	lcd_outhex4( 0, 7*FH, Current_current ) ;
//	lcd_outhex4( 30, 7*FH, Analog_values[8] ) ;
//	lcd_outhex4( 60, 7*FH, Current_adjust ) ;	
//#endif

}



//#ifdef JETI
#ifdef REVX

void menuProcJeti(uint8_t event)
{
    TITLE(XPSTR("JETI"));

    switch(event)
    {
    //case EVT_KEY_FIRST(KEY_MENU):
    //  break;
    case EVT_KEY_FIRST(KEY_EXIT):
//        JETI_DisableRXD();
        chainMenu(menuProc0);
        break;
    }

    for (uint8_t i = 0; i < 16; i++)
    {
        lcd_putcAtt((i+2)*FW, 1*FH, JetiBuffer[i], 0 ) ;
        lcd_putcAtt((i+2)*FW, 2*FH, JetiBuffer[i+16], 0 ) ;
//        lcd_putcAtt((i+2)*FW, 3*FH, JetiBuffer[i], 0 ) ;
//        lcd_putcAtt((i+2)*FW, 4*FH, JetiBuffer[i+16], 0 ) ;
    }

extern struct t_16bit_fifo32 Jeti_fifo ;
extern uint16_t Debug_frsky2 ;
extern uint16_t Debug_frsky3 ;

	lcd_outhex4( 50,  0*FH, JetiIndex ) ;
	lcd_outhex4( 75,  0*FH, Debug_frsky2 ) ;
	lcd_outhex4( 100,  0*FH, Debug_frsky3 ) ;

	lcd_outhex4( 0,  3*FH, Jeti_fifo.fifo[0] ) ;
	lcd_outhex4( 25, 3*FH, Jeti_fifo.fifo[1] ) ;
	lcd_outhex4( 50, 3*FH, Jeti_fifo.fifo[2] ) ;
	lcd_outhex4( 75, 3*FH, Jeti_fifo.fifo[3] ) ;
	lcd_outhex4( 100,3*FH, Jeti_fifo.fifo[4] ) ;

	lcd_outhex4( 0,  4*FH, Jeti_fifo.fifo[5] ) ;
	lcd_outhex4( 25, 4*FH, Jeti_fifo.fifo[6] ) ;
	lcd_outhex4( 50, 4*FH, Jeti_fifo.fifo[7] ) ;
	lcd_outhex4( 75, 4*FH, Jeti_fifo.fifo[8] ) ;
	lcd_outhex4( 100,4*FH, Jeti_fifo.fifo[9] ) ;

	lcd_outhex4( 0,  5*FH, Jeti_fifo.fifo[10] ) ;
	lcd_outhex4( 25, 5*FH, Jeti_fifo.fifo[11] ) ;
	lcd_outhex4( 50, 5*FH, Jeti_fifo.fifo[12] ) ;
	lcd_outhex4( 75, 5*FH, Jeti_fifo.fifo[13] ) ;
	lcd_outhex4( 100,5*FH, Jeti_fifo.fifo[14] ) ;

	lcd_outhex4( 0,  6*FH, Jeti_fifo.fifo[15] ) ;
	lcd_outhex4( 25, 6*FH, Jeti_fifo.fifo[16] ) ;
	lcd_outhex4( 50, 6*FH, Jeti_fifo.fifo[17] ) ;
	lcd_outhex4( 75, 6*FH, Jeti_fifo.fifo[18] ) ;
	lcd_outhex4( 100,6*FH, Jeti_fifo.fifo[19] ) ;

	lcd_outhex4( 0,  7*FH, Jeti_fifo.fifo[20] ) ;
	lcd_outhex4( 25, 7*FH, Jeti_fifo.fifo[21] ) ;
	lcd_outhex4( 50, 7*FH, Jeti_fifo.fifo[22] ) ;
	lcd_outhex4( 75, 7*FH, Jeti_fifo.fifo[23] ) ;
	lcd_outhex4( 100,7*FH, Jeti_fifo.fifo[24] ) ;

//extern uint8_t Scount1 ;
//extern uint8_t Scount2 ;
//extern uint8_t Scount3 ;
//extern uint16_t Sstat1 ;
//extern uint16_t Sstat2 ;
//extern uint16_t Sstat3 ;
//extern uint8_t JetiIndex ;
//extern uint8_t TelemetryType ;


//	lcd_outhex4( 0, 5*FH, JetiIndex ) ;
//	lcd_outhex4( 25, 5*FH, TelemetryType ) ;

//	lcd_outhex4( 50, 6*FH, Sstat1 ) ;
//	lcd_outhex4( 75, 6*FH, Sstat2 ) ;
//	lcd_outhex4( 100, 6*FH, Sstat3 ) ;
	
//	lcd_outhex4( 0, 7*FH, Scount1 ) ;
//	lcd_outhex4( 25, 7*FH, Scount2 ) ;
//	lcd_outhex4( 50, 7*FH, Scount3 ) ;


//    if (JetiBufferReady)
//    {
//        JETI_EnableTXD();

    switch(event)
		{
    	case EVT_KEY_FIRST(KEY_UP):
				jeti_keys &= JETI_KEY_UP ;
			break ;

    	case EVT_KEY_FIRST(KEY_DOWN):
				jeti_keys &= JETI_KEY_DOWN ;
			break ;
    	
			case EVT_KEY_FIRST(KEY_LEFT):
				jeti_keys &= JETI_KEY_LEFT ;
			break ;
    	
			case EVT_KEY_FIRST(KEY_RIGHT):
				jeti_keys &= JETI_KEY_RIGHT ;
			break ;
		}
//        if (keyState((EnumKeys)(KEY_UP))) jeti_keys &= JETI_KEY_UP;
//        if (keyState((EnumKeys)(KEY_DOWN))) jeti_keys &= JETI_KEY_DOWN;
//        if (keyState((EnumKeys)(KEY_LEFT))) jeti_keys &= JETI_KEY_LEFT;
//        if (keyState((EnumKeys)(KEY_RIGHT))) jeti_keys &= JETI_KEY_RIGHT;

        JetiBufferReady = 0;    // invalidate buffer
				
				if ( jeti_keys != JETI_KEY_NOCHANGE )
				{
					JetiIndex = 0 ;
					jetiSendWord( jeti_keys ) ;
				}

//        JETI_putw((uint16_t) jeti_keys);
//        _delay_ms (1);
//        JETI_DisableTXD();

        jeti_keys = JETI_KEY_NOCHANGE ;
//    }
}
#endif

void menuProcStatistic(uint8_t event)
{
//  TITLE(PSTR(STR_STAT));
	MENU(PSTR(STR_STAT), menuTabStat, e_stat1, 1, {0} ) ;
//  switch(event)
//  {
//  	case EVT_KEY_FIRST(KEY_UP):
//  	  chainMenu(menuProcStatistic2);
//  	break;
//		case EVT_KEY_FIRST(KEY_DOWN):
//  	  chainMenu(menuProcBattery) ;
//		break ;
//  	case EVT_KEY_FIRST(KEY_EXIT):
//  	  chainMenu(menuProc0);
//  	break;
//  }

  lcd_puts_P(  1*FW, FH*1, XPSTR("TME\017TSW"));
  putsTime(    6*FW, FH*1, s_timeCumAbs, 0, 0);
  putsTime(   12*FW, FH*1, s_timer[0].s_timeCumSw,      0, 0);

  lcd_puts_P(  1*FW, FH*2, XPSTR("STK\017ST%"));
  putsTime(    6*FW, FH*2, s_timer[0].s_timeCumThr, 0, 0);
  putsTime(   12*FW, FH*2, s_timer[0].s_timeCum16ThrP/16, 0, 0);

  lcd_puts_P( 15*FW, FH*0, XPSTR("TOT"));
  putsTime(   12*FW, FH*0, s_timeCumTot, 0, 0);

  uint16_t traceRd = s_traceCnt>MAXTRACE ? s_traceWr : 0;
  uint8_t x=5;
  uint8_t y=60;
  lcd_hline(x-3,y,120+3+3);
  lcd_vline(x,y-32,32+3);

  for(uint8_t i=0; i<120; i+=6)
  {
    lcd_vline(x+i+6,y-1,3);
  }
  for(uint8_t i=1; i<=120; i++)
  {
    lcd_vline(x+i,y-s_traceBuf[traceRd],s_traceBuf[traceRd]);
    traceRd++;
    if(traceRd>=MAXTRACE) traceRd=0;
    if(traceRd==s_traceWr) break;
  }

}

void resetTimern( uint32_t timer )
{
  struct t_timer *tptr = &s_timer[timer] ;
	tptr->s_timerState = TMR_OFF; //is changed to RUNNING dep from mode
  tptr->s_timeCumThr=0;
  tptr->s_timeCumSw=0;
  tptr->s_timeCum16ThrP=0;
	tptr->s_sum = 0 ;
	tptr->last_tmr = g_model.timer[timer].tmrVal ;
	tptr->s_timerVal = ( g_model.timer[timer].tmrDir ) ? 0 : tptr->last_tmr ;
}

void resetTimer1()
{
  s_timeCumAbs=0;
	resetTimern( 0 ) ;
}

void resetTimer2()
{
	resetTimern( 1 ) ;
}

void resetTimer()
{
	resetTimer1() ;
	resetTimer2() ;
}

//extern int8_t *TrimPtr[4] ;
#ifdef FRSKY
int16_t AltOffset = 0 ;
#endif

void displayTemp( uint8_t sensor, uint8_t x, uint8_t y, uint8_t size )
{
	putsTelemetryChannel( x, y, (int8_t)sensor+TEL_ITEM_T1-1, FrskyHubData[FR_TEMP1+sensor-1], size | LEFT, 
																( size & DBLSIZE ) ? (TELEM_LABEL | TELEM_UNIT_LEFT) : (TELEM_LABEL | TELEM_UNIT) ) ;
}


static int8_t io_subview = 0 ;

void switchDisplay( uint8_t j, uint8_t a )
{
	uint8_t b = a + 3 ;
	uint8_t y = 4*FH ;
#ifdef PCBSKY
	for(uint8_t i=a; i<b; y += FH, i += 1 )
	{
//#if REVX		
		if ( i == 2 )	// ELE
		{
			if ( g_eeGeneral.switchMapping & USE_ELE_3POS )
			{
				uint8_t k = HSW_Ele3pos0 ;
				k += switchPosition(k) ;
				lcd_putsAttIdx((2+j*15)*FW-2, y, PSTR(SWITCHES_STR), k-HSW_OFFSET, 0) ;
				continue ;
			}
			if ( g_eeGeneral.switchMapping & USE_ELE_6POS )
			{
				uint8_t k = HSW_Ele6pos0 ;
				k += switchPosition(k) ;
				lcd_putsAttIdx((2+j*15)*FW-2, y, PSTR(SWITCHES_STR), k-HSW_OFFSET, 0) ;
				continue ;
			}
		}
		if ( i == 1 )	// RUD
		{
			if ( g_eeGeneral.switchMapping & USE_RUD_3POS )
			{
				uint8_t k = HSW_Rud3pos0 ;
				k += switchPosition(k) ;
				lcd_putsAttIdx((2+j*15)*FW-2, y, PSTR(SWITCHES_STR), k-HSW_OFFSET, 0) ;
				continue ;
			}
		}
		if ( i == 6 )	// AIL
		{
			if ( g_eeGeneral.switchMapping & USE_AIL_3POS )
			{
				uint8_t k = HSW_Ail3pos0 ;
				k += switchPosition(k) ;
				lcd_putsAttIdx((2+j*15)*FW-2, y, PSTR(SWITCHES_STR), k-HSW_OFFSET, 0) ;
				continue ;
			}
		}
		if ( i == 7 )	// GEA
		{
			if ( g_eeGeneral.switchMapping & USE_GEA_3POS )
			{
				uint8_t k = HSW_Gear3pos0 ;
				k += switchPosition(k) ;
				lcd_putsAttIdx((2+j*15)*FW-2, y, PSTR(SWITCHES_STR), k-HSW_OFFSET, 0) ;
				continue ;
			}
		}
//#endif
		lcd_putsAttIdx((2+j*15)*FW-2, y, PSTR(SWITCHES_STR), i, getSwitch(i+1, 0) ? INVERS : 0) ;
	}
#endif
#ifdef PCBX9D
	for(uint8_t i=a; i<b; y += FH, i += 1 )
	{
#ifdef REV9E
		if ( i < 18 )
#else
		if ( i < 8 )
#endif	// REV9E
		{
			lcd_putsAttIdx((2+j*15)*FW-2, y, PSTR(HW_SWITCHES_STR), i, 0 ) ;
			uint32_t pos = switchPosition( i ) ;
			lcd_putc((4+j*15)*FW-2, y, PSTR(HW_SWITCHARROW_STR)[pos] ) ;
		}
		else
		{
#ifdef REV9E
			lcd_putsAttIdx((2+j*15)*FW-2, y, PSTR(SWITCHES_STR), i-10+1, getSwitch(i-10+2, 0) ? INVERS : 0 ) ;
#else
			lcd_putsAttIdx((2+j*15)*FW-2, y, PSTR(SWITCHES_STR), i+1, getSwitch(i+2, 0) ? INVERS : 0 ) ;
#endif	// REV9E
		}
	}
#endif

}

#define MAIN_POPUP_ENABLED	1

void menuProc0(uint8_t event)
{
  static uint8_t trimSwLock;
  uint8_t view = g_model.mview & 0xf;
  uint8_t tview = g_model.mview & 0x70 ;
	if ( tview > 0x50 )
	{
		tview = 0x50 ;
	}
//    static uint8_t displayCount = 0;
	
	StickScrollAllowed = 0 ;

#ifdef MAIN_POPUP_ENABLED

	if ( ! PopupData.PopupActive )
	{
		switch(event)
		{
    	case EVT_KEY_LONG(KEY_MENU) : // Nav Popup
	    case EVT_KEY_LONG(BTN_RE) :		// Nav Popup
				PopupData.PopupActive = 2 ;
				PopupData.PopupIdx = 0 ;
      	killEvents(event) ;
				event = 0 ;
    	break ;

#ifndef REV9E
	    case EVT_KEY_BREAK(KEY_MENU) :			// ***************
#endif	// nREV9E
			case EVT_KEY_BREAK(BTN_RE) :
				if ( view == e_telemetry )
				{
					PopupData.PopupActive = 1 ;
					PopupData.PopupIdx = 0 ;
      		killEvents(event) ;
					event = 0 ;
				}
    	break ;

#ifdef REV9E
	    case EVT_KEY_BREAK(KEY_MENU) :			// ***************
#endif	// REV9E
  	  case EVT_KEY_BREAK(KEY_RIGHT) :
        if(view <= e_inputs1)
				{
					int8_t x ;
					x = io_subview ;
#ifdef REV9E
					if ( ++x > ((view == e_inputs1) ? 6 : 2) ) x = 0 ;
#else
					if ( ++x > ((view == e_inputs1) ? 4 : 2) ) x = 0 ;
#endif	// REV9E
					io_subview = x ;
				}	
        if(view == e_telemetry)
				{
					tview += 0x10 ;
					if ( tview > ( ( FrskyTelemetryType != 2 ) ? 0x40 : 0x50 ) )
					{
						tview = 0 ;
					}
					
            g_model.mview = e_telemetry | tview ;
						eeModelChanged() ;
            //            STORE_GENERALVARS;     //eeWriteGeneral();
            //            eeDirty(EE_GENERAL);
            audioDefevent(AU_MENUS);
        }
      break ;

#ifndef REV9E
	    case EVT_KEY_BREAK(KEY_LEFT):
        if(view <= e_inputs1)
				{
					int8_t x ;
					x = io_subview ;
					if ( --x < 0 ) x = (view == e_inputs1) ? 4 : 2 ;
					io_subview = x ;
				}	
        else if(view == e_telemetry)
				{
					tview -= 0x10 ;
					if ( tview > 0x60 )
					{
						tview = ( FrskyTelemetryType != 2 ) ? 0x40 : 0x50 ;
					}
					
          g_model.mview = e_telemetry | tview ;
					eeModelChanged() ;
          audioDefevent(AU_MENUS) ;
        }
      break ;
#endif	// nREV9E

#ifndef REV9E
	    case EVT_KEY_LONG(KEY_RIGHT) :
        pushMenu(menuProcModelSelect) ;
        scroll_disabled = 1 ;
        killEvents(event) ;
			break ;
    
			case EVT_KEY_LONG(KEY_LEFT) :
        pushMenu(menuProcIndex) ;
        scroll_disabled = 1 ;
        killEvents(event) ;
			break ;
#endif	// nREV9E
    
			case EVT_KEY_FIRST(KEY_EXIT) :
        if(s_timer[0].s_timerState==TMR_BEEPING)
				{
          s_timer[0].s_timerState = TMR_STOPPED ;
          audioDefevent(AU_MENUS) ;
        }
        else if (view == e_telemetry)
				{
          resetTelemetry() ;
          audioDefevent(AU_MENUS) ;
        }
			break ;

#ifndef REV9E
			case EVT_KEY_BREAK(KEY_UP) :
				view += 1 ;
    	  if( view>=MAX_VIEWS) view = 0 ;
    	  audioDefevent(AU_KEYPAD_UP) ;
    	  g_model.mview = view | tview ;
				eeModelChanged() ;
				io_subview = 0 ;
    	break;

			case EVT_KEY_BREAK(KEY_DOWN) :
	      if(view>0)
  	      view = view - 1 ;
    	  else
      	  view = MAX_VIEWS-1 ;
  	    audioDefevent(AU_KEYPAD_DOWN) ;
	      g_model.mview = view | tview ;
				eeModelChanged() ;
				io_subview = 0 ;
	    break ;

	    case EVT_KEY_LONG(KEY_DOWN):
 				view = e_telemetry ;
				g_model.mview = view | tview ;
				eeModelChanged() ;
      	audioDefevent(AU_MENUS) ;
    	  killEvents(event) ;
  	  break;
    
			case EVT_KEY_LONG(KEY_UP) :
  		  pushMenu(menuProcBattery) ;
    	  killEvents(event) ;
    	break ;
#endif	// nREV9E

	    case EVT_KEY_LONG(KEY_EXIT) :
        resetTimer() ;
        resetTelemetry() ;
        audioDefevent(AU_MENUS) ;
			break ;
    	
			case EVT_ENTRY:
        killEvents(KEY_EXIT) ;
        killEvents(KEY_UP) ;
        killEvents(KEY_DOWN) ;
        trimSwLock = true ;
				io_subview = 0 ;
			break ;

		}
	} // !PopupActive
#else

 if ( ! PopupData.PopupActive )
 {
	switch(event)
	{
    case  EVT_KEY_LONG(KEY_MENU):// go to last menu
#ifdef FRSKY
        if( (view == e_telemetry) && ((tview & 0x70) == 0x30 ) )
        {
          AltOffset = -FrskyHubData[FR_ALT_BARO] ;
        }
        else if( (view == e_telemetry) && ( ( (tview & 0x70) == 0 ) !! ((tview & 0x70) == 0x10 )) )
        {
          for (uint32_t i=0; i<6; i++)
					{
						uint32_t j ;
						j = tview == 0 ? g_model.customDisplayIndex[i]-1 : g_model.customDisplay2Index[i]-1 ;
						// Might check for a Scaler as well to see if ALT is its source
						if ( j == TEL_ITEM_BALT )
						{
            	AltOffset = -FrskyHubData[FR_ALT_BARO] ;
						}
						if ( ( j >= TEL_ITEM_SC1 ) && (j <= TEL_ITEM_SC8 ) )
						{
							j = g_model.Scalers[j-TEL_ITEM_SC1].source - 1 ;
							if ( j == TEL_ITEM_BALT )
							{
    	        	AltOffset = -FrskyHubData[FR_ALT_BARO] ;
							}
						}
						if ( j == TEL_ITEM_A1 )
						{
            	if ( g_model.frsky.channels[0].type == 3 )		// Current (A)
							{
				    	  frskyTelemetry[0].setoffset() ;
							}
						}
						if ( j == TEL_ITEM_A2 )
						{
            	if ( g_model.frsky.channels[1].type == 3 )		// Current (A)
							{
				    	  frskyTelemetry[1].setoffset() ;
							}
						}
					}
        }
        else if( (view == e_telemetry) && ((tview & 0x70) == 0x40 ) )	// GPS
				{
					struct t_hub_max_min *maxMinPtr = &FrskyHubMaxMin ;
			
					maxMinPtr->hubMax[FR_GPS_SPEED] = 0 ;
					maxMinPtr->hubMax[FR_GPS_ALT] = 0 ;
				}
        else
        {
#endif
#ifdef REV9E
  	  pushMenu(menuProcBattery);
#else
    case  EVT_KEY_LONG(BTN_RE):// go to last menu
		        scroll_disabled = 1;
            pushMenu(lastPopMenu());
//            killEvents(event);
#endif	// REV9E
#ifdef FRSKY
        }
#endif
        killEvents(event);
    break;

#ifdef REV9E
    case EVT_KEY_LONG(BTN_RE):
#endif	// REV9E
    case EVT_KEY_LONG(KEY_RIGHT):
        scroll_disabled = 1;
        pushMenu(menuProcModelSelect);
        killEvents(event);
        break;

#ifdef REV9E
    case EVT_KEY_BREAK(KEY_MENU):			// ***************
#endif	// REV9E
    case EVT_KEY_BREAK(KEY_RIGHT):
        if(view <= e_inputs1)
				{
					int8_t x ;
					x = io_subview ;
#ifdef REV9E
					if ( ++x > ((view == e_inputs1) ? 6 : 2) ) x = 0 ;
#else
					if ( ++x > ((view == e_inputs1) ? 4 : 2) ) x = 0 ;
#endif	// REV9E
					io_subview = x ;
				}	
#ifdef FRSKY
        if(view == e_telemetry)
				{
					tview += 0x10 ;
					if ( tview > ( ( FrskyTelemetryType != 2 ) ? 0x30 : 0x40 ) )
					{
						tview = 0 ;
					}
					
            g_model.mview = e_telemetry | tview ;
						eeModelChanged() ;
            //            STORE_GENERALVARS;     //eeWriteGeneral();
            //            eeDirty(EE_GENERAL);
            audioDefevent(AU_MENUS);
        }
#endif
        break;
#ifndef REV9E
    case EVT_KEY_BREAK(KEY_LEFT):
        if(view <= e_inputs1)
				{
					int8_t x ;
					x = io_subview ;
					if ( --x < 0 ) x = (view == e_inputs1) ? 4 : 2 ;
					io_subview = x ;
				}	
#ifdef FRSKY
        if(view == e_telemetry)
				{
					tview -= 0x10 ;
					if ( tview > 0x40 )
					{
						tview = ( FrskyTelemetryType != 2 ) ? 0x30 : 0x40 ;
					}
            g_model.mview = e_telemetry | tview ;
						eeModelChanged() ;
            //            STORE_GENERALVARS;     //eeWriteGeneral();
            //            eeDirty(EE_GENERAL);
            audioDefevent(AU_MENUS);
        }
#endif
        break;
#endif	// nREV9E
    case EVT_KEY_LONG(KEY_LEFT):
        scroll_disabled = 1;
        pushMenu(menuProcIndex);
        killEvents(event);
        break;
		case EVT_KEY_BREAK(KEY_UP) :
			view += 1 ;
      if( view>=MAX_VIEWS) view = 0 ;
      g_model.mview = view | tview ;
			eeModelChanged() ;
//      STORE_GENERALVARS;     //eeWriteGeneral() ;
      audioDefevent(AU_KEYPAD_UP) ;
			io_subview = 0 ;
    break;
#ifdef REV9E
		case EVT_KEY_BREAK(KEY_PAGE) :
#endif	// REV9E
		case EVT_KEY_BREAK(KEY_DOWN) :
      if(view>0)
        view = view - 1;
      else
        view = MAX_VIEWS-1;
      g_model.mview = view | tview ;
			eeModelChanged() ;
//      STORE_GENERALVARS;     //eeWriteGeneral() ;
      audioDefevent(AU_KEYPAD_DOWN) ;
			io_subview = 0 ;
    break;
    case EVT_KEY_LONG(KEY_UP):
  	  pushMenu(menuProcBattery);
      killEvents(event);
    break;
    case EVT_KEY_LONG(KEY_DOWN):
//#if defined(JETI)
//        JETI_EnableRXD(); // enable JETI-Telemetry reception
//        chainMenu(menuProcJeti);
//#elif defined(ARDUPILOT)
#if defined(ARDUPILOT)
        ARDUPILOT_EnableRXD(); // enable ArduPilot-Telemetry reception
        chainMenu(menuProcArduPilot);
#elif defined(NMEA)
        NMEA_EnableRXD(); // enable NMEA-Telemetry reception
        chainMenu(menuProcNMEA);
#else
			
 #ifdef REVX
			if ( g_model.telemetryProtocol == TELEMETRY_JETI )
			{
				chainMenu(menuProcJeti);
			}			        
			else
			{
 #endif
 				view = e_telemetry ;
				g_model.mview = view | tview ;
				eeModelChanged() ;
 #ifdef REVX
			}
 #endif
      audioDefevent(AU_MENUS);
//        chainMenu(menuProcStatistic2);
#endif
      killEvents(event);
    break;
    case EVT_KEY_FIRST(KEY_EXIT):
        if(s_timer[0].s_timerState==TMR_BEEPING) {
            s_timer[0].s_timerState = TMR_STOPPED;
            audioDefevent(AU_MENUS);
        }
//        else if(view == e_timer2) {
//            resetTimer2();
//            // Timer2_running = !Timer2_running;
//            audioDefevent(AU_MENUS);
//        }
#ifdef FRSKY
        else if (view == e_telemetry) {
            resetTelemetry();
            audioDefevent(AU_MENUS);
        }
#endif
        break;
    case EVT_KEY_LONG(KEY_EXIT):
        resetTimer();
//        resetTimer2();
#ifdef FRSKY
        resetTelemetry();
#endif
        audioDefevent(AU_MENUS);
        break;
    case EVT_ENTRY:
        killEvents(KEY_EXIT);
        killEvents(KEY_UP);
        killEvents(KEY_DOWN);
        trimSwLock = true;
				io_subview = 0 ;
        break;

//#ifndef REV9E
//    case EVT_KEY_BREAK(KEY_MENU):			// ***************
//#endif	// nREV9E
//		case EVT_KEY_BREAK(BTN_RE) :
//			PopupData.PopupIdx = 0 ;
//			PopupData.PopupActive = 1 ;
//      killEvents(event) ;
//			event = 0 ;
//    break ;
  }
 } // !PopupActive

#endif	// MAIN_POPUP_ENABLED



   if(getSwitch(g_model.trimSw,0) && !trimSwLock) setStickCenter();
   trimSwLock = getSwitch(g_model.trimSw,0);

  if (view != e_telemetry)
	{
	  register  uint8_t x=FW*2;
  	register uint8_t att = (g_vbat100mV < g_eeGeneral.vBatWarn ? BLINK : 0) | DBLSIZE;
  	register uint32_t i ;

		putsDblSizeName( 0 ) ;

//#ifdef PCBX9D
// #ifdef REVPLUS
//extern uint8_t DmaDebugDone ;
//extern uint16_t DmaDebugCount ;
//extern uint16_t DmaDebugNDTR ;

//	lcd_outhex4( 20, 0, DmaDebugDone ) ;
//	lcd_outhex4( 20, FH, DmaDebugCount ) ;
//	lcd_outhex4( 50, FH, DmaDebugNDTR ) ;
// #endif
//#endif

//extern uint16_t SpiDebug[4] ;

//	lcd_outhex4( 0, 0, SPI3->CR1 ) ;
//	lcd_outhex4( 0, FH, SPI3->SR ) ;
//	lcd_outhex4( 32, 0, SpiDebug[0] ) ;
//	lcd_outhex4( 32, FH, SpiDebug[1] ) ;
//#endif

//extern uint16_t RotaryAnalogValue ;
//extern uint16_t VpaeValue ;
//extern volatile int32_t Rotary_position ;
//extern volatile int32_t Rotary_count ;
//lcd_outhex4( 0, 0, RotaryAnalogValue >> 1 ) ;
//lcd_outhex4( 25, 0, Rotary_position ) ;
//lcd_outhex4( 50, 0, Rotary_count ) ;
//lcd_outhex4( 75, 0, VpaeValue ) ;

//extern uint8_t AnaEncSw ;
//extern uint8_t AnaRot0 ;
//extern uint16_t AnaRot1 ;
//lcd_outhex4( 0, 0, AnaRot0 ) ;
//lcd_outhex4( 25, 0, AnaRot1 ) ;
//lcd_outhex4( 50, 0, AnaEncSw ) ;

    putsVBat( 6*FW+1, 2*FH, att|NO_UNIT);
    lcd_putc( 6*FW+2, 3*FH, 'V');

//    if(g_model.timer[0].tmrModeA != TMRMODE_NONE)
//		{
			displayTimer( x+14*FW-3, FH*2, 0, DBLSIZE ) ;
      putsTmrMode(x+7*FW-FW/2,FH*3,0, 0, 0 ) ;
//  	}

		i = getFlightPhase() ;
		if ( i && g_model.phaseData[i-1].name[0] )
		{
			lcd_putsnAtt( 6*FW+2, 2*FH, g_model.phaseData[i-1].name, 6, /*BSS*/ 0 ) ;
			lcd_rect( 6*FW+1, 2*FH-1, 6*FW+2, 9 ) ;
		}
		else
		{
    	lcd_putsAttIdx( 6*FW+2, 2*FH,PSTR(STR_TRIM_OPTS),g_model.trimInc, 0);
			if ( g_model.thrTrim )
			{
				lcd_puts_P(x+8*FW-FW/2-1,2*FH,PSTR(STR_TTM));
			}
		}

  	//trim sliders
  	for( i=0 ; i<4 ; i++ )
  	{
	#define TL 27
  	  //                        LH LV RV RH
  	  static uint8_t x[4]    = {128*1/4+2, 4, 128-4, 128*3/4-2};
//  	  static uint8_t vert[4] = {0,1,1,0};
  	  register uint8_t xm, ym ;
#ifdef FIX_MODE
			xm = modeFixValue( i ) ;
      xm = x[xm-1] ;
#else
  	  xm=x[i] ;
#endif
  	  
			
			register int16_t valt = getTrimValue( CurrentPhase, i ) ;
			uint8_t centre = (valt == 0) ;
      int8_t val = max((int8_t)-(TL+1),min((int8_t)(TL+1),(int8_t)(valt/4)));
//  	  if(vert[i])
      if( (i == 1) || ( i == 2 ))
			{
  	    ym=31;
  	    lcd_vline(xm,   ym-TL, TL*2);

#ifdef FIX_MODE
          if((i == 1) || !(g_model.thrTrim))
#else
  	      if(((g_eeGeneral.stickMode&1) != (i&1)) || !(g_model.thrTrim))
#endif
					{
  	          lcd_vline(xm-1, ym-1,  3);
  	          lcd_vline(xm+1, ym-1,  3);
  	      }
  	      ym -= val;
  	  }else{
  	    ym=59;
  	    lcd_hline(xm-TL,ym,    TL*2);
  	    lcd_hline(xm-1, ym-1,  3);
  	    lcd_hline(xm-1, ym+1,  3);
  	    xm += val;
  	  }
  		DO_SQUARE(xm,ym,7)
			if ( centre )
			{
        DO_SQUARE(xm,ym,5) ;
			}
		}
 	}
  else
	{
		uint8_t i ;
		i = getFlightPhase() ;
		if ( i && g_model.phaseData[i-1].name[0] )
		{
			lcd_putsnAtt( 2*FW, 0, g_model.phaseData[i-1].name, 6, /*BSS*/ 0 ) ;
		}
		else
		{
    	lcd_putsnAtt(0, 0, g_model.name, sizeof(g_model.name), INVERS);
		}
    
		uint8_t att = (g_vbat100mV < g_eeGeneral.vBatWarn ? BLINK : 0);
    putsVBat(14*FW,0,att);
//    if(g_model.timer[0].tmrModeA != TMRMODE_NONE)
//		{
			displayTimer( 18*FW+3, 0, 0, 0 ) ;
//    }
  }
  
	if(view<e_inputs1)
	{
		if ( io_subview > 2 )
		{
			io_subview = 0 ;
		}
    lcd_hlineStip(38, 33, 54, 0x55 ) ;
    lcd_hlineStip(38, 34, 54, 0x55 ) ;

		if ( ( io_subview == 0 ) || ( BLINK_ON_PHASE ) )
		{
    	lcd_hlineStip(38 + io_subview * 18, 33, 18, 0xAA ) ;
    	lcd_hlineStip(38 + io_subview * 18, 34, 18, 0xAA ) ;
		}
		
    register uint32_t i ;
    for( i=0; i<8; i++)
    {
      uint8_t x0,y0;
			uint8_t chan = 8 * io_subview + i ;
      int16_t val = g_chans512[chan];
          //val += g_model.limitData[i].revert ? g_model.limitData[i].offset : -g_model.limitData[i].offset;
      switch(view)
      {
        case e_outputValues:
          x0 = (i%4*9+3)*FW/2;
          y0 = i/4*FH+40;
              // *1000/1024 = x - x/8 + x/32
#define GPERC(x)  (x - x/32 + x/128)
          lcd_outdezAtt( x0+4*FW , y0, GPERC(val),PREC1 ) ;
        break;
            
				case e_outputBars:
#define WBAR2 (50/2)
          x0       = i<4 ? 128/4+2 : 128*3/4-2;
          y0       = 38+(i%4)*5;
    			int16_t limit = (g_model.extendedLimits ? 1280 : 1024);
          int8_t l = (abs(val) * WBAR2 + 512) / limit ;
          if(l>WBAR2)  l =  WBAR2;  // prevent bars from going over the end - comment for debugging

          lcd_hlineStip(x0-WBAR2,y0,WBAR2*2+1,0x55);
          lcd_vline(x0,y0-2,5);
          if(val>0)
					{
            x0+=1;
          }else{
            x0-=l;
          }
          lcd_hline(x0,y0+1,l);
          lcd_hline(x0,y0-1,l);
                
				break;
      }
    }
  }
#ifdef FRSKY
    else if(view == e_telemetry) {
//        static uint8_t staticTelemetry[4];
//        static uint8_t staticRSSI[2];
        static enum AlarmLevel alarmRaised[2];
//        int8_t unit ;
        int16_t value ;
//        if ( (frskyStreaming) || ( tview  == 0x30 ) )
				{
            uint8_t y0, x0, blink;
//            if (++displayCount > 49) {
//                displayCount = 0;
                for (int i=0; i<2; i++) {
//                    staticTelemetry[i] = frskyTelemetry[i].value;
//                    staticRSSI[i] = frskyRSSI[i].value;
//										if ( i < 2 )
//										{
                      alarmRaised[i] = FRSKY_alarmRaised(i);
//										}
                }
//            }
            if ( tview == 0x20 )
            {
//                if (g_model.frsky.channels[0].ratio || g_model.frsky.channels[1].ratio) {
                    x0 = 0;
                    for (int i=0; i<2; i++) {
                        if (g_model.frsky.channels[i].ratio) {
                            blink = (alarmRaised[i] ? INVERS : 0);
                            lcd_puts_P(x0, 3*FH, PSTR(STR_A_EQ) ) ;
                            lcd_putc(x0+FW, 3*FH, '1'+i);
                            x0 += 3*FW;
                            putsTelemValue( x0-2, 2*FH, frskyTelemetry[i].value, i,  blink|DBLSIZE|LEFT ) ;
                            if ( g_model.frsky.channels[i].type == 3 )		// Current (A)
														{
                              lcd_outdezAtt(x0+FW, 4*FH,  FrskyHubData[FR_A1_MAH+i], 0);
														}
														else
														{
                              putsTelemValue(x0+FW, 4*FH, FrskyHubMaxMin.hubMin[i], i, 0 ) ;
														}
                            putsTelemValue(x0+3*FW, 4*FH, FrskyHubMaxMin.hubMax[i], i, LEFT ) ;
                        }
                        x0 = 11*FW-2;
                    }
//                }
								// Fuel Gauge
                if (frskyUsrStreaming)
								{
                	lcd_puts_Pleft( 1*FH, PSTR(STR_FUEL)) ;
									x0 = FrskyHubData[FR_FUEL] ;		// Fuel gauge value
									lcd_hbar( 25, 9, 102, 6, x0 ) ;
								}
                lcd_puts_Pleft( 6*FH, PSTR(STR_RXEQ));
    						lcd_putsAttIdx( 11 * FW - 2, 6*FH, PSTR(STR_TXEQ), ( g_model.protocol == PROTO_PXX ), 0 ) ;
//                lcd_puts_P(11 * FW - 2, 6*FH, Str_TXeq );
//                if (frskyStreaming)
//								{
               	lcd_outdezAtt(3 * FW - 2, 5*FH, frskyTelemetry[2].value, DBLSIZE|LEFT);
//								}
                lcd_outdezAtt(14 * FW - 4, 5*FH, FrskyHubData[FR_TXRSI_COPY], DBLSIZE|LEFT);
                
								struct t_hub_max_min *maxMinPtr = &FrskyHubMaxMin ;
								
								lcd_outdezAtt(4 * FW, 7*FH, maxMinPtr->hubMin[2], 0);
                lcd_outdezAtt(6 * FW, 7*FH, maxMinPtr->hubMax[2], LEFT);
                lcd_outdezAtt(15 * FW - 2, 7*FH, maxMinPtr->hubMin[3], 0);
                lcd_outdezAtt(17 * FW - 2, 7*FH, maxMinPtr->hubMax[3], LEFT);
            }
            else if ( tview == 0x30 )
            {
                if (frskyUsrStreaming)
                {
									displayTemp( 1, 0, 2*FH, DBLSIZE ) ;
									displayTemp( 2, 14*FW, 7*FH, 0 ) ;

                    lcd_puts_Pleft( 2*FH, PSTR(STR_12_RPM));
                    lcd_outdezNAtt(13*FW, 1*FH, (uint16_t)FrskyHubData[FR_RPM], DBLSIZE|LEFT, 5);

//                    lcd_puts_Pleft( 4*FH, PSTR(STR_ALTEQ));
                    value = FrskyHubData[FR_ALT_BARO] + AltOffset ;
//										if (g_model.FrSkyUsrProto == 1)  // WS How High
//										{
//                        unit = 'f' ;  // and ignore met/imp option
//										}
//										else
//										{
//                      unit = 'm' ;
//                      if ( g_model.FrSkyImperial )
//                      {
//                        // m to ft *105/32
//                       	value = value * 3 + ( value >> 2 ) + (value >> 5) ;
//                        unit = 'f' ;
//                      }
//										}
//                    lcd_putc( 3*FW, 3*FH, unit ) ;
//                    lcd_outdezAtt(4*FW, 3*FH, value, DBLSIZE|LEFT);
									putsTelemetryChannel( 0, 4*FH, TEL_ITEM_BALT, value, DBLSIZE | LEFT, (TELEM_LABEL | TELEM_UNIT_LEFT)) ;
                
								
								}	
                if (g_model.frsky.channels[0].ratio)
                {
                    blink = (alarmRaised[0] ? INVERS+BLINK : 0);
                    lcd_puts_Pleft( 6*FH, XPSTR("A1="));
                    putsTelemValue( 3*FW-2, 5*FH, frskyTelemetry[0].value, 0,  blink|DBLSIZE|LEFT ) ;
                }
                if (g_model.frsky.channels[1].ratio)
                {
                    blink = (alarmRaised[1] ? INVERS+BLINK : 0);
                    lcd_puts_P(11*FW-2, 6*FH, XPSTR("A2="));
                    putsTelemValue( 14*FW-2, 5*FH, frskyTelemetry[1].value, 1,  blink|DBLSIZE|LEFT ) ;
                }
                lcd_puts_Pleft( 7*FH, PSTR(STR_RXEQ) );
                lcd_outdezAtt(3 * FW, 7*FH, FrskyHubData[FR_RXRSI_COPY], LEFT);
    						lcd_putsAttIdx( 8 * FW, 7*FH, PSTR(STR_TXEQ), ( g_model.protocol == PROTO_PXX ), 0 ) ;
//                lcd_puts_P(8 * FW, 7*FH, PSTR(STR_TXEQ) );
                lcd_outdezAtt(11 * FW, 7*FH, FrskyHubData[FR_TXRSI_COPY], LEFT);
            }
            else if ( tview == 0x40 )
            {
							uint8_t blink = BLINK ;
							uint16_t mspeed ;
              if (frskyUsrStreaming)
							{
								blink = 0 ;
							}

                lcd_puts_Pleft( 2*FH, PSTR(STR_LAT_EQ)) ;
                lcd_outdezNAtt(8*FW, 2*FH, FrskyHubData[FR_GPS_LAT], LEADING0 | blink, -5);
                lcd_putc(8*FW, 2*FH, '.') ;
                lcd_outdezNAtt(12*FW, 2*FH, FrskyHubData[FR_GPS_LATd], LEADING0 | blink, -4);
								lcd_putcAtt( 14*FW, 2*FH, FrskyHubData[FR_LAT_N_S], blink ) ;
                lcd_puts_Pleft( 3*FH, PSTR(STR_LON_EQ)) ;
                lcd_outdezNAtt(8*FW, 3*FH, FrskyHubData[FR_GPS_LONG], LEADING0 | blink, -5);
                lcd_putc(8*FW, 3*FH, '.') ;
                lcd_outdezNAtt(12*FW, 3*FH, FrskyHubData[FR_GPS_LONGd], LEADING0 | blink, -4);
								lcd_putcAtt( 14*FW, 3*FH, FrskyHubData[FR_LONG_E_W], blink ) ;
                lcd_puts_Pleft( 4*FH, PSTR(STR_ALT_MAX)) ;
                lcd_outdezAtt(20*FW, 4*FH, FrskyHubMaxMin.hubMax[FR_GPS_ALT], 0);
                lcd_outdez(20*FW, 3*FH, FrskyHubData[FR_COURSE] );
                
								lcd_puts_Pleft( 5*FH, PSTR(STR_SPD_KTS_MAX)) ;
//                lcd_outdezAtt(20*FW, 5*FH, MaxGpsSpeed, blink );

								mspeed = FrskyHubMaxMin.hubMax[FR_GPS_SPEED] ;
                if ( g_model.FrSkyImperial )
								{
									lcd_puts_Pleft( 5*FH, PSTR(STR_11_MPH)) ;
									mspeed = ( mspeed * 589 ) >> 9 ;
								}
                lcd_outdezAtt(20*FW, 5*FH, mspeed, blink );
              if (frskyUsrStreaming)
							{
								mspeed = FrskyHubData[FR_GPS_SPEED] ;
                if ( g_model.FrSkyImperial )
								{
									mspeed = ( mspeed * 589 ) >> 9 ;
								}
								lcd_outdezAtt(8 * FW, 4*FH, FrskyHubData[FR_GPS_ALT], 0 ) ;
                lcd_outdezAtt(8*FW, 5*FH, mspeed, 0);		// Speed
                
								lcd_puts_Pleft( 6*FH, XPSTR("V1=\007V2=\016V3=")) ;
								lcd_puts_Pleft( 7*FH, XPSTR("V4=\007V5=\016V6=")) ;
								{
									uint8_t x, y ;
									x = 6*FW ;
									y = 6*FH ;
	      					for (uint8_t k=0; k<FrskyBattCells; k++)
									{
										uint8_t blink=0;
										if ( k == 3 )
										{
											x = 6*FW ;
											y = 7*FH ;
										}
										if ((FrskyVolts[k] < g_model.frSkyVoltThreshold))
										{
										  blink = BLINK ;
										}
  									lcd_outdezNAtt( x, y, FrskyVolts[k] * 2 , blink | PREC2, 4 ) ;
										x += 7*FW ;
										if ( k == 5 )		// Max 6 cels displayable
										{
											break ;											
										}
	      					}
								}
								//              lcd_putsAtt(6, 2*FH, PSTR("To Be Done"), DBLSIZE);
              }
						}
            else if ( ( tview == 0 ) || ( tview == 0x10 ) )	// CUSTOM TELEMETRY DISPLAY
            {
							lcd_vline( 63, 8, 48 ) ;
							
              for (uint8_t i=0; i<6; i++)
							{
								uint8_t *pindex = ( tview == 0x10 ) ? g_model.customDisplay2Index : g_model.customDisplayIndex ;
								if ( pindex[i] )
								{
									uint32_t index = pindex[i]-1 ;
									uint32_t x = (i&1)?65:0 ;
									uint32_t y = (i&0x0E)*FH+2*FH ;
									
//									lcd_puts_P( x, y, XPSTR(":  :"));
//									lcd_outdezNAtt( 18*FW-2, y, Time.second, LEADING0, 2 ) ;
//									lcd_outdezNAtt( 15*FW-1, y, Time.minute, LEADING0, 2 ) ;

									putsTelemetryChannel( x, y, index, get_telemetry_value(index),
																							 DBLSIZE, TELEM_LABEL|TELEM_UNIT|TELEM_UNIT_LEFT|TELEM_VALUE_RIGHT ) ;
								}
							}

//                y0 = 6*FH;
//                //lcd_puts_P(2*FW-3, y0, PSTR("Tele:"));
//                x0 = 4*FW-3;
//                for (int i=0; i<2; i++) {
//                    if (g_model.frsky.channels[i].ratio) {
//                        blink = (alarmRaised[i] ? INVERS+BLINK : 0)|LEFT;
//												putsTelemetryChannel( x0, y0, TEL_ITEM_A1+i, frskyTelemetry[i].value, blink, TELEM_LABEL|TELEM_UNIT ) ;
////                        lcd_puts_P(x0, y0, PSTR(STR_A_EQ) ) ;
////                        lcd_putc(x0+FW, y0, '1'+i);
////                        putsTelemValue( x0+3*FW, y0, frskyTelemetry[i].value, i,  blink ) ;
//                        x0 = 13*FW-3;
//                    }
//                }
                y0 = 7*FH;
                //lcd_puts_P(2*FW-3, y0, PSTR("RSSI:"));
//                lcd_puts_P(4*FW-3, y0, PSTR(STR_RXEQ) );
//                lcd_outdezAtt(7*FW-3, y0, FrskyHubData[FR_RXRSI_COPY], LEFT);
//                lcd_puts_P(13*FW-3, y0, PSTR(STR_TXEQ) );
//                lcd_outdezAtt(16*FW-3, y0, FrskyHubData[FR_TXRSI_COPY], LEFT);
                
								lcd_puts_Pleft( y0, PSTR(STR_RXEQ) ) ;
//								lcd_putsn_P( 0, y0, PSTR(STR_RXEQ), 2 );
								lcd_hbar( 20, 57, 43, 6, FrskyHubData[FR_RXRSI_COPY] ) ;
    						lcd_putsAttIdx( 110, y0, PSTR(STR_TXEQ), ( g_model.protocol == PROTO_PXX ), 0 ) ;
//                lcd_putsn_P( 116, y0, PSTR(STR_TXEQ), 2 );
								lcd_hbar( 65, 57, 43, 6, FrskyHubData[FR_TXRSI_COPY] ) ;

            }
            else// if ( tview == 0x50 )
						{
							// Spektrum format screen
							lcd_puts_Pleft( 0*FH, XPSTR("\013rssi")) ;
							lcd_puts_Pleft( 2*FH, XPSTR("Vbat")) ;
							lcd_puts_Pleft( 2*FH, XPSTR("\013RxV")) ;
							lcd_puts_Pleft( 4*FH, XPSTR("AMP\013Temp")) ;
							lcd_puts_Pleft( 6*FH, XPSTR("RPM\021DSM2")) ;
							
							lcd_vline( 63, 8, 32 ) ;

              lcd_outdezAtt( 17*FW-2, 0*FH, FrskyHubData[FR_RXRSI_COPY], 0 ) ;
              lcd_outdezAtt( 61, 1*FH, FrskyHubData[FR_VOLTS], PREC1|DBLSIZE ) ;
							lcd_outdezAtt( 125, 1*FH, FrskyHubData[FR_RXV] * g_model.rxVratio / 255, PREC1|DBLSIZE ) ;
							lcd_outdezAtt( 61, 3*FH, FrskyHubData[FR_CURRENT], PREC1|DBLSIZE ) ;
              lcd_outdezAtt( 125, 3*FH, FrskyHubData[FR_TEMP1], DBLSIZE ) ;
              lcd_outdezAtt( 14*FW, 5*FH, FrskyHubData[FR_RPM], DBLSIZE ) ;

							if ( g_model.dsmMode & ORTX_USE_DSMX )
							{
								lcd_putc( 20*FW, 6*FH, 'X' ) ;
							}
							dsmDisplayABLRFH() ;
						}
//            else	// ( tview == 0x50 )
//						{
//							// second custom screen
							
//                lcd_puts_P(4*FW-3, y0, PSTR(STR_RXEQ) );
//                lcd_outdezAtt(7*FW-3, y0, FrskyHubData[FR_RXRSI_COPY], LEFT);
//                lcd_puts_P(13*FW-3, y0, PSTR(STR_TXEQ) );
//                lcd_outdezAtt(16*FW-3, y0, FrskyHubData[FR_TXRSI_COPY], LEFT);
//						}

        }
//        else {
//            lcd_putsAtt(22, 5*FH, PSTR("NO DATA"), DBLSIZE);
//        }
    }
#endif
  else if(view<e_timer2)
	{
		doMainScreenGrphics() ;

    int8_t a = io_subview ;
#ifdef PCBSKY
		if ( a != 0 ) a = a * 6 + 3 ;		// 0, 9, 15
#endif
#ifdef PCBX9D
		if ( a != 0 ) a = a * 6 ;		// 0, 6, 12
#endif
    uint8_t j ;
		for ( j = 0 ; j < 2 ; j += 1 )
		{
#ifdef PCBSKY
			if ( j == 1 )
			{
				a = io_subview ;
				a += 1 ;
				a *= 6 ;		// 6, 12, 18
			}
#endif
#ifdef PCBX9D
			if ( j == 1 )
			{
				a += 3 ;
			}
#endif
			switchDisplay( j, a ) ;
//			for(int8_t i=a; i<(a+3); i++)
//			{
//				uint8_t s = i ;
//				if ( i == 8 )
//				{
//					s = 3 ;
//					if ( ! getSwitch( s+1, 0 ) )		// ID0 ?
//					{
//						s = 4 ;
//						if ( ! getSwitch( s+1, 0 ) )		// ID1 ?
//						{
//							s = 5 ;
//						}
//					}
//				}
//				lcd_putsnAtt((2+j*15)*FW-2 ,(i-a+4)*FH,get_switches_string()+3*s,3,getSwitch(i+1, 0) ? INVERS : 0);
//			}
		}
	}
  else  // New Timer2 display
  {
		displayTimer( 30+5*FW, FH*5, 1, DBLSIZE ) ;
  }

	if ( PopupData.PopupActive )
	{
//		uint8_t count ;
		uint16_t mask = 0x1F ;

    if(PopupData.PopupActive == 1)
		{
			mask = 0x01E0 ;
		}
		
		uint8_t popaction = doPopup( XPSTR("Model Select\0Model Setup\0Last Menu\0Radio Setup\0Statistics\0"\
																"Zero Alt.\0Zero A1 Offs\0Zero A2 Offs\0Reset GPS"), mask, 14, event ) ;
																 
		UseLastSubmenuIndex = 0 ;
  	if ( popaction == POPUP_SELECT )
		{
			uint8_t popidx = PopupData.PopupSel ;
			if ( popidx == 0 )	// Model Select
			{
        pushMenu(menuProcModelSelect) ;
			}
			else if( popidx == 1 )	// Edit Model
			{
				RotaryState = ROTARY_MENU_UD ;
	  	  pushMenu(menuProcModelIndex) ;
			}
			else if( popidx == 2 )	// Last Menu
			{
				UseLastSubmenuIndex = 1 ;
        pushMenu(lastPopMenu());
			}
			else if ( popidx == 3 )	// Radio Setup
			{
        pushMenu(menuProcIndex) ;
			}
			else if( popidx == 4 )	// Statistics
			{
	  	  pushMenu(menuProcBattery) ;
			}
			else if( popidx == 5 )	// Zero Alt.
			{
        AltOffset = -FrskyHubData[FR_ALT_BARO] ;
			}
			else if( popidx == 6 )	// A1 Offset
			{
        if ( g_model.frsky.channels[0].type == 3 )		// Current (A)
				{
				  frskyTelemetry[0].setoffset() ;
				}
			}
			else if( popidx == 7 )	// A2 Offset
			{
        if ( g_model.frsky.channels[1].type == 3 )		// Current (A)
				{
				  frskyTelemetry[1].setoffset() ;
				}
			}
			else if( popidx == 8 )	// GPS reset
			{
				struct t_hub_max_min *maxMinPtr = &FrskyHubMaxMin ;

				maxMinPtr->hubMax[FR_GPS_SPEED] = 0 ;
				maxMinPtr->hubMax[FR_GPS_ALT] = 0 ;
			}
		}
//		else if ( popaction == POPUP_EXIT )
//		{
//			killEvents( event ) ;
//			PopupData.PopupActive = 0 ;
//		}
	}
//extern uint8_t SbusFrame[] ;
//extern uint16_t SbusTimer ;
//extern uint8_t SbusIndex ;
//extern uint8_t SbusCount ;

//	lcd_outhex4( 0, FH, (SbusFrame[0] << 8) | SbusFrame[1] ) ;
//	lcd_outhex4( 30, FH, (SbusFrame[2] << 8) | SbusFrame[3] ) ;
//	lcd_outhex4( 60, FH, (SbusFrame[4] << 8) | SbusFrame[5] ) ;
//	lcd_outhex4( 90, FH, (SbusFrame[6] << 8) | SbusFrame[7] ) ;

//  register Uart *pUart = UART0 ;
//	lcd_outhex4( 0, 2*FH, pUart->UART_BRGR ) ;
	
//	lcd_outhex4( 0, 0, Sound_g.VoiceActive ) ;
//extern uint8_t VoiceCount ;
//	lcd_outhex4( 30, 0, VoiceCount ) ;
//extern struct t_voice Voice ;
//	lcd_outhex4( 0, FH, Voice.VoiceQueueCount ) ;
//	lcd_outhex4( 30, FH, AudioVoiceUnderrun ) ;
//	lcd_outhex4( 60, FH, DACC->DACC_IMR ) ;
//	lcd_outhex4( 90, FH, DACC->DACC_PTSR ) ;
//extern uint32_t DebugVoice ;
//	lcd_outhex4( 0, 2*FH, DebugVoice >> 16 ) ;
//	lcd_outhex4( 30, 2*FH, DebugVoice ) ;
//extern uint32_t DebugVoice1 ;
//	lcd_outhex4( 60, 2*FH, DebugVoice1 ) ;
//	lcd_outhex4( 0, 3*FH, Sound_g.VoiceRequest ) ;
//	lcd_outhex4( 30, 3*FH, DACC->DACC_ISR ) ;
//	lcd_outhex4( 0, 4*FH, VoiceBuffer[0].flags ) ;
//	lcd_outhex4( 30, 4*FH, VoiceBuffer[1].flags ) ;
//	lcd_outhex4( 60, 4*FH, VoiceBuffer[2].flags ) ;

//extern uint8_t CurrentVolume ;

//extern void read_volume( void ) ;
//extern uint8_t Volume_read ;
//	static uint8_t volTimer = 0 ;
//	if ( ++volTimer > 49 )
//	{
//		volTimer = 0 ;
//		read_volume() ;
//	}
//	lcd_outhex4( 0, 0, CurrentVolume ) ;
//	lcd_outhex4( 25, 0, Volume_read ) ;

//#ifdef REV9E
//extern int16_t  anas[] ;
//#define X9D_OFFSET		42

//	lcd_outhex4( 212-X9D_OFFSET, 40, anas[4] ) ;
//	lcd_outhex4( 212-X9D_OFFSET, 48, calibratedStick[4] ) ;
//	lcd_outhex4( 212-X9D_OFFSET, 56, calibratedStick[8] ) ;


//	lcd_outhex4( 212-X9D_OFFSET+24, 0, calibratedStick[4] ) ;
//	lcd_outhex4( 212-X9D_OFFSET+24, 8, calibratedStick[5] ) ;
//	lcd_outhex4( 212-X9D_OFFSET+24, 16, calibratedStick[6] ) ;
//	lcd_outhex4( 212-X9D_OFFSET+24, 24, calibratedStick[7] ) ;
//	lcd_outhex4( 212-X9D_OFFSET+24, 32, calibratedStick[8] ) ;
//	lcd_outhex4( 212-X9D_OFFSET+24, 40, calibratedStick[9] ) ;
//	lcd_outhex4( 212-X9D_OFFSET+24, 48, calibratedStick[10] ) ;
//	lcd_outhex4( 212-X9D_OFFSET+24, 56, calibratedStick[11] ) ;
//#endif	// REV9E
}

uint16_t isqrt32(uint32_t n)
{
  uint16_t c = 0x8000;
  uint16_t g = 0x8000;

  for(;;)
	{
    if((uint32_t)g*g > n)
      g ^= c;
    c >>= 1;
    if(c == 0)
      return g;
    g |= c;
  }
}

int16_t intpol(int16_t x, uint8_t idx) // -100, -75, -50, -25, 0 ,25 ,50, 75, 100
{
#define D9 (RESX * 2 / 8)
#define D5 (RESX * 2 / 4)
    bool    cv9 = idx >= MAX_CURVE5;
    int8_t *crv = cv9 ? g_model.curves9[idx-MAX_CURVE5] : g_model.curves5[idx];
    int16_t erg;

    x+=RESXu;
    if(x < 0) {
        erg = (int16_t)crv[0] * (RESX/4);
    } else if(x >= (RESX*2)) {
        erg = (int16_t)crv[(cv9 ? 8 : 4)] * (RESX/4);
    } else {
				div_t qr ;
//        int16_t a,dx;
        if(cv9){
						qr = div( x, D9 ) ;
						qr.rem *= 2 ;
//            a   = (uint16_t)x / D9;
//            dx  =((uint16_t)x % D9) * 2;
        } else {
						qr = div( x, D5 ) ;
//            a   = (uint16_t)x / D5;
//            dx  = (uint16_t)x % D5;
        }
        erg  = (int16_t)crv[qr.quot]*((D5-qr.rem)/2) + (int16_t)crv[qr.quot+1]*(qr.rem/2);
    }
    return erg / 25; // 100*D5/RESX;
}

int16_t calcExpo( uint8_t channel, int16_t value )
{
  uint8_t expoDrOn = get_dr_state(channel);
  uint8_t stkDir = value > 0 ? DR_RIGHT : DR_LEFT ;

  if(IS_THROTTLE(channel) && g_model.thrExpo)
	{
    value  = 2*expo((value+RESX)/2,REG(g_model.expoData[channel].expo[expoDrOn][DR_EXPO][DR_RIGHT], -100, 100)) ;
    stkDir = DR_RIGHT ;
  }
  else
    value  = expo(value,REG(g_model.expoData[channel].expo[expoDrOn][DR_EXPO][stkDir], -100, 100)) ;

  value = (int32_t)value * (REG(g_model.expoData[channel].expo[expoDrOn][DR_WEIGHT][stkDir]+100, 0, 100))/100 ;
  if (IS_THROTTLE(channel) && g_model.thrExpo) value -= RESX;
	return value ;
}

// static variables used in perOut - moved here so they don't interfere with the stack
// It's also easier to initialize them here.
#if GVARS
int16_t  anas [NUM_SKYXCHNRAW+1+MAX_GVARS+1] ;		// To allow for 3POS and THIS and 8 extra PPM inputs
#else
int16_t  anas [NUM_SKYXCHNRAW+1+1] ;		// To allow for 3POS
#endif
int32_t  chans[NUM_SKYCHNOUT] = {0};
uint8_t inacPrescale ;
uint16_t inacCounter = 0;
uint16_t inacSum = 0;
uint8_t  bpanaCenter = 0;
uint16_t  sDelay[MAX_SKYMIXERS] = {0};
int32_t  act   [MAX_SKYMIXERS] = {0};
uint8_t  swOn  [MAX_SKYMIXERS] = {0};
uint8_t	CurrentPhase = 0 ;
int16_t rawSticks[4] ;

struct t_fade
{
uint8_t  fadePhases ;
uint16_t fadeRate ;
uint16_t fadeWeight ;
uint16_t fadeScale[MAX_MODES+1] ;
int32_t  fade[NUM_SKYCHNOUT];
} Fade ;

static void inactivityCheck()
{
  if(s_noHi) s_noHi--;
  uint16_t tsum = 0;
  for(uint8_t i=0;i<4;i++) tsum += anas[i];
  if(abs(int16_t(tsum-inacSum))>INACTIVITY_THRESHOLD)
	{
		inacSum = tsum;
		stickMoved = 1;  // reset in perMain
		inacCounter=0;
  }
  if( (g_eeGeneral.inactivityTimer + 10) && (g_vbat100mV>49))
	{
    if (++inacPrescale > 15 )
    {
      inacCounter++;
      inacPrescale = 0 ;
  	  if(inacCounter>((uint16_t)(g_eeGeneral.inactivityTimer+10)*(100*60/16)))
      if((inacCounter&0x1F)==1)
			{
				putVoiceQueue( 0xFF00 + NUM_VOL_LEVELS-3 ) ;
        audioVoiceDefevent( AU_INACTIVITY, V_INACTIVE ) ;
				putVoiceQueue( 0xFFFF ) ;

      }
    }
  }
}

#ifdef PCBX9D
#ifdef REV9E
const uint8_t switchIndex[18] = { HSW_SA0, HSW_SB0, HSW_SC0, HSW_SD0, HSW_SE0, HSW_SF2, HSW_SG0, HSW_SH2, HSW_SI0, 
																 HSW_SJ0, HSW_SK0, HSW_SL0, HSW_SM0, HSW_SN0, HSW_SO0, HSW_SP0, HSW_SQ0, HSW_SR0, } ;
#else
const uint8_t switchIndex[8] = { HSW_SA0, HSW_SB0, HSW_SC0, HSW_SD0, HSW_SE0, HSW_SF2, HSW_SG0, HSW_SH2 } ;
#endif	// REV9E
#endif

void perOutPhase( int16_t *chanOut, uint8_t att ) 
{
	static uint8_t lastPhase ;
	uint8_t thisPhase ;
	struct t_fade *pFade ;
	pFade = &Fade ;
	
	thisPhase = getFlightPhase() ;
	if ( thisPhase != lastPhase )
	{
		uint8_t time1 = 0 ;
		uint8_t time2 ;
		
		if ( lastPhase )
		{
      time1 = g_model.phaseData[(uint8_t)(lastPhase-1)].fadeOut ;
		}
		if ( thisPhase )
		{
      time2= g_model.phaseData[(uint8_t)(thisPhase-1)].fadeIn ;
			if ( time2 > time1 )
			{
        time1 = time2 ;
			}
		}
		if ( time1 )
		{
			pFade->fadeRate = (25600 / 50) / time1 ;
			pFade->fadePhases |= ( 1 << lastPhase ) | ( 1 << thisPhase ) ;
		}
		lastPhase = thisPhase ;
	}
	att |= FADE_FIRST ;
	if ( pFade->fadePhases )
	{
		pFade->fadeWeight = 0 ;
		uint8_t fadeMask = 1 ;
    for (uint8_t p=0; p<MAX_MODES+1; p++)
		{
			if ( pFade->fadePhases & fadeMask )
			{
				if ( p != thisPhase )
				{
					CurrentPhase = p ;
					pFade->fadeWeight += pFade->fadeScale[p] ;
					perOut( chanOut, att ) ;
					att &= ~FADE_FIRST ;				
				}
			}
			fadeMask <<= 1 ;
		}	
	}
	else
	{
		pFade->fadeScale[thisPhase] = 25600 ;
	}
	pFade->fadeWeight += pFade->fadeScale[thisPhase] ;
	CurrentPhase = thisPhase ;
	perOut( chanOut, att | FADE_LAST ) ;
	
	if ( pFade->fadePhases && tick10ms )
	{
		uint8_t fadeMask = 1 ;
    for (uint8_t p=0; p<MAX_MODES+1; p+=1)
		{
			uint16_t l_fadeScale = pFade->fadeScale[p] ;
			
			if ( pFade->fadePhases & fadeMask )
			{
				uint16_t x = pFade->fadeRate * tick10ms ;
				if ( p != thisPhase )
				{
          if ( l_fadeScale > x )
					{
						l_fadeScale -= x ;
					}
					else
					{
						l_fadeScale = 0 ;
						pFade->fadePhases &= ~fadeMask ;						
					}
				}
				else
				{
          if ( 25600 - l_fadeScale > x )
					{
						l_fadeScale += x ;
					}
					else
					{
						l_fadeScale = 25600 ;
						pFade->fadePhases &= ~fadeMask ;						
					}
				}
			}
			else
			{
				l_fadeScale = 0 ;
			}
			pFade->fadeScale[p] = l_fadeScale ;
			fadeMask <<= 1 ;
		}
	}
}

void perOut(int16_t *chanOut, uint8_t att )
{
    int16_t  trimA[4];
    uint8_t  anaCenter = 0;
    uint16_t d = 0;
		int16_t trainerThrottleValue = 0 ;
		uint8_t trainerThrottleValid = 0 ;
    {
        uint8_t ele_stick, ail_stick ;
        ele_stick = 1 ; //ELE_STICK ;
        ail_stick = 3 ; //AIL_STICK ;
        //===========Swash Ring================
        if(g_model.swashRingValue)
        {
            uint32_t v = (int32_t(calibratedStick[ele_stick])*calibratedStick[ele_stick] +
                          int32_t(calibratedStick[ail_stick])*calibratedStick[ail_stick]);
						uint32_t q = calc100toRESX(g_model.swashRingValue) ;
            q *= q;
            if(v>q)
                d = isqrt32(v);
        }
        //===========Swash Ring================

#ifdef FIX_MODE
				uint8_t stickIndex = g_eeGeneral.stickMode*4 ;
#endif        
#ifdef PCBSKY
        for(uint8_t i=0;i<7;i++)        // calc Sticks
#endif        
#ifdef PCBX9D
#ifdef REVPLUS
#ifdef REV9E
				for(uint8_t i=0;i<8+NUM_EXTRA_POTS;i++)        // calc Sticks
#else
        for(uint8_t i=0;i<9;i++)        // calc Sticks
#endif	// REV9E
#else		// REVPLUS
        for(uint8_t i=0;i<8;i++)        // calc Sticks
#endif	// REVPLUS
#endif	// PCBX9D
				{
            //Normalization  [0..2048] ->   [-1024..1024]

					int16_t v = anaIn(i);

#ifndef SIMU
 #ifdef PCBX9D
						if ( i == 7 )
						{
            v -= g_eeGeneral.x9dcalibMid ;
            v  =  v * (int32_t)RESX /  (max((int16_t)100,(v>0 ?
                                                              g_eeGeneral.x9dcalibSpanPos :
                                                              g_eeGeneral.x9dcalibSpanNeg)));
						}
#ifdef REVPLUS
#ifdef REV9E
						else if ( i == 11 )
						{
							v = anaIn(9) ;
            	v -= g_eeGeneral.x9dPcalibMid ;
	            v  =  v * (int32_t)RESX /  (max((int16_t)100,(v>0 ?
                                                              g_eeGeneral.x9dPcalibSpanPos :
                                                              g_eeGeneral.x9dPcalibSpanNeg)));
						}
						else if ( i >= 8 )
						{
							v = anaIn(i+2);
							v -= 1024 ;
						}
#else						
						else if ( i == 8 )
						{
            v -= g_eeGeneral.x9dPcalibMid ;
            v  =  v * (int32_t)RESX /  (max((int16_t)100,(v>0 ?
                                                              g_eeGeneral.x9dPcalibSpanPos :
                                                              g_eeGeneral.x9dPcalibSpanNeg)));
						}
#endif	// REV9E
#endif	// REVPLUS
						else
						{
 #endif
            v -= g_eeGeneral.calibMid[i];
#ifdef REVX
						if ( i < 4 )
						{
							if ( ( v > -1) && ( v < 1 ) )
							{
								v = 0 ;
							}
						}
#endif
            v  =  v * (int32_t)RESX /  (max((int16_t)100,(v>0 ?
                                                              g_eeGeneral.calibSpanPos[i] :
                                                              g_eeGeneral.calibSpanNeg[i])));
 #ifdef PCBX9D
						}
 #endif
#endif
            if(v <= -RESX) v = -RESX;
            if(v >=  RESX) v =  RESX;
	  				if ( g_eeGeneral.throttleReversed )
						{
							if ( i == THR_STICK )
							{
								v = -v ;
							}
						}
#ifdef FIX_MODE
						uint8_t index = i ;
						if ( i < 4 )
						{
            	phyStick[i] = v >> 4 ;
							index = stickScramble[stickIndex+i] ;
						}
#else
						uint8_t index = i ;
#endif
            calibratedStick[index] = v; //for show in expo

						// Filter beep centre
						{
							int8_t t = v/16 ;
#ifdef FIX_MODE
							uint8_t mask = 1 << index ;
#else
							uint8_t mask = 1<<(CONVERT_MODE((i+1))-1) ;
#endif
							if ( t < 0 )
							{
								t = -t ;		//abs(t)
							}
							if ( t <= 1 )
							{
            		anaCenter |= ( t == 0 ) ? mask : bpanaCenter & mask ;
							}
						}

            if(i<4) { //only do this for sticks
                //===========Trainer mode================
                if (!(att&NO_TRAINER) && g_model.traineron) {
                    TrainerMix* td = &g_eeGeneral.trainer.mix[index];
										exTrainerMix *xtd = &g_eeGeneral.exTrainer[index] ;
                    if (td->mode && getSwitch(xtd->swtch, 1))
										{
											if ( ppmInValid )
											{
                        uint8_t chStud = td->srcChn ;
                        int32_t vStud  = (g_ppmIns[chStud]- g_eeGeneral.trainer.calib[chStud]) /* *2 */ ;
//                        vStud /= 2 ;		// Only 2, because no *2 above
                        vStud *= xtd->studWeight ;
                        vStud /= 50 ;
//                        vStud /= 31 ;
//                        vStud *= 4 ;
                        switch ((uint8_t)td->mode) {
                        case 1: v += vStud;   break; // add-mode
                        case 2: v  = vStud;   break; // subst-mode
                        }
#ifdef FIX_MODE
												if ( index == 2 )
#else
												if ( index == THR_STICK )
#endif
												{
													trainerThrottleValue = vStud ;
													trainerThrottleValid = 1 ;
												}												 
											}
                    }
                }

                //===========Swash Ring================
                if(d && (index==ele_stick || index==ail_stick))
                    v = int32_t(v)*calc100toRESX(g_model.swashRingValue)/int32_t(d) ;
                //===========Swash Ring================

								if ( att & FADE_FIRST )
								{
#ifdef FIX_MODE
    		        	rawSticks[index] = v; //set values for mixer
#else
      		      	rawSticks[i] = v; //set values for mixer
#endif
								}
								v = calcExpo( index, v ) ;
//                uint8_t expoDrOn = get_dr_state(index);
//                uint8_t stkDir = v>0 ? DR_RIGHT : DR_LEFT;

//                if(IS_THROTTLE(index) && g_model.thrExpo){
//                    v  = 2*expo((v+RESX)/2,REG(g_model.expoData[index].expo[expoDrOn][DR_EXPO][DR_RIGHT], -100, 100));
//                    stkDir = DR_RIGHT;
//                }
//                else
//                    v  = expo(v,REG(g_model.expoData[index].expo[expoDrOn][DR_EXPO][stkDir], -100, 100));

//                int32_t x = (int32_t)v * (REG(g_model.expoData[index].expo[expoDrOn][DR_WEIGHT][stkDir]+100, 0, 100))/100;
//                v = (int16_t)x;
//                if (IS_THROTTLE(index) && g_model.thrExpo) v -= RESX;

                trimA[i] = getTrimValue( CurrentPhase, i )*2 ; //    if throttle trim -> trim low end
            }
						if ( att & FADE_FIRST )
						{
							if ( i < 7 )
							{
#ifdef FIX_MODE
        	    	anas[index] = v ; //set values for mixer
#else
	          	  anas[i] = v ; //set values for mixer
#endif
							}
						}
			    	if(att&NO_INPUT)
						{ //zero input for setStickCenter()
		      	  if ( i < 4 )
							{
    	    	    if(!IS_THROTTLE(index))
								{
									if ( ( v > (RESX/100 ) ) || ( v < -(RESX/100) ) )
									{
#ifdef FIX_MODE
				            anas[index] = 0; //set values for mixer
#else
          	      	anas[i]  = 0;
#endif
									}
//          	      trimA[index] = 0;
          	      trimA[i] = 0;
      	  	    }
        				anas[i+PPM_BASE] = 0;
        			}
    				}
        }
				//    if throttle trim -> trim low end
        if(g_model.thrTrim)
				{
					int8_t ttrim ;
//#ifdef PHASES		
					ttrim = getTrimValue( CurrentPhase, 2 ) ;
//#else
//					ttrim = *TrimPtr[2] ;
//#endif
					if(g_eeGeneral.throttleReversed)
					{
						ttrim = -ttrim ;
					}
         	trimA[2] = ((int32_t)ttrim+125)*(RESX-anas[2])/(RESX) ;
				}
			if ( att & FADE_FIRST )
			{

        //===========BEEP CENTER================
        anaCenter &= g_model.beepANACenter;
        if(((bpanaCenter ^ anaCenter) & anaCenter)) audioDefevent(AU_POT_STICK_MIDDLE);
        bpanaCenter = anaCenter;

				// Set up anas[] array
        anas[MIX_MAX-1]  = RESX;     // MAX
        anas[MIX_FULL-1] = RESX;     // FULL
#ifdef PCBSKY
        anas[MIX_3POS-1] = keyState(SW_ID0) ? -1024 : (keyState(SW_ID1) ? 0 : 1024) ;
#endif
#ifdef PCBX9D
        anas[MIX_3POS-1] = keyState(SW_SC0) ? -1024 : (keyState(SW_SC1) ? 0 : 1024) ;
#endif
        
        for(uint8_t i=0;i<4;i++) anas[i+PPM_BASE] = (g_ppmIns[i] - g_eeGeneral.trainer.calib[i])*2; //add ppm channels
        for(uint8_t i=4;i<NUM_PPM;i++)    anas[i+PPM_BASE]   = g_ppmIns[i]*2; //add ppm channels
        for(uint8_t i=0;i<NUM_SKYCHNOUT;i++) anas[i+CHOUT_BASE] = chans[i]; //other mixes previous outputs
#if GVARS
        for(uint8_t i=0;i<MAX_GVARS;i++) anas[i+MIX_3POS] = g_model.gvars[i].gvar * 1024 / 100 ;
#endif
//        for(uint8_t i=0;i<8;i++) anas[i+NUM_SKYXCHNRAW+1+MAX_GVARS+1] = g_ppmIns[i+8]*2; //add ppm channels

        //===========Swash Ring================
        if(g_model.swashRingValue)
        {
          uint32_t v = ((int32_t)anas[ele_stick]*anas[ele_stick] + (int32_t)anas[ail_stick]*anas[ail_stick]);
		      int16_t tmp = calc100toRESX(g_model.swashRingValue) ;
          uint32_t q ;
          q = (int32_t)tmp * tmp ;
          if(v>q)
          {
            uint16_t d = isqrt32(v);
            anas[ele_stick] = (int32_t)anas[ele_stick]*tmp/((int32_t)d) ;
            anas[ail_stick] = (int32_t)anas[ail_stick]*tmp/((int32_t)d) ;
          }
        }

#define REZ_SWASH_X(x)  ((x) - (x)/8 - (x)/128 - (x)/512)   //  1024*sin(60) ~= 886
#define REZ_SWASH_Y(x)  ((x))   //  1024 => 1024

        if(g_model.swashType)
        {
            int16_t vp = 0 ;
            int16_t vr = 0 ;

            if( !(att & NO_INPUT) )  //zero input for setStickCenter()
						{
	            vp = anas[ele_stick]+trimA[ele_stick];
  	          vr = anas[ail_stick]+trimA[ail_stick];
						}

            int16_t vc = 0;
            if(g_model.swashCollectiveSource)
                vc = anas[g_model.swashCollectiveSource-1];

            if(g_model.swashInvertELE) vp = -vp;
            if(g_model.swashInvertAIL) vr = -vr;
            if(g_model.swashInvertCOL) vc = -vc;

            switch (( uint8_t)g_model.swashType)
            {
            case (SWASH_TYPE_120):
                vp = REZ_SWASH_Y(vp);
                vr = REZ_SWASH_X(vr);
                anas[MIX_CYC1-1] = vc - vp;
                anas[MIX_CYC2-1] = vc + vp/2 + vr;
                anas[MIX_CYC3-1] = vc + vp/2 - vr;
                break;
            case (SWASH_TYPE_120X):
                vp = REZ_SWASH_X(vp);
                vr = REZ_SWASH_Y(vr);
                anas[MIX_CYC1-1] = vc - vr;
                anas[MIX_CYC2-1] = vc + vr/2 + vp;
                anas[MIX_CYC3-1] = vc + vr/2 - vp;
                break;
            case (SWASH_TYPE_140):
                vp = REZ_SWASH_Y(vp);
                vr = REZ_SWASH_Y(vr);
                anas[MIX_CYC1-1] = vc - vp;
                anas[MIX_CYC2-1] = vc + vp + vr;
                anas[MIX_CYC3-1] = vc + vp - vr;
                break;
            case (SWASH_TYPE_90):
                vp = REZ_SWASH_Y(vp);
                vr = REZ_SWASH_Y(vr);
                anas[MIX_CYC1-1] = vc - vp;
                anas[MIX_CYC2-1] = vc + vr;
                anas[MIX_CYC3-1] = vc - vr;
                break;
            default:
                break;
            }
	        }
  		  }

    		if(tick10ms)
				{
					inactivityCheck() ;
					trace(); //trace thr 0..32  (/32)
				}
			}
    memset(chans,0,sizeof(chans));        // All outputs to 0


    uint8_t mixWarning = 0;
    //========== MIXER LOOP ===============

    // Set the trim pointers back to the master set
//    TrimPtr[0] = &g_model.trim[0] ;
//    TrimPtr[1] = &g_model.trim[1] ;
//    TrimPtr[2] = &g_model.trim[2] ;
//    TrimPtr[3] = &g_model.trim[3] ;

    for(uint8_t i=0;i<MAX_SKYMIXERS;i++)
		{
//        MixData *md = mixaddress( i ) ;
        SKYMixData *md = &g_model.mixData[i] ;
				int8_t mixweight = REG( md->weight, -100, 100 ) ;

        if((md->destCh==0) || (md->destCh>NUM_SKYCHNOUT)) break;

        //Notice 0 = NC switch means not used -> always on line
        int16_t v  = 0;
        uint8_t swTog;
        uint8_t swon = swOn[i] ;
				
				bool t_switch = getSwitch(md->swtch,1) ;
        if (md->swtch && (md->srcRaw >= PPM_BASE) && (md->srcRaw < PPM_BASE+NUM_PPM)	&& (ppmInValid == 0) )
				{
					// then treat switch as false ???				
					t_switch = 0 ;
				}	
        if (md->swtch && (md->srcRaw > MIX_3POS+MAX_GVARS + NUM_SCALERS ) && (ppmInValid == 0) )
				{ // Extra PPM inputs (9-16)
					if (md->srcRaw <= MIX_3POS+MAX_GVARS + NUM_SCALERS + NUM_EXTRA_PPM )
					{
						// then treat switch as false ???				
						t_switch = 0 ;
					}
				}
      
        if ( t_switch )
				{
					if ( md->modeControl & ( 1 << CurrentPhase ) )
					{
						t_switch = 0 ;
					}
				}

        uint8_t k = md->srcRaw ;

#define DEL_MULT 256
				//swOn[i]=false;
        if(!t_switch)
        { // switch on?  if no switch selected => on
            swTog = swon ;
            swOn[i] = swon = false ;
            if (k == MIX_3POS+MAX_GVARS+1) act[i] = chans[md->destCh-1] * DEL_MULT / 100 ;
            if( k !=MIX_MAX && k !=MIX_FULL) continue;// if not MAX or FULL - next loop
            if(md->mltpx==MLTPX_REP) continue; // if switch is off and REPLACE then off
            v = ( k == MIX_FULL ? -RESX : 0); // switch is off and it is either MAX=0 or FULL=-512
        }
        else {
            swTog = !swon ;
            swon = true;
            k -= 1 ;
						v = anas[k]; //Switch is on. MAX=FULL=512 or value.
						if ( k < 4 )
						{
							if ( md->disableExpoDr )
							{
     		      	v = rawSticks[k]; //Switch is on. MAX=FULL=512 or value.
							}
						}
#ifdef PCBX9D
						if ( k == MIX_3POS-1 )
						{
							uint32_t /*EnumKeys*/ sw = switchIndex[md->switchSource] ;
							if ( ( md->switchSource == 5) || ( md->switchSource == 7) )
							{ // 2-POS switch
        				v = hwKeyState(sw) ? 1024 : -1024 ;
//        				v = keyState(sw) ? -1024 : 1024 ;
							}
							else
							{ // 3-POS switch
        				v = hwKeyState(sw) ? -1024 : (hwKeyState(sw+1) ? 0 : 1024) ;
//        				v = keyState(sw) ? -1024 : (keyState((EnumKeys)(sw+1)) ? 0 : 1024) ;
							}
						}
#endif
#ifdef PCBSKY
						if ( k == MIX_3POS-1 )
						{
							uint32_t sw = Sw3PosList[md->switchSource] ;
							if ( Sw3PosCount[md->switchSource] == 2 )
							{
        				v = hwKeyState(sw) ? 1024 : -1024 ;
							}
							else if ( Sw3PosCount[md->switchSource] == 6 )
							{
								v = ((int32_t)switchPosition( HSW_Ele6pos0 ) * 2048 - 5120)/5 ;
							}
							else
							{
        				v = hwKeyState(sw) ? -1024 : (hwKeyState(sw+1) ? 0 : 1024) ;
							}
						}
#endif
//            if(k>=CHOUT_BASE && (k<i)) v = chans[k]; // if we've already calculated the value - take it instead // anas[i+CHOUT_BASE] = chans[i]
						if ( k >= CHOUT_BASE )
            {
							if(k<CHOUT_BASE+NUM_SKYCHNOUT)
							{
								if ( md->disableExpoDr )
								{
									v = g_chans512[k-CHOUT_BASE] ;
								}
								else
								{
            			if(k<CHOUT_BASE+md->destCh-1)
									{
										v = chans[k-CHOUT_BASE] / 100 ; // if we've already calculated the value - take it instead // anas[i+CHOUT_BASE] = chans[i]
									}
									else
									{
										v = ex_chans[k-CHOUT_BASE] ;
									}
								}
							}
						}
						if (k == MIX_3POS+MAX_GVARS) v = chans[md->destCh-1] / 100 ;	// "THIS"
            if ( (k > MIX_3POS+MAX_GVARS) && ( k <= MIX_3POS+MAX_GVARS + NUM_SCALERS ) )
						{
							 v = calc_scaler( k - (MIX_3POS+MAX_GVARS+1), 0, 0 ) ;
            }
            if (k > MIX_3POS+MAX_GVARS + NUM_SCALERS)
						{
							if ( k <= MIX_3POS+MAX_GVARS + NUM_SCALERS + NUM_EXTRA_PPM )
							{
								v = g_ppmIns[k-(MIX_3POS+MAX_GVARS + NUM_SCALERS + 1) + 8]*2 ;
							}
							else	// ( k >= EXTRA_POTS_START )
							{
								// An extra pot
								v = calibratedStick[k-EXTRA_POTS_START+8] ;
							}
						}
						if(md->mixWarn) mixWarning |= 1<<(md->mixWarn-1); // Mix warning
//            if ( md->enableFmTrim )
//            {
//                if ( k <= 4 )
//                {
//                    TrimPtr[k] = &md->sOffset ;		// Use the value stored here for the trim
//                }
//            }
        }
        swOn[i] = swon ;

        //========== INPUT OFFSET ===============
//        if ( ( md->enableFmTrim == 0 ) && ( md->lateOffset == 0 ) )
        if ( md->lateOffset == 0 )
        {
#if GVARS
            if(md->sOffset) v += calc100toRESX( REG( md->sOffset, -125, 125 )	) ;
#else
            if(md->sOffset) v += calc100toRESX(md->sOffset);
#endif
        }

        //========== DELAY and PAUSE ===============
        if (md->speedUp || md->speedDown || md->delayUp || md->delayDown)  // there are delay values
        {

					int16_t my_delay = sDelay[i] ;
					int32_t tact = act[i] ;
            //if(init) {
            //act[i]=(int32_t)v*DEL_MULT;
            //swTog = false;
            //}
#if DEL_MULT == 256
						int16_t diff = v-(tact>>8) ;
#else
            int16_t diff = v-tact/DEL_MULT;
#endif
						if ( ( diff > 10 ) || ( diff < -10 ) )
						{
							if ( my_delay == 0 )
							{
        				if (md->delayUp || md->delayDown)  // there are delay values
								{								
									swTog = 1 ;
								}
							}
						}
						else
						{
							my_delay = 0 ;							
						}

            if(swTog) {
                //need to know which "v" will give "anas".
                //curves(v)*weight/100 -> anas
                // v * weight / 100 = anas => anas*100/weight = v
                if(md->mltpx==MLTPX_REP)
                {
                    tact = (int32_t)anas[md->destCh-1+CHOUT_BASE]*DEL_MULT * 100;
                    if(mixweight) tact /= mixweight ;
                }
                diff = v-tact/DEL_MULT;
                if(diff) my_delay = (diff<0 ? md->delayUp :  md->delayDown) * 10 ;
            }

            if(my_delay > 0)
						{ // perform delay
              if(tick10ms)
              {
                my_delay -= 1 ;
							}
              if ( my_delay != 0)
             	{ // At end of delay, use new V and diff
#if DEL_MULT == 256
               	v = tact >> 8 ;	   // Stay in old position until delay over
#else
                v = tact/DEL_MULT;   // Stay in old position until delay over
#endif
                diff = 0;
 	            }
							else
							{
								my_delay = -1 ;
							}
            }

					sDelay[i] = my_delay ;

            if(diff && (md->speedUp || md->speedDown)){
                //rate = steps/sec => 32*1024/100*md->speedUp/Down
                //act[i] += diff>0 ? (32768)/((int16_t)100*md->speedUp) : -(32768)/((int16_t)100*md->speedDown);
                //-100..100 => 32768 ->  100*83886/256 = 32768,   For MAX we divide by 2 sincde it's asymmetrical
                if(tick10ms) {
                    int32_t rate = (int32_t)DEL_MULT*2048*100;
                    if(mixweight) rate /= abs(mixweight);
// The next few lines could replace the long line act[i] = etc. - needs testing
										int16_t speed ;
                    if ( diff>0 )
										{
											speed = md->speedUp ;
										}
										else
										{
											rate = -rate ;											
											speed = md->speedDown ;
										}
										tact = (speed) ? tact+(rate)/((int16_t)10*speed) : (int32_t)v*DEL_MULT ;

//                    act[i] = (diff>0) ? ((md->speedUp>0)   ? act[i]+(rate)/((int16_t)10*(md->speedUp))   :  (int32_t)v*DEL_MULT) :
//                                        ((md->speedDown>0) ? act[i]-(rate)/((int16_t)10*(md->speedDown)) :  (int32_t)v*DEL_MULT) ;
                }
								{
#if DEL_MULT == 256
									int32_t tmp = tact>>8 ;
#else
									int32_t tmp = tact/DEL_MULT ;
#endif
                	if(((diff>0) && (v<tmp)) || ((diff<0) && (v>tmp))) tact=(int32_t)v*DEL_MULT; //deal with overflow
                }
#if DEL_MULT == 256
                v = tact >> 8 ;
#else
                v = tact/DEL_MULT;
#endif
            }
            else if (diff)
            {
              tact=(int32_t)v*DEL_MULT;
            }
					act[i] = tact ;
        }

        //========== CURVES ===============
				if ( md->differential )
				{
      		//========== DIFFERENTIAL =========
      		int8_t curveParam = REG( md->curve, -100, 100 ) ;
      		if (curveParam > 0 && v < 0)
      		  v = (v * (100 - curveParam)) / 100;
      		else if (curveParam < 0 && v > 0)
      		  v = (v * (100 + curveParam)) / 100;
				}
				else
				{
					if ( md->curve <= -28 )
					{
						// do expo using md->curve + 128
      			v = expo( v, md->curve + 128 ) ;
					}
					else
					{
        		switch(md->curve){
        		case 0:
        		    break;
        		case 1:
        		    if(md->srcRaw == MIX_FULL) //FUL
        		    {
        		        if( v<0 ) v=-RESX;   //x|x>0
        		        else      v=-RESX+2*v;
        		    }else{
        		        if( v<0 ) v=0;   //x|x>0
        		    }
        		    break;
        		case 2:
        		    if(md->srcRaw == MIX_FULL) //FUL
        		    {
        		        if( v>0 ) v=RESX;   //x|x<0
        		        else      v=RESX+2*v;
        		    }else{
        		        if( v>0 ) v=0;   //x|x<0
        		    }
        		    break;
        		case 3:       // x|abs(x)
        		    v = abs(v);
        		    break;
        		case 4:       //f|f>0
        		    v = v>0 ? RESX : 0;
        		    break;
        		case 5:       //f|f<0
        		    v = v<0 ? -RESX : 0;
        		    break;
        		case 6:       //f|abs(f)
        		    v = v>0 ? RESX : -RESX;
        		    break;
        		default: //c1..c16
								{
									int8_t idx = md->curve ;
									if ( idx < 0 )
									{
										v = -v ;
										idx = 6 - idx ;								
									}
        		    	v = intpol(v, idx - 7);
								}
        		}
					}
				}

        //========== TRIM ===============
        if((md->carryTrim==0) && (md->srcRaw>0) && (md->srcRaw<=4)) v += trimA[md->srcRaw-1];  //  0 = Trim ON  =  Default

        //========== MULTIPLEX ===============
#if GVARS
        int32_t dv = (int32_t)v*mixweight ;
#else
        int32_t dv = (int32_t)v*md->weight;
#endif
				
        //========== lateOffset ===============
//				if ( ( md->enableFmTrim == 0 ) && ( md->lateOffset ) )
				if ( md->lateOffset )
        {
#if GVARS
            if(md->sOffset) dv += calc100toRESX( REG( md->sOffset, -125, 125 )	) * 100 ;
#else
            if(md->sOffset) dv += calc100toRESX(md->sOffset) * 100 ;
#endif
        }
				
				int32_t *ptr ;			// Save calculating address several times
				ptr = &chans[md->destCh-1] ;
        switch((uint8_t)md->mltpx){
        case MLTPX_REP:
            *ptr = dv;
            break;
        case MLTPX_MUL:
						dv /= 100 ;
						dv *= *ptr ;
            dv /= RESXl;
            *ptr = dv ;
//            chans[md->destCh-1] *= dv/100l;
//            chans[md->destCh-1] /= RESXl;
            break;
        default:  // MLTPX_ADD
            *ptr += dv; //Mixer output add up to the line (dv + (dv>0 ? 100/2 : -100/2))/(100);
            break;
        }
    }

    //========== MIXER WARNING ===============
    //1= 00,08
    //2= 24,32,40
    //3= 56,64,72,80
    {
        uint16_t tmr10ms ;
        tmr10ms = get_tmr10ms() ;

        if(mixWarning & 1) if(((tmr10ms&0xFF)==  0)) audioDefevent(AU_MIX_WARNING_1);
        if(mixWarning & 2) if(((tmr10ms&0xFF)== 64) || ((tmr10ms&0xFF)== 72)) audioDefevent(AU_MIX_WARNING_2);
        if(mixWarning & 4) if(((tmr10ms&0xFF)==128) || ((tmr10ms&0xFF)==136) || ((tmr10ms&0xFF)==144)) audioDefevent(AU_MIX_WARNING_3);        


    }

		ThrottleStickyOn = 0 ;
    //========== LIMITS ===============
    for(uint8_t i=0;i<NUM_SKYCHNOUT;i++)
		{
        // chans[i] holds data from mixer.   chans[i] = v*weight => 1024*100
        // later we multiply by the limit (up to 100) and then we need to normalize
        // at the end chans[i] = chans[i]/100 =>  -1024..1024
        // interpolate value with min/max so we get smooth motion from center to stop
        // this limits based on v original values and min=-1024, max=1024  RESX=1024

        int32_t q = chans[i];// + (int32_t)g_model.limitData[i].offset*100; // offset before limit

				if ( Fade.fadePhases )
				{
					int32_t l_fade = Fade.fade[i] ;
					if ( att & FADE_FIRST )
					{
						l_fade = 0 ;
					}
					l_fade += ( q / 100 ) * Fade.fadeScale[CurrentPhase] ;
					Fade.fade[i] = l_fade ;
			
					if ( ( att & FADE_LAST ) == 0 )
					{
						continue ;
					}
					l_fade /= Fade.fadeWeight ;
					q = l_fade * 100 ;
				}
    	  chans[i] = q / 100 ; // chans back to -1024..1024
        
				ex_chans[i] = chans[i]; //for getswitch

        LimitData *limit = &g_model.limitData[i] ;
				int16_t ofs = limit->offset;
				int16_t xofs = ofs ;
				if ( xofs > 50 )
				{
					xofs = 50 ;					
				}
				else if ( xofs < -50 )
				{
					xofs = -50 ;
				}
        int16_t lim_p = 10*(limit->max+100) + xofs ;
        int16_t lim_n = 10*(limit->min-100) + xofs ; //multiply by 10 to get same range as ofs (-1000..1000)
				if ( lim_p > 1250 )
				{
					lim_p = 1250 ;
				}
				if ( lim_n < -1250 )
				{
					lim_n = -1250 ;
				}
        if(ofs>lim_p) ofs = lim_p;
        if(ofs<lim_n) ofs = lim_n;

        if(q) q = (q>0) ?
                    q*((int32_t)lim_p-ofs)/100000 :
                    -q*((int32_t)lim_n-ofs)/100000 ; //div by 100000 -> output = -1024..1024

        q += calc1000toRESX(ofs);
        lim_p = calc1000toRESX(lim_p);
        lim_n = calc1000toRESX(lim_n);
        if(q>lim_p) q = lim_p;
        if(q<lim_n) q = lim_n;
        if(g_model.limitData[i].revert) q=-q;// finally do the reverse.

				{
					uint8_t numSafety = NUM_SKYCHNOUT - g_model.numVoice ;
					if ( i < numSafety )
					{
        		if(g_model.safetySw[i].opt.ss.swtch)  //if safety sw available for channel check and replace val if needed
						{
							if ( ( g_model.safetySw[i].opt.ss.mode != 1 ) && ( g_model.safetySw[i].opt.ss.mode != 2 ) )	// And not used as an alarm
							{
								static uint8_t sticky = 0 ;
								uint8_t applySafety = 0 ;
								int8_t sSwitch = g_model.safetySw[i].opt.ss.swtch ;
								
								if(getSwitch( sSwitch,0))
								{
									applySafety = 1 ;
								}

								if ( g_model.safetySw[i].opt.ss.mode == 3 )
								{
									// Special case, sticky throttle
									if( applySafety )
									{
										sticky = 0 ;
									}
#ifdef FIX_MODE
									else
									{
										uint32_t throttleOK = 0 ;
										if ( g_model.throttleIdle )
										{
											if ( abs( calibratedStick[2] ) < 20 )
											{
												throttleOK = 1 ;
											}
										}
										else
										{
  										if(calibratedStick[2] < -1004)
  										{
												throttleOK = 1 ;
  										}
										}
										
										if ( throttleOK )
										{
											if ( trainerThrottleValid )
											{
												if ( trainerThrottleValue < -1004 )
												{
													sticky = 1 ;
												}
											}	
											else
											{
												sticky = 1 ;
											}
										}
									}
#else
									else if ( calibratedStick[THR_STICK] < -1004 )
									{
										if ( trainerThrottleValid )
										{
											if ( trainerThrottleValue < -1004 )
											{
												sticky = 1 ;
											}
										}	
										else
										{
											sticky = 1 ;
										}
									}
#endif
									if ( sticky == 0 )
									{
										applySafety = 1 ;
									}
									ThrottleStickyOn = applySafety ;
								}
								if ( applySafety ) q = calc100toRESX(g_model.safetySw[i].opt.ss.val) ;
							}
						}
					}
				}
//        cli();
        chanOut[i] = q; //copy consistent word to int-level
//        sei();
		}
}


void menuProcSDstat(uint8_t event)
{
//  MENU(PSTR(STR_ST_CARD_STAT), menuTabDiag, e_Setup3, 1, {0/*, 0*/});
	MENU(PSTR(STR_ST_CARD_STAT), menuTabStat, e_Setup3, 1, {0} ) ;

#ifdef PCBX9D
	if ( event == EVT_ENTRY )
	{
extern void sdInit( void ) ;
		sdInit() ;
	}
#endif
	 
	int8_t  sub    = mstate2.m_posVert;
//	uint8_t subSub = mstate2.m_posHorz;
//	bool    edit;
//	uint8_t blink ;

	evalOffset(sub, 1);

	lcd_puts_Pleft( 1*FH, PSTR(STR_4_READY));

#ifndef SIMU

#ifdef PCBSKY
	uint32_t i ;
	uint8_t x, y ;
#endif

#ifdef PCBX9D
extern uint8_t CardType ;
extern uint32_t sdMounted( void ) ;
	lcd_outhex4( 10*FW, 1*FH, CardType ) ;
	lcd_outhex4( 16*FW, 1*FH, sdMounted() ) ;
	lcd_outhex4( 10*FW, 2*FH, Card_state ) ;
#endif

#ifdef PCBSKY
	lcd_outhex4( 10*FW, 1*FH, Card_state ) ;
	 
extern uint32_t SDlastError ;
	lcd_outhex4( 16*FW, 1*FH, SDlastError ) ;
	 
//	if (sd_card_ready() )		// Moved for debugging
//	{
		y = 2*FH ;
		x = 4*FW ;
		lcd_puts_Pleft( y, XPSTR("CID"));
		for ( i = 0 ; i < 4 ; i += 1 )
		{
		  lcd_outhex4( x, y, Card_ID[i] >> 16 ) ;
		  lcd_outhex4( x+4*FW, y, Card_ID[i] ) ;
			x += 8*FW ;
			if ( i == 1 )
			{
				y += FH ;
				x = 4*FW ;				
			}			 
		}
		y = 4*FH ;
		x = 4*FW ;
		lcd_puts_Pleft( y, XPSTR("CSD"));
		for ( i = 0 ; i < 4 ; i += 1 )
		{
		  lcd_outhex4( x, y, Card_CSD[i] >> 16 ) ;
		  lcd_outhex4( x+4*FW, y, Card_CSD[i] ) ;
			x += 8*FW ;
			if ( i == 1 )
			{
				y += FH ;
				x = 4*FW ;				
			}			 
		}
		y = 6*FH ;
		x = 4*FW ;
		lcd_puts_Pleft( y, XPSTR("SCR"));
		for ( i = 0 ; i < 2 ; i += 1 )
		{
		  lcd_outhex4( x, y, Card_SCR[i] >> 16 ) ;
		  lcd_outhex4( x+4*FW, y, Card_SCR[i] ) ;
			x += 8*FW ;
		}
#endif
	if (sd_card_ready() )
	{
	}
	else
	{
		lcd_puts_Pleft( 1*FH, PSTR(STR_NOT));
	}
#endif
}


uint8_t evalOffset(int8_t sub, uint8_t max)
{
  uint8_t t_pgOfs = s_pgOfs ;
	int8_t x = sub-t_pgOfs ;
    if(sub<1) t_pgOfs=0;
    else if(x>max) t_pgOfs = sub-max;
    else if(x<max-6) t_pgOfs = sub-max+6;
		return (s_pgOfs = t_pgOfs) ;
}

const char Str_Display[] =     "Display" ;
const char Str_AudioHaptic[] = "AudioHaptic" ;
const char Str_Alarms[] =      "Alarms" ;
const char Str_General[] =     "General" ;
const char Str_Controls[] =    "Controls" ;
const char Str_Hardware[] =    "Hardware" ;
const char Str_Calibration[] = "Calibration" ;
const char Str_Trainer[] =     "Trainer" ;
const char Str_Version[] =     "Version" ;
const char Str_ModuleRssi[] =  "FrSky xSSI" ;
const char Str_DateTime[] =    "Date-Time" ;
const char Str_DiagSwtch[] =   "DiagSwtch" ; 
const char Str_DiagAna[] =     "DiagAna" ;
			
#define M_INDEX			0
#define M_DISPLAY		1
#define M_AUDIO			2
#define M_ALARMS		3
#define M_GENERAL		4
#define M_CONTROLS	5
#define M_HARDWARE	6
#define M_CALIB			7
#define M_TRAINER		8
#define M_VERSION		9
#define M_MODULE		10
#define M_DATE			11
#define M_DIAGKEYS	12
#define M_DIAGANA		13

static void displayIndex( const char *const strings[], uint8_t extra, uint8_t lines, uint8_t highlight )
{
	uint8_t offset = 7 ;
	if ( extra == 8 )
	{
		offset = 8 ;
		lcd_puts_P( 69, 0, (const char *)pgm_read_adr( &strings[7] ) ) ;
	}
	for ( uint8_t i = 0 ; i < lines ; i += 1 )
	{
		lcd_puts_Pleft((i+1)*FH, (const char *)pgm_read_adr( &strings[i] ) ) ;
		if ( i < extra )
		{
			lcd_puts_P( 69, (i+1)*FH, (const char *)pgm_read_adr( &strings[i+offset] ) ) ;
		}
	} 
	
	lcd_vline( 67, 0, 63 ) ;

	if ( highlight )
	{
		if ( highlight > 7 )
		{
			lcd_char_inverse( 69, (highlight-offset)*FH, 59, 0 ) ;
		}
		else
		{
			lcd_char_inverse( 0, highlight*FH, 66, 0 ) ;
		}
	}
}

static uint8_t SubMenuCall = 0 ;

static uint8_t indexProcess( uint8_t event, MState2 *pmstate, uint8_t extra )
{
	if (event == EVT_ENTRY)
	{
//		MenuTimer = 2000 ;	// * 0.01 Seconds = 20 seconds
		pmstate->m_posVert = SubmenuIndex - 1 ;
		SubmenuIndex = 0 ;
//		SubMenuFromIndex = 0 ;
	}
	if (event == EVT_ENTRY_UP)
	{
		pmstate->m_posVert = SubmenuIndex - 1 ;
		if ( SubMenuCall )
		{
			SubMenuCall = 0 ;
		}
		else
		{
			SubmenuIndex = 0 ;
//			SubMenuFromIndex = 0 ;
		}
	}
	
	if ( UseLastSubmenuIndex )
	{
		SubmenuIndex = LastSubmenuIndex & 0x7F ;
		UseLastSubmenuIndex = 0 ;
//		SubMenuFromIndex = 0 ;
	}
	
	if ( SubmenuIndex )
	{
  	if (event == EVT_KEY_LONG(KEY_EXIT) )
		{
      s_editMode = false;
			pmstate->m_posVert = SubmenuIndex ;
			SubmenuIndex = 0 ;
			killEvents(event) ;
			event = 0 ;
		}
  	if (event == EVT_KEY_FIRST(KEY_EXIT) )
		{
			event = 0 ;
		}
  	if ( (event == EVT_KEY_BREAK(KEY_EXIT) ) || (event == EVT_KEY_LONG(BTN_RE) ) )
		{
      if(s_editMode)
			{
        s_editMode = false;
			}
			else
			{
				pmstate->m_posVert = SubmenuIndex - 1 ;
				SubmenuIndex = 0 ;
				killEvents(event) ;
				audioDefevent(AU_MENUS) ;
			}
			event = 0 ;
		}
	}
	else
	{
		uint8_t pv = pmstate->m_posVert ;
		if (event == EVT_KEY_FIRST(KEY_RIGHT) )
		{
			if ( pv < ((extra == 8) ? 7 : extra) )
			{
				pv += (extra == 8) ? 8 : 7 ;
//				pv += 7 ;
			}
		}
		if (event == EVT_KEY_FIRST(KEY_LEFT) )
		{
			if ( pv >= ((extra == 8) ? 8 : 7) )
			{
				pv -= (extra == 8) ? 8 : 7 ;
			}
		}

 		if ( (event == EVT_KEY_FIRST(KEY_MENU) ) ||(event == EVT_KEY_BREAK(BTN_RE) ) )
		{
			SubmenuIndex = pv + 1 ;
			LastSubmenuIndex = SubmenuIndex | 0x80 ;
			pv = 0 ;
			killEvents(event) ;
			event = EVT_ENTRY ;
		}
		pmstate->m_posVert = pv ;
	}
	return event ;
}

void menuProcIndex(uint8_t event)
{
	static MState2 mstate;
	static uint8_t Columns = 0 ;

//	if (event == EVT_ENTRY)
//	{
//		mstate.m_posVert = SubmenuIndex - 1 ;
//		SubmenuIndex = 0 ;
//	}
//	if (event == EVT_ENTRY_UP)
//	{
//		mstate.m_posVert = SubmenuIndex - 1 ;
//		SubmenuIndex = 0 ;
//	}

//	if ( UseLastSubmenuIndex )
//	{
//		SubmenuIndex = LastSubmenuIndex & 0x7F ;
//		UseLastSubmenuIndex = 0 ;
//	}

//	if ( SubmenuIndex )
//	{
//  	if (event == EVT_KEY_LONG(KEY_EXIT) )
//		{
//      s_editMode = false;
//			SubmenuIndex = 0 ;
//      popMenu(false);
//			killEvents(event) ;
//			event = 0 ;
//		}
////		uint8_t saveEvent = event ;
//  	if (event == EVT_KEY_FIRST(KEY_EXIT) )
//		{
//			event = 0 ;
//		}
//  	if ( (event == EVT_KEY_BREAK(KEY_EXIT) ) || (event == EVT_KEY_LONG(BTN_RE) ) )
//		{
//      if(s_editMode)
//			{
//        s_editMode = false;
//			}
//			else
//			{
//				mstate.m_posVert = SubmenuIndex - 1 ;
//				SubmenuIndex = 0 ;
//				killEvents(event) ;
//			}
//			event = 0 ;
//		}
//		mstate.check( event, 0, NULL, 0, &Columns, 0, IlinesCount-1 ) ;
////		event = saveEvent ;
////  	if ( (event == EVT_KEY_BREAK(BTN_RE)) || ( event == EVT_KEY_FIRST(KEY_MENU) ) )
////    {
////			s_editMode = !s_editMode;
////		}
//	}
//	else
//	{
//		uint32_t pv = mstate.m_posVert ;
//		if (event == EVT_KEY_FIRST(KEY_RIGHT) )
//		{
//			if ( pv < 5)
//			{
//				pv += 7 ;
//			}
//		}
//		if (event == EVT_KEY_FIRST(KEY_LEFT) )
//		{
//			if ( pv > 6)
//			{
//				pv -= 7 ;
//			}
//		}
	 
//  	if ( (event == EVT_KEY_FIRST(KEY_MENU) ) ||(event == EVT_KEY_BREAK(BTN_RE) ) )
//		{
//			SubmenuIndex = pv + 1 ;
//			LastSubmenuIndex = SubmenuIndex ;
//			pv = 0 ;
//			killEvents(event) ;
//			event = 0 ;
			
//		}
//		mstate.m_posVert = pv ;
//		mstate.check_simple(event, 0, NULL, 0, IlinesCount-1 ) ;
//	}
	
	event = indexProcess( event, &mstate, 6 ) ;
	event = mstate.check( event, 0, NULL, 0, &Columns, 0, IlinesCount-1 ) ;

//	if ( SubmenuIndex > 6 )
//	{
		switch ( SubmenuIndex )
		{
			case M_CALIB :
        pushMenu(menuProcDiagCalib) ;
			break ;
			case M_TRAINER :
        pushMenu(menuProcTrainer) ;
			break ;
			case M_VERSION :
        pushMenu(menuProcDiagVers) ;
			break ;
			case M_DATE :
        pushMenu(menuProcDate) ;
			break ;
			case M_MODULE :
        pushMenu(menuProcRSSI) ;
			break ;
			case M_DIAGKEYS :
        pushMenu(menuProcDiagKeys) ;
			break ;
			case M_DIAGANA :
        pushMenu(menuProcDiagAna) ;
			break ;
			default :
				if ( event == EVT_ENTRY )
				{
					audioDefevent(AU_MENUS) ;
					event = 0 ;
				}
			break ;
		}
//	}

	uint32_t sub = mstate.m_posVert ;
	uint32_t y ;
	Columns = 0 ;
	uint8_t blink = InverseBlink ;

	switch ( SubmenuIndex )
	{
		case 0 :
  		TITLE(XPSTR("Radio Setup"));
			IlinesCount = 13 ;
			sub += 1 ;
			
			
static const char *const n_Strings[] = {
Str_Display,
Str_AudioHaptic,
Str_Alarms,
Str_General,
Str_Controls,
Str_Hardware,
Str_Calibration,
Str_Trainer,
Str_Version,
Str_ModuleRssi,
Str_DateTime,
Str_DiagSwtch,
Str_DiagAna
} ;	
			
			displayIndex( n_Strings, 6, 7, sub ) ;
			
		break ;
		
		case M_DISPLAY :
		{	
			uint8_t subN = 0 ;
			
      TITLE( XPSTR("DISPLAY") ) ;
#ifdef PCBX9D
 #ifdef REVPLUS
			IlinesCount = 7 ;
 #else            
			IlinesCount = 6 ;
 #endif            
#else            
 #ifdef PCBSKY
  #ifndef REVX
			IlinesCount = 7 ;
  #else            
			IlinesCount = 6 ;
  #endif            
 #endif            
#endif            
			y = FH ;
      lcd_puts_Pleft( y, PSTR(STR_CONTRAST) ) ;
      lcd_outdezAtt( PARAM_OFS+2*FW, y, g_eeGeneral.contrast, (sub==subN) ? blink : 0) ;
      if(sub==subN)
			{
#ifdef REVPLUS
				CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.contrast, 10, 60);
#else            
				CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.contrast, 10, 45);
#endif            
				lcdSetRefVolt(g_eeGeneral.contrast);
      }
			y += FH ;
			subN += 1 ;
  		
			lcd_puts_Pleft( y, PSTR(STR_BRIGHTNESS)) ;
#ifdef PCBX9D
#ifdef REVPLUS
		  lcd_puts_P(12*FW, y, XPSTR( "COLOUR"));
#endif
#endif
			lcd_outdezAtt( PARAM_OFS+2*FW, y, 100-g_eeGeneral.bright, (sub==subN) ? blink : 0 ) ;
      if(sub==subN)
			{
				uint8_t b ;
				b = 100 - g_eeGeneral.bright ;
				CHECK_INCDEC_H_GENVAR_0( event, b, 100 ) ;
				g_eeGeneral.bright = 100 - b ;
#ifdef PCBSKY
				PWM->PWM_CH_NUM[0].PWM_CDTYUPD = g_eeGeneral.bright ;
#endif
#ifdef PCBX9D
#ifdef REVPLUS
				backlight_set( g_eeGeneral.bright, 0 ) ;
#else
				backlight_set( g_eeGeneral.bright ) ;
#endif
#endif
			}
			y += FH ;
			subN += 1 ;

#ifdef REVPLUS
 			lcd_puts_Pleft(    y, PSTR(STR_BRIGHTNESS));
   		lcd_puts_P(12*FW, y, XPSTR( "WHITE"));
			lcd_outdezAtt( PARAM_OFS+2*FW, y, 100-g_eeGeneral.bright_white, (sub==subN) ? blink : 0 ) ;
      if(sub==subN)
			{
				uint8_t b ;
				b = 100 - g_eeGeneral.bright_white ;
				CHECK_INCDEC_H_GENVAR_0( event, b, 100 ) ;
				g_eeGeneral.bright_white = 100 - b ;
				backlight_set( g_eeGeneral.bright_white, 1 ) ;
			}
			y += FH ;
			subN++;
#endif
      
			lcd_puts_Pleft( y,PSTR(STR_LIGHT_SWITCH));
      putsDrSwitches(PARAM_OFS-FW,y,g_eeGeneral.lightSw,sub==subN ? blink : 0);
      if(sub==subN) CHECK_INCDEC_GENERALSWITCH(event, g_eeGeneral.lightSw, -MaxSwitchIndex, MaxSwitchIndex) ;
			y += FH ;
			subN++;

			for ( uint8_t i = 0 ; i < 2 ; i += 1 )
			{
				uint8_t b ;
    	  lcd_puts_Pleft( y,( i == 0) ? PSTR(STR_LIGHT_AFTER) : PSTR(STR_LIGHT_STICK) );
				b = ( i == 0 ) ? g_eeGeneral.lightAutoOff : g_eeGeneral.lightOnStickMove ;

    	  if(b) {
    	      lcd_outdezAtt(PARAM_OFS, y, b*5,LEFT|(sub==subN ? blink : 0));
    	      lcd_putc(Lcd_lastPos, y, 's');
    	  }
    	  else
    	      lcd_putsnAtt(PARAM_OFS, y, PSTR(STR_OFF),3,(sub==subN ? blink:0));
    	  if(sub==subN) CHECK_INCDEC_H_GENVAR_0(event, b, 600/5);
				if ( i == 0 )
				{
					g_eeGeneral.lightAutoOff = b ;
				}
				else
				{
					g_eeGeneral.lightOnStickMove = b ;
				}
  			y += FH ;
				subN += 1 ;
			}
		
#ifdef PCBSKY
#ifndef REVX
//	  	g_eeGeneral.optrexDisplay = onoffMenuItem( g_eeGeneral.optrexDisplay, y, XPSTR("Optrex Display"), sub, subN, event ) ;
	  	g_eeGeneral.optrexDisplay = onoffMenuItem_g( event, g_eeGeneral.optrexDisplay, y, XPSTR("Optrex Display"), sub==subN ) ;
  		y += FH ;
			subN += 1 ;
#endif
#endif

			g_eeGeneral.flashBeep = onoffMenuItem_g( event, g_eeGeneral.flashBeep, y, PSTR(STR_FLASH_ON_BEEP), sub==subN ) ;
  		y += FH ;
			subN += 1 ;
			 
		}
		break ;
		
		case M_AUDIO :
		{	
			uint8_t y = 1*FH;
			uint8_t current_volume ;
			uint8_t subN = 0 ;
			IlinesCount = 7 ;
      
			TITLE( XPSTR("AUDIO/HAPTIC") ) ;
  		lcd_puts_Pleft( y, PSTR(STR_VOLUME));
			current_volume = g_eeGeneral.volume ;
			lcd_outdezAtt( PARAM_OFS+2*FW, y, current_volume, (sub==subN) ? blink : 0 ) ;
  		if(sub==subN)
			{
				CHECK_INCDEC_H_GENVAR_0(event,current_volume,NUM_VOL_LEVELS-1);
				if ( current_volume != g_eeGeneral.volume )
				{
					setVolume( g_eeGeneral.volume = current_volume ) ;
				}
			}
  		y += FH ;
			subN += 1 ;
      
	    uint8_t b ;
  	  b = g_eeGeneral.beeperVal ;
      lcd_puts_Pleft( y,PSTR(STR_BEEPER));
      lcd_putsAttIdx(PARAM_OFS - FW - 4, y, PSTR(STR_BEEP_MODES),b,(sub==subN ? blink:0));
	    if(sub==subN) { CHECK_INCDEC_H_GENVAR_0(event, b, 6); g_eeGeneral.beeperVal = b ; }
  		y += FH ;
			subN += 1 ;

      g_eeGeneral.preBeep = onoffMenuItem( g_eeGeneral.preBeep, y, PSTR(STR_BEEP_COUNTDOWN), sub, subN, event ) ;
  		y += FH ;
			subN += 1 ;

      g_eeGeneral.minuteBeep = onoffMenuItem( g_eeGeneral.minuteBeep, y, PSTR(STR_MINUTE_BEEP), sub, subN, event ) ;
  		y += FH ;
			subN += 1 ;

	// audio start by rob
      lcd_puts_P(0, y,PSTR(STR_SPEAKER_PITCH));
      lcd_outdezAtt(PARAM_OFS+2*FW,y,g_eeGeneral.speakerPitch,(sub==subN ? blink : 0) );
      if(sub==subN) CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.speakerPitch, 1, 100) ;
  		y += FH ;
			subN += 1 ;
	 
      lcd_puts_P(0, y,PSTR(STR_HAPTICSTRENGTH));
      lcd_outdezAtt(PARAM_OFS+2*FW,y,g_eeGeneral.hapticStrength,(sub==subN ? blink : 0) );
      if(sub==subN) CHECK_INCDEC_H_GENVAR_0(event, g_eeGeneral.hapticStrength, 5) ;
  		y += FH ;
			subN += 1 ;

      lcd_puts_P(0, y,XPSTR("\001Haptic Min Run"));
      lcd_outdezAtt(PARAM_OFS+2*FW,y,g_eeGeneral.hapticMinRun+20,(sub==subN ? blink : 0) );
      if(sub==subN) CHECK_INCDEC_H_GENVAR_0(event, g_eeGeneral.hapticMinRun, 20 ) ;
  		y += FH ;
			subN += 1 ;


		}
		break ;

		case M_ALARMS	:
		{
			uint8_t y = 1*FH;
			uint8_t subN = 0 ;
#ifdef PCBSKY
#define NUM_ALARMS			6
#else
#define NUM_ALARMS			5
#endif
			IlinesCount = NUM_ALARMS ;
	    if( g_eeGeneral.frskyinternalalarm == 1)
			{
				IlinesCount += 3 ;
			}

  		uint8_t attr ;
			TITLE( XPSTR("ALARMS") ) ;
			
			if ( sub < NUM_ALARMS-1 )
			{
				displayNext() ;
  			attr = LEFT ;
      	lcd_puts_Pleft( y,PSTR(STR_BATT_WARN));
      	if(sub==subN) { attr = blink | LEFT ; CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.vBatWarn, 40, 120); } //5-10V
      	putsVolts(PARAM_OFS, y, g_eeGeneral.vBatWarn, attr);
  			y += FH ;
				subN += 1 ;

  			attr = 0 ;//LEFT ;
      	lcd_puts_Pleft( y,PSTR(STR_INACT_ALARM));
      	if(sub==subN) { attr = blink ; CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.inactivityTimer, -10, 110); } //0..120minutes
      	lcd_outdezAtt(PARAM_OFS+2*FW-2, y, g_eeGeneral.inactivityTimer+10, attr);
  			y += FH ;
				subN += 1 ;

#ifdef PCBSKY
  			lcd_puts_Pleft( y, PSTR(STR_CAPACITY_ALARM));
				lcd_outdezAtt( PARAM_OFS+2*FW, y, g_eeGeneral.mAh_alarm*50, (sub==subN) ? blink : 0 ) ;
  			if(sub==subN)
				{
					CHECK_INCDEC_H_GENVAR_0(event,g_eeGeneral.mAh_alarm,100);
				}
 				y += FH ;
				subN += 1 ;
#endif
		
      	uint8_t b = 1-g_eeGeneral.disableThrottleWarning;
      	g_eeGeneral.disableThrottleWarning = 1-onoffMenuItem( b, y, PSTR(STR_THR_WARNING), sub, subN, event ) ;
 				y += FH ;
				subN += 1 ;

      	b = 1-g_eeGeneral.disableAlarmWarning;
      	g_eeGeneral.disableAlarmWarning = 1-onoffMenuItem( b, y, PSTR(STR_ALARM_WARN), sub, subN, event ) ;
 				y += FH ;
				subN += 1 ;
			}
			else
			{
				subN = NUM_ALARMS-1 ;
      	g_eeGeneral.frskyinternalalarm = onoffMenuItem( g_eeGeneral.frskyinternalalarm, y, PSTR(STR_INT_FRSKY_ALRM), sub, subN, event ) ;
 				y += FH ;
				subN += 1 ;
				    
	    	if( g_eeGeneral.frskyinternalalarm == 1)
				{
					for ( uint8_t i = 0 ; i < 3 ; i += 1 )
					{
						uint8_t b ;
					
						b = g_eeGeneral.FRSkyYellow ;    // Done here to stop a compiler warning
								
						if ( i == 0 )
						{
						  lcd_puts_P(0, y,PSTR(STR_ALERT_YEL));
						}
						else if ( i == 1 )
						{
						  b = g_eeGeneral.FRSkyOrange ;
						  lcd_puts_P(0, y,PSTR(STR_ALERT_ORG));
						}
						else if ( i == 2 )
						{
						  b = g_eeGeneral.FRSkyRed ;
						  lcd_puts_P(0, y,PSTR(STR_ALERT_RED));
						}
						if((g_eeGeneral.speakerMode & 1) == 1)
						{
							lcd_putsAttIdx(PARAM_OFS - FW - 4, y, PSTR(STR_SOUNDS),b,(sub==subN ? blink:0));
						}
						if(sub==subN)
						{
							CHECK_INCDEC_H_GENVAR_0(event, b, 15);
							if ( i == 0 )
							{
								g_eeGeneral.FRSkyYellow = b ;
							}
							else if ( i == 1 )
							{
								g_eeGeneral.FRSkyOrange = b ;
							}
							else if ( i == 2 )
							{
								g_eeGeneral.FRSkyRed = b ;
							}
							if (checkIncDec_Ret)
							{
								audio.event(b);
							}
						}
				 		y += FH ;
						subN += 1 ;
					}
				}			  
			}
		}
		break ;

		case M_GENERAL :
		{
			uint8_t y = 1*FH;
			uint8_t subN = 0 ;
#ifdef PCBSKY
			IlinesCount = 6 ;
#else
			IlinesCount = 5 ;
#endif
			TITLE( XPSTR("GENERAL") ) ;

			if ( sub == 0 )
			{
				Columns = sizeof(g_eeGeneral.ownerName)-1 ;
			}
			editName( 11*FW-2, mstate.m_posHorz, y, (uint8_t *)g_eeGeneral.ownerName, sizeof(g_eeGeneral.ownerName), (sub==subN) ? EE_GENERAL : 0, event ) ;
 			y += FH ;
			subN += 1 ;

	    lcd_puts_Pleft( y,PSTR(STR_LANGUAGE));
	    lcd_putsAttIdx( 11*FW, y, XPSTR("\012   ENGLISH  FRANCAIS   DEUTSCH NORWEGIAN   SWEDISH"),g_eeGeneral.language,(sub==subN ? blink:0));
	    if(sub==subN) CHECK_INCDEC_H_GENVAR_0(event, g_eeGeneral.language, 4 ) ;
			setLanguage() ;
 			y += FH ;
			subN += 1 ;

      uint8_t b = 1-g_eeGeneral.disableSplashScreen;
      g_eeGeneral.disableSplashScreen = 1-onoffMenuItem( b, y, PSTR(STR_SPLASH_SCREEN), sub, subN, event ) ;
 			y += FH ;
			subN += 1 ;

      b = 1-g_eeGeneral.hideNameOnSplash;
      g_eeGeneral.hideNameOnSplash = 1-onoffMenuItem( b, y, PSTR(STR_SPLASH_NAME), sub, subN, event ) ;
 			y += FH ;
			subN += 1 ;

      b = 1-g_eeGeneral.disablePotScroll ;
			b |= g_eeGeneral.stickScroll << 1 ;
      lcd_puts_Pleft( y,PSTR(STR_SCROLLING));
      lcd_putsAttIdx(PARAM_OFS-3, y, XPSTR("\005NONE POT  STICKBOTH "),b,(sub==subN ? blink:0));
      if(sub==subN) CHECK_INCDEC_H_GENVAR_0(event, b, 3 ) ;
			g_eeGeneral.stickScroll = b >> 1 ;
			g_eeGeneral.disablePotScroll = 1 - ( b & 1 ) ;
 			y += FH ;
			subN += 1 ;

#ifdef PCBSKY
  		lcd_puts_Pleft( y, PSTR(STR_BT_BAUDRATE));
  		lcd_putsAttIdx(  PARAM_OFS-4*FW, y, XPSTR("\006115200  9600 19200 57600 38400"),g_eeGeneral.bt_baudrate,(sub==subN ? BLINK:0));
  		if(sub==subN)
			{
				b = g_eeGeneral.bt_baudrate ;
				CHECK_INCDEC_H_GENVAR_0(event,g_eeGeneral.bt_baudrate,4);
				if ( b != g_eeGeneral.bt_baudrate )
				{
//					uint32_t baudrate = 115200 ;
//					if ( b )
//					{
//						baudrate = ( b == 1 ) ? 9600 : ( ( b == 2 ) ? 19200 : 57600 ) ;
//					}
					BtBaudrateChanged = 1 ;
//					UART3_Configure( baudrate, Master_frequency ) ;
				}
			}
 			y += FH ;
			subN += 1 ;
#endif

		}			 
		break ;

		case M_CONTROLS :
		{
			uint8_t y = 1*FH;
			uint8_t subN = 0 ;
			IlinesCount = 9 ;
			TITLE( XPSTR("CONTROLS") ) ;

			if ( sub < 5 )
			{
				displayNext() ;
				uint8_t attr = sub==subN ? INVERS : 0 ;
	  		lcd_puts_Pleft( y, PSTR(STR_CHAN_ORDER) ) ;//   RAET->AETR
				uint8_t bch = bchout_ar[g_eeGeneral.templateSetup] ;
    		for ( uint8_t i = 4 ; i > 0 ; i -= 1 )
				{
					uint8_t letter ;
					letter = *(PSTR(STR_SP_RETA) +(bch & 3) + 1 ) ;
  				lcd_putcAtt( (16+i)*FW, y, letter, attr ) ;
					bch >>= 2 ;
				}
	  		if(attr) CHECK_INCDEC_H_GENVAR_0( event, g_eeGeneral.templateSetup, 23 ) ;
 				y += FH ;
				subN += 1 ;
      
				g_eeGeneral.throttleReversed = onoffMenuItem( g_eeGeneral.throttleReversed, y, PSTR(STR_THR_REVERSE), sub, subN, event ) ;
 				y += FH ;
				subN += 1 ;

				g_eeGeneral.crosstrim = onoffMenuItem( g_eeGeneral.crosstrim, y, PSTR(STR_CROSSTRIM), sub, subN, event ) ;
 				y += FH ;
				subN += 1 ;

	    	lcd_puts_Pleft( y, PSTR(STR_MODE) );
    		for ( uint8_t i = 0 ; i < 4 ; i += 1 )
				{
					lcd_img((6+4*i)*FW, y, sticks, i, 0 ) ;
					if (g_eeGeneral.stickReverse & (1<<i)) lcd_char_inverse( (6+4*i)*FW, y, 3*FW, 0 ) ;
				}
    		if(sub==subN)
				{
					CHECK_INCDEC_H_GENVAR_0( event, g_eeGeneral.stickReverse, 15 ) ;
					plotType = PLOT_BLACK ;
					lcd_rect( 6*FW-1, y-1, 15*FW+2, 9 ) ;
					plotType = PLOT_XOR ;
				}
 				y += FH ;
				subN += 1 ;
    
 				attr = 0 ;
				uint8_t mode = g_eeGeneral.stickMode ;
      	if(sub==subN)
				{
					attr = INVERS ;
					if ( s_editMode )
					{
					 	attr = BLINK ;
						
						CHECK_INCDEC_H_GENVAR_0(event, mode,3);
						if ( mode != g_eeGeneral.stickMode )
						{
							g_eeGeneral.stickScroll = 0 ;
							g_eeGeneral.stickMode = mode ;							
						}
					}
				}
      	lcd_putcAtt( 3*FW, y, '1'+g_eeGeneral.stickMode,attr);
	#ifdef FIX_MODE
      	for(uint8_t i=0; i<4; i++) putsChnRaw( (6+4*i)*FW, y, modeFixValue( i ), 0 ) ;//sub==3?INVERS:0);
	#else
      	for(uint8_t i=0; i<4; i++) putsChnRaw( (6+4*i)*FW, y,i+1,0);//sub==3?INVERS:0);
	#endif
 				y += FH ;
				subN += 1 ;
			}
			else
			{
				subN = 5 ;
				if ( sub >= IlinesCount )
				{
					sub = mstate.m_posVert = IlinesCount-1 ;
				}
				// Edit custom stick names
				Columns = 3 ;
      	for(uint8_t i=0; i<4; i++)
				{
      		lcd_putsAttIdx( FW*5, y, PSTR(STR_STICK_NAMES), i, 0 ) ;
					editName( 11*FW, mstate.m_posHorz, y, &g_eeGeneral.customStickNames[i*4], 4, sub==subN ? EE_GENERAL : 0, event ) ;
	 				y += FH ;
					subN += 1 ;
				}
			}
		}			 
		break ;


		case M_HARDWARE :
		{
			uint8_t y = 1*FH;
			uint8_t subN = 0 ;
#ifdef PCBSKY
			uint8_t sm = g_eeGeneral.switchMapping ;
			uint8_t num = 7 + 2 ;
			if ( sm & ( USE_ELE_3POS | USE_ELE_6POS ) )
			{
				num += 2 ;
				if ( sm & ( USE_THR_3POS | USE_RUD_3POS ) )
				{
					num -= 1 ;
				}
			}	 
			if ( sm & ( USE_AIL_3POS | USE_GEA_3POS ) )
			{
				num -= 1 ;
			}
			IlinesCount = num ;
#else
			IlinesCount = 2 ;
#endif
			TITLE( XPSTR("HARDWARE") ) ;
#ifdef PCBSKY
			if ( sub < 6 )
			{
				displayNext() ;
#endif
        lcd_puts_Pleft( y,PSTR(STR_FILTER_ADC));
        lcd_putsAttIdx(PARAM_OFS, y, XPSTR("\004SINGOSMPFILT"),g_eeGeneral.filterInput,(sub==subN ? INVERS:0));
        if(sub==subN) CHECK_INCDEC_H_GENVAR_0(event, g_eeGeneral.filterInput, 2);
 				y += FH ;
				subN += 1 ;

  			lcd_puts_Pleft( y, PSTR(STR_ROTARY_DIVISOR));
	  		lcd_putsAttIdx( 15*FW, y, XPSTR("\001142"),g_eeGeneral.rotaryDivisor,(sub==subN ? BLINK:0));
  			if(sub==subN)
				{
					CHECK_INCDEC_H_GENVAR_0(event,g_eeGeneral.rotaryDivisor,2);
				}
 				y += FH ;
				subN += 1 ;

#ifdef PCBSKY
				uint8_t lastValue ;
				uint8_t b ;
				lastValue = g_eeGeneral.stickGain ;
  			lcd_puts_Pleft( y, PSTR(STR_STICK_LV_GAIN));
				b = g_eeGeneral.stickGain & STICK_LV_GAIN ? 1 : 0 ;
    		lcd_putcAtt( 15*FW, y, b ? '2' : '1', sub==subN ? BLINK:0 );
  			if(sub==subN)
				{
					CHECK_INCDEC_H_GENVAR_0(event, b,1);
					g_eeGeneral.stickGain = (lastValue & ~STICK_LV_GAIN) | (b ? STICK_LV_GAIN : 0 ) ;
				}
 				y += FH ;
				subN += 1 ;
  		
				lcd_puts_Pleft( y, PSTR(STR_STICK_LH_GAIN));
				b = g_eeGeneral.stickGain & STICK_LH_GAIN ? 1 : 0 ;
    		lcd_putcAtt( 15*FW, y, b ? '2' : '1', sub==subN ? BLINK:0 );
  			if(sub==subN)
				{
					CHECK_INCDEC_H_GENVAR_0(event, b,1);
					g_eeGeneral.stickGain = (lastValue & ~STICK_LH_GAIN) | (b ? STICK_LH_GAIN : 0 ) ;
				}
 				y += FH ;
				subN += 1 ;

  			lcd_puts_Pleft( y, PSTR(STR_STICK_RV_GAIN));
				b = g_eeGeneral.stickGain & STICK_RV_GAIN ? 1 : 0 ;
    		lcd_putcAtt( 15*FW, y, b ? '2' : '1', sub==subN ? BLINK:0 );
  			if(sub==subN)
				{
					CHECK_INCDEC_H_GENVAR_0(event, b,1);
					g_eeGeneral.stickGain = (lastValue & ~STICK_RV_GAIN) | (b ? STICK_RV_GAIN : 0 ) ;
				}
 				y += FH ;
				subN += 1 ;

  			lcd_puts_Pleft( y, PSTR(STR_STICK_RH_GAIN));
				b = g_eeGeneral.stickGain & STICK_RH_GAIN ? 1 : 0 ;
    		lcd_putcAtt( 15*FW, y, b ? '2' : '1', sub==subN ? BLINK:0 );
  			if(sub==subN)
				{
					CHECK_INCDEC_H_GENVAR_0(event, b,1);
					g_eeGeneral.stickGain = (lastValue & ~STICK_RH_GAIN) | (b ? STICK_RH_GAIN : 0 ) ;
				}
				if ( lastValue != g_eeGeneral.stickGain )
				{
					set_stick_gain( g_eeGeneral.stickGain ) ;
				}
			}
#endif // PCBSKY

#ifdef PCBSKY
			else
			{
				subN = 6 ;
				if ( sub >= IlinesCount )
				{
					sub = mstate.m_posVert = IlinesCount-1 ;
				}
  			lcd_puts_Pleft( y, XPSTR("ELE  switch"));
//		lcd_outhex4( 18*FW, y, Analog_values[9] ) ;	// Debug
				uint8_t type = 0 ;
				if ( sm & USE_ELE_3POS )
				{
					type = 1 ;
				}
				if ( sm & USE_ELE_6POS )
				{
					type = ( sm & USE_ELE_6PSB ) ? 3 : 2 ;
				}
  			lcd_putsAttIdx(  12*FW, y, XPSTR("\0052POS 3POS 6POSA6POSB"), type, (sub==subN ? BLINK:0));
  			if(sub==subN)
				{
					CHECK_INCDEC_H_GENVAR_0(event, type, 3 ) ;
					g_eeGeneral.switchMapping = g_eeGeneral.switchMapping & ~(USE_ELE_3POS | USE_ELE_6POS | USE_ELE_6PSB ) ;
					if ( type == 0 )
					{
						g_eeGeneral.switchMapping = g_eeGeneral.switchMapping & ~(USE_THR_3POS | USE_RUD_3POS) ;
				
					}
					else if ( type == 1 )
					{
						g_eeGeneral.switchMapping |= USE_ELE_3POS ;
					}
					else
					{
						g_eeGeneral.switchMapping |= USE_ELE_6POS ;
						if ( type == 3 )
						{
							g_eeGeneral.switchMapping |= USE_ELE_6PSB ;
						}
					}
					if ( sm != g_eeGeneral.switchMapping )
					{
						createSwitchMapping() ;
					}
				}
 				y += FH ;
				subN++;

				if ( sm & ( USE_ELE_3POS | USE_ELE_6POS ) )
				{
					if ( ( sm & USE_RUD_3POS ) == 0 )
					{
						type = (sm & USE_THR_3POS )!=0 ;
				
				  	lcd_puts_Pleft( y, XPSTR("THR  switch"));
				  	lcd_putsAttIdx(  12*FW, y, XPSTR("\0042POS3POS"), type, (sub==subN ? BLINK:0));
				  	if(sub==subN)
						{
							CHECK_INCDEC_H_GENVAR_0(event, type, 1 ) ;
							g_eeGeneral.switchMapping = g_eeGeneral.switchMapping & ~(USE_THR_3POS) ;
							if ( type )
							{
								g_eeGeneral.switchMapping |= USE_THR_3POS ;
							}
						}
						if ( sm != g_eeGeneral.switchMapping )
						{
							createSwitchMapping() ;
						}
		 				y += FH ;
						subN++;
					}
					if ( ( sm & USE_THR_3POS ) == 0 )
					{
						type = (sm & USE_RUD_3POS )!=0 ;
				
				  	lcd_puts_Pleft( y, XPSTR("RUD  switch"));
				  	lcd_putsAttIdx(  12*FW, y, XPSTR("\0042POS3POS"), type, (sub==subN ? BLINK:0));
				  	if(sub==subN)
						{
							CHECK_INCDEC_H_GENVAR_0(event, type, 1 ) ;
							g_eeGeneral.switchMapping = g_eeGeneral.switchMapping & ~(USE_RUD_3POS) ;
							if ( type )
							{
								g_eeGeneral.switchMapping |= USE_RUD_3POS ;
							}
						}
						if ( sm != g_eeGeneral.switchMapping )
						{
							createSwitchMapping() ;
						}
 						y += FH ;
						subN++;
					}
				}

				if ( ( sm & USE_GEA_3POS ) == 0 )
				{
					type = (sm & USE_AIL_3POS )!=0 ;
				
				  lcd_puts_Pleft( y, XPSTR("AIL  switch"));
				  lcd_putsAttIdx(  12*FW, y, XPSTR("\0042POS3POS"), type, (sub==subN ? BLINK:0));
				  if(sub==subN)
					{
						CHECK_INCDEC_H_GENVAR_0(event, type, 1 ) ;
						g_eeGeneral.switchMapping = g_eeGeneral.switchMapping & ~(USE_AIL_3POS) ;
						if ( type )
						{
							g_eeGeneral.switchMapping |= USE_AIL_3POS ;
						}
					}
					if ( sm != g_eeGeneral.switchMapping )
					{
						createSwitchMapping() ;
					}
	 				y += FH ;
					subN++;
				}
				if ( ( sm & USE_AIL_3POS ) == 0 )
				{
					type = (sm & USE_GEA_3POS )!=0 ;
				
				  lcd_puts_Pleft( y, XPSTR("GEAR switch"));
				  lcd_putsAttIdx(  12*FW, y, XPSTR("\0042POS3POS"), type, (sub==subN ? BLINK:0));
				  if(sub==subN)
					{
						CHECK_INCDEC_H_GENVAR_0(event, type, 1 ) ;
						g_eeGeneral.switchMapping = g_eeGeneral.switchMapping & ~(USE_GEA_3POS) ;
						if ( type )
						{
							g_eeGeneral.switchMapping |= USE_GEA_3POS ;
						}
					}
					if ( sm != g_eeGeneral.switchMapping )
					{
						createSwitchMapping() ;
					}
 					y += FH ;
					subN++;
				}
			}
#endif // PCBSKY
		}			 
		break ;

	}
}

const char Str_Mixer[] =      "Mixer" ;
const char Str_Cswitches[] =  "L.Switches" ;
const char Str_Telemetry[] =  "Telemetry" ;
const char Str_limits[] =     "Limits" ;
const char Str_Bluetooth[] =  "BlueTooth" ;
const char Str_heli_setup[] = "Heli" ;
const char Str_Curves[] =     "Curves" ;
const char Str_Expo[] =		    "Expo/Dr" ;
const char Str_Globals[] =    "Globals" ;
const char Str_Timer[] =      "Timers" ;
const char Str_Modes[] =	 		"Modes" ;
const char Str_Voice[] =      "Voice" ;
const char Str_Protocol[] =   "Protocol" ;
const char Str_Safety[] =			"Safety Sws" ;

#define M_MINDEX			0
#define M_MIXER				1
#define M_HELI				2
#define M_LIMITS			3
#define M_EXPO				4
#define M_MODES				5
#define M_CURVE				6
#define M_SWITCHES		7
#define M_BLUETOOTH		8
#define M_SAFETY			9
#define M_GLOBALS			10
#define M_TELEMETRY		11
#define M_VOICE				12
#define M_TIMERS			13
#define M_MGENERAL		14
#define M_PROTOCOL		15

void menuProcModelIndex(uint8_t event)
{
	static MState2 mstate ;
	static uint8_t Columns = 0 ;
//	uint8_t extra = 8 ;

//	if (event == EVT_ENTRY)
//	{
//		mstate.m_posVert = SubmenuIndex ;
//		SubmenuIndex = 0 ;
////		SubMenuFromIndex = 0 ;
//	}
//	if (event == EVT_ENTRY_UP)
//	{
//		mstate.m_posVert = SubmenuIndex - 1 ;
//		if ( SubMenuCall & 0x80 )
//		{
//			SubMenuCall &= 0x7F ;
//		}
//		else
//		{
//			SubmenuIndex = 0 ;
////			SubMenuFromIndex = 0 ;
//		}
//	}

//	if ( UseLastSubmenuIndex )
//	{
//		SubmenuIndex = LastSubmenuIndex & 0x7F ;
//		UseLastSubmenuIndex = 0 ;
//	}
	
//	if ( SubmenuIndex )
//	{
//  	if (event == EVT_KEY_LONG(KEY_EXIT) )
//		{
//      s_editMode = false;
//			mstate.m_posVert = SubmenuIndex ;
//			SubmenuIndex = 0 ;
//			killEvents(event) ;
//			event = 0 ;
//		}
////		uint8_t saveEvent = event ;
//  	if (event == EVT_KEY_FIRST(KEY_EXIT) )
//		{
//			event = 0 ;
//		}
//  	if ( (event == EVT_KEY_BREAK(KEY_EXIT) ) || (event == EVT_KEY_LONG(BTN_RE) ) )
//		{
//      if(s_editMode)
//			{
//        s_editMode = false;
//			}
//			else
//			{
//				mstate.m_posVert = SubmenuIndex ;
//				SubmenuIndex = 0 ;
//				killEvents(event) ;
//			}
//			event = 0 ;
//		}
////  	if ( (event == EVT_KEY_BREAK(BTN_RE)) || ( event == EVT_KEY_FIRST(KEY_MENU) ) )
////    {
////			s_editMode = !s_editMode;
////		}
//	}
//	else
//	{
////		if ( mstate.m_posVert )
////		{
//			if (event == EVT_KEY_FIRST(KEY_RIGHT) )
//			{
////				if ( mstate.m_posVert && mstate.m_posVert < 8)
//				if ( mstate.m_posVert < 7 )
//				{
//					mstate.m_posVert += (extra == 8) ? 8 : 7 ;
//				}
//			}
//			if (event == EVT_KEY_FIRST(KEY_LEFT) )
//			{
//				if ( mstate.m_posVert >= ( (extra == 8) ? 8 : 7) )
//				{
//					mstate.m_posVert -= (extra == 8) ? 8 : 7 ;
//				}
//			}
	 
//  		if ( (event == EVT_KEY_FIRST(KEY_MENU) ) ||(event == EVT_KEY_BREAK(BTN_RE) ) )
//			{
//				SubmenuIndex = mstate.m_posVert + 1 ;
//				LastSubmenuIndex = SubmenuIndex | 0x80 ;
//				mstate.m_posVert = 0 ;
//				killEvents(event) ;
//				event = 0 ;
////				if ( SubmenuIndex > 6 )
////				{
////					SubMenuFromIndex = 0 ;
////				}
//			}
////		}
////		event = mstate.check( event, 0, NULL, 0, &Columns, 0, IlinesCount-1 ) ;
////		mstate.check_simple(event, e_mIndex, menuTabModel, DIM(menuTabModel), IlinesCount-1 ) ;
//	}
	event = indexProcess( event, &mstate, 8 ) ;
	event = mstate.check( event, 0, NULL, 0, &Columns, 0, IlinesCount-1 ) ;
					
	SubMenuFromIndex = 1 ;
	switch ( SubmenuIndex )
	{
		case M_MIXER :
      pushMenu(menuProcMix) ;
		break ;
		case M_SWITCHES :
      pushMenu(menuProcSwitches) ;
		break ;
		case M_TELEMETRY :
      pushMenu(menuProcTelemetry) ;
		break ;
		case M_LIMITS :
      pushMenu(menuProcLimits) ;
		break ;
		case M_HELI :
      pushMenu(menuProcHeli) ;
		break ;
		case M_VOICE :
      pushMenu(menuProcVoiceAlarm) ;
		break ;
		case M_CURVE :
      pushMenu(menuProcCurve) ;
		break ;
		case M_EXPO :
      pushMenu(menuProcExpoAll) ;
		break ;
		case M_MODES :
      pushMenu(menuModelPhases) ;
		break ;
		case M_GLOBALS :
      pushMenu(menuProcGlobals) ;
		break ;
		case M_PROTOCOL :
      pushMenu(menuProcModel) ;
		break ;
		case M_SAFETY :
      pushMenu(menuProcSafetySwitches) ;
		break ;
		default :
			SubMenuFromIndex = 0 ;
			if ( event == EVT_ENTRY )
			{
				audioDefevent(AU_MENUS) ;
				event = 0 ;
			}
		break ;
	}

	uint32_t sub = mstate.m_posVert ;
	uint32_t y ;
	Columns = 0 ;
//	uint8_t blink = InverseBlink ;

	switch ( SubmenuIndex )
	{
		case M_MINDEX :
  		TITLE(XPSTR("MODEL SETUP")) ;
			IlinesCount = 15 ;
			sub += 1 ;

static const char *const n_Strings[] = {
Str_Mixer,
Str_heli_setup,
Str_limits,
Str_Expo,
Str_Modes,
Str_Curves,
Str_Cswitches,
Str_Bluetooth,
Str_Safety,
Str_Globals,
Str_Telemetry,
Str_Voice,
Str_Timer,
Str_General,
Str_Protocol
// Need safety
};
			
		displayIndex( n_Strings, 8, 7, sub ) ;

		break ;

		case M_TIMERS :
		{	
			uint8_t subN = 0 ;
			uint32_t t ;
			div_t qr ;
			uint8_t attr = sub==subN ? INVERS : 0 ;
      TITLE( XPSTR("Timers") ) ;
			IlinesCount = 14 ;
			
			lcd_puts_P( 20*FW-4, 7*FH, XPSTR("->") ) ;
			
			if ( sub < 14 )
			{
				editTimer( sub, event ) ;
			}
			else
			{
				subN = 14 ;
				y = FH ;
				lcd_puts_Pleft( y,XPSTR("Total Time"));
				t = g_model.totalTime / 60 ;	// minutes
				qr = div( t, 60 ) ;
				lcd_outdezNAtt( 20*FW, y, (uint16_t)qr.rem, LEADING0|attr, 2 ) ;
				lcd_putcAtt(17*FW, y, ':', attr ) ;
				lcd_outdezAtt( 17*FW, y, (uint16_t)qr.quot, attr ) ;

      	if(sub==subN)
				{
					if (event == EVT_KEY_LONG(KEY_MENU) )
					{
						g_model.totalTime = 0 ;
					}
				}
			}
			
		}
		break ;

		case M_MGENERAL :
		{	
			uint8_t subN = 0 ;
			uint8_t attr = 0 ;
			
      TITLE( XPSTR("General") ) ;
			IlinesCount = 16 ;
			y = FH ;

			if ( SubMenuCall )
			{
				mstate.m_posVert = SubMenuCall ;
				SubMenuCall = 0 ;
				if ( FileSelectResult == 1 )
				{
					copyFileName( g_model.modelVname, SelectedVoiceFileName ) ;
					g_model.modelVoice = -1 ;
	  		  eeDirty(EE_MODEL) ;		// Save it
				}
			}

			if ( sub < 11 )
			{
				displayNext() ;
			}
			if ( sub < 6 )
			{
				if(sub==subN)
				{
					Columns = sizeof(g_model.name)-1 ;
				}
				editName( 11*FW-2, mstate.m_posHorz, y, (uint8_t *)g_model.name, sizeof(g_model.name), sub==subN ? EE_MODEL : 0, event ) ;
				y += FH ;
				subN += 1 ;

    		if(sub==subN)
				{
					if (event == EVT_KEY_LONG(KEY_MENU) )
					{
						SubMenuCall = 0x80 + mstate.m_posVert ;
    		   	pushMenu( menuProcSelectMvoiceFile ) ;				
  			  	s_editMode = 0 ;
    		    killEvents(event);
					}
					if ( mstate.m_posHorz == 0 )
					{
						attr = InverseBlink ;
  			    CHECK_INCDEC_H_MODELVAR( event, g_model.modelVoice, -1, 49 ) ;
					}
				}
				if ( g_model.modelVoice == -1 )
				{
					if(sub==subN)
					{
						Columns = VOICE_NAME_SIZE ;
					}
					uint8_t type = ( (sub==subN) && (mstate.m_posHorz) ) ? EE_MODEL : 0 ;
					editName( 9*FW-2, mstate.m_posHorz-1, y, (uint8_t *)g_model.modelVname, sizeof(g_model.modelVname), type, event ) ;
				}
				else
				{
					if (event == EVT_KEY_BREAK(KEY_MENU) )
					{
						if(sub==subN)
						{
							putVoiceQueue( g_model.modelVoice + 260 ) ;
							s_editMode = 0 ;
						}
					}
				}
    		lcd_puts_Pleft( y, PSTR(STR_VOICE_INDEX) ) ;
				if ( g_model.modelVoice == -1 )
				{
    			lcd_putcAtt( 7*FW+2, y, '>', attr) ;
				}
				else
				{
    			lcd_outdezAtt(  10*FW+2, y, (int16_t)g_model.modelVoice + 260 ,attr);
				}
				y += FH ;
				subN += 1 ;

				g_model.thrTrim = onoffMenuItem_m( event, g_model.thrTrim, y, PSTR(STR_T_TRIM), sub==subN) ;
				y += FH ;
				subN += 1 ;
			
				g_model.thrExpo = onoffMenuItem_m( event, g_model.thrExpo, y, PSTR(STR_T_EXPO), sub==subN) ;
				y += FH ;
				subN += 1 ;

				lcd_puts_Pleft( y,PSTR(STR_LIGHT_SWITCH));
  			attr = 0 ;
  			if(sub==subN) { attr = InverseBlink ; }
				g_model.mlightSw = edit_dr_switch( 17*FW, y, g_model.mlightSw, attr, attr ? EDIT_DR_SWITCH_EDIT : 0, event ) ;
				y += FH ;
				subN += 1 ;

				g_model.useCustomStickNames = onoffMenuItem_m( event, g_model.useCustomStickNames, y, XPSTR("CustomStkNames"), sub==subN) ;
			}
			else if ( sub < 11 )
			{
				subN = 6 ;
				y = FH ;
  			
				lcd_puts_Pleft(    y, PSTR(STR_TRIM_INC));
  			attr = 0 ;
				if(sub==subN)
				{
  			 	attr = InverseBlink ;
  			  CHECK_INCDEC_H_MODELVAR_0(event,g_model.trimInc,4) ;
				}
  			lcd_putsAttIdx(  14*FW, y, PSTR(STR_TRIM_OPTIONS),g_model.trimInc, attr);
				y += FH ;
				subN += 1 ;

  			lcd_puts_Pleft(    y, PSTR(STR_TRIM_SWITCH));
  			attr = 0 ;
  			if(sub==subN) { attr = InverseBlink ; }
				g_model.trimSw = edit_dr_switch( 17*FW, y, g_model.trimSw, attr, attr ? EDIT_DR_SWITCH_EDIT : 0, event ) ;
				y += FH ;
				subN += 1 ;
	
				g_model.extendedLimits = onoffMenuItem_m( event, g_model.extendedLimits, y, PSTR(STR_E_LIMITS), sub==subN) ;
				y += FH ;
				subN += 1 ;

				g_model.traineron = onoffMenuItem_m( event, g_model.traineron, y, PSTR(STR_Trainer), sub==subN) ;
				y += FH ;
				subN += 1 ;

  			attr = PREC1 ;
  			lcd_puts_Pleft(    y, PSTR(STR_AUTO_LIMITS));
  			if(sub==subN) { attr = InverseBlink | PREC1 ; CHECK_INCDEC_H_MODELVAR_0( event, g_model.sub_trim_limit, 100 ) ; }
  			lcd_outdezAtt(  20*FW, y, g_model.sub_trim_limit, attr ) ;
			}
			else
			{
				subN = 11 ;
    		
#ifdef REV9E
				uint32_t states = g_model.modelswitchWarningStates ;
#else				
				uint16_t states = g_model.modelswitchWarningStates ;
#endif
    		uint8_t b = 1 - (states & 1) ;
				b = 1 - onoffMenuItem_m( event, b, y, PSTR(STR_SWITCH_WARN), sub==subN ) ;
    		g_model.modelswitchWarningStates = (states & ~1) | b ;
				y += FH ;
				subN += 1 ;

		    lcd_puts_Pleft(    y, PSTR(STR_DEAFULT_SW)) ;
		 		states = g_model.modelswitchWarningStates >> 1 ;
#ifdef REV9E
				states |= (uint32_t)g_model.xmodelswitchWarningStates << 14 ;
#endif
        
#ifdef PCBSKY
				y += FH ;
				uint16_t index ;
						
				index = oneSwitchText( HSW_ThrCt, states ) ;
				attr = index >> 8 ;
				index &= 0x00FF ;
				lcd_putsAttIdx(0*FW+5, y, PSTR(SWITCHES_STR), index, attr ) ;
		
				index = oneSwitchText( HSW_RuddDR, states ) ;
				attr = index >> 8 ;
				index &= 0x00FF ;
				lcd_putsAttIdx(3*FW+7, y, PSTR(SWITCHES_STR), index, attr ) ;
		
				index = oneSwitchText( HSW_ElevDR, states ) ;
				attr = index >> 8 ;
				index &= 0x00FF ;
				lcd_putsAttIdx(6*FW+9, y, PSTR(SWITCHES_STR), index, attr ) ;

				index = oneSwitchText( HSW_ID0, states ) & 0x00FF ;
				lcd_putsAttIdx(9*FW+11, y, PSTR(SWITCHES_STR), index, 0 ) ;

				index = oneSwitchText( HSW_AileDR, states ) ;
				attr = index >> 8 ;
				index &= 0x00FF ;
				lcd_putsAttIdx(12*FW+13, y, PSTR(SWITCHES_STR), index, attr ) ;
		
				index = oneSwitchText( HSW_Gear, states ) ;
				attr = index >> 8 ;
				index &= 0x00FF ;
				lcd_putsAttIdx(15*FW+15, y, PSTR(SWITCHES_STR), index, attr ) ;
		    
				if(sub==subN)
				{
					plotType = PLOT_BLACK ;
					lcd_rect( 0*FW+4-1, y-1, 18*FW+2+12, 9 ) ;
					plotType = PLOT_XOR ;
#endif
#ifdef PCBX9D
#ifdef REV9E
    		for (uint8_t i=0 ; i<5 ; i += 1 )
				{
    		  lcd_putc( 11*FW+i*(2*FW), y, 'A'+i ) ;
					lcd_putc( 12*FW+i*(2*FW), y, PSTR(HW_SWITCHARROW_STR)[states & 0x03] ) ;
    		  states >>= 2 ;
    		}
				y += FH ;
    		
    		for (uint8_t i=0 ; i<10 ; i += 1 )
				{
    		  lcd_putc( 1+0*FW+i*(2*FW), y, 'A'+5+i ) ;
					lcd_putc( 1+1*FW+i*(2*FW), y, PSTR(HW_SWITCHARROW_STR)[states & 0x03] ) ;
    		  states >>= 2 ;
    		}
				if(sub==subN)
				{
					lcd_rect( 11*FW-1, y-1-FH, 10*FW+2, 9 ) ;
					lcd_rect( 0*FW, y-1, 20*FW+2, 9 ) ;
#else
				y += FH ;
    		for (uint8_t i=0 ; i<7 ; i += 1 )
				{
    		  lcd_putc( 2*FW+i*(2*FW+3), y, 'A'+i ) ;
					lcd_putc( 3*FW+i*(2*FW+3), y, PSTR(HW_SWITCHARROW_STR)[states & 0x03] ) ;
    		  states >>= 2 ;
    		}
    		if(sub==subN)
				{
					lcd_rect( 2*FW-2, y-1, 14*FW+2+18+1, 9 ) ;
#endif
#endif
		      if (event==EVT_KEY_FIRST(KEY_MENU) || event==EVT_KEY_FIRST(BTN_RE))
					{
    		    killEvents(event);
//    		uint16_t states = g_model.modelswitchWarningStates & 1 ;
#ifdef PCBSKY
			      g_model.modelswitchWarningStates = (getCurrentSwitchStates() << 1 ) ;// states ;
#endif
#ifdef PCBX9D
			  		getMovedSwitch() ;	// loads switches_states
#ifdef REV9E
extern uint32_t switches_states ;
				    g_model.modelswitchWarningStates = ((switches_states & 0x3FFF) << 1 ) ;// states ;
				    g_model.xmodelswitchWarningStates = (switches_states >> 14 ) ;// states ;
#else
extern uint16_t switches_states ;
				    g_model.modelswitchWarningStates = ((switches_states & 0x3FFF) << 1 ) ;// states ;
#endif
#endif
    		    s_editMode = false ;
		        STORE_MODELVARS ;
					}
				}
				y += FH ;
				subN += 1 ;

  			attr = 0 ;
    		lcd_puts_Pleft( y, XPSTR("Throttle Default") ) ;
    		if(sub==subN) { attr = INVERS ; CHECK_INCDEC_H_MODELVAR_0( event, g_model.throttleIdle, 1 ) ; }
    		lcd_putsAttIdx( 16*FW, y, XPSTR("\005-100%   0%"),g_model.throttleIdle, attr ) ;
				y += FH ;
				subN += 1 ;

		  	attr = 0 ;
    		lcd_puts_Pleft(    y, PSTR(STR_VOLUME_CTRL));
		    if(sub==subN) { attr = INVERS ; CHECK_INCDEC_H_MODELVAR_0( event, g_model.anaVolume, 7 ) ; }
#ifdef PCBSKY
		    lcd_putsAttIdx( 17*FW, y, XPSTR("\003---P1 P2 P3 GV4GV5GV6GV7"),g_model.anaVolume, attr ) ;
#endif
#ifdef PCBX9D
    		lcd_putsAttIdx( 17*FW, y, XPSTR("\003---P1 P2 SL SR GV5GV6GV7"),g_model.anaVolume, attr ) ;
#endif
				y += FH ;
				subN += 1 ;
				uint8_t subSub = mstate.m_posHorz ;

		    lcd_puts_Pleft(    y, PSTR(STR_BEEP_CENTRE));
    		for(uint8_t i=0;i<7;i++) lcd_putsnAtt((10+i)*FW, y, PSTR(STR_RETA123)+i,1, (((subSub)==i) && (sub==subN)) ? BLINK : ((g_model.beepANACenter & (1<<i)) ? INVERS : 0 ) );
    		if(sub==subN)
				{
					Columns = 6 ;
        	if((event==EVT_KEY_FIRST(KEY_MENU)) || P1values.p1valdiff)
					{
            killEvents(event);
            s_editMode = false;
            g_model.beepANACenter ^= (1<<(subSub));
            STORE_MODELVARS;
//            eeWaitComplete() ;
	        }
  		  }
			}
		}	
		break ;

#define BT_COMMAND_DEBUG	1

		case M_BLUETOOTH :
		{	
			uint8_t subN = 0 ;
			
      TITLE( XPSTR("BlueTooth") ) ;
#ifdef PCBSKY
 #ifdef BT_COMMAND_DEBUG
			IlinesCount = 3 ;
 #else
			IlinesCount = 2 ;
 #endif
#else
			IlinesCount = 0 ;
#endif
			y = FH ;
      
			lcd_puts_Pleft( y,XPSTR("BT Type"));
  		lcd_putsAttIdx( 16*FW, y, XPSTR("\005HC-06HC-05"), g_eeGeneral.BtType, sub==subN ? INVERS : 0 ) ;
			if(sub==subN) CHECK_INCDEC_H_GENVAR_0( event, g_eeGeneral.BtType, 1 ) ;
			y += FH ;
			subN += 1 ;

#ifdef PCBSKY
			lcd_puts_Pleft( y, XPSTR("BT as Trainer") );
			if ( (sub == subN) )
			{
				CHECK_INCDEC_H_MODELVAR_0(event, g_model.BTfunction, 1 ) ;
			}
  		menu_lcd_onoff( PARAM_OFS, y, g_model.BTfunction, sub==subN ) ;
			y += FH ;
			subN += 1 ;

 #ifdef BT_COMMAND_DEBUG

static uint8_t BtCommandMode ;
#define BT_TYPE_HC06		0
#define BT_TYPE_HC05		1

			lcd_puts_Pleft( y, XPSTR("BT Command Mode") ) ;
			if ( (sub == subN) )
			{
				BtCommandMode = checkIncDec( event, BtCommandMode, 0, 1, 0 ) ;
			}
  		menu_lcd_onoff( PARAM_OFS, y, BtCommandMode, sub==subN ) ;

			if ( g_eeGeneral.BtType == BT_TYPE_HC05 )
			{
				if ( BtCommandMode )
				{
					PIOB->PIO_SODR = PIO_PB12 ;		// Set bit B12 HIGH
				}
				else
				{
					PIOB->PIO_CODR = PIO_PB12 ;		// Set bit B12 LOW
				}
			}
			y += FH ;
			subN += 1 ;
 #endif

#endif

		}	
		break ;

	}	
}

extern uint16_t SplashDebug[9] ;

void menuProcBoot(uint8_t event)
{
  MENU(PSTR(STR_BOOT_REASON), menuTabStat, e_Boot, 1, {0/*, 0*/});

#ifdef PCBSKY
	if ( ( ResetReason & RSTC_SR_RSTTYP ) == (2 << 8) )	// Watchdog
	{
		lcd_puts_Pleft( 2*FH, PSTR(STR_6_WATCHDOG) ) ;
	}
	else if ( unexpectedShutdown )
	{
		lcd_puts_Pleft( 2*FH, PSTR(STR_5_UNEXPECTED) ) ;
		lcd_puts_Pleft( 3*FH, PSTR(STR_6_SHUTDOWN) ) ;
	}
	else
	{
		lcd_puts_Pleft( 2*FH, PSTR(STR_6_POWER_ON) ) ;
	}
#endif
  lcd_outdez( 0, 5*FH, ( ResetReason >> 8 ) & 7 ) ;

	lcd_outhex4( 0, 4*FH, SplashDebug[8] ) ;
	lcd_outhex4( 0, 6*FH, SplashDebug[0] ) ;
	lcd_outhex4( 30, 6*FH, SplashDebug[1] ) ;
	lcd_outhex4( 60, 6*FH, SplashDebug[2] ) ;
	lcd_outhex4( 90, 6*FH, SplashDebug[3] ) ;

	lcd_outhex4( 0, 7*FH, SplashDebug[4] ) ;
	lcd_outhex4( 30, 7*FH, SplashDebug[5] ) ;
	lcd_outhex4( 60, 7*FH, SplashDebug[6] ) ;
	lcd_outhex4( 90, 7*FH, SplashDebug[7] ) ;


}



