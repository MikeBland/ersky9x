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

#include "ersky9x.h"
#include "sound.h"
#include "lcd.h"
#include "myeeprom.h"
#include "debug.h"
#include "menus.h"
#include "drivers.h"

#include "sticks.lbm"

#define GET_DR_STATE(x) (!getSwitch(g_model.expoData[x].drSw1,0) ?   \
    DR_HIGH :                                  \
    !getSwitch(g_model.expoData[x].drSw2,0)?   \
    DR_MID : DR_LOW);


#define DO_SQUARE(xx,yy,ww)         \
{uint8_t x,y,w ; x = xx; y = yy; w = ww ; \
    lcd_vline(x-w/2,y-w/2,w);  \
    lcd_hline(x-w/2,y+w/2,w);  \
    lcd_vline(x+w/2,y-w/2,w);  \
    lcd_hline(x-w/2,y-w/2,w);}

#define DO_CROSS(xx,yy,ww)    \
    lcd_vline(xx,yy-ww/2,ww); \
    lcd_hline(xx-ww/2,yy,ww);

#define V_BAR(xx,yy,ll)       \
    lcd_vline(xx-1,yy-ll,ll); \
    lcd_vline(xx  ,yy-ll,ll); \
    lcd_vline(xx+1,yy-ll,ll);

#define NO_HI_LEN 25

#define WCHART 32
#define X0     (128-WCHART-2)
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


enum MainViews
{
    e_outputValues,
    e_outputBars,
    e_inputs1,
    e_inputs2,
    e_inputs3,
    e_timer2,
#ifdef FRSKY
    e_telemetry,
    e_telemetry2,
#endif
    MAX_VIEWS
};

int16_t calibratedStick[7];
int16_t ex_chans[NUM_CHNOUT];          // Outputs + intermidiates
uint8_t s_pgOfs;
uint8_t s_editMode;
uint8_t s_noHi;
uint8_t scroll_disabled;
int8_t scrollLR;
int8_t scrollUD;

int16_t g_chans512[NUM_CHNOUT];







const char *get_curve_string()
{
  return PSTR(CURV_STR)	;
}	

void menu_lcd_onoff( uint8_t x,uint8_t y, uint8_t value, uint8_t mode )
{
    lcd_putsnAtt( x, y, PSTR("OFFON ")+3*value,3,mode ? INVERS:0) ;
}

void menu_lcd_HYPHINV( uint8_t x,uint8_t y, uint8_t value, uint8_t mode )
{
    lcd_putsnAtt( x, y, PSTR("---INV")+3*value,3,mode ? INVERS:0) ;
}



void MState2::check_simple(uint8_t event, uint8_t curr, MenuFuncP *menuTab, uint8_t menuTabSize, uint8_t maxrow)
{
    check(event, curr, menuTab, menuTabSize, 0, 0, maxrow);
}

void MState2::check_submenu_simple(uint8_t event, uint8_t maxrow)
{
    check_simple(event, 0, 0, 0, maxrow);
}

void DisplayScreenIndex(uint8_t index, uint8_t count, uint8_t attr)
{
  lcd_outdezAtt(128,0,count,attr);
  lcd_putcAtt(1+128-FW*(count>9 ? 3 : 2),0,'/',attr);
  lcd_outdezAtt(1+128-FW*(count>9 ? 3 : 2),0,index+1,attr);
}

//#define MAXCOL(row) (horTab ? pgm_read_byte(horTab+min(row, horTabMax)) : (const uint8_t)0)
#define MAXCOL(row) (horTab ? pgm_read_byte(horTab+min(row, horTabMax)) : (const uint8_t)0)
#define INC(val,max) if(val<max) {val++;} else {val=0;}
#define DEC(val,max) if(val>0  ) {val--;} else {val=max;}
void MState2::check(uint8_t event, uint8_t curr, MenuFuncP *menuTab, uint8_t menuTabSize, prog_uint8_t *horTab, uint8_t horTabMax, uint8_t maxrow)
{
        scrollLR = 0;
        scrollUD = 0;

    //check pot 2 - if changed -> scroll menu
    //check pot 3 if changed -> cursor down/up
    //we do this in these brackets to prevent it from happening in the main screen
//    static int16_t p2valprev,p3valprev;
//    scrollLR = (p2valprev-calibratedStick[4])/SCROLL_TH;
//    scrollUD = (p3valprev-calibratedStick[5])/SCROLL_TH;

//    if(scrollLR) p2valprev = calibratedStick[4];
//    if(scrollUD) p3valprev = calibratedStick[5];

//    if(scroll_disabled || g_eeGeneral.disablePotScroll)
//    {
        scrollLR = 0;
        scrollUD = 0;
        scroll_disabled = 0;
//    }

//    if(scrollLR || scrollUD || p1valdiff) g_LightOffCounter = g_eeGeneral.lightAutoOff*500; // on keypress turn the light on 5*100

	if (menuTab)
	{
    uint8_t attr = m_posVert==0 ? INVERS : 0;


    if(m_posVert==0)
    {
      if(scrollLR && !s_editMode)
      {
        int8_t cc = curr - scrollLR;
        if(cc<1) cc = 0;
        if(cc>(menuTabSize-1)) cc = menuTabSize-1;

        if(cc!=curr)
        {
            //                    if(((MenuFuncP)pgm_read_adr(&menuTab[cc])) == menuProcDiagCalib)
            //                        chainMenu(menuProcDiagAna);
            //                    else
          chainMenu((MenuFuncP)pgm_read_adr(&menuTab[cc]));
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
      }
    }


    //        scrollLR = 0;
    DisplayScreenIndex(curr, menuTabSize, attr);
  }

  uint8_t maxcol = MAXCOL(m_posVert);

  if(!s_editMode)
  {
    if(scrollUD)
    {
      int8_t cc = m_posVert - scrollUD;
      if(cc<1) cc = 0;
      if(cc>=maxrow) cc = maxrow;
      m_posVert = cc;

      m_posHorz = min(m_posHorz, MAXCOL(m_posVert));
      m_posHorz = min(m_posHorz, MAXCOL(m_posVert));
      BLINK_SYNC;

      scrollUD = 0;
    }

    if(m_posVert>0 && scrollLR)
    {
      int8_t cc = m_posHorz - scrollLR;
      if(cc<1) cc = 0;
      if(cc>=MAXCOL(m_posVert)) cc = MAXCOL(m_posVert);
      m_posHorz = cc;

      BLINK_SYNC;
      //            scrollLR = 0;
    }
  }

  switch(event)
  {
    case EVT_ENTRY:
        //if(m_posVert>maxrow)
        init();
        s_editMode = false;
        //init();BLINK_SYNC;
    break;
    case EVT_KEY_FIRST(KEY_MENU):
        if (maxcol > 0)
            s_editMode = !s_editMode;
    break;
    case EVT_KEY_LONG(KEY_EXIT):
        s_editMode = false;
        //popMenu(true); //return to uppermost, beeps itself
        popMenu(false);
    break;
        //fallthrough
    case EVT_KEY_BREAK(KEY_EXIT):
        if(s_editMode) {
            s_editMode = false;
            break;
        }
        if(m_posVert==0 || !menuTab) {
            popMenu();  //beeps itself
        } else {
            audioDefevent(AUDIO_MENUS);
            init();BLINK_SYNC;
        }
    break;

    case EVT_KEY_REPT(KEY_RIGHT):  //inc
        if(m_posHorz==maxcol) break;
    case EVT_KEY_FIRST(KEY_RIGHT)://inc
        if(!horTab || s_editMode)break;
        INC(m_posHorz,maxcol);
        BLINK_SYNC;
    break;

    case EVT_KEY_REPT(KEY_LEFT):  //dec
        if(m_posHorz==0) break;
    case EVT_KEY_FIRST(KEY_LEFT)://dec
        if(!horTab || s_editMode)break;
        DEC(m_posHorz,maxcol);
        BLINK_SYNC;
    break;

    case EVT_KEY_REPT(KEY_DOWN):  //inc
        if(m_posVert==maxrow) break;
    case EVT_KEY_FIRST(KEY_DOWN): //inc
        if(s_editMode)break;
        INC(m_posVert,maxrow);
        m_posHorz = min(m_posHorz, MAXCOL(m_posVert));
        BLINK_SYNC;
    break;

    case EVT_KEY_REPT(KEY_UP):  //dec
        if(m_posVert==0) break;
    case EVT_KEY_FIRST(KEY_UP): //dec
        if(s_editMode)break;
        DEC(m_posVert,maxrow);
        m_posHorz = min(m_posHorz, MAXCOL(m_posVert));
        BLINK_SYNC;
    break;
  }
}



void doMainScreenGrphics()
{
#define BOX_WIDTH     23
#define BAR_HEIGHT    (BOX_WIDTH-1l)
#define MARKER_WIDTH  5
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define BOX_LIMIT     (BOX_WIDTH-MARKER_WIDTH)
#define LBOX_CENTERX  (  SCREEN_WIDTH/4 + 10)
#define LBOX_CENTERY  (SCREEN_HEIGHT-9-BOX_WIDTH/2)
#define RBOX_CENTERX  (3*SCREEN_WIDTH/4 - 10)
#define RBOX_CENTERY  (SCREEN_HEIGHT-9-BOX_WIDTH/2)

    DO_SQUARE(LBOX_CENTERX,LBOX_CENTERY,BOX_WIDTH);
    DO_SQUARE(RBOX_CENTERX,RBOX_CENTERY,BOX_WIDTH);

    DO_CROSS(LBOX_CENTERX,LBOX_CENTERY,3);
    DO_CROSS(RBOX_CENTERX,RBOX_CENTERY,3);
    DO_SQUARE(LBOX_CENTERX+(calibratedStick[0]*BOX_LIMIT/(2*RESX)), LBOX_CENTERY-(calibratedStick[1]*BOX_LIMIT/(2*RESX)), MARKER_WIDTH);
    DO_SQUARE(RBOX_CENTERX+(calibratedStick[3]*BOX_LIMIT/(2*RESX)), RBOX_CENTERY-(calibratedStick[2]*BOX_LIMIT/(2*RESX)), MARKER_WIDTH);

    //    V_BAR(SCREEN_WIDTH/2-5,SCREEN_HEIGHT-10,((calibratedStick[4]+RESX)*BAR_HEIGHT/(RESX*2))+1l) //P1
    //    V_BAR(SCREEN_WIDTH/2  ,SCREEN_HEIGHT-10,((calibratedStick[5]+RESX)*BAR_HEIGHT/(RESX*2))+1l) //P2
    //    V_BAR(SCREEN_WIDTH/2+5,SCREEN_HEIGHT-10,((calibratedStick[6]+RESX)*BAR_HEIGHT/(RESX*2))+1l) //P3

    // Optimization by Mike Blandford
    {
        uint8_t x, y, len ;			// declare temporary variables
        for( x = -5, y = 4 ; y < 7 ; x += 5, y += 1 )
        {
            len = ((calibratedStick[y]+RESX)*BAR_HEIGHT/(RESX*2))+1l ;  // calculate once per loop
            V_BAR(SCREEN_WIDTH/2+x,SCREEN_HEIGHT-8, len )
        }
    }
}





uint16_t g_tmr1Latency_max;
uint16_t g_tmr1Latency_min = 0x7ff;
uint16_t g_timeMain;
void menuProcStatistic2(uint8_t event)
{
  TITLE("STAT2");
	lcd_outhex4( 10*FW, 0*FH, event ) ;

  switch(event)
  {
    case EVT_KEY_FIRST(KEY_MENU):
        g_tmr1Latency_min = 0x7ff;
        g_tmr1Latency_max = 0;
        g_timeMain    = 0;
        audioDefevent(AUDIO_MENUS);
        break;
    case EVT_KEY_FIRST(KEY_DOWN):
//        chainMenu(menuProcStatistic);
        break;
    case EVT_KEY_FIRST(KEY_UP):
    case EVT_KEY_FIRST(KEY_EXIT):
        chainMenu(menuProc0);
        break;
  }
  lcd_puts_P( 0*FW,  1*FH, PSTR("tmr1Lat max    us"));
  lcd_outdez(14*FW , 1*FH, g_tmr1Latency_max/2 );
  lcd_puts_P( 0*FW,  2*FH, PSTR("tmr1Lat min    us"));
  lcd_outdez(14*FW , 2*FH, g_tmr1Latency_min/2 );
  lcd_puts_P( 0*FW,  3*FH, PSTR("tmr1 Jitter    us"));
  lcd_outdez(14*FW , 3*FH, (g_tmr1Latency_max - g_tmr1Latency_min) /2 );
  lcd_puts_P( 0*FW,  4*FH, PSTR("tmain          ms"));
  lcd_outdezAtt(14*FW , 4*FH, (g_timeMain*100)/16 ,PREC2);

//    lcd_puts_P( 0*FW,  5*FH, PSTR("Stack          b"));
//    lcd_outhex4( 10*FW+3, 5*FH, stack_free() ) ;

  lcd_puts_P( 3*FW,  7*FH, PSTR("[MENU] to refresh"));
}

void menuProc0(uint8_t event)
{
//  static uint8_t trimSwLock;
  uint8_t view = g_eeGeneral.view & 0xf;
	
	switch(event)
	{
    case EVT_KEY_LONG(KEY_UP):
      chainMenu(menuProcStatistic2);
      killEvents(event);
    break;

    case EVT_KEY_BREAK(KEY_UP) :
      g_eeGeneral.view = view+1;
      if(g_eeGeneral.view>=MAX_VIEWS) g_eeGeneral.view=0 ;
//      STORE_GENERALVARS;     //eeWriteGeneral() ;
        //        eeDirty(EE_GENERAL) ;
      audioDefevent(AUDIO_KEYPAD_UP) ;
    break;

    case EVT_KEY_BREAK(KEY_DOWN) :
      if(view>0)
        g_eeGeneral.view = view - 1 ;
      else
         g_eeGeneral.view = MAX_VIEWS-1 ;
//      STORE_GENERALVARS;     //eeWriteGeneral() ;
      //        eeDirty(EE_GENERAL);
      audioDefevent(AUDIO_KEYPAD_DOWN) ;
    break;
		 
	}

  if (g_eeGeneral.view < 0x10)
	{
	  register  uint8_t x=FW*2;
  	register uint8_t att = (g_vbat100mV < g_eeGeneral.vBatWarn ? BLINK : 0) | DBLSIZE;
  	register uint32_t i ;

		for( i=0 ; i<sizeof(g_model.name);i++)
		{
    	lcd_putcAtt(x+i*2*FW-i-2, 0*FH, g_model.name[i],DBLSIZE);
		}

	  putsVBat(x+4*FW, 2*FH, att|NO_UNIT ) ;
  	lcd_putc( x+4*FW, 3*FH, 'V' ) ;

//  if(s_timerState != TMR_OFF)
//	{
//      uint8_t att = DBLSIZE | (s_timerState==TMR_BEEPING ? BLINK : 0);
//      putsTime(x+14*FW-2, FH*2, s_timerVal, att,att);
//      putsTmrMode(x+7*FW-FW/2,FH*3,0);
//  }

  	lcd_putsnAtt(x+4*FW,     2*FH,PSTR("ExpExFFneMedCrs")+3*g_model.trimInc,3, 0);
  	lcd_putsnAtt(x+8*FW-FW/2,2*FH,PSTR("   TTm")+3*g_model.thrTrim,3, 0);

  	//trim sliders
  	for( i=0 ; i<4 ; i++ )
  	{
	#define TL 27
  	  //                        LH LV RV RH
  	  static uint8_t x[4]    = {128*1/4+2, 4, 128-4, 128*3/4-2};
  	  static uint8_t vert[4] = {0,1,1,0};
  	  register uint8_t xm, ym ;
  	  xm=x[i] ;
  	  register int8_t val = max((int8_t)-(TL+1),min((int8_t)(TL+1),(int8_t)(*TrimPtr[i]/4)));
  	  if(vert[i])
			{
  	    ym=31;
  	    lcd_vline(xm,   ym-TL, TL*2);

  	      if(((g_eeGeneral.stickMode&1) != (i&1)) || !(g_model.thrTrim)){
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
		}
 	}
  else
	{
    lcd_putsnAtt(0, 0, g_model.name, sizeof(g_model.name), INVERS);
    uint8_t att = (g_vbat100mV < g_eeGeneral.vBatWarn ? BLINK : 0);
    putsVBat(14*FW,0,att);
//    if(s_timerState != TMR_OFF)
// 		{
//        att = (s_timerState==TMR_BEEPING ? BLINK : 0);
//        putsTime(18*FW+3, 0, s_timerVal, att, att);
//    }
  }
  
	if(view<e_inputs1)
	{
    register uint32_t i ;
    for( i=0; i<8; i++)
    {
      uint8_t x0,y0;
      int16_t val = g_chans512[i];
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
          int8_t l = (abs(val) * WBAR2 + 512) / 1024;
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
  else if(view<e_timer2)
	{
		doMainScreenGrphics() ;

    register uint32_t i ;
  	register uint32_t a = (view == e_inputs1) ? 0 : 9+(view-3)*6;
  	register uint32_t b = (view == e_inputs1) ? 6 : 12+(view-3)*6;
  	for( i=a; i<(a+3); i++) lcd_putsnAtt(2*FW-2 ,(i-a)*FH+4*FH,get_switches_string()+3*i,3,getSwitch(i+1, 0, 0) ? INVERS : 0);
  	for( i=b; i<(b+3); i++) lcd_putsnAtt(17*FW-1,(i-b)*FH+4*FH,get_switches_string()+3*i,3,getSwitch(i+1, 0, 0) ? INVERS : 0);

	}
}



