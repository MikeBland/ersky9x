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


void menuProcSetup(uint8_t event) ;
void menuProcDiagAna(uint8_t event) ;
void menuProcDiagKeys(uint8_t event) ;
void menuProcDiagCalib(uint8_t event) ;
void menuProcTrainer(uint8_t event) ;
void menuProcDiagVers(uint8_t event) ;


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

enum EnumTabDiag
{
  e_Setup,
  e_Trainer,
  e_Vers,
  e_Keys,
  e_Ana,
  e_Calib
};

MenuFuncP menuTabDiag[] =
{
  menuProcSetup,
  menuProcTrainer,
  menuProcDiagVers,
  menuProcDiagKeys,
  menuProcDiagAna,
  menuProcDiagCalib
};


int16_t calibratedStick[7];
int16_t ex_chans[NUM_CHNOUT];          // Outputs + intermediates
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

#define PARAM_OFS   17*FW

void menu_lcd_onoff( uint8_t x,uint8_t y, uint8_t value, uint8_t mode )
{
    lcd_putsnAtt( x, y, PSTR("OFFON ")+3*value,3,mode ? INVERS:0) ;
}

void menu_lcd_HYPHINV( uint8_t x,uint8_t y, uint8_t value, uint8_t mode )
{
    lcd_putsnAtt( x, y, PSTR("---INV")+3*value,3,mode ? INVERS:0) ;
}



uint8_t onoffMenuItem( uint8_t value, uint8_t y, const char *s, uint8_t sub, int8_t subN, uint8_t event )
{
  lcd_puts_P(0, y, s);
  menu_lcd_onoff( PARAM_OFS, y, value, sub==subN ) ;
  if(sub==subN) CHECK_INCDEC_H_GENVAR(event, value, 0, 1);
  return value ;
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

//				uputs("\r\nI=") ;
//				txmit(index+'0') ;
//				if ( i != 0 )
//				{
       	chainMenu((MenuFuncP)pgm_read_adr(&menuTab[index-1]));
//				}
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
    	killEvents(event) ;
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
    
		case EVT_KEY_LONG(KEY_LEFT) :
      pushMenu(menuProcSetup) ;
      killEvents(event) ;
    break ;
		 
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
  else  // New Timer2 display
  {
    putsTime(30+5*FW, FH*5, Timer2, DBLSIZE, DBLSIZE);
  }
}


void menuProcSetup(uint8_t event)
{


///*
//#ifdef BEEPSPKR
//#define COUNT_ITEMS 22
//#else
//#define COUNT_ITEMS 20
//#endif
//*/

#ifdef FRSKY
	uint8_t vCountItems = 21; //21 is default
//		switch (g_eeGeneral.speakerMode){
//				//beeper
//				case 0:
//						vCountItems = 21;
//						break;
//				//piezo speaker
//			 	case 1:
//			 			vCountItems = 24;
//			 			break;
//			 	//pcmwav
//			  case 2:
//						vCountItems = 23;
//						break;	  	
//		}		
//		if((g_eeGeneral.speakerMode == 1 || g_eeGeneral.speakerMode == 2) && g_eeGeneral.frskyinternalalarm == 0){ // add in alert red/org/yel
//				vCountItems = vCountItems + 3;
//		}		
		
#else 
	uint8_t vCountItems = 19 ; //21 is default
//		switch (g_eeGeneral.speakerMode){
//				//beeper
//				case 0:
//						vCountItems = 21;
//						break;
//				//piezo speaker
//			 	case 1:
//			 			vCountItems = 23;
//			 			break;
//			 	//pcmwav
//			  case 2:
//						vCountItems = 22;
//						break;	  	
//		}
#endif


//	SIMPLE_MENU("RADIO SETUP", menuTabDiag, e_Setup, 20 ) ;
//  //  SIMPLE_MENU("RADIO SETUP", menuTabDiag, e_Setup, COUNT_ITEMS+1);
			SIMPLE_MENU("RADIO SETUP", menuTabDiag, e_Setup, vCountItems+1);
    uint8_t  sub    = mstate2.m_posVert;
    uint8_t subSub = mstate2.m_posHorz;

  evalOffset(sub, 7);

//    //if(s_pgOfs==COUNT_ITEMS-7) s_pgOfs= sub<(COUNT_ITEMS-4) ? COUNT_ITEMS-8 : COUNT_ITEMS-6;
    if(s_pgOfs==vCountItems-7) s_pgOfs= sub<(vCountItems-4) ? vCountItems-8 : vCountItems-6;
//    if(s_pgOfs==19-7) s_pgOfs= sub<(19-4) ? 20-8 : 20-6;
    uint8_t y = 1*FH;

  switch(event)
	{
		case EVT_KEY_FIRST(KEY_MENU):
  	  if(sub>0) s_editMode = !s_editMode;
  	break;

  	case EVT_KEY_FIRST(KEY_EXIT):
  	  if(s_editMode)
			{
  	    s_editMode = false;
  	    killEvents(event);
  	  }
  	break;

    case EVT_KEY_REPT(KEY_LEFT):
    case EVT_KEY_FIRST(KEY_LEFT):
      if(sub==1 && subSub>0 && s_editMode) mstate2.m_posHorz--;
    break;

    case EVT_KEY_REPT(KEY_RIGHT):
    case EVT_KEY_FIRST(KEY_RIGHT):
      if(sub==1 && subSub<sizeof(g_model.name)-1 && s_editMode) mstate2.m_posHorz++;
    break;

    case EVT_KEY_REPT(KEY_UP):
    case EVT_KEY_FIRST(KEY_UP):
    case EVT_KEY_REPT(KEY_DOWN):
    case EVT_KEY_FIRST(KEY_DOWN):
			if (!s_editMode) mstate2.m_posHorz = 0;
    break;
  }

  uint8_t subN = 1;

  if(s_pgOfs<subN)
	{
    lcd_puts_P(    0,    y, PSTR("Owner Name"));
    lcd_putsnAtt(11*FW,   y, g_eeGeneral.ownerName ,sizeof(g_eeGeneral.ownerName),0 | (sub==subN ? (s_editMode ? 0 : INVERS) : 0));
    if(!s_editMode && scrollLR<0) { s_editMode = true; scrollLR = 0; mstate2.m_posHorz = 0; }
    if(s_editMode && scrollLR>subSub) { s_editMode = false; scrollLR = 0; mstate2.m_posHorz = 0; }

    if(sub==subN && s_editMode)
		{
      mstate2.m_posHorz -= scrollLR;
      if((int8_t(mstate2.m_posHorz))<0) mstate2.m_posHorz = 0;
      if((int8_t(mstate2.m_posHorz))>(GENERAL_OWNER_NAME_LEN-1)) mstate2.m_posHorz = GENERAL_OWNER_NAME_LEN-1;
      scrollLR = 0;

      char v = char2idx(g_eeGeneral.ownerName[subSub]);
      if(  /*p1valdiff || */ event==EVT_KEY_FIRST(KEY_DOWN) || event==EVT_KEY_FIRST(KEY_UP) || event==EVT_KEY_REPT(KEY_DOWN) || event==EVT_KEY_REPT(KEY_UP))
        CHECK_INCDEC_H_GENVAR( event,v ,0,NUMCHARS-1);
      v = idx2char(v);
      g_eeGeneral.ownerName[subSub]=v;
      lcd_putcAtt((11+subSub)*FW, y, v,INVERS);
    }
    if((y+=FH)>7*FH) return;
  }subN++;

  if(s_pgOfs<subN)
	{
    uint8_t b ;
    b = g_eeGeneral.beeperVal ;
    lcd_puts_P(0, y,PSTR("Beeper"));
    lcd_putsnAtt(PARAM_OFS - FW - 4, y, PSTR("Quiet ""NoKey ""xShort""Short ""Norm  ""Long  ""xLong ")+6*b,6,(sub==subN ? INVERS:0));
    if(sub==subN) { CHECK_INCDEC_H_GENVAR(event, b, 0, 6); g_eeGeneral.beeperVal = b ; }

    if((y+=FH)>7*FH) return;
  }subN++;

//    if(s_pgOfs<subN) {
//        uint8_t b ;
//        b = g_eeGeneral.speakerMode ;
//        lcd_puts_P(0, y,PSTR("Sound Mode"));
//        lcd_putsnAtt(PARAM_OFS - FW - 4, y, PSTR("Beeper""PiSpkr""PcmWav")+6*b,6,(sub==subN ? INVERS:0));
//        if(sub==subN) { CHECK_INCDEC_H_GENVAR(event, b, 0, 2); g_eeGeneral.speakerMode = b ; }

//        if((y+=FH)>7*FH) return;
//    }subN++;


//if(g_eeGeneral.speakerMode == 1){
	
//    if(s_pgOfs<subN) {
//        lcd_puts_P(0, y,PSTR("Speaker Pitch"));
//        lcd_outdezAtt(PARAM_OFS,y,g_eeGeneral.speakerPitch,(sub==subN ? INVERS : 0)|LEFT);
//        if(sub==subN) {
//            CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.speakerPitch, 1, 100);
//        }
//        if((y+=FH)>7*FH) return;
//    }subN++;

//}
//if(g_eeGeneral.speakerMode == 1 || g_eeGeneral.speakerMode == 2 ){
//    if(s_pgOfs<subN) {
//        lcd_puts_P(0, y,PSTR("Haptic Strength"));
//        lcd_outdezAtt(PARAM_OFS,y,g_eeGeneral.hapticStrength,(sub==subN ? INVERS : 0)|LEFT);
//        if(sub==subN) {
//            CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.hapticStrength, 0, 5);
//        }
//        if((y+=FH)>7*FH) return;
//    }subN++;
//}

    if(s_pgOfs<subN) {
        lcd_puts_P(0, y,PSTR("Contrast"));
        lcd_outdezAtt(PARAM_OFS,y,g_eeGeneral.contrast,(sub==subN ? INVERS : 0)|LEFT);
        if(sub==subN) {
            CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.contrast, 10, 45);
            lcdSetRefVolt(g_eeGeneral.contrast);
        }
        if((y+=FH)>7*FH) return;
    }subN++;

    if(s_pgOfs<subN) {
        lcd_puts_P(0, y,PSTR("Battery warning"));
        putsVolts(PARAM_OFS, y, g_eeGeneral.vBatWarn, (sub==subN ? INVERS : 0)|LEFT);
        if(sub==subN) CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.vBatWarn, 40, 120); //5-10V
        if((y+=FH)>7*FH) return;
    }subN++;

    if(s_pgOfs<subN) {
        lcd_puts_P(0, y,PSTR("Inactivity alarm"));
        lcd_outdezAtt(PARAM_OFS, y, g_eeGeneral.inactivityTimer+10, (sub==subN ? INVERS : 0)|LEFT);
        lcd_putc(Lcd_lastPos, y, 'm');
        if(sub==subN) CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.inactivityTimer, -10, 110); //0..120minutes
        if((y+=FH)>7*FH) return;
    }subN++;

    if(s_pgOfs<subN) {
        lcd_puts_P(0, y,PSTR("Filter ADC"));
        lcd_putsnAtt(PARAM_OFS, y, PSTR("SINGOSMPFILT")+4*g_eeGeneral.filterInput,4,(sub==subN ? INVERS:0));
        if(sub==subN) CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.filterInput, 0, 2);
        if((y+=FH)>7*FH) return;
    }subN++;

    if(s_pgOfs<subN) {
        g_eeGeneral.throttleReversed = onoffMenuItem( g_eeGeneral.throttleReversed, y, PSTR("Throttle reverse"), sub, subN, event ) ;
        if((y+=FH)>7*FH) return;
    }subN++;

    if(s_pgOfs<subN) {
        g_eeGeneral.minuteBeep = onoffMenuItem( g_eeGeneral.minuteBeep, y, PSTR("Minute beep"), sub, subN, event ) ;
        if((y+=FH)>7*FH) return;
    }subN++;

    if(s_pgOfs<subN) {
        g_eeGeneral.preBeep = onoffMenuItem( g_eeGeneral.preBeep, y, PSTR("Beep countdown"), sub, subN, event ) ;
        if((y+=FH)>7*FH) return;
    }subN++;

    if(s_pgOfs<subN) {
        g_eeGeneral.flashBeep = onoffMenuItem( g_eeGeneral.flashBeep, y, PSTR("Flash on beep"), sub, subN, event ) ;
        if((y+=FH)>7*FH) return;
    }subN++;

    if(s_pgOfs<subN) {
        lcd_puts_P(0, y,PSTR("Light switch"));
        putsDrSwitches(PARAM_OFS-FW,y,g_eeGeneral.lightSw,sub==subN ? INVERS : 0);
        if(sub==subN) CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.lightSw, -MAX_DRSWITCH, MAX_DRSWITCH);
        if((y+=FH)>7*FH) return;
    }subN++;

    if(s_pgOfs<subN) {
        lcd_puts_P(0, y,PSTR("Light off after"));
        if(g_eeGeneral.lightAutoOff) {
            lcd_outdezAtt(PARAM_OFS, y, g_eeGeneral.lightAutoOff*5,LEFT|(sub==subN ? INVERS : 0));
            lcd_putc(Lcd_lastPos, y, 's');
        }
        else
            lcd_putsnAtt(PARAM_OFS, y, PSTR("OFF"),3,(sub==subN ? INVERS:0));
        if(sub==subN) CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.lightAutoOff, 0, 600/5);
        if((y+=FH)>7*FH) return;
    }subN++;

    if(s_pgOfs<subN) {
        uint8_t b = 1-g_eeGeneral.disableSplashScreen;
        g_eeGeneral.disableSplashScreen = 1-onoffMenuItem( b, y, PSTR("Splash screen"), sub, subN, event ) ;
        if((y+=FH)>7*FH) return;
    }subN++;

    if(s_pgOfs<subN) {
        uint8_t b = 1-g_eeGeneral.disableThrottleWarning;
        g_eeGeneral.disableThrottleWarning = 1-onoffMenuItem( b, y, PSTR("Throttle Warning"), sub, subN, event ) ;
        if((y+=FH)>7*FH) return;
    }subN++;

    if(s_pgOfs<subN) {
        uint8_t b = 1-g_eeGeneral.disableSwitchWarning;
        g_eeGeneral.disableSwitchWarning = 1-onoffMenuItem( b, y, PSTR("Switch Warning"), sub, subN, event ) ;
        if((y+=FH)>7*FH) return;
    }subN++;

    if(s_pgOfs<subN) {
        uint8_t b = 1-g_eeGeneral.disableMemoryWarning;
        g_eeGeneral.disableMemoryWarning = 1-onoffMenuItem( b, y, PSTR("Memory Warning"), sub, subN, event ) ;
        //						;
        //        }
        if((y+=FH)>7*FH) return;
    }subN++;

    if(s_pgOfs<subN) {
        uint8_t b = 1-g_eeGeneral.disableAlarmWarning;
        g_eeGeneral.disableAlarmWarning = 1-onoffMenuItem( b, y, PSTR("Alarm Warning"), sub, subN, event ) ;
        if((y+=FH)>7*FH) return;
    }subN++;

    
    if(s_pgOfs<subN)
    {
        uint8_t b ;
        b = 1-g_eeGeneral.disablePotScroll ;
        g_eeGeneral.disablePotScroll = 1-onoffMenuItem( b, y, PSTR("PotScroll"), sub, subN, event ) ;
        if((y+=FH)>7*FH) return;
    }subN++;

//    if(s_pgOfs<subN)
//    {
//        uint8_t b ;
//        b = 1-g_eeGeneral.disableBG ;
//        //		    lcd_puts_P(    0,    y, PSTR("BandGap"));
//        //		    menu_lcd_onoff( 10*FW, y, b, sub==subN ) ;
//        //		    if(sub==subN) { CHECK_INCDEC_H_MODELVAR(event,b,0,1); g_eeGeneral.disableBG = 1-b ; }
//        g_eeGeneral.disableBG = 1-onoffMenuItem( b, y, PSTR("BandGap"), sub, subN, event ) ;
//        if((y+=FH)>7*FH) return;
//    }subN++;

//frsky alert mappings
//#ifdef FRSKY

//		if(g_eeGeneral.speakerMode == 1 || g_eeGeneral.speakerMode == 2){
//						if(s_pgOfs<subN) {
//				        g_eeGeneral.frskyinternalalarm = onoffMenuItem( g_eeGeneral.frskyinternalalarm, y, PSTR("Int. Frsky alarm"), sub, subN, event ) ;
//				        if((y+=FH)>7*FH) return;
//				    }subN++;
//		}		    
				    
//    if((g_eeGeneral.speakerMode == 1 || g_eeGeneral.speakerMode == 2) && g_eeGeneral.frskyinternalalarm == 0){ 
    

					  
						
//						for ( uint8_t i = 0 ; i < 3 ; i += 1 )
//					  {
//					    uint8_t b ;
					
//					    b = g_eeGeneral.FRSkyYellow ;    // Done here to stop a compiler warning
//					    if(s_pgOfs<subN)
//							{
								
//								if ( i == 0 )
//								{
//					        lcd_puts_P(0, y,PSTR("Alert [Yel]"));
//								}
//								else if ( i == 1 )
//								{
//					        b = g_eeGeneral.FRSkyOrange ;
//					        lcd_puts_P(0, y,PSTR("Alert [Org]"));
//								}
//								else if ( i == 2 )
//								{
//					        b = g_eeGeneral.FRSkyRed ;
//					        lcd_puts_P(0, y,PSTR("Alert [Red]"));
//								}
//					      //lcd_putsnAtt(PARAM_OFS - FW - 4, y, PSTR("Tone1 ""Tone2 ""Tone3 ""Tone4 ""Tone5 ""hTone1""hTone2""hTone3""hTone4""hTone5")+6*b,6,(sub==subN ? INVERS:0));
//					      if(g_eeGeneral.speakerMode == 1){
//					      			lcd_putsnAtt(PARAM_OFS - FW - 4, y, PSTR("Warn1 ""Warn2 ""Cheep ""Ring  ""SciFi ""Robot ""Chirp ""Tada  ""Crickt""Siren ""AlmClk""Ratata""Tick  ""Haptc1""Haptc2""Haptc3")+6*b,6,(sub==subN ? INVERS:0));
//								}
//					      if(g_eeGeneral.speakerMode == 2){
//					      			lcd_putsnAtt(PARAM_OFS - FW - 4, y, PSTR("Trck1 ""Trck2 ""Trck3 ""Trck4 ""Trck4 ""Trck5 ""Trck6 ""Trck7 ""Trck8 ""Trck9 ""Trck10""Trck11""Trck12""Haptc1""Haptc2""Haptc3")+6*b,6,(sub==subN ? INVERS:0));
//								}								
//					      if(sub==subN)
//								{
//									//CHECK_INCDEC_H_GENVAR(event, b, 0, 9);
//									CHECK_INCDEC_H_GENVAR(event, b, 0, 15);
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
//								  audio.frskyeventSample(b);
//								}
//								if((y+=FH)>7*FH) return;
//					    }subN++;
//					  }
//		}			  
//#endif

    if(s_pgOfs<subN) {
        lcd_puts_P( 1*FW, y, PSTR("Mode"));//sub==3?INVERS:0);
        if(y<7*FH) {for(uint8_t i=0; i<4; i++) lcd_img((6+4*i)*FW, y, sticks,i,0); }
        if((y+=FH)>7*FH) return;

        lcd_putcAtt( 3*FW, y, '1'+g_eeGeneral.stickMode,sub==subN?INVERS:0);
        for(uint8_t i=0; i<4; i++) putsChnRaw( (6+4*i)*FW, y,i+1,0);//sub==3?INVERS:0);

        if(sub==subN) CHECK_INCDEC_H_GENVAR(event,g_eeGeneral.stickMode,0,3);
        if((y+=FH)>7*FH) return;
    }subN++;




}


void menuProcDiagAna(uint8_t event)
{
	register uint32_t i ;
  SIMPLE_MENU("ANA", menuTabDiag, e_Ana, 2);

  int8_t  sub    = mstate2.m_posVert ;
  for(i=0; i<8; i++)
  {
    uint8_t y=i*FH;
    lcd_putc( 4*FW, y, 'A' ) ;
    lcd_putc( 5*FW, y, '1'+i ) ;
    //        lcd_putsn_P( 4*FW, y,PSTR("A1A2A3A4A5A6A7A8")+2*i,2);
    lcd_outhex4( 7*FW, y,anaIn(i));
    if(i<7)  lcd_outdez(15*FW, y, (int32_t)calibratedStick[i]*100/1024);
    if(i==7) putsVBat(15*FW,y,(sub==1 ? INVERS : 0)|PREC1);
  }
//  lcd_putsn_P( 18*FW, 5*FH,PSTR("BG"),2) ;
//  lcd_outdezAtt(20*FW, 6*FH, BandGap, 0);
  if(sub==1)
  {
    scroll_disabled = 1;
    CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.vBatCalib, -127, 127);
  }
}




void evalOffset(int8_t sub, uint8_t max)
{
    if(sub<1) s_pgOfs=0;
    else if(sub-s_pgOfs>max) s_pgOfs = sub-max;
    else if(sub-s_pgOfs<max-6) s_pgOfs = sub-max+6;
}


void menuProcDiagKeys(uint8_t event)
{
  SIMPLE_MENU("DIAG", menuTabDiag, e_Keys, 1);

  uint8_t x=7*FW;
  for(uint8_t i=0; i<9; i++)
  {
    uint8_t y=i*FH; //+FH;
    if(i>(SW_ID0-SW_BASE_DIAG)) y-=FH; //overwrite ID0
    bool t=keyState((EnumKeys)(SW_BASE_DIAG+i));
    putsDrSwitches(x,y,i+1,0); //ohne off,on
    lcd_putcAtt(x+FW*4+2,  y,t+'0',t ? INVERS : 0);
  }

  x=0;
  for(uint8_t i=0; i<6; i++)
  {
    uint8_t y=(5-i)*FH+2*FH;
    bool t=keyState((EnumKeys)(KEY_MENU+i));
    lcd_putsn_P(x, y,PSTR(" Menu Exit Down   UpRight Left")+5*i,5);
    lcd_putcAtt(x+FW*5+2,  y,t+'0',t);
  }

  x=14*FW;
  lcd_putsn_P(x, 3*FH,PSTR("Trim- +"),7);
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


const char menuWhenDone[] = " [MENU] WHEN DONE " ;


void menuProcDiagCalib(uint8_t event)
{
  //    scroll_disabled = 1; // make sure we don't flick out of the screen
  SIMPLE_MENU("CALIBRATION", menuTabDiag, e_Calib, 2);

  //    int8_t  sub    = mstate2.m_posVert ;
  int8_t  sub    = 0;
  mstate2.m_posVert = 0; // make sure we don't scroll or move cursor here
  static int16_t midVals[7];
  static int16_t loVals[7];
  static int16_t hiVals[7];
  static uint8_t idxState;

  for(uint8_t i=0; i<7; i++)
	{ //get low and high vals for sticks and trims
    int16_t vt = anaIn(i);
    loVals[i] = min(vt,loVals[i]);
    hiVals[i] = max(vt,hiVals[i]);
    //if(i>=4) midVals[i] = (loVals[i] + hiVals[i])/2;
  }

  //    if(sub==0)
  //        idxState = 0;

  scroll_disabled = idxState; // make sure we don't scroll while calibrating

  switch(event)
  {
	  case EVT_ENTRY:
      idxState = 0;
    break;

  	case EVT_KEY_BREAK(KEY_MENU):
      idxState++;
      if(idxState==3)
      {
          audioDefevent(AUDIO_MENUS);
          STORE_GENERALVARS;     //eeWriteGeneral();
          idxState = 0;
      }
    break;
  }

  switch(idxState)
  {
  	case 0:
      //START CALIBRATION
      //[MENU]
      lcd_putsnAtt(2*FW, 2*FH, PSTR("START CALIBRATION "), 18, sub>0 ? INVERS : 0);
      lcd_putsnAtt(2*FW, 3*FH, PSTR(" [MENU] TO START  "), 18, sub>0 ? INVERS : 0);
    break;

	  case 1: //get mid
      //SET MIDPOINT
      //[MENU]
      lcd_putsnAtt(2*FW, 2*FH, PSTR("   SET MIDPOINT   "), 18, sub>0 ? INVERS : 0);
      lcd_putsnAtt(2*FW, 3*FH, menuWhenDone, 18, sub>0 ? BLINK : 0);

      for(uint8_t i=0; i<7; i++)
      {
          loVals[i] =  15000;
          hiVals[i] = -15000;
          midVals[i] = anaIn(i);
      }
    break;

	  case 2:
      //MOVE STICKS/POTS
      //[MENU]
      lcd_putsnAtt(2*FW, 2*FH, PSTR(" MOVE STICKS/POTS "), 18, sub>0 ? INVERS : 0);
      lcd_putsnAtt(2*FW, 3*FH, menuWhenDone, 18, sub>0 ? BLINK : 0);

      for(uint8_t i=0; i<7; i++)
          if(abs(loVals[i]-hiVals[i])>50) {
              g_eeGeneral.calibMid[i]  = midVals[i];
              int16_t v = midVals[i] - loVals[i];
              g_eeGeneral.calibSpanNeg[i] = v - v/64;
              v = hiVals[i] - midVals[i];
              g_eeGeneral.calibSpanPos[i] = v - v/64;
          }
      int16_t sum=0;
      for(uint8_t i=0; i<12;i++) sum+=g_eeGeneral.calibMid[i];
      g_eeGeneral.chkSum = sum;
    break;
  }

  doMainScreenGrphics();
}


void menuProcTrainer(uint8_t event)
{
  MENU("TRAINER", menuTabDiag, e_Trainer, 7, {0, 3, 3, 3, 3, 0/*, 0*/});

	int8_t  sub    = mstate2.m_posVert;
	uint8_t subSub = mstate2.m_posHorz;
	uint8_t y;
	bool    edit;
	uint8_t blink ;

//	if (SLAVE_MODE)
//	{ // i am the slave
//	    lcd_puts_P(7*FW, 3*FH, PSTR("Slave"));
//	    return;
//	}

	lcd_puts_P(3*FW, 1*FH, PSTR("mode   % src  sw"));

	sub--;
	y = 2*FH;
	blink =	s_editMode ? BLINK : INVERS ;

	for (uint8_t i=0; i<4; i++)
	{
	  volatile TrainerMix *td = &g_eeGeneral.trainer.mix[i];
	  putsChnRaw(0, y, i+1, 0);

	  edit = (sub==i && subSub==0);
	  lcd_putsnAtt(4*FW, y, PSTR("off += :=")+3*td->mode, 3, edit ? blink : 0);
	  if (edit && s_editMode)
	    CHECK_INCDEC_H_GENVAR(event, td->mode, 0, 2); //!! bitfield

	  edit = (sub==i && subSub==1);
	  lcd_outdezAtt(11*FW, y, td->studWeight*13/4, edit ? blink : 0);
	  if (edit && s_editMode)
	    CHECK_INCDEC_H_GENVAR(event, td->studWeight, -31, 31); //!! bitfield

	  edit = (sub==i && subSub==2);
	  lcd_putsnAtt(12*FW, y, PSTR("ch1ch2ch3ch4")+3*td->srcChn, 3, edit ? blink : 0);
	  if (edit && s_editMode)
	    CHECK_INCDEC_H_GENVAR(event, td->srcChn, 0, 3); //!! bitfield

	  edit = (sub==i && subSub==3);
	  putsDrSwitches(15*FW, y, td->swtch, edit ? blink : 0);
	  if (edit && s_editMode)
	    CHECK_INCDEC_H_GENVAR(event, td->swtch, -MAX_DRSWITCH, MAX_DRSWITCH);

	  y += FH;
	}

	lcd_puts_P(0*FW, y, PSTR("Multiplier"));
	lcd_outdezAtt(13*FW, y, g_eeGeneral.PPM_Multiplier+10, (sub==4 ? INVERS : 0)|PREC1);
	if(sub==4) CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.PPM_Multiplier, -10, 40);
	y += FH;

	edit = (sub==5);
	lcd_putsAtt(0*FW, y, PSTR("Cal"), edit ? INVERS : 0);
	for (uint8_t i=0; i<4; i++)
	{
	    uint8_t x = (i*8+16)*FW/2;
	    lcd_outdezAtt(x , y, (g_ppmIns[i]-g_eeGeneral.trainer.calib[i])*2, PREC1);
	}
	if (edit)
	{
	  if (event==EVT_KEY_FIRST(KEY_MENU))
		{
	  	memcpy(g_eeGeneral.trainer.calib, g_ppmIns, sizeof(g_eeGeneral.trainer.calib));
	  	STORE_GENERALVARS;     //eeWriteGeneral();
	  	//        eeDirty(EE_GENERAL);
	  	audioDefevent(AUDIO_MENUS);
	  }
	}
}

void menuProcDiagVers(uint8_t event)
{
  SIMPLE_MENU("VERSION", menuTabDiag, e_Vers, 1);

  lcd_puts_P(0, 2*FH,stamp4 );
  lcd_puts_P(0, 3*FH,stamp1 );
  lcd_puts_P(0, 4*FH,stamp2 );
  lcd_puts_P(0, 5*FH,stamp3 );
  lcd_puts_P(0, 6*FH,stamp5 );
}



