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
#include "file.h"
#include "templates.h"


#define GET_DR_STATE(x) (!getSwitch(g_model.expoData[x].drSw1,0) ?   \
    DR_HIGH :                                  \
    !getSwitch(g_model.expoData[x].drSw2,0)?   \
    DR_MID : DR_LOW);

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
void menuProcModel(uint8_t event) ;
void menuProcHeli(uint8_t event);
void menuProcModelSelect(uint8_t event) ;
void menuProcMix(uint8_t event) ;
void menuProcExpoAll(uint8_t event);
void menuProcLimits(uint8_t event);
void menuProcCurve(uint8_t event);
void menuProcSwitches(uint8_t event);
void menuProcSafetySwitches(uint8_t event);
void menuProcTemplates(uint8_t event);

uint8_t getMixerCount( void ) ;
bool reachMixerCountLimit( void ) ;


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

extern int16_t p1valdiff;

#include "sticks.lbm"

enum EnumTabModel {
    e_ModelSelect,
    e_Model,
#ifndef NO_HELI
    e_Heli,
#endif
    e_ExpoAll,
    e_Mix,
    e_Limits,
    e_Curve,
    e_Switches,
    e_SafetySwitches//,
//#ifdef FRSKY
//    ,e_Telemetry
//    ,e_Telemetry2
//#endif
#ifndef NO_TEMPLATES
    ,e_Templates
#endif
};

MenuFuncP menuTabModel[] = {
    menuProcModelSelect,
    menuProcModel,
    #ifndef NO_HELI
    menuProcHeli,
    #endif
    menuProcExpoAll,
    menuProcMix,
    menuProcLimits,
    menuProcCurve,
    menuProcSwitches,
    menuProcSafetySwitches//,
//    #ifdef FRSKY
//    ,menuProcTelemetry
//    ,menuProcTelemetry2
//    #endif
    #ifndef NO_TEMPLATES
    ,menuProcTemplates
    #endif
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
	uint8_t x ;
	lcd_outdezAtt(128,0,count,attr);
	x = 1+128-FW*(count>9 ? 3 : 2) ;
  lcd_putcAtt(x,0,'/',attr);
  lcd_outdezAtt(x,0,index+1,attr);
}

#define MAXCOL(row) (horTab ? pgm_read_byte(horTab+min(row, horTabMax)) : (const uint8_t)0)
#define INC(val,max) if(val<max) {val++;} else {val=0;}
#define DEC(val,max) if(val>0  ) {val--;} else {val=max;}
void MState2::check(uint8_t event, uint8_t curr, MenuFuncP *menuTab, uint8_t menuTabSize, prog_uint8_t *horTab, uint8_t horTabMax, uint8_t maxrow)
{
    //    scrollLR = 0;
    //    scrollUD = 0;

    //check pot 2 - if changed -> scroll menu
    //check pot 3 if changed -> cursor down/up
    //we do this in these brackets to prevent it from happening in the main screen
    static int16_t p2valprev,p3valprev;
    scrollLR = (p2valprev-calibratedStick[4])/SCROLL_TH;
    scrollUD = (p3valprev-calibratedStick[5])/SCROLL_TH;

    if(scrollLR) p2valprev = calibratedStick[4];
    if(scrollUD) p3valprev = calibratedStick[5];

    if(scroll_disabled || g_eeGeneral.disablePotScroll)
    {
        scrollLR = 0;
        scrollUD = 0;
        scroll_disabled = 0;
    }

    if(scrollLR || scrollUD || p1valdiff) g_LightOffCounter = g_eeGeneral.lightAutoOff*500; // on keypress turn the light on 5*100

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
    DO_SQUARE(LBOX_CENTERX+(calibratedStick[0]/((2*RESX)/BOX_LIMIT)), LBOX_CENTERY-(calibratedStick[1]/((2*RESX)/BOX_LIMIT)), MARKER_WIDTH);
    DO_SQUARE(RBOX_CENTERX+(calibratedStick[3]/((2*RESX)/BOX_LIMIT)), RBOX_CENTERY-(calibratedStick[2]/((2*RESX)/BOX_LIMIT)), MARKER_WIDTH);

    //    V_BAR(SCREEN_WIDTH/2-5,SCREEN_HEIGHT-10,((calibratedStick[4]+RESX)*BAR_HEIGHT/(RESX*2))+1l) //P1
    //    V_BAR(SCREEN_WIDTH/2  ,SCREEN_HEIGHT-10,((calibratedStick[5]+RESX)*BAR_HEIGHT/(RESX*2))+1l) //P2
    //    V_BAR(SCREEN_WIDTH/2+5,SCREEN_HEIGHT-10,((calibratedStick[6]+RESX)*BAR_HEIGHT/(RESX*2))+1l) //P3

    // Optimization by Mike Blandford
    {
        uint8_t x, y, len ;			// declare temporary variables
        for( x = -5, y = 4 ; y < 7 ; x += 5, y += 1 )
        {
            len = ((calibratedStick[y]+RESX)/((RESX*2)/BAR_HEIGHT))+1 ;  // calculate once per loop
            V_BAR(SCREEN_WIDTH/2+x,SCREEN_HEIGHT-8, len )
        }
    }
}

static uint8_t s_curveChan;

#define XD X0-2

void menuProcCurveOne(uint8_t event)
{
    bool    cv9 = s_curveChan >= MAX_CURVE5;

    SUBMENU("CURVE", 2+(cv9 ? 9 : 5), { 9,0/*repeated...*/});
lcd_outdezAtt(6*FW, 0, s_curveChan+1, INVERS);

int8_t *crv = cv9 ? g_model.curves9[s_curveChan-MAX_CURVE5] : g_model.curves5[s_curveChan];

int8_t  sub    = mstate2.m_posVert-1;
int8_t  subSub = mstate2.m_posHorz;

switch(event){
    case EVT_KEY_FIRST(KEY_EXIT):
        if(subSub!=0) {
            subSub = mstate2.m_posHorz = 0;
            killEvents(event);
        }
        break;
    case EVT_KEY_REPT(KEY_LEFT):
    case EVT_KEY_FIRST(KEY_LEFT):
        if(s_editMode && subSub>0) mstate2.m_posHorz--;
        break;
    case EVT_KEY_REPT(KEY_RIGHT):
    case EVT_KEY_FIRST(KEY_RIGHT):
        if(s_editMode && subSub<(cv9 ? 9 : 5)) mstate2.m_posHorz++;
        break;
}

s_editMode = mstate2.m_posHorz;

for (uint8_t i = 0; i < 5; i++) {
    uint8_t y = i * FH + 16;
    uint8_t attr = sub == i ? INVERS : 0;
    lcd_outdezAtt(4 * FW, y, crv[i], attr);
}
if(cv9)
for (uint8_t i = 0; i < 4; i++) {
    uint8_t y = i * FH + 16;
    uint8_t attr = sub == i + 5 ? INVERS : 0;
    lcd_outdezAtt(8 * FW, y, crv[i + 5], attr);
}
lcd_putsAtt( 2*FW, 1*FH,PSTR("EDIT->"),((sub == -1) && (subSub == 0)) ? INVERS : 0);
lcd_putsAtt( 2*FW, 7*FH,PSTR("PRESET"),sub == (cv9 ? 9 : 5) ? INVERS : 0);

static int8_t dfltCrv;
if((sub<(cv9 ? 9 : 5)) && (sub>-1))  CHECK_INCDEC_H_MODELVAR( event, crv[sub], -100,100);
else  if(sub>0){ //make sure we're not on "EDIT"
    dfltCrv = checkIncDec(event, dfltCrv, -4, 4, 0);
    if (checkIncDec_Ret) {
        if(cv9) for (uint8_t i = 0; i < 9; i++) crv[i] = (i-4)*dfltCrv* 100 / 16;
        else    for (uint8_t i = 0; i < 5; i++) crv[i] = (i-2)*dfltCrv* 100 /  8;
        STORE_MODELVARS;        
        //        eeDirty(EE_MODEL);
        eeWaitComplete() ;
    }
}

if(s_editMode)
{
    for(uint8_t i=0; i<(cv9 ? 9 : 5); i++)
    {
        uint8_t xx = XD-1-WCHART+i*WCHART/(cv9 ? 4 : 2);
        uint8_t yy = Y0-crv[i]*WCHART/100;


        if(subSub==(i+1))
        {
            if((yy-2)<WCHART*2) lcd_hline( xx-1, yy-2, 5); //do selection square
            if((yy-1)<WCHART*2) lcd_hline( xx-1, yy-1, 5);
            if(yy<WCHART*2)     lcd_hline( xx-1, yy  , 5);
            if((yy+1)<WCHART*2) lcd_hline( xx-1, yy+1, 5);
            if((yy+2)<WCHART*2) lcd_hline( xx-1, yy+2, 5);

            if(p1valdiff || event==EVT_KEY_FIRST(KEY_DOWN) || event==EVT_KEY_FIRST(KEY_UP) || event==EVT_KEY_REPT(KEY_DOWN) || event==EVT_KEY_REPT(KEY_UP))
                CHECK_INCDEC_H_MODELVAR( event, crv[i], -100,100);  // edit on up/down
        }
        else
        {
            if((yy-1)<WCHART*2) lcd_hline( xx, yy-1, 3); // do markup square
            if(yy<WCHART*2)     lcd_hline( xx, yy  , 3);
            if((yy+1)<WCHART*2) lcd_hline( xx, yy+1, 3);
        }
    }
}

for (uint8_t xv = 0; xv < WCHART * 2; xv++) {
    uint16_t yv = intpol(xv * (RESXu / WCHART) - RESXu, s_curveChan) / (RESXu
                                                                        / WCHART);
    lcd_plot(XD + xv - WCHART, Y0 - yv);
    if ((xv & 3) == 0) {
        lcd_plot(XD + xv - WCHART, Y0 + 0);
    }
}
lcd_vline(XD, Y0 - WCHART, WCHART * 2);
}



void menuProcCurve(uint8_t event)
{
    SIMPLE_MENU("CURVES", menuTabModel, e_Curve, 1+MAX_CURVE5+MAX_CURVE9);

    int8_t  sub    = mstate2.m_posVert - 1;

    evalOffset(sub, 6);

    switch (event) {
    case EVT_KEY_FIRST(KEY_RIGHT):
    case EVT_KEY_FIRST(KEY_MENU):
        if (sub >= 0) {
            s_curveChan = sub;
            pushMenu(menuProcCurveOne);
        }
        break;
    }

    uint8_t y    = 1*FH;
    uint8_t yd   = 1;
    uint8_t m    = 0;
    for (uint8_t i = 0; i < 7; i++) {
        uint8_t k = i + s_pgOfs;
        uint8_t attr = sub == k ? INVERS : 0;
        bool    cv9 = k >= MAX_CURVE5;

        if(cv9 && (yd>6)) break;
        if(yd>7) break;
        if(!m) m = attr;
        lcd_putsAtt(   FW*0, y,PSTR("CV"),attr);
        lcd_outdezAtt( (k<9) ? FW*3 : FW*4-1, y,k+1 ,attr);

        int8_t *crv = cv9 ? g_model.curves9[k-MAX_CURVE5] : g_model.curves5[k];
        for (uint8_t j = 0; j < (5); j++) {
            lcd_outdezAtt( j*(3*FW+3) + 7*FW, y, crv[j], 0);
        }
        y += FH;yd++;
        if(cv9){
            for (uint8_t j = 0; j < 4; j++) {
                lcd_outdezAtt( j*(3*FW+3) + 7*FW, y, crv[j+5], 0);
            }
            y += FH;yd++;
        }
    }

    if(!m) s_pgOfs++;
}

void setStickCenter() // copy state of 3 primary to subtrim
{
    int16_t zero_chans512_before[NUM_CHNOUT];
    int16_t zero_chans512_after[NUM_CHNOUT];

    perOut(zero_chans512_before,NO_TRAINER+NO_INPUT); // do output loop - zero input channels
    perOut(zero_chans512_after,NO_TRAINER); // do output loop - zero input channels

    for(uint8_t i=0; i<NUM_CHNOUT; i++)
    {
        int16_t v = g_model.limitData[i].offset;
        v += g_model.limitData[i].revert ?
                    (zero_chans512_before[i] - zero_chans512_after[i]) :
                    (zero_chans512_after[i] - zero_chans512_before[i]);
        g_model.limitData[i].offset = max(min(v,1000),-1000); // make sure the offset doesn't go haywire
    }

    // For this operation, keep using the 'MASTER' trims
    for(uint8_t i=0; i<4; i++)
        if(!IS_THROTTLE(i)) g_model.trim[i] = 0;// set trims to zero.
    STORE_MODELVARS_TRIM;
    audioDefevent(AUDIO_WARNING2);
}

void menuProcLimits(uint8_t event)
{
    MENU("LIMITS", menuTabModel, e_Limits, NUM_CHNOUT+2, {0, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 0});

static bool swVal[NUM_CHNOUT];

uint8_t y = 0;
uint8_t k = 0;
int8_t  sub    = mstate2.m_posVert - 1;
uint8_t subSub = mstate2.m_posHorz;

evalOffset(sub, 6);

switch(event)
{
    case EVT_KEY_LONG(KEY_MENU):
        if(sub>=0 && sub<NUM_CHNOUT) {
            int16_t v = g_chans512[sub - s_pgOfs];
            LimitData *ld = &g_model.limitData[sub];
            switch (subSub) {
            case 0:
                ld->offset = (ld->revert) ? -v : v;
                STORE_MODELVARS;
                eeWaitComplete() ;
                break;
            }
        }
        break;
}
//  lcd_puts_P( 4*FW, 1*FH,PSTR("subT min  max inv"));
for(uint8_t i=0; i<7; i++){
    y=(i+1)*FH;
    k=i+s_pgOfs;
    if(k==NUM_CHNOUT) break;
    LimitData *ld = &g_model.limitData[k];
    int16_t v = (ld->revert) ? -ld->offset : ld->offset;
    if((g_chans512[k] - v) >  50) swVal[k] = (true==ld->revert);// Switch to raw inputs?  - remove trim!
    if((g_chans512[k] - v) < -50) swVal[k] = (false==ld->revert);
    putsChn(0,y,k+1,0);
    lcd_putcAtt(12*FW+FW/2, y, (swVal[k] ? 127 : 126),0); //'<' : '>'
    for(uint8_t j=0; j<4;j++)
		{
        uint8_t attr = ((sub==k && subSub==j) ? (s_editMode ? BLINK : INVERS) : 0);
				uint8_t active = (attr && (s_editMode || p1valdiff)) ;
        switch(j)
        {
        case 0:
            lcd_outdezAtt(  8*FW, y,  ld->offset, attr|PREC1);
            if(active) {
                ld->offset = checkIncDec16(event, ld->offset, -1000, 1000, EE_MODEL);
            }
            break;
        case 1:
            lcd_outdezAtt(  12*FW, y, (int8_t)(ld->min-100),   attr);
            if(active) {
                ld->min -=  100;
                if(g_model.extendedLimits)
                    CHECK_INCDEC_H_MODELVAR( event, ld->min, -125,125);
                else
                    CHECK_INCDEC_H_MODELVAR( event, ld->min, -100,100);
                ld->min +=  100;
            }
            break;
        case 2:
            lcd_outdezAtt( 17*FW, y, (int8_t)(ld->max+100),    attr);
            if(active) {
                ld->max +=  100;
                if(g_model.extendedLimits)
                    CHECK_INCDEC_H_MODELVAR( event, ld->max, -125,125);
                else
                    CHECK_INCDEC_H_MODELVAR( event, ld->max, -100,100);
                ld->max -=  100;
            }
            break;
        case 3:
            lcd_putsnAtt(   18*FW, y, PSTR("---INV")+ld->revert*3,3,attr);
            if(active) {
                CHECK_INCDEC_H_MODELVAR(event, ld->revert, 0, 1);
            }
            break;
        }
    }
}
if(k==NUM_CHNOUT){
    //last line available - add the "copy trim menu" line
    uint8_t attr = (sub==NUM_CHNOUT) ? INVERS : 0;
    lcd_putsAtt(  3*FW,y,PSTR("COPY TRIM [MENU]"),s_noHi ? 0 : attr);
    if(attr && event==EVT_KEY_LONG(KEY_MENU)) {
        s_noHi = NO_HI_LEN;
        killEvents(event);
        setStickCenter(); //if highlighted and menu pressed - copy trims
    }
}
}

#ifdef FRSKY
void menuProcTelemetry(uint8_t event)
{
    MENU("TELEMETRY", menuTabModel, e_Telemetry, 8, {0, 1, 1, 2, 2, 1, 2/*, 2*/});

uint8_t  sub   = mstate2.m_posVert;
uint8_t subSub = mstate2.m_posHorz;
uint8_t blink;
uint8_t y = 2*FH;

switch(event){
    case EVT_KEY_BREAK(KEY_DOWN):
    case EVT_KEY_BREAK(KEY_UP):
    case EVT_KEY_BREAK(KEY_LEFT):
    case EVT_KEY_BREAK(KEY_RIGHT):
        if(s_editMode)
            FRSKY_setModelAlarms(); // update Fr-Sky module when edit mode exited
}

blink = s_editMode ? BLINK : INVERS ;
uint8_t subN = 1;

lcd_puts_Pleft(FH, PSTR("UsrProto"));
{
uint8_t b ;
b = g_model.FrSkyUsrProto ;
lcd_putsnAtt(  10*FW, FH, PSTR("FrHubWSHhi")+5*b,5,(sub==subN && subSub==0 ? INVERS:0));
if(sub==subN && subSub==0 && s_editMode) { CHECK_INCDEC_H_MODELVAR(event,b,0,1); g_model.FrSkyUsrProto = b ; }
b = g_model.FrSkyImperial ;
lcd_putsnAtt(  16*FW, FH, PSTR("MetImp")+3*b,3,(sub==subN && subSub==1 ? INVERS:0));
if(sub==subN && subSub==1 && s_editMode) { CHECK_INCDEC_H_MODELVAR(event,b,0,1); g_model.FrSkyImperial = b ; }
}
subN++;

for (int i=0; i<2; i++) {
    lcd_puts_Pleft(y, PSTR("A  channel"));
    lcd_putc(FW, y, '1'+i);
    putsTelemValue(16*FW, y, g_model.frsky.channels[i].ratio, i, (sub==subN && subSub==0 ? blink:0)|NO_UNIT, 0 ) ;
    //    lcd_putsnAtt(16*FW, y, PSTR("v-")+g_model.frsky.channels[i].type, 1, (sub==subN && subSub==1 ? blink:0));
    lcd_putsnAtt(16*FW, y, PSTR("v-VA")+g_model.frsky.channels[i].type, 1, (sub==subN && subSub==1 ? blink:0));

    if (sub==subN && (s_editMode || p1valdiff)) {
        switch (subSub) {
        case 0:
            g_model.frsky.channels[i].ratio = checkIncDec16(event, g_model.frsky.channels[i].ratio, 0, 255, EE_MODEL);
            break;
        case 1:
            //            CHECK_INCDEC_H_MODELVAR(event, g_model.frsky.channels[i].type, 0, 1);
            CHECK_INCDEC_H_MODELVAR(event, g_model.frsky.channels[i].type, 0, 3);
            break;
        }
    }
    subN++; y+=FH;

    for (int j=0; j<2; j++) {
        uint8_t ag;
        uint8_t al;
        al = ALARM_LEVEL(i, j);
        ag = ALARM_GREATER(i, j);
        lcd_putsAtt(4, y, PSTR("alrm"), 0);
        lcd_putsnAtt(6*FW, y, PSTR("---YelOrgRed")+3*al,3,(sub==subN && subSub==0 ? blink:0));
        lcd_putsnAtt(11*FW, y, PSTR("<>")+ag,1,(sub==subN && subSub==1 ? blink:0));
        putsTelemValue(16*FW, y, g_model.frsky.channels[i].alarms_value[j], i, (sub==subN && subSub==2 ? blink:0)|NO_UNIT, 1 ) ;

        if(sub==subN && (s_editMode || p1valdiff)) {
            uint8_t original ;
            uint8_t value ;
            switch (subSub) {
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
        subN++; y+=FH;
    }
}
}

extern uint8_t frskyRSSIlevel[2] ;
extern uint8_t frskyRSSItype[2] ;


void menuProcTelemetry2(uint8_t event)
{
    MENU("TELEMETRY2", menuTabModel, e_Telemetry2, 4, {0, 1, 1, 0});

uint8_t  sub    = mstate2.m_posVert;
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
            FrskyAlarmSendState |= 0x30 ;	 // update Fr-Sky module when edit mode exited
        break ;
}
blink = s_editMode ? BLINK : INVERS ;
uint8_t subN = 1;

for (uint8_t j=0; j<2; j++)
{
    lcd_puts_Pleft( y, PSTR("TxRSSIalrm") );
    if ( j == 1 )
    {
        lcd_putcAtt( 0, y, 'R', 0 ) ;
    }
    lcd_putsnAtt(11*FW, y, PSTR("---YelOrgRed")+3*frskyRSSItype[j],3,(sub==subN && subSub==0 ? blink:0));
    lcd_outdezNAtt(17*FW, y, frskyRSSIlevel[j], (sub==subN && subSub==1 ? blink:0), 3);

    if(sub==subN && (s_editMode || p1valdiff)) {
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

	lcd_puts_Pleft(5*FH, PSTR("AltAlarm"));
  lcd_putsnAtt(11*FW, 5*FH, PSTR("OFF122400")+3*g_model.FrSkyAltAlarm,3,(sub==subN && subSub==0 ? blink:0));
  if(sub==subN) {
  	g_model.FrSkyAltAlarm = checkIncDec16(event, g_model.FrSkyAltAlarm, 0, 2, EE_MODEL);
	}
}

#endif

#ifndef NO_TEMPLATES
void menuProcTemplates(uint8_t event)  //Issue 73
{
    SIMPLE_MENU("TEMPLATES", menuTabModel, e_Templates, NUM_TEMPLATES+3);

    uint8_t y = 0;
    uint8_t k = 0;
    int8_t  sub    = mstate2.m_posVert - 1;

    evalOffset(sub, 6);

    switch(event)
    {
    case EVT_KEY_LONG(KEY_MENU):
        killEvents(event);
        //apply mixes or delete
        s_noHi = NO_HI_LEN;
        if(sub==NUM_TEMPLATES+1)
            clearMixes();
        else if((sub>=0) && (sub<(int8_t)NUM_TEMPLATES))
            applyTemplate(sub);
        audioDefevent(AUDIO_WARNING2);
        break;
    }

    y=1*FH;
    for(uint8_t i=0; i<7; i++){
        k=i+s_pgOfs;
        if(k==NUM_TEMPLATES) break;

        //write mix names here
        lcd_outdezNAtt(3*FW, y, k+1, (sub==k ? INVERS : 0) + LEADING0,2);
        lcd_putsAtt(  4*FW, y, (const char*)(n_Templates[k]), (s_noHi ? 0 : (sub==k ? INVERS  : 0)));
        y+=FH;
    }

    if(y>7*FH) return;
    uint8_t attr = s_noHi ? 0 : ((sub==NUM_TEMPLATES) ? INVERS : 0);
    lcd_puts_P( 1*FW, y,PSTR("Channel Order"));//   RAET->AETR

    //  lcd_putsnAtt(15*FW, y, PSTR(" RETA")+CHANNEL_ORDER(1),1,attr);
    //  lcd_putsnAtt(16*FW, y, PSTR(" RETA")+CHANNEL_ORDER(2),1,attr);
    //  lcd_putsnAtt(17*FW, y, PSTR(" RETA")+CHANNEL_ORDER(3),1,attr);
    //  lcd_putsnAtt(18*FW, y, PSTR(" RETA")+CHANNEL_ORDER(4),1,attr);

    {
        uint8_t i ;
        for ( i = 1 ; i <= 4 ; i += 1 )
        {
            lcd_putsnAtt((14+i)*FW, y, PSTR(" RETA")+CHANNEL_ORDER(i),1,attr);
        }
    }


    if(attr) CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.templateSetup, 0, 23);
    y+=FH;

    if(y>7*FH) return;
    attr = s_noHi ? 0 : ((sub==NUM_TEMPLATES+1) ? INVERS : 0);
    lcd_putsAtt(  1*FW,y,PSTR("CLEAR MIXES [MENU]"),attr);
    y+=FH;

}
#endif

void menuProcSafetySwitches(uint8_t event)
{
    MENU("SAFETY SWITCHES", menuTabModel, e_SafetySwitches, NUM_CHNOUT+1, {0, 2/*repeated*/});

uint8_t y = 0;
uint8_t k = 0;
int8_t  sub    = mstate2.m_posVert - 1;
uint8_t subSub = mstate2.m_posHorz;

evalOffset(sub, 6);

//  lcd_puts_P( 0*FW, 1*FH,PSTR("ch    sw     val"));
for(uint8_t i=0; i<7; i++){
    y=(i+1)*FH;
    k=i+s_pgOfs;
    if(k==NUM_CHNOUT) break;
    SafetySwData *sd = &g_model.safetySw[k];
    putsChn(0,y,k+1,0);
    for(uint8_t j=0; j<=2;j++){
        uint8_t attr = ((sub==k && subSub==j) ? (s_editMode ? BLINK : INVERS) : 0);
				uint8_t active = (attr && (s_editMode || p1valdiff)) ;
        if (j == 0)
        {
            putsDrSwitches(5*FW, y, sd->swtch  , attr);
            if(active) {
                CHECK_INCDEC_H_MODELVAR( event, sd->swtch, -MAX_DRSWITCH,MAX_DRSWITCH);
            }
				}
				else
				{
            lcd_outdezAtt(  16*FW, y, sd->val,   attr);
            if(active) {
                CHECK_INCDEC_H_MODELVAR( event, sd->val, -125,125);
            }
        }
    }
}

}

void menuProcSwitches(uint8_t event)  //Issue 78
{
    MENU("CUSTOM SWITCHES", menuTabModel, e_Switches, NUM_CSW+1, {0, 2/*repeated...*/});

uint8_t y = 0;
uint8_t k = 0;
int8_t  sub    = mstate2.m_posVert - 1;
uint8_t subSub = mstate2.m_posHorz;

evalOffset(sub, 6);

//  lcd_puts_P( 4*FW, 1*FH,PSTR("Function V1  V2"));
for(uint8_t i=0; i<7; i++){
    y=(i+1)*FH;
    k=i+s_pgOfs;
    if(k==NUM_CSW) break;
    uint8_t attr = (sub==k ? (s_editMode ? BLINK : INVERS)  : 0);
    CSwData &cs = g_model.customSw[k];

    //write SW names here
    lcd_putsnAtt( 0*FW , y, PSTR("SW"),2,0);
    lcd_putc(  2*FW , y, k + (k>8 ? 'A'-9: '1'));
    lcd_putsnAtt( 4*FW , y, PSTR(CSWITCH_STR)+CSW_LEN_FUNC*cs.func,CSW_LEN_FUNC,subSub==0 ? attr : 0);

    uint8_t cstate = CS_STATE(cs.func);

    if(cstate == CS_VOFS)
    {
        putsChnRaw(    12*FW, y, cs.v1  ,subSub==1 ? attr : 0);
#ifdef FRSKY
        if (cs.v1 > CHOUT_BASE+NUM_CHNOUT)
            lcd_outdezAtt( 20*FW, y, 125+cs.v2  ,subSub==2 ? attr : 0);
        else
#endif
            lcd_outdezAtt( 20*FW, y, cs.v2  ,subSub==2 ? attr : 0);
    }
    else if(cstate == CS_VBOOL)
    {
        putsDrSwitches(12*FW, y, cs.v1  ,subSub==1 ? attr : 0);
        putsDrSwitches(16*FW, y, cs.v2  ,subSub==2 ? attr : 0);
    }
    else // cstate == CS_COMP
    {
        putsChnRaw(    12*FW, y, cs.v1  ,subSub==1 ? attr : 0);
        putsChnRaw(    17*FW, y, cs.v2  ,subSub==2 ? attr : 0);
    }

    if((s_editMode || p1valdiff) && attr)
        switch (subSub) {
        case 0:
            CHECK_INCDEC_H_MODELVAR( event, cs.func, 0,CS_MAXF);
            if(cstate != CS_STATE(cs.func))
            {
                cs.v1  = 0;
                cs.v2 = 0;
            }
            break;
        case 1:
            switch (cstate) {
            case (CS_VOFS):
                CHECK_INCDEC_H_MODELVAR( event, cs.v1, 0,NUM_XCHNRAW);
                break;
            case (CS_VBOOL):
                CHECK_INCDEC_H_MODELVAR( event, cs.v1, -MAX_DRSWITCH,MAX_DRSWITCH);
                break;
            case (CS_VCOMP):
                CHECK_INCDEC_H_MODELVAR( event, cs.v1, 0,NUM_XCHNRAW);
                break;
            default:
                break;
            }
            break;
        case 2:
            switch (cstate) {
            case (CS_VOFS):
                CHECK_INCDEC_H_MODELVAR( event, cs.v2, -125,125);
                break;
            case (CS_VBOOL):
                CHECK_INCDEC_H_MODELVAR( event, cs.v2, -MAX_DRSWITCH,MAX_DRSWITCH);
                break;
            case (CS_VCOMP):
                CHECK_INCDEC_H_MODELVAR( event, cs.v2, 0,NUM_XCHNRAW);
                break;
            default:
                break;
            }
        }
}
}

static int8_t s_currMixIdx;
static int8_t s_currDestCh;
static bool   s_currMixInsMode;


void deleteMix(uint8_t idx)
{
    memmove(&g_model.mixData[idx],&g_model.mixData[idx+1],
            (MAX_MIXERS-(idx+1))*sizeof(MixData));
    memset(&g_model.mixData[MAX_MIXERS-1],0,sizeof(MixData));
    STORE_MODELVARS;
    eeWaitComplete() ;
}

void insertMix(uint8_t idx)
{
    memmove(&g_model.mixData[idx+1],&g_model.mixData[idx],
            (MAX_MIXERS-(idx+1))*sizeof(MixData) );
    memset(&g_model.mixData[idx],0,sizeof(MixData));
    g_model.mixData[idx].destCh      = s_currDestCh; //-s_mixTab[sub];
    g_model.mixData[idx].srcRaw      = s_currDestCh; //1;   //
    g_model.mixData[idx].weight      = 100;
    STORE_MODELVARS;
    eeWaitComplete() ;
}

void menuProcMixOne(uint8_t event)
{
    SIMPLE_SUBMENU_NOTITLE(14);
    uint8_t x = TITLEP(s_currMixInsMode ? PSTR("INSERT MIX ") : PSTR("EDIT MIX "));

    MixData *md2 = &g_model.mixData[s_currMixIdx] ;
    putsChn(x+1*FW,0,md2->destCh,0);
    int8_t  sub    = mstate2.m_posVert;

    evalOffset(sub, 6);

    for(uint8_t k=0; k<7; k++)
    {
        uint8_t y = (k+1) * FH;
        uint8_t i = k + s_pgOfs;
        uint8_t attr = sub==i ? INVERS : 0;
    		uint8_t b ;
        switch(i){
        case 0:
            lcd_puts_P(  2*FW,y,PSTR("Source"));
            putsChnRaw(   FW*14,y,md2->srcRaw,attr);
            if(attr) CHECK_INCDEC_H_MODELVAR( event, md2->srcRaw, 1,NUM_XCHNRAW);
            break;
        case 1:
            lcd_puts_P(  2*FW,y,PSTR("Weight"));
            lcd_outdezAtt(FW*14,y,md2->weight,attr|LEFT);
            if(attr) CHECK_INCDEC_H_MODELVAR( event, md2->weight, -125,125);
            break;
        case 2:
            lcd_puts_P(  2*FW,y,md2->enableFmTrim ? PSTR("FmTrimVal") : PSTR("Offset"));
            lcd_outdezAtt(FW*14,y,md2->sOffset,attr|LEFT);
            if(attr) CHECK_INCDEC_H_MODELVAR( event, md2->sOffset, -125,125);
            break;
        case 3:
    				b = md2->enableFmTrim ;
            lcd_puts_P(  2*FW,y,PSTR("FlModetrim"));
            lcd_putsnAtt(FW*14,y, PSTR("OFFON ")+3*b,3,attr);  //default is 0=ON
            //            lcd_putsnAtt( x, y, PSTR("OFFON ")+3*value,3,mode ? INVERS:0) ;
            //            menu_lcd_onoff( FW*14, y, md2->enableFmTrim, sub==i ) ;
            if(attr) { CHECK_INCDEC_H_MODELVAR( event, b, 0,1); md2->enableFmTrim = b ; }
            break;
        case 4:
            lcd_puts_P(  2*FW,y,PSTR("Trim"));
            lcd_putsnAtt(FW*14,y, PSTR("ON OFF")+3*md2->carryTrim,3,attr);  //default is 0=ON
            if(attr) CHECK_INCDEC_H_MODELVAR( event, md2->carryTrim, 0,1);
            break;
        case 5:
            lcd_putsAtt(  2*FW,y,PSTR("Curves"),0);
            lcd_putsnAtt( FW*14,y,get_curve_string()+md2->curve*3,3,attr);
            if(attr) CHECK_INCDEC_H_MODELVAR( event, md2->curve, 0,MAX_CURVE5+MAX_CURVE9+7-1);
            if(attr && md2->curve>=CURVE_BASE && event==EVT_KEY_FIRST(KEY_MENU)){
                s_curveChan = md2->curve-CURVE_BASE;
                pushMenu(menuProcCurveOne);
            }
            break;
        case 6:
            lcd_puts_P(  2*FW,y,PSTR("Switch"));
            putsDrSwitches(13*FW,  y,md2->swtch,attr);
            if(attr) CHECK_INCDEC_H_MODELVAR( event, md2->swtch, -MAX_DRSWITCH, MAX_DRSWITCH);
            break;
        case 7:
            lcd_puts_P(  2*FW,y,PSTR("Warning"));
						b = md2->mixWarn ;
            if(b)
                lcd_outdezAtt(FW*14,y,b,attr|LEFT);
            else
                lcd_putsAtt(  FW*14,y,PSTR("OFF"),attr);
            if(attr) { CHECK_INCDEC_H_MODELVAR( event, b, 0,3); md2->mixWarn = b ; }
            break;
        case 8:
            lcd_puts_P(  2*FW,y,PSTR("Multpx"));
            lcd_putsnAtt(14*FW, y,PSTR("Add     MultiplyReplace ")+8*md2->mltpx,8,attr);
            if(attr) CHECK_INCDEC_H_MODELVAR( event, md2->mltpx, 0, 2); //!! bitfield
            break;
        case 9:
            lcd_puts_P(  2*FW,y,PSTR("Delay Down"));
            lcd_outdezAtt(FW*16,y,md2->delayDown,attr);
            if(attr)  CHECK_INCDEC_H_MODELVAR( event, md2->delayDown, 0,15); //!! bitfield
            break;
        case 10:
            lcd_puts_P(  2*FW,y,PSTR("Delay Up"));
            lcd_outdezAtt(FW*16,y,md2->delayUp,attr);
            if(attr)  CHECK_INCDEC_H_MODELVAR( event, md2->delayUp, 0,15); //!! bitfield
            break;
        case 11:
            lcd_puts_P(  2*FW,y,PSTR("Slow  Down"));
            lcd_outdezAtt(FW*16,y,md2->speedDown,attr);
            if(attr)  CHECK_INCDEC_H_MODELVAR( event, md2->speedDown, 0,15); //!! bitfield
            break;
        case 12:
            lcd_puts_P(  2*FW,y,PSTR("Slow  Up"));
            lcd_outdezAtt(FW*16,y,md2->speedUp,attr);
            if(attr)  CHECK_INCDEC_H_MODELVAR( event, md2->speedUp, 0,15); //!! bitfield
            break;
        case 13:
            lcd_putsAtt(  2*FW,y,PSTR("DELETE MIX [MENU]"),attr);
            if(attr && event==EVT_KEY_LONG(KEY_MENU)){
                killEvents(event);
                deleteMix(s_currMixIdx);
                audioDefevent(AUDIO_WARNING2);
                popMenu();
            }
            break;
        }
    }
}

struct MixTab{
    uint8_t chId:6;    //:4  1..NUM_XCHNOUT  dst chn id
    uint8_t   showCh:1;// show the dest chn
    uint8_t   hasDat:1;// show the data info
    int8_t selCh;   //:5  1..MAX_MIXERS+NUM_XCHNOUT sel sequence
    int8_t selDat;  //:5  1..MAX_MIXERS+NUM_XCHNOUT sel sequence
    int8_t insIdx;  //:5  0..MAX_MIXERS-1        insert index into mix data tab
    int8_t editIdx; //:5  0..MAX_MIXERS-1        edit   index into mix data tab
} s_mixTab[MAX_MIXERS+NUM_XCHNOUT+1];
int8_t s_mixMaxSel;

void genMixTab()
{
    uint8_t maxDst  = 0;
    uint8_t mtIdx   = 0;
    uint8_t sel     = 1;
    memset(s_mixTab,0,sizeof(s_mixTab));

    MixData *md=g_model.mixData;

    for(uint8_t i=0; i<MAX_MIXERS; i++)
    {
        uint8_t destCh = md[i].destCh;
        if(destCh==0) destCh=NUM_XCHNOUT;
        if(destCh > maxDst){
            while(destCh > maxDst){ //ch-loop, hole alle channels auf
                maxDst++;
                s_mixTab[mtIdx].chId  = maxDst; //mark channel header
                s_mixTab[mtIdx].showCh = true;
                s_mixTab[mtIdx].selCh = sel++; //vorab vergeben, falls keine dat
                s_mixTab[mtIdx].insIdx= i;     //
                mtIdx++;
            }
            mtIdx--; //folding: letztes ch bekommt zusaetzlich dat
            s_mixMaxSel =sel;
            sel--; //letzte zeile hat dat, falls nicht ist selCh schon belegt
        }
        if(md[i].destCh==0) break; //letzter eintrag in mix data tab
        s_mixTab[mtIdx].chId    = destCh; //mark channel header
        s_mixTab[mtIdx].editIdx = i;
        s_mixTab[mtIdx].hasDat  = true;
        s_mixTab[mtIdx].selDat  = sel++;
        if(md[i].destCh == md[i+1].destCh){
            s_mixTab[mtIdx].selCh  = 0; //ueberschreibt letzte Zeile von ch-loop
            s_mixTab[mtIdx].insIdx = 0; //
        }
        else{
            s_mixTab[mtIdx].selCh  = sel++;
            s_mixTab[mtIdx].insIdx = i+1; //
        }
        s_mixMaxSel =sel;
        mtIdx++;
    }
}

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

void moveMix(uint8_t idx,uint8_t mcopy, uint8_t dir) //true=inc=down false=dec=up - Issue 49
{
    if(idx>MAX_MIXERS || (idx==0 && !dir) || (idx==MAX_MIXERS && dir)) return;
    uint8_t tdx = dir ? idx+1 : idx-1;
    MixData *src= &g_model.mixData[idx];
    MixData *tgt= &g_model.mixData[tdx];

    if((src->destCh==0) || (src->destCh>NUM_CHNOUT) || (tgt->destCh>NUM_CHNOUT)) return;

    if(mcopy) {

        if (reachMixerCountLimit())
        {
            return;
        }

        memmove(&g_model.mixData[idx+1],&g_model.mixData[idx],
                (MAX_MIXERS-(idx+1))*sizeof(MixData) );
        return;
    }

    if(tgt->destCh!=src->destCh) {
        if ((dir)  && (src->destCh<NUM_CHNOUT)) src->destCh++;
        if ((!dir) && (src->destCh>0))          src->destCh--;
        return;
    }

    //flip between idx and tgt
    memswap( tgt, src, sizeof(MixData) ) ;
    STORE_MODELVARS;
    eeWaitComplete() ;

}

void menuMixersLimit(uint8_t event)
{
    lcd_puts_Pleft(2*FH, PSTR("Max mixers reach: "));
    lcd_outdezAtt(20*FW, 2*FH, getMixerCount(),0);

    lcd_puts_Pleft(4*FH, PSTR("Press [EXIT] to abort"));
    switch(event)
    {
    case  EVT_KEY_FIRST(KEY_EXIT):
        killEvents(event);
        popMenu(true);
        pushMenu(menuProcMix);
        break;
    }
}

uint8_t getMixerCount()
{
    uint8_t mixerCount = 0;
    uint8_t dch ;

    for(uint8_t i=0;i<MAX_MIXERS;i++)
    {
        dch = g_model.mixData[i].destCh ;
        if ((dch!=0) && (dch<=NUM_CHNOUT))
        {
            mixerCount++;
        }
    }
    return mixerCount;
}

bool reachMixerCountLimit()
{
    // check mixers count limit
    if (getMixerCount() >= MAX_MIXERS)
    {
        pushMenu(menuMixersLimit);
        return true;
    }
    else
    {
        return false;
    }
}

void menuProcMix(uint8_t event)
{
    SIMPLE_MENU("MIXER", menuTabModel, e_Mix, s_mixMaxSel);

    int8_t  sub    = mstate2.m_posVert;
    static uint8_t s_moveMode;
//    static uint8_t s_sourceMoveMix;			// Not used
    static uint8_t s_copyMix;
    if(sub==0) s_moveMode = false;

    MixData *md=g_model.mixData;
    switch(event)
    {
    case EVT_ENTRY:
//        s_sourceMoveMix=0;			// Not used
        s_copyMix=true;
        s_moveMode=false;
    case EVT_ENTRY_UP:
        genMixTab();
        break;
    case EVT_KEY_FIRST(KEY_MENU):
        if(sub>=1) s_moveMode = !s_moveMode;
        s_copyMix = s_moveMode;
        if(s_currMixInsMode) s_moveMode = false; //cant have edit mode if mix not selected
        break;
    case EVT_KEY_REPT(KEY_UP):
    case EVT_KEY_FIRST(KEY_UP):
        s_copyMix = false;  //only copy if going down
    case EVT_KEY_REPT(KEY_DOWN):
    case EVT_KEY_FIRST(KEY_DOWN):
        if(s_moveMode) {
            //killEvents(event);
            moveMix(s_currMixIdx,s_copyMix,event==EVT_KEY_FIRST(KEY_DOWN)); //true=inc=down false=dec=up
            if(s_copyMix && event==EVT_KEY_FIRST(KEY_UP)) sub--;
            s_copyMix = false;
            genMixTab();
        }
        break;
    case EVT_KEY_FIRST(KEY_EXIT):
        if(s_moveMode) {
            deleteMix(s_currMixIdx);
            s_moveMode = false;
            genMixTab();
            killEvents(event);
        }
        break;
    case EVT_KEY_LONG(KEY_MENU):
        if(sub<1) break;
        if (s_currMixInsMode && !reachMixerCountLimit())
        {
            insertMix(s_currMixIdx);
        }
        s_moveMode=false;
        pushMenu(menuProcMixOne);
        break;
    }

    int8_t markedIdx=-1;
    uint8_t i;
    int8_t minSel=99;
    int8_t maxSel=-1;
    //  lcd_puts_P( 6*FW, 1*FH,PSTR("wt src  sw crv"));
    for(i=0; i<7; i++){
			struct MixTab *mixtabptr ;
        uint8_t y = i * FH + 1*FH;
        uint8_t k = i + s_pgOfs;
				mixtabptr = &s_mixTab[k] ;
        if(! mixtabptr->showCh && !mixtabptr->hasDat ) break;

        if(mixtabptr->showCh){
            putsChn(0,y,mixtabptr->chId, 0); // show CHx
        }
        if(sub>0 && sub==mixtabptr->selCh) { //handle CHx is selected (other line)
            //if(BLINK_ON_PHASE)
            lcd_hline(0,y+7,FW*4);
            s_currMixIdx     = mixtabptr->insIdx;
            s_currDestCh     = mixtabptr->chId;
            s_currMixInsMode = true;
            markedIdx        = i;
        }
        if(mixtabptr->hasDat){ //show data
            MixData *md2=&md[mixtabptr->editIdx];
            uint8_t attr = sub==mixtabptr->selDat ? INVERS : 0;
            if(!mixtabptr->showCh) //show prefix only if not first mix
                lcd_putsnAtt(   3*FW, y, PSTR("+*R")+1*md2->mltpx,1,s_moveMode ? attr : 0);
            lcd_outdezAtt(  7*FW+FW/2, y, md2->weight,attr);
            lcd_putcAtt(    7*FW+FW/2, y, '%',s_moveMode ? attr : 0);
            putsChnRaw(     9*FW, y, md2->srcRaw,s_moveMode ? attr : 0);
            if(md2->swtch)putsDrSwitches( 13*FW, y, md2->swtch,s_moveMode ? attr : 0);
            if(md2->curve)lcd_putsnAtt(   17*FW, y, get_curve_string()+md2->curve*3,3,s_moveMode ? attr : 0);

            bool bs = (md2->speedDown || md2->speedUp);
            bool bd = (md2->delayUp || md2->delayDown);
            char cs = (bs && bd) ? '*' : (bs ? 'S' : 'D');
            if(bs || bd) lcd_putcAtt(20*FW+1, y, cs,s_editMode ? attr : 0);

            if(attr == INVERS){ //handle dat is selected
                CHECK_INCDEC_H_MODELVAR( event, md2->weight, -125,125);
                s_currMixIdx     = mixtabptr->editIdx;
                s_currDestCh     = mixtabptr->chId;
                s_currMixInsMode = false;
                markedIdx        = i;
//                if(!s_moveMode) s_sourceMoveMix = k;			// Not used
            }
        }
        //welche sub-indize liegen im sichtbaren bereich?
        if(mixtabptr->selCh){
            minSel = min(minSel,mixtabptr->selCh);
            maxSel = max(maxSel,mixtabptr->selCh);
        }
        if(mixtabptr->selDat){
            minSel = min(minSel,mixtabptr->selDat);
            maxSel = max(maxSel,mixtabptr->selDat);
        }

    } //for 7
    if( sub!=0 &&  markedIdx==-1) { //versuche die Marke zu finden
        if(sub < minSel) s_pgOfs = max(0,s_pgOfs-1);
        if(sub > maxSel) s_pgOfs++;
    }
    else if(markedIdx<0)              s_pgOfs = max(0,s_pgOfs-1);
    else if(markedIdx>=6 && i>=8)      s_pgOfs++;

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

static uint8_t s_expoChan;

void editExpoVals(uint8_t event,uint8_t stopBlink,uint8_t editMode, uint8_t edit,uint8_t x, uint8_t y, uint8_t chn, uint8_t which, uint8_t exWt, uint8_t stkRL)
{
    uint8_t  invBlk = edit ? (editMode ? BLINK : INVERS) : 0;
		uint8_t doedit ;
		volatile int8_t *ptr ;			// volatile forces compiler to produce 'better' code

		doedit = (edit && (editMode || p1valdiff)) ;
    if(edit && stopBlink) invBlk = INVERS;

    if(which==DR_DRSW1) {
				ptr = &g_model.expoData[chn].drSw1 ;
        putsDrSwitches(x,y,*ptr,invBlk);
        if(doedit) CHECK_INCDEC_H_MODELVAR(event,*ptr,-MAX_DRSWITCH,MAX_DRSWITCH);
    }
    else if(which==DR_DRSW2) {
				ptr = &g_model.expoData[chn].drSw2 ;
        putsDrSwitches(x,y,*ptr,invBlk);
        if(doedit) CHECK_INCDEC_H_MODELVAR(event,*ptr,-MAX_DRSWITCH,MAX_DRSWITCH);
    }
    else
        if(exWt==DR_EXPO){
						ptr = &g_model.expoData[chn].expo[which][exWt][stkRL] ;
            lcd_outdezAtt(x, y, *ptr, invBlk);
            if(doedit) CHECK_INCDEC_H_MODELVAR(event,*ptr,-100, 100);
        }
        else {
						ptr = &g_model.expoData[chn].expo[which][exWt][stkRL] ;
            lcd_outdezAtt(x, y, *ptr+100, invBlk);
            if(doedit) CHECK_INCDEC_H_MODELVAR(event,g_model.expoData[chn].expo[which][exWt][stkRL],-100, 0);
        }
}

void menuProcExpoOne(uint8_t event)
{
    SUBMENU("EXPO/DR", 4, {0});

static uint8_t stkVal;
putsChnRaw(8*FW,0,s_expoChan+1,0);
int8_t  sub    = mstate2.m_posVert;

uint8_t expoDrOn = GET_DR_STATE(s_expoChan);
uint8_t  y = 16;

if(calibratedStick[s_expoChan]> 25) stkVal = DR_RIGHT;
if(calibratedStick[s_expoChan]<-25) stkVal = DR_LEFT;
if(IS_THROTTLE(s_expoChan) && g_model.thrExpo) stkVal = DR_RIGHT;

lcd_puts_Pleft(y,PSTR("Expo"));
editExpoVals(event,true,true,sub==0,9*FW, y,s_expoChan, expoDrOn ,DR_EXPO,stkVal);
y+=FH;
lcd_puts_Pleft(y,PSTR("Weight"));
editExpoVals(event,true,true,sub==1,9*FW, y,s_expoChan, expoDrOn ,DR_WEIGHT,stkVal);
y+=FH;
lcd_puts_Pleft(y,PSTR("DrSw1"));
editExpoVals(event,true,true,sub==2,5*FW, y,s_expoChan, DR_DRSW1 , 0,0);
y+=FH;
lcd_puts_Pleft(y,PSTR("DrSw2"));
editExpoVals(event,true,true,sub==3,5*FW, y,s_expoChan, DR_DRSW2 , 0,0);
y+=FH;
switch (expoDrOn) {
    case DR_MID:
        lcd_puts_Pleft(y,PSTR("DR Mid"));
        break;
    case DR_LOW:
        lcd_puts_Pleft(y,PSTR("DR Low"));
        break;
    default: // DR_HIGH:
        lcd_puts_Pleft(y,PSTR("DR High"));
        break;
}
y+=FH;


int8_t   kViewR  = g_model.expoData[s_expoChan].expo[expoDrOn][DR_EXPO][DR_RIGHT];  //NormR;
int8_t   kViewL  = g_model.expoData[s_expoChan].expo[expoDrOn][DR_EXPO][DR_LEFT];  //NormL;
int8_t   wViewR  = g_model.expoData[s_expoChan].expo[expoDrOn][DR_WEIGHT][DR_RIGHT]+100;  //NormWeightR+100;
int8_t   wViewL  = g_model.expoData[s_expoChan].expo[expoDrOn][DR_WEIGHT][DR_LEFT]+100;  //NormWeightL+100;

#define WE_CHART	(WCHART-1)
#define WE_CHARTl	(WCHARTl-1)

if (IS_THROTTLE(s_expoChan) && g_model.thrExpo)
for(uint8_t xv=0;xv<WE_CHARTl*2;xv++)
{
    uint16_t yv=2*expo(xv*(RESXu/WE_CHARTl)/2,kViewR) / (RESXu/WE_CHARTl);
    yv = (yv * wViewR)/100;
    lcd_plot(X0l+xv-WE_CHARTl, 2*Y0l-yv);
    if((xv&3) == 0){
        lcd_plot(X0l+xv-WE_CHARTl, 2*Y0l-1);
        lcd_plot(X0l-WE_CHARTl   , Y0l+xv/2);
    }
}
else
for(uint8_t xv=0;xv<WE_CHARTl;xv++)
{
    uint16_t yv=expo(xv*(RESXu/WE_CHARTl),kViewR) / (RESXu/WE_CHARTl);
    yv = (yv * wViewR)/100;
    lcd_plot(X0l+xv, Y0l-yv);
    if((xv&3) == 0){
        lcd_plot(X0l+xv, Y0l+0);
        lcd_plot(X0l  , Y0l+xv);
    }

    yv=expo(xv*(RESXu/WE_CHARTl),kViewL) / (RESXu/WE_CHARTl);
    yv = (yv * wViewL)/100;
    lcd_plot(X0l-xv, Y0l+yv);
    if((xv&3) == 0){
        lcd_plot(X0l-xv, Y0l+0);
        lcd_plot(X0l  , Y0l-xv);
    }
}


int32_t x512  = calibratedStick[s_expoChan];

lcd_vline(X0l+x512/(int32_t)(RESXu/WE_CHARTl), Y0l-WE_CHARTl,WE_CHARTl*2);

int32_t y512 = 0;
if (IS_THROTTLE(s_expoChan) && g_model.thrExpo) {
    y512  = 2*expo((x512+RESX)/2,kViewR);
    y512 = y512 * (wViewR / 4)/(100 / 4);
    lcd_hline(X0l-WE_CHARTl, 2*Y0l-y512/(RESXu/WE_CHARTl),WE_CHARTl*2);
    y512 /= 2;
}
else {
y512  = expo(x512,(x512>0 ? kViewR : kViewL));
y512 = y512 * ((x512>0 ? wViewR : wViewL) / 4)/(100 / 4);

lcd_hline(X0l-WE_CHARTl, Y0l-y512/(int32_t)(RESXu/WE_CHARTl),WE_CHARTl*2);
}

lcd_outdezAtt( 19*FW, 6*FH,x512*25/((signed) RESXu/4), 0 );
lcd_outdezAtt( 14*FW, 1*FH,y512*25/((signed) RESXu/4), 0 );
//dy/dx
int16_t dy  = x512>0 ? y512-expo(x512-20,(x512>0 ? kViewR : kViewL)) : expo(x512+20,(x512>0 ? kViewR : kViewL))-y512;
lcd_outdezNAtt(14*FW, 2*FH,   dy*(100/20), LEADING0|PREC2,3);
}

void menuProcExpoAll(uint8_t event)
{
    MENU("EXPO/DR", menuTabModel, e_ExpoAll, 5, {0, 3/*repeated*/});

static uint8_t stkVal[4];
int8_t  sub    = mstate2.m_posVert - 1;
int8_t  subHor = mstate2.m_posHorz;

switch(event)
{
    case EVT_KEY_LONG(KEY_MENU):
        if(sub>=0){
            s_expoChan = sub;
            pushMenu(menuProcExpoOne);
        }
        break;
}

lcd_puts_P( 4*FW-FW/2, 1*FH,PSTR("exp  %  sw1 sw2"));
for(uint8_t i=0; i<4; i++)
{
    uint8_t expoDrOn = GET_DR_STATE(i);
    uint8_t valsEqual = (g_model.expoData[i].expo[expoDrOn][DR_WEIGHT][DR_LEFT]==g_model.expoData[i].expo[expoDrOn][DR_WEIGHT][DR_RIGHT]) &&
            (g_model.expoData[i].expo[expoDrOn][DR_EXPO][DR_LEFT]==g_model.expoData[i].expo[expoDrOn][DR_EXPO][DR_RIGHT]);
    uint8_t stickCentred = (abs(calibratedStick[i])<=25) && valsEqual;
    if(calibratedStick[i]> 25) stkVal[i] = DR_RIGHT;
    if(calibratedStick[i]<-25) stkVal[i] = DR_LEFT;
    if(IS_THROTTLE(i) && g_model.thrExpo) {
        stkVal[i] = DR_RIGHT;
        stickCentred = true;
    }

    uint8_t y=(i+2)*FH;
    putsChnRaw( 0, y,i+1,0);
    uint8_t stkOp = (stkVal[i] == DR_RIGHT) ? DR_LEFT : DR_RIGHT;

    uint8_t edtm = (s_editMode || p1valdiff);
    editExpoVals(event,false,edtm,sub==i && subHor==0, 7*FW-FW/2, y,i,expoDrOn,DR_EXPO,stkVal[i]);
    if(sub==i && subHor==0 && edtm && stickCentred)
        CHECK_INCDEC_H_MODELVAR(event,g_model.expoData[i].expo[expoDrOn][DR_EXPO][stkOp],-100, 100);

    editExpoVals(event,false,edtm,sub==i && subHor==1, 9*FW+FW/2, y,i,expoDrOn,DR_WEIGHT,stkVal[i]);
    if(sub==i && subHor==1 && edtm && stickCentred)
        CHECK_INCDEC_H_MODELVAR(event,g_model.expoData[i].expo[expoDrOn][DR_WEIGHT][stkOp],-100, 0);

    editExpoVals(event,false,edtm,sub==i && subHor==2,10*FW+FW/2, y,i,DR_DRSW1,0,0);
    editExpoVals(event,false,edtm,sub==i && subHor==3,14*FW+FW/2, y,i,DR_DRSW2,0,0);
    lcd_putc(9*FW+FW/2 + ((!stkVal[i] && !stickCentred) ? 2 : 1 ), y, stickCentred ? '-' : (stkVal[i] ? 127 : 126));//'|' : (stkVal[i] ? '<' : '>'),0);
    switch (expoDrOn) {
    case DR_MID:
        lcd_putc(19*FW+FW/2,y,'M');
        break;
    case DR_LOW:
        lcd_putc(19*FW+FW/2,y,'L');
        break;
    default: // DR_HIGH:
        lcd_putc(19*FW+FW/2,y,'H');
        break;
    }
}
}

uint8_t char2idx(char c)
{
	int8_t ret ;
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
//void menuDeleteModel(uint8_t event)
{
    lcd_puts_Pleft(1*FH,DupIfNonzero ? PSTR("DUPLICATE MODEL") : PSTR("DELETE MODEL"));
    //  lcd_putsAtt(0,1*FH,PSTR("DELETE MODEL"),0);
    lcd_putsnAtt(1,2*FH, g_model.name,sizeof(g_model.name),/*BSS*/0);
    lcd_putc(sizeof(g_model.name)*FW+FW,2*FH,'?');
    lcd_puts_P(3*FW,5*FH,PSTR("YES     NO"));
    lcd_puts_P(3*FW,6*FH,PSTR("[MENU]  [EXIT]"));

    uint8_t i;
    switch(event){
    case EVT_ENTRY:
        audioDefevent(AUDIO_WARNING1);
        break;
    case EVT_KEY_FIRST(KEY_MENU):
        if ( DupIfNonzero )
        {
            message(PSTR("Duplicating model"));
            if(eeDuplicateModel(DupSub))
            {
                audioDefevent(AUDIO_MENUS);
                DupIfNonzero = 2 ;		// sel_editMode = false;
            }
            else audioDefevent(AUDIO_WARNING1);
        }
        else
        {
            EFile::rm(FILE_MODEL(g_eeGeneral.currModel)); //delete file

            i = g_eeGeneral.currModel;//loop to find next available model
            while (!EFile::exists(FILE_MODEL(i))) {
                i--;
                if(i>MAX_MODELS) i=MAX_MODELS-1;
                if(i==g_eeGeneral.currModel) {
                    i=0;
                    break;
                }
            }
            g_eeGeneral.currModel = i;
            STORE_GENERALVARS;
            eeWaitComplete() ;
            eeLoadModel(g_eeGeneral.currModel); //load default values
            resetTimer();
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

void menuProcModel(uint8_t event)
{
  MENU("SETUP", menuTabModel, e_Model, 17, {0,sizeof(g_model.name)-1,1,0,0,0,0,0,0,6,2,0/*repeated...*/});
//  MENU("SETUP", menuTabModel, e_Model, 16, {0,sizeof(g_model.name)-1,1,0,0,0,0,0,0,6,2,0/*repeated...*/});

	int8_t  sub    = mstate2.m_posVert;
	uint8_t subSub = mstate2.m_posHorz;

	evalOffset(sub, 7);

	uint8_t y = 1*FH;

	lcd_outdezNAtt(7*FW,0,g_eeGeneral.currModel+1,INVERS+LEADING0,2);

	switch(event)
	{
    case EVT_KEY_REPT(KEY_LEFT):
    case EVT_KEY_FIRST(KEY_LEFT):
      if(sub==1 && subSub>0 && s_editMode) mstate2.m_posHorz--;
    break;
    case EVT_KEY_REPT(KEY_RIGHT):
    case EVT_KEY_FIRST(KEY_RIGHT):
      if(sub==1 && subSub<sizeof(g_model.name)-1 && s_editMode) mstate2.m_posHorz++;
    break;
	}

	uint8_t subN = 1;
	if(s_pgOfs<subN)
	{
    lcd_puts_Pleft(    y, PSTR("Name"));
    lcd_putsnAtt(10*FW,   y, g_model.name ,sizeof(g_model.name),/*BSS | */(sub==subN ? (s_editMode ? 0 : INVERS) : 0));
    if(!s_editMode && scrollLR<0) { s_editMode = true; scrollLR = 0; mstate2.m_posHorz = 0; }
    if(s_editMode && scrollLR>subSub) { s_editMode = false; scrollLR = 0; mstate2.m_posHorz = 0; }

    if(sub==subN && s_editMode){
        mstate2.m_posHorz -= scrollLR;
        if((int8_t(mstate2.m_posHorz))<0) mstate2.m_posHorz = 0;
        if((int8_t(mstate2.m_posHorz))>(MODEL_NAME_LEN-1)) mstate2.m_posHorz = MODEL_NAME_LEN-1;
        scrollLR = 0;

        char v = char2idx(g_model.name[subSub]);
        if(p1valdiff || event==EVT_KEY_FIRST(KEY_DOWN) || event==EVT_KEY_FIRST(KEY_UP) || event==EVT_KEY_REPT(KEY_DOWN) || event==EVT_KEY_REPT(KEY_UP))
            CHECK_INCDEC_H_MODELVAR( event,v ,0,NUMCHARS-1);
        v = idx2char(v);
        g_model.name[subSub]=v;
        lcd_putcAtt((10+subSub)*FW, y, v,INVERS);
    }
    if((y+=FH)>7*FH) return;
	}subN++;

	if(s_pgOfs<subN)
	{
    lcd_puts_Pleft(    y, PSTR("Timer"));
    putsTime(12*FW-1, y, g_model.tmrVal,(sub==subN && subSub==0 ? (s_editMode ? BLINK : INVERS):0),(sub==subN && subSub==1 ? (s_editMode ? BLINK : INVERS):0) );

    if(sub==subN && (s_editMode || p1valdiff))
		{
      switch (subSub)
			{
        case 0:
        {
            int8_t min=g_model.tmrVal/60;
            CHECK_INCDEC_H_MODELVAR( event,min ,0,59);
            g_model.tmrVal = g_model.tmrVal%60 + min*60;
            break;
        }
        case 1:
        {
            int8_t sec=g_model.tmrVal%60;
            sec -= checkIncDec_hm( event,sec+2 ,1,62)-2;
            g_model.tmrVal -= sec ;
            if((int16_t)g_model.tmrVal < 0) g_model.tmrVal=0;
            break;
        }
      }
		}
    if((y+=FH)>7*FH) return;
	}subN++;

if(s_pgOfs<subN) { //timer trigger source -> off, abs, stk, stk%, sw/!sw, !m_sw/!m_sw, chx(value > or < than tmrChVal), ch%
    lcd_puts_Pleft(    y, PSTR("Trigger"));
    uint8_t attr = (sub==subN ?  INVERS : 0);
    putsTmrMode(10*FW,y,attr);

    if(sub==subN)
        CHECK_INCDEC_H_MODELVAR( event,g_model.tmrMode ,-(13+2*MAX_DRSWITCH),(13+2*MAX_DRSWITCH));
    if((y+=FH)>7*FH) return;
}subN++;

if(s_pgOfs<subN) {
    lcd_puts_Pleft(    y, PSTR("Timer "));
    lcd_putsnAtt(  10*FW, y, PSTR("Count DownCount Up  ")+10*g_model.tmrDir,10,(sub==subN ? INVERS:0));
    if(sub==subN) CHECK_INCDEC_H_MODELVAR(event,g_model.tmrDir,0,1);
    if((y+=FH)>7*FH) return;
}subN++;

if(s_pgOfs<subN) {
    uint8_t b ;
    b = g_model.thrTrim ;
    lcd_puts_Pleft(    y, PSTR("T-Trim"));
    menu_lcd_onoff( 10*FW, y, b, sub==subN ) ;
    if(sub==subN) { CHECK_INCDEC_H_MODELVAR(event,b,0,1); g_model.thrTrim = b ; }
    if((y+=FH)>7*FH) return;
}subN++;

if(s_pgOfs<subN) {
    uint8_t b ;
    b = g_model.thrExpo ;
    lcd_puts_Pleft(    y, PSTR("T-Expo"));
    menu_lcd_onoff( 10*FW, y, b, sub==subN ) ;
    if(sub==subN) { CHECK_INCDEC_H_MODELVAR(event,b,0,1); g_model.thrExpo = b ; }
    if((y+=FH)>7*FH) return;
}subN++;

if(s_pgOfs<subN) {
    lcd_puts_Pleft(    y, PSTR("Trim Inc"));
    lcd_putsnAtt(  10*FW, y, PSTR("Exp   ExFineFine  MediumCoarse")+6*g_model.trimInc,6,(sub==subN ? INVERS:0));
    if(sub==subN) CHECK_INCDEC_H_MODELVAR(event,g_model.trimInc,0,4);
    if((y+=FH)>7*FH) return;
}subN++;

if(s_pgOfs<subN) {
    lcd_puts_Pleft(    y, PSTR("Trim Sw"));
    putsDrSwitches(9*FW,y,g_model.trimSw,sub==subN ? INVERS:0);
    if(sub==subN) CHECK_INCDEC_H_MODELVAR(event,g_model.trimSw,-MAX_DRSWITCH, MAX_DRSWITCH);
    if((y+=FH)>7*FH) return;
}subN++;

if(s_pgOfs<subN) {
    lcd_puts_Pleft(    y, PSTR("Beep Cnt"));
    for(uint8_t i=0;i<7;i++) lcd_putsnAtt((10+i)*FW, y, PSTR("RETA123")+i,1, (((subSub)==i) && (sub==subN)) ? BLINK : ((g_model.beepANACenter & (1<<i)) ? INVERS : 0 ) );
    if(sub==subN){
        if((event==EVT_KEY_FIRST(KEY_MENU)) || p1valdiff) {
            killEvents(event);
            s_editMode = false;
            g_model.beepANACenter ^= (1<<(subSub));
            STORE_MODELVARS;
            eeWaitComplete() ;
        }
    }
    if((y+=FH)>7*FH) return;
}subN++;

if(s_pgOfs<subN) {
    lcd_puts_Pleft(    y, PSTR("Proto"));//sub==2 ? INVERS:0);
    lcd_putsnAtt(  6*FW, y, PSTR(PROT_STR)+PROT_STR_LEN*g_model.protocol,PROT_STR_LEN,(sub==subN && subSub==0 ? (s_editMode ? BLINK : INVERS):0));
    if( g_model.protocol == PROTO_PPM ) {
        lcd_putsnAtt(  10*FW, y, PSTR("4CH 6CH 8CH 10CH12CH14CH16CH")+4*(g_model.ppmNCH+2),4,(sub==subN && subSub==1  ? (s_editMode ? BLINK : INVERS):0));
        lcd_putsAtt(    17*FW,    y, PSTR("uSec"),0);
        lcd_outdezAtt(  17*FW, y,  (g_model.ppmDelay*50)+300, (sub==subN && subSub==2 ? (s_editMode ? BLINK : INVERS):0));
    }
    if (g_model.protocol == PROTO_PPM16)
    {
        lcd_putsAtt(    15*FW,    y, PSTR("uSec"),0);
        lcd_outdezAtt(  15*FW, y,  (g_model.ppmDelay*50)+300, (sub==subN && subSub==1 ? (s_editMode ? BLINK : INVERS):0));
    }
    if (g_model.protocol == PROTO_PXX)
    {
        lcd_putsAtt(    11*FW,    y, PSTR(" RxNum"),0);
        lcd_outdezAtt(  21*FW, y,  g_model.rxnum+1, (sub==subN && subSub==1 ? (s_editMode ? BLINK : INVERS):0));
    }

    if(sub==subN && (s_editMode || p1valdiff))
        switch (subSub){
        case 0:
            CHECK_INCDEC_H_MODELVAR(event,g_model.protocol,0,PROT_MAX);
            break;
        case 1:
            if (g_model.protocol == PROTO_PPM)
                CHECK_INCDEC_H_MODELVAR(event,g_model.ppmNCH,-2,4);
            else if (g_model.protocol == PROTO_PPM16)
                CHECK_INCDEC_H_MODELVAR(event,g_model.ppmDelay,-4,10);
            else if (g_model.protocol == PROTO_PXX)
                CHECK_INCDEC_H_MODELVAR(event,g_model.rxnum,0,124);
            break;
        case 2:
            if (g_model.protocol == PROTO_PPM)
                CHECK_INCDEC_H_MODELVAR(event,g_model.ppmDelay,-4,10);
            break;
        }
    if((y+=FH)>7*FH) return;
}subN++;

if(s_pgOfs<subN) {
    if(g_model.protocol == PROTO_PPM || g_model.protocol == PROTO_PPM16)
    {
        lcd_puts_Pleft(    y, PSTR("PPM FrLen    mSec"));
        lcd_outdezAtt(  13*FW, y, (int16_t)g_model.ppmFrameLength*5 + 225 ,(sub==subN ? INVERS:0) | PREC1);
        if(sub==subN) CHECK_INCDEC_H_MODELVAR(event,g_model.ppmFrameLength,-20,20);
    }
    else if(g_model.protocol == PROTO_PXX)
    {
        lcd_putsAtt(0,    y, PSTR("Send Rx Number [MENU]"), (sub==subN ? INVERS:0));

        if(sub==subN && event==EVT_KEY_LONG(KEY_MENU))
        {
            //send reset code
            pxxFlag = PXX_SEND_RXNUM;
        }
    }
    else
    {
        lcd_puts_Pleft(    y, PSTR("DSM Type"));
        int8_t x ;
        x = g_model.ppmNCH ;
        if ( x < 0 ) x = 0 ;
        if ( x > 2 ) x = 2 ;
        g_model.ppmNCH = x ;
        lcd_putsnAtt(10*FW,y, PSTR(DSM2_STR)+DSM2_STR_LEN*(x),DSM2_STR_LEN, (sub==subN ? (s_editMode ? BLINK : INVERS):0));
        if(sub==subN) CHECK_INCDEC_H_MODELVAR(event,g_model.ppmNCH,0,2);
    }
    if((y+=FH)>7*FH) return;
}subN++;

if(s_pgOfs<subN) {
    lcd_puts_Pleft(    y, PSTR("Shift Sel"));
    lcd_putsnAtt(  10*FW, y, PSTR("POSNEG")+3*g_model.pulsePol,3,(sub==subN ? INVERS:0));
    if(sub==subN) CHECK_INCDEC_H_MODELVAR(event,g_model.pulsePol,0,1);
    if((y+=FH)>7*FH) return;
}subN++;

if(s_pgOfs<subN) {
    uint8_t b ;
    b = g_model.extendedLimits ;
    lcd_puts_Pleft(    y, PSTR("E. Limits"));
    menu_lcd_onoff( 10*FW, y, b, sub==subN ) ;
    if(sub==subN) { CHECK_INCDEC_H_MODELVAR(event,b,0,1); g_model.extendedLimits = b ; }
    if((y+=FH)>7*FH) return;
}subN++;

if(s_pgOfs<subN) {
    uint8_t b ;
    b = g_model.traineron ;
    lcd_puts_Pleft(    y, PSTR("Trainer"));
    menu_lcd_onoff( 10*FW, y, b, sub==subN ) ;
    if(sub==subN) { CHECK_INCDEC_H_MODELVAR(event,b,0,1); g_model.traineron = b ; }
    if((y+=FH)>7*FH) return;
}subN++;

if(s_pgOfs<subN) {
    uint8_t b ;
    b = g_model.t2throttle ;
    lcd_puts_Pleft(    y, PSTR("T2ThTrig"));
    menu_lcd_onoff( 10*FW, y, b, sub==subN ) ;
    if(sub==subN) { CHECK_INCDEC_H_MODELVAR(event,b,0,1); g_model.t2throttle = b ; }
    if((y+=FH)>7*FH) return;
}subN++;

if(s_pgOfs<subN) {
    lcd_putsAtt(0*FW, y, PSTR("DELETE MODEL   [MENU]"),s_noHi ? 0 : (sub==subN?INVERS:0));
    if(sub==subN && event==EVT_KEY_LONG(KEY_MENU)){
        s_editMode = false;
        s_noHi = NO_HI_LEN;
        killEvents(event);
        DupIfNonzero = 0 ;
        pushMenu(menuDeleteDupModel);
    }
    if((y+=FH)>7*FH) return;
}subN++;
}

#ifndef NO_HELI
void menuProcHeli(uint8_t event)
{
    MENU("HELI SETUP", menuTabModel, e_Heli, 7, {0 /*repeated*/});

int8_t  sub    = mstate2.m_posVert;

evalOffset(sub, 7);

uint8_t y = 1*FH;

uint8_t subN = 1;
    lcd_puts_Pleft(    y, PSTR("Swash Type"));
    lcd_putsnAtt(  14*FW, y, PSTR(SWASH_TYPE_STR)+6*g_model.swashType,6,(sub==subN ? INVERS:0));
    if(sub==subN) CHECK_INCDEC_H_MODELVAR(event,g_model.swashType,0,SWASH_TYPE_NUM);
    if((y+=FH)>7*FH) return;
subN++;

    lcd_puts_Pleft(    y, PSTR("Collective"));
    putsChnRaw(14*FW, y, g_model.swashCollectiveSource,  sub==subN ? INVERS : 0);
    if(sub==subN) CHECK_INCDEC_H_MODELVAR(event, g_model.swashCollectiveSource, 0, NUM_XCHNRAW);
    if((y+=FH)>7*FH) return;
subN++;

    lcd_puts_Pleft(    y, PSTR("Swash Ring"));
    lcd_outdezAtt(14*FW, y, g_model.swashRingValue,  LEFT|(sub==subN ? INVERS : 0));
    if(sub==subN) CHECK_INCDEC_H_MODELVAR(event, g_model.swashRingValue, 0, 100);
    if((y+=FH)>7*FH) return;
subN++;

    uint8_t b ;
    b = g_model.swashInvertELE ;
    lcd_puts_Pleft(    y, PSTR("ELE Direction"));
    menu_lcd_HYPHINV( 14*FW, y, b, sub==subN ) ;
    if(sub==subN) { CHECK_INCDEC_H_MODELVAR(event, b, 0, 1); g_model.swashInvertELE = b ; }
    if((y+=FH)>7*FH) return;
subN++;

    b = g_model.swashInvertAIL ;
    lcd_puts_Pleft(    y, PSTR("AIL Direction"));
    menu_lcd_HYPHINV( 14*FW, y, b, sub==subN ) ;
    if(sub==subN) { CHECK_INCDEC_H_MODELVAR(event, b, 0, 1); g_model.swashInvertAIL = b ; }
    if((y+=FH)>7*FH) return;
subN++;

    b = g_model.swashInvertCOL ;
    lcd_puts_Pleft(    y, PSTR("COL Direction"));
    menu_lcd_HYPHINV( 14*FW, y, b, sub==subN ) ;
    if(sub==subN) { CHECK_INCDEC_H_MODELVAR(event,b, 0, 1); g_model.swashInvertCOL = b ; }
    if((y+=FH)>7*FH) return;

}
#endif

void menuProcModelSelect(uint8_t event)
{
  static MState2 mstate2;
  TITLE("MODELSEL");

  if(!s_editMode)
  {
    if(scrollLR<0)
    {
      uint8_t cc = -scrollLR;
     if(cc>(DIM(menuTabModel)-1)) cc = DIM(menuTabModel)-1;

     chainMenu((MenuFuncP)pgm_read_adr(&menuTabModel[cc]));

      scrollLR = 0;
    }
  }


  int8_t subOld  = mstate2.m_posVert;
  mstate2.check_submenu_simple(event,MAX_MODELS-1) ;

  lcd_puts_P(     9*FW, 0, PSTR("free"));
  lcd_outdezAtt(  17*FW, 0, EeFsGetFree(),0);

  DisplayScreenIndex(e_ModelSelect, DIM(menuTabModel), INVERS);

  int8_t  sub    = mstate2.m_posVert;
  static uint8_t sel_editMode;
  if ( DupIfNonzero == 2 )
  {
      sel_editMode = false ;
      DupIfNonzero = 0 ;
  }

  switch(event)
  {
  //case  EVT_KEY_FIRST(KEY_MENU):
  case  EVT_KEY_FIRST(KEY_EXIT):
      if(sel_editMode){
          sel_editMode = false;
          audioDefevent(AUDIO_MENUS);
          killEvents(event);
          eeWaitComplete();    // Wait to load model if writing something
          eeLoadModel(g_eeGeneral.currModel = mstate2.m_posVert);
          resetTimer();
          STORE_GENERALVARS;
          eeWaitComplete();
          STORE_MODELVARS;
          eeWaitComplete();
          break;
      }
      //fallthrough
  case  EVT_KEY_FIRST(KEY_LEFT):
  case  EVT_KEY_FIRST(KEY_RIGHT):
      if(g_eeGeneral.currModel != mstate2.m_posVert)
      {
          killEvents(event);
          g_eeGeneral.currModel = mstate2.m_posVert;
          eeWaitComplete();    // Wait to load model if writing something
          eeLoadModel(g_eeGeneral.currModel);
          resetTimer();
          STORE_GENERALVARS;
          eeWaitComplete();
          audioDefevent(AUDIO_WARNING2);
      }
#ifndef NO_TEMPLATES
      if(event==EVT_KEY_FIRST(KEY_LEFT))  chainMenu(menuProcTemplates);//{killEvents(event);popMenu(true);}
//#elif defined(FRSKY)
//      if(event==EVT_KEY_FIRST(KEY_LEFT))  chainMenu(menuProcTelemetry);//{killEvents(event);popMenu(true);}
#else
      if(event==EVT_KEY_FIRST(KEY_LEFT))  chainMenu(menuProcSafetySwitches);//{killEvents(event);popMenu(true);}
#endif
      if(event==EVT_KEY_FIRST(KEY_RIGHT)) chainMenu(menuProcModel);
      //      if(event==EVT_KEY_FIRST(KEY_EXIT))  chainMenu(menuProcModelSelect);
      break;
  case  EVT_KEY_FIRST(KEY_MENU):
      sel_editMode = true;
      audioDefevent(AUDIO_MENUS);
      break;
  case  EVT_KEY_LONG(KEY_EXIT):  // make sure exit long exits to main
      popMenu(true);
      break;
  case  EVT_KEY_LONG(KEY_MENU):
      if(sel_editMode){

          DupIfNonzero = 1 ;
          DupSub = sub ;
          pushMenu(menuDeleteDupModel);//menuProcExpoAll);

          //        message(PSTR("Duplicating model"));
          //        if(eeDuplicateModel(sub)) {
          //          audioDefevent(AUDIO_MENUS);
          //          sel_editMode = false;
          //        }
          //        else audioDefevent(AUDIO_WARNING1);
      }
      break;

  case EVT_ENTRY:
      sel_editMode = false;

      mstate2.m_posVert = g_eeGeneral.currModel;
      eeCheck(true); //force writing of current model data before this is changed
      break;
  }
  if(sel_editMode && subOld!=sub)
	{
    EFile::swap(FILE_MODEL(subOld),FILE_MODEL(sub));
  }

  if(sub-s_pgOfs < 1)        s_pgOfs = max(0,sub-1);
  else if(sub-s_pgOfs >4 )  s_pgOfs = min(MAX_MODELS-6,sub-4);
  for(uint8_t i=0; i<6; i++)
	{
    uint8_t y=(i+2)*FH;
    uint8_t k=i+s_pgOfs;
    lcd_outdezNAtt(  3*FW, y, k+1, ((sub==k) ? INVERS : 0) + LEADING0,2);
    static char buf[sizeof(g_model.name)+5];
    if(k==g_eeGeneral.currModel) lcd_putc(1,  y,'*');
    eeLoadModelName(k,buf,sizeof(buf));
    lcd_putsnAtt(  4*FW, y, buf,sizeof(buf),/*BSS|*/((sub==k) ? (sel_editMode ? INVERS : 0 ) : 0));
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

void menuProcDiagVers(uint8_t event)
{
    SIMPLE_MENU("VERSION", menuTabDiag, e_Vers, 1);

    lcd_puts_Pleft( 2*FH,stamp4 );
    lcd_puts_Pleft( 3*FH,stamp1 );
    lcd_puts_Pleft( 4*FH,stamp2 );
    lcd_puts_Pleft( 5*FH,stamp3 );
    lcd_puts_Pleft( 6*FH,stamp5 );
}

// From Bertrand, allow trainer inputs without using mixers.
// Raw trianer inputs replace raw sticks.
// Only first 4 PPMin may be calibrated.
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


#define PARAM_OFS   17*FW

uint8_t onoffMenuItem( uint8_t value, uint8_t y, const char *s, uint8_t sub, int8_t subN, uint8_t event )
{
  lcd_puts_Pleft(y, s);
  menu_lcd_onoff( PARAM_OFS, y, value, sub==subN ) ;
  if(sub==subN) CHECK_INCDEC_H_GENVAR(event, value, 0, 1);
  return value ;
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
	uint8_t vCountItems = 22 ; //21 is default
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
        lcd_puts_Pleft(    y, PSTR("Owner Name"));
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
      if(  p1valdiff || event==EVT_KEY_FIRST(KEY_DOWN) || event==EVT_KEY_FIRST(KEY_UP) || event==EVT_KEY_REPT(KEY_DOWN) || event==EVT_KEY_REPT(KEY_UP))
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
        lcd_puts_Pleft( y,PSTR("Beeper"));
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
        lcd_puts_Pleft( y,PSTR("Contrast"));
        lcd_outdezAtt(PARAM_OFS,y,g_eeGeneral.contrast,(sub==subN ? INVERS : 0)|LEFT);
        if(sub==subN) {
            CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.contrast, 10, 45);
            lcdSetRefVolt(g_eeGeneral.contrast);
        }
        if((y+=FH)>7*FH) return;
    }subN++;

    if(s_pgOfs<subN) {
        lcd_puts_Pleft( y,PSTR("Battery warning"));
        putsVolts(PARAM_OFS, y, g_eeGeneral.vBatWarn, (sub==subN ? INVERS : 0)|LEFT);
        if(sub==subN) CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.vBatWarn, 40, 120); //5-10V
        if((y+=FH)>7*FH) return;
    }subN++;

    if(s_pgOfs<subN) {
        lcd_puts_Pleft( y,PSTR("Inactivity alarm"));
        lcd_outdezAtt(PARAM_OFS, y, g_eeGeneral.inactivityTimer+10, (sub==subN ? INVERS : 0)|LEFT);
        lcd_putc(Lcd_lastPos, y, 'm');
        if(sub==subN) CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.inactivityTimer, -10, 110); //0..120minutes
        if((y+=FH)>7*FH) return;
    }subN++;

    if(s_pgOfs<subN) {
        lcd_puts_Pleft( y,PSTR("Filter ADC"));
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
        lcd_puts_Pleft( y,PSTR("Light switch"));
        putsDrSwitches(PARAM_OFS-FW,y,g_eeGeneral.lightSw,sub==subN ? INVERS : 0);
        if(sub==subN) CHECK_INCDEC_H_GENVAR(event, g_eeGeneral.lightSw, -MAX_DRSWITCH, MAX_DRSWITCH);
        if((y+=FH)>7*FH) return;
    }subN++;

		for ( uint8_t i = 0 ; i < 2 ; i += 1 )
		{
    	if(s_pgOfs<subN) {
					uint8_t b ;
    	    lcd_puts_Pleft( y,( i == 0) ? PSTR("Light off after") : PSTR("Light on Stk Mv") );
					b = ( i == 0 ) ? g_eeGeneral.lightAutoOff : g_eeGeneral.lightOnStickMove ;

    	    if(b) {
    	        lcd_outdezAtt(PARAM_OFS, y, b*5,LEFT|(sub==subN ? INVERS : 0));
    	        lcd_putc(Lcd_lastPos, y, 's');
    	    }
    	    else
    	        lcd_putsnAtt(PARAM_OFS, y, PSTR("OFF"),3,(sub==subN ? INVERS:0));
    	    if(sub==subN) CHECK_INCDEC_H_GENVAR(event, b, 0, 600/5);
					if ( i == 0 )
					{
						g_eeGeneral.lightAutoOff = b ;
					}
					else
					{
						g_eeGeneral.lightOnStickMove = b ;
					}
    	    if((y+=FH)>7*FH) return;
    	}subN++;
			
		}

    if(s_pgOfs<subN) {
        uint8_t b = 1-g_eeGeneral.disableSplashScreen;
        g_eeGeneral.disableSplashScreen = 1-onoffMenuItem( b, y, PSTR("Splash screen"), sub, subN, event ) ;
        if((y+=FH)>7*FH) return;
    }subN++;

    if(s_pgOfs<subN) {
        uint8_t b = 1-g_eeGeneral.hideNameOnSplash;
        g_eeGeneral.hideNameOnSplash = 1-onoffMenuItem( b, y, PSTR("Splash Name"), sub, subN, event ) ;
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
        lcd_puts_Pleft(    y, PSTR("Default Sw"));
        for(uint8_t i=0;i<8;i++) lcd_putsnAtt((11+i)*FW, y, PSTR("TRE012AG")+i,1,  ((g_eeGeneral.switchWarningStates & (1<<i)) ? INVERS : 0 ) );

        if(sub==subN){
            mstate2.m_posHorz -= scrollLR;
            if((int8_t(mstate2.m_posHorz))<0) mstate2.m_posHorz = 0;
            if((int8_t(mstate2.m_posHorz))>(GENERAL_OWNER_NAME_LEN-1)) mstate2.m_posHorz = GENERAL_OWNER_NAME_LEN-1;
            scrollLR = 0;

            lcd_putsnAtt((11+subSub)*FW, y, PSTR("TRE012AG")+subSub,1,  BLINK );

            if((event==EVT_KEY_FIRST(KEY_MENU)) || p1valdiff) {
                killEvents(event);
                s_editMode = false;
                uint8_t p = (1<<(subSub));
                g_eeGeneral.switchWarningStates ^= p;

                switch (p)
                {
                case (SWP_ID0B):
                    if(g_eeGeneral.switchWarningStates & p) //if on - turn the others off
                        g_eeGeneral.switchWarningStates &= ~(SWP_ID1B | SWP_ID2B);
                    else
                        g_eeGeneral.switchWarningStates |= p; //else - turn on
                    break;
                case (SWP_ID1B):
                    if(g_eeGeneral.switchWarningStates & p) //if on - turn the others off
                        g_eeGeneral.switchWarningStates &= ~(SWP_ID0B | SWP_ID2B);
                    else
                        g_eeGeneral.switchWarningStates |= p; //else - turn on
                    break;
                case (SWP_ID2B):
                    if(g_eeGeneral.switchWarningStates & p) //if on - turn the others off
                        g_eeGeneral.switchWarningStates &= ~(SWP_ID0B | SWP_ID1B);
                    else
                        g_eeGeneral.switchWarningStates |= p; //else - turn on
                    break;
                }

                STORE_GENERALVARS;
                eeWaitComplete() ;
            }
        }

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

uint16_t s_timeCumTot;
uint16_t s_timeCumAbs;  //laufzeit in 1/16 sec
uint16_t s_timeCumSw;  //laufzeit in 1/16 sec
uint16_t s_timeCumThr;  //gewichtete laufzeit in 1/16 sec
uint16_t s_timeCum16ThrP; //gewichtete laufzeit in 1/16 sec
uint8_t  s_timerState;
#define TMR_OFF     0
#define TMR_RUNNING 1
#define TMR_BEEPING 2
#define TMR_STOPPED 3

int16_t  s_timerVal;
void timer(uint8_t val)
{
    int8_t tm = g_model.tmrMode;
    static uint16_t s_time;
    static uint16_t s_cnt;
    static uint16_t s_sum;
    static uint8_t sw_toggled;

    if(abs(tm)>=(TMR_VAROFS+MAX_DRSWITCH-1)){ //toggeled switch//abs(g_model.tmrMode)<(10+MAX_DRSWITCH-1)
        static uint8_t lastSwPos;
        if(!(sw_toggled | s_sum | s_cnt | s_time | lastSwPos)) lastSwPos = tm < 0;  // if initializing then init the lastSwPos
        uint8_t swPos = getSwitch(tm>0 ? tm-(TMR_VAROFS+MAX_DRSWITCH-1-1) : tm+(TMR_VAROFS+MAX_DRSWITCH-1-1) ,0);
        if(swPos && !lastSwPos)  sw_toggled = !sw_toggled;  //if switcdh is flipped first time -> change counter state
        lastSwPos = swPos;
    }

    s_cnt++;
    s_sum+=val;
    if(( get_tmr10ms()-s_time)<100) return; //1 sec
    s_time += 100;
    val     = s_sum/s_cnt;
    s_sum  -= val*s_cnt; //rest
    s_cnt   = 0;

    if(abs(tm)<TMR_VAROFS) sw_toggled = false; // not switch - sw timer off
    else if(abs(tm)<(TMR_VAROFS+MAX_DRSWITCH-1)) sw_toggled = getSwitch((tm>0 ? tm-(TMR_VAROFS-1) : tm+(TMR_VAROFS-1)) ,0); //normal switch

    s_timeCumTot               += 1;
    s_timeCumAbs               += 1;
    if(val) s_timeCumThr       += 1;
    if(sw_toggled) s_timeCumSw += 1;
    s_timeCum16ThrP            += val/2;

    s_timerVal = g_model.tmrVal;
    uint8_t tmrM = abs(g_model.tmrMode);
    if(tmrM==TMRMODE_NONE) s_timerState = TMR_OFF;
    else if(tmrM==TMRMODE_ABS) s_timerVal -= s_timeCumAbs;
    else if(tmrM<TMR_VAROFS) s_timerVal -= (tmrM&1) ? s_timeCum16ThrP/16 : s_timeCumThr;// stick% : stick
    else s_timerVal -= s_timeCumSw; //switch

    switch(s_timerState)
    {
    case TMR_OFF:
        if(g_model.tmrMode != TMRMODE_NONE) s_timerState=TMR_RUNNING;
        break;
    case TMR_RUNNING:
        if(s_timerVal<=0 && g_model.tmrVal) s_timerState=TMR_BEEPING;
        break;
    case TMR_BEEPING:
        if(s_timerVal <= -MAX_ALERT_TIME)   s_timerState=TMR_STOPPED;
        if(g_model.tmrVal == 0)             s_timerState=TMR_RUNNING;
        break;
    case TMR_STOPPED:
        break;
    }

    static int16_t last_tmr;

    if(last_tmr != s_timerVal)  //beep only if seconds advance
    {
        if(s_timerState==TMR_RUNNING)
        {
            if(g_eeGeneral.preBeep && g_model.tmrVal) // beep when 30, 15, 10, 5,4,3,2,1 seconds remaining
            {
            	

              	if(s_timerVal==30) {audioDefevent(AUDIO_TIMER_30);}	
              	if(s_timerVal==20) {audioDefevent(AUDIO_TIMER_20);}		
                if(s_timerVal==10) {audioDefevent(AUDIO_TIMER_10);}	
                if(s_timerVal<= 3) {audioDefevent(AUDIO_TIMER_LT3);}	               
                
                

                if(g_eeGeneral.flashBeep && (s_timerVal==30 || s_timerVal==20 || s_timerVal==10 || s_timerVal<=3))
                    g_LightOffCounter = FLASH_DURATION;
            }

            if(g_eeGeneral.minuteBeep && (((g_model.tmrDir ? g_model.tmrVal-s_timerVal : s_timerVal)%60)==0)) //short beep every minute
            {
                audioDefevent(AUDIO_WARNING1);
                if(g_eeGeneral.flashBeep) g_LightOffCounter = FLASH_DURATION;
            }
        }
        else if(s_timerState==TMR_BEEPING)
        {
            audioDefevent(AUDIO_TIMER_LT3);
            if(g_eeGeneral.flashBeep) g_LightOffCounter = FLASH_DURATION;
        }
    }
    last_tmr = s_timerVal;
    if(g_model.tmrDir) s_timerVal = g_model.tmrVal-s_timerVal; //if counting backwards - display backwards
}


#define MAXTRACE 120
uint8_t s_traceBuf[MAXTRACE];
uint8_t s_traceWr;
uint16_t s_traceCnt;
void trace()   // called in perOut - once envery 0.01sec
{
    //value for time described in g_model.tmrMode
    //OFFABSRUsRU%ELsEL%THsTH%ALsAL%P1P1%P2P2%P3P3%
  uint16_t v = 0;
  if((abs(g_model.tmrMode)>1) && (abs(g_model.tmrMode)<TMR_VAROFS))
	{
    v = calibratedStick[CONVERT_MODE(abs(g_model.tmrMode)/2)-1];
    v = (g_model.tmrMode<0 ? RESX-v : v+RESX ) / (RESX/16);
  }
  timer(v) ;

  uint16_t val = calibratedStick[CONVERT_MODE(3)-1]; //Get throttle channel value
  val = (val+RESX) / (RESX/16); //calibrate it
  if ( g_model.t2throttle )
  {
    if ( val >= 5 )
    {
      if ( Timer2_running == 0 )
      {
        Timer2_running = 3 ;  // Running (bit 0) and Started by throttle (bit 1)
      }
    }
  }
  static uint16_t s_time;
  static uint16_t s_cnt;
  static uint16_t s_sum;
//    static uint8_t test = 1;
  s_cnt++;
  s_sum+=val;
  if(( get_tmr10ms()-s_time)<1000) //10 sec
      //		uint16_t time10ms ;
      //		time10ms = get_tmr10ms() ;
      //    if(( time10ms-s_time)<1000) //10 sec
      return;
  s_time= get_tmr10ms() ;
 
  if ((g_model.protocol==PROTO_DSM2)&&getSwitch(MAX_DRSWITCH-1,0,0) ) audioDefevent(AUDIO_TADA);   //DSM2 bind mode warning
  //    s_time= time10ms ;
  val   = s_sum/s_cnt;
  s_sum = 0;
  s_cnt = 0;

  s_traceCnt++;
  s_traceBuf[s_traceWr++] = val;
  if(s_traceWr>=MAXTRACE) s_traceWr=0;
}


//extern unsigned int stack_free() ;

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
        chainMenu(menuProcStatistic);
        break;
    case EVT_KEY_FIRST(KEY_UP):
    case EVT_KEY_FIRST(KEY_EXIT):
      chainMenu(menuProc0);
    	killEvents(event) ;
    break;
  }
    lcd_puts_Pleft( 1*FH, PSTR("tmr1Lat max    us"));
  lcd_outdez(14*FW , 1*FH, g_tmr1Latency_max/2 );
    lcd_puts_Pleft( 2*FH, PSTR("tmr1Lat min    us"));
  lcd_outdez(14*FW , 2*FH, g_tmr1Latency_min/2 );
    lcd_puts_Pleft( 3*FH, PSTR("tmr1 Jitter    us"));
  lcd_outdez(14*FW , 3*FH, (g_tmr1Latency_max - g_tmr1Latency_min) /2 );
    lcd_puts_Pleft( 4*FH, PSTR("tmain          ms"));
  lcd_outdezAtt(14*FW , 4*FH, (g_timeMain)/20 ,PREC2);

//    lcd_puts_P( 0*FW,  5*FH, PSTR("Stack          b"));
//    lcd_outhex4( 10*FW+3, 5*FH, stack_free() ) ;

  lcd_puts_P( 3*FW,  7*FH, PSTR("[MENU] to refresh"));
}

#ifdef JETI

void menuProcJeti(uint8_t event)
{
    TITLE("JETI");

    switch(event)
    {
    //case EVT_KEY_FIRST(KEY_MENU):
    //  break;
    case EVT_KEY_FIRST(KEY_EXIT):
        JETI_DisableRXD();
        chainMenu(menuProc0);
        break;
    }

    for (uint8_t i = 0; i < 16; i++)
    {
        lcd_putcAtt((i+2)*FW,   3*FH, JetiBuffer[i], BSS);
        lcd_putcAtt((i+2)*FW,   4*FH, JetiBuffer[i+16], BSS);
    }

    if (JetiBufferReady)
    {
        JETI_EnableTXD();
        if (keyState((EnumKeys)(KEY_UP))) jeti_keys &= JETI_KEY_UP;
        if (keyState((EnumKeys)(KEY_DOWN))) jeti_keys &= JETI_KEY_DOWN;
        if (keyState((EnumKeys)(KEY_LEFT))) jeti_keys &= JETI_KEY_LEFT;
        if (keyState((EnumKeys)(KEY_RIGHT))) jeti_keys &= JETI_KEY_RIGHT;

        JetiBufferReady = 0;    // invalidate buffer

        JETI_putw((uint16_t) jeti_keys);
        _delay_ms (1);
        JETI_DisableTXD();

        jeti_keys = JETI_KEY_NOCHANGE;
    }
}
#endif

void menuProcStatistic(uint8_t event)
{
  TITLE("STAT");
  switch(event)
  {
  	case EVT_KEY_FIRST(KEY_UP):
  	  chainMenu(menuProcStatistic2);
  	break;
		case EVT_KEY_FIRST(KEY_DOWN):
  	case EVT_KEY_FIRST(KEY_EXIT):
  	  chainMenu(menuProc0);
  	break;
  }

  lcd_puts_P(  1*FW, FH*1, PSTR("TME"));
  putsTime(    7*FW, FH*1, s_timeCumAbs, 0, 0);
  lcd_puts_P( 17*FW, FH*1, PSTR("TSW"));
  putsTime(   13*FW, FH*1, s_timeCumSw,      0, 0);

  lcd_puts_P(  1*FW, FH*2, PSTR("STK"));
  putsTime(    7*FW, FH*2, s_timeCumThr, 0, 0);
  lcd_puts_P( 17*FW, FH*2, PSTR("ST%"));
  putsTime(   13*FW, FH*2, s_timeCum16ThrP/16, 0, 0);

  lcd_puts_P( 17*FW, FH*0, PSTR("TOT"));
  putsTime(   13*FW, FH*0, s_timeCumTot, 0, 0);

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

void resetTimer()
{
    s_timerState = TMR_OFF; //is changed to RUNNING dep from mode
    s_timeCumAbs=0;
    s_timeCumThr=0;
    s_timeCumSw=0;
    s_timeCum16ThrP=0;
}

extern int8_t *TrimPtr[4] ;
#ifdef FRSKY
int16_t AltOffset = 0 ;
#endif

void menuProc0(uint8_t event)
{
  static uint8_t trimSwLock;
  uint8_t view = g_eeGeneral.view & 0xf;
	
	switch(event)
	{
    case EVT_KEY_BREAK(KEY_MENU):
        if(view == e_timer2)
        {
            //            Timer2_running = !Timer2_running;
            Timer2_running ^= 1 ;
            audioDefevent(AUDIO_MENUS);
        }
        break;
    case  EVT_KEY_LONG(KEY_MENU):// go to last menu
#ifdef FRSKY
        if( (view == e_telemetry) && ((g_eeGeneral.view & 0x30) == 0x20 ) )
        {
            AltOffset = -FrskyHubData[16] ;
        }
        else if( (view == e_telemetry) && ((g_eeGeneral.view & 0x30) == 0 ) )
        {
            if ( g_model.frsky.channels[0].type == 3 )		// Current (A)
						{
				      frskyTelemetry[0].setoffset() ;
						}
            if ( g_model.frsky.channels[1].type == 3 )		// Current (A)
						{
				      frskyTelemetry[1].setoffset() ;
						}
        }
        else
        {
#endif
            pushMenu(lastPopMenu());
            killEvents(event);
#ifdef FRSKY
        }
#endif
        break;
    case EVT_KEY_LONG(KEY_RIGHT):
        pushMenu(menuProcModelSelect);
        killEvents(event);
        break;
#ifdef FRSKY
    case EVT_KEY_BREAK(KEY_RIGHT):
        if(view == e_telemetry) {
            g_eeGeneral.view = (g_eeGeneral.view + 0x10) & 0x3F;
            //            STORE_GENERALVARS;     //eeWriteGeneral();
            //            eeDirty(EE_GENERAL);
            audioDefevent(AUDIO_MENUS);
        }
        break;
    case EVT_KEY_BREAK(KEY_LEFT):
        if(view == e_telemetry) {
            g_eeGeneral.view = (g_eeGeneral.view - 0x10) & 0x3F;
            //            STORE_GENERALVARS;     //eeWriteGeneral();
            //            eeDirty(EE_GENERAL);
            audioDefevent(AUDIO_MENUS);
        }
        break;
#endif
    case EVT_KEY_LONG(KEY_LEFT):
        pushMenu(menuProcSetup);
        killEvents(event);
        break;
		case EVT_KEY_BREAK(KEY_UP) :
      g_eeGeneral.view = view+1;
      if(g_eeGeneral.view>=MAX_VIEWS) g_eeGeneral.view=0 ;
      STORE_GENERALVARS;     //eeWriteGeneral() ;
        //        eeDirty(EE_GENERAL) ;
      audioDefevent(AUDIO_KEYPAD_UP) ;
    break;
    case EVT_KEY_BREAK(KEY_DOWN) :
      if(view>0)
        g_eeGeneral.view = view - 1 ;
      else
         g_eeGeneral.view = MAX_VIEWS-1 ;
      STORE_GENERALVARS;     //eeWriteGeneral() ;
      //        eeDirty(EE_GENERAL);
      audioDefevent(AUDIO_KEYPAD_DOWN) ;
    break;
    case EVT_KEY_LONG(KEY_UP):
      chainMenu(menuProcStatistic);
      killEvents(event);
    break;
    case EVT_KEY_LONG(KEY_DOWN):
#if defined(JETI)
        JETI_EnableRXD(); // enable JETI-Telemetry reception
        chainMenu(menuProcJeti);
#elif defined(ARDUPILOT)
        ARDUPILOT_EnableRXD(); // enable ArduPilot-Telemetry reception
        chainMenu(menuProcArduPilot);
#elif defined(NMEA)
        NMEA_EnableRXD(); // enable NMEA-Telemetry reception
        chainMenu(menuProcNMEA);
#else
        chainMenu(menuProcStatistic2);
#endif
      killEvents(event);
    break;
    case EVT_KEY_FIRST(KEY_EXIT):
        if(s_timerState==TMR_BEEPING) {
            s_timerState = TMR_STOPPED;
            audioDefevent(AUDIO_MENUS);
        }
        else if(view == e_timer2) {
            resetTimer2();
            // Timer2_running = !Timer2_running;
            audioDefevent(AUDIO_MENUS);
        }
#ifdef FRSKY
        else if (view == e_telemetry) {
            resetTelemetry();
            audioDefevent(AUDIO_MENUS);
        }
#endif
        break;
    case EVT_KEY_LONG(KEY_EXIT):
        resetTimer();
        resetTimer2();
#ifdef FRSKY
        resetTelemetry();
#endif
        audioDefevent(AUDIO_MENUS);
        break;
    case EVT_ENTRY:
        killEvents(KEY_EXIT);
        killEvents(KEY_UP);
        killEvents(KEY_DOWN);
        trimSwLock = true;
        break;
    }

   if(getSwitch(g_model.trimSw,0) && !trimSwLock) setStickCenter();
   trimSwLock = getSwitch(g_model.trimSw,0);

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

  	if(s_timerState != TMR_OFF)
		{
  	  uint8_t att = DBLSIZE | (s_timerState==TMR_BEEPING ? BLINK : 0);
  	  putsTime(x+14*FW-2, FH*2, s_timerVal, att,att);
  	  putsTmrMode(x+7*FW-FW/2,FH*3,0);
  	}

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
    if(s_timerState != TMR_OFF)
 		{
      att = (s_timerState==TMR_BEEPING ? BLINK : 0);
      putsTime(18*FW+3, 0, s_timerVal, att, att);
    }
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
        int16_t a,dx;
        if(cv9){
            a   = (uint16_t)x / D9;
            dx  =((uint16_t)x % D9) * 2;
        } else {
            a   = (uint16_t)x / D5;
            dx  = (uint16_t)x % D5;
        }
        erg  = (int16_t)crv[a]*((D5-dx)/2) + (int16_t)crv[a+1]*(dx/2);
    }
    return erg / 25; // 100*D5/RESX;
}

// static variables used in perOut - moved here so they don't interfere with the stack
// It's also easier to initialize them here.
int16_t  anas [NUM_XCHNRAW] = {0};
int32_t  chans[NUM_CHNOUT] = {0};
uint8_t inacPrescale ;
uint16_t inacCounter = 0;
uint16_t inacSum = 0;
uint8_t  bpanaCenter = 0;
int16_t  sDelay[MAX_MIXERS] = {0};
int32_t  act   [MAX_MIXERS] = {0};
uint8_t  swOn  [MAX_MIXERS] = {0};

void perOut(int16_t *chanOut, uint8_t att)
{
    int16_t  trimA[4];
    uint8_t  anaCenter = 0;
    uint16_t d = 0;

    if(tick10ms) {
        if(s_noHi) s_noHi--;
        uint16_t tsum = 0;
        for(uint8_t i=0;i<4;i++) tsum += anas[i];
        if(abs(int16_t(tsum-inacSum))>INACTIVITY_THRESHOLD){
            inacSum = tsum;
            stickMoved = 1;  // reset in perMain
        }
        if( (g_eeGeneral.inactivityTimer + 10) && (g_vbat100mV>49)) {
            if (++inacPrescale > 15 )
            {
                inacCounter++;
                inacPrescale = 0 ;
            }
            uint16_t tsum = 0;
            for(uint8_t i=0;i<4;i++) tsum += anas[i];
            if(abs(int16_t(tsum-inacSum))>INACTIVITY_THRESHOLD){
                inacSum = tsum;
                inacCounter=0;
            }
            if(inacCounter>((uint16_t)(g_eeGeneral.inactivityTimer+10)*(100*60/16)))
                if((inacCounter&0x3)==1) {
                    audioDefevent(AUDIO_INACTIVITY);
                }
        }
    }
    {
        uint8_t ele_stick, ail_stick ;
        ele_stick = ELE_STICK ;
        ail_stick = AIL_STICK ;
        //===========Swash Ring================
        if(g_model.swashRingValue)
        {
            uint32_t v = (int32_t(calibratedStick[ele_stick])*calibratedStick[ele_stick] +
                          int32_t(calibratedStick[ail_stick])*calibratedStick[ail_stick]);
            uint32_t q = int32_t(RESX)*g_model.swashRingValue/100;
            q *= q;
            if(v>q)
                d = isqrt32(v);
        }
        //===========Swash Ring================

        for(uint8_t i=0;i<7;i++){        // calc Sticks

            //Normalization  [0..2048] ->   [-1024..1024]

            int16_t v = anaIn(i);
            v -= g_eeGeneral.calibMid[i];
            v  =  v * (int32_t)RESX /  (max((int16_t)100,(v>0 ?
                                                              g_eeGeneral.calibSpanPos[i] :
                                                              g_eeGeneral.calibSpanNeg[i])));
            if(v <= -RESX) v = -RESX;
            if(v >=  RESX) v =  RESX;
	  				if ( g_eeGeneral.throttleReversed )
						{
							if ( i == THR_STICK )
							{
								v = -v ;
							}
						}
            calibratedStick[i] = v; //for show in expo
            if(!(v/16)) anaCenter |= 1<<(CONVERT_MODE((i+1))-1);


            if(i<4) { //only do this for sticks
                //===========Trainer mode================
                if (!(att&NO_TRAINER) && g_model.traineron) {
                    TrainerMix* td = &g_eeGeneral.trainer.mix[i];
                    if (td->mode && getSwitch(td->swtch, 1)) {
                        uint8_t chStud = td->srcChn;
                        int16_t vStud  = (g_ppmIns[chStud]- g_eeGeneral.trainer.calib[chStud]) /* *2 */ ;
                        vStud /= 2 ;		// Only 2, because no *2 above
                        vStud *= td->studWeight ;
                        vStud /= 31 ;
                        vStud *= 4 ;
                        switch ((uint8_t)td->mode) {
                        case 1: v += vStud;   break; // add-mode
                        case 2: v  = vStud;   break; // subst-mode
                        }
                    }
                }

                //===========Swash Ring================
                if(d && (i==ele_stick || i==ail_stick))
                    v = int32_t(v)*g_model.swashRingValue*RESX/(int32_t(d)*100);
                //===========Swash Ring================

                uint8_t expoDrOn = GET_DR_STATE(i);
                uint8_t stkDir = v>0 ? DR_RIGHT : DR_LEFT;

                if(IS_THROTTLE(i) && g_model.thrExpo){
                    v  = 2*expo((v+RESX)/2,g_model.expoData[i].expo[expoDrOn][DR_EXPO][DR_RIGHT]);
                    stkDir = DR_RIGHT;
                }
                else
                    v  = expo(v,g_model.expoData[i].expo[expoDrOn][DR_EXPO][stkDir]);

                int32_t x = (int32_t)v * (g_model.expoData[i].expo[expoDrOn][DR_WEIGHT][stkDir]+100)/100;
                v = (int16_t)x;
                if (IS_THROTTLE(i) && g_model.thrExpo) v -= RESX;

                //do trim -> throttle trim if applicable
                int32_t vv = 2*RESX;
		            if(IS_THROTTLE(i) && g_model.thrTrim)
								{
									int8_t ttrim ;
									ttrim = *TrimPtr[i] ;
									if(g_eeGeneral.throttleReversed)
									{
										ttrim = -ttrim ;
									}
									vv = ((int32_t)ttrim+125)*(RESX-v)/(2*RESX);
								}
//                if(IS_THROTTLE(i) && g_model.thrTrim) vv = ((int32_t)*TrimPtr[i]+125)*(RESX-v)/(2*RESX);

                //trim
                trimA[i] = (vv==2*RESX) ? *TrimPtr[i]*2 : (int16_t)vv*2; //    if throttle trim -> trim low end
            }
            anas[i] = v; //set values for mixer
        }

        //===========BEEP CENTER================
        anaCenter &= g_model.beepANACenter;
        if(((bpanaCenter ^ anaCenter) & anaCenter)) audioDefevent(AUDIO_POT_STICK_MIDDLE);
        bpanaCenter = anaCenter;

        anas[MIX_MAX-1]  = RESX;     // MAX
        anas[MIX_FULL-1] = RESX;     // FULL
        for(uint8_t i=0;i<4;i++) anas[i+PPM_BASE] = (g_ppmIns[i] - g_eeGeneral.trainer.calib[i])*2; //add ppm channels
        for(uint8_t i=4;i<NUM_PPM;i++)    anas[i+PPM_BASE]   = g_ppmIns[i]*2; //add ppm channels
        for(uint8_t i=0;i<NUM_CHNOUT;i++) anas[i+CHOUT_BASE] = chans[i]; //other mixes previous outputs

        //===========Swash Ring================
        if(g_model.swashRingValue)
        {
            uint32_t v = ((int32_t)anas[ele_stick]*anas[ele_stick] + (int32_t)anas[ail_stick]*anas[ail_stick]);
            uint32_t q = (int32_t)RESX*g_model.swashRingValue/100;
            q *= q;
            if(v>q)
            {
                uint16_t d = isqrt32(v);
                anas[ele_stick] = (int32_t)anas[ele_stick]*g_model.swashRingValue*RESX/((int32_t)d*100);
                anas[ail_stick] = (int32_t)anas[ail_stick]*g_model.swashRingValue*RESX/((int32_t)d*100);
            }
        }

#define REZ_SWASH_X(x)  ((x) - (x)/8 - (x)/128 - (x)/512)   //  1024*sin(60) ~= 886
#define REZ_SWASH_Y(x)  ((x))   //  1024 => 1024

        if(g_model.swashType)
        {
            int16_t vp = anas[ele_stick]+trimA[ele_stick];
            int16_t vr = anas[ail_stick]+trimA[ail_stick];

            if(att&NO_INPUT)  //zero input for setStickCenter()
            {
                vp = vr = 0;
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

    if(tick10ms) trace(); //trace thr 0..32  (/32)

    memset(chans,0,sizeof(chans));        // All outputs to 0

    if(att&NO_INPUT) { //zero input for setStickCenter()
        for(uint8_t i=0;i<4;i++) {
            if(!IS_THROTTLE(i)) {
                anas[i]  = 0;
                trimA[i] = 0;
            }
        }
        for(uint8_t i=0;i<4;i++) anas[i+PPM_BASE] = 0;
    }

    uint8_t mixWarning = 0;
    //========== MIXER LOOP ===============

    // Set the trim pointers back to the master set
    TrimPtr[0] = &g_model.trim[0] ;
    TrimPtr[1] = &g_model.trim[1] ;
    TrimPtr[2] = &g_model.trim[2] ;
    TrimPtr[3] = &g_model.trim[3] ;

    for(uint8_t i=0;i<MAX_MIXERS;i++)
		{
//        MixData *md = mixaddress( i ) ;
        MixData *md = &g_model.mixData[i] ;

        if((md->destCh==0) || (md->destCh>NUM_CHNOUT)) break;

        //Notice 0 = NC switch means not used -> always on line
        int16_t v  = 0;
        uint8_t swTog;

        //swOn[i]=false;
        if(!getSwitch(md->swtch,1)){ // switch on?  if no switch selected => on
            swTog = swOn[i];
            swOn[i] = false;
            //            if(md->srcRaw==MIX_MAX) act[i] = 0;// MAX back to 0 for slow up
            //            if(md->srcRaw!=MIX_FULL) continue;// if not FULL - next loop
            //            v = -RESX; // switch is off  => FULL=-RESX

            if(md->srcRaw!=MIX_MAX && md->srcRaw!=MIX_FULL) continue;// if not MAX or FULL - next loop
            if(md->mltpx==MLTPX_REP) continue; // if switch is off and REPLACE then off
            v = (md->srcRaw == MIX_FULL ? -RESX : 0); // switch is off and it is either MAX=0 or FULL=-512
        }
        else {
            swTog = !swOn[i];
            swOn[i] = true;
            uint8_t k = md->srcRaw-1;
            v = anas[k]; //Switch is on. MAX=FULL=512 or value.
            if(k>=CHOUT_BASE && (k<i)) v = chans[k]; // if we've already calculated the value - take it instead // anas[i+CHOUT_BASE] = chans[i]
            if(md->mixWarn) mixWarning |= 1<<(md->mixWarn-1); // Mix warning
            if ( md->enableFmTrim )
            {
                if ( md->srcRaw <= 4 )
                {
                    TrimPtr[md->srcRaw-1] = &md->sOffset ;		// Use the value stored here for the trim
                }
            }
        }

        //========== INPUT OFFSET ===============
        if ( md->enableFmTrim == 0 )
        {
            if(md->sOffset) v += calc100toRESX(md->sOffset);
        }

        //========== DELAY and PAUSE ===============
        if (md->speedUp || md->speedDown || md->delayUp || md->delayDown)  // there are delay values
        {
#define DEL_MULT 256

            //if(init) {
            //act[i]=(int32_t)v*DEL_MULT;
            //swTog = false;
            //}
            int16_t diff = v-act[i]/DEL_MULT;

            if(swTog) {
                //need to know which "v" will give "anas".
                //curves(v)*weight/100 -> anas
                // v * weight / 100 = anas => anas*100/weight = v
                if(md->mltpx==MLTPX_REP)
                {
                    act[i] = (int32_t)anas[md->destCh-1+CHOUT_BASE]*DEL_MULT;
                    act[i] *=100;
                    if(md->weight) act[i] /= md->weight;
                }
                diff = v-act[i]/DEL_MULT;
                if(diff) sDelay[i] = (diff<0 ? md->delayUp :  md->delayDown) * 100;
            }

            if(sDelay[i]){ // perform delay
                if(tick10ms)
                {
                  sDelay[i]-- ;
                }
                if (sDelay[i] != 0)
                { // At end of delay, use new V and diff
                  v = act[i]/DEL_MULT;   // Stay in old position until delay over
                  diff = 0;
                }
            }

            if(diff && (md->speedUp || md->speedDown)){
                //rate = steps/sec => 32*1024/100*md->speedUp/Down
                //act[i] += diff>0 ? (32768)/((int16_t)100*md->speedUp) : -(32768)/((int16_t)100*md->speedDown);
                //-100..100 => 32768 ->  100*83886/256 = 32768,   For MAX we divide by 2 sincde it's asymmetrical
                if(tick10ms) {
                    int32_t rate = (int32_t)DEL_MULT*2048*100;
                    if(md->weight) rate /= abs(md->weight);
                    act[i] = (diff>0) ? ((md->speedUp>0)   ? act[i]+(rate)/((int16_t)100*md->speedUp)   :  (int32_t)v*DEL_MULT) :
                                        ((md->speedDown>0) ? act[i]-(rate)/((int16_t)100*md->speedDown) :  (int32_t)v*DEL_MULT) ;
                }

                if(((diff>0) && (v<(act[i]/DEL_MULT))) || ((diff<0) && (v>(act[i]/DEL_MULT)))) act[i]=(int32_t)v*DEL_MULT; //deal with overflow
                v = act[i]/DEL_MULT;
            }
            else if (diff)
            {
              act[i]=(int32_t)v*DEL_MULT;
            }
        }


        //========== CURVES ===============
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
            v = intpol(v, md->curve - 7);
        }

        //========== TRIM ===============
        if((md->carryTrim==0) && (md->srcRaw>0) && (md->srcRaw<=4)) v += trimA[md->srcRaw-1];  //  0 = Trim ON  =  Default

        //========== MULTIPLEX ===============
        int32_t dv = (int32_t)v*md->weight;
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

        if(mixWarning & 1) if(((tmr10ms&0xFF)==  0)) audioDefevent(AUDIO_MIX_WARNING_1);
        if(mixWarning & 2) if(((tmr10ms&0xFF)== 64) || ((tmr10ms&0xFF)== 72)) audioDefevent(AUDIO_MIX_WARNING_2);
        if(mixWarning & 4) if(((tmr10ms&0xFF)==128) || ((tmr10ms&0xFF)==136) || ((tmr10ms&0xFF)==144)) audioDefevent(AUDIO_MIX_WARNING_3);        


    }

    //========== LIMITS ===============
    for(uint8_t i=0;i<NUM_CHNOUT;i++){
        // chans[i] holds data from mixer.   chans[i] = v*weight => 1024*100
        // later we multiply by the limit (up to 100) and then we need to normalize
        // at the end chans[i] = chans[i]/100 =>  -1024..1024
        // interpolate value with min/max so we get smooth motion from center to stop
        // this limits based on v original values and min=-1024, max=1024  RESX=1024

        int32_t q = chans[i];// + (int32_t)g_model.limitData[i].offset*100; // offset before limit

        chans[i] /= 100; // chans back to -1024..1024
        ex_chans[i] = chans[i]; //for getswitch

        int16_t ofs = g_model.limitData[i].offset;
        int16_t lim_p = 10*(g_model.limitData[i].max+100);
        int16_t lim_n = 10*(g_model.limitData[i].min-100); //multiply by 10 to get same range as ofs (-1000..1000)
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

        if(g_model.safetySw[i].swtch)  //if safety sw available for channel check and replace val if needed
            if(getSwitch(g_model.safetySw[i].swtch,0)) q = calc100toRESX(g_model.safetySw[i].val);

//        cli();
        chanOut[i] = q; //copy consistent word to int-level
//        sei();
    }
}



void evalOffset(int8_t sub, uint8_t max)
{
    if(sub<1) s_pgOfs=0;
    else if(sub-s_pgOfs>max) s_pgOfs = sub-max;
    else if(sub-s_pgOfs<max-6) s_pgOfs = sub-max+6;
}




