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

#ifndef menus_h
#define menus_h

//extern audioQueue  audio;

/*#define IS_THROTTLE(x)  (((2-(g_eeGeneral.stickMode&1)) == x) && (x<4))
#define GET_DR_STATE(x) (!getSwitch(g_model.expoData[x].drSw1,0) ?   \
                          DR_HIGH :                                  \
                          !getSwitch(g_model.expoData[x].drSw2,0)?   \
                          DR_MID : DR_LOW);

#define DO_SQUARE(xx,yy,ww)         \
    lcd_vline(xx-ww/2,yy-ww/2,ww);  \
    lcd_hline(xx-ww/2,yy+ww/2,ww);  \
    lcd_vline(xx+ww/2,yy-ww/2,ww);  \
    lcd_hline(xx-ww/2,yy-ww/2,ww);

#define DO_CROSS(xx,yy,ww)          \
    lcd_vline(xx,yy-ww/2,ww);  \
    lcd_hline(xx-ww/2,yy,ww);  \

#define V_BAR(xx,yy,ll)       \
    lcd_vline(xx-1,yy-ll,ll); \
    lcd_vline(xx  ,yy-ll,ll); \
    lcd_vline(xx+1,yy-ll,ll); \

#define NO_HI_LEN 25

#define WCHART 32
#define X0     (128-WCHART-2)
#define Y0     32
#define WCHARTl 32l
#define X0l     (128l-WCHARTl-2)
#define Y0l     32l
#define RESX    1024
#define RESXu   1024u
#define RESXul  1024ul
#define RESXl   1024l
#define RESKul  100ul
#define RESX_PLUS_TRIM (RESX+128)

extern bool warble;*/
//extern int16_t p1valdiff;
//extern uint8_t scroll_disabled;

#define prog_uint8_t const uint8_t
#define pgm_read_adr(x) *(x)

#define min(x,y) ((x<y)?x:y)

// Menus related stuff ...
struct MState2
{
  uint8_t m_posVert;
  uint8_t m_posHorz;
  void init(){m_posVert=m_posHorz=0;};
  void check(uint8_t event, uint8_t curr, MenuFuncP *menuTab, uint8_t menuTabSize, prog_uint8_t *subTab, uint8_t subTabMax, uint8_t maxrow);
  void check_simple(uint8_t event, uint8_t curr, MenuFuncP *menuTab, uint8_t menuTabSize, uint8_t maxrow);
  void check_submenu_simple(uint8_t event, uint8_t maxrow);
};

void evalOffset(int8_t sub, uint8_t max);

//typedef PROGMEM void (*MenuFuncP_PROGMEM)(uint8_t event);
typedef const void (*MenuFunc)(uint8_t event) ;

#define TITLEP(pstr) lcd_putsAtt(0,0,pstr,INVERS)
//#define TITLE(str)   TITLEP(PSTR(str))
#define TITLE(str)   TITLEP(str)


#define SIMPLE_MENU(title, tab, menu, lines_count) \
TITLE(title); \
static MState2 mstate2; \
mstate2.check_simple(event,menu,tab,DIM(tab),lines_count-1)

#define MENU(title, tab, menu, lines_count, lines...) \
TITLE(title); \
static MState2 mstate2; \
static const uint8_t mstate_tab[] = lines; \
mstate2.check(event,menu,tab,DIM(tab),mstate_tab,DIM(mstate_tab)-1,lines_count-1)

/*

//#define SUBMENU(title, lines_count, lines...) \
//TITLE(title); \
//static MState2 mstate2; \
//static prog_uint8_t APM mstate_tab[] = lines; \
//mstate2.check(event,0,NULL,0,mstate_tab,DIM(mstate_tab)-1,lines_count-1)

//#define SIMPLE_SUBMENU_NOTITLE(lines_count) \
//static MState2 mstate2; \
//mstate2.check_submenu_simple(event,lines_count-1)

//#define SIMPLE_SUBMENU(title, lines_count) \
//TITLE(title); \
//SIMPLE_SUBMENU_NOTITLE(lines_count-1)
	*/


extern int16_t calibratedStick[7] ;
extern int16_t g_chans512[NUM_CHNOUT];

extern void doMainScreenGrphics( void ) ;
extern void menuProcStatistic2(uint8_t event) ;
extern void menuProc0(uint8_t event) ;


#endif
