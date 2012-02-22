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

// This version for ARM based ERSKY9X board

// Items set to make things compile, will need to be sorted eventually
#define assert(x)
#define wdt_reset()

#define VERSION	"V0.26"

#define DATE_STR "xx.xx.2012"
#define TIME_STR "xx:xx:xx"
#define SUB_VERS VERSION
#define SVN_VERS "trunk-rxxx"
#define MOD_VERS "Normal"

//#define STR2(s) #s
//#define DEFNUMSTR(s)  STR2(s)

const char stamp1[] = "VERS: "SUB_VERS ;
const char stamp2[] = "DATE: " DATE_STR;
const char stamp3[] = "TIME: " TIME_STR;
const char stamp4[] = " SVN: " SVN_VERS;
const char stamp5[] = " MOD: " MOD_VERS;

#define CPU_INT		int32_t
#define CPU_UINT	uint32_t

#define BITMASK(bit) (1<<(bit))

#define PSTR(a)  (char *)a
#define PROGMEM	 const unsigned char
#define strcpy_P(a,b)	strcpy(a,b)
#define strncpy_P(a,b,c)	strncpy(a,b,c)
#define pgm_read_byte(p)	(*(p))


#define DIM(arr) (sizeof((arr))/sizeof((arr)[0]))

enum EnumKeys {
    KEY_MENU ,
    KEY_EXIT ,
    KEY_DOWN ,
    KEY_UP  ,
    KEY_RIGHT ,
    KEY_LEFT ,
    TRM_LH_DWN  ,
    TRM_LH_UP   ,
    TRM_LV_DWN  ,
    TRM_LV_UP   ,
    TRM_RV_DWN  ,
    TRM_RV_UP   ,
    TRM_RH_DWN  ,
    TRM_RH_UP   ,
    //SW_NC     ,
    //SW_ON     ,
    SW_ThrCt  ,
    SW_RuddDR ,
    SW_ElevDR ,
    SW_ID0    ,
    SW_ID1    ,
    SW_ID2    ,
    SW_AileDR ,
    SW_Gear   ,
    SW_Trainer
};

#define CURV_STR "---x>0x<0|x|f>0f<0|f|c1 c2 c3 c4 c5 c6 c7 c8 c9 c10c11c12c13c14c15c16"
#define CURVE_BASE 7

#define CSWITCH_STR  "----   v>ofs  v<ofs  |v|>ofs|v|<ofsAND    OR     XOR    ""v1==v2 ""v1!=v2 ""v1>v2  ""v1<v2  ""v1>=v2 ""v1<=v2 "
#define CSW_LEN_FUNC 7


#define CS_OFF       0
#define CS_VPOS      1  //v>offset
#define CS_VNEG      2  //v<offset
#define CS_APOS      3  //|v|>offset
#define CS_ANEG      4  //|v|<offset
#define CS_AND       5
#define CS_OR        6
#define CS_XOR       7
#define CS_EQUAL     8
#define CS_NEQUAL    9
#define CS_GREATER   10
#define CS_LESS      11
#define CS_EGREATER  12
#define CS_ELESS     13
#define CS_MAXF      13  //max function

#define CS_VOFS       0
#define CS_VBOOL      1
#define CS_VCOMP      2
#define CS_STATE(x)   ((x)<CS_AND ? CS_VOFS : ((x)<CS_EQUAL ? CS_VBOOL : CS_VCOMP))

#define SW_BASE      SW_ThrCt
#define SW_BASE_DIAG SW_ThrCt
#define MAX_DRSWITCH (1+SW_Trainer-SW_ThrCt+1+NUM_CSW)

#define SWP_RUD (SW_RuddDR-SW_BASE)
#define SWP_ELE (SW_ElevDR-SW_BASE)
#define SWP_AIL (SW_AileDR-SW_BASE)
#define SWP_THR (SW_ThrCt-SW_BASE)
#define SWP_GEA (SW_Gear-SW_BASE)

#define SWP_RUDB (1<<SWP_RUD)
#define SWP_ELEB (1<<SWP_ELE)
#define SWP_AILB (1<<SWP_AIL)
#define SWP_THRB (1<<SWP_THR)
#define SWP_GEAB (1<<SWP_GEA)

#define SWP_ID0 (SW_ID0-SW_BASE)
#define SWP_ID1 (SW_ID1-SW_BASE)
#define SWP_ID2 (SW_ID2-SW_BASE)
#define SWP_ID0B (1<<SWP_ID0)
#define SWP_ID1B (1<<SWP_ID1)
#define SWP_ID2B (1<<SWP_ID2)

//Switch Position Illigal states
#define SWP_IL1 (0)
#define SWP_IL2 (SWP_ID0B | SWP_ID1B)
#define SWP_IL3 (SWP_ID0B | SWP_ID2B)
#define SWP_IL4 (SWP_ID1B | SWP_ID2B)
#define SWP_IL5 (SWP_ID0B | SWP_ID1B | SWP_ID2B)

#define SWITCHES_STR "THR""RUD""ELE""ID0""ID1""ID2""AIL""GEA""TRN""SW1""SW2""SW3""SW4""SW5""SW6""SW7""SW8""SW9""SWA""SWB""SWC"
#define NUM_CSW  12 //number of custom switches



#define NUM_KEYS TRM_RH_UP+1
#define TRM_BASE TRM_LH_DWN

#define _MSK_KEY_REPT    0x40
#define _MSK_KEY_DBL     0x10
#define IS_KEY_BREAK(key)  (((key)&0xf0)        ==  0x20)
#define EVT_KEY_BREAK(key) ((key)|                  0x20)
#define EVT_KEY_FIRST(key) ((key)|    _MSK_KEY_REPT|0x20)
#define EVT_KEY_REPT(key)  ((key)|    _MSK_KEY_REPT     )
#define EVT_KEY_LONG(key)  ((key)|0x80)
#define EVT_KEY_DBL(key)   ((key)|_MSK_KEY_DBL)
//#define EVT_KEY_DBL(key)   ((key)|0x10)
#define EVT_ENTRY               (0xff - _MSK_KEY_REPT)
#define EVT_ENTRY_UP            (0xfe - _MSK_KEY_REPT)
#define EVT_KEY_MASK             0x0f


#define INP_D_TRM_LH_UP   7
#define INP_D_TRM_LH_DWN  6
#define INP_D_TRM_RV_DWN  5
#define INP_D_TRM_RV_UP   4
#define INP_D_TRM_LV_DWN  3
#define INP_D_TRM_LV_UP   2
#define INP_D_TRM_RH_DWN  1
#define INP_D_TRM_RH_UP   0

#define RESX    (1<<10) // 1024
#define RESXu   1024u
#define RESXul  1024ul
#define RESXl   1024l
#define RESKul  100ul
#define RESX_PLUS_TRIM (RESX+128)

#define NUM_PPM     8
//number of real outputchannels CH1-CH16
#define NUM_CHNOUT  16
///number of real input channels (1-9) plus virtual input channels X1-X4
#define PPM_BASE    MIX_CYC3
#define CHOUT_BASE  (PPM_BASE+NUM_PPM)

#define SWASH_TYPE_STR   "---   ""120   ""120X  ""140   ""90    "

#define SWASH_TYPE_120   1
#define SWASH_TYPE_120X  2
#define SWASH_TYPE_140   3
#define SWASH_TYPE_90    4
#define SWASH_TYPE_NUM   4


#define MIX_P1    5
#define MIX_P2    6
#define MIX_P3    7
#define MIX_MAX   8
#define MIX_FULL  9
#define MIX_CYC1  10
#define MIX_CYC2  11
#define MIX_CYC3  12

#define DR_HIGH   0
#define DR_MID    1
#define DR_LOW    2
#define DR_EXPO   0
#define DR_WEIGHT 1
#define DR_RIGHT  0
#define DR_LEFT   1
#define DR_DRSW1  99
#define DR_DRSW2  98

#define DSW_THR  1
#define DSW_RUD  2
#define DSW_ELE  3
#define DSW_ID0  4
#define DSW_ID1  5
#define DSW_ID2  6
#define DSW_AIL  7
#define DSW_GEA  8
#define DSW_TRN  9
#define DSW_SW1  10
#define DSW_SW2  11
#define DSW_SW3  12
#define DSW_SW4  13
#define DSW_SW5  14
#define DSW_SW6  15
#define DSW_SW7   16
#define DSW_SW8   17
#define DSW_SW9   18
#define DSW_SWA   19
#define DSW_SWB   20
#define DSW_SWC   21

#define SCROLL_TH 64
#define INACTIVITY_THRESHOLD 256
#define THRCHK_DEADBAND 16
#define SPLASH_TIMEOUT  (4*100)  //400 msec - 4 seconds

#define IS_THROTTLE(x)  (((2-(g_eeGeneral.stickMode&1)) == x) && (x<4))

extern uint8_t Ee_lock ;

// Bit masks in Ee_lock
#define EE_LOCK      1
#define EE_TRIM_LOCK 2

#define EE_GENERAL 1
#define EE_MODEL   2
#define EE_TRIM    4           // Store model because of trim


#define TMR_VAROFS  16

#define TMRMODE_NONE     0
#define TMRMODE_ABS      1
#define TMRMODE_THR      2
#define TMRMODE_THR_REL  3
#define MAX_ALERT_TIME   60

#define PROTO_PPM        0
#define PROTO_PXX        1
#define PROTO_DSM2       2
#define PROTO_PPM16			 3
#define PROT_MAX         3
#define PROT_STR "PPM   PXX   DSM2  PPM16 "
#define PROT_STR_LEN     6
#define DSM2_STR "LP4/LP5  DSM2only DSM2/DSMX"
#define DSM2_STR_LEN   9
#define LPXDSM2          0
#define DSM2only         1
#define DSM2_DSMX        2

#define PXX_SEND_RXNUM     0x01
#define PXX_SEND_FAILSAFE  0x02

extern uint8_t pxxFlag;
extern uint8_t stickMoved;

#define FLASH_DURATION 50

extern uint16_t g_LightOffCounter;


template<class t> inline t min(t a, t b){ return a<b?a:b; }
template<class t> inline t max(t a, t b){ return a>b?a:b; }

// This doesn't need protection on this processor
#define get_tmr10ms() g_tmr10ms

#define sysFLAG_OLD_EEPROM (0x01)
extern uint8_t sysFlags;

const char s_charTab[]=" ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-.";
#define NUMCHARS (sizeof(s_charTab)-1)

#define NUM_PPM     8
//number of real outputchannels CH1-CH16
#define NUM_CHNOUT  16
///number of real input channels (1-9) plus virtual input channels X1-X4
#define PPM_BASE    MIX_CYC3
#define CHOUT_BASE  (PPM_BASE+NUM_PPM)

#ifdef FRSKY
#define NUM_TELEMETRY 2
#define TELEMETRY_CHANNELS "AD1 AD2 "
#else
#define NUM_TELEMETRY 0
#define TELEMETRY_CHANNELS ""
#endif

#define NUM_XCHNRAW (CHOUT_BASE+NUM_CHNOUT+NUM_TELEMETRY) // NUMCH + P1P2P3+ AIL/RUD/ELE/THR + MAX/FULL + CYC1/CYC2/CYC3
///number of real output channels (CH1-CH8) plus virtual output channels X1-X4
#define NUM_XCHNOUT (NUM_CHNOUT) //(NUM_CHNOUT)//+NUM_VIRT)


inline int32_t calc100toRESX(register int8_t x)
{
  return ((uint32_t)x*655)>>6 ;
}

inline int16_t calc1000toRESX( register int32_t x)  // improve calc time by Pat MacKenzie
{
    register int32_t y = x>>5;
    x+=y;
    y=y>>2;
    x-=y;
    return x+(y>>2);
    //  return x + x/32 - x/128 + x/512;
}



const char modn12x3[]= {
    1, 2, 3, 4,
    1, 3, 2, 4,
    4, 2, 3, 1,
    4, 3, 2, 1 };

extern const uint8_t chout_ar[] ;

//convert from mode 1 to mode g_eeGeneral.stickMode
//NOTICE!  =>  1..4 -> 1..4
extern uint8_t convert_mode_helper(uint8_t x) ;

#define CONVERT_MODE(x)  (((x)<=4) ? convert_mode_helper(x) : (x))
#define CHANNEL_ORDER(x) (chout_ar[g_eeGeneral.templateSetup*4 + (x)-1])
#define THR_STICK       (2-(g_eeGeneral.stickMode&1))
#define ELE_STICK       (1+(g_eeGeneral.stickMode&1))
#define AIL_STICK       ((g_eeGeneral.stickMode&2) ? 0 : 3)
#define RUD_STICK       ((g_eeGeneral.stickMode&2) ? 3 : 0)

#define STORE_MODELVARS_TRIM   eeDirty(EE_MODEL|EE_TRIM)
#define STORE_MODELVARS   eeDirty(EE_MODEL)
#define STORE_GENERALVARS eeDirty(EE_GENERAL)
#define BACKLIGHT_ON    (PWM->PWM_CH_NUM[0].PWM_CDTY = g_eeGeneral.bright)
#define BACKLIGHT_OFF   (PWM->PWM_CH_NUM[0].PWM_CDTY = 100)

typedef void (*MenuFuncP)(uint8_t event);


extern bool    checkIncDec_Ret;//global helper vars
extern uint8_t s_editMode;     //global editmode

int16_t checkIncDec16(uint8_t event, int16_t i_pval, int16_t i_min, int16_t i_max, uint8_t i_flags);
int8_t checkIncDec(uint8_t event, int8_t i_val, int8_t i_min, int8_t i_max, uint8_t i_flags);
int8_t checkIncDec_hm(uint8_t event, int8_t i_val, int8_t i_min, int8_t i_max);
int8_t checkIncDec_vm(uint8_t event, int8_t i_val, int8_t i_min, int8_t i_max);
int8_t checkIncDec_hg(uint8_t event, int8_t i_val, int8_t i_min, int8_t i_max);

#define CHECK_INCDEC_H_GENVAR( event, var, min, max)     \
    var = checkIncDec_hg(event,var,min,max)

#define CHECK_INCDEC_H_MODELVAR( event, var, min, max)     \
    var = checkIncDec_hm(event,var,min,max)

extern uint8_t heartbeat ;
extern int16_t g_chans512[NUM_CHNOUT];
extern uint8_t eeprom[4096] ;

uint8_t char2idx(char c);
char idx2char(uint8_t idx);

extern volatile uint32_t Timer2_count ;		// Modified in interrupt routine
extern volatile uint16_t g_tmr10ms ;
extern volatile uint8_t  g_blinkTmr10ms;
extern volatile uint8_t tick10ms ;
extern uint32_t Master_frequency ;

extern void alert(const char * s, bool defaults=false);
extern void message(const char * s);

void resetTimer();


extern uint8_t Timer2_running ;
extern uint16_t Timer2 ;
void resetTimer2( void ) ;

extern void putsTime(uint8_t x,uint8_t y,int16_t tme,uint8_t att,uint8_t att2) ;
extern void putsVolts(uint8_t x,uint8_t y, uint8_t volts, uint8_t att) ;
extern void putsVBat(uint8_t x,uint8_t y,uint8_t att) ;
extern void putsVBat(uint8_t x,uint8_t y,uint8_t att) ;
extern void putsChnRaw(uint8_t x,uint8_t y,uint8_t idx,uint8_t att) ;
extern void putsChn(uint8_t x,uint8_t y,uint8_t idx1,uint8_t att) ;
extern void putsDrSwitches(uint8_t x,uint8_t y,int8_t idx1,uint8_t att) ; //, bool nc) ;
extern void putsTmrMode(uint8_t x, uint8_t y, uint8_t attr);
extern const char *get_switches_string( void ) ;

extern int16_t intpol(int16_t x, uint8_t idx);

extern uint16_t anaIn(uint8_t chan) ;

extern int16_t ex_chans[NUM_CHNOUT];


void eeWaitComplete( void ) ;
void eeDirty(uint8_t msk);
void eeCheck(bool immediately=false ) ;
void eeReadAll( void ) ;
void eeLoadModelName(uint8_t id,char*buf,uint8_t len);
//uint16_t eeFileSize(uint8_t id);
void eeLoadModel(uint8_t id);
//void eeSaveModel(uint8_t id);
bool eeDuplicateModel(uint8_t id);
bool eeModelExists(uint8_t id);

extern char idx2char(uint8_t idx) ;
extern uint8_t char2idx(char c) ;

extern int16_t            g_ppmIns[8];
extern uint8_t ppmInState ; //0=unsync 1..8= wait for value i-1

/// goto given Menu, but substitute current menu in menuStack
extern void    chainMenu(MenuFuncP newMenu);
/// goto given Menu, store current menu in menuStack
extern void    pushMenu(MenuFuncP newMenu);
///deliver address of last menu which was popped from
extern MenuFuncP lastPopMenu();
/// return to last menu in menustack
/// if uppermost is set true, thenmenu return to uppermost menu in menustack
void    popMenu(bool uppermost=false);

#define NO_TRAINER 0x01
#define NO_INPUT   0x02


extern bool getSwitch(int8_t swtch, bool nc, uint8_t level = 0 ) ;
extern uint8_t g_vbat100mV ;
extern uint16_t Timer2 ;
extern void doSplash( void ) ;






