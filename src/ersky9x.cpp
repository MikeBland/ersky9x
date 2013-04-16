/****************************************************************************
*  Copyright (c) 2011 by Michael Blandford. All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*  1. Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*  2. Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*  3. Neither the name of the author nor the names of its contributors may
*     be used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
*  THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
*  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
*  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
*  THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
*  SUCH DAMAGE.
*
****************************************************************************
*  History:
*
****************************************************************************/
#define __ERSKY9X_CPP__

#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>


#include "AT91SAM3S4.h"
#ifndef SIMU
#include "core_cm3.h"
#endif


#include "ersky9x.h"
#include "audio.h"
#include "sound.h"
#include "lcd.h"
#include "myeeprom.h"
#include "drivers.h"
#include "file.h"
#include "menus.h"
#include "timers.h"
#include "logicio.h"
#include "pulses.h"
#ifdef FRSKY
#include "frsky.h"
#endif

#ifndef SIMU
#include "CoOS.h"
#endif

#ifndef SIMU
#define MAIN_STACK_SIZE		500
#ifdef PCBSKY
#define BT_STACK_SIZE			100
#endif
#define DEBUG_STACK_SIZE	130
#define VOICE_STACK_SIZE	130

OS_TID MainTask;
OS_STK main_stk[MAIN_STACK_SIZE] ;

#ifdef PCBSKY
OS_TID BtTask;
OS_STK Bt_stk[BT_STACK_SIZE] ;
#endif
OS_TID VoiceTask;
OS_STK voice_stk[VOICE_STACK_SIZE] ;

#ifdef	DEBUG
OS_TID DebugTask;
OS_STK debug_stk[DEBUG_STACK_SIZE] ;
#endif

#endif

const uint8_t splashdata[] = { 'S','P','S',0,
#include "s9xsplash.lbm"
	'S','P','E',0};
//const uchar *s9xsplash = splashdata+4;

#include "debug.h"

t_time Time ;

uint8_t unexpectedShutdown = 0;

//#define TRUE	1
//#define FALSE	0

//#define bool uint32_t

// Soft power operation
// When the main power switch is turned on, the CPU starts, finds PC17 is high
// so RF Power is on, enables the pullup on PA8 (50K to 175K), and thus turns
// the soft power switch ON. Even with 175K pullup, assuming a hFE of the transistor
// of 100, the transistor collector current should be at least 1.46 mA. This is
// enough to pull the gate of the soft switch low. When you turn the power switch
// off, RF power goes off, PC17 goes low (could have an internal pull down resistor),
// so the CPU knows the power switch is off, CPU tidies up,
// disables the pullup on PA8 and turns itself off.

// If you plug the trainer cable in, with the power switch off, the soft power
// switch turns on, the CPU finds PC17 is low, and PA8 is high so power must be
// because of the trainer cable. As long as the voltage from the trainer power
// is at least 6 volts, PA8 will be read as a 1 (>2.31 volts (0.7*VDDIO)). The
// CPU turns the pullup resistor on PA8 on, thus holding the soft power ON.
// When you unplug the trainer cable, the voltage at PA8 pin will drop. Even
// with a 50K pullup resistor, and 0.7V across the transistor base-emitter,
// the voltage at PA8 will be less than 0.87V. A logic 0 is a voltage less
// than 0.99V (0.3*VDDIO). So the CPU will see a logic 0 on PA8 and know it
// is time to tidy up, then turn the soft power off.


/*=========================================================================*/
/*  DEFINE: All Structures and Common Constants                            */
/*=========================================================================*/

/*=========================================================================*/
/*  DEFINE: Prototypes                                                     */
/*=========================================================================*/


extern uint16_t g_timeMain;

volatile int32_t Rotary_position ;
volatile int32_t Rotary_count ;
int32_t LastRotaryValue ;
int32_t Rotary_diff ;
uint8_t Vs_state[NUM_SKYCHNOUT] ;
int8_t RotaryControl ;


#ifdef PCBSKY
void tmrBt_Handle( void ) ;
void bt_task(void* pdata) ;
#endif
void main_loop( void* pdata ) ;
void mainSequence( uint32_t no_menu ) ;
void doSplash( void ) ;
void perMain( uint32_t no_menu ) ;
void UART_Configure( uint32_t baudrate, uint32_t masterClock) ;
void UART2_Configure( uint32_t baudrate, uint32_t masterClock) ;
void txmit( uint8_t c ) ;
void uputs( char *string ) ;
uint16_t rxuart( void ) ;
//static void start_timer2( void ) ;
extern "C" void TC2_IRQHandler( void ) ;
#ifdef SIMU
#define sam_boot()
#else
extern "C" void sam_boot( void ) ;
#endif
#ifdef	DEBUG
void handle_serial( void* pdata ) ;
#endif
uint16_t anaIn( uint8_t chan ) ;
void getADC_single( void ) ;
void getADC_osmp( void ) ;
void getADC_filt( void ) ;
void read_9_adc( void ) ;
void init_adc( void ) ;
//void poll_pwm( void ) ;
//uint32_t hextoi( uint8_t *string ) ;
//uint32_t gets( uint8_t *string, uint32_t maxcount ) ;
//uint32_t get_switches( void ) ;
void checkTHR( void ) ;
static void checkSwitches( void ) ;
void check_backlight( void ) ;
void checkQuickSelect( void ) ;
void actionUsb( void ) ;

static uint8_t checkTrim(uint8_t event) ;
//void screen0( void ) ;

void putsChnRaw(uint8_t x,uint8_t y,uint8_t idx,uint8_t att) ;
void putsChn(uint8_t x,uint8_t y,uint8_t idx1,uint8_t att) ;
void putsDrSwitches(uint8_t x,uint8_t y,int8_t idx1,uint8_t att) ;//, bool nc) ;
const char *get_switches_string( void ) ;
bool getSwitch(int8_t swtch, bool nc, uint8_t level) ;
static void init_soft_power( void ) ;
uint32_t check_soft_power( void ) ;
void soft_power_off( void ) ;

#if !defined(SIMU)
static void init_rotary_encoder( void ) ;
#endif
static void stop_rotary_encoder( void ) ;




/*=========================================================================*/
/*  DEFINE: Definition of all local Data                                   */
/*=========================================================================*/



uint32_t Master_frequency ;
//uint16_t Adc_data[32] ;
//volatile uint32_t Timer2_count ;		// Modified in interrupt routine
volatile uint32_t Tenms ;						// Modified in interrupt routine
volatile uint8_t tick10ms = 0 ;
uint16_t g_LightOffCounter ;
uint8_t  stickMoved = 0;

uint16_t S_anaFilt[NUMBER_ANALOG] ;				// Analog inputs after filtering
uint16_t Current_analogue ;
uint16_t Current_max ;
uint32_t Current_accumulator ;
uint32_t Current_used ;

uint16_t MAh_used ;
uint16_t Run_time ;

uint8_t sysFlags = 0 ;


//uint32_t Per10ms_action ;
//uint32_t Permenu_action ;

int16_t g_ppmIns[8];
uint8_t ppmInState = 0; //0=unsync 1..8= wait for value i-1


EEGeneral  g_eeGeneral;
ModelData  g_oldmodel;
SKYModelData  g_model;

const uint8_t chout_ar[] = { //First number is 0..23 -> template setup,  Second is relevant channel out
                                1,2,3,4 , 1,2,4,3 , 1,3,2,4 , 1,3,4,2 , 1,4,2,3 , 1,4,3,2,
                                2,1,3,4 , 2,1,4,3 , 2,3,1,4 , 2,3,4,1 , 2,4,1,3 , 2,4,3,1,
                                3,1,2,4 , 3,1,4,2 , 3,2,1,4 , 3,2,4,1 , 3,4,1,2 , 3,4,2,1,
                                4,1,2,3 , 4,1,3,2 , 4,2,1,3 , 4,2,3,1 , 4,3,1,2 , 4,3,2,1    };
const uint8_t bchout_ar[] = {
															0x1B, 0x1E, 0x27, 0x2D, 0x36, 0x39,
															0x4B, 0x4E, 0x63, 0x6C, 0x72, 0x78,
                              0x87, 0x8D, 0x93, 0x9C, 0xB1, 0xB4,
                              0xC6, 0xC9, 0xD2, 0xD8, 0xE1, 0xE4		} ;


//new audio object
audioQueue  audio;


uint8_t AlarmTimer = 100 ;		// Units of 10 mS
uint8_t AlarmCheckFlag = 0 ;
uint8_t CsCheckFlag = 0 ;
uint8_t VoiceTimer = 10 ;		// Units of 10 mS
uint8_t VoiceCheckFlag = 0 ;
int8_t  CsTimer[NUM_SKYCSW] ;

const char modi12x3[]=
  "RUD ELE THR AIL "
  "RUD THR ELE AIL "
  "AIL ELE THR RUD "
  "AIL THR ELE RUD ";
// Now indexed using modn12x3

MenuFuncP g_menuStack[5];

uint8_t  g_menuStackPtr = 0;


// Temporary to allow compile
uint8_t g_vbat100mV = 98 ;
//int16_t calibratedStick[7] ;
//int16_t g_chans512[NUM_SKYCHNOUT] ;
uint8_t heartbeat ;
uint8_t heartbeat_running ;

uint16_t ResetReason ;

//uint8_t Timer2_running = 0 ;
//uint8_t Timer2_pre = 0 ;
//uint16_t Timer2 = 0 ;


/*
//#define DO_CROSS(xx,yy,ww)          \
//    lcd_vline(xx,yy-ww/2,ww);  \
//    lcd_hline(xx-ww/2,yy,ww);  \

//#define V_BAR(xx,yy,ll)       \
//    lcd_vline(xx-1,yy-ll,ll); \
//    lcd_vline(xx  ,yy-ll,ll); \
//    lcd_vline(xx+1,yy-ll,ll); \
	*/

static void checkAlarm() // added by Gohst
{
    if(g_eeGeneral.disableAlarmWarning) return;
    if(!g_eeGeneral.beeperVal) alert(PSTR("Alarms Disabled"));
}

static void checkWarnings()
{
    if(sysFlags && sysFLAG_OLD_EEPROM)
    {
        alert(PSTR(" Old Version EEPROM   CHECK SETTINGS/CALIB")); //will update on next save
        sysFlags &= ~(sysFLAG_OLD_EEPROM); //clear flag
    }
}

inline uint8_t keyDown()
{
    return ~read_keys() & 0x7E ;
}

void clearKeyEvents()
{
    while(keyDown())
		{
			  // loop until all keys are up
			if ( PIOC->PIO_PDSR & 0x02000000 )
			{
				// Detected USB
				break ;
			}
			if ( heartbeat_running )
			{
  			if(heartbeat == 0x3)
  			{
  			  heartbeat = 0;
  			  wdt_reset();
  			}
			}
			else
			{
				wdt_reset() ;
			}
		}	
    putEvent(0);
}


/*=========================================================================*/
/*  DEFINE: All code exported                                              */
/*=========================================================================*/

/***************************************************************************/
/*  main                                                                   */
/***************************************************************************/

// extern uint32_t __RBIT(uint32_t value);


// WARNING check MATRIX->CCFG_SYSIO doesn't BLOCK I/O lines


// Still to test

// PPM-in						PC23 TIOA3
// SSC out					PA17 TD (sync PPM)

// Serial ports
// PA5, PA6
// PB2, PB3


// Tested
// RF-power-in			PC17	(prototype)
// PPM-jack-in			PC19	(prototype)

// The stock Beeper is on PA16 on the prototype board, use PA25 (LCD_CS2) on REVB.


//extern uint8_t CustomDisplayIndex[6] ;

#ifdef PCBSKY
#define BT_115200		0
#define BT_9600			1
#define BT_19200		2

void setBtBaudrate( uint32_t index )
{
	uint32_t brate ;
	switch ( index )
	{
		default :
		case 0 :
			brate = 115200 ;
		break ;
		case 1 :
			brate = 9600 ;
		break ;
		case 2 :
			brate = 19200 ;
		break ;
	}
	UART3_Configure( brate, Master_frequency ) ;		// Testing
}
#endif


int main(void)
{
//	register uint32_t goto_usb ;
	register Pio *pioptr ;
	// Debug variable
//	uint32_t both_on ;

	ResetReason = RSTC->RSTC_MR ;

	MATRIX->CCFG_SYSIO |= 0x000000F0L ;		// Disable syspins, enable B4,5,6,7

  PMC->PMC_PCER0 = (1<<ID_PIOC)|(1<<ID_PIOB)|(1<<ID_PIOA)|(1<<ID_UART0) ;				// Enable clocks to PIOB and PIOA and PIOC and UART0
	pioptr = PIOA ;
#ifdef REVB	
	init_soft_power() ;
#else	
	// On REVB, PA21 is used as AD8, and measures current consumption.
	pioptr->PIO_PER = PIO_PA21 ;		// Enable bit A21 (EXT3)
	pioptr->PIO_OER = PIO_PA21 ;		// Set bit A21 as output
	pioptr->PIO_SODR = PIO_PA21 ;	// Set bit A21 ON
#endif

	pioptr = PIOC ;
	pioptr->PIO_PER = PIO_PC25 ;		// Enable bit C25 (USB-detect)

	if ( ( pioptr->PIO_PDSR & 0x02000000 ) == 0 )
	{
		// USB not the power source
		WDT->WDT_MR = 0x3FFF217F ;				// Enable watchdog 1.5 Secs
	}

#ifdef REVB	
#else	
	// Configure RF_power (PC17) and PPM-jack-in (PC19), neither need pullups
	pioptr->PIO_PER = 0x000A0000L ;		// Enable bit C19, C17
	pioptr->PIO_ODR = 0x000A0000L ;		// Set bits C19 and C17 as input
#endif

	config_free_pins() ;

	init_keys() ;

	setup_switches() ;

  // Enable PCK2 on PB3, This is for testing of Timer 2 working
	// It will be used as serial data to the Bluetooth module
	pioptr->PIO_ABCDSR[0] |=  PIO_PB3 ;	// Peripheral B
  pioptr->PIO_ABCDSR[1] &= ~PIO_PB3 ;	// Peripheral B
  pioptr->PIO_PDR = PIO_PB3 ;					// Assign to peripheral
	PMC->PMC_SCER |= 0x0400 ;								// PCK2 enabled
	PMC->PMC_PCK[2] = 2 ;										// PCK2 is PLLA

	UART_Configure( 9600, Master_frequency ) ;
	UART2_Configure( 9600, Master_frequency ) ;		// Testing

	init5msTimer() ;
	
	start_timer0() ;
	init_adc() ;
	init_pwm() ;
#ifndef SIMU
	init_SDcard() ;
#endif

	__enable_irq() ;

	lcd_init() ;

  g_menuStack[0] =  menuProc0 ;

	start_sound() ;
	
	init_spi() ;
		
	init_eeprom() ;	
	
	if ( ( ( ResetReason & RSTC_SR_RSTTYP ) != (2 << 8) ) && !unexpectedShutdown )	// Not watchdog
	{
		pioptr = PIOC ;
		if ( pioptr->PIO_PDSR & 0x02000000 )
		{
			g_eeGeneral.optrexDisplay = 1 ;
			lcd_clear() ;
			refreshDisplay() ;
			g_eeGeneral.optrexDisplay = 0 ;
			actionUsb() ;
		}
	}
		 
	eeReadAll() ;
	if ( g_eeGeneral.unexpectedShutdown )
	{
		unexpectedShutdown = 1 ;
	}

#ifdef PCBSKY
	setBtBaudrate( g_eeGeneral.bt_baudrate ) ;
	// Set ADC gains here
	set_stick_gain( g_eeGeneral.stickGain ) ;
#endif

#ifdef FRSKY
  FRSKY_Init();
#endif

  checkQuickSelect();
	 
  lcdSetRefVolt(g_eeGeneral.contrast) ;
	set_volume( g_eeGeneral.volume ) ;
	PWM->PWM_CH_NUM[0].PWM_CDTYUPD = g_eeGeneral.bright ;
	MAh_used = g_eeGeneral.mAh_used ;

	// Choose here between PPM and PXX
	
  g_menuStack[1] = menuProcModelSelect ;	// this is so the first instance of [MENU LONG] doesn't freak out!

  //we assume that startup is like pressing a switch and moving sticks.  Hence the lightcounter is set
  //if we have a switch on backlight it will be able to turn on the backlight.
  if(g_eeGeneral.lightAutoOff > g_eeGeneral.lightOnStickMove)
    g_LightOffCounter = g_eeGeneral.lightAutoOff*500;
  if(g_eeGeneral.lightAutoOff < g_eeGeneral.lightOnStickMove)
    g_LightOffCounter = g_eeGeneral.lightOnStickMove*500;
  check_backlight();

  // moved here and logic added to only play statup tone if splash screen enabled.
  // that way we save a bit, but keep the option for end users!
  if((g_eeGeneral.speakerMode & 1) == 1)
	{
    if(!g_eeGeneral.disableSplashScreen)
    {
			audioVoiceDefevent( AU_TADA, V_HELLO ) ;
    }
  }
	if ( ( ( ResetReason & RSTC_SR_RSTTYP ) != (2 << 8) ) || unexpectedShutdown )	// Not watchdog
	{
		doSplash() ;
  	getADC_single();
  	checkTHR();
  	checkSwitches();
		checkAlarm();
		checkWarnings();
		clearKeyEvents(); //make sure no keys are down before proceeding

		putVoiceQueue( g_model.modelVoice + 260 ) ;
	}

	// Must do this to start PPM2 as well
	init_main_ppm( 3000, 1 ) ;		// Default for now, initial period 1.5 mS, output on

	start_ppm_capture() ;

  FrskyAlarmSendState |= 0x40 ;

	heartbeat_running = 1 ;

  if (!g_eeGeneral.unexpectedShutdown)
	{
    g_eeGeneral.unexpectedShutdown = 1;
    STORE_GENERALVARS ;
  }

#ifndef SIMU

	CoInitOS();

#ifdef PCBSKY
	BtTask = CoCreateTask(bt_task,NULL,19,&Bt_stk[BT_STACK_SIZE-1],BT_STACK_SIZE);
#endif

	MainTask = CoCreateTask( main_loop,NULL,5,&main_stk[MAIN_STACK_SIZE-1],MAIN_STACK_SIZE);

	VoiceTask = CoCreateTaskEx( voice_task,NULL,17,&voice_stk[VOICE_STACK_SIZE-1], VOICE_STACK_SIZE, 1, FALSE );

#ifdef	DEBUG

	DebugTask = CoCreateTaskEx( handle_serial,NULL,18,&debug_stk[DEBUG_STACK_SIZE-1],DEBUG_STACK_SIZE, 1, FALSE );

#endif 
	init_rotary_encoder() ;

	CoStartOS();

	while(1);
#endif
  /*
   * Prevent compiler warnings
   */
//  (void)c;

  /*
   * This return here make no sense.
   * But to prevent the compiler warning:
   * "return type of 'main' is not 'int'
   * we use an int as return :-)
   */
  return(0);
}

#ifndef SIMU
//void tmrBt_Handle( void )
//{
//	CoSetFlag(Bt1SFlag);		// 1 second return,set flag
//}

#ifdef PCBSKY
OS_FlagID Bt_flag ;
struct t_fifo32 Bt_fifo ;
struct t_serial_tx Bt_tx ;
uint8_t BtTxBuffer[32] ;

void bt_send_buffer()
{
	Bt_tx.buffer = BtTxBuffer ;
	txPdcBt( &Bt_tx ) ;
	while ( Bt_tx.ready == 1 )
	{
		// Wait
		CoTickDelay(1) ;					// 2mS for now
	}
	Bt_tx.size = 0 ;
}

#define BT_POLL_TIMEOUT		500


uint32_t poll_bt_device()
{
	uint16_t x ;
	uint32_t y ;
	uint16_t rxchar ;

	x = 'O' ;
	BtTxBuffer[0] = 'A' ;
	BtTxBuffer[1] = 'T' ;
	Bt_tx.size = 2 ;
	bt_send_buffer() ;
	for( y = 0 ; y < BT_POLL_TIMEOUT ;  y += 1 )
	{
		if ( ( rxchar = rxBtuart() ) != 0xFFFF )
		{
//			rxDebugproc( rxchar ) ;
			if ( rxchar == x )
			{
				if ( x == 'O' )
				{
					x = 'K' ;
				}
				else
				{
					break ;			// Found "OK"
				}
			}
		}
		else
		{
			CoTickDelay(1) ;					// 2mS
		}				 
	}
//	if ( ( rxchar = rxBtuart() ) != 0xFFFF )
//	{
//		rxDebugproc( rxchar ) ;
//	}
	if ( y < BT_POLL_TIMEOUT )
	{
		return 1 ;
	}
	else
	{
		return 0 ;
	}
}

uint32_t changeBtBaudrate( uint32_t baudIndex )
{
	uint16_t x ;
	uint32_t y ;
	uint16_t rxchar ;

//	if ( ( rxchar = rxBtuart() ) != 0xFFFF )
//	{
//		rxDebugproc( rxchar ) ;
//	}
	x = 4 ;		// 9600
	if ( baudIndex == 0 )
	{
		x = 8 ;		// 115200
	}
	else if ( baudIndex == 2 )
	{
		x = 5 ;		// 19200		
	}
	BtTxBuffer[0] = 'A' ;
	BtTxBuffer[1] = 'T' ;
	BtTxBuffer[2] = '+' ;
	BtTxBuffer[3] = 'B' ;
	BtTxBuffer[4] = 'A' ;
	BtTxBuffer[5] = 'U' ;
	BtTxBuffer[6] = 'D' ;
	BtTxBuffer[7] = '0' + x ;
	Bt_tx.size = 8 ;
	bt_send_buffer() ;
	x = 'O' ;
	for( y = 0 ; y < BT_POLL_TIMEOUT ;  y += 1 )
	{
		if ( ( rxchar = rxBtuart() ) != 0xFFFF )
		{
			if ( rxchar == x )
			{
				if ( x == 'O' )
				{
					x = 'K' ;
				}
				else
				{
					break ;			// Found "OK"
				}
			}
		}
		else
		{
			CoTickDelay(1) ;					// 2mS
		}				 
	}
//	if ( ( rxchar = rxBtuart() ) != 0xFFFF )
//	{
//		rxDebugproc( rxchar ) ;
//	}
	if ( y < BT_POLL_TIMEOUT )
	{
		return 1 ;
	}
	else
	{
		return 0 ;
	}
}


/*
Commands to BT module
AT+VERSION 	Returns the software version of the module
AT+BAUDx 	Sets the baud rate of the module:
1 	1200
2 	2400
3 	4800
4 	9600 (Default)
5 	19200
6 	38400
7 	57600
8 	115200
9 	230400
AT+NAME<name here> 	Sets the name of the module

Any name can be specified up to 20 characters
AT+PINxxxx 	Sets the pairing password of the device

Any 4 digit number can be used, the default pincode is 1234
AT+PN 	Sets the parity of the module 

So we could send AT+VERSION at different baudrates until we get a response
Then we can change the baudrate to the required value.
Or maybe just AT and get OK back
*/

uint32_t Bt_ok ;

void bt_task(void* pdata)
{
	uint32_t x ;
	int32_t y ;
//	static uint32_t count ;
	Bt_flag = CoCreateFlag(TRUE,0) ;
	Bt_tx.size = 0 ;

// Look for BT module baudrate, try 115200, and 9600
// Already initialised to g_eeGeneral.bt_baudrate
// 0 : 115200, 1 : 9600, 2 : 19200
	
	x = g_eeGeneral.bt_baudrate ;

//	rxDebugproc( 0x55 ) ;

	Bt_ok = poll_bt_device() ;		// Do we get a response?

	for ( y = 0 ; y < 2 ; y += 1 )
	{
		if ( Bt_ok == 0 )
		{
			x += 1 ;
			if ( x > 2 )
			{
				x = 0 ;			
			}
			setBtBaudrate( x ) ;
			CoTickDelay(1) ;					// 2mS
			Bt_ok = poll_bt_device() ;		// Do we get a response?
		}
	}

	if ( Bt_ok )
	{
		Bt_ok = x + 1 ;		
		if ( x != g_eeGeneral.bt_baudrate )
		{
			x = g_eeGeneral.bt_baudrate ;
			// Need to change Bt Baudrate
			changeBtBaudrate( x ) ;
			Bt_ok += (x+1) * 10 ;
			setBtBaudrate( x ) ;
		}
	}
	else
	{
		setBtBaudrate( g_eeGeneral.bt_baudrate ) ;
	}
	CoTickDelay(1) ;					// 2mS

	while(1)
	{
		x = CoWaitForSingleFlag( Bt_flag, 10 ) ;		// Wait for data in Fifo
		if ( x == E_OK )
		{
			// We have some data in the Fifo
			while ( ( y = get_fifo32( &Bt_fifo ) ) != -1 )
			{
				BtTxBuffer[Bt_tx.size++] = y ;
				if ( Bt_tx.size > 31 )
				{
					bt_send_buffer() ;
				}
			}
		}
		else if ( Bt_tx.size )
		{
			bt_send_buffer() ;
		}
//		txmitBt( 'X' ) ;		// Send an X to Bluetooth every second for testing
	}
}
#endif

#endif

void telem_byte_to_bt( uint8_t data )
{
#ifndef SIMU
        put_fifo32( &Bt_fifo, data ) ;
        CoSetFlag( Bt_flag ) ;                  // Tell the Bt task something to do
#endif
}

uint32_t UsbTimer = 0 ;
extern void usbMassStorage( void ) ;

// This is the main task for the RTOS
void main_loop(void* pdata)
{
	register uint32_t goto_usb ;
	register Pio *pioptr ;
	
	goto_usb = 0 ;
  while (1)
  {
//	  PMC->PMC_PCER0 = 0x1800 ;				// Enable clocks to PIOB and PIOA
//		PIOA->PIO_PER = 0x04200000L ;		// Enable bit A21 (EXT3), A26 (EXT2)
//		PIOA->PIO_OER = 0x04200000L ;		// Set bits A21, A26 as output

//		PIOB->PIO_PUER = 0x40 ;					// Enable pullup on bit B6
//		PIOB->PIO_PER = 0x40 ;					// Enable bit B6
//		PIOA->PIO_PUER = 0x80000000 ;		// Enable pullup on bit A31 (EXIT)
//		PIOA->PIO_PER = 0x80000000 ;		// Enable bit A31


		if ( UsbTimer < 1000 )		// 2 Seconds
		{
			UsbTimer += 1 ;
			pioptr = PIOC ;
			if ( pioptr->PIO_PDSR & 0x02000000 )
			{
				// Detected USB
				goto_usb = 1 ;
			}
		}
		else
		{
#ifndef SIMU
     	usbMassStorage() ;
#endif
		}
  
#ifdef REVB	
		if ( ( check_soft_power() == POWER_OFF ) || ( goto_usb ) )		// power now off
		{
			// Time to switch off
			lcd_clear() ;
			lcd_putsn_P( 4*FW, 3*FH, "SHUTTING DOWN", 13 ) ;
			if ( goto_usb )
			{
				lcd_putsn_P( 7*FW, 4*FH, "TO USB", 6 ) ;
			}
			refreshDisplay() ;

			// Wait for OK to turn off
			// Currently wait 1 sec, needs to check eeprom finished

  		g_eeGeneral.unexpectedShutdown = 0;
			MAh_used += Current_used/22/36 ;
			if ( g_eeGeneral.mAh_used != MAh_used )
			{
				g_eeGeneral.mAh_used = MAh_used ;
			}
      STORE_GENERALVARS ;		// To make sure we write "unexpectedShutdown"

  		uint16_t tgtime = get_tmr10ms() ;
	  	while( (get_tmr10ms() - tgtime ) < 50 ) // 50 - Half second
  		{
				if ( check_soft_power() )
				{
					break ;		// Power back on
				}
				wdt_reset() ;

				if ( ee32_check_finished() == 0 )
				{
					lcd_putsn_P( 5*FW, 5*FH, "EEPROM BUSY", 11 ) ;
					tgtime = get_tmr10ms() ;
				}
				else
				{
					lcd_putsn_P( 5*FW, 5*FH, "           ", 11 ) ;
				}
				refreshDisplay() ;
  		}
//			if ( check_soft_power() == 0 )
//			{
				lcd_clear() ;
				refreshDisplay() ;
			  lcdSetRefVolt(0);
				soft_power_off() ;		// Only turn power off if necessary

//			}
		}
#endif
		if ( goto_usb )
		{
			break ;		
		}
		mainSequence( MENUS ) ;
#ifndef SIMU
		CoTickDelay(1) ;					// 2mS for now
#endif
	}

	RSTC->RSTC_CR = 0xA5000000 | RSTC_CR_PROCRST | RSTC_CR_PERRST ;
//	actionUsb() ;
}
	
void actionUsb()
{
#if !defined(SIMU)
	lcd_clear() ;
	lcd_putcAtt( 48, 24, 'U', DBLSIZE ) ;
	lcd_putcAtt( 60, 24, 'S', DBLSIZE ) ;
	lcd_putcAtt( 72, 24, 'B', DBLSIZE ) ;
	refreshDisplay() ;

	// This might be replaced by a software reset
	// Any interrupts that have been enabled must be disabled here
	// BEFORE calling sam_boot()
	SysTick->CTRL = 0 ;				// Turn off systick

	stop_rotary_encoder() ;
	endPdcUsartReceive() ;		// Terminate any serial reception
	end_bt_tx_interrupt() ;
	soft_power_off() ;
	end_ppm_capture() ;
	end_spi() ;
	end_sound() ;
	TC0->TC_CHANNEL[2].TC_IDR = TC_IDR0_CPCS ;
	stop_timer0() ;
	TC0->TC_CHANNEL[1].TC_CCR = TC_CCR0_CLKDIS ;
	stop5msTimer() ;
	TC0->TC_CHANNEL[2].TC_CCR = TC_CCR0_CLKDIS ;
	TC1->TC_CHANNEL[0].TC_CCR = TC_CCR0_CLKDIS ;
	TC1->TC_CHANNEL[1].TC_CCR = TC_CCR0_CLKDIS ;
	TC1->TC_CHANNEL[2].TC_CCR = TC_CCR0_CLKDIS ;
	PWM->PWM_DIS = PWM_DIS_CHID0 | PWM_DIS_CHID1 | PWM_DIS_CHID2 | PWM_DIS_CHID3 ;	// Disable all
//	PWM->PWM_IDR1 = PWM_IDR1_CHID0 ;
	disable_main_ppm() ;
	disable_ppm2() ;

//	PWM->PWM_IDR1 = PWM_IDR1_CHID3 ;
//	NVIC_DisableIRQ(PWM_IRQn) ;
	disable_ssc() ;
	UART_Stop() ;
	Bt_UART_Stop() ;
	sam_boot() ;
#endif
}

static inline uint16_t getTmr2MHz()
{
	return TC1->TC_CHANNEL[0].TC_CV ;
}

uint32_t OneSecTimer ;

#ifdef FRSKY
extern int16_t AltOffset ;
#endif

//uint16_t Debug_analog[8] ;

void mainSequence( uint32_t no_menu )
{
	static uint32_t coProTimer = 0 ;
  uint16_t t0 = getTmr2MHz();
	uint8_t numSafety = NUM_SKYCHNOUT - g_model.numVoice ;
	
	if ( g_eeGeneral.filterInput == 1 )
	{
		getADC_osmp() ;
	}
	else if ( g_eeGeneral.filterInput == 2 )
	{
		getADC_filt() ;
	}
	else
	{
		getADC_single() ;
	}
	Current_analogue = ( Current_analogue * 31 + S_anaFilt[8] ) >> 5 ;
	if ( Current_analogue > Current_max )
	{
		Current_max = Current_analogue ;		
	}

//	// ADC debug code
//	{
//		uint32_t x ;
//		for( x = 0 ; x < 8 ; x += 1 )
//		{
//			read_9_adc() ;
//			Debug_analog[x] = Analog_values[0] ;
//		}
//	}
//	// End debug code

	perMain( no_menu ) ;		// Allow menu processing

  if(heartbeat == 0x3)
  {
    wdt_reset();
    heartbeat = 0;
  }
  

	if ( Tenms )
	{
		Tenms = 0 ;
		ee32_process() ;
		Current_accumulator += Current_analogue ;
		if ( ++OneSecTimer >= 100 )
		{
			OneSecTimer -= 100 ;
//			txmitBt( 'X' ) ;		// Send an X to Bluetooth every second for testing
			Current_used += Current_accumulator / 100 ;			// milliAmpSeconds (but scaled)
			Current_accumulator = 0 ;
		}
#ifndef SIMU
		sd_poll_10mS() ;
#endif

 #ifdef PCBSKY
		if ( ++coProTimer > 24 )
		{
			coProTimer -= 25 ;
			read_coprocessor() ;
		}
	}
#endif

	t0 = getTmr2MHz() - t0;
  if ( t0 > g_timeMain ) g_timeMain = t0 ;
  if ( AlarmCheckFlag > 1 )
  {
    AlarmCheckFlag = 0 ;
    // Check for alarms here
    // Including Altitude limit


#ifdef FRSKY
    if (frskyUsrStreaming)
    {
      int16_t limit = g_model.FrSkyAltAlarm ;
      int16_t altitude ;
      if ( limit )
      {
        if (limit == 2)  // 400
        {
          limit = 400 ;	//ft
        }
        else
        {
          limit = 122 ;	//m
        }
				altitude = FrskyHubData[FR_ALT_BARO] + AltOffset ;
				altitude /= 10 ;									
				if (g_model.FrSkyUsrProto == 0)  // Hub
				{
      		if ( g_model.FrSkyImperial )
					{
        		altitude = m_to_ft( altitude ) ;
					}
				}
        if ( altitude > limit )
        {
          audioDefevent(AU_WARNING2) ;
        }
      }
			uint16_t total_volts = 0 ;
			uint8_t audio_sounded = 0 ;
			uint8_t low_cell = 220 ;		// 4.4V
			for (uint8_t k=0; k<FrskyBattCells; k++)
			{
				total_volts += FrskyVolts[k] ;
				if ( FrskyVolts[k] < low_cell )
				{
					low_cell = FrskyVolts[k] ;
				}

				if ( audio_sounded == 0 )
				{
		      if ( FrskyVolts[k] < g_model.frSkyVoltThreshold )
					{
		        audioDefevent(AU_WARNING3);
						audio_sounded = 1 ;
					}
			  }
	  	}
			// Now we have total volts available
			FrskyHubData[FR_CELLS_TOT] = total_volts / 5 ;
			if ( low_cell < 220 )
			{
				FrskyHubData[FR_CELL_MIN] = low_cell ;
			}
    }


    // this var prevents and alarm sounding if an earlier alarm is already sounding
    // firing two alarms at once is pointless and sounds rubbish!
    // this also means channel A alarms always over ride same level alarms on channel B
    // up to debate if this is correct!
    //				bool AlarmRaisedAlready = false;

    if (frskyStreaming)
		{
      enum AlarmLevel level[4] ;
      // RED ALERTS
      if( (level[0]=FRSKY_alarmRaised(0,0)) == alarm_red) FRSKY_alarmPlay(0,0);
      else if( (level[1]=FRSKY_alarmRaised(0,1)) == alarm_red) FRSKY_alarmPlay(0,1);
      else	if( (level[2]=FRSKY_alarmRaised(1,0)) == alarm_red) FRSKY_alarmPlay(1,0);
      else if( (level[3]=FRSKY_alarmRaised(1,1)) == alarm_red) FRSKY_alarmPlay(1,1);
      // ORANGE ALERTS
      else	if( level[0] == alarm_orange) FRSKY_alarmPlay(0,0);
      else if( level[1] == alarm_orange) FRSKY_alarmPlay(0,1);
      else	if( level[2] == alarm_orange) FRSKY_alarmPlay(1,0);
      else if( level[3] == alarm_orange) FRSKY_alarmPlay(1,1);
      // YELLOW ALERTS
      else	if( level[0] == alarm_yellow) FRSKY_alarmPlay(0,0);
      else if( level[1] == alarm_yellow) FRSKY_alarmPlay(0,1);
      else	if( level[2] == alarm_yellow) FRSKY_alarmPlay(1,0);
      else if( level[3] == alarm_yellow) FRSKY_alarmPlay(1,1);
						
			// Check for current alarm
      for (int i=0; i<2; i++)
			{
				// To be enhanced by checking the type as well
       	if (g_model.frsky.channels[i].ratio)
				{
     		  if ( g_model.frsky.channels[i].type == 3 )		// Current (A)
					{
						if ( g_model.frskyAlarms.alarmData[0].frskyAlarmLimit )
						{
    		      if ( ( FrskyHubData[FR_A1_MAH+i] >> 6 ) >= g_model.frskyAlarms.alarmData[0].frskyAlarmLimit )
							{
								if ( g_eeGeneral.speakerMode & 2 )
								{
									putVoiceQueue( V_CAPACITY ) ;
								}
								else
								{
									audio.event( g_model.frskyAlarms.alarmData[0].frskyAlarmSound ) ;
								}
							}
						}
					}
       	}
      }
    }
#endif
		
		// Now for the Safety/alarm switch alarms
		{
			uint8_t i ;
			static uint8_t periodCounter ;
			
			periodCounter += 0x11 ;
			periodCounter &= 0xF7 ;
			if ( periodCounter > 0x5F )
			{
				periodCounter &= 0x0F ;
			}
			for ( i = 0 ; i < numSafety ; i += 1 )
			{
    		SKYSafetySwData *sd = &g_model.safetySw[i] ;
				if (sd->opt.ss.mode == 1)
				{
					if(getSwitch( sd->opt.ss.swtch,0))
					{
						audio.event( ((g_eeGeneral.speakerMode & 1) == 0) ? 1 : sd->opt.ss.val ) ;
					}
				}
				if (sd->opt.ss.mode == 2)
				{
					if ( sd->opt.ss.swtch > MAX_SKYDRSWITCH )
					{
						switch ( sd->opt.ss.swtch - MAX_SKYDRSWITCH -1 )
						{
							case 0 :
								if ( ( periodCounter & 3 ) == 0 )
								{
									voice_telem_item( sd->opt.ss.val ) ;
								}
							break ;
							case 1 :
								if ( ( periodCounter & 0xF0 ) == 0 )
								{
									voice_telem_item( sd->opt.ss.val ) ;
								}
							break ;
							case 2 :
								if ( ( periodCounter & 7 ) == 2 )
								{
									voice_telem_item( sd->opt.ss.val ) ;
								}
							break ;
						}
					}
					else if ( ( periodCounter & 1 ) == 0 )		// Every 4 seconds
					{
						if(getSwitch( sd->opt.ss.swtch,0))
						{
							putVoiceQueue( sd->opt.ss.val + 128 ) ;
						}
					}
				}
			}
		}
  }
	// New switch voices
	// New entries, Switch, (on/off/both), voice file index

  if ( VoiceCheckFlag )
  {
		uint32_t i ;
		static uint32_t timer ;
    
		timer += 1 ;
		
		for ( i = numSafety ; i < NUM_SKYCHNOUT ; i += 1 )
		{
			uint8_t curent_state ;
			uint8_t mode ;
    	SKYSafetySwData *sd = &g_model.safetySw[i];
			
			mode = sd->opt.vs.vmode ;
			if ( sd->opt.vs.vswtch )		// Configured
			{
				curent_state = getSwitch( sd->opt.vs.vswtch, 0 ) ;
				if ( VoiceCheckFlag != 2 )
				{
					if ( ( mode == 0 ) || ( mode == 2 ) )
					{ // ON
						if ( ( Vs_state[i] == 0 ) && curent_state )
						{
							putVoiceQueue( sd->opt.vs.vval ) ;
						}
					}
					if ( ( mode == 1 ) || ( mode == 2 ) )
					{ // OFF
						if ( ( Vs_state[i] == 1 ) && !curent_state )
						{
							uint8_t x ;
							x = sd->opt.vs.vval ;
							if ( mode == 2 )
							{
								x += 1 ;							
							}
							putVoiceQueue( x ) ;
						}
					}
					if ( mode > 5 )
					{
						if ( ( Vs_state[i] == 0 ) && curent_state )
						{
							voice_telem_item( sd->opt.vs.vval ) ;
						}					
					}
					else if ( mode > 2 )
					{ // 15, 30 or 60 secs
						if ( curent_state )
						{
							uint16_t mask ;
							mask = 150 ;
							if ( mode == 4 ) mask = 300 ;
							if ( mode == 5 ) mask = 600 ;
							if ( timer % mask == 0 )
							{
								putVoiceQueue( sd->opt.vs.vval ) ;
							}
						}
					}
				}
				Vs_state[i] = curent_state ;
			}
		}
		VoiceCheckFlag = 0 ;
	}
	if ( CsCheckFlag )		// Custom Switch Timers
	{
		CsCheckFlag = 0 ;
		uint8_t i ;
		
		for ( i = 0 ; i < NUM_SKYCSW ; i += 1 )
		{
    	SKYCSwData &cs = g_model.customSw[i];
    	uint8_t cstate = CS_STATE(cs.func);

    	if(cstate == CS_TIMER)
			{
				if ( CsTimer[i] == 0 )
				{
					CsTimer[i] = -cs.v1-1 ;
				}
				else if ( CsTimer[i] < 0 )
				{
					if ( ++CsTimer[i] == 0 )
					{
						CsTimer[i] = cs.v2 ;
					}
				}
				else  // if ( CsTimer[i] > 0 )
				{
					CsTimer[i] -= 1 ;
				}
				if ( cs.andsw )
				{
					int8_t x ;
					x = cs.andsw ;
					if ( x > 8 )
					{
						x += 1 ;
					}
					if ( x < -8 )
					{
						x -= 1 ;
					}
	        if (getSwitch( x, 0, 0) == 0 )
				  {
						CsTimer[i] = -1 ;
					}	
				}
			}
		}
	}
}

uint32_t check_power_or_usb()
{
#ifndef SIMU
	if ( check_soft_power() == POWER_OFF )		// power now off
	{
		return 1 ;
	}
	if ( PIOC->PIO_PDSR & 0x02000000 )
	{
		return 1 ;			// Detected USB
	}
#endif
	return 0 ;
}

void check_backlight()
{
  if(getSwitch(g_eeGeneral.lightSw,0) || g_LightOffCounter)
    BACKLIGHT_ON;
  else
    BACKLIGHT_OFF;
}

uint16_t stickMoveValue()
{
#define INAC_DEVISOR 256   // Issue 206 - bypass splash screen with stick movement
    uint16_t sum = 0;
    for(uint8_t i=0; i<4; i++)
        sum += anaIn(i)/INAC_DEVISOR;
    return sum ;
}

void doSplash()
{
	uint32_t i ;

extern uint8_t DisplayBuf[] ;
  
	if(!g_eeGeneral.disableSplashScreen)
  {
   	check_backlight() ;
    lcd_clear();
		refreshDisplay();
    lcdSetRefVolt(g_eeGeneral.contrast);
  	clearKeyEvents();

#ifndef SIMU
  	for( i=0; i<32; i++)
    	getADC_filt(); // init ADC array
#endif

  	uint16_t inacSum = stickMoveValue();

  	

		//        for(uint8_t i=0; i<4; i++)
		//           inacSum += anaIn(i)/INAC_DEVISOR;

  	uint16_t tgtime = get_tmr10ms() + SPLASH_TIMEOUT;  
  	uint16_t scrtime = get_tmr10ms() ;
//		lcd_clear();
// 		lcd_img(0, 0, &splashdata[4],0,0);
//  	refreshDisplay();
		
		i = 0 ;
  	while(tgtime > get_tmr10ms())
  	{
			if ( scrtime < get_tmr10ms() )
			{
				scrtime += 4 ;
				if ( i < 128 )
				{
					uint8_t *p ;
					uint8_t *q ;
					uint8_t x ;
					uint8_t y ;
					uint8_t z ;
					i += 4 ;
					lcd_clear();
  	 			lcd_img(0, 0, &splashdata[4],0,0);
					for ( y = 0 ; y < 8 ; y += 1 )
					{
						z = 128 - i ;
						p = &DisplayBuf[y*128];
						q = p + z ;
						for ( x = 0 ; x < i ; x += 1 )
						{
							*p++ = *q++ ;
						}
						while( x < 128 )
						{
							*p++ = 0 ;
							x += 1 ;
						}				
					}
					if ( i >= 128 )
					{
						if(!g_eeGeneral.hideNameOnSplash)
						lcd_putsnAtt( 0*FW, 7*FH, g_eeGeneral.ownerName ,sizeof(g_eeGeneral.ownerName),0);
					} 
  				refreshDisplay();
				}
			}

#ifdef SIMU
        if (!main_thread_running) return;
        sleep(1/*ms*/);
#else
        getADC_filt();
#endif

    	uint16_t tsum = stickMoveValue();
	//    for(uint8_t i=0; i<4; i++)
	//       tsum += anaIn(i)/INAC_DEVISOR;
    
			if(keyDown() || (tsum!=inacSum))   return;  //wait for key release

			if ( check_power_or_usb() ) return ;		// Usb on or power off

			check_backlight() ;
  	  wdt_reset();
  	}
	}
}


//global helper vars
bool    checkIncDec_Ret;
int16_t p1val;
int16_t p1valdiff;

int16_t checkIncDec16(uint8_t event, int16_t val, int16_t i_min, int16_t i_max, uint8_t i_flags)
{
  int16_t newval = val;
  uint8_t kpl=KEY_RIGHT, kmi=KEY_LEFT, kother = -1;
	uint8_t skipPause = 0 ;

  if(event & _MSK_KEY_DBL){
    uint8_t hlp=kpl;
    kpl=kmi;
    kmi=hlp;
    event=EVT_KEY_FIRST(EVT_KEY_MASK & event);
  }
  if(event==EVT_KEY_FIRST(kpl) || event== EVT_KEY_REPT(kpl) || (s_editMode && (event==EVT_KEY_FIRST(KEY_UP) || event== EVT_KEY_REPT(KEY_UP))) ) {
    newval++;

		audioDefevent(AU_KEYPAD_UP);

    kother=kmi;
  }else if(event==EVT_KEY_FIRST(kmi) || event== EVT_KEY_REPT(kmi) || (s_editMode && (event==EVT_KEY_FIRST(KEY_DOWN) || event== EVT_KEY_REPT(KEY_DOWN))) ) {
    newval--;

		audioDefevent(AU_KEYPAD_DOWN);

    kother=kpl;
  }
  if((kother != (uint8_t)-1) && keyState((EnumKeys)kother)){
    newval=-val;
    killEvents(kmi);
    killEvents(kpl);
  }
  if(i_min==0 && i_max==1 && (event==EVT_KEY_FIRST(KEY_MENU) || event==EVT_KEY_BREAK(BTN_RE)) )
  {
      s_editMode = false;
      newval=!val;
      killEvents(event);
			skipPause = 1 ;
			if ( event==EVT_KEY_BREAK(BTN_RE) )
			{
				RotaryState = ROTARY_MENU_UD ;
			}
  }

  if (s_editMode>0 && (i_flags & INCDEC_SWITCH))
	{
    uint8_t swtch = getMovedSwitch();
    if (swtch)
		{
      newval = (val == swtch ? -swtch : swtch);
    }
  }

  //change values based on P1
  newval -= p1valdiff ;
	if ( RotaryState == ROTARY_VALUE )
	{
		newval += Rotary_diff ;
	}
  if(newval>i_max)
  {
    newval = i_max;
    killEvents(event);
    audioDefevent(AU_KEYPAD_UP);
  }
  else if(newval < i_min)
  {
    newval = i_min;
    killEvents(event);
    audioDefevent(AU_KEYPAD_DOWN);

  }
  if(newval != val) {
    if(newval==0) {
			if ( !skipPause )
			{
     	  pauseEvents(event);
			}
  
		if (newval>val){
			audioDefevent(AU_KEYPAD_UP);
		} else {
			audioDefevent(AU_KEYPAD_DOWN);
		}		

    }
    eeDirty(i_flags & (EE_GENERAL|EE_MODEL));
    checkIncDec_Ret = true;
  }
  else {
    checkIncDec_Ret = false;
  }
  return newval;
}

int8_t checkIncDec(uint8_t event, int8_t i_val, int8_t i_min, int8_t i_max, uint8_t i_flags)
{
  return checkIncDec16(event,i_val,i_min,i_max,i_flags);
}

int8_t checkIncDec_hm(uint8_t event, int8_t i_val, int8_t i_min, int8_t i_max)
{
  return checkIncDec(event,i_val,i_min,i_max,EE_MODEL);
}

int8_t checkIncDec_hg(uint8_t event, int8_t i_val, int8_t i_min, int8_t i_max)
{
  return checkIncDec(event,i_val,i_min,i_max,EE_GENERAL);
}

int8_t *TrimPtr[4] = 
{
  &g_model.trim[0],
  &g_model.trim[1],
  &g_model.trim[2],
  &g_model.trim[3]
} ;

uint8_t StickScrollAllowed ;

void perMain( uint32_t no_menu )
{
  static uint16_t lastTMR;
  tick10ms = (get_tmr10ms() != lastTMR);
  lastTMR = get_tmr10ms();

  perOut(g_chans512, 0);
  if(!tick10ms) return ; //make sure the rest happen only every 10ms.

	heartbeat |= HEART_TIMER10ms;
  uint8_t evt=getEvent();
  evt = checkTrim(evt);

  uint16_t a = 0;
  uint16_t b = 0;
  if(g_LightOffCounter) g_LightOffCounter--;
  if(evt) a = g_eeGeneral.lightAutoOff*500; // on keypress turn the light on 5*100
  if(stickMoved) b = g_eeGeneral.lightOnStickMove*500;
  if(a>g_LightOffCounter) g_LightOffCounter = a;
   if(b>g_LightOffCounter) g_LightOffCounter = b;

	check_backlight() ;

  static int16_t p1valprev;
  p1valdiff = (p1val-calibratedStick[6])/32;
  if(p1valdiff) {
    p1valdiff = (p1valprev-calibratedStick[6])/2;
    p1val = calibratedStick[6];
  }
  p1valprev = calibratedStick[6];
  if ( g_eeGeneral.disablePotScroll )
  {
    p1valdiff = 0 ;			
	}

	if ( g_eeGeneral.rotaryDivisor == 1)
	{
		Rotary_diff = ( Rotary_count - LastRotaryValue ) / 4 ;
		LastRotaryValue += Rotary_diff * 4 ;
	}
	else if ( g_eeGeneral.rotaryDivisor == 2)
	{
		Rotary_diff = ( Rotary_count - LastRotaryValue ) / 2 ;
		LastRotaryValue += Rotary_diff * 2 ;
	}
	else
	{
		Rotary_diff = Rotary_count - LastRotaryValue ;
		LastRotaryValue = Rotary_count ;
	}

	if ( g_menuStack[g_menuStackPtr] == menuProc0)
	{
		if ( Rotary_diff )
		{
			int16_t x = RotaryControl ;
			x += Rotary_diff ;
			if ( x > 125 )
			{
				RotaryControl = 125 ;
			}
			else if ( x < -125 )
			{
				RotaryControl = -125 ;
			}
			else
			{
				RotaryControl = x ;					
			}
			Rotary_diff = 0 ;
		}
	}
	
	if ( g_eeGeneral.stickScroll && StickScrollAllowed )
	{
		const static uint8_t rate[8] = { 0, 75, 40, 25, 10, 5, 2, 1 } ;
		static uint8_t repeater ;
		uint8_t direction ;
		int8_t value ;
		
		if ( repeater < 128 )
		{
			repeater += 1 ;
		}
		value = calibratedStick[2] / 128 ;
		direction = value > 0 ? 1 : 0 ;
		if ( value < 0 )
		{
			value = -value ;			// (abs)
		}
		if ( value > 7 )
		{
			value = 7 ;			
		}
		value = rate[value] ;
		if ( value )
		{
			if ( repeater > value )
			{
				repeater = 0 ;
				if ( direction )
				{
					putEvent(EVT_KEY_FIRST(KEY_UP));
				}
				else
				{
					putEvent(EVT_KEY_FIRST(KEY_DOWN));
				}
			}
		}
		else
		{
			value = calibratedStick[3] / 128 ;
			direction = value > 0 ? 1 : 0 ;
			if ( value < 0 )
			{
				value = -value ;			// (abs)
			}
			if ( value > 7 )
			{
				value = 7 ;			
			}
			value = rate[value] ;
			if ( value )
			{
				if ( repeater > value )
				{
					repeater = 0 ;
					if ( direction )
					{
						putEvent(EVT_KEY_FIRST(KEY_RIGHT));
					}
					else
					{
						putEvent(EVT_KEY_FIRST(KEY_LEFT));
					}
				}
			}
		}
	}
	StickScrollAllowed = 1 ;

#if GVARS
	for( uint8_t i = 0 ; i < MAX_GVARS ; i += 1 )
	{
		// ToDo, test for trim inputs here
		if ( g_model.gvars[i].gvsource )
		{
			if ( g_model.gvars[i].gvsource <= 4 )
			{
				g_model.gvars[i].gvar = *TrimPtr[ convert_mode_helper(g_model.gvars[i].gvsource) - 1 ] ;
			}
			else if ( g_model.gvars[i].gvsource == 5 )	// REN
			{
				g_model.gvars[i].gvar = RotaryControl ;
			}
			else if ( g_model.gvars[i].gvsource <= 9 )	// Stick
			{
				g_model.gvars[i].gvar = limit( -125, calibratedStick[ convert_mode_helper(g_model.gvars[i].gvsource-5) - 1 ] / 8, 125 ) ;
			}
			else if ( g_model.gvars[i].gvsource <= 12 )	// Pot
			{
				g_model.gvars[i].gvar = limit( -125, calibratedStick[ (g_model.gvars[i].gvsource-6)] / 8, 125 ) ;
			}
			else if ( g_model.gvars[i].gvsource <= 36 )	// Chans
			{
				g_model.gvars[i].gvar = limit( -125, ex_chans[g_model.gvars[i].gvsource-13] / 10, 125 ) ;
			}
		}
	}
#endif

#ifdef FRSKY
	check_frsky() ;
#endif

// Here, if waiting for EEPROM response, don't action menus

	if ( no_menu == 0 )
	{
    lcd_clear();
    g_menuStack[g_menuStackPtr](evt);
    refreshDisplay();
	}


	if ( check_soft_power() == POWER_TRAINER )		// On trainer power
	{
		PIOC->PIO_PDR = PIO_PC22 ;						// Disable bit C22 Assign to peripheral
	}
	else
	{
		PIOC->PIO_PER = PIO_PC22 ;						// Enable bit C22 as input
	}

  switch( get_tmr10ms() & 0x1f )
	{ //alle 10ms*32

    case 2:
//        //check v-bat
////        Calculation By Mike Blandford
////        Resistor divide on battery voltage is 5K1 and 2K7 giving a fraction of 2.7/7.8
////        If battery voltage = 10V then A2D voltage = 3.462V
////        11 bit A2D count is 1417 (3.462/5*2048).
////        1417*18/256 = 99 (actually 99.6) to represent 9.9 volts.
////        Erring on the side of low is probably best.

        int32_t ab = anaIn(7);
        ab = ( ab + ab*(g_eeGeneral.vBatCalib)/128 ) * 4191 ;
//        ab = (uint16_t) ab / (g_eeGeneral.disableBG ? 240 : BandGap ) ;  // ab might be more than 32767
        ab /= 55296  ;
        g_vbat100mV = ( (ab + g_vbat100mV + 1) >> 1 ) + 3 ;  // Filter it a bit => more stable display
								// Also add on 0.3V for voltage drop across input diode

        static uint8_t s_batCheck;
        s_batCheck+=32;
        if(s_batCheck==0)
				{
					if( (g_vbat100mV<g_eeGeneral.vBatWarn) && (g_vbat100mV>49) )
					{
            audioVoiceDefevent(AU_TX_BATTERY_LOW, V_BATTERY_LOW);
            if (g_eeGeneral.flashBeep) g_LightOffCounter = FLASH_DURATION;
					}
					else if ( ( g_eeGeneral.mAh_alarm ) && ( ( MAh_used + Current_used*(488 + g_eeGeneral.current_calib)/8192/36 ) / 500 >= g_eeGeneral.mAh_alarm ) )
					{
            audioVoiceDefevent(AU_TX_BATTERY_LOW, V_BATTERY_LOW);
					}
        }
    break ;

  }
  stickMoved = 0; //reset this flag
		
	AUDIO_HEARTBEAT();  // the queue processing

}

// Starts TIMER2 at 200Hz
//static void start_timer2()
//{
//  register Tc *ptc ;
//	register uint32_t timer ;

//  PMC->PMC_PCER0 |= 0x02000000L ;		// Enable peripheral clock to TC2

//	timer = Master_frequency / 12800 / 2 ;		// MCK/128 and 200 Hz

//  ptc = TC0 ;		// Tc block 0 (TC0-2)
//	ptc->TC_BCR = 0 ;			// No sync
//	ptc->TC_BMR = 0 ;
//	ptc->TC_CHANNEL[2].TC_CMR = 0x00008000 ;	// Waveform mode
//	ptc->TC_CHANNEL[2].TC_RC = timer ;			// 10 Hz
//	ptc->TC_CHANNEL[2].TC_RA = timer >> 1 ;
//	ptc->TC_CHANNEL[2].TC_CMR = 0x0009C003 ;	// 0000 0000 0000 1001 1100 0000 0000 0011
//																						// MCK/128, set @ RA, Clear @ RC waveform
//	ptc->TC_CHANNEL[2].TC_CCR = 5 ;		// Enable clock and trigger it (may only need trigger)
	
//	NVIC_EnableIRQ(TC2_IRQn) ;
//	TC0->TC_CHANNEL[2].TC_IER = TC_IER0_CPCS ;
//}



#if !defined(SIMU)
static void init_rotary_encoder()
{
  register uint32_t dummy;

	configure_pins( PIO_PC19 | PIO_PC21, PIN_ENABLE | PIN_INPUT | PIN_PORTC | PIN_PULLUP ) ;	// 19 and 21 are rotary encoder
	configure_pins( PIO_PB6, PIN_ENABLE | PIN_INPUT | PIN_PORTB | PIN_PULLUP ) ;		// rotary encoder switch
	PIOC->PIO_IER = PIO_PC19 | PIO_PC21 ;
	dummy = PIOC->PIO_PDSR ;		// Read Rotary encoder (PC19, PC21)
	dummy >>= 19 ;
	dummy &= 0x05 ;			// pick out the three bits
	Rotary_position &= ~0x45 ;
	Rotary_position |= dummy ;
	NVIC_EnableIRQ(PIOC_IRQn) ;
	LastRotaryValue = Rotary_count ;
}
#endif

static void stop_rotary_encoder()
{
	NVIC_DisableIRQ(PIOC_IRQn) ;
	PIOC->PIO_IDR = PIO_PC19 | PIO_PC21 ;
}

extern "C" void PIOC_IRQHandler()
{
  register uint32_t dummy;
	
	dummy = PIOC->PIO_ISR ;			// Read and clear status register
	(void) dummy ;		// Discard value - prevents compiler warning

	dummy = PIOC->PIO_PDSR ;		// Read Rotary encoder (PC19, PC21)
	dummy >>= 19 ;
	dummy &= 0x05 ;			// pick out the three bits
	if ( dummy != ( Rotary_position & 0x05 ) )
	{
		if ( ( Rotary_position & 0x01 ) ^ ( ( dummy & 0x04) >> 2 ) )
		{
			Rotary_count -= 1 ;
		}
		else
		{
			Rotary_count += 1 ;
		}
		Rotary_position &= ~0x45 ;
		Rotary_position |= dummy ;
	}
	// Now for testing only
//	dummy = PIOB->PIO_PDSR & 0x40 ;		// Read the switch
//	if ( ( Rotary_position & 0x40 ) != dummy )
//	{
//		Rotary_position ^= 0x40 ;
//	}
}


void interrupt5ms()
{
	static uint32_t pre_scale ;		// Used to get 10 Hz counter

	sound_5ms() ;
	 
	if ( ++pre_scale >= 2 )
	{
		Tenms |= 1 ;			// 10 mS has passed
		if ( Buzzer_count )
		{
			if ( --Buzzer_count == 0 )
			{
				buzzer_off() ;			
			}
		}
//		Timer2_count += 1 ;
		pre_scale = 0 ;
  	per10ms();
		if (--AlarmTimer == 0 )
		{
			AlarmTimer = 100 ;		// Restart timer
			AlarmCheckFlag += 1 ;	// Flag time to check alarms
			CsCheckFlag = 1 ;
		}
		if (--VoiceTimer == 0 )
		{
			VoiceTimer = 10 ;		// Restart timer
			VoiceCheckFlag = 1 ;	// Flag time to check alarms
		}

	}
// Remove the following when properly implemented, this for testing
//	dummy = PIOB->PIO_PDSR & 0x40 ;		// Read the switch
//	if ( ( Rotary_position & 0x40 ) != dummy )
//	{
//		Rotary_position ^= 0x40 ;
//	}
}




// ADC channels are assigned to:
// AD1  stick_RH
// AD2  stick_LH
// AD3  PI#T_TRIM
// AD4  battery
// AD5  HOV_PIT
// AD9  stick_LV
// AD13 HOV_THR
// AD14 stick_RV
// AD15 Chip temperature
// Peripheral ID 29 (0x20000000)
// Note ADC sequencing won't work as it only operates on channels 0-7
//      and we need 9, 13 and 14 as well
// ALSO: Errata says only one channel converted for each trigger

// Needed implementation (11 bit result)
// SINGLE - 1 read then >> 1
// OVERSAMPLE - 4 reads - average = sum then >> 3
// FILTERED - 1 read but filter processing

// Filtering algorithm
// o/p = half previous o/p + temp1
// temp1 = average temp1 and temp0
// temp0 = average new reading and temp0

#ifndef SIMU
uint16_t anaIn(uint8_t chan)
{
  static uint8_t crossAna[]={1,5,7,0,4,6,2,3,8};
  volatile uint16_t *p = &S_anaFilt[crossAna[chan]] ;
	if ( chan == 8 )
	{
		return Current_analogue ;
	}
  return  *p;
}
#endif

void getADC_single()
{
	register uint32_t x ;

	read_9_adc() ;

	for( x = 0 ; x < NUMBER_ANALOG ; x += 1 )
	{
		S_anaFilt[x] = Analog_values[x] >> 1 ;
	}
}


void getADC_osmp()
{
	register uint32_t x ;
	register uint32_t y ;
	uint16_t temp[NUMBER_ANALOG] ;

	for( x = 0 ; x < NUMBER_ANALOG ; x += 1 )
	{
		temp[x] = 0 ;
	}
	for( y = 0 ; y < 4 ; y += 1 )
	{
		read_9_adc() ;
		for( x = 0 ; x < NUMBER_ANALOG ; x += 1 )
		{
			temp[x] += Analog_values[x] ;
		}
	}
	for( x = 0 ; x < NUMBER_ANALOG ; x += 1 )
	{
		S_anaFilt[x] = temp[x] >> 3 ;
	}
}


void getADC_filt()
{
	register uint32_t x ;
	static uint16_t t_ana[2][NUMBER_ANALOG] ;

	read_9_adc() ;
	for( x = 0 ; x < NUMBER_ANALOG ; x += 1 )
	{
		S_anaFilt[x] = S_anaFilt[x]/2 + (t_ana[1][x] >> 2 ) ;
		t_ana[1][x] = ( t_ana[1][x] + t_ana[0][x] ) >> 1 ;
		t_ana[0][x] = ( t_ana[0][x] + Analog_values[x] ) >> 1 ;
	}	 
}


uint32_t getFlightPhase()
{
	uint32_t i ;
  for ( i = 0 ; i < MAX_PHASES ; i += 1 )
	{
    PhaseData *phase = &g_model.phaseData[i];
    if ( phase->swtch && getSwitch( phase->swtch, 0 ) )
		{
      return i + 1 ;
    }
  }
  return 0 ;
}

int16_t getRawTrimValue( uint8_t phase, uint8_t idx )
{
	if ( phase )
	{
		return g_model.phaseData[phase-1].trim[idx] ;
	}	
	else
	{
		return *TrimPtr[idx] ;
	}
}

uint32_t getTrimFlightPhase( uint8_t phase, uint8_t idx )
{
  for ( uint32_t i=0 ; i<MAX_PHASES ; i += 1 )
	{
    if (phase == 0) return 0;
    int16_t trim = getRawTrimValue( phase, idx ) ;
    if ( trim <= TRIM_EXTENDED_MAX )
		{
			return phase ;
		}
    uint32_t result = trim-TRIM_EXTENDED_MAX-1 ;
    if (result >= phase)
		{
			result += 1 ;
		}
    phase = result;
  }
  return 0;
}


int16_t getTrimValue( uint8_t phase, uint8_t idx )
{
  return getRawTrimValue( getTrimFlightPhase( phase, idx ), idx ) ;
}


void setTrimValue(uint8_t phase, uint8_t idx, int16_t trim)
{
	if ( phase )
	{
		phase = getTrimFlightPhase( phase, idx ) ;
	}
	if ( phase )
	{
  	g_model.phaseData[phase-1].trim[idx] = trim ;
	}
	else
	{
    if(trim < -125 || trim > 125)
		{
			trim = ( trim > 0 ) ? 125 : -125 ;
		}	
   	*TrimPtr[idx] = trim ;
	}
  STORE_MODELVARS_TRIM ;
}


static uint8_t checkTrim(uint8_t event)
{
  int8_t  k = (event & EVT_KEY_MASK) - TRM_BASE;
  int8_t  s = g_model.trimInc;
  
	if (s>1) s = 1 << (s-1);  // 1=>1  2=>2  3=>4  4=>8

  if((k>=0) && (k<8))// && (event & _MSK_KEY_REPT))
  {
    //LH_DWN LH_UP LV_DWN LV_UP RV_DWN RV_UP RH_DWN RH_UP
    uint8_t idx = k/2;
		if ( g_eeGeneral.crosstrim )
		{
			idx = 3 - idx ;			
		}
		uint32_t phaseNo = getTrimFlightPhase( CurrentPhase, idx ) ;
    int16_t tm = getTrimValue( phaseNo, idx ) ;
    int8_t  v = (s==0) ? (abs(tm)/4)+1 : s;
    bool thrChan = ((2-(g_eeGeneral.stickMode&1)) == idx);
    bool thro = (thrChan && (g_model.thrTrim));
    if(thro) v = 4; // if throttle trim and trim trottle then step=4
    if(thrChan && g_eeGeneral.throttleReversed) v = -v;  // throttle reversed = trim reversed
    int16_t x = (k&1) ? tm + v : tm - v;   // positive = k&1

    if(((x==0)  ||  ((x>=0) != (tm>=0))) && (!thro) && (tm!=0))
		{
			setTrimValue( phaseNo, idx, 0 ) ;
      killEvents(event);
      audioDefevent(AU_TRIM_MIDDLE);
    }
		else if(x>-125 && x<125)
		{
			setTrimValue( phaseNo, idx, x ) ;
//      STORE_MODELVARS_TRIM;
      //if(event & _MSK_KEY_REPT) warble = true;
			if(x <= 125 && x >= -125)
			{
				if(g_eeGeneral.speakerMode == 0){
					audioDefevent(AU_TRIM_MOVE);
				}
				else
				{
					audio.event(AU_TRIM_MOVE,(abs(x)/4)+60);
				}
			}	
    }
    else
    {
			setTrimValue( phaseNo, idx, (x>0) ? 125 : -125 ) ;
//      STORE_MODELVARS_TRIM;
			if(x <= 125 && x >= -125){
				if(g_eeGeneral.speakerMode == 0){
					audioDefevent(AU_TRIM_MOVE);
				} else {
					audio.event(AU_TRIM_MOVE,(-abs(x)/4)+60);
				}
			}	
    }
    return 0;
  }
  return event;
}


void putsChnRaw(uint8_t x,uint8_t y,uint8_t idx,uint8_t att)
{
	uint8_t chanLimit = NUM_SKYXCHNRAW ;
	if ( att & MIX_SOURCE )
	{
#if GVARS
		chanLimit += MAX_GVARS + 1 ;
#else
		chanLimit += 1 ;
#endif
		att &= ~MIX_SOURCE ;		
	}
  if(idx==0)
    lcd_putsnAtt(x,y,PSTR("----"),4,att);
  else if(idx<=4)
    lcd_putsnAtt(x,y,modi12x3+g_eeGeneral.stickMode*16+4*(idx-1),4,att);
//    lcd_putsnAtt(x,y,modi12x3[(modn12x3[g_eeGeneral.stickMode*4]+(idx-1))-1)*4],4,att);
  else if(idx<=chanLimit)
#if GVARS
    lcd_putsnAtt(x,y,PSTR("P1  P2  P3  HALFFULLCYC1CYC2CYC3PPM1PPM2PPM3PPM4PPM5PPM6PPM7PPM8CH1 CH2 CH3 CH4 CH5 CH6 CH7 CH8 CH9 CH10CH11CH12CH13CH14CH15CH16CH17CH18CH19CH20CH21CH22CH23CH243POSGV1 GV2 GV3 GV4 GV5 GV6 GV7 ")+4*(idx-5),4,att);
#else
    lcd_putsnAtt(x,y,PSTR("P1  P2  P3  HALFFULLCYC1CYC2CYC3PPM1PPM2PPM3PPM4PPM5PPM6PPM7PPM8CH1 CH2 CH3 CH4 CH5 CH6 CH7 CH8 CH9 CH10CH11CH12CH13CH14CH15CH16CH17CH18CH19CH20CH21CH22CH23CH243POS")+4*(idx-5),4,att);
#endif
	else
  	lcd_putsAttIdx(x,y,Str_telemItems,(idx-NUM_SKYXCHNRAW-1),att);
}

void putsChn(uint8_t x,uint8_t y,uint8_t idx1,uint8_t att)
{
  // !! todo NUM_CHN !!
  lcd_putsnAtt(x,y,PSTR("--- CH1 CH2 CH3 CH4 CH5 CH6 CH7 CH8 CH9 CH10CH11CH12CH13CH14CH15CH16"
                        "CH17CH18CH19CH20CH21CH22CH23CH24CH25CH26CH27CH28CH29CH30")+4*idx1,4,att);
}

void putsDrSwitches(uint8_t x,uint8_t y,int8_t idx1,uint8_t att)//, bool nc)
{
	const char *pstr ;
  switch(idx1){
    case  0:            lcd_putsAtt(x+FW,y,PSTR("---"),att);return;
    case  MAX_SKYDRSWITCH: lcd_putsAtt(x+FW,y,PSTR("ON "),att);return;
    case -MAX_SKYDRSWITCH: lcd_putsAtt(x+FW,y,PSTR("OFF"),att);return;
  }
	if ( idx1 < 0 )
	{
  	lcd_putcAtt(x,y, '!',att);
	}
	pstr = get_switches_string()+3*(abs(idx1)-1) ;
  lcd_putsnAtt(x+FW,y,pstr,3,att);
	if ( att & CONDENSED )
	{
		pstr += 2 ;
		att &= ~CONDENSED ;
  	lcd_putcAtt(x+3*FW-1, y,*pstr,att);
	}
}

//Type 1-trigA, 2-trigB, 0 best for display
void putsTmrMode(uint8_t x, uint8_t y, uint8_t attr, uint8_t timer, uint8_t type )
{
  int8_t tm = g_model.timer[timer].tmrModeA ;
	if ( type < 2 )		// 0 or 1
	{
	  if(tm<TMR_VAROFS) {
        lcd_putsnAtt(  x, y, PSTR("OFFABSTHsTH%")+3*abs(tm),3,attr);
//    return;
  	}
		else
		{
  		tm -= TMR_VAROFS ;
  		lcd_putsnAtt(  x, y, PSTR( CURV_STR )+1 + 21 + 3*tm, 3, attr ) ;		// Cheat to get chan# text
			if ( tm < 9 )
			{
				x -= FW ;		
			}
  		lcd_putcAtt(x+3*FW,  y,'%',attr);
		}
	}
	if ( ( type == 2 ) || ( ( type == 0 ) && ( tm == 1 ) ) )
	{
    tm = g_model.timer[timer].tmrModeB;
    if(abs(tm)>=(MAX_SKYDRSWITCH))	 //momentary on-off
		{
  	  lcd_putcAtt(x+3*FW,  y,'m',attr);
			if ( tm > 0 )
			{
				tm -= MAX_SKYDRSWITCH - 1 ;
			}
		}			 
   	putsDrSwitches( x-1*FW, y, tm, attr );
	}
}

const char *get_switches_string()
{
  return PSTR(SWITCHES_STR)	;
}	




inline int16_t getValue(uint8_t i)
{
  if(i<PPM_BASE) return calibratedStick[i];//-512..512
  else if(i<PPM_BASE+4) return (g_ppmIns[i-PPM_BASE] - g_eeGeneral.trainer.calib[i-PPM_BASE])*2;
  else if(i<CHOUT_BASE) return g_ppmIns[i-PPM_BASE]*2;
  else if(i<CHOUT_BASE+NUM_SKYCHNOUT) return ex_chans[i-CHOUT_BASE];
  else if(i<CHOUT_BASE+NUM_SKYCHNOUT+NUM_TELEM_ITEMS)
	{
		return get_telemetry_value( i-CHOUT_BASE-NUM_SKYCHNOUT ) ;
	}
  return 0 ;
}



bool Last_switch[NUM_SKYCSW] ;

bool getSwitch(int8_t swtch, bool nc, uint8_t level)
{
  bool ret_value ;
  uint8_t cs_index ;
  
	if(level>5) return false ; //prevent recursive loop going too deep

  switch(swtch){
    case  0:            return  nc;
    case  MAX_SKYDRSWITCH: return  true ;
    case -MAX_SKYDRSWITCH: return  false ;
  }

	if ( swtch > MAX_SKYDRSWITCH )
	{
		return false ;
	}

  uint8_t dir = swtch>0;
  if(abs(swtch)<(MAX_SKYDRSWITCH-NUM_SKYCSW)) {
    if(!dir) return ! keyState((enum EnumKeys)(SW_BASE-swtch-1));
    return            keyState((enum EnumKeys)(SW_BASE+swtch-1));
  }

  //use putsChnRaw
  //input -> 1..4 -> sticks,  5..8 pots
  //MAX,FULL - disregard
  //ppm
  cs_index = abs(swtch)-(MAX_SKYDRSWITCH-NUM_SKYCSW);
  SKYCSwData &cs = g_model.customSw[cs_index];
  if(!cs.func) return false;

  if ( level>4 )
  {
    ret_value = Last_switch[cs_index] ;
    return swtch>0 ? ret_value : !ret_value ;
  }

  int8_t a = cs.v1;
  int8_t b = cs.v2;
  int16_t x = 0;
  int16_t y = 0;
	uint8_t valid = 1 ;

  // init values only if needed
  uint8_t s = CS_STATE(cs.func);

  if(s == CS_VOFS)
  {
      x = getValue(cs.v1-1);
#ifdef FRSKY
      if (cs.v1 > CHOUT_BASE+NUM_SKYCHNOUT)
			{
        y = convertTelemConstant( cs.v1-CHOUT_BASE-NUM_SKYCHNOUT-1, cs.v2 ) ;
				valid = telemItemValid( cs.v1-CHOUT_BASE-NUM_SKYCHNOUT-1 ) ;
			}
      else
#endif
      y = calc100toRESX(cs.v2);
  }
  else if(s == CS_VCOMP)
  {
      x = getValue(cs.v1-1);
      y = getValue(cs.v2-1);
  }

  switch (cs.func) {
  case (CS_VPOS):
      ret_value = (x>y);
      break;
  case (CS_VNEG):
      ret_value = (x<y) ;
      break;
  case (CS_APOS):
  {
      ret_value = (abs(x)>y) ;
  }
      break;
  case (CS_ANEG):
  {
      ret_value = (abs(x)<y) ;
  }
      break;

  case (CS_AND):
  case (CS_OR):
  case (CS_XOR):
  {
    bool res1 = getSwitch(a,0,level+1) ;
    bool res2 = getSwitch(b,0,level+1) ;
    if ( cs.func == CS_AND )
    {
      ret_value = res1 && res2 ;
    }
    else if ( cs.func == CS_OR )
    {
      ret_value = res1 || res2 ;
    }
    else  // CS_XOR
    {
      ret_value = res1 ^ res2 ;
    }
  }
  break;

  case (CS_EQUAL):
      ret_value = (x==y);
      break;
  case (CS_NEQUAL):
      ret_value = (x!=y);
      break;
  case (CS_GREATER):
      ret_value = (x>y);
      break;
  case (CS_LESS):
      ret_value = (x<y);
      break;
  case (CS_EGREATER):
      ret_value = (x>=y);
      break;
  case (CS_ELESS):
      ret_value = (x<=y);
      break;
  case (CS_TIME):
      ret_value = CsTimer[cs_index] >= 0 ;
      break;
  default:
      ret_value = false;
      break;
  }
	if ( valid == 0 )			// Catch telemetry values not present
	{
     ret_value = false;
	}
	if ( ret_value )
	{
		if ( cs.andsw )
		{
			int8_t x ;
			x = cs.andsw ;
			if ( x > 8 )
			{
				x += 1 ;
			}
      ret_value = getSwitch( x, 0, level+1) ;
		}
	}
	Last_switch[cs_index] = ret_value ;
	return swtch>0 ? ret_value : !ret_value ;

}


int8_t getMovedSwitch()
{
  static uint8_t switches_states = 0;
  static uint16_t s_last_time = 0;

  int8_t result = 0;

  for (uint8_t i=MAX_PSWITCH; i>0; i--) {
    bool prev;
    uint8_t mask = 0;
    if (i <= 3) {
      mask = (1<<(i-1));
      prev = (switches_states & mask);
    }
    else if (i <= 6) {
      prev = ((switches_states & 0x18) == ((i-3) << 3));
    }
    else {
      mask = (1<<(i-2));
      prev = (switches_states & mask);
    }
    bool next = getSwitch(i, 0, 0) ;
    if (prev != next) {
      if (i!=MAX_PSWITCH || next==true)
	      result = i;
      if (mask)
        switches_states ^= mask;
      else
        switches_states = (switches_states & 0xE7) | ((i-3) << 3);
    }
  }

  if (get_tmr10ms() - s_last_time > 10)
    result = 0;

  s_last_time = get_tmr10ms();

  return result;
}

void checkQuickSelect()
{
  uint8_t i = keyDown(); //check for keystate
  uint8_t j;
  for(j=1; j<8; j++)
      if(i & (1<<j)) break;
  j--;

  if(j<6)
	{
    if(!ee32ModelExists(j))
			return ;
    if( g_eeGeneral.currModel != j )
		{
	    ee32LoadModel(g_eeGeneral.currModel = j);
	    STORE_GENERALVARS;
		}
    lcd_clear();
    lcd_putsAtt(64-7*FW,0*FH,PSTR("LOADING"),DBLSIZE);

    for(uint8_t i=0;i<sizeof(g_model.name);i++)
      lcd_putcAtt(FW*2+i*2*FW-i-2, 3*FH, g_model.name[i],DBLSIZE);

    refreshDisplay();
    clearKeyEvents(); // wait for user to release key
  }
}

void alertMessages( const char * s, const char * t )
{
  lcd_clear();
  lcd_putsAtt(64-5*FW,0*FH,PSTR("ALERT"),DBLSIZE);
  lcd_puts_P(0,4*FH,s);
  lcd_puts_P(0,5*FH,t);
  lcd_puts_P(0,6*FH,  PSTR("Press any key to skip") ) ;
  refreshDisplay();
//  lcdSetRefVolt(g_eeGeneral.contrast);

  clearKeyEvents();
}


void alert(const char * s, bool defaults)
{
  lcd_clear();
  lcd_putsAtt(64-5*FW,0*FH,PSTR("ALERT"),DBLSIZE);
  lcd_puts_P(0,4*FH,s);
  lcd_puts_P(64-6*FW,7*FH,PSTR("press any Key"));
  refreshDisplay();
  lcdSetRefVolt(defaults ? 0x22 : g_eeGeneral.contrast);

  audioVoiceDefevent(AU_ERROR, V_ERROR);
  clearKeyEvents();
  while(1)
  {
#ifdef SIMU
    if (!main_thread_running) return;
    sleep(1/*ms*/);
#endif

    if(keyDown())
    {
      return;  //wait for key release
    }
    wdt_reset();
		if ( check_power_or_usb() ) return ;		// Usb on or power off
    if(getSwitch(g_eeGeneral.lightSw,0) || g_eeGeneral.lightAutoOff || defaults)
      BACKLIGHT_ON;
    else
      BACKLIGHT_OFF;
  }
}

void message(const char * s)
{
  lcd_clear();
  lcd_putsAtt(64-5*FW,0*FH,PSTR("MESSAGE"),DBLSIZE);
  lcd_puts_P(0,4*FW,s);
  refreshDisplay();
//  lcdSetRefVolt(g_eeGeneral.contrast);
}

int16_t tanaIn( uint8_t chan )
{
 	int16_t v = anaIn(chan) ;
	return  (g_eeGeneral.throttleReversed) ? -v : v ;
}

void checkTHR()
{
  if(g_eeGeneral.disableThrottleWarning) return;

  uint8_t thrchn=(2-(g_eeGeneral.stickMode&1));//stickMode=0123 -> thr=2121

#ifdef SIMU
  int16_t lowLim = THRCHK_DEADBAND - 1024 ;
#else
  getADC_single();   // if thr is down - do not display warning at all
	int16_t lowLim = g_eeGeneral.calibMid[thrchn] ;

	lowLim = (g_eeGeneral.throttleReversed ? (- lowLim) - g_eeGeneral.calibSpanPos[thrchn] : lowLim - g_eeGeneral.calibSpanNeg[thrchn]);
	lowLim += THRCHK_DEADBAND ;
#endif

  int16_t v = tanaIn(thrchn);
  if(v<=lowLim)
  {
    return;
  }

  // first - display warning
  alertMessages( PSTR("Throttle not idle"), PSTR("Reset throttle") ) ;
  
	//loop until throttle stick is low
  while (1)
  {
#ifdef SIMU
      if (!main_thread_running) return;
      sleep(1/*ms*/);
#else
      getADC_single();
#endif
			check_backlight() ;

      int16_t v = tanaIn(thrchn);
      if((v<=lowLim) || (keyDown()))
      {
        return;
      }
      wdt_reset();

		if ( check_power_or_usb() ) return ;		// Usb on or power off

  }
}



void putWarnSwitch( uint8_t x, const char * s )
{
  lcd_putsnAtt( x, 2*FH, s, 3, 0) ;
}

static void checkSwitches()
{
  if(g_eeGeneral.disableSwitchWarning) return; // if warning is on

  // first - display warning
  alertMessages( PSTR("Switches Warning"), PSTR("Please Reset Switches") ) ;
  uint8_t x = g_eeGeneral.switchWarningStates & SWP_IL5;
  if(x==SWP_IL1 || x==SWP_IL2 || x==SWP_IL3 || x==SWP_IL4 || x==SWP_IL5) //illegal states for ID0/1/2
  {
      g_eeGeneral.switchWarningStates &= ~SWP_IL5; // turn all off, make sure only one is on
      g_eeGeneral.switchWarningStates |=  SWP_ID0B;
  }
	
	//loop until all switches are reset
	
	while (1)
  {
    uint32_t i = 0;
		for(uint32_t j=0; j<8; j++)
    {
        bool t=keyState((EnumKeys)(SW_BASE_DIAG+7-j));
				i <<= 1 ;
        i |= (uint32_t)t;  // (!) casted to avoid a warning
    }
//        alertMessages( PSTR("Switches Warning"), PSTR("Please Reset Switches") ) ;

        //show the difference between i and switch?
        //show just the offending switches.
        //first row - THR, GEA, AIL, ELE, ID0/1/2
        uint32_t x = i ^ g_eeGeneral.switchWarningStates;

        lcd_putsnAtt(0*FW, 2*FH, PSTR("                      "), 22, 0);

        if(x & SWP_THRB)
            putWarnSwitch(2 + 0*FW, get_switches_string() );
        if(x & SWP_RUDB)
            putWarnSwitch(2 + 3*FW + FW/2, get_switches_string()+3 );
        if(x & SWP_ELEB)
            putWarnSwitch(2 + 7*FW, get_switches_string()+6 );

        if(x & SWP_IL5)
        {
            if(i & SWP_ID0B)
                putWarnSwitch(2 + 10*FW + FW/2, get_switches_string()+9 );
            if(i & SWP_ID1B)
                putWarnSwitch(2 + 10*FW + FW/2, get_switches_string()+12 );
            if(i & SWP_ID2B)
                putWarnSwitch(2 + 10*FW + FW/2, get_switches_string()+15 );
        }

        if(x & SWP_AILB)
            putWarnSwitch(2 + 14*FW, get_switches_string()+18 );
        if(x & SWP_GEAB)
            putWarnSwitch(2 + 17*FW + FW/2, get_switches_string()+21 );


        refreshDisplay();


    if( (i==g_eeGeneral.switchWarningStates) || (keyDown())) // check state against settings
    {
        return;  //wait for key release
    }
    wdt_reset();

		if ( check_power_or_usb() ) return ;		// Usb on or power off

		check_backlight() ;

#ifdef SIMU
    if (!main_thread_running) return;
    sleep(1/*ms*/);
#endif
  }
}


MenuFuncP lastPopMenu()
{
  return  g_menuStack[g_menuStackPtr+1];
}

void popMenu(bool uppermost)
{
  if(g_menuStackPtr>0 || uppermost){
    g_menuStackPtr = uppermost ? 0 : g_menuStackPtr-1;
    audioDefevent(AU_MENUS);
    (*g_menuStack[g_menuStackPtr])(EVT_ENTRY_UP);
  }else{
    alert(PSTR("menuStack underflow"));
  }
}

void chainMenu(MenuFuncP newMenu)
{
  g_menuStack[g_menuStackPtr] = newMenu;
  (*newMenu)(EVT_ENTRY);
  audioDefevent(AU_MENUS);
}
void pushMenu(MenuFuncP newMenu)
{

  g_menuStackPtr++;
  if(g_menuStackPtr >= DIM(g_menuStack))
  {
    g_menuStackPtr--;
    alert(PSTR("menuStack overflow"));
    return;
  }
  audioDefevent(AU_MENUS);
  g_menuStack[g_menuStackPtr] = newMenu;
  (*newMenu)(EVT_ENTRY);
}


// 
static void init_soft_power()
{
	// Configure RF_power (PC17)
	configure_pins( PIO_PC17, PIN_ENABLE | PIN_INPUT | PIN_PORTC | PIN_NO_PULLUP | PIN_PULLDOWN ) ;
	
	configure_pins( PIO_PA8, PIN_ENABLE | PIN_INPUT | PIN_PORTA | PIN_PULLUP ) ;
}


// Returns zero if power is switched off
//  1 if power switch is on
//  2 if power switch off, trainer power on
uint32_t check_soft_power()
{
#ifdef SIMU
  return POWER_ON;
#endif
#ifdef REVB	
	if ( PIOC->PIO_PDSR & PIO_PC17 )		// Power on
	{
		return POWER_ON ;
	}

	if ( PIOA->PIO_PDSR & PIO_PA8 )		// Trainer plugged in
	{
		return POWER_TRAINER ;
	}
#endif
	return POWER_OFF ;	
}


// turn off soft power
void soft_power_off()
{
#ifdef REVB
	
	configure_pins( PIO_PA8, PIN_ENABLE | PIN_OUTPUT | PIN_LOW | PIN_PORTA | PIN_NO_PULLUP ) ;
#endif
}

uint8_t *cpystr( uint8_t *dest, uint8_t *source )
{
  while ( (*dest++ = *source++) )
    ;
  return dest - 1 ;
}

#if GVARS
int8_t REG(int8_t x, int8_t min, int8_t max)
{
  int8_t result = x;
  if (x >= 126 || x <= -126) {
    x = (uint8_t)x - 126;
    result = g_model.gvars[x].gvar ;
    if (result < min) {
      g_model.gvars[x].gvar = result = min;
//      eeDirty( EE_MODEL | EE_TRIM ) ;
    }
    if (result > max) {
      g_model.gvars[x].gvar = result = max;
//      eeDirty( EE_MODEL | EE_TRIM ) ;
    }
  }
  return result;
}
#endif

/*** EOF ***/

