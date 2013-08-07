
#include <stdint.h>
#include <stdlib.h>
#include "ersky9x.h"
#include "timers.h"
#include "logicio.h"
#include "analog.h"
#include "drivers.h"
#include "myeeprom.h"
#include "sound.h"
#include "audio.h"
#include "diskio.h"

#include "x9d\stm32f2xx.h"
#include "x9d\stm32f2xx_gpio.h"
#include "x9d\stm32f2xx_rcc.h"
#include "x9d\hal.h"
#include "X9D/i2c_ee.h"

#include "lcd.h"

#ifndef SIMU
#include "CoOS.h"
#endif

#include "debug.h"

extern void initWatchdog( void ) ;

#define MAIN_STACK_SIZE		500
#define DEBUG_STACK_SIZE	130
#define VOICE_STACK_SIZE	130

OS_TID MainTask;
OS_STK main_stk[MAIN_STACK_SIZE] ;

#ifdef	DEBUG
OS_TID DebugTask;
OS_STK debug_stk[DEBUG_STACK_SIZE] ;
OS_TID VoiceTask;
OS_STK voice_stk[VOICE_STACK_SIZE] ;
#endif

//gtime_t g_rtcTime ;

void init_i2s1( void ) ;

//Temporary

const char Str_telemItems[] = "\004----A1= A2= RSSITSSITim1Tim2Alt GaltGspdT1= T2= RPM FUELMah1Mah2CvltBattAmpsMah CtotFasVAccXAccYAccZ" ; 
uint32_t sd_card_ready()
{
	return 0 ;
}
struct t_voice Voice ;

int16_t g_chans512[NUM_SKYCHNOUT];

// End temporary


EEGeneral  g_eeGeneral;
//ModelData  g_oldmodel;
SKYModelData  g_model;

int16_t g_ppmIns[8];
uint8_t ppmInState = 0; //0=unsync 1..8= wait for value i-1

const uint8_t splashdata[] = { 'S','P','S',0,
#include "s9xsplash.lbm"
	'S','P','E',0};

#include "sticks.lbm"


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
//audioQueue  audio;


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
uint8_t heartbeat ;
uint8_t heartbeat_running ;

uint16_t ResetReason ;



int main( void ) ;


void crlf( void ) ;
void p8hex( uint32_t value ) ;
void p4hex( uint16_t value ) ;
void p2hex( unsigned char c ) ;
void hex_digit_send( unsigned char c ) ;
void txmit( uint8_t c ) ;

void main_loop( void* pdata ) ;


uint32_t Master_frequency ;

volatile uint32_t Tenms ;						// Modified in interrupt routine
volatile uint8_t tick10ms = 0 ;



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
//			if ( PIOC->PIO_PDSR & 0x02000000 )
//			{
				// Detected USB
//				break ;
//			}
			if ( heartbeat_running )
			{
  			if(heartbeat == 0x3)
  			{
  			  heartbeat = 0;
//  			  wdt_reset();
  			}
			}
			else
			{
//				wdt_reset() ;
			}
		}	
    putEvent(0);
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

//  audioVoiceDefevent(AU_ERROR, V_ERROR);
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
//    wdt_reset();
//		if ( check_power_or_usb() ) return ;		// Usb on or power off
//    if(getSwitch(g_eeGeneral.lightSw,0) || g_eeGeneral.lightAutoOff || defaults)
//      BACKLIGHT_ON;
//    else
//      BACKLIGHT_OFF;
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

MenuFuncP lastPopMenu()
{
  return  g_menuStack[g_menuStackPtr+1];
}

void popMenu(bool uppermost)
{
  if(g_menuStackPtr>0 || uppermost){
    g_menuStackPtr = uppermost ? 0 : g_menuStackPtr-1;
//    audioDefevent(AU_MENUS);
    (*g_menuStack[g_menuStackPtr])(EVT_ENTRY_UP);
  }else{
    alert(PSTR("menuStack underflow"));
  }
}

void chainMenu(MenuFuncP newMenu)
{
  g_menuStack[g_menuStackPtr] = newMenu;
  (*newMenu)(EVT_ENTRY);
//  audioDefevent(AU_MENUS);
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
//  audioDefevent(AU_MENUS);
  g_menuStack[g_menuStackPtr] = newMenu;
  (*newMenu)(EVT_ENTRY);
}








void txmit( uint8_t c )
{
	/* Wait for the transmitter to be ready */
  while ( (USART3->SR & USART_SR_TXE) == 0 ) ;

  /* Send character */
	USART3->DR = c ;
}

uint16_t rxuart()
{
  if (USART3->SR & USART_SR_RXNE)
	{
		return USART3->DR ;
	}
	return 0xFFFF ;
}

void uputs( register char *string )
{
	while ( *string )
	{
		txmit( *string++ ) ;		
	}	
}


// Send a <cr><lf> combination to the serial port
void crlf()
{
	txmit( 13 ) ;
	txmit( 10 ) ;
}

// Send the 32 bit value to the RS232 port as 8 hex digits
void p8hex( uint32_t value )
{
	p4hex( value >> 16 ) ;
	p4hex( value ) ;
}

// Send the 16 bit value to the RS232 port as 4 hex digits
void p4hex( uint16_t value )
{
	p2hex( value >> 8 ) ;
	p2hex( value ) ;
}

// Send the 8 bit value to the RS232 port as 2 hex digits
void p2hex( unsigned char c )
{
//	asm("swap %c") ;
	hex_digit_send( c >> 4 ) ;
//	asm("swap %c") ;
	hex_digit_send( c ) ;
}

// Send a single 4 bit value to the RS232 port as a hex digit
void hex_digit_send( unsigned char c )
{
	c &= 0x0F ;
	if ( c > 9 )
	{
		c += 7 ;
	}
	c += '0' ;
	txmit( c ) ;
}

extern void disp_256( register uint32_t address, register uint32_t lines ) ;
extern void dispw_256( register uint32_t address, register uint32_t lines ) ;

//void disp_256( register uint32_t address, register uint32_t lines )
//{
//	register uint32_t i ;
//	register uint32_t j ;
//	for ( i = 0 ; i < lines ; i += 1 )
//	{
//		p8hex( address ) ;
//		for ( j = 0 ; j < 16 ; j += 1 )
//		{
//			txmit(' ') ;
//			p2hex( *( (uint8_t *)address++ ) ) ;
//		}
//		crlf() ;
//	}
//}



//volatile uint16_t g_tmr10ms;
//volatile uint8_t  g_blinkTmr10ms;


//void per10ms()
//{
//	register uint32_t i ;

//  g_tmr10ms++;
//  g_blinkTmr10ms++;

//  uint8_t enuk = KEY_MENU;
//  uint8_t    in = ~read_keys() ;
//  for( i=1; i<7; i++)
//  {
//    //INP_B_KEY_MEN 1  .. INP_B_KEY_LFT 6
//    keys[enuk].input(in & (1<<i),(EnumKeys)enuk);
//    ++enuk;
//  }
  
////	in = read_trims() ;

////	for( i=1; i<256; i<<=1)
////  {
////    // INP_D_TRM_RH_UP   0 .. INP_D_TRM_LH_UP   7
////    keys[enuk].input(in & i,(EnumKeys)enuk);
////    ++enuk;
////  }

////#if !defined(SIMU)
////	keys[enuk].input( ~PIOB->PIO_PDSR & 0x40,(EnumKeys)enuk); // Rotary Enc. Switch
////#endif
//}



void interrupt5ms()
{
	static uint32_t pre_scale ;		// Used to get 10 Hz counter
	
	sound_5ms() ;
	
	if ( ++pre_scale >= 2 )
	{
		Tenms |= 1 ;			// 10 mS has passed
		
//		if ( Buzzer_count )
//		{
//			if ( --Buzzer_count == 0 )
//			{
//				buzzer_off() ;			
//			}
//		}
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
}

uint16_t aaaa ;
uint16_t cccc ;


int main( void )
{
	uint32_t i ;
	uint16_t time ;

  


	// Serial configure  
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN ;		// Enable clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN ; 		// Enable portB clock
	GPIOB->MODER = (GPIOB->MODER & 0xFF0FFFFF ) | 0x00A00000 ;	// Alternate func.
	GPIOB->AFR[1] = (GPIOB->AFR[1] & 0xFFFF00FF ) | 0x00007700 ;	// Alternate func.
//	USART3->BRR = 0x0C35 ;		// 195.3125 divider => 9600 baud
	USART3->BRR = 0x061A ;		// 195.3125 divider => 9600 baud
	USART3->CR1 = 0x200C ;
	USART3->CR2 = 0 ;
	USART3->CR3 = 0 ;

	init5msTimer() ;
	init_hw_timer() ;

	__enable_irq() ;

	disp_256( RCC_BASE, 4 ) ;
	crlf() ;

	lcd_init();
	
//	GPIO_SetBits(GPIOB, GPIO_Pin_BL);

	lcd_clear();
  lcd_img(42, 0, &splashdata[4], 0, 0);
	refreshDisplay();
	
	init_keys() ;

	init_trims() ;

	init_adc() ;

	setup_switches() ;

	time = get_tmr10ms() ;
	for ( i = 0 ; i < 4000000 ; i += 1 )
	{
		if ( (uint16_t)( get_tmr10ms() - time ) > 100 )
		{
			break ;
		}
	}

	lcd_clear();

// Switches PE2,7,8,9,13,14
	configure_pins( 0x6384, PIN_INPUT | PIN_PULLUP | PIN_PORTE ) ;


	init_adc() ;

	init_main_ppm() ;
	
	init_trainer_ppm() ;

	init_trainer_capture() ;
//	stop_trainer_ppm() ;

	disp_256( TIM1_BASE, 6 ) ;
	crlf() ;

	disk_initialize( 0 ) ;

	dispw_256( GPIOB_BASE, 4 ) ;
	crlf() ;
	
	dispw_256( GPIOC_BASE, 4 ) ;
	crlf() ;
//	dispw_256( DMA2_BASE, 12 ) ;
//	crlf() ;

	start_sound() ;

	I2C_EE_Init() ;

	rtcInit() ;

	CoInitOS();
	
	MainTask = CoCreateTask( main_loop,NULL,5,&main_stk[MAIN_STACK_SIZE-1],MAIN_STACK_SIZE);

#ifdef	DEBUG
	DebugTask = CoCreateTaskEx( handle_serial,NULL,18,&debug_stk[DEBUG_STACK_SIZE-1],DEBUG_STACK_SIZE, 1, FALSE );
	VoiceTask = CoCreateTaskEx( voice_task,NULL,17,&voice_stk[VOICE_STACK_SIZE-1], VOICE_STACK_SIZE, 1, FALSE );
#endif 

	CoStartOS();
	while(1);
}

const char Switches_Str[] = "SA0SA1SA2SB0SB1SB2SC0SC1SC2SD0SD1SD2SE0SE1SE2SF0SF2SG0SG1SG2SH0SH2" ;

extern uint32_t test_sound( void ) ;
extern void stop_sound( void ) ;
uint32_t SoundTime ;

// This is the main task for the RTOS
void main_loop(void* pdata)
{
	uint32_t i ;
	uint16_t time ;
	uint32_t counter ;
	uint32_t counter1 ;
	uint32_t screen ;
	uint32_t ttest = 0 ;
	uint16_t tresult = 0 ;
	uint16_t xxx = 0 ;
	
	counter = 0 ;
	counter1 = 0 ;
	screen = 0 ;

	initWatchdog() ;
	
	for(;;)
	{
		time = get_tmr10ms() ;
		for ( i = 0 ; i < 4000000 ; i += 1 )
		{
			if ( (uint16_t)( get_tmr10ms() - time ) > 10 )
			{
				break ;
			}
			CoTickDelay(1) ;
		}
		wdt_reset() ;

		SPI1->DR = 0xA055 ;		// Test pattern
		for ( i = 0 ; i < 10000 ; i += 1 )
		{
			if ( SPI1->SR & 0x0002 )
			{
				break ;				
			}
		}
		SPI1->DR = 0xA055 ;		// Test pattern

		if ( xxx == 0  )
		{
			txmit( 'X' ) ;
		}
		if ( ++xxx > 7 )
		{
			xxx = 0 ;			
		}
		
		lcd_clear();

		lcd_outhex4( 0, 16, tresult ) ;
		 
		lcd_outhex4( 140, 56, g_tmr10ms ) ;

    if ( keyState((EnumKeys)KEY_MENU) )
		{
			counter += 1 ;
		}
		else
		{
			counter = 0 ;
		}
		if ( counter > 5 )
		{
			screen += 1 ;
			if ( screen > 1 )
			{
				screen = 0 ;				
			}
			counter = 0 ;
		}

    if ( keyState((EnumKeys)KEY_UP) )
		{
			counter1 += 1 ;
		}
		else
		{
			counter1 = 0 ;
		}
		if ( counter1 > 5 )
		{
			ttest = 1 ;
			counter1 = 0 ;
		}

		if ( ttest )
		{
			ttest = 0 ;
			time = get_tmr10ms() ;
			for ( i = 0 ; i < 100 ; i += 1 )
			{
				hw_delay( 20000 ) ;		// 2 mS				
			}
			cccc = TIM13->CNT ;
			tresult = get_tmr10ms() - time ;
			aaaa = TIM13->PSC ;
		}

		if ( screen == 0 )
		{
			lcd_outhex4( 0, 0, CoGetOSTime() ) ;
			lcd_outhex4( 0, 8, TIM6->CNT ) ;
			lcd_outhex4( 20, 8, DMA1_Stream5->NDTR ) ;
			
			lcd_outhex4( 180, 0, tresult ) ;
			lcd_outhex4( 180, 8, aaaa ) ;
			lcd_outhex4( 150, 0, cccc ) ;
			
  		for(i=0; i<6; i++)
  		{
  		  uint8_t y=(5-i)*FH+2*FH;
  		  bool t=keyState((EnumKeys)(KEY_MENU+i));
  		  lcd_putsn_P(172, y,PSTR(" Menu Exit Down   UpRight Left")+5*i,5) ;
  		  lcd_putcAtt(172+FW*5+2,  y,t+'0',t);
  		}

			static uint32_t last = 0 ;
			if ( keyState((EnumKeys)(KEY_RIGHT)) )
			{
				if ( last == 0 )
				{
					queueTone( 75 * 61 / 2, 10 * 10, 0 ) ;
					last = 1 ;
				}
			}
			else
			{
				last = 0 ;
			}

			static uint32_t lastx = 0 ;
			if ( keyState((EnumKeys)(KEY_LEFT)) )
			{
				if ( lastx == 0 )
				{
					queueTone( 65 * 61 / 2, 10 * 10, 0 ) ;
					lastx = 1 ;
				}
			}
			else
			{
				lastx = 0 ;
			}

			uint32_t x ;
  		x=0;
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
		
			lcd_outhex4( 100, 8, read_adc() ) ;

			static uint32_t counterx ;

			if ( ++counterx > 0 )
			{
				for( x = 0 ; x < 8 ; x += 1 )
				{
					g_chans512[x] = ( Analog[x] - 0x800) / 2 ;
				}
				counterx = 0 ;
			}

			lcd_outhex4( 100, 16, Analog[0] ) ;
			lcd_outhex4( 100, 24, Analog[1] ) ;
			lcd_outhex4( 100, 32, Analog[2] ) ;
			lcd_outhex4( 100, 40, Analog[3] ) ;

			lcd_outhex4( 70, 16, Analog[4] ) ;
			lcd_outhex4( 70, 24, Analog[5] ) ;
			lcd_outhex4( 70, 32, Analog[6] ) ;
			lcd_outhex4( 70, 40, Analog[7] ) ;
			lcd_outhex4( 70, 48, Analog[8] ) ;
		 
			lcd_outhex4( 125, 40, ADC1->SR ) ;
			lcd_outhex4( 125, 32, DMA2->LISR & 0x3F ) ;
			lcd_outhex4( 125, 24, DMA2_Stream0->NDTR ) ;
			
		}

		if ( screen == 1 )
		{
			// Test switches here
					
			for( i=0 ; i<5; i += 1)
			{
				lcd_putsnAtt( 0, (i+1)*FH, Switches_Str+3*i,3, keyState((EnumKeys)(SW_SA0+i)) ? INVERS : 0);
			}
			for( i=0 ; i<6; i += 1)
			{
				lcd_putsnAtt( 30, (i+1)*FH, Switches_Str+3*(i+5),3, keyState((EnumKeys)(SW_SA0+i+5)) ? INVERS : 0);
			}
			for( i=0 ; i<6; i += 1)
			{
				lcd_putsnAtt( 60, (i+1)*FH, Switches_Str+3*(i+11),3, keyState((EnumKeys)(SW_SA0+i+11)) ? INVERS : 0);
			}
			for( i=0 ; i<5; i += 1)
			{
				lcd_putsnAtt( 90, (i+1)*FH, Switches_Str+3*(i+17),3, keyState((EnumKeys)(SW_SA0+i+17)) ? INVERS : 0);
			}
			read_adc() ;
			lcd_outhex4( 120, 0, g_ppmIns[0] ) ;
			lcd_outhex4( 120, 8, g_ppmIns[1] ) ;
			lcd_outhex4( 120, 16, g_ppmIns[2] ) ;
			lcd_outhex4( 120, 24, g_ppmIns[3] ) ;
			lcd_outhex4( 120, 32, g_ppmIns[4] ) ;
			lcd_outhex4( 120, 40, g_ppmIns[5] ) ;

//extern uint16_t lastCapt ;
//extern uint16_t lastVal ;
//			lcd_outhex4( 120, 48, lastVal ) ;
//			lcd_outhex4( 120, 56, lastCapt ) ;



 		  lcd_puts_P(170, 0, "GPIO IN" ) ;
 		  lcd_putc(160,  8, 'A' ) ;
			lcd_outhex4( 170, 8, GPIOA->IDR ) ;
 		  lcd_putc(160,  16, 'B' ) ;
			lcd_outhex4( 170, 16, GPIOB->IDR ) ;
 		  lcd_putc(160,  24, 'C' ) ;
			lcd_outhex4( 170, 24, GPIOC->IDR ) ;
 		  lcd_putc(160, 32, 'D' ) ;
			lcd_outhex4( 170, 32, GPIOD->IDR ) ;
 		  lcd_putc(160,  40, 'E' ) ;
			lcd_outhex4( 170, 40, GPIOE->IDR ) ;

			lcd_outhex4( 0, 56, TIM10->CNT ) ;
			lcd_outhex4( 25, 56, TIM10->ARR ) ;
			lcd_outhex4( 50, 56, TIM10->PSC ) ;
			lcd_outhex4( 75, 56, TIM10->CCR1 ) ;


extern void backlight_set( uint16_t brightness ) ;
extern void backlight_on() ;
extern void backlight_off() ;
			static uint16_t bright = 50 ;
 		  bool t=keyState((EnumKeys)(KEY_UP));// up
			if ( t )
			{
				if ( bright < 100 )
				{
					bright += 1 ;
					backlight_set( bright ) ;
				}
			}
 		  t=keyState((EnumKeys)(KEY_DOWN));// down
			if ( t )
			{
				if ( bright )
				{
					bright -= 1 ;
					backlight_set( bright ) ;
				}
			}

			if ( keyState((EnumKeys)(KEY_RIGHT)) )
			{
				backlight_on() ;
			}
			if ( keyState((EnumKeys)(KEY_LEFT)) )
			{
				backlight_off() ;
			}


		}

//		if ( screen == 2 )
//		{
			
			
//		}

		time = get_tmr10ms() ;
		refreshDisplay();
		tresult = get_tmr10ms() - time ;
		
	}
}


void putsChnRaw(uint8_t x,uint8_t y,uint8_t idx,uint8_t att)
{
	uint8_t chanLimit = NUM_SKYXCHNRAW ;
	if ( att & MIX_SOURCE )
	{
#if GVARS
		chanLimit += 6 ;
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
    lcd_putsnAtt(x,y,PSTR("P1  P2  P3  HALFFULLCYC1CYC2CYC3PPM1PPM2PPM3PPM4PPM5PPM6PPM7PPM8CH1 CH2 CH3 CH4 CH5 CH6 CH7 CH8 CH9 CH10CH11CH12CH13CH14CH15CH16CH17CH18CH19CH20CH21CH22CH23CH243POSGV1 GV2 GV3 GV4 GV5 ")+4*(idx-5),4,att);
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

//void putsDrSwitches(uint8_t x,uint8_t y,int8_t idx1,uint8_t att)//, bool nc)
//{
//	const char *pstr ;
//  switch(idx1){
//    case  0:            lcd_putsAtt(x+FW,y,PSTR("---"),att);return;
//    case  MAX_SKYDRSWITCH: lcd_putsAtt(x+FW,y,PSTR("ON "),att);return;
//    case -MAX_SKYDRSWITCH: lcd_putsAtt(x+FW,y,PSTR("OFF"),att);return;
//  }
//	if ( idx1 < 0 )
//	{
//  	lcd_putcAtt(x,y, '!',att);
//	}
//	pstr = get_switches_string()+3*(abs(idx1)-1) ;
//  lcd_putsnAtt(x+FW,y,pstr,3,att);
//	if ( att & CONDENSED )
//	{
//		pstr += 2 ;
//		att &= ~CONDENSED ;
//  	lcd_putcAtt(x+3*FW-1, y,*pstr,att);
//	}
//}

////Type 1-trigA, 2-trigB, 0 best for display
//void putsTmrMode(uint8_t x, uint8_t y, uint8_t attr, uint8_t timer, uint8_t type )
//{
//  int8_t tm = g_model.timer[timer].tmrModeA ;
//	if ( type < 2 )		// 0 or 1
//	{
//	  if(tm<TMR_VAROFS) {
//        lcd_putsnAtt(  x, y, PSTR("OFFABSTHsTH%")+3*abs(tm),3,attr);
////    return;
//  	}
//		else
//		{
//  		tm -= TMR_VAROFS ;
//  		lcd_putsnAtt(  x, y, PSTR( CURV_STR ) + 21 + 3*tm, 3, attr ) ;		// Cheat to get chan# text
//			if ( tm < 9 )
//			{
//				x -= FW ;		
//			}
//  		lcd_putcAtt(x+3*FW,  y,'%',attr);
//		}
//	}
//	if ( ( type == 2 ) || ( ( type == 0 ) && ( tm == 1 ) ) )
//	{
//    tm = g_model.timer[timer].tmrModeB;
//    if(abs(tm)>=(MAX_SKYDRSWITCH))	 //momentary on-off
//		{
//  	  lcd_putcAtt(x+3*FW,  y,'m',attr);
//			if ( tm > 0 )
//			{
//				tm -= MAX_SKYDRSWITCH - 1 ;
//			}
//		}			 
//   	putsDrSwitches( x-1*FW, y, tm, attr );
//	}
//}



//inline int16_t getValue(uint8_t i)
//{
//	int8_t j ;
//	int16_t offset = 0 ;

//  if(i<PPM_BASE) return calibratedStick[i];//-512..512
//  else if(i<PPM_BASE+4) return (g_ppmIns[i-PPM_BASE] - g_eeGeneral.trainer.calib[i-PPM_BASE])*2;
//  else if(i<CHOUT_BASE) return g_ppmIns[i-PPM_BASE]*2;
//  else if(i<CHOUT_BASE+NUM_SKYCHNOUT) return ex_chans[i-CHOUT_BASE];
//#ifdef FRSKY
//  else if(i<CHOUT_BASE+NUM_SKYCHNOUT+NUM_TELEM_ITEMS)
//	{
//		j = TelemIndex[i-CHOUT_BASE-NUM_SKYCHNOUT] ;
//		if ( j >= 0 )
//		{
//      if ( j == FR_ALT_BARO )
//			{
//        offset = AltOffset ;
//			}
//			return FrskyHubData[j] + offset ;
//		}
//		else if ( j == -3 )		// Battery
//		{
//			return g_vbat100mV ;
//		}
//		else
//		{
//			return s_timer[j+2].s_timerVal ;
//		}
//	}
//#endif
//  else return 0;
//}



bool Last_switch[NUM_SKYCSW] ;

//bool getSwitch(int8_t swtch, bool nc, uint8_t level)
//{
//  bool ret_value ;
//  uint8_t cs_index ;
  
//	if(level>5) return false ; //prevent recursive loop going too deep

//  switch(swtch){
//    case  0:            return  nc;
//    case  MAX_SKYDRSWITCH: return  true ;
//    case -MAX_SKYDRSWITCH: return  false ;
//  }

//	if ( swtch > MAX_SKYDRSWITCH )
//	{
//		return false ;
//	}

//  uint8_t dir = swtch>0;
//  if(abs(swtch)<(MAX_SKYDRSWITCH-NUM_SKYCSW)) {
//    if(!dir) return ! keyState((enum EnumKeys)(SW_BASE-swtch-1));
//    return            keyState((enum EnumKeys)(SW_BASE+swtch-1));
//  }

//  //use putsChnRaw
//  //input -> 1..4 -> sticks,  5..8 pots
//  //MAX,FULL - disregard
//  //ppm
//  cs_index = abs(swtch)-(MAX_SKYDRSWITCH-NUM_SKYCSW);
//  SKYCSwData &cs = g_model.customSw[cs_index];
//  if(!cs.func) return false;

//  if ( level>4 )
//  {
//    ret_value = Last_switch[cs_index] ;
//    return swtch>0 ? ret_value : !ret_value ;
//  }

//  int8_t a = cs.v1;
//  int8_t b = cs.v2;
//  int16_t x = 0;
//  int16_t y = 0;
//	uint8_t valid = 1 ;

//  // init values only if needed
//  uint8_t s = CS_STATE(cs.func);

//  if(s == CS_VOFS)
//  {
//      x = getValue(cs.v1-1);
//#ifdef FRSKY
//      if (cs.v1 > CHOUT_BASE+NUM_SKYCHNOUT)
//			{
//        y = convertTelemConstant( cs.v1-CHOUT_BASE-NUM_SKYCHNOUT-1, cs.v2 ) ;
//				valid = telemItemValid( cs.v1-CHOUT_BASE-NUM_SKYCHNOUT-1 ) ;
//			}
//      else
//#endif
//      y = calc100toRESX(cs.v2);
//  }
//  else if(s == CS_VCOMP)
//  {
//      x = getValue(cs.v1-1);
//      y = getValue(cs.v2-1);
//  }

//  switch (cs.func) {
//  case (CS_VPOS):
//      ret_value = (x>y);
//      break;
//  case (CS_VNEG):
//      ret_value = (x<y) ;
//      break;
//  case (CS_APOS):
//  {
//      ret_value = (abs(x)>y) ;
//  }
//      break;
//  case (CS_ANEG):
//  {
//      ret_value = (abs(x)<y) ;
//  }
//      break;

//  case (CS_AND):
//  case (CS_OR):
//  case (CS_XOR):
//  {
//    bool res1 = getSwitch(a,0,level+1) ;
//    bool res2 = getSwitch(b,0,level+1) ;
//    if ( cs.func == CS_AND )
//    {
//      ret_value = res1 && res2 ;
//    }
//    else if ( cs.func == CS_OR )
//    {
//      ret_value = res1 || res2 ;
//    }
//    else  // CS_XOR
//    {
//      ret_value = res1 ^ res2 ;
//    }
//  }
//  break;

//  case (CS_EQUAL):
//      ret_value = (x==y);
//      break;
//  case (CS_NEQUAL):
//      ret_value = (x!=y);
//      break;
//  case (CS_GREATER):
//      ret_value = (x>y);
//      break;
//  case (CS_LESS):
//      ret_value = (x<y);
//      break;
//  case (CS_EGREATER):
//      ret_value = (x>=y);
//      break;
//  case (CS_ELESS):
//      ret_value = (x<=y);
//      break;
//  case (CS_TIME):
//      ret_value = CsTimer[cs_index] >= 0 ;
//      break;
//  default:
//      ret_value = false;
//      break;
//  }
//	if ( valid == 0 )			// Catch telemetry values not present
//	{
//     ret_value = false;
//	}
//	if ( ret_value )
//	{
//		if ( cs.andsw )
//		{
//			int8_t x ;
//			x = cs.andsw ;
//			if ( x > 8 )
//			{
//				x += 1 ;
//			}
//      ret_value = getSwitch( x, 0, level+1) ;
//		}
//	}
//	Last_switch[cs_index] = ret_value ;
//	return swtch>0 ? ret_value : !ret_value ;

//}


//int8_t getMovedSwitch()
//{
//  static uint8_t switches_states = 0;
//  static uint16_t s_last_time = 0;

//  int8_t result = 0;

//  for (uint8_t i=MAX_PSWITCH; i>0; i--) {
//    bool prev;
//    uint8_t mask = 0;
//    if (i <= 3) {
//      mask = (1<<(i-1));
//      prev = (switches_states & mask);
//    }
//    else if (i <= 6) {
//      prev = ((switches_states & 0x18) == ((i-3) << 3));
//    }
//    else {
//      mask = (1<<(i-2));
//      prev = (switches_states & mask);
//    }
//    bool next = getSwitch(i, 0, 0) ;
//    if (prev != next) {
//      if (i!=MAX_PSWITCH || next==true)
//	      result = i;
//      if (mask)
//        switches_states ^= mask;
//      else
//        switches_states = (switches_states & 0xE7) | ((i-3) << 3);
//    }
//  }

//  if (get_tmr10ms() - s_last_time > 10)
//    result = 0;

//  s_last_time = get_tmr10ms();

//  return result;
//}


// **** SPI1 does not seem to support I2S *** !!!

// Code to init SPI1 in I2S mode for DSM2 serial
//void init_i2s1()
//{
//  uint32_t i ;
////	RCC->PLLI2SCFGR = (PLLI2SN << 6) | (PLLI2SR << 28);
//	RCC->PLLI2SCFGR = ( 192<< 6) | ( 6 << 28) ;	// I2S clock 32MHz
//	RCC->CR |= 1 << 26 ;			// Enable

//	// Should wait for bit 27 to be set!
//	for ( i = 0 ; i < 50000 ; i += 1 )
//	{
//		if ( RCC->CR & (1 << 27) )			// Ready
//		{
//			break ;
//		}
//	}
//	RCC->CFGR &= ~(uint32_t)RCC_CFGR_I2SSRC ;
//	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN ; 		// Enable portA clock
//	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN ;			// Enable SPI1 clock
//	// PA7 assigned to SPI1
//	configure_pins( 0x0080, PIN_PERIPHERAL | PIN_PORTA | PIN_PER_5 ) ;
//	SPI1->CR1 = 0 ;
////	SPI1->I2SCFGR = 0 ;
//	SPI1->I2SCFGR = 0x0A10 ;
//	SPI1->I2SPR = 64 ;		// Clock divider
//	SPI1->I2SCFGR |= 0x0400 ;		// Enable
//}



