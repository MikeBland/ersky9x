
#include <stdint.h>
#include <stdlib.h>
#include "ersky9x.h"
#include "timers.h"
#include "logicio.h"
#include "analog.h"
#include "drivers.h"

#include "x9d\stm32f2xx.h"
#include "x9d\stm32f2xx_gpio.h"
#include "x9d\stm32f2xx_rcc.h"
#include "x9d\hal.h"

#include "lcd.h"



const uint8_t splashdata[] = { 'S','P','S',0,
#include "s9xsplash.lbm"
	'S','P','E',0};

#include "sticks.lbm"


int main( void ) ;


void crlf( void ) ;
void p8hex( uint32_t value ) ;
void p4hex( uint16_t value ) ;
void p2hex( unsigned char c ) ;
void hex_digit_send( unsigned char c ) ;
void txmit( uint8_t c ) ;


uint32_t Master_frequency ;

volatile uint32_t Tenms ;						// Modified in interrupt routine
volatile uint8_t tick10ms = 0 ;

uint8_t AlarmTimer = 100 ;		// Units of 10 mS
uint8_t AlarmCheckFlag = 0 ;
uint8_t CsCheckFlag = 0 ;
uint8_t VoiceTimer = 10 ;		// Units of 10 mS
uint8_t VoiceCheckFlag = 0 ;
//int8_t  CsTimer[NUM_SKYCSW] ;


void txmit( uint8_t c )
{
	/* Wait for the transmitter to be ready */
  while ( (USART3->SR & USART_SR_TXE) == 0 ) ;

  /* Send character */
	USART3->DR = c ;
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

void disp_256( register uint32_t address, register uint32_t lines ) ;

void disp_256( register uint32_t address, register uint32_t lines )
{
	register uint32_t i ;
	register uint32_t j ;
	for ( i = 0 ; i < lines ; i += 1 )
	{
		p8hex( address ) ;
		for ( j = 0 ; j < 16 ; j += 1 )
		{
			txmit(' ') ;
			p2hex( *( (uint8_t *)address++ ) ) ;
		}
		crlf() ;
	}
}


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
	
//	sound_5ms() ;
	
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
	uint32_t counter ;
	uint32_t counter1 ;
	uint32_t screen ;
	uint32_t ttest = 0 ;
	uint16_t tresult = 0 ;
  
	lcd_init();
  
	
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

	
	GPIO_SetBits(GPIOB, GPIO_Pin_BL);

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

	counter = 0 ;
	counter1 = 0 ;
	screen = 0 ;

	init_ppm() ;

	for(;;)
	{
		time = get_tmr10ms() ;
		for ( i = 0 ; i < 4000000 ; i += 1 )
		{
			if ( (uint16_t)( get_tmr10ms() - time ) > 25 )
			{
				break ;
			}
		}
		txmit( 'X' ) ;
		
		lcd_clear();
		
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
			screen ^= 1 ;
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
				lcd_putsnAtt( 0, (i+1)*FH, "SA0SA2SB0SB1SB2SC0SC1SC2SD0SD1SD2SE0SE1SE2SF0SF1SF2SG0SG1SG2SH0SH2"+3*i,3, keyState((EnumKeys)(SW_SA0+i)) ? INVERS : 0);
			}
			for( i=0 ; i<6; i += 1)
			{
				lcd_putsnAtt( 30, (i+1)*FH, "SA0SA2SB0SB1SB2SC0SC1SC2SD0SD1SD2SE0SE1SE2SF0SF1SF2SG0SG1SG2SH0SH2"+3*(i+5),3, keyState((EnumKeys)(SW_SA0+i+5)) ? INVERS : 0);
			}
			for( i=0 ; i<6; i += 1)
			{
				lcd_putsnAtt( 60, (i+1)*FH, "SA0SA2SB0SB1SB2SC0SC1SC2SD0SD1SD2SE0SE1SE2SF0SF1SF2SG0SG1SG2SH0SH2"+3*(i+11),3, keyState((EnumKeys)(SW_SA0+i+11)) ? INVERS : 0);
			}
			for( i=0 ; i<5; i += 1)
			{
				lcd_putsnAtt( 90, (i+1)*FH, "SA0SA2SB0SB1SB2SC0SC1SC2SD0SD1SD2SE0SE1SE2SF0SF1SF2SG0SG1SG2SH0SH2"+3*(i+17),3, keyState((EnumKeys)(SW_SA0+i+17)) ? INVERS : 0);
			}

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



		}
		refreshDisplay();
		
	}
}

