

#define PIN_ENABLE			0x001
#define PIN_PERIPHERAL	0x000
#define PIN_INPUT				0x002
#define PIN_OUTPUT			0x000
#define PIN_PULLUP			0x004
#define PIN_NO_PULLUP		0x000
#define PIN_PULLDOWN		0x008
#define PIN_NO_PULLDOWN	0x000
#define PIN_PERI_MASK_L	0x010
#define PIN_PERI_MASK_H	0x020
#define PIN_PER_A				0x000
#define PIN_PER_B				0x010
#define PIN_PER_C				0x020
#define PIN_PER_D				0x030
#define PIN_PORT_MASK		0x0C0
#define PIN_PORTA				0x000
#define PIN_PORTB				0x040
#define PIN_PORTC				0x080
#define PIN_LOW					0x000
#define PIN_HIGH				0x100



extern uint16_t Analog_values[NUMBER_ANALOG] ;
extern volatile uint32_t Spi_complete ;


extern void putEvent( register uint8_t evt) ;
extern uint32_t read_keys( void ) ;
extern uint32_t read_trims( void ) ;
extern void UART_Configure( uint32_t baudrate, uint32_t masterClock) ;
extern void txmit( uint8_t c ) ;
extern void uputs( register char *string ) ;
extern uint16_t rxuart( void ) ;
extern void txmit2nd( uint8_t c ) ;
extern uint16_t rx2nduart( void ) ;
extern void UART3_Configure( uint32_t baudrate, uint32_t masterClock) ;
extern void txmitBt( uint8_t c ) ;
extern uint16_t rxBtuart( void ) ;

extern void poll2ndUsart10mS( void ) ;
//extern void charProcess( uint8_t byte ) ;
extern void poll2ndUsart10mS( void ) ;
extern void startPdcUsartReceive( void ) ;
extern void endPdcUsartReceive( void ) ;
extern void rxPdcUsart( void (*pChProcess)(uint8_t x) ) ;
extern uint32_t txPdcUsart( uint8_t *buffer, uint32_t size ) ;
extern uint32_t txPdcPending( void ) ;

extern uint32_t keyState( enum EnumKeys enuk) ;
extern void per10ms( void ) ;
extern uint8_t getEvent( void ) ;
extern void pauseEvents(uint8_t event) ;
extern void killEvents(uint8_t event) ;
extern void init_spi( void ) ;
extern void end_spi( void ) ;
extern void eeprom_write_enable( void ) ;
extern uint32_t eeprom_read_status( void ) ;
extern uint32_t  eeprom_write_one( uint8_t byte ) ;
extern uint32_t spi_operation( uint8_t *tx, uint8_t *rx, uint32_t count ) ;
extern uint32_t spi_action( uint8_t *command, uint8_t *tx, uint8_t *rx, uint32_t comlen, uint32_t count ) ;
extern uint32_t spi_PDC_action( uint8_t *command, uint8_t *tx, uint8_t *rx, uint32_t comlen, uint32_t count ) ;
extern void crlf( void ) ;
extern void p8hex( uint32_t value ) ;
extern void p4hex( uint16_t value ) ;
extern void p2hex( unsigned char c ) ;
extern void hex_digit_send( unsigned char c ) ;
extern void read_9_adc(void ) ;
extern void init_adc( void ) ;
extern void init_ssc( void ) ;
extern void eeprom_write_byte_cmp (uint8_t dat, uint16_t pointer_eeprom) ;
extern void eeWriteBlockCmp(const void *i_pointer_ram, void *i_pointer_eeprom, size_t size) ;
extern void eeprom_read_block( void *i_pointer_ram, const void *i_pointer_eeprom, register uint32_t size ) ;
void start_ppm_capture( void ) ;
void end_ppm_capture( void ) ;

extern void init_ssc( void ) ;
extern void disable_ssc( void ) ;

uint32_t read32_eeprom_data( uint32_t eeAddress, register uint8_t *buffer, uint32_t size, uint32_t immediate ) ;

extern void configure_pins( uint32_t pins, uint16_t config ) ;

