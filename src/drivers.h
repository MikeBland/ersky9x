

extern uint16_t Analog_values[8] ;
extern volatile uint32_t Spi_complete ;


extern void putEvent( register uint8_t evt) ;
extern uint32_t read_keys( void ) ;
extern uint32_t read_trims( void ) ;
extern void UART_Configure( uint32_t baudrate, uint32_t masterClock) ;
extern void txmit( uint8_t c ) ;
extern void uputs( register char *string ) ;
extern uint16_t rxuart( void ) ;

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
extern void read_8_adc(void ) ;
extern void init_adc( void ) ;
extern void eeprom_write_byte_cmp (uint8_t dat, uint16_t pointer_eeprom) ;
extern void eeWriteBlockCmp(const void *i_pointer_ram, void *i_pointer_eeprom, size_t size) ;
extern void eeprom_read_block( void *i_pointer_ram, const void *i_pointer_eeprom, register uint32_t size ) ;
void start_ppm_capture( void ) ;
void end_ppm_capture( void ) ;


