

extern void init_eeprom( void ) ;

extern void handle_serial( void ) ;
extern void hello( void ) ;
extern void dbl9x( void ) ;
//extern uint32_t read_switch( enum EnumKeys enuk ) ;
extern void txmit( uint8_t c ) ;
extern void disp_mem( register uint32_t address ) ;
extern void disp_256( uint32_t address, uint32_t lines ) ;


struct t_file_entry
{
	uint32_t block_no ;
	uint32_t sequence_no ;
	uint16_t size ;
	uint16_t flags ;			// Unused at present
} ;

extern struct t_file_entry File_system[] ;


