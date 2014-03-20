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
#include "ersky9x.h"
#include "stdio.h"
#include "inttypes.h"
#include "string.h"
#include "myeeprom.h"
#include "drivers.h"
#include "file.h"
#include "ff.h"
#ifdef FRSKY
#include "frsky.h"
#endif
#ifndef SIMU
#include "CoOS.h"
#endif
#ifdef REVX
#include "sound.h"
#endif
#include "Stringidx.h"

extern PROGMEM s9xsplash[] ;


// EEPROM Addresses
// General 0x00000 - 0x01FFF
// Model01 0x02000 - 0x03FFF
// Model02 0x04000 - 0x05FFF
// Model03 0x06000 - 0x07FFF
// Model04 0x08000 - 0x09FFF
// Model05 0x0A000 - 0x0BFFF
// Model06 0x0C000 - 0x0DFFF
// Model07 0x0E000 - 0x0FFFF
// Model08 0x10000 - 0x11FFF
// Model09 0x12000 - 0x13FFF
// Model10 0x14000 - 0x15FFF
// Model11 0x16000 - 0x17FFF
// Model12 0x18000 - 0x19FFF
// Model13 0x1A000 - 0x1BFFF
// Model14 0x1C000 - 0x1DFFF
// Model15 0x1E000 - 0x1FFFF
// Model16 0x20000 - 0x21FFF
// Model17 0x22000 - 0x23FFF
// Model18 0x24000 - 0x25FFF
// Model19 0x26000 - 0x27FFF
// Model20 0x28000 - 0x29FFF



// Logic for storing to EEPROM/loading from EEPROM
// If main needs to wait for the eeprom, call mainsequence without actioning menus
// General configuration
// 'main' set flag STORE_GENERAL
// 'eeCheck' sees flag STORE_GENERAL, copies data, starts 2 second timer, clears flag STORE_GENERAL
// 'eeCheck' sees 2 second timer expire, locks copy, initiates write, enters WRITE_GENERAL mode
// 'eeCheck' completes write, unlocks copy, exits WRITE_GENERAL mode

// 'main' set flag STORE_MODEL(n)

// 'main' set flag STORE_MODEL_TRIM(n)


// 'main' needs to load a model




// These may not be needed, or might just be smaller
uint8_t Spi_tx_buf[24] ;

uint8_t Spi_rx_buf[24] ;


struct t_file_entry File_system[MAX_MODELS+1] ;

unsigned char ModelNames[MAX_MODELS+1][sizeof(g_model.name)+1] ;		// Allow for general

uint16_t General_timer ;
uint16_t Model_timer ;
uint32_t Update_timer ;
uint8_t General_dirty ;
uint8_t Model_dirty ;

uint8_t Ee32_general_write_pending ;
uint8_t Ee32_model_write_pending ;
uint8_t Ee32_model_delete_pending ;

uint8_t	Eeprom32_process_state ;
uint8_t	Eeprom32_state_after_erase ;
uint8_t	Eeprom32_write_pending ;
uint8_t Eeprom32_file_index ;
uint8_t *Eeprom32_buffer_address ;
uint8_t *Eeprom32_source_address ;
uint32_t Eeprom32_address ;
uint32_t Eeprom32_data_size ;


#define EE_WAIT			0
#define EE_NO_WAIT	1


// States in Eeprom32_process_state
#define E32_IDLE							1
#define E32_ERASESENDING			2
#define E32_ERASEWAITING			3
#define E32_WRITESENDING			4
#define E32_WRITEWAITING			5
#define E32_READSENDING				6
#define E32_READWAITING				7
#define E32_BLANKCHECK				8
#define E32_WRITESTART				9

//#define E32_READING						10		// Set elsewhere as a lock


void handle_serial( void ) ;

bool ee32ModelExists(uint8_t id) ;
uint32_t get_current_block_number( uint32_t block_no, uint16_t *p_size, uint32_t *p_seq ) ;
uint32_t read32_eeprom_data( uint32_t eeAddress, register uint8_t *buffer, uint32_t size, uint32_t immediate ) ;
uint32_t write32_eeprom_block( uint32_t eeAddress, register uint8_t *buffer, uint32_t size, uint32_t immediate ) ;
void ee32_read_model_names( void ) ;
void ee32LoadModelName(uint8_t id, unsigned char*buf,uint8_t len) ;
void ee32_update_name( uint32_t id, uint8_t *source ) ;
void convertModel( SKYModelData *dest, ModelData *source ) ;



//void eeprom_process( void ) ;
//uint32_t ee32_process( void ) ;

// New file system

// Start with 16 model memories, initial test uses 34 and 35 for these
// Blocks 0 and 1, general
// Blocks 2 and 3, model 1
// Blocks 4 and 5, model 2 etc
// Blocks 32 and 33, model 16

uint8_t Current_general_block ;		// 0 or 1 is active block
uint8_t Other_general_block_blank ;

struct t_eeprom_header
{
	uint32_t sequence_no ;		// sequence # to decide which block is most recent
	uint16_t data_size ;			// # bytes in data area
	uint8_t flags ;
	uint8_t hcsum ;
} ;

// Structure of data in a block
struct t_eeprom_block
{
	struct t_eeprom_header header ;
	union
	{
		uint8_t bytes[4088] ;
		uint32_t words[1022] ;
	} data ;
} ;


#define EEPROM_BUFFER_SIZE ((sizeof(SKYModelData) + sizeof( struct t_eeprom_header ) + 3)/4)


struct t_eeprom_buffer
{
	struct t_eeprom_header header ;
	union t_eeprom_data
	{
		EEGeneral general_data ;
		ModelData model_data ;
		SKYModelData sky_model_data ;
		uint32_t words[ EEPROM_BUFFER_SIZE ] ;
	} data ;	
} Eeprom_buffer ;


static void ee32WaitFinished()
{
  while ( ee32_check_finished() == 0 )
	{
		if ( General_timer )
		{
			General_timer = 1 ;		// Make these happen soon
		}
		if ( Model_timer )
		{
			Model_timer = 1 ;
		}
    ee32_process();
#ifdef SIMU
    sleep(5/*ms*/);
#endif
  }
}

// genaral data needs to be written to EEPROM
void ee32StoreGeneral()
{
	General_dirty = 1 ;
	General_timer = 500 ;		// 5 seconds timeout before writing
}


// Store model to EEPROM, trim is non-zero if this is the result of a trim change
void ee32StoreModel( uint8_t modelNumber, uint8_t trim )
{
	Model_dirty = modelNumber + 1 ;
	Model_timer = 500 ;	
	ee32_update_name( Model_dirty, (uint8_t *)&g_model ) ;		// In case it's changed
}

void ee32_delete_model( uint8_t id )
{
	uint8_t buffer[sizeof(g_model.name)+1] ;
  memset( buffer, ' ', sizeof(g_model.name) ) ;
	ee32_update_name( id + 1, buffer ) ;
	Ee32_model_delete_pending = id + 1 ;
	ee32_process() ;		// Kick it off
	ee32WaitFinished() ;
}

void ee32_read_model_names()
{
	uint32_t i ;

	for ( i = 1 ; i <= MAX_MODELS ; i += 1 )
	{
		ee32LoadModelName( i, ModelNames[i], sizeof(g_model.name) ) ;
	}
}

void ee32_update_name( uint32_t id, uint8_t *source )
{
	uint8_t * p ;
	uint32_t i ;

	p = ModelNames[id] ;
	for ( i = 0 ; i < sizeof(g_model.name) ; i += 1 )
	{
		*p++ = *source++ ;
	}
	*p = '\0' ;
}

bool ee32CopyModel(uint8_t dst, uint8_t src)
{
  uint16_t size = File_system[src].size ;
	
	while (ee32_check_finished() == 0)
	{	// wait
#ifndef SIMU
		if ( General_timer )
		{
			General_timer = 1 ;		// Make these happen soon
		}
		if ( Model_timer )
		{
			Model_timer = 1 ;
		}
		CoTickDelay(1) ;					// 2mS for now
#endif
	}

  read32_eeprom_data( (File_system[src].block_no << 12) + sizeof( struct t_eeprom_header), ( uint8_t *)&Eeprom_buffer.data.model_data, size, 0 ) ;

  if (size > sizeof(g_model.name))
    memcpy( ModelNames[dst], Eeprom_buffer.data.model_data.name, sizeof(g_model.name)) ;
  else
    memset( ModelNames[dst], ' ', sizeof(g_model.name)) ;

  Eeprom32_source_address = (uint8_t *)&Eeprom_buffer.data.model_data ;		// Get data from here
  Eeprom32_data_size = sizeof(g_model) ;																	// This much
  Eeprom32_file_index = dst ;																							// This file system entry
  Eeprom32_process_state = E32_BLANKCHECK ;
  ee32WaitFinished() ;
  return true;
}

void ee32SwapModels(uint8_t id1, uint8_t id2)
{
  ee32WaitFinished();
//  // eeCheck(true) should have been called before entering here

  uint16_t id2_size = File_system[id2].size ;
  uint32_t id2_block_no = File_system[id2].block_no ;

  ee32CopyModel(id2, id1);

//  // block_no(id1) has been shifted now, but we have the size
  if (id2_size > sizeof(g_model.name))
	{
    read32_eeprom_data( (id2_block_no << 12) + sizeof( struct t_eeprom_header), ( uint8_t *)&Eeprom_buffer.data.model_data, id2_size, 0 ) ;
    memcpy( ModelNames[id1], Eeprom_buffer.data.model_data.name, sizeof(g_model.name)) ;
  }
  else
	{
    memset( ModelNames[id1], ' ', sizeof(g_model.name)) ;
		id2_size = 0 ;
  }

  Eeprom32_source_address = (uint8_t *)&Eeprom_buffer.data.model_data ;		// Get data from here
  Eeprom32_data_size = id2_size ;																					// This much
  Eeprom32_file_index = id1 ;																							// This file system entry
  Eeprom32_process_state = E32_BLANKCHECK ;
  ee32WaitFinished();
}

// Read eeprom data starting at random address
uint32_t read32_eeprom_data( uint32_t eeAddress, register uint8_t *buffer, uint32_t size, uint32_t immediate )
{
#ifdef SIMU
  assert(size);
  eeprom_pointer = eeAddress;
  eeprom_buffer_data = (char*)buffer;
  eeprom_buffer_size = size;
  eeprom_read_operation = true;
  Spi_complete = false;
  sem_post(eeprom_write_sem);
  int x;
#else
	register uint8_t *p ;
	register uint32_t x ;

	p = Spi_tx_buf ;
	*p = 3 ;		// Read command
	*(p+1) = eeAddress >> 16 ;
	*(p+2) = eeAddress >> 8 ;
	*(p+3) = eeAddress ;		// 3 bytes address
	spi_PDC_action( p, 0, buffer, 4, size ) ;
#endif

	if ( immediate )
	{
		return 0 ;		
	}
	for ( x = 0 ; x < 100000 ; x += 1  )
	{
		if ( Spi_complete )
		{
			break ;				
		}
#ifdef SIMU
    sleep(5/*ms*/);
#endif
	}
	return x ;
}


uint32_t write32_eeprom_block( uint32_t eeAddress, register uint8_t *buffer, uint32_t size, uint32_t immediate )
{
	register uint8_t *p ;
	register uint32_t x ;

	eeprom_write_enable() ;

	p = Spi_tx_buf ;
	*p = 2 ;		// Write command
	*(p+1) = eeAddress >> 16 ;
	*(p+2) = eeAddress >> 8 ;
	*(p+3) = eeAddress ;		// 3 bytes address
	spi_PDC_action( p, buffer, 0, 4, size ) ;

	if ( immediate )
	{
		return 0 ;		
	}
	for ( x = 0 ; x < 100000 ; x += 1  )
	{
		if ( Spi_complete )
		{
			break ;				
		}        			
	}
	return x ; 
}

uint8_t byte_checksum( uint8_t *p, uint32_t size )
{
	uint32_t csum ;

	csum = 0 ;
	while( size )
	{
		csum += *p++ ;
		size -= 1 ;
	}
	return csum ;
}

uint32_t ee32_check_header( struct t_eeprom_header *hptr )
{
	uint8_t csum ;

	csum = byte_checksum( ( uint8_t *) hptr, 7 ) ;
	if ( csum == hptr->hcsum )
	{
		return 1 ;
	}
	return 0 ;
}

// Pass in an even block number, this and the next block will be checked
// to see which is the most recent, the block_no of the most recent
// is returned, with the corresponding data size if required
// and the sequence number if required
uint32_t get_current_block_number( uint32_t block_no, uint16_t *p_size, uint32_t *p_seq )
{
	struct t_eeprom_header b0 ;
	struct t_eeprom_header b1 ;
  uint32_t sequence_no ;
  uint16_t size ;
	read32_eeprom_data( block_no << 12, ( uint8_t *)&b0, sizeof(b0), EE_WAIT ) ;		// Sequence # 0
	read32_eeprom_data( (block_no+1) << 12, ( uint8_t *)&b1, sizeof(b1), EE_WAIT ) ;	// Sequence # 1

	if ( ee32_check_header( &b0 ) == 0 )
	{
		b0.sequence_no = 0 ;
		b0.data_size = 0 ;
		b0.flags = 0 ;
	}

	size = b0.data_size ;
  sequence_no = b0.sequence_no ;
	if ( ee32_check_header( &b0 ) == 0 )
	{
		if ( ee32_check_header( &b1 ) != 0 )
		{
  		size = b1.data_size ;
		  sequence_no = b1.sequence_no ;
			block_no += 1 ;
		}
		else
		{
			size = 0 ;
			sequence_no = 1 ;
		}
	}
	else
	{
		if ( ee32_check_header( &b1 ) != 0 )
		{
			if ( b1.sequence_no > b0.sequence_no )
			{
	  		size = b1.data_size ;
			  sequence_no = b1.sequence_no ;
				block_no += 1 ;
			}
		}
	}
  
	if ( size == 0xFFFF )
	{
		size = 0 ;
	}
  if ( p_size )
	{
		*p_size = size ;
	}
  if ( sequence_no == 0xFFFFFFFF )
	{  
		sequence_no = 0 ;
	}
  if ( p_seq )
	{
		*p_seq = sequence_no ;
	}
//	Block_needs_erasing = erase ;		
  
	return block_no ;
}

bool ee32LoadGeneral()
{
	uint16_t size ;
	
	size = File_system[0].size ;

  memset(&g_eeGeneral, 0, sizeof(EEGeneral));

	if ( size > sizeof(EEGeneral) )
	{
		size = sizeof(EEGeneral) ;
	}

	if ( size )
	{
		read32_eeprom_data( ( File_system[0].block_no << 12) + sizeof( struct t_eeprom_header), ( uint8_t *)&g_eeGeneral, size, 0 ) ;
	}
	else
	{
		return false ;		// No data to load
	}

//  for(uint8_t i=0; i<sizeof(g_eeGeneral.ownerName);i++) // makes sure name is valid
//  {
//      uint8_t idx = char2idx(g_eeGeneral.ownerName[i]);
//      g_eeGeneral.ownerName[i] = idx2char(idx);
//  }

  if(g_eeGeneral.myVers<MDVERS)
      sysFlags |= sysFLAG_OLD_EEPROM; // if old EEPROM - Raise flag

  g_eeGeneral.myVers   =  MDVERS; // update myvers

  uint16_t sum=0;
  if(size>(sizeof(EEGeneral)-20)) for(uint8_t i=0; i<12;i++) sum+=g_eeGeneral.calibMid[i];
	else return false ;
  return g_eeGeneral.chkSum == sum;
}

void ee32WaitLoadModel(uint8_t id)
{
	// Wait for EEPROM to be idle
	if ( General_timer )
	{
		General_timer = 1 ;		// Make these happen soon
	}
	if ( Model_timer )
	{
		Model_timer = 1 ;
	}

	while( ( Eeprom32_process_state != E32_IDLE )
			|| ( General_timer )
			|| ( Model_timer )
			|| ( Ee32_model_delete_pending) )
	{
		mainSequence( NO_MENU ) ;		// Wait for EEPROM to be IDLE
	}

	ee32LoadModel( id ) ;		// Now get the model
}

extern void closeLogs( void ) ;

void ee32LoadModel(uint8_t id)
{
	uint16_t size ;
	uint8_t version = 255 ;

  closeLogs() ;

    if(id<MAX_MODELS)
    {
			size =  File_system[id+1].size ;
			if ( sizeof(g_model) < 720 )
			{
				version = 0 ;
			}
			memset(&g_model, 0, sizeof(g_model));

			if ( size < 720 )
			{
				memset(&g_oldmodel, 0, sizeof(g_oldmodel));
				if ( size > sizeof(g_oldmodel) )
				{
					size = sizeof(g_oldmodel) ;
				}
			}
       
			if ( size > sizeof(g_model) )
			{
				size = sizeof(g_model) ;
			}
			 
      if(size<256) // if not loaded a fair amount
      {
        modelDefault(id) ;
      }
			else
			{
				if ( size < 720 )
				{
					read32_eeprom_data( ( File_system[id+1].block_no << 12) + sizeof( struct t_eeprom_header), ( uint8_t *)&g_oldmodel, size, 0 ) ;
					convertModel( &g_model, &g_oldmodel ) ;
				}
				else
				{
					read32_eeprom_data( ( File_system[id+1].block_no << 12) + sizeof( struct t_eeprom_header), ( uint8_t *)&g_model, size, 0 ) ;
				}	 
				
				if ( version != 255 )
				{
					g_model.modelVersion = 0 ;					// default version number
				}
			}

      for(uint8_t i=0; i<sizeof(g_model.name);i++) // makes sure name is valid
      {
          uint8_t idx = char2idx(g_model.name[i]);
          g_model.name[i] = idx2char(idx);
      }

//			g_model.modelVersion = MDSKYVERS ; //update mdvers

			if ( g_model.numBlades == 0 )
			{
				g_model.numBlades = g_model.xnumBlades + 2 ;				
			}
			if ( g_model.protocol == PROTO_PPM16 )			// PPM16 ?
			{
				g_model.protocol = PROTO_PPM ;
			}

#ifdef FIX_MODE

// check for updating mix sources
	if ( g_model.modelVersion < 2 )
	{
   	for(uint8_t i=0;i<MAX_MIXERS;i++)
		{
			SKYMixData *md = &g_model.mixData[i] ;
      if (md->srcRaw)
			{
       	if (md->srcRaw <= 4)		// Stick
				{
					md->srcRaw = modeFixValue( md->srcRaw-1 ) ;
				}
			}
		}
		for (uint8_t i = 0 ; i < NUM_SKYCSW ; i += 1 )
		{
    	SKYCSwData *cs = &g_model.customSw[i];
    	uint8_t cstate = CS_STATE(cs->func);
			uint8_t t = 0 ;
    	if(cstate == CS_VOFS)
			{
				t = 1 ;
			}
			else if(cstate == CS_VCOMP)
			{
				t = 1 ;
      	if (cs->v2)
				{
    		  if (cs->v2 <= 4)		// Stick
					{
    	    	cs->v2 = modeFixValue( cs->v2-1 ) ;
					}
				}
			}
			if ( t )
			{
      	if (cs->v1)
				{
    		  if (cs->v1 <= 4)		// Stick
					{
    	    	cs->v1 = modeFixValue( cs->v1-1 ) ;
					}
				}
			}
		}
		ExpoData texpoData[4] ;
    memmove( &texpoData, &g_model.expoData, sizeof(texpoData) ) ;
		for (uint8_t i = 0 ; i < 4 ; i += 1 )
		{
			uint8_t dest = modeFixValue( i ) - 1 ;
    	memmove( &g_model.expoData[dest], &texpoData[i], sizeof(texpoData[0]) ) ;
		}
		alert(PSTR(STR_CHK_MIX_SRC));
		g_model.modelVersion = 2 ;
		STORE_MODELVARS ;
	}
#endif


#ifdef FRSKY
  FrskyAlarmSendState |= 0x40 ;		// Get RSSI Alarms
        FRSKY_setModelAlarms();
#endif
    }

#ifdef REVX
	if ( g_model.telemetryRxInvert )
	{
		setMFP() ;
	}
	else
	{
		clearMFP() ;
	}
#endif

}

bool ee32ModelExists(uint8_t id)
{
	return ( File_system[id+1].size > 0 ) ;
}

void ee32LoadModelName( uint8_t id, unsigned char*buf, uint8_t len )
{
	if(id<=MAX_MODELS)
  {
		memset(buf,' ',len);
		if ( File_system[id].size > sizeof(g_model.name) )
		{
			read32_eeprom_data( ( File_system[id].block_no << 12) + 8, ( uint8_t *)buf, sizeof(g_model.name), 0 ) ;
		}
  }
}


void fill_file_index()
{
	uint32_t i ;
	for ( i = 0 ; i < MAX_MODELS + 1 ; i += 1 )
	{
		File_system[i].block_no = get_current_block_number( i * 2, &File_system[i].size, &File_system[i].sequence_no ) ;
	}
}

void init_eeprom()
{
	fill_file_index() ;
	ee32_read_model_names() ;
	Eeprom32_process_state = E32_IDLE ;
}


// For virtual USB diskio
uint32_t ee32_read_512( uint32_t sector, uint8_t *buffer )
{
	// Wait for EEPROM to be idle
	if ( General_timer )
	{
		General_timer = 1 ;		// Make these happen soon
	}
	if ( Model_timer )
	{
		Model_timer = 1 ;
	}
	while( ee32_check_finished() == 0 )
	{
		// null body
	}
	read32_eeprom_data( sector * 512, buffer, 512, 0 ) ;
	return 1 ;		// OK
}


uint32_t ee32_check_finished()
{
	if ( ( Eeprom32_process_state != E32_IDLE )
			|| ( General_timer )
			|| ( Model_timer )
			|| ( Ee32_model_delete_pending) )
	{
		ee32_process() ;
		return 0 ;
	}
	return 1 ;
}


void ee32_process()
{
	register uint8_t *p ;
	register uint8_t *q ;
	register uint32_t x ;
	register uint32_t eeAddress ;

//	return 0 ;

	if ( General_timer )
	{
		if ( --General_timer == 0 )
		{
			
			// Time to write g_eeGeneral
			Ee32_general_write_pending = 1 ;
		}
	}
	if ( Model_timer )
	{
		if ( --Model_timer == 0 )
		{
			
			// Time to write model
			Ee32_model_write_pending = 1 ;
		}
	}

	if ( Eeprom32_process_state == E32_IDLE )
	{
		if ( Ee32_general_write_pending )
		{
			Ee32_general_write_pending = 0 ;			// clear flag

			// Check we can write, == block is blank

			Eeprom32_source_address = (uint8_t *)&g_eeGeneral ;		// Get data fromm here
			Eeprom32_data_size = sizeof(g_eeGeneral) ;						// This much
			Eeprom32_file_index = 0 ;								// This file system entry
			Eeprom32_process_state = E32_BLANKCHECK ;
//			Writing_model = 0 ;
		}
		else if ( Ee32_model_write_pending )
		{
			Ee32_model_write_pending = 0 ;			// clear flag

			// Check we can write, == block is blank

			Eeprom32_source_address = (uint8_t *)&g_model ;		// Get data from here
			Eeprom32_data_size = sizeof(g_model) ;						// This much
			Eeprom32_file_index = Model_dirty ;								// This file system entry
			Eeprom32_process_state = E32_BLANKCHECK ;
//			Writing_model = Model_dirty ;
		}
		else if ( Ee32_model_delete_pending )
		{
			Eeprom32_source_address = (uint8_t *)&g_model ;		// Get data from here
			Eeprom32_data_size = 0 ;													// This much
			Eeprom32_file_index = Ee32_model_delete_pending ;	// This file system entry
			Ee32_model_delete_pending = 0 ;
			Eeprom32_process_state = E32_BLANKCHECK ;
		}
	}

	if ( Eeprom32_process_state == E32_BLANKCHECK )
	{
		eeAddress = File_system[Eeprom32_file_index].block_no ^ 1 ;
		eeAddress <<= 12 ;		// Block start address
		Eeprom32_address = eeAddress ;						// Where to put new data
//		x = Eeprom32_data_size + sizeof( struct t_eeprom_header ) ;	// Size needing to be checked
//		p = (uint8_t *) &Eeprom_buffer ;
//		read32_eeprom_data( eeAddress, p, x, 1 ) ;
//		Eeprom32_process_state = E32_READSENDING ;
//	}

//	if ( Eeprom32_process_state == E32_READSENDING )
//	{
//		if ( Spi_complete )
//		{
//			uint32_t blank = 1 ;
//			x = Eeprom32_data_size + sizeof( struct t_eeprom_header ) ;	// Size needing to be checked
//			p = (uint8_t *) &Eeprom_buffer ;
//			while ( x )
//			{
//				if ( *p++ != 0xFF )
//				{
//					blank = 0 ;
//					break ;					
//				}
//				x -= 1 ;
//			}
//			// If not blank, sort erasing here
//			if ( blank )
//			{
//				Eeprom32_state_after_erase = E32_IDLE ;
//				Eeprom32_process_state = E32_WRITESTART ;
//			}
//			else
//			{
//				eeAddress = Eeprom32_address ;
				eeprom_write_enable() ;
				p = Spi_tx_buf ;
				*p = 0x20 ;		// Block Erase command
				*(p+1) = eeAddress >> 16 ;
				*(p+2) = eeAddress >> 8 ;
				*(p+3) = eeAddress ;		// 3 bytes address
				spi_PDC_action( p, 0, 0, 4, 0 ) ;
				Eeprom32_process_state = E32_ERASESENDING ;
				Eeprom32_state_after_erase = E32_WRITESTART ;
//			}
//		}
	}

	if ( Eeprom32_process_state == E32_WRITESTART )
	{
		uint32_t total_size ;
		p = Eeprom32_source_address ;
		q = (uint8_t *)&Eeprom_buffer.data ;
    if (p != q)
		{
			for ( x = 0 ; x < Eeprom32_data_size ; x += 1 )
			{
				*q++ = *p++ ;			// Copy the data to temp buffer
			}
		}
		Eeprom_buffer.header.sequence_no = ++File_system[Eeprom32_file_index].sequence_no ;
		File_system[Eeprom32_file_index].size = Eeprom_buffer.header.data_size = Eeprom32_data_size ;
		Eeprom_buffer.header.flags = 0 ;
		Eeprom_buffer.header.hcsum = byte_checksum( (uint8_t *)&Eeprom_buffer, 7 ) ;
		total_size = Eeprom32_data_size + sizeof( struct t_eeprom_header ) ;
		eeAddress = Eeprom32_address ;		// Block start address
		x = total_size / 256 ;	// # sub blocks
		x <<= 8 ;						// to offset address
		eeAddress += x ;		// Add it in
		p = (uint8_t *) &Eeprom_buffer ;
		p += x ;						// Add offset
		x = total_size % 256 ;	// Size of last bit
		if ( x == 0 )						// Last bit empty
		{
			x = 256 ;
			p -= x ;
			eeAddress -= x ;
		}
		Eeprom32_buffer_address = p ;
		Eeprom32_address = eeAddress ;
		write32_eeprom_block( eeAddress, p, x, 1 ) ;
		Eeprom32_process_state = E32_WRITESENDING ;
	}

	if ( Eeprom32_process_state == E32_WRITESENDING )
	{
		if ( Spi_complete )
		{
			Eeprom32_process_state = E32_WRITEWAITING ;
		}			
	}		

	if ( Eeprom32_process_state == E32_WRITEWAITING )
	{
		x = eeprom_read_status() ;
		if ( ( x & 1 ) == 0 )
		{
			if ( ( Eeprom32_address & 0x0FFF ) != 0 )		// More to write
			{
				Eeprom32_address -= 256 ;
				Eeprom32_buffer_address -= 256 ;
				write32_eeprom_block( Eeprom32_address, Eeprom32_buffer_address, 256, 1 ) ;
				Eeprom32_process_state = E32_WRITESENDING ;
			}
			else
			{
				File_system[Eeprom32_file_index].block_no ^= 1 ;		// This is now the current block
				// now erase the other block
//				eeAddress = Eeprom32_address ^ 0x00001000 ;		// Address of block to erase
//				eeprom_write_enable() ;
//				p = Spi_tx_buf ;
//				*p = 0x20 ;		// Block Erase command
//				*(p+1) = eeAddress >> 16 ;
//				*(p+2) = eeAddress >> 8 ;
//				*(p+3) = eeAddress ;		// 3 bytes address
//				spi_PDC_action( p, 0, 0, 4, 0 ) ;
//				Eeprom32_process_state = E32_ERASESENDING ;
//				Eeprom32_state_after_erase = E32_IDLE ;
				Eeprom32_process_state = E32_IDLE ;
			}
		}
	}	

	if ( Eeprom32_process_state == E32_ERASESENDING )
	{
		if ( Spi_complete )
		{
			Eeprom32_process_state = E32_ERASEWAITING ;
		}			
	}	
		
	if ( Eeprom32_process_state == E32_ERASEWAITING )
	{
		x = eeprom_read_status() ;
		if ( ( x & 1 ) == 0 )
		{ // Command finished
			Eeprom32_process_state = Eeprom32_state_after_erase ;
		}			
	}

//	if ( Eeprom32_process_state == E32_IDLE )
//	{
//		return 0 ;			// inactive
//	}
//	else
//	{
//		return 1 ;
//	}
}



uint32_t unprotect_eeprom()
{
 	register uint8_t *p ;

	eeprom_write_enable() ;
		
	p = Spi_tx_buf ;
	*p = 0x39 ;		// Unprotect sector command
	*(p+1) = 0 ;
	*(p+2) = 0 ;
	*(p+3) = 0 ;		// 3 bytes address

	return spi_operation( p, Spi_rx_buf, 4 ) ;
}

void convertSwitch( int8_t *p )
{
	if ( *p <= -MAX_DRSWITCH )
	{
		*p -= (NUM_SKYCSW - NUM_CSW) ;
	}
	if ( *p >= MAX_DRSWITCH )
	{
		*p += (NUM_SKYCSW - NUM_CSW) ;
	}
}


void convertModel( SKYModelData *dest, ModelData *source )
{
	uint32_t i ;
	memset( dest, 0, sizeof(*dest) ) ;
  memcpy( dest->name, source->name, MODEL_NAME_LEN) ;
	dest->modelVoice = source->modelVoice ;
	dest->RxNum = source->RxNum ;
	dest->traineron = source->traineron ;
	dest->FrSkyUsrProto = source->FrSkyUsrProto ;
	dest->FrSkyGpsAlt = source->FrSkyGpsAlt ;
	dest->FrSkyImperial = source->FrSkyImperial ;
	dest->FrSkyAltAlarm = source->FrSkyAltAlarm ;
	dest->modelVersion = source->version ;
	dest->protocol = source->protocol ;
	dest->ppmNCH = source->ppmNCH ;
	dest->thrTrim = source->thrTrim ;
	dest->xnumBlades = source->numBlades ;
	dest->thrExpo = source->thrExpo ;
	dest->trimInc = source->trimInc ;
	dest->ppmDelay = source->ppmDelay ;
	dest->trimSw = source->trimSw ;
	dest->beepANACenter = source->beepANACenter ;
	dest->pulsePol = source->pulsePol ;
	dest->extendedLimits = source->extendedLimits ;
	dest->swashInvertELE = source->swashInvertELE ;
	dest->swashInvertAIL = source->swashInvertAIL ;
	dest->swashInvertCOL = source->swashInvertCOL ;
	dest->swashType = source->swashType ;
	dest->swashCollectiveSource = source->swashCollectiveSource ;
	dest->swashRingValue = source->swashRingValue ;
	dest->ppmFrameLength = source->ppmFrameLength ;
	for ( i = 0 ; i < MAX_MIXERS ; i += 1 )
	{
		MixData *src = &source->mixData[i] ;
		SKYMixData *dst = &dest->mixData[i] ;
		dst->destCh = src->destCh ;
		dst->srcRaw = src->srcRaw ;
		if ( dst->srcRaw == NUM_XCHNRAW+1 )		// MIX_3POS
		{
			dst->srcRaw += NUM_SKYCHNOUT - NUM_CHNOUT ;
		}

		dst->weight = src->weight ;
		dst->swtch = src->swtch ;
		convertSwitch( &dst->swtch ) ;
		
		dst->curve = src->curve ;
		dst->delayUp = src->delayUp * 10 ;
		dst->delayDown = src->delayDown * 10 ;
		dst->speedUp = src->speedUp * 10 ;
		dst->speedDown = src->speedDown * 10 ;
		dst->carryTrim = src->carryTrim ;
		dst->mltpx = src->mltpx ;
		dst->mixWarn = src->mixWarn ;
		dst->enableFmTrim = src->enableFmTrim ;
		dst->sOffset = src->sOffset ;
	}
	for ( i = 0 ; i < NUM_CHNOUT ; i += 1 )
	{
		dest->limitData[i] = source->limitData[i] ;
	}
	for ( i = 0 ; i < 4 ; i += 1 )
	{
		dest->expoData[i] = source->expoData[i] ;
		convertSwitch( &dest->expoData[i].drSw1 ) ;
		convertSwitch( &dest->expoData[i].drSw2 ) ;
		dest->trim[i] = source->trim[i] ;
	}
  memcpy( dest->curves5, source->curves5, sizeof(source->curves5) ) ;
  memcpy( dest->curves9, source->curves9, sizeof(source->curves9) ) ;
	for ( i = 0 ; i < NUM_CSW ; i += 1 )
	{
		CSwData *src = &source->customSw[i] ;
		SKYCSwData *dst = &dest->customSw[i] ;
		dst->v1 = src->v1 ;
		dst->v2 = src->v2 ;
		dst->func = src->func ;
  	switch (dst->func)
		{
  		case (CS_AND) :
  		case (CS_OR) :
  		case (CS_XOR) :
				convertSwitch( &dst->v1 ) ;
				convertSwitch( &dst->v2 ) ;
      break;
  		case (CS_VPOS):
  		case (CS_VNEG):
  		case (CS_APOS):
  		case (CS_ANEG):
				if ( dst->v1-1 >= CHOUT_BASE+NUM_CHNOUT )
				{
					dst->v1 += NUM_SKYCHNOUT - NUM_CHNOUT ;
				}
				if ( dst->v2-1 >= CHOUT_BASE+NUM_CHNOUT )
				{
					dst->v2 += NUM_SKYCHNOUT - NUM_CHNOUT ;
				}
      break;
		}
		dst->andsw = src->andsw ;
	}
	dest->frSkyVoltThreshold = source->frSkyVoltThreshold ;
	dest->bt_telemetry = source->bt_telemetry ;
	dest->numVoice = source->numVoice ;
	if ( dest->numVoice )
	{
		dest->numVoice += NUM_SKYCHNOUT - NUM_CHNOUT ;
	}
	for ( i = 0 ; i < NUM_CHNOUT ; i += 1 )
	{
		SafetySwData *src = &source->safetySw[i] ;
		SKYSafetySwData *dst = &dest->safetySw[i] ;
		if ( i < (uint32_t)NUM_CHNOUT - dest->numVoice )
		{
			dst->opt.ss.swtch = src->opt.ss.swtch ;
			dst->opt.ss.mode = src->opt.ss.mode ;
			dst->opt.ss.val = src->opt.ss.val ;
			if ( dst->opt.ss.mode == 3 )
			{
				dst->opt.ss.mode = 0 ;
			}
			switch ( dst->opt.ss.mode )
			{
				case 0 :
				case 1 :
				case 2 :
					convertSwitch( &dst->opt.ss.swtch ) ;
				break ;
			}
		}
		else
		{
			dst->opt.vs.vswtch = src->opt.vs.vswtch ;
			dst->opt.vs.vmode = src->opt.vs.vmode ;
			dst->opt.vs.vval = src->opt.vs.vval ;
			convertSwitch( (int8_t *)&dst->opt.vs.vswtch ) ;
		}
	}

	for ( i = 0 ; i < 2 ; i += 1 )
	{
		FrSkyChannelData *src = &source->frsky.channels[i] ;
		SKYFrSkyChannelData *dst = &dest->frsky.channels[i] ;
		dst->ratio = src->ratio ;
		dst->alarms_value[0] = src->alarms_value[0] ;
		dst->alarms_value[1] = src->alarms_value[1] ;
		dst->alarms_level = src->alarms_level ;
		dst->alarms_greater = src->alarms_greater ;
		dst->type = src->type ;
		dst->gain = 1 ;
	}

	for ( i = 0 ; i < 2 ; i += 1 )
	{
		dest->timer[i] = source->timer[i] ;
  	if( dest->timer[i].tmrModeB>=(MAX_DRSWITCH))	 //momentary on-off
		{
			dest->timer[i].tmrModeB += (NUM_SKYCSW - NUM_CSW) ;
		}
	}

	dest->frskyAlarms = source->frskyAlarms ;

  memcpy( &dest->customDisplayIndex[0], &source->customDisplayIndex[0], 6 ) ;

}

void setModelFilename( uint8_t *filename, uint8_t modelIndex )
{
	uint8_t *bptr ;
	uint32_t i ;
	
	bptr = cpystr( filename, (uint8_t *)"/MODELS/" ) ;
  memcpy( bptr, Eeprom_buffer.data.model_data.name, sizeof(g_model.name)) ;
	bptr += sizeof(g_model.name) - 1 ;
	for ( i = 0 ; i < sizeof(g_model.name) ; i += 1 )
	{
		if ( *bptr && ( *bptr != ' ' ) )
		{
			break ;
		}
		else
		{
			bptr -= 1 ;
		}
	}
	bptr += 1 ;
	if ( i >= sizeof(g_model.name) )
	{
		*bptr++ = 'x' ;
	}
	cpystr( bptr, (uint8_t *)".eepm" ) ;		// ".eepm"
}

static const uint8_t base64digits[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/" ;

// write XML file
FRESULT writeXMLfile( FIL *archiveFile, uint8_t *data, uint32_t size, UINT *totalWritten )
{
  UINT written ;
  UINT total = 0 ;
	uint32_t i ;
	FRESULT result ;
  uint8_t bytes[104] ;

  result = f_write( archiveFile, (BYTE *)"<!DOCTYPE ERSKY9X_EEPROM_FILE>\n<ERSKY9X_EEPROM_FILE>\n <MODEL_DATA number=\0420\042>\n  <Version>", 89, &written ) ;
	total += written ;
// MDVERS (2)
  bytes[0] = MDSKYVERS + '0' ;
  bytes[1] = 0 ;
  result = f_write( archiveFile, bytes, 1, &written) ;
	total += written ;
  result = f_write( archiveFile, (BYTE *)"</Version>\n  <Name>", 19, &written) ;
	total += written ;
  result = f_write( archiveFile, (BYTE *)Eeprom_buffer.data.model_data.name, sizeof(g_model.name), &written) ;
	total += written ;
  result = f_write( archiveFile, (BYTE *)"</Name>\n  <Data><![CDATA[", 25, &written) ;
	total += written ;

// Send base 64 here
	i = 0 ;
	while ( size > 2 )
	{
		bytes[i++] = base64digits[data[0] >> 2];
 		bytes[i++] = base64digits[((data[0] << 4) & 0x30) | (data[1] >> 4)];
		bytes[i++] = base64digits[((data[1] << 2) & 0x3c) | (data[2] >> 6)];
		bytes[i++] = base64digits[data[2] & 0x3f];
		data += 3 ;
		size -= 3 ;
		if ( i >= 100 )
		{
		  result = f_write( archiveFile, bytes, 100, &written) ;
			total += written ;
			i = 0 ;
		}
	}

	uint8_t fragment ;
	if ( size )
	{
		bytes[i++] = base64digits[data[0] >> 2] ;
		fragment = (data[0] << 4) & 0x30 ;
		if ( --size )
		{
 			fragment |= data[1] >> 4 ;
			bytes[i++] = base64digits[fragment];
		}
		bytes[i++] = ( size == 2 ) ? base64digits[(data[1] << 2) & 0x3c] : '=' ;
		bytes[i++] = '=' ;
		result = f_write( archiveFile, bytes, i, &written) ;
		total += written ;
	}
  result = f_write( archiveFile, (BYTE *)"]]></Data>\n </MODEL_DATA>\n</ERSKY9X_EEPROM_FILE>\n", 49, &written) ;
	total += written ;
	*totalWritten = total ;
 return result ;
}

uint8_t xmlDecode( uint8_t value )
{
	if ( value >= 'A' )
	{
		if ( value <= 'Z' )
		{
			return value - 'A' ;
		}
		return value - ( 'a' - 26 ) ;		// 'a'-'z'		
	}
	else
	{
		if ( value >= '0' )
		{
			return value + 4 ;
		}
		if ( value == '+' )
		{
			return 62 ;
		}
		else if ( value == '/' )
		{
			return 63 ;
		}
		else
		{
			return 255 ;		// '='
		}
	}
}


int32_t readXMLfile( FIL *archiveFile, uint8_t *data, uint32_t size, UINT *totalRead )
{
  UINT nread ;
  UINT total = 0 ;
	uint32_t i ;
	uint32_t j ;
	FRESULT result ;
	uint8_t stream[4] ;
  uint8_t bytes[104] ;

	result = f_read( archiveFile,  bytes, 100, &nread ) ;
	total += nread ;
	if ( strncmp( (const char *)&bytes[10], "ERSKY9X_", 8 ) )
	{
		return -1 ;		// Invalid file
	}
	result = f_read( archiveFile,  bytes, 100, &nread ) ;
	total += nread ;
	// Search for "CDATA[" starting at offset 30
	for ( i = 30 ; i < 50 ; i += 1 )
	{
		if ( !strncmp( (const char *)&bytes[i], "CDATA[", 6 ) )
		{
			break ;
		}
	}
	if ( i >= 50 )
	{
		return -1 ;		// Invalid file
	}
	i += 6 ;	// Index of base64 data
	j = 0 ;
	while ( size )
	{
		if ( i >= nread )
		{
			result = f_read( archiveFile,  bytes, 100, &nread ) ;
			total += nread ;
			i = 0 ;
		}
		if ( i < nread )
		{
			stream[j++] = xmlDecode( bytes[i++] ) ;
		}
		else
		{
			break ;
		}
		if ( j >= 4 )
		{
		  *data++ = ( stream[0] << 2) | (stream[1] >> 4) ;
			size -= 1 ;
			if ( stream[2] != 255 )
			{
		  	*data++ = ((stream[1] << 4) & 0xf0) | (stream[2] >> 2);
				size -= 1 ;
			}
			if ( stream[3] != 255 )
			{
  			*data++ = ((stream[2] << 6) & 0xc0) | stream[3] ;
				size -= 1 ;
			}
			j = 0 ;
		}
	}
	*totalRead = total ;
	return result ; 
}


const char *ee32BackupModel( uint8_t modelIndex )
{
//	uint8_t filename[50] ;
  uint16_t size ;
	FRESULT result ;
  DIR archiveFolder ;
  FIL archiveFile ;
  UINT written ;
	uint8_t filename[50] ;


	// Check for SD avaliable


  size = File_system[modelIndex].size ;
	 
	while (ee32_check_finished() == 0)
	{	// wait
#ifndef SIMU
		if ( General_timer )
		{
			General_timer = 1 ;		// Make these happen soon
		}
		if ( Model_timer )
		{
			Model_timer = 1 ;
		}
		CoTickDelay(1) ;					// 2mS for now
#endif
	}

  read32_eeprom_data( (File_system[modelIndex].block_no << 12) + sizeof( struct t_eeprom_header), ( uint8_t *)&Eeprom_buffer.data.sky_model_data, size, 0 ) ;

	// Build filename
	setModelFilename( filename, modelIndex ) ;

//  strcpy(buf, STR_MODELS_PATH);
  result = f_opendir(&archiveFolder, "/MODELS") ;
  if (result != FR_OK)
	{
    if (result == FR_NO_PATH)
		{
			WatchdogTimeout = 200 ;		// 2 seconds
      result = f_mkdir("/MODELS") ;
    	if (result != FR_OK)
			{
      	return "SDCARD ERROR" ;
			}
		}
  }
	
  result = f_open( &archiveFile, (TCHAR *)filename, FA_OPEN_ALWAYS | FA_CREATE_ALWAYS | FA_WRITE) ;
  if (result != FR_OK)
	{
   	return "CREATE ERROR" ;
  }

//  result = f_write(&archiveFile, (uint8_t *)&Eeprom_buffer.data.sky_model_data, size, &written) ;
	result = writeXMLfile( &archiveFile, (uint8_t *)&Eeprom_buffer.data.sky_model_data, size, &written) ;
  
	f_close(&archiveFile) ;
  if (result != FR_OK ) //	|| written != size)
	{
    return "WRITE ERROR" ;
  }

  return "MODEL SAVED" ;
}

const char *ee32RestoreModel( uint8_t modelIndex, char *filename )
{
	uint8_t *bptr ;
  FIL archiveFile ;
  UINT nread ;
	FRESULT result ;
	int32_t answer ;
	uint8_t fname[50] ;
	
	bptr = cpystr( fname, (uint8_t *)"/MODELS/" ) ;
	cpystr( bptr, (uint8_t *)filename ) ;

	while (ee32_check_finished() == 0)
	{	// wait
#ifndef SIMU
		if ( General_timer )
		{
			General_timer = 1 ;		// Make these happen soon
		}
		if ( Model_timer )
		{
			Model_timer = 1 ;
		}
		CoTickDelay(1) ;					// 2mS for now
#endif
	}
  
	result = f_open( &archiveFile, (TCHAR *)filename, FA_READ) ;
  if (result != FR_OK)
	{
   	return "OPEN ERROR" ;
  }

	memset(( uint8_t *)&Eeprom_buffer.data.sky_model_data, 0, sizeof(g_model));
	
	answer = readXMLfile( &archiveFile,  ( uint8_t *)&Eeprom_buffer.data.sky_model_data, sizeof(Eeprom_buffer.data.sky_model_data), &nread ) ;
	
	if ( answer == -1 )
	{
		return "BAD FILE" ;
	}
	result = (FRESULT) answer ;
	if (result != FR_OK)
	{
		return "READ ERROR" ;
	}
	// Can we validate the data here?
  Eeprom32_source_address = (uint8_t *)&Eeprom_buffer.data.sky_model_data ;		// Get data from here
  Eeprom32_data_size = sizeof(g_model) ;																	// This much
  Eeprom32_file_index = modelIndex ;																							// This file system entry
  Eeprom32_process_state = E32_BLANKCHECK ;
  ee32WaitFinished() ;
	ee32_read_model_names() ;		// Update

  return "MODEL RESTORED" ;
}


