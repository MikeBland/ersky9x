

#define MAXFILES (1+MAX_MODELS+3)

// Block size = 4K
// Number of blocks = 128

// First two blocks used alternately to update directory.
// Write to first block, erase second block, then write to second block, erase first.
// If freeList = 0xFFFF, then block is erased.

// To write a new file, allocate a directorey entry and a block, update ram copy of directory,
//   Write data to block (assume 1 is suficient), write directory to 'other' block, erase first block
// To erase file, update ram copy of directory, write directory to 'other' block, erase first block
//   erase data block
// To update file, allocate new block, write data to new block, add old block to end of free list,
//   update ram copy of directory, write directory to 'other' block, erase first block, erase freed block.


#  define EESIZE   2048
#  define BS       16
#  define RESV     64  //reserv for eeprom header with directory (eeFs)

struct DirEnt
{
  uint16_t  startBlk;
  uint16_t size:12;
  uint16_t typ:4;
}__attribute__((packed)) ;

struct EeFs
{
  uint8_t  version ;
  uint8_t  mySize ;
  uint16_t  freeList ;
	uint16_t sequence ;
//  uint16_t  bs;
  DirEnt   files[MAXFILES] ;
	uint16_t Allocation[128] ;
	uint16_t checksum ;
}__attribute__((packed)) EeFs ;






