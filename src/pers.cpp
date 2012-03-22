/*
 * Author - Erez Raviv <erezraviv@gmail.com>
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
#include <string.h>
//#include <stdlib.h>
#include "AT91SAM3S2.h"
#include "ersky9x.h"
#include "templates.h"
#include "myeeprom.h"
#include "file.h"
#include "debug.h"

#ifdef FRSKY
#include "frsky.h"
#endif

//EFile theFile;  //used for any file operation
//EFile theFile2; //sometimes we need two files
//EFile theWriteFile; //separate write file

//#define FILE_TYP_GENERAL 1
//#define FILE_TYP_MODEL   2
/*
#define partCopy(sizeDst,sizeSrc)                         \
      pSrc -= (sizeSrc);                                  \
      pDst -= (sizeDst);                                  \
      memmove(pDst, pSrc, (sizeSrc));                     \
      memset (pDst+(sizeSrc), 0,  (sizeDst)-(sizeSrc));
#define fullCopy(size) partCopy(size,size)
	*/

void generalDefault()
{
  memset(&g_eeGeneral,0,sizeof(g_eeGeneral));
  g_eeGeneral.myVers   =  MDVERS;
  g_eeGeneral.currModel=  0;
  g_eeGeneral.contrast = 30;
  g_eeGeneral.vBatWarn = 65;
  g_eeGeneral.stickMode=  1;
	g_eeGeneral.disablePotScroll=  1;
	g_eeGeneral.bright = 50 ;
	g_eeGeneral.volume = 2 ;
  for (int i = 0; i < 7; ++i) {
    g_eeGeneral.calibMid[i]     = 0x400;
    g_eeGeneral.calibSpanNeg[i] = 0x300;
    g_eeGeneral.calibSpanPos[i] = 0x300;
  }
  strncpy_P(g_eeGeneral.ownerName,PSTR("ME        "), 10);
  int16_t sum=0;
  for(int i=0; i<12;i++) sum+=g_eeGeneral.calibMid[i];
  g_eeGeneral.chkSum = sum;
}

//bool eeLoadGeneral()
//{
//  theFile.openRd(FILE_GENERAL);
//  memset(&g_eeGeneral, 0, sizeof(EEGeneral));
//  uint8_t sz = theFile.readRlc((uint8_t*)&g_eeGeneral, sizeof(EEGeneral));

//  for(uint8_t i=0; i<sizeof(g_eeGeneral.ownerName);i++) // makes sure name is valid
//  {
//      uint8_t idx = char2idx(g_eeGeneral.ownerName[i]);
//      g_eeGeneral.ownerName[i] = idx2char(idx);
//  }

//  if(g_eeGeneral.myVers<MDVERS)
//      sysFlags |= sysFLAG_OLD_EEPROM; // if old EEPROM - Raise flag

//  g_eeGeneral.myVers   =  MDVERS; // update myvers

//  uint16_t sum=0;
//  if(sz>(sizeof(EEGeneral)-20)) for(uint8_t i=0; i<12;i++) sum+=g_eeGeneral.calibMid[i];
//  return g_eeGeneral.chkSum == sum;
//}

void modelDefault(uint8_t id)
{
  memset(&g_model, 0, sizeof(ModelData));
  strncpy_P(g_model.name,PSTR("MODEL     "), 10 );
  g_model.name[5]='0'+(id+1)/10;
  g_model.name[6]='0'+(id+1)%10;
//  g_model.mdVers = MDVERS;

  applyTemplate(0); //default 4 channel template
}

//void eeLoadModelName(uint8_t id,char*buf,uint8_t len)
//{
//  if(id<MAX_MODELS)
//  {
//    //eeprom_read_block(buf,(void*)modelEeOfs(id),sizeof(g_model.name));
//    theFile.openRd(FILE_MODEL(id));
//    memset(buf,' ',len);
//    if(theFile.readRlc((uint8_t*)buf,sizeof(g_model.name)) == sizeof(g_model.name) )
//    {
//      uint16_t sz=theFile.size();
//      buf+=len;
//      while(sz){ --buf; *buf='0'+sz%10; sz/=10;}
//    }
//  }
//}

//uint16_t eeFileSize(uint8_t id)
//{
//    theFile.openRd(FILE_MODEL(id));
//    return theFile.size();
//}

//bool eeModelExists(uint8_t id)
//{
//    return EFile::exists(FILE_MODEL(id));
//}

//void eeLoadModel(uint8_t id)
//{
//    if(id<MAX_MODELS)
//    {
//        theFile.openRd(FILE_MODEL(id));
//        memset(&g_model, 0, sizeof(ModelData));
//        uint16_t sz = theFile.readRlc((uint8_t*)&g_model, sizeof(g_model));

//        if(sz<256) // if not loaded a fair amount
//        {
//            modelDefault(id);
//        }

//        for(uint8_t i=0; i<sizeof(g_model.name);i++) // makes sure name is valid
//        {
//            uint8_t idx = char2idx(g_model.name[i]);
//            g_model.name[i] = idx2char(idx);
//        }

////        g_model.mdVers = MDVERS; //update mdvers

////        resetTimer2();

//#ifdef FRSKY
//  FrskyAlarmSendState |= 0x40 ;		// Get RSSI Alarms
//        FRSKY_setModelAlarms();
//#endif
//    }
//}

bool eeDuplicateModel(uint8_t id)
{
//  uint8_t i;
//  for( i=id+1; i<MAX_MODELS; i++)
//  {
//    if(! EFile::exists(FILE_MODEL(i))) break;
//  }
//  if(i==MAX_MODELS) return false; //no free space in directory left

//  theFile.openRd(FILE_MODEL(id));
//  theFile2.create(FILE_MODEL(i),FILE_TYP_MODEL,600);
//  uint8_t buf[15];
//  uint8_t l;
//  while((l=theFile.read(buf,15)))
//  {
//    theFile2.write(buf,l);
////    if(theFile.errno()==ERR_TMO)
////    {
////        //wait for 10ms and try again
////        uint16_t tgtime = get_tmr10ms() + 100;
////        while (!=tgtime);
////        theFile2.write(buf,l);
////    }
//    wdt_reset();
//  }
//  theFile2.closeTrunc();
//  //todo error handling
//  return true;
	return false ;
}


void eeReadAll()
{
//	txmit('a') ;
  if(!ee32LoadGeneral() )
//		 !EeFsOpen()  ||
//     EeFsck() < 0 ||
//  )
  {
//	txmit('b') ;
		
    alert((char const *)"Bad EEprom Data", true);
    g_eeGeneral.contrast = 25 ;
    message(PSTR("EEPROM Formatting"));
//    EeFsFormat();
   //alert(PSTR("format ok"));
    generalDefault();
    // alert(PSTR("default ok"));

//    uint16_t sz = theFile.writeRlc(FILE_GENERAL,FILE_TYP_GENERAL,(uint8_t*)&g_eeGeneral,sizeof(EEGeneral),200);
//    if(sz!=sizeof(EEGeneral)) alert(PSTR("genwrite error"));

    modelDefault(0);
    //alert(PSTR("modef ok"));
//    theFile.writeRlc(FILE_MODEL(0),FILE_TYP_MODEL,(uint8_t*)&g_model,sizeof(g_model),200);
    //alert(PSTR("modwrite ok"));
    STORE_GENERALVARS;
    STORE_MODELVARS;        
  }
	else
	{
  	ee32LoadModel(g_eeGeneral.currModel);
	}
}


static uint8_t  s_eeDirtyMsk;
static uint16_t s_eeDirtyTime10ms;
void eeDirty(uint8_t msk)
{
  if(!msk) return;
  s_eeDirtyMsk      |= msk;
  s_eeDirtyTime10ms  = get_tmr10ms() ;

	// New file system operations
	if ( msk & EE_GENERAL )
	{
		ee32StoreGeneral() ;
	}
	if ( msk & EE_MODEL )
	{
		ee32StoreModel( g_eeGeneral.currModel, msk & EE_TRIM ) ;
	}

}

//#define WRITE_DELAY_10MS 100

//uint8_t Ee_lock = 0 ;
//extern uint8_t heartbeat;


//void eeWaitComplete()
//{
//  while(s_eeDirtyMsk)
//  {
//		eeCheck(true) ;
//    if(heartbeat == 0x3)
//    {
//      wdt_reset();
//      heartbeat = 0;
//    }
//  }
//}


//void eeCheck(bool immediately)
//{
//  uint8_t msk  = s_eeDirtyMsk;
//  if(!msk) return;
//  if( !immediately && (( get_tmr10ms() - s_eeDirtyTime10ms) < WRITE_DELAY_10MS)) return;
//  if ( Ee_lock ) return ;
//  Ee_lock = EE_LOCK ;      	// Lock eeprom writing from recursion
//  if ( msk & EE_TRIM )
//  {
//    Ee_lock |= EE_TRIM_LOCK ;    // So the lower levels know what is happening
//  }
  
//  s_eeDirtyMsk = 0;

//  if(msk & EE_GENERAL){
//    if(theWriteFile.writeRlc(FILE_TMP, FILE_TYP_GENERAL, (uint8_t*)&g_eeGeneral,
//                        sizeof(EEGeneral),20) == sizeof(EEGeneral))
//    {
//      EFile::swap(FILE_GENERAL,FILE_TMP);
//    }else{
//      if(theWriteFile.errno()==ERR_TMO){
//        s_eeDirtyMsk |= EE_GENERAL; //try again
//        s_eeDirtyTime10ms = get_tmr10ms() - WRITE_DELAY_10MS;
//      }else{
//        alert(PSTR("EEPROM overflow"));
//      }
//    }
//    //first finish GENERAL, then MODEL !!avoid Toggle effect
//  }
//  else if(msk & EE_MODEL){
//    if(theWriteFile.writeRlc(FILE_TMP, FILE_TYP_MODEL, (uint8_t*)&g_model,
//                        sizeof(g_model),20) == sizeof(g_model))
//    {
//      EFile::swap(FILE_MODEL(g_eeGeneral.currModel),FILE_TMP);
//    }else{
//      if(theWriteFile.errno()==ERR_TMO){
//        s_eeDirtyMsk |= EE_MODEL; //try again
//        if ( msk & EE_TRIM )
//        {
//          s_eeDirtyMsk |= EE_TRIM; //try again
//        }
//        s_eeDirtyTime10ms = get_tmr10ms() - WRITE_DELAY_10MS;
//      }else{
//        if ( ( msk & EE_TRIM ) == 0 )		// Don't stop if trim adjust
//        {
//          alert(PSTR("EEPROM overflow(1)"));
//        }
//      }
//    }
//  }
//  Ee_lock = 0 ;				// UnLock eeprom writing


//}


