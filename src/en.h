/****************************************************************************
*  Copyright (c) 2013 by Michael Blandford. All rights reserved.
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
* Other Authors:
 * - Andre Bernet
 * - Bertrand Songis
 * - Bryan J. Rentoul (Gruvin)
 * - Cameron Weeks
 * - Erez Raviv
 * - Jean-Pierre Parisy
 * - Karl Szmutny
 * - Michal Hlavinka
 * - Pat Mackenzie
 * - Philip Moss
 * - Rob Thomson
 * - Romolo Manfredini
 * - Thomas Husterer
*
****************************************************************************/

#define FWx4		"\030"
#define FWx5		"\036"
#define FWx10		"\074"
#define FWx11		"\102"
#define FWx12		"\110"
#define FWx13		"\116"
#define FWx14		"\124"
#define FWx15		"\132"
#define FWx16		"\140"
#define FWx17		"\146"
#define FWx18		"\152"

#define I_REMOVED						0

#define ISTR_X_OFF_ON				FWx17"\001""\003"ISTR_OFF ISTR_ON

#define ISTR_ON             "ON "
#define ISTR_OFF            "OFF"

#define ISTR_ALTEQ	         "Alt=" 
#define ISTR_TXEQ			       "\003Tx=Swr"
#define ISTR_RXEQ		       "Rx="
#define ISTR_TRE012AG	     "TRE012AG"

// ISTR_YELORGRED indexed 3 char each
#define ISTR_YELORGRED	     "\003---YelOrgRed"
#define ISTR_A_EQ		       "A ="
#define ISTR_SOUNDS	       "\006Warn1 ""Warn2 ""Cheap ""Ring  ""SciFi ""Robot ""Chirp ""Tada  ""Crickt""Siren ""AlmClk""Ratata""Tick  ""Haptc1""Haptc2""Haptc3"
#define ISTR_SWITCH_WARN	   "Switch Warning"
// ISTR_TIMER exactly 5 chars long
#define ISTR_TIMER          "Timer"

// ISTR_PPMCHANNELS indexed 4 char each
#define ISTR_PPMCHANNELS	   "CH"

#define ISTR_MAH_ALARM      "mAh Alarm"


// er9x.cpp
// ********
#define ISTR_LIMITS		     "LIMITS"
#define ISTR_EE_LOW_MEM     "EEPROM low mem"
#define ISTR_ALERT		      " ALERT"
#define ISTR_THR_NOT_IDLE   "Throttle not idle"
#define ISTR_RST_THROTTLE   "Reset throttle"
#define ISTR_PRESS_KEY_SKIP "Press any key to skip"
#define ISTR_ALARMS_DISABLE "Alarms Disabled"
#define ISTR_OLD_VER_EEPROM " Old Version EEPROM   CHECK SETTINGS/CALIB"
#define ISTR_RESET_SWITCHES "Please Reset Switches"
#define ISTR_LOADING        "LOADING"
#define ISTR_MESSAGE        "MESSAGE"
#define ISTR_PRESS_ANY_KEY  "press any Key"
#define ISTR_MSTACK_UFLOW   "mStack uflow"
#define ISTR_MSTACK_OFLOW   "mStack oflow"

#ifdef PCBSKY
#define ISTR_CHANS_GV	     "\004P1  P2  P3  HALFFULLCYC1CYC2CYC3PPM1PPM2PPM3PPM4PPM5PPM6PPM7PPM8CH1 CH2 CH3 CH4 CH5 CH6 CH7 CH8 CH9 CH10CH11CH12CH13CH14CH15CH16CH17CH18CH19CH20CH21CH22CH23CH24SWCHGV1 GV2 GV3 GV4 GV5 GV6 GV7 THISSC1 SC2 SC3 SC4 SC5 SC6 SC7 SC8 "
#define ISTR_CHANS_RAW	   "\004P1  P2  P3  HALFFULLCYC1CYC2CYC3PPM1PPM2PPM3PPM4PPM5PPM6PPM7PPM8CH1 CH2 CH3 CH4 CH5 CH6 CH7 CH8 CH9 CH10CH11CH12CH13CH14CH15CH16CH17CH18CH19CH20CH21CH22CH23CH24SWCH"
#endif
#ifdef PCBX9D
#define ISTR_CHANS_GV	     "\004P1  P2  SL  HALFFULLCYC1CYC2CYC3PPM1PPM2PPM3PPM4PPM5PPM6PPM7PPM8CH1 CH2 CH3 CH4 CH5 CH6 CH7 CH8 CH9 CH10CH11CH12CH13CH14CH15CH16CH17CH18CH19CH20CH21CH22CH23CH24SWCHGV1 GV2 GV3 GV4 GV5 GV6 GV7 THISSC1 SC2 SC3 SC4 SC5 SC6 SC7 SC8 "
#define ISTR_CHANS_RAW	   "\004P1  P2  SL  HALFFULLCYC1CYC2CYC3PPM1PPM2PPM3PPM4PPM5PPM6PPM7PPM8CH1 CH2 CH3 CH4 CH5 CH6 CH7 CH8 CH9 CH10CH11CH12CH13CH14CH15CH16CH17CH18CH19CH20CH21CH22CH23CH24SWCH"
#define ISTR_CHANS_EXTRA   "\004SR  P3  P4  P5  P6  "
#endif

#define ISTR_CH	           "CH"
#define ISTR_TMR_MODE	     "\003OFFON RUsRU%ELsEL%THsTH%ALsAL%P1 P1%P2 P2%P3 P3%"

// pers.cpp
// ********
#define ISTR_ME             "ME        "
#define ISTR_MODEL          "MODEL     "
#define ISTR_BAD_EEPROM     "Bad EEprom Data"
#define ISTR_EE_FORMAT      "EEPROM Formatting"
#define ISTR_GENWR_ERROR    "genwrite error"
#define ISTR_EE_OFLOW       "EEPROM overflow"

// templates.cpp
// ***********
#define ISTR_T_S_4CHAN      "Simple 4-CH"
#define ISTR_T_TCUT         "T-Cut"
#define ISTR_T_STICK_TCUT   "Sticky T-Cut"
#define ISTR_T_V_TAIL       "V-Tail"
#define ISTR_T_ELEVON       "Elevon\\Delta"
#define ISTR_T_HELI_SETUP   "Heli Setup"
#define ISTR_T_GYRO         "Gyro Setup"
#define ISTR_T_SERVO_TEST   "Servo Test"
#define ISTR_T_RANGE_TEST   "Range Test"

// menus.cpp
// ***********
#ifdef PCBSKY
#define ISTR_TELEM_ITEMS	   "\004----A1= A2= RSSITSSITim1Tim2Alt GaltGspdT1= T2= RPM FUELMah1Mah2CvltBattAmpsMah CtotFasVAccXAccYAccZVspdGvr1Gvr2Gvr3Gvr4Gvr5Gvr6Gvr7FwatRxV Hdg A3= A4= SC1 SC2 SC3 SC4 SC5 SC6 SC7 SC8 RTC TmOK"
#endif
#ifdef PCBX9D
#define ISTR_TELEM_ITEMS	   "\004----A1= A2= RSSISWR Tim1Tim2Alt GaltGspdT1= T2= RPM FUELMah1Mah2CvltBattAmpsMah CtotFasVAccXAccYAccZVspdGvr1Gvr2Gvr3Gvr4Gvr5Gvr6Gvr7FwatRxV Hdg A3= A4= SC1 SC2 SC3 SC4 SC5 SC6 SC7 SC8 RTC TmOK"
#endif
#define ISTR_TELEM_SHORT    "\004----TIM1TIM2BATTGvr1Gvr2Gvr3Gvr4Gvr5Gvr6Gvr7"
#define ISTR_GV             "GV"
#define ISTR_OFF_ON         "OFFON "
#define ISTR_HYPH_INV       FWx18"\001""\003---INV"
#define ISTR_VERSION        "VERSION"
#define ISTR_TRAINER        "TRAINER"
#define ISTR_SLAVE          "\007Slave" 
#define ISTR_MENU_DONE      "[MENU] WHEN DONE"
#define ISTR_CURVES         "CURVES"
#define ISTR_CURVE          "CURVE"
#define ISTR_GLOBAL_VAR     "GLOBAL VAR"
#define ISTR_VALUE          "Value"
#define ISTR_PRESET         "PRESET"
#define ISTR_CV             "CV"
#define ISTR_LIMITS         "LIMITS"
#define ISTR_COPY_TRIM      "COPY TRIM [MENU]"
#define ISTR_TELEMETRY      "TELEMETRY"
#define ISTR_USR_PROTO      "UsrProto"
#define ISTR_FRHUB_WSHHI    "\005FrHubWSHhiDSMx Jeti MavlkArduP"
#define ISTR_MET_IMP        "\003MetImp"
#define ISTR_A_CHANNEL      "A  channel"
#define ISTR_ALRM           "alrm"
#define ISTR_TELEMETRY2     "TELEMETRY2"
#define ISTR_TX_RSSIALRM    "TxRSSIalrm"
#define ISTR_NUM_BLADES     "Num Blades"
#define ISTR_ALT_ALARM      "AltAlarm"
#define ISTR_OFF122400      "\003OFF122400"
#define ISTR_VOLT_THRES     "Volt Thres="
#define ISTR_GPS_ALTMAIN    "GpsAltMain"
#define ISTR_CUSTOM_DISP    "Custom Display"
#define ISTR_FAS_OFFSET     "FAS Offset"
#define ISTR_VARIO_SRC      "Vario: Source"
#define ISTR_VSPD_A2        "\004----vspdA2  SC1 SC2 SC3 SC4 SC5 SC6 SC7 SC8 "
#define ISTR_2SWITCH        "\001Switch"
#define ISTR_2SENSITIVITY   "\001Sensitivity"
#define ISTR_GLOBAL_VARS    "GLOBAL VARS"
#ifdef PCBSKY
#define ISTR_GV_SOURCE      "\003---RtmEtmTtmAtmRENRudEleThrAilP1 P2 P3 c1 c2 c3 c4 c5 c6 c7 c8 c9 c10c11c12c13c14c15c16c17c18c19c20c21c22c23c24SC1SC2SC3SC4SC5SC6SC7SC8O1 O2 O3 O4 O5 O6 O7 O8 O9 O10O11O12O13O14O15O16O17O18O19O20O21O22O23O24"
#define ISTR_EXTRA_SOURCE   ""
#endif
#ifdef PCBX9D
#define ISTR_GV_SOURCE      "\003---RtmEtmTtmAtmRENRudEleThrAilP1 P2 SL SR c1 c2 c3 c4 c5 c6 c7 c8 c9 c10c11c12c13c14c15c16c17c18c19c20c21c22c23c24SC1SC2SC3SC4SC5SC6SC7SC8O1 O2 O3 O4 O5 O6 O7 O8 O9 O10O11O12O13O14O15O16O17O18O19O20O21O22O23O24"
#define ISTR_EXTRA_SOURCE   "\003SR P3 P4 P5 P6 "
#endif
#define ISTR_TEMPLATES      "TEMPLATES"
#define ISTR_CHAN_ORDER     "Channel Order"
#define ISTR_SP_RETA        " RETA"
#define ISTR_CLEAR_MIXES    "CLEAR MIXES [MENU]"
#define ISTR_SAFETY_SW      "SAFETY SWITCHES"
#define ISTR_NUM_VOICE_SW   "Number Voice Sw"
#define ISTR_V_OPT1         "\007 8 Secs12 Secs16 Secs"
#define ISTR_VS             "VS"
#define ISTR_VOICE_OPT      "\006ON    OFF   BOTH  15Secs30Secs60SecsVaribl"
#define ISTR_CUST_SWITCH    "LOGICAL SWITCHES"
#define ISTR_S              "S"
#define ISTR_15_ON          "\015On"
#define ISTR_EDIT_MIX       "EDIT MIX "
#define ISTR_2SOURCE        "\001Source"
#define ISTR_2WEIGHT        "\001Weight"
#define ISTR_FMTRIMVAL      "FmTrimVal"
#define ISTR_OFFSET         "Offset"
#define ISTR_2FIX_OFFSET    "\001Fix Offset"
#define ISTR_ENABLEEXPO     "\001EnableExpoDr"
#define ISTR_2TRIM          "\001Trim"
#define ISTR_15DIFF         "\010Diff"
#define ISTR_Curve          "Curve"
#define ISTR_2WARNING       "\001Warning"
#define ISTR_2MULTIPLEX     "\001Multpx"
// ISTR_ADD_MULT_REP indexed 8 chars each
#define ISTR_ADD_MULT_REP   "\010Add     MultiplyReplace "
#define ISTR_2DELAY_DOWN    "\001Delay Down"
#define ISTR_2DELAY_UP      "\001Delay Up"
#define ISTR_2SLOW_DOWN     "\001Slow  Down"
#define ISTR_2SLOW_UP       "\001Slow  Up"
#define ISTR_MAX_MIXERS     "Max mixers reach: 32"
#define ISTR_PRESS_EXIT_AB  "Press [EXIT] to abort"
#define ISTR_YES_NO         "\003YES\013NO"
#define ISTR_MENU_EXIT      "\003[MENU]\013[EXIT]"
#define ISTR_DELETE_MIX     "DELETE MIX?"
#define ISTR_MIX_POPUP      "EDIT\0INSERT\0COPY\0MOVE\0DELETE\0CLEAR ALL"
#define ISTR_MIXER          "MIXER"
// CHR_S S for Slow
#define ICHR_S              "S"
// CHR_D D for Delay
#define ICHR_D              "D"
// CHR_d d for differential
#define ICHR_d              "d"
#define ISTR_EXPO_DR        "EXPO/DR"
#define ISTR_4DR_MID        "\004DR Mid"
#define ISTR_4DR_LOW        "\004DR Low"
#define ISTR_4DR_HI         "\004DR Hi"
#define ISTR_2EXPO          "\002Expo"
#define ISTR_DR_SW1         "DrSw1"
#define ISTR_DR_SW2         "DrSw2"
#define ISTR_DUP_MODEL      "DUPLICATE MODEL"
#define ISTR_DELETE_MODEL   "DELETE MODEL"
#define ISTR_DUPLICATING    "Duplicating model"
#define ISTR_SETUP          "Model Setup"
#define ISTR_NAME           "Name"
#define ISTR_VOICE_INDEX    "Voice\021MENU"
#define ISTR_TRIGGERA       "Trigger"
#define ISTR_TRIGGERB       "TriggerB"
//ISTR_COUNT_DOWN_UP indexed, 10 chars each
#define ISTR_COUNT_DOWN_UP  "\012Count DownCount Up  "
#define ISTR_T_TRIM         "T-Trim"
#define ISTR_T_EXPO         "T-Expo"
#define ISTR_TRIM_INC       "Trim Inc""\037"ISTR_TRIM_SWITCH
// ISTR_TRIM_OPTIONS indexed 6 chars each
#define ISTR_TRIM_OPTIONS   FWx14"\004""\006Exp   ExFineFine  MediumCoarse"
#define ISTR_TRIM_SWITCH    "Trim Sw"
#define ISTR_BEEP_CENTRE    "Beep Cnt"
#define ISTR_RETA123        "RETA123"
#define ISTR_PROTO          "Proto"
// ISTR_21_USEC after \021 max 4 chars
#define ISTR_21_USEC        "\021uSec"
#define ISTR_13_RXNUM       "\013RxNum"
// ISTR_23_US after \023 max 2 chars
#define ISTR_23_US          "\023uS"
// ISTR_PPMFRAME_MSEC before \015 max 9 chars, after max 4 chars
#define ISTR_PPMFRAME_MSEC  " PPM FrLen\015mSec"
#define ISTR_SEND_RX_NUM    " Send Rx Number [MENU]"
#define ISTR_DSM_TYPE       " DSM Type"
#define ISTR_PPM_1ST_CHAN   " 1st Chan"
#define ISTR_SHIFT_SEL      " Shift Sel"
// ISTR_POS_NEG indexed 3 chars each
#define ISTR_POS_NEG        "\003POSNEG"
#define ISTR_E_LIMITS       "E. Limits"
#define ISTR_Trainer        "Trainer"
#define ISTR_T2THTRIG       "T2ThTrig"
#define ISTR_AUTO_LIMITS    "Auto Limits"
// ISTR_1_RETA indexed 1 char each
#define ISTR_1_RETA         "\001RETA"
#define ISTR_FL_MODE        "FL MODE"
#define ISTR_SWITCH         "Switch"
#define ISTR_TRIMS          "Trims"
#define ISTR_MODES          "MODES"
#define ISTR_SP_FM0         " FM0"
#define ISTR_SP_FM          " FM"
#define ISTR_HELI_SETUP     "HELI SETUP"
#define ISTR_HELI_TEXT			ISTR_SWASH_TYPE"\037"ISTR_COLLECTIVE"\037"ISTR_SWASH_RING"\037"ISTR_ELE_DIRECTION"\037"ISTR_AIL_DIRECTION"\037"ISTR_COL_DIRECTION
#define ISTR_SWASH_TYPE     "Swash Type"
#define ISTR_COLLECTIVE     "Collective"
#define ISTR_SWASH_RING     "Swash Ring"
#define ISTR_ELE_DIRECTION  "ELE Direction"
#define ISTR_AIL_DIRECTION  "AIL Direction"
#define ISTR_COL_DIRECTION  "COL Direction"
//#define ISTR_MODEL_POPUP    "SELECT\0COPY\0MOVE\0DELETE"
#define ISTR_MODEL_POPUP    "EDIT\0SELECT\0COPY\0MOVE\0DELETE\0BACKUP\0RESTORE"
#define ISTR_MODELSEL       "MODELSEL"
// ISTR_11_FREE after \011 max 4 chars
#define ISTR_11_FREE        "\011free"
#define ISTR_CALIBRATION    "CALIBRATION"
// ISTR_MENU_TO_START after \003 max 15 chars
#define ISTR_MENU_TO_START  "\003[MENU] TO START"
// ISTR_SET_MIDPOINT after \005 max 11 chars
#define ISTR_SET_MIDPOINT   "\005SET MIDPOINT"
// ISTR_MOVE_STICKS after \003 max 15 chars
#define ISTR_MOVE_STICKS    "\003MOVE STICKS/POTS"
#define ISTR_ANA            "ANA"
#define ISTR_DIAG           "DIAG"
// ISTR_KEYNAMES indexed 5 chars each
#define ISTR_KEYNAMES       "\005 Menu Exit Down   UpRight Left"
#define ISTR_TRIM_M_P       "Trim- +"
// ISTR_OFF_PLUS_EQ indexed 3 chars each
#define ISTR_OFF_PLUS_EQ    "\003off += :="
// ISTR_CH1_4 indexed 3 chars each
#define ISTR_CH1_4          "\003ch1ch2ch3ch4"
#define ISTR_MULTIPLIER     "Multiplier"
#define ISTR_CAL            "Cal"
#define ISTR_MODE_SRC_SW    "\003mode\012% src  sw"
#define ISTR_RADIO_SETUP    "RADIO SETUP"
#define ISTR_OWNER_NAME     "Owner Name"
#define ISTR_BEEPER         "Beeper"
// ISTR_BEEP_MODES indexed 6 chars each
#define ISTR_BEEP_MODES     "\006Quiet ""NoKey ""xShort""Short ""Norm  ""Long  ""xLong "
#define ISTR_SOUND_MODE     "Sound Mode"
// ISTR_SPEAKER_OPTS indexed 10 chars each
#define ISTR_SPEAKER_OPTS   "\012Beeper    ""PiSpkr    ""BeeprVoice""PiSpkVoice""MegaSound "
#define ISTR_VOLUME         "Volume"
#define ISTR_SPEAKER_PITCH  " Speaker Pitch"
#define ISTR_HAPTICSTRENGTH " Haptic Strength"
#define ISTR_CONTRAST       "Contrast"
#define ISTR_BATT_WARN      "Battery warning" 
// ISTR_INACT_ALARM m for minutes after \023 - single char
#define ISTR_INACT_ALARM    "Inactivity alarm\023m"
#define ISTR_THR_REVERSE    "Throttle reverse"
#define ISTR_MINUTE_BEEP    "Minute beep"
#define ISTR_BEEP_COUNTDOWN "Beep countdown"
#define ISTR_FLASH_ON_BEEP  "Flash on beep"
#define ISTR_LIGHT_SWITCH   "Light switch"
#define ISTR_LIGHT_INVERT   "Backlight invert"
#define ISTR_LIGHT_AFTER    "Light on key"
#define ISTR_LIGHT_STICK    "Light on Stk Mv"
#define ISTR_SPLASH_SCREEN  "Splash screen"
#define ISTR_SPLASH_NAME    "Splash Name"
#define ISTR_THR_WARNING    "Throttle Warning"
#define ISTR_DEAFULT_SW     "Default Sw"
#define ISTR_MEM_WARN       "Memory Warning"
#define ISTR_ALARM_WARN     "Alarm Warning"
#define ISTR_POTSCROLL      "PotScroll"
#define ISTR_STICKSCROLL    "StickScroll"
#define ISTR_BANDGAP        "BandGap"
#define ISTR_ENABLE_PPMSIM  "Enable PPMSIM"
#define ISTR_CROSSTRIM      "CrossTrim"
#define ISTR_INT_FRSKY_ALRM "Int. Frsky alarm"
#define ISTR_MODE           "Mode"

// SWITCHES_STR 3 chars each
#ifdef PCBSKY
#define ISWITCHES_STR "\003THRRUDELEID0ID1ID2AILGEATRNL1 L2 L3 L4 L5 L6 L7 L8 L9 LA LB LC LD LE LF LG LH LI LJ LK LL LM LN LO onfTH\200TH-TH\201RU\200RU-RU\201EL\200EL-EL\201AI\200AI-AI\201GE\200GE-GE\2016P06P16P26P36P46P5"
#define IHW_SWITCHES_STR     "\002SASBSCSDSESFSGSH"
#define IHW_SWITCHARROW_STR  "\200-\201"
#endif
#ifdef PCBX9D
#ifdef REV9E
#define ISWITCHES_STR				 "\003SF       SC\200SC-SC\201      SH L1 L2 L3 L4 L5 L6 L7 L8 L9 LA LB LC LD LE LF LG LH LI LJ LK LL LM LN LO onfSB\200SB-SB\201SE\200SE-SE\201SA\200SA-SA\201SD\200SD-SD\201SG\200SG-SG\2016P06P16P26P36P46P5"\
														 "SI\200SI-SI\201SJ\200SJ-SJ\201SK\200SK-SK\201SL\200SL-SL\201SM\200SM-SM\201SN\200SN-SN\201SO\200SO-SO\201SP\200SP-SP\201SQ\200SQ-SQ\201SR\200SR-SR\201"
#else
#define ISWITCHES_STR "\003SF       SC\200SC-SC\201      SH L1 L2 L3 L4 L5 L6 L7 L8 L9 LA LB LC LD LE LF LG LH LI LJ LK LL LM LN LO onfSB\200SB-SB\201SE\200SE-SE\201SA\200SA-SA\201SD\200SD-SD\201SG\200SG-SG\2016P06P16P26P36P46P5"
#endif	// REV9E

#ifdef REV9E
#define IHW_SWITCHES_STR     "\002SASBSCSDSESFSGSHSISJSKSLSMSNSOSPSQSR"
#else
#define IHW_SWITCHES_STR     "\002SASBSCSDSESFSGSH"
#endif	// REV9E
#define IHW_SWITCHARROW_STR "\200-\201"
#endif
#define ISWITCH_WARN_STR	   "Switch Warning"
// CURV_STR indexed 3 chars each
// c17-c24 added for timer mode A display
#define ICURV_STR					 "\003---x>0x<0|x|f>0f<0|f|c1 c2 c3 c4 c5 c6 c7 c8 c9 c10c11c12c13c14c15c16c17c18c19c20c21c22c23c24"
// CSWITCH_STR indexed 7 chars each
#define ICSWITCH_STR        "\007----   v>val  v<val  |v|>val|v|<valAND    OR     XOR    ""v1==v2 ""v1!=v2 ""v1>v2  ""v1<v2  ""Latch  F-Flop TimeOffNtmeOff1-Shot 1-ShotRv\140=val"

#define ISWASH_TYPE_STR     FWx17"\004""\004----""120 ""120X""140 ""90  "

#define ISTR_STICK_NAMES    "\004Rud Ele Thr Ail "

#define ISTR_STAT           "STAT"
#define ISTR_STAT2          "STAT2"
// ISTR_TRIM_OPTS indexed 3 chars each
#define ISTR_TRIM_OPTS      "\003ExpExFFneMedCrs"
#define ISTR_TTM            "TTm"
#define ISTR_FUEL           "Fuel"
#define ISTR_12_RPM         "\012RPM"
#define ISTR_LAT_EQ         "Lat=""\037"ISTR_LON_EQ"\037"ISTR_ALT_MAX"\037"ISTR_SPD_KTS_MAX
#define ISTR_LON_EQ         "Lon="
#define ISTR_ALT_MAX        "Alt=\011m   Max="
#define ISTR_SPD_KTS_MAX    "Spd=\011kts Max="
#define ISTR_11_MPH         "\011mph"

#define ISTR_SINK_TONES	   "Sink Tones"


// ersky9x strings
#define ISTR_ST_CARD_STAT   "SD CARD STAT"
#define ISTR_4_READY        "\004Ready"
#define ISTR_NOT            "NOT"
#define ISTR_BOOT_REASON    "BOOT REASON"
#define ISTR_6_WATCHDOG     "\006WATCHDOG"
#define ISTR_5_UNEXPECTED   "\005UNEXPECTED"
#define ISTR_6_SHUTDOWN     "\006SHUTDOWN"
#define ISTR_6_POWER_ON     "\006POWER ON"
// ISTR_MONTHS indexed 3 chars each
#define ISTR_MONTHS         "\003XxxJanFebMarAprMayJunJulAugSepOctNovDec"
#define ISTR_MENU_REFRESH   "[MENU] to refresh"
#define ISTR_DATE_TIME      "DATE-TIME"
#define ISTR_SEC            "Sec."
#define ISTR_MIN_SET        "Min.\015Set"
#define ISTR_HOUR_MENU_LONG "Hour\012MENU LONG"
#define ISTR_DATE           "Date"
#define ISTR_MONTH          "Month"
#define ISTR_YEAR_TEMP      "Year\013Temp."
#define ISTR_YEAR           "Year"
#define ISTR_BATTERY        "BATTERY"
#define ISTR_Battery        "Battery"
#define ISTR_CURRENT_MAX    "Current\016Max"
#define ISTR_CPU_TEMP_MAX   "CPU temp.\014C Max\024C"
#define ISTR_MEMORY_STAT    "MEMORY STAT"
#define ISTR_GENERAL        "General"
#define ISTR_Model          "Model"
#define ISTR_RADIO_SETUP2   "RADIO SETUP2"
#define ISTR_BRIGHTNESS     "Brightness"
#define ISTR_CAPACITY_ALARM "Capacity Alarm"
#define ISTR_BT_BAUDRATE    "Bt baudrate"
#define ISTR_ROTARY_DIVISOR "Rotary Divisor"
#define ISTR_STICK_LV_GAIN  "Stick LV Gain"
#define ISTR_STICK_LH_GAIN  "Stick LH Gain"
#define ISTR_STICK_RV_GAIN  "Stick RV Gain"
#define ISTR_STICK_RH_GAIN  "Stick RH Gain"
#define ISTR_BIND					  " Bind"
#define ISTR_RANGE					" Range Check"

#define ISTR_ALRMS_OFF			"Alarms Disabled"
#define ISTR_OLD_EEPROM			" Old Version EEPROM   CHECK SETTINGS/CALIB"
#define ISTR_TRIGA_OPTS			"OFFON THsTH%"
#define ISTR_CHK_MIX_SRC		"CHECK MIX SOURCES"

#define ISTR_BT_TELEMETRY		"BT Telemetry"
#define ISTR_FRSKY_COM_PORT "FrSky Com Port"
#define ISTR_INVERT_COM1		"Invert COM 1"
#define ISTR_LOG_SWITCH			"Log Switch"
#define ISTR_LOG_RATE				"Log Rate"
#define ISTR_6_BINDING			"\006BINDING"
#define ISTR_RANGE_RSSI			"RANGE CHECK RSSI:"
#define ISTR_FAILSAFE				"FAILSAFE"
#define ISTR_VOLUME_CTRL		"Volume Control"
#define ISTR_PROT_OPT				"\006PPM   PXX   DSM2  "
#define ISTR_TYPE						" Type"
#define ISTR_COUNTRY				" Country"
#define ISTR_SP_FAILSAFE		" Failsafe"
#define ISTR_PPM2_START			"PPM2 StartChan"
#define ISTR_FOLLOW					"Follow"
#define ISTR_PPM2_CHANNELS	"PPM2 Channels"
#define ISTR_FILTER_ADC			"Filter ADC"
#define ISTR_SCROLLING			"Scrolling"
#define ISTR_ALERT_YEL			"Alert [Yel]"
#define ISTR_ALERT_ORG			"Alert [Org]"
#define ISTR_ALERT_RED			"Alert [Red]"
#define ISTR_LANGUAGE				"Language"

#define ISTR_RSSI_WARN		  "RSSI Warn"
#define ISTR_RSSI_CRITICAL  "RSSI Critical"
#define ISTR_RX_VOLTAGE		  "Rx Voltage"
#define ISTR_DSM_WARNING	  "DSM Warning"
#define ISTR_FADESLOSSHOLDS "\006fades lossesholds "
#define ISTR_DSM_CRITICAL	  "DSM Critical"
#define ISTR_BT_TRAINER		  "BT as Trainer"

//"Current Source"
//"\004----A1  A2  FASVSC1 SC2 SC3 SC4 SC5 SC6 SC7 SC8 "
//"SC  ="
//"Source"
//"Multiplier"
//"Divisor"
//"Unit"
//"Sign"
//"Decimals"
//"Offset At"
//"\005FirstLast "
//"Voice Switch"
//"Function"
//"\007----   v>val  v<val  |v|>val|v|<valON     OFF    BOTH   "
//"Switch"
//"Rate"
//"\017Once"
//"Offset"
//"FileType"
//"\006  NameNumberHaptic"
//"Voice File"
//"\006Haptc1Haptc2Haptc3"
// SKY "\003IDxTHRRUDELEAILGEATRN"
// X9D "\002SASBSCSDSESFSGSH"
//"\002MODES"
// SKY "\004sIDxsTHRsRUDsELEsAILsGEAsTRN"
// X9D "\002SASBSCSDSESFSGSH"
//"Reset Switch"
// SKY "\003---P1 P2 P3 GV4GV5GV6GV7"
// X9D "\003---P1 P2 SL SR GV5GV6GV7"
//"Internal"
//"\003AmeJapEur"
//"External"
//"Fade In"
//"Fade Out"
//"Name"
//"Co Proc"
//"On Time"
//"ttimer1        us"
//"\013rssi"
//"Vbat"
//"\013RxV"
//"AMP\013Temp"
//"RPM\021DSM2"
//"SETTINGS"
//"Display"
//"AudioHaptic"
//"Alarms"
//"General"
//"Controls"
//"Hardware"
//"Calibration"
//"Trainer"
//"Version"
//"Date-Time"
//"DiagSwtch"
//"DiagAna"
//"DISPLAY"
//"BLUE"
//"WHITE"
//"Optrex Display"
//"AUDIO/HAPTIC"
//"ALARMS"
//"[Next]"
//"GENERAL"
//"\012   ENGLISH  FRANCAIS   DEUTSCH NORWEGIAN   SWEDISH"
//"\005NONE POT  STICKBOTH "
//"CONTROLS"
//"HARDWARE"
//"ELE  switch"
//"\0042POS3POS6POS"
//"THR  switch"
//"\0042POS3POS"
//"RUD  switch"
//"GEAR switch"
//"MODEL SETTINGS"
//"Mixer"
//"C.Switches"
//"Telemetry"
//"Limits"
//"Display"
//"MDISPLAY"





