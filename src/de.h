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
 * - Fabian Schurig <fabian.schurig.94@gmail.com>
 * - Max Mäusezahl
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

/**
MY CHANGELOG

v.xx.01-german
- replaced special characters
- added new Strings after ISTR_BIND (line )
- added new Strings in if-case ISTR_TELEM_ITEMS and ISTR_FRHUB_WSHHI
- some minor changes in translation
**/

// Special characters:
// Ä für AE benutze \300
// ä für ae benutze \301
// Ö für OE benutze \302
// ö für oe benutze \303
// Ü für UE benutze \304
// ü für ue benutze \305
// ß für ss benutze \306


#define ISTR_ON             "AN "
#define ISTR_OFF            "AUS"

#define ISTR_ALTEQ	         "H\303h=" 
#define ISTR_TXEQ		       "Sn=" // TX Transmitter - Sender
#define ISTR_RXEQ		       "Em=" // RX Reciever - Empfänger
#define ISTR_RX  		       "Em"
#define ISTR_TRE012AG	     "TRE012AG"

// ISTR_YELORGRED je genau 3 Zeichen lang
#define ISTR_YELORGRED	     "\003---GelOrgRot"
#define ISTR_A_EQ		       "A ="
#define ISTR_SOUNDS	       "\006Warn1 ""Warn2 ""Cheap ""Ring  ""SciFi ""Robot ""Chirp ""Tada  ""Crickt""Siren ""AlmClk""Ratata""Tick  ""Haptc1""Haptc2""Haptc3"
#define ISTR_SWITCH_WARN	   "Schalter Warnung" 
// ISTR_TIMER genau 5 Zeichen lang 
#define ISTR_TIMER          "Timer"			

// ISTR_PPMCHANNELS je genau 4 Zeichen lang
#define ISTR_PPMCHANNELS	   "CH"

#define ISTR_MAH_ALARM      "mAh Alarm"


// er9x.cpp
// ********
#define ISTR_LIMITS		     "GRENZEN"
#define ISTR_EE_LOW_MEM     "EEPROM wenig Speicher"
#define ISTR_ALERT		       "ALARM"
#define ISTR_THR_NOT_IDLE   "Gas nicht im Ruhezstd"
#define ISTR_RST_THROTTLE   "setze auf Standgas"
#define ISTR_PRESS_KEY_SKIP "bel. Taste dr\305cken"
#define ISTR_ALARMS_DISABLE "Alarm ist deaktiviert"
#define ISTR_OLD_VER_EEPROM " EEPROM ist veraltet   TESTE EINSTELL/KALIB"
#define ISTR_RESET_SWITCHES "Schalter ausschalten"
#define ISTR_LOADING        "L\300DT"
#define ISTR_MESSAGE        "NACHRICHT"
#define ISTR_PRESS_ANY_KEY  "Taste dr\305cken"
#define ISTR_MSTACK_UFLOW   "mStack uflow"
#define ISTR_MSTACK_OFLOW   "mStack oflow"

#ifdef PCBSKY
#define ISTR_CHANS_GV	     "\004P1  P2  P3  HALBVOLLCYC1CYC2CYC3PPM1PPM2PPM3PPM4PPM5PPM6PPM7PPM8CH1 CH2 CH3 CH4 CH5 CH6 CH7 CH8 CH9 CH10CH11CH12CH13CH14CH15CH16CH17CH18CH19CH20CH21CH22CH23CH243POSGV1 GV2 GV3 GV4 GV5 GV6 GV7 THIS"
#define ISTR_CHANS_RAW	   "\004P1  P2  P3  HALBVOLLCYC1CYC2CYC3PPM1PPM2PPM3PPM4PPM5PPM6PPM7PPM8CH1 CH2 CH3 CH4 CH5 CH6 CH7 CH8 CH9 CH10CH11CH12CH13CH14CH15CH16CH17CH18CH19CH20CH21CH22CH23CH243POS"
#endif
#ifdef PCBX9D
#define ISTR_CHANS_GV	     "\004P1  P2  SL  SR  HALBVOLLCYC1CYC2CYC3PPM1PPM2PPM3PPM4PPM5PPM6PPM7PPM8CH1 CH2 CH3 CH4 CH5 CH6 CH7 CH8 CH9 CH10CH11CH12CH13CH14CH15CH16CH17CH18CH19CH20CH21CH22CH23CH243POSGV1 GV2 GV3 GV4 GV5 GV6 GV7 THIS"
#define ISTR_CHANS_RAW	   "\004P1  P2  SL  SR  HALBVOLLCYC1CYC2CYC3PPM1PPM2PPM3PPM4PPM5PPM6PPM7PPM8CH1 CH2 CH3 CH4 CH5 CH6 CH7 CH8 CH9 CH10CH11CH12CH13CH14CH15CH16CH17CH18CH19CH20CH21CH22CH23CH243POS"
#endif
#define ISTR_CH	           "CH"
#define ISTR_TMR_MODE	     "\003AUSAN Se0Se%Ho0Ho%Ga0Ga%Qu0Qu%P1 P1%P2 P2%P3 P3%" // OFF=AUS=Timer aus ; ABS=AN=Timer an ; RUs=Se0=Seite bei 0 ; Ru%=Se%=Seite bei x% usw.

// pers.cpp
// ********
#define ISTR_ME             "ICH       "
#define ISTR_MODEL          "MODELL    "
#define ISTR_BAD_EEPROM     "falsche EEprom Daten"
#define ISTR_EE_FORMAT      "formatiere EEPROM"
#define ISTR_GENWR_ERROR    "Schreibfehler"
#define ISTR_EE_OFLOW       "EEPROM oflow"

// templates.cpp
// ***********
#define ISTR_T_S_4CHAN      "Einfache 4-CH"
#define ISTR_T_TCUT         "Gas aus"
#define ISTR_T_STICK_TCUT   "dauer Gas aus"
#define ISTR_T_V_TAIL       "V-Leitw"
#define ISTR_T_ELEVON       "Delta\\Nurfl\305gler"
#define ISTR_T_HELI_SETUP   "Heli Einst"
#define ISTR_T_GYRO         "Gyro Einst"
#define ISTR_T_SERVO_TEST   "Servo Test"

// menus.cpp
// ***********
#ifdef PCBSKY
#define ISTR_TELEM_ITEMS	   "\004----A1= A2= RSSITSSITim1Tim2H\302heGH\302hGGesT1= T2= UPM TANKMah1Mah2CvltAkkuAmpsMah CtotFasVAccXAccYAccZVspdGvr1Gvr2Gvr3Gvr4Gvr5Gvr6Gvr7FwatRxV Hdg "
#endif
#ifdef PCBX9D
#define ISTR_TELEM_ITEMS	   "\004----A1= A2= RSSISWR Tim1Tim2H\303heGH\303hGGesT1= T2= UPM TANKMah1Mah2CvltAkkuAmpsMah CtotFasVBesXBesYBesZVGesGvr1Gvr2Gvr3Gvr4Gvr5Gvr6Gvr7FwatRxV Hdg "
#endif
#define ISTR_TELEM_SHORT    "\004----TIM1TIM2AKKUGvr1Gvr2Gvr3Gvr4Gvr5Gvr6Gvr7"
#define ISTR_GV             "GV"
#define ISTR_OFF_ON         "AUSAN "
#define ISTR_HYPH_INV       "\003---UMK" // Umkehren
#define ISTR_VERSION        "VERSION"
#define ISTR_TRAINER        "TRAINER"
#define ISTR_SLAVE          "\007Slave" 
#define ISTR_MENU_DONE      "[MENU] WENN FERTIG"
#define ISTR_CURVES         "KURVEN"
#define ISTR_CURVE          "KURVE"
#define ISTR_GLOBAL_VAR     "GLOBALE VAR"
#define ISTR_VALUE          "Wert"
#define ISTR_PRESET         "VOREINST"
#define ISTR_CV             "KV"
#define ISTR_LIMITS         "GRENZEN"
#define ISTR_COPY_TRIM      "KOPIE TRIM [MENU]"
#define ISTR_TELEMETRY      "TELEMETRIE"
#define ISTR_USR_PROTO      "BenProto"
#ifdef REVX
#define ISTR_FRHUB_WSHHI    "\005FrHubWSHhiDSMx "
#else
#define ISTR_FRHUB_WSHHI    "\005FrHubWSHhi"
#endif
#define ISTR_MET_IMP        "\003MetImp" // Metrisches System / Imperiales System
#define ISTR_A_CHANNEL      "A  Kanal"
#define ISTR_ALRM           "alrm"
#define ISTR_TELEMETRY2     "TELEMETRIE2"
#define ISTR_TX_RSSIALRM    "SnRSSIalrm" // Sender
#define ISTR_NUM_BLADES     "Num Bl\301tter"
#define ISTR_ALT_ALARM      "H\303heAlarm"
#define ISTR_OFF122400      "\003AUS122400"
#define ISTR_VOLT_THRES     "MaxSpannung"
#define ISTR_GPS_ALTMAIN    "GPS H\303heHalten"
#define ISTR_CUSTOM_DISP    "Ind. Bildschirm"
#define ISTR_FAS_OFFSET     "FAS Offset" // FrSky Amperage Sensor (FAS-100) Offset
#define ISTR_VARIO_SRC      "Vario: Quelle" // Variometerquelle
#define ISTR_VSPD_A2        "\004----VGesA2  " // VGeschwindigkeit
#define ISTR_2SWITCH        "\002Schalter"
#define ISTR_2SENSITIVITY   "\002Empfindlichkt"
#define ISTR_GLOBAL_VARS    "GLOBALE VARS"
#define ISTR_GV_SOURCE      "\003---StmHtmGtmQtmRENSEIH\302HGASQUEP1 P2 P3 c1 c2 c3 c4 c5 c6 c7 c8 c9 c10c11c12c13c14c15c16c17c18c19c20c21c22c23c24" // xtm=Trim for channel "x" REN=Rotary Encoder  ... = Variablennamen
#define ISTR_TEMPLATES      "VORLAGEN"
#define ISTR_CHAN_ORDER     "Kanal Reihenfolge"
#define ISTR_SP_RETA        " SHGQ" // Seitenleitwerk=Rud Höhenleitwerk=Ele Gas=Thr Querruder=Ail
#define ISTR_CLEAR_MIXES    "L\302SCHE MISCHER [MENU]"
#define ISTR_SAFETY_SW      "SICHERHEITS SCH"
#define ISTR_NUM_VOICE_SW   "Nummer Ton Sch"
#define ISTR_V_OPT1         "\007 8 Sek 12 Sek 16 Sek "
#define ISTR_VS             "VS" // ?
#define ISTR_VOICE_OPT      "\006AN    AUS   BEIDE 15Sek 30Sek 60Sek Eigene"
#define ISTR_CUST_SWITCH    "IND. SCHALTER" // Individueller Schalter
#define ISTR_S              "S"
#define ISTR_15_ON          "\015An"
#define ISTR_EDIT_MIX       "Bearb MISCHER " // Bearbeite Mischer
#define ISTR_2SOURCE        "\002Quelle"
#define ISTR_2WEIGHT        "\002Gewicht"
#define ISTR_FMTRIMVAL      "FmTrimVal"
#define ISTR_OFFSET         "Offset"
#define ISTR_2FIX_OFFSET    "\002Fix Offset"
#define ISTR_FLMODETRIM     "\002FlModustrim"
#define ISTR_2TRIM          "\002Trimmen"
#define ISTR_15DIFF         "\015Diff"
#define ISTR_Curve          "Kurve"
#define ISTR_2WARNING       "\002Warnung"
#define ISTR_2MULTIPLEX     "\002Multpx"
// ISTR_ADD_MULT_REP je genau 8 Zeichen
#define ISTR_ADD_MULT_REP   "\010Hinzufgn  MultipliziErsetzen  "
#define ISTR_2DELAY_DOWN    "\002Verz. runter"
#define ISTR_2DELAY_UP      "\002Verz. hoch"
#define ISTR_2SLOW_DOWN     "\002Langsam runtr"
#define ISTR_2SLOW_UP       "\002Langsam hoch"
#define ISTR_MAX_MIXERS     "Max Mix erreicht: 32"
#define ISTR_PRESS_EXIT_AB  "[EXIT] zum Abbrechen"
#define ISTR_YES_NO         "\003JA \013NEIN"
#define ISTR_MENU_EXIT      "\003[MENU]\013[EXIT]"
#define ISTR_DELETE_MIX     "L\302SCHE MISCHER?"
#define ISTR_MIX_POPUP      "BEARBEI\0EINF\305GE\0KOPIER\0BEWEGE\0L\302SCHE"
#define ISTR_MIXER          "MISCHER"
// CHR_S S for Slow / Langsam
#define ICHR_S              "L"
// CHR_D D for Delay / VerzÃ¶gert
#define ICHR_D              "V"
// CHR_d d for differential
#define ICHR_d              "d"
#define ISTR_EXPO_DR        "EXPO/DR"
#define ISTR_4DR_MID        "\004DR Mittel"
#define ISTR_4DR_LOW        "\004DR Tief"
#define ISTR_4DR_HI         "\004DR Hoch"
#define ISTR_2EXPO          "\002Expo"
#define ISTR_DR_SW1         "DrSw1"
#define ISTR_DR_SW2         "DrSw2"
#define ISTR_DUP_MODEL      "KOPIERE MODELL"
#define ISTR_DELETE_MODEL   "L\302SCHE MODELL"
#define ISTR_DUPLICATING    "Kopiere Modell"
#define ISTR_SETUP          "EINST"
#define ISTR_NAME           "Name"
#define ISTR_VOICE_INDEX    "Ton Freq\021MENU"
#define ISTR_TRIGGERA       "Trigger"
#define ISTR_TRIGGERB       "TriggerB"
//ISTR_COUNT_DOWN_UP indexed, 10 chars each
#define ISTR_COUNT_DOWN_UP  "\012Z\301hl runteZ\301hl hoch"
#define ISTR_T_TRIM         "T-Trim"
#define ISTR_T_EXPO         "T-Expo"
#define ISTR_TRIM_INC       "Trim Ink"
// ISTR_TRIM_OPTIONS indexed 6 chars each
#define ISTR_TRIM_OPTIONS   "\006Expon ExFeinFein  MittelGrob  "
#define ISTR_TRIM_SWITCH    "Trim Sch"
#define ISTR_BEEP_CENTRE    "Piep Frq" //TonhÃ¶he Frequenz
#define ISTR_RETA123        "SHGQ123"
#define ISTR_PROTO          "Proto" // Protokoll
// ISTR_21_USEC after \021 max 4 chars
#define ISTR_21_USEC        "\021uSek" 
#define ISTR_13_RXNUM       "\013EmNum" //EmpfÃ¤nger
// ISTR_23_US after \023 max 2 chars
#define ISTR_23_US          "\023uS"
// ISTR_PPMFRAME_MSEC before \015 max 9 chars, after max 4 chars
#define ISTR_PPMFRAME_MSEC  "PPM FrLen\015mSek" // Puls Pausen Modulation
#define ISTR_SEND_RX_NUM    "Send Em Nummer [MENU]"
#define ISTR_DSM_TYPE       "DSM Typ" 
#define ISTR_PPM_1ST_CHAN   "PPM 1. Kanal"
#define ISTR_SHIFT_SEL      "Signalart" // Signalart
// ISTR_POS_NEG indexed 3 chars each
#define ISTR_POS_NEG        "\003POSNEG"
#define ISTR_E_LIMITS       "E. Grenze" //Erweiterte Grenze
#define ISTR_Trainer        "Trainer"
#define ISTR_T2THTRIG       "GasStaT2" // 2. Timer startet wenn Gas um 5% bewegt
#define ISTR_AUTO_LIMITS    "Auto Grenze"
// ISTR_1_RETA indexed 1 char each
#define ISTR_1_RETA         "\001SHGQ"
#define ISTR_FL_MODE        "FL MODUS"
#define ISTR_SWITCH         "Schalter"
#define ISTR_TRIMS          "Trimmer"
#define ISTR_MODES          "MODI"
#define ISTR_SP_FM0         " FM0"
#define ISTR_SP_FM          " FM"
#define ISTR_HELI_SETUP     "HELI EINST"
#define ISTR_SWASH_TYPE     "Taumel Typ" 
#define ISTR_COLLECTIVE     "Kollektive"
#define ISTR_SWASH_RING     "Taumel Ring"
#define ISTR_ELE_DIRECTION  "H\302H Richtung"
#define ISTR_AIL_DIRECTION  "QUE Richtung"
#define ISTR_COL_DIRECTION  "KOL Richtung" //Kollektive
#define ISTR_MODEL_POPUP    "BEARBEI\0KOPIER\0BEWEGE\0L\302SCHE"
#define ISTR_MODELSEL       "MODELWAHL"
// ISTR_11_FREE after \011 max 4 chars
#define ISTR_11_FREE        "\011frei"
#define ISTR_CALIBRATION    "KALIBRIERUNG"
// ISTR_MENU_TO_START after \003 max 15 chars
#define ISTR_MENU_TO_START  "\003[MENU] ZUM START"
// ISTR_SET_MIDPOINT after \005 max 11 chars
#define ISTR_SET_MIDPOINT   "\005SET MITPUNKT"
// ISTR_MOVE_STICKS after \003 max 15 chars
#define ISTR_MOVE_STICKS    "\003BEWG STICKS/POTS"
#define ISTR_ANA            "ANA" // Analog Input und Batterie Spannung Kalibrierung
#define ISTR_DIAG           "DIAG" // Diagnostics
// ISTR_KEYNAMES indexed 5 chars each
#define ISTR_KEYNAMES       "\005Men\305 ExitRuntr HochRechtLinks"
#define ISTR_TRIM_M_P       "Trim- +"
// ISTR_OFF_PLUS_EQ indexed 3 chars each
#define ISTR_OFF_PLUS_EQ    "\003aus += :="
// ISTR_CH1_4 indexed 3 chars each
#define ISTR_CH1_4          "\003ch1ch2ch3ch4"
#define ISTR_MULTIPLIER     "Multiplika"
#define ISTR_CAL            "Kal"
#define ISTR_MODE_SRC_SW    "\003mode\012% que sch" // Quelle Schalter
#define ISTR_RADIO_SETUP    "FUNK EINST"
#define ISTR_OWNER_NAME     "Nutzername"
#define ISTR_BEEPER         "Pieper"
// ISTR_BEEP_MODES indexed 6 chars each
#define ISTR_BEEP_MODES     "\006Lautls""TstAus""xKurz ""Kurz  ""Normal""Lang  ""xLang " // x = seh
#define ISTR_SOUND_MODE     "Sound Modus"
// ISTR_SPEAKER_OPTS indexed 10 chars each
#define ISTR_SPEAKER_OPTS   "\012Pieper    ""PiLautspre""PieprTon  ""PieLautTon""MegaSound "
#define ISTR_VOLUME         "Lautst"
#define ISTR_SPEAKER_PITCH  " Tonh\303he"
#define ISTR_HAPTICSTRENGTH " Haptische St\301rke"
#define ISTR_CONTRAST       "Kontrast"
#define ISTR_BATT_WARN      "Batterie Warnung" 
// ISTR_INACT_ALARM m for minutes after \023 - single char
#define ISTR_INACT_ALARM    "Inaktivit\301ts alarm\023m"
#define ISTR_THR_REVERSE    "Gas umkehren"
#define ISTR_MINUTE_BEEP    "Minutenton"
#define ISTR_BEEP_COUNTDOWN "Piep Countdown"
#define ISTR_FLASH_ON_BEEP  "Blitz auf Piep"
#define ISTR_LIGHT_SWITCH   "Lichtschalter"
#define ISTR_LIGHT_INVERT   "Licht umkehren"
#define ISTR_LIGHT_AFTER    "Licht aus nach"
#define ISTR_LIGHT_STICK    "Licht an Stk Mv"
#define ISTR_SPLASH_SCREEN  "Startbildschirm"
#define ISTR_SPLASH_NAME    "Start Name"
#define ISTR_THR_WARNING    "Gas Warnung"
#define ISTR_DEAFULT_SW     "Standard Sch"
#define ISTR_MEM_WARN       "Speicher Warnung"
#define ISTR_ALARM_WARN     "Alarm Warnung"
#define ISTR_POTSCROLL      "PotScroll"
#define ISTR_STICKSCROLL    "StickScroll"
#define ISTR_BANDGAP        "BandL\305cke"
#define ISTR_ENABLE_PPMSIM  "aktiviere PPMSIM"
#define ISTR_CROSSTRIM      "KreuzTrim"
#define ISTR_INT_FRSKY_ALRM "Int. Frsky alarm"
#define ISTR_MODE           "Modus"

// SWITCHES_STR 3 chars each
#ifdef PCBSKY
#define ISWITCHES_STR       "\003GAS""SEI""H\302H""ID0""ID1""ID2""QUE""FWK""TRN""SW1""SW2""SW3""SW4""SW5""SW6""SW7""SW8""SW9""SWA""SWB""SWC"
#define IHW_SWITCHES_STR     "\002SASBSCSDSESFSGSH"
#define IHW_SWITCHARROW_STR  "\200-\201"
#endif
#ifdef PCBX9D
#define ISWITCHES_STR        "\003SA\200SA-SA\201SB\200SB-SB\201SC\200SC-SC\201SD\200SD-SD\201SE\200SE-SE\201SF\200SF\201SG\200SG-SG\201SH\200SH\201BR1BR2BR3BR4BR5BR6BR7BR8BR9BRABRBBRCBRDBREBRFBRGBRHBRIBRJBRKBRLBRMBRNBRO"
#define IHW_SWITCHES_STR     "\002SASBSCSDSESFSGSH"
#define IHW_SWITCHARROW_STR  "\200-\201"
#endif
#define ISWITCH_WARN_STR	   "Schalter Warnung"
// CURV_STR indexed 3 chars each
// c17-c24 added for timer mode A display
#define ICURV_STR					 "\003---x>0x<0|x|f>0f<0|f|c1 c2 c3 c4 c5 c6 c7 c8 c9 c10c11c12c13c14c15c16c17c18c19c20c21c22c23c24"
// CSWITCH_STR indexed 7 chars each
#define ICSWITCH_STR        "\007----   v>ofs  v<ofs  |v|>ofs|v|<ofsUND    ODER   XOR    ""v1==v2 ""v1!=v2 ""v1>v2  ""v1<v2  ""v1>=v2 ""v1<=v2 ZeitAus"

#define ISWASH_TYPE_STR     "\006---   ""120   ""120X  ""140   ""90    "

#define ISTR_STICK_NAMES    "SEI H\302H GAS QUE "

#define ISTR_STAT           "STAT"
#define ISTR_STAT2          "STAT2"
// ISTR_TRIM_OPTS indexed 3 chars each
#define ISTR_TRIM_OPTS      "\003ExFxFeFeiMitStk" // ExF= extra fein; Fei = fein; Mit = medium - mittel; Crs = Coarse sehr stark
#define ISTR_TTM            "GTm" // Gas Trim
#define ISTR_FUEL           "TANK"
#define ISTR_12_RPM         "\012UPM"
#define ISTR_LAT_EQ         "Bre="
#define ISTR_LON_EQ         "L\301n="
#define ISTR_ALT_MAX        "H\302h=\011m   Max="
#define ISTR_SPD_KTS_MAX    "Ges=\011kts Max="
#define ISTR_11_MPH         "\011mph"

#define ISTR_SINK_TONES	   "Sink Tones"
#define ISTR_FRSKY_MOD      "Frksy Mod Done"

// ersky9x strings
#define ISTR_ST_CARD_STAT   "SD CARD STAT"
#define ISTR_4_READY        "\004Bereit"
#define ISTR_NOT            "NICHT"
#define ISTR_BOOT_REASON    "BOOT GRUND"
#define ISTR_6_WATCHDOG     "\006W\300CHTER"
#define ISTR_5_UNEXPECTED   "\005UNERWARTET"
#define ISTR_6_SHUTDOWN     "\006AUSSCHALTEN"
#define ISTR_6_POWER_ON     "\006EINSCHALTEN"
// ISTR_MONTHS indexed 3 chars each
#define ISTR_MONTHS         "\003XxxJanFebMrzAprMaiJunJulAugSepOktNovDez"
#define ISTR_MENU_REFRESH   "[MENU] NEU LADEN"
#define ISTR_DATE_TIME      "DATUM-ZEIT"
#define ISTR_SEC            "Sek."
#define ISTR_MIN_SET        "Min.\015Set"
#define ISTR_HOUR_MENU_LONG "Std.\012MENU LANG"
#define ISTR_DATE           "Datum"
#define ISTR_MONTH          "Monat"
#define ISTR_YEAR_TEMP      "Jahr\013Temp."
#define ISTR_YEAR           "Jahr"
#define ISTR_BATTERY        "BATTERIE"
#define ISTR_Battery        "Batterie"
#define ISTR_CURRENT_MAX    "Momentan\016Max"
#define ISTR_CPU_TEMP_MAX   "CPU temp.\014C Max\024C"
#define ISTR_MEMORY_STAT    "SPEICHER STAT"
#define ISTR_GENERAL        "Generell"
#define ISTR_Model          "Modell"
#define ISTR_RADIO_SETUP2   "FERNST EINST2"
#define ISTR_BRIGHTNESS     "Helligkeit"
#define ISTR_CAPACITY_ALARM "Kapazit\301ts Alarm"
#define ISTR_BT_BAUDRATE    "Bt baudrate" 
#define ISTR_ROTARY_DIVISOR "Rot Teiler"
#define ISTR_STICK_LV_GAIN  "Stick LV Anstieg"
#define ISTR_STICK_LH_GAIN  "Stick LH Anstieg"
#define ISTR_STICK_RV_GAIN  "Stick RV Anstieg"
#define ISTR_STICK_RH_GAIN  "Stick RH Anstieg"
#define ISTR_BIND					  "Binden"
#define ISTR_RANGE					"RWeite Test"

#define ISTR_ALRMS_OFF			"Alarme inaktiv"
#define ISTR_OLD_EEPROM			" EEPROM ist veraltet   TESTE EINSTELL/KALIB"
#define ISTR_TRIGA_OPTS			"OFFABSTHsTH%"
#define ISTR_CHK_MIX_SRC		"PR\304FE MIX QUELLEN"

#define ISTR_BT_TELEMETRY		"BT Telemetrie"
#define ISTR_FRSKY_COM_PORT "FrSky Com Port"
#define ISTR_INVERT_COM1		"Invert COM 1"
#define ISTR_LOG_SWITCH			"Log Schalt"
#define ISTR_LOG_RATE				"Log Rate"
#define ISTR_6_BINDING			"\006BINDE"
#define ISTR_RANGE_RSSI			"RWeite Test RSSI:"
#define ISTR_FAILSAFE				"FAILSAFE"
#define ISTR_VOLUME_CTRL		"Volume Control"
#define ISTR_PROT_OPT				"\006PPM   PXX   DSM2  "
#define ISTR_TYPE						"  Typ"
#define ISTR_COUNTRY				"    Land"
#define ISTR_SP_FAILSAFE		" Failsafe"
#define ISTR_PPM2_START			"PPM2 StartChan"
#define ISTR_FOLLOW					"Folgen"
#define ISTR_PPM2_CHANNELS	"PPM2 Kan\301le"
#define ISTR_FILTER_ADC			"Filter ADC"
#define ISTR_SCROLLING			"Scrolling"
#define ISTR_ALERT_YEL			"Alarm [Gel]"
#define ISTR_ALERT_ORG			"Alarm [Org]"
#define ISTR_ALERT_RED			"Alarm [Rot]"
#define ISTR_LANGUAGE				"Sprache"

