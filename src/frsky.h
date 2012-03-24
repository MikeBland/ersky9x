/*
 * Author - Bertrand Songis <bsongis@gmail.com>
 *
 * frsky.cpp original authors - Bryan J.Rentoul (Gruvin) <gruvin@gmail.com> and Philip Moss Adapted from jeti.cpp code by Karl
 * Szmutny <shadow@privy.de>* 
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

#ifndef FRSKY_H
#define FRSKY_H

/*  FrSky Hub Info
DataID Meaning       Unit   Range   Note
0x01   GPS altitude  m              Before”.”
0x09                                After “.”
0x02   Temperature1  °C     -30-250
0x03   RPM           BPS    0-60000
0x04   Fuel Level    %      0, 25, 50, 75, 100
0x05   Temperature2  °C     -30-250
0x06   Volt          1/500v 0-4.2v, top 4 bits are cell #
0x10   Altitude      m      0-9999  Before “.”
0x21   Altitude      m              After "."
0x11   GPS speed     Knots          Before “.”
0x19                                After “.”
0x12   Longitude     dddmm.mmmm     Before “.”
0x1A                                After “.”
0x22   E/W
0x13   Latitude      ddmm.mmmm      Before “.”
0x1B                                After “.”
0x23   N/S
0x14   Course        degree 0-360   Before “.”
0x1C                                After “.”
0x15   Date/Month
0x16   Year
0x17   Hour /Minute
0x18   Second
0x24   Acc-x         1/256g -8g ~ +8g
0x25   Acc-y         1/256g -8g ~ +8g
0x26   Acc-z         1/256g -8g ~ +8g
0x28   Current       1A   0-100A
0x3A   Voltage(amp sensor) 0.5v 0-48V Before “.”
0x3B   Voltage(amp sensor)            After “.”
 
DataID Meaning       Unit   Range   Note
0x01   GPS altitude  m              Before”.”
0x02   Temperature1  °C     -30-250
0x03   RPM           BPS    0-60000
0x04   Fuel Level    %      0, 25, 50, 75, 100
0x05   Temperature2  °C     -30-250
0x06   Volt          1/500v 0-4.2v
0x07
0x08
0x09   GPS altitude  m              After “.”
0x0A
0x0B
0x0C
0x0D
0x0E
0x0F
0x10   Altitude      m      0-9999
0x11   GPS speed     Knots          Before “.”
0x12   Longitude     dddmm.mmmm     Before “.”
0x13   Latitude      ddmm.mmmm      Before “.”
0x14   Course        degree 0-360   Before “.”
0x15   Date/Month
0x16   Year
0x17   Hour /Minute
0x18   Second
0x19   GPS speed     Knots          After “.”
0x1A   Longitude     dddmm.mmmm     After “.”
0x1B   Latitude      ddmm.mmmm      After “.”
0x1C   Course        degree 0-360   After “.”
0x1D
0x1E
0x1F
0x20
0x21   Altitude      m              After "."
0x22   E/W
0x23   N/S
0x24   Acc-x         1/256g -8g ~ +8g
0x25   Acc-y         1/256g -8g ~ +8g
0x26   Acc-z         1/256g -8g ~ +8g
0x27
0x28   Current       1A   0-100A
// . . .
0x3A   Voltage(amp sensor) 0.5v 0-48V Before “.”
0x3B   Voltage(amp sensor)            After “.”
  
 */





// .20 seconds
#define FRSKY_TIMEOUT10ms 20

enum AlarmLevel {
  alarm_off = 0,
  alarm_yellow = 1,
  alarm_orange = 2,
  alarm_red = 3
};

#define ALARM_GREATER(channel, alarm) ((g_model.frsky.channels[channel].alarms_greater >> alarm) & 1)
#define ALARM_LEVEL(channel, alarm) ((g_model.frsky.channels[channel].alarms_level >> (2*alarm)) & 3)

struct FrskyData {
  uint8_t value;
  uint8_t min;
  uint8_t max;
	uint8_t offset ;
  void set(uint8_t value);
	void setoffset();
};

struct Frsky_current_info
{
uint16_t Amp_hour_boundary ;
uint16_t Amp_hour_prescale ;
uint16_t Amp_hours ;
void update(uint8_t value);
} ;
 
extern Frsky_current_info Frsky_current[2] ;

// Global Fr-Sky telemetry data variables
extern uint8_t frskyStreaming; // >0 (true) == data is streaming in. 0 = nodata detected for some time
extern uint8_t frskyUsrStreaming; // >0 (true) == user data is streaming in. 0 = no user data detected for some time
extern uint8_t FrskyAlarmSendState;
extern FrskyData frskyTelemetry[4];
//extern FrskyData frskyRSSI[2];
extern int16_t FrskyHubData[] ;
extern uint8_t FrskyVolts[];
extern uint8_t FrskyBattCells;
extern uint8_t FrskyAlarmCheckFlag ;
extern uint8_t MaxGpsSpeed ;

void FRSKY_Init(void);
void FRSKY10mspoll(void);
uint32_t FRSKY_setTxPacket( uint8_t type, uint8_t value, uint8_t p1, uint8_t p2 ) ;
void check_frsky( void ) ;

void FRSKY_setModelAlarms(void) ;

enum AlarmLevel FRSKY_alarmRaised(uint8_t idx, uint8_t alarm=2) ;
void FRSKY_alarmPlay(uint8_t idx, uint8_t alarm) ;
void resetTelemetry();
extern void frskyTransmitBuffer( uint32_t size ) ;

#endif



