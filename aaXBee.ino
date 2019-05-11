//To do:
//1. Add FR command to init sequence (this in the gsXBee library) -- DONE
//2. Review sleep sequences, RTC alarm interrupt enable. -- Added RTC.alarmInterrupt call.
//                    ... need to look at moving code to gotoSleep() and/or look at the MCU interrupt setup code (not consistent)
//3. Look at using symbols from the libraries for conditional compilation. -- DONE

/*-----------------------------------------------------------------------------*
 * Double-A XBee Sensor Node for GroveStreams Wireless Sensor Network.         *
 * A low-power Arduino-based wireless sensor node than runs on two AA cells.   *
 * For complete information on the circuit design, including Eagle files,      *
 * configuration options, programming requirements, etc. see the GitHub        *
 * repository: http://github.com/JChristensen/aaXBee_HW                        *
 * Jack Christensen Jun 2015                                                   *
 *                                                                             *
 * This basic sketch sends a sequence number, the temperature from the         *
 * MCP9808, battery and regulator voltages.                                    *
 *                                                                             *
 * Developed with Arduino 1.0.6.                                               *
 * Set ATmega328P Fuses (L/H/E): 0x7F, 0xDE, 0x06.                             *
 * Uses an 8MHz crystal with the CKDIV8 fuse bit programmed, so the system     *
 * clock is 1MHz after reset. This is to ensure the MCU is not overclocked     *
 * at low voltages when the boost regulator is disabled. Clock is              *
 * changed to 8MHz after the regulator is enabled.                             *
 *                                                                             *
 * Double-A XBee Sensor Node by Jack Christensen is licensed under             *
 * CC BY-SA 4.0, http://creativecommons.org/licenses/by-sa/4.0/                *
 *-----------------------------------------------------------------------------*/

//Set ATmega328P Fuses:
//avrdude -p m328p -U lfuse:w:0x7f:m -U hfuse:w:0xde:m -U efuse:w:0x06:m -v

//XBee Configuration.
//Model no. XB24-Z7WIT-004 (XB24-ZB)
//Firmware: ZigBee End Device API 29A7
//All parameters factory default except:
//  ID PAN ID       42 (or other non-zero value, all modules on a network must have the same PAN ID)
//  NI Node ID      aaXB1_10010001
//  BD Baud Rate    57600 [6]
//  AP API Enable   2
//  SM Sleep Mode   Pin Hibernate [1]
//  D5 DIO5 Config  4 (disables ASSOC LED)
//  P0 PIO10 Config 4 (disables RSSI LED)
//  PO Polling Rate 0 (poll the parent device faster so we can sleep sooner)
//
//Because this node sleeps for relatively long intervals, parent devices
//in the network need their child poll timeout set long enough to maintain
//the sleeping node in their child table. The following settings should be
//made for the coordinator and routers in the network to allow for end
//devices that sleep up to 5 minutes:
//  SN Number of Sleep Periods    0x10
//  SP Sleep Period               0x7D0

//uncomment next line if a ChipCap2 is used
#include <ChipCap2.h>         //http://github.com/JChristensen/ChipCap2

//uncomment next line if a DHT22 is used
//#include <DHT.h>              //http://github.com/JChristensen/DHT-sensor-library

#include <avr/sleep.h>        //standard library
#include <avr/wdt.h>          //standard library
#include <DS3232RTC.h>        //http://github.com/JChristensen/DS3232RTC
#include <gsXBee.h>           //http://github.com/JChristensen/gsXBee
#include <MCP9808.h>          //http://github.com/JChristensen/MCP9808
#include <Streaming.h>        //http://arduiniana.org/libraries/streaming/
#include <Time.h>             //http://playground.arduino.cc/Code/Time
#include <Wire.h>             //http://arduino.cc/en/Reference/Wire
#include <XBee.h>             //http://github.com/andrewrapp/xbee-arduino
#include "circuit.h"          //part of this sketch

//trap the MCUSR value after reset to determine the reset source
//and ensure the watchdog is reset. this code does not work with a bootloader.
uint8_t mcusr __attribute__ ((section (".noinit")));
void wdt_init() __attribute__((naked)) __attribute__((used)) __attribute__((section(".init3")));

void wdt_init()
{
    mcusr = MCUSR;
    MCUSR = 0;        //must clear WDRF in MCUSR in order to clear WDE in WDTCSR
    wdt_reset();
    wdt_disable();
}

void setup()
{
    Circuit.begin( F(__FILE__) );
}

void loop()
{
    static time_t rtcTime;
    static time_t startupTime;
    static uint32_t msTX;
    static uint8_t txFails;

    enum STATES_t { INIT, SEND_DATA, WAIT_ACK, SEND_TIMESYNC, SET_ALARM };
    static STATES_t STATE = INIT;

    switch ( STATE )
    {
    case INIT:
        {
            //calculate the first alarm
            rtcTime = rtcGet();
            startupTime = rtcTime;
            time_t alarmTime = rtcTime - rtcTime % (XB.txInterval * 60) + XB.txOffset * 60 + XB.txSec - XB.txWarmup;
            if ( alarmTime <= rtcTime + 5 ) alarmTime += XB.txInterval * 60;
            //set RTC alarm to match on hours, minutes, seconds
            RTC.setAlarm(ALM1_MATCH_HOURS, second(alarmTime), minute(alarmTime), hour(alarmTime), 0);
            RTC.alarm(ALARM_1);                   //clear RTC interrupt flag
            RTC.alarmInterrupt(ALARM_1, true);    //enable alarm interrupts
            printTimes(rtcTime, alarmTime);

            EICRA = _BV(ISC11);               //interrupt on falling edge
            EIFR = _BV(INTF1);                //clear the interrupt flag (setting ISCnn can cause an interrupt)
            EIMSK = _BV(INT1);                //enable interrupt
            STATE = SEND_DATA;
        }
        break;

    case SEND_DATA:
        digitalWrite(PIN.builtinLED, LOW);
        Circuit.gotoSleep();                 //shut the regulator down and sleep until it's time to send data
        {                                    //read sensors, send data
            static uint16_t seqNbr;
            char buf[80];
            rtcTime = rtcGet();
            digitalWrite(PIN.sensorPower, HIGH);         //power up the sensors
#ifdef DHT_H
            dht.begin();
#endif
            time_t alarmTime = rtcTime + XB.txWarmup;    //sleep the MCU during sensor conversion time
            RTC.setAlarm(ALM1_MATCH_HOURS, second(alarmTime), minute(alarmTime), hour(alarmTime), 0);
            RTC.alarm(ALARM_1);                          //clear RTC interrupt flag
            RTC.alarmInterrupt(ALARM_1, true);           //enable alarm interrupts
            printTimes(rtcTime, alarmTime);
            Circuit.gotoSleep(true);                     //sleep while the sensors do their thing, leave boost on
            mcp9808.read();                              //read the temperature sensor
#ifdef _CHIPCAP2_H
            int dT, dH;                                  //temperature, humidity
            cc2Status_t cc2stat = cc2.read();
            if ( cc2stat == CC2_VALID )
            {
                dT = cc2.cRaw();
                dH = cc2.rhRaw();
            }
            else
            {
                dT = dH = -9999;
                Serial << F("CC2 error ") << cc2stat << endl;
            }
#endif
#ifdef DHT_H
            int dT, dH;                                  //DHT22 temperature, humidity
            if ( !dht.getData( &dT, &dH ) )              //read the DHT22
            {
                dT = dH = -9999;
                Serial << F("DHT error\n");
            }
#endif
            digitalWrite(PIN.sensorPower, LOW);          //power sensors down

            //build the payload -- assume CC2 *or* DHT22, not both
            //note -- should check cc2stat
#ifdef _CHIPCAP2_H
            sprintf(buf, "&s=%u&u=%lu&t=%i&d=%i&h=%i&b=%i&r=%i",
                ++seqNbr, rtcTime - startupTime, mcp9808.tAmbient, dT, dH, Circuit.vBat, Circuit.vReg );
#elif DHT_H
            sprintf(buf, "&s=%u&u=%lu&t=%i&d=%i&h=%i&b=%i&r=%i",
                ++seqNbr, rtcTime - startupTime, mcp9808.tAmbient, dT, dH, Circuit.vBat, Circuit.vReg );
#else
            sprintf(buf, "&s=%u&u=%lu&t=%i&b=%i&r=%i",
                ++seqNbr, rtcTime - startupTime, mcp9808.tAmbient, Circuit.vBat, Circuit.vReg );
#endif

            //Send the data
            Circuit.xbeeEnable(true);
            while ( XB.read() != NO_TRAFFIC );    //handle any incoming traffic
            digitalWrite(PIN.builtinLED, HIGH);
            XB.sendData(buf);
            msTX = millis();
            STATE = WAIT_ACK;
        }
        break;

    case WAIT_ACK:
        if ( millis() - msTX >= XBEE_TIMEOUT )
        {
            Serial << millis() << F("\tXBee ACK timeout\n");
            STATE = SEND_TIMESYNC;
            break;
        }
        else
        {
            xbeeReadStatus_t xbStatus = XB.read();
            if ( xbStatus == TX_ACK || xbStatus == TX_FAIL )
            {
                if ( xbStatus == TX_ACK )
                {
                    txFails = 0;
                }
                else if ( ++txFails >= MAX_TX_FAILS )
                {
                    Serial << millis() << F("\tToo many TX failures\n");
                    XB.mcuReset();
                }
                STATE = SEND_TIMESYNC;
                break;
            }
        }
        break;

    case SEND_TIMESYNC:
        if ( day(rtcTime) != day(Circuit.lastTimesync) )        //time sync once per day
        {
            XB.requestTimeSync(rtcTime);
            if ( XB.waitFor(RX_TIMESYNC, XBEE_TIMEOUT) == READ_TIMEOUT )    //wait for the sync response
            {
                Serial << millis() << F("\tTime sync timeout\n");
            }
        }
        STATE = SET_ALARM;
        break;

    case SET_ALARM:                               //calculate and set the next alarm
        while ( XB.read() != NO_TRAFFIC );        //get any other traffic
        Circuit.xbeeEnable(false);                //done with the XBee, put it to sleep
        {
            rtcTime = rtcGet();
            time_t alarmTime = rtcTime - rtcTime % (XB.txInterval * 60) + XB.txOffset * 60 + XB.txSec - XB.txWarmup;
            if ( alarmTime <= rtcTime + 5 ) alarmTime += XB.txInterval * 60;
            RTC.setAlarm(ALM1_MATCH_HOURS, second(alarmTime), minute(alarmTime), hour(alarmTime), 0);
            RTC.alarm(ALARM_1);                   //clear RTC interrupt flag
            RTC.alarmInterrupt(ALARM_1, true);    //enable alarm interrupts
            printTimes(rtcTime, alarmTime);
        }
        STATE = SEND_DATA;
        break;
    }
}

//interrupt from the RTC alarm.
ISR(INT1_vect)
{
    //nothing to do here, the interrupt is just to wake the MCU.
}
