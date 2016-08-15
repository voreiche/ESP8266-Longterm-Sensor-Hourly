
#include <ESP8266WiFi.h>

// set Analog2DigitalConverter battery test mode.
ADC_MODE(ADC_VCC); //vcc read-mode

// Include API-Headers
extern "C" {
#include "ets_sys.h"
#include "os_type.h"
#include "osapi.h"
#include "mem.h"
#include "user_interface.h"
#include "cont.h"
}

#include <credentials.h>;


// state definitions for state machine
#define STATE_COLDSTART 0
#define STATE_SLEEP_WAKE 1
#define STATE_ALARM 2
#define STATE_CONNECT_WIFI 4

#define ALARM_PIN 14                                 // pin to test for alarm condition
#define ALARM_POLARITY 1                            // signal representing alarm

// define DeepSleep interval
#define SLEEP_TIME 60*1*1000000                    // sleep interval in us

#define SLEEP_COUNTS_FOR_LIVE_MAIL 24*2*7           // send live mail every 7 days. Why is there a factor of two?
#define SLEEP_COUNTS_FOR_BATT_CHECK 2*24            // send low-Batt mail once a day

#define BATT_WARNING_VOLTAGE 2.4                    // Voltage for Low-Bat warning

#define WIFI_CONNECT_TIMEOUT_S 20                   // max time for wifi connect to router, if exceeded restart


// RTC-MEM Adresses
#define RTC_BASE 65
#define RTC_STATE 66
#define RTC_WAKE_COUNT 67
#define RTC_MAIL_TYPE 68

// mail types
#define MAIL_NO_MAIL 0         // don't send any mail
#define MAIL_WELCOME 1         // mail at startup
#define MAIL_LIVE_SIGNAL 2     // send 'alive' mail
#define MAIL_ALARM 3           // send an 'alarm' mail
#define MAIL_LOW_BAT 4         // send 'battery low' mail

#define SPARKFUN_BATTERY 1

// correction factor for sampled battery voltage 
#define VCC_ADJ 1.096

#define SERIAL_DEBUG


// global variables
// multi purpose byte buffer used to transfer data to/ from RTC Memory
byte buf[10];

typedef union rtc_mem_buffer {
  byte raw_bytes[10];
  byte magic_bytes[2];
  
} RTC_Mem_Buffer;

RTC_Mem_Buffer rtc_mem_buffer;

byte state;   // state variable

byte event = 0;

uint32_t sleepCount;   // number of sleep intervals passed?

// time1 and time2 are used to ensure the timeout interval for setting up a working wifi connection
uint32_t time1;        // point in time when wifi connection is about to be established
uint32_t time2;        // point in time during establishment of wifi connection

// Temporary buffer
uint32_t b = 0;

int i;

WiFiClient client;

void setup() {
  WiFi.forceSleepBegin();  // send wifi to sleep to reduce power consumption
  yield();
  system_rtc_mem_read(RTC_BASE, rtc_mem_buffer, 2); // read 2 bytes from RTC-MEMORY

  // only show debug info when needed
#ifdef SERIAL_DEBUG_ENABLED
  Serial.begin(115200);
  delay(10);
  Serial.println();
  Serial.println();
  Serial.println(F("Started from reset"));
#endif

  // do we need to perform a 'cold start'?
  if ((buf[0] != 0x55) || (buf[1] != 0xaa))  // do a 'cold start' if magic number is not present
  { 
    state = STATE_COLDSTART;
    
    // remember that we already did a 'cold start'
    buf[0] = 0x55; buf[1] = 0xaa;
    system_rtc_mem_write(RTC_BASE, buf, 2);
  }
  else // reset was due to sleep-wake or external event
  { 
    // read state from RTC Memory
    system_rtc_mem_read(RTC_STATE, buf, 1);
    state = buf[0];
    
    if (state == STATE_SLEEP_WAKE) // could be a sleep wake or an alarm
    { 
      pinMode(ALARM_PIN, INPUT);
      bool pinState = digitalRead(ALARM_PIN);  // read the alarm pin to find out whether an alarm is the reset cause
      Serial.printf("GPIO %d read: %d\r\n", ALARM_PIN, pinState);
      if (pinState == ALARM_POLARITY) { 
        // this is an alarm!
        state = STATE_ALARM;
      }
    }
  }

  // now the restart cause is clear, handle the different states
  Serial.printf("State: %d\r\n", state);

  switch (state)
  { case STATE_COLDSTART:   // first run after power on - initializes
      // reset the sleep count and write its value to RTC Memory
      sleepCount = 0;
      // this will overwrite the rtc_mail_type with the second byte of sleepCount.
      // More see below.
      system_rtc_mem_write(RTC_WAKE_COUNT, &sleepCount, 4);
      
      // 
      buf[0] = MAIL_WELCOME;
      // this call will overwrite the second byte of the sleep count in RTCMemory.
      // See also above
      system_rtc_mem_write(RTC_MAIL_TYPE, buf, 1); // send a welcome-mail when wifi is on
      
      // switch WiFi back on an become an active WiFi station
      WiFi.forceSleepWake();
      WiFi.mode(WIFI_STA);
      
      // prepare to activate wifi
      buf[0] = STATE_CONNECT_WIFI;  // one more sleep required to wake with wifi on
      system_rtc_mem_write(RTC_STATE, buf, 1); // set state for next wakeUp
      
      ESP.deepSleep(10, WAKE_RFCAL);
      yield();
      break;
    case STATE_SLEEP_WAKE:
      // we woke up so we should restore tghe sleepCount variable from RTC Memory
      system_rtc_mem_read(RTC_WAKE_COUNT, &sleepCount, 4); // read counter
      sleepCount++;
      
      // is it time to send an alive mail?
      if (sleepCount > SLEEP_COUNTS_FOR_LIVE_MAIL)
      { 
        // its time to send mail as live signal
        // this kills 2nd byte of sleepCounter in RTC
        buf[0] = MAIL_LIVE_SIGNAL;
        system_rtc_mem_write(RTC_MAIL_TYPE, buf, 1); // set mail type to send
        
        // activate wifi in station mode
        WiFi.forceSleepWake();
        WiFi.mode(WIFI_STA);
        
        // write new state to RTC Memory
        buf[0] = STATE_CONNECT_WIFI;  // one more sleep required to to wake with wifi on
        system_rtc_mem_write(RTC_STATE, buf, 1); // set state for next wakeUp
        ESP.deepSleep(10, WAKE_RFCAL);
        yield();
      }
      
      // do we need to check the battery?
      if (sleepCount > SLEEP_COUNTS_FOR_BATT_CHECK)
      { 
        // is the battery depleted less than the warning level?
        if (ESP.getVcc()* VCC_ADJ < BATT_WARNING_VOLTAGE)
        { 
          sleepCount = 0;  // reset sleepcount so battery warning is not sent every wakeup
          buf[0] = MAIL_LOW_BAT;
          system_rtc_mem_write(RTC_MAIL_TYPE, buf, 1); // set mail type to send
          system_rtc_mem_write(RTC_WAKE_COUNT, &sleepCount, 4); // reset sleepcounter in RTC Memory
          // overwrites mail type!!!
          // prepare to activate wifi
          WiFi.forceSleepWake();
          WiFi.mode(WIFI_STA);
          buf[0] = STATE_CONNECT_WIFI;  // one more sleep required to to wake with wifi on
          system_rtc_mem_write(RTC_STATE, buf, 1); // set state for next wakeUp
          ESP.deepSleep(10, WAKE_RFCAL);
          yield();
        }
      }

      // no special event, go to sleep again
      system_rtc_mem_write(RTC_WAKE_COUNT, &sleepCount, 4); // write counter
      buf[0] = STATE_SLEEP_WAKE;
      system_rtc_mem_write(RTC_STATE, buf, 1);            // set SLEEP_WAKE state for next wakeUp
      
      // goto sleep and disable WiFi
      ESP.deepSleep(SLEEP_TIME, WAKE_RF_DISABLED);
      yield();                                            // pass control back to background processes to prepare sleep
      break;

    case STATE_ALARM:
      // remember to send a mail alarm
      buf[0] = MAIL_ALARM;
      system_rtc_mem_write(RTC_MAIL_TYPE, buf, 1); // set mail type to send
      
      // prepare to activate wifi
      WiFi.forceSleepWake();
      WiFi.mode(WIFI_STA);
      
      // remember to activate WiFi. 
      buf[0] = STATE_CONNECT_WIFI;  // one more sleep required to to wake with wifi on
      system_rtc_mem_write(RTC_STATE, buf, 1); // set state for next wakeUp
      // sleep for a short period and bring up WiFi calibrated when snooze is finished.
      ESP.deepSleep(10, WAKE_RFCAL);
      yield();
      break;
      
    case STATE_CONNECT_WIFI:
      // wake up WiFi and wait some time to get it running
      WiFi.forceSleepWake();
      delay(500);
      // set wifi sleep mode
      wifi_set_sleep_type(MODEM_SLEEP_T);
      // become a wifi station
      WiFi.mode(WIFI_STA);
      yield();

      // begin of timeout interval
      time1 = system_get_time();

      // Connect to WiFi network
      Serial.println();
      Serial.println();
      Serial.print("Connecting to ");
      Serial.println(ASP_ssid);
      WiFi.begin(ASP_ssid, ASP_password);
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        time2 = system_get_time();
        // timeout pending? 
        if ((time2 - time1) > WIFI_CONNECT_TIMEOUT_US)  // wifi connection lasts too ling, retry
        { 
          // start all over again
          ESP.deepSleep(10, WAKE_RFCAL);
          yield();
        }
      }
      
      Serial.println("");
      Serial.println("WiFi connected");

      // read mail type from RTCMemory
      system_rtc_mem_read(RTC_MAIL_TYPE, buf, 1);

      if (buf[0] == MAIL_ALARM) {
        Serial.printf("Post to Sparkfun %d\r\n", buf[0]);
        //sendSparkfun(SPARKFUN_BATTERY);
        sendubidots(SPARKFUN_BATTERY);
      } else {
        Serial.printf("Send email %d\r\n", buf[0]);
        sendEmail(buf[0]);
      }
      
      // now re-initialize
      sleepCount = 0;
      system_rtc_mem_write(RTC_WAKE_COUNT, &sleepCount, 4); // initialize counter
      buf[0] = MAIL_NO_MAIL;
      system_rtc_mem_write(RTC_MAIL_TYPE, buf, 1); // no mail pending
      buf[0] = STATE_SLEEP_WAKE;
      system_rtc_mem_write(RTC_STATE, buf, 1); // set state for next wakeUp
      // sleep and when wake up occurs wifi will be disabled
      ESP.deepSleep(SLEEP_TIME, WAKE_RF_DISABLED);
      yield();                                            // pass control back to background processes to prepate sleep

      break;

  }
  delay(1000); // will this be executed?
}

void loop()
{
  delay(10);


}