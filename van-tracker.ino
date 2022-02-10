/*
   Written by Joshua Wallis
   Depends on https://github.com/jwallis/SIM7000-LTE-Shield
     which depends on https://github.com/botletics/SIM7000-LTE-Shield
     which depends on https://github.com/adafruit/Adafruit_FONA
   Support https://www.adafruit.com  and buy their neato products!
   Support https://www.botletics.com and buy their neato products!
*/

/*
Blink debug codes - the Arduino Nono will blink these codes during operation:

Basic info codes (0 longs followed by THIS MANY shorts):
  1 = about to check inbound SMSs
  2 = about to execute watchdog processes
  3 = about to reset SimCom chip
  4 = after powering on, Arduino successfully connected to SimCom chip
  5 = after powering on, Arduino trying to connect to SimCom
  6 = trying to connect to cell network
  7 = trying to connect to GPS

Event codes (1 long followed by THIS MANY shorts).  Notice odd numbers are bad, even numbers ok:
  1  = not used (see below for error codes in failure to send SMS)
  2  = success sending SMS
  3  = failed  deleting SMS
  4  = success deleting SMS
  5  = failed  turning on  GPS
  6  = success sending plain SMS
  7  = failed  getting fix on GPS
  8  = success getting fix on GPS

Error codes in failure to send SMS (2 longs followed by THIS MANY shorts) - see https://hologram.io/docs/reference/cloud/embedded/:
  1 = Connection was closed so we couldn’t read enough
  2 = Couldn’t parse the message - possibly the wrong devKey
  3 = Auth section of message was invalid
  4 = Payload type was invalid
  5 = Protocol type was invalid
  6 = An internal error occurred
  7 = Metadata section of message was formatted incorrectly
  8 = Topic contained invalid characters or was too long
  9 = DevKey is not set - contact Van Tracker support
  10 = Unknown
  11 = Failed to send plain SMS

Connection failure either to SimCom chip or cellular network (3 long followed by THIS MANY shorts):
  This happens before restarting, which will have its own blink code, see above
    1 = Failed to connect to SimCom chip
    2 = Not registered
    3 = Registration denied
    4 = Unknown
*/

//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//    BEFORE EVERY CHECK-IN, CHECK
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

//  1. Any time we increase the size of a PROGMEM string and call sendSMS() with it,
//     be sure to check the size of variable "char message[n]"
//     in function "bool sendSMS(char* send_to, const __FlashStringHelper* messageInProgmem)"

//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//    SET NETWORK & HARDWARE OPTIONS
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
#define APN               F("hologram")
#define SERVER_NAME       F("cloudsocket.hologram.io")
#define SERVER_PORT       9999

#define VT_VERSION        F("VT 3.2.0")

//    ONLY ONE OF THE FOLLOWING CONFIGURATIONS CAN BE UNCOMMENTED AT A TIME
//    Which VT model is this?
#define KILL_AND_DOOR_OPTION           // uses "kill" in all interactions, but also has door alerts
//#define DOOR_ONLY_OPTION               // uses "door" in all interactions, i.e. commands incoming from user as well as responses

//    ONLY ONE OF THE FOLLOWING CONFIGURATIONS CAN BE UNCOMMENTED AT A TIME
//    Choose what version you want to upload/execute on your board.
#define VAN_PROD                  // NO debug output
//#define VAN_TEST                  // Includes debug output to Serial Monitor
//#define SIMCOM_SERIAL             // Only for interacting with SimCom module using AT commands
//#define NEW_HARDWARE_ONLY         // Initializes Arduino Nano's EEPROM and a new SimCom module
//#define UPGRADING_HARDWARE_ONLY   // Initializes Arduino Nano's EEPROM

//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//    THERE ARE NO USER SETTINGS BELOW THIS LINE
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

// make sure our #defines are set up correctly
#if defined DOOR_ONLY_OPTION && defined KILL_AND_DOOR_OPTION
  "at most 1 VT model can be defined"
#endif
#if !defined DOOR_ONLY_OPTION && !defined KILL_AND_DOOR_OPTION
  "at least 1 VT model must be defined"
#endif

//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>
#include <EEPROM.h>

#define STARTER_INTERRUPT_ID 0    // interrupt 0 == pin 2.  I hate that.
#define STARTER_INTERRUPT_PIN 2   // interrupt 0 == pin 2.  I hate that.
#define DOOR_INTERRUPT_ID 1       // interrupt 1 == pin 3.  I hate that.
#define DOOR_INTERRUPT_PIN 3      // interrupt 1 == pin 3.  I hate that.
#define SIMCOM_RX_PIN 5
#define SIMCOM_TX_PIN 6

#define KILL_SWITCH_RELAY_PIN 4
#define DEBUG_PIN 13

// QUOTES ARE PART OF THE STRING: "20/01/31,17:03:01-20"
#define MINUTE_INDEX 13
#define HOUR_INDEX 10
#define YEAR_INDEX 1

SoftwareSerial SimComSS = SoftwareSerial(SIMCOM_TX_PIN, SIMCOM_RX_PIN);
SoftwareSerial *SimComSerial = &SimComSS;

Adafruit_FONA fona = Adafruit_FONA(99);

#define GEOFENCEENABLED_BOOL_1            0
#define GEOFENCEHOMELAT_CHAR_12           1
#define GEOFENCEHOMELON_CHAR_12           13
#define GEOFENCERADIUS_CHAR_7             25
#define GEOFENCESTART_CHAR_3              32
#define GEOFENCEEND_CHAR_3                35
#define GEOFENCEFOLLOW_BOOL_1             38
#define KILLSWITCHENABLED_BOOL_1          39
#define KILLSWITCHSTART_CHAR_3            40
#define KILLSWITCHEND_CHAR_3              43
#define OWNERPHONENUMBER_CHAR_15          46

#define LOCKDOWNENABLED_BOOL_1            61
#define GEOFENCEENABLED_BOOL_SAVED_1      62
#define GEOFENCEHOMELAT_CHAR_SAVED_12     63
#define GEOFENCEHOMELON_CHAR_SAVED_12     75
#define GEOFENCERADIUS_CHAR_SAVED_7       87    // this is deprecated, but do not re-use this space so that upgrades are simpler
#define GEOFENCESTART_CHAR_SAVED_3        94
#define GEOFENCEEND_CHAR_SAVED_3          97
#define KILLSWITCHENABLED_BOOL_SAVED_1    100
#define KILLSWITCHSTART_CHAR_SAVED_3      101
#define KILLSWITCHEND_CHAR_SAVED_3        104

#define DEVKEY_CHAR_9                     107
#define TWILIOPHONENUMBER_CHAR_12         116

#define TIMEZONE_CHAR_4                   128
#define USEPLAINSMS_BOOL_1                132
#define POWERON_BOOL_1                    133

const char STR_HOME[] PROGMEM = " feet\\\\nHome: google.com/search?q=";
const char STR_UNABLE_GPS[] PROGMEM = "Unable to get GPS signal";

// g_SimComConnectionStatus status meanings
// 0 = connected to cell network
// 1 = Failed to connect to SimCom chip
// 2 = Not registered on cell network
// 3 = Cell network registration denied
// 4 = Unknown
int8_t g_SimComConnectionStatus = 2;
int8_t g_totalFailedSendSMSAttempts = 0;

// What was the result of the most recent GPS connection attempt? Must be TRUE to start off with...
bool g_lastGPSConnAttemptWorked = true;
int16_t g_lastGPSConnAttemptTime = -1;

int16_t g_lastRestartTime = -1;
int8_t g_lastGeofenceWarningMinute = -1;

int16_t g_pauseStartedAt = -1;
int16_t g_pauseLengthInMinutes = -1;

int8_t g_geofenceWarningCount = 0;
bool g_geofenceWarningCountMessageSent = false;
int8_t g_followMessageCount = 0;

volatile bool g_v_killSwitchAndDoorAlertActive = false;
volatile bool g_v_killSwitchAndDoorAlertInitialized = false;

volatile bool g_v_killSwitchInterruptDebug = false;
volatile bool g_v_startAttemptedWhileKillSwitchActive = false;

volatile bool g_v_doorInterruptDebug = false;
volatile bool g_v_doorOpenedWhileDoorAlertActive = false;

volatile bool g_v_currentlySendingSMS = false;

void setup() {

  pinSetup();

#ifdef NEW_HARDWARE_ONLY
  setupSerial();
  initBaud();
  setupSimCom();
  initEEPROM();
  initSimCom();
  debugPrintln(F("\n\nNEW_HARDWARE_ONLY complete.  Please update #ifdefs and restart."));
  while (1) {
    debugBlink(1,0);
  }
#endif

#ifdef UPGRADING_HARDWARE_ONLY
  // This will likely be done with a cell phone and the VT unit powered off.
  //   That means when the sketch runs, it won't be able to connect to the SimCom module,
  //   so we need a version of the code that only updates EEPROM and doesn't try to mess with the SimCom.
  setupSerial();
  initEEPROM();
  debugPrintln(F("\n\nUPGRADING_HARDWARE_ONLY complete.  Please update #ifdefs and restart."));
  while (1) {
    debugBlink(1,1);
  }
#endif

#ifdef VAN_TEST
  setupSerial();
#endif

#ifdef SIMCOM_SERIAL
  setupSerial();
  setupSimCom();
  handleSerialInput("S");
#endif
  
  // Activate kill switch ASAP if Enabled and Always On
  if (isAlwaysOn(KILLSWITCHENABLED_BOOL_1, KILLSWITCHSTART_CHAR_3, KILLSWITCHEND_CHAR_3)) {
    setKillSwitchAndDoorAlert(true);
  }

  setupSimCom();
  waitUntilNetworkConnected(300);
  checkForDeadMessages();

  updateClock();
  updateLastResetTime();
  
  setKillSwitchAndDoorAlert(isActive(KILLSWITCHENABLED_BOOL_1, KILLSWITCHSTART_CHAR_3, KILLSWITCHEND_CHAR_3));

  // If VT is powered on while door is open, the ISR will not fire. So we manually check here so we can send a warning message
  if (g_v_killSwitchAndDoorAlertActive && !digitalRead(DOOR_INTERRUPT_PIN))
    g_v_doorOpenedWhileDoorAlertActive = true;


  char ownerPhoneNumber[15];
  bool powerOn;
  EEPROM.get(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);
  EEPROM.get(POWERON_BOOL_1, powerOn);
  if (powerOn)
    sendSMS(ownerPhoneNumber, VT_VERSION);
}

void loop() {
#ifdef VAN_TEST
  checkSerialInput();
  flushSimCom();
#endif

  // 0 is good - we're connected to the cellular network
  if (g_SimComConnectionStatus == 0) {
    debugBlink(0,1);
    checkSMSInput();

    debugBlink(0,2);
    watchDogForResume();
    watchDogForKillSwitch();
    watchDogForGeofence();
    watchDogForTurnOffGPS();
    watchDogForReset();
  }
  // 1 is very bad - could not connect to SimCom module
  if (g_SimComConnectionStatus == 1) {

    // if kill switch is always on, turn on, otherwise this won't do anything
    watchDogForKillSwitch();

    // Delay 30 minutes and reset.  This loop takes about 7 seconds. 250 loops * 7 seconds = 30 minutes.
    for (int16_t i = 0; i < 250; i++) {
      delay(2000);
      debugBlink(3,g_SimComConnectionStatus);
      #ifdef VAN_TEST
        checkSerialInput();
        flushSimCom();
      #endif
    }
    resetSystem();
  }
  // > 1 is also bad - could not connect to cellular network, but we might have a valid clock
  if (g_SimComConnectionStatus > 1) {
    debugBlink(3,g_SimComConnectionStatus);
    delay(2000);

    debugBlink(0,2);
    watchDogForResume();
    watchDogForKillSwitch();
    watchDogForTurnOffGPS();
    watchDogForReset();
  }
}

void watchDogForReset() {
  int16_t currentTime = getTimePartInt(HOUR_INDEX) * 60 + getTimePartInt(MINUTE_INDEX);

  // Edge case: if simcom stops responding, we may get both times == 0.  
  // Restart.  It will then either start working or go to g_SimComConnectionStatus == 1
  if (currentTime == 0 && g_lastRestartTime == 0) {
    resetSystem();
    return;
  }

  // If this is VT-generated (geofence alert) those only get sent every 5 minutes
  // In the slim chance this is a response to an incoming command, we don't want to just keep retrying a million times
  if (g_totalFailedSendSMSAttempts == 3) {
    // gotta increment to the next number or we'll reset every loop...
    g_totalFailedSendSMSAttempts++;
    resetSystem();
    return;
  }
  if (g_totalFailedSendSMSAttempts == 6) {
    g_totalFailedSendSMSAttempts = 0;

    bool follow;
    EEPROM.get(GEOFENCEFOLLOW_BOOL_1, follow);
  
    // if "follow mode" is on, keep trying to communicate with owner!
    if (follow) {
      delay(60000);   //data type = unsigned long
      resetSystem();
      return;
    }

    g_SimComConnectionStatus = 2;
  }

  // 0 is connected to cell network
  if (g_SimComConnectionStatus == 0) {
    // if it's been 120 minutes, restart
    if (g_lastRestartTime <= currentTime && currentTime - g_lastRestartTime > 120) {
      resetSystem();
    }
    else if (g_lastRestartTime > currentTime && g_lastRestartTime - currentTime < 1320) {
      resetSystem();
    }
  } // > 0 is not connected to cell network
  else {
    // if it's been 30 minutes, restart
    if (g_lastRestartTime <= currentTime && currentTime - g_lastRestartTime > 30) {
      resetSystem();
    }
    else if (g_lastRestartTime > currentTime && g_lastRestartTime - currentTime < 1410) {
      resetSystem();
    }
  }
}

int8_t isClockValid() {
  // When the SimCom module is powered on, the Real Time Clock says 1980-01-01.
  // When AT+CFUN=1,1 command is send to SimCom module, Clock is NOT wiped out, which is nice.

  int8_t year = getTimePartInt(YEAR_INDEX);
  if (year > 79) return 2;  // default year is 1980.  This is very bad, clock is unusable
  if (year < 20) return 1;  // when we set the year based on gps time we set it to 2011 (hour is correct).  Clock is decent, but the actual time would be better.
  return 0;                 // clock is correct based on network time
}

void updateTimezone() {
  // timeStr INCLUDES QUOTES and looks like "21/04/08,00:45:16-20"
  char timeStr[23];
  char tzStr[4];

  getTime(timeStr);
  timeStr[21] = '\0';
  EEPROM.get(TIMEZONE_CHAR_4, tzStr);

  // 0 means strings are equivalent
  if (strcmp(tzStr, &timeStr[18])) {
    strcpy(tzStr, &timeStr[18]);    // You have to do this so the EEPROM.put() will work.  See https://forum.arduino.cc/t/passing-in-a-char-then-using-eeprom-put-fails/653670
    EEPROM.put(TIMEZONE_CHAR_4, tzStr);
  }
}

void updateClock() {
  // When the SimCom module is powered on, the real time clock says 1980-01-01.
  // When AT+CFUN=1,1 command is send to SimCom module, Clock is NOT wiped out.
  
  // If cellular network gives us the time, we're good...
  if (isClockValid() == 0) {
    // Good news is TZ will be updated automatically, so the "time" command is deprecated.
    updateTimezone();
    return;
  }

  // If not, we'll try to get network time using Network Time Protocol
  char tzOffsetStr[4];
  char dummyString[1];

  // tzOffsetStr == "-48".."+00".."+48"
  EEPROM.get(TIMEZONE_CHAR_4, tzOffsetStr);
  fona.enableNTPTimeSync(tzOffsetStr, dummyString, 1);

  // either we're == 0 due to NTP time or we're == 1 from the GPS. Either way, no reason to re-do using GPS time, let's just return
  if (isClockValid() < 2) {
    return;
  }

  // If not, we resort to GPS and the timezone from EEPROM

  // From SIM7000 AT command reference:
  //   String type(string should be included in quotation marks) value
  //   format is "yy/MM/dd,hh:mm:ss±zz", where characters indicate year (two last digits),
  //   month, day, hour, minutes, seconds and time zone (indicates the difference, expressed 
  //   in quarters of an hour, between the local time and GMT; range -47...+48). 
  //   E.g. 6th of May 2010, 00:01:52 GMT+2 hours equals to "10/05/06,00:01:52+08".

  // convert gpsTimeStr
  //    YYYYMMDDhhmmss.xxx
  // to simComTimeStr
  //    INCLUDING QUOTES "yy/MM/dd,hh:mm:ss±zz"
  // example
  //    INCLUDING QUOTES "21/12/31,10:11:12-22"
  char gpsTimeStr[19] = {0};
  if (getGPSTime(gpsTimeStr)) {

    // We don't care about the date, just the time
    // Put in 2011 because it signifies this clock is "kinda" valid, so next reset it'll hopefully update using NTP
    char simComTimeStr[23];
    strcpy_P(simComTimeStr, PSTR("\"11/11/11,"));

    int8_t gpsHourInt = getTimePartInt(8, gpsTimeStr);
    int8_t tzOffsetInt = atoi(tzOffsetStr) / 4;  // change -48..48 to -12..12
    int8_t localHourInt;
    char localHourStr[3];

    // SIM7000 keeps local time, SIM7500 keeps GMT!  That SUCKS!
    // add utc time + offset to get local time... with some caveats
    if (fona.type() == SIM7000) {
      if (gpsHourInt + tzOffsetInt > 23)
        localHourInt = gpsHourInt + tzOffsetInt - 24;
      else {
        if (gpsHourInt + tzOffsetInt < 0)
          localHourInt = gpsHourInt + tzOffsetInt + 24;
        else
          localHourInt = gpsHourInt + tzOffsetInt;
      }
    }  //SIM7500
    else {
      localHourInt = gpsHourInt;
    }

    itoa(localHourInt, localHourStr, 10);
    insertZero(localHourStr);

    simComTimeStr[10] = localHourStr[0];
    simComTimeStr[11] = localHourStr[1];

    simComTimeStr[12] = ':';
    simComTimeStr[13] = gpsTimeStr[10];
    simComTimeStr[14] = gpsTimeStr[11];
    simComTimeStr[15] = ':';
    simComTimeStr[16] = gpsTimeStr[12];
    simComTimeStr[17] = gpsTimeStr[13];

    simComTimeStr[18] = tzOffsetStr[0];
    simComTimeStr[19] = tzOffsetStr[1];
    simComTimeStr[20] = tzOffsetStr[2];

    simComTimeStr[21] = '\"';
    simComTimeStr[22] = '\0';

    fona.setTime(simComTimeStr);
  }

  // Very edgy case - SimCom won't respond to AT+CPMS? to get SMS messages, but GPS stuff WILL work.
  // Don't worry, we'll reset SimCom in about 30 min
  if (g_SimComConnectionStatus == 1 && isClockValid() < 2) {
    // set connection to "bad"
    g_SimComConnectionStatus = 2;
  }
}

void updateLastResetTime() {
  g_lastRestartTime = getTimePartInt(HOUR_INDEX) * 60 + getTimePartInt(MINUTE_INDEX);
}

void resetSystem() {
  debugPrintln(F("R"));
  debugBlink(0,3);
  setSimComFunctionality(true);

  setupSimCom();
  waitUntilNetworkConnected(120);
  updateClock();

  updateLastResetTime();  
}

void watchDogForTurnOffGPS() {
  // shut down GPS module after 20 minutes of inactivity to save power

  // if already off, return
  if (fona.GPSstatus() == 0) {
    return;
  }

  int16_t currentTime = getTimePartInt(HOUR_INDEX) * 60 + getTimePartInt(MINUTE_INDEX);

  // case 1 example: lastQuery = 5:10pm, current = 5:35pm
  if (g_lastGPSConnAttemptTime <= currentTime && currentTime - g_lastGPSConnAttemptTime > 20) {
    setGPS(false);
  }
  // case 2 example: lastQuery = 11:56pm (which == 1436), current = 12:25am (which == 25)
  if (g_lastGPSConnAttemptTime > currentTime && g_lastGPSConnAttemptTime - currentTime < 1420) {
    setGPS(false);
  }
}

void watchDogForResume() {
  // Resume after X minutes
  // X = 15 by default

  // if no pause is in effect, just return
  if (g_pauseStartedAt == -1) {
    return;
  }

  int16_t currentTime = getTimePartInt(HOUR_INDEX) * 60 + getTimePartInt(MINUTE_INDEX);

  // case 1 example: g_pauseStartedAt = 1:10pm, current = 1:30pm
  if (g_pauseStartedAt <= currentTime && currentTime - g_pauseStartedAt > g_pauseLengthInMinutes) {
    g_pauseStartedAt = -1;
    return;
  }
  // case 2 example: g_pauseStartedAt = 11:55pm, current = 12:20am
  if (g_pauseStartedAt > currentTime && g_pauseStartedAt - currentTime < (1440-g_pauseLengthInMinutes)) {
    g_pauseStartedAt = -1;
    return;
  }

  // we're still paused, don't send alerts
  setKillSwitchAndDoorAlert(false);
}

void watchDogForKillSwitch() {
  if (g_v_doorInterruptDebug) {
      debugPrintln(F("iD"));
      g_v_doorInterruptDebug = false;
  }

  if (g_v_killSwitchInterruptDebug) {
      debugPrintln(F("iK"));
      g_v_killSwitchInterruptDebug = false;
  }

  if (g_pauseStartedAt >= 0)
    return;

  setKillSwitchAndDoorAlert(isActive(KILLSWITCHENABLED_BOOL_1, KILLSWITCHSTART_CHAR_3, KILLSWITCHEND_CHAR_3));

  if (g_v_doorOpenedWhileDoorAlertActive) {
    char ownerPhoneNumber[15];
    EEPROM.get(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);

    sendSMS(ownerPhoneNumber, F("WARNING!\\\\nDoor opened!"));
    g_v_doorOpenedWhileDoorAlertActive = false;  // whether sendSMS() is successful or not, set to false so we don't endlessly retry sending (could be bad if vehicle is out of cell range)
  }

  if (g_v_startAttemptedWhileKillSwitchActive) {
    char ownerPhoneNumber[15];
    EEPROM.get(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);

    sendSMS(ownerPhoneNumber, F("WARNING!\\\\nStart attempted!"));
    g_v_startAttemptedWhileKillSwitchActive = false;  // whether sendSMS() is successful or not, set to false so we don't endlessly retry sending (could be bad if vehicle is out of cell range)
  }
}

bool watchDogForFollow(char* currentLat, char* currentLon, char* currentSpeed, char* currentDir) {
  bool follow;
  EEPROM.get(GEOFENCEFOLLOW_BOOL_1, follow);

  if (!follow)
    return false;

  // yeah, let's keep trying!
  g_lastGPSConnAttemptWorked = true;

  // do not do "if (getGPS...) then sendGeofence...)
  // if we can't get GPS, follow will send 0,0 so at least the user knows we're trying
  getGPSLatLonSpeedDir(currentLat, currentLon, currentSpeed, currentDir);
  sendGeofenceWarning(true, currentLat, currentLon, currentSpeed, currentDir);

  g_followMessageCount++;
  if (g_followMessageCount > 60) {
    g_followMessageCount = 0;
    EEPROM.put(GEOFENCEFOLLOW_BOOL_1, false);

    char ownerPhoneNumber[15];
    EEPROM.get(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);
    sendSMS(ownerPhoneNumber, F("Follow auto-disabled. Send 'follow' to re-enable"));  
  }
  else {
    bool usePlainSMS = false;
    EEPROM.get(USEPLAINSMS_BOOL_1, usePlainSMS);
  
    delay(40000);  // send a Follow message every minute or so
  }

  return true;
}

void watchDogForGeofence() {
  char currentLat[12];
  char currentLon[12];
  char currentSpeed[4];
  char currentDir[4];   // starts out between 0..359 degrees, i.e. "227" and ends up as direction, i.e. "NW"

  if (watchDogForFollow(currentLat, currentLon, currentSpeed, currentDir))
    return;

  if (g_pauseStartedAt >= 0)
    return;

  bool geofenceActive = isActive(GEOFENCEENABLED_BOOL_1, GEOFENCESTART_CHAR_3, GEOFENCEEND_CHAR_3);

  if (!geofenceActive) {
    g_lastGeofenceWarningMinute = -1;
    return;
  }

  int8_t currentMinuteInt = getTimePartInt(MINUTE_INDEX);

  // If we've sent a geofence warning...
  if (g_lastGeofenceWarningMinute != -1) {

    // ...wait to send the next one after 15 minutes have passed.  User can use follow mode if she wants rapid updates.

    // There are 2 cases:
    // A) current minute > last query minute, example lastQuery = 10, current = 30
    if (g_lastGeofenceWarningMinute <= currentMinuteInt && currentMinuteInt - g_lastGeofenceWarningMinute < 15) {
      return;
    }
  
    // B) current minute < last query minute, example lastQuery = 57, current = 15
    if (g_lastGeofenceWarningMinute > currentMinuteInt && g_lastGeofenceWarningMinute - currentMinuteInt > 45) {
      return;
    }
  }

  // send at most 12 warning messages (1 message every 15 minutes for 3 hours)... and then stop
  if (g_geofenceWarningCount > 11) {

    // send a message about auto-disabling the geofence warning messages... but only send it once.
    if (!g_geofenceWarningCountMessageSent) {
      char ownerPhoneNumber[15];
      EEPROM.get(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);
      sendSMS(ownerPhoneNumber, F("Warnings auto-disabled. Send any command to re-enable"));
      g_geofenceWarningCountMessageSent = true;
    }
    return;
  }

  if (getGPSLatLonSpeedDir(currentLat, currentLon, currentSpeed, currentDir) && outsideGeofence(currentLat, currentLon)) {
    sendGeofenceWarning(false, currentLat, currentLon, currentSpeed, currentDir);
    g_geofenceWarningCount++;
  }
}

void sendGeofenceWarning(bool follow, char* currentLat, char* currentLon, char* currentSpeed, char* currentDir) {
  char geofenceHomeLat[12];
  char geofenceHomeLon[12];
  char ownerPhoneNumber[15];
  EEPROM.get(GEOFENCEHOMELAT_CHAR_12, geofenceHomeLat);
  EEPROM.get(GEOFENCEHOMELON_CHAR_12, geofenceHomeLon);
  EEPROM.get(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);

  char message[150];    // longest string: "FENCE WARNING!___Current:___google.com/search?q=-30.209830,-097.764757___Home:___google.com/search?q=-52.4322115,-097.7869289___Dir: NW @ 0.0 kph_"

  if (follow)
    strcpy_P(message, PSTR("FOLLOW MODE"));
  else
    strcpy_P(message, PSTR("FENCE WARNING!"));

  strcat_P(message, PSTR("\\\\nCurrent:\\\\ngoogle.com/search?q="));
  strcat(message, currentLat);
  strcat_P(message, PSTR(","));
  strcat(message, currentLon);
  strcat_P(message, PSTR("\\\\nHome:\\\\ngoogle.com/search?q="));
  strcat(message, geofenceHomeLat);
  strcat_P(message, PSTR(","));
  strcat(message, geofenceHomeLon);
  strcat_P(message, PSTR("\\\\nDir: "));
  strcat(message, currentDir);
  strcat_P(message, PSTR(" @ "));
  strcat(message, currentSpeed);
  strcat_P(message, PSTR(" kph"));

  sendSMS(ownerPhoneNumber, message);
  // we only want to send this message the first time the geofence is broken
  if (g_lastGeofenceWarningMinute == -1 && !follow) {
    sendSMS(ownerPhoneNumber, F("Send 'follow' for rapid location updates, 'follow disable' to stop"));
  }

  g_lastGeofenceWarningMinute = getTimePartInt(MINUTE_INDEX);
}

void checkSMSInput() {
  int8_t numberOfSMSs = fona.getNumSMS();
  debugPrint(F("SMS: ")); debugPrintln(numberOfSMSs);

  if (numberOfSMSs < 1)
    return;

  // We set g_lastGPSConnAttemptWorked = TRUE every time we receive a new command in case the user has fixed the GPS connection issue.
  g_lastGPSConnAttemptWorked = true;

  // Ugh more globals :(
  // If they send ANY message, if we've auto-disabled geofence warnings, reset the geofence warnings so they start sending again
  g_geofenceWarningCount = 0;
  g_geofenceWarningCountMessageSent = false;

  // Let's DON'T do this.  If they're trying to track their van, sending extra messages is annoying, plus a warning will be sent soon anyway
  // g_lastGeofenceWarningMinute = getTimePartInt(MINUTE_INDEX);

  char smsSender[15];
  char smsValue[51];
  uint16_t smsValueLength;
  int8_t smssFound = 0;

  for (int8_t smsSlotNumber = 0; smssFound < numberOfSMSs; smsSlotNumber++) {
    // SimCom module has 10 slots
    if (smsSlotNumber >= 10)
      break;

    if (fona.getSMSSender(smsSlotNumber, smsSender, 15))
      smssFound++;
    else
      continue;

    fona.readSMS(smsSlotNumber, smsValue, 50, &smsValueLength);
    toLower(smsValue);

    cleanString(smsValue, ' ');
    debugPrint(F("IN : "));
    debugPrintln(smsValue);

    // exact match
    if (strcmp_P(smsValue, PSTR("unlock")) == 0) {
      if (handleUnlockReq(smsSender))
        deleteSMS(smsSlotNumber);
      continue;
    }

    // exact match
    if (strcmp_P(smsValue, PSTR("lock")) == 0) {
      if (handleLockReq(smsSender))
        deleteSMS(smsSlotNumber);
      continue;
    }

    // exact match || contains match
    if (strcmp_P(smsValue, PSTR("loc")) == 0 || strstr_P(smsValue, PSTR("locat"))) {
      if (handleLocReq(smsSender))
        deleteSMS(smsSlotNumber);
      continue;
    }

    // "contains" match
    if (strstr_P(smsValue, PSTR("both"))) {
      if (checkLockdownStatus(smsSender, smsValue, smsSlotNumber))
        continue;

      if (handleBothReq(smsSender, smsValue))
        deleteSMS(smsSlotNumber);
      continue;
    } 

    // "contains" match
#ifdef DOOR_ONLY_OPTION
    if (strstr_P(smsValue, PSTR("door"))) {
#else
    if (strstr_P(smsValue, PSTR("kill"))) {
#endif
      if (checkLockdownStatus(smsSender, smsValue, smsSlotNumber))
        continue;

      if (handleKillSwitchReq(smsSender, smsValue, false))
        deleteSMS(smsSlotNumber);
      continue;
    }

    // "contains" match
    if (strstr_P(smsValue, PSTR("fence"))) {
      if (checkLockdownStatus(smsSender, smsValue, smsSlotNumber))
        continue;

      if (handleGeofenceReq(smsSender, smsValue, false))
        deleteSMS(smsSlotNumber);
      continue;
    }

    // "contains" match
    if (strstr_P(smsValue, PSTR("follow"))) {
      if (handleFollowReq(smsSender, smsValue))
        deleteSMS(smsSlotNumber);
      continue;
    }

    // "contains" match
    if (strstr_P(smsValue, PSTR("owner"))) {
      if (handleOwnerReq(smsSender, smsValue))
        deleteSMS(smsSlotNumber);
      continue;
    }

    // "contains" match
    if (strstr_P(smsValue, PSTR("resum")) || strstr_P(smsValue, PSTR("unpaus"))) {
      if (handleResumeReq(smsSender))
        deleteSMS(smsSlotNumber);
      continue;
    }

    // "contains" match
    if (strstr_P(smsValue, PSTR("paus"))) {
      if (handlePauseReq(smsSender, smsValue))
        deleteSMS(smsSlotNumber);
      continue;
    }

    // "contains" match
    if (strstr_P(smsValue, PSTR("devkey set"))) {
      // special: we must pass the case-sensitive version of smsValue to handleDevKeyReq because the devKey is case sensitive
      fona.readSMS(smsSlotNumber, smsValue, 50, &smsValueLength);
      handleDevKeyReq(smsSender, smsValue);
      deleteSMS(smsSlotNumber);
      continue;
    }

    // "contains" match
    if (strstr_P(smsValue, PSTR("twilio set"))) {
      handleTwilioReq(smsSender, smsValue);
      deleteSMS(smsSlotNumber);
      continue;
    }

    // exact match
    if (strcmp_P(smsValue, PSTR("status")) == 0) {
      if (handleStatusReq(smsSender))
        deleteSMS(smsSlotNumber);
      continue;
    }

    // exact match
    if (strcmp_P(smsValue, PSTR("commands")) == 0) {
      if (handleCommandsReq(smsSender))
        deleteSMS(smsSlotNumber);
      continue;
    }

    // "contains" match
    if (strstr_P(smsValue, PSTR("usesms"))) {
      handleUseSMSReq(smsSender, smsValue);
      deleteSMS(smsSlotNumber);
      continue;
    }

    // exact match
    if (strcmp_P(smsValue, PSTR("deleteallmessages")) == 0) {
      fona.deleteAllSMS();
      return;  // notice this is RETURN not continue!
    }

    // "contains" match
    if (strstr_P(smsValue, PSTR("poweron"))) {
      if (strstr_P(smsValue, PSTR("ena"))) {
        EEPROM.put(POWERON_BOOL_1, true);
        sendSMS(smsSender, F("Ok"));
      } else if (strstr_P(smsValue, PSTR("disa"))) {
        EEPROM.put(POWERON_BOOL_1, false);
        sendSMS(smsSender, F("Ok"));
      }
      deleteSMS(smsSlotNumber);
      continue;
    }

    // exact match first character
    if (smsValue[0] == '~') {
      // special: we must pass the case-sensitive version of smsValue to handleATCommandReq because the AT command could be case sensitive
      fona.readSMS(smsSlotNumber, smsValue, 50, &smsValueLength);
      handleATCommandReq(smsSender, &smsValue[1]);
      deleteSMS(smsSlotNumber);
      continue;
    }

    // default
    if (handleUnknownReq(smsSender))
      deleteSMS(smsSlotNumber);
  }
}

bool checkLockdownStatus(char* smsSender, char* smsValue, int8_t smsSlotNumber) {
  char message[166];        // longest string: "Lockdown Enabled. Try 'unlock' before updating fence or kill___Radius: 12500 feet___Home: google.com/search?q=-30.209794,-097.764715______TheVanTracker.com/h10_"
  char geofenceHomeLat[12];
  char geofenceHomeLon[12];
  char geofenceRadius[7];
  bool lockdownEnabled;

  // check if lockdown is ENabled
  EEPROM.get(LOCKDOWNENABLED_BOOL_1, lockdownEnabled);  

  if (lockdownEnabled && !strstr_P(smsValue, PSTR("stat"))) {
    EEPROM.get(GEOFENCEHOMELAT_CHAR_12, geofenceHomeLat);
    EEPROM.get(GEOFENCEHOMELON_CHAR_12, geofenceHomeLon);
    EEPROM.get(GEOFENCERADIUS_CHAR_7, geofenceRadius);
    
#ifdef DOOR_ONLY_OPTION
    strcpy_P(message, PSTR("Lockdown Enabled. Try 'unlock' before updating fence or door\\\\nRadius: "));
#else
    strcpy_P(message, PSTR("Lockdown Enabled. Try 'unlock' before updating fence or kill\\\\nRadius: "));
#endif
    strcat(message, geofenceRadius);
    strcat_P(message, STR_HOME);
    strcat(message, geofenceHomeLat);
    strcat_P(message, PSTR(","));
    strcat(message, geofenceHomeLon);

    sendSMS(smsSender, F("TheVanTracker.com/h7"));
    if (sendSMS(smsSender, message))
      deleteSMS(smsSlotNumber);
    return true;
  }
  return false;
}
      
bool handleLockReq(char* smsSender) {

  char message[125];
  char geofenceHomeLat[12];
  char geofenceHomeLon[12];
  char geofenceRadius[7];
  bool lockdownEnabled;

  strcpy_P(message, PSTR("Lockdown:"));
  
  // check if lockdown is ALREADY ENabled
  EEPROM.get(LOCKDOWNENABLED_BOOL_1, lockdownEnabled);
  EEPROM.get(GEOFENCERADIUS_CHAR_7, geofenceRadius);

  if (lockdownEnabled) {
    strcat_P(message, PSTR(" Already"));
    EEPROM.get(GEOFENCEHOMELAT_CHAR_12, geofenceHomeLat);
    EEPROM.get(GEOFENCEHOMELON_CHAR_12, geofenceHomeLon);
  }
  else {

    if (!getGPSLatLon(geofenceHomeLat, geofenceHomeLon)) {
      strcpy_P(message, STR_UNABLE_GPS);
      return sendSMS(smsSender, message);
    }

    bool geofenceEnabled;
    char geofenceStart[3];
    char geofenceEnd[3];
    bool killSwitchEnabled;
    char killSwitchStart[3];
    char killSwitchEnd[3];
  
    // store primary variables (except Follow) in saved state variables
    EEPROM.get(GEOFENCEENABLED_BOOL_1, geofenceEnabled);
    EEPROM.put(GEOFENCEENABLED_BOOL_SAVED_1, geofenceEnabled);
    EEPROM.get(GEOFENCEHOMELAT_CHAR_12, geofenceHomeLat);
    EEPROM.put(GEOFENCEHOMELAT_CHAR_SAVED_12, geofenceHomeLat);
    EEPROM.get(GEOFENCEHOMELON_CHAR_12, geofenceHomeLon);
    EEPROM.put(GEOFENCEHOMELON_CHAR_SAVED_12, geofenceHomeLon);
    EEPROM.get(GEOFENCESTART_CHAR_3, geofenceStart);
    EEPROM.put(GEOFENCESTART_CHAR_SAVED_3, geofenceStart);
    EEPROM.get(GEOFENCEEND_CHAR_3, geofenceEnd);
    EEPROM.put(GEOFENCEEND_CHAR_SAVED_3, geofenceEnd);
    EEPROM.get(KILLSWITCHENABLED_BOOL_1, killSwitchEnabled);
    EEPROM.put(KILLSWITCHENABLED_BOOL_SAVED_1, killSwitchEnabled);
    EEPROM.get(KILLSWITCHSTART_CHAR_3, killSwitchStart);
    EEPROM.put(KILLSWITCHSTART_CHAR_SAVED_3, killSwitchStart);
    EEPROM.get(KILLSWITCHEND_CHAR_3, killSwitchEnd);
    EEPROM.put(KILLSWITCHEND_CHAR_SAVED_3, killSwitchEnd);
  
    // set primary variables (except Follow) to enabled, always on, etc. and use radius = 500
    // and set fence Home to current location

    EEPROM.put(GEOFENCEENABLED_BOOL_1, true);
    getGPSLatLon(geofenceHomeLat, geofenceHomeLon);
    EEPROM.put(GEOFENCEHOMELAT_CHAR_12, geofenceHomeLat);
    EEPROM.put(GEOFENCEHOMELON_CHAR_12, geofenceHomeLon);
    EEPROM.put(GEOFENCESTART_CHAR_3, "00");
    EEPROM.put(GEOFENCEEND_CHAR_3, "00");
    EEPROM.put(KILLSWITCHENABLED_BOOL_1, true);
    EEPROM.put(KILLSWITCHSTART_CHAR_3, "00");
    EEPROM.put(KILLSWITCHEND_CHAR_3, "00");
  
    // set lockdown variable ON
    EEPROM.put(LOCKDOWNENABLED_BOOL_1, true);
  }

  // send SMS with new geofence home
  strcat_P(message, PSTR(" Enabled\\\\nRadius: "));
  strcat(message, geofenceRadius);
  
  // Keep this saying "HOME" even though it's a temporary home
  // Reason being - if someone steals the van when it's locked, the WARNING will say HOME and CURRENT
  strcat_P(message, STR_HOME);
  strcat(message, geofenceHomeLat);
  strcat_P(message, PSTR(","));
  strcat(message, geofenceHomeLon);

  return sendSMS(smsSender, message);
}

bool handleUnlockReq(char* smsSender) {

  char message[126];
  bool lockdownEnabled;

  // check if lockdown is ALREADY DISabled
  EEPROM.get(LOCKDOWNENABLED_BOOL_1, lockdownEnabled);

  if (lockdownEnabled) {
    bool geofenceEnabled;
    char geofenceHomeLat[12];
    char geofenceHomeLon[12];
    char geofenceStart[3];
    char geofenceEnd[3];
    bool killSwitchEnabled;
    char killSwitchStart[3];
    char killSwitchEnd[3];
  
    // put saved state variables (except Follow) back into primary variables
    EEPROM.get(GEOFENCEENABLED_BOOL_SAVED_1, geofenceEnabled);
    EEPROM.put(GEOFENCEENABLED_BOOL_1, geofenceEnabled);
    EEPROM.get(GEOFENCEHOMELAT_CHAR_SAVED_12, geofenceHomeLat);
    EEPROM.put(GEOFENCEHOMELAT_CHAR_12, geofenceHomeLat);
    EEPROM.get(GEOFENCEHOMELON_CHAR_SAVED_12, geofenceHomeLon);
    EEPROM.put(GEOFENCEHOMELON_CHAR_12, geofenceHomeLon);
    EEPROM.get(GEOFENCESTART_CHAR_SAVED_3, geofenceStart);
    EEPROM.put(GEOFENCESTART_CHAR_3, geofenceStart);
    EEPROM.get(GEOFENCEEND_CHAR_SAVED_3, geofenceEnd);
    EEPROM.put(GEOFENCEEND_CHAR_3, geofenceEnd);
    EEPROM.get(KILLSWITCHENABLED_BOOL_SAVED_1, killSwitchEnabled);
    EEPROM.put(KILLSWITCHENABLED_BOOL_1, killSwitchEnabled);
    EEPROM.get(KILLSWITCHSTART_CHAR_SAVED_3, killSwitchStart);
    EEPROM.put(KILLSWITCHSTART_CHAR_3, killSwitchStart);
    EEPROM.get(KILLSWITCHEND_CHAR_SAVED_3, killSwitchEnd);
    EEPROM.put(KILLSWITCHEND_CHAR_3, killSwitchEnd);
  
    // set lockdown variable OFF
    EEPROM.put(LOCKDOWNENABLED_BOOL_1, false);
  }

  return sendSMS(smsSender, F("Lockdown: Disabled"));
}

bool handleStatusReq(char* smsSender) {
  int8_t rssi;
  char rssiStr[4];
  char currentTimeStr[23];
  char message[148];          // longest string: "Owner: 1235415141272___Lockdown: Disabled___Fence: enabled 00-00___Kill: Enabled 00-00___Signal: 16___System Time: '21/07/19,02:52:06-28'_"
  char hour[3];

  char ownerPhoneNumber[15];
  EEPROM.get(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);
  bool fence;
  EEPROM.get(GEOFENCEENABLED_BOOL_1, fence);
  bool kill;
  EEPROM.get(KILLSWITCHENABLED_BOOL_1, kill);
  bool lockdown;
  EEPROM.get(LOCKDOWNENABLED_BOOL_1, lockdown);
  
  rssi = fona.getRSSI();
  itoa(rssi, rssiStr, 10);

  getTime(currentTimeStr);

  strcpy_P(message, PSTR("Owner: "));
  strcat(message, &ownerPhoneNumber[1]);  // strip + from phone number "+15559998888"

  if (g_pauseStartedAt >= 0)
    strcat_P(message, PSTR("\\\\nPaused"));

  else {
    if (lockdown)
      strcat_P(message, PSTR("\\\\nLockdown: Enabled"));
    else {
      strcat_P(message, PSTR("\\\\nLockdown: Disabled"));
  
      if (fence) {
        strcat_P(message, PSTR("\\\\nFence: Enabled "));
        EEPROM.get(GEOFENCESTART_CHAR_3, hour);
        strcat(message, hour);
        strcat_P(message, PSTR("-"));
        EEPROM.get(GEOFENCEEND_CHAR_3, hour);
        strcat(message, hour);
      }
      else
        strcat_P(message, PSTR("\\\\nFence: Disabled"));
    
      if (kill) {
  #ifdef DOOR_ONLY_OPTION
        strcat_P(message, PSTR("\\\\nDoor: Enabled "));
  #else
        strcat_P(message, PSTR("\\\\nKill: Enabled "));
  #endif
  
        EEPROM.get(KILLSWITCHSTART_CHAR_3, hour);
        strcat(message, hour);
        strcat_P(message, PSTR("-"));
        EEPROM.get(KILLSWITCHEND_CHAR_3, hour);
        strcat(message, hour);
      }
      else
  #ifdef DOOR_ONLY_OPTION
        strcat_P(message, PSTR("\\\\nDoor: Disabled"));
  #else
        strcat_P(message, PSTR("\\\\nKill: Disabled"));
  #endif
    }
  }

  strcat_P(message, PSTR("\\\\nSignal: "));
  strcat(message, rssiStr);
  strcat_P(message, PSTR("\\\\nSystem Time: "));
  strcat(message, currentTimeStr);
  return sendSMS(smsSender, message);
}

bool handleLocReq(char* smsSender) {
  char message[83];
  char latitude[12];
  char longitude[12];
  char speed[4];
  char dir[4];   // starts out between 0..359 degrees, i.e. "227" and ends up as direction, i.e. "NW"

  if (getGPSLatLonSpeedDir(latitude, longitude, speed, dir)) {
    strcpy_P(message, PSTR("google.com/search?q="));
    strcat(message, latitude);
    strcat_P(message, PSTR(","));
    strcat(message, longitude);
    strcat_P(message, PSTR("\\\\nDir: "));
    strcat(message, dir);
    strcat_P(message, PSTR(" @ "));
    strcat(message, speed);
    strcat_P(message, PSTR(" kph"));
  } else {
    strcpy_P(message, STR_UNABLE_GPS);
  }
  return sendSMS(smsSender, message);
}

bool handleUseSMSReq(char* smsSender, char* smsValue) {
  if (strstr_P(smsValue, PSTR("plain"))) {
    EEPROM.put(USEPLAINSMS_BOOL_1, true);
    sendSMS(smsSender, F("Ok"));
  }
  if (strstr_P(smsValue, PSTR("ip"))) {
    EEPROM.put(USEPLAINSMS_BOOL_1, false);
    sendSMS(smsSender, F("Ok"));
  }
}

bool handleFollowReq(char* smsSender, char* smsValue) {
  // if someone's car was just stolen, just plain "follow" is probably plenty to type
  if (strstr_P(smsValue, PSTR("enable")) || strcmp_P(smsValue, PSTR("follow")) == 0) {
    EEPROM.put(GEOFENCEFOLLOW_BOOL_1, true);
    g_followMessageCount = 0;
    return true;
    // save a little memory/data
    //return sendSMS(smsSender, F("Follow: Enabled"));
  }
  if (strstr_P(smsValue, PSTR("disable"))) {
    EEPROM.put(GEOFENCEFOLLOW_BOOL_1, false);
    return true;
    // save a little memory/data
    //return sendSMS(smsSender, F("Follow: Disabled"));
  }
  sendSMS(smsSender, F("TheVanTracker.com/h9"));
  return sendSMS(smsSender, F("Try 'follow' plus:\\\\nenable\\\\ndisable"));
}

bool handleBothReq(char* smsSender, char* smsValue) {
  return handleKillSwitchReq(smsSender, smsValue, true) && handleGeofenceReq(smsSender, smsValue, true);
}

bool handleKillSwitchReq(char* smsSender, char* smsValue, bool alternateSMSOnFailure) {
  char message[100];

  bool validMessage = false;
  bool killSwitchEnabled;
  char killSwitchStart[3];
  char killSwitchEnd[3];
  EEPROM.get(KILLSWITCHSTART_CHAR_3, killSwitchStart);
  EEPROM.get(KILLSWITCHEND_CHAR_3, killSwitchEnd);

  validMessage = setEnableAndHours(smsValue, KILLSWITCHENABLED_BOOL_1, KILLSWITCHSTART_CHAR_3, KILLSWITCHEND_CHAR_3, killSwitchEnabled, killSwitchStart, killSwitchEnd);

  if (validMessage || strstr_P(smsValue, PSTR("status"))) {
#ifdef DOOR_ONLY_OPTION
    if (killSwitchEnabled)
      strcpy_P(message, PSTR("Door: Enabled\\\\nHours: "));
    else
      strcpy_P(message, PSTR("Door: Disabled\\\\nHours: "));
#else
    if (killSwitchEnabled)
      strcpy_P(message, PSTR("Kill: Enabled\\\\nHours: "));
    else
      strcpy_P(message, PSTR("Kill: Disabled\\\\nHours: "));
#endif

    strcat(message, killSwitchStart);
    strcat_P(message, PSTR("-"));
    strcat(message, killSwitchEnd);
    if (strcmp(killSwitchStart, killSwitchEnd) == 0) {  // if start time == end time  
      strcat_P(message, PSTR(" (always on)"));
    }

    return sendSMS(smsSender, message);
  }
  else {
    // This whole section is not good because it's tied to how handleGeofenceReq() works in its final ELSE clause.
    // "both" was an afterthought, so this section was a retrofit.
    // The point is, we handle responding to invalid messages for the "both" command here in handleKillSwitchReq()

    strcpy_P(message, PSTR("Try '"));
    if (alternateSMSOnFailure) {
      strcat_P(message, PSTR("both"));
    }
    else {
#ifdef DOOR_ONLY_OPTION
      strcat_P(message, PSTR("door"));
#else
      strcat_P(message, PSTR("kill"));
#endif
    }

    strcat_P(message, PSTR("' plus:\\\\nenable\\\\ndisable\\\\nstatus\\\\nhours 23 7 (11pm-7am)"));

    if (alternateSMSOnFailure) {
      sendSMS(smsSender, F("TheVanTracker.com/h5"));
    }
    else {
#ifdef DOOR_ONLY_OPTION
      sendSMS(smsSender, F("TheVanTracker.com/h4"));
#else
      sendSMS(smsSender, F("TheVanTracker.com/h3"));
#endif
    }
    return sendSMS(smsSender, message);
  }
}

bool handleGeofenceReq(char* smsSender, char* smsValue, bool alternateSMSOnFailure) {
  char message[139] = {0};    // longest string "Fence: Disabled___Hours: 23-23 (always on)___Radius: 11500 feet___Home: google.com/search?q=-52.4322115,-110.7869289_"

  bool validMessage = false;
  bool geofenceEnabled;
  char geofenceStart[3];
  char geofenceEnd[3];
  char geofenceRadius[7];
  char geofenceHomeLat[12];
  char geofenceHomeLon[12];
  EEPROM.get(GEOFENCESTART_CHAR_3, geofenceStart);
  EEPROM.get(GEOFENCEEND_CHAR_3, geofenceEnd);
  EEPROM.get(GEOFENCERADIUS_CHAR_7, geofenceRadius);
  EEPROM.get(GEOFENCEHOMELAT_CHAR_12, geofenceHomeLat);
  EEPROM.get(GEOFENCEHOMELON_CHAR_12, geofenceHomeLon);

  validMessage = setEnableAndHours(smsValue, GEOFENCEENABLED_BOOL_1, GEOFENCESTART_CHAR_3, GEOFENCEEND_CHAR_3, geofenceEnabled, geofenceStart, geofenceEnd);  
  
  if (strstr_P(smsValue, PSTR("fence radius "))) {
    geofenceRadius[0] = '\0';
    if (getNumberFromString(smsValue, geofenceRadius, 7)) {
      EEPROM.put(GEOFENCERADIUS_CHAR_7, geofenceRadius);
      validMessage = true;
    }
  }
  if (strstr_P(smsValue, PSTR("fence home"))) {
    if (getGPSLatLon(geofenceHomeLat, geofenceHomeLon)) {
      EEPROM.put(GEOFENCEHOMELAT_CHAR_12, geofenceHomeLat);
      EEPROM.put(GEOFENCEHOMELON_CHAR_12, geofenceHomeLon);
      validMessage = true;
    } else {
      strcpy_P(message, STR_UNABLE_GPS);
      return sendSMS(smsSender, message);
    }
  }

  // reset this so the "follow" message will be sent when the fence is broken
  g_lastGeofenceWarningMinute = -1;

  if (validMessage || strstr_P(smsValue, PSTR("status"))) {
    if (geofenceEnabled)
      strcpy_P(message, PSTR("Fence: Enabled\\\\nHours: "));
    else
      strcpy_P(message, PSTR("Fence: Disabled\\\\nHours: "));

    strcat(message, geofenceStart);
    strcat_P(message, PSTR("-"));
    strcat(message, geofenceEnd);
    if (strcmp(geofenceStart, geofenceEnd) == 0) {  // if start time == end time  
      strcat_P(message, PSTR(" (always on)"));
    }
    strcat_P(message, PSTR("\\\\nRadius: "));
    strcat(message, geofenceRadius);
    strcat_P(message, STR_HOME);
    strcat(message, geofenceHomeLat);
    strcat_P(message, PSTR(","));
    strcat(message, geofenceHomeLon);

    return sendSMS(smsSender, message);
  }
  else {
    // This whole section is not good because it's tied to how handleKillSwitchReq() works in its final ELSE clause.
    // "both" was an afterthought, so this section was a retrofit.
    // The point is, we handle responding to invalid messages for the "both" command not here, but up in handleKillSwitchReq()

    if (alternateSMSOnFailure) {
      return true;
    }
    else {
      sendSMS(smsSender, F("TheVanTracker.com/h2"));
      return sendSMS(smsSender, F("Try 'fence' plus:\\\\nenable\\\\ndisable\\\\nstatus\\\\nhours 23 7 (11pm-7am)\\\\nhome (uses current loc)\\\\nradius 500 (500 feet)"));
    }
  }
}

bool handleOwnerReq(char* smsSender, char* smsValue) {
  char message[147] = {0};
  char ownerPhoneNumber[15] = {0};

  strcpy_P(message, PSTR("Owner: "));

  // set owner number
  if (strstr_P(smsValue, PSTR("owner set"))) {
    // If number is found in the SMS,
    if (getNumberFromString(smsValue, ownerPhoneNumber, 15))
      addPlusToPhoneNumber(ownerPhoneNumber);
    else
      strcpy(ownerPhoneNumber, smsSender);
  }

  // "+15554443333"
  if (strlen(ownerPhoneNumber) > 11) {
    EEPROM.put(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);
    strcat(message, &ownerPhoneNumber[1]);
  }
  else  // just respond with current owner number
  {
    EEPROM.get(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);
    strcat(message, &ownerPhoneNumber[1]);
    strcat_P(message, PSTR("\\\\nTry 'owner set' plus:\\\\nphone number WITH country code, or omit number to use your phone's number"));
    sendSMS(smsSender, F("TheVanTracker.com/h10"));
  }

  return sendSMS(smsSender, message);
}

bool handlePauseReq(char* smsSender, char* smsValue) {
  char pauseLengthInMinutes[4] = {};
  
  if (getNumberFromString(smsValue, pauseLengthInMinutes, 4)) {
    g_pauseLengthInMinutes = atoi(pauseLengthInMinutes);
  } else {
    g_pauseLengthInMinutes = 15;
  }

  g_pauseStartedAt = getTimePartInt(HOUR_INDEX) * 60 + getTimePartInt(MINUTE_INDEX);
  return sendSMS(smsSender, F("Paused"));
}

bool handleResumeReq(char* smsSender) {
  g_pauseStartedAt = -1;
  return sendSMS(smsSender, F("Resumed"));
}

void handleDevKeyReq(char* smsSender, char* smsValue) {

  // special: smsValue is still case-sensitive.
  char devKey[9];
  if (!getOccurrenceInDelimitedString(smsValue, devKey, 3, ' ', 8))   // max_length
    return;

  if (strlen(devKey) < 8)
    return;

  EEPROM.put(DEVKEY_CHAR_9, devKey);
  sendSMS(smsSender, F("Ok"));
}

void handleTwilioReq(char* smsSender, char* smsValue) {
  char twilioPhoneNumber[12];
  if (!getOccurrenceInDelimitedString(smsValue, twilioPhoneNumber, 3, ' ', 11))   // max_length
    return;

  if (strlen(twilioPhoneNumber) < 11)
    return;

  EEPROM.put(TWILIOPHONENUMBER_CHAR_12, twilioPhoneNumber);
  sendSMS(smsSender, F("Ok"));
}

bool handleCommandsReq(char* smsSender) {
  sendSMS(smsSender, F("TheVanTracker.com/help"));
#ifdef DOOR_ONLY_OPTION
  return sendSMS(smsSender, F("Commands:\\\\nstatus\\\\nfence\\\\ndoor\\\\nboth\\\\nlock\\\\nunlock\\\\npause\\\\nresume\\\\nloc\\\\nfollow\\\\nowner\\\\npoweron"));
#else
  return sendSMS(smsSender, F("Commands:\\\\nstatus\\\\nfence\\\\nkill\\\\nboth\\\\nlock\\\\nunlock\\\\npause\\\\nresume\\\\nloc\\\\nfollow\\\\nowner\\\\npoweron"));
#endif
}

void handleATCommandReq(char* smsSender, char* smsValue) {
  // This is for executing arbitrary AT commands.
  // if there are " chars, you have to escape them. Example messages:
  //    ~at+cgdcont=1,\"IP\",\"hologram\"
  //    ~at+cops=1,1,\"AT&T\" // AT&T - manual/force connect
  //    ~at+cops=1,2,310410   // AT&T - manual/force connect
  //    ~at+cops=4,1,\"AT&T\" // AT&T - automatic connect
  //    ~at+cops=4,2,310410   // AT&T - automatic connect
  //    ~at+cops=4,2,310260   // T-Mobile
  //
  // Response:
  //    <mode>[,<format>,<operator>[,< AcT>]]
  //    <mode>
  //      0 – automatic
  //      1 – manual
  //      2 – force deregister
  //      3 – set only <format>
  //      4 – manual/automatic
  //      5 – manual,but do not modify the network selection mode(e.g GSM,WCDMA) after module resets.
  //    <format>
  //      0 – long format alphanumeric <oper>
  //      1 – short format alphanumeric <oper>
  //      2 – numeric <oper>
  //    <operator>
  //      "AT&T", etc.
  //    <AcT> Access technology selected
  //      0 – GSM
  //      1 – GSM Compact
  //      2 – UTRAN
  //      7 – EUTRAN
  //      8 – CDMA/HDR
  
  // special: we must pass the case-sensitive version of smsValue to handleATCommandReq because the AT command could be case sensitive
  char response[141];
  fona.executeATCommand(smsValue, response, 140);
  removeNonAlphaNumChars(response);
  sendSMS(smsSender, response);
}

bool handleUnknownReq(char* smsSender) {
  char ownerPhoneNumber[15];
  EEPROM.get(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);

  // if they're not the owner, don't send them the commands, just return true so their msg will be deleted
  if (strcmp(smsSender, ownerPhoneNumber) == 0)
    return handleCommandsReq(smsSender);
  else
    return true;
}


////////////////////////////////
//GPS

bool setGPS(bool tf) {
  // turns SimCom GPS on or off (don't waste power)
  if (!tf) {
    if (fona.enableGPS(false)) {
      return true;
    }
    return false;
  }

  // If it isn't getting a GPS fix, do not try for the next 60 minutes (this is to save power).
  // FYI: We set g_lastGPSConnAttemptWorked = TRUE every time we receive a new command in case the user has fixed the GPS connection issue.
  if (!g_lastGPSConnAttemptWorked) {
    
    //////////////////////////////////////////////////////////////////////////////
    // DO NOT make a generic method for this!
    // Notice the > and < are NOT the same as in watchdogForTurnOffGPS()
    //////////////////////////////////////////////////////////////////////////////
    int16_t currentTime = getTimePartInt(HOUR_INDEX) * 60 + getTimePartInt(MINUTE_INDEX);

    if (g_lastGPSConnAttemptTime <= currentTime && currentTime - g_lastGPSConnAttemptTime < 60) {
      return false;
    }
    if (g_lastGPSConnAttemptTime > currentTime && g_lastGPSConnAttemptTime - currentTime > 1380) {
      return false;
    }
  }

  g_lastGPSConnAttemptTime = getTimePartInt(HOUR_INDEX) * 60 + getTimePartInt(MINUTE_INDEX);

  // -1 = error querying GPS
  //  0 = GPS off
  //  1 = no GPS fix
  //  2 = 2D fix
  //  3 = 3D fix
  if (fona.GPSstatus() >= 2) {
    g_lastGPSConnAttemptWorked = true;
    return true;
  }

  // Keep trying to get a valid (non-error) response. Maybe we should turn off/on?
  for (int8_t i = 1; i < 30; i++) {
    if (fona.GPSstatus() >= 0) {
      break;
    }
    delay(2000);
  }

  // error, give up
  if (fona.GPSstatus() < 0) {
    debugBlink(1,5);
    g_lastGPSConnAttemptWorked = false;
    fona.enableGPS(false);
    return false;
  }

  // turn on
  fona.enableGPS(true);
  delay(4000);

  // wait up to 90s to get GPS fix
  for (int8_t j = 0; j < 23; j++) {
    if (fona.GPSstatus() >= 2) {
      debugBlink(1,8);

      // I really hate to do this, but the first GPS response is sometimes WAY off (> 200 feet) and you get a geofence warning...
      // We have to sendRaw() because if we call getGPS we're calling the function that called this function.
      if (fona.type() == SIM7000) {
        sendRawCommand(F("AT+CGNSINF"));
        delay(3000);
        sendRawCommand(F("AT+CGNSINF"));
      } else {
        sendRawCommand(F("AT+CGNSSINFO"));
        delay(3000);
        sendRawCommand(F("AT+CGNSSINFO"));
      }
      g_lastGPSConnAttemptWorked = true;
      return true;
    }
    debugBlink(0,7);
    delay(1000);
  }

  // no fix, give up
  debugBlink(1,7);
  g_lastGPSConnAttemptWorked = false;
  fona.enableGPS(false);
  return false;
}

void getDirFromDegrees(char* degrees) {
  int16_t i = atoi(degrees);
  if (i > 337 || i < 23)
    strcpy_P(degrees, PSTR("N"));
  else if (i < 67)
    strcpy_P(degrees, PSTR("NE"));
  else if (i < 112)
    strcpy_P(degrees, PSTR("E"));
  else if (i < 157)
    strcpy_P(degrees, PSTR("SE"));
  else if (i < 202)
    strcpy_P(degrees, PSTR("S"));
  else if (i < 247)
    strcpy_P(degrees, PSTR("SW"));
  else if (i < 292)
    strcpy_P(degrees, PSTR("W"));
  else // (i < 337)
    strcpy_P(degrees, PSTR("NW"));
}

bool getGPSLatLon(char* latitude, char* longitude) {
  return getGPSLatLonSpeedDir(latitude, longitude, NULL, NULL);
}
  
bool getGPSLatLonSpeedDir(char* latitude, char* longitude, char* speed, char* dir) {
  char gpsString[120];
  char NS[2];   // char[] because we call getOccurrenceInDelimitedString().  I guess we could have used a char and &NS in the call.
  char EW[2];
  char tempLatLonBugStr[7];  // see Bug fix for SIM7500 farther down

  if (setGPS(true)){
    // full string SIM7000:
    //    1,1,20190913060459.000,30.213823,-97.782017,204.500,1.87,90.1,1,,1.2,1.5,0.9,,11,6,,,39,,
    // full string SIM7500:
    //    2,06,00,00,3012.586830,N,09745.886045,W,080421,220736.0,183.9,0.0,227.5,1.3,1.0,0.8
    for (int8_t i = 0; i < 10; i++) {
      fona.getGPS(0, gpsString, 120);

      if (fona.type() == SIM7000) {
        getOccurrenceInDelimitedString(gpsString, latitude, 4, ',', 11);
        getOccurrenceInDelimitedString(gpsString, longitude, 5, ',', 11);
      }
      else {
        getOccurrenceInDelimitedString(gpsString, latitude, 5, ',', 11);
        getOccurrenceInDelimitedString(gpsString, NS, 6, ',');
        getOccurrenceInDelimitedString(gpsString, longitude, 7, ',', 11);
        getOccurrenceInDelimitedString(gpsString, EW, 8, ',');

        convertDegreesToDecimal(latitude, NS[0]);
        convertDegreesToDecimal(longitude, EW[0]);

        // Bug fix for SIM7500
        // We have seen errors where the longitude is something like -097.000000 which I must assume means the GPS chip gave us a string like 3012.586830,N,09745.000000,W
        //   so we will try to detect values such as -097.000000 or 30.000000 and throw out those values.
        //   We could make convertDegreesToDecimal() do this work for us, but it's not really it's problem, it's our problem
        //   Ah ha!  Saw this bug in practice. 2 example bad GPS strings.  Hardware bug, as suspected:
        //     2,07,00,00,3012.588946,N,097,W,130721,003145.0,192.6,0.0,123.3,1.0,0.7,0.7
        //     2,07,00,00,3012.587635,N,097,
        getOccurrenceInDelimitedString(latitude, tempLatLonBugStr, 2, '.', 6);
        if (atof(tempLatLonBugStr) == 0.0) {
          delay(2500);
          continue;
        }
        getOccurrenceInDelimitedString(longitude, tempLatLonBugStr, 2, '.', 6);
        if (atof(tempLatLonBugStr) == 0.0) {
          delay(2500);
          continue;
        }
      }

      if (speed != NULL) {
        if (fona.type() == SIM7000) {
          getOccurrenceInDelimitedString(gpsString, speed, 7, ',', 3);
          getOccurrenceInDelimitedString(gpsString, dir, 8, ',', 3);
        }
        else {
          getOccurrenceInDelimitedString(gpsString, speed, 12, ',', 3);
          getOccurrenceInDelimitedString(gpsString, dir, 13, ',', 3);
        }
        getDirFromDegrees(dir);
      }

      // change "75." to "75"
      if (speed[2] == '.')
        speed[2] = '\0';

      // Bug fix #2
      // We have see errors where the lat,long come back as garbage like "9,43"
      // Leave the != NULL in there in case the '.' is at the 0th position, which I think is valid
      if (strlen(latitude) > 7 && strlen(longitude) > 7 && strchr(latitude, '.') != NULL && strchr(longitude, '.') != NULL) {
        return true;
      }
      delay(3000);
    }
  }

  // I've seen where setGPS(true) above worked, but then then fona.getGPS() failed a few times in a row, but each
  // time I saw this, fona.getGPS() began working consistently afterwards, so do NOT do either of the following:
  //      g_lastGPSConnAttemptWorked = false;
  //      setGPS(false);

  latitude[0] = '\0';
  longitude[0] = '\0';
  if (speed != NULL) {
    speed[0] = '\0';
    dir[0] = '\0';
  }
  return false;
}

void convertDegreesToDecimal(char* inLatStr, char NSEW) {
  // convert DDmm.mmmmmm (lat) or DDDmm.mmmmmm (lon) to decimal where D = Decimal, m = minute
  // This SUCKS. Why the hell did they format it like this?

  // NSEW is the char N or S or E or W

  // inLatStr will look like
  //    3012.586830 (lat)
  //    or
  //    12045.88604 or 09745.888562 (lon)
  float tempFloat;
  int8_t decimalSize;     // decimalSize will be 2 for latitude or 3 for longitude because lat is between 0..90 while lon is between 0..180

  if (NSEW == 'N' || NSEW == 'S')
    decimalSize = 2;      // we're working on latitude
  else
    decimalSize = 3;
  
  // if inLatStr == "09745.888562"
  // then make tempFloat = 45.888562
  // this is the "minutes" part of the number
  tempFloat = atof(&inLatStr[decimalSize]);

  // we want to keep the first 2-3 chars unmolested since they're correct as they are passed into this function.  Add a decimal point.
  inLatStr[decimalSize] = '.';

  // convert minutes to decimal by dividing by 60
  tempFloat = tempFloat / 60.0;       // 45.888562 -> 0.7648093

  uint16_t tempInt;
  // any fancy float -> string functions are too big to use.  i'll write my own.
  for (int i=decimalSize+1; i<decimalSize+7; i++) {
    tempFloat = tempFloat * 10;                 // 0.20978 -> 2.0978
    tempInt = tempFloat;                        // tempInt = 2
    inLatStr[i] = tempInt + '0';                // tempStr[i] = '2'
    tempFloat = tempFloat - (float)tempInt;     // 2.0978 -> 0.0978
  }

  inLatStr[decimalSize+7] = '\0';

  if (NSEW == 'S' || NSEW == 'W') {
    char tempStr[12];
    strcpy(tempStr, inLatStr);
    inLatStr[0] = '-';     // lat or long is negative
    strcpy(&inLatStr[1], tempStr);
  }
}

//bool getGPSLatLonSpeedDir(char* latitude, char* longitude, char* speed, char* dir) {
//  char gpsString[120];
//
//  // full GPS string:
//  // 1,1,20190913060459.000,30.213823,-97.782017,204.500,1.87,90.1,1,,1.2,1.5,0.9,,11,6,,,39,,
//  fona.getGPS(0, gpsString, 120);
//  getOccurrenceInDelimitedString(gpsString, latitude, 4, ',', 11);
//
//  // if GPS is already working OR we turn it on successfully
//  if (strlen(latitude) > 7 || setGPS(true)){
//
//    // We have seen errors where the lat,long come back as garbage like "9,43" so we may have to try a few times...
//    for (int8_t i = 0; i < 10; i++) {
//      getOccurrenceInDelimitedString(gpsString, latitude, 4, ',', 11);
//      getOccurrenceInDelimitedString(gpsString, longitude, 5, ',', 11);
//      if (speed != NULL) {
//        getOccurrenceInDelimitedString(gpsString, speed, 7, ',', 3);
//        getOccurrenceInDelimitedString(gpsString, dir, 8, ',', 3);
//        getDirFromDegrees(dir);
//      }
//  
//      // Leave the != NULL in there in case the '.' is at the 0th position, which I think is valid
//      if (strlen(latitude) > 7 && strlen(longitude) > 7 && strchr(latitude, '.') != NULL && strchr(longitude, '.') != NULL) {
//        g_lastGPSConnAttemptTime = getTimePartInt(HOUR_INDEX) * 60 + getTimePartInt(MINUTE_INDEX);
//        g_lastGPSConnAttemptWorked = true;
//        return true;
//      }
//      else
//        fona.getGPS(0, gpsString, 120);
//    }
//  }
//
//  // I've seen where setGPS(true) above worked, but then then fona.getGPS() failed a few times in a row, but each
//  // time I saw this, fona.getGPS() began working consistently afterwards, so do NOT do either of the following:
//  //      g_lastGPSConnAttemptWorked = false;
//  //      setGPS(false);
//  latitude[0] = '\0';
//  longitude[0] = '\0';
//  return false;
//}

bool getGPSTime(char* timeStr) {
  // timeStr will be in GMT and will look like YYYYMMDDhhmmss.xxx
  char gpsString[120];
  char tempTimeStr[9];

  if (setGPS(true)){
    // full string:
    // SIM7000: AT+CGNSINF:   +CGNSINF: 1,1,20190913060459.000,30.213823,-97.782017,204.500,1.87,90.1,1,,1.2,1.5,0.9,,11,6,,,39,,
    // SIM7500: AT+CGNSSINFO: +CGNSSINFO: 2,06,00,00,3012.586884,N,09745.881688,W,080421,223651.0,196.1,0.0,0.0,1.5,1.2,0.9
    //                                           ...,lat        ,S,lon         ,E,DDMMYY,HHmmss.0,  alt,spd,dir,...
    
    for (int8_t i = 0; i < 10; i++) {
      fona.getGPS(0, gpsString, 120);

      if (strlen(gpsString) > 40) {
        if (fona.type() == SIM7000)
          getOccurrenceInDelimitedString(gpsString, timeStr, 3, ',');
        else {
          getOccurrenceInDelimitedString(gpsString, tempTimeStr, 10, ',');
          strcpy_P(timeStr, PSTR("20210101_________0"));    // we don't care about the date, this is an edge case
          strcpy(&timeStr[8], tempTimeStr);                 // overwrite the _________ with the real time... from the exmple above, 20210101_________0 -> 20210101223651.0_0
          timeStr[16] = '0';                                // and change the '\0' that strcpy left to a '0'... from the exmple above, 20210101223651.0_0 -> 20210101223651.000
        }
        return true;
      }

      delay(2000);
    }
  }

  timeStr[0] = '\0';
  return false;
}

bool outsideGeofence(char* lat1Str, char* lon1Str) {
  if (lat1Str[0] == '\0') {
    return false;
  }

  char geofenceRadius[7];
  char geofenceHomeLat[12];
  char geofenceHomeLon[12];
  EEPROM.get(GEOFENCERADIUS_CHAR_7, geofenceRadius);
  EEPROM.get(GEOFENCEHOMELAT_CHAR_12, geofenceHomeLat);
  EEPROM.get(GEOFENCEHOMELON_CHAR_12, geofenceHomeLon);

  float geofenceRadiusFloat = atof(geofenceRadius);
  float lat1Float = atof(lat1Str);
  float lon1Float = atof(lon1Str);
  float lat2Float = atof(geofenceHomeLat);
  float lon2Float = atof(geofenceHomeLon);

  // Variables
  float dist_calc = 0;
  float dist_calc2 = 0;
  float dilat = 0;
  float dilon = 0;

  // Calculations
  dilat  = radians(lat2Float - lat1Float);
  lat1Float = radians(lat1Float);
  lat2Float = radians(lat2Float);
  dilon = radians((lon2Float) - (lon1Float));

  dist_calc = (sin(dilat / 2.0) * sin(dilat / 2.0));
  dist_calc2 = cos(lat1Float);
  dist_calc2 *= cos(lat2Float);
  dist_calc2 *= sin(dilon / 2.0);
  dist_calc2 *= sin(dilon / 2.0);
  dist_calc += dist_calc2;

  dist_calc = (2 * atan2(sqrt(dist_calc), sqrt(1.0 - dist_calc)));
  dist_calc *= 20902231.64; //Converting to feet

  return dist_calc > geofenceRadiusFloat;
}

////////////////////////////////
//Time

void getTime(char* currentTimeStr) {

  // sets currentTime to "20/01/31,17:03:55-20" INCLUDING quotes 
  for (int8_t i = 0; i < 3; i++) {
    fona.getTime(currentTimeStr, 23);

    // if time string looks good...
    if (currentTimeStr[0] == '"' && strlen(currentTimeStr) == 22)
      return;

    // else, try simple self-healing.  If echo is on, basically all commands to SimCom module won't work.
    fona.setEchoOff();
    delay(1000);
  }
}

int8_t getTimePartInt(int16_t index) {
  char timeStr[23];
  getTime(timeStr);
  return getTimePartInt(index, timeStr);
}

int8_t getTimePartInt(int16_t index, char* timeStr) {
  // get a 2-character "part" of the time string as an int, "part" being year, minute, hour, etc.
  char timePartStr[3];

  // for both, QUOTES ARE PART OF THE STRING!
  // CCLK: "20/01/31,17:03:01-20"
  // CNTP: "2020/05/26,21:26:21"
  timePartStr[0] = timeStr[index];
  timePartStr[1] = timeStr[index+1];
  timePartStr[2] = '\0';
  return atoi(timePartStr);
}

bool isAlwaysOn(int16_t eepromEnabled, int16_t eepromStart, int16_t eepromEnd) {
  // takes into account both "enabled" option
  // as well as the "hours"

  bool enabled;
  EEPROM.get(eepromEnabled, enabled);
  if (!enabled)
    return false;

  char startHour[3];
  char endHour[3];
  EEPROM.get(eepromStart, startHour);
  EEPROM.get(eepromEnd, endHour);

  // if start and end are the same, the fence/kill switch is ALWAYS on
  if (strcmp(startHour, endHour) == 0)
    return true;

  return false;
}

bool isActive(int16_t eepromEnabled, int16_t eepromStart, int16_t eepromEnd) {
  // takes into account both "enabled" option
  // as well as the "hours" 1am-8am option

  bool enabled;
  EEPROM.get(eepromEnabled, enabled);
  if (!enabled)
    return false;

  char startHour[3];
  char endHour[3];
  EEPROM.get(eepromStart, startHour);
  EEPROM.get(eepromEnd, endHour);

  // if start and end are the same, the fence/kill switch is ALWAYS on
  if (strcmp(startHour, endHour) == 0)
    return true;

  // 2 = clock is not valid
  if (isClockValid() == 2)
    return false;

  short currentHour = getTimePartInt(HOUR_INDEX);

  // simple case, current time is between start/end.  Start time is inclusive, end time is exclusive
  if (currentHour >= atoi(startHour) && currentHour < atoi(endHour))
    return true;

  // if start time is after end time (23-7 i.e. 11pm-7am), it only has to satisfy one of the conditions.  HOW INTERESTING
  if (strcmp(startHour, endHour) > 0)
    if (currentHour >= atoi(startHour) || currentHour < atoi(endHour))
      return true;

  return false;
}


////////////////////////////////
//SMS

void checkForDeadMessages() {
  // skip if we're not connected to the SimCom
  if (g_SimComConnectionStatus == 1) {
    return;
  }

  // SIM card can only hold 10 messages, it cannot see the rest until those 10 are processed.  That means if we are debugging
  // and send "deleteallmessages" and there are already 10 queue'd up, SimCom module will never see the "deleteallmessages" message.
  // SO, if we start up and there are 10 messages, 99% of the time that means one of them is causing problems.
  // This should never happen, but allows turning off/on to clear out messages if "deleteallmessages" isn't working.
  int8_t numberOfSMSs = fona.getNumSMS();
  if (numberOfSMSs == 10) {
    fona.deleteAllSMS();
  }
}

void deleteSMS(int8_t msg_number) {
  for (int8_t i = 0; i < 5; i++) {
    if (fona.deleteSMS(msg_number)) {
      debugBlink(1,4);
      return;
    }
    delay(2000);
  }
  debugBlink(1,3);
}

void cleanMessage(bool usePlainSMS, char* message) {
  // for all SMS
  //   change double quote char '\"' to single quote '\''
  // for plainSMS only
  //   change 3 chars "\\\\n" to newline char '\n'

  // strLen does NOT include terminating '\0'
  int16_t strLen = strlen(message);
  int16_t index = 0;

  for (int16_t i = 0; i < strLen; i++) {
      // Hack for Hologram.io + Twilio: we send send "\\\\n" for newline.
      // For plain SMSs, we need to change those back to plain "\n"
      if (usePlainSMS && message[i] == '\\' && message[i+1] == '\\' && message[i+2] == 'n') {
          message[index] = '\n';
          i+=2;
      // Also for Hologram.io + Twilio: '"' char messes things up.  Use '\'' char instead
      } else if (message[i] == '"') {
          message[index] = '\'';
      } else {
          message[index] = message[i];
      }
      index++;
  }
  message[index] = '\0';
}

bool sendSMS(char* send_to, char* message) {
  if (g_SimComConnectionStatus > 0)
    return false;

  bool usePlainSMS = false;
  EEPROM.get(USEPLAINSMS_BOOL_1, usePlainSMS);

  cleanMessage(usePlainSMS, message);

  if (usePlainSMS) {
    g_v_currentlySendingSMS = true;
    bool sentOK = fona.sendSMS(send_to, message);
    g_v_currentlySendingSMS = false;

    if (sentOK) {
      debugBlink(1,6);
      updateLastResetTime();
      g_totalFailedSendSMSAttempts = 0;   
      return true;
    } else {
      debugBlink(2,11);
      g_totalFailedSendSMSAttempts++;
      return false;
    }
  }

  // Example of hologramSMSString: 
  // "{\"k\":\"aaaabbbb\",\"d\":\"{\\\"t\\\":\\\"1115556667777\\\",\\\"f\\\":\\\"19998887777\\\",\\\"m\\\":\\\"this is the message\\\"}\",\"t\":\"TWIL\"}"
  // ...in other words:
  // {"k":"aaaabbbb","d":"{\"t\":\"1115556667777\",\"f\":\"19998887777\",\"m\":\"this is the message\"}","t":"TWIL"}
  //
  // devkey identifier                        {"k":"                  6
  // devkey                                   aaaabbbb                8
  // data identifier                          ","d":                  6
  // phone number identifier                  "{\"t\":                8
  // to phone number (3-digit country code)   \"1115556667777\"       17
  // message identifier                       ,\"f\":                 7
  // from twilio phone number (USA #)         \"19998887777\"         15
  // message identifier                       ,\"m\":                 7
  // the message (160) plus \" twice          \"...\"                 174       // we have to turn every 1 newline char '\n' into 3 chars "\\\\n" so we need a little more than 144 chars
  // topic identifier                         }","t":                 7
  // topic plus close brace                   "TWIL"}                 7
  // null-terminator for C string             \0                      1

  char hologramSMSString[263];
  int16_t hologramSMSStringLength;

  char devKey[9];
  EEPROM.get(DEVKEY_CHAR_9, devKey);

  // 0 is the default devKey (comes from initEEPROM)
  // return true so that we delete the incoming SMS, so that we don't try to send the msg indefinitely
  if (strcmp_P(devKey, PSTR("0")) == 0) {
    debugBlink(2,9);
    return true;
  }

  char twilioPhoneNumber[12];
  EEPROM.get(TWILIOPHONENUMBER_CHAR_12, twilioPhoneNumber);  
  
  strcpy_P(hologramSMSString, PSTR("{\"k\":\""));
  strcat(hologramSMSString, devKey);
  strcat_P(hologramSMSString, PSTR("\",\"d\":\"{\\\"t\\\":\\\""));
  strcat(hologramSMSString, &send_to[1]);    // strip + from phone number "+15559998888"
  strcat_P(hologramSMSString, PSTR("\\\",\\\"f\\\":\\\""));
  strcat(hologramSMSString, twilioPhoneNumber);
  strcat_P(hologramSMSString, PSTR("\\\",\\\"m\\\":\\\""));
  strcat(hologramSMSString, message);
  strcat_P(hologramSMSString, PSTR("\\\"}\",\"t\":\"TWIL\"}"));
  hologramSMSStringLength = strlen(hologramSMSString);
  
  debugPrint(F("OUT: "));
  debugPrintln(message);
  debugPrintln(hologramSMSString);

  g_v_currentlySendingSMS = true;
  int8_t successCode = fona.ConnectAndSendToHologram(SERVER_NAME, SERVER_PORT, hologramSMSString, hologramSMSStringLength);
  g_v_currentlySendingSMS = false;

  debugPrint(F("? "));
  debugPrintln(successCode);

  if (successCode == 0) {
    debugBlink(1,2);
    updateLastResetTime();
    g_totalFailedSendSMSAttempts = 0;
    return true;
  } else {
    // see very top for debug blink code meanings (which in this case are coming from the cellular module
    debugBlink(2,successCode);
    delay(3000);
    g_totalFailedSendSMSAttempts++;
    return false;
  }
}

bool sendSMS(char* send_to, const __FlashStringHelper* messageInProgmem) {
  // yeah this sucks, but it's better than having all those strings stored in SRAM as globals
  char message[161];
  strcpy_P(message, (const char*)messageInProgmem);
  return sendSMS(send_to, message);
}


///////////////////////////////////////////////////////////////////////////////////////////
//HELPERS
///////////////////////////////////////////////////////////////////////////////////////////
void debugBlink(int8_t longBlinks, int8_t shortBlinks) {
  for (int8_t cnt=0; cnt < longBlinks; cnt++) {
    digitalWrite(DEBUG_PIN, HIGH);
    delay(1000);
    digitalWrite(DEBUG_PIN, LOW);
    delay(300);    
  }
  for (int8_t cnt=0; cnt < shortBlinks; cnt++) {
    digitalWrite(DEBUG_PIN, HIGH);
    delay(50);
    digitalWrite(DEBUG_PIN, LOW);
    delay(300);
  }
  delay(300);
}

bool setHoursFromSMS(char* smsValue, char* hoursStart, char* hoursEnd) {
  // smsValue:
  // fence hours 0 21
  // 0 21 means 12am - 9pm
  getOccurrenceInDelimitedString(smsValue, hoursStart, 3, ' ', 2);
  getOccurrenceInDelimitedString(smsValue, hoursEnd, 4, ' ', 2);

  int8_t hoursStartInt = atoi(hoursStart);
  int8_t hoursEndInt = atoi(hoursEnd);

  // If atoi can't find a translation (it returns 0) AND the first character isn't '0'
  // or if hours are not valid
  // then something's wrong
  if ((hoursStartInt == 0 && hoursStart[0] != '0') ||
      (hoursEndInt   == 0 && hoursEnd[0] != '0') ||
      (hoursStartInt < 0) ||
      (hoursStartInt > 23) ||
      (hoursEndInt   < 0) ||
      (hoursEndInt   > 23)) {
    return false;
  }

  return (hoursStart[0] && hoursEnd[0]);
}

bool setEnableAndHours(char* smsValue, int16_t eepromEnabled, int16_t eepromStart, int16_t eepromEnd, bool &enabled, char* hoursStart, char* hoursEnd) {
  bool validMessage = false;

  if (strstr_P(smsValue, PSTR("enable"))) {
    EEPROM.put(eepromEnabled, true);
    validMessage = true;
  }
  if (strstr_P(smsValue, PSTR("disable"))) {
    EEPROM.put(eepromEnabled, false);
    validMessage = true;
  }
  
  EEPROM.get(eepromEnabled, enabled);
  
  if (strstr_P(smsValue, PSTR("hours"))) {
    if (setHoursFromSMS(smsValue, hoursStart, hoursEnd)) {
      // make 4 => 04
      insertZero(hoursStart);
      insertZero(hoursEnd);
      
      writeCStringToEEPROM(eepromStart, hoursStart);
      writeCStringToEEPROM(eepromEnd, hoursEnd);
      validMessage = true;
    }
  }
  return validMessage;
}

void writeCStringToEEPROM(int16_t eepromAddress, char* data) {
  int16_t i=0;
  for (;data[i]; i++) {
    EEPROM.put(eepromAddress+i, data[i]);
  }
  EEPROM.put(eepromAddress+i, '\0');
}

void setSimComFunctionality(bool onOff) {
  if (onOff) {
    sendRawCommand(F("AT+CFUN=1,1"));
    delay(20000);
  } else {
    setGPS(false);
    sendRawCommand(F("AT+CFUN=0,0"));
    delay(20000);
  }
}

void insertZero(char *in) {
  // makes "4" => "04"
  char tempCh = in[0];

  // if the string is 1 char long, i.e. if the 2nd char is the null char '\0'
  if (!in[1]) {
    in[0] = '0';
    in[1] = tempCh;
    in[2] = '\0';
  }
}

void sendRawCommand(char* command) {
  delay(200);
  fona.println(command);
  delay(1000);

  if (fona.available()) {
    flushSimCom();
  }
  delay(1000);
}

void sendRawCommand(const __FlashStringHelper* command) {
  delay(200);
  fona.println(command);
  delay(1000);

  if (fona.available()) {
    flushSimCom();
  }
  delay(1000);
}

void toLower(char* str) {
  for (int16_t i = 0; str[i]; i++)
    str[i] = tolower(str[i]);
}

void addPlusToPhoneNumber(char* phoneNumber) {
  // in:  "15554443333\0\0\0\0"
  // out: "+15554443333\0\0\0"
  phoneNumber[14] = '\0';  // phoneNumber is of size 15

  for (int8_t i = 14; i > 0; i--) {
    phoneNumber[i] = phoneNumber[i-1];
  }
  phoneNumber[0] = '+';
}

bool getNumberFromString(char* in, char* out, int8_t maxLen) {
  // if in == "aaa+123b-b4b5"
  // then out = "12345"
  // be sure to leave room for '\0'
  bool foundNumber = false;
  int16_t outCount = 0;

  for (int16_t i = 0; in[i] && outCount < maxLen-1; i++) {
    if (in[i] >= '0' && in[i] <= '9') {
      foundNumber = true;
      out[outCount] = in[i];
      outCount++;
    }
  }
  out[outCount] = '\0';

  return foundNumber;
}

void removeNonAlphaNumChars(char* stringToClean) {
  // Trying to send "AT&T" made the system crash.  Unit went into infinte loop trying and failing to send.
  // Resolution is to send 10+ SMSs and restart the system (on startup if > 9 SMSs on the module, it will delete them all... for just such an occasion).
  // To avoid this, we remove all non alphaNum chars.
  uint8_t len = strlen(stringToClean);
  for (uint8_t i = 0; i < len; i++) {
    if (!((stringToClean[i] >= 'a') && (stringToClean[i] <= 'z')) &&
        !((stringToClean[i] >= 'A') && (stringToClean[i] <= 'Z')) &&
        !((stringToClean[i] >= '0') && (stringToClean[i] <= '9'))
       )
      stringToClean[i] = '_';
  }
}

void cleanString(char* stringToClean, char charToClean) {
  // in:  "  some  string   with stuff  "
  // out: "some string with stuff"
  int16_t outCount = 0;
  bool lastCharWasCharToClean = false;
  
  for (int16_t i = 0; stringToClean[i]; i++) {
    if (stringToClean[i] == charToClean) {
      if (outCount == 0)                        // seeing ' ' at the beginning of the string, skip it
        continue;
      if (lastCharWasCharToClean == true)       // if this is the second ' ' in a row, skip it
        continue;
      else {
        stringToClean[outCount] = stringToClean[i];   // else, it's the first ' ' in a row, write it
        outCount++;
      }
      lastCharWasCharToClean = true;
    }
    else {
      stringToClean[outCount] = stringToClean[i];
      outCount++;
      lastCharWasCharToClean = false;
    }
  }

  if (stringToClean[outCount-1] == charToClean)
    stringToClean[outCount-1] = '\0';
  else
    stringToClean[outCount] = '\0';
}

bool getOccurrenceInDelimitedString(char* in, char* out, int8_t occurrenceNumber, char delim) {
  // yeah I know.  Trying to save program space.
  return getOccurrenceInDelimitedString(in, out, occurrenceNumber, delim, 9999);
}

bool getOccurrenceInDelimitedString(char* in, char* out, int8_t occurrenceNumber, char delim, int16_t maxLength) {
  // occurrenceNumber is 1-based, not 0-based
  // maxLength does NOT include ending '\0'

  // if in == "a,b,c"
  // and occurrenceNumber = 2
  // and delim = ','
  // sets out = "b"
  // be sure to leave room for '\0'

  // maxLength is due to the bug in the SIM7000A for strings like "aa{a" it'll tell us it's 5 chars long. See https://forum.arduino.cc/index.php?topic=660925.0
  int16_t delimCount = 0;
  int16_t outCount = 0;
  bool foundOccurrence = false;

  if (delim == ' ')
    cleanString(in, delim);

  for (int16_t i = 0; in[i] && outCount < maxLength; i++) {
    if (in[i] == delim) {
      if (delimCount + 1 == occurrenceNumber) {
        break;
      }
      else {
        delimCount++;
        continue;
      }
    }

    if (delimCount + 1 == occurrenceNumber) {
      out[outCount] = in[i];
      outCount++;
      foundOccurrence = true;
    }
  }
  out[outCount] = '\0';
  return foundOccurrence;
}

void flushSerial() {
  while (Serial.available())
    Serial.read();
}

void flushSimCom() {
  while (fona.available())
    Serial.write(fona.read());
}


///////////////////////////////////////////////////////////////////////////////////////////
//SETUP & PINS
///////////////////////////////////////////////////////////////////////////////////////////

void pinSetup() {
  pinMode(STARTER_INTERRUPT_PIN, INPUT_PULLUP);                 // when starter is on, PIN is LOW
  attachInterrupt(STARTER_INTERRUPT_ID, starterISR, FALLING);   // when starter is on, PIN is LOW
  pinMode(DOOR_INTERRUPT_PIN, INPUT_PULLUP);                    // when door is open, PIN is LOW
  attachInterrupt(DOOR_INTERRUPT_ID, doorISR, FALLING);         // when door is open, PIN is LOW
  
  pinMode(KILL_SWITCH_RELAY_PIN, OUTPUT);
  digitalWrite(KILL_SWITCH_RELAY_PIN, LOW);

  pinMode(DEBUG_PIN, OUTPUT);
  digitalWrite(DEBUG_PIN, LOW);
}

void starterISR() {

  // The resistor (hardware) should prevent little spikes from making the ISR fire all the time
  // but if that's happening, this will show us on the debug output
  g_v_killSwitchInterruptDebug = true;

  // If either door-open or start-attempt alert has already been triggered, let's just make sure it goes out and not interrupt whatever is going on
  if (g_v_startAttemptedWhileKillSwitchActive || g_v_doorOpenedWhileDoorAlertActive)
    return;

  // If we're currently sending an SMS, don't screw up the SMS.
  //    case 1: we're sending "Door open!" or "Start attempted!".  We REALLY want that to be successful, and sending the other alert doesn't add much value.
  //    case 2: we're sending a response to the owner.  Odds are they're standing next to their van, it's not being stolen, so let's just be sure to respond successfully.
  if (g_v_currentlySendingSMS)
    return;

  if (!g_v_killSwitchAndDoorAlertInitialized)
    return;

  if (!g_v_killSwitchAndDoorAlertActive)
    return;

  _delay_ms(400);  // on some starters, turning to the key to the "accessory" mode might jump to 12V for just a few milliseconds, so let's wait - make sure someone is actually trying to start the car

  // when starter is on, PIN is LOW
  if (!digitalRead(STARTER_INTERRUPT_PIN))
    g_v_startAttemptedWhileKillSwitchActive = true;
}

void doorISR() {

  // There shouldn't be little spikes making the ISR fire all the time
  // but if that's happening, this will show us on the debug output
  g_v_doorInterruptDebug = true;

  // If either door-open or start-attempt alert has already been triggered, let's just make sure it goes out and not interrupt whatever is going on
  if (g_v_startAttemptedWhileKillSwitchActive || g_v_doorOpenedWhileDoorAlertActive)
    return;

  // If we're currently sending an SMS, don't screw up the SMS.
  //    case 1: we're sending "Door open!" or "Start attempted!".  We REALLY want that to be successful, and sending the other alert doesn't add much value.
  //    case 2: we're sending a response to the owner.  Odds are they're standing next to their van, it's not being stolen, so let's just be sure to respond successfully.
  if (g_v_currentlySendingSMS)
    return;

  if (!g_v_killSwitchAndDoorAlertInitialized)
    return;

  if (!g_v_killSwitchAndDoorAlertActive)
    return;

  _delay_ms(400);  // shouldn't be necessary, but no one can open a door and close it in 0.4 seconds anyway

  // when starter is on, PIN is LOW
  if (!digitalRead(DOOR_INTERRUPT_PIN))
    g_v_doorOpenedWhileDoorAlertActive = true;
}

void setKillSwitchAndDoorAlert(bool tf) {
  g_v_killSwitchAndDoorAlertInitialized = true;
  g_v_killSwitchAndDoorAlertActive = tf;
  digitalWrite(KILL_SWITCH_RELAY_PIN, tf);
}

void setupSimCom() {
  // This can take up to 10 minutes (if it never connects)
  debugPrintln(F("Sim"));
  // let SimCom module start up before we try to connect
  SimComSerial->begin(9600);

  for (int8_t i = 0; i < 6; i++) {
    debugBlink(0,5);
    fona.begin(*SimComSerial);

    if (fona.getNumSMS() >= 0) {
      debugPrintln(F("\nOK"));
      g_SimComConnectionStatus = 2; // set to 2 because setupSimCom() needs to be followed by waitUntilNetworkConnected() which will update g_SimComConnectionStatus = 0
      debugBlink(0,4);
      return;
    }
    delay(10000);
  }
  g_SimComConnectionStatus = 1;
}

void waitUntilNetworkConnected(int16_t secondsToWait) {
  // This can take up to 10 minutes (if it never connects)
  // no point trying to conenct to network if we can't connect to SimCom
  if (g_SimComConnectionStatus == 1) {
    return;
  }

  debugPrintln(F("Net"));
  int8_t netConn;

  fona.setEchoOff();
  fona.setNetworkSettings(APN, F(""), F(""));

  // we're waiting 2s each loop
  secondsToWait = secondsToWait/2;
  
  for (int16_t i = secondsToWait; i > 0; i--) {
    debugBlink(0,6);
    // About to run out of time... last ditch effort...
    // set Cellular OPerator Selection to "automatic"
    if (i < 5)
      fona.setNetworkOperator(F("AT&T"));

    fona.setEchoOff();
    netConn = fona.getNetworkStatus();

    // netConn status meanings:
    // 0 Not registered, not currently searching an operator to register to, the GPRS service is disabled
    // 1 Registered, home
    // 2 Not registered, trying to attach or searching an operator to register to
    // 3 Registration denied
    // 4 Unknown
    // 5 Registered, roaming

    // g_SimComConnectionStatus status meanings
    // 0 = connected to cell network
    // 1 = Failed to connect to SimCom chip
    // 2 = Not registered on cell network
    // 3 = Cell network registration denied
    // 4 = Unknown
    if (netConn == 1 || netConn == 5) {
      debugPrintln(F("\nOK"));
      g_SimComConnectionStatus = 0;
      fona.setNetworkSettings(APN, F(""), F(""));
      fona.TCPshut();
      return;
    }
    delay(2000);
  }

  // netConn == 0 means not registered, so we translate it to 2 (also means not registered) before assigning its value to g_SimComConnectionStatus
  if (netConn == 0)
    netConn = 2;

  g_SimComConnectionStatus = netConn;
  setSimComFunctionality(false);
}

#if defined VAN_TEST || defined NEW_HARDWARE_ONLY || defined SIMCOM_SERIAL || defined UPGRADING_HARDWARE_ONLY
void setupSerial() {
  while (!Serial);
  Serial.begin(9600);
}
#endif


#if defined NEW_HARDWARE_ONLY || defined UPGRADING_HARDWARE_ONLY
void initBaud() {
  debugPrintln(F("Init Baud"));
  delay(2000);
  debugPrintln(F("Trying to connect at 9600"));
  SimComSerial->begin(9600);

  if (! fona.begin(*SimComSerial)) {
    debugPrintln(F("Trying to connect at 115200"));
    SimComSerial->begin(115200);
    
    if (! fona.begin(*SimComSerial)) {
      debugPrintln(F("ERROR: Could not connect at 9600 or 115200"));
      return;
    } else {
      debugPrintln(F("Connected at 115200, setting to 9600..."));
      if (fona.type() == SIM7000)
        sendRawCommand(F("AT+IPR=9600"));
      else
        sendRawCommand(F("AT+IPREX=9600"));
      SimComSerial->begin(9600);
    }
  } else {
    debugPrintln(F("Connected at 9600"));
    return;
  }
}

void initEEPROM() {
  // used on brand-new Arduino Nano
  debugPrintln(F("Begin initEEPROM()"));
  EEPROM.put(GEOFENCEENABLED_BOOL_1, false);
  EEPROM.put(GEOFENCEHOMELAT_CHAR_12, "52.4322115");
  EEPROM.put(GEOFENCEHOMELON_CHAR_12, "10.7869289");
  EEPROM.put(GEOFENCERADIUS_CHAR_7, "500");
  EEPROM.put(GEOFENCESTART_CHAR_3, "23");
  EEPROM.put(GEOFENCEEND_CHAR_3, "07");
  EEPROM.put(GEOFENCEFOLLOW_BOOL_1, false);
  EEPROM.put(KILLSWITCHENABLED_BOOL_1, false);
  EEPROM.put(KILLSWITCHSTART_CHAR_3, "23");
  EEPROM.put(KILLSWITCHEND_CHAR_3, "07");
  EEPROM.put(OWNERPHONENUMBER_CHAR_15, "+15551234567");
  
  EEPROM.put(LOCKDOWNENABLED_BOOL_1, false);
  EEPROM.put(GEOFENCEENABLED_BOOL_SAVED_1, false);
  EEPROM.put(GEOFENCEHOMELAT_CHAR_SAVED_12, "52.4322115");
  EEPROM.put(GEOFENCEHOMELON_CHAR_SAVED_12, "10.7869289");
  EEPROM.put(GEOFENCESTART_CHAR_SAVED_3, "00");
  EEPROM.put(GEOFENCEEND_CHAR_SAVED_3, "00");
  EEPROM.put(KILLSWITCHENABLED_BOOL_SAVED_1, false);
  EEPROM.put(KILLSWITCHSTART_CHAR_SAVED_3, "00");
  EEPROM.put(KILLSWITCHEND_CHAR_SAVED_3, "00");
  
  EEPROM.put(DEVKEY_CHAR_9, "0");
  EEPROM.put(TWILIOPHONENUMBER_CHAR_12, "00000000000");
  
  EEPROM.put(TIMEZONE_CHAR_4, "-20");
  EEPROM.put(USEPLAINSMS_BOOL_1, false);
  EEPROM.put(POWERON_BOOL_1, true);

  debugPrintln(F("End initEEPROM()"));
}

void initSimCom() {
  // used on brand-new SimCom module
  debugPrintln(F("Begin initSimCom()"));

  sendRawCommand(F("ATZ"));                   // Reset settings

  sendRawCommand(F("AT+CMEE=2"));             // Turn on verbose mode
  if (fona.type() == SIM7000) {
    sendRawCommand(F("AT+IPR=9600"));         // set connection baud to 9600
    sendRawCommand(F("AT+CLTS=1"));           // Turn on "get clock when registering w/network" see https://forums.adafruit.com/viewtopic.php?f=19&t=58002
    sendRawCommand(F("AT+CNETLIGHT=1"));      // Turn on "net" LED
  } else {
    sendRawCommand(F("AT+IPREX=9600"));       // also set connection baud to 9600
  }
  fona.setNetworkOperator(F("AT&T"));         // Set Cellular OPerator Selection to "automatic"
  sendRawCommand(F("AT+COPS=4,1,\"AT&T\""));  // ...this is really important, so I'm doubling up.
  sendRawCommand(F("AT+CMEE=0"));             // Turn off verbose mode

  sendRawCommand(F("AT&W"));                // save writeable settings

  debugPrintln(F("End initSimCom()"));
}
#endif


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// TESTING
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void debugPrint(char* str) {
#if defined VAN_TEST || defined NEW_HARDWARE_ONLY || defined SIMCOM_SERIAL || defined UPGRADING_HARDWARE_ONLY
  Serial.print(str);
#endif
}
void debugPrint(const __FlashStringHelper* str) {
#if defined VAN_TEST || defined NEW_HARDWARE_ONLY || defined SIMCOM_SERIAL || defined UPGRADING_HARDWARE_ONLY
  Serial.print(str);
#endif
}
void debugPrintln(char* str) {
#if defined VAN_TEST || defined NEW_HARDWARE_ONLY || defined SIMCOM_SERIAL || defined UPGRADING_HARDWARE_ONLY
  Serial.println(str);
#endif
}
void debugPrintln(const __FlashStringHelper* str) {
#if defined VAN_TEST || defined NEW_HARDWARE_ONLY || defined SIMCOM_SERIAL || defined UPGRADING_HARDWARE_ONLY
  Serial.println(str);
#endif
}
void debugPrintln(String s) {
#if defined VAN_TEST || defined NEW_HARDWARE_ONLY || defined SIMCOM_SERIAL || defined UPGRADING_HARDWARE_ONLY
  Serial.println(s);
#endif
}
void debugPrintln(int i) {
#if defined VAN_TEST || defined NEW_HARDWARE_ONLY || defined SIMCOM_SERIAL || defined UPGRADING_HARDWARE_ONLY
  Serial.println(i);
#endif
}






#if defined VAN_TEST || defined SIMCOM_SERIAL

void checkSerialInput() {
  if (!Serial.available()) {
    return;
  }

  char command[120];

  int16_t i = 0;
  for (; Serial.available(); i++) {
    command[i] = Serial.read();
  }
  command[i-1] = '\0';

  debugPrintln(command);
  handleSerialInput(command);
}

void handleSerialInput(char* temp) {

  // PART 1: 
  // Allow user to directly enter AT commands to the SimCom chip
  if (temp[0] == 'S') {
    sendRawCommand(F("AT+CMEE=2"));
    sendRawCommand(F("ATE1"));
    
    delay(2000);    

    while (1) {
      while (Serial.available()) {
        delay(1);
        fona.write(Serial.read());
      }
      if (fona.available()) {
        Serial.write(fona.read());
      }
    }
  }

  // PART 2:
  // Some convenience commands for testing
//  if (strcmp_P(temp, PSTR("t")) == 0) {
//    char gpsTimeStr[19];
//    getGPSTime(gpsTimeStr);
//  }
//  if (temp[0] == 'g') {
//    getEEPROM();
//  }
//  if (temp[0] == 'p') {
//    putEEPROM();
//  }
//  if (temp[0] == 'e') {
//    resetSystem();
//  }
//  if (temp[0] == 'd') {
//    fona.deleteAllSMS();
//  }
//  if (temp[0] == 'm') {
//    char ownerPhoneNumber[15];
//    EEPROM.get(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);
//    char message[7]="ip msg";
//    sendSMS(ownerPhoneNumber, message);
//  }
//  if (temp[0] == 'n') {
//    char ownerPhoneNumber[15];
//    EEPROM.get(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);
//    char message[10]="plain msg";
//    fona.sendSMS(ownerPhoneNumber, message);
//  }

  // PART 3:
  // Test incoming SMS, for example: 15554443333_fence status
//  if (strlen(temp) > 4){
//    char smsSender[15];
//    char smsValue[51];
//    getOccurrenceInDelimitedString(temp, smsSender, 1, '_');
//    getOccurrenceInDelimitedString(temp, smsValue, 2, '_');
//    testHandleSMSInput(smsSender, smsValue);
//  }

  flushSerial();
  flushSimCom();
}

void putEEPROM() {
  EEPROM.put(GEOFENCEENABLED_BOOL_1, false);
  EEPROM.put(GEOFENCEHOMELAT_CHAR_12, "0.0");
  EEPROM.put(GEOFENCEHOMELON_CHAR_12, "0.0");
  EEPROM.put(TIMEZONE_CHAR_4, "-12");
}

void getEEPROM() {
  char tempc[24] = {0};
  bool tempb;
  int16_t tempi;

  EEPROM.get(GEOFENCEHOMELAT_CHAR_12, tempc);
  debugPrint(F("HOME_LAT: "));
  debugPrintln(tempc);
  EEPROM.get(GEOFENCEHOMELON_CHAR_12, tempc);
  debugPrint(F("HOME_LON: "));
  debugPrintln(tempc);
  EEPROM.get(GEOFENCEENABLED_BOOL_1, tempb);
  debugPrint(F("FENCE_ON: "));
  debugPrintln(tempb);
  EEPROM.get(GEOFENCESTART_CHAR_3, tempc);
  debugPrint(F("FENCE_START: "));
  debugPrintln(tempc);
  EEPROM.get(GEOFENCEEND_CHAR_3, tempc);
  debugPrint(F("FENCE_END: "));
  debugPrintln(tempc);
  EEPROM.get(GEOFENCERADIUS_CHAR_7, tempc);
  debugPrint(F("RADIUS: "));
  debugPrintln(tempc);
  EEPROM.get(GEOFENCEFOLLOW_BOOL_1, tempb);
  debugPrint(F("FOLLOW: "));
  debugPrintln(tempb);
  EEPROM.get(KILLSWITCHENABLED_BOOL_1, tempb);
  debugPrint(F("KILL_ON: "));
  debugPrintln(tempb);
  EEPROM.get(KILLSWITCHSTART_CHAR_3, tempc);
  debugPrint(F("KILL_START: "));
  debugPrintln(tempc);
  EEPROM.get(KILLSWITCHEND_CHAR_3, tempc);
  debugPrint(F("KILL_END: "));
  debugPrintln(tempc);
  EEPROM.get(OWNERPHONENUMBER_CHAR_15, tempc);
  debugPrint(F("OWNER: "));
  debugPrintln(tempc);
  EEPROM.get(TWILIOPHONENUMBER_CHAR_12, tempc);
  debugPrint(F("TWILIO: "));
  debugPrintln(tempc);
  EEPROM.get(DEVKEY_CHAR_9, tempc);
  debugPrint(F("DEVKEY: "));
  debugPrintln(tempc);

  EEPROM.get(LOCKDOWNENABLED_BOOL_1, tempb);
  debugPrint(F("LOCK_ON: "));
  debugPrintln(tempb);

  EEPROM.get(POWERON_BOOL_1, tempb);
  debugPrint(F("POWERON: "));
  debugPrintln(tempb);

  EEPROM.get(TIMEZONE_CHAR_4, tempc);
  debugPrint(F("TIMEZONE: "));
  debugPrintln(tempc);
}

void testHandleSMSInput(char* smsSender, char* smsValue) {

  toLower(smsValue);

  debugPrintln(smsValue);

  if (strstr_P(smsValue, PSTR("loc"))) {
    handleLocReq(smsSender);
    return;
  }

  if (strstr_P(smsValue, PSTR("kill"))) {
    handleKillSwitchReq(smsSender, smsValue, false);
    return;
  }

  if (strstr_P(smsValue, PSTR("fence"))) {
    handleGeofenceReq(smsSender, smsValue, false);
    return;
  }

  if (strstr_P(smsValue, PSTR("owner"))) {
    handleOwnerReq(smsSender, smsValue);
    return;
  }

  if (strstr_P(smsValue, PSTR("devkey"))) {
    handleDevKeyReq(smsSender, smsValue);
    return;
  }

  if (strstr_P(smsValue, PSTR("status"))) {
    handleStatusReq(smsSender);
    return;
  }

  handleUnknownReq(smsSender);
}

#endif
