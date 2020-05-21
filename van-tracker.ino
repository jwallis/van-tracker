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
  3 = scheduled SimCom chip reset
  4 = connected to SimCom successfully

Event codes (1 long follow by THIS MANY shorts).  Notice odd numbers are bad, even numbers ok:
  1  = not used (see below for error codes in failure to send SMS)
  2  = success sending SMS
  3  = failed  deleting SMS
  4  = success deleting SMS
  5  = failed  turning on  GPS
  6  = not used
  7  = failed  getting fix on GPS
  8  = success getting fix on GPS
  9  = failed  turning off GPS
  10 = success turning off GPS

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

Connection failure either to SimCom chip or cellular network (3 long followed by THIS MANY shorts):
  This happens before restarting, which will have its own blink code, see above
    1 = Failed to connect to SimCom chip
    2 = Not registered, trying to attach or searching an operator to register to
    3 = Registration denied
    4 = Unknown
*/

//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//    SET NETWORK & HARDWARE OPTIONS
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

#define APN               F("hologram")
#define SERVER_NAME       "cloudsocket.hologram.io" // DO NOT USE F() MACRO HERE!  See https://hologram.io/docs/reference/cloud/embedded/#send-an-sms-via-the-hologram-cloud
#define SERVER_PORT       9999

#define VAN_PROD
//#define VAN_TEST
//#define NEW_HARDWARE_ONLY  // Initializes new SimCom module as well as new arduino's EEPROM

//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX


#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include "util/delay.h"

#define STARTER_INTERRUPT_ID 0   // interrupt 0 == pin 2.  I hate that.
#define STARTER_INTERRUPT_PIN 2  // interrupt 0 == pin 2.  I hate that.
#define SIMCOM_RX_PIN 3
#define SIMCOM_TX_PIN 4

#define KILL_SWITCH_RELAY_PIN 5
#define KILL_SWITCH_LED_PIN 6
#define GEOFENCE_LED_PIN 7
#define DEBUG_PIN 13

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
#define GEOFENCERADIUS_CHAR_SAVED_7       87
#define GEOFENCESTART_CHAR_SAVED_3        94
#define GEOFENCEEND_CHAR_SAVED_3          97
#define KILLSWITCHENABLED_BOOL_SAVED_1    100
#define KILLSWITCHSTART_CHAR_SAVED_3      101
#define KILLSWITCHEND_CHAR_SAVED_3        104

#define DEVKEY_CHAR_9                     107
#define SERVERNAME_CHAR_24                116
#define SERVERPORT_INT_2                  140


// simComConnectionStatus
// 0 = connected to cell network
// 1 = Failed to connect to SimCom chip
// 2 = Not registered, trying to attach or searching an operator to register to
// 3 = Registration denied
// 4 = Unknown
short simComConnectionStatus = 2;

short lastRestartHour = -1;
short lastRestartMinute = -1;
short lastGPSQueryMinute = -1;
short lastGeofenceWarningMinute = -1;

volatile bool killSwitchOnVolatile = false;
volatile bool startAttemptedWhileKillSwitchOnVolatile = false;

void setup() {

  pinSetup();
  
#ifdef NEW_HARDWARE_ONLY
  setupSerial();
  initBaud();
  setupSimCom();
  initEEPROM();
  initSimCom();
  debugPrintln(F("\n\nPlease update #ifdefs and restart."));
  while (1) {
    debugBlink(1,0);
  }
#endif

#ifdef VAN_TEST
  setupSerial();
#endif

  setupSimCom();
  waitUntilNetworkConnected(600);
  checkForDeadMessages();
  
  lastRestartHour = getCurrentHourInt();
  lastRestartMinute = getCurrentMinuteInt();
}

void loop() {
#ifdef VAN_TEST
  checkSerialInput();
  flushSimCom();
#endif

  // 0 is good - we're connected to the cellular network
  if (simComConnectionStatus == 0) {
    debugBlink(0,1);
    checkSMSInput();

    debugBlink(0,2);
    watchDogForKillSwitch();
    watchDogForGeofence();
    watchDogForTurnOffGPS();
    watchDogForReset();
  }
  // 1 is very bad - could not connect to SIM7000
  if (simComConnectionStatus == 1) {

    // Delay 30 minutes and reset.  This loop takes about 7 seconds. 250 loops * 7 seconds = 30 minutes.
    for (short i = 0; i < 250; i++) {
      delay(2000);
      debugBlink(3,simComConnectionStatus);
    }
    resetSystem();
  }
  // > 1 is also bad - could not connect to cellular network
  if (simComConnectionStatus > 1) {
    debugBlink(3,simComConnectionStatus);
    delay(2000);

    debugBlink(0,2);
    watchDogForKillSwitch();
    watchDogForTurnOffGPS();
    watchDogForReset();
  }

  delay(500);
}

void watchDogForReset() {
  short currentTime = getCurrentHourInt() * 60 + getCurrentMinuteInt();
  short lastRestartTime = lastRestartHour * 60 + lastRestartMinute;

  // Edge case: if simcom stops responding, we may get both times == 0.  
  // Restart.  It will then either start working or go to simComConnectionStatus == 1
  if (currentTime == 0 && lastRestartTime == 0) {
            debugPrintln("watchDogForReset3: ");
    resetSystem();
    return;
  }

  // 0 is connected
  if (simComConnectionStatus == 0) {
    // if it's been 120 minutes, restart
    if (lastRestartTime <= currentTime && currentTime - lastRestartTime > 120) {
      resetSystem();
    }
    if (lastRestartTime > currentTime && lastRestartTime - currentTime < 1320) {
      resetSystem();
    }
  } // > 0 is not connected
  else {
    // if it's been 30 minutes, restart
    if (lastRestartTime <= currentTime && currentTime - lastRestartTime > 30) {
      resetSystem();
    }
    if (lastRestartTime > currentTime && lastRestartTime - currentTime < 1410) {
      resetSystem();
    }
  }
}

void resetSystem() {
  debugBlink(0,3);
  setSimComFuntionality(1);
  setupSimCom();
  waitUntilNetworkConnected(120);

  lastRestartHour = getCurrentHourInt();
  lastRestartMinute = getCurrentMinuteInt();
}

void watchDogForTurnOffGPS() {
  // shut down GPS module after 10 minutes of inactivity

  if (lastGPSQueryMinute == -1)
    return;

  short currentMinuteInt = getCurrentMinuteInt();

  // if it's been > 10 min, take action (turn off gps to save power)

  // case 1 example: lastQuery = 5:10pm, current = 5:20pm
  if (lastGPSQueryMinute <= currentMinuteInt && currentMinuteInt - lastGPSQueryMinute > 10) {
      setGPS(false);
      lastGPSQueryMinute = -1;
  }

  // case 2 example: lastQuery = 5:55, current = 6:05pm
  if (lastGPSQueryMinute > currentMinuteInt && lastGPSQueryMinute - currentMinuteInt < 50) {
      setGPS(false);
      lastGPSQueryMinute = -1;
  }
}

void watchDogForKillSwitch() {
  setKillSwitchPins(isActive(KILLSWITCHENABLED_BOOL_1, KILLSWITCHSTART_CHAR_3, KILLSWITCHEND_CHAR_3));

  if (startAttemptedWhileKillSwitchOnVolatile) {
    char ownerPhoneNumber[15];
    EEPROM.get(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);

    // whether sendSMS() is successful or not, set to false so we don't endlessly retry sending (could be bad if vehicle is out of cell range)
    sendSMS(ownerPhoneNumber, F("WARNING!\\nStart attempted while kill switch enabled"));
    startAttemptedWhileKillSwitchOnVolatile = false;
  }
}

void watchDogForGeofence() {
  bool follow;
  EEPROM.get(GEOFENCEFOLLOW_BOOL_1, follow);

  if (follow) {
    sendGeofenceWarning(true);
    return;
  }

  bool geofenceActive = isActive(GEOFENCEENABLED_BOOL_1, GEOFENCESTART_CHAR_3, GEOFENCEEND_CHAR_3);
  setGeofencePins(geofenceActive);

  if (!geofenceActive) {
    lastGeofenceWarningMinute = -1;
    return;
  }

  short currentMinuteInt = getCurrentMinuteInt();

  // If the geofence was broken...
  if (lastGeofenceWarningMinute != -1) {

    // ...only send a warning SMS every 5 min.  User can use follow mode if she wants rapid updates.

    // There are 2 cases:
    // A) current minute > last query minute, example lastQuery = 10, current = 20
    if (lastGeofenceWarningMinute <= currentMinuteInt && currentMinuteInt - lastGeofenceWarningMinute < 5) {
      return;
    }
  
    // B) current minute < last query minute, example lastQuery = 57, current = 05
    if (lastGeofenceWarningMinute > currentMinuteInt && lastGeofenceWarningMinute - currentMinuteInt > 55) {
      return;
    }
  }

  char currentLat[12];
  char currentLon[12];

  getGPSLatLon(currentLat, currentLon);

  if (outsideGeofence(currentLat, currentLon)) {
    sendGeofenceWarning(false, currentLat, currentLon);
  }
}

void sendGeofenceWarning(bool follow) {
  char currentLat[12];
  char currentLon[12];

  getGPSLatLon(currentLat, currentLon);
  sendGeofenceWarning(follow, currentLat, currentLon);
}

void sendGeofenceWarning(bool follow, char* currentLat, char* currentLon) {
  char geofenceHomeLat[12];
  char geofenceHomeLon[12];
  char ownerPhoneNumber[15];
  EEPROM.get(GEOFENCEHOMELAT_CHAR_12, geofenceHomeLat);
  EEPROM.get(GEOFENCEHOMELON_CHAR_12, geofenceHomeLon);
  EEPROM.get(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);

  char message[139];      // SMS max len = 140

  if (follow)
    strcpy_P(message, PSTR("FOLLOW MODE"));
  else
    strcpy_P(message, PSTR("GEOFENCE WARNING!"));

  strcat_P(message, PSTR("\\nCurrent:\\ngoogle.com/search?q="));
  strcat(message, currentLat);
  strcat_P(message, PSTR(","));
  strcat(message, currentLon);
  strcat_P(message, PSTR("\\nHome:\\ngoogle.com/search?q="));
  strcat(message, geofenceHomeLat);
  strcat_P(message, PSTR(","));
  strcat(message, geofenceHomeLon);

  sendSMS(ownerPhoneNumber, message);
  // we only want to send this message the first time the geofence is broken
  if (lastGeofenceWarningMinute == -1 && !follow) {
    sendSMS(ownerPhoneNumber, F("Use \"follow enable\" to receive rapid location updates (\"follow disable\" to stop)"));
  }

  lastGeofenceWarningMinute = getCurrentMinuteInt();
}

void checkSMSInput() {
  int8_t numberOfSMSs;

  numberOfSMSs = fona.getNumSMS();
  debugPrint(F("Number SMSs: ")); debugPrintln(numberOfSMSs);

  if (numberOfSMSs > 0)
    handleSMSInput();
}

void handleSMSInput() {
  short numberOfSMSs = fona.getNumSMS();

  char smsSender[15];
  char smsValue[51];
  int8_t smssFound = 0;

  for (int8_t smsSlotNumber = 0; smssFound < numberOfSMSs; smsSlotNumber++) {
    // SimCom module has 10 slots
    if (smsSlotNumber >= 10)
      break;

    if (isSMSSlotFilled(smsSlotNumber))
      smssFound++;
    else
      continue;

    getSMSSender(smsSlotNumber, smsSender);
    getSMSValue(smsSlotNumber, smsValue);

    toLower(smsValue);

    debugPrintln(F("--read SMS--"));
    debugPrintln(smsSender);
    debugPrintln(smsValue);

    if (strcmp_P(smsValue, PSTR("unlock")) == 0) {
      if (handleUnlockReq(smsSender))
        deleteSMS(smsSlotNumber);
      continue;
    }

    if (strcmp_P(smsValue, PSTR("lock")) == 0) {
      if (handleLockReq(smsSender))
        deleteSMS(smsSlotNumber);
      continue;
    }

    if (strcmp_P(smsValue, PSTR("loc")) == 0) {
      if (handleLocReq(smsSender))
        deleteSMS(smsSlotNumber);
      continue;
    }

    if (strstr_P(smsValue, PSTR("both"))) {
      if (checkLockdownStatus(smsSender, smsValue, smsSlotNumber))
        continue;

      if (handleBothReq(smsSender, smsValue))
        deleteSMS(smsSlotNumber);
      continue;
    }    

    if (strstr_P(smsValue, PSTR("kill"))) {
      if (checkLockdownStatus(smsSender, smsValue, smsSlotNumber))
        continue;

      if (handleKillSwitchReq(smsSender, smsValue, false))
        deleteSMS(smsSlotNumber);
      continue;
    }

    if (strstr_P(smsValue, PSTR("fence"))) {
      if (checkLockdownStatus(smsSender, smsValue, smsSlotNumber))
        continue;

      if (handleGeofenceReq(smsSender, smsValue, false))
        deleteSMS(smsSlotNumber);
      continue;
    }

    if (strstr_P(smsValue, PSTR("follow"))) {
      if (handleFollowReq(smsSender, smsValue))
        deleteSMS(smsSlotNumber);
      continue;
    }

    if (strstr_P(smsValue, PSTR("owner"))) {
      if (handleOwnerReq(smsSender, smsValue))
        deleteSMS(smsSlotNumber);
      continue;
    }

    if (strstr_P(smsValue, PSTR("devkey"))) {
      // special: we want to pass the case-sensitive version to handleDevKeyReq because the devKey is case sensitive
      getSMSValue(smsSlotNumber, smsValue);
      handleDevKeyReq(smsSender, smsValue);
      deleteSMS(smsSlotNumber);
      continue;
    }

    if (strcmp_P(smsValue, PSTR("status")) == 0) {
      if (handleStatusReq(smsSender))
        deleteSMS(smsSlotNumber);
      continue;
    }

    if (strcmp_P(smsValue, PSTR("commands")) == 0) {
      if (handleCommandsMessagesReq(smsSender))
        deleteSMS(smsSlotNumber);
      continue;
    }

    if (strcmp_P(smsValue, PSTR("deleteallmessages")) == 0) {
      fona.deleteAllSMS();
      return;  // notice this is RETURN not continue!
    }

    // default
    if (handleUnknownReq(smsSender))
      deleteSMS(smsSlotNumber);
  }
}

bool checkLockdownStatus(char* smsSender, char* smsValue, int8_t smsSlotNumber) {
  char message[135];
  char geofenceHomeLat[12];
  char geofenceHomeLon[12];
  char geofenceRadius[7];
  bool lockdownEnabled;

  // check if lockdown is ENabled
  EEPROM.get(LOCKDOWNENABLED_BOOL_1, lockdownEnabled);  

  if (lockdownEnabled) {
    EEPROM.get(GEOFENCEHOMELAT_CHAR_12, geofenceHomeLat);
    EEPROM.get(GEOFENCEHOMELON_CHAR_12, geofenceHomeLon);
    EEPROM.get(GEOFENCERADIUS_CHAR_7, geofenceRadius);
    
    strcpy_P(message, PSTR("Lockdown Enabled. Try \"unlock\" before updating fence or kill\\nRadius: "));
    strcat(message, geofenceRadius);
    strcat_P(message, PSTR(" feet\\nHome: google.com/search?q="));
    strcat(message, geofenceHomeLat);
    strcat_P(message, PSTR(","));
    strcat(message, geofenceHomeLon);

    if (sendSMS(smsSender, message))
      deleteSMS(smsSlotNumber);
    return true;
  }
  return false;
}
      
bool handleLockReq(char* smsSender) {

  char message[123];
  char geofenceHomeLat[12];
  char geofenceHomeLon[12];
  char geofenceRadius[7];
  bool lockdownEnabled;

  // check if lockdown is ALREADY ENabled
  EEPROM.get(LOCKDOWNENABLED_BOOL_1, lockdownEnabled);

  if (lockdownEnabled) {
    EEPROM.get(GEOFENCEHOMELAT_CHAR_12, geofenceHomeLat);
    EEPROM.get(GEOFENCEHOMELON_CHAR_12, geofenceHomeLon);
    EEPROM.get(GEOFENCERADIUS_CHAR_7, geofenceRadius);
  }
  else {
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
    EEPROM.get(GEOFENCERADIUS_CHAR_7, geofenceRadius);
    EEPROM.put(GEOFENCERADIUS_CHAR_SAVED_7, geofenceRadius);
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
    strcpy_P(geofenceRadius, PSTR("500"));
    EEPROM.put(GEOFENCERADIUS_CHAR_7, geofenceRadius);
    EEPROM.put(GEOFENCESTART_CHAR_3, "00");
    EEPROM.put(GEOFENCEEND_CHAR_3, "00");
    EEPROM.put(KILLSWITCHENABLED_BOOL_1, true);
    EEPROM.put(KILLSWITCHSTART_CHAR_3, "00");
    EEPROM.put(KILLSWITCHEND_CHAR_3, "00");
  
    // set lockdown variable ON
    EEPROM.put(LOCKDOWNENABLED_BOOL_1, true);
  }

  // send SMS with new geofence home
  strcpy_P(message, PSTR("Lockdown: Enabled\\nRadius: "));
  strcat(message, geofenceRadius);
  strcat_P(message, PSTR(" feet\\nHome: google.com/search?q="));
  strcat(message, geofenceHomeLat);
  strcat_P(message, PSTR(","));
  strcat(message, geofenceHomeLon);

  return sendSMS(smsSender, message);
}

bool handleUnlockReq(char* smsSender) {

  char message[123];
  char geofenceHomeLat[12];
  char geofenceHomeLon[12];
  char geofenceRadius[7];
  bool lockdownEnabled;

  // check if lockdown is ALREADY DISabled
  EEPROM.get(LOCKDOWNENABLED_BOOL_1, lockdownEnabled);

  if (!lockdownEnabled) {
    EEPROM.get(GEOFENCEHOMELAT_CHAR_SAVED_12, geofenceHomeLat);
    EEPROM.get(GEOFENCEHOMELON_CHAR_SAVED_12, geofenceHomeLon);
    EEPROM.get(GEOFENCERADIUS_CHAR_SAVED_7, geofenceRadius);
  }
  else {  
    bool geofenceEnabled;
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
    EEPROM.get(GEOFENCERADIUS_CHAR_SAVED_7, geofenceRadius);
    EEPROM.put(GEOFENCERADIUS_CHAR_7, geofenceRadius);
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

  // send SMS with original geofenceHome
  strcpy_P(message, PSTR("Lockdown: Disabled\\nRadius: "));
  strcat(message, geofenceRadius);
  strcat_P(message, PSTR(" feet\\nHome: google.com/search?q="));
  strcat(message, geofenceHomeLat);
  strcat_P(message, PSTR(","));
  strcat(message, geofenceHomeLon);

  return sendSMS(smsSender, message);
}

bool handleStatusReq(char* smsSender) {
  uint8_t rssi;
  char rssiStr[4];
  char ccid[22];
  char currentTimeStr[23];
  char message[141];

  char ownerPhoneNumber[15] = "";
  EEPROM.get(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);

  bool lockdown;
  char lockdownStr[9];
  EEPROM.get(LOCKDOWNENABLED_BOOL_1, lockdown);
  
  if (lockdown)
    strcpy_P(lockdownStr, PSTR("Enabled"));
  else
    strcpy_P(lockdownStr, PSTR("Disabled"));

  rssi = fona.getRSSI();
  itoa(rssi, rssiStr, 10);
  fona.getSIMCCID(ccid);

  getTime(currentTimeStr);

  strcpy_P(message, PSTR("Owner: "));
  strcat(message, ownerPhoneNumber);
  strcat_P(message, PSTR("\\nLockdown: "));
  strcat(message, lockdownStr);
  strcat_P(message, PSTR("\\nRSSI: "));
  strcat(message, rssiStr);
  strcat_P(message, PSTR("\\nCCID: "));
  strcat(message, ccid);
  strcat_P(message, PSTR("\\nNetwork Time: "));
  strcat(message, currentTimeStr);
  return sendSMS(smsSender, message);
}

bool handleLocReq(char* smsSender) {
  char message[54];
  char latitude[12];
  char longitude[12];

  getGPSLatLon(latitude, longitude);

  strcpy_P(message, PSTR("google.com/search?q="));
  strcat(message, latitude);
  strcat_P(message, PSTR(","));
  strcat(message, longitude);
  return sendSMS(smsSender, message);
}

bool handleFollowReq(char* smsSender, char* smsValue) {
  if (strstr_P(smsValue, PSTR("enable"))) {
    EEPROM.put(GEOFENCEFOLLOW_BOOL_1, true);
    return sendSMS(smsSender, F("Follow: ENABLED"));
  }
  if (strstr_P(smsValue, PSTR("disable"))) {
    EEPROM.put(GEOFENCEFOLLOW_BOOL_1, false);
    return sendSMS(smsSender, F("Follow: DISABLED"));
  }
  return sendSMS(smsSender, F("Try \"follow\" plus:\\nenable/disable"));  
}

bool handleBothReq(char* smsSender, char* smsValue) {
  return handleKillSwitchReq(smsSender, smsValue, true) && handleGeofenceReq(smsSender, smsValue, true);
}

bool handleKillSwitchReq(char* smsSender, char* smsValue, bool alternateSMSOnFailure) {
  char message[65] = "";

  bool validMessage = false;
  bool killSwitchEnabled;
  char killSwitchStart[3];
  char killSwitchEnd[3];
  EEPROM.get(KILLSWITCHSTART_CHAR_3, killSwitchStart);
  EEPROM.get(KILLSWITCHEND_CHAR_3, killSwitchEnd);

  validMessage = setEnableAndHours(smsValue, KILLSWITCHENABLED_BOOL_1, KILLSWITCHSTART_CHAR_3, KILLSWITCHEND_CHAR_3, killSwitchEnabled, killSwitchStart, killSwitchEnd);

  if (validMessage || strstr_P(smsValue, PSTR("status"))) {
    //  Yay only 2k of RAM
    strcpy_P(message, PSTR("Kill: "));
    if (killSwitchEnabled)
      strcat_P(message, PSTR("En"));
    else
      strcat_P(message, PSTR("Dis"));

    strcat_P(message, PSTR("abled\\nHours: "));
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

    strcpy_P(message, PSTR("Try \""));
    if (alternateSMSOnFailure) {
      strcat_P(message, PSTR("both"));
    }
    else {
      strcat_P(message, PSTR("kill"));
    }
    strcat_P(message, PSTR("\" plus:\\nenable/disable\\nstatus\\nhours 0 21 (12am-9pm)"));
    return sendSMS(smsSender, message);
  }
}

bool handleGeofenceReq(char* smsSender, char* smsValue, bool alternateSMSOnFailure) {
  char message[135] = "";

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
  
  if (strstr_P(smsValue, PSTR("radius"))) {
    geofenceRadius[0] = '\0';
    if (getNumberFromString(smsValue, geofenceRadius, 7)) {
      EEPROM.put(GEOFENCERADIUS_CHAR_7, geofenceRadius);
      validMessage = true;
    }
  }
  if (strstr_P(smsValue, PSTR("home"))) {
    getGPSLatLon(geofenceHomeLat, geofenceHomeLon);
    EEPROM.put(GEOFENCEHOMELAT_CHAR_12, geofenceHomeLat);
    EEPROM.put(GEOFENCEHOMELON_CHAR_12, geofenceHomeLon);
    validMessage = true;
  }

  // reset this so the "follow" message will be sent when the fence is broken
  lastGeofenceWarningMinute = -1;

  if (validMessage || strstr_P(smsValue, PSTR("status"))) {
    //    Yay only 2k of RAM, doing this all piecemeal (instead of big strings as is done in this comment) saves 54 bytes!!!!!!!
    //    if (geofenceEnabled)
    //      sprintf(message, "ENABLED\nHours: %s-%s\nRadius: %s feet\nHome: google.com/search?q=%s,%s", geofenceStart, geofenceEnd, geofenceRadius, geofenceHomeLat, geofenceHomeLon);
    //    else
    //      sprintf(message, "DISABLED\nHours: %s-%s\nRadius: %s feet\nHome: google.com/search?q=%s,%s", geofenceStart, geofenceEnd, geofenceRadius, geofenceHomeLat, geofenceHomeLon);

    strcpy_P(message, PSTR("Fence: "));
    if (geofenceEnabled)
      strcat_P(message, PSTR("En"));
    else
      strcat_P(message, PSTR("Dis"));

    strcat_P(message, PSTR("abled\\nHours: "));
    strcat(message, geofenceStart);
    strcat_P(message, PSTR("-"));
    strcat(message, geofenceEnd);
    if (strcmp(geofenceStart, geofenceEnd) == 0) {  // if start time == end time  
      strcat_P(message, PSTR(" (always on)"));
    }
    strcat_P(message, PSTR("\\nRadius: "));
    strcat(message, geofenceRadius);
    strcat_P(message, PSTR(" feet\\nHome: google.com/search?q="));
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
      return sendSMS(smsSender, F("Try \"fence\" plus:\\nenable/disable\\nstatus\\nhours 0 21 (12am-9pm)\\nhome (uses current loc)\\nradius 300 (300 feet)"));
    }
  }
}

bool handleOwnerReq(char* smsSender, char* smsValue) {
  char message[92] = "";
  char ownerPhoneNumber[15] = "";

  // set owner number
  if (strstr_P(smsValue, PSTR("set"))) {
    if (getNumberFromString(smsValue, ownerPhoneNumber, 15))  // If number is found in the SMS,
      addPlusToPhoneNumber(ownerPhoneNumber);                 // add a '+' to the beginning...
    else
      strcpy(ownerPhoneNumber, smsSender);

    strcpy_P(message, PSTR("Setting owner phone # to "));
    strcat(message, &ownerPhoneNumber[1]);
    EEPROM.put(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);
  }

  // just respond with current owner number
  else {
    EEPROM.get(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);
    strcpy(message, &ownerPhoneNumber[1]);
    strcat_P(message, PSTR("\\nTry \"owner\" plus:\\nset (include number *with country code* or leave blank for YOUR number)"));
  }

  return sendSMS(smsSender, message);
}

bool handleDevKeyReq(char* smsSender, char* smsValue) {

  // special: smsValue is still case-sensitive.
  // To save memory, tempStr will be used for the toLower() version of smsValue as well as the message we send back
  char tempStr[46] = "";
  strcpy(tempStr, smsValue);
  toLower(tempStr);
  
  char devKey[9] = "";

  // set devKey
  if (strstr_P(tempStr, PSTR("devkey set"))) {
    getOccurrenceInDelimitedString(smsValue, devKey, 3, ' ', 8); // max_length
    EEPROM.put(DEVKEY_CHAR_9, devKey);

    strcpy_P(tempStr, PSTR("Setting devKey to "));
    strcat(tempStr, devKey);
  }

  // just respond with current devKey
  else {
    EEPROM.get(DEVKEY_CHAR_9, devKey);
    strcpy(tempStr, devKey);
    strcat_P(tempStr, PSTR("\\nTry \"devkey set ________\""));
  }

  return sendSMS(smsSender, tempStr);
}

bool handleCommandsMessagesReq(char* smsSender) {
  return sendSMS(smsSender, F("Commands:\\nfence\\nfollow\\nstatus\\nkill\\nloc\\nowner\\nlock\\nunlock"));
}

bool handleUnknownReq(char* smsSender) {
  char ownerPhoneNumber[15];
  EEPROM.get(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);

  // if they're not the owner, don't send them the commands, just return true so their msg will be deleted
  if (strcmp(smsSender, ownerPhoneNumber) == 0)
    return handleCommandsMessagesReq(smsSender);
  else
    return true;
}


////////////////////////////////
//GPS

bool setGPS(bool tf) {
  // turns SimCom GPS on or off (don't waste power)
  if (!tf) {
    if (fona.enableGPSSIM7000(false)) {
      debugBlink(1,10);
      return true;
    }
    debugBlink(1,9);
    debugPrintln(F("Failed to turn off GPS"));
    return false;
  }

  // -1 = error querying GPS
  //  0 = GPS off
  //  1 = no GPS fix
  //  2 = 2D fix
  //  3 = 3D fix
  if (fona.GPSstatusSIM7000() >= 2)
    return true;

  int8_t status;

  // Keep trying to get a valid (non-error) response. Maybe we should turn off/on?
  for (int i = 1; i < 30; i++) {
    if (fona.GPSstatusSIM7000() >= 0) {
      break;
    }
    delay(2000);
  }

  // error, give up
  if (fona.GPSstatusSIM7000() < 0) {
    debugPrintln(F("Failed to turn on GPS"));
    debugBlink(1,5);
    return false;
  }

  // turn on
  fona.enableGPSSIM7000(true);
  delay(4000);

  
  char currentLat[12];
  char currentLon[12];

  // wait up to 90s to get GPS fix
  for (int j = 0; j < 23; j++) {
    if (fona.GPSstatusSIM7000() >= 2) {
      debugBlink(1,8);

      // I really hate to do this, but the first GPS response is sometimes WAY off (> 200 feet) and you get a geofence warning...
      getGPSLatLon(currentLat, currentLon);
      delay(3000);
      getGPSLatLon(currentLat, currentLon);
      return true;
    }
    delay(4000);
  }

  // no fix, give up
  debugPrintln(F("Failed to get GPS fix"));
  debugBlink(1,7);
  return false;
}

bool getGPSLatLon(char* latitude, char* longitude) {
  char gpsString[120];

  if (setGPS(true)){
    // full string:
    // 1,1,20190913060459.000,30.213823,-97.782017,204.500,1.87,90.1,1,,1.2,1.5,0.9,,11,6,,,39,,
    for (short i = 0; i < 10; i++) {
      fona.getGPS(0, gpsString, 120);
      lastGPSQueryMinute = getCurrentMinuteInt();
  
      getOccurrenceInDelimitedString(gpsString, latitude, 4, ',');
      getOccurrenceInDelimitedString(gpsString, longitude, 5, ',');
  
      // we have see errors where the lat,long come back as garbage like "9,43"
      if (strlen(latitude) > 7 && strlen(longitude) > 7 && strchr(latitude, '.') != NULL && strchr(longitude, '.') != NULL) {
        return true;
      }
    }
  }

  latitude[0] = '\0';
  longitude[0] = '\0';
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

  debugPrint(F("Distance in feet: "));
  debugPrintln(dist_calc, 2);

  return dist_calc > geofenceRadiusFloat;
}

////////////////////////////////
//Time

void getTime(char* currentTimeStr) {

  // sets currentTime to "19/09/19,17:03:55-20" INCLUDING quotes  
  for (short i = 0; i < 3; i++) {
    fona.getTime(currentTimeStr, 23);

    // if time string looks good...
    if (currentTimeStr[0] == '"')
      return;

    // else, try simple self-healing.  If echo is on, basically all commands to SIM7000 won't work.
    fona.setEchoOff();
    delay(1000);
  }
}

int getCurrentMinuteInt() {
  char currentTimeStr[23];
  char currentMinuteStr[3];
  
  getTime(currentTimeStr);

  currentMinuteStr[0] = currentTimeStr[13];
  currentMinuteStr[1] = currentTimeStr[14];
  currentMinuteStr[2] = '\0';
  return atoi(currentMinuteStr);
}

int getCurrentHourInt() {
  char currentTimeStr[23];
  char currentHourStr[3];
  
  getTime(currentTimeStr);

  // QUOTES ARE PART OF THE STRING: "19/09/19,17:03:01-20"
  currentHourStr[0] = currentTimeStr[10];
  currentHourStr[1] = currentTimeStr[11];
  currentHourStr[2] = '\0';
  return atoi(currentHourStr);
}

bool isActive(short eepromEnabled, short eepromStart, short eepromEnd) {
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

  // if start and end are the same, the fence is ALWAYS on
  if (strcmp(startHour, endHour) == 0)
    return true;

  short currentHour = getCurrentHourInt();

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
  if (simComConnectionStatus == 1) {
    return;
  }

  // sim7000 can only hold 10 messages, it cannot see the rest until those 10 are processed.  That means if we are debugging
  // and send "deleteallmessages" and there are already 10 queue'd up, sim7000 will never see the "deleteallmessages" message.
  // SO, if we start up and there are 10 messages, 99% of the time that means one of them is causing problems.
  // This should never happen, but allows turning off/on to clear out messages if "deleteallmessages" isn't working.
  short numberOfSMSs = fona.getNumSMS();
  if (numberOfSMSs == 10) {
    fona.deleteAllSMS();
    debugPrintln(F("SMS storage FULL. Deleting ALL SMS"));
  }
}

void deleteSMS(uint8_t msg_number) {
  for (int i = 0; i < 5; i++) {
    debugPrintln(F("  Attempting to delete SMS"));
    if (fona.deleteSMS(msg_number)) {
      debugPrintln(F("  Success deleting SMS"));
      debugBlink(1,4);
      return;
    }
    delay(2000);
  }
  debugPrintln(F("  Failed to delete SMS"));
  debugBlink(1,3);
}

bool sendSMS(char* send_to, char* message) {

  if (simComConnectionStatus > 0)
    return false;

  // Example of hologramSMSString: "Saaaabbbb+15556667777 Hello, SMS over IP World!"
  // 'S' (tells Hologram that it's an SMS):               1
  // devKey:                                              8
  // phone number including '+' and 3-digit country code: 14
  // ' ' (required by Hologram):                          1
  // the message:                                         140
  // '\0' (it's null-terminated C string!):               1

  char hologramSMSString[165];
  uint16_t hologramSMSStringLength;

  char devKey[9] = "";
  char serverName[24] = "";
  uint16_t serverPort;
  EEPROM.get(DEVKEY_CHAR_9, devKey);
  EEPROM.get(SERVERNAME_CHAR_24, serverName);
  EEPROM.get(SERVERPORT_INT_2, serverPort);

  // 00000000 is the default devKey (comes from initEEPROM)
  // we delete the incoming SMS so we don't try to send the msg indefinitely
  if (strstr_P(devKey, PSTR("00000000"))) {
    debugBlink(2,9);
    debugPrintln(F("DEVKEY NOT SET! DELETING SMS!"));
    return true;
  }
  
  strcpy_P(hologramSMSString, PSTR("S"));
  strcat(hologramSMSString, devKey);
  strcat(hologramSMSString, send_to);
  strcat_P(hologramSMSString, PSTR(" "));
  strcat(hologramSMSString, message);
  hologramSMSStringLength = strlen(hologramSMSString);
  
  debugPrintln(F("  Attempting to send SMS:"));
  debugPrintln(serverName);
  debugPrintln(serverPort);
  debugPrintln(hologramSMSStringLength);
  debugPrintln(hologramSMSString);

  uint16_t successCode = fona.ConnectAndSendToHologram(serverName, serverPort, hologramSMSString, hologramSMSStringLength);

  debugPrint(F("  Success code: "));
  itoa(successCode, serverName, 10);
  debugPrintln(serverName);
  fona.TCPshut();

  if (successCode == 0) {
    debugPrintln(F("  Success sending SMS"));
    debugBlink(1,2);
    return true;
  } else {
    debugPrintln(F("  Failed to send SMS"));
    // see very top for debug blink code meanings (which in this case are coming from the cellular module
    debugBlink(2,successCode);
    return false;
  }
}

bool sendSMS(char* send_to, const __FlashStringHelper* messageInProgmem) {
  char message[141];  // yeah this sucks, but it's better than having all those strings stored in SRAM as globals
  strcpy_P(message, (const char*)messageInProgmem);
  return sendSMS(send_to, message);
}

bool isSMSSlotFilled(int8_t smsSlotNumber) {
  char smsSender[15];

  if (fona.getSMSSender(smsSlotNumber, smsSender, 15)) {
    return true;
  }
  return false;
}

void getSMSSender(int8_t smsSlotNumber, char* smsSender) {
  for (int i = 0; i < 3; i++) {
    if (fona.getSMSSender(smsSlotNumber, smsSender, 15))
      break;
    debugPrintln(F("  Failed getting SMS sender"));
    delay(1000);
  }
}

void getSMSValue(int8_t smsSlotNumber, char* smsValue) {
  uint16_t smsValueLength;

  for (int i = 0; i < 3; i++) {
    if (fona.readSMS(smsSlotNumber, smsValue, 50, &smsValueLength))
      break;
    debugPrintln(F("  Failed getting SMS value"));
    delay(1000);
  }
}




///////////////////////////////////////////////////////////////////////////////////////////
//HELPERS
///////////////////////////////////////////////////////////////////////////////////////////
void debugBlink(short longBlinks, short shortBlinks) {
  for (int cnt=0; cnt < longBlinks; cnt++) {
    digitalWrite(13, HIGH);
    delay(1000);
    digitalWrite(13, LOW);
    delay(300);    
  }
  for (int cnt=0; cnt < shortBlinks; cnt++) {
    digitalWrite(13, HIGH);
    delay(50);
    digitalWrite(13, LOW);
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

  // if atoi can't find an translation (it returns 0) AND the first character isn't '0' - then something's wrong
  if ((atoi(hoursStart) == 0 && hoursStart[0] != '0') || (atoi(hoursEnd) == 0 && hoursEnd[0] != '0')) {
    return false;
  }

  return (hoursStart[0] && hoursEnd[0]);
}

bool setEnableAndHours(char* smsValue, short eepromEnabled, short eepromStart, short eepromEnd, bool &enabled, char* hoursStart, char* hoursEnd) {
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

void writeCStringToEEPROM(int eepromAddress, char* data) {
  int i=0;
  for (;data[i]; i++) {
    EEPROM.put(eepromAddress+i, data[i]);
  }
  EEPROM.put(eepromAddress+i, '\0');
}

void setSimComFuntionality(short func) {
  if (func == 0) {
    setGPS(false);
    sendRawCommand(F("AT+CFUN=0,0"));
  }
  if (func == 1)
    sendRawCommand(F("AT+CFUN=1,1"));
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

void sendRawCommand(const __FlashStringHelper* command) {
  delay(200);
  debugPrintln(command);
  fona.println(command);
  delay(1000);

  if (fona.available()) {
    flushSimCom();
  }
  delay(1000);
}

void toLower(char* str) {
  for (int i = 0; str[i]; i++)
    str[i] = tolower(str[i]);
}

void addPlusToPhoneNumber(char* phoneNumber) {
  // in:  "15554443333\0\0\0\0"
  // out: "+15554443333\0\0\0"
  phoneNumber[14] = '\0';  // phoneNumber is of size 15

  for (int i = 14; i > 0; i--) {
    phoneNumber[i] = phoneNumber[i-1];
  }
  phoneNumber[0] = '+';
}

bool getNumberFromString(char* in, char* out, short maxLen) {
  // if in == "aaa+123b-b4b5"
  // then out = "12345"
  // be sure to leave room for '\0'
  bool foundNumber = false;
  short outCount = 0;

  for (int i = 0; in[i] && outCount < maxLen-1; i++) {
    if (in[i] >= '0' && in[i] <= '9') {
      foundNumber = true;
      out[outCount] = in[i];
      outCount++;
    }
  }
  out[outCount] = '\0';

  return foundNumber;
}

void cleanString(char* stringToClean, char charToClean) {
  // in:  "  some  string   with stuff  "
  // out: "some string with stuff"
  short outCount = 0;
  bool lastCharWasCharToClean = false;
  
  for (int i = 0; stringToClean[i]; i++) {
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

bool getOccurrenceInDelimitedString(char* in, char* out, short occurrenceNumber, char delim) {
  // yeah I know.  Trying to save program space.
  return getOccurrenceInDelimitedString(in, out, occurrenceNumber, delim, 9999);
}

bool getOccurrenceInDelimitedString(char* in, char* out, short occurrenceNumber, char delim, short maxLength) {
  // if in == "a,b,c"
  // and occurrenceNumber = 2
  // and delim = ','
  // sets out = "b"
  // be sure to leave room for '\0'

  // maxLength is due to the bug in the SIM7000A for strings like "aa{a" it'll tell us it's 5 chars long. See https://forum.arduino.cc/index.php?topic=660925.0
  short delimCount = 0;
  short outCount = 0;
  bool foundOccurrence = false;

  cleanString(in, delim);

  for (int i = 0; in[i] && outCount < maxLength; i++) {
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
  
  pinMode(GEOFENCE_LED_PIN, OUTPUT);
  pinMode(KILL_SWITCH_RELAY_PIN, OUTPUT);
  pinMode(KILL_SWITCH_LED_PIN, OUTPUT);
  pinMode(DEBUG_PIN, OUTPUT);
}

void starterISR() {
  _delay_ms(500);  // on some starters, turning to the key to the "accessory" mode might jump to 12V for just a few milliseconds, so let's wait - make sure someone is actually trying to start the car

  // when starter is on, PIN is LOW
  if (killSwitchOnVolatile && !digitalRead(STARTER_INTERRUPT_PIN))
    startAttemptedWhileKillSwitchOnVolatile = true;
}

void setKillSwitchPins(bool tf) {
  killSwitchOnVolatile = tf;
  digitalWrite(KILL_SWITCH_RELAY_PIN, tf);
  digitalWrite(KILL_SWITCH_LED_PIN, tf);
}

void setGeofencePins(bool tf) {
  digitalWrite(GEOFENCE_LED_PIN, tf);
}

void setupSimCom() {
  debugPrint(F("Connect to SimCom"));
  // let SimCom module start up before we try to connect
  delay(5000);

  SimComSerial->begin(9600);
  fona.beginSIM7000(*SimComSerial);
  
  for (int i = 0; i < 30; i++) {
    debugPrint(F("."));
    if (fona.getNumSMS() >= 0) {
      debugPrintln(F("\nSimCom OK"));
      simComConnectionStatus = 2;
      debugBlink(0,4);
      return;
    }
    delay(2000);
  }
  simComConnectionStatus = 1;
}

void waitUntilNetworkConnected(short secondsToWait) {
  // no point trying to conenct to network if we can't connect to SimCom
  if (simComConnectionStatus == 1) {
    return;
  }

  debugPrint(F("Connect to network"));
  short netConn;

  fona.setNetworkSettings(APN, F(""), F(""));

  // we're waiting 2s each loop
  secondsToWait = secondsToWait/2;
  
  for (int i = 0; i < secondsToWait; i++) {
    debugPrint(F("."));
    netConn = fona.getNetworkStatusSIM7000();
    
    // 0 Not registered, not currently searching an operator to register to, the GPRS service is disabled
    // 1 Registered, home
    // 2 Not registered, trying to attach or searching an operator to register to
    // 3 Registration denied
    // 4 Unknown
    // 5 Registered, roaming
    if (netConn == 1 || netConn == 5) {
      debugPrintln(F("\nConnected"));
      simComConnectionStatus = 0;
      fona.setNetworkSettings(APN, F(""), F(""));
      fona.TCPshut();  // just in case GPRS is still on for some reason, save power
      return;
    }
    delay(2000);
  }

  // 0 is good, so set it to 2 (not registered)
  if (netConn == 0)
    netConn = 2;

  simComConnectionStatus = netConn;
  setSimComFuntionality(0);
}

#if defined VAN_TEST || defined NEW_HARDWARE_ONLY
void setupSerial() {
  while (!Serial);
  Serial.begin(9600);
}
#endif


#ifdef NEW_HARDWARE_ONLY
void initBaud() {
  debugPrintln(F("Init Baud"));
  delay(2000);
  debugPrintln(F("Trying to connect at 9600"));
  SimComSerial->begin(9600);

  if (! fona.beginSIM7000(*SimComSerial)) {
    debugPrintln(F("Trying to connect at 115200"));
    SimComSerial->begin(115200);
    
    if (! fona.beginSIM7000(*SimComSerial)) {
      debugPrintln(F("ERROR: Could not connect at 9600 or 115200"));
      return;
    } else {
      debugPrintln(F("Connected at 115200, setting to 9600..."));
      sendRawCommand(F("AT+IPR=9600"));
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
  EEPROM.put(GEOFENCESTART_CHAR_3, "00");
  EEPROM.put(GEOFENCEEND_CHAR_3, "00");
  EEPROM.put(GEOFENCEFOLLOW_BOOL_1, false);
  EEPROM.put(KILLSWITCHENABLED_BOOL_1, false);
  EEPROM.put(KILLSWITCHSTART_CHAR_3, "00");
  EEPROM.put(KILLSWITCHEND_CHAR_3, "00");
  EEPROM.put(OWNERPHONENUMBER_CHAR_15, "+15551234567");
  
  EEPROM.put(LOCKDOWNENABLED_BOOL_1, false);
  EEPROM.put(GEOFENCEENABLED_BOOL_SAVED_1, false);
  EEPROM.put(GEOFENCEHOMELAT_CHAR_SAVED_12, "52.4322115");
  EEPROM.put(GEOFENCEHOMELON_CHAR_SAVED_12, "10.7869289");
  EEPROM.put(GEOFENCERADIUS_CHAR_SAVED_7, "500");
  EEPROM.put(GEOFENCESTART_CHAR_SAVED_3, "00");
  EEPROM.put(GEOFENCEEND_CHAR_SAVED_3, "00");
  EEPROM.put(KILLSWITCHENABLED_BOOL_SAVED_1, false);
  EEPROM.put(KILLSWITCHSTART_CHAR_SAVED_3, "00");
  EEPROM.put(KILLSWITCHEND_CHAR_SAVED_3, "00");
  
  EEPROM.put(DEVKEY_CHAR_9, "00000000");
  EEPROM.put(SERVERNAME_CHAR_24, SERVER_NAME);
  EEPROM.put(SERVERPORT_INT_2, SERVER_PORT);

  debugPrintln(F("End initEEPROM()"));
}

void initSimCom() {
  // used on brand-new SimCom module
  debugPrintln(F("Begin initSimCom()"));

  sendRawCommand(F("ATZ"));                 // Reset settings
  sendRawCommand(F("AT+CMEE=2"));           // Turn on verbose mode
  sendRawCommand(F("AT+CLTS=1"));           // Turn on "get clock when registering w/network" see https://forums.adafruit.com/viewtopic.php?f=19&t=58002
  sendRawCommand(F("AT+CNETLIGHT=1"));      // Turn on "net" LED
  sendRawCommand(F("AT+COPS=0"));           // Set Cellular OPerator Selection to "automatic"
  sendRawCommand(F("AT+CMEE=0"));           // Turn off verbose mode

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
#if defined VAN_TEST || defined NEW_HARDWARE_ONLY
  Serial.print(str);
#endif
}
void debugPrint(const __FlashStringHelper* str) {
#if defined VAN_TEST || defined NEW_HARDWARE_ONLY
  Serial.print(str);
#endif
}
void debugPrintln(char* str) {
#if defined VAN_TEST || defined NEW_HARDWARE_ONLY
  Serial.println(str);
#endif
}
void debugPrintln(const __FlashStringHelper* str) {
#if defined VAN_TEST || defined NEW_HARDWARE_ONLY
  Serial.println(str);
#endif
}
void debugPrintln(float f, int i) {
#if defined VAN_TEST || defined NEW_HARDWARE_ONLY
  Serial.println(f, i);
#endif
}
void debugPrintln(String s) {
#if defined VAN_TEST || defined NEW_HARDWARE_ONLY
  Serial.println(s);
#endif
}
void debugPrintln(uint16_t s) {
#if defined VAN_TEST || defined NEW_HARDWARE_ONLY
  Serial.println(s);
#endif
}
void debugPrintln(int8_t i) {
#if defined VAN_TEST || defined NEW_HARDWARE_ONLY
  Serial.println(i);
#endif
}
void debugPrintln(short s) {
#if defined VAN_TEST || defined NEW_HARDWARE_ONLY
  Serial.println(s);
#endif
}
void debugPrintln(bool b) {
#if defined VAN_TEST || defined NEW_HARDWARE_ONLY
  Serial.println(b);
#endif
}






#ifdef VAN_TEST


void putEEPROM() {
  EEPROM.put(GEOFENCEENABLED_BOOL_1, false);
  EEPROM.put(GEOFENCEHOMELAT_CHAR_12, "0.0");
  EEPROM.put(GEOFENCEHOMELON_CHAR_12, "0.0");

}


void getEEPROM() {
  char tempc[24];
  bool tempb;
  int tempi;

  EEPROM.get(GEOFENCEHOMELAT_CHAR_12, tempc);
  Serial.write ("GEOFENCEHOMELAT_CHAR_12: ");
  debugPrintln(tempc);
  EEPROM.get(GEOFENCEHOMELON_CHAR_12, tempc);
  Serial.write ("GEOFENCEHOMELON_CHAR_12: ");
  debugPrintln(tempc);
  EEPROM.get(GEOFENCEENABLED_BOOL_1, tempb);
  Serial.write ("GEOFENCEENABLED_BOOL_1: ");
  debugPrintln(tempb);
  EEPROM.get(GEOFENCESTART_CHAR_3, tempc);
  Serial.write ("GEOFENCESTART_CHAR_3: ");
  debugPrintln(tempc);
  EEPROM.get(GEOFENCEEND_CHAR_3, tempc);
  Serial.write ("GEOFENCEEND_CHAR_3: ");
  debugPrintln(tempc);
  EEPROM.get(GEOFENCERADIUS_CHAR_7, tempc);
  Serial.write ("GEOFENCERADIUS_CHAR_7: ");
  debugPrintln(tempc);
  EEPROM.get(GEOFENCEFOLLOW_BOOL_1, tempb);
  Serial.write ("GEOFENCEFOLLOW_BOOL_1: ");
  debugPrintln(tempb);
  EEPROM.get(KILLSWITCHENABLED_BOOL_1, tempb);
  Serial.write ("KILLSWITCHENABLED_BOOL_1: ");
  debugPrintln(tempb);
  EEPROM.get(KILLSWITCHSTART_CHAR_3, tempc);
  Serial.write ("KILLSWITCHSTART_CHAR_3: ");
  debugPrintln(tempc);
  EEPROM.get(KILLSWITCHEND_CHAR_3, tempc);
  Serial.write ("KILLSWITCHEND_CHAR_3: ");
  debugPrintln(tempc);
  EEPROM.get(OWNERPHONENUMBER_CHAR_15, tempc);
  Serial.write ("OWNERPHONENUMBER_CHAR_15: ");
  debugPrintln(tempc);
  EEPROM.get(DEVKEY_CHAR_9, tempc);
  Serial.write ("DEVKEY_CHAR_9: ");
  debugPrintln(tempc);
  EEPROM.get(SERVERNAME_CHAR_24, tempc);
  Serial.write ("SERVERNAME_CHAR_24: ");
  debugPrintln(tempc);
  EEPROM.get(SERVERPORT_INT_2, tempi);
  Serial.write ("SERVERPORT_INT_2: ");
  itoa(tempi, tempc, 10);
  debugPrintln(tempc);

  EEPROM.get(LOCKDOWNENABLED_BOOL_1, tempb);
  Serial.write ("LOCKDOWNENABLED_BOOL_1: ");
  debugPrintln(tempb);
}

void checkSerialInput() {
  if (!Serial.available()) {
    return;
  }

  String command;
  command = Serial.readString();
  debugPrintln(command);
  handleSerialInput(command);
}

void handleSerialInput(String command) {

  command.trim();
  char* temp = command.c_str();

  if (strcmp_P(temp, PSTR("g")) == 0) {
    getEEPROM();
  }
  if (strcmp_P(temp, PSTR("p")) == 0) {
    putEEPROM();
  }
  if (strcmp_P(temp, PSTR("w")) == 0) {
    watchDog();
  }

// for SERIAL TUBE:

// AT+CGNSPWR=1 - turn on gps
// AT+CGPSSTATUS? - get gps status
// AT+CGNSINF - get loc
// AT+CBC - battery
// AT+CMGF=1 - set text SMS mode
// AT+CPMS? - get number of SMSs
// AT&V - get profiles
// AT+CCLK? - get clock
// AT+CPOWD=1 - power down module

  // Allow user to directly enter serial commands to the SimCom chip
  if (strcmp_P(temp, PSTR("S")) == 0) {
    sendRawCommand(F("AT+CMEE=2"));
    sendRawCommand(F("ATE1"));
    
    delay(2000);    

    debugPrintln(F("Serial:"));
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

  if (strcmp_P(temp, PSTR("d")) == 0) {
    for (int i = 2; i < 10; i++) {
      debugPrintln(F("  Delete SMS:"));
      if (fona.deleteSMS(readnumber())) {
        debugPrintln(F("  Success"));
        return;
      }
      delay(i * 1000);
    }
    debugPrintln(F("  Failed"));
  }

  // Test incoming SMS, for example:
  // 5554443333_fence status
  if (command.length() > 2){
    char smsSender[15];
    char smsValue[51];
    getOccurrenceInDelimitedString(temp, smsSender, 1, '_');
    getOccurrenceInDelimitedString(temp, smsValue, 2, '_');
    testHandleSMSInput(smsSender, smsValue);
  }

  flushSerial();
  flushSimCom();
}

void testHandleSMSInput(char* smsSender, char* smsValue) {

  toLower(smsValue);

  debugPrintln(F("-read SMS-"));
  debugPrintln(smsSender);
  debugPrintln(smsValue);
  debugPrintln(F(""));

  if (strstr_P(smsValue, PSTR("gps")) || strstr_P(smsValue, PSTR("loc"))) {
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

char readBlocking() {
  while (!Serial.available());
  return Serial.read();
}

uint16_t readnumber() {
  uint16_t x = 0;
  char c;
  while (! isdigit(c = readBlocking())) {
    //Serial.print(c);
  }
  Serial.print(c);
  x = c - '0';
  while (isdigit(c = readBlocking())) {
    Serial.print(c);
    x *= 10;
    x += c - '0';
  }
  return x;
}

#endif
