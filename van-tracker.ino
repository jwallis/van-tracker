/*
   Depends on jwallis/Adafruit_FONA
   Written by Joshua Wallis, borrowing initially from AdaFruit's examples
   Support AdaFruit, buy their neato products!
*/

/*
   Quick TCs
    -garbage
    -info
    -loc
    -owner
    -owner set
    -owner set 333 444-5555
    -kill
    -kill disable
    -kill enable
    -fence
    -fence disable
    -fence enable
    -fence home
    -fence hours
    -fence hours 23
    -fence hours 23 8
    -fence hours 0 0
    -fence radius
    -fence radius 234
*/

/*
Blink debug codes - the Arduino Nono will blink these codes during operation:

Basic info codes (0 longs followed by THIS MANY shorts):
  1 = about to check inbound SMSs
  2 = about to execute watchdog processes
  5 = connected to FONA successfully at startup

Event codes (1 long follow by THIS MANY shorts).  Notice odd numbers are bad, even numbers ok:
  1  = failed  sending SMS
  2  = success sending SMS
  3  = failed  deleting SMS
  4  = success deleting SMS
  5  = failed  turning on  GPS
  6  = none
  7  = failed  getting fix on GPS
  8  = success getting fix on GPS
  9  = failed  turning off GPS
  10 = success turning off GPS

Error codes causing restart (2 longs followed by THIS MANY shorts):
  1 = in setupFONA(): "Couldn't find FONA, restarting."
  2 = in waitUntilSMSReady(): "SMS never became ready, restarting."
  3 = failed in getTime()
  4 = failed in checkSMSInput()
  5 = failed to turn on GPS
  6 = failed to get GPS satellite fix
  7 = failed in deleteSMS()
*/

//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//    SET HARDWARE OPTIONS
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

#define VAN_PROD
//#define VAN_TEST
//#define NEW_HARDWARE_ONLY  // Initializes new FONA/SIM808 module as well as new arduino's EEPROM

#define BOARD_UNO_NANO
//#define BOARD_MEGA

#define AND_TECH_SIM808_BREAKOUT
//#define ADAFRUIT_FONA_SHIELD

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
#define FONA_RX_PIN 3

#ifdef BOARD_UNO_NANO
#define FONA_TX_PIN 4
#endif
#ifdef BOARD_MEGA
#define FONA_TX_PIN 11
#endif

#define KILL_SWITCH_RELAY_PIN 5
#define KILL_SWITCH_LED_PIN 6
#define GEOFENCE_LED_PIN 7
#define RESET_PIN 8
#define DEBUG_PIN 13

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX_PIN, FONA_RX_PIN);
SoftwareSerial *fonaSerial = &fonaSS;
//HardwareSerial *fonaSerial = &Serial;

Adafruit_FONA fona = Adafruit_FONA(RESET_PIN);    //Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

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

short totalErrors = 0;
short lastError = 0;
short lastGPSQueryMinute = -1;
short lastGeofenceWarningMinute = -1;

volatile bool killSwitchOnVolatile = false;
volatile bool startAttemptedWhileKillSwitchOnVolatile = false;

void setup() {
#ifdef VAN_PROD
  pinSetup();
  setupFONA();
  waitUntilSMSReady();
#endif
  
#ifdef NEW_HARDWARE_ONLY
  setupSerial();
  setupFONA();
  initEEPROM();
  initFONA();
#endif

#ifdef VAN_TEST
  pinSetup();
  setupSerial();
  setupFONA();
  waitUntilSMSReady();
  printFONAType();
#endif
}

void loop() {
#ifdef VAN_TEST
  checkSerialInput();
  flushFONA();
#endif

  checkSMSInput();
  watchDog();
  delay(500);
}

void watchDog() {
  debugBlink(0,2);

  watchDogForKillSwitch();
  watchDogForGeofence();
  watchDogForTurnOffGPS();
  watchDogForErrors();
}

void watchDogForErrors() {
  if (totalErrors > 2) {
    totalErrors = 0;
    debugPrintln(F("_____________ TOTAL ERRORS > 2 _______________"));
    reportAndRestart(lastError, F("watchDogForErrors()"));
  }
}

void watchDogForTurnOffGPS() {
  // shut down GPS module after 10 minutes of inactivity

  if (lastGPSQueryMinute == -1)
    return;

  short currentMinuteInt = getCurrentMinuteInt();

  // if it's been > 10 min, take action (turn off gps to save power)

  // case 1 example: lastQuery = 5:10pm, current = 5:20pm
  if (lastGPSQueryMinute <= currentMinuteInt && currentMinuteInt - lastGPSQueryMinute > 10) {
      setFONAGPS(false);
      lastGPSQueryMinute = -1;
  }

  // case 2 example: lastQuery = 5:55, current = 6:05pm
  if (lastGPSQueryMinute > currentMinuteInt && lastGPSQueryMinute - currentMinuteInt < 50) {
      setFONAGPS(false);
      lastGPSQueryMinute = -1;
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

void watchDogForKillSwitch() {
  setKillSwitchPins(isActive(KILLSWITCHENABLED_BOOL_1, KILLSWITCHSTART_CHAR_3, KILLSWITCHEND_CHAR_3));
  if (startAttemptedWhileKillSwitchOnVolatile) {
    
    char ownerPhoneNumber[15];
    EEPROM.get(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);

    if (sendSMS(ownerPhoneNumber, F("WARNING!\nStart attempted while kill switch enabled"))) {
      startAttemptedWhileKillSwitchOnVolatile = false;
    }
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
    strcpy(message, "FOLLOW MODE");
  else
    strcpy(message, "GEOFENCE WARNING!");

  strcat(message, "\nCurrent:\ngoogle.com/search?q=");
  strcat(message, currentLat);
  strcat(message, ",");
  strcat(message, currentLon);
  strcat(message, "\nHome:\ngoogle.com/search?q=");
  strcat(message, geofenceHomeLat);
  strcat(message, ",");
  strcat(message, geofenceHomeLon);

  sendSMS(ownerPhoneNumber, message);
  // we only want to send this message the first time the geofence is broken
  if (lastGeofenceWarningMinute == -1 && !follow) {
    sendSMS(ownerPhoneNumber, F("Use 'follow enable' to receive rapid location updates"));
  }

  lastGeofenceWarningMinute = getCurrentMinuteInt();
}

void checkSMSInput() {
  int8_t numberOfSMSs;

  debugBlink(0,1);

  for (int i = 0; i < 30; i++) {
    numberOfSMSs = fona.getNumSMS();
    debugPrint(F("Number SMSs: ")); debugPrintln(numberOfSMSs);

    if (numberOfSMSs == 0)
      return;

    if (numberOfSMSs > 0) {
      handleSMSInput();
      return;
    }
    debugPrintln(F("error in checkSMSInput()"));
    delay(1000);
  }
  debugPrintln(F("failure in checkSMSInput()"));
  totalErrors++;
  lastError = 4;
}

void handleSMSInput() {
  short numberOfSMSs = fona.getNumSMS();

  char smsSender[16];
  char smsValue[51];
  int8_t smssFound = 0;

  // the SMS "slots" start at 1, not 0
  for (int8_t smsSlotNumber = 1; smssFound < numberOfSMSs; smsSlotNumber++) {

    // This is for handling possible runaway (infinite loop) if sim808 reports there is > 0 SMSs, but when checking individual slots, they all report they're empty
    if (smsSlotNumber == 10)
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

    if (strstr(smsValue, "unlock")) {
      if (checkLockdownStatus(smsSender, smsValue, smsSlotNumber))
        continue;

      if (handleUnlockReq(smsSender))
        deleteSMS(smsSlotNumber);
      continue;
    }

    if (strstr(smsValue, "lock")) {
      if (checkLockdownStatus(smsSender, smsValue, smsSlotNumber))
        continue;

      if (handleLockReq(smsSender))
        deleteSMS(smsSlotNumber);
      continue;
    }

    if (strstr(smsValue, "loc")) {
      if (handleLocReq(smsSender))
        deleteSMS(smsSlotNumber);
      continue;
    }

    if (strstr(smsValue, "kill")) {
      if (checkLockdownStatus(smsSender, smsValue, smsSlotNumber))
        continue;

      if (handleKillSwitchReq(smsSender, smsValue))
        deleteSMS(smsSlotNumber);
      continue;
    }

    if (strstr(smsValue, "fence")) {
      if (checkLockdownStatus(smsSender, smsValue, smsSlotNumber))
        continue;

      if (handleGeofenceReq(smsSender, smsValue))
        deleteSMS(smsSlotNumber);
      continue;
    }

    if (strstr(smsValue, "follow")) {
      if (handleFollowReq(smsSender, smsValue))
        deleteSMS(smsSlotNumber);
      continue;
    }

    if (strstr(smsValue, "owner")) {
      if (handleOwnerPhoneNumberReq(smsSender, smsValue))
        deleteSMS(smsSlotNumber);
      continue;
    }

    if (strstr(smsValue, "info")) {
      if (handleInfoReq(smsSender))
        deleteSMS(smsSlotNumber);
      continue;
    }

    // default
    if (handleUnknownReq(smsSender))
      deleteSMS(smsSlotNumber);
  }
}

bool checkLockdownStatus(char* smsSender, char* smsValue, int8_t smsSlotNumber) {
  bool lockdownEnabled;
  EEPROM.get(LOCKDOWNENABLED_BOOL_1, lockdownEnabled);

  if (lockdownEnabled) {
    if (strstr(smsValue, "kill") || strstr(smsValue, "fence")) {
      if (sendSMS(smsSender, F("Warning: Lockdown Enabled\nTry \"unlock\" before trying to modify fence or kill")))
        deleteSMS(smsSlotNumber);
      return true;
    }
    if (strstr(smsValue, "lock") && !strstr(smsValue, "unlock")) {
      if (sendSMS(smsSender, F("Lockdown already enabled")))
        deleteSMS(smsSlotNumber);
      return true;
    }
  }
  else {
    if (strstr(smsValue, "unlock")) {
      if (sendSMS(smsSender, F("Lockdown already disabled")))
        deleteSMS(smsSlotNumber);
      return true;
    }
  }
  return false;
}
      
bool handleLockReq(char* smsSender) {
  bool geofenceEnabled;
  char geofenceHomeLat[12];
  char geofenceHomeLon[12];
  char geofenceRadius[7];
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
  strcpy(geofenceRadius, "500");
  EEPROM.put(GEOFENCERADIUS_CHAR_7, geofenceRadius);
  EEPROM.put(GEOFENCESTART_CHAR_3, "00");
  EEPROM.put(GEOFENCEEND_CHAR_3, "00");
  EEPROM.put(KILLSWITCHENABLED_BOOL_1, true);
  EEPROM.put(KILLSWITCHSTART_CHAR_3, "00");
  EEPROM.put(KILLSWITCHEND_CHAR_3, "00");

  // set lockdown variable ON
  EEPROM.put(LOCKDOWNENABLED_BOOL_1, true);

  char message[120];

  // send SMS with new geofence home
  strcpy(message, "Lockdown: Enabled (all settings saved)\nRadius: ");
  strcat(message, geofenceRadius);
  strcat(message, " feet\nHome: google.com/search?q=");
  strcat(message, geofenceHomeLat);
  strcat(message, ",");
  strcat(message, geofenceHomeLon);

  return sendSMS(smsSender, message);
}

bool handleUnlockReq(char* smsSender) {
  bool geofenceEnabled;
  char geofenceHomeLat[12];
  char geofenceHomeLon[12];
  char geofenceRadius[7];
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

  char message[100];

  // send SMS with original geofenceHome
  strcpy(message, "Lockdown: Disabled (all settings restored)\nHome: google.com/search?q=");
  strcat(message, geofenceHomeLat);
  strcat(message, ",");
  strcat(message, geofenceHomeLon);

  return sendSMS(smsSender, message);
}

bool handleInfoReq(char* smsSender) {
  uint8_t rssi;
  char ccid[22];
  char imei[16];
  char currentTimeStr[23];
  char message[130];

  char ownerPhoneNumber[15] = "";
  EEPROM.get(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);

  bool lockdown;
  char lockdownStr[9];
  EEPROM.get(LOCKDOWNENABLED_BOOL_1, lockdown);
  
  if (lockdown)
    strcpy(lockdownStr, "Enabled");
  else
    strcpy(lockdownStr, "Disabled");

  bool follow;
  char followStr[9];
  EEPROM.get(GEOFENCEFOLLOW_BOOL_1, follow);
  
  if (follow)
    strcpy(followStr, "Enabled");
  else
    strcpy(followStr, "Disabled");

  rssi = fona.getRSSI();
  fona.getSIMCCID(ccid);
  fona.getIMEI(imei);

  getTime(currentTimeStr);


  sprintf(message, "Owner: %s\nLockdown: %s\nFollow: %s\nRSSI: %u\nCCID: %s\nIMEI: %s\nNetwork Time: %s", ownerPhoneNumber, lockdownStr, followStr, rssi, ccid, imei, currentTimeStr);
  return sendSMS(smsSender, message);
}

bool handleLocReq(char* smsSender) {
  char message[54];
  char latitude[12];
  char longitude[12];

  getGPSLatLon(latitude, longitude);

  sprintf(message, "google.com/search?q=%s,%s", latitude, longitude);
  return sendSMS(smsSender, message);
}

bool handleFollowReq(char* smsSender, char* smsValue) {
  if (strstr(smsValue, "enable") ) {
    EEPROM.put(GEOFENCEFOLLOW_BOOL_1, true);
    return sendSMS(smsSender, F("Follow: ENABLED"));
  }
  if (strstr(smsValue, "disable") ) {
    EEPROM.put(GEOFENCEFOLLOW_BOOL_1, false);
    return sendSMS(smsSender, F("Follow: DISABLED"));
  }
  return sendSMS(smsSender, F("Try \"follow\" plus:\nenable/disable"));  
}

bool handleKillSwitchReq(char* smsSender, char* smsValue) {
  char message[64] = "";

  bool killSwitchEnabled;
  char killSwitchStart[3];
  char killSwitchEnd[3];
  EEPROM.get(KILLSWITCHSTART_CHAR_3, killSwitchStart);
  EEPROM.get(KILLSWITCHEND_CHAR_3, killSwitchEnd);

  if (strstr(smsValue, "enable") ) {
    EEPROM.put(KILLSWITCHENABLED_BOOL_1, true);
    message[0] = '1';
  }
  if (strstr(smsValue, "disable") ) {
    EEPROM.put(KILLSWITCHENABLED_BOOL_1, false);
    message[0] = '1';
  }

  EEPROM.get(KILLSWITCHENABLED_BOOL_1, killSwitchEnabled);

  if (strstr(smsValue, "hours")) {
    if (setHoursFromSMS(smsValue, killSwitchStart, killSwitchEnd)) {
      // make 4 => 04
      insertZero(killSwitchStart);
      insertZero(killSwitchEnd);
      
      EEPROM.put(KILLSWITCHSTART_CHAR_3, killSwitchStart);
      EEPROM.put(KILLSWITCHEND_CHAR_3, killSwitchEnd);
      message[0] = '1';
    }
  }

  if (message[0] || strstr(smsValue, "info")) {
    //  Yay only 2k of RAM
    strcpy(message, "Kill: ");
    if (killSwitchEnabled)
      strcat(message, "En");
    else
      strcat(message, "Dis");

    strcat(message, "abled\nHours: ");
    strcat(message, killSwitchStart);
    strcat(message, "-");
    strcat(message, killSwitchEnd);
    if (strcmp(killSwitchStart, killSwitchEnd) == 0) {  // if start time == end time  
      strcat(message, " (always on)");
    }

    return sendSMS(smsSender, message);
  }
  else {
    return sendSMS(smsSender, F("Try \"kill\" plus:\nenable/disable\ninfo\nhours 0 21 (12am-9pm)"));
  }
}

bool handleGeofenceReq(char* smsSender, char* smsValue) {
  char message[132] = "";

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


  if (strstr(smsValue, "enable") ) {
    EEPROM.put(GEOFENCEENABLED_BOOL_1, true);
    message[0] = '1';
  }
  if (strstr(smsValue, "disable") ) {
    EEPROM.put(GEOFENCEENABLED_BOOL_1, false);
    message[0] = '1';
  }
  
  EEPROM.get(GEOFENCEENABLED_BOOL_1, geofenceEnabled);
  
  if (strstr(smsValue, "radius")) {
    geofenceRadius[0] = '\0';
    if (getNumberFromString(smsValue, geofenceRadius, 7)) {
      EEPROM.put(GEOFENCERADIUS_CHAR_7, geofenceRadius);
      message[0] = '1';
    }
  }
  if (strstr(smsValue, "home")) {
    getGPSLatLon(geofenceHomeLat, geofenceHomeLon);
    EEPROM.put(GEOFENCEHOMELAT_CHAR_12, geofenceHomeLat);
    EEPROM.put(GEOFENCEHOMELON_CHAR_12, geofenceHomeLon);
    message[0] = '1';
  }
  if (strstr(smsValue, "hours")) {
    if (setHoursFromSMS(smsValue, geofenceStart, geofenceEnd)) {
      // make 4 => 04
      insertZero(geofenceStart);
      insertZero(geofenceEnd);
      
      EEPROM.put(GEOFENCESTART_CHAR_3, geofenceStart);
      EEPROM.put(GEOFENCEEND_CHAR_3, geofenceEnd);
      message[0] = '1';
    }
  }

  // reset this so the "follow" message will be sent when the fence is broken
  lastGeofenceWarningMinute = -1;

  if (message[0] || strstr(smsValue, "info")) {
    //    Yay only 2k of RAM, doing this all piecemeal (instead of big strings as is done in this comment) saves 54 bytes!!!!!!!
    //    if (geofenceEnabled)
    //      sprintf(message, "ENABLED\nHours: %s-%s\nRadius: %s feet\nHome: google.com/search?q=%s,%s", geofenceStart, geofenceEnd, geofenceRadius, geofenceHomeLat, geofenceHomeLon);
    //    else
    //      sprintf(message, "DISABLED\nHours: %s-%s\nRadius: %s feet\nHome: google.com/search?q=%s,%s", geofenceStart, geofenceEnd, geofenceRadius, geofenceHomeLat, geofenceHomeLon);

    strcpy(message, "Fence: ");
    if (geofenceEnabled)
      strcat(message, "En");
    else
      strcat(message, "Dis");

    strcat(message, "abled\nHours: ");
    strcat(message, geofenceStart);
    strcat(message, "-");
    strcat(message, geofenceEnd);
    if (strcmp(geofenceStart, geofenceEnd) == 0) {  // if start time == end time  
      strcat(message, " (always on)");
    }
    strcat(message, "\nRadius: ");
    strcat(message, geofenceRadius);
    strcat(message, " feet\nHome: google.com/search?q=");
    strcat(message, geofenceHomeLat);
    strcat(message, ",");
    strcat(message, geofenceHomeLon);

    return sendSMS(smsSender, message);
  }
  else {
    return sendSMS(smsSender, F("Try \"fence\" plus:\nenable/disable\ninfo\nhours 0 21 (12am-9pm)\nhome (uses current loc)\nradius 100 (100 feet)"));
  }
}

bool handleOwnerPhoneNumberReq(char* smsSender, char* smsValue) {
  char message[70] = "";
  char ownerPhoneNumber[15] = "";

  // set owner number
  if (strstr(smsValue, "set")) {
    getNumberFromString(smsValue, ownerPhoneNumber, 15);

    // if "set" was found but no number was found, use the number of the person who sent the text
    if (!ownerPhoneNumber[0]) {
      getNumberFromString(smsSender, ownerPhoneNumber, 15);
    }

    sprintf(message, "Setting owner phone # to %s", ownerPhoneNumber);
    EEPROM.put(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);
  }

  // just respond with current owner number
  else {
    EEPROM.get(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);
    sprintf(message, "%s\nTry \"owner\" plus:\nset (include number or blank for YOUR number)", ownerPhoneNumber);
  }

  return sendSMS(smsSender, message);
}

bool handleUnknownReq(char* smsSender) {
  return sendSMS(smsSender, F("Commands:\nfence\nfollow\ninfo\nkill\nloc\nowner\nlock\nunlock"));
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
//BUSINESS FUNCTIONS
///////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////
//BATTERY

void getBatteryStats(char* batteryStats) {
  uint16_t batteryVolts;
  uint16_t batteryPercent;

  for (int i = 0; i < 3; i++) {
    if (fona.getBattVoltage(&batteryVolts))
      break;
    delay(1000);
  }

  for (int i = 0; i < 3; i++) {
    if (fona.getBattPercent(&batteryPercent))
      break;
    delay(1000);
  }

  sprintf(batteryStats, "%u%% %umV", batteryPercent, batteryVolts);
}

////////////////////////////////
//GPS

void setFONAGPS(bool tf) {
  // turns FONA GPS on or off (don't waste power)
  if (!tf) {
    fona.enableGPS(false);

    for (int k = 0; k < 3; k++) {
      if (fona.GPSstatus() == 0) {
        debugBlink(1,10);
        return;
      }
      delay(1000);
    }
    debugBlink(1,9);
    debugPrintln(F("Failed to turn off GPS"));
    return;
  }

  // -1 = error querying GPS
  //  0 = GPS off
  //  1 = no GPS fix
  //  2 = 2D fix
  //  3 = 3D fix
  if (fona.GPSstatus() >= 2)
    return;

  int8_t status;

  // Keep trying to get a valid (non-error) response. Maybe we should turn off/on?
  for (int i = 1; i < 30; i++) {
    if (fona.GPSstatus() >= 0) {
      break;
    }
    delay(2000);
  }

  // error, give up
  if (fona.GPSstatus() < 0) {
    totalErrors++;
    lastError = 5;
    debugPrintln(F("Failed to turn on GPS"));
    debugBlink(1,5);
    return;
  }

  // off, turn on
  if (fona.GPSstatus() == 0) {
    fona.enableGPS(true);
    delay(4000);
  }

  
  char currentLat[12];
  char currentLon[12];

  // wait up to 90s to get GPS fix
  for (int j = 1; j < 45; j++) {
    if (fona.GPSstatus() >= 2) {
      debugBlink(1,8);

      // I really hate to do this, but the first GPS response is sometimes WAY off (> 200 feet) and you get a geofence warning...
      getGPSLatLon(currentLat, currentLon);
      delay(3000);
      getGPSLatLon(currentLat, currentLon);
      return;
    }
    delay(2000);
  }

  // no fix, give up
  if (fona.GPSstatus() < 2) {
    totalErrors++;
    lastError = 6;
    debugPrintln(F("Failed to get GPS fix"));
    debugBlink(1,7);
    return;
  }
}

void getGPSLatLon(char* latitude, char* longitude) {
  char gpsString[120];

  setFONAGPS(true);
  fona.getGPS(0, gpsString, 120);
  lastGPSQueryMinute = getCurrentMinuteInt();

  // full string:
  // 1,1,20190913060459.000,30.213823,-97.782017,204.500,1.87,90.1,1,,1.2,1.5,0.9,,11,6,,,39,,
  getOccurrenceInDelimitedString(gpsString, latitude, 4, ',');
  getOccurrenceInDelimitedString(gpsString, longitude, 5, ',');

#ifdef VAN_TEST
  char message[70];
  strcpy(message, "google.com/search?q=");
  strcat(message, latitude);
  strcat(message, ",");
  strcat(message, longitude);
  debugPrintln(message);
#endif
}

void getTime(char* currentTimeStr) {
  // sets currentTime to "19/09/19,17:03:55-20" INCLUDING quotes
  
  for (short i = 0; i < 5; i++) {
    fona.getTime(currentTimeStr, 23);
    if (currentTimeStr[0] == '"')
      break;
    delay(1000);
  }

  // QUOTES ARE PART OF THE STRING: "19/09/19,17:03:01-20"
  if (!currentTimeStr[0] == '"') {
    totalErrors++;
    lastError = 3;
  }
}

////////////////////////////////
//SMS

void deleteSMS(uint8_t msg_number) {
  for (int i = 2; i < 40; i++) {
    debugPrintln(F("  Attempting to delete SMS"));
    if (fona.deleteSMS(msg_number)) {
      debugPrintln(F("  Success deleting SMS"));
      debugBlink(1,4);
      return;
    }
    delay(1000);
  }
  debugPrintln(F("  Failed to delete SMS"));
  debugBlink(1,3);
  totalErrors++;
  lastError = 7;
}

bool sendSMS(char* send_to, char* message) {
  debugPrintln(F("  Attempting to send SMS:"));
  debugPrintln(message);

  if (fona.sendSMS(send_to, message)) {
    debugPrintln(F("  Success sending SMS"));
    debugBlink(1,2);
    return true;
  } else {
    debugPrintln(F("  Failed to send SMS"));
    debugBlink(1,1);
    return false;
  }
}

bool sendSMS(char* send_to, const __FlashStringHelper* message) {
  debugPrintln(F("  Attempting to send SMS:"));
  debugPrintln(message);

  if (fona.sendSMS(send_to, message)) {
    debugPrintln(F("  Success sending SMS"));
    debugBlink(1,2);
    return true;
  } else {
    debugPrintln(F("  Failed to send SMS"));
    debugBlink(1,1);
    return false;
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
    delay(150);
    digitalWrite(13, LOW);
    delay(300);
  }
  delay(300);
}

bool setHoursFromSMS(char* smsValue, char* hoursStart, char* hoursEnd) {
  // smsValue:
  // fence hours 0 21
  // 0 21 means 12am - 9pm
  getOccurrenceInDelimitedString(smsValue, hoursStart, 3, ' ');
  getOccurrenceInDelimitedString(smsValue, hoursEnd, 4, ' ');

  return (hoursStart[0] && hoursEnd[0]);
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

void reportAndRestart(short shortBlinks, const __FlashStringHelper* message) {
  debugBlink(2,shortBlinks);

  //TBD add method gotError(errMsg) which replaces "totalErrors++" everywhere, which writes error to EEPROM then on next startup, send SMS with err msg.  don't send sms here.
  char ownerPhoneNumber[15];
  EEPROM.get(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);

  debugPrint(F("in reportAndRestart: "));debugPrintln(message);
  sendSMS(ownerPhoneNumber, message);
  restartSystem();
}

void restartSystem() {
  // reset sim808.  From the sim808 HW manual:
  //    Normal power off by sending the AT command “AT+CPOWD=1” or using the PWRKEY.
  //    The power management unit shuts down the power supply for the baseband part of the
  //    module, and only the power supply for the RTC is remained. Software is not active. The
  //    serial port is not accessible. Power supply (connected to VBAT) remains applied.
  sendRawCommand(F("AT+CPOWD=1"));
  delay(45000);

  // reset arduino
  wdt_disable();
  wdt_enable(WDTO_15MS);
  debugPrintln(F("about to reset"));
  while (1) {}
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
    flushFONA();
  }
  delay(1000);
}

void toLower(char* str) {
  for (int i = 0; str[i]; i++)
    str[i] = tolower(str[i]);
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

bool getOccurrenceInDelimitedString(char* in, char* out, short occur, char delim) {
  // if in == "a,b,c"
  // and occur = 2
  // and delim = ','
  // sets out = "b"
  // be sure to leave room for '\0'
  short delimCount = 0;
  short outCount = 0;
  bool foundOccurrence = false;

  for (int i = 0; in[i]; i++) {
    if (in[i] == delim) {
      if (delimCount + 1 == occur) {
        break;
      }
      else {
        delimCount++;
        continue;
      }
    }

    if (delimCount + 1 == occur) {
      out[outCount] = in[i];
      outCount++;
      foundOccurrence = true;
    }
  }
  out[outCount] = '\0';
  return foundOccurrence;
}

bool outsideGeofence(char* lat1Str, char* lon1Str) {
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

void flushSerial() {
  while (Serial.available())
    Serial.read();
}

void flushFONA() {
  while (fona.available())
    Serial.write(fona.read());
}


///////////////////////////////////////////////////////////////////////////////////////////
//SETUP & PINS
///////////////////////////////////////////////////////////////////////////////////////////

void pinSetup() {
  pinMode(STARTER_INTERRUPT_PIN, INPUT);
  attachInterrupt(STARTER_INTERRUPT_ID, starterISR, RISING);

  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);
  
  pinMode(GEOFENCE_LED_PIN, OUTPUT);
  pinMode(KILL_SWITCH_RELAY_PIN, OUTPUT);
  pinMode(KILL_SWITCH_LED_PIN, OUTPUT);
  pinMode(DEBUG_PIN, OUTPUT);  // for debugging only
}

void starterISR() {
  _delay_ms(500);  // on some starters, turning to the key to the "accessory" mode might jump to 12V for just a few milliseconds, so let's wait - make sure someone is actually trying to start the car
  if (killSwitchOnVolatile && digitalRead(STARTER_INTERRUPT_PIN))
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

#if defined VAN_TEST || defined NEW_HARDWARE_ONLY
void setupSerial() {
  while (!Serial);
  Serial.begin(115200);
  Serial.println(F("Initializing....(May take 3 seconds)"));
}
#endif

void setupFONA() {
#ifdef ADAFRUIT_FONA_SHIELD
  fonaSerial->begin(4800);
#endif
#ifdef AND_TECH_SIM808_BREAKOUT
  fonaSerial->begin(9600);
#endif

  // TBD make this loop for up to 90s
  if (! fona.begin(*fonaSerial)) {
    reportAndRestart(1, F("Couldn't find FONA, restarting."));
  }
  debugBlink(0,5);
  debugPrintln(F("FONA is OK"));
}

void waitUntilSMSReady() {
  debugPrint(F("Waiting until SMS is ready"));
  
  for (int i = 0; i < 60; i++) {
    debugPrint(F("."));
    if (fona.getNumSMS() >= 0) {
      debugPrintln(F("\nSMS is ready"));
      return;
    }
    delay(2000);
  }
  reportAndRestart(2, F("SMS never became ready, restarting."));
}

#ifdef NEW_HARDWARE_ONLY
void initEEPROM() {
  // used on brand-new FONA module
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
  EEPROM.put(OWNERPHONENUMBER_CHAR_15, "5551234567");
  EEPROM.put(LOCKDOWNENABLED_BOOL_1, false);

  debugPrintln(F("End initEEPROM()"));
}

void initFONA() {
  // used on brand-new FONA module
  debugPrintln(F("Begin initFONA()"));
  sendRawCommand(F("ATZ"));                 // Reset settings
  sendRawCommand(F("AT+CMEE=2"));           // Turn on verbose mode
  sendRawCommand(F("AT+CLTS=1"));           // turn on "get clock when registering w/network" see https://forums.adafruit.com/viewtopic.php?f=19&t=58002

// let's keep this LED on for now...
//  sendRawCommand(F("AT+CNETLIGHT=0"));      // stop "net" LED

  sendRawCommand(F("AT+CMEE=0"));           // Turn off verbose mode
  sendRawCommand(F("AT&W"));                // save writeable settings
  debugPrintln(F("End initFONA().\n\nPlease update #ifdefs and restart."));
  while (1) {}
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
  char tempc[13];
  bool tempb;

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

  if (strcmp(temp, "g") == 0) {
        getEEPROM();
  }
  if (strcmp(temp, "p") == 0) {
        putEEPROM();
  }
  if (strcmp(temp, "w") == 0) {
        watchDog();
  }

  if (strcmp(temp, "b") == 0) {
        uint16_t vbat;
        fona.getBattVoltage(&vbat);
        debugPrintln(vbat);
        fona.getBattPercent(&vbat);
        debugPrintln(vbat);
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

  if (strcmp(temp, "S") == 0) {
    debugPrintln(F("Creating SERIAL TUBE"));
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
    //    case 'n': {
    //        // read the network/cellular status
    //        uint8_t n = fona.getNetworkStatus();
    //        Serial.print(F("Network status "));
    //        Serial.print(n);
    //        Serial.print(F(": "));
    //        if (n == 0) debugPrintln(F("Not registered"));
    //        if (n == 1) debugPrintln(F("Registered (home)"));
    //        if (n == 2) debugPrintln(F("Not registered (searching)"));
    //        if (n == 3) debugPrintln(F("Denied"));
    //        if (n == 4) debugPrintln(F("Unknown"));
    //        if (n == 5) debugPrintln(F("Registered roaming"));
    //        break;
    //      }
    //
    //    /*** SMS ***/
    //
    //    case 'N': {
    //        // read the number of SMS's!
    //        int8_t smsnum = fona.getNumSMS();
    //        if (smsnum < 0) {
    //          debugPrintln(F("Could not read # SMS"));
    //        } else {
    //          Serial.print(smsnum);
    //          debugPrintln(F(" SMS's on SIM card!"));
    //        }
    //        break;
    //      }
    //    case 'r': {
    //        // read an SMS
    //        flushSerial();
    //        Serial.print(F("Read #"));
    //        uint8_t smsn = readnumber();
    //        Serial.print(F("\n\rReading SMS #")); debugPrintln(smsn);
    //
    //
    //        // Retrieve SMS sender address/phone number.
    //        if (! fona.getSMSSender(smsn, replybuffer, 250)) {
    //          debugPrintln("Failed!");
    //          break;
    //        }
    //        Serial.print(F("FROM: ")); debugPrintln(replybuffer);
    //
    //        // Retrieve SMS value.
    //        uint16_t smslen;
    //        if (! fona.readSMS(smsn, replybuffer, 250, &smslen)) { // pass in buffer and max len!
    //          debugPrintln("Failed!");
    //          break;
    //        }
    //        Serial.print(F("***** SMS #")); Serial.print(smsn);
    //        Serial.print(" ("); Serial.print(smslen); debugPrintln(F(") bytes *****"));
    //        debugPrintln(replybuffer);
    //        debugPrintln(F("*****"));
    //
    //        break;
    //      }
    //    case 'R': {
    //        // read all SMS
    //        int8_t smsnum = fona.getNumSMS();
    //        uint16_t smslen;
    //        int8_t smsn;
    //
    //        if ( (type == FONA3G_A) || (type == FONA3G_E) ) {
    //          smsn = 0; // zero indexed
    //          smsnum--;
    //        } else {
    //          smsn = 1;  // 1 indexed
    //        }
    //
    //        for ( ; smsn <= smsnum; smsn++) {
    //          Serial.print(F("\n\rReading SMS #")); debugPrintln(smsn);
    //          if (!fona.readSMS(smsn, replybuffer, 250, &smslen)) {  // pass in buffer and max len!
    //            debugPrintln(F("Failed!"));
    //            break;
    //          }
    //          // if the length is zero, its a special case where the index number is higher
    //          // so increase the max we'll look at!
    //          if (smslen == 0) {
    //            debugPrintln(F("[empty slot]"));
    //            smsnum++;
    //            continue;
    //          }
    //
    //          Serial.print(F("***** SMS #")); Serial.print(smsn);
    //          Serial.print(" ("); Serial.print(smslen); debugPrintln(F(") bytes *****"));
    //          debugPrintln(replybuffer);
    //          debugPrintln(F("*****"));
    //        }
    //        break;
    //      }
    //
  if (strcmp(temp, "d") == 0) {
    for (int i = 2; i < 10; i++) {
      debugPrintln(F("  Attempting to delete SMS"));
      if (fona.deleteSMS(readnumber())) {
        debugPrintln(F("  Success deleting SMS"));
        return;
      }
      delay(i * 1000);
    }
    debugPrintln(F("  Failed to delete SMS"));
  }

  if (command.length() > 2){
    char smsSender[16];
    char smsValue[51];
    getOccurrenceInDelimitedString(temp, smsSender, 1, '_');
    getOccurrenceInDelimitedString(temp, smsValue, 2, '_');
    testHandleSMSInput(smsSender, smsValue);
  }

  flushSerial();
  flushFONA();
}

void testHandleSMSInput(char* smsSender, char* smsValue) {

  toLower(smsValue);

  debugPrintln(F("--read SMS--"));
  debugPrintln(smsSender);
  debugPrintln(smsValue);
  debugPrintln(F(""));

  if (strstr(smsValue, "gps") || strstr(smsValue, "loc")) {
    handleLocReq(smsSender);
    return;
  }

  if (strstr(smsValue, "kill")) {
    handleKillSwitchReq(smsSender, smsValue);
    return;
  }

  if (strstr(smsValue, "fence")) {
    handleGeofenceReq(smsSender, smsValue);
    return;
  }

  if (strstr(smsValue, "owner")) {
    handleOwnerPhoneNumberReq(smsSender, smsValue);
    return;
  }

  if (strstr(smsValue, "info")) {
    handleInfoReq(smsSender);
    return;
  }

  handleUnknownReq(smsSender);
}

void printFONAType() {
  switch (fona.type()) {
    case FONA808_V1:
      debugPrintln(F("FONA 808 (v1)")); break;
    case FONA808_V2:
      debugPrintln(F("FONA 808 (v2)")); break;
    case FONA3G_A:
      debugPrintln(F("FONA 3G (American)")); break;
    case FONA3G_E:
      debugPrintln(F("FONA 3G (European)")); break;
    default:
      debugPrintln(F("FOMA ??? Unknown type")); break;
  }
  sendRawCommand(F("ATI"));
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
