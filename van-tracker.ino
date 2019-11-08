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

#define FONA_RX_PIN 2
#define RESET_PIN 4

#ifdef BOARD_UNO_NANO
#define FONA_TX_PIN 3
#endif
#ifdef BOARD_MEGA
#define FONA_TX_PIN 11
#endif

#define KILL_SWITCH_RELAY_PIN 5
#define KILL_SWITCH_LED_PIN 6
#define GEOFENCE_LED_PIN 7
#define DEBUG_PIN 13

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX_PIN, FONA_RX_PIN);
SoftwareSerial *fonaSerial = &fonaSS;
//HardwareSerial *fonaSerial = &Serial;

Adafruit_FONA fona = Adafruit_FONA(RESET_PIN);    //Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);


#define APN_ID "wholesale"
#define GEOFENCEENABLED_BOOL_1      0
#define GEOFENCEHOMELAT_CHAR_12     1
#define GEOFENCEHOMELON_CHAR_12     13
#define GEOFENCERADIUS_CHAR_7       25
#define GEOFENCESTART_CHAR_3        32
#define GEOFENCEEND_CHAR_3          35
#define GEOFENCEFOLLOW_BOOL_1       38
#define KILLSWITCHENABLED_BOOL_1    39
#define KILLSWITCHSTART_CHAR_3      40
#define KILLSWITCHEND_CHAR_3        43
#define OWNERPHONENUMBER_CHAR_15    46

uint8_t totalErrors = 0;
short lastGPSQueryMinute = -1;
short lastGeofenceWarningMinute = -1;

void setup() {
#ifdef VAN_PROD
  setupFONA();
  pinSetup();
  waitUntilSMSReady();
  moreSetup();
  //TBD - first GPS response is sometimes WAY off (> 200 feet) and you get a geofence warning...
#endif
  
#ifdef NEW_HARDWARE_ONLY
  setupSerial();
  setupFONA();
  initEEPROM();
  initFONA();
#endif

#ifdef VAN_TEST
  setupSerial();
  setupFONA();
  pinSetup();
  waitUntilSMSReady();
  moreSetup();
#endif
}

void loop() {
#ifdef VAN_TEST
  checkSerialInput();
  flushFONA();
#endif

  debugBlink(2);
  checkSMSInput();

  debugBlink(3);
  watchDog();
  delay(500);
}

void watchDog() {
  watchDogForErrors();
  watchDogForTurnOffGPS();
  watchDogForKillSwitch();
  watchDogForGeofence();
}

void watchDogForErrors() {
  if (totalErrors > 2) {
    totalErrors = 0;
    debugPrintln(F("_____________ TOTAL ERRORS > 2 _______________"));
    fixErrors(F("watchDogForErrors()"));
  }
}

void watchDogForTurnOffGPS() {
  // shut down GPS module after 20m of inactivity

  if (lastGPSQueryMinute == -1)
    return;

  short currentMinuteInt = getCurrentMinuteShort();

  // if it's been > 20 minutes, take action (turn off gps to save power)

  // lastQuery = 10, current = 20
  if (lastGPSQueryMinute <= currentMinuteInt && currentMinuteInt - lastGPSQueryMinute > 20) {
      setFONAGPS(false);
      lastGPSQueryMinute = -1;
  }

  // lastQuery = 50, current = 10
  if (lastGPSQueryMinute > currentMinuteInt && lastGPSQueryMinute - currentMinuteInt < 40) {
      setFONAGPS(false);
      lastGPSQueryMinute = -1;
  }
}


int getCurrentMinuteShort() {
  char currentTimeStr[23];
  char currentMinuteStr[3];
  
  getTime(currentTimeStr);

  for (short i = 0; i < 3; i++) {
    debugPrintln(F("in getCurrentMinuteShort"));
    getTime(currentTimeStr);
    if (currentTimeStr[0] == '"')
      break;
    delay(1000 * (i+1));
  }

  // QUOTES ARE PART OF THE STRING: "19/09/19,17:03:01-20"
  if (!currentTimeStr[0] == '"')
    totalErrors++;
  debugPrintln(currentTimeStr);

  currentMinuteStr[0] = currentTimeStr[13];
  currentMinuteStr[1] = currentTimeStr[14];
  currentMinuteStr[2] = '\0';
  return atoi(currentMinuteStr);
}

int getCurrentHourShort() {
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
}

void watchDogForGeofence() {
  bool follow;
  EEPROM.get(GEOFENCEFOLLOW_BOOL_1, follow);

  if (follow) {
    sendGeofenceWarning(true);
    delay(20000);
    return;
  }

  bool geofenceActive = isActive(GEOFENCEENABLED_BOOL_1, GEOFENCESTART_CHAR_3, GEOFENCEEND_CHAR_3);
  setGeofencePins(geofenceActive);

  if (!geofenceActive) {
    lastGeofenceWarningMinute = -1;
    return;
  }

  short currentMinuteInt = getCurrentMinuteShort();

  if (lastGeofenceWarningMinute != -1) {

    // If it's been < 5 minutes since last GPS query, do not take action i.e. we'll only check for breaking the fence every 5 minutes.
    // This is in case the fence IS broken, sending > every 5 min doesn't help.  User can use follow mode.

    // lastQuery = 10, current = 20
    if (lastGeofenceWarningMinute <= currentMinuteInt && currentMinuteInt - lastGeofenceWarningMinute < 5) {
      return;
    }
  
    // lastQuery = 57, current = 05
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
    strcpy(message, "GEOFENCE WARNING");

  strcat(message, ":\nHome:\ngoogle.com/search?q=");
  strcat(message, geofenceHomeLat);
  strcat(message, ",");
  strcat(message, geofenceHomeLon);
  strcat(message, "\nCurrent:\ngoogle.com/search?q=");
  strcat(message, currentLat);
  strcat(message, ",");
  strcat(message, currentLon);

  sendSMS(ownerPhoneNumber, message);
  // we only want to send this message the first time the geofence is broken
  if (lastGeofenceWarningMinute == -1 && !follow) {
    sendSMS(ownerPhoneNumber, F("Use 'follow enable' to receive rapid location updates"));
  }

  lastGeofenceWarningMinute = getCurrentMinuteShort();
}

void fixErrors(const __FlashStringHelper* message) {

  //TBD do something good here
  //TBD add method gotError(errMsg) which replaces "totalErrors++" everywhere, which writes error to EEPROM then on next startup, send SMS with err msg.  don't send sms here.
  char ownerPhoneNumber[15];
  EEPROM.get(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);

  debugPrint(F("in fixErrors: "));debugPrintln(message);
  sendSMS(ownerPhoneNumber, message);
  resetFONA();
}

void checkSMSInput() {
  int8_t numberOfSMSs;

  for (int i = 0; i < 8; i++) {
    numberOfSMSs = fona.getNumSMS();
    debugPrint(F("Number SMSs: ")); debugPrintln(numberOfSMSs);

    if (numberOfSMSs == 0)
      return;

    if (numberOfSMSs > 0) {
      handleSMSInput();
      return;
    }
    debugPrintln(F("error in checkSMSInput()"));
    delay(1000 * (i+1));
  }
  debugPrintln(F("failure in checkSMSInput()"));
  totalErrors++;
}

void handleSMSInput() {
  //totalErrors = 0;  //do we want to reset here? not sure why i did this
  int8_t numberOfSMSs = fona.getNumSMS();

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

    if (strstr(smsValue, "loc")) {
      handleLocReq(smsSender);
      deleteSMS(smsSlotNumber);
      continue;
    }

    if (strstr(smsValue, "kill")) {
      handleKillSwitchReq(smsSender, smsValue);
      deleteSMS(smsSlotNumber);
      continue;
    }

    if (strstr(smsValue, "fence")) {
      handleGeofenceReq(smsSender, smsValue);
      deleteSMS(smsSlotNumber);
      continue;
    }

    if (strstr(smsValue, "follow")) {
      handleFollowReq(smsSender, smsValue);
      deleteSMS(smsSlotNumber);
      continue;
    }

    if (strstr(smsValue, "owner")) {
      handleOwnerPhoneNumberReq(smsSender, smsValue);
      deleteSMS(smsSlotNumber);
      continue;
    }

    if (strstr(smsValue, "info")) {
      handleInfoReq(smsSender);
      deleteSMS(smsSlotNumber);
      continue;
    }

    // default
    handleUnknownReq(smsSender);
    deleteSMS(smsSlotNumber);
  }
}

void handleInfoReq(char* smsSender) {
  char battery[12];
  uint8_t rssi;
  char ccid[22];
  char imei[16];
  char message[141];

  char ownerPhoneNumber[15] = "";
  EEPROM.get(OWNERPHONENUMBER_CHAR_15, ownerPhoneNumber);

  getBatteryStats(battery);
  rssi = fona.getRSSI();
  fona.getSIMCCID(ccid);
  fona.getIMEI(imei);

  sprintf(message, "Battery: %s\nRSSI: %u\nOwner: %s\nCCID: %s\nIMEI: %s\nAPN: %s", battery, rssi, ownerPhoneNumber, ccid, imei, APN_ID);
  sendSMS(smsSender, message);
}

void handleLocReq(char* smsSender) {
  char message[54];
  char latitude[12];
  char longitude[12];

  getGPSLatLon(latitude, longitude);

  sprintf(message, "google.com/search?q=%s,%s", latitude, longitude);
  sendSMS(smsSender, message);
}

void handleFollowReq(char* smsSender, char* smsValue) {
  if (strstr(smsValue, "enable") ) {
    EEPROM.put(GEOFENCEFOLLOW_BOOL_1, true);
    sendSMS(smsSender, F("Follow: ENABLED"));
    return;
  }
  if (strstr(smsValue, "disable") ) {
    EEPROM.put(GEOFENCEFOLLOW_BOOL_1, false);
    sendSMS(smsSender, F("Follow: DISABLED"));
    return;
  }
  sendSMS(smsSender, F("Try \"follow\" plus:\nenable/disable"));  
}

void handleKillSwitchReq(char* smsSender, char* smsValue) {
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

    sendSMS(smsSender, message);
  }
  else {
    sendSMS(smsSender, F("Try \"kill\" plus:\nenable/disable\ninfo\nhours 0 21 (12am-9pm)"));
  }
}

void handleGeofenceReq(char* smsSender, char* smsValue) {
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
    //    Yay only 2k of RAM, this saves 54 bytes!!!!!!!
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

    sendSMS(smsSender, message);
  }
  else {
    sendSMS(smsSender, F("Try \"fence\" plus:\nenable/disable\ninfo\nhours 0 21 (12am-9pm)\nhome (uses current loc)\nradius 100 (100 feet)"));
  }
}

void handleOwnerPhoneNumberReq(char* smsSender, char* smsValue) {
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

  sendSMS(smsSender, message);
}

void handleUnknownReq(char* smsSender) {
  sendSMS(smsSender, F("Commands:\ninfo\nloc\nowner\nkill\nfence\nfollow"));
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
      if (fona.GPSstatus() == 0)
        return;
      delay(1000);
    }
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

  // error, keep trying status.  maybe we should turn off/on?
  for (int i = 1; i < 7; i++) {
    if (fona.GPSstatus() >= 0)
      break;
    delay(i * 2000);
  }

  // error, give up
  if (fona.GPSstatus() < 0) {
    totalErrors++;
    debugPrintln(F("Failed to turn on GPS"));
    return;
  }

  // off, turn on
  if (fona.GPSstatus() == 0) {
    fona.enableGPS(true);
    delay(4000);
  }

  // no fix, wait and ask again
  for (int j = 1; j < 9; j++) {
    if (fona.GPSstatus() >= 2)
      return;
    delay(j * 2000);
  }

  // no fix, give up
  if (fona.GPSstatus() < 2) {
    totalErrors++;
    debugPrintln(F("Failed to get GPS fix"));
    return;
  }
}

void getGPSLatLon(char* latitude, char* longitude) {
  char gpsString[120];

  setFONAGPS(true);
  fona.getGPS(0, gpsString, 120);
  lastGPSQueryMinute = getCurrentMinuteShort();

  // full string:
  // 1,1,20190913060459.000,30.213823,-97.782017,204.500,1.87,90.1,1,,1.2,1.5,0.9,,11,6,,,39,,
  getOccurrenceInDelimitedString(gpsString, latitude, 4, ',');
  getOccurrenceInDelimitedString(gpsString, longitude, 5, ',');
}

void getTime(char* currentTime) {
  // sets currentTime to "19/09/19,17:03:55-20" INCLUDING quotes
  fona.getTime(currentTime, 23);
}

////////////////////////////////
//GPRS

//TBD
void turnGPRSOff() {
  for (int i = 0; i < 3; i++) {
    if (fona.enableGPRS(false))
      return;
    delay(1000);
  }
  debugPrintln(F("  Failed to turn GPRS off"));
}

void turnGPRSOn() {
  for (int i = 0; i < 3; i++) {
    if (fona.enableGPRS(true))
      return;
    delay(1000);
  }
  debugPrintln(F("  Failed to turn GPRS on"));
}

////////////////////////////////
//SMS

void deleteSMS(uint8_t msg_number) {
  for (int i = 2; i < 10; i++) {
    debugPrintln(F("  Attempting to delete SMS"));
    if (fona.deleteSMS(msg_number)) {
      debugPrintln(F("  Success deleting SMS"));
      return;
    }
    delay(i * 1000);
  }
  debugPrintln(F("  Failed to delete SMS"));
  totalErrors++;
}

void sendSMS(char* send_to, char* message) {
  debugPrintln(F("  Attempting to send SMS:"));
  debugPrintln(message);
  if (!fona.sendSMS(send_to, message)) {
    debugPrintln(F("  Failed to send SMS"));
  } else {
    debugPrintln(F("  Success sending SMS"));
  }
}

void sendSMS(char* send_to, const __FlashStringHelper* message) {
  debugPrintln(F("  Attempting to send SMS:"));
  debugPrintln(message);
  if (!fona.sendSMS(send_to, message)) {
    debugPrintln(F("  Failed to send SMS"));
  } else {
    debugPrintln(F("  Success sending SMS"));
  }
}






///////////////////////////////////////////////////////////////////////////////////////////
//HELPERS
///////////////////////////////////////////////////////////////////////////////////////////
void debugBlink(int i) {
  for (int j=0; j < i; j++) {
    digitalWrite(13, HIGH);
    delay(150);
    digitalWrite(13, LOW);
    delay(300);
  }
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

  short currentHour = getCurrentHourShort();

  // simple case, current time is between start/end.  Start time is inclusive
  if (currentHour >= atoi(startHour) && currentHour < atoi(endHour))
    return true;

  // if start time is after end time (23-7 i.e. 11pm-7am), it only has to satisfy one of the conditions.  HOW INTERESTING
  if (strcmp(startHour, endHour) > 0)
    if (currentHour >= atoi(startHour) || currentHour < atoi(endHour))
      return true;

  return false;
}

void resetFONA() {
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
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);
  
  pinMode(GEOFENCE_LED_PIN, OUTPUT);
  pinMode(KILL_SWITCH_RELAY_PIN, OUTPUT);
  pinMode(KILL_SWITCH_LED_PIN, OUTPUT);
  pinMode(DEBUG_PIN, OUTPUT);  // for debugging only
}

void setKillSwitchPins(bool tf) {
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
    debugBlink(10);
    debugPrintln(F("Couldn't find FONA, restarting."));
    delay(30000);
    resetFONA();
  }
  debugBlink(5);
  debugPrintln(F("FONA is OK"));
}

void waitUntilSMSReady() {
  debugPrint(F("Waiting until SMS is ready"));
  
  for (int i = 0; i < 15; i++) {
    debugPrint(F("."));
    if (fona.getNumSMS() >= 0) {
      debugPrintln(F("SMS is ready"));
      return;
    }
    delay(1000 * (i+1));
  }
  fixErrors(F("waitUntilSMSReady()"));
}

void moreSetup() {
  setAPN();
#ifdef VAN_TEST
  printFONAType();
#endif
}

void setAPN() {
  //TBD
  fona.setGPRSNetworkSettings(F(APN_ID));
}


#ifdef NEW_HARDWARE_ONLY
void initEEPROM() {
  // used on brand-new FONA module
  debugPrintln(F("Begin initEEPROM()"));
  EEPROM.put(GEOFENCEENABLED_BOOL_1, false);
  EEPROM.put(GEOFENCEHOMELAT_CHAR_12, "52.4322115");
  EEPROM.put(GEOFENCEHOMELON_CHAR_12, "10.7869289");
  EEPROM.put(GEOFENCERADIUS_CHAR_7, "300");
  EEPROM.put(GEOFENCESTART_CHAR_3, "00");
  EEPROM.put(GEOFENCEEND_CHAR_3, "00");
  EEPROM.put(GEOFENCEFOLLOW_BOOL_1, false);
  EEPROM.put(KILLSWITCHENABLED_BOOL_1, false);
  EEPROM.put(KILLSWITCHSTART_CHAR_3, "00");
  EEPROM.put(KILLSWITCHEND_CHAR_3, "00");
  EEPROM.put(OWNERPHONENUMBER_CHAR_15, "5551234567");
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
  if (strcmp(temp, "t") == 0) {
//        debugPrintln(ggg("00", "00", "\"19/09/19,17:03:01-20\"")); //T
//        debugPrintln(ggg("01", "02", "\"19/09/19,17:03:02-20\"")); //F
//        debugPrintln(ggg("17", "18", "\"19/09/19,17:03:03-20\"")); //T
//        debugPrintln(ggg("22", "07", "\"19/09/19,17:03:04-20\"")); //F
//        debugPrintln(ggg("23", "21", "\"19/09/19,17:03:05-20\"")); //T
  }


  if (strcmp(temp, "b") == 0) {
        uint16_t vbat;
        fona.getBattVoltage(&vbat);
        debugPrintln(vbat);
        fona.getBattPercent(&vbat);
        debugPrintln(vbat);
      }
      

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

//  if (strcmp(temp, "g") == 0) {
//    turnGPRSOff();
//  }
//  if (strcmp(temp, "G") == 0) {
//    // turn GPRS on
//    if (!fona.enableGPRS(true))
//      debugPrintln(F("Failed to turn on"));
//  }
//  if (strcmp(temp, "l") == 0) {
//    // check for GSMLOC (requires GPRS)
//    uint16_t returncode;
//    char replybuffer[120];
//
//    if (!fona.getGSMLoc(&returncode, replybuffer, 250))
//      debugPrintln(F("Failed!"));
//    if (returncode == 0) {
//      debugPrintln(replybuffer);
//    } else {
//      Serial.print(F("Fail code #")); debugPrintln(returncode);
//    }
//  }

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

//uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout) {
//  uint16_t buffidx = 0;
//  bool timeoutvalid = true;
//  if (timeout == 0) timeoutvalid = false;
//
//  while (true) {
//    if (buffidx > maxbuff) {
//      //debugPrintln(F("SPACE"));
//      break;
//    }
//
//    while (Serial.available()) {
//      char c =  Serial.read();
//
//      //Serial.print(c, HEX); Serial.print("#"); debugPrintln(c);
//
//      if (c == '\r') continue;
//      if (c == 0xA) {
//        if (buffidx == 0)   // the first 0x0A is ignored
//          continue;
//
//        timeout = 0;         // the second 0x0A is the end of the line
//        timeoutvalid = true;
//        break;
//      }
//      buff[buffidx] = c;
//      buffidx++;
//    }
//
//    if (timeoutvalid && timeout == 0) {
//      //debugPrintln(F("TIMEOUT"));
//      break;
//    }
//    delay(1);
//  }
//  buff[buffidx] = 0;  // null term
//  return buffidx;
//}


#endif
