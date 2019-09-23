/*
   Depends on jwallis/Adafruit_FONA
   Written by Joshua Wallis, borrowing initially from AdaFruit's examples
   Support AdaFruit, buy their neato products!
*/

//TBD
// AT+CNETLIGHT=0 // stop "net" LED
// ATI - Get the module name and revision
// AT+CMEE=2 - Turn on verbose
// getSMSInterrupt()  ... eh, actually maybe not.

#include "Adafruit_FONA.h"
#include <SoftwareSerial.h>

#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4

#define KILL_SWITCH_PIN 5
#define GEOFENCE_PIN 6

SoftwareSerial fonaSS = SoftwareSerial(FONA_TX, FONA_RX);
SoftwareSerial *fonaSerial = &fonaSS;
//HardwareSerial *fonaSerial = &Serial;

Adafruit_FONA fona = Adafruit_FONA(FONA_RST);    //Adafruit_FONA_3G fona = Adafruit_FONA_3G(FONA_RST);

boolean geofenceEnabled = false;
char geofenceHomeLat[12] = "0";
char geofenceHomeLon[12] = "0";
char geofenceRadius[6] = "100";
char geofenceTZ[4] = "0";
char geofenceStart[3] = "0";
char geofenceEnd[3] = "0";
boolean killSwitchStatus = false;
char ownerPhoneNumber[13] = "+15127500974";
#define APN_ID "wholesale"


//uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
uint8_t totalErrors = 0;
int lastGPSQueryTime = 0;

void setup() {
  pinSetup();
  setupSerialAndFONA();
  waitUntilSMSReady();
  moreSetup();
  setFromFlash();
}

void loop() {
  flushFONA();
  checkSMSInput();
  checkSerialInput();
  watchDog();

  delay(5000);
}

void watchDog() {
  watchDogForErrors();
  watchDogForGPS();
  watchDogForGeofence();
}

void watchDogForGPS() {
  //TBD
  //+CCLK: "04/01/04,01:01:05+00"
  // if year > and month != 1, turn off
  // if month > and day != 1, turn off
  // if day > and hour != 0, turn off
  // convert hours to minutes (hours * 60 => minutes), then just compare minutes
}

void updateLastGPSQueryTime() {
  //TBD
}

void watchDogForGeofence() {
  if (!geofenceEnabled) {
    return;
  }

  char currentLat[12];
  char currentLon[12];
  char message[139];      // SMS max len = 140

  getGPSLatLon(currentLat, currentLon);

  if (outsideGeofence(currentLat, currentLon)) {
    
    Serial.println(F(">>>outsideGeofence"));
    
    sprintf(message, "GEOFENCE WARNING:\nHome:\ngoogle.com/search?q=%s,%s\nCurrent:\ngoogle.com/search?q=%s,%s\n", geofenceHomeLat, geofenceHomeLon, currentLat, currentLon);
    sendSMS(ownerPhoneNumber, message);
  }
}

void watchDogForErrors() {
  if (totalErrors > 10) {
    Serial.println(F("_____________ TOTAL ERRORS > 10 _______________"));
  }
}

void fixErrors() {
  Serial.println(F("FIXXXX ERRRRROORRRRSSSS"));
}

void checkSMSInput() {
  int8_t numberOfSMSs;

  for (int i = 0; i < 3; i++) {
    numberOfSMSs = fona.getNumSMS();

    if (numberOfSMSs >= 0) {
      handleSMSInput();
      Serial.print(F("."));
      return;
    }
    Serial.println(F("error in handleSMSInput()"));
    totalErrors++;
    delay(1000);
  }
}

void handleSMSInput() {
  //totalErrors = 0;  //do we want to reset here? not sure why i did this
  int8_t numberOfSMSs = fona.getNumSMS();

  if (numberOfSMSs == 0)
    return;

  char smsSender[16];
  char smsValue[51];
  int8_t smssFound = 0;

  // the SMS "slots" start at 1, not 0
  for (int8_t smsSlotNumber = 1; smssFound < numberOfSMSs; smsSlotNumber++) {
    if (isSMSSlotFilled(smsSlotNumber))
      smssFound++;
    else
      continue;

    getSMSSender(smsSlotNumber, smsSender);
    getSMSValue(smsSlotNumber, smsValue);

    toLower(smsValue);

    Serial.println(F("--read SMS--"));
    Serial.println(smsSender);
    Serial.println(smsValue);


    if (strstr(smsValue, "bat")) {
      handleBatteryReq(smsSender);
      deleteSMS(smsSlotNumber);
      continue;
    }

    if (strstr(smsValue, "info")) {
      handleInfoReq(smsSender);
      deleteSMS(smsSlotNumber);
      continue;
    }

    if (strstr(smsValue, "gps") || strstr(smsValue, "loc")) {
      handleGPSReq(smsSender);
      deleteSMS(smsSlotNumber);
      continue;
    }

    if (strstr(smsValue, "kill")) {
      handleKillSwitchReq(smsSender, smsValue);
      deleteSMS(smsSlotNumber);
      continue;
    }

    if (strstr(smsValue, "geo") || strstr(smsValue, "fence")) {
      handleGeofenceReq(smsSender, smsValue);
      deleteSMS(smsSlotNumber);
      continue;
    }

    if (strstr(smsValue, "phone")) {
      handlePhoneReq(smsSender, smsValue);
      deleteSMS(smsSlotNumber);
      continue;
    }

    // default
    handleUnknownReq(smsSender);
    deleteSMS(smsSlotNumber);
  }
}

void handleBatteryReq(char* smsSender) {
  char message[12];
  getBatteryStats(message);

  sendSMS(smsSender, message);
}

void handleInfoReq(char* smsSender) {
  char ccid[22];
  char imei[16];
  char message[50];

  fona.getSIMCCID(ccid);
  fona.getIMEI(imei);

  sprintf(message, "CCID: %s\nIMEI: %s", ccid, imei);
  sendSMS(smsSender, message);
}

void handleGPSReq(char* smsSender) {
  char message[54];
  char latitude[12];
  char longitude[12];

  getGPSLatLon(latitude, longitude);

  sprintf(message, "google.com/search?q=%s,%s", latitude, longitude);
  sendSMS(smsSender, message);
}

void handleKillSwitchReq(char* smsSender, char* smsValue) {
  char message[40];

  if (strstr(smsValue, "on") || strstr(smsValue, "enable") ) {
    setKillSwitch(true);
    sprintf(message, "Kill Switch: Enabled");
  }
  if (strstr(smsValue, "off") || strstr(smsValue, "disable") ) {
    setKillSwitch(false);
    sprintf(message, "Kill Switch: Disabled");
  }
  if (!message[0]) {
    sprintf(message, "Valid commands are:\nkill on\nkill off");
  }
  sendSMS(smsSender, message);
}

void handleGeofenceReq(char* smsSender, char* smsValue) {
  char message[132]="";

  if (strstr(smsValue, "on") || strstr(smsValue, "enable") ) {
    setGeofence(true);
    sprintf(message, "Geofence: Enabled %s-%s\nZone: %s\nRadius: %s feet\nHome:\ngoogle.com/search?q=%s,%s", geofenceStart, geofenceEnd, geofenceTZ, geofenceRadius, geofenceHomeLat, geofenceHomeLon);
  }
  if (strstr(smsValue, "off") || strstr(smsValue, "disable") ) {
    setGeofence(false);
    sprintf(message, "Geofence: Disabled\nZone: %s\nRadius: %s feet\nHome:\ngoogle.com/search?q=%s,%s", geofenceStart, geofenceEnd, geofenceTZ, geofenceRadius, geofenceHomeLat, geofenceHomeLon);
  }
  if (strstr(smsValue, "radius")) {
    if (getNumberFromString(smsValue, geofenceRadius)) {
      if (geofenceEnabled)
        sprintf(message, "Geofence: Enabled %s-%s\nZone: %s\nRadius: %s feet\nHome:\ngoogle.com/search?q=%s,%s", geofenceStart, geofenceEnd, geofenceTZ, geofenceRadius, geofenceHomeLat, geofenceHomeLon);
      else
        sprintf(message, "Geofence: Disabled\nZone: %s\nRadius: %s feet\nHome:\ngoogle.com/search?q=%s,%s", geofenceStart, geofenceEnd, geofenceTZ, geofenceRadius, geofenceHomeLat, geofenceHomeLon);
    }
  }
  if (strstr(smsValue, "home")) {
    setGeofenceHome();
    if (geofenceEnabled)
      sprintf(message, "Geofence: Enabled %s-%s\nZone: %s\nRadius: %s feet\nHome:\ngoogle.com/search?q=%s,%s", geofenceStart, geofenceEnd, geofenceTZ, geofenceRadius, geofenceHomeLat, geofenceHomeLon);
    else
      sprintf(message, "Geofence: Disabled\nZone: %s\nRadius: %s feet\nHome:\ngoogle.com/search?q=%s,%s", geofenceStart, geofenceEnd, geofenceTZ, geofenceRadius, geofenceHomeLat, geofenceHomeLon);
  }
  if (strstr(smsValue, "hours")) {
    setGeofenceHours(smsValue);
    if (geofenceEnabled)
      sprintf(message, "Geofence: Enabled %s-%s\nZone: %s\nRadius: %s feet\nHome:\ngoogle.com/search?q=%s,%s", geofenceStart, geofenceEnd, geofenceTZ, geofenceRadius, geofenceHomeLat, geofenceHomeLon);
    else
      sprintf(message, "Geofence: Disabled\nZone: %s\nRadius: %s feet\nHome:\ngoogle.com/search?q=%s,%s", geofenceStart, geofenceEnd, geofenceTZ, geofenceRadius, geofenceHomeLat, geofenceHomeLon);
  }
  if (strstr(smsValue, "zone")) {
    if (getNumberFromString(smsValue, geofenceTZ)) {
      if (geofenceEnabled)
        sprintf(message, "Geofence: Enabled %s-%s\nZone: %s\nRadius: %s feet\nHome:\ngoogle.com/search?q=%s,%s", geofenceStart, geofenceEnd, geofenceTZ, geofenceRadius, geofenceHomeLat, geofenceHomeLon);
      else
        sprintf(message, "Geofence: Disabled\nZone: %s\nRadius: %s feet\nHome:\ngoogle.com/search?q=%s,%s", geofenceStart, geofenceEnd, geofenceTZ, geofenceRadius, geofenceHomeLat, geofenceHomeLon);
    }
  }
  
  if (message[0]) {
    sendSMS(smsSender, message);
  }
  else {
    sendSMS(smsSender, "Valid commands are:\nfence on\nfence off\nfence home\nfence hours 0 21 meaning enabled 12am-9pm\nfence radius 100 meaning 100 feet");
  }
}

void handlePhoneReq(char* smsSender, char* smsValue) {
  char message[41]="";
  
  if (strstr(smsValue, "set")) {
    // if number is found: use it
    // if no number is found: use smsSender
    //TBD eeprom set ownerphone#
//    sprintf(message, "Setting owner phone # to %s", fromEEPROM);
  }
  else {
    //TBD
//    sprintf(message, "Owner phone # is %s", fromEEPROM);
  }
  sendSMS(smsSender, message);
}

void handleUnknownReq(char* smsSender) {
  sendSMS(smsSender, "Valid options:\nbat\ninfo\nloc\nkill on/off\nfence on/off/home/hours/zone/radius");
}




boolean isSMSSlotFilled(int8_t smsSlotNumber) {
  char smsSender[15];

  // TBD would be nice to check for errors here, totalErrors++
  // but I'm not sure what the response is when in error state

  if (fona.getSMSSender(smsSlotNumber, smsSender, 15)) {
    return true;
  }
  return false;
}

void getSMSSender(int8_t smsSlotNumber, char* smsSender) {
  for (int i = 0; i < 3; i++) {
    if (fona.getSMSSender(smsSlotNumber, smsSender, 15))
      break;
    Serial.println(F("  Failed getting SMS sender"));
    delay(1000);
  }
}

void getSMSValue(int8_t smsSlotNumber, char* smsValue) {
  uint16_t smsValueLength;

  for (int i = 0; i < 3; i++) {
    if (fona.readSMS(smsSlotNumber, smsValue, 50, &smsValueLength))
      break;
    Serial.println(F("  Failed getting SMS value"));
    delay(1000);
  }
}

void checkSerialInput() {
  if (!Serial.available()) {
    return;
  }

  char command;
  command = Serial.read();
  Serial.println(command);
  handleInput(command);
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

void setFONAGPS(boolean tf) {
  if (!tf) {
    fona.enableGPS(false);

    for (int k = 0; k < 3; k++) {
      if (fona.GPSstatus() == 0)
        return;
      delay(1000);
    }
    Serial.println(F("Failed to turn off GPS"));
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
  for (int i = 1; i < 4; i++) {
    if (fona.GPSstatus() >= 0)
      break;
    delay(i * 1000);
  }

  // error, give up
  if (fona.GPSstatus() < 0) {
    totalErrors++;
    Serial.println(F("Failed to turn on GPS"));
    return;
  }

  // off, turn on
  if (fona.GPSstatus() == 0) {
    fona.enableGPS(true);
    delay(5000);
  }

  // no fix, wait and ask again
  for (int j = 1; j < 9; j++) {
    if (fona.GPSstatus() >= 2)
      return;
    delay(j * 2 * 1000);
  }

  // no fix, give up
  if (fona.GPSstatus() < 2) {
    totalErrors++;
    Serial.println(F("Failed to get GPS fix"));
    return;
  }
}

void getGPSLatLon(char* latitude, char* longitude) {
  Serial.println(F(">>>getGPSLatLon"));
  char gpsString[120];

  setFONAGPS(true);
  updateLastGPSQueryTime();

  fona.getGPS(0, gpsString, 120);

  // full string:
  // 1,1,20190913060459.000,30.213823,-97.782017,204.500,1.87,90.1,1,,1.2,1.5,0.9,,11,6,,,39,,
  Serial.println(gpsString);

  // strtok is DESTRUCTIVE to the string in operates on. THANKS STRTOK!
  getOccurrenceInDelimitedString(gpsString, latitude, 4, ',');
  getOccurrenceInDelimitedString(gpsString, longitude, 5, ',');
}

void getTime(char* currentTime) {
  char gpsString[120];

  setFONAGPS(true);
  updateLastGPSQueryTime();

  fona.getGPS(0, gpsString, 120);
  getOccurrenceInDelimitedString(gpsString, currentTime, 3, ',');  
}

////////////////////////////////
//GEOFENCE

void setGeofence(boolean tf) {
  if (tf) {
    geofenceEnabled = true;
    digitalWrite(GEOFENCE_PIN, HIGH);
  }
  else {
    geofenceEnabled = false;
    digitalWrite(GEOFENCE_PIN, LOW);
  }
}

void setGeofenceHome() {
  getGPSLatLon(geofenceHomeLat, geofenceHomeLon);
}

void setGeofenceHours(char* smsValue) {
  // fence hours 0 21
  // means 12am - 9pm
  getOccurrenceInDelimitedString(smsValue, geofenceStart, 3, ' ');
  getOccurrenceInDelimitedString(smsValue, geofenceEnd, 4, ' ');
}

////////////////////////////////
//KILL SWITCH

void setKillSwitch(boolean tf) {
  if (tf) {
    killSwitchStatus = true;
    digitalWrite(KILL_SWITCH_PIN, HIGH);
  }
  else {
    killSwitchStatus = true;
    digitalWrite(KILL_SWITCH_PIN, LOW);
  }
}

////////////////////////////////
//GPRS

void turnGPRSOff() {
  for (int i = 0; i < 3; i++) {
    if (fona.enableGPRS(false))
      return;
    delay(1000);
  }
  Serial.println(F("  Failed to turn GPRS off"));
}

void turnGPRSOn() {
  for (int i = 0; i < 3; i++) {
    if (fona.enableGPRS(true))
      return;
    delay(1000);
  }
  Serial.println(F("  Failed to turn GPRS on"));
}

////////////////////////////////
//SMS

void deleteSMS(uint8_t msg_number) {
  for (int i = 2; i < 10; i++) {
    Serial.println(F("  Attempting to delete SMS"));
    if (fona.deleteSMS(msg_number)) {
      Serial.println(F("  Success deleting SMS"));
      return;
    }
    delay(i * 1000);
  }
  Serial.println(F("  Failed to delete SMS"));
}

void sendSMS(char* send_to, char* message) {
  flushSerial();
  Serial.println(F("  Attempting to send SMS:"));
  Serial.println(message);
  if (!fona.sendSMS(send_to, message)) {
    Serial.println(F("  Failed to send SMS"));
  } else {
    Serial.println(F("  Success sending SMS"));
  }
}








///////////////////////////////////////////////////////////////////////////////////////////
//HELPERS
///////////////////////////////////////////////////////////////////////////////////////////

void sendRawCommand(char* command) {
  delay(200);
  Serial.println(command);
  fona.println(command);
  delay(1000);
  
  if (fona.available()) {
    flushFONA();
  }
}

void toLower(char* str) {
  for (int i = 0; str[i]; i++)
    str[i] = tolower(str[i]);
}

boolean getNumberFromString(char* in, char* out) {
  // if in == "aaa123bbb"
  // sets out = "123"
  // be sure to leave room for '\0'
  boolean foundNumber = false;
  short outCount = 0;

  for (int i = 0; in[i]; i++) {
    if ((in[i] >= '0' && in[i] <= '9') || in[i] == '-') {
      foundNumber = true;
      out[outCount] = in[i];
      outCount++;
    }
    if (foundNumber && !(in[i] >= '0' && in[i] <= '9')) {
      break;
    }
  }
  out[outCount] = '\0';

  return foundNumber;
}

void getOccurrenceInDelimitedString(char* in, char* out, short occur, char delim) {
  // if in == "a,b,c"
  // and occur = 2
  // and delim = ','
  // sets out = "b"
  // be sure to leave room for '\0'
  short delimCount = 0;
  short outCount = 0;

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
    }
  }
  out[outCount] = '\0';
}

boolean outsideGeofence(char* lat1Str, char* lon1Str) {
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

  Serial.println(dist_calc, 10);

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
//  boolean timeoutvalid = true;
//  if (timeout == 0) timeoutvalid = false;
//
//  while (true) {
//    if (buffidx > maxbuff) {
//      //Serial.println(F("SPACE"));
//      break;
//    }
//
//    while (Serial.available()) {
//      char c =  Serial.read();
//
//      //Serial.print(c, HEX); Serial.print("#"); Serial.println(c);
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
//      //Serial.println(F("TIMEOUT"));
//      break;
//    }
//    delay(1);
//  }
//  buff[buffidx] = 0;  // null term
//  return buffidx;
//}



///////////////////////////////////////////////////////////////////////////////////////////
//SETUP
///////////////////////////////////////////////////////////////////////////////////////////

void pinSetup() {
  pinMode(GEOFENCE_PIN, OUTPUT);
  pinMode(KILL_SWITCH_PIN, OUTPUT);
}

void setupSerialAndFONA() {
  while (!Serial);

  Serial.begin(115200);
  Serial.println(F("Initializing....(May take 3 seconds)"));

  fonaSerial->begin(4800);
  if (! fona.begin(*fonaSerial)) {
    Serial.println(F("Couldn't find FONA"));
    while (1);
  }

  Serial.println(F("FONA is OK"));
}

void printFONAType() {
  type = fona.type();
  switch (type) {
    case FONA800L:
      Serial.println(F("FONA 800L")); break;
    case FONA800H:
      Serial.println(F("FONA 800H")); break;
    case FONA808_V1:
      Serial.println(F("FONA 808 (v1)")); break;
    case FONA808_V2:
      Serial.println(F("FONA 808 (v2)")); break;
    case FONA3G_A:
      Serial.println(F("FONA 3G (American)")); break;
    case FONA3G_E:
      Serial.println(F("FONA 3G (European)")); break;
    default:
      Serial.println(F("FOMA ??? Unknown type")); break;
  }
}

void waitUntilSMSReady() {
  for (int i = 0; i < 10; i++) {
    delay(1000);
    if (fona.getNumSMS() >= 0) {
      Serial.println(F("SMS is ready"));
      return;
    }
  }
  fixErrors();
}

void moreSetup() {
  setAPN();
  printFONAType();

  sendRawCommand("AT+CMEE=2");
  sendRawCommand("AT+CNETLIGHT=0");
  sendRawCommand("ATI");
}

void setAPN() {
  //TBD
  fona.setGPRSNetworkSettings(F(APN_ID));
}

void setFromFlash() {
  //TBD
  setGeofence(geofenceEnabled);
  setKillSwitch(killSwitchStatus);
}














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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////











void handleInput(char command) {

//  char temp[12];
//  char latitude[15];
//  char longitude[15];
//  char ch[19];


  switch (command) {

    case 'w': {
        watchDogForGeofence();

        break;
      }
      //
      //
      //    case 'n': {
      //        // read the network/cellular status
      //        uint8_t n = fona.getNetworkStatus();
      //        Serial.print(F("Network status "));
      //        Serial.print(n);
      //        Serial.print(F(": "));
      //        if (n == 0) Serial.println(F("Not registered"));
      //        if (n == 1) Serial.println(F("Registered (home)"));
      //        if (n == 2) Serial.println(F("Not registered (searching)"));
      //        if (n == 3) Serial.println(F("Denied"));
      //        if (n == 4) Serial.println(F("Unknown"));
      //        if (n == 5) Serial.println(F("Registered roaming"));
      //        break;
      //      }
      //
      //    /*** SMS ***/
      //
      //    case 'N': {
      //        // read the number of SMS's!
      //        int8_t smsnum = fona.getNumSMS();
      //        if (smsnum < 0) {
      //          Serial.println(F("Could not read # SMS"));
      //        } else {
      //          Serial.print(smsnum);
      //          Serial.println(F(" SMS's on SIM card!"));
      //        }
      //        break;
      //      }
      //    case 'r': {
      //        // read an SMS
      //        flushSerial();
      //        Serial.print(F("Read #"));
      //        uint8_t smsn = readnumber();
      //        Serial.print(F("\n\rReading SMS #")); Serial.println(smsn);
      //
      //
      //        // Retrieve SMS sender address/phone number.
      //        if (! fona.getSMSSender(smsn, replybuffer, 250)) {
      //          Serial.println("Failed!");
      //          break;
      //        }
      //        Serial.print(F("FROM: ")); Serial.println(replybuffer);
      //
      //        // Retrieve SMS value.
      //        uint16_t smslen;
      //        if (! fona.readSMS(smsn, replybuffer, 250, &smslen)) { // pass in buffer and max len!
      //          Serial.println("Failed!");
      //          break;
      //        }
      //        Serial.print(F("***** SMS #")); Serial.print(smsn);
      //        Serial.print(" ("); Serial.print(smslen); Serial.println(F(") bytes *****"));
      //        Serial.println(replybuffer);
      //        Serial.println(F("*****"));
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
      //          Serial.print(F("\n\rReading SMS #")); Serial.println(smsn);
      //          if (!fona.readSMS(smsn, replybuffer, 250, &smslen)) {  // pass in buffer and max len!
      //            Serial.println(F("Failed!"));
      //            break;
      //          }
      //          // if the length is zero, its a special case where the index number is higher
      //          // so increase the max we'll look at!
      //          if (smslen == 0) {
      //            Serial.println(F("[empty slot]"));
      //            smsnum++;
      //            continue;
      //          }
      //
      //          Serial.print(F("***** SMS #")); Serial.print(smsn);
      //          Serial.print(" ("); Serial.print(smslen); Serial.println(F(") bytes *****"));
      //          Serial.println(replybuffer);
      //          Serial.println(F("*****"));
      //        }
      //        break;
      //      }
      //
      //    case 'd': {
      //        // delete an SMS
      //        flushSerial();
      //        Serial.print(F("Delete #"));
      //        uint8_t smsn = readnumber();
      //
      //        Serial.print(F("\n\rDeleting SMS #")); Serial.println(smsn);
      //        deleteSMS(smsn);
      //        break;
      //      }
      //
      //
      //    /******** GPRS is HTTP transport *******/
      //    /**** we might use for GSM location ****/
      //    case 'g': {
      //        turnGPRSOff();
      //        break;
      //      }
      //    case 'G': {
      //        // turn GPRS on
      //        if (!fona.enableGPRS(true))
      //          Serial.println(F("Failed to turn on"));
      //        break;
      //      }
      //    case 'l': {
      //        // check for GSMLOC (requires GPRS)
      //        uint16_t returncode;
      //
      //        if (!fona.getGSMLoc(&returncode, replybuffer, 250))
      //          Serial.println(F("Failed!"));
      //        if (returncode == 0) {
      //          Serial.println(replybuffer);
      //        } else {
      //          Serial.print(F("Fail code #")); Serial.println(returncode);
      //        }
      //
      //        break;
      //      }
      //
      //
      //    /*****************************************/
      
          case 'S': {
              Serial.println(F("Creating SERIAL TUBE"));
              while (1) {
                while (Serial.available()) {
                  delay(1);
                  fona.write(Serial.read());
                }
                if (fona.available()) {
                  Serial.write(fona.read());
                }
              }
              break;
            }
  }

  flushSerial();
  flushFONA();

}
