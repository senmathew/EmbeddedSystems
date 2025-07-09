// Arduino Mega: NH3, CO2 Logger with SD Card and RTC (SerLCD 20x4 Qwiic Display Integration)
//Author: Sen Mathew
//Carbon Catcher | Meaglow Ltd.
//Ammonia & Carbon Dioxide Detection & Logging System

// Wiring schematic Module with Mega:

//-----------DS3231-----------//
//I2C Communication: SCL: Pin 21 ; SDA: Pin 20
//Power : 5V & GND

//-----------SD Module-----------//
//SPI Communiction: CD: Pin 49 ; CS: Pin 53; DI: Pin 51; DO: Pin 50; CLK: Pin 52
//Power: 5V & GND

//-----------LCD 20X4-----------//
//I2C Communication: SDA: SDA1 Pin ; SCL: SCL1 Pin --> Via 5v to 3.3V level shifter
//Power: RAW Pin: 3.3V ; GND: GND

//-----------Keypad-----------//
// Keypad Pin from left to right with keys facing you connection with Mega digitl pins respectively 
//Keypad_pINOUT(NC,1,2,3,4,5,6,7,NC) = (NC, C2, R1, C1, R4, C3, R3, R2, NC) 
//Mega_pINOUT                        = (NC, 31, 33, 30, 36, 32, 35, 34, NC)

#include <Wire.h>
#include <SerLCD.h> // SparkFun SerLCD library
#include <Keypad.h>
#include <SPI.h>
#include <SD.h> 
#include <RTClib.h>
#include <EEPROM.h>

// LCD - SerLCD 20x4
SerLCD lcd; // Uses default I2C address 0x72

// Keypad
const byte ROWS = 4;
const byte COLS = 3;
char hexaKeys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte rowPins[ROWS] = {33, 34, 35, 36};
byte colPins[COLS] = {30, 31, 32};
Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

// NH3 Sensor
const int sensorPin = A0;
const float Vref = 5.0;
const int ADC_resolution = 1024;
const float sensitivity_nA_per_ppm = 8.0;
const float feedback_resistor_ohm = 390000.0;
int sensorMax = 640;
int zeroOffset = 0;

// RTC
RTC_DS3231 rtc;

// SD
const int chipSelect = 53;
const int sdDetectPin = 49; // CD pin from SD module
File dataFile;
bool headerWritten = false;
String logFileName = "";
bool loggingEnabled = false;
bool sdInserted = true;
bool sdRemovedSinceLastPrompt = false;

// Use Serial1 for CO2 from UNO1 (hardware UART)
#define co2Serial Serial1
String co2Buffer = "";
bool newCO2Data = false;
String valA = "---";
String valB = "---";

// Logging interval control
unsigned long logIntervalMillis = 60000;
unsigned long lastLogMillis = 0;

void(*softReset) (void) = 0; // Function pointer to address 0 for software reset

void setup() 
{
  pinMode(sdDetectPin, INPUT);
  Serial.begin(9600);
  co2Serial.begin(9600);
  Wire.begin();
  lcd.begin(Wire);
  lcd.setBacklight(255, 255, 255);
  lcd.setContrast(5);
  lcd.clear();

  if (!rtc.begin()) {
    lcd.setCursor(0, 2);
    lcd.print("RTC error!"); while (1);
  }

  lcd.setCursor(0, 0);
  lcd.print("CARBON RECYCLER");
  lcd.setCursor(0, 2);
  lcd.print("Initializing: ");
  displayCountdown(60,15,2);
  delay(100);

  if (SD.begin(chipSelect)) {
    sdInserted = true;
    lcd.setCursor(0, 2);
    lcd.print("SD Ready");
  } else {
    sdInserted = false;
    lcd.setCursor(0, 2);
    lcd.print("No SD Card");
    delay(1500);
    clearDisplayExceptTitle();
  }

  // Load saved settings
  loadSavedSettings();
  generateLogFileName();
  loggingEnabled = true;

  if (sdInserted) {
    dataFile = SD.open(logFileName, FILE_WRITE);
    if (dataFile) {
      if (dataFile.size() == 0) {
        dataFile.println("Timestamp;NH3 (ppm);CO2 Inside (ppm);CO2 Outside (ppm)");
      }
      dataFile.close();
      headerWritten = true;
    } else {
      lcd.setCursor(0, 2);
      lcd.print("File create FAIL");
      Serial.println("SD file creation failed!");
    }
  }

  lcd.setCursor(0, 2); lcd.print("NH3/CO2 Logger Ready");
  delay(500);
  clearDisplayExceptTitle();
}


void loop() {
  if (digitalRead(sdDetectPin) == HIGH && !sdInserted && sdRemovedSinceLastPrompt) {
    clearDisplayExceptTitle();
    lcd.setCursor(0, 2);
    lcd.print("SD inserted");
    delay(500);
    if (SD.begin(chipSelect)) {
      generateLogFileName();
      sdInserted = true;
      sdRemovedSinceLastPrompt = false;
      clearDisplayExceptTitle();
      lcd.setCursor(0, 2);
      lcd.print("SD Ready");
      delay(500);
      clearDisplayExceptTitle();
    } else {
      clearDisplayExceptTitle();
      lcd.setCursor(0, 2);
      lcd.print("SD init fail");
    }
  }

  char key = customKeypad.getKey();
  if (key == '*') calibration();
  if (key == '#') zeroCalibration();
  if (key == '0') resetLogging();
  if (key == '1') { promptSetTime(); promptSetLogInterval(); saveSettingsToEEPROM(); clearDisplayExceptTitle(); lcd.setCursor(0, 2); lcd.print("Settings saved"); delay(1000); clearDisplayExceptTitle(); loggingEnabled = true; }
  if (key == '2') { loggingEnabled = false; clearDisplayExceptTitle(); lcd.setCursor(0, 2); lcd.print("Logging: OFF"); delay(500); clearDisplayExceptTitle(); }
  if (key == '3') promptSetLogInterval();
  if (key == '6') 
  { 
    clearDisplayExceptTitle(); 
    lcd.setCursor(0,2);  lcd.print("  Wipe EEPROM? ");
    lcd.setCursor(0,3);  lcd.print("  *:Yes   #:No ");

    while (true) {
    char confirm = customKeypad.getKey();
    if (confirm == '*') { wipeEEPROMSettings(); break; }
    if (confirm == '#') { clearDisplayExceptTitle(); break; }
  }
}
  if (key == '8') { clearDisplayExceptTitle(); lcd.setCursor(0, 2); lcd.print("Resetting..."); delay(500); softReset(); }
  if (key == '7') shutdownSystem();
  if (key == '9') safeRemoveSD();

  int raw = analogRead(sensorPin) - zeroOffset;
  raw = constrain(raw, 0, sensorMax);
  float voltage = (raw * Vref) / ADC_resolution;
  float current_uA = (voltage / feedback_resistor_ohm) * 1e6;
  float ppm = (current_uA * 1000.0) / sensitivity_nA_per_ppm;
  if (ppm > 1000) ppm = 1000;

  while (co2Serial.available()) 
  {
    char c = co2Serial.read();
    if (c == '<') {
      co2Buffer = "";
      newCO2Data = false; 
    } else if (c == '>') {
      int commaIdx = co2Buffer.indexOf(',');
      if (commaIdx > 0 && commaIdx < co2Buffer.length() - 1) {
        valA = co2Buffer.substring(0, commaIdx);
        valB = co2Buffer.substring(commaIdx + 1);
        newCO2Data = true;
      }
    } else {
      co2Buffer += c;
    }
  }

  DateTime now = rtc.now();
  char timestamp[20];
  sprintf(timestamp, "%02d%02d", now.month(), now.day());

  char timestamp_log[20];
  sprintf(timestamp_log, "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());

  char nh3Buffer[7], co2InBuffer[7], co2OutBuffer[7];
  dtostrf(ppm, 6, 1, nh3Buffer);  // width=6, precision=1
  snprintf(co2InBuffer, sizeof(co2InBuffer), "%6s", valA.c_str());
  snprintf(co2OutBuffer, sizeof(co2OutBuffer), "%6s", valB.c_str());


  lcd.setCursor(0, 0);
  lcd.print("CARBON RECYCLER");
  lcd.setCursor(16,0);
  lcd.print(timestamp);
  // lcd.setCursor(0, 1);
  // lcd.print(timestamp);
  lcd.setCursor(0, 1);
  lcd.print("NH3   :"); lcd.print(nh3Buffer); lcd.setCursor(13,1);lcd.print("ppm");
  lcd.setCursor(0, 2);
  lcd.print("CO2IN :"); lcd.print(co2InBuffer); lcd.setCursor(13,2);lcd.print("ppm"); lcd.setCursor(17, 2); lcd.print("LOG");
  lcd.setCursor(0, 3);
  lcd.print("CO2OUT:"); lcd.print(co2OutBuffer); lcd.setCursor(13,3);lcd.print("ppm");
  lcd.setCursor(17, 3);
  lcd.print(loggingEnabled ? "On" : "Off");

 

  if (loggingEnabled && sdInserted && (millis() - lastLogMillis >= logIntervalMillis)) {
    String logLine = String(timestamp_log) + ";" + String(ppm, 1) + ";" + valA + ";" + valB;

    dataFile = SD.open(logFileName, FILE_WRITE);
    if (dataFile) {
      if (!headerWritten) {
        dataFile.println("Timestamp;NH3 (ppm);CO2 Inside (ppm);CO2 Outside (ppm)");
        headerWritten = true;
      }
      dataFile.println(logLine);
      dataFile.close();
      Serial.println(logLine);
    } else {
      Serial.println("SD open fail");
    }

    lastLogMillis = millis();
  }

  delay(10);
}


void safeRemoveSD() {
  loggingEnabled = false;
  clearDisplayExceptTitle();
  lcd.setCursor(0, 2);
  lcd.print("Removing SD...");
  delay(1000);
  clearDisplayExceptTitle();
  lcd.setCursor(0, 2);
  lcd.print("Safe to Remove");

  unsigned long timeout = millis();
  while (digitalRead(sdDetectPin) == HIGH && millis() - timeout < 10000) 
  {
    lcd.setCursor(0,3); lcd.print("Countdown: "); displayCountdown(10,12,3);
    delay(100);
  }

  clearDisplayExceptTitle();
  lcd.setCursor(0, 2);
  if (digitalRead(sdDetectPin) == LOW) {
    lcd.print("SD Removed");
    sdInserted = false;
    sdRemovedSinceLastPrompt = true;
  } else {
    lcd.print("Timeout...");
  }
  delay(1500);
  clearDisplayExceptTitle();
}

void shutdownSystem() {
  loggingEnabled = false;
  clearDisplayExceptTitle();
  lcd.setCursor(0, 2);
  lcd.print("Shutting down...");
  delay(1000);

  if (sdInserted) {
    clearDisplayExceptTitle();
    lcd.setCursor(0, 2);
    lcd.print("Finalizing SD...");
    dataFile = SD.open(logFileName, FILE_WRITE);
    if (dataFile) {
      dataFile.flush();
      dataFile.close();
    }
    delay(500);
  }

  clearDisplayExceptTitle();
  lcd.setCursor(0, 2);
  lcd.print("Safe to power off");
  while (true); // halt
}

void calibration() {
  clearDisplayExceptTitle();
  lcd.setCursor(0, 2); lcd.print("Calibrating...   ");
  int maxVal = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
    int val = analogRead(sensorPin);
    if (val > maxVal) maxVal = val;
    delay(10);
  }
  sensorMax = maxVal;
  lcd.setCursor(0, 2); lcd.print("Span Cal Done    ");
  delay(1000);
  clearDisplayExceptTitle();
}

void zeroCalibration() {
  clearDisplayExceptTitle();
  lcd.setCursor(0, 2); lcd.print("Zero Calib...    ");
  long total = 0;
  for (int i = 0; i < 50; i++) {
    total += analogRead(sensorPin);
    delay(20);
  }
  zeroOffset = total / 50;
  lcd.setCursor(0, 2); lcd.print("Zero Cal Done    ");
  delay(1000);
  clearDisplayExceptTitle();
}

void resetLogging() {
  clearDisplayExceptTitle();
  lcd.setCursor(0, 1); lcd.print("  Reset log file ?  ");
  lcd.setCursor(0, 2); lcd.print("   *:Yes     #:No   ");
  while (true) {
    char k = customKeypad.getKey();
    if (k == '*') {
      SD.remove(logFileName);
      headerWritten = false;
      clearDisplayExceptTitle();
      lcd.setCursor(0, 2);
      lcd.print("Log Reset Done");
      delay(500);
      clearDisplayExceptTitle();
      return;
    } else if (k == '#') {
      clearDisplayExceptTitle();
      lcd.setCursor(0, 2);
      lcd.print("Reset Cancelled");
      delay(500);
      clearDisplayExceptTitle();
      return;
    }
  }
}

void promptSetTime() {
  clearDisplayExceptTitle();
  lcd.setCursor(0, 2);
  lcd.print("Set YYYYMMDDhhmm");
  String input = "";
  while (input.length() < 12) {
    char key = customKeypad.getKey();
    if (key >= '0' && key <= '9') {
      input += key;
      lcd.setCursor(input.length() - 1, 3);
      lcd.print(key);
    }
  }
  int year = input.substring(0, 4).toInt();
  int month = input.substring(4, 6).toInt();
  int day = input.substring(6, 8).toInt();
  int hour = input.substring(8, 10).toInt();
  int minute = input.substring(10, 12).toInt();
  rtc.adjust(DateTime(year, month, day, hour, minute, 0));
  clearDisplayExceptTitle();
  lcd.setCursor(0, 2);
  lcd.print("Time Set");
  delay(500);
  clearDisplayExceptTitle();
}

void promptSetLogInterval() {
  clearDisplayExceptTitle();
  lcd.setCursor(0, 1); lcd.print("Set Logging Interval");
  lcd.setCursor(0, 2);
  lcd.print("1:Sec  2:Min");

  char unitKey = 0;
  while (true) {
    unitKey = customKeypad.getKey();
    if (unitKey == '1' || unitKey == '2') break;
  }

  bool isSeconds = (unitKey == '1');
  clear_row(2);
  lcd.setCursor(0, 2);
  lcd.print(isSeconds ? "Secs:" : "Mins:");

  String input = "";
  while (true) {
    char key = customKeypad.getKey();
    if (key >= '0' && key <= '9') {
      input += key;
      lcd.setCursor(0, 3);
      lcd.print(input + "   ");
    } else if (key == '#' && input.length() > 0) {
      int val = input.toInt();
      logIntervalMillis = isSeconds ? val * 1000UL : val * 60000UL;
      clear_row(2);clear_row(3);
      lcd.setCursor(0, 2);
      lcd.print("Set OK");
      delay(500);
      clearDisplayExceptTitle();
      break;
    }
  }
}


void generateLogFileName() {
  DateTime now = rtc.now();
  char name[13];
  sprintf(name, "LOG_%02d%02d.CSV", now.month(), now.day());
  logFileName = String(name);

  dataFile = SD.open(logFileName, FILE_WRITE);
  if (dataFile) {
    if (!headerWritten) {
      dataFile.println("Timestamp;NH3 (ppm);CO2 Inside (ppm);CO2 Outside (ppm)");
      headerWritten = true;
    }
    dataFile.close();
    Serial.print("Log file created: ");
    Serial.println(logFileName);
  } else {
    Serial.print("Failed to create log file: ");
    Serial.println(logFileName);
  }
}

void clearDisplayExceptTitle() {
  lcd.setCursor(0, 1); lcd.print("                    ");
  lcd.setCursor(0, 2); lcd.print("                    ");
  lcd.setCursor(0, 3); lcd.print("                    ");
}
void clear_row(int row){
  lcd.setCursor(0,row); lcd.print("                    ");
}

void displayCountdown(int seconds, int column, int row) {
  for (int i = seconds; i >= 0; i--) {
    lcd.setCursor(column, row); // You can change line/column as needed
    lcd.print(i);
    lcd.print("s     "); // Padding to overwrite old digits
    delay(1000);
  }
}

void loadSavedSettings() {
  EEPROM.get(0, logIntervalMillis);
  int year, month, day, hour, minute;
  EEPROM.get(4, year);
  EEPROM.get(8, month);
  EEPROM.get(12, day);
  EEPROM.get(16, hour);
  EEPROM.get(20, minute);
  rtc.adjust(DateTime(year, month, day, hour, minute, 0));

  // Visual confirmation
  clearDisplayExceptTitle();
  lcd.setCursor(0, 2);
  lcd.print("Settings loaded");
  lcd.setCursor(0, 3);
  // lcd.print(logIntervalMillis / 1000);
  // lcd.print("s ");
  lcd.print(year);
  lcd.print("/");
  lcd.print(month);
  lcd.print("/");
  lcd.print(day);
  delay(2000);
  clearDisplayExceptTitle();
}


void saveSettingsToEEPROM() {
  EEPROM.put(0, logIntervalMillis);
  DateTime now = rtc.now();
  EEPROM.put(4, now.year());
  EEPROM.put(8, now.month());
  EEPROM.put(12, now.day());
  EEPROM.put(16, now.hour());
  EEPROM.put(20, now.minute());
}

void wipeEEPROMSettings() {
  unsigned long defaultInterval = 15*60000; // 15 min
  EEPROM.put(0, defaultInterval);
  EEPROM.put(4, 2025);
  EEPROM.put(8, 1);
  EEPROM.put(12, 1);
  EEPROM.put(16, 0);
  EEPROM.put(20, 0);
  lcd.setCursor(0, 2); lcd.print("EEPROM wiped");
  delay(1000);
  clearDisplayExceptTitle();
}
