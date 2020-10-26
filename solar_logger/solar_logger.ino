
// Science Fair 2020-2021 Solar Data Logger
//
// HW ideas from https://thecavepearlproject.org/2019/02/21/easy-1-hour-pro-mini-classroom-datalogger-build-update-feb-2019/
// SW ideas from https://github.com/EKMallon/ProMiniDatalogger-BasicStarterSketch
//
// Step 1 - Run SetTime sketch to initialize RTC (https://github.com/EKMallon/Utilities)
// Step 2 - Run CalVref sketch to calibrate ADC (https://github.com/openenergymonitor/emontx2/tree/master/firmware/CalVref)
// Step 3 - Run this data logger sketch to record data to EEPROM
// Step 4 - Run eeprom_datadump sketch to extract EEPROM data to CSV format
// Step 5 - Run eeprom_clear sketch to clear EEPROM (TODO)
//  TODO: Integrate SetTime,CalVref,datadump,clear sketches into utility sketch with selection menu

#include <Wire.h>                      // Arduino I2C library
#include <EEPROM.h>                    // Arduino Internal EEPROM library
#include <SparkFun_External_EEPROM.h>  // https://github.com/sparkfun/SparkFun_External_EEPROM_Arduino_Library
#include <RTClib.h>                    // https://github.com/MrAlvin/RTClib
#include <LowPower.h>                  // https://github.com/rocketscream/Low-Power

//============ CONFIGURATION SETTINGS ========================
#define ECHO_TO_SERIAL                // Enable UART status messages at slight power cost (comment out for false)
#define RESET_CONFIG          false   // Start back at the zero EEPROM address
#define START_HOUR            5       // Don't record solar data before 5am box time (temp and battery are monitored 24/7)
#define STOP_HOUR             21      // Don't record solar data after 9pm box time (temp and battery are monitored 24/7)
#define SampleIntervalMinutes 1       // number of minutes the loggers sleeps between each sensor reading
                                      // 1  = 68.3 hr (4.3 days @ 16 hr/day)
                                      // 2  = 136.5 hr (8.5 days @ 16 hr/day)
                                      // 3  = 204.8 hr (12.8 days @ 16 hr/day)
                                      // 4  = 273.1 hr (17.1 days @ 16 hr/day)
                                      // 5  = 341.3 hr (21.3 days @ 16 hr/day)
                                      // 6  = 409.6 hr (25.6 days @ 16 hr/day)
                                      // 10 = 682.7 hr (42.7 days @ 16 hr/day) 
                                      // 12 = 819.2 hr (51.2 days @ 16 hr/day) ** Nov 1 - Dec 21
                                      // 15 = 1024 hr (64.0 days @ 16 hr/day)
                                      // 20 = 1365 hr (85.3 days @ 16 hr/day)
                                      // 30 = 2048 hr (128 days @ 16 hr/day)

//============ HARDWARE DEFINITIONS ==========================
// Arduino Pro Mini 328p 8MHz
// No Vreg, 10-bit ADC
// External interrupts on pins 2,3 only
// https://www.arduino.cc/en/uploads/Main/Arduino-Pro-Mini-schematic.pdf
// https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
#define SOLAR_ADC_PIN     A0   // Solar cell input
#define RTC_INTERRUPT_PIN 2    // RTC Alarm interrupt input


//============ INTERNAL EEPROM MAP ==========================
// 0000 - long(32b) - InternalReferenceConstant
#define EE_IRC_ADDR       0
// 0004 - long(32b) - Next External EEPROM Reading Index
#define EE_RINDEX         4
// 0008 - float(32b) - Minimum Battery Voltage Reading
#define EE_BATT_MIN       8
// 0012 - long(32b) - Minimum Battery Voltage Time
#define EE_BATT_MIN_TIME 12
// 0016 - float(32b) - Minimum Ambient Temp Reading
#define EE_TAMB_MIN      16
// 0020 - long(32b) - Minimum Ambient Temp Time
#define EE_TAMB_MIN_TIME 20
// 0024 - float(32b) - Maximum Ambient Temp Reading
#define EE_TAMB_MAX      24
// 0028 - long(32b) - Maximum Ambient Temp Time
#define EE_TAMB_MAX_TIME 28
// 0032 - Empty
// ---
// 1024 - Empty


//============ I2C EEPROM CONFIG ============================
// AT24C256 I2C module
// http://ww1.microchip.com/downloads/en/devicedoc/atmel-8568-seeprom-at24c256c-datasheet.pdf
#define EE_SIZE         32768 // Bytes
#define EE_PAGELENGTH   64    // Bits
#define EE_I2C_ADDRESS  0x58
ExternalEEPROM ext_eep;

//============ EXTERNAL EEPROM MAP ==========================
#define extEE_RDATA_START  0
// 00000 - extEE_reading(64b) - Reading 1
// 00008 - extEE_reading(64b) - Reading 2
// 00016 - extEE_reading(64b) - Reading 3
// 00024 - extEE_reading(64b) - Reading 4
// ---
// 32768 - extEE_reading(64b) - Reading 4096

struct extEE_reading {
  uint32_t time;
  float voltage;
};
extEE_reading SolarData;


//============ I2C RTC CONFIGURATION =========================
// DS3231 + AT24C32 I2C module
// https://datasheets.maximintegrated.com/en/ds/DS3231.pdf
// http://ww1.microchip.com/downloads/en/DeviceDoc/doc0336.pdf
// variables for reading the RTC time & handling the D2=INT(0) alarm interrupt signal it generates
RTC_DS3231 RTC; // creates an RTC object in the code
#define DS3231_I2C_ADDRESS 0x68
#define DS3231_CONTROL_REG 0x0E
byte Alarmhour;
byte Alarmminute;
byte Alarmday;
char TimeStamp[ ] = "0000/00/00,00:00"; //16 ascii characters (without seconds because they are always zeros on wakeup)
volatile boolean clockInterrupt = false;  //this flag is set to true when the RTC interrupt handler is executed
float rtc_TEMP_degC;



//============ Global variables ==============================
// Most are used for temporary storage during communications & calculations
byte bytebuffer1 = 0;     // for functions that return a byte - usually comms with sensors
byte bytebuffer2 = 0;     // second buffer for 16-bit sensor register readings
int integerBuffer = 9999;    // for temp-swapping ADC readings
float floatbuffer = 9999.9;  // for temporary float calculations
int analogPinReading = 0;
int BatteryReading = 0;
long InternalReferenceConstant = 1126400;  // Nominal value in case EEPROM is empty
long extEE_addr = extEE_RDATA_START; // assume empty EEPROM
float batt_min = 999.0;
float tamb_min = 999.0;
float tamb_max = -999.0;


//======================================================================================================================
//  *  *   *   *   *   *   SETUP   *   *   *   *   *
//======================================================================================================================
void setup() {

  //============ PIN CONFIGURATION =============================
  // builds that jumper A4->A2 and A5->A3 to bring the I2C bus to the screw terminals MUST DISABLE digital I/O on these two pins
  // If you are doing only ADC conversions on the analog inputs you can disable the digital buffers on those pins, to save power
  // Once the input buffer is disabled, a digitalRead on those A-pins will always be zero.
  bitSet (DIDR0, ADC0D);  // disable digital buffer on A0
  bitSet (DIDR0, ADC1D);  // disable digital buffer on A1
  bitSet (DIDR0, ADC2D);  // disable digital buffer on A2
  bitSet (DIDR0, ADC3D);  // disable digital buffer on A3
  //setting UNUSED digital pins to INPUT_PULLUP reduces noise & risk of accidental short
  pinMode(7,INPUT_PULLUP); //only if you do not have anything connected to this pin
  pinMode(8,INPUT_PULLUP); //only if you do not have anything connected to this pin
  pinMode(9,INPUT_PULLUP); //only if you do not have anything connected to this pin
  #ifndef ECHO_TO_SERIAL
    pinMode(0,INPUT_PULLUP); //but not if we are connected to usb
    pinMode(1,INPUT_PULLUP); //then these pins are needed for RX & TX 
  #endif

  //============ BEGIN PERIPHERALS =============================
  // Serial (UART)
  #ifdef ECHO_TO_SERIAL
    Serial.begin(9600);
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("<<< BEGIN SERIAL COMMUNICATIONS >>>"));
    Serial.flush();
  #endif


  // Real-Time Clock (I2C)
  RTC.begin();
  RTC.turnOffAlarm(1);
  clearClockTrigger();   // Function stops RTC from holding interrupt line low after power reset
  pinMode(RTC_INTERRUPT_PIN,INPUT_PULLUP);  //not needed if you have hardware pullups on SQW, most RTC modules do but some don't
  DateTime now = RTC.now();
  sprintf(TimeStamp, "%04d/%02d/%02d %02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute());
  enableRTCAlarmsonBackupBattery(); // only needed if you cut the pin supplying power to the DS3231 chip
  #ifdef ECHO_TO_SERIAL
    Serial.println(F("<<< RTC INITIALIZED >>>"));
    Serial.flush();
  #endif

  // External EEPROM (I2C)
  if (ext_eep.begin() == false) {
    #ifdef ECHO_TO_SERIAL
      Serial.print(F("No memory detected. Freezing."));
    #endif
    digitalWrite(LED_BUILTIN, HIGH); // Leave red LED on if EEPROM doesn't initialize
    while (1);  //TODO Maybe add logic to write smaller dataset to internal EE only if external EE doesn't init instead of freezing
  }
  ext_eep.setMemorySize(EE_SIZE); //In bytes. 512kbit = 64kbyte
  ext_eep.setPageSize(EE_PAGELENGTH); //In bytes. Has 128 byte page size.
  ext_eep.enablePollForWriteComplete(); //Supports I2C polling of write completion
  #ifdef ECHO_TO_SERIAL
    Serial.println(F("<<< EXTERNAL EEPROM INITIALIZED >>>"));
    Serial.flush();
  #endif

  #ifdef ECHO_TO_SERIAL
    Serial.println(F("<<< READING CONFIG DATA FROM INTERNAL EEPROM >>>"));
    Serial.flush();
  #endif
  long ltemp;
  float ftemp;
  // Read Internal Reference Constant from EEPROM 
  // Must run CalVref sketch first to write value to EEPROM
  EEPROM.get(EE_IRC_ADDR,ltemp);
  if (ltemp < 1228800L && ltemp > 1024000L) {
    InternalReferenceConstant = ltemp;
  }
  #ifdef ECHO_TO_SERIAL  
    Serial.print(F("- Internal Ref Constant: "));
    Serial.println(InternalReferenceConstant);
    Serial.flush();
  #endif

  // Override default config params if RESET_CONFIG is false
  if (!RESET_CONFIG) {
    EEPROM.get(EE_RINDEX,ltemp);
    if ((ltemp > 0) && (ltemp < EE_SIZE)) {
      extEE_addr = ltemp;
    }
    // Read Environmental extremes from EEPROM
    EEPROM.get(EE_BATT_MIN,ftemp);
    if ((ftemp > 0.0) && (ftemp < 4.0)) {
      batt_min = ftemp;
    }
    EEPROM.get(EE_TAMB_MIN,ftemp);
    if ((ftemp != 0.00) && (ftemp < 80.0)) {
      tamb_min = ftemp;
    }
    EEPROM.get(EE_TAMB_MAX,ftemp);
    if (ftemp > 0.0) {
      tamb_max = ftemp;
    }
  }
  #ifdef ECHO_TO_SERIAL  
    Serial.print(F("- Reading Index: "));
    Serial.println(extEE_addr);
    Serial.print(F("- Battery Min: "));
    Serial.print(batt_min);
    Serial.println(F(" V"));
    Serial.print(F("- Ambient Min Temp: "));
    Serial.print(tamb_min);
    Serial.println(F(" °F"));
    Serial.print(F("- Ambient Max Temp: "));
    Serial.print(tamb_max);
    Serial.println(F(" °F"));
    Serial.flush();
  #endif


  //============ SYNC TO ALARM =============================
  //Delay logger start until alarm times are in sync with sampling intervals
  //this delay prevents a "short interval" from occuring @ the first hour rollover
  #ifdef ECHO_TO_SERIAL
    Serial.println(F("<<< SYNCHRONIZING TO ALARM >>>"));
    Serial.flush();
  #endif
    Alarmhour = now.hour(); Alarmminute = now.minute();
    int syncdelay=Alarmminute % SampleIntervalMinutes;  // 7 % 10 = 7 because 7 / 10 < 1, e.g. 10 does not fit even once in seven. So the entire value of 7 becomes the remainder.
    syncdelay=SampleIntervalMinutes-syncdelay; // when SampleIntervalMinutes is 1, syncdelay is 1, other cases are variable
    Alarmminute = Alarmminute + syncdelay;
    if (Alarmminute > 59) {  // check for roll-overs
    Alarmminute = 0; Alarmhour = Alarmhour + 1; 
    if (Alarmhour > 23) { Alarmhour = 0;}
    }
    RTC.setAlarm1Simple(Alarmhour, Alarmminute);
    RTC.turnOnAlarm(1);
    attachInterrupt(0, rtcISR, LOW);  // program hardware interrupt to respond to D2 pin being brought 'low' by RTC alarm
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);  //this puts logger to sleep
    detachInterrupt(0); // disables the interrupt after the alarm wakes the logger
    RTC.turnOffAlarm(1); // turns off the alarm on the RTC chip

  #ifdef ECHO_TO_SERIAL
    Serial.println(F("<<< INITIALIZATION COMPLETE >>>"));
    Serial.flush();
  #endif

}

// ========================================================================================================
//      *  *  *  *  *  *  MAIN LOOP   *  *  *  *  *  *
//========================================================================================================
void loop() {

  digitalWrite(LED_BUILTIN, HIGH);

  //============ READ RTC TIME ======================
  DateTime now = RTC.now(); // reads time from the RTC
  sprintf(TimeStamp, "%04d/%02d/%02d %02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute());
  SolarData.time = now.unixtime(); // Convert time to UNIX time
  //loads the time into a string variable - don’t record seconds in the time stamp because the interrupt to time reading interval is <1s, so seconds are always ’00’  
  #ifdef ECHO_TO_SERIAL
   Serial.print("Current Time: ");
   Serial.println(TimeStamp);
   Serial.flush();
  #endif

  //============ READ RTC TEMP ======================
  // Read RTC Temp Register... not recorded
  // Note: the DS3231 temp registers only update every 64seconds
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0x11);       //the register where the temp data is stored
  Wire.endTransmission(); // nothing really happens until the complier sends the .endTransmission command
  Wire.requestFrom(DS3231_I2C_ADDRESS, 2);   //ask for two bytes of data
  if (Wire.available()) {
  byte tMSB = Wire.read();            //2’s complement int portion
  byte tLSB = Wire.read();             //fraction portion
  rtc_TEMP_degC = ((((short)tMSB << 8) | (short)tLSB) >> 6) / 4.0;  // Allows for readings below freezing: thanks to Coding Badly
  }
  else {
    rtc_TEMP_degC = 0;  //if rtc_TEMP_degC contains zero, then you had a problem reading from the RTC!
  }
  float rtc_TEMP_degF = rtc_TEMP_degC * 1.8 + 32.0;
  #ifdef ECHO_TO_SERIAL
    Serial.print(F("Ambient Temp: "));
    Serial.print(rtc_TEMP_degF); 
    Serial.println(F(" °F"));
    Serial.flush();
  #endif

    
  //============ READ BATTERY VOLTAGE ======================
  //If you are running from raw battery power (with no regulator) VccBGap is the battery voltage
  BatteryReading = getRailVoltage();
  #ifdef ECHO_TO_SERIAL
   Serial.print(F("Battery Voltage: ")); 
   Serial.print(BatteryReading/1000.0);
   Serial.println(F(" V"));
  #endif

//============ RECORD ENVIRONMENTAL EXTREMES =====
  // Don't record Battery min if it's > 4.0V because you're plugged in
  if ((BatteryReading/1000.0 < batt_min) && (BatteryReading/1000.0 < 4.0)) { 
    EEPROM.put(EE_BATT_MIN_TIME,SolarData.time);
    EEPROM.put(EE_BATT_MIN,BatteryReading/1000.0);
    EEPROM.get(EE_BATT_MIN,batt_min);
 #ifdef ECHO_TO_SERIAL
    Serial.print(F(" *** New Min Battery Voltage: "));
    Serial.print(batt_min);
    Serial.print(F(" V ("));
    Serial.print(SolarData.time);
    Serial.println(F(") ***"));
    Serial.flush();
 #endif
   }
  if (rtc_TEMP_degF < tamb_min) {
    EEPROM.put(EE_TAMB_MIN_TIME,SolarData.time);
    EEPROM.put(EE_TAMB_MIN,rtc_TEMP_degF);
    EEPROM.get(EE_TAMB_MIN,tamb_min);
#ifdef ECHO_TO_SERIAL
    Serial.print(F(" *** New Min Ambient Temp: "));
    Serial.print(tamb_min);
    Serial.print(F(" °F ("));
    Serial.print(SolarData.time);
    Serial.println(F(") ***"));
    Serial.flush();
 #endif
   }
  if (rtc_TEMP_degF > tamb_max) {
    EEPROM.put(EE_TAMB_MAX_TIME,SolarData.time);
    EEPROM.put(EE_TAMB_MAX,rtc_TEMP_degF);
    EEPROM.get(EE_TAMB_MAX,tamb_max);
 #ifdef ECHO_TO_SERIAL
    Serial.print(F(" *** New Max Ambient Temp: "));
    Serial.print(tamb_max);
    Serial.print(F(" °F ("));
    Serial.print(SolarData.time);
    Serial.println(F(") ***"));
    Serial.flush();
 #endif
   }

  // Only Record Solar Data between the designated start and stop times to preserve EEPROM space
  if ((now.hour() > START_HOUR) && (now.hour() < STOP_HOUR)) {
    
    //============ READ SOLAR VOLTAGE =================
    analogReference(DEFAULT);analogRead(SOLAR_ADC_PIN); //always throw away the first reading
    delay(5);  //optional 5msec delay lets ADC input cap adjust if needed
    analogPinReading = median_of_3( analogRead(SOLAR_ADC_PIN), analogRead(SOLAR_ADC_PIN), analogRead(SOLAR_ADC_PIN));
    float SolarCellVoltage = analogPinReading/1023.0*BatteryReading; // Convert from ADC counts to Voltage
    SolarData.voltage = SolarCellVoltage / 1000.0;
    #ifdef ECHO_TO_SERIAL
      Serial.print(F("Solar Reading: "));
      Serial.print(SolarData.voltage); 
      Serial.print(F(" V ("));
      Serial.print(analogPinReading); 
      Serial.println(F(" ADC counts)"));
      Serial.flush();
    #endif
  
    //============ RECORD TO EEPROM ===================
    ext_eep.put(extEE_addr,SolarData);
    #ifdef ECHO_TO_SERIAL
      // Read back what was written to EEPROM
      extEE_reading tmp;
      ext_eep.get(extEE_addr,tmp);
      Serial.print(F(" -> EEPROM Address: "));
      Serial.print(extEE_addr); 
      Serial.print(F(" (0x"));
      Serial.print(extEE_addr,HEX); 
      Serial.print(F(", Reading "));
      Serial.print((extEE_addr - extEE_RDATA_START)/sizeof(SolarData)+1); 
      Serial.print(F(" of "));
      Serial.print((EE_SIZE - extEE_RDATA_START)/sizeof(SolarData)); 
      Serial.println(F(")"));
      Serial.print(F(" -> Time: "));
      Serial.println(tmp.time); 
      Serial.print(F(" -> Voltage: "));
      Serial.println(tmp.voltage); 
      Serial.flush();
    #endif
  
    // Move address pointer
    extEE_addr += sizeof(SolarData); // Move pointer by size of reading
    EEPROM.put(EE_RINDEX,extEE_addr); // Keep track of EE address in case of power failure
    // Wrap back to beginning if there's not enough space for the next reading
    if ((extEE_addr + sizeof(SolarData)) > ext_eep.length()) {
      extEE_addr = extEE_RDATA_START; // Wrap back to beginning when out of space
    }
  } else {
    #ifdef ECHO_TO_SERIAL
      // Read back what was written to EEPROM
      Serial.println(F("It's nighttime... no solar data to record"));
      Serial.print(F(" -> EEPROM Address: "));
      Serial.print(extEE_addr); 
      Serial.print(F(" (0x"));
      Serial.print(extEE_addr,HEX); 
      Serial.print(F(", Reading "));
      Serial.print((extEE_addr - extEE_RDATA_START)/sizeof(SolarData) + 1); 
      Serial.print(F(" of "));
      Serial.print((EE_SIZE - extEE_RDATA_START)/sizeof(SolarData)); 
      Serial.println(F(")"));
      Serial.flush();
    #endif
   }


  //============ SET NEXT ALARM TIME ================
  Alarmhour = now.hour();
  Alarmminute = now.minute() + SampleIntervalMinutes;
  Alarmday = now.day();
  // check for roll-overs
  if (Alarmminute > 59) { //error catching the 60 rollover!
    Alarmminute = 0;
    Alarmhour = Alarmhour + 1;
    if (Alarmhour > 23) {
      Alarmhour = 0;
      // put ONCE-PER-DAY code here -it will execute on the 24 hour rollover
    }
  }
  // then set the alarm
  RTC.setAlarm1Simple(Alarmhour, Alarmminute);
  RTC.turnOnAlarm(1);
  if (RTC.checkAlarmEnabled(1)) {
  #ifdef ECHO_TO_SERIAL
    Serial.print(F("Going to sleep for "));
    Serial.print(SampleIntervalMinutes);
    Serial.println(F(" minutes")); 
    Serial.println(F("Good Night!")); 
    Serial.flush();
  #endif
  }
  
  //============ SLEEP UNTIL ALARM ================
  // Enable interrupt on pin2 & attach it to rtcISR function:
  attachInterrupt(0, rtcISR, LOW);
  // Enter power down state with ADC module disabled to save power:
  digitalWrite(LED_BUILTIN, LOW);
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_ON);
  //processor starts HERE AFTER THE RTC ALARM WAKES IT UP
  detachInterrupt(0); // immediately disable the interrupt on waking
  // We set the clockInterrupt in the ISR, deal with that now:
  if (clockInterrupt) {
    if (RTC.checkIfAlarm(1)) {   //Is the RTC alarm still on?
      RTC.turnOffAlarm(1);       //then turn it off.
    }
  clockInterrupt = false;   //reset the interrupt flag to false
  }
  
  Serial.println(F("*********************")); 
  Serial.println(F("Good Morning!")); 
    
}

//====================================================================================
// Stand alone functions called from the main loop:
//====================================================================================

//====================================================================================
// This is the Interrupt subroutine that only executes when the RTC alarm goes off:
void rtcISR() {                      //called from attachInterrupt(0, rtcISR, LOW);
    clockInterrupt = true;
  }


//====================================================================================
void clearClockTrigger()   // from http://forum.arduino.cc/index.php?topic=109062.0
{
  Wire.beginTransmission(0x68);   //Tell devices on the bus we are talking to the DS3231
  Wire.write(0x0F);               //Tell the device which address we want to read or write
  Wire.endTransmission();         //Before you can write to and clear the alarm flag you have to read the flag first!
  Wire.requestFrom(0x68,1);       //Read one byte
  bytebuffer1=Wire.read();        //In this example we are not interest in actually using the byte
  Wire.beginTransmission(0x68);   //Tell devices on the bus we are talking to the DS3231 
  Wire.write(0x0F);               //Status Register: Bit 3: zero disables 32kHz, Bit 7: zero enables the main oscilator
  Wire.write(0b00000000);         //Bit1: zero clears Alarm 2 Flag (A2F), Bit 0: zero clears Alarm 1 Flag (A1F)
  Wire.endTransmission();
  clockInterrupt=false;           //Finally clear the flag we used to indicate the trigger occurred
}

//====================================================================================
// Enable Battery-Backed Square-Wave Enable on the DS3231 RTC module: 
/* Bit 6 (Battery-Backed Square-Wave Enable) of DS3231_CONTROL_REG 0x0E, can be set to 1 
 * When set to 1, it forces the wake-up alarms to occur when running the RTC from the back up battery alone. 
 * [note: This bit is usually disabled (logic 0) when power is FIRST applied]
 */
void enableRTCAlarmsonBackupBattery()
{
  Wire.beginTransmission(DS3231_I2C_ADDRESS);// Attention RTC 
  Wire.write(DS3231_CONTROL_REG);            // move the memory pointer to CONTROL_REG
  Wire.endTransmission();                    // complete the ‘move memory pointer’ transaction
  Wire.requestFrom(DS3231_I2C_ADDRESS,1);    // request data from register
  byte resisterData = Wire.read();           // byte from registerAddress
  bitSet(resisterData, 6);                   // Change bit 6 to a 1 to enable
  Wire.beginTransmission(DS3231_I2C_ADDRESS);// Attention RTC
  Wire.write(DS3231_CONTROL_REG);            // target the register
  Wire.write(resisterData);                  // put changed byte back into CONTROL_REG
  Wire.endTransmission();
}
  
//====================================================================================
int getRailVoltage()    // modified from http://forum.arduino.cc/index.php/topic,38119.0.html
{
  int result; // gets passed back to main loop
  int value;  // temp variable for the conversion to millivolts
    // ADC configuration command for ADC on 328p based Arduino boards  // REFS1 REFS0  --> 0 1, AVcc internal ref. // MUX3 MUX2 MUX1 MUX0 -->1110 sets 1.1V bandgap
    ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1) | (0 << MUX0);
    // Note: changing the ADC reference from the (default) 3.3v rail to internal 1.1V bandgap can take up to 10 msec to stabilize
    for (int i = 0; i < 7; i++) { // loop several times so the capacitor connected to the aref pin can discharge down to the 1.1v reference level
    ADCSRA |= _BV( ADSC );                     // Start an ADC conversion
    while ( ( (ADCSRA & (1 << ADSC)) != 0 ) ); // makes the processor wait for ADC reading to complete
    value = ADC; // value = the ADC reading
    delay(1);    // delay time for capacitor on Aref to discharge
    } // for(int i=0; i <= 5; i++)
    ADMUX = bit (REFS0) | (0 & 0x07); analogRead(A0); // post reading cleanup: select input channel A0 + re-engage the default rail as Aref
    result = (((InternalReferenceConstant) / (long)value)); //scale the ADC reading into milliVolts  
    return result;
  
}  // terminator for getRailVoltage() function

//================================================================================================
//  SIGNAL PROCESSING FUNCTIONS
//================================================================================================
/* 
This median3 filter is pretty good at getting rid of single NOISE SPIKES from flakey sensors
(It is better than any low pass filter, moving average, weighted moving average, etc. 
IN TERMS OF ITS RESPONSE TIME and its ability  to ignore such single-sample noise spike outliers. 
The median-of-3 requires very little CPU power, and is quite fast.
*/
// pass three separate positive integer readings into this filter:
// for more on bitwise xor operator see https://www.arduino.cc/reference/en/language/structure/bitwise-operators/bitwisexor/  
int median_of_3( int a, int b, int c ){  // created by David Cary 2014-03-25
    int the_max = max( max( a, b ), c );
    int the_min = min( min( a, b ), c );
    int the_median = the_max ^ the_min ^ a ^ b ^ c;
    return( the_median );
}
