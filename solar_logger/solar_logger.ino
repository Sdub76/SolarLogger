//Heavily leveraged from https://github.com/EKMallon/ProMiniDatalogger-BasicStarterSketch
#include <Wire.h>
#include <EEPROM.h>
#include <RTClib.h>     // https://github.com/MrAlvin/RTClib
#include <LowPower.h>   // https://github.com/rocketscream/Low-Power

//============ HARDWARE DEFINITIONS ==========================
#define SOLAR_ADC_PIN A0    //for analog pin reading
#define RXD 0
#define TXD 1
#define RTC_INTERRUPT_PIN 2
#define SDA A4
#define SCL A5


//============ CONFIGURATION SETTINGS ========================
#define ECHO_TO_SERIAL 
#define SampleIntervalMinutes 1  // Options: 1,2,3,4,5,6,10,12,15,20,30 ONLY (must be a divisor of 60)
                                  // number of minutes the loggers sleeps between each sensor reading

//============ RTC CONFIGURATION =============================
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

//============ EEPROM MAP ==========================
// 0000 - int(32b) - InternalReferenceConstant
#define EE_IRC_ADDR     0
// 0004 - int(16b) - Reading Index
#define EE_RINDEX       4
// 0006 - EE_addr(64b) - Reading 1
#define EE_RDATA_START  6
// 0014 - EE_addr(64b) - Reading 2
// 0022 - EE_addr(64b) - Reading 3
// 0030 - EE_addr(64b) - Reading 4
// ---
// 1014 - EE_addr(64b) - Reading 127

int EE_addr = EE_RDATA_START; // assume empty EEPROM
struct EE_reading {
  uint32_t time;
  float voltage;
};
EE_reading SolarData;


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
  #ifdef ECHO_TO_SERIAL
    Serial.begin(9600);    // Open serial communications and wait for port to open:
  #endif
  Wire.begin();          // Start the i2c interface
  RTC.begin();           // RTC initialization:
  RTC.turnOffAlarm(1);
  clearClockTrigger();   // Function stops RTC from holding interrupt line low after power reset
  pinMode(RTC_INTERRUPT_PIN,INPUT_PULLUP);  //not needed if you have hardware pullups on SQW, most RTC modules do but some don't
  DateTime now = RTC.now();
  sprintf(TimeStamp, "%04d/%02d/%02d %02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute());
  enableRTCAlarmsonBackupBattery(); // only needed if you cut the pin supplying power to the DS3231 chip

  // Read Internal REference Constant from EEPROM 
  // Must run CalVref sketch first to write value to EEPROM
  long temp;
  EEPROM.get(EE_IRC_ADDR,temp);
  if (temp > 0) {
    InternalReferenceConstant = temp;
  }
  #ifdef ECHO_TO_SERIAL  
    Serial.print(F("Internal Ref Constant: "));
    Serial.println(InternalReferenceConstant);
    Serial.flush();
  #endif

  // Move reading index if non-zero value is stored in EEPROM
  EEPROM.get(EE_RINDEX,temp);
  if (temp > 0) {
    EE_addr = temp;
  }
  #ifdef ECHO_TO_SERIAL  
    Serial.print(F("Reading Index: "));
    Serial.println(EE_addr);
    Serial.flush();
  #endif
  
  //============ SYNC TO ALARM =============================
  //Delay logger start until alarm times are in sync with sampling intervals
  //this delay prevents a "short interval" from occuring @ the first hour rollover
  
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
}

// ========================================================================================================
//      *  *  *  *  *  *  MAIN LOOP   *  *  *  *  *  *
//========================================================================================================
void loop() {

  digitalWrite(LED_BUILTIN, HIGH);
  
  //============ READ BATTERY VOLTAGE ======================
  //If you are running from raw battery power (with no regulator) VccBGap is the battery voltage
  BatteryReading = getRailVoltage();
  #ifdef ECHO_TO_SERIAL
   Serial.print("Battery Voltage:");  //(optional) debugging message
   Serial.println(BatteryReading);
   Serial.flush();
  #endif

  //============ READ RTC TIME ======================
  DateTime now = RTC.now(); // reads time from the RTC
  sprintf(TimeStamp, "%04d/%02d/%02d %02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute());
  SolarData.time = now.unixtime(); // Redefine as UNIX time long int
  //loads the time into a string variable - don’t record seconds in the time stamp because the interrupt to time reading interval is <1s, so seconds are always ’00’  
  #ifdef ECHO_TO_SERIAL
   Serial.print("System taking a new reading at:");  //(optional) debugging message
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
  #ifdef ECHO_TO_SERIAL
    Serial.print(F(" TEMPERATURE from RTC is: "));
    Serial.print(rtc_TEMP_degC); 
    Serial.println(F(" Celsius"));
    Serial.flush();
  #endif

  //============ READ SOLAR VOLTAGE =================
  analogReference(DEFAULT);analogRead(SOLAR_ADC_PIN); //always throw away the first reading
  delay(5);  //optional 5msec delay lets ADC input cap adjust if needed
  analogPinReading = median_of_3( analogRead(SOLAR_ADC_PIN), analogRead(SOLAR_ADC_PIN), analogRead(SOLAR_ADC_PIN));
  float SolarCellVoltage = analogPinReading/1024.0*BatteryReading; // Convert from ADC counts to Voltage
  SolarData.voltage = SolarCellVoltage / 1000.0;
  #ifdef ECHO_TO_SERIAL
    Serial.print(F(" Voltage from Solar cell is: "));
    Serial.print(SolarData.voltage); 
    Serial.println(F(" V"));
    Serial.flush();
  #endif

  //============ RECORD TO EEPROM ===================
  // Write to EEPROM
  // Struct{Time(32b float),Voltage(32b float)}

  EEPROM.put(EE_addr,SolarData);
  #ifdef ECHO_TO_SERIAL
    // Read back what was written to EEPROM
    EE_reading temp_readback;
    EEPROM.get(EE_addr,temp_readback);
    Serial.print(F(" -> Address: "));
    Serial.print(EE_addr); 
    Serial.println(F(""));
    Serial.print(F(" -> Time: "));
    Serial.print(temp_readback.time); 
    Serial.println(F(" s"));
    Serial.print(F(" -> Volage: "));
    Serial.print(temp_readback.voltage); 
    Serial.println(F(" V"));
    Serial.flush();
  #endif

  // Move address pointer
  EE_addr += sizeof(SolarData); // Move pointer by size of reading
  EEPROM.put(EE_RINDEX,EE_addr); // Keep track of EE address in case of power failure
  if (EE_addr > EEPROM.length()) {
    EE_addr = EE_RDATA_START; // Wrap back to beginning when out of space
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
    Serial.print(F("Alarm Enabled! Going to sleep for :"));
    Serial.print(SampleIntervalMinutes);
    Serial.println(F(" minute(s)")); // println adds a carriage return
    Serial.flush();//waits for buffer to empty
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
}                                        // teriminator for median_of_3
