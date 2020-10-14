// Date, Time, Alarm and Temperature functions using a DS3231 RTC connected via I2C and Wire lib

#include <Wire.h>
#include <RTClib.h>

RTC_DS3231 rtc;

#if defined(ARDUINO_ARCH_SAMD)
// for Zero, output on USB Serial console, remove line below if using programming port to program the Zero!
   #define Serial SerialUSB
#endif

    // The following string constants are saved in flash only
    // If an array of characters are defined as 'const' and with a set size of the double array, then it is possible to 
    // use PROGMEM and save much RAM space.  Size of array is: [number of strings][longest string + 2]
const char daysOfTheWeek[7][10] PROGMEM = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
const char months[12][11] PROGMEM = {"January", "February", "March", "April", "May", "June", "July", "August","September", "October", "November", "December"};

DateTime tc;  //variable of type DateTime

char buffer[3]; //char buffer used for simple formatting of output, so minutes and seconds are always shown with two digits


void setup () {

#ifndef ESP8266
    while (!Serial); // for Leonardo/Micro/Zero
#endif

    Serial.begin(57600);
    
    if (! rtc.begin()) {
    Serial.println( F("Couldn't find RTC") );  // using F() saves RAM - if text is longer than 4 characters
    while (1);   // sketch halts in an endless loop
  }
    
  
  if (! rtc.isrunning()) {
    Serial.println( F("RTC is NOT running!") );
  
    //set the RTC date & time - to the time this sketch was compiled
    rtc.adjust(DateTime(__DATE__, __TIME__));
 
    // set the RTC date & time - to any time you choose examples 
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));      // would set to January 21, 2014 at 3:00:00 (3am)
    // rtc.adjust(DateTime(2015, 2, 28, 14, 50, 00));   // would set to February 28, 2015 at 14:50:00 (2:50pm)

    if (! rtc.isrunning()) {
      Serial.println( F("Halted") );
      while(1);
    }
  }
  
  //get time 
  tc = rtc.now();
  
  //get temperature 
  float c = rtc.getTemperature();

  //print temperature
  Serial.print( F("(chip) temperature is ") );
  Serial.print(c);
  Serial.print( F("* Celcius and  "));
  Serial.print( ( c * (9/5) ) + 32);
  Serial.println( F("* Fahrenheit") );
  
  //set alarm 1
  rtc.setAlarm1Simple(tc.hour(), tc.minute() + 1 );
  rtc.turnOnAlarm(1);
  if (rtc.checkAlarmEnabled(1)) {
    Serial.print( F("   Alarm 1 set to ") );
    Serial.print(tc.hour());
    Serial.print(":");
    sprintf(buffer, "%02d", tc.minute() + 1);
    Serial.println(buffer);
  }

  Serial.println( F("Press [return] at any time, to begin setting RTC Time via serial input") );
  Serial.println();
  
}//setup()


void loop () {
 
    
    //get time 
    tc = rtc.now(); // read the time from the DS3231 RTC chip
    
    printTime24H();
    printTime_AM_PM();

  // check (in software) to see if Alarm 1 has been triggered
    if (rtc.checkIfAlarm(1)) {
      Serial.println( F("Alarm 1 Triggered") );
    }

    Serial.println();
    pause(3000); 
}//loop()


void pause(int delay_ms) {
  while(delay_ms){ //short for:  while(delay_ms <> 0)
    if( Serial.available() ) {
      oldSerialFlush();  // flush serial recieve buffer (so NL and/or CR is removed before looking for entered numbers)
      setTimeSerial();
      delay_ms = 1; //ready to go back to running the main loop
    }
    delay(1);  
    delay_ms--; //short for: delay_ms = delay_ms - 1
  }
}//pause()


void printTime24H() {
    Serial.print(tc.year(), DEC);
    Serial.print('/');
    Serial.print(tc.month(), DEC);
    Serial.print('/');
    sprintf(buffer, "%02d", tc.day());
    Serial.print(buffer);
    Serial.print(' ');
    Serial.print(tc.hour(), DEC);
    Serial.print(':');
    sprintf(buffer, "%02d", tc.minute());
    Serial.print(buffer);
    Serial.print(':');
    sprintf(buffer, "%02d", tc.second());
    Serial.print(buffer);
    Serial.println();
}//printTime24H()


void printTime_AM_PM() {
  const char* AMPM = 0;
  Serial.print( daysOfTheWeek[tc.dayOfTheWeek()] );
  Serial.print(" ");
  Serial.print(months[tc.month()-1]);
  Serial.print(" ");
  Serial.print(tc.day());
  Serial.print(", ");
  Serial.print(tc.year());
  Serial.print(" ");
  byte hour = tc.hour();
  if (hour > 12) {
    hour -= 12;
    AMPM = " PM";
  }else{
    AMPM = " AM";
  }
  Serial.print(hour);
  Serial.print(":");
  sprintf(buffer, "%02d", tc.minute() );
  Serial.print(buffer);
  Serial.println(AMPM);
}//printTime_AM_PM()


void setTimeSerial(){
  Serial.print( F("Please enter the current year, 00-99. - ") );
  byte year = readByte();
  Serial.println(year);
  Serial.println();
  
  Serial.print( F("Please enter the current month, 1-12. - ") );
  byte month = readByte();
  Serial.println(months[month-1]);
  Serial.println();
  
  Serial.print( F("Please enter the current day of the month, 1-31. - ") );
  byte day = readByte();
  Serial.println(day);
  Serial.println();
  
  Serial.print( F("Please enter the current hour in 24hr format, 0-23. - ") );
  byte hour = readByte();
  Serial.println(hour);
  Serial.println();
  
  Serial.print( F("Please enter the current minute, 0-59. - ") );
  byte minute = readByte();
  Serial.println(minute);

  Serial.println();
  Serial.println( F("Please enter your select second, 0-59. - ") );
  Serial.println( F("Press [return/enter] button at the correct second") );
  byte second = readByte();
  Serial.println(second);
  
  rtc.adjust(DateTime(year, month, day, hour, minute, second));
  Serial.println( F("The RTC has been set.") );
}//setTimeSerial()


byte readByte() {
  while (!Serial.available()) delay(10);
  byte reading = 0;
  byte incomingByte = Serial.read();
  while (incomingByte != '\n') {  // '\n' means 'newline' 
    if (incomingByte >= '0' && incomingByte <= '9')
      reading = reading * 10 + (incomingByte - '0');
    else;
    incomingByte = Serial.read();
  }
  oldSerialFlush(); //empties incomming serial buffer
  return reading;
}//readByte()


void oldSerialFlush(){
  while ( Serial.available() ) {
    byte incomingByte = Serial.read();
  }
}//oldSerialFlush()
  
