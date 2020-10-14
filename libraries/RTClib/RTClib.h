// Code by JeeLabs http://news.jeelabs.org/code/
// Released to the public domain! Enjoy!

// Merged DS3231 functions from: github/coobro/RTClib  by  MrAlvin 2012-02-27
// Alarm code for DS3231 heavily used/modified from Eric Ayars DS3231 library  by  Coobro
// Eric Ayars code is located at: http://hacks.ayars.org/2011/04/ds3231-real-time-clock.html

#ifndef _RTCLIB_H_
#define _RTCLIB_H_

#include <Arduino.h>
class TimeSpan;

// Simple general-purpose date/time class (no TZ / DST / leap second handling!)
class DateTime {
public:
    DateTime (uint32_t t =0);
    DateTime (uint16_t year, uint8_t month, uint8_t day,
                uint8_t hour =0, uint8_t min =0, uint8_t sec =0);
    DateTime (const DateTime& copy);
    DateTime (const char* date, const char* time);
    DateTime (const __FlashStringHelper* date, const __FlashStringHelper* time);
    uint16_t year() const       { return 2000 + yOff; }
    uint8_t month() const       { return m; }
    uint8_t day() const         { return d; }
    uint8_t hour() const        { return hh; }
    uint8_t minute() const      { return mm; }
    uint8_t second() const      { return ss; }
    uint8_t dayOfTheWeek() const;

    // 32-bit times as seconds since 1/1/2000
    long secondstime() const;   
    // 32-bit times as seconds since 1/1/1970
    uint32_t unixtime(void) const;
    
    //ISO 8601 Timestamp function
    enum timestampOpt{
        TIMESTAMP_FULL, TIMESTAMP_TIME, TIMESTAMP_DATE
    };
    String timestamp(timestampOpt opt = TIMESTAMP_FULL);

    DateTime operator+(const TimeSpan& span);
    DateTime operator-(const TimeSpan& span);
    TimeSpan operator-(const DateTime& right);

protected:
    uint8_t yOff, m, d, hh, mm, ss;
};

// Timespan which can represent changes in time with seconds accuracy.
class TimeSpan {
public:
    TimeSpan (int32_t seconds = 0);
    TimeSpan (int16_t days, int8_t hours, int8_t minutes, int8_t seconds);
    TimeSpan (const TimeSpan& copy);
    int16_t days() const         { return _seconds / 86400L; }
    int8_t  hours() const        { return _seconds / 3600 % 24; }
    int8_t  minutes() const      { return _seconds / 60 % 60; }
    int8_t  seconds() const      { return _seconds % 60; }
    int32_t totalseconds() const { return _seconds; }

    TimeSpan operator+(const TimeSpan& right);
    TimeSpan operator-(const TimeSpan& right);

protected:
    int32_t _seconds;
};

// RTC based on the DS1307 chip connected via I2C and the Wire library
enum Ds1307SqwPinMode { OFF = 0x00, ON = 0x80, SquareWave1HZ = 0x10, SquareWave4kHz = 0x11, SquareWave8kHz = 0x12, SquareWave32kHz = 0x13 };

class RTC_DS1307 {
public:
    boolean begin(void);
    static void adjust(const DateTime& dt);
    uint8_t isrunning(void);
    static DateTime now();
    static Ds1307SqwPinMode readSqwPinMode();
    static void writeSqwPinMode(Ds1307SqwPinMode mode);
    uint8_t readnvram(uint8_t address);
    void readnvram(uint8_t* buf, uint8_t size, uint8_t address);
    void writenvram(uint8_t address, uint8_t data);
    void writenvram(uint8_t address, uint8_t* buf, uint8_t size);
};

// RTC based on the PCF8523 chip connected via I2C and the Wire library
enum Pcf8523SqwPinMode { PCF8523_OFF = 7, PCF8523_SquareWave1HZ = 6, PCF8523_SquareWave32HZ = 5, PCF8523_SquareWave1kHZ = 4, PCF8523_SquareWave4kHz = 3, PCF8523_SquareWave8kHz = 2, PCF8523_SquareWave16kHz = 1, PCF8523_SquareWave32kHz = 0 };

class RTC_PCF8523 {
public:
    boolean begin(void);
    void adjust(const DateTime& dt);
    boolean isrunning(void);
    static DateTime now();

    Pcf8523SqwPinMode readSqwPinMode();
    void writeSqwPinMode(Pcf8523SqwPinMode mode);
};

// RTC using the internal millis() clock, has to be initialized before use
class RTC_Millis {
public:
    void checkRollover();
	
    boolean begin(void);
    void adjust(const DateTime& dt);
    uint8_t isrunning(void) {return 1;};
    DateTime now();
    Ds1307SqwPinMode readSqwPinMode();
    void writeSqwPinMode(Ds1307SqwPinMode mode) {};
    uint8_t readnvram(uint8_t address) {};
    void readnvram(uint8_t* buf, uint8_t size, uint8_t address) {};
    void writenvram(uint8_t address, uint8_t data) {};
    void writenvram(uint8_t address, uint8_t* buf, uint8_t size) {};
	
protected:
    long offset;
// Support for millis rollover:
// 1. Periodically compare current millis() with previosly captured millis
// 2. When previus millis is greater than current, a rollover count is increased
// 3. In calculating now(), use additional count of 2^32/1000 to compensate for rollovers
    unsigned long prevMillis;
    unsigned int  countRollovers;
};

// RTC based on the DS3231 chip connected via I2C
class RTC_DS3231
{
public:
    uint8_t begin(void);
    void adjust(const DateTime& dt);
    uint8_t isrunning(void);
    DateTime now();

    // Temperature function

    float getTemperature();     

    void getA1Time(byte& A1Day, byte& A1Hour, byte& A1Minute, byte& A1Second, byte& AlarmBits, bool& A1Dy, bool& A1h12, bool& A1PM); 
/* Retrieves everything you could want to know about alarm
 * one. 
 * A1Dy true makes the alarm go on A1Day = Day of Week,
 * A1Dy false makes the alarm go on A1Day = Date of month.
 *
 * byte AlarmBits sets the behavior of the alarms:
 *	Dy	A1M4	A1M3	A1M2	A1M1	Rate
 *	X	1		1		1		1		Once per second
 *	X	1		1		1		0		Alarm when seconds match
 *	X	1		1		0		0		Alarm when min, sec match
 *	X	1		0		0		0		Alarm when hour, min, sec match
 *	0	0		0		0		0		Alarm when date, h, m, s match
 *	1	0		0		0		0		Alarm when DoW, h, m, s match
 *
 *	Dy	A2M4	A2M3	A2M2	Rate
 *	X	1		1		1		Once per minute (at seconds = 00)
 *	X	1		1		0		Alarm when minutes match
 *	X	1		0		0		Alarm when hours and minutes match
 *	0	0		0		0		Alarm when date, hour, min match
 *	1	0		0		0		Alarm when DoW, hour, min match
 */	
    void getA2Time(byte& A2Day, byte& A2Hour, byte& A2Minute, byte& AlarmBits, bool& A2Dy, bool& A2h12, bool& A2PM); 
        // Same as getA1Time();, but A2 only goes on seconds == 00.
    void setA1Time(byte A1Day, byte A1Hour, byte A1Minute, byte A1Second, byte AlarmBits, bool A1Dy, bool A1h12, bool A1PM); 
        // Set the details for Alarm 1	
    void setAlarm1Simple(byte hour, byte minute);
        // A simple hour/minute alarm.
    void setA2Time(byte A2Day, byte A2Hour, byte A2Minute, byte AlarmBits, bool A2Dy, bool A2h12, bool A2PM); 
        // Set the details for Alarm 2
    void setAlarm2Simple(byte hour, byte minute);
        // A simple hour/minute alarm.	
    void turnOnAlarm(byte Alarm); 
        // Enables alarm 1 or 2 and the external interrupt pin.
        // If Alarm != 1, it assumes Alarm == 2.
    void turnOffAlarm(byte Alarm); 
        // Disables alarm 1 or 2 (default is 2 if Alarm != 1);
        // and leaves the interrupt pin alone.
    bool checkAlarmEnabled(byte Alarm); 
        // Returns T/F to indicate whether the requested alarm is
        // enabled. Defaults to 2 if Alarm != 1.
    bool checkIfAlarm(byte Alarm); 
        // Checks whether the indicated alarm (1 or 2, 2 default);
        // has been activated.

    // Oscillator functions

    void enableOscillator(bool TF, bool battery, byte frequency); 
        // turns oscillator on or off. True is on, false is off.
        // if battery is true, turns on even for battery-only operation,
        // otherwise turns off if Vcc is off.
        // frequency must be 0, 1, 2, or 3.
        // 0 = 1 Hz
        // 1 = 1.024 kHz
        // 2 = 4.096 kHz
        // 3 = 8.192 kHz (Default if frequency byte is out of range);
    void enable32kHz(bool TF); 
		// Turns the 32kHz output pin on (true); or off (false).
    bool oscillatorCheck();;
        // Checks the status of the OSF (Oscillator Stop Flag);.
        // If this returns false, then the clock is probably not
        // giving you the correct time.
        // The OSF is cleared by function setSecond();.		

private:
    byte readControlByte(bool which); 
		// Read selected control byte: (0); reads 0x0e, (1) reads 0x0f
    void writeControlByte(byte control, bool which); 
		// Write the selected control byte. 
		// which == false -> 0x0e, true->0x0f.    
};


#endif // _RTCLIB_H_
