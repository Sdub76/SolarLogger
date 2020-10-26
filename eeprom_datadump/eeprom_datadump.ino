// Dump EEPROM to console CSV
#include <Wire.h>                      // Arduino I2C library
#include <EEPROM.h>                    // Arduino Internal EEPROM library
#include <SparkFun_External_EEPROM.h>  // https://github.com/sparkfun/SparkFun_External_EEPROM_Arduino_Library


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
#define EE_SIZE         32768 // Bytes
#define EE_PAGELENGTH   64    // Bits
#define EE_I2C_ADDRESS  0x58
ExternalEEPROM ext_eep;

//============ EXTERNAL EEPROM MAP ==========================
#define extEE_RDATA_START  0
// 00000 - extEE_addr(64b) - Reading 1
// 00008 - extEE_addr(64b) - Reading 2
// 00016 - extEE_addr(64b) - Reading 3
// 00024 - extEE_addr(64b) - Reading 4
// ---
// 32000 - extEE_addr(64b) - Reading 4000


long IRC = 0;
int EE_addr = 0;
struct EE_reading {
  uint32_t time;
  float voltage;
};
EE_reading SolarData;


void setup() {

  Serial.begin(9600);

  // Print InternalReferenceConstant compensation value
  EEPROM.get(EE_IRC_ADDR,IRC);
  Serial.print("IRC compensation = ");
  Serial.print((IRC/(1.1*1024.0)/1000.0*100)-100);
  Serial.println("%");
  
  EEPROM.get(EE_RINDEX,EE_addr);
  Serial.print("EE Last Index = ");
  Serial.println(EE_addr);

  if (EE_addr == 0) {EE_addr = extEE_RDATA_START;} // Protect against empty EEPROM

  float ftemp;
  long ltemp;
  EEPROM.get(EE_BATT_MIN,ftemp);
  EEPROM.get(EE_BATT_MIN_TIME,ltemp);
  Serial.print("Battery Min: ");
  Serial.print(ftemp);
  Serial.print(" V (");
  Serial.print(ltemp);
  Serial.println(")");

  EEPROM.get(EE_TAMB_MIN,ftemp);
  EEPROM.get(EE_TAMB_MIN_TIME,ltemp);
  Serial.print("Tamb Min: ");
  Serial.print(ftemp);
  Serial.print(" V (");
  Serial.print(ltemp);
  Serial.println(")");

  EEPROM.get(EE_TAMB_MAX,ftemp);
  EEPROM.get(EE_TAMB_MAX_TIME,ltemp);
  Serial.print("Tamb Max: ");
  Serial.print(ftemp);
  Serial.print(" V (");
  Serial.print(ltemp);
  Serial.println(")");

  // External EEPROM (I2C)
  if (ext_eep.begin() == false) {
    Serial.print(F("No memory detected. Freezing."));
    while (1);
  }
  ext_eep.setMemorySize(EE_SIZE); //In bytes. 512kbit = 64kbyte
  ext_eep.setPageSize(EE_PAGELENGTH); //In bytes. Has 128 byte page size.
  
  // Iterate from stored index to end of EEPROM
  int index = EE_addr; 
  while ((index + sizeof(SolarData) -1) <= (ext_eep.length())) {

    ext_eep.get(index,SolarData);
    Serial.print(index);
    Serial.print(",");
    Serial.print(SolarData.time);
    Serial.print(",");
    Serial.println(SolarData.voltage);
    
    index += sizeof(SolarData);
    
  }

  // Iterate from start of EEPROM to stored index
  index = extEE_RDATA_START;
  while (index < EE_addr) {

    ext_eep.get(index,SolarData);
    Serial.print(index);
    Serial.print(",");
    Serial.print(SolarData.time);
    Serial.print(",");
    Serial.println(SolarData.voltage);

    index += sizeof(SolarData);
    
  }


}

void loop() {}
