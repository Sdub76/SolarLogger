// Dump EEPROM to console CSV

//============ EEPROM MAP ==========================
// 0000 - int(32b) - InternalReferenceConstant
#define EE_IRC_ADDR     0
// 0004 - int(16b) - Reading Index
#define EE_RINDEX       4
// 0008 - EE_addr(64b) - Reading 1
#define EE_RDATA_START  6 // change to 8 
// 0016 - EE_addr(64b) - Reading 2
// 0024 - EE_addr(64b) - Reading 3
// 0032 - EE_addr(64b) - Reading 4
// ---
// 1016 - EE_addr(64b) - Reading 127

long IRC = 0;
int EE_addr = 0;
struct EE_reading {
  uint32_t time;
  float voltage;
};
EE_reading SolarData;


#include <EEPROM.h>

void setup() {

  Serial.begin(9600);
  
  // Print InternalReferenceConstant compensation value
  EEPROM.get(EE_IRC_ADDR,IRC);
  Serial.print("IRC compensation = ");
  Serial.print(IRC/(1.1*1024)/100.0);
  Serial.println("%");
  
  EEPROM.get(EE_RINDEX,EE_addr);
  
  if (EE_addr == 0) {EE_addr = EE_RDATA_START;} // Protect against empty EEPROM
  
  // Iterate from stored index to end of EEPROM
  int index = EE_addr; // Back up one 
  while ((index + sizeof(SolarData)) <= (EEPROM.length()) {

    EEPROM.get(index,SolarData);
    Serial.print(index);
    Serial.print(",");
    Serial.print(SolarData.time);
    Serial.print(",");
    Serial.println(SolarData.voltage);
    
    index += sizeof(SolarData);
    
  }

  // Iterate from start of EEPROM to stored index
  index = EE_RDATA_START;
  while (index < EE_addr) {

    EEPROM.get(index,SolarData);
    Serial.print(index);
    Serial.print(",");
    Serial.print(SolarData.time);
    Serial.print(",");
    Serial.println(SolarData.voltage);

    index += sizeof(SolarData);
    
  }


}

void loop() {}
