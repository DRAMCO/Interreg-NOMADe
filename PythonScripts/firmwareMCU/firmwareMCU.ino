#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;


void setup(void) 
{
  Serial.begin(115200);
  while (!Serial) {
      // will pause Zero, Leonardo, etc until serial console opens
      delay(1);
  }

  uint32_t currentFrequency;
    
  //Serial.println("Hello!");
  
  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  ina219.begin();
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  ina219.setCalibration_16V_400mA();

  //Serial.println("Measuring voltage and current with INA219 ...");
}

void loop(void) 
{
  uint32_t timeNow = (uint32_t)(millis());
  uint16_t busvoltage = 0;
  uint16_t current_mA = 0;

  busvoltage = (uint16_t)(ina219.getBusVoltage_V()*100);
  current_mA = (uint16_t)(abs(ina219.getCurrent_mA())*10);
/*
  Serial.println(millis());
  Serial.print("Voltage:       "); Serial.print(busvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.println("");
*/
  
  uint8_t buf [] = {0x02, (uint8_t)timeNow, (uint8_t)(timeNow >> 8), (uint8_t)(timeNow >> 16), (uint8_t)(timeNow >> 24), (uint8_t)busvoltage, (uint8_t)(busvoltage >> 8), (uint8_t)current_mA, (uint8_t)(current_mA >> 8)};

  Serial.write(buf, sizeof(buf));
  Serial.write(calculateCS(buf, sizeof(buf)));

  delay(2000);
}


uint8_t calculateCS(uint8_t * data, uint8_t len){
    uint8_t checksum = *(data);
    for(uint8_t i = 1; i < len; i++){
        checksum = checksum ^ *(data + i);
    }
    return checksum;
}
