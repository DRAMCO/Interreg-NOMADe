#include "Arduino.h"
#include "BMS.h"

BMS::BMS(){
}

void BMS::init(){
    pinMode(CHECK_BAT,  OUTPUT  );
    pinMode(V_BAT,      INPUT   );
    pinMode(BAT_CHG,    INPUT   );
}

float BMS::getBatVoltage(){
    digitalWrite(CHECK_BAT, HIGH);
    uint16_t val = analogRead(V_BAT);
    float result = val/1023.0*3.3*133/100;
    digitalWrite(CHECK_BAT, LOW);
    return result;
}

bool BMS::batStatus(){
    if(getBatVoltage() < THRESHOLD_MIN_BAT_VOLTAGE)     return 0;
    else                                                return 1;
}

bool BMS::batCharging(){
    if(!digitalRead(BAT_CHG))   {   //digitalWrite(IND_LED, HIGH);  
    return 1; }
    else                        {   //digitalWrite(IND_LED, LOW);   
    return 0; }
}