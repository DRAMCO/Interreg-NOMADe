#include "Arduino.h"
#include "LED.h"

LED::LED(Variables *counters){
    this->counters = counters;
}

void LED::blink(uint8_t num, uint16_t wait_time){
    for(int i = 0; i < num; i++){
        on();
        delay(wait_time);
        off();
        delay(wait_time);
    }
}


void LED::blink(uint8_t num, uint16_t on_time, uint16_t off_time){
  if(!digitalRead(IND_LED)){
    off_counter++;
    if(off_counter == off_time/1){
        on();
        off_counter = 0;
    }
  }
  else{
    on_counter++;
    if(on_counter == on_time/1){
        off();
        on_counter = 0;
    }
  }
}

void LED::blink_Running(){
  counters->mpu_read_counter++;
  if(counters->mpu_read_counter%50 == 0){
    digitalWrite(IND_LED, !digitalRead(IND_LED));
    counters->mpu_read_counter = 0;
  }

}

void LED::on(void){
  digitalWrite(IND_LED, HIGH);
}

void LED::off(void){
  digitalWrite(IND_LED, LOW);
}