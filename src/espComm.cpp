#include <Arduino.h>

int64_t timer_readings[100];

void setup(){
  Serial.begin(115200);

  Serial.println("Lendo o timer do esp32 100x e vendo o overflow");
  for (int i = 0; i < 100; i++)
  {
    timer_readings[i] = esp_timer_get_time();
  }

  for (int i = 0; i < 100; i++)
  {
    Serial.print(timer_readings[i]);
    Serial.print(",");
  }

}
void loop(){

}