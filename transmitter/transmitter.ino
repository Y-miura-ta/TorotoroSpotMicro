#include <ArduinoJson.h>

int sensorValue[6];
int analogPin[] = {
  A0, A1, A2, A3, A4, A5
};

int neutral_pos[] = {
  536, 463, 524, 517, 846, 510
};
int sign[] = {
  1, 1, 1, 1, 1, 1
};
float pos2deg = 90.0/350.0;
float pos2rad = 1.5707963/350;

DynamicJsonDocument doc(1024);

void setup() {
  Serial.begin(115200);
  // シリアルの初期化待ち
  while (!Serial) { }
}

void loop() {
  int i;
  for(i=0; i<6; i++){
    sensorValue[i] = analogRead(analogPin[i]);
    doc["joint"][i] = sign[i]*(sensorValue[i] - neutral_pos[i])*pos2deg;
  }
  serializeJson(doc, Serial);
  Serial.print("\n");
  
  delay(1);
}
