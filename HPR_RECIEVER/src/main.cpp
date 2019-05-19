#include <Arduino.h>
#include <RH_RF95.h>
#include <SPI.h>

RH_RF95 rf95(8, 3);

void setup() {
  if(!rf95.init()){
    Serial.println("Radio not initialized!");
  }
  rf95.setFrequency(915.0);
}

void loop() {
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t length = sizeof(buf);
  if(rf95.recv(buf, &length)){
    float *values = (float *)buf;
    Serial.println("Packet Number : " + String(values[14]));
    Serial.println("Altitude : " + String(values[0]));
    Serial.println("Pressure : " + String(values[1]));
    Serial.println("Temperature : " + String(values[2]));
    Serial.println("Acceleration(X) : " + String(values[3]));
    Serial.println("Acceleration(Y) : " + String(values[4]));
    Serial.println("Acceleration(Z) : " + String(values[5]));
    Serial.println("Gyro Heading : " + String(values[6]));
    Serial.println("Gyro Pitch : " +  String(values[7]));
    Serial.println("Gyro Roll : " + String(values[8]));
    Serial.println("GPS Latitude : " + String(values[9]));
    Serial.println("GPS Longitude : " + String(values[10]));
    Serial.println("GPS Speed : " + String(values[11]));
    Serial.println("GPS Angle : " + String(values[12]));
    Serial.println("GPS Altitude : " + String(values[13]));
  } 
}