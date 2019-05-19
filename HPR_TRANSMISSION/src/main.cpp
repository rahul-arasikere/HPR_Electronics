#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_GPS.h>
#include <RH_RF95.h>
#include <SPI.h>

#define BMP_CS 10
#define BMP_MOSI 11
#define BMP_MISO 12
#define BMP_SCK 13
#define GPSSerial Serial1
#define GPSECHO false

Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO, BMP_SCK);
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
sensors_event_t accel, mag, gyro, temp;
Adafruit_GPS GPS(&GPSSerial);
RH_RF95 rf95(8, 3);
float packetNum = 0;

void setup()
{
  Serial.begin(9600);
  if (!bmp.begin())
  {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
  }
  if (!lsm.begin())
  {
    Serial.println("Could not find a valid LSM9DS1 sensor, check wiring!");
  }
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  //GPS.sendCommand(PGCMD_ANTENNA);

  //delay(1000);

  // Ask for firmware version
  //GPSSerial.println(PMTK_Q_RELEASE);
  if (!rf95.init())
  {
    Serial.println("Packet Radio Library initialization failed!");
  }
  rf95.setFrequency(915.0);
  Serial.println("Setup done!");
}

void loop()
{
  delay(1000);
  float values[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  values[0] = bmp.readAltitude();
  values[1] = bmp.readPressure();
  values[2] = bmp.readTemperature();
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  values[3] = accel.acceleration.x;
  values[4] = accel.acceleration.y;
  values[5] = accel.acceleration.z;
  values[6] = gyro.gyro.heading;
  values[7] = gyro.gyro.pitch;
  values[8] = gyro.gyro.roll;
  values[9] = GPS.latitude;
  values[10] = GPS.longitude;
  values[11] = GPS.speed;
  values[12] = GPS.angle;
  values[13] = GPS.altitude;
  values[14] = ++packetNum;
  rf95.send((uint8_t *)values, sizeof(values) * 4);
  Serial.println(packetNum);
}