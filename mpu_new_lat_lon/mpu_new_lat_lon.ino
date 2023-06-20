#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <math.h>

Adafruit_MPU6050 mpu;

Servo servo1; 

#define threshold 2
float initial_bearing = 0;
bool test_first = true;
float previous_difference = 0;
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//convert degrees to radians
double dtor(double fdegrees) {
  return(fdegrees * PI / 180);
}

//Convert radians to degrees
double rtod(double fradians) {
  return(fradians * 180.0 / PI);
}
//Calculate bearing from lat1/lon1 to lat2/lon2
//Returns bearing in degrees
double calcBearing(double lat1, double lon1, double lat2, double lon2) {
  lat1 = dtor(lat1);
  lon1 = dtor(lon1);
  lat2 = dtor(lat2);
  lon2 = dtor(lon2);

  //determine angle
  double bearing = atan2(sin(lon2-lon1)*cos(lat2), (cos(lat1)*sin(lat2))-(sin(lat1)*cos(lat2)*cos(lon2-lon1)));
  //convert to degrees
  bearing = rtod(bearing);
  //use mod to turn -90 = 270
  bearing = fmod((bearing + 360.0), 360);
  return (bearing + 0.5);
}

void setup(void) {
  Serial.begin(115200);

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);

  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);

  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
  servo1.attach(7);

  servo1.write(0);

}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float lon1 = 32.34;
  float lat1 = -106.76;

  float xi = a.gyro.x;
  float yi = a.gyro.y;
  float zi = a.gyro.z;

  Serial.println(xi);

  xi = mapFloat(xi, -10, 10, 0, 180);
  while(Serial.available() == 0){
    
  }
  float lon2 = Serial.readStringUntil('\n').toFloat();
  while(Serial.available() == 0){

  }
  float lat2 = Serial.readStringUntil('\n').toFloat();
 // float zf = Serial.readStringUntil('\n').toFloat();

  float bearing = calcBearing(lat1, lon1, lat2, lon2);
  
  float differencex;
  //comment this out if you want to use the below implementation
  differencex = bearing - xi; 

  if (previous_difference == 0){
    previous_difference = differencex;
    servo1.write((int) differencex);
  }
  else if (abs(differencex- previous_difference) >= threshold){
    servo1.write((int) differencex);
    previous_difference = differencex;
  }

  // or use this one

  /* initialize a reference bearing (neutral position before rocket goes off)
  then all bearings calculated after are in reference to this bearing when subtracted off in differencex, and then the difference in angle is written to servo
  if(test_first){
    initial_bearing = bearing;
    test_first = false; 
}
else{
  differencex = initial_bearing - bearing;
}
  if(abs(differencex) >= threshold){
  servo1.write(int(differencex));
}
*/
}