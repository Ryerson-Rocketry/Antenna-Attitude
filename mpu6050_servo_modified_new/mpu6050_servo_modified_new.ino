#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <math.h>

Adafruit_MPU6050 mpu;

Servo servo1; 

#define threshold 2
float previous_difference = 0;

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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

  float xi = a.gyro.x;
  float yi = a.gyro.y;
  float zi = a.gyro.z;

  Serial.println(xi);

  xi = mapFloat(xi, -10, 10, 0, 180);
  float xf = 0;//Serial.readStringUntil('\n').toFloat();
  float yf = 0;//Serial.readStringUntil('\n').toFloat();
  float zf = 0;//Serial.readStringUntil('\n').toFloat();

  float magnitude = sqrt(pow(xf - xi, 2) + pow(yf - yi, 2) + pow(zf - zi, 2));
  float anglex = acos((xf - xi)/magnitude) * 180/M_PI;
  float angley = acos((yf - yi)/magnitude) * 180/M_PI;
  float anglez = acos((zf - zi)/magnitude) * 180/M_PI;

  float differencex = anglex - xi; 

  if (previous_difference == 0){
    previous_difference = differencex;
    servo1.write((int) differencex);
  }
  else if (abs(differencex- previous_difference) >= threshold){
    servo1.write((int) differencex);
    previous_difference = differencex;
  }
}