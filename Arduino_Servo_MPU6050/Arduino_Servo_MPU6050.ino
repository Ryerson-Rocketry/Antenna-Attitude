#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Servo.h>
#include <math.h>

Adafruit_MPU6050 mpu;
int x = 0;
int y = 0;
int z = 0;

Servo servo1; 

int value  = 0;

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
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

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

  x = a.gyro.x;
  y = a.gyro.y;
  z = a.gyro.z;

  Serial.println(x);

  int xi = 33;//Serial.readUntil('');
  int yi = -33;//Serial.readUntil('');
  int zi = 79;//Serial.readUntil('');
  
  int xf, yf, zf;

  //xf = Serial.readStringUntil('\n').toInt();
  //yf = Serial.readStringUntil('\n').toInt();
  xf=56;
  yf=56;
  zf =79;
  float magnitude = sqrt(pow(xf - xi, 2) + pow(yf-yi, 2) + pow(zf - zi, 2));
  float anglex = acos((xf-xi)/magnitude) * 180/M_PI;
  float angley = acos((yf-yi)/magnitude) * 180/M_PI;
  float anglez = acos((zf-zi)/magnitude) * 180/M_PI;
  int value_x=map(x,-10,10,75,360);
  
  int differencex = value_x-anglex; 

  value = map(differencex,  0, 90, 0, 180);
  Serial.println(value);
  servo1.write(value);
  
}