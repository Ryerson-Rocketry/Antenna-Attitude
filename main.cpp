#include <Arduino.h>

#define MC esp32
#if MC == esp32
#include <Servo.h>

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
Servo esp32_servo;
#define boardVolt 3.3 // if microcontroller = 5V then max Analog = 1023 else its  const char outputFormat[]

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
}

mpu.initialize();
devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
  

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

            int posG[] = {0, 0, 1000};
            int posR[] = {(- 90 + random(180), (-180 + random(360)), (3000))};
            
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);

            int resultant[] = {posR[0] - posG[0], posR[1] - posG[1], posR[2] - posG[2]};
    
            float resultant_mag = sqrt(pow(resultant[0],2) + pow(resultant[1],2) + pow(resultant[2],2));
            float angle_needed[] = {acos(resultant[0]/resultant_mag) * 180.0 / PI, acos(resultant[1]/resultant_mag) * 180.0 / PI, acos(resultant[2]/resultant_mag) * 180.0 / PI};
            
            if (angle_needed[0] > ypr[0]){
              Serial.print("Turn positive in \t");
            } else {
              Serial.print("Turn negative in x \t");
            }

            if (angle_needed[1] > ypr[1]){
              Serial.print("Turn positive in y \t");
            } else {
              Serial.print("Turn negative in y \t");
            }
    
            if (angle_needed[2] > ypr[2]){
              Serial.print("Turn positive in z\t");
            } else {
              Serial.print("Turn negative in z\t");
            }
        #endif
    }
#else
#include <Servo.h>
#define boardVolt 5.0 // if microcontroller = 5V then max Analog = 1023 else its  const char outputFormat[]
Servo myservo; // create servo object to control a servo

#endif
#include <time.h>
// ========================================
//        prototypes:
//=========================================
uint32_t SERVO(int threshold, int inputControl_analog, int *rotation_servo_deg, int *rotationServo_checker, uint32_t maxTime);
// ========================================
//        CONSTANTS:
//=========================================


#define ServoPin 4
#define servo1_minUs 1000
#define servo1_maxUs 2000
#define servoWriteDelay 20               // time to wait until servo gets to desired position
#define rotationInputControl_threshold 6 // threshold for  input to write to servo so that back&forth action doesnt occur at idle input

const int maxAnalog = (boardVolt / 5) * 1023;
const int analogZero = maxAnalog / 2;

const char outputFormat[] = R"""(
  ============
  timer: %d seconds
  ===========
  FOR SERVO1:
  ===========
  servo1 raw input= %d
  Servo rotation = %d degrees
  Servo rotation checker = %d degrees
  )""";

// ========================================
//        GLOBALS:
//=========================================

int inputControl_analog = 0;
int rotationServo_checker = 0;
int rotation_servo_deg = 0;
int counter = 0;
unsigned int timer = 0;

char string[330] = {0};
// ========================================
//        SETUP:
//=========================================
void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);

  Serial.println("\nSerial Communication Started!");
  #if MC==esp32
    esp32_servo.attach(ServoPin); // attaches the servo on pin 9 to the servo object

  #else
  myservo.attach(ServoPin); // attaches the servo on pin 9 to the servo object
  #endif
}

// ========================================
//        MAIN LOOP:
//=========================================


void loop()
{

  timer = millis() / 1000;

  switch (SERVO(rotationInputControl_threshold, analogRead(2), &rotation_servo_deg, &rotationServo_checker, 1000))
  {
  case 0:

    Serial.println("servo write");
    break;

  case 1:

    Serial.println("error with Servo; exceeds time limit given.");
    break;
  }

  sprintf(string, outputFormat, timer, inputControl_analog, rotation_servo_deg, rotationServo_checker);
  Serial.print(string);
  delay(1000);
}

// ====================================
//         FUNCTIONS:
//=====================================

/*
@breif: function that writes to servo
@param threshold: the abs threshold value preset to calibrate servo value a bit
@param  inputControl_analog: Analog value used to control/know servo position
@param rotation_servo_deg: (return/outputted variable) servo position in degrees
@param rotationServo_checker: (outputted variable) old servo angle used with current rotation angle to see if writing should occur(if theres an actual value change)
@param maxTime: maximum wait time for the function to run until function termination
@return 0-on servo write success, 1-on failure(took too long),2-on no servo write
*/
uint32_t SERVO(int threshold, int inputControl_analog, int *rotation_servo_deg, int *rotationServo_checker, uint32_t maxTime)
{
  uint32_t start = millis();

  while (millis() - start <= maxTime) // added a limit to how long we'll wait before moving on
  {

    *rotation_servo_deg = map(inputControl_analog, 0, maxAnalog, 0, 180); // scale it to use it with the servo (value between 0 and 180)

    if (abs(*rotation_servo_deg - *rotationServo_checker) >= threshold) // only writes if last value within threshold != new value:
    {

      esp32_servo.write(*rotation_servo_deg);

      delay(servoWriteDelay);
      *rotationServo_checker = *(rotation_servo_deg);

      return 0;
    }
    else

    {
      goto servoNotWriteBreak;
    }
  }
  return 1;
servoNotWriteBreak: // break initialized when servo doesnt write
  return 2;
}
