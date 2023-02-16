#include <Arduino.h>

#define MC esp32
#if MC == esp32
#include <Servo.h>
Servo esp32_servo;
#define boardVolt 3.3 // if microcontroller = 5V then max Analog = 1023 else its  const char outputFormat[]

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
