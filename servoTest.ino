
/***************************************************
  This is an example for our Adafruit 16-channel PWM & Servo driver
  Servo test - this will drive 8 servos, one after the other on the
  first 8 pins of the PCA9685
  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/815

  These drivers use I2C to communicate, 2 pins are required to
  interface.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!
  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

// open busybox tty with
// busybox microcom -s 115200 /dev/ttyUSB0

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "version.hpp"

const uint16_t POTI_MIN      = 0;   // this is the 'minimum' poti value
const uint16_t POTI_MAX      = 680; // this is the 'maximum' poti value (actually 10bit but on the current one only about 700)
const uint16_t SERVO_MIN     = 100; // this is the 'minimum' pulse length count (out of 4096)
const uint16_t SERVO_MAX     = 560; // this is the 'maximum' pulse length count (out of 4096)
const uint16_t ANGLE_MIN     = 0;   // this is the 'minimum' servo angle
const uint16_t ANGLE_MAX     = 180; // this is the 'maximum' servo angle
const uint32_t WAIT_INTERVAL = 1000; // time to wait servos in milliseconds
const float    PWM_FREQUENCY = 60;
const uint8_t  SERVO_NUM_MAX = 1;

uint8_t  servonum = 0;
uint16_t poti     = 0;
uint16_t mapPoti  = 0;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(115200);
  printVersion();
  Serial.print(SERVO_NUM_MAX);
  Serial.println(" channel Servo test!");

  pwm.begin();
  pwm.setPWMFreq(PWM_FREQUENCY);  // Analog servos run at ~60 Hz updates

  delay(10);
}

void loop() {
  poti = analogRead(A0);
  if (poti <= POTI_MAX) {
    mapPoti = map(poti, POTI_MIN, POTI_MAX, SERVO_MIN, SERVO_MAX);
    Serial.print("poti: ");Serial.print(poti);Serial.print(";");Serial.println(mapPoti);
    pwm.setPWM(servonum, 0, mapPoti);
  } else {
    setServo(servonum, ANGLE_MIN);
    delay(WAIT_INTERVAL);
    setServo(servonum, ANGLE_MAX);
    delay(WAIT_INTERVAL);
  }
}

void setServo(uint8_t servonum, uint16_t pulselen)
{
  {
    if(pulselen < ANGLE_MIN) pulselen = ANGLE_MIN;
    if(pulselen > ANGLE_MAX) pulselen = ANGLE_MAX;
    pulselen = map(pulselen, ANGLE_MIN, ANGLE_MAX, SERVO_MIN, SERVO_MAX);
  }
  Serial.print("setServo: ");Serial.println(pulselen);
  pwm.setPWM(servonum, 0, pulselen);
}

void printVersion() {
  Serial.print(BUILD_PROGRAM);
  Serial.print(": ");
  Serial.print(BUILD_DATE);
  Serial.print(" - ");
  Serial.println(BUILD_VERSION);
  Serial.flush();
}
