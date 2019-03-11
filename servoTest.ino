
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

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);
// you can also call it with a different address and I2C interface
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(&Wire, 0x40);

// Depending on your servo make, the pulse width min and max may vary, you
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
const uint16_t SERVO_MIN = 50;  // this is the 'minimum' pulse length count (out of 4096)
const uint16_t SERVO_MAX = 650; // this is the 'maximum' pulse length count (out of 4096)
const uint16_t ANGLE_MIN = 0;   // this is the 'minimum' servo angle
const uint16_t ANGLE_MAX = 180; // this is the 'maximum' servo angle


#define REFRESH_INTERVAL 200000 // minumim time to refresh servos in microseconds
#define WAIT_INTERVAL     100 // time to wait servos in milliseconds

const uint16_t ANGLE_INC = (SERVO_MAX - SERVO_MIN) / ANGLE_MAX;

const float PWM_FREQUENCY = 60.0;

// our servo # counter
uint8_t servonum = 0;
boolean invert = false;
const uint8_t SERVO_NUM_MAX = 1;

void setup() {
  Serial.begin(115200);
  Serial.print(SERVO_NUM_MAX);
  Serial.println(" channel Servo test!");

  pwm.begin();
  pwm.setPWMFreq(PWM_FREQUENCY);  // Analog servos run at ~60 Hz updates

  delay(10);
}

void loop() {
  for (uint16_t angle = ANGLE_MIN; angle <= ANGLE_MAX; angle++) {
    uint16_t angleReal = angle;
    if (invert) {
      angleReal = ANGLE_MAX - angle;
    }
    Serial.println(angleReal);
    setServo(servonum, angleReal);
    delayMicroseconds(REFRESH_INTERVAL);
  }
  Serial.print(invert);Serial.println("WAIT");
  delay(WAIT_INTERVAL);
  invert = !invert;

  servonum ++;
  if (servonum > SERVO_NUM_MAX) servonum = 0;
}

void setServo(uint8_t servonum, uint16_t pulselen)
{
  {
    if(pulselen < ANGLE_MIN) pulselen = ANGLE_MIN;
    if(pulselen > ANGLE_MAX) pulselen = ANGLE_MAX;
    pulselen = map(pulselen, ANGLE_MIN, ANGLE_MAX, SERVO_MIN, SERVO_MAX);
  }
  pwm.setPWM(servonum, 0, pulselen);
}
