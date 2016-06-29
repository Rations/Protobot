#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include <PS2X_lib.h>
                            
// Specify Arduino pins for arm servo and PS2 controller connections. 
#define ARM_SERVO_PIN     4
#define PS2_CLOCK_PIN     9
#define PS2_COMMAND_PIN   7
#define PS2_ATTENTION_PIN 8
#define PS2_DATA_PIN      6

// PS2 joystick characteristics.
#define PS2_REFRESH       5 // Controller refresh rate (ms)
#define JOYSTICK_ZERO     128   // Joystick midpoint value
#define JOYSTICK_RANGE    124
#define JOYSTICK_DEADZONE 4     // Joystick deadzone value
boolean vacuumOn = false;      // Allows vacuum to be switched on and off alternatingly. 

// Arm servo characteristics. 0 = full speed in one direction, 90 = zero speed, 180 = full speed in opposite direction.
#define ARM_SPEED_ZERO    90
#define ARM_SPEED_RANGE   90
float ARM_SPEED_SCALE =   ARM_SPEED_RANGE / JOYSTICK_RANGE;

#define BASE_SPEED_ZERO    0     // In rpm
#define BASE_SPEED_RANGE   7.5   // In rpm = 1 rotation takes 8 seconds
float BASE_SPEED_SCALE =   BASE_SPEED_RANGE / JOYSTICK_RANGE;

#define LIFT_SPEED_ZERO    0   
#define LIFT_SPEED_RANGE   255
float LIFT_SPEED_SCALE =   LIFT_SPEED_RANGE / JOYSTICK_RANGE;

// Global variables storing servo speeds. Initialize to zero speed.  
float armSpeed = ARM_SPEED_ZERO;
float baseSpeed = BASE_SPEED_ZERO;
float liftSpeed = LIFT_SPEED_ZERO;

// Declare servo, motor, and PS2 controller objects.
PS2X  ps2;
Servo armServo;
Adafruit_MotorShield AFMStop(0x61);           // Rightmost jumper closed
Adafruit_MotorShield AFMSbot(0x60);           // Default address, no jumpers
Adafruit_DCMotor *liftMotor1 = AFMSbot.getMotor(1);
Adafruit_DCMotor *liftMotor2 = AFMSbot.getMotor(2); 
Adafruit_StepperMotor *baseMotor = AFMSbot.getStepper(400, 2);
Adafruit_DCMotor *vacuum = AFMStop.getMotor(1);
 
void setup() {
  armServo.attach(ARM_SERVO_PIN);

  // Set up PS2 controller; loop until ready.
  byte ps2Status;
  do {
    ps2Status = ps2.config_gamepad(PS2_CLOCK_PIN, PS2_COMMAND_PIN, PS2_ATTENTION_PIN, PS2_DATA_PIN);
  } while (ps2Status == 1);
  delay(100);
}
 
void loop() {
  ps2.read_gamepad();
  
  // Read vertical-axis inputs from right joystick (operates arm).
  float joystickRY = (float)(JOYSTICK_ZERO - ps2.Analog(PSS_RY));
  if (abs(joystickRY) > JOYSTICK_DEADZONE) {
    armSpeed = ARM_SPEED_ZERO + joystickRY * ARM_SPEED_SCALE;
    armServo.write(armSpeed);
  } else {
    armServo.write(ARM_SPEED_ZERO);
  }

  // Read horizontal-axis inputs from left joystick (operates rotating base).
  float joystickLX = (float)(ps2.Analog(PSS_LX) - JOYSTICK_ZERO);
  if (abs(joystickLX) > JOYSTICK_DEADZONE) {
    baseSpeed = BASE_SPEED_ZERO + joystickLX * BASE_SPEED_SCALE;
    baseMotor -> setSpeed(abs(baseSpeed));
    if (baseSpeed > 0) { 
      baseMotor -> step(3, FORWARD, SINGLE);
    } else {
      baseMotor -> step(3, BACKWARD, SINGLE);
    } 
  } else {
      baseMotor -> setSpeed(BASE_SPEED_ZERO);
  }

  // Read vertical-axis inputs from left joystick (operates lift).
  float joystickLY = (float)(JOYSTICK_ZERO - ps2.Analog(PSS_LY));
  if (abs(joystickLY) > JOYSTICK_DEADZONE) {
    liftSpeed = LIFT_SPEED_ZERO + joystickLY * LIFT_SPEED_SCALE;
    liftMotor1 -> setSpeed(abs(liftSpeed));
    liftMotor2 -> setSpeed(abs(liftSpeed));
    if (liftSpeed > 0) { 
      liftMotor1 -> run(FORWARD);
      liftMotor2 -> run(FORWARD);
    } else {
      liftMotor1 -> run(BACKWARD);
      liftMotor2 -> run(BACKWARD);
    } 
  } else {
      liftMotor1 -> run(RELEASE);
      liftMotor2 -> run(RELEASE);
  }

  // Read R2. If R2 depressed, switch vacuum on/off as necessary.
  if (ps2.Button(PSB_R2)) {
    if (!vacuumOn) {
      vacuum -> run(FORWARD);                 
      vacuum -> setSpeed(255);
    } else {
      vacuum -> run(RELEASE);
    }
    vacuumOn = !vacuumOn;
  }
  delay(PS2_REFRESH);
}
