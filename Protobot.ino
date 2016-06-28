// To address: SM max speed
// VM speed and direction
// step(, , DOUBLE or SINGLE)?

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>
#include <PS2X_lib.h>
                            
// Specify pins for servos, motors, PS2 controller connections. 
#define PS2_CLK_PIN   9     // Clock
#define PS2_CMD_PIN   7     // Command
#define PS2_ATT_PIN   8     // Attention
#define PS2_DAT_PIN   6     // Data

#define ARM_SERVO_PIN 4

#define GM1_PIN 3            // Currently a placeholder
#define GM2_PIN 10
#define SM_PIN 2            // Currently a placeholder
#define VM_PIN 1            // Currently a placeholder

// PS2 controller characteristics.
#define JS_ZERO       128   // Joystick midpoint value
#define JS_RANGE      124
#define JS_DEAD       4     // Joystick deadzone value
boolean VM_dir = false;

// CRS speed data. 0 = full speed in one direction, 90 = zero speed, 180 = full speed in opposite direction.
#define R_SPD_ZERO    90
#define R_SPD_RANGE   90
float R_SPD_SCALE =   R_SPD_RANGE / JS_RANGE;

#define T_SPD_ZERO    0     // In rpm
#define T_SPD_RANGE   12    // In rpm
float T_SPD_SCALE =   T_SPD_RANGE / JS_RANGE;

#define Z_SPD_ZERO    0   
#define Z_SPD_RANGE   255
float Z_SPD_SCALE =   Z_SPD_RANGE / JS_RANGE;

// Global variables storing servo speeds. Initialize to zero speed.  
float r_spd = R_SPD_ZERO;
float t_spd = T_SPD_ZERO;
float z_spd = Z_SPD_ZERO;

// Declare PS2 controller and servo objects
PS2X  Ps2x;
Servo arm_servo;
Adafruit_MotorShield AFMStop(0x61); // Rightmost jumper closed
Adafruit_MotorShield AFMSbot(0x60); // Default address, no jumpers
Adafruit_DCMotor *GM1 = AFMSbot.getMotor(1);
Adafruit_DCMotor *GM2 = AFMSbot.getMotor(2); 
Adafruit_StepperMotor *stepperMotor_1 = AFMSbot.getStepper(400, 2);
Adafruit_DCMotor *VM = AFMStop.getMotor(1);
 
void setup() {
  arm_servo.attach(ARM_SERVO_PIN);

  // Set up PS2 controller; loop until ready.
  byte ps2_status;
  do {
    ps2_status = Ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_ATT_PIN, PS2_DAT_PIN);
  } while (ps2_status == 1);
  delay(100);
}
 
void loop() {
  Ps2x.read_gamepad();
  
  // Read right joystick; adjust value with respect to joystick zero point. 
  // Corresponds to arm servo.
  float rh_js_y = (float)(JS_ZERO - Ps2x.Analog(PSS_RY));
  if (abs(rh_js_y) > JS_DEAD) {
    r_spd = R_SPD_ZERO + rh_js_y * R_SPD_SCALE;
    arm_servo.write(r_spd);
  }

  // Read left joystick; adjust value with respect to joystick zero point.
  // Corresponds to stepper motor (SM).
  float lh_js_x = (float)(Ps2x.Analog(PSS_LX) - JS_ZERO);
  if (abs(lh_js_x) > JS_DEAD) {
    t_spd = T_SPD_ZERO + lh_js_x * T_SPD_SCALE;
  }

  // Read left joystick; adjust value with respect to joystick zero point.
  // Corresponds to gear motors (GM).
  float lh_js_y = (float)(JS_ZERO - Ps2x.Analog(PSS_LY));
  if (abs(lh_js_y) > JS_DEAD) {
    z_spd = Z_SPD_ZERO + lh_js_y * Z_SPD_SCALE;
    GM1->setSpeed(abs(z_spd));
    GM2->setSpeed(abs(z_spd));
    if (z_spd > 0) { 
      GM1->run(FORWARD);
      GM2->run(FORWARD);
    } else if (z_spd < 0) {
      GM1->run(BACKWARD);
      GM2->run(BACKWARD);
    } 
  } else {
      GM1->run(RELEASE);
      GM2->run(RELEASE);
  }

  // Read R2. If R2 pressed, turn vacuum on/off as necessary.
  if (Ps2x.Button(PSB_R2)) {
    if (!VM_dir) {
      VM->run(FORWARD);                 
      VM->setSpeed(0);    // Justin's speed
    } else {
      VM->run(RELEASE);
    }
    VM_dir = !VM_dir;
  }
  delay(5);
}
