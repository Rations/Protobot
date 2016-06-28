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

#define GM_PIN 3            // Currently a placeholder
#define SM_PIN 2            // Currently a placeholder
#define VM_PIN 1            // Currently a placeholder

// PS2 controller characteristics.
#define JS_ZERO       128   // Joystick midpoint value
#define JS_RANGE      124
#define JS_DEAD       4     // Joystick deadzone value
#define R2_DEAD       5

// CRS speed data. 0 = full speed in one direction, 90 = zero speed, 180 = full speed in opposite direction.
#define R_SPD_ZERO    90
#define R_SPD_RANGE   90
float R_SPD_SCALE =   R_SPD_RANGE / JS_RANGE;

#define T_SPD_ZERO    90    // Currently a placeholder
#define T_SPD_RANGE   255   // Currently a placeholder
float T_SPD_SCALE =   T_SPD_RANGE / JS_RANGE;

#define Z_SPD_ZERO    90    // Currently a placeholder
#define Z_SPD_RANGE   90    // Currently a placeholder
float Z_SPD_SCALE =   Z_SPD_RANGE / JS_RANGE;

// Global variables storing servo speeds. Initialize to zero speed.  
float r_spd = R_SPD_ZERO;
float t_spd = T_SPD_ZERO;
float z_spd = Z_SPD_ZERO;

// Declare PS2 controller and servo objects
PS2X  Ps2x;
Servo arm_servo;
 
void setup() {
  arm_servo.attach(ARM_SERVO_PIN);
  pinMode(GM_PIN, OUTPUT);
  pinMode(SM_PIN, OUTPUT);
  pinMode(VM_PIN, OUTPUT);

  // Set up PS2 controller; loop until ready.
  byte ps2_status;
  do {
    ps2_status = Ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_ATT_PIN, PS2_DAT_PIN);
  } while (ps2_status == 1);

  // Park robot when ready.                            
  move_robot(R_SPD_ZERO, T_SPD_ZERO, Z_SPD_ZERO);
  delay(100);
}
 
void loop() {
  // Indidates whether robot should move. 
  boolean move = false;
  Ps2x.read_gamepad();
  
  // Read right joystick; adjust value with respect to joystick zero point.
  float rh_js_y = (float)(Ps2x.Analog(PSS_RY) - JS_ZERO);
  if (abs(rh_js_y) > JS_DEAD) {
    r_spd = R_SPD_ZERO + rh_js_y * R_SPD_SCALE;
    move = true;
  }

  // Read left joystick; adjust value with respect to joystick zero point.
  float lh_js_x = (float)(Ps2x.Analog(PSS_LX) - JS_ZERO);
  if (abs(lh_js_x) > JS_DEAD) {
    t_spd = T_SPD_ZERO + lh_js_x * T_SPD_SCALE;
    move = true;
  }

  // Read left joystick; adjust value with respect to joystick zero point.
  float lh_js_y = (float)(Ps2x.Analog(PSS_LY) - JS_ZERO);
  if (abs(lh_js_y) > JS_DEAD) {
    z_spd = Z_SPD_ZERO + lh_js_y * Z_SPD_SCALE;
    move = true;
  }

  // Read R2. If R2 pressed, turn vacuum on/off as necessary.
  if (Ps2x.Button(PSB_R2)) {
    if (analogRead(VM_PIN) > R2_DEAD) {
      analogWrite(VM_PIN, 0);
    } else {
      analogWrite(VM_PIN, 255);
    }
  }
  if (move) {
    move_robot(r_spd, t_spd, z_spd);
    move = false;
  }
  delay(10);
}
 
// Position robot.
void move_robot(float r_spd, float t_spd, float z_spd)
{
  arm_servo.write(r_spd);
  analogWrite(GM_PIN, z_spd);
  analogWrite(SM_PIN, t_spd);
}
