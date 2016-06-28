#include <Servo.h>
#include <PS2X_lib.h>

int dummy;                  // Defining this dummy variable to work around a bug in the IDE (1.0.3) pre-processor that messes up #ifdefs
//#define DEBUG             // Uncomment to turn on debugging output
                            
// Specify Arduino pins for servos and PS2 controller connections. 
#define ARM_SERVO_PIN 4     // Elbow Servo HS-755HB
#define PS2_CLK_PIN 9       // Clock
#define PS2_CMD_PIN 7       // Command
#define PS2_ATT_PIN 8       // Attention
#define PS2_DAT_PIN 6       // Data

// Practical navigation limit.
#define R_MIN 100.0         // mm

// PS2 controller characteristics
#define JS_ZERO 128     // Numeric value for joystick midpoint
#define JS_MAX 256
#define JS_DEAD 4       // Ignore movement this close to the center position
#define JS_IK_SCALE 50.0    // Divisor for scaling JS output for IK control
#define Z_STEP 2.0     // Change in Z axis (mm) per button press
float RANGE_JS = JS_MAX - JS_DEAD;
 
// IK function return values
#define IK_SUCCESS  0
#define IK_ERROR    1       // Desired position not possible

// Boundary positions.
#define MAX_R   100.0
#define MIN_R   0.0
float RANGE_R = MAX_R - MIN_R;

// Default positions.
#define READY_R 45.0
#define READY_T 0
#define READY_Z 45.0

// Global variables storing position. Initialize to default positions. 
float R = READY_R;  // Radial distance (mm) from base.
float T = READY_T;  // Angle.
float Z = READY_Z;  // Height (mm) from base plane.

// Speed info. For CRS, 0 = full speed in one direction, 90 = zero speed, 180 = full speed in opposite direction.
#define R_SPD_ZERO  90 
#define R_SPD_RANGE 90
float R_SPD_SCALE = R_SPD_RANGE / JS_RANGE;

// Declare PS2 controller and servo objects
PS2X  Ps2x;
Servo arm_servo;
 
void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif
  arm_servo.attach(ARM_SERVO_PIN);
  // Set up PS2 controller; loop until ready.
  byte ps2_status;
  do {
    ps2_status = Ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_ATT_PIN, PS2_DAT_PIN);
#ifdef DEBUG
  if (ps2_status == 1) Serial.println("No controller found. Re-trying ...");
#endif
  } while (ps2_status == 1);
#ifdef DEBUG
  switch (ps2_status) {
    case 0:
      Serial.println("Found Controller, configured successfully.");
      break;
    case 2:
      Serial.println("Controller found but not accepting commands.");
      break;
    case 3:
      Serial.println("Controller refusing to enter 'Pressures' mode, may not support it. ");      
      break;
  }
#endif
  // Park robot when ready.                            
  set_robot(READY_R, READY_T, READY_Z);
#ifdef DEBUG
  Serial.println("Started.");
#endif
  delay(500);
}
 
void loop()
{
  // Store desired position in temporary variables until confirmed by set_robot() logic.
  float r_spd_temp = R;
  float t_temp = T;
  float z_temp = Z;
  
  // Indidates whether input can move arm. 
  boolean move_arm = false;

  Ps2x.read_gamepad();
  // Read right joystick; adjust value with respect to joystick zero point.
  float RH_JS_Y = (float)(JS_ZERO - Ps2x.Analog(PSS_RY));
  // r position (mm); must be > R_MIN.
  if (abs(RH_JS_Y) > JS_DEAD) {
    r_spd_temp = R_SPD_ZERO + RH_JS_Y * R_SPD_SCALE;
    move_arm = true;
  }
    
  // z position (mm).
  if (Ps2x.Button(PSB_L1) || Ps2x.Button(PSB_R1)) {
    if (Ps2x.Button(PSB_L1)) {
      z_temp -= Z_STEP;
    } else {
      z_temp += Z_STEP;
    }
    // Must be positive.
    z_temp = max(z_temp, 0);
    move_arm = true;
  }
  
  // Check if motion is needed.
  if (move_arm) {
    if (set_robot(r_spd_temp, t_temp, z_temp) == IK_SUCCESS) {
      // If the arm was positioned successfully, record the new vales. Otherwise, ignore them.
      R = arm.servo.read();
      T = t_temp;
      Z = z_temp;
    } else {
      Serial.print("IK_ERROR 2");
    }
    // Reset the flag
    move_arm = false;
  }
  delay(10);
}
 
// Position robot.
int set_robot(float r_spd, float theta, float z)
{
  arm_servo.write(r_spd);
  // CODE FOR OTHER MOTORS GOES HERE
#ifdef DEBUG
  //DEBUGGING PRINTING GOES HERE
  Serial.println();
#endif
    return IK_SUCCESS;
}
