/*
 * Default (i.e., original) PS2 controls:
 * Right Joystick L/R: Gripper tip X position (side to side)
 * Right Joystick U/D: Gripper tip Y position (distance out from base center)
 * R1/R2 Buttons:      Gripper tip Z position (height from surface)
 * Left  Joystick L/R: Wrist rotate (if installed)
 * Left  Joystick U/D: Wrist angle
 * L1/L2 Buttons:      Gripper close/open
 * X Button:           Gripper fully open
 * Digital Pad U/D:    Speed increase/decrease
 */
#include <Servo.h>
#include <PS2X_lib.h>

int dummy;                  // Defining this dummy variable to work around a bug in the IDE (1.0.3) pre-processor that messes up #ifdefs
//#define DEBUG             // Uncomment to turn on debugging output
                            
// Arduino pin numbers for servo connections
#define ARM_SERVO_PIN 4     // Elbow Servo HS-755HB

// Arduino pin numbers for PS2 controller connections
#define PS2_CLK_PIN 9       // Clock
#define PS2_CMD_PIN 7       // Command
#define PS2_ATT_PIN 8       // Attention
#define PS2_DAT_PIN 6       // Data

// Define generic range limits for servos, in microseconds (us) and degrees (deg)
// Used to map range of 180 deg to 1800 us (native servo units).
#define SERVO_MIN_US 600
#define SERVO_MID_US 1500
#define SERVO_MAX_US 2400
#define SERVO_MIN_DEG 0.0
#define SERVO_MID_DEG 90.0
#define SERVO_MAX_DEG 180.0

// Speed adjustment parameters
// Percentages (1.0 = 100%) - applied to all arm movements
#define SPEED_DEFAULT 1.0

// Practical navigation limit.
// Enforced on controller input, and used for CLV calculation 
// for base rotation in 2D mode. 
#define Y_MIN 100.0         // mm

// PS2 controller characteristics
#define JS_MIDPOINT 128     // Numeric value for joystick midpoint
#define JS_DEADBAND 4       // Ignore movement this close to the center position
#define JS_IK_SCALE 50.0    // Divisor for scaling JS output for IK control
#define JS_SCALE 100.0      // Divisor for scaling JS output for raw servo control
#define Z_INCREMENT 2.0     // Change in Z axis (mm) per button press
#define G_INCREMENT 2.0     // Change in Gripper jaw opening (servo angle) per button press
 
// IK function return values
#define IK_SUCCESS 0
#define IK_ERROR 1          // Desired position not possible

// Arm parking positions
#define PARK_MIDPOINT 1     // Servos at midpoints
#define PARK_READY 2        // Arm at Ready-To-Run position

// Ready-To-Run arm position. See descriptions below
// NOTE: Have the arm near this position before turning on the 
//       servo power to prevent whiplash

 #define READY_X 0.0

#define READY_Y 170.0
#define READY_Z 45.0
#define READY_G GRI_MID

// Global variables for arm position, and initial settings

float X = READY_X;         // Left/right distance (mm) from base centerline - 0 is straight
float Y = READY_Y;          // Distance (mm) out from base center
float Z = READY_Z;          // Height (mm) from surface (i.e. X/Y plane)
float Speed = SPEED_DEFAULT;


// Declare PS2 controller and servo objects
PS2X    Ps2x;
Servo   arm_servo;
 
void setup() {
#ifdef DEBUG
  Serial.begin(115200);
#endif
  // Attach to the servos and specify range limits
  arm_servo.attach(ARM_SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  
  // Setup PS2 controller. Loop until ready.
  byte ps2_stat;
  do {
    ps2_stat = Ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_ATT_PIN, PS2_DAT_PIN);
#ifdef DEBUG
        if (ps2_stat == 1)
            Serial.println("No controller found. Re-trying ...");
#endif
  } while (ps2_stat == 1);
 
#ifdef DEBUG
  switch (ps2_stat) {
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
    servo_park(PARK_READY);

#ifdef DEBUG
    Serial.println("Start");
#endif

    delay(500);
}
 
void loop()
{
    // Store desired position in tmp variables until confirmed by set_arm() logic
    float x_tmp = X;
    float y_tmp = Y;
    float z_tmp = Z;
    
    // Used to indidate whether an input occurred that can move the arm
    boolean arm_move = false;

    Ps2x.read_gamepad();

    // Read the left and right joysticks and translate the normal range of values (0-255) to zero-centered values (-128 - 128)
              //    int ly_trans = JS_MIDPOINT - Ps2x.Analog(PSS_LY);
              //    int lx_trans = Ps2x.Analog(PSS_LX) - JS_MIDPOINT;
    int r_joystick_y = JS_MIDPOINT - Ps2x.Analog(PSS_RY);

    // Y Position (in mm)
    // Must be > Y_MIN. Servo range checking in IK code
    if (abs(r_joystick_y) > JS_DEADBAND) {
        y_tmp += ((float)r_joystick_y / JS_IK_SCALE * Speed);
        y_tmp = max(y_tmp, Y_MIN);
        arm_move = true;
        
        if (y_tmp == Y_MIN) {
            Serial.print("IK ERROR 1");
        }
    }

    // Z Position (in mm)
    // Must be positive. Servo range checking in IK code
    if (Ps2x.Button(PSB_R1) || Ps2x.Button(PSB_R2)) {
        if (Ps2x.Button(PSB_R1)) {
            z_tmp += Z_INCREMENT * Speed;   // up
        } else {
            z_tmp -= Z_INCREMENT * Speed;   // down
        }
        z_tmp = max(z_tmp, 0);
        arm_move = true;
    }
 
    // Only perform IK calculations if arm motion is needed.
    if (arm_move) {
        if (set_arm(x_tmp, y_tmp, z_tmp) == IK_SUCCESS) {
            // If the arm was positioned successfully, record
            // the new vales. Otherwise, ignore them.
            X = x_tmp;
            Y = y_tmp;
            Z = z_tmp;
        } else {
            Serial.print("IK_ERROR 2");
        }
        // Reset the flag
        arm_move = false;
    }
    delay(10);
 }
 
// Arm positioning routine utilizing Inverse Kinematics.
// Z is height, Y is distance from base center out, X is side to side. Y, Z can only be positive.
// Input dimensions are for the gripper, just short of its tip, where it grabs things.
// If resulting arm position is physically unreachable, return error code.
int set_arm(float x, float y, float z)
{
    // rdist is y coordinate for the arm
    y = rdist;
    
    // Position the servos
    arm_servo.writeMicroseconds(deg_to_us(elb_pos));
#ifdef DEBUG
  //DEBUGGING PRINTING GOES HERE
  Serial.println();
#endif
    return IK_SUCCESS;
}
 
// Move servos to parking position
void servo_park(int park_type) {
    switch (park_type) {
        // All servos at midpoint
        case PARK_MIDPOINT:
            arm_servo.writeMicroseconds(deg_to_us(ELB_MID));
            break;
        
        // Ready-To-Run position
        case PARK_READY:
            set_arm(READY_X, READY_Y, READY_Z);
            break;
    }
    return;
}

// Converts deg to us to take advantage of additional servo resolution.
int deg_to_us(float value) {
    // Apply basic constraints
    if (value < SERVO_MIN_DEG) value = SERVO_MIN_DEG;
    if (value > SERVO_MAX_DEG) value = SERVO_MAX_DEG;
    
    // Map degrees to microseconds, and round the result to a whole number
    return(round(map_float(value, SERVO_MIN_DEG, SERVO_MAX_DEG, (float)SERVO_MIN_US, (float)SERVO_MAX_US)));      
}

// Same logic as native map() function, just operates on float instead of long
float map_float(float x, float in_min, float in_max, float out_min, float out_max) {
  return ((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
}
