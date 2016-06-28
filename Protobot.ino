/*
 * PS2 Controls
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

// Arm dimensions (mm). Standard AL5D arm, but with longer arm segments
#define BASE_HGT 80.9625    // Base height to X/Y plane 3.1875"
#define HUMERUS 263.525     // Shoulder-to-elbow "bone" 10.375"
#define ULNA 325.4375       // Elbow-to-wrist "bone" 12.8125"
#define GRIPPER 73.025      // Gripper length, to middle of grip surface 2.875" (3.375" - 0.5")

// Arduino pin numbers for servo connections
#define base_servo_PIN 2     // Base servo HS-485HB
#define shoulder_servo_PIN 3     // Shoulder Servo HS-805BB
#define elbow_servo_PIN 4     // Elbow Servo HS-755HB
#define wrist_servo_PIN 10    // Wrist servo HS-645MG
#define gripper_servo_PIN 11    // Gripper servo HS-422

// Arduino pin numbers for PS2 controller connections
#define PS2_CLK_PIN 9       // Clock
#define PS2_CMD_PIN 7       // Command
#define PS2_ATT_PIN 8       // Attention
#define PS2_DAT_PIN 6       // Data

// Arduino pin number of on-board speaker
#define SPK_PIN 5

// Define generic range limits for servos, in microseconds (us) and degrees (deg)
// Used to map range of 180 deg to 1800 us (native servo units).
// Specific per-servo/joint limits are defined below
#define SERVO_MIN_US 600
#define SERVO_MID_US 1500
#define SERVO_MAX_US 2400
#define SERVO_MIN_DEG 0.0
#define SERVO_MID_DEG 90.0
#define SERVO_MAX_DEG 180.0

// Set physical limits (in degrees) per servo/joint.
// Will vary for each servo/joint, depending on mechanical range of motion.
// The MID setting is the required servo input needed to achieve a 
// 90 degree joint angle, to allow compensation for horn misalignment
#define BAS_MIN 0.0         // Fully CCW
#define BAS_MID 90.0
#define BAS_MAX 180.0       // Fully CW

#define SHL_MIN 20.0        // Max forward motion
#define SHL_MID 81.0
#define SHL_MAX 140.0       // Max rearward motion

#define ELB_MIN 20.0        // Max upward motion
#define ELB_MID 88.0
#define ELB_MAX 165.0       // Max downward motion

#define WRI_MIN 0.0         // Max downward motion
#define WRI_MID 93.0
#define WRI_MAX 180.0       // Max upward motion

#define GRI_MIN 25.0        // Fully open
#define GRI_MID 90.0
#define GRI_MAX 165.0       // Fully closed

// Speed adjustment parameters
// Percentages (1.0 = 100%) - applied to all arm movements
#define SPEED_MIN 0.5
#define SPEED_MAX 1.5
#define SPEED_DEFAULT 1.0
#define SPEED_INCREMENT 0.25

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

// Ready-to-run arm position.
#define READY_X 0.0
#define READY_Y 170.0
#define READY_Z 45.0
#define READY_GA 0.0
#define READY_G GRI_MID


// Global variables for arm position, and initial settings
float X = READY_X;         // Left/right distance (mm) from base centerline - 0 is straight
float Y = READY_Y;          // Distance (mm) out from base center
float Z = READY_Z;          // Height (mm) from surface (i.e. X/Y plane)
float GA = READY_GA;        // Gripper angle. Servo degrees, relative to X/Y plane - 0 is horizontal
float G = READY_G;          // Gripper jaw opening. Servo degrees - midpoint is halfway open
float Speed = SPEED_DEFAULT;

// Pre-calculations
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;

// PS2 Controller object
PS2X    Ps2x;

// Servo objects 
Servo   base_servo;
Servo   shoulder_servo;
Servo   elbow_servo;
Servo   wrist_servo;
Servo   gripper_servo;

 
void setup()
{
    // Attach to the servos and specify range limits
    base_servo.attach(base_servo_PIN, SERVO_MIN_US, SERVO_MAX_US);
    shoulder_servo.attach(shoulder_servo_PIN, SERVO_MIN_US, SERVO_MAX_US);
    elbow_servo.attach(elbow_servo_PIN, SERVO_MIN_US, SERVO_MAX_US);
    wrist_servo.attach(wrist_servo_PIN, SERVO_MIN_US, SERVO_MAX_US);
    gripper_servo.attach(gripper_servo_PIN, SERVO_MIN_US, SERVO_MAX_US);


    // Setup PS2 controller. Loop until ready.
    byte    ps2_stat;
    do {
        ps2_stat = Ps2x.config_gamepad(PS2_CLK_PIN, PS2_CMD_PIN, PS2_ATT_PIN, PS2_DAT_PIN);

    } while (ps2_stat == 1);
 


    // NOTE: Ensure arm is close to the desired park position before turning on servo power!
    park_servos(PARK_READY);



    delay(500);


}
 
void loop()
{
    // Store desired position in tmp variables until confirmed by set_arm() logic
         // 3D kinematics
    float x_tmp = X;

    float y_tmp = Y;
    float z_tmp = Z;
    float ga_tmp = GA;
    
    // Used to indidate whether an input occurred that can move the arm
    boolean arm_move = false;

    Ps2x.read_gamepad();        //read controller

    // Read the left and right joysticks and translate the 
    // normal range of values (0-255) to zero-centered values (-128 - 128)
    int ly_trans = JS_MIDPOINT - Ps2x.Analog(PSS_LY);
    int lx_trans = Ps2x.Analog(PSS_LX) - JS_MIDPOINT;
    int ry_trans = JS_MIDPOINT - Ps2x.Analog(PSS_RY);
    int rx_trans = Ps2x.Analog(PSS_RX) - JS_MIDPOINT;



    // X Position (in mm)
    // Can be positive or negative. Servo range checking in IK code
    if (abs(rx_trans) > JS_DEADBAND) {
        x_tmp += ((float)rx_trans / JS_IK_SCALE * Speed);
        arm_move = true;


    // Y Position (in mm)
    // Must be > Y_MIN. Servo range checking in IK code
    if (abs(ry_trans) > JS_DEADBAND) {
        y_tmp += ((float)ry_trans / JS_IK_SCALE * Speed);
        y_tmp = max(y_tmp, Y_MIN);
        arm_move = true;
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

    // Gripper angle (in degrees) relative to horizontal
    // Can be positive or negative. Servo range checking in IK code
    if (abs(ly_trans) > JS_DEADBAND) {
        ga_tmp -= ((float)ly_trans / JS_SCALE * Speed);
        arm_move = true;
    }

    // Gripper jaw position (in degrees - determines width of jaw opening)
    // Restrict to MIN/MAX range of servo
    if (Ps2x.Button(PSB_L1) || Ps2x.Button(PSB_L2)) {
        if (Ps2x.Button(PSB_L1)) {
            G += G_INCREMENT;   // close
        } else {
            G -= G_INCREMENT;   // open
        }
        G = constrain(G, GRI_MIN, GRI_MAX);
        gripper_servo.writeMicroseconds(deg_to_us(G));
    }

    // Fully open gripper
    if (Ps2x.ButtonPressed(PSB_BLUE)) {
        G = GRI_MIN;
        gripper_servo.writeMicroseconds(deg_to_us(G));
    }
    
    // Speed increase/decrease
    if (Ps2x.ButtonPressed(PSB_PAD_UP) || Ps2x.ButtonPressed(PSB_PAD_DOWN)) {
        if (Ps2x.ButtonPressed(PSB_PAD_UP)) {
            Speed += SPEED_INCREMENT;   // increase speed
        } else {
            Speed -= SPEED_INCREMENT;   // decrease speed
        }
        // Constrain to limits
        Speed = constrain(Speed, SPEED_MIN, SPEED_MAX);
    }
    

    // Only perform IK calculations if arm motion is needed.
    if (arm_move) {

        if (set_arm(x_tmp, y_tmp, z_tmp, ga_tmp) == IK_SUCCESS) {
            // If the arm was positioned successfully, record
            // the new vales. Otherwise, ignore them.
            X = x_tmp;
            Y = y_tmp;
            Z = z_tmp;
            GA = ga_tmp;
        } else {
            Serial.print("IK Error");
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
int set_arm(float x, float y, float z, float grip_angle_d)
{
    //grip angle in radians for use in calculations
    float grip_angle_r = radians(grip_angle_d);    
  
    // Base angle and radial distance from x,y coordinates
    float bas_angle_r = atan2(x, y);
    float rdist = sqrt((x * x) + (y * y));
  
    // rdist is y coordinate for the arm
    y = rdist;
    
    // Grip offsets calculated based on grip angle
    float grip_off_z = (sin(grip_angle_r)) * GRIPPER;
    float grip_off_y = (cos(grip_angle_r)) * GRIPPER;
    
    // Wrist position
    float wrist_z = (z - grip_off_z) - BASE_HGT;
    float wrist_y = y - grip_off_y;
    
    // Shoulder to wrist distance (AKA sw)
    float s_w = (wrist_z * wrist_z) + (wrist_y * wrist_y);
    float s_w_sqrt = sqrt(s_w);
    
    // s_w angle to ground
    float a1 = atan2(wrist_z, wrist_y);
    
    // s_w angle to humerus
    float a2 = acos(((hum_sq - uln_sq) + s_w) / (2 * HUMERUS * s_w_sqrt));
    
    // Shoulder angle
    float shl_angle_r = a1 + a2;
    // If result is NAN or Infinity, the desired arm position is not possible
    if (isnan(shl_angle_r) || isinf(shl_angle_r))
        return IK_ERROR;
    float shl_angle_d = degrees(shl_angle_r);
    
    // Elbow angle
    float elb_angle_r = acos((hum_sq + uln_sq - s_w) / (2 * HUMERUS * ULNA));
    // If result is NAN or Infinity, the desired arm position is not possible
    if (isnan(elb_angle_r) || isinf(elb_angle_r))
        return IK_ERROR;
    float elb_angle_d = degrees(elb_angle_r);
    float elb_angle_dn = -(180.0 - elb_angle_d);
    
    // Wrist angle
    float wri_angle_d = (grip_angle_d - elb_angle_dn) - shl_angle_d;
 
    // Calculate servo angles
    // Calc relative to servo midpoint to allow compensation for servo alignment
    float bas_pos = BAS_MID + degrees(bas_angle_r);
    float shl_pos = SHL_MID + (shl_angle_d - 90.0);
    float elb_pos = ELB_MID - (elb_angle_d - 90.0);
    float wri_pos = WRI_MID + wri_angle_d;
    
    // If any servo ranges are exceeded, return an error
    if (bas_pos < BAS_MIN || bas_pos > BAS_MAX || shl_pos < SHL_MIN || shl_pos > SHL_MAX || elb_pos < ELB_MIN || elb_pos > ELB_MAX || wri_pos < WRI_MIN || wri_pos > WRI_MAX)
        return IK_ERROR;
    
    // Position the servos
    base_servo.writeMicroseconds(deg_to_us(bas_pos));
    shoulder_servo.writeMicroseconds(deg_to_us(shl_pos));
    elbow_servo.writeMicroseconds(deg_to_us(elb_pos));
    wrist_servo.writeMicroseconds(deg_to_us(wri_pos));


    return IK_SUCCESS;
}
 
// Move servos to parking position
void park_servos(int park_type)
{
    switch (park_type) {
        // All servos at midpoint
        case PARK_MIDPOINT:
            base_servo.writeMicroseconds(deg_to_us(BAS_MID));
            shoulder_servo.writeMicroseconds(deg_to_us(SHL_MID));
            elbow_servo.writeMicroseconds(deg_to_us(ELB_MID));
            wrist_servo.writeMicroseconds(deg_to_us(WRI_MID));
            gripper_servo.writeMicroseconds(deg_to_us(GRI_MID));
            break;
        
        // Ready-To-Run position
        case PARK_READY:

            set_arm(READY_X, READY_Y, READY_Z, READY_GA);

            gripper_servo.writeMicroseconds(deg_to_us(READY_G));

            break;
    }

    return;
}

// Converts deg to us to take advantage of extra servo resolution.
int deg_to_us(float value)
{
    // Apply basic constraints
    if (value < SERVO_MIN_DEG) value = SERVO_MIN_DEG;
    if (value > SERVO_MAX_DEG) value = SERVO_MAX_DEG;
    
    // Map degrees to microseconds, and round the result to a whole number
    return(round(map_float(value, SERVO_MIN_DEG, SERVO_MAX_DEG, (float)SERVO_MIN_US, (float)SERVO_MAX_US)));      
}

// Same logic as native map() function, just operates on float instead of long
float map_float(float x, float in_min, float in_max, float out_min, float out_max)
{
  return ((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
}
