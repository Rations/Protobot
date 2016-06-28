#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

Adafruit_MotorShield AFMStop(0x61); // Rightmost jumper closed
Adafruit_MotorShield AFMSbot(0x60); // Default address, no jumpers

Adafruit_DCMotor *gearMotor_1 = AFMSbot.getMotor(1);
Adafruit_DCMotor *gearMotor_2 = AFMSbot.getMotor(2); 

Adafruit_StepperMotor *stepperMotor_1 = AFMSbot.getStepper(400, 2);

void setup() {
  AFMStop.begin();
  AFMSbot.begin();
  
  gearMotor_1->setSpeed(//speed);
  gearMotor_2->setSpeed(//speed); on a scale of 0 to 255 

  //when we want to run the motor use gearMotor_i->run(DIRECTION)
  //DIRECTION is in all caps and is either FORWARD, BACKWARD, or RELEASE
    
  stepperMotor_1->setSpeed(//speed in rpm);

    /*Then every time you want the motor to move, call the step(#steps, direction, steptype) procedure. #steps is how many steps you'd like it to take. direction is either FORWARD or BACKWARD and the step type is SINGLE, DOUBLE, INTERLEAVE or MICROSTEP.
      "Single" means single-coil activation
      "Double" means 2 coils are activated at once (for higher torque)
      "Interleave" means that it alternates between single and double to get twice the resolution (but of course its half the speed).
      "Microstepping" is a method where the coils are PWM'd to create smooth motion between steps.
    */
    
  //note on multiple shields
  /* The only thing to watch for when stacking shields is every shield must have a unique I2C address. 
     The default address is 0x60. 
     You can adjust the address by soldering the jumpers above the logic jumpers.
     Solder bridge A0 for the second board.
  */

}

void loop() {
  // put your main code here, to run repeatedly:

}
