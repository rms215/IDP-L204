// File:          IDP_controller.cpp
#include <webots/Robot.hpp>

// Added a new include file
#include <webots/Motor.hpp>
#include <webots/LightSensor.hpp>
#include <webots/DistanceSensor.hpp>

#define TIME_STEP 64
#define MAX_SPEED 10

// All the webots classes are defined in the "webots" namespace
using namespace webots;

Robot *robot = new Robot();

DistanceSensor *ds[4];
char dsNames[4][40] = {"us_right","us_left","ir_left","ir_right"};

/*LightSensor *light_sensor = robot->getLightSensor("TEPT4400");
light_sensor->enable(TIME_STEP);
*/

void move_forwards() {
   // get the motor devices
   Motor *leftMotor = robot->getMotor("Wheel_L");
   Motor *rightMotor = robot->getMotor("Wheel_R");
   // set the target position of the motors
   leftMotor->setPosition(INFINITY);
   rightMotor->setPosition(INFINITY);

   // set up the motor speeds at 50% of the MAX_SPEED.
   leftMotor->setVelocity(0.5 * MAX_SPEED);
   rightMotor->setVelocity(0.5 * MAX_SPEED);
}
void move_backwards() {
   // get the motor devices
   Motor *leftMotor = robot->getMotor("Wheel_L");
   Motor *rightMotor = robot->getMotor("Wheel_R");
   // set the target position of the motors
   leftMotor->setPosition(INFINITY);
   rightMotor->setPosition(INFINITY);

   // set up the motor speeds at 50% of the MAX_SPEED.
   leftMotor->setVelocity(-0.5 * MAX_SPEED);
   rightMotor->setVelocity(-0.5 * MAX_SPEED);
}

void move_backwards_position(x)
// get the motor devices
   Motor *leftMotor = robot->getMotor("Wheel_L");
   Motor *rightMotor = robot->getMotor("Wheel_R");
   // set the target position of the motors
   leftMotor->setPosition(x);
   rightMotor->setPosition(x);

   // set up the motor speeds at 50% of the MAX_SPEED.
   leftMotor->setVelocity(-0.5 * MAX_SPEED);
   rightMotor->setVelocity(-0.5 * MAX_SPEED);
}
void rotate_CW() {
   // get the motor devices
   Motor *leftMotor = robot->getMotor("Wheel_L");
   Motor *rightMotor = robot->getMotor("Wheel_R");
   // set the target position of the motors
   leftMotor->setPosition(INFINITY);
   rightMotor->setPosition(INFINITY);

   // set up the motor speeds at 50% of the MAX_SPEED.
   leftMotor->setVelocity(0.5 * MAX_SPEED);
   rightMotor->setVelocity(-0.5 * MAX_SPEED);
}
void rotate_ACW() {
   // get the motor devices
   Motor *leftMotor = robot->getMotor("Wheel_L");
   Motor *rightMotor = robot->getMotor("Wheel_R");
   // set the target position of the motors
   leftMotor->setPosition(INFINITY);
   rightMotor->setPosition(INFINITY);

   // set up the motor speeds at 50% of the MAX_SPEED.
   leftMotor->setVelocity(-0.5*MAX_SPEED);
   rightMotor->setVelocity(0.5*MAX_SPEED);
}
void open_arms() {
   // get the motor devices
   Motor *leftMotor = robot->getMotor("Arm_L");
   Motor *rightMotor = robot->getMotor("Arm_R");
   // set the target position of the motors
   leftMotor->setPosition(0.5);
   rightMotor->setPosition(-0.5);
}
void close_arms() {
   // get the motor devices
   Motor *leftMotor = robot->getMotor("Arm_L");
   Motor *rightMotor = robot->getMotor("Arm_R");
   // set the target position of the motors
   leftMotor->setPosition(0.0);
   rightMotor->setPosition(0.0);
}

//Reads distance sensors
void scanOnSpot(){
    for (int i = 0; i < 4 ; i++){
    double dsValues[4];
    dsValues[i] = ds[i]->getValue();
    rotate_ACW();
    }
}

int main(int argc, char **argv) {

  //Initialising GPS
  GPS *gps = robot->getGPS("gps");
  gps->enable(TIME_STEP);
  //start_pos_green = []   //reccord initial position
  
  Compass *compass = robot->getCompass("compass");
  compass->enable(TIME_STEP);
  // write code to reccord initial orientation
  double initial_position[3] = {0.0, 1.0, 2.0};
  int i = 0;
  if (i = 0){
    initial_position[0] = gps->getValues()[0];
    initial_position[1] = gps->getValues()[1];
    initial_position[2] = gps->getValues()[2];
   //const double initial_north = compass->getValues();
  };
  
//Retrieving device tags and enabling with refresh time step
  for(int i = 0; i < 4; i++){
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP); //TIME_STEP defines rate at which sensor is refreshed
  }
  
  

 while (robot->step(TIME_STEP) != -1){
    
    // Main (algorithmic loop)
    rotate_ACW();
    
 }
    
 delete robot;
 return 0;
 }