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

// get the time step of the current world.
int timeStep = (int)robot->getBasicTimeStep();

LightSensor *light_sensor = robot->getLightSensor("TEPT4400");
light_sensor->enable(TIME_STEP);

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
   leftMotor->setVelocity(-0.5 * MAX_SPEED);
   rightMotor->setVelocity(0.5 * MAX_SPEED);
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
    double dsValues[3];
    for (int i = 0; i < 3 ; i++)
    dsValues[i] = ds[i]->getValue();
    rotate_ACW();
}

int main(int argc, char **argv) {
  //Initialising US and IR sensors
//Initialise array of pointers of type DistanceSensor device tag
  DistanceSensor *ds[3];
  char dsNames[3][40] = {"ds_right","ds_left","Sharp's IR sensor GP2Y0A02YK0F"};
  
//Retrieving device tags and enabling with refresh time step
  for(int i = 0; i < 3; i++){
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP); //TIME_STEP defines rate at which sensor is refreshed
  }

 while (robot->step(TIME_STEP) != -1){
    
    // Main (algorithmic loop)
    scanOnSpot();
    
 };
    
 delete robot;
 return 0;
 }