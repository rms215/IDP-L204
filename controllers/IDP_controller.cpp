// File:          IDP_controller.cpp
#include <webots/Robot.hpp>

// Added a new include file
#include <webots/Motor.hpp>

#define TIME_STEP 64
#define MAX_SPEED 10

// All the webots classes are defined in the "webots" namespace
using namespace webots;

//Initialising instance of Robot
Robot *robot = new Robot();

//Initialising US and IR sensors
DistanceSensor *ds[3];
char dsNames[3][40] = {"us_right","us_left","Sharp's IR sensor GP2Y0A02YK0F")};

 //Retrieving device tags and enabling with refresh time step
for(int i = 0; i < 3; i++){
   ds[i] = robot->getDistanceSensor(dsNames[i]);
   ds[i]->enable(TIME_STEP); //TIME_STEP defines rate at which sensor is refreshed
}

LightSensor *light = robot->getLightSensor("TEPT4400");
light->enable(TIME_STEP);

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
int main(int argc, char **argv) {



 while (robot->step(TIME_STEP) != -1){
 
    // Read the sensors:
    double psValues[3];
    for (int i = 0; i < 3 ; i++)
    psValues[i] = ps[i]->getValue();
    
    
    
    };
    
 delete robot;
 return 0;
 }
}