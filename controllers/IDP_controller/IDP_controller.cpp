// File:          IDP_controller.cpp
#include <webots/Robot.hpp>

// Added a new include file
#include <webots/Motor.hpp>
#include <webots/LightSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>

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

//Initialising GPS
GPS *gps = robot->getGPS("gps");
gps->enable(TIME_STEP);

//Initialising compass
Compass *compass = robot->getCompass("compass");
compass->enable(TIME_STEP);

// get the motor devices
Motor *leftMotor = robot->getMotor("Wheel_L");
Motor *rightMotor = robot->getMotor("Wheel_R");

// get the arm motor devices
Motor *leftArmMotor = robot->getMotor("Arm_L");
Motor *rightArmMotor = robot->getMotor("Arm_R");

void move_forwards() {
   // set the target position of the motors
   leftMotor->setPosition(INFINITY);
   rightMotor->setPosition(INFINITY);

   // set up the motor speeds at 50% of the MAX_SPEED.
   leftMotor->setVelocity(0.5 * MAX_SPEED);
   rightMotor->setVelocity(0.5 * MAX_SPEED);
}
void move_backwards() {
   // set the target position of the motors
   leftMotor->setPosition(INFINITY);
   rightMotor->setPosition(INFINITY);

   // set up the motor speeds at 50% of the MAX_SPEED.
   leftMotor->setVelocity(-0.5 * MAX_SPEED);
   rightMotor->setVelocity(-0.5 * MAX_SPEED);
}

void move_position(double x){
   // set the target position of the motors
   leftMotor->setPosition(x);
   rightMotor->setPosition(x);
}

void rotate_CW() {
   // set the target position of the motors
   leftMotor->setPosition(INFINITY);
   rightMotor->setPosition(INFINITY);

   // set up the motor speeds at 50% of the MAX_SPEED.
   leftMotor->setVelocity(0.5 * MAX_SPEED);
   rightMotor->setVelocity(-0.5 * MAX_SPEED);
}
void rotate_ACW() {
   // set the target position of the motors
   leftMotor->setPosition(INFINITY);
   rightMotor->setPosition(INFINITY);

   // set up the motor speeds at 50% of the MAX_SPEED.
   leftMotor->setVelocity(-0.5*MAX_SPEED);
   rightMotor->setVelocity(0.5*MAX_SPEED);
}
void open_arms() {
   // set the target position of the motors
   leftArmMotor->setPosition(0.5);
   rightArmMotor->setPosition(-0.5);
}
void close_arms() {
   // set the target position of the motors
   leftMotor->setPosition(0.0);
   rightMotor->setPosition(0.0);
}

void scanOnSpot(){
    for (int i = 0; i < 4 ; i++){
    double dsValues[4];
    dsValues[i] = ds[i]->getValue();
    rotate_ACW();
    }
}

double get_bearing_in_degrees(const double *north) {
  //const double *north = wb_compass_get_values(tag);
  double rad = atan2(north[0], north[2]);
  double bearing = (rad - 1.5708) / M_PI * 180.0;
  if (bearing < 0.0)
    bearing = bearing + 360.0;
  return bearing;
}

void rotate_360() { //function to be used in scan_on_spot
}

int main(int argc, char **argv) {
  
  // position and orientation pointers
  const double *position = gps->getValues();
  const double *north = compass->getValues();

//Retrieving device tags and enabling with refresh time step
  for(int i = 0; i < 4; i++){
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP); //TIME_STEP defines rate at which sensor is refreshed
  }
  
  
  double initial_position[3] = {0.0, 1.0, 2.0}; //initiate initial position
  int i = 0; //initiate time step counter
 while (robot->step(TIME_STEP) != -1){

    if (i == 0){
    initial_position[0] = gps->getValues()[0];
    initial_position[1] = gps->getValues()[1];
    initial_position[2] = gps->getValues()[2];
   //const double initial_north = compass->getValues();
  };
  
    // Main (algorithmic loop)
    rotate_ACW();
    double bearing = get_bearing_in_degrees(north);
    std::cout << bearing << std::endl;
    //std::cout <<  position[0] << ", "<< position[1] << ", "<< position[2] << std::endl;
    //std::cout <<  north[0] << ", "<< north[1] << ", "<< north[2] << std::endl;
    //std::cout <<  initial_position[0] << ", "<< initial_position[1] << ", "<< initial_position[2] << std::endl;
    //std::cout << i << std::endl;
    
    i+=1;
 }
    
 delete robot;
 return 0;
 }