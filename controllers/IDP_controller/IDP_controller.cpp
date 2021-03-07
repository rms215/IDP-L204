// File:          IDP_controller.cpp
#include <webots/Robot.hpp>
#include <webots/GPS.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Compass.hpp>

#define TIME_STEP 64
#define MAX_SPEED 10

using namespace webots;
Robot *robot = new Robot();


//class GPS : public Device {
/*class GPS : public Device {
  virtual void enable(int samplingPeriod);
  virtual void disable();
  int getSamplingPeriod() const;
  const double *getValues() const;
  const double getSpeed() const;
  // ...
 };*/

void move_forwards() {
   // get the motor devices
   Motor *leftMotor = robot->getMotor("Wheel_L");
   Motor *rightMotor = robot->getMotor("Wheel_R");
   // set the target position of the motors
   leftMotor->setPosition(INFINITY);
   rightMotor->setPosition(INFINITY);

   // set up the motor speeds at 50% of the MAX_SPEED.
   leftMotor->setVelocity(1.0 * MAX_SPEED);
   rightMotor->setVelocity(1.0 * MAX_SPEED);
};
void move_backwards() {
   // get the motor devices
   Motor *leftMotor = robot->getMotor("Wheel_L");
   Motor *rightMotor = robot->getMotor("Wheel_R");
   // set the target position of the motors
   leftMotor->setPosition(INFINITY);
   rightMotor->setPosition(INFINITY);

   // set up the motor speeds at 50% of the MAX_SPEED.
   leftMotor->setVelocity(-1.0 * MAX_SPEED);
   rightMotor->setVelocity(-1.0 * MAX_SPEED);
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
 // wb_robot_init();
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
  while (robot->step(TIME_STEP) != -1){
    if (i == 0){ //recccording initial values
      initial_position[0] = gps->getValues()[0];
      initial_position[1] = gps->getValues()[1];
      initial_position[2] = gps->getValues()[2];
      //const double initial_north = compass->getValues();
      };
    const double *position = gps->getValues();
    const double *north = compass->getValues();
    move_forwards();

    std::cout <<  position[0] << ", "<< position[1] << ", "<< position[2] << std::endl;
    //std::cout <<  north[0] << ", "<< north[1] << ", "<< north[2] << std::endl;
    std::cout <<  initial_position[0] << ", "<< initial_position[1] << ", "<< initial_position[2] << std::endl;
    std::cout << i << std::endl;
    i += 1;
    }

 delete robot;

 return 0;
}