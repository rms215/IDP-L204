#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/LightSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/GPS.hpp>
#include <webots/Compass.hpp>
#include <math.h>

#define TIME_STEP 32
#define MAX_SPEED 10

// All the webots classes are defined in the "webots" namespace
using namespace webots;

//Creating new robot instance
Robot *robot = new Robot();

//Get devices
Motor *leftMotor = robot->getMotor("Wheel_L");
Motor *rightMotor = robot->getMotor("Wheel_R");
Motor *leftArmMotor = robot->getMotor("Arm_L");
Motor *rightArmMotor = robot->getMotor("Arm_R");
GPS *gps = robot->getGPS("gps");
Compass *compass = robot->getCompass("compass");

//Initialsing US and IR sensors
DistanceSensor *ds[4];
char dsNames[4][20] = {"us_right","us_left","ir_left","ir_right"};
  
/*LightSensor *light_sensor = robot->getLightSensor("TEPT4400");
light_sensor->enable(TIME_STEP);
*/

//Initiate vector to store sensor readings
std::vector<std::vector<double>> dsValueScan(1000,std::vector<double> (4,0));

void rotate(double theta){

// set the target position of the motors
  leftMotor->setPosition(theta);
  rightMotor->setPosition(-theta);
  
  // set up the motor speeds at 50% of the MAX_SPEED.
   leftMotor->setVelocity(0.5 * MAX_SPEED);
   rightMotor->setVelocity(0.5 * MAX_SPEED);
}

void move_forwards() {

   leftMotor->setPosition(INFINITY);
   rightMotor->setPosition(INFINITY);

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

std::vector<double> getdsValues(){
    std::vector<double> dsValues(4);
    for (int i = 0; i < 4; i++){
      dsValues[i] = ds[i]->getValue();
    }
    return dsValues;
}

double get_bearing_in_degrees(const double *north) {
    double rad = atan2(north[0], north[2]);
    double bearing = (rad - 1.5708) / M_PI * 180.0;
    if (bearing < 0.0){
      bearing = bearing + 360.0;
    }
    return bearing;
}

/*void rotate_theta(const double *north, double theta) { //function to be used in scan_on_spot
  
  double initial_bearing = get_bearing_in_degrees(north);
  double angle_rotated = 0.0;
  
  while (angle_rotated < theta) {
    rotate_CW();
    angle_rotated++;
    double bearing = get_bearing_in_degrees(north);
    std::cout << bearing;
    
    if ((bearing - initial_bearing) >= 0.0) {
      angle_rotated = bearing - initial_bearing;
    }
    if ((bearing - initial_bearing) < 0.0) {
      angle_rotated = bearing + 360.0 - initial_bearing;
    }
    
  } 
}
*/

std::vector<double> findBlocks(std::vector<double> &dsValueScan);
   
int main(int argc, char **argv) {
    
    //Enabling navigation
    gps->enable(TIME_STEP);
    compass->enable(TIME_STEP);
  
    //Retrieving device tags and enabling with refresh time step
    for(int i = 0; i < 4; i++){
      ds[i] = robot->getDistanceSensor(dsNames[i]);
      ds[i]->enable(TIME_STEP); //TIME_STEP defines rate at which sensor is refreshed
    }
    
   int j = 0;
   
   while (robot->step(TIME_STEP) != -1){
   
      /*double initial_bearing = get_bearing_in_degrees(north);
      rotate(18.2);
      double final_bearing = get_bearing_in_degrees(north);*/
      
      std::vector<double> Values = getdsValues();
      j++;
      rotate_ACW();
      
      for(int i = 0; i<4; i++){
        std::cout << Values[i]<< ",";
       }
       
      std::cout << "\n";
      
      if(j==100){
        break;
       }
     
      /*std::cout << initial_bearing << "," << final_bearing << "\n";
      
      double initial_position[3];
      if (i == 0){
      initial_position[0] = gps->getValues()[0];
      initial_position[1] = gps->getValues()[1];
      initial_position[2] = gps->getValues()[2];
     const double initial_north = compass->getValues();
    };
      
      double bearing = get_bearing_in_degrees(north);
      //std::cout << bearing << std::endl;
      //std::cout <<  position[0] << ", "<< position[1] << ", "<< position[2] << std::endl;
      //std::cout <<  north[0] << ", "<< north[1] << ", "<< north[2] << std::endl;
      //std::cout <<  initial_position[0] << ", "<< initial_position[1] << ", "<< initial_position[2] << std::endl;
      //std::cout << i << std::endl;
      //i++;
      */
   }
      
   delete robot;
   return 0;
 }