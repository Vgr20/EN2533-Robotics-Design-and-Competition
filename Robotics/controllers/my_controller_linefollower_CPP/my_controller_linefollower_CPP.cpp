// File:          linefollower_cpp.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/PositionSensor.hpp>


#define TIME_STEP 16
#define baseSpeed 10


// All the webots classes are defined in the "webots" namespace
using namespace webots;

double kp = 1.7;
double ki = 0;
double kd = 1.5;
double last_error = 0;
double P = 0;
double I = 0;
double D = 0;
double error = 0;



// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  
  

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  int timeStep = (int)robot->getBasicTimeStep();
  Motor *leftMotor1 = robot->getMotor("Motor_1");
  Motor *leftMotor2 = robot->getMotor("Motor_2");
  Motor *rightMotor1 = robot->getMotor("Motor_3");
  Motor *rightMotor2 = robot->getMotor("Motor_4");
  

 
  
  leftMotor1->setPosition(INFINITY);
  leftMotor2->setPosition(INFINITY);
  rightMotor1->setPosition(INFINITY);
  rightMotor2->setPosition(INFINITY);
  

  leftMotor1->setVelocity(0.0);
  leftMotor2->setVelocity(0.0);
  rightMotor1->setVelocity(0.0);
  rightMotor2->setVelocity(0.0);
  
  DistanceSensor *leftIR = robot->getDistanceSensor("ds_left");
  DistanceSensor *midIR = robot->getDistanceSensor("ds_mid");
  DistanceSensor *rightIR = robot->getDistanceSensor("ds_right");
  DistanceSensor *rightMIR = robot->getDistanceSensor("ds_right_most");
  DistanceSensor *leftMIR = robot->getDistanceSensor("ds_left_most");
  
  
  leftIR->enable(TIME_STEP);
  midIR->enable(TIME_STEP);
  rightIR->enable(TIME_STEP);
  rightMIR->enable(TIME_STEP);
  leftMIR->enable(TIME_STEP);
  
  
  double left_speed = baseSpeed;
  double right_speed = baseSpeed;

  

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    double rightIR_value = rightIR->getValue();
    double midIR_value = midIR->getValue();
    double leftIR_value = leftIR->getValue();
    double leftMIR_value = leftMIR->getValue();
    double rightMIR_value = rightMIR->getValue();
    

    
    std::cout<<"left_Most= "<<leftMIR_value<<" left = "<<leftIR_value<< "  mid = "<<midIR_value<<"  right = "<<rightIR_value<<"  right_Most = "<<rightMIR_value<<std::endl;

    
    if ((leftIR_value >= 400) && (rightIR_value >= 400) && (midIR_value < 400)){
      error = 0;
    } else if ((leftMIR_value >= 400) &&(leftIR_value >= 400) && (rightIR_value < 400) && (midIR_value >= 400) && (rightMIR_value >= 400)){
      error = -1;
    } else if ((leftMIR_value >= 400) &&(leftIR_value < 400) && (rightIR_value >= 400) && (midIR_value >= 400) && (rightMIR_value >= 400)){
      error = 1;
    } else if ((leftMIR_value >= 400) && (leftIR_value >= 400) && (rightIR_value >= 400) && (midIR_value >= 400) && (rightMIR_value < 400)){
      error = -2;
    } else if ((leftMIR_value < 400) && (leftIR_value >= 400) && (rightIR_value >=  400) && (midIR_value >= 400)&& (rightMIR_value >= 400)){
      error = 2;
    } 
    
    P = error;
    std::cout<<"p = "<<P<<std::endl;
    I = error + I;
    std::cout<<"I = "<<I<<std::endl;
    D = error - last_error;
    std::cout<<"D = "<<D<<std::endl;
    double balance = (kp*P)+(ki*I)+(kd*D);
    std::cout<<"balance = "<<balance<<std::endl;
    last_error = error;
    std::cout<<"last_error = "<<last_error<<std::endl;
    
    left_speed = baseSpeed + balance;
    std::cout<<"left_speed = "<<left_speed<<std::endl;
    
    right_speed = baseSpeed - balance;
    std::cout<<"right_speed = "<<right_speed<<std::endl; 
    


    
    
    if (left_speed > baseSpeed){
      leftMotor1->setVelocity(left_speed);
      rightMotor1->setVelocity(0);
      leftMotor2->setVelocity(left_speed);
      rightMotor2->setVelocity(0);
    }
    
    if (right_speed >  baseSpeed){
      rightMotor1->setVelocity(left_speed);
      leftMotor1->setVelocity(0);
      rightMotor2->setVelocity(left_speed);
      leftMotor2->setVelocity(0);
    }
    
    if (right_speed < 0){
      leftMotor1->setVelocity(left_speed);
      rightMotor1->setVelocity(0);
      leftMotor2->setVelocity(left_speed);
      rightMotor2->setVelocity(0);
    }
    
    if (left_speed < 0){
      rightMotor1->setVelocity(left_speed);
      leftMotor1->setVelocity(0);
      rightMotor2->setVelocity(left_speed);
      leftMotor2->setVelocity(0);
    }
    
    if (left_speed ==  baseSpeed){
      leftMotor1->setVelocity(left_speed);
      rightMotor1->setVelocity(right_speed);
      leftMotor2->setVelocity(left_speed);
      rightMotor2->setVelocity(right_speed);
      
    }
    
   


    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
    
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}