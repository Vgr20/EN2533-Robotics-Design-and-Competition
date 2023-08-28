#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

#define TIME_STEP 64
#define MAX_SPEED 10


using namespace webots;


int main(int argc, char **argv) {

  Robot *robot = new Robot();


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
  
  while (robot->step(timeStep) != -1) {

    leftMotor1->setVelocity(MAX_SPEED);
    rightMotor1->setVelocity(MAX_SPEED);
    leftMotor2->setVelocity(MAX_SPEED);
    rightMotor2->setVelocity(MAX_SPEED);

  };

  delete robot;
  return 0;
}