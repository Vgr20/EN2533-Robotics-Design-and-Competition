#include <iostream>
#include <sstream>
#include <map>

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>

#define TIME_STEP 16

using namespace webots;

static double   last_error,error = 0;
static double   left_speed;
static double   right_speed;
static double P = 0;
static double I = 0;
static double D = 0;
static double P_1 = 0;
static double I_1 = 0;
static double D_1 = 0;
static double P_2 = 0;
static double I_2 = 0;
static double D_2 = 0;
int REFERENCE_DISTANCE = 800;


double error1,error2,last_error1,last_error2;

 


Robot*robot = new Robot();
DistanceSensor *leftIR     = robot->getDistanceSensor("ds_left");
DistanceSensor *midIR      = robot->getDistanceSensor("ds_mid");
DistanceSensor *rightIR    = robot->getDistanceSensor("ds_right");
DistanceSensor *rightMIR   = robot->getDistanceSensor("ds_right_most");
DistanceSensor *leftMIR    = robot->getDistanceSensor("ds_left_most");

DistanceSensor *sensors[5] = {leftIR,midIR,rightIR,rightMIR,leftMIR};

DistanceSensor *leftPS     = robot->getDistanceSensor("sensor_left");    
DistanceSensor *centerPS   = robot->getDistanceSensor("sensor_center");
DistanceSensor *rightPS    = robot->getDistanceSensor("sensor_right");
DistanceSensor *f_leftPS   = robot->getDistanceSensor("sensor_front_left");
DistanceSensor *f_rightPS  = robot->getDistanceSensor("sensor_front_right");
DistanceSensor *r_leftPS   = robot->getDistanceSensor("sensor_rear_left");
DistanceSensor *r_rightPS  = robot->getDistanceSensor("sensor_rear_right");

Motor *leftMotor1 = robot->getMotor("Motor_1");
Motor *leftMotor2 = robot->getMotor("Motor_2");
Motor *rightMotor1 = robot->getMotor("Motor_3");
Motor *rightMotor2 = robot->getMotor("Motor_4");



void lineFollow(double &kp,double &kd,double &ki,double &baseSpeed){ 
  if ((leftIR->getValue() >= 400) && (rightIR->getValue() >= 400) && (midIR->getValue() < 400)){
      error = 0;
    } else if ((leftMIR->getValue() >= 400) &&(leftIR->getValue() >= 400) && (rightIR->getValue() < 400) && (midIR->getValue() >= 400) && (rightMIR->getValue() >= 400)){
      error = -1;
    } else if ((leftMIR->getValue() >= 400) &&(leftIR->getValue() < 400) && (rightIR->getValue() >= 400) && (midIR->getValue() >= 400) && (rightMIR->getValue() >= 400)){
      error = 1;
    } else if ((leftMIR->getValue() >= 400) && (leftIR->getValue() >= 400) && (rightIR->getValue() >= 400) && (midIR->getValue() >= 400) && (rightMIR->getValue() < 400)){
      error = -2;
    } else if ((leftMIR->getValue() < 400) && (leftIR->getValue() >= 400) && (rightIR->getValue() >=  400) && (midIR->getValue() >= 400)&& (rightMIR->getValue() >= 400)){
      error = 2;
    } else if ((leftMIR->getValue() < 400) && (leftIR->getValue() < 400) && (rightIR->getValue() >=  400) && (midIR->getValue() < 400)&& (rightMIR->getValue() >= 400)){
      error = 0;
    }
    P = error;
    std::cout<<"p = "<<P<<std::endl;
    I = error + I;
    std::cout<<"I = "<<I<<std::endl;
    D = error - last_error;
    std::cout<<"D = "<<D<<std::endl;
    double correction = (kp*P)+(ki*I)+(kd*D);
    std::cout<<"correction = "<<correction<<std::endl;
    last_error = error;
    std::cout<<"last_error = "<<last_error<<std::endl;
    
    left_speed = baseSpeed + correction;
    std::cout<<"left_speed = "<<left_speed<<std::endl;
    
    right_speed = baseSpeed - correction;
    std::cout<<"right_speed = "<<right_speed<<std::endl; 
     
  std::cout<<"left_Most= "<<leftMIR->getValue()<<" left = "<<leftIR->getValue()<< "  mid = "<<midIR->getValue()<<"  right = "<<rightIR->getValue()<<"  right_Most = "<<rightMIR->getValue()<<std::endl;
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
  
  return;
}


void wallFollow(double &kp,double &kd,double &ki,double &baseSpeed){
  std::cout << f_leftPS->getValue() << " " << r_leftPS->getValue() << " " << centerPS->getValue() << " " << f_rightPS->getValue() << " " << r_rightPS->getValue() << std::endl;
  if (centerPS->getValue() <900){
    left_speed=0;
    right_speed=0;

    if((f_rightPS->getValue())==(r_leftPS->getValue())){
      
      if((r_rightPS->getValue())>(r_leftPS->getValue())){
        left_speed=10.0;
        right_speed=-10.0;
      }
      
      if((r_rightPS->getValue())<(r_leftPS->getValue())){
        left_speed=-10.0;
        right_speed=10.0; 
      }
      
    }


    if((f_rightPS->getValue()+r_rightPS->getValue())>(f_leftPS->getValue()+r_leftPS->getValue())){
      left_speed=10.0;
      right_speed=-10.0;
    }
      
    else if((f_rightPS->getValue()+r_rightPS->getValue())<(f_leftPS->getValue()+r_leftPS->getValue())){
      left_speed=-10.0;
      right_speed=10.0;
    } 

  }

  else {
    if ((f_leftPS->getValue()+r_leftPS->getValue()+rightPS->getValue())==(f_rightPS->getValue()+r_rightPS->getValue()+leftPS->getValue())){
      left_speed=baseSpeed;
      right_speed=baseSpeed;

    }
  

    else if ((f_leftPS->getValue()+r_leftPS->getValue()+leftPS->getValue())<(f_rightPS->getValue()+r_rightPS->getValue()+rightPS->getValue())){
      if ((rightPS->getValue()==1000)&&(f_leftPS->getValue()==1000)){
        error1=(REFERENCE_DISTANCE-leftPS->getValue()*0.8)/30;   
      }

      else{
        error1=(REFERENCE_DISTANCE-(f_leftPS->getValue()+r_leftPS->getValue())/2)/30;
      }

      P_1=error1;
      I_1+=error1;
      D_1=(error1-last_error1);
      double correction1=(kp*P_1+ki*I_1+kd*D_1);
      last_error1=error1;  
            
      left_speed=baseSpeed+correction1;
      right_speed=baseSpeed-correction1;
    
    }

    else{
      if ((rightPS->getValue()==1000)&&(f_leftPS->getValue()==1000)){
        error2=(REFERENCE_DISTANCE-rightPS->getValue()*0.8)/30;
      }

      else{
        error2=(REFERENCE_DISTANCE-(f_rightPS->getValue()+r_rightPS->getValue())/2)/30;   
      }

      P_2=error2;
      I_2+=error2;
      D_2=error2-last_error2;
      double correction2=(kp*P_1+ki*I_1+kd*D_1);
      last_error2=error2;

      left_speed=baseSpeed-correction2;
      right_speed=baseSpeed+correction2;
    }

    leftMotor1->setVelocity(left_speed);
    rightMotor1->setVelocity(right_speed);
    leftMotor2->setVelocity(left_speed);
    rightMotor2->setVelocity(right_speed);

  }
}
   




int main(int argc, char **argv){
    int timeStep = (int)robot->getBasicTimeStep();
  
    leftMotor1->setPosition(INFINITY);
    leftMotor2->setPosition(INFINITY);
    rightMotor1->setPosition(INFINITY);
    rightMotor2->setPosition(INFINITY);
  

    leftMotor1->setVelocity(0.0);
    leftMotor2->setVelocity(0.0);
    rightMotor1->setVelocity(0.0);
    rightMotor2->setVelocity(0.0);

    leftIR->enable(TIME_STEP);
    midIR->enable(TIME_STEP);
    rightIR->enable(TIME_STEP);
    rightMIR->enable(TIME_STEP);
    leftMIR->enable(TIME_STEP);

    leftPS     ->enable(TIME_STEP);   
    centerPS   ->enable(TIME_STEP);
    rightPS    ->enable(TIME_STEP);
    f_leftPS   ->enable(TIME_STEP);
    f_rightPS  ->enable(TIME_STEP);
    r_leftPS   ->enable(TIME_STEP);
    r_rightPS  ->enable(TIME_STEP);



    double kp         = 1.71;          // line following parameters
    double kd         = 1.5;
    double ki         = 0.0;
    double baseSpeed  = 6.5;

    while (robot->step(timeStep) != -1){

        
        //lineFollow(kp,kd,ki,baseSpeed);
        wallFollow(kp,kd,ki,baseSpeed);

        

    }

    delete robot;
    return 0;


}

