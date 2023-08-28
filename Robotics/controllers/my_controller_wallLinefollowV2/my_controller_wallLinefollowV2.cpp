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
static bool redcolor= false;
static bool bluecolor= true;




static bool isTjunction=false;


static double kp         = 1.3;          // line following parameters
static double kd         = 1.9;
static double ki         = 0.0;
static double baseSpeed  = 6.5;
static double kpw         = 0.9;          // line following parameters
static double kdw         = 0.5;
static double kiw         = 0.02;


 


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

Motor *leftMotor1 = robot->getMotor("Motor_4");
Motor *leftMotor2 = robot->getMotor("Motor_3");
Motor *rightMotor1 = robot->getMotor("Motor_2");
Motor *rightMotor2 = robot->getMotor("Motor_1");

void speedcontrol(double error){
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




void lineFollow(double &kp,double &kd,double &ki,double &baseSpeed){ 
  if ((leftIR->getValue() >= 400) && (rightIR->getValue() >= 400) && (midIR->getValue() < 400)){
      error = 0;
    }  if ((leftMIR->getValue() >= 400) &&(leftIR->getValue() >= 400) && (rightIR->getValue() < 400) && (midIR->getValue() >= 400) && (rightMIR->getValue() >= 400)){
      error = 1;
    }  if ((leftMIR->getValue() >= 400) &&(leftIR->getValue() < 400) && (rightIR->getValue() >= 400) && (midIR->getValue() >= 400) && (rightMIR->getValue() >= 400)){
      error = -1;
    }  if ((leftMIR->getValue() >= 400) && (leftIR->getValue() >= 400) && (rightIR->getValue() >= 400) && (midIR->getValue() >= 400) && (rightMIR->getValue() < 400)){
      error = 2;
    }  if ((leftMIR->getValue() < 400) && (leftIR->getValue() >= 400) && (rightIR->getValue() >=  400) && (midIR->getValue() >= 400)&& (rightMIR->getValue() >= 400)){
      error = -2;
    }
    
    std::cout<<"left_MostIR= "<<leftMIR->getValue()<<" leftIR = "<<leftIR->getValue()<< "  midIR = "<<midIR->getValue()<<"  rightIR = "<<rightIR->getValue()<<"  right_MostIR = "<<rightMIR->getValue()<<std::endl;
    
    speedcontrol(error);
    return;
    
}


void bluelineFollow(double &kp,double &kd,double &ki,double &baseSpeed){ 
  if ((leftIR->getValue() > 800) && (rightIR->getValue() > 800) && (midIR->getValue() < 800) && (midIR->getValue() >= 500)){
      error = 0;
    }  if ((leftMIR->getValue() > 800) &&(leftIR->getValue() > 800) && (rightIR->getValue() < 800) && (rightIR->getValue() >= 500) && (midIR->getValue() < 800) && (midIR->getValue() >= 500) && (rightMIR->getValue() > 800)){
      error = 1;
    }  if ((leftMIR->getValue() > 800) && (leftIR->getValue() < 800) && (leftIR->getValue() >= 500) && (rightIR->getValue() > 800) && (midIR->getValue() > 800) && (rightMIR->getValue() > 800)){
      error = -1;
    }  if ((leftMIR->getValue() > 800) && (leftIR->getValue() > 800) && (rightIR->getValue() > 800) && (midIR->getValue() > 800) && (rightMIR->getValue() < 800) && (rightMIR->getValue() >= 500)){
      error = 2;
    }  if ((leftMIR->getValue() < 800) && (leftMIR->getValue() >= 500) && (leftIR->getValue() > 800) && (rightIR->getValue() > 800) && (midIR->getValue() > 800)&& (rightMIR->getValue() > 800)){
      error = -2;
    }
    
    std::cout<<"left_MostIR= "<<leftMIR->getValue()<<" leftIR = "<<leftIR->getValue()<< "  midIR = "<<midIR->getValue()<<"  rightIR = "<<rightIR->getValue()<<"  right_MostIR = "<<rightMIR->getValue()<<std::endl;
    
    speedcontrol(error);
    return;
    
}





void wallFollow(double &kp,double &kd,double &ki,double &baseSpeed){ 
  if ((leftPS->getValue() >= 800) && (rightPS->getValue() >= 800) && (centerPS->getValue() > 800)){
      error = 0;
    } if ((f_leftPS->getValue() >= 800) &&(leftPS->getValue() >= 800) && (rightPS->getValue() < 800) && (centerPS->getValue() >=800) && (f_rightPS->getValue() >= 800)){
      error = -1;
    } if ((f_leftPS->getValue() >= 800) &&(leftPS->getValue() < 800) && (rightPS->getValue() >= 800) && (centerPS->getValue() >= 800) && (f_rightPS->getValue() >= 800)){
      error = 1;
    } if ((f_leftPS->getValue() >= 800) && (leftPS->getValue() >= 800) && (rightPS->getValue() >= 800) && (centerPS->getValue() >= 800) && (f_rightPS->getValue() < 800)){
      error = -2;
    } if ((f_leftPS->getValue() < 800) && (leftPS->getValue() >= 800) && (rightPS->getValue() >=  800) && (centerPS->getValue() >= 800)&& (f_rightPS->getValue() >= 800)){
      error = 2;
    }
    
    std::cout<<"left_MostPS= "<<f_leftPS->getValue()<<" leftPS = "<<leftPS->getValue()<< "  midPS = "<<centerPS->getValue()<<"  rightPS = "<<rightPS->getValue()<<"  right_MostPS = "<<f_rightPS->getValue()<<std::endl; 
    
    speedcontrol(error);
    return;
}
   

void T_junctionFollow(double &kp,double &kd,double &ki,double &baseSpeed){
  if ((leftMIR->getValue() >= 400) &&(leftIR->getValue() >= 400) && (rightIR->getValue() < 400) && (midIR->getValue() < 400) && (rightMIR->getValue() < 400)){
      error =2;
      std::cout<<"a"<<std::endl;
    }  if ((leftMIR->getValue() < 400) &&(leftIR->getValue() < 400) && (rightIR->getValue() >= 400) && (midIR->getValue() < 400) && (rightMIR->getValue() >= 400)){
      error =-2;
      std::cout<<"b"<<std::endl;
    }  if ((leftMIR->getValue() >= 400) && (leftIR->getValue() < 400) && (rightIR->getValue() < 400) && (midIR->getValue() < 400) && (rightMIR->getValue() < 400)){
      error =2;
      std::cout<<"c"<<std::endl;
    }  if ((leftMIR->getValue() < 400) && (leftIR->getValue() < 400) && (rightIR->getValue() <  400) && (midIR->getValue() < 400)&& (rightMIR->getValue() >= 400)){
      error =-2;
      std::cout<<"d"<<std::endl;
    }
     
    speedcontrol(error);  
    std::cout<<"left_MostIR= "<<leftMIR->getValue()<<" leftIR = "<<leftIR->getValue()<< "  midIR = "<<midIR->getValue()<<"  rightIR = "<<rightIR->getValue()<<"  right_MostIR = "<<rightMIR->getValue()<<std::endl;

    return;
}

void T_junctionFollow2(double &kp,double &kd,double &ki,double &baseSpeed){
  if ((leftMIR->getValue() > 800) &&(leftIR->getValue() > 800) && (rightIR->getValue() < 800) && (rightIR->getValue() >= 500) && (midIR->getValue() < 800)  && (rightMIR->getValue() < 800) && (rightMIR->getValue() >= 500)){
      error =2;
      std::cout<<"a"<<std::endl;
    }  if ((leftMIR->getValue() < 800) && (leftMIR->getValue() >= 500) && (leftIR->getValue() < 800) && (leftIR->getValue() >= 500) && (rightIR->getValue() > 800) && (midIR->getValue() < 800) && (rightMIR->getValue() > 800) ){
      error =-2;
      std::cout<<"b"<<std::endl;
    }  if ((leftMIR->getValue() > 800) && (leftIR->getValue() < 800) && (leftIR->getValue() >= 500) && (rightIR->getValue() < 800) && (rightIR->getValue() >= 500) && (midIR->getValue() < 800) && (rightMIR->getValue() < 800) && (rightMIR->getValue() >= 500)){
      error =2;
      std::cout<<"c"<<std::endl;
    }  if ((leftMIR->getValue() < 800) && (leftMIR->getValue() >= 500) && (leftIR->getValue() < 800) &&  (leftIR->getValue() >= 500) && (rightIR->getValue() <  800) && (rightIR->getValue() >= 500) && (midIR->getValue() < 800)&& (rightMIR->getValue() > 800)){
      error =-2;
      std::cout<<"d"<<std::endl;
    }
     
    speedcontrol(error);  
    std::cout<<"left_MostIR= "<<leftMIR->getValue()<<" leftIR = "<<leftIR->getValue()<< "  midIR = "<<midIR->getValue()<<"  rightIR = "<<rightIR->getValue()<<"  right_MostIR = "<<rightMIR->getValue()<<std::endl;

    return;
}




void T_junctioncheck(double &kp,double &kd,double &ki,double &baseSpeed){

  if ((leftMIR->getValue() >= 400) &&(leftIR->getValue() >= 400) && (rightIR->getValue() < 400) && (midIR->getValue() < 400) && (rightMIR->getValue() < 400)){
     isTjunction=true; 
  }else if ((leftMIR->getValue() < 400) &&(leftIR->getValue() < 400) && (rightIR->getValue() >= 400) && (midIR->getValue() < 400) && (rightMIR->getValue() >= 400)){
      isTjunction=true; 
  }else if ((leftMIR->getValue() >= 400) && (leftIR->getValue() < 400) && (rightIR->getValue() < 400) && (midIR->getValue() < 400) && (rightMIR->getValue() < 400)){
      isTjunction=true; 
  }else if ((leftMIR->getValue() < 400) && (leftIR->getValue() < 400) && (rightIR->getValue() <  400) && (midIR->getValue() < 400)&& (rightMIR->getValue() >= 400)){
      isTjunction=true; 
  }else if ((leftMIR->getValue() >= 400) && (leftIR->getValue() >= 400) && (rightIR->getValue() >=  400) && (midIR->getValue() < 400)&& (rightMIR->getValue() >= 400)){
      isTjunction=false;
  }else if ((leftMIR->getValue() >= 400) && (leftIR->getValue() >= 400) && (rightIR->getValue() >=  400) && (midIR->getValue() >= 400)&& (rightMIR->getValue() < 400)){
      isTjunction=false;
  }else if ((leftMIR->getValue() < 400) && (leftIR->getValue() >= 400) && (rightIR->getValue() >=  400) && (midIR->getValue() >= 400)&& (rightMIR->getValue() >= 400)){
      isTjunction=false;
  }else if ((leftMIR->getValue() >= 400) && (leftIR->getValue() < 400) && (rightIR->getValue() >=  400) && (midIR->getValue() >= 400)&& (rightMIR->getValue() >= 400)){
      isTjunction=false;
  }else if ((leftMIR->getValue() >= 400) && (leftIR->getValue() >= 400) && (rightIR->getValue() <  400) && (midIR->getValue() >= 400)&& (rightMIR->getValue() >= 400)){
      isTjunction=false;
  }else if ((leftMIR->getValue() >= 400) && (leftIR->getValue() >= 400) && (rightIR->getValue() >=  400) && (midIR->getValue() >= 400)&& (rightMIR->getValue() >= 400)){
      isTjunction=false;
  }
    std::cout<<"T_junction: "<<isTjunction<<std::endl;
    return;


}

void T_junctioncheck2(double &kp,double &kd,double &ki,double &baseSpeed){

  if ((leftMIR->getValue() > 800) &&(leftIR->getValue() > 800) && (rightIR->getValue() < 800) && (rightIR->getValue() >= 500) && (midIR->getValue() < 400) && (rightMIR->getValue() < 800)  && (rightMIR->getValue() >= 500)){
     isTjunction=true; 
    }  if ((leftMIR->getValue() < 800) && (leftMIR->getValue() >= 500) && (leftIR->getValue() < 800) && (leftIR->getValue() >= 500) && (rightIR->getValue() > 800) && (midIR->getValue() < 400) && (rightMIR->getValue() > 800)){
      isTjunction=true; 
    }  if ((leftMIR->getValue() > 800) && (leftIR->getValue() < 800) && (leftIR->getValue() >= 500) && (rightIR->getValue() < 800) && (rightIR->getValue() >= 500) && (midIR->getValue() < 400) && (rightMIR->getValue() < 800) && (rightMIR->getValue() >= 500)){
      isTjunction=true; 
    }  if ((leftMIR->getValue() < 800) && (leftMIR->getValue() >= 500) && (leftIR->getValue() < 800) && (leftIR->getValue() >= 500) && (rightIR->getValue() <  800) && (midIR->getValue() < 400) && (midIR->getValue() >= 500)&& (rightMIR->getValue() > 800)){
      isTjunction=true; 
    } if ((leftMIR->getValue() > 800) && (leftIR->getValue() > 800) && (rightIR->getValue() > 800) && (midIR->getValue() < 400) && (rightMIR->getValue() > 800)){
      isTjunction=false;
    } if ((leftMIR->getValue() > 800) && (leftIR->getValue() > 800) && (rightIR->getValue() > 800) && (midIR->getValue() > 800) && (rightMIR->getValue() < 800) && (rightMIR->getValue() >= 500)){
      isTjunction=false;
    } if ((leftMIR->getValue() < 800) && (leftMIR->getValue() >= 500) && (leftIR->getValue() > 800) && (rightIR->getValue() > 800) && (midIR->getValue() > 800)&& (rightMIR->getValue() > 800)){
      isTjunction=false;
    } if ((leftMIR->getValue() > 800) && (leftIR->getValue() < 800) && (leftIR->getValue() >= 500) && (rightIR->getValue() > 800) && (midIR->getValue() > 800)&& (rightMIR->getValue() > 800)){
      isTjunction=false;
    } if ((leftMIR->getValue() > 800) && (leftIR->getValue() > 800) && (rightIR->getValue() <  800) && (rightIR->getValue() >=500) && (midIR->getValue() > 800)&& (rightMIR->getValue() > 800)){
      isTjunction=false;
    }
    std::cout<<"T_junction: "<<isTjunction<<std::endl;
    return;

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
    
    static bool scene1=false;
    static bool scene2=false;
    static bool scene3=false;
    static bool scene4=false;
    static bool scene5=false;
    static bool scene6=false;
    //static bool scene7=false;
    //static bool scene8=false;
    //static bool scene9=false;
    //static bool scene10=false;

    static bool iswhitebox=false;
    static bool isbluedetect=false;


    while (robot->step(timeStep) != -1){
    
    if (bluecolor==true){
    if ((scene4==false) && (scene5==false) && (leftMIR->getValue() >= 400) && (leftIR->getValue() >= 400) && (rightIR->getValue() >=  400) && (midIR->getValue() < 400)&& (rightMIR->getValue() >= 400)){
      scene1=true;
    } else{
      scene1=false;
    }
    if (scene1==true){
      lineFollow(kp,kd,ki,baseSpeed);
    } 
    
    
    if ((scene1==false) && (scene5==false) &&  ((leftMIR->getValue() >= 400) && (leftIR->getValue() >= 400) && (rightIR->getValue() >=  400) && (midIR->getValue() >= 400) && (rightMIR->getValue() >= 400))){
      scene2=true;
    } else{
      scene2=false;
    }
      
     if (scene2==true){
      wallFollow(kpw,kdw,kiw,baseSpeed);
    }
    
    if  ((scene1==false) && (scene2==false) && (scene4==false) && (scene5==false) && (iswhitebox==false) && (((leftMIR->getValue()) + (leftIR->getValue()) + (rightIR->getValue()) + (midIR->getValue()) + (rightMIR->getValue())) >= 4000)){
      scene3=true;     
    }
    
    if ((leftMIR->getValue() < 400) && (leftIR->getValue() < 400) && (rightIR->getValue() <  400) && (midIR->getValue() < 400)&& (rightMIR->getValue() < 400)){
          iswhitebox=true;
      }
    if((((leftMIR->getValue()) + (leftIR->getValue()) + (rightIR->getValue()) + (midIR->getValue()) + (rightMIR->getValue())) > 4000 ) && (iswhitebox==true)){ 
       scene3=false;
    }
    
    if (scene3==true){
    
    leftMotor1->setVelocity(1.0);
    leftMotor2->setVelocity(1.0);
    rightMotor1->setVelocity(1.0);
    rightMotor2->setVelocity(1.0);
      
    }
    
    if (((scene1==false) && (scene2==false) && (iswhitebox==true) &&  (scene3==false)) && ((leftMIR->getValue()) + (leftIR->getValue()) + (rightIR->getValue()) + (midIR->getValue()) + (rightMIR->getValue())) >= 4000 ){
      scene4=true;
    }
    else {
      scene4= false;
      }
    
    if (scene4==true){
    
      lineFollow(kp,kd,ki,baseSpeed);
              
    }
    
    if ((scene4==true) && (scene1==false) && (scene2==false)  && (scene3==false) && ((leftMIR->getValue()) + (leftIR->getValue()) + (rightIR->getValue()) + (midIR->getValue()) + (rightMIR->getValue())) >= 2500 ){
      scene5=true;
    }
    
    if( scene5==true){
      T_junctioncheck2(kp,kd,ki,baseSpeed);
      if (isTjunction==false){
        bluelineFollow(kp,kd,ki,baseSpeed);
      } else {
        T_junctionFollow2(kp,kd,ki,baseSpeed);
      }
    
    }
    
    if ((midIR->getValue() < 800) && (midIR->getValue() >= 500)){
    
      isbluedetect=true;
    
    }
    
    if ((scene5==true) && (midIR->getValue() < 400) && (isbluedetect==true)){
      scene6=true;
    }
    
    if (scene6==true){
    
     T_junctioncheck(kp,kd,ki,baseSpeed);
      if (isTjunction==false){
        lineFollow(kp,kd,ki,baseSpeed);
      } else {
        T_junctionFollow(kp,kd,ki,baseSpeed);
      }
      
    
    }
    
    
    
        
    
    std::cout<<"scene1: "<<scene1<<std::endl;
    std::cout<<"scene2: "<<scene2<<std::endl;
    std::cout<<"scene3: "<<scene3<<std::endl;
    std::cout<<"scene4: "<<scene4<<std::endl;
    std::cout<<"scene5: "<<scene5<<std::endl;
    std::cout<<"scene6: "<<scene6<<std::endl;
    std::cout<<"whitebix "<<iswhitebox<<std::endl;
    std::cout<<"isblue: "<<isbluedetect<<std::endl;
    std::cout<<"left_MostIR= "<<leftMIR->getValue()<<" leftIR = "<<leftIR->getValue()<< "  midIR = "<<midIR->getValue()<<"  rightIR = "<<rightIR->getValue()<<"  right_MostIR = "<<rightMIR->getValue()<<std::endl;

    
      
          
               
        //lineFollow(kp,kd,ki,baseSpeed);
        //wallFollow(kp,kd,ki,baseSpeed);
      
        
    }
    
    if (redcolor==true){
    if ((scene4==false) && (scene5==false) && (leftMIR->getValue() >= 400) && (leftIR->getValue() >= 400) && (rightIR->getValue() >=  400) && (midIR->getValue() < 400)&& (rightMIR->getValue() >= 400)){
      scene1=true;
    } else{
      scene1=false;
    }
    if (scene1==true){
      lineFollow(kp,kd,ki,baseSpeed);
    } 
    
    
    if ((scene1==false) && (scene5==false) &&  ((leftMIR->getValue() >= 400) && (leftIR->getValue() >= 400) && (rightIR->getValue() >=  400) && (midIR->getValue() >= 400) && (rightMIR->getValue() >= 400))){
      scene2=true;
    } else{
      scene2=false;
    }
      
     if (scene2==true){
      wallFollow(kpw,kdw,kiw,baseSpeed);
    }
    
    if  ((scene1==false) && (scene2==false) && (scene4==false) && (scene5==false) && (iswhitebox==false) && (((leftMIR->getValue()) + (leftIR->getValue()) + (rightIR->getValue()) + (midIR->getValue()) + (rightMIR->getValue())) >= 4000)){
      scene3=true;     
    }
    
    if ((leftMIR->getValue() < 400) && (leftIR->getValue() < 400) && (rightIR->getValue() <  400) && (midIR->getValue() < 400)&& (rightMIR->getValue() < 400)){
          iswhitebox=true;
      }
    if((((leftMIR->getValue()) + (leftIR->getValue()) + (rightIR->getValue()) + (midIR->getValue()) + (rightMIR->getValue())) > 4000 ) && (iswhitebox==true)){ 
       scene3=false;
    }
    
    if (scene3==true){
    
    leftMotor1->setVelocity(1.0);
    leftMotor2->setVelocity(1.0);
    rightMotor1->setVelocity(1.0);
    rightMotor2->setVelocity(1.0);
      
    }
    
    if (((scene1==false) && (scene2==false) && (iswhitebox==true) &&  (scene3==false)) && ((leftMIR->getValue()) + (leftIR->getValue()) + (rightIR->getValue()) + (midIR->getValue()) + (rightMIR->getValue())) >= 4000 ){
      scene4=true;
    }
    else {
      scene4= false;
      }
    
    if (scene4==true){
    
      lineFollow(kp,kd,ki,baseSpeed);
              
    }
    
    if ((scene4==true) && (scene1==false) && (scene2==false)  && (scene3==false) && ((leftMIR->getValue()) + (leftIR->getValue()) + (rightIR->getValue()) + (midIR->getValue()) + (rightMIR->getValue())) >= 2500 ){
      scene5=true;
    }
    
    if( scene5==true){
      T_junctioncheck(kp,kd,ki,baseSpeed);
      if (isTjunction==false){
        lineFollow(kp,kd,ki,baseSpeed);
      } else {
        T_junctionFollow(kp,kd,ki,baseSpeed);
      }
    
    }
    
    
    
    
    
        
    
    std::cout<<"scene1: "<<scene1<<std::endl;
    std::cout<<"scene2: "<<scene2<<std::endl;
    std::cout<<"scene3: "<<scene3<<std::endl;
    std::cout<<"scene4: "<<scene4<<std::endl;
    std::cout<<"scene5: "<<scene5<<std::endl;
    std::cout<<"scene6: "<<scene6<<std::endl;
    std::cout<<"whitebix "<<iswhitebox<<std::endl;
    std::cout<<"isblue: "<<isbluedetect<<std::endl;
    std::cout<<"left_MostIR= "<<leftMIR->getValue()<<" leftIR = "<<leftIR->getValue()<< "  midIR = "<<midIR->getValue()<<"  rightIR = "<<rightIR->getValue()<<"  right_MostIR = "<<rightMIR->getValue()<<std::endl;

    
      
          
               
        //lineFollow(kp,kd,ki,baseSpeed);
        //wallFollow(kp,kd,ki,baseSpeed);
      
        
    }
    
    
    }

    delete robot;
    return 0;


}

