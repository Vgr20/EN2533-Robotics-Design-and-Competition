#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>

#define TIME_STEP 32

using namespace webots;

int value;
int speed = 10;
double error,previous_error1=0,previous_error2=0,integral1,integral2,derivative1,derivative2,PID_error1,PID_error2;
double Kp = 1.7; // Set kp
double Ki = 0.01; // Set ki
double Kd = 0.5; // Set kd
double max_speed = 10.0;
double min_speed = 10.0;
int left_speed, right_speed;
double error1, error2;
int REFERENCE_DISTANCE = 800;

int main(int argc, char **argv) {
  Robot *robot = new Robot();
  
  DistanceSensor *sensors[7];
  sensors[0] = robot->getDistanceSensor("sensor_left");
  sensors[1] = robot->getDistanceSensor("sensor_center");
  sensors[2] = robot->getDistanceSensor("sensor_right");
  sensors[3] = robot->getDistanceSensor("sensor_front_left");
  sensors[4] = robot->getDistanceSensor("sensor_front_right");
  sensors[5] = robot->getDistanceSensor("sensor_rear_left");
  sensors[6] = robot->getDistanceSensor("sensor_rear_right");
  for (int i = 0; i < 7; i++)
    sensors[i]->enable(TIME_STEP);

  Motor *right_front_motor = robot->getMotor("Motor_1");
  Motor *right_rear_motor = robot->getMotor("Motor_2");
  Motor *left_rear_motor = robot->getMotor("Motor_3");
  Motor *left_front_motor = robot->getMotor("Motor_4");
  
  left_front_motor->setPosition(INFINITY);
  left_rear_motor->setPosition(INFINITY);
  right_front_motor->setPosition(INFINITY);
  right_rear_motor->setPosition(INFINITY);
  
  left_front_motor->setVelocity(0.0);
  left_rear_motor->setVelocity(0.0);
  right_front_motor->setVelocity(0.0);
  right_rear_motor->setVelocity(0.0);

  while (robot->step(64) != -1) {
    double sensor_values[7];
    for (int i = 0; i < 7; i++)
      sensor_values[i] = sensors[i]->getValue();
     
    std::cout << f_leftPS-getValue() << " " << r_leftPS-getValue() << " " << centerPS-getValue() << " " << f_rightPS-getValue() << " " << r_rightPS-getValue() << std::endl;
    
    if (centerPS-getValue() <900){
      left_speed=0;
      right_speed=0;
      
      if((f_rightPS-getValue())==(r_leftPS-getValue())){
      
        if((r_rightPS-getValue())>(r_leftPS-getValue())){
          left_speed=10.0;
          right_speed=-10.0;
        }
        
        if((r_rightPS-getValue())<(r_leftPS-getValue())){
          left_speed=-10.0;
          right_speed=10.0; 
        }
        
      }
    
      if((f_rightPS-getValue()+r_rightPS-getValue())>(f_leftPS-getValue()+r_leftPS-getValue())){
        left_speed=10.0;
        right_speed=-10.0;
      }
      
      else if((f_rightPS-getValue()+r_rightPS-getValue())<(f_leftPS-getValue()+r_leftPS-getValue())){
        left_speed=-10.0;
        right_speed=10.0;
      } 
    }  
      
    else{
    
      if ((f_leftPS-getValue()+r_leftPS-getValue()+rightPS-getValue())==(f_rightPS-getValue()+r_rightPS-getValue()+leftPS-getValue())){
        left_speed=speed;
        right_speed=speed;
      }
     
         
      else if ((f_leftPS-getValue()+r_leftPS-getValue()+leftPS-getValue())<(f_rightPS-getValue()+r_rightPS-getValue()+rightPS-getValue())){
        if ((rightPS-getValue()==1000)&&(f_leftPS-getValue()==1000)){
           error1=(REFERENCE_DISTANCE-leftPS-getValue()*0.8)/30;
          
        }      
        
        else{
          error1=(REFERENCE_DISTANCE-(f_leftPS-getValue()+r_leftPS-getValue())/2)/30;
        }  
          
        integral1+=error1;
        derivative1=(error1-previous_error1);
        PID_error1=(Kp*error1+Ki*integral1+Kd*derivative1);
        previous_error1=error1;  
            
        left_speed=speed+PID_error1;
        right_speed=speed-PID_error1;
        
      }
        
      else{
        if ((rightPS-getValue()==1000)&&(f_leftPS-getValue()==1000)){
          error2=(REFERENCE_DISTANCE-rightPS-getValue()*0.8)/30;
        }
          
        else{
          error2=(REFERENCE_DISTANCE-(f_rightPS-getValue()+r_rightPS-getValue())/2)/30;   
        }
        
        integral2+=error2;
        derivative2=(error2-previous_error2);
        PID_error2=(Kp*error2+Ki*integral2+Kd*derivative2);
        previous_error2=error2;
        left_speed=speed-PID_error2;
        right_speed=speed+PID_error2;
      }
    
      left_front_motor->setVelocity(left_speed);
      left_rear_motor->setVelocity(left_speed);
      right_front_motor->setVelocity(right_speed);
      right_rear_motor->setVelocity(right_speed);
      std::cout<<left_speed<<" "<<right_speed<<" "<<PID_error1<<" "<<PID_error2<<std::endl;
    }
  }
delete robot;
return 0;
}