//Code Authors:
//*Caio Maia - caiomaia3@gmail.com
#include <Arduino.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

#define LEFT_MOTOR_PWM_PIN_1 5 
#define LEFT_MOTOR_PWM_PIN_2 6 

#define RIGHT_MOTOR_PWM_PIN_1 10 
#define RIGHT_MOTOR_PWM_PIN_2 11


#define INV_MOTOR_CRTL_A 7 // Invert rotation control
#define INV_MOTOR_CRTL_B 8

#define test_velocity 254 

double w_r=0, w_l=0;
//wheel_rad is the wheel radius ,wheel_sep is
double wheel_rad = 0.0325, wheel_sep = 0.295;
ros::NodeHandle nh;
int lowSpeed = 200;
int highSpeed = 50;
double speed_ang=0, speed_lin=0;
void messageCb( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z;
  speed_lin = msg.linear.x;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
void Motor(int Pulse_Width1, int motorPinInput1, int motorPinInput2);

void motors_init();

void setup(){
    motors_init();
}
void loop(){
    Motor(test_velocity,LEFT_MOTOR_PWM_PIN_1,LEFT_MOTOR_PWM_PIN_2);
    Motor(test_velocity,RIGHT_MOTOR_PWM_PIN_1,RIGHT_MOTOR_PWM_PIN_2);
    delay(5000);
    Motor(-test_velocity,LEFT_MOTOR_PWM_PIN_1,LEFT_MOTOR_PWM_PIN_2);
    Motor(-test_velocity,RIGHT_MOTOR_PWM_PIN_1,RIGHT_MOTOR_PWM_PIN_2);
    delay(5000);
}

void motors_init(){
    pinMode(LEFT_MOTOR_PWM_PIN_1,OUTPUT);
    pinMode(LEFT_MOTOR_PWM_PIN_2,OUTPUT);
    pinMode(RIGHT_MOTOR_PWM_PIN_1,OUTPUT);
    pinMode(RIGHT_MOTOR_PWM_PIN_2,OUTPUT);

    digitalWrite(LEFT_MOTOR_PWM_PIN_1,LOW);
    digitalWrite(LEFT_MOTOR_PWM_PIN_2,LOW);
    digitalWrite(LEFT_MOTOR_PWM_PIN_1,LOW);
    digitalWrite(LEFT_MOTOR_PWM_PIN_2,LOW);
}


void Motor(int Pulse_Width1, int motorPinInput1, int motorPinInput2){
 if (Pulse_Width1 > 0){
    analogWrite(motorPinInput1,Pulse_Width1);
    analogWrite(motorPinInput2,LOW);
 }
 if (Pulse_Width1 < 0){
    Pulse_Width1=abs(Pulse_Width1);
    analogWrite(motorPinInput1,LOW);
    analogWrite(motorPinInput2,Pulse_Width1);
 }
  if (Pulse_Width1 == 0){
     digitalWrite(motorPinInput1, LOW);
     digitalWrite(motorPinInput2, LOW);
 }
}