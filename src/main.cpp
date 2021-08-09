// autores
// Mateus Seixas - mateus_seixas@hotmail.com.br
// Caio Maia - caiomaia3@gmail.com

#include <HCSR04.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <Arduino.h>
#include <geometry_msgs/Twist.h>

#define LEFT_MOTOR_PWM_PIN_1 5 // MOTOR A
#define LEFT_MOTOR_PWM_PIN_2 6 // MOTOR A
#define RIGHT_MOTOR_PWM_PIN_1 11 // MOTOR B
#define RIGHT_MOTOR_PWM_PIN_2 10 // MOTOR B

double w_r=0, w_l=0;
double wheel_rad = 0.07/5, wheel_sep = 0.17; // RAIO E SEPARACAO DAS RODAS
ros::NodeHandle nh;
std_msgs::Float64 distancia;
ros::Publisher pub_dist("distancia", &distancia);

double speed_ang=0, speed_lin=0;
void messageCb( const geometry_msgs::Twist& msg){
  speed_ang = msg.angular.z * 80;
  speed_lin = msg.linear.x * 80;
  w_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
  w_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );
void Motor(int Pulse_Width1, int motorPinInput1, int motorPinInput2);

void motors_init();

UltraSonicDistanceSensor distanceSensor(13, 12);  // TRIGGER,ECHO

const long PERIODO = 500;
unsigned long tempoAnterior = 0;

void setup () {
    motors_init();
    nh.initNode();
    nh.advertise(pub_dist);
    nh.subscribe(sub);
}

void loop () {
    unsigned long tempoAtual = millis();
    if (tempoAtual - tempoAnterior >= PERIODO) {
      tempoAnterior = tempoAtual;
      distancia.data = distanceSensor.measureDistanceCm();
      pub_dist.publish(&distancia);
    } 
    
    if (abs(w_l)<15)
    {
        w_l = 0;
    }
    if (abs(w_r)<15)
    {
        w_r = 0;
    }
    if (w_l>25) // Consertar o sinal, para contemplar os valores negativos.
    {
        w_l = 25;    
    }
    if (w_r>25)
    {
        w_r = 25;    
    }
    if (w_l<-25) // Consertar o sinal, para contemplar os valores negativos.
    {
        w_l = -25;    
    }
    if (w_r<-25)
    {
        w_r = -25;    
    }
    
    Motor(w_l*10,LEFT_MOTOR_PWM_PIN_1,LEFT_MOTOR_PWM_PIN_2);
    Motor(w_r*10,RIGHT_MOTOR_PWM_PIN_1,RIGHT_MOTOR_PWM_PIN_2);
    
    nh.spinOnce();
    
    // Every 500 miliseconds, do a measurement using the sensor and print the distance in centimeters.
    //Serial.println(distanceSensor.measureDistanceCm());
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