/* 
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */
const byte motorpins[6] = {11,13,12,8,7,10}; //Motor pins for motor for motor driver
const byte RMotorFpin = motorpins[1];        //INT 1
const byte RMotorBpin = motorpins[2];        //INT 2
const byte LMotorFpin = motorpins[3];        //INT 3
const byte LMotorBpin = motorpins[4];        //INT 4
const byte RPWMpin = motorpins[0];           //ENA
const byte LPWMpin = motorpins[5];           //ENB
const byte encoderpinsB[2] = {4,5};          // Encoder B pins
const byte RencoderA = 2;                    // Interrupt pin Right encoder
const byte RencoderB = encoderpinsB[0];      // Right B pin
const byte LencoderA = 3;                    // Interrupt pin Left encoder
const byte LencoderB = encoderpinsB[1];      // Left B pin
const byte Rledpin = 9;
const byte Gledpin = 6;
const byte diameter = 136;                   // wheel diameter in mm
const int pulses_per_revolution = 663;       // encoder pulses per shaft revolution
const int loops = 50;
long pulses[2];
byte last;
byte PWM = 100;
long Rdistancemm;
long Ldistancemm;
long Avgdistancemm;
byte direction;
byte D = 0;
bool human;
bool go = true;
int counter = 0;
#include <ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

ros::NodeHandle  nh;
std_msgs::Float32 dist_msg;
ros::Publisher distance("distance", &dist_msg);
void changeLED( const std_msgs::Bool& hmn_msg){
  human = hmn_msg.data;
  if (human == true){
  digitalWrite(Gledpin , HIGH);
  digitalWrite(Rledpin , LOW);
  }
  else{
    digitalWrite(Gledpin , LOW);
    digitalWrite(Rledpin , HIGH);
 }
}
void changeDir( const std_msgs::UInt8& cmd_msg){
  direction = cmd_msg.data; 
 }
void changePWM( const std_msgs::UInt8& pwm_msg){
  PWM = pwm_msg.data;
  dist_msg.data = PWM;
  distance.publish( &dist_msg );
}

ros::Subscriber<std_msgs::UInt8> dir("dir", &changeDir );
ros::Subscriber<std_msgs::UInt8> pwm("PWM", &changePWM );
ros::Subscriber<std_msgs::Bool> humancheck("humancheck", &changeLED );

void setup()
{ 
   REncoderInit();
  LEncoderInit();
  MotorInit();
  LEDInit();
  nh.initNode();
  nh.subscribe(dir);
  nh.subscribe(pwm);
  nh.subscribe(humancheck);
  nh.advertise(distance);
}

void loop()
{  
  nh.spinOnce();
  switch (direction) {
    case 0:
      STOP();
      last = 0;
      break;
    case 1:
      FOWARD();
      if(last != 1){
        reset_counter();
        D = 0;
        }
       last = 1;
       break;
    case 2:
       BACK();
       if(last != 2){
        reset_counter();
        D = 0;
        }
       last = 2;
       break;
    case 3:
      if (go){
        RIGHT();
      }
      else{
        STOP();
        delay(5);
      }
      if (counter == loops){
        go = !go;
        counter = 0;
       }
       counter=counter + 1;
       last = 3;
        break;
    case 4:
        if (go){
        LEFT();
      }
      else{
        STOP();
        delay(5);
      }
      if (counter == loops){
        go = !go;
        counter = 0;
       }
       counter=counter + 1;
       last = 3;
        break;
    case 5:
        ORIGHT();
        if(last != 5){
        reset_counter();
        D = 0;
        }
       counter++;
       last = 5;
        break;
    case 6: 
        OLEFT();
        if(last != 6){
        reset_counter();
        D = 0;
        }
       last = 6;
        break;   
    default:
        STOP();
        last = 0;
        reset_counter();
        D = 0;
        break;
    }
  noInterrupts();
  Rdistancemm = pulses[0];
  Ldistancemm = pulses[1];
  interrupts();
  
  dist_msg.data = Rdistancemm;
  distance.publish( &dist_msg );
  dist_msg.data = Ldistancemm;
  distance.publish( &dist_msg );
  reset_counter();
  delay(10);
}
void MotorInit(){
  for(byte i=0; i<6;i++)
  pinMode(motorpins[i],OUTPUT);
}
void RMotorF(byte PWM){
  digitalWrite(RMotorFpin,HIGH);
  digitalWrite(RMotorBpin,LOW);
  analogWrite(RPWMpin, PWM);
}
void LMotorF(byte PWM){
  digitalWrite(LMotorFpin,HIGH);
  digitalWrite(LMotorBpin,LOW);
  analogWrite(LPWMpin, PWM);
}
void RMotorB(byte PWM){
  digitalWrite(RMotorBpin,HIGH);
  digitalWrite(RMotorFpin,LOW);
  analogWrite(RPWMpin, PWM);
}
void LMotorB(byte PWM){
  digitalWrite(LMotorBpin,HIGH);
  digitalWrite(LMotorFpin,LOW);
  analogWrite(LPWMpin, PWM);
}
void RMotorStop(){
  digitalWrite(RMotorFpin,LOW);
  digitalWrite(RMotorBpin,LOW);
  analogWrite(RPWMpin, 0);
}
void LMotorStop(){
  digitalWrite(LMotorFpin,LOW);
  digitalWrite(LMotorBpin,LOW);
  analogWrite(LPWMpin, 0);
}
void FOWARD(){
  RMotorF(PWM-D);
  LMotorF(PWM);
  if (pulses[0] > pulses [1]) D++;
  else if(pulses [1] > pulses[0]) D--;
}
void BACK(){
  RMotorB(PWM-D);
  LMotorB(PWM);
  if (pulses[0] < pulses [1]) D++;
  else if(pulses [1] < pulses[0]) D--;
}
void STOP(){
  RMotorStop();
  LMotorStop();
}
void LEFT(){
  RMotorB(65);
  LMotorF(0);
  if (-pulses[0] > pulses [1]) D++;
  else if(pulses [1] > -pulses[0]) D--;
  }
void RIGHT(){
    RMotorF(65);
    LMotorB(0);
    if (pulses[0] > -pulses [1]) D++;
    else if(-pulses [1] > pulses[0]) D--;
}
void ORIGHT(){
    RMotorF(PWM-D);
    LMotorB(PWM);
    if (pulses[0] > -pulses [1]) D++;
    else if(-pulses [1] > pulses[0]) D--;
}
void OLEFT(){
  RMotorB(PWM-D);
  LMotorF(PWM);
  if (-pulses[0] > pulses [1]) D++;
  else if(pulses [1] > -pulses[0]) D--;
  }
void reset_counter()
{
  for(byte i=0; i<2;i++){
    pulses[i] = 0;
  }
}
void REncoderInit(){
  pinMode(RencoderB,INPUT);
  attachInterrupt(digitalPinToInterrupt(RencoderA), Rpulsecounter, RISING);
}
void LEncoderInit(){
  pinMode(LencoderB,INPUT);
  attachInterrupt(digitalPinToInterrupt(LencoderA), Lpulsecounter, RISING);
}
void Rpulsecounter()
{
    int val = digitalRead(encoderpinsB[0]);
    if(val == LOW)pulses[0]++;
    else if(val == HIGH) pulses[0]--;
}
void Lpulsecounter()
{
    int val = digitalRead(encoderpinsB[1]);
    if(val == LOW)pulses[1]--;
    else if(val == HIGH) pulses[1]++;
}
void LEDInit(){
  pinMode(Gledpin,OUTPUT);
  pinMode(Rledpin,OUTPUT); 
}
