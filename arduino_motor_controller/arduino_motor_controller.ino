// PID Parameters (Untested settings) ajust acordingly
const double PID_left_param[] = { 0.1, 0.1, 0.1 }; //Respectively Kp, Ki and Kd for left motor PID
const double PID_right_param[] = { 0.1, 0.1, 0.1 }; //Respectively Kp, Ki and Kd for right motor PID

#include <Wire.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

//initializing all the variables
#define LOOPTIME                      100     //Looptime in millisecond
const byte noCommLoopMax = 255;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter

char log_msg[50];
char result[8];
const byte sampleTime = 95;
const byte RencoderA = 2;               //A channel for encoder of left motor
const byte RencoderB = 4;               //B channel for encoder of left motor

const byte LencoderA = 3;              //A channel for encoder of right motor
const byte LencoderB = 5;              //B channel for encoder of right motor

const byte motorpins[6] = {11,13,12,8,7,10}; //Motor pins for motor for motor driver
const byte RMotorFpin = motorpins[1];        //INT 1
const byte RMotorBpin = motorpins[2];        //INT 2
const byte LMotorFpin = motorpins[3];        //INT 3
const byte LMotorBpin = motorpins[4];        //INT 4
const byte RPWMpin = motorpins[0];           //ENA
const byte LPWMpin = motorpins[5];           //ENB

unsigned long lastMilli = 0;

const double radius = 0.068;              //Wheel radius, in m
const double track = 0.272;               //track(distance between driving wheels), in m
const int pulses_per_revolution = 663;    // encoder pulses per shaft revolution

double speed_req = 0;                         //Desired linear speed for the robot, in m/s
double angular_speed_req = 0;                 //Desired angular speed for the robot, in rad/s

double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double PWM_leftMotor = 0;                    //PWM command for left motor

double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
double PWM_rightMotor = 0;                   //PWM command for right motor

volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position

PID PID_leftMotor(&speed_act_left, &PWM_leftMotor, &speed_req_left, PID_left_param[0], PID_left_param[1], PID_left_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_rightMotor(&speed_act_right, &PWM_rightMotor, &speed_req_right, PID_right_param[0], PID_right_param[1], PID_right_param[2], DIRECT);   //Setting up the PID for right motor


ros::NodeHandle nh;

//function that will be called when receiving command from host
void handle_cmd (const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication

  speed_req = cmd_vel.linear.x;                                     //Extract the commanded linear speed from the message

  angular_speed_req = cmd_vel.angular.z;                            //Extract the commanded angular speed from the message

  speed_req_left = speed_req - angular_speed_req*(track/2);     //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
  speed_req_right = speed_req + angular_speed_req*(track/2);    //Calculate the required speed for the right motor to comply with commanded linear and angular speeds
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel", handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type

//__________________________________________________________________________

void setup() {


  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic

  MotorInit();                              //setting motor pins

  SetupPID();                               //setting PID parameters

  encoderSetup();                           //setup encoders

}

//_________________________________________________________________________

void loop() {
  nh.spinOnce();
  if((millis()-lastMilli) >= LOOPTIME)
  {                                                                           // enter timed loop
    lastMilli = millis();

    if (abs(pos_left) < 5){                                                   //Avoid taking in account small disturbances
      speed_act_left = 0;
    }
    else {
      speed_act_left=((pos_left/pulses_per_revolution)*2*PI)*(1000/LOOPTIME)*radius;           // calculate speed of left wheel
    }

    if (abs(pos_right) < 5){                                                  //Avoid taking in account small disturbances
      speed_act_right = 0;
    }
    else {
    speed_act_right=((pos_right/pulses_per_revolution)*2*PI)*(1000/LOOPTIME)*radius;          // calculate speed of right wheel
    }

    pos_left = 0;
    pos_right = 0;
    PID_leftMotor.Compute();                                                 // compute PWM value for left motor

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      LMotorStop();
    }
    else if (speed_req_left == 0){                        //Stopping
      LMotorStop();
    }
    else if (speed_req_left > 0){                          //Going forward
      LMotorF(PWM_leftMotor);
    }
    else {                                               //Going backward
      LMotorB(PWM_leftMotor);
    }

    PID_rightMotor.Compute();                                                 // compute PWM value for right motor

    if (noCommLoops >= noCommLoopMax) {                   //Stopping if too much time without command
      RMotorStop();
    }
    else if (speed_req_right == 0){                       //Stopping
      RMotorStop();
    }
    else if (PWM_rightMotor > 0){                         //Going forward
      RMotorF(PWM_rightMotor);
    }
    else {                                                //Going backward
      RMotorB(PWM_rightMotor);
    }

    if((millis()-lastMilli) >= LOOPTIME){         //write an error if execution time of the loop in longer than the specified looptime
      Serial.println(" TOO LONG ");
    }

    noCommLoops++;
    if (noCommLoops == 65535){
      noCommLoops = noCommLoopMax;
    }

  }
 }

//Left motor encoder counter
void encoderLeftMotor() {
  if (digitalRead(LencoderA) == digitalRead(LencoderB)) pos_left++;
  else pos_left--;
}

//Right motor encoder counter
void encoderRightMotor() {
  if (digitalRead(RencoderA) == digitalRead(RencoderB)) pos_right--;
  else pos_right++;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
void encoderSetup(){
  // Define the rotary encoder for left motor
  pinMode(LencoderA, INPUT);
  pinMode(LencoderB, INPUT);
  attachInterrupt(digitalPinToInterrupt(LencoderA), encoderLeftMotor, RISING);

  // Define the rotary encoder for right motor
  pinMode(RencoderA, INPUT);
  pinMode(RencoderB, INPUT);
  attachInterrupt(digitalPinToInterrupt(RencoderA), encoderRightMotor, RISING);
}
void SetupPID(){
  PID_leftMotor.SetSampleTime(sampleTime);
  PID_rightMotor.SetSampleTime(sampleTime);
  PID_leftMotor.SetOutputLimits(75, 255);
  PID_rightMotor.SetOutputLimits(75, 255);
  PID_leftMotor.SetMode(AUTOMATIC);           //turn on PID left motor
  PID_rightMotor.SetMode(AUTOMATIC);          //turn on PID right motor
}
void MotorInit(){
  for(byte i=0; i<6;i++)
  pinMode(motorpins[i],OUTPUT);
  STOP();
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
void STOP(){
  RMotorStop();
  LMotorStop();
}
