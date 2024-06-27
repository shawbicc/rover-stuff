/*

JOYSTICK SUPPORT FOR PROCHESTA ARM
------------------------------------------------

AVAILABLE CONTROLS -> Base, Shoulder, Elbow 

CONTROL SCHEME:
LB/RB -> BASE LEFT/RIGHT
Right stick UP/DOWN -> SHOULDER UP/DOWN
Y/A -> ELBOW UP/DOWN

*/

// libraries
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
ros::NodeHandle nh;

// pin definition
#define base_pin_a 8 // pwm 
#define base_pin_b 7 // pwm
#define base_speed 250

#define elbow_pin_a 3 // pwm
#define elbow_pin_b 5// pwm 
#define elbow_speed 250


#define shoulder_pin_a 10 // pwm
#define shoulder_pin_b 11 // pwm
#define shoulder_speed 250

// variables for joint directions
int BASE_DIR = 0;
int SHOULDER_DIR = 0;
int ELBOW_DIR = 0;

// callback for subscriber: update the joint directions
void jointAnglesCallback(const std_msgs::Int32MultiArray& msg) {
  BASE_DIR = msg.data[0];
  SHOULDER_DIR = msg.data[1];
  ELBOW_DIR = msg.data[2];
}

// ROS subscriber
ros::Subscriber<std_msgs::Int32MultiArray> sub("joint_angles", jointAnglesCallback);

// function for actuating joints according to directions received from ROS
void actuate_joints() {

  if(BASE_DIR==0 && SHOULDER_DIR==0 && ELBOW_DIR==0){
    analogWrite(base_pin_a, 0);
    analogWrite(base_pin_b, 0);

    analogWrite(shoulder_pin_a, 0);
    analogWrite(shoulder_pin_b, 0);

    analogWrite(elbow_pin_a, 0);
    analogWrite(elbow_pin_b, 0);

    Serial.println("all stopped");
  }

  if(BASE_DIR == 1){
    analogWrite(base_pin_a, base_speed);
    analogWrite(base_pin_b, 0);
    Serial.println("base right");
  }  
  else if(BASE_DIR == -1){
    analogWrite(base_pin_a, 0);
    analogWrite(base_pin_b, base_speed);
    Serial.println("base left");
  }
  else{
    analogWrite(base_pin_a, 0);
    analogWrite(base_pin_b, 0);
    Serial.println("base stop");
  }

  if(SHOULDER_DIR == 1){
    analogWrite(shoulder_pin_a, shoulder_speed);
    analogWrite(shoulder_pin_b, 0);
    Serial.println("shoulder right");
  }
  else if(SHOULDER_DIR == -1){
    analogWrite(shoulder_pin_a, 0);
    analogWrite(shoulder_pin_b, shoulder_speed);
    Serial.println("shoulder left");
  }
  else{
    analogWrite(shoulder_pin_a, 0);
    analogWrite(shoulder_pin_b, 0);
    Serial.println("shoulder stop");
  }

  if(ELBOW_DIR == 1){
    analogWrite(elbow_pin_a, elbow_speed);
    analogWrite(elbow_pin_b, 0);
    Serial.println("elbow right");
  }
  else if(ELBOW_DIR == -1){
    analogWrite(elbow_pin_a, 0);
    analogWrite(elbow_pin_b, elbow_speed);
    Serial.println("elbow left");
  }
  else{
    analogWrite(elbow_pin_a, 0);
    analogWrite(elbow_pin_b, 0);
    Serial.println("elbow stop");
  }

}

void setup() {
  Serial.begin(9600);
  nh.initNode();
  nh.subscribe(sub);

  pinMode(base_pin_a, OUTPUT);
  pinMode(base_pin_b, OUTPUT);

  pinMode(shoulder_pin_a, OUTPUT);
  pinMode(shoulder_pin_b, OUTPUT);

  pinMode(elbow_pin_a, OUTPUT);
  pinMode(elbow_pin_b, OUTPUT);

}

void loop(){
  actuate_joints();
  nh.spinOnce();
  delay(10);
}
