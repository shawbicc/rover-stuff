 /* software ---->
 A -> left
 B -> right

 A1 -> front
 A2 -> back

 B1 -> front
 B2 -> back

 

*/
#include "ros.h"
#include "geometry_msgs/Twist.h"

int x, z;

ros::NodeHandle nh;


// left
#define ARPWM 6
#define ALPWM 9
#define A1ENRF 7
#define A2ENBL 12

// right
#define BRPWM 3
#define BLPWM 5
//#define B1ENLF 10
#define B1ENLF A3  // <---------- changed
#define B2ENBR 2

#define speed 200

void moveBackward(int speed){
  // LF + LB
  analogWrite(A1ENRF, 255);
  analogWrite(A2ENBL, 255);
  digitalWrite(ARPWM, LOW);
  analogWrite(ALPWM, speed);

  // RF + RB
  analogWrite(B1ENLF, 255);
  analogWrite(B2ENBR, 255);
  digitalWrite(BRPWM, LOW);
  analogWrite(BLPWM, speed);
 
}

void moveForward(int spd){
  // LF + LB
  analogWrite(A1ENRF, 255);
  analogWrite(A2ENBL, 255);
  digitalWrite(ALPWM, LOW);
  analogWrite(ARPWM, spd);

  // RF + RB
  analogWrite(B1ENLF, 255);
  analogWrite(B2ENBR, 255);
  digitalWrite(BLPWM, LOW);
  analogWrite(BRPWM, spd);
 
}


void turnLeft(int spd){
   // LF + LB
  analogWrite(A1ENRF, 255);
  analogWrite(A2ENBL, 255);
  digitalWrite(ALPWM, LOW);
  analogWrite(ARPWM, spd);

  // RF + RB
  analogWrite(B1ENLF, 255);
  analogWrite(B2ENBR, 255);
  analogWrite(BLPWM, spd);
  digitalWrite(BRPWM, LOW);

 
}

void turnRight(int spd){
   // LF + LB
  analogWrite(A1ENRF, 255);
  analogWrite(A2ENBL, 255);
  analogWrite(ALPWM, spd);
  digitalWrite(ARPWM, LOW);

  // RF + RB
  analogWrite(B1ENLF, 255);
  analogWrite(B2ENBR, 255);
  digitalWrite(BLPWM, LOW);
  analogWrite(BRPWM, spd);

 
}

void turnforwardright(int spd_x, int spd_z){
  
}

void velCallback(const geometry_msgs::Twist& vel){
  x = vel.linear.x;
  z = vel.angular.z;
  

  int absolute_x;

  if (x<0)
  {  absolute_x=-x;
  }
  if (x<0 && z==0)
  {
    moveForward(absolute_x);
  }
  if (x>0 && z==0)
  {
  moveBackward(x);
  }

  if (z<0)
  {  absolute_z=-z;
  }

  if (x==0 && z>0)
  {
   turnRight(z);
  }
 
   if (x==0 && z<0)
  {
   turnLeft(absolute_z);
  }

  if (x>0 && z>0)
  {
    moveForward(x);
    turnRight(z);
  }
 
   if (x>0 && z<0)
  {
   moveForward(x);
   turnLeft(absolute_z);
  }

  if (x<0 && z>0)
  {
   moveBackward(absolute_x);
   turnRight(z);
  }
 
   if (x<0 && z<0)
  {
   moveBackward(absolute_x);
   turnLeft(absolute_z);
  }

   
  }

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", velCallback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);

  // Set motor connections as outputs
  pinMode(ARPWM, OUTPUT);
  pinMode(ALPWM, OUTPUT);
  pinMode(BRPWM, OUTPUT);
  pinMode(BLPWM, OUTPUT);
  pinMode(A1ENRF, OUTPUT);
  pinMode(A2ENBL, OUTPUT);
  pinMode(B1ENLF, OUTPUT);
  pinMode(B2ENBR, OUTPUT);
 

}

void loop() {
  nh.spinOnce();
  delay(10);
}
