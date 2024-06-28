#include <ros.h>
#include <std_msgs/Int32.h>

#define ena1 6//give here the pin
#define in1 5 //for dc motor 1
#define in2 13  //

#define ena2 6//give here the pin
#define in3 5 //for dc motor 2
#define in4 13  //


int state=0
int start=0



void messageCallback(const std_msgs::Int32& msg) {
  // Handle received message
  received_msg = msg;
}

ros::NodeHandle nh1;
std_msgs::Int32 received_msg;
ros::Subscriber<std_msgs::Int32> sub("motor_command_value", &messageCallback);






void setup() {
 nh1.initNode();
nh1.subscribe(sub);

pinMode(ena1,OUTPUT);
pinMode(in1,OUTPUT);
pinMode(in2,OUTPUT);

pinMode(ena2,OUTPUT);
pinMode(in3,OUTPUT);
pinMode(in4,OUTPUT);
  



 
}

void loop() {

while (received_msg.data == 0) {
    nh.spinOnce(); // Handle incoming messages while waiting
    delay(100); // Add a small delay to avoid busy-waiting
  }

if(received_msg.data==5){
state=5;
}
else if(received_msg.data==6){
state=6;
}
else if(received_msg.data==7){
state=7;
}
else if(received_msg.data==8){
state=8;
}
else(received_msg.data==9){
state=9;
}

if(state==5){

if(received_msg.data==1){
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);

}

else if(received_msg.data==2){
    digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);

}

else if(received_msg.data==3){
    
    start=1;
}
else if(received_msg.data==4){
    
    start=0;
    state=0;
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      analogWrite(ena, 0);

}
else if(start==1){
    
      analogWrite(ena, (255/100)*(received_msg.data-7));
   

}

}

if(state==6){

if(received_msg.data==1){
      digitalWrite(in3, HIGH);
      digitalWrite(in4, LOW);

}

else if(received_msg.data==2){
    digitalWrite(in3, LOW);
      digitalWrite(in4, HIGH);

}

else if(received_msg.data==3){
    
    start=1;
}
else if(received_msg.data==4){
    
    start=0;
    state=0;
      digitalWrite(in3, LOW);
      digitalWrite(in4, LOW);
      analogWrite(ena2, 0);

}
else if(start==1){
    
      analogWrite(ena2, (255/100)*(received_msg.data-7));
   

}

}






}
