#define base_pin_a 8 // pwm 
#define base_pin_b 7 // pwm
#define base_speed 250

#define elbow_pin_a 3 // pwm
#define elbow_pin_b 5// pwm
#define elbow_speed 250


#define shoulder_pin_a 10 // pwm
#define shoulder_pin_b 11 // pwm
#define shoulder_speed 250

#define ch4_pin A1
#define ch5_pin A0 
#define ch6_pin A2 

double ch4 = 0; 
double ch5 = 0;
double ch6 = 0; 


void setup() {
  Serial.begin(9600); 
  pinMode(base_pin_a, OUTPUT);
  pinMode(base_pin_b, OUTPUT);

  pinMode(shoulder_pin_a, OUTPUT);
  pinMode(shoulder_pin_b, OUTPUT);

  pinMode(elbow_pin_a, OUTPUT);
  pinMode(elbow_pin_b, OUTPUT);

  pinMode(ch4_pin, INPUT); 
  pinMode(ch5_pin, INPUT); 

}

void loop() {
  ch4 = pulseIn(ch4_pin, HIGH); 
  ch5 = pulseIn(ch5_pin, HIGH); 
  ch6 = pulseIn(ch6_pin, HIGH); 

  Serial.print("CH 4 ");
  Serial.print(ch4); 
  Serial.print(" CH 5 "); 
  Serial.print(ch5);
  Serial.print(" CH 6 "); 
  Serial.print(ch6);
  Serial.println(); 

  if(ch4 == 0 && ch5==0 && ch6==0){
      analogWrite(base_pin_a, 0);
      analogWrite(base_pin_b, 0);

        analogWrite(shoulder_pin_a, 0);
    analogWrite(shoulder_pin_b, 0);

        analogWrite(elbow_pin_a, 0);
    analogWrite(elbow_pin_b, 0);    
  }

  if(ch4 > 1600){
      analogWrite(base_pin_a, base_speed);
      analogWrite(base_pin_b, 0);
  }  
  else if(ch4 < 1300){
      analogWrite(base_pin_a, 0);
      analogWrite(base_pin_b, base_speed);
  }
  else{
      analogWrite(base_pin_a, 0);
      analogWrite(base_pin_b, 0);
  }

  if(ch5 > 1300){
      analogWrite(shoulder_pin_a, shoulder_speed);
      analogWrite(shoulder_pin_b, 0);
  }
  else if(ch5 < 1050){
      analogWrite(shoulder_pin_a, 0);
      analogWrite(shoulder_pin_b, shoulder_speed);
  }
  else{
    analogWrite(shoulder_pin_a, 0);
    analogWrite(shoulder_pin_b, 0);
  }

  if(ch6 > 1300){
    analogWrite(elbow_pin_a, base_speed);
    analogWrite(elbow_pin_b, 0);
  }
  else if(ch6 < 1050){
    analogWrite(elbow_pin_a, 0);
    analogWrite(elbow_pin_b, base_speed);
  }
  else{
    analogWrite(elbow_pin_a, 0);
    analogWrite(elbow_pin_b, 0);
  }

}

