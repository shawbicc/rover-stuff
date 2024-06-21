
double ch1=A0;
double ch2=A1;
int sp=0;

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

void stop(){
    // LF + LB
  analogWrite(A1ENRF, 255);
  analogWrite(A2ENBL, 255);
  digitalWrite(ARPWM, LOW);
  analogWrite(ALPWM, LOW);

  // RF + RB
  analogWrite(B1ENLF, 255);
  analogWrite(B2ENBR, 255);
  digitalWrite(BRPWM, LOW);
  analogWrite(BLPWM, LOW);
}

void moveBackward(int spd){
  // LF + LB
  analogWrite(A1ENRF, 255);
  analogWrite(A2ENBL, 255);
  digitalWrite(ARPWM, LOW);
  analogWrite(ALPWM, spd);

  // RF + RB
  analogWrite(B1ENLF, 255);
  analogWrite(B2ENBR, 255);
  digitalWrite(BRPWM, LOW);
  analogWrite(BLPWM, spd);
 
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


void turnleft(int spd){
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

void turnright(int spd){
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



void setup()
{
  Serial.begin(9600);
  
  pinMode(A0,INPUT);
  pinMode(A1,INPUT);

  pinMode(ARPWM, OUTPUT);
  pinMode(ALPWM, OUTPUT);
  pinMode(BRPWM, OUTPUT);
  pinMode(BLPWM, OUTPUT);
  pinMode(A1ENRF, OUTPUT);
  pinMode(A2ENBL, OUTPUT);
  pinMode(B1ENLF, OUTPUT);
  pinMode(B2ENBR, OUTPUT);
 
  
}

void loop()
{
ch2 = pulseIn(A0,HIGH);
ch1 = pulseIn(A1,HIGH);

Serial.print("CH 1 ");
Serial.print(ch1); 

Serial.print("CH 2 ");
Serial.print(ch2); 
Serial.println();

if(ch1 == 0 && ch2 == 0){
  stop(); 
}


if((ch1 > 1530)){ 
    sp = map(ch1, 1530, 1980, 100, 255); 
   moveForward(sp); 
}
else if(ch1 < 1230){ 
  if(ch1 < 1230 && ch1 > 1000){
       sp = 130; 
  }
  else if(ch1 < 1000 && ch1 > 980){
     sp = 170; 
  }
  moveBackward(sp); 
}
else if(ch2 > 1530){ 
   sp = map(ch2, 1530, 1980, 150, 255); 
  turnright(sp); 
}
else if(ch2 < 1230){
  if(ch2 < 1230 && ch2 > 1000){
       sp = 150; 
  }
  else if(ch2 < 1000 && ch2 > 980){
     sp = 200; 
  }
  turnleft(sp); 
}
else {
  stop(); 
}

}