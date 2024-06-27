#define x_factor 200
#define z_factor 50

// left
#define ARPWM 6
#define ALPWM 9
#define A1ENRF 7
#define A2ENBL 12

// right
#define BRPWM 3
#define BLPWM 5
#define B1ENLF 10
#define B2ENBR 2


// Global variables to store speed values
float x = 0.0; // Speed in forward/backward direction (0 to 1 for forward, -1 to 0 for backward)
float z = 0.0; // Speed in left/right turning (-1 to 1)

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  // Set motor driver pins as outputs
  pinMode(ARPWM, OUTPUT);
  pinMode(ALPWM, OUTPUT);
  pinMode(A1ENRF, OUTPUT);
  pinMode(A2ENBL, OUTPUT);
    
  pinMode(BRPWM, OUTPUT);
  pinMode(BLPWM, OUTPUT);
  pinMode(B1ENLF, OUTPUT);
  pinMode(B2ENBR, OUTPUT);

  // enable the motor drivers
  digitalWrite(A1ENRF, HIGH);
  digitalWrite(A2ENBL, HIGH);
  digitalWrite(B1ENLF, HIGH);
  digitalWrite(B2ENBR, HIGH);
}

void loop() {
  // Read data from serial monitor if available
  if (Serial.available() > 0) {
    // Read the incoming character
    char command = Serial.read();

    // Check if the received command is 'x' or 'z'
    if (command == 'x' || command == 'z') {
      // Read the value associated with the command
      float value = Serial.parseFloat();

      // Update global variables based on the command
      if (command == 'x') {
        x = value;
      } else if (command == 'z') {
        z = value;
      }
    }
  }

  // Adjust motor speeds based on the values of x and z
  // speed will be purely rotational if x == 0, otherwise it will be a combined effect of x and z, where x is dominating
  float leftSpeed = x == 0.0 ? (z * x_factor) : (x * x_factor) + (z * z_factor);
  float rightSpeed = x == 0.0 ? (z * x_factor * (-1)) : (x * x_factor) - (z * z_factor);

  // set motor PWM values
  // left
  if(leftSpeed >= 0){

    analogWrite(ALPWM, abs(leftSpeed));
    analogWrite(ARPWM, 0);
  } else if(leftSpeed < 0){
    analogWrite(ALPWM, 0);
    analogWrite(ARPWM, abs(leftSpeed));
  }

  // right
  if(rightSpeed >= 0){
    analogWrite(BLPWM, abs(rightSpeed));
    analogWrite(BRPWM, 0);
  } else if(rightSpeed < 0){
    analogWrite(BLPWM, 0);
    analogWrite(BRPWM, abs(rightSpeed));
  }
  Serial.println("leftSpeed:");
  Serial.println(leftSpeed);
  Serial.println("rightSpeed:");
  Serial.println(rightSpeed);
}
