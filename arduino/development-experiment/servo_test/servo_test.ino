#include <Servo.h>

Servo myServo; // Create servo object to control a servo
int servoPin = 9; // Define the pin where the servo is connected
String inputString = ""; // A string to hold incoming data
bool stringComplete = false; // Whether the string is complete

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud
  myServo.attach(servoPin); // Attach the servo on pin 9 to the servo object
  inputString.reserve(200); // Reserve 200 bytes for the inputString
}

void loop() {
  // Process the input string when it's complete
  if (stringComplete) {
    if (inputString.startsWith("s ")) { // Check if the command starts with 's '
      int angle = inputString.substring(2).toInt(); // Get the angle from the command
      if (angle >= 0 && angle <= 180) { // Ensure the angle is within the valid range
        myServo.write(angle); // Move the servo to the specified angle
        int pulseWidth = myServo.readMicroseconds(); // Read the pulse width in microseconds
        Serial.print("angle: ");
        Serial.print(angle);
        Serial.print(", microsecond: ");
        Serial.println(pulseWidth);
      } else {
        Serial.println("Invalid angle. Please enter a value between 0 and 180.");
      }
    } else {
      Serial.println("Invalid command. Please use the format: s <angle>");
    }
    // Clear the string and reset the flag
    inputString = "";
    stringComplete = false;
  }
}

// SerialEvent occurs whenever a new data comes in the hardware serial RX. This
// function is called between each time loop() runs. Placing serialEvent() outside
// loop() means it will run after each time loop() runs.
void serialEvent() {
  while (Serial.available()) {
    // Get the new byte:
    char inChar = (char)Serial.read();
    // Add it to the inputString:
    inputString += inChar;
    // If the incoming character is a newline, set a flag so the main loop can
    // process the string:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
