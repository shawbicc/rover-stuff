#include "arm_pico.h"

// ######################################### Soft I2C #############################################

int check_I2C_soft()  // checks if sensor is connected properly over I2C
{
  softWire.beginTransmission(addr_encdr);
  byte error = 1;
  error = softWire.endTransmission();
  if (error)
    return 1;
  else
    return 0;
}

int check_Magnet_soft()  // checks is the sensor has a properly placed magnet
{
  int magnetStatus = 0;  // value of the status register (MD, ML, MH)

  softWire.beginTransmission(addr_encdr);
  softWire.write(0x0B);                 // figure 21 - register map: Status: MD ML MH
  softWire.endTransmission();           // end transmission
  softWire.requestFrom(addr_encdr, 1);  // request from the sensor

  unsigned long t = millis();
  while (softWire.available() == 0 && (millis() - t) < WIRE_AVAIL_WAIT_TIMEOUT)
    ;                              // wait until there is a byte to read, i.e. wire.available() > 0
  magnetStatus = softWire.read();  // Reading the data after the request

  if (magnetStatus & (1 << 5))  // Ideally it should be 55
  {
    // snprintf(log_data, 30, "MD:Magnet Detected");
    // log_msg.data = log_data;
    // ard_log_arm.publish(&log_msg);
    return mag_MD; //magnet detected
  } else {
    if (magnetStatus & (1 << 4)) {
      // snprintf(log_data, 30, "ML:Magnet Low");
      // log_msg.data = log_data;
      // ard_log_arm.publish(&log_msg);
      return mag_ML; // magnet LOW(far away)
    } else {
      // snprintf(log_data, 30, "MH:Magnet High");
      // log_msg.data = log_data;
      // ard_log_arm.publish(&log_msg);
      return mag_MH; // magnet HIGH(too close)
    }
  }
}

uint16_t get_raw_angle_soft()  // outputs the raw angle
{
  uint16_t rawAngle;
  // 7:0 - bits
  softWire.beginTransmission(addr_encdr);  // connect to the sensor
  softWire.write(0x0C);                    // figure 21 - register map: Raw angle (11:8)
  softWire.endTransmission();              // end transmission
  softWire.requestFrom(addr_encdr, 2);     // request from the sensor
  unsigned long t = millis();
  while (softWire.available() < 2 && (millis() - t) < WIRE_AVAIL_WAIT_TIMEOUT)
    ;                                            // wait until it becomes available and timeout 5 seconds
  rawAngle = softWire.read();                    // Reading the data after the request (11:9)
  rawAngle = (rawAngle << 8) | softWire.read();  // (7:0)

  return rawAngle;
}

// ######################################### Wire 0 #############################################

int check_I2C_0()  // checks if sensor is connected properly over I2C
{
  Wire.beginTransmission(addr_encdr);
  byte error = 1;
  error = Wire.endTransmission();
  if (error)
    return 1;
  else
    return 0;
}

int check_Magnet_0()  // checks is the sensor has a properly placed magnet
{
  int magnetStatus = 0;  // value of the status register (MD, ML, MH)

  Wire.beginTransmission(addr_encdr);
  Wire.write(0x0B);                 // figure 21 - register map: Status: MD ML MH
  Wire.endTransmission();           // end transmission
  Wire.requestFrom(addr_encdr, 1);  // request from the sensor
  unsigned long t = millis();
  while (Wire.available() == 0 && (millis() - t) < WIRE_AVAIL_WAIT_TIMEOUT)
    ;                          // wait until there is a byte to read, i.e. wire.available() > 0
  magnetStatus = Wire.read();  // Reading the data after the request

  if (magnetStatus & (1 << 5))  // Ideally it should be 55
  {
    // snprintf(log_data, 30, "MD:Magnet Detected");
    // log_msg.data = log_data;
    // ard_log_arm.publish(&log_msg);
    return mag_MD; //magnet detected
  } else {
    if (magnetStatus & (1 << 4)) {
      // snprintf(log_data, 30, "ML:Magnet Low");
      // log_msg.data = log_data;
      // ard_log_arm.publish(&log_msg);
      return mag_ML; // magnet LOW(far away)
    } else {
      // snprintf(log_data, 30, "MH:Magnet High");
      // log_msg.data = log_data;
      // ard_log_arm.publish(&log_msg);
      return mag_MH; // magnet HIGH(too close)
    }
  }
}

uint16_t get_raw_angle_0()  // outputs the raw angle
{
  uint16_t rawAngle;
  // 7:0 - bits
  Wire.beginTransmission(addr_encdr);  // connect to the sensor
  Wire.write(0x0C);                    // figure 21 - register map: Raw angle (11:8)
  Wire.endTransmission();              // end transmission
  Wire.requestFrom(addr_encdr, 2);     // request from the sensor
  unsigned long t = millis();
  while (Wire.available() < 2 && (millis() - t) < WIRE_AVAIL_WAIT_TIMEOUT)
    ;                                        // wait until it becomes available and timeout 5 seconds
  rawAngle = Wire.read();                    // Reading the data after the request (11:9)
  rawAngle = (rawAngle << 8) | Wire.read();  // (7:0)

  return rawAngle;
}

// ######################################### Wire 1 #############################################

int check_I2C_1()  // checks if sensor is connected properly over I2C
{
  Wire1.beginTransmission(addr_encdr);
  byte error = 1;
  error = Wire1.endTransmission();
  if (error)
    return 1;
  else
    return 0;
}

int check_Magnet_1()  // checks is the sensor has a properly placed magnet
{
  int magnetStatus = 0;  // value of the status register (MD, ML, MH)

  Wire1.beginTransmission(addr_encdr);
  Wire1.write(0x0B);                 // figure 21 - register map: Status: MD ML MH
  Wire1.endTransmission();           // end transmission
  Wire1.requestFrom(addr_encdr, 1);  // request from the sensor
  unsigned long t = millis();
  while (Wire1.available() == 0 && (millis() - t) < WIRE_AVAIL_WAIT_TIMEOUT)
    ;                           // wait until there is a byte to read, i.e. Wire1.available() > 0
  magnetStatus = Wire1.read();  // Reading the data after the request

  if (magnetStatus & (1 << 5))  // Ideally it should be 55
  {
    // snprintf(log_data, 30, "MD:Magnet Detected");
    // log_msg.data = log_data;
    // ard_log_arm.publish(&log_msg);
    return mag_MD; //magnet detected
  } else {
    if (magnetStatus & (1 << 4)) {
      // snprintf(log_data, 30, "ML:Magnet Low");
      // log_msg.data = log_data;
      // ard_log_arm.publish(&log_msg);
      return mag_ML; // magnet LOW(far away)
    } else {
      // snprintf(log_data, 30, "MH:Magnet High");
      // log_msg.data = log_data;
      // ard_log_arm.publish(&log_msg);
      return mag_MH; // magnet HIGH(too close)
    }
  }
}

uint16_t get_raw_angle_1()  // outputs the raw angle
{
  uint16_t rawAngle;
  // 7:0 - bits
  Wire1.beginTransmission(addr_encdr);  // connect to the sensor
  Wire1.write(0x0C);                    // figure 21 - register map: Raw angle (11:8)
  Wire1.endTransmission();              // end transmission
  Wire1.requestFrom(addr_encdr, 2);     // request from the sensor
  unsigned long t = millis();
  while (Wire1.available() < 2 && (millis() - t) < WIRE_AVAIL_WAIT_TIMEOUT)
    ;                                         // wait until it becomes available and timeout 5 seconds
  rawAngle = Wire1.read();                    // Reading the data after the request (11:9)
  rawAngle = (rawAngle << 8) | Wire1.read();  // (7:0)

  return rawAngle;
}

// ######################################### Rotary encoder #############################################


//https://www.tinyosshop.com/index.php?route=product/product&product_id=1084
//https://www.dfrobot.com/product-634.html
void isr_encoder() {
  encoder_state = (encoder_state << 4) | (digitalRead(ROTA) << 1) | digitalRead(ROTB);
  switch (encoder_state) {              //               - - A B - - A B
    case 0x23: encoder_steps++; break;  // A leading: 0b 0 0 1 0 0 0 1 1 // every even callbacks
    case 0x13: encoder_steps--; break;  // B leading: 0b 0 0 0 1 0 0 1 1 // every even callbacks
    default: break;
  }
}

void encoder_init() {
  pinMode(ROTA, INPUT_PULLUP);
  pinMode(ROTB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ROTA), isr_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ROTB), isr_encoder, RISING);
  encoder_state = (digitalRead(ROTA) << 1) | digitalRead(ROTB);
  encoder_steps = 0;
  interrupts(); //Arduino core function to enable interrupts
}

// ######################################### Filtering #############################################

double filter_angle(double newAngle, double previousFilteredAngle) {
  // return SampleTime_by_SampleTime_plus_TimeConstant * newAngle +
  //        TimeConstant_by_SampleTime_plus_TimeConstant * previousFilteredAngle;
  if(newAngle - previousFilteredAngle > 20) return previousFilteredAngle;
  else return newAngle;
}

double getMovingAverage(double newValue) {
  // Add the new value to the readings array
  readings[readIndex] = newValue;

  // Update the index for the next reading (circular buffer)
  readIndex = (readIndex + 1) % numReadings;

  // Calculate the total by subtracting the oldest reading and adding the new one
  total = total - readings[(readIndex + numReadings + 1) % numReadings] + newValue;

  // Calculate and return the average
  return total / numReadings;
}

// ############################### Angle Measurement Utility Functions ###################################

double get_angle_in(uint16_t rawAngle, uint8_t angle_unit) {
  if (angle_unit == ANGLE_DEG) {
    return rawAngle * (360.0 / 4096.0);  // total 4096 steps in one rev
  } else if (angle_unit == ANGLE_RAD) {
    return rawAngle * (TWO_PI / 4096.0);  // total 4096 steps in one rev
  } else {
    return rawAngle;
  }
}

double roll_deg_to_steps(double deg) {
  return 1.9444444 * deg;  // 700/360
}

double roll_steps_to_deg(double steps) {
  return 0.5142857 * steps;  // 360/700
}

// For some reason this works, idk how !!!
// https://stackoverflow.com/a/50860805/8928251
double continuous_rotation_angle(int old_reading, int new_reading) {
  /* angle readings are in [0..360] range */
  /* compute the difference modulo 360 and shift it in range [-180..179] */
  return (360 + 180 + new_reading - old_reading) % 360 - 180;
}
