#include "pico/types.h"
#ifndef _ARM_PICO_H_
#define _ARM_PICO_H_
#include <Arduino.h> //Arduino IDE intellisence don't work without including it
#include <sys/_stdint.h>
#include <pico/stdlib.h>
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>
#include "SoftI2C_mod.h"

#define addr_encdr 0x36

#define PIN_SDA_SOFT 26
#define PIN_SCL_SOFT 27
#define PIN_DIR_SOFT 28
#define PIN_SDA_0 20
#define PIN_SCL_0 21
#define PIN_DIR_0 22
#define PIN_SDA_1 18
#define PIN_SCL_1 19
#define PIN_DIR_1 17
#define WIRE_AVAIL_WAIT_TIMEOUT 500

// Define Pins
#define base_pin_a 4  // pwm
#define base_pin_b 5  // pwm

#define shoulder_pin_a 6  // pwm
#define shoulder_pin_b 7  // pwm

#define elbow_pin_a 8  // pwm
#define elbow_pin_b 9  // pwm

#define press_pin 0  //
#define pitch_pin 2  //
#define yaw_pin 3    //

#define roll_pin_a 10  //
#define roll_pin_b 11  //

#define gripper_pin_a 12  //
#define gripper_pin_b 13  //

#define GRIP_CLOSE 1
#define GRIP_OPEN 2

#define PRESS_ON 1
#define PRESS_OFF 2

#define ROLL_CW 700
#define ROLL_CCW 800

// MAX 2400 us MIN 550 us
#define SERVO_MAX_MICROS 2400
#define SERVO_MIN_MICROS 550
#define SERVO_MIDDLE (SERVO_MAX_MICROS + SERVO_MIN_MICROS) / 2 //servo angle 135 degree
#define PITCH_SERVO_LOW_DEGREE 90
#define PITCH_SERVO_HIGH_DEGREE 180
#define PITCH_SERVO_LOW 1167 // servo angle (135 - 45) = 90 degree
#define PITCH_SERVO_HIGH 1783 // servo angle (135 + 45) = 180 degree

#define ROLL_MAX_PWM 127 // this motor is very high speed even after gear reduction

#define ANGLE_RAW 1
#define ANGLE_DEG 2
#define ANGLE_RAD 3

#define mag_MD 0
#define mag_ML 1
#define mag_MH 2

#define FLAG_ENCODER 900
#define FLAG_MAGNET 800

#define ROTA 14
#define ROTB 15
#define ROLL_STEPS_PER_REV 700 // datasheet

#define MSG_LEN 46

#define SAMPLE_TIME_PID 100  // millis, SampleTime for PID controller

const double CUT_OFF_FREQ = 2.0; //Hz
const double SAMPLE_TIME_SENS = 100.0;  // millis, SampleTime for Sensor read
const double TIME_CONSTANT  = 1.0 / (2.0 * PI * CUT_OFF_FREQ); 
const double SampleTime_by_SampleTime_plus_TimeConstant = (SAMPLE_TIME_SENS / (SAMPLE_TIME_SENS + TIME_CONSTANT));
const double TimeConstant_by_SampleTime_plus_TimeConstant = (TIME_CONSTANT / (SAMPLE_TIME_SENS + TIME_CONSTANT));

const int numReadings = 8; // Number of readings to average
double readings[numReadings]; // Array to store past readings
int readIndex = 0;        // Index for the next reading in the array
double total = 0.0;        // Variable to store the sum of readings



extern char sending_data_buffer[MSG_LEN];
extern char receiving_data_buffer[MSG_LEN];
extern int bytes_read;
extern double ros_cmd_positions[8];         // [base, shoulder, elbow. yaw, pitch. roll, gripper, press]
extern int manual_cmd_speeds[8];         // [base, shoulder, elbow. yaw, pitch. roll, gripper, press]
extern double positions_from_sensor[6];     // [base, shoulder, elbow. yaw, pitch. roll]
extern double prev_enc_angles[3];
extern double new_angle;
bool angle_values_requested = true;

uint8_t encoder_state;
double encoder_steps;


extern int last_state_pitch;
extern int last_state_yaw;


extern SoftI2C_mod softWire;

extern double base_pid_out;
extern double base_K_P;          // p:1.50, i:0.50, d:0.05 on zero load
extern double base_K_I;          // p:1.50, i:0.50, d:0.05 on zero load
extern double base_K_D;          // p:1.50, i:0.50, d:0.05 on zero load
extern PID basePID;

extern double shoulder_pid_out;
extern double shoulder_K_P;          // p:1.50, i:0.50, d:0.05 on zero load
extern double shoulder_K_I;          // p:1.50, i:0.50, d:0.05 on zero load
extern double shoulder_K_D;          // p:1.50, i:0.50, d:0.05 on zero load
extern PID shoulderPID;

extern double elbow_pid_out;
extern double elbow_K_P;          // p:1.50, i:0.50, d:0.05 on zero load
extern double elbow_K_I;          // p:1.50, i:0.50, d:0.05 on zero load
extern double elbow_K_D;          // p:1.50, i:0.50, d:0.05 on zero load
extern PID elbowPID;

extern double roll_pid_out;
extern double roll_K_P;          // p:1.50, i:0.50, d:0.05 on zero load
extern double roll_K_I;          // p:1.50, i:0.50, d:0.05 on zero load
extern double roll_K_D;          // p:1.50, i:0.50, d:0.05 on zero load
extern PID rollPID;
extern bool roll_prev_manual;

Servo servoYaw;
Servo servoPitch;

// silence protection
extern int last_heard;


// necessary functions
void set_all_pid_manual();
void set_all_pid_automatic();
void process_manual_commands(char * cmd);

void set_pin_modes();
int check_I2C_soft(int address);  // checks if sensor is connected properly over I2C
int check_Magnet_soft();          // checks is the sensor has a properly placed magnet
uint16_t get_raw_angle_soft();         // outputs the raw angle

int check_I2C_0(int address);  // checks if sensor is connected properly over I2C
int check_Magnet_0();          // checks is the sensor has a properly placed magnet
uint16_t get_raw_angle_0();         // outputs the raw angle

int check_I2C_1(int address);  // checks if sensor is connected properly over I2C
int check_Magnet_1();          // checks is the sensor has a properly placed magnet
uint16_t get_raw_angle_1();         // outputs the raw angle

// int get_Angle(int address);    // outputs the degree angle
// void get_all_angles();
// void print_angles();
double get_angle_in(uint16_t rawAngle, uint8_t angle_unit);
// double get_angle_for(uint8_t i, uint8_t angle_unit);
void move_base(int speed);
void move_shoulder(int speed);
void move_elbow(int speed);
void move_pitch(int speed);
void move_yaw(int speed);
void move_roll(int speed);
void move_gripper(int command);
void move_press(int command);
void all_stop(void);


void isr_encoder();
void encoder_init();


void send_data_to_serial(double *state_positions);
void read_data_from_serial();

double roll_deg_to_steps(double deg);
double roll_steps_to_deg(double steps);

void init_angles();
double continuous_rotation_angle(int old_reading, int new_reading);
double filter_angle(double newAngle, double previousFilteredAngle);
double getMovingAverage(double newValue);

void n_blink_LED(uint8_t n_times);
void n_blink_LED(uint8_t n_times, uint16_t time_delay);
template<typename T>
inline T clamp(T val, T lo, T hi);
#endif
#pragma once