#include "arm_pico.h"

double base_pid_out = 0;
double base_K_P = 0;  // p:1.50, i:0.50, d:0.05 on zero load
double base_K_I = 0;  // p:1.50, i:0.50, d:0.05 on zero load
double base_K_D = 0;  // p:1.50, i:0.50, d:0.05 on zero load
PID basePID = PID(&positions_from_sensor[0], &base_pid_out, &ros_cmd_positions[0], base_K_P, base_K_I, base_K_D, DIRECT);


double shoulder_pid_out = 0;
double shoulder_K_P = 0;  // p:1.50, i:0.50, d:0.05 on zero load
double shoulder_K_I = 0;  // p:1.50, i:0.50, d:0.05 on zero load
double shoulder_K_D = 0;  // p:1.50, i:0.50, d:0.05 on zero load
PID shoulderPID = PID(&positions_from_sensor[1], &shoulder_pid_out, &ros_cmd_positions[1], shoulder_K_P, shoulder_K_I, shoulder_K_D, DIRECT);

double elbow_pid_out = 0;
double elbow_K_P = 0;  // p:1.50, i:0.50, d:0.05 on zero load
double elbow_K_I = 0;  // p:1.50, i:0.50, d:0.05 on zero load
double elbow_K_D = 0;  // p:1.50, i:0.50, d:0.05 on zero load
PID elbowPID = PID(&positions_from_sensor[2], &elbow_pid_out, &ros_cmd_positions[2], elbow_K_P, elbow_K_I, elbow_K_D, DIRECT);

double roll_pid_out = 0;
double roll_K_P = 0;  // p:1.50, i:0.50, d:0.05 on zero load P_ON_M: p:0.50, i:2.00, d:0.00
double roll_K_I = 0;  // p:1.50, i:0.50, d:0.05 on zero load P_ON_M: p:0.50, i:2.00, d:0.00
double roll_K_D = 0;  // p:1.50, i:0.50, d:0.05 on zero load P_ON_M: p:0.50, i:2.00, d:0.00
PID rollPID = PID(&positions_from_sensor[5], &roll_pid_out, &ros_cmd_positions[5], roll_K_P, roll_K_I, roll_K_D, P_ON_M, DIRECT);


void move_base(int speed) {
  analogWrite(base_pin_a, abs(max(0, speed)));
  analogWrite(base_pin_b, abs(min(0, speed)));
}

void move_shoulder(int speed) {
  analogWrite(shoulder_pin_a, abs(max(0, speed)));
  analogWrite(shoulder_pin_b, abs(min(0, speed)));
}

void move_elbow(int speed) {
  analogWrite(elbow_pin_a, abs(max(0, speed)));
  analogWrite(elbow_pin_b, abs(min(0, speed)));
}

void move_pitch(int current, int target) {
  // Serial.print("pitch stby  ");
  if ((last_state_pitch != target)) {
    if (target > last_state_pitch) {
      while (target != last_state_pitch) {
        int onTime = map(last_state_pitch, 0, 270, 500, 2500);
        int offTime = 20000 - onTime;
        digitalWrite(pitch_pin, HIGH);
        delayMicroseconds(onTime);
        digitalWrite(pitch_pin, LOW);
        delayMicroseconds(offTime);
        delay(30);
        last_state_pitch = last_state_pitch + 1;
      }
    } else if (target < last_state_pitch) {
      while (target != last_state_pitch) {
        int onTime = map(last_state_pitch, 0, 270, 500, 2500);
        int offTime = 20000 - onTime;
        digitalWrite(pitch_pin, HIGH);
        delayMicroseconds(onTime);
        digitalWrite(pitch_pin, LOW);
        delayMicroseconds(offTime);
        delay(30);
        last_state_pitch = last_state_pitch - 1;
      }
    }
  }
  // delay(250);
}

void move_yaw(int current, int target) {
  // Serial.print("yaw stby  ");
  if (last_state_yaw != target) {
    if (target > last_state_yaw) {
      while (target != last_state_yaw) {
        int onTime = map(last_state_yaw, 0, 270, 500, 2500);
        int offTime = 20000 - onTime;
        digitalWrite(yaw_pin, HIGH);
        delayMicroseconds(onTime);
        digitalWrite(yaw_pin, LOW);
        delayMicroseconds(offTime);
        delay(30);
        last_state_yaw = last_state_yaw + 1;
      }
    } else if (target < last_state_yaw) {
      while (target != last_state_yaw) {
        int onTime = map(last_state_yaw, 0, 270, 500, 2500);
        int offTime = 20000 - onTime;
        digitalWrite(yaw_pin, HIGH);
        delayMicroseconds(onTime);
        digitalWrite(yaw_pin, LOW);
        delayMicroseconds(offTime);
        delay(30);
        last_state_yaw = last_state_yaw - 1;
      }
    }
  }
  // delay(250);
}

void move_roll(int speed) {
  analogWrite(roll_pin_a, abs(max(0, speed)));
  analogWrite(roll_pin_b, abs(min(0, speed)));
}

void move_gripper(int command) {
  if (command == GRIP_CLOSE) {
    digitalWrite(gripper_pin_a, HIGH);
    digitalWrite(gripper_pin_b, LOW);
  } else if (command == GRIP_OPEN) {
    digitalWrite(gripper_pin_a, LOW);
    digitalWrite(gripper_pin_b, HIGH);
  } else {
    digitalWrite(gripper_pin_a, LOW);
    digitalWrite(gripper_pin_b, LOW);
  }
}

void move_press(int command) {
  if (command == PRESS_ON) {
    digitalWrite(press_pin, HIGH);
  } else if (PRESS_OFF) {
    digitalWrite(press_pin, LOW);
  } else {
    digitalWrite(press_pin, LOW);
  }
}

void all_stop(void) {
  digitalWrite(base_pin_a, LOW);
  digitalWrite(base_pin_b, LOW);

  digitalWrite(shoulder_pin_a, LOW);
  digitalWrite(shoulder_pin_b, LOW);

  digitalWrite(elbow_pin_a, LOW);
  digitalWrite(elbow_pin_b, LOW);

  digitalWrite(roll_pin_a, LOW);
  digitalWrite(roll_pin_b, LOW);

  digitalWrite(gripper_pin_a, LOW);
  digitalWrite(gripper_pin_b, LOW);

  digitalWrite(press_pin, LOW);
}

// ############################### Utility Functions ###################################
// from: https://stackoverflow.com/a/46963317/8928251
template<typename T>
inline T clamp(T val, T lo, T hi) {
  return max(lo, min(hi, val));
}

