#include "arm_pico.h"

// bool serial_received;

char sending_data_buffer[MSG_LEN];
char receiving_data_buffer[MSG_LEN];
int bytes_read;
double ros_cmd_positions[8];      // [base, shoulder, elbow. yaw, pitch. roll, gripper, press]
double positions_from_sensor[6];  // [base, shoulder, elbow. yaw, pitch, roll]


void send_data_to_serial(double *state_positions) {
  String s = String(state_positions[0]);
  for (int i = 1; i < 6; i++) {
    s += String("_") + state_positions[i];
  }
  // Add the temperature
  char temprtr_buff[5];
  sprintf(temprtr_buff, "%2.1f", analogReadTemp());
  s += String("_") + (temprtr_buff);
  
  for (int i = 0; i < (MSG_LEN - 1); i++) {  // MSG_LEN - 1 because there is a trailing newline char
    if (i < s.length()) {                                             // copy the entire string s
      sending_data_buffer[i] = s.c_str()[i];
    } else {  // fillup the rest with '_'
      sending_data_buffer[i] = '_';
    }
  }
  Serial.flush();
  // Serial.print(sending_data_buffer);
  Serial.printf("%s\n", sending_data_buffer);
}

void read_data_from_serial() {
  bytes_read = Serial.readBytesUntil('\n', receiving_data_buffer, MSG_LEN);
  int i = bytes_read;
  while (i < MSG_LEN) {
    receiving_data_buffer[i++] = '\0';
  }
  // Serial.printf("bytes_read:%d. bytes:'%s'\n", bytes_read, receiving_data_buffer);
  if (bytes_read > 0) {
    if (receiving_data_buffer[0] == 'a') {
      last_heard = millis();
      set_all_pid_automatic();
      for (int i = 1, j = 0, last_i = 0; i < bytes_read && j < 6; i++) {
        // '\n' '\r' for reading 6 values anyhow otherwise if data is broken while sending you will loose the serialization
        if (receiving_data_buffer[i] == '_' || receiving_data_buffer[i] == '\r' || receiving_data_buffer[i] == '\n') {
          sscanf(receiving_data_buffer + last_i, "%lf", &ros_cmd_positions[j++]);
          // Serial.print("receiving_data_buffer + last_i:");
          // Serial.print(receiving_data_buffer + last_i);
          // Serial.print("J:");
          // Serial.println(j);
          last_i = i + 1;
        }
      }
    } else if (receiving_data_buffer[0] == 'r') {  // roll
      sscanf(receiving_data_buffer + 1, "%lf", &ros_cmd_positions[5]);
    } else if (receiving_data_buffer[0] == 'g') {  // grip
      sscanf(receiving_data_buffer + 1, "%lf", &ros_cmd_positions[6]);
    } else if (receiving_data_buffer[0] == 'p') {  // press
      sscanf(receiving_data_buffer + 1, "%lf", &ros_cmd_positions[7]);
    } else if (receiving_data_buffer[0] == 'm') {  // manual
      last_heard = millis();
      set_all_pid_manual();
      // Serial.println("read_data_from_serial");
      process_manual_commands(receiving_data_buffer + 1);
    }

    // for (int i = 0; i < 6; i++) {
    //   Serial.print(i);
    //   Serial.print(":");
    //   Serial.println(cmd_positions[i]);
    // }
  }
}


