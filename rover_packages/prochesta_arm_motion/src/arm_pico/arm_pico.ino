#include "arm_pico.h"
int last_state_pitch = 135;
int last_state_yaw = 135;

SoftI2C_mod softWire = SoftI2C_mod(PIN_SDA_SOFT, PIN_SCL_SOFT);  // sda, scl


bool roll_prev_manual = true;
double prev_enc_angles[3];
double new_angle;
int manual_cmd_speeds[8];  // [base, shoulder, elbow. yaw, pitch. roll, gripper, press]

int last_heard;

void setup() {  // Core0: For communication(i2c & serial)
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.setTimeout(100);
  Serial.begin(115200);

  softWire.begin();
  // pinMode(26, INPUT_PULLUP);
  // pinMode(27, INPUT_PULLUP);
  Wire.setClock(1000000);  // fast mode 1MHz
  Wire.setSDA(PIN_SDA_0);
  Wire.setSCL(PIN_SCL_0);
  Wire.begin();
  Wire1.setClock(1000000);  // fast mode 1MHz
  Wire1.setSDA(PIN_SDA_1);
  Wire1.setSCL(PIN_SCL_1);
  Wire1.begin();

  set_pin_modes();
  init_angles();
  while (!Serial)
    ;  // Pico takes some time after powering on to get the serial connection
}



void loop() {  // Serial r/w, i2cSoft, i2c1, i2c0
  if (Serial.available() > 0) {
    read_data_from_serial();
  }

  if (check_I2C_soft()) {  // base
    prev_enc_angles[0] = 0;
    basePID.SetMode(MANUAL);                  // But have to be manual if no sensor data is present
    positions_from_sensor[0] = FLAG_ENCODER;  // 900
  } else if (check_Magnet_soft()) {
    prev_enc_angles[0] = 0;
    basePID.SetMode(MANUAL);                                       // But have to be manual if no sensor data is present
    positions_from_sensor[0] = FLAG_MAGNET + check_Magnet_soft();  // 800
  } else {
    // basePID.SetMode(AUTOMATIC); // AUTOMATIC will be set upon receiving message (sometimes sensor data may be present still manual can be sent)
    new_angle = get_angle_in(get_raw_angle_soft(), ANGLE_DEG);
    if (positions_from_sensor[0] == FLAG_ENCODER || positions_from_sensor[0] == FLAG_MAGNET + mag_MH || positions_from_sensor[0] == FLAG_MAGNET + mag_ML) {
      positions_from_sensor[0] = new_angle;
    }
    // new_angle = filter_angle(new_angle, prev_enc_angles[0]);
    // positions_from_sensor[0] += continuous_rotation_angle(prev_enc_angles[0], new_angle);
    // positions_from_sensor[0] = getMovingAverage(positions_from_sensor[0]);
    // positions_from_sensor[0] = filter_angle(new_angle, positions_from_sensor[0]);
    positions_from_sensor[0] = new_angle;
    prev_enc_angles[0] = new_angle;
  }

  if (check_I2C_0()) {  // shoulder
    prev_enc_angles[1] = 0;
    shoulderPID.SetMode(MANUAL);  // But have to be manual if no sensor data is present
    positions_from_sensor[1] = FLAG_ENCODER;
  } else if (check_Magnet_0()) {
    prev_enc_angles[1] = 0;
    shoulderPID.SetMode(MANUAL);  // But have to be manual if no sensor data is present
    positions_from_sensor[1] = FLAG_MAGNET + check_Magnet_0();
  } else {
    // shoulderPID.SetMode(AUTOMATIC); // AUTOMATIC will be set upon receiving message (sometimes sensor data may be present still manual can be sent)
    new_angle = get_angle_in(get_raw_angle_0(), ANGLE_DEG);
    if (positions_from_sensor[1] == FLAG_ENCODER || positions_from_sensor[1] == FLAG_MAGNET + mag_MH || positions_from_sensor[1] == FLAG_MAGNET + mag_ML) {
      positions_from_sensor[1] = new_angle;
    }
    // new_angle = filter_angle(new_angle, prev_enc_angles[1]);
    // positions_from_sensor[1] += continuous_rotation_angle(prev_enc_angles[1], new_angle);
    // positions_from_sensor[1] = getMovingAverage(positions_from_sensor[1]);
    positions_from_sensor[1] = new_angle;
    // positions_from_sensor[1] = filter_angle(new_angle, positions_from_sensor[1]);
    prev_enc_angles[1] = new_angle;
  }

  if (check_I2C_1()) {  // elbow
    prev_enc_angles[2] = 0;
    elbowPID.SetMode(MANUAL);  // But have to be manual if no sensor data is present
    positions_from_sensor[2] = FLAG_ENCODER;
  } else if (check_Magnet_1()) {
    prev_enc_angles[2] = 0;
    elbowPID.SetMode(MANUAL);  // But have to be manual if no sensor data is present
    positions_from_sensor[2] = FLAG_MAGNET + check_Magnet_1();
  } else {
    // elbowPID.SetMode(AUTOMATIC); // AUTOMATIC will be set upon receiving message (sometimes sensor data may be present still manual can be sent)
    new_angle = get_angle_in(get_raw_angle_1(), ANGLE_DEG);
    if (positions_from_sensor[2] == FLAG_ENCODER || positions_from_sensor[2] == FLAG_MAGNET + mag_MH || positions_from_sensor[2] == FLAG_MAGNET + mag_ML) {
      positions_from_sensor[2] = new_angle;
    }
    // new_angle = filter_angle(new_angle, prev_enc_angles[2]);
    // positions_from_sensor[2] += continuous_rotation_angle(prev_enc_angles[2], new_angle);
    // positions_from_sensor[2] = getMovingAverage(positions_from_sensor[2]);
    // positions_from_sensor[2] = filter_angle(new_angle, positions_from_sensor[2]);
    positions_from_sensor[2] = new_angle;
    prev_enc_angles[2] = new_angle;
  }

  positions_from_sensor[3] = map(servoYaw.readMicroseconds(), SERVO_MIN_MICROS, SERVO_MAX_MICROS, 0, 270);
  positions_from_sensor[4] = map(servoPitch.readMicroseconds(), SERVO_MIN_MICROS, SERVO_MAX_MICROS, 0, 270);

  // Sending will be stopped if commanded
  if (angle_values_requested) {
    send_data_to_serial(positions_from_sensor);
  }
  delay(SAMPLE_TIME_SENS - 1);
}

void setup1() {    //Core1: For actuating
  encoder_init();  // so the interrupts are handled by Core1
  analogWriteFreq(10'000);
  analogWriteResolution(8);

  basePID.SetMode(AUTOMATIC);
  basePID.SetOutputLimits(-255, 255);
  basePID.SetSampleTime(SAMPLE_TIME_PID);

  shoulderPID.SetMode(AUTOMATIC);
  shoulderPID.SetOutputLimits(-255, 255);
  shoulderPID.SetSampleTime(SAMPLE_TIME_PID);

  elbowPID.SetMode(AUTOMATIC);
  elbowPID.SetOutputLimits(-255, 255);
  elbowPID.SetSampleTime(SAMPLE_TIME_PID);

  rollPID.SetMode(AUTOMATIC);
  rollPID.SetOutputLimits(-127, 127);  // roll motor rpm limit(so that it moves slowly)
  rollPID.SetSampleTime(SAMPLE_TIME_PID);

  servoYaw.attach(yaw_pin);
  servoPitch.attach(pitch_pin);

  while (!Serial)
    ;  // Pico takes some time after powering on to get the serial connection
}

void loop1() {
  positions_from_sensor[5] = roll_steps_to_deg(encoder_steps);
  if (ros_cmd_positions[5] == ROLL_CW) {  // manual roll for screw
    roll_prev_manual = true;
    rollPID.SetMode(MANUAL);
    move_roll(100);
  } else if (ros_cmd_positions[5] == ROLL_CCW) {  // manual roll for screw
    roll_prev_manual = true;
    rollPID.SetMode(MANUAL);
    move_roll(-100);
  } else if (roll_prev_manual) {
    roll_prev_manual = false;
    move_roll(0);  // stop
    rollPID.SetMode(AUTOMATIC);
  }

  // move_gripper(ros_cmd_positions[6]);
  // move_press(ros_cmd_positions[7]);

  if (ros_cmd_positions[4] <= PITCH_SERVO_HIGH_DEGREE && ros_cmd_positions[4] >= PITCH_SERVO_LOW_DEGREE) {
    servoPitch.writeMicroseconds(map(ros_cmd_positions[4], 0, 270, SERVO_MIN_MICROS, SERVO_MAX_MICROS));
  }
  servoYaw.writeMicroseconds(map(ros_cmd_positions[3], 0, 270, SERVO_MIN_MICROS, SERVO_MAX_MICROS));

  if (basePID.GetMode() == AUTOMATIC) {
    basePID.Compute();
    move_base(base_pid_out);
  }
  if (shoulderPID.GetMode() == AUTOMATIC) {
    shoulderPID.Compute();
    move_shoulder(shoulder_pid_out);
  }
  if (elbowPID.GetMode() == AUTOMATIC) {
    elbowPID.Compute();
    move_elbow(elbow_pid_out);
  }
  if (rollPID.GetMode() == AUTOMATIC) {
    rollPID.Compute();
    move_roll(roll_pid_out);
  }
  // if (millis() - last_heard > 1000) {  // Serial comm silence protection
  //   all_stop();
  // }
  if (BOOTSEL) {
    // Use bootsel button as a stop button
    all_stop();
    Serial.println("Emergency Stopped");
    n_blink_LED(5, 100);
    delay(1000);
  }
  delay(SAMPLE_TIME_PID - 1);  // a bit less than the sampletime of pid so the pid maybe at correct time
}

void process_manual_commands(char* cmd) {
  if (cmd[0] == '0') {
    all_stop();
    return;
  }
  n_blink_LED(1, 10);

  if (cmd[0] == 'b' || cmd[0] == 'B') {
    // must be coming from keyboard
    if (cmd[0] == 'b') {  // backward
      analogWrite(base_pin_a, 0);
      analogWrite(base_pin_b, 255);
      // Serial.println("b-");
    } else {  // forward
      analogWrite(base_pin_a, 255);
      analogWrite(base_pin_b, 0);
      // Serial.println("b+");
    }
    delay(250);
    analogWrite(base_pin_a, 0);
    analogWrite(base_pin_b, 0);
  } else if (cmd[0] == 's' || cmd[0] == 'S') {
    // must be coming from keyboard
    if (cmd[0] == 's') {  // backward
      analogWrite(shoulder_pin_a, 0);
      analogWrite(shoulder_pin_b, 255);
      // Serial.println("s-");
    } else {  // forward
      analogWrite(shoulder_pin_a, 255);
      analogWrite(shoulder_pin_b, 0);
      // Serial.println("s+");
    }
    delay(250);
    analogWrite(shoulder_pin_a, 0);
    analogWrite(shoulder_pin_b, 0);
  } else if (cmd[0] == 'e' || cmd[0] == 'E') {
    // must be coming from keyboard
    if (cmd[0] == 'e') {  // backward
      analogWrite(elbow_pin_a, 0);
      analogWrite(elbow_pin_b, 255);
      // Serial.println("e-");
    } else {  // forward
      analogWrite(elbow_pin_a, 255);
      analogWrite(elbow_pin_b, 0);
      // Serial.println("e+");
    }
    delay(250);
    analogWrite(elbow_pin_a, 0);
    analogWrite(elbow_pin_b, 0);
  } else if (cmd[0] == 'r' || cmd[0] == 'R') {
    // must be coming from keyboard
    if (cmd[0] == 'r') {  // backward
      analogWrite(roll_pin_a, 0);
      analogWrite(roll_pin_b, ROLL_MAX_PWM);  // roll motor rpm limit(so that it moves slowly)
      // Serial.println("r-");
    } else {                                  // forward
      analogWrite(roll_pin_a, ROLL_MAX_PWM);  // roll motor rpm limit(so that it moves slowly)
      analogWrite(roll_pin_b, 0);
      // Serial.println("r+");
    }
    delay(100);
    analogWrite(roll_pin_a, 0);
    analogWrite(roll_pin_b, 0);
  } else if (cmd[0] == 'g' || cmd[0] == 'G') {
    // must be coming from keyboard
    if (cmd[0] == 'g') {  // backward
      analogWrite(gripper_pin_a, 0);
      analogWrite(gripper_pin_b, 255);
      // Serial.println("g-");
    } else {  // forward
      analogWrite(gripper_pin_a, 255);
      analogWrite(gripper_pin_b, 0);
      // Serial.println("g+");
    }
    delay(250);
    analogWrite(gripper_pin_a, 0);
    analogWrite(gripper_pin_b, 0);
  } else if (cmd[0] == 'p' || cmd[0] == 'P') {
    int ms;
    if (cmd[0] == 'p') {  // backward
      ms = servoPitch.readMicroseconds() - 10;
      // Serial.print("pitch: ");
      // Serial.println(servoPitch.readMicroseconds());
    } else {  // forward
      ms = servoPitch.readMicroseconds() + 10;
      // Serial.print("pitch: ");
      // Serial.println(servoPitch.readMicroseconds());
    }
    if (ms <= PITCH_SERVO_HIGH && ms >= PITCH_SERVO_LOW) {
      servoPitch.writeMicroseconds(ms);
    }
  } else if (cmd[0] == 't' || cmd[0] == 'T') {
    move_press(PRESS_ON);
    delay(250);
    move_press(PRESS_OFF);
  }


  char* speed_str;
  uint8_t i = 0;
  speed_str = strtok(cmd, "_\n");
  while (speed_str != NULL) {
    sscanf(speed_str, "%d", &manual_cmd_speeds[i++]);
    speed_str = strtok(NULL, "_\n");
  }
  move_base(manual_cmd_speeds[0]);
  move_shoulder(manual_cmd_speeds[1]);
  move_elbow(manual_cmd_speeds[2]);
  servoPitch.writeMicroseconds(clamp(servoPitch.readMicroseconds() + clamp(manual_cmd_speeds[4], -10, 10), PITCH_SERVO_LOW, PITCH_SERVO_HIGH));
  move_roll(clamp(manual_cmd_speeds[5], -ROLL_MAX_PWM, ROLL_MAX_PWM));
  move_gripper(manual_cmd_speeds[6]);
  move_press(manual_cmd_speeds[7]);
}

void set_pin_modes() {
  pinMode(base_pin_a, OUTPUT);
  pinMode(base_pin_b, OUTPUT);
  pinMode(shoulder_pin_a, OUTPUT);
  pinMode(shoulder_pin_b, OUTPUT);
  pinMode(elbow_pin_a, OUTPUT);
  pinMode(elbow_pin_b, OUTPUT);
  pinMode(roll_pin_a, OUTPUT);
  pinMode(roll_pin_b, OUTPUT);
  pinMode(gripper_pin_a, OUTPUT);
  pinMode(gripper_pin_b, OUTPUT);
  pinMode(press_pin, OUTPUT);
  pinMode(pitch_pin, OUTPUT);
  pinMode(yaw_pin, OUTPUT);

  pinMode(PIN_SDA_SOFT, INPUT);
  pinMode(PIN_SCL_SOFT, INPUT);
  pinMode(PIN_DIR_SOFT, OUTPUT);
  pinMode(PIN_SDA_0, INPUT);
  pinMode(PIN_SCL_0, INPUT);
  pinMode(PIN_DIR_0, OUTPUT);
  pinMode(PIN_SDA_1, INPUT);
  pinMode(PIN_SCL_1, INPUT);
  pinMode(PIN_DIR_1, OUTPUT);
  pinMode(ROTA, INPUT_PULLUP);
  pinMode(ROTB, INPUT_PULLUP);
}

void set_all_pid_automatic() {
  basePID.SetMode(AUTOMATIC);
  shoulderPID.SetMode(AUTOMATIC);
  elbowPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);
}

void set_all_pid_manual() {
  basePID.SetMode(MANUAL);
  shoulderPID.SetMode(MANUAL);
  elbowPID.SetMode(MANUAL);
  rollPID.SetMode(MANUAL);
}

void init_angles() {
  prev_enc_angles[0] = 0.0;
  prev_enc_angles[1] = 0.0;
  prev_enc_angles[2] = 0.0;

  digitalWrite(PIN_DIR_SOFT, LOW);
  digitalWrite(PIN_DIR_0, LOW);
  digitalWrite(PIN_DIR_1, HIGH);

  servoPitch.writeMicroseconds(SERVO_MIDDLE);
}


void n_blink_LED(uint8_t n_times) {
  while (n_times-- > 0) {
    digitalWriteFast(LED_BUILTIN, HIGH);
    delay(50);
    digitalWriteFast(LED_BUILTIN, LOW);
    delay(50);
  }
}

void n_blink_LED(uint8_t n_times, uint16_t time_delay) {
  while (n_times-- > 0) {
    digitalWriteFast(LED_BUILTIN, HIGH);
    delay(time_delay);
    digitalWriteFast(LED_BUILTIN, LOW);
    delay(time_delay);
  }
}
