/*
 * CAN communication to MyActuator motors, v3 and Bionic
 * Thomas Jacobs, August 2024
 * Boards Manager URL: https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
 * Boards Manager: Raspberry Pi Pico/RP2040
 * Board: Raspberry Pi Pico/RP2040 -> Adafruit Feather RP2040 CAN
 * Library: Adafruit MCP2515
 */

#include <Adafruit_MCP2515.h>
#include "motor.h"

// CAN IDs for v3 MyActuator motors
#define MOTOR 0x140
#define NUM_MOTORS 6
#define motor1 (MOTOR + 1)
#define motor2 (MOTOR + 4)
#define motor3 (MOTOR + 2)
#define motor4 (MOTOR + 3)

// CAN comms
#define CAN_BAUDRATE (1000000)
Adafruit_MCP2515 mcp(PIN_CAN_CS);

// Serial packet
int header = 0;
int version = 0;
int command = 0;
int ax1 = 0;
int ax2 = 0;
int ay1 = 0;
int ay2 = 0;
int footer = 0;
int timeout = 0;

// State
int state = -1;
int step = 0;
int angle1 = 0;
int angle2 = 0;
int angle3 = 0;
int old_angle1 = 0;
int old_angle2 = 0;
int old_angle3 = 0;
int read_angles[NUM_MOTORS] = {};
int speed = 150;
int angle_test = 0;
int angle_direction = 0;
bool print = false;

void setup() {
  // Print
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial2.begin(115200);

  // Set up blink LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Init CAN
  if (!mcp.begin(CAN_BAUDRATE)) {
    Serial.println("Error initializing CAN chip.");
    while(1) delay(10);
  }

  // Set acceleration
  if (false) {
    int acceleration = 1500;
    sendCANCommand(motor2, SET_ACCELERATION, acceleration, 0, 0); delay(10);
    sendCANCommand(motor2, SET_ACCELERATION, acceleration, 1, 0); delay(10);
    sendCANCommand(motor1, SET_ACCELERATION, acceleration, 0, 0); delay(10);
    sendCANCommand(motor1, SET_ACCELERATION, acceleration, 1, 0); delay(10);
  }

  // Zero motors
  if (true) {
    delay(1000);
    sendCANCommand(motor2, SET_OUTPUT_ZERO_CURRENT, 0, 0, 0); delay(10);
    sendCANCommand(motor2, RESET_MOTOR, 0, 0, 0); delay(1000);
    sendCANCommand(motor1, SET_OUTPUT_ZERO_CURRENT, 0, 0, 0); delay(10);
    sendCANCommand(motor1, RESET_MOTOR, 0, 0, 0); delay(1000);
    if (print) Serial.println("Zeroed motors.");
  }

  // Get and set motor IDs
  if (false) {
    sendCANCommand(ALL_MOTORS, COMMAND, SET_CAN_FILTER, 0, 0); delay(10);
    sendCANCommand(ALL_MOTORS, GET_SET_ID, GET_ID, 0, 0); delay(1000);
    sendCANCommand(motor1, GET_SET_ID, SET_ID, 4, 0); delay(10);
    sendCANCommand(motor1, RESET_MOTOR, 0, 0, 0); delay(10);
    for (int i = 1; i < MAX_MOTORS; i++) sendCANCommand(MOTOR + i, GET_OUTPUT_ANGLE, 0, 0, 0); delay(10);
  }
}

void loop() {
  // Read serial
  if (readSerial()) {
    // Set angles from serial commands
    angle1 = ax1 * 100;
    angle2 = ay1 * 100;
    angle3 = ax2 * 10;
  }

  // Test movement
  else if (timeout < 1000) {
    // Back and forth movement test
    int max = 50 * 150;
    if      ((timeout % 200) <  50) step = 0;
    else if ((timeout % 200) < 100) step = 1;
    else if ((timeout % 200) < 150) step = 2;
    else if ((timeout % 200) < 200) step = 3;
    if (step == 0) { angle2 = -max; angle3 =  1; }
    if (step == 1) { angle1 =  max; }
    if (step == 2) { angle2 = 1; angle3 = 60; }
    if (step == 3) { angle1 = 1; }
    //if (angle_direction)   angle_test -= 1;
    //else                   angle_test += 1;
    //if (angle_test <= 0)   angle_direction = 0;
    //if (angle_test >= max) angle_direction = 1;
    //angle1 = angle_test;
    //angle2 = angle_test;
    //angle1 = angle_direction * max;
    //angle2 = angle_direction * max;
    if (false) { Serial.print(angle1); Serial.print(", "); Serial.print(angle2); Serial.print(", "); Serial.println(angle3); }
    state = 1;
  }
  else if (timeout < 1200) {
    angle1 = 0;
    angle2 = 0;
    state = 1;
  }
  else if (true) {
    state = 0;
  }

  // Read positions only
  //state = 2;

  // Output joint positions
  for (int i = 0; i < NUM_MOTORS; i++) { Serial.print(read_angles[i]); Serial.print(", "); }
  Serial.println("");

  // Do action
  if (state == 1) {
    // Set output angles
    int speed_b = 5;
    int current_b = 1;
    if (angle2 != old_angle2) sendCANCommand(motor2, SET_OUTPUT_ANGLE, angle2, speed, 0); delay(10);
    if (angle1 != old_angle1) sendCANCommand(motor1, SET_OUTPUT_ANGLE, angle1, speed, 0); delay(10);
    if (angle3 != old_angle3) sendCANCommand_b(1, B_SET_OUTPUT_ANGLE, angle3, speed_b, current_b); delay(10);
    old_angle1 = angle1;
    old_angle2 = angle2;
    old_angle3 = angle3;
    state = 2;

    // Get output angles
    sendCANCommand(motor2, GET_OUTPUT_ANGLE, 0, 0, 0); delay(10);
    sendCANCommand(motor1, GET_OUTPUT_ANGLE, 0, 0, 0); delay(10);
  }
  else if (state == 0) {
    // Shut down motors
    sendCANCommand(motor2, SHUT_DOWN_MOTOR, 0, 0, 0); delay(10);
    sendCANCommand(motor1, SHUT_DOWN_MOTOR, 0, 0, 0); delay(10);
    state = -2;
  }
  else if (state == 2) {
    // Get output angle
    //sendCANCommand_b(1,  B_GET_OUTPUT_ANGLE, 0, 0, 0); delay(10);
    sendCANCommand(motor2, GET_OUTPUT_ANGLE, 0, 0, 0); delay(10);
    sendCANCommand(motor1, GET_OUTPUT_ANGLE, 0, 0, 0); delay(10);
  }
  // Bionic motors
  else if (state == 3) {
    // Get position
    sendCANCommand_b(1, B_GET_OUTPUT_ANGLE, 0, 0, 0); delay(10);

    // Set position
    int speed_b = 10;
    int current_b = 1;
    sendCANCommand_b(1, B_SET_OUTPUT_ANGLE, angle1, speed_b, current_b); delay(10);
  }

  // Receive packets
  while (receiveCANPacket()) { };

  // Update gripper
  updateGripper();

  // Sleep
  delay(1);
}

// Read commands from USB serial
bool readSerial() {
  if (Serial.available() > 0) {
    header = Serial.read();
    if (print) {
      Serial.print("Read: 0x");
      Serial.println(header, HEX);
    }
    if (header == 'A') {
      // Read packet
      version = Serial.read();
      command = Serial.read();
      ax1 = Serial.read() - 'A';
      ay1 = Serial.read() - 'A';
      ax2 = Serial.read() - 'A';
      ay2 = Serial.read() - 'A';
      footer = Serial.read();
      if (print) {
        Serial.print("Got: ");
        Serial.print(ax1);
        Serial.print(", ");
        Serial.print(ay1);
        Serial.print(", ");
        Serial.print(ax2);
        Serial.print(", ");
        Serial.print(ay2);
        Serial.println("");
      }

      // LED on, got data
      digitalWrite(LED_BUILTIN, HIGH);

      // Run motors
      timeout = 0;
      state = 1;
      return true;
    }
  }

  // LED off if no data
  if (timeout > 2000) {
    // LED off
    digitalWrite(LED_BUILTIN, LOW);

    // Stop motors
    if (state >= -1) state = 0;
  }
  timeout++;

  // No data
  return false;
}

void sendCANCommand(int id, uint8_t command, int param1, int param2, int param3) {
  // Print
  if (false) {
    Serial.print("Sending packet to id 0x");
    Serial.print(id, HEX);
    Serial.print(": 0x");
    Serial.print(command, HEX);
    Serial.print(" ");
  }

  // Pack packet
  uint8_t data1 = 0, data2 = 0, data3 = 0, data4 = 0, data5 = 0, data6 = 0, data7 = 0;
  if (command == COMMAND) {
    if (print) Serial.print("Command: ");
    data1 = (uint8_t)(param1);
    data4 = (uint8_t)(param2);
    data5 = (uint8_t)(param2 >> 8);
    data6 = (uint8_t)(param2 >> 16);
    data7 = (uint8_t)(param2 >> 24);
  }
  else if (command == GET_SET_ID) {
    data2 = param1;
    data7 = param2;
    if (print) { 
      Serial.print("Get set ID: ");
      Serial.print(data2);
      Serial.print(" ");
      Serial.print(data7);
    }
  }
  else if (command == SET_ACCELERATION) {
    if (print) Serial.print("Set acceleration: ");
    data1 = param2;               // 0 = posAcc, 1 = posDec, 2 = speedAcc, 3 = speedDec
    data4 = (uint8_t)(param1);
    data5 = (uint8_t)(param1 >> 8);
    data6 = (uint8_t)(param1 >> 16);
    data7 = (uint8_t)(param1 >> 24);
  }
  else if (command == SET_TORQUE) {
    if (print) Serial.print("Set torque: ");
    data4 = (uint8_t)(param1);
    data5 = (uint8_t)(param1 >> 8);
  }
  else if (command == SET_MOTOR_ANGLE) {
    if (print) Serial.print("Set motor angle: ");
    data1 = param3;               // spinDirection
    data2 = (uint8_t)(param2);    // maxSpeed
    data3 = (uint8_t)(param2 >> 8);
    data4 = (uint8_t)(param1);    // angleControl
    data5 = (uint8_t)(param1 >> 8);
  }
  else if (command == SET_OUTPUT_ANGLE) {
    if (print) {
      Serial.print("Set output angle: ");
      Serial.println(param1);
    }
    data2 = (uint8_t)(param2);    // maxSpeed
    data3 = (uint8_t)(param2 >> 8);
    data4 = (uint8_t)(param1);    // angleControl
    data5 = (uint8_t)(param1 >> 8);
    data6 = (uint8_t)(param1 >> 16);
    data7 = (uint8_t)(param1 >> 24);
  }
  else {
    // Print
    if (print) {
      Serial.print("0x");
      Serial.print(data1, HEX);
      Serial.print(" 0x");
      Serial.print(data2, HEX);
      Serial.print(" 0x");
      Serial.print(data3, HEX);
      Serial.print(" 0x");
      Serial.print(data4, HEX);
      Serial.print(" 0x");
      Serial.print(data5, HEX);
      Serial.print(" 0x");
      Serial.print(data6, HEX);
      Serial.print(" 0x");
      Serial.println(data7, HEX);
    }
  }

  // Send packet
  mcp.beginPacket(id);
  mcp.write(command);
  mcp.write(data1);
  mcp.write(data2);
  mcp.write(data3);
  mcp.write(data4);
  mcp.write(data5);
  mcp.write(data6);
  mcp.write(data7);
  mcp.endPacket();
}

// Send a command to X-24 bionic motors
void sendCANCommand_b(int id, uint8_t command, float param1, int param2, int param3) {
  // Print
  if (false) {
    Serial.print("Sending packet to id 0x");
    Serial.print(id, HEX);
    Serial.print(": 0x");
    Serial.print(command, HEX);
    Serial.print(" ");
  }

  // Pack packet
  int packet_len = MAX_BYTES;
  uint8_t packet[packet_len];
  if (command == B_SET_MOTOR_ID) {
    if (print) Serial.print("Set motor ID: ");
    packet_len = set_motor_id(id, param1, packet);
  }
  else if (command == B_GET_MOTOR_ID) {
    if (print) Serial.print("Get motor ID: ");
    packet_len = get_motor_id(packet);
  }
  else if (command == B_SET_OUTPUT_ANGLE) {
    if (false) Serial.print("Set angle: ");
    packet_len = set_position_control(param1, param2, param3, 1, packet);
  }
  else if (command == B_GET_OUTPUT_ANGLE) {
    if (print) Serial.print("Get angle: ");
    packet_len = get_motor_position(packet);
  }
  else if (command == B_SET_OUTPUT_SPEED) {
    if (print) Serial.print("Set speed: ");
    packet_len = set_speed_control(param1, param2, 1, packet);
  }

  // Print
  if (false) {
    Serial.print("Command: ");
    for (int i = 0; i < packet_len; i++) {
        Serial.print(" 0x");
        Serial.print(packet[i], HEX);
    }
    Serial.println("");
  }

  // Send packet
  mcp.beginPacket(id);
  mcp.write((uint8_t*)&packet, packet_len);
  mcp.endPacket();
}

bool receiveCANPacket() {
  // Receive packet
  int packetSize = mcp.parsePacket();
  if (packetSize) {
    // Received a packet
    if (false) {
      Serial.print("Received packet from 0x");
      Serial.print(mcp.packetId(), HEX);
      Serial.print(":");
    }

    // Parse packet
    if (mcp.packetId() >= MOTOR_REPLY && mcp.packetId() <= MOTOR_REPLY + MAX_MOTORS) {
      uint8_t reply = (uint8_t)mcp.read();
      if (reply == GET_PID_PARAMS) {
        if (print) Serial.print(" Motor params:");
      }
      else if (reply == GET_SET_ID) {
        if (print) Serial.print(" ID:");
      }
      else if (reply == SET_TORQUE) {
        if (print) Serial.print(" Torque:");
      }
      else if (reply == GET_MOTOR_ANGLE) {
        if (print) Serial.print(" Get motor angle: ");
        mcp.read();
        mcp.read();
        mcp.read();
        mcp.read();
        mcp.read();
        int data6 = mcp.read();
        int data7 = mcp.read();
        int angle = (data7 << 8) + data6;
        if (print) Serial.print(angle);
      }
      else if (reply == GET_OUTPUT_ANGLE) {
        if (print) Serial.print(" Get output angle: ");
        int id = mcp.packetId() - MOTOR_REPLY;
        mcp.read();
        mcp.read();
        mcp.read();
        int data4 = mcp.read();
        int data5 = mcp.read();
        int data6 = mcp.read();
        int data7 = mcp.read();
        int angle = (data7 << 24) + (data6 << 16) + (data5 << 8) + data4;
        if (print) Serial.println(angle);
        if (id > 0 && id <= NUM_MOTORS + 1) read_angles[id - 1] = angle;
      }
      else if (reply == SET_MOTOR_ANGLE) {
        if (print) Serial.print(" Set angle:");
      }
      else if (reply == STOP_MOTOR) {
        if (print) Serial.print(" Stop motor:");
      }
      else if (reply == SHUT_DOWN_MOTOR) {
        if (print) Serial.print(" Shut down motor:");
        state = -1;
      }
    }
    // Bionic motor reply
    else if (mcp.packetId() >= 1 && mcp.packetId() <= MAX_MOTORS) {
      uint8_t reply = (uint8_t)mcp.read();
      if (reply == 0xA0) {
        if (print) Serial.print(" Angle: ");
        int data1 = mcp.read();
        int data2 = mcp.read();
        int data3 = mcp.read();
        int data4 = mcp.read();
        int data5 = mcp.read();
        int angle = ((data2-60) << 16) + (data3 << 8) + data4; // Actually a float32
        if (print) {
          Serial.print(data2);
          Serial.print(", ");
          Serial.print(data3);
          Serial.print(", ");
          Serial.print(data4);
          Serial.print(" = ");
          Serial.print(angle);
        }
      }
      else if (reply == 0x20) {
        if (false) Serial.print(" Data: ");
      }
      else if (true) {
        if (print) {
          Serial.print(" Reply: 0x");
          Serial.print(reply, HEX);
          Serial.print(", ");
        }
      }
    }

    // Print hex bytes
    while (mcp.available()) {
      if (print) Serial.print(" 0x");
      uint8_t byte = mcp.read();
      if (print) Serial.print(byte, HEX);
    }
    if (print) Serial.println();

    // We got a packet
    return true;
  }
  
  // No packet found
  return false;
}

// Send serial commands out other serial port to gripper board
void updateGripper() {
    // Set gripper command
  int gripper1 = ax2;
  int gripper2 = ay2;

  // Test gripper
  if (false) {
    static float pos = 0;
    static int direction = 1;
    if (direction == 1) pos += 0.01;
    else                pos -= 0.01;
    if (pos > 50 || pos < -50) direction = -direction;
    gripper1 = pos;
    gripper2 = pos;
  }

  // Send gripper command
  char data[8] = {0x61, 1, 1, (char)gripper1, (char)gripper2, 0, 0, 0};
  Serial1.print(data);

  // Read gripper
  if (false) {
    if (Serial1.available() > 0) {
      header = Serial1.read();
      Serial1.print("Read: 0x");
      Serial1.println(header, HEX);
    }
  }
}

// Also compile bionic motor CAN packing
#include "motor.c"
