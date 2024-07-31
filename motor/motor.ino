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
#define motor1 (MOTOR + 1)
#define motor2 (MOTOR + 4)
#define motor3 (MOTOR + 2)
#define motor4 (MOTOR + 3)

// CAN comms
#define CAN_BAUDRATE (1000000)
Adafruit_MCP2515 mcp(PIN_CAN_CS);

// Commands for v3 MyActuator motors
#define COMMAND 0x20
#define GET_PID_PARAMS 0x30
#define SET_ACCELERATION 0x43
#define GET_OUTPUT_ANGLE_ENCODER 0x60
#define GET_OUTPUT_ANGLE_ORIGINAL 0x61
#define GET_OUTPUT_ANGLE_ZERO 0x62
#define SET_OUTPUT_ZERO_VALUE 0x63
#define SET_OUTPUT_ZERO_CURRENT 0x64
#define GET_OUTPUT_ANGLE 0x92
#define SET_OUTPUT_ANGLE 0xA4
#define GET_MOTOR_ANGLE 0x94
#define SET_MOTOR_ANGLE 0xA6
#define SHUT_DOWN_MOTOR 0x80
#define STOP_MOTOR 0x81
#define RESET_MOTOR 0x76
#define SET_TORQUE 0xA1
#define GET_SET_ID 0x79

// Params for v3 MyActuator motors
#define SET_ID 0x00
#define GET_ID 0x01
#define SET_CAN_FILTER 0x02

// IDs for v3 MyActuator motors
#define MOTOR_REPLY 0x240
#define ALL_MOTORS 0x280
#define MAX_MOTORS 32

// Commands for X4-24 Bionic motor commands
#define B_ALL_MOTORS 0x7FF
#define B_SET_MOTOR_ID 0x04
#define B_GET_MOTOR_ID 0x82
#define B_SET_OUTPUT_ANGLE 0x01
#define B_GET_OUTPUT_ANGLE 0x07
#define B_SET_OUTPUT_SPEED 0x02
#define B_SET_OUTPUT_TORQUE 0x03
#define B_SET_ACCELERATION 0x06

// Functions
void sendCANCommand(int id, uint8_t command, int param1, int param2, int param3);
bool receiveCANPacket();

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
int angle1 = 0;
int angle2 = 0;
int old_angle1 = 0;
int old_angle2 = 0;
int speed = 100;
int angle_test = 0;

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
  int acceleration = 1500;
  sendCANCommand(motor2, SET_ACCELERATION, acceleration, 0, 0); delay(10);
  sendCANCommand(motor2, SET_ACCELERATION, acceleration, 1, 0); delay(10);
  sendCANCommand(motor1, SET_ACCELERATION, acceleration, 0, 0); delay(10);
  sendCANCommand(motor1, SET_ACCELERATION, acceleration, 1, 0); delay(10);
}

void loop() {
  // Get and set motor IDs
  //for (int i = 1; i < 32; i++) {
    //sendCANCommand(motor1, COMMAND, SET_CAN_FILTER, 0, 0); delay(10);
    //sendCANCommand(ALL_MOTORS, GET_SET_ID, GET_ID, 0, 0); delay(1000);
    //sendCANCommand(ALL_MOTORS, GET_SET_ID, SET_ID, 4, 0); delay(10);
    //sendCANCommand(MOTOR + i, GET_OUTPUT_ANGLE, 0, 0, 0); delay(10);
    //sendCANCommand(motor1, RESET_MOTOR, 0, 0, 0); delay(10);
    //sendCANCommand(motor2, RESET_MOTOR, 0, 0, 0); delay(10);
  //}

  // Zero motors at current positions
  if (false) {
    sendCANCommand(motor1, SET_OUTPUT_ZERO_CURRENT, 0, 0, 0); delay(10);
    //sendCANCommand(motor2, SET_OUTPUT_ZERO_CURRENT, 0, 0, 0); delay(10);
    sendCANCommand(motor1, RESET_MOTOR, 0, 0, 0); delay(5000);
    //sendCANCommand(motor2, RESET_MOTOR, 0, 0, 0); delay(5000);
  }

  // Get output angle
  if (true) sendCANCommand(motor1, GET_OUTPUT_ANGLE, 0, 0, 0);

  // Test bionic motors
  if (false) {
    // Set position
    float angle_b = 20.0;
    int speed_b = 10;
    int current_b = 1;
    sendCANCommand_b(1, B_SET_OUTPUT_ANGLE, angle_b, speed_b, current_b); delay(10);

    // Set speed
    //sendCANCommand_b(1, B_SET_OUTPUT_SPEED, angle_b, speed_b, 0); delay(10);
  }

  // Read serial
  readSerial();

  // Set angles from serial commands
  angle1 = ax1 * 200;
  angle2 = ay1 * 200;

  // Test
  if (false) { //} && timeout > 1000 && timeout < 11100) {
    angle1 = (angle_test % 50) * 100;
    angle2 = angle1;
    angle_test++;
    state = 1;
    delay(100);
  }
  if (false && timeout > 11100) {
    // Stop motors
    if (state >= -1) state = 0;
  }

  // Do action
  if (state == 1) {
    // Set output angle
    if (angle2 != old_angle2) sendCANCommand(motor2, SET_OUTPUT_ANGLE, angle2, speed, 0);
    if (angle1 != old_angle1) sendCANCommand(motor1, SET_OUTPUT_ANGLE, angle1, speed, 0);
    state = 2;
    old_angle1 = angle1;
    old_angle2 = angle2;
  }
  else if (state == 0) {
    // Shut down motors
    sendCANCommand(motor2, SHUT_DOWN_MOTOR, 0, 0, 0); delay(10);
    sendCANCommand(motor1, SHUT_DOWN_MOTOR, 0, 0, 0); delay(10);
    state = -2;
  }
  
  // Receive packets
  while( receiveCANPacket() ) { };

  // Update gripper
  updateGripper();

  // Sleep
  delay(1);
}

// Read commands from USB serial
void readSerial() {
  if (Serial.available() > 0) {
    header = Serial.read();
    Serial.print("Read: 0x");
    Serial.println(header, HEX);
    if (header == 'A') {
      // Read packet
      version = Serial.read();
      command = Serial.read();
      ax1 = Serial.read() - 'A';
      ay1 = Serial.read() - 'A';
      ax2 = Serial.read() - 'A';
      ay2 = Serial.read() - 'A';
      footer = Serial.read();
      Serial.print("Got: ");
      Serial.print(ax1);
      Serial.print(", ");
      Serial.print(ay1);
      Serial.print(", ");
      Serial.print(ax2);
      Serial.print(", ");
      Serial.print(ay2);
      Serial.println("");

      // LED on, got data
      digitalWrite(LED_BUILTIN, HIGH);

      // Run motors
      timeout = 0;
      state = 1;
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
}

void sendCANCommand(int id, uint8_t command, int param1, int param2, int param3) {
  // Print
  bool print = false;
  if (print) {
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
    Serial.print("Set acceleration: ");
    data1 = param2;               // 0 = posAcc, 1 = posDec, 2 = speedAcc, 3 = speedDec
    data4 = (uint8_t)(param1);
    data5 = (uint8_t)(param1 >> 8);
    data6 = (uint8_t)(param1 >> 16);
    data7 = (uint8_t)(param1 >> 24);
  }
  else if (command == SET_TORQUE) {
    Serial.print("Set torque: ");
    data4 = (uint8_t)(param1);
    data5 = (uint8_t)(param1 >> 8);
  }
  else if (command == SET_MOTOR_ANGLE) {
    Serial.print("Set motor angle: ");
    data1 = param3;               // spinDirection
    data2 = (uint8_t)(param2);    // maxSpeed
    data3 = (uint8_t)(param2 >> 8);
    data4 = (uint8_t)(param1);    // angleControl
    data5 = (uint8_t)(param1 >> 8);
  }
  else if (command == SET_OUTPUT_ANGLE) {
    Serial.print("Set output angle: ");
    Serial.println(param1);
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
  Serial.print("Sending packet to id 0x");
  Serial.print(id, HEX);
  Serial.print(": 0x");
  Serial.print(command, HEX);
  Serial.print(" ");

  // Pack packet
  int packet_len = MAX_BYTES;
  uint8_t packet[packet_len];
  if (command == B_SET_MOTOR_ID) {
    Serial.print("Set motor ID: ");
    packet_len = set_motor_id(id, param1, packet);
    debug(packet, packet_len);
  }
  else if (command == B_GET_MOTOR_ID) {
    Serial.print("Get motor ID: ");
    packet_len = get_motor_id(packet);
    debug(packet, packet_len);
  }
  else if (command == B_SET_OUTPUT_ANGLE) {
    Serial.print("Set angle: ");
    packet_len = set_position_control(param1, param2, param3, 1, packet);
    debug(packet, packet_len);
  }
  else if (command == B_GET_OUTPUT_ANGLE) {
    Serial.print("Get angle: ");
    packet_len = get_motor_position(packet);
    debug(packet, packet_len);
  }
  else if (command == B_SET_OUTPUT_SPEED) {
    Serial.print("Set speed: ");
    packet_len = set_speed_control(param1, param2, 1, packet);
    debug(packet, packet_len);
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
    Serial.print("Received packet from 0x");
    Serial.print(mcp.packetId(), HEX);
    Serial.print(":");

    // Parse packet
    if (mcp.packetId() >= MOTOR_REPLY && mcp.packetId() <= MOTOR_REPLY + MAX_MOTORS) {
      uint8_t reply = (uint8_t)mcp.read();
      if (reply == GET_PID_PARAMS) {
        Serial.print(" Motor params:");
      }
      else if (reply == GET_SET_ID) {
        Serial.print(" ID:");
      }
      else if (reply == SET_TORQUE) {
        Serial.print(" Torque:");
      }
      else if (reply == GET_MOTOR_ANGLE) {
        Serial.print(" Get motor angle: ");
        mcp.read();
        mcp.read();
        mcp.read();
        mcp.read();
        mcp.read();
        int data6 = mcp.read();
        int data7 = mcp.read();
        int angle = (data7 << 8) + data6;
        Serial.print(angle);
      }
      else if (reply == GET_OUTPUT_ANGLE) {
        Serial.print(" Get output angle: ");
        mcp.read();
        mcp.read();
        mcp.read();
        int data4 = mcp.read();
        int data5 = mcp.read();
        int data6 = mcp.read();
        int data7 = mcp.read();
        int angle = (data7 << 24) + (data6 << 16) + (data5 << 8) + data4;
        Serial.print(angle);
      }
      else if (reply == SET_MOTOR_ANGLE) {
        Serial.print(" Set angle:");
      }
      else if (reply == STOP_MOTOR) {
        Serial.print(" Stop motor:");
      }
      else if (reply == SHUT_DOWN_MOTOR) {
        //Serial.print(" Shut down motor:");
        state = -1;
      }
    }

    // Print hex bytes
    while (mcp.available()) {
      Serial.print(" 0x");
      Serial.print((uint8_t)mcp.read(), HEX);
    }
    Serial.println();

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

// Debug function
void debug(uint8_t *command, int length) {
    for (int i = 0; i < length; i++) {
        Serial.print(command[i], HEX);
    }
    Serial.print("\n");
}
