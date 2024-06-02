/*
 * CAN communication to MyActuator motors
 * Thomas Jacobs, June 2024
 * Boards Manager URL: https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json
 * Boards Manager: Raspberry Pi Pico/RP2040
 * Board: Raspberry Pi Pico/RP2040 -> Adafruit Feather RP2040 CAN
 * Library: Adafruit MCP2515
 */

#include <Adafruit_MCP2515.h>

// CAN comms
#define CAN_BAUDRATE (1000000)
Adafruit_MCP2515 mcp(PIN_CAN_CS);

// Commands
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

// Params
#define SET_ID 0x00
#define GET_ID 0x01
#define SET_CAN_FILTER 0x02

// IDs
#define MOTOR 0x140
#define MOTOR_REPLY 0x240
#define ALL_MOTORS 0x280
#define MAX_MOTORS 32

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

  // Set up blink LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Init CAN
  if (!mcp.begin(CAN_BAUDRATE)) {
    Serial.println("Error initializing CAN chip.");
    while(1) delay(10);
  }

  // Set acceleration
  sendCANCommand(MOTOR + 2, SET_ACCELERATION, 1000, 0, 0);
  delay(10);
  sendCANCommand(MOTOR + 2, SET_ACCELERATION, 1000, 1, 0);
  delay(10);
  sendCANCommand(MOTOR + 1, SET_ACCELERATION, 1000, 0, 0);
  delay(10);
  sendCANCommand(MOTOR + 1, SET_ACCELERATION, 1000, 1, 0);
  delay(10);
}

void loop() {
  // Get and set motor IDs
  if (false) {
    sendCANCommand(MOTOR + 1, COMMAND, SET_CAN_FILTER, 0, 0);
    sendCANCommand(MOTOR + 1, GET_SET_ID, GET_ID, 0, 0);
  }

  // Zero motors at current positions
  if (false) {
    sendCANCommand(MOTOR + 1, SET_OUTPUT_ZERO_CURRENT, 0, 0, 0); delay(1);
    sendCANCommand(MOTOR + 2, SET_OUTPUT_ZERO_CURRENT, 0, 0, 0); delay(1);
    sendCANCommand(MOTOR + 1, RESET_MOTOR, 0, 0, 0); delay(1);
    sendCANCommand(MOTOR + 2, RESET_MOTOR, 0, 0, 0); delay(1);
  }

  // Get output angle
  if (false) sendCANCommand(MOTOR + 1, GET_OUTPUT_ANGLE, 0, 0, 0);

  // Read USB serial
  if (Serial.available() > 0) {
    header = Serial.read();
    Serial.print("Read: 0x");
    Serial.println(header, HEX);
    if (header == 0x61) {
      // Read packet
      version = Serial.read();
      command = Serial.read();
      ax1 = Serial.read();
      ay1 = Serial.read();
      ax2 = Serial.read();
      ay2 = Serial.read();
      footer = Serial.read();
      Serial.print("Got: ");
      Serial.print(ax1);
      Serial.print(", ");
      Serial.print(ay1);
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

  // Set angles
  angle1 = ax1 * 100;
  angle2 = ay1 * 100;

  // Test
  if (timeout > 10000 && timeout < 10100) {
    angle1 = (angle_test % 50) * 100;
    angle2 = angle1;
    angle_test++;
    state = 1;
    delay(100);
  }
  if (timeout > 10100) {
    // Stop motors
    if (state >= -1) state = 0;
  }

  // Do action
  if (state == 1) {
    // Set output angle
    if (angle2 != old_angle2)
      sendCANCommand(MOTOR + 2, SET_OUTPUT_ANGLE, angle2, speed, 0);
    if (angle1 != old_angle1)
      sendCANCommand(MOTOR + 1, SET_OUTPUT_ANGLE, angle1, speed, 0);
    state = 2;
    old_angle1 = angle1;
    old_angle2 = angle2;
  }
  else if (state == 0) {
    // Shut down motor
    sendCANCommand(MOTOR + 2, SHUT_DOWN_MOTOR, 0, 0, 0);
    delay(1);
    sendCANCommand(MOTOR + 1, SHUT_DOWN_MOTOR, 0, 0, 0);
    state = -2;
  }
  
  // Receive packets
  //while( receiveCANPacket() ) { };

  // Wait
  delay(1);
}

void sendCANCommand(int id, uint8_t command, int param1, int param2, int param3) {
  // Print
  Serial.print("Sending packet to id 0x");
  Serial.print(id, HEX);
  Serial.print(": 0x");
  Serial.print(command, HEX);
  Serial.print(" ");

  // Pack packet
  uint8_t data1 = 0, data2 = 0, data3 = 0, data4 = 0, data5 = 0, data6 = 0, data7 = 0;
  if (command == COMMAND) {
    Serial.print("Command: ");
    data1 = (uint8_t)(param1);
    data4 = (uint8_t)(param2);
    data5 = (uint8_t)(param2 >> 8);
    data6 = (uint8_t)(param2 >> 16);
    data7 = (uint8_t)(param2 >> 24);
  }
  else if (command == GET_SET_ID) {
    Serial.print("Get set ID: ");
    data2 = param1;
    data7 = param2;
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
        Serial.print(" Shut down motor:");
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
