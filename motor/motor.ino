/*
 * CAN communication to MyActuator motors
 * Thomas Jacobs, May 2024
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

void setup() {
  // Print
  Serial.begin(115200);
  while(!Serial) delay(10);
  Serial.println("CAN motor comms");

  // Set up blink LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Init CAN
  if (!mcp.begin(CAN_BAUDRATE)) {
    Serial.println("Error initializing CAN chip.");
    while(1) delay(10);
  }
  Serial.println("CAN chip found");
}

void loop() {
  // Get and set motor IDs
  if (false) {
    sendCANCommand(MOTOR + 1, COMMAND, SET_CAN_FILTER, 0, 0);
    sendCANCommand(MOTOR + 1, GET_SET_ID, GET_ID, 0, 0);
  }

  // Zero motors at current positions
  if (false) {
    sendCANCommand(MOTOR + 1, SET_OUTPUT_ZERO_CURRENT, 0, 0, 0);
    delay(100);
    sendCANCommand(MOTOR + 2, SET_OUTPUT_ZERO_CURRENT, 0, 0, 0);
    delay(100);
    sendCANCommand(MOTOR + 1, RESET_MOTOR, 0, 0, 0);
    delay(100);
    sendCANCommand(MOTOR + 2, RESET_MOTOR, 0, 0, 0);
    delay(100);
  }

  // Set action
  static int state = 1;
  static int angle = 0;
  static int direction = 1;
  static int speed = 100;
  static int t = 0;
  static int time = 0;
  t++;
  if (t >= 2000) {
    // Reset
    t = 0;

    // Shut down after a while
    time++;
    if (time > 10) state = 0;

    // Flip
    if      (direction ==  1) direction = -1;
    else if (direction == -1) direction = 1;
    angle = direction * 5000;

    // Do action
    if (state == 1) {
      // Set output angle
      sendCANCommand(MOTOR + 2, SET_OUTPUT_ANGLE, angle, speed, 0);
    }
    else if (state == 0) {
      // Shut down motor
      sendCANCommand(MOTOR + 1, SHUT_DOWN_MOTOR, 0, 0, 0);
      delay(100);
      sendCANCommand(MOTOR + 2, SHUT_DOWN_MOTOR, 0, 0, 0);
      delay(100);
    }
    
    // Get output angle
    sendCANCommand(MOTOR + 2, GET_OUTPUT_ANGLE, 0, 0, 0);

    // Wait and blink
    digitalWrite(LED_BUILTIN, HIGH);
    delay(50);
    digitalWrite(LED_BUILTIN, LOW);
    delay(50);
  }

  // Receive packets
  while( receiveCANPacket() ) { };

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
    data5 = (uint8_t)(param2>>8);
    data6 = (uint8_t)(param2>>16);
    data7 = (uint8_t)(param2>>24);
  }
  else if (command == GET_SET_ID) {
    Serial.print("Get set ID: ");
    data2 = param1;
    data7 = param2;
  }
  else if (command == SET_TORQUE) {
    Serial.print("Set torque: ");
    data4 = (uint8_t)(param1);
    data5 = (uint8_t)(param1>>8);
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
    data2 = (uint8_t)(param2);    // maxSpeed
    data3 = (uint8_t)(param2 >> 8);
    data4 = (uint8_t)(param1);    // angleControl
    data5 = (uint8_t)(param1 >> 8);
    data6 = (uint8_t)(param1 >> 16);
    data7 = (uint8_t)(param1 >> 24);
  }

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
