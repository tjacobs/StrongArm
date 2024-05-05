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

  // Get motor angle command
  sendCANCommand(MOTOR + 2, GET_MOTOR_ANGLE, 0, 0, 0);

  // Set action
  static int state = 3;
  static int angle = 10;
  static int t = 0;
  t++;
  if (t >= 15) {
    // Flip
    if      (angle == 10) angle = 20000;
    else if (angle == 20000) angle = 10;
    t = 0;
  }

  // Do action
  if (state == 1) {
    // Set motor torque command
    sendCANCommand(MOTOR + 1, SET_TORQUE, 50, 0, 0);
    delay(100);
    sendCANCommand(MOTOR + 2, SET_TORQUE, 50, 0, 0);
  }
  else if (state == 2) {
    // Set motor torque command
    sendCANCommand(MOTOR + 1, SET_TORQUE, -50, 0, 0);
    delay(100);
    sendCANCommand(MOTOR + 2, SET_TORQUE, -50, 0, 0);
  }
  else if (state == 3) {
    // Set motor angle command
    sendCANCommand(MOTOR + 2, SET_MOTOR_ANGLE, 0, 50, angle);
  }
  else if (state == 4) {
    // Stop motor
    sendCANCommand(MOTOR + 1, SHUT_DOWN_MOTOR, 0, 0, 0);
    delay(100);
    sendCANCommand(MOTOR + 2, SHUT_DOWN_MOTOR, 0, 0, 0);
  }

  // Receive packets
  while( receiveCANPacket() ) { };

  // Wait and blink
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}

void sendCANCommand(int id, uint8_t command, int param1, int param2, int param3) {
  // Pack packet
  uint8_t data1 = 0, data2 = 0, data3 = 0, data4 = 0, data5 = 0, data6 = 0, data7 = 0;
  if (command == COMMAND) {
    data1 = (uint8_t)(param1);
    data4 = (uint8_t)(param2);
    data5 = (uint8_t)(param2>>8);
    data6 = (uint8_t)(param2>>16);
    data7 = (uint8_t)(param2>>24);
  }
  else if (command == GET_SET_ID) {
    data2 = param1;
    data7 = param2;
  }
  else if (command == SET_TORQUE) {
    data4 = (uint8_t)(param1);
    data5 = (uint8_t)(param1>>8);
  }
  else if (command == SET_MOTOR_ANGLE) {
    data1 = param1;               // spinDirection
    data2 = (uint8_t)(param2);    // maxSpeed
    data3 = (uint8_t)(param2>>8);
    data4 = (uint8_t)(param3);    // angleControl
    data5 = (uint8_t)(param3>>8);
  }

  // Send packet
  Serial.print("Sending CAN packet to 0x");
  Serial.print(id, HEX);
  Serial.print(" 0x");
  Serial.print(command, HEX);
  Serial.print(" 0x");
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
    Serial.print("Received packet with id 0x");
    Serial.print(mcp.packetId(), HEX);
    Serial.print(": ");

    // Parse packet
    if (mcp.packetId() >= MOTOR_REPLY && mcp.packetId() <= MOTOR_REPLY + MAX_MOTORS) {
      uint8_t reply = (uint8_t)mcp.read();
      if (reply == GET_PID_PARAMS) {
        Serial.print("Motor params:");
      }
      else if (reply == GET_SET_ID) {
        Serial.print("ID:");
      }
      else if (reply == SET_TORQUE) {
        Serial.print("Torque:");
      }
      else if (reply == GET_MOTOR_ANGLE) {
        Serial.print("Get angle: ");
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
      else if (reply == SET_MOTOR_ANGLE) {
        Serial.print("Set angle:");
      }
      else if (reply == STOP_MOTOR) {
        Serial.print("Stop motor:");
      }
      else if (reply == SHUT_DOWN_MOTOR) {
        Serial.print("Shut down motor:");
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
