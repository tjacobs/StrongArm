#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

// Max CAN packet size
#define MAX_BYTES 8

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
#define ALL_MOTORS 0x300
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

// Function to set communication mode
int set_communication_mode(int motor_id, int mode, uint8_t *result);

// Function to set zero position
int set_zero_position(int motor_id, uint8_t *result);

// Function to set the CAN motor ID
int set_motor_id(int old_motor_id, int new_motor_id, uint8_t *result);

// Function to get the CAN motor ID
int get_motor_id(uint8_t *result);

// Function to set position control
int set_position_control(float position, float max_speed, float max_current, int message_return, uint8_t *result);

// Function to set speed control
int set_speed_control(float speed, float current, int message_return, uint8_t *result);

// Function to set current torque control
int set_current_torque_control(int value, int control_status, int motor_mode, int message_return, uint8_t *result);

// Function to get motor position
int get_motor_position(uint8_t *result);

// Function to get motor speed
int get_motor_speed(uint8_t *result);

// Function to get motor current
int get_motor_current(uint8_t *result);

// Function to get motor power
int get_motor_power(uint8_t *result);

// Function to force position hybrid control
int force_position_hybrid_control(float kp, float kd, float position, float speed, int torque_ff, uint8_t *result);
