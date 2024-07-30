#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define MAX_BYTES 8

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
