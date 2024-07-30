#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define MAX_BYTES 8

// Function to set position control
void set_position_control(int motor_id, float position, float max_speed, float max_current, int message_return, uint8_t *result);

// Function to set speed control
void set_speed_control(int motor_id, float speed, float current, int message_return, uint8_t *result);

// Function to set current torque control
void set_current_torque_control(int motor_id, int value, int control_status, int motor_mode, int message_return, uint8_t *result);

// Function to set zero position
void set_zero_position(int motor_id, uint8_t *result);

// Function to get motor position
void get_motor_pos(uint8_t *result);

// Function to get motor speed
void get_motor_speed(int motor_id, uint8_t *result);

// Function to get motor current
void get_motor_current(uint8_t *result);

// Function to get motor power
void get_motor_power(uint8_t *result);

// Function to force position hybrid control
void force_position_hybrid_control(float kp, float kd, float position, float speed, int torque_ff, uint8_t *result);