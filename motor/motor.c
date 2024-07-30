/* 
 * Functions to pack commands into CAN packets for MyActuator Bionic motors
 */
#include "motor.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Function to shift and push bits
uint64_t push_bits(uint64_t value, uint32_t data, int num_bits) {
    value <<= num_bits;
    value |= data & ((1 << num_bits) - 1);
    return value;
}

// Function to push 32-bit floating point bits
uint64_t push_fp32_bits(uint64_t value, float data) {
    uint32_t data_bits;
    memcpy(&data_bits, &data, sizeof(data));
    value = push_bits(value, data_bits, 32);
    return value;
}

// Function to split command into bytes
void split_into_bytes(uint64_t command, uint8_t *bytes_list, int length, int little_endian) {
    for (int i = 0; i < length; i++) {
        bytes_list[i] = command & 0xFF;
        command >>= 8;
    }
    if (little_endian) {
        for (int i = 0; i < length / 2; i++) {
            uint8_t temp = bytes_list[i];
            bytes_list[i] = bytes_list[length - 1 - i];
            bytes_list[length - 1 - i] = temp;
        }
    }
}

// Function to set communication mode
int set_communication_mode(int motor_id, int mode, uint8_t *result) {
    uint8_t upper = motor_id >> 8;
    uint8_t lower = motor_id & 0xFF;
    int packet_len = 4;
    uint64_t command = 0;
    command = push_bits(command, upper, 8);
    command = push_bits(command, lower, 8);
    command = push_bits(command, 0, 8);
    command = push_bits(command, mode, 8);
    split_into_bytes(command, result, packet_len, 1);
    return packet_len;
}

// Function to set zero position
int set_zero_position(int motor_id, uint8_t *result) {
    uint8_t upper = motor_id >> 8;
    uint8_t lower = motor_id & 0xFF;
    int packet_len = 4;
    uint64_t command = 0;
    command = push_bits(command, upper, 8);
    command = push_bits(command, lower, 8);
    command = push_bits(command, 0, 8);
    command = push_bits(command, 3, 8); // 3 = Set zero position
    split_into_bytes(command, result, packet_len, 1);
    return packet_len;
}

// Function to set the CAN motor ID
int set_motor_id(int old_motor_id, int new_motor_id, uint8_t *result) {
    uint8_t old_upper = old_motor_id >> 8;
    uint8_t old_lower = old_motor_id & 0xFF;
    uint8_t new_upper = new_motor_id >> 8;
    uint8_t new_lower = new_motor_id & 0xFF;
    int packet_len = 6;
    uint64_t command = 0;
    command = push_bits(command, old_upper, 8);
    command = push_bits(command, old_lower, 8);
    command = push_bits(command, 0, 8);
    command = push_bits(command, 4, 8); // 4 = Set motor ID
    command = push_bits(command, new_upper, 8);
    command = push_bits(command, new_lower, 8);
    split_into_bytes(command, result, packet_len, 1);
    return packet_len;
}

// Function to get the CAN motor ID
int get_motor_id(uint8_t *result) {
    int motor_id = 0x7FF;
    uint8_t upper = motor_id >> 8;
    uint8_t lower = motor_id & 0xFF;
    int packet_len = 4;
    uint64_t command = 0;
    command = push_bits(command, upper, 8);
    command = push_bits(command, lower, 8);
    command = push_bits(command, 0, 8);
    command = push_bits(command, 0x82, 8); // 0x82 = Get motor ID
    split_into_bytes(command, result, packet_len, 1);
    return packet_len;
}

// Function to set position control
int set_position_control(float position, float max_speed, float max_current, int message_return, uint8_t *result) {
    int motor_mode = 1;
    int packet_len = 8;
    uint64_t command = 0;
    command = push_bits(command, motor_mode, 3);
    command = push_fp32_bits(command, position);
    command = push_bits(command, (int)(max_speed * 10), 15);
    command = push_bits(command, (int)(max_current * 10), 12);
    command = push_bits(command, message_return, 2);
    split_into_bytes(command, result, packet_len, 1);
    return packet_len;
}

// Function to set speed control
int set_speed_control(float speed, float current, int message_return, uint8_t *result) {
    int motor_mode = 2;
    int packet_len = 7;
    uint64_t command = 0;
    command = push_bits(command, motor_mode, 3);
    command = push_bits(command, 0, 3);
    command = push_bits(command, message_return, 2);
    command = push_fp32_bits(command, speed);
    command = push_bits(command, (int)(current * 10), 16);
    split_into_bytes(command, result, packet_len, 1);
    return packet_len;
}

// Function to set current torque control
int set_current_torque_control(int value, int control_status, int motor_mode, int message_return, uint8_t *result) {
    int packet_len = 3;
    uint64_t command = 0;
    command = push_bits(command, motor_mode, 3);
    command = push_bits(command, control_status, 3);
    command = push_bits(command, message_return, 2);
    command = push_bits(command, value * 10, 16);
    split_into_bytes(command, result, packet_len, 1);
    return packet_len;
}

// Function to get motor position
int get_motor_position(uint8_t *result) {
    int packet_len = 2;
    uint64_t command = 0;
    command = push_bits(command, 0x7, 3);
    command = push_bits(command, 0x0, 5);
    command = push_bits(command, 0x1, 8);
    split_into_bytes(command, result, packet_len, 1);
    return packet_len;
}

// Function to get motor speed
int get_motor_speed(uint8_t *result) {
    int packet_len = 2;
    uint64_t command = 0;
    command = push_bits(command, 0x7, 3);
    command = push_bits(command, 0x0, 5);
    command = push_bits(command, 0x2, 8);
    split_into_bytes(command, result, packet_len, 1);
    return packet_len;
}

// Function to get motor current
int get_motor_current(uint8_t *result) {
    int packet_len = 2;
    uint64_t command = 0;
    command = push_bits(command, 0x7, 3);
    command = push_bits(command, 0x0, 5);
    command = push_bits(command, 0x3, 8);
    split_into_bytes(command, result, packet_len, 1);
    return packet_len;
}

// Function to get motor power
int get_motor_power(uint8_t *result) {
    int packet_len = 2;
    uint64_t command = 0;
    command = push_bits(command, 0x7, 3);
    command = push_bits(command, 0x0, 5);
    command = push_bits(command, 0x4, 8);
    split_into_bytes(command, result, packet_len, 1);
    return packet_len;
}

// Function to force position hybrid control
int force_position_hybrid_control(float kp, float kd, float position, float speed, int torque_ff, uint8_t *result) {
    int packet_len = 8;
    uint64_t command = 0;
    int degrees_to_int = (int)(((position + 12.5) / 25.0) * 65536);
    int rpm_to_int = (int)(((speed + 18.0) / 36.0) * 4095);
    int torque_to_int = (int)(((torque_ff + 150) / 300) * 4095);
    command = push_bits(command, 0, 3);
    command = push_bits(command, (int)(kp * 4095 / 500), 12);
    command = push_bits(command, (int)(kd * 511 / 5), 9);
    command = push_bits(command, degrees_to_int, 16);
    command = push_bits(command, rpm_to_int, 12);
    command = push_bits(command, torque_to_int, 12);
    split_into_bytes(command, result, packet_len, 1);
    return packet_len;
}
