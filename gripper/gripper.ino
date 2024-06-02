/*
 * Gripper
 * Thomas Jacobs, June 2024
 * Boards Manager URL: https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
 * Boards Manager: Adafruit SAMD Boards
 * Board: Adafruit Feather M0 Express (SAMD21)
 * Library: Adafruit DMA neopixel library
 */

// Servos
#include <Servo.h>
Servo servo1;
Servo servo2;
int servo_stop_value = 92;
int s1_desired = 500;
int s2_desired = 500;
int timeout = 0;

// Neopixel
#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel pixel(1, 8, NEO_GRB + NEO_KHZ800);

// Pins
#define S1_O A0
#define S1_I A1
#define S2_O A2
#define S2_I A3

// Incoming serial packet
int header = 0;
int version = 0;
int command = 0;
int ax1 = 0;
int ax2 = 0;
int ay1 = 0;
int ay2 = 0;
int footer = 0;

// Read PWM
volatile int s1_position = -1;
volatile int s1_position_prev = -1;
volatile int s1_time = 0;
volatile int s2_position = -1;
volatile int s2_position_prev = -1;
volatile int s2_time = 0;

void setup() {
  // Servos
  servo1.attach(S1_O);
  servo2.attach(S2_O);

  // Serial
  Serial.begin(115200);
  Serial1.begin(115200);

  // Read PWM from serials
  attachInterrupt(S1_I, s1_rising, RISING);
  attachInterrupt(S2_I, s2_rising, RISING);

  // Neopixel
  pixel.begin();
  pixel.clear();
}

void loop() {
  // Read serial
  if (Serial1.available() > 0) {
    header = Serial1.read();
    Serial.println("Got data");
    if (header == 0x61) {
      // Read packet
      version = Serial1.read();
      command = Serial1.read();
      ax1 = Serial1.read();
      ay1 = Serial1.read();
      ax2 = Serial1.read();
      ay2 = Serial1.read();
      footer = Serial1.read();

      // LED green on
      pixel.setPixelColor(0, pixel.Color(0, 10, 0)); pixel.show();
      timeout = 0;
    }
  }

  /*while (Serial1.available() > 0) {
    header = Serial1.read();
    Serial.print("0x");
    Serial.print(header, HEX);
    Serial.print(" ");
  }
  Serial.println("");*/

  // LED off if no data
  if (timeout > 100) {
      // LED off
      pixel.setPixelColor(0, pixel.Color(0, 0, 0)); pixel.show();
  }
  timeout++;

  // Test servos
  if (false) {
    static float movement = 10;
    static float position = 0;
    static int direction = 0;
    if (direction == 0) {
      position += 0.005f;
      if (position > movement) direction = 1;
    } else {
      position -= 0.005f;
      if (position <= -movement) direction = 0;        
    }
    ax1 = 50 + position;
    ay1 = 45 + position;
    command = 1;

    // Set LED color
    int red = 0;
    int green = max(0, -position / 10);
    int blue = max(0, position / 10);
    pixel.setPixelColor(0, pixel.Color(red, green, blue)); pixel.show();
    timeout = 0;
  }

  // Command: set servo positions
  if (command == 1) {
    s1_desired = ax1 * 10;
    s2_desired = ay1 * 10;

    // Limit
    if (s2_desired > 500) s2_desired = 500;
    if (s2_desired < 300) s2_desired = 300;    
  }

  // Move servos to desired positions with P D control from servo feedback position wire
  float kp = 0.1f;
  float kd = 0.1f;
  static float s1_error_prev = 0;
  static float s2_error_prev = 0;
  float s1_error = s1_position - s1_desired;
  float s1_velocity = kp * s1_error - kd * (s1_error - s1_error_prev);
  float s2_error = s2_position - s2_desired;
  float s2_velocity = kp * s2_error - kd * (s2_error - s2_error_prev);
  s1_error_prev = s1_error;
  s2_error_prev = s2_error;
  servo1.write(servo_stop_value + s1_velocity);
  servo2.write(servo_stop_value + s2_velocity);

  // Error
  if (abs(s1_error) > 100 || abs(s2_error) > 100) {
      pixel.setPixelColor(0, pixel.Color(10, 0, 0)); pixel.show();
      timeout = 0;
      String c = ", ";
      Serial.println(s1_error + c + s1_position + c + s1_desired);      
      Serial.println(s2_error + c + s2_position + c + s2_desired);      
  }

  // Wait
  delay(1);
}

// Read PWM
void s1_rising() {
  attachInterrupt(S1_I, s1_falling, FALLING);
  s1_time = micros();
} 
void s1_falling() {
  attachInterrupt(S1_I, s1_rising, RISING);
  int s1_position_new = micros() - s1_time;
  if (abs(s1_position_new - s1_position_prev) < 100 || s1_position == -1) s1_position = s1_position_new;
  s1_position_prev = s1_position;
}
void s2_rising() {
  attachInterrupt(S2_I, s2_falling, FALLING);
  s2_time = micros();
}
void s2_falling() {
  attachInterrupt(S2_I, s2_rising, RISING);
  int s2_position_new = micros() - s2_time;
  if (abs(s2_position_new - s2_position_prev) < 100 || s2_position == -1) s2_position = s2_position_new;
  s2_position_prev = s2_position;
}
