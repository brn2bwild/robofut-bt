#include <stdint.h>
#include "Arduino.h"
#include "Motors.h"
#include "esp32-hal-ledc.h"
#include "esp32-hal.h"

Motors::Motors(uint8_t motor_left_a, uint8_t motor_left_b, uint8_t motor_left_enable, uint8_t motor_right_a, uint8_t motor_right_b, uint8_t motor_right_enable, int frequency, uint8_t resolution) {
  MOTOR_LEFT_A = motor_left_a;
  MOTOR_LEFT_B = motor_left_b;
  MOTOR_LEFT_EN = motor_left_enable;

  MOTOR_RIGHT_A = motor_right_a;
  MOTOR_RIGHT_B = motor_right_b;
  MOTOR_RIGHT_EN = motor_right_enable;

  FREQUENCY = frequency;
  RESOLUTION = resolution;

  // Initializa left motor pins
  pinMode(MOTOR_LEFT_A, OUTPUT);
  pinMode(MOTOR_LEFT_B, OUTPUT);
  pinMode(MOTOR_LEFT_EN, OUTPUT);
  ledcSetup(0, FREQUENCY, RESOLUTION);
  ledcAttachPin(MOTOR_LEFT_EN, 0);
  // ledcAttachPin(MOTOR_LEFT_EN, FREQUENCY, RESOLUTION);


  // Initializa right motor pins
  pinMode(MOTOR_RIGHT_A, OUTPUT);
  pinMode(MOTOR_RIGHT_B, OUTPUT);
  pinMode(MOTOR_RIGHT_EN, OUTPUT);
  ledcSetup(1, FREQUENCY, RESOLUTION);
  ledcAttachPin(MOTOR_RIGHT_EN, 1);
  // ledcAttachPin(MOTOR_RIGHT_EN, FREQUENCY, RESOLUTION);

  // Set motors on stop
  ledcWrite(0, 0);
  ledcWrite(1, 0);

  digitalWrite(MOTOR_LEFT_A, LOW);
  digitalWrite(MOTOR_LEFT_B, LOW);

  digitalWrite(MOTOR_RIGHT_A, LOW);
  digitalWrite(MOTOR_RIGHT_B, LOW);
}

void Motors::speeds(int left_speed, int right_speed) {
  leftMotor(left_speed);
  rightMotor(right_speed);
}

void Motors::leftMotor(int speed) {
  if (speed >= 0) {
    digitalWrite(MOTOR_LEFT_A, HIGH);
    digitalWrite(MOTOR_LEFT_B, LOW);
    ledcWrite(0, speed);
  } else {
    digitalWrite(MOTOR_LEFT_A, LOW);
    digitalWrite(MOTOR_LEFT_B, HIGH);
    ledcWrite(0, speed * (-1));
  }
}

void Motors::rightMotor(int speed) {
  if (speed >= 0) {
    digitalWrite(MOTOR_RIGHT_A, HIGH);
    digitalWrite(MOTOR_RIGHT_B, LOW);
    ledcWrite(1, speed);
  } else {
    digitalWrite(MOTOR_RIGHT_A, LOW);
    digitalWrite(MOTOR_RIGHT_B, HIGH);
    ledcWrite(1, speed * (-1));
  }
}
