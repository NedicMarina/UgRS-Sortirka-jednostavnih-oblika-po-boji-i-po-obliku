#pragma once

// L298N IN2 is wired to GND. GPIO4 is reserved for the disabled flash.
constexpr int MOTOR_IN1 = 15;
constexpr int MOTOR_ENA = 2;
constexpr int MOTOR_SPEED = 90;
constexpr int SERVO_PIN = 14;
constexpr int IR1_PIN = 3;
constexpr int IR2_PIN = 13;

bool motorStart();
bool motorStop();
bool servoWriteAngleLocal(int angle);
