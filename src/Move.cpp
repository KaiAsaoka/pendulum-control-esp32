#include "Move.h"
#include "Encoder.h"
#include "Driver.h"
#include <cmath>
#include <algorithm>
#include <Arduino.h>

Move::Move(Driver dvr1, Driver dvr2) : dvr1(dvr1), dvr2(dvr2) {}

void Move::moveX(int speed, bool direction) {
    dvr1.move(speed, direction);
    dvr2.move(speed, direction);
}
    
void Move::moveY(int speed, bool direction) {  
    dvr1.move(speed, direction);
    dvr2.move(speed, !direction);
}

void Move::moveXY(int speedX, bool directionX, int speedY, bool directionY) {
    // Calculate raw motor speeds by combining X and Y components
    int motor1Speed = speedX + speedY;  // For motor 1
    int motor2Speed = speedX - speedY;  // For motor 2 (subtract Y since it runs opposite for Y movement)
    
    // Calculate scaling factor if speeds exceed 255
    float scaleFactor = 1.0;
    int maxSpeed = std::max(abs(motor1Speed), abs(motor2Speed));
    if (maxSpeed > 255) {
        scaleFactor = 255.0 / maxSpeed;
    }
    
    // Apply scaling and determine directions
    motor1Speed = abs(round(motor1Speed * scaleFactor));
    motor2Speed = abs(round(motor2Speed * scaleFactor));
    
    bool motor1Dir = (directionX && motor1Speed >= 0) || (!directionX && motor1Speed < 0);
    bool motor2Dir = (directionX && motor2Speed >= 0) || (!directionX && motor2Speed < 0);
    
    // Move motors with calculated speeds and directions
    dvr1.move(motor1Speed, motor1Dir);
    Serial.print("Motor 1: ");
    Serial.print(motor1Speed);
    Serial.print(" Direction: ");
    Serial.println(motor1Dir);
    
    dvr2.move(motor2Speed, motor2Dir);
    Serial.print("Motor 2: ");
    Serial.print(motor2Speed);
    Serial.print(" Direction: ");
    Serial.println(motor2Dir);
}

void Move::stop() {
    dvr1.move(0, 0);
    dvr2.move(0, 0);
}

