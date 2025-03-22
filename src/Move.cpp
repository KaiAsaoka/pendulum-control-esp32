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
    bool motor1Dir;
    bool motor2Dir;

    if (speedY == 0 && speedX != 0) {
        motor1Dir = directionX;
        motor2Dir = directionX;
    }
    else if (speedX == 0 && speedY != 0) {
        motor1Dir = directionY;
        motor2Dir = !directionY;
    }   
    else{
        motor1Dir = (directionX && directionY) || (directionX && !directionY);
        motor2Dir = (directionX && directionY) || (!directionX && directionY);
    }
    // Convert boolean direction to multiplier (-1 or 1)
    int dirX = directionX ? 1 : -1;
    int dirY = directionY ? 1 : -1;

    int motor1Speed = speedX*dirX + speedY*dirY;  // For motor 1
    int motor2Speed = speedX*dirX - speedY*dirY;  // For motor 2 (subtract Y since it runs opposite for Y movement)
    
    // Calculate scaling factor if speeds exceed 255
    float scaleFactor = 1.0;
    int maxSpeed = std::max(abs(motor1Speed), abs(motor2Speed));
    if (maxSpeed > 255) {
        scaleFactor = 255.0 / maxSpeed;
    }
    
    // Apply scaling and determine directions
    motor1Speed = abs(round(motor1Speed * scaleFactor));
    motor2Speed = abs(round(motor2Speed * scaleFactor));
    

    
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

