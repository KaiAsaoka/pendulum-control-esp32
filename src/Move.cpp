#include "Move.h"
#include "Encoder.h"
#include "Driver.h"
#include <cmath>
#include <algorithm>
#include <Arduino.h>

#define STATIC_BRAKE 0
#define DYNAMIC_BRAKE 0
#define BELT_DRIVE_RADIUS 16.1671f
#define ENCODER_360 0x3FFF

Move::Move(Driver& dvr1, Driver& dvr2, Encoder& enc1, Encoder& enc2) : dvr1(dvr1), dvr2(dvr2), enc1(enc1), enc2(enc2) {}

// void Move::moveX(int speed, bool direction) {
//     dvr1.move(speed, direction);
//     dvr2.move(speed, direction);
// }
    
// void Move::moveY(int speed, bool direction) {  
//     dvr1.move(speed, direction);
//     dvr2.move(speed, !direction);
// }

// Scaling Factor to mm for determining position
constexpr float SCALE_FACTOR = 10 * (0.87f * M_PI * BELT_DRIVE_RADIUS) / ENCODER_360;

void Move::moveXY(int speedX, int speedY) {
    speedX = -speedX;
    
    // Calculate motor speeds by combining X and Y components
    int motor1Speed = (speedX) + (speedY);
    int motor2Speed = (speedX) - (speedY);
    
    vel1 = motor1Speed;
    vel2 = motor2Speed;

    // Determine motor directions based on calculated speeds
    bool motor1Dir = motor1Speed >= 0;
    bool motor2Dir = motor2Speed >= 0;

    // Get absolute values for motor speeds
    motor1Speed = abs(motor1Speed);
    motor2Speed = abs(motor2Speed);

    if (motor1Speed!=0 && motor2Speed==0) {
        // Move motors with calculated speeds and directions
        motor2Speed = DYNAMIC_BRAKE;
        motor2Dir = motor1Dir;
        // Serial.println("Engaging Motor 2 Brake");

        dvr1.move(motor1Speed, motor1Dir);
        // Serial.print(", Motor 1: ");
        // Serial.print(motor1Speed);
        // Serial.print(" Direction: ");
        // Serial.print(motor1Dir);

        dvr2.move(motor2Speed, motor2Dir);
        // Serial.print(", Motor 2: ");
        // Serial.print(motor2Speed);
        // Serial.print(" Direction: ");
        // Serial.println(motor2Dir);

    } else if (motor2Speed!=0 && motor1Speed==0) {
        // Move motors with calculated speeds and directions

        motor1Speed = DYNAMIC_BRAKE;
        motor1Dir = motor2Dir;
        // Serial.println("Engaging Motor 1 Brake");

        dvr1.move(motor1Speed, motor1Dir);
        // Serial.print("Motor 1: ");
        // Serial.print(motor1Speed);
        // Serial.print(" Direction: ");
        // Serial.println(motor1Dir);

        dvr2.move(motor2Speed, motor2Dir);
        // Serial.print("Motor 2: ");
        // Serial.print(motor2Speed);
        // Serial.print(" Direction: ");
        // Serial.println(motor2Dir);

    } else{
    // Move motors with calculated speeds and directions
        dvr1.move(motor1Speed, motor1Dir);
        // Serial.print("Motor 1: ");
        // Serial.print(motor1Speed);
        // Serial.print(" Direction: ");
        // Serial.println(motor1Dir);
        
        dvr2.move(motor2Speed, motor2Dir);
        // Serial.print("Motor 2: ");
        // Serial.print(motor2Speed);
        // Serial.print(" Direction: ");
        // Serial.println(motor2Dir);
    }
}

void Move::stop() {
    dvr1.move(0, 0);
    dvr2.move(0, 0);
}

void Move::brake() {
    if (vel1 > 0){
        dvr1.move(STATIC_BRAKE, 0);
        Serial.println("Motor 1: 2 Direction: 0");   
    }
    else if (vel1 < 0){
        dvr1.move(STATIC_BRAKE, 1);
        Serial.println("Motor 1: 2 Direction: 1");
    }
    else{
        dvr1.move(0, 0);
        Serial.println("Motor 1: 0 Direction: 0");
    }
    if (vel2 > 0){
        dvr2.move(STATIC_BRAKE, 0);
        Serial.println("Motor 2: 2 Direction: 0");
    }
    else if (vel2 < 0){
        dvr2.move(STATIC_BRAKE, 1);
        Serial.println("Motor 2: 2 Direction: 1");
    }
    else{
        dvr2.move(0, 0);
        Serial.println("Motor 2: 0 Direction: 0");
    }
}

int Move::returnPosX(){
    // Get raw encoder values
    long angle1 = enc1.getTotalAngle();
    long angle2 = enc2.getTotalAngle();

    // We'll multiply by a scaling factor to get to physical units (mm * 10)
    int posX = (float(angle1) + float(angle2)) * SCALE_FACTOR;
    return posX;
}

int Move::returnPosY(){
    // Get raw encoder values
    long angle1 = enc1.getTotalAngle();
    long angle2 = enc2.getTotalAngle();
    
    // We'll multiply by a scaling factor to get to physical units (mm * 10)
    int posY = (float(angle1) - float(angle2)) * SCALE_FACTOR;
    return posY;
}
