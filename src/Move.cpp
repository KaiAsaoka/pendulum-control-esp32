#include "Move.h"
#include "Encoder.h"
#include "Driver.h"
#include <cmath>
#include <algorithm>
#include <Arduino.h>

#define STATIC_BRAKE 3
#define DYNAMIC_BRAKE 3

Move::Move(Driver& dvr1, Driver& dvr2, Encoder& enc1, Encoder& enc2) : dvr1(dvr1), dvr2(dvr2), enc1(enc1), enc2(enc2) {}

// void Move::moveX(int speed, bool direction) {
//     dvr1.move(speed, direction);
//     dvr2.move(speed, direction);
// }
    
// void Move::moveY(int speed, bool direction) {  
//     dvr1.move(speed, direction);
//     dvr2.move(speed, !direction);
// }

void Move::moveXY(int speedX, bool directionX, int speedY, bool directionY) {
    // Convert boolean direction to multiplier (-1 or 1)
    int dirX = !directionX ? 1 : -1;
    int dirY = !directionY ? 1 : -1;
    
    // Calculate motor speeds by combining X and Y components
    int motor1Speed = (speedX * dirX) + (speedY * dirY);
    int motor2Speed = (speedX * dirX) - (speedY * dirY);
    
    vel1 = motor1Speed;
    vel2 = motor2Speed;

    // Determine motor directions based on calculated speeds
    bool motor1Dir = motor1Speed >= 0;
    bool motor2Dir = motor2Speed >= 0;

    // Get absolute values for motor speeds
    motor1Speed = abs(motor1Speed);
    motor2Speed = abs(motor2Speed);
    
    // Scale speeds if they exceed maximum
    int maxSpeed = max(motor1Speed, motor2Speed);
    if (maxSpeed > 255) {
        float scaleFactor = 255.0 / maxSpeed;
        motor1Speed = round(motor1Speed * scaleFactor);
        motor2Speed = round(motor2Speed * scaleFactor);
    }
    

    if (motor1Speed!=0 && motor2Speed==0) {
        // Move motors with calculated speeds and directions
        motor2Speed = DYNAMIC_BRAKE;
        motor2Dir = motor1Dir;
        Serial.println("Engaging Motor 2 Brake");

        dvr1.move(motor1Speed, motor1Dir);
        Serial.print(", Motor 1: ");
        Serial.print(motor1Speed);
        Serial.print(" Direction: ");
        Serial.print(motor1Dir);

        dvr2.move(motor2Speed, motor2Dir);
        Serial.print(", Motor 2: ");
        Serial.print(motor2Speed);
        Serial.print(" Direction: ");
        Serial.println(motor2Dir);

    } else if (motor2Speed!=0 && motor1Speed==0) {
        // Move motors with calculated speeds and directions

        motor1Speed = DYNAMIC_BRAKE;
        motor1Dir = motor2Dir;
        Serial.println("Engaging Motor 1 Brake");

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

    } else{
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

float Move::returnPosX(){
    // Get raw encoder values
    long angle1 = enc1.getTotalAngle();
    long angle2 = enc2.getTotalAngle();

    // Convert to float and scale appropriately
    // The sqrt(2) factor comes from the mechanical coupling of the motors
    // We'll multiply by a scaling factor to get to physical units (mm)
    const float SCALE_FACTOR = 0.1; // Adjust this based on your mechanical setup
    float posX = (float(angle1) + float(angle2)) * SCALE_FACTOR / sqrt(2.0f);
    return posX;
}

float Move::returnPosY(){
    // Get raw encoder values
    long angle1 = enc1.getTotalAngle();
    long angle2 = enc2.getTotalAngle();
    
    // Convert to float and scale appropriately
    // The sqrt(2) factor comes from the mechanical coupling of the motors
    // We'll multiply by a scaling factor to get to physical units (mm)
    const float SCALE_FACTOR = 0.1; // Adjust this based on your mechanical setup
    float posY = (float(angle1)-(angle2)) * SCALE_FACTOR / sqrt(2.0f);
    return posY;
}

