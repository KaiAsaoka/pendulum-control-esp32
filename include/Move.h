#ifndef Move_H
#define Move_H

#include <Encoder.h>
#include <Driver.h>

struct motorPWMs {
    int xPWM;
    int yPWM;
};

class Move {
public:
    Move(Driver& dvr1, Driver& dvr2, Encoder& enc1, Encoder& enc2);

    // void moveX(int speed, bool direction);
    
    // void moveY(int speed, bool direction);

    void moveXY(int speedX, int speedY);

    void brake();
    
    void stop();

    int returnPosX();

    int returnPosY();

    float returnVel();

    


private:

    Driver& dvr1;
    Driver& dvr2;
    Encoder& enc1;
    Encoder& enc2;
    float sumAngle1;
    float sumAngle2;
    float fullRot1;
    float fullRot2;
    float xPos;
    float yPos;
    float vel1;
    float vel2;

};

#endif // Driver_H