#ifndef Move_H
#define Move_H

#include <Encoder.h>
#include <Driver.h>

class Move {
public:
    Move(Driver dvr1, Driver dvr2);

    // void moveX(int speed, bool direction);
    
    // void moveY(int speed, bool direction);

    void moveXY(int speedX, bool directionX, int speedY, bool directionY);

    void brake();
    
    void stop();

    float returnPos();

    float returnVel();

    


private:

    Driver dvr1;
    Driver dvr2;
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