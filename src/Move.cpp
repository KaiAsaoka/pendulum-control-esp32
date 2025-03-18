#include "Move.h"
#include "Encoder.h"
#include "Driver.h"


Move::Move(Encoder enc1, Encoder enc2, Driver dvr1, Driver dvr2)
    : enc1(enc1), enc2(enc2), dvr1(dvr1), dvr2(dvr2) {}

void Move::moveX(int speed, bool direction) {
    dvr1.move(speed, direction);
    dvr2.move(speed, direction);
}
    
void Move::moveY(int speed, bool direction) {  
    dvr1.move(speed, direction);
    dvr2.move(speed, -direction);
}

