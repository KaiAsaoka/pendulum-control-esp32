#include "Move.h"
#include "Encoder.h"
#include "Driver.h"


Move::Move(Driver dvr1, Driver dvr2) : dvr1(dvr1), dvr2(dvr2) {}

void Move::moveX(int speed, bool direction) {
    dvr1.move(speed, direction);
    dvr2.move(speed, direction);
}
    
void Move::moveY(int speed, bool direction) {  
    dvr1.move(speed, direction);
    dvr2.move(speed, -direction);
}

void Move::stop() {
    dvr1.move(0, 0);
    dvr2.move(0, 0);
}

