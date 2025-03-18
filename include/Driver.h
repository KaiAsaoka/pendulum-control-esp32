#ifndef Driver_H
#define Driver_H

class Driver {
public:
    Driver(int pwm, int dir);
    void begin();
    float move(int speed, int direction);

private:
    float pwm;
    float dir;
    float speed;
    float direction;
};

#endif // Driver_H