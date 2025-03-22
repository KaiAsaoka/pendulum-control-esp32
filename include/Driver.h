#ifndef Driver_H
#define Driver_H

class Driver {
public:
    Driver(int pwm, int dir);
    void begin();
    void move(int speed, bool direction);

private:
    int pwm;
    int dir;
    float speed;
    float direction;
};

#endif // Driver_H