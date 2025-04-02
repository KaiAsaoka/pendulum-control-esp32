#ifndef PID_H
#define PID_H
#include <tuple>


class PID {
public:
    PID(float kp, float ki, float kd);
    std::tuple<float, float, float, float> calculate(float error);
    void reset_I();
private:
    float kp;
    float ki;
    float kd;
    float previous_error;
    float integral;
};

#endif // PID_H