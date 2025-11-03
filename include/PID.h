#ifndef PID_H
#define PID_H
#include <tuple>


class PID {
public:
    //PID(float kp, float ki, float kd);
    PID(volatile float& kp, volatile float& ki, volatile float& kd, volatile float& lpf_gain, volatile float& int_cutoff);
    
    std::tuple<float,float,float,float> calculate(float error, float dt);

    std::tuple<float, float, float, float> calculate(float error);
    void reset_I();
private:
    volatile float& kp;
    volatile float& ki;
    volatile float& kd;
    volatile float& lpf_gain;
    volatile float& int_cutoff;
    float previous_error;
    float integral;
    float d_term;
};

#endif // PID_H