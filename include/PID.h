#ifndef PID_H
#define PID_H
#include <tuple>


class PID {
public:
    //PID(float kp, float ki, float kd);
<<<<<<< HEAD
    PID(float kp, float ki, float kd, float lpf_gain, float int_cutoff);
=======
    PID(volatile float& kp, volatile float& ki, volatile float& kd, volatile float& lpf_gain, volatile float& int_cutoff);
    
    std::tuple<float,float,float,float> calculate(float error, float dt);

>>>>>>> 7693d39 (Changed loop variables to global to allow for the telemetry task to access them)
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