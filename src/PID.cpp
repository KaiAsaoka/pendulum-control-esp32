#include "PID.h"
#include <tuple>
#include <Arduino.h>

PID::PID(float kp, float ki, float kd, float lpf_gain, float int_cutoff) : kp(kp), ki(ki), kd(kd), previous_error(0), integral(0), d_term(0), lpf_gain(lpf_gain), int_cutoff(int_cutoff){}
//PID::PID(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd), previous_error(0), integral(0), d_term(0), lpf_gain(0.75), int_cutoff(100000) {}

std::tuple<float, float, float, float> PID::calculate(float error) {

    integral += error;
    integral = constrain(integral, -int_cutoff, int_cutoff); // limit integral term to prevent windup

    float derivative = error - previous_error;
    previous_error = error;

    float p_term = kp * error;
    float i_term = ki * integral;

    float prev_d_term = d_term; // record for filter
    float d_term = kd * derivative;
    d_term = lpf_gain * (prev_d_term) + (1 - lpf_gain) * d_term; // exponential low-pass filter

    float output = p_term + i_term + d_term;
    
    return std::make_tuple(output, p_term, i_term, d_term);
}

void PID::reset_I() {
    integral = 0;
}