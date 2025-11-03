#include "PID.h"
#include <tuple>
#include <Arduino.h>

PID::PID(volatile float& kp, volatile float& ki, volatile float& kd, volatile float& lpf_gain, volatile float& int_cutoff)
: kp(kp), ki(ki), kd(kd),
  previous_error(0.0f), integral(0.0f), d_term(0.0f),
  lpf_gain(lpf_gain), int_cutoff(int_cutoff) {}

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