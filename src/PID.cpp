#include "PID.h"
#include <tuple>

PID::PID(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd), previous_error(0), integral(0) {}

std::tuple<float, float, float, float> PID::calculate(float error) {
    integral += error;
    float derivative = error - previous_error;
    previous_error = error;
    
    float p_term = kp * error;
    float i_term = ki * integral;
    float d_term = kd * derivative;
    float output = p_term + i_term + d_term;
    
    return std::make_tuple(output, p_term, i_term, d_term);
}