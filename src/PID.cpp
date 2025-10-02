#include "PID.h"
#include <Arduino.h>   // for constrain()
#include <tuple>

PID::PID(float kp, float ki, float kd, float lpf_gain, float int_cutoff)
: kp(kp), ki(ki), kd(kd),
  previous_error(0.0f), integral(0.0f), d_term(0.0f),
  lpf_gain(lpf_gain), int_cutoff(int_cutoff) {}

// Time-aware PID: dt in seconds
std::tuple<float, float, float, float> PID::calculate(float error, float dt) {
    // Guard against bad dt
    if (dt <= 0.0f) dt = 1.0f;

    // Integral with windup clamp (units: error·seconds)
    integral += error * dt;
    integral = constrain(integral, -int_cutoff, int_cutoff);

    // Derivative (per second)
    float d_raw = (error - previous_error) / dt;
    previous_error = error;

    // Terms
    float p_term = kp * error;
    float i_term = ki * integral;

    // Low-pass the D term (alpha in [0,1])
    float alpha = constrain(lpf_gain, 0.0f, 1.0f);
    float d_unf = kd * d_raw;
    d_term = alpha * d_term + (1.0f - alpha) * d_unf;

    float u = p_term + i_term + d_term;
    return {p_term, i_term, d_term, u};
}

// Compatibility: dt defaults to 1.0 s if you call the 1-arg version
std::tuple<float, float, float, float> PID::calculate(float error) {
    return calculate(error, 1.0f);
}

void PID::reset_I() {
    integral = 0.0f;
    previous_error = 0.0f;
    d_term = 0.0f;
}
