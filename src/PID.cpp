#include "PID.h"
#include <Arduino.h>   // for constrain()
#include <tuple>

PID::PID(pidParams pidParams)
: kp(pidParams.p / 1000.0f),
  ki(pidParams.i / 1000.0f),
  kd(pidParams.d / 1000.0f),
  alpha_p(pidParams.alpha_p / 1000.0f),
  alpha_i(pidParams.alpha_i / 1000.0f),
  alpha_d(pidParams.alpha_d / 1000.0f),
  alpha_o(pidParams.alpha_o / 1000.0f),
  int_cutoff(pidParams.iCutoff / 100.0f){}

// Time-aware PID: dt in seconds
pidOutputs PID::calculate(float error, float dt) {
    // Guard against bad dt
    if (dt <= 0.0f) dt = 1.0f;

    // Integral with windup clamp (units: error·seconds)
    integral += error * dt;
    integral = constrain(integral, -int_cutoff, int_cutoff);

    // Derivative (per second)
    derivative = (error - previous_error) / dt;
    previous_error = error;

    // Raw Terms
    float p_new = kp * error;
    float i_new = ki * integral;
    float d_new = kd * derivative;

    // Low-pass filter (alpha in [0, 1])
    p_term = lowPassFilter(alpha_p, p_term, p_new);
    i_term = lowPassFilter(alpha_i, i_term, i_new); 
    d_term = lowPassFilter(alpha_d, d_term, d_new);

    // Combine terms
    float sum_new = p_term + i_term + d_term;

    // Low-pass filter output
    sum = lowPassFilter(alpha_o, sum, sum_new);

    pidOutputs outputs = {p_term, i_term, d_term, (int)sum};

    return outputs;
}

// Compatibility: dt defaults to 1.0 s if you call the 1-arg version
pidOutputs PID::calculate(float error) {
    return calculate(error, 1.0f);
}

float PID::lowPassFilter(float alpha, float prev_val, float new_val) {
    alpha = constrain(alpha, 0.0f, 1.0f); // Constrain to [0, 1]
    return alpha * prev_val + (1.0f - alpha) * new_val;
}

void PID::reset() {
    integral = 0.0f;
    previous_error = 0.0f;
    p_term = 0.0f;
    i_term = 0.0f;
    d_term = 0.0f;
    sum = 0.0f;
}

void PID::readNewGains(pidParams newParams) {
    kp = newParams.p / 1000.0f;
    ki = newParams.i / 1000.0f;
    kd = newParams.d / 1000.0f;
    alpha_p = newParams.alpha_p / 1000.0f;
    alpha_i = newParams.alpha_i / 1000.0f;
    alpha_d = newParams.alpha_d / 1000.0f;
    alpha_o = newParams.alpha_o / 1000.0f;
    int_cutoff = newParams.iCutoff / 100.0f;
}

pidParams PID::currentGains() {
    pidParams currentGains = {
        (int)(kp*1000),
        (int)(ki*1000),
        (int)(kd*1000),
        (int)(alpha_p*1000),
        (int)(alpha_i*1000),
        (int)(alpha_d*1000),
        (int)(alpha_o*1000),
        (int)(int_cutoff*100),
    };
    return currentGains;
}