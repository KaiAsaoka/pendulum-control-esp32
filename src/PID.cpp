#include "PID.h"
#include <Arduino.h>   // for constrain()
#include <tuple>

PID::PID(pidParams pidParams)
: kp(static_cast<float>(pidParams.p/1000)), 
  ki(static_cast<float>(pidParams.i/1000)), 
  kd(static_cast<float>(pidParams.d)/1000),
  previous_error(0.0f), integral(0.0f), d_term(0.0f),
  lpf_gain(static_cast<float>(pidParams.lpf/1000)), 
  int_cutoff(static_cast<float>(pidParams.iCutoff/1000)) {}

// Time-aware PID: dt in seconds
pidOutputs PID::calculate(float error, float dt) {
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

    float sum = p_term + i_term + d_term;

    pidOutputs outputs = {p_term, i_term, d_term, sum};

    return outputs;
}

// Compatibility: dt defaults to 1.0 s if you call the 1-arg version
pidOutputs PID::calculate(float error) {
    return calculate(error, 1.0f);
}

void PID::reset() {
    integral = 0.0f;
    previous_error = 0.0f;
    d_term = 0.0f;
}

void PID::readNewGains(pidParams newParams) {
    kp = static_cast<float>(newParams.p/1000);
    ki = static_cast<float>(newParams.i/1000);
    kd = static_cast<float>(newParams.d/1000);
    lpf_gain = static_cast<float>(newParams.lpf/1000);
    int_cutoff = static_cast<float>(newParams.iCutoff/1000);
}