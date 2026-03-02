#ifndef PID_H
#define PID_H
#include <tuple>

struct pidParams {
  int p;
  int i;
  int d;
  int alpha_p;
  int alpha_i;
  int alpha_d;
  int alpha_o;
  int iCutoff;
};

// should these be ints?
struct pidOutputs {
  float pOut;
  float iOut;
  float dOut;
  int output;
};

class PID {
public:
    //PID(float kp, float ki, float kd);
    PID(pidParams pidParams);
    
    pidOutputs calculate(float error, float dt);
    pidOutputs calculate(float error);

    float lowPassFilter(float alpha, float new_d, float prev_d);
    void reset();
    void readNewGains(pidParams newParams);
    pidParams currentGains();
    
private:
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    float alpha_p = 0.0f;
    float alpha_i = 0.0f;
    float alpha_d = 0.0f;
    float alpha_o = 0.0f;
    float int_cutoff = 0.0f;
    float previous_error = 0.0f;
    float integral = 0.0f;
    float derivative = 0.0f;
    float p_term = 0.0f;
    float i_term = 0.0f;
    float d_term = 0.0f;
    float sum = 0.0f;
};

#endif // PID_H