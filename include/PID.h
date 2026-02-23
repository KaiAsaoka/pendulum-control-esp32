#ifndef PID_H
#define PID_H
#include <tuple>

struct pidParams {
  int p;
  int i;
  int d;
  int lpf;
  int iCutoff;
  int zeroOffset;
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

    void reset();
    void readNewGains(pidParams newParams);
    pidParams currentGains();
    
private:
    float kp;
    float ki;
    float kd;
    float lpf_gain;
    float int_cutoff;
    float previous_error;
    float integral;
    float d_term;
};

#endif // PID_H