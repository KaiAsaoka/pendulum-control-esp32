#ifndef PID_H
#define PID_H
#include <tuple>

struct pidParams {
  int p;
  int i;
  int d;
  int lpf;
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

    void reset();
    void readNewGains(pidParams newParams);
    
private:
    int kp;
    int ki;
    int kd;
    int lpf_gain;
    int int_cutoff;
    float previous_error;
    float integral;
    float d_term;
};

#endif // PID_H