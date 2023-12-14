# usage:
```
#include "PID_v1.h"

int main(void){
  float kp = 1, ki = 1, kd = 1, min = 0, max = 1000;
  uint32_t ts = 100;
  PID* a = initialize(kp, ki, kd, ts, min, max);

  while(1){
    float setpoint, feedback;
    a->compute(&setpoint, &feedback, a);
    TIM1->CCR1 = (uint32_t)a->output;
  }
}
```
