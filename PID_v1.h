/*
 * PID_v1.h
 *
 *  Created on: Dec 10, 2023
 *      Author: udin
 */

#ifndef INC_PID_V1_H_
#define INC_PID_V1_H_

#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "math.h"
#include "stdlib.h"
#include "stdbool.h"


typedef struct _PID_{
	uint32_t start_t, ts, dt, last_t;
	float error, d_input, last_feedback;
	float out_sum, output;
	float kp, ki, kd;
	float out_min, out_max;
	void(*compute)(float*, float*, struct _PID_*);
} PID;

PID *initialize(float Kp, float Ki, float Kd, uint32_t ts, float out_min, float out_max);



#endif /* INC_PID_V1_H_ */
