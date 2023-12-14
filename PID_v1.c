/*
 * PID_v1.c
 *
 *  Created on: Dec 10, 2023
 *      Author: udin
 */

#include "PID_v1.h"

static void compute(float *setpoint, float *feedback, PID *self);

PID *initialize(float Kp, float Ki, float Kd, uint32_t ts, float out_min, float out_max){
	PID *self;
	self = malloc(sizeof(self));
	self->kp = Kp;
	self->ki = Ki;
	self->kd = Kd;
	self->ts = ts;
	self->out_min = out_min;
	self->out_max = out_max;
	self->compute = compute;
	return self;
}

static void compute(float *setpoint, float *feedback, PID *self){
	self->start_t = HAL_GetTick();
	self->dt = self->start_t - self->last_t;
	if(self->dt >= self->ts){
		self->error = (*setpoint) - (*feedback);
		self->d_input = (*feedback) - self->last_feedback;

		self->out_sum += self->error * self->ki;
		self->out_sum -= self->d_input * self->kp;

		if(self->out_sum > self->out_max) self->out_sum = self->out_max;
		else if(self->out_sum < self->out_min) self->out_sum = self->out_min;

		self->output += self->out_sum - self->kd * self->d_input;

		if(self->output > self->out_max) self->output = self->out_max;
		else if(self->output < self->out_min) self->output = self->out_min;

		self->last_feedback = *feedback;
		self->last_t = self->start_t;
	}
}
