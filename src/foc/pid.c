#include "pid.h"

void pid_init(pid_ctrl_t *p, float kp, float ki, float out_min, float out_max)
{
	p->kp        = kp;
	p->ki        = ki;
	p->integrator = 0.0f;
	p->out_min   = out_min;
	p->out_max   = out_max;
}

void pid_reset(pid_ctrl_t *p)
{
	p->integrator = 0.0f;
}

float pid_update(pid_ctrl_t *p, float error, float dt)
{
	p->integrator += p->ki * error * dt;

	/* Integrator anti-windup clamp */
	if (p->integrator > p->out_max) {
		p->integrator = p->out_max;
	} else if (p->integrator < p->out_min) {
		p->integrator = p->out_min;
	}

	float out = p->kp * error + p->integrator;

	if (out > p->out_max) {
		out = p->out_max;
	} else if (out < p->out_min) {
		out = p->out_min;
	}

	return out;
}
