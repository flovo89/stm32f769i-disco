#include "pid.h"

#include <stdbool.h>

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
	float unclamped = p->kp * error + p->integrator;
	float out = unclamped;

	if (out > p->out_max) {
		out = p->out_max;
	} else if (out < p->out_min) {
		out = p->out_min;
	}

	/* Conditional integration anti-windup: only accumulate while the
	 * output isn't saturated, or while the error is pulling it back out
	 * of saturation. Avoids the integrator sitting pegged at out_max/min
	 * independently of the proportional term. */
	bool driving_into_high_sat = (unclamped > p->out_max) && (error > 0.0f);
	bool driving_into_low_sat  = (unclamped < p->out_min) && (error < 0.0f);

	if (!driving_into_high_sat && !driving_into_low_sat) {
		p->integrator += p->ki * error * dt;
	}

	return out;
}
