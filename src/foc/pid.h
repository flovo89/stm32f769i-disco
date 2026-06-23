#pragma once

typedef struct {
	float kp;
	float ki;
	float integrator;
	float out_min;
	float out_max;
} pid_ctrl_t;

void  pid_init(pid_ctrl_t *p, float kp, float ki, float out_min, float out_max);
void  pid_reset(pid_ctrl_t *p);
float pid_update(pid_ctrl_t *p, float error, float dt);
