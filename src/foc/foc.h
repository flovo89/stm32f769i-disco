#pragma once

#include "pid.h"
#include <stdbool.h>
#include <stdint.h>

/* ─── Tunable parameters ────────────────────────────────────────────────── */
#define FOC_CONTROL_HZ       10000      /* Control loop rate */
#define FOC_CONTROL_DT       (1.0f / FOC_CONTROL_HZ)
#define FOC_PWM_HZ           20000      /* PWM switching frequency */

#define FOC_POLE_PAIRS       7          /* Motor pole pair count */
#define FOC_MAX_CURRENT_A    10.0f      /* Peak phase current limit [A] */
#define FOC_MAX_SPEED_RPM    3000.0f

/* Alignment: apply Id_align at theta=0 for align_ms */
#define FOC_ALIGN_CURRENT_A  1.0f
#define FOC_ALIGN_MS         500

/* Default current PI gains (tune per motor) */
#define FOC_KP_CURRENT       0.5f
#define FOC_KI_CURRENT       50.0f

/* Default speed PI gains */
#define FOC_KP_SPEED         0.05f
#define FOC_KI_SPEED         1.0f

/* ─── State machine ─────────────────────────────────────────────────────── */
typedef enum {
	FOC_STATE_IDLE,
	FOC_STATE_ALIGNING,
	FOC_STATE_RUNNING,
	FOC_STATE_ERROR,
} foc_state_t;

typedef enum {
	FOC_MODE_TORQUE,   /* Direct Iq reference */
	FOC_MODE_SPEED,    /* Speed loop sets Iq */
} foc_mode_t;

typedef struct {
	foc_state_t state;
	foc_mode_t  mode;

	/* Measured */
	float ia, ib, ic;        /* Phase currents [A] */
	float id, iq;            /* d/q currents [A] */
	float ialpha, ibeta;     /* α/β currents [A] */
	float theta_e;           /* Electrical angle [rad] */
	float omega_e;           /* Electrical angular velocity [rad/s] */
	float speed_rpm;

	/* References */
	float id_ref;
	float iq_ref;
	float speed_ref;

	/* Outputs */
	float vd, vq;
	float valpha, vbeta;
	float da, db, dc;        /* Duty cycles [0..1] */
	float vbus;              /* DC bus voltage [V] (configured value) */

	/* Controllers */
	pid_ctrl_t pid_id;
	pid_ctrl_t pid_iq;
	pid_ctrl_t pid_speed;

	/* Alignment */
	int   align_ticks_left;

	/* Statistics */
	uint32_t overcurrent_count;
	uint32_t loop_count;
} foc_ctx_t;

/* ─── API ───────────────────────────────────────────────────────────────── */
void foc_init(foc_ctx_t *foc, float vbus_v);
void foc_reset(foc_ctx_t *foc);

/*
 * Called from the control thread at FOC_CONTROL_HZ.
 * theta_mech: mechanical angle [rad], wrapping [0, 2π)
 * omega_mech: mechanical angular velocity [rad/s]
 */
void foc_step(foc_ctx_t *foc, float ia, float ib,
              float theta_mech, float omega_mech);

void foc_enable(foc_ctx_t *foc, bool enable);
void foc_set_mode(foc_ctx_t *foc, foc_mode_t mode);
void foc_set_speed_ref(foc_ctx_t *foc, float rpm);
void foc_set_torque_ref(foc_ctx_t *foc, float iq_amps);
void foc_tune_current_pid(foc_ctx_t *foc, float kp, float ki);
void foc_tune_speed_pid(foc_ctx_t *foc, float kp, float ki);

/* Math helpers (exposed for testing) */
void foc_clarke(float ia, float ib, float *alpha, float *beta);
void foc_park(float alpha, float beta, float theta,
              float *d, float *q);
void foc_inv_park(float vd, float vq, float theta,
                  float *alpha, float *beta);
void foc_svpwm(float valpha, float vbeta, float vbus,
               float *da, float *db, float *dc);

const char *foc_state_str(foc_state_t s);
