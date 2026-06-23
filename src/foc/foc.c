#include "foc.h"

#include <math.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(foc, LOG_LEVEL_INF);

#define PI      3.14159265358979f
#define TWO_PI  6.28318530717959f
#define SQRT3   1.73205080756888f
#define SQRT3_2 0.86602540378444f  /* sqrt(3)/2 */

/* ─── Transforms ────────────────────────────────────────────────────────── */

/*
 * Clarke transform: 3-phase (ia, ib) → α/β
 * Assumes balanced currents: ic = -(ia + ib)
 * Amplitude-invariant convention.
 */
void foc_clarke(float ia, float ib, float *alpha, float *beta)
{
	*alpha = ia;
	*beta  = (ia + 2.0f * ib) / SQRT3;
}

/*
 * Park transform: α/β → d/q rotating frame.
 * theta: electrical angle [rad]
 */
void foc_park(float alpha, float beta, float theta,
              float *d, float *q)
{
	float c = cosf(theta);
	float s = sinf(theta);

	*d =  alpha * c + beta * s;
	*q = -alpha * s + beta * c;
}

/*
 * Inverse Park: d/q → α/β
 */
void foc_inv_park(float vd, float vq, float theta,
                  float *alpha, float *beta)
{
	float c = cosf(theta);
	float s = sinf(theta);

	*alpha = vd * c - vq * s;
	*beta  = vd * s + vq * c;
}

/*
 * SVPWM via min-max zero-sequence injection.
 * Produces duty cycles in [0, 1] for centre-aligned or edge-aligned PWM.
 * vbus: DC link voltage [V]
 */
void foc_svpwm(float valpha, float vbeta, float vbus,
               float *da, float *db, float *dc)
{
	/* Inverse Clarke → three reference voltages */
	float va =  valpha;
	float vb = -0.5f * valpha + SQRT3_2 * vbeta;
	float vc = -0.5f * valpha - SQRT3_2 * vbeta;

	/* Zero-sequence injection equivalent to SVPWM */
	float vmax = va > vb ? (va > vc ? va : vc) : (vb > vc ? vb : vc);
	float vmin = va < vb ? (va < vc ? va : vc) : (vb < vc ? vb : vc);
	float vzero = -0.5f * (vmax + vmin);

	va += vzero;
	vb += vzero;
	vc += vzero;

	/* Normalise to [0, 1] */
	float half = vbus * 0.5f;

	*da = (va + half) / vbus;
	*db = (vb + half) / vbus;
	*dc = (vc + half) / vbus;

	/* Clamp — should not be needed with linear modulation but be safe */
#define CLAMP01(x) ((x) < 0.0f ? 0.0f : ((x) > 1.0f ? 1.0f : (x)))
	*da = CLAMP01(*da);
	*db = CLAMP01(*db);
	*dc = CLAMP01(*dc);
#undef CLAMP01
}

/* ─── Init ──────────────────────────────────────────────────────────────── */

void foc_init(foc_ctx_t *foc, float vbus_v)
{
	memset(foc, 0, sizeof(*foc));

	foc->vbus  = vbus_v;
	foc->state = FOC_STATE_IDLE;
	foc->mode  = FOC_MODE_SPEED;

	/* Voltage limits: ±Vbus/√3 is the linear modulation limit */
	float vlim = vbus_v / SQRT3;

	pid_init(&foc->pid_id,    FOC_KP_CURRENT, FOC_KI_CURRENT, -vlim, vlim);
	pid_init(&foc->pid_iq,    FOC_KP_CURRENT, FOC_KI_CURRENT, -vlim, vlim);
	pid_init(&foc->pid_speed, FOC_KP_SPEED,   FOC_KI_SPEED,
	         -FOC_MAX_CURRENT_A, FOC_MAX_CURRENT_A);
}

void foc_reset(foc_ctx_t *foc)
{
	foc->state = FOC_STATE_IDLE;
	foc->da = foc->db = foc->dc = 0.5f;  /* 50% = zero voltage */
	pid_reset(&foc->pid_id);
	pid_reset(&foc->pid_iq);
	pid_reset(&foc->pid_speed);
}

/* ─── Control step (called at FOC_CONTROL_HZ) ──────────────────────────── */

void foc_step(foc_ctx_t *foc, float ia, float ib,
              float theta_mech, float omega_mech)
{
	foc->loop_count++;
	foc->ia = ia;
	foc->ib = ib;
	foc->ic = -(ia + ib);

	/* Electrical angle */
	foc->theta_e = fmodf(theta_mech * FOC_POLE_PAIRS, TWO_PI);
	if (foc->theta_e < 0.0f) {
		foc->theta_e += TWO_PI;
	}

	foc->omega_e   = omega_mech * FOC_POLE_PAIRS;
	foc->speed_rpm = omega_mech * (60.0f / TWO_PI);

	switch (foc->state) {

	/* ── IDLE: all PWM at 50%, motor driver disabled externally ── */
	case FOC_STATE_IDLE:
		foc->da = foc->db = foc->dc = 0.5f;
		return;

	/* ── ALIGNING: open-loop current injection to seat rotor at θ=0 ── */
	case FOC_STATE_ALIGNING: {
		float align_angle = 0.0f;  /* Drive d-axis to 0 rad */
		float valpha, vbeta;

		foc_inv_park(FOC_ALIGN_CURRENT_A * foc->pid_id.kp, 0.0f,
		             align_angle, &valpha, &vbeta);
		foc_svpwm(valpha, vbeta, foc->vbus,
		          &foc->da, &foc->db, &foc->dc);

		if (--foc->align_ticks_left <= 0) {
			LOG_INF("Alignment done");
			foc->state = FOC_STATE_RUNNING;
			pid_reset(&foc->pid_id);
			pid_reset(&foc->pid_iq);
			pid_reset(&foc->pid_speed);
		}
		return;
	}

	/* ── ERROR: output zero voltage ── */
	case FOC_STATE_ERROR:
		foc->da = foc->db = foc->dc = 0.5f;
		return;

	case FOC_STATE_RUNNING:
		break;
	}

	/* ── Overcurrent protection ── */
	if (fabsf(ia) > FOC_MAX_CURRENT_A || fabsf(ib) > FOC_MAX_CURRENT_A) {
		foc->overcurrent_count++;
		foc->state = FOC_STATE_ERROR;
		foc->da = foc->db = foc->dc = 0.5f;
		LOG_ERR("Overcurrent: ia=%.2f ib=%.2f", (double)ia, (double)ib);
		return;
	}

	/* ── Clarke transform ── */
	foc_clarke(ia, ib, &foc->ialpha, &foc->ibeta);

	/* ── Park transform ── */
	foc_park(foc->ialpha, foc->ibeta, foc->theta_e, &foc->id, &foc->iq);

	/* ── Speed loop (sets Iq reference) ── */
	if (foc->mode == FOC_MODE_SPEED) {
		float speed_err = foc->speed_ref - foc->speed_rpm;

		foc->iq_ref = pid_update(&foc->pid_speed, speed_err, FOC_CONTROL_DT);
	}

	/* ── Current loops ── */
	foc->vd = pid_update(&foc->pid_id, foc->id_ref - foc->id, FOC_CONTROL_DT);
	foc->vq = pid_update(&foc->pid_iq, foc->iq_ref - foc->iq, FOC_CONTROL_DT);

	/* ── Feed-forward decoupling (back-EMF compensation) ── */
	foc->vd -= foc->omega_e * foc->pid_iq.kp * foc->iq;
	foc->vq += foc->omega_e * foc->pid_id.kp * foc->id;

	/* ── Inverse Park + SVPWM ── */
	foc_inv_park(foc->vd, foc->vq, foc->theta_e, &foc->valpha, &foc->vbeta);
	foc_svpwm(foc->valpha, foc->vbeta, foc->vbus,
	          &foc->da, &foc->db, &foc->dc);
}

/* ─── State control ─────────────────────────────────────────────────────── */

void foc_enable(foc_ctx_t *foc, bool enable)
{
	if (enable) {
		if (foc->state == FOC_STATE_IDLE ||
		    foc->state == FOC_STATE_ERROR) {
			foc->align_ticks_left =
				(FOC_ALIGN_MS * FOC_CONTROL_HZ) / 1000;
			foc->state = FOC_STATE_ALIGNING;
			LOG_INF("Aligning rotor (%d ms)...", FOC_ALIGN_MS);
		}
	} else {
		foc_reset(foc);
	}
}

void foc_set_mode(foc_ctx_t *foc, foc_mode_t mode)
{
	foc->mode = mode;
	pid_reset(&foc->pid_speed);
}

void foc_set_speed_ref(foc_ctx_t *foc, float rpm)
{
	float clamped = rpm;

	if (clamped >  FOC_MAX_SPEED_RPM) clamped =  FOC_MAX_SPEED_RPM;
	if (clamped < -FOC_MAX_SPEED_RPM) clamped = -FOC_MAX_SPEED_RPM;

	foc->speed_ref = clamped;
}

void foc_set_torque_ref(foc_ctx_t *foc, float iq_amps)
{
	float clamped = iq_amps;

	if (clamped >  FOC_MAX_CURRENT_A) clamped =  FOC_MAX_CURRENT_A;
	if (clamped < -FOC_MAX_CURRENT_A) clamped = -FOC_MAX_CURRENT_A;

	foc->iq_ref = clamped;
}

void foc_tune_current_pid(foc_ctx_t *foc, float kp, float ki)
{
	float vlim = foc->vbus / SQRT3;

	pid_init(&foc->pid_id, kp, ki, -vlim, vlim);
	pid_init(&foc->pid_iq, kp, ki, -vlim, vlim);
}

void foc_tune_speed_pid(foc_ctx_t *foc, float kp, float ki)
{
	pid_init(&foc->pid_speed, kp, ki,
	         -FOC_MAX_CURRENT_A, FOC_MAX_CURRENT_A);
}

const char *foc_state_str(foc_state_t s)
{
	switch (s) {
	case FOC_STATE_IDLE:      return "IDLE";
	case FOC_STATE_ALIGNING:  return "ALIGNING";
	case FOC_STATE_RUNNING:   return "RUNNING";
	case FOC_STATE_ERROR:     return "ERROR";
	default:                  return "UNKNOWN";
	}
}
