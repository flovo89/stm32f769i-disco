#pragma once

#include "pid.h"
#include <stdbool.h>
#include <stdint.h>

/* ─── Tunable parameters ────────────────────────────────────────────────── */
#define FOC_CONTROL_HZ       5000       /* Control loop rate — limited by the two-point
                                          * ADC sampling (CCR4 busy-wait ~50 µs + 25 µs
                                          * ripple-phase wait), leaving ~100 µs/step free
                                          * for the shell thread at 200 µs period       */
#define FOC_CONTROL_DT       (1.0f / FOC_CONTROL_HZ)
#define FOC_PWM_HZ           20000      /* PWM switching frequency */

#define FOC_POLE_PAIRS       7          /* Motor pole pair count */
#define FOC_MAX_CURRENT_A    8.0f      /* Overcurrent trip threshold [A] */
#define FOC_MAX_TORQUE_A     7.5f       /* Speed PI output clamp [A] — must be
                                         * < FOC_MAX_CURRENT_A so phase current
                                         * transients don't trip overcurrent     */
#define FOC_MAX_SPEED_RPM    5000.0f

/* Motor electrical parameters for decoupling feedforward.
 * Override via set_motor_params at runtime if needed. */
#define FOC_MOTOR_L_H        1.6e-5f   /* Stator inductance [H]  — measured ~16 µH */
#define FOC_MOTOR_PSI_WB     1.58e-3f  /* PM flux linkage [Wb]   — identified from
                                         * vq saturation at 3500–3600 RPM under 3 A:
                                         * ψ ≈ (vq - R·iq) / ω_e ≈ 0.00156–0.00170 Wb */

/* Alignment: two-phase open-loop voltage (no current PI, no OC check).
 * Phase 1 (first 25%): hold field at π/2 — rotor settles near π/2 equilibrium.
 * Phase 2 (last 75%): linearly ramp field from π/2 → 0° — rotor is dragged to 0°.
 * Direct voltage bypasses the current loop for reliable operation regardless of
 * ADC-range limitations; OC check is re-enabled on entry to RUNNING.
 * FOC_ALIGN_MS is the config value; actual duration ≈ 1.5 s at the ~4 kHz
 * real control rate (10 kHz assumed but timer runs slower in practice). */
#define FOC_ALIGN_VOLTAGE_V  0.5f   /* Limits alignment id to ~2.75 A (=0.5/R); 2.0 V
                                      * was saturating the ADC at ~6.5 A during alignment */
#define FOC_ALIGN_MS         1500

/* Default current PI gains — tuned for L=16µH, R=0.182Ω, ω_bw=500 rad/s
 * Kp = ω_bw × L = 0.008,  Ki = ω_bw × R = 91 */
#define FOC_KP_CURRENT       0.008f
#define FOC_KI_CURRENT       91.0f

/* Default speed PI gains — J ≈ 1e-4 kg·m², Kt ≈ 0.0166 N·m/A (with ψ=1.58e-3):
 * K_plant = Kt*60/(J*2π) = 1584 RPM/s/A → ωn = sqrt(K_plant*Ki) = 12.6 rad/s
 * ζ = K_plant*Kp / (2*ωn); ζ=0.31 at Kp=0.005.
 * Motor has cogging torque that causes instability near 0 RPM.  Alignment
 * residual velocity (300–1800 RPM) gets the motor past this zone before the
 * PI takes over, so cogging is not normally a problem in practice.  */
#define FOC_KP_SPEED         0.005f  /* ζ≈0.31; higher Kp risks cogging-zone oscillation
                                      * on RUNNING entry — alignment residual gets motor
                                      * past cogging before PI takes over, but residual
                                      * speed is variable (300–1800 RPM) so keep Kp low */
#define FOC_KI_SPEED         0.10f

/* ─── State machine ─────────────────────────────────────────────────────── */
typedef enum {
	FOC_STATE_IDLE,
	FOC_STATE_ALIGNING,
	FOC_STATE_RUNNING,
	FOC_STATE_FORCED,   /* hold da/db/dc fixed — used by diagnostics */
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

	/* Motor electrical params for decoupling feedforward */
	float L_motor;   /* Stator inductance [H]  */
	float psi_motor; /* PM flux linkage [Wb]   */

	/* Alignment */
	int   align_ticks_left;

	/* Output voltage clamp — limits max phase voltage and hence peak current.
	 * Applied to both PI output and post-feedforward combined voltage.
	 * Default: Vbus/√3 (full linear modulation range).
	 * Reduce to limit peak current when current sensing range is small. */
	float vlim;

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
void foc_set_vlim(foc_ctx_t *foc, float vlim_v);
void foc_set_forced_duty(foc_ctx_t *foc, float da, float db, float dc);

/* Math helpers (exposed for testing) */
void foc_clarke(float ia, float ib, float *alpha, float *beta);
void foc_park(float alpha, float beta, float theta,
              float *d, float *q);
void foc_inv_park(float vd, float vq, float theta,
                  float *alpha, float *beta);
void foc_svpwm(float valpha, float vbeta, float vbus,
               float *da, float *db, float *dc);

const char *foc_state_str(foc_state_t s);
