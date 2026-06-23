#pragma once

#include <stdbool.h>

/*
 * Surface-mounted PMSM (SPMSM) simulation model, d-q synchronous frame.
 *
 * Electrical:
 *   L·dId/dt = Vd - R·Id + ωe·L·Iq
 *   L·dIq/dt = Vq - R·Iq - ωe·(L·Id + ψm)
 *
 * Mechanical:
 *   J·dω/dt = Tem - B·ω - Tload
 *   Tem = 1.5 · p · ψm · Iq
 *
 * Default parameters (typical small BLDC):
 *   R = 0.5 Ω,  L = 1 mH,  ψm = 0.05 Wb
 *   J = 1e-5 kg·m²,  B = 1e-4 N·m·s/rad
 */

typedef struct {
    float R;          /* Stator resistance [Ω] */
    float L;          /* Stator inductance [H] (Ld = Lq for SPMSM) */
    float psi_m;      /* PM flux linkage [Wb] */
    float J;          /* Moment of inertia [kg·m²] */
    float B_fric;     /* Viscous friction [N·m·s/rad] */

    float T_load;     /* External load torque [N·m] */

    /* State (integrated each control tick) */
    float id;
    float iq;
    float omega_mech; /* Mechanical angular velocity [rad/s] */
    float theta_mech; /* Mechanical angle [rad, 0..2π) */

    /* Last applied voltages (set by motor_sim_update) */
    float vd;
    float vq;
} sim_ctx_t;

void  sim_init(sim_ctx_t *s);
void  sim_reset(sim_ctx_t *s);
void  sim_step(sim_ctx_t *s, float dt);

/* Set applied voltages from FOC output before calling sim_step */
void  sim_set_voltages(sim_ctx_t *s, float vd, float vq);

/* Adjust load torque at runtime (shell command: sim_load <Nm>) */
void  sim_set_load(sim_ctx_t *s, float T_load_nm);

/* Override motor model parameters (shell command: sim_params ...) */
void  sim_set_params(sim_ctx_t *s,
                     float R, float L_H, float psi_Wb,
                     float J_kgm2, float B_Nms);

/* Reconstruct phase A/B currents via inverse Park + inverse Clarke */
void  sim_get_currents(const sim_ctx_t *s, float *ia, float *ib);

/* Encoder interface */
void  sim_get_encoder(const sim_ctx_t *s,
                      float *theta_mech, float *omega_mech);

float sim_get_speed_rpm(const sim_ctx_t *s);
