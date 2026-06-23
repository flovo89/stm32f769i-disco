#include "sim.h"

#include <math.h>
#include <string.h>

#include "foc/foc.h"   /* FOC_POLE_PAIRS */

#define TWO_PI  6.28318530717959f
#define SQRT3   1.73205080756888f

/*
 * Default SPMSM parameters chosen for stability at 10 kHz control rate:
 *
 *  R   = 0.5 Ω       τ_e  = L/R = 2 ms (dt/τ_e = 0.05 — Euler stable)
 *  L   = 1 mH
 *  psi = 0.02 Wb     max linear speed = Vbus/(√3·p·ψ) = 24/(1.732·7·0.02) ≈ 990 RPM
 *                     (psi=0.05 capped at 378 RPM, too low for set_speed 500)
 *  J   = 10 g·m²     α_max = T_em = 1.5·p·ψ·iq_max/J = 1.5·7·0.02·8/0.01 = 168 rad/s²
 *                     → ~300 ms to reach 500 RPM (~3000 control ticks)
 *  B   = 5 mN·m·s    τ_mech = J/B = 2 s
 *
 * Override at runtime with the `sim_params` shell/UDP command.
 */
#define SIM_R_DEFAULT     0.5f
#define SIM_L_DEFAULT     1e-3f
#define SIM_PSI_M_DEFAULT 0.02f
#define SIM_J_DEFAULT     1e-2f
#define SIM_B_DEFAULT     5e-3f

void sim_init(sim_ctx_t *s)
{
    memset(s, 0, sizeof(*s));
    s->R      = SIM_R_DEFAULT;
    s->L      = SIM_L_DEFAULT;
    s->psi_m  = SIM_PSI_M_DEFAULT;
    s->J      = SIM_J_DEFAULT;
    s->B_fric = SIM_B_DEFAULT;
}

void sim_reset(sim_ctx_t *s)
{
    s->id = s->iq = 0.0f;
    s->omega_mech = s->theta_mech = 0.0f;
    s->vd = s->vq = 0.0f;
}

void sim_set_voltages(sim_ctx_t *s, float vd, float vq)
{
    s->vd = vd;
    s->vq = vq;
}

void sim_set_load(sim_ctx_t *s, float T_load_nm)
{
    s->T_load = T_load_nm;
}

void sim_set_params(sim_ctx_t *s,
                    float R, float L_H, float psi_Wb,
                    float J_kgm2, float B_Nms)
{
    s->R      = R;
    s->L      = L_H;
    s->psi_m  = psi_Wb;
    s->J      = J_kgm2;
    s->B_fric = B_Nms;
}

void sim_step(sim_ctx_t *s, float dt)
{
    float pp      = (float)FOC_POLE_PAIRS;
    float omega_e = s->omega_mech * pp;

    /* Electrical d-q model — forward Euler */
    float did_dt = (s->vd - s->R * s->id + omega_e * s->L * s->iq) / s->L;
    float diq_dt = (s->vq - s->R * s->iq
                   - omega_e * (s->L * s->id + s->psi_m)) / s->L;

    s->id += did_dt * dt;
    s->iq += diq_dt * dt;

    /* Electromagnetic torque (SPMSM: Ld = Lq, so no reluctance term) */
    float T_em      = 1.5f * pp * s->psi_m * s->iq;
    float domega_dt = (T_em - s->B_fric * s->omega_mech - s->T_load) / s->J;

    s->omega_mech += domega_dt * dt;
    s->theta_mech += s->omega_mech * dt;

    s->theta_mech = fmodf(s->theta_mech, TWO_PI);
    if (s->theta_mech < 0.0f) {
        s->theta_mech += TWO_PI;
    }
}

void sim_get_currents(const sim_ctx_t *s, float *ia, float *ib)
{
    float theta_e = s->theta_mech * (float)FOC_POLE_PAIRS;
    float c  = cosf(theta_e);
    float sv = sinf(theta_e);

    /* Inverse Park: (id, iq) → (iα, iβ) */
    float ialpha = s->id * c - s->iq * sv;
    float ibeta  = s->id * sv + s->iq * c;

    /* Inverse Clarke matching foc.c forward Clarke (iβ = (ia + 2·ib)/√3):
     *   ia = iα
     *   ib = (√3·iβ - iα) / 2                                          */
    *ia = ialpha;
    *ib = (SQRT3 * ibeta - ialpha) * 0.5f;
}

void sim_get_encoder(const sim_ctx_t *s,
                     float *theta_mech, float *omega_mech)
{
    *theta_mech = s->theta_mech;
    *omega_mech = s->omega_mech;
}

float sim_get_speed_rpm(const sim_ctx_t *s)
{
    return s->omega_mech * (60.0f / TWO_PI);
}
