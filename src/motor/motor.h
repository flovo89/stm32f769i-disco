#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Current sensor calibration.
 * Adjust shunt_ohm and amp_gain to match your motor driver shield.
 * Common values for DRV830x shields: 10 mΩ shunt, ×20 amp.
 *
 * Scale: 1 LSB = (Vref / 4096) / (shunt * gain)  [A/LSB]
 * Offset: 2048 LSB = zero current (mid-rail)
 */
#define MOTOR_SHUNT_OHM    0.01f   /* 10 mΩ */
#define MOTOR_AMP_GAIN     20.0f   /* ×20   */
#define MOTOR_ADC_VREF_V   3.3f
#define MOTOR_ADC_BITS     12
#define MOTOR_ADC_COUNTS   (1 << MOTOR_ADC_BITS)

#define MOTOR_CURRENT_SCALE \
	(MOTOR_ADC_VREF_V / (MOTOR_ADC_COUNTS * MOTOR_SHUNT_OHM * MOTOR_AMP_GAIN))

/* Encoder */
#define MOTOR_ENCODER_CPR  1024   /* Lines per revolution (×4 for quadrature) */

/* DC bus voltage (no bus-voltage sense pin in this hardware setup) */
#define MOTOR_VBUS_V       24.0f

int  motor_init(void);

void motor_enable(bool enable);
bool motor_is_enabled(void);

/*
 * Read phase A and B currents [A].
 * Returns 0 on success, negative errno on error.
 */
int motor_read_currents(float *ia, float *ib);

/*
 * Read mechanical angle [rad, 0..2π) and angular velocity [rad/s].
 * Returns speed in RPM.
 */
float motor_read_encoder(float *theta_rad, float *omega_rad_s);

/*
 * Apply PWM duty cycles [0..1] to the three inverter legs.
 * 0.5 → zero voltage, 0 → negative rail, 1 → positive rail.
 */
void motor_set_pwm(float da, float db, float dc);

/* Zero the current sensor offsets at standstill (motor must be disabled). */
int motor_calibrate_currents(void);

/* Encoder count — exposed so alignment can reset it. */
void motor_reset_encoder(void);

#ifdef CONFIG_MOTOR_SIM
#include "sim/sim.h"

/*
 * Step the simulation model with the voltages the FOC just commanded.
 * Call once per control tick, after motor_set_pwm().
 *   vd, vq : d/q voltages from foc_ctx_t [V]
 *   dt     : control period [s]  (typically FOC_CONTROL_DT)
 */
void motor_sim_update(float vd, float vq, float dt);

/* Adjust simulated load torque at runtime. */
void motor_sim_set_load(float T_load_nm);

/* Read-only access to the internal sim state (for status reporting). */
const sim_ctx_t *motor_sim_get_ctx(void);

/* Override motor model parameters at runtime. */
void motor_sim_set_params(float R, float L_H, float psi_Wb,
                          float J_kgm2, float B_Nms);
#endif /* CONFIG_MOTOR_SIM */
