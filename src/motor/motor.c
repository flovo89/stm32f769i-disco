#include "motor.h"

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_MOTOR_SIM
#include "sim/sim.h"
static sim_ctx_t g_sim;
#else
#include <zephyr/drivers/adc.h>
#endif

LOG_MODULE_REGISTER(motor, LOG_LEVEL_INF);

/* ─── Device tree handles ────────────────────────────────────────────────── */

#ifndef CONFIG_MOTOR_SIM
static const struct device *adc_dev  = DEVICE_DT_GET(DT_NODELABEL(adc1));
static const struct device *qdec_dev = DEVICE_DT_GET(DT_NODELABEL(qdec0));
#endif

static const struct device *gpiof_dev = DEVICE_DT_GET(DT_NODELABEL(gpiof));

static const struct device *pwm_a_dev =
	DEVICE_DT_GET(DT_NODELABEL(pwm_phase_a));
static const struct device *pwm_b_dev =
	DEVICE_DT_GET(DT_NODELABEL(pwm_phase_b));
static const struct device *pwm_c_dev =
	DEVICE_DT_GET(DT_NODELABEL(pwm_phase_c));

#define PWM_CH_A  3   /* TIM3 CH3  — PC8 */
#define PWM_CH_B  1   /* TIM12 CH1 — PH6 */
#define PWM_CH_C  1   /* TIM11 CH1 — PF7 */

static const struct gpio_dt_spec motor_en_gpio =
	GPIO_DT_SPEC_GET(DT_PATH(motor_gpios, motor_en), gpios);

/* ─── ADC configuration (real hardware only) ─────────────────────────────── */

#ifndef CONFIG_MOTOR_SIM

static struct adc_channel_cfg adc_ch6_cfg = {
	.gain             = ADC_GAIN_1,
	.reference        = ADC_REF_INTERNAL,
	.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 15),
	.channel_id       = 6,
	.differential     = 0,
};

static struct adc_channel_cfg adc_ch12_cfg = {
	.gain             = ADC_GAIN_1,
	.reference        = ADC_REF_INTERNAL,
	.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_TICKS, 15),
	.channel_id       = 12,
	.differential     = 0,
};

static int16_t adc_raw_a;
static int16_t adc_raw_b;

/* Read channels one at a time — multi-channel sequences block forever on
 * STM32F7 without DMA because the EOS interrupt never fires.
 * Must be non-const (RAM) — the STM32 ADC driver writes into the struct. */
static struct adc_sequence adc_seq_a = {
	.channels    = BIT(6),
	.buffer      = &adc_raw_a,
	.buffer_size = sizeof(adc_raw_a),
	.resolution  = 12,
};

static struct adc_sequence adc_seq_b = {
	.channels    = BIT(12),
	.buffer      = &adc_raw_b,
	.buffer_size = sizeof(adc_raw_b),
	.resolution  = 12,
};

#endif /* CONFIG_MOTOR_SIM */

/* ─── Encoder state (real hardware only) ─────────────────────────────────── */

#ifndef CONFIG_MOTOR_SIM

#define CPR4    (MOTOR_ENCODER_CPR * 4)   /* 4096 counts/rev */
#define TWO_PI  6.28318530717959f

/* The Zephyr QDEC driver sets ARR = UINT16_MAX - (UINT16_MAX % CPR4) - 1 = 61439
 * for a 16-bit timer.  The counter rolls over at 61440, not CPR4. */
#define QDEC_PERIOD  (UINT16_MAX - (UINT16_MAX % CPR4))  /* 61440 */

/* Software position zero — set by motor_reset_encoder() */
static int32_t enc_offset;

/* Windowed velocity estimator — update omega every OMEGA_WINDOW ticks.
 * Per-tick (300 µs) instantaneous delta is 0 or 1 count at low speed;
 * a ~10 ms window accumulates enough counts for usable resolution. */
#define OMEGA_WINDOW 32

static uint32_t omega_tick;
static int32_t  omega_raw_base;
static int64_t  omega_time_base;
static float    omega_hold;

static int16_t cal_offset_ch6  = 2048;
static int16_t cal_offset_ch12 = 2048;

#endif /* CONFIG_MOTOR_SIM */

/* ─── PWM period (ns) ────────────────────────────────────────────────────── */

#define PWM_PERIOD_NS (1000000000UL / 20000UL)  /* 50 µs → 20 kHz */

/* ─── Init ──────────────────────────────────────────────────────────────── */

int motor_init(void)
{
#ifdef CONFIG_MOTOR_SIM
	LOG_WRN("SIMULATION MODE — ADC and encoder are synthetic");
	sim_init(&g_sim);
#else
	int ret;

	/* --- ADC --- */
	if (!device_is_ready(adc_dev)) {
		LOG_ERR("ADC not ready");
		return -ENODEV;
	}

	ret = adc_channel_setup(adc_dev, &adc_ch6_cfg);
	if (ret) { LOG_ERR("ADC ch6 setup: %d", ret); return ret; }

	ret = adc_channel_setup(adc_dev, &adc_ch12_cfg);
	if (ret) { LOG_ERR("ADC ch12 setup: %d", ret); return ret; }

	/* --- QDEC (TIM8 hardware encoder interface) --- */
	if (!device_is_ready(qdec_dev)) {
		LOG_ERR("QDEC not ready");
		return -ENODEV;
	}

	/* Float unused former encoder GPIO pins (PF6, PJ1, PJ0). */
	gpio_pin_configure(gpiof_dev, 6, GPIO_INPUT);          /* old PF6/D3        */
	gpio_pin_configure(motor_en_gpio.port, 1, GPIO_INPUT); /* old enc_b PJ1/D2  */
	gpio_pin_configure(motor_en_gpio.port, 0, GPIO_INPUT); /* old enc_z PJ0/D4  */
#endif /* CONFIG_MOTOR_SIM */

	/* --- PWM (always active — oscilloscope verification in sim mode) --- */
	if (!device_is_ready(pwm_a_dev) || !device_is_ready(pwm_b_dev) ||
	    !device_is_ready(pwm_c_dev)) {
		LOG_ERR("PWM device(s) not ready");
		return -ENODEV;
	}

	uint32_t half = PWM_PERIOD_NS / 2;

	pwm_set(pwm_a_dev, PWM_CH_A, PWM_PERIOD_NS, half, PWM_POLARITY_NORMAL);
	pwm_set(pwm_b_dev, PWM_CH_B, PWM_PERIOD_NS, half, PWM_POLARITY_NORMAL);
	pwm_set(pwm_c_dev, PWM_CH_C, PWM_PERIOD_NS, half, PWM_POLARITY_NORMAL);

	/* --- Motor enable GPIO --- */
	if (!gpio_is_ready_dt(&motor_en_gpio)) {
		LOG_ERR("Motor enable GPIO not ready");
		return -ENODEV;
	}
	gpio_pin_configure_dt(&motor_en_gpio, GPIO_OUTPUT_INACTIVE);

	LOG_INF("Motor hardware initialised");
	return 0;
}

/* ─── Enable / disable ──────────────────────────────────────────────────── */

void motor_enable(bool enable)
{
	gpio_pin_set_dt(&motor_en_gpio, enable ? 1 : 0);
}

bool motor_is_enabled(void)
{
	return gpio_pin_get_dt(&motor_en_gpio) > 0;
}

/* ─── Current sensing ───────────────────────────────────────────────────── */

int motor_read_currents(float *ia, float *ib)
{
#ifdef CONFIG_MOTOR_SIM
	sim_get_currents(&g_sim, ia, ib);
	return 0;
#else
	int ret = adc_read(adc_dev, &adc_seq_a);

	if (ret) { LOG_ERR("ADC ch6 read: %d", ret); return ret; }

	ret = adc_read(adc_dev, &adc_seq_b);
	if (ret) { LOG_ERR("ADC ch12 read: %d", ret); return ret; }

	*ia = (adc_raw_a - cal_offset_ch6)  * MOTOR_CURRENT_SCALE;
	*ib = -(adc_raw_b - cal_offset_ch12) * MOTOR_CURRENT_SCALE;

	return 0;
#endif
}

int motor_calibrate_currents(void)
{
#ifdef CONFIG_MOTOR_SIM
	LOG_INF("Simulation mode — calibration is a no-op");
	return 0;
#else
	int32_t sum_a = 0, sum_b = 0;
	const int N = 64;

	for (int i = 0; i < N; i++) {
		int ret = adc_read(adc_dev, &adc_seq_a);

		if (ret) { return ret; }

		ret = adc_read(adc_dev, &adc_seq_b);
		if (ret) { return ret; }
		sum_a += adc_raw_a;
		sum_b += adc_raw_b;
	}

	cal_offset_ch6  = (int16_t)(sum_a / N);
	cal_offset_ch12 = (int16_t)(sum_b / N);

	LOG_INF("Current offsets: ch6=%d ch12=%d",
	        cal_offset_ch6, cal_offset_ch12);
	return 0;
#endif
}

/* ─── Encoder ───────────────────────────────────────────────────────────── */

void motor_reset_encoder(void)
{
#ifdef CONFIG_MOTOR_SIM
	sim_reset(&g_sim);
#else
	struct sensor_value val;

	sensor_sample_fetch(qdec_dev);
	sensor_channel_get(qdec_dev, SENSOR_CHAN_ENCODER_COUNT, &val);
	enc_offset = (int32_t)val.val1;

	/* Reset velocity window to avoid a spike on the next read */
	omega_tick      = 0;
	omega_raw_base  = enc_offset;
	omega_time_base = k_uptime_get() * 1000LL;
	omega_hold      = 0.0f;
#endif
}

float motor_read_encoder(float *theta_rad, float *omega_rad_s)
{
#ifdef CONFIG_MOTOR_SIM
	sim_get_encoder(&g_sim, theta_rad, omega_rad_s);
	return sim_get_speed_rpm(&g_sim);
#else
	struct sensor_value val;

	sensor_sample_fetch(qdec_dev);
	sensor_channel_get(qdec_dev, SENSOR_CHAN_ENCODER_COUNT, &val);

	int32_t raw    = (int32_t)val.val1;
	int64_t now_us = k_uptime_get() * 1000LL;

	/* Position: signed movement from enc_offset, corrected for the 61440-period
	 * QDEC counter rollover, then wrapped into [0, CPR4). */
	int32_t pos_delta = raw - enc_offset;

	if (pos_delta >  (int32_t)(QDEC_PERIOD / 2)) { pos_delta -= QDEC_PERIOD; }
	if (pos_delta < -(int32_t)(QDEC_PERIOD / 2)) { pos_delta += QDEC_PERIOD; }

	int32_t pos = ((pos_delta % CPR4) + CPR4) % CPR4;

	*theta_rad = (float)pos * TWO_PI / (float)CPR4;

	/* Velocity: windowed estimator with QDEC-period-correct wraparound */
	if (++omega_tick >= OMEGA_WINDOW) {
		omega_tick = 0;

		int32_t delta = raw - omega_raw_base;

		if (delta >  (int32_t)(QDEC_PERIOD / 2)) { delta -= QDEC_PERIOD; }
		if (delta < -(int32_t)(QDEC_PERIOD / 2)) { delta += QDEC_PERIOD; }

		int64_t dt_us = now_us - omega_time_base;

		if (dt_us > 1000) {
			omega_hold = (float)delta * TWO_PI / (float)CPR4
			             / ((float)dt_us * 1e-6f);
		}

		omega_raw_base  = raw;
		omega_time_base = now_us;
	}

	*omega_rad_s = omega_hold;

	return omega_hold * (60.0f / TWO_PI);
#endif
}

/* ─── PWM output ────────────────────────────────────────────────────────── */

void motor_set_pwm(float da, float db, float dc)
{
	uint32_t pa = (uint32_t)(da * PWM_PERIOD_NS);
	uint32_t pb = (uint32_t)(db * PWM_PERIOD_NS);
	uint32_t pc = (uint32_t)(dc * PWM_PERIOD_NS);

	pwm_set(pwm_a_dev, PWM_CH_A, PWM_PERIOD_NS, pa, PWM_POLARITY_NORMAL);
	pwm_set(pwm_b_dev, PWM_CH_B, PWM_PERIOD_NS, pb, PWM_POLARITY_NORMAL);
	pwm_set(pwm_c_dev, PWM_CH_C, PWM_PERIOD_NS, pc, PWM_POLARITY_NORMAL);
}

/* ─── Simulation update (CONFIG_MOTOR_SIM only) ──────────────────────────── */

#ifdef CONFIG_MOTOR_SIM
void motor_sim_update(float vd, float vq, float dt)
{
	sim_set_voltages(&g_sim, vd, vq);
	sim_step(&g_sim, dt);
}

void motor_sim_set_load(float T_load_nm)
{
	sim_set_load(&g_sim, T_load_nm);
}

const sim_ctx_t *motor_sim_get_ctx(void)
{
	return &g_sim;
}

void motor_sim_set_params(float R, float L_H, float psi_Wb,
                          float J_kgm2, float B_Nms)
{
	sim_set_params(&g_sim, R, L_H, psi_Wb, J_kgm2, B_Nms);
}
#endif /* CONFIG_MOTOR_SIM */
