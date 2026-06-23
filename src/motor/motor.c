#include "motor.h"

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
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
static const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc1));
#endif

/* PWM_DT_SPEC_GET is consumer-side API (needs a 'pwms' property).
 * We access the PWM devices directly by label and supply channel numbers. */
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

#ifndef CONFIG_MOTOR_SIM
static const struct gpio_dt_spec enc_a_gpio =
	GPIO_DT_SPEC_GET(DT_PATH(encoder, enc_a), gpios);
static const struct gpio_dt_spec enc_b_gpio =
	GPIO_DT_SPEC_GET(DT_PATH(encoder, enc_b), gpios);
static const struct gpio_dt_spec enc_z_gpio =
	GPIO_DT_SPEC_GET(DT_PATH(encoder, enc_z), gpios);
#endif

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

static int16_t adc_buf[2];

static const struct adc_sequence adc_seq = {
	.channels    = BIT(6) | BIT(12),
	.buffer      = adc_buf,
	.buffer_size = sizeof(adc_buf),
	.resolution  = 12,
};

#endif /* CONFIG_MOTOR_SIM */

/* ─── Encoder state and current offsets (real hardware only) ─────────────── */

#ifndef CONFIG_MOTOR_SIM

static atomic_t enc_count = ATOMIC_INIT(0);
static volatile int32_t enc_count_prev;
static volatile int64_t enc_time_prev_us;

static struct gpio_callback enc_a_cb;
static struct gpio_callback enc_b_cb;
static struct gpio_callback enc_z_cb;

static void enc_a_isr(const struct device *dev,
                       struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	int a = gpio_pin_get_dt(&enc_a_gpio);
	int b = gpio_pin_get_dt(&enc_b_gpio);

	/* Standard 4× quadrature decode table */
	if (a)  { atomic_add(&enc_count, b ? -1 : 1); }
	else    { atomic_add(&enc_count, b ?  1 : -1); }
}

static void enc_b_isr(const struct device *dev,
                       struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	int a = gpio_pin_get_dt(&enc_a_gpio);
	int b = gpio_pin_get_dt(&enc_b_gpio);

	if (b)  { atomic_add(&enc_count, a ? 1 : -1); }
	else    { atomic_add(&enc_count, a ? -1 : 1); }
}

static void enc_z_isr(const struct device *dev,
                       struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	/* Index pulse: snap count to nearest multiple of CPR×4 */
	int32_t cpr4 = MOTOR_ENCODER_CPR * 4;
	int32_t c    = (int32_t)atomic_get(&enc_count);
	int32_t rem  = c % cpr4;

	atomic_set(&enc_count, c - rem);
}

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

	/* --- Motor enable GPIO (always active) --- */
	if (!gpio_is_ready_dt(&motor_en_gpio)) {
		LOG_ERR("Motor enable GPIO not ready");
		return -ENODEV;
	}
	gpio_pin_configure_dt(&motor_en_gpio, GPIO_OUTPUT_INACTIVE);

#ifndef CONFIG_MOTOR_SIM
	/* --- Encoder GPIOs --- */
	if (!gpio_is_ready_dt(&enc_a_gpio) ||
	    !gpio_is_ready_dt(&enc_b_gpio) ||
	    !gpio_is_ready_dt(&enc_z_gpio)) {
		LOG_ERR("Encoder GPIOs not ready");
		return -ENODEV;
	}

	gpio_pin_configure_dt(&enc_a_gpio, GPIO_INPUT);
	gpio_pin_configure_dt(&enc_b_gpio, GPIO_INPUT);
	gpio_pin_configure_dt(&enc_z_gpio, GPIO_INPUT);

	gpio_init_callback(&enc_a_cb, enc_a_isr, BIT(enc_a_gpio.pin));
	gpio_init_callback(&enc_b_cb, enc_b_isr, BIT(enc_b_gpio.pin));
	gpio_init_callback(&enc_z_cb, enc_z_isr, BIT(enc_z_gpio.pin));

	gpio_add_callback(enc_a_gpio.port, &enc_a_cb);
	gpio_add_callback(enc_b_gpio.port, &enc_b_cb);
	gpio_add_callback(enc_z_gpio.port, &enc_z_cb);

	gpio_pin_interrupt_configure_dt(&enc_a_gpio, GPIO_INT_EDGE_BOTH);
	gpio_pin_interrupt_configure_dt(&enc_b_gpio, GPIO_INT_EDGE_BOTH);
	gpio_pin_interrupt_configure_dt(&enc_z_gpio, GPIO_INT_EDGE_RISING);

	enc_time_prev_us = k_uptime_get() * 1000LL;
#endif /* CONFIG_MOTOR_SIM */

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
	struct adc_sequence seq = adc_seq;  /* local copy so we can re-read */
	int ret = adc_read(adc_dev, &seq);

	if (ret) {
		LOG_ERR("ADC read error: %d", ret);
		return ret;
	}

	*ia = (adc_buf[0] - cal_offset_ch6)  * MOTOR_CURRENT_SCALE;
	*ib = (adc_buf[1] - cal_offset_ch12) * MOTOR_CURRENT_SCALE;

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
	struct adc_sequence seq = adc_seq;

	for (int i = 0; i < N; i++) {
		int ret = adc_read(adc_dev, &seq);

		if (ret) { return ret; }
		sum_a += adc_buf[0];
		sum_b += adc_buf[1];
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
	atomic_set(&enc_count, 0);
	enc_count_prev   = 0;
	enc_time_prev_us = k_uptime_get() * 1000LL;
#endif
}

float motor_read_encoder(float *theta_rad, float *omega_rad_s)
{
#ifdef CONFIG_MOTOR_SIM
	sim_get_encoder(&g_sim, theta_rad, omega_rad_s);
	return sim_get_speed_rpm(&g_sim);
#else
	static const float CPR4   = MOTOR_ENCODER_CPR * 4.0f;
	static const float TWO_PI = 6.28318530717959f;

	int32_t count = (int32_t)atomic_get(&enc_count);
	int64_t now_us = k_uptime_get() * 1000LL;

	int32_t pos_mod = count % (int32_t)(CPR4);

	if (pos_mod < 0) { pos_mod += (int32_t)CPR4; }

	*theta_rad = (float)pos_mod * TWO_PI / CPR4;

	int64_t dt_us = now_us - enc_time_prev_us;
	float   dt    = (float)dt_us * 1e-6f;
	float   omega = 0.0f;

	if (dt > 0.0001f) {
		int32_t delta = count - enc_count_prev;

		omega = (float)delta * TWO_PI / CPR4 / dt;
	}

	enc_count_prev   = count;
	enc_time_prev_us = now_us;

	*omega_rad_s = omega;

	return omega * (60.0f / TWO_PI);
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
