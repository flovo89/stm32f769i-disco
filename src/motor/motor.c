#include "motor.h"

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(motor, LOG_LEVEL_INF);

/* ─── Device tree handles ────────────────────────────────────────────────── */

static const struct device *adc_dev = DEVICE_DT_GET(DT_NODELABEL(adc1));

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

static const struct gpio_dt_spec enc_a_gpio =
	GPIO_DT_SPEC_GET(DT_PATH(encoder, enc_a), gpios);
static const struct gpio_dt_spec enc_b_gpio =
	GPIO_DT_SPEC_GET(DT_PATH(encoder, enc_b), gpios);
static const struct gpio_dt_spec enc_z_gpio =
	GPIO_DT_SPEC_GET(DT_PATH(encoder, enc_z), gpios);

/* ─── ADC configuration ──────────────────────────────────────────────────── */

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

/* ─── Encoder state (modified in ISR, read in thread) ─────────────────────── */

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

/* ─── Current offset calibration ─────────────────────────────────────────── */

static int16_t cal_offset_ch6  = 2048;
static int16_t cal_offset_ch12 = 2048;

/* ─── PWM period (ns) ────────────────────────────────────────────────────── */

#define PWM_PERIOD_NS (1000000000UL / 20000UL)  /* 50 µs → 20 kHz */

/* ─── Init ──────────────────────────────────────────────────────────────── */

int motor_init(void)
{
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

	/* --- PWM --- */
	if (!device_is_ready(pwm_a_dev) || !device_is_ready(pwm_b_dev) ||
	    !device_is_ready(pwm_c_dev)) {
		LOG_ERR("PWM device(s) not ready");
		return -ENODEV;
	}

	/* Start all phases at 50% (zero voltage, safe starting point) */
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
	struct adc_sequence seq = adc_seq;  /* local copy so we can re-read */
	int ret = adc_read(adc_dev, &seq);

	if (ret) {
		LOG_ERR("ADC read error: %d", ret);
		return ret;
	}

	/* Convert raw → amps (subtract calibrated zero offset) */
	*ia = (adc_buf[0] - cal_offset_ch6)  * MOTOR_CURRENT_SCALE;
	*ib = (adc_buf[1] - cal_offset_ch12) * MOTOR_CURRENT_SCALE;

	return 0;
}

int motor_calibrate_currents(void)
{
	/* Average several samples with motor stopped */
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
}

/* ─── Encoder ───────────────────────────────────────────────────────────── */

void motor_reset_encoder(void)
{
	atomic_set(&enc_count, 0);
	enc_count_prev   = 0;
	enc_time_prev_us = k_uptime_get() * 1000LL;
}

float motor_read_encoder(float *theta_rad, float *omega_rad_s)
{
	static const float CPR4   = MOTOR_ENCODER_CPR * 4.0f;
	static const float TWO_PI = 6.28318530717959f;

	int32_t count = (int32_t)atomic_get(&enc_count);
	int64_t now_us = k_uptime_get() * 1000LL;

	/* Mechanical angle */
	int32_t pos_mod = count % (int32_t)(CPR4);

	if (pos_mod < 0) { pos_mod += (int32_t)CPR4; }

	*theta_rad = (float)pos_mod * TWO_PI / CPR4;

	/* Angular velocity from finite difference */
	int64_t dt_us = now_us - enc_time_prev_us;
	float   dt    = (float)dt_us * 1e-6f;
	float   omega = 0.0f;

	if (dt > 0.0001f) {  /* Avoid division by zero at startup */
		int32_t delta = count - enc_count_prev;

		omega = (float)delta * TWO_PI / CPR4 / dt;
	}

	enc_count_prev   = count;
	enc_time_prev_us = now_us;

	*omega_rad_s = omega;

	return omega * (60.0f / TWO_PI);  /* RPM */
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
