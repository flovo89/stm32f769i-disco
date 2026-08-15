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
#include <zephyr/cache.h>  /* sys_cache_data_invd_range — DMA/D-cache coherency */
#include <soc.h>  /* STM32F769 CMSIS peripheral definitions */
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

/* DMA destination: [0] = CH6 (phase A), [1] = CH12 (phase B).
 * Written by DMA2 Stream0 at 20kHz, triggered by TIM3 OC4 at 75% of the PWM
 * period — safely inside the low-side-ON window for duty cycles up to 0.75. */
/* Align to Cortex-M7 cache line (32 bytes) so SCB_InvalidateDCache_by_Addr
 * operates on a single line and does not disturb adjacent variables. */
static volatile uint16_t __aligned(32) adc_dma_buf[2];

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

/* ─── Hardware ADC trigger setup (real hardware only) ───────────────────── */

#ifndef CONFIG_MOTOR_SIM

/*
 * Configure TIM3 CH4 to generate OC4REF at 75% of the PWM period, then route
 * that signal to TIM3_TRGO so it can trigger ADC conversions.
 *
 * TIM3 is already running (started by Zephyr's PWM driver).  CH4 is unused
 * for PWM output — we use it only as an internal compare event.
 *
 * With OC4M = PWM mode 2 and CCR4 = 75% × ARR:
 *   OC4REF = 0 while CNT < CCR4  (first 75% of the period)
 *   OC4REF = 1 while CNT ≥ CCR4  (rising edge at the match → TRGO fires)
 *
 * At 20kHz the trigger fires at t = 37.5 µs after the period start, which
 * is in the centre of the A-low-ON and B-low-ON windows for duty cycles ≤ 0.75.
 */
static void motor_setup_tim3_trgo(void)
{
	uint32_t arr = TIM3->ARR;  /* read ARR set by the Zephyr PWM driver */

	TIM3->CCR4 = (arr * 3U) / 4U;

	/* CCMR2: OC4M = PWM mode 2 (OC4M[2:0] = 0b111), preload enable */
	TIM3->CCMR2 = (TIM3->CCMR2 & ~TIM_CCMR2_OC4M)
	            | TIM_CCMR2_OC4M_0 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2
	            | TIM_CCMR2_OC4PE;

	/* Force an update event to load CCR4 from its preload register immediately.
	 * Motor is not yet enabled so this counter reset is harmless. */
	TIM3->EGR = TIM_EGR_UG;
}

/*
 * Configure DMA2 Stream0 Channel0 to transfer ADC1→adc_dma_buf in circular
 * mode, then reconfigure ADC1 for hardware-triggered scan mode.
 *
 * ADC sampling times were set by adc_channel_setup() and are preserved;
 * only the scan sequence, trigger source, and DMA routing are changed here.
 */
static void motor_setup_adc_dma(void)
{
	/* Enable DMA2 clock (safe to set even if already on). */
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
	__DSB();

	/* Disable Stream0 and wait for it to stop. */
	DMA2_Stream0->CR &= ~DMA_SxCR_EN;
	while (DMA2_Stream0->CR & DMA_SxCR_EN) {}

	/* Clear all Stream0 interrupt flags. */
	DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0
	            | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0;

	/* Stream0: ADC1 DR → adc_dma_buf, 2 half-words, circular. */
	DMA2_Stream0->PAR  = (uint32_t)&ADC1->DR;
	DMA2_Stream0->M0AR = (uint32_t)adc_dma_buf;
	DMA2_Stream0->NDTR = 2U;
	DMA2_Stream0->FCR  = 0U;  /* direct mode, no FIFO */

	DMA2_Stream0->CR =
		(0U << DMA_SxCR_CHSEL_Pos) |  /* channel 0 = ADC1 on DMA2 */
		DMA_SxCR_MSIZE_0            |  /* memory size: 16-bit */
		DMA_SxCR_PSIZE_0            |  /* peripheral size: 16-bit */
		DMA_SxCR_MINC               |  /* memory pointer increments */
		DMA_SxCR_CIRC;                 /* circular mode — wraps after 2 items */
	/* DIR[1:0] = 00 (peripheral-to-memory) — reset value, no explicit set needed */

	DMA2_Stream0->CR |= DMA_SxCR_EN;

	/* Reconfigure ADC1 for scan mode + TIM3_CH4 external trigger + DMA.
	 * Stop ADC first; some CR1/CR2 bits must not be written while converting. */
	ADC1->CR2 &= ~ADC_CR2_ADON;

	/* Clear any residual OVR flag — with OVR set the ADC blocks all DMA
	 * requests (RM0410 §15.8.1), so this must happen before enabling DMA. */
	ADC1->SR = 0;

	/* Scan sequence: SQ1 = CH6 (phase A), SQ2 = CH12 (phase B), L = 1 (2 conv.) */
	ADC1->SQR1 = (1U << ADC_SQR1_L_Pos);
	ADC1->SQR3 = (6U  << ADC_SQR3_SQ1_Pos)
	           | (12U << ADC_SQR3_SQ2_Pos);

	/* CR1: enable scan mode, disable all ADC interrupts (DMA handles data). */
	ADC1->CR1 = (ADC1->CR1 | ADC_CR1_SCAN) & ~(ADC_CR1_EOCIE | ADC_CR1_OVRIE);

	/* CR2: software trigger (EXTEN=00 enables SWSTART), DMA circular.
	 * Hardware trigger via TIM3_CH4 was attempted but proved unreliable;
	 * motor_read_currents() fires SWSTART on every FOC tick instead (~2 µs). */
	ADC1->CR2 = (ADC1->CR2
	           & ~(ADC_CR2_EXTSEL | ADC_CR2_EXTEN | ADC_CR2_CONT))
	          | ADC_CR2_DMA   /* enable DMA mode         */
	          | ADC_CR2_DDS;  /* keep DMA requests going */

	/* Re-enable ADC and wait for the mandatory stabilisation delay (~3 µs). */
	ADC1->CR2 |= ADC_CR2_ADON;
	k_busy_wait(10);

	/* Verify ADC+DMA path with one software-triggered scan. */
	DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0;
	ADC1->CR2 |= ADC_CR2_SWSTART;
	k_busy_wait(10);
	sys_cache_data_invd_range((void *)adc_dma_buf, sizeof(adc_dma_buf));
	LOG_INF("ADC init scan: buf[0]=%u buf[1]=%u NDTR=%u (expect ~2048, NDTR=2)",
	        adc_dma_buf[0], adc_dma_buf[1], (unsigned)DMA2_Stream0->NDTR);
}

#endif /* CONFIG_MOTOR_SIM */

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

#ifndef CONFIG_MOTOR_SIM
	/* Configure TIM3 OC4 → TRGO at 75% of period, then start ADC+DMA. */
	motor_setup_tim3_trgo();
	motor_setup_adc_dma();
#endif

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
	/* Phase-lock the sample to the 75% point of the TIM3 period.
	 * At CCR4 (= ARR × 3/4 = 4049): all low-side FETs are ON, and the
	 * current-ripple triangle crosses zero (I_instant = I_average).
	 * With L=16µH, Vbus=10V, fsw=20kHz the ripple is ~15A peak-to-peak;
	 * sampling off-phase aliases this ripple directly into the control loop.
	 * Max busy-wait = one TIM3 period (50 µs) < 100 µs FOC budget. */
	uint32_t target = TIM3->CCR4;

	if (TIM3->CNT >= target) {
		while (TIM3->CNT >= target);   /* wait for counter overflow */
	}
	while (TIM3->CNT < target);         /* wait until CNT reaches 75% */

	DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0;
	ADC1->CR2 |= ADC_CR2_SWSTART;

	/* Spin until DMA completes both channels (~1.5 µs at 27 MHz ADC clock). */
	uint32_t spin = 10000U;
	while (!(DMA2->LISR & DMA_LISR_TCIF0) && spin) {
		spin--;
	}

	sys_cache_data_invd_range((void *)adc_dma_buf, sizeof(adc_dma_buf));

	int32_t raw_a = (int32_t)adc_dma_buf[0] - (int32_t)cal_offset_ch6;
	int32_t raw_b = (int32_t)adc_dma_buf[1] - (int32_t)cal_offset_ch12;
	*ia =  (float)raw_a * MOTOR_CURRENT_SCALE;
	*ib = -(float)raw_b * MOTOR_CURRENT_SCALE;
	return spin ? 0 : -EIO;
#endif
}

int motor_calibrate_currents(void)
{
#ifdef CONFIG_MOTOR_SIM
	LOG_INF("Simulation mode — calibration is a no-op");
	return 0;
#else
	/* Read 64 software-triggered samples.
	 * Motor must be disabled and duty at 50% so shunt current is zero. */
	int64_t sum_a = 0, sum_b = 0;
	const int N = 64;

	for (int i = 0; i < N; i++) {
		k_usleep(100);
		/* Sample at 75% of TIM3 period (ripple zero, low-sides on). */
		uint32_t target = TIM3->CCR4;

		if (TIM3->CNT >= target) {
			while (TIM3->CNT >= target);
		}
		while (TIM3->CNT < target);

		DMA2->LIFCR = DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0;
		ADC1->CR2 |= ADC_CR2_SWSTART;

		uint32_t spin = 10000U;
		while (!(DMA2->LISR & DMA_LISR_TCIF0) && spin) {
			spin--;
		}
		sys_cache_data_invd_range((void *)adc_dma_buf, sizeof(adc_dma_buf));
		sum_a += adc_dma_buf[0];
		sum_b += adc_dma_buf[1];
	}

	cal_offset_ch6  = (int16_t)(sum_a / N);
	cal_offset_ch12 = (int16_t)(sum_b / N);

	/* Offsets should be near 2048 (mid-rail).  Far-from-2048 values mean
	 * the DMA buffer was not being updated — check ADC/DMA hardware setup. */
	LOG_INF("Current offsets: ch6=%d ch12=%d (expect ~2048)",
	        cal_offset_ch6, cal_offset_ch12);

	if (cal_offset_ch6 < 1024 || cal_offset_ch6 > 3072 ||
	    cal_offset_ch12 < 1024 || cal_offset_ch12 > 3072) {
		LOG_ERR("Calibration offsets out of range — ADC/DMA may not be running");
		sys_cache_data_invd_range((void *)adc_dma_buf, sizeof(adc_dma_buf));
		LOG_ERR("  ADC1 SR=0x%02x CR2=0x%08x DMA2 LISR=0x%08x",
		        (unsigned)ADC1->SR, (unsigned)ADC1->CR2, (unsigned)DMA2->LISR);
		LOG_ERR("  DMA2 S0: EN=%u NDTR=%u M0AR=0x%08x",
		        !!(DMA2_Stream0->CR & DMA_SxCR_EN),
		        (unsigned)DMA2_Stream0->NDTR,
		        (unsigned)DMA2_Stream0->M0AR);
		LOG_ERR("  raw buf[0]=%u buf[1]=%u", adc_dma_buf[0], adc_dma_buf[1]);
		return -EIO;
	}
	return 0;
#endif
}

void motor_adc_dump(char *buf, size_t len)
{
#ifdef CONFIG_MOTOR_SIM
	snprintf(buf, len, "SIM mode — no hardware ADC");
#else
	sys_cache_data_invd_range((void *)adc_dma_buf, sizeof(adc_dma_buf));
	uint16_t raw0 = adc_dma_buf[0];
	uint16_t raw1 = adc_dma_buf[1];
	snprintf(buf, len,
	         "raw[0]=%u raw[1]=%u off6=%d off12=%d ia_raw=%d ib_raw=%d "
	         "DMA EN=%u NDTR=%u LISR=0x%08x "
	         "ADC SR=0x%02x CR2=0x%08x "
	         "TIM3 CR1=0x%04x CCMR2=0x%08x CCR4=%u ARR=%u",
	         raw0, raw1,
	         (int)cal_offset_ch6, (int)cal_offset_ch12,
	         (int)raw0 - (int)cal_offset_ch6,
	         -((int)raw1 - (int)cal_offset_ch12),
	         !!(DMA2_Stream0->CR & DMA_SxCR_EN),
	         (unsigned)DMA2_Stream0->NDTR,
	         (unsigned)DMA2->LISR,
	         (unsigned)ADC1->SR,
	         (unsigned)ADC1->CR2,
	         (unsigned)TIM3->CR1,
	         (unsigned)TIM3->CCMR2,
	         (unsigned)TIM3->CCR4,
	         (unsigned)TIM3->ARR);
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
	 * QDEC counter rollover, then wrapped into [0, CPR4).
	 * MOTOR_ENCODER_POLARITY (-1) reverses direction when the encoder counts
	 * backward relative to the motor's positive-torque rotation direction. */
	int32_t pos_delta = raw - enc_offset;

	if (pos_delta >  (int32_t)(QDEC_PERIOD / 2)) { pos_delta -= QDEC_PERIOD; }
	if (pos_delta < -(int32_t)(QDEC_PERIOD / 2)) { pos_delta += QDEC_PERIOD; }

	pos_delta *= MOTOR_ENCODER_POLARITY;

	int32_t pos = ((pos_delta % CPR4) + CPR4) % CPR4;

	*theta_rad = (float)pos * TWO_PI / (float)CPR4;

	/* Velocity: windowed estimator with QDEC-period-correct wraparound */
	if (++omega_tick >= OMEGA_WINDOW) {
		omega_tick = 0;

		int32_t delta = raw - omega_raw_base;

		if (delta >  (int32_t)(QDEC_PERIOD / 2)) { delta -= QDEC_PERIOD; }
		if (delta < -(int32_t)(QDEC_PERIOD / 2)) { delta += QDEC_PERIOD; }

		delta *= MOTOR_ENCODER_POLARITY;

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
