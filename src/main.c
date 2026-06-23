#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "foc/foc.h"
#include "motor/motor.h"
#include "interface/cmd.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* ─── Thread definitions ─────────────────────────────────────────────────── */

#define FOC_THREAD_STACK  2048
#define UDP_THREAD_STACK  4096
#define FOC_THREAD_PRIO   (-2)   /* Cooperative, high priority */
#define UDP_THREAD_PRIO     5

K_THREAD_STACK_DEFINE(foc_stack, FOC_THREAD_STACK);
K_THREAD_STACK_DEFINE(udp_stack, UDP_THREAD_STACK);

static struct k_thread foc_thread;
static struct k_thread udp_thread;

/* ─── FOC control loop ───────────────────────────────────────────────────── */

/*
 * Timer fires at FOC_CONTROL_HZ, posts semaphore to wake control thread.
 * This avoids busy-waiting while still giving precise timing.
 */
static K_SEM_DEFINE(foc_sem, 0, 1);
static struct k_timer foc_timer;

static void foc_timer_cb(struct k_timer *t)
{
	ARG_UNUSED(t);
	k_sem_give(&foc_sem);
}

static void foc_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	k_timer_init(&foc_timer, foc_timer_cb, NULL);
	k_timer_start(&foc_timer,
	              K_USEC(1000000 / FOC_CONTROL_HZ),
	              K_USEC(1000000 / FOC_CONTROL_HZ));

	LOG_INF("FOC control loop started at %d Hz", FOC_CONTROL_HZ);

	while (1) {
		k_sem_take(&foc_sem, K_FOREVER);

		float ia, ib, theta, omega;

		if (motor_read_currents(&ia, &ib) != 0) {
			/* ADC error — keep running with zero current */
			ia = ib = 0.0f;
		}

		motor_read_encoder(&theta, &omega);
		foc_step(&g_foc, ia, ib, theta, omega);
		motor_set_pwm(g_foc.da, g_foc.db, g_foc.dc);
	}
}

/* ─── UDP server thread ──────────────────────────────────────────────────── */

static void udp_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	/* Give networking stack time to come up */
	k_sleep(K_SECONDS(2));

	udp_server_run();  /* blocks forever */
}

/* ─── main ───────────────────────────────────────────────────────────────── */

int main(void)
{
	LOG_INF("BLDC FOC Controller — STM32F769I-DISCO");
	LOG_INF("Firmware built " __DATE__ " " __TIME__);

	/* Initialise motor hardware */
	int ret = motor_init();

	if (ret) {
		LOG_ERR("motor_init failed: %d — halting", ret);
		return ret;
	}

	/* Initialise FOC context */
	foc_init(&g_foc, MOTOR_VBUS_V);

	/* USB is initialised automatically by the kernel via the
	 * cdc_acm_uart0 node in the device tree overlay. */

	/* Start control thread */
	k_thread_create(&foc_thread, foc_stack, FOC_THREAD_STACK,
	                foc_thread_fn, NULL, NULL, NULL,
	                FOC_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&foc_thread, "foc_ctrl");

	/* Start UDP server thread */
	k_thread_create(&udp_thread, udp_stack, UDP_THREAD_STACK,
	                udp_thread_fn, NULL, NULL, NULL,
	                UDP_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&udp_thread, "udp_srv");

	LOG_INF("Ready — connect via USB serial or UDP %s:%d",
	        CONFIG_NET_CONFIG_MY_IPV4_ADDR, 5000);

	return 0;
}
