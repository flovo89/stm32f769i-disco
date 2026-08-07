/*
 * Command interface — two backends:
 *
 *  1. Zephyr Shell over USB CDC-ACM (interactive terminal)
 *  2. UDP server on port 5000 (same text protocol, for the Python script)
 *
 * Text protocol (both backends):
 *   Request  → single line, '\n' terminated
 *   Response ← single line, '\n' terminated
 *
 * Commands:
 *   enable
 *   disable
 *   set_speed <rpm>
 *   set_torque <amps>
 *   set_pid_current <kp> <ki>
 *   set_pid_speed <kp> <ki>
 *   calibrate
 *   status
 */

#include "cmd.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/net/socket.h>
#include <zephyr/shell/shell.h>

#include "motor/motor.h"

LOG_MODULE_REGISTER(cmd, LOG_LEVEL_INF);

foc_ctx_t g_foc;  /* definition — declared extern in cmd.h */

#define UDP_PORT 5000
#define BUF_LEN  256

/* ─── MOSFET test helper ─────────────────────────────────────────────────── */

/*
 * Applies 6 duty-cycle vectors that exercise each high-side and low-side
 * MOSFET in turn.  If sh != NULL, prints per-vector results to the shell.
 * Always writes a summary line to *summary / slen.
 *
 * Requires: motor not in RUNNING/ALIGNING state.
 * Leaves:   motor disabled, FOC in IDLE.
 */
static void run_mosfet_test(const struct shell *sh, char *summary, size_t slen)
{
	if (g_foc.state == FOC_STATE_RUNNING ||
	    g_foc.state == FOC_STATE_ALIGNING) {
		snprintf(summary, slen, "ERR disable motor before mosfet_test");
		return;
	}

	/* delta: duty offset that produces vlim across one phase pair */
	float delta = g_foc.vlim / (2.0f * g_foc.vbus);

	typedef struct {
		const char *name;
		float da, db, dc;
		int sa, sb, sc;   /* expected sign: +1 positive, -1 negative, 0 near-zero */
	} vec_t;

	/* HS = high-side on, LS = low-side on.
	 * ia,ib from ADC; ic = -(ia+ib). */
	vec_t vecs[6] = {
		{"AH-BL", 0.5f+delta, 0.5f-delta, 0.5f,       +1, -1,  0},
		{"AH-CL", 0.5f+delta, 0.5f,       0.5f-delta,  +1,  0, -1},
		{"BH-AL", 0.5f-delta, 0.5f+delta, 0.5f,        -1, +1,  0},
		{"BH-CL", 0.5f,       0.5f+delta, 0.5f-delta,   0, +1, -1},
		{"CH-AL", 0.5f-delta, 0.5f,       0.5f+delta,  -1,  0, +1},
		{"CH-BL", 0.5f,       0.5f-delta, 0.5f+delta,   0, -1, +1},
	};

	/* Wait up to 3 s for rotor to coast to a stop before applying test vectors.
	 * Back-EMF from a spinning rotor swamps the small test voltage on the
	 * neutral (floating) phase and causes spurious failures. */
	{
		float th, om;
		for (int w = 0; w < 30; w++) {
			motor_read_encoder(&th, &om);
			if (fabsf(om) < 2.0f) { break; }
			k_sleep(K_MSEC(100));
		}
	}

	motor_enable(true);

	if (sh) {
		shell_print(sh, "MOSFET test  delta=%.3f  vlim=%.2f V",
		            (double)delta, (double)g_foc.vlim);
		shell_print(sh, "Vec  Phases    ia      ib      ic      Result");
		shell_print(sh, "---  -------  ------  ------  ------  ------");
	}

	int pass = 0, fail = 0;

	for (int i = 0; i < 6; i++) {
		vec_t *v = &vecs[i];

		foc_set_forced_duty(&g_foc, v->da, v->db, v->dc);
		k_sleep(K_MSEC(40));   /* let current settle */

		float sum_a = 0.0f, sum_b = 0.0f;
		bool  oc    = false;

		for (int s = 0; s < 5; s++) {
			if (g_foc.state == FOC_STATE_ERROR) { oc = true; break; }
			sum_a += g_foc.ia;
			sum_b += g_foc.ib;
			k_sleep(K_MSEC(5));
		}

		/* return to neutral before evaluating */
		foc_set_forced_duty(&g_foc, 0.5f, 0.5f, 0.5f);
		k_sleep(K_MSEC(10));

		if (oc) {
			if (sh) {
				shell_print(sh, "%d    %-7s  OC (overcurrent tripped)",
				            i + 1, v->name);
			}
			fail++;
			foc_reset(&g_foc);
			motor_enable(true);
			continue;
		}

		float ia = sum_a / 5.0f;
		float ib = sum_b / 5.0f;
		float ic = -(ia + ib);

/* Neutral phase (sign==0) is not checked: back-EMF from rotor motion
 * circulates current through all phases and makes ic=-(ia+ib) unreliable. */
#define CHKSIGN(val, sign) \
	((sign) > 0 ? (val) > 0.10f : (sign) < 0 ? (val) < -0.10f : true)

		bool ok = CHKSIGN(ia, v->sa) && CHKSIGN(ib, v->sb) && CHKSIGN(ic, v->sc);

#undef CHKSIGN

		if (ok) { pass++; } else { fail++; }

		if (sh) {
			shell_print(sh, "%d    %-7s  %+.3f  %+.3f  %+.3f  %s",
			            i + 1, v->name,
			            (double)ia, (double)ib, (double)ic,
			            ok ? "PASS" : "FAIL");
		}
	}

	foc_reset(&g_foc);
	motor_enable(false);

	snprintf(summary, slen, "MOSFET test done: %d/6 PASS  %d/6 FAIL", pass, fail);
}

/* ─── Shared command dispatcher ─────────────────────────────────────────── */

static void dispatch(const char *line, char *resp, size_t resp_len)
{
	char cmd[64] = {0};
	float f1 = 0.0f, f2 = 0.0f, f3 = 0.0f, f4 = 0.0f, f5 = 0.0f;
	int   n  = sscanf(line, "%63s %f %f %f %f %f", cmd, &f1, &f2, &f3, &f4, &f5);

	if (n < 1) {
		snprintf(resp, resp_len, "ERR empty command");
		return;
	}

	if (strcmp(cmd, "enable") == 0) {
		motor_enable(true);
		k_sleep(K_MSEC(10)); /* gate driver wake-up before FOC reads currents */
		foc_enable(&g_foc, true);
		snprintf(resp, resp_len, "OK");

	} else if (strcmp(cmd, "disable") == 0) {
		foc_enable(&g_foc, false);
		motor_enable(false);
		snprintf(resp, resp_len, "OK");

	} else if (strcmp(cmd, "set_speed") == 0) {
		if (n < 2) { snprintf(resp, resp_len, "ERR usage: set_speed <rpm>"); return; }
		foc_set_mode(&g_foc, FOC_MODE_SPEED);
		foc_set_speed_ref(&g_foc, f1);
		snprintf(resp, resp_len, "OK speed_ref=%.1f", (double)f1);

	} else if (strcmp(cmd, "set_torque") == 0) {
		if (n < 2) { snprintf(resp, resp_len, "ERR usage: set_torque <amps>"); return; }
		foc_set_mode(&g_foc, FOC_MODE_TORQUE);
		foc_set_torque_ref(&g_foc, f1);
		snprintf(resp, resp_len, "OK iq_ref=%.3f", (double)f1);

	} else if (strcmp(cmd, "set_pid_current") == 0) {
		if (n < 3) { snprintf(resp, resp_len, "ERR usage: set_pid_current <kp> <ki>"); return; }
		foc_tune_current_pid(&g_foc, f1, f2);
		snprintf(resp, resp_len, "OK kp=%.4f ki=%.4f", (double)f1, (double)f2);

	} else if (strcmp(cmd, "reboot") == 0) {
		snprintf(resp, resp_len, "OK rebooting...");
		/* Give the response time to be sent before resetting */
		k_sleep(K_MSEC(50));
		sys_reboot(SYS_REBOOT_COLD);

	} else if (strcmp(cmd, "force_duty") == 0) {
		/* force_duty <da> <db> <dc>  — hold fixed duty cycles indefinitely.
		 * Uses FOC_STATE_FORCED so the control thread does not overwrite them.
		 * Use 'disable' to exit. Values in [0.0, 1.0], 0.5 = zero voltage. */
		if (n < 4) {
			snprintf(resp, resp_len,
			         "ERR usage: force_duty <da> <db> <dc>  (0.0-1.0, 0.5=zero)");
			return;
		}
		float da = f1 < 0.0f ? 0.0f : (f1 > 1.0f ? 1.0f : f1);
		float db = f2 < 0.0f ? 0.0f : (f2 > 1.0f ? 1.0f : f2);
		float dc = f3 < 0.0f ? 0.0f : (f3 > 1.0f ? 1.0f : f3);

		motor_enable(true);
		foc_set_forced_duty(&g_foc, da, db, dc);
		snprintf(resp, resp_len, "OK da=%.3f db=%.3f dc=%.3f  (disable to exit)",
		         (double)da, (double)db, (double)dc);

	} else if (strcmp(cmd, "set_vmax") == 0) {
		if (n < 2) {
			snprintf(resp, resp_len, "ERR usage: set_vmax <V>  (0 = full range)");
			return;
		}
		foc_set_vlim(&g_foc, f1);
		snprintf(resp, resp_len, "OK vlim=%.2fV (full=%.2fV)",
		         (double)g_foc.vlim, (double)(g_foc.vbus / 1.73205f));

	} else if (strcmp(cmd, "mosfet_test") == 0) {
		run_mosfet_test(NULL, resp, resp_len);

	} else if (strcmp(cmd, "set_motor_params") == 0) {
		/* set_motor_params <L_mH> <psi_Wb> — update feedforward model */
		if (n < 3) {
			snprintf(resp, resp_len,
			         "ERR usage: set_motor_params <L_mH> <psi_Wb>");
			return;
		}
		g_foc.L_motor   = f1 * 1e-3f;
		g_foc.psi_motor = f2;
		snprintf(resp, resp_len,
		         "OK L=%.3fmH psi=%.4f", (double)f1, (double)f2);

	} else if (strcmp(cmd, "set_pid_speed") == 0) {
		if (n < 3) { snprintf(resp, resp_len, "ERR usage: set_pid_speed <kp> <ki>"); return; }
		foc_tune_speed_pid(&g_foc, f1, f2);
		snprintf(resp, resp_len, "OK kp=%.4f ki=%.4f", (double)f1, (double)f2);

	} else if (strcmp(cmd, "calibrate") == 0) {
		if (motor_is_enabled()) {
			snprintf(resp, resp_len, "ERR disable motor first");
			return;
		}
		int r = motor_calibrate_currents();

		snprintf(resp, resp_len, r == 0 ? "OK" : "ERR adc");

	} else if (strcmp(cmd, "status") == 0) {
		/* T = 1.5 · p · ψ · iq  [N·m] */
		float kt = 1.5f * FOC_POLE_PAIRS * g_foc.psi_motor;
		snprintf(resp, resp_len,
		         "STATE=%s MODE=%s SPEED=%.1f SPEED_REF=%.1f "
		         "IA=%.3f IB=%.3f ID=%.3f IQ=%.3f IQ_REF=%.3f "
		         "VD=%.3f VQ=%.3f "
		         "TORQUE=%.4f TORQUE_REF=%.4f "
		         "THETA_E=%.3f VBUS=%.1f OC=%u "
		         "DA=%.3f DB=%.3f DC=%.3f LOOPS=%u"
#ifdef CONFIG_MOTOR_SIM
		         " SIM=1"
#endif
		         ,
		         foc_state_str(g_foc.state),
		         g_foc.mode == FOC_MODE_SPEED ? "SPEED" : "TORQUE",
		         (double)g_foc.speed_rpm,
		         (double)g_foc.speed_ref,
		         (double)g_foc.ia, (double)g_foc.ib,
		         (double)g_foc.id, (double)g_foc.iq,
		         (double)g_foc.iq_ref,
		         (double)g_foc.vd, (double)g_foc.vq,
		         (double)(kt * g_foc.iq),
		         (double)(kt * g_foc.iq_ref),
		         (double)g_foc.theta_e,
		         (double)g_foc.vbus,
		         g_foc.overcurrent_count,
		         (double)g_foc.da, (double)g_foc.db, (double)g_foc.dc,
		         g_foc.loop_count);

#ifdef CONFIG_MOTOR_SIM
	} else if (strcmp(cmd, "sim_load") == 0) {
		if (n < 2) {
			snprintf(resp, resp_len, "ERR usage: sim_load <Nm>");
			return;
		}
		motor_sim_set_load(f1);
		snprintf(resp, resp_len, "OK T_load=%.4f Nm", (double)f1);

	} else if (strcmp(cmd, "sim_info") == 0) {
		const sim_ctx_t *s = motor_sim_get_ctx();
		snprintf(resp, resp_len,
		         "R=%.3f L=%.6f psi=%.4f J=%.2e B=%.2e "
		         "T_load=%.4f id=%.3f iq=%.3f omega=%.2f rpm=%.1f",
		         (double)s->R, (double)s->L,
		         (double)s->psi_m, (double)s->J,
		         (double)s->B_fric, (double)s->T_load,
		         (double)s->id, (double)s->iq,
		         (double)s->omega_mech,
		         (double)sim_get_speed_rpm(s));

	} else if (strcmp(cmd, "sim_params") == 0) {
		/* sim_params <R_ohm> <L_mH> <psi_Wb> <J_kgm2> <B_Nms> */
		if (n < 6) {
			snprintf(resp, resp_len,
			         "ERR usage: sim_params <R_ohm> <L_mH> <psi_Wb> <J_kgm2> <B_Nms>");
			return;
		}
		motor_sim_set_params(f1, f2 * 1e-3f, f3, f4, f5);
		snprintf(resp, resp_len,
		         "OK R=%.3f L=%.3fmH psi=%.4f J=%.2e B=%.2e",
		         (double)f1, (double)f2, (double)f3,
		         (double)f4, (double)f5);
#endif /* CONFIG_MOTOR_SIM */

	} else {
		snprintf(resp, resp_len,
		         "ERR unknown command '%s'  "
		         "[enable|disable|set_speed|set_torque|"
		         "set_pid_current|set_pid_speed|set_motor_params|set_vmax|"
		         "calibrate|status|mosfet_test|force_duty|reboot"
#ifdef CONFIG_MOTOR_SIM
		         "|sim_load|sim_info|sim_params"
#endif
		         "]", cmd);
	}
}

/* ─── Zephyr Shell backend ───────────────────────────────────────────────── */

static int sh_enable(const struct shell *sh, size_t argc, char **argv)
{
	char resp[BUF_LEN];

	dispatch("enable", resp, sizeof(resp));
	shell_print(sh, "%s", resp);
	return 0;
}

static int sh_disable(const struct shell *sh, size_t argc, char **argv)
{
	char resp[BUF_LEN];

	dispatch("disable", resp, sizeof(resp));
	shell_print(sh, "%s", resp);
	return 0;
}

static int sh_set_speed(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Usage: set_speed <rpm>");
		return -EINVAL;
	}
	char line[64];
	char resp[BUF_LEN];

	snprintf(line, sizeof(line), "set_speed %s", argv[1]);
	dispatch(line, resp, sizeof(resp));
	shell_print(sh, "%s", resp);
	return 0;
}

static int sh_set_torque(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Usage: set_torque <amps>");
		return -EINVAL;
	}
	char line[64];
	char resp[BUF_LEN];

	snprintf(line, sizeof(line), "set_torque %s", argv[1]);
	dispatch(line, resp, sizeof(resp));
	shell_print(sh, "%s", resp);
	return 0;
}

static int sh_status(const struct shell *sh, size_t argc, char **argv)
{
	char resp[BUF_LEN];

	dispatch("status", resp, sizeof(resp));
	shell_print(sh, "%s", resp);
	return 0;
}

static int sh_calibrate(const struct shell *sh, size_t argc, char **argv)
{
	char resp[BUF_LEN];

	dispatch("calibrate", resp, sizeof(resp));
	shell_print(sh, "%s", resp);
	return 0;
}

static int sh_set_pid_current(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		shell_error(sh, "Usage: set_pid_current <kp> <ki>");
		return -EINVAL;
	}
	char line[64];
	char resp[BUF_LEN];

	snprintf(line, sizeof(line), "set_pid_current %s %s", argv[1], argv[2]);
	dispatch(line, resp, sizeof(resp));
	shell_print(sh, "%s", resp);
	return 0;
}

static int sh_set_pid_speed(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		shell_error(sh, "Usage: set_pid_speed <kp> <ki>");
		return -EINVAL;
	}
	char line[64];
	char resp[BUF_LEN];

	snprintf(line, sizeof(line), "set_pid_speed %s %s", argv[1], argv[2]);
	dispatch(line, resp, sizeof(resp));
	shell_print(sh, "%s", resp);
	return 0;
}

static int sh_set_motor_params(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 3) {
		shell_error(sh, "Usage: set_motor_params <L_mH> <psi_Wb>");
		return -EINVAL;
	}
	char line[64];
	char resp[BUF_LEN];

	snprintf(line, sizeof(line), "set_motor_params %s %s", argv[1], argv[2]);
	dispatch(line, resp, sizeof(resp));
	shell_print(sh, "%s", resp);
	return 0;
}

static int sh_reboot(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh, "Rebooting...");
	k_sleep(K_MSEC(50));
	sys_reboot(SYS_REBOOT_COLD);
	return 0;
}

static int sh_force_duty(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 4) {
		shell_error(sh, "Usage: force_duty <da> <db> <dc>  (0.0-1.0, 0.5=zero voltage)");
		return -EINVAL;
	}
	char line[96];
	char resp[BUF_LEN];

	snprintf(line, sizeof(line), "force_duty %s %s %s", argv[1], argv[2], argv[3]);
	dispatch(line, resp, sizeof(resp));
	shell_print(sh, "%s", resp);
	return 0;
}

static int sh_set_vmax(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Usage: set_vmax <V>  (0 = full range = Vbus/sqrt3)");
		return -EINVAL;
	}
	char line[64];
	char resp[BUF_LEN];

	snprintf(line, sizeof(line), "set_vmax %s", argv[1]);
	dispatch(line, resp, sizeof(resp));
	shell_print(sh, "%s", resp);
	return 0;
}

static int sh_mosfet_test(const struct shell *sh, size_t argc, char **argv)
{
	char summary[128];

	run_mosfet_test(sh, summary, sizeof(summary));
	shell_print(sh, "%s", summary);
	return 0;
}

SHELL_CMD_ARG_REGISTER(enable,          NULL, "Enable motor",                sh_enable,         1, 0);
SHELL_CMD_ARG_REGISTER(disable,         NULL, "Disable motor",               sh_disable,        1, 0);
SHELL_CMD_ARG_REGISTER(set_speed,       NULL, "Set speed [RPM]",             sh_set_speed,      2, 0);
SHELL_CMD_ARG_REGISTER(set_torque,      NULL, "Set torque Iq [A]",           sh_set_torque,     2, 0);
SHELL_CMD_ARG_REGISTER(status,          NULL, "Print motor status",          sh_status,         1, 0);
SHELL_CMD_ARG_REGISTER(calibrate,       NULL, "Zero current sensors",        sh_calibrate,      1, 0);
SHELL_CMD_ARG_REGISTER(set_pid_current,   NULL, "Tune current PID <kp> <ki>",    sh_set_pid_current,   3, 0);
SHELL_CMD_ARG_REGISTER(set_pid_speed,     NULL, "Tune speed PID <kp> <ki>",      sh_set_pid_speed,     3, 0);
SHELL_CMD_ARG_REGISTER(set_motor_params,  NULL, "Set feedforward: <L_mH> <psi>", sh_set_motor_params,  3, 0);
SHELL_CMD_ARG_REGISTER(set_vmax,          NULL, "Limit output voltage [V] (0=full)", sh_set_vmax,     2, 0);
SHELL_CMD_ARG_REGISTER(force_duty,        NULL, "Force PWM duty <da> <db> <dc>",     sh_force_duty,   4, 0);
SHELL_CMD_ARG_REGISTER(reboot,            NULL, "Reboot the board",                  sh_reboot,       1, 0);
SHELL_CMD_ARG_REGISTER(mosfet_test,       NULL, "Test all 6 MOSFETs",                sh_mosfet_test,  1, 0);

#ifdef CONFIG_MOTOR_SIM
static int sh_sim_load(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 2) {
		shell_error(sh, "Usage: sim_load <Nm>");
		return -EINVAL;
	}
	char line[64];
	char resp[BUF_LEN];

	snprintf(line, sizeof(line), "sim_load %s", argv[1]);
	dispatch(line, resp, sizeof(resp));
	shell_print(sh, "%s", resp);
	return 0;
}

static int sh_sim_info(const struct shell *sh, size_t argc, char **argv)
{
	char resp[BUF_LEN];

	dispatch("sim_info", resp, sizeof(resp));
	shell_print(sh, "%s", resp);
	return 0;
}

static int sh_sim_params(const struct shell *sh, size_t argc, char **argv)
{
	if (argc < 6) {
		shell_error(sh, "Usage: sim_params <R_ohm> <L_mH> <psi_Wb> <J_kgm2> <B_Nms>");
		return -EINVAL;
	}
	char line[96];
	char resp[BUF_LEN];

	snprintf(line, sizeof(line), "sim_params %s %s %s %s %s",
	         argv[1], argv[2], argv[3], argv[4], argv[5]);
	dispatch(line, resp, sizeof(resp));
	shell_print(sh, "%s", resp);
	return 0;
}

SHELL_CMD_ARG_REGISTER(sim_load,   NULL, "Set simulated load torque [Nm]",         sh_sim_load,   2, 0);
SHELL_CMD_ARG_REGISTER(sim_info,   NULL, "Show simulation model state",             sh_sim_info,   1, 0);
SHELL_CMD_ARG_REGISTER(sim_params, NULL, "Set motor params: R L_mH psi J B",       sh_sim_params, 6, 0);
#endif /* CONFIG_MOTOR_SIM */

/* ─── UDP server ─────────────────────────────────────────────────────────── */

void udp_server_run(void)
{
	struct sockaddr_in addr = {
		.sin_family = AF_INET,
		.sin_port   = htons(UDP_PORT),
	};
	addr.sin_addr.s_addr = INADDR_ANY;

	int sock = zsock_socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	if (sock < 0) {
		LOG_ERR("UDP socket: %d", errno);
		return;
	}

	if (zsock_bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		LOG_ERR("UDP bind: %d", errno);
		zsock_close(sock);
		return;
	}

	LOG_INF("UDP server listening on port %d", UDP_PORT);

	static char rx[BUF_LEN];
	static char tx[BUF_LEN];

	while (1) {
		struct sockaddr_in client;
		socklen_t          clen = sizeof(client);

		ssize_t n = zsock_recvfrom(sock, rx, sizeof(rx) - 1, 0,
		                           (struct sockaddr *)&client, &clen);
		if (n <= 0) { continue; }

		rx[n] = '\0';

		/* Strip trailing newline */
		for (int i = n - 1; i >= 0 && (rx[i] == '\n' || rx[i] == '\r'); i--) {
			rx[i] = '\0';
		}

		dispatch(rx, tx, sizeof(tx));

		size_t tlen = strlen(tx);
		tx[tlen]    = '\n';
		zsock_sendto(sock, tx, tlen + 1, 0,
		             (struct sockaddr *)&client, clen);
	}
}
