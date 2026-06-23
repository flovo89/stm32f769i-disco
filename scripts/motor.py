#!/usr/bin/env python3
"""
BLDC FOC — Ethernet control script
Usage: python3 motor.py <command> [args]

Commands:
  enable
  disable
  speed   <rpm>
  torque  <amps>
  status
  monitor [interval_s]     continuous status (Ctrl-C to stop)
  plot    [interval_s]     live matplotlib plot (requires matplotlib)
  calibrate
  tune current <kp> <ki>
  tune speed   <kp> <ki>

Options:
  --host  Board IP  (default 192.168.7.100)
  --port  UDP port  (default 5000)
"""

import argparse
import socket
import sys
import time
import os

HOST    = "192.168.7.100"
PORT    = 5000
TIMEOUT = 1.0


# ── Transport ──────────────────────────────────────────────────────────────────

class Board:
    def __init__(self, host=HOST, port=PORT, timeout=TIMEOUT):
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(timeout)

    def send(self, cmd: str) -> str:
        self._sock.sendto((cmd.strip() + "\n").encode(), self._addr)
        try:
            data, _ = self._sock.recvfrom(1024)
            return data.decode().strip()
        except socket.timeout:
            return "TIMEOUT"

    def status(self) -> dict:
        raw = self.send("status")
        if "TIMEOUT" in raw or raw.startswith("ERR"):
            return {}
        result = {}
        for token in raw.split():
            if "=" in token:
                k, _, v = token.partition("=")
                try:
                    result[k] = float(v)
                except ValueError:
                    result[k] = v
        return result

    def close(self):
        self._sock.close()


# ── Formatting ─────────────────────────────────────────────────────────────────

def _state_color(state: str) -> str:
    colors = {
        "RUNNING":  "\033[32m",   # green
        "ALIGNING": "\033[33m",   # yellow
        "IDLE":     "\033[36m",   # cyan
        "ERROR":    "\033[31m",   # red
    }
    reset = "\033[0m"
    return colors.get(state, "") + state + reset


def _is_sim(s: dict) -> bool:
    return bool(s.get("SIM", 0))


def _fmt_status(s: dict) -> str:
    if not s:
        return "  No response from board"
    state = str(s.get("STATE", "?"))
    sim_tag = "  \033[35m[SIMULATION]\033[0m" if _is_sim(s) else ""
    return (
        f"  State  : {_state_color(state):<20}  Mode   : {s.get('MODE', '?')}{sim_tag}\n"
        f"  Speed  : {s.get('SPEED', 0):8.1f} rpm      Ref    : {s.get('SPEED_REF', 0):8.1f} rpm\n"
        f"  Ia     : {s.get('IA', 0):8.3f} A        Ib     : {s.get('IB', 0):8.3f} A\n"
        f"  Id     : {s.get('ID', 0):8.3f} A        Iq     : {s.get('IQ', 0):8.3f} A\n"
        f"  IqRef  : {s.get('IQ_REF', 0):8.3f} A        θ_e    : {s.get('THETA_E', 0):8.3f} rad\n"
        f"  Torque : {s.get('TORQUE', 0):8.4f} N·m      TqRef  : {s.get('TORQUE_REF', 0):8.4f} N·m\n"
        f"  Vbus   : {s.get('VBUS', 0):8.1f} V        OC     : {int(s.get('OC', 0))}\n"
    )


# ── Commands ───────────────────────────────────────────────────────────────────

def cmd_enable(b, _args):
    print(b.send("enable"))

def cmd_disable(b, _args):
    print(b.send("disable"))

def cmd_speed(b, args):
    print(b.send(f"set_speed {args.rpm}"))

def cmd_torque(b, args):
    print(b.send(f"set_torque {args.amps}"))

def cmd_status(b, _args):
    s = b.status()
    print(_fmt_status(s))

def cmd_calibrate(b, _args):
    confirm = input("Motor must be stopped. Continue? [y/N] ")
    if confirm.strip().lower() == "y":
        print(b.send("calibrate"))

def cmd_tune(b, args):
    if args.loop == "current":
        print(b.send(f"set_pid_current {args.kp} {args.ki}"))
    elif args.loop == "speed":
        print(b.send(f"set_pid_speed {args.kp} {args.ki}"))
    else:
        print(f"Unknown loop '{args.loop}'. Use 'current' or 'speed'.")


def cmd_monitor(b, args):
    interval = args.interval

    HEADER = (
        f"{'STATE':<10} {'MODE':<8} {'SPEED':>8} {'SPD_REF':>8} "
        f"{'IA':>7} {'IB':>7} {'ID':>7} {'IQ':>7} {'IQ_REF':>7} "
        f"{'TORQUE':>8} {'TQ_REF':>8} {'OC':>4}"
    )
    SEP = "─" * len(HEADER)

    print(f"Monitoring {b._addr[0]}:{b._addr[1]}  (Ctrl-C to stop)\n")

    try:
        header_printed = False
        while True:
            s = b.status()
            if not s:
                print("  <no response>")
            else:
                if not header_printed:
                    sim_note = "  [SIMULATION MODE]" if _is_sim(s) else ""
                    print(HEADER + sim_note)
                    print(SEP)
                    header_printed = True
                sim_mark = "*" if _is_sim(s) else " "
                print(
                    f"{str(s.get('STATE','?')):<10} "
                    f"{str(s.get('MODE','?')):<8} "
                    f"{s.get('SPEED',0):8.1f} "
                    f"{s.get('SPEED_REF',0):8.1f} "
                    f"{s.get('IA',0):7.3f} "
                    f"{s.get('IB',0):7.3f} "
                    f"{s.get('ID',0):7.3f} "
                    f"{s.get('IQ',0):7.3f} "
                    f"{s.get('IQ_REF',0):7.3f} "
                    f"{s.get('TORQUE',0):8.4f} "
                    f"{s.get('TORQUE_REF',0):8.4f} "
                    f"{int(s.get('OC',0)):4d}{sim_mark}"
                )
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nStopped.")


def cmd_plot(b, args):
    try:
        import matplotlib.pyplot as plt
        import matplotlib.animation as animation
        from collections import deque
    except ImportError:
        print("matplotlib not installed. Run: pip install matplotlib")
        sys.exit(1)

    interval_ms = int(args.interval * 1000)
    WINDOW = 200    # samples to show

    t_data     = deque(maxlen=WINDOW)
    speed      = deque(maxlen=WINDOW)
    spd_ref    = deque(maxlen=WINDOW)
    iq         = deque(maxlen=WINDOW)
    iq_ref     = deque(maxlen=WINDOW)
    ia         = deque(maxlen=WINDOW)
    ib         = deque(maxlen=WINDOW)
    torque     = deque(maxlen=WINDOW)
    torque_ref = deque(maxlen=WINDOW)

    t0 = time.time()
    sim_mode = [False]   # updated on first status response

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    title_base = f"BLDC FOC — {b._addr[0]}"
    fig.suptitle(title_base)

    ln_speed,   = ax1.plot([], [], label="Speed (rpm)",     color="tab:blue")
    ln_spd_ref, = ax1.plot([], [], label="Speed ref (rpm)", color="tab:blue",
                           linestyle="--", alpha=0.5)
    ax1.set_ylabel("Speed [rpm]")
    ax1.legend(loc="upper left")
    ax1.grid(True, alpha=0.3)

    ln_iq,     = ax2.plot([], [], label="Iq (A)",     color="tab:orange")
    ln_iq_ref, = ax2.plot([], [], label="Iq ref (A)", color="tab:orange",
                          linestyle="--", alpha=0.5)
    ln_ia,     = ax2.plot([], [], label="Ia (A)",     color="tab:green",  alpha=0.7)
    ln_ib,     = ax2.plot([], [], label="Ib (A)",     color="tab:red",    alpha=0.7)
    ax2.set_ylabel("Current [A]")
    ax2.legend(loc="upper left")
    ax2.grid(True, alpha=0.3)

    ln_torque,     = ax3.plot([], [], label="Torque (N·m)",     color="tab:purple")
    ln_torque_ref, = ax3.plot([], [], label="Torque ref (N·m)", color="tab:purple",
                              linestyle="--", alpha=0.5)
    ax3.set_ylabel("Torque [N·m]")
    ax3.set_xlabel("Time [s]")
    ax3.legend(loc="upper left")
    ax3.grid(True, alpha=0.3)

    state_text = ax1.text(0.01, 0.95, "", transform=ax1.transAxes,
                          fontsize=9, verticalalignment="top",
                          bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))

    def _update(_frame):
        s = b.status()
        if not s:
            return ln_speed, ln_spd_ref, ln_iq, ln_iq_ref, ln_ia, ln_ib, \
                   ln_torque, ln_torque_ref

        if _is_sim(s) and not sim_mode[0]:
            sim_mode[0] = True
            fig.suptitle(title_base + "  [SIMULATION]")

        t = time.time() - t0
        t_data.append(t)
        speed.append(s.get("SPEED", 0))
        spd_ref.append(s.get("SPEED_REF", 0))
        iq.append(s.get("IQ", 0))
        iq_ref.append(s.get("IQ_REF", 0))
        ia.append(s.get("IA", 0))
        ib.append(s.get("IB", 0))
        torque.append(s.get("TORQUE", 0))
        torque_ref.append(s.get("TORQUE_REF", 0))

        td = list(t_data)
        ln_speed.set_data(td, list(speed))
        ln_spd_ref.set_data(td, list(spd_ref))
        ln_iq.set_data(td, list(iq))
        ln_iq_ref.set_data(td, list(iq_ref))
        ln_ia.set_data(td, list(ia))
        ln_ib.set_data(td, list(ib))
        ln_torque.set_data(td, list(torque))
        ln_torque_ref.set_data(td, list(torque_ref))

        for ax in (ax1, ax2, ax3):
            ax.relim()
            ax.autoscale_view()

        state_text.set_text(f"State: {s.get('STATE','?')}  OC: {int(s.get('OC',0))}")
        return ln_speed, ln_spd_ref, ln_iq, ln_iq_ref, ln_ia, ln_ib, \
               ln_torque, ln_torque_ref

    ani = animation.FuncAnimation(fig, _update, interval=interval_ms,
                                  blit=False, cache_frame_data=False)
    plt.tight_layout()
    try:
        plt.show()
    except KeyboardInterrupt:
        pass


# ── Argument parsing ───────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="BLDC FOC Ethernet control",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)

    sub = parser.add_subparsers(dest="cmd", metavar="command")
    sub.required = True

    sub.add_parser("enable",    help="Enable motor (runs alignment first)")
    sub.add_parser("disable",   help="Disable motor")
    sub.add_parser("status",    help="Print current status")
    sub.add_parser("calibrate", help="Zero current sensors (motor must be stopped)")

    p = sub.add_parser("speed", help="Set speed reference [rpm]")
    p.add_argument("rpm", type=float)

    p = sub.add_parser("torque", help="Set Iq reference [A]")
    p.add_argument("amps", type=float)

    p = sub.add_parser("monitor", help="Continuous status readout")
    p.add_argument("interval", type=float, nargs="?", default=0.2,
                   metavar="interval_s")

    p = sub.add_parser("plot", help="Live matplotlib plot")
    p.add_argument("interval", type=float, nargs="?", default=0.1,
                   metavar="interval_s")

    p = sub.add_parser("tune", help="Set PID gains")
    p.add_argument("loop", choices=["current", "speed"])
    p.add_argument("kp", type=float)
    p.add_argument("ki", type=float)

    args = parser.parse_args()

    b = Board(host=args.host, port=args.port)

    dispatch = {
        "enable":    cmd_enable,
        "disable":   cmd_disable,
        "speed":     cmd_speed,
        "torque":    cmd_torque,
        "status":    cmd_status,
        "calibrate": cmd_calibrate,
        "monitor":   cmd_monitor,
        "plot":      cmd_plot,
        "tune":      cmd_tune,
    }

    try:
        dispatch[args.cmd](b, args)
    finally:
        b.close()


if __name__ == "__main__":
    main()
