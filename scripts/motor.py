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

def cmd_vmax(b, args):
    print(b.send(f"set_vmax {args.volts}"))

def cmd_mosfet_test(b, _args):
    """Send mosfet_test over UDP — gets compact summary (multi-line output requires shell)."""
    print(b.send("mosfet_test"))


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
    import threading

    interval_ms = int(args.interval * 1000)
    WINDOW = 200    # samples to show

    # Single deque of (t, speed, spd_ref, iq, iq_ref, ia, ib, torque,
    # torque_ref, theta_e, pos, pos_ref) tuples — appended atomically.
    records = deque(maxlen=WINDOW)

    # Mutable state shared between poll thread and _update (GUI thread).
    latest_state  = ["?"]
    latest_oc     = [0]
    consecutive_timeouts = [0]
    sim_detected  = [False]
    poll_running  = [True]

    t0 = time.time()

    # Separate Board with a short timeout so the poll thread never blocks
    # longer than the animation interval.
    # Stored in a list so _poll can replace it on socket errors without
    # needing a nonlocal declaration (Python 2 style, but avoids closure
    # issues with daemon thread lifetime).
    poll_timeout = min(args.interval * 0.8, 0.4)
    poll_board   = [Board(b._addr[0], b._addr[1], timeout=poll_timeout)]

    def _poll():
        while poll_running[0]:
            t_start = time.time()
            try:
                s = poll_board[0].status()
            except OSError:
                if not poll_running[0]:
                    break   # intentional shutdown from finally block
                # Network error (interface down during board reboot).
                # Close the broken socket, wait for the link to recover,
                # then open a fresh socket so ARP is re-resolved.
                try:
                    poll_board[0].close()
                except OSError:
                    pass
                consecutive_timeouts[0] += 1
                time.sleep(2.0)
                if poll_running[0]:
                    poll_board[0] = Board(b._addr[0], b._addr[1],
                                          timeout=poll_timeout)
                continue
            if s:
                consecutive_timeouts[0] = 0
                records.append((
                    time.time() - t0,
                    s.get("SPEED", 0),    s.get("SPEED_REF", 0),
                    s.get("IQ", 0),       s.get("IQ_REF", 0),
                    s.get("IA", 0),       s.get("IB", 0),
                    s.get("TORQUE", 0),   s.get("TORQUE_REF", 0),
                    s.get("THETA_E", 0),
                    s.get("POS", 0),      s.get("POS_REF", 0),
                ))
                latest_state[0] = str(s.get("STATE", "?"))
                latest_oc[0]    = int(s.get("OC", 0))
                if _is_sim(s):
                    sim_detected[0] = True
            else:
                consecutive_timeouts[0] += 1
            elapsed = time.time() - t_start
            remaining = args.interval - elapsed
            if remaining > 0:
                time.sleep(remaining)

    fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, figsize=(10, 12), sharex=True)
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
    ax3.legend(loc="upper left")
    ax3.grid(True, alpha=0.3)

    ln_theta_e, = ax4.plot([], [], label="θ_e (rad)", color="tab:cyan")
    ax4.axhline(3.14159, color="gray", linestyle=":", linewidth=0.8, alpha=0.6)
    ax4.axhline(6.28318, color="gray", linestyle=":", linewidth=0.8, alpha=0.6)
    ax4.set_ylabel("θ_e [rad]")
    ax4.set_ylim(-0.1, 6.4)
    ax4.legend(loc="upper left")
    ax4.grid(True, alpha=0.3)

    ln_pos,     = ax5.plot([], [], label="Position (°)",     color="tab:brown")
    ln_pos_ref, = ax5.plot([], [], label="Position ref (°)", color="tab:brown",
                           linestyle="--", alpha=0.5)
    ax5.set_ylabel("Position [°]")
    ax5.set_xlabel("Time [s]")
    ax5.legend(loc="upper left")
    ax5.grid(True, alpha=0.3)

    state_text = ax1.text(0.01, 0.95, "", transform=ax1.transAxes,
                          fontsize=9, verticalalignment="top",
                          bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.5))
    # Shown when no data is arriving (board unreachable)
    no_data_text = ax1.text(0.5, 0.5, "Waiting for board…",
                            transform=ax1.transAxes,
                            fontsize=13, color="gray",
                            ha="center", va="center", visible=False)

    def _clamp_ylim(ax, min_span):
        """Expand y-axis to min_span when data range is too small to be visible."""
        lo, hi = ax.get_ylim()
        if hi - lo < min_span:
            center = (lo + hi) / 2
            # auto=True re-enables autoscaling so the next frame can expand further
            ax.set_ylim(center - min_span / 2, center + min_span / 2, auto=True)

    def _update(_frame):
        if sim_detected[0]:
            fig.suptitle(title_base + "  [SIMULATION]")
            sim_detected[0] = False

        recs = list(records)   # single atomic snapshot — no per-deque race
        no_data = not recs
        no_data_text.set_visible(no_data)
        if no_data:
            if consecutive_timeouts[0] > 5:
                no_data_text.set_text(
                    f"No response from board ({consecutive_timeouts[0]} timeouts)")
            else:
                no_data_text.set_text("Waiting for board…")
            return

        td, spd, sref, iq_, iqr, ia_, ib_, tq, tqr, th, pos, pref = zip(*recs)

        ln_speed.set_data(td, spd)
        ln_spd_ref.set_data(td, sref)
        ln_iq.set_data(td, iq_)
        ln_iq_ref.set_data(td, iqr)
        ln_ia.set_data(td, ia_)
        ln_ib.set_data(td, ib_)
        ln_torque.set_data(td, tq)
        ln_torque_ref.set_data(td, tqr)
        ln_theta_e.set_data(td, th)
        ln_pos.set_data(td, pos)
        ln_pos_ref.set_data(td, pref)

        for ax in (ax1, ax2, ax3):
            ax.relim()
            ax.autoscale_view()
        ax4.relim()
        ax4.autoscale_view(scaley=False)
        ax5.relim()
        ax5.autoscale_view()

        # Keep y-ranges wide enough that near-zero signals remain visible
        _clamp_ylim(ax1, 200)    # ±100 RPM
        _clamp_ylim(ax2, 4.0)    # ±2 A
        _clamp_ylim(ax3, 0.10)   # ±0.05 N·m
        _clamp_ylim(ax5, 10.0)   # ±5°

        state_text.set_text(f"State: {latest_state[0]}  OC: {latest_oc[0]}")

    poll_thread = threading.Thread(target=_poll, daemon=True)
    poll_thread.start()

    ani = animation.FuncAnimation(fig, _update, interval=interval_ms,
                                  blit=False, cache_frame_data=False)
    plt.tight_layout()
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        poll_running[0] = False
        try:
            poll_board[0].close()
        except OSError:
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

    p = sub.add_parser("vmax", help="Limit output voltage [V] (0 = full Vbus/sqrt3)")
    p.add_argument("volts", type=float)

    sub.add_parser("mosfet_test", help="Test all 6 MOSFETs (use shell for full output)")

    args = parser.parse_args()

    b = Board(host=args.host, port=args.port)

    dispatch = {
        "enable":      cmd_enable,
        "disable":     cmd_disable,
        "speed":       cmd_speed,
        "torque":      cmd_torque,
        "status":      cmd_status,
        "calibrate":   cmd_calibrate,
        "monitor":     cmd_monitor,
        "plot":        cmd_plot,
        "tune":        cmd_tune,
        "vmax":        cmd_vmax,
        "mosfet_test": cmd_mosfet_test,
    }

    try:
        dispatch[args.cmd](b, args)
    finally:
        b.close()


if __name__ == "__main__":
    main()
