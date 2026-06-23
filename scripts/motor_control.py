#!/usr/bin/env python3
"""
BLDC FOC Motor Controller — Python host script

Connects to the STM32F769I-DISCO over UDP (port 5000) and provides
both an interactive menu and a simple Python API.

Usage:
    python3 motor_control.py [--host 192.168.1.100] [--port 5000]
"""

import argparse
import socket
import sys
import time
from typing import Optional


class MotorController:
    def __init__(self, host: str = "192.168.1.100", port: int = 5000,
                 timeout: float = 1.0):
        self.host    = host
        self.port    = port
        self.timeout = timeout
        self._sock   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(timeout)

    def close(self):
        self._sock.close()

    def _send(self, cmd: str) -> str:
        payload = (cmd.strip() + "\n").encode()
        self._sock.sendto(payload, (self.host, self.port))
        try:
            data, _ = self._sock.recvfrom(1024)
            return data.decode().strip()
        except socket.timeout:
            return "TIMEOUT"

    # ── Commands ──────────────────────────────────────────────────────────

    def enable(self) -> str:
        return self._send("enable")

    def disable(self) -> str:
        return self._send("disable")

    def set_speed(self, rpm: float) -> str:
        return self._send(f"set_speed {rpm:.2f}")

    def set_torque(self, iq_amps: float) -> str:
        return self._send(f"set_torque {iq_amps:.4f}")

    def set_pid_current(self, kp: float, ki: float) -> str:
        return self._send(f"set_pid_current {kp:.6f} {ki:.6f}")

    def set_pid_speed(self, kp: float, ki: float) -> str:
        return self._send(f"set_pid_speed {kp:.6f} {ki:.6f}")

    def calibrate(self) -> str:
        return self._send("calibrate")

    def status(self) -> dict:
        """Return status as a dict. Keys match the STATUS= fields."""
        raw = self._send("status")
        if raw.startswith("ERR") or raw == "TIMEOUT":
            return {"error": raw}
        result = {}
        for token in raw.split():
            if "=" in token:
                k, _, v = token.partition("=")
                try:
                    result[k] = float(v)
                except ValueError:
                    result[k] = v
        return result

    # ── Logging helper ────────────────────────────────────────────────────

    def log_status(self, interval_s: float = 0.5, count: int = 0):
        """
        Print status lines. count=0 → loop forever (Ctrl-C to stop).
        """
        header = (
            f"{'STATE':<10} {'SPEED':>8} {'SPD_REF':>8} "
            f"{'IA':>7} {'IB':>7} {'ID':>7} {'IQ':>7} {'IQ_REF':>7}"
        )
        print(header)
        print("-" * len(header))

        i = 0
        try:
            while count == 0 or i < count:
                s = self.status()
                if "error" in s:
                    print(f"  {s['error']}")
                else:
                    print(
                        f"{s.get('STATE','?'):<10} "
                        f"{s.get('SPEED', 0):>8.1f} "
                        f"{s.get('SPEED_REF', 0):>8.1f} "
                        f"{s.get('IA', 0):>7.3f} "
                        f"{s.get('IB', 0):>7.3f} "
                        f"{s.get('ID', 0):>7.3f} "
                        f"{s.get('IQ', 0):>7.3f} "
                        f"{s.get('IQ_REF', 0):>7.3f}"
                    )
                time.sleep(interval_s)
                i += 1
        except KeyboardInterrupt:
            pass


# ── Interactive menu ───────────────────────────────────────────────────────

MENU = """
╔══════════════════════════════════════╗
║   BLDC FOC Controller — Host Menu   ║
╠══════════════════════════════════════╣
║  1  Enable motor                     ║
║  2  Disable motor                    ║
║  3  Set speed [RPM]                  ║
║  4  Set torque [A Iq]                ║
║  5  Print status                     ║
║  6  Monitor status (Ctrl-C to stop)  ║
║  7  Calibrate current sensors        ║
║  8  Tune current PID                 ║
║  9  Tune speed PID                   ║
║  0  Exit                             ║
╚══════════════════════════════════════╝
"""


def interactive(mc: MotorController):
    while True:
        print(MENU)
        choice = input("Choice: ").strip()

        if choice == "0":
            mc.disable()
            print("Motor disabled. Goodbye.")
            break

        elif choice == "1":
            print(mc.enable())

        elif choice == "2":
            print(mc.disable())

        elif choice == "3":
            try:
                rpm = float(input("  Speed [RPM]: "))
                print(mc.set_speed(rpm))
            except ValueError:
                print("Invalid number.")

        elif choice == "4":
            try:
                iq = float(input("  Iq reference [A]: "))
                print(mc.set_torque(iq))
            except ValueError:
                print("Invalid number.")

        elif choice == "5":
            s = mc.status()
            for k, v in s.items():
                print(f"  {k} = {v}")

        elif choice == "6":
            try:
                interval = float(input("  Interval [s, default 0.5]: ") or "0.5")
            except ValueError:
                interval = 0.5
            mc.log_status(interval_s=interval)

        elif choice == "7":
            confirm = input("  Motor must be stopped. Continue? [y/N] ")
            if confirm.lower() == "y":
                print(mc.calibrate())

        elif choice == "8":
            try:
                kp = float(input("  kp: "))
                ki = float(input("  ki: "))
                print(mc.set_pid_current(kp, ki))
            except ValueError:
                print("Invalid number.")

        elif choice == "9":
            try:
                kp = float(input("  kp: "))
                ki = float(input("  ki: "))
                print(mc.set_pid_speed(kp, ki))
            except ValueError:
                print("Invalid number.")

        else:
            print("Unknown option.")


def main():
    parser = argparse.ArgumentParser(description="BLDC FOC host controller")
    parser.add_argument("--host", default="192.168.1.100",
                        help="Board IP address (default: 192.168.1.100)")
    parser.add_argument("--port", type=int, default=5000,
                        help="UDP port (default: 5000)")
    parser.add_argument("--cmd",  default=None,
                        help="Run a single command and exit")
    args = parser.parse_args()

    mc = MotorController(host=args.host, port=args.port)
    print(f"Connecting to {args.host}:{args.port} ...")

    if args.cmd:
        print(mc._send(args.cmd))
    else:
        interactive(mc)

    mc.close()


if __name__ == "__main__":
    main()
