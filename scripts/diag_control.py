#!/usr/bin/env python3
"""Control quality diagnostic.

Measures actual FOC loop rate, then runs short torque and speed tests
to assess current and speed PI quality.  Motor-on time ≤ 3 s per test.

Usage:
    python3 scripts/diag_control.py [/dev/ttyACM0]
"""
import serial, time, re, sys, math

PORT = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
ANSI = re.compile(r'\x1b\[[0-9;]*[mABCDEFGHJKSTnRhlu?]')
TWO_PI = 2 * math.pi

ser = serial.Serial(PORT, 115200, timeout=0.05)
time.sleep(0.3)
ser.reset_input_buffer()

def xfer(cmd, wait=0.15):
    ser.reset_input_buffer()
    ser.write((cmd + '\r\n').encode())
    t0 = time.monotonic()
    buf = b''
    while time.monotonic() - t0 < wait:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
        else:
            time.sleep(0.005)
    return ANSI.sub('', buf.decode('utf-8', errors='replace'))

def get_st():
    raw = xfer('status', 0.12)
    d = {}
    for m in re.finditer(r'([A-Z_]+)=([-\d.eE+]+)', raw):
        try: d[m.group(1)] = float(m.group(2))
        except: pass
    for k in ('STATE', 'MODE'):
        m = re.search(rf'{k}=(\w+)', raw)
        if m: d[k] = m.group(1)
    return d

# ── 1. Measure actual loop rate ─────────────────────────────────────────────
print("=== LOOP RATE ===")
d1 = get_st()
t1 = time.monotonic()
time.sleep(1.0)
d2 = get_st()
t2 = time.monotonic()
loops1 = d1.get('LOOPS', 0)
loops2 = d2.get('LOOPS', 0)
if loops2 > loops1:
    rate = (loops2 - loops1) / (t2 - t1)
    print(f"  Actual loop rate: {rate:.0f} Hz  (target: 5000 Hz)")
    if rate < 4000:
        print(f"  WARNING: rate is {rate:.0f} Hz — significantly below 5 kHz target")
    elif rate > 6000:
        print(f"  WARNING: rate is {rate:.0f} Hz — above 5 kHz (ADC timing may be too tight)")
    else:
        print(f"  OK: running at ~5 kHz")
else:
    print("  Could not measure (LOOPS not incrementing)")
print()

# ── 2. Calibrate ─────────────────────────────────────────────────────────────
xfer('disable', 0.2)
print("=== CALIBRATE ===")
xfer('calibrate', 1.5)
time.sleep(0.2)
print()

def unwrap(prev, curr):
    d = curr - (prev % TWO_PI)
    if d >  math.pi: d -= TWO_PI
    if d < -math.pi: d += TWO_PI
    return prev + d

def run(label, cmd, n_run=6, timeout=4.5, quiet=False, poll_s=0.10):
    """Enable, send cmd, sample until RUNNING n_run times or timeout."""
    print(f"=== {label} ===")
    xfer('enable', 0.05)
    xfer(cmd, 0.05)

    theta_u = None
    theta_u0 = None
    t0 = time.monotonic()
    running_n = 0
    rows = []
    prev_st = None

    while time.monotonic() - t0 < timeout:
        d = get_st()
        d['t'] = time.monotonic() - t0
        th = d.get('THETA_E', 0.0)
        theta_u = th if theta_u is None else unwrap(theta_u, th)
        d['theta_u'] = theta_u
        st = d.get('STATE', '?')
        if prev_st and prev_st != st and st == 'RUNNING':
            theta_u0 = theta_u
        net_e = theta_u - (theta_u0 if theta_u0 is not None else theta_u)
        if not quiet or st == 'RUNNING':
            tag = f'  <<<{prev_st}→{st}>>>' if prev_st and prev_st != st else ''
            print(f"  t={d['t']:.2f}s {st:10s} "
                  f"spd={d.get('SPEED',0):7.1f} "
                  f"id={d.get('ID',0):+.3f} iq={d.get('IQ',0):+.3f}(ref={d.get('IQ_REF',0):.2f}) "
                  f"vd={d.get('VD',0):+.3f} vq={d.get('VQ',0):+.3f} "
                  f"Δθ={net_e:+.2f}rad{tag}")
            sys.stdout.flush()
        rows.append(d)
        prev_st = st
        if st == 'ERROR':
            print("  *** OC/ERROR ***"); break
        if st == 'RUNNING':
            running_n += 1
            if running_n >= n_run:
                break
        time.sleep(poll_s)

    xfer('disable', 0.4)
    xfer('foc_reset', 0.1)

    running = [r for r in rows if r.get('STATE') == 'RUNNING']
    print()
    if len(running) >= 2:
        iqs  = [r.get('IQ',0)    for r in running]
        vqs  = [r.get('VQ',0)    for r in running]
        spds = [r.get('SPEED',0) for r in running]
        avg_iq    = sum(iqs)/len(iqs)
        avg_vq    = sum(vqs)/len(vqs)
        avg_speed = sum(spds)/len(spds)
        spd_std   = (sum((s-avg_speed)**2 for s in spds)/len(spds))**0.5
        spinning  = abs(avg_speed) > 20.0
        print(f"  RESULT: avg_speed={avg_speed:+.0f} RPM  std={spd_std:.0f}  "
              f"avg_iq={avg_iq:+.3f}  avg_vq={avg_vq:+.3f}  "
              f"{'SPINNING' if spinning else 'STALL'}")
        if avg_vq > 5.0:
            print("  WARNING: vq saturated — current loop not closing")
        elif abs(avg_vq) < 3.0:
            print("  vq within range — current loop looks healthy")
    else:
        print(f"  Not enough RUNNING samples (got {len(running)})")
    print()
    time.sleep(5.0)  # let motor spin down to near-rest before next alignment
    return rows

# ── 3. Torque test — does current loop close? ────────────────────────────────
run("+1A torque",  "set_torque 1",  n_run=6)

# ── 4. Speed tests ───────────────────────────────────────────────────────────
run("+500 RPM speed",  "set_speed  500", n_run=20, poll_s=0.10)
run("+1000 RPM speed", "set_speed 1000", n_run=20, poll_s=0.10)
run("-500 RPM speed",  "set_speed -500", n_run=20, poll_s=0.10)

print("=== DONE ===")
ser.close()
