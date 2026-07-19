# BLDC FOC Controller — STM32F769I-DISCO

Field Oriented Control of a BLDC motor using Zephyr RTOS on the
STM32F769I-DISCO evaluation board.

---

## Hardware connections

| Signal          | MCU Pin | Peripheral    | Notes                          |
|-----------------|---------|---------------|--------------------------------|
| Phase A current | PA6     | ADC1 CH6      | INA240 inline current sense    |
| Phase B current | PC2     | ADC1 CH12     | INA240 inline current sense    |
| Phase A PWM     | PC8     | TIM3 CH3      |                                |
| Phase B PWM     | PH6     | TIM12 CH1     |                                |
| Phase C PWM     | PF7     | TIM11 CH1     |                                |
| Encoder A       | PC6     | TIM8 CH1      | Hardware QDEC, pull-up enabled |
| Encoder B       | PC7     | TIM8 CH2      | Hardware QDEC, pull-up enabled |
| Driver enable   | PJ4     | GPIO OUT      | Active-high                    |

Phase C current is reconstructed as `ic = -(ia + ib)`.

### Current sensing constants

Edit `src/motor/motor.h` to match your motor driver shield:

```c
#define MOTOR_SHUNT_OHM   0.005f   /* shunt resistor [Ω]           */
#define MOTOR_AMP_GAIN    50.0f    /* INA240 gain (20/50/100/200)  */
#define MOTOR_ENCODER_CPR 1024     /* encoder lines/rev            */
#define MOTOR_VBUS_V      24.0f    /* DC bus voltage [V]           */
```

Scale formula: `1 LSB = Vref / (4096 × shunt_ohm × amp_gain)` [A/LSB]

---

## Software architecture

```
src/
  main.c              Entry point, FOC and UDP threads
  foc/
    foc.h / foc.c     FOC algorithm (Clarke, Park, SVPWM, PI loops)
    pid.h / pid.c     Generic PI controller with anti-windup
  motor/
    motor.h / motor.c Hardware abstraction (ADC, PWM, GPIO, QDEC encoder)
  interface/
    cmd.h / cmd.c     Shell commands + UDP server (port 5000)

scripts/
  motor.py            Python host controller (subcommand interface)
```

### Control loop

```
ADC (ia, ib)  ──┐
                 ├──► Clarke ──► Park ──► PI_id ──┐
Encoder (θ, ω) ─┤                                  ├──► Inv-Park ──► SVPWM ──► PWM
                 │              Speed PI ──► PI_iq ──┘
Speed ref ──────┘
```

- **Rate:** 10 kHz (`FOC_CONTROL_HZ` in `foc.h`)
- **PWM switching:** 20 kHz, centre-aligned
- **Rotor alignment:** open-loop d-axis current injection for 500 ms on `enable`

### Interfaces

| Interface     | Transport             | Details                               |
|---------------|-----------------------|---------------------------------------|
| Shell         | ST-Link VCP (USART1)  | 115200 baud via ST-Link USB (CN1)     |
| Python script | UDP port 5000         | `scripts/motor.py`                    |

Both share the same text protocol (one command per line).

---

## Getting started

### 1 — Install Zephyr SDK and west

```bash
pip install west
west init zephyrproject
cd zephyrproject
west update
west zephyr-export
pip install -r zephyr/scripts/requirements.txt
```

Download and install the Zephyr SDK (ARM toolchain):
<https://docs.zephyrproject.org/latest/develop/getting_started/index.html>

### 2 — Build

```bash
source venv/bin/activate
cd zephyrproject
west build -b stm32f769i_disco /home/flo/projects/stm32f769i-disco \
    --build-dir /home/flo/projects/stm32f769i-disco/build
```

### 3 — Flash

```bash
west flash --build-dir /home/flo/projects/stm32f769i-disco/build
```

### 4 — Connect via shell

Connect the **ST-Link USB cable** (CN1 — the mini-USB connector next to the
reset button). The ST-Link enumerates a virtual COM port.

```bash
screen /dev/ttyACM0 115200
# or
picocom -b 115200 /dev/ttyACM0
```

Available shell commands:

```
enable                          Enable motor (runs alignment first)
disable                         Disable motor (coast)
set_speed <rpm>                 Speed mode, set reference RPM
set_torque <amps>               Torque mode, set Iq reference [A]
set_vmax <V>                    Clamp output voltage (0 = full Vbus/√3)
set_pid_current <kp> <ki>       Tune d/q current PI gains
set_pid_speed   <kp> <ki>       Tune speed PI gains
set_motor_params <L_mH> <psi>   Update feedforward model
status                          Print full status line
calibrate                       Zero current sensors (motor must be stopped)
mosfet_test                     Test all 6 half-bridge MOSFETs
force_duty <da> <db> <dc>       Open-loop PWM (0.0–1.0, 0.5 = zero V)
reboot                          Cold reboot the MCU
```

### 5 — Connect via Ethernet (Python)

The board has static IP `192.168.7.100`. Connect your host to the same subnet.

```bash
python3 scripts/motor.py status
python3 scripts/motor.py enable
python3 scripts/motor.py speed 500
python3 scripts/motor.py torque 0.5
python3 scripts/motor.py vmax 1.0
python3 scripts/motor.py monitor          # continuous readout
python3 scripts/motor.py plot             # live matplotlib plot
python3 scripts/motor.py mosfet_test      # returns pass/fail summary
python3 scripts/motor.py --host 192.168.1.50 status   # custom IP
```

---

## Commissioning procedure

### 1 — Test the inverter hardware

Before attempting to spin the motor, verify all six MOSFETs:

```
calibrate
set_vmax 0.3
mosfet_test
```

All six vectors should PASS. A near-zero current on both vectors that share a
high-side switch means that MOSFET or its gate driver is faulty.

### 2 — Calibrate current sensors

With the motor disabled and coasting (no PWM drive):

```
calibrate
```

### 3 — Limit output voltage for first spin

Set a conservative voltage limit to protect the INA240 measurement range and
reduce peak current on a low-resistance motor:

```
set_vmax 1.0
```

Increase once the motor is confirmed spinning cleanly.

### 4 — Enable and align

```
enable
```

The board applies a small d-axis current for 500 ms to seat the rotor at
electrical angle 0, then enters closed-loop FOC.

### 5 — Spin slowly and confirm direction

```
set_speed 100
status
```

Check that `SPEED` tracks `SPEED_REF` and that `IA`/`IB` are within the
INA240's ±6.6 A range. If the motor spins the wrong direction, swap any two
motor phase wires.

### 6 — Tune current PI

Start with lower `kp` (0.1–0.5) and adjust `ki` (10–100).

```
set_pid_current 0.3 30
```

### 7 — Tune speed PI

After the current loop is stable:

```
set_pid_speed 0.05 1.0
```

---

## PID tuning reference

| Loop    | Starting point    | kp too high             |
|---------|-------------------|-------------------------|
| Current | kp=0.5, ki=50     | Current oscillations    |
| Speed   | kp=0.05, ki=1.0   | Speed overshoot/ringing |

Id reference is held at 0 (non-salient SPMSM). For IPMSM with reluctance
torque, set `id_ref` according to the motor's Ld/Lq ratio (MTPA).

---

## Adapting the device-tree overlay

The overlay in `boards/stm32f769i_disco.overlay` uses pinctrl node names
from Zephyr's STM32F769IG pinctrl DTSI. If you get build errors about
missing pinctrl nodes, verify the names in:

```
$ZEPHYR_BASE/dts/arm/st/f7/stm32f769ig-pinctrl.dtsi
```

---

## Licence

MIT
