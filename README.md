# BLDC FOC Controller — STM32F769I-DISCO

Field Oriented Control of a BLDC motor using Zephyr RTOS on the
STM32F769I-DISCO evaluation board.

---

## Hardware connections

| Signal           | MCU Pin | Function                          |
|------------------|---------|-----------------------------------|
| Phase A current  | PA6     | ADC1 CH6 (current sense shunt)    |
| Phase B current  | PC2     | ADC1 CH12 (current sense shunt)   |
| Phase A PWM      | PC8     | TIM3 CH3                          |
| Phase B PWM      | PH6     | TIM12 CH1                         |
| Phase C PWM      | PF7     | TIM11 CH1                         |
| Encoder A        | PF6     | GPIO EXTI (quadrature)            |
| Encoder B        | PJ1     | GPIO EXTI (quadrature)            |
| Encoder Index    | PJ0     | GPIO EXTI (rising edge)           |
| Driver enable    | PJ14    | GPIO OUT — high to enable         |

Phase C current is reconstructed as `ic = -(ia + ib)`.

### Current sensing assumptions

Edit `src/motor/motor.h` to match your shield:

```c
#define MOTOR_SHUNT_OHM   0.01f   /* shunt resistor [Ω]  */
#define MOTOR_AMP_GAIN    20.0f   /* sense amplifier gain */
#define MOTOR_ENCODER_CPR 1024    /* encoder lines/rev    */
#define MOTOR_VBUS_V      24.0f   /* DC bus voltage [V]   */
```

---

## Software architecture

```
src/
  main.c              Entry point, thread creation
  foc/
    foc.h / foc.c     FOC algorithm (Clarke, Park, SVPWM, PID loops)
    pid.h / pid.c     Generic PI controller with anti-windup
  motor/
    motor.h / motor.c Hardware abstraction (ADC, PWM, GPIO, encoder)
  interface/
    cmd.h / cmd.c     Shell commands + UDP server (port 5000)

scripts/
  motor_control.py    Python host controller
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
- **PWM switching:** 20 kHz, edge-aligned
- **Rotor alignment:** open-loop d-axis current injection for 500 ms on `enable`

### Interfaces

| Interface     | Transport      | Details                                       |
|---------------|----------------|-----------------------------------------------|
| Shell         | USB CDC-ACM    | Connect with any serial terminal at 115200 baud|
| Python script | UDP port 5000  | `scripts/motor_control.py`                    |

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

Download and install the Zephyr SDK (arm toolchain):
<https://docs.zephyrproject.org/latest/develop/getting_started/index.html>

### 2 — Build

```bash
cd zephyrproject
west build -b stm32f769i_disco /home/flo/projects/stm32f769i-disco \
    --build-dir /home/flo/projects/stm32f769i-disco/build
```

### 3 — Flash

```bash
west flash --build-dir /home/flo/projects/stm32f769i-disco/build
```

Or use OpenOCD / STM32CubeProgrammer with the generated
`build/zephyr/zephyr.elf`.

### 4 — Connect via USB serial

Connect a USB cable to the **USB OTG** connector (CN13) — not the ST-Link.
The board will enumerate as a CDC-ACM serial port.

```bash
screen /dev/ttyACM0 115200
# or
picocom -b 115200 /dev/ttyACM0
```

Available shell commands:

```
enable                     — Enable motor (runs alignment first)
disable                    — Disable motor (coast)
set_speed <rpm>            — Speed mode, set reference RPM
set_torque <amps>          — Torque mode, set Iq reference [A]
status                     — Print full status line
calibrate                  — Zero current sensors (motor must be stopped)
set_pid_current <kp> <ki>  — Tune d/q current PI gains
set_pid_speed   <kp> <ki>  — Tune speed PI gains
```

### 5 — Connect via Ethernet (Python)

The board gets static IP `192.168.7.100`. Connect your host to the same subnet.

```bash
# Interactive menu
python3 scripts/motor_control.py

# Single command
python3 scripts/motor_control.py --cmd "set_speed 500"

# Different board IP
python3 scripts/motor_control.py --host 192.168.1.50
```

---

## Commissioning procedure

1. **Calibrate current sensors** (motor unpowered, coasting):
   ```
   calibrate
   ```

2. **Enable and align**:
   ```
   enable
   ```
   The board applies a small d-axis current for 500 ms to seat the rotor
   at electrical angle 0, then enters closed-loop FOC.

3. **Spin slowly** and confirm direction:
   ```
   set_speed 100
   status
   ```

4. **Tune current PI** (if oscillating or sluggish):
   Start with lower `kp` (0.1–0.5) and adjust `ki` (10–100).
   ```
   set_pid_current 0.3 30
   ```

5. **Tune speed PI** after current loop is stable:
   ```
   set_pid_speed 0.05 1.0
   ```

---

## Adapting the device-tree overlay

The overlay in `boards/stm32f769i_disco.overlay` uses pinctrl node names
from Zephyr's STM32F769IG pinctrl DTSI.  If you get build errors about
missing pinctrl nodes, verify the names in:

```
$ZEPHYR_BASE/dts/arm/st/f7/stm32f769ig-pinctrl.dtsi
```

Common renames between Zephyr versions:
- `tim3_ch3_pc8` → might appear as `tim3_ch3n_pc8` for complementary output
- `adc1_in6_pa6` → ADC pins are always in ANALOG mode; the name encodes the channel

---

## PID tuning notes

| Loop    | Typical starting point                   | Effect of kp too high   |
|---------|------------------------------------------|-------------------------|
| Current | kp=0.5, ki=50 (depends on motor Ld/Lq)  | Current oscillations    |
| Speed   | kp=0.05, ki=1.0                          | Speed overshoot/ringing |

Id reference is kept at 0 for non-salient motors (SPMSM).
For IPMSM with reluctance torque, implement MTPA by setting `id_ref` based
on the motor's Ld/Lq ratio and desired torque.

---

## Licence

MIT
# stm32f769i-disco
