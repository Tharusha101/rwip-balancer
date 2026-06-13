# Phase 3 — PID baseline + robustness under realism

Design notes for the controller comparison and the hardware-realism stress test.
Pairs with `scripts/phase3_pid_robust.m`, `src/ctrl_lqr.m`, `src/ctrl_pid.m`,
`src/sensor_imu.m`, `src/simulate_sampled.m`.

---

## Sampled-data framework

Everything in Phase 3 runs through `simulate_sampled.m`: a fixed-rate (200 Hz)
controller with a zero-order hold on the actuator command, integrating the
continuous nonlinear plant with `ode45` across each 5 ms interval. Controllers
and the sensor are **function handles**, so LQR/PID and clean/noisy sensing swap
in without touching the harness:

- controller: `[u, s] = ctrl(s, y, Ts, cfg)` — `s` carries state (PID integral).
- sensor: `y = sensor(x, p, k, scfg)` — `[]` for perfect measurement.
- actuator: ideal torque, or brushed-DC motor with back-EMF feed-forward,
  supply saturation, optional **PWM quantisation** and **electrical lag**
  (motor current state, inductance `Lm`).

This is the realistic sampled-data structure that the Phase-5 ESP32 firmware
will mirror.

---

## Discrete LQR

ZOH-discretise the plant (`c2d` at 200 Hz) and redesign with `dlqr` using the
Phase-2 weights `Q = diag([200, 5, 2e-4])`, `R = 20`:

```
continuous   K  = [-4.4742, -0.6381, -0.00316]
discrete    Kd  = [-3.9078, -0.5541, -0.00269]   <-- used in Phase 3 and exported to Phase 5
```

The ZOH discretisation lowers the gain ~15% at 200 Hz; both are stable. `Kd` is
the gain that goes to the hardware.

---

## Part A — PID baseline vs LQR (the honest result)

PID acts on arm angle only: `u = Kp·θ + Kd·θ̇ + Ki·∫θ` with
`Kp=1.2, Kd=0.14, Ki=1.0`. The derivative uses the measured rate (gyro).
**Structurally, PID has no `φ̇` term — it cannot see or regulate wheel speed.**

### A1 — clean 8° release (200 Hz, ideal sensors/actuator)

| metric | LQR | PID |
|--------|-----|-----|
| settling time [s] | 1.20 | 1.37 |
| overshoot [deg] | 0.77 | 1.98 |
| control effort ∫τ² | 0.0037 | 0.0021 |
| peak wheel [rad/s] | 24.7 | 35.6 |

On a *single* clean recovery, PID is competitive — actually cheaper on effort —
because the arm starts and ends at rest, so neither controller leaves much wheel
speed. The naive "LQR wins everything" story is **not** what happens, and the
script reports it honestly. LQR's edge here is lower overshoot and gentler wheel
use; the decisive difference is in A2.

### A2 — impulse "shove" (+120°/s at t=1.5 s): wheel-momentum management

Both controllers keep the arm balanced through the shove. The wheel tells the
real story (`results/phase3a2_disturbance_wheel.png`):

| 4.5 s after the shove | LQR | PID |
|------------------------|-----|-----|
| wheel speed [rad/s] | **0.36** | **39.3** |

PID's wheel-speed mode is an **unregulated integrator** (closed-loop pole exactly
at the origin): momentum dumped into the wheel by a disturbance stays there
forever. Under a sequence of shoves it random-walks to the 400 rad/s ceiling and
the system falls. LQR's `φ̇` feedback actively **bleeds the momentum back off**,
returning the wheel toward zero so it can absorb disturbances indefinitely. This
is the reaction-wheel saturation problem — the same momentum-management issue as
magnetorquer desaturation in the CubeSat ADCS project — and it is the real reason
to prefer LQR here.

---

## Part B — robustness under full hardware realism

The **same discrete LQR** (`Kd`) driven through the realistic chain:

- **Sensors (MPU6050-class):** accel/fused tilt noise `σθ = 0.30°`, gyro white
  noise `0.25°/s`, gyro **constant bias `1.50°/s`**, wheel-speed noise `0.5 rad/s`.
- **Actuation (ESP32 + DRV8833):** 200 Hz loop, motor electrical lag
  (`Lm/R = 1 ms`), **10-bit PWM** voltage quantisation (0.023 V step), ±12 V supply.

Result (`results/phase3b_robustness.png`):

| metric | value |
|--------|-------|
| peak \|θ\| | 8.0° (the initial condition) |
| steady-state RMS θ (last 1 s) | **0.066°** |
| peak wheel | 25.7 rad/s (limit 400) |
| final wheel | −5.3 rad/s |

LQR balances tightly despite the noisy measurement, with only a brief PWM
saturation at `t = 0`. The ~5 rad/s steady wheel offset is the controller holding
the arm against the **constant gyro bias** (a constant fake rate → constant
corrective torque → a parked wheel speed), and it is bounded — the `φ̇` penalty
keeps it from running away. **The LQR survives the realism that will be present
on hardware**, which is what de-risks the Phase 5 port.

> Tuning knobs if hardware behaves worse than sim: raise the loop rate, add a
> complementary/Kalman filter for `θ` (reduce `σθ`), or estimate & subtract the
> gyro bias at startup.

---

## Forward to Phase 4
Animate the arm + spinning wheel, inject the impulse shove (the A2 scenario is
ready), and export a GIF — the portfolio centerpiece.
