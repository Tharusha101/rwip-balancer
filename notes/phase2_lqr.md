# Phase 2 — Linearization + LQR balancing

Design notes for the upright-balancing controller. Pairs with
`scripts/phase2_lqr.m`, `src/rwip_linearize.m`, `src/design_lqr.m`,
`src/rwip_motor.m`, `src/simulate_rwip.m`.

---

## 1. Linearization about upright

Linearize the nonlinear EOM (`notes/EOM_derivation.md` §6) about the inverted
equilibrium `x = [θ, θ̇, φ̇] = 0`, `τ = 0`, using `sin θ → θ`. Input is the wheel
torque `τ`; `k ≡ mgl·g`.

```
       ⎡ 0        1        0      ⎤        ⎡ 0          ⎤
  A =  ⎢ k/I_p   -b_θ/I_p  b_w/I_p⎥   B =  ⎢ -1/I_p     ⎥
       ⎣-k/I_p    b_θ/I_p -b_w(1/I_w+1/I_p)⎦  ⎣ 1/I_w+1/I_p⎦
```

With the project parameters (`b_θ = b_w = 0`, `k = mgl·g = 0.06·9.81 = 0.5886`,
`I_p = 9.0e-3`, `I_w = 5.0e-4`):

```
A = [ 0      1   0 ;        B = [   0     ;
      65.4   0   0 ;             -111.11  ;
     -65.4   0   0 ]             2111.11  ]
```

**Cross-check:** the analytic `A,B` match a central-difference Jacobian of
`rwip_dynamics` to `1.1e-11` (script test 1) — the linearization is correct.

### Open-loop structure
Poles of `A`: `{ +8.087, −8.087, 0 } 1/s`.
- `+8.087 = √(k/I_p)` — the unstable falling mode (≈124 ms to grow by *e*).
- `−8.087` — its stable counterpart.
- `0` — the **free wheel**: with no torque the wheel speed is a pure integrator
  (marginally stable). This zero is the crux of the wheel-saturation problem.

`(A,B)` is controllable (rank 3), so all three modes can be placed.

---

## 2. Why wheel speed *must* be in the cost

The `φ̇` mode (open-loop pole at the origin) has eigenvector `[0,0,1]ᵀ`. If the
LQR state cost penalizes only `θ` and `θ̇` (i.e. `Q = diag(qθ, qθ̇, 0)`), that
mode is **unobservable through the cost** and sits exactly on the imaginary axis
→ `(A, Q^{1/2})` is **not detectable** → no stabilizing Riccati solution exists.
In the script, `q_phidot = 0` makes `lqr` fail outright with
*"Cannot compute the stabilizing Riccati solution."*

This is the formal version of the RWIP intuition: **you cannot just balance the
arm — you must also regulate the wheel**, or its momentum is left to drift and it
eventually saturates (losing all control authority). It is the direct analog of
momentum management / magnetorquer desaturation in the CubeSat ADCS project.

A *tiny* penalty (`q_phidot = 1e-9`) is technically stabilizing but parks the
wheel-regulation pole at `−0.003 1/s` (τ ≈ 386 s): over a 4 s window the wheel
speed parks at a constant offset (~29 rad/s for the 8° case) instead of bleeding
off — see the orange curve in `results/phase2_lqr_balance.png`.

---

## 3. LQR weights and the tradeoff

State `x = [θ, θ̇, φ̇]`, control `u = τ`, cost `J = ∫ (xᵀQx + R u²) dt`,
feedback `τ = −Kx`.

```
Q = diag([ 200 ,  5 ,  2e-4 ])      R = 20
```

Starting point was Bryson's rule (`q_i = 1/x_i,max²`, `R = 1/τ_max²`) then a
short tune. Resulting gain and closed loop:

```
K = [ -4.4742 ,  -0.6381 ,  -0.00316 ]
closed-loop poles:  −56.7 ,  −6.33 ,  −1.15   1/s
```

The three closed-loop poles map onto three jobs:
- `−56.7` fast pole — the wheel reacting (large `B` entry `1/I_w+1/I_p`).
- `−6.33` — arm balancing (catch and return `θ`).
- `−1.15` — **wheel-speed regulation**: the slow bleed-off of stored momentum.
  Its location is set almost entirely by `q_phidot`.

**The tuning tradeoff (three-way):**

| Push this | Effect | Cost |
|-----------|--------|------|
| ↑ `q_theta` / ↓ `R` | faster, tighter balancing | larger torque & peak wheel speed → nearer saturation |
| ↑ `q_phidot` | wheel bled off faster, lower peak wheel speed | controller "reluctant" to spin the wheel → slower arm settling |
| ↑ `R` | gentler on the motor (less voltage/torque) | sluggish balancing, bigger excursions |

So it is genuinely three-way: **aggressive balancing ↔ control effort ↔ wheel
saturation**. The chosen weights keep the 8° recovery fast (~1.2 s) while holding
peak wheel speed to ~25 rad/s — 6% of the 400 rad/s ceiling — with plenty of
margin to spare.

---

## 4. Actuator: brushed-DC motor model

The LQR is designed on the torque-input model; the realistic actuator
(`rwip_motor.m`) then realizes `τ_cmd`:

```
V_req = (R/Kt)·τ_cmd + Ke·φ̇         (volts needed, incl. back-EMF)
V     = sat(V_req, ±V_max)           (supply saturation)
τ     = (Kt/R)·(V − Ke·φ̇)            (torque actually delivered)
```

Captured effects: torque headroom shrinks as the wheel speeds up (back-EMF), and
hard voltage saturation. Controller and actuator are passed to
`simulate_rwip.m` as **function handles**, so PID (Phase 3) or a different
actuator drops in without touching the harness.

---

## 5. Result — nonlinear balance from 8°

Closed-loop nonlinear simulation (`results/phase2_lqr_balance.png`,
`results/phase2_voltage.png`):

| Metric | Value |
|--------|-------|
| settling time (\|θ\| < 0.5°) | **1.23 s** |
| final θ | −0.02° |
| peak wheel speed | 25.3 rad/s (241 rpm) — limit 400 |
| final wheel speed | 0.36 rad/s → bleeding to 0 |
| peak voltage *demand* | 20.8 V, capped to 12 V |
| motor saturated | 16 ms (initial transient only) |

The controller briefly saturates the motor at `t = 0` (commands 0.62 N·m, the
12 V supply delivers 0.36 N·m) and still recovers — the realistic-motor and
ideal-torque trajectories are nearly indistinguishable. **All Phase-2 checks
PASS.** The money plot is the middle panel: with the `φ̇` penalty the wheel
returns to zero; without it the wheel parks at an offset and would creep to
saturation under repeated disturbances.

---

## 6. Forward to Phase 3
- Baseline PID on `θ` for comparison (settling, overshoot, effort, peak wheel).
- Sensor/actuator realism: gyro noise+bias, accel noise, 200 Hz discretization,
  motor lag, PWM quantization — confirm this LQR still stabilizes.
