# RWIP — Equations of Motion (Lagrangian derivation)

Reaction-Wheel Inverted Pendulum: a rigid arm on a single pivot bearing with a
motor-driven flywheel near the top. Accelerating the wheel produces a reaction
torque on the arm that is used to balance it upright.

This note derives the nonlinear equations of motion from the Lagrangian and
fixes **every sign and convention** used in the code. Read it alongside
`src/rwip_dynamics.m`, `src/rwip_energy.m`, and `src/rwip_params.m`.

---

## 1. Coordinates and sign conventions

Planar problem; everything rotates about a single horizontal axis (out of the
page). Positive = **counter-clockwise (CCW)**.

| Symbol      | Meaning                                                            | Units |
|-------------|-------------------------------------------------------------------|-------|
| `θ`         | Arm angle measured **from upright**, +CCW. `θ = 0` is inverted/up. | rad   |
| `θ̇`         | Arm angular rate                                                  | rad/s |
| `φ`         | Wheel angle **relative to the arm**, +CCW                         | rad   |
| `φ̇`         | Wheel speed **relative to the arm**                               | rad/s |
| `ψ = θ + φ` | Wheel angle in the **inertial** frame (absolute)                  | rad   |
| `ψ̇ = θ̇ + φ̇`| Absolute wheel rate                                              | rad/s |
| `τ`         | Motor torque applied **to the wheel**; reaction `−τ` on the arm   | N·m   |

Why `θ = 0` at the **top**: it puts the controlled equilibrium at the origin of
the state space, which is what linearization and LQR expect. The stable
hanging-down equilibrium is then at `θ = π`.

Why `φ` is **relative to the arm** (not inertial): it is what the hardware
measures — a motor encoder/tachometer reads the rotor speed relative to its
housing, which is bolted to the arm — and the motor back-EMF is proportional to
that same relative speed. Using `φ̇` directly in the state therefore makes the
Phase-2 motor model and the Phase-5 hardware mapping clean.

### State vector
```
x = [θ, θ̇, φ̇]
```
The wheel angle `φ` is **deliberately omitted**: it appears nowhere in the
dynamics (no term depends on `φ`, only on `φ̇`), so it is a cyclic coordinate.
Integrating it would only add a redundant, drift-prone state.

---

## 2. Parameters (and the "effective" lumped coefficients)

Physical parameters (see `rwip_params.m`):

- Arm: mass `m_p`, pivot→COM distance `l`, arm-only inertia about the pivot `I_arm`.
- Wheel: mass `m_w`, pivot→centre distance `l_w`, spin inertia `I_w`.
- `g` gravitational acceleration.

The EOM only ever need two **lumped** coefficients:

- **Effective swing inertia about the pivot** (wheel treated as a point mass at
  `l_w`, *excluding* its spin):
  ```
  I_p = I_arm + m_w · l_w²
  ```
- **Gravitational moment coefficient** (combined first moment of mass about the
  pivot):
  ```
  mgl = m_p · l + m_w · l_w
  ```
  so the gravity torque about the pivot is `mgl · g · sin θ`.

> Naming note: in the code `p.I_p` is this **effective** inertia
> `I_arm + m_w·l_w²`, *not* the arm-only `I_arm`. The wheel's spin inertia `I_w`
> is kept separate because it enters the dynamics differently (see below).

---

## 3. Kinetic and potential energy

**Kinetic energy.** The arm rotates about the fixed pivot at `θ̇`. The wheel's
centre rides with the arm at radius `l_w` (translational KE `½ m_w l_w² θ̇²`,
already folded into `I_p`), and the wheel spins at its absolute rate `ψ̇ = θ̇ + φ̇`:

```
T = ½ I_p θ̇²  +  ½ I_w (θ̇ + φ̇)²
```

The first term is the whole structure swinging about the pivot with the wheel
"frozen"; the second is the wheel's spin KE about its own axis, using the
**absolute** wheel rate.

**Potential energy.** Heights of the two centres of mass above the pivot are
`l cos θ` and `l_w cos θ`. With the reference `V = 0` at the horizontal:

```
V = (m_p l + m_w l_w) g cos θ = mgl · g · cos θ
```

`V` is maximal at `θ = 0` (upright → unstable) and minimal at `θ = π`
(hanging → stable), as required.

**Lagrangian.**
```
L = T − V = ½ I_p θ̇² + ½ I_w (θ̇ + φ̇)² − mgl g cos θ
```

---

## 4. Generalized forces (motor torque + friction)

Use virtual work to map the motor torque to the generalized coordinates `(θ, φ)`.
The motor applies `+τ` to the wheel (absolute angle `ψ`) and `−τ` to the arm
(angle `θ`):
```
δW = τ δψ − τ δθ = τ (δθ + δφ) − τ δθ = τ δφ
```
So the motor torque is conjugate to the **relative** angle `φ` only:
```
Q_θ(motor) = 0 ,   Q_φ(motor) = τ
```
This is the formal statement of the reaction-wheel principle: the motor cannot
directly torque the arm about the pivot — it acts on the arm **only** through the
wheel's reaction.

Viscous friction (linear, opposing relative motion):

- Pivot/bearing friction on the arm, conjugate to `θ`:  `Q_θ(fric) = −b_θ θ̇`
- Wheel-bearing friction, conjugate to the relative spin `φ`:  `Q_φ(fric) = −b_w φ̇`

Totals:
```
Q_θ = −b_θ θ̇
Q_φ = τ − b_w φ̇
```

---

## 5. Euler–Lagrange equations

`d/dt(∂L/∂q̇) − ∂L/∂q = Q_q`.

**θ equation:**
```
∂L/∂θ̇ = I_p θ̇ + I_w (θ̇ + φ̇)
∂L/∂θ  = mgl g sin θ
⇒ (I_p + I_w) θ̈ + I_w φ̈ − mgl g sin θ = −b_θ θ̇        ...(1)
```

**φ equation:**
```
∂L/∂φ̇ = I_w (θ̇ + φ̇)
∂L/∂φ  = 0
⇒ I_w (θ̈ + φ̈) = τ − b_w φ̇                              ...(2)
```

---

## 6. Solved (explicit) accelerations

Subtract (2) from (1). The `I_w φ̈` and `I_w θ̈` cross-terms cancel cleanly:
```
I_p θ̈ = mgl g sin θ − b_θ θ̇ − τ + b_w φ̇
```
giving the two state derivatives implemented in `rwip_dynamics.m`:

```
            mgl·g·sin θ − b_θ θ̇ − τ + b_w φ̇
  θ̈  =  ───────────────────────────────────────         ... (I)
                        I_p

         τ − b_w φ̇
  φ̈  =  ───────────  −  θ̈                                ... (II)
            I_w
```

### Reading the physics off equation (I)
- `mgl g sin θ` — gravity; for small `θ > 0` it is `+`, i.e. it pushes the arm
  *away* from upright. Upright is unstable, as it must be.
- `−τ` — the **reaction torque**. Driving the wheel `+CCW` (`τ > 0`) accelerates
  the arm `−CW`. This is the control authority. A positive `θ` error is corrected
  with a positive `τ`.
- `+b_w φ̇` — a spinning wheel dragging on its bearing reacts a small torque back
  onto the arm.

### Free-wheel decoupling (used by the Phase-1 checks)
Set `τ = 0`, `b_θ = b_w = 0`. Then from (2), `I_w(θ̈ + φ̈) = 0 ⇒ ψ̈ = 0`: the
wheel's **absolute** rate `ψ̇` is constant and its spin angular momentum
`L_w = I_w ψ̇` is conserved. Equation (I) collapses to the bare pendulum
```
θ̈ = (mgl g / I_p) sin θ ,
```
which depends on `I_p` **only** — a frictionless free wheel does not change the
swing frequency, it just counter-rotates underneath the arm. These two facts are
exactly what `verify_dynamics.m` Test 2 checks.

---

## 7. Energy (verification instrument)

```
E = T + V = ½ I_p θ̇² + ½ I_w (θ̇ + φ̇)² + mgl g cos θ
```
With `τ = 0` and zero friction there are no non-conservative generalized forces,
so `dE/dt = 0`. `rwip_energy.m` evaluates this; Test 1 confirms numerical
conservation. (`dE/dt = Q_θ θ̇ + Q_φ φ̇ = τ φ̇ − b_θ θ̇² − b_w φ̇²`, so any drift in
the frictionless, torque-free case is pure integration error.)

---

## 8. Small-oscillation period (analytic check)

About the hanging equilibrium `θ = π`, let `θ = π + α`. Then `sin θ = −sin α ≈ −α`
and (I) with no input/friction gives `α̈ = −(mgl g / I_p) α`: SHM with
```
ω = √(mgl g / I_p) ,     T = 2π √(I_p / (mgl g)) .
```
Test 3 measures the period of small free oscillations and compares to this.

(For completeness, linearizing about **upright** `θ = 0` gives `θ̈ = +(mgl g/I_p) θ`,
an unstable mode with growth rate `√(mgl g / I_p)` — the quantity the balancing
controller must overcome in Phase 2.)

---

## 9. Motor model (forward reference, Phase 2+)

Not used in Phase 1, but fixed here for consistency. Brushed DC motor with the
wheel on its shaft; back-EMF tracks the **relative** wheel rate `φ̇`:
```
τ = Kt · i ,     V = i R + Ke · φ̇
⇒ τ = (Kt / R) (V − Ke φ̇)
```
This gives the realistic effects we care about: torque falls off as the wheel
speeds up (back-EMF), and there are voltage/torque saturation limits plus a
finite wheel-speed ceiling — the saturation problem that the LQR wheel-speed
penalty is designed to manage (the direct analog of magnetorquer saturation in
the CubeSat ADCS detumbling project this extends).
