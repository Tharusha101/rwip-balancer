# CLAUDE.md — rwip-balancer project tracker

Project context and running progress log for the **Reaction-Wheel Inverted
Pendulum (RWIP)** controller project. Loaded into context each session — keep it
current as phases complete. Detailed docs live in `README.md` and
`notes/EOM_derivation.md`; this file is the short, authoritative status + the
conventions that must never drift.

---

## What this project is

A rigid arm on a single pivot bearing with a motor-driven flywheel near the top;
reaction torque from accelerating the wheel balances the arm upright. Controller
designed and validated in **MATLAB/Simulink, sim-first**, then ported to hardware
(ESP32 + MPU6050 + brushed DC motor + DRV8833). Portfolio-quality control-theory
project. Extends a prior CubeSat ADCS detumbling simulator — the **wheel-speed
saturation** problem here is the direct analog of magnetorquer saturation there,
and is the key subtlety vs. a textbook inverted pendulum.

**Tools:** MATLAB + Control System Toolbox (`lqr`, `care`, `ss`, `tf`) from
Phase 2 on; Phase 1 uses base MATLAB (`ode45`) only. Symbolic Math Toolbox
optional. MATLAB is installed and runnable here via `matlab -batch "..."`.

---

## Status

| Phase | Scope | State |
|-------|-------|-------|
| **1** | Nonlinear EOM + verification | ✅ **DONE & VERIFIED** (all 3 checks pass in MATLAB) |
| **2** | Linearize about upright → (A,B,C,D) → LQR balance (with wheel-speed penalty) + motor model | ✅ **DONE & VERIFIED** (all 5 checks pass) |
| **3** | Baseline PID + sensor/actuator realism (noise, 200 Hz discretization, motor lag, PWM quant.) | ✅ **DONE & VERIFIED** (all 3 checks pass) |
| **4** | Animation + impulse "shove" recovery + GIF export | ✅ **DONE** (GIF + montage rendered) |
| **5** | Discretize controller, package gains, document ESP32 mapping | ✅ **DONE** (gains.h + spec, gain re-validated) |

**🎉 All 5 phases complete.** Sim-first design fully verified; hardware handoff packaged.

> **Workflow:** build in verified phases; run the phase's diagnostic and confirm
> PASS before moving on. **Stop after each phase for user review** unless told
> otherwise.

---

## Conventions (DO NOT change without updating notes + code together)

- Planar; positive = **CCW**. SI units throughout.
- `θ` = arm angle from **UPRIGHT**, `θ = 0` is inverted/up (the controlled
  equilibrium → origin of state space). Hanging-down stable eq. is `θ = π`.
- `φ̇` = wheel speed **relative to the arm** (what a motor encoder reads; back-EMF
  tracks this). Absolute wheel rate is `ψ̇ = θ̇ + φ̇` (used in the energy term).
- **State `x = [θ, θ̇, φ̇]`** — wheel angle `φ` omitted (cyclic, no term depends on it).
- Motor torque `τ` acts on the wheel; reaction `−τ` on the arm. `τ` enters the arm
  equation ONLY as `−τ` (the entire control authority).
- `p.I_p` is the **effective** swing inertia `I_arm + m_w·l_w²` (wheel as point
  mass, excludes spin). `p.I_w` is wheel spin inertia, kept separate.
  `p.mgl = m_p·l + m_w·l_w` (gravity torque = `mgl·g·sin θ`).

### Equations of motion (full form with friction in `rwip_dynamics.m` / notes §6)
```
θ̈ = (mgl·g·sin θ − b_θ·θ̇ − τ + b_w·φ̇) / I_p
φ̈ = (τ − b_w·φ̇) / I_w − θ̈
```

---

## File layout

```
balance/
├── CLAUDE.md                   this tracker
├── README.md                   full system description + run instructions
├── notes/
│   ├── EOM_derivation.md       Lagrangian derivation + every sign choice
│   ├── phase2_lqr.md           linearization, LQR weights/tradeoff, motor model
│   ├── phase3_robustness.md    PID vs LQR (honest), sampled-data realism
│   ├── phase4_visualization.md animation + shove, GIF build notes
│   └── phase5_handoff.md       ESP32 implementation spec (the hardware bridge)
├── src/
│   ├── rwip_params.m           physical params → struct (+ derived I_p, mgl)
│   ├── rwip_dynamics.m         nonlinear state derivative, signature (t,x,tau,p)
│   ├── rwip_energy.m           total mechanical energy [E,T,V] (verification tool)
│   ├── rwip_linearize.m        (A,B,C,D[,sys]) about upright; input = tau
│   ├── rwip_motor.m            brushed-DC actuator: [tau,V]=f(tau_cmd,phidot,p)
│   ├── ideal_actuator.m        pass-through torque source (same signature)
│   ├── design_lqr.m            lqr() wrapper → struct {K,S,poles,Acl,ok,warn}
│   ├── simulate_rwip.m         continuous closed-loop harness (Phase 2)
│   ├── ctrl_lqr.m              LQR law in sampled interface [u,s]=f(s,y,Ts,cfg)
│   ├── ctrl_pid.m              PID on theta (stateful integral), same interface
│   ├── sensor_imu.m            noisy IMU+encoder measurement model (MPU6050-like)
│   ├── simulate_sampled.m      sampled-data harness (ZOH, sensor, PWM, motor lag, kick)
│   └── draw_rwip.m             render one pose (arm + wheel) into an axes
├── scripts/
│   ├── verify_dynamics.m       Phase-1 checks, prints PASS/FAIL, saves plots
│   ├── phase2_lqr.m            Phase-2 linearize+LQR+balance, PASS/FAIL, plots
│   ├── phase3_pid_robust.m     Phase-3 PID vs LQR + robustness, PASS/FAIL, plots
│   ├── phase4_animate.m        Phase-4 animation: GIF + snapshot montage
│   └── phase5_export.m         Phase-5 export gains.h/.mat + re-validate
└── results/                    PNGs + phase4_balance.gif + rwip_gains.h/.mat
```

**Interfaces to keep stable** (so new control laws / actuators drop in):
- Plant: `rwip_dynamics(t, x, tau, p)` → bind as `@(t,x) rwip_dynamics(t,x,tau,p)`.
- Continuous harness (Phase 2): controller `tau_cmd = ctrl(t,x)`, actuator
  `[tau,V] = actuator(tau_cmd,phidot,p)`, `simulate_rwip(p,x0,ctrl,actuator,tspan)`.
- Sampled harness (Phase 3+): controller `[u,s] = ctrl(s,y,Ts,cfg)` (stateful),
  sensor `y = sensor(x,p,k,scfg)`, `simulate_sampled(p,x0,sim)`. `sim` fields:
  `.Ts .Tend .controller .cfg .s0 .sensor .scfg .use_motor .use_pwm .pwm_bits
  .use_motor_lag .kick`. `.kick=struct('time',t,'dtheta_dot',v)` injects an impulse.

### Run the verifications
```bash
matlab -batch "run('scripts/verify_dynamics.m')"    # Phase 1
matlab -batch "run('scripts/phase2_lqr.m')"         # Phase 2
matlab -batch "run('scripts/phase3_pid_robust.m')"  # Phase 3
matlab -batch "run('scripts/phase4_animate.m')"     # Phase 4 (GIF + montage)
matlab -batch "run('scripts/phase5_export.m')"      # Phase 5 (gains.h/.mat)
```
Each exits non-zero if any check fails. Plots/GIF/headers → `results/`.

---

## Default parameters (`rwip_params.m`)

> **Updated in Phase 2** so the motor has real control authority (the Phase-1
> placeholder motor could only recover ~2.4°). Phase 1 verify is
> parameter-independent and still passes with these.

| | value | | |
|---|---|---|---|
| arm mass `m_p` | 0.30 kg | wheel mass `m_w` | 0.15 kg |
| pivot→arm COM `l` | 0.10 m | pivot→wheel `l_w` | 0.20 m |
| arm inertia `I_arm` | 3.0e-3 kg·m² | wheel spin `I_w` | 5.0e-4 kg·m² |
| friction `b_θ,b_w` | 0 | motor `Kt,Ke,R,Vmax` | 0.030, 0.030, 1.0 Ω, 12 V |

Derived: `I_p = 9.0e-3`, `mgl = 0.06`. Upright unstable, growth rate
`√(mgl·g/I_p) ≈ 8.09 rad/s` (~0.124 s). Hanging period `≈ 0.777 s`. Motor stall
torque `Kt·Vmax/R = 0.36 N·m`, no-load wheel speed `Vmax/Ke = 400 rad/s`,
static recoverable angle `τ_stall/(mgl·g) ≈ 35°`. Easy to retune.

---

## Phase 1 results (verified in MATLAB)

| Test | Check | Result |
|------|-------|--------|
| 1 | Energy conservation (τ=0, no friction) | rel drift 7.7e-10 (tol 1e-6) ✅ |
| 2 | Free response = pendulum; wheel L conserved | L drift 3.9e-18; θ matches pendulum to 7e-9 rad ✅ |
| 3 | Small-angle period vs. analytic | 0.77766 s vs 0.77754 s, 0.016% err ✅ |

Emergent sanity check captured by Test 2: with τ=0 the free wheel's absolute
momentum is conserved, so the arm swings at a frequency set by `I_p` alone (the
frictionless wheel just counter-rotates underneath). Confirmed exactly.

## Phase 2 results (verified in MATLAB)

Linearization, LQR design, and nonlinear 8° balance — details in
`notes/phase2_lqr.md`.

- **Model:** `A=[0 1 0; 65.4 0 0; -65.4 0 0]`, `B=[0; -111.1; 2111.1]`;
  open-loop poles `{±8.087, 0}`; analytic vs FD Jacobian err `1.1e-11`.
- **LQR:** `Q=diag([200, 5, 2e-4])`, `R=20` → **`K=[-4.4742, -0.6381, -0.00316]`**;
  closed-loop poles `{-56.7, -6.33, -1.15}`. The slow `-1.15` pole is the
  wheel-speed bleed-off; set mainly by `q_phidot`.
- **Key result:** `q_phidot=0` → LQR has *no stabilizing solution* (wheel mode
  undetectable). Wheel speed **must** be penalized — this is the RWIP momentum-
  management subtlety (analog of magnetorquer desaturation).
- **8° balance (realistic motor):** settles 1.23 s, final θ −0.02°, peak wheel
  25.3 rad/s (6% of 400 limit) bleeding to ~0, motor saturates only 16 ms at t=0.
  All 5 checks PASS.

Plots: `results/phase2_lqr_balance.png` (θ, wheel, torque + the with/without
`φ̇`-penalty contrast), `results/phase2_voltage.png`.

## Phase 3 results (verified in MATLAB)

PID baseline + robustness — details in `notes/phase3_robustness.md`.

- **Discrete LQR** (`dlqr` on ZOH plant @200 Hz): **`Kd=[-3.9078, -0.5541, -0.00269]`**
  (~15% below continuous `K`; this is the gain exported to Phase 5).
- **A1 clean 8° release:** honest finding — PID (`Kp=1.2,Kd=0.14,Ki=1.0`) is
  competitive on a single recovery (settle 1.37 s vs LQR 1.20 s; *lower* effort).
  LQR edges it on overshoot (0.77° vs 1.98°) and peak wheel (24.7 vs 35.6).
- **A2 impulse shove (+120°/s):** the real difference. Both keep the arm up, but
  4.5 s later the wheel is **LQR 0.36 vs PID 39.3 rad/s**. PID has *no* `φ̇`
  feedback (wheel mode = unregulated integrator) → momentum accumulates → walks
  to saturation under repeated shoves. LQR bleeds it back to 0. (Money plot:
  `results/phase3a2_disturbance_wheel.png`.)
- **B full realism** (gyro bias 1.5°/s + noise, accel noise 0.3°, motor lag 1 ms,
  10-bit PWM, 200 Hz): LQR stable, **steady RMS θ = 0.066°**, wheel bounded
  (parks −5.3 rad/s holding the gyro bias). De-risks the hardware port.

Plots: `phase3a_lqr_vs_pid.png`, `phase3a2_disturbance_wheel.png`,
`phase3b_robustness.png`.

## Phase 4 results

Visualization centerpiece — details in `notes/phase4_visualization.md`.

- Balance from +10° → settle → **+560°/s shove at t=1.5 s** → recover. Reuses
  `simulate_sampled` with `sim.kick`. Wheel angle for display = `cumtrapz(t,φ̇)`,
  spoke at absolute angle `ψ=θ+φ` (φ not a dynamical state).
- Peak |θ| after shove 7.7°, peak wheel 228 rad/s (57% of 400 limit), bleeds to
  ~1.5 rad/s. Outputs: **`results/phase4_balance.gif`** (151 frames @ 25 fps,
  mechanism + live traces), `results/phase4_snapshots.png` (6-pose montage).
- GIF capture is headless-safe: `exportgraphics`→temp PNG→`imwrite` append
  (not `getframe`).

## Phase 5 results — handoff (project complete)

ESP32 implementation spec + validated gains — details in `notes/phase5_handoff.md`.

- `scripts/phase5_export.m` regenerates the gain from source, **re-validates it**
  on the full realistic chain (RMS θ 0.066°, peak wheel 25.7 rad/s — PASS), and
  writes **`results/rwip_gains.h`** (C header: gains, motor consts, PWM, fusion,
  safety limits) and `results/rwip_gains.mat`.
- `notes/phase5_handoff.md` covers: sign/unit table, signal chain, θ
  complementary filter, gyro de-bias, the `φ̇` wheel-speed sensing decision
  (encoder recommended), DRV8833 sign-magnitude PWM with measured `V_bus`,
  200 Hz timing budget, safety logic (no swing-up; enable within 5°; give up at
  30°; wheel soft limit 350 rad/s), and full control-loop pseudocode.
- Re-tuning path documented: measure real plant/motor params → drop into
  `rwip_params.m` → re-run Phases 2→5 → gains/limits regenerate.

---

## Project status: COMPLETE
All 5 phases done & verified. To extend: write the actual ESP32 firmware against
`rwip_gains.h` + `phase5_handoff.md`, or feed measured hardware params back into
`rwip_params.m` and regenerate. Everything downstream is parameterized off that
one file.

---

## Changelog

- **2026-06-13** — Phase 1 complete: project scaffolding, README, EOM derivation
  notes, `rwip_params/dynamics/energy.m`, `verify_dynamics.m`. All 3 verification
  checks PASS in MATLAB.
- **2026-06-13** — Phase 2 complete: retuned params for control authority;
  `rwip_linearize.m`, `rwip_motor.m`, `ideal_actuator.m`, `design_lqr.m`,
  `simulate_rwip.m`, `scripts/phase2_lqr.m`, `notes/phase2_lqr.md`. LQR
  `K=[-4.4742,-0.6381,-0.00316]`; nonlinear 8° balance with motor model; all 5
  checks PASS. Demonstrated wheel-speed penalty is mandatory.
- **2026-06-13** — Phase 3 complete: sampled-data framework (`ctrl_lqr/ctrl_pid/
  sensor_imu/simulate_sampled.m`), `scripts/phase3_pid_robust.m`,
  `notes/phase3_robustness.md`. Added motor inductance `Lm` to params. Discrete
  LQR `Kd=[-3.9078,-0.5541,-0.00269]`. Honest PID comparison: PID balances but
  can't regulate wheel (impulse test: wheel ends 0.36 vs 39.3 rad/s). LQR robust
  under full realism (RMS θ 0.066°). All 3 checks PASS.
- **2026-06-13** — Phase 4 complete: `src/draw_rwip.m`, `scripts/phase4_animate.m`,
  `notes/phase4_visualization.md`. Balance→+560°/s shove→recovery; rendered
  `phase4_balance.gif` (151 frames) + `phase4_snapshots.png`. Headless GIF via
  exportgraphics-to-temp.
- **2026-06-13** — Phase 5 complete (PROJECT DONE): `scripts/phase5_export.m`,
  `notes/phase5_handoff.md`, `results/rwip_gains.h` + `.mat`. Discrete gain
  re-validated on the realistic chain before export. Full ESP32 spec (fusion,
  signs/units, DRV8833 PWM, safety, pseudocode). All 5 phases verified.
