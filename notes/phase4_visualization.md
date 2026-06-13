# Phase 4 — Visualization + disturbance (the centerpiece)

Pairs with `scripts/phase4_animate.m` and `src/draw_rwip.m`.

## What it shows
The RWIP balances from a +10° tilt under the Phase-3 discrete LQR (`Kd`), settles
upright, then takes a **+560 °/s impulse "shove"** at t = 1.5 s and recovers. The
reaction wheel spins up to ~228 rad/s to catch the disturbance, then the `φ̇`
penalty bleeds that momentum back off toward zero — the visual payoff of the
whole project.

Outputs (in `results/`):
- **`phase4_balance.gif`** — animated mechanism (arm + spinning wheel with a
  spoke/rim marker) beside live `θ(t)` and wheel-speed traces, 151 frames @ 25 fps.
- **`phase4_snapshots.png`** — static montage of six key poses (start → shove →
  kicked back → catching → recovering → recovered) with the wheel speed at each.

## How it's built
- Reuses `simulate_sampled` with `sim.kick = struct('time',1.5,'dtheta_dot',deg2rad(560))`.
- Wheel display angle: `φ = cumtrapz(t, φ̇)` (integrated for drawing only — `φ`
  is not a dynamical state), absolute spoke angle `ψ = θ + φ`.
- `draw_rwip(ax, theta, psi, p, viz)` renders one pose; screen convention is
  "up = +y", a +θ (CCW) tip leans the arm toward −x.
- **Headless-safe capture:** each frame is written with `exportgraphics` to a temp
  PNG, read back, resized to a fixed size, and appended to the GIF with `imwrite`
  (`getframe` is unreliable under `matlab -batch`).

## Numbers from the run
peak |θ| after the shove = 7.7°, peak wheel = 228 rad/s (57% of the 400 rad/s
ceiling — margin to spare), final wheel = 1.5 rad/s (still bleeding to 0).

## Forward to Phase 5
Package the discrete gain `Kd`, the loop structure, and the EOM/sensor mapping
into an ESP32 implementation spec (read MPU6050 → fuse θ → compute `u = -Kd·x` →
back-EMF feed-forward → PWM). No firmware yet — just the spec.
