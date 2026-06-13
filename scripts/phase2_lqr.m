%% phase2_lqr.m  --  Phase 2: linearization about upright + LQR balancing
%
% Pipeline:
%   1. Linearize the nonlinear plant about upright -> (A,B,C,D); cross-check the
%      analytic Jacobian against a finite-difference Jacobian of rwip_dynamics.
%   2. Open-loop analysis: poles (one unstable), controllability.
%   3. LQR design with a phi_dot (wheel-speed) penalty in Q; show the closed-loop
%      poles. Contrast with a design that does NOT penalise wheel speed, to make
%      the RWIP-specific point that wheel speed must be regulated.
%   4. Simulate the NONLINEAR plant from an 8 deg perturbation under the LQR law
%      driving the realistic brushed-DC motor (back-EMF + voltage saturation).
%   5. PASS/FAIL on: linearization match, arm settling (theta->0), wheel speed
%      bounded AND bled back to ~0, motor within limits.
%
% Run from the project root:  matlab -batch "run('scripts/phase2_lqr.m')"

%% ---- Paths & setup ------------------------------------------------------
clear; close all; clc;

thisFile  = mfilename('fullpath');
scriptDir = fileparts(thisFile);
projRoot  = fileparts(scriptDir);
addpath(fullfile(projRoot, 'src'));
resultsDir = fullfile(projRoot, 'results');
if ~exist(resultsDir, 'dir'); mkdir(resultsDir); end

p = rwip_params();

fprintf('==================================================================\n');
fprintf(' RWIP Phase 2 -- linearization + LQR balancing\n');
fprintf('==================================================================\n');
fprintf(' I_p = %.5g  I_w = %.5g  mgl = %.5g  | motor: Kt=%.3g R=%.3g Vmax=%g\n', ...
        p.I_p, p.I_w, p.mgl, p.Kt, p.R, p.V_max);
fprintf(' stall torque Kt*Vmax/R = %.3f N*m | no-load wheel speed Vmax/Ke = %.0f rad/s\n\n', ...
        p.Kt*p.V_max/p.R, p.V_max/p.Ke);

results = struct('name', {}, 'pass', {}, 'detail', {});

%% ======================================================================
%% 1. Linearization + finite-difference cross-check
%% ======================================================================
[A, B, C, D] = rwip_linearize(p);

% central-difference Jacobian of the nonlinear dynamics at x=0, tau=0
x0e = [0;0;0];  u0 = 0;  h = 1e-6;
An = zeros(3,3);
for j = 1:3
    e = zeros(3,1); e(j) = h;
    An(:,j) = (rwip_dynamics(0, x0e+e, u0, p) - rwip_dynamics(0, x0e-e, u0, p)) / (2*h);
end
Bn = (rwip_dynamics(0, x0e, u0+h, p) - rwip_dynamics(0, x0e, u0-h, p)) / (2*h);
lin_err = max(max(abs(An - A), [], 'all'), max(abs(Bn - B)));
pass_lin = lin_err < 1e-6;

fprintf('-- 1. Linearization about upright (input = tau) --\n');
disp('   A ='); disp(A);
disp('   B ='); disp(B);
fprintf('   max |analytic - finite-difference Jacobian| = %.2e  (tol 1e-6) --> %s\n\n', ...
        lin_err, tf(pass_lin));
results(end+1) = mkres('Linearization vs FD Jacobian', pass_lin, sprintf('err %.1e', lin_err));

%% ======================================================================
%% 2. Open-loop analysis
%% ======================================================================
ol_poles = eig(A);
Co = [B, A*B, A*A*B];
ctrb_rank = rank(Co);
fprintf('-- 2. Open-loop analysis --\n');
fprintf('   open-loop poles: '); fprintf('% .4g  ', sort(real(ol_poles))); fprintf('[1/s]\n');
fprintf('   (unstable pole +%.3f = falling rate; one stable; one 0 = free wheel)\n', sqrt(p.mgl*p.g/p.I_p));
fprintf('   controllability rank = %d / 3  --> %s\n\n', ctrb_rank, tf(ctrb_rank==3));

%% ======================================================================
%% 3. LQR design (with wheel-speed penalty) + contrast without it
%% ======================================================================
% State x = [theta, theta_dot, phi_dot]. Weights (Bryson-rule starting point,
% then tuned): penalise angle hard, rate moderately, wheel speed lightly but
% NON-ZERO so the controller bleeds wheel momentum; R penalises torque.
q_theta = 200;      % angle           [1/rad^2]
q_thdot = 5;        % arm rate        [1/(rad/s)^2]
q_phidot = 2e-4;    % wheel speed     [1/(rad/s)^2]   <-- key RWIP term
R_tau    = 20;      % torque effort   [1/(N*m)^2]

Q = diag([q_theta, q_thdot, q_phidot]);
d = design_lqr(A, B, Q, R_tau);
K = d.K;

fprintf('-- 3. LQR design --\n');
fprintf('   Q = diag([%.4g %.4g %.4g])   R = %g\n', q_theta, q_thdot, q_phidot, R_tau);
fprintf('   K = [% .4f  % .4f  % .5f]   (tau_cmd = -K x)\n', K(1), K(2), K(3));
fprintf('   closed-loop poles: '); fprintf('% .3f ', sort(real(d.poles))); fprintf('[1/s]\n');
fprintf('   asymptotically stable: %s\n', tf(d.ok));

% Contrast A: NO wheel-speed penalty -> free-wheel mode (open-loop pole at the
% origin) is undetectable from the cost, so no stabilizing Riccati solution
% exists. This is the formal statement of "you must regulate wheel speed".
Q0 = diag([q_theta, q_thdot, 0]);
d0 = design_lqr(A, B, Q0, R_tau);
fprintf('   [contrast] q_phidot = 0     : ');
if d0.ok
    fprintf('stable (unexpected)\n');
else
    fprintf('LQR has NO stabilizing solution -- wheel mode undetectable.\n');
end

% Contrast B: a NEGLIGIBLE wheel penalty. This is technically stabilizing, but
% the wheel-regulation pole sits a hair inside the LHP, so over the demo window
% the wheel speed parks at an offset instead of bleeding off -> it would creep
% toward saturation under repeated disturbances. Used for the comparison plot.
q_phidot_weak = 1e-9;
dW = design_lqr(A, B, diag([q_theta, q_thdot, q_phidot_weak]), R_tau);
[~, slow_idx] = min(abs(real(dW.poles)));
fprintf('   [contrast] q_phidot = %.0e : stable, but wheel pole at %.3f 1/s (tau ~ %.1f s)\n\n', ...
        q_phidot_weak, real(dW.poles(slow_idx)), 1/abs(real(dW.poles(slow_idx))));
results(end+1) = mkres('LQR stable w/ phi_dot penalty', d.ok, ...
    sprintf('poles max real %.2f', max(real(d.poles))));

%% ======================================================================
%% 4. Nonlinear closed-loop simulation from 8 deg, realistic motor
%% ======================================================================
theta0 = deg2rad(8);
x0 = [theta0; 0; 0];
tspan = linspace(0, 4, 2001);
ctrl = @(t, x) -K * x;                          % LQR control law (modular handle)

% (a) realistic brushed-DC motor actuator
outM = simulate_rwip(p, x0, ctrl, @rwip_motor, tspan);
% (b) ideal torque source, for reference
outI = simulate_rwip(p, x0, ctrl, @ideal_actuator, tspan);
% (c) the negligible-wheel-penalty controller through the same motor: shows the
%     wheel speed parking at an offset instead of being regulated to zero.
ctrlW = @(t, x) -dW.K * x;
out0  = simulate_rwip(p, x0, ctrlW, @rwip_motor, tspan);

% --- success metrics on the realistic-motor run ---
th   = outM.X(:,1);
phid = outM.X(:,3);
t    = outM.t;

settle_tol = deg2rad(0.5);                       % |theta| < 0.5 deg
idx_after  = find(abs(th) > settle_tol, 1, 'last');
if isempty(idx_after)
    settle_time = 0;
else
    settle_time = t(min(idx_after+1, numel(t)));
end
theta_final_deg = rad2deg(th(end));
wheel_peak  = max(abs(phid));
wheel_final = abs(phid(end));
V_peak      = max(abs(outM.V));
V_demand    = max(abs((p.R/p.Kt)*outM.tau_cmd + p.Ke*phid));  % unsaturated demand
dt_uniform  = t(2) - t(1);
sat_dur     = sum(abs(outM.V) >= p.V_max - 1e-6) * dt_uniform;  % seconds saturated

pass_settle = abs(theta_final_deg) < 0.5 && settle_time < 3.0;
pass_wheel  = wheel_peak < p.V_max/p.Ke && wheel_final < 1.0;   % bounded & bled off
pass_motor  = V_peak <= p.V_max + 1e-6;

fprintf('-- 4. Nonlinear balance from %.0f deg (realistic motor) --\n', rad2deg(theta0));
fprintf('   settling time (|theta|<0.5deg) = %.3f s\n', settle_time);
fprintf('   final theta                    = %+.4f deg\n', theta_final_deg);
fprintf('   peak wheel speed               = %.1f rad/s (%.0f rpm) [limit %.0f rad/s]\n', ...
        wheel_peak, wheel_peak*60/(2*pi), p.V_max/p.Ke);
fprintf('   final wheel speed              = %.3f rad/s   (bled off -> ~0)\n', phid(end));
fprintf('   peak voltage demand            = %.1f V  -> delivered |V| capped at %.2f V (limit %g)\n', ...
        V_demand, V_peak, p.V_max);
fprintf('   motor saturated for            = %.0f ms (brief initial transient)\n', 1e3*sat_dur);
fprintf('   --> settling %s | wheel %s | motor %s\n\n', tf(pass_settle), tf(pass_wheel), tf(pass_motor));

results(end+1) = mkres('Arm settles (theta->0)', pass_settle, ...
    sprintf('t_s=%.2fs, final %.3f deg', settle_time, theta_final_deg));
results(end+1) = mkres('Wheel bounded & regulated', pass_wheel, ...
    sprintf('peak %.0f, final %.2f rad/s', wheel_peak, phid(end)));
results(end+1) = mkres('Motor within limits', pass_motor, ...
    sprintf('Vpeak %.2f V', V_peak));

%% ======================================================================
%% Plots
%% ======================================================================
f1 = figure('Visible','off','Position',[80 80 900 760]);

subplot(3,1,1);
plot(outM.t, rad2deg(outM.X(:,1)), 'LineWidth',1.6); hold on;
plot(outI.t, rad2deg(outI.X(:,1)), '--', 'LineWidth',1.0);
yline(0,'k:'); grid on; ylabel('\theta  [deg]');
legend('realistic motor','ideal torque','Location','northeast');
title(sprintf('Phase 2: LQR balance from %.0f deg', rad2deg(theta0)));

subplot(3,1,2);
yline(0,'k:','HandleVisibility','off'); hold on;
h_good = plot(outM.t, outM.X(:,3), 'LineWidth',1.6);
if ~isempty(out0)
    h_park = plot(out0.t, out0.X(:,3), '-.', 'LineWidth',1.2);
    legend([h_good h_park], 'with \phi'' penalty (bled to ~0)', ...
           'negligible \phi'' penalty (parks at offset)', 'Location','east');
end
grid on; ylabel('wheel speed \phi''  [rad/s]');

subplot(3,1,3);
plot(outM.t, outM.tau, 'LineWidth',1.6); hold on;
plot(outM.t, outM.tau_cmd, '--', 'LineWidth',0.9);
grid on; ylabel('torque  [N*m]'); xlabel('time [s]');
legend('delivered \tau','commanded \tau_{cmd}','Location','northeast');
save_fig(f1, fullfile(resultsDir, 'phase2_lqr_balance.png'));

% voltage / saturation detail
f2 = figure('Visible','off','Position',[80 80 900 360]);
plot(outM.t, outM.V, 'LineWidth',1.4); hold on;
yline(p.V_max,'r:'); yline(-p.V_max,'r:'); grid on;
xlabel('time [s]'); ylabel('motor voltage [V]');
title(sprintf('Motor voltage (limit \\pm%g V)', p.V_max));
save_fig(f2, fullfile(resultsDir, 'phase2_voltage.png'));

%% ======================================================================
%% Summary
%% ======================================================================
allpass = all([results.pass]);
fprintf('==================================================================\n');
fprintf(' SUMMARY\n');
fprintf('------------------------------------------------------------------\n');
for i = 1:numel(results)
    fprintf('  [%s]  %-30s  %s\n', tf(results(i).pass), results(i).name, results(i).detail);
end
fprintf('------------------------------------------------------------------\n');
fprintf('  OVERALL: %s\n', tf(allpass));
fprintf('  Gain K = [% .4f % .4f % .5f]\n', K(1), K(2), K(3));
fprintf('  Plots written to: %s\n', resultsDir);
fprintf('==================================================================\n');

if ~allpass
    error('phase2_lqr:FAIL', 'One or more Phase-2 checks FAILED.');
end

%% ---- local helpers ------------------------------------------------------
function s = tf(b)
    if b; s = 'PASS'; else; s = 'FAIL'; end
end
function r = mkres(name, pass, detail)
    r = struct('name', name, 'pass', logical(pass), 'detail', detail);
end
function save_fig(figh, fname)
    try
        exportgraphics(figh, fname, 'Resolution', 150);
    catch
        saveas(figh, fname);
    end
end
