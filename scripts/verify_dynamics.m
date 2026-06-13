%% verify_dynamics.m  --  Phase 1 verification of the RWIP nonlinear dynamics
%
% Runs three independent checks on rwip_dynamics.m, with the motor torque set
% to zero and all friction set to zero, so the model must reduce to a
% conservative physical pendulum:
%
%   TEST 1  Energy conservation        : total energy E(t) stays constant
%   TEST 2  Free response = pendulum    : (a) wheel spin angular momentum is
%                                           conserved (decoupled free wheel),
%                                         (b) theta(t) matches an independent
%                                           single-pendulum integration
%   TEST 3  Small-angle period          : oscillation period about the hanging
%                                           equilibrium matches 2*pi*sqrt(I_p/(mgl*g))
%
% Each test prints a clear PASS/FAIL with the measured numbers; a summary
% banner is printed at the end, and the script errors (non-zero exit under
% `matlab -batch`) if any test fails. Diagnostic plots are written to results/.
%
% Run from the project root:   matlab -batch "run('scripts/verify_dynamics.m')"
% or from MATLAB:              >> run scripts/verify_dynamics.m

%% ---- Paths & setup ------------------------------------------------------
clear; close all; clc;

thisFile  = mfilename('fullpath');
scriptDir = fileparts(thisFile);
projRoot  = fileparts(scriptDir);
addpath(fullfile(projRoot, 'src'));

resultsDir = fullfile(projRoot, 'results');
if ~exist(resultsDir, 'dir'); mkdir(resultsDir); end

p = rwip_params();

% Conservative copy: force zero friction so the checks are exact.
pc = p; pc.b_theta = 0; pc.b_w = 0;

fprintf('==================================================================\n');
fprintf(' RWIP Phase 1 -- nonlinear dynamics verification\n');
fprintf('==================================================================\n');
fprintf(' Effective params:  I_p = %.6g kg*m^2   I_w = %.6g kg*m^2\n', pc.I_p, pc.I_w);
fprintf('                    mgl = %.6g kg*m      g   = %.4g m/s^2\n\n', pc.mgl, pc.g);

results = struct('name', {}, 'pass', {}, 'detail', {});

%% ======================================================================
%% TEST 1 -- Energy conservation
%% ======================================================================
% Large-amplitude swing through the bottom (strong PE<->KE exchange), with a
% spinning wheel to exercise the I_w term. tau = 0 => E must be constant.
x0   = [deg2rad(30); 0; 30];           % [theta, theta_dot, phi_dot]
tend = 5.0;
tspan = linspace(0, tend, 5000);
optsE = odeset('RelTol', 1e-10, 'AbsTol', 1e-12);
[t1, X1] = ode45(@(t,x) rwip_dynamics(t, x, 0, pc), tspan, x0, optsE);

E1 = rwip_energy(X1, pc);
E_drift_abs = max(abs(E1 - E1(1)));
E_drift_rel = E_drift_abs / abs(E1(1));
tol1  = 1e-6;
pass1 = E_drift_rel < tol1;

fprintf('-- TEST 1: Energy conservation (tau=0, no friction) --\n');
fprintf('   E(0)                = %+.10e J\n', E1(1));
fprintf('   max |E(t)-E(0)|     = %.3e J\n', E_drift_abs);
fprintf('   relative drift      = %.3e   (tol %.0e)\n', E_drift_rel, tol1);
fprintf('   --> %s\n\n', tf(pass1));
results(end+1) = mkres('Energy conservation', pass1, ...
    sprintf('rel drift %.2e', E_drift_rel));

%% ======================================================================
%% TEST 2 -- Free response equals a physical pendulum
%% ======================================================================
% (a) With tau=0 and no wheel friction, NO torque acts on the wheel, so its
%     absolute spin angular momentum L_w = I_w*(theta_dot + phi_dot) is
%     conserved -> the free wheel decouples from the swing.
% (b) Consequently theta(t) must satisfy the bare single-pendulum equation
%     theta_ddot = (mgl*g/I_p)*sin(theta). We integrate that separately and
%     compare.
x0   = [deg2rad(30); 0; 25];           % nonzero initial wheel spin
tspan = linspace(0, 3, 3000);
opts2 = odeset('RelTol', 1e-9, 'AbsTol', 1e-11);
[t2, X2] = ode45(@(t,x) rwip_dynamics(t, x, 0, pc), tspan, x0, opts2);

psidot = X2(:,2) + X2(:,3);            % absolute wheel rate
Lw     = pc.I_w * psidot;              % wheel spin angular momentum
Lw_drift = max(abs(Lw - Lw(1)));

% Independent single-pendulum reference for theta.
pend = @(t,y) [y(2); (pc.mgl*pc.g/pc.I_p)*sin(y(1))];
[~, TH] = ode45(pend, tspan, [x0(1); x0(2)], opts2);
theta_err = max(abs(X2(:,1) - TH(:,1)));

tolLw    = 1e-9;
tolTheta = 1e-4;
pass2 = (Lw_drift < tolLw) && (theta_err < tolTheta);

fprintf('-- TEST 2: Free response = physical pendulum (tau=0, no friction) --\n');
fprintf('   L_w(0) = I_w*(thd+phid) = %+.6e kg*m^2/s\n', Lw(1));
fprintf('   max |L_w(t)-L_w(0)|     = %.3e   (tol %.0e)  [wheel decoupled]\n', Lw_drift, tolLw);
fprintf('   max |theta_full - theta_pendulum| = %.3e rad  (tol %.0e)\n', theta_err, tolTheta);
fprintf('   --> %s\n\n', tf(pass2));
results(end+1) = mkres('Free response = pendulum', pass2, ...
    sprintf('Lw drift %.2e, theta err %.2e', Lw_drift, theta_err));

%% ======================================================================
%% TEST 3 -- Small-angle period about the hanging equilibrium
%% ======================================================================
% Stable equilibrium is theta = pi (hanging down). Linearising there gives
% SHM with omega^2 = mgl*g/I_p, i.e. period T = 2*pi*sqrt(I_p/(mgl*g)).
T_analytic = 2*pi*sqrt(pc.I_p/(pc.mgl*pc.g));
A0 = 0.05;                              % small amplitude [rad] (~2.9 deg)
x0 = [pi + A0; 0; 0];
nP = 8;
tspan = linspace(0, nP*T_analytic, 8000);
opts3 = odeset('RelTol', 1e-9, 'AbsTol', 1e-11);
[t3, X3] = ode45(@(t,x) rwip_dynamics(t, x, 0, pc), tspan, x0, opts3);

alpha = X3(:,1) - pi;                    % displacement from hanging eq.
% Upward zero crossings of alpha, refined by linear interpolation.
cross_t = [];
for k = 1:numel(alpha)-1
    if alpha(k) <= 0 && alpha(k+1) > 0
        tc = t3(k) - alpha(k)*(t3(k+1)-t3(k))/(alpha(k+1)-alpha(k));
        cross_t(end+1) = tc; %#ok<SAGROW>
    end
end
assert(numel(cross_t) >= 2, 'Not enough zero crossings to measure a period.');
T_measured = mean(diff(cross_t));
T_err_rel  = abs(T_measured - T_analytic) / T_analytic;
tol3  = 0.01;
pass3 = T_err_rel < tol3;

fprintf('-- TEST 3: Small-angle period about hanging eq. (theta=pi) --\n');
fprintf('   analytic  T = 2*pi*sqrt(I_p/(mgl*g)) = %.6f s\n', T_analytic);
fprintf('   measured  T (mean of %d cycles)       = %.6f s\n', numel(cross_t)-1, T_measured);
fprintf('   relative error                        = %.3e   (tol %.0e)\n', T_err_rel, tol3);
fprintf('   --> %s\n\n', tf(pass3));
results(end+1) = mkres('Small-angle period', pass3, ...
    sprintf('T_meas %.4fs vs %.4fs (%.2e)', T_measured, T_analytic, T_err_rel));

%% ======================================================================
%% Plots
%% ======================================================================
% Test 1: energy drift
f1 = figure('Visible','off','Position',[100 100 760 420]);
plot(t1, E1 - E1(1), 'LineWidth', 1.2); grid on;
xlabel('time [s]'); ylabel('E(t) - E(0)  [J]');
title(sprintf('Test 1: energy drift (rel %.2e)', E_drift_rel));
save_fig(f1, fullfile(resultsDir, 'phase1_test1_energy.png'));

% Test 2: theta overlay + wheel momentum
f2 = figure('Visible','off','Position',[100 100 760 560]);
subplot(2,1,1);
plot(t2, rad2deg(X2(:,1)), 'LineWidth', 1.4); hold on;
plot(t2, rad2deg(TH(:,1)), '--', 'LineWidth', 1.0); grid on;
xlabel('time [s]'); ylabel('\theta [deg]');
legend('full RWIP model','independent pendulum','Location','best');
title('Test 2: free arm response matches a single pendulum');
subplot(2,1,2);
plot(t2, Lw, 'LineWidth', 1.4); grid on;
xlabel('time [s]'); ylabel('L_w = I_w(\theta'' + \phi'')  [kg m^2/s]');
title(sprintf('wheel spin angular momentum conserved (drift %.1e)', Lw_drift));
save_fig(f2, fullfile(resultsDir, 'phase1_test2_pendulum.png'));

% Test 3: oscillation about hanging eq.
f3 = figure('Visible','off','Position',[100 100 760 420]);
plot(t3, rad2deg(alpha), 'LineWidth', 1.2); grid on; hold on;
yline(0, 'k:');
xlabel('time [s]'); ylabel('\theta - \pi  [deg]');
title(sprintf('Test 3: T_{meas}=%.4fs  vs  T_{analytic}=%.4fs', T_measured, T_analytic));
save_fig(f3, fullfile(resultsDir, 'phase1_test3_period.png'));

%% ======================================================================
%% Summary
%% ======================================================================
allpass = all([results.pass]);
fprintf('==================================================================\n');
fprintf(' SUMMARY\n');
fprintf('------------------------------------------------------------------\n');
for i = 1:numel(results)
    fprintf('  [%s]  %-28s  %s\n', tf(results(i).pass), results(i).name, results(i).detail);
end
fprintf('------------------------------------------------------------------\n');
fprintf('  OVERALL: %s\n', tf(allpass));
fprintf('  Plots written to: %s\n', resultsDir);
fprintf('==================================================================\n');

if ~allpass
    error('verify_dynamics:FAIL', 'One or more Phase-1 verification checks FAILED.');
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
