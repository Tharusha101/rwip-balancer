%% phase3_pid_robust.m  --  Phase 3: PID baseline + robustness under realism
%
% Part A1  Clean 8 deg release (200 Hz, ideal sensors/actuator): discrete LQR
%          vs PID on arm angle. Metrics: settling, overshoot, effort, peak wheel.
% Part A2  Impulse "shove": the decisive difference. PID has NO wheel-speed
%          feedback, so momentum dumped into the wheel by a disturbance stays
%          there forever (the wheel mode is an unregulated integrator); LQR
%          returns the wheel toward zero. This is the wheel-saturation problem.
% Part B   Robustness: drive the SAME discrete LQR through the realistic hardware
%          chain -- gyro noise + bias, accel noise, motor electrical lag, 10-bit
%          PWM, 200 Hz -- and confirm it still stabilises.
%
% Run from the project root:  matlab -batch "run('scripts/phase3_pid_robust.m')"

%% ---- Paths & setup ------------------------------------------------------
clear; close all; clc;
rng(1);                                   % reproducible noise

thisFile  = mfilename('fullpath');
scriptDir = fileparts(thisFile);
projRoot  = fileparts(scriptDir);
addpath(fullfile(projRoot, 'src'));
resultsDir = fullfile(projRoot, 'results');
if ~exist(resultsDir, 'dir'); mkdir(resultsDir); end

p = rwip_params();
[A, B, C, D] = rwip_linearize(p);

Ts = 1/200;                               % 200 Hz control loop
theta0 = deg2rad(8);
x0 = [theta0; 0; 0];

fprintf('==================================================================\n');
fprintf(' RWIP Phase 3 -- PID baseline + robustness (loop rate %d Hz)\n', round(1/Ts));
fprintf('==================================================================\n\n');

results = struct('name', {}, 'pass', {}, 'detail', {});

%% ======================================================================
%% Controller designs
%% ======================================================================
Q = diag([200, 5, 2e-4]);  Rw = 20;
sysd = c2d(ss(A,B,C,D), Ts, 'zoh');
[Ad, Bd] = ssdata(sysd);
Kd = dlqr(Ad, Bd, Q, Rw);                  % discrete LQR gain (used everywhere)
Kc = lqr(A, B, Q, Rw);                      % continuous gain, reference

% PID on arm angle. PD-dominant; small integral to reject the gyro bias in
% Part B. NOTE the gain on phi_dot is structurally ZERO -- PID cannot see or
% regulate wheel speed.
pid.Kp = 1.2;  pid.Kd = 0.14;  pid.Ki = 1.0;  pid.Imax = 0.05;

fprintf('-- Controller designs --\n');
fprintf('   continuous LQR  K  = [% .4f % .4f % .5f]\n', Kc);
fprintf('   discrete  LQR  Kd  = [% .4f % .4f % .5f]  (dlqr @ %d Hz)\n', Kd, round(1/Ts));
fprintf('   ZOH discretisation lowers the gain ~%.0f%% vs continuous (still stable)\n', ...
        100*max(abs(Kc(:)-Kd(:))./abs(Kc(:))));
fprintf('   PID(theta): Kp=%.2f Kd=%.3f Ki=%.2f -> gain on phi_dot = 0 (wheel unmanaged)\n\n', ...
        pid.Kp, pid.Kd, pid.Ki);

%% ======================================================================
%% Part A1 -- clean comparison (ideal sensors + ideal torque, 200 Hz)
%% ======================================================================
simBase = struct('Ts',Ts,'Tend',4.0,'sensor',[],'scfg',[], ...
                 'use_motor',false,'use_pwm',false,'use_motor_lag',false);

simL = simBase; simL.controller=@ctrl_lqr; simL.cfg.K=Kd;  simL.s0=struct();
simP = simBase; simP.controller=@ctrl_pid; simP.cfg=pid;   simP.s0=struct('I',0);
outL = simulate_sampled(p, x0, simL);
outP = simulate_sampled(p, x0, simP);
mL = balance_metrics(outL);  mP = balance_metrics(outP);

fprintf('-- Part A1: clean 8 deg balance, discrete LQR vs PID (200 Hz, ideal) --\n');
fprintf('   %-24s %10s %10s\n', '', 'LQR', 'PID');
fprintf('   %-24s %10.3f %10.3f\n', 'settling time [s]',   mL.settle, mP.settle);
fprintf('   %-24s %10.3f %10.3f\n', 'overshoot [deg]',     mL.overshoot, mP.overshoot);
fprintf('   %-24s %10.4f %10.4f\n', 'control effort Jtau', outL.effort, outP.effort);
fprintf('   %-24s %10.1f %10.1f\n', 'peak wheel [rad/s]',  mL.wheel_peak, mP.wheel_peak);
fprintf('   note: PID balances the arm (a touch faster, cheaper) but uses more\n');
fprintf('         wheel speed and -- crucially -- has no wheel-speed feedback.\n');
passA1 = mL.settle < 1.5 && mP.settle < 1.5 && abs(rad2deg(outL.X(end,1))) < 0.5;
fprintf('   --> both controllers balance the arm: %s\n\n', tf(passA1));
results(end+1) = mkres('A1: LQR & PID both balance', passA1, ...
    sprintf('t_s LQR=%.2f PID=%.2f', mL.settle, mP.settle));

%% ======================================================================
%% Part A2 -- impulse "shove": wheel-momentum management
%% ======================================================================
simBaseD = simBase; simBaseD.Tend = 6.0;
simBaseD.kick = struct('time', 1.5, 'dtheta_dot', deg2rad(120));  % +120 deg/s shove

simLD = simBaseD; simLD.controller=@ctrl_lqr; simLD.cfg.K=Kd; simLD.s0=struct();
simPD = simBaseD; simPD.controller=@ctrl_pid; simPD.cfg=pid;  simPD.s0=struct('I',0);
outLD = simulate_sampled(p, x0, simLD);
outPD = simulate_sampled(p, x0, simPD);

wheel_end_L = outLD.X(end,3);
wheel_end_P = outPD.X(end,3);
arm_ok = abs(rad2deg(outLD.X(end,1))) < 1 && abs(rad2deg(outPD.X(end,1))) < 1;

fprintf('-- Part A2: +120 deg/s impulse shove at t=1.5 s (both keep the arm up) --\n');
fprintf('   wheel speed 4.5 s after the shove:  LQR = %.2f rad/s   PID = %.1f rad/s\n', ...
        wheel_end_L, wheel_end_P);
fprintf('   LQR returns the wheel toward zero; PID leaves it spinning (no phi_dot\n');
fprintf('   feedback) -> under repeated shoves PID''s wheel walks to saturation.\n');
passA2 = abs(wheel_end_L) < 2.0 && abs(wheel_end_P) > 10 && arm_ok;
fprintf('   --> wheel-regulation advantage demonstrated: %s\n\n', tf(passA2));
results(end+1) = mkres('A2: LQR regulates wheel, PID does not', passA2, ...
    sprintf('wheel_end LQR=%.2f vs PID=%.1f rad/s', wheel_end_L, wheel_end_P));

%% ======================================================================
%% Part B -- LQR through the full realistic hardware chain
%% ======================================================================
scfg.sigma_theta = deg2rad(0.30);     % accel/fused tilt noise   [rad]
scfg.sigma_gyro  = deg2rad(0.25);     % gyro white noise         [rad/s]
scfg.gyro_bias   = deg2rad(1.50);     % gyro CONSTANT bias       [rad/s]
scfg.sigma_wheel = 0.50;              % wheel-speed estimate noise [rad/s]

simR = struct('Ts',Ts,'Tend',4.0,'controller',@ctrl_lqr,'cfg',struct('K',Kd), ...
              's0',struct(),'sensor',@sensor_imu,'scfg',scfg, ...
              'use_motor',true,'use_pwm',true,'pwm_bits',10,'use_motor_lag',true);
outR = simulate_sampled(p, x0, simR);

tail = outR.t >= (simR.Tend - 1.0);
theta_rms_deg = rad2deg(sqrt(mean(outR.X(tail,1).^2)));
theta_pk_deg  = rad2deg(max(abs(outR.X(:,1))));
wheel_peakR   = max(abs(outR.X(:,3)));

stable_R = theta_pk_deg < 15 && theta_rms_deg < 2.0 && wheel_peakR < p.V_max/p.Ke;
fprintf('-- Part B: LQR under gyro bias+noise, accel noise, motor lag, 10-bit PWM, 200 Hz --\n');
fprintf('   sensor: sigma_theta=%.2f deg, gyro bias=%.2f deg/s (+/-%.2f), wheel noise=%.2f rad/s\n', ...
        rad2deg(scfg.sigma_theta), rad2deg(scfg.gyro_bias), rad2deg(scfg.sigma_gyro), scfg.sigma_wheel);
fprintf('   peak |theta| = %.2f deg | steady RMS theta = %.3f deg (last 1 s)\n', theta_pk_deg, theta_rms_deg);
fprintf('   peak wheel   = %.1f rad/s [limit %.0f] | final wheel = %.2f rad/s\n', ...
        wheel_peakR, p.V_max/p.Ke, outR.X(end,3));
fprintf('   (the ~%.0f rad/s wheel offset is LQR holding against the constant gyro bias)\n', abs(outR.X(end,3)));
fprintf('   --> LQR remains stable under full realism: %s\n\n', tf(stable_R));
results(end+1) = mkres('B: LQR stable under full realism', stable_R, ...
    sprintf('peak %.1f deg, RMS %.2f deg', theta_pk_deg, theta_rms_deg));

%% ======================================================================
%% Plots
%% ======================================================================
% A1 clean comparison
fA = figure('Visible','off','Position',[60 60 920 760]);
subplot(3,1,1);
plot(outL.t, rad2deg(outL.X(:,1)),'LineWidth',1.6); hold on;
plot(outP.t, rad2deg(outP.X(:,1)),'--','LineWidth',1.4); yline(0,'k:','HandleVisibility','off');
grid on; ylabel('\theta [deg]'); legend('LQR','PID','Location','northeast');
title('Phase 3A1: discrete LQR vs PID, clean 8 deg release (200 Hz, ideal)');
subplot(3,1,2);
plot(outL.t, outL.X(:,3),'LineWidth',1.6); hold on;
plot(outP.t, outP.X(:,3),'--','LineWidth',1.4); yline(0,'k:','HandleVisibility','off');
grid on; ylabel('wheel \phi'' [rad/s]'); legend('LQR','PID','Location','northeast');
subplot(3,1,3);
plot(outL.t, outL.tau,'LineWidth',1.6); hold on;
plot(outP.t, outP.tau,'--','LineWidth',1.4);
grid on; ylabel('torque [N*m]'); xlabel('time [s]'); legend('LQR','PID','Location','northeast');
save_fig(fA, fullfile(resultsDir,'phase3a_lqr_vs_pid.png'));

% A2 disturbance -- the wheel-regulation money plot
fD = figure('Visible','off','Position',[60 60 920 560]);
subplot(2,1,1);
plot(outLD.t, rad2deg(outLD.X(:,1)),'LineWidth',1.5); hold on;
plot(outPD.t, rad2deg(outPD.X(:,1)),'--','LineWidth',1.4);
xline(1.5,'k:','shove','HandleVisibility','off'); yline(0,'k:','HandleVisibility','off');
grid on; ylabel('\theta [deg]'); legend('LQR','PID','Location','northeast');
title('Phase 3A2: +120 deg/s shove at t=1.5 s -- both keep the arm up');
subplot(2,1,2);
plot(outLD.t, outLD.X(:,3),'LineWidth',1.6); hold on;
plot(outPD.t, outPD.X(:,3),'--','LineWidth',1.6);
xline(1.5,'k:','HandleVisibility','off'); yline(0,'k:','HandleVisibility','off');
grid on; ylabel('wheel \phi'' [rad/s]'); xlabel('time [s]');
legend('LQR (wheel \rightarrow 0)','PID (wheel stays spun up)','Location','northwest');
title('...but PID leaves the wheel spinning -- LQR bleeds the momentum back off');
save_fig(fD, fullfile(resultsDir,'phase3a2_disturbance_wheel.png'));

% B robustness
fB = figure('Visible','off','Position',[60 60 920 760]);
subplot(3,1,1);
plot(outR.t, rad2deg(outR.X(:,1)),'LineWidth',1.2); hold on;
plot(outR.ts, rad2deg(outR.y(:,1)),'.','MarkerSize',4);
yline(0,'k:','HandleVisibility','off'); grid on; ylabel('\theta [deg]');
legend('true \theta','measured \theta (noisy)','Location','northeast');
title('Phase 3B: LQR under full sensor/actuator realism');
subplot(3,1,2);
plot(outR.t, outR.X(:,3),'LineWidth',1.2); yline(0,'k:','HandleVisibility','off');
grid on; ylabel('wheel \phi'' [rad/s]');
subplot(3,1,3);
stairs(outR.ts, outR.V,'LineWidth',1.0); hold on;
yline(p.V_max,'r:','HandleVisibility','off'); yline(-p.V_max,'r:','HandleVisibility','off');
grid on; ylabel('PWM voltage [V]'); xlabel('time [s]');
title(sprintf('10-bit PWM voltage (step %.3f V), \\pm%g V supply', 2*p.V_max/(2^10-1), p.V_max));
save_fig(fB, fullfile(resultsDir,'phase3b_robustness.png'));

%% ======================================================================
%% Summary
%% ======================================================================
allpass = all([results.pass]);
fprintf('==================================================================\n');
fprintf(' SUMMARY\n');
fprintf('------------------------------------------------------------------\n');
for i = 1:numel(results)
    fprintf('  [%s]  %-36s  %s\n', tf(results(i).pass), results(i).name, results(i).detail);
end
fprintf('------------------------------------------------------------------\n');
fprintf('  OVERALL: %s\n', tf(allpass));
fprintf('  Discrete LQR gain Kd = [% .4f % .4f % .5f]  (200 Hz, for Phase 5 export)\n', Kd);
fprintf('  Plots written to: %s\n', resultsDir);
fprintf('==================================================================\n');

if ~allpass
    error('phase3:FAIL', 'One or more Phase-3 checks FAILED.');
end

%% ---- local helpers ------------------------------------------------------
function m = balance_metrics(out)
    th = out.X(:,1);  t = out.t;  phid = out.X(:,3);
    idx = find(abs(th) > deg2rad(0.5), 1, 'last');
    if isempty(idx); m.settle = 0; else; m.settle = t(min(idx+1,numel(t))); end
    m.overshoot   = rad2deg(max(-th));      % release is +8 deg; opposite swing
    m.wheel_peak  = max(abs(phid));
    m.wheel_final = phid(end);
end
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
