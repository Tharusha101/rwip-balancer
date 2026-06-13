%% phase4_animate.m  --  Phase 4: visualization + disturbance (GIF centerpiece)
%
% Balances the RWIP from a small tilt, injects an impulse "shove" mid-balance,
% and shows the recovery -- the arm catches itself while the reaction wheel
% spins up and then bleeds its momentum back off. Produces:
%
%   results/phase4_balance.gif    animated mechanism + live theta/wheel traces
%   results/phase4_snapshots.png  static montage of key poses (portfolio still)
%
% Uses the Phase-3 discrete LQR (Kd) through simulate_sampled with sim.kick.
% Headless-safe: each frame is rendered with exportgraphics to a temp PNG and
% appended to the GIF (getframe is unreliable under `matlab -batch`).
%
% Run from the project root:  matlab -batch "run('scripts/phase4_animate.m')"

%% ---- Paths & setup ------------------------------------------------------
clear; close all; clc;
thisFile  = mfilename('fullpath');
scriptDir = fileparts(thisFile);
projRoot  = fileparts(scriptDir);
addpath(fullfile(projRoot, 'src'));
resultsDir = fullfile(projRoot, 'results');
if ~exist(resultsDir, 'dir'); mkdir(resultsDir); end

p = rwip_params();
[A, B, C, D] = rwip_linearize(p);
Ts = 1/200;
sysd = c2d(ss(A,B,C,D), Ts, 'zoh');
[Ad, Bd] = ssdata(sysd);
Kd = dlqr(Ad, Bd, diag([200,5,2e-4]), 20);   % Phase-3 discrete LQR gain

%% ---- Simulate: balance from 5 deg, shove at t = 1.5 s -------------------
t_shove = 1.5;
sim = struct('Ts',Ts, 'Tend',6.0, 'controller',@ctrl_lqr, 'cfg',struct('K',Kd), ...
             's0',struct(), 'sensor',[], 'scfg',[], ...
             'use_motor',false, 'use_pwm',false, 'use_motor_lag',false, ...
             'kick',struct('time',t_shove, 'dtheta_dot',deg2rad(560)));
x0 = [deg2rad(10); 0; 0];
out = simulate_sampled(p, x0, sim);

% display angles: integrate wheel speed to get wheel angle (display only)
phi = cumtrapz(out.t, out.X(:,3));      % wheel angle relative to arm
psi = out.X(:,1) + phi;                 % ABSOLUTE wheel angle for the spoke
th  = out.X(:,1);  wsp = out.X(:,3);

fprintf('Phase 4: simulated %.1f s, shove +%.0f deg/s at t=%.1f s.\n', ...
        sim.Tend, rad2deg(sim.kick.dtheta_dot), t_shove);
fprintf('  peak |theta| after shove = %.1f deg, peak wheel = %.1f rad/s, final wheel = %.2f rad/s\n', ...
        rad2deg(max(abs(th(out.t>t_shove)))), max(abs(wsp)), wsp(end));

viz.r_w = 0.06;                          % wheel display radius [m]
th_lim  = max(8, rad2deg(max(abs(th)))*1.15);
w_lim   = max(abs(wsp))*1.15;

%% ---- Animated GIF -------------------------------------------------------
fps  = 25;
tf   = 0:1/fps:sim.Tend;
thf  = interp1(out.t, th,  tf);
psif = interp1(out.t, psi, tf);
wspf = interp1(out.t, wsp, tf);

gifFile = fullfile(resultsDir, 'phase4_balance.gif');
tmpPng  = fullfile(resultsDir, '_frame_tmp.png');

fig = figure('Visible','off','Color','w','Position',[100 100 980 480]);
tl  = tiledlayout(fig, 2, 2, 'TileSpacing','compact', 'Padding','compact');
ax1 = nexttile(tl, 1, [2 1]);            % mechanism (spans both rows, left col)
ax2 = nexttile(tl, 2);                   % theta trace (top right)
ax3 = nexttile(tl, 4);                   % wheel trace (bottom right)

refSize = [];
for i = 1:numel(tf)
    % --- mechanism ---
    draw_rwip(ax1, thf(i), psif(i), p, viz);
    title(ax1, sprintf('RWIP balancing under LQR    t = %.2f s', tf(i)), 'FontWeight','bold');
    text(ax1, -0.30, 0.305, sprintf('\\theta = %+5.1f\\circ', rad2deg(thf(i))), 'FontSize',11);
    text(ax1, -0.30, 0.280, sprintf('wheel = %+5.0f rad/s', wspf(i)), 'FontSize',11, 'Color',[0.85 0.33 0.10]);
    if abs(tf(i)-t_shove) < 0.12
        text(ax1, 0.02, 0.16, 'SHOVE!', 'FontSize',14, 'FontWeight','bold', 'Color',[0.8 0 0]);
    end

    % --- theta trace ---
    cla(ax2); hold(ax2,'on'); set(ax2,'Color','w');
    plot(ax2, tf(1:i), rad2deg(thf(1:i)), 'Color',[0.00 0.45 0.74], 'LineWidth',1.6);
    plot(ax2, tf(i), rad2deg(thf(i)), 'o', 'MarkerSize',5, 'MarkerFaceColor',[0.00 0.45 0.74], 'MarkerEdgeColor','k');
    xline(ax2, t_shove, 'r:'); yline(ax2, 0, 'k:');
    xlim(ax2,[0 sim.Tend]); ylim(ax2,[-th_lim th_lim]);
    ylabel(ax2,'\theta [deg]'); grid(ax2,'on'); hold(ax2,'off');
    title(ax2,'arm angle');

    % --- wheel trace ---
    cla(ax3); hold(ax3,'on'); set(ax3,'Color','w');
    plot(ax3, tf(1:i), wspf(1:i), 'Color',[0.85 0.33 0.10], 'LineWidth',1.6);
    plot(ax3, tf(i), wspf(i), 'o', 'MarkerSize',5, 'MarkerFaceColor',[0.85 0.33 0.10], 'MarkerEdgeColor','k');
    xline(ax3, t_shove, 'r:'); yline(ax3, 0, 'k:');
    xlim(ax3,[0 sim.Tend]); ylim(ax3,[-w_lim w_lim]);
    ylabel(ax3,'wheel [rad/s]'); xlabel(ax3,'time [s]'); grid(ax3,'on'); hold(ax3,'off');
    title(ax3,'wheel speed (spins up on the shove, then bleeds to 0)');

    % --- capture frame (headless-safe via exportgraphics) ---
    exportgraphics(fig, tmpPng, 'Resolution', 96);
    im = imread(tmpPng);
    if isempty(refSize); refSize = [size(im,1) size(im,2)]; else; im = imresize(im, refSize); end
    [Aind, map] = rgb2ind(im, 256);
    if i == 1
        imwrite(Aind, map, gifFile, 'gif', 'LoopCount',Inf, 'DelayTime',1/fps);
    else
        imwrite(Aind, map, gifFile, 'gif', 'WriteMode','append', 'DelayTime',1/fps);
    end
end
if exist(tmpPng,'file'); delete(tmpPng); end
close(fig);
fprintf('  wrote %s  (%d frames @ %d fps)\n', gifFile, numel(tf), fps);

%% ---- Static snapshot montage (portfolio still) --------------------------
snapT = [0.0, t_shove-0.05, 1.68, 2.10, 3.20, 5.80];
labels = {'start: +10\circ', 'shove instant', 'kicked back', 'catching', 'recovering', 'recovered'};
fM = figure('Visible','off','Color','w','Position',[80 80 1320 300]);
tlM = tiledlayout(fM, 1, numel(snapT), 'TileSpacing','compact', 'Padding','compact');
for j = 1:numel(snapT)
    axj = nexttile(tlM);
    thj  = interp1(out.t, th,  snapT(j));
    psij = interp1(out.t, psi, snapT(j));
    wj   = interp1(out.t, wsp, snapT(j));
    draw_rwip(axj, thj, psij, p, viz);
    title(axj, sprintf('%s\n t=%.2fs, %+.0f rad/s', labels{j}, snapT(j), wj), 'FontSize',9);
end
title(tlM, sprintf('Phase 4: balance \\rightarrow +%.0f deg/s shove \\rightarrow recovery (LQR)', ...
      rad2deg(sim.kick.dtheta_dot)), 'FontWeight','bold');
exportgraphics(fM, fullfile(resultsDir,'phase4_snapshots.png'), 'Resolution', 150);
close(fM);
fprintf('  wrote %s\n', fullfile(resultsDir,'phase4_snapshots.png'));
fprintf('Phase 4 done.\n');
