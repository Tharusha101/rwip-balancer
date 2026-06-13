function out = simulate_sampled(p, x0, sim)
%SIMULATE_SAMPLED  Sampled-data closed-loop simulation of the RWIP.
%
%   out = SIMULATE_SAMPLED(p, x0, sim) runs the nonlinear plant under a
%   discrete-time controller updated every sim.Ts seconds (zero-order hold on
%   the actuator command between samples), integrating the continuous plant
%   with ode45 across each interval. This is the realistic sampled-data setup
%   used in Phase 3 (and reused in Phase 4).
%
%   sim fields:
%     .Ts, .Tend        sample period [s] and total time [s]
%     .controller       handle  [u,s] = controller(s, y, Ts, cfg)
%     .cfg              controller config (gains etc.)
%     .s0               initial controller state struct
%     .sensor           handle  y = sensor(x, p, k, scfg)   ([] -> perfect, y=x)
%     .scfg             sensor config
%     .use_motor        true  -> brushed-DC actuator driven by voltage
%                       false -> ideal torque source (tau = u directly)
%     .use_pwm          true  -> quantise the voltage to .pwm_bits resolution
%     .pwm_bits         PWM resolution in bits (e.g. 10 for ESP32 LEDC)
%     .use_motor_lag    true  -> first-order electrical lag (motor current
%                       state, inductance p.Lm); false -> instantaneous torque
%     .odeopts          (optional) odeset for the per-interval integration
%
%   out: .t,.X        fine trajectory (each sample subdivided), X is N-by-3
%        .tau         fine delivered torque
%        .ts,.y       sample times and measurements (Ns-by-3)
%        .tau_cmd,.V  sample-time commanded torque and applied voltage
%        .effort      control effort  integral( tau^2 dt )
%
%   Controller and sensor are function handles, so PID/LQR and clean/noisy
%   sensing swap in without touching this harness.

    if ~isfield(sim,'odeopts') || isempty(sim.odeopts)
        sim.odeopts = odeset('RelTol',1e-7,'AbsTol',1e-9);
    end
    nfine = 4;                                  % plot points per sample interval
    Ts = sim.Ts;  N = round(sim.Tend/Ts);

    x = x0(:);  s = sim.s0;  ie = 0;            % plant state, ctrl state, motor current
    kicked = false;

    T = [];  X = [];  TAU = [];
    ts = zeros(N,1);  Ylog = zeros(N,3);  Ucmd = zeros(N,1);  Vlog = zeros(N,1);
    effort = 0;

    for k = 0:N-1
        tk = k*Ts;

        % ---- optional impulse disturbance: a velocity "shove" on the arm ----
        if isfield(sim,'kick') && ~isempty(sim.kick) && ~kicked && tk >= sim.kick.time
            x(2) = x(2) + sim.kick.dtheta_dot;
            kicked = true;
        end

        % ---- measure ----
        if isempty(sim.sensor)
            y = x;
        else
            y = sim.sensor(x, p, k, sim.scfg);
        end

        % ---- control (held over the interval) ----
        [u, s] = sim.controller(s, y, Ts, sim.cfg);

        % ---- actuator: command -> voltage (if a real motor is modelled) ----
        if sim.use_motor
            Vk = cmd_to_voltage(u, y(3), p, sim);   % back-EMF FF uses measured wheel speed
        else
            Vk = NaN;
        end

        % ---- integrate the plant across [tk, tk+Ts] with the held command ----
        tgrid = linspace(tk, tk+Ts, nfine+1);
        if ~sim.use_motor
            rhs = @(t,z) rwip_dynamics(t, z, u, p);
            [tt, zz] = ode45(rhs, tgrid, x, sim.odeopts);
            xx = zz;  tau_fine = u*ones(numel(tt),1);
        elseif ~sim.use_motor_lag
            rhs = @(t,z) rwip_dynamics(t, z, (p.Kt/p.R)*(Vk - p.Ke*z(3)), p);
            [tt, zz] = ode45(rhs, tgrid, x, sim.odeopts);
            xx = zz;  tau_fine = (p.Kt/p.R)*(Vk - p.Ke*zz(:,3));
        else
            % augmented state [theta; theta_dot; phi_dot; i], tau = Kt*i,
            % L*di/dt = Vk - i*R - Ke*phi_dot
            rhs = @(t,z) [ rwip_dynamics(t, z(1:3), p.Kt*z(4), p);
                          (Vk - z(4)*p.R - p.Ke*z(3))/p.Lm ];
            [tt, zz] = ode45(rhs, tgrid, [x; ie], sim.odeopts);
            xx = zz(:,1:3);  tau_fine = p.Kt*zz(:,4);  ie = zz(end,4);
        end
        x = xx(end,:).';

        % ---- logs ----
        if k == 0
            T = tt;  X = xx;  TAU = tau_fine;
        else
            T = [T; tt(2:end)];  X = [X; xx(2:end,:)];  TAU = [TAU; tau_fine(2:end)]; %#ok<AGROW>
        end
        effort = effort + trapz(tt, tau_fine.^2);
        ts(k+1) = tk;  Ylog(k+1,:) = y(:).';  Ucmd(k+1) = u;  Vlog(k+1) = Vk;
    end

    out = struct('t',T, 'X',X, 'tau',TAU, 'ts',ts, 'y',Ylog, ...
                 'tau_cmd',Ucmd, 'V',Vlog, 'effort',effort);
end

% -------------------------------------------------------------------------
function Vk = cmd_to_voltage(tau_cmd, phid_meas, p, sim)
%CMD_TO_VOLTAGE  Torque command -> motor voltage, with back-EMF feed-forward,
%   supply saturation, and optional PWM quantisation.
    V_req = (p.R/p.Kt)*tau_cmd + p.Ke*phid_meas;     % volts for the requested torque
    Vk    = max(min(V_req, p.V_max), -p.V_max);       % supply saturation
    if isfield(sim,'use_pwm') && sim.use_pwm
        q  = 2*p.V_max / (2^sim.pwm_bits - 1);        % PWM voltage step
        Vk = round(Vk/q) * q;                          % quantise
    end
end
