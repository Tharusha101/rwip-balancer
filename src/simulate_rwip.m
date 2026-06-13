function out = simulate_rwip(p, x0, ctrl, actuator, tspan, odeopts)
%SIMULATE_RWIP  Closed-loop nonlinear simulation of the RWIP.
%
%   out = SIMULATE_RWIP(p, x0, ctrl, actuator, tspan, odeopts) integrates the
%   nonlinear plant rwip_dynamics under a feedback controller and an actuator.
%
%   Inputs
%     p        : parameter struct (rwip_params)
%     x0       : initial state [theta; theta_dot; phi_dot]
%     ctrl     : controller, function handle  tau_cmd = ctrl(t, x)
%     actuator : actuator, handle  [tau,V] = actuator(tau_cmd, phidot, p)
%                e.g. @rwip_motor (realistic) or @ideal_actuator. [] -> ideal.
%     tspan    : time vector for ode45 output
%     odeopts  : (optional) odeset struct; sensible default if omitted
%
%   Output struct out:
%     .t       N-by-1 time
%     .X       N-by-3 states  [theta theta_dot phi_dot]
%     .tau_cmd N-by-1 commanded torque
%     .tau     N-by-1 delivered torque (after actuator)
%     .V       N-by-1 motor voltage (NaN for ideal actuator)
%
%   The controller and actuator are passed as handles so a different control
%   law (PID in Phase 3) or actuator drops in without touching this harness.

    if nargin < 6 || isempty(odeopts)
        odeopts = odeset('RelTol', 1e-8, 'AbsTol', 1e-10);
    end
    if nargin < 4 || isempty(actuator)
        actuator = @ideal_actuator;
    end

    % Closed loop: command -> actuator -> plant. ode45 uses the 1st (tau) output.
    rhs = @(t, x) rwip_dynamics(t, x, actuator(ctrl(t, x), x(3), p), p);
    [t, X] = ode45(rhs, tspan, x0(:), odeopts);

    % Reconstruct command / delivered torque / voltage along the trajectory.
    N = numel(t);
    tau_cmd = zeros(N, 1);
    tau     = zeros(N, 1);
    V       = zeros(N, 1);
    for i = 1:N
        xc = X(i, :).';
        tc = ctrl(t(i), xc);
        [ta, Vi] = actuator(tc, xc(3), p);
        tau_cmd(i) = tc;
        tau(i)     = ta;
        V(i)       = Vi;
    end

    out = struct('t', t, 'X', X, 'tau_cmd', tau_cmd, 'tau', tau, 'V', V);
end
