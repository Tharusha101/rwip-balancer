function [u, s] = ctrl_lqr(s, y, Ts, cfg) %#ok<INUSL>
%CTRL_LQR  Full-state LQR control law in the sampled-controller interface.
%
%   [u, s] = CTRL_LQR(s, y, Ts, cfg) returns the commanded wheel torque
%   u = -cfg.K * y for the measured state y = [theta; theta_dot; phi_dot].
%
%   Stateless (s is passed through unchanged) so it shares the exact call
%   signature of ctrl_pid, letting simulate_sampled swap control laws without
%   any change to the harness. cfg.K is the (discrete or continuous) gain.

    u = -cfg.K * y(:);
end
