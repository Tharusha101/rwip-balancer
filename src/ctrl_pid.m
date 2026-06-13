function [u, s] = ctrl_pid(s, y, Ts, cfg)
%CTRL_PID  Baseline PID on arm angle, sampled-controller interface.
%
%   [u, s] = CTRL_PID(s, y, Ts, cfg) computes the commanded wheel torque to
%   hold theta = 0, using ONLY the arm measurements:
%
%       u = Kp*theta + Kd*theta_dot + Ki*Integral(theta dt)
%
%   y   = [theta; theta_dot; phi_dot]  (measured). The derivative term uses the
%         measured rate y(2) (i.e. the gyro), not a numerical difference.
%   s.I = running integral of theta (carried between samples).
%   cfg : .Kp .Kd .Ki and optional .Imax (anti-windup clamp on the integral).
%
%   Deliberately ignores phi_dot: this is the textbook baseline that balances
%   the arm but does NOT regulate wheel speed, so the wheel parks at / drifts to
%   an offset -- the contrast that motivates the LQR phi_dot penalty.

    e   = y(1);                          % angle error about upright (target 0)
    s.I = s.I + e * Ts;                  % integrate
    if isfield(cfg, 'Imax') && ~isempty(cfg.Imax)
        s.I = max(min(s.I, cfg.Imax), -cfg.Imax);   % anti-windup
    end
    u = cfg.Kp*e + cfg.Kd*y(2) + cfg.Ki*s.I;
end
