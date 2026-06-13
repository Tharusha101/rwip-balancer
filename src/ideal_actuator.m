function [tau, V] = ideal_actuator(tau_cmd, phidot, p) %#ok<INUSD>
%IDEAL_ACTUATOR  Pass-through "perfect torque source" actuator.
%
%   [tau, V] = IDEAL_ACTUATOR(tau_cmd, phidot, p) returns tau = tau_cmd and
%   V = NaN. Same call signature as rwip_motor so the two are interchangeable
%   in simulate_rwip; use this to see the LQR design against an ideal actuator
%   before the realistic motor limits are imposed.

    tau = tau_cmd;
    V   = NaN;
end
