function [tau, V, info] = rwip_motor(tau_cmd, phidot, p)
%RWIP_MOTOR  Brushed-DC motor actuator: realise a commanded wheel torque.
%
%   [tau, V] = RWIP_MOTOR(tau_cmd, phidot, p) returns the torque the motor can
%   actually deliver when asked for tau_cmd while the wheel spins at relative
%   speed phidot. Models back-EMF and a hard supply-voltage saturation +-V_max.
%
%   Electrical model (notes/EOM_derivation.md, sec. 9):
%       tau = Kt*i ,   V = i*R + Ke*phidot
%     => to command tau_cmd the driver must apply
%       V_req = (R/Kt)*tau_cmd + Ke*phidot
%     => after saturating V to [-V_max, V_max] the delivered torque is
%       tau   = (Kt/R)*(V - Ke*phidot)
%
%   Consequences captured: (1) the available torque shrinks as the wheel speeds
%   up (back-EMF eats headroom), and (2) torque saturates -- this is the
%   actuator side of the wheel-saturation problem the LQR phi_dot penalty is
%   there to avoid hitting.
%
%   Optional field p.tau_max (if present & nonempty) applies a further hard
%   torque/current cap on top of voltage saturation.
%
%   info struct (3rd output): .V_req, .saturated (logical).

    V_req = (p.R / p.Kt) * tau_cmd + p.Ke * phidot;     % volts the driver wants
    V     = max(min(V_req, p.V_max), -p.V_max);          % supply saturation
    tau   = (p.Kt / p.R) * (V - p.Ke * phidot);          % torque actually produced

    if isfield(p, 'tau_max') && ~isempty(p.tau_max)
        tau = max(min(tau, p.tau_max), -p.tau_max);
    end

    if nargout > 2
        info.V_req     = V_req;
        info.saturated = (V ~= V_req);
    end
end
