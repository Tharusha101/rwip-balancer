function [E, T, V] = rwip_energy(X, p)
%RWIP_ENERGY  Total mechanical energy of the reaction-wheel inverted pendulum.
%
%   [E, T, V] = RWIP_ENERGY(X, p) returns total energy E = T + V, kinetic
%   energy T and potential energy V for the RWIP.
%
%   X may be a single state (length-3 vector) or an N-by-3 trajectory with
%   each ROW a state [theta, theta_dot, phi_dot] (the shape ode45 returns).
%   E, T, V are then N-by-1.
%
%   Energy model (see notes/EOM_derivation.md):
%     T = 1/2 * I_p * theta_dot^2 + 1/2 * I_w * (theta_dot + phi_dot)^2
%     V = mgl * g * cos(theta)                    (reference: V=0 at horizontal)
%
%   The wheel's KE uses the ABSOLUTE wheel rate psi_dot = theta_dot + phi_dot,
%   since phi_dot is measured relative to the arm.
%
%   With tau = 0 and zero friction, E is conserved; this function is the
%   instrument used by verify_dynamics.m to check that.

    if isvector(X)
        X = X(:).';      % treat a single state as one row
    end

    theta     = X(:,1);
    theta_dot = X(:,2);
    phi_dot   = X(:,3);

    psi_dot = theta_dot + phi_dot;       % absolute wheel rate

    T = 0.5*p.I_p*theta_dot.^2 + 0.5*p.I_w*psi_dot.^2;
    V = p.mgl * p.g * cos(theta);
    E = T + V;
end
