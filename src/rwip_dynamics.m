function dx = rwip_dynamics(t, x, tau, p)
%RWIP_DYNAMICS  Nonlinear state derivative of the reaction-wheel inverted pendulum.
%
%   dx = RWIP_DYNAMICS(t, x, tau, p) returns d/dt of the state
%
%       x = [theta; theta_dot; phi_dot]
%
%   for motor torque tau [N*m] and parameter struct p (see RWIP_PARAMS).
%   The signature matches ode45's expected @(t,x) form once tau and p are
%   bound, e.g.  ode45(@(t,x) rwip_dynamics(t,x,0,p), tspan, x0).
%
%   State / sign conventions (see notes/EOM_derivation.md):
%     theta      arm angle from UPRIGHT, +CCW [rad]   (theta=0 is inverted/up)
%     theta_dot  arm angular rate              [rad/s]
%     phi_dot    wheel speed RELATIVE TO THE ARM, +CCW [rad/s]
%     tau        motor torque applied to the wheel; reaction -tau acts on arm [N*m]
%
%   The wheel angle phi is deliberately NOT a state: it appears nowhere in
%   the dynamics (the model has no phi dependence), so integrating it would
%   add a redundant, drift-prone state.
%
%   Equations of motion (full derivation in notes/EOM_derivation.md):
%       I_p*theta_ddot = mgl*g*sin(theta) - b_theta*theta_dot - tau + b_w*phi_dot
%       I_w*(theta_ddot + phi_ddot) = tau - b_w*phi_dot
%   solved below for theta_ddot and phi_ddot.

    theta     = x(1);
    theta_dot = x(2);
    phi_dot   = x(3);

    % Gravity torque about the pivot. Destabilising at theta = 0 (upright):
    % for small +theta this is +mgl*g*theta, pushing theta further from 0.
    grav = p.mgl * p.g * sin(theta);

    % Arm angular acceleration.  The motor torque tau enters with a MINUS
    % sign here: spinning the wheel +CCW drives the arm -CW (Newton's 3rd law).
    theta_ddot = (grav - p.b_theta*theta_dot - tau + p.b_w*phi_dot) / p.I_p;

    % Wheel angular acceleration relative to the arm.
    phi_ddot = (tau - p.b_w*phi_dot) / p.I_w - theta_ddot;

    dx = [theta_dot; theta_ddot; phi_ddot];
end
