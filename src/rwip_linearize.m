function [A, B, C, D, sys] = rwip_linearize(p)
%RWIP_LINEARIZE  Linear state-space model of the RWIP about the upright equilibrium.
%
%   [A,B,C,D] = RWIP_LINEARIZE(p) returns the continuous-time linearization of
%   rwip_dynamics about the inverted equilibrium  x = [theta;theta_dot;phi_dot] = 0,
%   tau = 0, with the motor torque tau as the input.
%
%   [A,B,C,D,sys] = RWIP_LINEARIZE(p) also returns an ss() object with named
%   states/inputs (needs Control System Toolbox for ss()).
%
%   Derivation (notes/EOM_derivation.md, sec. 6; linearised with sin theta -> theta):
%       theta_ddot = ( k*theta - b_theta*theta_dot + b_w*phi_dot - tau ) / I_p
%       phi_ddot   = ( tau - b_w*phi_dot ) / I_w  -  theta_ddot
%   where k = mgl*g.  Output C = I (full-state) by default.
%
%   The open loop has poles { +sqrt(k/I_p), -sqrt(k/I_p), 0 }: one unstable
%   (the falling pendulum), one stable, and a marginal integrator (the free
%   wheel speed). That integrator is why the LQR cost MUST penalise phi_dot to
%   regulate wheel speed -- see design_lqr.m / notes/phase2_lqr.md.

    Ip = p.I_p;  Iw = p.I_w;
    bt = p.b_theta;  bw = p.b_w;
    k  = p.mgl * p.g;

    A = [ 0,        1,        0;
          k/Ip,    -bt/Ip,    bw/Ip;
         -k/Ip,     bt/Ip,   -bw*(1/Iw + 1/Ip) ];

    B = [ 0;
         -1/Ip;
          1/Iw + 1/Ip ];

    C = eye(3);
    D = zeros(3,1);

    if nargout > 4
        sys = ss(A, B, C, D);
        sys.StateName  = {'theta','theta_dot','phi_dot'};
        sys.StateUnit  = {'rad','rad/s','rad/s'};
        sys.InputName  = {'tau'};
        sys.InputUnit  = {'N*m'};
        sys.OutputName = {'theta','theta_dot','phi_dot'};
    end
end
