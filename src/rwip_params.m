function p = rwip_params()
%RWIP_PARAMS  Physical parameters for the reaction-wheel inverted pendulum (RWIP).
%
%   p = RWIP_PARAMS() returns a struct of SI parameters plus the derived
%   "effective" coefficients that the equations of motion actually use.
%
%   Bodies
%     - Pendulum arm : mass m_p, COM at distance l from the pivot, own
%                      inertia I_arm about the pivot.
%     - Reaction wheel: mass m_w, centre at distance l_w from the pivot,
%                      spin inertia I_w about its own axis. Driven by a motor
%                      that applies torque tau to the wheel; the equal-and-
%                      opposite reaction -tau acts on the arm.
%
%   Derived coefficients (see notes/EOM_derivation.md)
%     - I_p = I_arm + m_w*l_w^2   effective swing inertia of the structure
%             about the pivot, EXCLUDING the wheel's spin inertia I_w. This
%             is the inertia that sets the free-pendulum swing frequency.
%     - mgl = m_p*l + m_w*l_w      gravitational moment coefficient, so the
%             gravity torque about the pivot is  mgl*g*sin(theta).
%
%   Conventions: theta = arm angle from UPRIGHT (+CCW), theta=0 is inverted.
%   SI units throughout. See README.md.

    g = 9.81;            % gravitational acceleration [m/s^2]

    % ----- Pendulum arm -----
    m_p   = 0.30;        % arm mass [kg]
    l     = 0.10;        % pivot -> arm COM distance [m]
    I_arm = 3.0e-3;      % arm-only inertia about the pivot [kg*m^2]

    % ----- Reaction wheel -----
    m_w   = 0.15;        % wheel mass [kg]
    l_w   = 0.20;        % pivot -> wheel centre distance [m]
    I_w   = 5.0e-4;      % wheel spin inertia about its own axis [kg*m^2]

    % ----- Friction (set to 0 for the Phase-1 conservation checks) -----
    b_theta = 0.0;       % pivot/bearing viscous friction on the arm  [N*m*s]
    b_w     = 0.0;       % wheel-bearing viscous friction (rel. spin) [N*m*s]

    % ----- Brushed DC motor (12 V class; used from Phase 2 onward) -----
    % Sized so stall torque Kt*V_max/R = 0.36 N*m comfortably exceeds the
    % gravity torque mgl*g*sin(theta) over the working range (recoverable angle
    % ~35 deg static), and no-load wheel speed V_max/Ke = 400 rad/s.
    Kt    = 0.030;       % torque constant      [N*m/A]
    Ke    = 0.030;       % back-EMF constant     [V*s/rad]
    R     = 1.0;         % winding resistance    [ohm]
    Lm    = 1.0e-3;      % winding inductance    [H]  (elec. time const Lm/R = 1 ms)
    V_max = 12.0;        % supply voltage limit  [V]

    % ----- Derived effective coefficients used by the EOM -----
    I_p = I_arm + m_w*l_w^2;   % effective swing inertia about pivot (no wheel spin)
    mgl = m_p*l + m_w*l_w;     % gravitational moment coefficient [kg*m]

    p = struct( ...
        'g',g, ...
        'm_p',m_p, 'l',l, 'I_arm',I_arm, ...
        'm_w',m_w, 'l_w',l_w, 'I_w',I_w, ...
        'b_theta',b_theta, 'b_w',b_w, ...
        'Kt',Kt, 'Ke',Ke, 'R',R, 'Lm',Lm, 'V_max',V_max, ...
        'I_p',I_p, 'mgl',mgl);
end
