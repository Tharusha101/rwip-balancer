function d = design_lqr(A, B, Q, R)
%DESIGN_LQR  Infinite-horizon LQR gain for u = -K x, with diagnostics.
%
%   d = DESIGN_LQR(A, B, Q, R) wraps lqr() and returns a struct d with:
%       .K      state-feedback gain (u = -K*x)
%       .S      Riccati solution
%       .poles  closed-loop eigenvalues of (A - B*K)
%       .Acl    closed-loop A matrix
%       .Q,.R   the weights used
%       .ok     true if the closed loop is asymptotically stable
%               (all poles strictly in the open left-half plane)
%       .warn   warning message captured during the lqr solve, if any
%
%   If Q fails to make (A, sqrt(Q)) detectable -- e.g. Q penalises only theta
%   and theta_dot but NOT phi_dot -- the free-wheel mode at the origin is left
%   unregulated and the design will not be asymptotically stable. This wrapper
%   captures that instead of erroring, so the failure can be shown explicitly.

    d.Q = Q;  d.R = R;  d.warn = '';

    lastwarn('');                      % clear, so we can capture lqr warnings
    ws = warning('off', 'all');
    try
        [K, S, P] = lqr(A, B, Q, R);
        d.K = K;  d.S = S;  d.poles = P;
        d.Acl = A - B*K;
        [msg, ~] = lastwarn;
        d.warn = msg;
        d.ok = all(real(eig(d.Acl)) < -1e-9);
    catch ME
        warning(ws);
        d.K = [];  d.S = [];  d.poles = [];  d.Acl = [];
        d.ok = false;  d.warn = ME.message;
        return;
    end
    warning(ws);
end
