function [abcd, tm, CAVs] = plan_unconstrained_reference(v0, t0, Lk, CAVs, i)
%PLAN_UNCONSTRAINED_REFERENCE  Energy+time-optimal trajectory, free terminal time.
%
%   Solves the unconstrained decentralized optimal control problem of Paper 1
%   (Xu et al., eq. 14): the trajectory minimizing
%        beta*(tm - t0) + integral_{t0}^{tm} (1/2) u^2 dt
%   subject to x(t0)=0, v(t0)=v0, x(tm)=Lk and the free-terminal-time conditions
%   u(tm)=0 and the transversality  beta - (1/2)b^2 + a*c = 0. The solution is
%        u*(t) = a t + b,  v*(t) = (1/2)a t^2 + b t + c,
%        x*(t) = (1/6)a t^3 + (1/2)b t^2 + c t + d.
%
%   Closed-form solution (no numerical root finder needed). Writing T = tm - t0
%   and using v(t0)=v0, x(t0)=0, u(tm)=0, x(tm)=Lk gives, after shifting to
%   tau = t - t0,
%        c = v0,  d = 0,  b = -a T,  a = 3 (v0 T - Lk) / T^3.
%   Substituting these into the transversality condition eliminates a,b,c,d and
%   leaves a single quartic in the duration T:
%        2*beta*T^4 - 3*v0^2*T^2 + 12*v0*Lk*T - 9*Lk^2 = 0.
%   Its positive real root gives tm = t0 + T exactly (via roots). With tm known,
%   the four boundary/transversality conditions are LINEAR in [a;b;c;d], so the
%   coefficients follow from a single 4x4 solve. This replaces the previous
%   fsolve, removing initial-guess sensitivity and convergence fallbacks.
%
%   Returns abcd = [a;b;c;d] and tm; also stores them on CAVs(i).

global beta

% --- Terminal time: positive real root of the quartic in T = tm - t0 ----
p = [2*beta, 0, -3*v0^2, 12*v0*Lk, -9*Lk^2];
r = roots(p);
T = real(r(abs(imag(r)) < 1e-9 & real(r) > 0));      % candidate durations

if isempty(T)
    T = Lk / max(v0, 0.1);                            % degenerate fallback
elseif numel(T) > 1
    % Quartic can have up to three positive real roots (stationary durations);
    % take the one of least total cost beta*T + (1/2) int u^2 dt.
    J = inf(size(T));
    for q = 1:numel(T)
        a_s  = 3*(v0*T(q) - Lk)/T(q)^3;
        b_s  = -a_s*T(q);
        J(q) = beta*T(q) + (a_s^2*T(q)^3/3 + a_s*b_s*T(q)^2 + b_s^2*T(q))/2;
    end
    [~, q] = min(J);
    T = T(q);
end

tm = t0 + T;

% --- a,b,c,d from the linear conditions with tm known -------------------
%   row 1: v(t0) = v0     row 2: x(t0) = 0
%   row 3: x(tm) = Lk     row 4: u(tm) = 0
A = [ (1/2)*t0^2,  t0,          1, 0;
      (1/6)*t0^3,  (1/2)*t0^2,  t0, 1;
      (1/6)*tm^3,  (1/2)*tm^2,  tm, 1;
       tm,          1,           0, 0];
abcd = A \ [v0; 0; Lk; 0];

CAVs(i).abcd = abcd;
CAVs(i).tm   = tm;
CAVs(i).t0   = t0;
end
