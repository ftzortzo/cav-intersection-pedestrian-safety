function u = ocbf_controller(CAVs, i, u_ref, v_ref)
%OCBF_CONTROLLER  Optimal-control + control-barrier-function tracking (Paper 1).
%
%   u = OCBF_CONTROLLER(CAVs, i, u_ref, v_ref) returns the acceleration for CAV i
%   that optimally tracks the reference (u_ref, v_ref) while guaranteeing safety,
%   by solving the QP (Paper 1, eq. 17)
%
%       min_{u,e}  (1/2)(u - u_ref)^2 + clf_weight * e^2
%       s.t.  rear-end CBF (eq. 18) w.r.t. CAVs(i).Preceding,
%             lateral  CBF (eq. 19) for each duty in CAVs(i).ConflictSet,
%             speed-limit CBFs (vmax, vmin),
%             CLF speed-tracking (eq. 16, soft via relaxation e),
%             umin <= u <= umax.
%
%   All constraints are linear in the decision z = [u; e], so a single QP is
%   solved per call. Lateral priority (who yields to whom) is supplied by
%   determine_conflict_sets from the dynamic-resequencing order.

global phi delta k_rear k_lat use_clf clf_weight clf_epsilon k_speed
global umin umax vmax vmin

vi = CAVs(i).v;

A = [];
b = [];

% --- Rear-end + lateral CBFs (eqs. 18-19), shared with the emergency QP ---
[Au, blat] = intervehicle_cbf_rows(CAVs, i);
for r = 1:numel(blat)
    A(end+1, :) = [Au(r), 0]; %#ok<AGROW>
    b(end+1, 1) = blat(r);    %#ok<AGROW>
end

% --- Speed-limit CBFs ----------------------------------------------------
A(end+1, :) = [ 1, 0];   b(end+1, 1) =  k_speed * (vmax - vi);   % u <=  k(vmax - v)
A(end+1, :) = [-1, 0];   b(end+1, 1) =  k_speed * (vi - vmin);   % -u <= k(v - vmin)

% --- CLF speed tracking (soft) ------------------------------------------
if use_clf
    A(end+1, :) = [2*(vi - v_ref), -1];
    b(end+1, 1) = 2*(vi - v_ref)*u_ref - clf_epsilon*(vi - v_ref)^2;
end

% --- Solve the QP --------------------------------------------------------
H  = [1, 0; 0, 2*clf_weight];
f  = [-u_ref; 0];
lb = [umin; 0];
ub = [umax; inf];

opts = optimoptions('quadprog', 'Display', 'off');
[z, ~, exitflag] = quadprog(H, f, A, b, [], [], lb, ub, [], opts);

if exitflag <= 0 || isempty(z)
    % Infeasible: the active safety constraints demand more deceleration than
    % the actuator can deliver (u < umin). Apply maximum braking -- this is the
    % minimal-violation action (every violated CBF wants u even more negative)
    % and the only safe response. NEVER fall back to u_ref here: that would
    % ignore all safety constraints and can accelerate into a conflict.
    u = umin;
else
    u = min(max(z(1), umin), umax);
end
end
