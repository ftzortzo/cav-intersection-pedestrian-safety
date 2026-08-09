function [Au, b] = intervehicle_cbf_rows(CAVs, i)
%INTERVEHICLE_CBF_ROWS  Rear-end + lateral CBF constraints for CAV i.
%
%   [Au, b] = INTERVEHICLE_CBF_ROWS(CAVs, i) returns the inter-vehicle safety
%   constraints as  Au * u <= b , where u is the (scalar) acceleration of CAV i.
%   Rows:
%       * rear-end CBF (Paper 1, eq. 18) w.r.t. CAVs(i).Preceding,
%       * lateral  CBF (Paper 1, eq. 19) for each duty in CAVs(i).ConflictSet
%         -- the ORIGINAL formulation, unchanged, so normal coordination is
%            identical to before,
%       * a SUPPLEMENTARY lateral row, added ONLY when the yielder has gotten
%         ahead of its target at a node (z = d_i - d_j < 0). The original row is
%         too soft to recover this pathological case (its v^2 term is divided by
%         the total trip length L_beh, so it loses authority near the node); the
%         supplementary row is a relative-degree-2 HOCBF on z that commands
%         braking proportional to how far ahead the yielder is.
%
%   Factored out of ocbf_controller so the SAME constraints are enforced in both
%   the normal longitudinal QP and the emergency VRU-avoidance QP. In the
%   emergency QP (decision z = [u1;u2;e]) the caller embeds these as [0,Au,0] so
%   a CAV avoiding the pedestrian still respects rear-end and lateral safety.

global phi delta k_rear k_lat umin lam_lat

vi = CAVs(i).v;
Au = zeros(0, 1);
b  = zeros(0, 1);

% --- Rear-end CBF (eq. 18) ----------------------------------------------
ip = CAVs(i).Preceding;
if ip > 0
    z   = CAVs(i).p - CAVs(ip).p;          % spacing (i behind => z > 0)
    vip = CAVs(ip).v;
    Au(end+1, 1) = phi;
    b(end+1, 1)  = (vip - vi) + k_rear * (z - phi*vi - delta);
end

% --- Lateral CBFs (eq. 19), one per yielding duty ------------------------
lam = lam_lat;                             % HOCBF gains for the supplementary row
for c = 1:numel(CAVs(i).ConflictSet)
    mp = CAVs(i).ConflictSet(c).mp;
    j  = CAVs(i).ConflictSet(c).j;

    idx_i = find(CAVs(i).Conflict_Points.ID == mp);
    idx_j = find(CAVs(j).Conflict_Points.ID == mp);
    if isempty(idx_i) || isempty(idx_j), continue; end

    d_i = CAVs(i).Conflict_Points.live_distance(idx_i);   % i's remaining distance
    d_j = CAVs(j).Conflict_Points.live_distance(idx_j);   % j's remaining distance
    if d_i <= 0 || d_j <= 0, continue; end                % node already cleared

    L_beh = CAVs(i).Conflict_Points.distance(idx_i);      % i's total distance to mp
    if L_beh <= 0, continue; end
    p_beh = L_beh - d_i;                                  % distance travelled toward mp
    v_pre = CAVs(j).v;                                    % higher-priority (leading) speed
    v_beh = vi;                                           % i yields (behind)

    % Signed node-arrival gap: z > 0 => i correctly behind; z < 0 => i ahead of
    % the CAV it must yield to.
    z = d_i - d_j;

    % Original lateral CBF row (unchanged) ----------------------------------
    Au(end+1, 1) = phi*(p_beh/L_beh);                                       %#ok<AGROW>
    b(end+1, 1)  = (v_pre - v_beh - (phi/L_beh)*v_beh^2) + ...
                   k_lat*(z - phi*(p_beh/L_beh)*v_beh - delta);             %#ok<AGROW>

    % Supplementary recovery row, only when the yielder is AHEAD (z < 0) -----
    % HOCBF on b_z = z (rel. deg. 2): z'' + (l1+l2) z' + l1*l2 z >= 0 with
    % z' = v_pre - v_beh, z'' = u_pre - u, u_pre approximated by 0 (conservative
    % if the target is itself braking):
    %     u <= (l1+l2)(v_pre - v_beh) + l1*l2 * z
    % which brakes harder the farther ahead the yielder is. Clamped at umin so
    % the row never makes the QP infeasible (max braking is the strongest it can
    % demand).
    if z < 0
        b_supp = lam*2*(v_pre - v_beh) + lam^2 * z;
        Au(end+1, 1) = 1;                         %#ok<AGROW>
        b(end+1, 1)  = max(b_supp, umin);         %#ok<AGROW>
    end
end

% --- Feasibility guard ---------------------------------------------------
% Drop rows with ~zero acceleration authority (cannot be enforced through u,
% and may be degenerate when far from the node), and clamp every remaining row
% so it never demands u < umin: maximal braking is the strongest physical
% action, so a binding duty should saturate at umin rather than render the QP
% infeasible (which previously triggered a brake-straight fallback).
keep = Au > 1e-6;
Au = Au(keep);  b = b(keep);
b  = max(b, Au*umin);
end
