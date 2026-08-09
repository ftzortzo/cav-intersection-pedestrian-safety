function [u1, u2, e] = emergency_cbf_controller(x, y, theta, v, mode, ped, lane, u_ref, prm, Au_iv, b_iv, u1_pp)
%EMERGENCY_CBF_CONTROLLER  VRU-avoidance QP on the bicycle kinematic model.
%
%   [u1,u2] = EMERGENCY_CBF_CONTROLLER(...,Au_iv,b_iv,u1_pp) returns the
%   control-affine inputs u1 = tan(delta) (steering) and u2 = a (acceleration)
%   for a CAV that must avoid a pedestrian while following its reference path,
%   staying on the road, and respecting inter-vehicle safety, by solving the QP
%
%     min  steer_weight*(u1-u1_pp)^2 + (u2-u_ref)^2 + w*e^2
%          + rho_road*d_road^2 + rho_v*d_v^2
%     s.t. VRU HOCBF                         (HARD: disk unsafe set, rel. deg. 2),
%          rear-end + lateral CBFs           (HARD: Au_iv*u2 <= b_iv),
%          road-boundary HOCBFs              (SOFT, slack d_road >= 0),
%          speed-limit + curvature-speed CBF (SOFT, slack d_v >= 0),
%          lane-centre recovery CLF          (soft, slack e),
%          |u1| <= tan(delta_max(v)),  umin <= u2 <= umax.
%
%   Decision z = [u1; u2; e; d_road; d_v]. Only the genuinely safety-critical
%   constraints (pedestrian avoidance, vehicle-vehicle safety) are HARD; the
%   road-keeping, speed-limit and curvature constraints are SOFT with large
%   penalties. This keeps the QP feasible: rather than dropping all steering when
%   a hard road/speed bound momentarily conflicts with the steering limit Tmax(v)
%   (which forced a brake-straight fallback and made turning CAVs run wide), the
%   solver yields a small, penalised, bounded violation while still avoiding the
%   pedestrian and other vehicles.
%
%   u1_pp = tan(delta) of the pure-pursuit steering that FOLLOWS the reference
%   path (including its turns); the steering objective is centred on it so the
%   CAV tracks its curving path by default. Through a turn (lane.curved) the
%   road boundary is a path-tube on the signed offset n (which follows the road
%   across the eastbound->southbound type transition); on a straight it is the
%   axis-aligned half-road band, to which the tube rows reduce exactly.
%
%   mode = 'emergency' : w = 0, longitudinal reference removed (u_ref ignored),
%                        so safety dominates while the VRU is ahead.
%   mode = 'recovery'  : w = w_recover, u_ref tracked, CLF realigns to lane.
%
%   prm carries: sigma (wheelbase), vmax, vmin, umin, umax, k_speed,
%   delta_max0, steer_weight, w_recover, optional v_curve, and the configurable
%   class-K gains k_vru1/k_vru2 (VRU), k_road1/k_road2 (road boundary),
%   k_over1/k_over2 (no-overshoot), k_clf1/k_clf2 (recovery CLF), k_curve
%   (curvature speed cap), and soft-constraint penalties rho_road, rho_v.
%   For a relative-degree-2 barrier b the HOCBF condition is
%       b'' + (k1+k2) b' + (k1*k2) b >= 0   (k1=k2=1 -> the previous 2,1 form).

    sigma = prm.sigma;
    c = cos(theta);  s = sin(theta);

    % Decision vector z = [u1; u2; e; d_road; d_v].
    %   col 4 (d_road): slack shared by the road-boundary rows
    %   col 5 (d_v)   : slack shared by the speed-limit / curvature rows
    A = zeros(0, 5);
    b = zeros(0, 1);

    % ---- VRU avoidance HOCBF (disk unsafe set), relative degree 2 [HARD] -
    % b5 = (x-xp)^2 + (y-yp)^2 - r^2 >= 0
    dx = x - ped.x;   dy = y - ped.y;
    cu1 = (2*v^2/sigma) * (-dx*s + dy*c);
    cu2 = 2 * (dx*c + dy*s);
    con = 2*v^2 + (prm.k_vru1+prm.k_vru2)*2*v*(dx*c + dy*s) ...
              + prm.k_vru1*prm.k_vru2*(dx^2 + dy^2 - ped.r^2);
    A(end+1, :) = [-cu1, -cu2, 0, 0, 0];   b(end+1, 1) = con;

    % ---- Road-boundary HOCBFs [SOFT: slack d_road] -----------------------
    % Straight segment: axis-aligned half-road band (tight, exact). Through a
    % turn: a path-tube on the signed offset n, which follows the curving centre-
    % line across the segment transition -- the axis-aligned band of the CURRENT
    % segment would otherwise forbid moving into the next segment (e.g. a path-12
    % CAV could not leave the eastbound band to turn south). The tube rows reduce
    % exactly to the axis-aligned rows on a straight (nhat = [0,1] or [1,0]).
    kr = prm.k_road1 + prm.k_road2;   krr = prm.k_road1 * prm.k_road2;
    if lane.curved
        nx = -sin(lane.theta_nom);  ny = cos(lane.theta_nom);   % path left-normal
        a  = nx*c + ny*s;           e_ = -nx*s + ny*c;
        tube = lane.tube_half;
        % b = n + tube >= 0   (n >= -tube)
        A(end+1, :) = [-(v^2/sigma)*e_, -a, 0, -1, 0];  b(end+1, 1) =  kr*v*a + krr*(lane.n + tube);
        % b = tube - n >= 0   (n <= +tube)
        A(end+1, :) = [ (v^2/sigma)*e_,  a, 0, -1, 0];  b(end+1, 1) = -kr*v*a + krr*(tube - lane.n);
    elseif lane.axis == 'y'          % travelling along y -> bound x in [lo,hi]
        A(end+1, :) = [ (v^2/sigma)*s, -c, 0, -1, 0];   b(end+1, 1) =  kr*v*c + krr*(x - lane.lo);
        A(end+1, :) = [-(v^2/sigma)*s,  c, 0, -1, 0];   b(end+1, 1) = -kr*v*c + krr*(lane.hi - x);
    else                             % travelling along x -> bound y in [lo,hi]
        A(end+1, :) = [-(v^2/sigma)*c, -s, 0, -1, 0];   b(end+1, 1) =  kr*v*s + krr*(y - lane.lo);
        A(end+1, :) = [ (v^2/sigma)*c,  s, 0, -1, 0];   b(end+1, 1) = -kr*v*s + krr*(lane.hi - y);
    end

    % ---- No-overshoot barrier (recovery only) [SOFT: slack d_road] -------
    % Forbid crossing the lane centre during realignment, for a smooth monotone
    % return to the nominal path (paper remark on overshoot).
    if strcmp(mode, 'recovery')
        ko = prm.k_over1 + prm.k_over2;   kor = prm.k_over1 * prm.k_over2;
        if lane.axis == 'y'
            if x > lane.center
                A(end+1, :) = [ (v^2/sigma)*s, -c, 0, -1, 0];  b(end+1, 1) =  ko*v*c + kor*(x - lane.center);
            elseif x < lane.center
                A(end+1, :) = [-(v^2/sigma)*s,  c, 0, -1, 0];  b(end+1, 1) = -ko*v*c + kor*(lane.center - x);
            end
        else
            if y > lane.center
                A(end+1, :) = [-(v^2/sigma)*c, -s, 0, -1, 0];  b(end+1, 1) =  ko*v*s + kor*(y - lane.center);
            elseif y < lane.center
                A(end+1, :) = [ (v^2/sigma)*c,  s, 0, -1, 0];  b(end+1, 1) = -ko*v*s + kor*(lane.center - y);
            end
        end
    end

    % ---- Speed-limit CBFs (act on u2) [SOFT: slack d_v] ------------------
    % vmin is soft so the CAV can brake to a stop for the pedestrian (a hard vmin
    % CBF forbids braking near v=vmin and was a major infeasibility source).
    A(end+1, :) = [0,  1, 0, 0, -1];   b(end+1, 1) = prm.k_speed * (prm.vmax - v);
    A(end+1, :) = [0, -1, 0, 0, -1];   b(end+1, 1) = prm.k_speed * (v - prm.vmin);

    % ---- Curvature-limited speed CBF [SOFT: slack d_v] -------------------
    % Slow enough to steer the previewed curve within Tmax(v); soft so it biases
    % toward slowing without ever forcing infeasibility.
    if isfield(prm, 'v_curve')
        A(end+1, :) = [0, 1, 0, 0, -1];   b(end+1, 1) = prm.k_curve * (prm.v_curve - v);
    end

    % ---- Lane-centre recovery CLF (soft: LHS <= e) -----------------------
    if lane.axis == 'y'
        xref = lane.center;  yref = y;       % realign laterally, keep moving in y
    else
        xref = x;            yref = lane.center;
    end
    ex = x - xref;   ey = y - yref;
    cu1c = (2*v^2/sigma) * (-ex*s + ey*c);
    cu2c = 2 * (ex*c + ey*s);
    conc = 2*v^2 + (prm.k_clf1+prm.k_clf2)*2*v*(ex*c + ey*s) ...
               + prm.k_clf1*prm.k_clf2*(ex^2 + ey^2);
    A(end+1, :) = [cu1c, cu2c, -1, 0, 0];   b(end+1, 1) = -conc;

    % ---- Inter-vehicle CBFs (rear-end + lateral), act on u2 [HARD] -------
    for r = 1:numel(b_iv)
        A(end+1, :) = [0, Au_iv(r), 0, 0, 0]; %#ok<AGROW>
        b(end+1, 1) = b_iv(r);                %#ok<AGROW>
    end

    % ---- Objective weights and reference ---------------------------------
    if strcmp(mode, 'emergency')
        w = 0;             uref = u_ref;   % safety dominates; no longitudinal track
    else
        w = prm.w_recover; uref = u_ref;   % recovery: track reference + realign
    end
    rho_road = prm.rho_road;   rho_v = prm.rho_v;   % soft-constraint penalties
    H = diag([2*prm.steer_weight, 2, 2*(w + 1e-4), 2*rho_road, 2*rho_v]);
    f = [-2*prm.steer_weight*u1_pp; -2*uref; 0; 0; 0];

    % ---- Speed-dependent steering limit as bounds ------------------------
    Tmax = tan(abs(prm.delta_max0 * (1 - v/prm.vmax)));
    lb = [-Tmax; prm.umin; -inf; 0; 0];
    ub = [ Tmax; prm.umax;  inf; inf; inf];

    opts = optimoptions('quadprog', 'Display', 'off');
    [z, ~, exitflag] = quadprog(H, f, A, b, [], [], lb, ub, [], opts);

    if exitflag <= 0 || isempty(z)
        % Still infeasible (hard constraints conflict, e.g. required braking below
        % umin): brake hard but KEEP following the path -- do not drop steering,
        % or a turning CAV would run straight off its path.
        u1 = max(-Tmax, min(Tmax, u1_pp));   u2 = prm.umin;   e = 0;
    else
        u1 = z(1);  u2 = z(2);  e = z(3);
    end
end
