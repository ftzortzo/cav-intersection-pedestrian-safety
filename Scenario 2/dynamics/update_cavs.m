% UPDATE_CAVS  Advance every CAV by one time step under OCBF + dynamic resequencing,
% with emergency CBF-based avoidance of a crossing pedestrian (VRU).
%
% Two passes per step:
%   Pass 1  refresh each active in-zone CAV's distances/times to its conflict
%           nodes (a consistent snapshot used by the lateral CBFs).
%   Pass 2  for each CAV:
%             * outside the CZ (not yet planned): coast straight;
%             * on first entry: plan the unconstrained reference, register its
%               conflict nodes, run dynamic resequencing, recompute conflict sets;
%             * every step inside the CZ: pick a control mode and act --
%                 NORMAL    -> OCBF QP (longitudinal) + path-tracking steering;
%                 EMERGENCY -> bicycle-model VRU-avoidance QP (steering + accel),
%                              when the pedestrian is detected ahead on this road;
%                 RECOVERY  -> same QP with lane-centre CLF, realigning after the
%                              VRU has been passed -- then revert to NORMAL;
%             * integrate the bicycle kinematics;
%             * on leaving the CZ: mark Done and drop from the crossing sequence.
%
% Runs in MAIN's workspace; expects: number_of_CAVs, range_of_coordinator, dt, t,
% L, length_of_control_zone, CAVs, coordinator (global), pedestrian, and the VRU
% parameters vru_detect_radius, delta_max0, steer_weight, w_recover, plus the
% speed/accel limits vmax/vmin/umax/umin and k_speed.

global coordinator

% ---- Advance the pedestrian (VRU) ---------------------------------------
if pedestrian.active
    pedestrian.x = pedestrian.x + pedestrian.vx*dt;
    pedestrian.y = pedestrian.y + pedestrian.vy*dt;
    if pedestrian.x < -13 || pedestrian.x > 13   % cleared the road onto the grass
        pedestrian.active = false;
    end
end

% Shared parameter bundle for the emergency controller.
vru_prm = struct('sigma', L, 'vmax', vmax, 'vmin', vmin, 'umin', umin, ...
                 'umax', umax, 'k_speed', k_speed, 'delta_max0', delta_max0, ...
                 'steer_weight', steer_weight, 'w_recover', w_recover, ...
                 'k_vru1', k_vru1, 'k_vru2', k_vru2, ...
                 'k_road1', k_road1, 'k_road2', k_road2, ...
                 'k_over1', k_over1, 'k_over2', k_over2, ...
                 'k_clf1', k_clf1, 'k_clf2', k_clf2, 'k_curve', k_curve, ...
                 'rho_road', rho_road, 'rho_v', rho_v);

% Nominal reference polylines (cached) -- used for path-aware VRU detection and
% for the position-dependent lane frame in the emergency controller.
RP = reference_paths();

% ---- Pass 1: refresh conflict-node snapshot for active in-zone CAVs -----
for i = 1:number_of_CAVs
    if CAVs(i).Passed_control_zone == 1 && ~CAVs(i).Done
        update_arrival_times_at_conflict_points;
        add_preceding_vehicles;          % refresh rear-end partner (ignores Done)
    end
end

% Periodic resequencing + replanning.
if t + 1e-9 >= coordinator.next_replan
    CAVs = replan_and_resequence(CAVs, t);
    coordinator.next_replan = coordinator.next_replan + replan_interval;
end

% Recompute lateral duties from the fresh snapshot.
CAVs = determine_conflict_sets(CAVs);

% ---- Pass 2: resequencing (on arrival), control, integration ------------
for i = 1:number_of_CAVs

    dctr = sqrt(CAVs(i).x^2 + CAVs(i).y^2);

    if CAVs(i).Done
        % Coast away after clearing the intersection.
        xnew = CAVs(i).x + dt*cos(CAVs(i).theta)*CAVs(i).v;
        ynew = CAVs(i).y + dt*sin(CAVs(i).theta)*CAVs(i).v;
        CAVs(i).x = xnew;  CAVs(i).y = ynew;
        CAVs(i).p = CAVs(i).p - dt*CAVs(i).v;
        plot_cav
        continue;
    end

    if dctr > range_of_coordinator && CAVs(i).Passed_control_zone == 0
        % ---- Outside the control zone: coast straight ----
        thetanew = CAVs(i).theta;
        xnew = CAVs(i).x + dt*cos(thetanew)*CAVs(i).v;
        ynew = CAVs(i).y + dt*sin(thetanew)*CAVs(i).v;
        CAVs(i).direction = [xnew-CAVs(i).x, ynew-CAVs(i).y];
        CAVs(i).direction = CAVs(i).direction / norm(CAVs(i).direction);
        CAVs(i).x = xnew;  CAVs(i).y = ynew;  CAVs(i).theta = thetanew;
        CAVs(i).p = CAVs(i).p - dt*CAVs(i).v;
        CAVs(i).delta = 0;

    else
        % ---- Inside the control zone ----
        if CAVs(i).Passed_control_zone == 0
            % First entry: plan + register + resequence + conflict sets.
            add_preceding_vehicles
            CAVs(i).vstart = CAVs(i).v;
            CAVs(i).direction = [cos(CAVs(i).theta), sin(CAVs(i).theta)];
            t0 = t;
            Lk = length_of_control_zone;
            CAVs(i).p_entry = CAVs(i).p;                 % entry arc-length reference
            [~, ~, CAVs] = plan_unconstrained_reference(CAVs(i).v, t0, Lk, CAVs, i);
            update_arrival_times_at_conflict_points;     % Passed==0 branch: init nodes
            CAVs(i).Passed_control_zone = 1;
            dynamic_resequencing(i, CAVs);               % insert into crossing order
            CAVs = replan_and_resequence(CAVs, t);       % all in-zone replan from current state
            CAVs = determine_conflict_sets(CAVs);        % lateral duties from new order
        end

        % Longitudinal reference from the unconstrained optimal trajectory
        % (clamped past tm so a behind-schedule CAV coasts at terminal speed).
        abcd  = CAVs(i).abcd;
        tau   = min(t, CAVs(i).tm);
        u_ref = abcd(1)*tau + abcd(2);
        v_ref = 0.5*abcd(1)*tau^2 + abcd(2)*tau + abcd(3);

        % ---- Decide control mode w.r.t. the pedestrian ----
        xi = CAVs(i).x;  yi = CAVs(i).y;  thi = CAVs(i).theta;
        Pk   = RP(CAVs(i).path);
        lane = local_lane(xi, yi, Pk);     % frame at the vehicle's actual location
        [threat, not_passed, vru_ahead] = vru_threat(xi, yi, Pk, pedestrian, vru_detect_radius);
        lateral_offset = abs(lane.n);
        herr     = abs(atan2(sin(thi - lane.theta_nom), cos(thi - lane.theta_nom)));
        aligned  = (lateral_offset < 0.5) && (herr < 0.15);   % pursuit aligns heading; snap clears residual

        % VRU detection: engage emergency whenever the VRU threatens this CAV's
        % path ahead and has not been passed. Inter-vehicle (rear-end + lateral)
        % safety is now enforced INSIDE the emergency QP, so emergency mode no
        % longer drops intersection coordination -- the earlier deferral gate is
        % no longer needed (one consistent safety mechanism).
        ld = CAVs(i).Conflict_Points.live_distance;

        if threat && not_passed
            mode = 'emergency';
            CAVs(i).vru_engaged = true;
        elseif CAVs(i).vru_engaged && ~aligned
            mode = 'recovery';
        else
            mode = 'normal';
            if CAVs(i).vru_engaged          % finished recovering: snap heading, resume
                CAVs(i).theta = lane.theta_nom;
                thi = lane.theta_nom;
                CAVs(i).vru_engaged = false;
            end
        end

        % ---- Resequencing penalty for a VRU-blocked CAV ----------------------
        % A CAV whose path to the merging zone is gated by the pedestrian cannot
        % cross until the VRU clears its lane, so it must not keep priority over
        % free-flowing CAVs on other paths while the VRU holds it. The penalty is
        % how much the VRU pushes the CAV's earliest feasible MZ arrival past its
        % (VRU-unaware) optimal exit time tm:
        %     earliest = t + t_clear + max(0, d_node - vru_ahead)/vmax,
        % where t_clear is the time for the VRU to clear this CAV's lane band and
        % d_node is the distance to its nearest uncrossed conflict node. It is
        % derived from pedestrian/path GEOMETRY (not the CAV's instantaneous,
        % possibly chattering, speed), so the CAV is deprioritized as soon as the
        % VRU genuinely blocks it -- not only once it has crawled to a stop, which
        % was too late to stop other CAVs from yielding to it. A fast CAV that
        % would clear the VRU before tm (earliest <= tm) is left unpenalized.
        CAVs(i).vru_wait = 0;
        if pedestrian.active && CAVs(i).vru_engaged && not_passed && ...
                vru_ahead > 0 && any(ld > 0.5 & ld > vru_ahead - 1)
            if lane.axis == 'y', plat = pedestrian.x; pv = pedestrian.vx;
            else,                plat = pedestrian.y; pv = pedestrian.vy;
            end
            if abs(pv) > 1e-3
                band     = pedestrian.r + 6;               % clears when `band` past centre
                target   = lane.center + sign(pv)*band;
                t_clear  = max(0, (target - plat)/pv);     % time for the VRU to clear [s]
                d_node   = min(ld(ld > 0.5));              % nearest uncrossed node [m]
                earliest = t + t_clear + max(0, d_node - vru_ahead)/vmax;
                CAVs(i).vru_wait = max(0, earliest - CAVs(i).tm);
            end
        end

        % ---- Produce (newdelta, unew) for the chosen mode ----
        if strcmp(mode, 'normal')
            unew = ocbf_controller(CAVs, i, u_ref, v_ref);   % longitudinal QP
            if CAVs(i).v < 0.01, CAVs(i).v = 0.1; end
            cav_steering_update;                             % open-loop path steering
        elseif strcmp(mode, 'emergency')
            if CAVs(i).v < 0.01, CAVs(i).v = 0.1; end
            [Au_iv, b_iv] = intervehicle_cbf_rows(CAVs, i);     % rear-end + lateral safety
            Ld   = max(pp_lookahead_min, min(pp_lookahead_max, pp_lookahead_gain*CAVs(i).v));  % look-ahead tracks turns tighter
            u1pp = tan(pure_pursuit_steer(xi, yi, thi, Pk, L, Ld));  % follow the path (turn) by default
            % Curvature-limited speed: be slow enough to steer through the curve
            % ahead within the speed-dependent steering limit, else the QP cannot
            % turn (a CAV must slow for a turn). Preview the max curvature over the
            % next curve_preview m of path and cap the speed, leaving a steering
            % margin (curve_margin) for tracking the turn and for VRU avoidance.
            mask = Pk.s >= lane.s & Pk.s <= lane.s + curve_preview;
            kap  = max([0; Pk.kappa(mask)]);
            dreq = atan(kap * L);                               % steering needed for kap
            vru_prm.v_curve = max(vmin, vmax*(1 - min(dreq/(curve_margin*delta_max0), 1)));
            [u1, u2] = emergency_cbf_controller(xi, yi, thi, CAVs(i).v, mode, ...
                                                pedestrian, lane, u_ref, vru_prm, Au_iv, b_iv, u1pp);
            newdelta = atan(u1);     % recover the physical steering angle
            unew     = u2;
        else   % recovery: re-enter intersection coordination + steer back onto path
            if CAVs(i).v < 0.01, CAVs(i).v = 0.1; end
            unew     = ocbf_controller(CAVs, i, u_ref, v_ref);  % restores lateral CBF
            Ld       = max(4, 0.8*CAVs(i).v);                  % look-ahead distance
            newdelta = pure_pursuit_steer(xi, yi, thi, Pk, L, Ld);
        end

        % ---- Bicycle kinematic integration ----
        if abs(newdelta) < 1e-7
            xnew     = CAVs(i).x + dt*cos(CAVs(i).theta)*CAVs(i).v + unew*cos(CAVs(i).theta)*(dt^2/2);
            ynew     = CAVs(i).y + dt*sin(CAVs(i).theta)*CAVs(i).v + unew*sin(CAVs(i).theta)*(dt^2/2);
            thetanew = CAVs(i).theta;
            vnew     = CAVs(i).v + dt*unew;
        else
            omega    = tan(newdelta) / L;
            xnew     = CAVs(i).x + (1/omega)*(sin(CAVs(i).theta + omega*dt*(CAVs(i).v+0.5*dt*unew)) - sin(CAVs(i).theta));
            ynew     = CAVs(i).y + (1/omega)*(cos(CAVs(i).theta) - cos(CAVs(i).theta + omega*(CAVs(i).v+0.5*dt*unew)*dt));
            thetanew = CAVs(i).theta + omega*dt*(CAVs(i).v+0.5*dt*unew);
            vnew     = CAVs(i).v + dt*unew;
        end

        CAVs(i).delta     = newdelta;
        % Direction of travel from heading. A displacement-based direction flips
        % backward when hard braking nudges a near-stopped vehicle back a hair,
        % which would wrongly sign every conflict-node distance negative ("passed")
        % and corrupt both vru_wait and the lateral conflict sets. Heading is
        % well-defined at v ~ 0 and is the true travel direction of the bicycle
        % model, so it removes that chatter.
        CAVs(i).direction = [cos(thetanew), sin(thetanew)];
        CAVs(i).v     = max(vnew, 0);    % forward-only: a CAV does not roll backward at a stop
        CAVs(i).u     = unew;
        CAVs(i).x     = xnew;
        CAVs(i).y     = ynew;
        CAVs(i).theta = thetanew;
        CAVs(i).p     = CAVs(i).p - dt*CAVs(i).v + 0.5*(dt^2)*CAVs(i).u;

        % Leaving the CZ after clearing all conflict nodes -> retire from S.
        cleared = isempty(CAVs(i).Conflict_Points.live_distance) || ...
                  all(CAVs(i).Conflict_Points.live_distance <= 0);
        if sqrt(xnew^2 + ynew^2) > range_of_coordinator && cleared
            CAVs(i).Done = true;
            coordinator.S(coordinator.S == i) = [];
            CAVs = determine_conflict_sets(CAVs);   % refresh remaining CAVs' duties
        end
    end

    plot_cav
end

% ---- Draw the pedestrian and its unsafe set -----------------------------
if pedestrian.active
    pang = linspace(0, 2*pi, 40);
    plot(pedestrian.x + pedestrian.r*cos(pang), pedestrian.y + pedestrian.r*sin(pang), ...
         'r--', 'LineWidth', 1);
    fill(pedestrian.x + 0.6*cos(pang), pedestrian.y + 0.6*sin(pang), [1 0 1], ...
         'EdgeColor', 'k');
end
