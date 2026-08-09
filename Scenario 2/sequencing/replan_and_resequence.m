function CAVs = replan_and_resequence(CAVs, t)
%REPLAN_AND_RESEQUENCE  Replan every in-zone CAV from its current state, then
%   re-order the crossing sequence by the updated control-zone exit time.
%
%   Called whenever resequencing occurs: when a new CAV enters the CZ, and every
%   `replan_interval` seconds. For each CAV currently in the sequence
%   coordinator.S, it re-solves the energy/time-optimal reference
%   (plan_unconstrained_reference) starting from
%       * the CURRENT speed  v_i(t),
%       * the CURRENT time   t  (the new t0), and
%       * the REMAINING distance to the CZ exit, Lk - (p_entry - p),
%   i.e. from the vehicle's actual current state rather than its entry state.
%   The crossing sequence is then sorted by the refreshed t_m (earliest exit =
%   highest priority), which is the resequencing decision.
%
%   p_entry is the value of CAVs(i).p recorded when the CAV entered the CZ;
%   p_entry - p is the arc length travelled since entry (p decreases as the CAV
%   advances), so Lk - (p_entry - p) is the distance still to cover.
%
%   Reads globals: coordinator, length_of_control_zone (and beta via the planner).

global coordinator length_of_control_zone

S = coordinator.S;

% --- Replan each in-zone CAV from its current state ---------------------
for idx = 1:numel(S)
    i = S(idx);
    travelled = CAVs(i).p_entry - CAVs(i).p;            % arc length covered since entry
    Lk_rem    = length_of_control_zone - travelled;     % distance left to the CZ exit
    if Lk_rem > 1                                        % skip if essentially at the exit
        v_now = max(CAVs(i).v, 0.1);
        [~, ~, CAVs] = plan_unconstrained_reference(v_now, t, Lk_rem, CAVs, i);
    end
end

% --- Resequence by the refreshed exit time (+ VRU wait penalty) --------
S  = coordinator.S;
key = arrayfun(@(k) CAVs(k).tm + CAVs(k).vru_wait, S);   % effective MZ-arrival time
[~, order] = sort(key, 'ascend');
coordinator.S = S(order);
end
