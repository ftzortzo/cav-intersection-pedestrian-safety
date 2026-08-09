function dynamic_resequencing(i, CAVs)
%DYNAMIC_RESEQUENCING  Order the crossing sequence by control-zone exit time.
%
%   Maintains coordinator.S, the crossing sequence (priority order; highest
%   priority = first to exit the control zone). When CAV i enters, it is added
%   and the whole in-zone set is ordered by ascending t_m, where t_m is the time
%   at which the CAV exits the control zone (reaches the MZ entry) under its
%   energy/time-optimal reference, i.e. x*(t_m) = Lk. The SAME definition of t_m
%   is used for every CAV.
%
%   Consequences (matching Paper 2's intent):
%     * Two CAVs entering at the same speed are ordered by entry time, so the
%       earlier one keeps priority -- it never has to yield to a CAV that arrived
%       later. (The previous spread-minimizing surrogate could invert this.)
%     * A CAV that is genuinely closer / faster to the MZ is prioritized, which
%       is the throughput benefit of resequencing over strict FIFO.
%     * Because t_m is fixed at entry and never changes, sorting preserves the
%       relative order of the CAVs already in the zone (Paper 2's requirement).
%
%   Same-lane order is not enforced here: same-lane pairs are pure car-following
%   (handled by the rear-end CBF) and are excluded from the lateral conflict
%   sets, so their relative position in S does not affect lateral safety.
%
%   Reads/writes the global coordinator; reads CAVs (not modified).

global coordinator

S = coordinator.S;
S(S == i) = [];                 % ensure i is listed exactly once
S = [S, i];

tm = arrayfun(@(k) CAVs(k).tm + CAVs(k).vru_wait, S);   % CZ exit time + VRU wait
[~, order] = sort(tm, 'ascend');     % earliest effective exit -> highest priority
coordinator.S = S(order);
end
