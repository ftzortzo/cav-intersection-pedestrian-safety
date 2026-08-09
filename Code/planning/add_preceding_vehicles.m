% ADD_PRECEDING_VEHICLES  Identify CAV i's immediate same-lane predecessor.
%
% Sets CAVs(i).Preceding to the index of the nearest vehicle ahead of i on the
% same path (smallest positive gap in p), or -1 if none. This is the rear-end
% (car-following) partner i_p used by the rear-end CBF. Recomputed every step
% (from update_cavs Pass 1) so the partner stays current as vehicles progress;
% vehicles that have left the zone (Done) are ignored -- their frozen state must
% not drive a follower's rear-end constraint.
%
% Runs in the caller's workspace; expects: i, CAVs, number_of_CAVs.

CAVs(i).Preceding = -1;
min_diff = inf;
for j = 1:number_of_CAVs
    if i == j,                        continue; end
    if CAVs(j).Done,                  continue; end
    if CAVs(i).path ~= CAVs(j).path,  continue; end
    if CAVs(i).p <= CAVs(j).p,        continue; end
    gap = CAVs(i).p - CAVs(j).p;
    if gap < min_diff
        min_diff = gap;
        CAVs(i).Preceding = j;
    end
end
