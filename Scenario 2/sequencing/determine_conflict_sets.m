function CAVs = determine_conflict_sets(CAVs)
%DETERMINE_CONFLICT_SETS  Map the crossing order to per-CAV lateral duties.
%
%   Implements Paper 1's conflict-search (Algorithm 2) over the current crossing
%   sequence coordinator.S. For each CAV i and each conflict node (MP) it has
%   NOT yet crossed, it finds the single CLOSEST higher-priority CAV j (above i
%   in S) that travels on a DIFFERENT path, also passes that node, and has NOT
%   yet crossed it. CAV i must satisfy the lateral safety constraint at that node
%   only with respect to that one j -- not with respect to any other vehicle
%   (conflicts with farther higher-priority CAVs are handled transitively).
%
%   Re-targeting (Paper-2-style coordinator update): because a node is skipped
%   once its target has crossed it (live_distance <= 0), recomputing this every
%   step makes a yielding CAV automatically switch to the next higher-priority
%   CAV at that node when its current target clears it. This function is
%   therefore called every step from update_cavs, not only on arrival/departure.
%
%   Same-path interactions are excluded here: they are pure car-following and are
%   handled by the rear-end CBF via CAVs(i).Preceding.
%
%   Result: CAVs(i).ConflictSet, a struct array with fields .mp (conflict node)
%   and .j (the single CAV to yield to there), at most one entry per node.
%
%   Reads the global coordinator; returns the updated CAVs.

global coordinator
S = coordinator.S;

for pos = 1:numel(S)
    i  = S(pos);
    cs = struct('mp', {}, 'j', {});
    i_nodes = coordinator.conflictpoints{CAVs(i).path};

    for n = i_nodes
        % Skip nodes CAV i has already crossed.
        ii = find(CAVs(i).Conflict_Points.ID == n, 1);
        if isempty(ii) || CAVs(i).Conflict_Points.live_distance(ii) <= 0
            continue;
        end

        % Closest higher-priority, different-path CAV still contesting node n.
        for up = pos-1:-1:1
            j = S(up);
            if CAVs(j).path == CAVs(i).path
                continue;                                  % same path -> rear-end
            end
            if ~ismember(n, coordinator.conflictpoints{CAVs(j).path})
                continue;                                  % j does not pass node n
            end
            jj = find(CAVs(j).Conflict_Points.ID == n, 1);
            if isempty(jj) || CAVs(j).Conflict_Points.live_distance(jj) <= 0
                continue;                                  % j has already crossed n
            end
            cs(end+1) = struct('mp', n, 'j', j); %#ok<AGROW>
            break;                                         % only the closest target
        end
    end

    CAVs(i).ConflictSet = cs;
end
end
