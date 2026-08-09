% UPDATE_ARRIVAL_TIMES_AT_CONFLICT_POINTS  Refresh CAV i's MP distances and times.
%
% For each conflict node (MP) on CAV i's path, compute the distance from i's
% current position to the node (arc length for right-turn paths 1,4,7,10,
% Euclidean otherwise; negative once passed) and the time at which i reaches it
% by rooting the reference position polynomial x*(t) = (1/6)a t^3 + (1/2)b t^2 +
% c t + d.
%
% On first call (Passed_control_zone == 0) the per-MP arrays are initialized
% (ID/time/distance, and live_distance seeded to distance). Afterwards only
% live_distance is refreshed.
%
% Runs in the caller's workspace; expects: i, t, CAVs, coordinator.
% Uses a local arc-length variable (arc_len) so the wheelbase L is never touched.

geom              = intersection_geometry();
center_cycle_path = geom.center;
initial_cycle_path = geom.initial;

path_number    = CAVs(i).path;
conflict_nodes = coordinator.conflictpoints{path_number};

abcd = CAVs(i).abcd;
a = abcd(1); b = abcd(2); c = abcd(3); d = abcd(4);

for a_idx = 1:length(conflict_nodes)
    node = conflict_nodes(a_idx);

    if path_number==1 || path_number==4 || path_number==7 || path_number==10
        on_approach = (path_number==1 && CAVs(i).y<=-18.25) || ...
                      (path_number==4 && CAVs(i).x>= 18.25) || ...
                      (path_number==7 && CAVs(i).y>= 18.25) || ...
                      (path_number==10 && CAVs(i).x<=-18.25);
        if on_approach
            uvec = center_cycle_path{path_number} - initial_cycle_path{path_number};
            vvec = (center_cycle_path{path_number} - coordinator.conflictpoints_pos{node})';
            arc_len = 20 * acos(uvec*vvec/(norm(vvec)*norm(uvec)));
            dist = sqrt((CAVs(i).x-initial_cycle_path{path_number}(1))^2 + ...
                        (CAVs(i).y-initial_cycle_path{path_number}(2))^2) + arc_len;
        else
            uvec = center_cycle_path{path_number} - [CAVs(i).x, CAVs(i).y];
            vvec = (center_cycle_path{path_number} - coordinator.conflictpoints_pos{node})';
            arc_len = 20 * acos(uvec*vvec/(norm(vvec)*norm(uvec)));
            dist = arc_len;
        end
    else
        dist = sqrt((CAVs(i).x-coordinator.conflictpoints_pos{node}(1))^2 + ...
                    (CAVs(i).y-coordinator.conflictpoints_pos{node}(2))^2);
    end

    % Negative once the node has been passed.
    vector_to_conflict = [coordinator.conflictpoints_pos{node}(1)-CAVs(i).x, ...
                          coordinator.conflictpoints_pos{node}(2)-CAVs(i).y];
    if dot(CAVs(i).direction, vector_to_conflict) < 0
        dist = -dist;
    end

    % Arrival time: first real root of x*(t) = (current travelled distance + dist).
    % x* is measured from CZ entry; the distance to a node maps to a position
    % target of (current reference position + dist). We solve for the first time
    % after t at which the reference reaches that target.
    target = polyval([a/6, b/2, c, d], t) + dist;
    coeffs = [a/6, b/2, c, d - target];
    t_roots = roots(coeffs);
    positive_real_roots = t_roots(real(t_roots) > t & abs(imag(t_roots)) < 1e-9);
    if isempty(positive_real_roots)
        t_root = t;          % already there / past: clamp to now
    else
        t_root = min(real(positive_real_roots));
    end

    if CAVs(i).Passed_control_zone == 0
        CAVs(i).Conflict_Points.ID            = [CAVs(i).Conflict_Points.ID, node];
        CAVs(i).Conflict_Points.time          = [CAVs(i).Conflict_Points.time, t_root];
        CAVs(i).Conflict_Points.distance      = [CAVs(i).Conflict_Points.distance, dist];
        CAVs(i).Conflict_Points.live_distance = [CAVs(i).Conflict_Points.live_distance, dist];
    else
        index = find(CAVs(i).Conflict_Points.ID == node);
        CAVs(i).Conflict_Points.live_distance(index) = dist;
        CAVs(i).Conflict_Points.time(index)          = t_root;
    end
end
