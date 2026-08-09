% INITIALIZE_CARS  Generate the initial CAV population.
%
% Builds the CAVs struct array. Identical spawning logic to v2 (random per-path
% flows summing to F_tot, vehicles placed back-to-front with randomized headway,
% initial (x,y,theta) from the lane), with extra fields for the OCBF + DR scheme:
%   abcd        reference cubic coefficients [a;b;c;d] (u=at+b, v=.5at^2+bt+c, ...)
%   tm          reference terminal (MZ-entry) time of the free-terminal-time OC
%   Preceding   same-lane preceding CAV index (rear-end partner), -1 if none
%   ConflictSet struct array of lateral duties: fields .mp (node) and .j (yield-to CAV)
%   Done        true once the CAV has left the control zone
%
% Runs in MAIN's workspace; expects: F_tot, number_of_CAVs, phi, delta.

numLinks = 12;

r     = rand(1, numLinks);
flows = round(F_tot * (r / sum(r)));
delta_round = F_tot - sum(flows);
[~, idx_desc] = sort(r, 'descend');
for k = 1:abs(delta_round)
    flows(idx_desc(k)) = flows(idx_desc(k)) + sign(delta_round);
end

average_speed = 12;
total_flow    = sum(flows);

number_of_vehicles_n_each_path = round(flows / total_flow * number_of_CAVs);
delta_round = number_of_CAVs - sum(number_of_vehicles_n_each_path);
[~, idx_desc] = sort(flows, 'descend');
for k = 1:abs(delta_round)
    number_of_vehicles_n_each_path(idx_desc(k)) = ...
        number_of_vehicles_n_each_path(idx_desc(k)) + sign(delta_round);
end

emptyConflict = struct('mp', {}, 'j', {});

CAVs = struct('v', {}, 'p', {}, 'x', {}, 'y', {}, 'theta', {}, 'u', {}, ...
    'vstart', [], 'abcd', [], 'tm', [], 'path', {}, 't0', [], 'direction', [], ...
    'Passed_control_zone', 0, 'Preceding', {}, 'Done', false, ...
    'Conflict_Points', struct('ID', [], 'time', [], 'distance', [], 'live_distance', []), ...
    'ConflictSet', emptyConflict, 'delta', 0, 'vru_engaged', false, 'vru_wait', 0);

id = 1;
for i = 1:length(number_of_vehicles_n_each_path)
    for j = 1:number_of_vehicles_n_each_path(i)

        s     = average_speed * 3600 / flows(i) + 3 * randn(1, 1);
        speed = average_speed + 3 * abs(randn(1, 1));

        idxs = find([CAVs.path] == i);
        if isempty(idxs)
            CAVs(id).p = 100 + 15 * abs(randn(1, 1));
            CAVs(id).v = speed;
        else
            p_vals   = [CAVs(idxs).p];
            [~, loc] = max(p_vals);
            i_max    = idxs(loc);
            CAVs(id).p = CAVs(i_max).p + s;
            CAVs(id).v = speed;
            if CAVs(i_max).v * phi + delta > CAVs(id).p - CAVs(i_max).p
                error('Really big flow on path %d.', i);
            end
        end

        path = i;
        p    = CAVs(id).p;
        v    = CAVs(id).v;

        switch path
            case 1,  x =  1.75; y = -p;    theta = pi/2;
            case 2,  x =  5.25; y = -p;    theta = pi/2;
            case 3,  x =  8.75; y = -p;    theta = pi/2;
            case 4,  x =  p;    y =  1.75; theta = pi;
            case 5,  x =  p;    y =  5.25; theta = pi;
            case 6,  x =  p;    y =  8.75; theta = pi;
            case 7,  x = -1.75; y =  p;    theta = pi + pi/2;
            case 8,  x = -5.25; y =  p;    theta = pi + pi/2;
            case 9,  x = -8.75; y =  p;    theta = pi + pi/2;
            case 10, x = -p;    y = -1.75; theta = 0;
            case 11, x = -p;    y = -5.25; theta = 0;
            case 12, x = -p;    y = -8.75; theta = 0;
        end

        CAVs(id) = struct('v', v, 'p', p, 'x', x, 'y', y, 'theta', theta, ...
            'u', NaN, 'vstart', [], 'abcd', [], 'tm', NaN, 'path', path, ...
            't0', NaN, 'direction', [], 'Passed_control_zone', 0, ...
            'Preceding', -1, 'Done', false, ...
            'Conflict_Points', struct('ID', [], 'time', [], 'distance', [], 'live_distance', []), ...
            'ConflictSet', emptyConflict, 'delta', 0, 'vru_engaged', false, 'vru_wait', 0);

        id = id + 1;
    end
end

here    = fileparts(mfilename('fullpath'));
dataDir = fullfile(here, '..', 'data');
save(fullfile(dataDir, '_Last_Initial_Conditions.mat'), 'CAVs');
