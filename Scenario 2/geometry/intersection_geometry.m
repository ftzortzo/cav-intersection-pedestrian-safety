function geom = intersection_geometry()
%INTERSECTION_GEOMETRY  Static geometric layout of the signal-free intersection.
%
%   geom = INTERSECTION_GEOMETRY() returns a struct of fixed geometric
%   constants: the right-turn arc geometry, the conflict-node (MP) ID lists per
%   path, and the (x,y) position of each conflict node. Single source of truth
%   for the planner, the conflict-set determination, and the coordinator.
%
%   Paths 1,4,7,10 are right-turn lanes; 3,6,9,12 are left-turn lanes;
%   2,5,8,11 are straight-through lanes.

geom.radius = 20;   % right-turn arc radius [m]

geom.center = cell(1, 12);
geom.center{1}  = [-18.25, -18.25];
geom.center{4}  = [ 18.25, -18.25];
geom.center{7}  = [ 18.25,  18.25];
geom.center{10} = [-18.25,  18.25];

geom.initial = cell(1, 12);
geom.initial{1}  = [  1.75, -18.25];
geom.initial{4}  = [ 18.25,   1.75];
geom.initial{7}  = [ -1.75,  18.25];
geom.initial{10} = [-18.25,  -1.75];

% Conflict-node (MP) IDs along each of the 12 paths.
geom.conflictpoints = { ...
    [16 12 10 8], [15 13 7 6], [],          [9 13 14 16], ...
    [6 5 3 2],    [],          [1 5 7 9],   [2 4 10 11],  ...
    [],           [8 4 3 1],   [11 12 14 15], [] };

% (x,y) position of each of the 16 conflict nodes.
geom.conflictpoints_pos = { ...
    [ 0     10.07], [-5.25  5.25], [-3.05  5.25], [-5.25  3.05], ...
    [ 3.05  5.25],  [ 5.25  5.25], [ 5.25  3.05], [-10.07 0   ], ...
    [10.07  0   ],  [-5.25 -3.05], [-5.25 -5.25], [-3.05 -5.25], ...
    [ 5.25 -3.05],  [ 3.05 -5.25], [ 5.25 -5.25], [ 0    -10.07] };
end
