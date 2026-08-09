function lane = lane_geometry(path)
%LANE_GEOMETRY  Lateral lane geometry of the straight approach for each path.
%
%   lane = LANE_GEOMETRY(path) returns a struct describing the nominal lane the
%   vehicle should hold on the straight portion of its approach, used by the
%   emergency VRU-avoidance controller for lane-boundary barriers and the
%   lane-centre recovery CLF:
%
%     lane.axis      'y' if the vehicle travels along y (vertical road, lateral
%                    coordinate is x); 'x' if it travels along x (horizontal
%                    road, lateral coordinate is y).
%     lane.center    lane-centre value of the lateral coordinate [m].
%     lane.lo,.hi    admissible band of the lateral coordinate [m] (road edges,
%                    inset by 0.25 m), so the vehicle stays on its half-road.
%     lane.theta_nom nominal heading on the straight approach [rad].
%
%   Lane centres follow the spawn layout in initialize_cars: inner lanes
%   (right-turn paths 1,4,7,10) at 1.75 m, middle (straight 2,5,8,11) at 5.25 m,
%   outer (left-turn 3,6,9,12) at 8.75 m, signed by travel direction. The band
%   [lo,hi] is the vehicle's own half of the 21 m road (centre line at 0, edge
%   at +/-10.5), inset by 0.25 m.

    switch path
        % ---- vertical road, travelling north (+y): lateral coord = x, east half
        case 1,  lane = mk('y',  1.75,  0.25,  10.25,  pi/2);
        case 2,  lane = mk('y',  5.25,  0.25,  10.25,  pi/2);
        case 3,  lane = mk('y',  8.75,  0.25,  10.25,  pi/2);
        % ---- vertical road, travelling south (-y): lateral coord = x, west half
        case 7,  lane = mk('y', -1.75, -10.25, -0.25,  3*pi/2);
        case 8,  lane = mk('y', -5.25, -10.25, -0.25,  3*pi/2);
        case 9,  lane = mk('y', -8.75, -10.25, -0.25,  3*pi/2);
        % ---- horizontal road, travelling west (-x): lateral coord = y, north half
        case 4,  lane = mk('x',  1.75,  0.25,  10.25,  pi);
        case 5,  lane = mk('x',  5.25,  0.25,  10.25,  pi);
        case 6,  lane = mk('x',  8.75,  0.25,  10.25,  pi);
        % ---- horizontal road, travelling east (+x): lateral coord = y, south half
        case 10, lane = mk('x', -1.75, -10.25, -0.25,  0);
        case 11, lane = mk('x', -5.25, -10.25, -0.25,  0);
        case 12, lane = mk('x', -8.75, -10.25, -0.25,  0);
        otherwise, lane = mk('y', 0, -10.25, 10.25, pi/2);
    end
end

function lane = mk(axis, center, lo, hi, theta_nom)
    lane = struct('axis', axis, 'center', center, 'lo', lo, 'hi', hi, ...
                  'theta_nom', theta_nom);
end
