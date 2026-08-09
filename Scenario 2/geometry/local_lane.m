function lane = local_lane(x, y, Pk)
%LOCAL_LANE  Lane frame at the vehicle's CURRENT location along its ref. path.
%
%   lane = LOCAL_LANE(x, y, Pk) projects (x,y) onto the reference path Pk (one
%   element of reference_paths()) and returns the local lane frame used by the
%   emergency controller:
%       lane.axis      'y' (travelling along y) or 'x' (along x)
%       lane.center    centre-line value of the lateral coordinate [m]
%       lane.lo,.hi    admissible lateral band (own half-road, 0.25 m inset)
%       lane.theta_nom local path tangent heading [rad]
%       lane.n         signed lateral offset of (x,y) from the centre line [m]
%       lane.s         arc length of the projection [m]
%       lane.curved    true if a turn lies within ~10 m ahead on the path
%       lane.tube_half path-tube half-width used through a turn [m]
%
%   This is the position-dependent generalisation of lane_geometry: rather than
%   the fixed straight-APPROACH values, it returns the frame valid where the
%   vehicle actually is -- so a path-4 / path-12 vehicle on its post-turn
%   SOUTHBOUND exit correctly reports axis 'y', heading ~3*pi/2, and the west
%   half-road, instead of its (now irrelevant) horizontal approach lane.
%
%   On straight segments the emergency controller uses the axis-aligned [lo,hi]
%   band. Through a turn (lane.curved) the axis-aligned band of the CURRENT
%   segment would forbid moving into the NEXT segment, so the controller instead
%   constrains the signed offset lane.n to +/- lane.tube_half of the curving
%   reference centre-line, which follows the road across the segment transition.

    proj = project_to_path(x, y, Pk, 0);   % lookahead 0 -> nearest centre point
    th = proj.theta;
    lane.theta_nom = th;
    lane.n = proj.n;
    lane.s = proj.s;

    global tube_half %#ok<GVMIS>
    if isempty(tube_half), tube_half = 5; end

    % Is a turn within ~10 m ahead? (curvature preview -> use the path-tube)
    if isfield(Pk, 'kappa')
        sm = Pk.s >= proj.s & Pk.s <= proj.s + 10;
        lane.curved = max([0; Pk.kappa(sm)]) > 0.005;
    else
        lane.curved = false;
    end
    lane.tube_half = tube_half;  % half-width of the turn corridor [m]

    if abs(sin(th)) >= abs(cos(th))          % travelling along y (vertical road)
        lane.axis   = 'y';
        lane.center = proj.xref;             % centre-line x at the nearest point
        if sin(th) >= 0, lane.lo = 0.25;  lane.hi = 10.25;    % north -> east half
        else,            lane.lo = -10.25; lane.hi = -0.25;   % south -> west half
        end
    else                                     % travelling along x (horizontal road)
        lane.axis   = 'x';
        lane.center = proj.yref;             % centre-line y at the nearest point
        if cos(th) < 0, lane.lo = 0.25;  lane.hi = 10.25;     % west  -> north half
        else,           lane.lo = -10.25; lane.hi = -0.25;    % east  -> south half
        end
    end
end
