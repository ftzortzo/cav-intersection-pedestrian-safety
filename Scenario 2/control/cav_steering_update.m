% CAV_STEERING_UPDATE  Set the steering angle newdelta for vehicle i.
%
% Right-turn paths (1,4,7,10) and left-turn paths (3,6,9,12) follow a fixed arc
% of radius `radius` while inside their turning boxes; outside the box, and on
% all straight-through paths (2,5,8,11), the steering angle is zero. Uses the
% bicycle relation delta = atan(curvature * wheelbase), curvature +1/radius for
% right turns and -1/radius for left turns.
%
% Runs in the caller's workspace; expects: CAVs, i, L (wheelbase). Produces newdelta.

radius = 20;
x = CAVs(i).x;
y = CAVs(i).y;

switch CAVs(i).path
    case 1,  newdelta = arc_delta(y >= -18.25 && x >= -18.25,  1/radius, L);
    case 4,  newdelta = arc_delta(y >= -18.25 && x <=  18.25,  1/radius, L);
    case 7,  newdelta = arc_delta(y <=  18.25 && x <=  18.25,  1/radius, L);
    case 10, newdelta = arc_delta(y <=  18.25 && x >= -18.25,  1/radius, L);
    case 3,  newdelta = arc_delta(y >= -28.75 && x <=  28.75, -1/radius, L);
    case 6,  newdelta = arc_delta(y <=  28.75 && x <=  28.75, -1/radius, L);
    case 9,  newdelta = arc_delta(y <=  28.75 && x >= -28.75, -1/radius, L);
    case 12, newdelta = arc_delta(y >= -28.75 && x >= -28.75, -1/radius, L);
    otherwise, newdelta = 0;   % straight-through paths 2, 5, 8, 11
end


function d = arc_delta(in_curve, k, L)
%ARC_DELTA  Steering angle on the arc when inside the turning box, else 0.
if in_curve
    d = atan(k * L);
else
    d = 0;
end
end
