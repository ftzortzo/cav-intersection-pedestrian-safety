function delta = pure_pursuit_steer(x, y, theta, Pk, L, Ld)
%PURE_PURSUIT_STEER  Steering angle to follow a reference path (pure pursuit).
%
%   delta = PURE_PURSUIT_STEER(x, y, theta, Pk, L, Ld) returns the bicycle-model
%   steering angle that drives the vehicle toward a look-ahead point Ld metres
%   ahead on its reference path Pk. Unlike a position-only CLF (which targets the
%   nearest centre point and cannot hold heading on a curve), pure pursuit
%   re-centres a deviated vehicle, follows the curved sections, and aligns the
%   heading with the path tangent -- which is what recovery after a VRU
%   avoidance maneuver requires, including on the turning paths.

    proj = project_to_path(x, y, Pk, Ld);     % look-ahead target on the path
    gx = proj.xref - x;
    gy = proj.yref - y;

    alpha = atan2(gy, gx) - theta;            % bearing to target rel. to heading
    alpha = atan2(sin(alpha), cos(alpha));    % wrap to (-pi, pi]

    ld = max(hypot(gx, gy), 1e-3);
    delta = atan2(2 * L * sin(alpha), ld);    % pure-pursuit steering law
end
