function [threat, not_passed, vru_ahead] = vru_threat(x, y, Pk, ped, detect_radius)
%VRU_THREAT  Path-aware pedestrian detection for a single vehicle.
%
%   [threat, not_passed, vru_ahead] = VRU_THREAT(x, y, Pk, ped, detect_radius)
%   decides whether the VRU is relevant to a vehicle at (x,y) on reference path
%   Pk:
%       threat     = the VRU disk lies near this vehicle's reference path,
%                    within a preview window AHEAD, and inside the gross
%                    detection radius.
%       not_passed = the vehicle has not yet driven past the VRU's location
%                    along the path (arc-length test).
%       vru_ahead  = path (arc-length) distance from the vehicle to the VRU's
%                    closest point on the path [m] (Inf if not applicable). Used
%                    by the caller to keep coordinating through the intersection
%                    until the VRU is the immediate next event.
%
%   Unlike the previous approach-axis corridor test (|x_ped|<=11 for "vertical
%   road"), this fires for ANY path whose trajectory passes near the VRU --
%   including the post-turn southbound exits of paths 4 and 12 -- and it
%   releases the vehicle once it has driven past the VRU's path location, even
%   for a laterally-crossing VRU (the arc-length gap flips sign on passing,
%   whereas a heading projection never does).

    threat = false;  not_passed = true;  vru_ahead = inf;
    if ~ped.active, return; end
    if hypot(x - ped.x, y - ped.y) > detect_radius, return; end

    global vru_margin vru_preview %#ok<GVMIS>
    if isempty(vru_margin),  vru_margin  = 6;  end   % lateral slack beyond disk [m]
    if isempty(vru_preview), vru_preview = 60; end   % path distance ahead to react [m]
    margin      = vru_margin;
    preview_len = vru_preview;

    d2v = (Pk.xy(:,1) - x).^2     + (Pk.xy(:,2) - y).^2;       [~,  iv ] = min(d2v);
    d2p = (Pk.xy(:,1) - ped.x).^2 + (Pk.xy(:,2) - ped.y).^2;   [dp2, ip] = min(d2p);

    s_proj     = Pk.s(iv);
    s_ped      = Pk.s(ip);
    d_ped_path = sqrt(dp2);              % closest approach of the path to the VRU
    ahead_gap  = s_ped - s_proj;         % path distance: vehicle -> VRU location

    not_passed = ahead_gap >= -(ped.r + 1);
    vru_ahead  = ahead_gap;
    threat     = (d_ped_path <= ped.r + margin) && (ahead_gap <= preview_len);
end
