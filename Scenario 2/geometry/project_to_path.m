function proj = project_to_path(x, y, Pk, lookahead)
%PROJECT_TO_PATH  Project a pose onto a reference polyline.
%
%   proj = PROJECT_TO_PATH(x, y, Pk, lookahead) projects the point (x,y) onto
%   the reference path Pk (one element of reference_paths()) and returns:
%       proj.idx      index of the nearest polyline vertex
%       proj.xref     look-ahead target point x  (lookahead metres ahead on path)
%       proj.yref     look-ahead target point y
%       proj.theta    path tangent heading at the projection [rad]
%       proj.n        signed lateral offset of (x,y) from the centre line [m]
%                     (left of the path tangent is positive)
%       proj.s        arc length of the projection from the entry [m]
%
%   The signed offset uses the left normal nhat = [-sin(theta), cos(theta)], so
%   n > 0 means the vehicle is to the left of its nominal path.

    if nargin < 4, lookahead = 0; end

    d2 = (Pk.xy(:,1) - x).^2 + (Pk.xy(:,2) - y).^2;
    [~, idx] = min(d2);

    th = Pk.th(idx);
    nhat = [-sin(th), cos(th)];                 % left normal to the tangent
    rel  = [x - Pk.xy(idx,1), y - Pk.xy(idx,2)];
    n    = rel * nhat.';                         % signed perpendicular offset

    % look-ahead target: advance `lookahead` metres along the path from idx
    s_target = Pk.s(idx) + lookahead;
    j = find(Pk.s >= s_target, 1, 'first');
    if isempty(j), j = numel(Pk.s); end

    proj.idx   = idx;
    proj.theta = th;
    proj.n     = n;
    proj.s     = Pk.s(idx);
    proj.xref  = Pk.xy(j,1);
    proj.yref  = Pk.xy(j,2);
end
