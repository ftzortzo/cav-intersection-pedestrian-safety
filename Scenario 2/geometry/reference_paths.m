function P = reference_paths()
%REFERENCE_PATHS  Nominal trajectory polyline for each of the 12 paths.
%
%   P = REFERENCE_PATHS() returns a 1x12 struct array (cached) with fields
%       P(k).xy    : M-by-2 points [x y] of the nominal centre-line of path k
%       P(k).th    : M-by-1 path tangent heading [rad] at each point
%       P(k).s     : M-by-1 cumulative arc length [m] from the entry
%       P(k).kappa : M-by-1 path curvature magnitude |d(theta)/ds| [1/m]
%
%   The polyline is produced by integrating the same open-loop steering law as
%   cav_steering_update (fixed radius-20 arcs inside the turn boxes, straight
%   otherwise) from each path's spawn pose, at constant speed. It is therefore
%   the exact nominal path the normal-mode controller tracks, so the emergency
%   controller's projection-based recovery is consistent with normal driving.
%
%   Computed once and cached (persistent); cleared by `clear`/`clear all`.

persistent CACHE
if ~isempty(CACHE), P = CACHE; return; end

L = 2; radius = 20; v = 1; ds = 0.05;   % v*dt = ds spacing; v,dt only set step size
dt = ds / v;
p0 = 115;                                % start just outside the control zone

spawn = [ 1.75 -p0  pi/2;   5.25 -p0  pi/2;   8.75 -p0  pi/2; ...
          p0    1.75 pi;     p0    5.25 pi;     p0    8.75 pi; ...
         -1.75  p0  3*pi/2; -5.25  p0  3*pi/2; -8.75  p0  3*pi/2; ...
         -p0   -1.75 0;     -p0   -5.25 0;     -p0   -8.75 0   ];

P = struct('xy', {}, 'th', {}, 's', {});
for k = 1:12
    x = spawn(k,1); y = spawn(k,2); th = spawn(k,3);
    XY = zeros(7000,2); TH = zeros(7000,1); m = 0; entered = false;
    for n = 1:7000
        nd = local_steer(k, x, y, radius, L);
        if abs(nd) < 1e-9
            x = x + dt*cos(th)*v;  y = y + dt*sin(th)*v;
        else
            om = tan(nd)/L;
            x  = x + (1/om)*(sin(th + om*dt*v) - sin(th));
            y  = y + (1/om)*(cos(th) - cos(th + om*dt*v));
            th = th + om*dt*v;
        end
        m = m + 1;  XY(m,:) = [x y];  TH(m) = th;
        d = hypot(x,y);
        if d < 105, entered = true; end
        if entered && d > 113, break; end
    end
    XY = XY(1:m,:);  TH = TH(1:m);
    s = [0; cumsum(hypot(diff(XY(:,1)), diff(XY(:,2))))];
    % Path curvature kappa = |d(theta)/ds| (TH is accumulated, so continuous).
    kappa = zeros(m,1);
    if m >= 3
        kappa(2:end-1) = (TH(3:end) - TH(1:end-2)) ./ max(s(3:end) - s(1:end-2), 1e-6);
        kappa(1) = kappa(2);  kappa(end) = kappa(end-1);
    end
    P(k).xy = XY;  P(k).th = TH;  P(k).s = s;  P(k).kappa = abs(kappa);
end
CACHE = P;
end

function nd = local_steer(path, x, y, radius, L)
    switch path
        case 1,  in=(y>=-18.25 && x>=-18.25); k= 1/radius;
        case 4,  in=(y>=-18.25 && x<= 18.25); k= 1/radius;
        case 7,  in=(y<= 18.25 && x<= 18.25); k= 1/radius;
        case 10, in=(y<= 18.25 && x>=-18.25); k= 1/radius;
        case 3,  in=(y>=-28.75 && x<= 28.75); k=-1/radius;
        case 6,  in=(y<= 28.75 && x<= 28.75); k=-1/radius;
        case 9,  in=(y<= 28.75 && x>=-28.75); k=-1/radius;
        case 12, in=(y>=-28.75 && x>=-28.75); k=-1/radius;
        otherwise, in=false; k=0;
    end
    if in, nd = atan(k*L); else, nd = 0; end
end
