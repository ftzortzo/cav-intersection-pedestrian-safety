% PLOT_INTERSECTION  Draw the static intersection scene for the current frame.
%
% Grass, circular control zone, road surfaces, lane markings, centre lines, and
% the four rounded corners. Signal-free: no traffic-light rendering.

hold on
radius   = 20;
anglePts = linspace(0, 2*pi, 200);

fill([-500 -500 500 500], [-500 500 500 -500], [0.6 0.8 0.5], 'EdgeColor', 'none');

xCZ = 100*cos(anglePts);
yCZ = 100*sin(anglePts);
fill(xCZ, yCZ, [0.9 0.9 1], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
plot(xCZ, yCZ, 'k--', 'LineWidth', 1);

fill([-500 -500 500 500], [-10.5 10.5 10.5 -10.5], [0.5 0.5 0.5], 'EdgeColor', 'none');
fill([-10.5 10.5 10.5 -10.5], [-500 -500 500 500], [0.5 0.5 0.5], 'EdgeColor', 'none');

plot([-600, -radius-10.5], [-10.5 -10.5], 'k', 'LineWidth', 2);
plot([-600, -radius-10.5], [ 10.5  10.5], 'k', 'LineWidth', 2);
plot([ radius+10.5, 600],  [-10.5 -10.5], 'k', 'LineWidth', 2);
plot([ radius+10.5, 600],  [ 10.5  10.5], 'k', 'LineWidth', 2);
plot([-10.5 -10.5], [-600, -radius-10.5], 'k', 'LineWidth', 2);
plot([-10.5 -10.5], [ radius+10.5, 600],  'k', 'LineWidth', 2);
plot([ 10.5  10.5], [-600, -radius-10.5], 'k', 'LineWidth', 2);
plot([ 10.5  10.5], [ radius+10.5, 600],  'k', 'LineWidth', 2);

for y = [3.5 7 -3.5 -7]
    plot([-600, -radius-10.5], [y y], 'w--', 'LineWidth', 0.5);
    plot([ radius+10.5, 600],  [y y], 'w--', 'LineWidth', 0.5);
end
for x = [-3.5 -7 3.5 7]
    plot([x x], [-600, -radius-10.5], 'w--', 'LineWidth', 0.5);
    plot([x x], [ radius+10.5, 600],  'w--', 'LineWidth', 0.5);
end

plot([-600, -radius], [0 0], '-', 'Color', [1 1 0], 'LineWidth', 2);
plot([ radius, 600],  [0 0], '-', 'Color', [1 1 0], 'LineWidth', 2);
plot([0 0], [ radius, 600],  '-', 'Color', [1 1 0], 'LineWidth', 2);
plot([0 0], [-600, -radius], '-', 'Color', [1 1 0], 'LineWidth', 2);

R      = radius;
thetaW = linspace(0, pi/2, 400);
xW0    = R*cos(thetaW);
yW0    = R*sin(thetaW);
sq     = polyshape([0 R R 0], [0 0 R R]);
disk   = polyshape([xW0 0], [yW0 0]);
wedge  = subtract(sq, disk);

cornerPhi = [pi, pi/2, 0, 3*pi/2];
cornerDx  = [ 30.5,  30.5, -30.5, -30.5];
cornerDy  = [ 30.5, -30.5, -30.5,  30.5];
% NOTE: use local names that cannot collide with the global parameters.
% Reusing `phi` here as a rotation angle previously overwrote the GLOBAL
% reaction time (phi) in the caller's workspace, corrupting the CBFs.
for kc = 1:4
    ang = cornerPhi(kc);  dx = cornerDx(kc);  dy = cornerDy(kc);
    [xw, yw] = boundary(wedge);
    xr = xw*cos(ang) - yw*sin(ang) + dx;
    yr = xw*sin(ang) + yw*cos(ang) + dy;
    fill(xr, yr, [0.5 0.5 0.5], 'EdgeColor', 'none');
    xC2 = xW0*cos(ang) - yW0*sin(ang) + dx;
    yC2 = xW0*sin(ang) + yW0*cos(ang) + dy;
    plot(xC2, yC2, 'k', 'LineWidth', 2);
end
