%% qp_feasibility_demo.m
% Feasibility timing for the emergency-mode QP (44) in
% "Handling Uncertainty of Vulnerable Road Users in Coordinating
% Connected and Automated Vehicles at Signal-Free Intersections"
% (Tzortzoglou & Malikopoulos, 2026).
%
% The QP has 3 decision variables z = [u1; u2; e], a quadratic cost
%     (u1 - u_ref)^2 + u2^2 + w * e^2
% and a bounded number of affine inequality constraints:
%     one VRU HOCBF (29), four lane-boundary HOCBFs (38),
%     two steering-angle bounds (40), two longitudinal-accel bounds,
%     one CLF soft constraint (43) coupling the recovery objective to e.
% The total constraint count is O(1) in the number of CAVs; each
% additional detected VRU adds exactly one row of the form (29).
%
% This script assembles and solves such a QP at every control step
% across a 30 s horizon, and reports per-step solve-time statistics.
%
% Requirements: MATLAB with Optimization Toolbox (quadprog).

clear; close all; clc;

%% Parameters (representative values from Section 7)
dt          = 0.02;        % control step [s]
T_sim       = 30;          % horizon [s]
sigma       = 4.5;         % vehicle length [m]
v_max       = 20;          % max speed [m/s]
u_max       = 5;           % max |longitudinal accel| [m/s^2]
r_disk      = 4;           % VRU unsafe-disk radius [m]
delta_max_0 = pi/6;        % max steering angle at zero speed [rad]
w_slack     = 1;           % CLF slack weight

x_left = -1.75;  x_right = 1.75;
y_low  = -1.75;  y_up    = 1.75;

N = round(T_sim / dt);
solve_ms  = zeros(N, 1);
exit_flag = zeros(N, 1);

%% Representative state trajectory (mock; realistic magnitudes)
x = 0; y = 0; th = 0; v = 12;                 % vehicle (x, y, theta, v)
xp = 25; yp = -5; vp = [-0.35, 1.20];          % pedestrian (x_p, y_p, velocity)
u_ref = @(t) 0.5 * sin(0.15 * t);              % smooth reference u1

opts = optimoptions('quadprog', ...
    'Display', 'off', ...
    'Algorithm', 'interior-point-convex', ...
    'MaxIterations', 100);

%% Main loop: assemble and solve
for k = 1:N
    t = (k - 1) * dt;

    % Mock state evolution (representative magnitudes only)
    x  = x  + v * cos(th) * dt;
    y  = y  + v * sin(th) * dt;
    xp = xp + vp(1) * dt;
    yp = yp + vp(2) * dt;

    dx = x - xp;  dy = y - yp;
    ct = cos(th); st = sin(th);

    d_max = max(delta_max_0 * (1 - v / v_max), 1e-2);

    % ---- Cost: (u1 - u_ref)^2 + u2^2 + w * e^2  ->  (1/2) z' H z + f' z ----
    H = 2 * diag([1, 1, w_slack]);
    f = [-2 * u_ref(t); 0; 0];

    % ---- Constraint stack A * z <= b ----
    A = zeros(0, 3);  b = zeros(0, 1);

    % (29) VRU HOCBF from the disk unsafe set
    A(end+1, :) = [ -(2 * v^2 / sigma) * (-dx * st + dy * ct), ...
                    -2 * (dx * ct + dy * st), ...
                     0 ];
    b(end+1, 1) =  2 * v^2 + 4 * v * (dx * ct + dy * st) + dx^2 + dy^2 - r_disk^2;

    % (38) Lane-boundary HOCBFs (four sides)
    A(end+1, :) = [  (v^2 / sigma) * st, -ct, 0 ];  b(end+1, 1) =  2 * v * ct + (x - x_left);
    A(end+1, :) = [ -(v^2 / sigma) * st,  ct, 0 ];  b(end+1, 1) = -2 * v * ct + (x_right - x);
    A(end+1, :) = [ -(v^2 / sigma) * ct, -st, 0 ];  b(end+1, 1) =  2 * v * st + (y - y_low);
    A(end+1, :) = [  (v^2 / sigma) * ct,  st, 0 ];  b(end+1, 1) = -2 * v * st + (y_up - y);

    % (40) Steering-angle bounds: |u1| <= tan(delta_max)
    A(end+1, :) = [  1, 0, 0 ];  b(end+1, 1) = tan(d_max);
    A(end+1, :) = [ -1, 0, 0 ];  b(end+1, 1) = tan(d_max);

    % Longitudinal-accel box: |u2| <= u_max
    A(end+1, :) = [ 0,  1, 0 ];  b(end+1, 1) = u_max;
    A(end+1, :) = [ 0, -1, 0 ];  b(end+1, 1) = u_max;

    % (43) CLF soft constraint on lane-centering (with slack e)
    xr = 0; yr = 0;
    A(end+1, :) = [ (2 * v^2 / sigma) * ((xr - x) * st + (y - yr) * ct), ...
                     2 * ((x - xr) * ct + (y - yr) * st), ...
                    -1 ];
    b(end+1, 1) = -(2 * v^2 + 4 * v * ((x - xr) * ct + (y - yr) * st) ...
                    + (x - xr)^2 + (y - yr)^2);

    % Solve and record timing
    t0 = tic;
    [~, ~, exf] = quadprog(H, f, A, b, [], [], [], [], [], opts);
    solve_ms(k)  = 1000 * toc(t0);
    exit_flag(k) = exf;
end

%% Report
step_ms = 1000 * dt;
fprintf('\n===== Emergency-mode QP feasibility demo =====\n');
fprintf('Steps solved:            %d  (%.1f s horizon)\n', N, T_sim);
fprintf('Decision variables:      3  (u1, u2, e)\n');
fprintf('Inequality constraints:  %d\n', size(A, 1));
fprintf('Control step (dt):       %.2f ms\n', step_ms);
fprintf('Mean solve time:         %.3f ms\n', mean(solve_ms));
fprintf('Median solve time:       %.3f ms\n', median(solve_ms));
fprintf('Max solve time:          %.3f ms\n', max(solve_ms));
fprintf('99th percentile:         %.3f ms\n', quantile(solve_ms, 0.99));
fprintf('Headroom (dt / mean):    %.1fx\n', step_ms / mean(solve_ms));
fprintf('Feasible solves:         %d / %d\n\n', sum(exit_flag == 1), N);

