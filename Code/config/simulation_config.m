% SIMULATION_CONFIG  Central parameters for the OCBF + dynamic-resequencing controller.
%
% Runs in MAIN's workspace. Defines all globals and workspace variables used by
% the planner, the OCBF QP controller, and the resequencing routines.
%
% Method summary:
%   * Reference  : unconstrained energy+time-optimal trajectory, free terminal
%                  time (Paper 1, eq. 14). `beta` is the time weight.
%   * Controller : OCBF QP tracking that reference (Paper 1, eq. 17) with
%                  rear-end + lateral CBFs, control/speed limits, and a soft CLF.
%   * Sequencing : dynamic resequencing on each arrival (Paper 2) -> crossing
%                  order -> lateral conflict priorities (Paper 1, Algorithm 2).

global vmax vmin umax umin
global beta phi delta k_rear k_lat
global use_clf clf_weight clf_epsilon k_speed
global length_of_control_zone coordinator headway
global lam_lat tube_half vru_margin vru_preview

% --- Vehicle dynamic limits --------------------------------------------
umin = -5;        % min acceleration [m/s^2]
umax =  5;        % max acceleration [m/s^2]
vmax = 20;        % max speed [m/s]
vmin = 0.001;     % min speed [m/s] (kept > 0)

% --- Reference trajectory (unconstrained OC, free terminal time) --------
beta = 3;         % time weight in the energy+time objective (eq. 7 / eq. 14).
                  %   larger beta  -> faster (more time-penalizing) trajectories
                  %   beta -> 0    -> pure minimum-energy

% --- Safety / CBF parameters -------------------------------------------
phi    = 0.5;     % reaction time [s]   (safe gap: z >= phi*v + delta)
delta  = 2;       % minimum safety distance [m] (~ vehicle length)
k_rear = 0.1;     % class-K gain for the rear-end CBF (eq. 18)
k_lat  = 0.05;       % class-K gain for the lateral CBF  (eq. 19)

% --- CLF speed tracking (soft constraint, eq. 16) ----------------------
use_clf     = true;   % include the CLF speed-tracking constraint in the QP
clf_weight  = 1;      % weight on the relaxation e^2 in the objective (beta in eq. 17)
clf_epsilon = 1;      % CLF convergence rate (epsilon in eq. 16)

% --- Speed-limit CBFs ---------------------------------------------------
k_speed = 1;      % class-K gain for the vmax / vmin barrier constraints

% --- Vulnerable road user (VRU) avoidance (emergency mode) --------------
vru_detect_radius = 100;    % a CAV reacts to the VRU within this range [m]
vru_radius        = 4;      % unsafe-set disk radius r [m] (margin + footprint)
delta_max0        = pi/6;   % max steering angle at zero speed [rad] (speed-scaled)
steer_weight      = 1;      % weight on steering effort u1^2 in the emergency QP
w_recover         = 1;      % CLF weight driving the return to lane centre

% --- CBF class-K gains --------------------------------------------------
% HOCBF stacking for a relative-degree-2 barrier b:
%     psi0 = b,  psi1 = d/dt(psi0) + k1*psi0,  require  d/dt(psi1) + k2*psi1 >= 0
%   => b'' + (k1+k2) b' + (k1*k2) b >= 0.
% Defaults (k1,k2) = (1,1) reproduce the b'' + 2 b' + b >= 0 form used before.
% Larger gains let a CAV approach the boundary faster (barrier acts later, less
% conservative); smaller gains make it react earlier (more conservative).
% These mirror the existing rear-end (k_rear) and lateral (k_lat) gains.
k_vru1  = 1;   k_vru2  = 1;    % pedestrian-avoidance HOCBF (disk unsafe set)
k_road1 = 1;   k_road2 = 1;    % road-boundary HOCBF (lane band / curving path-tube)
k_over1 = 1;   k_over2 = 1;    % no-overshoot HOCBF (recovery realignment)
k_clf1  = 1;   k_clf2  = 1;    % lane-centre recovery CLF
k_curve = 1;                   % curvature-limited-speed CBF (relative degree 1)
lam_lat = 2;                   % supplementary lateral HOCBF gain, applied only
                               %   when a yielder has gotten ahead of its target
                               %   (z = d_i - d_j < 0); see intervehicle_cbf_rows

% --- Emergency QP soft-constraint penalties ----------------------------
% The emergency QP keeps VRU + inter-vehicle constraints hard and the
% road-boundary / speed constraints soft (slacks d_road, d_v); these weight
% how strongly each soft constraint is enforced.
rho_road = 1e4;   % penalty on road-boundary slack d_road
rho_v    = 1e4;   % penalty on speed-limit / curvature slack d_v

% --- Path-following (pure pursuit) + curvature speed cap ----------------
pp_lookahead_gain = 0.5;   % emergency look-ahead distance Ld = gain*v ...
pp_lookahead_min  = 4;     %   ... clamped to [min, max] [m]
pp_lookahead_max  = 8;
curve_margin      = 0.55;  % steering-authority margin for the curve speed cap
                           %   (smaller -> slower through turns, more margin)
curve_preview     = 30;    % curvature preview distance for the speed cap [m]
tube_half         = 5;     % path-tube half-width through a turn [m]

% --- VRU detection (pedestrian threat test, see vru_threat) ------------
vru_margin  = 6;    % lateral slack beyond the disk radius for early reaction [m]
vru_preview = 60;   % only react to a VRU within this path distance ahead [m]

% --- Control-zone geometry ---------------------------------------------
length_of_control_zone = 200;   % reference horizon distance Lk [m] (= 2*range:
                                % entry -> centre -> exit)
range_of_coordinator   = 100;   % coordinator sensing radius [m] (< spawn distance,
                                % so vehicles coast in across the boundary)

% --- Network ------------------------------------------------------------
number_of_paths = 12;

% --- Resequencing -------------------------------------------------------
headway = 1.0;    % minimum crossing-time separation [s] (legacy; unused by tm sort)
replan_interval = 3;   % [s] period for periodic resequencing + replanning

% --- Vehicle body -------------------------------------------------------
L           = 2;     % wheelbase [m]
rect_length = 2.5;   % drawn vehicle length [m]
rect_width  = 1.2;   % drawn vehicle width  [m]

% --- Road rendering -----------------------------------------------------
road_width  = 10;
road_length = 50;

% --- Demand / population ------------------------------------------------
F_tot           = 3000;
number_of_CAVs  = 30;   % ~2 per path: moderate load (paper-like), not saturated

% --- Time stepping ------------------------------------------------------
dt              = 0.02;
Simulation_Time = 35;
