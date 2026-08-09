% MAIN  Signal-free intersection coordination — OCBF + dynamic resequencing.
%
% Top-level driver for the v3 approach:
%   * Reference  : unconstrained energy+time-optimal trajectory, free terminal
%                  time  (Paper 1, Xu et al. 2022, eq. 14).
%   * Controller : OCBF QP tracking that reference with rear-end + lateral CBFs,
%                  speed limits, and a soft CLF  (Paper 1, eqs. 16-19).
%   * Sequencing : dynamic resequencing on each arrival  (Paper 2, Zhang &
%                  Cassandras 2018) -> crossing order -> lateral conflict
%                  priorities  (Paper 1, Algorithm 2).
%
% Project layout
%   config/      simulation_config              parameters (single source of truth)
%   geometry/    intersection_geometry          static layout + conflict nodes
%   setup/       initialize_coordinator         coordinator + empty crossing seq S
%                initialize_cars                initial CAV population
%   planning/    plan_unconstrained_reference   free-terminal-time OC reference
%                update_arrival_times_...        conflict-node distances/times
%                add_preceding_vehicles         rear-end partner lookup
%   control/     ocbf_controller                the OCBF QP
%                cav_steering_update            steering (bicycle model)
%   sequencing/  dynamic_resequencing           insert-into-crossing-order (DR)
%                determine_conflict_sets        Algorithm 2: order -> duties
%   dynamics/    update_cavs                    per-step update + render
%   viz/         plot_intersection, plot_cav
%   data/        regenerated .mat files
%
% See README.md for the full method description, paper mapping, assumptions,
% and tuning notes.

clc;
clear;
close all;

% --- Put the whole project tree on the MATLAB path -----------------------
projectRoot = fileparts(mfilename('fullpath'));
addpath(genpath(projectRoot));

% Crossing sequence lives in the global coordinator, shared with the
% resequencing / conflict-set functions and the in-loop scripts.
global coordinator %#ok<NUSED>

% --- Configuration, coordinator, and vehicle population ------------------
simulation_config        % all parameters into this workspace
initialize_coordinator   % coordinator + empty crossing sequence S

% Set true to REPLAY the last run's initial conditions instead of generating a
% new random population. They are loaded from data/_Last_Initial_Conditions.mat,
% which initialize_cars writes at the end of every run. Because the dynamics are
% deterministic, replaying the same initial conditions reproduces the run
% exactly. NOTE: initialize_cars OVERWRITES that file each run, so to keep a
% particular scenario, copy the file under another name and load that instead.
reuse_last_ICs = true;
if reuse_last_ICs
    load(fullfile(projectRoot, 'data', '_Last_Initial_Conditions.mat'), 'CAVs');
    number_of_CAVs = numel(CAVs);   % keep the loop bound consistent with the loaded set
    if ~isfield(CAVs, 'vru_engaged') % older IC files predate the VRU field
        [CAVs.vru_engaged] = deal(false);
    end
    if ~isfield(CAVs, 'vru_wait')    % VRU resequencing penalty field
        [CAVs.vru_wait] = deal(0);
    end
else
    initialize_cars      % CAVs struct array (also saves _Last_Initial_Conditions.mat)
end

initialize_pedestrian    % crossing VRU (uses vru_radius from simulation_config)

% --- Figure / axes (fixed 800x800, no auto-resize) ----------------------- 
fig = figure('Units', 'pixels', 'Position', [100, 100, 800, 800]);
set(fig, 'Renderer', 'opengl');
ax = axes('Parent', fig, 'Units', 'normalized', 'Position', [0 0 1 1]);
axis(ax, 'equal');
xlim(ax, [-110, 110]);
ylim(ax, [-110, 110]);
axis(ax, 'manual');
set(ax, 'LooseInset', [0 0 0 0]);

% --- Video Recording Setup ------------------------------------------------
video_filename = 'intersection_simulation.mp4'; % Output filename
myVideo = VideoWriter(video_filename, 'MPEG-4'); % Use MPEG-4 for high compatibility
myVideo.FrameRate = 30;                         % Adjust frame rate as needed
open(myVideo);                                  % Open the file for writing

% --- Simulation loop -----------------------------------------------------
for t = 0:dt:Simulation_Time
    cla;                  % clear the previous frame
    plot_intersection     % static scene
    update_cavs           % resequence (on arrival), OCBF-track, integrate, draw
    drawnow limitrate;
    
    % --- Capture and write the current frame -----------------------------
    frame = getframe(gcf); % Captures the current figure window
    writeVideo(myVideo, frame);
    
    pause(0.01);
end

% --- Close Video Writer --------------------------------------------------
close(myVideo); % Finalize and save the video file
disp(['Simulation video saved as: ', video_filename]);