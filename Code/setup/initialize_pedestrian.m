% INITIALIZE_PEDESTRIAN  Create the crossing pedestrian (VRU).
%
% Places a single pedestrian just east of the southern road and has it walk
% westward across the approach at a typical walking speed. While active and
% within vru_detect_radius of a CAV travelling on the road it is crossing, the
% CAV switches to the emergency CBF controller (see emergency_cbf_controller).
%
% Runs in MAIN's workspace; expects: vru_radius (from simulation_config).

pedestrian = struct( ...
    'x',      13,    ...   % initial x [m] (outside the roadway, east edge)
    'y',     -32,    ...   % initial y [m] (on the southern approach)
    'vx',   -1.32,    ...   % velocity x [m/s] (westward across the road)
    'vy',     0,     ...   % velocity y [m/s]
    'r',  vru_radius,...   % unsafe-set radius r [m]
    'active', true);       % becomes false once the VRU clears the roadway
