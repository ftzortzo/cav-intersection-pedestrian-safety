% INITIALIZE_COORDINATOR  Build the coordinator state for the OCBF + DR scheme.
%
% In this approach the coordinator is a lightweight shared database holding:
%   * the static conflict-node geometry (from intersection_geometry), and
%   * S, the dynamic crossing sequence (priority order) maintained by the
%     dynamic-resequencing routine.
%
% Unlike the v2 reservation scheme, no rear-end/lateral time-window reservations
% are stored: lateral priority is derived from S at runtime (Paper 1, Alg. 2),
% and all safety is enforced online by the OCBF QP controller.

global coordinator

geom = intersection_geometry();

coordinator = struct();
coordinator.conflictpoints     = geom.conflictpoints;
coordinator.conflictpoints_pos = geom.conflictpoints_pos;
coordinator.S                  = [];   % crossing sequence (CAV IDs, highest priority first)

% Timer for periodic resequencing + replanning. The first periodic event fires
% at t = replan_interval; arrivals trigger resequencing independently.
if exist('replan_interval', 'var')
    coordinator.next_replan = replan_interval;
else
    coordinator.next_replan = 3;
end

here    = fileparts(mfilename('fullpath'));
dataDir = fullfile(here, '..', 'data');
save(fullfile(dataDir, 'coordinator.mat'), 'coordinator');
