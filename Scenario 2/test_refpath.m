% TEST_REFPATH  Verify reference_paths + project_to_path.
clc; clear;
root = 'C:\Users\Filippos\Desktop\automatica_matlab_code_no_simulink\Intersection_Controller_v3_OCBF_DR';
addpath(genpath(root));
P = reference_paths();

fprintf('path | #pts | entry (x,y,th) | exit (x,y,th) | length\n');
for k = [1 3 4 8 12]
    fprintf(' %2d  | %4d | (%6.1f,%6.1f,%3.0f) | (%6.1f,%6.1f,%3.0f) | %.1f m\n', ...
        k, size(P(k).xy,1), P(k).xy(1,1),P(k).xy(1,2),mod(rad2deg(P(k).th(1)),360), ...
        P(k).xy(end,1),P(k).xy(end,2),mod(rad2deg(P(k).th(end)),360), P(k).s(end));
end

fprintf('\nProjection tests:\n');
% (a) path 3 straight approach, vehicle pushed +2 m in x (left of north heading is -x, so +x is right)
pr = project_to_path(8.75+2, -40, P(3), 5);
fprintf('p3 @ (10.75,-40): theta=%.0f deg, n=%.2f, ref=(%.2f,%.2f)\n', ...
        mod(rad2deg(pr.theta),360), pr.n, pr.xref, pr.yref);
% (b) path 4 SOUTH exit region (should be heading ~270, centre x<0)
pr = project_to_path(-1.7, -32, P(4), 5);
fprintf('p4 @ (-1.70,-32): theta=%.0f deg, n=%.2f, ref=(%.2f,%.2f)\n', ...
        mod(rad2deg(pr.theta),360), pr.n, pr.xref, pr.yref);
% (c) path 4 vehicle pushed 2 m off its south-exit lane
pr = project_to_path(-3.7, -32, P(4), 5);
fprintf('p4 @ (-3.70,-32): theta=%.0f deg, n=%.2f (off-lane), ref=(%.2f,%.2f)\n', ...
        mod(rad2deg(pr.theta),360), pr.n, pr.xref, pr.yref);
% (d) path 12 south exit
pr = project_to_path(-8.8, -32, P(12), 5);
fprintf('p12@ (-8.80,-32): theta=%.0f deg, n=%.2f, ref=(%.2f,%.2f)\n', ...
        mod(rad2deg(pr.theta),360), pr.n, pr.xref, pr.yref);
