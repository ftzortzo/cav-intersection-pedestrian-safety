% VRU_TEST_HARNESS  veh20 turn trace + infeasibility count + full validation.
clc; clear; close all;
root = 'C:\Users\Filippos\Desktop\automatica_matlab_code_no_simulink\Intersection_Controller_v3_OCBF_DR';
addpath(genpath(root));
global coordinator EMERG_INFEAS_COUNT
EMERG_INFEAS_COUNT = 0;
simulation_config
initialize_coordinator
load(fullfile(root,'data','_Last_Initial_Conditions.mat'),'CAVs');
number_of_CAVs = numel(CAVs);
if ~isfield(CAVs,'vru_engaged'), [CAVs.vru_engaged]=deal(false); end
if ~isfield(CAVs,'vru_wait'),    [CAVs.vru_wait]=deal(0); end
initialize_pedestrian
RP = reference_paths();
fig=figure('Visible','off'); ax=axes('Parent',fig); hold(ax,'on');
axis(ax,'equal'); xlim(ax,[-110 110]); ylim(ax,[-110 110]);

turn_paths=[1 3 4 6 7 9 10 12]; tv=find(ismember([CAVs.path],turn_paths));
maxdev=zeros(size(tv));
bmin_moving=inf; vmax_viol=0; crossmin=inf; ct=NaN; cij=[0 0]; v3=find([CAVs.path]==3,1);
TR=[]; ok=true; emsg='';
try
  for t=0:dt:24
    cla(ax); update_cavs;
    if ~CAVs(20).Done
      pr=project_to_path(CAVs(20).x,CAVs(20).y,RP(12),0);
      TR(end+1,:)=[t CAVs(20).x CAVs(20).y mod(rad2deg(CAVs(20).theta),360) CAVs(20).v pr.n]; %#ok<AGROW>
    end
    for q=1:numel(tv)
      k=tv(q);
      if CAVs(k).Done || CAVs(k).Passed_control_zone~=1, continue; end
      pr=project_to_path(CAVs(k).x,CAVs(k).y,RP(CAVs(k).path),0);
      maxdev(q)=max(maxdev(q),abs(pr.n));
    end
    if pedestrian.active
      for k=1:number_of_CAVs
        if CAVs(k).Done, continue; end
        b5=(CAVs(k).x-pedestrian.x)^2+(CAVs(k).y-pedestrian.y)^2-pedestrian.r^2;
        if CAVs(k).v>1 && b5<bmin_moving, bmin_moving=b5; end
        if b5<0, vmax_viol=max(vmax_viol,CAVs(k).v); end
      end
    end
    act=find(~[CAVs.Done] & [CAVs.Passed_control_zone]==1);
    for a=1:numel(act), for bb=a+1:numel(act)
        ii=act(a); jj=act(bb);
        if CAVs(ii).path==CAVs(jj).path, continue; end
        d=hypot(CAVs(ii).x-CAVs(jj).x, CAVs(ii).y-CAVs(jj).y);
        if d<crossmin, crossmin=d; ct=t; cij=[ii jj]; end
    end, end
  end
catch ME
  ok=false; emsg=getReport(ME,'basic');
end
fprintf('ran OK: %s %s\n', string(ok), emsg);
fprintf('--- veh20 (path12) turn region ---\n');
sel = TR(:,1)>=11.5 & TR(:,1)<=20 & mod(round(TR(:,1)*100),50)==0;
S=TR(sel,:);
for r=1:size(S,1)
  fprintf('  %5.2f (%6.1f,%6.1f) th=%3.0f v=%5.2f n=%6.2f\n', S(r,1),S(r,2),S(r,3),S(r,4),S(r,5),S(r,6));
end
fprintf('  veh20 max|n| over run = %.2f m\n', max(abs(TR(:,6))));
fprintf('--- turning-path maxDev (controlled phase) ---\n');
for q=1:numel(tv)
  fprintf('  veh%2d p%2d: maxDev=%5.2f m  done=%d\n', tv(q), CAVs(tv(q)).path, maxdev(q), CAVs(tv(q)).Done);
end
fprintf('--- safety ---\n');
fprintf('  EMERG infeasible (brake-straight) steps: %d\n', EMERG_INFEAS_COUNT);
fprintf('  min b5 moving(>1): %.2f ; fastest at b5<0: %.2f\n', bmin_moving, vmax_viol);
fprintf('  closest CROSS-PATH: %.2f m (veh%d[p%d] & %d[p%d] @ t=%.2f)\n', ...
        crossmin,cij(1),CAVs(cij(1)).path,cij(2),CAVs(cij(2)).path,ct);
fprintf('  veh%d(p3) FINAL (%.1f,%.1f) th=%.0f done=%d\n', v3,CAVs(v3).x,CAVs(v3).y,mod(rad2deg(CAVs(v3).theta),360),CAVs(v3).Done);
close(fig);
