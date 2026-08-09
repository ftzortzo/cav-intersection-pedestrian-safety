% GEN_REFPATHS_DIAG  Integrate each path's nominal trajectory; report which ones
% pass through the pedestrian crossing region (y in [-36,-28], |x|<=13).
clc; clear;
root = 'C:\Users\Filippos\Desktop\automatica_matlab_code_no_simulink\Intersection_Controller_v3_OCBF_DR';
addpath(genpath(root));
L=2; radius=20; v=12; dt=0.01; p0=70;

S = { 1,[1.75,-p0,pi/2];   2,[5.25,-p0,pi/2];   3,[8.75,-p0,pi/2]; ...
      4,[p0,1.75,pi];      5,[p0,5.25,pi];      6,[p0,8.75,pi]; ...
      7,[-1.75,p0,3*pi/2]; 8,[-5.25,p0,3*pi/2]; 9,[-8.75,p0,3*pi/2]; ...
      10,[-p0,-1.75,0];    11,[-p0,-5.25,0];    12,[-p0,-8.75,0] };

fprintf('path | end (x,y)      | final heading | crosses ped-region(y~-32) | heading there\n');
for r = 1:12
    path = S{r,1}; st = S{r,2};
    x=st(1); y=st(2); th=st(3); XY=[]; TH=[];
    for k=1:8000
        nd = steer(path,x,y,radius,L);
        if abs(nd)<1e-9
            x=x+dt*cos(th)*v; y=y+dt*sin(th)*v;
        else
            om=tan(nd)/L;
            x=x+(1/om)*(sin(th+om*dt*v)-sin(th));
            y=y+(1/om)*(cos(th)-cos(th+om*dt*v));
            th=th+om*dt*v;
        end
        XY(end+1,:)=[x,y]; TH(end+1,1)=th; %#ok<AGROW>
        if hypot(x,y)>108, break; end
    end
    inreg = XY(:,2)>=-36 & XY(:,2)<=-28 & abs(XY(:,1))<=13;
    hd = NaN;
    if any(inreg), idx=find(inreg,1); hd = mod(rad2deg(TH(idx)),360); end
    fprintf(' %2d  | (%6.1f,%6.1f) |   %6.1f deg   |        %d              | %s\n', ...
        path, XY(end,1), XY(end,2), mod(rad2deg(th),360), any(inreg), ...
        ternary(any(inreg), sprintf('%.0f deg', hd), '--'));
end

function nd = steer(path,x,y,radius,L)
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
    if in, nd=atan(k*L); else, nd=0; end
end
function o=ternary(c,a,b), if c, o=a; else, o=b; end, end
