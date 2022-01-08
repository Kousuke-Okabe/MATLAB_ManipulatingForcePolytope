%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   ëÄçÏóÕëΩñ ëÃÇ∆ìÆìIâ¬ëÄçÏê´ëΩñ ëÃÇÃî‰är
%
%                                                       21.09.25 by.OKB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp(' ')
disp('Simulation of Translating Force Polytope')
clear all

type = '2d_RRR'

% set parameter
x = [0.2; 0.2; pi/3];
dx = [0; 0; 0];
ddx = [0; 0; 0];
f = [ 0; 0; 0];
g = [0; 0; 0];

V0 = [x(1:2);0];

% Load parameter
param = get_parameter(type);
    Tlim = param.Tlim;

[q,dq,ddq] = fIKinematics_plus(type, x(1:2),dx(1:2),ddx(1:2), x(3),dx(3),ddx(3));
[J,Jp,U] = fJacobi(type, x);

% View Scale
Gt = 1*10^-3;
Gz = 5*10^-5;
G_DMP = diag([Gt, Gt, Gz]);
Gt = 2*10^-2;
Gz = 5*10^-1;
G_MFP = diag([Gt,Gt,Gz]);

% MFP
[Graph,V_MFP] = get_MFPgraph(type, x);
V_MFP = G_MFP*V_MFP;
Vt_MFP = get_MFPtranlate(type, x,dx,ddx,g);
Vt_MFP = G_MFP*Vt_MFP;

for i = 1:size(V_MFP,2)
    V_MFP(:,i) = V_MFP(:,i) + Vt_MFP + V0;
end

% DMP
[Graph_DMP,V_DMP] = get_DMPgraph(type, x);
V_DMP = G_DMP*V_DMP;
Vt_DMP = get_DMPtranlate(type, x,dx,g,f);
Vt_DMP = G_DMP*Vt_DMP;

for i = 1:size(V_DMP,2)
    V_DMP(:,i) = V_DMP(:,i) + Vt_DMP + V0;
end

% ï`âÊ
FH = 1;
figure(FH)
clf(FH)
hold on

fRoboAnimation_Robo(FH,type,q,x(1:2))

% MFP
dimension = 3;
fDraw_Graph3(FH,Graph,V_MFP,dimension, 'g');
% DMP
fDraw_Graph3(FH,Graph_DMP,V_DMP,dimension, 'm');


% Translation Vector
quiver3(V0(1),V0(2),V0(3), Vt_MFP(1),Vt_MFP(2),Vt_MFP(3),'b', 'AutoScaleFactor',1)
quiver3(V0(1),V0(2),V0(3), Vt_DMP(1),Vt_DMP(2),Vt_DMP(3),'g', 'AutoScaleFactor',1)


set(gca, 'xtick',[0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4])

% area = axis;
% plot3(area(1:2),zeros(2,1),zeros(2,1),'k')
% plot3(zeros(2,1),area(3:4),zeros(2,1),'k')

xlim([-0.0, 0.4])
ylim([-0.0, 0.4])
zlim([-0.2, 0.2])

xlabel('fx [N]')
ylabel('fy [N]')
zlabel('fz [N]')

view([20,30])
% view([0,90])
rotate3d on

box off
hold on


%% ï`âÊ
FH = 1;
figure(FH)
clf(FH)
hold on

for i = 1:2
for j = 1:2
    subplot(2,2,2*(i-1)+j)
    
    fDraw_Graph3(FH,Graph,V_MFP,3);
    surf(Sx,Sy,Sz, 'FaceAlpha',0.8, 'FaceColor','w')

    set(gca, 'DataAspectRatio',[1,1,0.2])
    set(gca, 'XTick', [-10,-5,0,5,10])
    set(gca, 'YTick', [-10,-5,0,5,10])
    set(gca, 'ZTick', [-1,-0.5,0,0.5,1])

    xlim([-1, 1]*10)
    ylim([-1, 1]*10)
    zlim([-1, 1]*1)

    xlabel('fx [N]')
    ylabel('fy [N]')
    zlabel('fz ')

    if 2*(i-1)+j == 1
        view([0,90])
    elseif 2*(i-1)+j == 2
        view([30,30])
    elseif 2*(i-1)+j == 3
        view([0,0])
    else
        view([90,0])
    end
    
    rotate3d on
end
end