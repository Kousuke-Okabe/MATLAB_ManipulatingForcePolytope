%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   ëÄçÏóÕëΩñ ëÃÇ∆ìÆìIâ¬ëÄçÏê´ëΩñ ëÃÇÃî‰är
%
%                                                       21.09.25 by.OKB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp(' ')
disp('Simulation of Translating Force Polytope')
clear all

type = '2d_RRR';

% set parameter
x = [0.25; 0.2; 0];

dx = [1, 0, 1, 0, 1, 0, 1, 0;
      1, 1, 0, 0, 1, 1, 0, 0;
      1, 1, 1, 1, 0, 0, 0, 0]; 

ddx = [0; 0; 0];
f = [ 0; 0; 0];
g = [0; 0; 0];

V0 = [x(1:2);0];

Vt = nan(3,2^3);
Vt_DMP = nan(3,2^3);

% Load parameter
type
param = get_parameter(type);
    Tlim = param.Tlim;

[q] = fIKinematics(type, x(1:2),dx(1:2));
% [J,Jp,U] = fJacobi(type, x(1:2),x(3));

% View Scale
G_MFP = diag([1/50,1/50,1/10]);
G_DMP = diag([3*10^-4, 3*10^-4, 5*10^-6]);

% ï`âÊ
FH = 1;
figure(FH)
clf(FH)
hold on

for i = 1:2^3
    % MFP
%     [Graph,V] = get_MFPgraph(type, x);
%     V = G_MFP*V;
    Vt(:,i) = get_MFPtranlate(type, x,dx(:,i),ddx,g);
    Vt(:,i) = G_MFP*Vt(:,i);

%     for i = 1:size(V,2)
%         V(:,i) = V(:,i) + Vt + V0;
%     end

    % DMP
%     [Graph_DMP,V_DMP] = get_DMPgraph(type, x);
%     V_DMP = G_DMP*V_DMP;
    Vt_DMP(:,i) = get_DMPtranlate(type, x,dx(:,i),g,f);
    Vt_DMP(:,i) = G_DMP*Vt_DMP(:,i);

%     for i = 1:size(V_DMP,2)
%         V_DMP(:,i) = V_DMP(:,i) + Vt_DMP + V0;
%     end

    % Translation Vector
    quiver3(V0(1),V0(2),V0(3), Vt(1,i),Vt(2,i),Vt(3,i),'b', 'AutoScaleFactor',1)
    quiver3(V0(1),V0(2),V0(3), Vt_DMP(1,i),Vt_DMP(2,i),Vt_DMP(3,i),'m', 'AutoScaleFactor',1)

end

% area = axis;
% plot3(area(1:2),zeros(2,1),zeros(2,1),'k')
% plot3(zeros(2,1),area(3:4),zeros(2,1),'k')


fRoboAnimation_Robo(FH,type,q,x(1:2))

% xlim([-0.0, 0.4])
% ylim([-0.0, 0.4])
% zlim([-0.1, 0.1])
set(gca, 'xtick',[0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4])

xlabel('fx [N]')
ylabel('fy [N]')
zlabel('fz [N]')

view([20,30])
rotate3d on

box off
grid off

