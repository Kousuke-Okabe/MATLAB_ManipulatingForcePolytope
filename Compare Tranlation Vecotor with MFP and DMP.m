%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   操作力多面体と動的可操作性多面体の並進ベクトルの比較
%
%                                                       21.09.25 by.OKB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp(' ')
disp('Simulation of Translating Force Polytope')
clear all

type = '2d_RRR';

% set parameter
% x = [0.25; 0.2; 0];
x = [rand([2,1])*0.2; rand(1)*pi];
%%
dx = [0; 0; 0];
dx = [0; 0.2; 0];

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

[J,Jp,U] = fJacobi(type, x);
Je = [J;U'];
Jei = [Jp, U];

[q] = fIKinematics(type, x);
[M,G] = get_matrix_minus(type,q);

% View Scale
Gt = 1;
Gz = Gt;
G_DMP = diag([Gt, Gt, Gz]);
G_MFP = G_DMP;

% 描画
FH = 2;
dim = 23;
figure(FH)
clf(FH)


fRoboAnimation_Robo(FH,type,q)

% MFP
Vt = get_MFPtranlate(type, x,dx,ddx,g);
Vt = G_MFP*Vt/abs(det(Jei));
quiver3(V0(1),V0(2),V0(3), Vt(1),Vt(2),Vt(3),'g', 'AutoScaleFactor',1)

[Graph,V] = get_MFPgraph(type, x);
V = G_MFP*V/abs(det(Jei)) + diag(Vt)*ones(size(V)) + diag(V0)*ones(size(V));
fDraw_Graph3(FH,Graph,V,dim,'b')


% DMP
Vt_DMP = get_DMPtranlate(type, x,dx,g,f);
Vt_DMP = G_DMP*Vt_DMP/abs(det(Je/M));
quiver3(V0(1),V0(2),V0(3), Vt_DMP(1),Vt_DMP(2),Vt_DMP(3),'r', 'AutoScaleFactor',1)

[Graph_DMP,V_DMP] = get_DMPgraph(type, x);
V_DMP = G_DMP*V_DMP/abs(det(Je/M)) + diag(Vt_DMP)*ones(size(V_DMP)) + diag(V0)*ones(size(V));
fDraw_Graph3(FH,Graph_DMP,V_DMP,dim,'m')


% Draw setting
xlabel('fx [N]')
ylabel('fy [N]')
zlabel('fz [N]')

% xlim([0,1]*0.35)
% ylim([0,1]*0.35)
view([0,90])
% set(gca, 'xtick', 0:0.05:0.35)
% set(gca, 'ytick', 0:0.05:0.35)

hold on
axis square
rotate3d on
box off
grid off