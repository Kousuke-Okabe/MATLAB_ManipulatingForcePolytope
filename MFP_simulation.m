%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   操作力超多面体並進シミュレーション
%
%                                                       20.03.16 by.OKB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp(' ')
disp('Simulation of Translating Force Polytope')
clear all

type = '2d_RRR';

% set parameter
x = [0.25; 0.2; 0];
dx = [0; -2.5; 0];
ddx = [0; 0; 0];
f = [ 3; 0; 0];

V0 = [x(1:2);0];

% Load parameter
type
param = get_parameter(type);
    Tlim = param.Tlim;

[q,dq,ddq] = fIKinematics_plus(type, x(1:2),dx(1:2),ddx(1:2), x(3),dx(3),ddx(3));
[J,Jp,U] = fJacobi(type, x(1:2),x(3));
[M,H,G] = get_matrix(type, q,dq);
T = M*ddq + H + G + [J;U']'*f

disp('Norm is')
norm(T)

        disp('Tlim-T is')
        Tlim-abs(T)

% View Scale
Gr = 1/50;
Gz = 1/10;
Gx = eye(size(x,1));
    Gx(1,1) = Gr;
    Gx(2,2) = Gr;
    Gx(3,3) = Gz;

[Graph,V] = get_MFPgraph(type, x);
V = Gx*V;
Vt = get_MFPtranlate(type, x,dx,ddx)
Vt = Gx*Vt;

for i = 1:size(V,2)
    V(:,i) = V(:,i) + Vt + V0;
end

% 描画
FH = 1;
figure(FH)
clf(FH)
    xlabel('fx [N]')
    ylabel('fy [N]')
    zlabel('fz [N]')
    
    view([20,30])
    rotate3d on
    
%     xlim([0,1]*max(max(abs(V))))
%     ylim([0,1]*max(max(abs(V))))
%     zlim([-1,1]*max(max(abs(V)))/2)
    box off
    hold on

fRoboAnimation_Robo(FH,type,q,x(1:2))
% plot3(0,0,0,'k+')
fDraw_Graph3(FH,Graph,V,3);


% Translation Vector
quiver3(V0(1),V0(2),V0(3), Vt(1),Vt(2),Vt(3),'b', 'AutoScaleFactor',1)

% % Velocity Vector
% quiver3(V0(1),V0(2),V0(3), dx(1)/100,dx(2)/100,dx(3),'g', 'AutoScaleFactor',1)

% Force Vector
% quiver3(V0(1),V0(2),V0(3), f(1)*Gr,f(2)*Gr,f(3)*Gz,'r', 'AutoScaleFactor',1)


xlim([-0.0, 0.4])
ylim([-0.0, 0.4])
zlim([-0.1, 0.1])
set(gca, 'xtick',[0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4])

% area = axis;
% plot3(area(1:2),zeros(2,1),zeros(2,1),'k')
% plot3(zeros(2,1),area(3:4),zeros(2,1),'k')

% A=1.198242188;
% quiver3(V0(1),V0(2),0, 3*Gr,0*Gr,0*Gz,'r', 'AutoScaleFactor',1)
% quiver3(V0(1),V0(2),0, 3*Gr,-A*Gr,0*Gz,'r', 'AutoScaleFactor',1)
% quiver3(V0(1),V0(2),0, 3*Gr,-3*Gr,0*Gz,'r', 'AutoScaleFactor',1)
