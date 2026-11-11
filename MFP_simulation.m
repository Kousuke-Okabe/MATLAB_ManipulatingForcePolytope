%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   操作力超多面体並進シミュレーション
%
%                                                       20.03.16 by.OKB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp(' ')
disp('Simulation of Translating Force Polytope')
clear all

type = '2d_RRR'

% set parameter
x = [0.25; 0.2; 0];

% dx = [0;-0.2; 0];
dx = [0; 0; 2];

% ddx = [0; 0; 0];
ddx = [0; 0.0; 0];

f = [ 3; 0; 0];

V0 = [x(1:2);0];
% V0 = zeros(3,1);

% Load parameter
param = get_parameter(type);
    Tlim = param.Tlim;

[q,dq,ddq] = fIKinematics_plus(type, x(1:2),dx(1:2),ddx(1:2), x(3),dx(3),ddx(3));
[J,Jp,U] = fJacobi(type, x);
[M,H,g] = get_matrix(type, q,dq);
T = M*ddq + H + g + [J;U']'*f

% disp('Norm is')
% norm(T)
% 
% disp('Tlim-T is')
% Tlim-abs(T)

% View Scale
% Gr = 1/50;
% Gz = 1/10;
Gr = 1;
Gz = 1;
Gx = eye(size(x,1));
    Gx(1,1) = Gr;
    Gx(2,2) = Gr;
    Gx(3,3) = Gz;

[Graph,V] = get_MFPgraph(type, x);
V = Gx*V;
Vt = get_MFPtranlate(type, x,dx,ddx,g)
llVtll = norm(Vt);
Vt = Gx*Vt;

for i = 1:size(V,2)
    V(:,i) = V(:,i) + Vt + V0;
end

% 描画
FH = 1;
dim = 2;
figure(FH)
clf(FH)
xlabel('fx [N]')
ylabel('fy [N]')
zlabel('fz [N]')

fRoboAnimation_Robo(FH,type,q)
fDraw_Graph3(FH,Graph,V,dim,'k');


% Translation Vector
quiver3(V0(1),V0(2),V0(3), Vt(1),Vt(2),Vt(3),'b', 'AutoScaleFactor',1)

% % Velocity Vector
% quiver3(V0(1),V0(2),V0(3), dx(1)/100,dx(2)/100,dx(3),'g', 'AutoScaleFactor',1)

% Force Vector
quiver3(V0(1),V0(2),V0(3), f(1)*Gr,f(2)*Gr,f(3)*Gz,'r', 'AutoScaleFactor',1)

if dim == 3 || dim == 23 || dim == 32
%     view([30,20])
    view([10,50])
    rotate3d on
elseif dim == 2
    view([0,90])
end

% xlim([-0.1, 0.3])
% ylim([-0.0, 0.4])
% set(gca, 'xtick',[0, 0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.4])

% xlim([-1,1]*15)
% ylim([-1,1]*15)
% zlim([-1,1]*1)
% set(gca, 'ztick',-1:0.5:1)

% xlim([-1,1]*0.25)
% ylim([-0.2,0.25])
% zlim([-1,1]*0.1)
% set(gca, 'ztick',-0.1:0.05:0.1)

box off
hold on
axis square

%% 描画 三面図
FH = 2;
figure(FH)
clf(FH)
hold on

for i = 1:2
for j = 1:2
    subplot(2,2,2*(i-1)+j)
    
    fDraw_Graph3(FH,Graph,V,3,'g');
    quiver3(V0(1),V0(2),V0(3), Vt(1),Vt(2),Vt(3),'b', 'AutoScaleFactor',1)

    set(gca, 'DataAspectRatio',[1,1,0.2])
    set(gca, 'XTick', [-10,-5,0,5,10])
    set(gca, 'YTick', [-10,-5,0,5,10])
    set(gca, 'ZTick', [-1,-0.5,0,0.5,1])

    xlim([-1, 1]*15)
    ylim([-1, 1]*15)
    zlim([-1, 1]*1)

    xlabel('fx [N]')
    ylabel('fy [N]')
    zlabel('fz ')

    if 2*(i-1)+j == 1
        view([0,90])
    elseif 2*(i-1)+j == 2
        view([30,30])
        box on
    elseif 2*(i-1)+j == 3
        view([0,0])
    else
        view([90,0])
    end
    
    rotate3d off
    axis square
end
end