%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   MFP‚ÌMFE‚Æ‚Ì”äŠr
%
%                                                       21.09.22 by.OKB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp(' ')
disp('Comparation with Ellipsoid and Polytope')
clear all

type = '2d_RRR';

% set parameter
x = [0.25; 0.2; 0];
dx = [0; 0; 0];
ddx = [0; 0; 0];
f = [ 3; 0; 0];
g = [0;0;0];

V0 = [x(1:2);0];

% Load parameter
type
param = get_parameter(type);
    Tlim = param.Tlim;
    W = diag(Tlim);

[q,dq,ddq] = fIKinematics_plus(type, x(1:2),dx(1:2),ddx(1:2), x(3),dx(3),ddx(3));
[J,Jp,U] = fJacobi(type, x);
Jei = [Jp,U];

% View Scale
Gr = 1/1;
Gz = 1/1;
Gx = eye(size(x,1));
    Gx(1,1) = Gr;
    Gx(2,2) = Gr;
    Gx(3,3) = Gz;

[Graph,V] = get_MFPgraph(type, x);
V = Gx*V;
Vt = get_MFPtranlate(type, x,dx,ddx, g);
Vt = Gx*Vt;

% MFE
[Sx, Sy, Sz] = sphere(10);
for i = 1:size(Sx,1)
    for j = 1:size(Sx,2)
        temp = Jei'*W*[Sx(i,j);Sy(i,j);Sz(i,j)];
        temp = Gx*temp + x;
        Sx(i,j) = temp(1);
        Sy(i,j) = temp(2);
        Sz(i,j) = temp(3);
    end
end

% •`‰æ
FH = 1;
figure(FH)
clf(FH)
hold on

for i = 1:2
for j = 1:2
    subplot(2,2,2*(i-1)+j)
    
    fDraw_Graph3(FH,Graph,V,3);
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