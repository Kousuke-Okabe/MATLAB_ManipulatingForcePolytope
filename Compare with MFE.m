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
Je = [J;U'];
Jei = [Jp,U];

% View Scale
Gr = 2*10^-2;
Gz = 1*10^-1;
Gx = eye(size(x,1));
    Gx(1,1) = Gr;
    Gx(2,2) = Gr;
    Gx(3,3) = Gz;

[Graph,V] = get_MFPgraph(type, x);
V = Gx*V;
Vt = get_MFPtranlate(type, x,dx,ddx, g);
Vt = Gx*Vt;
V = V + diag(Vt)*ones(size(V)) + diag(x)*ones(size(V));

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
FH = 2;
dim = 2;
figure(FH)
clf(FH)
hold on

fRoboAnimation_Figure(FH);
fRoboAnimation_Robo(FH,type,q);

% MFP
fDraw_Graph3(FH,Graph,V,dim, 'g')
% fDraw_Graph3(FH,Graph,V,2, 'g')

% MFE
if dim == 3
    surf(Sx,Sy,Sz, 'FaceAlpha',0, 'FaceColor','w')
end

MFE = nan(2,100);
theta = linspace(0,2*pi,size(MFE,2));
Fmax = 10;
Fnorm = Fmax/2*ones(1,size(MFE,2));
for i = 1:size(MFE,2)
    for k = 1:20
        Fcheck = [ Fnorm(1,i)*cos(theta(i));
                   Fnorm(1,i)*sin(theta(i));
                   0];
        if Fcheck'*Je*(diag(Tlim))^-2*Je'*Fcheck < 1
            Fnorm(1,i) = Fnorm(1,i) + Fmax/2^(k+1);
        else
            Fnorm(1,i) = Fnorm(1,i) - Fmax/2^(k+1);
        end
    end
    MFE(:,i) = [ Fnorm(1,i)*cos(theta(i));
                 Fnorm(1,i)*sin(theta(i))];
end
MFE = Gx(1:2,1:2)*MFE + diag(x(1:2))*ones(2,size(MFE,2));
plot(MFE(1,:),MFE(2,:),'r')

xlim([-0, 0.4])
ylim([-0, 0.35])
view([0,90])
set(gca, 'XTick',0:0.05:0.4)
set(gca, 'YTick',0:0.1:0.4)


%% •`‰æ ŽO–Ê}
FH = 1;
figure(FH)
clf(FH)
hold on

for i = 1:2
for j = 1:2
    subplot(2,2,2*(i-1)+j)
    
    fDraw_Graph3(FH,Graph,V,3,'g');
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
        box on
    elseif 2*(i-1)+j == 3
        view([0,0])
    else
        view([90,0])
    end
    
    rotate3d on
end
end