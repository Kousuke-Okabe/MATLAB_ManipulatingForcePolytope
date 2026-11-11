%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   MFP並進ベクトル軌跡描画（変数：拡張速度）
%
%                                                       22.09.28 by.OKB
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

disp(' ')
disp('Simulatin of tranlation vector locus by velocity')
clear all

type = '2d_RRR'

% set parameter
x = [0.25; 0.2; 0];

N = 49;
a = 0.2;
dx1 = linspace(-a,a,N);
% dx1 = 0;

% a = 0.1;
% dx2 = linspace(0,-a,N);
dx2 = 0;

a = 2;
dx3 = linspace(0,a,N);

% ddx = [0; 0; 0];
ddx = [0; 0.0; 0];

f = [ 0; 0; 0];

% V0 = [x(1:2);0];
V0 = zeros(3,1);

% Load parameter
param = get_parameter(type);
    Tlim = param.Tlim;

% View Scale
Gr = 1/1;
Gz = 1/1;
% Gr = 1;
% Gz = 1;
Gx = eye(size(x,1));
    Gx(1,1) = Gr;
    Gx(2,2) = Gr;
    Gx(3,3) = Gz;

Vt = nan(3,N,N);
for i = 1:N
    for j = 1:N
        Vt(:,i,j) = get_MFPtranlate(type, x,[dx1(i);dx2;dx3(j)],ddx,zeros(3,1));
        Vt(:,i,j) = Gx*Vt(:,i,j);
    end
end

% 描画
FH = 2;
dim = 2;
figure(FH)
clf(FH)
hold on
box on
grid off
view([-15,20])
xlabel('fx [N]')
ylabel('fy [N]')
zlabel('fz [N]')

% PlotNumber = 5;
% plotdata = nan(3,N);
% for i = ceil( fix( linspace(-N/2,N/2,PlotNumber) ) + N/2 )
%     for j = 1:N
%         plotdata(:,j) = Vt(:,i,j);
%     end
%     plot3(plotdata(1,:),plotdata(2,:),plotdata(3,:),'b')
%     
% %     for k = [1,N]
% %         text(plotdata(1,k),plotdata(2,k),plotdata(3,k), strcat('(',num2str(dx1(i),'%.1f'),', 0, ',num2str(dx3(k),'%.1f'),')'), 'FontSize',12)
% %     end
% end

PlotNumber = 3;
plotdata = nan(3,N);
for j = ceil( fix( linspace(-N/2,N/2,PlotNumber) ) + N/2 )
    for i = 1:N
        plotdata(:,i) = Vt(:,i,j);
    end
    plot3(plotdata(1,:),plotdata(2,:),plotdata(3,:),'r')
    
%     for k = [1,N]
%         text(plotdata(1,k),plotdata(2,k),plotdata(3,k), strcat('(',num2str(dx1(k),'%.1f'),', 0, ',num2str(dx3(j),'%.1f'),')'), 'FontSize',12)
%     end
end