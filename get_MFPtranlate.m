function[Vt] = get_MFPtranlate(type, x,dx,ddx, g)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   MFPの並進ベクトル
%
%   [Vt] = get_MFPtranlate(type, x,dx,ddx, g)
%       x   : 拡張作業空間上の位置
%       dx  : 拡張作業空間上の速度
%       ddx : 拡張作業空間上の加速度
%       g   : 重力ベクトル
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %%
% clear all
% 
% x = [0.25; 0.1; 0]
% dx = [0; 2; 0]
% ddx = [0;0;0]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

r = x(1:2);
Rx = x(3);
dr = dx(1:2);
dz = dx(3);

% % test
% disp('test')
% [J,Jp,dJ,U] = fJacobi_plus(r,dr, Rx,dz);
% 
% q = fIKinematics(r,Rx);
% [J,Jp,Jd,U,Ud,Jed] = fJacobi_tensor(r,dr, Rx,dz);
% dq = Jp*dr + U*dz;
% dJe = zeros(3,3);
% for i = 1:3
% for j = 1:3
% for k = 1:3
%     dJe(i,j) = dJe(i,j) + Jed(i,j,k)*dq(k);
% end
% end
% end
% %%

% 変数宣言
param = get_parameter(type);
    Link = param.Link;

Vt = zeros(size(dx));

[J,Jp,Jd,U,Ud,Jed] = fJacobi_tensor(type, x);
Jei = [Jp,U];

q = fIKinematics(type, x);
dq = Jei*dx;
[M,H,G,Hd] = get_matrix_plus(type, q,dq);

for i = 1:Link
for j = 1:Link
for k = 1:Link
for l = 1:Link
    Vt(i) = Vt(i) - Jei(j,i)*M(j,k)*Jei(k,l)*ddx(l);
end
end
end
end

for i = 1:Link
for j = 1:Link
    Vt(i) = Vt(i) - Jei(j,i)*g(j);
end
end

for i = 1:Link
for j = 1:Link
for k = 1:Link
for l = 1:Link
for m = 1:Link
for n = 1:Link
for o = 1:Link
for p = 1:Link
    Vt(i) = Vt(i) + Jei(j,i)*M(j,k)*Jei(k,l)*Jed(l,m,n)*Jei(m,o)*Jei(n,p)*dx(o)*dx(p);
end
end
end
end
end
end
end
end

for i = 1:Link
for j = 1:Link
for k = 1:Link
for l = 1:Link
for m = 1:Link
for n = 1:Link
    Vt(i) = Vt(i) - Jei(j,i)*Hd(j,k,l)*Jei(k,m)*Jei(l,n)*dx(m)*dx(n);
end
end
end
end
end
end


