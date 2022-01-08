function[Vt] = get_DMPtranlate(type, x,dx,g,f)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   MFP�̕��i�x�N�g��
%
%   [Vt] = get_DMPtranlate(type, x,dx,g,f)
%       x   : �g����Ƌ�ԏ�̈ʒu
%       dx  : �g����Ƌ�ԏ�̑��x
%       g   : �d�̓x�N�g��
%       f   : �g����Ƌ�ԏ�̗�
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %%
% clear all
% 
% x = [0.1;0.15;0];
% dx = [5;5;1];
% ddx = [1;1;0.1];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

r = x(1:2);
Rx = x(3);
dr = dx(1:2);
dz = dx(3);

% �ϐ��錾
param = get_parameter(type);
    Link = param.Link;

Vt = zeros(size(dx));


[J,Jp,Jd,U,Ud,Jed] = fJacobi_tensor(type, x);
Je = [J;U'];
Jei = [Jp,U];

q = fIKinematics(type, x);
dq = Jei*dx;
[M,H,G,Hd] = get_matrix_plus(type,q,dq);
Mi = inv(M);

for i = 1:Link
for j = 1:Link
for k = 1:Link
for n = 1:Link
for o = 1:Link
    Vt(i) = Vt(i) + Jed(i,j,k)*Jei(j,n)*Jei(k,o)*dx(n)*dx(o);
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
for o = 1:Link
    Vt(i) = Vt(i) - Je(i,j)*Mi(j,k)*Hd(k,l,m)*Jei(l,n)*Jei(m,o)*dx(n)*dx(o);
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
    Vt(i) = Vt(i) - Je(i,j)*Mi(j,k)*g(k);
end
end
end

for i = 1:Link
for j = 1:Link
for k = 1:Link
for l = 1:Link
    Vt(i) = Vt(i) - Je(i,j)*Mi(j,k)*Je(l,k)*f(l);
end
end
end
end