function[G,V] = get_MFPgraph(type, x)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%   MFPの頂点グラフをReturnする関数
%
%   [G,V] = get_MFPgraph(x)
%       G : グラフ
%       V : 頂点データ
%       x : 作業空間上の位置
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% %%
% clear all
% 
% x = [0.25;0.2;0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 引数の再配置
r = x(1:2);
Rx = x(3);

% 変数宣言
param = get_parameter(type);
    Link = param.Link;
    Tlim = param.Tlim;

% 頂点データ作成
V = nan(Link,2^Link);
for i = 1:Link
    for j = 1:size(V,2)
        if j-floor((j-1)/(2^(Link+1)/2^i))*(2^(Link+1)/2^i) <= 2^Link/2^i
            V(i,j) = Tlim(i);
        else
            V(i,j) = -Tlim(i);
        end
    end
end

% グラフ作成
G = zeros(2^Link);    % グラフ

for i = 1:size(G,2)
    for j = i+1:size(G,2)
        temp = 0;
        for k = 1:Link
            if V(k,i)*V(k,j) < 0
                temp = temp+1;
            end
        end
        
        if temp == 1
            G(i,j) = 1;
            G(j,i) = 1;
        end
    end
end

% MFP頂点作成
q = fIKinematics(type, x);
[M,g] = get_matrix_minus(type, q);

for i = 1:size(V,2)
    V(:,i) = V(:,i)-g;
end

[J,Jp,U] = fJacobi(type, x);
JEiT = [Jp,U]';
V = JEiT*V;


%         FH = 1;
%         clf(FH)
%         fDraw_MFP3(FH,G,V)
