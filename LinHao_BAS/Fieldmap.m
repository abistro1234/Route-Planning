function f = Fieldmap(map,goal)
%% 地图建立场 由于本身地图是0代表障碍物，建立场的时候进行取反    具体的转置这些可能还有问题。
I = find(map==2);
map(I)=0;
[ncols,nrows]=size(map);
% [xgrid, ygrid] = meshgrid (1:ncols, 1:nrows);
[xgrid, ygrid] = meshgrid (1:nrows, 1:ncols);
d = bwdist(~map);
d2 = (d/100) + 1;

d0 = 2;
nu = 800;

repulsive = nu*((1./d2 - 1/d0).^2);

repulsive (d2 > d0) = 0;

% figure;m = mesh (repulsive);m.FaceLighting = 'phong';axis equal;
% Compute attractive force

xi = 1/1400;
% 对应的xgrid和goal，因此要换一下
attractive = xi * ( (xgrid - goal(2)).^2 + (ygrid - goal(1)).^2 );
% attractive = xi * ( (xgrid - goal(2)).^2 + (ygrid - goal(1)).^2 );
% attractive = attractive';
% figure;m = mesh (attractive);m.FaceLighting = 'phong';axis equal;
%合立场
f = attractive + repulsive;
figure;m = mesh (f);m.FaceLighting = 'phong';axis equal;title ('Total Potential');
end