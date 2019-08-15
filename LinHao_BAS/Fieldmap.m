function f = Fieldmap(map,goal)
%% ��ͼ������ ���ڱ����ͼ��0�����ϰ����������ʱ�����ȡ��    �����ת����Щ���ܻ������⡣
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
% ��Ӧ��xgrid��goal�����Ҫ��һ��
attractive = xi * ( (xgrid - goal(2)).^2 + (ygrid - goal(1)).^2 );
% attractive = xi * ( (xgrid - goal(2)).^2 + (ygrid - goal(1)).^2 );
% attractive = attractive';
% figure;m = mesh (attractive);m.FaceLighting = 'phong';axis equal;
%������
f = attractive + repulsive;
figure;m = mesh (f);m.FaceLighting = 'phong';axis equal;title ('Total Potential');
end