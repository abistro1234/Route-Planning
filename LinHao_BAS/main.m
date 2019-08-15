% function [path,smoothPath,time,pathLength,spathLength] = main(start,goal,mapname)
clearvars -except start goal mapname;
close all;
isdynamic = true;% if dynamic = true obstacle move
isdynamicdisplay = false;% if dynamic = true obstacle move
display = false; % display of path plan

[mapname,start,goal]=initpar; % 批量测试时候参数从loop函数中传递
safedist = 5;
map=im2bw(imread(mapname)); % input map read from a bmp file. for new maps write the file name here
% map = map';
[ncols,nrows]=size(map);
%example:map=500*100 logical ncols = 500,nrows = 100
%newmap = safemap(map,safedist);
step = 10;  % size of each step of the BAS
threshold = 15; % nodes closer than this threshold are taken as almost the same
maxFailedAttempts = 50000;
figure;imshow(map.');axis ([0 ncols 0 nrows]);
% axis xy;
axis on;xlabel ('x');ylabel ('y');
hold on;
plot (start(1), start(2), 'b.', 'MarkerSize', 25);
plot (goal(1), goal(2), 'r.', 'MarkerSize', 25);
%% 
tic;
[path,iteration,map] = BAS(map,start,goal,step,maxFailedAttempts,display,isdynamic,isdynamicdisplay);
% 写个判定。如果平滑后的路径点位于障碍物内，使用原本的对应路径点进行替换
smoothPath = Spline_Smooth(path(:,1), path(:,2), round(length(path(:,1))/4),3);
smoothPath = smoothPath';
toc;
time = toc;
pathLength = 0;
for i=1:length(path)-1, pathLength = pathLength + distanceCost(path(i,1:2),path(i+1,1:2)); end % calculate path length
spathLength = 0;
for i=1:length(smoothPath)-1, spathLength = spathLength + distanceCost(smoothPath(i,1:2),smoothPath(i+1,1:2)); end % calculate path length
fprintf('processing time=%d \nPath Length=%d \nSmooth Path Length=%d \n', time, pathLength,spathLength); 
figure;
imshow(map.');
axis ([0 ncols 0 nrows]);
% axis xy;
axis on;
% imshow(~obstacle);
hold on;
plot (start(1), start(2), 'b.', 'MarkerSize', 25);
plot (goal(1), goal(2), 'r.', 'MarkerSize', 25);
% 平滑曲线
a = path(:,1)';  %横坐标
b = path(:,2)'; %纵坐标
plot(a,b, 'r-o');
a1 = smoothPath(:,1);  %横坐标
b1 = smoothPath(:,2); %纵坐标
plot(a1,b1, 'b-o');
title ('final path');
print('-dbmp', 'finalPath');
% end